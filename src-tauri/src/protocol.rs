//! IM6014 wire protocol, verified against Unitree SDK commit
//! 0803d033b530fcfa0e6fa9236de68c2773fc9402, MotorProtocol.cpp / arm_motor_bus.cpp.
//! One owner, seven-joint command batches. No action retries or cached feedback.

use crate::{
    calibration_path::PlaybackPhase,
    journal::{Journal, TraceData},
    reference::{Previous, Reference, SampleStatus, Sampling, Timing},
    trace::ServoSample,
};
use crossbeam_channel::{Receiver as EventReceiver, Sender, bounded, unbounded};
use serde::{Deserialize, Serialize};
use serialport::{
    ClearBuffer, DataBits, FlowControl, Parity, SerialPort, SerialPortType, StopBits,
};
use std::collections::BTreeMap;
use std::io;
use std::sync::{
    Arc, Mutex, OnceLock,
    atomic::{AtomicBool, AtomicU64, Ordering},
};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};
use tokio::sync::watch;

const COMMAND_LEN: usize = 20;
const FEEDBACK_LEN: usize = 26;
const IO_SLICE: Duration = Duration::from_millis(1);
const WRITE_PERIOD: Duration = Duration::from_millis(10);
const PI: f64 = std::f64::consts::PI;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct PortInfo {
    pub name: String,
    pub label: String,
    pub kind: String,
}

pub fn enumerate_ports() -> Vec<PortInfo> {
    let ports = serialport::available_ports()
        .unwrap_or_default()
        .into_iter()
        .map(|p| {
            let (kind, detail) = match p.port_type {
                SerialPortType::UsbPort(info) => {
                    let description = [info.manufacturer, info.product]
                        .into_iter()
                        .flatten()
                        .collect::<Vec<_>>()
                        .join(" ");
                    (
                        "usb",
                        format!("{description} [{:04x}:{:04x}]", info.vid, info.pid),
                    )
                }
                SerialPortType::BluetoothPort => ("bluetooth", "Bluetooth".into()),
                SerialPortType::PciPort => ("pci", "PCI".into()),
                SerialPortType::Unknown => ("unknown", String::new()),
            };
            let label = if detail.is_empty() {
                p.port_name.clone()
            } else {
                format!("{} — {}", p.port_name, detail.trim())
            };
            PortInfo {
                name: p.port_name,
                label,
                kind: kind.into(),
            }
        })
        .collect();
    dedupe_ports(ports)
}

fn port_key(name: &str) -> &str {
    #[cfg(target_os = "macos")]
    {
        if let Some(suffix) = name.strip_prefix("/dev/cu.") {
            return suffix;
        }
        if let Some(suffix) = name.strip_prefix("/dev/tty.") {
            return suffix;
        }
    }
    name
}

fn prefer_candidate(candidate: &PortInfo, existing: &PortInfo) -> bool {
    #[cfg(target_os = "macos")]
    {
        let candidate_cu = candidate.name.starts_with("/dev/cu.");
        let existing_cu = existing.name.starts_with("/dev/cu.");
        if candidate_cu != existing_cu {
            return candidate_cu;
        }
    }
    (existing.kind == "unknown" && candidate.kind != "unknown")
        || (existing.label == existing.name && candidate.label != candidate.name)
}

fn dedupe_ports(ports: Vec<PortInfo>) -> Vec<PortInfo> {
    let mut unique: BTreeMap<String, PortInfo> = BTreeMap::new();
    for candidate in ports {
        let key = port_key(&candidate.name).to_string();
        match unique.get_mut(&key) {
            Some(existing) if prefer_candidate(&candidate, existing) => {
                let mut merged = candidate;
                if merged.kind == "unknown" && existing.kind != "unknown" {
                    merged.kind = existing.kind.clone();
                    if let Some(detail) = existing.label.strip_prefix(&existing.name) {
                        merged.label = format!("{}{}", merged.name, detail);
                    }
                }
                *existing = merged;
            }
            Some(_) => {}
            None => {
                unique.insert(key, candidate);
            }
        }
    }
    let mut ports: Vec<_> = unique.into_values().collect();
    ports.sort_by(|a, b| a.name.cmp(&b.name));
    ports
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DriveMode {
    Disabled,
    Servo,
    ClearFault,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Setpoint {
    pub mode: DriveMode,
    pub q: f64,
    pub dq: f64,
    pub tau: f64,
    pub kp: f64,
    pub kd: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Feedback {
    pub id: u8,
    pub q: f64,
    pub dq: f64,
    pub tau: f64,
    /// Driver temperature, signed degrees Celsius (byte 3).
    pub temperature: f64,
    /// Winding temperature, unsigned degrees Celsius (byte 4).
    pub winding_temperature: f64,
    pub voltage: f64,
    pub error: u32,
    pub warning: u8,
    /// Actual device mode, 0..7, independent of the requested mode.
    pub mode: u8,
    /// Device-reported watchdog state, separate from host transaction timeouts.
    pub timeout: bool,
}

/// Decode the IM6014 MError bitmask (SDK MotorProtocol.cpp, lines 421–466).
pub fn motor_fault_description(code: u32) -> String {
    static FAULTS: OnceLock<BTreeMap<u32, &'static str>> = OnceLock::new();
    if code == 0 {
        return "正常".into();
    }
    let mut unknown = code;
    let mut descriptions = Vec::new();
    let faults = FAULTS.get_or_init(|| {
        serde_json::from_str(include_str!("../../resources/motor-faults.json"))
            .expect("invalid embedded motor fault descriptions")
    });
    for (&mask, &description) in faults {
        if code == mask {
            return description.into();
        }
        if code & mask != 0 {
            descriptions.push(description);
            unknown &= !mask;
        }
    }
    let known = descriptions.join("、");
    if known.is_empty() {
        format!("未知故障位 0x{code:08X}")
    } else if unknown == 0 {
        format!("{known}（0x{code:08X}）")
    } else {
        format!("{known}、未知故障位 0x{unknown:08X}（0x{code:08X}）")
    }
}

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct BusStats {
    /// All CRC-valid frames received during exchanges, including other IDs.
    pub received: u64,
    pub crc_errors: u64,
    /// Packet progress deadlines (or test feedback deadlines) that expired.
    pub timeouts: u64,
    pub transmit: TransmitStats,
}

/// Host-side batch timing, not USB wire timing or an actuator acknowledgement.
#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct TransmitStats {
    pub batches: u64,
    pub intervals: u64,
    pub interval_sum_ms: f64,
    pub interval_min_ms: f64,
    pub interval_max_ms: f64,
    pub write_max_ms: f64,
}

impl TransmitStats {
    fn record(&mut self, previous: Option<Instant>, started: Instant, finished: Instant) {
        self.batches += 1;
        self.write_max_ms = self
            .write_max_ms
            .max(finished.saturating_duration_since(started).as_secs_f64() * 1000.0);
        if let Some(previous) = previous {
            let ms = started.saturating_duration_since(previous).as_secs_f64() * 1000.0;
            self.interval_min_ms = if self.intervals == 0 {
                ms
            } else {
                self.interval_min_ms.min(ms)
            };
            self.intervals += 1;
            self.interval_sum_ms += ms;
            self.interval_max_ms = self.interval_max_ms.max(ms);
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct Calibration {
    gain: f32,
    offset: f32,
}

fn calibrations(side: &str) -> Result<[Calibration; 7], String> {
    let source = match side {
        "right" => include_str!("../../resources/calibration/joint_calib_right.txt"),
        "left" => include_str!("../../resources/calibration/joint_calib_left.txt"),
        _ => return Err("side must be right or left".into()),
    };
    parse_calibration(source)
}

fn parse_calibration(source: &str) -> Result<[Calibration; 7], String> {
    let mut values = [Calibration {
        gain: 1.0,
        offset: 0.0,
    }; 7];
    let mut seen = [false; 7];
    for line in source.lines() {
        let fields: Vec<_> = line
            .split('#')
            .next()
            .unwrap_or("")
            .split_whitespace()
            .collect();
        if fields.is_empty() {
            continue;
        }
        if fields.len() != 4 {
            return Err("invalid embedded calibration row".into());
        }
        let id: usize = fields[0].parse().map_err(|_| "invalid calibration joint")?;
        let number = |s: &str| -> Result<f32, String> {
            let n: f32 = s.parse().map_err(|_| "invalid calibration number")?;
            if n.is_finite() {
                Ok(n)
            } else {
                Err("nonfinite calibration".into())
            }
        };
        let dir = number(fields[1])?;
        let scale = number(fields[2])?.abs();
        let offset = number(fields[3])?;
        let gain = dir * scale;
        if id >= 7 || seen[id] || !gain.is_finite() || gain == 0.0 {
            return Err("invalid or duplicate calibration joint".into());
        }
        values[id] = Calibration { gain, offset };
        seen[id] = true;
    }
    if seen.iter().any(|v| !v) {
        return Err("incomplete embedded calibration".into());
    }
    Ok(values)
}

impl Calibration {
    fn command(self, s: Setpoint) -> Setpoint {
        // Match the SDK's float32 calibration before its double wire arithmetic.
        Setpoint {
            q: (((s.q as f32) - self.offset) / self.gain) as f64,
            dq: ((s.dq as f32) / self.gain) as f64,
            tau: ((s.tau as f32) / self.gain) as f64,
            ..s
        }
    }

    fn feedback(self, mut f: Feedback) -> Feedback {
        [f.q, f.dq, f.tau] = self.feedback_motion([f.q, f.dq, f.tau]);
        f
    }

    fn feedback_motion(self, [q, dq, tau]: [f64; 3]) -> [f64; 3] {
        [
            (self.gain * q as f32 + self.offset) as f64,
            (self.gain * dq as f32) as f64,
            (self.gain * tau as f32) as f64,
        ]
    }
}

fn ratio(id: u8) -> f64 {
    match id {
        0..=1 => 32.0,
        2..=4 => 38.0 / 3.0,
        _ => 16.0,
    }
}

/// SDK CRC32: init FFFFFFFF, polynomial 04C11DB7, MSB first, each
/// little-endian 32-bit word consumed from its highest byte, no final xor.
fn crc32(words: &[u8]) -> u32 {
    debug_assert_eq!(words.len() % 4, 0);
    let mut crc = u32::MAX;
    for word in words.as_chunks::<4>().0 {
        for byte in word.iter().rev() {
            crc ^= (*byte as u32) << 24;
            for _ in 0..8 {
                crc = if crc & 0x8000_0000 != 0 {
                    (crc << 1) ^ 0x04c1_1db7
                } else {
                    crc << 1
                };
            }
        }
    }
    crc
}

pub(crate) fn encode(id: u8, s: Setpoint) -> Result<[u8; COMMAND_LEN], String> {
    if id >= 7 {
        return Err("joint ID must be 0..6".into());
    }
    let mut packet = [0u8; COMMAND_LEN];
    packet[..2].copy_from_slice(&[0xfe, 0xee]);
    match s.mode {
        // SDK softStopAll(): zero fields; position is NOT the calibrated zero.
        DriveMode::Disabled => packet[2] = id,
        // SDK buildClearPacket(): mode 6, watchdog off, raw torque -256.
        DriveMode::ClearFault => {
            packet[2] = id | 0x60;
            packet[4..6].copy_from_slice(&(-256i16).to_le_bytes());
        }
        DriveMode::Servo => {
            let numbers = [s.q, s.dq, s.tau, s.kp, s.kd];
            if numbers
                .iter()
                .any(|v| !v.is_finite() || !(*v as f32).is_finite())
            {
                return Err("nonfinite servo setpoint".into());
            }
            let q = s.q as f32;
            let dq = s.dq as f32;
            let tau = s.tau as f32;
            let kp = s.kp as f32;
            let kd = s.kd as f32;
            // Never wrap SDK's premature int16/int32 narrowing. Reject values
            // outside either its advertised limits or the actual wire range.
            #[allow(clippy::excessive_precision)]
            let in_sdk_range = (-32508.539391f32..=32508.539376f32).contains(&q)
                && (-253.972964f32..=253.965213f32).contains(&dq)
                && (-162.133333f32..=162.128385f32).contains(&tau)
                && (0.0..=410.725243f32).contains(&kp)
                && (0.0..=102.681311f32).contains(&kd);
            if !in_sdk_range {
                return Err("servo setpoint exceeds SDK range".into());
            }
            let r = ratio(id);
            let t = (tau as f64 / r * 2560.0).round();
            let v = (dq as f64 * r * 64.0 / PI / 2.0).round();
            let p = (q as f64 * r * 32768.0 / PI / 2.0).round();
            let k = (kp as f64 / (r * r) * 12800.0).round();
            let d = (kd as f64 / (r * r) * 51200.0).round();
            if [t, v, k, d]
                .iter()
                .any(|x| *x < i16::MIN as f64 || *x > i16::MAX as f64)
                || p < i32::MIN as f64
                || p > i32::MAX as f64
            {
                return Err("servo setpoint exceeds signed wire range for this joint".into());
            }
            packet[2] = id | 0x90; // FOC + SDK timeout protection; no implicit enable.
            packet[4..6].copy_from_slice(&(t as i16).to_le_bytes());
            packet[6..8].copy_from_slice(&(v as i16).to_le_bytes());
            packet[8..12].copy_from_slice(&(p as i32).to_le_bytes());
            packet[12..14].copy_from_slice(&(k as i16).to_le_bytes());
            packet[14..16].copy_from_slice(&(d as i16).to_le_bytes());
        }
    }
    let crc = crc32(&packet[..16]);
    packet[16..].copy_from_slice(&crc.to_le_bytes());
    Ok(packet)
}

fn decode_motion(id: u8, t: i16, v: i16, q: i32) -> [f64; 3] {
    // IDs outside this seven-joint bus use the SDK's fallback reduction.
    let r = if id < 7 { ratio(id) } else { 38.0 / 3.0 };
    [
        ((2.0 * PI * q as f64 / 32768.0 / r) as f32) as f64,
        (((v as f32 / 64.0 * 2.0) as f64 * PI / r) as f32) as f64,
        (((t as f32 / 2560.0) as f64 * r) as f32) as f64,
    ]
}

fn decode(frame: &[u8]) -> Option<Feedback> {
    if frame.len() != FEEDBACK_LEN || frame[..2] != [0xfc, 0xee] {
        return None;
    }
    if crc32(&frame[2..22]) != u32::from_le_bytes(frame[22..26].try_into().ok()?) {
        return None;
    }
    let id = frame[2] & 0x0f;
    let t = i16::from_le_bytes(frame[6..8].try_into().ok()?);
    let v = i16::from_le_bytes(frame[8..10].try_into().ok()?);
    let q = i32::from_le_bytes(frame[10..14].try_into().ok()?);
    let [q, dq, tau] = decode_motion(id, t, v, q);
    Some(Feedback {
        id,
        q,
        dq,
        tau,
        temperature: (frame[3] as i8) as f64,
        winding_temperature: f64::from(frame[4]),
        voltage: frame[5] as f64 / 2.0,
        error: u32::from_le_bytes(frame[14..18].try_into().ok()?),
        warning: (u16::from_le_bytes(frame[18..20].try_into().ok()?) >> 13) as u8,
        mode: (frame[2] >> 4) & 7,
        timeout: frame[2] & 0x80 != 0,
    })
}

#[derive(Default)]
struct Receiver {
    bytes: Vec<u8>,
}

impl Receiver {
    fn feed(&mut self, input: &[u8], stats: &mut BusStats) -> Vec<Feedback> {
        let mut frames = Vec::new();
        // Process per byte so arbitrary input/noise never grows retained state
        // past 26 bytes. On bad CRC advance ONE byte, preserving nested headers.
        for byte in input {
            self.bytes.push(*byte);
            loop {
                if self.bytes.len() < 2 {
                    break;
                }
                if self.bytes[..2] != [0xfc, 0xee] {
                    self.bytes.remove(0);
                    continue;
                }
                if self.bytes.len() < FEEDBACK_LEN {
                    break;
                }
                if let Some(frame) = decode(&self.bytes) {
                    stats.received = stats.received.saturating_add(1);
                    frames.push(frame);
                    self.bytes.clear();
                } else {
                    stats.crc_errors = stats.crc_errors.saturating_add(1);
                    self.bytes.remove(0);
                }
            }
        }
        frames
    }
}

pub struct Bus {
    port: Box<dyn SerialPort>,
    calibration: [Calibration; 7],
    receiver: Receiver,
    budget: Duration,
    stats: BusStats,
    incomplete_write: bool,
}

impl Bus {
    /// Opens an exclusive native serial port; sends nothing. The supplied
    /// timeout bounds progress within one packet. Completed packets and the
    /// required inter-packet gaps do not consume the next packet's budget.
    pub fn open(port: &str, side: &str, baud: u32, timeout: Duration) -> Result<Self, String> {
        let calibration = calibrations(side)?;
        if baud == 0 {
            return Err("baud rate must be positive".into());
        }
        if timeout.is_zero() || Instant::now().checked_add(timeout).is_none() {
            return Err("serial I/O timeout must be positive and representable".into());
        }
        let budget = timeout;
        let port = serialport::new(port, baud)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .timeout(budget.min(IO_SLICE))
            .open()
            .map_err(|e| format!("open serial port: {e}"))?;
        port.clear(ClearBuffer::Input)
            .map_err(|e| format!("clear stale input after open: {e}"))?;
        Ok(Self {
            port,
            calibration,
            receiver: Receiver::default(),
            budget,
            stats: BusStats::default(),
            incomplete_write: false,
        })
    }

    pub fn stats(&self) -> BusStats {
        self.stats
    }

    /// Returns only a CRC-valid frame of the requested ID read in this exchange.
    /// Preexisting host/OS input is discarded before TX. There is no wire
    /// sequence number: a delayed same-ID reply arriving after TX cannot be
    /// distinguished from this request's reply. No hardware ACK is fabricated.
    /// Ok(None) means response deadline; Err may mean the command was sent.
    #[cfg(test)]
    pub fn exchange(&mut self, id: u8, setpoint: Setpoint) -> Result<Option<Feedback>, String> {
        if self.incomplete_write {
            return Err("previous command was partially written; reconnect serial port".into());
        }
        let cal = *self
            .calibration
            .get(id as usize)
            .ok_or("joint ID must be 0..6")?;
        let packet = encode(id, cal.command(setpoint))?;
        let deadline = Instant::now() + self.budget;
        self.receiver.bytes.clear();
        self.port
            .clear(ClearBuffer::Input)
            .map_err(|e| format!("clear stale input: {e}"))?;
        self.write_packet(&packet, deadline)?;
        // Never call flush/tcdrain here: it has no portable bounded deadline.
        let mut bytes = [0u8; 256];
        loop {
            if !self.set_io_deadline(deadline)? {
                return Ok(None);
            }
            match self.port.read(&mut bytes) {
                Ok(0) => return Err("serial read returned zero (device closed)".into()),
                Ok(n) => {
                    let frames = self.receiver.feed(&bytes[..n], &mut self.stats);
                    // A driver may return after its requested timeout. Do not
                    // publish that feedback as a successful bounded exchange.
                    if Instant::now() >= deadline {
                        continue;
                    }
                    if let Some(frame) = frames.into_iter().find(|frame| frame.id == id) {
                        return Ok(Some(cal.feedback(frame)));
                    }
                }
                Err(e) if transient(&e) => std::thread::yield_now(),
                Err(e) => return Err(format!("serial read after command sent: {e}")),
            }
        }
    }

    /// Sends all seven joint setpoints in one cycle, then collects any feedback
    /// that arrives before the shared deadline. Missing frames are left as
    /// `None`; freshness checks decide whether motion may continue.
    #[cfg(test)]
    pub fn exchange_all(
        &mut self,
        setpoints: &[Setpoint; 7],
    ) -> Result<[Option<Feedback>; 7], String> {
        self.receiver.bytes.clear();
        self.port
            .clear(ClearBuffer::Input)
            .map_err(|e| format!("clear stale input: {e}"))?;
        self.write_all(setpoints)?;
        let deadline = Instant::now() + self.budget;
        let mut feedback: [Option<Feedback>; 7] = std::array::from_fn(|_| None);
        let mut bytes = [0u8; 256];
        loop {
            if feedback.iter().all(Option::is_some) {
                return Ok(feedback);
            }
            if !self.set_io_deadline(deadline)? {
                return Ok(feedback);
            }
            match self.port.read(&mut bytes) {
                Ok(0) => return Err("serial read returned zero (device closed)".into()),
                Ok(n) => {
                    let frames = self.receiver.feed(&bytes[..n], &mut self.stats);
                    // A driver may return after its requested timeout. Do not
                    // publish that feedback as a successful bounded exchange.
                    if Instant::now() >= deadline {
                        continue;
                    }
                    for frame in frames {
                        let id = frame.id as usize;
                        if id < 7 {
                            feedback[id] = Some(self.calibration[id].feedback(frame));
                        }
                    }
                }
                Err(e) if transient(&e) => std::thread::yield_now(),
                Err(e) => return Err(format!("serial read after command sent: {e}")),
            }
        }
    }

    /// Writes one seven-joint command batch. It never reads feedback and can
    /// be called from a dedicated I/O thread while another path drains frames.
    #[cfg(test)]
    pub fn write_all(&mut self, setpoints: &[Setpoint; 7]) -> Result<(), String> {
        self.write_all_recorded(setpoints, None)
    }

    fn write_all_recorded(
        &mut self,
        setpoints: &[Setpoint; 7],
        recording: Option<(&Journal, Instant, TransmitMetadata)>,
    ) -> Result<(), String> {
        if self.incomplete_write {
            return Err("previous command was partially written; reconnect serial port".into());
        }
        let started = Instant::now();
        let mut packets = [[0u8; COMMAND_LEN]; 7];
        for (i, packet) in packets.iter_mut().enumerate() {
            *packet = encode(i as u8, self.calibration[i].command(setpoints[i]))?;
        }
        for (joint, packet) in packets.iter().enumerate() {
            // Only an incomplete packet has an I/O progress deadline. Time
            // spent after a completed packet cannot prove a transport failure.
            let deadline = Instant::now() + self.budget;
            self.write_packet(packet, deadline).map_err(|error| {
                format!(
                    "serial batch J{} after {} us: {error}",
                    joint + 1,
                    started.elapsed().as_micros()
                )
            })?;
            // The official SDK keeps this gap after every packet, including
            // the last one, before the adapter turns around to receive.
            std::thread::sleep(Duration::from_micros(200));
        }
        if let Some((journal, started, metadata)) = recording {
            sample_transmit(
                journal,
                started,
                setpoints,
                metadata,
                &packets,
                &self.calibration,
            );
        }
        Ok(())
    }

    /// Reads at most one driver chunk. A read timeout or a partial frame is
    /// not an error; the persistent receiver keeps partial bytes for later.
    pub fn read_available(&mut self) -> Result<Vec<Feedback>, String> {
        let mut bytes = [0u8; 512];
        match self.port.read(&mut bytes) {
            Ok(0) => Err("serial read returned zero (device closed)".into()),
            Ok(n) => {
                let frames = self.receiver.feed(&bytes[..n], &mut self.stats);
                Ok(frames
                    .into_iter()
                    .filter_map(|frame| {
                        let id = frame.id as usize;
                        (id < 7).then(|| self.calibration[id].feedback(frame))
                    })
                    .collect())
            }
            Err(e) if transient(&e) => Ok(Vec::new()),
            Err(e) => Err(format!("serial read: {e}")),
        }
    }

    fn write_packet(&mut self, packet: &[u8], deadline: Instant) -> Result<(), String> {
        let mut offset = 0;
        while offset < packet.len() {
            let ready = self.set_io_deadline(deadline).and_then(|ready| {
                if ready {
                    Ok(())
                } else {
                    Err("serial transaction deadline expired".into())
                }
            });
            if let Err(error) = ready {
                self.incomplete_write = offset != 0;
                return Err(format!(
                    "{error}; wrote {offset}/{COMMAND_LEN} command bytes"
                ));
            }
            match self.port.write(&packet[offset..]) {
                Ok(0) => {
                    self.incomplete_write = offset != 0;
                    return Err(format!(
                        "serial write returned zero; wrote {offset}/{COMMAND_LEN} command bytes"
                    ));
                }
                Ok(n) => offset += n,
                Err(e) if transient(&e) => std::thread::yield_now(),
                Err(e) => {
                    self.incomplete_write = offset != 0;
                    return Err(format!(
                        "serial write: {e}; wrote {offset}/{COMMAND_LEN} command bytes"
                    ));
                }
            }
        }
        Ok(())
    }

    fn set_io_deadline(&mut self, deadline: Instant) -> Result<bool, String> {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            self.stats.timeouts = self.stats.timeouts.saturating_add(1);
            return Ok(false);
        }
        self.port
            .set_timeout(remaining.min(IO_SLICE))
            .map(|()| true)
            .map_err(|e| format!("set serial I/O timeout: {e}"))
    }
}

#[derive(Debug)]
pub struct ReceivedFeedback {
    pub value: Feedback,
    pub at: Instant,
    /// This joint's position (rad) and feedforward effort (Nm) from the last
    /// published successful servo write when the fault frame was received.
    pub fault_command: Option<(f64, f64)>,
}

impl ReceivedFeedback {
    fn capture(
        value: Feedback,
        sent: &watch::Receiver<Option<SentReference>>,
        now: impl FnOnce() -> Instant,
    ) -> Self {
        let axis = usize::from(value.id);
        let receipt = (value.error != 0 && axis < 7).then(|| sent.borrow());
        // Timestamp while the fault frame's receipt is pinned against replacement.
        let at = now();
        let fault_command = receipt
            .as_deref()
            .and_then(Option::as_ref)
            .filter(|s| s.completed_at <= at)
            .map(|s| (s.q[axis], s.feedforward_effort[axis]));
        Self {
            value,
            at,
            fault_command,
        }
    }
}

enum BusEvent {
    Feedback(ReceivedFeedback),
    Error(String),
    ReferenceRejected {
        generation: u64,
        epoch: u64,
        held: [f64; 7],
        reason: SampleStatus,
    },
}

#[derive(Debug)]
pub enum BusError {
    Io(String),
    Reference {
        generation: u64,
        epoch: u64,
        held: [f64; 7],
        reason: SampleStatus,
    },
}

enum BusCommand {
    Disable(Sender<Result<(), String>>),
    Write(Box<[Setpoint; 7]>, Sender<Result<(), String>>),
}

fn stopped_setpoints() -> [Setpoint; 7] {
    std::array::from_fn(|_| Setpoint {
        mode: DriveMode::Disabled,
        q: 0.0,
        dq: 0.0,
        tau: 0.0,
        kp: 0.0,
        kd: 0.0,
    })
}

#[cfg(test)]
pub(crate) fn test_wire_targets(side: &str, q: [f64; 7], dq: [f64; 7]) -> ([f64; 7], [f64; 7]) {
    let calibration = calibrations(side).unwrap();
    let values: [[f64; 3]; 7] = std::array::from_fn(|i| {
        let packet = encode(
            i as u8,
            calibration[i].command(Setpoint {
                mode: DriveMode::Servo,
                q: q[i],
                dq: dq[i],
                tau: 0.0,
                kp: 60.0,
                kd: 10.0,
            }),
        )
        .unwrap();
        calibration[i].feedback_motion(decode_motion(
            i as u8,
            i16::from_le_bytes(packet[4..6].try_into().unwrap()),
            i16::from_le_bytes(packet[6..8].try_into().unwrap()),
            i32::from_le_bytes(packet[8..12].try_into().unwrap()),
        ))
    });
    (values.map(|v| v[0]), values.map(|v| v[1]))
}

#[derive(Clone)]
struct PublishedSetpoints {
    values: [Setpoint; 7],
    seq: u64,
    reference: Option<Reference>,
}

struct Prepared {
    enable_epoch: u64,
    hold_scale: [f64; 7],
    calibration: Option<(u64, PlaybackPhase)>,
    values: [Setpoint; 7],
    timing: Option<Timing>,
    rejection: Option<SampleStatus>,
    motion_velocity: [f64; 7],
    gravity: [f64; 7],
    tuning: Option<ServoSample>,
}
impl Prepared {
    fn committed(&self, at: Instant) -> Committed {
        Committed {
            enable_epoch: self.enable_epoch,
            hold_scale: self.hold_scale,
            calibration: self.calibration,
            at,
            values: self.values,
            motion_velocity: self.motion_velocity,
            gravity: self.gravity,
        }
    }

    fn direct(values: [Setpoint; 7]) -> Self {
        Self {
            enable_epoch: 0,
            hold_scale: [1.0; 7],
            calibration: None,
            values,
            timing: None,
            rejection: None,
            motion_velocity: values.map(|s| s.dq),
            gravity: values.map(|s| s.tau),
            tuning: None,
        }
    }
}

#[derive(Clone, Copy)]
struct Committed {
    enable_epoch: u64,
    hold_scale: [f64; 7],
    calibration: Option<(u64, PlaybackPhase)>,
    at: Instant,
    values: [Setpoint; 7],
    motion_velocity: [f64; 7],
    gravity: [f64; 7],
}

/// Latest successfully written servo batch and its calibration clock, if active.
#[derive(Clone, Copy, Debug)]
pub struct SentReference {
    pub at: Instant,
    /// Host completion of the successful seven-packet write, not drive execution.
    pub completed_at: Instant,
    #[cfg(test)]
    pub generation: u64,
    pub epoch: u64,
    pub calibration_phase: Option<PlaybackPhase>,
    pub q: [f64; 7],
    pub dq: [f64; 7],
    /// Feedforward torque of this successful write in joint Nm, before wire quantization.
    pub feedforward_effort: [f64; 7],
    /// A stale/invalid source retained the last successful position with zero
    /// trajectory velocity; the original program's acceleration no longer applies.
    pub fallback_hold: bool,
}
impl Committed {
    fn direct(at: Instant, values: [Setpoint; 7]) -> Self {
        Self {
            enable_epoch: 0,
            hold_scale: [1.0; 7],
            calibration: None,
            at,
            values,
            motion_velocity: values.map(|s| s.dq),
            gravity: values.map(|s| s.tau),
        }
    }
    fn sent(
        &self,
        source: &PublishedSetpoints,
        completed_at: Instant,
        rejection: Option<SampleStatus>,
    ) -> Option<SentReference> {
        let reference = source.reference.as_ref()?;
        let previous = self.servo()?;
        Some(SentReference {
            at: self.at,
            completed_at,
            #[cfg(test)]
            generation: reference.generation,
            epoch: reference.epoch,
            calibration_phase: self.calibration.map(|(_, phase)| phase),
            q: previous.q,
            dq: previous.dq,
            feedforward_effort: self.values.map(|s| s.tau),
            fallback_hold: rejection.is_some(),
        })
    }

    fn servo(&self) -> Option<Previous> {
        self.values
            .iter()
            .all(|s| s.mode == DriveMode::Servo)
            .then(|| Previous {
                at: self.at,
                q: self.values.map(|s| s.q),
                dq: self.motion_velocity,
            })
    }
}

impl PublishedSetpoints {
    fn is_current(&self, epoch: &AtomicU64, at: Instant) -> bool {
        self.values.iter().all(|s| s.mode == DriveMode::Disabled)
            || self.reference.as_ref().is_none_or(|r| {
                r.epoch == epoch.load(Ordering::Acquire)
                    && r.valid_until.is_none_or(|until| at < until)
            })
    }
    fn prepare(&self, at: Instant, committed: Option<Committed>) -> Result<Prepared, String> {
        if self.values.iter().all(|s| s.mode != DriveMode::Servo) {
            return Ok(Prepared::direct(self.values));
        }
        let Some(reference) = &self.reference else {
            return Ok(Prepared::direct(self.values));
        };
        let previous = committed.and_then(|c| c.servo());
        let last_phase = committed.and_then(|c| {
            let (epoch, phase) = c.calibration?;
            (epoch == reference.epoch).then_some((phase, c.at))
        });
        let send_phase = match reference.sampling {
            Sampling::Calibration { phase, limits, .. } => {
                Some(last_phase.map_or(phase, |(last, _)| last.for_send(phase, at, limits)))
            }
            _ => None,
        };
        let sample = reference.sample_with_phase(at, previous, send_phase);
        let (mut values, q, dq, gravity, timing, rejection) = match sample {
            Ok(sample) => (
                self.values,
                sample.q,
                sample.dq,
                self.values.map(|s| s.tau),
                sample.timing,
                None,
            ),
            Err(reason) => {
                let committed = committed.filter(|c| c.servo().is_some()).ok_or_else(|| {
                    format!("send reference {reason:?} before a valid servo target")
                })?;
                (
                    committed.values,
                    committed.values.map(|s| s.q),
                    [0.0; 7],
                    committed.gravity,
                    reference.timing(at, reason),
                    Some(reason),
                )
            }
        };
        // Actuator feedforward is separate from the velocity used to integrate
        // positions. A zero feedforward weight must not stop the trajectory.
        let (applied, delivered_gravity) = reference.servo_tuning.apply_with_effort_limits(
            dq,
            gravity,
            reference.effort_limits,
        )?;
        // Blend from the last successful servo write in either direction.
        // At the nominal rate half-to-full gain takes 100 ms. A delayed write
        // advances by at most one normal step; a first enable starts at its target.
        // A fallback repeats the last successful command, including its session.
        let enable_epoch = match (rejection, committed) {
            (Some(_), Some(last)) => last.enable_epoch,
            _ => reference.enable_epoch,
        };
        let last_servo = committed.filter(|c| previous.is_some() && c.enable_epoch == enable_epoch);
        let mut hold_scale = last_servo
            .map_or([crate::tuning::hold_stiffness_scale(0.0); 7], |last| {
                last.hold_scale
            });
        let max_change = last_servo.map_or(1.0, |last| {
            5.0 * at
                .saturating_duration_since(last.at)
                .min(WRITE_PERIOD)
                .as_secs_f64()
        });
        for (i, value) in values.iter_mut().enumerate() {
            if value.mode == DriveMode::Servo {
                // A rejected sample already contains the previous effective Kp.
                if rejection.is_none() {
                    let target = reference.hold_feedback.as_ref().map_or(1.0, |measured| {
                        measured[i].map_or(hold_scale[i], |position| {
                            crate::tuning::hold_stiffness_scale(q[i] - position)
                        })
                    });
                    hold_scale[i] += (target - hold_scale[i]).clamp(-max_change, max_change);
                    value.kp *= hold_scale[i];
                }
                value.q = q[i];
                value.dq = applied.dq_feedforward[i];
                value.tau = applied.tau[i];
                value.kd = reference.servo_tuning.joints[i].kd;
            }
        }
        Ok(Prepared {
            enable_epoch,
            hold_scale,
            calibration: send_phase.map(|phase| {
                let phase = if rejection.is_none() {
                    phase
                } else if let Some((held @ PlaybackPhase::Held { .. }, _)) = last_phase {
                    held
                } else {
                    PlaybackPhase::Held {
                        since: at,
                        offset: last_phase.map_or(0.0, |(last, sent)| last.progress(sent)),
                    }
                };
                (reference.epoch, phase)
            }),
            values,
            motion_velocity: dq,
            gravity,
            timing: Some(timing),
            rejection,
            tuning: Some(ServoSample {
                revision: reference.tuning_revision,
                dq_feedforward: applied.dq_feedforward,
                tau_gravity: delivered_gravity,
                friction_requested: applied.friction_requested,
                friction_applied: applied.friction_applied,
                clamped: applied.clamped,
            }),
        })
    }
}

#[derive(Clone, Copy)]
struct TransmitMetadata {
    seq: u64,
    timing: Option<Timing>,
    motion_velocity: Option<[f64; 7]>,
    tuning: Option<ServoSample>,
}
impl TransmitMetadata {
    fn direct(seq: u64) -> Self {
        Self {
            seq,
            timing: None,
            motion_velocity: None,
            tuning: None,
        }
    }
}

// Stay on the original deadline grid. Skip missed slots rather than emitting
// catch-up batches; leave at least half a period between ordinary starts.
fn next_deadline(scheduled: Instant, started: Instant, finished: Instant) -> Instant {
    let minimum = finished.max(started + WRITE_PERIOD / 2);
    if scheduled > minimum {
        return scheduled;
    }
    let slots =
        minimum.saturating_duration_since(scheduled).as_nanos() / WRITE_PERIOD.as_nanos() + 1;
    scheduled + WRITE_PERIOD.mul_f64(slots as f64)
}

fn sample_transmit(
    journal: &Journal,
    started: Instant,
    values: &[Setpoint; 7],
    metadata: TransmitMetadata,
    packets: &[[u8; COMMAND_LEN]; 7],
    calibration: &[Calibration; 7],
) {
    let write_us = started.elapsed().as_micros().min(u64::MAX as u128) as u64;
    // Read the fields actually sent, after calibration, f32 narrowing and
    // integer rounding. Reuse the feedback conversion into joint coordinates.
    // Disabled/clear packets describe their raw fields, not an active target.
    let wire: [[f64; 3]; 7] = std::array::from_fn(|i| {
        let packet = &packets[i];
        calibration[i].feedback_motion(decode_motion(
            packet[2] & 0x0f,
            i16::from_le_bytes(packet[4..6].try_into().unwrap()),
            i16::from_le_bytes(packet[6..8].try_into().unwrap()),
            i32::from_le_bytes(packet[8..12].try_into().unwrap()),
        ))
    });
    journal.sample(
        started,
        TraceData::Transmit {
            seq: metadata.seq,
            timing: metadata.timing,
            tuning: metadata.tuning,
            mode: std::array::from_fn(|i| (packets[i][2] >> 4) & 0x07),
            q_ref: std::array::from_fn(|i| values[i].q),
            dq_ref: metadata
                .motion_velocity
                .unwrap_or_else(|| values.map(|s| s.dq)),
            tau_ff: std::array::from_fn(|i| values[i].tau),
            q_wire: std::array::from_fn(|i| wire[i][0]),
            dq_wire: std::array::from_fn(|i| wire[i][1]),
            tau_wire: std::array::from_fn(|i| wire[i][2]),
            kp: std::array::from_fn(|i| values[i].kp),
            kd: std::array::from_fn(|i| values[i].kd),
            write_us,
        },
    );
}

/// Owns the serial bus on a dedicated thread. The control loop only publishes
/// the latest setpoints and drains timestamped feedback, so a slow feedback
/// frame cannot lengthen the command cycle.
pub struct BusHandle {
    setpoints: watch::Sender<PublishedSetpoints>,
    sent: watch::Receiver<Option<SentReference>>,
    reference_write: Arc<Mutex<()>>,
    command: Sender<BusCommand>,
    events: EventReceiver<BusEvent>,
    stats: Arc<Mutex<BusStats>>,
    failure: Arc<OnceLock<String>>,
    stop: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
    reader: Option<JoinHandle<()>>,
}

impl BusHandle {
    pub fn open_recorded(
        port: &str,
        side: &str,
        baud: u32,
        timeout: Duration,
        journal: Journal,
        reference_epoch: Arc<AtomicU64>,
    ) -> Result<Self, String> {
        Self::start_with_epoch(
            Bus::open(port, side, baud, timeout)?,
            Some(journal),
            reference_epoch,
        )
    }

    #[cfg(test)]
    fn start(bus: Bus) -> Result<Self, String> {
        Self::start_recorded(bus, None)
    }
    #[cfg(test)]
    fn start_recorded(bus: Bus, journal: Option<Journal>) -> Result<Self, String> {
        Self::start_with_epoch(bus, journal, Arc::new(AtomicU64::new(0)))
    }
    fn start_with_epoch(
        mut bus: Bus,
        journal: Option<Journal>,
        reference_epoch: Arc<AtomicU64>,
    ) -> Result<Self, String> {
        let reader_port = bus
            .port
            .try_clone()
            .map_err(|error| format!("clone serial reader: {error}"))?;
        let mut reader = Bus {
            port: reader_port,
            calibration: bus.calibration,
            receiver: Receiver::default(),
            budget: bus.budget,
            stats: BusStats::default(),
            incomplete_write: false,
        };
        let (setpoint_tx, mut setpoint_rx) = watch::channel(PublishedSetpoints {
            values: stopped_setpoints(),
            seq: 0,
            reference: None,
        });
        let (sent_tx, sent) = watch::channel(None);
        let reference_write = Arc::new(Mutex::new(()));
        let writer_gate = reference_write.clone();
        let (command_tx, command_rx) = bounded::<BusCommand>(1);
        let (event_tx, event_rx) = unbounded();
        let writer_events = event_tx.clone();
        let stats = Arc::new(Mutex::new(BusStats::default()));
        let handle_stats = stats.clone();
        let stop = Arc::new(AtomicBool::new(false));
        let worker_stop = stop.clone();
        let reader_stats = stats.clone();
        let writer_stats = stats;
        let writer_journal = journal.clone();
        let failure = Arc::new(OnceLock::new());
        let writer_failure = failure.clone();
        let join = thread::Builder::new()
            .name("arm-serial-tx".into())
            .spawn(move || {
                let mut last = setpoint_rx.borrow_and_update().clone();
                let mut next_write = Instant::now();
                let mut previous_write = None;
                let mut committed: Option<Committed> = None;
                let mut rejected: Option<(u64, SampleStatus)> = None;
                let mut disabled = false;
                loop {
                    if worker_stop.load(Ordering::Acquire) {
                        break;
                    }
                    if let Ok(command) = command_rx.try_recv() {
                        match command {
                            BusCommand::Disable(reply) => {
                                let started = Instant::now();
                                let values = stopped_setpoints();
                                let result = bus.write_all_recorded(
                                    &values,
                                    writer_journal.as_ref().map(|journal| {
                                        (journal, started, TransmitMetadata::direct(0))
                                    }),
                                );
                                if let Err(error) = &result {
                                    let _ = writer_failure.set(error.clone());
                                    let _ = writer_events.send(BusEvent::Error(error.clone()));
                                }
                                if result.is_ok() {
                                    committed = Some(Committed::direct(started, values));
                                }
                                sent_tx.send_replace(None);
                                let failed = result.is_err();
                                let _ = reply.send(result);
                                if failed {
                                    break;
                                }
                                // Sticky until this bus is closed; cached motion cannot revive it.
                                disabled = true;
                            }
                            BusCommand::Write(setpoints, reply) => {
                                let started = Instant::now();
                                let result = bus.write_all_recorded(
                                    &setpoints,
                                    writer_journal.as_ref().map(|journal| {
                                        (journal, started, TransmitMetadata::direct(0))
                                    }),
                                );
                                if let Err(error) = &result {
                                    let _ = writer_failure.set(error.clone());
                                    let _ = writer_events.send(BusEvent::Error(error.clone()));
                                }
                                if result.is_ok() {
                                    last = PublishedSetpoints {
                                        values: *setpoints,
                                        seq: 0,
                                        reference: None,
                                    };
                                    // ClearFault is an action, not a hold target.
                                    // Keep other joints' states, but never replay it as a heartbeat.
                                    for value in &mut last.values {
                                        if value.mode == DriveMode::ClearFault {
                                            value.mode = DriveMode::Disabled;
                                        }
                                    }
                                    committed = Some(Committed::direct(started, *setpoints));
                                    rejected = None;
                                    next_write = next_deadline(next_write, started, Instant::now());
                                    if let Ok(mut shared) = writer_stats.lock() {
                                        shared.transmit.record(
                                            previous_write,
                                            started,
                                            Instant::now(),
                                        );
                                    }
                                    previous_write = Some(started);
                                }
                                sent_tx.send_replace(None);
                                let failed = result.is_err();
                                let _ = reply.send(result);
                                if failed {
                                    break;
                                }
                            }
                        }
                    }
                    if !disabled && Instant::now() >= next_write {
                        // Select immediately before sampling; updates do not move the deadline.
                        if setpoint_rx.has_changed().unwrap_or(false) {
                            last = setpoint_rx.borrow_and_update().clone();
                        }
                        let started = Instant::now();
                        if !last.is_current(&reference_epoch, started) {
                            next_write = next_deadline(next_write, started, started);
                        } else {
                            let prepared = match last.prepare(started, committed) {
                                Ok(prepared) => prepared,
                                Err(error) => {
                                    let _ = writer_failure.set(error.clone());
                                    sent_tx.send_replace(None);
                                    let _ = writer_events.send(BusEvent::Error(error));
                                    break;
                                }
                            };
                            // A stop or priority command can arrive while FK is evaluated.
                            // Once writing begins, each packet has a bounded progress deadline.
                            // A controller that invalidated the epoch can acquire this gate
                            // only before our final check or after our receipt is visible.
                            let writing = writer_gate.lock().unwrap();
                            let valid = last.is_current(&reference_epoch, Instant::now());
                            if valid && command_rx.is_empty() {
                                if let Err(error) = bus.write_all_recorded(
                                    &prepared.values,
                                    writer_journal.as_ref().map(|journal| {
                                        (
                                            journal,
                                            started,
                                            TransmitMetadata {
                                                seq: last.seq,
                                                timing: prepared.timing,
                                                motion_velocity: Some(prepared.motion_velocity),
                                                tuning: prepared.tuning,
                                            },
                                        )
                                    }),
                                ) {
                                    let _ = writer_failure.set(error.clone());
                                    sent_tx.send_replace(None);
                                    let _ = writer_events.send(BusEvent::Error(error));
                                    break;
                                }
                                let written = prepared.committed(started);
                                sent_tx.send_replace(written.sent(
                                    &last,
                                    Instant::now(),
                                    prepared.rejection,
                                ));
                                drop(writing);
                                committed = Some(written);
                                if let Some(reason) = prepared.rejection
                                    && let Some(source) = &last.reference
                                {
                                    if rejected.is_none_or(|(epoch, previous)| {
                                        epoch != source.epoch
                                            || (reason == SampleStatus::PathDeviation
                                                && previous != reason)
                                    }) {
                                        let _ = writer_events.send(BusEvent::ReferenceRejected {
                                            generation: source.generation,
                                            epoch: source.epoch,
                                            held: prepared.values.map(|s| s.q),
                                            reason,
                                        });
                                    }
                                    rejected = Some((source.epoch, reason));
                                } else {
                                    rejected = None;
                                }
                                if let Ok(mut shared) = writer_stats.lock() {
                                    shared
                                        .transmit
                                        .record(previous_write, started, Instant::now());
                                }
                                previous_write = Some(started);
                                next_write = next_deadline(next_write, started, Instant::now());
                                for value in &mut last.values {
                                    if value.mode == DriveMode::ClearFault {
                                        value.mode = DriveMode::Disabled;
                                    }
                                }
                            }
                        }
                    }
                    if let Ok(mut shared) = writer_stats.lock() {
                        shared.timeouts = bus.stats().timeouts;
                    }
                    std::thread::sleep(Duration::from_millis(1));
                }
            })
            .map_err(|error| format!("spawn serial I/O thread: {error}"))?;
        let reader_stop = stop.clone();
        let reader_failure = failure.clone();
        let reader_sent = sent.clone();
        let reader = match thread::Builder::new()
            .name("arm-serial-rx".into())
            .spawn(move || {
                loop {
                    if reader_stop.load(Ordering::Acquire) {
                        break;
                    }
                    match reader.read_available() {
                        Ok(frames) => {
                            for frame in frames {
                                let received =
                                    ReceivedFeedback::capture(frame, &reader_sent, Instant::now);
                                if let Some(journal) = journal.as_ref() {
                                    let frame = &received.value;
                                    journal.sample(
                                        received.at,
                                        TraceData::Feedback {
                                            joint: frame.id,
                                            q: frame.q,
                                            dq: frame.dq,
                                            tau: frame.tau,
                                            temperature: frame.temperature,
                                            winding_temperature: Some(frame.winding_temperature),
                                            voltage: frame.voltage,
                                            error: frame.error,
                                            mode: frame.mode,
                                            timeout: frame.timeout,
                                        },
                                    );
                                }
                                let _ = event_tx.send(BusEvent::Feedback(received));
                            }
                        }
                        Err(error) => {
                            let _ = reader_failure.set(error.clone());
                            let _ = event_tx.send(BusEvent::Error(error));
                            break;
                        }
                    }
                    if let Ok(mut shared) = reader_stats.lock() {
                        shared.received = reader.stats.received;
                        shared.crc_errors = reader.stats.crc_errors;
                    }
                }
            }) {
            Ok(reader) => reader,
            Err(error) => {
                stop.store(true, Ordering::Release);
                let _ = join.join();
                return Err(format!("spawn serial I/O thread: {error}"));
            }
        };
        Ok(Self {
            setpoints: setpoint_tx,
            sent,
            reference_write,
            command: command_tx,
            events: event_rx,
            stats: handle_stats,
            failure,
            stop,
            join: Some(join),
            reader: Some(reader),
        })
    }

    #[cfg(test)]
    pub fn send(&self, setpoints: [Setpoint; 7]) -> Result<(), String> {
        self.send_with_sequence(setpoints, 0)
    }
    #[cfg(test)]
    pub fn send_with_sequence(&self, setpoints: [Setpoint; 7], seq: u64) -> Result<(), String> {
        self.setpoints
            .send(PublishedSetpoints {
                values: setpoints,
                seq,
                reference: None,
            })
            .map_err(|_| self.io_error("serial I/O thread is closed".into()))
    }

    pub fn publish(
        &self,
        values: [Setpoint; 7],
        seq: u64,
        reference: Reference,
    ) -> Result<(), String> {
        self.setpoints
            .send(PublishedSetpoints {
                values,
                seq,
                reference: Some(reference),
            })
            .map_err(|_| self.io_error("serial I/O thread is closed".into()))
    }

    fn io_error(&self, fallback: String) -> String {
        self.failure.get().cloned().unwrap_or(fallback)
    }

    pub fn latest_reference(&self) -> Option<SentReference> {
        *self.sent.borrow()
    }

    /// Call after invalidating the source epoch, before publishing its successor.
    pub fn settled_reference(&self) -> std::task::Poll<Option<SentReference>> {
        match self.reference_write.try_lock() {
            Err(std::sync::TryLockError::WouldBlock) => std::task::Poll::Pending,
            _ => std::task::Poll::Ready(self.latest_reference()),
        }
    }

    #[cfg(test)]
    pub(crate) fn test_reference_write(&self) -> Arc<Mutex<()>> {
        self.reference_write.clone()
    }

    #[cfg(test)]
    pub(crate) fn emulated_board(journal: Journal, epoch: Arc<AtomicU64>) -> Result<Self, String> {
        tests::emulated_board(journal, epoch)
    }

    #[cfg(test)]
    pub(crate) fn test_transmit_sink(&self) -> impl Fn() -> Option<Reference> + use<> {
        let published = self.setpoints.subscribe();
        move || published.borrow().reference.clone()
    }

    #[cfg(test)]
    pub(crate) fn test_servo_writer()
    -> impl FnMut(Reference, [Setpoint; 7], Instant) -> SentReference {
        let mut committed = None;
        move |reference, values, at| {
            let source = PublishedSetpoints {
                values,
                seq: 0,
                reference: Some(reference),
            };
            let prepared = source.prepare(at, committed).unwrap();
            let written = prepared.committed(at);
            committed = Some(written);
            written.sent(&source, at, prepared.rejection).unwrap()
        }
    }

    #[cfg(test)]
    pub(crate) fn test_idle_bus() -> (Self, watch::Sender<Option<SentReference>>) {
        let (setpoints, _) = watch::channel(PublishedSetpoints {
            values: stopped_setpoints(),
            seq: 0,
            reference: None,
        });
        let (command, _) = bounded(1);
        let (_, events) = unbounded();
        let (sent_tx, sent) = watch::channel(None);
        (
            Self {
                setpoints,
                sent,
                reference_write: Arc::default(),
                command,
                events,
                stats: Arc::new(Mutex::new(BusStats::default())),
                failure: Arc::new(OnceLock::new()),
                stop: Arc::new(AtomicBool::new(false)),
                join: None,
                reader: None,
            },
            sent_tx,
        )
    }

    #[cfg(test)]
    pub(crate) fn test_feedback_queue(&mut self) -> impl Fn(Feedback, Instant) + use<> {
        let (tx, rx) = unbounded();
        self.events = rx;
        let sent = self.sent.clone();
        move |frame, at| {
            let received = ReceivedFeedback::capture(frame, &sent, || at);
            tx.send(BusEvent::Feedback(received)).unwrap();
        }
    }

    #[cfg(test)]
    pub(crate) fn test_failed_bus(
        message: &str,
        frames: impl IntoIterator<Item = (Feedback, Instant)>,
    ) -> Self {
        let (mut bus, _) = Self::test_idle_bus();
        let (events, receiver) = unbounded();
        bus.failure.set(message.into()).unwrap();
        for (frame, at) in frames {
            events
                .send(BusEvent::Feedback(ReceivedFeedback {
                    value: frame,
                    at,
                    fault_command: None,
                }))
                .unwrap();
        }
        events.send(BusEvent::Error(message.into())).unwrap();
        bus.events = receiver;
        bus
    }

    #[cfg(test)]
    pub(crate) fn test_clear_response_bus() -> Self {
        tests::clear_response_bus()
    }

    pub fn drain(&self) -> (Vec<ReceivedFeedback>, Option<BusError>) {
        let mut feedback = Vec::new();
        let mut error = None;
        for event in self.events.try_iter() {
            match event {
                BusEvent::Feedback(received) => feedback.push(received),
                BusEvent::Error(message) => error = Some(BusError::Io(self.io_error(message))),
                BusEvent::ReferenceRejected {
                    generation,
                    epoch,
                    held,
                    reason,
                } => {
                    if !matches!(error, Some(BusError::Io(_))) {
                        error = Some(BusError::Reference {
                            generation,
                            epoch,
                            held,
                            reason,
                        });
                    }
                }
            }
        }
        (feedback, error)
    }

    pub fn disable(&self) -> Result<(), String> {
        let (reply, result) = bounded(1);
        self.command
            .send_timeout(BusCommand::Disable(reply), Duration::from_millis(100))
            .map_err(|error| self.io_error(format!("serial I/O command queue: {error}")))?;
        result
            .recv_timeout(Duration::from_millis(500))
            .map_err(|error| self.io_error(format!("serial disable acknowledgement: {error}")))?
            .map_err(|error| self.io_error(error))
    }

    pub fn write_blocking(&self, setpoints: [Setpoint; 7]) -> Result<(), String> {
        let (reply, result) = bounded(1);
        self.command
            .send_timeout(
                BusCommand::Write(Box::new(setpoints), reply),
                Duration::from_millis(100),
            )
            .map_err(|error| self.io_error(format!("serial I/O command queue: {error}")))?;
        result
            .recv_timeout(Duration::from_millis(500))
            .map_err(|error| self.io_error(format!("serial write acknowledgement: {error}")))?
            .map_err(|error| self.io_error(error))
    }

    pub fn stats(&self) -> BusStats {
        self.stats.lock().map(|value| *value).unwrap_or_default()
    }

    /// Reset only the diagnostic window; receive/error counters remain cumulative.
    pub fn take_transmit_stats(&self) -> TransmitStats {
        self.stats
            .lock()
            .map(|mut stats| std::mem::take(&mut stats.transmit))
            .unwrap_or_default()
    }
}

impl Drop for BusHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
        if let Some(reader) = self.reader.take() {
            let _ = reader.join();
        }
    }
}

fn transient(e: &io::Error) -> bool {
    matches!(
        e.kind(),
        io::ErrorKind::Interrupted | io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
    )
}

#[cfg(test)]
pub(crate) use tests::paused_hold_bus;

#[cfg(test)]
mod tests {
    use super::*;

    const TEST_IO_BUDGET: Duration = Duration::from_millis(20);

    #[test]
    fn baseline_tuning_keeps_encoded_commands_and_disabled_packets_unchanged() {
        let now = Instant::now();
        let values = [Setpoint {
            mode: DriveMode::Servo,
            q: 0.5,
            dq: 0.05,
            tau: 0.3,
            kp: 60.0,
            kd: 10.0,
        }; 7];
        let mut reference = Reference::joint(now, [0.5; 7], [0.05; 7], [0.0; 7], 1, 0);
        // Nonzero coefficients are inert while the global switch is off.
        for joint in &mut reference.servo_tuning.joints {
            joint.coulomb = 0.4;
            joint.viscous = 1.0;
        }
        let published = PublishedSetpoints {
            values,
            seq: 1,
            reference: Some(reference),
        };
        let prepared = published
            .prepare(now + Duration::from_millis(20), None)
            .unwrap();
        for side in ["left", "right"] {
            let calibration = calibrations(side).unwrap();
            for (i, cal) in calibration.iter().enumerate() {
                let baseline = Setpoint {
                    q: 0.501,
                    ..values[i]
                };
                assert_eq!(
                    encode(i as u8, cal.command(prepared.values[i])).unwrap(),
                    encode(i as u8, cal.command(baseline)).unwrap()
                );
            }
        }
        assert_eq!(prepared.tuning.unwrap().friction_applied, [0.0; 7]);
        let direct = PublishedSetpoints {
            values: stopped_setpoints(),
            ..published
        }
        .prepare(now + Duration::from_secs(1), None)
        .unwrap();
        assert!(direct.tuning.is_none());
        assert_eq!(direct.values.map(|v| v.mode), [DriveMode::Disabled; 7]);
    }

    #[test]
    fn zero_velocity_feedforward_does_not_change_position_sampling_or_hold_friction() {
        let now = Instant::now();
        let values = [Setpoint {
            mode: DriveMode::Servo,
            q: 0.5,
            dq: 0.05,
            tau: 0.3,
            kp: 60.0,
            kd: 10.0,
        }; 7];
        let mut reference = Reference::joint(now, [0.5; 7], [0.05; 7], [0.0; 7], 1, 0);
        reference.servo_tuning.friction_enabled = true;
        reference.tuning_revision = 4;
        for (i, joint) in reference.servo_tuning.joints.iter_mut().enumerate() {
            joint.velocity_ff = 0.0;
            joint.kd = 3.0 + i as f64;
            joint.coulomb = 0.2;
            joint.viscous = 1.0;
            joint.velocity_scale = 0.01;
        }
        let published = PublishedSetpoints {
            values,
            seq: 2,
            reference: Some(reference),
        };
        let at = now + Duration::from_millis(20);
        let first = published.prepare(at, None).unwrap();
        assert_eq!(first.values.map(|s| s.dq), [0.0; 7]);
        assert_eq!(first.motion_velocity, [0.05; 7]);
        assert_eq!(first.values.map(|s| s.q), [0.501; 7]);
        assert!(first.values.iter().all(|s| s.tau > 0.54 && s.tau < 0.55));
        assert_eq!(
            first.values.map(|s| s.kd),
            [3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]
        );
        let committed = first.committed(at);
        let next = published
            .prepare(now + Duration::from_millis(30), Some(committed))
            .unwrap();
        assert_trace_numbers(next.values.map(|s| s.q), [0.5015; 7]);
        let held = published
            .prepare(now + Duration::from_millis(60), Some(committed))
            .unwrap();
        assert_eq!(held.rejection, Some(SampleStatus::Stale));
        assert_eq!(held.values.map(|s| s.q), [0.501; 7]);
        assert_eq!(held.values.map(|s| s.dq), [0.0; 7]);
        assert_eq!(held.values.map(|s| s.tau), [0.3; 7]);
        assert_eq!(held.motion_velocity, [0.0; 7]);
        let details = held.tuning.unwrap();
        assert_eq!(details.revision, 4);
        assert_eq!(details.friction_applied, [0.0; 7]);
    }

    #[test]
    fn tuned_transmit_records_the_trajectory_feedforward_and_successful_motion_velocity() {
        let directory = tempfile::tempdir().unwrap();
        let journal = Journal::start(directory.path().join("events.db")).unwrap();
        let trace = journal
            .trace_start(20_000, "right", Some("memory-only"))
            .unwrap();
        let (bus, wire) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            ..Wire::default()
        });
        let handle = BusHandle::start_recorded(bus, Some(journal.clone())).unwrap();
        let now = Instant::now();
        let mut reference = Reference::joint(now, [0.5; 7], [0.05; 7], [0.0; 7], 1, 0);
        reference.tuning_revision = 7;
        reference.servo_tuning.friction_enabled = true;
        for joint in &mut reference.servo_tuning.joints {
            joint.velocity_ff = 0.0;
            joint.coulomb = 0.2;
        }
        let values = [Setpoint {
            mode: DriveMode::Servo,
            q: 0.5,
            dq: 0.05,
            tau: 0.3,
            kp: 60.0,
            kd: 10.0,
        }; 7];
        handle.publish(values, 77, reference).unwrap();
        wait_for_protocol(|| {
            handle
                .latest_reference()
                .is_some_and(|sent| sent.q[0] > 0.5)
        });
        let sent = handle.latest_reference().unwrap();
        assert_eq!(sent.dq, [0.05; 7]);
        let bytes = wire.lock().unwrap().written.clone();
        let servo: Vec<_> = bytes
            .as_chunks::<COMMAND_LEN>()
            .0
            .iter()
            .filter(|p| (p[2] >> 4) & 7 == 1)
            .collect();
        assert!(servo.len() >= 7);
        assert!(servo.iter().all(|p| p[6..8] == [0, 0]));
        handle.disable().unwrap();
        let recording = finish_trace(&journal, &trace.id);
        let transmit = recording
            .samples
            .iter()
            .find_map(|s| match s.data {
                TraceData::Transmit {
                    seq: 77,
                    tuning: Some(t),
                    dq_ref,
                    dq_wire,
                    tau_ff,
                    ..
                } => Some((t, dq_ref, dq_wire, tau_ff)),
                _ => None,
            })
            .unwrap();
        assert_eq!(transmit.0.revision, 7);
        assert_eq!(transmit.0.dq_feedforward, [0.0; 7]);
        assert_eq!(transmit.1, [0.05; 7]);
        assert_eq!(transmit.2, [0.0; 7]);
        assert!(transmit.3.iter().all(|t| *t > 0.49));
        assert_eq!(transmit.0.tau_gravity, [0.3; 7]);
    }

    #[test]
    fn tuned_cartesian_sends_keep_the_tcp_path_and_brake_to_zero_friction() {
        use crate::{
            kinematics::{
                CartesianPath, Frame, Kinematics, LocalTarget, MotionLimits, Pose, SmoothingState,
                StepInput,
            },
            reference::CartesianGuard,
            trajectory::{JOINT_ACCELERATION, JOINT_JERK, JOINT_SPEED},
        };
        for stribeck in [false, true] {
            let friction_model = crate::tuning::StribeckTuning {
                enabled: stribeck,
                positive: crate::tuning::FrictionCurve {
                    coulomb: 0.3,
                    static_friction: 0.5,
                    viscous: 1.0,
                    stribeck_velocity: 0.03,
                },
                negative: crate::tuning::FrictionCurve {
                    coulomb: 0.2,
                    static_friction: 0.6,
                    viscous: 1.1,
                    stribeck_velocity: 0.04,
                },
            };
            let origin = Instant::now();
            let tcp = Pose {
                translation: [0.0, 0.0, 0.1],
                ..Pose::identity()
            };
            for side in ["left", "right"] {
                let model = Arc::new(Kinematics::load(side).unwrap());
                for frame in [Frame::Base, Frame::Tool] {
                    for axis in 0..6 {
                        let mut q = [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3];
                        let (mut dq, mut ddq) = ([0.0; 7], [0.0; 7]);
                        let path = CartesianPath {
                            origin: model.fk(&q, &tcp).unwrap(),
                            axis,
                            frame,
                        };
                        let (mut baseline, mut tuned) = (None, None);
                        let mut peak_friction = 0.0_f64;
                        for tick in 0..150 {
                            let stopping = tick >= 60;
                            let mut twist = [0.0; 6];
                            if !stopping {
                                twist[axis] = if axis < 3 { 0.014 } else { 0.14 };
                            }
                            let step = model
                                .step(StepInput {
                                    planning: None,
                                    reference: q,
                                    measured: q,
                                    previous_velocity: dq,
                                    target: LocalTarget::Velocity { twist, frame },
                                    dt: 0.01,
                                    tcp,
                                    limits: MotionLimits {
                                        joint_velocity: [JOINT_SPEED; 7],
                                        joint_acceleration: [JOINT_ACCELERATION; 7],
                                        linear_speed: 0.015,
                                        angular_speed: 0.15,
                                    },
                                    smoothing: Some(SmoothingState {
                                        hold: None,
                                        acceleration: ddq,
                                        jerk: [JOINT_JERK; 7],
                                        path: Some(path),
                                        stopping,
                                    }),
                                })
                                .unwrap();
                            (q, dq, ddq) = (step.q, step.dq, step.ddq);
                            let at = origin + Duration::from_millis(tick * 10);
                            let mut reference = Reference::joint(at, q, dq, ddq, 1, 0);
                            reference.stopping = stopping;
                            reference.cartesian = Some(CartesianGuard {
                                model: model.clone(),
                                tcp,
                                path,
                                goal: None,
                            });
                            let values = std::array::from_fn(|i| Setpoint {
                                mode: DriveMode::Servo,
                                q: q[i],
                                dq: dq[i],
                                tau: 0.3,
                                kp: 60.0,
                                kd: 10.0,
                            });
                            let at = at + Duration::from_millis(2);
                            let normal = PublishedSetpoints {
                                values,
                                seq: tick,
                                reference: Some(reference.clone()),
                            }
                            .prepare(at, baseline)
                            .unwrap();
                            reference.servo_tuning.friction_enabled = true;
                            for (i, joint) in reference.servo_tuning.joints.iter_mut().enumerate() {
                                joint.kd = 3.0 + i as f64;
                                joint.velocity_ff = 0.0;
                                joint.coulomb = 0.3;
                                joint.viscous = 1.0;
                                joint.stribeck = friction_model;
                            }
                            let actual = PublishedSetpoints {
                                values,
                                seq: tick,
                                reference: Some(reference),
                            }
                            .prepare(at, tuned)
                            .unwrap();
                            assert!(normal.rejection.is_none() && actual.rejection.is_none());
                            assert_eq!(actual.values.map(|s| s.q), normal.values.map(|s| s.q));
                            assert_eq!(actual.motion_velocity, normal.motion_velocity);
                            assert_eq!(actual.values.map(|s| s.dq), [0.0; 7]);
                            let (wire_q, _) = test_wire_targets(
                                side,
                                actual.values.map(|s| s.q),
                                actual.values.map(|s| s.dq),
                            );
                            path.check(&model.fk(&wire_q, &tcp).unwrap()).unwrap();
                            let friction = actual.tuning.unwrap().friction_applied;
                            peak_friction = friction
                                .into_iter()
                                .fold(peak_friction, |peak, f| peak.max(f.abs()));
                            baseline = Some(normal.committed(at));
                            tuned = Some(actual.committed(at));
                        }
                        assert!(peak_friction > 0.01);
                        let held = tuned.unwrap();
                        assert!(held.motion_velocity.iter().all(|v| v.abs() < 1e-5));
                        // The controller finishes its brake with a zero-velocity
                        // hold. No previous friction is carried into that hold.
                        let mut idle = Reference::joint(
                            held.at,
                            held.values.map(|s| s.q),
                            [0.0; 7],
                            [0.0; 7],
                            1,
                            0,
                        );
                        idle.servo_tuning.friction_enabled = true;
                        for joint in &mut idle.servo_tuning.joints {
                            joint.coulomb = 0.3;
                            joint.viscous = 1.0;
                            joint.stribeck = friction_model;
                            joint.velocity_ff = 0.0;
                        }
                        let rest = PublishedSetpoints {
                            values: held.values.map(|s| Setpoint { tau: 0.3, ..s }),
                            seq: 151,
                            reference: Some(idle),
                        };
                        let rest = rest.prepare(held.at + WRITE_PERIOD, Some(held)).unwrap();
                        assert_eq!(rest.motion_velocity, [0.0; 7]);
                        assert_eq!(rest.tuning.unwrap().friction_applied, [0.0; 7]);
                        assert_eq!(rest.values.map(|s| s.tau), [0.3; 7]);
                    }
                }
            }
        }
    }

    #[test]
    fn deadline_grid_does_not_accumulate_wakeup_delay_or_catch_up() {
        let origin = Instant::now();
        let mut scheduled = origin;
        for tick in 0..1000 {
            let started = scheduled + Duration::from_micros(676);
            scheduled = next_deadline(scheduled, started, started + Duration::from_micros(1856));
            assert_eq!(scheduled, origin + WRITE_PERIOD * (tick + 1));
        }
        let started = scheduled + Duration::from_millis(37);
        let finished = started + Duration::from_millis(19);
        let next = next_deadline(scheduled, started, finished);
        assert!(next > finished);
        assert!(next >= started + WRITE_PERIOD / 2);
        assert_eq!(next.duration_since(origin).as_millis() % 10, 0);
        for late_ms in 0..10 {
            for write_ms in [0, 1, 5, 20] {
                let started = origin + Duration::from_millis(late_ms);
                let finished = started + Duration::from_millis(write_ms);
                let next = next_deadline(origin, started, finished);
                assert!(next >= started + WRITE_PERIOD / 2);
                assert!(next > finished);
                assert_eq!(next.duration_since(origin).as_millis() % 10, 0);
            }
        }
        // An out-of-band clear can finish before an already scheduled slot.
        // Keep that slot if the ordinary minimum spacing still fits.
        assert_eq!(
            next_deadline(
                origin + WRITE_PERIOD,
                origin,
                origin + Duration::from_millis(1)
            ),
            origin + WRITE_PERIOD
        );
    }

    #[test]
    fn stopped_epoch_cannot_commit_a_prepared_old_motion() {
        let now = Instant::now();
        let epoch = AtomicU64::new(4);
        let values = [Setpoint {
            mode: DriveMode::Servo,
            q: 0.5,
            dq: 0.02,
            tau: 0.1,
            kp: 60.0,
            kd: 10.0,
        }; 7];
        let published = PublishedSetpoints {
            values,
            seq: 12,
            reference: Some(Reference::joint(now, [0.5; 7], [0.02; 7], [0.0; 7], 3, 4)),
        };
        assert!(published.is_current(&epoch, now));
        let prepared = published
            .prepare(now + Duration::from_millis(10), None)
            .unwrap();
        assert!(prepared.values[0].q > 0.5);
        epoch.fetch_add(1, Ordering::AcqRel);
        assert!(!published.is_current(&epoch, now));
    }

    #[test]
    fn prepared_targets_and_timing_are_the_values_sent_for_the_current_time() {
        let now = Instant::now();
        let values = [Setpoint {
            mode: DriveMode::Servo,
            q: 0.5,
            dq: 0.05,
            tau: 0.3,
            kp: 60.0,
            kd: 10.0,
        }; 7];
        let published = PublishedSetpoints {
            values,
            seq: 7,
            reference: Some(Reference::joint(now, [0.5; 7], [0.05; 7], [0.0; 7], 1, 0)),
        };
        let prepared = published
            .prepare(now + Duration::from_millis(20), None)
            .unwrap();
        assert_eq!(prepared.values.map(|s| s.q), [0.501; 7]);
        assert_eq!(prepared.values.map(|s| s.dq), [0.05; 7]);
        assert_eq!(prepared.values.map(|s| s.tau), [0.3; 7]);
        let timing = prepared.timing.unwrap();
        assert_eq!(timing.age_us, Some(20_000));
        assert_eq!(timing.status, SampleStatus::Aligned);
        let disabled = PublishedSetpoints {
            values: stopped_setpoints(),
            seq: 8,
            reference: published.reference,
        };
        let direct = disabled
            .prepare(now + Duration::from_secs(1), None)
            .unwrap();
        assert!(direct.values.iter().all(|s| s.mode == DriveMode::Disabled));
        assert!(direct.timing.is_none());
    }

    #[test]
    fn teaching_absolute_sources_keep_gravity_and_damping_through_gain_changes_and_wire_encoding() {
        let origin = Instant::now();
        let model = crate::kinematics::Kinematics::load("right").unwrap();
        let calibration = calibrations("right").unwrap();
        let initial = [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3];
        let mut guidance = crate::teaching::Guidance::new(initial, 0.4);
        guidance.set_drag(true, initial, 0.0);
        let mut committed = None;
        let mut last = None;
        for tick in 0..=150 {
            let t = tick as f64 * 0.01;
            let mut actual = initial;
            actual[1] += 0.05 * t.min(1.0);
            if tick == 100 {
                guidance.set_drag(false, actual, t);
            }
            let target = guidance.sample(t, actual);
            let gravity = model.gravity(&target.q).unwrap();
            let values = std::array::from_fn(|i| Setpoint {
                mode: DriveMode::Servo,
                q: target.q[i],
                dq: 0.0,
                tau: gravity[i],
                kp: 60.0 * target.stiffness,
                kd: 10.0,
            });
            let at = origin + Duration::from_millis(tick * 10);
            let mut reference = Reference::joint(at, target.q, [0.0; 7], [0.0; 7], 1, 0);
            reference.sampling = crate::reference::Sampling::Absolute;
            reference.hold_feedback = Some(actual.map(Some));
            let published = PublishedSetpoints {
                values,
                seq: tick,
                reference: Some(reference),
            };
            let prepared = published.prepare(at, committed).unwrap();
            assert_eq!(
                prepared.values.map(|s| s.q),
                target.q,
                "old send integration must not pin hand guidance to its initial position"
            );
            assert!(
                prepared
                    .values
                    .iter()
                    .all(|s| s.mode == DriveMode::Servo && s.kd == 10.0)
            );
            assert_eq!(prepared.values.map(|s| s.tau), gravity);
            assert!(gravity.iter().any(|tau| tau.abs() > 0.1));
            // J1 never leaves its target; its hold and drag curves compose.
            assert!((prepared.values[0].kp - 30.0 * target.stiffness).abs() < 1e-10);
            for (i, s) in prepared.values.into_iter().enumerate() {
                let packet = encode(i as u8, calibration[i].command(s)).unwrap();
                assert_eq!(packet.len(), COMMAND_LEN);
            }
            committed = Some(prepared.committed(at));
            last = Some(published);
        }
        let committed = committed.unwrap();
        let held = last
            .unwrap()
            .prepare(
                committed.at + crate::reference::MAX_REFERENCE_AGE + Duration::from_millis(1),
                Some(committed),
            )
            .unwrap();
        assert_eq!(held.rejection, Some(SampleStatus::Stale));
        assert_eq!(held.values.map(|s| s.tau), committed.values.map(|s| s.tau));
        assert!(
            held.values
                .iter()
                .all(|s| s.mode == DriveMode::Servo && s.kp == 30.0 && s.kd == 10.0)
        );
        assert_ne!(held.values.map(|s| s.q), initial);
    }

    #[test]
    fn powered_hold_changes_only_the_position_gain_on_the_wire() {
        let at = Instant::now();
        let q = [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3];
        let errors = [0.0_f64, 1.0, -1.0, 2.0, -2.0, 6.0, -6.0];
        for side in ["left", "right"] {
            let model = crate::kinematics::Kinematics::load(side).unwrap();
            let gravity = model.gravity(&q).unwrap();
            let values = std::array::from_fn(|i| Setpoint {
                mode: DriveMode::Servo,
                q: q[i],
                dq: 0.0,
                tau: gravity[i],
                kp: 60.0,
                kd: 10.0,
            });
            let mut reference =
                Reference::joint_for_model(at, q, [0.0; 7], [0.0; 7], 1, 0, Some(&model));
            reference.sampling = Sampling::Absolute;
            reference.hold_feedback =
                Some(std::array::from_fn(|i| Some(q[i] - errors[i].to_radians())));
            let source = PublishedSetpoints {
                values,
                seq: 0,
                reference: Some(reference),
            };
            let prepared = source.prepare(at, None).unwrap();
            let calibration = calibrations(side).unwrap();
            for (i, expected) in [30.0, 40.0, 40.0, 45.0, 45.0, 52.5, 52.5]
                .into_iter()
                .enumerate()
            {
                let mut desired = values[i];
                desired.kp = expected;
                assert_eq!(
                    encode(i as u8, calibration[i].command(prepared.values[i])).unwrap(),
                    encode(i as u8, calibration[i].command(desired)).unwrap(),
                    "{side} J{}",
                    i + 1
                );
            }
            // A late J7 keeps its own gain while fresh J6 feedback returns to the anchor.
            let mut next = source.clone();
            let positions = next
                .reference
                .as_mut()
                .unwrap()
                .hold_feedback
                .as_mut()
                .unwrap();
            positions[5] = Some(q[5]);
            positions[6] = None;
            let next = next
                .prepare(at + WRITE_PERIOD, Some(prepared.committed(at)))
                .unwrap();
            assert!(next.values[5].kp < prepared.values[5].kp);
            assert_eq!(next.values[6].kp, prepared.values[6].kp);
        }
    }

    #[test]
    fn hold_gain_follows_the_sent_target_and_blends_only_from_successful_writes() {
        let start = Instant::now();
        let mut reference = Reference::joint(start, [0.0; 7], [0.0; 7], [0.0; 7], 1, 0);
        reference.hold_feedback = Some([Some(0.0); 7]);
        let mut source = PublishedSetpoints {
            values: [Setpoint {
                mode: DriveMode::Servo,
                q: 0.0,
                dq: 0.0,
                tau: 1.0,
                kp: 60.0,
                kd: 10.0,
            }; 7],
            seq: 0,
            reference: Some(reference),
        };
        let first = source.prepare(start, None).unwrap();
        assert_eq!(first.values[0].kp, 30.0);
        let mut last = first.committed(start);
        let nominal = WRITE_PERIOD;
        // An integrated hold keeps the last wire position even if control q differs.
        let at = start + nominal;
        source.reference.as_mut().unwrap().q = [1.0; 7];
        let held = source.prepare(at, Some(last)).unwrap();
        assert_eq!((held.values[0].q, held.values[0].kp), (0.0, 30.0));
        last = held.committed(at);
        source.reference.as_mut().unwrap().hold_feedback = None;
        let mut at = at + Duration::from_millis(300);
        source.reference.as_mut().unwrap().at = at;
        let resumed = source.prepare(at, Some(last)).unwrap();
        assert!((resumed.values[0].kp - 33.0).abs() < 1e-10);
        assert_eq!(
            source.prepare(at, Some(last)).unwrap().values[0].kp,
            resumed.values[0].kp
        );
        last = resumed.committed(at);
        for _ in 0..9 {
            at += nominal;
            source.reference.as_mut().unwrap().at = at;
            last = source.prepare(at, Some(last)).unwrap().committed(at);
        }
        assert_eq!(last.values[0].kp, 60.0);
        // The receiver can observe a fresh enable before TX sees a disabled batch.
        // A stale retry must not consume that new session's first valid hold.
        let mut reenabled = source.clone();
        let new_ref = reenabled.reference.as_mut().unwrap();
        new_ref.enable_epoch += 1;
        new_ref.sampling = Sampling::Absolute;
        new_ref.hold_feedback = Some(new_ref.q.map(Some));
        let late_at = at + Duration::from_millis(100);
        let retry = reenabled.prepare(late_at, Some(last)).unwrap();
        assert_eq!(retry.rejection, Some(SampleStatus::Stale));
        assert_eq!(retry.enable_epoch, last.enable_epoch);
        reenabled.reference.as_mut().unwrap().at = late_at + nominal;
        let fresh = reenabled
            .prepare(late_at + nominal, Some(retry.committed(late_at)))
            .unwrap();
        assert_eq!(fresh.values[0].kp, 30.0);
        assert_eq!(fresh.values[0].q, 1.0);
        source.reference.as_mut().unwrap().hold_feedback = Some([Some(0.0); 7]);
        at += nominal;
        source.reference.as_mut().unwrap().at = at;
        let settling = source.prepare(at, Some(last)).unwrap();
        assert!((settling.values[0].kp - 57.0).abs() < 1e-10);
        last = settling.committed(at);
        for _ in 0..2 {
            at += Duration::from_millis(100);
            let stale = source.prepare(at, Some(last)).unwrap();
            assert_eq!(stale.rejection, Some(SampleStatus::Stale));
            assert_eq!(stale.values[0].kp, last.values[0].kp);
            last = stale.committed(at);
        }
        let disabled = Committed::direct(at, stopped_setpoints());
        at += nominal;
        source.reference.as_mut().unwrap().q = [0.0; 7];
        source.reference.as_mut().unwrap().at = at;
        assert_eq!(
            source.prepare(at, Some(disabled)).unwrap().values[0].kp,
            30.0
        );
    }

    #[test]
    fn stale_reference_holds_only_a_successfully_committed_servo_target() {
        let now = Instant::now();
        let values = [Setpoint {
            mode: DriveMode::Servo,
            q: 0.5,
            dq: 0.1,
            tau: 0.2,
            kp: 60.0,
            kd: 10.0,
        }; 7];
        let published = PublishedSetpoints {
            values,
            seq: 9,
            reference: Some(Reference::joint(now, [0.5; 7], [0.1; 7], [0.0; 7], 1, 0)),
        };
        let at = now + Duration::from_millis(60);
        assert!(published.prepare(at, None).is_err());
        let committed = Committed::direct(
            now + Duration::from_millis(10),
            values.map(|s| Setpoint { q: 0.501, ..s }),
        );
        let held = published.prepare(at, Some(committed)).unwrap();
        assert_eq!(held.values.map(|s| s.q), [0.501; 7]);
        assert_eq!(held.values.map(|s| s.dq), [0.0; 7]);
        assert_eq!(held.timing.unwrap().status, SampleStatus::Stale);
        let later = published
            .prepare(at + Duration::from_secs(1), Some(committed))
            .unwrap();
        assert_eq!(later.values.map(|s| s.q), [0.501; 7]);
        let mut fresh = published.clone();
        fresh.reference.as_mut().unwrap().at = at;
        let resumed = fresh
            .prepare(at + WRITE_PERIOD, Some(held.committed(at)))
            .unwrap();
        assert!(resumed.rejection.is_none());
        assert!(resumed.values[0].q > held.values[0].q);
        // An initial disabled target is not an observed servo position.
        assert!(
            published
                .prepare(at, Some(Committed::direct(now, stopped_setpoints())))
                .is_err()
        );
    }

    #[test]
    fn timed_bus_skips_canceled_motion_and_disable_cannot_be_revived() {
        let (bus, wire) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            ..Wire::default()
        });
        let epoch = Arc::new(AtomicU64::new(0));
        let handle = BusHandle::start_with_epoch(bus, None, epoch.clone()).unwrap();
        let values = [Setpoint {
            mode: DriveMode::Servo,
            q: 0.5,
            dq: 0.02,
            tau: 0.1,
            kp: 60.0,
            kd: 10.0,
        }; 7];
        handle
            .publish(
                values,
                10,
                Reference::joint(Instant::now(), [0.5; 7], [0.02; 7], [0.0; 7], 1, 0),
            )
            .unwrap();
        let servo_count = || {
            wire.lock()
                .unwrap()
                .written
                .as_chunks::<COMMAND_LEN>()
                .0
                .iter()
                .filter(|p| (p[2] >> 4) & 7 == 1)
                .count()
        };
        wait_for_protocol(|| servo_count() >= 7);
        epoch.fetch_add(1, Ordering::AcqRel);
        // An already-started seven-packet batch may finish. No old snapshot
        // may start another batch after the bounded in-flight write completes.
        thread::sleep(TEST_IO_BUDGET + WRITE_PERIOD);
        let canceled_count = wire.lock().unwrap().written.len();
        thread::sleep(WRITE_PERIOD * 2);
        assert_eq!(wire.lock().unwrap().written.len(), canceled_count);
        let before = servo_count();
        let held = values.map(|s| Setpoint { dq: 0.0, ..s });
        handle
            .publish(
                held,
                11,
                Reference::joint(Instant::now(), [0.5; 7], [0.0; 7], [0.0; 7], 2, 1),
            )
            .unwrap();
        wait_for_protocol(|| servo_count() >= before + 7);
        handle.disable().unwrap();
        assert!(handle.latest_reference().is_none());
        let disabled_count = wire.lock().unwrap().written.len();
        handle
            .publish(
                values,
                12,
                Reference::joint(Instant::now(), [0.5; 7], [0.02; 7], [0.0; 7], 3, 1),
            )
            .unwrap();
        thread::sleep(WRITE_PERIOD * 2);
        assert_eq!(wire.lock().unwrap().written.len(), disabled_count);
        assert!(handle.drain().1.is_none());
    }
    use serde_json::Value;
    use std::collections::VecDeque;
    use std::io::{Read, Write};
    use std::sync::{Arc, Mutex};

    fn golden() -> Value {
        serde_json::from_str(include_str!(
            "../../resources/calibration/protocol-golden.json"
        ))
        .unwrap()
    }

    fn unhex(s: &str) -> Vec<u8> {
        s.as_bytes()
            .as_chunks::<2>()
            .0
            .iter()
            .map(|b| u8::from_str_radix(std::str::from_utf8(b).unwrap(), 16).unwrap())
            .collect()
    }

    fn feedback_for_id(base: &[u8], id: u8) -> Vec<u8> {
        let mut frame = base.to_vec();
        frame[2] = (frame[2] & 0xf0) | id;
        let crc = crc32(&frame[2..22]);
        frame[22..26].copy_from_slice(&crc.to_le_bytes());
        frame
    }

    fn stop() -> Setpoint {
        Setpoint {
            mode: DriveMode::Disabled,
            q: 0.,
            dq: 0.,
            tau: 0.,
            kp: 0.,
            kd: 0.,
        }
    }

    #[test]
    fn exact_duplicate_ports_collapse() {
        let port = PortInfo {
            name: "/dev/cu.usbserial-A".into(),
            label: "/dev/cu.usbserial-A".into(),
            kind: "usb".into(),
        };
        assert_eq!(dedupe_ports(vec![port.clone(), port]).len(), 1);
    }

    #[cfg(target_os = "macos")]
    #[test]
    fn macos_cu_and_tty_are_one_device_and_cu_wins() {
        let ports = vec![
            PortInfo {
                name: "/dev/tty.usbserial-A".into(),
                label: "/dev/tty.usbserial-A — FTDI [0403:6001]".into(),
                kind: "usb".into(),
            },
            PortInfo {
                name: "/dev/cu.usbserial-A".into(),
                label: "/dev/cu.usbserial-A".into(),
                kind: "unknown".into(),
            },
            PortInfo {
                name: "/dev/cu.Bluetooth-Incoming-Port".into(),
                label: "/dev/cu.Bluetooth-Incoming-Port".into(),
                kind: "bluetooth".into(),
            },
        ];
        let deduped = dedupe_ports(ports);
        assert_eq!(deduped.len(), 2);
        let usb = deduped
            .iter()
            .find(|port| port.name.contains("usbserial-A"))
            .unwrap();
        assert_eq!(usb.name, "/dev/cu.usbserial-A");
        assert_eq!(usb.kind, "usb");
        assert!(usb.label.contains("FTDI"));
    }

    #[test]
    fn official_cpp_control_golden_positive_negative_and_zero_for_each_ratio() {
        let oracle = golden();
        assert_eq!(
            oracle["sdk_commit"],
            "0803d033b530fcfa0e6fa9236de68c2773fc9402"
        );
        for c in oracle["control"].as_array().unwrap() {
            let s = Setpoint {
                mode: DriveMode::Servo,
                q: c["q"].as_f64().unwrap(),
                dq: c["dq"].as_f64().unwrap(),
                tau: c["tau"].as_f64().unwrap(),
                kp: c["kp"].as_f64().unwrap(),
                kd: c["kd"].as_f64().unwrap(),
            };
            let id = c["id"].as_u64().unwrap() as u8;
            assert_eq!(
                encode(id, s).unwrap().as_slice(),
                unhex(c["packet"].as_str().unwrap()),
                "{c}"
            );
        }
    }

    #[test]
    fn official_cpp_disabled_and_clear_do_not_enable_or_use_calibrated_zero() {
        let g = golden();
        let mut s = stop();
        // These fields MUST be ignored for Disabled/ClearFault.
        s.q = f64::NAN;
        s.tau = 100.;
        assert_eq!(
            encode(6, s).unwrap().as_slice(),
            unhex(g["disabled"].as_str().unwrap())
        );
        s.mode = DriveMode::ClearFault;
        assert_eq!(
            encode(6, s).unwrap().as_slice(),
            unhex(g["clear_fault"].as_str().unwrap())
        );
    }

    #[test]
    fn official_cpp_feedback_crc_and_signed_extrema() {
        for expected in golden()["feedback"].as_array().unwrap() {
            let bytes = unhex(expected["packet"].as_str().unwrap());
            let f = decode(&bytes).unwrap();
            assert_eq!(f.id as u64, expected["id"].as_u64().unwrap());
            // The official golden packet generator sets byte 4 to 101 °C.
            assert_eq!(f.winding_temperature, 101.0);
            for (key, actual) in [
                ("q", f.q),
                ("dq", f.dq),
                ("tau", f.tau),
                ("temperature", f.temperature),
                ("voltage", f.voltage),
            ] {
                assert_eq!(actual, expected[key].as_f64().unwrap(), "{key}: {expected}");
            }
            assert_eq!(f.error as u64, expected["error"].as_u64().unwrap());
            assert_eq!(f.warning as u64, expected["warning"].as_u64().unwrap());
            assert_eq!(f.mode as u64, expected["mode"].as_u64().unwrap());
            assert_eq!(f.timeout, expected["timeout"] == 1);
            // Corruption anywhere in the protected payload or CRC is rejected.
            for i in 2..26 {
                let mut damaged = bytes.clone();
                damaged[i] ^= 1;
                assert!(decode(&damaged).is_none(), "corruption offset {i}");
            }
        }
    }

    #[test]
    fn motor_fault_description_decodes_single_combined_and_unknown_bits() {
        assert_eq!(motor_fault_description(0), "正常");
        assert_eq!(motor_fault_description(512), "电机绕组过热");
        assert_eq!(motor_fault_description(8), "供电电压短时过低");
        assert_eq!(
            motor_fault_description(512 | 8),
            "供电电压短时过低、电机绕组过热（0x00000208）"
        );
        assert_eq!(motor_fault_description(0x80), "未知故障位 0x00000080");
        assert_eq!(
            motor_fault_description(0x8000_0200),
            "电机绕组过热、未知故障位 0x80000000（0x80000200）"
        );
    }

    #[test]
    fn split_at_every_byte_multiframe_noise_and_bad_crc_recover() {
        let g = golden();
        let a = unhex(g["feedback"][0]["packet"].as_str().unwrap());
        let b = unhex(g["feedback"][3]["packet"].as_str().unwrap());
        for split in 1..FEEDBACK_LEN {
            let mut r = Receiver::default();
            let mut stats = BusStats::default();
            assert!(r.feed(&a[..split], &mut stats).is_empty());
            assert_eq!(r.feed(&a[split..], &mut stats).len(), 1);
            assert_eq!(stats.received, 1);
        }
        let mut bad = a.clone();
        bad[17] ^= 0x80;
        let mut data = vec![0x55, 0xfc, 0xfc, 0xee, 0xfc];
        data.extend(bad);
        data.extend(&a);
        data.extend(&b);
        data.extend([0xfc, 0xee]);
        let mut r = Receiver::default();
        let mut stats = BusStats::default();
        let frames = r.feed(&data, &mut stats);
        assert_eq!(frames.iter().map(|f| f.id).collect::<Vec<_>>(), [0, 2]);
        assert_eq!(stats.received, 2);
        assert!(stats.crc_errors >= 1);
        assert_eq!(r.bytes, [0xfc, 0xee]);
        assert_eq!(r.feed(&a[2..], &mut stats).len(), 1);
        // Long noise containing false headers cannot grow retained state.
        r.feed(&vec![0xfc; 100_000], &mut stats);
        assert!(r.bytes.len() < FEEDBACK_LEN);
        assert_eq!(r.feed(&a, &mut stats).len(), 1);
    }

    #[test]
    fn official_calibration_both_sides_direction_offsets_and_roundtrip() {
        for (side, signs) in [
            ("right", [-1., 1., 1., -1., -1., 1., -1.]),
            ("left", [1., -1., -1., 1., 1., -1., 1.]),
        ] {
            let cal = calibrations(side).unwrap();
            for i in 0..7 {
                assert_eq!(cal[i].gain, signs[i]);
                let s = Setpoint {
                    mode: DriveMode::Servo,
                    q: 0.25,
                    dq: -0.5,
                    tau: 1.5,
                    kp: 20.,
                    kd: 1.,
                };
                let motor = cal[i].command(s);
                assert_eq!(motor.dq, -0.5 / signs[i] as f64);
                assert_eq!(motor.tau, 1.5 / signs[i] as f64);
                let f = cal[i].feedback(Feedback {
                    id: i as u8,
                    q: motor.q,
                    dq: motor.dq,
                    tau: motor.tau,
                    temperature: 20.,
                    winding_temperature: 47.,
                    voltage: 48.,
                    error: 0,
                    warning: 0,
                    mode: 1,
                    timeout: false,
                });
                assert!((f.q - s.q).abs() < 3e-7);
                assert_eq!(f.dq, s.dq);
                assert_eq!(f.tau, s.tau);
            }
        }
        let right = calibrations("right").unwrap();
        let left = calibrations("left").unwrap();
        assert_eq!(
            right[0].command(Setpoint { q: 0., ..stop() }).q,
            (-2.721f32) as f64
        );
        assert_eq!(
            left[0].command(Setpoint { q: 0., ..stop() }).q,
            (-2.721f32) as f64
        );
        assert!(calibrations("RIGHT").is_err());
        assert!(parse_calibration("0 0 1 0").is_err());
        assert!(parse_calibration("0 1 NaN 0").is_err());
        assert!(parse_calibration("0 1 1 0\n0 1 1 0").is_err());
        assert!(parse_calibration("0 1 1 0").is_err());
    }

    #[test]
    fn invalid_servo_values_rejected_before_narrowing() {
        let s = Setpoint {
            mode: DriveMode::Servo,
            q: 0.,
            dq: 0.,
            tau: 0.,
            kp: 10.,
            kd: 1.,
        };
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, f64::MAX] {
            assert!(encode(0, Setpoint { q: bad, ..s }).is_err());
            assert!(encode(0, Setpoint { kp: bad, ..s }).is_err());
        }
        // SDK's advertised speed/position range can overflow raw integers at r=32.
        for sign in [-1., 1.] {
            assert!(
                encode(
                    0,
                    Setpoint {
                        q: sign * 32000.,
                        ..s
                    }
                )
                .is_err()
            );
            assert!(
                encode(
                    0,
                    Setpoint {
                        dq: sign * 250.,
                        ..s
                    }
                )
                .is_err()
            );
        }
        assert!(encode(0, Setpoint { kp: -1., ..s }).is_err());
        assert!(encode(7, s).is_err());
    }

    #[derive(Default)]
    struct Wire {
        written: Vec<u8>,
        command_times: Vec<(u8, Instant)>,
        write_chunk: usize,
        fail_after: Option<usize>,
        fail_kind: Option<io::ErrorKind>,
        active: VecDeque<Vec<u8>>,
        after_tx: VecDeque<Vec<u8>>,
        after_clear: VecDeque<Vec<u8>>,
        clears: usize,
        read_error: Option<io::ErrorKind>,
        late_read: bool,
        pause_after_servo_packet: Option<(u8, Duration)>,
        hold_write: Option<(Sender<()>, EventReceiver<()>)>,
        fail_once_for_mode: Option<(u8, io::ErrorKind)>,
        board: Option<EmulatedBoard>,
    }

    /// A packet-level board emulator. Quantized encoder positions follow
    /// the received servo command with lag, independent of Core's target.
    struct EmulatedBoard {
        encoders: [i32; 7],
    }

    impl EmulatedBoard {
        fn reply(&mut self, command: &[u8]) -> Vec<u8> {
            assert_eq!(command[..2], [0xfe, 0xee]);
            assert_eq!(
                u32::from_le_bytes(command[16..20].try_into().unwrap()),
                crc32(&command[..16])
            );
            let id = command[2] & 0x0f;
            let servo = (command[2] >> 4) & 7 == 1;
            let encoder = &mut self.encoders[id as usize];
            let mut advance = 0;
            if servo {
                let target = i32::from_le_bytes(command[8..12].try_into().unwrap());
                let remaining = i64::from(target) - i64::from(*encoder);
                advance = (remaining / 3).abs().max(1) * remaining.signum();
                *encoder = (i64::from(*encoder) + advance) as i32;
            }
            let velocity = ((advance as f64 / (512.0 * WRITE_PERIOD.as_secs_f64())).round()) as i16;
            let mut reply = vec![0; FEEDBACK_LEN];
            reply[..2].copy_from_slice(&[0xfc, 0xee]);
            reply[2] = id | if servo { 0x10 } else { 0 };
            reply[3..6].copy_from_slice(&[28, 28, 48]);
            reply[8..10].copy_from_slice(&velocity.to_le_bytes());
            reply[10..14].copy_from_slice(&encoder.to_le_bytes());
            let crc = crc32(&reply[2..22]);
            reply[22..26].copy_from_slice(&crc.to_le_bytes());
            reply
        }
    }

    struct FakePort {
        wire: Arc<Mutex<Wire>>,
        timeout: Duration,
    }

    impl Read for FakePort {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            let mut w = self.wire.lock().unwrap();
            if let Some(kind) = w.read_error {
                return Err(io::Error::from(kind));
            }
            if let Some(mut bytes) = w.active.pop_front() {
                if w.late_read {
                    std::thread::sleep(Duration::from_millis(8));
                }
                let n = buf.len().min(bytes.len());
                buf[..n].copy_from_slice(&bytes[..n]);
                bytes.drain(..n);
                if !bytes.is_empty() {
                    w.active.push_front(bytes);
                }
                return Ok(n);
            }
            drop(w);
            std::thread::sleep(self.timeout.min(Duration::from_millis(1)));
            Err(io::Error::from(io::ErrorKind::TimedOut))
        }
    }

    impl Write for FakePort {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            let mut w = self.wire.lock().unwrap();
            if let Some((mode, kind)) = w.fail_once_for_mode
                && buf.len() == COMMAND_LEN
                && (buf[2] >> 4) & 7 == mode
            {
                w.fail_once_for_mode = None;
                return Err(io::Error::from(kind));
            }
            if w.fail_after.is_some_and(|n| w.written.len() >= n) {
                return Err(io::Error::from(
                    w.fail_kind.unwrap_or(io::ErrorKind::BrokenPipe),
                ));
            }
            let n = buf.len().min(w.write_chunk);
            if w.written.len().is_multiple_of(COMMAND_LEN) && buf.len() == COMMAND_LEN {
                w.command_times.push((buf[2] & 0x0f, Instant::now()));
            }
            w.written.extend_from_slice(&buf[..n]);
            if w.written.len().is_multiple_of(COMMAND_LEN) {
                if let Some(board) = &mut w.board {
                    assert_eq!(n, COMMAND_LEN);
                    let reply = board.reply(buf);
                    w.active.push_back(reply);
                }
                let staged = std::mem::take(&mut w.after_tx);
                w.active.extend(staged);
                if n == COMMAND_LEN && buf[2] == 0x66 {
                    let healthy = std::mem::take(&mut w.after_clear);
                    w.active.extend(healthy);
                }
            }
            let pause = w.pause_after_servo_packet.and_then(|(joint, pause)| {
                (n == COMMAND_LEN && buf[2] & 0x7f == (joint | 0x10)).then_some(pause)
            });
            if pause.is_some() {
                w.pause_after_servo_packet = None;
            }
            let hold = if n == COMMAND_LEN && buf[2] == 0x90 && buf[6..8] == [0, 0] {
                w.hold_write.take()
            } else {
                None
            };
            drop(w);
            if let Some((entered, release)) = hold {
                entered.send(()).unwrap();
                release
                    .recv_timeout(Duration::from_secs(3))
                    .map_err(io::Error::other)?;
            }
            // The complete packet was accepted before this scheduling gap.
            // This must not consume the next packet's I/O progress deadline.
            if let Some(pause) = pause {
                thread::sleep(pause);
            }
            Ok(n)
        }
        fn flush(&mut self) -> io::Result<()> {
            panic!("exchange must not call unbounded drain")
        }
    }

    // Unused native-port operations deliberately panic, so an accidental modem
    // signal change or unrelated I/O cannot be hidden by this test port.
    impl SerialPort for FakePort {
        fn name(&self) -> Option<String> {
            None
        }
        fn baud_rate(&self) -> serialport::Result<u32> {
            unimplemented!()
        }
        fn data_bits(&self) -> serialport::Result<DataBits> {
            unimplemented!()
        }
        fn flow_control(&self) -> serialport::Result<FlowControl> {
            unimplemented!()
        }
        fn parity(&self) -> serialport::Result<Parity> {
            unimplemented!()
        }
        fn stop_bits(&self) -> serialport::Result<StopBits> {
            unimplemented!()
        }
        fn timeout(&self) -> Duration {
            self.timeout
        }
        fn set_baud_rate(&mut self, _: u32) -> serialport::Result<()> {
            unimplemented!()
        }
        fn set_data_bits(&mut self, _: DataBits) -> serialport::Result<()> {
            unimplemented!()
        }
        fn set_flow_control(&mut self, _: FlowControl) -> serialport::Result<()> {
            unimplemented!()
        }
        fn set_parity(&mut self, _: Parity) -> serialport::Result<()> {
            unimplemented!()
        }
        fn set_stop_bits(&mut self, _: StopBits) -> serialport::Result<()> {
            unimplemented!()
        }
        fn set_timeout(&mut self, timeout: Duration) -> serialport::Result<()> {
            assert!(timeout <= IO_SLICE && !timeout.is_zero());
            self.timeout = timeout;
            Ok(())
        }
        fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> {
            unimplemented!()
        }
        fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> {
            unimplemented!()
        }
        fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
            unimplemented!()
        }
        fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
            unimplemented!()
        }
        fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
            unimplemented!()
        }
        fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
            unimplemented!()
        }
        fn bytes_to_read(&self) -> serialport::Result<u32> {
            unimplemented!()
        }
        fn bytes_to_write(&self) -> serialport::Result<u32> {
            unimplemented!()
        }
        fn clear(&self, target: ClearBuffer) -> serialport::Result<()> {
            assert_eq!(target, ClearBuffer::Input);
            let mut w = self.wire.lock().unwrap();
            w.active.clear();
            w.clears += 1;
            Ok(())
        }
        fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
            Ok(Box::new(FakePort {
                wire: self.wire.clone(),
                timeout: self.timeout,
            }))
        }
        fn set_break(&self) -> serialport::Result<()> {
            unimplemented!()
        }
        fn clear_break(&self) -> serialport::Result<()> {
            unimplemented!()
        }
    }

    fn fake_bus(wire: Wire) -> (Bus, Arc<Mutex<Wire>>) {
        let wire = Arc::new(Mutex::new(wire));
        (
            Bus {
                port: Box::new(FakePort {
                    wire: wire.clone(),
                    timeout: IO_SLICE,
                }),
                calibration: calibrations("right").unwrap(),
                receiver: Receiver::default(),
                budget: Duration::from_millis(4),
                stats: BusStats::default(),
                incomplete_write: false,
            },
            wire,
        )
    }

    pub(super) fn emulated_board(
        journal: Journal,
        epoch: Arc<AtomicU64>,
    ) -> Result<BusHandle, String> {
        let calibration = calibrations("right")?;
        let initial = [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3];
        let mut encoders = [0; 7];
        for (i, (encoder, q)) in encoders.iter_mut().zip(initial).enumerate() {
            let packet = encode(
                i as u8,
                calibration[i].command(Setpoint {
                    mode: DriveMode::Servo,
                    q,
                    ..stop()
                }),
            )?;
            *encoder = i32::from_le_bytes(packet[8..12].try_into().unwrap());
        }
        let (mut bus, _) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            board: Some(EmulatedBoard { encoders }),
            ..Wire::default()
        });
        bus.budget = TEST_IO_BUDGET;
        BusHandle::start_with_epoch(bus, Some(journal), epoch)
    }

    pub(super) fn clear_response_bus() -> BusHandle {
        let healthy = (0..7)
            .map(|id| {
                let mut packet = vec![0; FEEDBACK_LEN];
                packet[..2].copy_from_slice(&[0xfc, 0xee]);
                packet[2] = id | 0x10;
                let crc = crc32(&packet[2..22]);
                packet[22..26].copy_from_slice(&crc.to_le_bytes());
                packet
            })
            .collect();
        let (bus, _) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            after_clear: healthy,
            ..Wire::default()
        });
        BusHandle::start(bus).unwrap()
    }

    pub(crate) fn paused_hold_bus(
        epoch: Arc<AtomicU64>,
    ) -> (BusHandle, EventReceiver<()>, Sender<()>) {
        let (entered, in_flight) = bounded(1);
        let (release, wait) = bounded(1);
        let (mut bus, _) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            hold_write: Some((entered, wait)),
            ..Wire::default()
        });
        bus.budget = TEST_IO_BUDGET;
        (
            BusHandle::start_with_epoch(bus, None, epoch).unwrap(),
            in_flight,
            release,
        )
    }

    fn wait_for_protocol(mut ready: impl FnMut() -> bool) {
        let deadline = Instant::now() + Duration::from_secs(3);
        while !ready() {
            assert!(Instant::now() < deadline, "offline protocol test timed out");
            thread::sleep(Duration::from_millis(1));
        }
    }

    fn finish_trace(journal: &Journal, id: &str) -> crate::journal::TraceRecording {
        journal.trace_stop().unwrap();
        wait_for_protocol(|| journal.trace_list(1).unwrap()[0].status == "complete");
        let recording = journal.trace_read(id).unwrap();
        assert_eq!(recording.trace.errors, 0);
        assert_eq!(recording.trace.pending, 0);
        assert_eq!(recording.trace.accepted, recording.samples.len() as u64);
        assert_eq!(recording.trace.written, recording.trace.accepted);
        assert!(journal.shutdown(Duration::from_secs(2)));
        recording
    }

    fn assert_source_time(at_us: u64, source: Instant, before: Instant, after: Instant) {
        assert!(
            source_time_matches(at_us, source, before, after),
            "source timestamp was not retained"
        );
    }

    fn source_time_matches(at_us: u64, source: Instant, before: Instant, after: Instant) -> bool {
        let earliest = source.duration_since(after).as_micros() as u64;
        let latest = source.duration_since(before).as_micros() as u64;
        (earliest..=latest).contains(&at_us)
    }

    fn trace_number_matches(actual: f64, expected: f64) -> bool {
        (actual - expected).abs() <= 4.0 * f64::EPSILON * expected.abs().max(1.0)
    }

    fn assert_trace_numbers<const N: usize>(actual: [f64; N], expected: [f64; N]) {
        // Persisted JSON can round-trip with a few f64 ULPs of parser error.
        // This allowance is far smaller than either f32 or any wire LSB.
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert!(
                trace_number_matches(actual, expected),
                "trace value {actual} differs from {expected}"
            );
        }
    }

    #[test]
    fn offline_trace_links_published_sequences_wire_values_and_every_feedback_frame() {
        for side in ["right", "left"] {
            let directory = tempfile::tempdir().unwrap();
            let journal = Journal::start(directory.path().join("events.db")).unwrap();
            let before = Instant::now();
            let trace = journal
                .trace_start(20_000, side, Some("fake-port"))
                .unwrap();
            let after = Instant::now();
            let calibration = calibrations(side).unwrap();
            let g = golden();
            let base = unhex(g["feedback"][0]["packet"].as_str().unwrap());
            let duplicate = feedback_for_id(&base, 2);
            let mut changed = duplicate.clone();
            changed[4] = 0xff;
            changed[10..14].copy_from_slice(&12345i32.to_le_bytes());
            let crc = crc32(&changed[2..22]);
            changed[22..26].copy_from_slice(&crc.to_le_bytes());
            let frames = [
                duplicate.clone(),
                duplicate,
                changed,
                feedback_for_id(&base, 6),
            ];
            let mut corrupt = frames[0].clone();
            corrupt[22] ^= 1;
            let mut input = vec![0x55, 0xaa];
            input.extend(corrupt);
            input.extend(frames.iter().flatten());
            let (mut bus, wire) = fake_bus(Wire {
                write_chunk: 3,
                // Split a frame across reads; one read also contains repeated IDs.
                after_tx: VecDeque::from([input[..19].to_vec(), input[19..].to_vec()]),
                ..Wire::default()
            });
            bus.calibration = calibration;
            bus.budget = TEST_IO_BUDGET;
            let handle = BusHandle::start_recorded(bus, Some(journal.clone())).unwrap();
            let first = std::array::from_fn(|i| Setpoint {
                mode: DriveMode::Servo,
                q: 0.123456789 + i as f64 * 0.07,
                dq: if i == 0 { 0.0007 } else { -0.005 * i as f64 },
                tau: 0.1234567 * (i + 1) as f64,
                kp: 60.0,
                kd: 1.0,
            });
            let second = first.map(|s| Setpoint {
                q: s.q + 0.00003,
                ..s
            });
            let encoded = |values: [Setpoint; 7]| -> Vec<u8> {
                values
                    .iter()
                    .enumerate()
                    .flat_map(|(i, s)| encode(i as u8, calibration[i].command(*s)).unwrap())
                    .collect()
            };
            let first_bytes = encoded(first);
            let second_bytes = encoded(second);
            handle.send_with_sequence(first, 71).unwrap();
            wait_for_protocol(|| {
                wire.lock()
                    .unwrap()
                    .written
                    .as_chunks::<{ 7 * COMMAND_LEN }>()
                    .0
                    .iter()
                    .filter(|batch| batch.as_slice() == first_bytes.as_slice())
                    .count()
                    >= 2
            });
            handle.send_with_sequence(second, 72).unwrap();
            wait_for_protocol(|| {
                wire.lock()
                    .unwrap()
                    .written
                    .as_chunks::<{ 7 * COMMAND_LEN }>()
                    .0
                    .iter()
                    .any(|batch| batch == second_bytes.as_slice())
            });
            let mut feedback = Vec::new();
            wait_for_protocol(|| {
                let (received, error) = handle.drain();
                assert!(error.is_none(), "{error:?}");
                feedback.extend(received);
                feedback.len() == frames.len()
            });
            assert_eq!(feedback[2].value.temperature, -23.0);
            assert_eq!(feedback[2].value.winding_temperature, 255.0);
            let clear = [Setpoint {
                mode: DriveMode::ClearFault,
                q: 5.0,
                ..first[0]
            }; 7];
            handle.write_blocking(clear).unwrap();
            handle.disable().unwrap();
            let stopped_len = wire.lock().unwrap().written.len();
            handle.send_with_sequence(first, 999).unwrap();
            thread::sleep(WRITE_PERIOD * 2);
            assert_eq!(wire.lock().unwrap().written.len(), stopped_len);
            drop(handle);

            let recording = finish_trace(&journal, &trace.id);
            let written = wire.lock().unwrap().written.clone();
            assert_eq!(written.len() % (7 * COMMAND_LEN), 0);
            let batches = written.as_chunks::<{ 7 * COMMAND_LEN }>().0;
            assert_eq!(
                recording.samples.len() as u64 + recording.trace.dropped,
                (batches.len() + frames.len()) as u64
            );
            let transmits: Vec<_> = recording
                .samples
                .iter()
                .filter(|s| matches!(s.data, TraceData::Transmit { .. }))
                .collect();
            let mut remaining_batches = batches.iter();
            for sample in &transmits {
                let TraceData::Transmit {
                    seq,
                    mode,
                    q_ref,
                    dq_ref,
                    tau_ff,
                    q_wire,
                    dq_wire,
                    tau_wire,
                    kp,
                    kd,
                    write_us,
                    timing: _,
                    tuning: _,
                } = sample.data
                else {
                    unreachable!()
                };
                let values = match seq {
                    71 => first,
                    72 => second,
                    0 if q_ref[0] == clear[0].q => clear,
                    0 => stopped_setpoints(),
                    _ => panic!("unpublished transmit sequence {seq}"),
                };
                let expected_bytes = encoded(values);
                let batch = remaining_batches
                    .by_ref()
                    .find(|batch| batch.as_slice() == expected_bytes.as_slice())
                    .expect("captured TX is not an ordered subsequence of written batches");
                assert_trace_numbers(q_ref, values.map(|s| s.q));
                assert_trace_numbers(dq_ref, values.map(|s| s.dq));
                assert_trace_numbers(tau_ff, values.map(|s| s.tau));
                assert_eq!(kp, values.map(|s| s.kp));
                assert_eq!(kd, values.map(|s| s.kd));
                assert!(write_us > 0);
                for (i, packet) in batch.as_chunks::<COMMAND_LEN>().0.iter().enumerate() {
                    assert_eq!(mode[i], (packet[2] >> 4) & 0x07);
                    // Feed the actual command fields through the independently
                    // golden-tested feedback path, with its different offsets/CRC.
                    let mut reply = [0u8; FEEDBACK_LEN];
                    reply[..2].copy_from_slice(&[0xfc, 0xee]);
                    reply[2] = packet[2];
                    reply[6..14].copy_from_slice(&packet[4..12]);
                    let crc = crc32(&reply[2..22]);
                    reply[22..26].copy_from_slice(&crc.to_le_bytes());
                    let decoded = calibration[i].feedback(decode(&reply).unwrap());
                    assert_trace_numbers(
                        [q_wire[i], dq_wire[i], tau_wire[i]],
                        [decoded.q, decoded.dq, decoded.tau],
                    );
                }
                if seq == 71 || seq == 72 {
                    assert_eq!(dq_wire[0], 0.0);
                    assert_ne!(dq_ref[0], dq_wire[0]);
                    assert_ne!(q_ref, q_wire);
                }
            }
            if recording.trace.dropped == 0 {
                assert_eq!(transmits.len(), batches.len());
                assert!(
                    transmits
                        .iter()
                        .filter(|s| matches!(s.data, TraceData::Transmit { seq: 71, .. }))
                        .count()
                        >= 2
                );
                assert!(
                    transmits
                        .iter()
                        .any(|s| matches!(s.data, TraceData::Transmit { seq: 72, .. }))
                );
            }
            let captured: Vec<_> = recording
                .samples
                .iter()
                .filter(|s| matches!(s.data, TraceData::Feedback { .. }))
                .collect();
            if recording.trace.dropped == 0 {
                assert_eq!(captured.len(), frames.len());
            }
            let mut remaining_feedback = feedback.iter().zip(&frames);
            for sample in captured {
                let TraceData::Feedback {
                    joint,
                    q,
                    dq,
                    tau,
                    temperature,
                    winding_temperature,
                    voltage,
                    error,
                    mode,
                    timeout,
                } = sample.data
                else {
                    unreachable!()
                };
                let (event, frame) = remaining_feedback
                    .by_ref()
                    .find(|(event, _)| {
                        joint == event.value.id
                            && trace_number_matches(q, event.value.q)
                            && source_time_matches(sample.at_us, event.at, before, after)
                    })
                    .expect("captured feedback is not an ordered subsequence of decoded frames");
                let expected =
                    calibration[event.value.id as usize].feedback(decode(frame).unwrap());
                assert_eq!(winding_temperature, Some(f64::from(frame[4])));
                assert_eq!(
                    (joint, error, mode, timeout),
                    (expected.id, expected.error, expected.mode, expected.timeout)
                );
                assert_trace_numbers(
                    [q, dq, tau, temperature, voltage],
                    [
                        expected.q,
                        expected.dq,
                        expected.tau,
                        expected.temperature,
                        expected.voltage,
                    ],
                );
                assert_eq!(event.value.q, expected.q);
                assert_source_time(sample.at_us, event.at, before, after);
            }
        }
    }

    #[test]
    fn offline_trace_wire_lsb_staircase_and_float32_rounding_use_encoded_fields() {
        let directory = tempfile::tempdir().unwrap();
        let journal = Journal::start(directory.path().join("events.db")).unwrap();
        let before = Instant::now();
        let trace = journal
            .trace_start(20_000, "right", Some("fake-port"))
            .unwrap();
        let after = Instant::now();
        let (mut bus, wire) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            ..Wire::default()
        });
        bus.calibration = [Calibration {
            gain: 1.0,
            offset: 0.0,
        }; 7];
        bus.budget = TEST_IO_BUDGET;
        let position_lsb: [f64; 7] = std::array::from_fn(|i| 2.0 * PI / 32768.0 / ratio(i as u8));
        let velocity_lsb: [f64; 7] = std::array::from_fn(|i| 2.0 * PI / 64.0 / ratio(i as u8));
        let torque_lsb: [f64; 7] = std::array::from_fn(|i| ratio(i as u8) / 2560.0);
        let fractions = [0.0, 0.2, 0.4, 0.6, 0.8, 1.2];
        let mut sources = Vec::new();
        let mut values_by_sequence = Vec::new();
        for (seq, fraction) in fractions.into_iter().enumerate() {
            let values = std::array::from_fn(|i| Setpoint {
                mode: DriveMode::Servo,
                q: position_lsb[i] * fraction,
                dq: position_lsb[i] * 0.2 / WRITE_PERIOD.as_secs_f64(),
                tau: torque_lsb[i] * 0.51,
                kp: 60.0,
                kd: 1.0,
            });
            let started = Instant::now();
            if seq == 0 {
                // Delayed submission must keep the source Instant, not write,
                // queue-consumption, serialization or SQLite completion time.
                thread::sleep(Duration::from_millis(20));
            }
            bus.write_all_recorded(
                &values,
                Some((&journal, started, TransmitMetadata::direct(seq as u64))),
            )
            .unwrap();
            sources.push(started);
            values_by_sequence.push(values);
        }
        for (n, delta) in [-1e-10, 1e-10].into_iter().enumerate() {
            let values = std::array::from_fn(|i| Setpoint {
                mode: DriveMode::Servo,
                q: position_lsb[i] * (0.5 + delta),
                dq: velocity_lsb[i] * (0.5 + delta),
                tau: torque_lsb[i] * (0.5 + delta),
                kp: 60.0,
                kd: 1.0,
            });
            let started = Instant::now();
            bus.write_all_recorded(
                &values,
                Some((&journal, started, TransmitMetadata::direct((n + 6) as u64))),
            )
            .unwrap();
            sources.push(started);
            values_by_sequence.push(values);
        }
        let negative = std::array::from_fn(|i| Setpoint {
            mode: DriveMode::Servo,
            q: -position_lsb[i] * 0.6,
            dq: -velocity_lsb[i] * 0.6,
            tau: -torque_lsb[i] * 0.6,
            kp: 60.0,
            kd: 1.0,
        });
        let started = Instant::now();
        bus.write_all_recorded(
            &negative,
            Some((&journal, started, TransmitMetadata::direct(8))),
        )
        .unwrap();
        sources.push(started);
        values_by_sequence.push(negative);
        let recording = finish_trace(&journal, &trace.id);
        assert_eq!(
            recording.samples.len() as u64 + recording.trace.dropped,
            sources.len() as u64
        );
        let written = wire.lock().unwrap().written.clone();
        let batches = written.as_chunks::<{ 7 * COMMAND_LEN }>().0;
        assert_eq!(batches.len(), sources.len());
        // Check every actual batch even if the bounded recorder dropped it.
        for (n, batch) in batches.iter().enumerate() {
            for packet in batch.as_chunks::<COMMAND_LEN>().0 {
                if n < 6 {
                    assert_eq!(
                        i32::from_le_bytes(packet[8..12].try_into().unwrap()),
                        [0, 0, 0, 1, 1, 1][n]
                    );
                    assert_eq!(i16::from_le_bytes(packet[6..8].try_into().unwrap()), 0);
                    assert_eq!(i16::from_le_bytes(packet[4..6].try_into().unwrap()), 1);
                } else if n == 8 {
                    assert_eq!(i32::from_le_bytes(packet[8..12].try_into().unwrap()), -1);
                    assert_eq!(i16::from_le_bytes(packet[6..8].try_into().unwrap()), -1);
                    assert_eq!(i16::from_le_bytes(packet[4..6].try_into().unwrap()), -1);
                }
            }
        }
        let mut previous_seq = None;
        for sample in &recording.samples {
            let TraceData::Transmit {
                seq,
                mode,
                q_ref,
                dq_ref,
                tau_ff,
                q_wire,
                dq_wire,
                tau_wire,
                ..
            } = sample.data
            else {
                unreachable!()
            };
            let n = usize::try_from(seq).unwrap();
            assert!(n < batches.len() && previous_seq.is_none_or(|prev| seq > prev));
            previous_seq = Some(seq);
            assert_source_time(sample.at_us, sources[n], before, after);
            assert_eq!(mode, [1; 7]);
            assert_trace_numbers(q_ref, values_by_sequence[n].map(|s| s.q));
            assert_trace_numbers(dq_ref, values_by_sequence[n].map(|s| s.dq));
            assert_trace_numbers(tau_ff, values_by_sequence[n].map(|s| s.tau));
            for (i, packet) in batches[n].as_chunks::<COMMAND_LEN>().0.iter().enumerate() {
                let p = i32::from_le_bytes(packet[8..12].try_into().unwrap());
                let v = i16::from_le_bytes(packet[6..8].try_into().unwrap());
                let t = i16::from_le_bytes(packet[4..6].try_into().unwrap());
                assert_trace_numbers(
                    [q_wire[i], dq_wire[i], tau_wire[i]],
                    [
                        (p as f64 * position_lsb[i]) as f32 as f64,
                        (v as f64 * velocity_lsb[i]) as f32 as f64,
                        (t as f64 * torque_lsb[i]) as f32 as f64,
                    ],
                );
            }
        }
        // Opposite sides of a half-LSB boundary collapse to the same f32.
        // A pre-encoding f64 estimate would incorrectly show different steps.
        for i in 0..7 {
            let a = values_by_sequence[6][i];
            let b = values_by_sequence[7][i];
            assert!(a.q < position_lsb[i] * 0.5 && b.q > position_lsb[i] * 0.5);
            assert!(a.dq < velocity_lsb[i] * 0.5 && b.dq > velocity_lsb[i] * 0.5);
            assert!(a.tau < torque_lsb[i] * 0.5 && b.tau > torque_lsb[i] * 0.5);
            assert_eq!(a.q as f32, b.q as f32);
            assert_eq!(a.dq as f32, b.dq as f32);
            assert_eq!(a.tau as f32, b.tau as f32);
        }
        assert_eq!(
            &written[6 * 7 * COMMAND_LEN..7 * 7 * COMMAND_LEN],
            &written[7 * 7 * COMMAND_LEN..8 * 7 * COMMAND_LEN]
        );
    }

    #[test]
    fn offline_trace_does_not_report_rejected_or_partially_written_batches() {
        let directory = tempfile::tempdir().unwrap();
        let journal = Journal::start(directory.path().join("events.db")).unwrap();
        let trace = journal
            .trace_start(20_000, "right", Some("fake-port"))
            .unwrap();
        let (mut bus, wire) = fake_bus(Wire {
            write_chunk: 5,
            fail_after: Some(5),
            fail_kind: Some(io::ErrorKind::BrokenPipe),
            ..Wire::default()
        });
        let mut invalid = stopped_setpoints();
        invalid[6] = Setpoint {
            mode: DriveMode::Servo,
            q: f64::NAN,
            ..stop()
        };
        assert!(
            bus.write_all_recorded(
                &invalid,
                Some((&journal, Instant::now(), TransmitMetadata::direct(81)))
            )
            .is_err()
        );
        assert!(wire.lock().unwrap().written.is_empty());
        assert!(
            bus.write_all_recorded(
                &stopped_setpoints(),
                Some((&journal, Instant::now(), TransmitMetadata::direct(82)))
            )
            .is_err()
        );
        assert_eq!(wire.lock().unwrap().written.len(), 5);
        assert!(
            bus.write_all_recorded(
                &stopped_setpoints(),
                Some((&journal, Instant::now(), TransmitMetadata::direct(83)))
            )
            .unwrap_err()
            .contains("reconnect")
        );
        assert_eq!(wire.lock().unwrap().written.len(), 5);
        let recording = finish_trace(&journal, &trace.id);
        assert!(recording.samples.is_empty());
        assert_eq!(recording.trace.dropped, 0);
    }

    #[test]
    fn exchange_handles_short_writes_and_returns_only_new_matching_feedback() {
        let g = golden();
        let a = unhex(g["feedback"][0]["packet"].as_str().unwrap());
        let b = unhex(g["feedback"][2]["packet"].as_str().unwrap());
        let (mut bus, wire) = fake_bus(Wire {
            write_chunk: 3,
            active: VecDeque::from([a.clone()]), // queued BEFORE TX: stale
            after_tx: VecDeque::from([b.clone(), a[..7].to_vec(), a[7..].to_vec()]),
            ..Wire::default()
        });
        let f = bus.exchange(0, stop()).unwrap().unwrap();
        assert_eq!(f.id, 0);
        assert_eq!(wire.lock().unwrap().written, encode(0, stop()).unwrap());
        assert_eq!(wire.lock().unwrap().clears, 1);
        assert_eq!(bus.stats().received, 2);
        // No cached previous frame is returned, and the partial buffer is fresh.
        wire.lock().unwrap().active.push_back(a);
        assert!(bus.exchange(0, stop()).unwrap().is_none());
        assert_eq!(bus.stats().timeouts, 1);
        assert_eq!(bus.stats().received, 2);
        assert_eq!(wire.lock().unwrap().written.len(), 2 * COMMAND_LEN);
    }

    #[test]
    fn batch_exchange_sends_seven_joints_before_collecting_feedback() {
        let g = golden();
        let base = unhex(g["feedback"][0]["packet"].as_str().unwrap());
        let frames: VecDeque<Vec<u8>> = (0..7).map(|id| feedback_for_id(&base, id)).collect();
        let (mut bus, wire) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            after_tx: frames,
            ..Wire::default()
        });
        let setpoints = [Setpoint {
            mode: DriveMode::Servo,
            q: 0.1,
            dq: 0.0,
            tau: 0.0,
            kp: 60.0,
            kd: 0.5,
        }; 7];
        let feedback = bus.exchange_all(&setpoints).unwrap();
        for (id, value) in feedback.iter().enumerate() {
            assert_eq!(value.as_ref().unwrap().id as usize, id);
        }
        let cal = calibrations("right").unwrap();
        let written = wire.lock().unwrap().written.clone();
        assert_eq!(written.len(), 7 * COMMAND_LEN);
        for (id, setpoint) in setpoints.iter().enumerate() {
            let expected = encode(id as u8, cal[id].command(*setpoint)).unwrap();
            assert_eq!(
                &written[id * COMMAND_LEN..(id + 1) * COMMAND_LEN],
                expected.as_slice()
            );
        }
        assert_eq!(bus.stats().received, 7);
        assert_eq!(bus.stats().timeouts, 0);
        assert_eq!(wire.lock().unwrap().clears, 1);
    }

    #[test]
    fn async_bus_writes_and_drains_timestamped_feedback_without_control_thread_blocking() {
        let g = golden();
        let base = unhex(g["feedback"][0]["packet"].as_str().unwrap());
        let frames: VecDeque<Vec<u8>> = (0..7).map(|id| feedback_for_id(&base, id)).collect();
        let (bus, wire) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            after_tx: frames,
            ..Wire::default()
        });
        let handle = BusHandle::start(bus).unwrap();
        handle
            .send(
                [Setpoint {
                    mode: DriveMode::Servo,
                    q: 0.1,
                    dq: 0.0,
                    tau: 0.0,
                    kp: 60.0,
                    kd: 1.0,
                }; 7],
            )
            .unwrap();
        let deadline = Instant::now() + Duration::from_millis(500);
        let mut received = Vec::new();
        while received.len() < 7 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(5));
            let (events, error) = handle.drain();
            assert!(error.is_none());
            received.extend(events);
        }
        assert!(received.len() >= 7);
        for (id, event) in received.iter().take(7).enumerate() {
            assert_eq!(event.value.id as usize, id);
            assert!(event.at.elapsed() < Duration::from_secs(1));
            assert!(event.fault_command.is_none());
        }
        assert!(wire.lock().unwrap().written.len() >= 7 * COMMAND_LEN);
        handle.disable().unwrap();
        assert!(wire.lock().unwrap().written.len() >= 14 * COMMAND_LEN);
    }

    #[test]
    fn serial_reader_keeps_the_faults_own_joint_command_until_drain() {
        let (bus, wire) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            ..Wire::default()
        });
        let handle = BusHandle::start(bus).unwrap();
        let values = std::array::from_fn(|i| Setpoint {
            mode: DriveMode::Servo,
            q: 0.05 * i as f64,
            tau: 0.2 * i as f64,
            ..stop()
        });
        let reference = Reference::joint(
            Instant::now(),
            values.map(|s| s.q),
            [0.0; 7],
            [0.0; 7],
            1,
            0,
        );
        handle.publish(values, 1, reference).unwrap();
        wait_for_protocol(|| handle.latest_reference().is_some());

        let fault = unhex(golden()["feedback"][4]["packet"].as_str().unwrap());
        let mut healthy = fault.clone();
        healthy[14..18].fill(0);
        let crc = crc32(&healthy[2..22]);
        healthy[22..26].copy_from_slice(&crc.to_le_bytes());
        wire.lock().unwrap().active.extend([fault, healthy]);
        wait_for_protocol(|| handle.stats().received == 2);
        handle.disable().unwrap();
        assert!(handle.latest_reference().is_none());

        let (frames, error) = handle.drain();
        assert!(error.is_none());
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].value.id, 5);
        assert_eq!(frames[0].fault_command, Some((values[5].q, values[5].tau)));
        assert!(frames[1].fault_command.is_none());
    }

    #[test]
    fn transmit_timing_records_batch_starts_and_can_reset_its_window() {
        let at = Instant::now();
        let mut stats = TransmitStats::default();
        stats.record(None, at, at + Duration::from_millis(2));
        stats.record(
            Some(at),
            at + Duration::from_millis(10),
            at + Duration::from_millis(13),
        );
        stats.record(
            Some(at + Duration::from_millis(10)),
            at + Duration::from_millis(25),
            at + Duration::from_millis(27),
        );
        assert_eq!(stats.batches, 3);
        assert_eq!(stats.intervals, 2);
        assert!((stats.interval_sum_ms - 25.0).abs() < 1e-9);
        assert!((stats.interval_min_ms - 10.0).abs() < 1e-9);
        assert!((stats.interval_max_ms - 15.0).abs() < 1e-9);
        assert!((stats.write_max_ms - 3.0).abs() < 1e-9);
        let finished = std::mem::take(&mut stats);
        assert_eq!(finished.batches, 3);
        assert_eq!(stats.batches, 0);
        stats.record(
            Some(at + Duration::from_millis(25)),
            at + Duration::from_millis(35),
            at + Duration::from_millis(36),
        );
        assert!((stats.interval_min_ms - 10.0).abs() < 1e-9);
    }

    #[test]
    fn serial_target_update_after_heartbeat_preserves_send_period() {
        let (bus, wire) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            ..Wire::default()
        });
        let handle = BusHandle::start(bus).unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        // A producer update just after an entire heartbeat batch is legal.
        // Updates cannot bypass the writer's deadline grid. A late wakeup can
        // shorten the next interval to the existing half-period spacing floor.
        while wire.lock().unwrap().command_times.len() < 14 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_micros(100));
        }
        let baseline = wire.lock().unwrap().command_times.len();
        assert!(baseline >= 14);
        handle
            .send(
                [Setpoint {
                    mode: DriveMode::Servo,
                    q: 0.1,
                    dq: 0.1,
                    tau: 0.0,
                    kp: 60.0,
                    kd: 10.0,
                }; 7],
            )
            .unwrap();
        while wire.lock().unwrap().command_times.len() < baseline + 7 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_micros(100));
        }
        let times = wire.lock().unwrap().command_times.clone();
        let batches: Vec<_> = times
            .iter()
            .filter(|(id, _)| *id == 0)
            .map(|(_, at)| *at)
            .collect();
        let intervals: Vec<_> = batches
            .windows(2)
            .map(|pair| pair[1].duration_since(pair[0]).as_secs_f64() * 1000.0)
            .collect();
        println!("TX batch intervals (ms): {intervals:?}");
        // The fake driver timestamps the first packet after preparation; allow
        // 0.5 ms variation there. The grid's exact floor is checked above.
        let observed_floor_ms = (WRITE_PERIOD / 2).as_secs_f64() * 1000.0 - 0.5;
        assert!(
            intervals.iter().all(|ms| *ms >= observed_floor_ms),
            "watch updates bypass the writer spacing floor: {intervals:?}"
        );
    }

    #[test]
    fn exchange_wrong_id_corrupt_frame_and_partial_frame_do_not_become_zero_feedback() {
        let g = golden();
        let other = unhex(g["feedback"][2]["packet"].as_str().unwrap());
        let valid = unhex(g["feedback"][0]["packet"].as_str().unwrap());
        let mut bad = valid.clone();
        bad[22] ^= 1;
        let (mut bus, _) = fake_bus(Wire {
            write_chunk: 20,
            after_tx: VecDeque::from([other, bad, valid[..13].to_vec()]),
            ..Wire::default()
        });
        let start = Instant::now();
        assert!(bus.exchange(0, stop()).unwrap().is_none());
        assert!(start.elapsed() < Duration::from_millis(200));
        assert_eq!(bus.stats().received, 1);
        assert_eq!(bus.stats().crc_errors, 1);
        assert_eq!(bus.stats().timeouts, 1);
    }

    #[test]
    fn partial_write_error_does_not_retry_whole_packet_or_allow_following_commands() {
        for kind in [
            io::ErrorKind::BrokenPipe,
            io::ErrorKind::TimedOut,
            io::ErrorKind::Interrupted,
        ] {
            let (mut bus, wire) = fake_bus(Wire {
                write_chunk: 5,
                fail_after: Some(5),
                fail_kind: Some(kind),
                ..Wire::default()
            });
            assert!(bus.exchange(0, stop()).is_err());
            assert_eq!(
                wire.lock().unwrap().written,
                encode(0, stop()).unwrap()[..5]
            );
            assert!(bus.exchange(1, stop()).unwrap_err().contains("reconnect"));
            assert_eq!(wire.lock().unwrap().written.len(), 5);
            assert_eq!(
                bus.stats().timeouts,
                if kind == io::ErrorKind::BrokenPipe {
                    0
                } else {
                    1
                }
            );
        }
    }

    #[test]
    fn exited_writer_keeps_the_first_io_failure_for_publish_and_commands() {
        let (bus, wire) = fake_bus(Wire {
            write_chunk: 5,
            fail_after: Some(5),
            ..Wire::default()
        });
        let handle = BusHandle::start(bus).unwrap();
        let until = Instant::now() + Duration::from_secs(1);
        while !handle.join.as_ref().unwrap().is_finished() && Instant::now() < until {
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(handle.join.as_ref().unwrap().is_finished());
        let first = handle.send(stopped_setpoints()).unwrap_err();
        assert!(first.contains("serial batch J1"), "{first}");
        assert!(first.contains("wrote 5/20 command bytes"), "{first}");
        assert!(matches!(handle.drain().1, Some(BusError::Io(ref e)) if e == &first));
        wire.lock().unwrap().read_error = Some(io::ErrorKind::ConnectionReset);
        let until = Instant::now() + Duration::from_secs(1);
        while !handle.reader.as_ref().unwrap().is_finished() && Instant::now() < until {
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(handle.reader.as_ref().unwrap().is_finished());
        assert!(matches!(handle.drain().1, Some(BusError::Io(ref e)) if e == &first));
        assert_eq!(handle.send(stopped_setpoints()).unwrap_err(), first);
        assert_eq!(
            handle.write_blocking(stopped_setpoints()).unwrap_err(),
            first
        );
        assert_eq!(handle.disable().unwrap_err(), first);
        assert_eq!(wire.lock().unwrap().written.len(), 5);
    }

    #[test]
    fn configured_motion_above_the_old_compensation_cap_keeps_transmitting() {
        for (speed, weight, friction) in [
            (-0.21, 1.0, false),
            (0.6, 1.0, false),
            (0.6, 0.0, false),
            (-0.6, 1.0, true),
        ] {
            let (bus, _wire) = fake_bus(Wire {
                write_chunk: COMMAND_LEN,
                ..Wire::default()
            });
            let handle = BusHandle::start(bus).unwrap();
            let mut velocity = [0.0; 7];
            velocity[5] = speed;
            let mut reference =
                Reference::joint(Instant::now(), [0.0; 7], velocity, [0.0; 7], 1, 0);
            reference.joint_velocity = [1.0; 7];
            reference.servo_tuning.friction_enabled = friction;
            reference.servo_tuning.joints[5].velocity_ff = weight;
            reference.servo_tuning.joints[5].coulomb = 0.2;
            let values = std::array::from_fn(|i| Setpoint {
                mode: DriveMode::Servo,
                q: 0.0,
                dq: velocity[i],
                tau: 0.0,
                kp: 60.0,
                kd: 10.0,
            });
            handle.publish(values, 1, reference).unwrap();
            let until = Instant::now() + Duration::from_millis(500);
            while handle.latest_reference().is_none()
                && !handle.join.as_ref().unwrap().is_finished()
                && Instant::now() < until
            {
                std::thread::sleep(Duration::from_millis(1));
            }
            let sent = handle.latest_reference().unwrap_or_else(|| {
                panic!(
                    "configured speed {speed}, FF {weight}: {:?}",
                    handle.drain().1
                )
            });
            assert_eq!(sent.dq[5], speed);
            assert!(handle.drain().1.is_none());
            handle.disable().unwrap();
        }
    }

    #[test]
    fn sender_completed_packet_scheduling_gap_does_not_fail_the_next_packet() {
        let (mut bus, wire) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            pause_after_servo_packet: Some((2, Duration::from_millis(30))),
            ..Wire::default()
        });
        bus.budget = Duration::from_millis(20);
        let handle = BusHandle::start(bus).unwrap();
        let now = Instant::now();
        let reference = Reference::joint(now, [0.5; 7], [0.0; 7], [0.0; 7], 42, 0);
        let values = [Setpoint {
            mode: DriveMode::Servo,
            q: 0.5,
            dq: 0.0,
            tau: 0.0,
            kp: 60.0,
            kd: 10.0,
        }; 7];
        handle.publish(values, 1, reference).unwrap();
        wait_for_protocol(|| {
            handle.latest_reference().is_some() || handle.join.as_ref().unwrap().is_finished()
        });
        let sent = handle.latest_reference().unwrap_or_else(|| {
            panic!("completed packet gap killed writer: {:?}", handle.drain().1)
        });
        assert_eq!(sent.generation, 42);
        assert!(handle.drain().1.is_none());
        let written = {
            let wire = wire.lock().unwrap();
            assert!(wire.pause_after_servo_packet.is_none());
            assert!(wire.command_times.windows(2).any(|pair| {
                pair[0].0 == 2
                    && pair[1].0 == 3
                    && pair[1].1.duration_since(pair[0].1) >= Duration::from_millis(30)
            }));
            wire.written.clone()
        };
        let servo: Vec<_> = written
            .as_chunks::<COMMAND_LEN>()
            .0
            .iter()
            .filter(|p| p[2] & 0x70 == 0x10)
            .take(7)
            .map(|p| p[2] & 0x0f)
            .collect();
        assert_eq!(servo, (0..7).collect::<Vec<_>>());
        handle.disable().unwrap();
    }

    #[test]
    fn sender_priority_command_failure_exits_instead_of_resuming_cached_motion() {
        for mode in [0, 6] {
            let (bus, wire) = fake_bus(Wire {
                write_chunk: COMMAND_LEN,
                ..Wire::default()
            });
            let handle = BusHandle::start(bus).unwrap();
            let reference = Reference::joint(Instant::now(), [0.5; 7], [0.0; 7], [0.0; 7], 1, 0);
            handle
                .publish(
                    [Setpoint {
                        mode: DriveMode::Servo,
                        q: 0.5,
                        dq: 0.0,
                        tau: 0.0,
                        kp: 60.0,
                        kd: 10.0,
                    }; 7],
                    1,
                    reference,
                )
                .unwrap();
            wait_for_protocol(|| handle.latest_reference().is_some());
            wire.lock().unwrap().fail_once_for_mode = Some((mode, io::ErrorKind::BrokenPipe));
            let error = if mode == 0 {
                handle.disable()
            } else {
                handle.write_blocking(
                    [Setpoint {
                        mode: DriveMode::ClearFault,
                        ..stop()
                    }; 7],
                )
            }
            .unwrap_err();
            assert!(error.contains("wrote 0/20 command bytes"), "{error}");
            let after_failure = wire.lock().unwrap().written.len();
            thread::sleep(WRITE_PERIOD * 3);
            assert!(
                handle.join.as_ref().unwrap().is_finished(),
                "failed priority write left writer alive"
            );
            assert_eq!(wire.lock().unwrap().written.len(), after_failure);
            assert!(handle.latest_reference().is_none());
            assert_eq!(handle.disable().unwrap_err(), error);
        }
    }

    #[test]
    fn sender_priority_failure_keeps_an_earlier_reader_error() {
        let (bus, wire) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            ..Wire::default()
        });
        let handle = BusHandle::start(bus).unwrap();
        wire.lock().unwrap().read_error = Some(io::ErrorKind::ConnectionReset);
        wait_for_protocol(|| handle.reader.as_ref().unwrap().is_finished());
        let first = handle.failure.get().unwrap().clone();
        wire.lock().unwrap().fail_once_for_mode = Some((6, io::ErrorKind::BrokenPipe));
        let error = handle
            .write_blocking(
                [Setpoint {
                    mode: DriveMode::ClearFault,
                    ..stop()
                }; 7],
            )
            .unwrap_err();
        assert_eq!(error, first);
        wait_for_protocol(|| handle.join.as_ref().unwrap().is_finished());
        assert_eq!(handle.disable().unwrap_err(), first);
        assert!(matches!(handle.drain().1, Some(BusError::Io(ref e)) if e == &first));
    }

    #[test]
    fn sender_tuning_above_old_software_caps_reaches_prepare_and_wire() {
        let (bus, wire) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            ..Wire::default()
        });
        let handle = BusHandle::start(bus).unwrap();
        let mut reference = Reference::joint(Instant::now(), [0.0; 7], [0.4; 7], [0.0; 7], 7, 0);
        reference.servo_tuning.friction_enabled = true;
        for joint in &mut reference.servo_tuning.joints {
            joint.kd = 50.0;
            joint.velocity_ff = 1.5;
            joint.coulomb = 3.0;
            joint.viscous = 12.0;
            joint.velocity_scale = 1.0;
            joint.max_friction = 3.0;
        }
        let values = [Setpoint {
            mode: DriveMode::Servo,
            q: 0.0,
            dq: 0.4,
            tau: 0.0,
            kp: 60.0,
            kd: 10.0,
        }; 7];
        handle.publish(values, 1, reference.clone()).unwrap();
        wait_for_protocol(|| {
            handle.latest_reference().is_some() || handle.join.as_ref().unwrap().is_finished()
        });
        let sent = handle
            .latest_reference()
            .unwrap_or_else(|| panic!("tuning hit old caps: {:?}", handle.drain().1));
        assert_eq!(sent.dq, [0.4; 7]);
        let written = wire.lock().unwrap().written.clone();
        let packets: Vec<_> = written
            .as_chunks::<COMMAND_LEN>()
            .0
            .iter()
            .filter(|p| p[2] & 0x70 == 0x10)
            .take(7)
            .collect();
        assert_eq!(packets.len(), 7);
        let calibration = calibrations("right").unwrap();
        for (i, packet) in packets.into_iter().enumerate() {
            let applied = reference.servo_tuning.apply([0.4; 7], [0.0; 7]).unwrap();
            let expected = encode(
                i as u8,
                calibration[i].command(Setpoint {
                    q: sent.q[i],
                    dq: applied.dq_feedforward[i],
                    tau: applied.tau[i],
                    kd: 50.0,
                    ..values[i]
                }),
            )
            .unwrap();
            // Position is sampled on every heartbeat. The first packet's
            // tuning fields remain the same even if latest_reference advanced.
            assert_eq!(packet[2], expected[2]);
            assert_eq!(packet[4..8], expected[4..8]);
            assert_eq!(packet[12..16], expected[12..16]);
        }
        assert!(handle.drain().1.is_none());
        handle.disable().unwrap();
    }

    #[test]
    fn sender_partial_packet_timeout_never_commits_or_retries() {
        let (bus, wire) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            ..Wire::default()
        });
        let handle = BusHandle::start(bus).unwrap();
        let reference = Reference::joint(Instant::now(), [0.5; 7], [0.0; 7], [0.0; 7], 1, 0);
        handle
            .publish(
                [Setpoint {
                    mode: DriveMode::Servo,
                    q: 0.5,
                    dq: 0.0,
                    tau: 0.0,
                    kp: 60.0,
                    kd: 10.0,
                }; 7],
                1,
                reference,
            )
            .unwrap();
        wait_for_protocol(|| handle.latest_reference().is_some());
        let limit = {
            let mut wire = wire.lock().unwrap();
            let limit = wire.written.len() + 5;
            wire.write_chunk = 5;
            wire.fail_after = Some(limit);
            wire.fail_kind = Some(io::ErrorKind::TimedOut);
            limit
        };
        wait_for_protocol(|| handle.join.as_ref().unwrap().is_finished());
        let error = handle.disable().unwrap_err();
        assert!(error.contains("deadline expired"), "{error}");
        assert!(error.contains("wrote 5/20 command bytes"), "{error}");
        assert!(matches!(handle.drain().1, Some(BusError::Io(ref e)) if e == &error));
        assert!(handle.latest_reference().is_none());
        assert_eq!(wire.lock().unwrap().written.len(), limit);
    }

    #[test]
    fn sender_torque_uses_the_friction_parameter_and_final_encoder() {
        for (side, sign) in [
            ("right", -1.0),
            ("right", 1.0),
            ("left", -1.0),
            ("left", 1.0),
        ] {
            let model = crate::kinematics::Kinematics::load(side).unwrap();
            let effort_limits = model.effort_limits();
            let calibration = calibrations(side).unwrap();
            let (mut bus, wire) = fake_bus(Wire {
                write_chunk: COMMAND_LEN,
                ..Wire::default()
            });
            bus.calibration = calibration;
            let handle = BusHandle::start(bus).unwrap();
            let mut reference = Reference::joint_for_model(
                Instant::now(),
                [0.0; 7],
                [sign * 0.4; 7],
                [0.0; 7],
                1,
                0,
                Some(&model),
            );
            reference.servo_tuning.friction_enabled = true;
            for joint in &mut reference.servo_tuning.joints {
                joint.coulomb = 30.0;
                joint.max_friction = 20.0;
            }
            let values = [Setpoint {
                mode: DriveMode::Servo,
                q: 0.0,
                dq: sign * 0.4,
                tau: sign * 12.0,
                kp: 60.0,
                kd: 10.0,
            }; 7];
            handle.publish(values, 1, reference).unwrap();
            wait_for_protocol(|| {
                handle.latest_reference().is_some() || handle.join.as_ref().unwrap().is_finished()
            });
            assert!(
                handle.latest_reference().is_some(),
                "torque rejected before encoder: {:?}",
                handle.drain().1
            );
            let packets: Vec<_> = wire
                .lock()
                .unwrap()
                .written
                .as_chunks::<COMMAND_LEN>()
                .0
                .iter()
                .filter(|p| p[2] & 0x70 == 0x10)
                .take(7)
                .copied()
                .collect();
            assert_eq!(packets.len(), 7);
            for (i, packet) in packets.into_iter().enumerate() {
                let tau = sign * 32.0_f64.min(effort_limits[i]);
                let expected = encode(
                    i as u8,
                    calibration[i].command(Setpoint { tau, ..values[i] }),
                )
                .unwrap();
                assert_eq!(packet[2..8], expected[2..8]);
                assert_eq!(packet[12..16], expected[12..16]);
            }
            assert!(handle.drain().1.is_none());
            handle.disable().unwrap();
        }
        let (bus, wire) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            ..Wire::default()
        });
        let handle = BusHandle::start(bus).unwrap();
        let values = [Setpoint {
            mode: DriveMode::Servo,
            q: 0.0,
            dq: 0.0,
            tau: 170.0,
            kp: 60.0,
            kd: 10.0,
        }; 7];
        // A direct packet has no model reference and must still fail encoder preflight.
        handle.send_with_sequence(values, 1).unwrap();
        wait_for_protocol(|| handle.join.as_ref().unwrap().is_finished());
        let error = handle.disable().unwrap_err();
        assert!(error.contains("SDK range"), "{error}");
        assert!(handle.latest_reference().is_none());
        assert!(
            wire.lock()
                .unwrap()
                .written
                .as_chunks::<COMMAND_LEN>()
                .0
                .iter()
                .all(|p| p[2] & 0x70 == 0)
        );
    }

    #[test]
    fn calibration_trial_saturates_each_encoded_axis_and_finishes_with_gravity_only() {
        use crate::{calibration_path::Playback, kinematics::Kinematics, teaching::Approach};
        let started = Instant::now();
        let origin = [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3];
        for (side, xml) in [
            (
                "left",
                include_str!("../../resources/models/left/robot.urdf"),
            ),
            (
                "right",
                include_str!("../../resources/models/right/robot.urdf"),
            ),
        ] {
            let model = Kinematics::load(side).unwrap();
            let calibration = calibrations(side).unwrap();
            let urdf = urdf_rs::read_from_string(xml).unwrap();
            for direction in [-1.0, 1.0] {
                let target = origin.map(|q| q + direction * 0.1);
                let limits = crate::trajectory::MotionTuning::default();
                let program = Arc::new(Playback {
                    approach: Approach::for_motion(origin, target, model.speed_limits(), limits)
                        .unwrap(),
                    leg: None,
                });
                let mut committed = None;
                for moving in [true, false] {
                    let t = if moving {
                        program.duration() / 2.0
                    } else {
                        program.duration() + 0.01
                    };
                    let at = started + Duration::from_secs_f64(t);
                    let phase = PlaybackPhase::Original(started);
                    let sample = phase.sample(&program, at);
                    let gravity = model.gravity(&sample.q).unwrap();
                    let mut reference = Reference::joint_for_model(
                        at,
                        sample.q,
                        sample.dq,
                        sample.ddq,
                        1,
                        0,
                        Some(&model),
                    );
                    reference.sampling = Sampling::Calibration {
                        program: program.clone(),
                        phase,
                        limits,
                        feedback_at: Some(at),
                    };
                    reference.servo_tuning.friction_enabled = true;
                    for joint in &mut reference.servo_tuning.joints {
                        joint.stribeck.enabled = true;
                        joint.stribeck.positive.viscous = f64::MAX;
                        joint.stribeck.negative.viscous = f64::MAX;
                        joint.max_friction = f64::MAX;
                    }
                    reference.servo_tuning.validate().unwrap();
                    let values = std::array::from_fn(|i| Setpoint {
                        mode: DriveMode::Servo,
                        q: sample.q[i],
                        dq: sample.dq[i],
                        tau: gravity[i],
                        kp: 60.0,
                        kd: reference.servo_tuning.joints[i].kd,
                    });
                    let source = PublishedSetpoints {
                        values,
                        seq: 1,
                        reference: Some(reference),
                    };
                    let prepared = source.prepare(at, committed).unwrap();
                    assert_eq!(prepared.rejection, None);
                    assert_eq!(prepared.motion_velocity, sample.dq);
                    let trace = prepared.tuning.unwrap();
                    assert_eq!(trace.tau_gravity, gravity);
                    for (i, value) in prepared.values.iter().enumerate() {
                        let name = format!("J{}", i + 1);
                        let effort = urdf
                            .joints
                            .iter()
                            .find(|j| j.name == name)
                            .unwrap()
                            .limit
                            .effort;
                        let tau = if moving {
                            direction * effort
                        } else {
                            gravity[i]
                        };
                        assert_eq!(value.tau, tau, "{side} {name}");
                        assert_eq!(trace.clamped[i], moving);
                        assert_eq!(trace.friction_applied[i], tau - gravity[i]);
                        if moving {
                            assert!(trace.friction_requested[i] * direction > effort);
                        } else {
                            assert_eq!(trace.friction_requested[i], 0.0);
                        }
                        let packet = encode(i as u8, calibration[i].command(*value)).unwrap();
                        let expected = Setpoint { tau, ..values[i] };
                        assert_eq!(
                            packet,
                            encode(i as u8, calibration[i].command(expected)).unwrap()
                        );
                        let raw = i16::from_le_bytes(packet[4..6].try_into().unwrap());
                        let wire =
                            calibration[i].feedback_motion(decode_motion(i as u8, raw, 0, 0))[2];
                        assert!(wire.abs() <= effort, "{side} {name} {wire} > {effort}");
                        assert!((wire - tau).abs() <= ratio(i as u8) / 2560.0 / 2.0);
                    }
                    committed = Some(prepared.committed(at));
                }
            }
        }
    }

    #[test]
    fn gravity_clipping_and_opposing_friction_are_distinct_in_fresh_and_fallback_trace() {
        let at = Instant::now();
        let model = crate::kinematics::Kinematics::load("left").unwrap();
        let efforts = model.effort_limits();
        for sign in [-1.0, 1.0] {
            // The raw gravity alone is two Nm beyond each joint's budget.
            let values = efforts.map(|limit| Setpoint {
                mode: DriveMode::Servo,
                q: 0.0,
                dq: 0.0,
                tau: sign * (limit + 2.0),
                kp: 60.0,
                kd: 10.0,
            });
            for (enabled, speed, applied_delta, clipped) in [
                (false, 0.5, 0.0, true),
                (true, 0.0, 0.0, true),
                (true, 0.125, 0.0, true),
                (true, 0.25, 0.0, false),
                (true, 0.5, -2.0, false),
            ] {
                let mut reference = Reference::joint_for_model(
                    at,
                    [0.0; 7],
                    [-sign * speed; 7],
                    [0.0; 7],
                    1,
                    0,
                    Some(&model),
                );
                reference.servo_tuning.friction_enabled = enabled;
                for joint in &mut reference.servo_tuning.joints {
                    joint.viscous = 8.0;
                    joint.max_friction = 8.0;
                }
                let source = PublishedSetpoints {
                    values,
                    seq: 1,
                    reference: Some(reference),
                };
                let prepared = source.prepare(at, None).unwrap();
                let trace = prepared.tuning.unwrap();
                assert_eq!(prepared.rejection, None);
                assert_eq!(trace.tau_gravity, efforts.map(|e| sign * e));
                assert_eq!(
                    trace.friction_requested,
                    [if enabled { -sign * speed * 8.0 } else { 0.0 }; 7]
                );
                assert_eq!(trace.friction_applied, [sign * applied_delta; 7]);
                assert_eq!(trace.clamped, [clipped; 7]);
                assert_eq!(
                    prepared.values.map(|s| s.tau),
                    efforts.map(|e| sign * (e + applied_delta))
                );

                let stale = at + crate::reference::MAX_REFERENCE_AGE + Duration::from_millis(1);
                let held = source.prepare(stale, Some(prepared.committed(at))).unwrap();
                assert_eq!(held.rejection, Some(SampleStatus::Stale));
                assert_eq!(held.values.map(|s| s.tau), trace.tau_gravity);
                let held_trace = held.tuning.unwrap();
                assert_eq!(held_trace.friction_requested, [0.0; 7]);
                assert_eq!(held_trace.friction_applied, [0.0; 7]);
                assert_eq!(held_trace.tau_gravity, trace.tau_gravity);
                assert_eq!(held_trace.clamped, [true; 7]);
            }
        }
    }

    #[test]
    fn sender_calibration_requires_a_representable_gain_instead_of_a_numeric_floor() {
        let source = (0..7)
            .map(|i| format!("{i} 1 1e-8 0"))
            .collect::<Vec<_>>()
            .join("\n");
        let calibration = parse_calibration(&source).unwrap();
        let (mut bus, wire) = fake_bus(Wire {
            write_chunk: COMMAND_LEN,
            ..Wire::default()
        });
        bus.calibration = calibration;
        let handle = BusHandle::start(bus).unwrap();
        let reference = Reference::joint(Instant::now(), [0.0; 7], [0.0; 7], [0.0; 7], 1, 0);
        handle
            .publish(
                [Setpoint {
                    mode: DriveMode::Servo,
                    q: 0.0,
                    dq: 0.0,
                    tau: 0.0,
                    kp: 60.0,
                    kd: 10.0,
                }; 7],
                1,
                reference,
            )
            .unwrap();
        wait_for_protocol(|| {
            handle.latest_reference().is_some() || handle.join.as_ref().unwrap().is_finished()
        });
        assert!(
            handle.latest_reference().is_some(),
            "{:?}",
            handle.drain().1
        );
        assert!(
            wire.lock()
                .unwrap()
                .written
                .as_chunks::<COMMAND_LEN>()
                .0
                .iter()
                .any(|p| p[2] & 0x70 == 0x10)
        );
        handle.disable().unwrap();
        for (direction, scale) in [("0", "1"), ("1", "0"), ("3e38", "3e38"), ("1e-30", "1e-30")] {
            let source = (0..7)
                .map(|i| format!("{i} {direction} {scale} 0"))
                .collect::<Vec<_>>()
                .join("\n");
            assert!(
                parse_calibration(&source).is_err(),
                "gain={direction}*{scale}"
            );
        }
    }

    #[test]
    fn tuning_kd_validation_matches_the_actual_encoder() {
        for index in 0..7 {
            for kd in [
                0.0,
                30.0,
                30.0001,
                50.0,
                102.68,
                102.68131,
                102.69,
                103.0,
                f32::MAX as f64,
                f64::MAX,
            ] {
                let mut tuning = crate::tuning::ServoTuning::default();
                tuning.joints[index].kd = kd;
                let mut value = stop();
                value.mode = DriveMode::Servo;
                value.kd = kd;
                assert_eq!(
                    tuning.validate().is_ok(),
                    encode(index as u8, value).is_ok(),
                    "J{} kd={kd}",
                    index + 1
                );
            }
        }
    }

    #[test]
    fn sender_clear_fault_is_sent_once_without_periodic_action_retries() {
        for published in [false, true] {
            let (bus, wire) = fake_bus(Wire {
                write_chunk: COMMAND_LEN,
                ..Wire::default()
            });
            let handle = BusHandle::start(bus).unwrap();
            let values = [Setpoint {
                mode: DriveMode::ClearFault,
                ..stop()
            }; 7];
            if published {
                let reference =
                    Reference::joint(Instant::now(), [0.0; 7], [0.0; 7], [0.0; 7], 1, 0);
                handle.publish(values, 1, reference).unwrap();
            } else {
                handle.write_blocking(values).unwrap();
            }
            wait_for_protocol(|| {
                wire.lock()
                    .unwrap()
                    .written
                    .as_chunks::<COMMAND_LEN>()
                    .0
                    .iter()
                    .any(|p| p[2] & 0x70 == 0x60)
            });
            thread::sleep(WRITE_PERIOD * 4);
            handle.disable().unwrap();
            let clears = wire
                .lock()
                .unwrap()
                .written
                .as_chunks::<COMMAND_LEN>()
                .0
                .iter()
                .filter(|p| p[2] & 0x70 == 0x60)
                .count();
            assert_eq!(
                clears, 7,
                "ClearFault was retried by the heartbeat (published={published})"
            );
        }
    }

    #[test]
    #[ignore = "requires R17A_CALIBRATION_TRACE pointing to a private calibration capture"]
    fn recorded_calibration_tail_keeps_compensation_within_configured_motion() {
        let path = std::env::var("R17A_CALIBRATION_TRACE").expect("set the capture path");
        let capture: serde_json::Value =
            serde_json::from_slice(&std::fs::read(path).unwrap()).unwrap();
        let points = capture["curves"].as_array().unwrap();
        let last = &points[points.len() - 1];
        let previous = &points[points.len() - 2];
        let dt = (last["t_ms"].as_f64().unwrap() - previous["t_ms"].as_f64().unwrap()) / 1000.0;
        let q: [f64; 7] = serde_json::from_value(last["reference"].clone()).unwrap();
        let dq: [f64; 7] = serde_json::from_value(last["velocity"].clone()).unwrap();
        let previous_dq: [f64; 7] = serde_json::from_value(previous["velocity"].clone()).unwrap();
        let ddq = std::array::from_fn(|i| (dq[i] - previous_dq[i]) / dt);
        let program: crate::calibration_path::CartesianProgram =
            serde_json::from_value(capture["settings"]["cartesian"].clone()).unwrap();
        let motion = program.motion;
        let model = Arc::new(crate::kinematics::Kinematics::load(&program.side).unwrap());
        let origin = model.fk(&program.origin, &program.tcp).unwrap();
        let mut delta = [0.0; 6];
        delta[program.segments[0].axis] = program.segments[0].offset;
        let now = Instant::now();
        let mut reference = Reference::joint(now, q, dq, ddq, 1, 0);
        reference.joint_velocity = motion.speed_limits(model.speed_limits());
        reference.cartesian = Some(crate::reference::CartesianGuard {
            model,
            tcp: program.tcp,
            path: crate::kinematics::CartesianPath {
                origin,
                axis: program.segments[0].axis,
                frame: crate::kinematics::Frame::Base,
            },
            goal: Some(
                crate::kinematics::offset_pose(&origin, delta, crate::kinematics::Frame::Base)
                    .unwrap(),
            ),
        });
        reference.servo_tuning = serde_json::from_value(capture["baseline"].clone()).unwrap();
        let source = PublishedSetpoints {
            values: std::array::from_fn(|i| Setpoint {
                mode: DriveMode::Servo,
                q: q[i],
                dq: dq[i],
                tau: 0.0,
                kp: 60.0,
                kd: 10.0,
            }),
            seq: 1,
            reference: Some(reference),
        };
        let sample = source
            .prepare(now + Duration::from_millis(10), None)
            .unwrap();
        println!(
            "recorded J6 {:.9}, continued 10 ms {:.9} rad/s",
            dq[5], sample.motion_velocity[5]
        );
        assert!(dq[5].abs() < 0.2 && sample.motion_velocity[5].abs() > 0.2);
        assert!(sample.motion_velocity[5].abs() < motion.joint_speed);
        assert_eq!(sample.values[5].dq, sample.motion_velocity[5]);
        assert!(sample.rejection.is_none());
    }

    #[test]
    fn read_failure_and_driver_overrun_do_not_publish_feedback_or_resend() {
        let (mut bus, wire) = fake_bus(Wire {
            write_chunk: 20,
            read_error: Some(io::ErrorKind::BrokenPipe),
            ..Wire::default()
        });
        assert!(
            bus.exchange(0, stop())
                .unwrap_err()
                .contains("after command sent")
        );
        assert_eq!(wire.lock().unwrap().written.len(), COMMAND_LEN);
        let g = golden();
        let (mut bus, wire) = fake_bus(Wire {
            write_chunk: 20,
            late_read: true,
            after_tx: VecDeque::from([unhex(g["feedback"][0]["packet"].as_str().unwrap())]),
            ..Wire::default()
        });
        assert!(bus.exchange(0, stop()).unwrap().is_none());
        assert_eq!(wire.lock().unwrap().written.len(), COMMAND_LEN);
        assert_eq!(bus.stats().timeouts, 1);
    }
}
