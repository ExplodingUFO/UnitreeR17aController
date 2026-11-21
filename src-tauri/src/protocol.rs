//! IM6014 wire protocol, verified against Unitree SDK commit
//! 0803d033b530fcfa0e6fa9236de68c2773fc9402, MotorProtocol.cpp / arm_motor_bus.cpp.
//! One owner, seven-joint command batches. No action retries or cached feedback.

use crate::journal::{Journal, TraceData};
use crossbeam_channel::{Receiver as EventReceiver, Sender, bounded, unbounded};
use serde::{Deserialize, Serialize};
use serialport::{
    ClearBuffer, DataBits, FlowControl, Parity, SerialPort, SerialPortType, StopBits,
};
use std::collections::BTreeMap;
use std::io;
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};
use tokio::sync::watch;

const COMMAND_LEN: usize = 20;
const FEEDBACK_LEN: usize = 26;
const MAX_TRANSACTION: Duration = Duration::from_millis(20);
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
    /// Driver temperature, signed degrees Celsius (not winding temperature).
    pub temperature: f64,
    pub voltage: f64,
    pub error: u32,
    pub warning: u8,
    /// Actual device mode, 0..7, independent of the requested mode.
    pub mode: u8,
    /// Device-reported watchdog state, separate from host transaction timeouts.
    pub timeout: bool,
}

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct BusStats {
    /// All CRC-valid frames received during exchanges, including other IDs.
    pub received: u64,
    pub crc_errors: u64,
    /// Seven-joint cycles whose shared feedback budget expired.
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
        if id >= 7 || seen[id] || dir.abs() < 1e-6 || scale < 1e-6 {
            return Err("invalid or duplicate calibration joint".into());
        }
        values[id] = Calibration {
            gain: dir * scale,
            offset,
        };
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

fn encode(id: u8, s: Setpoint) -> Result<[u8; COMMAND_LEN], String> {
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
    /// timeout is the TOTAL seven-joint cycle budget, capped at 20ms (minimum 1ms).
    pub fn open(port: &str, side: &str, baud: u32, timeout: Duration) -> Result<Self, String> {
        let calibration = calibrations(side)?;
        if baud == 0 {
            return Err("baud rate must be positive".into());
        }
        let budget = timeout.clamp(Duration::from_millis(1), MAX_TRANSACTION);
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
        recording: Option<(&Journal, Instant, u64)>,
    ) -> Result<(), String> {
        if self.incomplete_write {
            return Err("previous command was partially written; reconnect serial port".into());
        }
        let deadline = Instant::now() + self.budget;
        let mut packets = [[0u8; COMMAND_LEN]; 7];
        for (i, packet) in packets.iter_mut().enumerate() {
            *packet = encode(i as u8, self.calibration[i].command(setpoints[i]))?;
        }
        for packet in &packets {
            self.write_packet(packet, deadline)?;
            // The official SDK keeps this gap after every packet, including
            // the last one, before the adapter turns around to receive.
            std::thread::sleep(Duration::from_micros(200));
        }
        if let Some((journal, started, seq)) = recording {
            sample_transmit(
                journal,
                started,
                setpoints,
                seq,
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

enum BusEvent {
    Feedback(Feedback, Instant),
    Error(String),
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

#[derive(Clone, Copy)]
struct PublishedSetpoints {
    values: [Setpoint; 7],
    seq: u64,
}

fn sample_transmit(
    journal: &Journal,
    started: Instant,
    values: &[Setpoint; 7],
    seq: u64,
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
            seq,
            mode: std::array::from_fn(|i| (packets[i][2] >> 4) & 0x07),
            q_ref: std::array::from_fn(|i| values[i].q),
            dq_ref: std::array::from_fn(|i| values[i].dq),
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
    command: Sender<BusCommand>,
    events: EventReceiver<BusEvent>,
    stats: Arc<Mutex<BusStats>>,
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
    ) -> Result<Self, String> {
        Self::start_recorded(Bus::open(port, side, baud, timeout)?, Some(journal))
    }

    #[cfg(test)]
    fn start(bus: Bus) -> Result<Self, String> {
        Self::start_recorded(bus, None)
    }
    fn start_recorded(mut bus: Bus, journal: Option<Journal>) -> Result<Self, String> {
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
        });
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
        let join = thread::Builder::new()
            .name("arm-serial-tx".into())
            .spawn(move || {
                let mut last = *setpoint_rx.borrow_and_update();
                let mut next_write = Instant::now();
                let mut previous_write = None;
                let mut disabled = false;
                loop {
                    if worker_stop.load(Ordering::Acquire) {
                        break;
                    }
                    if let Ok(command) = command_rx.try_recv() {
                        match command {
                            BusCommand::Disable(reply) => {
                                let started = Instant::now();
                                let result = bus.write_all_recorded(
                                    &stopped_setpoints(),
                                    writer_journal.as_ref().map(|journal| (journal, started, 0)),
                                );
                                let _ = reply.send(result);
                                disabled = true;
                            }
                            BusCommand::Write(setpoints, reply) => {
                                let started = Instant::now();
                                let result = bus.write_all_recorded(
                                    &setpoints,
                                    writer_journal.as_ref().map(|journal| (journal, started, 0)),
                                );
                                if result.is_ok() {
                                    last = PublishedSetpoints {
                                        values: *setpoints,
                                        seq: 0,
                                    };
                                    next_write = started + WRITE_PERIOD;
                                    if let Ok(mut shared) = writer_stats.lock() {
                                        shared.transmit.record(
                                            previous_write,
                                            started,
                                            Instant::now(),
                                        );
                                    }
                                    previous_write = Some(started);
                                }
                                let _ = reply.send(result);
                            }
                        }
                    }
                    if !disabled {
                        if setpoint_rx.has_changed().unwrap_or(false) {
                            last = *setpoint_rx.borrow_and_update();
                        }
                        if Instant::now() >= next_write {
                            let started = Instant::now();
                            if let Err(error) = bus.write_all_recorded(
                                &last.values,
                                writer_journal
                                    .as_ref()
                                    .map(|journal| (journal, started, last.seq)),
                            ) {
                                let _ = writer_events.send(BusEvent::Error(error));
                                break;
                            }
                            if let Ok(mut shared) = writer_stats.lock() {
                                shared
                                    .transmit
                                    .record(previous_write, started, Instant::now());
                            }
                            previous_write = Some(started);
                            // A target update changes the next payload, not the clock.
                            // Count start-to-start, including the seven-packet write;
                            // after an overrun send the latest target once, never catch up.
                            next_write = started + WRITE_PERIOD;
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
                                let at = Instant::now();
                                if let Some(journal) = journal.as_ref() {
                                    journal.sample(
                                        at,
                                        TraceData::Feedback {
                                            joint: frame.id,
                                            q: frame.q,
                                            dq: frame.dq,
                                            tau: frame.tau,
                                            temperature: frame.temperature,
                                            voltage: frame.voltage,
                                            error: frame.error,
                                            mode: frame.mode,
                                            timeout: frame.timeout,
                                        },
                                    );
                                }
                                let _ = event_tx.send(BusEvent::Feedback(frame, at));
                            }
                        }
                        Err(error) => {
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
            command: command_tx,
            events: event_rx,
            stats: handle_stats,
            stop,
            join: Some(join),
            reader: Some(reader),
        })
    }

    #[cfg(test)]
    pub fn send(&self, setpoints: [Setpoint; 7]) -> Result<(), String> {
        self.send_with_sequence(setpoints, 0)
    }
    pub fn send_with_sequence(&self, setpoints: [Setpoint; 7], seq: u64) -> Result<(), String> {
        self.setpoints
            .send(PublishedSetpoints {
                values: setpoints,
                seq,
            })
            .map_err(|_| "serial I/O thread is closed".into())
    }

    pub fn drain(&self) -> (Vec<(Feedback, Instant)>, Option<String>) {
        let mut feedback = Vec::new();
        let mut error = None;
        for event in self.events.try_iter() {
            match event {
                BusEvent::Feedback(value, at) => feedback.push((value, at)),
                BusEvent::Error(message) => error = Some(message),
            }
        }
        (feedback, error)
    }

    pub fn disable(&self) -> Result<(), String> {
        let (reply, result) = bounded(1);
        self.command
            .send_timeout(BusCommand::Disable(reply), Duration::from_millis(100))
            .map_err(|error| format!("serial I/O command queue: {error}"))?;
        result
            .recv_timeout(Duration::from_millis(500))
            .map_err(|error| format!("serial disable acknowledgement: {error}"))?
    }

    pub fn write_blocking(&self, setpoints: [Setpoint; 7]) -> Result<(), String> {
        let (reply, result) = bounded(1);
        self.command
            .send_timeout(
                BusCommand::Write(Box::new(setpoints), reply),
                Duration::from_millis(100),
            )
            .map_err(|error| format!("serial I/O command queue: {error}"))?;
        result
            .recv_timeout(Duration::from_millis(500))
            .map_err(|error| format!("serial write acknowledgement: {error}"))?
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
mod tests {
    use super::*;
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
        clears: usize,
        read_error: Option<io::ErrorKind>,
        late_read: bool,
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
                let staged = std::mem::take(&mut w.after_tx);
                w.active.extend(staged);
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
            bus.budget = MAX_TRANSACTION;
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
                    voltage,
                    error,
                    mode,
                    timeout,
                } = sample.data
                else {
                    unreachable!()
                };
                let ((event, at), frame) = remaining_feedback
                    .by_ref()
                    .find(|((event, at), _)| {
                        joint == event.id
                            && trace_number_matches(q, event.q)
                            && source_time_matches(sample.at_us, *at, before, after)
                    })
                    .expect("captured feedback is not an ordered subsequence of decoded frames");
                let expected = calibration[event.id as usize].feedback(decode(frame).unwrap());
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
                assert_eq!(event.q, expected.q);
                assert_source_time(sample.at_us, *at, before, after);
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
        bus.budget = MAX_TRANSACTION;
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
            bus.write_all_recorded(&values, Some((&journal, started, seq as u64)))
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
            bus.write_all_recorded(&values, Some((&journal, started, (n + 6) as u64)))
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
        bus.write_all_recorded(&negative, Some((&journal, started, 8)))
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
            bus.write_all_recorded(&invalid, Some((&journal, Instant::now(), 81)))
                .is_err()
        );
        assert!(wire.lock().unwrap().written.is_empty());
        assert!(
            bus.write_all_recorded(&stopped_setpoints(), Some((&journal, Instant::now(), 82)))
                .is_err()
        );
        assert_eq!(wire.lock().unwrap().written.len(), 5);
        assert!(
            bus.write_all_recorded(&stopped_setpoints(), Some((&journal, Instant::now(), 83)))
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
        for (id, (value, at)) in received.iter().enumerate() {
            if id == 7 {
                break;
            }
            assert_eq!(value.id as usize, id);
            assert!(at.elapsed() < Duration::from_secs(1));
        }
        assert!(wire.lock().unwrap().written.len() >= 7 * COMMAND_LEN);
        handle.disable().unwrap();
        assert!(wire.lock().unwrap().written.len() >= 14 * COMMAND_LEN);
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
        // It must not force a second batch at a fraction of the stated 10ms period.
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
        assert!(
            intervals.iter().all(|ms| *ms >= 9.0),
            "watch updates bypass the 10ms writer cadence: {intervals:?}"
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
