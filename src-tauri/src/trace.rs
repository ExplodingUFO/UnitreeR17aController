//! Bounded observation captures. Producers try a preallocated queue's mutex
//! once and drop on contention or overflow; they never wait for its lock or allocate.
//! Serialization and disk work run on readers/writer after releasing the queue lock.
use crate::reference::Timing;
use crate::tuning::ServoTuning;
use rusqlite::{Connection, OpenFlags, OptionalExtension, params};
use serde::{Deserialize, Serialize};
use std::{
    collections::VecDeque,
    path::{Path, PathBuf},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU8, AtomicU64, AtomicUsize, Ordering},
    },
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

pub const DEFAULT_DURATION_MS: u64 = 10_000;
const KEEP: usize = 10;
const CAPACITY: usize = 4096;
const BATCH: usize = 128;
// Also bounds disk use if a faulty source submits at an arbitrarily high rate.
const MAX_SAMPLES: u64 = 100_000;
const CLOSED: usize = 1 << (usize::BITS - 1);
const RECORDING: u8 = 1;
const WRITING: u8 = 2;
const COMPLETE: u8 = 3;
const FAILED: u8 = 4;
const POLL: Duration = Duration::from_millis(10);
const BUSY: Duration = Duration::from_millis(40);

pub(crate) fn duration_ns(duration_ms: u64) -> Result<u64, String> {
    duration_ms
        .checked_mul(1_000_000)
        .filter(|duration| *duration > 0)
        .ok_or_else(|| "invalid_trace_duration".into())
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(tag = "source", rename_all = "snake_case")]
#[allow(
    clippy::large_enum_variant,
    reason = "Inline Copy samples avoid producer allocation; the preallocated bounded queue caps memory."
)]
pub enum TraceData {
    Control {
        seq: u64,
        generation: u64,
        #[serde(default)]
        servo_tuning: Option<ServoTuning>,
        #[serde(default)]
        servo_tuning_revision: u64,
        enabled: bool,
        stopping: bool,
        motion: u8,
        axis: Option<u8>,
        q_ref: [f64; 7],
        dq_ref: [f64; 7],
        ddq_ref: [f64; 7],
        jerk_ref: [f64; 7],
        tau_ff: [f64; 7],
        q: [Option<f64>; 7],
        dq: [Option<f64>; 7],
        tau: [Option<f64>; 7],
        voltage: [Option<f64>; 7],
        age_us: [Option<u64>; 7],
    },
    Transmit {
        seq: u64,
        #[serde(default)]
        tuning: Option<ServoSample>,
        /// Missing in captures made before send-time sampling was introduced.
        #[serde(default)]
        timing: Option<Timing>,
        /// Encoded `(packet[2] >> 4) & 0x07`: 0 disabled, 1 servo, 6 clear.
        mode: [u8; 7],
        q_ref: [f64; 7],
        dq_ref: [f64; 7],
        tau_ff: [f64; 7],
        q_wire: [f64; 7],
        dq_wire: [f64; 7],
        tau_wire: [f64; 7],
        kp: [f64; 7],
        kd: [f64; 7],
        write_us: u64,
    },
    Feedback {
        joint: u8,
        q: f64,
        dq: f64,
        tau: f64,
        temperature: f64,
        /// Older feedback samples recorded only the driver temperature.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        winding_temperature: Option<f64>,
        voltage: f64,
        error: u32,
        mode: u8,
        timeout: bool,
    },
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ServoSample {
    pub revision: u64,
    pub dq_feedforward: [f64; 7],
    pub tau_gravity: [f64; 7],
    pub friction_requested: [f64; 7],
    pub friction_applied: [f64; 7],
    pub clamped: [bool; 7],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TraceSummary {
    pub id: String,
    pub started_at_ms: i64,
    pub duration_ms: u64,
    pub side: String,
    pub port: Option<String>,
    pub status: String,
    pub accepted: u64,
    pub written: u64,
    pub dropped: u64,
    pub pending: u64,
    pub errors: u64,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct TraceSample {
    /// Original source Instant, relative to this capture's start (not dequeue time).
    pub at_us: u64,
    #[serde(flatten)]
    pub data: TraceData,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TraceRecording {
    pub trace: TraceSummary,
    pub samples: Vec<TraceSample>,
}

struct Slot {
    token: AtomicU64,
    gate: AtomicUsize,
    phase: AtomicU8,
    start_ns: AtomicU64,
    deadline_ns: AtomicU64,
    attempts: AtomicU64,
    accepted: AtomicU64,
    written: AtomicU64,
    dropped: AtomicU64,
    pending: AtomicU64,
    errors: AtomicU64,
    abandoned: AtomicU64,
}

impl Default for Slot {
    fn default() -> Self {
        Self {
            token: AtomicU64::new(0),
            gate: AtomicUsize::new(CLOSED),
            phase: AtomicU8::new(0),
            start_ns: AtomicU64::new(0),
            deadline_ns: AtomicU64::new(0),
            attempts: AtomicU64::new(0),
            accepted: AtomicU64::new(0),
            written: AtomicU64::new(0),
            dropped: AtomicU64::new(0),
            pending: AtomicU64::new(0),
            errors: AtomicU64::new(0),
            abandoned: AtomicU64::new(0),
        }
    }
}

#[derive(Clone)]
struct Entry {
    token: u64,
    summary: TraceSummary,
}

#[derive(Clone, Copy)]
struct Queued {
    token: u64,
    position: u64,
    sample: TraceSample,
}

struct State {
    slots: [Slot; KEEP],
    entries: Mutex<VecDeque<Entry>>,
    queue: Mutex<VecDeque<Queued>>,
    active: AtomicU64,
    next: AtomicU64,
    // Orders shutdown against a concurrent start; producers use the per-slot gate.
    lifecycle: AtomicUsize,
    abort: AtomicBool,
    done: AtomicBool,
    clean: AtomicBool,
}

impl Default for State {
    fn default() -> Self {
        Self {
            slots: std::array::from_fn(|_| Slot::default()),
            entries: Mutex::new(VecDeque::with_capacity(KEEP)),
            queue: Mutex::new(VecDeque::with_capacity(CAPACITY)),
            active: AtomicU64::new(0),
            next: AtomicU64::new(1),
            lifecycle: AtomicUsize::new(0),
            abort: AtomicBool::new(false),
            done: AtomicBool::new(false),
            clean: AtomicBool::new(true),
        }
    }
}

impl State {
    fn slot(&self, token: u64) -> &Slot {
        &self.slots[(token as usize & 15) - 1]
    }

    fn stop(&self, token: u64) {
        let slot = self.slot(token);
        slot.gate.fetch_or(CLOSED, Ordering::SeqCst);
        let _ =
            slot.phase
                .compare_exchange(RECORDING, WRITING, Ordering::AcqRel, Ordering::Acquire);
        let _ = self
            .active
            .compare_exchange(token, 0, Ordering::SeqCst, Ordering::SeqCst);
    }

    fn summaries(&self) -> Result<Vec<TraceSummary>, String> {
        let entries = self.entries.lock().map_err(|_| "trace_unavailable")?;
        Ok(entries
            .iter()
            .rev()
            .map(|e| snapshot(e, self.slot(e.token)))
            .collect())
    }
}

struct Admission<'a>(&'a AtomicUsize);
impl Drop for Admission<'_> {
    fn drop(&mut self) {
        self.0.fetch_sub(1, Ordering::SeqCst);
    }
}

struct Shared {
    origin: Instant,
    path: PathBuf,
    state: Arc<State>,
}

impl Drop for Shared {
    fn drop(&mut self) {
        self.state.lifecycle.fetch_or(CLOSED, Ordering::SeqCst);
        self.state.abort.store(true, Ordering::Release);
        for slot in &self.state.slots {
            slot.gate.fetch_or(CLOSED, Ordering::SeqCst);
        }
        // Detached worker: Drop never joins or waits for SQLite/OS calls.
    }
}

#[derive(Clone)]
pub(crate) struct Recorder(Arc<Shared>);

impl Recorder {
    pub(crate) fn start(event_path: &Path) -> Result<Self, String> {
        let mut path = event_path.as_os_str().to_os_string();
        path.push(".traces.sqlite3");
        let path = PathBuf::from(path);
        let state = Arc::new(State::default());
        let worker_state = state.clone();
        let worker_path = path.clone();
        let origin = Instant::now();
        thread::Builder::new()
            .name("short-traces".into())
            .spawn(move || writer(&worker_path, origin, worker_state))
            .map_err(|e| format!("start trace writer: {e}"))?;
        Ok(Self(Arc::new(Shared {
            origin,
            path,
            state,
        })))
    }

    pub(crate) fn trace_start(
        &self,
        duration_ms: u64,
        side: &str,
        port: Option<&str>,
    ) -> Result<TraceSummary, String> {
        let duration = duration_ns(duration_ms)?;
        if !matches!(side, "left" | "right") || port.is_some_and(|p| p.len() > 512) {
            return Err("invalid_trace_context".into());
        }
        let state = &self.0.state;
        let previous = state.lifecycle.fetch_add(1, Ordering::SeqCst);
        let _admission = Admission(&state.lifecycle);
        if previous & CLOSED != 0 {
            return Err("closing".into());
        }
        let mut entries = state.entries.lock().map_err(|_| "trace_unavailable")?;
        if state.lifecycle.load(Ordering::SeqCst) & CLOSED != 0 {
            return Err("closing".into());
        }
        let now = nanos(self.0.origin, Instant::now()).unwrap_or(0);
        let deadline = now.checked_add(duration).ok_or("invalid_trace_duration")?;
        let active = state.active.load(Ordering::SeqCst);
        if active != 0 {
            if now < state.slot(active).deadline_ns.load(Ordering::Relaxed) {
                return Err("trace_active".into());
            }
            state.stop(active);
        }
        let index = if entries.len() == KEEP {
            let oldest = entries.front().unwrap();
            let slot = state.slot(oldest.token);
            if !matches!(slot.phase.load(Ordering::Acquire), COMPLETE | FAILED)
                || slot.gate.load(Ordering::SeqCst) != CLOSED
            {
                return Err("trace_capacity".into());
            }
            let index = (oldest.token as usize & 15) - 1;
            entries.pop_front();
            index
        } else {
            state
                .slots
                .iter()
                .position(|s| s.token.load(Ordering::Relaxed) == 0)
                .ok_or("trace_capacity")?
        };
        let token = (state.next.fetch_add(1, Ordering::Relaxed) << 4) | (index as u64 + 1);
        let slot = &state.slots[index];
        slot.token.store(token, Ordering::Release);
        slot.start_ns.store(now, Ordering::Relaxed);
        slot.deadline_ns.store(deadline, Ordering::Relaxed);
        for counter in [
            &slot.attempts,
            &slot.accepted,
            &slot.written,
            &slot.dropped,
            &slot.pending,
            &slot.errors,
            &slot.abandoned,
        ] {
            counter.store(0, Ordering::Relaxed);
        }
        let summary = TraceSummary {
            id: uuid::Uuid::new_v4().to_string(),
            started_at_ms: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis()
                .min(i64::MAX as u128) as i64,
            duration_ms,
            side: side.to_owned(),
            port: port.map(str::to_owned),
            status: "recording".into(),
            accepted: 0,
            written: 0,
            dropped: 0,
            pending: 0,
            errors: 0,
        };
        entries.push_back(Entry {
            token,
            summary: summary.clone(),
        });
        slot.phase.store(RECORDING, Ordering::Release);
        // Preserve late, rejected producers still releasing the old closed gate.
        slot.gate.fetch_and(!CLOSED, Ordering::SeqCst);
        state.active.store(token, Ordering::SeqCst);
        Ok(summary)
    }

    pub(crate) fn trace_stop(&self) -> Result<Option<TraceSummary>, String> {
        let state = &self.0.state;
        let entries = state.entries.lock().map_err(|_| "trace_unavailable")?;
        let token = state.active.load(Ordering::SeqCst);
        if token == 0 {
            return Ok(None);
        }
        state.stop(token);
        Ok(entries
            .iter()
            .find(|e| e.token == token)
            .map(|e| snapshot(e, state.slot(token))))
    }

    /// Try the queue mutex once; contention, poisoning, or overflow counts as a drop.
    /// No allocation, blocking lock acquisition, queue wakeup, serialization, or I/O.
    pub(crate) fn sample(&self, at: Instant, data: TraceData) {
        let state = &self.0.state;
        if state.lifecycle.load(Ordering::SeqCst) & CLOSED != 0 {
            return;
        }
        let token = state.active.load(Ordering::SeqCst);
        if token == 0 {
            return;
        }
        let slot = state.slot(token);
        let previous = slot.gate.fetch_add(1, Ordering::SeqCst);
        let _admission = Admission(&slot.gate);
        // A delayed producer may observe a reused slot. Its old token must never
        // be admitted to the new capture, even if the new gate is already open.
        if previous & CLOSED != 0 || slot.token.load(Ordering::Acquire) != token {
            return;
        }
        let Some(at_ns) = nanos(self.0.origin, at) else {
            return;
        };
        let start = slot.start_ns.load(Ordering::Relaxed);
        if at_ns < start {
            return;
        }
        if at_ns >= slot.deadline_ns.load(Ordering::Relaxed) {
            state.stop(token);
            return;
        }
        let position = slot.attempts.fetch_add(1, Ordering::Relaxed);
        if position >= MAX_SAMPLES {
            slot.dropped.fetch_add(1, Ordering::Relaxed);
            return;
        }
        let sample = Queued {
            token,
            position,
            sample: TraceSample {
                at_us: (at_ns - start) / 1000,
                data,
            },
        };
        let Ok(mut queue) = state.queue.try_lock() else {
            slot.dropped.fetch_add(1, Ordering::Relaxed);
            return;
        };
        if queue.len() >= CAPACITY {
            slot.dropped.fetch_add(1, Ordering::Relaxed);
            return;
        }
        // Account before publication; the worker cannot finish while this gate is held.
        // The queue retains its initial capacity, so this bounded push never allocates.
        slot.accepted.fetch_add(1, Ordering::Relaxed);
        slot.pending.fetch_add(1, Ordering::Relaxed);
        queue.push_back(sample);
    }

    /// Blocking disk read. Failed storage still exposes this process's bounded summaries.
    pub(crate) fn trace_list(&self, limit: u32) -> Result<Vec<TraceSummary>, String> {
        if limit == 0 {
            return Ok(Vec::new());
        }
        let disk = read_summaries(&self.0.path);
        let mut live = self.0.state.summaries()?;
        match disk {
            Ok(stored) => {
                for summary in stored {
                    if !live.iter().any(|s| s.id == summary.id) {
                        live.push(summary);
                    }
                }
            }
            Err(_) if !live.is_empty() => {}
            Err(_) if !self.0.path.exists() => {}
            Err(_) => return Err("trace_unavailable".into()),
        }
        live.truncate((limit as usize).min(KEEP));
        Ok(live)
    }

    /// Read summary and samples from one SQLite snapshot. Source times are sorted,
    /// never replaced by queue/commit timestamps. Repeated transmit seqs survive.
    pub(crate) fn trace_read(&self, id: &str) -> Result<TraceRecording, String> {
        if uuid::Uuid::parse_str(id).is_err() {
            return Err("trace_not_found".into());
        }
        // Snapshot memory before opening the read transaction: a newer terminal
        // in-memory state cannot make an older, partial disk snapshot look complete.
        let live = self.0.state.summaries()?.into_iter().find(|s| s.id == id);
        let stored = (|| -> Result<_, String> {
            let mut conn = open_reader(&self.0.path).map_err(|_| "trace_unavailable")?;
            let tx = conn.transaction().map_err(|_| "trace_unavailable")?;
            let summary: Option<String> = tx
                .query_row("SELECT summary FROM traces WHERE id=?1", [id], |r| r.get(0))
                .optional()
                .map_err(|_| "trace_unavailable")?;
            let Some(summary) = summary else {
                return Ok(None);
            };
            let summary: TraceSummary =
                serde_json::from_str(&summary).map_err(|_| "trace_unavailable")?;
            let mut stmt = tx
                .prepare("SELECT payload FROM samples WHERE trace_id=?1 ORDER BY at_us,position")
                .map_err(|_| "trace_unavailable")?;
            let rows = stmt
                .query_map([id], |r| r.get::<_, String>(0))
                .map_err(|_| "trace_unavailable")?;
            let mut samples = Vec::new();
            for row in rows {
                samples.push(
                    serde_json::from_str::<TraceSample>(&row.map_err(|_| "trace_unavailable")?)
                        .map_err(|_| "trace_unavailable")?,
                );
            }
            Ok(Some(TraceRecording {
                trace: summary,
                samples,
            }))
        })();
        match stored {
            Ok(Some(mut recording)) => {
                if !matches!(recording.trace.status.as_str(), "complete" | "failed")
                    && let Some(mut summary) = live
                {
                    let written = recording.samples.len() as u64;
                    summary.accepted = summary.accepted.max(written);
                    summary.pending = summary
                        .pending
                        .saturating_add(summary.written)
                        .saturating_sub(written);
                    summary.written = written;
                    recording.trace = summary;
                }
                Ok(recording)
            }
            result => {
                if let Some(mut summary) = live {
                    if summary.written > 0 {
                        return Err("trace_unavailable".into());
                    }
                    // No committed samples yet. Pending and errors explain the empty body.
                    summary.written = 0;
                    return Ok(TraceRecording {
                        trace: summary,
                        samples: Vec::new(),
                    });
                }
                match result {
                    Ok(None) => Err("trace_not_found".into()),
                    _ => Err("trace_unavailable".into()),
                }
            }
        }
    }

    pub(crate) fn request_shutdown(&self) {
        let state = &self.0.state;
        state.lifecycle.fetch_or(CLOSED, Ordering::SeqCst);
        for slot in &state.slots {
            slot.gate.fetch_or(CLOSED, Ordering::SeqCst);
        }
        state.active.store(0, Ordering::SeqCst);
    }
    pub(crate) fn abort(&self) {
        self.0.state.abort.store(true, Ordering::Release);
    }
    pub(crate) fn done(&self) -> bool {
        self.0.state.done.load(Ordering::Acquire)
    }
    pub(crate) fn clean(&self) -> bool {
        self.0.state.clean.load(Ordering::Acquire)
    }
}

fn nanos(origin: Instant, at: Instant) -> Option<u64> {
    at.checked_duration_since(origin)
        .map(|t| t.as_nanos().min(u64::MAX as u128) as u64)
}

fn snapshot(entry: &Entry, slot: &Slot) -> TraceSummary {
    let mut summary = entry.summary.clone();
    summary.status = match slot.phase.load(Ordering::Acquire) {
        RECORDING => "recording",
        WRITING => "writing",
        COMPLETE => "complete",
        _ => "failed",
    }
    .into();
    summary.accepted = slot.accepted.load(Ordering::Relaxed);
    summary.written = slot.written.load(Ordering::Relaxed);
    summary.dropped = slot.dropped.load(Ordering::Relaxed);
    summary.pending = slot.pending.load(Ordering::Relaxed);
    summary.errors = slot.errors.load(Ordering::Relaxed);
    summary
}

fn open_reader(path: &Path) -> rusqlite::Result<Connection> {
    let conn = Connection::open_with_flags(
        path,
        OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_NO_MUTEX,
    )?;
    conn.busy_timeout(BUSY)?;
    Ok(conn)
}

fn read_summaries(path: &Path) -> Result<Vec<TraceSummary>, String> {
    let conn = open_reader(path).map_err(|_| "trace_unavailable")?;
    let mut stmt = conn
        .prepare("SELECT summary FROM traces ORDER BY ordinal DESC LIMIT 10")
        .map_err(|_| "trace_unavailable")?;
    let rows = stmt
        .query_map([], |r| r.get::<_, String>(0))
        .map_err(|_| "trace_unavailable")?;
    let mut summaries = Vec::new();
    for row in rows {
        summaries.push(
            serde_json::from_str(&row.map_err(|_| "trace_unavailable")?)
                .map_err(|_| "trace_unavailable")?,
        );
    }
    Ok(summaries)
}

fn open_writer(path: &Path) -> rusqlite::Result<Connection> {
    if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent)
            .map_err(|e| rusqlite::Error::ToSqlConversionFailure(Box::new(e)))?;
    }
    let conn = Connection::open(path)?;
    conn.busy_timeout(BUSY)?;
    conn.execute_batch(
        "PRAGMA journal_mode=WAL;
         PRAGMA synchronous=NORMAL;
         PRAGMA wal_autocheckpoint=0;
         PRAGMA foreign_keys=ON;
         CREATE TABLE IF NOT EXISTS traces (
             ordinal INTEGER PRIMARY KEY AUTOINCREMENT,
             id TEXT NOT NULL UNIQUE,
             summary TEXT NOT NULL
         );
         CREATE TABLE IF NOT EXISTS samples (
             trace_id TEXT NOT NULL REFERENCES traces(id) ON DELETE CASCADE,
             position INTEGER NOT NULL,
             at_us INTEGER NOT NULL,
             payload TEXT NOT NULL,
             PRIMARY KEY(trace_id,position)
         );",
    )?;
    let mode: String = conn.query_row("PRAGMA journal_mode", [], |r| r.get(0))?;
    if mode != "wal" {
        return Err(rusqlite::Error::InvalidQuery);
    }
    Ok(conn)
}

fn encode(value: &impl Serialize) -> rusqlite::Result<String> {
    serde_json::to_string(value).map_err(|e| rusqlite::Error::ToSqlConversionFailure(Box::new(e)))
}

fn recover(conn: &mut Connection) -> rusqlite::Result<()> {
    let tx = conn.transaction()?;
    let summaries: Vec<(String, String)> = {
        let mut stmt = tx.prepare("SELECT id,summary FROM traces ORDER BY ordinal")?;
        stmt.query_map([], |r| Ok((r.get(0)?, r.get(1)?)))?
            .collect::<rusqlite::Result<_>>()?
    };
    for (id, payload) in summaries {
        let mut summary: TraceSummary = serde_json::from_str(&payload)
            .map_err(|e| rusqlite::Error::ToSqlConversionFailure(Box::new(e)))?;
        if matches!(summary.status.as_str(), "recording" | "writing") {
            let count: i64 = tx.query_row(
                "SELECT count(*) FROM samples WHERE trace_id=?1",
                [&id],
                |r| r.get(0),
            )?;
            let count = count.max(0) as u64;
            summary.accepted = summary.accepted.max(count);
            summary.written = count;
            summary.dropped += summary.accepted - count;
            summary.pending = 0;
            summary.errors += 1;
            summary.status = "failed".into();
            tx.execute(
                "UPDATE traces SET summary=?2 WHERE id=?1",
                params![id, encode(&summary)?],
            )?;
        }
    }
    tx.commit()
}

// Returns completion publications; they occur only after the transaction commits.
fn write_batch(
    conn: &mut Connection,
    entries: &[Entry],
    state: &State,
    batch: &[Queued],
) -> rusqlite::Result<Vec<(u64, u64, u8)>> {
    let tx = conn.transaction()?;
    let mut updates = Vec::with_capacity(KEEP);
    for entry in entries {
        let slot = state.slot(entry.token);
        // Only this writer publishes terminal phases. Check phase before token:
        // an already terminal slot may be reused concurrently, and observing the
        // new recording phase must be followed by checking its new token.
        if matches!(slot.phase.load(Ordering::Acquire), COMPLETE | FAILED)
            || slot.token.load(Ordering::Acquire) != entry.token
        {
            continue;
        }
        let count = batch.iter().filter(|q| q.token == entry.token).count() as u64;
        // Read the gate first: reading counters before a producer leaves could
        // otherwise publish complete using an obsolete zero pending count.
        let quiesced = slot.gate.load(Ordering::SeqCst) == CLOSED;
        let mut summary = snapshot(entry, slot);
        summary.written += count;
        summary.pending -= count;
        let terminal = quiesced && summary.pending == 0;
        let phase = if terminal {
            if slot.abandoned.load(Ordering::Relaxed) == 0 {
                COMPLETE
            } else {
                FAILED
            }
        } else {
            0
        };
        if phase != 0 {
            summary.status = if phase == COMPLETE {
                "complete"
            } else {
                "failed"
            }
            .into();
        }
        tx.execute(
            "INSERT INTO traces(id,summary) VALUES(?1,?2)
             ON CONFLICT(id) DO UPDATE SET summary=excluded.summary",
            params![summary.id, encode(&summary)?],
        )?;
        updates.push((entry.token, count, phase));
    }
    {
        let mut insert = tx.prepare_cached(
            "INSERT INTO samples(trace_id,position,at_us,payload) VALUES(?1,?2,?3,?4)",
        )?;
        for sample in batch {
            let entry = entries
                .iter()
                .find(|e| e.token == sample.token)
                .ok_or(rusqlite::Error::InvalidQuery)?;
            let payload = encode(&sample.sample)?;
            insert.execute(params![
                entry.summary.id,
                sample.position as i64,
                sample.sample.at_us as i64,
                payload
            ])?;
        }
    }
    tx.execute("DELETE FROM traces WHERE ordinal NOT IN (SELECT ordinal FROM traces ORDER BY ordinal DESC LIMIT 10)", [])?;
    tx.commit()?;
    Ok(updates)
}

fn abandon(state: &State, sample: Queued) {
    let slot = state.slot(sample.token);
    slot.abandoned.fetch_add(1, Ordering::Relaxed);
    slot.dropped.fetch_add(1, Ordering::Relaxed);
    slot.pending.fetch_sub(1, Ordering::Relaxed);
    state.clean.store(false, Ordering::Release);
}

fn finite(data: &TraceData) -> bool {
    match data {
        TraceData::Control {
            q_ref,
            dq_ref,
            ddq_ref,
            jerk_ref,
            tau_ff,
            q,
            dq,
            tau,
            voltage,
            servo_tuning,
            ..
        } => {
            [q_ref, dq_ref, ddq_ref, jerk_ref, tau_ff]
                .iter()
                .all(|a| a.iter().all(|x| x.is_finite()))
                && [q, dq, tau, voltage]
                    .iter()
                    .all(|a| a.iter().all(|x| x.is_none_or(f64::is_finite)))
                && servo_tuning.as_ref().is_none_or(|config| {
                    config.joints.iter().all(|j| {
                        [
                            j.kd,
                            j.velocity_ff,
                            j.coulomb,
                            j.viscous,
                            j.velocity_scale,
                            j.max_friction,
                        ]
                        .into_iter()
                        .chain(
                            [j.stribeck.positive, j.stribeck.negative]
                                .into_iter()
                                .flat_map(|curve| {
                                    [
                                        curve.coulomb,
                                        curve.static_friction,
                                        curve.viscous,
                                        curve.stribeck_velocity,
                                    ]
                                }),
                        )
                        .all(f64::is_finite)
                    })
                })
        }
        TraceData::Transmit {
            q_ref,
            dq_ref,
            tau_ff,
            kp,
            kd,
            tuning,
            ..
        } => {
            [q_ref, dq_ref, tau_ff, kp, kd]
                .iter()
                .all(|a| a.iter().all(|x| x.is_finite()))
                && tuning.as_ref().is_none_or(|sample| {
                    [
                        sample.dq_feedforward,
                        sample.tau_gravity,
                        sample.friction_requested,
                        sample.friction_applied,
                    ]
                    .into_iter()
                    .flatten()
                    .all(f64::is_finite)
                })
        }
        TraceData::Feedback {
            q,
            dq,
            tau,
            temperature,
            winding_temperature,
            voltage,
            ..
        } => {
            [q, dq, tau, temperature, voltage]
                .iter()
                .all(|x| x.is_finite())
                && winding_temperature.is_none_or(f64::is_finite)
        }
    }
}

fn writer(path: &Path, origin: Instant, state: Arc<State>) {
    let mut conn = None;
    let mut recovered = false;
    let mut batch = Vec::with_capacity(BATCH);
    let mut retry_at = Instant::now();
    let mut delay = Duration::from_millis(20);
    let mut checkpoint_at = Instant::now();
    loop {
        let mut entries: Vec<_> = state
            .entries
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
            .cloned()
            .collect();
        let closing = state.lifecycle.load(Ordering::SeqCst) & CLOSED != 0;
        let now = nanos(origin, Instant::now()).unwrap_or(0);
        for entry in &entries {
            let slot = state.slot(entry.token);
            if slot.phase.load(Ordering::Acquire) == RECORDING
                && slot.token.load(Ordering::Acquire) == entry.token
                && (closing || now >= slot.deadline_ns.load(Ordering::Relaxed))
            {
                state.stop(entry.token);
            }
        }
        if closing
            && entries.is_empty()
            && state.lifecycle.load(Ordering::SeqCst) == CLOSED
            && state.slots.iter().all(|s| {
                s.gate.load(Ordering::SeqCst) == CLOSED
                    && s.pending.load(Ordering::Relaxed) == 0
                    && matches!(s.phase.load(Ordering::Acquire), 0 | COMPLETE | FAILED)
            })
        {
            break;
        }
        if state.abort.load(Ordering::Acquire) {
            for sample in batch.drain(..) {
                abandon(&state, sample);
            }
            {
                let mut queue = state.queue.lock().unwrap_or_else(|p| p.into_inner());
                for sample in queue.drain(..) {
                    abandon(&state, sample);
                }
            }
            if state.lifecycle.load(Ordering::SeqCst) == CLOSED
                && state.slots.iter().all(|s| {
                    s.gate.load(Ordering::SeqCst) == CLOSED
                        && s.pending.load(Ordering::Relaxed) == 0
                })
            {
                // A start admitted just before close may have finished since the
                // loop's first metadata snapshot. No admitted starts remain now.
                entries = state
                    .entries
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .iter()
                    .cloned()
                    .collect();
                // Best effort only; failed persistence remains visible in memory and
                // is repaired as interrupted on reopen if this transaction cannot commit.
                for entry in &entries {
                    let slot = state.slot(entry.token);
                    if !matches!(slot.phase.load(Ordering::Acquire), COMPLETE | FAILED) {
                        slot.errors.fetch_add(1, Ordering::Relaxed);
                        slot.abandoned.fetch_add(1, Ordering::Relaxed);
                    }
                }
                if let Some(c) = conn.as_mut() {
                    let _ = write_batch(c, &entries, &state, &[]);
                }
                for entry in &entries {
                    let slot = state.slot(entry.token);
                    if !matches!(slot.phase.load(Ordering::Acquire), COMPLETE | FAILED) {
                        slot.phase.store(FAILED, Ordering::Release);
                    }
                }
                state.clean.store(false, Ordering::Release);
                break;
            }
            thread::sleep(POLL);
            continue;
        }
        {
            // Only the worker may wait for this lock. Release it before checking
            // samples, cloning metadata, serializing, sleeping, or touching SQLite.
            let mut queue = state.queue.lock().unwrap_or_else(|p| p.into_inner());
            while batch.len() < BATCH {
                let Some(sample) = queue.pop_front() else {
                    break;
                };
                batch.push(sample);
            }
        }
        if batch.is_empty() {
            // Polling needs no producer-side notification or wakeup lock.
            thread::sleep(POLL);
        }
        batch.retain(|sample| {
            if finite(&sample.sample.data) {
                return true;
            }
            state
                .slot(sample.token)
                .errors
                .fetch_add(1, Ordering::Relaxed);
            abandon(&state, *sample);
            false
        });
        if Instant::now() < retry_at {
            thread::sleep(POLL);
            continue;
        }
        // A capture can start between polls. Include its metadata
        // before inserting any of its queued samples into the foreign-key table.
        entries = state
            .entries
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
            .cloned()
            .collect();
        let operation = (|| {
            if conn.is_none() {
                conn = Some(open_writer(path)?);
            }
            let connection = conn.as_mut().unwrap();
            if !recovered {
                recover(connection)?;
                recovered = true;
            }
            let updates = write_batch(connection, &entries, &state, &batch)?;
            for (token, count, phase) in updates {
                let slot = state.slot(token);
                slot.written.fetch_add(count, Ordering::Relaxed);
                slot.pending.fetch_sub(count, Ordering::Relaxed);
                // Last access to this slot: publication permits start to reuse it.
                if phase != 0 {
                    slot.phase.store(phase, Ordering::Release);
                }
            }
            Ok::<_, rusqlite::Error>(())
        })();
        match operation {
            Ok(()) => {
                batch.clear();
                delay = Duration::from_millis(20);
            }
            Err(_) => {
                for entry in &entries {
                    let slot = state.slot(entry.token);
                    if !matches!(slot.phase.load(Ordering::Acquire), COMPLETE | FAILED)
                        && slot.token.load(Ordering::Acquire) == entry.token
                    {
                        slot.errors.fetch_add(1, Ordering::Relaxed);
                    }
                }
                conn = None;
                retry_at = Instant::now() + delay;
                delay = (delay * 2).min(Duration::from_millis(500));
                continue;
            }
        }
        if closing
            && state.lifecycle.load(Ordering::SeqCst) == CLOSED
            && state.slots.iter().all(|s| {
                s.gate.load(Ordering::SeqCst) == CLOSED
                    && s.pending.load(Ordering::Relaxed) == 0
                    && matches!(s.phase.load(Ordering::Acquire), 0 | COMPLETE | FAILED)
            })
        {
            break;
        }
        if checkpoint_at.elapsed() >= Duration::from_secs(5) {
            if let Some(c) = conn.as_ref() {
                let _ = c.execute_batch("PRAGMA wal_checkpoint(PASSIVE)");
            }
            checkpoint_at = Instant::now();
        }
    }
    // A connection close can still be in an OS call; shutdown/Drop never joins it.
    drop(conn);
    state.done.store(true, Ordering::Release);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::journal::Journal;

    fn wait_for(mut predicate: impl FnMut() -> bool) {
        let start = Instant::now();
        while !predicate() {
            assert!(
                start.elapsed() < Duration::from_secs(8),
                "trace worker timed out"
            );
            thread::sleep(Duration::from_millis(2));
        }
    }

    // Single-producer fixtures retry only rejected submissions. Count every
    // attempt so queue contention stays visible in their accounting assertions.
    fn retry_rejected(mut submit: impl FnMut() -> bool) -> u64 {
        let mut attempts = 0;
        wait_for(|| {
            attempts += 1;
            submit()
        });
        attempts
    }

    fn sample_until_queued(recorder: &Recorder, at: Instant, data: TraceData) -> u64 {
        let token = recorder.0.state.active.load(Ordering::SeqCst);
        let slot = recorder.0.state.slot(token);
        retry_rejected(|| {
            let accepted = slot.accepted.load(Ordering::Relaxed);
            recorder.sample(at, data);
            slot.accepted.load(Ordering::Relaxed) == accepted + 1
        })
    }

    fn journal_sample_until_queued(journal: &Journal, at: Instant, data: TraceData) -> u64 {
        retry_rejected(|| {
            let accepted = journal.trace_list(1).unwrap()[0].accepted;
            journal.sample(at, data);
            journal.trace_list(1).unwrap()[0].accepted == accepted + 1
        })
    }

    fn feedback() -> TraceData {
        TraceData::Feedback {
            joint: 3,
            q: 1.2,
            dq: -0.3,
            tau: 4.0,
            temperature: 33.0,
            winding_temperature: Some(46.0),
            voltage: 48.0,
            error: 7,
            mode: 1,
            timeout: true,
        }
    }

    fn control(seq: u64) -> TraceData {
        TraceData::Control {
            seq,
            servo_tuning: None,
            servo_tuning_revision: 0,
            generation: 5,
            enabled: true,
            stopping: false,
            motion: 2,
            axis: Some(6),
            q_ref: [0.1; 7],
            dq_ref: [0.2; 7],
            ddq_ref: [0.3; 7],
            jerk_ref: [0.4; 7],
            tau_ff: [0.5; 7],
            q: [Some(1.0); 7],
            dq: [None; 7],
            tau: [Some(2.0); 7],
            voltage: [Some(48.0); 7],
            age_us: [Some(123); 7],
        }
    }

    fn transmit(seq: u64) -> TraceData {
        TraceData::Transmit {
            seq,
            tuning: None,
            timing: None,
            mode: [1; 7],
            q_ref: [0.1; 7],
            dq_ref: [0.2; 7],
            tau_ff: [0.5; 7],
            q_wire: [0.1; 7],
            dq_wire: [0.2; 7],
            tau_wire: [0.5; 7],
            kp: [60.0; 7],
            kd: [10.0; 7],
            write_us: 88,
        }
    }

    fn start_time(recorder: &Recorder) -> Instant {
        let token = recorder.0.state.active.load(Ordering::SeqCst);
        recorder.0.origin
            + Duration::from_nanos(
                recorder
                    .0
                    .state
                    .slot(token)
                    .start_ns
                    .load(Ordering::Relaxed),
            )
    }

    fn summary(recorder: &Recorder, id: &str) -> TraceSummary {
        recorder
            .0
            .state
            .summaries()
            .unwrap()
            .into_iter()
            .find(|s| s.id == id)
            .unwrap()
    }

    fn close(recorder: &Recorder) {
        recorder.request_shutdown();
        wait_for(|| recorder.done());
        assert!(recorder.clean());
    }

    #[test]
    fn legacy_samples_decode_without_tuning_and_new_samples_round_trip() {
        let mut config = ServoTuning {
            friction_enabled: true,
            ..ServoTuning::default()
        };
        config.joints[3].coulomb = 0.3;
        let applied = config.apply([0.04; 7], [0.2; 7]).unwrap();
        let mut tx = transmit(5);
        if let TraceData::Transmit { tuning, .. } = &mut tx {
            *tuning = Some(ServoSample {
                revision: 8,
                dq_feedforward: applied.dq_feedforward,
                tau_gravity: [0.2; 7],
                friction_requested: applied.friction_requested,
                friction_applied: applied.friction_applied,
                clamped: applied.clamped,
            });
        }
        let mut ctrl = control(5);
        if let TraceData::Control {
            servo_tuning,
            servo_tuning_revision,
            ..
        } = &mut ctrl
        {
            *servo_tuning = Some(config);
            *servo_tuning_revision = 8;
        }
        let directory = tempfile::tempdir().unwrap();
        let recorder = Recorder::start(&directory.path().join("events.db")).unwrap();
        let trace = recorder.trace_start(1000, "right", None).unwrap();
        sample_until_queued(&recorder, Instant::now(), ctrl);
        sample_until_queued(&recorder, Instant::now(), tx);
        recorder.trace_stop().unwrap();
        wait_for(|| summary(&recorder, &trace.id).status == "complete");
        let recording = recorder.trace_read(&trace.id).unwrap();
        assert_eq!(recording.samples.len(), 2);
        for (sample, expected) in recording.samples.iter().zip([ctrl, tx]) {
            assert_eq!(
                serde_json::to_value(sample.data).unwrap(),
                serde_json::to_value(expected).unwrap()
            );
            let mut legacy = serde_json::to_value(sample).unwrap();
            let object = legacy.as_object_mut().unwrap();
            for field in ["servo_tuning", "servo_tuning_revision", "tuning", "timing"] {
                object.remove(field);
            }
            match serde_json::from_value::<TraceSample>(legacy).unwrap().data {
                TraceData::Control {
                    servo_tuning,
                    servo_tuning_revision,
                    ..
                } => {
                    assert!(servo_tuning.is_none());
                    assert_eq!(servo_tuning_revision, 0);
                }
                TraceData::Transmit { tuning, timing, .. } => {
                    assert!(tuning.is_none() && timing.is_none());
                }
                _ => panic!("wrong sample source"),
            }
        }
        close(&recorder);
    }

    #[test]
    fn legacy_feedback_trace_preserves_driver_temperature_and_absent_winding_temperature() {
        let legacy = serde_json::json!({
            "at_us": 17, "source": "feedback", "joint": 0,
            "q": 0.1, "dq": 0.2, "tau": 0.3, "temperature": -23.0,
            "voltage": 48.5, "error": 0, "mode": 1, "timeout": false
        });
        let sample: TraceSample = serde_json::from_value(legacy.clone()).unwrap();
        assert!(matches!(
            sample.data,
            TraceData::Feedback {
                temperature: -23.0,
                winding_temperature: None,
                ..
            }
        ));
        assert_eq!(serde_json::to_value(sample).unwrap(), legacy);
    }

    #[test]
    fn tuning_capture_finite_check_covers_configuration_and_transmit_arrays() {
        let mut ctrl = control(5);
        if let TraceData::Control { servo_tuning, .. } = &mut ctrl {
            *servo_tuning = Some(ServoTuning::default());
        }
        assert!(finite(&ctrl));
        if let TraceData::Control { servo_tuning, .. } = &mut ctrl {
            servo_tuning.as_mut().unwrap().joints[6].velocity_scale = f64::NAN;
        }
        assert!(!finite(&ctrl));
        for direction in [false, true] {
            for field in 0..4 {
                let mut sample = control(7);
                let mut config = ServoTuning::default();
                let curve = if direction {
                    &mut config.joints[3].stribeck.negative
                } else {
                    &mut config.joints[3].stribeck.positive
                };
                match field {
                    0 => curve.coulomb = f64::NAN,
                    1 => curve.static_friction = f64::INFINITY,
                    2 => curve.viscous = f64::NEG_INFINITY,
                    _ => curve.stribeck_velocity = f64::NAN,
                }
                if let TraceData::Control { servo_tuning, .. } = &mut sample {
                    *servo_tuning = Some(config);
                }
                assert!(
                    !finite(&sample),
                    "nonfinite directional parameters must not be serialized as null"
                );
            }
        }
        let mut tx = transmit(5);
        if let TraceData::Transmit { tuning, .. } = &mut tx {
            *tuning = Some(ServoSample {
                revision: 2,
                dq_feedforward: [0.0; 7],
                tau_gravity: [0.0; 7],
                friction_requested: [0.0; 7],
                friction_applied: [0.0; 7],
                clamped: [false; 7],
            });
        }
        assert!(finite(&tx));
        for field in 0..4 {
            let mut bad = tx;
            if let TraceData::Transmit { tuning, .. } = &mut bad {
                let t = tuning.as_mut().unwrap();
                let array = match field {
                    0 => &mut t.dq_feedforward,
                    1 => &mut t.tau_gravity,
                    2 => &mut t.friction_requested,
                    _ => &mut t.friction_applied,
                };
                array[6] = f64::INFINITY;
            }
            assert!(!finite(&bad));
        }
    }

    #[test]
    fn source_time_boundaries_duplicate_transmit_and_reopen() {
        let directory = tempfile::tempdir().unwrap();
        let event_path = directory.path().join("events.db");
        let recorder = Recorder::start(&event_path).unwrap();
        // The seven-joint configuration remains inline and Copy. Queue capacity,
        // tested separately, bounds memory without a stale per-record byte cap.
        fn assert_copy<T: Copy>() {}
        assert_copy::<TraceData>();
        for duration in [0, u64::MAX / 1_000_000 + 1, u64::MAX] {
            assert_eq!(
                recorder.trace_start(duration, "left", None).unwrap_err(),
                "invalid_trace_duration"
            );
        }
        let trace = recorder
            .trace_start(1000, "left", Some("offline-test"))
            .unwrap();
        assert_eq!(
            recorder.trace_start(1000, "left", None).unwrap_err(),
            "trace_active"
        );
        let start = start_time(&recorder);
        let end = start + Duration::from_millis(1000);
        recorder.sample(start - Duration::from_nanos(1), feedback());
        let mut submitted = sample_until_queued(&recorder, start, control(42));
        submitted += sample_until_queued(&recorder, start + Duration::from_micros(200), feedback());
        submitted +=
            sample_until_queued(&recorder, start + Duration::from_micros(100), transmit(42));
        submitted +=
            sample_until_queued(&recorder, start + Duration::from_micros(100), transmit(42));
        submitted += sample_until_queued(&recorder, end - Duration::from_nanos(1), feedback());
        recorder.sample(end, transmit(99));
        recorder.sample(end + Duration::from_micros(1), feedback());
        wait_for(|| summary(&recorder, &trace.id).status == "complete");
        let recording = recorder.trace_read(&trace.id).unwrap();
        assert_eq!(recording.trace.accepted, 5);
        assert_eq!(recording.trace.written, 5);
        assert_eq!(recording.trace.pending, 0);
        assert_eq!(recording.trace.errors, 0);
        assert_eq!(recording.trace.dropped, submitted - 5);
        assert_eq!(recording.trace.written + recording.trace.dropped, submitted);
        assert_eq!(
            recording
                .samples
                .iter()
                .map(|s| s.at_us)
                .collect::<Vec<_>>(),
            [0, 100, 100, 200, 999_999]
        );
        assert_eq!(
            recording
                .samples
                .iter()
                .filter(|s| matches!(s.data, TraceData::Transmit { seq: 42, .. }))
                .count(),
            2
        );
        let json = serde_json::to_value(&recording).unwrap();
        assert_eq!(json["samples"][0]["source"], "control");
        assert_eq!(json["samples"][0]["generation"], 5);
        assert_eq!(json["samples"][0]["age_us"][0], 123);
        assert!(json["samples"][0]["dq"][0].is_null());
        assert_eq!(json["samples"][1]["write_us"], 88);
        assert_eq!(
            json["samples"][1]["mode"],
            serde_json::json!([1, 1, 1, 1, 1, 1, 1])
        );
        assert_eq!(json["samples"][3]["temperature"], 33.0);
        assert_eq!(json["samples"][3]["winding_temperature"], 46.0);
        assert_eq!(json["samples"][3]["error"], 7);
        assert!(json["samples"][3]["timeout"].as_bool().unwrap());
        assert!(recorder.trace_stop().unwrap().is_none());
        close(&recorder);
        drop(recorder);
        let reopened = Recorder::start(&event_path).unwrap();
        assert_eq!(reopened.trace_list(20).unwrap()[0].id, trace.id);
        assert_eq!(reopened.trace_read(&trace.id).unwrap().samples.len(), 5);
        assert!(reopened.trace_list(0).unwrap().is_empty());
        close(&reopened);
    }

    #[test]
    fn resource_limits_accept_short_and_long_recordings() {
        let directory = tempfile::tempdir().unwrap();
        let recorder = Recorder::start(&directory.path().join("events.db")).unwrap();
        for duration_ms in [25, 60_000] {
            let trace = recorder.trace_start(duration_ms, "right", None).unwrap();
            let start = start_time(&recorder);
            let submitted = sample_until_queued(
                &recorder,
                start + Duration::from_millis(duration_ms - 1),
                feedback(),
            );
            recorder.sample(start + Duration::from_millis(duration_ms), feedback());
            wait_for(|| summary(&recorder, &trace.id).status == "complete");
            let recording = recorder.trace_read(&trace.id).unwrap();
            assert_eq!(recording.trace.duration_ms, duration_ms);
            assert_eq!(recording.trace.accepted, 1);
            assert_eq!(recording.trace.written, 1);
            assert_eq!(recording.trace.dropped, submitted - 1);
            assert_eq!(recording.samples.len(), 1);
            assert_eq!(recording.samples[0].at_us, (duration_ms - 1) * 1000);
        }
        close(&recorder);
    }

    #[test]
    fn deadline_ends_without_another_sample_and_stop_is_idempotent() {
        let directory = tempfile::tempdir().unwrap();
        let recorder = Recorder::start(&directory.path().join("events.db")).unwrap();
        let trace = recorder.trace_start(1000, "right", None).unwrap();
        let submitted = sample_until_queued(&recorder, Instant::now(), feedback());
        wait_for(|| summary(&recorder, &trace.id).status == "complete");
        let completed = summary(&recorder, &trace.id);
        assert_eq!(completed.accepted, 1);
        assert_eq!(completed.written, 1);
        assert_eq!(completed.dropped, submitted - 1);
        assert!(recorder.trace_stop().unwrap().is_none());
        let second = recorder.trace_start(20_000, "right", None).unwrap();
        let stopped = recorder.trace_stop().unwrap().unwrap();
        assert_eq!(stopped.id, second.id);
        assert!(matches!(stopped.status.as_str(), "writing" | "complete"));
        assert!(recorder.trace_stop().unwrap().is_none());
        close(&recorder);
        assert_eq!(summary(&recorder, &second.id).status, "complete");
    }

    #[test]
    fn queue_contention_drops_without_waiting_for_the_writer() {
        let directory = tempfile::tempdir().unwrap();
        let recorder = Recorder::start(&directory.path().join("events.db")).unwrap();
        let trace = recorder.trace_start(20_000, "left", None).unwrap();
        let at = start_time(&recorder);
        // Hold the same mutex used by the writer throughout the completion wait.
        let queue = recorder.0.state.queue.lock().unwrap();
        let worker = recorder.clone();
        let (completed_tx, completed_rx) = std::sync::mpsc::channel();
        let producer = thread::spawn(move || {
            for _ in 0..100 {
                worker.sample(at, feedback());
            }
            completed_tx.send(()).unwrap();
        });
        let completed = completed_rx.recv_timeout(Duration::from_secs(1));
        drop(queue);
        producer.join().unwrap();
        completed.expect("producer waited for the queue mutex");
        let rejected = summary(&recorder, &trace.id);
        assert_eq!(rejected.accepted, 0);
        assert_eq!(rejected.pending, 0);
        assert_eq!(rejected.dropped, 100);
        recorder.trace_stop().unwrap();
        close(&recorder);
        let recording = recorder.trace_read(&trace.id).unwrap();
        assert_eq!(recording.trace.status, "complete");
        assert_eq!(recording.trace.accepted, recording.trace.written);
        assert_eq!(recording.trace.pending, 0);
        assert_eq!(recording.trace.dropped, 100);
        assert!(recording.samples.is_empty());
    }

    #[test]
    fn queue_overflow_drops_without_waiting_or_growing_and_then_drains() {
        let directory = tempfile::tempdir().unwrap();
        // Delay starting the writer so every submission observes a deterministic queue.
        let recorder = Recorder(Arc::new(Shared {
            origin: Instant::now(),
            path: directory.path().join("traces.sqlite3"),
            state: Arc::new(State::default()),
        }));
        let trace = recorder.trace_start(20_000, "left", None).unwrap();
        let capacity = recorder.0.state.queue.lock().unwrap().capacity();
        let at = start_time(&recorder);
        let worker = recorder.clone();
        let (completed_tx, completed_rx) = std::sync::mpsc::channel();
        let producer = thread::spawn(move || {
            for _ in 0..CAPACITY + 100 {
                worker.sample(at, feedback());
            }
            completed_tx.send(()).unwrap();
        });
        let completed = completed_rx.recv_timeout(Duration::from_secs(1));
        // Start draining even if the producer timed out, so a regression can unwind.
        let worker_state = recorder.0.state.clone();
        let worker_path = recorder.0.path.clone();
        let origin = recorder.0.origin;
        let writer = thread::spawn(move || writer(&worker_path, origin, worker_state));
        producer.join().unwrap();
        close(&recorder);
        writer.join().unwrap();
        completed.expect("producer waited for queue space");
        let recording = recorder.trace_read(&trace.id).unwrap();
        assert_eq!(recording.trace.status, "complete");
        assert_eq!(recording.trace.accepted, CAPACITY as u64);
        assert_eq!(recording.trace.written, CAPACITY as u64);
        assert_eq!(recording.trace.pending, 0);
        assert_eq!(recording.trace.dropped, 100);
        assert_eq!(recording.samples.len(), CAPACITY);
        let queue = recorder.0.state.queue.lock().unwrap();
        assert!(queue.is_empty());
        assert_eq!(queue.capacity(), capacity);
    }

    #[test]
    fn unwritable_full_queue_and_registry_lock_never_wait_on_producer() {
        let directory = tempfile::tempdir().unwrap();
        let blocker = directory.path().join("blocked");
        std::fs::write(&blocker, "not a directory").unwrap();
        let recorder = Recorder::start(&blocker.join("events.db")).unwrap();
        let trace = recorder.trace_start(20_000, "left", None).unwrap();
        wait_for(|| summary(&recorder, &trace.id).errors > 0);
        let at = start_time(&recorder);
        // Deliberately hold the metadata mutex across every submission. A producer
        // that ever tries to acquire it deadlocks this test.
        let held = recorder.0.state.entries.lock().unwrap();
        let begin = Instant::now();
        for _ in 0..20_000 {
            recorder.sample(at, feedback());
        }
        assert!(begin.elapsed() < Duration::from_secs(1));
        drop(held);
        let saturated = summary(&recorder, &trace.id);
        assert!(saturated.pending <= (CAPACITY + BATCH) as u64);
        assert!(saturated.dropped > 0);
        assert_eq!(saturated.accepted + saturated.dropped, 20_000);
        assert_eq!(saturated.accepted, saturated.pending);
        recorder.trace_stop().unwrap();
        assert_eq!(summary(&recorder, &trace.id).status, "writing");
        std::fs::remove_file(&blocker).unwrap();
        wait_for(|| summary(&recorder, &trace.id).status == "complete");
        let recovered = recorder.trace_read(&trace.id).unwrap();
        assert_eq!(recovered.samples.len() as u64, saturated.accepted);
        assert_eq!(recovered.trace.pending, 0);
        assert!(recovered.trace.errors > 0);
        close(&recorder);
    }

    #[test]
    fn locked_trace_database_does_not_contend_with_events_and_recovers_once() {
        let directory = tempfile::tempdir().unwrap();
        let journal = Journal::start(directory.path().join("events.db")).unwrap();
        let trace = journal.trace_start(20_000, "left", None).unwrap();
        let mut submitted = journal_sample_until_queued(&journal, Instant::now(), control(1));
        wait_for(|| journal.trace_list(1).unwrap()[0].written == 1);
        let locker = Connection::open(directory.path().join("events.db.traces.sqlite3")).unwrap();
        locker.execute_batch("BEGIN IMMEDIATE").unwrap();
        submitted += journal_sample_until_queued(&journal, Instant::now(), control(2));
        let begin = Instant::now();
        for seq in 3..20_002 {
            journal.sample(Instant::now(), control(seq));
            submitted += 1;
        }
        assert!(begin.elapsed() < Duration::from_secs(1));
        let event_attempts = retry_rejected(|| {
            let dropped = journal.stats().dropped;
            journal.log(
                "info",
                "test",
                "trace_locked",
                "events remain independent",
                None,
            );
            journal.stats().dropped == dropped
        });
        wait_for(|| journal.stats().written == 1 && journal.trace_list(1).unwrap()[0].errors > 0);
        assert_eq!(journal.stats().pending, 0);
        assert_eq!(journal.stats().errors, 0);
        assert_eq!(journal.stats().dropped, event_attempts - 1);
        let stats = journal.stats();
        assert_eq!(
            stats.written + stats.dropped + stats.pending,
            event_attempts
        );
        let stopped = journal.trace_stop().unwrap().unwrap();
        assert_eq!(stopped.status, "writing");
        assert!(stopped.pending > 0);
        locker.execute_batch("ROLLBACK").unwrap();
        wait_for(|| journal.trace_list(1).unwrap()[0].status == "complete");
        let recording = journal.trace_read(&trace.id).unwrap();
        assert_eq!(recording.trace.accepted, recording.trace.written);
        assert_eq!(recording.samples.len() as u64, recording.trace.written);
        assert_eq!(recording.trace.written + recording.trace.dropped, submitted);
        let mut seqs = std::collections::HashSet::new();
        for sample in recording.samples {
            let TraceData::Control { seq, .. } = sample.data else {
                panic!("unexpected source")
            };
            assert!(seqs.insert(seq));
        }
        assert!(journal.shutdown(Duration::from_secs(2)));
    }

    #[test]
    fn retention_is_bounded_on_disk_and_after_reopen() {
        let directory = tempfile::tempdir().unwrap();
        let event_path = directory.path().join("events.db");
        let recorder = Recorder::start(&event_path).unwrap();
        let mut ids = Vec::new();
        for seq in 0..14 {
            let trace = recorder.trace_start(1000, "right", None).unwrap();
            let submitted = sample_until_queued(&recorder, Instant::now(), control(seq));
            recorder.trace_stop().unwrap();
            wait_for(|| summary(&recorder, &trace.id).status == "complete");
            let completed = summary(&recorder, &trace.id);
            assert_eq!(completed.accepted, 1);
            assert_eq!(completed.written, 1);
            assert_eq!(completed.pending, 0);
            assert_eq!(completed.dropped, submitted - 1);
            ids.push(trace.id);
        }
        assert_eq!(recorder.0.state.entries.lock().unwrap().len(), KEEP);
        let list = recorder.trace_list(u32::MAX).unwrap();
        assert_eq!(list.len(), KEEP);
        assert_eq!(list[0].id, ids[13]);
        assert_eq!(list[9].id, ids[4]);
        assert_eq!(recorder.trace_read(&ids[0]).unwrap_err(), "trace_not_found");
        close(&recorder);
        let reader = open_reader(&recorder.0.path).unwrap();
        assert_eq!(
            reader
                .query_row("SELECT count(*) FROM traces", [], |r| r.get::<_, i64>(0))
                .unwrap(),
            10
        );
        assert_eq!(
            reader
                .query_row("SELECT count(*) FROM samples", [], |r| r.get::<_, i64>(0))
                .unwrap(),
            10
        );
        drop(reader);
        drop(recorder);
        let reopened = Recorder::start(&event_path).unwrap();
        assert_eq!(reopened.trace_list(20).unwrap().len(), KEEP);
        assert_eq!(reopened.trace_list(20).unwrap()[0].id, ids[13]);
        close(&reopened);
    }

    #[test]
    fn interruption_recovery_and_shutdown_abandonment_are_visible() {
        let directory = tempfile::tempdir().unwrap();
        let event_path = directory.path().join("events.db");
        let recorder = Recorder::start(&event_path).unwrap();
        let trace = recorder.trace_start(20_000, "left", None).unwrap();
        let mut submitted = sample_until_queued(&recorder, Instant::now(), feedback());
        wait_for(|| summary(&recorder, &trace.id).written == 1);
        let locker = Connection::open(&recorder.0.path).unwrap();
        locker.execute_batch("BEGIN IMMEDIATE").unwrap();
        for _ in 0..100 {
            submitted += sample_until_queued(&recorder, Instant::now(), feedback());
        }
        wait_for(|| summary(&recorder, &trace.id).errors > 0);
        recorder.request_shutdown();
        recorder.abort();
        wait_for(|| recorder.done());
        let failed = summary(&recorder, &trace.id);
        assert_eq!(failed.status, "failed");
        assert_eq!(failed.pending, 0);
        assert_eq!(failed.written, 1);
        assert_eq!(failed.accepted, 101);
        assert_eq!(failed.dropped, submitted - 1);
        assert_eq!(failed.written + failed.dropped + failed.pending, submitted);
        assert!(failed.errors > 0);
        locker.execute_batch("ROLLBACK").unwrap();
        let start = Instant::now();
        drop(recorder);
        assert!(start.elapsed() < Duration::from_millis(100));
        let reopened = Recorder::start(&event_path).unwrap();
        wait_for(|| {
            reopened
                .trace_list(1)
                .is_ok_and(|s| s[0].status == "failed")
        });
        let recovered = reopened.trace_read(&trace.id).unwrap();
        assert_eq!(recovered.trace.status, "failed");
        assert_eq!(recovered.trace.written, 1);
        assert_eq!(recovered.trace.pending, 0);
        assert_eq!(recovered.samples.len(), 1);
        close(&reopened);
    }

    #[test]
    fn nonfinite_feedback_does_not_poison_the_capture() {
        let directory = tempfile::tempdir().unwrap();
        let recorder = Recorder::start(&directory.path().join("events.db")).unwrap();
        let trace = recorder.trace_start(1000, "left", None).unwrap();
        let mut bad = feedback();
        if let TraceData::Feedback { q, .. } = &mut bad {
            *q = f64::NAN;
        }
        let mut submitted = sample_until_queued(&recorder, Instant::now(), bad);
        submitted += sample_until_queued(&recorder, Instant::now(), control(3));
        recorder.trace_stop().unwrap();
        wait_for(|| summary(&recorder, &trace.id).status == "failed");
        let recording = recorder.trace_read(&trace.id).unwrap();
        assert_eq!(recording.trace.accepted, 2);
        assert_eq!(recording.trace.written, 1);
        assert_eq!(recording.trace.pending, 0);
        assert_eq!(recording.trace.dropped, submitted - 1);
        assert_eq!(recording.trace.written + recording.trace.dropped, submitted);
        assert_eq!(recording.trace.errors, 1);
        assert_eq!(recording.samples.len(), 1);
        recorder.request_shutdown();
        wait_for(|| recorder.done());
    }

    #[test]
    fn concurrent_producers_stop_and_reuse_slots_preserve_final_accounting() {
        let directory = tempfile::tempdir().unwrap();
        let recorder = Recorder::start(&directory.path().join("events.db")).unwrap();
        let running = Arc::new(AtomicBool::new(true));
        let producers: Vec<_> = (0..4)
            .map(|_| {
                let recorder = recorder.clone();
                let running = running.clone();
                thread::spawn(move || {
                    while running.load(Ordering::Relaxed) {
                        recorder.sample(Instant::now(), feedback());
                        thread::yield_now();
                    }
                })
            })
            .collect();
        for _ in 0..24 {
            let trace = recorder.trace_start(1000, "left", None).unwrap();
            thread::sleep(Duration::from_millis(2));
            recorder.trace_stop().unwrap();
            wait_for(|| summary(&recorder, &trace.id).status == "complete");
            let data = recorder.trace_read(&trace.id).unwrap();
            assert_eq!(data.trace.accepted, data.trace.written);
            assert_eq!(data.trace.pending, 0);
            assert_eq!(data.samples.len() as u64, data.trace.written);
        }
        running.store(false, Ordering::Relaxed);
        for producer in producers {
            producer.join().unwrap();
        }
        close(&recorder);
    }

    #[test]
    fn queued_segments_and_per_segment_submission_budget_are_bounded() {
        let directory = tempfile::tempdir().unwrap();
        let blocker = directory.path().join("blocked");
        std::fs::write(&blocker, "not a directory").unwrap();
        let recorder = Recorder::start(&blocker.join("events.db")).unwrap();
        for _ in 0..KEEP {
            recorder.trace_start(20_000, "left", None).unwrap();
            sample_until_queued(&recorder, Instant::now(), feedback());
            recorder.trace_stop().unwrap();
        }
        assert_eq!(
            recorder.trace_start(1000, "left", None).unwrap_err(),
            "trace_capacity"
        );
        assert_eq!(recorder.trace_list(20).unwrap().len(), KEEP);
        std::fs::remove_file(&blocker).unwrap();
        wait_for(|| {
            recorder
                .0
                .state
                .summaries()
                .unwrap()
                .iter()
                .all(|s| s.status == "complete")
        });
        let trace = recorder.trace_start(20_000, "left", None).unwrap();
        let at = start_time(&recorder);
        for _ in 0..MAX_SAMPLES + 100 {
            recorder.sample(at, feedback());
        }
        wait_for(|| summary(&recorder, &trace.id).pending == 0);
        let before = summary(&recorder, &trace.id);
        assert_eq!(before.accepted + before.dropped, MAX_SAMPLES + 100);
        // Queue is now empty; an exhausted capture still cannot grow on disk.
        recorder.sample(at, feedback());
        let after = summary(&recorder, &trace.id);
        assert_eq!(after.accepted, before.accepted);
        assert_eq!(after.dropped, before.dropped + 1);
        assert!(after.accepted <= MAX_SAMPLES);
        recorder.trace_stop().unwrap();
        close(&recorder);
    }

    #[test]
    fn journal_shutdown_and_drop_do_not_wait_for_a_locked_trace_database() {
        let directory = tempfile::tempdir().unwrap();
        let journal = Journal::start(directory.path().join("events.db")).unwrap();
        journal.trace_start(20_000, "left", None).unwrap();
        let mut submitted = journal_sample_until_queued(&journal, Instant::now(), feedback());
        wait_for(|| journal.trace_list(1).unwrap()[0].written == 1);
        let locker = Connection::open(directory.path().join("events.db.traces.sqlite3")).unwrap();
        locker.execute_batch("BEGIN IMMEDIATE").unwrap();
        for _ in 0..100 {
            submitted += journal_sample_until_queued(&journal, Instant::now(), feedback());
        }
        wait_for(|| journal.trace_list(1).unwrap()[0].errors > 0);
        let start = Instant::now();
        assert!(!journal.shutdown(Duration::from_millis(15)));
        assert!(start.elapsed() < Duration::from_millis(250));
        wait_for(|| journal.trace_list(1).unwrap()[0].status == "failed");
        let summary = journal.trace_list(1).unwrap().remove(0);
        assert_eq!(summary.pending, 0);
        assert_eq!(summary.accepted, 101);
        assert_eq!(summary.written, 1);
        assert_eq!(summary.dropped, submitted - 1);
        assert_eq!(
            summary.written + summary.dropped + summary.pending,
            submitted
        );
        assert_eq!(journal.stats().written, 0);
        assert_eq!(journal.stats().dropped, 0);
        assert_eq!(journal.stats().errors, 0);
        let start = Instant::now();
        drop(journal);
        assert!(start.elapsed() < Duration::from_millis(100));
        locker.execute_batch("ROLLBACK").unwrap();
    }

    #[test]
    fn close_orders_against_a_start_waiting_for_metadata() {
        let directory = tempfile::tempdir().unwrap();
        let blocker = directory.path().join("blocked");
        std::fs::write(&blocker, "not a directory").unwrap();
        let recorder = Recorder::start(&blocker.join("events.db")).unwrap();
        let metadata = recorder.0.state.entries.lock().unwrap();
        let worker = recorder.clone();
        let starter = thread::spawn(move || worker.trace_start(1000, "left", None));
        wait_for(|| recorder.0.state.lifecycle.load(Ordering::SeqCst) == 1);
        recorder.request_shutdown();
        assert!(!recorder.done());
        drop(metadata);
        assert_eq!(starter.join().unwrap().unwrap_err(), "closing");
        wait_for(|| recorder.done());
        assert!(recorder.clean());
        assert!(recorder.trace_stop().unwrap().is_none());
        assert_eq!(
            recorder.trace_start(1000, "left", None).unwrap_err(),
            "closing"
        );
    }
}
