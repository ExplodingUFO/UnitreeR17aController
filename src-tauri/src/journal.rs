//! Bounded event journal with an independent capture worker.
//! `log` accepts summaries, never credentials or per-frame telemetry; use `sample` for frames.
//! Queue admission tries a mutex once; contention and overflow count as drops.
//! Disk readers must run within `server`'s spawn_blocking boundary.
pub use crate::trace::{TraceData, TraceRecording, TraceSummary};
use crate::{
    calibration::Report,
    protocol::motor_fault_description,
    teaching::{Capture, Summary as TeachingSummary, Trajectory},
    trace::Recorder,
};
use rusqlite::{Connection, OpenFlags, OptionalExtension, functions::FunctionFlags, params};
use serde::{Deserialize, Serialize};
use std::{
    collections::VecDeque,
    path::{Path, PathBuf},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering},
    },
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

const CAPACITY: usize = 1024;
const CALIBRATION_CAPACITY: usize = 8;
const TEACHING_CAPACITY: usize = 8;
const BATCH_SIZE: usize = 128;
const CLOSED: usize = 1 << (usize::BITS - 1);
const POLL: Duration = Duration::from_millis(10);
const BUSY: Duration = Duration::from_millis(40);

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct JournalStats {
    pub pending: u64,
    /// Successfully committed events in this process, including subsequently pruned events.
    pub written: u64,
    /// Queue contention, poisoning, overflow, calls after close, and shutdown abandonment.
    pub dropped: u64,
    /// Failed background database operations (not the number of affected records).
    pub errors: u64,
    pub healthy: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LogRecord {
    pub id: i64,
    pub ts_ms: i64,
    pub level: String,
    pub source: String,
    pub code: String,
    pub message: String,
    pub request_id: Option<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct LogPageQuery {
    pub page: u32,
    pub page_size: u32,
    pub search: String,
    pub through_id: Option<i64>,
    pub level: Option<String>,
    pub source: Option<String>,
    pub code: Option<String>,
    pub from_ms: Option<i64>,
    pub to_ms: Option<i64>,
}

impl Default for LogPageQuery {
    fn default() -> Self {
        Self {
            page: 1,
            page_size: 50,
            search: String::new(),
            through_id: None,
            level: None,
            source: None,
            code: None,
            from_ms: None,
            to_ms: None,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LogPage {
    pub items: Vec<LogRecord>,
    pub page: u32,
    pub page_size: u32,
    pub total: u64,
    pub total_pages: u32,
    pub through_id: i64,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct LogFilters {
    pub levels: Vec<String>,
    pub sources: Vec<String>,
    pub codes: Vec<String>,
}

struct TeachingQueue {
    queue: VecDeque<Arc<Capture>>,
    // Includes the writer's in-flight capture. Keep each admission distinct so
    // committing an older upsert cannot remove a newer capture with the same ID.
    uncommitted: Vec<Arc<Capture>>,
}

impl Default for TeachingQueue {
    fn default() -> Self {
        Self {
            queue: VecDeque::with_capacity(TEACHING_CAPACITY),
            uncommitted: Vec::with_capacity(TEACHING_CAPACITY + 1),
        }
    }
}

struct State {
    queue: Mutex<VecDeque<LogRecord>>,
    calibrations: Mutex<VecDeque<Report>>,
    teachings: Mutex<TeachingQueue>,
    // One atomic admission gate orders close against producers already in flight.
    gate: AtomicUsize,
    pending: AtomicU64,
    written: AtomicU64,
    dropped: AtomicU64,
    errors: AtomicU64,
    calibration_pending: AtomicU64,
    calibration_dropped: AtomicU64,
    calibration_errors: AtomicU64,
    teaching_pending: AtomicU64,
    teaching_dropped: AtomicU64,
    teaching_errors: AtomicU64,
    #[cfg(test)]
    pause_retirement: AtomicBool,
    healthy: AtomicBool,
    abort: AtomicBool,
    done: AtomicBool,
}

impl Default for State {
    fn default() -> Self {
        Self {
            queue: Mutex::new(VecDeque::with_capacity(CAPACITY)),
            calibrations: Mutex::new(VecDeque::with_capacity(CALIBRATION_CAPACITY)),
            teachings: Mutex::new(TeachingQueue::default()),
            gate: AtomicUsize::new(0),
            pending: AtomicU64::new(0),
            written: AtomicU64::new(0),
            dropped: AtomicU64::new(0),
            errors: AtomicU64::new(0),
            calibration_pending: AtomicU64::new(0),
            calibration_dropped: AtomicU64::new(0),
            calibration_errors: AtomicU64::new(0),
            teaching_pending: AtomicU64::new(0),
            teaching_dropped: AtomicU64::new(0),
            teaching_errors: AtomicU64::new(0),
            #[cfg(test)]
            pause_retirement: AtomicBool::new(false),
            healthy: AtomicBool::new(false),
            abort: AtomicBool::new(false),
            done: AtomicBool::new(false),
        }
    }
}

struct Admission<'a>(&'a AtomicUsize);

impl Drop for Admission<'_> {
    fn drop(&mut self) {
        self.0.fetch_sub(1, Ordering::SeqCst);
    }
}

struct Shared {
    path: PathBuf,
    state: Arc<State>,
    traces: Recorder,
}

impl Drop for Shared {
    fn drop(&mut self) {
        self.state.gate.fetch_or(CLOSED, Ordering::SeqCst);
        self.state.abort.store(true, Ordering::Release);
        // The writer is detached; neither dropping a clone nor the last owner joins it.
    }
}

#[derive(Clone)]
pub struct Journal(Arc<Shared>);

impl Journal {
    /// Starts the worker only. Opening the directory, database and schema is asynchronous;
    /// a path failure is reflected in stats and retried, rather than failing the controller.
    pub fn start(path: PathBuf) -> Result<Self, String> {
        let traces = Recorder::start(&path)?;
        let state = Arc::new(State::default());
        let worker_state = state.clone();
        let worker_path = path.clone();
        thread::Builder::new()
            .name("event-journal".into())
            .spawn(move || writer(&worker_path, worker_state))
            .map_err(|e| format!("start journal writer: {e}"))?;
        Ok(Self(Arc::new(Shared {
            path,
            state,
            traces,
        })))
    }

    /// Bounded in-memory work only; fields are truncated at UTF-8 boundaries.
    /// Callers supply event summaries and request IDs, never tokens or lease IDs.
    /// String allocation is bounded; queue admission tries its mutex once and drops
    /// on contention, poisoning, or overflow without a queue wakeup or database work.
    pub fn log(
        &self,
        level: &str,
        source: &str,
        code: &str,
        message: &str,
        request_id: Option<&str>,
    ) {
        let s = &self.0.state;
        let previous = s.gate.fetch_add(1, Ordering::SeqCst);
        let _admission = Admission(&s.gate);
        if previous & CLOSED != 0 {
            s.dropped.fetch_add(1, Ordering::Relaxed);
            return;
        }
        let record = LogRecord {
            id: 0,
            ts_ms: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis()
                .min(i64::MAX as u128) as i64,
            level: capped(level, 24),
            source: capped(source, 96),
            code: capped(code, 96),
            message: capped(message, 2048),
            request_id: request_id.map(|s| capped(s, 128)),
        };
        let Ok(mut queue) = s.queue.try_lock() else {
            s.dropped.fetch_add(1, Ordering::Relaxed);
            return;
        };
        if queue.len() >= CAPACITY {
            s.dropped.fetch_add(1, Ordering::Relaxed);
            return;
        }
        // Account before publication, so a fast consumer cannot underflow pending.
        // The queue retains its preallocated capacity; only the strings above allocate.
        s.pending.fetch_add(1, Ordering::Relaxed);
        queue.push_back(record);
    }

    /// Tries admission once; clones only after acquiring the lock and finding space.
    /// Curves share their Arc allocation; no point copies, serialization, SQL, wait or wakeup.
    /// Rejected calls leave the report with its caller.
    /// False means close, contention, poisoning or a full preallocated report queue.
    /// Report admission and persistence are accounted separately from event stats.
    pub fn record_calibration(&self, report: &Report) -> bool {
        let s = &self.0.state;
        let previous = s.gate.fetch_add(1, Ordering::SeqCst);
        let _admission = Admission(&s.gate);
        if previous & CLOSED != 0 {
            s.calibration_dropped.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        let Ok(mut queue) = s.calibrations.try_lock() else {
            s.calibration_dropped.fetch_add(1, Ordering::Relaxed);
            return false;
        };
        if queue.len() >= CALIBRATION_CAPACITY {
            s.calibration_dropped.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        let owned = report.clone();
        s.calibration_pending.fetch_add(1, Ordering::Relaxed);
        queue.push_back(owned);
        true
    }

    /// One try-lock and a cheap shared capture clone; no flattening, SQL or wakeup.
    /// Rejected captures remain with the controller for retry. Teaching accounting
    /// is independent of event counts and uses the same atomic close/admission gate.
    pub fn record_teaching(&self, capture: &Capture) -> bool {
        let s = &self.0.state;
        let previous = s.gate.fetch_add(1, Ordering::SeqCst);
        let _admission = Admission(&s.gate);
        if previous & CLOSED != 0 {
            s.teaching_dropped.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        let Ok(mut queue) = s.teachings.try_lock() else {
            s.teaching_dropped.fetch_add(1, Ordering::Relaxed);
            return false;
        };
        if queue.queue.len() >= TEACHING_CAPACITY {
            s.teaching_dropped.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        let owned = Arc::new(capture.clone());
        s.teaching_pending.fetch_add(1, Ordering::Relaxed);
        queue.uncommitted.push(owned.clone());
        queue.queue.push_back(owned);
        true
    }

    #[cfg(test)]
    pub(crate) fn with_teaching_queue_locked<T>(&self, f: impl FnOnce() -> T) -> T {
        let _queue = self
            .0
            .state
            .teachings
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        f()
    }

    /// Blocking summary-only read. User recordings have no automatic retention.
    /// A database predating teaching support has an empty teaching history.
    pub fn teachings(&self) -> Result<Vec<TeachingSummary>, String> {
        // Snapshot cache ownership before opening SQLite: a commit and cache
        // removal between these reads must not make an admitted capture vanish.
        let captures = self
            .0
            .state
            .teachings
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .uncommitted
            .clone();
        let mut summaries: Vec<TeachingSummary> = Vec::new();
        for capture in captures.iter().rev() {
            if !summaries
                .iter()
                .any(|summary| summary.id == capture.summary.id)
            {
                summaries.push(capture.summary.clone());
            }
        }
        let stored = self.stored_teachings()?;
        let cached_count = summaries.len();
        for summary in stored {
            if !summaries[..cached_count]
                .iter()
                .any(|cached| cached.id == summary.id)
            {
                summaries.push(summary);
            }
        }
        summaries.sort_by(|a, b| b.created_at_ms.cmp(&a.created_at_ms).then(b.id.cmp(&a.id)));
        Ok(summaries)
    }

    fn stored_teachings(&self) -> Result<Vec<TeachingSummary>, String> {
        let Some(conn) = self.teaching_reader()? else {
            return Ok(Vec::new());
        };
        if !teaching_table_exists(&conn)? {
            return Ok(Vec::new());
        }
        let mut stmt = conn
            .prepare("SELECT payloadJSON FROM teachings ORDER BY created_at_ms DESC,id DESC")
            .map_err(|e| e.to_string())?;
        let rows = stmt
            .query_map([], |row| row.get::<_, String>(0))
            .map_err(|e| e.to_string())?;
        rows.map(|row| {
            let payload = row.map_err(|e| e.to_string())?;
            serde_json::from_str(&payload).map_err(|e| format!("read teaching summary: {e}"))
        })
        .collect()
    }

    /// Blocking read. A transaction keeps the summary and points on one commit.
    /// HTTP readers flatten any uncommitted in-memory capture off-thread instead.
    pub fn teaching(&self, id: &str) -> Result<Option<Trajectory>, String> {
        let capture = self
            .0
            .state
            .teachings
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .uncommitted
            .iter()
            .rev()
            .find(|capture| capture.summary.id == id)
            .cloned();
        if let Some(capture) = capture {
            return Ok(Some(capture.trajectory()));
        }
        let Some(mut conn) = self.teaching_reader()? else {
            return Ok(None);
        };
        let tx = conn.transaction().map_err(|e| e.to_string())?;
        if !teaching_table_exists(&tx)? {
            return Ok(None);
        }
        let payload: Option<String> = tx
            .query_row(
                "SELECT payloadJSON FROM teachings WHERE id = ?1",
                [id],
                |row| row.get(0),
            )
            .optional()
            .map_err(|e| e.to_string())?;
        let Some(payload) = payload else {
            return Ok(None);
        };
        let points: String = tx
            .query_row(
                "SELECT payloadJSON FROM teaching_points WHERE id = ?1",
                [id],
                |row| row.get(0),
            )
            .map_err(|e| e.to_string())?;
        tx.commit().map_err(|e| e.to_string())?;
        Ok(Some(Trajectory {
            summary: serde_json::from_str(&payload)
                .map_err(|e| format!("read teaching summary: {e}"))?,
            points: serde_json::from_str(&points)
                .map_err(|e| format!("read teaching points: {e}"))?,
        }))
    }

    fn teaching_reader(&self) -> Result<Option<Connection>, String> {
        // Startup creates the database asynchronously; an in-memory capture can
        // already be available while the writer is still opening a new journal.
        if !self.0.path.try_exists().map_err(|e| e.to_string())? {
            return Ok(None);
        }
        let conn = Connection::open_with_flags(
            &self.0.path,
            OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_NO_MUTEX,
        )
        .map_err(|e| format!("open teaching reader: {e}"))?;
        conn.busy_timeout(BUSY).map_err(|e| e.to_string())?;
        Ok(Some(conn))
    }

    /// Holds the report queue mutex for deterministic admission tests.
    #[cfg(test)]
    pub(crate) fn with_calibration_queue_locked<T>(&self, f: impl FnOnce() -> T) -> T {
        let _queue = self
            .0
            .state
            .calibrations
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        f()
    }

    /// Blocking read, including open and SQLite busy waits; use spawn_blocking.
    /// Callers supply the usual default of 20; at most 100 reports are returned.
    pub fn calibrations(&self, limit: usize) -> Result<Vec<Report>, String> {
        let limit = limit.min(100);
        if limit == 0 {
            return Ok(Vec::new());
        }
        let conn = self.calibration_reader()?;
        let mut stmt = conn
            .prepare(
                "SELECT payloadJSON FROM calibrations ORDER BY created_at_ms DESC,id DESC LIMIT ?1",
            )
            .map_err(|e| e.to_string())?;
        let rows = stmt
            .query_map([limit as i64], |row| row.get::<_, String>(0))
            .map_err(|e| e.to_string())?;
        rows.map(|row| {
            let payload = row.map_err(|e| e.to_string())?;
            read_calibration(&payload).map(|report| report.summary())
        })
        .collect()
    }

    /// Blocking read; HTTP callers must use spawn_blocking.
    pub fn calibration(&self, id: &str) -> Result<Option<Report>, String> {
        let mut conn = self.calibration_reader()?;
        // Retention and upserts cannot mix metadata and curves from different commits.
        let tx = conn.transaction().map_err(|e| e.to_string())?;
        let payload: Option<String> = tx
            .query_row(
                "SELECT payloadJSON FROM calibrations WHERE id = ?1",
                [id],
                |row| row.get(0),
            )
            .optional()
            .map_err(|e| e.to_string())?;
        // A read-only legacy database may not have been reopened by the new writer yet.
        let curves: Option<String> = if payload.is_some()
            && tx
                .query_row(
                    "SELECT EXISTS(SELECT 1 FROM sqlite_master
                     WHERE type='table' AND name='calibration_curves')",
                    [],
                    |row| row.get::<_, bool>(0),
                )
                .map_err(|e| e.to_string())?
        {
            tx.query_row(
                "SELECT payloadJSON FROM calibration_curves WHERE id = ?1",
                [id],
                |row| row.get(0),
            )
            .optional()
            .map_err(|e| e.to_string())?
        } else {
            None
        };
        tx.commit().map_err(|e| e.to_string())?;
        payload
            .map(|payload| {
                let mut report = read_calibration(&payload)?;
                if let Some(curves) = curves {
                    report.curves = serde_json::from_str(&curves)
                        .map_err(|e| format!("read calibration curves: {e}"))?;
                }
                Ok(report)
            })
            .transpose()
    }

    fn calibration_reader(&self) -> Result<Connection, String> {
        let conn = Connection::open_with_flags(
            &self.0.path,
            OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_NO_MUTEX,
        )
        .map_err(|e| format!("open calibration reader: {e}"))?;
        conn.busy_timeout(BUSY).map_err(|e| e.to_string())?;
        Ok(conn)
    }

    /// Independent atomic samples, not a transactionally consistent snapshot.
    pub fn stats(&self) -> JournalStats {
        let s = &self.0.state;
        JournalStats {
            pending: s.pending.load(Ordering::Relaxed),
            written: s.written.load(Ordering::Relaxed),
            dropped: s.dropped.load(Ordering::Relaxed),
            errors: s.errors.load(Ordering::Relaxed),
            healthy: s.healthy.load(Ordering::Acquire)
                && s.gate.load(Ordering::SeqCst) & CLOSED == 0,
        }
    }

    /// Observation only. HTTP supplies the 10000ms default; explicit durations
    /// must be positive and fit the recorder's nanosecond clock.
    pub fn trace_start(
        &self,
        duration_ms: u64,
        side: &str,
        port: Option<&str>,
    ) -> Result<TraceSummary, String> {
        self.0.traces.trace_start(duration_ms, side, port)
    }

    pub fn trace_stop(&self) -> Result<Option<TraceSummary>, String> {
        self.0.traces.trace_stop()
    }

    /// Blocking read: HTTP callers must use spawn_blocking.
    pub fn trace_list(&self, limit: u32) -> Result<Vec<TraceSummary>, String> {
        self.0.traces.trace_list(limit)
    }

    /// Blocking read: HTTP callers must use spawn_blocking.
    pub fn trace_read(&self, id: &str) -> Result<TraceRecording, String> {
        self.0.traces.trace_read(id)
    }

    /// Fixed-size observation submission. Never waits for a lock, disk or queue space.
    pub fn sample(&self, at: Instant, data: TraceData) {
        self.0.traces.sample(at, data);
    }

    /// Blocking read, including open and SQLite busy waits; use spawn_blocking.
    pub fn recent(&self, limit: u32) -> Result<Vec<LogRecord>, String> {
        let limit = limit.min(500);
        if limit == 0 {
            return Ok(Vec::new());
        }
        let conn = Connection::open_with_flags(
            &self.0.path,
            OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_NO_MUTEX,
        )
        .map_err(|e| format!("open journal reader: {e}"))?;
        conn.busy_timeout(BUSY).map_err(|e| e.to_string())?;
        let mut stmt = conn.prepare(
            "SELECT id,ts_ms,level,source,code,message,request_id FROM events ORDER BY id DESC LIMIT ?1"
        ).map_err(|e| e.to_string())?;
        let rows = stmt
            .query_map([limit], log_record_from_row)
            .map_err(|e| e.to_string())?;
        rows.collect::<rusqlite::Result<Vec<_>>>()
            .map_err(|e| e.to_string())
    }

    /// Blocking read; HTTP callers must use spawn_blocking. Reuse through_id to
    /// keep subsequent pages fixed against new inserts (the retention cap still applies).
    pub fn page(&self, query: LogPageQuery) -> Result<LogPage, String> {
        if query.through_id.is_some_and(|id| id < 0)
            || query.search.len() > 512
            || matches!((query.from_ms, query.to_ms), (Some(from), Some(to)) if from > to)
        {
            return Err("invalid_log_query".into());
        }
        let page_size = query.page_size.clamp(1, 500);
        let search = query.search.to_ascii_lowercase();
        let level = query.level.as_deref().filter(|value| !value.is_empty());
        let source = query.source.as_deref().filter(|value| !value.is_empty());
        let code = query.code.as_deref().filter(|value| !value.is_empty());
        let mut conn = Connection::open_with_flags(
            &self.0.path,
            OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_NO_MUTEX,
        )
        .map_err(|e| format!("open journal reader: {e}"))?;
        conn.busy_timeout(BUSY).map_err(|e| e.to_string())?;
        conn.create_scalar_function(
            "legacy_motor_fault_display",
            1,
            FunctionFlags::SQLITE_UTF8 | FunctionFlags::SQLITE_DETERMINISTIC,
            |ctx| {
                let mut message = ctx.get(0)?;
                describe_legacy_motor_fault(&mut message);
                Ok(message)
            },
        )
        .map_err(|e| e.to_string())?;
        let tx = conn.transaction().map_err(|e| e.to_string())?;
        // The first SELECT establishes the same SQLite snapshot for MAX/COUNT/items.
        let max_id: i64 = tx
            .query_row("SELECT COALESCE(MAX(id),0) FROM events", [], |r| r.get(0))
            .map_err(|e| e.to_string())?;
        let through_id = query.through_id.unwrap_or(max_id).min(max_id);
        // instr performs literal substring matching, including %, _ and backslashes.
        let filter = "?1 > 0 AND id <= ?1 AND (?2 = ''
            OR instr(lower(level),?2) > 0 OR instr(lower(source),?2) > 0
            OR instr(lower(code),?2) > 0 OR instr(lower(message),?2) > 0
            OR instr(lower(request_id),?2) > 0
            OR (message GLOB 'J? 故障码 *' AND instr(lower(legacy_motor_fault_display(message)),?2) > 0))
            AND (?3 IS NULL OR level = ?3)
            AND (?4 IS NULL OR source = ?4)
            AND (?5 IS NULL OR code = ?5)
            AND (?6 IS NULL OR ts_ms >= ?6)
            AND (?7 IS NULL OR ts_ms <= ?7)";
        let total: u64 = tx
            .query_row(
                &format!("SELECT COUNT(*) FROM events WHERE {filter}"),
                params![
                    through_id,
                    search,
                    level,
                    source,
                    code,
                    query.from_ms,
                    query.to_ms
                ],
                |r| r.get::<_, i64>(0),
            )
            .map_err(|e| e.to_string())? as u64;
        let total_pages = total
            .div_ceil(u64::from(page_size))
            .clamp(1, u64::from(u32::MAX)) as u32;
        let page = query.page.max(1).min(total_pages);
        let offset = i64::from(page - 1) * i64::from(page_size);
        let items = {
            let mut stmt = tx
                .prepare(&format!(
                    "SELECT id,ts_ms,level,source,code,message,request_id FROM events
                     WHERE {filter} ORDER BY id DESC LIMIT ?8 OFFSET ?9"
                ))
                .map_err(|e| e.to_string())?;
            stmt.query_map(
                params![
                    through_id,
                    search,
                    level,
                    source,
                    code,
                    query.from_ms,
                    query.to_ms,
                    page_size,
                    offset
                ],
                log_record_from_row,
            )
            .map_err(|e| e.to_string())?
            .collect::<rusqlite::Result<Vec<_>>>()
            .map_err(|e| e.to_string())?
        };
        tx.commit().map_err(|e| e.to_string())?;
        Ok(LogPage {
            items,
            page,
            page_size,
            total,
            total_pages,
            through_id,
        })
    }

    /// Blocking whole-history choices. A read transaction keeps the three lists
    /// on one persisted snapshot; this never waits on the producer queue.
    pub fn log_filters(&self) -> Result<LogFilters, String> {
        let mut conn = Connection::open_with_flags(
            &self.0.path,
            OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_NO_MUTEX,
        )
        .map_err(|e| format!("open journal reader: {e}"))?;
        conn.busy_timeout(BUSY).map_err(|e| e.to_string())?;
        let tx = conn.transaction().map_err(|e| e.to_string())?;
        let distinct = |column: &str| -> Result<Vec<String>, String> {
            let mut stmt = tx
                .prepare(&format!(
                    "SELECT DISTINCT {column} FROM events ORDER BY {column}"
                ))
                .map_err(|e| e.to_string())?;
            stmt.query_map([], |row| row.get(0))
                .map_err(|e| e.to_string())?
                .collect::<rusqlite::Result<Vec<_>>>()
                .map_err(|e| e.to_string())
        };
        let choices = LogFilters {
            levels: distinct("level")?,
            sources: distinct("source")?,
            codes: distinct("code")?,
        };
        tx.commit().map_err(|e| e.to_string())?;
        Ok(choices)
    }

    /// Application-close coordination only. True means every accepted event/report/capture/sample
    /// was committed and both workers exited. False means a timeout or abandoned data.
    /// A blocked OS/SQLite call can outlive this return; no forced cancellation or join.
    pub fn shutdown(&self, timeout: Duration) -> bool {
        let s = &self.0.state;
        self.0.traces.request_shutdown();
        s.gate.fetch_or(CLOSED, Ordering::SeqCst);
        let start = Instant::now();
        while !s.done.load(Ordering::Acquire) || !self.0.traces.done() {
            let remaining = timeout.saturating_sub(start.elapsed());
            if remaining.is_zero() {
                s.abort.store(true, Ordering::Release);
                self.0.traces.abort();
                return false;
            }
            thread::sleep(remaining.min(Duration::from_millis(1)));
        }
        !s.abort.load(Ordering::Acquire) && self.0.traces.clean()
    }
}

fn log_record_from_row(row: &rusqlite::Row<'_>) -> rusqlite::Result<LogRecord> {
    let mut message = row.get(5)?;
    describe_legacy_motor_fault(&mut message);
    Ok(LogRecord {
        id: row.get(0)?,
        ts_ms: row.get(1)?,
        level: row.get(2)?,
        source: row.get(3)?,
        code: row.get(4)?,
        message,
        request_id: row.get(6)?,
    })
}

fn read_calibration(payload: &str) -> Result<Report, String> {
    let mut report: Report =
        serde_json::from_str(payload).map_err(|e| format!("read calibration: {e}"))?;
    describe_legacy_motor_fault(&mut report.message);
    Ok(report)
}

fn describe_legacy_motor_fault(message: &mut String) {
    // The old writer emitted J1..J7 and a nonzero u32 in canonical decimal.
    let Some((joint, raw_code)) = message.split_once(" 故障码 ") else {
        return;
    };
    if matches!(joint.as_bytes(), [b'J', b'1'..=b'7'])
        && matches!(raw_code.as_bytes(), [b'1'..=b'9', ..])
        && let Ok(code) = raw_code.parse::<u32>()
    {
        *message = format!(
            "{joint} {}（故障码 {raw_code}）",
            motor_fault_description(code)
        );
    }
}

fn capped(value: &str, max: usize) -> String {
    let mut end = value.len().min(max);
    while !value.is_char_boundary(end) {
        end -= 1;
    }
    value[..end].to_owned()
}

fn teaching_table_exists(conn: &Connection) -> Result<bool, String> {
    conn.query_row(
        "SELECT EXISTS(SELECT 1 FROM sqlite_master WHERE type='table' AND name='teachings')",
        [],
        |row| row.get(0),
    )
    .map_err(|e| e.to_string())
}

fn open_writer(path: &Path) -> rusqlite::Result<Connection> {
    // Directory setup is intentionally also on the writer, never on start/log.
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
         CREATE TABLE IF NOT EXISTS events (
             id INTEGER PRIMARY KEY AUTOINCREMENT,
             ts_ms INTEGER NOT NULL,
             level TEXT NOT NULL,
             source TEXT NOT NULL,
             code TEXT NOT NULL,
             message TEXT NOT NULL,
             request_id TEXT
         );
         CREATE TABLE IF NOT EXISTS calibrations (
             id TEXT PRIMARY KEY,
             created_at_ms INTEGER NOT NULL,
             payloadJSON TEXT NOT NULL
         );
         CREATE TABLE IF NOT EXISTS calibration_curves (
             id TEXT PRIMARY KEY,
             payloadJSON TEXT NOT NULL
         );
         CREATE TABLE IF NOT EXISTS teachings (
             id TEXT PRIMARY KEY,
             created_at_ms INTEGER NOT NULL,
             payloadJSON TEXT NOT NULL
         );
         CREATE TABLE IF NOT EXISTS teaching_points (
             id TEXT PRIMARY KEY,
             payloadJSON TEXT NOT NULL
         );",
    )?;
    // Verify WAL actually took effect (e.g. reject :memory:, which cannot persist).
    let mode: String = conn.query_row("PRAGMA journal_mode", [], |r| r.get(0))?;
    if mode != "wal" {
        return Err(rusqlite::Error::InvalidQuery);
    }
    Ok(conn)
}

fn write_batch(conn: &mut Connection, batch: &[LogRecord]) -> rusqlite::Result<()> {
    let tx = conn.transaction()?;
    {
        let mut insert = tx.prepare_cached(
            "INSERT INTO events(ts_ms,level,source,code,message,request_id) VALUES(?1,?2,?3,?4,?5,?6)"
        )?;
        for r in batch {
            insert.execute(params![
                r.ts_ms,
                r.level,
                r.source,
                r.code,
                r.message,
                r.request_id
            ])?;
        }
    }
    tx.execute(
        "DELETE FROM events WHERE id < (SELECT id FROM events ORDER BY id DESC LIMIT 1 OFFSET 99999)", []
    )?;
    tx.commit()
}

fn failed(s: &State) {
    s.errors.fetch_add(1, Ordering::Relaxed);
    s.healthy.store(false, Ordering::Release);
}

fn write_calibration(conn: &mut Connection, report: &Report) -> Result<(), String> {
    // The writer owns this report; all producer and sample locks have been released.
    // Serialize before opening the independent short report transaction.
    let report = report.for_plot(0);
    let payload = serde_json::to_string(&report.summary()).map_err(|e| e.to_string())?;
    let curves = serde_json::to_string(&report.curves).map_err(|e| e.to_string())?;
    let tx = conn.transaction().map_err(|e| e.to_string())?;
    tx.execute(
        "INSERT INTO calibrations(id,created_at_ms,payloadJSON) VALUES(?1,?2,?3)
         ON CONFLICT(id) DO UPDATE SET
             created_at_ms=excluded.created_at_ms,payloadJSON=excluded.payloadJSON",
        params![report.id, report.created_at_ms, payload],
    )
    .map_err(|e| e.to_string())?;
    tx.execute(
        "INSERT INTO calibration_curves(id,payloadJSON) VALUES(?1,?2)
         ON CONFLICT(id) DO UPDATE SET payloadJSON=excluded.payloadJSON",
        params![report.id, curves],
    )
    .map_err(|e| e.to_string())?;
    tx.execute(
        "DELETE FROM calibrations WHERE id IN (
             SELECT id FROM calibrations ORDER BY created_at_ms DESC,id DESC LIMIT -1 OFFSET 100
         )",
        [],
    )
    .map_err(|e| e.to_string())?;
    tx.execute(
        "DELETE FROM calibration_curves WHERE NOT EXISTS (
             SELECT 1 FROM calibrations WHERE calibrations.id=calibration_curves.id
         )",
        [],
    )
    .map_err(|e| e.to_string())?;
    tx.commit().map_err(|e| e.to_string())
}

fn write_teaching(
    conn: &mut Connection,
    capture: &Capture,
    serialized: &mut Option<(String, String)>,
) -> Result<(), String> {
    // This runs only on the writer, after releasing the admission mutex. Keep
    // encoded payloads across SQLite busy retries instead of flattening again.
    if serialized.is_none() {
        let trajectory = capture.trajectory();
        *serialized = Some((
            serde_json::to_string(&trajectory.summary).map_err(|e| e.to_string())?,
            serde_json::to_string(&trajectory.points).map_err(|e| e.to_string())?,
        ));
    }
    let (summary, points) = serialized.as_ref().unwrap();
    let tx = conn.transaction().map_err(|e| e.to_string())?;
    tx.execute(
        "INSERT INTO teachings(id,created_at_ms,payloadJSON) VALUES(?1,?2,?3)
         ON CONFLICT(id) DO UPDATE SET
             created_at_ms=excluded.created_at_ms,payloadJSON=excluded.payloadJSON",
        params![capture.summary.id, capture.summary.created_at_ms, summary],
    )
    .map_err(|e| e.to_string())?;
    tx.execute(
        "INSERT INTO teaching_points(id,payloadJSON) VALUES(?1,?2)
         ON CONFLICT(id) DO UPDATE SET payloadJSON=excluded.payloadJSON",
        params![capture.summary.id, points],
    )
    .map_err(|e| e.to_string())?;
    tx.commit().map_err(|e| e.to_string())
}

fn backoff(s: &State, delay: Duration) {
    let start = Instant::now();
    while !s.abort.load(Ordering::Acquire) && start.elapsed() < delay {
        thread::sleep(POLL.min(delay.saturating_sub(start.elapsed())));
    }
}

fn writer(path: &Path, state: Arc<State>) {
    let s = &*state;
    let mut conn = None;
    let mut batch = Vec::with_capacity(BATCH_SIZE);
    let mut calibration = None;
    let mut teaching = None;
    let mut teaching_serialized = None;
    let mut retired: Vec<Arc<Capture>> = Vec::new();
    let mut delay = Duration::from_millis(20);
    let mut checkpoint_at = Instant::now();
    loop {
        #[cfg(test)]
        let retirement_paused = s.pause_retirement.load(Ordering::Acquire);
        #[cfg(not(test))]
        let retirement_paused = false;
        if !retirement_paused {
            // Core owns a distinct Capture sharing these chunks. Keep our copy
            // until its last control/watch copy has gone, so chunk destruction
            // happens here rather than during a control tick.
            retired.retain(|capture| !capture.can_retire());
        }
        if s.abort.load(Ordering::Acquire) {
            let mut count = batch.len() as u64;
            batch.clear();
            {
                let mut queue = s.queue.lock().unwrap_or_else(|p| p.into_inner());
                count += queue.len() as u64;
                queue.clear();
            }
            s.dropped.fetch_add(count, Ordering::Relaxed);
            s.pending.fetch_sub(count, Ordering::Relaxed);
            let mut count = u64::from(calibration.take().is_some());
            {
                let mut queue = s.calibrations.lock().unwrap_or_else(|p| p.into_inner());
                count += queue.len() as u64;
                queue.clear();
            }
            s.calibration_dropped.fetch_add(count, Ordering::Relaxed);
            s.calibration_pending.fetch_sub(count, Ordering::Relaxed);
            let mut count = u64::from(teaching.take().is_some());
            teaching_serialized = None;
            {
                let mut queue = s.teachings.lock().unwrap_or_else(|p| p.into_inner());
                count += queue.queue.len() as u64;
                queue.queue.clear();
            }
            s.teaching_dropped.fetch_add(count, Ordering::Relaxed);
            s.teaching_pending.fetch_sub(count, Ordering::Relaxed);
            // Producers admitted just before close can still publish a final record.
            if s.gate.load(Ordering::SeqCst) == CLOSED
                && s.pending.load(Ordering::Relaxed) == 0
                && s.calibration_pending.load(Ordering::Relaxed) == 0
                && s.teaching_pending.load(Ordering::Relaxed) == 0
            {
                break;
            }
            thread::sleep(POLL);
            continue;
        }
        if conn.is_none() {
            match open_writer(path) {
                Ok(c) => {
                    conn = Some(c);
                    s.healthy.store(true, Ordering::Release);
                }
                Err(_) => {
                    failed(s);
                    backoff(s, delay);
                    delay = (delay * 2).min(Duration::from_secs(1));
                    continue;
                }
            }
        }
        {
            // Only the writer waits for this lock. No SQL, allocation, or sleeping
            // occurs while draining into the preallocated batch.
            let mut queue = s.queue.lock().unwrap_or_else(|p| p.into_inner());
            while batch.len() < BATCH_SIZE {
                let Some(record) = queue.pop_front() else {
                    break;
                };
                batch.push(record);
            }
        }
        if calibration.is_none() {
            calibration = s
                .calibrations
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .pop_front();
        }
        if teaching.is_none() {
            teaching = s
                .teachings
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .queue
                .pop_front();
        }
        let idle = batch.is_empty() && calibration.is_none() && teaching.is_none();
        if !batch.is_empty() {
            match write_batch(conn.as_mut().unwrap(), &batch) {
                Ok(()) => {
                    let count = batch.len() as u64;
                    s.written.fetch_add(count, Ordering::Relaxed);
                    s.pending.fetch_sub(count, Ordering::Relaxed);
                    batch.clear();
                    s.healthy.store(true, Ordering::Release);
                    delay = Duration::from_millis(20);
                }
                Err(_) => {
                    failed(s);
                    conn = None;
                    backoff(s, delay);
                    delay = (delay * 2).min(Duration::from_secs(1));
                    continue;
                }
            }
        }
        // Bound report work to one transaction per event batch so neither queue
        // can monopolize a healthy writer. Retain the report across busy retries.
        if let Some(report) = &calibration {
            match write_calibration(conn.as_mut().unwrap(), report) {
                Ok(()) => {
                    s.calibration_pending.fetch_sub(1, Ordering::Relaxed);
                    calibration = None;
                    delay = Duration::from_millis(20);
                }
                Err(_) => {
                    s.calibration_errors.fetch_add(1, Ordering::Relaxed);
                    backoff(s, delay);
                    delay = (delay * 2).min(Duration::from_secs(1));
                    continue;
                }
            }
        }
        if let Some(capture) = &teaching {
            match write_teaching(conn.as_mut().unwrap(), capture, &mut teaching_serialized) {
                Ok(()) => {
                    s.teachings
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .uncommitted
                        .retain(|cached| !Arc::ptr_eq(cached, capture));
                    let completed = teaching.take().unwrap();
                    // Repeated admissions share a tail. Keep one worker owner
                    // so duplicate retired captures cannot pin one another.
                    if !retired
                        .iter()
                        .any(|previous| previous.shares_data(&completed))
                    {
                        retired.push(completed);
                    }
                    s.teaching_pending.fetch_sub(1, Ordering::Relaxed);
                    teaching_serialized = None;
                    delay = Duration::from_millis(20);
                }
                Err(_) => {
                    s.teaching_errors.fetch_add(1, Ordering::Relaxed);
                    backoff(s, delay);
                    delay = (delay * 2).min(Duration::from_secs(1));
                    continue;
                }
            }
        }
        if idle {
            // Polling avoids producer-side notification and its hidden wakeup locks.
            thread::sleep(POLL);
        }
        let closing = s.gate.load(Ordering::SeqCst) == CLOSED
            && s.pending.load(Ordering::Relaxed) == 0
            && s.calibration_pending.load(Ordering::Relaxed) == 0
            && s.teaching_pending.load(Ordering::Relaxed) == 0;
        if closing || checkpoint_at.elapsed() >= Duration::from_secs(5) {
            // PASSIVE never waits for readers; outstanding WAL pages remain safe to reopen.
            if conn
                .as_ref()
                .unwrap()
                .execute_batch("PRAGMA wal_checkpoint(PASSIVE)")
                .is_err()
            {
                failed(s);
                if !closing {
                    conn = None;
                }
            }
            checkpoint_at = Instant::now();
        }
        if closing {
            break;
        }
    }
    drop(conn);
    s.healthy.store(false, Ordering::Release);
    s.done.store(true, Ordering::Release);
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;

    fn wait_for(mut condition: impl FnMut() -> bool) {
        let start = Instant::now();
        while !condition() {
            assert!(
                start.elapsed() < Duration::from_secs(8),
                "background operation timed out"
            );
            thread::sleep(Duration::from_millis(5));
        }
    }

    // Single-producer fixtures retry only rejected calls to keep their expected
    // payloads deterministic. Return every attempt so drops remain fully accounted.
    fn log_until_queued(journal: &Journal, mut emit: impl FnMut()) -> u64 {
        let mut attempts = 0;
        wait_for(|| {
            let dropped = journal.stats().dropped;
            emit();
            attempts += 1;
            journal.stats().dropped == dropped
        });
        attempts
    }

    fn teaching_capture(id: &str, created_at_ms: i64, samples: usize) -> Capture {
        let mut recorder = crate::teaching::Recorder::new(TeachingSummary {
            id: id.into(),
            name: "示教 ' 与换行\n完整保存".into(),
            created_at_ms,
            side: "right".into(),
            tcp: crate::kinematics::Pose::identity(),
            duration_ms: 0,
            sample_count: 0,
            outcome: String::new(),
        });
        for i in 0..samples {
            recorder.push(crate::teaching::Point {
                t_ms: i as u64 * 10,
                q: [i as f64 / 1024.0; 7],
            });
        }
        recorder.finish("completed")
    }

    fn assert_teaching_matches(actual: &Trajectory, expected: &Capture) {
        assert_eq!(
            serde_json::to_value(actual).unwrap(),
            serde_json::to_value(expected.trajectory()).unwrap()
        );
    }

    #[test]
    fn teaching_flush_reopen_keeps_every_point_and_separates_event_accounting() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("nested/events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        let mut capture = teaching_capture("quote' OR 1=1 --", 1234, 8193);
        let expected = capture.clone();
        wait_for(|| journal.record_teaching(&capture));
        capture.summary.name = "caller changed after admission".into();
        assert!(journal.shutdown(Duration::from_secs(8)));
        assert_teaching_matches(
            &journal.teaching(&expected.summary.id).unwrap().unwrap(),
            &expected,
        );
        assert!(journal.teaching("unknown' OR 1=1 --").unwrap().is_none());
        let stats = journal.stats();
        assert_eq!(
            (stats.written, stats.pending, stats.dropped, stats.errors),
            (0, 0, 0, 0)
        );
        drop(journal);

        let reopened = Journal::start(path.clone()).unwrap();
        wait_for(|| reopened.stats().healthy);
        assert_teaching_matches(
            &reopened.teaching(&expected.summary.id).unwrap().unwrap(),
            &expected,
        );
        let history = reopened.teachings().unwrap();
        assert_eq!(
            serde_json::to_value(&history).unwrap(),
            serde_json::json!([expected.summary])
        );
        let conn = Connection::open(path).unwrap();
        let (summary, points): (String, String) = conn.query_row(
            "SELECT t.payloadJSON,p.payloadJSON FROM teachings t JOIN teaching_points p ON p.id=t.id",
            [], |row| Ok((row.get(0)?, row.get(1)?)),
        ).unwrap();
        assert!(!summary.contains("points"));
        assert!(summary.len() * 50 < points.len());
        assert!(reopened.shutdown(Duration::from_secs(2)));
    }

    #[test]
    fn teaching_history_has_no_retention_cap_and_upserts_literal_ids() {
        let dir = tempdir().unwrap();
        let journal = Journal::start(dir.path().join("events.sqlite3")).unwrap();
        for i in (0..106).rev() {
            let capture = teaching_capture(&format!("teaching-{i:03}"), 1000 + i / 2, 3);
            wait_for(|| journal.record_teaching(&capture));
        }
        let mut changed = teaching_capture("teaching-002", 2000, 7);
        changed.summary.name = "updated".into();
        wait_for(|| journal.record_teaching(&changed));
        assert!(journal.shutdown(Duration::from_secs(8)));
        let history = journal.teachings().unwrap();
        assert_eq!(history.len(), 106);
        assert_eq!(history[0].id, "teaching-002");
        assert_eq!(history[1].id, "teaching-105");
        assert_eq!(history[2].id, "teaching-104");
        assert_eq!(history.last().unwrap().id, "teaching-000");
        assert_teaching_matches(
            &journal.teaching("teaching-002").unwrap().unwrap(),
            &changed,
        );
        let conn = Connection::open(&journal.0.path).unwrap();
        let count: i64 = conn
            .query_row("SELECT COUNT(*) FROM teaching_points", [], |row| row.get(0))
            .unwrap();
        assert_eq!(count, 106);
    }

    #[test]
    fn teaching_contention_and_sqlite_busy_retry_preserve_admitted_captures() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        wait_for(|| journal.stats().healthy);
        let mut capture = teaching_capture("busy-shared", 1, 6001);
        let expected = capture.clone();
        thread::scope(|scope| {
            let (tx, rx) = std::sync::mpsc::channel();
            let (producer, result) = journal.with_teaching_queue_locked(|| {
                let borrowed = &capture;
                let worker = &journal;
                let producer =
                    scope.spawn(move || tx.send(worker.record_teaching(borrowed)).unwrap());
                (producer, rx.recv_timeout(Duration::from_secs(1)))
            });
            producer.join().unwrap();
            assert!(!result.expect("teaching admission waited for the queue mutex"));
        });
        let locker = Connection::open(path).unwrap();
        locker.execute_batch("BEGIN IMMEDIATE").unwrap();
        let mut admission_attempts = 0;
        wait_for(|| {
            admission_attempts += 1;
            journal.record_teaching(&capture)
        });
        capture.summary.name = "caller changed after admission".into();
        let start = Instant::now();
        let accepted: Vec<_> = (0..100)
            .filter_map(|i| {
                let capture = teaching_capture(&format!("busy-{i}"), 100 + i, 3);
                journal.record_teaching(&capture).then_some(capture)
            })
            .collect();
        assert!(start.elapsed() < Duration::from_secs(1));
        assert!(accepted.len() <= TEACHING_CAPACITY);
        wait_for(|| journal.0.state.teaching_errors.load(Ordering::Relaxed) > 0);
        assert_eq!(
            journal.0.state.teaching_pending.load(Ordering::Relaxed),
            accepted.len() as u64 + 1
        );
        assert_eq!(journal.teachings().unwrap().len(), accepted.len() + 1);
        assert_teaching_matches(
            &journal.teaching(&capture.summary.id).unwrap().unwrap(),
            &expected,
        );
        for capture in &accepted {
            assert_teaching_matches(
                &journal.teaching(&capture.summary.id).unwrap().unwrap(),
                capture,
            );
        }
        assert!(journal.stored_teachings().unwrap().is_empty());
        let stats = journal.stats();
        assert_eq!(
            (stats.written, stats.pending, stats.dropped, stats.errors),
            (0, 0, 0, 0)
        );
        locker.execute_batch("ROLLBACK").unwrap();
        assert!(journal.shutdown(Duration::from_secs(8)));
        assert_teaching_matches(
            &journal.teaching(&capture.summary.id).unwrap().unwrap(),
            &expected,
        );
        for capture in &accepted {
            assert_teaching_matches(
                &journal.teaching(&capture.summary.id).unwrap().unwrap(),
                capture,
            );
        }
        assert_eq!(journal.teachings().unwrap().len(), accepted.len() + 1);
        assert_eq!(journal.0.state.teaching_pending.load(Ordering::Relaxed), 0);
        assert_eq!(
            journal.0.state.teaching_dropped.load(Ordering::Relaxed),
            100 + admission_attempts - accepted.len() as u64
        );
    }

    #[test]
    fn teaching_concurrent_admission_close_flushes_exactly_the_accepted_set() {
        let dir = tempdir().unwrap();
        let journal = Journal::start(dir.path().join("events.sqlite3")).unwrap();
        let first = teaching_capture("first", 1, 3);
        let mut first_attempts = 0;
        wait_for(|| {
            first_attempts += 1;
            journal.record_teaching(&first)
        });
        let barrier = Arc::new(std::sync::Barrier::new(5));
        let producers: Vec<_> = (0..4)
            .map(|worker| {
                let journal = journal.clone();
                let barrier = barrier.clone();
                thread::spawn(move || {
                    barrier.wait();
                    (0..100)
                        .filter_map(|i| {
                            let id = format!("parallel-{worker}-{i}");
                            let capture = teaching_capture(&id, i, 3);
                            journal.record_teaching(&capture).then_some(id)
                        })
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        barrier.wait();
        assert!(journal.shutdown(Duration::from_secs(8)));
        let mut accepted = vec!["first".to_owned()];
        for producer in producers {
            accepted.extend(producer.join().unwrap());
        }
        let mut actual: Vec<_> = journal
            .teachings()
            .unwrap()
            .into_iter()
            .map(|s| s.id)
            .collect();
        accepted.sort();
        actual.sort();
        assert_eq!(actual, accepted);
        assert_eq!(journal.0.state.teaching_pending.load(Ordering::Relaxed), 0);
        assert_eq!(
            journal.0.state.teaching_dropped.load(Ordering::Relaxed),
            400 + first_attempts - accepted.len() as u64
        );
        assert!(!journal.record_teaching(&first));
        let stats = journal.stats();
        assert_eq!(
            (stats.written, stats.pending, stats.dropped, stats.errors),
            (0, 0, 0, 0)
        );
    }

    #[test]
    fn teaching_busy_shutdown_timeout_accounts_abandoned_and_closed_admissions() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        wait_for(|| journal.stats().healthy);
        let locker = Connection::open(path).unwrap();
        locker.execute_batch("BEGIN IMMEDIATE").unwrap();
        let capture = teaching_capture("busy-at-close", 1, 3);
        let mut attempts = 0;
        wait_for(|| {
            attempts += 1;
            journal.record_teaching(&capture)
        });
        wait_for(|| journal.0.state.teaching_errors.load(Ordering::Relaxed) > 0);
        let start = Instant::now();
        assert!(!journal.shutdown(Duration::from_millis(15)));
        assert!(start.elapsed() < Duration::from_millis(250));
        assert!(!journal.record_teaching(&capture));
        wait_for(|| journal.0.state.done.load(Ordering::Acquire));
        assert_eq!(journal.0.state.teaching_pending.load(Ordering::Relaxed), 0);
        assert_eq!(
            journal.0.state.teaching_dropped.load(Ordering::Relaxed),
            attempts + 1
        );
        assert_teaching_matches(
            &journal.teaching(&capture.summary.id).unwrap().unwrap(),
            &capture,
        );
        assert!(journal.stored_teachings().unwrap().is_empty());
        let stats = journal.stats();
        assert_eq!(
            (stats.written, stats.pending, stats.dropped, stats.errors),
            (0, 0, 0, 0)
        );
        locker.execute_batch("ROLLBACK").unwrap();
    }

    #[test]
    fn teaching_inflight_and_queued_captures_remain_readable_and_plannable_until_reopen() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        wait_for(|| journal.stats().healthy);
        let locker = Connection::open(&path).unwrap();
        locker.execute_batch("BEGIN IMMEDIATE").unwrap();
        let captures = [
            teaching_capture("in-flight", 1, 3),
            teaching_capture("queued-a", 2, 5),
            teaching_capture("queued-b", 3, 7),
        ];
        wait_for(|| journal.record_teaching(&captures[0]));
        wait_for(|| journal.0.state.teaching_errors.load(Ordering::Relaxed) > 0);
        for capture in &captures[1..] {
            wait_for(|| journal.record_teaching(capture));
        }
        {
            let state = journal.0.state.teachings.lock().unwrap();
            assert_eq!(state.queue.len(), 2);
            assert_eq!(state.uncommitted.len(), 3);
            assert_eq!(state.queue.capacity(), TEACHING_CAPACITY);
            assert_eq!(state.uncommitted.capacity(), TEACHING_CAPACITY + 1);
        }
        assert!(journal.stored_teachings().unwrap().is_empty());
        assert_eq!(journal.teachings().unwrap().len(), 3);
        let model = crate::kinematics::Kinematics::load("right").unwrap();
        for capture in &captures {
            let trajectory = journal.teaching(&capture.summary.id).unwrap().unwrap();
            assert_teaching_matches(&trajectory, capture);
            let program = crate::teaching::plan(
                &trajectory,
                [0.0; 7],
                &model,
                crate::trajectory::MotionTuning::default(),
                crate::teaching::PlaybackOptions::default(),
            )
            .unwrap();
            assert_eq!(program.trajectory_id(), capture.summary.id);
            assert!(
                program
                    .preview(&model)
                    .unwrap()
                    .points
                    .iter()
                    .any(|point| point.phase == "path")
            );
        }
        locker.execute_batch("ROLLBACK").unwrap();
        assert!(journal.shutdown(Duration::from_secs(8)));
        assert!(
            journal
                .0
                .state
                .teachings
                .lock()
                .unwrap()
                .uncommitted
                .is_empty()
        );
        assert_eq!(journal.teachings().unwrap().len(), 3);
        drop(journal);
        let reopened = Journal::start(path).unwrap();
        wait_for(|| reopened.stats().healthy);
        for capture in &captures {
            let trajectory = reopened.teaching(&capture.summary.id).unwrap().unwrap();
            assert_teaching_matches(&trajectory, capture);
            assert!(
                crate::teaching::plan(
                    &trajectory,
                    [0.0; 7],
                    &model,
                    crate::trajectory::MotionTuning::default(),
                    crate::teaching::PlaybackOptions::default(),
                )
                .is_ok()
            );
        }
        assert_eq!(reopened.teachings().unwrap().len(), 3);
        assert!(reopened.shutdown(Duration::from_secs(2)));
    }

    #[test]
    fn teaching_old_upsert_commit_cannot_remove_a_newer_uncommitted_capture() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        wait_for(|| journal.stats().healthy);
        let locker = Connection::open(&path).unwrap();
        locker.busy_timeout(Duration::from_secs(1)).unwrap();
        locker
            .execute_batch(
                "CREATE TRIGGER pause_newer_teaching BEFORE INSERT ON teachings
             WHEN NEW.created_at_ms=2 BEGIN SELECT RAISE(ABORT, 'pause newer upsert'); END;
             BEGIN IMMEDIATE;",
            )
            .unwrap();
        let older = teaching_capture("upsert", 1, 3);
        let newer = teaching_capture("upsert", 2, 9);
        wait_for(|| journal.record_teaching(&older));
        wait_for(|| journal.0.state.teaching_errors.load(Ordering::Relaxed) > 0);
        wait_for(|| journal.record_teaching(&newer));
        let newer_task = {
            let state = journal.0.state.teachings.lock().unwrap();
            assert_eq!(state.uncommitted.len(), 2);
            state.queue.front().unwrap().clone()
        };
        assert_teaching_matches(&journal.teaching("upsert").unwrap().unwrap(), &newer);
        locker.execute_batch("ROLLBACK").unwrap();
        wait_for(|| {
            journal
                .stored_teachings()
                .is_ok_and(|summaries| summaries.len() == 1 && summaries[0].created_at_ms == 1)
        });
        wait_for(|| journal.0.state.teaching_pending.load(Ordering::Relaxed) == 1);
        {
            let state = journal.0.state.teachings.lock().unwrap();
            assert_eq!(state.uncommitted.len(), 1);
            assert!(Arc::ptr_eq(&state.uncommitted[0], &newer_task));
        }
        assert_teaching_matches(&journal.teaching("upsert").unwrap().unwrap(), &newer);
        let summaries = journal.teachings().unwrap();
        assert_eq!(summaries.len(), 1);
        assert_eq!(summaries[0].sample_count, 9);
        locker
            .execute_batch("DROP TRIGGER pause_newer_teaching")
            .unwrap();
        assert!(journal.shutdown(Duration::from_secs(8)));
        assert!(
            journal
                .0
                .state
                .teachings
                .lock()
                .unwrap()
                .uncommitted
                .is_empty()
        );
        drop(journal);
        let reopened = Journal::start(path).unwrap();
        wait_for(|| reopened.stats().healthy);
        assert_teaching_matches(&reopened.teaching("upsert").unwrap().unwrap(), &newer);
        assert!(reopened.shutdown(Duration::from_secs(2)));
    }

    #[test]
    fn teaching_retirement_frees_chunks_on_the_writer_including_duplicate_admissions() {
        for admissions in [1, 2] {
            let dir = tempdir().unwrap();
            let journal = Journal::start(dir.path().join("events.sqlite3")).unwrap();
            wait_for(|| journal.stats().healthy);
            journal
                .0
                .state
                .pause_retirement
                .store(true, Ordering::Release);
            let capture = teaching_capture("retire-on-writer", 1, 8193);
            let alive = capture.data_alive();
            for _ in 0..admissions {
                wait_for(|| journal.record_teaching(&capture));
            }
            wait_for(|| journal.0.state.teaching_pending.load(Ordering::Relaxed) == 0);
            assert!(
                journal
                    .0
                    .state
                    .teachings
                    .lock()
                    .unwrap()
                    .uncommitted
                    .is_empty()
            );
            drop(capture);
            assert!(
                alive(),
                "control-side final capture drop released its chunks"
            );
            journal
                .0
                .state
                .pause_retirement
                .store(false, Ordering::Release);
            wait_for(|| !alive());
            let saved = journal.teaching("retire-on-writer").unwrap().unwrap();
            assert_eq!(saved.points.len(), 8193);
            let stats = journal.stats();
            assert_eq!(
                (stats.written, stats.pending, stats.dropped, stats.errors),
                (0, 0, 0, 0)
            );
            assert!(journal.shutdown(Duration::from_secs(2)));
        }
    }

    #[test]
    fn teaching_summary_and_points_roll_back_together() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.sqlite3");
        let mut conn = open_writer(&path).unwrap();
        let original = teaching_capture("atomic", 1, 3);
        write_teaching(&mut conn, &original, &mut None).unwrap();
        conn.execute_batch(
            "CREATE TRIGGER reject_teaching_points BEFORE UPDATE ON teaching_points
             BEGIN SELECT RAISE(ABORT, 'point write failed'); END;",
        )
        .unwrap();
        let changed = teaching_capture("atomic", 2, 9);
        assert!(write_teaching(&mut conn, &changed, &mut None).is_err());
        let (summary, points): (String, String) = conn.query_row(
            "SELECT t.payloadJSON,p.payloadJSON FROM teachings t JOIN teaching_points p ON p.id=t.id",
            [], |row| Ok((row.get(0)?, row.get(1)?)),
        ).unwrap();
        assert_eq!(
            serde_json::from_str::<serde_json::Value>(&summary).unwrap(),
            serde_json::to_value(&original.summary).unwrap()
        );
        assert_eq!(
            serde_json::from_str::<serde_json::Value>(&points).unwrap(),
            serde_json::to_value(original.trajectory().points).unwrap()
        );
    }

    #[test]
    fn teaching_legacy_database_reads_are_empty_and_do_not_migrate_on_read() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        wait_for(|| journal.stats().healthy);
        assert!(journal.shutdown(Duration::from_secs(2)));
        let conn = Connection::open(&path).unwrap();
        conn.execute_batch("DROP TABLE teaching_points; DROP TABLE teachings;")
            .unwrap();
        let before: i64 = conn
            .query_row("PRAGMA schema_version", [], |row| row.get(0))
            .unwrap();
        assert!(journal.teachings().unwrap().is_empty());
        assert!(journal.teaching("missing").unwrap().is_none());
        let after: i64 = conn
            .query_row("PRAGMA schema_version", [], |row| row.get(0))
            .unwrap();
        assert_eq!(before, after);
        assert!(!teaching_table_exists(&conn).unwrap());
        drop(conn);
        drop(journal);
        let reopened = Journal::start(path).unwrap();
        let capture = teaching_capture("after-migration", 10, 5);
        wait_for(|| reopened.record_teaching(&capture));
        assert!(reopened.shutdown(Duration::from_secs(2)));
        assert_teaching_matches(
            &reopened.teaching(&capture.summary.id).unwrap().unwrap(),
            &capture,
        );
    }

    #[test]
    fn teaching_concurrent_reads_keep_summary_and_points_on_the_same_commit() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        let original = teaching_capture("concurrent-upsert", 1, 3);
        wait_for(|| journal.record_teaching(&original));
        wait_for(|| {
            journal
                .stored_teachings()
                .is_ok_and(|summaries| !summaries.is_empty())
        });
        let barrier = Arc::new(std::sync::Barrier::new(2));
        let writer_barrier = barrier.clone();
        let updates = thread::spawn(move || {
            let mut conn = Connection::open(path).unwrap();
            conn.busy_timeout(Duration::from_secs(1)).unwrap();
            writer_barrier.wait();
            for i in 0..100 {
                let capture = teaching_capture("concurrent-upsert", 1000 + i, 3 + i as usize);
                write_teaching(&mut conn, &capture, &mut None).unwrap();
            }
        });
        barrier.wait();
        for _ in 0..100 {
            let trajectory = journal.teaching(&original.summary.id).unwrap().unwrap();
            assert_eq!(trajectory.summary.sample_count, trajectory.points.len());
            assert_eq!(
                trajectory.summary.duration_ms,
                trajectory.points.last().unwrap().t_ms
            );
        }
        updates.join().unwrap();
        let last = journal.teaching(&original.summary.id).unwrap().unwrap();
        assert_eq!(last.summary.created_at_ms, 1099);
        assert_eq!(last.points.len(), 102);
        assert!(journal.shutdown(Duration::from_secs(2)));
    }

    fn calibration_report(id: &str, created_at_ms: i64) -> crate::calibration::Report {
        let baseline = crate::tuning::ServoTuning::default();
        let mut recommended = baseline;
        recommended.friction_enabled = true;
        recommended.joints[2].coulomb = 0.25;
        recommended.joints[2].stribeck = crate::tuning::StribeckTuning {
            enabled: true,
            positive: crate::tuning::FrictionCurve {
                coulomb: 0.12,
                static_friction: 0.24,
                viscous: 0.2,
                stribeck_velocity: 0.025,
            },
            negative: crate::tuning::FrictionCurve {
                coulomb: 0.15,
                static_friction: 0.28,
                viscous: 0.25,
                stribeck_velocity: 0.035,
            },
        };
        serde_json::from_value(serde_json::json!({
            "id": id,
            "request_id": "calibration-request",
            "created_at_ms": created_at_ms,
            "side": "left",
            "port": "test-port",
            "settings": { "axis": 2, "excursion": 0.1, "speed": 0.02, "max_friction": 0.3 },
            "center_reference": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            "center_measured": [0.01, 0.11, 0.21, 0.31, 0.41, 0.51, 0.61],
            "tcp": { "translation": [0.01, 0.02, 0.03], "rotation": [0.0, 0.0, 0.0, 1.0] },
            "baseline": baseline,
            "recommended": recommended,
            "trials": [{
                "index": 0, "phase": "baseline", "config": baseline,
                "metrics": {
                    "windows": 12, "velocity_rms": 0.001, "tracking_rms": 0.002,
                    "tracking_peak": 0.003, "other_peak": 0.0001, "valid_ms": 1600
                }
            }, { "index": 1, "phase": "candidate", "config": recommended, "metrics": null }],
            "identification": [{
                "axis": 2, "source": "feedback_effort_residual", "samples": 1200, "inertia": 0.03, "max_friction": 0.3,
                "positive": { "curve": recommended.joints[2].stribeck.positive, "samples": 600,
                    "velocity_min": 0.001, "velocity_max": 0.1, "rmse": 0.002,
                    "points": [{"velocity": 0.02, "effort": 0.15, "predicted": 0.148, "samples": 30}] },
                "negative": { "curve": recommended.joints[2].stribeck.negative, "samples": 600,
                    "velocity_min": -0.1, "velocity_max": -0.001, "rmse": 0.003,
                    "points": [{"velocity": -0.02, "effort": -0.18, "predicted": -0.177, "samples": 30}] }
            }],
            "outcome": "improved",
            "improvement": 0.15,
            "message": "低速报告，含引号 ' 与换行\n完整保存"
        }))
        .unwrap()
    }

    fn calibration_with_curves(id: &str, created_at_ms: i64, samples: usize) -> Report {
        let mut report = calibration_report(id, created_at_ms);
        report.sample_count = samples;
        report.choices = vec![crate::calibration::Choice {
            axis: 2,
            baseline_index: 0,
            candidate_index: 1,
            after_index: None,
            velocity_change: -0.1,
            tracking_change: -0.2,
        }];
        report.curves = Arc::new(
            (0..samples)
                .map(|i| crate::calibration::Point {
                    space: "joint".into(),
                    t_ms: i as u32 * 20,
                    trial: usize::from(i >= samples / 2),
                    axis: 2,
                    reference: [(i % 32) as f64 / 1024.0; 7],
                    command: Some([(i % 32) as f64 / 1024.0; 7]),
                    feedback: [((i % 32) as f64 - 1.0) / 1024.0; 7],
                    velocity: [1.0 / 128.0; 7],
                    feedback_velocity: [3.0 / 256.0; 7],
                    feedback_age_ms: [2; 7],
                    command_at_ms: Some(i as u32 * 20),
                    effort: None,
                    feedforward_effort: None,
                    driver_temperature: None,
                    winding_temperature: None,
                })
                .collect(),
        );
        report
    }

    fn assert_report_matches(actual: &Report, expected: &Report) {
        assert_eq!(
            serde_json::to_value(actual.summary()).unwrap(),
            serde_json::to_value(expected.summary()).unwrap()
        );
        assert_eq!(actual.curves.len(), expected.curves.len());
        for (i, (actual, expected)) in actual.curves.iter().zip(expected.curves.iter()).enumerate()
        {
            assert_eq!(
                serde_json::to_value(actual).unwrap(),
                serde_json::to_value(expected).unwrap(),
                "curve sample {i}"
            );
        }
    }

    #[test]
    fn borrowed_calibration_retries_after_contention_and_keeps_an_owned_persisted_copy() {
        let dir = tempdir().unwrap();
        let journal = Journal::start(dir.path().join("events.sqlite3")).unwrap();
        wait_for(|| journal.stats().healthy);
        let mut report = calibration_with_curves("borrowed-retry", 1234, 6001);
        let expected = report.clone();
        thread::scope(|scope| {
            let (completed_tx, completed_rx) = std::sync::mpsc::channel();
            let (producer, completed) = journal.with_calibration_queue_locked(|| {
                let borrowed = &report;
                let worker = &journal;
                let producer = scope.spawn(move || {
                    completed_tx
                        .send(worker.record_calibration(borrowed))
                        .unwrap();
                });
                let completed = completed_rx.recv_timeout(Duration::from_secs(1));
                (producer, completed)
            });
            producer.join().unwrap();
            assert!(!completed.expect("borrowed report waited for the queue mutex"));
        });
        assert_report_matches(&report, &expected);
        let stats = journal.stats();
        assert_eq!(
            (stats.written, stats.pending, stats.dropped, stats.errors),
            (0, 0, 0, 0)
        );
        let locker = Connection::open(&journal.0.path).unwrap();
        locker.execute_batch("BEGIN IMMEDIATE").unwrap();
        let curve_owners = Arc::strong_count(&report.curves);
        wait_for(|| journal.record_calibration(&report));
        // A blocked writer retains the accepted report, sharing the exact point allocation.
        assert!(Arc::strong_count(&report.curves) > curve_owners);
        report.message = "caller changed its report after admission".into();
        report.trials.clear();
        Arc::make_mut(&mut report.curves).clear();
        report.sample_count = 0;
        report.choices.clear();
        locker.execute_batch("ROLLBACK").unwrap();
        assert!(journal.shutdown(Duration::from_secs(2)));
        assert_report_matches(
            &journal.calibration(&report.id).unwrap().unwrap(),
            &expected,
        );
        let stats = journal.stats();
        assert_eq!(
            (stats.written, stats.pending, stats.dropped, stats.errors),
            (0, 0, 0, 0)
        );
    }

    #[test]
    fn borrowed_calibration_close_while_queue_locked_is_bounded_and_keeps_event_counts() {
        let dir = tempdir().unwrap();
        let journal = Journal::start(dir.path().join("events.sqlite3")).unwrap();
        wait_for(|| journal.stats().healthy);
        let report = calibration_report("borrowed-close", 1234);
        let expected = serde_json::to_value(&report).unwrap();
        thread::scope(|scope| {
            let (completed_tx, completed_rx) = std::sync::mpsc::channel();
            let (closer, completed) = journal.with_calibration_queue_locked(|| {
                let borrowed = &report;
                let worker = &journal;
                let closer = scope.spawn(move || {
                    let start = Instant::now();
                    let _ = worker.shutdown(Duration::from_millis(15));
                    let accepted = worker.record_calibration(borrowed);
                    completed_tx.send((accepted, start.elapsed())).unwrap();
                });
                let completed = completed_rx.recv_timeout(Duration::from_secs(1));
                (closer, completed)
            });
            closer.join().unwrap();
            let (accepted, elapsed) =
                completed.expect("close or late admission waited for the queue mutex");
            assert!(!accepted);
            assert!(elapsed < Duration::from_millis(250));
        });
        wait_for(|| journal.0.state.done.load(Ordering::Acquire));
        assert!(!journal.record_calibration(&report));
        assert_eq!(serde_json::to_value(&report).unwrap(), expected);
        assert!(journal.calibration(&report.id).unwrap().is_none());
        let stats = journal.stats();
        assert_eq!(
            (stats.written, stats.pending, stats.dropped, stats.errors),
            (0, 0, 0, 0)
        );
    }

    #[test]
    fn calibration_report_flushes_on_shutdown_and_survives_reopen_without_event_counts() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("nested/events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        let attempts = log_until_queued(&journal, || {
            journal.log("info", "app", "event", "event alongside report", None);
        });
        let report = calibration_with_curves("report-1", 1234, 6001);
        wait_for(|| journal.record_calibration(&report));
        assert!(journal.shutdown(Duration::from_secs(8)));
        let stats = journal.stats();
        assert_eq!((stats.written, stats.pending, stats.errors), (1, 0, 0));
        assert_eq!(stats.dropped, attempts - 1);
        assert_eq!(journal.recent(20).unwrap().len(), 1);
        drop(journal);

        let reopened = Journal::start(path).unwrap();
        wait_for(|| reopened.stats().healthy);
        let saved = reopened.calibration("report-1").unwrap().unwrap();
        assert_report_matches(&saved, &report);
        let history = reopened.calibrations(20).unwrap();
        assert_eq!(history.len(), 1);
        assert_eq!(history[0].id, "report-1");
        assert!(history[0].curves.is_empty());
        assert_eq!(history[0].sample_count, 6001);
        assert_eq!(
            serde_json::to_value(&history[0]).unwrap(),
            serde_json::to_value(report.summary()).unwrap()
        );
        let conn = Connection::open(&reopened.0.path).unwrap();
        let summary: String = conn
            .query_row("SELECT payloadJSON FROM calibrations", [], |row| row.get(0))
            .unwrap();
        let curves: String = conn
            .query_row("SELECT payloadJSON FROM calibration_curves", [], |row| {
                row.get(0)
            })
            .unwrap();
        assert!(summary.len() * 50 < curves.len());
        assert_eq!(
            serde_json::from_str::<serde_json::Value>(&summary).unwrap()["curves"],
            serde_json::json!([])
        );
        assert!(reopened.calibration("missing").unwrap().is_none());
        let stats = reopened.stats();
        assert_eq!(
            (stats.written, stats.pending, stats.dropped, stats.errors),
            (0, 0, 0, 0)
        );
        assert!(reopened.shutdown(Duration::from_secs(2)));
    }

    #[test]
    fn calibration_temperatures_and_motor_fault_survive_async_write_and_reopen() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        let mut report = calibration_with_curves("overheat", 1234, 2);
        let driver = [32.0, 33.5, 37.0, 29.5, 34.0, 42.5, 35.0];
        let winding = [39.5, 35.0, 44.5, 36.0, 41.0, 86.5, 40.0];
        let feedforward = [-0.25, 0.0, 0.125, 0.75, -0.5, 1.625, -0.125];
        let points = Arc::make_mut(&mut report.curves);
        points[1].driver_temperature = Some(driver);
        points[1].winding_temperature = Some(winding);
        points[1].feedforward_effort = Some(feedforward);
        report.outcome = "failed".into();
        report.improvement = None;
        report.recommended = None;
        report.message = "J6 电机绕组过热（故障码 512）".into();
        let fault = serde_json::json!({
            "axis": 5, "code": 512, "message": report.message, "t_ms": 23,
            "driver_temperature": 43.0, "winding_temperature": 87.0,
            "position": -0.125, "command": -0.121, "effort": 1.875,
            "feedforward_effort": 1.5, "voltage": 23.75
        });
        report.fault = Some(serde_json::from_value(fault.clone()).unwrap());
        wait_for(|| journal.record_calibration(&report));
        assert!(journal.shutdown(Duration::from_secs(2)));
        drop(journal);

        let reopened = Journal::start(path).unwrap();
        wait_for(|| reopened.stats().healthy);
        let detail = reopened.calibration(&report.id).unwrap().unwrap();
        let history = reopened.calibrations(1).unwrap();
        for stored in [&detail, &history[0]] {
            assert_eq!(stored.message, report.message);
            assert_eq!(serde_json::to_value(&stored.fault).unwrap(), fault);
        }
        assert!(history[0].curves.is_empty());
        assert_eq!(detail.curves.len(), 2);
        assert_eq!(detail.curves[0].driver_temperature, None);
        assert_eq!(detail.curves[0].winding_temperature, None);
        assert_eq!(detail.curves[1].driver_temperature, Some(driver));
        assert_eq!(detail.curves[1].winding_temperature, Some(winding));
        assert_eq!(detail.curves[1].feedforward_effort, Some(feedforward));
        assert!(reopened.shutdown(Duration::from_secs(2)));
    }

    #[test]
    fn calibration_history_retains_newest_orders_ties_and_upserts_literal_ids() {
        let dir = tempdir().unwrap();
        let journal = Journal::start(dir.path().join("events.sqlite3")).unwrap();
        // Insert newest first: retention must use report time and ID, not insertion order.
        for i in (0..105).rev() {
            let report = calibration_with_curves(&format!("calibration-{i:03}"), 1000 + i / 2, 3);
            wait_for(|| journal.record_calibration(&report));
        }
        let marker = calibration_with_curves("marker", 10_000, 4);
        wait_for(|| journal.record_calibration(&marker));
        wait_for(|| journal.calibration("marker").is_ok_and(|r| r.is_some()));
        let history = journal.calibrations(usize::MAX).unwrap();
        assert_eq!(history.len(), 100);
        assert_eq!(history[0].id, "marker");
        assert_eq!(history[1].id, "calibration-104");
        assert_eq!(history[2].id, "calibration-103");
        assert_eq!(history[3].id, "calibration-102");
        assert_eq!(history[99].id, "calibration-006");
        assert!(journal.calibration("calibration-005").unwrap().is_none());
        let first_page = journal.calibrations(20).unwrap();
        assert_eq!(first_page.len(), 20);
        assert_eq!(first_page[19].id, "calibration-086");
        assert!(journal.calibrations(0).unwrap().is_empty());

        let mut updated = calibration_with_curves("calibration-100", 20_000, 1);
        updated.message = "updated report".into();
        updated.recommended = None;
        wait_for(|| journal.record_calibration(&updated));
        wait_for(|| {
            journal
                .calibration("calibration-100")
                .is_ok_and(|r| r.is_some_and(|r| r.message == "updated report"))
        });
        assert_eq!(journal.calibrations(usize::MAX).unwrap().len(), 100);
        let newest = journal.calibrations(2).unwrap();
        assert_eq!(newest[0].id, "calibration-100");
        assert_eq!(newest[0].created_at_ms, 20_000);
        assert!(newest[0].recommended.is_none());
        assert_eq!(newest[1].id, "marker");
        let detail = journal.calibration("calibration-100").unwrap().unwrap();
        assert_eq!(detail.sample_count, 1);
        assert_eq!(detail.curves.len(), 1);

        let literal = calibration_with_curves("quote' OR 1=1 --", 30_000, 2);
        wait_for(|| journal.record_calibration(&literal));
        assert!(journal.shutdown(Duration::from_secs(8)));
        assert!(journal.calibration("unknown' OR 1=1 --").unwrap().is_none());
        assert_eq!(
            journal.calibration(&literal.id).unwrap().unwrap().id,
            literal.id
        );
        assert_eq!(journal.calibrations(1).unwrap()[0].id, literal.id);
        assert_eq!(journal.calibrations(usize::MAX).unwrap().len(), 100);
        assert!(journal.calibration("calibration-006").unwrap().is_none());
        let conn = Connection::open(&journal.0.path).unwrap();
        let curve_count: i64 = conn
            .query_row("SELECT COUNT(*) FROM calibration_curves", [], |row| {
                row.get(0)
            })
            .unwrap();
        let orphans: i64 = conn
            .query_row(
                "SELECT COUNT(*) FROM calibration_curves WHERE id NOT IN (SELECT id FROM calibrations)",
                [],
                |row| row.get(0),
            )
            .unwrap();
        assert_eq!((curve_count, orphans), (100, 0));
        let stats = journal.stats();
        assert_eq!(
            (stats.written, stats.pending, stats.dropped, stats.errors),
            (0, 0, 0, 0)
        );
        assert!(journal.recent(20).unwrap().is_empty());
    }

    #[test]
    fn calibration_summary_and_curves_roll_back_together_on_curve_write_failure() {
        let dir = tempdir().unwrap();
        let mut conn = open_writer(&dir.path().join("events.sqlite3")).unwrap();
        let original = calibration_with_curves("atomic", 1234, 3);
        write_calibration(&mut conn, &original).unwrap();
        conn.execute_batch(
            "CREATE TRIGGER reject_curve_update BEFORE UPDATE ON calibration_curves
             BEGIN SELECT RAISE(ABORT, 'curve write failed'); END;
             CREATE TRIGGER reject_curve_insert BEFORE INSERT ON calibration_curves
             WHEN NEW.id <> 'atomic'
             BEGIN SELECT RAISE(ABORT, 'curve insert failed'); END;",
        )
        .unwrap();
        let mut changed = calibration_with_curves("atomic", 5678, 5);
        changed.message = "must roll back".into();
        assert!(write_calibration(&mut conn, &changed).is_err());
        assert!(write_calibration(&mut conn, &calibration_with_curves("new", 9999, 2)).is_err());
        let (summary, curves): (String, String) = conn
            .query_row(
                "SELECT c.payloadJSON,p.payloadJSON FROM calibrations c
                 JOIN calibration_curves p ON p.id=c.id",
                [],
                |row| Ok((row.get(0)?, row.get(1)?)),
            )
            .unwrap();
        assert_eq!(
            serde_json::from_str::<serde_json::Value>(&summary).unwrap(),
            serde_json::to_value(original.summary()).unwrap()
        );
        assert_eq!(
            serde_json::from_str::<serde_json::Value>(&curves).unwrap(),
            serde_json::to_value(&original.curves).unwrap()
        );
        let count: i64 = conn
            .query_row("SELECT COUNT(*) FROM calibrations", [], |row| row.get(0))
            .unwrap();
        assert_eq!(count, 1);
    }

    #[test]
    fn legacy_motor_fault_displays_in_logs_and_reports_without_rewriting_history() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.sqlite3");
        let conn = open_writer(&path).unwrap();
        let old_message = "J6 故障码 512";
        conn.execute(
            "INSERT INTO events(ts_ms,level,source,code,message)
             VALUES(1234,'error','device','device_error',?1)",
            [old_message],
        )
        .unwrap();
        let mut metadata = serde_json::to_value(calibration_report("legacy", 1234)).unwrap();
        metadata["outcome"] = "failed".into();
        metadata["message"] = old_message.into();
        metadata["sample_count"] = 1.into();
        metadata.as_object_mut().unwrap().remove("fault");
        let metadata = metadata.to_string();
        let curves = r#"[{"t_ms":20,"trial":0,"axis":5,"reference":[0,0,0,0,0,0,0],
            "feedback":[0,0,0,0,0,-0.125,0],"velocity":[0,0,0,0,0,0,0],
            "feedback_velocity":[0,0,0,0,0,0,0],"feedback_age_ms":[1,1,1,1,1,1,1]}]"#;
        conn.execute(
            "INSERT INTO calibrations VALUES('legacy',1234,?1)",
            [&metadata],
        )
        .unwrap();
        conn.execute(
            "INSERT INTO calibration_curves VALUES('legacy',?1)",
            [curves],
        )
        .unwrap();

        let journal = Journal::start(path).unwrap();
        wait_for(|| journal.stats().healthy);
        let expected = "J6 电机绕组过热（故障码 512）";
        let recent = journal.recent(1).unwrap();
        let page = journal
            .page(LogPageQuery {
                search: old_message.into(),
                ..LogPageQuery::default()
            })
            .unwrap();
        for record in [&recent[0], &page.items[0]] {
            assert_eq!(record.message, expected);
            assert_eq!(record.code, "device_error");
        }
        let history = journal.calibrations(1).unwrap();
        let detail = journal.calibration("legacy").unwrap().unwrap();
        for report in [&history[0], &detail] {
            assert_eq!(report.message, expected);
            assert_eq!(report.sample_count, 1);
            assert!(report.fault.is_none());
        }
        assert!(history[0].curves.is_empty());
        assert_eq!(detail.curves.len(), 1);
        assert_eq!(detail.curves[0].feedback[5], -0.125);
        assert_eq!(detail.curves[0].feedforward_effort, None);
        assert_eq!(detail.curves[0].driver_temperature, None);
        assert_eq!(detail.curves[0].winding_temperature, None);
        assert!(journal.shutdown(Duration::from_secs(2)));
        let stored: (String, String, String) = conn
            .query_row(
                "SELECT events.message,c.payloadJSON,p.payloadJSON FROM events
                 CROSS JOIN calibrations c JOIN calibration_curves p ON p.id=c.id",
                [],
                |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?)),
            )
            .unwrap();
        assert_eq!(
            stored,
            (old_message.to_owned(), metadata, curves.to_owned())
        );
    }

    #[test]
    fn legacy_motor_fault_requires_the_exact_old_format() {
        for original in [
            "J6 故障码 512 附加说明",
            "故障记录：J6 故障码 512",
            "J6 故障码 512\n",
            "J6  故障码 512",
            "J6 故障码  512",
            "J6 故障码 +512",
            "J6 故障码 0512",
            "J6 故障码 0",
            "J6 故障码 4294967296",
            "J0 故障码 512",
            "J8 故障码 512",
            "J06 故障码 512",
            "J6 已记录的说明（故障码 512）",
        ] {
            let mut message = original.to_owned();
            describe_legacy_motor_fault(&mut message);
            assert_eq!(message, original);
        }
        for (joint, code) in [(1, 1), (7, u32::MAX)] {
            let mut message = format!("J{joint} 故障码 {code}");
            describe_legacy_motor_fault(&mut message);
            assert_eq!(
                message,
                format!(
                    "J{joint} {}（故障码 {code}）",
                    motor_fault_description(code)
                )
            );
        }
    }

    #[test]
    fn legacy_calibration_reads_need_no_curve_table_and_writer_reopen_preserves_reports() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        wait_for(|| journal.stats().healthy);
        assert!(journal.shutdown(Duration::from_secs(2)));
        let conn = Connection::open(&path).unwrap();
        conn.execute_batch("DROP TABLE calibration_curves").unwrap();
        let expected = calibration_report("legacy", 1234);
        let mut legacy = serde_json::to_value(&expected).unwrap();
        for field in ["curves", "choices", "sample_count"] {
            legacy.as_object_mut().unwrap().remove(field);
        }
        assert_eq!(legacy["settings"]["axis"], 2);
        conn.execute(
            "INSERT INTO calibrations(id,created_at_ms,payloadJSON) VALUES(?1,?2,?3)",
            params![expected.id, expected.created_at_ms, legacy.to_string()],
        )
        .unwrap();
        let before: i64 = conn
            .query_row("PRAGMA schema_version", [], |row| row.get(0))
            .unwrap();
        let saved = journal.calibration("legacy").unwrap().unwrap();
        assert_eq!(saved.settings.axis, Some(2));
        assert!(saved.curves.is_empty());
        assert!(saved.choices.is_empty());
        assert_eq!(saved.sample_count, 0);
        assert_eq!(
            serde_json::to_value(&saved).unwrap(),
            serde_json::to_value(&expected).unwrap()
        );
        assert_eq!(journal.calibrations(20).unwrap().len(), 1);
        assert!(journal.calibration("missing").unwrap().is_none());
        let after: i64 = conn
            .query_row("PRAGMA schema_version", [], |row| row.get(0))
            .unwrap();
        assert_eq!(before, after);
        assert!(conn.prepare("SELECT * FROM calibration_curves").is_err());
        drop(conn);
        drop(journal);
        let reopened = Journal::start(path).unwrap();
        wait_for(|| reopened.stats().healthy);
        assert_eq!(
            serde_json::to_value(reopened.calibration("legacy").unwrap().unwrap()).unwrap(),
            serde_json::to_value(expected).unwrap()
        );
        assert!(reopened.shutdown(Duration::from_secs(2)));
    }

    #[test]
    fn calibration_history_ignores_curve_json_and_reports_json_or_sql_read_errors() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        let report = calibration_with_curves("read-errors", 1234, 3);
        wait_for(|| journal.record_calibration(&report));
        assert!(journal.shutdown(Duration::from_secs(2)));
        let conn = Connection::open(path).unwrap();
        conn.execute(
            "UPDATE calibration_curves SET payloadJSON='broken json'",
            [],
        )
        .unwrap();
        let history = journal.calibrations(20).unwrap();
        assert_eq!(history.len(), 1);
        assert!(history[0].curves.is_empty());
        assert_eq!(history[0].sample_count, 3);
        assert!(
            journal
                .calibration(&report.id)
                .unwrap_err()
                .contains("curves")
        );
        assert!(journal.calibration("missing").unwrap().is_none());
        conn.execute("UPDATE calibrations SET payloadJSON='broken json'", [])
            .unwrap();
        assert!(journal.calibrations(20).is_err());
        assert!(journal.calibration(&report.id).is_err());
        conn.execute_batch("DROP TABLE calibrations").unwrap();
        assert!(journal.calibrations(20).is_err());
        assert!(journal.calibration("missing").is_err());
        let stats = journal.stats();
        assert_eq!(
            (stats.written, stats.pending, stats.dropped, stats.errors),
            (0, 0, 0, 0)
        );
    }

    #[test]
    fn calibration_contention_and_closed_admission_reject_without_waiting_or_event_counts() {
        let dir = tempdir().unwrap();
        let journal = Journal::start(dir.path().join("events.sqlite3")).unwrap();
        wait_for(|| journal.stats().healthy);
        let queue = journal.0.state.calibrations.lock().unwrap();
        let worker = journal.clone();
        let (completed_tx, completed_rx) = std::sync::mpsc::channel();
        let producer = thread::spawn(move || {
            let report = calibration_report("contended", 1);
            let accepted = (0..100)
                .filter(|_| worker.record_calibration(&report))
                .count();
            completed_tx.send(accepted).unwrap();
        });
        let completed = completed_rx.recv_timeout(Duration::from_secs(1));
        drop(queue);
        producer.join().unwrap();
        assert_eq!(
            completed.expect("report producer waited for the queue mutex"),
            0
        );
        assert!(journal.shutdown(Duration::from_secs(2)));
        let queue = journal.0.state.calibrations.lock().unwrap();
        assert!(!journal.record_calibration(&calibration_report("closed", 2)));
        assert!(queue.is_empty());
        assert_eq!(
            journal.0.state.calibration_pending.load(Ordering::Relaxed),
            0
        );
        assert_eq!(
            journal.0.state.calibration_dropped.load(Ordering::Relaxed),
            101
        );
        let stats = journal.stats();
        assert_eq!(
            (stats.written, stats.pending, stats.dropped, stats.errors),
            (0, 0, 0, 0)
        );
        assert!(journal.calibrations(20).unwrap().is_empty());
    }

    #[test]
    fn calibration_busy_database_saturates_without_blocking_and_recovers_accepted_reports() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        let event_attempts = log_until_queued(&journal, || {
            journal.log("info", "app", "initial", "one event", None);
        });
        wait_for(|| journal.stats().written == 1);
        let locker = Connection::open(&path).unwrap();
        locker.execute_batch("BEGIN IMMEDIATE").unwrap();
        let worker = journal.clone();
        let mut report = calibration_report("busy", 1234);
        let (completed_tx, completed_rx) = std::sync::mpsc::channel();
        let producer = thread::spawn(move || {
            let mut accepted = Vec::new();
            for i in 0..10_000 {
                report.id = format!("busy-{i}");
                if worker.record_calibration(&report) {
                    accepted.push(report.id.clone());
                }
            }
            completed_tx.send(accepted).unwrap();
        });
        let completed = completed_rx.recv_timeout(Duration::from_secs(2));
        // Always release the database before joining a regressed blocking producer.
        if completed.is_err() {
            locker.execute_batch("ROLLBACK").unwrap();
        }
        producer.join().unwrap();
        let accepted = completed.expect("report producer waited for SQLite or queue space");
        assert!(!accepted.is_empty());
        assert!(accepted.len() <= CALIBRATION_CAPACITY + 1);
        wait_for(|| journal.0.state.calibration_errors.load(Ordering::Relaxed) > 0);
        assert_eq!(
            journal.0.state.calibration_pending.load(Ordering::Relaxed),
            accepted.len() as u64
        );
        assert_eq!(
            journal.0.state.calibration_dropped.load(Ordering::Relaxed),
            10_000 - accepted.len() as u64
        );
        let stats = journal.stats();
        assert_eq!(
            (stats.written, stats.pending, stats.dropped, stats.errors),
            (1, 0, event_attempts - 1, 0)
        );
        assert!(journal.calibrations(20).unwrap().is_empty());
        locker.execute_batch("ROLLBACK").unwrap();
        wait_for(|| {
            journal
                .calibrations(100)
                .is_ok_and(|r| r.len() == accepted.len())
        });
        assert!(journal.shutdown(Duration::from_secs(2)));
        let saved: std::collections::HashSet<_> = journal
            .calibrations(100)
            .unwrap()
            .into_iter()
            .map(|r| r.id)
            .collect();
        assert_eq!(saved, accepted.into_iter().collect());
        assert_eq!(
            journal.0.state.calibration_pending.load(Ordering::Relaxed),
            0
        );
        assert_eq!(journal.recent(20).unwrap().len(), 1);
        let stats = journal.stats();
        assert_eq!(
            (stats.written, stats.pending, stats.dropped, stats.errors),
            (1, 0, event_attempts - 1, 0)
        );
    }

    #[test]
    fn calibration_shutdown_timeout_accounts_abandoned_reports_and_drop_never_joins() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        wait_for(|| journal.stats().healthy);
        let locker = Connection::open(&path).unwrap();
        locker.execute_batch("BEGIN IMMEDIATE").unwrap();
        let active = calibration_report("active", 1);
        wait_for(|| journal.record_calibration(&active));
        wait_for(|| journal.0.state.calibration_errors.load(Ordering::Relaxed) > 0);
        let queued = calibration_report("queued", 2);
        wait_for(|| journal.record_calibration(&queued));
        let rejected = journal.0.state.calibration_dropped.load(Ordering::Relaxed);
        let start = Instant::now();
        assert!(!journal.shutdown(Duration::from_millis(15)));
        assert!(start.elapsed() < Duration::from_millis(250));
        assert!(!journal.record_calibration(&calibration_report("closed", 3)));
        wait_for(|| journal.0.state.done.load(Ordering::Acquire));
        assert_eq!(
            journal.0.state.calibration_pending.load(Ordering::Relaxed),
            0
        );
        assert_eq!(
            journal.0.state.calibration_dropped.load(Ordering::Relaxed),
            rejected + 3
        );
        assert!(journal.calibrations(20).unwrap().is_empty());
        let stats = journal.stats();
        assert_eq!(
            (stats.written, stats.pending, stats.dropped, stats.errors),
            (0, 0, 0, 0)
        );
        locker.execute_batch("ROLLBACK").unwrap();

        let second = Journal::start(path).unwrap();
        wait_for(|| second.stats().healthy);
        locker.execute_batch("BEGIN IMMEDIATE").unwrap();
        wait_for(|| second.record_calibration(&active));
        wait_for(|| second.0.state.calibration_errors.load(Ordering::Relaxed) > 0);
        let state = second.0.state.clone();
        let start = Instant::now();
        drop(second);
        assert!(start.elapsed() < Duration::from_millis(100));
        wait_for(|| state.done.load(Ordering::Acquire));
        assert_eq!(state.calibration_pending.load(Ordering::Relaxed), 0);
        assert_eq!(state.pending.load(Ordering::Relaxed), 0);
        assert_eq!(state.dropped.load(Ordering::Relaxed), 0);
        assert_eq!(state.errors.load(Ordering::Relaxed), 0);
        locker.execute_batch("ROLLBACK").unwrap();
    }

    #[test]
    fn queue_contention_drops_without_waiting_and_calls_after_close_still_count() {
        let dir = tempdir().unwrap();
        let journal = Journal::start(dir.path().join("events.db")).unwrap();
        // Hold the writer's queue mutex until the producer has returned.
        let queue = journal.0.state.queue.lock().unwrap();
        let worker = journal.clone();
        let (completed_tx, completed_rx) = std::sync::mpsc::channel();
        let producer = thread::spawn(move || {
            for _ in 0..100 {
                worker.log("info", "app", "contended", "rejected summary", None);
            }
            completed_tx.send(()).unwrap();
        });
        let completed = completed_rx.recv_timeout(Duration::from_secs(1));
        drop(queue);
        producer.join().unwrap();
        completed.expect("event producer waited for the queue mutex");
        let stats = journal.stats();
        assert_eq!(stats.written, 0);
        assert_eq!(stats.pending, 0);
        assert_eq!(stats.dropped, 100);
        assert_eq!(stats.written + stats.dropped + stats.pending, 100);
        assert_eq!(journal.0.state.gate.load(Ordering::SeqCst), 0);
        assert!(journal.shutdown(Duration::from_secs(2)));
        let queue = journal.0.state.queue.lock().unwrap();
        journal.log("info", "app", "late", "after close", None);
        let stats = journal.stats();
        assert_eq!(stats.written, 0);
        assert_eq!(stats.pending, 0);
        assert_eq!(stats.dropped, 101);
        assert_eq!(stats.written + stats.dropped + stats.pending, 101);
        assert_eq!(journal.0.state.gate.load(Ordering::SeqCst), CLOSED);
        assert!(queue.is_empty());
    }

    #[test]
    fn persisted_query_order_limit_and_reopen() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("nested/events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        let mut submitted = 0;
        for i in 0..600 {
            submitted += log_until_queued(&journal, || {
                journal.log(
                    "info",
                    "control",
                    "accepted",
                    &format!("event {i}"),
                    Some("request-1"),
                );
            });
        }
        assert!(journal.shutdown(Duration::from_secs(8)));
        let stats = journal.stats();
        assert_eq!((stats.written, stats.pending, stats.errors), (600, 0, 0));
        assert_eq!(stats.written + stats.dropped + stats.pending, submitted);
        let records = journal.recent(u32::MAX).unwrap();
        assert_eq!(records.len(), 500);
        assert_eq!(records[0].message, "event 599");
        assert_eq!(records[499].message, "event 100");
        assert!(records.windows(2).all(|r| r[0].id > r[1].id));
        assert_eq!(records[0].request_id.as_deref(), Some("request-1"));
        assert!(records[0].ts_ms > 0);
        let reopened = Journal::start(path).unwrap();
        let submitted = log_until_queued(&reopened, || {
            reopened.log("info", "app", "started", "new session", None);
        });
        assert!(reopened.shutdown(Duration::from_secs(8)));
        let stats = reopened.stats();
        assert_eq!(stats.written + stats.dropped + stats.pending, submitted);
        assert_eq!(reopened.recent(1).unwrap()[0].message, "new session");
        assert!(reopened.recent(0).unwrap().is_empty());
    }

    #[test]
    fn paged_history_survives_reopen_and_new_inserts_do_not_move_snapshot() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        for i in 0..123 {
            log_until_queued(&journal, || {
                journal.log("info", "history", "event", &format!("event {i}"), None);
            });
        }
        assert!(journal.shutdown(Duration::from_secs(8)));
        drop(journal);
        let journal = Journal::start(path).unwrap();
        wait_for(|| journal.stats().healthy);

        let first = journal.page(LogPageQuery::default()).unwrap();
        assert_eq!(
            (first.page, first.page_size, first.total, first.total_pages),
            (1, 50, 123, 3)
        );
        assert_eq!(first.through_id, 123);
        assert_eq!(first.items.len(), 50);
        assert_eq!(first.items[0].message, "event 122");
        assert_eq!(first.items[49].message, "event 73");
        let ids = |page: &LogPage| page.items.iter().map(|r| r.id).collect::<Vec<_>>();
        let second = journal
            .page(LogPageQuery {
                page: 2,
                through_id: Some(first.through_id),
                ..LogPageQuery::default()
            })
            .unwrap();
        let last = journal
            .page(LogPageQuery {
                page: u32::MAX,
                through_id: Some(first.through_id),
                ..LogPageQuery::default()
            })
            .unwrap();
        assert_eq!((last.page, last.items.len()), (3, 23));
        let all: Vec<_> = ids(&first)
            .into_iter()
            .chain(ids(&second))
            .chain(ids(&last))
            .collect();
        assert_eq!(all, (1..=123).rev().collect::<Vec<_>>());
        let normalized = journal
            .page(LogPageQuery {
                page: 0,
                ..LogPageQuery::default()
            })
            .unwrap();
        assert_eq!(normalized.page, 1);
        assert_eq!(ids(&normalized), ids(&first));
        let smallest = journal
            .page(LogPageQuery {
                page: u32::MAX,
                page_size: 0,
                ..LogPageQuery::default()
            })
            .unwrap();
        assert_eq!(
            (smallest.page, smallest.page_size, smallest.total_pages),
            (123, 1, 123)
        );
        assert_eq!(ids(&smallest), vec![1]);
        let largest = journal
            .page(LogPageQuery {
                page: u32::MAX,
                page_size: u32::MAX,
                ..LogPageQuery::default()
            })
            .unwrap();
        assert_eq!(
            (
                largest.page,
                largest.page_size,
                largest.items.len(),
                largest.total_pages
            ),
            (1, 500, 123, 1)
        );

        for i in 0..7 {
            log_until_queued(&journal, || {
                journal.log("info", "history", "new", &format!("new {i}"), None);
            });
        }
        wait_for(|| journal.stats().written == 7);
        let fixed = journal
            .page(LogPageQuery {
                page: 2,
                through_id: Some(first.through_id),
                ..LogPageQuery::default()
            })
            .unwrap();
        assert_eq!(
            (fixed.total, fixed.total_pages, fixed.through_id),
            (123, 3, 123)
        );
        assert_eq!(ids(&fixed), ids(&second));
        let current = journal.page(LogPageQuery::default()).unwrap();
        assert_eq!(
            (current.total, current.through_id, current.items[0].id),
            (130, 130, 130)
        );
        let clamped = journal
            .page(LogPageQuery {
                through_id: Some(i64::MAX),
                ..LogPageQuery::default()
            })
            .unwrap();
        assert_eq!((clamped.through_id, clamped.total), (130, 130));
        let empty_snapshot = journal
            .page(LogPageQuery {
                page: u32::MAX,
                through_id: Some(0),
                ..LogPageQuery::default()
            })
            .unwrap();
        assert_eq!(
            (
                empty_snapshot.page,
                empty_snapshot.total,
                empty_snapshot.total_pages,
                empty_snapshot.through_id
            ),
            (1, 0, 1, 0)
        );
        assert!(empty_snapshot.items.is_empty());
        let roundtrip: LogPage =
            serde_json::from_value(serde_json::to_value(&fixed).unwrap()).unwrap();
        assert_eq!(ids(&roundtrip), ids(&fixed));
        assert!(journal.shutdown(Duration::from_secs(8)));
    }

    #[test]
    fn page_search_matches_legacy_fault_display_and_original_text() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.sqlite3");
        let original = "J6 故障码 512";
        let displayed = "J6 电机绕组过热（故障码 512）";
        let conn = open_writer(&path).unwrap();
        for message in [original, "J6 故障码 0512", displayed] {
            conn.execute(
                "INSERT INTO events(ts_ms,level,source,code,message)
                 VALUES(1,'error','device','device_error',?1)",
                [message],
            )
            .unwrap();
        }
        drop(conn);
        let journal = Journal::start(path.clone()).unwrap();
        for (search, page, total, expected_id) in [
            ("绕组过热", 1, 2, 3),
            ("绕组过热", 2, 2, 1),
            ("j6 电机绕组过热（故障码 512）", 2, 2, 1),
            (original, 1, 1, 1),
        ] {
            let result = journal
                .page(LogPageQuery {
                    search: search.into(),
                    page,
                    page_size: 1,
                    ..LogPageQuery::default()
                })
                .unwrap();
            assert_eq!(result.total, total, "{search}");
            assert_eq!((result.page, result.total_pages), (page, total as u32));
            assert_eq!(result.items.len(), 1);
            assert_eq!(result.items[0].id, expected_id);
            assert_eq!(result.items[0].message, displayed);
        }
        assert!(journal.shutdown(Duration::from_secs(2)));
        let conn = Connection::open_with_flags(path, OpenFlags::SQLITE_OPEN_READ_ONLY).unwrap();
        let stored: String = conn
            .query_row("SELECT message FROM events WHERE id=1", [], |r| r.get(0))
            .unwrap();
        assert_eq!(stored, original);
    }

    #[test]
    fn page_search_matches_all_fields_and_treats_sql_metacharacters_literally() {
        let dir = tempdir().unwrap();
        let journal = Journal::start(dir.path().join("events.db")).unwrap();
        for (level, source, code, message, request_id) in [
            ("NeedLE", "app", "event", "normal", None),
            ("info", "prefix-NEEDLE-suffix", "event", "normal", None),
            ("info", "app", "Needle-event", "normal", None),
            ("info", "app", "event", "contains NEEDLE", None),
            ("info", "app", "event", "normal", Some("req-NeEdLe-1")),
            ("info", "app", "event", "literal 50%_\\'界", None),
            ("info", "app", "event", "literal 50xxX\\'界", None),
        ] {
            log_until_queued(&journal, || {
                journal.log(level, source, code, message, request_id)
            });
        }
        assert!(journal.shutdown(Duration::from_secs(8)));
        for (search, expected) in [
            ("nEeDlE", vec![5, 4, 3, 2, 1]),
            ("req-needle", vec![5]),
            ("50%_", vec![6]),
            ("%", vec![6]),
            ("_", vec![6]),
            ("\\'界", vec![7, 6]),
            ("' OR 1=1 --", vec![]),
        ] {
            let result = journal
                .page(LogPageQuery {
                    search: search.into(),
                    ..LogPageQuery::default()
                })
                .unwrap();
            assert_eq!(result.total, expected.len() as u64, "search {search:?}");
            assert_eq!(
                result.items.iter().map(|r| r.id).collect::<Vec<_>>(),
                expected,
                "search {search:?}"
            );
            assert_eq!(result.through_id, 7);
            assert_eq!(result.total_pages, 1);
        }
        let filtered_page = journal
            .page(LogPageQuery {
                page: 2,
                page_size: 2,
                search: "needle".into(),
                ..LogPageQuery::default()
            })
            .unwrap();
        assert_eq!(
            (
                filtered_page.total,
                filtered_page.total_pages,
                filtered_page.page
            ),
            (5, 3, 2)
        );
        assert_eq!(
            filtered_page.items.iter().map(|r| r.id).collect::<Vec<_>>(),
            vec![3, 2]
        );
        let missing = journal
            .page(LogPageQuery {
                page: u32::MAX,
                search: "absent".into(),
                ..LogPageQuery::default()
            })
            .unwrap();
        assert_eq!(
            (
                missing.page,
                missing.total,
                missing.total_pages,
                missing.through_id
            ),
            (1, 0, 1, 7)
        );
        assert!(missing.items.is_empty());
    }

    #[test]
    fn page_defaults_empty_database_and_utf8_query_validation() {
        let dir = tempdir().unwrap();
        let journal = Journal::start(dir.path().join("events.db")).unwrap();
        wait_for(|| journal.stats().healthy);
        let defaults: LogPageQuery = serde_json::from_str("{}").unwrap();
        assert_eq!((defaults.page, defaults.page_size), (1, 50));
        assert!(defaults.search.is_empty());
        assert_eq!(defaults.through_id, None);
        let empty = journal.page(defaults).unwrap();
        assert_eq!(journal.log_filters().unwrap(), LogFilters::default());
        assert_eq!(
            (
                empty.page,
                empty.page_size,
                empty.total,
                empty.total_pages,
                empty.through_id
            ),
            (1, 50, 0, 1, 0)
        );
        assert!(empty.items.is_empty());
        for query in [
            LogPageQuery {
                through_id: Some(-1),
                ..LogPageQuery::default()
            },
            LogPageQuery {
                search: "x".repeat(513),
                ..LogPageQuery::default()
            },
            LogPageQuery {
                search: "界".repeat(171),
                ..LogPageQuery::default()
            },
        ] {
            assert_eq!(journal.page(query).unwrap_err(), "invalid_log_query");
        }
        let boundary = journal
            .page(LogPageQuery {
                search: format!("{}xx", "界".repeat(170)),
                ..LogPageQuery::default()
            })
            .unwrap();
        assert!(boundary.items.is_empty());
        assert!(journal.shutdown(Duration::from_secs(2)));
    }

    #[test]
    fn combined_filters_cover_persisted_sessions_and_keep_anchored_pages() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.db");
        let journal = Journal::start(path.clone()).unwrap();
        wait_for(|| journal.stats().healthy);
        let conn = Connection::open(&path).unwrap();
        for (ts, level, source, code, message) in [
            (999, "info", "motor", "move", "needle old"),
            (1000, "info", "motor", "move", "needle boundary start"),
            (1001, "info", "motor", "move", "needle middle"),
            (1002, "info", "motor", "move", "needle boundary end"),
            (1003, "info", "motor", "move", "needle newer"),
            (1001, "warn", "motor", "move", "needle warning"),
            (1001, "info", "api", "move", "needle source"),
            (1001, "info", "motor", "move-more", "needle code"),
            (1001, "info", "motor", "move", "other message"),
            (1001, "custom", "new session", "50%_界", "literal text"),
        ] {
            conn.execute(
                "INSERT INTO events(ts_ms,level,source,code,message) VALUES(?1,?2,?3,?4,?5)",
                params![ts, level, source, code, message],
            )
            .unwrap();
        }
        drop(conn);
        assert!(journal.shutdown(Duration::from_secs(2)));
        drop(journal);
        let journal = Journal::start(path.clone()).unwrap();
        wait_for(|| journal.stats().healthy);
        let options = journal.log_filters().unwrap();
        assert_eq!(options.levels, ["custom", "info", "warn"]);
        assert_eq!(options.sources, ["api", "motor", "new session"]);
        assert_eq!(options.codes, ["50%_界", "move", "move-more"]);
        let query = LogPageQuery {
            page_size: 2,
            search: "NEEDLE".into(),
            level: Some("info".into()),
            source: Some("motor".into()),
            code: Some("move".into()),
            from_ms: Some(1000),
            to_ms: Some(1002),
            ..LogPageQuery::default()
        };
        let first = journal.page(query.clone()).unwrap();
        assert_eq!((first.total, first.total_pages, first.page), (3, 2, 1));
        assert_eq!(first.items.iter().map(|r| r.id).collect::<Vec<_>>(), [4, 3]);
        let second_query = LogPageQuery {
            page: 2,
            through_id: Some(first.through_id),
            ..query.clone()
        };
        let second = journal.page(second_query.clone()).unwrap();
        assert_eq!(second.total, 3);
        assert_eq!(second.items.iter().map(|r| r.id).collect::<Vec<_>>(), [2]);
        let conn = Connection::open(&path).unwrap();
        conn.execute("INSERT INTO events(ts_ms,level,source,code,message) VALUES(1001,'info','motor','move','needle after anchor')", []).unwrap();
        drop(conn);
        let stable = journal.page(second_query).unwrap();
        assert_eq!(stable.total, 3);
        assert_eq!(stable.items[0].id, second.items[0].id);
        assert_eq!(journal.page(query.clone()).unwrap().total, 4);
        let boundary = journal
            .page(LogPageQuery {
                from_ms: Some(1000),
                to_ms: Some(1000),
                ..query.clone()
            })
            .unwrap();
        assert_eq!(boundary.total, 1);
        assert_eq!(boundary.items[0].ts_ms, 1000);
        for (query, total) in [
            (
                LogPageQuery {
                    level: Some("warn".into()),
                    ..LogPageQuery::default()
                },
                1,
            ),
            (
                LogPageQuery {
                    source: Some("api".into()),
                    ..LogPageQuery::default()
                },
                1,
            ),
            (
                LogPageQuery {
                    code: Some("move-more".into()),
                    ..LogPageQuery::default()
                },
                1,
            ),
            (
                LogPageQuery {
                    code: Some("50%_界".into()),
                    ..LogPageQuery::default()
                },
                1,
            ),
            (
                LogPageQuery {
                    source: Some("MOTOR".into()),
                    ..LogPageQuery::default()
                },
                0,
            ),
            (
                LogPageQuery {
                    code: Some("' OR 1=1 --".into()),
                    ..LogPageQuery::default()
                },
                0,
            ),
            (
                LogPageQuery {
                    level: Some("".into()),
                    source: Some("".into()),
                    code: Some("".into()),
                    ..LogPageQuery::default()
                },
                11,
            ),
            (
                LogPageQuery {
                    from_ms: Some(i64::MIN),
                    to_ms: Some(i64::MAX),
                    ..LogPageQuery::default()
                },
                11,
            ),
        ] {
            let page = journal.page(query).unwrap();
            assert_eq!(page.total, total);
            assert_eq!(page.items.len(), total as usize);
        }
        assert_eq!(
            journal
                .page(LogPageQuery {
                    from_ms: Some(1002),
                    to_ms: Some(1001),
                    ..LogPageQuery::default()
                })
                .unwrap_err(),
            "invalid_log_query"
        );
        assert!(journal.shutdown(Duration::from_secs(2)));
    }

    #[test]
    fn unwritable_saturated_producers_recover_without_waiting() {
        let dir = tempdir().unwrap();
        let blocker = dir.path().join("blocked");
        std::fs::write(&blocker, "not a directory").unwrap();
        let journal = Journal::start(blocker.join("events.db")).unwrap();
        wait_for(|| journal.stats().errors > 0);
        let capacity = journal.0.state.queue.lock().unwrap().capacity();
        let start = Instant::now();
        for _ in 0..20_000 {
            journal.log("warn", "app", "offline", "summary", None);
        }
        // Database remains impossible to open for the entire producer run. This
        // deadline catches any accidental send/SQLite busy wait on the caller.
        assert!(start.elapsed() < Duration::from_secs(2));
        let stats = journal.stats();
        assert_eq!(stats.pending, CAPACITY as u64);
        assert_eq!(stats.written + stats.dropped + stats.pending, 20_000);
        assert_eq!(journal.0.state.queue.lock().unwrap().capacity(), capacity);
        assert!(!stats.healthy);
        std::fs::remove_file(&blocker).unwrap();
        wait_for(|| journal.stats().pending == 0 && journal.stats().healthy);
        assert_eq!(journal.stats().written, CAPACITY as u64);
        assert_eq!(journal.recent(5).unwrap().len(), 5);
        assert!(journal.shutdown(Duration::from_secs(2)));
    }

    #[test]
    fn locked_database_timeout_and_nonblocking_drop() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.db");
        let journal = Journal::start(path.clone()).unwrap();
        let initial_attempts = log_until_queued(&journal, || {
            journal.log("info", "app", "start", "before lock", None);
        });
        wait_for(|| journal.stats().written == 1);
        let locker = Connection::open(&path).unwrap();
        locker.execute_batch("BEGIN IMMEDIATE").unwrap();
        let start = Instant::now();
        for _ in 0..10_000 {
            journal.log("info", "app", "event", "under lock", None);
        }
        assert!(start.elapsed() < Duration::from_secs(2));
        wait_for(|| journal.stats().errors > 0);
        let start = Instant::now();
        assert!(!journal.shutdown(Duration::from_millis(15)));
        assert!(start.elapsed() < Duration::from_millis(250));
        journal.log("info", "app", "late", "after close", None);
        wait_for(|| journal.0.state.done.load(Ordering::Acquire));
        let stats = journal.stats();
        assert_eq!(stats.pending, 0);
        assert_eq!(
            stats.written + stats.dropped + stats.pending,
            initial_attempts + 10_001
        );
        assert_eq!(journal.recent(500).unwrap().len(), 1);
        locker.execute_batch("ROLLBACK").unwrap();
        let second = Journal::start(path).unwrap();
        wait_for(|| second.stats().healthy);
        let start = Instant::now();
        drop(second);
        assert!(start.elapsed() < Duration::from_millis(100));
    }

    #[test]
    fn write_lock_recovery_commits_each_accepted_event_once() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.db");
        let journal = Journal::start(path.clone()).unwrap();
        let mut submitted = log_until_queued(&journal, || {
            journal.log("info", "app", "start", "initial", None);
        });
        wait_for(|| journal.stats().written == 1);
        let locker = Connection::open(path).unwrap();
        locker.execute_batch("BEGIN IMMEDIATE").unwrap();
        for i in 0..200 {
            submitted += log_until_queued(&journal, || {
                journal.log("info", "control", "accepted", &format!("event {i}"), None);
            });
        }
        wait_for(|| journal.stats().errors > 0);
        assert_eq!(journal.stats().pending, 200);
        let stats = journal.stats();
        assert_eq!(stats.written + stats.dropped + stats.pending, submitted);
        assert_eq!(journal.recent(500).unwrap().len(), 1);
        locker.execute_batch("ROLLBACK").unwrap();
        wait_for(|| journal.stats().written == 201 && journal.stats().pending == 0);
        assert!(journal.shutdown(Duration::from_secs(2)));
        let records = journal.recent(500).unwrap();
        assert_eq!(records.len(), 201);
        let unique: std::collections::HashSet<_> = records.iter().map(|r| &r.message).collect();
        assert_eq!(unique.len(), 201);
        let stats = journal.stats();
        assert_eq!(stats.dropped, submitted - 201);
        assert_eq!(stats.written + stats.dropped + stats.pending, submitted);
    }

    #[test]
    fn concurrent_admission_close_accounts_every_event() {
        let dir = tempdir().unwrap();
        let journal = Journal::start(dir.path().join("events.db")).unwrap();
        let threads: Vec<_> = (0..4)
            .map(|_| {
                let j = journal.clone();
                thread::spawn(move || {
                    for _ in 0..3000 {
                        j.log("info", "app", "event", "parallel", None);
                    }
                })
            })
            .collect();
        let _ = journal.shutdown(Duration::from_secs(5));
        for t in threads {
            t.join().unwrap();
        }
        wait_for(|| journal.0.state.done.load(Ordering::Acquire));
        let stats = journal.stats();
        assert_eq!(stats.pending, 0);
        assert_eq!(stats.written + stats.dropped + stats.pending, 12_000);
    }

    #[test]
    fn retention_is_persisted_and_utf8_fields_are_bounded() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("events.db");
        let seed = open_writer(&path).unwrap();
        seed.execute_batch(
            "WITH RECURSIVE n(x) AS (VALUES(1) UNION ALL SELECT x+1 FROM n WHERE x<100010)
             INSERT INTO events(ts_ms,level,source,code,message)
             SELECT x,'info','seed','event','old' FROM n;",
        )
        .unwrap();
        drop(seed);
        let journal = Journal::start(path.clone()).unwrap();
        let submitted = log_until_queued(&journal, || {
            journal.log("info", "app", "event", &"界".repeat(3000), None);
        });
        assert!(journal.shutdown(Duration::from_secs(8)));
        let stats = journal.stats();
        assert_eq!(stats.written + stats.dropped + stats.pending, submitted);
        let conn = Connection::open(path).unwrap();
        let count: i64 = conn
            .query_row("SELECT count(*) FROM events", [], |r| r.get(0))
            .unwrap();
        assert_eq!(count, 100_000);
        let recent = journal.recent(1).unwrap();
        assert_eq!(recent[0].message.len(), 2046);
        assert!(recent[0].message.chars().all(|c| c == '界'));
    }
}
