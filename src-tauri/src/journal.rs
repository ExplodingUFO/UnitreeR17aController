//! Bounded event journal with an independent short-capture worker.
//! `log` accepts summaries, never credentials or per-frame telemetry; use `sample` for frames.
//! Queue admission tries a mutex once; contention and overflow count as drops.
//! Disk readers must run within `server`'s spawn_blocking boundary.
use crate::trace::Recorder;
pub use crate::trace::{TraceData, TraceRecording, TraceSummary};
use rusqlite::{Connection, OpenFlags, params};
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
}

impl Default for LogPageQuery {
    fn default() -> Self {
        Self {
            page: 1,
            page_size: 50,
            search: String::new(),
            through_id: None,
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

struct State {
    queue: Mutex<VecDeque<LogRecord>>,
    // One atomic admission gate orders close against producers already in flight.
    gate: AtomicUsize,
    pending: AtomicU64,
    written: AtomicU64,
    dropped: AtomicU64,
    errors: AtomicU64,
    healthy: AtomicBool,
    abort: AtomicBool,
    done: AtomicBool,
}

impl Default for State {
    fn default() -> Self {
        Self {
            queue: Mutex::new(VecDeque::with_capacity(CAPACITY)),
            gate: AtomicUsize::new(0),
            pending: AtomicU64::new(0),
            written: AtomicU64::new(0),
            dropped: AtomicU64::new(0),
            errors: AtomicU64::new(0),
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

    /// Observation only. HTTP supplies the 10000ms default; Rust callers supply
    /// 1000..=20000 milliseconds explicitly.
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
        if query.through_id.is_some_and(|id| id < 0) || query.search.len() > 512 {
            return Err("invalid_log_query".into());
        }
        let page_size = query.page_size.clamp(1, 500);
        let search = query.search.to_ascii_lowercase();
        let mut conn = Connection::open_with_flags(
            &self.0.path,
            OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_NO_MUTEX,
        )
        .map_err(|e| format!("open journal reader: {e}"))?;
        conn.busy_timeout(BUSY).map_err(|e| e.to_string())?;
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
            OR instr(lower(request_id),?2) > 0)";
        let total: u64 = tx
            .query_row(
                &format!("SELECT COUNT(*) FROM events WHERE {filter}"),
                params![through_id, search],
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
                     WHERE {filter} ORDER BY id DESC LIMIT ?3 OFFSET ?4"
                ))
                .map_err(|e| e.to_string())?;
            stmt.query_map(
                params![through_id, search, page_size, offset],
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

    /// Application-close coordination only. True means every accepted event/sample
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
    Ok(LogRecord {
        id: row.get(0)?,
        ts_ms: row.get(1)?,
        level: row.get(2)?,
        source: row.get(3)?,
        code: row.get(4)?,
        message: row.get(5)?,
        request_id: row.get(6)?,
    })
}

fn capped(value: &str, max: usize) -> String {
    let mut end = value.len().min(max);
    while !value.is_char_boundary(end) {
        end -= 1;
    }
    value[..end].to_owned()
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
    let mut delay = Duration::from_millis(20);
    let mut checkpoint_at = Instant::now();
    loop {
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
            // Producers admitted just before close can still publish a final event.
            if s.gate.load(Ordering::SeqCst) == CLOSED && s.pending.load(Ordering::Relaxed) == 0 {
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
        } else {
            // Polling avoids producer-side notification and its hidden wakeup locks.
            thread::sleep(POLL);
        }
        let closing =
            s.gate.load(Ordering::SeqCst) == CLOSED && s.pending.load(Ordering::Relaxed) == 0;
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
