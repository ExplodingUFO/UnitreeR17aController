//! Bounded structured event journal. Never pass credentials or per-frame telemetry here.
//! `recent` performs disk I/O: HTTP callers must use `server`'s spawn_blocking boundary.
use crossbeam_channel::{Receiver, Sender, bounded};
use rusqlite::{Connection, OpenFlags, params};
use serde::{Deserialize, Serialize};
use std::{
    path::{Path, PathBuf},
    sync::{
        Arc,
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
    /// Queue overflow, calls after close, and accepted events abandoned at shutdown.
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

#[derive(Default)]
struct State {
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

struct Shared {
    path: PathBuf,
    tx: Sender<LogRecord>,
    state: Arc<State>,
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
        let (tx, rx) = bounded(CAPACITY);
        let state = Arc::new(State::default());
        let worker_state = state.clone();
        let worker_path = path.clone();
        thread::Builder::new()
            .name("event-journal".into())
            .spawn(move || writer(&worker_path, rx, worker_state))
            .map_err(|e| format!("start journal writer: {e}"))?;
        Ok(Self(Arc::new(Shared { path, tx, state })))
    }

    /// Bounded in-memory work only; fields are truncated at UTF-8 boundaries.
    /// Callers supply event summaries and request IDs, never tokens or lease IDs.
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
        if previous & CLOSED != 0 {
            s.dropped.fetch_add(1, Ordering::Relaxed);
            s.gate.fetch_sub(1, Ordering::SeqCst);
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
        // Account before publication, so a fast consumer cannot underflow pending.
        s.pending.fetch_add(1, Ordering::Relaxed);
        if self.0.tx.try_send(record).is_err() {
            s.pending.fetch_sub(1, Ordering::Relaxed);
            s.dropped.fetch_add(1, Ordering::Relaxed);
        }
        s.gate.fetch_sub(1, Ordering::SeqCst);
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
            .query_map([limit], |r| {
                Ok(LogRecord {
                    id: r.get(0)?,
                    ts_ms: r.get(1)?,
                    level: r.get(2)?,
                    source: r.get(3)?,
                    code: r.get(4)?,
                    message: r.get(5)?,
                    request_id: r.get(6)?,
                })
            })
            .map_err(|e| e.to_string())?;
        rows.collect::<rusqlite::Result<Vec<_>>>()
            .map_err(|e| e.to_string())
    }

    /// Application-close coordination only. True means every accepted event was committed
    /// and the worker exited. False means the deadline expired or events were abandoned.
    /// A blocked OS/SQLite call can outlive this return; no forced cancellation or join.
    pub fn shutdown(&self, timeout: Duration) -> bool {
        let s = &self.0.state;
        s.gate.fetch_or(CLOSED, Ordering::SeqCst);
        let start = Instant::now();
        while !s.done.load(Ordering::Acquire) {
            let remaining = timeout.saturating_sub(start.elapsed());
            if remaining.is_zero() {
                s.abort.store(true, Ordering::Release);
                return false;
            }
            thread::sleep(remaining.min(Duration::from_millis(1)));
        }
        !s.abort.load(Ordering::Acquire)
    }
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

fn writer(path: &Path, rx: Receiver<LogRecord>, state: Arc<State>) {
    let s = &*state;
    let mut conn = None;
    let mut batch = Vec::with_capacity(BATCH_SIZE);
    let mut delay = Duration::from_millis(20);
    let mut checkpoint_at = Instant::now();
    loop {
        if s.abort.load(Ordering::Acquire) {
            let count = batch.len() as u64;
            batch.clear();
            s.dropped.fetch_add(count, Ordering::Relaxed);
            s.pending.fetch_sub(count, Ordering::Relaxed);
            for _ in rx.try_iter() {
                s.dropped.fetch_add(1, Ordering::Relaxed);
                s.pending.fetch_sub(1, Ordering::Relaxed);
            }
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
        if batch.is_empty()
            && let Ok(record) = rx.recv_timeout(POLL)
        {
            batch.push(record);
        }
        while batch.len() < BATCH_SIZE {
            match rx.try_recv() {
                Ok(record) => batch.push(record),
                Err(_) => break,
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

    #[test]
    fn persisted_query_order_limit_and_reopen() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("nested/events.sqlite3");
        let journal = Journal::start(path.clone()).unwrap();
        for i in 0..600 {
            journal.log(
                "info",
                "control",
                "accepted",
                &format!("event {i}"),
                Some("request-1"),
            );
        }
        assert!(journal.shutdown(Duration::from_secs(8)));
        let stats = journal.stats();
        assert_eq!(
            (stats.written, stats.pending, stats.dropped, stats.errors),
            (600, 0, 0, 0)
        );
        let records = journal.recent(u32::MAX).unwrap();
        assert_eq!(records.len(), 500);
        assert_eq!(records[0].message, "event 599");
        assert_eq!(records[499].message, "event 100");
        assert!(records.windows(2).all(|r| r[0].id > r[1].id));
        assert_eq!(records[0].request_id.as_deref(), Some("request-1"));
        assert!(records[0].ts_ms > 0);
        let reopened = Journal::start(path).unwrap();
        reopened.log("info", "app", "started", "new session", None);
        assert!(reopened.shutdown(Duration::from_secs(8)));
        assert_eq!(reopened.recent(1).unwrap()[0].message, "new session");
        assert!(reopened.recent(0).unwrap().is_empty());
    }

    #[test]
    fn unwritable_saturated_producers_recover_without_waiting() {
        let dir = tempdir().unwrap();
        let blocker = dir.path().join("blocked");
        std::fs::write(&blocker, "not a directory").unwrap();
        let journal = Journal::start(blocker.join("events.db")).unwrap();
        wait_for(|| journal.stats().errors > 0);
        let start = Instant::now();
        for _ in 0..20_000 {
            journal.log("warn", "app", "offline", "summary", None);
        }
        // Database remains impossible to open for the entire producer run. This
        // deadline catches any accidental send/SQLite busy wait on the caller.
        assert!(start.elapsed() < Duration::from_secs(2));
        let stats = journal.stats();
        assert_eq!(stats.pending, CAPACITY as u64);
        assert_eq!(stats.pending + stats.dropped, 20_000);
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
        journal.log("info", "app", "start", "before lock", None);
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
        assert_eq!(stats.written + stats.dropped, 10_002);
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
        journal.log("info", "app", "start", "initial", None);
        wait_for(|| journal.stats().written == 1);
        let locker = Connection::open(path).unwrap();
        locker.execute_batch("BEGIN IMMEDIATE").unwrap();
        for i in 0..200 {
            journal.log("info", "control", "accepted", &format!("event {i}"), None);
        }
        wait_for(|| journal.stats().errors > 0);
        assert_eq!(journal.stats().pending, 200);
        assert_eq!(journal.recent(500).unwrap().len(), 1);
        locker.execute_batch("ROLLBACK").unwrap();
        wait_for(|| journal.stats().written == 201 && journal.stats().pending == 0);
        assert!(journal.shutdown(Duration::from_secs(2)));
        let records = journal.recent(500).unwrap();
        assert_eq!(records.len(), 201);
        let unique: std::collections::HashSet<_> = records.iter().map(|r| &r.message).collect();
        assert_eq!(unique.len(), 201);
        assert_eq!(journal.stats().dropped, 0);
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
        assert_eq!(stats.written + stats.dropped, 12_000);
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
        journal.log("info", "app", "event", &"界".repeat(3000), None);
        assert!(journal.shutdown(Duration::from_secs(8)));
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
