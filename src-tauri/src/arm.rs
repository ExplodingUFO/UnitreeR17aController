use crate::{
    journal::{Journal, JournalStats, TraceData},
    kinematics::{
        self, CartesianPath, Frame, Kinematics, LocalTarget, MotionLimits, Pose, SmoothingState,
        StepInput,
    },
    protocol::{BusHandle, DriveMode, Feedback, Setpoint},
    trajectory::{self, JOINT_ACCELERATION, JOINT_JERK, JOINT_SPEED},
};
use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use serde::{Deserialize, Serialize};
use std::{
    collections::VecDeque,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
    },
    time::{Duration, Instant},
};
use tokio::sync::{oneshot, watch};

const LEASE_MS: u64 = 2500;
const FEEDBACK_MS: u64 = 200;
const FEEDBACK_SPAN_MS: u64 = 100;
const ENABLE_TIMEOUT: Duration = Duration::from_secs(1);
const CONTROL_PERIOD: Duration = Duration::from_millis(10);
// The official motion demo tolerates 0.6 rad tracking error while moving.
// Position error is what creates holding torque; do not clamp it so tightly
// that a load drags the commanded reference down with the measured joint.
const MAX_TRACKING_ERROR: f64 = 0.60;
const LINEAR_SPEED: f64 = 0.015;
const ANGULAR_SPEED: f64 = 0.15;
// The official motion demo uses these gains for loaded joint movement.
const SERVO_KP: f64 = 60.0;
// The official R1-7A teleoperation stack uses kd=10 with gravity feedforward.
const SERVO_KD: f64 = 10.0;
const TAU_FF_LIMIT: f64 = 8.0;
const RETURN_SETTLE_TIME: Duration = Duration::from_millis(200);

#[derive(Clone, Debug, Serialize)]
pub struct ApiError {
    pub code: String,
    pub message: String,
}
impl ApiError {
    pub fn new(code: &str, message: impl Into<String>) -> Self {
        Self {
            code: code.into(),
            message: message.into(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Phase {
    Disconnected,
    Connecting,
    Disabled,
    Enabling,
    Ready,
    Moving,
    Stopping,
    Fault,
    Closing,
}

#[derive(Clone, Debug, Serialize)]
pub struct OwnerSnapshot {
    pub client_id: String,
    pub expires_in_ms: u64,
}
#[derive(Clone, Debug, Serialize)]
pub struct JointSnapshot {
    pub id: usize,
    pub name: String,
    pub q: Option<f64>,
    pub dq: Option<f64>,
    pub tau: Option<f64>,
    pub age_ms: Option<u64>,
    pub temperature: Option<f64>,
    pub voltage: Option<f64>,
    pub error: Option<u32>,
}
#[derive(Clone, Debug, Serialize)]
pub struct CommandResult {
    pub request_id: String,
    pub status: String,
    pub code: String,
    pub message: String,
}
#[derive(Clone, Debug, Serialize)]
pub struct ActionSnapshot {
    pub request_id: String,
    pub space: Space,
    pub mode: String,
    pub axis: Option<usize>,
    pub progress: f64,
}
#[derive(Clone, Debug, Default, Serialize)]
pub struct Communication {
    pub received: u64,
    pub crc_errors: u64,
    pub timeouts: u64,
}
#[derive(Clone, Serialize)]
pub struct Snapshot {
    pub seq: u64,
    pub generation: u64,
    pub phase: Phase,
    pub side: String,
    pub port: Option<String>,
    pub enabled: bool,
    pub owner: Option<OwnerSnapshot>,
    pub joints: Vec<JointSnapshot>,
    pub tcp: Option<Pose>,
    pub tcp_transform: Option<Pose>,
    pub tcp_configured: bool,
    pub feedback_age_ms: Option<u64>,
    pub action: Option<ActionSnapshot>,
    pub last_result: Option<CommandResult>,
    pub results: Vec<CommandResult>,
    pub error: Option<ApiError>,
    pub journal: JournalStats,
    pub communication: Communication,
}

#[derive(Clone, Debug, Deserialize)]
pub struct ControlRequest {
    pub action: String,
    pub client_id: String,
    pub lease_id: Option<String>,
}
#[derive(Clone, Debug, Serialize)]
pub struct ControlReply {
    pub lease_id: Option<String>,
    pub expires_in_ms: u64,
    pub generation: u64,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Space {
    Joint,
    Cartesian,
}
fn base_frame() -> Frame {
    Frame::Base
}
fn default_ttl() -> u64 {
    150
}

#[derive(Clone, Debug, Deserialize)]
pub struct CommandRequest {
    pub request_id: String,
    pub client_id: String,
    pub lease_id: String,
    pub generation: u64,
    #[serde(flatten)]
    pub operation: Operation,
}
#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Operation {
    Connect {
        port: String,
        side: String,
    },
    Disconnect {
        #[serde(default)]
        allow_release: bool,
    },
    Enable,
    Disable {
        #[serde(default)]
        allow_release: bool,
    },
    ClearFault,
    ConfigureTcp {
        tcp: Pose,
    },
    JogStart {
        space: Space,
        axis: usize,
        direction: i8,
        speed: f64,
        #[serde(default = "base_frame")]
        frame: Frame,
        #[serde(default = "default_ttl")]
        ttl_ms: u64,
        input_seq: u64,
    },
    JogUpdate {
        action_id: String,
        input_seq: u64,
        #[serde(default = "default_ttl")]
        ttl_ms: u64,
    },
    Step {
        space: Space,
        axis: usize,
        direction: i8,
        amount: f64,
        speed: f64,
        #[serde(default = "base_frame")]
        frame: Frame,
    },
    ReturnToPose {
        joints: [f64; 7],
        speed: f64,
        #[serde(default)]
        disconnect_after: bool,
    },
    Stop,
}
impl Operation {
    fn name(&self) -> &'static str {
        match self {
            Self::Connect { .. } => "connect",
            Self::Disconnect { .. } => "disconnect",
            Self::Enable => "enable",
            Self::Disable { .. } => "disable",
            Self::ClearFault => "clear_fault",
            Self::ConfigureTcp { .. } => "configure_tcp",
            Self::JogStart { .. } => "jog_start",
            Self::JogUpdate { .. } => "jog_update",
            Self::Step { .. } => "step",
            Self::ReturnToPose { .. } => "return_to_pose",
            Self::Stop => "stop",
        }
    }
}

enum Work {
    PrepareClose(bool, Instant, oneshot::Sender<Result<(), ApiError>>),
    Control(
        ControlRequest,
        Instant,
        oneshot::Sender<Result<ControlReply, ApiError>>,
    ),
    Command(CommandRequest, Instant),
}

#[derive(Clone)]
pub struct ArmHandle {
    ordinary: Sender<Work>,
    priority: Sender<Work>,
    latest: Sender<Work>,
    latest_rx: Receiver<Work>,
    host_epoch: Arc<AtomicU64>,
    stop_reason: Arc<Mutex<Option<String>>>,
    shutdown: Arc<AtomicBool>,
    finished: Receiver<()>,
    state: watch::Receiver<Snapshot>,
}

impl ArmHandle {
    pub fn spawn(journal: Journal) -> Result<Self, String> {
        let core = Core::new(journal);
        let (state_tx, state) = watch::channel(core.snapshot(Instant::now()));
        let (ordinary, ordinary_rx) = bounded(128);
        let (priority, priority_rx) = bounded(32);
        let (latest, latest_rx) = bounded(1);
        let (done_tx, finished) = bounded(1);
        let host_epoch = Arc::new(AtomicU64::new(0));
        let stop_reason = Arc::new(Mutex::new(None));
        let shutdown = Arc::new(AtomicBool::new(false));
        let worker_epoch = host_epoch.clone();
        let worker_reason = stop_reason.clone();
        let worker_shutdown = shutdown.clone();
        let worker_latest = latest_rx.clone();
        std::thread::Builder::new()
            .name("arm-control".into())
            .spawn(move || {
                run(
                    core,
                    ordinary_rx,
                    priority_rx,
                    worker_latest,
                    worker_epoch,
                    worker_reason,
                    worker_shutdown,
                    state_tx,
                );
                let _ = done_tx.try_send(());
            })
            .map_err(|e| e.to_string())?;
        Ok(Self {
            ordinary,
            priority,
            latest,
            latest_rx,
            host_epoch,
            stop_reason,
            shutdown,
            finished,
            state,
        })
    }
    pub fn snapshot(&self) -> Snapshot {
        self.state.borrow().clone()
    }
    pub fn subscribe(&self) -> watch::Receiver<Snapshot> {
        self.state.clone()
    }
    pub fn is_closing(&self) -> bool {
        self.shutdown.load(Ordering::Acquire)
    }
    pub fn host_stop(&self, reason: impl Into<String>) {
        if let Ok(mut current) = self.stop_reason.lock() {
            *current = Some(reason.into());
        }
        self.host_epoch.fetch_add(1, Ordering::AcqRel);
    }
    pub fn request_shutdown(&self) {
        self.shutdown.store(true, Ordering::Release);
        self.host_stop("shutdown");
    }
    pub fn shutdown(&self, timeout: Duration) -> bool {
        self.request_shutdown();
        self.finished.recv_timeout(timeout).is_ok()
    }
    /// Check release intent on the control thread before admitting shutdown.
    pub async fn prepare_close(&self, allow_release: bool) -> Result<(), ApiError> {
        if self.is_closing() {
            return Ok(());
        }
        let (reply, result) = oneshot::channel();
        self.priority
            .try_send(Work::PrepareClose(
                allow_release,
                Instant::now() + Duration::from_millis(300),
                reply,
            ))
            .map_err(|_| ApiError::new("queue_full", "请求队列已满"))?;
        tokio::time::timeout(Duration::from_millis(350), result)
            .await
            .map_err(|_| ApiError::new("timeout", "关闭请求超时"))?
            .map_err(|_| ApiError::new("closing", "控制线程已结束"))?
    }
    pub async fn control(&self, request: ControlRequest) -> Result<ControlReply, ApiError> {
        if self.is_closing() {
            return Err(ApiError::new("closing", "应用正在关闭"));
        }
        let (reply, result) = oneshot::channel();
        let priority = request.action == "release";
        let work = Work::Control(request, Instant::now() + Duration::from_millis(300), reply);
        (if priority {
            &self.priority
        } else {
            &self.ordinary
        })
        .try_send(work)
        .map_err(|_| ApiError::new("queue_full", "请求队列已满"))?;
        tokio::time::timeout(Duration::from_millis(350), result)
            .await
            .map_err(|_| ApiError::new("timeout", "控制请求超时"))?
            .map_err(|_| ApiError::new("closing", "控制线程已结束"))?
    }
    pub fn command(&self, request: CommandRequest) -> Result<(), ApiError> {
        if self.is_closing() {
            return Err(ApiError::new("closing", "应用正在关闭"));
        }
        if request.request_id.is_empty() || request.request_id.len() > 100 {
            return Err(ApiError::new("invalid_request", "请求标识无效"));
        }
        let update = matches!(request.operation, Operation::JogUpdate { .. });
        let stop = matches!(request.operation, Operation::Stop);
        let work = Work::Command(request, Instant::now());
        if update {
            match self.latest.try_send(work) {
                Ok(()) => Ok(()),
                Err(TrySendError::Full(work)) => {
                    let _ = self.latest_rx.try_recv();
                    self.latest
                        .try_send(work)
                        .map_err(|_| ApiError::new("queue_full", "输入更新繁忙"))
                }
                Err(_) => Err(ApiError::new("closing", "控制线程已结束")),
            }
        } else {
            (if stop { &self.priority } else { &self.ordinary })
                .try_send(work)
                .map_err(|_| ApiError::new("queue_full", "请求队列已满"))
        }
    }
}

struct Lease {
    client_id: String,
    secret: String,
    expires: Instant,
}
struct Observation {
    value: Feedback,
    at: Instant,
}
#[derive(Clone, Copy)]
enum MotionMode {
    Continuous,
    Step,
    Return { disconnect_after: bool },
}
impl MotionMode {
    fn timed(self) -> bool {
        !matches!(self, Self::Continuous)
    }
    fn name(self) -> &'static str {
        match self {
            Self::Continuous => "continuous",
            Self::Step => "step",
            Self::Return { .. } => "return",
        }
    }
}
struct Motion {
    request_id: String,
    space: Space,
    axis: usize,
    direction: f64,
    speed: f64,
    frame: Frame,
    mode: MotionMode,
    amount: f64,
    started: Instant,
    duration: f64,
    initial: [f64; 7],
    target: [f64; 7],
    initial_pose: Option<Pose>,
    target_pose: Option<Pose>,
    deadline: Option<Instant>,
    input_seq: u64,
    settled_since: Option<Instant>,
}
struct Core {
    journal: Journal,
    model: Option<Kinematics>,
    side: String,
    port: Option<String>,
    bus: Option<BusHandle>,
    feedback: [Option<Observation>; 7],
    reference: [f64; 7],
    velocity: [f64; 7],
    acceleration: [f64; 7],
    jerk: [f64; 7],
    control_seq: u64,
    control_at: Instant,
    stop_path: Option<CartesianPath>,
    tcp: Option<Pose>,
    enabled: bool,
    close_requested: bool,
    phase: Phase,
    lease: Option<Lease>,
    motion: Option<Motion>,
    stopping: bool,
    generation: u64,
    seq: u64,
    results: VecDeque<CommandResult>,
    seen: VecDeque<(u64, String)>,
    error: Option<ApiError>,
    communication: Communication,
    connected_at: Option<Instant>,
    clear_pending: [bool; 7],
    clear_request: Option<(String, Instant)>,
    enable_request: Option<(String, Instant)>,
    diag_at: Instant,
    diag_cycles: u64,
    diag_dt_sum: f64,
    diag_dt_max: f64,
    diag_work_sum: f64,
    diag_work_max: f64,
    dirty: bool,
}

impl Core {
    fn new(journal: Journal) -> Self {
        let loaded = Kinematics::load("right");
        let error = loaded
            .as_ref()
            .err()
            .map(|e| ApiError::new("model_error", e.clone()));
        Self {
            journal,
            model: loaded.ok(),
            side: "right".into(),
            port: None,
            bus: None,
            feedback: std::array::from_fn(|_| None),
            reference: [0.0; 7],
            velocity: [0.0; 7],
            acceleration: [0.0; 7],
            jerk: [0.0; 7],
            control_seq: 0,
            control_at: Instant::now(),
            stop_path: None,
            tcp: None,
            enabled: false,
            close_requested: false,
            phase: Phase::Disconnected,
            lease: None,
            motion: None,
            stopping: false,
            generation: 1,
            seq: 0,
            results: VecDeque::new(),
            seen: VecDeque::new(),
            error,
            communication: Communication::default(),
            connected_at: None,
            clear_pending: [false; 7],
            clear_request: None,
            enable_request: None,
            diag_at: Instant::now(),
            diag_cycles: 0,
            diag_dt_sum: 0.0,
            diag_dt_max: 0.0,
            diag_work_sum: 0.0,
            diag_work_max: 0.0,
            dirty: true,
        }
    }
    fn record(&mut self, id: &str, status: &str, code: &str, message: impl Into<String>) {
        let result = CommandResult {
            request_id: id.into(),
            status: status.into(),
            code: code.into(),
            message: message.into(),
        };
        self.journal.log(
            if status == "rejected" || status == "failed" {
                "error"
            } else {
                "info"
            },
            "control",
            code,
            &result.message,
            Some(id),
        );
        self.results.push_front(result);
        self.results.truncate(32);
        self.dirty = true;
    }
    fn phase(&mut self) {
        self.phase = if self.error.is_some() && self.bus.is_some() {
            Phase::Fault
        } else if self.bus.is_none() {
            Phase::Disconnected
        } else if self.enable_request.is_some() {
            Phase::Enabling
        } else if self.stopping {
            Phase::Stopping
        } else if self.motion.is_some() {
            Phase::Moving
        } else if self.enabled {
            Phase::Ready
        } else {
            Phase::Disabled
        };
    }
    fn end_motion(&mut self, status: &str, code: &str, message: &str) {
        if let Some((id, _)) = self.enable_request.take() {
            self.enabled = false;
            self.reset_derivatives();
            self.record(&id, status, code, message);
        }
        if let Some(motion) = self.motion.take() {
            self.stop_path = Self::motion_path(&motion);
            self.record(&motion.request_id, status, code, message);
        }
        self.generation += 1;
        self.stopping = self.enabled
            && (self.velocity.iter().any(|v| v.abs() > 1e-12)
                || self.acceleration.iter().any(|a| a.abs() > 1e-12));
        self.phase();
        self.dirty = true;
    }
    fn fault(&mut self, code: &str, message: impl Into<String>) {
        let message = message.into();
        let changed = self
            .error
            .as_ref()
            .is_none_or(|e| e.code != code || e.message != message);
        if self.motion.is_some() || self.enable_request.is_some() {
            self.end_motion("failed", code, &message);
        } else if changed {
            self.generation += 1;
        }
        self.enabled = false;
        self.reset_derivatives();
        self.stopping = false;
        self.error = Some(ApiError::new(code, message.clone()));
        self.phase = Phase::Fault;
        if changed {
            self.journal.log("error", "device", code, &message, None);
            self.dirty = true;
        }
    }
    fn measured(&self, now: Instant) -> Result<[f64; 7], ApiError> {
        let mut q = [0.0; 7];
        let mut oldest = now;
        let mut newest = now - Duration::from_secs(10);
        for (i, sample) in self.feedback.iter().enumerate() {
            let sample = sample
                .as_ref()
                .ok_or_else(|| ApiError::new("feedback_missing", "关节反馈不完整"))?;
            if now.saturating_duration_since(sample.at).as_millis() > FEEDBACK_MS as u128 {
                return Err(ApiError::new("feedback_stale", "关节反馈过期"));
            }
            if !sample.value.q.is_finite() {
                return Err(ApiError::new("invalid_feedback", "关节反馈无效"));
            }
            oldest = oldest.min(sample.at);
            newest = newest.max(sample.at);
            q[i] = sample.value.q;
        }
        if newest.saturating_duration_since(oldest).as_millis() > FEEDBACK_SPAN_MS as u128 {
            return Err(ApiError::new("feedback_span", "关节反馈时间跨度超限"));
        }
        Ok(q)
    }
    fn validate_owner(&self, client: &str, secret: &str, now: Instant) -> Result<(), ApiError> {
        match &self.lease {
            Some(lease)
                if lease.client_id == client && lease.secret == secret && lease.expires > now =>
            {
                Ok(())
            }
            _ => Err(ApiError::new("not_owner", "未持有控制权")),
        }
    }
    fn expire_lease(&mut self, now: Instant) {
        if self
            .lease
            .as_ref()
            .is_some_and(|lease| now >= lease.expires)
        {
            self.end_motion("cancelled", "lease_expired", "控制权已过期");
            self.lease = None;
            self.journal
                .log("info", "control", "lease_expired", "控制权已过期", None);
        }
    }
    fn control(&mut self, request: ControlRequest, now: Instant) -> Result<ControlReply, ApiError> {
        if self.close_requested {
            return Err(ApiError::new("closing", "应用正在关闭"));
        }
        self.expire_lease(now);
        if request.client_id.is_empty() || request.client_id.len() > 64 {
            return Err(ApiError::new("invalid_client", "客户端标识无效"));
        }
        let secret = match request.action.as_str() {
            "acquire" => {
                if self.lease.is_some() {
                    return Err(ApiError::new("owned", "控制权已被占用"));
                }
                let secret = uuid::Uuid::new_v4().to_string();
                self.lease = Some(Lease {
                    client_id: request.client_id,
                    secret: secret.clone(),
                    expires: now + Duration::from_millis(LEASE_MS),
                });
                self.generation += 1;
                self.journal
                    .log("info", "control", "acquired", "控制权已取得", None);
                Some(secret)
            }
            "renew" => {
                self.validate_owner(
                    &request.client_id,
                    request.lease_id.as_deref().unwrap_or(""),
                    now,
                )?;
                self.lease.as_mut().unwrap().expires = now + Duration::from_millis(LEASE_MS);
                request.lease_id
            }
            "release" => {
                self.validate_owner(
                    &request.client_id,
                    request.lease_id.as_deref().unwrap_or(""),
                    now,
                )?;
                self.end_motion("cancelled", "released", "控制权已释放");
                self.lease = None;
                self.journal
                    .log("info", "control", "released", "控制权已释放", None);
                None
            }
            _ => return Err(ApiError::new("invalid_action", "控制权操作无效")),
        };
        self.dirty = true;
        Ok(ControlReply {
            lease_id: secret,
            expires_in_ms: if self.lease.is_some() { LEASE_MS } else { 0 },
            generation: self.generation,
        })
    }
    fn work(&mut self, work: Work, publisher: &watch::Sender<Snapshot>) {
        let now = Instant::now();
        self.expire_lease(now);
        match work {
            Work::PrepareClose(allow_release, deadline, reply) => {
                let result = if now > deadline {
                    Err(ApiError::new("timeout", "关闭请求已过期"))
                } else {
                    self.prepare_close(allow_release)
                };
                let _ = reply.send(result);
            }
            Work::Control(request, deadline, reply) => {
                let result = if now > deadline {
                    Err(ApiError::new("timeout", "控制请求已过期"))
                } else {
                    self.control(request, now)
                };
                if self.dirty {
                    self.seq += 1;
                    publisher.send_replace(self.snapshot(Instant::now()));
                    self.dirty = false;
                }
                let _ = reply.send(result);
            }
            Work::Command(request, received) => self.command(request, received, now),
        }
    }
    fn command(&mut self, request: CommandRequest, received: Instant, now: Instant) {
        let id = request.request_id.clone();
        if self.close_requested {
            self.record(&id, "rejected", "closing", "应用正在关闭");
            return;
        }
        if let Err(e) = self.validate_owner(&request.client_id, &request.lease_id, now) {
            self.record(&id, "rejected", &e.code, e.message);
            return;
        }
        if request.generation != self.generation {
            self.record(&id, "rejected", "stale_context", "操作上下文已改变");
            return;
        }
        if self
            .seen
            .iter()
            .any(|(generation, seen)| *generation == request.generation && seen == &id)
        {
            return;
        }
        self.seen.push_back((request.generation, id.clone()));
        if self.seen.len() > 256 {
            self.seen.pop_front();
        }
        let operation = request.operation.name();
        let outcome = self.apply(&id, request.operation, received, now);
        match outcome {
            Ok(()) => {
                if operation != "jog_update" {
                    self.record(&id, "accepted", operation, "操作已接受");
                }
            }
            Err(error) => self.record(&id, "rejected", &error.code, error.message),
        }
    }
    fn apply(
        &mut self,
        id: &str,
        operation: Operation,
        received: Instant,
        now: Instant,
    ) -> Result<(), ApiError> {
        match operation {
            Operation::Stop => self.end_motion("cancelled", "stopped", "动作已取消"),
            Operation::Connect { port, side } => {
                if self.bus.is_some() {
                    return Err(ApiError::new("connected", "请先断开当前连接"));
                }
                if side != "left" && side != "right" || port.is_empty() || port.len() > 300 {
                    return Err(ApiError::new("invalid_device", "设备配置无效"));
                }
                let model = Kinematics::load(&side).map_err(|e| ApiError::new("model_error", e))?;
                // The first FTDI reply after opening can arrive after the
                // adapter's latency timer; 20 ms also bounds a failed probe.
                let bus = BusHandle::open_recorded(
                    &port,
                    &side,
                    6_000_000,
                    Duration::from_millis(20),
                    self.journal.clone(),
                )
                .map_err(|e| ApiError::new("connect_failed", e))?;
                if self.side != side || self.tcp.is_none() {
                    // Link7 itself is a usable jog reference on a bare flange.
                    // A user-supplied tool transform replaces it explicitly.
                    self.tcp = Some(Pose {
                        translation: [0.0; 3],
                        rotation: [0.0, 0.0, 0.0, 1.0],
                    });
                }
                self.model = Some(model);
                self.side = side;
                self.port = Some(port);
                self.bus = Some(bus);
                self.feedback = std::array::from_fn(|_| None);
                self.enabled = false;
                self.error = None;
                self.motion = None;
                self.enable_request = None;
                self.reset_derivatives();
                self.connected_at = Some(now);
                self.phase = Phase::Connecting;
                self.generation += 1;
                self.journal
                    .log("info", "device", "connected", "串口连接已建立", Some(id));
            }
            Operation::Disconnect { allow_release } => {
                self.require_release(allow_release)?;
                self.disconnect();
            }
            Operation::Enable => {
                if self.bus.is_none() {
                    return Err(ApiError::new("not_connected", "设备未连接"));
                }
                if self.error.is_some() || self.clear_request.is_some() {
                    return Err(ApiError::new("fault", "请先清除当前故障"));
                }
                if self.enabled || self.enable_request.is_some() {
                    return Err(ApiError::new("enable_busy", "设备已使能或正在使能"));
                }
                if self.feedback.iter().any(|o| {
                    o.as_ref()
                        .is_some_and(|o| o.value.error != 0 || o.value.timeout)
                }) {
                    return Err(ApiError::new("fault", "请先清除设备故障"));
                }
                self.reference = self.measured(now)?;
                self.check_positions(&self.reference)?;
                self.reset_derivatives();
                self.enable_request = Some((id.into(), now));
                self.stopping = false;
                self.phase = Phase::Enabling;
                self.generation += 1;
            }
            Operation::Disable { allow_release } => {
                self.require_release(allow_release)?;
                self.end_motion("cancelled", "disabled", "设备已失能");
                self.enabled = false;
                self.reset_derivatives();
                self.stopping = false;
                self.phase();
            }
            Operation::ClearFault => {
                if self.enabled || self.enable_request.is_some() {
                    return Err(ApiError::new("disable_first", "请先失能再清错"));
                }
                if self.bus.is_none() {
                    return Err(ApiError::new("not_connected", "设备未连接"));
                }
                self.end_motion("cancelled", "clearing", "正在清除故障");
                self.enabled = false;
                self.stopping = false;
                self.clear_pending = [true; 7];
                self.clear_request = Some((id.into(), now));
            }
            Operation::ConfigureTcp { mut tcp } => {
                if self.motion.is_some() || self.stopping || self.enable_request.is_some() {
                    return Err(ApiError::new("motion_busy", "当前动作尚未结束"));
                }
                let norm = tcp.rotation.iter().map(|q| q * q).sum::<f64>().sqrt();
                if tcp
                    .translation
                    .iter()
                    .any(|q| !q.is_finite() || q.abs() > 2.0)
                    || !norm.is_finite()
                    || norm < 1e-9
                {
                    return Err(ApiError::new("invalid_tcp", "TCP 参数无效"));
                }
                tcp.rotation.iter_mut().for_each(|q| *q /= norm);
                self.tcp = Some(tcp);
                self.generation += 1;
            }
            Operation::JogStart {
                space,
                axis,
                direction,
                speed,
                frame,
                ttl_ms,
                input_seq,
            } => {
                let deadline = input_deadline(received, ttl_ms, now)?;
                self.begin(
                    id,
                    space,
                    axis,
                    direction,
                    speed,
                    frame,
                    false,
                    0.0,
                    Some(deadline),
                    input_seq,
                    now,
                )?;
            }
            Operation::Step {
                space,
                axis,
                direction,
                amount,
                speed,
                frame,
            } => {
                if !amount.is_finite()
                    || amount <= 0.0
                    || amount
                        > if space == Space::Cartesian && axis < 3 {
                            0.05
                        } else {
                            0.35
                        }
                {
                    return Err(ApiError::new("invalid_step", "步长超出局部范围"));
                }
                self.begin(
                    id, space, axis, direction, speed, frame, true, amount, None, 0, now,
                )?;
            }
            Operation::ReturnToPose {
                joints,
                speed,
                disconnect_after,
            } => self.begin_return(id, joints, speed, disconnect_after, now)?,
            Operation::JogUpdate {
                action_id,
                input_seq,
                ttl_ms,
            } => {
                let deadline = input_deadline(received, ttl_ms, now)?;
                let motion = self
                    .motion
                    .as_mut()
                    .ok_or_else(|| ApiError::new("no_action", "连续动作已结束"))?;
                if motion.mode.timed()
                    || motion.request_id != action_id
                    || input_seq <= motion.input_seq
                    || motion.deadline.is_some_and(|expires| now >= expires)
                {
                    return Err(ApiError::new("stale_input", "连续输入已失效"));
                }
                motion.deadline = Some(deadline);
                motion.input_seq = input_seq;
            }
        }
        self.dirty = true;
        Ok(())
    }
    fn require_release(&self, allow_release: bool) -> Result<(), ApiError> {
        if !allow_release && (self.enabled || self.enable_request.is_some()) {
            return Err(ApiError::new(
                "support_required",
                "请先收纳或确认机械臂已支撑",
            ));
        }
        Ok(())
    }
    fn prepare_close(&mut self, allow_release: bool) -> Result<(), ApiError> {
        self.require_release(allow_release)?;
        // Subsequent queued enable/motion requests must not pass this decision.
        self.close_requested = true;
        Ok(())
    }
    fn disconnect(&mut self) {
        self.end_motion("cancelled", "disconnected", "设备连接已断开");
        self.enabled = false;
        self.reset_derivatives();
        self.stopping = false;
        self.send_disable();
        self.bus = None;
        self.port = None;
        self.feedback = std::array::from_fn(|_| None);
        self.error = None;
        self.clear_request = None;
        self.clear_pending = [false; 7];
        self.phase = Phase::Disconnected;
    }
    fn motion_anchor(&self, now: Instant) -> Result<[f64; 7], ApiError> {
        if self.close_requested {
            return Err(ApiError::new("closing", "应用正在关闭"));
        }
        if !self.enabled || self.error.is_some() {
            return Err(ApiError::new("not_ready", "设备尚未就绪"));
        }
        if self.motion.is_some() || self.stopping {
            return Err(ApiError::new("motion_busy", "当前动作尚未结束"));
        }
        let measured = self.measured(now)?;
        self.check_positions(&measured)?;
        let preserve_reference = self.reference.iter().all(|value| value.is_finite())
            && (0..7).all(|i| (self.reference[i] - measured[i]).abs() <= MAX_TRACKING_ERROR);
        Ok(if preserve_reference {
            self.reference
        } else {
            measured
        })
    }
    fn begin_return(
        &mut self,
        id: &str,
        target: [f64; 7],
        speed: f64,
        disconnect_after: bool,
        now: Instant,
    ) -> Result<(), ApiError> {
        let anchor = self.motion_anchor(now)?;
        if !speed.is_finite() || speed <= 0.0 || speed > JOINT_SPEED {
            return Err(ApiError::new("speed_limit", "收纳速度超出当前限制"));
        }
        if target.iter().any(|q| !q.is_finite()) {
            return Err(ApiError::new("invalid_pose", "收纳位参数无效"));
        }
        let distance = (0..7)
            .map(|i| (target[i] - anchor[i]).abs())
            .fold(0.0, f64::max);
        if distance > std::f64::consts::TAU {
            return Err(ApiError::new("invalid_pose", "收纳位与当前位置相差过大"));
        }
        // Quintic interpolation: zero velocity and acceleration at both ends.
        // Peak |ds/dt|=1.875/T, peak |d²s/dt²|=10/sqrt(3)/T².
        let duration = (1.875 * distance / speed)
            .max((10.0 / 3.0_f64.sqrt() * distance / JOINT_ACCELERATION).sqrt())
            .max(0.2);
        if !duration.is_finite() {
            return Err(ApiError::new("invalid_pose", "收纳轨迹时长无效"));
        }
        self.reference = anchor;
        self.reset_derivatives();
        self.motion = Some(Motion {
            request_id: id.into(),
            space: Space::Joint,
            axis: 0,
            direction: 0.0,
            speed,
            frame: Frame::Base,
            mode: MotionMode::Return { disconnect_after },
            amount: distance,
            started: now,
            duration,
            initial: anchor,
            target,
            initial_pose: None,
            target_pose: None,
            deadline: None,
            input_seq: 0,
            settled_since: None,
        });
        self.phase = Phase::Moving;
        Ok(())
    }
    #[allow(clippy::too_many_arguments)]
    fn begin(
        &mut self,
        id: &str,
        space: Space,
        axis: usize,
        direction: i8,
        speed: f64,
        frame: Frame,
        step: bool,
        amount: f64,
        deadline: Option<Instant>,
        input_seq: u64,
        now: Instant,
    ) -> Result<(), ApiError> {
        let anchor = self.motion_anchor(now)?;
        if axis >= if space == Space::Joint { 7 } else { 6 }
            || ![-1, 1].contains(&direction)
            || !speed.is_finite()
            || speed <= 0.0
        {
            return Err(ApiError::new("invalid_motion", "点动参数无效"));
        }
        let limit = if space == Space::Joint {
            JOINT_SPEED
        } else if axis < 3 {
            LINEAR_SPEED
        } else {
            ANGULAR_SPEED
        };
        if speed > limit {
            return Err(ApiError::new("speed_limit", "速度超出当前限制"));
        }
        self.reference = anchor;
        self.reset_derivatives();
        let mut target = anchor;
        let mut initial_pose = None;
        let mut target_pose = None;
        if space == Space::Joint {
            if step {
                target[axis] += direction as f64 * amount;
                self.check_positions(&target)?;
            }
        } else {
            let tcp = self
                .tcp
                .as_ref()
                .ok_or_else(|| ApiError::new("tcp_missing", "TCP 未配置"))?;
            let pose = self
                .model
                .as_ref()
                .unwrap()
                .fk(&anchor, tcp)
                .map_err(|e| ApiError::new("kinematics", e))?;
            if step {
                let mut delta = [0.0; 6];
                delta[axis] = direction as f64 * amount;
                target_pose = Some(
                    kinematics::offset_pose(&pose, delta, frame)
                        .map_err(|e| ApiError::new("kinematics", e))?,
                );
            }
            initial_pose = Some(pose);
        }
        let duration = if step {
            let duration = (1.5 * amount / speed).max(0.10);
            if space == Space::Joint {
                duration.max((6.0 * amount / JOINT_ACCELERATION).sqrt())
            } else {
                duration
            }
        } else {
            0.0
        };
        self.motion = Some(Motion {
            request_id: id.into(),
            space,
            axis,
            direction: direction as f64,
            speed,
            frame,
            mode: if step {
                MotionMode::Step
            } else {
                MotionMode::Continuous
            },
            amount,
            started: now,
            duration,
            initial: anchor,
            target,
            initial_pose,
            target_pose,
            deadline,
            input_seq,
            settled_since: None,
        });
        self.phase = Phase::Moving;
        Ok(())
    }
    fn check_positions(&self, q: &[f64; 7]) -> Result<(), ApiError> {
        // Absolute travel is bounded by the arm's physical stops. URDF limits
        // describe the nominal model and need not match the installed zero.
        if q.iter().any(|value| !value.is_finite()) {
            return Err(ApiError::new("invalid_feedback", "关节位置无效"));
        }
        Ok(())
    }
    fn reset_derivatives(&mut self) {
        self.velocity = [0.0; 7];
        self.acceleration = [0.0; 7];
        self.jerk = [0.0; 7];
        self.stop_path = None;
    }
    fn motion_path(motion: &Motion) -> Option<CartesianPath> {
        (motion.space == Space::Cartesian).then(|| CartesianPath {
            origin: motion.initial_pose.unwrap(),
            axis: motion.axis,
            frame: motion.frame,
        })
    }
    // Constrain the controller velocity before integrating its reference once.
    // The serial sender may coalesce updates; capture its timing separately.
    // Absolute-time waypoints alone do not bound derivatives under scheduler jitter.
    fn commit_joint_velocity(&mut self, desired: [f64; 7], dt: f64) -> Result<(), ApiError> {
        let window = trajectory::velocity_window(
            self.velocity,
            self.acceleration,
            desired,
            [JOINT_SPEED; 7],
            [JOINT_ACCELERATION; 7],
            [JOINT_JERK; 7],
            dt,
        )
        .map_err(|e| ApiError::new("joint_dynamics", e))?;
        let velocity = std::array::from_fn(|i| desired[i].clamp(window.low[i], window.high[i]));
        let (reference, acceleration) =
            trajectory::integrate(self.reference, self.velocity, velocity, dt);
        self.check_positions(&reference)?;
        self.jerk = std::array::from_fn(|i| (acceleration[i] - self.acceleration[i]) / dt);
        self.reference = reference;
        self.velocity = velocity;
        self.acceleration = acceleration;
        Ok(())
    }
    fn commit_cartesian(
        &mut self,
        target: LocalTarget,
        path: Option<CartesianPath>,
        measured: [f64; 7],
        dt: f64,
        stopping: bool,
    ) -> Result<(), String> {
        let candidate = self
            .model
            .as_ref()
            .ok_or("missing model")?
            .step(StepInput {
                reference: self.reference,
                measured,
                previous_velocity: self.velocity,
                target,
                dt,
                tcp: self.tcp.ok_or("missing TCP")?,
                limits: MotionLimits {
                    joint_velocity: [JOINT_SPEED; 7],
                    joint_acceleration: [JOINT_ACCELERATION; 7],
                    max_reference_error: MAX_TRACKING_ERROR,
                    max_joint_step: 0.01,
                    linear_speed: LINEAR_SPEED,
                    angular_speed: ANGULAR_SPEED,
                },
                smoothing: Some(SmoothingState {
                    acceleration: self.acceleration,
                    jerk: [JOINT_JERK; 7],
                    path,
                    stopping,
                }),
            })?;
        self.jerk = std::array::from_fn(|i| (candidate.ddq[i] - self.acceleration[i]) / dt);
        self.reference = candidate.q;
        self.velocity = candidate.dq;
        self.acceleration = candidate.ddq;
        Ok(())
    }
    fn decelerate(&mut self, measured: [f64; 7], dt: f64) -> Result<(), ApiError> {
        let near_rest = self.velocity.iter().all(|v| v.abs() < 1e-6)
            && self.acceleration.iter().all(|a| a.abs() < 1e-4);
        if let Some(path) = self.stop_path
            && !near_rest
        {
            match self.commit_cartesian(
                LocalTarget::Velocity {
                    twist: [0.0; 6],
                    frame: Frame::Base,
                },
                Some(path),
                measured,
                dt,
                true,
            ) {
                Ok(()) => return Ok(()),
                Err(error) => {
                    // Stopping remains possible if the selected Cartesian path
                    // becomes infeasible. Do not keep an old motion alive.
                    self.journal
                        .log("warn", "control", "stop_path", &error, None);
                    self.stop_path = None;
                }
            }
        }
        self.commit_joint_velocity([0.0; 7], dt)
    }
    fn at_rest(&self) -> bool {
        self.velocity.iter().all(|v| v.abs() < 1e-12)
            && self.acceleration.iter().all(|a| a.abs() < 1e-12)
    }
    fn advance(&mut self, now: Instant, dt: f64) {
        self.control_seq = self.control_seq.wrapping_add(1);
        self.control_at = now;
        self.jerk = [0.0; 7];
        self.settle_transitions(now);
        if !self.enabled {
            return;
        }
        let measured = match self.measured(now) {
            Ok(q) => q,
            Err(e) => {
                self.fault(&e.code, e.message);
                return;
            }
        };
        if dt > 0.05 || !dt.is_finite() {
            self.fault("control_timeout", "控制周期超时");
            return;
        }
        if self
            .motion
            .as_ref()
            .is_some_and(|motion| motion.space == Space::Cartesian)
            && (0..7).any(|i| (self.reference[i] - measured[i]).abs() > MAX_TRACKING_ERROR)
        {
            self.fault("following_error", "关节跟随偏差超限");
            return;
        }
        if self
            .motion
            .as_ref()
            .is_some_and(|motion| matches!(motion.mode, MotionMode::Return { .. }))
            && (0..7).any(|i| (self.reference[i] - measured[i]).abs() > MAX_TRACKING_ERROR)
        {
            self.end_motion("failed", "following_error", "收纳跟随偏差超限，已停止返回");
        }
        if self
            .motion
            .as_ref()
            .is_some_and(|motion| motion.deadline.is_some_and(|deadline| now >= deadline))
        {
            self.end_motion("cancelled", "input_expired", "连续输入已过期");
        }
        if self.stopping {
            if let Err(e) = self.decelerate(measured, dt) {
                self.fault(&e.code, e.message);
                return;
            }
            if self.at_rest() {
                self.stopping = false;
                self.stop_path = None;
                self.phase();
                self.dirty = true;
            }
            return;
        }
        let Some(motion) = self.motion.as_ref() else {
            return;
        };
        let elapsed = now.saturating_duration_since(motion.started).as_secs_f64();
        let mode = motion.mode;
        if matches!(mode, MotionMode::Return { .. }) && elapsed > motion.duration + 3.0 {
            self.end_motion("failed", "target_timeout", "收纳到位超时，已停止返回");
            if let Err(e) = self.decelerate(measured, dt) {
                self.fault(&e.code, e.message);
            }
            return;
        }
        let t = if mode.timed() {
            (elapsed / motion.duration).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let smooth = if matches!(mode, MotionMode::Return { .. }) {
            t * t * t * (10.0 + t * (-15.0 + 6.0 * t))
        } else {
            t * t * (3.0 - 2.0 * t)
        };
        let within_target = |q: &[f64; 7]| {
            if motion.space == Space::Joint {
                (0..7).all(|i| (q[i] - motion.target[i]).abs() <= 0.01)
            } else {
                self.model
                    .as_ref()
                    .unwrap()
                    .fk(q, self.tcp.as_ref().unwrap())
                    .is_ok_and(|current| {
                        let error =
                            kinematics::pose_error(&current, motion.target_pose.as_ref().unwrap());
                        error[..3].iter().map(|v| v * v).sum::<f64>().sqrt() <= 0.001
                            && error[3..].iter().map(|v| v * v).sum::<f64>().sqrt()
                                <= std::f64::consts::PI / 180.0
                    })
            }
        };
        let at_target =
            mode.timed() && t >= 1.0 && within_target(&measured) && within_target(&self.reference);
        if at_target {
            self.stop_path = Self::motion_path(motion);
            if let Err(e) = self.decelerate(measured, dt) {
                self.fault(&e.code, e.message);
                return;
            }
            if self.at_rest() {
                if let MotionMode::Return { disconnect_after } = mode {
                    let slow = self.feedback.iter().all(|sample| {
                        sample.as_ref().is_some_and(|sample| {
                            sample.value.dq.is_finite() && sample.value.dq.abs() <= 0.02
                        })
                    });
                    let motion = self.motion.as_mut().unwrap();
                    if !slow {
                        motion.settled_since = None;
                        return;
                    }
                    let since = *motion.settled_since.get_or_insert(now);
                    let observed_until = self
                        .feedback
                        .iter()
                        .flatten()
                        .map(|sample| sample.at)
                        .min()
                        .unwrap();
                    if now.saturating_duration_since(since) < RETURN_SETTLE_TIME
                        || observed_until.saturating_duration_since(since) < RETURN_SETTLE_TIME
                    {
                        return;
                    }
                    let id = self.motion.take().unwrap().request_id;
                    if disconnect_after {
                        self.disconnect();
                        self.record(&id, "completed", "parked", "收纳到位，连接已断开");
                    } else {
                        self.generation += 1;
                        self.stop_path = None;
                        self.record(&id, "completed", "returned", "已返回收纳位");
                        self.phase();
                    }
                    return;
                }
                let id = self.motion.take().unwrap().request_id;
                self.stop_path = None;
                self.generation += 1;
                self.record(&id, "completed", "completed", "动作已完成");
                self.phase();
            }
            return;
        }
        if matches!(mode, MotionMode::Return { .. }) {
            self.motion.as_mut().unwrap().settled_since = None;
        }
        let motion = self.motion.as_ref().unwrap();
        if motion.space == Space::Joint {
            let desired_velocity = if motion.mode.timed() {
                let next: [f64; 7] = std::array::from_fn(|i| {
                    motion.initial[i] + (motion.target[i] - motion.initial[i]) * smooth
                });
                std::array::from_fn(|i| {
                    ((next[i] - self.reference[i]) / dt).clamp(-motion.speed, motion.speed)
                })
            } else {
                let i = motion.axis;
                let mut desired = [0.0; 7];
                desired[i] = motion.direction * motion.speed;
                desired
            };
            if let Err(e) = self.commit_joint_velocity(desired_velocity, dt) {
                self.fault(&e.code, e.message);
                return;
            }
        } else {
            let target = if motion.mode.timed() {
                let mut delta = [0.0; 6];
                delta[motion.axis] = motion.direction * motion.amount * smooth;
                match kinematics::offset_pose(
                    motion.initial_pose.as_ref().unwrap(),
                    delta,
                    motion.frame,
                ) {
                    Ok(target) => LocalTarget::Pose(target),
                    Err(e) => {
                        self.end_motion("failed", "kinematics", &e);
                        if let Err(e) = self.decelerate(measured, dt) {
                            self.fault(&e.code, e.message);
                        }
                        return;
                    }
                }
            } else {
                let mut twist = [0.0; 6];
                twist[motion.axis] = motion.direction * motion.speed;
                LocalTarget::Velocity {
                    twist,
                    frame: motion.frame,
                }
            };
            match self.commit_cartesian(target, Self::motion_path(motion), measured, dt, false) {
                Ok(()) => {}
                Err(e) => {
                    self.end_motion("failed", "kinematics", &e);
                    if let Err(e) = self.decelerate(measured, dt) {
                        self.fault(&e.code, e.message);
                    }
                    return;
                }
            }
        }
        if self
            .motion
            .as_ref()
            .is_some_and(|m| m.mode.timed() && elapsed > m.duration + 3.0)
        {
            self.end_motion("failed", "target_timeout", "目标到达超时");
        }
    }
    fn poll(&mut self, now: Instant) {
        let servo = self.enabled || self.enable_request.is_some();
        let gravity = if servo {
            self.model
                .as_ref()
                .and_then(|model| model.gravity(&self.reference).ok())
                .unwrap_or([0.0; 7])
        } else {
            [0.0; 7]
        };
        let setpoints: [Setpoint; 7] = std::array::from_fn(|i| Setpoint {
            mode: if self.clear_pending[i] {
                DriveMode::ClearFault
            } else if servo {
                DriveMode::Servo
            } else {
                DriveMode::Disabled
            },
            q: self.reference[i],
            dq: self.velocity[i],
            tau: if servo && !self.clear_pending[i] {
                gravity[i].clamp(-TAU_FF_LIMIT, TAU_FF_LIMIT)
            } else {
                0.0
            },
            kp: if servo { SERVO_KP } else { 0.0 },
            kd: if servo { SERVO_KD } else { 0.0 },
        });
        let clearing = self.clear_pending.iter().any(|pending| *pending);
        let motion = self.motion.as_ref();
        self.journal.sample(
            self.control_at,
            TraceData::Control {
                seq: self.control_seq,
                generation: self.generation,
                enabled: self.enabled,
                stopping: self.stopping,
                motion: motion.map_or(0, |m| match (m.space, m.mode) {
                    (_, MotionMode::Return { .. }) => 5,
                    (Space::Joint, MotionMode::Continuous) => 1,
                    (Space::Joint, MotionMode::Step) => 2,
                    (Space::Cartesian, MotionMode::Continuous) => 3,
                    (Space::Cartesian, MotionMode::Step) => 4,
                }),
                axis: motion
                    .filter(|m| !matches!(m.mode, MotionMode::Return { .. }))
                    .map(|m| m.axis as u8),
                q_ref: self.reference,
                dq_ref: self.velocity,
                ddq_ref: self.acceleration,
                jerk_ref: self.jerk,
                tau_ff: std::array::from_fn(|i| setpoints[i].tau),
                q: std::array::from_fn(|i| self.feedback[i].as_ref().map(|s| s.value.q)),
                dq: std::array::from_fn(|i| self.feedback[i].as_ref().map(|s| s.value.dq)),
                tau: std::array::from_fn(|i| self.feedback[i].as_ref().map(|s| s.value.tau)),
                voltage: std::array::from_fn(|i| {
                    self.feedback[i].as_ref().map(|s| s.value.voltage)
                }),
                age_us: std::array::from_fn(|i| {
                    self.feedback[i].as_ref().map(|s| {
                        self.control_at
                            .saturating_duration_since(s.at)
                            .as_micros()
                            .min(u64::MAX as u128) as u64
                    })
                }),
            },
        );
        let Some(bus) = self.bus.as_ref() else {
            return;
        };
        let clear_result = if clearing {
            bus.write_blocking(setpoints)
        } else {
            Ok(())
        };
        let send_result = bus.send_with_sequence(setpoints, self.control_seq);
        let (values, read_error) = bus.drain();
        let stats = bus.stats();
        if let Err(e) = clear_result {
            self.fault("communication_error", e);
            return;
        }
        if clearing {
            self.clear_pending = [false; 7];
        }
        if let Err(e) = send_result {
            self.fault("communication_error", e);
            return;
        }
        self.communication = Communication {
            received: stats.received,
            crc_errors: stats.crc_errors,
            timeouts: stats.timeouts,
        };
        for (value, at) in values {
            if self.error.is_some() {
                break;
            }
            self.accept_feedback(value, at);
        }
        if let Some(e) = read_error {
            self.fault("communication_error", e);
        }
        if self.phase == Phase::Connecting && self.measured(Instant::now()).is_ok() {
            self.phase = Phase::Disabled;
            self.dirty = true;
        }
        if self.phase == Phase::Connecting
            && self
                .connected_at
                .is_some_and(|at| now.saturating_duration_since(at) > Duration::from_secs(1))
        {
            self.fault("feedback_timeout", "设备反馈超时");
        }
        self.settle_transitions(Instant::now());
    }
    fn diagnose(&mut self, now: Instant, dt_ms: f64, work_ms: f64) {
        if !self.enabled && self.motion.is_none() {
            if let Some(bus) = &self.bus {
                bus.take_transmit_stats();
            }
            self.diag_at = now;
            self.diag_cycles = 0;
            self.diag_dt_sum = 0.0;
            self.diag_dt_max = 0.0;
            self.diag_work_sum = 0.0;
            self.diag_work_max = 0.0;
            return;
        }
        self.diag_cycles += 1;
        self.diag_dt_sum += dt_ms;
        self.diag_dt_max = self.diag_dt_max.max(dt_ms);
        self.diag_work_sum += work_ms;
        self.diag_work_max = self.diag_work_max.max(work_ms);
        if now.saturating_duration_since(self.diag_at) < Duration::from_secs(1) {
            return;
        }
        let cycles = self.diag_cycles.max(1) as f64;
        let age_ms = self
            .feedback
            .iter()
            .filter_map(|sample| {
                sample
                    .as_ref()
                    .map(|sample| now.saturating_duration_since(sample.at).as_millis())
            })
            .max()
            .unwrap_or(0);
        let (joint, error) = self
            .feedback
            .iter()
            .enumerate()
            .filter_map(|(i, sample)| {
                sample
                    .as_ref()
                    .map(|sample| (i, (self.reference[i] - sample.value.q).abs()))
            })
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap_or((0, 0.0));
        let (q, dq, tau, voltage) = self
            .feedback
            .get(joint)
            .and_then(Option::as_ref)
            .map(|sample| {
                (
                    sample.value.q,
                    sample.value.dq,
                    sample.value.tau,
                    sample.value.voltage,
                )
            })
            .unwrap_or((0.0, 0.0, 0.0, 0.0));
        let transmit = self
            .bus
            .as_ref()
            .map(|bus| bus.take_transmit_stats())
            .unwrap_or_default();
        let gravity = self
            .model
            .as_ref()
            .and_then(|model| model.gravity(&self.reference).ok())
            .unwrap_or([0.0; 7]);
        let motion = self
            .motion
            .as_ref()
            .map(|motion| {
                if matches!(motion.mode, MotionMode::Return { .. }) {
                    return "return joint".into();
                }
                format!(
                    "{}{}J{}",
                    if motion.mode.timed() {
                        "step "
                    } else {
                        "continuous "
                    },
                    if motion.space == Space::Joint {
                        "joint "
                    } else {
                        "cartesian "
                    },
                    motion.axis + 1
                )
            })
            .unwrap_or_else(|| "-".into());
        let message = format!(
            "周期 {:.2}/{:.2} ms，控制线程 {:.2}/{:.2} ms，action={motion}，反馈年龄 {age_ms} ms，J{} 参考误差 {:.3} rad，qref={:.3} q={:.3} dq={:.3} tau={:.2}，comm={}/{}/{}，tx批次={} 间隔均/小/大={:.2}/{:.2}/{:.2} ms 写入最大={:.2} ms，dqcmd={:.4} tauff={:.3} voltage={:.2}",
            self.diag_dt_sum / cycles,
            self.diag_dt_max,
            self.diag_work_sum / cycles,
            self.diag_work_max,
            joint + 1,
            error,
            self.reference[joint],
            q,
            dq,
            tau,
            self.communication.received,
            self.communication.crc_errors,
            self.communication.timeouts,
            transmit.batches,
            transmit.interval_sum_ms / transmit.intervals.max(1) as f64,
            transmit.interval_min_ms,
            transmit.interval_max_ms,
            transmit.write_max_ms,
            self.velocity[joint],
            gravity[joint].clamp(-TAU_FF_LIMIT, TAU_FF_LIMIT),
            voltage,
        );
        self.journal.log("info", "control", "cycle", &message, None);
        self.diag_at = now;
        self.diag_cycles = 0;
        self.diag_dt_sum = 0.0;
        self.diag_dt_max = 0.0;
        self.diag_work_sum = 0.0;
        self.diag_work_max = 0.0;
    }
    fn accept_feedback(&mut self, value: Feedback, at: Instant) {
        let id = value.id as usize;
        if id >= 7 {
            return;
        }
        let error = value.error;
        let timeout = value.timeout;
        let mode = value.mode;
        if let Some(motion) = &mut self.motion
            && matches!(motion.mode, MotionMode::Return { .. })
            && (!value.q.is_finite()
                || !value.dq.is_finite()
                || (value.q - motion.target[id]).abs() > 0.01
                || value.dq.abs() > 0.02)
        {
            // Do not let a later frame in the same receive batch conceal
            // a position/velocity excursion during the settle interval.
            motion.settled_since = None;
        }
        self.feedback[id] = Some(Observation { value, at });
        if error != 0 && self.clear_request.is_none() {
            self.fault("device_error", format!("J{} 故障码 {}", id + 1, error));
        } else if timeout && (self.enabled || self.enable_request.is_some()) {
            self.fault("device_timeout", format!("J{} 设备通信超时", id + 1));
        } else if mode != 1 && self.enabled {
            self.fault("device_disabled", format!("J{} 已退出使能", id + 1));
        }
    }
    fn settle_transitions(&mut self, now: Instant) {
        if let Some((id, started)) = &self.enable_request {
            let confirmed = self.error.is_none()
                && self.measured(now).is_ok()
                && self.feedback.iter().all(|o| {
                    o.as_ref().is_some_and(|o| {
                        o.at > *started
                            && o.value.mode == 1
                            && o.value.error == 0
                            && !o.value.timeout
                    })
                });
            if confirmed {
                let id = id.clone();
                self.enable_request = None;
                self.enabled = true;
                self.phase = Phase::Ready;
                self.record(&id, "completed", "enabled", "七轴使能已确认");
            } else if now.saturating_duration_since(*started) >= ENABLE_TIMEOUT {
                self.fault("enable_timeout", "七轴使能确认超时");
            }
        }
        if let Some((id, started)) = &self.clear_request {
            if !self.clear_pending.iter().any(|pending| *pending)
                && self.measured(now).is_ok()
                && self.feedback.iter().all(|o| {
                    o.as_ref()
                        .is_some_and(|o| o.at > *started && o.value.error == 0 && !o.value.timeout)
                })
            {
                let id = id.clone();
                self.clear_request = None;
                self.clear_pending = [false; 7];
                self.error = None;
                self.phase();
                self.record(&id, "completed", "cleared", "故障已清除");
            } else if now.saturating_duration_since(*started) > Duration::from_secs(1) {
                let id = id.clone();
                self.clear_request = None;
                self.record(&id, "failed", "clear_failed", "故障未清除");
                self.fault("device_error", "故障未清除");
            }
        }
    }
    fn send_disable(&mut self) {
        if let Some(bus) = self.bus.as_ref() {
            let _ = bus.disable();
        }
    }
    fn snapshot(&self, now: Instant) -> Snapshot {
        let names = self
            .model
            .as_ref()
            .map(|model| model.joint_names())
            .unwrap_or_else(|| std::array::from_fn(|i| format!("J{}", i + 1)));
        let joints = (0..7)
            .map(|i| {
                let value = self.feedback[i].as_ref();
                JointSnapshot {
                    id: i,
                    name: names[i].clone(),
                    q: value.map(|o| o.value.q),
                    dq: value.map(|o| o.value.dq),
                    tau: value.map(|o| o.value.tau),
                    age_ms: value.map(|o| now.saturating_duration_since(o.at).as_millis() as u64),
                    temperature: value.map(|o| o.value.temperature),
                    voltage: value.map(|o| o.value.voltage),
                    error: value.map(|o| o.value.error),
                }
            })
            .collect();
        let tcp = self.tcp.as_ref().and_then(|tcp| {
            self.measured(now)
                .ok()
                .and_then(|q| self.model.as_ref()?.fk(&q, tcp).ok())
        });
        let feedback_age_ms = self
            .feedback
            .iter()
            .filter_map(|o| {
                o.as_ref()
                    .map(|o| now.saturating_duration_since(o.at).as_millis() as u64)
            })
            .max();
        Snapshot {
            seq: self.seq,
            generation: self.generation,
            phase: self.phase,
            side: self.side.clone(),
            port: self.port.clone(),
            enabled: self.enabled,
            owner: self.lease.as_ref().map(|lease| OwnerSnapshot {
                client_id: lease.client_id.clone(),
                expires_in_ms: lease.expires.saturating_duration_since(now).as_millis() as u64,
            }),
            joints,
            tcp,
            tcp_transform: self.tcp,
            tcp_configured: self.tcp.is_some(),
            feedback_age_ms,
            action: self.motion.as_ref().map(|motion| ActionSnapshot {
                request_id: motion.request_id.clone(),
                space: motion.space,
                mode: motion.mode.name().into(),
                axis: (!matches!(motion.mode, MotionMode::Return { .. })).then_some(motion.axis),
                progress: if motion.mode.timed() {
                    (now.saturating_duration_since(motion.started).as_secs_f64() / motion.duration)
                        .min(1.0)
                } else {
                    0.0
                },
            }),
            last_result: self.results.front().cloned(),
            results: self.results.iter().cloned().collect(),
            error: self.error.clone(),
            journal: self.journal.stats(),
            communication: Communication {
                received: self.communication.received,
                crc_errors: self.communication.crc_errors,
                timeouts: self.communication.timeouts,
            },
        }
    }
}

fn input_deadline(received: Instant, ttl_ms: u64, now: Instant) -> Result<Instant, ApiError> {
    if !(50..=500).contains(&ttl_ms) {
        return Err(ApiError::new("invalid_ttl", "输入有效期无效"));
    }
    let deadline = received + Duration::from_millis(ttl_ms);
    if now >= deadline {
        return Err(ApiError::new("stale_input", "连续输入已过期"));
    }
    Ok(deadline)
}
#[allow(clippy::too_many_arguments)]
fn run(
    mut core: Core,
    ordinary: Receiver<Work>,
    priority: Receiver<Work>,
    latest: Receiver<Work>,
    host_epoch: Arc<AtomicU64>,
    stop_reason: Arc<Mutex<Option<String>>>,
    shutdown: Arc<AtomicBool>,
    state: watch::Sender<Snapshot>,
) {
    let mut seen_epoch = 0;
    let mut last_control = Instant::now();
    let mut published = last_control;
    let mut next_control = last_control;
    loop {
        let now = Instant::now();
        let epoch = host_epoch.load(Ordering::Acquire);
        if epoch != seen_epoch {
            let reason = stop_reason
                .lock()
                .ok()
                .and_then(|mut reason| reason.take())
                .unwrap_or_else(|| "unspecified".into());
            core.journal.log(
                "info",
                "control",
                "host_stop",
                &format!("停止请求：{reason}"),
                None,
            );
            core.end_motion("cancelled", "host_stop", "宿主已取消当前动作");
            seen_epoch = epoch;
        }
        if shutdown.load(Ordering::Acquire) || core.close_requested {
            shutdown.store(true, Ordering::Release);
            core.end_motion("cancelled", "closing", "应用正在关闭");
            core.lease = None;
            core.enabled = false;
            core.send_disable();
            core.bus = None;
            core.phase = Phase::Closing;
            core.seq += 1;
            state.send_replace(core.snapshot(Instant::now()));
            break;
        }
        core.expire_lease(now);
        for _ in 0..32 {
            match priority.try_recv() {
                Ok(work) => core.work(work, &state),
                Err(_) => break,
            }
        }
        if let Ok(work) = latest.try_recv() {
            core.work(work, &state);
        }
        for _ in 0..8 {
            match ordinary.try_recv() {
                Ok(work) => core.work(work, &state),
                Err(_) => break,
            }
        }
        // Recheck after potentially slow connection operations, before committing references.
        if host_epoch.load(Ordering::Acquire) != seen_epoch
            || shutdown.load(Ordering::Acquire)
            || core.close_requested
        {
            continue;
        }
        let at = Instant::now();
        let dt = at
            .saturating_duration_since(last_control)
            .as_secs_f64()
            .max(0.0001);
        let work_at = Instant::now();
        last_control = at;
        core.expire_lease(at);
        core.advance(at, dt);
        if host_epoch.load(Ordering::Acquire) != seen_epoch
            || shutdown.load(Ordering::Acquire)
            || core.close_requested
        {
            continue;
        }
        core.expire_lease(Instant::now());
        core.poll(Instant::now());
        let work_ms = work_at.elapsed().as_secs_f64() * 1000.0;
        core.diagnose(at, dt * 1000.0, work_ms);
        if core.dirty || published.elapsed() >= Duration::from_millis(33) {
            core.seq += 1;
            state.send_replace(core.snapshot(Instant::now()));
            core.dirty = false;
            published = Instant::now();
        }
        next_control += CONTROL_PERIOD;
        let wait = next_control.saturating_duration_since(Instant::now());
        if wait.is_zero() {
            next_control = Instant::now();
        } else {
            std::thread::sleep(wait);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn core() -> (Core, tempfile::TempDir) {
        let directory = tempfile::tempdir().unwrap();
        let journal = Journal::start(directory.path().join("events.sqlite3")).unwrap();
        (Core::new(journal), directory)
    }
    fn observed_core(at: Instant) -> (Core, tempfile::TempDir) {
        let (mut core, directory) = core();
        let (lower, upper) = core.model.as_ref().unwrap().position_limits();
        for i in 0..7 {
            core.accept_feedback(
                Feedback {
                    id: i as u8,
                    q: (lower[i] + upper[i]) / 2.0,
                    dq: 0.0,
                    tau: 0.0,
                    temperature: 25.0,
                    voltage: 24.0,
                    error: 0,
                    warning: 0,
                    mode: 1,
                    timeout: false,
                },
                at,
            );
        }
        core.reference = core.measured(at).unwrap();
        (core, directory)
    }
    fn start_joint(core: &mut Core, id: &str, at: Instant, step: bool) {
        core.begin(
            id,
            Space::Joint,
            0,
            1,
            0.05,
            Frame::Base,
            step,
            0.01,
            (!step).then_some(at + Duration::from_millis(150)),
            1,
            at,
        )
        .unwrap();
    }
    fn follow_reference(core: &mut Core, at: Instant) {
        for i in 0..7 {
            let mut sample = core.feedback[i].as_ref().unwrap().value.clone();
            sample.q = core.reference[i];
            sample.dq = core.velocity[i];
            core.accept_feedback(sample, at);
        }
    }
    fn assert_integrated_dynamics(core: &Core, q: [f64; 7], dq: [f64; 7], dt: f64) {
        assert!(core.error.is_none());
        for i in 0..7 {
            assert!((core.velocity[i] - dq[i]).abs() <= JOINT_ACCELERATION * dt + 1e-10);
            assert!((core.reference[i] - q[i] - core.velocity[i] * dt).abs() < 1e-10);
            assert!(core.velocity[i].abs() <= JOINT_SPEED + 1e-10);
        }
    }
    #[test]
    fn return_pose_moves_all_joints_with_bounded_dynamics_before_optional_disconnect() {
        for disconnect_after in [false, true] {
            let mut now = Instant::now();
            let (mut core, _directory) = observed_core(now);
            core.enabled = true;
            core.port = Some("memory-only".into());
            let initial = core.reference;
            let delta = [0.14, 0.08, -0.12, 0.06, 0.02, -0.04, 0.07];
            let target = std::array::from_fn(|i| initial[i] + delta[i]);
            core.begin_return("return", target, 0.08, disconnect_after, now)
                .unwrap();
            let snapshot = core.snapshot(now);
            assert_eq!(snapshot.action.as_ref().unwrap().mode, "return");
            assert_eq!(snapshot.action.as_ref().unwrap().axis, None);
            for dt in [0.040, 0.002, 0.007, 0.003].into_iter().cycle().take(700) {
                now += Duration::from_secs_f64(dt);
                follow_reference(&mut core, now);
                let q = core.reference;
                let dq = core.velocity;
                core.advance(now, dt);
                assert_integrated_dynamics(&core, q, dq, dt);
                assert!(core.velocity.iter().all(|v| v.abs() <= 0.08 + 1e-10));
                if core.motion.is_none() {
                    break;
                }
                assert!(core.enabled);
                assert!(core.port.is_some());
            }
            assert!(core.motion.is_none());
            assert!((0..7).all(|i| (core.reference[i] - target[i]).abs() <= 0.01));
            assert!(core.velocity.iter().all(|v| v.abs() < 1e-12));
            assert_eq!(core.enabled, !disconnect_after);
            assert_eq!(core.port.is_none(), disconnect_after);
            let result = core.results.front().unwrap();
            assert_eq!(result.status, "completed");
            assert_eq!(
                result.code,
                if disconnect_after {
                    "parked"
                } else {
                    "returned"
                }
            );
        }
    }
    #[test]
    fn stopped_return_never_disconnects_later() {
        let mut now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        core.port = Some("memory-only".into());
        let mut target = core.reference;
        target[1] += 0.1;
        core.begin_return("return", target, 0.08, true, now)
            .unwrap();
        now += Duration::from_millis(10);
        follow_reference(&mut core, now);
        core.advance(now, 0.01);
        core.apply("stop", Operation::Stop, now, now).unwrap();
        for _ in 0..500 {
            now += Duration::from_millis(10);
            follow_reference(&mut core, now);
            core.advance(now, 0.01);
        }
        assert!(core.enabled);
        assert!(core.port.is_some());
        assert_eq!(core.results.front().unwrap().status, "cancelled");
        assert!(core.results.iter().all(|r| r.code != "parked"));
    }
    #[test]
    fn return_timeout_keeps_connection_and_never_reports_parked() {
        let mut now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        core.port = Some("memory-only".into());
        let mut target = core.reference;
        target[1] += 0.1;
        core.begin_return("return", target, 0.1, true, now).unwrap();
        // Fresh feedback is stationary: elapsed time alone is not arrival.
        for _ in 0..600 {
            now += Duration::from_millis(10);
            for sample in core.feedback.iter_mut().flatten() {
                sample.at = now;
            }
            core.advance(now, 0.01);
            if core.motion.is_none() {
                break;
            }
        }
        assert!(core.motion.is_none());
        assert!(core.enabled);
        assert!(core.port.is_some());
        assert_eq!(core.results.front().unwrap().code, "target_timeout");
        assert!(core.results.iter().all(|r| r.code != "parked"));
    }
    #[test]
    fn return_requires_a_fresh_uninterrupted_stationary_settle_window() {
        let mut now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        core.port = Some("memory-only".into());
        core.begin_return("return", core.reference, 0.1, true, now)
            .unwrap();
        for tick in 1..=50 {
            now += Duration::from_millis(10);
            follow_reference(&mut core, now);
            if tick <= 25 {
                core.feedback[0].as_mut().unwrap().value.dq = 0.1;
            }
            // A new position excursion resets the earlier settle interval.
            if tick == 40 {
                core.feedback[0].as_mut().unwrap().value.q += 0.02;
            }
            core.advance(now, 0.01);
            assert!(core.motion.is_some());
            assert!(core.enabled);
        }
        for _ in 0..12 {
            now += Duration::from_millis(10);
            follow_reference(&mut core, now);
            core.advance(now, 0.01);
            if core.motion.is_none() {
                break;
            }
        }
        assert!(!core.enabled);
        assert!(core.port.is_none());
        assert_eq!(core.results.front().unwrap().code, "parked");
    }
    #[test]
    fn release_without_confirmation_preserves_holding_and_current_motion() {
        for enabling in [false, true] {
            let now = Instant::now();
            let (mut core, _directory) = observed_core(now);
            core.port = Some("memory-only".into());
            core.enabled = true;
            start_joint(&mut core, "joint", now, true);
            if enabling {
                core.enabled = false;
                core.enable_request = Some(("enable".into(), now));
            }
            for operation in [
                Operation::Disconnect {
                    allow_release: false,
                },
                Operation::Disable {
                    allow_release: false,
                },
            ] {
                let generation = core.generation;
                assert_eq!(
                    core.apply("release", operation, now, now).unwrap_err().code,
                    "support_required"
                );
                assert_eq!(core.generation, generation);
                assert_eq!(core.enabled, !enabling);
                assert!(core.port.is_some());
                assert!(core.motion.is_some());
            }
            assert_eq!(
                core.prepare_close(false).unwrap_err().code,
                "support_required"
            );
            assert!(!core.close_requested);
            assert_eq!(
                core.apply("clear", Operation::ClearFault, now, now)
                    .unwrap_err()
                    .code,
                "disable_first"
            );
            core.prepare_close(true).unwrap();
            assert!(core.close_requested);
        }
    }
    #[test]
    fn accepted_close_rejects_already_queued_enable_requests() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.prepare_close(false).unwrap();
        core.command(
            CommandRequest {
                request_id: "enable".into(),
                client_id: "desktop".into(),
                lease_id: "unused".into(),
                generation: core.generation,
                operation: Operation::Enable,
            },
            now,
            now,
        );
        assert!(!core.enabled);
        assert!(core.enable_request.is_none());
        assert_eq!(core.results.front().unwrap().code, "closing");
    }
    #[tokio::test]
    async fn approved_close_publishes_terminal_state_even_if_reply_is_gone() {
        let directory = tempfile::tempdir().unwrap();
        let journal = Journal::start(directory.path().join("events.sqlite3")).unwrap();
        let arm = ArmHandle::spawn(journal.clone()).unwrap();
        let mut states = arm.subscribe();
        let (reply, response) = oneshot::channel();
        drop(response);
        arm.priority
            .try_send(Work::PrepareClose(
                false,
                Instant::now() + Duration::from_millis(300),
                reply,
            ))
            .unwrap();
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if states.borrow().phase == Phase::Closing {
                    break;
                }
                states.changed().await.unwrap();
            }
        })
        .await
        .unwrap();
        assert!(arm.is_closing());
        assert!(arm.shutdown(Duration::from_millis(100)));
        journal.shutdown(Duration::from_millis(200));
    }
    #[test]
    fn return_cannot_wait_forever_on_nonzero_measured_velocity() {
        let mut now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        core.port = Some("memory-only".into());
        core.begin_return("return", core.reference, 0.1, true, now)
            .unwrap();
        for _ in 0..400 {
            now += Duration::from_millis(10);
            for sample in core.feedback.iter_mut().flatten() {
                sample.at = now;
                sample.value.dq = 0.1;
            }
            core.advance(now, 0.01);
            if core.motion.is_none() {
                break;
            }
        }
        assert!(core.motion.is_none());
        assert!(core.enabled);
        assert!(core.port.is_some());
        assert_eq!(core.results.front().unwrap().code, "target_timeout");
    }
    #[test]
    fn return_settle_observes_every_feedback_even_if_a_later_frame_recovers() {
        let mut now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        core.port = Some("memory-only".into());
        core.begin_return("return", core.reference, 0.1, true, now)
            .unwrap();
        for _ in 0..39 {
            now += Duration::from_millis(10);
            follow_reference(&mut core, now);
            core.advance(now, 0.01);
        }
        assert!(core.motion.as_ref().unwrap().settled_since.is_some());
        let mut spike = core.feedback[0].as_ref().unwrap().value.clone();
        spike.dq = 0.1;
        core.accept_feedback(spike.clone(), now + Duration::from_millis(2));
        spike.dq = 0.0;
        core.accept_feedback(spike, now + Duration::from_millis(3));
        now += Duration::from_millis(10);
        follow_reference(&mut core, now);
        core.advance(now, 0.01);
        assert!(
            core.enabled,
            "a recovered last sample must not conceal a speed spike"
        );
        assert!(core.port.is_some());
        assert!(core.motion.is_some());
    }
    #[test]
    fn return_needs_feedback_timestamps_that_cover_the_settle_window() {
        let mut now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        core.port = Some("memory-only".into());
        core.begin_return("return", core.reference, 0.1, true, now)
            .unwrap();
        for _ in 0..20 {
            now += Duration::from_millis(10);
            follow_reference(&mut core, now);
            core.advance(now, 0.01);
        }
        // A cached sample can stay inside FEEDBACK_MS without observing the end
        // of the settle interval. It must not authorize releasing the arm.
        for _ in 0..20 {
            now += Duration::from_millis(10);
            core.advance(now, 0.01);
        }
        assert!(core.enabled);
        assert!(core.motion.is_some());
        now += Duration::from_millis(10);
        follow_reference(&mut core, now);
        core.advance(now, 0.01);
        assert!(!core.enabled);
        assert_eq!(core.results.front().unwrap().code, "parked");
    }
    #[test]
    fn release_confirmation_is_explicit_in_json_requests() {
        for kind in ["disconnect", "disable"] {
            let now = Instant::now();
            let (mut core, _directory) = observed_core(now);
            core.enabled = true;
            core.port = Some("memory-only".into());
            let implicit: Operation =
                serde_json::from_value(serde_json::json!({"kind": kind})).unwrap();
            assert_eq!(
                core.apply("implicit", implicit, now, now).unwrap_err().code,
                "support_required"
            );
            assert!(core.enabled);
            let confirmed: Operation =
                serde_json::from_value(serde_json::json!({"kind": kind, "allow_release": true}))
                    .unwrap();
            core.apply("confirmed", confirmed, now, now).unwrap();
            assert!(!core.enabled);
            assert_eq!(core.port.is_none(), kind == "disconnect");
        }
    }
    #[test]
    fn invalid_return_does_not_mutate_hold_reference() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        let initial = core.reference;
        let mut invalid = initial;
        invalid[0] = f64::NAN;
        assert_eq!(
            core.begin_return("bad", invalid, 0.1, true, now)
                .unwrap_err()
                .code,
            "invalid_pose"
        );
        for speed in [0.0, -0.1, 0.21, f64::NAN] {
            assert_eq!(
                core.begin_return("bad", initial, speed, true, now)
                    .unwrap_err()
                    .code,
                "speed_limit"
            );
        }
        assert!(core.motion.is_none());
        assert_eq!(core.reference, initial);
        assert!(core.enabled);
    }
    #[test]
    fn joint_step_respects_final_dynamics_under_period_jitter_through_completion() {
        let mut now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        start_joint(&mut core, "jitter", now, true);
        let mut completed = false;
        for dt in [0.040, 0.002, 0.007, 0.003].into_iter().cycle().take(250) {
            now += Duration::from_secs_f64(dt);
            follow_reference(&mut core, now);
            let previous_q = core.reference;
            let previous_dq = core.velocity;
            core.advance(now, dt);
            assert_integrated_dynamics(&core, previous_q, previous_dq, dt);
            if core.motion.is_none() {
                completed = true;
                assert!(core.velocity.iter().all(|v| v.abs() < 1e-12));
                assert_eq!(core.results.front().unwrap().code, "completed");
                break;
            }
        }
        assert!(completed);
    }

    #[test]
    fn every_joint_jog_keeps_steady_low_speed_and_jerk_bounded_through_stop() {
        for axis in 0..7 {
            for speed in [0.001, 0.05, JOINT_SPEED] {
                for direction in [-1, 1] {
                    let mut now = Instant::now();
                    let (mut core, _directory) = observed_core(now);
                    core.enabled = true;
                    core.port = Some("memory-only".into());
                    core.begin(
                        "jog",
                        Space::Joint,
                        axis,
                        direction,
                        speed,
                        Frame::Base,
                        false,
                        0.0,
                        None,
                        0,
                        now,
                    )
                    .unwrap();
                    for tick in 0..600 {
                        let dt = [0.040, 0.002, 0.007, 0.003][tick % 4];
                        now += Duration::from_secs_f64(dt);
                        follow_reference(&mut core, now);
                        if tick == 180 {
                            core.apply("stop", Operation::Stop, now, now).unwrap();
                        }
                        let q = core.reference;
                        let v = core.velocity;
                        let a = core.acceleration;
                        core.advance(now, dt);
                        assert_integrated_dynamics(&core, q, v, dt);
                        for (i, previous_a) in a.into_iter().enumerate() {
                            assert!(
                                (core.acceleration[i] - previous_a).abs() <= JOINT_JERK * dt + 1e-9,
                                "axis={axis} speed={speed} tick={tick} joint={i}"
                            );
                            if i != axis {
                                assert_eq!(core.velocity[i], 0.0);
                            }
                        }
                        assert!(core.velocity[axis] * direction as f64 >= -1e-10);
                        if (100..180).contains(&tick) {
                            assert!((core.velocity[axis] - direction as f64 * speed).abs() < 1e-9);
                            assert!(core.acceleration[axis].abs() < 1e-9);
                        }
                        if tick >= 180 && !core.stopping {
                            break;
                        }
                    }
                    assert!(core.motion.is_none() && !core.stopping && core.at_rest());
                    assert!(core.enabled && core.port.is_some());
                    assert!(core.results.iter().any(|r| r.request_id == "jog"
                        && r.status == "cancelled"
                        && r.code == "stopped"));
                }
            }
        }
    }

    #[test]
    fn every_joint_step_reaches_rest_with_bounded_jerk_under_jitter() {
        for axis in 0..7 {
            for speed in [0.001, 0.05, JOINT_SPEED] {
                let mut now = Instant::now();
                let (mut core, _directory) = observed_core(now);
                core.enabled = true;
                core.begin(
                    "step",
                    Space::Joint,
                    axis,
                    -1,
                    speed,
                    Frame::Base,
                    true,
                    0.02,
                    None,
                    0,
                    now,
                )
                .unwrap();
                for tick in 0..4000 {
                    let dt = [0.040, 0.002, 0.007, 0.003][tick % 4];
                    now += Duration::from_secs_f64(dt);
                    follow_reference(&mut core, now);
                    let q = core.reference;
                    let v = core.velocity;
                    let a = core.acceleration;
                    core.advance(now, dt);
                    assert_integrated_dynamics(&core, q, v, dt);
                    for (i, previous_a) in a.into_iter().enumerate() {
                        assert!(
                            (core.acceleration[i] - previous_a).abs() <= JOINT_JERK * dt + 1e-9,
                            "axis={axis} speed={speed} tick={tick}"
                        );
                    }
                    if core.motion.is_none() {
                        break;
                    }
                }
                assert!(
                    core.motion.is_none() && core.at_rest(),
                    "axis={axis} speed={speed}"
                );
                assert_eq!(core.results.front().unwrap().code, "completed");
                assert!(core.enabled);
            }
        }
    }

    #[test]
    fn cartesian_jogs_and_steps_preserve_offset_tcp_and_jerk_at_low_speed() {
        for frame in [Frame::Base, Frame::Tool] {
            for axis in 0..6 {
                for step in [false, true] {
                    let mut now = Instant::now();
                    let (mut core, _directory) = observed_core(now);
                    let q = [1.5576, -0.0350, 0.0102, -0.0217, 1.6217, 0.0088, 0.1133];
                    for (i, value) in q.into_iter().enumerate() {
                        let mut f = core.feedback[i].as_ref().unwrap().value.clone();
                        f.q = value;
                        core.accept_feedback(f, now);
                    }
                    core.reference = q;
                    core.enabled = true;
                    core.tcp = Some(Pose {
                        translation: [0.0, 0.0, 0.1],
                        rotation: [0.0, 0.0, 0.0, 1.0],
                    });
                    core.port = Some("memory-only".into());
                    let speed = if axis < 3 { 0.0005 } else { 0.005 };
                    let amount = if axis < 3 { 0.002 } else { 0.03 };
                    core.begin(
                        "cartesian",
                        Space::Cartesian,
                        axis,
                        1,
                        speed,
                        frame,
                        step,
                        amount,
                        None,
                        0,
                        now,
                    )
                    .unwrap();
                    let path = Core::motion_path(core.motion.as_ref().unwrap()).unwrap();
                    for tick in 0..2200 {
                        let dt = [0.040, 0.002, 0.007, 0.003][tick % 4];
                        now += Duration::from_secs_f64(dt);
                        follow_reference(&mut core, now);
                        if !step && tick == 180 {
                            core.apply("stop", Operation::Stop, now, now).unwrap();
                        }
                        let old_q = core.reference;
                        let old_v = core.velocity;
                        let old_a = core.acceleration;
                        core.advance(now, dt);
                        assert_integrated_dynamics(&core, old_q, old_v, dt);
                        for (i, previous_a) in old_a.into_iter().enumerate() {
                            assert!(
                                (core.acceleration[i] - previous_a).abs() <= JOINT_JERK * dt + 1e-8,
                                "{frame:?} axis={axis} step={step} tick={tick}"
                            );
                        }
                        path.check(
                            &core
                                .model
                                .as_ref()
                                .unwrap()
                                .fk(&core.reference, core.tcp.as_ref().unwrap())
                                .unwrap(),
                        )
                        .unwrap();
                        if core.motion.is_none() && !core.stopping {
                            break;
                        }
                    }
                    assert!(
                        core.motion.is_none() && !core.stopping && core.at_rest(),
                        "{frame:?} axis={axis} step={step}"
                    );
                    assert_eq!(
                        core.results.front().unwrap().code,
                        if step { "completed" } else { "stopped" }
                    );
                    assert!(core.enabled && core.port.is_some());
                }
            }
        }
    }
    #[test]
    fn cartesian_step_decelerates_before_reporting_completed() {
        let mut now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        core.tcp = Some(Pose::identity());
        core.begin(
            "cartesian",
            Space::Cartesian,
            0,
            1,
            0.005,
            Frame::Base,
            true,
            0.001,
            None,
            0,
            now,
        )
        .unwrap();
        // A final segment has reached its pose tolerance while feedforward is nonzero.
        let pose = core
            .model
            .as_ref()
            .unwrap()
            .fk(&core.reference, core.tcp.as_ref().unwrap())
            .unwrap();
        core.motion.as_mut().unwrap().target_pose = Some(pose);
        core.motion.as_mut().unwrap().duration = 0.1;
        core.velocity[0] = 0.02;
        now += Duration::from_millis(110);
        follow_reference(&mut core, now);
        let q = core.reference;
        let dq = core.velocity;
        core.advance(now, 0.002);
        assert_integrated_dynamics(&core, q, dq, 0.002);
        assert!(core.motion.is_some());
        assert!(core.velocity[0] > 0.0);
        for _ in 0..300 {
            now += Duration::from_millis(2);
            follow_reference(&mut core, now);
            let q = core.reference;
            let dq = core.velocity;
            core.advance(now, 0.002);
            assert_integrated_dynamics(&core, q, dq, 0.002);
            if core.motion.is_none() {
                break;
            }
        }
        assert!(core.motion.is_none());
        assert!(core.velocity.iter().all(|v| v.abs() < 1e-12));
        assert_eq!(core.results.front().unwrap().code, "completed");
    }
    #[test]
    fn cartesian_step_does_not_complete_with_reference_outside_pose_tolerance() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        core.tcp = Some(Pose::identity());
        core.begin(
            "cartesian",
            Space::Cartesian,
            5,
            1,
            0.005,
            Frame::Tool,
            true,
            0.001,
            None,
            0,
            now,
        )
        .unwrap();
        let pose = core
            .model
            .as_ref()
            .unwrap()
            .fk(&core.reference, core.tcp.as_ref().unwrap())
            .unwrap();
        core.motion.as_mut().unwrap().target_pose = Some(pose);
        core.motion.as_mut().unwrap().duration = 0.1;
        // J7 turns about the Link7 tool-Z axis without leaving the selected
        // TCP rotation path; the reference is still outside pose tolerance.
        core.reference[6] += 0.03;
        core.advance(now + Duration::from_millis(110), 0.002);
        assert!(core.motion.is_some());
        assert!(
            !core
                .results
                .iter()
                .any(|result| result.status == "completed")
        );
    }
    #[test]
    fn watchdog_trip_latches_and_normal_feedback_cannot_resume_motion() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        start_joint(&mut core, "jog", now, false);
        core.velocity[0] = 0.03;
        let mut sample = core.feedback[0].as_ref().unwrap().value.clone();
        sample.timeout = true;
        core.accept_feedback(sample.clone(), now + Duration::from_millis(1));
        assert!(!core.enabled);
        assert!(core.motion.is_none());
        assert_eq!(core.velocity, [0.0; 7]);
        assert_eq!(core.error.as_ref().unwrap().code, "device_timeout");
        sample.timeout = false;
        core.accept_feedback(sample, now + Duration::from_millis(2));
        core.settle_transitions(now + Duration::from_millis(2));
        assert!(!core.enabled);
        assert!(core.error.is_some());
        assert!(core.motion.is_none());
    }
    #[test]
    fn drive_exiting_servo_cancels_active_motion() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        start_joint(&mut core, "step", now, true);
        let mut sample = core.feedback[3].as_ref().unwrap().value.clone();
        sample.mode = 0;
        core.accept_feedback(sample, now + Duration::from_millis(1));
        assert!(!core.enabled);
        assert!(core.motion.is_none());
        assert_eq!(core.error.as_ref().unwrap().code, "device_disabled");
    }
    #[test]
    fn enable_requires_all_seven_post_request_confirmations() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enable_request = Some(("enable".into(), now));
        core.settle_transitions(now);
        assert!(!core.enabled);
        for i in 0..6 {
            let sample = core.feedback[i].as_ref().unwrap().value.clone();
            core.accept_feedback(sample, now + Duration::from_millis(10));
        }
        core.settle_transitions(now + Duration::from_millis(10));
        assert!(!core.enabled);
        let sample = core.feedback[6].as_ref().unwrap().value.clone();
        core.accept_feedback(sample, now + Duration::from_millis(15));
        core.settle_transitions(now + Duration::from_millis(15));
        assert!(core.enabled);
        assert!(core.enable_request.is_none());
        assert_eq!(core.results.front().unwrap().code, "enabled");
    }
    #[test]
    fn finite_pose_below_urdf_lower_passes_enable_check_and_joint_step() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        let (lower, _) = core.model.as_ref().unwrap().position_limits();
        let mut q = core.reference;
        // Real installation zero: J2 ~ -0.035, J4 ~ -0.022, both below the
        // official model's lower bound of zero.
        q[1] = lower[1] - 0.035;
        q[3] = lower[3] - 0.022;
        for (i, target) in q.iter().enumerate() {
            let mut sample = core.feedback[i].as_ref().unwrap().value.clone();
            sample.q = *target;
            core.accept_feedback(sample, now + Duration::from_millis(1));
        }
        let at = now + Duration::from_millis(1);
        assert!(core.check_positions(&q).is_ok());
        core.enabled = true;
        core.begin(
            "outside-urdf",
            Space::Joint,
            1,
            1,
            0.05,
            Frame::Base,
            true,
            0.01,
            None,
            0,
            at,
        )
        .unwrap();
        assert!(core.error.is_none());
        assert!(core.motion.is_some());
        core.advance(at + Duration::from_millis(2), 0.002);
        assert!(core.error.is_none(), "{:?}", core.error);
        assert!(core.motion.is_some());
        for (reference, target) in core.reference.iter().zip(q) {
            assert!((reference - target).abs() <= 0.01 + 1e-12);
        }
    }
    #[test]
    fn joint_reference_keeps_pushing_up_when_feedback_sags_under_load() {
        let mut now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        let start = core.reference;
        core.begin(
            "loaded-joint",
            Space::Joint,
            0,
            1,
            0.05,
            Frame::Base,
            false,
            0.0,
            Some(now + Duration::from_secs(5)),
            1,
            now,
        )
        .unwrap();
        for _ in 0..40 {
            now += Duration::from_millis(20);
            for (i, target) in start.iter().enumerate() {
                let mut sample = core.feedback[i].as_ref().unwrap().value.clone();
                sample.q = *target;
                sample.dq = 0.0;
                core.accept_feedback(sample, now);
            }
            core.advance(now, 0.02);
        }
        assert!(core.reference[0] > start[0] + 0.02);
        let before_sag = core.reference[0];
        now += Duration::from_millis(20);
        for (i, target) in start.iter().enumerate() {
            let mut sample = core.feedback[i].as_ref().unwrap().value.clone();
            sample.q = if i == 0 { *target - 0.20 } else { *target };
            sample.dq = 0.0;
            core.accept_feedback(sample, now);
        }
        core.advance(now, 0.02);
        assert!(core.error.is_none(), "{:?}", core.error);
        assert!(core.motion.is_some());
        assert!(core.reference[0] > before_sag);
    }
    #[test]
    fn restarting_joint_jog_preserves_existing_holding_reference() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        core.reference[0] += 0.10;
        let held = core.reference;
        let measured = core.measured(now).unwrap();
        assert!((held[0] - measured[0]).abs() < MAX_TRACKING_ERROR);
        core.begin(
            "restarted-joint",
            Space::Joint,
            0,
            -1,
            0.05,
            Frame::Base,
            false,
            0.0,
            Some(now + Duration::from_millis(500)),
            1,
            now,
        )
        .unwrap();
        assert_eq!(core.reference, held);
        assert_eq!(core.motion.as_ref().unwrap().initial, held);
    }
    #[test]
    fn continuous_joint_jog_below_urdf_lower_does_not_stop_on_model_limit() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        let (lower, _) = core.model.as_ref().unwrap().position_limits();
        let mut q = core.reference;
        q[1] = lower[1] - 0.035;
        for (i, target) in q.iter().enumerate() {
            let mut sample = core.feedback[i].as_ref().unwrap().value.clone();
            sample.q = *target;
            core.accept_feedback(sample, now + Duration::from_millis(1));
        }
        let at = now + Duration::from_millis(1);
        core.enabled = true;
        core.begin(
            "below-lower",
            Space::Joint,
            1,
            -1,
            0.05,
            Frame::Base,
            false,
            0.01,
            Some(at + Duration::from_millis(150)),
            1,
            at,
        )
        .unwrap();
        core.advance(at + Duration::from_millis(2), 0.002);
        assert!(core.error.is_none(), "{:?}", core.error);
        assert!(core.motion.is_some());
        assert!(core.reference[1] < q[1]);
    }
    #[test]
    fn cancelled_or_timed_out_enable_cannot_complete_later() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enable_request = Some(("cancel".into(), now));
        core.end_motion("cancelled", "host_stop", "停止");
        core.settle_transitions(now + Duration::from_millis(10));
        assert!(!core.enabled);
        assert!(core.enable_request.is_none());
        core.enable_request = Some(("timeout".into(), now));
        core.settle_transitions(now + ENABLE_TIMEOUT);
        assert_eq!(core.error.as_ref().unwrap().code, "enable_timeout");
        for i in 0..7 {
            let sample = core.feedback[i].as_ref().unwrap().value.clone();
            core.accept_feedback(sample, now + ENABLE_TIMEOUT + Duration::from_millis(1));
        }
        core.settle_transitions(now + ENABLE_TIMEOUT + Duration::from_millis(1));
        assert!(!core.enabled);
        assert!(core.enable_request.is_none());
    }
    #[test]
    fn clear_fault_requires_fresh_feedback_without_timeout() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now + Duration::from_millis(1));
        core.fault("device_timeout", "超时");
        core.clear_request = Some(("clear".into(), now));
        core.feedback[0].as_mut().unwrap().value.timeout = true;
        core.settle_transitions(now + Duration::from_millis(2));
        assert!(core.error.is_some());
        core.feedback[0].as_mut().unwrap().value.timeout = false;
        core.settle_transitions(now + Duration::from_millis(3));
        assert!(core.error.is_none());
        assert!(!core.enabled);
        assert!(core.enable_request.is_none());
    }
    #[test]
    fn expired_continuous_action_cannot_be_revived_by_late_update() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        start_joint(&mut core, "jog", now, false);
        core.advance(now + Duration::from_millis(151), 0.002);
        assert!(core.motion.is_none());
        let late = core.apply(
            "update",
            Operation::JogUpdate {
                action_id: "jog".into(),
                input_seq: 2,
                ttl_ms: 150,
            },
            now + Duration::from_millis(151),
            now + Duration::from_millis(151),
        );
        assert_eq!(late.unwrap_err().code, "no_action");
        assert!(core.motion.is_none());
    }
    #[test]
    fn duplicate_request_does_not_restart_or_queue_a_step() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        let lease = core
            .control(
                ControlRequest {
                    action: "acquire".into(),
                    client_id: "client".into(),
                    lease_id: None,
                },
                now,
            )
            .unwrap();
        let request = CommandRequest {
            request_id: "same-step".into(),
            client_id: "client".into(),
            lease_id: lease.lease_id.unwrap(),
            generation: lease.generation,
            operation: Operation::Step {
                space: Space::Joint,
                axis: 0,
                direction: 1,
                amount: 0.01,
                speed: 0.05,
                frame: Frame::Base,
            },
        };
        core.command(request.clone(), now, now);
        let count = core.results.len();
        core.command(request.clone(), now, now + Duration::from_millis(1));
        assert_eq!(core.results.len(), count);
        assert_eq!(core.motion.as_ref().unwrap().started, now);
        core.end_motion("cancelled", "host_stop", "停止");
        core.command(request, now, now + Duration::from_millis(2));
        assert!(core.motion.is_none());
        assert_eq!(core.results.front().unwrap().code, "stale_context");
    }
    #[test]
    fn offline_tcp_configuration_does_not_fabricate_device_feedback() {
        let (mut core, _directory) = core();
        let now = Instant::now();
        let tcp = Pose {
            translation: [0.0, 0.0, 0.02],
            rotation: [0.0, 0.0, 0.0, 1.0],
        };
        core.apply("tcp", Operation::ConfigureTcp { tcp }, now, now)
            .unwrap();
        let snapshot = core.snapshot(now);
        assert!(snapshot.tcp_configured);
        assert_eq!(
            snapshot.tcp_transform.unwrap().translation,
            [0.0, 0.0, 0.02]
        );
        assert!(snapshot.tcp.is_none());
        assert!(snapshot.joints.iter().all(|j| j.q.is_none()));
        assert_eq!(snapshot.phase, Phase::Disconnected);
    }
    #[test]
    fn only_secret_holder_can_renew_and_expired_lease_cannot_revive() {
        let (mut core, _directory) = core();
        let now = Instant::now();
        let reply = core
            .control(
                ControlRequest {
                    action: "acquire".into(),
                    client_id: "first".into(),
                    lease_id: None,
                },
                now,
            )
            .unwrap();
        assert!(
            core.control(
                ControlRequest {
                    action: "renew".into(),
                    client_id: "first".into(),
                    lease_id: Some("wrong".into())
                },
                now
            )
            .is_err()
        );
        assert!(
            core.control(
                ControlRequest {
                    action: "acquire".into(),
                    client_id: "other".into(),
                    lease_id: None
                },
                now
            )
            .is_err()
        );
        assert!(
            core.control(
                ControlRequest {
                    action: "renew".into(),
                    client_id: "first".into(),
                    lease_id: reply.lease_id
                },
                now + Duration::from_millis(LEASE_MS + 1)
            )
            .is_err()
        );
        assert!(core.lease.is_none());
    }
    #[test]
    fn queue_delay_does_not_refresh_input_deadline() {
        let received = Instant::now();
        assert!(input_deadline(received, 150, received + Duration::from_millis(151)).is_err());
        assert_eq!(
            input_deadline(received, 150, received).unwrap(),
            received + Duration::from_millis(150)
        );
    }
    #[test]
    fn host_stop_keeps_owner_and_enable_and_invalidates_context() {
        let (mut core, _directory) = core();
        let now = Instant::now();
        core.control(
            ControlRequest {
                action: "acquire".into(),
                client_id: "external".into(),
                lease_id: None,
            },
            now,
        )
        .unwrap();
        core.enabled = true;
        let generation = core.generation;
        core.end_motion("cancelled", "host_stop", "停止");
        assert!(core.enabled);
        assert_eq!(core.lease.as_ref().unwrap().client_id, "external");
        assert!(core.generation > generation);
        assert!(core.snapshot(now).joints.iter().all(|q| q.q.is_none()));
    }
    #[test]
    fn snapshot_never_discloses_control_secret() {
        let (mut core, _directory) = core();
        let now = Instant::now();
        let reply = core
            .control(
                ControlRequest {
                    action: "acquire".into(),
                    client_id: "client".into(),
                    lease_id: None,
                },
                now,
            )
            .unwrap();
        let json = serde_json::to_string(&core.snapshot(now)).unwrap();
        assert!(!json.contains(&reply.lease_id.unwrap()));
        assert!(!json.contains("lease_id"));
    }
    #[test]
    fn control_reply_observes_committed_snapshot() {
        let (mut core, _directory) = core();
        let (publisher, snapshot) = watch::channel(core.snapshot(Instant::now()));
        let (reply, mut response) = oneshot::channel();
        core.work(
            Work::Control(
                ControlRequest {
                    action: "acquire".into(),
                    client_id: "desktop".into(),
                    lease_id: None,
                },
                Instant::now() + Duration::from_secs(1),
                reply,
            ),
            &publisher,
        );
        let reply = response.try_recv().unwrap().unwrap();
        assert_eq!(snapshot.borrow().generation, reply.generation);
        assert_eq!(
            snapshot.borrow().owner.as_ref().unwrap().client_id,
            "desktop"
        );
    }
}
