use crate::{
    journal::{Journal, JournalStats},
    kinematics::{self, Frame, Kinematics, LocalTarget, MotionLimits, Pose, StepInput},
    protocol::{Bus, DriveMode, Feedback, Setpoint},
};
use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use serde::{Deserialize, Serialize};
use std::{
    collections::VecDeque,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU64, Ordering},
    },
    time::{Duration, Instant},
};
use tokio::sync::{oneshot, watch};

const LEASE_MS: u64 = 2500;
const FEEDBACK_MS: u64 = 200;
const FEEDBACK_SPAN_MS: u64 = 100;
const ENABLE_TIMEOUT: Duration = Duration::from_secs(1);
const MAX_REFERENCE_ERROR: f64 = 0.15;
const JOINT_SPEED: f64 = 0.20;
const JOINT_ACCELERATION: f64 = 0.60;
const LINEAR_SPEED: f64 = 0.015;
const ANGULAR_SPEED: f64 = 0.15;
// Defaults from the official test_arm_lowlevel Options, at the recorded SDK commit.
const SERVO_KP: f64 = 60.0;
const SERVO_KD: f64 = 0.5;

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
    pub axis: usize,
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
    Disconnect,
    Enable,
    Disable,
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
    Stop,
}
impl Operation {
    fn name(&self) -> &'static str {
        match self {
            Self::Connect { .. } => "connect",
            Self::Disconnect => "disconnect",
            Self::Enable => "enable",
            Self::Disable => "disable",
            Self::ClearFault => "clear_fault",
            Self::ConfigureTcp { .. } => "configure_tcp",
            Self::JogStart { .. } => "jog_start",
            Self::JogUpdate { .. } => "jog_update",
            Self::Step { .. } => "step",
            Self::Stop => "stop",
        }
    }
}

enum Work {
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
        let shutdown = Arc::new(AtomicBool::new(false));
        let worker_epoch = host_epoch.clone();
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
    pub fn host_stop(&self) {
        self.host_epoch.fetch_add(1, Ordering::AcqRel);
    }
    pub fn request_shutdown(&self) {
        self.shutdown.store(true, Ordering::Release);
        self.host_stop();
    }
    pub fn shutdown(&self, timeout: Duration) -> bool {
        self.request_shutdown();
        self.finished.recv_timeout(timeout).is_ok()
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
struct Motion {
    request_id: String,
    space: Space,
    axis: usize,
    direction: f64,
    speed: f64,
    frame: Frame,
    step: bool,
    amount: f64,
    started: Instant,
    duration: f64,
    initial: [f64; 7],
    target: [f64; 7],
    initial_pose: Option<Pose>,
    target_pose: Option<Pose>,
    deadline: Option<Instant>,
    input_seq: u64,
}
struct Core {
    journal: Journal,
    model: Option<Kinematics>,
    side: String,
    port: Option<String>,
    bus: Option<Bus>,
    feedback: [Option<Observation>; 7],
    reference: [f64; 7],
    velocity: [f64; 7],
    tcp: Option<Pose>,
    enabled: bool,
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
    next_joint: usize,
    connected_at: Option<Instant>,
    clear_pending: [bool; 7],
    clear_request: Option<(String, Instant)>,
    enable_request: Option<(String, Instant)>,
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
            tcp: None,
            enabled: false,
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
            next_joint: 0,
            connected_at: None,
            clear_pending: [false; 7],
            clear_request: None,
            enable_request: None,
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
            self.velocity = [0.0; 7];
            self.record(&id, status, code, message);
        }
        if let Some(motion) = self.motion.take() {
            self.record(&motion.request_id, status, code, message);
        }
        self.generation += 1;
        self.stopping = self.enabled && self.velocity.iter().any(|v| v.abs() > 1e-6);
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
        self.velocity = [0.0; 7];
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
                let bus = Bus::open(&port, &side, 6_000_000, Duration::from_millis(2))
                    .map_err(|e| ApiError::new("connect_failed", e))?;
                if self.side != side {
                    self.tcp = None;
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
                self.velocity = [0.0; 7];
                self.connected_at = Some(now);
                self.phase = Phase::Connecting;
                self.generation += 1;
                self.journal
                    .log("info", "device", "connected", "串口连接已建立", Some(id));
            }
            Operation::Disconnect => {
                self.end_motion("cancelled", "disconnected", "设备连接已断开");
                self.enabled = false;
                self.send_disable();
                self.bus = None;
                self.port = None;
                self.feedback = std::array::from_fn(|_| None);
                self.error = None;
                self.clear_request = None;
                self.clear_pending = [false; 7];
                self.phase = Phase::Disconnected;
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
                self.velocity = [0.0; 7];
                self.enable_request = Some((id.into(), now));
                self.stopping = false;
                self.phase = Phase::Enabling;
                self.generation += 1;
            }
            Operation::Disable => {
                self.end_motion("cancelled", "disabled", "设备已失能");
                self.enabled = false;
                self.velocity = [0.0; 7];
                self.stopping = false;
                self.phase();
            }
            Operation::ClearFault => {
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
                if motion.step
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
        if !self.enabled || self.error.is_some() {
            return Err(ApiError::new("not_ready", "设备尚未就绪"));
        }
        if self.motion.is_some() || self.stopping {
            return Err(ApiError::new("motion_busy", "当前动作尚未结束"));
        }
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
        let measured = self.measured(now)?;
        self.check_positions(&measured)?;
        self.reference = measured;
        self.velocity = [0.0; 7];
        let mut target = measured;
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
                .fk(&measured, tcp)
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
            step,
            amount,
            started: now,
            duration,
            initial: measured,
            target,
            initial_pose,
            target_pose,
            deadline,
            input_seq,
        });
        self.phase = Phase::Moving;
        Ok(())
    }
    fn check_positions(&self, q: &[f64; 7]) -> Result<(), ApiError> {
        let (lower, upper) = self
            .model
            .as_ref()
            .ok_or_else(|| ApiError::new("model_error", "模型不可用"))?
            .position_limits();
        if (0..7).any(|i| !q[i].is_finite() || q[i] < lower[i] - 1e-8 || q[i] > upper[i] + 1e-8) {
            return Err(ApiError::new("joint_limit", "关节位置超限"));
        }
        Ok(())
    }
    // Constrain the velocity that will actually be transmitted, then integrate once.
    // Absolute-time waypoints alone do not bound derivatives under scheduler jitter.
    fn commit_joint_velocity(
        &mut self,
        desired: [f64; 7],
        measured: &[f64; 7],
        dt: f64,
    ) -> Result<(), ApiError> {
        let (lower, upper) = self.model.as_ref().unwrap().position_limits();
        let mut velocity = [0.0; 7];
        let mut reference = self.reference;
        for i in 0..7 {
            let low_position = lower[i].max(measured[i] - MAX_REFERENCE_ERROR);
            let high_position = upper[i].min(measured[i] + MAX_REFERENCE_ERROR);
            let brake_speed = |distance: f64| {
                let at = JOINT_ACCELERATION * dt;
                let twice_ad = 2.0 * JOINT_ACCELERATION * distance.max(0.0);
                twice_ad / ((at * at + twice_ad).sqrt() + at)
            };
            let low = (-JOINT_SPEED)
                .max(self.velocity[i] - JOINT_ACCELERATION * dt)
                .max((low_position - self.reference[i]) / dt)
                .max(-brake_speed(self.reference[i] - low_position));
            let high = JOINT_SPEED
                .min(self.velocity[i] + JOINT_ACCELERATION * dt)
                .min((high_position - self.reference[i]) / dt)
                .min(brake_speed(high_position - self.reference[i]));
            if !desired[i].is_finite() || low > high {
                return Err(ApiError::new("joint_dynamics", "关节动态约束无法满足"));
            }
            velocity[i] = desired[i].clamp(low, high);
            reference[i] += velocity[i] * dt;
        }
        self.check_positions(&reference)?;
        self.reference = reference;
        self.velocity = velocity;
        Ok(())
    }
    fn advance(&mut self, now: Instant, dt: f64) {
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
        if (0..7).any(|i| (self.reference[i] - measured[i]).abs() > MAX_REFERENCE_ERROR) {
            self.fault("following_error", "关节跟随偏差超限");
            return;
        }
        if self
            .motion
            .as_ref()
            .is_some_and(|motion| motion.deadline.is_some_and(|deadline| now >= deadline))
        {
            self.end_motion("cancelled", "input_expired", "连续输入已过期");
        }
        if self.stopping {
            if let Err(e) = self.commit_joint_velocity([0.0; 7], &measured, dt) {
                self.fault(&e.code, e.message);
                return;
            }
            if self.velocity.iter().all(|v| v.abs() < 1e-12) {
                self.stopping = false;
                self.phase();
                self.dirty = true;
            }
            return;
        }
        let Some(motion) = self.motion.as_ref() else {
            return;
        };
        let elapsed = now.saturating_duration_since(motion.started).as_secs_f64();
        let t = if motion.step {
            (elapsed / motion.duration).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let smooth = t * t * (3.0 - 2.0 * t);
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
            motion.step && t >= 1.0 && within_target(&measured) && within_target(&self.reference);
        if at_target {
            if let Err(e) = self.commit_joint_velocity([0.0; 7], &measured, dt) {
                self.fault(&e.code, e.message);
                return;
            }
            if self.velocity.iter().all(|v| v.abs() < 1e-12) {
                let id = self.motion.take().unwrap().request_id;
                self.generation += 1;
                self.record(&id, "completed", "completed", "动作已完成");
                self.phase();
            }
            return;
        }
        if motion.space == Space::Joint {
            let desired_velocity = if motion.step {
                let next: [f64; 7] = std::array::from_fn(|i| {
                    motion.initial[i] + (motion.target[i] - motion.initial[i]) * smooth
                });
                std::array::from_fn(|i| {
                    ((next[i] - self.reference[i]) / dt).clamp(-motion.speed, motion.speed)
                })
            } else {
                let i = motion.axis;
                let (lower, upper) = self.model.as_ref().unwrap().position_limits();
                let remaining = if motion.direction > 0.0 {
                    upper[i] - self.reference[i]
                } else {
                    self.reference[i] - lower[i]
                };
                if remaining <= 1e-5 {
                    self.end_motion("failed", "joint_limit", "已到达关节限位");
                    return;
                }
                let mut desired = [0.0; 7];
                desired[i] = motion.direction * motion.speed;
                desired
            };
            if let Err(e) = self.commit_joint_velocity(desired_velocity, &measured, dt) {
                self.fault(&e.code, e.message);
                return;
            }
        } else {
            let tcp = *self.tcp.as_ref().unwrap();
            let target = if motion.step {
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
            let limits = MotionLimits {
                joint_velocity: [JOINT_SPEED; 7],
                joint_acceleration: [JOINT_ACCELERATION; 7],
                max_reference_error: MAX_REFERENCE_ERROR,
                max_joint_step: 0.01,
                linear_speed: LINEAR_SPEED,
                angular_speed: ANGULAR_SPEED,
            };
            match self.model.as_ref().unwrap().step(StepInput {
                reference: self.reference,
                measured,
                previous_velocity: self.velocity,
                target,
                dt: dt.max(0.0001),
                tcp,
                limits,
            }) {
                Ok(candidate) => {
                    self.reference = candidate.q;
                    self.velocity = candidate.dq;
                }
                Err(e) => {
                    self.end_motion("failed", "kinematics", &e);
                    return;
                }
            }
        }
        if self
            .motion
            .as_ref()
            .is_some_and(|m| m.step && elapsed > m.duration + 3.0)
        {
            self.end_motion("failed", "target_timeout", "目标到达超时");
        }
    }
    fn poll(&mut self, now: Instant) {
        let i = self.next_joint;
        self.next_joint = (i + 1) % 7;
        let clear = self.clear_pending[i];
        let servo = self.enabled || self.enable_request.is_some();
        let setpoint = Setpoint {
            mode: if clear {
                DriveMode::ClearFault
            } else if servo {
                DriveMode::Servo
            } else {
                DriveMode::Disabled
            },
            q: self.reference[i],
            dq: self.velocity[i],
            tau: 0.0,
            kp: if servo { SERVO_KP } else { 0.0 },
            kd: if servo { SERVO_KD } else { 0.0 },
        };
        let Some(bus) = self.bus.as_mut() else {
            return;
        };
        let result = bus.exchange(i as u8, setpoint);
        let stats = bus.stats();
        self.communication = Communication {
            received: stats.received,
            crc_errors: stats.crc_errors,
            timeouts: stats.timeouts,
        };
        self.clear_pending[i] = false;
        match result {
            Ok(Some(value)) if value.id < 7 => {
                self.accept_feedback(value, Instant::now());
            }
            Ok(_) => {}
            Err(e) => self.fault("communication_error", e),
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
    fn accept_feedback(&mut self, value: Feedback, at: Instant) {
        let id = value.id as usize;
        if id >= 7 {
            return;
        }
        let error = value.error;
        let timeout = value.timeout;
        let mode = value.mode;
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
        if let Some(bus) = self.bus.as_mut() {
            for i in 0..7 {
                let _ = bus.exchange(
                    i,
                    Setpoint {
                        mode: DriveMode::Disabled,
                        q: 0.0,
                        dq: 0.0,
                        tau: 0.0,
                        kp: 0.0,
                        kd: 0.0,
                    },
                );
            }
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
                mode: if motion.step { "step" } else { "continuous" }.into(),
                axis: motion.axis,
                progress: if motion.step {
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
fn run(
    mut core: Core,
    ordinary: Receiver<Work>,
    priority: Receiver<Work>,
    latest: Receiver<Work>,
    host_epoch: Arc<AtomicU64>,
    shutdown: Arc<AtomicBool>,
    state: watch::Sender<Snapshot>,
) {
    let mut seen_epoch = 0;
    let mut last_control = Instant::now();
    let mut published = last_control;
    loop {
        let now = Instant::now();
        let epoch = host_epoch.load(Ordering::Acquire);
        if epoch != seen_epoch {
            core.journal
                .log("info", "control", "host_stop", "停止请求", None);
            core.end_motion("cancelled", "host_stop", "宿主已取消当前动作");
            seen_epoch = epoch;
        }
        if shutdown.load(Ordering::Acquire) {
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
        if host_epoch.load(Ordering::Acquire) != seen_epoch || shutdown.load(Ordering::Acquire) {
            continue;
        }
        let at = Instant::now();
        let dt = at
            .saturating_duration_since(last_control)
            .as_secs_f64()
            .max(0.0001);
        last_control = at;
        core.expire_lease(at);
        core.advance(at, dt);
        if host_epoch.load(Ordering::Acquire) != seen_epoch || shutdown.load(Ordering::Acquire) {
            continue;
        }
        core.expire_lease(Instant::now());
        core.poll(Instant::now());
        if core.dirty || published.elapsed() >= Duration::from_millis(33) {
            core.seq += 1;
            state.send_replace(core.snapshot(Instant::now()));
            core.dirty = false;
            published = Instant::now();
        }
        std::thread::sleep(Duration::from_millis(2));
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
        for _ in 0..50 {
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
        let pose = core
            .model
            .as_ref()
            .unwrap()
            .fk(&core.reference, core.tcp.as_ref().unwrap())
            .unwrap();
        core.motion.as_mut().unwrap().target_pose = Some(pose);
        core.motion.as_mut().unwrap().duration = 0.1;
        core.reference[0] += 0.03;
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
