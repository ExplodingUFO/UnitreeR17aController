#[cfg(test)]
use crate::trajectory::{JOINT_ACCELERATION, JOINT_JERK, JOINT_SPEED};
use crate::{
    calibration::{
        self, Choice, Collector, Leg, Point, Report as CalibrationReport,
        Settings as CalibrationSettings, Status as CalibrationStatus, Trial,
    },
    calibration_path::{Playback, PlaybackPhase},
    journal::{Journal, JournalStats, TraceData},
    kinematics::{
        self, CartesianPath, CartesianPlanning, Frame, HoldPose, Kinematics, LocalTarget,
        MotionLimits, Pose, SmoothingState, StepInput,
    },
    protocol::{BusError, BusHandle, DriveMode, Feedback, SentReference, Setpoint},
    reference::{CartesianGuard, MAX_REFERENCE_AGE, Reference, Sampling},
    teaching::{
        self, Capture as TeachingCapture, Guidance, PlaybackOptions, Program as TeachingProgram,
        Recorder as TeachingRecorder, Summary as TeachingSummary,
    },
    trajectory::{self, MotionTuning},
    tuning::ServoTuning,
};
use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use serde::{Deserialize, Serialize};
use std::{
    collections::{HashSet, VecDeque},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
    },
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
use tokio::sync::{oneshot, watch};

const LEASE_MS: u64 = 2500;
const FEEDBACK_MS: u64 = 200;
const ENABLE_TIMEOUT: Duration = Duration::from_secs(1);
const CONTROL_PERIOD: Duration = Duration::from_millis(10);
// The official motion demo uses these gains for loaded joint movement.
const SERVO_KP: f64 = 60.0;
// The official R1-7A teleoperation stack uses kd=10 with gravity feedforward.
const SERVO_KD: f64 = 10.0;
const RETURN_POSITION_TOLERANCE: f64 = 0.02;

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
    Teaching,
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
    pub servo_tuning: ServoTuning,
    pub servo_tuning_revision: u64,
    pub calibration: Option<CalibrationStatus>,
    pub motion_tuning: MotionTuning,
    pub teaching: Option<TeachingStatus>,
    pub teaching_mode: Option<TeachingModeStatus>,
    pub teaching_recording: Option<TeachingStatus>,
    pub latest_teaching: Option<TeachingSummary>,
    #[serde(skip)]
    pub(crate) teaching_capture: Option<Arc<TeachingCapture>>,
    #[serde(skip)]
    pub(crate) teaching_pending: Vec<Arc<TeachingCapture>>,
    #[serde(skip)]
    pub(crate) calibration_report: Option<Arc<CalibrationReport>>,
    #[serde(skip)]
    pub(crate) reference: [f64; 7],
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
    ConfigureServo {
        config: Box<ServoTuning>,
    },
    ConfigureMotion {
        config: MotionTuning,
    },
    CalibrateStart {
        #[serde(default)]
        settings: CalibrationSettings,
    },
    CalibrateApply {
        calibration_id: String,
    },
    TeachStart {
        #[serde(default)]
        name: String,
    },
    TeachMode {
        enabled: bool,
        #[serde(default = "default_teaching_transition")]
        transition_s: f64,
    },
    TeachFinish {
        #[serde(default)]
        recording_request_id: Option<String>,
    },
    TeachReplay {
        trajectory_id: String,
        #[serde(flatten)]
        options: PlaybackOptions,
    },
    /// The HTTP worker supplies validated, precomputed paths. It cannot be
    /// deserialized from a client's command body.
    #[serde(skip)]
    TeachingPrepared {
        program: Arc<TeachingProgram>,
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
            Self::ConfigureServo { .. } => "configure_servo",
            Self::ConfigureMotion { .. } => "configure_motion",
            Self::CalibrateStart { .. } => "calibrate_start",
            Self::CalibrateApply { .. } => "calibrate_apply",
            Self::TeachStart { .. } => "teach_start",
            Self::TeachMode { .. } => "teach_mode",
            Self::TeachFinish { .. } => "teach_finish",
            Self::TeachReplay { .. } | Self::TeachingPrepared { .. } => "teach_replay",
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

fn default_teaching_transition() -> f64 {
    0.4
}

pub use crate::teaching::Status as TeachingStatus;

#[derive(Clone, Serialize)]
pub struct TeachingModeStatus {
    pub request_id: String,
    pub state: String,
}

#[derive(Default)]
struct PendingRecords {
    calibrations: VecDeque<CalibrationReport>,
    teachings: VecDeque<Arc<TeachingCapture>>,
}

#[derive(Clone)]
pub struct ArmHandle {
    ordinary: Sender<Work>,
    priority: Sender<Work>,
    latest: Sender<Work>,
    latest_rx: Receiver<Work>,
    host_epoch: Arc<AtomicU64>,
    reference_epoch: Arc<AtomicU64>,
    stop_reason: Arc<Mutex<Option<String>>>,
    shutdown: Arc<AtomicBool>,
    finished: Receiver<PendingRecords>,
    journal: Journal,
    state: watch::Receiver<Snapshot>,
}

impl ArmHandle {
    pub fn spawn(journal: Journal) -> Result<Self, String> {
        Self::spawn_core(Core::new(journal))
    }
    fn spawn_core(core: Core) -> Result<Self, String> {
        let journal = core.journal.clone();
        let reference_epoch = core.reference_epoch.clone();
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
                let pending_report = run(
                    core,
                    ordinary_rx,
                    priority_rx,
                    worker_latest,
                    worker_epoch,
                    worker_reason,
                    worker_shutdown,
                    state_tx,
                );
                let _ = done_tx.try_send(pending_report);
            })
            .map_err(|e| e.to_string())?;
        Ok(Self {
            ordinary,
            priority,
            latest,
            latest_rx,
            host_epoch,
            reference_epoch,
            stop_reason,
            shutdown,
            finished,
            journal,
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
        self.reference_epoch.fetch_add(1, Ordering::AcqRel);
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
        let deadline = Instant::now() + timeout;
        self.request_shutdown();
        let Ok(pending_reports) = self.finished.recv_timeout(timeout) else {
            return false;
        };
        // Motion and device I/O have ended. Only this close coordinator may wait
        // for report admission, before the journal itself is asked to flush.
        for report in pending_reports.calibrations {
            loop {
                if self.journal.record_calibration(&report) {
                    break;
                }
                let remaining = deadline.saturating_duration_since(Instant::now());
                if remaining.is_zero() {
                    return false;
                }
                std::thread::sleep(remaining.min(Duration::from_millis(1)));
            }
        }
        for capture in pending_reports.teachings {
            loop {
                if self.journal.record_teaching(&capture) {
                    break;
                }
                let remaining = deadline.saturating_duration_since(Instant::now());
                if remaining.is_zero() {
                    return false;
                }
                std::thread::sleep(remaining.min(Duration::from_millis(1)));
            }
        }
        true
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
        if request.request_id.is_empty() {
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
    calibration_phase: Option<PlaybackPhase>,
    planned: Option<Arc<crate::calibration_path::Playback>>,
    calibration: bool,
    finishing: Option<HoldPose>,
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
    path_origin: Option<Pose>,
    target_pose: Option<Pose>,
    deadline: Option<Instant>,
    input_seq: u64,
}

#[derive(Debug)]
enum CommandOutcome {
    Accepted,
    Reported,
}

struct TeachingRun {
    id: String,
    request_id: String,
    started: Instant,
    recorder: TeachingRecorder,
    finish: Option<(Vec<String>, String)>,
}

struct TeachingModeRun {
    request_id: String,
    started: Instant,
    guidance: Guidance,
    finish: Option<(Vec<String>, String)>,
}

struct TeachingReplay {
    request_id: String,
    started: Instant,
    program: Arc<TeachingProgram>,
}

#[derive(Clone, Copy)]
enum CalibrationStep {
    Configure,
    Move,
    Moving,
    Identifying,
    Restore,
}
struct CalibrationPlan {
    phase: String,
    axis: usize,
    config: ServoTuning,
    cartesian: bool,
}
#[derive(Clone)]
enum CalibrationLeg {
    Joint(Leg),
    Planned {
        space: Space,
        axis: usize,
        speed: f64,
        leg: Arc<crate::calibration_spline::TimedLeg>,
    },
}
struct CalibrationRun {
    original_motion: MotionTuning,
    report: CalibrationReport,
    queue: Vec<CalibrationPlan>,
    legs: Vec<CalibrationLeg>,
    trial: usize,
    leg: usize,
    step: CalibrationStep,
    collectors: [Collector; 7],
    started: Instant,
    observed: Option<Instant>,
    feedback_wait: Option<Instant>,
    sent: VecDeque<SentReference>,
    identification: Option<Receiver<Vec<crate::friction::Identification>>>,
    observations: calibration::ObservationRecording,
    after_added: bool,
    proposed: Option<ServoTuning>,
    points: calibration::Recording<Point>,
    last_sample: Instant,
    cancelled: bool,
}
impl CalibrationRun {
    fn axis(&self) -> usize {
        if let Some(CalibrationLeg::Planned { axis, .. }) = self.legs.get(self.leg) {
            *axis
        } else {
            self.queue[self.trial.min(self.queue.len() - 1)].axis
        }
    }
    fn cartesian(&self) -> bool {
        self.queue[self.trial.min(self.queue.len() - 1)].cartesian
    }
    fn make_legs(&self) -> Vec<CalibrationLeg> {
        let settings = &self.report.settings;
        let axis = self.queue[self.trial].axis;
        let Some(program) = &settings.cartesian else {
            return calibration::legs(settings, axis, self.report.center_reference)
                .into_iter()
                .map(CalibrationLeg::Joint)
                .collect();
        };
        if self.cartesian() {
            let mut legs: Vec<_> = program
                .segments
                .iter()
                .zip(&program.timed)
                .map(|(segment, leg)| CalibrationLeg::Planned {
                    space: Space::Cartesian,
                    axis: segment.axis,
                    speed: program.speed,
                    leg: leg.clone(),
                })
                .collect();
            legs.push(CalibrationLeg::Joint(Leg {
                target: program.origin,
                speed: settings.speed,
            }));
            legs
        } else {
            program.joint_legs[axis]
                .iter()
                .enumerate()
                .map(|(i, leg)| CalibrationLeg::Planned {
                    space: Space::Joint,
                    axis,
                    speed: program.joint_speeds[i / 3],
                    leg: leg.clone(),
                })
                .collect()
        }
    }
}
fn calibration_reference(sent: &VecDeque<SentReference>, at: Instant) -> Option<SentReference> {
    sent.iter()
        .rev()
        .find(|sent| sent.completed_at <= at)
        .copied()
}
struct Core {
    journal: Journal,
    model: Option<Arc<Kinematics>>,
    reference_epoch: Arc<AtomicU64>,
    reference_version: u64,
    servo_tuning: ServoTuning,
    motion_tuning: MotionTuning,
    servo_tuning_revision: u64,
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
    pending_calibration_stop: Option<(u64, Arc<Playback>)>,
    tcp: Option<Pose>,
    enabled: bool,
    close_requested: bool,
    phase: Phase,
    lease: Option<Lease>,
    motion: Option<Motion>,
    calibration: Option<CalibrationRun>,
    calibration_result: Option<CalibrationStatus>,
    last_calibration_report: Option<Arc<CalibrationReport>>,
    pending_calibration_report: VecDeque<CalibrationReport>,
    teaching: Option<TeachingRun>,
    teaching_mode: Option<TeachingModeRun>,
    replay: Option<TeachingReplay>,
    last_teaching: Option<Arc<TeachingCapture>>,
    pending_teachings: VecDeque<Arc<TeachingCapture>>,
    /// A measured-position hold must not be integrated from an older command.
    absolute_hold: bool,
    teaching_stiffness: f64,
    stopping: bool,
    generation: u64,
    seq: u64,
    results: VecDeque<CommandResult>,
    seen: VecDeque<(u64, String)>,
    recording_ids: HashSet<String>,
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
            model: loaded.ok().map(Arc::new),
            reference_epoch: Arc::new(AtomicU64::new(0)),
            reference_version: 0,
            servo_tuning: ServoTuning::default(),
            motion_tuning: MotionTuning::default(),
            servo_tuning_revision: 1,
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
            pending_calibration_stop: None,
            tcp: None,
            enabled: false,
            close_requested: false,
            phase: Phase::Disconnected,
            lease: None,
            motion: None,
            calibration: None,
            calibration_result: None,
            last_calibration_report: None,
            pending_calibration_report: VecDeque::new(),
            teaching: None,
            teaching_mode: None,
            replay: None,
            last_teaching: None,
            pending_teachings: VecDeque::new(),
            absolute_hold: false,
            teaching_stiffness: 1.0,
            stopping: false,
            generation: 1,
            seq: 0,
            results: VecDeque::new(),
            seen: VecDeque::new(),
            recording_ids: HashSet::new(),
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
    fn invalidate_reference(&mut self) {
        self.reference_version = self
            .reference_epoch
            .fetch_add(1, Ordering::AcqRel)
            .wrapping_add(1);
    }
    fn bump_generation(&mut self) {
        self.generation += 1;
        self.invalidate_reference();
    }
    fn reset_servo_tuning(&mut self) {
        self.servo_tuning = ServoTuning::default();
        self.servo_tuning_revision += 1;
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
        } else if self.teaching_mode.is_some() {
            Phase::Teaching
        } else if self.motion.is_some() || self.calibration.is_some() || self.replay.is_some() {
            Phase::Moving
        } else if self.enabled {
            Phase::Ready
        } else {
            Phase::Disabled
        };
    }
    fn end_motion(&mut self, status: &str, code: &str, message: &str) {
        self.end_motion_at(status, code, message, self.control_at.max(Instant::now()));
    }
    fn stop_command(&mut self, code: &str, message: &str, now: Instant) {
        if self.motion.as_ref().is_some_and(|m| m.planned.is_some()) {
            self.drain_feedback(false);
        }
        self.end_motion_at("cancelled", code, message, now.max(Instant::now()));
    }
    fn end_motion_at(&mut self, status: &str, code: &str, message: &str, now: Instant) {
        let close_recording = matches!(
            code,
            "lease_expired" | "released" | "disconnected" | "closing"
        );
        let finishing_recording = self
            .teaching
            .as_ref()
            .is_some_and(|run| run.finish.is_some());
        if self.teaching_mode.is_some() || close_recording || finishing_recording {
            let at = self.control_at.max(Instant::now());
            let measured = self.measured(at).unwrap_or(self.reference);
            let outcome = if status == "failed" {
                "failed"
            } else {
                "stopped"
            };
            self.finish_teaching_mode_request(None, outcome, measured, at);
            if close_recording || finishing_recording {
                self.finish_teaching_request(None, outcome, measured, at);
            }
        }
        if let Some(replay) = self.replay.take() {
            // Brake from the last batch that actually reached the wire, including
            // the playback derivatives at that send time.
            if let Some(sent) = self.bus.as_ref().and_then(BusHandle::latest_reference) {
                self.reference = sent.q;
                self.velocity = sent.dq;
                self.acceleration = if sent.fallback_hold {
                    [0.0; 7]
                } else {
                    replay
                        .program
                        .sample(
                            sent.at
                                .saturating_duration_since(replay.started)
                                .as_secs_f64(),
                        )
                        .ddq
                };
            }
            self.absolute_hold = false;
            self.record(&replay.request_id, status, code, message);
        }
        if let Some(run) = self.calibration.as_mut()
            && !run.cancelled
        {
            run.cancelled = true;
            run.report.outcome = if status == "failed" {
                "failed"
            } else {
                "cancelled"
            }
            .into();
            run.report.message = message.into();
            run.report.recommended = None;
            run.step = CalibrationStep::Restore;
        }
        if let Some((id, _)) = self.enable_request.take() {
            self.enabled = false;
            self.reset_derivatives();
            self.record(&id, status, code, message);
        }
        if let Some(motion) = self.motion.take() {
            self.stop_path = Self::motion_path(&motion);
            if let Some(program) = motion.planned {
                self.pending_calibration_stop = Some((self.reference_version, program));
                self.absolute_hold = false;
            }
            if !motion.calibration {
                self.record(&motion.request_id, status, code, message);
            }
        }
        self.bump_generation();
        self.stopping = self.enabled
            && (!self.settle_calibration_stop(now)
                || self.velocity.iter().any(|v| v.abs() > 1e-12)
                || self.acceleration.iter().any(|a| a.abs() > 1e-12));
        if !self.stopping {
            self.stop_path = None;
        }
        self.phase();
        self.dirty = true;
    }
    fn settle_calibration_stop(&mut self, now: Instant) -> bool {
        use std::task::Poll;
        let Some((epoch, program)) = &self.pending_calibration_stop else {
            return true;
        };
        let receipt = self
            .bus
            .as_ref()
            .map_or(Poll::Ready(None), BusHandle::settled_reference);
        // A seven-packet write can finish after this control tick started.
        let checked_at = now.max(Instant::now());
        let checked_at = match receipt {
            Poll::Ready(Some(sent)) => checked_at.max(sent.completed_at),
            _ => checked_at,
        };
        // Latch at the request and on each pending handoff. New RX while a
        // write finishes must not restart braking from a stale velocity.
        if self
            .oldest_feedback()
            .is_none_or(|rx| checked_at.saturating_duration_since(rx) > MAX_REFERENCE_AGE)
            || self
                .calibration
                .as_ref()
                .is_some_and(|run| run.feedback_wait.is_some())
        {
            self.absolute_hold = true;
        }
        let Poll::Ready(sent) = receipt else {
            return false;
        };
        if let Some(sent) = sent {
            let phase = sent.calibration_phase.filter(|_| sent.epoch == *epoch);
            if phase.is_none_or(|p| p.progress(sent.at) < program.approach.duration) {
                self.stop_path = None;
            }
            self.reference = sent.q;
            self.velocity = sent.dq;
            self.acceleration = phase
                .filter(|_| !sent.fallback_hold)
                .map_or([0.0; 7], |p| p.sample(program, sent.at).ddq);
            if self.absolute_hold {
                // Stop may precede the first hold receipt. Pin the final in-flight
                // position without integrating its old velocity into a new batch.
                self.stop_path = None;
                self.velocity = [0.0; 7];
                self.acceleration = [0.0; 7];
            }
        } else {
            // Before the first servo receipt the plan starts at the held anchor.
            self.stop_path = None;
            self.reference = program.sample(0.0).q;
            self.velocity = [0.0; 7];
            self.acceleration = [0.0; 7];
        }
        self.pending_calibration_stop = None;
        true
    }
    fn fault(&mut self, code: &str, message: impl Into<String>) {
        let message = message.into();
        self.clear_pending = [false; 7];
        if let Some((id, _)) = self.clear_request.take() {
            self.record(&id, "failed", code, &message);
        }
        let changed = self
            .error
            .as_ref()
            .is_none_or(|e| e.code != code || e.message != message);
        if self.motion.is_some()
            || self.enable_request.is_some()
            || self.calibration.is_some()
            || self.teaching.is_some()
            || self.teaching_mode.is_some()
            || self.replay.is_some()
        {
            self.end_motion("failed", code, &message);
        } else if changed {
            self.bump_generation();
        }
        self.enabled = false;
        self.finish_teaching_disabled();
        self.reset_derivatives();
        self.stopping = false;
        self.error = Some(ApiError::new(code, message.clone()));
        self.restore_calibration_disabled();
        self.phase = Phase::Fault;
        if changed {
            self.journal.log("error", "device", code, &message, None);
            self.dirty = true;
        }
    }
    fn measured(&self, now: Instant) -> Result<[f64; 7], ApiError> {
        let mut q = [0.0; 7];
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
            q[i] = sample.value.q;
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
        if request.client_id.is_empty() {
            return Err(ApiError::new("invalid_client", "客户端标识无效"));
        }
        let secret = match request.action.as_str() {
            "acquire" => {
                if self.lease.is_some() {
                    return Err(ApiError::new("owned", "控制权已被占用"));
                }
                self.recording_ids.clear();
                let secret = uuid::Uuid::new_v4().to_string();
                self.lease = Some(Lease {
                    client_id: request.client_id,
                    secret: secret.clone(),
                    expires: now + Duration::from_millis(LEASE_MS),
                });
                self.bump_generation();
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
        let recording_command = matches!(
            request.operation,
            Operation::TeachStart { .. } | Operation::TeachFinish { .. }
        );
        if !recording_command && request.generation != self.generation {
            self.record(&id, "rejected", "stale_context", "操作上下文已改变");
            return;
        }
        if recording_command {
            if !self.recording_ids.insert(id.clone()) {
                return;
            }
        } else {
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
        }
        let operation = request.operation.name();
        let outcome = self.apply(&id, request.operation, received, now);
        match outcome {
            Ok(CommandOutcome::Accepted) => {
                if operation != "jog_update" {
                    self.record(&id, "accepted", operation, "操作已接受");
                }
            }
            Ok(CommandOutcome::Reported) => {}
            Err(error) => self.record(&id, "rejected", &error.code, error.message),
        }
    }
    fn apply(
        &mut self,
        id: &str,
        operation: Operation,
        received: Instant,
        now: Instant,
    ) -> Result<CommandOutcome, ApiError> {
        let recording_operation = matches!(
            operation,
            Operation::TeachStart { .. } | Operation::TeachFinish { .. }
        );
        let hold_operation = matches!(operation, Operation::TeachMode { enabled: false, .. });
        if self.teaching_mode.is_some()
            && !matches!(
                operation,
                Operation::TeachMode { .. }
                    | Operation::TeachStart { .. }
                    | Operation::TeachFinish { .. }
                    | Operation::Stop
                    | Operation::Disable { .. }
                    | Operation::Disconnect { .. }
            )
        {
            return Err(ApiError::new("motion_busy", "请先结束示教"));
        }
        if self.replay.is_some()
            && !recording_operation
            && !hold_operation
            && !matches!(
                operation,
                Operation::Stop | Operation::Disable { .. } | Operation::Disconnect { .. }
            )
        {
            return Err(ApiError::new("motion_busy", "轨迹正在重放"));
        }
        if self.calibration.is_some()
            && !recording_operation
            && !hold_operation
            && !matches!(
                operation,
                Operation::Stop | Operation::Disable { .. } | Operation::Disconnect { .. }
            )
        {
            return Err(ApiError::new("calibration_busy", "校准正在进行"));
        }
        match operation {
            Operation::Stop => self.stop_command("stopped", "动作已取消", now),
            Operation::Connect { port, side } => {
                if self.bus.is_some() {
                    return Err(ApiError::new("connected", "请先断开当前连接"));
                }
                if side != "left" && side != "right" || port.is_empty() {
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
                    self.reference_epoch.clone(),
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
                self.model = Some(Arc::new(model));
                self.calibration_result = None;
                self.reset_servo_tuning();
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
                self.bump_generation();
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
                self.drain_feedback(true);
                let now = now.max(Instant::now());
                if self.error.is_some()
                    || self.feedback.iter().any(|o| {
                        o.as_ref()
                            .is_some_and(|o| o.value.error != 0 || o.value.timeout)
                    })
                {
                    return Err(ApiError::new("fault", "请先清除设备故障"));
                }
                self.reference = self.measured(now)?;
                self.check_positions(&self.reference)?;
                self.reset_derivatives();
                self.absolute_hold = true;
                self.enable_request = Some((id.into(), now));
                self.stopping = false;
                self.phase = Phase::Enabling;
                self.bump_generation();
            }
            Operation::Disable { allow_release } => {
                self.require_release(allow_release)?;
                self.end_motion("cancelled", "disabled", "设备已失能");
                self.enabled = false;
                self.finish_teaching_disabled();
                self.restore_calibration_disabled();
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
                let norm = tcp.rotation.iter().fold(0.0_f64, |norm, q| norm.hypot(*q));
                if tcp.translation.iter().any(|q| !q.is_finite())
                    || !norm.is_finite()
                    || norm == 0.0
                {
                    return Err(ApiError::new("invalid_tcp", "TCP 参数无效"));
                }
                tcp.rotation.iter_mut().for_each(|q| *q /= norm);
                self.tcp = Some(tcp);
                self.bump_generation();
            }
            Operation::ConfigureServo { config } => {
                if self.bus.is_none()
                    || matches!(
                        self.phase,
                        Phase::Disconnected | Phase::Connecting | Phase::Closing
                    )
                {
                    return Err(ApiError::new("not_connected", "请先连接机械臂"));
                }
                if self.error.is_some() || self.clear_request.is_some() {
                    return Err(ApiError::new("fault", "请先清除当前故障"));
                }
                if self.motion.is_some() || self.stopping || self.enable_request.is_some() {
                    return Err(ApiError::new("motion_busy", "当前动作尚未结束"));
                }
                config
                    .validate()
                    .map_err(|e| ApiError::new("invalid_servo_tuning", e))?;
                if self.enabled {
                    let held = self.motion_anchor(now)?;
                    self.reference = held;
                    self.reset_derivatives();
                }
                self.servo_tuning = *config;
                self.servo_tuning_revision += 1;
                self.bump_generation();
                self.journal.log(
                    "info",
                    "control",
                    "servo_tuning",
                    &format!(
                        "伺服参数已应用，版本 {}，摩擦补偿{}",
                        self.servo_tuning_revision,
                        if config.friction_enabled {
                            "开启"
                        } else {
                            "关闭"
                        }
                    ),
                    Some(id),
                );
            }
            Operation::ConfigureMotion { config } => {
                config
                    .validate()
                    .map_err(|e| ApiError::new("invalid_motion", e))?;
                if self.motion.is_some() || self.stopping || self.enable_request.is_some() {
                    return Err(ApiError::new("motion_busy", "请先结束当前动作"));
                }
                self.motion_tuning = config;
                self.bump_generation();
            }
            Operation::CalibrateStart { settings } => self.begin_calibration(id, settings, now)?,
            Operation::CalibrateApply { calibration_id } => {
                self.apply_calibration(&calibration_id, now)?
            }
            Operation::TeachStart { name } => self.begin_teaching(id, name, now)?,
            Operation::TeachMode {
                enabled,
                transition_s,
            } => {
                return self.set_teaching_mode(id, enabled, transition_s, now);
            }
            Operation::TeachFinish {
                recording_request_id,
            } => {
                if let Some(target) = recording_request_id {
                    let cancelled_start = self.recording_ids.insert(target.clone());
                    if self
                        .teaching
                        .as_ref()
                        .is_none_or(|run| run.request_id != target)
                    {
                        if cancelled_start {
                            self.record(
                                &target,
                                "cancelled",
                                "recording_cancelled",
                                "已取消开始录制",
                            );
                        }
                        self.record(id, "completed", "recording_ended", "录制已结束");
                        return Ok(CommandOutcome::Reported);
                    }
                }
                let measured = self.measured(now).unwrap_or(self.reference);
                if self.teaching.is_none() {
                    return Err(ApiError::new("no_teaching", "录制已结束"));
                }
                self.finish_teaching_request(Some(id.into()), "completed", measured, now);
            }
            Operation::TeachReplay { .. } => {
                return Err(ApiError::new("invalid_trajectory", "轨迹尚未准备"));
            }
            Operation::TeachingPrepared { mut program } => {
                let anchor = self.motion_anchor(now)?;
                if program.side() != self.side {
                    return Err(ApiError::new(
                        "invalid_trajectory",
                        "轨迹与当前机械臂不一致",
                    ));
                }
                Arc::make_mut(&mut program)
                    .rebase_approach(anchor)
                    .map_err(|e| ApiError::new("invalid_trajectory", e))?;
                self.reference = anchor;
                self.reset_derivatives();
                self.absolute_hold = false;
                self.replay = Some(TeachingReplay {
                    request_id: id.into(),
                    started: now,
                    program,
                });
                self.bump_generation();
                self.phase();
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
                if !amount.is_finite() || amount <= 0.0 {
                    return Err(ApiError::new("invalid_step", "步长必须为正数"));
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
        Ok(CommandOutcome::Accepted)
    }

    fn begin_teaching(&mut self, id: &str, name: String, now: Instant) -> Result<(), ApiError> {
        if self.teaching.is_some() {
            return Err(ApiError::new("teaching_active", "正在录制轨迹"));
        }
        let measured = self.measured(now)?;
        let trajectory_id = uuid::Uuid::new_v4().to_string();
        let summary = TeachingSummary {
            id: trajectory_id.clone(),
            name: if name.trim().is_empty() {
                "示教轨迹".into()
            } else {
                name
            },
            created_at_ms: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis()
                .min(i64::MAX as u128) as i64,
            side: self.side.clone(),
            tcp: self.tcp.unwrap_or(Pose {
                translation: [0.0; 3],
                rotation: [0.0, 0.0, 0.0, 1.0],
            }),
            duration_ms: 0,
            sample_count: 0,
            outcome: "recording".into(),
        };
        let mut recorder = TeachingRecorder::new(summary);
        recorder.push(teaching::Point {
            t_ms: 0,
            q: measured,
        });
        self.teaching = Some(TeachingRun {
            id: trajectory_id,
            request_id: id.into(),
            started: now,
            recorder,
            finish: None,
        });
        // Recording observes feedback; it never replaces the active reference.
        self.dirty = true;
        Ok(())
    }

    fn set_teaching_mode(
        &mut self,
        id: &str,
        enabled: bool,
        transition_s: f64,
        now: Instant,
    ) -> Result<CommandOutcome, ApiError> {
        if !enabled {
            let measured = self.measured(now).unwrap_or(self.reference);
            if self.teaching_mode.is_some() {
                self.finish_teaching_mode_request(Some(id.into()), "completed", measured, now);
            } else {
                self.record(id, "completed", "teaching_mode", "已保持当前位置");
                return Ok(CommandOutcome::Reported);
            }
            return Ok(CommandOutcome::Accepted);
        }
        let measured = self.measured(now)?;
        if !transition_s.is_finite() || transition_s <= 0.0 {
            return Err(ApiError::new("invalid_teaching", "切换时间必须为正数"));
        }
        if let Some(run) = &mut self.teaching_mode {
            if let Some((waiting, _)) = run.finish.take() {
                run.guidance.set_drag(
                    true,
                    measured,
                    now.saturating_duration_since(run.started).as_secs_f64(),
                );
                for request in waiting {
                    self.record(&request, "cancelled", "teaching_mode", "已继续示教");
                }
                self.dirty = true;
            }
            return Ok(CommandOutcome::Accepted);
        }
        let anchor = self.motion_anchor(now)?;
        let mut guidance = Guidance::new(anchor, transition_s);
        guidance.set_drag(true, measured, 0.0);
        self.teaching_mode = Some(TeachingModeRun {
            request_id: id.into(),
            started: now,
            guidance,
            finish: None,
        });
        self.reference = anchor;
        self.reset_derivatives();
        self.absolute_hold = true;
        self.teaching_stiffness = 1.0;
        self.bump_generation();
        self.phase();
        self.dirty = true;
        Ok(CommandOutcome::Accepted)
    }

    fn finish_teaching_mode_request(
        &mut self,
        request_id: Option<String>,
        outcome: &str,
        measured: [f64; 7],
        now: Instant,
    ) {
        if let Some(run) = self.teaching_mode.as_mut() {
            if let Some((waiting, previous_outcome)) = run.finish.as_mut() {
                if let Some(id) = request_id {
                    waiting.push(id);
                }
                if outcome == "failed"
                    || (*previous_outcome == "completed" && outcome != "completed")
                {
                    *previous_outcome = outcome.into();
                }
            } else {
                run.guidance.set_drag(
                    false,
                    measured,
                    now.saturating_duration_since(run.started).as_secs_f64(),
                );
                run.finish = Some((request_id.into_iter().collect(), outcome.into()));
            }
            self.dirty = true;
        }
    }

    fn finish_teaching_request(
        &mut self,
        request_id: Option<String>,
        outcome: &str,
        measured: [f64; 7],
        now: Instant,
    ) {
        if let Some(run) = self.teaching.as_mut() {
            if let Some((waiting, previous_outcome)) = run.finish.as_mut() {
                if let Some(request_id) = request_id {
                    waiting.push(request_id);
                }
                if outcome == "failed"
                    || (*previous_outcome == "completed" && outcome != "completed")
                {
                    *previous_outcome = outcome.into();
                }
            } else {
                run.finish = Some((request_id.into_iter().collect(), outcome.into()));
            }
            self.finish_teaching_mode_request(None, outcome, measured, now);
            self.dirty = true;
        }
    }

    fn save_teaching(&mut self, run: TeachingRun) {
        let (waiting, outcome) = run.finish.unwrap_or((Vec::new(), "stopped".into()));
        let capture = Arc::new(run.recorder.finish(&outcome));
        self.last_teaching = Some(capture.clone());
        self.pending_teachings.push_back(capture);
        self.flush_teaching_records();
        let status = if outcome == "completed" {
            "completed"
        } else if outcome == "failed" {
            "failed"
        } else {
            "cancelled"
        };
        let message = if status == "completed" {
            "示教轨迹已保存"
        } else {
            "录制已结束，轨迹已保存"
        };
        self.record(&run.request_id, status, "teaching_saved", message);
        for request in waiting {
            self.record(&request, status, "teaching_saved", message);
        }
    }

    fn complete_teaching_mode(&mut self) {
        if let Some(run) = self.teaching_mode.take() {
            let (waiting, outcome) = run.finish.unwrap_or((Vec::new(), "stopped".into()));
            let status = if outcome == "completed" {
                "completed"
            } else if outcome == "failed" {
                "failed"
            } else {
                "cancelled"
            };
            self.record(&run.request_id, status, "teaching_mode", "示教模式已结束");
            for request in waiting {
                self.record(
                    &request,
                    status,
                    "teaching_mode",
                    if status == "completed" {
                        "保持当前位置"
                    } else {
                        "示教模式已结束"
                    },
                );
            }
            self.teaching_stiffness = 1.0;
            self.bump_generation();
            self.phase();
        }
    }

    fn finish_teaching_disabled(&mut self) {
        self.complete_teaching_mode();
        if self
            .teaching
            .as_ref()
            .is_some_and(|run| run.finish.is_some())
        {
            let run = self.teaching.take().unwrap();
            self.save_teaching(run);
        }
        self.absolute_hold = false;
        self.teaching_stiffness = 1.0;
    }

    fn flush_teaching_records(&mut self) {
        if self
            .pending_teachings
            .front()
            .is_some_and(|capture| self.journal.record_teaching(capture))
        {
            self.pending_teachings.pop_front();
        }
    }

    fn advance_teaching_mode(&mut self, measured: [f64; 7], now: Instant) {
        let Some(run) = self.teaching_mode.as_mut() else {
            return;
        };
        let sample = run.guidance.sample(
            now.saturating_duration_since(run.started).as_secs_f64(),
            measured,
        );
        self.reference = sample.q;
        self.teaching_stiffness = sample.stiffness;
        self.velocity = [0.0; 7];
        self.acceleration = [0.0; 7];
        self.jerk = [0.0; 7];
        if run.finish.is_some() && sample.settled && !sample.dragging {
            self.complete_teaching_mode();
        }
        self.dirty = true;
    }

    fn advance_teaching_recording(&mut self, now: Instant) {
        let measured = self.measured(now).ok();
        let Some(run) = self.teaching.as_mut() else {
            return;
        };
        if let Some(q) = measured {
            run.recorder.push(teaching::Point {
                t_ms: now
                    .saturating_duration_since(run.started)
                    .as_millis()
                    .min(u64::MAX as u128) as u64,
                q,
            });
        }
        let waiting_for_hold = self
            .teaching_mode
            .as_ref()
            .is_some_and(|mode| mode.finish.is_some());
        if run.finish.is_some() && !waiting_for_hold {
            let run = self.teaching.take().unwrap();
            self.save_teaching(run);
        }
        self.dirty = true;
    }

    fn teaching_mode_status(&self, now: Instant) -> Option<TeachingModeStatus> {
        self.teaching_mode.as_ref().map(|run| {
            let measured = std::array::from_fn(|i| {
                self.feedback[i]
                    .as_ref()
                    .map_or(self.reference[i], |s| s.value.q)
            });
            let sample = run.guidance.sample(
                now.saturating_duration_since(run.started).as_secs_f64(),
                measured,
            );
            TeachingModeStatus {
                request_id: run.request_id.clone(),
                state: if !sample.settled {
                    if sample.dragging {
                        "entering"
                    } else {
                        "leaving"
                    }
                } else if sample.dragging {
                    "dragging"
                } else {
                    "holding"
                }
                .into(),
            }
        })
    }

    fn teaching_recording_status(&self) -> Option<TeachingStatus> {
        self.teaching.as_ref().map(|run| TeachingStatus {
            id: run.id.clone(),
            request_id: run.request_id.clone(),
            state: if run.finish.is_some() {
                "finishing"
            } else {
                "recording"
            }
            .into(),
            recording: true,
            sample_count: run.recorder.sample_count(),
            duration_ms: run.recorder.duration_ms(),
            progress: 0.0,
            trajectory_id: Some(run.id.clone()),
        })
    }

    fn teaching_status(&self, now: Instant) -> Option<TeachingStatus> {
        self.replay.as_ref().map(|run| {
            let elapsed = now.saturating_duration_since(run.started).as_secs_f64();
            TeachingStatus {
                id: run.program.trajectory_id().into(),
                request_id: run.request_id.clone(),
                state: if elapsed < run.program.approach_duration() {
                    "approaching"
                } else {
                    "replaying"
                }
                .into(),
                recording: false,
                sample_count: 0,
                duration_ms: (run.program.duration() * 1000.0) as u64,
                progress: if run.program.duration() == 0.0 {
                    1.0
                } else {
                    (elapsed / run.program.duration()).clamp(0.0, 1.0)
                },
                trajectory_id: Some(run.program.trajectory_id().into()),
            }
        })
    }
    fn begin_calibration(
        &mut self,
        id: &str,
        mut settings: CalibrationSettings,
        now: Instant,
    ) -> Result<(), ApiError> {
        self.flush_calibration_report();
        let anchor = self.motion_anchor(now)?;
        let measured = self.measured(now)?;
        if let Some(program) = &settings.cartesian {
            self.model
                .as_ref()
                .unwrap()
                .fk(&program.origin, &program.tcp)
                .map_err(|e| ApiError::new("invalid_calibration", e))?;
            if program.side != self.side
                || self.tcp.is_none_or(|tcp| {
                    kinematics::pose_error(&tcp, &program.tcp)
                        .iter()
                        .any(|v| v.abs() > 1e-6)
                })
                || (0..7).any(|i| (program.origin[i] - anchor[i]).abs() > 0.03)
            {
                return Err(ApiError::new(
                    "stale_preview",
                    "姿态或 TCP 已变化，请更新轨迹预览",
                ));
            }
            if program.timed.len() != program.segments.len()
                || program.joint_legs.iter().any(Vec::is_empty)
            {
                return Err(ApiError::new("stale_preview", "请重新生成校准轨迹预览"));
            }
            let requested_speeds = !settings.speeds.is_empty()
                || settings.speed != CalibrationSettings::default().speed;
            if requested_speeds && calibration::speeds(&settings) != program.joint_speeds {
                return Err(ApiError::new(
                    "stale_preview",
                    "关节速度已变化，请重新生成预览",
                ));
            }
            settings.speeds = program.joint_speeds.clone();
            settings.excursion = program.joint_excursion;
            settings.axis = None;
        }
        calibration::validate(&settings, &self.servo_tuning)
            .map_err(|e| ApiError::new("invalid_calibration", e))?;
        let original_motion = self.motion_tuning;
        let calibration_motion = settings.cartesian.as_ref().map(|program| program.motion);
        if let Some(config) = calibration_motion {
            config
                .validate()
                .map_err(|e| ApiError::new("invalid_motion", e))?;
        }
        let axes = settings
            .axis
            .map_or_else(|| (0..7).collect::<Vec<_>>(), |axis| vec![axis]);
        let mut queue: Vec<_> = axes
            .iter()
            .map(|axis| CalibrationPlan {
                phase: "baseline".into(),
                axis: *axis,
                config: self.servo_tuning,
                cartesian: false,
            })
            .collect();
        let first = axes[0];
        if settings
            .cartesian
            .as_ref()
            .is_some_and(|p| !p.segments.is_empty())
        {
            queue.insert(
                0,
                CalibrationPlan {
                    phase: "cartesian_before".into(),
                    axis: 0,
                    config: self.servo_tuning,
                    cartesian: true,
                },
            );
        }
        let trial_count = if queue.iter().any(|trial| trial.cartesian) {
            axes.len() * 2 + 2
        } else {
            axes.len() * 3
        };
        let observation_groups = axes.len() * calibration::legs(&settings, first, anchor).len();
        // Recording memory is bounded independently of movement duration.
        // Every trial/leg receives a share, including the final comparison.
        let recording_budget = 90_000;
        self.absolute_hold = false;
        self.calibration = Some(CalibrationRun {
            original_motion,
            report: CalibrationReport {
                id: uuid::Uuid::new_v4().to_string(),
                request_id: id.into(),
                created_at_ms: SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_millis()
                    .min(i64::MAX as u128) as i64,
                side: self.side.clone(),
                port: self
                    .port
                    .clone()
                    .ok_or_else(|| ApiError::new("not_connected", "请先连接机械臂"))?,
                settings: settings.clone(),
                center_reference: anchor,
                center_measured: measured,
                tcp: self.tcp,
                baseline: self.servo_tuning,
                recommended: None,
                trials: Vec::with_capacity(20),
                choices: Vec::new(),
                sample_count: 0,
                captured_samples: 0,
                curves: Arc::default(),
                identification: Vec::new(),
                outcome: "unchanged".into(),
                improvement: None,
                message: "测试完成，参数保持原样".into(),
            },
            queue,
            legs: Vec::new(),
            trial: 0,
            leg: 0,
            step: CalibrationStep::Configure,
            collectors: std::array::from_fn(|axis| Collector::new(axis, measured)),
            started: now,
            observed: None,
            feedback_wait: None,
            sent: VecDeque::with_capacity(32),
            identification: None,
            observations: calibration::ObservationRecording::new(
                recording_budget,
                observation_groups,
            ),
            after_added: false,
            proposed: None,
            points: calibration::Recording::new(recording_budget, trial_count),
            last_sample: now,
            cancelled: false,
        });
        self.calibration_result = None;
        if let Some(config) = calibration_motion {
            self.motion_tuning = config;
        }
        self.phase();
        self.dirty = true;
        Ok(())
    }
    fn calibration_snapshot(&self, run: &CalibrationRun) -> CalibrationStatus {
        let restoring = matches!(run.step, CalibrationStep::Restore);
        let axes = if run.report.settings.axis.is_some() {
            1
        } else {
            7
        };
        let total = if run.after_added {
            run.queue.len()
        } else {
            (if run
                .report
                .settings
                .cartesian
                .as_ref()
                .is_some_and(|p| !p.segments.is_empty())
            {
                axes * 2 + 2
            } else {
                axes * 3
            })
            .max(run.queue.len())
        };
        let phase = &run.queue[run.trial.min(run.queue.len() - 1)].phase;
        CalibrationStatus {
            id: run.report.id.clone(),
            request_id: run.report.request_id.clone(),
            state: if restoring { "stopping" } else { "running" }.into(),
            stage: if restoring {
                "restoring"
            } else if matches!(run.step, CalibrationStep::Identifying) {
                "identifying"
            } else if phase == "after" {
                "validation"
            } else {
                phase
            }
            .into(),
            axis: run.axis(),
            trial_index: run.trial.min(total),
            trial_count: total,
            leg_index: run.leg.min(run.legs.len()),
            leg_count: run.legs.len(),
            progress: ((run.trial as f64 + run.leg as f64 / run.legs.len().max(1) as f64)
                / total.max(1) as f64)
                .min(0.99),
            report: None,
            live: run.points.last().cloned(),
            space: if run.cartesian() {
                "cartesian"
            } else {
                "joint"
            }
            .into(),
        }
    }
    fn finish_calibration(&mut self, mut run: CalibrationRun) {
        let axis = run.axis();
        self.motion_tuning = run.original_motion;
        if let Some(trial) = run.report.trials.get_mut(run.trial) {
            if trial.space == "cartesian" && trial.metrics_by_joint.is_none() {
                trial.metrics_by_joint =
                    Some(run.collectors.iter().map(Collector::finish).collect());
            } else if trial.metrics.is_none()
                && let Some(axis) = trial.axis
            {
                trial.metrics = run.collectors[axis].finish();
            }
        }
        run.report.sample_count = run.points.len();
        run.report.captured_samples = run.points.captured();
        run.report.curves = Arc::new(run.points.into_samples());
        let report = run.report;
        self.last_calibration_report = Some(Arc::new(report.clone()));
        let state = match report.outcome.as_str() {
            "cancelled" => "cancelled",
            "failed" => "failed",
            _ => "completed",
        };
        self.record(
            &report.request_id,
            state,
            "calibration_done",
            &report.message,
        );
        self.calibration_result = Some(CalibrationStatus {
            id: report.id.clone(),
            request_id: report.request_id.clone(),
            state: state.into(),
            stage: "done".into(),
            axis,
            trial_index: report.trials.len(),
            trial_count: report.trials.len(),
            leg_index: run.leg.min(run.legs.len()),
            leg_count: run.legs.len(),
            progress: 1.0,
            report: Some(report.summary()),
            live: None,
            space: "joint".into(),
        });
        // Normally the previous report was admitted during the next control tick.
        // Keep a completed report available in the UI even if storage is unavailable.
        self.flush_calibration_report();
        if self.pending_calibration_report.len() == 100 {
            self.pending_calibration_report.pop_front();
        }
        self.pending_calibration_report.push_back(report);
        self.flush_calibration_report();
        self.phase();
        self.dirty = true;
    }
    fn flush_calibration_report(&mut self) {
        if self
            .pending_calibration_report
            .front()
            .is_some_and(|report| self.journal.record_calibration(report))
        {
            self.pending_calibration_report.pop_front();
        }
    }
    fn restore_calibration_disabled(&mut self) {
        let Some(run) = self.calibration.take() else {
            return;
        };
        if self.servo_tuning != run.report.baseline {
            self.servo_tuning = run.report.baseline;
            self.servo_tuning_revision += 1;
            self.bump_generation();
        }
        self.finish_calibration(run);
    }
    fn start_friction_identification(&self, run: &mut CalibrationRun) {
        let data = std::mem::replace(
            &mut run.observations,
            calibration::ObservationRecording::new(0, 1),
        );
        let model = self.model.as_ref().unwrap().clone();
        let settings = run.report.settings.clone();
        let baseline = run.report.baseline;
        let (sender, receiver) = bounded(1);
        match std::thread::Builder::new()
            .name("friction-identification".into())
            .spawn(move || {
                let identified =
                    calibration::identify(&data.into_samples(), &model, &settings, &baseline);
                // Stopping drops the receiver. A late analysis result cannot
                // change the arm's configuration or restart its program.
                let _ = sender.try_send(identified);
            }) {
            Ok(_) => {
                run.identification = Some(receiver);
                run.step = CalibrationStep::Identifying;
            }
            Err(error) => {
                self.journal.log(
                    "error",
                    "calibration",
                    "analysis_start",
                    &error.to_string(),
                    None,
                );
                Self::extend_calibration_plan(run);
            }
        }
    }
    fn extend_calibration_plan(run: &mut CalibrationRun) {
        if !run.after_added {
            run.after_added = true;
            let mut proposed = run.report.baseline;
            for axis in 0..7 {
                let Some(baseline) = run
                    .report
                    .trials
                    .iter()
                    .find(|t| t.phase == "baseline" && t.axis == Some(axis))
                else {
                    continue;
                };
                let candidates = run
                    .report
                    .trials
                    .iter()
                    .filter(|t| t.phase == "candidate" && t.axis == Some(axis))
                    .cloned()
                    .collect::<Vec<_>>();
                let Some(best) = calibration::recommend(baseline, &candidates) else {
                    continue;
                };
                let selected = &run.report.trials[best];
                if selected.config.friction_enabled && !proposed.friction_enabled {
                    for joint in &mut proposed.joints {
                        joint.coulomb = 0.0;
                        joint.viscous = 0.0;
                        joint.stribeck.enabled = false;
                    }
                    proposed.friction_enabled = true;
                }
                proposed.joints[axis] = selected.config.joints[axis];
                if proposed.friction_enabled && !selected.config.friction_enabled {
                    proposed.joints[axis].coulomb = 0.0;
                    proposed.joints[axis].viscous = 0.0;
                    proposed.joints[axis].stribeck.enabled = false;
                }
                let before = baseline.metrics.as_ref().unwrap();
                let after = selected.metrics.as_ref().unwrap();
                run.report.choices.push(Choice {
                    axis,
                    baseline_index: baseline.index,
                    candidate_index: best,
                    after_index: None,
                    velocity_change: if before.velocity_rms > 0.0 {
                        1.0 - after.velocity_rms / before.velocity_rms
                    } else {
                        0.0
                    },
                    tracking_change: if before.tracking_rms > 0.0 {
                        1.0 - after.tracking_rms / before.tracking_rms
                    } else {
                        0.0
                    },
                });
            }
            if !run.report.choices.is_empty() {
                run.proposed = Some(proposed);
            }
            if run
                .report
                .settings
                .cartesian
                .as_ref()
                .is_some_and(|p| !p.segments.is_empty())
            {
                run.queue.push(CalibrationPlan {
                    phase: "cartesian_after".into(),
                    axis: 0,
                    config: run.proposed.unwrap_or(run.report.baseline),
                    cartesian: true,
                });
                return;
            }
            if run.proposed.is_some() {
                for trial in run.report.trials.iter().filter(|t| t.phase == "baseline") {
                    run.queue.push(CalibrationPlan {
                        phase: "after".into(),
                        axis: trial.axis.unwrap_or(0),
                        config: proposed,
                        cartesian: false,
                    });
                }
                return;
            }
        }
        let mut before_total = 0.0;
        let mut after_total = 0.0;
        if let (Some(before), Some(after)) = (
            run.report
                .trials
                .iter()
                .find(|t| t.phase == "cartesian_before")
                .and_then(|t| t.metrics_by_joint.as_ref()),
            run.report
                .trials
                .iter()
                .find(|t| t.phase == "cartesian_after")
                .and_then(|t| t.metrics_by_joint.as_ref()),
        ) {
            for (b, a) in before.iter().zip(after) {
                if let (Some(b), Some(a)) = (b, a) {
                    before_total += calibration::score(b, b);
                    after_total += calibration::score(b, a);
                }
            }
        }
        for after in run.report.trials.iter().filter(|t| t.phase == "after") {
            if let Some(before) = run
                .report
                .trials
                .iter()
                .find(|t| t.phase == "baseline" && t.axis == after.axis)
                && let (Some(b), Some(a)) = (&before.metrics, &after.metrics)
            {
                before_total += calibration::score(b, b);
                after_total += calibration::score(b, a);
            }
        }
        for choice in &mut run.report.choices {
            if let Some(after) = run
                .report
                .trials
                .iter()
                .find(|t| t.phase == "after" && t.axis == Some(choice.axis))
            {
                choice.after_index = Some(after.index);
                if let (Some(b), Some(a)) = (
                    &run.report.trials[choice.baseline_index].metrics,
                    &after.metrics,
                ) {
                    choice.velocity_change = if b.velocity_rms > 0.0 {
                        1.0 - a.velocity_rms / b.velocity_rms
                    } else {
                        0.0
                    };
                    choice.tracking_change = if b.tracking_rms > 0.0 {
                        1.0 - a.tracking_rms / b.tracking_rms
                    } else {
                        0.0
                    };
                }
            }
        }
        if run.proposed.is_some() && before_total > 0.0 && after_total < before_total {
            run.report.recommended = run.proposed;
            run.report.improvement = Some((before_total - after_total) / before_total);
            run.report.outcome = "improved".into();
            run.report.message = "测试完成，可使用对照参数".into();
        } else {
            run.report.outcome = "unchanged".into();
            run.report.message = "测试完成，参数保持原样".into();
        }
        run.step = CalibrationStep::Restore;
    }
    fn advance_calibration(&mut self, now: Instant) {
        let Some(mut run) = self.calibration.take() else {
            return;
        };
        if !self.enabled || self.error.is_some() {
            run.cancelled = true;
            run.report.outcome = "failed".into();
            run.report.recommended = None;
            run.report.message = self
                .error
                .as_ref()
                .map_or("测试已结束".into(), |e| e.message.clone());
            self.calibration = Some(run);
            self.restore_calibration_disabled();
            return;
        }
        let latest = self.bus.as_ref().and_then(|bus| bus.latest_reference());
        let sent_phase = latest
            .filter(|s| s.epoch == self.reference_version)
            .and_then(|s| s.calibration_phase);
        let waiting_for_motion_feedback = match sent_phase {
            Some(PlaybackPhase::Held { .. }) => true,
            Some(PlaybackPhase::Resuming { started, .. }) => {
                self.oldest_feedback().is_none_or(|rx| rx < started)
            }
            _ => false,
        };
        if !run.cancelled && (run.feedback_wait.is_some() || waiting_for_motion_feedback) {
            self.calibration = Some(run);
            return;
        }
        if let Some(sent) = latest
            && run.sent.back().is_none_or(|last| sent.at > last.at)
        {
            if run.sent.len() == 32 {
                run.sent.pop_front();
            }
            run.sent.push_back(sent);
        }
        if run.feedback_wait.is_none()
            && !run.report.trials.is_empty()
            && !matches!(run.step, CalibrationStep::Identifying)
            && now.saturating_duration_since(run.last_sample) >= Duration::from_millis(20)
            && let Ok(feedback) = self.measured(now)
        {
            run.last_sample = now;
            run.points.push(
                (run.trial.min(run.report.trials.len() - 1), 0),
                Point {
                    t_ms: now
                        .saturating_duration_since(run.started)
                        .as_millis()
                        .min(u32::MAX as u128) as u32,
                    trial: run.trial.min(run.report.trials.len() - 1),
                    axis: run.report.trials[run.trial.min(run.report.trials.len() - 1)]
                        .axis
                        .unwrap_or(run.axis()),
                    reference: self.reference,
                    command: latest.map(|sent| sent.q),
                    feedback,
                    velocity: latest.map_or(self.velocity, |sent| sent.dq),
                    feedback_velocity: std::array::from_fn(|i| {
                        self.feedback[i].as_ref().unwrap().value.dq
                    }),
                    feedback_age_ms: std::array::from_fn(|i| {
                        now.saturating_duration_since(self.feedback[i].as_ref().unwrap().at)
                            .as_millis()
                            .min(u16::MAX as u128) as u16
                    }),
                    command_at_ms: latest.map(|sent| {
                        sent.completed_at
                            .saturating_duration_since(run.started)
                            .as_millis()
                            .min(u32::MAX as u128) as u32
                    }),
                    effort: Some(std::array::from_fn(|i| {
                        self.feedback[i].as_ref().unwrap().value.tau
                    })),

                    space: run.report.trials[run.trial.min(run.report.trials.len() - 1)]
                        .space
                        .clone(),
                },
            );
        }
        if matches!(run.step, CalibrationStep::Moving) {
            let axis = run.axis();
            if let Some(at) = self.feedback[axis]
                .as_ref()
                .map(|o| o.at)
                .filter(|at| run.observed.is_none_or(|last| *at > last))
            {
                run.observed = Some(at);
                if let Ok(q) = self.measured(now) {
                    if run.queue[run.trial].phase == "baseline" {
                        let feedback = &self.feedback[axis].as_ref().unwrap().value;
                        run.observations.push(
                            (run.trial, run.leg),
                            calibration::Observation::new(
                                axis,
                                at.saturating_duration_since(run.started).as_micros() as u64,
                                q,
                                feedback.dq,
                                feedback.tau,
                            ),
                        );
                    }
                    let sent = calibration_reference(&run.sent, at);
                    let moving = self.motion.is_some();
                    for collector in &mut run.collectors {
                        collector.observe(
                            at.saturating_duration_since(run.started).as_micros() as u64,
                            sent.map_or(self.reference, |s| s.q),
                            q,
                            moving,
                        );
                    }
                }
            }
        }
        let mut failure = None;
        match run.step {
            CalibrationStep::Configure => {
                let plan = &run.queue[run.trial];
                let config = plan.config;
                let axis = plan.axis;
                let applied = if self.servo_tuning == config {
                    Ok(())
                } else {
                    self.apply(
                        &run.report.request_id,
                        Operation::ConfigureServo {
                            config: Box::new(config),
                        },
                        now,
                        now,
                    )
                    .map(|_| ())
                };
                match applied {
                    Ok(()) => {
                        run.report.trials.push(Trial {
                            index: run.trial,
                            axis: (!plan.cartesian).then_some(axis),
                            phase: plan.phase.clone(),
                            config,
                            metrics: None,
                            space: if plan.cartesian { "cartesian" } else { "joint" }.into(),
                            metrics_by_joint: None,
                        });
                        run.leg = 0;
                        run.legs = run.make_legs();
                        run.collectors = std::array::from_fn(|axis| {
                            Collector::new(axis, run.report.center_measured)
                        });
                        run.leg = 0;
                        run.step = CalibrationStep::Move;
                        run.observed = None;
                        run.sent.clear();
                    }
                    Err(e) if e.code == "motion_busy" => {}
                    Err(e) => failure = Some(e),
                }
            }
            CalibrationStep::Identifying => {
                let result = run.identification.as_ref().unwrap().try_recv();
                match result {
                    Ok(identified) => {
                        run.identification = None;
                        run.report.identification = identified;
                        for fit in &run.report.identification {
                            if let Some(config) = calibration::fitted_config(
                                &run.report.settings,
                                run.report.baseline,
                                fit,
                            ) {
                                run.queue.push(CalibrationPlan {
                                    phase: "candidate".into(),
                                    axis: fit.axis,
                                    config,
                                    cartesian: false,
                                });
                            }
                        }
                        run.step = CalibrationStep::Configure;
                        if run.trial == run.queue.len() {
                            Self::extend_calibration_plan(&mut run);
                        }
                    }
                    Err(crossbeam_channel::TryRecvError::Empty) => {}
                    Err(crossbeam_channel::TryRecvError::Disconnected) => {
                        run.identification = None;
                        Self::extend_calibration_plan(&mut run);
                    }
                }
            }
            CalibrationStep::Move => {
                let leg = run.legs[run.leg].clone();
                let id = format!("cal:{}:{}:{}", run.report.id, run.trial, run.leg);
                let started = match leg {
                    CalibrationLeg::Joint(leg) => {
                        self.begin_calibration_leg(&id, leg.target, leg.speed, None, now)
                    }
                    CalibrationLeg::Planned {
                        space,
                        axis,
                        speed,
                        leg,
                    } => self.begin_calibration_leg(
                        &id,
                        leg.sample(0.0).q,
                        speed,
                        Some((space, axis, leg)),
                        now,
                    ),
                };
                match started {
                    Ok(()) => {
                        run.step = CalibrationStep::Moving;
                    }
                    Err(e) if e.code == "motion_busy" => {}
                    Err(e) => failure = Some(e),
                }
            }
            CalibrationStep::Moving => {
                if self.motion.is_none() && !self.stopping {
                    for collector in &mut run.collectors {
                        collector.observe(
                            now.saturating_duration_since(run.started).as_micros() as u64,
                            self.reference,
                            self.measured(now).unwrap_or(run.report.center_measured),
                            false,
                        );
                    }
                    run.leg += 1;
                    run.step = CalibrationStep::Move;
                    if run.leg == run.legs.len() {
                        if run.cartesian() {
                            run.report.trials[run.trial].metrics_by_joint =
                                Some(run.collectors.iter().map(Collector::finish).collect());
                        } else {
                            run.report.trials[run.trial].metrics =
                                run.collectors[run.queue[run.trial].axis].finish();
                        }
                        run.trial += 1;
                        run.step = CalibrationStep::Configure;
                        if run.trial == run.queue.len() {
                            if run.report.identification.is_empty() && !run.after_added {
                                self.start_friction_identification(&mut run);
                            } else {
                                Self::extend_calibration_plan(&mut run);
                            }
                        }
                    }
                }
            }
            CalibrationStep::Restore => {
                if self.motion.is_none() && !self.stopping && self.at_rest() {
                    if self.servo_tuning != run.report.baseline {
                        self.servo_tuning = run.report.baseline;
                        self.servo_tuning_revision += 1;
                        self.bump_generation();
                    }
                    self.finish_calibration(run);
                    return;
                }
            }
        }
        self.calibration = Some(run);
        if let Some(e) = failure {
            self.end_motion("failed", &e.code, &e.message);
        }
        self.phase();
    }
    fn apply_calibration(&mut self, id: &str, now: Instant) -> Result<(), ApiError> {
        let report = self
            .calibration_result
            .as_ref()
            .and_then(|s| s.report.as_ref())
            .filter(|r| r.id == id)
            .ok_or_else(|| ApiError::new("calibration_unavailable", "当前没有这份测试结果"))?;
        let config = report
            .recommended
            .ok_or_else(|| ApiError::new("calibration_unavailable", "本次参数保持原样"))?;
        self.apply(
            id,
            Operation::ConfigureServo {
                config: Box::new(config),
            },
            now,
            now,
        )?;
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
        self.finish_teaching_disabled();
        self.restore_calibration_disabled();
        self.reset_servo_tuning();
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
        if self.motion.is_some()
            || self.stopping
            || self.teaching_mode.is_some()
            || self.replay.is_some()
        {
            return Err(ApiError::new("motion_busy", "当前动作尚未结束"));
        }
        let measured = self.measured(now)?;
        self.check_positions(&measured)?;
        if let Some(bus) = &self.bus {
            // Use the last position actually sent on this connection. Context
            // expiry cancels actions; it does not make that position disappear.
            if let Some(sent) = bus.latest_reference() {
                return Ok(sent.q);
            }
        }
        let preserve_reference = self.reference.iter().all(|value| value.is_finite());
        Ok(if preserve_reference {
            self.reference
        } else {
            measured
        })
    }
    fn begin_calibration_leg(
        &mut self,
        id: &str,
        first: [f64; 7],
        speed: f64,
        leg: Option<(Space, usize, Arc<crate::calibration_spline::TimedLeg>)>,
        now: Instant,
    ) -> Result<(), ApiError> {
        let anchor = self.motion_anchor(now)?;
        let mut speeds = self.joint_speeds();
        let (space, axis, path) = match leg {
            Some((space, axis, path)) => (space, axis, Some(path)),
            None => {
                speeds = speeds.map(|v| v.min(speed));
                (Space::Joint, 0, None)
            }
        };
        let approach = teaching::Approach::for_motion(anchor, first, speeds, self.motion_tuning)
            .map_err(|e| ApiError::new("invalid_calibration", e))?;
        let plan = Arc::new(crate::calibration_path::Playback {
            approach,
            leg: path,
        });
        let duration = plan.duration();
        let target = plan.sample(duration).q;
        let (initial_pose, path_origin, target_pose) = if space == Space::Cartesian {
            let model = self.model.as_ref().unwrap();
            let tcp = self.tcp.as_ref().unwrap();
            let pose = |q| model.fk(q, tcp).map_err(|e| ApiError::new("kinematics", e));
            (
                Some(pose(&anchor)?),
                Some(pose(&first)?),
                Some(pose(&target)?),
            )
        } else {
            (None, None, None)
        };
        let distance = initial_pose
            .zip(target_pose)
            .map_or(0.0, |(a, b)| b.translation[axis] - a.translation[axis]);
        self.reference = anchor;
        self.reset_derivatives();
        self.absolute_hold = false;
        self.motion = Some(Motion {
            calibration_phase: None,
            planned: Some(plan),
            calibration: true,
            finishing: None,
            request_id: id.into(),
            space,
            axis,
            direction: distance.signum(),
            speed,
            frame: Frame::Base,
            mode: MotionMode::Step,
            amount: distance.abs(),
            started: now,
            duration,
            initial: anchor,
            target,
            initial_pose,
            path_origin,
            target_pose,
            deadline: None,
            input_seq: 0,
        });
        self.phase = Phase::Moving;
        Ok(())
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
        if !speed.is_finite() || speed <= 0.0 {
            return Err(ApiError::new("speed_limit", "收纳速度必须为正数"));
        }
        if target.iter().any(|q| !q.is_finite()) {
            return Err(ApiError::new("invalid_pose", "收纳位参数无效"));
        }
        let distance = (0..7)
            .map(|i| (target[i] - anchor[i]).abs())
            .fold(0.0, f64::max);
        let speed = speed.min(
            self.joint_speeds()
                .into_iter()
                .fold(f64::INFINITY, f64::min),
        );
        let duration = trajectory::segment_duration(
            distance,
            speed,
            self.motion_tuning.acceleration,
            self.motion_tuning.jerk,
        );
        if !duration.is_finite() {
            return Err(ApiError::new("invalid_pose", "收纳轨迹时长无效"));
        }
        self.reference = anchor;
        self.reset_derivatives();
        self.absolute_hold = false;
        self.motion = Some(Motion {
            calibration_phase: None,
            planned: None,
            calibration: false,
            finishing: None,
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
            path_origin: None,
            target_pose: None,
            deadline: None,
            input_seq: 0,
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
        let speed = if space == Space::Joint {
            speed.min(self.joint_speeds()[axis])
        } else {
            speed
        };
        self.reference = anchor;
        self.reset_derivatives();
        self.absolute_hold = false;
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
            if space == Space::Joint {
                trajectory::segment_duration(
                    amount,
                    speed,
                    self.motion_tuning.acceleration,
                    self.motion_tuning.jerk,
                )
            } else {
                self.cartesian_step_duration(
                    anchor,
                    self.tcp.as_ref().unwrap(),
                    target_pose.as_ref().unwrap(),
                    amount,
                    speed,
                )?
            }
        } else {
            0.0
        };
        self.motion = Some(Motion {
            calibration_phase: None,
            planned: None,
            calibration: false,
            finishing: None,
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
            path_origin: None,
            target_pose,
            deadline,
            input_seq,
        });
        self.phase = Phase::Moving;
        Ok(())
    }

    fn cartesian_step_duration(
        &self,
        anchor: [f64; 7],
        tcp: &Pose,
        goal: &Pose,
        amount: f64,
        speed: f64,
    ) -> Result<f64, ApiError> {
        let displacement = self
            .model
            .as_ref()
            .unwrap()
            .local_displacement(&anchor, tcp, goal)
            .map_err(|e| ApiError::new("kinematics", e))?;
        let distance = displacement.into_iter().map(f64::abs).fold(0.0, f64::max);
        let joint_speed = self
            .joint_speeds()
            .into_iter()
            .fold(f64::INFINITY, f64::min);
        Ok((1.5 * amount / speed).max(trajectory::segment_duration(
            distance,
            joint_speed,
            self.motion_tuning.acceleration,
            self.motion_tuning.jerk,
        )))
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
        self.invalidate_reference();
        self.pending_calibration_stop = None;
        self.velocity = [0.0; 7];
        self.acceleration = [0.0; 7];
        self.jerk = [0.0; 7];
        self.stop_path = None;
    }
    fn motion_path(motion: &Motion) -> Option<CartesianPath> {
        (motion.space == Space::Cartesian).then(|| CartesianPath {
            origin: motion.path_origin.or(motion.initial_pose).unwrap(),
            axis: motion.axis,
            frame: motion.frame,
        })
    }
    // Constrain the controller velocity before integrating its reference once.
    // The serial sender may coalesce updates; capture its timing separately.
    // Absolute-time waypoints alone do not bound derivatives under scheduler jitter.
    fn joint_speeds(&self) -> [f64; 7] {
        self.motion_tuning.speed_limits(
            self.model
                .as_ref()
                .map_or([self.motion_tuning.joint_speed; 7], |model| {
                    model.speed_limits()
                }),
        )
    }
    fn commit_joint_velocity(&mut self, desired: [f64; 7], dt: f64) -> Result<(), ApiError> {
        let window = trajectory::velocity_window(
            self.velocity,
            self.acceleration,
            desired,
            self.joint_speeds(),
            [self.motion_tuning.acceleration; 7],
            [self.motion_tuning.jerk; 7],
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
        let model = self.model.as_ref().ok_or("missing model")?;
        let tcp = self.tcp.ok_or("missing TCP")?;
        let joint_velocity = self.joint_speeds();
        let (mut linear_speed, mut angular_speed) =
            model.cartesian_speed_limits(&self.reference, &tcp, joint_velocity)?;
        if let Some(motion) = &self.motion
            && motion.space == Space::Cartesian
        {
            if motion.axis < 3 {
                linear_speed = linear_speed.min(motion.speed);
            } else {
                angular_speed = angular_speed.min(motion.speed);
            }
        }
        // Keep translation and orientation objectives dimensionally balanced.
        // A low translation request must not make orientation hundreds of
        // times less important merely because model angular speed is high.
        let length = model.reach_radius(&tcp);
        let task_rate = (linear_speed / length).min(angular_speed);
        linear_speed = task_rate * length;
        angular_speed = task_rate;
        let planning = if let Some(run) = &self.calibration
            && let Some(program) = &run.report.settings.cartesian
        {
            let origin = model.fk(&program.origin, &program.tcp)?;
            let current = model.fk(&self.reference, &tcp)?;
            Some(CartesianPlanning {
                origin: run.report.center_reference,
                guide: path.filter(|p| p.axis < 3).and_then(|p| {
                    program.guide_at(
                        p.axis,
                        current.translation[p.axis] - origin.translation[p.axis],
                    )
                }),
            })
        } else {
            None
        };
        if !stopping && let Some(guide) = planning.and_then(|p| p.guide) {
            let path_speed = joint_velocity
                .iter()
                .zip(guide.tangent)
                .map(|(speed, tangent)| speed / tangent.abs())
                .fold(f64::INFINITY, f64::min);
            let scale = (path_speed / linear_speed).min(1.0);
            linear_speed *= scale;
            angular_speed *= scale;
        }
        let candidate = self
            .model
            .as_ref()
            .ok_or("missing model")?
            .step(StepInput {
                planning,
                reference: self.reference,
                measured,
                previous_velocity: self.velocity,
                target,
                dt,
                tcp: self.tcp.ok_or("missing TCP")?,
                limits: MotionLimits {
                    joint_velocity,
                    joint_acceleration: [self.motion_tuning.acceleration; 7],
                    linear_speed,
                    angular_speed,
                },
                smoothing: Some(SmoothingState {
                    hold: if stopping {
                        self.motion.as_ref().and_then(|m| m.finishing)
                    } else {
                        None
                    },
                    acceleration: self.acceleration,
                    jerk: [self.motion_tuning.jerk; 7],
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
        if let Some(path) = self.stop_path
            && !self.can_finish_stop(dt)
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
    fn can_finish_stop(&self, dt: f64) -> bool {
        // Hand off once zero velocity, then zero acceleration, fit the configured
        // derivatives. This avoids an absolute near-zero velocity threshold.
        let jerk_step = self.motion_tuning.jerk * dt;
        (0..7).all(|i| {
            let acceleration = -self.velocity[i] / dt;
            acceleration.abs() <= self.motion_tuning.acceleration
                && (acceleration - self.acceleration[i]).abs() <= jerk_step
                && acceleration.abs() <= jerk_step
        })
    }
    fn advance(&mut self, now: Instant, dt: f64) {
        self.drain_feedback(false);
        self.flush_calibration_report();
        self.flush_teaching_records();
        self.advance_motion(now, dt);
        self.advance_calibration(now);
        self.advance_teaching_recording(now);
    }
    fn advance_motion(&mut self, now: Instant, dt: f64) {
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
                if self.calibration.as_ref().is_some_and(|r| !r.cancelled)
                    && matches!(e.code.as_str(), "feedback_missing" | "feedback_stale")
                {
                    self.fault_stale_calibration(now);
                } else {
                    self.fault(&e.code, e.message);
                }
                return;
            }
        };
        if dt <= 0.0 || !dt.is_finite() {
            self.fault("control_timeout", "控制周期超时");
            return;
        }
        if self.wait_for_calibration_feedback(now) {
            return;
        }
        if self.teaching_mode.is_some() {
            self.advance_teaching_mode(measured, now);
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
            if !self.settle_calibration_stop(now) {
                return;
            }
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
        if let Some(replay) = self.replay.as_ref() {
            let elapsed = now.saturating_duration_since(replay.started).as_secs_f64();
            let sample = replay.program.sample(elapsed);
            self.reference = sample.q;
            self.velocity = sample.dq;
            self.acceleration = sample.ddq;
            if elapsed >= replay.program.duration() {
                let replay = self.replay.take().unwrap();
                self.reset_derivatives();
                self.absolute_hold = true;
                self.bump_generation();
                self.record(&replay.request_id, "completed", "replayed", "轨迹重放完成");
                self.phase();
            }
            self.dirty = true;
            return;
        }
        let Some(motion) = self.motion.as_ref() else {
            return;
        };
        let elapsed = now.saturating_duration_since(motion.started).as_secs_f64();
        if let Some(plan) = &motion.planned {
            let sent = self
                .bus
                .as_ref()
                .and_then(BusHandle::latest_reference)
                .filter(|s| s.epoch == self.reference_version);
            let clock = sent
                .and_then(|s| s.calibration_phase)
                .unwrap_or(PlaybackPhase::Original(motion.started));
            let sample = clock.sample(plan, now);
            self.jerk = std::array::from_fn(|i| (sample.ddq[i] - self.acceleration[i]) / dt);
            self.reference = sample.q;
            self.velocity = sample.dq;
            self.acceleration = sample.ddq;
            // A stale hold may still be in flight as our clock reaches the end.
            // Hand off only after the writer actually sent the endpoint.
            if sent.is_some_and(|s| {
                !s.fallback_hold
                    && s.calibration_phase.is_some()
                    && clock.progress(s.at.min(now)) >= motion.duration
            }) {
                self.motion = None;
                self.absolute_hold = true;
                self.reset_derivatives();
                self.bump_generation();
                self.phase();
            } else {
                self.motion.as_mut().unwrap().calibration_phase = Some(clock);
            }
            self.dirty = true;
            return;
        }
        let mode = motion.mode;
        let t = if mode.timed() {
            (elapsed / motion.duration).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let smooth = if motion.space == Space::Joint {
            t * t * t * (10.0 + t * (-15.0 + 6.0 * t))
        } else {
            t * t * (3.0 - 2.0 * t)
        };
        let within_target = |q: &[f64; 7]| {
            if motion.space == Space::Joint {
                let tolerance = if matches!(mode, MotionMode::Return { .. }) {
                    RETURN_POSITION_TOLERANCE
                } else {
                    0.01
                };
                (0..7).all(|i| (q[i] - motion.target[i]).abs() <= tolerance)
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
        let needs_measured_position = !motion.calibration
            && !matches!(
                mode,
                MotionMode::Return {
                    disconnect_after: false
                }
            );
        let at_target = mode.timed()
            && t >= 1.0
            && within_target(&self.reference)
            && (!needs_measured_position || within_target(&measured));
        // A Cartesian pose controller finishes its position trajectory before
        // handing off to zero-velocity hold. Switching modes merely on entering
        // the pose tolerance can send a slow jerk ramp past the goal repeatedly.
        let pose_finish = motion.space == Space::Cartesian && matches!(mode, MotionMode::Step);
        let finishing = motion.finishing;
        let arrival_path = Self::motion_path(motion);
        if at_target || finishing.is_some() {
            if pose_finish && (!at_target || !self.can_finish_stop(dt)) {
                let arrival = finishing.unwrap_or_else(|| {
                    let velocity = self
                        .velocity
                        .iter()
                        .map(|v| v.abs())
                        .fold(f64::EPSILON, f64::max);
                    let jerk = self.motion_tuning.jerk;
                    HoldPose {
                        joints: self.reference,
                        // Keep the arrival bandwidth fixed as velocity decays.
                        bandwidth: (jerk / velocity).sqrt(),
                    }
                });
                self.motion.as_mut().unwrap().finishing = Some(arrival);
                let goal = self
                    .model
                    .as_ref()
                    .unwrap()
                    .fk(&arrival.joints, self.tcp.as_ref().unwrap())
                    .unwrap();
                if let Err(error) =
                    self.commit_cartesian(LocalTarget::Pose(goal), None, measured, dt, true)
                {
                    self.end_motion("failed", "kinematics", &error);
                }
                return;
            }
            self.stop_path = arrival_path;
            if let Err(e) = self.decelerate(measured, dt) {
                self.fault(&e.code, e.message);
                return;
            }
            if self.at_rest() {
                if let MotionMode::Return { disconnect_after } = mode {
                    let completed = self.motion.take().unwrap();
                    let internal = completed.calibration;
                    let id = completed.request_id;
                    if disconnect_after {
                        self.disconnect();
                        self.record(&id, "completed", "parked", "收纳到位，连接已断开");
                    } else {
                        self.bump_generation();
                        self.stop_path = None;
                        if !internal {
                            self.record(&id, "completed", "returned", "已返回收纳位");
                        }
                        self.phase();
                    }
                    return;
                }
                let id = self.motion.take().unwrap().request_id;
                self.stop_path = None;
                self.bump_generation();
                self.record(&id, "completed", "completed", "动作已完成");
                self.phase();
            }
            return;
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
                }
            }
        }
    }
    fn outgoing_reference(&self) -> Reference {
        let mut reference = Reference::joint(
            self.control_at,
            self.reference,
            self.velocity,
            self.acceleration,
            self.generation,
            self.reference_version,
        );
        reference.joint_velocity = self.joint_speeds();
        reference.servo_tuning = self.servo_tuning;
        reference.tuning_revision = self.servo_tuning_revision;
        reference.sampling = if let Some(replay) = &self.replay {
            Sampling::Playback {
                program: replay.program.clone(),
                started: replay.started,
            }
        } else if let Some((motion, program)) = self
            .motion
            .as_ref()
            .and_then(|m| m.planned.as_ref().map(|p| (m, p)))
        {
            Sampling::Calibration {
                program: program.clone(),
                feedback_at: self.oldest_feedback().filter(|_| {
                    !self
                        .calibration
                        .as_ref()
                        .is_some_and(|run| run.feedback_wait.is_some())
                }),
                phase: motion
                    .calibration_phase
                    .unwrap_or(PlaybackPhase::Original(motion.started)),
                limits: self.motion_tuning,
            }
        } else if self.absolute_hold {
            Sampling::Absolute
        } else {
            Sampling::Integrated
        };
        reference.stopping = self.stopping || self.stop_path.is_some();
        let motion = self.motion.as_ref();
        if let Some(motion) = motion {
            reference.valid_until = motion.deadline;
            if let Some(lease) = &self.lease {
                reference.valid_until = Some(
                    reference
                        .valid_until
                        .map_or(lease.expires, |until| until.min(lease.expires)),
                );
            }
        }
        if self.replay.is_some() {
            reference.valid_until = self.lease.as_ref().map(|lease| lease.expires);
        }
        reference.joint_goal = motion
            .filter(|m| m.space == Space::Joint && m.mode.timed())
            .map(|m| m.target);
        if let (Some(model), Some(tcp), Some(path)) = (
            &self.model,
            self.tcp,
            motion.and_then(Self::motion_path).or(self.stop_path),
        ) {
            reference.cartesian = Some(CartesianGuard {
                model: model.clone(),
                tcp,
                path,
                goal: motion.and_then(|m| m.target_pose),
            });
        }
        reference
    }
    fn reject_send_reference(
        &mut self,
        generation: u64,
        epoch: u64,
        held: [f64; 7],
        reason: crate::reference::SampleStatus,
    ) {
        // A rejection from an earlier action cannot alter a newer stop/return.
        if (!self.enabled && self.enable_request.is_none())
            || self.generation != generation
            || self.reference_version != epoch
            || self.reference_epoch.load(Ordering::Acquire) != epoch
        {
            return;
        }
        if matches!(
            reason,
            crate::reference::SampleStatus::Stale
                | crate::reference::SampleStatus::Future
                | crate::reference::SampleStatus::Constraint
        ) {
            self.journal.log(
                "debug",
                "control",
                "send_hold",
                &format!("发送暂缓：{reason:?}"),
                None,
            );
            return;
        }
        self.end_motion("failed", "send_reference", "发送参考无法对齐，动作已停止");
        self.reference = held;
        self.reset_derivatives();
        self.stopping = false;
        self.phase();
        self.journal.log(
            "warn",
            "control",
            "send_reference",
            &format!("发送参考已保持：{reason:?}"),
            None,
        );
    }
    fn desired_setpoints(&self) -> [Setpoint; 7] {
        let servo = self.enabled || self.enable_request.is_some();
        let gravity = if servo {
            self.model
                .as_ref()
                .and_then(|model| model.gravity(&self.reference).ok())
                .unwrap_or([0.0; 7])
        } else {
            [0.0; 7]
        };
        std::array::from_fn(|i| Setpoint {
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
                gravity[i]
            } else {
                0.0
            },
            kp: if servo {
                SERVO_KP * self.teaching_stiffness
            } else {
                0.0
            },
            kd: if servo { SERVO_KD } else { 0.0 },
        })
    }
    fn poll(&mut self, now: Instant) {
        let setpoints = self.desired_setpoints();
        let clearing = self.clear_pending.iter().any(|pending| *pending);
        let motion = self.motion.as_ref();
        self.journal.sample(
            self.control_at,
            TraceData::Control {
                seq: self.control_seq,
                generation: self.generation,
                servo_tuning: Some(self.servo_tuning),
                servo_tuning_revision: self.servo_tuning_revision,
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
        let heartbeat = setpoints.map(|mut value| {
            if value.mode == DriveMode::ClearFault {
                value.mode = DriveMode::Disabled;
            }
            value
        });
        let send_result = if self.pending_calibration_stop.is_some() && self.enabled {
            Ok(())
        } else {
            bus.publish(heartbeat, self.control_seq, self.outgoing_reference())
        };
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
        // The blocking ClearFault write can produce replies after this tick's
        // pre-motion drain. Consume them before checking the clear acknowledgement.
        if self.clear_request.is_some() {
            self.drain_feedback(false);
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
    fn drain_feedback(&mut self, enabling: bool) {
        let Some(bus) = &self.bus else { return };
        let (frames, error) = bus.drain();
        if let Some(BusError::Io(message)) = error {
            self.fault("communication_error", message);
            return;
        }
        for (value, at) in frames {
            self.handle_feedback(value, at, enabling);
        }
        if let Some(BusError::Reference {
            generation,
            epoch,
            held,
            reason,
        }) = error
        {
            self.reject_send_reference(generation, epoch, held, reason);
        }
    }

    fn oldest_feedback(&self) -> Option<Instant> {
        self.feedback
            .iter()
            .map(|observation| observation.as_ref().map(|o| o.at))
            .min()
            .flatten()
    }

    fn wait_for_calibration_feedback(&mut self, now: Instant) -> bool {
        if !self.calibration.as_ref().is_some_and(|run| !run.cancelled) {
            return false;
        }
        // measured() already checked all seven axes against the hard timeout.
        let oldest = self.oldest_feedback().unwrap();
        let fresh = now.saturating_duration_since(oldest) <= MAX_REFERENCE_AGE;
        let sent = self.bus.as_ref().and_then(BusHandle::latest_reference);
        let current = sent.filter(|s| s.epoch == self.reference_version);
        let planned = self.motion.as_ref().filter(|m| m.planned.is_some());
        // The writer may already have held while the controller was descheduled.
        // Acknowledge that specific pause even if the RX queue is now fresh.
        let new_hold = planned.and_then(|m| {
            let phase = current?.calibration_phase?;
            match phase {
                PlaybackPhase::Held { since, .. } if m.calibration_phase != Some(phase) => {
                    Some(since)
                }
                _ => None,
            }
        });
        let late_writer = planned.is_some_and(|m| {
            now.saturating_duration_since(current.map_or(m.started, |s| s.at)) > MAX_REFERENCE_AGE
        });
        let run = self.calibration.as_mut().unwrap();
        if run.feedback_wait.is_none() && (!fresh || new_hold.is_some() || late_writer) {
            run.feedback_wait = Some(if fresh { new_hold.unwrap_or(now) } else { now });
            run.observations.break_window();
            for collector in &mut run.collectors {
                collector.break_window();
            }
        }
        let Some(since) = run.feedback_wait else {
            return false;
        };
        let held = current.is_some_and(|s| {
            s.at >= since && matches!(s.calibration_phase, Some(PlaybackPhase::Held { .. }))
        });
        if fresh && oldest > since && (planned.is_none() || (held && !late_writer)) {
            run.feedback_wait = None;
            run.observed = self.feedback[run.axis()].as_ref().map(|o| o.at);
            return false;
        }
        if let Some(sent) = sent {
            self.reference = sent.q;
        }
        self.velocity = [0.0; 7];
        self.acceleration = [0.0; 7];
        true
    }

    fn fault_stale_calibration(&mut self, at: Instant) {
        let axes: Vec<_> = self
            .feedback
            .iter()
            .enumerate()
            .filter_map(|(i, observation)| {
                let joint = i + 1;
                match observation {
                    None => Some(format!("J{joint} 无反馈")),
                    Some(o) => {
                        let age = at.saturating_duration_since(o.at).as_millis();
                        (age > FEEDBACK_MS as u128).then(|| format!("J{joint} {age} ms"))
                    }
                }
            })
            .collect();
        let wait = self
            .calibration
            .as_ref()
            .and_then(|r| r.feedback_wait)
            .map_or(0, |since| at.saturating_duration_since(since).as_millis());
        self.fault(
            "feedback_stale",
            format!("关节反馈过期：{}；已暂缓等待 {wait} ms", axes.join("、")),
        );
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
            gravity[joint],
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
    #[cfg(test)]
    fn accept_feedback(&mut self, value: Feedback, at: Instant) {
        self.handle_feedback(value, at, false);
    }
    fn handle_feedback(&mut self, value: Feedback, at: Instant, enabling: bool) {
        let id = value.id as usize;
        if id >= 7 {
            return;
        }
        let error = value.error;
        let timeout = value.timeout;
        let mode = value.mode;
        let report_fault = self.error.is_none() || self.clear_request.is_some();
        self.feedback[id] = Some(Observation { value, at });
        if !report_fault {
            return;
        }
        if error != 0 && self.clear_request.is_none() {
            self.fault("device_error", format!("J{} 故障码 {}", id + 1, error));
        } else if timeout && (enabling || self.enabled || self.enable_request.is_some()) {
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
            action: self
                .calibration
                .as_ref()
                .map(|run| ActionSnapshot {
                    request_id: run.report.request_id.clone(),
                    space: if run.cartesian() {
                        Space::Cartesian
                    } else {
                        Space::Joint
                    },
                    mode: "calibration".into(),
                    axis: Some(run.axis()),
                    progress: self.calibration_snapshot(run).progress,
                })
                .or_else(|| {
                    self.motion.as_ref().map(|motion| ActionSnapshot {
                        request_id: motion.request_id.clone(),
                        space: motion.space,
                        mode: motion.mode.name().into(),
                        axis: (!matches!(motion.mode, MotionMode::Return { .. }))
                            .then_some(motion.axis),
                        progress: if motion.mode.timed() {
                            (now.saturating_duration_since(motion.started).as_secs_f64()
                                / motion.duration)
                                .min(1.0)
                        } else {
                            0.0
                        },
                    })
                })
                .or_else(|| {
                    self.teaching_mode.as_ref().map(|run| ActionSnapshot {
                        request_id: run.request_id.clone(),
                        space: Space::Joint,
                        mode: "teaching".into(),
                        axis: None,
                        progress: 0.0,
                    })
                })
                .or_else(|| {
                    self.teaching_status(now).map(|status| ActionSnapshot {
                        request_id: status.request_id,
                        space: Space::Joint,
                        mode: "replay".into(),
                        axis: None,
                        progress: status.progress,
                    })
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
            servo_tuning: self.servo_tuning,
            motion_tuning: self.motion_tuning,
            servo_tuning_revision: self.servo_tuning_revision,
            calibration: self
                .calibration
                .as_ref()
                .map(|run| self.calibration_snapshot(run))
                .or_else(|| self.calibration_result.clone()),
            calibration_report: self.last_calibration_report.clone(),
            teaching: self.teaching_status(now),
            teaching_mode: self.teaching_mode_status(now),
            teaching_recording: self.teaching_recording_status(),
            latest_teaching: self
                .last_teaching
                .as_ref()
                .map(|capture| capture.summary.clone()),
            teaching_capture: self.last_teaching.clone(),
            teaching_pending: self.pending_teachings.iter().cloned().collect(),
            reference: self.reference,
        }
    }
}

fn input_deadline(received: Instant, ttl_ms: u64, now: Instant) -> Result<Instant, ApiError> {
    if ttl_ms == 0 {
        return Err(ApiError::new("invalid_ttl", "输入有效期无效"));
    }
    let deadline = received
        .checked_add(Duration::from_millis(ttl_ms))
        .ok_or_else(|| ApiError::new("invalid_ttl", "输入有效期超出计时范围"))?;
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
) -> PendingRecords {
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
            core.stop_command("host_stop", "宿主已取消当前动作", now);
            seen_epoch = epoch;
        }
        if shutdown.load(Ordering::Acquire) || core.close_requested {
            shutdown.store(true, Ordering::Release);
            core.end_motion("cancelled", "closing", "应用正在关闭");
            core.lease = None;
            core.enabled = false;
            core.finish_teaching_disabled();
            core.restore_calibration_disabled();
            core.flush_calibration_report();
            core.flush_teaching_records();
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
        let dt = at.saturating_duration_since(last_control).as_secs_f64();
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
    PendingRecords {
        calibrations: std::mem::take(&mut core.pending_calibration_report),
        teachings: std::mem::take(&mut core.pending_teachings),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calibration_start_is_a_typed_motion_command() {
        let request = serde_json::from_value::<CommandRequest>(serde_json::json!({
            "request_id": "calibration-test", "client_id": "desktop", "lease_id": "lease", "generation": 1,
            "kind": "calibrate_start",
            "settings": { "axis": 1, "excursion": 0.1, "speed": 0.02, "max_friction": 0.1 }
        }));
        assert!(
            request.is_ok(),
            "calibration must enter through the existing command interface"
        );
    }

    #[test]
    fn servo_configuration_is_atomic_owner_only_and_resets_on_disconnect() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        let (bus, _sent) = BusHandle::test_idle_bus();
        core.bus = Some(bus);
        core.port = Some("memory-only".into());
        core.phase = Phase::Disabled;
        let lease = core
            .control(
                ControlRequest {
                    action: "acquire".into(),
                    client_id: "owner".into(),
                    lease_id: None,
                },
                now,
            )
            .unwrap();
        let secret = lease.lease_id.unwrap();
        let old_generation = core.generation;
        let old_revision = core.servo_tuning_revision;
        let mut config = ServoTuning {
            friction_enabled: true,
            ..ServoTuning::default()
        };
        config.joints[3].kd = 4.0;
        config.joints[3].velocity_ff = 0.0;
        config.joints[3].coulomb = 0.2;
        let request = |id: &str, client: &str, generation, config| CommandRequest {
            request_id: id.into(),
            client_id: client.into(),
            lease_id: secret.clone(),
            generation,
            operation: Operation::ConfigureServo {
                config: Box::new(config),
            },
        };
        core.command(
            request("wrong-owner", "reader", old_generation, config),
            now,
            now,
        );
        assert_eq!(core.results.front().unwrap().code, "not_owner");
        assert_eq!(core.servo_tuning, ServoTuning::default());
        let mut invalid = config;
        invalid.joints[2].kd = f64::NAN;
        core.command(
            request("invalid", "owner", old_generation, invalid),
            now,
            now,
        );
        assert_eq!(core.results.front().unwrap().code, "invalid_servo_tuning");
        assert_eq!(core.generation, old_generation);
        assert_eq!(core.servo_tuning_revision, old_revision);
        core.command(request("apply", "owner", old_generation, config), now, now);
        assert_eq!(core.results.front().unwrap().status, "accepted");
        assert_eq!(core.servo_tuning, config);
        assert_eq!(core.servo_tuning_revision, old_revision + 1);
        let state = core.snapshot(now);
        assert_eq!(state.servo_tuning, config);
        assert_eq!(state.servo_tuning_revision, old_revision + 1);
        core.command(
            request("stale", "owner", old_generation, ServoTuning::default()),
            now,
            now,
        );
        assert_eq!(core.results.front().unwrap().code, "stale_context");
        assert_eq!(core.servo_tuning, config);
        core.disconnect();
        assert_eq!(core.servo_tuning, ServoTuning::default());
        assert!(core.servo_tuning_revision > old_revision + 1);
    }

    #[test]
    fn servo_configuration_preserves_the_hold_tcp_and_rejects_motion_or_enabling() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        let (bus, sent) = BusHandle::test_idle_bus();
        core.bus = Some(bus);
        core.port = Some("memory-only".into());
        core.enabled = true;
        core.phase = Phase::Ready;
        core.tcp = Some(Pose {
            translation: [0.0, 0.0, 0.1],
            rotation: [0.0, 0.0, 0.0, 1.0],
        });
        let tcp = core.tcp;
        let held = core.reference.map(|q| q + 0.0003);
        sent.send_replace(Some(crate::protocol::SentReference {
            at: now,
            completed_at: now,
            generation: core.generation,
            epoch: core.reference_version,
            q: held,
            dq: [0.0; 7],
            calibration_phase: None,
            fallback_hold: false,
        }));
        let mut config = ServoTuning::default();
        config.joints[0].velocity_ff = 0.0;
        core.apply(
            "tune",
            Operation::ConfigureServo {
                config: Box::new(config),
            },
            now,
            now,
        )
        .unwrap();
        assert!(core.enabled && core.port.is_some());
        assert_eq!(core.reference, held);
        assert_eq!(core.tcp, tcp);
        let r = core.outgoing_reference();
        assert_eq!(r.servo_tuning, config);
        assert_eq!(r.tuning_revision, core.servo_tuning_revision);
        core.stopping = true;
        assert_eq!(
            core.apply(
                "blocked",
                Operation::ConfigureServo {
                    config: Box::new(config),
                },
                now,
                now,
            )
            .unwrap_err()
            .code,
            "motion_busy"
        );
        core.stopping = false;
        core.enable_request = Some(("enable".into(), now));
        assert_eq!(
            core.apply(
                "blocked",
                Operation::ConfigureServo {
                    config: Box::new(config),
                },
                now,
                now,
            )
            .unwrap_err()
            .code,
            "motion_busy"
        );
        assert_eq!(core.servo_tuning, config);
    }

    #[test]
    fn late_send_rejection_cannot_cancel_a_new_context_or_pending_host_stop() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        let idle = core.outgoing_reference();
        start_joint(&mut core, "new", now, false);
        let anchor = core.reference;
        assert_eq!(idle.generation, core.generation);
        assert_ne!(idle.epoch, core.reference_version);
        core.reject_send_reference(
            idle.generation,
            idle.epoch,
            [999.0; 7],
            crate::reference::SampleStatus::Constraint,
        );
        assert_eq!(core.reference, anchor);
        assert_eq!(core.motion.as_ref().unwrap().request_id, "new");
        let active = core.outgoing_reference();
        core.reference_epoch.fetch_add(1, Ordering::AcqRel);
        core.reject_send_reference(
            active.generation,
            active.epoch,
            [999.0; 7],
            crate::reference::SampleStatus::Stale,
        );
        assert_eq!(core.reference, anchor);
        assert_eq!(core.motion.as_ref().unwrap().request_id, "new");
    }

    #[test]
    fn send_rejection_finishes_pending_enable_before_feedback_can_confirm_it() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.port = Some("memory-only".into());
        core.enable_request = Some(("enable".into(), now));
        let source = core.outgoing_reference();
        let held = core.reference.map(|q| q - 0.02);
        // poll drains fresh mode=1 feedback before dispatching a writer rejection,
        // then tries to finish enabling. Reproduce that exact event order.
        for i in 0..7 {
            let value = core.feedback[i].as_ref().unwrap().value.clone();
            core.accept_feedback(value, now + Duration::from_millis(10));
        }
        core.reject_send_reference(
            source.generation,
            source.epoch,
            held,
            crate::reference::SampleStatus::Invalid,
        );
        core.settle_transitions(now + Duration::from_millis(10));
        assert!(core.enable_request.is_none() && !core.enabled);
        assert!(!matches!(core.phase, Phase::Ready | Phase::Enabling));
        assert_eq!(core.reference, held);
        assert_eq!(core.results.front().unwrap().request_id, "enable");
        assert_eq!(core.results.front().unwrap().status, "failed");
        assert_eq!(core.results.front().unwrap().code, "send_reference");
        assert!(core.at_rest() && core.port.is_some());
    }

    #[test]
    fn a_rejected_return_holds_the_sent_target_and_never_disconnects_later() {
        let mut now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        core.port = Some("memory-only".into());
        let mut goal = core.reference;
        goal[1] += 0.1;
        core.begin_return("return", goal, 0.05, true, now).unwrap();
        let reference = core.outgoing_reference();
        let held = core.reference;
        core.reject_send_reference(
            reference.generation,
            reference.epoch,
            held,
            crate::reference::SampleStatus::Invalid,
        );
        assert!(core.motion.is_none() && !core.stopping && core.enabled);
        assert!(core.port.is_some());
        assert_eq!(core.results.front().unwrap().code, "send_reference");
        for _ in 0..500 {
            now += Duration::from_millis(10);
            follow_reference(&mut core, now);
            core.advance(now, 0.01);
        }
        assert!(core.port.is_some() && core.enabled);
        assert_eq!(core.reference, held);
        assert!(core.at_rest());
    }

    #[test]
    fn sampling_context_keeps_tcp_center_during_stop_and_removes_input_deadline() {
        let mut now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        core.tcp = Some(Pose {
            translation: [0.0, 0.0, 0.1],
            rotation: [0.0, 0.0, 0.0, 1.0],
        });
        let expires = now + Duration::from_millis(150);
        core.begin(
            "cartesian",
            Space::Cartesian,
            3,
            1,
            0.01,
            Frame::Tool,
            false,
            0.0,
            Some(expires),
            0,
            now,
        )
        .unwrap();
        now += Duration::from_millis(10);
        follow_reference(&mut core, now);
        core.advance(now, 0.01);
        assert!(core.error.is_none());
        let moving = core.outgoing_reference();
        assert_eq!(moving.valid_until, Some(expires));
        let path = moving.cartesian.as_ref().unwrap().path;
        core.end_motion("cancelled", "stopped", "已停止");
        let stopped = core.outgoing_reference();
        assert!(stopped.valid_until.is_none());
        assert!(stopped.joint_goal.is_none());
        if core.stopping {
            let guard = stopped.cartesian.as_ref().unwrap();
            assert_eq!(guard.path.origin, path.origin);
            assert_eq!(guard.tcp, core.tcp.unwrap());
            assert!(guard.goal.is_none());
        }
    }

    #[test]
    fn return_settling_and_optional_disconnect_follow_time_sampled_wire_targets() {
        for (disconnect_after, tuned) in
            [(false, false), (true, false), (false, true), (true, true)]
        {
            let mut now = Instant::now();
            let (mut core, _directory) = observed_core(now);
            core.enabled = true;
            core.port = Some("memory-only".into());
            if tuned {
                core.servo_tuning.friction_enabled = true;
                for joint in &mut core.servo_tuning.joints {
                    joint.kd = 5.0;
                    joint.velocity_ff = 0.0;
                    joint.coulomb = 0.3;
                }
            }
            let config = core.servo_tuning;
            let mut goal = core.reference;
            for (q, d) in goal
                .iter_mut()
                .zip([0.08, -0.04, 0.06, 0.03, -0.02, 0.01, 0.04])
            {
                *q += d;
            }
            core.begin_return("return", goal, 0.05, disconnect_after, now)
                .unwrap();
            let (mut applied_q, mut applied_dq) = (core.reference, [0.0; 7]);
            let mut previous = None;
            for _ in 0..1000 {
                now += Duration::from_millis(10);
                for i in 0..7 {
                    let mut feedback = core.feedback[i].as_ref().unwrap().value.clone();
                    feedback.q = applied_q[i];
                    feedback.dq = applied_dq[i];
                    core.accept_feedback(feedback, now);
                }
                core.advance(now, 0.01);
                assert!(core.error.is_none());
                if core.motion.is_none() {
                    break;
                }
                let at = now + Duration::from_millis(2);
                let reference = core.outgoing_reference();
                assert_eq!(reference.servo_tuning, config);
                let sample = reference.sample(at, previous).unwrap();
                (applied_q, applied_dq) =
                    crate::protocol::test_wire_targets("right", sample.q, sample.dq);
                previous = Some(crate::reference::Previous {
                    at,
                    q: sample.q,
                    dq: sample.dq,
                });
                assert!(core.enabled && core.port.is_some());
            }
            assert!(core.motion.is_none());
            assert_eq!(core.enabled, !disconnect_after);
            assert_eq!(core.port.is_none(), disconnect_after);
            assert_eq!(
                core.results.front().unwrap().code,
                if disconnect_after {
                    "parked"
                } else {
                    "returned"
                }
            );
            assert!(core.at_rest());
            assert!((0..7).all(|i| (core.reference[i] - goal[i]).abs() <= 0.01));
            assert_eq!(
                core.servo_tuning,
                if disconnect_after {
                    ServoTuning::default()
                } else {
                    config
                }
            );
        }
    }

    #[test]
    fn repeated_jogs_handoff_the_sent_hold_before_rotating_an_offset_tcp() {
        use crate::{protocol::SentReference, reference::Previous};
        let mut now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        let q = [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3];
        core.reference = q;
        core.enabled = true;
        core.port = Some("memory-only".into());
        core.tcp = Some(Pose {
            translation: [0.0, 0.0, 0.1],
            rotation: [0.0, 0.0, 0.0, 1.0],
        });
        for (i, value) in q.into_iter().enumerate() {
            let mut feedback = core.feedback[i].as_ref().unwrap().value.clone();
            feedback.q = value;
            core.accept_feedback(feedback, now);
        }
        core.advance(now, 0.01);
        let (bus, sent) = BusHandle::test_idle_bus();
        core.bus = Some(bus);
        assert_eq!(core.motion_anchor(now).unwrap(), q);
        sent.send_replace(Some(SentReference {
            at: now,
            completed_at: now,
            generation: core.generation,
            epoch: core.reference_version,
            q,
            dq: [0.0; 7],
            calibration_phase: None,
            fallback_hold: false,
        }));
        let mut previous = Some(Previous {
            at: now,
            q,
            dq: [0.0; 7],
        });
        let (mut applied_q, mut applied_dq) = (q, [0.0; 7]);
        let mut next_send = now + Duration::from_millis(2);
        for segment in 0..11 {
            let held = sent.borrow().unwrap();
            let cartesian = segment == 10;
            core.begin(
                "chain",
                if cartesian {
                    Space::Cartesian
                } else {
                    Space::Joint
                },
                if cartesian { 3 } else { 0 },
                1,
                if cartesian { 0.02 } else { 0.2 },
                Frame::Base,
                false,
                0.0,
                None,
                0,
                now,
            )
            .unwrap();
            assert_eq!(core.motion.as_ref().unwrap().initial, held.q);
            let path = Core::motion_path(core.motion.as_ref().unwrap());
            let started = now;
            let mut stopped = false;
            let mut source = core.outgoing_reference();
            for tick in 0..400 {
                let dt = [0.040, 0.002, 0.007, 0.003][tick % 4];
                let control_at = now + Duration::from_secs_f64(dt);
                while next_send < control_at {
                    if source.epoch == core.reference_version {
                        let sample = source.sample(next_send, previous).unwrap();
                        if let Some(path) = path {
                            path.check(
                                &core
                                    .model
                                    .as_ref()
                                    .unwrap()
                                    .fk(&sample.q, core.tcp.as_ref().unwrap())
                                    .unwrap(),
                            )
                            .unwrap();
                        }
                        (applied_q, applied_dq) =
                            crate::protocol::test_wire_targets("right", sample.q, sample.dq);
                        previous = Some(Previous {
                            at: next_send,
                            q: sample.q,
                            dq: sample.dq,
                        });
                        sent.send_replace(Some(SentReference {
                            at: next_send,
                            completed_at: next_send,
                            generation: source.generation,
                            epoch: source.epoch,
                            q: sample.q,
                            dq: sample.dq,
                            calibration_phase: None,
                            fallback_hold: false,
                        }));
                    }
                    next_send += Duration::from_millis(10);
                }
                now = control_at;
                for i in 0..7 {
                    let mut feedback = core.feedback[i].as_ref().unwrap().value.clone();
                    feedback.q = applied_q[i];
                    feedback.dq = applied_dq[i];
                    core.accept_feedback(feedback, now);
                }
                if !stopped && now.duration_since(started) >= Duration::from_millis(730) {
                    core.apply("stop", Operation::Stop, now, now).unwrap();
                    stopped = true;
                }
                core.advance(now, dt);
                assert!(core.error.is_none(), "{:?}", core.error);
                source = core.outgoing_reference();
                let sent_hold = sent.borrow().is_some_and(|last| {
                    last.epoch == core.reference_version && last.dq.iter().all(|v| v.abs() < 1e-12)
                });
                if stopped && core.motion.is_none() && core.at_rest() && sent_hold {
                    break;
                }
            }
            assert!(core.motion.is_none() && core.at_rest(), "segment {segment}");
            assert!(core.enabled && core.port.is_some());
            assert_eq!(core.motion_anchor(now).unwrap(), sent.borrow().unwrap().q);
        }
        // A completed write remains the start position after its timestamp ages.
        let later = now + MAX_REFERENCE_AGE + Duration::from_millis(1);
        follow_reference(&mut core, later);
        assert_eq!(core.motion_anchor(later).unwrap(), sent.borrow().unwrap().q);
    }

    #[test]
    fn cartesian_steps_complete_on_recorded_send_clock_with_an_offset_tcp() {
        #[derive(Deserialize)]
        struct Clock {
            control_us: Vec<u64>,
            sends: Vec<Send>,
        }
        #[derive(Deserialize)]
        struct Send {
            at_us: u64,
            source: usize,
        }
        let clock: Clock =
            serde_json::from_str(include_str!("../tests/fixtures/send-clock.json")).unwrap();
        for frame in [Frame::Base, Frame::Tool] {
            for axis in 0..6 {
                for direction in [-1, 1] {
                    let origin = Instant::now();
                    let (mut core, _directory) = observed_core(origin);
                    let q = [1.5576, -0.035, 0.0102, -0.0217, 1.6217, 0.0088, 0.1133];
                    for (i, value) in q.into_iter().enumerate() {
                        let mut feedback = core.feedback[i].as_ref().unwrap().value.clone();
                        feedback.q = value;
                        core.accept_feedback(feedback, origin);
                    }
                    core.reference = q;
                    core.enabled = true;
                    core.port = Some("memory-only".into());
                    core.tcp = Some(Pose {
                        translation: [0.0, 0.0, 0.1],
                        rotation: [0.0, 0.0, 0.0, 1.0],
                    });
                    core.begin(
                        "sampled-step",
                        Space::Cartesian,
                        axis,
                        direction,
                        if axis < 3 { 0.002 } else { 0.02 },
                        frame,
                        true,
                        if axis < 3 { 0.002 } else { 0.02 },
                        None,
                        0,
                        origin,
                    )
                    .unwrap();
                    let path = Core::motion_path(core.motion.as_ref().unwrap()).unwrap();
                    let target = core.motion.as_ref().unwrap().target_pose.unwrap();
                    let mut references = Vec::new();
                    let (mut applied_q, mut applied_dq) = (q, [0.0; 7]);
                    let mut previous = None;
                    let mut sent = 0;
                    let mut last_control = origin;
                    for &control_us in &clock.control_us {
                        let at = origin + Duration::from_micros(control_us);
                        // Replay all writes before this control tick. The selected
                        // source is taken from the recorded clock, not a new solver.
                        while let Some(send) =
                            clock.sends.get(sent).filter(|s| s.at_us < control_us)
                        {
                            let source: &Reference = &references[send.source];
                            if source.epoch == core.reference_version {
                                let send_at = origin + Duration::from_micros(send.at_us);
                                let sample = source.sample(send_at, previous).unwrap_or_else(|e| {
                                    panic!(
                                        "{frame:?} axis={axis} direction={direction} \
                                        source={} at={} {e:?}",
                                        send.source, send.at_us
                                    )
                                });
                                (applied_q, applied_dq) = crate::protocol::test_wire_targets(
                                    "right", sample.q, sample.dq,
                                );
                                previous = Some(crate::reference::Previous {
                                    at: send_at,
                                    q: sample.q,
                                    dq: sample.dq,
                                });
                            }
                            sent += 1;
                        }
                        for i in 0..7 {
                            let mut feedback = core.feedback[i].as_ref().unwrap().value.clone();
                            feedback.q = applied_q[i];
                            feedback.dq = applied_dq[i];
                            core.accept_feedback(feedback, at);
                        }
                        core.advance(at, at.duration_since(last_control).as_secs_f64());
                        last_control = at;
                        assert!(core.error.is_none(), "{:?}", core.error);
                        let pose = core
                            .model
                            .as_ref()
                            .unwrap()
                            .fk(&core.reference, core.tcp.as_ref().unwrap())
                            .unwrap();
                        path.check(&pose).unwrap();
                        references.push(core.outgoing_reference());
                        if core.motion.is_none() {
                            break;
                        }
                    }
                    assert!(
                        core.motion.is_none() && core.at_rest(),
                        "{frame:?} axis={axis} direction={direction}"
                    );
                    assert_eq!(core.results.front().unwrap().code, "completed");
                    let actual = core
                        .model
                        .as_ref()
                        .unwrap()
                        .fk(&applied_q, core.tcp.as_ref().unwrap())
                        .unwrap();
                    let error = crate::kinematics::pose_error(&actual, &target);
                    assert!(error[..3].iter().map(|v| v * v).sum::<f64>().sqrt() <= 0.001);
                    assert!(
                        error[3..].iter().map(|v| v * v).sum::<f64>().sqrt()
                            <= 1.0_f64.to_radians()
                    );
                    assert!(core.enabled && core.port.is_some());
                }
            }
        }
    }
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

    fn teaching_tick(
        core: &mut Core,
        sent: &watch::Sender<Option<SentReference>>,
        at: Instant,
        actual: [f64; 7],
    ) -> [Setpoint; 7] {
        for (axis, q) in actual.into_iter().enumerate() {
            let old = core.feedback[axis].as_ref().unwrap();
            let dt = at.saturating_duration_since(old.at).as_secs_f64();
            let mut value = old.value.clone();
            value.dq = if dt > 0.0 { (q - value.q) / dt } else { 0.0 };
            value.q = q;
            core.accept_feedback(value, at);
        }
        core.advance(at, 0.01);
        let previous = sent
            .borrow()
            .as_ref()
            .map(|value| crate::reference::Previous {
                at: value.at,
                q: value.q,
                dq: value.dq,
            });
        let sample = core.outgoing_reference().sample(at, previous).unwrap();
        sent.send_replace(Some(SentReference {
            at,
            completed_at: at,
            generation: core.generation,
            epoch: core.reference_version,
            q: sample.q,
            dq: sample.dq,
            calibration_phase: None,
            fallback_hold: false,
        }));
        core.desired_setpoints()
    }

    #[test]
    fn teaching_mode_switch_keeps_servo_gravity_and_holds_the_new_position() {
        let origin = Instant::now();
        let (mut core, _directory, sent) = calibration_fixture(origin);
        let initial = core.reference;
        let tcp = core.tcp;
        let baseline = core.servo_tuning;
        core.apply(
            "teach",
            Operation::TeachStart {
                name: "弧线".into(),
            },
            origin,
            origin,
        )
        .unwrap();
        core.apply(
            "press",
            Operation::TeachMode {
                enabled: true,
                transition_s: 0.4,
            },
            origin,
            origin,
        )
        .unwrap();
        let mut last_kp = SERVO_KP;
        let mut last_q = initial;
        for tick in 1..=100 {
            let mut actual = initial;
            actual[1] += tick as f64 * 0.0005;
            let values = teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                actual,
            );
            assert!(
                values
                    .iter()
                    .all(|s| s.mode == DriveMode::Servo && s.kd > 0.0)
            );
            assert!(
                values.iter().any(|s| s.tau.abs() > 0.1),
                "gravity must remain active while stiffness changes"
            );
            assert!(values[0].kp <= last_kp + 1e-10);
            assert!((values[0].kp - last_kp).abs() < SERVO_KP * 0.05);
            assert!((values[1].q - last_q[1]).abs() < 0.003);
            last_q = values.map(|s| s.q);
            last_kp = values[0].kp;
        }
        assert_eq!(last_kp, 0.0);
        let release_at = origin + Duration::from_secs(1);
        let released = core.measured(release_at).unwrap();
        core.apply(
            "release",
            Operation::TeachMode {
                enabled: false,
                transition_s: 0.4,
            },
            release_at,
            release_at,
        )
        .unwrap();
        for tick in 101..=150 {
            let values = teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                released,
            );
            assert!(core.enabled);
            assert!(values.iter().all(|s| s.mode == DriveMode::Servo));
            assert!(values[0].kp >= last_kp - 1e-10);
            assert!((values[0].kp - last_kp).abs() < SERVO_KP * 0.05);
            last_kp = values[0].kp;
        }
        assert_eq!(last_kp, SERVO_KP);
        assert_eq!(core.reference, released);
        assert_ne!(core.reference, initial);
        assert_eq!(core.tcp, tcp);
        assert_eq!(core.servo_tuning, baseline);
        let finish_at = origin + Duration::from_millis(1500);
        core.apply(
            "finish",
            Operation::TeachFinish {
                recording_request_id: None,
            },
            finish_at,
            finish_at,
        )
        .unwrap();
        teaching_tick(
            &mut core,
            &sent,
            finish_at + Duration::from_millis(10),
            released,
        );
        assert!(core.teaching.is_none());
        assert!(core.absolute_hold && core.enabled);
        let capture = core.last_teaching.as_ref().unwrap();
        assert_eq!(capture.summary.outcome, "completed");
        assert!(capture.summary.sample_count >= 150);
        let data = capture.trajectory();
        assert_eq!(data.points.last().unwrap().q, released);
        assert!(data.points.windows(2).all(|p| p[0].t_ms < p[1].t_ms));
        assert!(
            core.results
                .iter()
                .any(|r| r.request_id == "finish" && r.status == "completed")
        );
    }

    #[test]
    fn teaching_mode_persists_without_button_renewal_and_lease_loss_restores_hold() {
        let origin = Instant::now();
        let (mut core, _directory, sent) = calibration_fixture(origin);
        let initial = core.reference;
        core.apply(
            "mode",
            Operation::TeachMode {
                enabled: true,
                transition_s: 0.05,
            },
            origin,
            origin,
        )
        .unwrap();
        assert!(
            core.teaching.is_none(),
            "mode changes must not create a recording"
        );
        for tick in 1..=100 {
            let mut actual = initial;
            actual[0] += tick as f64 * 0.0002;
            teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                actual,
            );
        }
        assert_eq!(core.teaching_stiffness, 0.0);
        assert!(core.teaching_mode.is_some() && core.teaching.is_none());
        let at = origin + Duration::from_secs(1);
        let measured = core.measured(at).unwrap();
        let reference = core.reference;
        core.apply(
            "same-mode",
            Operation::TeachMode {
                enabled: true,
                transition_s: 0.05,
            },
            at,
            at,
        )
        .unwrap();
        assert_eq!(core.reference, reference);
        assert_eq!(
            core.teaching_stiffness, 0.0,
            "same mode must not restart transition"
        );
        core.apply(
            "record",
            Operation::TeachStart {
                name: "续录".into(),
            },
            at,
            at,
        )
        .unwrap();
        core.lease.as_mut().unwrap().expires = at;
        core.expire_lease(at);
        for tick in 101..=110 {
            teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                measured,
            );
        }
        assert!(core.teaching_mode.is_none() && core.teaching.is_none());
        assert!(core.enabled && core.absolute_hold);
        assert_eq!(core.reference, measured);
        assert_eq!(
            core.last_teaching.as_ref().unwrap().summary.outcome,
            "stopped"
        );
    }

    #[test]
    fn teaching_recording_survives_mode_changes_and_observes_disabled_feedback() {
        let origin = Instant::now();
        let (mut core, _directory, sent) = calibration_fixture(origin);
        let initial = core.reference;
        let generation = core.generation;
        core.enabled = false;
        core.apply(
            "record",
            Operation::TeachStart {
                name: "模式切换".into(),
            },
            origin,
            origin,
        )
        .unwrap();
        assert_eq!(core.generation, generation);
        assert_eq!(core.reference, initial);
        assert!(!core.enabled && core.teaching_mode.is_none());
        for tick in 1..=20 {
            let mut actual = initial;
            actual[0] += tick as f64 * 0.0001;
            teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                actual,
            );
        }
        assert!(core.teaching.as_ref().unwrap().recorder.sample_count() >= 20);
        core.enabled = true;
        let at = origin + Duration::from_millis(200);
        core.apply(
            "mode",
            Operation::TeachMode {
                enabled: true,
                transition_s: 0.1,
            },
            at,
            at,
        )
        .unwrap();
        let mut actual = initial;
        actual[1] += 0.02;
        for tick in 21..=50 {
            teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                actual,
            );
        }
        let at = origin + Duration::from_millis(500);
        core.apply(
            "hold",
            Operation::TeachMode {
                enabled: false,
                transition_s: 0.1,
            },
            at,
            at,
        )
        .unwrap();
        for tick in 51..=70 {
            teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                actual,
            );
        }
        assert!(core.teaching_mode.is_none() && core.teaching.is_some());
        let state = core.snapshot(origin + Duration::from_millis(700));
        assert!(state.teaching.is_none() && state.teaching_recording.unwrap().recording);
        assert_eq!(state.phase, Phase::Ready);
        assert!(state.action.is_none(), "a recorder alone is not a motion");
        let at = origin + Duration::from_millis(700);
        core.apply(
            "save",
            Operation::TeachFinish {
                recording_request_id: None,
            },
            at,
            at,
        )
        .unwrap();
        teaching_tick(&mut core, &sent, at + Duration::from_millis(10), actual);
        let data = core.last_teaching.as_ref().unwrap().trajectory();
        assert!(data.points.len() >= 70);
        assert_eq!(data.points[0].q, initial);
        assert_eq!(data.points.last().unwrap().q, actual);
        assert!(core.enabled);
    }

    #[test]
    fn teaching_recording_can_start_and_finish_during_joint_motion_without_changing_it() {
        let origin = Instant::now();
        let (mut core, _directory, sent) = calibration_fixture(origin);
        core.begin(
            "jog",
            Space::Joint,
            0,
            1,
            0.08,
            Frame::Base,
            false,
            0.0,
            None,
            1,
            origin,
        )
        .unwrap();
        let mut actual = core.reference;
        for tick in 1..=20 {
            teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                actual,
            );
            actual = core.reference;
        }
        let at = origin + Duration::from_millis(200);
        let reference = core.reference;
        let derivatives = (core.velocity, core.acceleration);
        let generation = core.generation;
        calibration_request(
            &mut core,
            "record",
            Operation::TeachStart {
                name: "点动实测".into(),
            },
            at,
        );
        assert_eq!(core.reference, reference);
        assert_eq!((core.velocity, core.acceleration), derivatives);
        assert_eq!(core.generation, generation);
        for tick in 21..=70 {
            teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                actual,
            );
            actual = core.reference;
        }
        assert!(core.teaching.is_some() && core.motion.is_some());
        let at = origin + Duration::from_millis(700);
        calibration_request(
            &mut core,
            "save",
            Operation::TeachFinish {
                recording_request_id: None,
            },
            at,
        );
        teaching_tick(&mut core, &sent, at + Duration::from_millis(10), actual);
        assert!(core.teaching.is_none() && core.motion.is_some());
        assert_eq!(core.generation, generation);
        let data = core.last_teaching.as_ref().unwrap().trajectory();
        assert!(data.points.len() >= 50);
        assert!(data.points.last().unwrap().q[0] > data.points[0].q[0]);
    }

    #[test]
    fn teaching_recording_continues_across_stops_and_disable_then_saves_without_fresh_feedback() {
        let origin = Instant::now();
        let (mut core, _directory, sent) = calibration_fixture(origin);
        core.apply(
            "record",
            Operation::TeachStart {
                name: "连续点动".into(),
            },
            origin,
            origin,
        )
        .unwrap();
        let initial = core.reference;
        let mut tick = 0;
        for (name, direction) in [("first", 1), ("second", -1)] {
            let at = origin + Duration::from_millis(tick * 10);
            core.begin(
                name,
                Space::Joint,
                0,
                direction,
                0.08,
                Frame::Base,
                false,
                0.0,
                None,
                1,
                at,
            )
            .unwrap();
            for _ in 0..50 {
                tick += 1;
                let actual = core.reference;
                teaching_tick(
                    &mut core,
                    &sent,
                    origin + Duration::from_millis(tick * 10),
                    actual,
                );
            }
            let at = origin + Duration::from_millis(tick * 10);
            core.apply("stop", Operation::Stop, at, at).unwrap();
            for _ in 0..200 {
                tick += 1;
                let actual = core.reference;
                teaching_tick(
                    &mut core,
                    &sent,
                    origin + Duration::from_millis(tick * 10),
                    actual,
                );
                if !core.stopping {
                    break;
                }
            }
            assert!(core.teaching.is_some() && core.motion.is_none() && !core.stopping);
        }
        let at = origin + Duration::from_millis(tick * 10);
        core.apply(
            "disable",
            Operation::Disable {
                allow_release: true,
            },
            at,
            at,
        )
        .unwrap();
        assert!(core.teaching.is_some() && !core.enabled);
        for _ in 0..20 {
            tick += 1;
            let mut actual = initial;
            actual[1] += 0.02;
            teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                actual,
            );
        }
        let count = core.teaching.as_ref().unwrap().recorder.sample_count();
        let stale = origin + Duration::from_millis(tick * 10 + FEEDBACK_MS + 1);
        assert!(core.measured(stale).is_err());
        core.apply(
            "hold",
            Operation::TeachMode {
                enabled: false,
                transition_s: 0.4,
            },
            stale,
            stale,
        )
        .unwrap();
        core.apply(
            "save",
            Operation::TeachFinish {
                recording_request_id: None,
            },
            stale,
            stale,
        )
        .unwrap();
        core.advance(stale, 0.01);
        let data = core.last_teaching.as_ref().unwrap().trajectory();
        assert_eq!(
            data.points.len(),
            count,
            "missing feedback is not replaced by command positions"
        );
        assert_eq!(data.summary.outcome, "completed");
        assert_eq!(data.points.last().unwrap().q[1], initial[1] + 0.02);
        assert!(!core.enabled);
    }

    #[test]
    fn teaching_recording_commands_keep_ownership_but_do_not_race_mode_generations() {
        let origin = Instant::now();
        let (mut core, _directory, sent) = calibration_fixture(origin);
        let generation = core.generation;
        let client_id = core.lease.as_ref().unwrap().client_id.clone();
        let lease_id = core.lease.as_ref().unwrap().secret.clone();
        core.apply(
            "mode",
            Operation::TeachMode {
                enabled: true,
                transition_s: 0.1,
            },
            origin,
            origin,
        )
        .unwrap();
        assert_ne!(core.generation, generation);
        core.command(
            CommandRequest {
                request_id: "record".into(),
                client_id: client_id.clone(),
                lease_id: lease_id.clone(),
                generation,
                operation: Operation::TeachStart {
                    name: "同时切换与录制".into(),
                },
            },
            origin,
            origin,
        );
        assert!(core.teaching.is_some() && core.teaching_mode.is_some());
        let id = core.teaching.as_ref().unwrap().id.clone();
        core.command(
            CommandRequest {
                request_id: "not-owner".into(),
                client_id: "other".into(),
                lease_id: "invalid".into(),
                generation,
                operation: Operation::TeachFinish {
                    recording_request_id: None,
                },
            },
            origin,
            origin,
        );
        assert_eq!(core.results.front().unwrap().code, "not_owner");
        assert!(core.teaching.as_ref().unwrap().finish.is_none());
        let q = core.reference;
        for tick in 1..=20 {
            teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                q,
            );
        }
        let at = origin + Duration::from_millis(200);
        core.command(
            CommandRequest {
                request_id: "finish".into(),
                client_id: client_id.clone(),
                lease_id: lease_id.clone(),
                generation,
                operation: Operation::TeachFinish {
                    recording_request_id: None,
                },
            },
            at,
            at,
        );
        for tick in 21..=40 {
            teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                q,
            );
        }
        assert_eq!(core.last_teaching.as_ref().unwrap().summary.id, id);
        assert!(core.teaching.is_none() && core.teaching_mode.is_none() && core.enabled);
        // A retransmission with a newer context cannot start a second capture.
        core.command(
            CommandRequest {
                request_id: "record".into(),
                client_id,
                lease_id,
                generation: core.generation,
                operation: Operation::TeachStart {
                    name: "duplicate".into(),
                },
            },
            at,
            at,
        );
        assert!(core.teaching.is_none());
    }

    #[test]
    fn teaching_close_targets_an_unapplied_start_and_cannot_end_a_new_capture() {
        let origin = Instant::now();
        let (mut core, _directory, _sent) = calibration_fixture(origin);
        let generation = core.generation;
        calibration_request(
            &mut core,
            "close",
            Operation::TeachFinish {
                recording_request_id: Some("pending-start".into()),
            },
            origin,
        );
        assert!(core.teaching.is_none());
        assert_eq!(
            core.results
                .iter()
                .find(|result| result.request_id == "close")
                .unwrap()
                .status,
            "completed",
            "the newest matching result must remain terminal"
        );

        assert!(
            core.results
                .iter()
                .any(|result| result.request_id == "pending-start" && result.status == "cancelled")
        );
        core.command(
            CommandRequest {
                request_id: "pending-start".into(),
                client_id: core.lease.as_ref().unwrap().client_id.clone(),
                lease_id: core.lease.as_ref().unwrap().secret.clone(),
                generation,
                operation: Operation::TeachStart {
                    name: "迟到请求".into(),
                },
            },
            origin,
            origin,
        );
        assert!(
            core.teaching.is_none(),
            "closing must cancel the original start even if it arrives later"
        );
        calibration_request(
            &mut core,
            "new-start",
            Operation::TeachStart {
                name: "新记录".into(),
            },
            origin,
        );
        let id = core.teaching.as_ref().unwrap().id.clone();
        calibration_request(
            &mut core,
            "old-close",
            Operation::TeachFinish {
                recording_request_id: Some("pending-start".into()),
            },
            origin,
        );
        assert_eq!(core.teaching.as_ref().unwrap().id, id);
        assert!(core.teaching.as_ref().unwrap().finish.is_none());
    }

    #[test]
    fn teaching_recording_ids_survive_generic_history_eviction_for_the_current_lease() {
        let origin = Instant::now();
        let (mut core, _directory, sent) = calibration_fixture(origin);
        let generation = core.generation;
        calibration_request(
            &mut core,
            "record",
            Operation::TeachStart {
                name: String::new(),
            },
            origin,
        );
        calibration_request(
            &mut core,
            "finish",
            Operation::TeachFinish {
                recording_request_id: None,
            },
            origin,
        );
        let actual = core.reference;
        teaching_tick(&mut core, &sent, origin + Duration::from_millis(10), actual);
        for i in 0..300 {
            calibration_request(&mut core, &format!("stop-{i}"), Operation::Stop, origin);
        }
        let client_id = core.lease.as_ref().unwrap().client_id.clone();
        let lease_id = core.lease.as_ref().unwrap().secret.clone();
        core.command(
            CommandRequest {
                request_id: "record".into(),
                client_id: client_id.clone(),
                lease_id: lease_id.clone(),
                generation,
                operation: Operation::TeachStart {
                    name: "重试".into(),
                },
            },
            origin,
            origin,
        );
        assert!(core.teaching.is_none());
        calibration_request(
            &mut core,
            "new-record",
            Operation::TeachStart {
                name: "新记录".into(),
            },
            origin,
        );
        core.command(
            CommandRequest {
                request_id: "finish".into(),
                client_id,
                lease_id,
                generation,
                operation: Operation::TeachFinish {
                    recording_request_id: None,
                },
            },
            origin,
            origin,
        );
        assert!(
            core.teaching.as_ref().unwrap().finish.is_none(),
            "old finish cannot terminate the new recording"
        );
    }

    #[test]
    fn teaching_mode_reversal_completes_a_pending_recording_instead_of_waiting_forever() {
        let origin = Instant::now();
        let (mut core, _directory, sent) = calibration_fixture(origin);
        let q = core.reference;
        core.apply(
            "mode",
            Operation::TeachMode {
                enabled: true,
                transition_s: 0.2,
            },
            origin,
            origin,
        )
        .unwrap();
        core.apply(
            "record",
            Operation::TeachStart {
                name: String::new(),
            },
            origin,
            origin,
        )
        .unwrap();
        for tick in 1..=30 {
            teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                q,
            );
        }
        let at = origin + Duration::from_millis(300);
        core.apply(
            "save",
            Operation::TeachFinish {
                recording_request_id: None,
            },
            at,
            at,
        )
        .unwrap();
        assert!(core.teaching.as_ref().unwrap().finish.is_some());
        core.apply(
            "continue-mode",
            Operation::TeachMode {
                enabled: true,
                transition_s: 0.2,
            },
            at,
            at,
        )
        .unwrap();
        teaching_tick(&mut core, &sent, at + Duration::from_millis(10), q);
        assert!(core.teaching.is_none() && core.teaching_mode.is_some() && core.enabled);
        assert_eq!(
            core.last_teaching.as_ref().unwrap().summary.outcome,
            "completed"
        );
    }

    #[test]
    fn teaching_finish_waiters_observe_stop_or_fault_before_the_hold_transition_finishes() {
        for fault in [false, true] {
            let origin = Instant::now();
            let (mut core, _dir, sent) = calibration_fixture(origin);
            let q = core.reference;
            core.apply(
                "teach",
                Operation::TeachStart {
                    name: String::new(),
                },
                origin,
                origin,
            )
            .unwrap();
            core.apply(
                "press",
                Operation::TeachMode {
                    enabled: true,
                    transition_s: 0.2,
                },
                origin,
                origin,
            )
            .unwrap();
            for tick in 1..=30 {
                teaching_tick(
                    &mut core,
                    &sent,
                    origin + Duration::from_millis(tick * 10),
                    q,
                );
            }
            let at = origin + Duration::from_millis(300);
            for id in ["finish-a", "finish-b"] {
                core.apply(
                    id,
                    Operation::TeachFinish {
                        recording_request_id: None,
                    },
                    at,
                    at,
                )
                .unwrap();
            }
            if fault {
                core.fault("device_error", "simulated device fault");
            } else {
                core.end_motion("cancelled", "stopped", "停止");
                for tick in 31..=60 {
                    teaching_tick(
                        &mut core,
                        &sent,
                        origin + Duration::from_millis(tick * 10),
                        q,
                    );
                }
            }
            assert!(core.teaching.is_none());
            assert_eq!(
                core.last_teaching.as_ref().unwrap().summary.outcome,
                if fault { "failed" } else { "stopped" }
            );
            for id in ["teach", "finish-a", "finish-b"] {
                let result = core.results.iter().find(|r| r.request_id == id).unwrap();
                assert_eq!(result.status, if fault { "failed" } else { "cancelled" });
            }
            assert_eq!(core.enabled, !fault);
        }
    }

    #[test]
    fn teaching_zero_duration_replay_has_finite_progress_and_fallback_stop_stays_at_rest() {
        let origin = Instant::now();
        let (mut core, _directory, sent) = calibration_fixture(origin);
        let q = core.reference;
        let one = teaching::Trajectory {
            summary: TeachingSummary {
                id: "pose".into(),
                name: "示教姿态".into(),
                created_at_ms: 0,
                side: "right".into(),
                tcp: core.tcp.unwrap(),
                duration_ms: 0,
                sample_count: 1,
                outcome: "completed".into(),
            },
            points: vec![teaching::Point { t_ms: 0, q }],
        };
        let program = teaching::plan(
            &one,
            q,
            core.model.as_ref().unwrap(),
            core.motion_tuning,
            PlaybackOptions::default(),
        )
        .unwrap();
        assert_eq!(program.duration(), 0.0);
        core.apply(
            "pose",
            Operation::TeachingPrepared {
                program: Arc::new(program),
            },
            origin,
            origin,
        )
        .unwrap();
        assert_eq!(core.snapshot(origin).teaching.unwrap().progress, 1.0);
        assert!(
            serde_json::to_value(core.snapshot(origin)).unwrap()["teaching"]["progress"]
                .is_number()
        );
        teaching_tick(&mut core, &sent, origin + Duration::from_millis(10), q);
        assert!(core.replay.is_none() && core.enabled);

        // A sender fallback has zero actual trajectory velocity. Its abandoned
        // playback's nonzero acceleration must not restart movement on Stop.
        let trajectory = teaching::Trajectory {
            summary: one.summary,
            points: (0..101)
                .map(|i| {
                    let mut point = q;
                    point[0] += 0.04 * (i as f64 / 100.0).powi(2);
                    teaching::Point {
                        t_ms: i * 20,
                        q: point,
                    }
                })
                .collect(),
        };
        let program = Arc::new(
            teaching::plan(
                &trajectory,
                q,
                core.model.as_ref().unwrap(),
                core.motion_tuning,
                PlaybackOptions::default(),
            )
            .unwrap(),
        );
        let (elapsed, sample) = (1..200)
            .find_map(|i| {
                let t = i as f64 * 0.01;
                let s = program.sample(t);
                s.ddq.iter().any(|a| a.abs() > 0.001).then_some((t, s))
            })
            .unwrap();
        core.replay = Some(TeachingReplay {
            request_id: "fallback".into(),
            started: origin,
            program,
        });
        core.reference = sample.q;
        core.velocity = sample.dq;
        core.acceleration = sample.ddq;
        let at = origin + Duration::from_secs_f64(elapsed);
        sent.send_replace(Some(SentReference {
            at,
            completed_at: at,
            generation: core.generation,
            epoch: core.reference_version,
            q: sample.q,
            dq: [0.0; 7],
            calibration_phase: None,
            fallback_hold: true,
        }));
        core.end_motion("cancelled", "stopped", "停止");
        assert_eq!(core.velocity, [0.0; 7]);
        assert_eq!(core.acceleration, [0.0; 7]);
        assert!(!core.stopping);
        assert_eq!(core.reference, sample.q);
    }

    #[test]
    fn teaching_saved_replay_samples_send_time_stops_and_preserves_tcp() {
        let origin = Instant::now();
        let (mut core, _directory, sent) = calibration_fixture(origin);
        let initial = core.reference;
        let tcp = core.tcp.unwrap();
        let summary = TeachingSummary {
            id: "replay".into(),
            name: "示教".into(),
            created_at_ms: 0,
            side: "right".into(),
            tcp,
            duration_ms: 2000,
            sample_count: 101,
            outcome: "completed".into(),
        };
        let trajectory = teaching::Trajectory {
            summary,
            points: (0..101)
                .map(|i| {
                    let mut q = initial;
                    q[0] += 0.05 * (std::f64::consts::PI * i as f64 / 100.0).sin();
                    teaching::Point { t_ms: i * 20, q }
                })
                .collect(),
        };
        let program = teaching::plan(
            &trajectory,
            initial,
            core.model.as_ref().unwrap(),
            core.motion_tuning,
            PlaybackOptions::default(),
        )
        .unwrap();
        core.apply(
            "replay",
            Operation::TeachingPrepared {
                program: Arc::new(program),
            },
            origin,
            origin,
        )
        .unwrap();
        let generation = core.generation;
        for tick in 1..=60 {
            let actual = core.reference;
            if tick == 20 {
                let at = origin + Duration::from_millis(190);
                core.apply(
                    "replay-record",
                    Operation::TeachStart {
                        name: "重放实测".into(),
                    },
                    at,
                    at,
                )
                .unwrap();
                let state = core.snapshot(at);
                assert!(state.teaching.is_some() && state.teaching_recording.is_some());
                assert!(state.teaching_mode.is_none());
            }
            teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                actual,
            );
        }
        let at = origin + Duration::from_millis(600);
        core.apply(
            "save-replay-record",
            Operation::TeachFinish {
                recording_request_id: None,
            },
            at,
            at,
        )
        .unwrap();
        core.advance_teaching_recording(at);
        assert!(core.teaching.is_none() && core.replay.is_some());
        assert_eq!(core.generation, generation);
        let recorded = core.last_teaching.as_ref().unwrap().trajectory();
        assert_eq!(
            recorded.points.last().unwrap().q,
            core.measured(at).unwrap()
        );
        assert!(recorded.points.len() >= 40);
        let outgoing = core.outgoing_reference();
        let expected = core.replay.as_ref().unwrap().program.sample(0.607);
        let sampled = outgoing
            .sample(at + Duration::from_millis(7), None)
            .unwrap();
        assert_eq!(sampled.q, expected.q);
        assert_eq!(sampled.dq, expected.dq);
        assert!(core.velocity.iter().any(|v| v.abs() > 1e-6));
        sent.send_replace(Some(SentReference {
            at: at + Duration::from_millis(7),
            completed_at: at + Duration::from_millis(7),
            generation: core.generation,
            epoch: core.reference_version,
            q: sampled.q,
            dq: sampled.dq,
            calibration_phase: None,
            fallback_hold: false,
        }));
        core.end_motion("cancelled", "stopped", "停止");
        assert!(core.replay.is_none());
        assert_eq!(core.reference, sampled.q);
        assert_eq!(core.velocity, sampled.dq);
        for tick in 61..=1000 {
            let actual = core.reference;
            teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                actual,
            );
            if !core.stopping {
                break;
            }
        }
        assert!(!core.stopping && core.enabled);
        assert_eq!(core.tcp, Some(tcp));
        assert!(core.motion.is_none() && core.replay.is_none());
        assert_eq!(
            core.results
                .iter()
                .find(|r| r.request_id == "replay")
                .unwrap()
                .status,
            "cancelled"
        );
        core.begin_return("park", initial, 0.1, false, core.control_at)
            .unwrap();
        assert!(matches!(
            core.motion.as_ref().unwrap().mode,
            MotionMode::Return {
                disconnect_after: false
            }
        ));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn teaching_finished_capture_is_readable_before_sqlite_commit_and_survives_restart() {
        let origin = Instant::now();
        let (mut core, directory, sent) = calibration_fixture(origin);
        let journal = core.journal.clone();
        let deadline = Instant::now() + Duration::from_secs(2);
        while !journal.stats().healthy {
            assert!(Instant::now() < deadline);
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        let db = rusqlite::Connection::open(directory.path().join("events.sqlite3")).unwrap();
        db.execute_batch("BEGIN IMMEDIATE").unwrap();
        core.apply(
            "teach",
            Operation::TeachStart {
                name: "离线录制".into(),
            },
            origin,
            origin,
        )
        .unwrap();
        core.apply(
            "press",
            Operation::TeachMode {
                enabled: true,
                transition_s: 0.05,
            },
            origin,
            origin,
        )
        .unwrap();
        let initial = core.reference;
        for tick in 1..=50 {
            let mut q = initial;
            q[0] += 0.0002 * tick as f64;
            teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                q,
            );
        }
        let actual = core.measured(origin + Duration::from_millis(500)).unwrap();
        core.apply(
            "finish",
            Operation::TeachFinish {
                recording_request_id: None,
            },
            origin + Duration::from_millis(500),
            origin + Duration::from_millis(500),
        )
        .unwrap();
        for tick in 51..=60 {
            teaching_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                actual,
            );
        }
        assert!(core.teaching.is_none() && core.enabled);
        let capture = core.last_teaching.as_ref().unwrap().clone();
        let id = capture.summary.id.clone();
        let committed: i64 = db
            .query_row("SELECT count(*) FROM teachings WHERE id=?1", [&id], |row| {
                row.get(0)
            })
            .unwrap();
        assert_eq!(committed, 0, "the write transaction is still held");
        let _transmit_sink = core.bus.as_ref().unwrap().test_transmit_sink();
        let arm = ArmHandle::spawn_core(core).unwrap();
        let server = crate::server::launch_at(
            arm.clone(),
            journal.clone(),
            "teaching-test".into(),
            std::net::SocketAddr::from(([127, 0, 0, 1], 0)),
        )
        .await
        .unwrap();
        let client = reqwest::Client::new();
        let remote: teaching::Trajectory = client
            .get(format!(
                "{}/v1/teachings/{id}?download=true",
                server.base_url()
            ))
            .bearer_auth("teaching-test")
            .send()
            .await
            .unwrap()
            .error_for_status()
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(remote.summary.id, id);
        assert_eq!(remote.summary.sample_count, capture.summary.sample_count);
        assert_eq!(remote.points.last().unwrap().q, actual);
        let preview: teaching::Preview = client
            .post(format!("{}/v1/teachings/{id}/preview", server.base_url()))
            .bearer_auth("teaching-test")
            .json(&serde_json::json!({"speed_scale":2.0}))
            .send()
            .await
            .unwrap()
            .error_for_status()
            .unwrap()
            .json()
            .await
            .unwrap();
        assert!(!preview.points.is_empty());
        assert!(arm.snapshot().action.is_none());
        db.execute_batch("ROLLBACK").unwrap();
        server.shutdown().await;
        assert!(arm.shutdown(Duration::from_secs(2)));
        assert!(journal.shutdown(Duration::from_secs(3)));
        let reopened = Journal::start(directory.path().join("events.sqlite3")).unwrap();
        let restored = reopened.teaching(&id).unwrap().unwrap();
        let expected = capture.trajectory();
        assert_eq!(restored.summary, expected.summary);
        assert_eq!(restored.points.len(), expected.points.len());
        for (restored, expected) in restored.points.iter().zip(&expected.points) {
            assert_eq!(restored.t_ms, expected.t_ms);
            for (a, b) in restored.q.into_iter().zip(expected.q) {
                assert!((a - b).abs() <= 8.0 * f64::EPSILON * a.abs().max(b.abs()).max(1.0));
            }
        }
        assert!(reopened.shutdown(Duration::from_secs(2)));
    }

    #[test]
    fn independently_fresh_joint_samples_do_not_need_a_second_span_gate() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.feedback[0].as_mut().unwrap().at = now - Duration::from_millis(180);
        assert!(core.measured(now).is_ok());
        core.feedback[0].as_mut().unwrap().at = now - Duration::from_millis(FEEDBACK_MS + 1);
        assert_eq!(core.measured(now).unwrap_err().code, "feedback_stale");
    }

    #[test]
    fn failed_clear_does_not_retry_or_replace_the_original_io_error() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        let message = "serial write: device disconnected";
        core.bus = Some(BusHandle::test_failed_bus(message));
        core.clear_request = Some(("clear".into(), now));
        core.clear_pending = [true; 7];
        core.poll(now);
        assert!(core.clear_request.is_none());
        assert_eq!(core.clear_pending, [false; 7]);
        core.settle_transitions(now + Duration::from_secs(2));
        assert_eq!(core.error.as_ref().unwrap().message, message);
        assert_eq!(core.results.front().unwrap().request_id, "clear");
        assert_eq!(core.results.front().unwrap().status, "failed");
    }

    #[test]
    fn clear_fault_accepts_healthy_feedback_through_the_serial_reader() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.bus = Some(BusHandle::test_clear_response_bus());
        core.error = Some(ApiError::new("device_error", "recorded device fault"));
        core.phase = Phase::Fault;
        core.apply("clear", Operation::ClearFault, now, now)
            .unwrap();
        let until = Instant::now() + Duration::from_millis(200);
        while core.clear_request.is_some() && Instant::now() < until {
            core.poll(Instant::now());
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(core.error.is_none(), "{:?}", core.error);
        assert!(core.clear_request.is_none());
        assert_eq!(core.results.front().unwrap().code, "cleared");
        assert!(!core.enabled);
    }

    #[test]
    fn clear_fault_timeout_cancels_a_not_yet_sent_request() {
        let now = Instant::now();
        let (mut core, _directory, _sent) = calibration_fixture(now);
        core.enabled = false;
        core.apply("clear", Operation::ClearFault, now, now)
            .unwrap();
        core.settle_transitions(now + Duration::from_secs(2));
        assert!(core.clear_request.is_none());
        assert_eq!(core.clear_pending, [false; 7]);
    }

    #[test]
    fn tcp_configuration_uses_finite_geometry_without_a_two_metre_cap() {
        let now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.apply(
            "tcp",
            Operation::ConfigureTcp {
                tcp: Pose {
                    translation: [2.5, -3.0, 0.1],
                    rotation: [0.0, 0.0, 0.0, 1e-12],
                },
            },
            now,
            now,
        )
        .unwrap();
        assert_eq!(core.tcp.unwrap().translation, [2.5, -3.0, 0.1]);
        assert_eq!(core.tcp.unwrap().rotation, [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn continuous_input_accepts_positive_durations_and_rejects_expired_commands() {
        let now = Instant::now();
        for milliseconds in [1, 49, 501, 5_000] {
            assert_eq!(
                input_deadline(now, milliseconds, now).unwrap(),
                now + Duration::from_millis(milliseconds)
            );
        }
        assert_eq!(input_deadline(now, 0, now).unwrap_err().code, "invalid_ttl");
        assert_eq!(
            input_deadline(now, 1, now + Duration::from_millis(2))
                .unwrap_err()
                .code,
            "stale_input"
        );
    }
    fn calibration_fixture(
        now: Instant,
    ) -> (
        Core,
        tempfile::TempDir,
        watch::Sender<Option<SentReference>>,
    ) {
        let (mut core, directory) = observed_core(now);
        core.enabled = true;
        core.tcp = Some(Pose {
            translation: [0.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
        });
        core.port = Some("calibration-simulation".into());
        core.phase = Phase::Ready;
        core.control(
            ControlRequest {
                action: "acquire".into(),
                client_id: "calibration-test".into(),
                lease_id: None,
            },
            now,
        )
        .unwrap();
        let (bus, sent) = BusHandle::test_idle_bus();
        core.bus = Some(bus);
        sent.send_replace(Some(SentReference {
            at: now,
            completed_at: now,
            generation: core.generation,
            epoch: core.reference_version,
            q: core.reference,
            dq: core.velocity,
            calibration_phase: None,
            fallback_hold: false,
        }));
        (core, directory, sent)
    }
    #[test]
    fn planned_stop_waits_for_the_in_flight_hold_before_publishing_its_first_frame() {
        let start = Instant::now() - Duration::from_millis(500);
        let (mut core, _dir, _sent) = calibration_fixture(start);
        let mut target = core.reference;
        target[0] += 0.2;
        core.begin_calibration_leg("in-flight", target, 0.2, None, start)
            .unwrap();
        let (bus, in_flight, release) =
            crate::protocol::paused_hold_bus(core.reference_epoch.clone());
        let published = bus.test_transmit_sink();
        core.bus = Some(bus);
        let tick = |core: &mut Core| {
            let at = Instant::now();
            follow_reference(core, at);
            core.advance_motion(at, 0.01);
            core.poll(at);
        };
        let receipt = |core: &Core, predicate: fn(SentReference) -> bool| {
            let deadline = Instant::now() + Duration::from_secs(3);
            loop {
                if let std::task::Poll::Ready(Some(sent)) =
                    core.bus.as_ref().unwrap().settled_reference()
                    && predicate(sent)
                {
                    break sent;
                }
                assert!(
                    Instant::now() < deadline,
                    "writer did not commit the expected batch"
                );
                std::thread::yield_now();
            }
        };
        tick(&mut core);
        // The fake port accepts J0 of the stale hold, then blocks before J1.
        in_flight.recv_timeout(Duration::from_secs(3)).unwrap();
        let moving = core.bus.as_ref().unwrap().latest_reference().unwrap();
        assert!(moving.dq[0] > 0.0 && !moving.fallback_hold);
        core.apply("stop", Operation::Stop, Instant::now(), Instant::now())
            .unwrap();
        tick(&mut core);
        assert!(core.stopping);
        assert_eq!(published().unwrap().epoch, moving.epoch);

        release.send(()).unwrap();
        let held = receipt(&core, |s| s.fallback_hold);
        assert_eq!((held.q, held.dq), (moving.q, [0.0; 7]));
        tick(&mut core);
        assert!(!core.stopping && core.enabled && core.error.is_none());
        let first = receipt(&core, |s| !s.fallback_hold);
        assert_eq!(first.epoch, core.reference_version);
        assert_eq!(
            (first.q, first.dq, core.acceleration),
            (held.q, [0.0; 7], [0.0; 7])
        );
    }

    #[test]
    fn calibration_sender_recovers_on_the_approved_leg_after_repeated_holds() {
        for stop_during_recovery in [false, true] {
            exercise_calibration_recovery(stop_during_recovery);
        }
    }

    fn exercise_calibration_recovery(stop_during_recovery: bool) {
        let start = Instant::now();
        let (mut core, _dir, sent) = calibration_fixture(start);
        core.lease.as_mut().unwrap().expires = start + Duration::from_secs(120);
        let initial = core.reference;
        let mut middle = initial;
        middle[0] += 0.05;
        let mut target = middle;
        target[0] += 0.02;
        target[1] += 0.03;
        let leg = crate::calibration_spline::TimedLeg::fit(
            &[(0.0, initial), (0.01, middle), (0.02, target)],
            0.03,
            core.motion_tuning,
            core.joint_speeds(),
        )
        .unwrap();
        let duration = leg.duration();
        core.begin_calibration_leg(
            "resume",
            initial,
            0.03,
            Some((Space::Joint, 0, Arc::new(leg))),
            start,
        )
        .unwrap();
        let mut write = BusHandle::test_servo_writer();
        let tick = |core: &mut Core, at| {
            follow_reference(core, at);
            core.advance_motion(at, 0.01);
        };
        let mut now = start + Duration::from_secs_f64(duration * 0.25);
        tick(&mut core, now);
        let mut source = core.outgoing_reference();
        // The serial sample is later than the last control sample.
        now += Duration::from_millis(8);
        let mut moving = write(source.clone(), core.desired_setpoints(), now);
        assert!(moving.dq[0] > 0.0);
        sent.send_replace(Some(moving));
        let mut previous_ack: Option<Reference> = None;
        for _ in 0..2 {
            now += Duration::from_millis(80);
            let held = write(source.clone(), core.desired_setpoints(), now);
            assert!(held.fallback_hold);
            assert_eq!((held.q, held.dq), (moving.q, [0.0; 7]));
            // Fresh publication races the writer's hold receipt.
            now += Duration::from_millis(10);
            tick(&mut core, now);
            let late = write(core.outgoing_reference(), core.desired_setpoints(), now);
            assert!(
                late.fallback_hold,
                "a fresh timestamp alone released the hold"
            );
            assert_eq!(late.q, held.q);
            sent.send_replace(Some(late));
            if let Some(mut ack) = previous_ack.take() {
                now += Duration::from_millis(10);
                ack.at = now;
                assert!(write(ack, core.desired_setpoints(), now).fallback_hold);
            }

            // Wait beyond the original endpoint and delay the first authorized send.
            now += Duration::from_secs_f64(duration + 0.1);
            tick(&mut core, now);
            assert!(core.motion.is_some());
            assert_eq!(
                (core.reference, core.velocity, core.acceleration),
                (held.q, [0.0; 7], [0.0; 7])
            );
            source = core.outgoing_reference();
            previous_ack = Some(source.clone());
            now += Duration::from_millis(30);
            moving = write(source.clone(), core.desired_setpoints(), now);
            assert!(!moving.fallback_hold);
            assert_eq!((moving.q, moving.dq), (held.q, [0.0; 7]));
            sent.send_replace(Some(moving));
            for _ in 0..30 {
                now += Duration::from_millis(10);
                let previous_a = core.acceleration;
                tick(&mut core, now);
                source = core.outgoing_reference();
                let previous = moving;
                moving = write(source.clone(), core.desired_setpoints(), now);
                assert!(!moving.fallback_hold);
                assert_eq!((core.reference, core.velocity), (moving.q, moving.dq));
                for (i, old_acceleration) in previous_a.into_iter().enumerate() {
                    let limits = core.motion_tuning;
                    assert!(core.acceleration[i].abs() <= limits.acceleration);
                    assert!((core.acceleration[i] - old_acceleration).abs() <= limits.jerk * 0.01);
                    let trapezoid = (moving.dq[i] + previous.dq[i]) * 0.005;
                    assert!(
                        (moving.q[i] - previous.q[i] - trapezoid).abs()
                            <= limits.jerk * 0.01_f64.powi(3) / 12.0 + 1e-14
                    );
                }
                sent.send_replace(Some(moving));
            }
            assert!(moving.q[0] > held.q[0] && moving.dq[0] > 0.0);
        }
        if stop_during_recovery {
            now += Duration::from_millis(7);
            moving = write(source, core.desired_setpoints(), now);
            let phase = moving.calibration_phase.unwrap();
            let plan = core.motion.as_ref().unwrap().planned.as_ref().unwrap();
            let expected = phase.sample(plan, now);
            assert_ne!(expected.ddq, core.acceleration);
            sent.send_replace(Some(moving));
            core.end_motion("cancelled", "stopped", "停止");
            assert!(core.stopping && core.motion.is_none());
            assert_eq!(
                (core.reference, core.velocity, core.acceleration),
                (moving.q, moving.dq, expected.ddq)
            );
            return;
        }
        while core.motion.is_some() {
            assert!(now < start + Duration::from_secs(100));
            now += Duration::from_millis(10);
            tick(&mut core, now);
            moving = write(core.outgoing_reference(), core.desired_setpoints(), now);
            sent.send_replace(Some(moving));
        }
        assert_eq!(
            (moving.q, moving.dq, core.acceleration),
            (target, [0.0; 7], [0.0; 7])
        );
    }
    fn calibration_request(core: &mut Core, id: &str, mut operation: Operation, now: Instant) {
        if let Operation::CalibrateStart { settings } = &mut operation
            && let Some(program) = &mut settings.cartesian
            && program.timed.is_empty()
        {
            Arc::make_mut(program)
                .prepare(core.model.as_ref().unwrap())
                .unwrap();
        }
        let lease = core.lease.as_ref().unwrap();
        let request = CommandRequest {
            request_id: id.into(),
            client_id: lease.client_id.clone(),
            lease_id: lease.secret.clone(),
            generation: core.generation,
            operation,
        };
        core.command(request, now, now);
    }
    fn calibration_settings() -> CalibrationSettings {
        CalibrationSettings {
            axis: Some(1),
            excursion: 0.1,
            speed: 0.02,
            max_friction: Some(0.1),
            speeds: Vec::new(),
            cartesian: None,
        }
    }
    fn joint_calibration_test_ticks(core: &Core, settings: &CalibrationSettings) -> u64 {
        let axis = settings.axis.unwrap_or(0);
        let mut previous = core.reference;
        let speed_limit = core
            .joint_speeds()
            .into_iter()
            .fold(f64::INFINITY, f64::min);
        let per_trial: u64 = calibration::legs(settings, axis, previous)
            .iter()
            .map(|leg| {
                let distance = previous
                    .iter()
                    .zip(leg.target)
                    .map(|(a, b)| (a - b).abs())
                    .fold(0.0_f64, f64::max);
                previous = leg.target;
                let duration = trajectory::segment_duration(
                    distance,
                    leg.speed.min(speed_limit),
                    core.motion_tuning.acceleration,
                    core.motion_tuning.jerk,
                );
                // Configure, begin, transition and the final measured target tick.
                (duration / CONTROL_PERIOD.as_secs_f64()).ceil() as u64 + 4
            })
            .sum();
        let axes = if settings.axis.is_some() { 1 } else { 7 };
        (per_trial + 2) * axes * 3 + 2
    }

    #[test]
    fn calibration_feedback_gap_holds_until_all_seven_reply_then_restarts_at_rest() {
        let (mut core, _dir, sent) = moving_feedback_fixture(Instant::now());
        let last_rx = core.control_at;
        let at = |ms| last_rx + Duration::from_millis(ms);
        let mut write = BusHandle::test_servo_writer();
        core.advance(at(40), 0.04);
        let source = core.outgoing_reference();
        let moving = write(source.clone(), core.desired_setpoints(), at(41));
        assert!(moving.dq.iter().any(|v| v.abs() > 0.0));
        sent.send_replace(Some(moving));
        let held = write(source, core.desired_setpoints(), at(51));
        assert!(
            held.fallback_hold,
            "refreshing control time extended the lifetime of old feedback"
        );
        assert_eq!((held.q, held.dq), (moving.q, [0.0; 7]));
        sent.send_replace(Some(held));
        let count = core.calibration.as_ref().unwrap().points.captured();
        core.advance(at(60), 0.02);
        let queue = core.bus.as_mut().unwrap().test_feedback_queue();
        for axis in 0..6 {
            queue(core.feedback[axis].as_ref().unwrap().value.clone(), at(160));
        }
        core.advance(at(168), 0.108);
        let pinned = write(core.outgoing_reference(), core.desired_setpoints(), at(169));
        assert!(pinned.fallback_hold && core.enabled);
        assert_eq!(
            (core.reference, core.velocity, pinned.q),
            (held.q, [0.0; 7], held.q)
        );
        assert_eq!(core.calibration.as_ref().unwrap().points.captured(), count);
        sent.send_replace(Some(pinned));
        queue(core.feedback[6].as_ref().unwrap().value.clone(), at(175));
        core.advance(at(180), 0.012);
        let first = write(core.outgoing_reference(), core.desired_setpoints(), at(181));
        assert!(!first.fallback_hold && core.error.is_none());
        assert_eq!((first.q, first.dq), (held.q, [0.0; 7]));
        sent.send_replace(Some(first));
        core.advance(at(190), 0.01);
        assert!(core.velocity.iter().any(|v| v.abs() > 0.0));
    }

    #[test]
    fn calibration_feedback_queue_continuous_during_a_late_control_tick() {
        let origin = Instant::now();
        let (mut core, _dir, _sent) = calibration_fixture(origin);
        calibration_request(
            &mut core,
            "queue",
            Operation::CalibrateStart {
                settings: calibration_settings(),
            },
            origin,
        );
        let queue = core.bus.as_mut().unwrap().test_feedback_queue();
        for ms in (30..=240).step_by(30) {
            for axis in 0..7 {
                queue(
                    core.feedback[axis].as_ref().unwrap().value.clone(),
                    origin + Duration::from_millis(ms),
                );
            }
        }
        core.advance(origin + Duration::from_millis(245), 0.245);
        assert!(core.enabled && core.error.is_none());
        assert!(core.calibration.as_ref().unwrap().feedback_wait.is_none());
    }

    fn moving_feedback_fixture(
        origin: Instant,
    ) -> (
        Core,
        tempfile::TempDir,
        watch::Sender<Option<SentReference>>,
    ) {
        let (mut core, dir, sent) = calibration_fixture(origin);
        calibration_request(
            &mut core,
            "feedback",
            Operation::CalibrateStart {
                settings: calibration_settings(),
            },
            origin,
        );
        for i in 0..25 {
            calibration_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(i * 10),
                origin,
                false,
            );
        }
        (core, dir, sent)
    }

    #[test]
    fn calibration_feedback_recovered_gap_is_not_a_current_timeout() {
        let (mut core, _dir, sent) = moving_feedback_fixture(Instant::now());
        let last = core.control_at;
        let at = |ms| last + Duration::from_millis(ms);
        let mut write = BusHandle::test_servo_writer();
        let source = core.outgoing_reference();
        let moving = write(source.clone(), core.desired_setpoints(), at(1));
        let held = write(source, core.desired_setpoints(), at(60));
        assert!(held.fallback_hold && moving.dq[1] > 0.0);
        sent.send_replace(Some(held));
        let queue = core.bus.as_mut().unwrap().test_feedback_queue();
        for axis in 0..7 {
            queue(core.feedback[axis].as_ref().unwrap().value.clone(), at(240));
        }
        let windows = core.calibration.as_ref().unwrap().collectors[1]
            .finish()
            .unwrap()
            .valid_ms;
        core.advance(at(245), 0.245);
        assert!(core.enabled && core.error.is_none());
        // A past hold proves a pause, but request a current powered hold receipt
        // before resuming if the serial writer was also late.
        let confirmed = write(core.outgoing_reference(), core.desired_setpoints(), at(246));
        assert!(confirmed.fallback_hold);
        sent.send_replace(Some(confirmed));
        core.advance(at(247), 0.002);
        let first = write(core.outgoing_reference(), core.desired_setpoints(), at(248));
        assert_eq!((first.q, first.dq), (held.q, [0.0; 7]));
        assert!(!first.fallback_hold);
        sent.send_replace(Some(first));
        for axis in 0..7 {
            queue(core.feedback[axis].as_ref().unwrap().value.clone(), at(250));
        }
        core.advance(at(251), 0.006);
        assert!(core.velocity[1] > 0.0);
        assert_eq!(
            core.calibration.as_ref().unwrap().collectors[1]
                .finish()
                .unwrap()
                .valid_ms,
            windows
        );
    }

    #[test]
    fn calibration_feedback_timeout_lists_every_axis_still_missing_after_drain() {
        let (mut core, _dir, _) = moving_feedback_fixture(Instant::now());
        let last = core.control_at;
        let at = |ms| last + Duration::from_millis(ms);
        core.advance(at(60), 0.06);
        let queue = core.bus.as_mut().unwrap().test_feedback_queue();
        for axis in 0..5 {
            queue(core.feedback[axis].as_ref().unwrap().value.clone(), at(240));
        }
        core.advance(at(245), 0.185);
        let e = core.error.as_ref().unwrap();
        assert_eq!(e.code, "feedback_stale");
        assert!(
            e.message.contains("J6 245 ms")
                && e.message.contains("J7 245 ms")
                && e.message.contains("185 ms"),
            "{}",
            e.message
        );
        assert!(!e.message.contains("J1"), "{}", e.message);
        assert_eq!(
            core.last_calibration_report.as_ref().unwrap().message,
            e.message
        );
        follow_reference(&mut core, at(250));
        core.advance(at(250), 0.005);
        assert!(!core.enabled && core.calibration.is_none());
    }

    #[test]
    fn calibration_feedback_device_fault_is_latched_across_new_healthy_frames() {
        let (mut core, _dir, _) = moving_feedback_fixture(Instant::now());
        let last = core.control_at;
        let queue = core.bus.as_mut().unwrap().test_feedback_queue();
        let mut fault = core.feedback[5].as_ref().unwrap().value.clone();
        fault.timeout = true;
        queue(fault, last + Duration::from_millis(240));
        for axis in 0..7 {
            queue(
                core.feedback[axis].as_ref().unwrap().value.clone(),
                last + Duration::from_millis(241),
            );
        }
        core.advance(last + Duration::from_millis(245), 0.245);
        assert_eq!(core.error.as_ref().unwrap().code, "device_timeout");
        assert!(!core.enabled && core.calibration.is_none());
        assert_eq!(
            core.last_calibration_report.as_ref().unwrap().outcome,
            "failed"
        );
    }

    #[test]
    fn planned_stop_before_feedback_wait_uses_the_final_write_and_the_stop_clock() {
        // Request and handoff ages are measured from the oldest of seven real replies.
        for (stop_ms, tick_ms, handoff_ms, in_flight, refresh_rx, expect_hold) in [
            (51, 52, 52, false, false, true),
            (51, 53, 53, true, true, true),
            (49, 52, 52, true, false, true),
            (49, 50, 50, true, false, false),
            (49, 50, 52, true, false, true),
        ] {
            let (mut core, _dir, sent) = moving_feedback_fixture(Instant::now());
            let last_rx = core.control_at;
            let at = |ms| last_rx + Duration::from_millis(ms);
            core.advance(at(40), 0.04);
            let source = core.outgoing_reference();
            let mut write = BusHandle::test_servo_writer();
            let previous = write(source.clone(), core.desired_setpoints(), at(41));
            sent.send_replace(Some(previous));
            let gate = core.bus.as_ref().unwrap().test_reference_write();
            let pending = in_flight.then(|| gate.lock().unwrap());
            let mut moving = write(source, core.desired_setpoints(), at(48));
            assert!(moving.dq[1] > 0.0 && moving.q[1] > previous.q[1]);
            if !in_flight {
                sent.send_replace(Some(moving));
            }
            let queue = core.bus.as_mut().unwrap().test_feedback_queue();
            for axis in &core.feedback[..6] {
                queue(axis.as_ref().unwrap().value.clone(), at(48));
            }
            calibration_request(&mut core, "stop", Operation::Stop, at(stop_ms));
            if in_flight {
                assert!(core.stopping);
                moving.completed_at = at(handoff_ms);
                sent.send_replace(Some(moving));
                drop(pending);
            }
            if refresh_rx {
                follow_reference(&mut core, at(handoff_ms));
            }
            core.advance(at(tick_ms), 0.002);
            let first = write(
                core.outgoing_reference(),
                core.desired_setpoints(),
                at(handoff_ms + 1),
            );
            assert!(core.enabled && core.error.is_none() && !first.fallback_hold);
            if expect_hold {
                assert_eq!(
                    (first.q, first.dq, core.acceleration),
                    (moving.q, [0.0; 7], [0.0; 7]),
                    "Stop at {stop_ms} ms, handoff at {handoff_ms} ms"
                );
            } else {
                assert!(first.dq[1] > 0.0 && first.q[1] > moving.q[1] && core.stopping);
            }
        }
    }

    #[test]
    fn calibration_feedback_wait_cannot_resume_after_stop_or_release() {
        for release in [false, true] {
            let (mut core, _dir, sent) = moving_feedback_fixture(Instant::now());
            let moving = sent.borrow().unwrap();
            assert!(moving.dq[1] > 0.0);
            let now = core.control_at + Duration::from_millis(60);
            core.advance(now, 0.06);
            // The stop races the first zero-speed hold write.
            assert!(core.calibration.as_ref().unwrap().feedback_wait.is_some());
            if release {
                let lease = core.lease.as_ref().unwrap();
                core.control(
                    ControlRequest {
                        action: "release".into(),
                        client_id: lease.client_id.clone(),
                        lease_id: Some(lease.secret.clone()),
                    },
                    now,
                )
                .unwrap();
            } else {
                calibration_request(&mut core, "stop", Operation::Stop, now);
            }
            let next = now + CONTROL_PERIOD;
            follow_reference(&mut core, next);
            core.advance(next, 0.01);
            let held = core.outgoing_reference().sample(next, None).unwrap();
            assert_eq!((held.q, held.dq), (moving.q, [0.0; 7]));
            assert!(core.motion.is_none() && core.calibration.is_none());
            assert_eq!(
                core.last_calibration_report.as_ref().unwrap().outcome,
                "cancelled"
            );
        }
    }
    #[test]
    fn calibration_retains_the_serial_failure_after_the_writer_exits() {
        let now = Instant::now();
        let (mut core, _directory, _sent) = calibration_fixture(now);
        calibration_request(
            &mut core,
            "serial-failure",
            Operation::CalibrateStart {
                settings: calibration_settings(),
            },
            now,
        );
        assert!(core.calibration.is_some());
        let message = "serial transaction deadline expired; wrote 0/20 command bytes";
        core.bus = Some(BusHandle::test_failed_bus(message));
        core.poll(now);
        assert_eq!(core.error.as_ref().unwrap().message, message);
        assert_eq!(
            core.calibration_result
                .as_ref()
                .unwrap()
                .report
                .as_ref()
                .unwrap()
                .message,
            message
        );
        core.poll(now + Duration::from_millis(10));
        assert_eq!(core.error.as_ref().unwrap().message, message);
    }
    #[test]
    fn calibration_start_accepts_stationary_positions_with_noisy_velocity_feedback() {
        let origin = Instant::now();
        let (mut core, _directory, sent) = calibration_fixture(origin);
        // Position remains at the hold pose while the device's velocity field
        // crosses the old 0.02 rad/s threshold, as in the recorded hold segment.
        for tick in 0..=60 {
            let now = origin + Duration::from_millis(tick * 10);
            for i in 0..7 {
                let mut value = core.feedback[i].as_ref().unwrap().value.clone();
                value.q = core.reference[i];
                value.dq = if i == 5 { 0.0306796166 } else { 0.0 };
                core.accept_feedback(value, now);
            }
            sent.send_replace(Some(SentReference {
                at: now,
                completed_at: now,
                generation: core.generation,
                epoch: core.reference_version,
                q: core.reference,
                dq: [0.0; 7],
                calibration_phase: None,
                fallback_hold: false,
            }));
        }
        calibration_request(
            &mut core,
            "stationary-noisy-dq",
            Operation::CalibrateStart {
                settings: calibration_settings(),
            },
            origin + Duration::from_millis(600),
        );
        assert!(core.calibration.is_some(), "{:?}", core.results.front());
        assert_eq!(core.results.front().unwrap().status, "accepted");
    }
    #[test]
    #[ignore = "requires R17A_CALIBRATION_TRACE pointing to a private local capture"]
    fn calibration_recorded_hold_starts_despite_velocity_noise() {
        let origin = Instant::now();
        let (mut core, _directory, sent) = calibration_fixture(origin);
        let path = std::env::var("R17A_CALIBRATION_TRACE").expect("set the local capture path");
        let capture: serde_json::Value =
            serde_json::from_slice(&std::fs::read(path).unwrap()).unwrap();
        let samples = capture["samples"].as_array().unwrap();
        let first_tx = samples.iter().find(|s| s["source"] == "transmit").unwrap();
        core.reference = std::array::from_fn(|i| first_tx["q_ref"][i].as_f64().unwrap());
        let mut reproduced = false;
        for sample in samples {
            let us = sample["at_us"].as_u64().unwrap();
            if us > 1_500_000 {
                break;
            }
            if sample["source"] != "feedback" {
                continue;
            }
            let joint = sample["joint"].as_u64().unwrap() as usize;
            let now = origin + Duration::from_micros(us);
            let mut value = core.feedback[joint].as_ref().unwrap().value.clone();
            value.q = sample["q"].as_f64().unwrap();
            value.dq = sample["dq"].as_f64().unwrap();
            value.mode = sample["mode"].as_u64().unwrap() as u8;
            value.error = sample["error"].as_u64().unwrap() as u32;
            value.timeout = sample["timeout"].as_bool().unwrap();
            core.accept_feedback(value, now);
            sent.send_replace(Some(SentReference {
                at: now,
                completed_at: now,
                generation: core.generation,
                epoch: core.reference_version,
                q: core.reference,
                dq: [0.0; 7],
                calibration_phase: None,
                fallback_hold: false,
            }));
            let old_velocity_gate = core
                .feedback
                .iter()
                .any(|o| o.as_ref().is_some_and(|o| o.value.dq.abs() > 0.02));
            if us >= 400_000 && core.motion_anchor(now).is_ok() && old_velocity_gate {
                calibration_request(
                    &mut core,
                    "recorded-hold",
                    Operation::CalibrateStart {
                        settings: calibration_settings(),
                    },
                    now,
                );
                assert!(core.calibration.is_some(), "{:?}", core.results.front());
                assert_eq!(core.results.front().unwrap().status, "accepted");
                println!("recorded hold accepted at {us} us");
                reproduced = true;
                break;
            }
        }
        assert!(
            reproduced,
            "capture did not contain a stationary window with a noisy velocity sample"
        );
        assert!(core.journal.shutdown(Duration::from_secs(2)));
    }
    #[test]
    fn calibration_start_records_minor_movement_instead_of_rejecting_it() {
        let origin = Instant::now();
        let (mut core, _directory, sent) = calibration_fixture(origin);
        for tick in 0..=60 {
            let now = origin + Duration::from_millis(tick * 10);
            for i in 0..7 {
                let mut value = core.feedback[i].as_ref().unwrap().value.clone();
                value.q = core.reference[i] + if i == 1 { tick as f64 * 0.0005 } else { 0.0 };
                value.dq = 0.0;
                core.accept_feedback(value, now);
            }
            sent.send_replace(Some(SentReference {
                at: now,
                completed_at: now,
                generation: core.generation,
                epoch: core.reference_version,
                q: core.reference,
                dq: [0.0; 7],
                calibration_phase: None,
                fallback_hold: false,
            }));
        }
        calibration_request(
            &mut core,
            "drifting-zero-dq",
            Operation::CalibrateStart {
                settings: calibration_settings(),
            },
            origin + Duration::from_millis(600),
        );
        assert!(core.calibration.is_some());
        assert_eq!(core.results.front().unwrap().status, "accepted");
    }
    #[test]
    fn calibration_internal_return_settles_with_noisy_velocity_feedback() {
        let origin = Instant::now();
        let (mut core, _directory, sent) = calibration_fixture(origin);
        core.begin_return(
            "internal-calibration-leg",
            core.reference,
            0.02,
            false,
            origin,
        )
        .unwrap();
        core.motion.as_mut().unwrap().calibration = true;
        for tick in 1..=120 {
            let now = origin + Duration::from_millis(tick * 10);
            for i in 0..7 {
                let mut value = core.feedback[i].as_ref().unwrap().value.clone();
                value.q = core.reference[i];
                value.dq = if i == 5 { 0.0306796166 } else { 0.0 };
                core.accept_feedback(value, now);
            }
            sent.send_replace(Some(SentReference {
                at: now,
                completed_at: now,
                generation: core.generation,
                epoch: core.reference_version,
                q: core.reference,
                dq: core.velocity,
                calibration_phase: None,
                fallback_hold: false,
            }));
            core.advance(now, 0.01);
            if core.motion.is_none() {
                break;
            }
        }
        assert!(
            core.motion.is_none(),
            "internal calibration leg never settles with stationary q"
        );
        assert!(core.enabled);
        assert!(core.error.is_none());
    }
    #[test]
    fn calibration_does_not_pair_receive_time_with_an_unfinished_transmit() {
        let origin = Instant::now();
        let make = |start, finish, value| SentReference {
            at: origin + Duration::from_millis(start),
            completed_at: origin + Duration::from_millis(finish),
            generation: 1,
            epoch: 1,
            q: [value; 7],
            dq: [0.0; 7],
            calibration_phase: None,
            fallback_hold: false,
        };
        let sent = VecDeque::from([make(90, 93, 1.0), make(100, 103, 2.0)]);
        assert!(calibration_reference(&sent, origin + Duration::from_millis(92)).is_none());
        assert_eq!(
            calibration_reference(&sent, origin + Duration::from_millis(101))
                .unwrap()
                .q,
            [1.0; 7]
        );
        assert_eq!(
            calibration_reference(&sent, origin + Duration::from_millis(103))
                .unwrap()
                .q,
            [2.0; 7]
        );
        assert_eq!(
            calibration_reference(&sent, origin + Duration::from_millis(154))
                .unwrap()
                .q,
            [2.0; 7]
        );
    }
    fn calibration_tick(
        core: &mut Core,
        sent: &watch::Sender<Option<SentReference>>,
        now: Instant,
        origin: Instant,
        jitter: bool,
    ) {
        // Simulation time must not race ahead while the real background worker
        // is fitting. Production ticks only try_recv; this wait is test-only.
        if let Some(run) = core.calibration.as_mut()
            && matches!(run.step, CalibrationStep::Identifying)
        {
            let fitted = run
                .identification
                .as_ref()
                .unwrap()
                .recv()
                .expect("background friction analysis");
            let (sender, receiver) = bounded(1);
            sender.send(fitted).unwrap();
            run.identification = Some(receiver);
        }
        if let Some(lease) = core.lease.as_mut() {
            lease.expires = now + Duration::from_millis(LEASE_MS);
        }
        // Construct a quasi-static torque-error response. The same analytic
        // friction generates feedback effort independently of the fitted model;
        // the test checks data flow and comparison, not real-arm performance.
        fn friction(v: f64, axis: usize) -> f64 {
            let fc = 0.05 + axis as f64 * 0.002;
            let fs = if v >= 0.0 { 0.13 } else { 0.16 };
            let vs = if v >= 0.0 { 0.025 } else { 0.035 };
            (fc + (fs - fc) * (-(v / vs).powi(2)).exp()) * (v / 0.02).tanh() + 0.2 * v
        }
        let source = core.outgoing_reference();
        let (wire_q, wire_dq, calibration_phase) =
            if let Sampling::Calibration { program, phase, .. } = &source.sampling {
                let previous = sent.borrow().map(|p| crate::reference::Previous {
                    at: p.at,
                    q: p.q,
                    dq: p.dq,
                });
                let wire = source.sample(now, previous).unwrap();
                let expected = phase.sample(program, now);
                assert_eq!((wire.q, wire.dq), (expected.q, expected.dq));
                (wire.q, wire.dq, Some(*phase))
            } else {
                (core.reference, core.velocity, None)
            };
        let compensation = core.servo_tuning.apply(core.velocity, [0.0; 7]).unwrap();
        let measured: [f64; 7] = std::array::from_fn(|i| {
            let residual = if jitter {
                friction(core.velocity[i], i) - compensation.friction_applied[i]
            } else {
                0.0
            };
            wire_q[i]
                - residual / SERVO_KP
                    * (1.0
                        + 0.3 * (now.saturating_duration_since(origin).as_secs_f64() * 19.0).sin())
        });
        let gravity = core.model.as_ref().unwrap().gravity(&measured).unwrap();
        for i in 0..7 {
            let old = core.feedback[i].as_ref().unwrap();
            let dt = now.saturating_duration_since(old.at).as_secs_f64();
            let mut value = old.value.clone();
            value.q = measured[i];
            value.dq = if dt > 0.0 {
                (value.q - old.value.q) / dt
            } else {
                core.velocity[i]
            };
            value.tau = gravity[i] + if jitter { friction(value.dq, i) } else { 0.0 };
            core.accept_feedback(value, now);
        }
        sent.send_replace(Some(SentReference {
            at: now,
            completed_at: now,
            generation: source.generation,
            epoch: source.epoch,
            q: wire_q,
            dq: wire_dq,
            calibration_phase,
            fallback_hold: false,
        }));
        let dt = now.saturating_duration_since(core.control_at).as_secs_f64();
        core.advance(now, if dt == 0.0 { 0.01 } else { dt });
    }
    #[test]
    fn calibration_completes_smooth_baseline_without_applying_compensation() {
        let origin = Instant::now();
        let (mut core, _dir, sent) = calibration_fixture(origin);
        let baseline = core.servo_tuning;
        calibration_request(
            &mut core,
            "smooth-calibration",
            Operation::CalibrateStart {
                settings: calibration_settings(),
            },
            origin,
        );
        assert!(core.calibration.is_some(), "{:?}", core.results.front());
        let center = core.reference;
        for tick in 1..30000 {
            let now = origin + Duration::from_millis(tick * 10);
            calibration_tick(&mut core, &sent, now, origin, false);
            assert!(core.enabled);
            assert!(core.error.is_none());
            assert!((core.reference[1] - center[1]).abs() <= 0.1);
            if core.calibration.is_none() {
                break;
            }
        }
        let report = core.snapshot(origin).calibration.unwrap().report.unwrap();
        assert_eq!(report.outcome, "unchanged");
        assert!(report.trials.iter().any(|trial| trial.phase == "baseline"));
        assert!(report.recommended.is_none());
        assert_eq!(core.servo_tuning, baseline);
        assert!(core.motion.is_none());
        assert!(core.at_rest());
    }
    #[test]
    fn stopping_while_fitting_discards_late_results_and_restores_the_original_tuning() {
        let origin = Instant::now();
        let (mut core, _dir, sent) = calibration_fixture(origin);
        let baseline = core.servo_tuning;
        calibration_request(
            &mut core,
            "analyse",
            Operation::CalibrateStart {
                settings: calibration_settings(),
            },
            origin,
        );
        let (sender, receiver) = bounded(1);
        let run = core.calibration.as_mut().unwrap();
        run.trial = run.queue.len();
        run.step = CalibrationStep::Identifying;
        run.identification = Some(receiver);
        // The pending receiver is deliberately held empty. Ticks continue and
        // stop uses the normal command path, without waiting for analysis.
        for tick in 1..5 {
            core.advance(origin + Duration::from_millis(tick * 10), 0.01);
            assert_eq!(core.servo_tuning, baseline);
            assert!(core.enabled && core.motion.is_none());
        }
        calibration_request(
            &mut core,
            "stop-fit",
            Operation::Stop,
            origin + Duration::from_millis(50),
        );
        calibration_tick(
            &mut core,
            &sent,
            origin + Duration::from_millis(60),
            origin,
            false,
        );
        assert!(core.calibration.is_none());
        assert_eq!(core.calibration_result.as_ref().unwrap().state, "cancelled");
        assert!(sender.try_send(Vec::new()).is_err());
        for tick in 7..15 {
            calibration_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                origin,
                false,
            );
            assert!(core.motion.is_none());
            assert_eq!(core.servo_tuning, baseline);
        }
    }
    #[test]
    fn calibration_default_program_sweeps_seven_axes_and_persists_actual_time_curves() {
        let origin = Instant::now();
        let (mut core, _dir, sent) = calibration_fixture(origin);
        let center = core.reference;
        let baseline = core.servo_tuning;
        calibration_request(
            &mut core,
            "automatic-seven-axes",
            serde_json::from_value(serde_json::json!({"kind": "calibrate_start"})).unwrap(),
            origin,
        );
        let ticks = joint_calibration_test_ticks(&core, &CalibrationSettings::default());
        for tick in 1..ticks {
            let now = origin + Duration::from_millis(tick * 10);
            calibration_tick(&mut core, &sent, now, origin, true);
            assert!(core.error.is_none());
            assert!(
                (0..7)
                    .all(|i| (core.reference[i] - center[i]).abs() <= 16.0_f64.to_radians() + 1e-6)
            );
            if core.calibration.is_none() {
                break;
            }
        }
        assert!(core.calibration.is_none());
        let report = core
            .calibration_result
            .as_ref()
            .unwrap()
            .report
            .as_ref()
            .unwrap();
        let measured_axes = report
            .trials
            .iter()
            .filter(|trial| trial.phase == "baseline")
            .map(|trial| trial.axis.unwrap())
            .collect::<Vec<_>>();
        assert_eq!(measured_axes, (0..7).collect::<Vec<_>>());
        assert!(report.sample_count > 1000);
        assert!(
            report.curves.is_empty(),
            "state snapshots carry metadata, not the full recording"
        );
        assert!(report.trials.iter().any(|trial| trial.phase == "after"));
        let id = report.id.clone();
        let count = report.sample_count;
        assert_eq!(core.servo_tuning, baseline);
        assert!((0..7).all(|i| (core.reference[i] - center[i]).abs() < 0.01));
        assert!(core.journal.shutdown(Duration::from_secs(5)));
        let stored = core.journal.calibration(&id).unwrap().unwrap();
        assert_eq!(stored.curves.len(), count);
        assert!(
            stored
                .curves
                .windows(2)
                .all(|pair| pair[1].t_ms > pair[0].t_ms)
        );
        assert!(stored.curves.iter().all(|point| point.command.is_some()));
        assert!(
            stored
                .curves
                .iter()
                .all(|point| { stored.trials[point.trial].axis == Some(point.axis) })
        );
        assert!(stored.curves.iter().any(|point| {
            (point.command.unwrap()[point.axis] - point.feedback[point.axis]).abs() > 0.0001
        }));
    }
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn completed_calibration_can_be_read_and_downloaded_while_sqlite_is_busy() {
        let origin = Instant::now();
        let (mut core, _dir, sent) = calibration_fixture(origin);
        let journal = core.journal.clone();
        let deadline = Instant::now() + Duration::from_secs(2);
        while !journal.stats().healthy {
            assert!(Instant::now() < deadline);
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        let db = rusqlite::Connection::open(_dir.path().join("events.sqlite3")).unwrap();
        db.execute_batch("BEGIN IMMEDIATE").unwrap();
        calibration_request(
            &mut core,
            "finish-before-commit",
            Operation::CalibrateStart {
                settings: calibration_settings(),
            },
            origin,
        );
        for tick in 1..90_000 {
            let now = origin + Duration::from_millis(tick * 10);
            calibration_tick(&mut core, &sent, now, origin, true);
            if core.calibration.is_none() {
                break;
            }
        }
        let result = core.calibration_result.as_ref().unwrap();
        assert_eq!(result.state, "completed");
        let id = result.id.clone();
        let count = result.report.as_ref().unwrap().sample_count;
        assert!(journal.calibration(&id).unwrap().is_none());
        // Keep the fake transmitter alive when the fixture starts its worker.
        let _transmit_sink = core.bus.as_ref().unwrap().test_transmit_sink();
        let arm = ArmHandle::spawn_core(core).unwrap();
        let server = crate::server::launch_at(
            arm.clone(),
            journal.clone(),
            "finished-report-test".into(),
            std::net::SocketAddr::from(([127, 0, 0, 1], 0)),
        )
        .await
        .unwrap();
        let client = reqwest::Client::new();
        let history: serde_json::Value = client
            .get(format!("{}/v1/calibrations", server.base_url()))
            .bearer_auth("finished-report-test")
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(history[0]["id"], id);
        let before_preview = arm.snapshot();
        let preview: crate::calibration_path::CalibrationPreview = client
            .post(format!("{}/v1/calibrations/preview", server.base_url()))
            .bearer_auth("finished-report-test")
            .json(&serde_json::json!({"amplitude":0.05}))
            .send()
            .await
            .unwrap()
            .error_for_status()
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(preview.program.amplitude, 0.05);
        assert!(!preview.points.is_empty());
        assert!(
            arm.snapshot().action.is_none(),
            "preview must not start a move"
        );
        assert_eq!(arm.snapshot().generation, before_preview.generation);
        for query in ["?points=3000", "?download=true"] {
            let response = client
                .get(format!("{}/v1/calibrations/{id}{query}", server.base_url()))
                .bearer_auth("finished-report-test")
                .send()
                .await
                .unwrap();
            assert_eq!(
                response.status(),
                reqwest::StatusCode::OK,
                "completed report must be readable before disk commit"
            );
            let report: CalibrationReport = response.json().await.unwrap();
            assert_eq!(report.id, id);
            assert_eq!(report.sample_count, count);
            assert!(!report.curves.is_empty());
            if query.contains("download") {
                assert_eq!(report.curves.len(), count);
            }
        }
        db.execute_batch("ROLLBACK").unwrap();
        server.shutdown().await;
        assert!(arm.shutdown(Duration::from_secs(2)));
        assert!(journal.shutdown(Duration::from_secs(3)));
        assert_eq!(
            journal.calibration(&id).unwrap().unwrap().curves.len(),
            count
        );
        let reopened = Journal::start(_dir.path().join("events.sqlite3")).unwrap();
        let fresh_arm = ArmHandle::spawn(reopened.clone()).unwrap();
        let fresh_server = crate::server::launch_at(
            fresh_arm.clone(),
            reopened.clone(),
            "reopened-test".into(),
            std::net::SocketAddr::from(([127, 0, 0, 1], 0)),
        )
        .await
        .unwrap();
        let persisted: CalibrationReport = client
            .get(format!(
                "{}/v1/calibrations/{id}?download=true",
                fresh_server.base_url()
            ))
            .bearer_auth("reopened-test")
            .send()
            .await
            .unwrap()
            .error_for_status()
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(persisted.curves.len(), count);
        fresh_server.shutdown().await;
        assert!(fresh_arm.shutdown(Duration::from_secs(1)));
        assert!(reopened.shutdown(Duration::from_secs(2)));
    }
    #[test]
    fn previewed_cartesian_calibration_executes_before_and_after_and_records_every_joint() {
        run_previewed_calibration(
            [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3],
            Pose {
                translation: [0.025, -0.015, 0.1],
                rotation: [
                    0.09879603932153468,
                    -0.148194058982302,
                    0.19759207864306935,
                    0.9639684818261637,
                ],
            },
        );
    }
    #[test]
    fn captured_pose_executes_its_full_automatic_calibration_range() {
        run_previewed_calibration(
            [
                1.0485035052359992,
                0.3369648851818667,
                -0.024104830439799833,
                0.5221767742481414,
                1.683944156955803,
                0.07362801043445026,
                0.34971860713891384,
            ],
            Pose {
                translation: [0.0; 3],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        );
    }

    #[test]
    fn november_23_capture_executes_full_xyz_before_and_after() {
        // Start of the 981-sample run that stopped on the first positive X leg.
        run_previewed_calibration(
            [
                1.0588989507340771,
                0.4330010897186444,
                0.02301950359586734,
                0.6914591879254376,
                1.5165053704519271,
                -0.06798913184098254,
                1.0028188649979972,
            ],
            Pose::identity(),
        );
    }

    #[test]
    fn approved_joint_speeds_respect_an_explicit_legacy_speed() {
        let now = Instant::now();
        let (mut core, _directory, _sent) = calibration_fixture(now);
        let joint_speeds = vec![0.01, 0.02];
        let program = crate::calibration_path::preview(
            core.model.as_ref().unwrap(),
            &core.side,
            core.reference,
            core.tcp.unwrap(),
            crate::calibration_path::PreviewRequest {
                amplitude: Some(0.01),
                axes: vec![0],
                joint_speeds: joint_speeds.clone(),
                ..Default::default()
            },
        )
        .unwrap()
        .program;
        let mut settings = CalibrationSettings {
            cartesian: Some(Arc::new(program)),
            speed: 0.2_f64.to_radians(),
            ..Default::default()
        };
        let error = core
            .begin_calibration("explicit-speed", settings.clone(), now)
            .unwrap_err();
        assert_eq!(error.code, "stale_preview");
        assert!(core.calibration.is_none() && core.motion.is_none());

        settings.speed = CalibrationSettings::default().speed;
        core.begin_calibration("use-approved-speeds", settings, now)
            .unwrap();
        assert_eq!(
            core.calibration.as_ref().unwrap().report.settings.speeds,
            joint_speeds
        );
    }

    #[test]
    fn calibration_distance_and_target_use_the_same_sent_anchor() {
        let origin = Instant::now();
        let (mut core, _directory, sent) = calibration_fixture(origin);
        let q = [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3];
        core.reference = q;
        let tcp = core.tcp.unwrap();
        let model = core.model.as_ref().unwrap().clone();
        let pose = model.fk(&q, &tcp).unwrap();
        let shifted =
            kinematics::offset_pose(&pose, [0.002, 0.0, 0.0, 0.0, 0.0, 0.0], Frame::Base).unwrap();
        let anchor = model
            .continue_pose(
                &q,
                &tcp,
                &shifted,
                0.2,
                model.continuation_bounds(&q).unwrap(),
            )
            .unwrap();
        let preview = crate::calibration_path::preview(
            &model,
            "right",
            q,
            tcp,
            crate::calibration_path::PreviewRequest {
                amplitude: Some(0.02),
                axes: vec![0],
                ..Default::default()
            },
        )
        .unwrap();
        for (i, position) in anchor.into_iter().enumerate() {
            let mut feedback = core.feedback[i].as_ref().unwrap().value.clone();
            feedback.q = position;
            core.accept_feedback(feedback, origin);
        }
        sent.send_replace(Some(SentReference {
            at: origin,
            completed_at: origin,
            generation: core.generation,
            epoch: core.reference_version,
            q: anchor,
            dq: [0.0; 7],
            calibration_phase: None,
            fallback_hold: false,
        }));
        calibration_request(
            &mut core,
            "one-anchor",
            Operation::CalibrateStart {
                settings: CalibrationSettings {
                    cartesian: Some(preview.program.into()),
                    max_friction: Some(0.0),
                    ..Default::default()
                },
            },
            origin,
        );
        assert!(core.calibration.is_some(), "{:?}", core.results.front());
        core.advance(origin + Duration::from_millis(10), 0.01);
        core.advance(origin + Duration::from_millis(20), 0.01);
        let motion = core.motion.as_ref().unwrap();
        assert_eq!(motion.initial, anchor);
        let end = motion.initial_pose.unwrap().translation[0] + motion.direction * motion.amount;
        assert!(
            (end - motion.target_pose.unwrap().translation[0]).abs() < 1e-12,
            "trajectory end {end}, target {:?}",
            motion.target_pose
        );
        let mut write = BusHandle::test_servo_writer();
        for tick in 3..5000 {
            let now = origin + Duration::from_millis(tick * 10);
            core.lease.as_mut().unwrap().expires = now + Duration::from_millis(LEASE_MS);
            sent.send_replace(Some(write(
                core.outgoing_reference(),
                core.desired_setpoints(),
                now,
            )));
            follow_reference(&mut core, now);
            core.advance_motion(now, 0.01);
            if core.motion.is_none() {
                break;
            }
        }
        assert!(core.motion.is_none(), "first Cartesian leg never finished");
        assert!(core.error.is_none(), "{:?}", core.error);
    }
    fn run_previewed_calibration(initial: [f64; 7], tcp: Pose) {
        let origin = Instant::now();
        let (mut core, _dir, sent) = calibration_fixture(origin);
        core.reference = initial;
        core.tcp = Some(tcp);
        calibration_tick(&mut core, &sent, origin, origin, false);
        let preview = crate::calibration_path::preview(
            core.model.as_ref().unwrap(),
            &core.side,
            core.reference,
            core.tcp.unwrap(),
            crate::calibration_path::PreviewRequest::default(),
        )
        .unwrap();
        let program = preview.program.clone();
        let center = core
            .model
            .as_ref()
            .unwrap()
            .fk(&program.origin, &program.tcp)
            .unwrap();
        let baseline = core.servo_tuning;
        let original_motion = core.motion_tuning;
        let (lower, upper) = core.model.as_ref().unwrap().position_limits();
        calibration_request(
            &mut core,
            "cartesian-program",
            serde_json::from_str(&serde_json::json!({"kind":"calibrate_start","settings":{"cartesian":program,"max_friction":0.0}}).to_string()).unwrap(),
            origin,
        );
        assert!(core.calibration.is_some(), "{:?}", core.results.front());
        let mut low = [[0.0_f64; 3]; 2];
        let mut high = low;
        // Allow both Cartesian passes and a second visit to each joint trial.
        // Include control ticks for starting legs, endpoint receipts and trial transitions.
        let ticks_per_pass: u64 = program
            .timed
            .iter()
            .chain(program.joint_legs.iter().flatten())
            .map(|leg| (leg.duration() / CONTROL_PERIOD.as_secs_f64()).ceil() as u64 + 4)
            .sum();
        for tick in 1..=2 * ticks_per_pass + 128 {
            let now = origin
                + Duration::from_micros(tick * 10_000 + [0, 2_100, 3_000, 900][tick as usize % 4]);
            calibration_tick(&mut core, &sent, now, origin, false);
            assert!(core.error.is_none(), "{:?}", core.error);
            for axis in 0..7 {
                assert!(
                    core.reference[axis] >= lower[axis].min(initial[axis]) - 1e-6
                        && core.reference[axis] <= upper[axis].max(initial[axis]) + 1e-6,
                    "axis {axis} q={} range {}..{} tick={tick}",
                    core.reference[axis],
                    lower[axis],
                    upper[axis]
                );
            }
            if let Some(run) = &core.calibration {
                assert!(!run.cancelled, "{}", run.report.message);
                if run.cartesian()
                    && core
                        .motion
                        .as_ref()
                        .is_some_and(|m| m.space == Space::Cartesian)
                {
                    let stage = usize::from(run.queue[run.trial].phase == "cartesian_after");
                    let pose = core
                        .model
                        .as_ref()
                        .unwrap()
                        .fk(&core.reference, &program.tcp)
                        .unwrap();
                    for i in 0..3 {
                        let d = pose.translation[i] - center.translation[i];
                        low[stage][i] = low[stage][i].min(d);
                        high[stage][i] = high[stage][i].max(d);
                    }
                    let e = kinematics::pose_error(&center, &pose);
                    // Full model travel includes derivative-limited turns near
                    // reach boundaries. Use the API's existing one-degree pose
                    // tolerance; small local-path tests check finer accuracy.
                    assert!(
                        e[3..].iter().map(|x| x * x).sum::<f64>().sqrt()
                            <= std::f64::consts::PI / 180.0,
                        "tick={tick} stage={} leg={} error={e:?} q={:?}",
                        run.queue[run.trial].phase,
                        run.leg,
                        core.reference
                    );
                }
            } else {
                break;
            }
        }
        assert!(
            core.calibration.is_none(),
            "programme did not finish: phase={:?}, calibration={:?}, motion={:?}, reference={:?}",
            core.phase,
            core.calibration
                .as_ref()
                .map(|r| (r.queue[r.trial].phase.as_str(), r.leg, r.trial)),
            core.motion
                .as_ref()
                .map(|m| (m.axis, m.duration, m.target_pose)),
            core.reference
        );
        let status = core.calibration_result.as_ref().unwrap();
        assert_eq!(
            status.state,
            "completed",
            "{}",
            status.report.as_ref().unwrap().message
        );
        for stage in 0..2 {
            for axis in 0..3 {
                let expected_low = program
                    .segments
                    .iter()
                    .filter(|s| s.axis == axis)
                    .map(|s| s.offset)
                    .fold(0.0_f64, f64::min);
                let expected_high = program
                    .segments
                    .iter()
                    .filter(|s| s.axis == axis)
                    .map(|s| s.offset)
                    .fold(0.0_f64, f64::max);
                assert!(
                    low[stage][axis] <= expected_low + 0.003,
                    "{stage} {axis}: {:?}",
                    low[stage]
                );
                assert!(
                    high[stage][axis] >= expected_high - 0.003,
                    "{stage} {axis}: {:?}",
                    high[stage]
                );
            }
        }
        let full = core.last_calibration_report.as_ref().unwrap();
        for phase in ["cartesian_before", "cartesian_after"] {
            let trial = full.trials.iter().find(|t| t.phase == phase).unwrap();
            assert_eq!(trial.space, "cartesian");
            let metrics = trial.metrics_by_joint.as_ref().unwrap();
            assert_eq!(metrics.len(), 7);
            assert!(metrics.iter().all(Option::is_some));
        }
        assert!(
            full.curves
                .iter()
                .filter(|p| p.space == "cartesian")
                .count()
                > 1000
        );
        assert!(
            full.curves
                .iter()
                .all(|p| full.trials.get(p.trial).is_some())
        );
        assert_eq!(core.servo_tuning, baseline);
        assert_eq!(core.motion_tuning, original_motion);
        assert!((0..7).all(|i| (core.reference[i] - program.origin[i]).abs() < 0.02));
        let json = serde_json::to_value(core.snapshot(Instant::now())).unwrap();
        assert!(json.get("calibration_report").is_none());
        assert!(json.get("reference").is_none());
        assert!(
            json["calibration"]["report"]["curves"]
                .as_array()
                .unwrap()
                .is_empty()
        );
        assert!(core.journal.shutdown(Duration::from_secs(5)));
    }
    #[test]
    fn calibration_repeats_candidate_then_restores_and_requires_explicit_apply() {
        let origin = Instant::now();
        let (mut core, _dir, sent) = calibration_fixture(origin);
        let baseline = core.servo_tuning;
        calibration_request(
            &mut core,
            "wobble-calibration",
            Operation::CalibrateStart {
                settings: calibration_settings(),
            },
            origin,
        );
        assert!(core.calibration.is_some(), "{:?}", core.results.front());
        let mut now = origin;
        for tick in 1..90000 {
            now = origin + Duration::from_millis(tick * 10);
            calibration_tick(&mut core, &sent, now, origin, true);
            for i in 0..7 {
                assert_eq!(core.servo_tuning.joints[i].kd, baseline.joints[i].kd);
                if i != 1 {
                    assert_eq!(core.servo_tuning.joints[i], baseline.joints[i]);
                }
            }
            if core.calibration.is_none() {
                break;
            }
        }
        let snapshot = core.snapshot(now);
        let report = snapshot.calibration.unwrap().report.unwrap();
        assert_eq!(
            report.outcome,
            "improved",
            "{} {:?}; fits {:?}",
            report.message,
            report
                .trials
                .iter()
                .map(|t| (&t.phase, t.metrics.as_ref().map(|m| m.velocity_rms)))
                .collect::<Vec<_>>(),
            report
                .identification
                .iter()
                .map(|f| (
                    f.axis,
                    f.samples,
                    f.positive.as_ref().map(|d| (&d.curve, d.samples, d.rmse)),
                    f.negative.as_ref().map(|d| (&d.curve, d.samples, d.rmse))
                ))
                .collect::<Vec<_>>()
        );
        assert_eq!(report.trials.last().unwrap().phase, "after");
        assert_eq!(report.choices.len(), 1);
        assert_eq!(core.servo_tuning, baseline);
        assert!(core.enabled);
        assert!(core.at_rest());
        now += Duration::from_millis(10);
        calibration_tick(&mut core, &sent, now, origin, false);
        calibration_request(
            &mut core,
            "use-calibration",
            Operation::CalibrateApply {
                calibration_id: report.id.clone(),
            },
            now,
        );
        assert_eq!(
            core.results.front().unwrap().status,
            "accepted",
            "{:?}",
            core.results.front()
        );
        assert_eq!(core.servo_tuning, report.recommended.unwrap());
        calibration_request(
            &mut core,
            "use-twice",
            Operation::CalibrateApply {
                calibration_id: report.id,
            },
            now,
        );
        assert_eq!(core.results.front().unwrap().code, "calibrate_apply");
        assert_eq!(core.servo_tuning, report.recommended.unwrap());
    }
    #[test]
    fn calibration_stop_release_and_lease_expiry_cancel_future_legs_and_restore_parameters() {
        for cancel in ["stop", "release", "expiry"] {
            let origin = Instant::now();
            let (mut core, _dir, sent) = calibration_fixture(origin);
            let baseline = core.servo_tuning;
            calibration_request(
                &mut core,
                "cancel-calibration",
                Operation::CalibrateStart {
                    settings: calibration_settings(),
                },
                origin,
            );
            for tick in 1..200 {
                calibration_tick(
                    &mut core,
                    &sent,
                    origin + Duration::from_millis(tick * 10),
                    origin,
                    true,
                );
            }
            assert!(core.calibration.is_some());
            let now = origin + Duration::from_secs(2);
            if cancel == "release" {
                let lease = core.lease.as_ref().unwrap();
                let request = ControlRequest {
                    action: "release".into(),
                    client_id: lease.client_id.clone(),
                    lease_id: Some(lease.secret.clone()),
                };
                core.control(request, now).unwrap();
            } else if cancel == "expiry" {
                core.lease.as_mut().unwrap().expires = now;
                core.expire_lease(now);
                assert!(core.lease.is_none());
            } else {
                calibration_request(&mut core, "stop-calibration", Operation::Stop, now);
            }
            for tick in 201..800 {
                calibration_tick(
                    &mut core,
                    &sent,
                    origin + Duration::from_millis(tick * 10),
                    origin,
                    true,
                );
            }
            assert!(core.calibration.is_none());
            assert!(core.motion.is_none());
            assert!(core.enabled);
            assert!(core.at_rest());
            assert_eq!(core.servo_tuning, baseline);
            assert_eq!(core.calibration_result.as_ref().unwrap().state, "cancelled");
            let report = core.last_calibration_report.as_ref().unwrap();
            assert!(report.sample_count > 0);
            assert!(
                report.trials[0].metrics.is_some(),
                "a stopped measurement must retain the observations already collected"
            );
        }
    }
    #[test]
    fn temporary_sender_hold_keeps_calibration_and_the_motion_context() {
        let origin = Instant::now();
        let (mut core, _dir, sent) = calibration_fixture(origin);
        calibration_request(
            &mut core,
            "hold-calibration",
            Operation::CalibrateStart {
                settings: calibration_settings(),
            },
            origin,
        );
        for tick in 1..60 {
            calibration_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                origin,
                false,
            );
        }
        let epoch = core.reference_version;
        let generation = core.generation;
        let action = core.motion.as_ref().unwrap().request_id.clone();
        for reason in [
            crate::reference::SampleStatus::Stale,
            crate::reference::SampleStatus::Future,
        ] {
            core.reject_send_reference(generation, epoch, core.reference, reason);
            assert!(!core.calibration.as_ref().unwrap().cancelled);
            assert_eq!(core.motion.as_ref().unwrap().request_id, action);
            assert_eq!(core.reference_version, epoch);
        }
        calibration_tick(
            &mut core,
            &sent,
            origin + Duration::from_millis(600),
            origin,
            false,
        );
        assert!(core.error.is_none());
        assert!(core.calibration.is_some());
    }
    #[test]
    fn accepted_preview_keeps_an_actual_anchor_just_outside_the_nominal_model_range() {
        let origin = Instant::now();
        let (mut core, _dir, sent) = calibration_fixture(origin);
        let mut preview_origin = core.reference;
        preview_origin[1] = 0.005;
        core.reference[1] = -0.004;
        calibration_tick(&mut core, &sent, origin, origin, false);
        let program = serde_json::from_value(serde_json::json!({
            "side":core.side,"origin":preview_origin,"tcp":core.tcp,
            "amplitude":0.02,"speed":0.01,
            "segments":[{"axis":2,"offset":0.02},{"axis":2,"offset":0}]
        }))
        .unwrap();
        calibration_request(
            &mut core,
            "near-limit-preview",
            Operation::CalibrateStart {
                settings: CalibrationSettings {
                    cartesian: Some(program),
                    ..CalibrationSettings::default()
                },
            },
            origin,
        );
        assert!(core.calibration.is_some(), "accepted preview must start");
        for tick in 1..100 {
            calibration_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                origin,
                false,
            );
            assert!(core.error.is_none(), "{:?}", core.error);
            assert!(
                core.calibration.as_ref().is_some_and(|run| !run.cancelled),
                "{:?}",
                core.calibration_result
                    .as_ref()
                    .map(|s| s.report.as_ref().map(|r| r.message.as_str()))
            );
        }
    }
    #[test]
    fn stopping_a_cartesian_measurement_retains_seven_axis_metrics_and_curves() {
        let origin = Instant::now();
        let (mut core, _dir, sent) = calibration_fixture(origin);
        let program = crate::calibration_path::preview(
            core.model.as_ref().unwrap(),
            &core.side,
            core.reference,
            core.tcp.unwrap(),
            crate::calibration_path::PreviewRequest {
                amplitude: Some(0.15),
                speed: 0.04,
                axes: vec![2],
                ..Default::default()
            },
        )
        .unwrap()
        .program
        .into();
        calibration_request(
            &mut core,
            "partial-cartesian",
            Operation::CalibrateStart {
                settings: CalibrationSettings {
                    cartesian: Some(program),
                    ..CalibrationSettings::default()
                },
            },
            origin,
        );
        for tick in 1..200 {
            calibration_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                origin,
                false,
            );
        }
        assert!(core.calibration.as_ref().is_some_and(|run| !run.cancelled));
        calibration_request(
            &mut core,
            "cancel",
            Operation::Stop,
            origin + Duration::from_secs(2),
        );
        for tick in 200..500 {
            calibration_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                origin,
                false,
            );
            if core.calibration.is_none() {
                break;
            }
        }
        let report = core.last_calibration_report.as_ref().unwrap();
        assert_eq!(report.outcome, "cancelled");
        assert!(report.curves.len() > 20);
        let metrics = report.trials[0].metrics_by_joint.as_ref().unwrap();
        assert_eq!(metrics.len(), 7);
        assert!(metrics.iter().all(Option::is_some));
    }
    #[test]
    fn motion_preferences_replace_the_old_jog_and_step_software_caps() {
        let now = Instant::now();
        for (space, axis, speed, amount) in [
            (Space::Joint, 0, 0.4, 0.5),
            (Space::Cartesian, 0, 0.04, 0.15),
            (Space::Cartesian, 5, 0.4, 0.5),
        ] {
            let (mut core, _dir) = observed_core(now);
            core.enabled = true;
            core.tcp = Some(Pose::identity());
            core.apply(
                "motion",
                Operation::ConfigureMotion {
                    config: MotionTuning {
                        joint_speed: 0.7,
                        acceleration: 0.8,
                        jerk: 8.0,
                    },
                },
                now,
                now,
            )
            .unwrap();
            core.begin(
                "larger-step",
                space,
                axis,
                1,
                speed,
                Frame::Base,
                true,
                amount,
                None,
                0,
                now,
            )
            .unwrap();
            assert_eq!(core.motion.as_ref().unwrap().speed, speed);
            assert_eq!(core.motion.as_ref().unwrap().amount, amount);
            assert_eq!(core.outgoing_reference().joint_velocity, [0.7; 7]);
        }
    }
    #[test]
    fn slow_jerk_joint_and_cartesian_steps_finish_after_the_former_wall_clock_deadline() {
        for (space, axis, amount, speed) in [
            (Space::Joint, 0, 0.5, 0.4),
            (Space::Cartesian, 0, 0.01, 0.04),
        ] {
            let origin = Instant::now();
            let (mut core, _dir) = observed_core(origin);
            core.reference = [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3];
            follow_reference(&mut core, origin);
            core.enabled = true;
            core.tcp = Some(Pose {
                translation: [0.0, 0.0, 0.1],
                ..Pose::identity()
            });
            core.motion_tuning = MotionTuning {
                joint_speed: 1.0,
                acceleration: 0.8,
                jerk: 0.01,
            };
            core.begin(
                "slow-jerk",
                space,
                axis,
                1,
                speed,
                Frame::Base,
                true,
                amount,
                None,
                0,
                origin,
            )
            .unwrap();
            let mut elapsed = 0.0;
            for tick in 1..10000 {
                elapsed = tick as f64 * 0.01;
                let now = origin + Duration::from_millis(tick * 10);
                follow_reference(&mut core, now);
                let previous_acceleration = core.acceleration;
                core.advance(now, 0.01);
                assert!(core.error.is_none(), "{space:?}: {:?}", core.error);
                for (i, a) in previous_acceleration.into_iter().enumerate() {
                    assert!((core.acceleration[i] - a).abs() <= 0.01 * 0.01 + 1e-8);
                }
                if core.motion.is_none() {
                    break;
                }
            }
            assert!(
                elapsed > 1.5 * amount / speed + 3.0,
                "fixture must exceed the former timeout"
            );
            assert!(
                core.motion.is_none() && core.at_rest(),
                "{space:?} did not complete"
            );
            assert_eq!(core.results.front().unwrap().code, "completed");
        }
    }
    #[test]
    fn calibration_joint_compensation_does_not_require_retesting_after_tcp_reference_changes() {
        let origin = Instant::now();
        let (mut core, _dir, sent) = calibration_fixture(origin);
        calibration_request(
            &mut core,
            "context-calibration",
            Operation::CalibrateStart {
                settings: calibration_settings(),
            },
            origin,
        );
        let mut now = origin;
        for tick in 1..90000 {
            now = origin + Duration::from_millis(tick * 10);
            calibration_tick(&mut core, &sent, now, origin, true);
            if core.calibration.is_none() {
                break;
            }
        }
        let report = core
            .calibration_result
            .as_ref()
            .unwrap()
            .report
            .as_ref()
            .unwrap()
            .clone();
        assert_eq!(report.outcome, "improved");
        let original = core.tcp.unwrap();
        let mut changed = original;
        changed.translation[0] += 0.01;
        calibration_request(
            &mut core,
            "tool-b",
            Operation::ConfigureTcp { tcp: changed },
            now,
        );
        calibration_request(
            &mut core,
            "tool-a",
            Operation::ConfigureTcp { tcp: original },
            now,
        );
        now += Duration::from_millis(10);
        calibration_tick(&mut core, &sent, now, origin, false);
        calibration_request(
            &mut core,
            "use-old-tool",
            Operation::CalibrateApply {
                calibration_id: report.id,
            },
            now,
        );
        assert_eq!(core.results.front().unwrap().code, "calibrate_apply");
    }
    #[test]
    fn calibration_boundary_ranges_speeds_and_axes_finish_without_leaving_limits() {
        for axis in [0, 6] {
            for excursion in [1.0_f64.to_radians(), 10.0_f64.to_radians()] {
                for speed in [0.005, 0.04] {
                    let origin = Instant::now();
                    let (mut core, _dir, sent) = calibration_fixture(origin);
                    let center = core.reference;
                    calibration_request(
                        &mut core,
                        "edge-calibration",
                        Operation::CalibrateStart {
                            settings: CalibrationSettings {
                                axis: Some(axis),
                                excursion,
                                speed,
                                max_friction: Some(0.1),
                                speeds: Vec::new(),
                                cartesian: None,
                            },
                        },
                        origin,
                    );
                    assert!(core.calibration.is_some(), "{:?}", core.results.front());
                    for tick in 1..180000 {
                        let at = origin + Duration::from_millis(tick * 10);
                        let previous_q = core.reference;
                        let previous_v = core.velocity;
                        let previous_a = core.acceleration;
                        calibration_tick(&mut core, &sent, at, origin, false);
                        assert!(core.error.is_none());
                        for i in 0..7 {
                            assert!(
                                (core.velocity[i] - previous_v[i]).abs()
                                    <= JOINT_ACCELERATION * 0.01 + 1e-9
                            );
                            assert!(
                                (core.acceleration[i] - previous_a[i]).abs()
                                    <= JOINT_JERK * 0.01 + 1e-8
                            );
                            assert!(
                                (core.reference[i] - previous_q[i]).abs()
                                    <= JOINT_SPEED * 0.01 + 1e-9
                            );
                        }
                        assert!(
                            (core.reference[axis] - center[axis]).abs() <= excursion + 1e-6,
                            "axis{axis} span{excursion} speed{speed} tick{tick} q={} delta={} v={} a={} status={:?}",
                            core.reference[axis],
                            core.reference[axis] - center[axis],
                            core.velocity[axis],
                            core.acceleration[axis],
                            core.calibration_result
                        );
                        if core.calibration.is_none() {
                            break;
                        }
                    }
                    assert!(core.calibration.is_none());
                    assert_eq!(core.calibration_result.as_ref().unwrap().state, "completed");
                    assert!(core.enabled);
                    assert!(core.journal.shutdown(Duration::from_secs(2)));
                }
            }
        }
    }
    #[test]
    fn calibration_rejects_mutations_and_fault_never_reenables_or_resumes() {
        let origin = Instant::now();
        let (mut core, _dir, sent) = calibration_fixture(origin);
        let baseline = core.servo_tuning;
        calibration_request(
            &mut core,
            "fault-calibration",
            Operation::CalibrateStart {
                settings: calibration_settings(),
            },
            origin,
        );
        calibration_request(
            &mut core,
            "other-motion",
            Operation::Step {
                space: Space::Joint,
                axis: 0,
                direction: 1,
                amount: 0.02,
                speed: 0.02,
                frame: Frame::Base,
            },
            origin,
        );
        assert_eq!(core.results.front().unwrap().code, "calibration_busy");
        calibration_request(
            &mut core,
            "other-parameters",
            Operation::ConfigureServo {
                config: Box::new(baseline),
            },
            origin,
        );
        assert_eq!(core.results.front().unwrap().code, "calibration_busy");
        core.fault("device_error", "simulated drive fault");
        for tick in 1..40 {
            calibration_tick(
                &mut core,
                &sent,
                origin + Duration::from_millis(tick * 10),
                origin,
                false,
            );
        }
        assert!(!core.enabled);
        assert!(core.calibration.is_none());
        assert!(core.motion.is_none());
        assert_eq!(core.servo_tuning, baseline);
        assert_eq!(core.calibration_result.as_ref().unwrap().state, "failed");
    }
    #[test]
    fn calibration_disable_and_disconnect_restore_without_resuming() {
        for disconnect in [false, true] {
            let origin = Instant::now();
            let (mut core, _dir, sent) = calibration_fixture(origin);
            let baseline = core.servo_tuning;
            calibration_request(
                &mut core,
                "calibration",
                Operation::CalibrateStart {
                    settings: calibration_settings(),
                },
                origin,
            );
            // Stop in a non-baseline trial, not just the first trajectory.
            let mut now = origin;
            for tick in 1..40000 {
                now = origin + Duration::from_millis(tick * 10);
                calibration_tick(&mut core, &sent, now, origin, true);
                if core.servo_tuning != baseline && core.motion.is_some() {
                    break;
                }
            }
            assert_ne!(core.servo_tuning, baseline);
            calibration_request(
                &mut core,
                "release-power",
                if disconnect {
                    Operation::Disconnect {
                        allow_release: true,
                    }
                } else {
                    Operation::Disable {
                        allow_release: true,
                    }
                },
                now,
            );
            assert!(core.calibration.is_none());
            assert!(core.motion.is_none());
            assert!(!core.enabled);
            assert_eq!(core.servo_tuning, baseline);
            assert_eq!(core.calibration_result.as_ref().unwrap().state, "cancelled");
            assert_eq!(core.bus.is_none(), disconnect);
            core.advance(now + Duration::from_secs(1), 0.01);
            assert!(!core.enabled);
            assert!(core.motion.is_none());
            assert!(core.journal.shutdown(Duration::from_secs(2)));
        }
    }
    #[test]
    fn calibration_report_retries_contention_and_hands_off_on_shutdown() {
        for close_immediately in [false, true] {
            let origin = Instant::now();
            let (mut core, directory, _sent) = calibration_fixture(origin);
            let journal = core.journal.clone();
            calibration_request(
                &mut core,
                "stored-calibration",
                Operation::CalibrateStart {
                    settings: calibration_settings(),
                },
                origin,
            );
            let mut run = core.calibration.take().unwrap();
            run.report.outcome = "cancelled".into();
            run.report.message = "已停止".into();
            let report_id = run.report.id.clone();
            journal.with_calibration_queue_locked(|| {
                core.finish_calibration(run);
                assert!(!core.pending_calibration_report.is_empty());
                core.flush_calibration_report();
                assert_eq!(core.pending_calibration_report.len(), 1);
            });
            if close_immediately {
                let arm = ArmHandle::spawn_core(core).unwrap();
                // Keep the report queue unavailable until the control thread has
                // stopped. The close coordinator must retain its handoff payload.
                let stopped = journal.with_calibration_queue_locked(|| {
                    let closer = arm.clone();
                    let (tx, rx) = bounded(1);
                    let worker = std::thread::spawn(move || {
                        tx.send(closer.shutdown(Duration::from_secs(2))).unwrap();
                    });
                    let deadline = Instant::now() + Duration::from_secs(1);
                    while arm.snapshot().phase != Phase::Closing && Instant::now() < deadline {
                        std::thread::sleep(Duration::from_millis(1));
                    }
                    assert_eq!(arm.snapshot().phase, Phase::Closing);
                    assert!(rx.try_recv().is_err(), "report admission still contended");
                    (worker, rx)
                });
                assert!(stopped.1.recv_timeout(Duration::from_secs(2)).unwrap());
                stopped.0.join().unwrap();
            } else {
                core.flush_calibration_report();
                assert!(core.pending_calibration_report.is_empty());
            }
            assert!(journal.shutdown(Duration::from_secs(2)));
            let reopened = Journal::start(directory.path().join("events.sqlite3")).unwrap();
            let report = reopened.calibration(&report_id).unwrap().unwrap();
            assert_eq!(report.request_id, "stored-calibration");
            assert_eq!(report.outcome, "cancelled");
            assert!(reopened.shutdown(Duration::from_secs(2)));
        }
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
    fn return_completes_at_target_despite_continuous_velocity_noise_and_small_position_jitter() {
        for disconnect_after in [false, true] {
            let mut now = Instant::now();
            let (mut core, _dir) = observed_core(now);
            core.enabled = true;
            core.port = Some("memory-only".into());
            let mut target = core.reference;
            target[3] += 0.05;
            core.begin_return("noisy-return", target, 0.05, disconnect_after, now)
                .unwrap();
            for tick in 0..1000 {
                now += Duration::from_millis(10);
                for i in 0..7 {
                    let mut feedback = core.feedback[i].as_ref().unwrap().value.clone();
                    feedback.q = core.reference[i]
                        + if i == 3 {
                            if tick % 2 == 0 { 0.0015 } else { -0.0015 }
                        } else {
                            0.0
                        };
                    feedback.dq = if i == 3 { 0.05 } else { 0.0 };
                    core.accept_feedback(feedback, now);
                }
                core.advance(now, 0.01);
                if core.motion.is_none() {
                    break;
                }
            }
            let result = core.results.front().unwrap();
            assert_eq!(result.status, "completed", "{result:?}");
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
    fn return_waits_for_arrival_without_an_estimated_time_deadline() {
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
        assert!(core.motion.is_some());
        assert!(core.enabled);
        assert!(core.port.is_some());
        assert!(core.results.iter().all(|r| r.code != "parked"));
        follow_reference(&mut core, now);
        core.advance(now + Duration::from_millis(1), 0.001);
        assert!(core.motion.is_none());
        assert_eq!(core.results.front().unwrap().code, "parked");
    }
    #[test]
    fn return_disconnect_waits_for_position_arrival() {
        let mut now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        core.port = Some("memory-only".into());
        core.begin_return("return", core.reference, 0.1, true, now)
            .unwrap();
        for _ in 0..50 {
            now += Duration::from_millis(10);
            follow_reference(&mut core, now);
            core.feedback[0].as_mut().unwrap().value.q += 0.03;
            core.advance(now, 0.01);
            assert!(core.motion.is_some());
            assert!(core.enabled);
        }
        for _ in 0..25 {
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
    fn return_disconnect_uses_the_latest_arrived_position_without_a_noise_window() {
        let mut now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        core.port = Some("memory-only".into());
        core.begin_return("return", core.reference, 0.1, true, now)
            .unwrap();
        let mut spike = core.feedback[0].as_ref().unwrap().value.clone();
        spike.q += 0.03;
        core.accept_feedback(spike.clone(), now + Duration::from_millis(2));
        spike.q -= 0.03;
        core.accept_feedback(spike, now + Duration::from_millis(3));
        now += Duration::from_millis(10);
        follow_reference(&mut core, now);
        core.advance(now, 0.01);
        assert!(!core.enabled);
        assert!(core.port.is_none());
        assert!(core.motion.is_none());
        assert_eq!(core.results.front().unwrap().code, "parked");
    }
    #[test]
    fn return_disconnect_does_not_use_expired_feedback() {
        let mut now = Instant::now();
        let (mut core, _directory) = observed_core(now);
        core.enabled = true;
        core.port = Some("memory-only".into());
        core.begin_return("return", core.reference, 0.1, true, now)
            .unwrap();
        now += Duration::from_millis(FEEDBACK_MS + 1);
        core.advance(now, 0.01);
        assert!(!core.enabled);
        assert!(core.port.is_some());
        assert_eq!(core.error.as_ref().unwrap().code, "feedback_stale");
        assert!(core.results.iter().all(|r| r.code != "parked"));
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
        for speed in [0.0, -0.1, f64::NAN] {
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
                        // Allow the configured acceleration and jerk to bring
                        // the new 1 rad/s preference up to constant speed.
                        if (160..180).contains(&tick) {
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
        run_cartesian_motion_checks(
            [1.5576, -0.0350, 0.0102, -0.0217, 1.6217, 0.0088, 0.1133],
            0.0005,
            0.005,
            6.0,
            [0.0, 0.0, 0.1],
            &[false, true],
        );
    }
    #[test]
    fn cartesian_jogs_above_the_former_caps_preserve_offset_tcp_through_stop() {
        run_cartesian_motion_checks(
            [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3],
            0.04,
            0.4,
            6.0,
            [0.0, 0.0, 0.1],
            &[false],
        );
    }
    #[test]
    fn gentle_jerk_stops_keep_the_offset_tcp_center_at_higher_jog_speeds() {
        let origin = Instant::now();
        let (mut core, _dir) = observed_core(origin);
        core.reference = [0.3, 0.7, 1.1, 0.9, -0.4, 0.2, -0.3];
        follow_reference(&mut core, origin);
        core.enabled = true;
        core.port = Some("offline-stop".into());
        core.motion_tuning.jerk = 0.6;
        core.tcp = Some(Pose {
            translation: [0.025, -0.015, 0.1],
            ..Pose::identity()
        });
        core.begin(
            "rotation",
            Space::Cartesian,
            4,
            1,
            0.7,
            Frame::Base,
            false,
            0.0,
            None,
            0,
            origin,
        )
        .unwrap();
        let path = Core::motion_path(core.motion.as_ref().unwrap()).unwrap();
        let mut stopped = false;
        for tick in 1..2000 {
            let now = origin + Duration::from_millis(tick * 10);
            follow_reference(&mut core, now);
            if tick == 200 {
                core.apply("stop", Operation::Stop, now, now).unwrap();
            }
            let q = core.reference;
            let v = core.velocity;
            let a = core.acceleration;
            core.advance(now, 0.01);
            assert_integrated_dynamics(&core, q, v, 0.01);
            for (i, previous) in a.into_iter().enumerate() {
                assert!((core.acceleration[i] - previous).abs() <= 0.6 * 0.01 + 1e-8);
            }
            if tick >= 200 && core.at_rest() && !core.stopping {
                stopped = true;
                break;
            }
        }
        assert!(stopped, "jerk-limited rotation must finish stopping");
        let pose = core
            .model
            .as_ref()
            .unwrap()
            .fk(&core.reference, core.tcp.as_ref().unwrap())
            .unwrap();
        path.check(&pose).unwrap();
        assert_eq!(core.results.front().unwrap().code, "stopped");
        assert!(core.enabled && core.port.is_some());
    }
    fn run_cartesian_motion_checks(
        q: [f64; 7],
        linear_speed: f64,
        angular_speed: f64,
        jerk: f64,
        tcp_offset: [f64; 3],
        steps: &[bool],
    ) {
        for frame in [Frame::Base, Frame::Tool] {
            for axis in 0..6 {
                for &step in steps {
                    let mut now = Instant::now();
                    let (mut core, _directory) = observed_core(now);
                    for (i, value) in q.into_iter().enumerate() {
                        let mut f = core.feedback[i].as_ref().unwrap().value.clone();
                        f.q = value;
                        core.accept_feedback(f, now);
                    }
                    core.reference = q;
                    core.enabled = true;
                    core.motion_tuning.jerk = jerk;
                    core.tcp = Some(Pose {
                        translation: tcp_offset,
                        rotation: [0.0, 0.0, 0.0, 1.0],
                    });
                    core.port = Some("memory-only".into());
                    let speed = if axis < 3 {
                        linear_speed
                    } else {
                        angular_speed
                    };
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
                                (core.acceleration[i] - previous_a).abs() <= jerk * dt + 1e-8,
                                "{frame:?} axis={axis} step={step} tick={tick}"
                            );
                        }
                        let pose = core
                            .model
                            .as_ref()
                            .unwrap()
                            .fk(&core.reference, core.tcp.as_ref().unwrap())
                            .unwrap();
                        path.check(&pose).unwrap_or_else(|error| {
                            panic!(
                                "{frame:?} axis={axis} tick={tick} stopping={}: {error}",
                                core.stopping
                            )
                        });
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
        for _ in 0..3000 {
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
        assert!(
            core.motion.is_none(),
            "remaining v={:?} a={:?} q={:?}",
            core.velocity,
            core.acceleration,
            core.reference
        );
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
    fn enable_first_servo_frame_uses_the_queued_seven_axis_pose() {
        let start = Instant::now();
        let (mut core, _dir, _) = calibration_fixture(start);
        let mut write = BusHandle::test_servo_writer();
        core.advance(start, 0.01);
        let a = write(core.outgoing_reference(), core.desired_setpoints(), start);
        calibration_request(
            &mut core,
            "disable",
            Operation::Disable {
                allow_release: true,
            },
            start,
        );
        let queue = core.bus.as_mut().unwrap().test_feedback_queue();
        let b = std::array::from_fn(|i| a.q[i] + 0.01 * (i + 1) as f64);
        let rx = start + Duration::from_millis(10);
        for (i, q) in b.into_iter().enumerate() {
            let mut value = core.feedback[i].as_ref().unwrap().value.clone();
            value.mode = 0;
            value.q = q;
            queue(value, rx);
        }
        let now = rx + Duration::from_millis(1);
        calibration_request(&mut core, "enable", Operation::Enable, now);
        core.advance(now, 0.011);
        assert!(core.enable_request.is_some() && !core.enabled);
        let first = write(core.outgoing_reference(), core.desired_setpoints(), now);
        assert_eq!((first.q, first.dq), (b, [0.0; 7]));
    }

    #[test]
    fn enable_rejects_a_queued_device_fault_followed_by_healthy_feedback() {
        for timeout in [false, true] {
            let start = Instant::now();
            let (mut core, _dir, _) = calibration_fixture(start);
            calibration_request(
                &mut core,
                "disable",
                Operation::Disable {
                    allow_release: true,
                },
                start,
            );
            let queue = core.bus.as_mut().unwrap().test_feedback_queue();
            let mut bad = core.feedback[5].as_ref().unwrap().value.clone();
            bad.error = if timeout { 0 } else { 1 };
            bad.timeout = timeout;
            queue(bad, start + Duration::from_millis(1));
            for axis in &core.feedback {
                queue(
                    axis.as_ref().unwrap().value.clone(),
                    start + Duration::from_millis(2),
                );
            }
            calibration_request(
                &mut core,
                "enable",
                Operation::Enable,
                start + Duration::from_millis(3),
            );
            assert!(core.enable_request.is_none() && !core.enabled);
            assert_eq!(core.results.front().unwrap().status, "rejected");
            assert_eq!(
                core.error.as_ref().unwrap().code,
                if timeout {
                    "device_timeout"
                } else {
                    "device_error"
                }
            );
            assert!(
                core.desired_setpoints()
                    .iter()
                    .all(|s| s.mode == DriveMode::Disabled)
            );
        }
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
        core.reference = q;
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
        assert!(held[0] > measured[0]);
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
        core.reference = q;
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

    #[test]
    #[ignore = "local hardware report required: set R17A_FEEDBACK_GAP_REPORT; no serial device is opened"]
    fn local_report_j6_feedback_gap_holds_without_advancing_or_fabricating_feedback() {
        use std::collections::BTreeSet;

        let path = std::env::var_os("R17A_FEEDBACK_GAP_REPORT")
            .expect("set R17A_FEEDBACK_GAP_REPORT to the original local calibration JSON");
        let file = std::fs::File::open(path).expect("open local report read-only");
        let report: CalibrationReport = serde_json::from_reader(std::io::BufReader::new(file))
            .expect("parse the original complete calibration report");
        assert_eq!(report.curves.len(), report.sample_count);
        assert!(report.captured_samples >= report.sample_count);
        let last = report.curves.iter().max_by_key(|p| p.t_ms).unwrap();
        let trial = &report.trials[last.trial];
        let axis = trial.axis.expect("the final trial is a joint baseline");
        assert_eq!((axis, trial.phase.as_str()), (5, "baseline"));
        assert_eq!(last.feedback_age_ms, [168; 7]);

        let mut rows: Vec<_> = report
            .curves
            .iter()
            .filter(|p| p.trial == last.trial)
            .collect();
        rows.sort_unstable_by_key(|p| p.t_ms);
        let moving_end = rows.iter().rposition(|p| p.velocity[axis] < 0.0).unwrap();
        let moving_start = rows[..=moving_end]
            .iter()
            .rposition(|p| p.velocity[axis] >= 0.0)
            .map_or(0, |index| index + 1);
        let rows = &rows[moving_start..];
        let first = rows[0];
        let origin = Instant::now();
        let instant = |ms: u64| origin + Duration::from_millis(ms);

        // Only q/dq/tau and inferred receive times below come from the report.
        // mode=1, error/warning=0, timeout=false, 24 V and 25 C are simulated:
        // this report contains none of those drive-state fields or hardware ACKs.
        // floor(t_ms)-floor(age_ms) locates a receive time to approximately1 ms;
        // duplicate reconstructed receive timestamps are not extra real frames.
        let feedback = |point: &Point, joint: usize| Feedback {
            id: joint as u8,
            q: point.feedback[joint],
            dq: point.feedback_velocity[joint],
            tau: point.effort.expect("recorded torque is required")[joint],
            temperature: 25.0,
            voltage: 24.0,
            error: 0,
            warning: 0,
            mode: 1,
            timeout: false,
        };
        let receive_ms = |point: &Point, joint: usize| {
            u64::from(point.t_ms) - u64::from(point.feedback_age_ms[joint])
        };
        let (mut core, _directory, sent) = calibration_fixture(instant(first.t_ms.into()));
        core.model = Some(Arc::new(Kinematics::load(&report.side).unwrap()));
        core.side = report.side.clone();
        core.tcp = report.tcp;
        core.servo_tuning = report.baseline;
        core.feedback = Default::default();
        for joint in 0..7 {
            core.accept_feedback(feedback(first, joint), instant(receive_ms(first, joint)));
        }
        core.reference = first
            .command
            .expect("first recorded successful send target");
        core.velocity = first.velocity;
        core.control_at = instant(first.t_ms.into());
        core.lease.as_mut().unwrap().expires = instant(u64::from(last.t_ms) + LEASE_MS);

        // Seed the production sender with the available actual target/velocity.
        // command_at_ms is completion time; packet start/duration was not recorded.
        let mut write = BusHandle::test_servo_writer();
        let seed_at = instant(first.command_at_ms.unwrap().into());
        let mut seed = Reference::joint(
            seed_at,
            core.reference,
            first.velocity,
            [0.0; 7], // Controller acceleration is absent from this report.
            core.generation,
            core.reference_version,
        );
        seed.sampling = Sampling::Absolute;
        seed.servo_tuning = report.baseline;
        let seeded = write(seed, core.desired_setpoints(), seed_at);
        assert_eq!(
            (seeded.q, seeded.dq),
            (first.command.unwrap(), first.velocity)
        );
        sent.send_replace(Some(seeded));

        let mut program = report.settings.cartesian.as_ref().unwrap().as_ref().clone();
        program.prepare(core.model.as_ref().unwrap()).unwrap();
        core.motion_tuning = program.motion;
        // This capture ends in the highest-speed negative J6 leg. Reconstruct
        // its approved joint program, not an unrecorded original start phase.
        let leg_index = program.joint_legs[axis].len() - 2;
        let leg = program.joint_legs[axis][leg_index].clone();
        let speed = program.joint_speeds[leg_index / 3];
        let mut settings = report.settings.clone();
        settings.axis = Some(axis);
        settings.cartesian = None;
        core.begin_calibration("local-feedback-gap", settings, instant(first.t_ms.into()))
            .unwrap();
        {
            let run = core.calibration.as_mut().unwrap();
            run.report.settings = report.settings.clone();
            run.report.settings.cartesian = Some(Arc::new(program));
            run.report.trials = report.trials.clone();
            run.queue = report
                .trials
                .iter()
                .map(|trial| CalibrationPlan {
                    phase: trial.phase.clone(),
                    axis: trial.axis.unwrap_or(0),
                    config: trial.config,
                    cartesian: trial.space == "cartesian",
                })
                .collect();
            run.trial = last.trial;
            run.legs = run.make_legs();
            run.leg = leg_index;
            run.step = CalibrationStep::Moving;
            run.started = origin;
            run.observed = None;
        }
        core.begin_calibration_leg(
            "local-recorded-j6-leg",
            leg.sample(0.0).q,
            speed,
            Some((Space::Joint, axis, leg)),
            instant(first.t_ms.into()),
        )
        .unwrap();

        let mut latest_ms: [u64; 7] = std::array::from_fn(|j| receive_ms(first, j));
        let mut arrivals = Vec::new();
        for point in rows.iter().skip(1) {
            for (joint, latest) in latest_ms.iter_mut().enumerate() {
                let at = receive_ms(point, joint);
                if at > *latest {
                    arrivals.push((at, feedback(point, joint)));
                    *latest = at;
                }
            }
        }
        arrivals.sort_by_key(|(ms, frame)| (*ms, frame.id));
        // The100 Hz control clock is an explicit simulation, not reconstructed
        // original control ticks. Sender calls use only retained completion times.
        let controls: BTreeSet<_> = (u64::from(first.t_ms)..=u64::from(last.t_ms))
            .step_by(CONTROL_PERIOD.as_millis() as usize)
            .chain([u64::from(last.t_ms)])
            .collect();
        let sends: BTreeSet<_> = rows
            .iter()
            .filter_map(|p| p.command_at_ms.map(u64::from))
            .filter(|ms| *ms >= u64::from(first.t_ms))
            .collect();
        let ticks: BTreeSet<_> = controls.union(&sends).copied().collect();
        let mut next_feedback = 0;
        let mut delivered_ms: [u64; 7] = std::array::from_fn(|j| receive_ms(first, j));
        let mut previous_control = instant(first.t_ms.into()) - CONTROL_PERIOD;
        let mut source = core.outgoing_reference();
        let mut setpoints = core.desired_setpoints();
        let mut held: Option<(SentReference, f64)> = None;
        let mut held_reference = None;
        let mut held_capture_count = None;
        for ms in ticks {
            let at = instant(ms);
            while next_feedback < arrivals.len() && arrivals[next_feedback].0 <= ms {
                let (received, frame) = &arrivals[next_feedback];
                core.accept_feedback(frame.clone(), instant(*received));
                delivered_ms[frame.id as usize] = *received;
                next_feedback += 1;
            }
            if controls.contains(&ms) {
                core.advance(at, at.duration_since(previous_control).as_secs_f64());
                previous_control = at;
                assert!(core.error.is_none() && core.enabled, "{:?}", core.error);
                let run = core
                    .calibration
                    .as_ref()
                    .expect("the recorded leg remains active");
                assert_eq!((run.trial, run.leg), (last.trial, leg_index));
                assert_eq!(
                    core.motion.as_ref().unwrap().request_id,
                    "local-recorded-j6-leg"
                );
                if delivered_ms
                    .iter()
                    .any(|rx| ms - rx > MAX_REFERENCE_AGE.as_millis() as u64)
                {
                    assert_eq!(
                        core.velocity, [0.0; 7],
                        "old feedback advanced the planned leg"
                    );
                    assert_eq!(core.acceleration, [0.0; 7]);
                    assert_eq!(
                        core.reference,
                        *held_reference.get_or_insert(core.reference),
                        "the held reference jumped without new feedback"
                    );
                    let count = *held_capture_count.get_or_insert(run.points.captured());
                    assert_eq!(
                        run.points.captured(),
                        count,
                        "held values became new measurements"
                    );
                }
                source = core.outgoing_reference();
                setpoints = core.desired_setpoints();
            }
            if sends.contains(&ms) {
                let receipt = write(source.clone(), setpoints, at);
                if delivered_ms
                    .iter()
                    .any(|rx| ms - rx > MAX_REFERENCE_AGE.as_millis() as u64)
                {
                    assert!(
                        receipt.fallback_hold,
                        "a fresh control timestamp extended old feedback"
                    );
                    assert_eq!(receipt.dq, [0.0; 7]);
                    let progress = receipt.calibration_phase.unwrap().progress(at);
                    let (first_hold, first_progress) = *held.get_or_insert((receipt, progress));
                    assert_eq!((receipt.q, progress), (first_hold.q, first_progress));
                }
                sent.send_replace(Some(receipt));
            }
        }
        assert_eq!(next_feedback, arrivals.len());
        let (held, _) = held.expect("the recorded168 ms feedback gap must enter sender hold");
        assert_eq!((core.reference, core.velocity), (held.q, [0.0; 7]));
        for joint in 0..7 {
            let observed = core.feedback[joint].as_ref().unwrap();
            assert_eq!(observed.at, instant(receive_ms(last, joint)));
            assert_eq!(
                (observed.value.q, observed.value.dq, observed.value.tau),
                (
                    last.feedback[joint],
                    last.feedback_velocity[joint],
                    last.effort.unwrap()[joint]
                )
            );
            assert_eq!(
                instant(last.t_ms.into())
                    .duration_since(observed.at)
                    .as_millis(),
                168
            );
        }

        let run = core.calibration.as_ref().unwrap();
        assert_eq!(run.points.captured(), held_capture_count.unwrap());
        assert_eq!((run.trial, run.leg), (last.trial, leg_index));
        assert!(core.error.is_none() && core.enabled);
        eprintln!(
            "local report: {} retained/{} captured; {} reconstructed per-axis arrivals; held through{} ms age; drive mode/error/voltage and control clock are simulated; no feedback after EOF",
            report.sample_count,
            report.captured_samples,
            arrivals.len(),
            last.feedback_age_ms[axis]
        );
    }
}
