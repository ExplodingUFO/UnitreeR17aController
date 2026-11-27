import { invoke } from "@tauri-apps/api/core";
import motorFaults from "../resources/motor-faults.json" with { type: "json" };
export const FEEDBACK_MAX_AGE_MS = 200;
export type Side = "right" | "left";
export type Space = "joint" | "cartesian";
export type Frame = "base" | "tool";
export type Phase =
  | "disconnected"
  | "connecting"
  | "disabled"
  | "enabling"
  | "ready"
  | "teaching"
  | "moving"
  | "stopping"
  | "fault"
  | "closing";
export interface Joint {
  id: number;
  name: string;
  q: number | null;
  dq: number | null;
  tau: number | null;
  age_ms: number | null;
  temperature: number | null;
  winding_temperature: number | null;
  voltage: number | null;
  error: number | null;
  error_reason: string | null;
}
export interface Pose {
  translation: [number, number, number];
  rotation: [number, number, number, number];
}
export interface FrictionCurve {
  coulomb: number;
  static_friction: number;
  viscous: number;
  stribeck_velocity: number;
}
export interface StribeckTuning {
  enabled: boolean;
  positive: FrictionCurve;
  negative: FrictionCurve;
}
export interface JointTuning {
  kd: number;
  velocity_ff: number;
  coulomb: number;
  viscous: number;
  velocity_scale: number;
  max_friction: number;
  stribeck?: StribeckTuning;
}
export interface ServoTuning {
  friction_enabled: boolean;
  joints: [
    JointTuning,
    JointTuning,
    JointTuning,
    JointTuning,
    JointTuning,
    JointTuning,
    JointTuning,
  ];
}
export interface MotionTuning {
  joint_speed: number;
  acceleration: number;
  jerk: number;
}
export const DEFAULT_MOTION_TUNING: MotionTuning = {
  joint_speed: 1,
  acceleration: 0.6,
  jerk: 6,
};
export interface CalibrationSettings {
  axis: number | null;
  excursion: number;
  speed: number;
  speeds?: number[];
  max_friction: number | null;
  cartesian?: CartesianProgram | null;
}
export interface CalibrationPreviewRequest {
  amplitude?: number | null;
  speed: number;
  joint_speeds: number[];
  axes: number[];
  motion: MotionTuning;
}
export interface CartesianProgram {
  preview_id?: string;
  joint_targets?: [number, number][];
  joint_speeds?: number[];
  side: Side;
  origin: JointPositions;
  tcp: Pose;
  amplitude: number;
  speed: number;
  joint_excursion: number;
  segments: { axis: number; offset: number }[];
  guide?: { axis: number; offset: number; joints: JointPositions }[];
  motion?: MotionTuning;
  auto_range?: boolean;
}
export interface PreviewPoint {
  t_s?: number;
  joints: JointPositions;
  tcp: [number, number, number];
  segment: number;
  space: string;
  axis: number;
}
export interface CalibrationPreview {
  cartesian_duration_s?: number;
  program: CartesianProgram;
  points: PreviewPoint[];
  ranges?: { axis: number; negative: number; positive: number }[];
}
export interface TeachingPoint {
  t_ms: number;
  q: JointPositions;
}
export interface TeachingSummary {
  id: string;
  name: string;
  created_at_ms: number;
  side: Side;
  tcp: Pose;
  duration_ms: number;
  sample_count: number;
  outcome: string;
}
export interface TeachingTrajectory extends TeachingSummary {
  points: TeachingPoint[];
}
export interface TeachingPlaybackOptions {
  speed_scale: number;
  smoothing_s: number;
}
export interface TeachingPreviewPoint {
  t_ms: number;
  joints: JointPositions;
  tcp: [number, number, number];
  phase: "approach" | "path";
}
export interface TeachingPreview {
  trajectory_id: string;
  side: Side;
  tcp: Pose;
  origin: JointPositions;
  duration_ms: number;
  approach_duration_ms: number;
  requested_speed_scale: number;
  effective_speed_scale: number;
  points: TeachingPreviewPoint[];
}
export interface TeachingMode {
  request_id: string;
  state: "entering" | "dragging" | "leaving" | "holding";
}
export interface TeachingStatus {
  id: string;
  request_id: string;
  state: "recording" | "finishing" | "replaying" | "approaching";
  recording: boolean;
  sample_count: number;
  duration_ms: number;
  progress: number;
  trajectory_id: string | null;
}
export interface CalibrationMetrics {
  windows: number;
  velocity_rms: number;
  tracking_rms: number;
  tracking_peak: number;
  other_peak: number;
  valid_ms: number;
}
export interface CalibrationTrial {
  index: number;
  phase: string;
  axis?: number | null;
  config: ServoTuning;
  metrics: CalibrationMetrics | null;
  space?: Space;
  metrics_by_joint?: (CalibrationMetrics | null)[] | null;
}
export interface CalibrationPoint {
  t_ms: number;
  trial: number;
  axis: number;
  space?: Space;
  reference: JointPositions;
  command: JointPositions | null;
  feedback: JointPositions;
  velocity: JointPositions;
  feedback_velocity: JointPositions;
  feedback_age_ms: JointPositions;
  command_at_ms: number | null;
  effort?: JointPositions | null;
  feedforward_effort?: JointPositions | null;
  driver_temperature?: JointPositions | null;
  winding_temperature?: JointPositions | null;
}
export interface CalibrationFault {
  axis: number;
  code: number;
  message: string;
  t_ms: number;
  driver_temperature: number;
  winding_temperature: number;
  position: number;
  command: number | null;
  effort: number;
  voltage: number;
  feedforward_effort?: number | null;
}
export interface FitPoint {
  velocity: number;
  effort: number;
  predicted: number;
  samples: number;
}
export interface DirectionFit {
  curve: FrictionCurve;
  samples: number;
  velocity_min: number;
  velocity_max: number;
  rmse: number;
  points: FitPoint[];
}
export interface Identification {
  axis: number;
  source: string;
  samples: number;
  positive: DirectionFit | null;
  negative: DirectionFit | null;
  inertia: number;
  max_friction?: number;
}
export interface CalibrationChoice {
  axis: number;
  baseline_index: number;
  candidate_index: number;
  after_index: number | null;
  velocity_change: number;
  tracking_change: number;
}
export interface CalibrationReport {
  id: string;
  request_id: string;
  created_at_ms: number;
  side: Side;
  port: string | null;
  settings: CalibrationSettings;
  center_reference: JointPositions;
  center_measured: JointPositions;
  tcp: Pose | null;
  baseline: ServoTuning;
  recommended: ServoTuning | null;
  trials: CalibrationTrial[];
  sample_count: number;
  captured_samples?: number;
  curves: CalibrationPoint[];
  fault?: CalibrationFault | null;
  choices: CalibrationChoice[];
  identification?: Identification[];
  outcome: "improved" | "unchanged" | "insufficient" | "cancelled" | "failed";
  improvement: number | null;
  message: string;
}
export interface CalibrationStatus {
  id: string;
  request_id: string;
  state: "running" | "stopping" | "completed" | "cancelled" | "failed";
  stage:
    | "baseline"
    | "identifying"
    | "candidate"
    | "baseline_check"
    | "validation"
    | "after"
    | "cartesian_before"
    | "cartesian_after"
    | "restoring"
    | "done";
  axis: number;
  space?: Space;
  trial_index: number;
  trial_count: number;
  leg_index: number;
  leg_count: number;
  progress: number;
  live?: CalibrationPoint | null;
  report: CalibrationReport | null;
}
const SERVO_TUNING_FIELDS = [
  "kd",
  "velocity_ff",
  "coulomb",
  "viscous",
  "velocity_scale",
  "max_friction",
] as const;
const FRICTION_CURVE_FIELDS = [
  "coulomb",
  "static_friction",
  "viscous",
  "stribeck_velocity",
] as const;
export function createStribeckTuning(): StribeckTuning {
  const curve = (): FrictionCurve => ({
    coulomb: 0,
    static_friction: 0,
    viscous: 0,
    stribeck_velocity: 0.02,
  });
  return { enabled: false, positive: curve(), negative: curve() };
}
export function isFrictionCurve(value: unknown): value is FrictionCurve {
  if (!value || typeof value !== "object") return false;
  const curve = value as FrictionCurve;
  return FRICTION_CURVE_FIELDS.every((key) => {
    const number = curve[key];
    return (
      typeof number === "number" &&
      Number.isFinite(number) &&
      (key === "stribeck_velocity" ? number > 0 : number >= 0)
    );
  });
}
export function sameStribeckTuning(
  a: StribeckTuning | undefined,
  b: StribeckTuning | undefined,
): boolean {
  const left = a ?? createStribeckTuning();
  const right = b ?? createStribeckTuning();
  return (
    left.enabled === right.enabled &&
    (["positive", "negative"] as const).every((direction) =>
      FRICTION_CURVE_FIELDS.every(
        (key) => left[direction][key] === right[direction][key],
      ),
    )
  );
}
export function createServoTuning(): ServoTuning {
  return {
    friction_enabled: false,
    joints: Array.from({ length: 7 }, () => ({
      kd: 10,
      velocity_ff: 1,
      coulomb: 0,
      viscous: 0,
      velocity_scale: 0.02,
      max_friction: 0.5,
      stribeck: createStribeckTuning(),
    })) as ServoTuning["joints"],
  };
}
export function isServoTuning(value: unknown): value is ServoTuning {
  if (!value || typeof value !== "object") return false;
  const config = value as Partial<ServoTuning>;
  return (
    typeof config.friction_enabled === "boolean" &&
    Array.isArray(config.joints) &&
    config.joints.length === 7 &&
    Array.from(config.joints).every(
      (joint) =>
        joint &&
        typeof joint === "object" &&
        SERVO_TUNING_FIELDS.every((key) => {
          const number = joint[key];
          return (
            typeof number === "number" &&
            Number.isFinite(number) &&
            (key === "velocity_scale" ? number > 0 : number >= 0)
          );
        }) &&
        (joint.stribeck === undefined ||
          (joint.stribeck &&
            typeof joint.stribeck.enabled === "boolean" &&
            isFrictionCurve(joint.stribeck.positive) &&
            isFrictionCurve(joint.stribeck.negative))),
    )
  );
}
export function sameServoTuning(a: ServoTuning, b: ServoTuning): boolean {
  return (
    a.friction_enabled === b.friction_enabled &&
    a.joints.length === 7 &&
    b.joints.length === 7 &&
    a.joints.every(
      (joint, index) =>
        SERVO_TUNING_FIELDS.every(
          (key) => joint[key] === b.joints[index][key],
        ) && sameStribeckTuning(joint.stribeck, b.joints[index].stribeck),
    )
  );
}
export type ResultStatus =
  | "accepted"
  | "rejected"
  | "completed"
  | "cancelled"
  | "failed";
export interface Result {
  request_id: string;
  status: ResultStatus;
  code: string;
  message: string;
  measured_tcp?: Pose;
  actual_duration_s?: number;
}
export type JointPositions = [
  number,
  number,
  number,
  number,
  number,
  number,
  number,
];
export const isJointPositions = (value: unknown): value is JointPositions =>
  Array.isArray(value) &&
  value.length === 7 &&
  Array.from(value).every((q) => typeof q === "number" && Number.isFinite(q));

// Match the canonical J1–J7/nonzero u32 message emitted by the old writer.
function legacyFaultMessage(message: string) {
  const match = /^(J[1-7]) 故障码 ([1-9][0-9]{0,9})$/.exec(message);
  if (!match) return message;
  const code = Number(match[2]);
  if (code > 0xffffffff) return message;
  const hex = (bits: number) =>
    "0x" + bits.toString(16).toUpperCase().padStart(8, "0");
  const parts: string[] = [];
  let unknown = code;
  for (const [key, label] of Object.entries(motorFaults)) {
    const mask = Number(key);
    if ((code & mask) === 0) continue;
    parts.push(label);
    unknown = (unknown & ~mask) >>> 0;
  }
  if (unknown) parts.push(`未知故障位 ${hex(unknown)}`);
  const description =
    parts.join("、") + (parts.length > 1 ? `（${hex(code)}）` : "");
  return `${match[1]} ${description}（故障码 ${code}）`;
}

export function parseCalibrationReport(value: unknown): CalibrationReport {
  if (!value || typeof value !== "object") throw new Error("报告格式无效");
  const report = value as CalibrationReport;
  const axis = report.settings?.axis ?? null;
  const validAxis = (n: unknown) =>
    typeof n === "number" && Number.isInteger(n) && n >= 0 && n < 7;
  const validCount = (n: number) => Number.isInteger(n) && n >= 0;
  const validDirection = (fit: DirectionFit | null) =>
    fit === null ||
    (fit &&
      isFrictionCurve(fit.curve) &&
      validCount(fit.samples) &&
      Number.isFinite(fit.velocity_min) &&
      Number.isFinite(fit.velocity_max) &&
      Number.isFinite(fit.rmse) &&
      fit.rmse >= 0 &&
      Array.isArray(fit.points) &&
      fit.points.every(
        (point) =>
          point &&
          Number.isFinite(point.velocity) &&
          Number.isFinite(point.effort) &&
          Number.isFinite(point.predicted) &&
          validCount(point.samples),
      ));
  const validMetrics = (metrics: CalibrationMetrics | null) =>
    metrics === null ||
    (metrics &&
      [
        "windows",
        "velocity_rms",
        "tracking_rms",
        "tracking_peak",
        "other_peak",
        "valid_ms",
      ].every((key) =>
        Number.isFinite(metrics[key as keyof CalibrationMetrics]),
      ));
  if (
    typeof report.id !== "string" ||
    !report.settings ||
    (axis !== null && !validAxis(axis)) ||
    (report.settings.max_friction != null &&
      (!Number.isFinite(report.settings.max_friction) ||
        report.settings.max_friction < 0)) ||
    (report.settings.speeds !== undefined &&
      (!Array.isArray(report.settings.speeds) ||
        !report.settings.speeds.every(
          (speed) => Number.isFinite(speed) && speed > 0,
        ))) ||
    !isServoTuning(report.baseline) ||
    (report.recommended != null && !isServoTuning(report.recommended)) ||
    !isJointPositions(report.center_reference) ||
    !isJointPositions(report.center_measured) ||
    !Array.isArray(report.trials) ||
    !report.trials.every(
      (trial) =>
        trial &&
        Number.isInteger(trial.index) &&
        typeof trial.phase === "string" &&
        (trial.axis == null || validAxis(trial.axis)) &&
        isServoTuning(trial.config) &&
        validMetrics(trial.metrics) &&
        (trial.metrics_by_joint == null ||
          (Array.isArray(trial.metrics_by_joint) &&
            trial.metrics_by_joint.length === 7 &&
            trial.metrics_by_joint.every(validMetrics))),
    ) ||
    !["improved", "unchanged", "insufficient", "cancelled", "failed"].includes(
      report.outcome,
    )
  )
    throw new Error("请选择有效的校准 JSON 报告");
  const curves = report.curves ?? [];
  const choices = report.choices ?? [];
  const fault = report.fault;
  if (
    fault != null &&
    (!validAxis(fault.axis) ||
      !validCount(fault.code) ||
      typeof fault.message !== "string" ||
      !Number.isFinite(fault.t_ms) ||
      fault.t_ms < 0 ||
      ![
        fault.position,
        fault.effort,
        fault.voltage,
        fault.driver_temperature,
        fault.winding_temperature,
      ].every(Number.isFinite) ||
      (fault.command !== null && !Number.isFinite(fault.command)) ||
      (fault.feedforward_effort != null &&
        !Number.isFinite(fault.feedforward_effort)))
  )
    throw new Error("报告中的故障记录无效");
  if (
    !Array.isArray(curves) ||
    !curves.every(
      (point) =>
        point &&
        Number.isFinite(point.t_ms) &&
        point.t_ms >= 0 &&
        Number.isInteger(point.trial) &&
        validAxis(point.axis) &&
        [
          point.reference,
          point.feedback,
          point.velocity,
          point.feedback_velocity,
          point.feedback_age_ms,
        ].every(isJointPositions) &&
        (point.command === null || isJointPositions(point.command)) &&
        (point.effort == null || isJointPositions(point.effort)) &&
        (point.feedforward_effort == null ||
          isJointPositions(point.feedforward_effort)) &&
        (point.driver_temperature == null ||
          isJointPositions(point.driver_temperature)) &&
        (point.winding_temperature == null ||
          isJointPositions(point.winding_temperature)) &&
        (point.command_at_ms === null || Number.isFinite(point.command_at_ms)),
    ) ||
    !Array.isArray(choices) ||
    !choices.every(
      (choice) =>
        choice &&
        validAxis(choice.axis) &&
        [choice.baseline_index, choice.candidate_index].every(
          Number.isInteger,
        ) &&
        (choice.after_index == null || Number.isInteger(choice.after_index)) &&
        Number.isFinite(choice.velocity_change) &&
        Number.isFinite(choice.tracking_change),
    )
  )
    throw new Error("报告中的曲线或比较数据无效");
  if (
    report.identification !== undefined &&
    (!Array.isArray(report.identification) ||
      !report.identification.every(
        (fit) =>
          fit &&
          validAxis(fit.axis) &&
          typeof fit.source === "string" &&
          validCount(fit.samples) &&
          Number.isFinite(fit.inertia) &&
          (fit.max_friction === undefined ||
            (Number.isFinite(fit.max_friction) && fit.max_friction >= 0)) &&
          validDirection(fit.positive) &&
          validDirection(fit.negative),
      ))
  )
    throw new Error("报告中的摩擦拟合数据无效");
  return {
    ...report,
    message: legacyFaultMessage(report.message),
    settings: { ...report.settings, axis },
    curves,
    choices,
    sample_count: report.sample_count ?? curves.length,
  };
}
export type Action = {
  request_id: string;
  progress: number;
} & (
  | { space: Space; mode: "continuous" | "step"; axis: number }
  | { space: "joint"; mode: "return"; axis: null }
  | { space: "cartesian"; mode: "move_tcp"; axis: null }
  | { space: "joint"; mode: "teaching" | "replay"; axis: null }
  | { space: Space; mode: "calibration"; axis: number }
);
export interface Snapshot {
  seq: number;
  generation: number;
  phase: Phase;
  side: Side;
  port: string | null;
  enabled: boolean;
  owner: { client_id: string; expires_in_ms: number } | null;
  joints: Joint[];
  tcp: Pose | null;
  tcp_transform: Pose | null;
  tcp_configured: boolean;
  servo_tuning: ServoTuning;
  servo_tuning_revision: number;
  motion_tuning?: MotionTuning;
  feedback_age_ms: number | null;
  action: Action | null;
  calibration?: CalibrationStatus | null;
  teaching_mode?: TeachingMode | null;
  teaching_recording?: TeachingStatus | null;
  teaching?: TeachingStatus | null;
  latest_teaching?: TeachingSummary | null;
  last_result: Result | null;
  results: Result[];
  error: { code: string; message: string } | null;
  journal: {
    pending: number;
    written: number;
    dropped: number;
    errors: number;
    healthy: boolean;
  };
  communication: { received: number; crc_errors: number; timeouts: number };
}
export interface Port {
  name: string;
  label: string;
  kind: string;
}
export interface Log {
  id: number;
  ts_ms: number;
  level: string;
  source: string;
  code: string;
  message: string;
  request_id: string | null;
}
export interface LogPageQuery {
  page: number;
  page_size: number;
  search: string;
  through_id?: number;
  level?: string;
  source?: string;
  code?: string;
  from_ms?: number;
  to_ms?: number;
}
export interface LogFilters {
  levels: string[];
  sources: string[];
  codes: string[];
}
export interface LogPage {
  items: Log[];
  page: number;
  page_size: number;
  total: number;
  total_pages: number;
  through_id: number;
}
export interface TraceSummary {
  id: string;
  started_at_ms: number;
  duration_ms: number;
  side: string;
  port: string | null;
  status: "recording" | "writing" | "complete" | "failed";
  accepted: number;
  written: number;
  dropped: number;
  pending: number;
  errors: number;
}
export interface Session {
  base_url: string;
  token: string;
  client_id: string;
  version: string;
  journal_path: string;
}
export interface ControlReply {
  lease_id: string | null;
  expires_in_ms: number;
  generation: number;
}
export interface CommandAck {
  request_id: string;
  status: "queued";
}
export class ApiError extends Error {
  constructor(
    message: string,
    readonly status: number | null = null,
    readonly code: string | null = null,
    readonly notQueued = status !== null &&
      ((status >= 400 && status < 500 && status !== 408 && status !== 499) ||
        (status === 503 && code === "closing")),
  ) {
    super(message);
    this.name = "ApiError";
  }
}
export interface MotionArguments {
  space: Space;
  axis: number;
  direction: -1 | 1;
  speed: number;
  frame?: Frame;
}
export interface ReturnToPoseArguments {
  joints: JointPositions;
  speed: number;
  disconnect_after?: boolean;
}
export type Operation =
  | { kind: "connect"; port: string; side: Side }
  | { kind: "disconnect" | "disable"; allow_release?: boolean }
  | { kind: "enable" | "clear_fault" | "stop" }
  | ({ kind: "return_to_pose" } & ReturnToPoseArguments)
  | { kind: "configure_tcp"; tcp: Pose }
  | { kind: "move_tcp"; tcp: Pose; duration_s: number }
  | { kind: "configure_servo"; config: ServoTuning }
  | { kind: "configure_motion"; config: MotionTuning }
  | { kind: "calibrate_start"; settings?: Partial<CalibrationSettings> }
  | { kind: "calibrate_apply"; calibration_id: string }
  | { kind: "teach_mode"; enabled: boolean; transition_s?: number }
  | { kind: "teach_start"; name?: string }
  | { kind: "teach_finish"; recording_request_id?: string }
  | ({ kind: "teach_replay"; trajectory_id: string } & TeachingPlaybackOptions)
  | ({
      kind: "jog_start";
      input_seq: number;
      ttl_ms?: number;
    } & MotionArguments)
  | {
      kind: "jog_update";
      action_id: string;
      input_seq: number;
      ttl_ms?: number;
    }
  | ({ kind: "step"; amount: number } & MotionArguments);
export type CommandRequest = Operation & {
  request_id: string;
  client_id: string;
  lease_id: string;
  generation: number;
};
export const emptySnapshot: Snapshot = {
  seq: 0,
  generation: 0,
  phase: "disconnected",
  side: "right",
  port: null,
  enabled: false,
  owner: null,
  joints: [],
  tcp: null,
  tcp_transform: null,
  tcp_configured: false,
  servo_tuning: createServoTuning(),
  servo_tuning_revision: 0,
  motion_tuning: { ...DEFAULT_MOTION_TUNING },
  feedback_age_ms: null,
  action: null,
  calibration: null,
  teaching_mode: null,
  teaching_recording: null,
  teaching: null,
  latest_teaching: null,
  last_result: null,
  results: [],
  error: null,
  journal: { pending: 0, written: 0, dropped: 0, errors: 0, healthy: true },
  communication: { received: 0, crc_errors: 0, timeouts: 0 },
};
export interface View {
  snapshot: Snapshot;
  connectionEpoch?: number;
  online: boolean;
  receivedAt: number;
  error: string;
  logs: Log[];
  ports: Port[];
  baseUrl: string;
  hasControl: boolean;
}
export function hasJointFeedback(view: View) {
  const s = view.snapshot;
  return (
    view.online &&
    !["disconnected", "connecting", "closing"].includes(s.phase) &&
    isJointPositions(s.joints.map((joint) => joint.q))
  );
}
class DesktopApi {
  private session: Session | null = null;
  private lease: string | null = null;
  private socket: WebSocket | null = null;
  private renewTimer: ReturnType<typeof setTimeout> | null = null;
  private pollTimer: ReturnType<typeof setInterval> | null = null;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private jog: {
    id: string;
    seq: number;
    generation: number;
    timer: ReturnType<typeof setInterval> | null;
  } | null = null;
  private listeners = new Set<(v: View) => void>();
  private stopped = true;
  private renewing = false;
  private leaseEpoch = 0;
  private polling = false;
  private cancelReturn: (() => void) | null = null;
  private calibrationRequest: { id: string; pending: boolean } | null = null;
  private teachingRequests = new Set<string>();
  view: View = {
    snapshot: emptySnapshot,
    connectionEpoch: 0,
    online: false,
    receivedAt: 0,
    error: "",
    logs: [],
    ports: [],
    baseUrl: "",
    hasControl: false,
  };
  subscribe(fn: (v: View) => void) {
    this.listeners.add(fn);
    fn(this.view);
    return () => {
      this.listeners.delete(fn);
    };
  }
  private publish(p: Partial<View>) {
    this.view = { ...this.view, ...p };
    this.listeners.forEach((fn) => fn(this.view));
  }
  fail(e: unknown) {
    this.publish({ error: this.errorMessage(e) });
  }
  clearError() {
    this.publish({ error: "" });
  }
  getConnectionInfo() {
    return this.session
      ? { base_url: this.session.base_url, token: this.session.token }
      : null;
  }
  async copyConnectionInfo(field: "base_url" | "token"): Promise<boolean> {
    const info = this.getConnectionInfo();
    if (!info || !navigator.clipboard?.writeText) return false;
    try {
      await navigator.clipboard.writeText(info[field]);
      return true;
    } catch {
      return false;
    }
  }
  private errorMessage(error: unknown): string {
    let message = error instanceof Error ? error.message : String(error);
    for (const secret of [this.session?.token, this.lease]) {
      if (secret) message = message.split(secret).join("[已隐藏]");
    }
    return (
      message
        .replace(/\bBearer\s+[^\s"'<>]+/gi, "Bearer [已隐藏]")
        .replace(
          /(["']?(?:token|lease_id)["']?\s*[:=]\s*["']?)[^"',}\s]+/gi,
          "$1[已隐藏]",
        )
        .replace(/\s+/g, " ")
        .trim()
        .slice(0, 400) || "请求失败"
    );
  }
  private async request<T>(
    path: string,
    body?: unknown,
    options: {
      signal?: AbortSignal;
      download?: boolean;
      timeoutMs?: number | null;
    } = {},
  ): Promise<T> {
    if (!this.session) throw new Error("未连接");
    try {
      const timeout =
        options.timeoutMs === null
          ? undefined
          : AbortSignal.timeout(
              options.timeoutMs ?? (options.download ? 20000 : 4000),
            );
      const response = await fetch(
        `${this.session.base_url.replace(/\/$/, "")}${path}`,
        {
          method: body ? "POST" : "GET",
          headers: {
            Authorization: `Bearer ${this.session.token}`,
            ...(body ? { "Content-Type": "application/json" } : {}),
          },
          body: body ? JSON.stringify(body) : undefined,
          signal:
            options.signal && timeout
              ? AbortSignal.any([options.signal, timeout])
              : (options.signal ?? timeout),
        },
      );
      if (response.ok && options.download) return (await response.blob()) as T;
      const text = await response.text().catch((error) => {
        if (!response.ok)
          throw new ApiError(`HTTP ${response.status}`, response.status);
        throw error;
      });
      let data: unknown;
      try {
        data = JSON.parse(text);
      } catch {
        if (response.ok) throw new Error("响应数据格式无效");
      }
      if (!response.ok) {
        const fields =
          data && typeof data === "object"
            ? (data as { code?: unknown; message?: unknown; error?: unknown })
            : null;
        const nested =
          fields?.error && typeof fields.error === "object"
            ? (fields.error as { code?: unknown; message?: unknown })
            : null;
        const message = [fields?.message, nested?.message, fields?.error].find(
          (value): value is string => typeof value === "string" && !!value,
        );
        const code = [fields?.code, nested?.code].find(
          (value): value is string => typeof value === "string" && !!value,
        );
        throw new ApiError(
          this.errorMessage(
            message ??
              (text.trim() && !text.trimStart().startsWith("<")
                ? text
                : `HTTP ${response.status}`),
          ),
          response.status,
          code ? this.errorMessage(code) : null,
        );
      }
      return data as T;
    } catch (error) {
      if (error instanceof ApiError) throw error;
      throw new ApiError(this.errorMessage(error));
    }
  }
  async readLogsPage(query: LogPageQuery, signal?: AbortSignal) {
    const params = new URLSearchParams({
      page: String(query.page),
      page_size: String(query.page_size),
      search: query.search,
    });
    if (query.through_id !== undefined)
      params.set("through_id", String(query.through_id));
    for (const key of [
      "level",
      "source",
      "code",
      "from_ms",
      "to_ms",
    ] as const) {
      const value = query[key];
      if (value !== undefined && value !== "") params.set(key, String(value));
    }
    return this.request<LogPage>(`/v1/logs/page?${params}`, undefined, {
      signal,
    });
  }
  async readLogFilters(signal?: AbortSignal) {
    return this.request<LogFilters>("/v1/logs/filters", undefined, { signal });
  }
  async startTrace(durationMs = 10000, signal?: AbortSignal) {
    return this.request<TraceSummary>(
      "/v1/traces",
      { duration_ms: durationMs },
      { signal },
    );
  }
  async listTraces(limit = 20, signal?: AbortSignal) {
    return this.request<TraceSummary[]>(
      `/v1/traces?limit=${limit}`,
      undefined,
      { signal },
    );
  }
  async listCalibrations(limit = 20, signal?: AbortSignal) {
    const reports = await this.request<CalibrationReport[]>(
      `/v1/calibrations?limit=${encodeURIComponent(String(limit))}`,
      undefined,
      { signal },
    );
    return reports.map(parseCalibrationReport);
  }
  async listTeachings(signal?: AbortSignal) {
    const saved = await this.request<TeachingSummary[]>(
      "/v1/teachings",
      undefined,
      { signal, timeoutMs: null },
    );
    const latest = this.view.snapshot.latest_teaching;
    return latest
      ? [latest, ...saved.filter((item) => item.id !== latest.id)]
      : saved;
  }
  async readTeaching(id: string, signal?: AbortSignal) {
    return this.request<TeachingTrajectory>(
      `/v1/teachings/${encodeURIComponent(id)}`,
      undefined,
      { signal, timeoutMs: null },
    );
  }
  async previewTeaching(
    id: string,
    options: TeachingPlaybackOptions,
    signal?: AbortSignal,
  ) {
    return this.request<TeachingPreview>(
      `/v1/teachings/${encodeURIComponent(id)}/preview`,
      options,
      { signal, timeoutMs: null },
    );
  }
  async downloadTeaching(id: string, signal?: AbortSignal) {
    return this.request<Blob>(
      `/v1/teachings/${encodeURIComponent(id)}?download=true`,
      undefined,
      { signal, download: true, timeoutMs: null },
    );
  }
  async previewCalibration(
    request: CalibrationPreviewRequest,
    signal?: AbortSignal,
  ) {
    const preview = await this.request<CalibrationPreview>(
      "/v1/calibrations/preview",
      request,
      { signal, timeoutMs: null },
    );
    if (
      !preview?.program ||
      !isJointPositions(preview.program.origin) ||
      !Number.isFinite(preview.program.joint_excursion) ||
      !Array.isArray(preview.points) ||
      preview.points.length < 2 ||
      !preview.points.every(
        (point) =>
          isJointPositions(point.joints) &&
          point.tcp?.length === 3 &&
          point.tcp.every(Number.isFinite) &&
          typeof point.space === "string",
      )
    )
      throw new Error("预览数据不完整，请重新生成");
    return preview;
  }
  async readCalibration(id: string, signal?: AbortSignal, points = 3000) {
    return parseCalibrationReport(
      await this.request<CalibrationReport>(
        `/v1/calibrations/${encodeURIComponent(id)}?points=${encodeURIComponent(String(points))}`,
        undefined,
        { signal },
      ),
    );
  }
  async downloadCalibration(id: string, signal?: AbortSignal) {
    return this.request<Blob>(
      `/v1/calibrations/${encodeURIComponent(id)}?download=true`,
      undefined,
      { signal, download: true },
    );
  }
  async stopTrace(signal?: AbortSignal) {
    return this.request<TraceSummary | null>("/v1/traces/stop", {}, { signal });
  }
  async downloadTrace(id: string, signal?: AbortSignal) {
    return this.request<Blob>(
      `/v1/traces/${encodeURIComponent(id)}?attachment=true`,
      undefined,
      { signal, download: true },
    );
  }
  async readState(): Promise<Snapshot> {
    return this.request<Snapshot>("/v1/state");
  }
  async start() {
    this.stopped = false;
    try {
      this.session = await invoke<Session>("session_info");
      if (this.stopped) return;
      this.publish({ baseUrl: this.session.base_url });
      this.openSocket();
      await this.refreshPorts();
    } catch (e) {
      if ("__TAURI_INTERNALS__" in window) this.fail(e);
    }
    if (!this.stopped && this.session)
      this.pollTimer = setInterval(() => void this.poll(), 1500);
  }
  dispose() {
    if (
      this.cancelReturn ||
      this.hasLocalCalibration() ||
      this.hasLocalTeaching()
    )
      void this.hostStop("desktop_disposed").catch((error) => this.fail(error));
    this.stopped = true;
    this.clearJog();
    this.dropLease();
    this.socket?.close();
    if (this.pollTimer) clearInterval(this.pollTimer);
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
  }
  private accept(snapshot: Snapshot) {
    if (this.view.online && snapshot.seq < this.view.snapshot.seq) return;
    const previous = this.view.snapshot;
    const detached = (s: Snapshot) =>
      ["disconnected", "connecting", "closing"].includes(s.phase);
    const connectionChanged =
      snapshot.side !== previous.side ||
      snapshot.port !== previous.port ||
      detached(snapshot) !== detached(previous);
    snapshot = {
      ...snapshot,
      servo_tuning: snapshot.servo_tuning ?? createServoTuning(),
      servo_tuning_revision: snapshot.servo_tuning_revision ?? 0,
    };
    if (this.calibrationRequest) {
      const id = this.calibrationRequest.id;
      const result = snapshot.results?.find((item) => item.request_id === id);
      const calibration = snapshot.calibration;
      if (
        connectionChanged ||
        (result && result.status !== "accepted") ||
        (calibration?.request_id === id &&
          ["completed", "cancelled", "failed"].includes(calibration.state))
      )
        this.calibrationRequest = null;
      else if (
        snapshot.action?.request_id === id ||
        calibration?.request_id === id
      )
        this.calibrationRequest.pending = false;
    }
    for (const id of this.teachingRequests) {
      const result =
        snapshot.results?.find((item) => item.request_id === id) ??
        (snapshot.last_result?.request_id === id ? snapshot.last_result : null);
      const active =
        snapshot.teaching_recording?.request_id === id ||
        snapshot.teaching_mode?.request_id === id ||
        snapshot.teaching?.request_id === id ||
        snapshot.action?.request_id === id;
      if (connectionChanged || (result && result.status !== "accepted"))
        this.teachingRequests.delete(id);
      else if (active || result?.status === "accepted") {
        // Once accepted, the three independent snapshot fields describe the
        // running operation. Keep only requests whose outcome is still unknown.
        this.teachingRequests.delete(id);
      }
    }
    if (this.lease && snapshot.owner?.client_id !== "desktop")
      this.dropLease(false);
    if (
      this.jog &&
      (snapshot.generation !== this.jog.generation ||
        !snapshot.enabled ||
        snapshot.phase === "fault" ||
        snapshot.results?.some(
          (r) =>
            r.request_id === this.jog?.id &&
            ["rejected", "cancelled", "failed", "completed"].includes(r.status),
        ))
    )
      this.clearJog();
    this.publish({
      snapshot,
      connectionEpoch:
        (this.view.connectionEpoch ?? 0) + (connectionChanged ? 1 : 0),
      online: true,
      receivedAt: performance.now(),
      hasControl: !!this.lease && snapshot.owner?.client_id === "desktop",
    });
  }
  private openSocket() {
    if (!this.session || this.stopped) return;
    const url = new URL(
      `${this.session.base_url.replace(/\/$/, "")}/v1/events`,
    );
    url.protocol = url.protocol === "https:" ? "wss:" : "ws:";
    const ws = new WebSocket(url);
    this.socket = ws;
    ws.onopen = () => ws.send(JSON.stringify({ token: this.session!.token }));
    ws.onmessage = (e) => {
      try {
        const data = JSON.parse(e.data);
        if (data.type === "state") this.accept(data.data);
      } catch {
        this.fail(new Error("状态数据无效"));
      }
    };
    ws.onclose = () => {
      if (this.socket !== ws) return;
      const active =
        !!this.jog ||
        !!this.cancelReturn ||
        this.hasLocalCalibration() ||
        this.hasLocalTeaching();
      this.clearJog();
      this.dropLease(false);
      this.publish({ online: false, hasControl: false });
      if (active)
        void this.hostStop("websocket_closed").catch((e) => this.fail(e));
      if (!this.stopped)
        this.reconnectTimer = setTimeout(() => this.openSocket(), 1500);
    };
  }
  async refreshPorts() {
    try {
      this.publish({ ports: await this.request<Port[]>("/v1/ports") });
    } catch (e) {
      this.fail(e);
    }
  }
  private async poll() {
    if (this.polling) return;
    this.polling = true;
    try {
      const logs = await this.request<Log[]>("/v1/logs?limit=50");
      if (!this.stopped) this.publish({ logs });
    } catch (e) {
      this.fail(e);
    } finally {
      this.polling = false;
    }
  }
  private dropLease(notify = true) {
    this.leaseEpoch++;
    this.lease = null;
    if (this.renewTimer) clearTimeout(this.renewTimer);
    this.renewTimer = null;
    if (notify) this.publish({ hasControl: false });
  }
  async control(action: "acquire" | "release" | "renew") {
    if (action === "release") {
      this.leaseEpoch++;
      if (this.renewTimer) clearTimeout(this.renewTimer);
      if (this.hasLocalCalibration() || this.hasLocalTeaching())
        await this.hostStop("release_control");
      else await this.cancelJog("release_control");
    }
    const epoch = this.leaseEpoch;
    const data = await this.request<ControlReply>("/v1/control", {
      action,
      client_id: "desktop",
      ...(this.lease ? { lease_id: this.lease } : {}),
    });
    if (epoch !== this.leaseEpoch) return;
    if (action === "release") {
      this.dropLease();
      return;
    }
    this.lease = data.lease_id ?? this.lease;
    if (this.renewTimer) clearTimeout(this.renewTimer);
    this.renewTimer = setTimeout(
      () => void this.renew(),
      Math.max(100, Math.min(1000, data.expires_in_ms / 3)),
    );
    this.accept(await this.request<Snapshot>("/v1/state"));
  }
  private async renew() {
    if (!this.lease || this.stopped) return;
    if (this.renewing) {
      this.renewTimer = setTimeout(() => void this.renew(), 100);
      return;
    }
    const epoch = this.leaseEpoch;
    this.renewing = true;
    try {
      await this.control("renew");
    } catch (e) {
      if (epoch === this.leaseEpoch) {
        const calibrationActive =
          this.hasLocalCalibration() || this.hasLocalTeaching();
        this.dropLease();
        if (calibrationActive)
          await this.hostStop("renew_failed").catch(() => {});
        else await this.cancelJog("renew_failed").catch(() => {});
        this.fail(e);
      }
    } finally {
      this.renewing = false;
    }
  }
  async command(
    kind: string,
    params: Record<string, unknown> = {},
    requestId: string = crypto.randomUUID(),
    generation = this.view.snapshot.generation,
    signal?: AbortSignal,
  ) {
    if (!this.view.online)
      throw new ApiError("未连接", null, "not_connected", true);
    if (!this.lease || !this.view.hasControl)
      throw new ApiError("未持有控制权", null, "no_control", true);
    if (kind === "calibrate_start")
      this.calibrationRequest = { id: requestId, pending: true };
    if (["teach_start", "teach_mode", "teach_replay"].includes(kind))
      this.teachingRequests.add(requestId);
    try {
      return await this.request<CommandAck>(
        "/v1/commands",
        {
          ...params,
          kind,
          request_id: requestId,
          client_id: "desktop",
          lease_id: this.lease,
          generation,
        },
        kind.startsWith("teach_") || kind === "calibrate_start"
          ? { signal, timeoutMs: null }
          : { signal },
      );
    } catch (error) {
      if (
        error instanceof ApiError &&
        error.notQueued &&
        this.calibrationRequest?.id === requestId &&
        this.calibrationRequest.pending
      )
        this.calibrationRequest = null;
      if (error instanceof ApiError && error.notQueued)
        this.teachingRequests.delete(requestId);
      throw error;
    }
  }
  async configureTcp(tcp: Pose): Promise<Pose> {
    const { snapshot, connectionEpoch } = this.view;
    const requestId = crypto.randomUUID();
    const ack = await this.command(
      "configure_tcp",
      { tcp },
      requestId,
      snapshot.generation,
    );
    if (ack.request_id !== requestId || ack.status !== "queued")
      throw new Error("TCP 请求响应无效");

    const norm = Math.hypot(...tcp.rotation);
    let unsubscribe = () => {};
    let timeout: ReturnType<typeof setTimeout> | undefined;
    try {
      return await new Promise<Pose>((resolve, reject) => {
        timeout = setTimeout(() => reject(new Error("未确认 TCP 生效")), 4000);
        unsubscribe = this.subscribe((view) => {
          if (!view.online || view.connectionEpoch !== connectionEpoch) {
            reject(new Error("连接已变化，TCP 未确认"));
            return;
          }
          const { results, last_result, tcp_configured, tcp_transform } =
            view.snapshot;
          const result =
            results.find((item) => item.request_id === requestId) ??
            (last_result?.request_id === requestId ? last_result : null);
          if (!result) return;
          if (result.status !== "accepted" || result.code !== "configure_tcp") {
            reject(new Error(result.message || "TCP 未应用"));
            return;
          }
          if (
            !tcp_configured ||
            !tcp_transform ||
            !tcp_transform.translation.every(
              (v, i) => v === tcp.translation[i],
            ) ||
            !tcp_transform.rotation.every(
              (v, i) => Math.abs(v - tcp.rotation[i] / norm) < 1e-9,
            )
          ) {
            reject(new Error("TCP 配置已变化，无法确认本次应用"));
            return;
          }
          resolve(tcp_transform);
        });
      });
    } finally {
      clearTimeout(timeout);
      unsubscribe();
    }
  }
  async configureServo(
    config: ServoTuning,
    signal?: AbortSignal,
  ): Promise<Result> {
    if (signal?.aborted) throw new Error("参数确认已取消");
    if (!isServoTuning(config)) throw new Error("请输入七轴有效参数");
    const current = this.view;
    const initial = current.snapshot;
    if (!current.online) throw new Error("未连接");
    if (!current.hasControl) throw new Error("未持有控制权");
    if (
      !["disabled", "ready"].includes(initial.phase) ||
      initial.action ||
      this.jog ||
      this.cancelReturn
    )
      throw new Error("请等待机械臂静止");
    const submitted = structuredClone(config);
    const requestId = crypto.randomUUID();
    const revision = initial.servo_tuning_revision ?? 0;
    const connectionEpoch = current.connectionEpoch ?? 0;
    return new Promise<Result>((resolve, reject) => {
      let settled = false;
      let acknowledged = false;
      let accepted: Result | null = null;
      let unsubscribe = () => {};
      const finish = (error: Error | null, result?: Result) => {
        if (settled) return;
        settled = true;
        clearTimeout(timer);
        unsubscribe();
        signal?.removeEventListener("abort", abort);
        if (error) reject(error);
        else resolve(result!);
      };
      const abort = () => finish(new Error("参数确认已取消"));
      const timer = setTimeout(() => finish(new Error("未确认参数生效")), 8000);
      const inspect = (view: View) => {
        if (settled) return;
        const snapshot = view.snapshot;
        const result =
          snapshot.results?.find((item) => item.request_id === requestId) ??
          (snapshot.last_result?.request_id === requestId
            ? snapshot.last_result
            : null);
        if (result) {
          if (result.status === "accepted" && result.code === "configure_servo")
            accepted = result;
          else {
            finish(
              new Error(
                [
                  ["accepted", "completed"].includes(result.status)
                    ? "参数结果不匹配"
                    : "",
                  result.code,
                  result.message,
                ]
                  .filter(Boolean)
                  .join(" · ") || "参数未应用",
              ),
            );
            return;
          }
        }
        if (
          !view.online ||
          (view.connectionEpoch ?? 0) !== connectionEpoch ||
          snapshot.side !== initial.side ||
          snapshot.port !== initial.port ||
          ["disconnected", "connecting", "closing"].includes(snapshot.phase)
        ) {
          finish(new Error("连接已变化，参数未确认"));
          return;
        }
        if (
          acknowledged &&
          accepted &&
          snapshot.servo_tuning_revision === revision + 1 &&
          snapshot.servo_tuning &&
          sameServoTuning(snapshot.servo_tuning, submitted)
        ) {
          finish(null, accepted);
          return;
        }
        if (snapshot.servo_tuning_revision > revision + 1)
          finish(new Error("配置版本已变化，参数未确认"));
        // Ownership and generation gate dispatch. After dispatch, observe this
        // request's result even if another controller takes over; never resend.
      };
      signal?.addEventListener("abort", abort, { once: true });
      unsubscribe = this.subscribe(inspect);
      if (settled) {
        unsubscribe();
        return;
      }
      void this.command(
        "configure_servo",
        { config: submitted },
        requestId,
        initial.generation,
      )
        .then((ack) => {
          if (ack.request_id !== requestId || ack.status !== "queued") {
            finish(new Error("参数请求响应无效"));
            return;
          }
          acknowledged = true;
          inspect(this.view);
        })
        .catch((error) => finish(new Error(this.errorMessage(error))));
    });
  }
  async returnToPose(
    params: ReturnToPoseArguments,
    signal: AbortSignal,
    requestId: string = crypto.randomUUID(),
  ): Promise<Result> {
    if (signal.aborted) throw new Error("返回已取消");
    if (this.cancelReturn) throw new Error("正在返回收纳位");
    if (!isJointPositions(params.joints))
      throw new Error("收纳位必须包含七个有效关节位置");
    if (!Number.isFinite(params.speed) || params.speed <= 0)
      throw new Error("返回速度须为有限正数");
    const s = this.view.snapshot;
    if (!hasJointFeedback(this.view))
      throw new Error("需要完整、有效的七轴反馈");
    if (!s.enabled || s.phase !== "ready" || s.action || this.jog)
      throw new Error("请先停止动作，并等待使能就绪");
    if (!this.view.hasControl) throw new Error("请先取得控制权");
    const generation = s.generation;
    return new Promise<Result>((resolve, reject) => {
      let settled = false;
      let cancelled = false;
      let unsubscribe = () => {};
      const finish = (error: Error | null, result?: Result) => {
        if (settled) return;
        settled = true;
        unsubscribe();
        signal.removeEventListener("abort", abort);
        if (this.cancelReturn === cancel) this.cancelReturn = null;
        if (error) reject(error);
        else resolve(result!);
      };
      const cancel = () => {
        cancelled = true;
        finish(new Error("返回已停止"));
      };
      const abort = () => {
        cancel();
        void this.hostStop("parking_dialog_cancelled").catch((error) =>
          this.fail(error),
        );
      };
      this.cancelReturn = cancel;
      signal.addEventListener("abort", abort, { once: true });
      unsubscribe = this.subscribe((view) => {
        if (!view.online) {
          cancelled = true;
          finish(new Error("状态连接已断开"));
          return;
        }
        // Results are newest first. An older accepted/terminal entry must never
        // override the latest result for this exact request.
        const result = view.snapshot.results.find(
          (item) => item.request_id === requestId,
        );
        if (result && result.status !== "accepted") {
          const expectedCode = params.disconnect_after ? "parked" : "returned";
          if (result.status === "completed" && result.code === expectedCode)
            finish(null, result);
          else finish(new Error(result.message || "返回未完成"));
          return;
        }
        if (
          view.snapshot.generation !== generation ||
          !view.hasControl ||
          !view.snapshot.enabled ||
          view.snapshot.phase === "fault"
        ) {
          cancelled = true;
          finish(new Error("返回条件已变化，已停止"));
          void this.hostStop("parking_context_changed").catch((error) =>
            this.fail(error),
          );
        }
      });
      void this.command(
        "return_to_pose",
        { ...params, joints: [...params.joints] },
        requestId,
        generation,
      )
        .then(() => {
          if (cancelled) return this.hostStop("parking_cancelled_before_ack");
        })
        .catch((error) => {
          if (settled && !cancelled) return;
          cancelled = true;
          finish(new Error(this.errorMessage(error)));
          void this.hostStop("parking_request_failed").catch((stopError) =>
            this.fail(stopError),
          );
        });
    });
  }
  async cancelReturnToPose(reason = "return_cancelled") {
    if (this.cancelReturn) await this.hostStop(reason);
  }
  async closeApp(allowRelease = false) {
    await invoke("close_app", { allowRelease });
  }
  async startJog(params: Record<string, unknown>) {
    if (this.jog || this.view.snapshot.action) throw new Error("运动中");
    const jog = {
      id: crypto.randomUUID(),
      seq: 0,
      generation: this.view.snapshot.generation,
      timer: null as ReturnType<typeof setInterval> | null,
      updating: false,
    };
    this.jog = jog;
    try {
      await this.command(
        "jog_start",
        { ...params, ttl_ms: 350, input_seq: 0 },
        jog.id,
        jog.generation,
      );
      if (this.jog !== jog) return;
      jog.timer = setInterval(() => {
        if (this.jog !== jog || jog.updating) return;
        jog.updating = true;
        void this.command(
          "jog_update",
          { action_id: jog.id, input_seq: ++jog.seq, ttl_ms: 350 },
          crypto.randomUUID(),
          jog.generation,
        )
          .catch((e) => {
            if (this.jog === jog) {
              this.fail(e);
              void this.cancelJog("update_failed").catch((e) => this.fail(e));
            }
          })
          .finally(() => {
            jog.updating = false;
          });
      }, 100);
    } catch (e) {
      if (this.jog !== jog) return;
      this.clearJog();
      void this.hostStop("start_failed").catch((e) => this.fail(e));
      throw e;
    }
  }
  private clearJog() {
    if (this.jog?.timer) clearInterval(this.jog.timer);
    this.jog = null;
  }
  async cancelJog(reason = "unspecified") {
    if (!this.jog) return;
    this.clearJog();
    await this.hostStop(reason);
  }
  private hasLocalCalibration() {
    const request = this.calibrationRequest;
    if (!request) return false;
    const s = this.view.snapshot;
    if (s.action)
      return (
        s.action.mode === "calibration" && s.action.request_id === request.id
      );
    if (s.calibration?.request_id === request.id)
      return ["running", "stopping"].includes(s.calibration.state);
    return request.pending && this.view.hasControl;
  }
  private hasLocalTeaching() {
    const s = this.view.snapshot;
    return (
      (s.owner?.client_id === "desktop" &&
        !!(s.teaching_mode || s.teaching_recording || s.teaching)) ||
      (this.view.hasControl && this.teachingRequests.size > 0)
    );
  }
  async closeTeaching(pendingStart?: string) {
    const recording = this.view.snapshot.teaching_recording;
    if (
      (recording || pendingStart) &&
      this.view.online &&
      this.view.hasControl
    ) {
      const requestId = crypto.randomUUID();
      let settled = false;
      let finish: () => void;
      const completed = new Promise<void>((resolve) => {
        finish = resolve;
      });
      const end = () => {
        if (settled) return;
        settled = true;
        finish();
      };
      const unsubscribe = this.subscribe((view) => {
        if (settled) return;
        const result =
          view.snapshot.results?.find(
            (item) => item.request_id === requestId,
          ) ??
          (view.snapshot.last_result?.request_id === requestId
            ? view.snapshot.last_result
            : null);
        if (result && result.status !== "accepted") {
          end();
          if (recording && ["failed", "rejected"].includes(result.status))
            this.fail(new Error(result.message));
        } else if (
          !view.online ||
          !view.hasControl ||
          (recording &&
            !view.snapshot.teaching_recording &&
            view.snapshot.latest_teaching?.id === recording.id)
        )
          end();
      });
      // Confirmation comes from state, even when the HTTP acknowledgement is
      // delayed. Closing a modal never waits behind that acknowledgement.
      void this.command(
        "teach_finish",
        { recording_request_id: pendingStart ?? recording?.request_id },
        requestId,
      ).catch((error) => {
        if (settled) return;
        end();
        this.fail(error);
      });
      await completed;
      unsubscribe();
    }
    if (
      this.view.snapshot.teaching_mode ||
      this.view.snapshot.teaching ||
      this.teachingRequests.size
    )
      await this.hostStop("teaching_dialog_unmounted");
  }
  async hostStop(reason = "unspecified") {
    this.teachingRequests.clear();
    this.clearJog();
    this.calibrationRequest = null;
    this.cancelReturn?.();
    await invoke("host_stop", { reason });
  }
}
export const api = new DesktopApi();
