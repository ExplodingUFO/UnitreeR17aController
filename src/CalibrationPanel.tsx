import { useEffect, useMemo, useRef, useState, type ChangeEvent } from "react";
import {
  api,
  ApiError,
  DEFAULT_MOTION_TUNING,
  parseCalibrationReport,
  sameServoTuning,
  type CalibrationMetrics,
  type CalibrationPoint,
  type CalibrationPreview,
  type CalibrationReport,
  type CalibrationStatus,
  type CalibrationTrial,
  type ServoTuning,
  type View,
  hasJointFeedback,
} from "./api";
import RobotScene, { type PreviewEnvelope } from "./RobotScene";
import {
  jointTravel,
  stagePoints,
  stageTimes,
  sampleStage,
  type PreviewStage,
} from "./previewTimeline";
import {
  MotionTuningFields,
  toMotionDraft,
  readMotionDraft,
} from "./ServoPanel";
import CalibrationChart, {
  calibrationParameters,
  calibrationParameterValue,
  calibrationTrialLabel,
  calibrationTrials,
} from "./CalibrationChart";

export interface CalibrationPanelProps {
  view: View;
  open: boolean;
  onClose: () => void;
  onStop: () => void;
}
type Page = "settings" | "progress" | "result" | "curves" | "history";
const previewStages: PreviewStage[] = [
  "xyz",
  "cartesian:0",
  "cartesian:1",
  "cartesian:2",
  ...Array.from({ length: 7 }, (_, axis) => `joint:${axis}` as const),
  "return",
];
const browseSeconds = 16;
type Pending = {
  kind: "start" | "apply" | "restore";
  id: string;
  connection: string;
  generation: number;
  config: ServoTuning;
  accepted: boolean;
  uncertain: boolean;
};
type Run = {
  id: string;
  connection: string;
  state: "waiting" | "running" | "terminal";
};
type Feedback = { text: string; tone: "info" | "error" | "success" };
type Selection = { id: string; local: boolean };
const DEG = 180 / Math.PI;
const stages: Record<CalibrationStatus["stage"], string> = {
  cartesian_before: "末端路径 · 调整前",
  cartesian_after: "末端路径 · 调整后",
  baseline: "逐轴采集摩擦",
  identifying: "拟合摩擦曲线",
  candidate: "验证拟合参数",
  baseline_check: "复测基线",
  validation: "整体对比",
  after: "整体对比",
  restoring: "回到原位，恢复参数",
  done: "测试结束",
};
const terminal = (status: CalibrationStatus) =>
  ["completed", "cancelled", "failed"].includes(status.state);
const degrees = (value: number | null | undefined, digits = 3) =>
  value != null && Number.isFinite(value) ? (value * DEG).toFixed(digits) : "—";
const percent = (value: number) =>
  Math.round(
    Math.min(1, Math.max(0, Number.isFinite(value) ? value : 0)) * 100,
  );
const timestamp = (value: number) =>
  new Date(value).toLocaleString("zh-CN", {
    month: "2-digit",
    day: "2-digit",
    hour: "2-digit",
    minute: "2-digit",
    hour12: false,
  });
const axisLabel = (report: CalibrationReport) =>
  report.settings.axis == null ? "J1–J7" : `J${report.settings.axis + 1}`;
function outcome(report: CalibrationReport) {
  if (report.outcome === "cancelled") return "测试已停止";
  if (report.outcome === "failed") return "测试未完成";
  return report.recommended ? "测试完成，有推荐参数" : "测试完成，参数保持原样";
}
function change(before: number | undefined, after: number | undefined) {
  if (
    before == null ||
    after == null ||
    !Number.isFinite(before) ||
    !Number.isFinite(after) ||
    before <= 0
  )
    return null;
  return (before - after) / before;
}
function Change({ value }: { value: number | null }) {
  return value == null ? null : (
    <small data-change={value > 0 ? "better" : value < 0 ? "worse" : "same"}>
      {Math.abs(value) < 0.0005
        ? "持平"
        : `${value > 0 ? "降低" : "增加"} ${(Math.abs(value) * 100).toFixed(1)}%`}
    </small>
  );
}
function measuredAxes(report: CalibrationReport) {
  return Array.from({ length: 7 }, (_, axis) => axis).filter(
    (axis) =>
      calibrationTrials(report, axis).length > 0 ||
      report.identification?.some((fit) => fit.axis === axis) ||
      (report.recommended &&
        calibrationParameters(
          report.baseline.joints[axis],
          report.recommended.joints[axis],
        ).some((row) => row.before !== row.after)),
  );
}
function comparisonFor(report: CalibrationReport, axis: number) {
  const trials = calibrationTrials(report, axis);
  const choice = report.choices?.find((item) => item.axis === axis);
  const baseline =
    trials.find((trial) => trial.index === choice?.baseline_index) ??
    trials.find((trial) => trial.phase === "baseline") ??
    trials[0];
  const candidates = trials.filter(
    (trial) => trial !== baseline && trial.metrics,
  );
  const score = (trial: CalibrationTrial) => {
    const b = baseline?.metrics,
      m = trial.metrics;
    if (!b || !m) return Infinity;
    const velocity =
      b.velocity_rms > 0
        ? m.velocity_rms / b.velocity_rms
        : m.velocity_rms === 0
          ? 1
          : Infinity;
    const tracking =
      b.tracking_rms > 0
        ? m.tracking_rms / b.tracking_rms
        : m.tracking_rms === 0
          ? 1
          : Infinity;
    return 0.6 * velocity + 0.4 * tracking;
  };
  const comparison =
    trials.find((trial) => trial.index === choice?.after_index) ??
    trials.find((trial) => trial.phase === "after") ??
    trials.find((trial) => trial.index === choice?.candidate_index) ??
    candidates.find(
      (trial) =>
        report.recommended && sameServoTuning(trial.config, report.recommended),
    ) ??
    candidates.reduce<CalibrationTrial | undefined>(
      (best, trial) => (!best || score(trial) < score(best) ? trial : best),
      undefined,
    );
  return { baseline, comparison };
}
function MetricComparison({
  before,
  after,
  field,
}: {
  before: CalibrationMetrics | null | undefined;
  after: CalibrationMetrics | null | undefined;
  field: "tracking_rms" | "velocity_rms" | "tracking_peak";
}) {
  return (
    <td>
      <span>
        {degrees(before?.[field])} → {degrees(after?.[field])}
      </span>
      <Change value={change(before?.[field], after?.[field])} />
    </td>
  );
}
function ReportDetails({
  report,
  local,
  applied,
  restored,
  onCurves,
}: {
  report: CalibrationReport;
  local: boolean;
  applied: boolean;
  restored: boolean;
  onCurves: () => void;
}) {
  const axes = measuredAxes(report);
  const cartesianBefore = report.trials.find(
    (trial) => trial.phase === "cartesian_before",
  );
  const cartesianAfter = report.trials.find(
    (trial) => trial.phase === "cartesian_after",
  );
  const hasCartesian = !!cartesianBefore || !!cartesianAfter;
  const recordedCount = report.sample_count || report.curves?.length || 0;
  const capturedCount = report.captured_samples || recordedCount;
  const recordedDuration = report.curves?.length
    ? Math.max(...report.curves.map((point) => point.t_ms)) / 1000
    : null;
  const beforeMetrics = cartesianBefore?.metrics_by_joint;
  const afterMetrics = cartesianAfter?.metrics_by_joint;
  const cartesianAxes = Array.from({ length: 7 }, (_, axis) => axis).filter(
    (axis) => beforeMetrics?.[axis] || afterMetrics?.[axis],
  );
  const pairedCartesian =
    !!beforeMetrics?.some(Boolean) && !!afterMetrics?.some(Boolean);
  const hasJointStatistics = report.trials.some(
    (trial) =>
      trial.space !== "cartesian" &&
      !trial.phase.startsWith("cartesian_") &&
      trial.metrics,
  );
  const deltas = report.baseline.joints.flatMap((before, axis) =>
    calibrationParameters(before, report.recommended?.joints[axis])
      .filter((row) => row.after !== undefined && row.after !== row.before)
      .map((row) => ({ axis, ...row })),
  );
  const controlUnchanged =
    !report.recommended ||
    report.baseline.joints.every(
      (joint, axis) =>
        joint.kd === report.recommended!.joints[axis].kd &&
        joint.velocity_ff === report.recommended!.joints[axis].velocity_ff,
    );
  return (
    <div className="calibration-report">
      <div className="calibration-result-heading">
        <div>
          <small>
            {axisLabel(report)} · {timestamp(report.created_at_ms)}
            {local ? " · 导入报告，只读" : ""}
          </small>
          <h3>{outcome(report)}</h3>
        </div>
        <small>
          {report.curves?.length
            ? capturedCount > recordedCount
              ? `采集 ${capturedCount.toLocaleString()} · 保留 ${recordedCount.toLocaleString()}`
              : `${recordedCount.toLocaleString()} 采样`
            : report.trials.length
              ? `${report.trials.length} 轮测量`
              : "统计报告"}
        </small>
      </div>
      <p className="calibration-report-message">
        {["failed", "cancelled"].includes(report.outcome)
          ? report.message
          : report.recommended
            ? applied
              ? "已应用本次推荐参数。"
              : restored
                ? "已恢复起始参数。"
                : "推荐参数待应用。"
            : "各轮测量已保留，可查看每轴的变化与曲线。"}
      </p>
      {(recordedCount > 0 || !!report.identification?.length) && (
        <div className="calibration-recorded">
          <span>
            <b>
              {recordedCount > 0
                ? `${recordedCount.toLocaleString()} 点`
                : "力矩残差拟合"}
            </b>
            {recordedDuration != null && (
              <small>已录制 {recordedDuration.toFixed(3)} s</small>
            )}
            {!pairedCartesian && hasCartesian && (
              <small>末端动作 · 已采集部分</small>
            )}
          </span>
          <button onClick={onCurves}>
            {report.identification?.length ? "查看曲线 / 拟合" : "查看曲线"}
          </button>
        </div>
      )}
      {(!["failed", "cancelled"].includes(report.outcome) ||
        report.recommended) && (
        <>
          <details
            className="calibration-deltas"
            aria-label="参数变化"
            open={deltas.length <= 4}
          >
            <summary className="calibration-deltas-heading">
              <h4>{applied ? "已应用的参数变化" : "推荐参数变化"}</h4>
              <b>
                {deltas.some((row) => row.key.startsWith("stribeck"))
                  ? "双向 Stribeck · "
                  : ""}
                {deltas.length} 项
                {controlUnchanged ? " · Kd / 速度前馈保持原值" : ""}
              </b>
            </summary>
            {deltas.length ? (
              <div className="calibration-delta-list">
                {deltas.map(({ axis, key, label, unit, before, after }) => (
                  <div key={`${axis}:${key}`}>
                    <span>
                      J{axis + 1} · {label}
                    </span>
                    <strong>
                      {calibrationParameterValue(before)} <span>→</span>{" "}
                      {calibrationParameterValue(after)} {unit}
                    </strong>
                  </div>
                ))}
              </div>
            ) : (
              <p>逐轴参数保持原值。</p>
            )}
            {report.recommended &&
              report.baseline.friction_enabled !==
                report.recommended.friction_enabled && (
                <small>
                  摩擦补偿：{report.baseline.friction_enabled ? "开启" : "关闭"}{" "}
                  → {report.recommended.friction_enabled ? "开启" : "关闭"}
                </small>
              )}
          </details>
          {!!report.identification?.some((fit) => fit.max_friction != null) && (
            <div className="calibration-fit-budgets" aria-label="实际补偿幅值">
              <span>
                {report.settings.max_friction == null
                  ? "自动补偿幅值"
                  : "补偿幅值"}{" "}
                · Nm
              </span>
              {report.identification
                .filter((fit) => fit.max_friction != null)
                .map((fit) => (
                  <span key={fit.axis}>
                    J{fit.axis + 1} <b>{fit.max_friction}</b>
                  </span>
                ))}
            </div>
          )}
        </>
      )}
      {hasCartesian && cartesianAxes.length > 0 && (
        <>
          <h4 className="calibration-section-label">
            {pairedCartesian ? "末端动作前后对照" : "末端动作已录统计"}{" "}
            <small>
              {pairedCartesian
                ? "同一条路径 · 七轴实测响应 · 调整前 → 调整后"
                : "已采集的关节响应"}
            </small>
          </h4>
          <div className="calibration-table-scroll">
            <table
              className="calibration-table calibration-cartesian-results"
              aria-label="末端动作七轴前后对照"
            >
              <thead>
                <tr>
                  <th>关节</th>
                  <th>跟随 RMS · °</th>
                  <th>速度 RMS · °/s</th>
                  <th>跟随峰值 · °</th>
                </tr>
              </thead>
              <tbody>
                {cartesianAxes.map((axis) => (
                  <tr key={axis}>
                    <th>J{axis + 1}</th>
                    {pairedCartesian ? (
                      <MetricComparison
                        before={cartesianBefore?.metrics_by_joint?.[axis]}
                        after={cartesianAfter?.metrics_by_joint?.[axis]}
                        field="tracking_rms"
                      />
                    ) : (
                      <td>
                        {degrees(
                          (afterMetrics?.[axis] ?? beforeMetrics?.[axis])
                            ?.tracking_rms,
                        )}
                      </td>
                    )}
                    {pairedCartesian ? (
                      <MetricComparison
                        before={cartesianBefore?.metrics_by_joint?.[axis]}
                        after={cartesianAfter?.metrics_by_joint?.[axis]}
                        field="velocity_rms"
                      />
                    ) : (
                      <td>
                        {degrees(
                          (afterMetrics?.[axis] ?? beforeMetrics?.[axis])
                            ?.velocity_rms,
                        )}
                      </td>
                    )}
                    {pairedCartesian ? (
                      <MetricComparison
                        before={cartesianBefore?.metrics_by_joint?.[axis]}
                        after={cartesianAfter?.metrics_by_joint?.[axis]}
                        field="tracking_peak"
                      />
                    ) : (
                      <td>
                        {degrees(
                          (afterMetrics?.[axis] ?? beforeMetrics?.[axis])
                            ?.tracking_peak,
                        )}
                      </td>
                    )}
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </>
      )}
      {!hasCartesian && hasJointStatistics && axes.length > 0 && (
        <div className="calibration-table-scroll">
          <table
            className="calibration-table calibration-axis-results"
            aria-label="每轴测量比较"
          >
            <thead>
              <tr>
                <th>关节 / 对比</th>
                <th>跟随 RMS · °</th>
                <th>速度 RMS · °/s</th>
                <th>跟随峰值 · °</th>
              </tr>
            </thead>
            <tbody>
              {axes.map((axis) => {
                const { baseline, comparison } = comparisonFor(report, axis);
                return (
                  <tr key={axis}>
                    <th>
                      <b>J{axis + 1}</b>
                      <small>
                        {comparison
                          ? calibrationTrialLabel(report, comparison, axis)
                          : "仅基线"}
                      </small>
                    </th>
                    <MetricComparison
                      before={baseline?.metrics}
                      after={comparison?.metrics}
                      field="tracking_rms"
                    />
                    <MetricComparison
                      before={baseline?.metrics}
                      after={comparison?.metrics}
                      field="velocity_rms"
                    />
                    <MetricComparison
                      before={baseline?.metrics}
                      after={comparison?.metrics}
                      field="tracking_peak"
                    />
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
      )}
      {hasJointStatistics ? (
        <details className="calibration-round-details" open={!hasCartesian}>
          <summary>
            关节测量 <small>采集与拟合参数验证</small>
          </summary>
          <div className="calibration-table-scroll">
            <table
              className="calibration-table calibration-rounds"
              aria-label="各轮位置速度峰值对比"
            >
              <thead>
                <tr>
                  <th>试验</th>
                  <th>跟随 RMS · °</th>
                  <th>速度 RMS · °/s</th>
                  <th>跟随峰值 · °</th>
                  <th>有效时间 · s</th>
                </tr>
              </thead>
              <tbody>
                {report.trials
                  .filter(
                    (trial) =>
                      trial.space !== "cartesian" &&
                      !trial.phase.startsWith("cartesian_"),
                  )
                  .map((trial) => {
                    const axis = trial.axis ?? report.settings.axis;
                    const baseline =
                      axis == null
                        ? null
                        : comparisonFor(report, axis).baseline;
                    return (
                      <tr
                        key={`${trial.index}:${axis ?? "all"}:${trial.phase}`}
                      >
                        <th>
                          {axis == null ? "七轴" : `J${axis + 1}`} ·{" "}
                          {axis == null
                            ? trial.phase === "after"
                              ? "调整后"
                              : trial.phase === "baseline"
                                ? "调整前"
                                : trial.phase === "baseline_check"
                                  ? "基线复测"
                                  : trial.phase === "candidate"
                                    ? "拟合参数"
                                    : "整体对照"
                            : calibrationTrialLabel(report, trial, axis)}
                        </th>
                        <td>
                          {degrees(trial.metrics?.tracking_rms)}
                          {trial !== baseline && (
                            <Change
                              value={change(
                                baseline?.metrics?.tracking_rms,
                                trial.metrics?.tracking_rms,
                              )}
                            />
                          )}
                        </td>
                        <td>
                          {degrees(trial.metrics?.velocity_rms)}
                          {trial !== baseline && (
                            <Change
                              value={change(
                                baseline?.metrics?.velocity_rms,
                                trial.metrics?.velocity_rms,
                              )}
                            />
                          )}
                        </td>
                        <td>{degrees(trial.metrics?.tracking_peak)}</td>
                        <td>
                          {trial.metrics
                            ? (trial.metrics.valid_ms / 1000).toFixed(1)
                            : "—"}
                        </td>
                      </tr>
                    );
                  })}
              </tbody>
            </table>
          </div>
        </details>
      ) : (
        <p className="calibration-note">
          {recordedCount > 0
            ? "本次未完成对照测量，可查看已录制曲线。"
            : "此报告没有完整测量轮次。"}
        </p>
      )}
      <details className="calibration-parameters">
        <summary>参数明细</summary>
        <p className="calibration-note">
          摩擦补偿：起始{report.baseline.friction_enabled ? "开启" : "关闭"} ·
          推荐
          {report.recommended
            ? report.recommended.friction_enabled
              ? "开启"
              : "关闭"
            : "保持原样"}
        </p>
        <div className="calibration-table-scroll">
          <table className="calibration-table" aria-label="参数明细">
            <thead>
              <tr>
                <th>关节</th>
                <th>参数</th>
                <th>起始</th>
                <th>推荐</th>
              </tr>
            </thead>
            <tbody>
              {(axes.length ? axes : [report.settings.axis ?? 0]).flatMap(
                (axis) =>
                  calibrationParameters(
                    report.baseline.joints[axis],
                    report.recommended?.joints[axis],
                  ).map(({ key, label, unit, before, after }) => (
                    <tr key={`${axis}:${key}`}>
                      <th>J{axis + 1}</th>
                      <th>
                        {label} <small>{unit}</small>
                      </th>
                      <td>{calibrationParameterValue(before)}</td>
                      <td
                        data-changed={after !== undefined && after !== before}
                      >
                        {calibrationParameterValue(after)}
                      </td>
                    </tr>
                  )),
              )}
            </tbody>
          </table>
        </div>
      </details>
    </div>
  );
}

export default function CalibrationPanel({
  view,
  open,
  onClose,
  onStop,
}: CalibrationPanelProps) {
  const s = view.snapshot,
    status = s.calibration;
  const connection = `${view.baseUrl}:${view.connectionEpoch ?? 0}:${s.side}:${s.port ?? ""}:${["disconnected", "connecting", "closing"].includes(s.phase) ? "detached" : "attached"}`;
  const [page, setPage] = useState<Page>("settings");
  const [autoRange, setAutoRange] = useState(true);
  const [amplitudeText, setAmplitudeText] = useState("50");
  const [speedText, setSpeedText] = useState("10");
  const [jointSpeedsText, setJointSpeedsText] = useState("");
  const [frictionBudgetText, setFrictionBudgetText] = useState("");
  const speedEntries = jointSpeedsText.trim()
    ? jointSpeedsText
        .trim()
        .split(/[,，\s/]+/)
        .filter(Boolean)
    : [];
  const jointSpeeds = speedEntries.map((value) => Number(value) / DEG);
  const jointSpeedsValid =
    !jointSpeedsText.trim() ||
    (jointSpeeds.length > 0 &&
      jointSpeeds.every((value) => Number.isFinite(value) && value > 0));
  const jointSpeedsError = jointSpeedsValid
    ? ""
    : "关节速度须为正数，以逗号分隔";
  const frictionBudget = frictionBudgetText.trim()
    ? Number(frictionBudgetText)
    : null;
  const frictionBudgetValid =
    frictionBudget === null ||
    (Number.isFinite(frictionBudget) && frictionBudget >= 0);
  const [motionDraft, setMotionDraft] = useState(() =>
    toMotionDraft(s.motion_tuning ?? DEFAULT_MOTION_TUNING),
  );
  const [motionEdited, setMotionEdited] = useState(false);
  const appliedMotionKey = JSON.stringify(
    s.motion_tuning ?? DEFAULT_MOTION_TUNING,
  );
  useEffect(() => {
    if (!motionEdited)
      setMotionDraft(toMotionDraft(s.motion_tuning ?? DEFAULT_MOTION_TUNING));
  }, [appliedMotionKey, motionEdited]);
  const motion = readMotionDraft(motionDraft);
  const amplitude = autoRange ? null : Number(amplitudeText);
  const amplitudeValid =
    autoRange ||
    (amplitudeText.trim() !== "" &&
      Number.isFinite(amplitude) &&
      amplitude! > 0);
  const speed = Number(speedText);
  const speedValid =
    speedText.trim() !== "" && Number.isFinite(speed) && speed > 0;
  const tcpGeometry = s.tcp_transform
    ? [...s.tcp_transform.translation, ...s.tcp_transform.rotation]
        .map((value) => Math.round(value * 1e7))
        .join(":")
    : "unset";
  const previewContext = `${connection}:${tcpGeometry}`;
  const previewSettings = JSON.stringify({
    amplitude,
    speed,
    motion,
    jointSpeeds: jointSpeedsValid ? jointSpeeds : null,
  });
  const previewContextRef = useRef(previewContext);
  previewContextRef.current = previewContext;
  const [generated, setGenerated] = useState<{
    data: CalibrationPreview;
    context: string;
    settings: string;
  } | null>(null);
  const preview =
    generated?.context === previewContext &&
    generated.settings === previewSettings
      ? generated.data
      : null;
  const plannedSpeeds = preview?.program.joint_speeds;
  const speedLabel = plannedSpeeds?.length
    ? plannedSpeeds
        .map((value) => Number((value * DEG).toPrecision(6)))
        .join(" / ")
    : speedEntries.length && jointSpeedsValid
      ? speedEntries.join(" / ")
      : "1 / 2 / 4";
  const [previewBusy, setPreviewBusy] = useState(false);
  const previewRequest = useRef<AbortController | null>(null);
  const [previewProgress, setPreviewProgress] = useState(0.25);
  const [previewPlaying, setPreviewPlaying] = useState(false);
  const [previewStage, setPreviewStage] = useState<PreviewStage>("cartesian:0");
  const [envelope, setEnvelope] = useState<PreviewEnvelope | null>(null);
  const selectedPoints = useMemo(
    () => (preview ? stagePoints(preview, previewStage) : []),
    [preview, previewStage],
  );
  const availableStages = useMemo(
    () =>
      previewStages.filter(
        (stage) => preview && stagePoints(preview, stage).length,
      ),
    [preview],
  );
  const selectedTimes = useMemo(
    () => stageTimes(selectedPoints),
    [selectedPoints],
  );
  const stageSeconds = selectedTimes?.at(-1);
  const jointRange = useMemo(() => {
    const travel = preview && jointTravel(preview.program, selectedPoints);
    return travel
      ?.map(
        (radians) =>
          (radians * DEG).toLocaleString("zh-CN", {
            maximumFractionDigits: 2,
            signDisplay: "exceptZero",
          }) + "°",
      )
      .join(" 至 ");
  }, [preview, selectedPoints]);
  const playbackSeconds = stageSeconds ?? browseSeconds;
  const currentPoint = sampleStage(
    selectedPoints,
    selectedTimes,
    previewProgress,
  )?.from;
  const currentStageLabel = !currentPoint
    ? "无路径"
    : currentPoint.space === "cartesian"
      ? `末端 ${"XYZ"[currentPoint.axis]}`
      : currentPoint.space === "joint_return"
        ? "回到原位"
        : `关节 J${currentPoint.axis + 1}`;
  const roundSeconds = useMemo(() => {
    if (!preview) return undefined;
    const seconds = preview.cartesian_duration_s;
    if (seconds !== undefined && Number.isFinite(seconds) && seconds >= 0)
      return seconds;
    return stageTimes(stagePoints(preview, "xyz"))?.at(-1);
  }, [preview]);
  const tcpExtents = useMemo(() => {
    const points = preview?.points.filter(
      (point) => point.space === "cartesian",
    );
    if (!points?.length) return null;
    return [0, 1, 2].map((axis) => {
      const values = points.map(
        (point) => (point.tcp[axis] - points[0].tcp[axis]) * 1000,
      );
      return [Math.min(...values), Math.max(...values)];
    });
  }, [preview]);
  function chooseStage(stage: PreviewStage, data = preview) {
    setPreviewPlaying(false);
    if (!data) {
      setPreviewStage(stage);
      return;
    }
    const candidates = [stage, ...previewStages.slice(1), "xyz" as const];
    const visibleStage =
      candidates.find((candidate) => {
        const points = stagePoints(data, candidate);
        const first = points[0];
        return (
          first &&
          points.some(
            (point) =>
              point.tcp.some(
                (value, axis) => Math.abs(value - first.tcp[axis]) > 1e-9,
              ) ||
              point.joints.some(
                (value, axis) => Math.abs(value - first.joints[axis]) > 1e-9,
              ),
          )
        );
      }) ??
      candidates.find((candidate) => stagePoints(data, candidate).length) ??
      stage;
    setPreviewStage(visibleStage);
    const points = stagePoints(data, visibleStage);
    // Start paused at a distinct real pose, instead of covering the measured arm.
    let selected = 0,
      displacement = -1;
    points.forEach((point, index) => {
      const delta = point.joints.reduce(
        (sum, q, axis) => sum + (q - data.program.origin[axis]) ** 2,
        0,
      );
      if (delta > displacement) {
        selected = index;
        displacement = delta;
      }
    });
    const times = stageTimes(points);
    setPreviewProgress(
      times
        ? times[selected] / times.at(-1)!
        : points.length > 1
          ? selected / (points.length - 1)
          : 0,
    );
  }
  const [pending, setPendingState] = useState<Pending | null>(null);
  const pendingRef = useRef<Pending | null>(null);
  const [run, setRunState] = useState<Run | null>(null);
  const runRef = useRef<Run | null>(null);
  const [liveReport, setLiveReport] = useState<CalibrationReport | null>(null);
  const [livePoints, setLivePoints] = useState<{
    id: string;
    points: CalibrationPoint[];
  }>({ id: "", points: [] });
  const [feedback, setFeedback] = useState<Feedback | null>(null);
  const [stopPending, setStopPending] = useState<{
    seq: number;
    id: string | null;
  } | null>(null);
  const [history, setHistory] = useState<CalibrationReport[]>([]);
  const [historyBusy, setHistoryBusy] = useState(false);
  const [historyError, setHistoryError] = useState("");
  const [reload, setReload] = useState(0);
  const [selection, setSelection] = useState<Selection | null>(null);
  const [details, setDetails] = useState<Record<string, CalibrationReport>>({});
  const [localReports, setLocalReports] = useState<
    { report: CalibrationReport; blob: Blob }[]
  >([]);
  const [detailBusy, setDetailBusy] = useState(false);
  const [detailError, setDetailError] = useState("");
  const [downloading, setDownloading] = useState(false);
  const fileInput = useRef<HTMLInputElement>(null);
  const dialog = useRef<HTMLElement>(null);
  const lastFocus = useRef<HTMLElement | null>(null);
  const alive = useRef(true);
  const previousConnection = useRef({ connection, online: view.online });
  const lastTerminal = useRef("");
  const latest = useRef({ connection, open, onClose, stop: () => {} });
  latest.current = { connection, open, onClose, stop };
  const active =
    s.action?.mode === "calibration" || (!!status && !terminal(status));
  const waiting = run?.connection === connection && run.state !== "terminal";
  const busy = active || !!s.action || !!pending || !!stopPending || waiting;
  const shownHistory =
    liveReport && !history.some((report) => report.id === liveReport.id)
      ? [liveReport, ...history]
      : history;
  const local = selection?.local ?? false;
  const selectedSummary = selection
    ? local
      ? localReports.find((item) => item.report.id === selection.id)?.report
      : shownHistory.find((report) => report.id === selection.id)
    : liveReport;
  const report = selectedSummary
    ? local
      ? selectedSummary
      : (details[selectedSummary.id] ?? selectedSummary)
    : null;
  const reportId = report?.id ?? null;
  const recommendedApplied =
    !!report?.recommended &&
    sameServoTuning(s.servo_tuning, report.recommended);
  const completedKey =
    status && terminal(status)
      ? `${status.id}:${status.state}:${status.report?.id ?? ""}`
      : "";

  function setPending(value: Pending | null) {
    pendingRef.current = value;
    setPendingState(value);
  }
  function setRun(value: Run | null) {
    runRef.current = value;
    setRunState(value);
  }

  useEffect(() => {
    alive.current = true;
    return () => {
      alive.current = false;
      previewRequest.current?.abort();
    };
  }, []);
  useEffect(() => {
    previewRequest.current?.abort();
    setGenerated(null);
    setEnvelope(null);
    setPreviewBusy(false);
    setPreviewPlaying(false);
    setPreviewProgress(0);
  }, [previewContext, previewSettings]);
  useEffect(() => {
    if (open) return;
    previewRequest.current?.abort();
    setPreviewBusy(false);
    setPreviewPlaying(false);
  }, [open]);
  useEffect(() => {
    if (!previewPlaying || !preview || !open || page !== "settings") return;
    let frame = 0;
    const duration = playbackSeconds * 1000;
    const start = performance.now() - previewProgress * duration;
    const animate = (time: number) => {
      const progress = Math.min(1, Math.max(0, (time - start) / duration));
      setPreviewProgress(progress);
      if (progress < 1) frame = requestAnimationFrame(animate);
      else setPreviewPlaying(false);
    };
    frame = requestAnimationFrame(animate);
    return () => cancelAnimationFrame(frame);
  }, [previewPlaying, preview, open, page, playbackSeconds]);
  useEffect(() => {
    const before = previousConnection.current;
    previousConnection.current = { connection, online: view.online };
    if (before.connection === connection && !(before.online && !view.online))
      return;
    const request = pendingRef.current;
    setPending(null);
    setRun(null);
    setStopPending(null);
    if (request) setFeedback({ text: "连接已变化，请求未确认", tone: "info" });
  }, [connection, view.online]);
  useEffect(() => {
    const point = status?.live;
    if (!point || !status || terminal(status)) return;
    setLivePoints((before) => {
      if (before.id !== status.id) return { id: status.id, points: [point] };
      if (before.points.some((item) => item.t_ms === point.t_ms)) return before;
      return { id: before.id, points: [...before.points.slice(-4999), point] };
    });
  }, [status?.id, status?.live]);

  // Observe reports even after Stop, Close, or a lost acknowledgement.
  useEffect(() => {
    if (status?.report) {
      setLiveReport((before) =>
        before?.id === status.report!.id ? before : status.report!,
      );
      setHistory((before) =>
        before.some((item) => item.id === status.report!.id)
          ? before
          : [status.report!, ...before],
      );
    }
    if (!status) return;
    if (!terminal(status) && view.online) {
      const before = runRef.current;
      if (
        !before ||
        before.id !== status.request_id ||
        before.state === "waiting"
      )
        setRun({ id: status.request_id, connection, state: "running" });
    }
    if (terminal(status)) {
      if (
        runRef.current?.id === status.request_id &&
        runRef.current.state !== "terminal"
      )
        setRun({ ...runRef.current, state: "terminal" });
      if (lastTerminal.current !== completedKey) {
        lastTerminal.current = completedKey;
        setSelection(null);
        setPage("result");
      }
    }
  }, [status, connection, completedKey, view.online]);

  useEffect(() => {
    const request = pendingRef.current;
    if (!request || request.connection !== connection || !view.online) return;
    const result =
      s.results?.find((item) => item.request_id === request.id) ??
      (s.last_result?.request_id === request.id ? s.last_result : null);
    const matching =
      request.kind === "start" && status?.request_id === request.id;
    if (matching) {
      setPending(null);
      setRun({
        id: request.id,
        connection,
        state: terminal(status) ? "terminal" : "running",
      });
      setFeedback(null);
      setPage(terminal(status) ? "result" : "progress");
      return;
    }
    if (result && ["rejected", "failed", "cancelled"].includes(result.status)) {
      setPending(null);
      if (request.kind === "start") {
        if (result.code === "stale_preview") setGenerated(null);
        setRun(
          result.status === "rejected"
            ? null
            : { id: request.id, connection, state: "terminal" },
        );
        setPage(result.status === "rejected" ? "settings" : "result");
      }
      setFeedback({
        text: result.message || "请求未完成",
        tone: result.status === "failed" ? "error" : "info",
      });
      return;
    }
    const accepted =
      request.accepted ||
      (!!result && ["accepted", "completed"].includes(result.status));
    if (request.kind === "start" && accepted) {
      setPending(null);
      setRun({
        id: request.id,
        connection,
        state: result?.status === "completed" ? "terminal" : "running",
      });
      setFeedback(null);
      setPage(result?.status === "completed" ? "result" : "progress");
      return;
    }
    if (
      request.kind !== "start" &&
      accepted &&
      sameServoTuning(s.servo_tuning, request.config)
    ) {
      setPending(null);
      setFeedback({
        text: request.kind === "restore" ? "起始参数已恢复" : "推荐参数已应用",
        tone: "success",
      });
      return;
    }
    if (
      !accepted &&
      (!view.hasControl || s.generation !== request.generation)
    ) {
      setPending(null);
      if (request.kind === "start") setRun(null);
      setFeedback({ text: "控制状态已变化，请求未确认", tone: "info" });
    } else if (accepted && !request.accepted)
      setPending({ ...request, accepted: true });
  }, [view, pending, connection]);

  useEffect(() => {
    if (!pending || pending.uncertain) return;
    const timer = setTimeout(() => {
      const request = pendingRef.current;
      if (!request || request.id !== pending.id) return;
      setPending({ ...request, uncertain: true });
      setFeedback({ text: "请求尚未确认，继续等待状态或停止", tone: "info" });
    }, 8000);
    return () => clearTimeout(timer);
  }, [pending?.id, pending?.uncertain]);
  useEffect(() => {
    if (
      stopPending &&
      view.online &&
      s.seq > stopPending.seq &&
      !s.action &&
      ["ready", "disabled", "fault", "disconnected"].includes(s.phase) &&
      (!status || terminal(status) || status.request_id !== stopPending.id)
    ) {
      setStopPending(null);
      if (runRef.current && runRef.current.state !== "terminal")
        setRun({ ...runRef.current, state: "terminal" });
    }
  }, [stopPending, view]);
  useEffect(() => {
    if (open && (active || runRef.current?.state === "waiting"))
      setPage("progress");
  }, [open]);

  useEffect(() => {
    if (!open || !view.online) {
      setHistoryBusy(false);
      return;
    }
    const controller = new AbortController();
    setHistoryBusy(true);
    setHistoryError("");
    void api
      .listCalibrations(20, controller.signal)
      .then((reports) => {
        if (
          controller.signal.aborted ||
          latest.current.connection !== connection
        )
          return;
        setHistory((before) => [
          ...reports,
          ...before.filter(
            (item) => !reports.some((report) => report.id === item.id),
          ),
        ]);
      })
      .catch((error) => {
        if (
          !controller.signal.aborted &&
          latest.current.connection === connection
        )
          setHistoryError(
            error instanceof Error ? error.message : "读取历史失败",
          );
      })
      .finally(() => {
        if (!controller.signal.aborted) setHistoryBusy(false);
      });
    return () => controller.abort();
  }, [open, view.online, connection, reload, completedKey]);
  useEffect(() => {
    if (
      !open ||
      !view.online ||
      !reportId ||
      local ||
      !["result", "curves"].includes(page)
    ) {
      setDetailBusy(false);
      return;
    }
    const controller = new AbortController();
    setDetailBusy(true);
    setDetailError("");
    void api
      .readCalibration(reportId, controller.signal)
      .then((detail) => {
        if (
          !controller.signal.aborted &&
          latest.current.connection === connection
        )
          setDetails((before) => ({ ...before, [detail.id]: detail }));
      })
      .catch((error) => {
        if (
          !controller.signal.aborted &&
          latest.current.connection === connection
        )
          setDetailError(
            error instanceof Error ? error.message : "读取报告失败",
          );
      })
      .finally(() => {
        if (!controller.signal.aborted) setDetailBusy(false);
      });
    return () => controller.abort();
  }, [
    open,
    view.online,
    connection,
    reportId,
    local,
    reload,
    page === "result" || page === "curves",
  ]);

  useEffect(() => {
    if (!open) return;
    const previous = document.activeElement;
    const frame = requestAnimationFrame(() => {
      if (
        lastFocus.current?.isConnected &&
        lastFocus.current.getClientRects().length &&
        !lastFocus.current.closest("details:not([open])")
      )
        lastFocus.current.focus();
      else dialog.current?.focus();
    });
    return () => {
      cancelAnimationFrame(frame);
      if (
        previous instanceof HTMLElement &&
        previous.isConnected &&
        previous.getClientRects().length
      )
        previous.focus();
    };
  }, [open]);
  useEffect(() => {
    let escapeHeld = false;
    const keydown = (event: KeyboardEvent) => {
      if (event.key === "Escape" && (latest.current.open || escapeHeld)) {
        event.preventDefault();
        event.stopImmediatePropagation();
        if (latest.current.open && !escapeHeld && !event.repeat) {
          escapeHeld = true;
          latest.current.stop();
          latest.current.onClose();
        }
        return;
      }
      if (!latest.current.open || event.key !== "Tab") return;
      const controls = Array.from(
        dialog.current?.querySelectorAll<HTMLElement>(
          'button:not(:disabled), input:not(:disabled), select:not(:disabled), summary, a[href], [tabindex="0"]',
        ) ?? [],
      ).filter(
        (element) =>
          element.getClientRects().length &&
          (element.tagName === "SUMMARY" ||
            !element.closest("details:not([open])")),
      );
      const first = controls[0],
        last = controls.at(-1);
      if (
        !controls.includes(document.activeElement as HTMLElement) ||
        (event.shiftKey && document.activeElement === first) ||
        (!event.shiftKey && document.activeElement === last)
      ) {
        event.preventDefault();
        (event.shiftKey ? last : first)?.focus();
      }
    };
    const keyup = (event: KeyboardEvent) => {
      if (event.key === "Escape") escapeHeld = false;
    };
    document.addEventListener("keydown", keydown, true);
    document.addEventListener("keyup", keyup, true);
    return () => {
      document.removeEventListener("keydown", keydown, true);
      document.removeEventListener("keyup", keyup, true);
    };
  }, []);

  const idleReason =
    !view.online || ["disconnected", "connecting", "closing"].includes(s.phase)
      ? "请先连接机械臂"
      : !view.hasControl
        ? "请先取得控制权"
        : s.phase === "fault"
          ? s.error?.message || "请先处理故障"
          : !["ready", "disabled"].includes(s.phase) || s.action
            ? "请先停止当前动作"
            : "";
  const startReason =
    idleReason ||
    (!s.enabled ? "请先使能机械臂" : "") ||
    (busy ? "等待当前请求完成" : "") ||
    jointSpeedsError ||
    (!frictionBudgetValid ? "补偿幅值须为非负数或留空" : "") ||
    (previewBusy || !preview ? "请先生成并检查运动预览" : "");
  const previewReason =
    !view.online || ["disconnected", "connecting", "closing"].includes(s.phase)
      ? "请先连接机械臂"
      : busy
        ? "等待当前动作完成"
        : !amplitudeValid
          ? "请输入大于 0 的单侧行程"
          : !speedValid
            ? "请输入大于 0 的末端速度"
            : !motion
              ? "运动偏好须为正数"
              : jointSpeedsError;
  // The current report and servo config are sufficient; TCP/pose changes do not invalidate tuning.
  const applyReason =
    local || reportId !== status?.report?.id
      ? "历史报告仅供查看"
      : !report?.recommended
        ? "本次参数保持原样"
        : idleReason ||
          (busy ? "等待当前请求完成" : "") ||
          (recommendedApplied ? "推荐参数已应用" : "");
  const restoreReason =
    local || reportId !== status?.report?.id
      ? "历史报告仅供查看"
      : idleReason ||
        (busy ? "等待当前请求完成" : "") ||
        (report && sameServoTuning(s.servo_tuning, report.baseline)
          ? "已是起始参数"
          : "");
  function stop() {
    const id =
      status?.request_id ??
      runRef.current?.id ??
      pendingRef.current?.id ??
      null;
    setPending(null);
    previewRequest.current?.abort();
    setGenerated(null);
    setPreviewBusy(false);
    setPreviewPlaying(false);
    setStopPending({ seq: s.seq, id });
    setFeedback({ text: "已请求停止", tone: "info" });
    onStop();
  }
  async function generatePreview() {
    if (!open || previewReason || previewBusy) return;
    previewRequest.current?.abort();
    const controller = new AbortController();
    previewRequest.current = controller;
    setGenerated(null);
    setEnvelope(null);
    setPreviewProgress(0);
    setPreviewPlaying(false);
    setPreviewBusy(true);
    setFeedback(null);
    try {
      const data = await api.previewCalibration(
        {
          amplitude: amplitude == null ? null : amplitude / 1000,
          speed: speed / 1000,
          joint_speeds: jointSpeeds,
          axes: [0, 1, 2],
          motion: motion!,
        },
        controller.signal,
      );
      if (
        controller.signal.aborted ||
        !alive.current ||
        previewContextRef.current !== previewContext
      )
        return;
      setGenerated({
        data,
        context: previewContext,
        settings: previewSettings,
      });
      chooseStage("cartesian:0", data);
    } catch (error) {
      if (!controller.signal.aborted && alive.current) {
        let reason = error instanceof Error ? error.message : "请重试";
        const planningFailed =
          error instanceof ApiError &&
          (error.code === "preview_failed" ||
            (!error.code && error.status === 422));
        if (planningFailed && !/调整|缩短|降低|请先|请重新/.test(reason))
          reason += "。可在高级设置中缩短单侧行程或降低末端速度后重试";
        setFeedback({
          text: `生成预览失败。${reason}`,
          tone: "error",
        });
      }
    } finally {
      if (!controller.signal.aborted && alive.current) setPreviewBusy(false);
    }
  }
  function send(kind: Pending["kind"]) {
    if (
      !open ||
      pendingRef.current ||
      (kind === "start"
        ? startReason
        : kind === "restore"
          ? restoreReason
          : applyReason)
    )
      return;
    const id = crypto.randomUUID();
    const request: Pending = {
      kind,
      id,
      connection,
      generation: s.generation,
      config: structuredClone(
        kind === "apply"
          ? report!.recommended!
          : kind === "restore"
            ? report!.baseline
            : s.servo_tuning,
      ),
      accepted: false,
      uncertain: false,
    };
    setPending(request);
    setFeedback(null);
    if (kind === "start") {
      setRun({ id, connection, state: "waiting" });
      setSelection(null);
      setPage("progress");
    }
    void api
      .command(
        kind === "start"
          ? "calibrate_start"
          : kind === "restore"
            ? "configure_servo"
            : "calibrate_apply",
        kind === "start"
          ? {
              settings: {
                cartesian: preview!.program,
                speeds: jointSpeeds,
                max_friction: frictionBudget,
              },
            }
          : kind === "restore"
            ? { config: request.config }
            : { calibration_id: reportId },
        id,
        s.generation,
      )
      .then((ack) => {
        if (
          !alive.current ||
          pendingRef.current?.id !== id ||
          latest.current.connection !== connection
        )
          return;
        if (ack.request_id !== id || ack.status !== "queued") {
          setPending({ ...pendingRef.current, uncertain: true });
          setFeedback({ text: "请求尚未确认，正在等待状态", tone: "info" });
        }
      })
      .catch((error) => {
        if (
          !alive.current ||
          pendingRef.current?.id !== id ||
          latest.current.connection !== connection
        )
          return;
        if (
          error instanceof ApiError &&
          error.notQueued &&
          !pendingRef.current.accepted
        ) {
          setPending(null);
          if (kind === "start") {
            if (error.code === "stale_preview") setGenerated(null);
            setRun(null);
            setPage("settings");
          }
          setFeedback({ text: error.message, tone: "info" });
        } else {
          setPending({ ...pendingRef.current, uncertain: true });
          setFeedback({
            text: `${kind === "start" ? "启动" : kind === "restore" ? "恢复" : "应用"}未确认：${error instanceof Error ? error.message : "请求失败"}`,
            tone: "info",
          });
        }
      });
  }
  async function importReport(event: ChangeEvent<HTMLInputElement>) {
    const file = event.target.files?.[0];
    event.target.value = "";
    if (!file) return;
    try {
      const imported = parseCalibrationReport(JSON.parse(await file.text()));
      if (!alive.current) return;
      setLocalReports((before) => [
        { report: imported, blob: file },
        ...before.filter((item) => item.report.id !== imported.id),
      ]);
      setSelection({ id: imported.id, local: true });
      setFeedback(null);
      setPage("result");
    } catch (error) {
      setFeedback({
        text: error instanceof Error ? error.message : "报告读取失败",
        tone: "error",
      });
    }
  }
  async function download() {
    if (!report || downloading) return;
    setDownloading(true);
    try {
      const blob = local
        ? localReports.find((item) => item.report.id === report.id)!.blob
        : await api.downloadCalibration(report.id);
      const url = URL.createObjectURL(blob),
        link = document.createElement("a");
      link.href = url;
      link.download = `calibration-${report.id.replace(/[^a-zA-Z0-9_-]/g, "_")}.json`;
      document.body.append(link);
      link.click();
      link.remove();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
    } catch (error) {
      if (alive.current)
        setFeedback({
          text: error instanceof Error ? error.message : "下载失败",
          tone: "error",
        });
    } finally {
      if (alive.current) setDownloading(false);
    }
  }

  const progress = percent(
    status?.progress ??
      (s.action?.mode === "calibration" ? s.action.progress : 0),
  );
  const progressTitle =
    stopPending || status?.state === "stopping"
      ? "正在停止并恢复参数"
      : pending?.kind === "start"
        ? pending.uncertain
          ? "等待启动确认"
          : "等待测试开始"
        : active && status
          ? stages[status.stage]
          : waiting
            ? "等待测试状态"
            : status
              ? stages[status.stage]
              : "尚未开始";
  const axis =
    status?.axis ?? (s.action?.mode === "calibration" ? s.action.axis : 0);
  const footer =
    feedback?.text ||
    (page === "settings"
      ? preview
        ? startReason || "测试完成后恢复起始参数"
        : previewReason || "预览路径后确认开始"
      : page === "result" && report?.recommended
        ? applyReason || "点击“应用推荐”采用本次建议。"
        : "");
  const historyEntries = [
    ...localReports.map((item) => ({ report: item.report, local: true })),
    ...shownHistory.map((item) => ({ report: item, local: false })),
  ];
  return (
    <div
      className="calibration-backdrop"
      hidden={!open}
      onClick={(event) => {
        if (event.target === event.currentTarget) onClose();
      }}
    >
      <section
        ref={dialog}
        className="calibration-panel"
        data-page={page}
        id="calibration-dialog"
        role="dialog"
        aria-modal="true"
        aria-labelledby="calibration-title"
        tabIndex={-1}
        onFocusCapture={(event) => {
          if (event.target instanceof HTMLElement)
            lastFocus.current = event.target;
        }}
      >
        <header className="calibration-header">
          <h2 id="calibration-title">低速校准</h2>
          <span>
            {s.side === "left" ? "左臂" : "右臂"} ·{" "}
            {view.online ? s.port || "已连接" : "未连接"}
          </span>
          <button className="calibration-stop" onClick={stop}>
            停止 <kbd>Esc</kbd>
          </button>
          <button
            className="calibration-close"
            aria-label="关闭低速校准"
            onClick={onClose}
          >
            ×
          </button>
        </header>
        <nav className="calibration-tabs" aria-label="校准页面">
          {(
            [
              ["settings", "开始"],
              ["progress", "进度"],
              ["result", "结果"],
              ["curves", "曲线"],
              ["history", "历史"],
            ] as const
          ).map(([key, label]) => (
            <button
              key={key}
              aria-pressed={page === key}
              aria-controls={`calibration-${key}`}
              onClick={() => {
                setPage(key);
                setFeedback(null);
              }}
            >
              {label}
              {key === "progress" && (active || waiting) && (
                <i className="calibration-running-dot" />
              )}
            </button>
          ))}
        </nav>
        <div className="calibration-body" id={`calibration-${page}`}>
          {page === "settings" && (
            <div className="calibration-start">
              <div className="calibration-intro">
                <small>XYZ 末端动作 + J1–J7 关节试验</small>
                <h3>一键校准摩擦</h3>
                <p>双向多速度采集，拟合逐轴 Stribeck 曲线。</p>
                <details className="calibration-advanced">
                  <summary>
                    高级设置 <small>行程 / 速度 / 幅值</small>
                  </summary>
                  <div className="calibration-range-settings">
                    <label className="calibration-range-mode">
                      <span>末端行程</span>
                      <select
                        aria-label="末端行程模式"
                        value={autoRange ? "auto" : "manual"}
                        disabled={busy}
                        onChange={(event) =>
                          setAutoRange(event.target.value === "auto")
                        }
                      >
                        <option value="auto">自动可达行程</option>
                        <option value="manual">指定行程</option>
                      </select>
                    </label>
                    {!autoRange && (
                      <label className="calibration-amplitude">
                        <span>单侧行程 · mm</span>
                        <span>
                          <b>±</b>
                          <input
                            type="number"
                            aria-label="末端单侧幅度 mm"
                            min={0}
                            step="any"
                            value={amplitudeText}
                            disabled={busy}
                            aria-invalid={!amplitudeValid}
                            onChange={(event) =>
                              setAmplitudeText(event.target.value)
                            }
                          />
                          <b>mm</b>
                        </span>
                        {!amplitudeValid && <small>输入大于 0 的行程</small>}
                      </label>
                    )}
                    <label className="calibration-speed">
                      <span>
                        末端速度 <small>mm/s</small>
                      </span>
                      <input
                        type="number"
                        aria-label="校准末端速度 mm/s"
                        min={0}
                        step="any"
                        value={speedText}
                        disabled={busy}
                        aria-invalid={!speedValid}
                        onChange={(event) => setSpeedText(event.target.value)}
                      />
                    </label>
                    <label className="calibration-fit-setting">
                      <span>
                        关节速度 <small>°/s · 逗号分隔</small>
                      </span>
                      <input
                        type="text"
                        inputMode="decimal"
                        aria-label="校准关节速度 °/s"
                        placeholder="自动：1, 2, 4"
                        value={jointSpeedsText}
                        disabled={busy}
                        aria-invalid={!jointSpeedsValid}
                        onChange={(event) => {
                          setJointSpeedsText(event.target.value);
                          setFeedback(null);
                        }}
                      />
                    </label>
                    <label className="calibration-fit-setting">
                      <span>
                        补偿幅值 <small>Nm</small>
                      </span>
                      <input
                        type="number"
                        aria-label="校准补偿幅值 Nm"
                        placeholder="自动"
                        title="留空自动；0 不试用摩擦"
                        min={0}
                        step="any"
                        value={frictionBudgetText}
                        disabled={busy}
                        aria-invalid={!frictionBudgetValid}
                        onChange={(event) =>
                          setFrictionBudgetText(event.target.value)
                        }
                      />
                    </label>
                    <details className="motion-preferences calibration-motion">
                      <summary>运动偏好</summary>
                      <MotionTuningFields
                        draft={motionDraft}
                        disabled={busy}
                        onChange={(value) => {
                          setMotionEdited(true);
                          setMotionDraft(value);
                        }}
                      />
                      <small>用于本次预览与执行</small>
                    </details>
                  </div>
                </details>
                <ol className="calibration-plan">
                  <li>
                    <b>末端动作 · 调整前</b>
                    <span>基座 X → Y → Z 往返</span>
                  </li>
                  <li>
                    <b>双向采集 · 拟合与验证</b>
                    <span>
                      单侧最多{" "}
                      {preview
                        ? degrees(preview.program.joint_excursion / 2, 0)
                        : "8"}
                      ° · {speedLabel}
                      °/s
                    </span>
                  </li>
                  <li>
                    <b>同路径 · 调整后</b>
                    <span>末端复测，恢复原位与起始参数</span>
                  </li>
                </ol>
                <p className="calibration-note">
                  Kd / 速度前馈保持原值 ·{" "}
                  {frictionBudget == null
                    ? "自动补偿幅值"
                    : `补偿幅值 ${frictionBudget} Nm`}
                </p>
              </div>
              <div className="calibration-preview-view">
                <div className="calibration-preview-scene">
                  {open && (
                    <RobotScene
                      side={s.side}
                      joints={s.joints}
                      valid={hasJointFeedback(view)}
                      source={
                        hasJointFeedback(view)
                          ? "实测姿态 · 拖动旋转 / 滚轮缩放"
                          : "参考姿态 · 拖动旋转 / 滚轮缩放"
                      }
                      preview={preview}
                      previewProgress={previewProgress}
                      previewStage={previewStage}
                      onPreviewEnvelope={setEnvelope}
                    />
                  )}
                  {!preview && (
                    <div className="calibration-preview-placeholder">
                      <strong>
                        {previewBusy
                          ? "正在计算完整试验范围…"
                          : "查看路径与全臂扫掠范围"}
                      </strong>
                      <span>生成预览后，可拖动进度查看计划姿态</span>
                    </div>
                  )}
                </div>
                {preview ? (
                  <>
                    <p
                      className="calibration-preview-hint"
                      title="仅计一次 XYZ 往返，不含关节试验、分析和重复执行"
                    >
                      XYZ 往返单轮
                      {roundSeconds === undefined
                        ? "用时未知"
                        : `预计 ${roundSeconds.toFixed(1)} 秒`}
                      {jointRange && currentPoint && (
                        <>
                          {" "}
                          · J{currentPoint.axis + 1} 相对原位 {jointRange}
                        </>
                      )}
                    </p>
                    <nav
                      className="calibration-preview-stages"
                      aria-label="预览阶段"
                    >
                      {previewStages.map((stage) => {
                        const axis = Number(stage.split(":")[1]);
                        return (
                          <button
                            key={stage}
                            aria-pressed={previewStage === stage}
                            disabled={!availableStages.includes(stage)}
                            className={
                              stage.startsWith("cartesian:")
                                ? `axis-${"xyz"[axis]}`
                                : ""
                            }
                            onClick={() => chooseStage(stage)}
                          >
                            {stage === "xyz"
                              ? "XYZ"
                              : stage === "return"
                                ? "归位"
                                : stage.startsWith("cartesian:")
                                  ? "XYZ"[axis]
                                  : `J${axis + 1}`}
                          </button>
                        );
                      })}
                    </nav>
                    <div className="calibration-preview-scrub">
                      <button
                        aria-label={previewPlaying ? "暂停预览" : "播放预览"}
                        title={
                          stageSeconds === undefined
                            ? "用 16 秒浏览当前阶段"
                            : "按当前阶段的计划时间播放"
                        }
                        onClick={() => {
                          if (previewProgress >= 1) setPreviewProgress(0);
                          setPreviewPlaying(!previewPlaying);
                        }}
                      >
                        {previewPlaying
                          ? "暂停"
                          : stageSeconds === undefined
                            ? "16 秒浏览"
                            : "按计划播放"}
                      </button>
                      <input
                        type="range"
                        aria-label="计划姿态进度"
                        aria-valuetext={
                          stageSeconds === undefined
                            ? `${currentStageLabel}，${Math.round(previewProgress * 100)}%`
                            : `${currentStageLabel}，${(previewProgress * stageSeconds).toFixed(1)} 秒，共 ${stageSeconds.toFixed(1)} 秒`
                        }
                        min={0}
                        max={1}
                        step={stageSeconds === undefined ? 0.001 : "any"}
                        value={previewProgress}
                        onChange={(event) => {
                          setPreviewPlaying(false);
                          setPreviewProgress(Number(event.target.value));
                        }}
                      />
                      <output style={{ width: "auto" }} aria-live="off">
                        {currentStageLabel}
                        {stageSeconds === undefined
                          ? previewPlaying
                            ? " · 16 秒浏览"
                            : ""
                          : ` · ${(previewProgress * stageSeconds).toFixed(1)} / ${stageSeconds.toFixed(1)} s`}
                      </output>
                    </div>
                    <div className="calibration-preview-extents">
                      <div>
                        <b>
                          {preview.ranges?.length
                            ? "模型可达行程"
                            : "TCP 实际行程"}{" "}
                          <small>相对原位 · mm</small>
                        </b>
                        <div>
                          {(preview.ranges?.length
                            ? preview.ranges
                            : tcpExtents?.map(([negative, positive], axis) => ({
                                axis,
                                negative: negative / 1000,
                                positive: positive / 1000,
                              }))
                          )?.map(({ axis, negative, positive }) => (
                            <span key={axis}>
                              <i className={`axis-${"xyz"[axis]}`}>
                                {"XYZ"[axis]}
                              </i>
                              {(negative * 1000).toFixed(1)} →{" "}
                              {(positive * 1000).toFixed(1)}
                            </span>
                          ))}
                        </div>
                      </div>
                      <details>
                        <summary>全臂范围 · mm</summary>
                        <small>
                          {envelope
                            ? envelope.min
                                .map(
                                  (value, axis) =>
                                    `${"XYZ"[axis]} ${(value * 1000).toFixed(0)}…${(envelope.max[axis] * 1000).toFixed(0)}`,
                                )
                                .join(" · ")
                            : "计算中…"}
                        </small>
                      </details>
                    </div>
                  </>
                ) : (
                  <p className="calibration-preview-hint">
                    {autoRange
                      ? "按当前姿态计算 XYZ 可达行程"
                      : `末端单侧 ${amplitudeValid ? amplitude : "—"} mm`}{" "}
                    · 含七轴试验
                  </p>
                )}
              </div>
            </div>
          )}
          {page === "progress" && (
            <div className="calibration-progress-view">
              <div className="calibration-progress-heading">
                <div>
                  <small>
                    {active
                      ? status?.space === "cartesian"
                        ? `当前末端 ${"XYZ"[axis] ?? ""}`
                        : `当前 J${axis + 1}`
                      : "末端与七轴测试"}
                  </small>
                  <h3>{progressTitle}</h3>
                </div>
                <strong>{progress}%</strong>
              </div>
              <progress
                className="calibration-progress"
                max={100}
                value={progress}
                aria-label="校准进度"
              />
              <ol className="calibration-stages">
                {(
                  [
                    "cartesian_before",
                    "baseline",
                    "identifying",
                    "candidate",
                    "cartesian_after",
                    "restoring",
                  ] as const
                ).map((stage) => (
                  <li
                    key={stage}
                    data-active={
                      status?.stage === stage ||
                      (stage === "cartesian_after" &&
                        ["validation", "after"].includes(status?.stage ?? ""))
                    }
                  >
                    {stages[stage]}
                  </li>
                ))}
              </ol>
              <dl className="calibration-progress-facts">
                <div>
                  <dt>
                    {status?.space === "cartesian" ? "末端运动轴" : "当前关节"}
                  </dt>
                  <dd>
                    {status?.space === "cartesian"
                      ? `${"XYZ"[axis] ?? "—"} · 基座坐标`
                      : `J${axis + 1} · ${degrees(s.joints[axis]?.q, 2)}°`}
                  </dd>
                </div>
                <div>
                  <dt>试验轮次</dt>
                  <dd>
                    {status
                      ? `${status.trial_index} / ${status.trial_count}`
                      : "—"}
                  </dd>
                </div>
                <div>
                  <dt>运动片段</dt>
                  <dd>
                    {status ? `${status.leg_index} / ${status.leg_count}` : "—"}
                  </dd>
                </div>
              </dl>
              <p className="calibration-note">
                {!view.online
                  ? "连接已断开"
                  : "完成后回原位并恢复起始参数，结果与曲线会保留。"}
              </p>
              <button onClick={() => setPage("curves")}>查看实时曲线</button>
              {liveReport && (
                <button
                  onClick={() => {
                    setSelection(null);
                    setPage("result");
                  }}
                >
                  查看结果
                </button>
              )}
            </div>
          )}
          {page === "curves" && active && status
            ? open && (
                <CalibrationChart
                  key={status.id}
                  live={{
                    id: status.id,
                    axis: status.axis,
                    space: status.space,
                    points:
                      livePoints.id === status.id ? livePoints.points : [],
                  }}
                />
              )
            : (page === "result" || page === "curves") && (
                <>
                  {detailError && !local && (
                    <p className="calibration-error" role="alert">
                      {detailError}{" "}
                      <button onClick={() => setReload((before) => before + 1)}>
                        重试
                      </button>
                    </p>
                  )}
                  {detailBusy && (
                    <p className="calibration-loading" role="status">
                      正在读取曲线…
                    </p>
                  )}
                  {report ? (
                    page === "result" ? (
                      <ReportDetails
                        key={`${local}:${report.id}`}
                        report={report}
                        local={local}
                        applied={recommendedApplied}
                        restored={sameServoTuning(
                          s.servo_tuning,
                          report.baseline,
                        )}
                        onCurves={() => setPage("curves")}
                      />
                    ) : (
                      <CalibrationChart
                        key={`${local}:${report.id}`}
                        report={report}
                      />
                    )
                  ) : (
                    <div className="calibration-empty">
                      <h3>
                        {active || waiting || run?.state === "terminal"
                          ? "等待测试报告"
                          : "尚无测试报告"}
                      </h3>
                      <p>完成一次测试，或从历史导入已有报告。</p>
                    </div>
                  )}
                </>
              )}
          {page === "history" && (
            <div className="calibration-history">
              <div className="calibration-history-toolbar">
                <span>{historyEntries.length} 份报告</span>
                <small>{historyBusy ? "正在读取…" : "导入报告仅供查看"}</small>
                <button onClick={() => fileInput.current?.click()}>
                  导入报告
                </button>
                <button
                  disabled={!view.online || historyBusy}
                  onClick={() => setReload((before) => before + 1)}
                >
                  刷新
                </button>
              </div>
              {historyError && (
                <p className="calibration-error" role="alert">
                  {historyError}
                </p>
              )}
              {historyEntries.length ? (
                <div className="calibration-history-list">
                  {historyEntries.map(({ report: item, local: isLocal }) => (
                    <button
                      className="calibration-history-item"
                      key={`${isLocal}:${item.id}`}
                      onClick={() => {
                        setSelection({ id: item.id, local: isLocal });
                        setFeedback(null);
                        setPage("result");
                      }}
                    >
                      <span>
                        <b>{axisLabel(item)}</b>
                        <small>
                          {timestamp(item.created_at_ms)}
                          {isLocal
                            ? " · 导入"
                            : ` · ${item.side === "left" ? "左臂" : "右臂"}`}
                        </small>
                      </span>
                      <span>{outcome(item)}</span>
                      <span aria-hidden="true">›</span>
                    </button>
                  ))}
                </div>
              ) : (
                <p className="calibration-empty">
                  {historyBusy
                    ? "正在读取校准记录…"
                    : "暂无校准记录，可导入已有 JSON 报告。"}
                </p>
              )}
            </div>
          )}
        </div>
        <input
          ref={fileInput}
          type="file"
          accept="application/json,.json"
          aria-label="导入校准报告"
          hidden
          onChange={(event) => void importReport(event)}
        />
        <footer className="calibration-footer">
          <span
            id="calibration-feedback"
            data-tone={feedback?.tone ?? "info"}
            role={feedback?.tone === "error" ? "alert" : "status"}
          >
            {footer}
          </span>
          {page === "settings" && (
            <>
              <button
                className={preview ? "" : "primary"}
                disabled={!!previewReason || previewBusy}
                aria-busy={previewBusy}
                onClick={() => void generatePreview()}
              >
                {previewBusy
                  ? "正在生成…"
                  : preview
                    ? "重新生成预览"
                    : "生成预览"}
              </button>
              {preview && (
                <button
                  className="primary"
                  disabled={!!startReason}
                  title={startReason || "按当前完整预览执行"}
                  aria-busy={pending?.kind === "start"}
                  onClick={() => send("start")}
                >
                  确认并开始
                </button>
              )}
            </>
          )}
          {(page === "result" || page === "curves") && report && (
            <button
              disabled={downloading || (!local && !view.online)}
              onClick={() => void download()}
            >
              {downloading ? "正在下载…" : "下载完整 JSON"}
            </button>
          )}
          {(page === "result" || page === "curves") &&
            report?.recommended &&
            !local &&
            reportId === status?.report?.id && (
              <>
                <button
                  disabled={!!restoreReason}
                  title={restoreReason || "恢复本次校准起始参数"}
                  aria-busy={pending?.kind === "restore"}
                  onClick={() => send("restore")}
                >
                  {pending?.kind === "restore" ? "等待确认…" : "恢复起始"}
                </button>
                <button
                  className="primary"
                  disabled={!!applyReason}
                  title={applyReason || "应用推荐参数"}
                  aria-busy={pending?.kind === "apply"}
                  onClick={() => send("apply")}
                >
                  {pending?.kind === "apply"
                    ? "等待确认…"
                    : recommendedApplied
                      ? "已应用"
                      : "应用推荐"}
                </button>
              </>
            )}
          <button onClick={onClose}>关闭</button>
        </footer>
      </section>
    </div>
  );
}
