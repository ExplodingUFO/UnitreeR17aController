import { useEffect, useRef, useState } from "react";
import {
  api,
  ApiError,
  hasFreshFeedback,
  sameServoTuning,
  type CalibrationReport,
  type CalibrationSettings,
  type CalibrationStatus,
  type ServoTuning,
  type View,
} from "./api";

export interface CalibrationPanelProps {
  view: View;
  open: boolean;
  onClose: () => void;
  onStop: () => void;
}

type Page = "settings" | "progress" | "result" | "history";
type Draft = {
  axis: number;
  excursion: string;
  speed: string;
  friction: string;
};
type Pending = {
  kind: "start" | "apply";
  id: string;
  connection: string;
  generation: number;
  revision: number;
  config: ServoTuning;
  tcp: string;
  timedOut: boolean;
  accepted: boolean;
};
type Run = {
  id: string;
  connection: string;
  state: "waiting" | "running" | "terminal";
};
type Eligibility = {
  id: string;
  connection: string;
  revision: number;
  tcp: string;
};
type Feedback = {
  text: string;
  tone: "error" | "info" | "success";
  code?: string;
};
const RAD = Math.PI / 180;
const stages: Record<CalibrationStatus["stage"], string> = {
  baseline: "测量基线",
  candidate: "比较候选参数",
  baseline_check: "复测基线",
  validation: "验证推荐参数",
  restoring: "恢复起始参数",
  done: "已结束",
};
const outcomes: Record<CalibrationReport["outcome"], string> = {
  improved: "低速表现改善",
  unchanged: "保持原参数",
  insufficient: "有效数据不足",
  cancelled: "校准已取消",
  failed: "校准未完成",
};
const terminal = (status: CalibrationStatus) =>
  ["completed", "cancelled", "failed"].includes(status.state);
const degrees = (value: number | null | undefined, digits = 2) =>
  value != null && Number.isFinite(value) ? (value / RAD).toFixed(digits) : "—";
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
function readSettings(draft: Draft): CalibrationSettings | null {
  const excursion = Number(draft.excursion);
  const speed = Number(draft.speed) * RAD;
  const friction = Number(draft.friction);
  if (
    !Number.isInteger(draft.axis) ||
    draft.axis < 0 ||
    draft.axis > 6 ||
    !draft.excursion.trim() ||
    !Number.isFinite(excursion) ||
    excursion < 1 ||
    excursion > 10 ||
    !draft.speed.trim() ||
    !Number.isFinite(speed) ||
    speed < 0.005 ||
    speed > 0.04 ||
    !draft.friction.trim() ||
    !Number.isFinite(friction) ||
    friction < 0 ||
    friction > 2
  )
    return null;
  return {
    axis: draft.axis,
    excursion: excursion * RAD,
    speed,
    max_friction: friction,
  };
}
function downloadReport(report: CalibrationReport) {
  const url = URL.createObjectURL(
    new Blob([JSON.stringify(report, null, 2)], { type: "application/json" }),
  );
  const link = document.createElement("a");
  link.href = url;
  link.download = `calibration-${report.id.replace(/[^a-zA-Z0-9_-]/g, "_")}.json`;
  document.body.append(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
function ReportDetails({ report }: { report: CalibrationReport }) {
  const [details, setDetails] = useState<"config" | "metrics">("config");
  const axis = report.settings.axis;
  const baseline = report.baseline.joints[axis];
  const recommended = report.recommended?.joints[axis];
  const fields = [
    ["kd", "阻尼 Kd", "Nm·s/rad"],
    ["velocity_ff", "速度前馈", ""],
    ["coulomb", "库仑补偿 Fc", "Nm"],
    ["viscous", "黏性补偿 B", "Nm·s/rad"],
    ["velocity_scale", "过零速度", "rad/s"],
    ["max_friction", "补偿限幅", "Nm"],
  ] as const;
  return (
    <div className="calibration-report">
      <div className="calibration-result-heading">
        <div>
          <small>
            J{axis + 1} · {timestamp(report.created_at_ms)}
          </small>
          <h3>{outcomes[report.outcome]}</h3>
        </div>
        {report.improvement !== null && Number.isFinite(report.improvement) && (
          <strong className="calibration-improvement">
            {(report.improvement * 100).toFixed(1)}%<small>改善</small>
          </strong>
        )}
      </div>
      <p className="calibration-report-message">{report.message}</p>
      <div className="calibration-report-context">
        <span>参考中心 {degrees(report.center_reference[axis])}°</span>
        <span>实测中心 {degrees(report.center_measured[axis])}°</span>
        <span>最大位移 ±{degrees(report.settings.excursion, 1)}°</span>
        <span>{degrees(report.settings.speed)}°/s</span>
      </div>
      <div className="calibration-detail-tabs" aria-label="报告内容">
        <button
          aria-pressed={details === "config"}
          onClick={() => setDetails("config")}
        >
          参数对比
        </button>
        <button
          aria-pressed={details === "metrics"}
          onClick={() => setDetails("metrics")}
        >
          测量记录 · {report.trials.length}
        </button>
      </div>
      {details === "config" ? (
        <table
          className="calibration-table"
          aria-label="起始参数与推荐参数对比"
        >
          <thead>
            <tr>
              <th>J{axis + 1} 参数</th>
              <th>起始</th>
              <th>推荐</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <th>摩擦补偿</th>
              <td>{report.baseline.friction_enabled ? "开启" : "关闭"}</td>
              <td>
                {report.recommended
                  ? report.recommended.friction_enabled
                    ? "开启"
                    : "关闭"
                  : "—"}
              </td>
            </tr>
            {fields.map(([key, label, unit]) => (
              <tr key={key}>
                <th>
                  {label} <small>{unit}</small>
                </th>
                <td>{baseline[key]}</td>
                <td
                  data-changed={
                    recommended && recommended[key] !== baseline[key]
                  }
                >
                  {recommended?.[key] ?? "—"}
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      ) : report.trials.length ? (
        <div className="calibration-table-scroll">
          <table
            className="calibration-table calibration-metrics"
            aria-label="校准测量记录"
          >
            <thead>
              <tr>
                <th>试验</th>
                <th>窗口</th>
                <th>
                  速度 RMS<small>°/s</small>
                </th>
                <th>
                  跟踪 RMS<small>°</small>
                </th>
                <th>
                  跟踪峰值<small>°</small>
                </th>
                <th>
                  其他轴峰值<small>°</small>
                </th>
                <th>
                  有效时间<small>s</small>
                </th>
              </tr>
            </thead>
            <tbody>
              {report.trials.map((trial) => (
                <tr key={`${trial.index}:${trial.phase}`}>
                  <th>
                    {trial.index} ·{" "}
                    {stages[trial.phase as CalibrationStatus["stage"]] ??
                      trial.phase}
                  </th>
                  <td>{trial.metrics?.windows ?? "—"}</td>
                  <td>{degrees(trial.metrics?.velocity_rms, 3)}</td>
                  <td>{degrees(trial.metrics?.tracking_rms, 3)}</td>
                  <td>{degrees(trial.metrics?.tracking_peak, 3)}</td>
                  <td>{degrees(trial.metrics?.other_peak, 3)}</td>
                  <td>
                    {trial.metrics
                      ? (trial.metrics.valid_ms / 1000).toFixed(1)
                      : "—"}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      ) : (
        <p className="calibration-empty">没有测量记录</p>
      )}
    </div>
  );
}

export default function CalibrationPanel({
  view,
  open,
  onClose,
  onStop,
}: CalibrationPanelProps) {
  const s = view.snapshot;
  const status = s.calibration;
  const readiness = s.calibration_readiness;
  const feedbackFresh = hasFreshFeedback(view);
  const connection = `${view.baseUrl}:${view.connectionEpoch ?? 0}:${s.side}:${s.port ?? ""}:${["disconnected", "connecting", "closing"].includes(s.phase) ? "detached" : "attached"}`;
  const tcp = JSON.stringify(s.tcp_transform);
  const [page, setPage] = useState<Page>("settings");
  const [draft, setDraft] = useState<Draft>({
    axis: 1,
    excursion: "6",
    speed: "1",
    friction: "0.1",
  });
  const [pending, setPendingState] = useState<Pending | null>(null);
  const pendingRef = useRef<Pending | null>(null);
  const [run, setRunState] = useState<Run | null>(null);
  const runRef = useRef<Run | null>(null);
  const [liveReport, setLiveReport] = useState<CalibrationReport | null>(null);
  const [eligibility, setEligibility] = useState<Eligibility | null>(null);
  const [appliedReport, setAppliedReport] = useState<string | null>(null);
  const [feedback, setFeedback] = useState<Feedback | null>(null);
  const [stopPending, setStopPending] = useState<{
    seq: number;
    id: string | null;
  } | null>(null);
  const [history, setHistory] = useState<CalibrationReport[]>([]);
  const [historyBusy, setHistoryBusy] = useState(false);
  const [historyError, setHistoryError] = useState("");
  const [historyReload, setHistoryReload] = useState(0);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [historyDetail, setHistoryDetail] = useState<CalibrationReport | null>(
    null,
  );
  const [detailBusy, setDetailBusy] = useState(false);
  const [detailError, setDetailError] = useState("");
  const invalidated = useRef(new Set<string>());
  const listRequest = useRef<AbortController | null>(null);
  const detailRequest = useRef<AbortController | null>(null);
  const dialog = useRef<HTMLElement>(null);
  const lastFocus = useRef<HTMLElement | null>(null);
  const alive = useRef(true);
  const previousConnection = useRef({ connection, online: view.online });
  const latest = useRef({
    view,
    connection,
    open,
    onClose,
    onStop,
    stop: () => {},
  });
  latest.current = { view, connection, open, onClose, onStop, stop };
  const settings = readSettings(draft);
  const calibrationActive =
    s.action?.mode === "calibration" || (!!status && !terminal(status));
  const waitingForStart =
    run?.connection === connection && run.state !== "terminal";
  const busy =
    calibrationActive ||
    !!s.action ||
    !!pending ||
    !!stopPending ||
    waitingForStart;
  const currentAxis = calibrationActive
    ? (status?.axis ?? s.action?.axis ?? draft.axis)
    : draft.axis;
  const center = feedbackFresh ? s.joints[currentAxis]?.q : null;
  const excursion = settings?.excursion;
  const completedKey =
    status && terminal(status) ? `${status.id}:${status.state}` : "";

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
      listRequest.current?.abort();
      detailRequest.current?.abort();
    };
  }, []);

  useEffect(() => {
    const previous = previousConnection.current;
    previousConnection.current = { connection, online: view.online };
    if (
      previous.connection === connection &&
      !(previous.online && !view.online)
    )
      return;
    if (runRef.current) invalidated.current.add(runRef.current.id);
    const request = pendingRef.current;
    if (request) invalidated.current.add(request.id);
    setPending(null);
    setRun(null);
    setEligibility(null);
    setAppliedReport(null);
    setStopPending(null);
    if (request) setFeedback({ text: "连接已变化，请求未确认", tone: "error" });
  }, [connection, view.online]);

  useEffect(() => {
    if (status?.report) setLiveReport(status.report);
    if (!view.online || !status || invalidated.current.has(status.request_id))
      return;
    const current = runRef.current;
    if (
      !terminal(status) &&
      s.action?.mode === "calibration" &&
      s.action.request_id === status.request_id
    ) {
      if (
        !current ||
        current.id !== status.request_id ||
        current.state === "waiting"
      )
        setRun({ id: status.request_id, connection, state: "running" });
    } else if (
      terminal(status) &&
      current?.id === status.request_id &&
      current.connection === connection
    ) {
      if (current.state !== "terminal") {
        setRun({ ...current, state: "terminal" });
        setPage("result");
      }
      if (status.report) {
        setEligibility((before) =>
          before?.id === status.report!.id
            ? before
            : {
                id: status.report!.id,
                connection,
                revision: s.servo_tuning_revision,
                tcp,
              },
        );
      }
    }
  }, [status, s.action, s.servo_tuning_revision, connection, tcp, view.online]);

  useEffect(() => {
    const request = pendingRef.current;
    if (!request) return;
    const fail = (text: string, invalidate = true, code?: string) => {
      if (invalidate) invalidated.current.add(request.id);
      setPending(null);
      if (request.kind === "start") {
        setRun(null);
        setPage("settings");
      }
      setFeedback({
        text,
        code,
        tone: code === "motion_busy" ? "info" : "error",
      });
    };
    if (!view.online || request.connection !== connection) {
      fail("连接已变化，请求未确认");
      return;
    }
    if (tcp !== request.tcp) {
      fail("工具配置已变化，请求未确认");
      return;
    }
    const result =
      s.results?.find((item) => item.request_id === request.id) ??
      (s.last_result?.request_id === request.id ? s.last_result : null);
    const code =
      request.kind === "start" ? "calibrate_start" : "calibrate_apply";
    const matchingStatus =
      request.kind === "start" && status?.request_id === request.id;
    // The first matching snapshot may already contain the terminal task.
    // That confirms admission even when intermediate accepted frames were missed.
    if (matchingStatus && terminal(status)) {
      setPending(null);
      setRun({ id: request.id, connection, state: "terminal" });
      setFeedback(null);
      setPage("result");
      return;
    }
    if (result && !["accepted", "completed"].includes(result.status)) {
      fail(
        result.message || "请求未接受",
        false,
        result.status === "rejected" ? result.code : undefined,
      );
      return;
    }
    const accepted =
      request.accepted ||
      (result?.status === "accepted" && result.code === code);
    if (result?.status === "accepted" && result.code !== code) {
      fail("命令结果不匹配");
      return;
    }
    // Calibration changes generation while configuring its trials and applying
    // recommendations. Once accepted, keep observing this request even if its
    // HTTP acknowledgement or an ownership change arrives later; never resend.
    if (
      !accepted &&
      !matchingStatus &&
      (!view.hasControl || s.generation !== request.generation)
    ) {
      fail("控制权或操作状态已变化，请求未确认");
      return;
    }
    if (request.kind === "start") {
      if (matchingStatus || accepted) {
        setPending(null);
        setRun({
          id: request.id,
          connection,
          state: matchingStatus && terminal(status) ? "terminal" : "running",
        });
        setFeedback(null);
        setPage(matchingStatus && terminal(status) ? "result" : "progress");
      } else if (
        s.servo_tuning_revision !== request.revision ||
        !sameServoTuning(s.servo_tuning, request.config)
      ) {
        fail("伺服参数已变化，启动未确认");
      }
      return;
    }
    if (
      accepted &&
      s.servo_tuning_revision === request.revision + 1 &&
      sameServoTuning(s.servo_tuning, request.config)
    ) {
      setPending(null);
      setAppliedReport(liveReport?.id ?? null);
      setFeedback({ text: "推荐参数已应用", tone: "success" });
    } else if (
      s.servo_tuning_revision > request.revision + 1 ||
      (s.servo_tuning_revision !== request.revision &&
        !sameServoTuning(s.servo_tuning, request.config)) ||
      s.action ||
      !["ready", "disabled"].includes(s.phase)
    ) {
      fail("参数或动作已变化，应用未确认");
    } else if (accepted && !request.accepted)
      setPending({ ...request, accepted: true });
  }, [view, connection, tcp, pending, liveReport]);

  useEffect(() => {
    if (!pending || pending.timedOut) return;
    const id = pending.id;
    const timer = setTimeout(() => {
      const request = pendingRef.current;
      if (!request || request.id !== id) return;
      setPending({ ...request, timedOut: true });
      setFeedback({
        text:
          request.kind === "start"
            ? "启动尚未确认，正在等待任务状态"
            : "应用尚未确认，正在等待参数状态",
        tone: "error",
      });
    }, 8000);
    return () => clearTimeout(timer);
  }, [pending?.id, pending?.timedOut]);

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
      setRun(null);
    }
  }, [stopPending, view.online, s.seq, s.phase, s.action, status]);

  useEffect(() => {
    if (open && (calibrationActive || runRef.current?.state === "waiting"))
      setPage("progress");
  }, [open]);

  useEffect(() => {
    if (!open || !view.online) {
      setHistoryBusy(false);
      return;
    }
    const controller = new AbortController();
    listRequest.current = controller;
    setHistoryBusy(true);
    setHistoryError("");
    void api
      .listCalibrations(20, controller.signal)
      .then((reports) => {
        if (
          !alive.current ||
          controller.signal.aborted ||
          latest.current.connection !== connection
        )
          return;
        setHistory((previous) => {
          const received = new Set(reports.map((report) => report.id));
          return [
            ...reports,
            ...previous.filter((report) => !received.has(report.id)),
          ];
        });
      })
      .catch((error) => {
        if (
          alive.current &&
          !controller.signal.aborted &&
          latest.current.connection === connection
        )
          setHistoryError(
            error instanceof Error ? error.message : "历史读取失败",
          );
      })
      .finally(() => {
        if (
          alive.current &&
          !controller.signal.aborted &&
          listRequest.current === controller
        )
          setHistoryBusy(false);
      });
    return () => controller.abort();
  }, [open, view.online, connection, historyReload, completedKey]);

  useEffect(() => {
    if (!open || !view.online || !selectedId) {
      setDetailBusy(false);
      return;
    }
    const controller = new AbortController();
    detailRequest.current = controller;
    setDetailBusy(true);
    setDetailError("");
    void api
      .readCalibration(selectedId, controller.signal)
      .then((report) => {
        if (
          !alive.current ||
          controller.signal.aborted ||
          latest.current.connection !== connection
        )
          return;
        setHistoryDetail(report);
        setHistory((reports) =>
          reports.map((item) => (item.id === report.id ? report : item)),
        );
      })
      .catch((error) => {
        if (
          alive.current &&
          !controller.signal.aborted &&
          latest.current.connection === connection
        )
          setDetailError(
            error instanceof Error ? error.message : "报告读取失败",
          );
      })
      .finally(() => {
        if (
          alive.current &&
          !controller.signal.aborted &&
          detailRequest.current === controller
        )
          setDetailBusy(false);
      });
    return () => controller.abort();
  }, [open, view.online, connection, selectedId]);

  useEffect(() => {
    if (!open) return;
    const previous = document.activeElement;
    const frame = requestAnimationFrame(() => {
      const field = lastFocus.current;
      if (field?.isConnected && field.getClientRects().length) field.focus();
      else
        (
          dialog.current?.querySelector<HTMLElement>(
            ".calibration-joints button[aria-pressed=true]",
          ) ?? dialog.current
        )?.focus();
    });
    return () => {
      cancelAnimationFrame(frame);
      if (
        previous instanceof HTMLElement &&
        previous.isConnected &&
        previous.getClientRects().length
      )
        previous.focus();
      else
        Array.from(document.querySelectorAll<HTMLElement>(".stop-button"))
          .find((button) => button.getClientRects().length)
          ?.focus();
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
          'button:not(:disabled), input:not(:disabled), select:not(:disabled), a[href], [tabindex="0"]',
        ) ?? [],
      ).filter((element) => element.getClientRects().length);
      const first = controls[0];
      const last = controls[controls.length - 1];
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

  const preparationBlock =
    !view.online || ["disconnected", "connecting", "closing"].includes(s.phase)
      ? "未连接"
      : !view.hasControl
        ? "未取得控制权"
        : s.phase === "fault" || s.error
          ? s.error?.message || "故障"
          : !s.enabled
            ? "未使能"
            : s.phase !== "ready" || s.action || calibrationActive
              ? "等待当前动作结束"
              : !feedbackFresh
                ? "等待新鲜的七轴反馈"
                : "";
  const preparationTitle =
    (preparationBlock &&
      (s.phase === "fault" || s.error
        ? "准备异常"
        : view.online && view.hasControl && readiness?.code === "motion_busy"
          ? "检测中"
          : preparationBlock)) ||
    (readiness
      ? {
          ready: "可开始",
          settling: "检测中",
          moving: "待停稳",
          unavailable:
            readiness.code === "motion_busy" ? "检测中" : "暂不可开始",
        }[readiness.state]
      : "等待准备状态");
  const unavailableReason =
    view.online && view.hasControl && readiness?.state === "unavailable"
      ? readiness.message
      : "";
  const preparationMessage =
    s.error?.message ||
    unavailableReason ||
    (preparationBlock
      ? "准备条件恢复后会重新检测。"
      : readiness?.message || "等待准备状态，确认后请手动开始。");
  const startReason =
    (preparationBlock && (unavailableReason || preparationBlock)) ||
    (busy ? "等待当前请求完成" : "") ||
    (readiness?.state === "ready"
      ? ""
      : readiness?.message || preparationTitle);
  const observing =
    !preparationBlock &&
    !!readiness &&
    ["ready", "settling", "moving"].includes(readiness.state);
  const requiredMs =
    readiness && readiness.required_ms > 0 ? readiness.required_ms : 300;
  const observedMs = observing
    ? Math.min(requiredMs, Math.max(0, readiness.observed_ms))
    : 0;
  const preparationTone =
    s.error || view.error || readiness?.code === "fault" ? "error" : "info";
  // A refused start is never queued for retry. Follow live preparation state
  // instead of keeping that temporary refusal on screen after it recovers.
  const currentFeedback = feedback?.code === "motion_busy" ? null : feedback;
  const validation = !settings
    ? "最大位移 1–10°，速度 0.2865–2.2918°/s，补偿上限 0–2 Nm"
    : "";
  const applyReason = !liveReport?.recommended
    ? "本次没有推荐参数"
    : !eligibility ||
        eligibility.id !== liveReport.id ||
        eligibility.connection !== connection
      ? "此报告不属于当前连接的校准"
      : preparationBlock
        ? unavailableReason || preparationBlock
        : readiness?.state !== "ready"
          ? readiness?.message || preparationTitle
          : stopPending
            ? "等待停止确认"
            : tcp !== eligibility.tcp
              ? "工具配置已变化"
              : appliedReport === liveReport.id
                ? "推荐参数已应用"
                : s.servo_tuning_revision !== eligibility.revision ||
                    !sameServoTuning(s.servo_tuning, liveReport.baseline)
                  ? "起始参数已变化，请重新校准"
                  : pending
                    ? "等待当前请求完成"
                    : "";

  function stop() {
    const current = runRef.current;
    const request = pendingRef.current;
    if (current) invalidated.current.add(current.id);
    if (request) invalidated.current.add(request.id);
    setPending(null);
    setRun(null);
    setStopPending({
      seq: s.seq,
      id: status?.request_id ?? current?.id ?? request?.id ?? null,
    });
    setFeedback({ text: "已请求停止", tone: "info" });
    onStop();
  }
  function send(kind: "start" | "apply") {
    if (pendingRef.current || !open) return;
    if (
      kind === "start"
        ? !settings || !!startReason || !!validation
        : !!applyReason || !liveReport?.recommended
    )
      return;
    if (!hasFreshFeedback(latest.current.view)) {
      setFeedback({ text: "等待新鲜的七轴反馈", tone: "info" });
      return;
    }
    const id = crypto.randomUUID();
    const request: Pending = {
      kind,
      id,
      connection,
      generation: s.generation,
      revision: s.servo_tuning_revision,
      config: structuredClone(
        kind === "apply" ? liveReport!.recommended! : s.servo_tuning,
      ),
      tcp,
      timedOut: false,
      accepted: false,
    };
    setPending(request);
    setFeedback(null);
    if (kind === "start") {
      setRun({ id, connection, state: "waiting" });
      setEligibility(null);
      setAppliedReport(null);
      setPage("progress");
    }
    void api
      .command(
        kind === "start" ? "calibrate_start" : "calibrate_apply",
        kind === "start" ? { settings } : { calibration_id: liveReport!.id },
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
          setPending({ ...pendingRef.current, timedOut: true });
          setFeedback({
            text: "命令入队响应无效，正在等待状态",
            tone: "error",
          });
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
            if (runRef.current?.id === id) setRun(null);
            setPage("settings");
          }
          setFeedback({
            text: error.message,
            code: error.code ?? undefined,
            tone: error.code === "motion_busy" ? "info" : "error",
          });
          return;
        }
        setPending({ ...pendingRef.current, timedOut: true });
        setFeedback({
          text: `${kind === "start" ? "启动" : "应用"}未确认：${error instanceof Error ? error.message : "请求失败"}`,
          tone: "error",
        });
      });
  }

  const progress = percent(
    status?.progress ??
      (s.action?.mode === "calibration" ? s.action.progress : 0),
  );
  const progressTitle =
    stopPending || status?.state === "stopping"
      ? "正在停止并恢复参数"
      : pending?.kind === "start"
        ? pending.timedOut
          ? "等待启动确认"
          : "等待任务接受"
        : status && !terminal(status)
          ? stages[status.stage]
          : calibrationActive || waitingForStart
            ? "等待校准状态"
            : status
              ? stages[status.stage]
              : "尚未开始校准";
  const shownHistory =
    liveReport && !history.some((report) => report.id === liveReport.id)
      ? [liveReport, ...history]
      : history;
  const selectedReport =
    historyDetail?.id === selectedId
      ? historyDetail
      : shownHistory.find((report) => report.id === selectedId);
  const footerFeedback =
    s.error?.message ||
    view.error ||
    currentFeedback?.text ||
    (page === "settings"
      ? validation || startReason
      : page === "result" && liveReport?.recommended
        ? applyReason
        : "") ||
    (page === "settings"
      ? "完成后恢复起始参数，推荐值需手动应用。"
      : "关闭窗口后可从参数入口继续查看。");

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
            title="关闭"
            onClick={onClose}
          >
            ×
          </button>
        </header>
        <nav className="calibration-tabs" aria-label="校准页面">
          {(
            [
              ["settings", "设置"],
              ["progress", "进度"],
              ["result", "结果"],
              ["history", "历史"],
            ] as const
          ).map(([key, label]) => (
            <button
              key={key}
              aria-pressed={page === key}
              aria-controls={`calibration-${key}`}
              onClick={() => setPage(key)}
            >
              {label}
              {key === "progress" && calibrationActive && (
                <i className="calibration-running-dot" />
              )}
            </button>
          ))}
        </nav>
        <div className="calibration-body" id={`calibration-${page}`}>
          {page === "settings" && (
            <div className="calibration-settings">
              <div className="calibration-setup">
                <h3>测试关节</h3>
                <div className="calibration-joints" aria-label="测试关节">
                  {Array.from({ length: 7 }, (_, axis) => (
                    <button
                      key={axis}
                      aria-pressed={draft.axis === axis}
                      disabled={busy}
                      onClick={() => {
                        setDraft({ ...draft, axis });
                        setFeedback(null);
                      }}
                    >
                      J{axis + 1}
                    </button>
                  ))}
                </div>
                <div className="calibration-fields">
                  {(
                    [
                      ["excursion", "最大位移 ±", "°", 1, 10, 0.5],
                      [
                        "speed",
                        "测试速度",
                        "°/s",
                        0.005 / RAD,
                        0.04 / RAD,
                        0.1,
                      ],
                      ["friction", "补偿试验上限", "Nm", 0, 2, 0.01],
                    ] as const
                  ).map(([key, label, unit, min, max, step]) => {
                    const value = Number(draft[key]);
                    const invalid =
                      !draft[key].trim() ||
                      !Number.isFinite(value) ||
                      value < min ||
                      value > max;
                    return (
                      <label key={key} className="calibration-field">
                        <span>{label}</span>
                        <div>
                          <input
                            type="number"
                            aria-label={label}
                            aria-invalid={invalid}
                            aria-describedby={
                              invalid ? "calibration-feedback" : undefined
                            }
                            min={min}
                            max={max}
                            step={step}
                            value={draft[key]}
                            disabled={busy}
                            onChange={(event) => {
                              setDraft({ ...draft, [key]: event.target.value });
                              setFeedback(null);
                            }}
                          />
                          <span>{unit}</span>
                        </div>
                      </label>
                    );
                  })}
                </div>
                <p className="calibration-note">
                  固定阻尼，比较摩擦补偿与速度前馈。
                </p>
              </div>
              <div className="calibration-context">
                <section
                  className="calibration-readiness"
                  aria-label="校准准备状态"
                  data-state={
                    preparationBlock
                      ? "unavailable"
                      : (readiness?.state ?? "unavailable")
                  }
                  data-tone={preparationTone}
                >
                  <div className="calibration-readiness-heading">
                    <small>开始前检查</small>
                    <strong role="status">{preparationTitle}</strong>
                  </div>
                  <p>{preparationMessage}</p>
                  <progress
                    className="calibration-progress"
                    aria-label="静止确认进度"
                    max={requiredMs}
                    value={observedMs}
                  />
                  <div className="calibration-readiness-time">
                    <span>
                      {readiness
                        ? `${Math.round(observedMs)} / ${requiredMs} ms`
                        : "尚无准备数据"}
                    </span>
                    <span>需连续静止</span>
                  </div>
                  <dl>
                    <div>
                      <dt>
                        {observing && readiness.joint != null
                          ? `最大波动 J${readiness.joint + 1}`
                          : "最大位置波动"}
                      </dt>
                      <dd>
                        {degrees(observing ? readiness.position_span : null, 3)}
                        °
                        <small>
                          {" "}
                          / {degrees(readiness?.position_limit, 3)}°
                        </small>
                      </dd>
                    </div>
                  </dl>
                </section>
                <div className="calibration-range">
                  <div className="calibration-range-heading">
                    <small>
                      J{currentAxis + 1}{" "}
                      {feedbackFresh ? "当前实测" : "实测反馈不可用"}
                    </small>
                    <strong>
                      {degrees(center)}
                      <span>°</span>
                    </strong>
                  </div>
                  <div className="calibration-range-track">
                    <span />
                    <i />
                  </div>
                  <div className="calibration-range-limits">
                    <span>
                      {excursion != null ? `−${degrees(excursion, 1)}` : "—"}°
                    </span>
                    <span>
                      {excursion != null ? `+${degrees(excursion, 1)}` : "—"}°
                    </span>
                  </div>
                  <p>目标相对发送参考；实际位移相对起始实测。</p>
                  <dl>
                    <div>
                      <dt>目标偏移</dt>
                      <dd>
                        {excursion != null
                          ? `±${degrees(excursion / 2, 1)}°`
                          : "—"}
                      </dd>
                    </div>
                    <div>
                      <dt>实测偏移上限</dt>
                      <dd>
                        {excursion != null ? `±${degrees(excursion, 1)}°` : "—"}
                      </dd>
                    </div>
                  </dl>
                </div>
              </div>
            </div>
          )}
          {page === "progress" && (
            <div className="calibration-progress-view">
              <div className="calibration-progress-heading">
                <div>
                  <small>
                    {calibrationActive
                      ? `J${currentAxis + 1} · 有限往返`
                      : "低速往返测试"}
                  </small>
                  <h3>{progressTitle}</h3>
                </div>
                <strong>{progress}%</strong>
              </div>
              <progress
                className="calibration-progress"
                max="100"
                value={progress}
                aria-label="校准进度"
              />
              <ol className="calibration-stages">
                {(
                  [
                    "baseline",
                    "candidate",
                    "baseline_check",
                    "validation",
                    "restoring",
                  ] as const
                ).map((stage) => (
                  <li key={stage} data-active={status?.stage === stage}>
                    {stages[stage]}
                  </li>
                ))}
              </ol>
              <dl className="calibration-progress-facts">
                <div>
                  <dt>试验</dt>
                  <dd>
                    {status
                      ? `${status.trial_index} / ${status.trial_count}`
                      : "—"}
                  </dd>
                </div>
                <div>
                  <dt>往返片段</dt>
                  <dd>
                    {status ? `${status.leg_index} / ${status.leg_count}` : "—"}
                  </dd>
                </div>
                <div>
                  <dt>当前实测</dt>
                  <dd>
                    J{currentAxis + 1} · {degrees(center)}°
                  </dd>
                </div>
              </dl>
              <p className="calibration-note">
                {!view.online
                  ? "状态连接已断开"
                  : terminal(
                        status ?? ({ state: "running" } as CalibrationStatus),
                      )
                    ? "校准已结束，可在结果页查看报告。"
                    : "结束时恢复起始参数，随后可查看比较结果。"}
              </p>
              {status?.report && (
                <button onClick={() => setPage("result")}>查看结果</button>
              )}
            </div>
          )}
          {page === "result" &&
            (liveReport ? (
              <ReportDetails key={liveReport.id} report={liveReport} />
            ) : (
              <div className="calibration-empty">
                <h3>尚无校准报告</h3>
                <p>
                  {calibrationActive
                    ? "校准结束后在此查看结果。"
                    : "开始一次低速往返测试，或查看历史记录。"}
                </p>
              </div>
            ))}
          {page === "history" && (
            <div className="calibration-history">
              <div className="calibration-history-toolbar">
                {selectedId ? (
                  <button
                    onClick={() => {
                      setSelectedId(null);
                      setDetailError("");
                    }}
                  >
                    ‹ 返回列表
                  </button>
                ) : (
                  <span>最近 {shownHistory.length} 份报告</span>
                )}
                <small>
                  {detailBusy && selectedId
                    ? "正在读取报告…"
                    : historyBusy
                      ? "正在读取…"
                      : !view.online
                        ? "离线记录"
                        : ""}
                </small>
                <button
                  disabled={!view.online || historyBusy}
                  onClick={() => setHistoryReload((value) => value + 1)}
                >
                  刷新
                </button>
              </div>
              {(selectedId ? detailError : historyError) && (
                <p className="calibration-error" role="alert">
                  {selectedId ? detailError : historyError}
                </p>
              )}
              {selectedId ? (
                selectedReport ? (
                  <>
                    <ReportDetails
                      key={selectedReport.id}
                      report={selectedReport}
                    />
                    <div className="calibration-history-download">
                      <button onClick={() => downloadReport(selectedReport)}>
                        下载 JSON
                      </button>
                    </div>
                  </>
                ) : (
                  <p className="calibration-empty">
                    {detailBusy ? "正在读取报告…" : "未读取到报告"}
                  </p>
                )
              ) : shownHistory.length ? (
                <div className="calibration-history-list">
                  {shownHistory.map((report) => (
                    <button
                      className="calibration-history-item"
                      key={report.id}
                      onClick={() => {
                        setHistoryDetail(report);
                        setSelectedId(report.id);
                        setDetailError("");
                      }}
                    >
                      <span>
                        <b>J{report.settings.axis + 1}</b>
                        <small>
                          {report.side === "left" ? "左臂" : "右臂"} ·{" "}
                          {timestamp(report.created_at_ms)}
                        </small>
                      </span>
                      <span>{outcomes[report.outcome]}</span>
                      <span aria-hidden="true">›</span>
                    </button>
                  ))}
                </div>
              ) : (
                <p className="calibration-empty">
                  {historyBusy ? "正在读取校准记录…" : "暂无校准记录"}
                </p>
              )}
            </div>
          )}
        </div>
        <footer className="calibration-footer">
          <span
            id="calibration-feedback"
            data-tone={
              s.error || view.error
                ? "error"
                : (currentFeedback?.tone ??
                  (validation && page === "settings"
                    ? "error"
                    : preparationTone))
            }
            role={
              s.error || view.error || currentFeedback?.tone === "error"
                ? "alert"
                : "status"
            }
          >
            {footerFeedback}
          </span>
          {page === "settings" && (
            <button
              className="primary"
              disabled={!!startReason || !settings || busy}
              aria-busy={pending?.kind === "start"}
              onClick={() => send("start")}
            >
              开始校准
            </button>
          )}
          {page === "result" && liveReport && (
            <button onClick={() => downloadReport(liveReport)}>
              下载 JSON
            </button>
          )}
          {page === "result" && liveReport?.recommended && (
            <button
              className="primary"
              disabled={!!applyReason}
              title={applyReason || "应用推荐参数"}
              aria-busy={pending?.kind === "apply"}
              onClick={() => send("apply")}
            >
              {pending?.kind === "apply"
                ? "等待确认…"
                : appliedReport === liveReport.id
                  ? "已应用"
                  : "应用推荐"}
            </button>
          )}
          <button onClick={onClose}>关闭</button>
        </footer>
      </section>
    </div>
  );
}
