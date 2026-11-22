import { useEffect, useRef, useState, type ChangeEvent } from "react";
import {
  api,
  ApiError,
  parseCalibrationReport,
  sameServoTuning,
  type CalibrationMetrics,
  type CalibrationPoint,
  type CalibrationReport,
  type CalibrationStatus,
  type CalibrationTrial,
  type ServoTuning,
  type View,
} from "./api";
import CalibrationChart, {
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
type Pending = {
  kind: "start" | "apply";
  id: string;
  connection: string;
  generation: number;
  revision: number;
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
  baseline: "测量七轴基线",
  candidate: "比较候选参数",
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
    (axis) => calibrationTrials(report, axis).length > 0,
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
    return (
      (0.6 * m.velocity_rms) / Math.max(b.velocity_rms, 1e-12) +
      (0.4 * m.tracking_rms) / Math.max(b.tracking_rms, 1e-12)
    );
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
}: {
  report: CalibrationReport;
  local: boolean;
}) {
  const axes = measuredAxes(report);
  const fields = [
    ["kd", "阻尼 Kd"],
    ["velocity_ff", "速度前馈"],
    ["coulomb", "补偿 Fc"],
    ["viscous", "黏性补偿 B"],
    ["velocity_scale", "过零速度"],
    ["max_friction", "补偿限幅"],
  ] as const;
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
            ? `${(report.sample_count || report.curves.length).toLocaleString()} 采样`
            : report.trials.length
              ? `${report.trials.length} 轮测量`
              : "统计报告"}
        </small>
      </div>
      <p className="calibration-report-message">
        {["failed", "cancelled"].includes(report.outcome)
          ? report.message
          : report.recommended
            ? "已恢复起始参数。"
            : "各轮测量已保留，可查看每轴的变化与曲线。"}
      </p>
      {axes.length > 0 && (
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
      {report.trials.length > 0 ? (
        <>
          <h4 className="calibration-section-label">
            各轮测量 <small>相对同轴基线</small>
          </h4>
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
                {report.trials.map((trial) => {
                  const axis = trial.axis ?? report.settings.axis;
                  const baseline =
                    axis == null ? null : comparisonFor(report, axis).baseline;
                  return (
                    <tr key={`${trial.index}:${axis ?? "all"}:${trial.phase}`}>
                      <th>
                        {axis == null ? "七轴" : `J${axis + 1}`} ·{" "}
                        {axis == null
                          ? trial.phase === "after"
                            ? "整体对比 after"
                            : trial.phase
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
        </>
      ) : (
        <p className="calibration-note">此报告没有完整测量轮次。</p>
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
                  fields.map(([key, label]) => (
                    <tr key={`${axis}:${key}`}>
                      <th>J{axis + 1}</th>
                      <th>{label}</th>
                      <td>{report.baseline.joints[axis][key]}</td>
                      <td
                        data-changed={
                          !!report.recommended &&
                          report.recommended.joints[axis][key] !==
                            report.baseline.joints[axis][key]
                        }
                      >
                        {report.recommended?.joints[axis][key] ?? "—"}
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
  const [appliedReport, setAppliedReport] = useState<string | null>(null);
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
    };
  }, []);
  useEffect(() => {
    const before = previousConnection.current;
    previousConnection.current = { connection, online: view.online };
    if (before.connection === connection && !(before.online && !view.online))
      return;
    const request = pendingRef.current;
    setPending(null);
    setRun(null);
    setStopPending(null);
    setAppliedReport(null);
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
      request.kind === "apply" &&
      accepted &&
      s.servo_tuning_revision > request.revision &&
      sameServoTuning(s.servo_tuning, request.config)
    ) {
      setPending(null);
      setAppliedReport(status?.report?.id ?? null);
      setFeedback({ text: "推荐参数已应用", tone: "success" });
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
        lastFocus.current.getClientRects().length
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
      ).filter((element) => element.getClientRects().length);
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
        : s.phase === "fault" || s.error
          ? s.error?.message || "请先处理故障"
          : !s.enabled
            ? "请先使能机械臂"
            : s.phase !== "ready" || s.action
              ? "请先停止当前动作"
              : "";
  const startReason = idleReason || (busy ? "等待当前请求完成" : "");
  // The current report and servo config are sufficient; TCP/pose changes do not invalidate tuning.
  const applyReason =
    local || reportId !== status?.report?.id
      ? "历史报告仅供查看"
      : !report?.recommended
        ? "本次参数保持原样"
        : idleReason ||
          (busy ? "等待当前请求完成" : "") ||
          (appliedReport === reportId ? "推荐参数已应用" : "") ||
          (!sameServoTuning(s.servo_tuning, report.baseline)
            ? "伺服参数已修改，请重新测试"
            : "");
  function stop() {
    const id =
      status?.request_id ??
      runRef.current?.id ??
      pendingRef.current?.id ??
      null;
    setPending(null);
    setStopPending({ seq: s.seq, id });
    setFeedback({ text: "已请求停止", tone: "info" });
    onStop();
  }
  function send(kind: "start" | "apply") {
    if (
      !open ||
      pendingRef.current ||
      (kind === "start" ? startReason : applyReason)
    )
      return;
    const id = crypto.randomUUID();
    const request: Pending = {
      kind,
      id,
      connection,
      generation: s.generation,
      revision: s.servo_tuning_revision,
      config: structuredClone(
        kind === "apply" ? report!.recommended! : s.servo_tuning,
      ),
      accepted: false,
      uncertain: false,
    };
    setPending(request);
    setFeedback(null);
    if (kind === "start") {
      setRun({ id, connection, state: "waiting" });
      setAppliedReport(null);
      setSelection(null);
      setPage("progress");
    }
    void api
      .command(
        kind === "start" ? "calibrate_start" : "calibrate_apply",
        kind === "start" ? {} : { calibration_id: reportId },
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
            setRun(null);
            setPage("settings");
          }
          setFeedback({ text: error.message, tone: "info" });
        } else {
          setPending({ ...pendingRef.current, uncertain: true });
          setFeedback({
            text: `${kind === "start" ? "启动" : "应用"}未确认：${error instanceof Error ? error.message : "请求失败"}`,
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
      ? startReason || "完成后回原位并恢复起始参数。"
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
                <small>J1–J7 · 自动小范围测试</small>
                <h3>以当前姿态为原位</h3>
                <p>
                  逐轴往返 ±2°，以 1、2°/s
                  测量跟随与速度变化。自动比较后，七轴回到原位。
                </p>
                <div className="calibration-start-facts">
                  <span>
                    <b>7</b> 个关节
                  </span>
                  <span>
                    <b>±2°</b> 目标范围
                  </span>
                  <span>
                    <b>1 / 2</b> °/s
                  </span>
                </div>
                <p className="calibration-note">
                  连接、取得控制权并使能后即可开始，测试中可随时停止。
                </p>
              </div>
              <div className="calibration-preview">
                <h3>试验预览</h3>
                <ol>
                  <li>
                    <b>测量基线</b>
                    <span>J1 → J7，逐轴记录</span>
                  </li>
                  <li>
                    <b>比较参数</b>
                    <span>对跟随与速度变化较大的轴试验</span>
                  </li>
                  <li>
                    <b>整体对比</b>
                    <span>复测七轴，回原位并恢复参数</span>
                  </li>
                </ol>
                <p className="calibration-note">
                  曲线记录控制参考、成功发送与实测值。
                </p>
              </div>
            </div>
          )}
          {page === "progress" && (
            <div className="calibration-progress-view">
              <div className="calibration-progress-heading">
                <div>
                  <small>
                    {active ? `当前 J${axis + 1}` : "七轴小范围测试"}
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
                {(["baseline", "candidate", "after", "restoring"] as const).map(
                  (stage) => (
                    <li
                      key={stage}
                      data-active={
                        status?.stage === stage ||
                        (stage === "after" && status?.stage === "validation")
                      }
                    >
                      {stages[stage]}
                    </li>
                  ),
                )}
              </ol>
              <dl className="calibration-progress-facts">
                <div>
                  <dt>当前关节</dt>
                  <dd>
                    J{axis + 1} · {degrees(s.joints[axis]?.q, 2)}°
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
            <button
              className="primary"
              disabled={!!startReason}
              aria-busy={pending?.kind === "start"}
              onClick={() => send("start")}
            >
              开始校准
            </button>
          )}
          {(page === "result" || page === "curves") && report && (
            <button
              disabled={downloading || (!local && !view.online)}
              onClick={() => void download()}
            >
              {downloading ? "正在下载…" : "下载完整 JSON"}
            </button>
          )}
          {page === "result" &&
            report?.recommended &&
            !local &&
            reportId === status?.report?.id && (
              <button
                className="primary"
                disabled={!!applyReason}
                title={applyReason || "应用推荐参数"}
                aria-busy={pending?.kind === "apply"}
                onClick={() => send("apply")}
              >
                {pending?.kind === "apply"
                  ? "等待确认…"
                  : appliedReport === reportId
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
