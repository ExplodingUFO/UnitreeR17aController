import { useEffect, useMemo, useRef, useState } from "react";
import {
  api,
  ApiError,
  hasJointFeedback,
  isJointPositions,
  type JointPositions,
  type TeachingPreview,
  type TeachingMode,
  type TeachingStatus,
  type TeachingSummary,
  type TeachingTrajectory,
  type View,
} from "./api";
import RobotScene, { type PreviewStage } from "./RobotScene";

interface Props {
  view: View;
  open: boolean;
  blocked: boolean;
  onClose: () => void;
  onFold: () => void;
  onStop: () => void;
}
type Lane = "record" | "mode" | "replay";
type Pending = {
  kind: "start" | "finish" | "replay" | "mode";
  id: string;
  connection: string;
  session?: string;
  enabled?: boolean;
  waiting: boolean;
};
type ReplaySample = { t_ms: number; q: JointPositions };
const states: Record<TeachingStatus["state"], string> = {
  recording: "录制中",
  finishing: "保存中",
  replaying: "回放中",
  approaching: "接近起点",
};
const modeStates: Record<TeachingMode["state"], string> = {
  entering: "切换拖动示教",
  dragging: "可手动拖动",
  leaving: "恢复位置保持",
  holding: "位置保持",
};
const clock = (ms: number) => `${(ms / 1000).toFixed(1)} s`;
const nameOf = (item: TeachingSummary) => item.name || "未命名轨迹";
const connectionOf = (view: View) =>
  `${view.baseUrl}:${view.connectionEpoch ?? 0}:${view.snapshot.side}:${view.snapshot.port ?? ""}`;
function resultFor(view: View, id: string) {
  const results = [
    ...(view.snapshot.results ?? []),
    view.snapshot.last_result,
  ].filter((result) => result?.request_id === id);
  return results.find((result) => result!.status !== "accepted") ?? results[0];
}
const messageOf = (error: unknown) =>
  error instanceof Error ? error.message : String(error);
const mergeLatest = (
  items: TeachingSummary[],
  latest?: TeachingSummary | null,
) =>
  latest ? [latest, ...items.filter((item) => item.id !== latest.id)] : items;

export default function TeachingPanel({
  view,
  open,
  blocked,
  onClose,
  onFold,
  onStop,
}: Props) {
  const [page, setPage] = useState<"record" | "paths">("record");
  const [name, setName] = useState("");
  const [transition, setTransition] = useState("0.4");
  const [speed, setSpeed] = useState("1");
  const [smoothing, setSmoothing] = useState("0.12");
  const [items, setItems] = useState<TeachingSummary[]>([]);
  const [selected, setSelected] = useState<string | null>(null);
  const [trajectory, setTrajectory] = useState<TeachingTrajectory | null>(null);
  const [plan, setPlan] = useState<{
    preview: TeachingPreview;
    key: string;
    generation: number;
  } | null>(null);
  const [loading, setLoading] = useState(false);
  const [listLoading, setListLoading] = useState(false);
  const [listError, setListError] = useState("");
  const [previewError, setPreviewError] = useState("");
  const [feedback, setFeedback] = useState("");
  const [error, setError] = useState("");
  const [requests, setRequests] = useState<Partial<Record<Lane, Pending>>>({});
  const requestsRef = useRef(requests);
  const closing = useRef(false);
  const folded = useRef(false);
  const cleanupRequest = useRef<Promise<void> | null>(null);
  const dialog = useRef<HTMLElement>(null);
  const [revision, setRevision] = useState(0);
  const [stage, setStage] = useState<PreviewStage>("teaching:all");
  const [progress, setProgress] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [axis, setAxis] = useState(0);
  const replaySamples = useRef<ReplaySample[]>([]);
  const replayPreview = useRef<TeachingPreview | null>(null);
  const replayId = useRef<string | null>(null);
  const replaySeq = useRef(-1);
  const [sampleRevision, setSampleRevision] = useState(0);
  const [downloading, setDownloading] = useState(false);
  const current = useRef({ view, open, onClose, onStop });
  current.current = { view, open, onClose, onStop };
  const actions = useRef({
    close: () => {},
    stop: () => {},
    cleanup: () => {},
  });
  const s = view.snapshot;
  const teaching = s.teaching;
  const capture = s.teaching_recording;
  const mode = s.teaching_mode;
  const recording = !!capture;
  const running = !!teaching;
  const guiding =
    !!mode && mode.state !== "leaving" && mode.state !== "holding";
  const recordPending = requests.record;
  const replayPending = requests.replay?.waiting ? requests.replay : null;
  const modePending = requests.mode;
  const summaries = useMemo(
    () => mergeLatest(items, s.latest_teaching),
    [items, s.latest_teaching],
  );
  const summary = summaries.find((item) => item.id === selected);
  const sameSide = !summary || summary.side === s.side;
  const optionsValid =
    speed.trim() !== "" &&
    Number.isFinite(Number(speed)) &&
    Number(speed) > 0 &&
    smoothing.trim() !== "" &&
    Number.isFinite(Number(smoothing)) &&
    Number(smoothing) >= 0;
  const transitionValid =
    transition.trim() !== "" &&
    Number.isFinite(Number(transition)) &&
    Number(transition) > 0;
  const previewGeneration =
    running && plan?.preview.trajectory_id === teaching.trajectory_id
      ? plan.generation
      : s.generation;
  const previewKey = `${selected}:${Number(speed)}:${Number(smoothing)}:${connectionOf(view)}:${previewGeneration}`;
  const preview = plan?.key === previewKey ? plan.preview : null;
  const controlled = view.online && view.hasControl && hasJointFeedback(view);
  const canRecord = controlled && !recordPending;
  const canGuide = controlled && s.enabled && !s.action && !running && !blocked;
  const canReplay =
    controlled &&
    s.enabled &&
    !s.action &&
    !mode &&
    !blocked &&
    !requests.replay;
  const unavailable = !view.online
    ? "未连接"
    : !view.hasControl
      ? "未取得控制权"
      : !hasJointFeedback(view)
        ? "等待反馈"
        : "";

  function setRequest(lane: Lane, request?: Pending) {
    const next = { ...requestsRef.current };
    if (request) next[lane] = request;
    else delete next[lane];
    requestsRef.current = next;
    setRequests(next);
  }
  function stop() {
    closing.current = false;
    const record = requestsRef.current.record;
    requestsRef.current = record ? { record } : {};
    setRequests(requestsRef.current);
    setFeedback("停止中");
    setError("");
    setPlaying(false);
    current.current.onStop();
  }
  function closeAfterSave() {
    closing.current = false;
    const state = api.view.snapshot;
    if (
      state.teaching_mode ||
      state.teaching ||
      requestsRef.current.mode ||
      requestsRef.current.replay
    ) {
      void api.hostStop("teaching_dialog_closed").then(
        () => current.current.onClose(),
        (cause) => setError(messageOf(cause)),
      );
    } else current.current.onClose();
  }
  function finishSaved(request: Pending, actualMessage = "") {
    setRequest("record");
    const latest = api.view.snapshot.latest_teaching;
    const saved = latest?.id === request.session ? latest : null;
    const label =
      saved?.outcome === "failed"
        ? "录制中断，已保存采集部分"
        : saved?.outcome === "stopped"
          ? "录制已停止并保存"
          : "轨迹已保存";
    setFeedback(label);
    setError(saved?.outcome === "failed" ? actualMessage || label : "");
    setPage("paths");
    if (saved) setSelected(saved.id);
    setRevision((value) => value + 1);
    if (closing.current) closeAfterSave();
  }
  function holdPosition() {
    if (requestsRef.current.mode?.enabled) {
      // The enter request may have already changed generation while the
      // corresponding state is still in transit. Stop adopts current hold
      // without depending on that older generation; recording stays active.
      setRequest("mode");
      setFeedback("恢复位置保持");
      void api
        .hostStop("teaching_mode_cancelled")
        .catch((cause) => setError(messageOf(cause)));
    } else if (mode) submit("mode", false);
  }
  function submit(kind: Pending["kind"], enabled?: boolean) {
    const lane: Lane =
      kind === "mode" ? "mode" : kind === "replay" ? "replay" : "record";
    if (lane !== "mode" && requestsRef.current[lane]?.waiting) return;
    const request: Pending = {
      kind,
      id: crypto.randomUUID(),
      connection: connectionOf(view),
      session: capture?.id,
      enabled,
      waiting: true,
    };
    const params =
      kind === "start"
        ? { name: name.trim() }
        : kind === "mode"
          ? { enabled, transition_s: Number(transition) }
          : kind === "replay"
            ? {
                trajectory_id: selected,
                speed_scale: Number(speed),
                smoothing_s: Number(smoothing),
              }
            : { recording_request_id: capture?.request_id };
    if (kind === "replay") {
      replaySamples.current = [];
      replayPreview.current = preview;
      replayId.current = selected;
      replaySeq.current = -1;
      setSampleRevision((value) => value + 1);
      setPlaying(false);
    }
    setRequest(lane, request);
    setFeedback("");
    setError("");
    void api
      .command(`teach_${kind}`, params, request.id, s.generation)
      .catch(async (cause) => {
        // Snapshot results can precede the HTTP acknowledgement. A response for
        // an older intent never changes a newer operation or the current mode.
        if (requestsRef.current[lane]?.id !== request.id) return;
        const state = api.view.snapshot;
        const known = resultFor(api.view, request.id);
        if (
          kind === "finish" &&
          state.latest_teaching?.id === request.session &&
          !state.teaching_recording
        ) {
          finishSaved(request, known?.message);
          return;
        }
        if (
          known ||
          state.teaching_recording?.request_id === request.id ||
          state.teaching_mode?.request_id === request.id ||
          state.teaching?.request_id === request.id
        )
          return;
        setRequest(lane);
        closing.current = false;
        setError(messageOf(cause));
        if (!(cause instanceof ApiError && cause.notQueued))
          await api.hostStop("teaching_request_failed").catch((error) => {
            if (current.current.open) setError(messageOf(error));
          });
      });
  }
  function close() {
    setPlaying(false);
    const state = api.view.snapshot;
    if (!view.online || !view.hasControl) {
      const pendingStart =
        requestsRef.current.record?.kind === "start"
          ? requestsRef.current.record.id
          : undefined;
      requestsRef.current = {};
      setRequests({});
      void api.closeTeaching(pendingStart).then(
        () => current.current.onClose(),
        (cause) => setError(messageOf(cause)),
      );
    } else if (requestsRef.current.record?.kind === "finish") {
      closing.current = true;
      setFeedback("保存后关闭");
    } else if (state.teaching_recording && !requestsRef.current.record) {
      closing.current = true;
      submit("finish");
    } else if (
      state.teaching_mode ||
      state.teaching ||
      Object.keys(requestsRef.current).length
    ) {
      const pendingStart =
        requestsRef.current.record?.kind === "start"
          ? requestsRef.current.record.id
          : undefined;
      requestsRef.current = {};
      setRequests({});
      void api.closeTeaching(pendingStart).then(
        () => current.current.onClose(),
        (cause) => setError(messageOf(cause)),
      );
    } else current.current.onClose();
  }
  function cleanup() {
    if (cleanupRequest.current) return;
    const state = api.view.snapshot;
    if (
      !state.teaching_mode &&
      !state.teaching_recording &&
      !state.teaching &&
      !Object.keys(requestsRef.current).length
    )
      return;
    const pendingStart =
      requestsRef.current.record?.kind === "start"
        ? requestsRef.current.record.id
        : undefined;
    requestsRef.current = {};
    setRequests({});
    cleanupRequest.current = api
      .closeTeaching(pendingStart)
      .catch((error) => api.fail(error));
  }
  function fold() {
    folded.current = true;
    setPlaying(false);
    onFold();
  }
  actions.current = { close, stop, cleanup };

  useEffect(() => () => actions.current.cleanup(), []);

  useEffect(() => {
    for (const [key, request] of Object.entries(requestsRef.current)) {
      const lane = key as Lane;
      const result = resultFor(view, request.id);
      if (
        request.kind === "finish" &&
        s.latest_teaching?.id === request.session &&
        !capture
      ) {
        finishSaved(request, result?.message);
        continue;
      }
      if (result && result.status !== "accepted") {
        setRequest(lane);
        if (request.kind === "finish" && result.status === "completed")
          finishSaved(request, result.message);
        else {
          setFeedback(
            result.message ||
              (request.kind === "replay" ? "回放完成" : "操作已完成"),
          );
          setError(
            ["failed", "rejected"].includes(result.status)
              ? result.message || "操作未完成"
              : "",
          );
          if (request.kind === "finish") closing.current = false;
        }
        continue;
      }
      const observed =
        request.kind === "start"
          ? capture?.request_id === request.id
          : request.kind === "mode"
            ? mode?.request_id === request.id ||
              (!!result && !request.enabled && !mode)
            : request.kind === "replay"
              ? teaching?.request_id === request.id ||
                s.action?.request_id === request.id
              : false;
      if (observed) {
        if (request.kind === "replay") {
          if (request.waiting) setRequest(lane, { ...request, waiting: false });
        } else setRequest(lane);
        continue;
      }
      if (
        !view.online ||
        !view.hasControl ||
        connectionOf(view) !== request.connection
      ) {
        setRequest(lane);
        closing.current = false;
        setError(!view.online ? "连接已断开" : "控制已结束");
      }
    }
  }, [view, requests]);

  useEffect(() => {
    if (!open) return;
    closing.current = false;
    folded.current = false;
    cleanupRequest.current = null;
    const previous = document.activeElement;
    dialog.current
      ?.querySelector<HTMLElement>("button:not(:disabled)")
      ?.focus();
    const keydown = (event: KeyboardEvent) => {
      if (!dialog.current?.contains(document.activeElement)) return;
      if (event.key === "Escape") {
        event.preventDefault();
        event.stopPropagation();
        const state = current.current.view.snapshot;
        if (
          state.teaching_mode ||
          state.teaching_recording ||
          state.teaching ||
          Object.keys(requestsRef.current).length
        )
          actions.current.stop();
        else actions.current.close();
      } else if (event.key === "Tab") {
        const controls = Array.from(
          dialog.current.querySelectorAll<HTMLElement>(
            'button:not(:disabled), input:not(:disabled), select:not(:disabled), summary, a[href], [tabindex="0"]',
          ),
        ).filter((element) => element.getClientRects().length > 0);
        const first = controls[0],
          last = controls.at(-1);
        if (
          (!event.shiftKey &&
            (document.activeElement === last ||
              document.activeElement === dialog.current)) ||
          (event.shiftKey &&
            (document.activeElement === first ||
              document.activeElement === dialog.current))
        ) {
          event.preventDefault();
          (event.shiftKey ? last : first)?.focus();
        }
      }
    };
    window.addEventListener("keydown", keydown, true);
    return () => {
      window.removeEventListener("keydown", keydown, true);
      if (!folded.current) actions.current.cleanup();
      if (previous instanceof HTMLElement && previous.isConnected)
        previous.focus();
    };
  }, [open]);

  useEffect(() => {
    if (!open || !view.online || page !== "paths") return;
    const controller = new AbortController();
    setListLoading(true);
    setListError("");
    void api
      .listTeachings(controller.signal)
      .then(
        (saved) => {
          if (controller.signal.aborted) return;
          setItems(saved);
          setSelected((id) => id ?? saved[0]?.id ?? null);
        },
        (e) => {
          if (!controller.signal.aborted) setListError(messageOf(e));
        },
      )
      .finally(() => {
        if (!controller.signal.aborted) setListLoading(false);
      });
    return () => controller.abort();
  }, [open, page, view.online, view.connectionEpoch, revision]);

  useEffect(() => {
    if (s.latest_teaching) {
      setItems((saved) => mergeLatest(saved, s.latest_teaching));
      setSelected((id) => id ?? s.latest_teaching!.id);
    }
  }, [s.latest_teaching?.id]);

  useEffect(() => {
    if (!open || page !== "paths" || !selected || !view.online) return;
    const controller = new AbortController();
    setTrajectory(null);
    void api.readTeaching(selected, controller.signal).then(
      (value) => {
        if (!controller.signal.aborted) setTrajectory(value);
      },
      (e) => {
        if (!controller.signal.aborted) setError(messageOf(e));
      },
    );
    return () => controller.abort();
  }, [open, page, selected, view.online, view.connectionEpoch, revision]);

  useEffect(() => {
    setPlaying(false);
    setProgress(0);
    setPlan(null);
    setPreviewError("");
    setLoading(false);
    if (
      !open ||
      page !== "paths" ||
      !selected ||
      !view.online ||
      !optionsValid ||
      !sameSide
    )
      return;
    const controller = new AbortController();
    setLoading(true);
    const timer = setTimeout(() => {
      void api
        .previewTeaching(
          selected,
          { speed_scale: Number(speed), smoothing_s: Number(smoothing) },
          controller.signal,
        )
        .then(
          (value) => {
            if (controller.signal.aborted) return;
            setPlan({
              preview: value,
              key: previewKey,
              generation: previewGeneration,
            });
          },
          (e) => {
            if (!controller.signal.aborted) setPreviewError(messageOf(e));
          },
        )
        .finally(() => {
          if (!controller.signal.aborted) setLoading(false);
        });
    }, 180);
    return () => {
      clearTimeout(timer);
      controller.abort();
    };
  }, [open, page, previewKey, view.online, optionsValid, sameSide, revision]);

  const stageRange = useMemo(() => {
    if (!preview) return [0, 0];
    return stage === "teaching:approach"
      ? [0, preview.approach_duration_ms]
      : stage === "teaching:path"
        ? [preview.approach_duration_ms, preview.duration_ms]
        : [0, preview.duration_ms];
  }, [preview, stage]);
  useEffect(() => {
    if (!playing || !open || !preview) return;
    if (stageRange[1] <= stageRange[0]) {
      setProgress(1);
      setPlaying(false);
      return;
    }
    let frame = 0;
    const started = performance.now();
    const initialProgress = progress;
    const tick = (time: number) => {
      const next =
        initialProgress +
        (time - started) / Math.max(1, stageRange[1] - stageRange[0]);
      setProgress(Math.min(1, next));
      if (next >= 1) setPlaying(false);
      else frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frame);
  }, [playing, open, preview, stageRange]);

  useEffect(() => {
    if (
      !running ||
      !preview ||
      teaching.trajectory_id !== replayId.current ||
      teaching.trajectory_id !== preview.trajectory_id ||
      replaySeq.current === s.seq
    )
      return;
    const q = s.joints.map((joint) => joint.q);
    if (!isJointPositions(q)) return;
    replaySeq.current = s.seq;
    replaySamples.current.push({
      t_ms: teaching.progress * preview.duration_ms,
      q,
    });
    setSampleRevision((value) => value + 1);
  }, [s.seq, running, teaching, preview]);

  async function download() {
    if (!selected || downloading) return;
    setDownloading(true);
    try {
      const blob = await api.downloadTeaching(selected);
      const url = URL.createObjectURL(blob);
      const link = document.createElement("a");
      link.href = url;
      link.download = `${(summary ? nameOf(summary) : "轨迹").replace(/[/\\:*?"<>|]/g, "_")}.json`;
      link.click();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
    } catch (e) {
      setError(messageOf(e));
    } finally {
      setDownloading(false);
    }
  }

  if (!open) return null;
  const cursorMs = stageRange[0] + progress * (stageRange[1] - stageRange[0]);
  return (
    <div className="teaching-backdrop">
      <section
        ref={dialog}
        id="teaching-dialog"
        className="teaching-panel"
        role="dialog"
        aria-modal="true"
        aria-labelledby="teaching-title"
        tabIndex={-1}
        data-page={page}
      >
        <header className="teaching-header">
          <h2 id="teaching-title">示教与录制</h2>
          <span>{s.side === "right" ? "右臂" : "左臂"}</span>
          <span
            className="teaching-state"
            data-recording={recording}
            role="status"
          >
            {capture
              ? states[capture.state]
              : teaching
                ? states[teaching.state]
                : "待命"}
          </span>
          <button className="teaching-fold" onClick={fold}>
            收起
          </button>
          <button
            className="calibration-close"
            aria-label="关闭示教"
            onClick={close}
          >
            ×
          </button>
        </header>
        <div className="teaching-toolbar">
          <nav className="teaching-tabs" aria-label="示教视图">
            {(
              [
                ["record", "录制"],
                ["paths", "轨迹"],
              ] as const
            ).map(([id, label]) => (
              <button
                key={id}
                aria-pressed={page === id}
                onClick={() => {
                  setPlaying(false);
                  setPage(id);
                }}
              >
                {label}
              </button>
            ))}
          </nav>
          <div className="teaching-controls">
            <div className="teaching-mode" role="group" aria-label="运动模式">
              <button
                aria-pressed={!(modePending?.enabled ?? guiding)}
                aria-busy={modePending?.enabled === false}
                disabled={!view.online || !view.hasControl}
                onClick={holdPosition}
              >
                位置保持
              </button>
              <button
                aria-pressed={modePending?.enabled ?? guiding}
                aria-busy={modePending?.enabled === true}
                disabled={!canGuide || !transitionValid}
                onClick={() => {
                  if (!guiding && !modePending?.enabled) submit("mode", true);
                }}
              >
                拖动示教
              </button>
            </div>
            <span role="status">
              {mode
                ? modeStates[mode.state]
                : modePending
                  ? "切换中…"
                  : s.action
                    ? "程序运动中"
                    : s.enabled
                      ? "保持当前位置"
                      : "未使能"}
              {guiding && <small>重力补偿与阻尼</small>}
            </span>
          </div>
        </div>
        <div className="teaching-body">
          <aside className="teaching-sidebar">
            {page === "record" ? (
              <>
                <label className="teaching-field">
                  <span>轨迹名称</span>
                  <input
                    aria-label="轨迹名称"
                    placeholder="未命名轨迹"
                    value={name}
                    disabled={recording || !!recordPending}
                    onChange={(e) => setName(e.target.value)}
                  />
                </label>
                <dl className="teaching-stats">
                  <div>
                    <dt>录制时长</dt>
                    <dd>{clock(capture?.duration_ms ?? 0)}</dd>
                  </div>
                  <div>
                    <dt>采样</dt>
                    <dd>
                      {capture ? capture.sample_count.toLocaleString() : "—"}
                    </dd>
                  </div>
                </dl>
                <p className="teaching-record-hint">
                  {recording
                    ? "正在记录实测位置，可切换模式或查看轨迹。"
                    : "拖动示教下可直接移动机械臂。录制记录七轴实测位置。"}
                </p>
                <details className="teaching-settings">
                  <summary>示教设置</summary>
                  <label className="teaching-field">
                    <span>
                      增益过渡 <small>s</small>
                    </span>
                    <input
                      aria-label="增益过渡秒数"
                      type="number"
                      step="any"
                      value={transition}
                      aria-invalid={!transitionValid}
                      disabled={!!modePending}
                      onChange={(e) => setTransition(e.target.value)}
                    />
                  </label>
                </details>
              </>
            ) : (
              <>
                <div className="teaching-list-heading">
                  <span>已保存</span>
                  <button
                    aria-label="刷新轨迹"
                    onClick={() => setRevision((value) => value + 1)}
                    disabled={listLoading}
                  >
                    ↻
                  </button>
                </div>
                <div className="teaching-list">
                  {summaries.map((item) => (
                    <button
                      key={item.id}
                      className="teaching-item"
                      aria-pressed={selected === item.id}
                      onClick={() => {
                        setSelected(item.id);
                        setError("");
                        setFeedback("");
                      }}
                    >
                      <b>{nameOf(item)}</b>
                      <span>
                        {item.side === "right" ? "右臂" : "左臂"}，{" "}
                        {clock(item.duration_ms)}，{" "}
                        {item.sample_count.toLocaleString()} 点
                      </span>
                      <small>
                        {new Date(item.created_at_ms).toLocaleString("zh-CN", {
                          month: "2-digit",
                          day: "2-digit",
                          hour: "2-digit",
                          minute: "2-digit",
                          hour12: false,
                        })}
                      </small>
                    </button>
                  ))}
                  {!summaries.length && (
                    <p className="teaching-empty">
                      {listLoading ? "读取中…" : "暂无轨迹"}
                    </p>
                  )}
                  {listError && <p className="teaching-error">{listError}</p>}
                </div>
                <label className="teaching-speed">
                  <span>回放倍速</span>
                  <input
                    aria-label="回放倍速"
                    type="number"
                    step="any"
                    value={speed}
                    aria-invalid={!optionsValid}
                    disabled={running || !!replayPending}
                    onChange={(e) => setSpeed(e.target.value)}
                  />
                  <span>×</span>
                </label>
                <details className="teaching-settings">
                  <summary>回放设置</summary>
                  <label className="teaching-field">
                    <span>
                      轨迹平滑 <small>s</small>
                    </span>
                    <input
                      aria-label="轨迹平滑秒数"
                      type="number"
                      step="any"
                      value={smoothing}
                      aria-invalid={
                        smoothing.trim() === "" ||
                        !Number.isFinite(Number(smoothing)) ||
                        Number(smoothing) < 0
                      }
                      disabled={running || !!replayPending}
                      onChange={(e) => setSmoothing(e.target.value)}
                    />
                  </label>
                </details>
                <button
                  className="primary teaching-main-action"
                  disabled={
                    !canReplay || !preview || !optionsValid || !sameSide
                  }
                  onClick={() => submit("replay")}
                >
                  {!!replayPending
                    ? "准备回放…"
                    : running
                      ? states[teaching.state]
                      : "回放轨迹"}
                </button>
              </>
            )}
          </aside>
          <div className="teaching-viewer">
            <div className="teaching-viewer-heading">
              <strong>
                {page === "record"
                  ? "实时姿态"
                  : summary
                    ? nameOf(summary)
                    : "3D 预览"}
              </strong>
              {page === "paths" && selected && (
                <button onClick={() => void download()} disabled={downloading}>
                  {downloading ? "导出中…" : "导出 JSON"}
                </button>
              )}
              {page === "paths" && preview && (
                <small>
                  接近 {clock(preview.approach_duration_ms)}，全程{" "}
                  {clock(preview.duration_ms)}
                </small>
              )}
            </div>
            <div className="teaching-scene">
              <RobotScene
                side={s.side}
                joints={s.joints}
                valid={hasJointFeedback(view)}
                source={page === "record" ? "关节反馈" : "轨迹预览"}
                preview={page === "paths" ? preview : null}
                previewProgress={progress}
                previewStage={stage}
              />
              {page === "paths" && (loading || previewError || !selected) && (
                <div className="teaching-scene-message" role="status">
                  {loading ? "生成预览…" : previewError || "选择轨迹预览"}
                </div>
              )}
            </div>
            {page === "record" ? (
              <div className="teaching-joints" aria-label="实测关节位置">
                {Array.from({ length: 7 }, (_, i) => (
                  <span key={i}>
                    J{i + 1}
                    <b>
                      {s.joints[i]?.q != null
                        ? `${((s.joints[i].q! * 180) / Math.PI).toFixed(1)}°`
                        : "—"}
                    </b>
                  </span>
                ))}
              </div>
            ) : (
              <>
                <div className="teaching-preview-controls">
                  <button
                    disabled={!preview}
                    onClick={() => {
                      if (progress >= 1) setProgress(0);
                      setPlaying((value) => !value);
                    }}
                  >
                    {playing ? "暂停预览" : "播放预览"}
                  </button>
                  <select
                    aria-label="预览阶段"
                    value={stage}
                    disabled={!preview}
                    onChange={(e) => {
                      setStage(e.target.value as PreviewStage);
                      setProgress(0);
                      setPlaying(false);
                    }}
                  >
                    <option value="teaching:all">全程</option>
                    <option value="teaching:approach">接近</option>
                    <option value="teaching:path">录制路径</option>
                  </select>
                  <input
                    aria-label="示教预览进度"
                    type="range"
                    min="0"
                    max="1"
                    step="0.001"
                    value={progress}
                    disabled={!preview}
                    onChange={(e) => {
                      setPlaying(false);
                      setProgress(Number(e.target.value));
                    }}
                  />
                  <output>{clock(cursorMs)}</output>
                </div>
                <TeachingChart
                  trajectory={trajectory?.id === selected ? trajectory : null}
                  preview={
                    replayId.current === selected &&
                    replaySamples.current.length
                      ? replayPreview.current
                      : preview
                  }
                  samples={
                    replayId.current === selected ? replaySamples.current : []
                  }
                  revision={sampleRevision}
                  axis={axis}
                  onAxis={setAxis}
                  cursorMs={cursorMs}
                />
              </>
            )}
          </div>
        </div>
        <footer className="teaching-footer">
          <span
            role="status"
            className={error || previewError ? "teaching-error" : ""}
          >
            {error ||
              previewError ||
              (recordPending
                ? recordPending.kind === "finish"
                  ? "保存中…"
                  : "开始录制…"
                : "") ||
              (replayPending ? "准备回放…" : "") ||
              feedback ||
              (capture
                ? `录制 ${clock(capture.duration_ms)}，${capture.sample_count.toLocaleString()} 点`
                : "") ||
              (teaching ? states[teaching.state] : "") ||
              unavailable ||
              (preview
                ? `实际倍速 ${preview.effective_speed_scale.toFixed(2)}×`
                : "")}
          </span>
          {running && (
            <progress aria-label="回放进度" max="1" value={teaching.progress} />
          )}
          <button
            className="primary teaching-record-action"
            aria-busy={!!recordPending || capture?.state === "finishing"}
            disabled={
              !!recordPending ||
              capture?.state === "finishing" ||
              (recording ? !view.online || !view.hasControl : !canRecord)
            }
            onClick={() => submit(recording ? "finish" : "start")}
          >
            {recordPending?.kind === "start"
              ? "开始中…"
              : recordPending?.kind === "finish" ||
                  capture?.state === "finishing"
                ? "保存中…"
                : recording
                  ? "结束并保存"
                  : "开始录制"}
          </button>
          <button className="calibration-stop" onClick={stop}>
            停止 <kbd>Esc</kbd>
          </button>
        </footer>
      </section>
    </div>
  );
}

function TeachingChart({
  trajectory,
  preview,
  samples,
  revision,
  axis,
  onAxis,
  cursorMs,
}: {
  trajectory: TeachingTrajectory | null;
  preview: TeachingPreview | null;
  samples: ReplaySample[];
  revision: number;
  axis: number;
  onAxis: (axis: number) => void;
  cursorMs: number;
}) {
  const chart = useMemo(() => {
    const duration = preview?.duration_ms ?? trajectory?.duration_ms ?? 1;
    const approach = preview?.approach_duration_ms ?? 0;
    const rawScale = trajectory?.duration_ms
      ? (duration - approach) / trajectory.duration_ms
      : 1;
    const series = [
      {
        key: "recorded",
        points:
          trajectory?.points.map((p) => [
            approach + p.t_ms * rawScale,
            (p.q[axis] * 180) / Math.PI,
          ]) ?? [],
      },
      {
        key: "command",
        points:
          preview?.points.map((p) => [
            p.t_ms,
            (p.joints[axis] * 180) / Math.PI,
          ]) ?? [],
      },
      {
        key: "feedback",
        points: samples.map((p) => [p.t_ms, (p.q[axis] * 180) / Math.PI]),
      },
    ];
    let minimum = Infinity,
      maximum = -Infinity;
    series.forEach((line) =>
      line.points.forEach(([, value]) => {
        minimum = Math.min(minimum, value);
        maximum = Math.max(maximum, value);
      }),
    );
    if (!Number.isFinite(minimum)) {
      minimum = -1;
      maximum = 1;
    }
    const padding = Math.max((maximum - minimum) * 0.12, 0.05);
    minimum -= padding;
    maximum += padding;
    const x = (t: number) => 44 + (580 * t) / Math.max(duration, 1);
    const y = (q: number) => 10 + (76 * (maximum - q)) / (maximum - minimum);
    return {
      duration,
      approach,
      minimum,
      maximum,
      x,
      y,
      paths: series.map((line) => {
        // Min/max buckets keep small excursions visible without a giant SVG.
        const stride = Math.max(1, Math.ceil(line.points.length / 580));
        const points: number[][] = [];
        for (let i = 0; i < line.points.length; i += stride) {
          const bucket = line.points.slice(i, i + stride);
          const low = bucket.reduce((a, b) => (a[1] < b[1] ? a : b));
          const high = bucket.reduce((a, b) => (a[1] > b[1] ? a : b));
          points.push(
            ...[bucket[0], low, high, bucket.at(-1)!].sort(
              (a, b) => a[0] - b[0],
            ),
          );
        }
        return {
          key: line.key,
          count: line.points.length,
          path: points
            .map(
              ([t, value], i) =>
                `${i ? "L" : "M"}${x(t).toFixed(2)},${y(value).toFixed(2)}`,
            )
            .join(" "),
        };
      }),
    };
  }, [trajectory, preview, samples, revision, axis]);
  return (
    <div className="teaching-chart">
      <div className="teaching-chart-heading">
        <select
          aria-label="曲线关节"
          value={axis}
          onChange={(e) => onAxis(Number(e.target.value))}
        >
          {Array.from({ length: 7 }, (_, i) => (
            <option key={i} value={i}>
              J{i + 1} · 位置 °
            </option>
          ))}
        </select>
        <div className="teaching-chart-legend">
          <span data-series="recorded">录制</span>
          <span data-series="command">规划指令</span>
          <span data-series="feedback">回放实测</span>
        </div>
      </div>
      <svg
        viewBox="0 0 640 108"
        preserveAspectRatio="none"
        role="img"
        aria-label={`J${axis + 1} 录制、规划指令与回放实测位置曲线`}
      >
        {chart.approach > 0 && (
          <rect
            className="teaching-chart-approach"
            x="44"
            y="8"
            width={chart.x(chart.approach) - 44}
            height="80"
          />
        )}
        {[
          chart.minimum,
          (chart.minimum + chart.maximum) / 2,
          chart.maximum,
        ].map((value, i) => (
          <g className="teaching-chart-grid" key={i}>
            <line x1="44" x2="624" y1={chart.y(value)} y2={chart.y(value)} />
            <text x="38" y={chart.y(value) + 3} textAnchor="end">
              {value.toFixed(1)}
            </text>
          </g>
        ))}
        {chart.paths.map((line) => (
          <path
            key={line.key}
            data-series={line.key}
            data-samples={line.count}
            d={line.path}
            fill="none"
            vectorEffect="non-scaling-stroke"
          />
        ))}
        <line
          className="teaching-chart-cursor"
          x1={chart.x(cursorMs)}
          x2={chart.x(cursorMs)}
          y1="8"
          y2="88"
        />
        <text className="teaching-chart-tick" x="44" y="103">
          0 s
        </text>
        <text className="teaching-chart-tick" x="624" y="103" textAnchor="end">
          {clock(chart.duration)}
        </text>
      </svg>
    </div>
  );
}
