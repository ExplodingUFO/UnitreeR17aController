import { useEffect, useMemo, useRef, useState } from "react";
import {
  api,
  ApiError,
  hasJointFeedback,
  isJointPositions,
  type JointPositions,
  type TeachingPreview,
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
  onStop: () => void;
}
type Pending = {
  kind: "start" | "finish" | "replay";
  id: string;
  generation: number;
  connection: string;
  session?: string;
};
type ReplaySample = { t_ms: number; q: JointPositions };
const states: Record<TeachingStatus["state"], string> = {
  holding: "保持中",
  dragging: "拖动中",
  entering: "进入拖动",
  leaving: "恢复保持",
  finishing: "保存中",
  replaying: "回放中",
  approaching: "接近起点",
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
  const [pending, setPendingState] = useState<Pending | null>(null);
  const pendingRef = useRef<Pending | null>(null);
  const owned = useRef<Pending | null>(null);
  const closing = useRef(false);
  const dialog = useRef<HTMLElement>(null);
  const source = useRef<"pointer" | "space" | null>(null);
  const [pressed, setPressed] = useState(false);
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
    release: () => {},
    press: (_source: "pointer" | "space") => {},
    close: () => {},
    stop: () => {},
  });
  const s = view.snapshot;
  const teaching = s.teaching;
  const recording = !!teaching?.recording;
  const running = !!teaching && !recording;
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
  const available =
    view.online &&
    view.hasControl &&
    s.enabled &&
    hasJointFeedback(view) &&
    s.phase === "ready" &&
    !s.action &&
    !teaching &&
    !blocked &&
    !pending;
  const canDrag =
    open &&
    page === "record" &&
    recording &&
    teaching.state !== "finishing" &&
    view.online &&
    view.hasControl &&
    s.enabled &&
    !pending;
  const unavailable = !view.online
    ? "未连接"
    : !view.hasControl
      ? "未取得控制权"
      : !s.enabled
        ? "未使能"
        : !hasJointFeedback(view)
          ? "等待反馈"
          : !sameSide
            ? `请选择${s.side === "right" ? "右" : "左"}臂轨迹`
            : "";

  function setPending(value: Pending | null) {
    pendingRef.current = value;
    setPendingState(value);
  }
  function release() {
    source.current = null;
    setPressed(false);
    void api.releaseTeachingDrag("teaching_released").catch((e) => {
      if (current.current.open) setError(messageOf(e));
    });
  }
  function press(input: "pointer" | "space") {
    if (!canDrag || source.current) return;
    source.current = input;
    setPressed(true);
    api.pressTeachingDrag();
  }
  function stop() {
    release();
    closing.current = false;
    setPending(null);
    if (
      owned.current &&
      api.view.snapshot.teaching?.request_id !== owned.current.id &&
      resultFor(api.view, owned.current.id)?.status !== "accepted"
    )
      owned.current = null;
    setFeedback("停止中");
    setError("");
    setPlaying(false);
    current.current.onStop();
  }
  function complete(request: Pending, actualMessage = "") {
    setPending(null);
    setError("");
    if (request.kind === "finish") {
      owned.current = null;
      const latest = api.view.snapshot.latest_teaching;
      const outcome =
        latest && latest.id === request.session ? latest.outcome : "completed";
      const label =
        outcome === "completed"
          ? "轨迹已保存"
          : outcome === "failed"
            ? "示教失败，轨迹已保存"
            : "示教已停止，轨迹已保存";
      const message =
        actualMessage && actualMessage !== label
          ? `${label} · ${actualMessage}`
          : label;
      setFeedback(message);
      setError(outcome === "failed" ? message : "");
      setPage("paths");
      if (latest) setSelected(latest.id);
      setRevision((value) => value + 1);
      if (closing.current) {
        closing.current = false;
        if (outcome !== "failed") current.current.onClose();
      }
    } else if (!api.view.snapshot.teaching) {
      owned.current = null;
      setFeedback(
        actualMessage ||
          (request.kind === "replay" ? "回放完成" : "录制已结束"),
      );
    }
  }
  function submit(kind: Pending["kind"]) {
    if (pendingRef.current) return;
    const request: Pending = {
      kind,
      id: crypto.randomUUID(),
      generation: s.generation,
      connection: connectionOf(view),
      session: teaching?.id,
    };
    const params =
      kind === "start"
        ? { name: name.trim(), transition_s: Number(transition) }
        : kind === "replay"
          ? {
              trajectory_id: selected,
              speed_scale: Number(speed),
              smoothing_s: Number(smoothing),
            }
          : {};
    if (kind !== "finish") owned.current = request;
    if (kind === "replay") {
      replaySamples.current = [];
      replayPreview.current = preview;
      replayId.current = selected;
      replaySeq.current = -1;
      setSampleRevision((value) => value + 1);
      setPlaying(false);
    }
    setPending(request);
    setFeedback("");
    setError("");
    source.current = null;
    setPressed(false);
    const send = async () => {
      if (kind === "finish")
        void api
          .releaseTeachingDrag("teaching_finished")
          .catch((e) => api.fail(e));
      if (pendingRef.current !== request) return;
      await api.command(
        `teach_${kind}`,
        params,
        request.id,
        request.generation,
      );
    };
    void send().catch(async (cause) => {
      // A terminal state may arrive before the HTTP response, including an error.
      if (pendingRef.current !== request) return;
      const known = resultFor(api.view, request.id);
      if (known?.status === "completed") {
        complete(request, known.message);
        return;
      }
      if (known?.status === "accepted") return;
      if (known) {
        if (
          kind === "finish" &&
          api.view.snapshot.latest_teaching?.id === request.session &&
          !api.view.snapshot.teaching
        ) {
          complete(request, known.message);
          return;
        }
        setPending(null);
        closing.current = false;
        if (kind !== "finish") owned.current = null;
        setError(known.message || "操作已结束");
        return;
      }
      if (
        kind !== "finish" &&
        api.view.snapshot.teaching?.request_id === request.id
      ) {
        setPending(null);
        owned.current = {
          ...request,
          generation: api.view.snapshot.generation,
        };
        return;
      }
      setPending(null);
      closing.current = false;
      setError(messageOf(cause));
      if (!(cause instanceof ApiError && cause.notQueued)) {
        source.current = null;
        setPressed(false);
        await api.hostStop("teaching_request_failed").catch((e) => {
          if (current.current.open) setError(messageOf(e));
        });
      }
      if (kind !== "finish") owned.current = null;
    });
  }
  function close() {
    release();
    setPlaying(false);
    if (pendingRef.current?.kind === "finish") {
      closing.current = true;
      setFeedback("保存后关闭");
    } else if (recording && !pendingRef.current) {
      closing.current = true;
      submit("finish");
    } else if (teaching || pendingRef.current || owned.current) {
      setPending(null);
      owned.current = null;
      void api.hostStop("teaching_dialog_closed").then(
        () => current.current.onClose(),
        (e) => setError(messageOf(e)),
      );
    } else {
      current.current.onClose();
    }
  }
  actions.current = { release, press, close, stop };

  useEffect(() => {
    const request = pendingRef.current ?? owned.current;
    if (!request) return;
    const result = resultFor(view, request.id);
    const terminal = result?.status !== "accepted" ? result : null;
    if (terminal) {
      if (
        request.kind === "start" &&
        terminal.status === "completed" &&
        teaching?.request_id === request.id &&
        teaching.recording
      ) {
        if (pendingRef.current?.id === request.id) setPending(null);
        return;
      }
      if (
        terminal.status === "completed" ||
        (request.kind === "finish" &&
          s.latest_teaching?.id === request.session &&
          !teaching)
      )
        complete(request, terminal.message);
      else {
        setPending(null);
        closing.current = false;
        if (request.kind !== "finish") owned.current = null;
        setFeedback(terminal.message || "操作已结束");
        setError(
          ["failed", "rejected"].includes(terminal.status)
            ? terminal.message || "操作未完成"
            : "",
        );
      }
      return;
    }
    if (
      request.kind !== "finish" &&
      (teaching?.request_id === request.id ||
        s.action?.request_id === request.id)
    ) {
      setPending(null);
      if (owned.current?.id === request.id)
        owned.current = {
          ...request,
          generation: s.generation,
          session: teaching?.id ?? request.session,
        };
      return;
    }
    if (
      request.kind === "finish" &&
      s.latest_teaching?.id === request.session &&
      !teaching
    ) {
      complete(request);
      return;
    }
    if (
      !view.online ||
      !view.hasControl ||
      connectionOf(view) !== request.connection
    ) {
      setPending(null);
      owned.current = null;
      closing.current = false;
      actions.current.release();
      setError(!view.online ? "连接已断开" : "操作已结束");
    }
  }, [view, pending?.id]);

  useEffect(() => {
    if (!open) return;
    closing.current = false;
    const previous = document.activeElement;
    dialog.current?.focus();
    const keydown = (event: KeyboardEvent) => {
      if (!dialog.current?.contains(document.activeElement)) return;
      if (event.key === "Escape") {
        event.preventDefault();
        event.stopPropagation();
        const active =
          current.current.view.snapshot.teaching || pendingRef.current;
        if (active) actions.current.stop();
        else actions.current.close();
      } else if (event.code === "Space") {
        if (event.altKey || event.ctrlKey || event.metaKey) return;
        const target = event.target;
        if (
          target instanceof Element &&
          (target.closest(
            "input, textarea, select, [contenteditable], summary",
          ) ||
            (target.closest("button") && !target.closest(".teaching-drag")))
        )
          return;
        if (event.repeat) {
          if (source.current === "space") event.preventDefault();
          return;
        }
        event.preventDefault();
        actions.current.press("space");
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
    const keyup = (event: KeyboardEvent) => {
      if (event.code === "Space" && source.current === "space") {
        event.preventDefault();
        actions.current.release();
      }
    };
    const pointerup = () => {
      if (source.current === "pointer") actions.current.release();
    };
    const releaseAll = () => actions.current.release();
    const visibility = () => {
      if (document.hidden) releaseAll();
    };
    window.addEventListener("keydown", keydown, true);
    window.addEventListener("keyup", keyup, true);
    window.addEventListener("pointerup", pointerup);
    window.addEventListener("pointercancel", releaseAll);
    window.addEventListener("blur", releaseAll);
    document.addEventListener("visibilitychange", visibility);
    return () => {
      window.removeEventListener("keydown", keydown, true);
      window.removeEventListener("keyup", keyup, true);
      window.removeEventListener("pointerup", pointerup);
      window.removeEventListener("pointercancel", releaseAll);
      window.removeEventListener("blur", releaseAll);
      document.removeEventListener("visibilitychange", visibility);
      source.current = null;
      setPressed(false);
      void api
        .releaseTeachingDrag("teaching_dialog_unmounted")
        .catch((e) => api.fail(e));
      const active =
        current.current.view.snapshot.teaching ||
        pendingRef.current ||
        owned.current;
      if (active) {
        setPending(null);
        owned.current = null;
        void api
          .hostStop("teaching_dialog_unmounted")
          .catch((e) => api.fail(e));
      }
      if (previous instanceof HTMLElement && previous.isConnected)
        previous.focus();
    };
  }, [open]);

  useEffect(() => {
    if (pressed && !canDrag) actions.current.release();
  }, [canDrag, teaching, pending, pressed]);

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
          <h2 id="teaching-title">示教</h2>
          <span>{s.side === "right" ? "右臂" : "左臂"}</span>
          <span
            className="teaching-state"
            data-recording={recording}
            role="status"
          >
            {teaching ? states[teaching.state] : "待命"}
          </span>
          <button
            className="calibration-close"
            aria-label="关闭示教"
            onClick={close}
          >
            ×
          </button>
        </header>
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
                release();
                setPlaying(false);
                setPage(id);
              }}
            >
              {label}
              {id === "record" && recording && <i />}
            </button>
          ))}
          <small>
            {page === "record"
              ? "重力补偿 + 阻尼"
              : `${summaries.length} 条轨迹`}
          </small>
        </nav>
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
                    disabled={recording || !!pending}
                    onChange={(e) => setName(e.target.value)}
                  />
                </label>
                <dl className="teaching-stats">
                  <div>
                    <dt>录制时长</dt>
                    <dd>{clock(recording ? teaching.duration_ms : 0)}</dd>
                  </div>
                  <div>
                    <dt>采样</dt>
                    <dd>
                      {recording ? teaching.sample_count.toLocaleString() : "—"}
                    </dd>
                  </div>
                </dl>
                <button
                  className="teaching-drag"
                  aria-label="按住拖动"
                  aria-pressed={pressed}
                  disabled={!canDrag}
                  onPointerDown={(event) => {
                    if (event.button !== 0 || !canDrag) return;
                    event.preventDefault();
                    event.currentTarget.focus();
                    event.currentTarget.setPointerCapture(event.pointerId);
                    press("pointer");
                  }}
                  onPointerUp={release}
                  onPointerCancel={release}
                  onLostPointerCapture={release}
                  onContextMenu={(event) => event.preventDefault()}
                >
                  <span className="teaching-hand" aria-hidden="true">
                    ↔
                  </span>
                  <strong>{pressed ? "拖动中" : "按住拖动"}</strong>
                  <small>松开保持 · 空格同效</small>
                </button>
                <details className="teaching-settings">
                  <summary>录制设置</summary>
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
                      disabled={recording || !!pending}
                      onChange={(e) => setTransition(e.target.value)}
                    />
                  </label>
                </details>
                <button
                  className="primary teaching-main-action"
                  disabled={
                    recording
                      ? !!pending ||
                        teaching.state === "finishing" ||
                        !view.hasControl
                      : !available || !transitionValid
                  }
                  onClick={() => submit(recording ? "finish" : "start")}
                >
                  {pending?.kind === "start"
                    ? "开始中…"
                    : pending?.kind === "finish" ||
                        teaching?.state === "finishing"
                      ? "保存中…"
                      : recording
                        ? "结束并保存"
                        : "开始录制"}
                </button>
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
                        {item.side === "right" ? "右臂" : "左臂"} ·{" "}
                        {clock(item.duration_ms)} ·{" "}
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
                    disabled={running || !!pending}
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
                      disabled={running || !!pending}
                      onChange={(e) => setSmoothing(e.target.value)}
                    />
                  </label>
                </details>
                <button
                  className="primary teaching-main-action"
                  disabled={
                    !available || !preview || !optionsValid || !sameSide
                  }
                  onClick={() => submit("replay")}
                >
                  {pending?.kind === "replay"
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
                  接近 {clock(preview.approach_duration_ms)} · 全程{" "}
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
              (pending
                ? pending.kind === "finish"
                  ? "保存中…"
                  : pending.kind === "replay"
                    ? "准备回放…"
                    : "开始录制…"
                : feedback) ||
              (teaching
                ? `${states[teaching.state]} · ${clock(teaching.duration_ms)}`
                : unavailable) ||
              (preview
                ? `实际倍速 ${preview.effective_speed_scale.toFixed(2)}×`
                : "")}
          </span>
          {running && (
            <progress aria-label="回放进度" max="1" value={teaching.progress} />
          )}
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
