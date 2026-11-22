import { useEffect, useRef, useState } from "react";
import { createPortal } from "react-dom";
import { api, type TraceSummary } from "./api";

const statusLabels: Record<TraceSummary["status"], string> = {
  recording: "采样中",
  writing: "写入中",
  complete: "已完成",
  failed: "失败",
};
const timeFormat = new Intl.DateTimeFormat("zh-CN", {
  month: "2-digit",
  day: "2-digit",
  hour: "2-digit",
  minute: "2-digit",
  second: "2-digit",
  hour12: false,
});
const errorText = (error: unknown) =>
  error instanceof Error ? error.message : "请求失败";

export default function TracePanel({
  online,
  connected,
  visible,
}: {
  online: boolean;
  connected: boolean;
  visible: boolean;
}) {
  const [open, setOpen] = useState(false);
  const [duration, setDuration] = useState("10");
  const [traces, setTraces] = useState<TraceSummary[]>([]);
  const [listState, setListState] = useState<"loading" | "ready" | "error">(
    "loading",
  );
  const [listError, setListError] = useState("");
  const [actionError, setActionError] = useState("");
  const [notice, setNotice] = useState("");
  const [busy, setBusy] = useState<"start" | "stop" | null>(null);
  const [downloading, setDownloading] = useState<string | null>(null);
  const [revision, setRevision] = useState(0);
  const pollRequest = useRef<AbortController | null>(null);
  const actionRequest = useRef<AbortController | null>(null);
  const downloadRequest = useRef<AbortController | null>(null);
  const downloadUrls = useRef(new Map<string, ReturnType<typeof setTimeout>>());
  const dialog = useRef<HTMLElement>(null);
  const historyButton = useRef<HTMLButtonElement>(null);

  useEffect(() => {
    if (!visible) setOpen(false);
  }, [visible]);

  useEffect(() => {
    if (!open) return;
    dialog.current?.querySelector<HTMLButtonElement>("button")?.focus();
    const trigger = historyButton.current;
    return () => {
      if (trigger?.getClientRects().length) trigger.focus();
    };
  }, [open]);

  useEffect(() => {
    if (!online) return;
    const controller = new AbortController();
    pollRequest.current = controller;
    let timer: ReturnType<typeof setTimeout> | undefined;
    setListState("loading");
    const refresh = async () => {
      if (controller.signal.aborted) return;
      try {
        const summaries = await api.listTraces(20, controller.signal);
        if (controller.signal.aborted) return;
        setTraces(
          [...summaries]
            .sort((a, b) => b.started_at_ms - a.started_at_ms)
            .slice(0, 20),
        );
        setListState("ready");
        setListError("");
      } catch (error) {
        if (controller.signal.aborted) return;
        setListState("error");
        setListError(errorText(error));
      } finally {
        if (!controller.signal.aborted)
          timer = setTimeout(() => void refresh(), 1000);
      }
    };
    void refresh();
    return () => {
      controller.abort();
      clearTimeout(timer);
      if (pollRequest.current === controller) pollRequest.current = null;
    };
  }, [online, revision]);

  useEffect(() => {
    const urls = downloadUrls.current;
    return () => {
      actionRequest.current?.abort();
      downloadRequest.current?.abort();
      for (const [url, timer] of urls) {
        clearTimeout(timer);
        URL.revokeObjectURL(url);
      }
      urls.clear();
    };
  }, []);

  useEffect(() => {
    if (online) return;
    actionRequest.current?.abort();
    downloadRequest.current?.abort();
    actionRequest.current = null;
    downloadRequest.current = null;
    setBusy(null);
    setDownloading(null);
  }, [online]);

  const seconds = Number(duration);
  const durationMs = Math.ceil(seconds * 1000);
  const validDuration =
    duration.trim() !== "" &&
    Number.isFinite(seconds) &&
    seconds > 0 &&
    Number.isSafeInteger(durationMs) &&
    durationMs > 0;
  const recording = traces.some((trace) => trace.status === "recording");
  const active = traces.some((trace) =>
    ["recording", "writing"].includes(trace.status),
  );
  const canStart = online && connected && !recording && !busy && validDuration;
  const canStop = online && recording && !busy;

  const changeTrace = async (kind: "start" | "stop") => {
    if (actionRequest.current || (kind === "start" ? !canStart : !canStop))
      return;
    const controller = new AbortController();
    actionRequest.current = controller;
    pollRequest.current?.abort();
    setListState("loading");
    setBusy(kind);
    setActionError("");
    setNotice("");
    try {
      const trace =
        kind === "start"
          ? await api.startTrace(durationMs, controller.signal)
          : await api.stopTrace(controller.signal);
      if (controller.signal.aborted) return;
      if (trace)
        setTraces((current) =>
          [trace, ...current.filter((item) => item.id !== trace.id)]
            .sort((a, b) => b.started_at_ms - a.started_at_ms)
            .slice(0, 20),
        );
      else {
        setTraces((current) =>
          current.filter((item) => item.status !== "recording"),
        );
        setNotice("当前没有正在采样的记录");
      }
    } catch (error) {
      if (!controller.signal.aborted)
        setActionError(
          `${kind === "start" ? "开始" : "停止"}采样失败：${errorText(error)}`,
        );
    } finally {
      if (!controller.signal.aborted) {
        actionRequest.current = null;
        setBusy(null);
        setRevision((value) => value + 1);
      }
    }
  };

  const download = async (trace: TraceSummary) => {
    if (downloadRequest.current) return;
    const controller = new AbortController();
    downloadRequest.current = controller;
    setDownloading(trace.id);
    setActionError("");
    try {
      const blob = await api.downloadTrace(trace.id, controller.signal);
      if (controller.signal.aborted) return;
      const url = URL.createObjectURL(blob);
      const timer = setTimeout(() => {
        URL.revokeObjectURL(url);
        downloadUrls.current.delete(url);
      }, 1000);
      downloadUrls.current.set(url, timer);
      const link = document.createElement("a");
      link.href = url;
      link.download = `trace-${trace.id.replace(/[^a-zA-Z0-9_-]/g, "_").slice(0, 80)}.json`;
      document.body.append(link);
      try {
        link.click();
      } finally {
        link.remove();
      }
    } catch (error) {
      if (!controller.signal.aborted)
        setActionError(`下载失败：${errorText(error)}`);
    } finally {
      if (!controller.signal.aborted) {
        downloadRequest.current = null;
        setDownloading(null);
      }
    }
  };

  const status = !online
    ? "服务未连接"
    : recording
      ? "正在录制"
      : active && traces.some((trace) => trace.status === "writing")
        ? "写入中"
        : !connected
          ? "端口未连接"
          : "录制就绪";

  return (
    <div className="trace-controls">
      <form
        className="trace-toolbar"
        aria-label="采样录制"
        onSubmit={(event) => {
          event.preventDefault();
          void changeTrace(recording ? "stop" : "start");
        }}
      >
        <label className="trace-duration" htmlFor="trace-duration">
          <input
            id="trace-duration"
            type="number"
            min="0"
            step="any"
            required
            value={duration}
            aria-label="录制时长，秒"
            title="正数秒，按整数毫秒记录"
            aria-invalid={!validDuration}
            disabled={recording || !!busy}
            onChange={(event) => setDuration(event.target.value)}
          />
          <span>秒</span>
        </label>
        <button
          type="submit"
          className="trace-toggle"
          data-recording={recording}
          disabled={recording ? !canStop : !canStart}
          aria-busy={busy !== null}
          aria-label={recording ? "停止录制" : "开始录制"}
          title={status}
        >
          <span className="trace-indicator" aria-hidden="true" />
          {busy ? "处理中" : recording ? "停止" : "录制"}
        </button>
        <button
          ref={historyButton}
          type="button"
          className="trace-history-button"
          aria-label="录制记录"
          aria-haspopup="dialog"
          aria-expanded={open}
          aria-controls="trace-dialog"
          onClick={() => setOpen(true)}
        >
          记录
        </button>
      </form>
      {!open && (actionError || (online && listError)) && (
        <div className="trace-inline-error" role="alert">
          <button type="button" onClick={() => setOpen(true)}>
            {actionError || `录制记录读取失败：${listError}`}
          </button>
        </div>
      )}
      {open &&
        createPortal(
          <div
            className="modal-backdrop trace-backdrop"
            onClick={(event) => {
              if (event.target === event.currentTarget) setOpen(false);
            }}
            onKeyDown={(event) => {
              if (event.key === "Escape") setOpen(false);
              if (event.key !== "Tab") return;
              const controls = Array.from(
                dialog.current?.querySelectorAll<HTMLElement>(
                  'button:not(:disabled), input:not(:disabled), [tabindex="0"]',
                ) ?? [],
              );
              const first = controls[0];
              const last = controls[controls.length - 1];
              if (event.shiftKey && document.activeElement === first) {
                event.preventDefault();
                last?.focus();
              } else if (!event.shiftKey && document.activeElement === last) {
                event.preventDefault();
                first?.focus();
              }
            }}
          >
            <section
              ref={dialog}
              className="panel trace-panel"
              id="trace-dialog"
              role="dialog"
              aria-modal="true"
              aria-labelledby="trace-title"
            >
              <div className="panel-heading">
                <h2 id="trace-title">录制记录</h2>
                <span className="trace-dialog-status" role="status">
                  {status}
                </span>
                <button
                  type="button"
                  aria-label="关闭录制记录"
                  onClick={() => setOpen(false)}
                >
                  ×
                </button>
              </div>
              <p className="trace-note">控制参考 · 实际发送 · 关节反馈</p>
              {online && listError && (
                <p className="trace-error" role="alert">
                  录制记录读取失败：{listError}
                </p>
              )}
              {actionError && (
                <p className="trace-error" role="alert">
                  {actionError}
                </p>
              )}
              {notice && (
                <p className="trace-note" role="status">
                  {notice}
                </p>
              )}
              <div className="trace-list-heading">
                <h3 id="trace-list-title">最近 10 次</h3>
                <span>
                  {!online
                    ? traces.length
                      ? "上次结果"
                      : "服务未连接"
                    : listState === "loading"
                      ? "读取中…"
                      : `${traces.length} 次`}
                </span>
              </div>
              <div className="trace-list" aria-labelledby="trace-list-title">
                {traces.length ? (
                  <table className="trace-table">
                    <colgroup>
                      <col />
                      <col className="trace-col-duration" />
                      <col className="trace-col-status" />
                      {Array.from({ length: 5 }, (_, index) => (
                        <col key={index} className="trace-col-count" />
                      ))}
                      <col className="trace-col-download" />
                    </colgroup>
                    <thead>
                      <tr>
                        <th scope="col">开始 / 设备</th>
                        <th scope="col">时长</th>
                        <th scope="col">状态</th>
                        <th scope="col">样本</th>
                        <th scope="col">已写入</th>
                        <th scope="col">待写</th>
                        <th scope="col">丢弃</th>
                        <th scope="col">错误</th>
                        <th scope="col">文件</th>
                      </tr>
                    </thead>
                    <tbody>
                      {traces.map((trace) => {
                        const incomplete =
                          trace.status === "complete" &&
                          (trace.pending > 0 ||
                            trace.errors > 0 ||
                            trace.written < trace.accepted);
                        const terminal = ["complete", "failed"].includes(
                          trace.status,
                        );
                        const device = `${trace.side === "right" ? "右臂" : trace.side === "left" ? "左臂" : trace.side} · ${trace.port ?? "未指定端口"}`;
                        return (
                          <tr key={trace.id}>
                            <td title={`采样 ${trace.id}`}>
                              <time
                                dateTime={new Date(
                                  trace.started_at_ms,
                                ).toISOString()}
                                title={new Date(
                                  trace.started_at_ms,
                                ).toLocaleString("zh-CN", { hour12: false })}
                              >
                                {timeFormat.format(trace.started_at_ms)}
                              </time>
                              <small className="trace-device" title={device}>
                                {device}
                              </small>
                            </td>
                            <td>{trace.duration_ms / 1000} 秒</td>
                            <td>
                              <span
                                className="trace-status"
                                data-status={
                                  incomplete ? "failed" : trace.status
                                }
                                title={trace.status}
                              >
                                {incomplete
                                  ? "写入异常"
                                  : (statusLabels[trace.status] ??
                                    trace.status)}
                              </span>
                            </td>
                            <td>{trace.accepted}</td>
                            <td>{trace.written}</td>
                            <td>{trace.pending}</td>
                            <td
                              className={
                                trace.dropped > 0 ? "warning" : undefined
                              }
                            >
                              {trace.dropped}
                            </td>
                            <td
                              className={
                                trace.errors > 0 ? "danger" : undefined
                              }
                            >
                              {trace.errors}
                            </td>
                            <td>
                              <button
                                type="button"
                                disabled={
                                  !online || !terminal || downloading !== null
                                }
                                aria-busy={downloading === trace.id}
                                title={
                                  !terminal
                                    ? "采样与写入结束后可下载"
                                    : undefined
                                }
                                onClick={() => void download(trace)}
                              >
                                {downloading === trace.id
                                  ? "下载中…"
                                  : "下载 JSON"}
                              </button>
                            </td>
                          </tr>
                        );
                      })}
                    </tbody>
                  </table>
                ) : (
                  <div className="logs-empty">
                    {!online
                      ? "连接服务后查看最近采样"
                      : listState === "loading"
                        ? "正在读取采样记录…"
                        : listState === "error"
                          ? "暂时无法读取采样记录"
                          : "暂无录制记录"}
                  </div>
                )}
              </div>
            </section>
          </div>,
          document.body,
        )}
    </div>
  );
}
