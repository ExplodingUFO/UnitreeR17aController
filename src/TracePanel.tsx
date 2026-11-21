import { useEffect, useRef, useState } from "react";
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
}: {
  online: boolean;
  connected: boolean;
}) {
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
  const validDuration =
    duration.trim() !== "" &&
    Number.isInteger(seconds) &&
    seconds >= 1 &&
    seconds <= 20;
  const recording = traces.some((trace) => trace.status === "recording");
  const active = traces.some((trace) =>
    ["recording", "writing"].includes(trace.status),
  );
  const canStart =
    online &&
    connected &&
    listState === "ready" &&
    !active &&
    !busy &&
    validDuration;
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
          ? await api.startTrace(seconds * 1000, controller.signal)
          : await api.stopTrace(controller.signal);
      if (controller.signal.aborted) return;
      if (trace)
        setTraces((current) =>
          [trace, ...current.filter((item) => item.id !== trace.id)]
            .sort((a, b) => b.started_at_ms - a.started_at_ms)
            .slice(0, 20),
        );
      else setNotice("当前没有正在采样的记录");
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

  return (
    <div
      className="trace-panel"
      id="trace-panel"
      role="tabpanel"
      aria-labelledby="log-tab-traces"
    >
      <form
        className="trace-toolbar"
        onSubmit={(event) => {
          event.preventDefault();
          void changeTrace("start");
        }}
      >
        <label className="trace-duration" htmlFor="trace-duration">
          采样时长
          <input
            id="trace-duration"
            type="number"
            min="1"
            max="20"
            step="1"
            required
            value={duration}
            aria-describedby="trace-duration-hint"
            aria-invalid={!validDuration}
            onChange={(event) => setDuration(event.target.value)}
          />
          秒
        </label>
        <span className="trace-duration-hint" id="trace-duration-hint">
          1–20 秒
        </span>
        <button
          type="submit"
          className="primary"
          disabled={!canStart}
          aria-busy={busy === "start"}
          title={
            !connected
              ? "先连接端口"
              : active
                ? "等待当前采样写入结束"
                : undefined
          }
        >
          {busy === "start" ? "开始中…" : "开始采样"}
        </button>
        <button
          type="button"
          disabled={!canStop}
          aria-busy={busy === "stop"}
          onClick={() => void changeTrace("stop")}
        >
          {busy === "stop" ? "停止采样中…" : "停止采样"}
        </button>
        <span className="trace-connection" role="status">
          {!online ? "服务未连接" : !connected ? "端口未连接" : "端口已连接"}
        </span>
      </form>
      <p className="trace-note">控制参考 · 实际发送 · 关节反馈</p>
      {online && listError && (
        <p className="trace-error" role="alert">
          最近采样读取失败：{listError}
          {traces.length > 0 && "；以下保留上次结果"}
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
        <h3 id="trace-list-title">最近采样</h3>
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
                const terminal = ["complete", "failed"].includes(trace.status);
                const device = `${trace.side === "right" ? "右臂" : trace.side === "left" ? "左臂" : trace.side} · ${trace.port ?? "未指定端口"}`;
                return (
                  <tr key={trace.id}>
                    <td title={`采样 ${trace.id}`}>
                      <time
                        dateTime={new Date(trace.started_at_ms).toISOString()}
                        title={new Date(trace.started_at_ms).toLocaleString(
                          "zh-CN",
                          { hour12: false },
                        )}
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
                        data-status={incomplete ? "failed" : trace.status}
                        title={trace.status}
                      >
                        {incomplete
                          ? "写入异常"
                          : (statusLabels[trace.status] ?? trace.status)}
                      </span>
                    </td>
                    <td>{trace.accepted}</td>
                    <td>{trace.written}</td>
                    <td>{trace.pending}</td>
                    <td className={trace.dropped > 0 ? "warning" : undefined}>
                      {trace.dropped}
                    </td>
                    <td className={trace.errors > 0 ? "danger" : undefined}>
                      {trace.errors}
                    </td>
                    <td>
                      <button
                        type="button"
                        disabled={!online || !terminal || downloading !== null}
                        aria-busy={downloading === trace.id}
                        title={!terminal ? "采样与写入结束后可下载" : undefined}
                        onClick={() => void download(trace)}
                      >
                        {downloading === trace.id ? "下载中…" : "下载 JSON"}
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
                  : "暂无采样，连接端口后可开始"}
          </div>
        )}
      </div>
    </div>
  );
}
