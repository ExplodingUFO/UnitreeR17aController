import { useEffect, useRef, useState } from "react";
import { api, type LogFilters, type LogPage, type LogPageQuery } from "./api";

const dateFormat = new Intl.DateTimeFormat("zh-CN", {
  year: "numeric",
  month: "2-digit",
  day: "2-digit",
});
const timeFormat = new Intl.DateTimeFormat("zh-CN", {
  hour: "2-digit",
  minute: "2-digit",
  second: "2-digit",
  hour12: false,
});

export default function LogsPanel({
  online,
  active,
}: {
  online: boolean;
  active: boolean;
}) {
  const [search, setSearch] = useState("");
  const [from, setFrom] = useState("");
  const [to, setTo] = useState("");
  const [choices, setChoices] = useState<LogFilters>({
    levels: [],
    sources: [],
    codes: [],
  });
  const [choicesError, setChoicesError] = useState("");
  const [retry, setRetry] = useState(0);
  const fromMs = from ? new Date(from).getTime() : undefined;
  const toMs = to ? new Date(to).getTime() : undefined;
  const rangeError =
    fromMs !== undefined && toMs !== undefined && fromMs > toMs
      ? "开始时间不能晚于结束时间"
      : "";
  const [query, setQuery] = useState<LogPageQuery>({
    page: 1,
    page_size: 50,
    search: "",
  });
  const [data, setData] = useState<LogPage | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState("");
  const scroll = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (search === query.search) return;
    const timer = setTimeout(
      () =>
        setQuery((current) => ({
          ...current,
          page: 1,
          search,
          through_id: undefined,
        })),
      250,
    );
    return () => clearTimeout(timer);
  }, [search, query.search]);

  useEffect(() => {
    if (!online || !active) return;
    const controller = new AbortController();
    void api
      .readLogFilters(controller.signal)
      .then((filters) => {
        if (!controller.signal.aborted) {
          setChoices(filters);
          setChoicesError("");
        }
      })
      .catch((error) => {
        if (!controller.signal.aborted)
          setChoicesError(
            error instanceof Error ? error.message : "筛选项读取失败",
          );
      });
    return () => controller.abort();
  }, [online, active, retry, data?.through_id]);

  useEffect(() => {
    if (!online || !active || rangeError) return;
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout> | undefined;
    setLoading(true);
    setError("");
    let first = true;
    const refresh = async () => {
      try {
        const page = await api.readLogsPage(query, controller.signal);
        if (controller.signal.aborted) return;
        setData(page);
        setError("");
        if (first) scroll.current?.scrollTo(0, 0);
      } catch (error) {
        if (!controller.signal.aborted)
          setError(error instanceof Error ? error.message : "日志读取失败");
      } finally {
        if (!controller.signal.aborted) {
          setLoading(false);
          first = false;
          if (query.page === 1) timer = setTimeout(() => void refresh(), 2000);
        }
      }
    };
    void refresh();
    return () => {
      controller.abort();
      clearTimeout(timer);
    };
  }, [online, active, query, rangeError, retry]);

  const go = (page: number) => {
    if (!data || loading) return;
    setQuery((current) => ({
      ...current,
      page,
      through_id: page === 1 ? undefined : data.through_id,
    }));
  };
  const latest = () => {
    setQuery((current) => ({ ...current, page: 1, through_id: undefined }));
    setRetry((current) => current + 1);
  };
  const filter = (fields: Partial<LogPageQuery>) => {
    setData(null);
    setQuery((current) => ({
      ...current,
      ...fields,
      page: 1,
      through_id: undefined,
    }));
  };
  const time = (field: "from_ms" | "to_ms", value: string) => {
    if (field === "from_ms") setFrom(value);
    else setTo(value);
    filter({ [field]: value ? new Date(value).getTime() : undefined });
  };
  const filtered = Boolean(
    search || query.level || query.source || query.code || from || to,
  );
  const clear = () => {
    setSearch("");
    setFrom("");
    setTo("");
    setData(null);
    setQuery((current) => ({
      page: 1,
      page_size: current.page_size,
      search: "",
    }));
  };
  const pendingSearch = search !== query.search;
  const busy = loading || pendingSearch || Boolean(rangeError);
  const start = data?.total ? (data.page - 1) * data.page_size + 1 : 0;
  const end = data ? Math.min(data.page * data.page_size, data.total) : 0;

  return (
    <div className="log-history" aria-label="历史日志">
      <div className="log-history-toolbar">
        <label className="log-history-search">
          <span>搜索</span>
          <input
            type="search"
            value={search}
            maxLength={128}
            placeholder="来源、编号、内容或请求 ID"
            aria-label="搜索历史日志"
            onChange={(event) => setSearch(event.target.value)}
          />
        </label>
        {(
          [
            ["level", "级别", choices.levels],
            ["source", "来源", choices.sources],
            ["code", "编号", choices.codes],
          ] as const
        ).map(([key, label, values]) => (
          <label className="log-history-filter" key={key}>
            <span>{label}</span>
            <select
              aria-label={`筛选${label}`}
              value={query[key] ?? ""}
              onChange={(event) =>
                filter({ [key]: event.target.value || undefined })
              }
            >
              <option value="">全部</option>
              {Array.from(
                new Set([...(query[key] ? [query[key]!] : []), ...values]),
              )
                .filter(Boolean)
                .map((value) => (
                  <option key={value} value={value}>
                    {value}
                  </option>
                ))}
            </select>
          </label>
        ))}
        <button
          type="button"
          className="log-clear"
          disabled={!filtered}
          onClick={clear}
        >
          清除
        </button>
      </div>
      <div className="log-history-times">
        <label className="log-time-filter">
          <span>开始</span>
          <input
            type="datetime-local"
            step="0.001"
            value={from}
            aria-label="开始时间（本地）"
            onChange={(event) => time("from_ms", event.target.value)}
          />
        </label>
        <span className="log-time-separator">至</span>
        <label className="log-time-filter">
          <span>结束</span>
          <input
            type="datetime-local"
            step="0.001"
            value={to}
            aria-label="结束时间（本地）"
            onChange={(event) => time("to_ms", event.target.value)}
          />
        </label>
        <label className="log-page-size">
          每页
          <select
            value={query.page_size}
            aria-label="每页日志条数"
            onChange={(event) =>
              filter({ page_size: Number(event.target.value) })
            }
          >
            {[25, 50, 100].map((size) => (
              <option key={size}>{size}</option>
            ))}
          </select>
        </label>
        <button
          type="button"
          className="log-latest"
          disabled={!online || busy}
          onClick={latest}
        >
          最新
        </button>
      </div>
      {rangeError && (
        <div className="log-history-error" role="status">
          {rangeError}
        </div>
      )}
      {(!online || error || choicesError) && (
        <div className="log-history-error" role="status">
          <span>
            {!online
              ? "服务未连接"
              : error
                ? `日志读取失败：${error}`
                : `筛选项读取失败：${choicesError}`}
          </span>
          {online && (
            <button
              type="button"
              onClick={() => setRetry((current) => current + 1)}
            >
              重试
            </button>
          )}
        </div>
      )}
      <div
        className="logs-scroll log-history-scroll"
        ref={scroll}
        aria-busy={busy}
      >
        {data?.items.length ? (
          <table>
            <thead>
              <tr>
                <th scope="col">时间</th>
                <th scope="col">级别</th>
                <th scope="col">来源</th>
                <th scope="col">编号</th>
                <th scope="col">内容</th>
              </tr>
            </thead>
            <tbody>
              {data.items.map((log) => (
                <tr key={log.id}>
                  <td
                    title={new Date(log.ts_ms).toLocaleString("zh-CN", {
                      hour12: false,
                    })}
                  >
                    <time dateTime={new Date(log.ts_ms).toISOString()}>
                      <span>{timeFormat.format(log.ts_ms)}</span>
                      <small>{dateFormat.format(log.ts_ms)}</small>
                    </time>
                  </td>
                  <td className={`log-level ${log.level.toLowerCase()}`}>
                    {log.level}
                  </td>
                  <td title={log.source}>{log.source}</td>
                  <td title={log.code}>{log.code}</td>
                  <td className="log-history-message">
                    {log.message}
                    {log.request_id && (
                      <small className="log-request-id" title={log.request_id}>
                        {log.request_id}
                      </small>
                    )}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        ) : (
          <div className="logs-empty" role="status">
            {rangeError
              ? "请调整时间范围"
              : busy
                ? "正在读取日志…"
                : filtered
                  ? "没有匹配的日志"
                  : "暂无日志"}
          </div>
        )}
      </div>
      <div className="log-pagination" aria-label="日志分页">
        <span className="log-history-total">
          {data
            ? `${start}–${end} / ${data.total.toLocaleString("zh-CN")} 条`
            : "—"}
        </span>
        <span className="log-history-mode">
          {query.page === 1 ? "最新记录 · 自动更新" : "历史记录"}
        </span>
        <div className="log-page-controls">
          <button
            type="button"
            aria-label="上一页日志"
            disabled={!online || busy || !data || data.page <= 1}
            onClick={() => go(data!.page - 1)}
          >
            上一页
          </button>
          <span aria-live="polite">
            {data ? `${data.page} / ${data.total_pages}` : "— / —"}
          </span>
          <button
            type="button"
            aria-label="下一页日志"
            disabled={!online || busy || !data || data.page >= data.total_pages}
            onClick={() => go(data!.page + 1)}
          >
            下一页
          </button>
        </div>
      </div>
    </div>
  );
}
