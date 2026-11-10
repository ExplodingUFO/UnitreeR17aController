import { useEffect, useId, useRef, useState, type KeyboardEvent } from "react";
import rawOpenApi from "../docs/openapi.json?raw";
import { api, type ResultStatus, type Snapshot, type View } from "./api";

type Tab = "access" | "reference" | "results";
type Example = { key: string; value: unknown };
type Media = {
  example?: unknown;
  examples?: Record<string, { value: unknown }>;
};
type Response = {
  $ref?: string;
  description?: string;
  content?: Record<string, Media>;
};
type Operation = {
  summary: string;
  description?: string;
  requestBody?: { content: Record<string, Media> };
  responses: Record<string, Response>;
  parameters?: { name: string; example?: unknown }[];
  "x-websocket"?: {
    authentication: { example: unknown };
    outbound: { example: unknown };
  };
};
type OpenApi = {
  paths: Record<string, Record<string, Operation>>;
  components: { responses: Record<string, Response> };
};

const specification = JSON.parse(rawOpenApi) as OpenApi;
const endpoints = Object.entries(specification.paths).flatMap(([path, item]) =>
  Object.entries(item).map(([method, operation]) => ({
    path,
    method: method.toUpperCase(),
    operation,
  })),
);
const tabs: { key: Tab; label: string }[] = [
  { key: "access", label: "接入信息" },
  { key: "reference", label: "接口参考" },
  { key: "results", label: "请求结果" },
];
const exampleLabels: Record<string, string> = {
  acquire: "取得控制权",
  renew: "续期控制权",
  release: "释放控制权",
  connect: "连接机械臂",
  disconnect: "断开连接",
  enable: "使能",
  disable: "失能",
  clear_fault: "清除故障",
  configure_tcp: "设置 TCP",
  jog_start: "开始连续点动",
  jog_update: "续期连续点动",
  step: "固定步长",
  stop: "停止",
};
const resultLabels: Record<ResultStatus, string> = {
  accepted: "已接受",
  rejected: "已拒绝",
  completed: "已完成",
  cancelled: "已取消",
  failed: "失败",
};
const endpointNotes: Record<string, string> = {
  "/v1/ports": "将返回的 name 用作连接命令的 port。",
  "/v1/state": "读取当前完整快照，字段单位为 m、rad、s 和 Nm。",
  "/v1/control": "控制权有效期 2500 ms，建议每 800 ms 续期。",
  "/v1/commands":
    "202 表示已入队。按 request_id 从状态的 results 中读取执行结果。",
  "/v1/logs": "按 id 从新到旧返回，limit 默认 50，最多 500 条。",
  "/v1/events": "连接后 2 秒内发送令牌，随后接收完整状态快照。",
};

function initialTab(): Tab {
  const [page, section] = window.location.hash.slice(1).split("/");
  return page === "api" && tabs.some((tab) => tab.key === section)
    ? (section as Tab)
    : "access";
}

function mediaExamples(media?: Media): Example[] {
  if (!media) return [];
  if (media.examples)
    return Object.entries(media.examples).map(([key, example]) => ({
      key,
      value: example.value,
    }));
  return "example" in media ? [{ key: "default", value: media.example }] : [];
}

async function writeClipboard(value: string): Promise<boolean> {
  try {
    if (!navigator.clipboard?.writeText) return false;
    await navigator.clipboard.writeText(value);
    return true;
  } catch {
    return false;
  }
}

function CodeExample({
  label,
  value,
  plainText = false,
}: {
  label: string;
  value: unknown;
  plainText?: boolean;
}) {
  const code = useRef<HTMLPreElement>(null);
  const [feedback, setFeedback] = useState("");
  const text = plainText
    ? String(value)
    : (JSON.stringify(value, null, 2) ?? "");
  useEffect(() => setFeedback(""), [text]);
  useEffect(() => {
    if (!feedback) return;
    const timer = setTimeout(() => setFeedback(""), 2200);
    return () => clearTimeout(timer);
  }, [feedback]);

  async function copy() {
    if (await writeClipboard(text)) {
      setFeedback("已复制");
      return;
    }
    const element = code.current;
    const selection = window.getSelection();
    if (!element?.getClientRects().length || !selection) {
      setFeedback("复制失败");
      return;
    }
    element.focus();
    const range = document.createRange();
    range.selectNodeContents(element);
    selection.removeAllRanges();
    selection.addRange(range);
    setFeedback(
      selection.toString() === text ? "已选中，请手动复制" : "复制失败",
    );
  }

  return (
    <section className="api-example">
      <div className="api-example-heading">
        <h3>{label}</h3>
        <button
          type="button"
          onClick={() => void copy()}
          aria-label={`复制${label}`}
        >
          复制
        </button>
      </div>
      <pre
        className="api-code"
        ref={code}
        tabIndex={0}
        role="region"
        aria-label={label}
      >
        <code>{text}</code>
      </pre>
      <p className="api-feedback" role="status">
        {feedback}
      </p>
    </section>
  );
}

export default function ApiPanel({ view }: { view: View }) {
  const id = useId();
  const [tab, setTab] = useState<Tab>(initialTab);
  const tabButtons = useRef<(HTMLButtonElement | null)[]>([]);
  const [tokenVisible, setTokenVisible] = useState(false);
  const [feedback, setFeedback] = useState("");
  const baseInput = useRef<HTMLInputElement>(null);
  const socketInput = useRef<HTMLInputElement>(null);
  const tokenInput = useRef<HTMLInputElement>(null);
  const [readStatus, setReadStatus] = useState<
    "idle" | "loading" | "success" | "error"
  >("idle");
  const [stateReply, setStateReply] = useState<Snapshot | null>(null);
  const [readError, setReadError] = useState("");
  const [selectedPath, setSelectedPath] = useState("/v1/state");
  const [inputKey, setInputKey] = useState("");
  const [responseStatus, setResponseStatus] = useState("");
  const info = api.getConnectionInfo();
  const baseUrl = info?.base_url ?? view.baseUrl;
  let socketUrl = "";
  if (baseUrl) {
    try {
      const url = new URL(`${baseUrl.replace(/\/$/, "")}/v1/events`);
      url.protocol = url.protocol === "https:" ? "wss:" : "ws:";
      socketUrl = url.toString();
    } catch {
      socketUrl = "";
    }
  }

  useEffect(() => {
    const followHash = () => {
      if (window.location.hash.split("/")[0] === "#api") setTab(initialTab());
      else setTokenVisible(false);
    };
    window.addEventListener("hashchange", followHash);
    return () => window.removeEventListener("hashchange", followHash);
  }, []);
  useEffect(() => setTokenVisible(false), [info?.token]);
  useEffect(() => {
    if (!feedback) return;
    const timer = setTimeout(() => setFeedback(""), 3000);
    return () => clearTimeout(timer);
  }, [feedback]);

  function chooseTab(next: Tab) {
    setTab(next);
    window.location.hash = `api/${next}`;
  }

  function moveTab(event: KeyboardEvent<HTMLButtonElement>, index: number) {
    let next = index;
    if (event.key === "ArrowRight" || event.key === "ArrowDown")
      next = (index + 1) % tabs.length;
    else if (event.key === "ArrowLeft" || event.key === "ArrowUp")
      next = (index + tabs.length - 1) % tabs.length;
    else if (event.key === "Home") next = 0;
    else if (event.key === "End") next = tabs.length - 1;
    else return;
    event.preventDefault();
    chooseTab(tabs[next].key);
    tabButtons.current[next]?.focus();
  }

  async function copyField(field: "base_url" | "websocket" | "token") {
    const value = field === "websocket" ? socketUrl : info?.[field];
    if (!value) return;
    const label =
      field === "token"
        ? "访问令牌"
        : field === "websocket"
          ? "订阅地址"
          : "服务地址";
    if (await writeClipboard(value)) {
      setFeedback(`${label}已复制`);
      return;
    }
    if (field === "token") setTokenVisible(true);
    requestAnimationFrame(() => {
      const element =
        field === "token"
          ? tokenInput.current
          : field === "websocket"
            ? socketInput.current
            : baseInput.current;
      if (!element?.getClientRects().length) return;
      element.focus();
      element.select();
      setFeedback(
        element.selectionStart === 0 && element.selectionEnd === value.length
          ? "已选中，请手动复制"
          : "复制失败",
      );
    });
  }

  async function readState() {
    setReadStatus("loading");
    setReadError("");
    try {
      const snapshot = await api.readState();
      setStateReply(snapshot);
      setReadStatus("success");
    } catch (error) {
      setReadError(error instanceof Error ? error.message : "状态读取失败");
      setReadStatus("error");
    }
  }

  function downloadSpecification() {
    const url = URL.createObjectURL(
      new Blob([rawOpenApi], { type: "application/json;charset=utf-8" }),
    );
    const link = document.createElement("a");
    link.href = url;
    link.download = "r17a-openapi.json";
    document.body.append(link);
    link.click();
    link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }

  function chooseEndpoint(path: string) {
    setSelectedPath(path);
    setInputKey("");
    setResponseStatus("");
  }

  const endpoint =
    endpoints.find((item) => item.path === selectedPath) ?? endpoints[0];
  const operation = endpoint.operation;
  const requestExamples = mediaExamples(
    operation.requestBody?.content["application/json"],
  );
  const requestExample =
    requestExamples.find((example) => example.key === inputKey) ??
    requestExamples[0];
  const status = responseStatus || Object.keys(operation.responses)[0];
  let response = operation.responses[status];
  if (response.$ref)
    response =
      specification.components.responses[response.$ref.split("/").at(-1)!];
  const responseMedia =
    response.content?.["application/json"] ?? response.content?.["text/plain"];
  const responseExamples = mediaExamples(responseMedia);
  const controlReplyKey: Record<string, string> = {
    acquire: "control_acquired",
    renew: "control_renewed",
    release: "control_released",
  };
  const responseExample =
    responseExamples.find(
      (example) => example.key === controlReplyKey[requestExample?.key],
    ) ?? responseExamples[0];
  const query = Object.fromEntries(
    (operation.parameters ?? [])
      .filter((parameter) => "example" in parameter)
      .map((parameter) => [parameter.name, parameter.example]),
  );
  const socket = operation["x-websocket"];
  const requestValue = socket
    ? socket.authentication.example
    : (requestExample?.value ?? query);
  let responseValue =
    socket && status === "101"
      ? socket.outbound.example
      : (responseExample?.value ?? null);
  if (
    endpoint.path === "/v1/commands" &&
    status === "202" &&
    responseValue &&
    typeof responseValue === "object" &&
    requestExample?.value &&
    typeof requestExample.value === "object"
  ) {
    responseValue = {
      ...responseValue,
      request_id: (requestExample.value as { request_id: string }).request_id,
    };
  }

  return (
    <section className="api-panel" aria-label="API 接入">
      <header className="secondary-toolbar api-toolbar">
        <h2 className="secondary-heading">API 接入</h2>
        <div className="secondary-tabs" role="tablist" aria-label="API 页面">
          {tabs.map((item, index) => (
            <button
              type="button"
              role="tab"
              key={item.key}
              id={`${id}-tab-${item.key}`}
              aria-selected={tab === item.key}
              aria-controls={`${id}-panel-${item.key}`}
              tabIndex={tab === item.key ? 0 : -1}
              ref={(element) => {
                tabButtons.current[index] = element;
              }}
              onClick={() => chooseTab(item.key)}
              onKeyDown={(event) => moveTab(event, index)}
            >
              {item.label}
            </button>
          ))}
        </div>
        <button type="button" onClick={downloadSpecification}>
          下载 OpenAPI
        </button>
      </header>
      <div className="api-content">
        <section
          className="api-access"
          role="tabpanel"
          id={`${id}-panel-access`}
          aria-labelledby={`${id}-tab-access`}
          hidden={tab !== "access"}
        >
          <div className="api-detail api-connection">
            <h3>连接信息</h3>
            <dl className="api-status">
              <div>
                <dt>状态订阅</dt>
                <dd>{view.online ? "已连接" : "未连接"}</dd>
              </div>
              <div>
                <dt>控制端</dt>
                <dd>
                  {view.online
                    ? (view.snapshot.owner?.client_id ?? "未分配")
                    : "—"}
                </dd>
              </div>
            </dl>
            <div className="api-field">
              <label htmlFor={`${id}-base-url`}>HTTP 地址</label>
              <div className="api-field-row">
                <input
                  id={`${id}-base-url`}
                  ref={baseInput}
                  value={baseUrl}
                  readOnly
                  spellCheck={false}
                  autoComplete="off"
                  placeholder="等待服务地址"
                />
                <button
                  type="button"
                  disabled={!info}
                  aria-label="复制 HTTP 地址"
                  onClick={() => void copyField("base_url")}
                >
                  复制地址
                </button>
              </div>
            </div>
            <div className="api-field">
              <label htmlFor={`${id}-socket-url`}>WebSocket 地址</label>
              <div className="api-field-row">
                <input
                  id={`${id}-socket-url`}
                  ref={socketInput}
                  value={socketUrl}
                  readOnly
                  spellCheck={false}
                  autoComplete="off"
                  placeholder="等待订阅地址"
                />
                <button
                  type="button"
                  disabled={!socketUrl}
                  aria-label="复制 WebSocket 地址"
                  onClick={() => void copyField("websocket")}
                >
                  复制地址
                </button>
              </div>
            </div>
            <div className="api-field">
              <label htmlFor={`${id}-token`}>访问令牌</label>
              <div className="api-field-row">
                <input
                  id={`${id}-token`}
                  ref={tokenInput}
                  value={info?.token ?? ""}
                  type={tokenVisible ? "text" : "password"}
                  readOnly
                  spellCheck={false}
                  autoComplete="off"
                  aria-describedby={`${id}-auth-hint`}
                  placeholder="等待访问令牌"
                />
                <button
                  type="button"
                  disabled={!info}
                  aria-pressed={tokenVisible}
                  aria-controls={`${id}-token`}
                  onClick={() => setTokenVisible((visible) => !visible)}
                >
                  {tokenVisible ? "隐藏" : "显示"}
                </button>
                <button
                  type="button"
                  disabled={!info}
                  onClick={() => void copyField("token")}
                >
                  复制令牌
                </button>
              </div>
            </div>
            <p id={`${id}-auth-hint`}>
              请求头：<code>Authorization: Bearer &lt;token&gt;</code>
            </p>
            {!info && <p>启动桌面程序后显示接入信息。</p>}
            <p className="api-feedback" role="status">
              {feedback}
            </p>
          </div>
          <section
            className="api-detail api-query"
            aria-label="状态查询"
            aria-busy={readStatus === "loading"}
          >
            <div className="secondary-toolbar">
              <h3>状态查询</h3>
              <button
                type="button"
                disabled={!info || readStatus === "loading"}
                onClick={() => void readState()}
              >
                {readStatus === "loading" ? "读取中…" : "读取状态"}
              </button>
            </div>
            <p>
              <code>GET /v1/state</code>
            </p>
            {readStatus === "idle" && (
              <div className="api-query-empty">
                <p>读取状态后在此查看完整 JSON。</p>
              </div>
            )}
            {readStatus === "loading" && !stateReply && (
              <p role="status">正在读取状态…</p>
            )}
            {readStatus === "error" && (
              <p className="api-error" role="alert">
                {readError}
              </p>
            )}
            {stateReply && <CodeExample label="状态返回" value={stateReply} />}
          </section>
        </section>
        <section
          className="api-reference"
          role="tabpanel"
          id={`${id}-panel-reference`}
          aria-labelledby={`${id}-tab-reference`}
          hidden={tab !== "reference"}
        >
          <nav className="api-endpoints" aria-label="接口列表">
            {endpoints.map((item) => (
              <button
                type="button"
                className="api-endpoint"
                aria-current={item.path === selectedPath ? "true" : undefined}
                aria-label={`${item.method} ${item.path}，${item.operation.summary}`}
                key={item.path}
                onClick={() => chooseEndpoint(item.path)}
              >
                <span className="api-method">
                  {item.path === "/v1/events" ? "WS" : item.method}
                </span>
                <code>{item.path}</code>
              </button>
            ))}
          </nav>
          <div className="api-detail">
            <header className="secondary-toolbar">
              <h3>{operation.summary}</h3>
              <code>
                {endpoint.method} {endpoint.path}
              </code>
            </header>
            <p>{endpointNotes[endpoint.path]}</p>
            <div className="secondary-toolbar">
              {requestExamples.length > 1 && (
                <div className="api-field">
                  <label htmlFor={`${id}-input-variant`}>
                    {endpoint.path === "/v1/commands" ? "命令" : "操作"}
                  </label>
                  <select
                    id={`${id}-input-variant`}
                    value={requestExample.key}
                    onChange={(event) => setInputKey(event.target.value)}
                  >
                    {requestExamples.map((example) => (
                      <option key={example.key} value={example.key}>
                        {exampleLabels[example.key] ?? example.key}
                      </option>
                    ))}
                  </select>
                </div>
              )}
              <div className="api-field">
                <label htmlFor={`${id}-response-status`}>返回状态</label>
                <select
                  id={`${id}-response-status`}
                  value={status}
                  onChange={(event) => setResponseStatus(event.target.value)}
                >
                  {Object.keys(operation.responses).map((code) => (
                    <option value={code} key={code}>
                      {code}
                    </option>
                  ))}
                </select>
              </div>
            </div>
            <div className="api-examples">
              <CodeExample
                label={
                  socket
                    ? "认证首帧"
                    : endpoint.method === "GET"
                      ? "查询参数"
                      : "请求 JSON"
                }
                value={requestValue}
              />
              <CodeExample
                label={socket && status === "101" ? "状态推送" : "返回示例"}
                value={responseValue}
                plainText={
                  !!response.content?.["text/plain"] &&
                  !response.content?.["application/json"]
                }
              />
            </div>
            <details>
              <summary>调用说明</summary>
              <p>{operation.description}</p>
              <p>{response.description}</p>
            </details>
          </div>
        </section>
        <section
          className="api-results"
          role="tabpanel"
          id={`${id}-panel-results`}
          aria-labelledby={`${id}-tab-results`}
          hidden={tab !== "results"}
        >
          <div className="secondary-toolbar">
            <h3>最近请求</h3>
            <span>新到旧</span>
          </div>
          {!view.online && <p role="status">状态订阅未连接</p>}
          {view.snapshot.results.length ? (
            <div
              className="api-result-table"
              tabIndex={0}
              role="region"
              aria-label="请求结果列表"
            >
              <table className="data-table">
                <thead>
                  <tr>
                    <th scope="col">状态</th>
                    <th scope="col">请求 ID</th>
                    <th scope="col">代码</th>
                    <th scope="col">说明</th>
                  </tr>
                </thead>
                <tbody>
                  {view.snapshot.results.map((result, index) => (
                    <tr key={`${result.request_id}-${result.status}-${index}`}>
                      <td data-status={result.status}>
                        {resultLabels[result.status] ?? result.status}
                      </td>
                      <td>
                        <code>{result.request_id}</code>
                      </td>
                      <td>
                        <code>{result.code}</code>
                      </td>
                      <td>{result.message}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          ) : (
            <div className="api-empty">
              <p>暂无请求结果</p>
              <p>提交操作后在此查看执行状态。</p>
              <button
                type="button"
                onClick={() => {
                  chooseEndpoint("/v1/commands");
                  chooseTab("reference");
                }}
              >
                查看命令接口
              </button>
            </div>
          )}
        </section>
      </div>
    </section>
  );
}
