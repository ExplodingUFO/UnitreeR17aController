import { invoke } from "@tauri-apps/api/core";
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
  voltage: number | null;
  error: number | null;
}
export interface Pose {
  translation: [number, number, number];
  rotation: [number, number, number, number];
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
}
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
  feedback_age_ms: number | null;
  action: {
    request_id: string;
    space: Space;
    mode: string;
    axis: number;
    progress: number;
  } | null;
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
export interface MotionArguments {
  space: Space;
  axis: number;
  direction: -1 | 1;
  speed: number;
  frame?: Frame;
}
export type Operation =
  | { kind: "connect"; port: string; side: Side }
  | { kind: "disconnect" | "enable" | "disable" | "clear_fault" | "stop" }
  | { kind: "configure_tcp"; tcp: Pose }
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
  feedback_age_ms: null,
  action: null,
  last_result: null,
  results: [],
  error: null,
  journal: { pending: 0, written: 0, dropped: 0, errors: 0, healthy: true },
  communication: { received: 0, crc_errors: 0, timeouts: 0 },
};
export interface View {
  snapshot: Snapshot;
  online: boolean;
  receivedAt: number;
  error: string;
  logs: Log[];
  ports: Port[];
  baseUrl: string;
  hasControl: boolean;
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
  view: View = {
    snapshot: emptySnapshot,
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
  private async request<T>(path: string, body?: unknown): Promise<T> {
    if (!this.session) throw new Error("未连接");
    try {
      const response = await fetch(
        `${this.session.base_url.replace(/\/$/, "")}${path}`,
        {
          method: body ? "POST" : "GET",
          headers: {
            Authorization: `Bearer ${this.session.token}`,
            ...(body ? { "Content-Type": "application/json" } : {}),
          },
          body: body ? JSON.stringify(body) : undefined,
          signal: AbortSignal.timeout(4000),
        },
      );
      const text = await response.text();
      let data: unknown;
      try {
        data = JSON.parse(text);
      } catch {
        if (response.ok) throw new Error("响应数据格式无效");
      }
      if (!response.ok) {
        const fields =
          data && typeof data === "object"
            ? (data as { message?: unknown; error?: unknown })
            : null;
        const nested =
          fields?.error && typeof fields.error === "object"
            ? (fields.error as { message?: unknown }).message
            : null;
        const message = [fields?.message, nested, fields?.error].find(
          (value): value is string => typeof value === "string" && !!value,
        );
        throw new Error(
          message ??
            (text.trim() && !text.trimStart().startsWith("<")
              ? text
              : `HTTP ${response.status}`),
        );
      }
      return data as T;
    } catch (error) {
      throw new Error(this.errorMessage(error));
    }
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
    this.stopped = true;
    this.clearJog();
    this.dropLease();
    this.socket?.close();
    if (this.pollTimer) clearInterval(this.pollTimer);
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
  }
  private accept(snapshot: Snapshot) {
    if (this.view.online && snapshot.seq < this.view.snapshot.seq) return;
    if (this.lease && snapshot.owner?.client_id !== "desktop") this.dropLease();
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
      const active = !!this.jog;
      this.clearJog();
      this.dropLease();
      this.publish({ online: false });
      if (active) void this.hostStop().catch((e) => this.fail(e));
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
  private dropLease() {
    this.leaseEpoch++;
    this.lease = null;
    if (this.renewTimer) clearTimeout(this.renewTimer);
    this.renewTimer = null;
    this.publish({ hasControl: false });
  }
  async control(action: "acquire" | "release" | "renew") {
    if (action === "release") {
      this.leaseEpoch++;
      if (this.renewTimer) clearTimeout(this.renewTimer);
      await this.cancelJog();
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
        this.dropLease();
        await this.cancelJog().catch(() => {});
        this.fail(e);
      }
    } finally {
      this.renewing = false;
    }
  }
  async command(
    kind: string,
    params: Record<string, unknown> = {},
    requestId = crypto.randomUUID(),
    generation = this.view.snapshot.generation,
  ) {
    if (!this.view.online) throw new Error("未连接");
    if (!this.lease || !this.view.hasControl) throw new Error("未持有控制权");
    return this.request<CommandAck>("/v1/commands", {
      ...params,
      kind,
      request_id: requestId,
      client_id: "desktop",
      lease_id: this.lease,
      generation,
    });
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
        if (
          this.view.snapshot.feedback_age_ms === null ||
          this.view.snapshot.feedback_age_ms +
            Math.max(0, performance.now() - this.view.receivedAt) >
            FEEDBACK_MAX_AGE_MS
        ) {
          void this.cancelJog().catch((e) => this.fail(e));
          return;
        }
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
              void this.cancelJog().catch((e) => this.fail(e));
            }
          })
          .finally(() => {
            jog.updating = false;
          });
      }, 100);
    } catch (e) {
      if (this.jog !== jog) return;
      this.clearJog();
      void this.hostStop().catch((e) => this.fail(e));
      throw e;
    }
  }
  private clearJog() {
    if (this.jog?.timer) clearInterval(this.jog.timer);
    this.jog = null;
  }
  async cancelJog() {
    if (!this.jog) return;
    this.clearJog();
    await this.hostStop();
  }
  async hostStop() {
    this.clearJog();
    await invoke("host_stop");
  }
}
export const api = new DesktopApi();
