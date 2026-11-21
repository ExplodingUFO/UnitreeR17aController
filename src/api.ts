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
export type JointPositions = [
  number,
  number,
  number,
  number,
  number,
  number,
  number,
];
export const isJointPositions = (value: unknown): value is JointPositions =>
  Array.isArray(value) &&
  value.length === 7 &&
  Array.from(value).every((q) => typeof q === "number" && Number.isFinite(q));
export type Action = {
  request_id: string;
  progress: number;
} & (
  | { space: Space; mode: "continuous" | "step"; axis: number }
  | { space: "joint"; mode: "return"; axis: null }
);
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
  action: Action | null;
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
export interface ReturnToPoseArguments {
  joints: JointPositions;
  speed: number;
  disconnect_after?: boolean;
}
export type Operation =
  | { kind: "connect"; port: string; side: Side }
  | { kind: "disconnect" | "disable"; allow_release?: boolean }
  | { kind: "enable" | "clear_fault" | "stop" }
  | ({ kind: "return_to_pose" } & ReturnToPoseArguments)
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
export function hasFreshFeedback(view: View, now = performance.now()) {
  const s = view.snapshot;
  const elapsed = Math.max(0, now - view.receivedAt);
  const fresh = (age: number | null) =>
    age !== null &&
    Number.isFinite(age) &&
    age >= 0 &&
    age + elapsed <= FEEDBACK_MAX_AGE_MS;
  return (
    view.online &&
    !["disconnected", "connecting", "closing"].includes(s.phase) &&
    fresh(s.feedback_age_ms) &&
    isJointPositions(s.joints.map((joint) => joint.q)) &&
    s.joints.every((joint) => fresh(joint.age_ms))
  );
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
  private cancelReturn: (() => void) | null = null;
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
    if (this.cancelReturn)
      void this.hostStop("desktop_disposed").catch((error) => this.fail(error));
    this.stopped = true;
    this.clearJog();
    this.dropLease();
    this.socket?.close();
    if (this.pollTimer) clearInterval(this.pollTimer);
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
  }
  private accept(snapshot: Snapshot) {
    if (this.view.online && snapshot.seq < this.view.snapshot.seq) return;
    if (this.lease && snapshot.owner?.client_id !== "desktop")
      this.dropLease(false);
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
      const active = !!this.jog || !!this.cancelReturn;
      this.clearJog();
      this.dropLease(false);
      this.publish({ online: false, hasControl: false });
      if (active)
        void this.hostStop("websocket_closed").catch((e) => this.fail(e));
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
  private dropLease(notify = true) {
    this.leaseEpoch++;
    this.lease = null;
    if (this.renewTimer) clearTimeout(this.renewTimer);
    this.renewTimer = null;
    if (notify) this.publish({ hasControl: false });
  }
  async control(action: "acquire" | "release" | "renew") {
    if (action === "release") {
      this.leaseEpoch++;
      if (this.renewTimer) clearTimeout(this.renewTimer);
      await this.cancelJog("release_control");
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
        await this.cancelJog("renew_failed").catch(() => {});
        this.fail(e);
      }
    } finally {
      this.renewing = false;
    }
  }
  async command(
    kind: string,
    params: Record<string, unknown> = {},
    requestId: string = crypto.randomUUID(),
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
  async returnToPose(
    params: ReturnToPoseArguments,
    signal: AbortSignal,
    requestId: string = crypto.randomUUID(),
  ): Promise<Result> {
    if (signal.aborted) throw new Error("返回已取消");
    if (this.cancelReturn) throw new Error("正在返回收纳位");
    if (!isJointPositions(params.joints))
      throw new Error("收纳位必须包含七个有效关节位置");
    if (
      !Number.isFinite(params.speed) ||
      params.speed <= 0 ||
      params.speed > 0.2
    )
      throw new Error("返回速度须大于 0，且不超过 0.20 rad/s");
    const s = this.view.snapshot;
    if (!hasFreshFeedback(this.view))
      throw new Error("需要完整、新鲜的七轴反馈");
    if (!s.enabled || s.phase !== "ready" || s.action || this.jog)
      throw new Error("请先停止动作，并等待使能就绪");
    if (!this.view.hasControl) throw new Error("请先取得控制权");
    if (
      params.joints.some(
        (q, index) => Math.abs(q - s.joints[index].q!) > Math.PI * 2,
      )
    )
      throw new Error("返回行程不能超过单轴一圈（2π rad）");
    const generation = s.generation;
    return new Promise<Result>((resolve, reject) => {
      let settled = false;
      let cancelled = false;
      let unsubscribe = () => {};
      const finish = (error: Error | null, result?: Result) => {
        if (settled) return;
        settled = true;
        unsubscribe();
        signal.removeEventListener("abort", abort);
        if (this.cancelReturn === cancel) this.cancelReturn = null;
        if (error) reject(error);
        else resolve(result!);
      };
      const cancel = () => {
        cancelled = true;
        finish(new Error("返回已停止"));
      };
      const abort = () => {
        cancel();
        void this.hostStop("parking_dialog_cancelled").catch((error) =>
          this.fail(error),
        );
      };
      this.cancelReturn = cancel;
      signal.addEventListener("abort", abort, { once: true });
      unsubscribe = this.subscribe((view) => {
        if (!view.online) {
          cancelled = true;
          finish(new Error("状态连接已断开"));
          return;
        }
        // Results are newest first. An older accepted/terminal entry must never
        // override the latest result for this exact request.
        const result = view.snapshot.results.find(
          (item) => item.request_id === requestId,
        );
        if (result && result.status !== "accepted") {
          const expectedCode = params.disconnect_after ? "parked" : "returned";
          if (result.status === "completed" && result.code === expectedCode)
            finish(null, result);
          else finish(new Error(result.message || "返回未完成"));
          return;
        }
        if (
          view.snapshot.generation !== generation ||
          !view.hasControl ||
          !view.snapshot.enabled ||
          view.snapshot.phase === "fault"
        ) {
          cancelled = true;
          finish(new Error("返回条件已变化，已停止"));
          void this.hostStop("parking_context_changed").catch((error) =>
            this.fail(error),
          );
        }
      });
      void this.command(
        "return_to_pose",
        { ...params, joints: [...params.joints] },
        requestId,
        generation,
      )
        .then(() => {
          if (cancelled) return this.hostStop("parking_cancelled_before_ack");
        })
        .catch((error) => {
          if (settled && !cancelled) return;
          cancelled = true;
          finish(new Error(this.errorMessage(error)));
          void this.hostStop("parking_request_failed").catch((stopError) =>
            this.fail(stopError),
          );
        });
    });
  }
  async cancelReturnToPose(reason = "return_cancelled") {
    if (this.cancelReturn) await this.hostStop(reason);
  }
  async closeApp(allowRelease = false) {
    await invoke("close_app", { allowRelease });
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
          void this.cancelJog("feedback_stale").catch((e) => this.fail(e));
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
              void this.cancelJog("update_failed").catch((e) => this.fail(e));
            }
          })
          .finally(() => {
            jog.updating = false;
          });
      }, 100);
    } catch (e) {
      if (this.jog !== jog) return;
      this.clearJog();
      void this.hostStop("start_failed").catch((e) => this.fail(e));
      throw e;
    }
  }
  private clearJog() {
    if (this.jog?.timer) clearInterval(this.jog.timer);
    this.jog = null;
  }
  async cancelJog(reason = "unspecified") {
    if (!this.jog) return;
    this.clearJog();
    await this.hostStop(reason);
  }
  async hostStop(reason = "unspecified") {
    this.clearJog();
    this.cancelReturn?.();
    await invoke("host_stop", { reason });
  }
}
export const api = new DesktopApi();
