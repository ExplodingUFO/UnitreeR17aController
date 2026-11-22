import { useEffect, useRef, useState, type PointerEvent } from "react";
import { listen } from "@tauri-apps/api/event";
import RobotScene from "./RobotScene";
import ApiPanel from "./ApiPanel";
import TracePanel from "./TracePanel";
import LogsPanel from "./LogsPanel";
import ServoPanel from "./ServoPanel";
import CalibrationPanel from "./CalibrationPanel";
import TeachingPanel from "./TeachingPanel";
import { PanelDivider, usePanelLayout } from "./PanelLayout";
import {
  api,
  FEEDBACK_MAX_AGE_MS,
  hasJointFeedback,
  isJointPositions,
  type JointPositions,
  type Phase,
  type Side,
  type Space,
  type View,
} from "./api";
const DEG = 180 / Math.PI;
type Page = "control" | "api" | "logs";
type SafetyIntent = "parking" | "disconnect" | "disable" | "exit";
const pages: { id: Page; label: string }[] = [
  { id: "control", label: "控制" },
  { id: "api", label: "接口" },
  { id: "logs", label: "日志" },
];
const currentPage = (): Page => {
  const page = location.hash.slice(1).split("/")[0];
  return page === "api" || page === "logs" ? page : "control";
};
const phaseLabels: Record<Phase, string> = {
  disconnected: "未连接",
  connecting: "连接中",
  disabled: "未使能",
  enabling: "使能中",
  ready: "就绪",
  teaching: "示教中",
  moving: "运动中",
  stopping: "停止中",
  fault: "故障",
  closing: "关闭中",
};
const fmt = (value: number | null | undefined, digits = 2) =>
  value == null || !Number.isFinite(value) ? "—" : value.toFixed(digits);
function rpy(q: [number, number, number, number]) {
  const [x, y, z, w] = q;
  return [
    Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))),
    Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
  ].map((v) => v * DEG);
}
function quaternion(values: number[]): [number, number, number, number] {
  const [r, p, y] = values.map((v) => v / DEG / 2);
  const [cr, sr, cp, sp, cy, sy] = [
    Math.cos(r),
    Math.sin(r),
    Math.cos(p),
    Math.sin(p),
    Math.cos(y),
    Math.sin(y),
  ];
  return [
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy,
  ];
}
export default function App() {
  const layout = usePanelLayout();
  const [page, setPage] = useState<Page>(currentPage);
  const [view, setView] = useState<View>(api.view);
  const [now, setNow] = useState(performance.now());
  const [side, setSide] = useState<Side>("right");
  const [port, setPort] = useState("");
  const [space, setSpace] = useState<Space>("cartesian");
  const [mode, setMode] = useState<"continuous" | "step">("continuous");
  const [frame, setFrame] = useState("base");
  const [linearSpeed, setLinearSpeed] = useState(10);
  const [angularSpeed, setAngularSpeed] = useState(5);
  const [linearStep, setLinearStep] = useState(1);
  const [angularStep, setAngularStep] = useState(1);
  const [busy, setBusy] = useState(false);
  const [motionPending, setMotionPending] = useState<string | null>(null);
  const pendingGeneration = useRef(0);
  const [tcpOpen, setTcpOpen] = useState(false);
  const [servoOpen, setServoOpen] = useState(false);
  const [calibrationOpen, setCalibrationOpen] = useState(false);
  const [teachingOpen, setTeachingOpen] = useState(false);
  const [tcpFields, setTcpFields] = useState(["0", "0", "0", "0", "0", "0"]);
  const [interfaceOpen, setInterfaceOpen] = useState(false);
  const [safetyIntent, setSafetyIntent] = useState<SafetyIntent | null>(null);
  const [safetyDevice, setSafetyDevice] = useState({
    side: "right" as Side,
    port: "",
  });
  const [safetyStopGeneration, setSafetyStopGeneration] = useState<
    number | null
  >(null);
  const closeRequested = useRef(() => {});
  const [enablePending, setEnablePending] = useState<string | null>(null);
  const enableRequest = useRef<string | null>(null);
  const [pressed, setPressed] = useState<string | null>(null);
  const pointer = useRef<number | null>(null);
  const stepGate = useRef(false);
  const s = view.snapshot;
  useEffect(() => {
    const unsub = api.subscribe(setView);
    void api.start();
    const timer = setInterval(() => setNow(performance.now()), 100);
    return () => {
      unsub();
      clearInterval(timer);
      api.dispose();
    };
  }, []);
  useEffect(() => {
    if (!("__TAURI_INTERNALS__" in window)) return;
    let disposed = false;
    let unlisten: (() => void) | undefined;
    void listen("close-requested", () => closeRequested.current())
      .then((stopListening) => {
        if (disposed) stopListening();
        else unlisten = stopListening;
      })
      .catch((error) => api.fail(error));
    return () => {
      disposed = true;
      unlisten?.();
    };
  }, []);
  useEffect(() => {
    if (!port && view.ports.length) setPort(view.ports[0].name);
  }, [port, view.ports]);
  const cancel = (reason = "unspecified") => {
    pointer.current = null;
    setPressed(null);
    void api.cancelJog(reason).catch((e) => api.fail(e));
  };
  useEffect(() => {
    const release = (event: Event) => cancel(`window_${event.type}`);
    const visibility = () => {
      if (document.hidden) cancel("document_hidden");
    };
    window.addEventListener("pointerup", release);
    window.addEventListener("pointercancel", release);
    window.addEventListener("blur", release);
    document.addEventListener("visibilitychange", visibility);
    return () => {
      window.removeEventListener("pointerup", release);
      window.removeEventListener("pointercancel", release);
      window.removeEventListener("blur", release);
      document.removeEventListener("visibilitychange", visibility);
    };
  }, []);
  useEffect(() => {
    if (!motionPending) return;
    const terminal =
      s.results?.some(
        (r) =>
          r.request_id === motionPending &&
          ["completed", "cancelled", "failed", "rejected"].includes(r.status),
      ) ||
      (s.last_result?.request_id === motionPending &&
        ["completed", "cancelled", "failed", "rejected"].includes(
          s.last_result.status,
        ));
    if (
      terminal ||
      s.generation !== pendingGeneration.current ||
      !view.online
    ) {
      setMotionPending(null);
      stepGate.current = false;
    }
  }, [s, motionPending, view.online]);
  useEffect(() => {
    if (!enablePending) return;
    const terminal =
      s.results.some(
        (result) =>
          result.request_id === enablePending &&
          ["completed", "cancelled", "failed", "rejected"].includes(
            result.status,
          ),
      ) ||
      (s.last_result?.request_id === enablePending &&
        ["completed", "cancelled", "failed", "rejected"].includes(
          s.last_result.status,
        ));
    if (
      terminal ||
      s.enabled ||
      ["fault", "disconnected", "closing"].includes(s.phase) ||
      !view.online
    ) {
      enableRequest.current = null;
      setEnablePending(null);
    }
  }, [s, view.online, enablePending]);
  const enableInProgress = !!enablePending || s.phase === "enabling";
  const connected =
    view.online && !["disconnected", "connecting", "closing"].includes(s.phase);
  const feedbackAge =
    s.feedback_age_ms == null
      ? null
      : s.feedback_age_ms + Math.max(0, now - view.receivedAt);
  const feedbackExpired =
    feedbackAge !== null && feedbackAge > FEEDBACK_MAX_AGE_MS;
  const valid = hasJointFeedback(view);
  const feedbackStatus = !connected
    ? "未连接"
    : feedbackAge === null
      ? "等待反馈"
      : feedbackExpired
        ? "反馈过期"
        : valid
          ? "实时"
          : "等待反馈";
  useEffect(() => {
    if (!valid || !view.hasControl) {
      cancel(!valid ? "feedback_invalid" : "control_lost");
    }
  }, [valid, view.hasControl]);
  const hasOwner = !!s.owner && view.online;
  const deviceWritable =
    view.online && view.hasControl && !busy && !safetyIntent;
  const motionReason =
    !view.online || !connected
      ? "未连接"
      : !view.hasControl
        ? hasOwner
          ? "其他客户端控制"
          : "未取得控制权"
        : s.phase === "fault"
          ? "故障"
          : enableInProgress
            ? "使能中"
            : !valid
              ? "等待反馈"
              : !s.enabled
                ? "未使能"
                : space === "cartesian" && !s.tcp_configured
                  ? "未设置 TCP"
                  : s.phase === "stopping"
                    ? "停止中"
                    : s.action || motionPending
                      ? "运动中"
                      : "";
  const motionAllowed = !motionReason && !busy && !safetyIntent;
  useEffect(() => {
    if (connected) setSide(s.side);
  }, [connected, s.side]);
  useEffect(() => {
    cancel("generation_changed");
  }, [s.generation]);
  const modelSide = connected ? s.side : side;
  const source =
    !connected || feedbackAge === null
      ? "参考姿态"
      : feedbackExpired
        ? "反馈过期"
        : valid
          ? "关节反馈"
          : "参考姿态";
  const tcpValues =
    valid && s.tcp && s.tcp_configured
      ? [...s.tcp.translation.map((v) => v * 1000), ...rpy(s.tcp.rotation)]
      : null;
  const tcpAtLink7 =
    !!s.tcp_transform &&
    s.tcp_transform.translation.every((value) => Math.abs(value) < 1e-9) &&
    s.tcp_transform.rotation[0] * s.tcp_transform.rotation[0] +
      s.tcp_transform.rotation[1] * s.tcp_transform.rotation[1] +
      s.tcp_transform.rotation[2] * s.tcp_transform.rotation[2] <
      1e-18 &&
    Math.abs(Math.abs(s.tcp_transform.rotation[3]) - 1) < 1e-9;
  const run = async (fn: () => Promise<unknown>) => {
    if (busy) return;
    setBusy(true);
    api.clearError();
    try {
      await fn();
    } catch (e) {
      api.fail(e);
    } finally {
      setBusy(false);
    }
  };
  const toggleEnable = () => {
    if (s.enabled || enableInProgress) {
      openSafety("disable");
      return;
    }
    void run(async () => {
      if (enableRequest.current || s.phase === "enabling") return;
      const requestId = crypto.randomUUID();
      enableRequest.current = requestId;
      setEnablePending(requestId);
      try {
        await api.command("enable", {}, requestId);
      } catch (error) {
        if (enableRequest.current === requestId) {
          enableRequest.current = null;
          setEnablePending(null);
        }
        throw error;
      }
    });
  };
  const stop = () => {
    pointer.current = null;
    setPressed(null);
    setMotionPending(null);
    stepGate.current = false;
    void api.hostStop("stop_button").catch((e) => api.fail(e));
  };
  const openSafety = (intent: SafetyIntent) => {
    if (safetyIntent === intent) return;
    const moving =
      !!s.action ||
      !!motionPending ||
      pointer.current !== null ||
      ["moving", "stopping"].includes(s.phase);
    setSafetyStopGeneration(moving ? s.generation : null);
    setSafetyDevice({
      side: modelSide,
      port: connected ? (s.port ?? port) : port,
    });
    if (moving) stop();
    cancel("safety_dialog");
    setTcpOpen(false);
    setServoOpen(false);
    setCalibrationOpen(false);
    setTeachingOpen(false);
    setInterfaceOpen(false);
    setSafetyIntent(intent);
  };
  closeRequested.current = () => openSafety("exit");
  const dismissSafety = () => {
    void api
      .cancelReturnToPose("parking_dialog_closed")
      .catch((error) => api.fail(error));
    setSafetyIntent(null);
  };
  const changeControls = (fn: () => void) => {
    cancel("controls_changed");
    fn();
  };
  const openServo = () => {
    if (s.action?.mode === "calibration") {
      setCalibrationOpen(true);
      return;
    }
    cancel("servo_dialog");
    setTcpOpen(false);
    setInterfaceOpen(false);
    setServoOpen(true);
  };
  const openTeaching = () => {
    cancel("teaching_dialog");
    setTcpOpen(false);
    setInterfaceOpen(false);
    setServoOpen(false);
    setCalibrationOpen(false);
    setTeachingOpen(true);
  };
  const openCalibration = () => {
    cancel("calibration_dialog");
    setTcpOpen(false);
    setInterfaceOpen(false);
    setServoOpen(false);
    setCalibrationOpen(true);
  };
  const navigate = (next: Page) => {
    cancel("navigation");
    dismissSafety();
    setTcpOpen(false);
    setServoOpen(false);
    setCalibrationOpen(false);
    setTeachingOpen(false);
    setInterfaceOpen(false);
    setPage(next);
    location.hash = next;
  };
  useEffect(() => {
    const changed = () => {
      cancel("hashchange");
      void api
        .cancelReturnToPose("navigation")
        .catch((error) => api.fail(error));
      setSafetyIntent(null);
      setTcpOpen(false);
      setServoOpen(false);
      setCalibrationOpen(false);
      setTeachingOpen(false);
      setInterfaceOpen(false);
      setPage(currentPage());
    };
    window.addEventListener("hashchange", changed);
    return () => window.removeEventListener("hashchange", changed);
  }, []);
  const beginMotion = (axis: number, direction: number) => {
    const angular = space === "joint" || axis >= 3;
    const speed = angular ? angularSpeed / DEG : linearSpeed / 1000;
    if (!Number.isFinite(speed) || speed <= 0) {
      pointer.current = null;
      api.fail(new Error("请输入大于 0 的速度"));
      return;
    }
    const amount = angular ? angularStep / DEG : linearStep / 1000;
    if (mode === "step" && (!Number.isFinite(amount) || amount <= 0)) {
      pointer.current = null;
      api.fail(new Error("请输入大于 0 的步长"));
      return;
    }
    setPressed(`${axis}:${direction}`);
    api.clearError();
    const params = { space, axis, direction, speed, frame };
    if (mode === "continuous") {
      void api.startJog(params).catch((e) => {
        setPressed(null);
        pointer.current = null;
        api.fail(e);
      });
    } else {
      stepGate.current = true;
      const id = crypto.randomUUID();
      pendingGeneration.current = s.generation;
      setMotionPending(id);
      void api
        .command(
          "step",
          {
            ...params,
            amount,
          },
          id,
        )
        .catch((e) => {
          setMotionPending(null);
          stepGate.current = false;
          api.fail(e);
        });
    }
  };
  const startMotion = (
    event: PointerEvent<HTMLButtonElement>,
    axis: number,
    direction: number,
  ) => {
    if (
      event.button !== 0 ||
      !motionAllowed ||
      stepGate.current ||
      pointer.current !== null
    )
      return;
    event.preventDefault();
    event.currentTarget.setPointerCapture(event.pointerId);
    pointer.current = event.pointerId;
    beginMotion(axis, direction);
  };
  const openTcp = () => {
    const transform = s.tcp_transform;
    setTcpFields(
      transform
        ? [
            ...transform.translation.map((value) => value * 1000),
            ...rpy(transform.rotation),
          ].map(String)
        : ["0", "0", "0", "0", "0", "0"],
    );
    setTcpOpen(true);
  };
  const applyTcp = () =>
    void run(async () => {
      const values = tcpFields.map(Number);
      if (
        tcpFields.some((v) => v.trim() === "") ||
        values.some((v) => !Number.isFinite(v))
      )
        throw new Error("请输入有效的 TCP 数值");
      await api.command("configure_tcp", {
        tcp: {
          translation: values.slice(0, 3).map((v) => v / 1000),
          rotation: quaternion(values.slice(3)),
        },
      });
      setTcpOpen(false);
    });
  const result = s.last_result;
  const resolvedMotionBusy =
    result?.status === "rejected" &&
    result.code === "motion_busy" &&
    s.phase === "ready" &&
    !s.action;
  const fault =
    s.error?.message ||
    (!resolvedMotionBusy &&
    ["failed", "rejected"].includes(result?.status ?? "")
      ? result?.message
      : "");
  const error = view.error || fault;
  return (
    <div
      className="workbench"
      ref={layout.rootRef}
      style={layout.style}
      data-resizing={layout.resizing}
      data-short-workspace={layout.shortWorkspace}
      data-page={page}
    >
      <header className="topbar">
        <div className="brand">
          <h1>R1-7a 控制器</h1>
        </div>
        <nav
          className="page-tabs"
          aria-label="工作区"
          role="tablist"
          onKeyDown={(event) => {
            const index = pages.findIndex((item) => item.id === page);
            const next =
              event.key === "ArrowRight"
                ? (index + 1) % pages.length
                : event.key === "ArrowLeft"
                  ? (index + pages.length - 1) % pages.length
                  : event.key === "Home"
                    ? 0
                    : event.key === "End"
                      ? pages.length - 1
                      : null;
            if (next === null) return;
            event.preventDefault();
            navigate(pages[next].id);
            document.getElementById(`page-${pages[next].id}`)?.focus();
          }}
        >
          {pages.map((item) => (
            <button
              type="button"
              key={item.id}
              id={`page-${item.id}`}
              role="tab"
              aria-selected={page === item.id}
              aria-controls={
                item.id === "logs" ? "logs-panel" : `${item.id}-view`
              }
              tabIndex={page === item.id ? 0 : -1}
              onClick={() => navigate(item.id)}
            >
              {item.label}
            </button>
          ))}
        </nav>
        <div
          className={`status-pill ${s.phase === "fault" ? "danger" : connected ? "good" : ""}`}
        >
          {view.online ? (phaseLabels[s.phase] ?? s.phase) : "未连接"}
        </div>
        <div className="top-metric">
          <span>距上次反馈</span>
          <b className={connected && feedbackExpired ? "warning" : ""}>
            {feedbackAge == null ? "—" : `${Math.round(feedbackAge)} ms`}
          </b>
        </div>
        <div className="ownership">
          <span>控制端</span>
          <b title={hasOwner ? s.owner!.client_id : "无"}>
            {hasOwner
              ? s.owner!.client_id === "desktop"
                ? "本机"
                : s.owner!.client_id
              : "—"}
          </b>
        </div>
        <button
          className="control-lease"
          disabled={
            !view.online ||
            busy ||
            !!safetyIntent ||
            (hasOwner && !view.hasControl)
          }
          onClick={() =>
            void run(() => api.control(view.hasControl ? "release" : "acquire"))
          }
        >
          {view.hasControl ? "释放控制" : "取得控制"}
        </button>
        <button className="stop-button" onClick={stop}>
          停止 <kbd>Esc</kbd>
        </button>
      </header>
      <main
        className="workspace"
        ref={layout.workspaceRef}
        id="control-view"
        role="tabpanel"
        aria-labelledby="page-control"
        hidden={page !== "control"}
      >
        <section className="panel left-panel" id="device-panel">
          <div className="panel-heading">
            <h2>连接</h2>
            <button
              type="button"
              aria-haspopup="dialog"
              aria-controls="teaching-dialog"
              aria-expanded={teachingOpen}
              onClick={openTeaching}
            >
              示教
            </button>
            <button
              type="button"
              aria-haspopup="dialog"
              aria-controls="calibration-dialog"
              aria-expanded={calibrationOpen}
              onClick={openCalibration}
            >
              校准
            </button>
            <button disabled={busy} onClick={() => openSafety("parking")}>
              收纳
            </button>
          </div>
          <div className="connection-form">
            <label htmlFor="side">机械臂</label>
            <select
              id="side"
              value={modelSide}
              disabled={connected || busy}
              onChange={(e) => setSide(e.target.value as Side)}
            >
              <option value="right">右臂</option>
              <option value="left">左臂</option>
            </select>
            <label htmlFor="port">串口</label>
            <div className="port-select">
              <select
                id="port"
                value={connected ? (s.port ?? port) : port}
                disabled={connected || busy}
                onChange={(e) => setPort(e.target.value)}
              >
                <option value="">选择串口</option>
                {view.ports.map((p) => (
                  <option key={p.name} value={p.name}>
                    {p.label || p.name}
                  </option>
                ))}
              </select>
              <button
                title="刷新串口"
                aria-label="刷新串口"
                disabled={!view.online || busy}
                onClick={() => void api.refreshPorts()}
              >
                ↻
              </button>
            </div>
          </div>
          <div className="device-actions">
            <button
              disabled={
                !deviceWritable ||
                s.phase === "connecting" ||
                s.phase === "closing" ||
                (!connected && !port)
              }
              onClick={() =>
                connected
                  ? openSafety("disconnect")
                  : void run(() => api.command("connect", { port, side }))
              }
            >
              {connected ? "断开" : "连接"}
            </button>
            <button
              className={s.enabled ? "" : "primary"}
              disabled={
                !deviceWritable ||
                !connected ||
                (!s.enabled &&
                  !enableInProgress &&
                  (s.phase === "fault" || !valid))
              }
              onClick={toggleEnable}
            >
              {enableInProgress ? "取消使能" : s.enabled ? "失能" : "使能"}
            </button>
            <button
              disabled={
                !deviceWritable || !connected || s.enabled || enableInProgress
              }
              title={
                s.enabled || enableInProgress ? "请先失能，再清错" : undefined
              }
              onClick={() => void run(() => api.command("clear_fault"))}
            >
              清错
            </button>
          </div>
          <div className="section-title">
            <h2>关节</h2>
            <span>{feedbackStatus}</span>
          </div>
          <div className="joint-table-wrap">
            <table className="joint-table">
              <thead>
                <tr>
                  <th>轴</th>
                  <th>位置 °</th>
                  <th>速度 °/s</th>
                  <th>扭矩 Nm</th>
                </tr>
              </thead>
              <tbody>
                {Array.from({ length: 7 }, (_, i) => {
                  const j = s.joints[i];
                  const fresh = valid && j?.q != null;
                  return (
                    <tr
                      key={i}
                      title={
                        fresh
                          ? `温度 ${fmt(j?.temperature, 1)} °C，电压 ${fmt(j?.voltage, 1)} V，故障码 ${j?.error ?? "—"}`
                          : feedbackStatus
                      }
                    >
                      <th>{j?.name || `J${i + 1}`}</th>
                      <td>{fresh ? fmt(j.q! * DEG) : "—"}</td>
                      <td>
                        {fresh && j.dq != null ? fmt(j.dq * DEG, 1) : "—"}
                      </td>
                      <td>{fresh ? fmt(j.tau) : "—"}</td>
                    </tr>
                  );
                })}
              </tbody>
            </table>
          </div>
          <div className="device-detail">
            <span>设备</span>
            <b title={s.port ?? ""}>{connected ? s.port : "—"}</b>
            <span>收包 / 校验错 / 超时</span>
            <b>
              {s.communication.received} / {s.communication.crc_errors} /{" "}
              {s.communication.timeouts}
            </b>
          </div>
        </section>
        <PanelDivider {...layout.divider("left")} />
        <section className="center-panel">
          <div className="panel scene-panel">
            <div className="panel-heading">
              <h2>{modelSide === "right" ? "右臂" : "左臂"}</h2>
              <TracePanel
                online={view.online}
                connected={connected}
                visible={page === "control"}
              />
            </div>
            <RobotScene
              side={modelSide}
              joints={s.joints}
              valid={valid}
              source={source}
            />
          </div>
          <PanelDivider {...layout.divider("tcp")} />
          <div className="panel tcp-panel" id="tcp-panel">
            <div className="panel-heading">
              <h2>
                {s.tcp_configured
                  ? tcpAtLink7
                    ? "TCP 坐标（Link7 法兰）"
                    : "TCP 坐标"
                  : "末端坐标（Link7）"}
              </h2>
              <button
                disabled={!deviceWritable || !!s.action}
                onClick={openTcp}
              >
                设置 TCP
              </button>
            </div>
            <div className="tcp-grid">
              {["X", "Y", "Z", "Rx", "Ry", "Rz"].map((name, i) => (
                <div className="tcp-value" key={name}>
                  <span>
                    {name}
                    <small>{i < 3 ? "mm" : "°"}</small>
                  </span>
                  <b>{fmt(tcpValues?.[i], i < 3 ? 1 : 2)}</b>
                </div>
              ))}
            </div>
            <div className="tcp-state">
              {!connected
                ? "未连接"
                : feedbackAge === null
                  ? "等待反馈"
                  : feedbackExpired
                    ? "反馈过期"
                    : !s.tcp_configured
                      ? "未设置 TCP"
                      : valid
                        ? tcpAtLink7
                          ? "默认 Link7 法兰"
                          : "基座坐标系"
                        : "等待反馈"}
            </div>
          </div>
        </section>
        <PanelDivider {...layout.divider("right")} />
        <section className="panel jog-panel" id="jog-panel">
          <div className="panel-heading">
            <h2>点动</h2>
            <div className="jog-heading-actions">
              <select
                className="jog-frame"
                aria-label="坐标系"
                value={frame}
                disabled={space === "joint" || !!s.action}
                onChange={(e) => changeControls(() => setFrame(e.target.value))}
              >
                <option value="base">基座坐标</option>
                <option value="tool">工具坐标</option>
              </select>
              <button
                type="button"
                aria-label="伺服参数"
                aria-haspopup="dialog"
                aria-controls="servo-dialog"
                aria-expanded={servoOpen}
                title="伺服参数"
                onClick={openServo}
              >
                参数
              </button>
            </div>
          </div>
          <div className="jog-settings">
            <div className="segmented" aria-label="点动空间">
              {(["cartesian", "joint"] as Space[]).map((value) => (
                <button
                  key={value}
                  className={space === value ? "selected" : ""}
                  aria-pressed={space === value}
                  onClick={() =>
                    changeControls(() => {
                      setSpace(value);
                    })
                  }
                >
                  {value === "cartesian" ? "末端" : "关节"}
                </button>
              ))}
            </div>
            <div className="segmented" aria-label="点动模式">
              {(["continuous", "step"] as const).map((value) => (
                <button
                  key={value}
                  className={mode === value ? "selected" : ""}
                  aria-pressed={mode === value}
                  onClick={() => changeControls(() => setMode(value))}
                >
                  {value === "continuous" ? "按住连续" : "固定步长"}
                </button>
              ))}
            </div>
          </div>
          <div className="jog-axis-list">
            {(space === "joint"
              ? ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]
              : ["X", "Y", "Z", "Rx", "Ry", "Rz"]
            ).map((name, axis) => (
              <div className="jog-axis" key={`${space}-${name}`}>
                <span
                  className={
                    axis < 3 && space === "cartesian"
                      ? `axis-${name.toLowerCase()}`
                      : ""
                  }
                >
                  {name}
                </span>
                {[-1, 1].map((direction) => (
                  <button
                    key={direction}
                    aria-label={`${name} ${direction < 0 ? "负向" : "正向"}`}
                    disabled={
                      !motionAllowed && pressed !== `${axis}:${direction}`
                    }
                    className={`jog-button ${pressed === `${axis}:${direction}` ? "pressed" : ""}`}
                    onPointerDown={(event) =>
                      startMotion(event, axis, direction)
                    }
                    onPointerUp={() => cancel("pointerup")}
                    onPointerCancel={() => cancel("pointercancel")}
                    onLostPointerCapture={() => cancel("lost_pointer_capture")}
                    onKeyDown={(event) => {
                      if (event.key !== " " && event.key !== "Enter") return;
                      event.preventDefault();
                      if (
                        event.repeat ||
                        !motionAllowed ||
                        stepGate.current ||
                        pointer.current !== null
                      )
                        return;
                      pointer.current = -1;
                      beginMotion(axis, direction);
                    }}
                    onKeyUp={(event) => {
                      if (event.key !== " " && event.key !== "Enter") return;
                      event.preventDefault();
                      cancel("key_up");
                    }}
                    onBlur={() => cancel("button_blur")}
                    onContextMenu={(e) => e.preventDefault()}
                  >
                    {direction < 0 ? "−" : "+"}
                  </button>
                ))}
              </div>
            ))}
          </div>
          <div className="motion-values">
            {space === "cartesian" && (
              <label>
                <span>
                  线速度 <small>mm/s</small>
                </span>
                <input
                  aria-label="线速度 mm/s"
                  type="number"
                  min="0"
                  step="any"
                  value={linearSpeed}
                  onChange={(e) => setLinearSpeed(Number(e.target.value))}
                />
              </label>
            )}
            <label>
              <span>
                {space === "joint" ? "关节速度" : "角速度"} <small>°/s</small>
              </span>
              <input
                aria-label="角速度 度每秒"
                type="number"
                min="0"
                step="any"
                value={angularSpeed}
                onChange={(e) => setAngularSpeed(Number(e.target.value))}
              />
            </label>
            {mode === "step" && (
              <>
                {space === "cartesian" && (
                  <label>
                    <span>
                      平移步长 <small>mm</small>
                    </span>
                    <input
                      aria-label="平移步长 mm"
                      type="number"
                      min="0"
                      step="any"
                      value={linearStep}
                      onChange={(e) => setLinearStep(Number(e.target.value))}
                    />
                  </label>
                )}
                <label>
                  <span>
                    角度步长 <small>°</small>
                  </span>
                  <input
                    aria-label="角度步长 度"
                    type="number"
                    min="0"
                    step="any"
                    value={angularStep}
                    onChange={(e) => setAngularStep(Number(e.target.value))}
                  />
                </label>
              </>
            )}
          </div>
          <div className="motion-state">
            <span>
              {s.action?.mode === "return"
                ? "返回收纳位"
                : motionReason || "就绪"}
            </span>
            {space === "cartesian" && connected && !s.tcp_configured && (
              <button
                className="motion-tcp"
                disabled={!deviceWritable || !!s.action}
                onClick={openTcp}
              >
                设置 TCP
              </button>
            )}
            {s.action && <b>{Math.round(s.action.progress * 100)}%</b>}
          </div>
        </section>
      </main>
      <section
        className="secondary-screen"
        id="api-view"
        role="tabpanel"
        aria-labelledby="page-api"
        hidden={page !== "api"}
      >
        <ApiPanel view={view} />
      </section>
      {error && (
        <div className="error-line" role="alert">
          <span>{error}</span>
          {view.error && (
            <button aria-label="关闭错误" onClick={() => api.clearError()}>
              ×
            </button>
          )}
        </div>
      )}
      <PanelDivider {...layout.divider("logs")} hidden={page !== "control"} />
      <section
        className="panel logs-panel"
        id="logs-panel"
        aria-label="运行日志"
        hidden={page !== "control" && page !== "logs"}
        data-fullscreen={page === "logs"}
        data-collapsed={page === "control" && layout.logsCollapsed}
      >
        <div className="logs-heading">
          <h2>日志</h2>
          <div className="journal-counts">
            {!s.journal.healthy && <span className="warning">写入异常</span>}
            <span>
              本次写入 <b>{s.journal.written}</b>
            </span>
            <span>
              待写 <b>{s.journal.pending}</b>
            </span>
            <span>
              丢弃 <b>{s.journal.dropped}</b>
            </span>
          </div>
          <span className="endpoint" title={view.baseUrl}>
            {view.baseUrl || "服务未连接"}
          </span>
          <button
            className="interface-button"
            disabled={!view.baseUrl}
            onClick={() => {
              cancel("interface_dialog");
              setTcpOpen(false);
              setInterfaceOpen(true);
            }}
          >
            连接信息
          </button>
          {page === "control" && (
            <button
              type="button"
              onClick={layout.toggleLogs}
              aria-expanded={!layout.logsCollapsed}
              aria-controls="logs-scroll"
            >
              {layout.logsCollapsed ? "展开" : "收起"}
            </button>
          )}
        </div>
        <div
          className="logs-scroll"
          id="logs-scroll"
          hidden={page !== "control"}
        >
          {view.logs.length ? (
            <table>
              <tbody>
                {view.logs.map((log) => (
                  <tr key={log.id}>
                    <td>
                      {new Date(log.ts_ms).toLocaleTimeString("zh-CN", {
                        hour12: false,
                      })}
                    </td>
                    <td className={`log-level ${log.level.toLowerCase()}`}>
                      {log.level}
                    </td>
                    <td>{log.source}</td>
                    <td title={log.code}>{log.code}</td>
                    <td title={log.message}>{log.message}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          ) : (
            <div className="logs-empty">暂无日志</div>
          )}
        </div>
        <div className="log-history-container" hidden={page !== "logs"}>
          <LogsPanel online={view.online} active={page === "logs"} />
        </div>
      </section>
      <ServoPanel
        key={
          view.connectionEpoch ??
          `${s.side}:${s.port ?? ""}:${["disconnected", "connecting", "closing"].includes(s.phase)}`
        }
        view={view}
        open={servoOpen}
        blocked={busy || enableInProgress || !!motionPending || !!safetyIntent}
        onClose={() => setServoOpen(false)}
        onStop={stop}
        onCalibrate={openCalibration}
      />
      <CalibrationPanel
        view={view}
        open={calibrationOpen}
        onClose={() => setCalibrationOpen(false)}
        onStop={stop}
      />
      <TeachingPanel
        view={view}
        open={teachingOpen}
        blocked={busy || enableInProgress || !!motionPending || !!safetyIntent}
        onClose={() => setTeachingOpen(false)}
        onStop={stop}
      />
      {tcpOpen && (
        <div
          className="modal-backdrop"
          onKeyDown={(e) => {
            if (e.key === "Escape") {
              setTcpOpen(false);
            }
          }}
        >
          <section
            className="tcp-dialog"
            role="dialog"
            aria-modal="true"
            aria-labelledby="tcp-title"
          >
            <div className="panel-heading">
              <h2 id="tcp-title">TCP 偏移（相对 Link7）</h2>
              <button
                aria-label="关闭 TCP 配置"
                onClick={() => setTcpOpen(false)}
              >
                ×
              </button>
            </div>
            <p className="dialog-hint">
              末端点动默认使用 Link7 法兰，可直接操作。安装工具后在这里填写相对
              Link7 的 TCP 偏移。
            </p>
            <div className="tcp-fields">
              {["X / mm", "Y / mm", "Z / mm", "Rx / °", "Ry / °", "Rz / °"].map(
                (name, i) => (
                  <label key={name}>
                    {name}
                    <input
                      autoFocus={i === 0}
                      type="number"
                      step="0.1"
                      value={tcpFields[i]}
                      onChange={(e) =>
                        setTcpFields((fields) =>
                          fields.map((field, index) =>
                            index === i ? e.target.value : field,
                          ),
                        )
                      }
                    />
                  </label>
                ),
              )}
            </div>
            <div className="dialog-actions">
              <button onClick={() => setTcpOpen(false)}>取消</button>
              <button
                className="primary"
                disabled={!deviceWritable || !!s.action}
                onClick={applyTcp}
              >
                应用
              </button>
            </div>
          </section>
        </div>
      )}
      {interfaceOpen && (
        <ConnectionInfoDialog onClose={() => setInterfaceOpen(false)} />
      )}
      {safetyIntent && (
        <ParkingDialog
          key={`${safetyIntent}:${safetyDevice.side}:${safetyDevice.port}`}
          intent={safetyIntent}
          side={safetyDevice.side}
          port={safetyDevice.port}
          view={view}
          stoppingFrom={safetyStopGeneration}
          enablePending={!!enablePending}
          motionPending={!!motionPending}
          onClose={dismissSafety}
        />
      )}
      <EscapeStop onStop={stop} />
    </div>
  );
}
type ParkingPose = { joints: JointPositions; savedAt: number };
function readParkingPose(key: string): {
  pose: ParkingPose | null;
  error: string;
} {
  try {
    const stored = localStorage.getItem(key);
    if (stored === null) return { pose: null, error: "" };
    const value: unknown = JSON.parse(stored);
    if (
      !value ||
      typeof value !== "object" ||
      !("joints" in value) ||
      !isJointPositions(value.joints) ||
      !("savedAt" in value) ||
      typeof value.savedAt !== "number" ||
      !Number.isFinite(value.savedAt) ||
      value.savedAt <= 0 ||
      !Number.isFinite(new Date(value.savedAt).getTime())
    )
      return { pose: null, error: "收纳位数据无效，请重新记录。" };
    return {
      pose: { joints: value.joints, savedAt: value.savedAt },
      error: "",
    };
  } catch {
    return { pose: null, error: "收纳位读取失败。" };
  }
}
function ParkingDialog({
  intent,
  side,
  port,
  view,
  stoppingFrom,
  enablePending,
  motionPending,
  onClose,
}: {
  intent: SafetyIntent;
  side: Side;
  port: string;
  view: View;
  stoppingFrom: number | null;
  enablePending: boolean;
  motionPending: boolean;
  onClose: () => void;
}) {
  const storageKey = `unitree.parking.v1:${side}:${encodeURIComponent(port)}`;
  const [stored, setStored] = useState(() =>
    port ? readParkingPose(storageKey) : { pose: null, error: "" },
  );
  const [speed, setSpeed] = useState("5");
  const [pending, setPending] = useState<"return" | "park" | "release" | null>(
    null,
  );
  const [message, setMessage] = useState("");
  const [error, setError] = useState("");
  const dialog = useRef<HTMLElement>(null);
  const alive = useRef(true);
  const returning = useRef<{ controller: AbortController; id: string } | null>(
    null,
  );
  const s = view.snapshot;
  const matchedDevice = !!port && s.port === port && s.side === side;
  const connected =
    view.online &&
    matchedDevice &&
    !["disconnected", "connecting", "closing"].includes(s.phase);
  const hasFeedback = connected && hasJointFeedback(view);
  const moving =
    view.online &&
    (!!s.action || motionPending || ["moving", "stopping"].includes(s.phase));
  const waitingForStop =
    view.online && stoppingFrom !== null && s.generation === stoppingFrom;
  const enabling = enablePending || s.phase === "enabling";
  const saveAllowed =
    hasFeedback && !moving && !waitingForStop && !enabling && !pending;
  const validSpeed =
    speed.trim() !== "" && Number.isFinite(Number(speed)) && Number(speed) > 0;
  const returnReason = !port
    ? "请先选择串口。"
    : !stored.pose
      ? "尚未记录收纳位。"
      : !view.online
        ? "状态连接已断开。"
        : !connected
          ? "请连接此机械臂。"
          : !view.hasControl
            ? "请先取得控制权。"
            : moving || waitingForStop
              ? "等待动作停止。"
              : enabling
                ? "请等待使能完成。"
                : !s.enabled || s.phase !== "ready"
                  ? "需要使能就绪后返回。"
                  : !hasFeedback
                    ? "需要完整、有效的七轴反馈。"
                    : !validSpeed
                      ? "速度须为有限正数。"
                      : "";
  const releaseAllowed =
    !pending &&
    !moving &&
    !waitingForStop &&
    (intent === "exit" || (connected && view.hasControl));
  const releaseLabel =
    intent === "exit" ? "退出" : intent === "disable" ? "失能" : "断开";
  const title = intent === "parking" ? "收纳" : `确认${releaseLabel}`;
  useEffect(() => {
    alive.current = true;
    const previous = document.activeElement;
    dialog.current?.focus();
    return () => {
      alive.current = false;
      returning.current?.controller.abort();
      if (previous instanceof HTMLElement && previous.isConnected)
        previous.focus();
    };
  }, []);
  const dismiss = () => {
    returning.current?.controller.abort();
    onClose();
  };
  const save = () => {
    const current = api.view;
    const snapshot = current.snapshot;
    const joints = snapshot.joints.map((joint) => joint.q);
    if (
      pending ||
      !port ||
      snapshot.side !== side ||
      snapshot.port !== port ||
      !hasJointFeedback(current) ||
      snapshot.action ||
      motionPending ||
      ["moving", "stopping", "enabling"].includes(snapshot.phase) ||
      enablePending ||
      (stoppingFrom !== null && snapshot.generation === stoppingFrom) ||
      !isJointPositions(joints)
    ) {
      setError("请停止动作并等待七轴反馈。");
      return;
    }
    const pose: ParkingPose = { joints, savedAt: Date.now() };
    try {
      localStorage.setItem(storageKey, JSON.stringify(pose));
      setStored({ pose, error: "" });
      setError("");
      setMessage("收纳位已记录");
    } catch {
      setError("收纳位保存失败。");
    }
  };
  const startReturn = async (disconnectAfter: boolean) => {
    if (pending || returning.current || returnReason || !stored.pose) return;
    if (api.view.snapshot.side !== side || api.view.snapshot.port !== port) {
      setError("连接设备已变化。");
      return;
    }
    const task = { controller: new AbortController(), id: crypto.randomUUID() };
    returning.current = task;
    setPending(disconnectAfter ? "park" : "return");
    setError("");
    setMessage("");
    try {
      const result = await api.returnToPose(
        {
          joints: stored.pose.joints,
          speed: Number(speed) / DEG,
          disconnect_after: disconnectAfter,
        },
        task.controller.signal,
        task.id,
      );
      if (
        !alive.current ||
        task.controller.signal.aborted ||
        returning.current !== task
      )
        return;
      if (intent === "exit" && disconnectAfter) {
        const current = api.view;
        const latest = current.snapshot.results.find(
          (item) => item.request_id === task.id,
        );
        if (
          !current.online ||
          latest?.status !== "completed" ||
          latest.code !== "parked" ||
          result.request_id !== task.id ||
          current.snapshot.enabled ||
          current.snapshot.phase !== "disconnected" ||
          current.snapshot.action
        )
          throw new Error("尚未确认收纳完成。");
        await api.closeApp();
        if (alive.current && !task.controller.signal.aborted) onClose();
      } else if (disconnectAfter && intent !== "parking") {
        onClose();
      } else {
        setMessage(
          disconnectAfter ? "已收纳并断开。" : "已返回收纳位，保持使能。",
        );
      }
    } catch (error) {
      if (alive.current && returning.current === task)
        setError(error instanceof Error ? error.message : "返回未完成");
    } finally {
      if (returning.current === task) {
        returning.current = null;
        if (alive.current) setPending(null);
      }
    }
  };
  const release = async () => {
    if (!releaseAllowed) return;
    setPending("release");
    setError("");
    try {
      if (intent === "exit") await api.closeApp(true);
      else
        await api.command(intent === "disable" ? "disable" : "disconnect", {
          allow_release: true,
        });
      if (alive.current) onClose();
    } catch (error) {
      if (alive.current)
        setError(
          error instanceof Error ? error.message : `${releaseLabel}失败`,
        );
    } finally {
      if (alive.current) setPending(null);
    }
  };
  const progress =
    s.action && s.action.request_id === returning.current?.id
      ? s.action.progress
      : null;
  return (
    <div
      className="modal-backdrop"
      onClick={(event) => {
        if (event.target === event.currentTarget) dismiss();
      }}
      onKeyDown={(event) => {
        if (event.key === "Escape") {
          event.preventDefault();
          dismiss();
          void api.hostStop("parking_escape").catch((error) => api.fail(error));
        } else if (event.key === "Tab") {
          const controls = Array.from(
            dialog.current?.querySelectorAll<HTMLElement>(
              'button:not(:disabled), input:not(:disabled), [tabindex="0"]',
            ) ?? [],
          );
          const first = controls[0];
          const last = controls[controls.length - 1];
          if (
            event.shiftKey &&
            (document.activeElement === first ||
              document.activeElement === dialog.current)
          ) {
            event.preventDefault();
            last?.focus();
          } else if (
            !event.shiftKey &&
            (document.activeElement === last ||
              document.activeElement === dialog.current)
          ) {
            event.preventDefault();
            first?.focus();
          }
        }
      }}
    >
      <section
        ref={dialog}
        tabIndex={-1}
        className="tcp-dialog parking-dialog"
        role="dialog"
        aria-modal="true"
        aria-labelledby="parking-title"
        aria-describedby="parking-support"
      >
        <div className="panel-heading">
          <h2 id="parking-title">{title}</h2>
          <button aria-label="关闭收纳窗口" onClick={dismiss}>
            ×
          </button>
        </div>
        <div className="parking-body">
          {intent !== "parking" && (
            <p className="parking-warning">
              {releaseLabel}会停止保持，请确认机械臂已有支撑。
            </p>
          )}
          <p id="parking-support">将机械臂放在支撑上，再记录收纳位。</p>
          <div className="parking-device">
            <b>{side === "right" ? "右臂" : "左臂"}</b>
            <span title={port}>{port || "未选择串口"}</span>
          </div>
          <div className="parking-record">
            <span>
              {stored.pose
                ? `记录于 ${new Date(stored.pose.savedAt).toLocaleString("zh-CN", { hour12: false })}`
                : "尚未记录收纳位"}
            </span>
            <button disabled={!saveAllowed} onClick={save}>
              {stored.pose ? "更新收纳位" : "记录当前姿态"}
            </button>
          </div>
          {stored.pose && (
            <dl
              className="parking-joints"
              aria-label="已保存的关节位置，单位度"
            >
              {stored.pose.joints.map((q, index) => (
                <div key={index}>
                  <dt>J{index + 1}</dt>
                  <dd>{fmt(q * DEG, 1)}°</dd>
                </div>
              ))}
            </dl>
          )}
          <label className="parking-speed" htmlFor="parking-speed">
            <span>返回速度</span>
            <input
              id="parking-speed"
              type="number"
              min="0"
              step="any"
              value={speed}
              disabled={!!pending}
              onChange={(event) => setSpeed(event.target.value)}
            />
            <span>°/s</span>
          </label>
          {returnReason && !pending && (
            <p className="parking-hint">{returnReason}</p>
          )}
          {(stored.error || error) && (
            <p className="parking-error" role="alert">
              {error || stored.error}
            </p>
          )}
          <p className="parking-status" role="status" aria-live="polite">
            {pending === "release"
              ? `正在${releaseLabel}…`
              : pending
                ? `正在返回收纳位${progress === null ? "…" : ` · ${Math.round(progress * 100)}%`}`
                : message}
          </p>
        </div>
        <div className="parking-choices">
          {intent === "parking" && (
            <button
              disabled={!!pending || !!returnReason}
              onClick={() => void startReturn(false)}
            >
              返回收纳位
            </button>
          )}
          <button
            className="primary"
            disabled={!!pending || !!returnReason}
            onClick={() => void startReturn(true)}
          >
            {intent === "exit" ? "收纳后退出" : "收纳后断开"}
          </button>
          {intent !== "parking" && (
            <button disabled={!releaseAllowed} onClick={() => void release()}>
              已支撑，{releaseLabel}
            </button>
          )}
        </div>
        <div className="dialog-actions parking-footer">
          {(pending === "return" ||
            pending === "park" ||
            moving ||
            waitingForStop) && (
            <button
              onClick={() => {
                setMessage("");
                void api
                  .hostStop("parking_stop_button")
                  .catch((error) => api.fail(error));
              }}
            >
              停止
            </button>
          )}
          <button onClick={dismiss}>{pending ? "停止并取消" : "取消"}</button>
        </div>
      </section>
    </div>
  );
}
function EscapeStop({ onStop }: { onStop: () => void }) {
  const callback = useRef(onStop);
  callback.current = onStop;
  useEffect(() => {
    const handler = (event: KeyboardEvent) => {
      if (event.key === "Escape" && !event.defaultPrevented) callback.current();
    };
    window.addEventListener("keydown", handler);
    return () => window.removeEventListener("keydown", handler);
  }, []);
  return null;
}

function ConnectionInfoDialog({ onClose }: { onClose: () => void }) {
  const [info] = useState(() => api.getConnectionInfo());
  const [copied, setCopied] = useState<
    Partial<Record<"base_url" | "token", string>>
  >({});
  const baseInput = useRef<HTMLInputElement>(null);
  const tokenInput = useRef<HTMLInputElement>(null);
  const copy = async (field: "base_url" | "token") => {
    const ok = await api.copyConnectionInfo(field);
    if (!ok) {
      const input =
        field === "base_url" ? baseInput.current : tokenInput.current;
      input?.focus();
      input?.select();
    }
    setCopied((status) => ({ ...status, [field]: ok ? "已复制" : "已选中" }));
  };
  return (
    <div
      className="modal-backdrop"
      onClick={(event) => {
        if (event.target === event.currentTarget) onClose();
      }}
      onKeyDown={(event) => {
        if (event.key === "Escape") onClose();
      }}
    >
      <section
        className="tcp-dialog interface-dialog"
        role="dialog"
        aria-modal="true"
        aria-labelledby="interface-title"
      >
        <div className="panel-heading">
          <h2 id="interface-title">本机接口</h2>
          <button aria-label="关闭接口信息" onClick={onClose}>
            ×
          </button>
        </div>
        <div className="interface-fields">
          {(["base_url", "token"] as const).map((field, index) => (
            <label key={field} htmlFor={`interface-${field}`}>
              <span>{field === "base_url" ? "服务地址" : "访问令牌"}</span>
              <div className="interface-field">
                <input
                  id={`interface-${field}`}
                  ref={index === 0 ? baseInput : tokenInput}
                  autoFocus={index === 0}
                  readOnly
                  value={info?.[field] ?? ""}
                  autoComplete="off"
                  spellCheck={false}
                  onFocus={(event) => event.currentTarget.select()}
                />
                <button
                  disabled={!info}
                  onClick={() => void copy(field)}
                  aria-label={`复制 ${field}`}
                >
                  {copied[field] ?? "复制"}
                </button>
              </div>
            </label>
          ))}
        </div>
        <div className="dialog-actions">
          <button onClick={onClose}>关闭</button>
        </div>
      </section>
    </div>
  );
}
