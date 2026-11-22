import { useEffect, useRef, useState } from "react";
import {
  api,
  createServoTuning,
  createStribeckTuning,
  sameServoTuning,
  isServoTuning,
  DEFAULT_MOTION_TUNING,
  type MotionTuning,
  type JointTuning,
  type FrictionCurve,
  type ServoTuning,
  type View,
} from "./api";

export type MotionDraft = Record<keyof MotionTuning, string>;
const motionFields = [
  ["joint_speed", "关节速度", "rad/s"],
  ["acceleration", "加速度", "rad/s²"],
  ["jerk", "加加速度", "rad/s³"],
] as const;
export function toMotionDraft(config: MotionTuning): MotionDraft {
  return {
    joint_speed: String(config.joint_speed),
    acceleration: String(config.acceleration),
    jerk: String(config.jerk),
  };
}
export function readMotionDraft(draft: MotionDraft): MotionTuning | null {
  if (
    motionFields.some(
      ([key]) =>
        !draft[key].trim() ||
        !Number.isFinite(Number(draft[key])) ||
        Number(draft[key]) <= 0,
    )
  )
    return null;
  return {
    joint_speed: Number(draft.joint_speed),
    acceleration: Number(draft.acceleration),
    jerk: Number(draft.jerk),
  };
}
export function MotionTuningFields({
  draft,
  disabled = false,
  onChange,
}: {
  draft: MotionDraft;
  disabled?: boolean;
  onChange: (draft: MotionDraft) => void;
}) {
  return (
    <div className="motion-preference-fields">
      {motionFields.map(([key, label, unit]) => (
        <label key={key}>
          <span>
            {label} <small>{unit}</small>
          </span>
          <input
            type="number"
            aria-label={`运动偏好 ${label} ${unit}`}
            min={0}
            step="any"
            value={draft[key]}
            disabled={disabled}
            aria-invalid={
              !draft[key].trim() ||
              !Number.isFinite(Number(draft[key])) ||
              Number(draft[key]) <= 0
            }
            onChange={(event) =>
              onChange({ ...draft, [key]: event.target.value })
            }
          />
        </label>
      ))}
    </div>
  );
}

const fields: {
  key: Exclude<keyof JointTuning, "stribeck">;
  label: string;
  unit: string;
  title: string;
  step: number;
}[] = [
  {
    key: "kd",
    label: "阻尼 Kd",
    unit: "Nm·s/rad",
    title: "关节阻尼",
    step: 0.1,
  },
  {
    key: "velocity_ff",
    label: "速度前馈",
    unit: "权重",
    title: "速度前馈权重，仅用于最终下发",
    step: 0.01,
  },
  {
    key: "coulomb",
    label: "库仑 Fc",
    unit: "Nm",
    title: "库仑摩擦补偿",
    step: 0.01,
  },
  {
    key: "viscous",
    label: "黏性 B",
    unit: "Nm·s/rad",
    title: "黏性摩擦补偿",
    step: 0.01,
  },
  {
    key: "velocity_scale",
    label: "过零速度",
    unit: "rad/s",
    title: "tanh 平滑过零速度",
    step: 0.001,
  },
  {
    key: "max_friction",
    label: "补偿限幅",
    unit: "Nm",
    title: "摩擦补偿扭矩限幅",
    step: 0.01,
  },
];

const curveFields = [
  ["static_friction", "Fs", "Nm"],
  ["coulomb", "Fc", "Nm"],
  ["viscous", "B", "Nm·s/rad"],
  ["stribeck_velocity", "Vs", "rad/s"],
] as const;
type DraftCurve = Record<keyof FrictionCurve, string>;
type DraftJoint = Record<Exclude<keyof JointTuning, "stribeck">, string> & {
  stribeck: {
    enabled: boolean;
    positive: DraftCurve;
    negative: DraftCurve;
  };
};
type Draft = { friction_enabled: boolean; joints: DraftJoint[] };
const toCurveDraft = (curve: FrictionCurve): DraftCurve => ({
  static_friction: String(curve.static_friction),
  coulomb: String(curve.coulomb),
  viscous: String(curve.viscous),
  stribeck_velocity: String(curve.stribeck_velocity),
});
function toDraft(config: ServoTuning): Draft {
  return {
    friction_enabled: config.friction_enabled,
    joints: config.joints.map((joint) => {
      const model = joint.stribeck ?? createStribeckTuning();
      return {
        ...Object.fromEntries(
          fields.map(({ key }) => [key, String(joint[key])]),
        ),
        stribeck: {
          enabled: model.enabled,
          positive: toCurveDraft(model.positive),
          negative: toCurveDraft(model.negative),
        },
      } as DraftJoint;
    }),
  };
}
const readCurveDraft = (curve: DraftCurve): FrictionCurve => ({
  static_friction: Number(curve.static_friction),
  coulomb: Number(curve.coulomb),
  viscous: Number(curve.viscous),
  stribeck_velocity: Number(curve.stribeck_velocity),
});
function readDraft(draft: Draft): ServoTuning | null {
  if (
    draft.joints.length !== 7 ||
    draft.joints.some(
      (joint) =>
        fields.some(({ key }) => !joint[key].trim()) ||
        (["positive", "negative"] as const).some((direction) =>
          curveFields.some(([key]) => !joint.stribeck[direction][key].trim()),
        ),
    )
  )
    return null;
  const config = {
    friction_enabled: draft.friction_enabled,
    joints: draft.joints.map((joint) => ({
      kd: Number(joint.kd),
      velocity_ff: Number(joint.velocity_ff),
      coulomb: Number(joint.coulomb),
      viscous: Number(joint.viscous),
      velocity_scale: Number(joint.velocity_scale),
      max_friction: Number(joint.max_friction),
      stribeck: {
        enabled: joint.stribeck.enabled,
        positive: readCurveDraft(joint.stribeck.positive),
        negative: readCurveDraft(joint.stribeck.negative),
      },
    })) as ServoTuning["joints"],
  };
  return isServoTuning(config) ? config : null;
}

export default function ServoPanel({
  view,
  blocked,
  open,
  onClose,
  onStop,
  onCalibrate,
}: {
  view: View;
  blocked: boolean;
  open: boolean;
  onClose: () => void;
  onStop: () => void;
  onCalibrate?: () => void;
}) {
  const s = view.snapshot;
  const applied = s.servo_tuning ?? createServoTuning();
  const appliedKey = JSON.stringify(applied);
  const connectionKey = ["disconnected", "connecting", "closing"].includes(
    s.phase,
  )
    ? "disconnected"
    : `${s.side}:${s.port ?? ""}`;
  const previous = useRef({ connectionKey, applied });
  const [draft, setDraft] = useState(() => toDraft(applied));
  const [selectedAxis, setSelectedAxis] = useState(0);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState("");
  const [copyFeedback, setCopyFeedback] = useState("");
  const [copyText, setCopyText] = useState("");
  const copyInput = useRef<HTMLTextAreaElement>(null);
  const dialog = useRef<HTMLElement>(null);
  const lastField = useRef<HTMLInputElement>(null);
  const applying = useRef<AbortController | null>(null);
  const alive = useRef(true);
  const config = readDraft(draft);
  const dirty = !config || !sameServoTuning(config, applied);
  const appliedMotion = s.motion_tuning ?? DEFAULT_MOTION_TUNING;
  const motionKey = JSON.stringify(appliedMotion);
  const motionPrevious = useRef({ connectionKey, motionKey });
  const [motionDraft, setMotionDraft] = useState(() =>
    toMotionDraft(appliedMotion),
  );
  const [motionPending, setMotionPending] = useState<string | null>(null);
  const motionConfig = readMotionDraft(motionDraft);
  const motionDirty =
    !motionConfig || JSON.stringify(motionConfig) !== motionKey;
  useEffect(() => {
    const before = motionPrevious.current;
    motionPrevious.current = { connectionKey, motionKey };
    setMotionDraft((current) =>
      before.connectionKey !== connectionKey ||
      JSON.stringify(readMotionDraft(current)) === before.motionKey
        ? toMotionDraft(appliedMotion)
        : current,
    );
    if (before.connectionKey !== connectionKey) setMotionPending(null);
  }, [connectionKey, motionKey]);
  useEffect(() => {
    if (!motionPending) return;
    const result =
      s.results?.find((item) => item.request_id === motionPending) ??
      (s.last_result?.request_id === motionPending ? s.last_result : null);
    if (!result) return;
    if (["rejected", "failed", "cancelled"].includes(result.status)) {
      setError(result.message || "运动偏好未应用");
      setMotionPending(null);
    } else if (
      ["accepted", "completed"].includes(result.status) &&
      !motionDirty
    ) {
      setCopyFeedback("运动偏好已应用");
      setMotionPending(null);
    }
  }, [s, motionPending, motionDirty]);

  useEffect(() => {
    const before = previous.current;
    previous.current = { connectionKey, applied };
    setDraft((current) => {
      const parsed = readDraft(current);
      return before.connectionKey !== connectionKey ||
        (parsed && sameServoTuning(parsed, before.applied))
        ? toDraft(applied)
        : current;
    });
    if (before.connectionKey !== connectionKey) {
      setError("");
      setCopyFeedback("");
      setCopyText("");
    }
  }, [connectionKey, appliedKey]);
  useEffect(() => {
    alive.current = true;
    return () => {
      alive.current = false;
      applying.current?.abort();
    };
  }, []);
  useEffect(() => {
    if (!open) return;
    const previous = document.activeElement;
    const field =
      lastField.current?.isConnected &&
      lastField.current.getClientRects().length &&
      !lastField.current.closest("details:not([open])")
        ? lastField.current
        : Array.from(
            dialog.current?.querySelectorAll<HTMLInputElement>(
              'input[type="number"]',
            ) ?? [],
          ).find(
            (input) =>
              input.getClientRects().length &&
              !input.closest("details:not([open])"),
          );
    (field ?? dialog.current)?.focus();
    return () => {
      if (previous instanceof HTMLElement && previous.isConnected)
        previous.focus();
    };
  }, [open]);
  useEffect(() => {
    if (!open || !copyText) return;
    copyInput.current?.focus();
    copyInput.current?.select();
  }, [open, copyText]);

  const invalid = draft.joints.flatMap((joint, index) =>
    fields.flatMap((field) => {
      const number = Number(joint[field.key]);
      return !joint[field.key].trim() ||
        !Number.isFinite(number) ||
        number < 0 ||
        (field.key === "velocity_scale" && number === 0)
        ? [{ index, ...field }]
        : [];
    }),
  )[0];
  const invalidCurve = draft.joints.flatMap((joint, index) =>
    (["positive", "negative"] as const).flatMap((direction) =>
      curveFields.flatMap(([key, label, unit]) => {
        const text = joint.stribeck[direction][key];
        const number = Number(text);
        return !text.trim() ||
          !Number.isFinite(number) ||
          (key === "stribeck_velocity" ? number <= 0 : number < 0)
          ? [{ index, direction, key, label, unit }]
          : [];
      }),
    ),
  )[0];
  const applyReason =
    !view.online || s.phase === "disconnected"
      ? "未连接"
      : !view.hasControl
        ? "未取得控制权"
        : s.phase === "connecting"
          ? "连接中"
          : s.phase === "fault"
            ? "故障"
            : s.phase === "closing"
              ? "关闭中"
              : s.phase === "enabling"
                ? "使能中"
                : s.phase === "stopping"
                  ? "停止中"
                  : s.phase === "moving" || s.action
                    ? "运动中"
                    : blocked
                      ? "等待当前操作完成"
                      : "";
  const validation = invalid
    ? `J${invalid.index + 1} ${invalid.label}：须为有限${invalid.key === "velocity_scale" ? "正数" : "非负数"}${invalid.unit === "权重" ? "" : ` ${invalid.unit}`}`
    : invalidCurve
      ? `J${invalidCurve.index + 1} ${invalidCurve.direction === "positive" ? "正向" : "反向"} ${invalidCurve.label}：须为有限${invalidCurve.key === "stribeck_velocity" ? "正数" : "非负数"} ${invalidCurve.unit}`
      : "";
  const feedback = error || validation || copyFeedback || applyReason;
  const status = pending
    ? "等待确认…"
    : error && !dirty
      ? "未确认"
      : dirty
        ? "已修改"
        : connectionKey === "disconnected"
          ? "草稿"
          : "已应用";

  function edited() {
    setError("");
    setCopyFeedback("");
    setCopyText("");
  }
  async function apply() {
    if (!config || !dirty || applyReason || applying.current) return;
    const controller = new AbortController();
    applying.current = controller;
    setPending(true);
    edited();
    try {
      await api.configureServo(config, controller.signal);
    } catch (error) {
      if (alive.current)
        setError(error instanceof Error ? error.message : "参数未应用");
    } finally {
      if (applying.current === controller) {
        applying.current = null;
        if (alive.current) setPending(false);
      }
    }
  }
  async function copy() {
    if (!config) return;
    const text = JSON.stringify(config, null, 2);
    setError("");
    try {
      if (!navigator.clipboard?.writeText) throw new Error("clipboard");
      await navigator.clipboard.writeText(text);
      if (alive.current) {
        setCopyText("");
        setCopyFeedback("已复制草稿 JSON");
      }
    } catch {
      if (!alive.current) return;
      setCopyText(text);
      setCopyFeedback("已选中，请手动复制");
      requestAnimationFrame(() => {
        copyInput.current?.focus();
        copyInput.current?.select();
      });
    }
  }
  async function applyMotion() {
    if (
      !motionConfig ||
      !motionDirty ||
      applyReason ||
      motionPending ||
      pending
    )
      return;
    const requestId = crypto.randomUUID();
    setMotionPending(requestId);
    edited();
    try {
      await api.command(
        "configure_motion",
        { config: motionConfig },
        requestId,
        s.generation,
      );
    } catch (error) {
      if (alive.current) {
        setError(error instanceof Error ? error.message : "运动偏好未确认");
        setMotionPending(null);
      }
    }
  }

  return (
    <div
      className="modal-backdrop servo-backdrop"
      hidden={!open}
      onClick={(event) => {
        if (event.target === event.currentTarget) onClose();
      }}
      onKeyDown={(event) => {
        if (event.key === "Escape") {
          event.preventDefault();
          onStop();
          onClose();
        } else if (event.key === "Tab") {
          const controls = Array.from(
            dialog.current?.querySelectorAll<HTMLElement>(
              "button:not(:disabled), input:not(:disabled), textarea:not(:disabled), summary",
            ) ?? [],
          ).filter(
            (element) =>
              element.getClientRects().length &&
              (element.tagName === "SUMMARY" ||
                !element.closest("details:not([open])")),
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
        }
      }}
    >
      <section
        ref={dialog}
        id="servo-dialog"
        className="panel servo-panel"
        role="dialog"
        aria-modal="true"
        aria-labelledby="servo-title"
        tabIndex={-1}
        onFocusCapture={(event) => {
          if (
            event.target instanceof HTMLInputElement &&
            event.target.type === "number"
          )
            lastField.current = event.target;
        }}
      >
        <div className="secondary-toolbar servo-toolbar">
          <h2 id="servo-title">伺服参数</h2>
          <span className="servo-context">
            本次连接 · r{s.servo_tuning_revision ?? 0}
          </span>
          <span
            className="servo-draft-state"
            data-dirty={dirty}
            role="status"
            aria-live="polite"
          >
            {status}
          </span>
          <label className="servo-friction">
            <input
              type="checkbox"
              role="switch"
              checked={draft.friction_enabled}
              onChange={(event) => {
                edited();
                setDraft((current) => ({
                  ...current,
                  friction_enabled: event.target.checked,
                }));
              }}
            />
            摩擦补偿
          </label>
          <button className="stop-button" onClick={onStop}>
            停止 <kbd>Esc</kbd>
          </button>
          {onCalibrate && <button onClick={onCalibrate}>低速校准</button>}
          <button
            className="servo-close"
            aria-label="关闭伺服参数"
            title="关闭"
            onClick={onClose}
          >
            ×
          </button>
        </div>
        <div className="servo-body">
          <details className="motion-preferences servo-motion">
            <summary>
              运动偏好 <small>速度 / 加速度 / 加加速度</small>
            </summary>
            <MotionTuningFields
              draft={motionDraft}
              disabled={!!motionPending || pending}
              onChange={(value) => {
                edited();
                setMotionDraft(value);
              }}
            />
            <div className="motion-preference-actions">
              <small>
                {motionConfig ? "用于点动与轨迹规划" : "请输入大于 0 的数值"}
              </small>
              <button
                disabled={!motionDirty || !!motionPending}
                onClick={() => setMotionDraft(toMotionDraft(appliedMotion))}
              >
                重置
              </button>
              <button
                disabled={
                  !motionDirty ||
                  !motionConfig ||
                  !!applyReason ||
                  !!motionPending ||
                  pending
                }
                onClick={() => void applyMotion()}
              >
                {motionPending ? "等待确认…" : "应用运动偏好"}
              </button>
            </div>
          </details>
          <div className="servo-table-wrap">
            <table className="servo-table" aria-label="七轴伺服参数草稿">
              <thead>
                <tr>
                  <th scope="col">轴</th>
                  {fields.map(({ key, label, unit, title }) => (
                    <th key={key} scope="col" title={title}>
                      {label}
                      <small>{unit}</small>
                    </th>
                  ))}
                </tr>
              </thead>
              <tbody>
                {draft.joints.map((joint, index) => (
                  <tr key={index} data-selected={selectedAxis === index}>
                    <th scope="row">
                      <button
                        aria-label={`选择 J${index + 1} 摩擦模型`}
                        aria-pressed={selectedAxis === index}
                        onClick={() => setSelectedAxis(index)}
                      >
                        J{index + 1}
                      </button>
                    </th>
                    {fields.map(({ key, label, unit, step }) => {
                      const number = Number(joint[key]);
                      const invalid =
                        !joint[key].trim() ||
                        !Number.isFinite(number) ||
                        number < 0 ||
                        (key === "velocity_scale" && number === 0);
                      return (
                        <td key={key}>
                          <input
                            type="number"
                            aria-label={`J${index + 1} ${label}`}
                            aria-invalid={invalid}
                            aria-describedby={
                              invalid ? "servo-feedback" : undefined
                            }
                            title={`${key === "velocity_scale" ? "大于 0" : "不小于 0"} ${unit}；已应用 ${applied.joints[index][key]}`}
                            min={0}
                            step={step}
                            value={joint[key]}
                            data-changed={
                              invalid || number !== applied.joints[index][key]
                            }
                            onFocus={() => setSelectedAxis(index)}
                            onChange={(event) => {
                              const value = event.target.value;
                              edited();
                              setDraft((current) => ({
                                ...current,
                                joints: current.joints.map((joint, i) =>
                                  i === index
                                    ? { ...joint, [key]: value }
                                    : joint,
                                ),
                              }));
                            }}
                          />
                        </td>
                      );
                    })}
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
          <section
            className="servo-stribeck"
            aria-label={`J${selectedAxis + 1} 摩擦模型`}
          >
            <div className="servo-model-heading">
              <strong>J{selectedAxis + 1} · 摩擦模型</strong>
              <small>
                {draft.joints[selectedAxis].stribeck.enabled
                  ? "双向 Stribeck"
                  : "库仑 / 黏性"}
              </small>
              <label className="servo-friction">
                <input
                  type="checkbox"
                  role="switch"
                  aria-label={`J${selectedAxis + 1} Stribeck`}
                  checked={draft.joints[selectedAxis].stribeck.enabled}
                  onChange={(event) => {
                    const enabled = event.target.checked;
                    edited();
                    setDraft((current) => ({
                      ...current,
                      joints: current.joints.map((joint, index) =>
                        index === selectedAxis
                          ? {
                              ...joint,
                              stribeck: { ...joint.stribeck, enabled },
                            }
                          : joint,
                      ),
                    }));
                  }}
                />
                Stribeck
              </label>
            </div>
            <div className="servo-curve-fields">
              <span />
              {curveFields.map(([key, label, unit]) => (
                <span key={key}>
                  {label} <small>{unit}</small>
                </span>
              ))}
              {(["positive", "negative"] as const).map((direction) => {
                const label = direction === "positive" ? "正向" : "反向";
                const curve = draft.joints[selectedAxis].stribeck[direction];
                const original = (applied.joints[selectedAxis].stribeck ??
                  createStribeckTuning())[direction];
                return (
                  <div className="servo-curve-row" key={direction}>
                    <span>{label}</span>
                    {curveFields.map(([key, name]) => {
                      const number = Number(curve[key]);
                      const invalid =
                        !curve[key].trim() ||
                        !Number.isFinite(number) ||
                        (key === "stribeck_velocity"
                          ? number <= 0
                          : number < 0);
                      return (
                        <input
                          key={key}
                          type="number"
                          aria-label={`J${selectedAxis + 1} ${label} ${name}`}
                          aria-invalid={invalid}
                          aria-describedby={
                            invalid ? "servo-feedback" : undefined
                          }
                          data-changed={invalid || number !== original[key]}
                          min={0}
                          step="any"
                          value={curve[key]}
                          onChange={(event) => {
                            const value = event.target.value;
                            edited();
                            setDraft((current) => ({
                              ...current,
                              joints: current.joints.map((joint, index) =>
                                index === selectedAxis
                                  ? {
                                      ...joint,
                                      stribeck: {
                                        ...joint.stribeck,
                                        [direction]: {
                                          ...joint.stribeck[direction],
                                          [key]: value,
                                        },
                                      },
                                    }
                                  : joint,
                              ),
                            }));
                          }}
                        />
                      );
                    })}
                  </div>
                );
              })}
            </div>
          </section>
          {copyText && (
            <textarea
              className="servo-json"
              ref={copyInput}
              aria-label="参数草稿 JSON，请手动复制"
              value={copyText}
              readOnly
              spellCheck={false}
              onFocus={(event) => event.currentTarget.select()}
            />
          )}
        </div>
        <div className="servo-actions">
          <p
            id="servo-feedback"
            className={error || validation ? "danger" : ""}
            role={error ? "alert" : "status"}
            title={feedback}
          >
            {feedback}
          </p>
          <button
            disabled={!config}
            title="复制当前草稿参数"
            onClick={() => void copy()}
          >
            复制 JSON
          </button>
          <button
            title="将默认参数填入草稿"
            onClick={() => {
              edited();
              setDraft(toDraft(createServoTuning()));
            }}
          >
            基线
          </button>
          <button
            disabled={!dirty}
            onClick={() => {
              edited();
              setDraft(toDraft(applied));
            }}
          >
            重置草稿
          </button>
          <button
            className="primary"
            disabled={pending || !config || !dirty || !!applyReason}
            aria-busy={pending}
            title={validation || applyReason || "应用于本次连接"}
            onClick={() => void apply()}
          >
            {pending ? "等待确认…" : "应用"}
          </button>
          <button onClick={onClose}>关闭</button>
        </div>
      </section>
    </div>
  );
}
