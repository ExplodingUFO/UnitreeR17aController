import { useEffect, useRef, useState } from "react";
import {
  api,
  createServoTuning,
  sameServoTuning,
  SERVO_TUNING_RANGES,
  type JointTuning,
  type ServoTuning,
  type View,
} from "./api";

const fields: {
  key: keyof JointTuning;
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

type DraftJoint = Record<keyof JointTuning, string>;
type Draft = { friction_enabled: boolean; joints: DraftJoint[] };
function toDraft(config: ServoTuning): Draft {
  return {
    friction_enabled: config.friction_enabled,
    joints: config.joints.map(
      (joint) =>
        Object.fromEntries(
          fields.map(({ key }) => [key, String(joint[key])]),
        ) as DraftJoint,
    ),
  };
}
function readDraft(draft: Draft): ServoTuning | null {
  if (
    draft.joints.length !== 7 ||
    draft.joints.some((joint) =>
      fields.some(({ key }) => {
        const value = Number(joint[key]);
        const [min, max] = SERVO_TUNING_RANGES[key];
        return (
          !joint[key].trim() ||
          !Number.isFinite(value) ||
          value < min ||
          value > max
        );
      }),
    )
  )
    return null;
  return {
    friction_enabled: draft.friction_enabled,
    joints: draft.joints.map((joint) => ({
      kd: Number(joint.kd),
      velocity_ff: Number(joint.velocity_ff),
      coulomb: Number(joint.coulomb),
      viscous: Number(joint.viscous),
      velocity_scale: Number(joint.velocity_scale),
      max_friction: Number(joint.max_friction),
    })) as ServoTuning["joints"],
  };
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
    const field = lastField.current?.isConnected
      ? lastField.current
      : dialog.current?.querySelector<HTMLInputElement>('input[type="number"]');
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
      const [min, max] = SERVO_TUNING_RANGES[field.key];
      const number = Number(joint[field.key]);
      return !joint[field.key].trim() ||
        !Number.isFinite(number) ||
        number < min ||
        number > max
        ? [{ index, ...field, min, max }]
        : [];
    }),
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
    ? `J${invalid.index + 1} ${invalid.label}：${invalid.min}–${invalid.max}${invalid.unit === "权重" ? "" : ` ${invalid.unit}`}`
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
              "button:not(:disabled), input:not(:disabled), textarea:not(:disabled)",
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
                  <tr key={index}>
                    <th scope="row">J{index + 1}</th>
                    {fields.map(({ key, label, unit, step }) => {
                      const [min, max] = SERVO_TUNING_RANGES[key];
                      const number = Number(joint[key]);
                      const invalid =
                        !joint[key].trim() ||
                        !Number.isFinite(number) ||
                        number < min ||
                        number > max;
                      return (
                        <td key={key}>
                          <input
                            type="number"
                            aria-label={`J${index + 1} ${label}`}
                            aria-invalid={invalid}
                            aria-describedby={
                              invalid ? "servo-feedback" : undefined
                            }
                            title={`${min}–${max} ${unit}；已应用 ${applied.joints[index][key]}`}
                            min={min}
                            max={max}
                            step={step}
                            value={joint[key]}
                            data-changed={
                              invalid || number !== applied.joints[index][key]
                            }
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
