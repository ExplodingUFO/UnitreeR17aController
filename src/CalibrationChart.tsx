import { useMemo, useState } from "react";
import type {
  CalibrationPoint,
  CalibrationReport,
  CalibrationTrial,
  Space,
} from "./api";

const DEG = 180 / Math.PI;
type Metric = "position" | "velocity" | "error";
const metrics: [Metric, string][] = [
  ["position", "位置"],
  ["velocity", "速度"],
  ["error", "误差"],
];
const series = [
  { key: "reference", label: "控制参考 reference" },
  { key: "command", label: "成功发送 command" },
  { key: "feedback", label: "实测 feedback" },
] as const;
type Series = (typeof series)[number]["key"];
type Live = {
  id: string;
  axis: number;
  space?: Space;
  points: CalibrationPoint[];
};
type Plot = { key: string; label: string; points: CalibrationPoint[] };

export function calibrationTrials(
  report: CalibrationReport,
  axis: number,
  space: Space = "joint",
) {
  return report.trials.filter((trial) => {
    const trialSpace =
      trial.space ??
      (trial.phase.startsWith("cartesian_") ? "cartesian" : "joint");
    if (trialSpace !== space) return false;
    // Cartesian motion-axis X/Y/Z is not a measured joint: all seven respond.
    if (space === "cartesian") return true;
    const measuredAxis = trial.axis ?? report.settings.axis;
    return measuredAxis == null
      ? report.curves?.some(
          (point) => point.trial === trial.index && point.axis === axis,
        )
      : measuredAxis === axis;
  });
}
export function calibrationTrialLabel(
  report: CalibrationReport,
  trial: CalibrationTrial,
  axis: number,
) {
  if (trial.phase === "cartesian_before") return "末端动作 · 调整前";
  if (trial.phase === "cartesian_after") return "末端动作 · 调整后";
  if (trial.phase === "baseline") return "基线";
  if (trial.phase === "after") return "调整后";
  if (trial.phase === "baseline_check") return "基线复测";
  if (trial.phase === "validation") return "验证";
  const config = trial.config.joints[axis],
    original = report.baseline.joints[axis];
  return (
    [
      config.coulomb !== original.coulomb ? `Fc ${config.coulomb}` : "",
      config.velocity_ff !== original.velocity_ff
        ? `前馈 ${config.velocity_ff}`
        : "",
      config.kd !== original.kd ? `Kd ${config.kd}` : "",
    ]
      .filter(Boolean)
      .join(" / ") || `试验 #${trial.index}`
  );
}

export default function CalibrationChart({
  report,
  live,
}: {
  report?: CalibrationReport;
  live?: Live;
}) {
  const [axis, setAxis] = useState(
    live?.axis ??
      report?.settings.axis ??
      report?.trials.find((trial) => trial.axis != null)?.axis ??
      0,
  );
  const [metric, setMetric] = useState<Metric>("position");
  const hasCartesian = !!report?.trials.some(
    (trial) =>
      trial.space === "cartesian" || trial.phase.startsWith("cartesian_"),
  );
  const [scope, setScope] = useState<Space | "all">(
    hasCartesian ? "cartesian" : "all",
  );
  const [mode, setMode] = useState<"full" | "compare">(
    hasCartesian ? "compare" : "full",
  );
  const [comparison, setComparison] = useState<number | null>(null);
  const [hover, setHover] = useState<{ plot: string; index: number } | null>(
    null,
  );
  const trials = useMemo(
    () =>
      report
        ? calibrationTrials(
            report,
            axis,
            scope === "cartesian" ? "cartesian" : "joint",
          )
        : [],
    [report, axis, scope],
  );
  const baseline =
    trials.find(
      (trial) =>
        trial.phase ===
        (scope === "cartesian" ? "cartesian_before" : "baseline"),
    ) ?? trials[0];
  const alternatives = trials.filter((trial) => trial !== baseline);
  const choice = report?.choices?.find((item) => item.axis === axis);
  const selected =
    alternatives.find((trial) => trial.index === comparison) ??
    alternatives.find((trial) => trial.phase === "cartesian_after") ??
    alternatives.find((trial) => trial.phase === "after") ??
    alternatives.find((trial) => trial.index === choice?.candidate_index) ??
    alternatives.at(-1);
  const points = report?.curves ?? live?.points;
  const plots = useMemo<Plot[]>(() => {
    if (!report || mode === "full")
      return [
        {
          key: "full",
          label: live
            ? `实时全程 · 正在测试 ${live.space === "cartesian" ? `末端 ${"XYZ"[live.axis]}` : `J${live.axis + 1}`}`
            : scope === "cartesian"
              ? "末端动作全程"
              : scope === "joint"
                ? "关节试验全程"
                : "全程",
          points: (points ?? []).filter(
            (point) =>
              !report || scope === "all" || (point.space ?? "joint") === scope,
          ),
        },
      ];
    return [baseline, selected]
      .filter((trial): trial is CalibrationTrial => !!trial)
      .map((trial) => ({
        key: String(trial.index),
        label: calibrationTrialLabel(report, trial, axis),
        points: (points ?? []).filter((point) => point.trial === trial.index),
      }));
  }, [
    report,
    live?.axis,
    live?.space,
    mode,
    scope,
    points,
    baseline,
    selected,
    axis,
  ]);
  const value = (point: CalibrationPoint, key: Series): number | null => {
    if (metric === "velocity") {
      if (key === "reference" || (key === "command" && point.command === null))
        return null;
      return (
        (key === "command"
          ? point.velocity[axis]
          : point.feedback_velocity[axis]) * DEG
      );
    }
    const position =
      key === "command" ? point.command?.[axis] : point[key][axis];
    return position == null
      ? null
      : (position - (metric === "error" ? point.reference[axis] : 0)) * DEG;
  };
  let low = Infinity,
    high = -Infinity;
  plots.forEach((plot) =>
    plot.points.forEach((point) =>
      series.forEach(({ key }) => {
        const n = value(point, key);
        if (n !== null && Number.isFinite(n)) {
          low = Math.min(low, n);
          high = Math.max(high, n);
        }
      }),
    ),
  );
  if (!Number.isFinite(low)) {
    low = 0;
    high = 1;
  }
  const padding = Math.max(
    (high - low) * 0.12,
    metric === "velocity" ? 0.05 : 0.005,
  );
  low -= padding;
  high += padding;
  const width = 720,
    height = mode === "compare" && report ? 100 : 268,
    left = 57,
    right = 12,
    top = 12,
    bottom = 26;
  const y = (n: number) =>
    top + ((high - n) / (high - low)) * (height - top - bottom);
  const sharedSpan = Math.max(
    1,
    ...plots.map(
      (plot) => (plot.points.at(-1)?.t_ms ?? 0) - (plot.points[0]?.t_ms ?? 0),
    ),
  );
  const unit = metric === "velocity" ? "°/s" : "°";
  const hasCurves = (points?.length ?? 0) > 0;
  return (
    <div
      className="calibration-chart"
      data-mode={report ? mode : "full"}
      data-live-count={live?.points.length}
    >
      {hasCartesian && (
        <div className="calibration-space-tabs" aria-label="曲线动作类型">
          {(
            [
              ["cartesian", "末端动作"],
              ["joint", "关节试验"],
              ["all", "完整测试"],
            ] as const
          ).map(([key, label]) => (
            <button
              key={key}
              aria-pressed={scope === key}
              onClick={() => {
                setScope(key);
                setComparison(null);
                setHover(null);
                setMode(key === "all" ? "full" : "compare");
              }}
            >
              {label}
            </button>
          ))}
          <span>每个关节均记录实测响应</span>
        </div>
      )}
      <div className="calibration-chart-controls">
        <div className="calibration-axis-tabs" aria-label="曲线关节">
          {Array.from({ length: 7 }, (_, index) => (
            <button
              key={index}
              aria-pressed={axis === index}
              onClick={() => {
                setAxis(index);
                setComparison(null);
                setHover(null);
              }}
            >
              J{index + 1}
            </button>
          ))}
        </div>
        <div className="calibration-metric-tabs" aria-label="曲线指标">
          {metrics.map(([key, label]) => (
            <button
              key={key}
              aria-pressed={metric === key}
              onClick={() => {
                setMetric(key);
                setHover(null);
              }}
            >
              {label}
            </button>
          ))}
        </div>
        <select
          className="calibration-chart-mode"
          aria-label="曲线范围"
          value={report ? mode : "full"}
          onChange={(event) => {
            setMode(event.target.value as "full" | "compare");
            setHover(null);
          }}
        >
          <option value="full">全程</option>
          {report && (
            <option value="compare">
              {scope === "cartesian" ? "末端调整前 / 后" : "基线 / 候选对比"}
            </option>
          )}
        </select>
      </div>
      {hasCurves ? (
        <>
          <div className="calibration-chart-toolbar">
            <div className="calibration-chart-legend">
              {series.map(({ key, label }) => (
                <span
                  key={key}
                  data-series={key}
                  data-unavailable={
                    metric === "velocity" && key === "reference"
                  }
                >
                  <i />
                  {label}
                </span>
              ))}
            </div>
            {mode === "compare" && !!alternatives.length && (
              <label>
                对比{" "}
                <select
                  aria-label="对比试验"
                  value={selected?.index ?? ""}
                  onChange={(event) => {
                    setComparison(Number(event.target.value));
                    setHover(null);
                  }}
                >
                  {alternatives.map((trial) => (
                    <option key={trial.index} value={trial.index}>
                      {calibrationTrialLabel(report!, trial, axis)}
                    </option>
                  ))}
                </select>
              </label>
            )}
          </div>
          {plots.some((plot) => plot.points.length) ? (
            plots.map((plot) => {
              const first = plot.points[0]?.t_ms ?? 0,
                last = plot.points.at(-1)?.t_ms ?? first;
              const span =
                mode === "compare" && report
                  ? sharedSpan
                  : Math.max(1, last - first);
              const x = (time: number) =>
                left + ((time - first) / span) * (width - left - right);
              const focused =
                hover?.plot === plot.key ? plot.points[hover.index] : null;
              const path = (key: Series) => {
                let connected = false;
                return plot.points
                  .map((point) => {
                    const n = value(point, key);
                    if (n === null || !Number.isFinite(n)) {
                      connected = false;
                      return "";
                    }
                    const command = connected ? "L" : "M";
                    connected = true;
                    return `${command}${x(point.t_ms).toFixed(2)},${y(n).toFixed(2)}`;
                  })
                  .join(" ");
              };
              return (
                <figure
                  className="calibration-plot"
                  key={plot.key}
                  data-point-count={plot.points.length}
                  data-first-ms={first}
                  data-last-ms={last}
                >
                  <figcaption>
                    <strong>
                      J{axis + 1} · {plot.label}
                    </strong>
                    <small>
                      {plot.points.length} 点 · {unit}
                    </small>
                  </figcaption>
                  {plot.points.length ? (
                    <>
                      <svg
                        viewBox={`0 0 ${width} ${height}`}
                        preserveAspectRatio="none"
                        role="img"
                        tabIndex={0}
                        aria-label={`J${axis + 1} ${plot.label} ${metrics.find(([key]) => key === metric)?.[1]}曲线，时间为本次测试起点后的秒数`}
                        onPointerMove={(event) => {
                          const bounds =
                            event.currentTarget.getBoundingClientRect();
                          const time =
                            first +
                            ((((event.clientX - bounds.left) / bounds.width) *
                              width -
                              left) /
                              (width - left - right)) *
                              span;
                          let index = 0;
                          plot.points.forEach((point, i) => {
                            if (
                              Math.abs(point.t_ms - time) <
                              Math.abs(plot.points[index].t_ms - time)
                            )
                              index = i;
                          });
                          setHover({ plot: plot.key, index });
                        }}
                        onKeyDown={(event) => {
                          if (!["ArrowLeft", "ArrowRight"].includes(event.key))
                            return;
                          event.preventDefault();
                          const before =
                            hover?.plot === plot.key ? hover.index : 0;
                          setHover({
                            plot: plot.key,
                            index: Math.max(
                              0,
                              Math.min(
                                plot.points.length - 1,
                                before + (event.key === "ArrowRight" ? 1 : -1),
                              ),
                            ),
                          });
                        }}
                      >
                        {[0, 0.5, 1].map((fraction) => {
                          const n = low + fraction * (high - low);
                          return (
                            <g
                              key={fraction}
                              className="calibration-chart-grid"
                            >
                              <line
                                x1={left}
                                x2={width - right}
                                y1={y(n)}
                                y2={y(n)}
                              />
                              <text x={left - 7} y={y(n) + 3} textAnchor="end">
                                {n.toFixed(metric === "velocity" ? 2 : 3)}
                              </text>
                            </g>
                          );
                        })}
                        {[0, 0.5, 1].map((fraction) => (
                          <text
                            key={fraction}
                            className="calibration-chart-tick"
                            x={x(first + fraction * span)}
                            y={height - 6}
                            textAnchor={
                              fraction === 0
                                ? "start"
                                : fraction === 1
                                  ? "end"
                                  : "middle"
                            }
                          >
                            {((first + fraction * span) / 1000).toFixed(2)} s
                          </text>
                        ))}
                        {series.map(({ key }) => (
                          <path
                            key={key}
                            data-series={key}
                            d={path(key)}
                            fill="none"
                            vectorEffect="non-scaling-stroke"
                          />
                        ))}
                        {focused && (
                          <g className="calibration-chart-cursor">
                            <line
                              x1={x(focused.t_ms)}
                              x2={x(focused.t_ms)}
                              y1={top}
                              y2={height - bottom}
                            />
                            {series.map(({ key }) => {
                              const n = value(focused, key);
                              return n === null ? null : (
                                <circle
                                  key={key}
                                  data-series={key}
                                  cx={x(focused.t_ms)}
                                  cy={y(n)}
                                  r={3}
                                />
                              );
                            })}
                          </g>
                        )}
                      </svg>
                      <div
                        className="calibration-chart-sample"
                        role="region"
                        aria-label="采样详情"
                        tabIndex={0}
                        aria-live="polite"
                      >
                        {focused ? (
                          <>
                            <span>
                              t {focused.t_ms} ms · 测试{" "}
                              {focused.space === "cartesian"
                                ? `末端 ${"XYZ"[focused.axis]}`
                                : `J${focused.axis + 1}`}
                            </span>
                            {series.map(({ key }) => (
                              <span key={key} data-series={key}>
                                {key} {value(focused, key)?.toFixed(3) ?? "—"}
                                {unit}
                              </span>
                            ))}
                            <span>
                              发送 {focused.command_at_ms ?? "—"} ms · 反馈{" "}
                              {focused.feedback_age_ms[axis]} ms
                            </span>
                          </>
                        ) : (
                          <span>指向曲线查看原始采样值</span>
                        )}
                      </div>
                    </>
                  ) : (
                    <p className="calibration-chart-missing">
                      此轮没有曲线采样
                    </p>
                  )}
                </figure>
              );
            })
          ) : (
            <p className="calibration-empty">
              J{axis + 1} 没有该轮曲线，可切回全程查看被动响应。
            </p>
          )}
          <p className="calibration-note">
            {mode === "compare" && report
              ? "两轮按首个采样对齐，时间刻度保留原始值，纵轴相同。"
              : scope === "cartesian"
                ? "末端动作包含七轴的耦合响应；时间为测试原始时间。"
                : "全程包含其他关节测试时的本轴响应；时间为测试原始时间。"}
            {metric === "velocity"
              ? "未单独记录参考速度。"
              : metric === "error"
                ? "误差为位置减去同一时刻的控制参考。"
                : "按原始点连线。"}{" "}
            {live
              ? `最近 ${live.points.length} / 5000 点`
              : `${(report?.curves.length ?? 0).toLocaleString()} / ${(report?.sample_count ?? report?.curves.length ?? 0).toLocaleString()} 点`}
          </p>
        </>
      ) : (
        <p className="calibration-empty">
          {live
            ? "等待实时采样…"
            : "这份报告只记录了各轮测量，可在结果页查看位置、速度与峰值对比。"}
        </p>
      )}
    </div>
  );
}
