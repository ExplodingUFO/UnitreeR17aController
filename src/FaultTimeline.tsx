import { useMemo, useState } from "react";
import type { CalibrationFault, CalibrationPoint } from "./api";

const DEG = 180 / Math.PI;
const fitNumber = (value: number) => Number(value.toPrecision(4)).toString();

type DiagnosticFrame = {
  t_ms: number;
  command?: number | null;
  feedback?: number | null;
  effort?: number | null;
  feedforward_effort?: number | null;
  driver_temperature?: number | null;
  winding_temperature?: number | null;
};
type DiagnosticSeries = Exclude<keyof DiagnosticFrame, "t_ms">;
const diagnosticLanes: {
  label: string;
  unit: string;
  series: [DiagnosticSeries, string][];
}[] = [
  {
    label: "位置",
    unit: "°",
    series: [
      ["command", "成功发送"],
      ["feedback", "实际反馈"],
    ],
  },
  {
    label: "扭矩",
    unit: "Nm",
    series: [
      ["feedforward_effort", "发送前馈"],
      ["effort", "实测扭矩"],
    ],
  },
  {
    label: "温度",
    unit: "℃",
    series: [
      ["driver_temperature", "驱动"],
      ["winding_temperature", "绕组"],
    ],
  },
];

const degreesOrNull = (radians: number | null | undefined) =>
  radians == null ? null : radians * DEG;
const faultWindowMs = 5000;
const diagnosticWidth = 720,
  plotLeft = 50,
  plotRight = 704,
  plotTop = 7,
  plotBottom = 44;

// One row per event time. An absent key belongs to another clock; a null
// measurement breaks its own curve. Repeated snapshots of one send share a row.
export function diagnosticEvents(
  points: CalibrationPoint[] | undefined,
  axis: number,
  fault?: CalibrationFault | null,
) {
  const events = new Map<number, DiagnosticFrame>();
  function at(ms: number) {
    let row = events.get(ms);
    if (!row) {
      row = { t_ms: ms };
      events.set(ms, row);
    }
    return row;
  }
  for (const point of points ?? []) {
    if (point.command_at_ms != null && point.command != null) {
      const sent = at(point.command_at_ms);
      sent.command = degreesOrNull(point.command[axis]);
      sent.feedforward_effort =
        point.feedforward_effort?.[axis] ?? sent.feedforward_effort ?? null;
    } else {
      // A gap at sampling time is not a send. Preserve any known send there.
      const gap = at(point.t_ms);
      gap.command ??= null;
      gap.feedforward_effort ??= null;
    }
    const received = at(point.t_ms - point.feedback_age_ms[axis]);
    received.feedback = degreesOrNull(point.feedback[axis]);
    received.effort = point.effort?.[axis] ?? null;
    received.driver_temperature = point.driver_temperature?.[axis] ?? null;
    received.winding_temperature = point.winding_temperature?.[axis] ?? null;
  }
  if (fault) {
    const received = at(fault.t_ms);
    if (fault.axis === axis) {
      received.feedback = degreesOrNull(fault.position);
      received.effort = fault.effort;
      received.driver_temperature = fault.driver_temperature;
      received.winding_temperature = fault.winding_temperature;
    }
  }
  return Array.from(events.values()).sort((a, b) => a.t_ms - b.t_ms);
}

export default function FaultTimeline({
  points,
  axis,
  fault,
}: {
  points?: CalibrationPoint[];
  axis: number;
  fault?: CalibrationFault | null;
}) {
  const [tail, setTail] = useState(!!fault);
  const [cursorMs, setCursorMs] = useState<number | null>(null);
  const events = useMemo(
    () => diagnosticEvents(points, axis, fault),
    [points, axis, fault],
  );
  const timeline = useMemo(() => {
    const first = Math.min(0, events[0]?.t_ms ?? 0);
    const last = (tail ? fault?.t_ms : undefined) ?? events.at(-1)?.t_ms ?? 0;
    const start = tail ? Math.max(first, last - faultWindowMs) : first;
    const frames = tail
      ? events.filter((frame) => frame.t_ms >= start && frame.t_ms <= last)
      : events;
    const span = Math.max(1, last - start);
    const x = (ms: number) =>
      plotLeft + ((ms - start) / span) * (plotRight - plotLeft);
    const lanes = diagnosticLanes.map((lane) => {
      let min = Infinity,
        max = -Infinity;
      for (const frame of frames) {
        for (const [key] of lane.series) {
          const value = frame[key];
          if (value != null) {
            min = Math.min(min, value);
            max = Math.max(max, value);
          }
        }
      }
      const y = (n: number) =>
        max === min
          ? (plotTop + plotBottom) / 2
          : plotBottom - ((n - min) / (max - min)) * (plotBottom - plotTop);
      const paths = lane.series.map(([key, label]) => {
        let connected = false;
        const d = frames
          .map((frame) => {
            const n = frame[key];
            if (n === undefined) return "";
            if (n === null) {
              connected = false;
              return "";
            }
            const op = connected ? "L" : "M";
            connected = true;
            // A zero-length stroke with a round cap also displays isolated observations.
            return `${op}${x(frame.t_ms).toFixed(2)},${y(n).toFixed(2)}${op === "M" ? "l0,0" : ""}`;
          })
          .join("");
        return { key, label, d };
      });
      const ticks = min === Infinity ? [] : max === min ? [min] : [min, max];
      return { ...lane, paths, ticks, y };
    });
    return { frames, lanes, start, span, x };
  }, [events, fault, tail]);
  const { frames, lanes, start, span, x } = timeline;
  function nearest(ms: number) {
    let lo = 0,
      hi = frames.length - 1;
    while (lo < hi) {
      const mid = Math.floor((lo + hi) / 2);
      if (frames[mid].t_ms < ms) lo = mid + 1;
      else hi = mid;
    }
    return lo && ms - frames[lo - 1].t_ms < frames[lo].t_ms - ms ? lo - 1 : lo;
  }
  if (!frames.length)
    return <p className="calibration-empty">暂无故障分析采样</p>;
  const index = nearest(
    cursorMs ?? fault?.t_ms ?? frames[frames.length - 1].t_ms,
  );
  const focused = frames[index];
  return (
    <section
      className="calibration-diagnostic"
      aria-label={`J${axis + 1} 故障分析`}
    >
      <header>
        {fault ? (
          <strong>
            {fault.message} · {(fault.t_ms / 1000).toFixed(3)} s ·{" "}
            {fault.voltage} V · 最近指令{" "}
            {fault.command == null ? "—" : `${fitNumber(fault.command * DEG)}°`}
            {" · 最近指令前馈 "}
            {fault.feedforward_effort == null
              ? "—"
              : `${fitNumber(fault.feedforward_effort)} Nm`}
          </strong>
        ) : (
          <strong>J{axis + 1} 同步采样</strong>
        )}
        <select
          aria-label="故障分析范围"
          value={tail ? "tail" : "full"}
          onChange={(event) => {
            setTail(event.target.value === "tail");
            setCursorMs(null);
          }}
        >
          <option value="full">测试全程</option>
          <option value="tail">{fault ? "故障前 5 秒" : "末段 5 秒"}</option>
        </select>
      </header>
      <figure
        className="calibration-plot calibration-diagnostic-plot"
        tabIndex={0}
        aria-label="同步游标，使用左右方向键逐点查看"
        onKeyDown={(event) => {
          if (event.key !== "ArrowLeft" && event.key !== "ArrowRight") return;
          event.preventDefault();
          setCursorMs(
            frames[
              Math.max(
                0,
                Math.min(
                  frames.length - 1,
                  index + (event.key === "ArrowRight" ? 1 : -1),
                ),
              )
            ].t_ms,
          );
        }}
      >
        <figcaption>
          <strong>J{axis + 1} · 指令按发送时刻，实测按接收时刻</strong>
          <span>
            游标 {focused.t_ms} ms
            {focused.t_ms === fault?.t_ms ? " · 故障时刻" : ""}
          </span>
        </figcaption>
        {lanes.map(({ label, unit, paths, ticks, y }, laneIndex) => {
          const lastLane = laneIndex === lanes.length - 1;
          return (
            <div className="calibration-diagnostic-lane" key={label}>
              <div className="calibration-chart-legend">
                <b>
                  {label}（{unit}）
                </b>
                {paths.map(({ key, label }) => (
                  <span key={key} data-series={key}>
                    <i />
                    {label}{" "}
                    {focused[key] == null ? "—" : fitNumber(focused[key])}
                  </span>
                ))}
              </div>
              <svg
                viewBox={`0 0 ${diagnosticWidth} ${lastLane ? 70 : 51}`}
                preserveAspectRatio="none"
                role="img"
                aria-label={`J${axis + 1} ${label}（${unit}）`}
                onPointerMove={(event) => {
                  const bounds = event.currentTarget.getBoundingClientRect();
                  const svgX =
                    ((event.clientX - bounds.left) / bounds.width) *
                    diagnosticWidth;
                  setCursorMs(
                    frames[
                      nearest(
                        start +
                          ((svgX - plotLeft) / (plotRight - plotLeft)) * span,
                      )
                    ].t_ms,
                  );
                }}
              >
                {ticks.map((n) => (
                  <g key={n} className="calibration-chart-grid">
                    <line x1={plotLeft} x2={plotRight} y1={y(n)} y2={y(n)} />
                    <text x={plotLeft - 6} y={y(n) + 3} textAnchor="end">
                      {fitNumber(n)}
                    </text>
                  </g>
                ))}
                {!ticks.length && (
                  <text className="calibration-chart-tick" x={plotLeft} y={29}>
                    无采样
                  </text>
                )}
                {paths.map(({ key, d }) => (
                  <path
                    key={key}
                    data-series={key}
                    d={d}
                    fill="none"
                    strokeLinecap="round"
                    vectorEffect="non-scaling-stroke"
                  />
                ))}
                {fault && (
                  <line
                    className="calibration-fault-marker"
                    x1={x(fault.t_ms)}
                    x2={x(fault.t_ms)}
                    y1={plotTop}
                    y2={plotBottom}
                  />
                )}
                <g className="calibration-chart-cursor">
                  <line
                    x1={x(focused.t_ms)}
                    x2={x(focused.t_ms)}
                    y1={plotTop}
                    y2={plotBottom}
                  />
                  {paths.map(
                    ({ key }) =>
                      focused[key] != null && (
                        <circle
                          key={key}
                          data-series={key}
                          cx={x(focused.t_ms)}
                          cy={y(focused[key])}
                          r={2.8}
                        />
                      ),
                  )}
                </g>
                {lastLane &&
                  [0, 0.5, 1].map((fraction) => (
                    <text
                      key={fraction}
                      className="calibration-chart-tick"
                      x={x(start + fraction * span)}
                      y={65}
                      textAnchor={
                        fraction === 0
                          ? "start"
                          : fraction === 1
                            ? "end"
                            : "middle"
                      }
                    >
                      {((start + fraction * span) / 1000).toFixed(3)} s
                    </text>
                  ))}
              </svg>
            </div>
          );
        })}
      </figure>
    </section>
  );
}
