import assert from "node:assert/strict";
import { after, before, test } from "node:test";
import { createElement } from "react";
import { renderToStaticMarkup } from "react-dom/server";
import { createServer } from "vite";

let server, Chart, diagnosticEvents, parseReport, baseline;
before(async () => {
  server = await createServer({
    server: { middlewareMode: true, hmr: false, watch: null },
  });
  ({ default: Chart } = await server.ssrLoadModule(
    "/src/CalibrationChart.tsx",
  ));
  ({ diagnosticEvents } = await server.ssrLoadModule("/src/FaultTimeline.tsx"));
  const api = await server.ssrLoadModule("/src/api.ts");
  parseReport = api.parseCalibrationReport;
  baseline = api.createServoTuning();
});
after(async () => server?.close());

const axes = (n) => Array(7).fill(n);
const fault = {
  axis: 5,
  t_ms: 7300,
  code: 512,
  message: "电机绕组过热",
  position: Math.PI / 2,
  command: Math.PI,
  effort: 2,
  driver_temperature: 43,
  winding_temperature: 76,
  voltage: 24,
  feedforward_effort: 0.73,
};
function point(t_ms, command_at_ms, age = 17) {
  return {
    t_ms,
    command_at_ms,
    command: command_at_ms == null ? null : axes(Math.PI / 4),
    axis: 0,
    trial: 0,
    reference: axes(0),
    velocity: axes(0),
    feedback: axes(Math.PI / 6),
    feedback_velocity: axes(0),
    feedback_age_ms: [1, 2, 3, 4, 5, age, 7],
    effort: axes(0),
    feedforward_effort: [1, 2, 3, 4, 5, 0.37, 0.27],
    driver_temperature: axes(30),
    winding_temperature: axes(40),
  };
}
function report(overrides) {
  return parseReport({
    id: "offline",
    settings: { axis: 0 },
    trials: [],
    outcome: "failed",
    baseline,
    center_reference: axes(0),
    center_measured: axes(0),
    ...overrides,
  });
}
const render = (report) =>
  renderToStaticMarkup(createElement(Chart, { report }));

test("send and feedback clocks are independent and repeated events share one timestamp", () => {
  const before = point(7200, 7170);
  const after = point(7210, 7170, 27);
  const events = diagnosticEvents([before, after], 5, fault);
  assert.deepEqual(
    events.map((row) => row.t_ms),
    [7170, 7183, 7300],
  );
  assert.deepEqual(events[0], {
    t_ms: 7170,
    command: 45,
    feedforward_effort: 0.37,
  });
  assert.equal(events[1].effort, 0);
  assert.equal(events[1].winding_temperature, 40);
  assert.equal(events[2].feedback, 90);
  assert.equal(events[2].effort, 2);
  assert.ok(
    events.every((row) => row.t_ms === 7170 || row.command === undefined),
  );

  // J7 received the first frame 7 ms before sampling. The J6 fault is only a marker.
  const j7 = diagnosticEvents([before], 6, fault);
  assert.equal(j7[1].t_ms, 7193);
  assert.equal(j7[0].feedforward_effort, 0.27);
  assert.deepEqual(j7.at(-1), { t_ms: 7300 });
});

test("a genuine send coinciding with a fault keeps its own recorded value", () => {
  const events = diagnosticEvents([point(7310, fault.t_ms, 10)], 5, fault);
  assert.deepEqual(events, [
    {
      t_ms: fault.t_ms,
      command: 45,
      feedforward_effort: 0.37,
      feedback: 90,
      effort: 2,
      driver_temperature: 43,
      winding_temperature: 76,
    },
  ]);
});

test("failed report opens J6 on a common fault cursor and clips all three lanes to five seconds", () => {
  const early = point(100, 90);
  const missing = point(7150, null);
  delete missing.driver_temperature;
  delete missing.winding_temperature;
  const html = render(
    report({
      fault,
      curves: [early, point(7100, 7080), missing, point(7200, 7180)],
    }),
  );
  assert.match(html, /aria-label="J6 故障分析"/);
  assert.match(html, /最近指令 180°/);
  assert.match(html, /最近指令前馈 0.73 Nm/);
  assert.match(html, /游标 7300 ms · 故障时刻/);
  assert.match(html, /2\.300 s/);
  assert.match(html, /7\.300 s/);
  assert.equal(
    (html.match(/class="calibration-chart-cursor"/g) ?? []).length,
    3,
  );
  for (const key of [
    "command",
    "feedforward_effort",
    "driver_temperature",
    "winding_temperature",
  ]) {
    const d = html.match(
      new RegExp('<path data-series="' + key + '" d="([^"]*)"'),
    )[1];
    assert.equal(
      d.split("M").length - 1,
      2,
      key + " has two segments around the missing observation",
    );
    assert.doesNotMatch(d, /NaN|Infinity/);
  }
  assert.doesNotMatch(
    html,
    /<circle data-series="(command|feedforward_effort)"/,
  );
});

test("fault-only reports and legacy summary-only reports render from imported JSON", () => {
  const only = render(
    report({
      fault: { ...fault, command: null, feedforward_effort: undefined },
    }),
  );
  assert.match(only, /最近指令 —/);
  assert.match(only, /<path data-series="command" d=""/);
  assert.match(only, /最近指令前馈 —/);
  assert.match(only, /<path data-series="feedforward_effort" d=""/);
  assert.match(only, /绕组 76/);
  const old = render(report({}));
  assert.match(old, /这份报告只记录了各轮测量/);
  const legacyPoint = point(10, null);
  delete legacyPoint.effort;
  delete legacyPoint.driver_temperature;
  delete legacyPoint.winding_temperature;
  const events = diagnosticEvents(report({ curves: [legacyPoint] }).curves, 5);
  assert.deepEqual(events[0], {
    t_ms: -7,
    feedback: (Math.PI / 6) * (180 / Math.PI),
    effort: null,
    driver_temperature: null,
    winding_temperature: null,
  });
  assert.deepEqual(events[1], {
    t_ms: 10,
    command: null,
    feedforward_effort: null,
  });
});

test("report import validates seven-axis temperatures and single-axis fault data", () => {
  assert.throws(
    () => report({ curves: [{ ...point(10, 9), driver_temperature: [30] }] }),
    /曲线/,
  );
  assert.throws(() => report({ fault: { ...fault, axis: 7 } }), /故障记录/);
  assert.throws(
    () => report({ fault: { ...fault, command: "0" } }),
    /故障记录/,
  );
});

test("missing feedforward creates a gap while recorded zero remains a send", () => {
  const known = { ...point(10, 8), feedforward_effort: axes(0) };
  const missing = point(20, 18);
  delete missing.feedforward_effort;
  const events = diagnosticEvents([known, missing], 5, fault);
  assert.equal(events.find((row) => row.t_ms === 8).feedforward_effort, 0);
  assert.equal(events.find((row) => row.t_ms === 18).feedforward_effort, null);
  assert.equal(events.at(-1).feedforward_effort, undefined);
  assert.throws(
    () => report({ curves: [{ ...known, feedforward_effort: [1] }] }),
    /曲线/,
  );
  assert.throws(
    () => report({ fault: { ...fault, feedforward_effort: "0" } }),
    /故障记录/,
  );
});

test("all reports with a fault, including cancelled runs, open the fault view", () => {
  for (const outcome of [
    "cancelled",
    "failed",
    "insufficient",
    "unchanged",
    "improved",
  ]) {
    assert.match(
      render(report({ outcome, fault })),
      /aria-label="J6 故障分析"/,
    );
  }
});

test("legacy JSON formats canonical motor fault codes like SQLite and preserves other messages", () => {
  for (const [code, expected] of [
    [512, "电机绕组过热"],
    [520, "供电电压短时过低、电机绕组过热（0x00000208）"],
    [128, "未知故障位 0x00000080"],
    [0x80000200, "电机绕组过热、未知故障位 0x80000000（0x80000200）"],
  ]) {
    const translated = `J6 ${expected}（故障码 ${code}）`;
    assert.equal(report({ message: `J6 故障码 ${code}` }).message, translated);
    assert.equal(report({ message: translated }).message, translated);
  }
  for (const message of [
    "J6 故障码 512\n",
    "J6 故障码 512 附注",
    "J06 故障码 512",
    "J8 故障码 512",
    "J6 故障码 0512",
    "J6 故障码 0",
    "J6 故障码 4294967296",
  ]) {
    assert.equal(report({ message }).message, message);
  }
});
