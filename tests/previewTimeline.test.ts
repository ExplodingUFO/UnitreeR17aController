import assert from "node:assert/strict";
import test from "node:test";
import {
  jointTravel,
  sampleStage,
  stagePoints,
  stageTimes,
  type ScenePoint,
} from "../src/previewTimeline.ts";
import type { TeachingPreview } from "../src/api.ts";

function point(
  axis: number,
  segment: number,
  t_s: number,
  q: number,
): ScenePoint {
  return {
    axis,
    segment,
    t_s,
    space: "cartesian",
    joints: [q, 0, 0, 0, 0, 0, 0],
    tcp: [q, 0, 0],
  };
}

const x = [
  point(0, 0, 0, 0),
  point(0, 0, 1, 2),
  point(0, 0, 6, 4),
  point(0, 1, 0, 4),
  point(0, 1, 2, 0),
];
const y = [
  point(1, 2, 0, 0),
  point(1, 2, 1, 8),
  point(1, 3, 0, 8),
  point(1, 3, 3, 0),
];
const joint = x.map((p) => ({ ...p, space: "joint" }));
const preview = { points: [...x, ...y, ...joint] };

test("XYZ and individual axes concatenate only their selected Cartesian legs", () => {
  const all = stagePoints(preview, "xyz");
  const times = stageTimes(all);
  assert.deepEqual(times, [0, 1, 6, 6, 8, 8, 9, 9, 12]);
  assert.deepEqual(
    stageTimes(stagePoints(preview, "cartesian:0")),
    [0, 1, 6, 6, 8],
  );
  assert.deepEqual(
    stageTimes(stagePoints(preview, "cartesian:1")),
    [0, 1, 1, 4],
  );
  assert.deepEqual(sampleStage(all, times, 0.25), {
    from: x[1],
    to: x[2],
    blend: 0.4,
  });
  assert.equal(sampleStage(all, times, 0.5)?.from, x[3]);
  assert.equal(sampleStage(all, times, 8 / 12)?.from, y[0]);
  assert.deepEqual(sampleStage(all, times, 1), {
    from: y[3],
    to: y[3],
    blend: 0,
  });
});

test("scrubbing preserves every display knot and interpolates within the return leg", () => {
  const points = stagePoints(preview, "cartesian:1");
  const times = stageTimes(points)!;
  const turn = sampleStage(points, times, times[1] / times.at(-1)!)!;
  assert.equal(turn.from.joints[0], 8);
  assert.equal(turn.blend, 0);
  const midway = sampleStage(points, times, 2.5 / 4)!;
  assert.deepEqual(midway, { from: y[2], to: y[3], blend: 0.5 });
  assert.equal(sampleStage(points, times, -1)?.from, y[0]);
  assert.equal(sampleStage(points, times, 2)?.from, y[3]);
});

test("incomplete or inconsistent times and joint trials keep the point-index path", () => {
  for (const t_s of [undefined, NaN, -1, 0.5]) {
    const points = x.map((p, i) => (i === 2 ? { ...p, t_s } : p));
    assert.equal(stageTimes(points), null);
    assert.deepEqual(sampleStage(points, null, 0.375), {
      from: points[1],
      to: points[2],
      blend: 0.5,
    });
  }
  assert.equal(stageTimes(stagePoints(preview, "joint:0")), null);
  assert.deepEqual(sampleStage(joint, null, 0.625), {
    from: joint[2],
    to: joint[3],
    blend: 0,
  });
  assert.equal(sampleStage([], stageTimes([]), 0), null);
});

test("teaching phases retain their continuous millisecond clock and transition", () => {
  const preview: TeachingPreview = {
    trajectory_id: "demo",
    side: "right",
    origin: x[0].joints,
    tcp: { translation: [0, 0, 0], rotation: [0, 0, 0, 1] },
    duration_ms: 4000,
    approach_duration_ms: 1000,
    requested_speed_scale: 1,
    effective_speed_scale: 1,
    points: x.slice(0, 3).map((p, i) => ({
      t_ms: [0, 1000, 4000][i],
      joints: p.joints,
      tcp: p.tcp,
      phase: i === 0 ? "approach" : "path",
    })),
  };
  const all = stagePoints(preview, "teaching:all");
  assert.deepEqual(stageTimes(all), [0, 1, 4]);
  assert.equal(sampleStage(all, stageTimes(all), 0.125)?.blend, 0.5);
  assert.deepEqual(stageTimes(stagePoints(preview, "teaching:path")), [0, 3]);
});

test("joint limits and legacy display points yield signed travel relative to origin", () => {
  const origin: ScenePoint["joints"] = [0, 1.5, 0, 0, 0, 0, 0];
  const points = [1.5, 1.75, 1.0, 1.5].map((q) => {
    const p = point(1, 0, 0, 0);
    p.joints[1] = q;
    return { ...p, space: "joint" };
  });
  assert.deepEqual(
    jointTravel(
      {
        origin,
        joint_targets: [
          [0, 0],
          [1, 1.75],
        ],
      },
      points,
    ),
    [-0.5, 0.25],
  );
  assert.deepEqual(jointTravel({ origin }, points), [-0.5, 0.25]);
  assert.deepEqual(
    jointTravel(
      {
        origin,
        joint_targets: [
          [0, 0],
          [1.5, 1.75],
        ],
      },
      points,
    ),
    [0, 0.25],
  );
  assert.equal(jointTravel({ origin }, x), null);
});
