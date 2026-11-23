import type { CartesianProgram, PreviewPoint, TeachingPreview } from "./api";

export type PreviewStage =
  | "xyz"
  | "return"
  | `teaching:${"approach" | "path" | "all"}`
  | `cartesian:${number}`
  | `joint:${number}`;
export type ScenePoint = PreviewPoint & { t_ms?: number };
export type ScenePreview = { points: ScenePoint[] };
export function scenePreview(
  preview: ScenePreview | TeachingPreview,
): ScenePreview {
  return "trajectory_id" in preview
    ? {
        points: preview.points.map((point) => ({
          ...point,
          space: `teaching_${point.phase}`,
          axis: 0,
          segment: point.phase === "approach" ? 0 : 1,
        })),
      }
    : preview;
}
export function stagePoints(
  preview: ScenePreview | TeachingPreview,
  stage: PreviewStage,
) {
  return scenePreview(preview).points.filter((point) =>
    stage.startsWith("teaching:")
      ? stage === "teaching:all"
        ? point.space.startsWith("teaching_")
        : point.space === `teaching_${stage.split(":")[1]}`
      : stage === "xyz"
        ? point.space === "cartesian"
        : stage === "return"
          ? point.space === "joint_return"
          : stage.startsWith("cartesian:")
            ? point.space === "cartesian" &&
              point.axis === Number(stage.split(":")[1])
            : point.space === "joint" &&
              point.axis === Number(stage.split(":")[1]),
  );
}

// Seconds from the start of the selected stage. Cartesian clocks restart on
// each leg, while teaching uses one continuous clock even across phases.
export function stageTimes(points: ScenePoint[]): number[] | null {
  const first = points[0];
  if (!first) return null;
  const cartesian = first.space === "cartesian";
  if (!cartesian && !first.space.startsWith("teaching_")) return null;

  const times: number[] = [];
  let offset = 0;
  for (const [index, point] of points.entries()) {
    const previous = points[index - 1];
    if (
      cartesian &&
      previous &&
      (previous.axis !== point.axis || previous.segment !== point.segment)
    ) {
      offset = times[index - 1];
    }
    const seconds = cartesian
      ? point.t_s
      : point.t_ms !== undefined && first.t_ms !== undefined
        ? (point.t_ms - first.t_ms) / 1000
        : undefined;
    if (seconds === undefined || !Number.isFinite(seconds) || seconds < 0)
      return null;
    const time = offset + seconds;
    if (index > 0 && time < times[index - 1]) return null;
    times.push(time);
  }
  return cartesian && times.at(-1) === 0 ? null : times;
}

/** Signed joint trial endpoints in radians, relative to the preview origin. */
export function jointTravel(
  program: Pick<CartesianProgram, "origin" | "joint_targets">,
  points: ScenePoint[],
): [number, number] | null {
  const first = points[0];
  if (first?.space !== "joint") return null;
  const axis = first.axis;
  const origin = program.origin[axis];
  const targets = program.joint_targets?.[axis];
  if (targets?.length === 2 && targets.every(Number.isFinite))
    return [targets[0] - origin, targets[1] - origin];

  let low = first.joints[axis],
    high = low;
  for (const point of points) {
    low = Math.min(low, point.joints[axis]);
    high = Math.max(high, point.joints[axis]);
  }
  return [low - origin, high - origin];
}

export function sampleStage(
  points: ScenePoint[],
  times: number[] | null,
  progress: number,
) {
  const last = points.length - 1;
  if (last < 0) return null;
  const fraction = Math.max(0, Math.min(1, progress));
  let index = Math.floor(fraction * last);
  let blend = fraction * last - index;
  if (times) {
    const time = fraction * times[last];
    let low = 0,
      high = last;
    while (low < high) {
      const mid = Math.ceil((low + high) / 2);
      if (times[mid] <= time) low = mid;
      else high = mid - 1;
    }
    index = low;
    blend =
      index < last
        ? Math.max(0, (time - times[index]) / (times[index + 1] - times[index]))
        : 0;
  }
  const from = points[index];
  const to = points[Math.min(index + 1, last)];
  if (
    !from.space.startsWith("teaching_") &&
    (from.space !== to.space ||
      from.axis !== to.axis ||
      from.segment !== to.segment)
  )
    blend = 0;
  return { from, to, blend };
}
