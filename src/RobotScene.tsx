import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";
import URDFLoader, { type URDFRobot } from "urdf-loader";
import type { CalibrationPreview, Joint, PreviewPoint, Side } from "./api";
export interface PreviewEnvelope {
  min: number[];
  max: number[];
  size: number[];
}
export type PreviewStage =
  | "xyz"
  | "return"
  | `cartesian:${number}`
  | `joint:${number}`;
export function stagePoints(preview: CalibrationPreview, stage: PreviewStage) {
  return preview.points.filter((point) =>
    stage === "xyz"
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
interface Props {
  side: Side;
  joints: Joint[];
  valid: boolean;
  source: string;
  preview?: CalibrationPreview | null;
  previewProgress?: number;
  previewStage?: PreviewStage;
  onPreviewEnvelope?: (envelope: PreviewEnvelope | null) => void;
}
interface ModelSpec {
  joint_names: string[];
  urdf: string;
}
type CameraView = "orbit" | "top" | "front" | "side";
const AXIS_COLORS = [0xff7478, 0x52e6a4, 0x65baff];
const JOINT_COLORS = [
  0xffc567, 0xc7a0ff, 0xff8cce, 0xffd168, 0xe5df75, 0x9eb1ff, 0xffa179,
];
// URDFRobot.clone shares immutable geometry. A removed preview owns its materials,
// paths and sweep surfaces, but must never dispose the loaded robot's geometry.
function disposeObject(
  object: THREE.Object3D,
  shared = new Set<THREE.BufferGeometry>(),
) {
  const geometries = new Set<THREE.BufferGeometry>();
  const materials = new Set<THREE.Material>();
  object.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if (mesh.geometry && !shared.has(mesh.geometry))
      geometries.add(mesh.geometry);
    if (mesh.material)
      (Array.isArray(mesh.material) ? mesh.material : [mesh.material]).forEach(
        (m) => materials.add(m),
      );
  });
  geometries.forEach((geometry) => geometry.dispose());
  materials.forEach((material) => material.dispose());
}
function poseDistance(a: PreviewPoint, b: PreviewPoint) {
  return a.joints.reduce((sum, q, i) => sum + (q - b.joints[i]) ** 2, 0);
}
// Farthest-point sampling retains different configurations even on a round trip.
function keyPoses(points: PreviewPoint[]) {
  if (!points.length) return [];
  const chosen = [points[0]];
  while (chosen.length < 5) {
    let best: PreviewPoint | undefined;
    let separation = 1e-9;
    for (const point of points) {
      const distance = Math.min(
        ...chosen.map((pose) => poseDistance(point, pose)),
      );
      if (distance > separation) {
        separation = distance;
        best = point;
      }
    }
    if (!best) break;
    chosen.push(best);
  }
  return chosen;
}
export default function RobotScene({
  side,
  joints,
  valid,
  source,
  preview,
  previewProgress = 0,
  previewStage = "cartesian:0",
  onPreviewEnvelope,
}: Props) {
  const container = useRef<HTMLDivElement>(null);
  const labels = useRef<HTMLDivElement>(null);
  const ruler = useRef<HTMLDivElement>(null);
  const [showBox, setShowBox] = useState(false);
  const [cameraView, setCameraView] = useState<CameraView>("orbit");
  const current = useRef({
    joints,
    valid,
    preview,
    previewProgress,
    previewStage,
    onPreviewEnvelope,
    showBox,
  });
  current.current = {
    joints,
    valid,
    preview,
    previewProgress,
    previewStage,
    onPreviewEnvelope,
    showBox,
  };
  const changeView = useRef<(view: CameraView) => void>(() => {});
  const [error, setError] = useState("");
  useEffect(() => {
    const element = container.current;
    if (!element) return;
    let renderer: THREE.WebGLRenderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    } catch {
      setError("无法显示模型");
      return;
    }
    setError("");
    let disposed = false;
    let robot: URDFRobot | undefined;
    let planned: URDFRobot | undefined;
    let spec: ModelSpec | undefined;
    let modelReady = false;
    let builtPreview: CalibrationPreview | null | undefined;
    let builtStage: PreviewStage | undefined;
    let cachedEnvelope: THREE.Box3 | undefined;
    let selected: PreviewPoint[] = [];
    const sharedGeometry = new Set<THREE.BufferGeometry>();
    const lineMaterials: LineMaterial[] = [];
    const endpointLabels: {
      position: THREE.Vector3;
      element: HTMLSpanElement;
    }[] = [];
    const theme = getComputedStyle(document.documentElement);
    const themeColor = (token: string) =>
      new THREE.Color(theme.getPropertyValue(token).trim());
    const referenceColor = themeColor("--scene-model-reference");
    const plannedColor = new THREE.Color(0x7ceaff);
    const scene = new THREE.Scene();
    const overlay = new THREE.Group();
    scene.add(overlay);
    scene.background = themeColor("--scene-background");
    const camera = new THREE.PerspectiveCamera(38, 1, 0.005, 100);
    camera.up.set(0, 0, 1);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = !window.matchMedia(
      "(prefers-reduced-motion: reduce)",
    ).matches;
    controls.minDistance = 0.08;
    controls.maxDistance = 10;
    let center = new THREE.Vector3(0, 0, 0.25);
    let distance = 1.3;
    let activeView: CameraView = "orbit";
    const resetCamera = (view = activeView) => {
      activeView = view;
      camera.up.set(0, view === "top" ? 1 : 0, view === "top" ? 0 : 1);
      const offset =
        view === "top"
          ? new THREE.Vector3(0, 0, 1)
          : view === "front"
            ? new THREE.Vector3(0, -1, 0)
            : view === "side"
              ? new THREE.Vector3(1, 0, 0)
              : new THREE.Vector3(1, -0.9, 0.65).normalize();
      camera.position.copy(center).addScaledVector(offset, distance);
      controls.target.copy(center);
      controls.update();
    };
    changeView.current = resetCamera;
    resetCamera();
    scene.add(
      new THREE.HemisphereLight(
        themeColor("--scene-light-sky"),
        themeColor("--scene-light-ground"),
        2.6,
      ),
    );
    const light = new THREE.DirectionalLight(themeColor("--scene-light"), 3);
    light.position.set(2, -3, 4);
    scene.add(light);
    const grid = new THREE.GridHelper(
      2,
      20,
      themeColor("--scene-grid-main"),
      themeColor("--scene-grid-minor"),
    );
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -0.005;
    scene.add(grid);
    const axes = new THREE.AxesHelper(0.1);
    axes.setColors(
      new THREE.Color(AXIS_COLORS[0]),
      new THREE.Color(AXIS_COLORS[1]),
      new THREE.Color(AXIS_COLORS[2]),
    );
    scene.add(axes);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    element.appendChild(renderer.domElement);
    let width = 1,
      height = 1;
    const resize = () => {
      const rect = element.getBoundingClientRect();
      if (rect.width <= 0 || rect.height <= 0) return;
      width = rect.width;
      height = rect.height;
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      lineMaterials.forEach((material) =>
        material.resolution.set(width, height),
      );
    };
    const observer = new ResizeObserver(resize);
    observer.observe(element);
    const applyPose = (model: URDFRobot, point: PreviewPoint) => {
      spec!.joint_names.forEach((name, axis) =>
        model.setJointValue(name, point.joints[axis]),
      );
      model.updateMatrixWorld(true);
    };
    const cloneModel = (color: THREE.Color, opacity: number) => {
      const clone = robot!.clone() as URDFRobot;
      clone.traverse((object) => {
        const mesh = object as THREE.Mesh;
        if (!mesh.material) return;
        const tint = (material: THREE.Material) => {
          const owned = material.clone();
          if ("color" in owned)
            (owned as THREE.MeshStandardMaterial).color.copy(color);
          owned.transparent = opacity < 1;
          owned.opacity = opacity;
          owned.depthWrite = opacity >= 0.8;
          return owned;
        };
        mesh.material = Array.isArray(mesh.material)
          ? mesh.material.map(tint)
          : tint(mesh.material);
        mesh.renderOrder = opacity >= 0.5 ? 2 : 1;
      });
      return clone;
    };
    const addLine = (
      positions: THREE.Vector3[],
      color: number,
      linewidth: number,
      opacity = 1,
    ) => {
      if (positions.length < 2) return;
      const material = new LineMaterial({
        color,
        linewidth,
        transparent: opacity < 1,
        opacity,
        depthTest: false,
        depthWrite: false,
        resolution: new THREE.Vector2(width, height),
      });
      lineMaterials.push(material);
      const line = new Line2(
        new LineGeometry().setPositions(
          positions.flatMap((point) => point.toArray()),
        ),
        material,
      );
      line.computeLineDistances();
      line.renderOrder = linewidth >= 4 ? 5 : 3;
      overlay.add(line);
      return line;
    };
    const abort = new AbortController();
    const manifestUrl = new URL("models/manifest.json", document.baseURI);
    fetch(manifestUrl, { signal: abort.signal })
      .then((r) => {
        if (!r.ok) throw new Error("模型清单加载失败");
        return r.json();
      })
      .then((manifest) => {
        if (disposed) return;
        spec = manifest.sides[side];
        if (!spec) throw new Error("模型配置缺失");
        const manager = new THREE.LoadingManager();
        manager.onError = () => {
          if (!disposed) setError("模型资源加载失败");
        };
        manager.onLoad = () => {
          if (!robot) return;
          if (disposed) {
            disposeObject(robot);
            return;
          }
          modelReady = true;
          const state = current.current;
          if (state.valid)
            spec!.joint_names.forEach((name, index) => {
              const q = (
                state.joints.find((joint) => joint.name === name) ??
                state.joints[index]
              )?.q;
              if (q != null) robot!.setJointValue(name, q);
            });
          robot.updateMatrixWorld(true);
          const box = new THREE.Box3().setFromObject(robot);
          if (!box.isEmpty()) {
            center = box.getCenter(new THREE.Vector3());
            distance = Math.max(
              box.getSize(new THREE.Vector3()).length() * 1.7,
              0.5,
            );
            resetCamera();
          }
          robot.traverse((object) => {
            const mesh = object as THREE.Mesh;
            if (mesh.geometry) sharedGeometry.add(mesh.geometry);
            (Array.isArray(mesh.material) ? mesh.material : [mesh.material])
              .filter(Boolean)
              .forEach((material) => {
                if ("color" in material)
                  material.userData.originalColor = (
                    material as THREE.MeshStandardMaterial
                  ).color.clone();
              });
          });
        };
        const path = spec.urdf.replace(/^\.?\//, "");
        const url = new URL(
          path.startsWith("models/") ? path.slice(7) : path,
          manifestUrl,
        );
        new URDFLoader(manager).load(
          url.href,
          (loaded) => {
            robot = loaded;
            if (disposed) {
              disposeObject(loaded);
              return;
            }
            Object.values(robot.joints).forEach((joint) => {
              joint.ignoreLimits = true;
            });
            scene.add(robot);
          },
          undefined,
          () => {
            if (!disposed) setError("模型加载失败");
          },
        );
      })
      .catch((e) => {
        if (!disposed) setError(e.message);
      });
    renderer.setAnimationLoop(() => {
      const state = current.current;
      if (
        modelReady &&
        robot &&
        spec &&
        (builtPreview !== state.preview || builtStage !== state.previewStage)
      ) {
        const changedPreview = builtPreview !== state.preview;
        disposeObject(overlay, sharedGeometry);
        overlay.clear();
        lineMaterials.length = 0;
        endpointLabels.length = 0;
        labels.current?.replaceChildren();
        planned = undefined;
        builtPreview = state.preview;
        builtStage = state.previewStage;
        selected = state.preview
          ? stagePoints(state.preview, state.previewStage)
          : [];
        if (state.preview && selected.length) {
          planned = cloneModel(plannedColor, 0.72);
          planned.name = "preview-selected-pose";
          overlay.add(planned);
          const envelope =
            changedPreview || !cachedEnvelope
              ? new THREE.Box3()
              : cachedEnvelope.clone();
          // The envelope is optional; compute bounds from every supplied URDF pose.
          if (changedPreview || !cachedEnvelope) {
            for (const point of state.preview.points) {
              applyPose(planned, point);
              envelope.expandByObject(planned);
            }
            cachedEnvelope = envelope.clone();
          }
          const poses = keyPoses(selected);
          poses.forEach((point, index) => {
            const ghost = cloneModel(
              new THREE.Color(
                point.space === "cartesian"
                  ? AXIS_COLORS[point.axis]
                  : JOINT_COLORS[point.axis],
              ),
              0.2 + index * 0.015,
            );
            ghost.name = `preview-key-pose-${index}`;
            applyPose(ghost, point);
            overlay.add(ghost);
          });
          // Split at real stage/segment boundaries, never invent inter-stage paths.
          const chunks: PreviewPoint[][] = [];
          for (const point of selected) {
            const previous = chunks.at(-1)?.at(-1);
            if (
              !previous ||
              previous.space !== point.space ||
              previous.axis !== point.axis ||
              previous.segment !== point.segment
            )
              chunks.push([]);
            chunks.at(-1)!.push(point);
          }
          let arrowCount = 0,
            sweepCount = 0;
          chunks.forEach((chunk) => {
            const color =
              chunk[0].space === "cartesian"
                ? AXIS_COLORS[chunk[0].axis]
                : JOINT_COLORS[chunk[0].axis];
            const path = chunk.map((point) => new THREE.Vector3(...point.tcp));
            const line = addLine(path, color, 4.5);
            if (line)
              line.name = `preview-path-${chunk[0].space}-${chunk[0].axis}-${chunk[0].segment}`;
            if (path.length > 1) {
              const lengths = [0];
              for (let i = 1; i < path.length; i++)
                lengths.push(lengths[i - 1] + path[i].distanceTo(path[i - 1]));
              const total = lengths.at(-1)!;
              if (total > 1e-7)
                for (const fraction of [0.3, 0.75]) {
                  const i = Math.max(
                    1,
                    lengths.findIndex((value) => value >= total * fraction),
                  );
                  const direction = path[i].clone().sub(path[i - 1]);
                  if (direction.lengthSq() < 1e-16) continue;
                  const length = Math.min(0.035, Math.max(total * 0.14, 0.006));
                  const arrow = new THREE.ArrowHelper(
                    direction.normalize(),
                    path[i].clone(),
                    length,
                    color,
                    length * 0.5,
                    length * 0.3,
                  );
                  arrow.traverse((object) => {
                    const drawable = object as THREE.Mesh;
                    if (drawable.material)
                      (Array.isArray(drawable.material)
                        ? drawable.material
                        : [drawable.material]
                      ).forEach((material) => {
                        material.depthTest = false;
                      });
                    object.renderOrder = 6;
                  });
                  overlay.add(arrow);
                  arrowCount++;
                }
            }
            // Joint pivots come from the actual URDF at trajectory samples. Their
            // traces and connecting ribbons expose elbow, wrist and link motion.
            const samples = chunk.filter(
              (_, i) =>
                i % Math.max(1, Math.ceil(chunk.length / 80)) === 0 ||
                i === chunk.length - 1,
            );
            const linkPoses = samples.map((point) => {
              applyPose(planned!, point);
              return spec!.joint_names.map((name) =>
                planned!.joints[name].getWorldPosition(new THREE.Vector3()),
              );
            });
            for (let link = 2; link < spec!.joint_names.length; link++) {
              const trace = addLine(
                linkPoses.map((pose) => pose[link]),
                link === 3 ? 0xffd070 : link === 6 ? 0xd4a0ff : color,
                link === 3 || link === 6 ? 2.8 : 1.2,
                link === 3 || link === 6 ? 0.9 : 0.45,
              );
              if (trace) {
                trace.name = `preview-link-trace-${link}`;
                sweepCount++;
              }
            }
            const surfaces: number[] = [];
            for (let i = 1; i < linkPoses.length; i++)
              for (let link = 1; link < spec!.joint_names.length - 1; link++) {
                const a = linkPoses[i - 1][link],
                  b = linkPoses[i - 1][link + 1],
                  c = linkPoses[i][link],
                  d = linkPoses[i][link + 1];
                surfaces.push(
                  ...a.toArray(),
                  ...b.toArray(),
                  ...c.toArray(),
                  ...b.toArray(),
                  ...d.toArray(),
                  ...c.toArray(),
                );
              }
            const surface = new THREE.Mesh(
              new THREE.BufferGeometry().setAttribute(
                "position",
                new THREE.Float32BufferAttribute(surfaces, 3),
              ),
              new THREE.MeshBasicMaterial({
                color,
                transparent: true,
                opacity: 0.065,
                side: THREE.DoubleSide,
                depthWrite: false,
              }),
            );
            surface.name = "preview-link-sweep";
            overlay.add(surface);
          });
          const cartesianAxes = [
            ...new Set(
              selected
                .filter((point) => point.space === "cartesian")
                .map((point) => point.axis),
            ),
          ];
          const endpoints: {
            point: PreviewPoint;
            label: string;
            color: number;
          }[] = [];
          cartesianAxes.forEach((axis) => {
            const points = selected.filter(
              (point) => point.axis === axis && point.space === "cartesian",
            );
            const base = state.preview!.points.find(
              (point) => point.space === "cartesian",
            )!.tcp[axis];
            for (const sign of [-1, 1]) {
              const point = points.reduce((a, b) =>
                sign * a.tcp[axis] > sign * b.tcp[axis] ? a : b,
              );
              const mm = (point.tcp[axis] - base) * 1000;
              if (Math.abs(mm) < 0.05) continue;
              endpoints.push({
                point,
                label: `${"XYZ"[axis]} ${mm > 0 ? "+" : ""}${mm.toFixed(1)} mm`,
                color: AXIS_COLORS[axis],
              });
            }
          });
          if (!cartesianAxes.length && selected.length > 1) {
            const origin = new THREE.Vector3(...state.preview.points[0].tcp);
            const extrema = [...selected].sort(
              (a, b) =>
                new THREE.Vector3(...b.tcp).distanceToSquared(origin) -
                new THREE.Vector3(...a.tcp).distanceToSquared(origin),
            );
            const p = extrema[0];
            endpoints.push({
              point: p,
              label: `${p.space === "joint_return" ? "归位" : `J${p.axis + 1}`} · TCP ${(new THREE.Vector3(...p.tcp).distanceTo(origin) * 1000).toFixed(1)} mm`,
              color: JOINT_COLORS[p.axis],
            });
          }
          endpoints.forEach(({ point, label, color }) => {
            const span = document.createElement("span");
            span.textContent = label;
            span.style.color = `#${color.toString(16).padStart(6, "0")}`;
            labels.current?.appendChild(span);
            endpointLabels.push({
              position: new THREE.Vector3(...point.tcp),
              element: span,
            });
          });
          const marker = new THREE.Mesh(
            new THREE.SphereGeometry(0.005, 14, 10),
            new THREE.MeshBasicMaterial({
              color: plannedColor,
              depthTest: false,
            }),
          );
          marker.name = "preview-tcp";
          marker.renderOrder = 7;
          overlay.add(marker);
          if (!envelope.isEmpty()) {
            const box = new THREE.Box3Helper(envelope, 0x8696a6);
            box.name = "preview-arm-envelope";
            box.visible = state.showBox;
            overlay.add(box);
            if (changedPreview)
              state.onPreviewEnvelope?.({
                min: envelope.min.toArray(),
                max: envelope.max.toArray(),
                size: envelope.getSize(new THREE.Vector3()).toArray(),
              });
            const frame = new THREE.Box3();
            poses.forEach((point) => {
              applyPose(planned!, point);
              frame.expandByObject(planned!);
            });
            selected.forEach((point) =>
              frame.expandByPoint(new THREE.Vector3(...point.tcp)),
            );
            center = frame.getCenter(new THREE.Vector3());
            const size = frame.getSize(new THREE.Vector3());
            distance = Math.max(
              size.length() * (camera.aspect < 1 ? 1.75 / camera.aspect : 1.5),
              0.45,
            );
            resetCamera();
            element.dataset.envelope = JSON.stringify({
              min: envelope.min.toArray(),
              max: envelope.max.toArray(),
            });
          }
          element.dataset.previewPoints = String(state.preview.points.length);
          element.dataset.previewStage = state.previewStage;
          element.dataset.selectedPoints = String(selected.length);
          element.dataset.keyPoses = String(poses.length);
          element.dataset.pathWidth = "4.5";
          element.dataset.arrows = String(arrowCount);
          element.dataset.linkTraces = String(sweepCount);
          element.dataset.cartesianSegments = String(
            selected.filter((point) => point.space === "cartesian").length,
          );
          element.dataset.jointPoints = String(
            selected.filter((point) => point.space !== "cartesian").length,
          );
        } else {
          cachedEnvelope = undefined;
          if (changedPreview) state.onPreviewEnvelope?.(null);
          for (const key of [
            "envelope",
            "previewPoints",
            "selectedPoints",
            "keyPoses",
            "arrows",
            "linkTraces",
            "plannedJoints",
          ])
            delete element.dataset[key];
        }
      }
      const box = overlay.getObjectByName("preview-arm-envelope");
      if (box) box.visible = state.showBox;
      element.dataset.aabb = String(!!box?.visible);
      if (planned && selected.length && spec) {
        const at =
          THREE.MathUtils.clamp(state.previewProgress, 0, 1) *
          (selected.length - 1);
        const from = selected[Math.floor(at)],
          to = selected[Math.min(selected.length - 1, Math.floor(at) + 1)];
        // Stage transitions are discrete; interpolation only belongs to one path.
        const blend =
          from.space === to.space &&
          from.axis === to.axis &&
          from.segment === to.segment
            ? at % 1
            : 0;
        spec.joint_names.forEach((name, axis) =>
          planned!.setJointValue(
            name,
            THREE.MathUtils.lerp(from.joints[axis], to.joints[axis], blend),
          ),
        );
        element.dataset.plannedJoints = JSON.stringify(
          spec.joint_names.map((name) => planned!.joints[name].jointValue[0]),
        );
        overlay
          .getObjectByName("preview-tcp")
          ?.position.lerpVectors(
            new THREE.Vector3(...from.tcp),
            new THREE.Vector3(...to.tcp),
            blend,
          );
      }
      if (robot && spec) {
        if (state.valid)
          spec.joint_names.forEach((name, index) => {
            const q = (
              state.joints.find((joint) => joint.name === name) ??
              state.joints[index]
            )?.q;
            if (q != null) robot!.setJointValue(name, q);
          });
        element.dataset.actualJoints = JSON.stringify(
          spec.joint_names.map((name) => robot!.joints[name].jointValue[0]),
        );
        robot.traverse((object) => {
          const mesh = object as THREE.Mesh;
          (Array.isArray(mesh.material) ? mesh.material : [mesh.material])
            .filter(Boolean)
            .forEach((material) => {
              if ("color" in material && material.userData.originalColor)
                (material as THREE.MeshStandardMaterial).color.copy(
                  state.preview || !state.valid
                    ? referenceColor
                    : material.userData.originalColor,
                );
            });
        });
      }
      controls.update();
      camera.updateMatrixWorld();
      const occupied: { x: number; y: number; w: number }[] = [];
      endpointLabels.forEach(({ position, element: label }, index) => {
        const p = position.clone().project(camera);
        const w = Math.min(label.offsetWidth || 100, width - 16);
        const x = THREE.MathUtils.clamp(
          ((p.x + 1) * width) / 2 + (index % 2 ? 10 : -w - 10),
          8,
          width - w - 8,
        );
        let y = THREE.MathUtils.clamp(
          ((1 - p.y) * height) / 2 - 10,
          54,
          height - 62,
        );
        for (
          let tries = 0;
          tries < 12 &&
          occupied.some(
            (rect) =>
              Math.abs(rect.y - y) < 23 &&
              x < rect.x + rect.w + 5 &&
              x + w + 5 > rect.x,
          );
          tries++
        )
          y = 54 + ((y - 54 + 24) % Math.max(24, height - 132));
        occupied.push({ x, y, w });
        label.style.transform = `translate(${Math.round(x)}px, ${Math.round(y)}px)`;
        label.hidden = p.z < -1 || p.z > 1;
      });
      if (ruler.current) {
        const mmPerPixel =
          (2000 *
            Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)) *
            camera.position.distanceTo(controls.target)) /
          height;
        const ideal = mmPerPixel * 65;
        const magnitude = 10 ** Math.floor(Math.log10(ideal));
        const mm =
          [1, 2, 5, 10]
            .map((value) => value * magnitude)
            .find((value) => value >= ideal) ?? magnitude;
        ruler.current.style.width = `${mm / mmPerPixel}px`;
        ruler.current.textContent = `${mm} mm`;
      }
      renderer.render(scene, camera);
    });
    return () => {
      disposed = true;
      abort.abort();
      observer.disconnect();
      renderer.setAnimationLoop(null);
      controls.dispose();
      disposeObject(scene);
      renderer.dispose();
      renderer.domElement.remove();
      labels.current?.replaceChildren();
    };
  }, [side]);
  return (
    <div className="scene" data-preview={!!preview}>
      <div className="scene-canvas" ref={container} />
      <div
        className="scene-distance-labels"
        ref={labels}
        aria-label="路径端点距离"
      />
      <div className="scene-toolbar">
        <span className="source-badge" role="status">
          {source}
        </span>
        <div className="scene-views" aria-label="模型视角">
          {(
            [
              ["orbit", "立体"],
              ["top", "顶视"],
              ["front", "前视"],
              ["side", "侧视"],
            ] as const
          ).map(([key, label]) => (
            <button
              key={key}
              aria-pressed={cameraView === key}
              onClick={() => {
                setCameraView(key);
                changeView.current(key);
              }}
            >
              {label}
            </button>
          ))}
        </div>
      </div>
      <div className="scene-axes">
        <b className="axis-x">X</b>
        <b className="axis-y">Y</b>
        <b className="axis-z">Z</b>
        <span>基座 · 网格 100 mm</span>
      </div>
      <div className="scene-scale" ref={ruler} aria-label="毫米比例尺" />
      {error && (
        <div className="scene-error" role="status">
          {error}
        </div>
      )}
      {preview && (
        <div className="scene-preview-legend" aria-label="预览图例">
          <span data-kind="actual">实测</span>
          <span data-kind="planned">选中姿态</span>
          <span data-kind="sweep">关键姿态</span>
          <span data-kind="elbow">肘轨迹</span>
          <span data-kind="wrist">腕轨迹</span>
          <label>
            <input
              type="checkbox"
              checked={showBox}
              onChange={(event) => setShowBox(event.target.checked)}
            />
            范围盒
          </label>
        </div>
      )}
    </div>
  );
}
