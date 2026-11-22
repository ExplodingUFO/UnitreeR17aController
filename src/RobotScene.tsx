import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import URDFLoader, { type URDFRobot } from "urdf-loader";
import type { CalibrationPreview, Joint, Side } from "./api";
export interface PreviewEnvelope {
  min: number[];
  max: number[];
  size: number[];
}
interface Props {
  side: Side;
  joints: Joint[];
  valid: boolean;
  source: string;
  preview?: CalibrationPreview | null;
  previewProgress?: number;
  onPreviewEnvelope?: (envelope: PreviewEnvelope | null) => void;
}
interface ModelSpec {
  joint_names: string[];
  lower: number[];
  upper: number[];
  urdf: string;
}
function disposeObject(object: THREE.Object3D) {
  const geometries = new Set<THREE.BufferGeometry>();
  const materials = new Set<THREE.Material>();
  object.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if (mesh.geometry) geometries.add(mesh.geometry);
    if (mesh.material) {
      const owned = Array.isArray(mesh.material)
        ? mesh.material
        : [mesh.material];
      owned.forEach((m) => materials.add(m));
    }
  });
  geometries.forEach((geometry) => geometry.dispose());
  materials.forEach((material) => material.dispose());
}
export default function RobotScene({
  side,
  joints,
  valid,
  source,
  preview,
  previewProgress = 0,
  onPreviewEnvelope,
}: Props) {
  const container = useRef<HTMLDivElement>(null);
  const current = useRef({
    joints,
    valid,
    preview,
    previewProgress,
    onPreviewEnvelope,
  });
  current.current = {
    joints,
    valid,
    preview,
    previewProgress,
    onPreviewEnvelope,
  };
  const reset = useRef<() => void>(() => {});
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
    let lastValid = false;
    const theme = getComputedStyle(document.documentElement);
    const themeColor = (token: string) =>
      new THREE.Color(theme.getPropertyValue(token).trim());
    const referenceColor = themeColor("--scene-model-reference");
    const plannedColor = themeColor("--color-focus");
    const jointPathColor = themeColor("--color-warning");
    const scene = new THREE.Scene();
    const overlay = new THREE.Group();
    scene.add(overlay);
    scene.background = themeColor("--scene-background");
    const camera = new THREE.PerspectiveCamera(38, 1, 0.005, 100);
    // URDF remains Z-up; rotate only the world-view helper coordinate convention.
    camera.up.set(0, 0, 1);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = !window.matchMedia(
      "(prefers-reduced-motion: reduce)",
    ).matches;
    controls.minDistance = 0.18;
    controls.maxDistance = 4;
    let center = new THREE.Vector3(0, 0, 0.25);
    let distance = 1.3;
    const resetCamera = () => {
      camera.position
        .copy(center)
        .add(new THREE.Vector3(distance, -distance * 0.9, distance * 0.65));
      controls.target.copy(center);
      controls.update();
    };
    reset.current = resetCamera;
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
    const axes = new THREE.AxesHelper(0.16);
    axes.setColors(
      themeColor("--axis-x"),
      themeColor("--axis-y"),
      themeColor("--axis-z"),
    );
    scene.add(axes);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    element.appendChild(renderer.domElement);
    const observer = new ResizeObserver(() => {
      const { width, height } = element.getBoundingClientRect();
      if (width <= 0 || height <= 0) return;
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    });
    observer.observe(element);
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
          if (!disposed && robot) {
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
                box.getSize(new THREE.Vector3()).length() * 0.95,
                0.45,
              );
              resetCamera();
            }
          }
        };
        const loader = new URDFLoader(manager);
        const path = spec.urdf.replace(/^\.?\//, "");
        const url = new URL(
          path.startsWith("models/") ? path.slice(7) : path,
          manifestUrl,
        );
        loader.load(
          url.href,
          (loaded) => {
            if (disposed) {
              disposeObject(loaded);
              return;
            }
            robot = loaded;
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
      if (modelReady && robot && spec && builtPreview !== state.preview) {
        disposeObject(overlay);
        overlay.clear();
        builtPreview = state.preview;
        if (planned) planned.visible = !!state.preview;
        if (state.preview) {
          if (!planned) {
            planned = robot.clone() as URDFRobot;
            // Object3D.clone shares mesh resources: own every ghost material.
            // Geometry is immutable and disposed once with the complete scene.
            planned.traverse((object) => {
              const mesh = object as THREE.Mesh;
              if (!mesh.material) return;
              const tint = (material: THREE.Material) => {
                const clone = material.clone();
                if ("color" in clone)
                  (clone as THREE.MeshStandardMaterial).color.copy(
                    plannedColor,
                  );
                clone.transparent = true;
                clone.opacity = 0.32;
                clone.depthWrite = false;
                return clone;
              };
              mesh.material = Array.isArray(mesh.material)
                ? mesh.material.map(tint)
                : tint(mesh.material);
              mesh.renderOrder = 2;
            });
            scene.add(planned);
          }
          const envelope = new THREE.Box3();
          const { points } = state.preview;
          // Include every supplied pose, including all seven joint trials.
          for (const point of points) {
            spec.joint_names.forEach((name, axis) =>
              planned!.setJointValue(name, point.joints[axis]),
            );
            planned.updateMatrixWorld(true);
            envelope.expandByObject(planned);
          }
          for (const space of ["cartesian", "joint"] as const) {
            const positions: THREE.Vector3[] = [];
            // Separate line segments prevent a fictitious connection between programmes.
            for (let index = 1; index < points.length; index++) {
              const from = points[index - 1],
                to = points[index];
              if (
                (from.space === "cartesian" ? "cartesian" : "joint") ===
                  space &&
                (to.space === "cartesian" ? "cartesian" : "joint") === space
              )
                positions.push(
                  new THREE.Vector3(...from.tcp),
                  new THREE.Vector3(...to.tcp),
                );
            }
            const line = new THREE.LineSegments(
              new THREE.BufferGeometry().setFromPoints(positions),
              new THREE.LineBasicMaterial({
                color: space === "cartesian" ? plannedColor : jointPathColor,
                depthTest: false,
                transparent: true,
                opacity: 0.9,
              }),
            );
            line.name = `preview-path-${space}`;
            line.renderOrder = 3;
            overlay.add(line);
          }
          const tcpMarker = new THREE.Mesh(
            new THREE.SphereGeometry(0.004, 12, 8),
            new THREE.MeshBasicMaterial({
              color: plannedColor,
              depthTest: false,
            }),
          );
          tcpMarker.name = "preview-tcp";
          tcpMarker.renderOrder = 4;
          overlay.add(tcpMarker);
          if (!envelope.isEmpty()) {
            const box = new THREE.Box3Helper(envelope, plannedColor);
            box.name = "preview-arm-envelope";
            const boxMaterials = Array.isArray(box.material)
              ? box.material
              : [box.material];
            boxMaterials.forEach((material) => {
              material.transparent = true;
              material.opacity = 0.45;
              material.depthWrite = false;
            });
            overlay.add(box);
            state.onPreviewEnvelope?.({
              min: envelope.min.toArray(),
              max: envelope.max.toArray(),
              size: envelope.getSize(new THREE.Vector3()).toArray(),
            });
            const frame = envelope.clone();
            points.forEach((point) =>
              frame.expandByPoint(new THREE.Vector3(...point.tcp)),
            );
            center = frame.getCenter(new THREE.Vector3());
            distance = Math.max(
              frame.getSize(new THREE.Vector3()).length() * 1.05,
              0.45,
            );
            resetCamera();
            element.dataset.envelope = JSON.stringify({
              min: envelope.min.toArray(),
              max: envelope.max.toArray(),
            });
          }
          element.dataset.previewPoints = String(points.length);
          element.dataset.cartesianSegments = String(
            overlay.getObjectByName("preview-path-cartesian") &&
              points.filter((point) => point.space === "cartesian").length,
          );
          element.dataset.jointPoints = String(
            points.filter((point) => point.space !== "cartesian").length,
          );
        } else {
          state.onPreviewEnvelope?.(null);
          delete element.dataset.envelope;
          delete element.dataset.previewPoints;
          delete element.dataset.cartesianSegments;
          delete element.dataset.jointPoints;
        }
      }
      if (planned && state.preview) {
        const points = state.preview.points;
        const at = Math.min(
          points.length - 1,
          Math.max(0, state.previewProgress) * (points.length - 1),
        );
        const from = points[Math.floor(at)],
          to = points[Math.min(points.length - 1, Math.floor(at) + 1)];
        if (from && to && spec) {
          spec.joint_names.forEach((name, axis) =>
            planned!.setJointValue(
              name,
              THREE.MathUtils.lerp(from.joints[axis], to.joints[axis], at % 1),
            ),
          );
          element.dataset.plannedJoints = JSON.stringify(
            spec.joint_names.map((name) => planned!.joints[name].jointValue[0]),
          );
          const tcpMarker = overlay.getObjectByName("preview-tcp");
          tcpMarker?.position.lerpVectors(
            new THREE.Vector3(...from.tcp),
            new THREE.Vector3(...to.tcp),
            at % 1,
          );
        }
      }
      if (robot && spec) {
        if (state.valid) {
          const values: Record<string, number> = {};
          spec.joint_names.forEach((name, index) => {
            const joint =
              state.joints.find((j) => j.name === name) ?? state.joints[index];
            if (joint?.q !== null && joint?.q !== undefined)
              values[name] = joint.q;
          });
          robot.setJointValues(values);
        }
        element.dataset.actualJoints = JSON.stringify(
          spec.joint_names.map((name) => robot!.joints[name].jointValue[0]),
        );
        if (lastValid !== state.valid || !robot.userData.colored) {
          robot.traverse((object) => {
            const mesh = object as THREE.Mesh;
            if (!mesh.material) return;
            const materials = Array.isArray(mesh.material)
              ? mesh.material
              : [mesh.material];
            materials.forEach((material) => {
              if (!material.userData.originalColor && "color" in material)
                material.userData.originalColor = (
                  material as THREE.MeshStandardMaterial
                ).color.clone();
              if ("color" in material) {
                const color = (material as THREE.MeshStandardMaterial).color;
                color.copy(
                  state.valid
                    ? material.userData.originalColor
                    : referenceColor,
                );
              }
            });
          });
          robot.userData.colored = true;
          lastValid = state.valid;
        }
      }
      controls.update();
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
    };
  }, [side]);
  return (
    <div className="scene" data-preview={!!preview}>
      <div className="scene-canvas" ref={container} />
      <div className="scene-toolbar">
        <span className="source-badge" role="status">
          {source}
        </span>
        <button onClick={() => reset.current()}>复位视角</button>
      </div>
      <div className="scene-axes">
        <b className="axis-x">X</b>
        <b className="axis-y">Y</b>
        <b className="axis-z">Z</b>
        <span>基座坐标（m）</span>
      </div>
      {error && (
        <div className="scene-error" role="status">
          {error}
        </div>
      )}
      {preview && (
        <div className="scene-preview-legend" aria-label="预览图例">
          <span data-kind="actual">实测</span>
          <span data-kind="planned">计划 / 全臂包络</span>
          <span data-kind="cartesian">TCP · 末端动作</span>
          <span data-kind="joint">TCP · 七轴试验</span>
        </div>
      )}
    </div>
  );
}
