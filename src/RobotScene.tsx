import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import URDFLoader, { type URDFRobot } from "urdf-loader";
import type { Joint, Side } from "./api";
interface Props {
  side: Side;
  joints: Joint[];
  valid: boolean;
  source: string;
}
interface ModelSpec {
  joint_names: string[];
  lower: number[];
  upper: number[];
  urdf: string;
}
function disposeObject(object: THREE.Object3D) {
  object.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if (mesh.geometry) mesh.geometry.dispose();
    if (mesh.material) {
      const materials = Array.isArray(mesh.material)
        ? mesh.material
        : [mesh.material];
      materials.forEach((m) => m.dispose());
    }
  });
}
export default function RobotScene({ side, joints, valid, source }: Props) {
  const container = useRef<HTMLDivElement>(null);
  const current = useRef({ joints, valid });
  current.current = { joints, valid };
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
    let spec: ModelSpec | undefined;
    let lastValid = false;
    const theme = getComputedStyle(document.documentElement);
    const themeColor = (token: string) =>
      new THREE.Color(theme.getPropertyValue(token).trim());
    const referenceColor = themeColor("--scene-model-reference");
    const scene = new THREE.Scene();
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
      if (robot && spec) {
        const state = current.current;
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
    <div className="scene">
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
    </div>
  );
}
