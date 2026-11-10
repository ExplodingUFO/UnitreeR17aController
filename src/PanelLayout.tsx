import {
  useEffect,
  useLayoutEffect,
  useRef,
  useState,
  type CSSProperties,
  type HTMLAttributes,
  type KeyboardEvent,
  type PointerEvent,
} from "react";

type Panel = "left" | "right" | "tcp" | "logs";
type Sizes = Record<Panel, number>;
type Geometry = { width: number; height: number };
const GUTTER = 8;
const MIN_LEFT = 196;
const MIN_RIGHT = 216;
const MIN_CENTER = 260;
const MIN_WORKSPACE = 390;
const MIN_LOGS = 36;
const MIN_SCENE = 160;
const MIN_TCP = 128;
const STORAGE_KEY = "r17a.panel-layout.v1";
const panels = {
  left: {
    label: "调整连接栏宽度",
    target: "device-panel",
    vertical: true,
    direction: 1,
  },
  right: {
    label: "调整点动栏宽度",
    target: "jog-panel",
    vertical: true,
    direction: -1,
  },
  tcp: {
    label: "调整末端坐标区高度",
    target: "tcp-panel",
    vertical: false,
    direction: -1,
  },
  logs: {
    label: "调整日志区高度",
    target: "logs-panel",
    vertical: false,
    direction: -1,
  },
} as const;

const clamp = (value: number, min: number, max: number) =>
  Math.round(Math.max(min, Math.min(max, value)));

function defaults(): Sizes {
  return {
    left: window.innerWidth <= 1100 ? 216 : 260,
    right: window.innerWidth <= 1100 ? 232 : 264,
    tcp: 144,
    logs: MIN_LOGS,
  };
}

function load(): Sizes {
  const sizes = defaults();
  try {
    const saved = JSON.parse(localStorage.getItem(STORAGE_KEY) ?? "null");
    for (const key of Object.keys(panels) as Panel[]) {
      if (
        typeof saved?.[key] === "number" &&
        Number.isFinite(saved[key]) &&
        saved[key] > 0
      ) {
        sizes[key] = saved[key];
      }
    }
  } catch {
    // Layout storage is optional; the controller remains usable without it.
  }
  return sizes;
}

function save(sizes: Sizes) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(sizes));
  } catch {
    // Keep the current in-memory layout when storage is unavailable.
  }
}

function fit(sizes: Sizes, geometry: Geometry): Sizes {
  const sideBudget = Math.max(
    MIN_LEFT + MIN_RIGHT,
    geometry.width - MIN_CENTER - GUTTER * 2,
  );
  const leftExtra = Math.max(0, sizes.left - MIN_LEFT);
  const rightExtra = Math.max(0, sizes.right - MIN_RIGHT);
  const scale = Math.min(
    1,
    (sideBudget - MIN_LEFT - MIN_RIGHT) / Math.max(1, leftExtra + rightExtra),
  );
  const logs = clamp(sizes.logs, MIN_LOGS, geometry.height - MIN_WORKSPACE);
  return {
    left: MIN_LEFT + Math.floor(leftExtra * scale),
    right: MIN_RIGHT + Math.floor(rightExtra * scale),
    logs,
    tcp: clamp(sizes.tcp, MIN_TCP, geometry.height - logs - GUTTER - MIN_SCENE),
  };
}

function bounds(
  panel: Panel,
  sizes: Sizes,
  geometry: Geometry,
): [number, number] {
  switch (panel) {
    case "left":
      return [MIN_LEFT, geometry.width - GUTTER * 2 - sizes.right - MIN_CENTER];
    case "right":
      return [MIN_RIGHT, geometry.width - GUTTER * 2 - sizes.left - MIN_CENTER];
    case "tcp":
      return [MIN_TCP, geometry.height - sizes.logs - GUTTER - MIN_SCENE];
    case "logs":
      return [MIN_LOGS, geometry.height - MIN_WORKSPACE];
  }
}

type Drag = {
  panel: Panel;
  pointer: number;
  origin: number;
  start: Sizes;
  before: Sizes;
};
type DividerProps = HTMLAttributes<HTMLDivElement> & {
  panel: Panel;
  value: number;
  min: number;
  max: number;
  active: boolean;
};

export function PanelDivider({
  panel,
  value,
  min,
  max,
  active,
  ...events
}: DividerProps) {
  const spec = panels[panel];
  return (
    <div
      {...events}
      className={`panel-divider ${spec.vertical ? "vertical" : "horizontal"}${active ? " resizing" : ""} divider-${panel}`}
      role="separator"
      tabIndex={0}
      aria-label={spec.label}
      aria-orientation={spec.vertical ? "vertical" : "horizontal"}
      aria-controls={spec.target}
      aria-valuemin={Math.round(min)}
      aria-valuemax={Math.round(max)}
      aria-valuenow={value}
      aria-valuetext={`${value} 像素`}
      title="拖动调整大小，双击恢复默认"
    />
  );
}

export function usePanelLayout() {
  const rootRef = useRef<HTMLDivElement>(null);
  const workspaceRef = useRef<HTMLElement>(null);
  const [preferred, setPreferred] = useState(load);
  const [geometry, setGeometry] = useState<Geometry>({
    width: Math.max(688, Math.min(1520, window.innerWidth) - 20),
    height: Math.max(MIN_WORKSPACE + MIN_LOGS, window.innerHeight - 80),
  });
  const [active, setActive] = useState<Panel | null>(null);
  const drag = useRef<Drag | null>(null);
  const sizes = fit(preferred, geometry);
  const current = useRef({ sizes, preferred, geometry });
  current.current = { sizes, preferred, geometry };

  useLayoutEffect(() => {
    const root = rootRef.current;
    const workspace = workspaceRef.current;
    if (!root || !workspace) return;
    const measure = () => {
      if (root.dataset.page !== "control") return;
      const rootRect = root.getBoundingClientRect();
      const workspaceRect = workspace.getBoundingClientRect();
      const error = root.querySelector<HTMLElement>(".error-line");
      const next = {
        width: workspace.clientWidth,
        height: Math.max(
          MIN_WORKSPACE + MIN_LOGS,
          root.clientHeight -
            (workspaceRect.top - rootRect.top) -
            parseFloat(getComputedStyle(root).paddingBottom) -
            (error?.offsetHeight ?? 0) -
            GUTTER,
        ),
      };
      setGeometry((previous) =>
        previous.width === next.width && previous.height === next.height
          ? previous
          : next,
      );
    };
    measure();
    const observer = new ResizeObserver(measure);
    observer.observe(root);
    observer.observe(workspace);
    return () => observer.disconnect();
  }, []);

  const commit = (next: Sizes, persist = false) => {
    current.current = { ...current.current, sizes: next, preferred: next };
    setPreferred(next);
    if (persist) save(next);
  };
  const finish = () => {
    if (!drag.current) return;
    drag.current = null;
    setActive(null);
    save(current.current.sizes);
  };
  const cancel = () => {
    const started = drag.current;
    if (!started) return;
    drag.current = null;
    setActive(null);
    setPreferred(started.before);
  };
  useEffect(() => {
    window.addEventListener("blur", finish);
    return () => window.removeEventListener("blur", finish);
  }, []);

  const divider = (panel: Panel): DividerProps => {
    const spec = panels[panel];
    const [min, max] = bounds(panel, sizes, geometry);
    const change = (value: number, persist: boolean) => {
      const state = current.current;
      const [low, high] = bounds(panel, state.sizes, state.geometry);
      commit(
        fit(
          { ...state.sizes, [panel]: clamp(value, low, high) },
          state.geometry,
        ),
        persist,
      );
    };
    const move = (event: PointerEvent<HTMLDivElement>) => {
      const started = drag.current;
      if (
        !started ||
        started.panel !== panel ||
        started.pointer !== event.pointerId
      )
        return;
      const position = spec.vertical ? event.clientX : event.clientY;
      change(
        started.start[panel] + (position - started.origin) * spec.direction,
        false,
      );
    };
    const keyDown = (event: KeyboardEvent<HTMLDivElement>) => {
      if (event.key === "Escape") {
        cancel();
        return;
      }
      if (drag.current) return;
      const negative = spec.vertical ? "ArrowLeft" : "ArrowUp";
      const positive = spec.vertical ? "ArrowRight" : "ArrowDown";
      const step = event.shiftKey ? 32 : 8;
      let next: number;
      if (event.key === negative || event.key === positive) {
        next =
          current.current.sizes[panel] +
          (event.key === positive ? step : -step) * spec.direction;
      } else if (event.key === "Home") next = min;
      else if (event.key === "End") next = max;
      else if (event.key === "Enter") next = defaults()[panel];
      else return;
      event.preventDefault();
      change(next, true);
    };
    return {
      panel,
      value: sizes[panel],
      min,
      max,
      active: active === panel,
      onPointerDown: (event) => {
        if (event.button !== 0 || drag.current) return;
        event.preventDefault();
        event.currentTarget.setPointerCapture(event.pointerId);
        event.currentTarget.focus({ preventScroll: true });
        drag.current = {
          panel,
          pointer: event.pointerId,
          origin: spec.vertical ? event.clientX : event.clientY,
          start: { ...current.current.sizes },
          before: { ...current.current.preferred },
        };
        setActive(panel);
      },
      onPointerMove: move,
      onPointerUp: (event) => {
        if (drag.current?.pointer !== event.pointerId) return;
        move(event);
        finish();
      },
      onPointerCancel: (event) => {
        if (drag.current?.pointer === event.pointerId) cancel();
      },
      onLostPointerCapture: (event) => {
        if (drag.current?.pointer === event.pointerId) finish();
      },
      onDoubleClick: () => change(defaults()[panel], true),
      onKeyDown: keyDown,
    };
  };

  return {
    rootRef,
    workspaceRef,
    divider,
    logsCollapsed: sizes.logs <= 48,
    toggleLogs: () => {
      const state = current.current;
      commit(
        fit(
          { ...state.preferred, logs: state.sizes.logs <= 48 ? 160 : MIN_LOGS },
          state.geometry,
        ),
        true,
      );
    },
    resizing: active
      ? panels[active].vertical
        ? "vertical"
        : "horizontal"
      : undefined,
    shortWorkspace: geometry.height - sizes.logs < 480,
    style: {
      "--panel-gutter": `${GUTTER}px`,
      "--left-panel-size": `${sizes.left}px`,
      "--right-panel-size": `${sizes.right}px`,
      "--tcp-panel-size": `${sizes.tcp}px`,
      "--logs-panel-size": `${sizes.logs}px`,
    } as CSSProperties,
  };
}
