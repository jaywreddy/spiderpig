import * as THREE from 'three';

/** An arm's joint sliders (2026-10-12): pose the baked arm anywhere in its box.
 *
 * The bake animates an arm over a tour of its inputs; a slider per joint (the linkage's
 * inputs from ``/api/linkages``, the turntable's and wrist's from the glb's ``tree``) asks
 * ``/api/pose/<mode>`` for each node's matrix from the clip's first frame
 * (``spiderpig.pose.node_poses``); the node posed is that times its first-frame matrix. */

export interface Joint {
  name: string;
  label: string;
  lo: number;      // degrees
  hi: number;
  value: number;
}

interface LinkageInfo {
  key: string;
  kind: string;
  inputs?: string[];
  ranges?: Record<string, [number, number]>;
}

interface TreeJoint { name: string; range_deg: [number, number]; label?: string }

/** The joints of an arm: its planar inputs (their ranges, starting mid-range) and its
 *  spatial joints (from the glb's ``tree``: starting at 0, the first frame's when it
 *  is mid-range). Empty for anything but an arm. */
export function jointsOf(info: LinkageInfo | undefined, tree: { joints?: TreeJoint[] } | undefined): Joint[] {
  if (!info || info.kind !== 'arm') return [];
  const out: Joint[] = [];
  for (const name of info.inputs ?? []) {
    const r = info.ranges?.[name];
    if (!r) continue;
    out.push({ name, label: name, lo: r[0], hi: r[1], value: (r[0] + r[1]) / 2 });
  }
  for (const j of tree?.joints ?? []) {
    out.push({ name: j.name, label: j.label || j.name, lo: j.range_deg[0], hi: j.range_deg[1],
      value: (j.range_deg[0] + j.range_deg[1]) / 2 });
  }
  return out;
}

/** The node's matrix posed: the server's pose (16 numbers, column-major) times its
 *  first-frame matrix. */
export function posedMatrix(pose: ArrayLike<number>, frame0: THREE.Matrix4): THREE.Matrix4 {
  return new THREE.Matrix4().fromArray(Array.from(pose)).multiply(frame0);
}

/** The query that asks for a pose: the design's query and ``j.<name>=<degrees>`` each. */
export function poseQuery(designQuery: string, joints: readonly Joint[]): string {
  const p = new URLSearchParams(designQuery);
  for (const j of joints) p.set(`j.${j.name}`, j.value.toFixed(4));
  return p.toString();
}

export interface ArmPanelDeps {
  container: HTMLElement;
  /** The loaded glb's root (its nodes) and the mode and query it was baked for. */
  current: () => { root: THREE.Object3D; mode: string; query: string; pause: () => void } | null;
  invalidate: () => void;
  status: (text: string) => void;
}

/** The panel: built on each load (``onLoad``), hidden for a walker. */
export function createArmPanel(deps: ArmPanelDeps) {
  let linkages: LinkageInfo[] | null = null;
  let joints: Joint[] = [];
  let frame0 = new Map<string, THREE.Matrix4>();
  let pending: AbortController | null = null;

  async function info(key: string): Promise<LinkageInfo | undefined> {
    if (!linkages) {
      const res = await fetch('/api/linkages');
      linkages = res.ok ? ((await res.json()) as { linkages: LinkageInfo[] }).linkages : [];
    }
    return linkages.find((l) => l.key === key);
  }

  async function apply(): Promise<void> {
    const cur = deps.current();
    if (!cur || joints.length === 0) return;
    pending?.abort();
    const ctl = new AbortController();
    pending = ctl;
    const res = await fetch(`/api/pose/${encodeURIComponent(cur.mode)}?${poseQuery(cur.query, joints)}`,
      { signal: ctl.signal }).catch(() => null);
    if (!res || !res.ok) { if (res) deps.status(`pose: ${res.status}`); return; }
    const poses = (await res.json()) as Record<string, number[]>;
    cur.pause();
    cur.root.traverse((obj) => {
      const pose = poses[obj.name];
      const m0 = frame0.get(obj.name);
      if (!pose || !m0) return;
      posedMatrix(pose, m0).decompose(obj.position, obj.quaternion, obj.scale);
    });
    deps.invalidate();
  }

  async function onLoad(root: THREE.Object3D): Promise<void> {
    deps.container.replaceChildren();
    const key = String(root.userData.linkage ?? '');
    joints = jointsOf(await info(key), root.userData.tree as { joints?: TreeJoint[] } | undefined);
    deps.container.hidden = joints.length === 0;
    frame0 = new Map();
    root.traverse((obj) => { if (obj.name) frame0.set(obj.name, obj.matrix.clone()); });
    for (const j of joints) {
      const label = document.createElement('label');
      label.textContent = j.label;
      const input = document.createElement('input');
      input.type = 'range';
      input.min = String(j.lo);
      input.max = String(j.hi);
      input.step = '0.1';
      input.value = String(j.value);
      input.dataset.joint = j.name;
      input.title = `${j.name}: ${j.lo}..${j.hi} deg`;
      input.addEventListener('input', () => { j.value = Number(input.value); void apply(); });
      label.append(input);
      deps.container.append(label);
    }
  }

  return { onLoad, get joints() { return joints; } };
}
