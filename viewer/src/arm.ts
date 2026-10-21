import * as THREE from 'three';
import { PhysicsLink, type Hello } from './drive/physics';
import type { LoadedScene } from './loader';

/** A fixed design's panel: an arm's joint sliders (2026-10-12) and the live sim of any design
 * that doesn't walk (2026-10-14).
 *
 * The bake animates an arm over a tour of its inputs; a slider per joint (the linkage's ranged
 * inputs from ``/api/linkages``, the turntable's and wrist's from the glb's ``tree``) asks
 * ``/api/pose/<mode>`` for each node's matrix from the clip's first frame
 * (``spiderpig.pose.node_poses``); the node posed is that times its first-frame matrix.
 *
 * **live (MuJoCo)** hands the parts to the server's model of the design on screen
 * (``/ws/sim``, the one session ``drive/physics.ts``'s ``PhysicsLink`` speaks, the walker's
 * physics drive's): what it offers follows the hello, never the kind's name. Each joint the
 * session declares (``hello.joints``: an arm's steppers) takes its slider's angle as its
 * target; each crank (``hello.drives``: a mechanism's servo) the crank-speed slider's
 * fraction of its no-load speed. A walker (``walks``) is driven from the drive panel. */

export interface Joint {
  name: string;
  label: string;
  lo: number;      // degrees
  hi: number;
  value: number;
}

export interface LinkageInfo {
  key: string;
  kind: string;
  inputs?: string[];
  ranges?: Record<string, [number, number]>;
  /** What it declares (``/api/linkages``): feet on a floor; ranged inputs (no crank). */
  walks?: boolean;
  ranged?: boolean;
  feet?: number;
}

interface TreeJoint { name: string; range_deg: [number, number]; label?: string }

/** The joints of a design with ranged inputs (an arm), each starting where the clip's first
 *  frame has it: its planar inputs (their ranges; ``inputs0``, the bake's first-frame inputs,
 *  else mid-range) and its spatial joints (from the glb's ``tree``: mid-range, the bake's first
 *  frame). Empty for a design whose inputs are a crank's. */
export function jointsOf(info: LinkageInfo | undefined, tree: { joints?: TreeJoint[] } | undefined,
  inputs0: Record<string, number> = {}): Joint[] {
  if (!info?.ranged) return [];
  const out: Joint[] = [];
  for (const name of info.inputs ?? []) {
    const r = info.ranges?.[name];
    if (!r) continue;
    out.push({ name, label: name, lo: r[0], hi: r[1], value: inputs0[name] ?? (r[0] + r[1]) / 2 });
  }
  for (const j of tree?.joints ?? []) {
    out.push({ name: j.name, label: j.label || j.name, lo: j.range_deg[0], hi: j.range_deg[1],
      value: (j.range_deg[0] + j.range_deg[1]) / 2 });
  }
  return out;
}

/** Whether this panel runs the design live: one that doesn't walk (a walker's live sim is the
 *  drive panel's physics). Unknown (an older server's catalogue): by its feet. */
export function liveCapable(info: LinkageInfo | undefined): boolean {
  if (!info) return false;
  return info.walks === undefined ? (info.feet ?? 0) === 0 : !info.walks;
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

/** The live session's query: the design's and the view on screen (``mode``: the robot or one
 *  side, whose glb the model must match). */
export function liveQuery(mode: string, designQuery: string): string {
  const p = new URLSearchParams(designQuery);
  p.set('mode', mode);
  return p.toString();
}

/** The session's joint targets (degrees by name): each slider whose joint the session takes
 *  (``hello.joints``), clipped to the model's range. */
export function liveTargets(joints: readonly Joint[], hello: Pick<Hello, 'joints'>): Record<string, number> {
  const out: Record<string, number> = {};
  for (const j of joints) {
    const m = hello.joints?.[j.name];
    if (m) out[j.name] = Math.min(m.hi, Math.max(m.lo, j.value));
  }
  return out;
}

/** The crank command: ``speed`` (a fraction of the servo's no-load speed, in [-1, 1]) for each
 *  of the session's cranks; empty for a design with none. */
export function crankCommand(speed: number, hello: Pick<Hello, 'drives'>): number[] {
  const v = Math.max(-1, Math.min(1, Number.isFinite(speed) ? speed : 0));
  return (hello.drives ?? []).map(() => v);
}

/** One line of what a frame says, by what the session streams: each site (world mm), each
 *  crank's turns and torque, each joint's angle and torque. */
export function liveReadout(values: Record<string, number>, hello: Pick<Hello, 'drives' | 'joints' | 'sites'>): string {
  const v = (k: string): number => values[k] ?? 0;
  const parts: string[] = [];
  for (const s of hello.sites ?? []) {
    parts.push(`${s} (${['x', 'y', 'z'].map((a) => v(`${s}.${a}`).toFixed(0)).join(', ')}) mm`);
  }
  for (const d of hello.drives ?? []) {
    parts.push(`${d} ${(v(`${d}.angle`) / (2 * Math.PI)).toFixed(2)} rev ${v(`${d}.torque`).toFixed(3)} N·m`);
  }
  for (const q of Object.keys(hello.joints ?? {})) {
    parts.push(`${q} ${((v(`${q}.angle`) * 180) / Math.PI).toFixed(1)}° ${v(`${q}.torque`).toFixed(2)} N·m`);
  }
  return parts.join(' · ');
}

export interface ArmPanelDeps {
  container: HTMLElement;
  /** The loaded glb's root (its nodes) and the mode and query it was baked for. */
  current: () => { root: THREE.Object3D; mode: string; query: string; pause: () => void } | null;
  /** The glb on screen (the live session poses its nodes). */
  loaded: () => LoadedScene | null;
  /** Hand the clip back to the animation (live off): playable again, at its first frame. */
  restoreClip: () => void;
  invalidate: () => void;
  status: (text: string) => void;
  /** Go live as each design lands (``?live=1``). */
  autoLive?: boolean;
}

const READOUT_MS = 150;

/** The panel: built on each load (``onLoad``), hidden for a walker. */
export function createArmPanel(deps: ArmPanelDeps) {
  let linkages: LinkageInfo[] | null = null;
  let joints: Joint[] = [];
  let frame0 = new Map<string, THREE.Matrix4>();
  let inFlight = false;          // one pose request at a time; the latest sliders after it
  let again = false;
  const physics = new PhysicsLink();
  let live = false, wantLive = false, crankSpeed = 0, readoutAt = 0;
  let liveBox: HTMLInputElement | null = null;
  let crankRow: HTMLLabelElement | null = null;
  let readout: HTMLElement | null = null;

  async function info(key: string): Promise<LinkageInfo | undefined> {
    if (!linkages) {
      const res = await fetch('/api/linkages');
      linkages = res.ok ? ((await res.json()) as { linkages: LinkageInfo[] }).linkages : [];
    }
    return linkages.find((l) => l.key === key);
  }

  async function apply(): Promise<void> {
    if (live) { send(); return; }
    if (inFlight) { again = true; return; }
    const cur = deps.current();
    if (!cur || joints.length === 0) return;
    inFlight = true;
    try {
      const res = await fetch(`/api/pose/${encodeURIComponent(cur.mode)}?${poseQuery(cur.query, joints)}`);
      if (!res.ok) { deps.status(`pose: ${res.status}`); return; }
      const poses = (await res.json()) as Record<string, number[]>;
      if (deps.current()?.root !== cur.root || live) return;      // another glb since, or live
      cur.pause();
      cur.root.traverse((obj) => {
        const pose = poses[obj.name];
        const m0 = frame0.get(obj.name);
        if (!pose || !m0) return;
        posedMatrix(pose, m0).decompose(obj.position, obj.quaternion, obj.scale);
      });
      deps.invalidate();
    } catch (err) {
      deps.status(`pose: ${(err as Error).message}`);
    } finally {
      inFlight = false;
      if (again) { again = false; void apply(); }
    }
  }

  /** The sliders' commands to the live session: the joints' targets, the cranks' speed. */
  function send(): void {
    const h = physics.info;
    if (!live || !h) return;
    physics.joints(liveTargets(joints, h));
    const cmd = crankCommand(crankSpeed, h);
    if (cmd.length) physics.command(...cmd);
  }

  /** Live on: the session for the glb on screen (its mode and query), its nodes bound. */
  async function setLive(on: boolean): Promise<void> {
    wantLive = on;
    if (!on) {
      if (live) stopLive(true);
      else physics.close();               // a connect in flight: superseded (it rejects with null)
      return;
    }
    if (live) return;
    const cur = deps.current(), l = deps.loaded();
    if (!cur || !l) return;
    deps.status('live: connecting…');
    try {
      const hello = await physics.connect(liveQuery(cur.mode, cur.query), deps.status, l);
      if (deps.loaded() !== l || !wantLive) { physics.close(); return; }   // superseded meanwhile
      if (hello.walks !== false) {          // a walker's live sim is the drive panel's
        physics.close();
        throw new Error(`${hello.design.linkage} walks: drive it (drive mode, physics)`);
      }
      live = true;
      cur.pause();
      for (const j of joints) {             // the sliders take the model's ranges
        const m = hello.joints?.[j.name];
        const input = deps.container.querySelector<HTMLInputElement>(`input[data-joint="${j.name}"]`);
        if (!m || !input) continue;
        j.lo = Math.max(j.lo, m.lo); j.hi = Math.min(j.hi, m.hi);
        j.value = Math.min(j.hi, Math.max(j.lo, j.value));
        Object.assign(input, { min: String(j.lo), max: String(j.hi), value: String(j.value) });
      }
      if (crankRow) crankRow.hidden = !(hello.drives ?? []).length;
      physics.onLost = (reason, stale) => {
        stopLive(true);
        deps.status(`live: ${reason}${stale ? ': reconnecting…' : ' (live off)'}`);
        if (stale) void setLive(true);
      };
      physics.onError = (m) => deps.status(`live: ${m}`);
      send();
      const what = [(hello.drives ?? []).length ? 'turn the crank' : '',
        Object.keys(hello.joints ?? {}).length ? 'move the joints' : ''].filter(Boolean).join(', ');
      deps.status(`live: MuJoCo (${hello.design.linkage}${hello.robot ? ' robot' : ''})`
        + (what ? ` — ${what} with the sliders` : ''));
    } catch (e) {
      if (e === null) return;               // superseded (a newer connect, or live off): theirs
      live = false;
      wantLive = false;
      if (liveBox) liveBox.checked = false;
      if (e) deps.status(`live: ${(e as Error).message}`);
    }
    deps.invalidate();
  }

  /** Live off: the session closed, the nodes given back (``restore``: the clip and the
   *  sliders' pose again; not when a new glb replaces them). */
  function stopLive(restore: boolean): void {
    const was = live;
    live = false;
    physics.onLost = null;
    physics.unbind();
    physics.close();
    if (liveBox) liveBox.checked = false;
    if (crankRow) crankRow.hidden = true;
    if (readout) readout.textContent = '';
    if (was && restore) {
      deps.restoreClip();
      void apply();
    }
    deps.invalidate();
  }

  /** Each animation frame while live: the nodes posed from the session's frames, the readout. */
  function frame(): void {
    const l = deps.loaded();
    if (!live || !l || !physics.apply(l)) return;
    const h = physics.info, fr = physics.frame;
    if (readout && h && fr && performance.now() - readoutAt > READOUT_MS) {
      readoutAt = performance.now();
      readout.textContent = `t ${fr.t.toFixed(1)} s · ${liveReadout(fr.values, h)}`;
    }
  }

  /** Each node's first-frame matrix, from its position, rotation and scale: call it as the
   *  glb lands (the mixer at frame 0), before anything awaits. */
  function capture(root: THREE.Object3D): void {
    frame0 = new Map();
    root.traverse((obj) => {
      if (obj.name) frame0.set(obj.name, new THREE.Matrix4().compose(obj.position, obj.quaternion, obj.scale));
    });
  }

  function slider(label: string, attrs: Record<string, string>, onInput: (v: number) => void): HTMLLabelElement {
    const row = document.createElement('label');
    row.textContent = label;
    const input = document.createElement('input');
    input.type = 'range';
    for (const [k, v] of Object.entries(attrs)) input.setAttribute(k, v);
    input.addEventListener('input', () => onInput(Number(input.value)));
    row.append(input);
    return row;
  }

  async function onLoad(root: THREE.Object3D): Promise<void> {
    const relive = live || wantLive || !!deps.autoLive;
    if (live || physics.info) stopLive(false);    // the session was the previous glb's
    deps.container.replaceChildren();
    liveBox = null; crankRow = null; readout = null;
    const key = String(root.userData.linkage ?? '');
    const lk = await info(key);
    joints = jointsOf(lk, root.userData.tree as { joints?: TreeJoint[] } | undefined,
      (root.userData.inputs0 ?? {}) as Record<string, number>);
    const canLive = liveCapable(lk);
    deps.container.hidden = joints.length === 0 && !canLive;
    if (canLive) {
      const row = document.createElement('label');
      row.className = 'live';
      liveBox = document.createElement('input');
      liveBox.type = 'checkbox';
      liveBox.dataset.live = '1';
      liveBox.addEventListener('change', () => void setLive(liveBox!.checked));
      row.append(liveBox, document.createTextNode(' live (MuJoCo)'));
      row.title = 'run this design in MuJoCo on the server: its joints and its crank, live';
      deps.container.append(row);
      crankSpeed = 0;
      crankRow = slider('crank speed', { min: '-1', max: '1', step: '0.05', value: '0', 'data-crank': '1',
        title: "the crank's speed, a fraction of its servo's no-load speed" }, (v) => { crankSpeed = v; send(); });
      crankRow.hidden = true;               // until the hello says it has a crank
      deps.container.append(crankRow);
    }
    for (const j of joints) {
      deps.container.append(slider(j.label, { min: String(j.lo), max: String(j.hi), step: '0.1',
        value: String(j.value), 'data-joint': j.name, title: `${j.name}: ${j.lo}..${j.hi} deg` },
      (v) => { j.value = v; void apply(); }));
    }
    if (canLive) {
      readout = document.createElement('div');
      readout.className = 'readout';
      deps.container.append(readout);
      if (relive) {
        liveBox!.checked = true;
        void setLive(true);
      }
    }
  }

  return {
    capture, onLoad, frame, setLive, physics,
    get joints() { return joints; },
    /** The session is up and posing the parts (the render loop runs ``frame`` then). */
    get live() { return live; },
  };
}

export type ArmPanel = ReturnType<typeof createArmPanel>;
