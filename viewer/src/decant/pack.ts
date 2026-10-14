/**
 * The cradle's pack (``/api/cradle/{key}/pack``, ``spiderpig.cradle_pack``): each piece it
 * comes apart into (flat, A: the base plate, the side panels, the cradle links, the tie
 * rods, the saddles…; as modules, B: the base board, the passive side panel, the cradle box,
 * the drive side panel, the hand crank) flies from the assembled cradle into its place in the
 * box, one after another in the order they go in (bottom up), and the box fades in round
 * them. The transforms are in the baked model's own frame (the ``walker`` node's: x along
 * the linkage, y up, z across the bay), from each part's pose at the pack's pose (``pose``:
 * its clip fraction; the clip's first frame when it has none).
 */
import * as THREE from 'three';

export interface PackModule {
  key: string;
  label: string;
  how: string;
  parts: string[];
  matrix: number[][];       // 4 x 4, row-major
  part_matrix?: Record<string, number[][]>;   // a part's own (a hanging link swung in)
  rests?: 'direct' | 'padded' | 'none';       // on what it rests (``on``), on foam pads or not
  on?: string[];
  pads?: number[][];        // x, z, height (mm) of each foam pad under it
  down?: string;            // the face it lies on, and its turn from there (degrees)
  spin?: number;
}

export interface PackData {
  modules: PackModule[];    // in the order they go in the box, bottom up
  box: { inside_mm: number[]; outside_mm: number[]; floor: number[]; volume_l: number };
  steps: string[];          // how it comes apart, in order
  /** How it goes back together, in order (modules only). */
  reassembly?: string[];
  mode?: 'flat' | 'modules';
  /** The pose the transforms start from: the crank cranked to it (B: its arm along the
   * outer plate's rear leg, about 5½ turns from rest). */
  pose?: { crank_t: number; clip_fraction: number; tilt_deg: number; turns_from_rest: number | null };
  linkage?: string;
  plan?: string;            // the plan it was packed by (``cradle_pack.PACK_PLANS``)
  fill?: number;            // the parts' own volume over the box's inside
}

/** The clip fraction the pack's transforms start from (0, the clip's first frame, if none). */
export function packFraction(data: PackData): number {
  return data.pose?.clip_fraction ?? 0;
}

/** ``a @ b`` of two 4 x 4 row-major matrices. */
export function matmul(a: number[][], b: number[][]): number[][] {
  return a.map((row) => [0, 1, 2, 3].map((j) => row.reduce((s, v, k) => s + v * b[k]![j]!, 0)));
}

/** A module moved by ``delta`` (4 x 4, row-major, the cradle's frame: applied after its
 * matrix): its matrix and each of its parts' own (``part_matrix``). */
export function movedModule(m: PackModule, delta: number[][]): PackModule {
  const out: PackModule = { ...m, matrix: matmul(delta, m.matrix) };
  if (m.part_matrix) {
    out.part_matrix = Object.fromEntries(
      Object.entries(m.part_matrix).map(([k, v]) => [k, matmul(delta, v)]));
  }
  return out;
}

/** The edit that turns a piece ``yaw`` quarter turns about the vertical and ``tip`` quarter
 * turns about x, both through ``centre`` (mm), then shifts it by ``shift`` (mm), each
 * component rounded to ``snap`` mm: 4 x 4, row-major. Quarter turns keep a packed piece's
 * faces square to the box (``spiderpig pack-inspect --layout`` judges the result). */
export function snappedDelta(centre: readonly number[], shift: readonly number[], yaw = 0,
                             tip = 0, snap = 3): number[][] {
  const turn = (q: number): [number, number] => {
    const k = ((Math.round(q) % 4) + 4) % 4;
    return [[1, 0], [0, 1], [-1, 0], [0, -1]][k] as [number, number];   // cos, sin
  };
  const [cy, sy] = turn(yaw), [cx, sx] = turn(tip);
  const ry = [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]];
  const rx = [[1, 0, 0], [0, cx, -sx], [0, sx, cx]];
  const r = ry.map((row) => [0, 1, 2].map((j) => row.reduce((s, v, k) => s + v * rx[k]![j]!, 0)));
  const c = centre.slice(0, 3), d = shift.slice(0, 3).map((v) => Math.round(v / snap) * snap);
  const t = [0, 1, 2].map((i) => c[i]! + d[i]! - r[i]!.reduce((s, v, k) => s + v * c[k]!, 0));
  return [[...r[0]!, t[0]!], [...r[1]!, t[1]!], [...r[2]!, t[2]!], [0, 0, 0, 1]].map(
    (row) => row.map((v) => (Object.is(v, -0) ? 0 : v)));
}

interface Track { node: THREE.Object3D; from: Mover; to: Mover; module: number; at0: THREE.Matrix4 }
interface Mover { p: THREE.Vector3; q: THREE.Quaternion; s: THREE.Vector3 }

export interface Pack {
  /** Pose every module at ``s`` (0 assembled, 1 packed), staggered one after another. */
  set(s: number): void;
  /** The box and the packed modules, for the camera. */
  bounds(): THREE.Box3;
  /** The module ``node`` (a part's node or a mesh under it) belongs to, or -1. */
  moduleOf(node: THREE.Object3D): number;
  /** Module ``i`` packed, as it is now (edits included). */
  module(i: number): PackModule;
  /** Replace module ``i``'s packed transforms (an edit: ``movedModule``); posed packed. */
  setModule(i: number, m: PackModule): void;
  /** Module ``i``'s packed parts' box (the cradle's frame), for its edit's pivot. */
  moduleBounds(i: number): THREE.Box3;
  /** The pack as edited: the data with each module's transforms as they are now. */
  data(): PackData;
  dispose(): void;
}

const WALL = 5;

function matrix4(r: number[][]): THREE.Matrix4 {
  return new THREE.Matrix4().set(
    r[0]![0]!, r[0]![1]!, r[0]![2]!, r[0]![3]!,
    r[1]![0]!, r[1]![1]!, r[1]![2]!, r[1]![3]!,
    r[2]![0]!, r[2]![1]!, r[2]![2]!, r[2]![3]!,
    0, 0, 0, 1);
}

function mover(m: THREE.Matrix4): Mover {
  const p = new THREE.Vector3(), q = new THREE.Quaternion(), s = new THREE.Vector3();
  m.decompose(p, q, s);
  return { p, q, s };
}

/** One module's share of the overall progress ``s``: they go one after another, overlapping. */
export function stagger(s: number, i: number, n: number, overlap = 0.5): number {
  const span = 1 / (n - (n - 1) * overlap);
  const start = i * span * (1 - overlap);
  const u = Math.min(1, Math.max(0, (s - start) / span));
  return u * u * (3 - 2 * u);              // smoothstep
}

/** Build the pack for the loaded model: ``walker`` its baked root (posed at the pack's pose,
 * ``packFraction``, when this is called), ``hide`` the display-only nodes (the bottle, the
 * decanter). */
export function createPack(walker: THREE.Object3D, data: PackData, hide: string[]): Pack {
  const tracks: Track[] = [];
  const mods = data.modules.map((m) => ({ ...m }));
  const target = (m: PackModule, name: string, at0: THREE.Matrix4): Mover => {
    const own = m.part_matrix?.[name];
    return mover((own ? matrix4(own) : matrix4(m.matrix)).multiply(at0));
  };
  mods.forEach((m, i) => {
    for (const name of m.parts) {
      // (GLTFLoader sanitizes node names: "L.frame_outer" is "Lframe_outer")
      const node = walker.getObjectByName(THREE.PropertyBinding.sanitizeNodeName(name));
      if (!node) continue;
      node.updateMatrix();
      const at0 = node.matrix.clone();
      tracks.push({ node, from: mover(at0), to: target(m, name, at0), module: i, at0 });
    }
  });
  const nodeModule = new Map(tracks.map((t) => [t.node, t.module]));
  const nodeName = new Map(mods.flatMap((m) => m.parts.map(
    (n) => [THREE.PropertyBinding.sanitizeNodeName(n), n] as const)));
  const hidden = hide.map((n) => walker.getObjectByName(THREE.PropertyBinding.sanitizeNodeName(n)))
    .filter((o): o is THREE.Object3D => !!o);

  // the box: its inside from the floor corner, the corrugated wall round it
  const [ix, iy, iz] = data.box.inside_mm as [number, number, number];
  const [fx, fy, fz] = data.box.floor as [number, number, number];
  const geo = new THREE.BoxGeometry(ix + 2 * WALL, iy + WALL, iz + 2 * WALL);
  const box = new THREE.Group();
  box.name = 'flat_pack_box';
  const walls = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
    color: 0xc79a62, roughness: 0.9, metalness: 0, transparent: true, opacity: 0,
    depthWrite: false, side: THREE.DoubleSide,
  }));
  const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo),
    new THREE.LineBasicMaterial({ color: 0x8a6436, transparent: true, opacity: 0 }));
  box.add(walls, edges);
  box.position.set(fx + ix / 2, fy - WALL + (iy + WALL) / 2, fz + iz / 2);
  walker.add(box);

  const n = data.modules.length;
  (window as unknown as { __packDebug: () => object }).__packDebug = () => data.modules.map((m, i) => {
    const b = new THREE.Box3();
    walker.updateMatrixWorld(true);
    const inv = walker.matrixWorld.clone().invert();
    for (const t of tracks) if (t.module === i) {
      const bb = new THREE.Box3().setFromObject(t.node, true).applyMatrix4(inv);   // (precise: a turned mesh)
      b.union(bb);
    }
    return { key: m.key, min: b.min.toArray().map(Math.round), max: b.max.toArray().map(Math.round) };
  });
  const pos = new THREE.Vector3(), quat = new THREE.Quaternion(), scl = new THREE.Vector3();
  return {
    set(s: number): void {
      for (const t of tracks) {
        const u = stagger(s, t.module, n);
        pos.lerpVectors(t.from.p, t.to.p, u);
        quat.slerpQuaternions(t.from.q, t.to.q, u);
        scl.lerpVectors(t.from.s, t.to.s, u);
        t.node.matrix.compose(pos, quat, scl);
        t.node.matrix.decompose(t.node.position, t.node.quaternion, t.node.scale);
      }
      for (const h of hidden) h.visible = s < 0.02;
      const fade = Math.min(1, Math.max(0, (s - 0.6) / 0.4));
      (walls.material as THREE.MeshStandardMaterial).opacity = 0.28 * fade;
      (edges.material as THREE.LineBasicMaterial).opacity = fade;
      box.visible = fade > 0;
    },
    bounds(): THREE.Box3 {
      walker.updateMatrixWorld(true);
      return new THREE.Box3().setFromObject(walls);
    },
    moduleOf(node: THREE.Object3D): number {
      for (let o: THREE.Object3D | null = node; o; o = o.parent) {
        const i = nodeModule.get(o);
        if (i !== undefined) return i;
      }
      return -1;
    },
    module(i: number): PackModule {
      return mods[i]!;
    },
    setModule(i: number, m: PackModule): void {
      mods[i] = m;
      for (const t of tracks) {
        const name = nodeName.get(t.node.name);
        if (t.module !== i || name === undefined) continue;
        t.to = target(m, name, t.at0);
        t.node.matrix.compose(t.to.p, t.to.q, t.to.s);
        t.node.matrix.decompose(t.node.position, t.node.quaternion, t.node.scale);
      }
    },
    moduleBounds(i: number): THREE.Box3 {
      walker.updateMatrixWorld(true);
      const inv = walker.matrixWorld.clone().invert();
      const b = new THREE.Box3();
      for (const t of tracks) {
        if (t.module === i) b.union(new THREE.Box3().setFromObject(t.node, true).applyMatrix4(inv));
      }
      return b;
    },
    data(): PackData {
      return { ...data, modules: mods.map((m) => ({ ...m })) };
    },
    dispose(): void {
      walker.remove(box);
      geo.dispose();
      (walls.material as THREE.Material).dispose();
      (edges.material as THREE.Material).dispose();
      for (const h of hidden) h.visible = true;
    },
  };
}
