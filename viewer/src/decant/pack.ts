/**
 * The cradle's flat pack (``/api/cradle/{key}/pack``, ``spiderpig.cradle_pack``): each
 * transport module (the base plate, the two side panels, the cradle box…) flies from the
 * assembled cradle into its place in the box, one after another, and the box fades in round
 * them. The transforms are in the baked model's own frame (the ``walker`` node's: x along
 * the linkage, y up, z across the bay), from each part's pose at the clip's first frame.
 */
import * as THREE from 'three';

export interface PackModule {
  key: string;
  label: string;
  how: string;
  parts: string[];
  matrix: number[][];       // 4 x 4, row-major
}

export interface PackData {
  modules: PackModule[];
  box: { inside_mm: number[]; outside_mm: number[]; floor: number[]; volume_l: number };
  steps: string[];
}

interface Track { node: THREE.Object3D; from: Mover; to: Mover; module: number }
interface Mover { p: THREE.Vector3; q: THREE.Quaternion; s: THREE.Vector3 }

export interface Pack {
  /** Pose every module at ``s`` (0 assembled, 1 packed), staggered one after another. */
  set(s: number): void;
  /** The box and the packed modules, for the camera. */
  bounds(): THREE.Box3;
  dispose(): void;
}

const WALL = 5;

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

/** Build the pack for the loaded model: ``walker`` its baked root (posed at the clip's first
 * frame when this is called), ``hide`` the display-only nodes (the bottle, the decanter). */
export function createPack(walker: THREE.Object3D, data: PackData, hide: string[]): Pack {
  const tracks: Track[] = [];
  data.modules.forEach((m, i) => {
    const r = m.matrix;
    const pack = new THREE.Matrix4().set(
      r[0]![0]!, r[0]![1]!, r[0]![2]!, r[0]![3]!,
      r[1]![0]!, r[1]![1]!, r[1]![2]!, r[1]![3]!,
      r[2]![0]!, r[2]![1]!, r[2]![2]!, r[2]![3]!,
      0, 0, 0, 1);
    for (const name of m.parts) {
      const node = walker.getObjectByName(name);
      if (!node) continue;
      node.updateMatrix();
      const at0 = node.matrix.clone();
      tracks.push({ node, from: mover(at0), to: mover(pack.clone().multiply(at0)), module: i });
    }
  });
  const hidden = hide.map((n) => walker.getObjectByName(n)).filter((o): o is THREE.Object3D => !!o);

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
    dispose(): void {
      walker.remove(box);
      geo.dispose();
      (walls.material as THREE.Material).dispose();
      (edges.material as THREE.Material).dispose();
      for (const h of hidden) h.visible = true;
    },
  };
}
