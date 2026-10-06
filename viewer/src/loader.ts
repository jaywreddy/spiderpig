import * as THREE from 'three';
import { GLTFLoader, type GLTF } from 'three/examples/jsm/loaders/GLTFLoader.js';
import type { Mode } from './types';

export interface LoadedScene {
  root: THREE.Group;
  /** The baked ``walker`` node: stands the model up; bodies are its children. */
  walker: THREE.Object3D;
  mixer: THREE.AnimationMixer;
  action: THREE.AnimationAction;
  clipDuration: number;
  clip: THREE.AnimationClip;
  footLine: THREE.Line | null;
  nodeCount: number;
}

interface SceneExtras {
  foot_path?: ReadonlyArray<readonly [number, number]>;
  foot_path_z?: number;
  output_path?: ReadonlyArray<readonly [number, number]>;   // a mechanism's, instead
  output_path_z?: number;
}

const loader = new GLTFLoader();

// How much of the studio environment each baked material reflects: glossy
// acrylic and metal pick up highlights, matte prints mostly don't.
const ENV_INTENSITY: Record<string, number> = {
  acrylic: 0.9,
  acrylic_frame: 0.6,
  metal: 1.0,
  servo: 0.5,
  electronics: 0.4,
};

/** Foot trail in model coordinates (the linkage plane is XY at ``z``). */
function buildFootPath(
  pathXY: ReadonlyArray<readonly [number, number]>, z: number,
): THREE.Line | null {
  if (!pathXY || pathXY.length === 0) return null;
  const pts = pathXY.map(([x, y]) => new THREE.Vector3(x, y, z));
  const first = pts[0];
  if (first) pts.push(first.clone());
  const geom = new THREE.BufferGeometry().setFromPoints(pts);
  const mat = new THREE.LineBasicMaterial({ color: 0xff4040 });
  const line = new THREE.Line(geom, mat);
  line.name = 'foot_path';
  return line;
}

/** The baked clip with its loop closed. The bake writes keys over [0, D) evenly (no key at D,
 *  the first one's repeat), so without a closing key the last interval never plays and the
 *  loop jumps from the last key to the first. The same closing key drive mode adds
 *  (``drive/view.ts`` ``sideActions``, which takes the baked clip as it is). */
function closedLoop(clip: THREE.AnimationClip): THREE.AnimationClip {
  const t = clip.tracks[0]?.times;
  if (!t || t.length < 2) return clip;
  const period = t[t.length - 1]! + (t[1]! - t[0]!);
  type Ctor = new (n: string, t: ArrayLike<number>, v: ArrayLike<number>) => THREE.KeyframeTrack;
  return new THREE.AnimationClip(clip.name, period, clip.tracks.map((tr) => {
    const k = tr.getValueSize(), v = new Float32Array(tr.values.length + k);
    v.set(tr.values); v.set(tr.values.subarray(0, k), tr.values.length);
    return new (tr.constructor as Ctor)(tr.name, [...tr.times, period], v);
  }));
}

function disposeRoot(root: THREE.Object3D): void {
  root.traverse((obj) => {
    const mesh = obj as THREE.Mesh | THREE.Line;
    if ((mesh as THREE.Mesh).isMesh || (mesh as THREE.Line).isLine) {
      mesh.geometry?.dispose();
      const mat = mesh.material;
      if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
      else mat?.dispose();
    }
  });
}

export function teardown(scene: THREE.Scene, prev: LoadedScene | null): void {
  if (!prev) return;
  prev.mixer.stopAllAction();
  prev.mixer.uncacheRoot(prev.mixer.getRoot());
  scene.remove(prev.root);
  disposeRoot(prev.root);  // the foot path lives under the walker node
}

/** Load ``/api/glb/<mode>?<query>`` (design parameters); rejects with the server's error detail. */
export async function loadGlb(scene: THREE.Scene, mode: Mode, query = ''): Promise<LoadedScene> {
  const res = await fetch(`/api/glb/${encodeURIComponent(mode)}${query ? `?${query}` : ''}`);
  if (!res.ok) {
    const detail = ((await res.json().catch(() => ({}))) as { detail?: unknown }).detail;
    throw new Error(typeof detail === 'string' ? detail : `${res.status} ${JSON.stringify(detail ?? res.statusText)}`);
  }
  const gltf: GLTF = await loader.parseAsync(await res.arrayBuffer(), '');
  const root = gltf.scene;

  // Flat shading reads plate edges crisply; translucent acrylic keeps writing
  // depth off (GLTFLoader's BLEND default) so the parts behind show through.
  // Materials are named by the baker after the fabrication kind.
  root.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    if (mesh.isMesh && mesh.material) {
      const mat = mesh.material as THREE.MeshStandardMaterial;
      mat.flatShading = true;
      mat.envMapIntensity = ENV_INTENSITY[mat.name] ?? 0.35;
      mat.needsUpdate = true;
    }
  });

  const walker = root.getObjectByName('walker') ?? root;

  // Foot-path overlay comes from scene.extras stashed by the baker; it is in
  // model coordinates, so it hangs off the walker node like the bodies do.
  const json = gltf.parser.json as { scene?: number; scenes?: Array<{ extras?: SceneExtras }> };
  const extras = json.scenes?.[json.scene ?? 0]?.extras ?? {};
  let footLine: THREE.Line | null = null;
  const path = extras.foot_path ?? extras.output_path;
  if (Array.isArray(path)) {
    footLine = buildFootPath(path, extras.foot_path_z ?? extras.output_path_z ?? 0.2);
    if (footLine) walker.add(footLine);
  }

  const clip = gltf.animations[0];
  if (!clip) throw new Error(`${mode}.glb has no animations`);
  const loop = closedLoop(clip);
  const mixer = new THREE.AnimationMixer(root);
  const action = mixer.clipAction(loop);
  action.play();

  const nodeCount = (gltf.parser.json as { nodes?: unknown[] }).nodes?.length
    ?? root.children.filter((o) => (o as THREE.Mesh).isMesh).length;

  scene.add(root);
  return {
    root,
    walker,
    mixer,
    action,
    clipDuration: loop.duration,
    clip,
    footLine,
    nodeCount,
  };
}
