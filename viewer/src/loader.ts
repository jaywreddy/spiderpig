import * as THREE from 'three';
import { GLTFLoader, type GLTF } from 'three/examples/jsm/loaders/GLTFLoader.js';
import type { Mode } from './types';

export interface LoadedScene {
  root: THREE.Group;
  mixer: THREE.AnimationMixer;
  action: THREE.AnimationAction;
  clipDuration: number;
  clip: THREE.AnimationClip;
  footLine: THREE.Line | null;
  nodeCount: number;
}

const loader = new GLTFLoader();

function buildFootPath(pathXY: ReadonlyArray<readonly [number, number]>): THREE.Line | null {
  if (!pathXY || pathXY.length === 0) return null;
  const pts = pathXY.map(([x, y]) => new THREE.Vector3(x, y, 0.2));
  const first = pts[0];
  if (first) pts.push(first.clone());
  const geom = new THREE.BufferGeometry().setFromPoints(pts);
  const mat = new THREE.LineBasicMaterial({ color: 0xff4040 });
  return new THREE.Line(geom, mat);
}

function disposeRoot(root: THREE.Object3D): void {
  root.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    if (mesh.isMesh) {
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
  disposeRoot(prev.root);
  if (prev.footLine) {
    scene.remove(prev.footLine);
    prev.footLine.geometry?.dispose();
    (prev.footLine.material as THREE.Material).dispose();
  }
}

export async function loadGlb(scene: THREE.Scene, mode: Mode): Promise<LoadedScene> {
  const gltf: GLTF = await loader.loadAsync(`/api/glb/${mode}`);
  const root = gltf.scene;
  scene.add(root);

  // Force flat shading to match prior viewer look.
  root.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    if (mesh.isMesh && mesh.material) {
      const mat = mesh.material as THREE.MeshStandardMaterial;
      mat.flatShading = true;
      mat.needsUpdate = true;
    }
  });

  // Foot-path overlay comes from scene.extras.foot_path stashed by the baker.
  const json = gltf.parser.json as {
    scene?: number;
    scenes?: Array<{ extras?: { foot_path?: ReadonlyArray<readonly [number, number]> } }>;
  };
  const sceneIdx = json.scene ?? 0;
  const extras = json.scenes?.[sceneIdx]?.extras ?? {};
  let footLine: THREE.Line | null = null;
  if (Array.isArray(extras.foot_path)) {
    footLine = buildFootPath(extras.foot_path);
    if (footLine) scene.add(footLine);
  }

  const clip = gltf.animations[0];
  if (!clip) throw new Error(`${mode}.glb has no animations`);
  const mixer = new THREE.AnimationMixer(root);
  const action = mixer.clipAction(clip);
  action.play();

  const nodeCount = (gltf.parser.json as { nodes?: unknown[] }).nodes?.length
    ?? root.children.filter((o) => (o as THREE.Mesh).isMesh).length;

  return {
    root,
    mixer,
    action,
    clipDuration: clip.duration,
    clip,
    footLine,
    nodeCount,
  };
}
