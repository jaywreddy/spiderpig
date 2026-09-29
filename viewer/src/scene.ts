import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js';
import type { View } from './types';

export interface Stage {
  canvas: HTMLCanvasElement;
  renderer: THREE.WebGLRenderer;
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
}

// World frame: Z up, ground plane z = 0. The baked ``walker`` root stands the
// robot on it: it walks along X and its layer stack runs along Y, with the
// outer face of the first side (and the foot-path overlay) towards +Y.
const VIEW_DIRS: Record<View, THREE.Vector3> = {
  'three-quarter': new THREE.Vector3(1.0, 1.3, 0.55),
  side: new THREE.Vector3(0, 1, 0.06),
  front: new THREE.Vector3(1, 0, 0.12),
  top: new THREE.Vector3(0, 0.25, 1),
};

export function createStage(canvas: HTMLCanvasElement): Stage {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(window.innerWidth, window.innerHeight, false);
  renderer.setClearColor(0x101418);

  const scene = new THREE.Scene();
  // Soft studio reflections: acrylic and metal read as such instead of flat.
  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(new RoomEnvironment(renderer), 0.04).texture;
  pmrem.dispose();

  const camera = new THREE.PerspectiveCamera(
    35, window.innerWidth / window.innerHeight, 1, 10000,
  );
  camera.up.set(0, 0, 1);
  // Default framing for the robot (~520 mm long, ~250 mm tall) until a
  // model loads and frameView() fits the camera to it.
  camera.position.set(700, 900, 560);

  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.target.set(0, 0, 120);

  scene.add(new THREE.HemisphereLight(0xdde6f0, 0x20262c, 0.5));
  const key = new THREE.DirectionalLight(0xffffff, 0.9);
  key.position.set(1, 1.2, 2);
  scene.add(key);
  const rim = new THREE.DirectionalLight(0xffffff, 0.3);
  rim.position.set(-1, -1, 0.6);
  scene.add(rim);

  const grid = new THREE.GridHelper(1200, 48, 0x404a55, 0x252d36);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);
  scene.add(new THREE.AxesHelper(40));

  window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight, false);
  });

  return { canvas, renderer, scene, camera, controls };
}

/**
 * Point the camera along a preset ``view`` at ``object``'s bounding box, close
 * enough that the box's corners just fit the viewport (with ``margin``).
 */
export function frameView(
  stage: Stage, object: THREE.Object3D, view: View, margin = 1.08,
): void {
  object.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(object);
  if (box.isEmpty()) return;
  const { camera, controls } = stage;
  const center = box.getCenter(new THREE.Vector3());
  const back = VIEW_DIRS[view].clone().normalize();          // center -> camera
  const right = new THREE.Vector3().crossVectors(camera.up, back).normalize();
  const up = new THREE.Vector3().crossVectors(back, right);
  const tanV = Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2);
  const tanH = tanV * camera.aspect;
  let dist = 0;
  for (let i = 0; i < 8; i++) {
    const c = new THREE.Vector3(
      i & 1 ? box.max.x : box.min.x,
      i & 2 ? box.max.y : box.min.y,
      i & 4 ? box.max.z : box.min.z,
    ).sub(center);
    // A corner at depth offset d toward the camera needs D - d >= |x| / tan.
    const d = c.dot(back);
    dist = Math.max(dist, Math.abs(c.dot(right)) / tanH + d, Math.abs(c.dot(up)) / tanV + d);
  }
  dist *= margin;
  controls.target.copy(center);
  camera.position.copy(center).addScaledVector(back, dist);
  camera.near = Math.max(0.5, dist / 100);
  camera.far = dist * 20;
  camera.updateProjectionMatrix();
  controls.update();
}
