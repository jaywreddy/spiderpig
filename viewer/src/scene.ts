import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { RoomEnvironment } from './room-environment';
import type { Insets } from './layout';
import type { View } from './types';

export interface Stage {
  canvas: HTMLCanvasElement;
  renderer: THREE.WebGLRenderer;
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
  grid: THREE.GridHelper;
  /** The chrome over the canvas (``layout.chromeInsets``): the camera centres on the band between. */
  insets: Insets;
  setInsets(insets: Insets): void;
}

/** The vertical field of view: 35° on a landscape screen; on a portrait one wider, so the
 * horizontal field doesn't shrink to a slit (a phone's 0.46 aspect at 35° sees 16° across). */
const FOV = 35, FOV_MAX = 65;
function fovFor(aspect: number): number {
  if (aspect >= 1) return FOV;
  const t = Math.tan(THREE.MathUtils.degToRad(FOV) / 2) / aspect;
  return Math.min(FOV_MAX, THREE.MathUtils.radToDeg(2 * Math.atan(t)));
}

/** The share of the viewport's height the chrome leaves the model. */
function visibleFraction(stage: Stage): number {
  const { top, bottom } = stage.insets;
  return Math.max(0.3, 1 - (top + bottom) / window.innerHeight);
}

/** Shift the projection so the orbit target sits mid-way between the top bar and the dock
 * (rendering the full frame's rows from ``dy`` down moves its centre up by ``dy``). */
function applyInsets(stage: Stage): void {
  const w = window.innerWidth, h = window.innerHeight, { top, bottom } = stage.insets;
  stage.camera.setViewOffset(w, h, 0, (bottom - top) / 2, w, h);
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
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
  pmrem.dispose();

  const aspect = window.innerWidth / window.innerHeight;
  const camera = new THREE.PerspectiveCamera(fovFor(aspect), aspect, 1, 10000);
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

  const stage: Stage = {
    canvas, renderer, scene, camera, controls, grid,
    insets: { top: 0, bottom: 0 },
    setInsets(insets) {
      stage.insets = insets;
      applyInsets(stage);
    },
  };
  window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.fov = fovFor(camera.aspect);
    applyInsets(stage);   // updates the projection
    renderer.setSize(window.innerWidth, window.innerHeight, false);
  });
  return stage;
}

/** How much further than a desktop window's the drive camera starts from the robot, so a
 * phone (portrait: narrow; landscape: the dock takes much of the height) sees it whole. */
export function followScale(stage: Stage): number {
  const t = Math.tan(THREE.MathUtils.degToRad(stage.camera.fov) / 2);
  return Math.max(1, 0.32 / (t * stage.camera.aspect), 0.25 / (t * visibleFraction(stage)));
}

/**
 * Point the camera along a preset ``view`` at ``object``'s bounding box, close
 * enough that the box's corners just fit the viewport left between the chrome
 * (``stage.insets``; with ``margin``).
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
  const tan = Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2);
  const tanV = tan * visibleFraction(stage);   // fit the band between the chrome
  const tanH = tan * camera.aspect;
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
