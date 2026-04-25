import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

export interface Stage {
  canvas: HTMLCanvasElement;
  renderer: THREE.WebGLRenderer;
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
}

export function createStage(canvas: HTMLCanvasElement): Stage {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(window.innerWidth, window.innerHeight, false);
  renderer.setClearColor(0x101418);

  const scene = new THREE.Scene();

  const camera = new THREE.PerspectiveCamera(
    35, window.innerWidth / window.innerHeight, 1, 5000,
  );
  camera.up.set(0, 0, 1);
  camera.position.set(260, -360, 320);

  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.target.set(100, -30, 30);

  scene.add(new THREE.AmbientLight(0xffffff, 0.55));
  const dir = new THREE.DirectionalLight(0xffffff, 0.85);
  dir.position.set(1, -1, 2);
  scene.add(dir);

  const grid = new THREE.GridHelper(400, 40, 0x404040, 0x2a333d);
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
