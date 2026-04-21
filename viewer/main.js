import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const DATA_URL = './data/frames.json';
const PARTS_URL = './data/parts/';

const canvas = document.getElementById('stage');
const statusEl = document.getElementById('status');
const slider = document.getElementById('slider');
const playBtn = document.getElementById('play');
const readout = document.getElementById('readout');

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight, false);
renderer.setClearColor(0x101418);

const scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(
  35, window.innerWidth / window.innerHeight, 1, 5000
);
camera.up.set(0, 0, 1);
camera.position.set(180, -260, 220);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.target.set(100, -30, 0);

// Lights.
scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const dir = new THREE.DirectionalLight(0xffffff, 0.85);
dir.position.set(1, -1, 2);
scene.add(dir);

// Ground grid + axes for scale.
const grid = new THREE.GridHelper(400, 40, 0x404040, 0x2a333d);
grid.rotation.x = Math.PI / 2;  // flip from XZ to XY
scene.add(grid);

const axes = new THREE.AxesHelper(40);
scene.add(axes);

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight, false);
});

// State ---------------------------------------------------------------------
let frames = [];          // per-frame pose dict
let bodyNames = [];       // render order
let meshes = new Map();   // name -> THREE.Mesh
let footLine = null;
let playing = false;
let playStartedAt = 0;    // timestamp when playback (re)started
let playStartedFrame = 0; // slider value when playback (re)started
let fps = 30;

// Load pipeline -------------------------------------------------------------
async function loadData() {
  const resp = await fetch(DATA_URL);
  if (!resp.ok) throw new Error(`frames.json: ${resp.status}`);
  return await resp.json();
}

function loadStl(name) {
  return new Promise((resolve, reject) => {
    new STLLoader().load(
      PARTS_URL + name + '.stl',
      (geom) => resolve(geom),
      undefined,
      (err) => reject(err),
    );
  });
}

function buildFootPath(pathXY) {
  const pts = pathXY.map(([x, y]) => new THREE.Vector3(x, y, 0.2));
  pts.push(pts[0].clone());  // close the loop
  const geom = new THREE.BufferGeometry().setFromPoints(pts);
  const mat = new THREE.LineBasicMaterial({ color: 0xff4040 });
  return new THREE.Line(geom, mat);
}

async function init() {
  statusEl.textContent = 'loading data…';
  const data = await loadData();
  frames = data.frames;
  bodyNames = data.body_names;
  fps = data.fps;
  slider.max = String(data.n_frames - 1);

  statusEl.textContent = 'loading meshes…';
  const geoms = await Promise.all(bodyNames.map(loadStl));
  bodyNames.forEach((name, i) => {
    const color = data.body_colors[name] ?? '#888888';
    const mat = new THREE.MeshStandardMaterial({
      color, metalness: 0.15, roughness: 0.75, flatShading: true,
    });
    const mesh = new THREE.Mesh(geoms[i], mat);
    mesh.matrixAutoUpdate = false;
    scene.add(mesh);
    meshes.set(name, mesh);
  });

  footLine = buildFootPath(data.foot_path);
  scene.add(footLine);

  setFrame(0);
  statusEl.textContent = `${bodyNames.length} bodies · ${data.n_frames} frames · ${fps} fps`;
  requestAnimationFrame(tick);
}

function setFrame(i) {
  const f = frames[i];
  if (!f) return;
  for (const name of bodyNames) {
    const m = meshes.get(name);
    const arr = f.poses[name];
    if (!m || !arr) continue;
    m.matrix.fromArray(arr);
    m.matrixWorldNeedsUpdate = true;
  }
  slider.value = String(i);
  readout.textContent = `frame ${i} / phase ${f.phase.toFixed(3)}`;
}

// Slider + play controls ----------------------------------------------------
slider.addEventListener('input', () => {
  playing = false;
  playBtn.textContent = '▶';
  setFrame(Number(slider.value));
});

playBtn.addEventListener('click', () => {
  playing = !playing;
  playBtn.textContent = playing ? '⏸' : '▶';
  if (playing) {
    playStartedAt = performance.now();
    playStartedFrame = Number(slider.value);
  }
});

// Main RAF loop -------------------------------------------------------------
function tick(now) {
  if (playing && frames.length > 0) {
    const elapsed = (now - playStartedAt) / 1000;
    const i = Math.floor(playStartedFrame + elapsed * fps) % frames.length;
    setFrame(i);
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}

init().catch((err) => {
  console.error(err);
  statusEl.textContent = 'error: ' + err.message;
});
