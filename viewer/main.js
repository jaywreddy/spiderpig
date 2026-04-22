import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const GLB_URL = './api/klann.glb';

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
camera.position.set(260, -360, 320);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.target.set(100, -30, 30);

// Lights.
scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const dir = new THREE.DirectionalLight(0xffffff, 0.85);
dir.position.set(1, -1, 2);
scene.add(dir);

// Ground grid + axes for scale.
const grid = new THREE.GridHelper(400, 40, 0x404040, 0x2a333d);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
scene.add(new THREE.AxesHelper(40));

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight, false);
});

// State ---------------------------------------------------------------------
let mixer = null;
let action = null;
let clipDuration = 1;
let playing = false;
let footLine = null;
const clock = new THREE.Clock();

function buildFootPath(pathXY) {
  if (!pathXY || pathXY.length === 0) return null;
  const pts = pathXY.map(([x, y]) => new THREE.Vector3(x, y, 0.2));
  pts.push(pts[0].clone());
  const geom = new THREE.BufferGeometry().setFromPoints(pts);
  const mat = new THREE.LineBasicMaterial({ color: 0xff4040 });
  return new THREE.Line(geom, mat);
}

// Live-reload over /ws ------------------------------------------------------
function connectLiveReload() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const url = `${proto}://${location.host}/ws`;
  let socket;
  let retry = 0;

  const open = () => {
    socket = new WebSocket(url);
    socket.addEventListener('open', () => { retry = 0; });
    socket.addEventListener('message', (ev) => {
      let msg;
      try { msg = JSON.parse(ev.data); } catch { return; }
      if (msg.type === 'rebaking') {
        statusEl.textContent = `rebaking (${(msg.files || []).join(', ')})…`;
      } else if (msg.type === 'reload') {
        location.reload();
      } else if (msg.type === 'error') {
        statusEl.textContent = `bake error: ${msg.msg}`;
      }
    });
    socket.addEventListener('close', () => {
      retry = Math.min(retry + 1, 10);
      setTimeout(open, Math.min(500 * retry, 5000));
    });
  };
  open();
}

function formatTime(t) {
  return `t ${t.toFixed(3)}s / ${clipDuration.toFixed(3)}s`;
}

async function init() {
  statusEl.textContent = 'loading klann.glb…';
  const gltf = await new GLTFLoader().loadAsync(GLB_URL);
  scene.add(gltf.scene);

  // Force flat shading on every imported mesh to match prior viewer look.
  gltf.scene.traverse((obj) => {
    if (obj.isMesh && obj.material) {
      obj.material.flatShading = true;
      obj.material.needsUpdate = true;
    }
  });

  // Foot-path overlay comes from scene.extras.foot_path (stashed by the baker).
  const sceneData =
    gltf.parser.json.scenes?.[gltf.parser.json.scene ?? 0]?.extras ?? {};
  if (Array.isArray(sceneData.foot_path)) {
    footLine = buildFootPath(sceneData.foot_path);
    if (footLine) scene.add(footLine);
  }

  if (gltf.animations.length === 0) {
    throw new Error('klann.glb has no animations');
  }
  const clip = gltf.animations[0];
  clipDuration = clip.duration;
  mixer = new THREE.AnimationMixer(gltf.scene);
  action = mixer.clipAction(clip);
  action.play();
  action.paused = true;  // start paused; slider shows frame 0.

  slider.min = '0';
  slider.max = String(clipDuration);
  slider.step = String(clipDuration / 1000);
  slider.value = '0';

  const nNodes = gltf.scene.children.reduce((n, o) => n + (o.isMesh ? 1 : 0), 0);
  statusEl.textContent =
    `${gltf.nodes?.length ?? nNodes} nodes · ${clip.tracks.length} tracks · ${clipDuration.toFixed(2)}s loop`;
  readout.textContent = formatTime(0);

  clock.start();
  requestAnimationFrame(tick);

  // Test hook — expose animation state for e2e tests / debugging.
  window.__viewer = {
    get mixer() { return mixer; },
    get action() { return action; },
    get clipDuration() { return clipDuration; },
    get playing() { return playing; },
    step(dt) { mixer.update(dt); },
    ready: true,
  };
}

// Slider + play controls ----------------------------------------------------
slider.addEventListener('input', () => {
  if (!action) return;
  const t = Number(slider.value);
  playing = false;
  action.paused = true;
  playBtn.textContent = '▶';
  action.time = t;
  mixer.update(0);
  readout.textContent = formatTime(t);
});

playBtn.addEventListener('click', () => {
  if (!action) return;
  playing = !playing;
  playBtn.textContent = playing ? '⏸' : '▶';
  action.paused = !playing;
});

// Main RAF loop -------------------------------------------------------------
function tick() {
  if (mixer) {
    mixer.update(clock.getDelta());
    if (action && playing) {
      const t = action.time % clipDuration;
      slider.value = String(t);
      readout.textContent = formatTime(t);
    }
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}

init().catch((err) => {
  console.error(err);
  statusEl.textContent = 'error: ' + err.message;
});

connectLiveReload();
