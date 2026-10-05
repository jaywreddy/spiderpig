import * as THREE from 'three';
import './style.css';
import { createStage, frameView } from './scene';
import { loadGlb, teardown, type LoadedScene } from './loader';
import { bindControls } from './controls';
import { connectLiveReload } from './live-reload';
import { createDrive } from './drive';
import { chromeInsets } from './layout';
import type { Mode, View, ViewerHandle } from './types';

const canvas = document.getElementById('stage') as HTMLCanvasElement;
const stage = createStage(canvas);
const clock = new THREE.Clock();

// Deep links: ?mode=robot&view=side&t=0.3 (t in clip seconds; pauses there);
// &linkage=jansen and the tune panel's design parameters (drive/index.ts);
// &design=<id> a stored design (`spiderpig view`): its glb, its mode (robot or one
// side), and its linkage, module, phases and proportions in the tune panel.
const params = new URLSearchParams(location.search);
const VIEWS: readonly View[] = ['three-quarter', 'side', 'front', 'top'];
const paramView = params.get('view') as View | null;
let view: View = paramView && VIEWS.includes(paramView) ? paramView : 'three-quarter';

let loaded: LoadedScene | null = null;
let playing = false;
let currentMode: Mode = 'robot';  // last requested (live reload re-requests it)
let currentQuery = '';            // its design parameters ('' = default)
let loadedMode: Mode = '';        // on screen; set only once its GLB has swapped in

// Render on demand: while playing, while the camera moves, or after a change.
// A paused viewer then costs nothing (the env-mapped, translucent robot is
// heavy on software GL and laptop GPUs alike).
let dirty = true;
function invalidate(): void { dirty = true; }
stage.controls.addEventListener('change', invalidate);
window.addEventListener('resize', invalidate);

const ui = bindControls({
  onSeek(t) {
    seek(t);
  },
  onTogglePlay(p) {
    playing = p;
    if (loaded) loaded.action.paused = !p;
  },
  onModeChange(m) {
    void loadMode(m, drive.baseQuery(m)).catch((err: unknown) => {
      console.error(err);
      ui.setStatus(`error: ${(err as Error).message}`);
    });
  },
});

const drive = createDrive({
  stage,
  loaded: () => loaded,
  loadRobot: (query) => loadMode('robot', query),
  loadSide: (query) => loadMode('klann', query),   // 'klann': the server's side-only single mode (an old URL id, any linkage)
  status: (text) => ui.setStatus(text),
  seek,
  reframe: () => { if (loaded) frameView(stage, loaded.root, view); invalidate(); },
});

// The chrome over the canvas (the top bar, the dock): the camera frames the band between,
// and an open panel stops short of the dock (``--dock-h``).
function syncChrome(): void {
  const insets = chromeInsets();
  document.documentElement.style.setProperty('--dock-h', `${insets.bottom}px`);
  stage.setInsets(insets);
  invalidate();
}
syncChrome();
new ResizeObserver(syncChrome).observe(document.getElementById('dock')!);
window.addEventListener('resize', syncChrome);
// A phone turned over: frame the model for the new shape (once the resize has landed; not
// while driving, where the camera follows the robot).
matchMedia('(orientation: portrait)').addEventListener('change', () => requestAnimationFrame(() => {
  if (loaded && !drive.active) frameView(stage, loaded.root, view);
  invalidate();
}));

function formatTime(t: number): string {
  const dur = loaded?.clipDuration ?? 1;
  return `t ${t.toFixed(3)}s / ${dur.toFixed(3)}s`;
}

function seek(t: number): void {
  if (!loaded) return;
  playing = false;
  ui.setPlaying(false);
  loaded.action.paused = true;
  loaded.action.time = t;
  loaded.mixer.update(0);
  ui.setSliderValue(t);
  ui.setReadout(formatTime(t));
  invalidate();
}

async function loadMode(mode: Mode, query = ''): Promise<void> {
  currentMode = mode;
  currentQuery = query;
  ui.setModeValue(mode);
  ui.setStatus(`loading ${mode}…`);
  ui.setModeDisabled(true);
  try {
    const next = await loadGlb(stage.scene, mode, query);
    teardown(stage.scene, loaded);
    const reframe = mode !== loadedMode;  // a live reload keeps the user's camera
    loaded = next;
    loadedMode = mode;
    next.action.paused = !playing;

    ui.setSliderRange(next.clipDuration);
    ui.setSliderValue(0);
    next.action.time = 0;
    next.mixer.update(0);
    if (reframe) frameView(stage, next.root, view);

    ui.setStatus(
      `${mode} · ${String(next.root.userData.linkage ?? '')} · ${next.nodeCount} nodes · ` +
      `${next.clip.tracks.length} tracks · ${next.clipDuration.toFixed(2)}s loop`,
    );
    ui.setReadout(formatTime(0));
    await drive.onLoad(next, query);
    invalidate();
  } finally {
    ui.setModeDisabled(false);
  }
}

function tick(): void {
  const dt = clock.getDelta();
  if (drive.active) {
    drive.frame(dt);   // drive mode renders continuously
    dirty = true;
  } else if (loaded && playing) {
    loaded.mixer.update(dt);
    const t = loaded.action.time % loaded.clipDuration;
    ui.setSliderValue(t);
    ui.setReadout(formatTime(t));
    dirty = true;
  }
  if (stage.controls.update()) dirty = true;   // orbiting, or damping settling
  if (dirty) {
    dirty = false;
    stage.renderer.render(stage.scene, stage.camera);
  }
  requestAnimationFrame(tick);
}

const viewerHandle: ViewerHandle = {
  get mixer() { return loaded?.mixer ?? null; },
  get action() { return loaded?.action ?? null; },
  get clipDuration() { return loaded?.clipDuration ?? 1; },
  get playing() { return playing; },
  get mode() { return loadedMode; },
  get walker() { return loaded?.walker ?? null; },
  camera: stage.camera,
  step(dt) {
    loaded?.mixer.update(dt);
    invalidate();
  },
  seek,
  setView(v) {
    view = v;
    if (loaded) frameView(stage, loaded.root, v);
    invalidate();
  },
  loadMode,
  drive,
  ready: false,
};
window.__viewer = viewerHandle;

/** Server's mode catalogue (ids in order, their labels); falls back to the static
 * options in index.html. */
interface ModeCatalogue { modes: Mode[]; default: Mode; labels?: Record<string, string> }

async function fetchModes(): Promise<ModeCatalogue | null> {
  try {
    const res = await fetch('/api/modes');
    if (!res.ok) return null;
    return (await res.json()) as ModeCatalogue;
  } catch {
    return null;
  }
}

async function init(): Promise<void> {
  const catalogue = await fetchModes();
  // ?design=<id>: the stored design's card seeds the tune panel and picks the mode
  // (robot, or `side` for a one-sided design) unless ?mode= says otherwise.
  const designId = params.get('design');
  const card = designId
    ? await drive.loadDesign(designId).catch((err: unknown) => {
      ui.setStatus(`design ${designId}: ${(err as Error).message}`);
      return null;
    })
    : null;
  const requested = params.get('mode') ?? card?.mode ?? null;
  let initial = ui.modeValue();
  if (catalogue) {
    // The server's dropdown ids, plus the design's own mode when it isn't one of them
    // (`side`: one side of a one-sided design; an id for URLs, not offered otherwise).
    const modes = card && !catalogue.modes.includes(card.mode) ? [...catalogue.modes, card.mode]
      : catalogue.modes;
    const labels = card && !catalogue.modes.includes(card.mode)
      ? { ...catalogue.labels, [card.mode]: `${card.mode} (this design)` } : catalogue.labels;
    initial = requested && modes.includes(requested) ? requested : catalogue.default;
    ui.setModes(modes, initial, labels);
  }
  // ?linkage=... (the tune panel's). A design that can't be built still tunes (stick preview).
  await loadMode(initial, drive.baseQuery(initial))
    .catch((err: unknown) => ui.setStatus(`error: ${(err as Error).message}`));
  const t = Number(params.get('t'));
  if (params.has('t') && Number.isFinite(t)) seek(t);
  await drive.init().catch((err: unknown) => ui.setStatus(`drive: ${(err as Error).message}`));
  clock.start();
  requestAnimationFrame(tick);
  viewerHandle.ready = true;
}

init().catch((err: unknown) => {
  console.error(err);
  ui.setStatus(`error: ${(err as Error).message}`);
});

connectLiveReload({
  onReload: () => { void loadMode(currentMode, currentQuery); },
  onStatus: (s) => ui.setStatus(s),
});
