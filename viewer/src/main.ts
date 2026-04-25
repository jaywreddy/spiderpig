import * as THREE from 'three';
import './style.css';
import { createStage } from './scene';
import { loadGlb, teardown, type LoadedScene } from './loader';
import { bindControls } from './controls';
import { connectLiveReload } from './live-reload';
import type { Mode, ViewerHandle } from './types';

const canvas = document.getElementById('stage') as HTMLCanvasElement;
const stage = createStage(canvas);
const clock = new THREE.Clock();

let loaded: LoadedScene | null = null;
let playing = false;
let currentMode: Mode = 'klann';

const ui = bindControls({
  onSeek(t) {
    if (!loaded) return;
    playing = false;
    loaded.action.paused = true;
    loaded.action.time = t;
    loaded.mixer.update(0);
    ui.setReadout(formatTime(t));
  },
  onTogglePlay(p) {
    playing = p;
    if (loaded) loaded.action.paused = !p;
  },
  onModeChange(m) {
    void loadMode(m).catch((err: unknown) => {
      console.error(err);
      ui.setStatus(`error: ${(err as Error).message}`);
    });
  },
});

function formatTime(t: number): string {
  const dur = loaded?.clipDuration ?? 1;
  return `t ${t.toFixed(3)}s / ${dur.toFixed(3)}s`;
}

async function loadMode(mode: Mode): Promise<void> {
  currentMode = mode;
  ui.setStatus(`loading ${mode}…`);
  ui.setModeDisabled(true);
  try {
    const next = await loadGlb(stage.scene, mode);
    teardown(stage.scene, loaded);
    loaded = next;
    next.action.paused = !playing;

    ui.setSliderRange(next.clipDuration);
    ui.setSliderValue(0);
    next.action.time = 0;
    next.mixer.update(0);

    ui.setStatus(
      `${mode} · ${next.nodeCount} nodes · ${next.clip.tracks.length} tracks · ` +
      `${next.clipDuration.toFixed(2)}s loop`,
    );
    ui.setReadout(formatTime(0));
  } finally {
    ui.setModeDisabled(false);
  }
}

function tick(): void {
  if (loaded) {
    loaded.mixer.update(clock.getDelta());
    if (playing) {
      const t = loaded.action.time % loaded.clipDuration;
      ui.setSliderValue(t);
      ui.setReadout(formatTime(t));
    }
  }
  stage.controls.update();
  stage.renderer.render(stage.scene, stage.camera);
  requestAnimationFrame(tick);
}

const viewerHandle: ViewerHandle = {
  get mixer() { return loaded?.mixer ?? null; },
  get action() { return loaded?.action ?? null; },
  get clipDuration() { return loaded?.clipDuration ?? 1; },
  get playing() { return playing; },
  get mode() { return currentMode; },
  step(dt) { loaded?.mixer.update(dt); },
  loadMode,
  ready: false,
};
window.__viewer = viewerHandle;

async function init(): Promise<void> {
  await loadMode(ui.modeValue());
  clock.start();
  requestAnimationFrame(tick);
  viewerHandle.ready = true;
}

init().catch((err: unknown) => {
  console.error(err);
  ui.setStatus(`error: ${(err as Error).message}`);
});

connectLiveReload({
  onReload: () => { void loadMode(currentMode); },
  onStatus: (s) => ui.setStatus(s),
});
