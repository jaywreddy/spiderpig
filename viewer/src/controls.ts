import type { Mode } from './types';

export interface ControlsApi {
  /** Called when the user drags the slider; returns the new t in seconds. */
  onSeek: (t: number) => void;
  /** Toggle play/pause; pass the new playing flag. */
  onTogglePlay: (playing: boolean) => void;
  /** Mode dropdown changed. */
  onModeChange: (mode: Mode) => void;
}

export interface ControlsHandles {
  setPlaying(playing: boolean): void;
  setSliderRange(duration: number): void;
  setSliderValue(t: number): void;
  setReadout(text: string): void;
  setStatus(text: string): void;
  /** Replace the dropdown's options (server order) and select ``selected``. */
  setModes(modes: readonly Mode[], selected: Mode): void;
  modeValue(): Mode;
  setModeValue(mode: Mode): void;
  setModeDisabled(disabled: boolean): void;
}

const PLAY = '▶';
const PAUSE = '⏸';

// Dropdown labels for the server's mode ids; unknown ids show as-is.
const MODE_LABELS: Record<string, string> = {
  robot: 'robot',
  klann: 'klann (one leg)',
  double: 'double',
  decker: 'decker',
  double_double: 'double double',
};

export function bindControls(api: ControlsApi): ControlsHandles {
  const slider = document.getElementById('slider') as HTMLInputElement;
  const playBtn = document.getElementById('play') as HTMLButtonElement;
  const modeEl = document.getElementById('mode') as HTMLSelectElement;
  const readout = document.getElementById('readout') as HTMLSpanElement;
  const statusEl = document.getElementById('status') as HTMLDivElement;

  let playing = false;

  slider.addEventListener('input', () => {
    playing = false;
    playBtn.textContent = PLAY;
    api.onTogglePlay(false);
    api.onSeek(Number(slider.value));
  });

  playBtn.addEventListener('click', () => {
    playing = !playing;
    playBtn.textContent = playing ? PAUSE : PLAY;
    api.onTogglePlay(playing);
  });

  modeEl.addEventListener('change', () => {
    api.onModeChange(modeEl.value);
  });

  return {
    setPlaying(p) {
      playing = p;
      playBtn.textContent = p ? PAUSE : PLAY;
    },
    setSliderRange(duration) {
      slider.min = '0';
      slider.max = String(duration);
      slider.step = String(duration / 1000);
    },
    setSliderValue(t) {
      slider.value = String(t);
    },
    setReadout(text) { readout.textContent = text; },
    setStatus(text) { statusEl.textContent = text; },
    setModes(modes, selected) {
      modeEl.replaceChildren(...modes.map((m) => {
        const opt = document.createElement('option');
        opt.value = m;
        opt.textContent = MODE_LABELS[m] ?? m;
        return opt;
      }));
      modeEl.value = selected;
    },
    modeValue() { return modeEl.value; },
    setModeValue(mode) { modeEl.value = mode; },
    setModeDisabled(disabled) { modeEl.disabled = disabled; },
  };
}
