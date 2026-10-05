/**
 * Driving: operator input (keyboard, gamepad, the on-screen pad) -> per-side crank rates, and
 * the robot's pose on the ground integrated from the walking model.
 */
import { evaluate, type DriveData, type Side, type WalkState } from './model';

export type Scheme = 'tank' | 'arcade';
const TAU = 2 * Math.PI;

export type PadDir = 'up' | 'down' | 'left' | 'right';

/** Held keys -> a command per track in [-1, 1] (left / right as seen walking forward). */
export class Input {
  readonly held = new Set<string>();
  /** The on-screen arrow pad's held buttons (``bindPad``). */
  readonly pad = new Set<PadDir>();
  /** Called when the held keys change (a key down, up, or the window losing focus): the
   * physics drive sends its command then, not only from the animation loop. */
  onChange: (() => void) | null = null;
  constructor(private readonly enabled: () => boolean) {
    addEventListener('keydown', (e) => {
      const t = e.target as HTMLInputElement;   // typing in a text / number field isn't driving
      if (!this.enabled() || !/^(Key[WASD]|Arrow)/.test(e.code)) return;
      if (t.tagName === 'INPUT' && /text|number/.test(t.type)) return;
      if (!this.held.has(e.code)) { this.held.add(e.code); this.onChange?.(); }
      e.preventDefault();
    });
    addEventListener('keyup', (e) => { if (this.held.delete(e.code)) this.onChange?.(); });
    addEventListener('blur', () => {
      if (this.held.size || this.pad.size) { this.held.clear(); this.clearPad(); this.onChange?.(); }
    });
  }

  /** The on-screen pad: each ``[data-dir]`` button of ``el`` is held while a pointer (a finger, the
   * mouse) is down on it; several at once with several fingers. ``onPress`` runs on every press. */
  bindPad(el: HTMLElement, onPress: () => void): void {
    el.querySelectorAll<HTMLElement>('[data-dir]').forEach((b) => {
      const dir = b.dataset.dir as PadDir;
      const pointers = new Set<number>();        // the pointers holding this button
      this.padPointers.set(b, pointers);
      const up = (e: PointerEvent): void => {
        pointers.delete(e.pointerId);
        if (pointers.size) return;              // another finger still holds it
        b.classList.remove('held');
        if (this.pad.delete(dir)) this.onChange?.();
      };
      b.addEventListener('pointerdown', (e) => {
        e.preventDefault();               // no focus, no text selection, no emulated mouse events
        b.setPointerCapture(e.pointerId);
        pointers.add(e.pointerId);
        b.classList.add('held');
        onPress();
        if (!this.pad.has(dir)) { this.pad.add(dir); this.onChange?.(); }
      });
      for (const t of ['pointerup', 'pointercancel', 'lostpointercapture'] as const) b.addEventListener(t, up);
      b.addEventListener('contextmenu', (e) => e.preventDefault());   // a long press isn't a menu
    });
  }

  private readonly padPointers = new Map<HTMLElement, Set<number>>();

  private clearPad(): void {
    this.pad.clear();
    this.padPointers.forEach((ids, b) => { ids.clear(); b.classList.remove('held'); });
  }

  /** The command: keys and gamepad in ``scheme``, plus the pad, which reads arcade-style in
   * either scheme (▲ ▼ throttle, ◀ ▶ turn: tank's arrows would drive one side only). */
  read(scheme: Scheme): [number, number] {
    const [l, r] = this.keys(scheme);
    if (!this.pad.size) return [l, r];
    const p = (d: PadDir): number => +this.pad.has(d);
    const [pl, pr] = arcade(p('up') - p('down'), p('right') - p('left'));
    return [clamp(l + pl), clamp(r + pr)];
  }

  private keys(scheme: Scheme): [number, number] {
    const k = (c: string): number => +this.held.has(c);
    const pad = navigator.getGamepads?.().find((p) => p?.connected);
    const ax = (i: number): number => { const v = pad?.axes[i] ?? 0; return Math.abs(v) < 0.12 ? 0 : v; };
    if (scheme === 'tank') {
      return [k('KeyW') - k('KeyS') - ax(1), k('ArrowUp') - k('ArrowDown') - ax(3)].map(clamp) as [number, number];
    }
    const throttle = clamp(Math.max(k('KeyW'), k('ArrowUp')) - Math.max(k('KeyS'), k('ArrowDown')) - ax(1));
    const turn = clamp(Math.max(k('KeyD'), k('ArrowRight')) - Math.max(k('KeyA'), k('ArrowLeft')) + ax(0));
    return arcade(throttle, turn);
  }
}

/** Throttle and turn in [-1, 1] -> (left, right) tracks, scaled down together so neither saturates. */
function arcade(throttle: number, turn: number): [number, number] {
  const m = Math.max(1, Math.abs(throttle + turn), Math.abs(throttle - turn));
  return [(throttle + turn) / m, (throttle - turn) / m];
}

const clamp = (v: number): number => Math.max(-1, Math.min(1, v));

export interface Sample {
  t: number; speed: number; yawRate: number; height: number; pitch: number; roll: number; slip: number; margin: number;
}

/** Crank angles and rates per side, and the body's pose on the ground (x, z, yaw about +y). */
export class Sim {
  data: DriveData | null = null;
  theta: Record<Side, number> = { L: 0, R: 0 };
  omega: Record<Side, number> = { L: 0, R: 0 };
  x = 0; z = 0; yaw = 0; time = 0;
  phaseOffset = 0;        // right side's crank ahead of the left (rad)
  speedScale = 1;         // fraction of the servo's max rate for a full command
  accel = 3;              // crank acceleration limit, in max rates per second
  forward = 1;            // +1 when the cranks' design direction walks toward mech +x
  state: WalkState | null = null;
  history: Sample[] = [];
  lastRev: Record<string, number> | null = null;
  private rev = this.newRev();

  get modelTheta(): Record<Side, number> { return { L: this.theta.L, R: this.theta.R + this.phaseOffset }; }
  get wMax(): number { return ((this.data?.rpmMax ?? 0) * TAU) / 60; }

  setData(data: DriveData | null, forward = this.forward): void {
    this.data = data;
    this.forward = forward;
    this.state = data && evaluate(data, this.modelTheta, this.omega);
  }

  reset(): void {
    Object.assign(this, { theta: { L: 0, R: 0 }, omega: { L: 0, R: 0 }, x: 0, z: 0, yaw: 0, time: 0 });
    Object.assign(this, { history: [], lastRev: null });
    this.rev = this.newRev();
    this.setData(this.data);
  }

  /** Advance ``dt`` s with track commands (robot's left, right) in [-1, 1]. */
  step(dt: number, left: number, right: number): void {
    const data = this.data;
    if (!data || !(dt > 0)) return;
    // The robot's left side is L when it walks toward +x (L is at -z), else R.
    const cmd: Record<Side, number> = this.forward > 0 ? { L: left, R: right } : { L: right, R: left };
    const a = this.accel * this.wMax * dt;
    for (const s of ['L', 'R'] as const) {
      const target = cmd[s] * this.speedScale * this.wMax;   // + = the design's crank direction
      this.omega[s] += Math.max(-a, Math.min(a, target - this.omega[s]));
    }
    // Sub-steps: at most 1 deg of crank per model evaluation.
    const sweep = Math.max(Math.abs(this.omega.L), Math.abs(this.omega.R)) * dt;
    const nSub = Math.min(200, Math.max(1, Math.ceil(sweep / (Math.PI / 180))));
    const h = dt / nSub;
    for (let k = 0; k < nSub; k++) {
      const s = evaluate(data, this.modelTheta, this.omega);
      const dx = (Math.cos(this.yaw) * s.vx + Math.sin(this.yaw) * s.vz) * h;
      const dz = (-Math.sin(this.yaw) * s.vx + Math.cos(this.yaw) * s.vz) * h;
      this.x += dx; this.z += dz; this.yaw += s.w * h;
      this.theta.L += this.omega.L * h;
      this.theta.R += this.omega.R * h;
      this.accumulate(s, ((Math.abs(this.omega.L) + Math.abs(this.omega.R)) / 2) * h, Math.hypot(dx, dz), h);
    }
    this.time += dt;
    const s = (this.state = evaluate(data, this.modelTheta, this.omega));
    this.history.push({
      t: this.time, speed: s.vx * this.forward, yawRate: (s.w * 180) / Math.PI, height: s.height,
      pitch: s.pitch, roll: s.roll, slip: s.slip, margin: s.margin,
    });
    while (this.history[0]!.t < this.time - 6) this.history.shift();
  }

  private newRev() {
    return { turned: 0, t: 0, dist: 0, x: this.x, z: this.z, yaw: this.yaw, hMin: Infinity, hMax: -Infinity,
      pMin: Infinity, pMax: -Infinity, rMin: Infinity, rMax: -Infinity, slip: 0, margin: Infinity, tip: 0 };
  }

  /** Per-revolution stats, over crank progress (the mean of both sides' turning). */
  private accumulate(s: WalkState, dTheta: number, dist: number, dt: number): void {
    if (dTheta <= 0) return;
    const r = this.rev;
    r.turned += dTheta; r.t += dt; r.dist += dist;
    r.hMin = Math.min(r.hMin, s.height); r.hMax = Math.max(r.hMax, s.height);
    r.pMin = Math.min(r.pMin, s.pitch); r.pMax = Math.max(r.pMax, s.pitch);
    r.rMin = Math.min(r.rMin, s.roll); r.rMax = Math.max(r.rMax, s.roll);
    r.slip += s.slip * dt; r.margin = Math.min(r.margin, s.margin);
    if (s.tipping || s.degenerate) r.tip += dTheta;
    if (r.turned < TAU) return;
    this.lastRev = {
      seconds: r.t, travelled: r.dist, advance: Math.hypot(this.x - r.x, this.z - r.z),
      turn: ((this.yaw - r.yaw) * 180) / Math.PI, bob: r.hMax - r.hMin, pitchMin: r.pMin, pitchMax: r.pMax,
      rollMin: r.rMin, rollMax: r.rMax, slipMm: r.slip, minMargin: r.margin, tipping: r.tip / r.turned,
    };
    this.rev = this.newRev();
  }
}
