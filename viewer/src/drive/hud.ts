/**
 * The drive mode's HUD (``index.ts``): its rows, the warning line and the sparkline, read from
 * the walking model (``updateModelHud``) or from a short history of the physics drive's frames
 * (``PhysicsHud``).
 */
import type * as THREE from 'three';
import type GUI from 'three/examples/jsm/libs/lil-gui.module.min.js';
import type { Sample, Sim } from './sim';
import type { PhysicsFrame, PhysicsLink, Steering } from './physics';

export const PLOTS: (keyof Sample)[] = ['speed', 'yawRate', 'height', 'pitch', 'roll', 'slip', 'margin'];
export const HUD_ROWS = ['speed', 'yaw rate', 'height', 'pitch', 'roll', 'contacts', 'slip', 'margin', 'cranks', 'torque',
  'loads', 'steering', 'rev distance', 'rev turn', 'rev bob', 'rev pitch', 'rev roll', 'rev slip', 'rev margin',
  'model'];
/** Rows the physics drive has no source for (the walking model's support geometry). */
export const MODEL_ONLY_ROWS = ['slip', 'margin', 'rev slip', 'rev margin'];
/** Rows only the physics drive has (the walking model has no dynamics). */
export const PHYSICS_ONLY_ROWS = ['torque', 'loads', 'steering'];
/** A design whose quasi-static stability margin is under this (mm) is marginal: the walking model
 * warns (Jansen's quad: 4 mm predicted) but MuJoCo decides (the server's straight run, the hello's
 * ``forward``: Jansen's quad walks it at 17° of tilt). Physics refuses a design only when that run
 * fell, unless the URL says ``physics=force``. */
export const MIN_MARGIN_MM = 15;
export const HISTORY_S = 1.5;      // of physics frames behind the HUD's speed and yaw rate
/** One side on fewer than two feet more often than this (``sim.run.SUPPORT_LOW``) is weak support. */
export const SUPPORT_LOW = 0.10;
/** The sim running slower than this fraction of real time is flagged (a loaded server), and the flag
 * clears above ``SLOW_CLEAR`` (hysteresis: the rate jitters with the frame arrivals). */
const SLOW_RATE = 0.85, SLOW_CLEAR = 0.95;
/** Drive torque above the servo's rated torque for longer than this (s) is flagged. */
const OVER_RATED_S = 2;
/** The differential a steering excursion uses when the server proved a bounded side offset
 * (``steering.step_deg``) but no unbounded turn: the (1, 0.6) the step test ran. */
const STEP_TURN = 0.4;
/** The turn authority the hello grants: the proven-safe unbounded differential, else the excursion's
 * when the server's phase lock bounds one, else 0 (steering disabled for this design). */
export const safeTurn = (steer: Steering | null): number =>
  !steer ? 0 : steer.turn > 0 ? steer.turn : steer.step_deg > 0 ? STEP_TURN : 0;

export const fmt = (v: unknown): string => (typeof v === 'number' ? String(+v.toFixed(2))
  : Array.isArray(v) ? v.map(fmt).join(v.length === 2 ? ' … ' : ', ') : String(v));
export const f = (v: number | undefined, d = 1): string => (Number.isFinite(v) ? v!.toFixed(d) : '–');
export const deg = (rad: number): number => (rad * 180) / Math.PI;

/** The HUD folder of the drive panel: a row per ``HUD_ROWS``, the plot picker and the sparkline,
 * and the warning line over the page. */
export function createHudPanel(gui: GUI, opts: { plot: keyof Sample }) {
  const hud: Record<string, string> = Object.fromEntries(HUD_ROWS.map((k) => [k, '–']));
  const hudF = gui.addFolder('HUD');
  for (const k of HUD_ROWS) hudF.add(hud, k).disable().listen();
  hudF.add(opts, 'plot', PLOTS);
  const spark = Object.assign(document.createElement('canvas'), { width: 290, height: 56, id: 'drive-spark' });
  hudF.$children.append(spark);
  const warn = Object.assign(document.createElement('div'), { id: 'drive-warn', hidden: true });
  document.body.append(warn);
  return { hud, hudF, spark, warn };
}

/** The walking model's readings: the HUD's rows, the warning line and the sparkline. */
export function updateModelHud(sim: Sim, hud: Record<string, string>, warn: HTMLElement,
  spark: HTMLCanvasElement, plot: keyof Sample): void {
  const s = sim.state, h = sim.history[sim.history.length - 1], r = sim.lastRev;
  if (!s || !h) return;
  const rpm = (w: number): string => f((w * 60) / (2 * Math.PI));
  const dPhase = ((deg(sim.modelTheta.R - sim.modelTheta.L) % 360) + 540) % 360 - 180;
  Object.assign(hud, {
    speed: `${f(h.speed)} mm/s`, 'yaw rate': `${f(h.yawRate)} °/s`, height: `${f(h.height)} mm`,
    pitch: `${f(h.pitch, 2)}°`, roll: `${f(h.roll, 2)}°`, contacts: `${s.nContacts} / ${s.feet.length}`,
    slip: `${f(h.slip)} mm/s`, margin: `${f(h.margin)} mm`,
    cranks: `${rpm(sim.omega.L)} · ${rpm(sim.omega.R)} rpm, R−L ${f(dPhase, 0)}°`,
  });
  if (r) {
    Object.assign(hud, {
      'rev distance': `${f(r.advance)} mm (path ${f(r.travelled)}) in ${f(r.seconds, 2)} s`,
      'rev turn': `${f(r.turn)}°`, 'rev bob': `${f(r.bob)} mm`,
      'rev pitch': `${f(r.pitchMin)} … ${f(r.pitchMax)}°`, 'rev roll': `${f(r.rollMin)} … ${f(r.rollMax)}°`,
      'rev slip': `${f(r.slipMm)} mm`, 'rev margin': `≥ ${f(r.minMargin)} mm · tip ${f(r.tipping! * 100, 0)} %`,
    });
  }
  const bad = s.degenerate ? 'DEGENERATE SUPPORT' : s.tipping ? 'TIPPING' : s.margin < 0 ? 'COM OUTSIDE SUPPORT' : '';
  warn.hidden = !bad;
  warn.textContent = `⚠ ${bad} (margin ${f(s.margin)} mm)`;
  // One sparkline: the chosen metric over the last 6 s.
  const g = spark.getContext('2d')!, vals = sim.history.map((x) => x[plot]);
  let lo = Math.min(...vals), hi = Math.max(...vals);
  if (hi - lo < 1) { lo -= 0.5; hi += 0.5; }
  g.clearRect(0, 0, spark.width, spark.height);
  g.strokeStyle = '#6fb6ff';
  g.beginPath();
  sim.history.forEach((x, i) => g.lineTo(((x.t - h.t + 6) / 6) * spark.width,
    spark.height - 4 - ((vals[i]! - lo) / (hi - lo)) * (spark.height - 8)));
  g.stroke();
  g.fillStyle = '#cfd6de';
  g.fillText(`${plot} ${f(lo)} … ${f(hi)}`, 4, 11);
}

/** One physics frame reduced for the HUD. */
interface PhysSample { t: number; x: number; y: number; heading: number; height: number; pitch: number; roll: number }

/** The physics drive's HUD: ``HISTORY_S`` of frames, the per-revolution window and the
 * warnings (a fall, the body down, steering beyond what the server proved, a stall). */
export class PhysicsHud {
  private history: PhysSample[] = [];
  // The per-revolution window, closed by the cranks' mean |travel| (a spin turns them opposite ways).
  private rev: { travel: number; t: number; x: number; y: number; heading: number; hMin: number; hMax: number;
    pMin: number; pMax: number; rMin: number; rMax: number } | null = null;
  private lastRev: Record<string, number> | null = null;
  private travel = 0;
  private lastCrank: [number, number] | null = null;
  private epoch = 0;
  private slow = false;
  private loads = { az: 0, pin: 0 };
  private overRatedSince: number | null = null;

  constructor(private readonly physics: PhysicsLink) {}

  /** The sim clock went back (a reset, ours or a frame's): every history starts over. */
  newEpoch(): void {
    this.history.length = 0;
    this.rev = null;
    this.lastRev = null;
    this.travel = 0;
    this.lastCrank = null;
    this.loads = { az: 0, pin: 0 };
    this.overRatedSince = null;
  }

  /** A new session's frames: its epoch is the one to compare against. */
  startEpoch(): void {
    this.newEpoch();
    this.epoch = this.physics.epoch;
  }

  /** Keep ``HISTORY_S`` of frames and the per-revolution window (closed by the cranks' mean |travel|,
   * so a turn in place completes revolutions too). A frame whose clock went back (a reset landed on
   * the server) starts a new epoch: nothing before it is comparable. */
  record(fr: PhysicsFrame, heading: number, pitch: number, roll: number): void {
    const physics = this.physics;
    let last = this.history[this.history.length - 1];
    if (physics.epoch !== this.epoch || (last && fr.t < last.t)) { this.newEpoch(); this.epoch = physics.epoch; last = undefined; }
    if (last && fr.t <= last.t) return;
    const height = fr.pos.z;
    if (last) heading = last.heading + Math.atan2(Math.sin(heading - last.heading), Math.cos(heading - last.heading));
    this.history.push({ t: fr.t, x: fr.pos.x, y: fr.pos.y, heading, height, pitch, roll });
    while (this.history.length > 2 && this.history[0]!.t < fr.t - HISTORY_S) this.history.shift();
    this.loads = { az: Math.max(this.loads.az, Math.abs(fr.az)), pin: Math.max(this.loads.pin, fr.pinLoad) };
    if (this.lastCrank) {
      this.travel += (Math.abs(fr.crank[0] - this.lastCrank[0]) + Math.abs(fr.crank[1] - this.lastCrank[1])) / 2;
    }
    this.lastCrank = [fr.crank[0], fr.crank[1]];
    if (!this.rev) {
      this.rev = { travel: this.travel, t: fr.t, x: fr.pos.x, y: fr.pos.y, heading, hMin: height, hMax: height,
        pMin: pitch, pMax: pitch, rMin: roll, rMax: roll };
      return;
    }
    const r = this.rev;
    r.hMin = Math.min(r.hMin, height); r.hMax = Math.max(r.hMax, height);
    r.pMin = Math.min(r.pMin, pitch); r.pMax = Math.max(r.pMax, pitch);
    r.rMin = Math.min(r.rMin, roll); r.rMax = Math.max(r.rMax, roll);
    if (this.travel - r.travel >= 2 * Math.PI) {
      this.lastRev = { seconds: fr.t - r.t, advance: Math.hypot(fr.pos.x - r.x, fr.pos.y - r.y),
        turn: deg(heading - r.heading), bob: r.hMax - r.hMin, pitchMin: r.pMin, pitchMax: r.pMax,
        rollMin: r.rMin, rollMax: r.rMax, az: this.loads.az, pin: this.loads.pin };
      this.loads = { az: 0, pin: 0 };
      this.rev = null;
    }
  }

  /** The HUD's rows and the warning line from the frame, the history and the command sent
   * (``lastCommand``); ``model``: the walking model's speed, shown as information. */
  update(hud: Record<string, string>, warn: HTMLElement, fr: PhysicsFrame, up: THREE.Vector3, pitch: number,
    roll: number, turning: boolean, lastCommand: [number, number], model: number | undefined): void {
    const physics = this.physics;
    const fell = up.z <= Math.cos(Math.PI / 4);
    if (fell) this.history.length = 0;     // it isn't walking any more: speed and yaw rate read 0
    const a = this.history[0], b = this.history[this.history.length - 1];
    const span = a && b && b.t > a.t ? b.t - a.t : 0;
    const speed = span ? Math.hypot(b!.x - a!.x, b!.y - a!.y) / span : 0;
    const yawRate = span ? deg(b!.heading - a!.heading) / span : 0;
    const rpm = (w: number): string => f((w * 60) / (2 * Math.PI), 0);
    Object.assign(hud, {
      speed: `${speed.toFixed(1)} mm/s (${HISTORY_S} s mean)`, 'yaw rate': `${yawRate.toFixed(1)} °/s`,
      height: `${fr.pos.z.toFixed(1)} mm`, pitch: `${pitch.toFixed(2)}°`, roll: `${roll.toFixed(2)}°`,
      contacts: `${fr.feetDown} / ${physics.feet}${fr.bodyDown ? ' · BODY ON THE FLOOR' : ''}`,
      cranks: `${deg(fr.crank[0]).toFixed(0)}° · ${deg(fr.crank[1]).toFixed(0)}° turned, L−R ${f(deg(fr.sidePhase), 0)}°`
        + (this.lastRev ? ` · ${rpm((2 * Math.PI) / this.lastRev.seconds!)} rpm` : ''),
    });
    // The quasi-static model's speed is information, not a check: it assumes no slip on the
    // lowest feet, and MuJoCo's feet slip (the Klann quad strides 1.7x further per revolution).
    const rate = physics.rate, fps = physics.fps;
    if (Number.isFinite(rate)) this.slow = this.slow ? rate < SLOW_CLEAR : rate < SLOW_RATE;
    const pace = Number.isFinite(rate) ? ` · ×${Math.min(1, rate).toFixed(2)} real time, ${fps.toFixed(0)} frames/s`
      + `${this.slow ? ' ⚠ physics slower than real time' : ''}` : '';
    hud.model = `MuJoCo · t ${fr.t.toFixed(1)} s${pace}${model ? ` · quasi-static model ${model.toFixed(0)} mm/s (no slip)` : ''}`;
    // Drive torque against the servo's ratings; over the rated torque for OVER_RATED_S is a stall warning.
    const [tl, tr] = fr.torque, rated = physics.torqueRated, stall = physics.torqueStall;
    const over = rated !== null && Math.max(Math.abs(tl), Math.abs(tr)) > rated;
    this.overRatedSince = over ? (this.overRatedSince ?? fr.t) : null;
    const overFor = this.overRatedSince !== null ? fr.t - this.overRatedSince : 0;
    hud.torque = `L ${tl.toFixed(2)} · R ${tr.toFixed(2)} N·m (rated ${rated === null ? '?' : rated.toFixed(2)}, stall ${stall.toFixed(2)})`
      + (overFor > OVER_RATED_S ? ` ⚠ over rated ${overFor.toFixed(0)} s` : '');
    if (this.lastRev) {
      const r = this.lastRev, kin = physics.kinematicStride;
      // The stride is the kinematic one minus the skating (the Klann quad's feet slip ~40 % away).
      const skate = kin && kin > 1 ? ` (kinematic ${f(kin, 0)}, skating ${f((1 - r.advance! / kin) * 100, 0)} %)` : '';
      Object.assign(hud, {
        'rev distance': `${f(r.advance)} mm in ${f(r.seconds, 2)} s${skate}`, 'rev turn': `${f(r.turn)}°`,
        'rev bob': `${f(r.bob)} mm`, 'rev pitch': `${f(r.pitchMin)} … ${f(r.pitchMax)}°`,
        'rev roll': `${f(r.rollMin)} … ${f(r.rollMax)}°`,
        loads: `az peak ${f(r.az! / 9.81)} g · pin ${f(r.pin, 0)} N (last rev)`,
      });
    } else {
      hud.loads = `az peak ${f(this.loads.az / 9.81)} g · pin ${f(this.loads.pin, 0)} N (so far)`;
    }
    const steer = physics.steering, [cl, cr] = lastCommand, safe = safeTurn(steer);
    const wantsTurn = Math.abs(cl - cr) > 1e-3 || Math.abs(cl + cr) < 1e-6 && turning;
    const disabled = turning && !!steer && safe === 0 && Math.abs(cl + cr) > 1e-6;
    const beyond = turning && steer && Math.abs(cl + cr) > 1e-6 && Math.abs(cl - cr) > safe + 1e-3;
    const lowSupport = steer?.tests.walk_turn?.side_support_low ?? 0;
    const weak = wantsTurn && lowSupport > SUPPORT_LOW;
    const stalled = overFor > OVER_RATED_S;
    const down = fr.bodyDown;
    warn.hidden = !fell && !beyond && !weak && !stalled && !disabled && !down;
    warn.textContent = fell ? '⚠ FELL OVER (reset)'
      : down ? '⚠ BODY ON THE FLOOR: it is not walking on its feet'
      : disabled ? '⚠ steering is disabled for this design (it rolls over in MuJoCo): the keys walk it straight'
      : beyond ? `⚠ turning beyond the proven-safe |L−R| ≤ ${safe}: this design rolls over in MuJoCo`
      : stalled ? `⚠ drive torque over the rated ${rated!.toFixed(2)} N·m for ${overFor.toFixed(0)} s: stalling (the real servo cuts out)`
      : `⚠ turning on single-foot support (MuJoCo: a side on < 2 feet ${(lowSupport * 100).toFixed(0)} % of the time): go easy`;
  }
}
