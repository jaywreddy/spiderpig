/**
 * The physics drive's wiring (``index.ts``): the server's MuJoCo model of the glb on screen
 * (``physics.ts``'s ``PhysicsLink``) switched on and off, its command sent from the keys, and
 * each frame posed and read into the HUD (``hud.ts``'s ``PhysicsHud``).
 */
import * as THREE from 'three';
import type GUI from 'three/examples/jsm/libs/lil-gui.module.min.js';
import { followScale, type Stage } from '../scene';
import type { DriveHost } from './index';
import type { BakedDesign } from './tune';
import { MIN_MARGIN_MM, MODEL_ONLY_ROWS, HUD_ROWS, SUPPORT_LOW, deg, safeTurn, type PhysicsHud } from './hud';
import type { Input, Scheme } from './sim';
import type { PhysicsLink, Steering } from './physics';
import type { DriveView, sideActions } from './view';

/** The physics command goes up on every key change and this often besides (the animation loop
 * alone runs at ~2 Hz under software GL, and not at all in a hidden tab). */
const COMMAND_MS = 50;

/** The drive's state the physics wiring shares with it (``index.ts`` owns it). */
export interface DriveState {
  side: ReturnType<typeof sideActions> | null;
  /** The side actions run the parts (``bindGlb``); physics stops them but keeps this set. */
  engaged: boolean;
  /** A robot load in flight (its ``onLoad`` binds it). */
  loading: boolean;
  /** The query the glb on screen was baked with ('' = default): what physics simulates. */
  glbQuery: string;
  /** The walking model's straight-walk metrics. */
  lastPred: Record<string, unknown> | null;
  /** When the HUD was last written (ms, ``performance.now``). */
  hudAt: number;
}

export interface PhysicsHost {
  host: DriveHost;
  stage: Stage;
  url: URLSearchParams;
  st: DriveState;
  opts: { drive: boolean; physics: boolean; scheme: Scheme; speed: number; turn: number; camera: string };
  gui: GUI;
  /** The controls only the walking model reads (disabled under physics). */
  modelOnly: ReturnType<GUI['add']>[];
  turnC: ReturnType<GUI['add']>;
  turnName(steer: Steering | null): void;
  hud: Record<string, string>;
  warn: HTMLElement;
  physHud: PhysicsHud;
  physics: PhysicsLink;
  input: Input;
  view: DriveView;
  setDrive(on: boolean): Promise<void>;
  setPreview(on: boolean): Promise<void>;
  bindGlb(): Promise<void>;
  syncUrl(): void;
  fail(e: unknown): void;
}

export function createPhysicsMode(ctx: PhysicsHost) {
  const { host, stage, url, st, opts, gui, hud, warn, physHud, physics, input, view } = ctx;
  let physicsOn = false, physicsWanted = false, physicsBusy: Promise<void> | null = null;
  let commandTimer = 0, lastCommand: [number, number] = [0, 0];

  /** Physics drive: the server's MuJoCo model of this design takes over the body and every
   * part (``physics.ts``); off hands them back to the walking model. Calls are serialised:
   * the last state asked for wins, however fast the toggle went. */
  function setPhysics(on: boolean): Promise<void> {
    physicsWanted = on;
    if (physicsBusy) return physicsBusy;
    if (physicsWanted === physicsOn) return Promise.resolve();   // nothing to do: no promise to get stuck on
    let done = false, run: Promise<void> | undefined;
    run = (async () => {
      try {
        while (physicsWanted !== physicsOn) {
          try {
            await applyPhysics(physicsWanted);
          } catch (e) {
            if (e !== null) throw e;    // null: the connect was superseded (a new glb landed): go round for it
          }
        }
      } finally {
        done = true;                    // maybe before ``run`` is even assigned (a synchronous off)
        if (run && physicsBusy === run) physicsBusy = null;
      }
    })();
    if (!done) physicsBusy = run;       // only a promise still running may be handed out
    return run;
  }

  const physicsC = (): ReturnType<GUI['add']> | undefined => gui.controllers.find((c) => c.property === 'physics');

  async function applyPhysics(on: boolean): Promise<void> {
    if (on) {
      try {
        if (!opts.drive) await ctx.setDrive(true);
        await ctx.setPreview(false);
        await connectPhysics();
      } catch (e) {
        if (e === null) throw e;      // superseded, not refused: the caller tries again
        physicsWanted = false;
        await applyPhysics(false);
        throw e;
      }
    } else {
      const wasOn = physicsOn;
      physicsOff();
      if (opts.drive && !st.loading) {   // mid-load, onLoad binds the new glb once
        if (wasOn) st.engaged = false;   // the side actions were stopped for physics: bindGlb restarts them
        await ctx.bindGlb();
        host.status('drive: walking model (quasi-static)');
      } else if (wasOn && !opts.drive) {
        host.status('physics: off');  // setDrive(false) releases the parts (``engaged`` stayed set)
      }
    }
    ctx.syncUrl();
  }

  /** The physics drive's off path (``applyPhysics(false)`` and a glb that can't be driven landing,
   * ``onLoad``): the session closed, the nodes handed back, the model's controls enabled again, the
   * command timer stopped and every physics-only reading cleared. */
  function physicsOff(): void {
    physicsOn = opts.physics = false;
    physicsC()?.updateDisplay();
    clearInterval(commandTimer);
    commandTimer = 0;
    physics.unbind();
    physics.close();
    view.overlayVisible = true;
    for (const c of ctx.modelOnly) c.enable();
    for (const k of HUD_ROWS) hud[k] = '–';      // the walking model refills its own rows
    ctx.turnName(null);
    physHud.newEpoch();
    warn.hidden = true;
  }

  /** Connect the physics session for the glb on screen (its ``glbQuery``), refusing a design
   * MuJoCo's straight run fell in (the hello's ``forward``; the walking model's margin is a
   * warning, not the gate: Jansen's quad walks at 4 mm), and a model whose nodes aren't the glb's. */
  async function connectPhysics(): Promise<void> {
    const l = host.loaded();
    if (!l || !st.side?.usable) throw new Error('physics needs the robot loaded');
    if (!st.lastPred) await ctx.bindGlb();
    // The side actions stop (physics poses every node); ``engaged`` stays set so release()
    // hands the clip and the standing pose back when drive goes off from here.
    if (st.engaged) { st.side.L.stop(); st.side.R.stop(); }
    host.status('physics: connecting…');
    const query = st.glbQuery;
    const hello = await physics.connect(query, host.status, l);
    if (host.loaded() !== l || st.glbQuery !== query) {   // a new glb landed meanwhile: this hello is its predecessor's
      physics.close();
      throw null;
    }
    const baked = l.root.userData as BakedDesign;
    if (baked.linkage && baked.linkage !== hello.design.linkage) {
      physics.close();
      throw new Error(`physics: the server built ${hello.design.linkage} for a ${baked.linkage} glb`);
    }
    const fwd = hello.forward ?? hello.steering.forward;
    if (fwd && 'fell' in fwd && fwd.fell && url.get('physics') !== 'force') {
      physics.close();
      throw new Error(`physics: this design fell over in MuJoCo's straight run (${hello.design.linkage} `
        + `${hello.design.module}: tilt ${fwd.max_tilt.toFixed(0)}°): tune it (phases, proportions) first`);
    }
    physicsOn = opts.physics = true;
    physicsC()?.updateDisplay();
    const steer = hello.steering;
    // The proven-safe authority is the default; the slider only raises it beyond (with the warning).
    opts.turn = safeTurn(steer);
    ctx.turnC.updateDisplay();
    ctx.turnName(steer);
    lastCommand = [0, 0];
    sendCommand();
    clearInterval(commandTimer);
    commandTimer = window.setInterval(sendCommand, COMMAND_MS);
    physics.onLost = (reason, stale) => {
      void (async () => {
        await setPhysics(false);
        if (stale && opts.drive) {
          host.status(`physics: ${reason}: reconnecting…`);
          await setPhysics(true);
        } else {
          host.status(`physics: ${reason} (physics off)`);
        }
      })().catch(ctx.fail);
    };
    physics.onError = (m) => host.status(`physics: ${m}`);
    // What the server proved (the steering sentence lives in the HUD's ``steering`` row).
    hud.steering = steer.turn > 0 ? `|L−R| ≤ ${steer.turn} while walking` + (steer.spin > 0 ? `, spin ≤ ${steer.spin}` : '')
      : steer.step_deg > 0 ? `${steer.step_deg}° excursions only (phase lock re-locks)` + (steer.spin > 0 ? `, spin ≤ ${steer.spin}` : ', no spin')
      : steer.spin > 0 ? `DISABLED while walking (rolls over), spin ≤ ${steer.spin} only`
      : 'DISABLED for this design (rolls over in MuJoCo)';
    const margin = st.lastPred?.min_margin_mm as number | undefined;
    const notes = [
      fwd && 'walks' in fwd && fwd.walks === false ? 'MuJoCo: it does not walk' : '',
      fwd && 'side_support_low' in fwd && fwd.side_support_low > SUPPORT_LOW
        ? `weak support (a side on < 2 feet ${(fwd.side_support_low * 100).toFixed(0)} % of the time)` : '',
      margin !== undefined && margin < MIN_MARGIN_MM ? `quasi-static margin ${margin.toFixed(1)} mm: marginal` : '',
    ].filter(Boolean);
    host.status(`physics: MuJoCo live (${hello.design.linkage} ${hello.design.module}) — drive with the keys`
      + (notes.length ? ` · ⚠ ${notes.join('; ')}` : ''));
    view.overlayVisible = false;
    for (const c of ctx.modelOnly) c.disable();
    for (const k of MODEL_ONLY_ROWS) hud[k] = '–';
    view.clearTrail();
    physHud.startEpoch();
  }

  /** The command the physics drive sends: the keys' tracks at ``opts.speed`` with the
   * difference between the sides capped at ``opts.turn`` (the server's proven-safe authority by
   * default, ``safeTurn``: the Klann quad rolls over at |L−R| 0.4 while walking but walks a 45°
   * excursion, which its phase lock bounds and re-locks; 0 means steering is disabled and the keys
   * walk it straight at the mean); a pure turn in place (no net forward command) may use the
   * proven-safe spin speed instead. */
  function physicsCommand(left: number, right: number): [number, number] {
    const m = (left + right) / 2, d = (left - right) / 2, steer = physics.steering;
    let cap = opts.turn / 2;
    if (steer && Math.abs(m) < 1e-6) cap = Math.max(cap, steer.spin);
    const dc = Math.sign(d) * Math.min(Math.abs(d), cap);
    return [(m + dc) * opts.speed, (m - dc) * opts.speed];
  }

  /** Send the keys' command now (on change; every ``COMMAND_MS`` besides; [0, 0] while the
   * tab is hidden, so a throttled page doesn't leave the robot walking on its last key). */
  function sendCommand(): void {
    if (!physicsOn) return;
    const [left, right] = document.hidden ? [0, 0] : input.read(opts.scheme);
    lastCommand = physicsCommand(left, right);
    physics.command(lastCommand[0], lastCommand[1]);
  }

  function physicsFrame(left: number, right: number): void {
    const l = host.loaded();
    const [cl, cr] = physicsCommand(left, right);
    if (!document.hidden) { lastCommand = [cl, cr]; physics.command(cl, cr); }
    if (!l || !physics.apply(l)) return;
    const fr = physics.frame!;
    l.walker.updateMatrixWorld(true);
    // Heading: the body's walking axis (mech +x) on the ground.
    const ax = new THREE.Vector3(1, 0, 0).applyQuaternion(fr.quat);
    const heading = Math.atan2(ax.y, ax.x);
    const m = new THREE.Matrix4().compose(fr.pos, fr.quat, new THREE.Vector3(1, 1, 1));
    view.trailTo(fr.pos);
    if (opts.camera !== 'free') view.follow(stage.camera, stage.controls, m, heading, opts.camera === 'chase', followScale(stage));
    const up = new THREE.Vector3(0, 1, 0).applyQuaternion(fr.quat);   // mech +y
    const pitch = deg(Math.atan2(up.x * Math.cos(heading) + up.y * Math.sin(heading), up.z));
    const roll = deg(Math.atan2(-up.x * Math.sin(heading) + up.y * Math.cos(heading), up.z));
    physHud.record(fr, heading, pitch, roll);
    if (performance.now() - st.hudAt > 120) {
      st.hudAt = performance.now();
      physHud.update(hud, warn, fr, up, pitch, roll, Math.abs(cl - cr) > 1e-3, lastCommand,
        st.lastPred?.speed_mm_s as number | undefined);
    }
  }

  return {
    setPhysics, physicsOff, physicsCommand, sendCommand, physicsFrame,
    get on() { return physicsOn; },
    get wanted() { return physicsWanted; },
    set wanted(v: boolean) { physicsWanted = v; },
  };
}
