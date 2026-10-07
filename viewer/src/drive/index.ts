/**
 * Drive mode and the tune panel (lil-gui): drive the robot tank- or
 * arcade-style over a ground plane with the SPEC walking model, and tune its
 * design with an instant stick-figure preview from ``/api/walk``: the
 * linkage (``/api/linkages``; switching one re-bakes the robot), its module
 * and its parameters.
 *
 * Walking data comes from the glb's ``walker`` extras ``drive`` or, when the
 * glb has none, from ``/api/walk`` for the glb's design.
 *
 * Physics (``physics.ts``) is a third mode, exclusive with the stick preview:
 * the server's MuJoCo model of the glb on screen (the query it was baked
 * with, ``glbQuery``) takes over the body and every part; the HUD then reads
 * from a short history of its frames.
 */
import * as THREE from 'three';
import type { Quaternion, Vector3 } from 'three';
import GUI from 'three/examples/jsm/libs/lil-gui.module.min.js';
import type { LoadedScene } from '../loader';
import { followScale, type Stage } from '../scene';
import { isCompact } from '../layout';
import { evaluate, parseDrive, straightWalk, type DriveData, type Side, type WalkJson } from './model';
import { Input, Sim, type Sample, type Scheme } from './sim';
import { PhysicsLink, type PhysicsFrame, type Steering } from './physics';
import { DriveView, bodyMatrix, sideActions } from './view';

export interface DriveHost {
  stage: Stage;
  loaded(): LoadedScene | null;
  /** Load the robot baked with design ``query`` ('' = default); rejects with the server's detail. */
  loadRobot(query: string): Promise<void>;
  /** Load one side (the side-only ``single`` mode) baked with design ``query``. */
  loadSide(query: string): Promise<void>;
  status(text: string): void;
  seek(t: number): void;
  reframe(): void;
}

interface WalkResponse extends WalkJson {
  valid: boolean; error: string | null; linkage: string; module: string; phases_deg: number[];
  proportions: Record<string, number>; metrics?: Record<string, unknown>;
  legs: { leg: number; joints: Record<string, [number, number][]> }[]; links: [string, string][];
  side_z?: Partial<Record<Side, number>>;
}

/** ``/api/design/{id}``: a stored design's card (``spiderpig view``). */
export interface DesignCard {
  design: string; kind: 'walker' | 'mechanism'; linkage: string; module: string; sides: number;
  mode: string; phases_deg: number[]; params: Record<string, number>; servo: string;
}

/** One entry of ``/api/linkages``. */
export interface LinkageInfo {
  key: string; name: string; family: string; notes: string; source: string;
  params: { name: string; default: number; angle: boolean }[];
  modules: Record<string, number>; default_module: string; labels: Record<string, string>; feet: number;
  kind: 'walker' | 'mechanism'; output: { kind: string; name: string; motion: string } | null;
}

/** The glb root's design (``bake.py`` writes ``linkage``/``module`` on the root and the
 * walker's ``drive.params``: ``config.design_json()``). */
interface BakedDesign { linkage?: string; module?: string; phases_deg?: number[]; proportions?: Record<string, number> }

/** What the server builds for a query naming no linkage or module (``linkage.DEFAULT``,
 * ``config.default_module``): the first load's assumption until ``/api/linkages`` answers. */
const SERVER_DEFAULT = { linkage: 'strider', module: 'double' };

const PLOTS: (keyof Sample)[] = ['speed', 'yawRate', 'height', 'pitch', 'roll', 'slip', 'margin'];
const HUD_ROWS = ['speed', 'yaw rate', 'height', 'pitch', 'roll', 'contacts', 'slip', 'margin', 'cranks', 'torque',
  'loads', 'steering', 'rev distance', 'rev turn', 'rev bob', 'rev pitch', 'rev roll', 'rev slip', 'rev margin',
  'model'];
/** Rows the physics drive has no source for (the walking model's support geometry). */
const MODEL_ONLY_ROWS = ['slip', 'margin', 'rev slip', 'rev margin'];
/** Rows only the physics drive has (the walking model has no dynamics). */
const PHYSICS_ONLY_ROWS = ['torque', 'loads', 'steering'];
/** A design whose quasi-static stability margin is under this (mm) is marginal: the walking model
 * warns (Jansen's quad: 4 mm predicted) but MuJoCo decides (the server's straight run, the hello's
 * ``forward``: Jansen's quad walks it at 17° of tilt). Physics refuses a design only when that run
 * fell, unless the URL says ``physics=force``. */
export const MIN_MARGIN_MM = 15;
const HISTORY_S = 1.5;      // of physics frames behind the HUD's speed and yaw rate
/** The physics command goes up on every key change and this often besides (the animation loop
 * alone runs at ~2 Hz under software GL, and not at all in a hidden tab). */
const COMMAND_MS = 50;
/** One side on fewer than two feet more often than this (``sim.run.SUPPORT_LOW``) is weak support. */
const SUPPORT_LOW = 0.10;
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
const safeTurn = (steer: Steering | null): number =>
  !steer ? 0 : steer.turn > 0 ? steer.turn : steer.step_deg > 0 ? STEP_TURN : 0;

async function getJson<T>(url: string, signal?: AbortSignal): Promise<T> {
  const res = await fetch(url, { signal });
  if (res.ok) return (await res.json()) as T;
  const d = ((await res.json().catch(() => ({}))) as { detail?: unknown }).detail;
  throw new Error(typeof d === 'string' ? d : `${res.status} ${JSON.stringify(d ?? res.statusText)}`);
}

const fmt = (v: unknown): string => (typeof v === 'number' ? String(+v.toFixed(2))
  : Array.isArray(v) ? v.map(fmt).join(v.length === 2 ? ' … ' : ', ') : String(v));
const f = (v: number | undefined, d = 1): string => (Number.isFinite(v) ? v!.toFixed(d) : '–');
const deg = (rad: number): number => (rad * 180) / Math.PI;
const paramKeys = (q: URLSearchParams): string[] => {
  const k: string[] = [];
  q.forEach((_, n) => k.push(n));
  return k;
};

/** One physics frame reduced for the HUD. */
interface PhysSample { t: number; x: number; y: number; heading: number; height: number; pitch: number; roll: number }

export function createDrive(host: DriveHost) {
  const { stage } = host;
  const sim = new Sim();
  const view = new DriveView(stage.scene);
  const url = new URLSearchParams(location.search);
  // ``turn`` is the physics drive's |L − R| cap: set from the server's steering check of the
  // design on connect (the hello's ``steering``, ``safeTurn``); the slider may raise it beyond.
  const opts = {
    drive: false, physics: false, scheme: (url.get('scheme') === 'arcade' ? 'arcade' : 'tank') as Scheme,
    speed: 1, accel: 3, phase: 0, turn: 0, camera: 'chase', support: true, plot: 'speed' as keyof Sample,
    reset: () => { sim.reset(); physics.reset(); view.clearTrail(); newEpoch(); },
  };
  const input = new Input(() => opts.drive);
  const physics = new PhysicsLink();
  let physicsOn = false, physicsWanted = false, physicsBusy: Promise<void> | null = null;
  let commandTimer = 0, lastCommand: [number, number] = [0, 0], overRatedSince: number | null = null;
  const physHistory: PhysSample[] = [];
  // The per-revolution window, closed by the cranks' mean |travel| (a spin turns them opposite ways).
  let physRev: { travel: number; t: number; x: number; y: number; heading: number; hMin: number; hMax: number;
    pMin: number; pMax: number; rMin: number; rMax: number } | null = null;
  let physLastRev: Record<string, number> | null = null;
  let physTravel = 0, physLastCrank: [number, number] | null = null, physEpoch = 0, slow = false;
  let physLoads = { az: 0, pin: 0 };

  /** The sim clock went back (a reset, ours or a frame's): every history starts over. */
  function newEpoch(): void {
    physHistory.length = 0;
    physRev = null;
    physLastRev = null;
    physTravel = 0;
    physLastCrank = null;
    physLoads = { az: 0, pin: 0 };
    overRatedSince = null;
  }
  let side: ReturnType<typeof sideActions> | null = null;
  let saved: { p: Vector3; q: Quaternion } | null = null;
  let engaged = false, preview = false, loading = false, lastWalk: WalkResponse | null = null;
  // The query the glb on screen was baked with ('' = default): what physics simulates.
  let glbQuery = '';
  let lastPred: Record<string, unknown> | null = null;   // the walking model's straight-walk metrics

  // --- GUI: drive options + HUD (right), tune (left) ------------------------
  const gui = new GUI({ title: 'Drive', width: 300 });
  gui.domElement.id = 'drive-gui';
  gui.add(opts, 'drive').name('drive mode').onChange((on: boolean) => void setDrive(on).catch(fail));
  gui.add(opts, 'physics').name('physics (MuJoCo)').onChange((on: boolean) => void setPhysics(on).catch(fail));
  gui.add(opts, 'scheme', ['tank', 'arcade']).name('controls').onChange(syncUrl);
  gui.add(opts, 'speed', 0.1, 1, 0.05).name('speed (× max rpm)').onChange((v: number) => { sim.speedScale = v; });
  // Only the walking model shapes the command: these are inert (disabled) under physics.
  const modelOnly = [
    gui.add(opts, 'accel', 0.5, 20, 0.5).name('accel (max/s)').onChange((v: number) => { sim.accel = v; }),
    gui.add(opts, 'phase', -180, 180, 1).name('R − L phase (°)')
      .onChange((v: number) => { sim.phaseOffset = (v * Math.PI) / 180; }),
    gui.add(opts, 'support').name('support + COM'),
  ];
  gui.add(opts, 'turn', 0, 2, 0.05).name('turn authority |L−R| ≤ (physics)');
  const turnC = gui.controllers[gui.controllers.length - 1]!;
  const turnName = (steer: Steering | null): void => {
    const safe = safeTurn(steer);
    turnC.name(steer ? `turn authority |L−R| ≤ (server: ${safe}${steer.turn > 0 ? '' : steer.step_deg > 0
      ? `, ${steer.step_deg}° excursion` : ', none'})` : 'turn authority |L−R| ≤ (physics)');
  };
  gui.add(opts, 'camera', ['chase', 'follow', 'free']);
  gui.add(opts, 'reset');
  const hud: Record<string, string> = Object.fromEntries(HUD_ROWS.map((k) => [k, '–']));
  const hudF = gui.addFolder('HUD');
  for (const k of HUD_ROWS) hudF.add(hud, k).disable().listen();
  hudF.add(opts, 'plot', PLOTS);
  const spark = Object.assign(document.createElement('canvas'), { width: 290, height: 56, id: 'drive-spark' });
  hudF.$children.append(spark);
  const pad = document.getElementById('pad');      // the on-screen arrows: a press starts driving
  if (pad) input.bindPad(pad, () => { if (!opts.drive) void setDrive(true).catch(fail); });
  const warn = Object.assign(document.createElement('div'), { id: 'drive-warn', hidden: true });
  document.body.append(warn);

  const design = {
    id: url.get('design') as string | null,   // a stored design: every query carries it
    linkage: url.get('linkage') ?? SERVER_DEFAULT.linkage,
    module: url.get('module') ?? '',          // '': the linkage's own default (below)
    phases: (url.get('phases')?.split(',').map(Number) ?? null) as number[] | null,
    props: Object.fromEntries(paramKeys(url).filter((k) => k.startsWith('p.'))
      .map((k) => [k.slice(2), Number(url.get(k))])) as Record<string, number>,
  };
  let linkages: LinkageInfo[] | null = null, defaultLinkage = SERVER_DEFAULT.linkage;   // from /api/linkages
  const info = (): LinkageInfo | undefined => linkages?.find((l) => l.key === design.linkage);
  /** The module a design of ``design.linkage`` gets when a query names none: the catalogue's
   * ``default_module`` (Strider's double, another walker's quad); ``SERVER_DEFAULT`` before it. */
  const defaultModule = (): string => info()?.default_module
    ?? (design.linkage === SERVER_DEFAULT.linkage ? SERVER_DEFAULT.module : 'quad');
  // A URL naming a linkage but no module gets that linkage's default module (Klann's quad), not
  // the default linkage's (it used to get Strider's double: ?linkage=klann loaded a Klann double).
  if (!design.module) design.module = defaultModule();
  const guessedModule = url.has('module') || design.id ? null : design.module;
  const defaults = (): Record<string, number> =>
    Object.fromEntries((info()?.params ?? []).map((p) => [p.name, p.default]));
  const tune = { preview: false, status: '', rebuild: () => void rebuild(), reset: () => resetDefaults() };
  const tuneGui = new GUI({ title: 'Tune (instant preview)', width: 300, autoPlace: false }).close();
  tuneGui.domElement.id = 'tune-gui';
  document.body.append(tuneGui.domElement);
  tuneGui.add(tune, 'preview').name('stick preview').onChange((on: boolean) => void setPreview(on).catch(fail));
  const designF = tuneGui.addFolder('design');           // linkage, module
  const phaseF = tuneGui.addFolder('phases (°)');
  const propF = tuneGui.addFolder('parameters (lengths ×0.5–1.5, angles ±45°)');
  tuneGui.add(tune, 'rebuild').name('Rebuild parts');
  tuneGui.add(tune, 'reset').name('Reset to defaults');
  tuneGui.add(tune, 'status').disable().listen();
  const metricF = tuneGui.addFolder('metrics (per revolution)');
  // A phone or tablet starts with both panels collapsed (title bars only) and opens one at a time:
  // either one open covers most of the screen.
  const compact = isCompact();
  if (compact) { gui.close(); hudF.close(); }
  const exclusive = (a: GUI, b: GUI): void => {
    a.domElement.querySelector(':scope > .title')?.addEventListener('click', () => {
      if (isCompact() && !a._closed) b.close();
    });
  };
  exclusive(gui, tuneGui);
  exclusive(tuneGui, gui);
  let moduleC: ReturnType<GUI['add']> | null = null, linkageC: ReturnType<GUI['add']> | null = null;

  function fail(e: unknown): void {
    if (e === null) return;               // a superseded physics connect: nothing to show
    tune.status = (e as Error).message;
    host.status(`drive: ${tune.status}`);
  }

  // --- data, drive on / off, preview on / off ---------------------------------
  function useData(data: DriveData): Record<string, unknown> {
    const pred = (lastPred = straightWalk(data));
    sim.setData(data, (pred.stride_signed_mm as number) < 0 ? -1 : 1);
    hud.model = `${fmt(pred.stride_mm)} mm/rev · ${fmt(pred.speed_mm_s)} mm/s`;
    return pred;
  }

  /** Drive the full model: its extras, else ``/api/walk`` for its design. */
  async function bindGlb(): Promise<void> {
    const l = host.loaded();
    if (!l || !side?.usable) return;
    const extras = l.walker.userData.drive as WalkJson | undefined;
    const { module = defaultModule(), linkage = defaultLinkage } = l.root.userData as { module?: string; linkage?: string };
    useData(parseDrive(extras ?? await getJson<WalkResponse>(
      `/api/walk?${glbQuery || new URLSearchParams({ module, linkage }).toString()}`)
      .catch((e: Error) => { throw new Error(`no walking data (no glb extras; /api/walk: ${e.message})`); })));
    l.root.visible = true;
    if (!engaged) {   // the whole clip -> one action per side, posed per frame
      host.seek(0);
      l.action.stop();
      for (const a of [side.L, side.R]) { a.play(); a.paused = true; }
      engaged = true;
    }
  }

  function release(): void {
    const l = host.loaded();
    if (l && side && engaged) {
      side.L.stop(); side.R.stop();
      l.action.play();
      if (saved) { l.walker.position.copy(saved.p); l.walker.quaternion.copy(saved.q); }
      host.seek(0);
    }
    engaged = false;
  }

  /** ``host.loadRobot`` with the load marked in flight (``onLoad`` binds once it lands). */
  async function loadRobot(query: string): Promise<void> {
    loading = true;
    try { await host.loadRobot(query); } finally { loading = false; }
  }

  async function setDrive(on: boolean): Promise<void> {
    opts.drive = on;
    try {
      if (on && !preview && side?.usable) await bindGlb();
      // its onLoad binds it; with no glb on screen yet (the page's own load failed), the page's
      // design, not '' (the server's default: ?drive=1&linkage=X used to drive a Strider)
      else if (on && !preview) await loadRobot(host.loaded() ? glbQuery : baseQuery('robot'));
    } catch (e) {
      opts.drive = false;
      gui.controllers[0]!.updateDisplay();
      throw e;
    }
    if (!on) {
      await setPhysics(false);
      await setPreview(false);
      release();
      host.reframe();
      for (const k of HUD_ROWS) hud[k] = '–';     // nothing is driving: no stale readings
    }
    view.visible = on;
    stage.grid.visible = !on;
    if (on) {
      view.clearTrail();
      Object.assign(stage.camera, { near: 5, far: 60000 }).updateProjectionMatrix();
    }
    warn.hidden = true;
    document.body.classList.toggle('driving', on);
    gui.controllers[0]!.updateDisplay();
    syncUrl();
  }

  /** The stick preview and physics are exclusive: the preview hides the parts physics moves. */
  async function setPreview(on: boolean): Promise<void> {
    if (on === preview) return;
    if (on && (physicsOn || physicsWanted)) await setPhysics(false);
    preview = tune.preview = on;
    tuneGui.controllers[0]!.updateDisplay();
    const l = host.loaded();
    view.stick.visible = on;
    if (l) l.root.visible = !on;
    if (on) {
      tuneGui.open();
      if (compact) gui.close();
      if (!opts.drive) await setDrive(true);
      await loadLinkages();
      request(0);
    } else if (opts.drive) {
      await bindGlb();
    }
    syncUrl();
  }

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
        if (!opts.drive) await setDrive(true);
        await setPreview(false);
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
      if (opts.drive && !loading) {   // mid-load, onLoad binds the new glb once
        if (wasOn) engaged = false;   // the side actions were stopped for physics: bindGlb restarts them
        await bindGlb();
        host.status('drive: walking model (quasi-static)');
      } else if (wasOn && !opts.drive) {
        host.status('physics: off');  // setDrive(false) releases the parts (``engaged`` stayed set)
      }
    }
    syncUrl();
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
    for (const c of modelOnly) c.enable();
    for (const k of HUD_ROWS) hud[k] = '–';      // the walking model refills its own rows
    turnName(null);
    newEpoch();
    warn.hidden = true;
  }

  /** Connect the physics session for the glb on screen (its ``glbQuery``), refusing a design
   * MuJoCo's straight run fell in (the hello's ``forward``; the walking model's margin is a
   * warning, not the gate: Jansen's quad walks at 4 mm), and a model whose nodes aren't the glb's. */
  async function connectPhysics(): Promise<void> {
    const l = host.loaded();
    if (!l || !side?.usable) throw new Error('physics needs the robot loaded');
    if (!lastPred) await bindGlb();
    // The side actions stop (physics poses every node); ``engaged`` stays set so release()
    // hands the clip and the standing pose back when drive goes off from here.
    if (engaged) { side.L.stop(); side.R.stop(); }
    host.status('physics: connecting…');
    const query = glbQuery;
    const hello = await physics.connect(query, host.status, l);
    if (host.loaded() !== l || glbQuery !== query) {   // a new glb landed meanwhile: this hello is its predecessor's
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
    turnC.updateDisplay();
    turnName(steer);
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
      })().catch(fail);
    };
    physics.onError = (m) => host.status(`physics: ${m}`);
    // What the server proved (the steering sentence lives in the HUD's ``steering`` row).
    hud.steering = steer.turn > 0 ? `|L−R| ≤ ${steer.turn} while walking` + (steer.spin > 0 ? `, spin ≤ ${steer.spin}` : '')
      : steer.step_deg > 0 ? `${steer.step_deg}° excursions only (phase lock re-locks)` + (steer.spin > 0 ? `, spin ≤ ${steer.spin}` : ', no spin')
      : steer.spin > 0 ? `DISABLED while walking (rolls over), spin ≤ ${steer.spin} only`
      : 'DISABLED for this design (rolls over in MuJoCo)';
    const margin = lastPred?.min_margin_mm as number | undefined;
    const notes = [
      fwd && 'walks' in fwd && fwd.walks === false ? 'MuJoCo: it does not walk' : '',
      fwd && 'side_support_low' in fwd && fwd.side_support_low > SUPPORT_LOW
        ? `weak support (a side on < 2 feet ${(fwd.side_support_low * 100).toFixed(0)} % of the time)` : '',
      margin !== undefined && margin < MIN_MARGIN_MM ? `quasi-static margin ${margin.toFixed(1)} mm: marginal` : '',
    ].filter(Boolean);
    host.status(`physics: MuJoCo live (${hello.design.linkage} ${hello.design.module}) — drive with the keys`
      + (notes.length ? ` · ⚠ ${notes.join('; ')}` : ''));
    view.overlayVisible = false;
    for (const c of modelOnly) c.disable();
    for (const k of MODEL_ONLY_ROWS) hud[k] = '–';
    view.clearTrail();
    newEpoch();
    physEpoch = physics.epoch;
  }

  // --- tune ------------------------------------------------------------------
  /** The design as ``/api/walk`` and ``/api/glb`` query parameters (defaults left out;
   * a stored design's id first: the server starts from its config and applies the rest). */
  function designQuery(): string {
    const q = new URLSearchParams(design.id ? { design: design.id, module: design.module }
      : { module: design.module });
    if (design.linkage !== defaultLinkage) q.set('linkage', design.linkage);
    if (design.phases) q.set('phases', design.phases.map((v) => +v.toFixed(2)).join(','));
    const d = defaults();
    for (const [k, v] of Object.entries(design.props)) {
      if (!(k in d) || Math.abs(v - d[k]!) > 1e-9) q.set(`p.${k}`, String(+v.toPrecision(6)));
    }
    return q.toString();
  }

  /** A stored design (``?design=<id>``): its card seeds the tune panel's state (linkage,
   * module, phases, proportions), and every query carries its id from then on, so the
   * server answers with its servo, sheet and constructions too. */
  async function loadDesign(id: string): Promise<DesignCard> {
    const card = await getJson<DesignCard>(`/api/design/${encodeURIComponent(id)}`);
    design.id = card.design;
    design.linkage = card.linkage;
    design.module = card.module;
    design.phases = [...card.phases_deg];
    design.props = { ...card.params };
    return card;
  }

  /** The registered linkages (once): the walker dropdown, the design controls, and a
   * mechanism picker (one side, not tuned). */
  async function loadLinkages(): Promise<void> {
    if (linkages) return;
    const r = await getJson<{ default: string; linkages: LinkageInfo[] }>('/api/linkages');
    [linkages, defaultLinkage] = [r.linkages.filter((l) => l.kind === 'walker'), r.default];
    if (!info()) design.linkage = r.default;
    // a module guessed before the catalogue came (and not changed since): the catalogue's
    if (guessedModule !== null && design.module === guessedModule) design.module = info()!.default_module;
    design.props = { ...defaults(), ...design.props };     // the URL's p.NAME win
    linkageC = designF.add(design, 'linkage', Object.fromEntries(linkages.map((l) => [l.name, l.key])))
      .onChange(() => queueMicrotask(switchLinkage));
    const mechs = r.linkages.filter((l) => l.kind === 'mechanism');
    designF.add({ mechanism: '' }, 'mechanism',
      { '–': '', ...Object.fromEntries(mechs.map((l) => [l.name, l.key])) })
      .name('mechanism (one side)')
      .onChange((k: string) => { if (k) host.loadSide(`linkage=${k}`).catch(fail); });
    buildDesign();
  }

  /** The module dropdown (the linkage's modules) and a slider per parameter. */
  function buildDesign(): void {
    const lk = info()!;
    moduleC?.destroy();
    moduleC = designF.add(design, 'module', Object.keys(lk.modules))
      .onChange(() => { design.phases = null; request(); });
    propF.controllers.slice().forEach((c) => c.destroy());
    for (const p of lk.params) {
      const [lo, hi] = p.angle ? [p.default - 45, p.default + 45] : [0.5 * p.default, 1.5 * p.default];
      propF.add(design.props, p.name, lo, hi, p.angle ? 0.1 : p.default / 1000)
        .name(`${p.name} (${p.default})`).onChange(() => request());
    }
  }

  /** Another linkage: its defaults, its modules (keeping this one if it has it), and the parts re-baked. */
  function switchLinkage(): void {
    const mods = Object.keys(info()!.modules);
    if (!mods.includes(design.module)) design.module = info()!.default_module;
    design.phases = null;
    design.props = defaults();
    buildDesign();
    if (preview) request(0);
    void rebuild();
  }

  let timer = 0, inflight: AbortController | null = null;
  function request(delay = 150): void {
    clearTimeout(timer);
    timer = window.setTimeout(() => {
      inflight?.abort();
      inflight = new AbortController();
      tune.status = 'computing…';
      getJson<WalkResponse>(`/api/walk?${designQuery()}`, inflight.signal).then(applyWalk).catch((e: Error) => {
        if (e.name !== 'AbortError') tune.status = `error: ${e.message}`;
      });
    }, delay);
    if (!preview) void setPreview(true).catch(fail);
  }

  function applyWalk(w: WalkResponse): void {
    if (!w.valid) { tune.status = `invalid: ${w.error ?? '?'}`; return; }
    lastWalk = w;
    if (!design.phases || phaseF.controllers.length !== w.phases_deg.length) {
      const phases = (design.phases = [...w.phases_deg]);
      phaseF.controllers.slice().forEach((c) => c.destroy());
      phases.forEach((_, i) => phaseF.add(phases as unknown as Record<string, number>, String(i), 0, 360, 1)
        .name(`leg ${i}`).onChange(() => request()));
    }
    view.setStick(w);
    const data = parseDrive(w), pred = preview ? useData(data) : straightWalk(data);
    metricF.controllers.slice().forEach((c) => c.destroy());
    const shown = Object.fromEntries(Object.entries(w.metrics ?? pred).map(([k, v]) => [k, fmt(v)]));
    for (const k of Object.keys(shown)) metricF.add(shown, k).disable();
    const margin = (w.metrics?.min_margin_mm ?? pred.min_margin_mm) as number;
    const walks = (w as { walks?: boolean }).walks ?? (w.metrics?.walks as boolean | undefined) ?? true;
    tune.status = `ok · ${w.linkage} ${w.module}${w.metrics ? '' : ' (metrics: viewer model)'}`
      + (!walks ? ' · ⚠ DOES NOT WALK (no stride, or on < 3 feet)'
        : margin < MIN_MARGIN_MM ? ` · ⚠ margin ${margin.toFixed(1)} mm: may tip (MuJoCo decides)` : '');
    syncUrl();
  }

  /** Re-bake the robot for the tune panel's design; the glb that lands sets ``glbQuery``
   * (``onLoad``). When the bake fails the panel goes back to the design on screen. */
  async function rebuild(): Promise<void> {
    const q = designQuery(), t0 = performance.now();
    const tick = window.setInterval(() => {
      tune.status = `baking parts… ${f((performance.now() - t0) / 1000, 0)} s`;
    }, 500);
    try {
      await loadRobot(q);
      await setPreview(false);
      tune.status = 'parts rebuilt';
    } catch (e) {
      adoptLoaded();
      fail(new Error(`rebuild failed: ${(e as Error).message}`));
    } finally {
      clearInterval(tick);
    }
  }

  /** The tune panel's design back to what the glb on screen was baked as. */
  function adoptLoaded(): void {
    const l = host.loaded();
    const baked = (l?.walker.userData.drive as { params?: BakedDesign } | undefined)?.params
      ?? (l?.root.userData as BakedDesign | undefined);
    if (!baked?.linkage) return;
    design.linkage = baked.linkage;
    design.module = baked.module ?? design.module;
    design.phases = baked.phases_deg ? [...baked.phases_deg] : null;
    design.props = { ...defaults(), ...(baked.proportions ?? {}) };
    if (linkages) buildDesign();
    linkageC?.updateDisplay();
    if (preview) request(0);
    syncUrl();
  }

  function resetDefaults(): void {
    Object.assign(design.props, defaults());
    design.phases = null;
    propF.controllers.forEach((c) => c.updateDisplay());
    request(0);
  }

  /** Cheap deep links: ``drive=1``, ``physics=1``, ``scheme=arcade``, the design on screen
   * (``linkage``, ``module``, ``design`` when not the default) and, with the tune panel open,
   * ``tune=1`` + its design (``phases``, ``p.NAME``). */
  function syncUrl(): void {
    const q = new URLSearchParams(location.search);
    for (const k of paramKeys(q)) if (/^(drive|physics|scheme|tune|linkage|module|design|phases|p\..*)$/.test(k)) q.delete(k);
    if (opts.drive) q.set('drive', '1');
    if (physicsOn) q.set('physics', '1');
    if (opts.scheme !== 'tank') q.set('scheme', opts.scheme);
    new URLSearchParams(baseQuery('robot')).forEach((v, k) => q.set(k, v));
    if (preview) { q.set('tune', '1'); new URLSearchParams(designQuery()).forEach((v, k) => q.set(k, v)); }
    history.replaceState(null, '', `?${q.toString()}`.replace(/%2C/g, ','));
  }

  /** The query a plain load of ``mode`` carries: the stored design's id, the selected
   * linkage (and the module for the robot or one side of it). */
  function baseQuery(mode: string): string {
    const q = new URLSearchParams();
    if (design.id) q.set('design', design.id);
    if ((mode === 'robot' || mode === 'side') && design.module !== defaultModule()) q.set('module', design.module);
    if (design.linkage !== defaultLinkage) q.set('linkage', design.linkage);
    return q.toString();
  }

  // --- per frame -------------------------------------------------------------
  let hudAt = 0;
  function frame(dt: number): void {
    const [left, right] = input.read(opts.scheme);
    if (physicsOn) { physicsFrame(left, right); return; }
    sim.step(Math.min(dt, 0.5), left, right);   // real time, unless the tab stalled
    const s = sim.state, l = host.loaded();
    if (!s || !sim.data) return;
    const m = bodyMatrix(sim, s);
    if (engaged && !preview && l && side) {
      m.decompose(l.walker.position, l.walker.quaternion, l.walker.scale);
      const tau = (th: number): number => (((th / (2 * Math.PI)) % 1) + 1) % 1 * side!.period;
      side.L.time = tau(sim.modelTheta.L);
      side.R.time = tau(sim.modelTheta.R);
      l.mixer.update(0);
    }
    view.update(m, s, sim.data.com, sim, opts.support);
    const heading = sim.yaw + (sim.forward < 0 ? Math.PI : 0);
    if (opts.camera !== 'free') view.follow(stage.camera, stage.controls, m, heading, opts.camera === 'chase', followScale(stage));
    if (performance.now() - hudAt > 120) { hudAt = performance.now(); updateHud(); }
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
  input.onChange = sendCommand;
  addEventListener('visibilitychange', sendCommand);

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
    recordPhysics(fr, heading, pitch, roll);
    if (performance.now() - hudAt > 120) {
      hudAt = performance.now();
      updatePhysicsHud(fr, up, pitch, roll, Math.abs(cl - cr) > 1e-3);
    }
  }

  /** Keep ``HISTORY_S`` of frames and the per-revolution window (closed by the cranks' mean |travel|,
   * so a turn in place completes revolutions too). A frame whose clock went back (a reset landed on
   * the server) starts a new epoch: nothing before it is comparable. */
  function recordPhysics(fr: PhysicsFrame, heading: number, pitch: number, roll: number): void {
    let last = physHistory[physHistory.length - 1];
    if (physics.epoch !== physEpoch || (last && fr.t < last.t)) { newEpoch(); physEpoch = physics.epoch; last = undefined; }
    if (last && fr.t <= last.t) return;
    const height = fr.pos.z;
    if (last) heading = last.heading + Math.atan2(Math.sin(heading - last.heading), Math.cos(heading - last.heading));
    physHistory.push({ t: fr.t, x: fr.pos.x, y: fr.pos.y, heading, height, pitch, roll });
    while (physHistory.length > 2 && physHistory[0]!.t < fr.t - HISTORY_S) physHistory.shift();
    physLoads = { az: Math.max(physLoads.az, Math.abs(fr.az)), pin: Math.max(physLoads.pin, fr.pinLoad) };
    if (physLastCrank) {
      physTravel += (Math.abs(fr.crank[0] - physLastCrank[0]) + Math.abs(fr.crank[1] - physLastCrank[1])) / 2;
    }
    physLastCrank = [fr.crank[0], fr.crank[1]];
    if (!physRev) {
      physRev = { travel: physTravel, t: fr.t, x: fr.pos.x, y: fr.pos.y, heading, hMin: height, hMax: height,
        pMin: pitch, pMax: pitch, rMin: roll, rMax: roll };
      return;
    }
    const r = physRev;
    r.hMin = Math.min(r.hMin, height); r.hMax = Math.max(r.hMax, height);
    r.pMin = Math.min(r.pMin, pitch); r.pMax = Math.max(r.pMax, pitch);
    r.rMin = Math.min(r.rMin, roll); r.rMax = Math.max(r.rMax, roll);
    if (physTravel - r.travel >= 2 * Math.PI) {
      physLastRev = { seconds: fr.t - r.t, advance: Math.hypot(fr.pos.x - r.x, fr.pos.y - r.y),
        turn: deg(heading - r.heading), bob: r.hMax - r.hMin, pitchMin: r.pMin, pitchMax: r.pMax,
        rollMin: r.rMin, rollMax: r.rMax, az: physLoads.az, pin: physLoads.pin };
      physLoads = { az: 0, pin: 0 };
      physRev = null;
    }
  }

  function updatePhysicsHud(fr: PhysicsFrame, up: THREE.Vector3, pitch: number, roll: number, turning: boolean): void {
    const fell = up.z <= Math.cos(Math.PI / 4);
    if (fell) physHistory.length = 0;     // it isn't walking any more: speed and yaw rate read 0
    const a = physHistory[0], b = physHistory[physHistory.length - 1];
    const span = a && b && b.t > a.t ? b.t - a.t : 0;
    const speed = span ? Math.hypot(b!.x - a!.x, b!.y - a!.y) / span : 0;
    const yawRate = span ? deg(b!.heading - a!.heading) / span : 0;
    const rpm = (w: number): string => f((w * 60) / (2 * Math.PI), 0);
    Object.assign(hud, {
      speed: `${speed.toFixed(1)} mm/s (${HISTORY_S} s mean)`, 'yaw rate': `${yawRate.toFixed(1)} °/s`,
      height: `${fr.pos.z.toFixed(1)} mm`, pitch: `${pitch.toFixed(2)}°`, roll: `${roll.toFixed(2)}°`,
      contacts: `${fr.feetDown} / ${physics.feet}${fr.bodyDown ? ' · BODY ON THE FLOOR' : ''}`,
      cranks: `${deg(fr.crank[0]).toFixed(0)}° · ${deg(fr.crank[1]).toFixed(0)}° turned, L−R ${f(deg(fr.sidePhase), 0)}°`
        + (physLastRev ? ` · ${rpm((2 * Math.PI) / physLastRev.seconds!)} rpm` : ''),
    });
    // The quasi-static model's speed is information, not a check: it assumes no slip on the
    // lowest feet, and MuJoCo's feet slip (the Klann quad strides 1.7x further per revolution).
    const model = lastPred?.speed_mm_s as number | undefined;
    const rate = physics.rate, fps = physics.fps;
    if (Number.isFinite(rate)) slow = slow ? rate < SLOW_CLEAR : rate < SLOW_RATE;
    const pace = Number.isFinite(rate) ? ` · ×${Math.min(1, rate).toFixed(2)} real time, ${fps.toFixed(0)} frames/s`
      + `${slow ? ' ⚠ physics slower than real time' : ''}` : '';
    hud.model = `MuJoCo · t ${fr.t.toFixed(1)} s${pace}${model ? ` · quasi-static model ${model.toFixed(0)} mm/s (no slip)` : ''}`;
    // Drive torque against the servo's ratings; over the rated torque for OVER_RATED_S is a stall warning.
    const [tl, tr] = fr.torque, rated = physics.torqueRated, stall = physics.torqueStall;
    const over = rated !== null && Math.max(Math.abs(tl), Math.abs(tr)) > rated;
    overRatedSince = over ? (overRatedSince ?? fr.t) : null;
    const overFor = overRatedSince !== null ? fr.t - overRatedSince : 0;
    hud.torque = `L ${tl.toFixed(2)} · R ${tr.toFixed(2)} N·m (rated ${rated === null ? '?' : rated.toFixed(2)}, stall ${stall.toFixed(2)})`
      + (overFor > OVER_RATED_S ? ` ⚠ over rated ${overFor.toFixed(0)} s` : '');
    if (physLastRev) {
      const r = physLastRev, kin = physics.kinematicStride;
      // The stride is the kinematic one minus the skating (the Klann quad's feet slip ~40 % away).
      const skate = kin && kin > 1 ? ` (kinematic ${f(kin, 0)}, skating ${f((1 - r.advance! / kin) * 100, 0)} %)` : '';
      Object.assign(hud, {
        'rev distance': `${f(r.advance)} mm in ${f(r.seconds, 2)} s${skate}`, 'rev turn': `${f(r.turn)}°`,
        'rev bob': `${f(r.bob)} mm`, 'rev pitch': `${f(r.pitchMin)} … ${f(r.pitchMax)}°`,
        'rev roll': `${f(r.rollMin)} … ${f(r.rollMax)}°`,
        loads: `az peak ${f(r.az! / 9.81)} g · pin ${f(r.pin, 0)} N (last rev)`,
      });
    } else {
      hud.loads = `az peak ${f(physLoads.az / 9.81)} g · pin ${f(physLoads.pin, 0)} N (so far)`;
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

  function updateHud(): void {
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
    const g = spark.getContext('2d')!, vals = sim.history.map((x) => x[opts.plot]);
    let lo = Math.min(...vals), hi = Math.max(...vals);
    if (hi - lo < 1) { lo -= 0.5; hi += 0.5; }
    g.clearRect(0, 0, spark.width, spark.height);
    g.strokeStyle = '#6fb6ff';
    g.beginPath();
    sim.history.forEach((x, i) => g.lineTo(((x.t - h.t + 6) / 6) * spark.width,
      spark.height - 4 - ((vals[i]! - lo) / (hi - lo)) * (spark.height - 8)));
    g.stroke();
    g.fillStyle = '#cfd6de';
    g.fillText(`${opts.plot} ${f(lo)} … ${f(hi)}`, 4, 11);
  }

  return {
    get active() { return opts.drive; },
    get preview() { return preview; },
    get lastWalk() { return lastWalk; },
    get lastPred() { return lastPred; },
    /** The query the glb on screen was baked with (what physics simulates). */
    get glbQuery() { return glbQuery; },
    get physicsOn() { return physicsOn; },
    opts, sim, view, tune, design, hud, gui, tuneGui,
    model: { parseDrive, evaluate, straightWalk },
    setDrive, setPreview, setPhysics, frame, physics, physicsCommand,
    baseQuery,
    loadDesign,
    /** A new glb is on screen, baked with ``query``: split its clip by side; keep driving if it
     * can be driven, and if physics was on, reconnect it for this design. */
    async onLoad(l: LoadedScene, query = ''): Promise<void> {
      engaged = false;
      glbQuery = query;
      lastPred = null;
      side = sideActions(l.mixer, l.clip, l.walker);
      saved = { p: l.walker.position.clone(), q: l.walker.quaternion.clone() };
      const wantPhysics = physicsOn || physicsWanted;
      // The session, live or still building, is the previous glb's: off, through the one off path
      // (a connect in flight rejects with null and setPhysics's loop reconnects for this one).
      if (physicsOn) physicsOff();
      else if (wantPhysics) physics.close();
      if (!side.usable && opts.drive) await setDrive(false);
      else if (opts.drive && !preview) await bindGlb();
      else if (preview) l.root.visible = false;
      if (wantPhysics && side.usable && opts.drive) {
        physicsWanted = true;
        // Not awaited: the in-flight setPhysics may itself be waiting on this load (physics
        // asked while a mechanism was on screen loads the robot first).
        void setPhysics(true).catch(fail);
      }
      syncUrl();    // the deep link follows the design on screen, physics or not
    },
    async init(): Promise<void> {
      await loadLinkages().catch(fail);
      if (url.get('tune') === '1') await setPreview(true);
      else if (url.get('drive') === '1') await setDrive(true);
      if (url.get('physics') === '1') await setPhysics(true).catch(fail);
    },
  };
}

export type Drive = ReturnType<typeof createDrive>;
