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
 *
 * The pieces: ``tune.ts`` (the tune panel and the design's queries), ``physicsMode.ts`` (the
 * physics drive's wiring), ``hud.ts`` (the HUD's rows, warnings and sparkline); this module
 * wires them to the drive panel, the walking model and the glb on screen.
 */
import type { Quaternion, Vector3 } from 'three';
import GUI from 'three/examples/jsm/libs/lil-gui.module.min.js';
import type { LoadedScene } from '../loader';
import { followScale, type Stage } from '../scene';
import { isCompact } from '../layout';
import { evaluate, parseDrive, straightWalk, type DriveData, type WalkJson } from './model';
import { Input, Sim, type Sample, type Scheme } from './sim';
import { PhysicsLink, type Steering } from './physics';
import { DriveView, bodyMatrix, sideActions } from './view';
import { HUD_ROWS, PhysicsHud, createHudPanel, fmt, safeTurn, updateModelHud } from './hud';
import { createTune, getJson, paramKeys, type WalkResponse } from './tune';
import { createPhysicsMode, type DriveState } from './physicsMode';

export { MIN_MARGIN_MM } from './hud';
export type { DesignCard, LinkageInfo } from './tune';

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
    reset: () => { sim.reset(); physics.reset(); view.clearTrail(); physHud.newEpoch(); },
  };
  const input = new Input(() => opts.drive);
  const physics = new PhysicsLink();
  const physHud = new PhysicsHud(physics);
  const st: DriveState = { side: null, engaged: false, loading: false, glbQuery: '', lastPred: null, hudAt: 0 };
  let saved: { p: Vector3; q: Quaternion } | null = null;
  let preview = false;

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
  const { hud, hudF, spark, warn } = createHudPanel(gui, opts);
  const pad = document.getElementById('pad');      // the on-screen arrows: a press starts driving
  if (pad) input.bindPad(pad, () => { if (!opts.drive) void setDrive(true).catch(fail); });

  const t = createTune({
    host, view, url, fail, preview: () => preview, setPreview, loadRobot, useData, syncUrl,
  });
  const { tune, tuneGui, design } = t;
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

  const ph = createPhysicsMode({
    host, stage, url, st, opts, gui, modelOnly, turnC, turnName, hud, warn, physHud, physics, input,
    view, setDrive, setPreview, bindGlb, syncUrl, fail,
  });
  const { setPhysics } = ph;

  function fail(e: unknown): void {
    if (e === null) return;               // a superseded physics connect: nothing to show
    tune.status = (e as Error).message;
    host.status(`drive: ${tune.status}`);
  }

  // --- data, drive on / off, preview on / off ---------------------------------
  function useData(data: DriveData): Record<string, unknown> {
    const pred = (st.lastPred = straightWalk(data));
    sim.setData(data, (pred.stride_signed_mm as number) < 0 ? -1 : 1);
    hud.model = `${fmt(pred.stride_mm)} mm/rev · ${fmt(pred.speed_mm_s)} mm/s`;
    return pred;
  }

  /** Drive the full model: its extras, else ``/api/walk`` for its design. */
  async function bindGlb(): Promise<void> {
    const l = host.loaded();
    const side = st.side;
    if (!l || !side?.usable) return;
    const extras = l.walker.userData.drive as WalkJson | undefined;
    const { module = t.defaultModule(), linkage = t.defaultLinkage } = l.root.userData as { module?: string; linkage?: string };
    useData(parseDrive(extras ?? await getJson<WalkResponse>(
      `/api/walk?${st.glbQuery || new URLSearchParams({ module, linkage }).toString()}`)
      .catch((e: Error) => { throw new Error(`no walking data (no glb extras; /api/walk: ${e.message})`); })));
    l.root.visible = true;
    if (!st.engaged) {   // the whole clip -> one action per side, posed per frame
      host.seek(0);
      l.action.stop();
      for (const a of [side.L, side.R]) { a.play(); a.paused = true; }
      st.engaged = true;
    }
  }

  function release(): void {
    const l = host.loaded(), side = st.side;
    if (l && side && st.engaged) {
      side.L.stop(); side.R.stop();
      l.action.play();
      if (saved) { l.walker.position.copy(saved.p); l.walker.quaternion.copy(saved.q); }
      host.seek(0);
    }
    st.engaged = false;
  }

  /** ``host.loadRobot`` with the load marked in flight (``onLoad`` binds once it lands). */
  async function loadRobot(query: string): Promise<void> {
    st.loading = true;
    try { await host.loadRobot(query); } finally { st.loading = false; }
  }

  async function setDrive(on: boolean): Promise<void> {
    opts.drive = on;
    try {
      if (on && !preview && st.side?.usable) await bindGlb();
      // its onLoad binds it; with no glb on screen yet (the page's own load failed), the page's
      // design, not '' (the server's default: ?drive=1&linkage=X used to drive a Strider)
      else if (on && !preview) await loadRobot(host.loaded() ? st.glbQuery : t.baseQuery('robot'));
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
    if (on && (ph.on || ph.wanted)) await setPhysics(false);
    preview = tune.preview = on;
    tuneGui.controllers[0]!.updateDisplay();
    const l = host.loaded();
    view.stick.visible = on;
    if (l) l.root.visible = !on;
    if (on) {
      tuneGui.open();
      if (compact) gui.close();
      if (!opts.drive) await setDrive(true);
      await t.loadLinkages();
      t.request(0);
    } else if (opts.drive) {
      await bindGlb();
    }
    syncUrl();
  }

  /** Cheap deep links: ``drive=1``, ``physics=1``, ``scheme=arcade``, the design on screen
   * (``linkage``, ``module``, ``design`` when not the default) and, with the tune panel open,
   * ``tune=1`` + its design (``phases``, ``p.NAME``). */
  function syncUrl(): void {
    const q = new URLSearchParams(location.search);
    for (const k of paramKeys(q)) if (/^(drive|physics|scheme|tune|linkage|module|design|phases|p\..*)$/.test(k)) q.delete(k);
    if (opts.drive) q.set('drive', '1');
    if (ph.on) q.set('physics', '1');
    if (opts.scheme !== 'tank') q.set('scheme', opts.scheme);
    new URLSearchParams(t.baseQuery('robot')).forEach((v, k) => q.set(k, v));
    if (preview) { q.set('tune', '1'); new URLSearchParams(t.designQuery()).forEach((v, k) => q.set(k, v)); }
    history.replaceState(null, '', `?${q.toString()}`.replace(/%2C/g, ','));
  }

  // --- per frame -------------------------------------------------------------
  function frame(dt: number): void {
    const [left, right] = input.read(opts.scheme);
    if (ph.on) { ph.physicsFrame(left, right); return; }
    sim.step(Math.min(dt, 0.5), left, right);   // real time, unless the tab stalled
    const s = sim.state, l = host.loaded(), side = st.side;
    if (!s || !sim.data) return;
    const m = bodyMatrix(sim, s);
    if (st.engaged && !preview && l && side) {
      m.decompose(l.walker.position, l.walker.quaternion, l.walker.scale);
      const tau = (th: number): number => (((th / (2 * Math.PI)) % 1) + 1) % 1 * side.period;
      side.L.time = tau(sim.modelTheta.L);
      side.R.time = tau(sim.modelTheta.R);
      l.mixer.update(0);
    }
    view.update(m, s, sim.data.com, sim, opts.support);
    const heading = sim.yaw + (sim.forward < 0 ? Math.PI : 0);
    if (opts.camera !== 'free') view.follow(stage.camera, stage.controls, m, heading, opts.camera === 'chase', followScale(stage));
    if (performance.now() - st.hudAt > 120) { st.hudAt = performance.now(); updateModelHud(sim, hud, warn, spark, opts.plot); }
  }

  input.onChange = ph.sendCommand;
  addEventListener('visibilitychange', ph.sendCommand);

  return {
    get active() { return opts.drive; },
    get preview() { return preview; },
    get lastWalk() { return t.lastWalk; },
    get lastPred() { return st.lastPred; },
    /** The query the glb on screen was baked with (what physics simulates). */
    get glbQuery() { return st.glbQuery; },
    get physicsOn() { return ph.on; },
    opts, sim, view, tune, design, hud, gui, tuneGui,
    model: { parseDrive, evaluate, straightWalk },
    setDrive, setPreview, setPhysics, frame, physics, physicsCommand: ph.physicsCommand,
    baseQuery: t.baseQuery,
    loadDesign: t.loadDesign,
    /** A new glb is on screen, baked with ``query``: split its clip by side; keep driving if it
     * can be driven, and if physics was on, reconnect it for this design. */
    async onLoad(l: LoadedScene, query = ''): Promise<void> {
      st.engaged = false;
      st.glbQuery = query;
      st.lastPred = null;
      const side = (st.side = sideActions(l.mixer, l.clip, l.walker));
      saved = { p: l.walker.position.clone(), q: l.walker.quaternion.clone() };
      const wantPhysics = ph.on || ph.wanted;
      // The session, live or still building, is the previous glb's: off, through the one off path
      // (a connect in flight rejects with null and setPhysics's loop reconnects for this one).
      if (ph.on) ph.physicsOff();
      else if (wantPhysics) physics.close();
      if (!side.usable && opts.drive) await setDrive(false);
      else if (opts.drive && !preview) await bindGlb();
      else if (preview) l.root.visible = false;
      if (wantPhysics && side.usable && opts.drive) {
        ph.wanted = true;
        // Not awaited: the in-flight setPhysics may itself be waiting on this load (physics
        // asked while a mechanism was on screen loads the robot first).
        void setPhysics(true).catch(fail);
      }
      syncUrl();    // the deep link follows the design on screen, physics or not
    },
    async init(): Promise<void> {
      await t.loadLinkages().catch(fail);
      if (url.get('tune') === '1') await setPreview(true);
      else if (url.get('drive') === '1') await setDrive(true);
      if (url.get('physics') === '1') await setPhysics(true).catch(fail);
    },
  };
}

export type Drive = ReturnType<typeof createDrive>;
