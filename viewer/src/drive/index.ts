/**
 * Drive mode and the tune panel (lil-gui): drive the robot tank- or
 * arcade-style over a ground plane with the SPEC walking model, and tune its
 * design with an instant stick-figure preview from ``/api/walk``.
 *
 * Walking data comes from the glb's ``walker`` extras ``drive`` or, when the
 * glb has none, from ``/api/walk`` for the glb's design.
 */
import type { Quaternion, Vector3 } from 'three';
import GUI from 'three/examples/jsm/libs/lil-gui.module.min.js';
import type { LoadedScene } from '../loader';
import type { Stage } from '../scene';
import { evaluate, parseDrive, straightWalk, type DriveData, type Side, type WalkJson } from './model';
import { Input, Sim, type Sample, type Scheme } from './sim';
import { DriveView, bodyMatrix, sideActions } from './view';

export interface DriveHost {
  stage: Stage;
  loaded(): LoadedScene | null;
  /** Load the robot baked with design ``query`` ('' = default); rejects with the server's detail. */
  loadRobot(query: string): Promise<void>;
  status(text: string): void;
  seek(t: number): void;
  reframe(): void;
}

interface WalkResponse extends WalkJson {
  valid: boolean; error: string | null; module: string; phases_deg: number[];
  proportions: Record<string, number>; metrics?: Record<string, unknown>;
  legs: { leg: number; joints: Record<string, [number, number][]> }[]; links: [string, string][];
  side_z?: Partial<Record<Side, number>>;
}

const MODULES = ['single', 'double', 'decker', 'quad'];
const PLOTS: (keyof Sample)[] = ['speed', 'yawRate', 'height', 'pitch', 'roll', 'slip', 'margin'];
const HUD_ROWS = ['speed', 'yaw rate', 'height', 'pitch', 'roll', 'contacts', 'slip', 'margin', 'cranks',
  'rev distance', 'rev turn', 'rev bob', 'rev pitch', 'rev roll', 'rev slip', 'rev margin', 'model'];

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

export function createDrive(host: DriveHost) {
  const { stage } = host;
  const sim = new Sim();
  const view = new DriveView(stage.scene);
  const url = new URLSearchParams(location.search);
  const opts = {
    drive: false, scheme: (url.get('scheme') === 'arcade' ? 'arcade' : 'tank') as Scheme,
    speed: 1, accel: 3, phase: 0, camera: 'chase', support: true, plot: 'speed' as keyof Sample,
    reset: () => { sim.reset(); view.clearTrail(); },
  };
  const input = new Input(() => opts.drive);
  let side: ReturnType<typeof sideActions> | null = null;
  let saved: { p: Vector3; q: Quaternion } | null = null;
  let engaged = false, preview = false, glbQuery = '', lastWalk: WalkResponse | null = null;

  // --- GUI: drive options + HUD (right), tune (left) ------------------------
  const gui = new GUI({ title: 'Drive', width: 300 });
  gui.domElement.id = 'drive-gui';
  gui.add(opts, 'drive').name('drive mode').onChange((on: boolean) => void setDrive(on).catch(fail));
  gui.add(opts, 'scheme', ['tank', 'arcade']).name('controls').onChange(syncUrl);
  gui.add(opts, 'speed', 0.1, 1, 0.05).name('speed (× max rpm)').onChange((v: number) => { sim.speedScale = v; });
  gui.add(opts, 'accel', 0.5, 20, 0.5).name('accel (max/s)').onChange((v: number) => { sim.accel = v; });
  gui.add(opts, 'phase', -180, 180, 1).name('R − L phase (°)')
    .onChange((v: number) => { sim.phaseOffset = (v * Math.PI) / 180; });
  gui.add(opts, 'camera', ['chase', 'follow', 'free']);
  gui.add(opts, 'support').name('support + COM');
  gui.add(opts, 'reset');
  const hud: Record<string, string> = Object.fromEntries(HUD_ROWS.map((k) => [k, '–']));
  const hudF = gui.addFolder('HUD');
  for (const k of HUD_ROWS) hudF.add(hud, k).disable().listen();
  hudF.add(opts, 'plot', PLOTS);
  const spark = Object.assign(document.createElement('canvas'), { width: 290, height: 56, id: 'drive-spark' });
  hudF.$children.append(spark);
  const warn = Object.assign(document.createElement('div'), { id: 'drive-warn', hidden: true });
  document.body.append(warn);

  const design = {
    module: url.get('module') ?? 'quad',
    phases: (url.get('phases')?.split(',').map(Number) ?? null) as number[] | null,
    props: Object.fromEntries(paramKeys(url).filter((k) => k.startsWith('p.'))
      .map((k) => [k.slice(2), Number(url.get(k))])) as Record<string, number>,
  };
  let klann: Record<string, number> | null = null;       // Klann's proportions, from the server
  const tune = { preview: false, status: '', rebuild: () => void rebuild(), klann: () => resetKlann() };
  const tuneGui = new GUI({ title: 'Tune (instant preview)', width: 300, autoPlace: false }).close();
  tuneGui.domElement.id = 'tune-gui';
  document.body.append(tuneGui.domElement);
  tuneGui.add(tune, 'preview').name('stick preview').onChange((on: boolean) => void setPreview(on).catch(fail));
  tuneGui.add(design, 'module', MODULES).onChange(() => { design.phases = null; request(); });
  const phaseF = tuneGui.addFolder('phases (°)');
  const propF = tuneGui.addFolder('proportions (±30 % of Klann)');
  tuneGui.add(tune, 'rebuild').name('Rebuild parts');
  tuneGui.add(tune, 'klann').name('Reset to Klann');
  tuneGui.add(tune, 'status').disable().listen();
  const metricF = tuneGui.addFolder('metrics (per revolution)');

  function fail(e: unknown): void {
    tune.status = (e as Error).message;
    host.status(`drive: ${tune.status}`);
  }

  // --- data, drive on / off, preview on / off ---------------------------------
  function useData(data: DriveData): Record<string, unknown> {
    const pred = straightWalk(data);
    sim.setData(data, (pred.stride_signed_mm as number) < 0 ? -1 : 1);
    hud.model = `${fmt(pred.stride_mm)} mm/rev · ${fmt(pred.speed_mm_s)} mm/s`;
    return pred;
  }

  /** Drive the full model: its extras, else ``/api/walk`` for its design. */
  async function bindGlb(): Promise<void> {
    const l = host.loaded();
    if (!l || !side?.usable) return;
    const extras = l.walker.userData.drive as WalkJson | undefined;
    const module = (l.root.userData.module as string | undefined) ?? 'quad';
    useData(parseDrive(extras ?? await getJson<WalkResponse>(`/api/walk?${glbQuery || `module=${module}`}`)
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

  async function setDrive(on: boolean): Promise<void> {
    opts.drive = on;
    try {
      if (on && !preview && side?.usable) await bindGlb();
      else if (on && !preview) await host.loadRobot(glbQuery);   // its onLoad binds it
    } catch (e) {
      opts.drive = false;
      gui.controllers[0]!.updateDisplay();
      throw e;
    }
    if (!on) { await setPreview(false); release(); host.reframe(); }
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

  async function setPreview(on: boolean): Promise<void> {
    if (on === preview) return;
    preview = tune.preview = on;
    tuneGui.controllers[0]!.updateDisplay();
    const l = host.loaded();
    view.stick.visible = on;
    if (l) l.root.visible = !on;
    if (on) {
      tuneGui.open();
      if (!opts.drive) await setDrive(true);
      if (klann) request(0); else await start();
    } else if (opts.drive) {
      await bindGlb();
    }
    syncUrl();
  }

  // --- tune ------------------------------------------------------------------
  function designQuery(): string {
    const q = new URLSearchParams({ module: design.module });
    if (design.phases) q.set('phases', design.phases.map((v) => +v.toFixed(2)).join(','));
    for (const [k, v] of Object.entries(design.props)) {
      if (!klann || Math.abs(v - (klann[k] ?? v)) > 1e-9) q.set(`p.${k}`, String(+v.toPrecision(6)));
    }
    return q.toString();
  }

  /** First use: Klann's proportions (the defaults and slider ranges) from a plain ``/api/walk``. */
  async function start(): Promise<void> {
    klann = (await getJson<WalkResponse>(`/api/walk?module=${design.module}`)).proportions;
    design.props = { ...klann, ...design.props };
    for (const [k, v] of Object.entries(klann)) {
      propF.add(design.props, k, v - 0.3 * Math.abs(v), v + 0.3 * Math.abs(v), Math.abs(v) / 1000)
        .name(`${k} (${v})`).onChange(() => request());
    }
    request(0);
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
    tune.status = `ok · ${w.module}${w.metrics ? '' : ' (metrics: viewer model)'}`;
    syncUrl();
  }

  async function rebuild(): Promise<void> {
    const q = designQuery(), prev = glbQuery, t0 = performance.now();
    const tick = window.setInterval(() => {
      tune.status = `baking parts… ${f((performance.now() - t0) / 1000, 0)} s`;
    }, 500);
    try {
      glbQuery = q;
      await host.loadRobot(q);
      await setPreview(false);
      tune.status = 'parts rebuilt';
    } catch (e) {
      glbQuery = prev;
      fail(new Error(`rebuild failed: ${(e as Error).message}`));
    } finally {
      clearInterval(tick);
    }
  }

  function resetKlann(): void {
    if (!klann) return;
    Object.assign(design.props, klann);
    design.phases = null;
    propF.controllers.forEach((c) => c.updateDisplay());
    request(0);
  }

  /** Cheap deep links: ``drive=1``, ``scheme=arcade``, ``tune=1`` + the design (``module``, ``phases``, ``p.NAME``). */
  function syncUrl(): void {
    const q = new URLSearchParams(location.search);
    for (const k of paramKeys(q)) if (/^(drive|scheme|tune|module|phases|p\..*)$/.test(k)) q.delete(k);
    if (opts.drive) q.set('drive', '1');
    if (opts.scheme !== 'tank') q.set('scheme', opts.scheme);
    if (preview) { q.set('tune', '1'); new URLSearchParams(designQuery()).forEach((v, k) => q.set(k, v)); }
    history.replaceState(null, '', `?${q.toString()}`.replace(/%2C/g, ','));
  }

  // --- per frame -------------------------------------------------------------
  let hudAt = 0;
  function frame(dt: number): void {
    const [left, right] = input.read(opts.scheme);
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
    if (opts.camera !== 'free') view.follow(stage.camera, stage.controls, m, heading, opts.camera === 'chase');
    if (performance.now() - hudAt > 120) { hudAt = performance.now(); updateHud(); }
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
    opts, sim, view, tune, design, hud, gui, tuneGui,
    model: { parseDrive, evaluate, straightWalk },
    setDrive, setPreview, frame,
    /** A new glb is on screen: split its clip by side; keep driving if it can be driven. */
    async onLoad(l: LoadedScene): Promise<void> {
      engaged = false;
      side = sideActions(l.mixer, l.clip, l.walker);
      saved = { p: l.walker.position.clone(), q: l.walker.quaternion.clone() };
      if (!side.usable && opts.drive) await setDrive(false);
      else if (opts.drive && !preview) await bindGlb();
      else if (preview) l.root.visible = false;
    },
    async init(): Promise<void> {
      if (url.get('tune') === '1') await setPreview(true);
      else if (url.get('drive') === '1') await setDrive(true);
    },
  };
}

export type Drive = ReturnType<typeof createDrive>;
