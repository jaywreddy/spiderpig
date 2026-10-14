/**
 * The tune panel (``index.ts``'s left lil-gui): the design on screen or being tuned (linkage,
 * module, phases, proportions; a stored design's id), its instant stick-figure preview from
 * ``/api/walk``, and the query strings ``/api/walk`` and ``/api/glb`` take for it.
 */
import GUI from 'three/examples/jsm/libs/lil-gui.module.min.js';
import type { DriveHost } from './index';
import { parseDrive, straightWalk, type DriveData, type Side, type WalkJson } from './model';
import { f, fmt, MIN_MARGIN_MM } from './hud';
import { groupSelect, linkageGroups, linkageOptions } from '../modes';
import type { DriveView } from './view';

export interface WalkResponse extends WalkJson {
  valid: boolean; error: string | null; linkage: string; module: string; phases_deg: number[];
  proportions: Record<string, number>; metrics?: Record<string, unknown>;
  legs: { leg: number; joints: Record<string, [number, number][]> }[]; links: [string, string][];
  side_z?: Partial<Record<Side, number>>;
}

/** ``/api/design/{id}``: a stored design's card (``spiderpig view``). */
export interface DesignCard {
  design: string; kind: 'walker' | 'mechanism' | 'arm'; linkage: string; module: string; sides: number;
  mode: string; phases_deg: number[]; params: Record<string, number>; servo: string;
}

/** One entry of ``/api/linkages``. */
export interface LinkageInfo {
  key: string; name: string; family: string; notes: string; source: string;
  params: { name: string; default: number; angle: boolean }[];
  modules: Record<string, number>; default_module: string; labels: Record<string, string>; feet: number;
  kind: 'walker' | 'mechanism' | 'arm'; output: { kind: string; name: string; motion: string } | null;
}

/** The glb root's design (``bake.py`` writes ``linkage``/``module`` on the root and the
 * walker's ``drive.params``: ``config.design_json()``). */
export interface BakedDesign { linkage?: string; module?: string; phases_deg?: number[]; proportions?: Record<string, number> }

/** What the server builds for a query naming no linkage or module (``linkage.DEFAULT``,
 * ``config.default_module``): the first load's assumption until ``/api/linkages`` answers. */
export const SERVER_DEFAULT = { linkage: 'strider', module: 'double' };

export async function getJson<T>(url: string, signal?: AbortSignal): Promise<T> {
  const res = await fetch(url, { signal });
  if (res.ok) return (await res.json()) as T;
  const d = ((await res.json().catch(() => ({}))) as { detail?: unknown }).detail;
  throw new Error(typeof d === 'string' ? d : `${res.status} ${JSON.stringify(d ?? res.statusText)}`);
}

export const paramKeys = (q: URLSearchParams): string[] => {
  const k: string[] = [];
  q.forEach((_, n) => k.push(n));
  return k;
};

/** What the tune panel needs of the drive (``index.ts``). */
export interface TuneHost {
  host: DriveHost;
  view: DriveView;
  url: URLSearchParams;
  fail(e: unknown): void;
  /** Is the stick preview on? */
  preview(): boolean;
  setPreview(on: boolean): Promise<void>;
  /** ``host.loadRobot`` with the load marked in flight. */
  loadRobot(query: string): Promise<void>;
  /** The walking model driven by ``data``: its straight-walk metrics. */
  useData(data: DriveData): Record<string, unknown>;
  syncUrl(): void;
}

export function createTune(ctx: TuneHost) {
  const { host, view, url, fail } = ctx;
  const design = {
    id: url.get('design') as string | null,   // a stored design: every query carries it
    linkage: url.get('linkage') ?? SERVER_DEFAULT.linkage,
    module: url.get('module') ?? '',          // '': the linkage's own default (below)
    phases: (url.get('phases')?.split(',').map(Number) ?? null) as number[] | null,
    props: Object.fromEntries(paramKeys(url).filter((k) => k.startsWith('p.'))
      .map((k) => [k.slice(2), Number(url.get(k))])) as Record<string, number>,
  };
  // every registered linkage (/api/linkages), in the one linkage dropdown grouped by kind
  let linkages: LinkageInfo[] | null = null, defaultLinkage = SERVER_DEFAULT.linkage;
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
  tuneGui.add(tune, 'preview').name('stick preview').onChange((on: boolean) => void ctx.setPreview(on).catch(fail));
  const designF = tuneGui.addFolder('design');           // linkage, module
  const phaseF = tuneGui.addFolder('phases (°)');
  const propF = tuneGui.addFolder('parameters (lengths ×0.5–1.5, angles ±45°)');
  tuneGui.add(tune, 'rebuild').name('Rebuild parts');
  tuneGui.add(tune, 'reset').name('Reset to defaults');
  tuneGui.add(tune, 'status').disable().listen();
  const metricF = tuneGui.addFolder('metrics (per revolution)');
  let moduleC: ReturnType<GUI['add']> | null = null, linkageC: ReturnType<GUI['add']> | null = null;
  let lastWalk: WalkResponse | null = null;

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

  /** The registered linkages (once): the one linkage dropdown (every kind, grouped by kind:
   * walkers, mechanisms, arms) and the design controls. A linkage picked loads in its own
   * default view, the view picker offering what it declares (``host.loadLinkage``). */
  async function loadLinkages(): Promise<void> {
    if (linkages) return;
    const r = await getJson<{ default: string; linkages: LinkageInfo[] }>('/api/linkages');
    [linkages, defaultLinkage] = [r.linkages, r.default];
    if (!info()) design.linkage = r.default;
    // a module guessed before the catalogue came (and not changed since): the catalogue's
    if (guessedModule !== null && design.module === guessedModule) design.module = info()!.default_module;
    design.props = { ...defaults(), ...design.props };     // the URL's p.NAME win
    const groups = linkageGroups(linkages);
    linkageC = designF.add(design, 'linkage', linkageOptions(groups))
      .onChange(() => queueMicrotask(() => void switchLinkage()));
    const select = linkageC.domElement.querySelector('select');
    if (select) groupSelect(select, groups);
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

  /** Another linkage, of any kind: its defaults, its modules (keeping this one if it has it),
   * and its model loaded; the view picker then offers what it declares (``/api/modes``: a
   * walker's robot and modules, an arm's robot and side, a mechanism's side). One with feet
   * walks: its robot re-baked with the tune panel's design, to drive (``rebuild``); one
   * without loads in its own default view. A load that fails puts the panel back to the
   * design on screen. */
  async function switchLinkage(): Promise<void> {
    const lk = info()!;
    if (!Object.keys(lk.modules).includes(design.module)) design.module = lk.default_module;
    design.phases = null;
    design.props = defaults();
    buildDesign();
    if (ctx.preview()) request(0);
    if (lk.feet > 0) return rebuild();
    tune.status = `loading ${lk.key}…`;
    try {
      await host.loadLinkage(lk.key);
      tune.status = `${lk.key}: its own views (no feet: nothing to drive)`;
    } catch (e) {
      adoptLoaded();
      fail(new Error(`load failed: ${(e as Error).message}`));
    }
    ctx.syncUrl();
  }

  let timer = 0, inflight: AbortController | null = null;
  function request(delay = 150): void {
    clearTimeout(timer);
    if (info()?.feet === 0) {          // no feet: no walk to preview
      tune.status = `${design.linkage} has no feet: the stick preview is a walker's`;
      return;
    }
    timer = window.setTimeout(() => {
      inflight?.abort();
      inflight = new AbortController();
      tune.status = 'computing…';
      getJson<WalkResponse>(`/api/walk?${designQuery()}`, inflight.signal).then(applyWalk).catch((e: Error) => {
        if (e.name !== 'AbortError') tune.status = `error: ${e.message}`;
      });
    }, delay);
    if (!ctx.preview()) void ctx.setPreview(true).catch(fail);
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
    const data = parseDrive(w), pred = ctx.preview() ? ctx.useData(data) : straightWalk(data);
    metricF.controllers.slice().forEach((c) => c.destroy());
    const shown = Object.fromEntries(Object.entries(w.metrics ?? pred).map(([k, v]) => [k, fmt(v)]));
    for (const k of Object.keys(shown)) metricF.add(shown, k).disable();
    const margin = (w.metrics?.min_margin_mm ?? pred.min_margin_mm) as number;
    const walks = (w as { walks?: boolean }).walks ?? (w.metrics?.walks as boolean | undefined) ?? true;
    tune.status = `ok · ${w.linkage} ${w.module}${w.metrics ? '' : ' (metrics: viewer model)'}`
      + (!walks ? ' · ⚠ DOES NOT WALK (no stride, or on < 3 feet)'
        : margin < MIN_MARGIN_MM ? ` · ⚠ margin ${margin.toFixed(1)} mm: may tip (MuJoCo decides)` : '');
    ctx.syncUrl();
  }

  /** Re-bake the robot for the tune panel's design; the glb that lands sets ``glbQuery``
   * (``onLoad``). When the bake fails the panel goes back to the design on screen. */
  async function rebuild(): Promise<void> {
    const q = designQuery(), t0 = performance.now();
    const tick = window.setInterval(() => {
      tune.status = `baking parts… ${f((performance.now() - t0) / 1000, 0)} s`;
    }, 500);
    try {
      await ctx.loadRobot(q);
      await ctx.setPreview(false);
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
    if (ctx.preview()) request(0);
    ctx.syncUrl();
  }

  function resetDefaults(): void {
    Object.assign(design.props, defaults());
    design.phases = null;
    propF.controllers.forEach((c) => c.updateDisplay());
    request(0);
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

  return {
    tune, tuneGui, design, defaultModule, designQuery, baseQuery, loadDesign, loadLinkages, request,
    get defaultLinkage() { return defaultLinkage; },
    get lastWalk() { return lastWalk; },
  };
}
