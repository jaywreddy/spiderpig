/**
 * The decanting cradle page (``decant.html``): pick a design, watch it pour. The baked
 * cradle in 3D (``/api/glb/klann?linkage=<key>``: the whole cradle, the bottle and the
 * decanter) beside its mechanism in the linkage's plane (``/api/cradle/<key>``: SVG,
 * ``diagram.ts``), both posed from one clip fraction ``f`` (0: the crank at 0; the clip is
 * one crank turn). The slider and the readouts speak in handle turns from rest and the
 * bottle's tilt. Deep links: ``?design=decanter_rcm&turns=7.5&view=side``.
 */
import * as THREE from 'three';
import './decant.css';
import { createStage, frameView } from '../scene';
import { loadGlb, teardown, type LoadedScene } from '../loader';
import { isAbort } from '../loads';
import type { View } from '../types';
import { COLORS, createDiagram, type Diagram } from './diagram';
import { createPack, type Pack, type PackData } from './pack';
import { createPour } from './pour';
import {
  fractionAtTurns, phaseAt, tiltAt, tiltWords, turnsAt, type CradleMotion,
} from './motion';

interface CradleCard { key: string; name: string; notes: string; verdict: 'build' | 'study'; why: string }

const $ = <T extends HTMLElement = HTMLElement>(id: string): T => document.getElementById(id) as T;
const canvas = $<HTMLCanvasElement>('stage');
const stage = createStage(canvas);
stage.grid.visible = true;
const svg = document.getElementById('diagram') as unknown as SVGSVGElement;
const slider = $<HTMLInputElement>('slider');
const play = $<HTMLButtonElement>('play');
const speedSel = $<HTMLSelectElement>('speed');
const viewSel = $<HTMLSelectElement>('view');

const params = new URLSearchParams(location.search);
const VIEWS: readonly View[] = ['three-quarter', 'side', 'front', 'top'];
let view: View = VIEWS.includes(params.get('view') as View) ? params.get('view') as View : 'three-quarter';
viewSel.value = view;

let cards: CradleCard[] = [];
let motion: CradleMotion | null = null;
let diagram: Diagram | null = null;
let loaded: LoadedScene | null = null;
let f = 0;                    // the clip fraction both views are posed at
let playing = true;
let turnsPerSecond = Number(speedSel.value) || 1;
let dirty = true;
let ticket = 0;               // the latest design asked for (an older load is dropped)
let packData: PackData | null = null;   // the flat pack (fetched on first use)
let pack: Pack | null = null;
let packS = 0;                // 0 assembled .. 1 packed
let packTarget = 0;
let abort: AbortController | null = null;
const pour = createPour(stage, () => { dirty = true; });   // the look, the wine, the vessel

// -- layout: the side panel, the canvas beside it, the dock under it --------------------

const NARROW = '(max-width: 820px)';
let sideOpen = !matchMedia(NARROW).matches;

const DIAG_W = 460;

function layout(): void {
  document.body.classList.toggle('side-open', sideOpen);
  const narrow = matchMedia(NARROW).matches;
  const left = sideOpen && !narrow ? $('side').getBoundingClientRect().width : 0;
  const right = narrow || $('diagram-panel').classList.contains('collapsed') ? 0 : DIAG_W;
  const root = document.documentElement.style;
  root.setProperty('--side-w', `${left}px`);
  root.setProperty('--diag-w', `${right}px`);
  const dock = $('dock').getBoundingClientRect();
  root.setProperty('--dock-h', `${Math.ceil(dock.height)}px`);
  const w = Math.max(1, window.innerWidth - left - right), h = window.innerHeight;
  stage.renderer.setSize(w, h, false);
  stage.camera.aspect = w / h;
  // on a phone the diagram sits over the canvas's lower half: frame the cradle above it
  const under = narrow ? Math.min(dock.top, $('diagram-panel').getBoundingClientRect().top) : dock.top;
  stage.setInsets({ top: narrow ? 56 : 0, bottom: Math.max(0, h - under) });
  dirty = true;
}

/** Frame the camera on the cradle's whole swing (posed at rest, mid-way and at full tilt),
 * not one pose: the bottle's base sweeps far up at full tilt. */
function reframe(): void {
  if (pack && packS > 0) {     // packed (or on the way): the box, and the clip left alone
    frameBox();
    dirty = true;
    return;
  }
  if (loaded && motion) {
    const box = new THREE.Box3();
    for (const turns of [0, motion.turns_to_full / 2, motion.turns_to_full, motion.ratio * 0.75]) {
      loaded.action.time = fractionAtTurns(motion, turns) * loaded.clipDuration;
      loaded.mixer.update(0);
      loaded.root.updateMatrixWorld(true);
      box.union(new THREE.Box3().setFromObject(loaded.root));
    }
    const size = box.getSize(new THREE.Vector3()), center = box.getCenter(new THREE.Vector3());
    const proxy = new THREE.Mesh(new THREE.BoxGeometry(size.x, size.y, size.z));
    proxy.position.copy(center);
    frameView(stage, proxy, view, 1.04);
    proxy.geometry.dispose();
  }
  dirty = true;      // (the frame loop poses it back at ``f``)
}
window.addEventListener('resize', layout);
stage.controls.addEventListener('change', () => { dirty = true; });
$('side-toggle').addEventListener('click', () => {
  sideOpen = !sideOpen;
  layout();
  reframe();
});

// -- the designs ---------------------------------------------------------------------------

function renderCards(selected: string): void {
  const box = $('designs');
  box.replaceChildren();
  for (const c of cards) {
    const b = document.createElement('button');
    b.className = `card${c.key === selected ? ' selected' : ''}`;
    b.innerHTML = `<span class="chip ${c.verdict}">${c.verdict === 'build' ? 'to build' : 'study'}</span>`
      + `<strong></strong><small></small>`;
    b.querySelector('strong')!.textContent = c.name;
    b.querySelector('small')!.textContent = c.notes;
    b.addEventListener('click', () => { void selectDesign(c.key); });
    box.appendChild(b);
  }
}

function renderAbout(m: CradleMotion): void {
  const about = $('about');
  const rows: [string, string][] = [
    ['Tilt', `${m.rest_deg.toFixed(0)}° at rest (${tiltWords(m.rest_deg)}) to +${m.full_deg.toFixed(0)}° at full tilt`],
    ['Handle', `${m.turns_to_full.toFixed(1)} turns rest → full tilt, ${m.ratio} a whole cycle (a ${m.ratio}:1 worm: it holds wherever you stop)`],
    ['Mouth', `moves ${m.mouth_wander_mm.toFixed(2)} mm over the swing: the pour stays over the decanter`],
    ['At a turn a second', `${Math.round(m.turns_to_full)} s from rest to full tilt`],
  ];
  const legend: [string, string][] = [
    [COLORS.cradle, m.labels[m.output_link ?? ''] ?? 'the cradle link'],
    [COLORS.crank, 'the crank (on the worm box)'],
    ...Object.entries(m.labels).filter(([b]) => b !== m.output_link)
      .map(([, label]) => [COLORS.link, label] as [string, string]),
    [COLORS.bottle, `the bottle (${m.bottle.name})`],
    [COLORS.saddle, 'the saddles'],
    [COLORS.mouth, 'the mouth: the point it all turns about'],
  ];
  about.replaceChildren();
  const why = document.createElement('p');
  why.className = `why ${m.verdict}`;
  why.textContent = m.why;
  about.appendChild(why);
  const dl = document.createElement('dl');
  for (const [k, v] of rows) {
    const dt = document.createElement('dt'); dt.textContent = k;
    const dd = document.createElement('dd'); dd.textContent = v;
    dl.append(dt, dd);
  }
  about.appendChild(dl);
  const h = document.createElement('h3'); h.textContent = 'In the diagram';
  const ul = document.createElement('ul'); ul.className = 'legend';
  for (const [color, label] of legend) {
    const li = document.createElement('li');
    li.innerHTML = `<i style="background:${color}"></i>`;
    li.appendChild(document.createTextNode(label));
    ul.appendChild(li);
  }
  const links = document.createElement('p');
  links.className = 'links';
  links.innerHTML = `<a href="/?linkage=${encodeURIComponent(m.key)}&mode=klann" target="_blank">Open in the parts viewer ↗</a>`;
  if (m.source.startsWith('http')) {
    links.innerHTML += ` · <a href="${m.source}" target="_blank" rel="noopener">Reference ↗</a>`;
  }
  about.append(h, ul, links);
  $('diagram-title').textContent = `${m.name}: one side, in its plane`;
}

async function getJson<T>(url: string, signal?: AbortSignal): Promise<T> {
  const res = await fetch(url, { signal });
  if (!res.ok) {
    const detail = ((await res.json().catch(() => ({}))) as { detail?: unknown }).detail;
    throw new Error(typeof detail === 'string' ? detail : `${res.status} ${res.statusText}`);
  }
  return (await res.json()) as T;
}

function status(text: string): void {
  $('status').textContent = text;
  $('status').hidden = !text;
}

async function selectDesign(key: string, turns?: number): Promise<void> {
  const mine = ++ticket;
  abort?.abort();
  abort = new AbortController();
  const signal = abort.signal;
  renderCards(key);
  resetPack();
  const url = new URL(location.href);
  url.searchParams.set('design', key);
  history.replaceState(null, '', url);
  try {
    const m = await getJson<CradleMotion>(`/api/cradle/${encodeURIComponent(key)}`, signal);
    if (mine !== ticket) return;
    motion = m;
    diagram = createDiagram(svg, m);
    renderAbout(m);
    slider.max = String(m.ratio);
    const fullAt = (m.turns_to_full / m.ratio) * 100;
    $('tick-full').style.left = `${fullAt}%`;
    f = fractionAtTurns(m, turns ?? 0);
    dirty = true;
    pour.detach();
    teardown(stage.scene, loaded);
    loaded = null;
    status(`baking ${m.name} in 3D… (the first time takes a minute or two; the diagram works now)`);
    const next = await loadGlb(stage.scene, 'klann', `linkage=${encodeURIComponent(key)}`, signal);
    if (mine !== ticket) { teardown(stage.scene, next); return; }
    loaded = next;
    next.action.paused = true;     // posed from ``f`` every frame
    pour.attach(next, m);
    reframe();
    status('');
    dirty = true;
    // the room under the mouth (a fabrication: a minute the first time), then the riser
    void getJson<{ room: [number, number][]; room_pouring: [number, number][] }>(
      `/api/cradle/${encodeURIComponent(key)}/room`, signal).then((r) => {
      if (mine === ticket && loaded === next) {
        pour.setRoom(r.room, r.room_pouring);
        if (packTarget > 0 || packS > 0) pour.setVisible(false);   // (packed meanwhile)
        dirty = true;
      }
    }).catch((err: unknown) => { if (!isAbort(err)) console.error(err); });
  } catch (err) {
    if (isAbort(err)) return;
    console.error(err);
    status(`could not load ${key}: ${(err as Error).message}`);
  }
}

// -- the flat pack ------------------------------------------------------------------------

function resetPack(): void {
  pack?.dispose();
  pack = null;
  packData = null;
  packS = 0;
  packTarget = 0;
  $('pack-toggle').textContent = 'Flat pack';
  $('pack-info').hidden = true;
}

function renderPackInfo(d: PackData): void {
  const info = $('pack-info');
  const [x, y, z] = d.box.outside_mm.map((v) => Math.round(v)) as [number, number, number];
  info.replaceChildren();
  const h = document.createElement('h3'); h.textContent = 'Flat pack';
  const size = document.createElement('p');
  size.className = 'pack-size';
  size.textContent = `One box, ${x} × ${z} × ${y} mm outside (${d.box.volume_l} L).`;
  const how = document.createElement('p');
  how.textContent = 'It comes apart where it is already screwed together, in this order:';
  const ol = document.createElement('ol');
  for (const step of d.steps) {
    const li = document.createElement('li');
    li.textContent = step;
    ol.appendChild(li);
  }
  // the pieces as they go in the box: what is under another goes in first
  const h2 = document.createElement('h3'); h2.textContent = `In the box, from the floor up (${d.modules.length} pieces)`;
  const pieces = document.createElement('ol');
  pieces.className = 'pack-pieces';
  for (const m of d.modules) {
    const li = document.createElement('li');
    li.textContent = m.label;
    li.title = m.how;
    pieces.appendChild(li);
  }
  info.append(h, size, how, ol, h2, pieces);
  info.hidden = false;
}

/** How long the pieces take to fly into the box (or back), seconds: longer for more pieces. */
function packSeconds(): number {
  return Math.max(3, 0.3 * (packData?.modules.length ?? 0));
}

async function togglePack(): Promise<void> {
  if (!motion || !loaded) return;
  if (packTarget > 0) {           // back to the assembled cradle
    packTarget = 0;
    $('pack-toggle').textContent = 'Flat pack';
    return;
  }
  const mine = ticket;
  if (!packData) {
    status('working out the flat pack… (the first time takes a minute)');
    try {
      packData = await getJson<PackData>(`/api/cradle/${encodeURIComponent(motion.key)}/pack`);
    } catch (err) {
      status(`could not pack it: ${(err as Error).message}`);
      return;
    }
    status('');
    if (mine !== ticket || !loaded) return;
  }
  if (!pack) {
    loaded.action.time = 0;         // the transforms start from the clip's first frame
    loaded.mixer.update(0);
    pack = createPack(loaded.walker, packData, ['bottle', 'decanter', 'decanter_riser']);
  }
  renderPackInfo(packData);
  setPlaying(false);
  pour.setVisible(false);
  packTarget = 1;
  $('pack-toggle').textContent = 'Assemble';
}

/** Frame the camera on the box once it's packed. */
function frameBox(): void {
  if (!pack) return;
  const b = pack.bounds();
  const size = b.getSize(new THREE.Vector3()), centre = b.getCenter(new THREE.Vector3());
  const proxy = new THREE.Mesh(new THREE.BoxGeometry(size.x, size.y, size.z));
  proxy.position.copy(centre);
  frameView(stage, proxy, view, 1.1);
  proxy.geometry.dispose();
}

// -- the controls ------------------------------------------------------------------------

function setPlaying(p: boolean): void {
  playing = p;
  play.textContent = p ? '❚❚' : '▶';
  play.title = p ? 'Pause' : 'Play';
}
play.addEventListener('click', () => setPlaying(!playing));
$('pack-toggle').addEventListener('click', () => { void togglePack(); });
speedSel.addEventListener('change', () => { turnsPerSecond = Number(speedSel.value) || 1; });
slider.addEventListener('input', () => {
  if (!motion) return;
  setPlaying(false);
  f = fractionAtTurns(motion, Number(slider.value));
  dirty = true;
});
$('to-rest').addEventListener('click', () => {
  if (!motion) return;
  setPlaying(false);
  f = fractionAtTurns(motion, 0);
  dirty = true;
});
$('to-full').addEventListener('click', () => {
  if (!motion) return;
  setPlaying(false);
  f = fractionAtTurns(motion, motion.turns_to_full);
  dirty = true;
});
viewSel.addEventListener('change', () => {
  view = viewSel.value as View;
  reframe();
});
$('diagram-toggle').addEventListener('click', () => {
  const panel = $('diagram-panel');
  panel.classList.toggle('collapsed');
  $('diagram-toggle').textContent = panel.classList.contains('collapsed') ? 'show' : 'hide';
  layout();
  reframe();
});
window.addEventListener('keydown', (e) => {
  if (e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement) return;
  if (e.key === ' ') { e.preventDefault(); setPlaying(!playing); }
});

// -- the frame loop ------------------------------------------------------------------------

function readouts(m: CradleMotion): void {
  const tilt = tiltAt(m, f), turns = turnsAt(m, f);
  $('tilt-value').textContent = `${tilt >= 0 ? '+' : '−'}${Math.abs(tilt).toFixed(1)}°`;
  $('tilt-words').textContent = tiltWords(tilt);
  $('turns-value').textContent = `${turns.toFixed(1)} / ${m.ratio}`;
  const phase = phaseAt(m, f);
  const left = phase === 'tipping' ? m.turns_to_full - turns : m.ratio - turns;
  $('phase').textContent = phase === 'tipping'
    ? `tipping: ${left.toFixed(1)} turns to full tilt`
    : `returning: ${left.toFixed(1)} turns to rest`;
  $('tilt-bar').style.width = `${Math.max(0, Math.min(1, (tilt - m.rest_deg) / (m.full_deg - m.rest_deg))) * 100}%`;
  if (document.activeElement !== slider) slider.value = String(turns);
}

const clock = new THREE.Timer();
function tick(): void {
  const raw = clock.update().getDelta();
  const dt = Math.min(0.1, raw);
  if (motion && playing && packS === 0) {
    f = (f + (dt * turnsPerSecond) / motion.ratio) % 1;
    dirty = true;
  }
  if (pack && packS !== packTarget) {   // the modules on their way (packSeconds each way)
    const was = packS;
    const step = Math.min(0.5, raw) / packSeconds();     // (real time: a slow GPU takes as long)
    packS = packTarget > packS ? Math.min(packTarget, packS + step) : Math.max(packTarget, packS - step);
    pack.set(packS);
    dirty = true;
    if (packS === 1 && was < 1) frameBox();
    if (packS === 0 && was > 0) { $('pack-info').hidden = true; pour.setVisible(true); reframe(); }
  }
  if (stage.controls.update()) dirty = true;
  if (dirty && motion) {
    diagram?.update(f);
    readouts(motion);
    if (loaded && packS === 0) {
      loaded.action.time = f * loaded.clipDuration;
      loaded.mixer.update(0);
      pour.update(f);
    }
  }
  if (dirty) {
    dirty = false;
    stage.renderer.render(stage.scene, stage.camera);
  }
  requestAnimationFrame(tick);
}

async function init(): Promise<void> {
  layout();
  setPlaying(true);
  requestAnimationFrame(tick);
  try {
    cards = (await getJson<{ cradles: CradleCard[] }>('/api/cradles')).cradles;
  } catch (err) {
    status(`could not list the cradles: ${(err as Error).message}`);
    return;
  }
  const want = params.get('design');
  const first = cards.find((c) => c.key === want) ?? cards.find((c) => c.verdict === 'build') ?? cards[0];
  if (!first) { status('no decanting cradles registered'); return; }
  const turns = params.has('turns') ? Number(params.get('turns')) : undefined;
  if (turns !== undefined && Number.isFinite(turns)) setPlaying(false);
  await selectDesign(first.key, Number.isFinite(turns) ? turns : undefined);
  if (params.get('pack') === '1') {         // straight to the packed box (a deep link)
    await togglePack();
    if (pack) { packS = 1; pack.set(1); frameBox(); dirty = true; }
  }
}

void init();
