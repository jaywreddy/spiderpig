/**
 * The decanting cradle page (``decant.html``): pick a design, watch it pour. The baked
 * cradle in 3D (``/api/glb/klann?linkage=<key>``: the whole cradle, the bottle and the
 * decanter) beside its mechanism in the linkage's plane (``/api/cradle/<key>``: SVG,
 * ``diagram.ts``), both posed from one clip fraction ``f`` (0: the crank at 0; the clip is
 * one crank turn). The slider and the readouts speak in handle turns from rest and the
 * bottle's tilt. Deep links: ``?design=decanter_rcm&turns=7.5&view=side`` (``&overlays=0``:
 * without the gap's measuring line, for a picture; ``&pack=1``: packed). ``window.__decant``
 * is a browser test's or a render's handle: whether the page has settled, the camera on named
 * parts, parts hidden, the pack held part-way.
 */
import * as THREE from 'three';
import './decant.css';
import { createStage, frameView } from '../scene';
import { loadGlb, teardown, type LoadedScene } from '../loader';
import { isAbort } from '../loads';
import type { View } from '../types';
import { COLORS, createDiagram, type Diagram } from './diagram';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import {
  createPack, movedModule, packFraction, packOrder, snappedDelta, type Pack, type PackData,
  type PackModule,
} from './pack';
import { createPour, type RoomData } from './pour';
import {
  fractionAtTurns, inToggleZone, phaseAt, tiltAt, tiltWords, toggleZones, turnsAt, type CradleMotion,
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

/** How it's used (``motion.use``: the guide's own lines): the use limits' five-line card,
 * then each use step folded (design review round 2's M5: the page had no use limits). */
export function renderUse(box: HTMLElement, m: CradleMotion): void {
  box.replaceChildren();
  const use = m.use ?? [];
  box.hidden = !use.length;
  if (!use.length) return;
  const h = document.createElement('h3');
  h.textContent = 'Using it: the five rules';
  const limits = use.find((u) => u.tag === 'limits');
  const card = document.createElement('ol');
  card.className = 'card';
  for (const line of (limits?.lines ?? []).filter((l) => /^\d\.\s/.test(l))) {
    const li = document.createElement('li');
    li.textContent = line.replace(/^\d\.\s*/, '');
    card.appendChild(li);
  }
  box.append(h, card);
  for (const u of use) {
    const d = document.createElement('details');
    const s = document.createElement('summary');
    s.textContent = u.tag === 'limits' ? 'The rest of the use limits' : u.title;
    const ul = document.createElement('ul');
    for (const line of u.lines.filter((l) => u.tag !== 'limits' || !/^\d\.\s/.test(l))) {
      const li = document.createElement('li');
      li.textContent = line;
      ul.appendChild(li);
    }
    d.append(s, ul);
    box.appendChild(d);
  }
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
    renderUse($('use'), m);
    slider.max = String(m.ratio);
    const fullAt = (m.turns_to_full / m.ratio) * 100;
    $('tick-full').style.left = `${fullAt}%`;
    renderZones(m);
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
    renderAbout(m);               // (the bottle the pour opens on: its legend line)
    reframe();
    status('');
    dirty = true;
    // the room under the mouth (a fabrication: a minute the first time), then the riser
    roomKnown = false;
    void getJson<RoomData>(`/api/cradle/${encodeURIComponent(key)}/room`, signal).then((r) => {
      if (mine === ticket && loaded === next) {
        pour.setRoom(r);
        if (packTarget > 0 || packS > 0) pour.setVisible(false);   // (packed meanwhile)
        dirty = true;
      }
    }).catch((err: unknown) => { if (!isAbort(err)) console.error(err); })
      .finally(() => { if (mine === ticket) roomKnown = true; });
  } catch (err) {
    if (isAbort(err)) return;
    console.error(err);
    status(`could not load ${key}: ${(err as Error).message}`);
  }
}

// -- the flat pack ------------------------------------------------------------------------

function resetPack(): void {
  stopEditing();
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
  const h = document.createElement('h3');
  h.textContent = d.mode === 'modules' ? 'Packed in modules' : 'Flat pack';
  const size = document.createElement('p');
  size.className = 'pack-size';
  size.textContent = `One box, ${x} × ${z} × ${y} mm outside (${d.box.volume_l} L).`;
  const how = document.createElement('p');
  how.textContent = d.mode === 'modules'
    ? 'It comes apart in modules, by hand, in this order:'
    : 'It comes apart where it is already screwed together, in this order:';
  const list = (steps: string[]): HTMLOListElement => {
    const ol = document.createElement('ol');
    for (const step of steps) {
      const li = document.createElement('li');
      li.textContent = step;
      ol.appendChild(li);
    }
    return ol;
  };
  const ol = list(d.steps);
  const back: HTMLElement[] = [];
  if (d.reassembly?.length) {
    const p = document.createElement('p');
    p.textContent = 'It goes back together in this order:';
    back.push(p, list(d.reassembly));
  }
  // the pieces as they go in the box: what is under another goes in first
  const order = packOrder(d);
  const nSpacers = order.filter((m) => m.spacer && !m.filler).length;
  const nFillers = order.filter((m) => m.filler).length;
  const h2 = document.createElement('h3');
  h2.textContent = `In the box, from the floor up (${d.modules.length} pieces`
    + (nSpacers ? `, ${nSpacers} printed spacers` : '')
    + (nFillers ? `, ${nFillers} corrugated fillers` : '') + ')';
  const pieces = document.createElement('ol');
  pieces.className = 'pack-pieces';
  for (const m of order) {
    const li = document.createElement('li');
    li.textContent = m.label;
    li.title = m.how;
    if (m.spacer) li.className = 'pack-spacer';
    pieces.appendChild(li);
  }
  info.append(h, size, how, ol, ...back, h2, pieces, editPanel());
  info.hidden = false;
}

// -- editing the packing: move and turn a piece, export the layout --------------------------
//
// A piece picked (a click on it, packed) gets a move gizmo; R turns it a quarter turn about
// the vertical, T tips it a quarter turn, Esc lets it go; moves snap to 3 mm. Export writes
// the pack as edited (the /pack body's format) for ``spiderpig pack-inspect --layout``,
// which judges it: the box, the overlaps, what rests on what.

let editing = false;
let picked = -1;
let pickedFrom: PackModule | null = null;   // the piece as it was when picked
const pickedCentre = new THREE.Vector3();
let yaw = 0;
let tip = 0;
const gizmo = new TransformControls(stage.camera, canvas);
gizmo.setMode('translate');
gizmo.setSpace('world');
const gizmoProxy = new THREE.Object3D();
gizmoProxy.name = 'pack_edit_pivot';
gizmo.addEventListener('dragging-changed', (e) => { stage.controls.enabled = !e.value; });
gizmo.addEventListener('objectChange', () => applyEdit());
gizmo.addEventListener('change', () => { dirty = true; });

function editPanel(): HTMLElement {
  const box = document.createElement('div');
  box.className = 'pack-edit';
  const edit = document.createElement('button');
  edit.textContent = editing ? 'Stop editing' : 'Edit the packing';
  edit.title = 'Pick a piece to move it (3 mm steps); R turns it, T tips it, Esc lets it go';
  edit.addEventListener('click', () => {
    if (editing) stopEditing(); else startEditing();
    edit.textContent = editing ? 'Stop editing' : 'Edit the packing';
  });
  const out = document.createElement('button');
  out.textContent = 'Export layout';
  out.title = 'The pack as it is now, as JSON: spiderpig pack-inspect --layout judges it';
  out.addEventListener('click', exportLayout);
  const help = document.createElement('p');
  help.className = 'pack-edit-help';
  help.textContent = 'Editing: click a piece, drag its arrows; R turns it, T tips it, Esc lets it go.';
  box.append(edit, out, help);
  return box;
}

function startEditing(): void {
  if (!pack || !loaded || packS < 1) return;
  editing = true;
  loaded.walker.add(gizmoProxy);
  stage.scene.add(gizmo.getHelper());
  document.body.classList.add('pack-editing');
  dirty = true;
}

function stopEditing(): void {
  editing = false;
  picked = -1;
  pickedFrom = null;
  gizmo.detach();
  gizmo.getHelper().removeFromParent();
  gizmoProxy.removeFromParent();
  stage.controls.enabled = true;
  document.body.classList.remove('pack-editing');
  dirty = true;
}

function pick(i: number): void {
  if (!pack) return;
  picked = i;
  pickedFrom = pack.module(i);
  yaw = 0;
  tip = 0;
  pack.moduleBounds(i).getCenter(pickedCentre);
  gizmoProxy.position.copy(pickedCentre);
  gizmo.attach(gizmoProxy);
  status(`editing: ${pickedFrom.label}`);
}

function applyEdit(): void {
  if (!pack || picked < 0 || !pickedFrom) return;
  const shift = gizmoProxy.position.clone().sub(pickedCentre);
  const delta = snappedDelta(pickedCentre.toArray(), shift.toArray(), yaw, tip);
  pack.setModule(picked, movedModule(pickedFrom, delta));
  dirty = true;
}

const pointerAt = new THREE.Vector2();
canvas.addEventListener('pointerdown', (e) => pointerAt.set(e.clientX, e.clientY));
canvas.addEventListener('pointerup', (e) => {
  if (!editing || !pack || !loaded || gizmo.dragging || gizmo.axis !== null) return;
  if (Math.hypot(e.clientX - pointerAt.x, e.clientY - pointerAt.y) > 4) return;   // (a turn)
  const r = canvas.getBoundingClientRect();
  const ndc = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1,
    -((e.clientY - r.top) / r.height) * 2 + 1);
  const ray = new THREE.Raycaster();
  ray.setFromCamera(ndc, stage.camera);
  for (const hit of ray.intersectObject(loaded.walker, true)) {
    const i = pack.moduleOf(hit.object);
    if (i >= 0) { pick(i); return; }
  }
});
window.addEventListener('keydown', (e) => {
  if (!editing || picked < 0) return;
  const k = e.key.toLowerCase();
  if (k === 'r') yaw += 1;
  else if (k === 't') tip += 1;
  else if (k === 'escape') { gizmo.detach(); picked = -1; status(''); dirty = true; return; }
  else return;
  applyEdit();
});

// (a hook for a browser test: edit without aiming a click at a piece)
(window as unknown as { __packEdit: object }).__packEdit = {
  start: startEditing, pick, data: () => pack?.data(), editing: () => editing,
};

// (a hook for a browser test or a render: the page settled; the camera on named parts)
let roomKnown = false;
(window as unknown as { __decant: object }).__decant = {
  /** the glb posed, the room under the mouth in (or failed), a pack asked for flown in */
  ready: () => !!loaded && !!motion && roomKnown && packS === packTarget,
  /** The camera from ``dir`` (world, Z up) on the named nodes' box, ``margin`` round it. */
  frameOn(names: string[], dir: [number, number, number], margin = 1.15): boolean {
    if (!loaded) return false;
    loaded.root.updateMatrixWorld(true);
    const box = new THREE.Box3();
    for (const n of names.map((x) => THREE.PropertyBinding.sanitizeNodeName(x))) {
      loaded.root.traverse((o) => {
        if (o.visible && o.name.startsWith(n)) box.expandByObject(o);
      });
    }
    if (box.isEmpty()) return false;
    const size = box.getSize(new THREE.Vector3()), centre = box.getCenter(new THREE.Vector3());
    const cam = stage.camera;
    const r = (size.length() / 2) * margin;
    const dist = r / Math.sin(THREE.MathUtils.degToRad(cam.fov) / 2);
    stage.controls.target.copy(centre);
    cam.position.copy(centre).addScaledVector(new THREE.Vector3(...dir).normalize(), dist);
    cam.near = Math.max(0.5, dist / 100);
    cam.far = dist * 40;
    cam.updateProjectionMatrix();
    stage.controls.update();
    dirty = true;
    return true;
  },
  /** The camera at ``pos`` looking at ``target`` (world mm, Z up). */
  look(pos: [number, number, number], target: [number, number, number]): void {
    stage.camera.position.set(...pos);
    stage.controls.target.set(...target);
    stage.camera.near = 2;
    stage.camera.far = 40000;
    stage.camera.updateProjectionMatrix();
    stage.controls.update();
    dirty = true;
  },
  /** The node names under the cradle (what ``frameOn`` takes). */
  names: () => {
    const out: string[] = [];
    loaded?.walker.traverse((o) => { if (o.name) out.push(o.name); });
    return out;
  },
  camera: () => ({ pos: stage.camera.position.toArray(), target: stage.controls.target.toArray() }),
  /** The measuring overlay (the mouth-to-rim gap) on or off. */
  overlays: (v: boolean) => pour.setOverlays(v),
  /** The stage (a test's look at the scene, the renderer's info). */
  stage: () => stage,
  /** Hide (or show) the named nodes (the bottle, to see the saddles under it). */
  hide(names: string[], hidden = true): void {
    for (const n of names) {
      const o = loaded?.root.getObjectByName(THREE.PropertyBinding.sanitizeNodeName(n));
      if (o) o.visible = !hidden;
    }
    dirty = true;
  },
  /** The flat pack held at ``s`` (0 assembled .. 1 packed; after ``?pack=1``). */
  packAt(s: number): void {
    if (!pack) return;
    packS = packTarget = Math.max(0, Math.min(1, s));
    pack.set(packS);
    dirty = true;
  },
};

function exportLayout(): void {
  if (!pack || !motion) return;
  const d = pack.data();
  const edited = { ...d, linkage: motion.key, plan: `${d.plan ?? 'a packing'}, edited in the viewer` };
  const blob = new Blob([JSON.stringify(edited)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `${motion.key}.edited.layout.json`;
  a.click();
  URL.revokeObjectURL(a.href);
}

/** How long the pieces take to fly into the box (or back), seconds: longer for more pieces. */
function packSeconds(): number {
  return Math.max(3, 0.3 * (packData?.modules.length ?? 0));
}

async function togglePack(): Promise<void> {
  if (!motion || !loaded) return;
  if (packTarget > 0) {           // back to the assembled cradle
    stopEditing();
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
    // the transforms start from the pack's pose (B is cranked to it first: the readouts and
    // the diagram show it too, and the cradle comes back to it)
    f = packFraction(packData);
    loaded.action.time = f * loaded.clipDuration;
    loaded.mixer.update(0);
    dirty = true;
    pack = createPack(loaded.walker, packData, ['bottle', 'decanter']);
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
  $('zone').hidden = !inToggleZone(m, turns);
  if (document.activeElement !== slider) slider.value = String(turns);
}

/** The toggle zones as red bands on the slider's track (the gearbox's red arcs). */
function renderZones(m: CradleMotion): void {
  const box = $('zones');
  box.replaceChildren(...toggleZones(m).map(([a, b]) => {
    const band = document.createElement('span');
    band.style.left = `${(a / m.ratio) * 100}%`;
    band.style.width = `${((b - a) / m.ratio) * 100}%`;
    return band;
  }));
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
