/**
 * The pour on the cradle page: the bottle drawn as glass round the wine it holds, the
 * wine's surface level in the world as the cradle tips it, the stream from the mouth once
 * the wine reaches it, and the vessel under the mouth (a decanter or a glass picked from
 * ``vessels.ts``, on the table or a riser) filling as it pours. A small panel picks the
 * vessel and the print colour and reads out the gap from the mouth to the rim, the printed
 * riser it stands on (the highest stack the rule allows: its rim 20 mm under the light's
 * clip, ``liquid.riserFits``, once the API sends the room under the mouth) and how much has
 * poured.
 *
 * The wine: the bottle starts full (its nominal volume, ~6 cm ullage), and at each tilt
 * keeps what lies under its mouth's lowest point (``liquid.ts``); swinging back to rest
 * refills it (the page is a simulator: rewinding rewinds the pour).
 *
 * Use: ``createPour(stage, onChange)``; ``attach(loaded, motion)`` after a glb loads (it
 * dresses the scene through ``realism.ts``), ``update(f)`` after posing at clip fraction
 * ``f``, ``detach()`` before tearing the scene down.
 */
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js';
import type { Stage } from '../scene';
import type { LoadedScene } from '../loader';
import {
  adjustShift, fractionAtTurns, saddleAt, settingOffsets, turnsAt, type CradleMotion, type Setting,
} from './motion';
import {
  bestRiser, bottleInside, fitsRoom, heightFor, keptFromRest, levelFor, nominalVolume, radiusAt,
  riserFits, riserStack, stackWords, startingFill, surfaceRuns, ML, RISER_RULE, type Profile,
  type RiserRule, type Stack,
} from './liquid';
import {
  DEFAULT_BOTTLE, DEFAULT_VESSEL, VESSELS, brimMl, cavityOf, heightOf, maxRadiusOf, vesselFor,
  type Vessel,
} from './vessels';
import { createLook, PRINT_COLORS, type Look } from './realism';
import './pour.css';

/** The room under the mouth and the riser's rule (``/api/cradle/{key}/room``,
 * ``cradle_report.room``). */
export interface RoomData {
  /** the room over the whole swing: ``[[height_over_table_mm, max_radius_mm], ...]`` */
  room: [number, number][];
  /** the room once the cradle is past -20° (the pour), the same form: not offered (one riser
   * for the whole swing, never raised mid-pour) */
  room_pouring: [number, number][];
  /** the cradle box's lowest part over the drawn decanter's rim over the swing */
  lowest?: { part: string; mm: number; tilt_deg: number; within_mm: number };
  /** the rule (``construction.cradle.RISER_CLEAR``): the rim this far under the lowest part,
   * this far off the room's side */
  rule?: { rim_under_mm: number; side_mm: number };
  /** every stack the printed riser makes, its height and pieces */
  stacks?: [number, string[]][];
}

/** What the API may add to the swing for the pour (optional). */
interface PourMotion extends CradleMotion {
  /** a riser under the vessel, mm */
  riser_mm?: number;
  room?: RoomData;
}



/** A mesh a bottle's setting moves (``motion.adjust``): its geometry as baked, its group, and
 * the cradle's frame at clip time 0 in its own (a setting's shift there, in its geometry). */
interface Adjusted { mesh: THREE.Mesh; base: THREE.BufferGeometry; group: 'front' | 'rear' | 'stop'; inMesh: THREE.Matrix3 }

/** The label's size line for a bottle's name ("1.5 L Bordeaux magnum": MAGNUM · 1,5 L). */
function sizeLine(name: string): string {
  const n = name.toLowerCase();
  if (n.includes('methuselah')) return 'MATHUSALEM · 6 L';
  if (n.includes('6 l')) return 'IMPÉRIALE · 6 L';
  if (n.includes('3 l')) return 'JÉROBOAM · 3 L';
  if (n.includes('1.5 l') || n.includes('magnum')) return 'MAGNUM · 1,5 L';
  return '75 cl';
}

export interface Pour {
  look: Look;
  attach(loaded: LoadedScene, motion: CradleMotion): void;
  detach(): void;
  update(f: number): void;
  /** The room under the mouth arrived (``/api/cradle/{key}/room``): pick the riser. */
  setRoom(room: RoomData): void;
  /** Show or hide the vessel, the wine and the stream (the flat pack hides them). */
  setVisible(v: boolean): void;
  /** Put another bottle size in the cradle (``motion.bottles``' ``key``): its V blocks and its
   * base stop move to its setting. */
  setBottle(key: string): void;
}

const ULLAGE_MM = 60;
const RISER_HIGH = 250;          // the highest riser the rule is tried at
const WINE_INSET = 0.5;          // the wine stays this far inside the glass (no z-fight)

// -- geometry ---------------------------------------------------------------------------------

/** A closed glass solid of revolution: ``outer`` (``[[axial, r], ...]``) out, ``inner``
 * back, about +Y (three's lathe), then turned onto ``axis``. */
function glassSolid(outer: Profile, inner: Profile, axis: 'x' | 'z', segments = 72,
  closeOuterAt: 'start' | 'end' = 'start'): THREE.BufferGeometry {
  const pts: THREE.Vector2[] = [];
  // outer, from the closed end (on the axis) to the open end, then the inner back to the axis
  const o = closeOuterAt === 'start' ? [...outer] : [...outer].reverse();
  const i = closeOuterAt === 'start' ? [...inner].reverse() : [...inner];
  pts.push(new THREE.Vector2(0, o[0]![0]));
  for (const [s, r] of o) pts.push(new THREE.Vector2(r, s));
  for (const [s, r] of i) pts.push(new THREE.Vector2(Math.max(0.01, r), s));
  pts.push(new THREE.Vector2(0, i[i.length - 1]![0]));
  const g = new THREE.LatheGeometry(pts, segments);
  if (axis === 'x') g.rotateZ(-Math.PI / 2); else g.rotateX(Math.PI / 2);
  return g;
}

/** A solid of revolution closed at both ends (the wine). */
function solid(p: Profile, axis: 'x' | 'z', segments = 64): THREE.BufferGeometry {
  const pts: THREE.Vector2[] = [new THREE.Vector2(0, p[0]![0])];
  for (const [s, r] of p) pts.push(new THREE.Vector2(Math.max(0.01, r), s));
  pts.push(new THREE.Vector2(0, p[p.length - 1]![0]));
  const g = new THREE.LatheGeometry(pts, segments);
  if (axis === 'x') g.rotateZ(-Math.PI / 2); else g.rotateX(Math.PI / 2);
  return g;
}

function inset(p: Profile, by: number): Profile {
  return p.map(([s, r]) => [s, Math.max(0.2, r - by)] as [number, number]);
}

/** The bottle's outside, the upper half of ``motion.bottle.outline``, with a bead at the
 * lip (the string ring a cork-finish bottle has). */
function bottleOutside(outline: readonly (readonly [number, number])[]): Profile {
  const up = outline.filter(([, y]) => y >= 0).map(([x, y]) => [x, y] as [number, number]);
  up.sort((a, b) => a[0] - b[0]);
  const out: [number, number][] = [];
  for (const [x, r] of up) {
    const last = out[out.length - 1];
    if (last && Math.abs(last[0] - x) < 1e-6) { last[1] = Math.max(last[1], r); continue; }
    out.push([x, r]);
  }
  const r0 = out[0]![1];
  const bead: [number, number][] = [[0, r0 + 0.6], [1.2, r0 + 1.6], [8, r0 + 1.6], [10, r0 + 0.4], [12, r0]];
  return [...bead, ...out.filter(([x]) => x > 12)];
}

/** A paper label round the bottle's upper side (the label up, as a sommelier pours). */
function labelMesh(radius: number, s0: number, s1: number, size = 'MAGNUM · 1,5 L'): THREE.Mesh {
  const cv = document.createElement('canvas');
  cv.width = 512; cv.height = 512;
  const ctx = cv.getContext('2d')!;
  ctx.fillStyle = '#efe6d2';
  ctx.fillRect(0, 0, 512, 512);
  ctx.strokeStyle = '#8c6a3a';
  ctx.lineWidth = 6;
  ctx.strokeRect(22, 22, 468, 468);
  ctx.lineWidth = 2;
  ctx.strokeRect(36, 36, 440, 440);
  ctx.fillStyle = '#3a1d16';
  ctx.textAlign = 'center';
  ctx.font = 'italic 34px Georgia, serif';
  ctx.fillText('Grand Vin de', 256, 150);
  ctx.font = 'bold 64px Georgia, serif';
  ctx.fillText('SPIDERPIG', 256, 240);
  ctx.font = '28px Georgia, serif';
  ctx.fillText('Mis en bouteille au château', 256, 310);
  ctx.font = 'bold 44px Georgia, serif';
  ctx.fillText('2019', 256, 395);
  ctx.font = '22px Georgia, serif';
  ctx.fillText(size, 256, 445);
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  // a cylinder patch: ±60° about +y (the cradle frame's "up" at rest), s0..s1 along x
  const span = (2 * Math.PI) / 3;
  const g = new THREE.CylinderGeometry(radius, radius, s1 - s0, 48, 1, true, -span / 2, span);
  // three's cylinder runs along Y with theta from +Z towards +X; turn it onto the X axis, facing +Y
  g.rotateX(-Math.PI / 2);      // the patch's middle +Z -> +Y, the axis Y -> -Z
  g.rotateY(Math.PI / 2);       // the axis Z -> X
  g.translate((s0 + s1) / 2, 0, 0);
  const m = new THREE.Mesh(g, new THREE.MeshPhysicalMaterial({ map: tex, roughness: 0.8, metalness: 0, side: THREE.FrontSide }));
  m.castShadow = true;
  m.name = 'label';
  return m;
}

/** The printed riser ``stack`` (``liquid.Stack``: its pieces bottom up, ``blocks`` the two
 * 100 × 49.8 mm blocks side by side, else a 100 × 100 mm plate that thick): each piece a
 * rounded block, its 2 mm boss on top up into the next one's recess (``riser_pieces``, the
 * fabricated riser). */
function riserStackMesh(stack: Stack, mat: THREE.Material, block = 85, lip = 2): THREE.Group {
  const g = new THREE.Group();
  g.name = 'riser_stack';
  const W = 100;
  const add = (m: THREE.Mesh): void => { m.castShadow = m.receiveShadow = true; g.add(m); };
  let z = 0;
  for (const piece of stack[1]) {
    const h = piece === 'blocks' ? block : Number(piece);
    const parts = piece === 'blocks' ? [-W / 4 - 0.1, W / 4 + 0.1] : [0];
    for (const y of parts) {
      const m = new THREE.Mesh(new RoundedBoxGeometry(W, parts.length > 1 ? W / 2 - 0.2 : W, h, 2, 2), mat);
      m.position.set(0, y, z + h / 2);
      add(m);
    }
    z += h;
  }
  if (stack[1].length) {           // the top piece's boss
    const boss = new THREE.Mesh(new RoundedBoxGeometry(W - 9, W - 9, lip, 1, 0.8), mat);
    boss.position.z = z + lip / 2;
    add(boss);
  }
  return g;
}

/** A tube along ``curve`` whose radius at ``t`` is ``radius(t)``. */
function taperedTube(curve: THREE.Curve<THREE.Vector3>, radius: (t: number) => number,
  geometry: THREE.BufferGeometry, n = 48, m = 12): void {
  const frames = curve.computeFrenetFrames(n, false);
  const pos = new Float32Array((n + 1) * (m + 1) * 3), nor = new Float32Array((n + 1) * (m + 1) * 3);
  const p = new THREE.Vector3(), v = new THREE.Vector3();
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    curve.getPointAt(t, p);
    const N = frames.normals[i]!, B = frames.binormals[i]!, r = radius(t);
    for (let j = 0; j <= m; j++) {
      const a = (j / m) * Math.PI * 2, c = Math.cos(a), s = Math.sin(a);
      v.set(c * N.x + s * B.x, c * N.y + s * B.y, c * N.z + s * B.z);
      const k = (i * (m + 1) + j) * 3;
      pos[k] = p.x + r * v.x; pos[k + 1] = p.y + r * v.y; pos[k + 2] = p.z + r * v.z;
      nor[k] = v.x; nor[k + 1] = v.y; nor[k + 2] = v.z;
    }
  }
  if (!geometry.index) {
    const idx: number[] = [];
    for (let i = 0; i < n; i++) for (let j = 0; j < m; j++) {
      const a = i * (m + 1) + j, b = (i + 1) * (m + 1) + j;
      idx.push(a, b, a + 1, b, b + 1, a + 1);
    }
    geometry.setIndex(idx);
  }
  geometry.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geometry.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  geometry.computeBoundingSphere();
}

// -- the panel ---------------------------------------------------------------------------------

function el<K extends keyof HTMLElementTagNameMap>(tag: K, cls = '', text = ''): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text) e.textContent = text;
  return e;
}

const litres = (mm3: number): string => {
  const ml = mm3 / ML;
  return ml >= 995 ? `${(ml / 1000).toFixed(2)} L` : `${Math.round(ml)} mL`;
};

// -- the pour -----------------------------------------------------------------------------------

export function createPour(stage: Stage, onChange: () => void): Pour {
  const look = createLook(stage, onChange);
  const params = new URLSearchParams(location.search);
  let vessel: Vessel = VESSELS.find((v) => v.key === params.get('vessel'))
    ?? VESSELS.find((v) => v.key === DEFAULT_VESSEL) ?? VESSELS[0]!;
  if (params.get('print')) look.setPrintColor(params.get('print')!);

  // the panel
  const panel = el('section', '', '');
  panel.id = 'pour-panel';
  const pick = el('select');
  pick.title = 'What the cradle pours into';
  const groups: Record<string, HTMLOptGroupElement> = {};
  for (const v of VESSELS) {
    const label = v.kind === 'glass' ? 'Wine glasses' : 'Decanters';
    let g = groups[label];
    if (!g) { g = document.createElement('optgroup'); g.label = label; groups[label] = g; pick.appendChild(g); }
    const short = v.name.replace(/\s*\(.*?\)\s*/g, ' ').trim();
    const o = el('option', '', `${short} · ${v.capacity_ml >= 1000 ? `${v.capacity_ml / 1000} L` : `${v.capacity_ml} mL`}`);
    o.value = v.key;
    g.appendChild(o);
  }
  pick.value = vessel.key;
  const facts = el('div', 'facts');
  const riserRow = el('label', 'pick');
  const riserIn = el('input');
  riserIn.type = 'range';
  riserIn.min = '0';
  riserIn.max = '0';
  riserIn.step = '1';
  riserIn.title = 'The printed riser’s stack under the vessel (by the rule: its rim 20 mm '
    + 'under the light’s clip); slide to try another';
  riserIn.disabled = true;
  const riserVal = el('span', 'v');
  riserRow.append(el('span', 'k', 'Riser'), riserIn, riserVal);
  const riserNote = el('div', 'note');
  const gapRow = el('div', 'row');
  const gapVal = el('b');
  const badge = el('span', 'badge');
  gapRow.append(el('span', 'k', 'Mouth to rim'), gapVal, badge);
  const pourRow = el('div', 'row');
  const pourVal = el('b');
  const overflow = el('span', 'badge');
  pourRow.append(el('span', 'k', 'Poured'), pourVal, overflow);
  const bar = el('div', 'fill');
  const barIn = el('div');
  bar.appendChild(barIn);
  const keptRow = el('div', 'row small');
  const printSel = el('select');
  printSel.title = 'The printed parts’ colour (PLA)';
  for (const name of Object.keys(PRINT_COLORS)) {
    const o = el('option', '', name);
    o.value = name;
    printSel.appendChild(o);
  }
  printSel.value = look.printColor;
  const pickRow = el('label', 'pick');
  pickRow.append(el('span', 'k', 'Pour into'), pick);
  const bottlePick = el('select');
  bottlePick.title = 'The bottle in the cradle';
  const bottleRow = el('label', 'pick');
  bottleRow.append(el('span', 'k', 'Bottle'), bottlePick);
  const printRow = el('label', 'pick');
  printRow.append(el('span', 'k', 'Prints'), printSel);
  const watch = el('button', 'watch', 'Watch the pour');
  watch.title = 'Frame the camera on the mouth and the vessel (from the mouth’s side)';
  printRow.appendChild(watch);
  panel.append(bottleRow, pickRow, facts, riserRow, riserNote, gapRow, pourRow, bar, keptRow,
    printRow);
  if (look.quality === 'low') panel.append(el('div', 'note', 'simplified glass (software GPU): ?quality=high for refraction'));
  // in the right-hand column under the readouts if the page has one, else floating
  const hud = document.getElementById('hud');
  if (hud) { hud.after(panel); panel.classList.add('docked'); } else document.body.appendChild(panel);
  const gapLabel = el('div', 'gap-label');
  gapLabel.id = 'gap-label';
  gapLabel.hidden = true;
  document.body.appendChild(gapLabel);

  // the state of one loaded design
  let loaded: LoadedScene | null = null;
  let motion: PourMotion | null = null;
  let frame: THREE.Group | null = null;            // the cradle frame, riding the bottle node
  let adjusted: Adjusted[] = [];                    // the V blocks' and the base stop's meshes
  let drawn: Setting | undefined;                   // the setting the model is drawn at
  let baseSaddles: CradleMotion['saddles'] = [];
  let wineBody: THREE.Mesh | null = null, wineTop: THREE.Mesh | null = null;
  let inside: Profile = [], wineProfile: Profile = [], mouthOuter = 15;
  let fill = 0, kept: Float64Array = new Float64Array(0);
  let stand: THREE.Group | null = null;            // the vessel (and riser) on the table
  let vesselWine: THREE.Mesh | null = null, vesselWineVol = -1;
  let stream: THREE.Mesh | null = null;
  let gapLine: THREE.LineSegments | null = null;
  let anchor = new THREE.Vector3();                 // the vessel's axis on the table
  let lastF = 0;
  const plane = new THREE.Plane(new THREE.Vector3(0, 0, -1), 0);
  const wineClipped = look.materials.wine.clone();
  wineClipped.clippingPlanes = [plane];
  wineClipped.clipShadows = true;
  const topMat = look.materials.wine.clone();
  topMat.side = THREE.DoubleSide;
  const streamMat = look.materials.wine.clone();
  streamMat.color.set('#7d0f26');
  streamMat.emissive.set('#2a0309');      // a thin stream glows garnet against the light

  // the riser under the vessel: the highest printed stack the rule allows (the room under
  // the mouth, from the API: none until it arrives); the panel's slider tries another
  let stack: Stack = [0, []], riserByHand = false, mouthZ = 0;
  let vesselByHand = params.has('vessel');
  let modelRiser: THREE.Object3D[] = [];           // the baked model's riser (it packs)
  /** The vessel for a bottle by its name's volume (``vesselFor``); whether it changed. */
  function pickVessel(bottleName: string): boolean {
    const v = vesselFor(nominalVolume(bottleName, 750) / ML);
    if (v.key === vessel.key) return false;
    vessel = v;
    pick.value = v.key;
    return true;
  }
  const riser = (): number => stack[0];
  const stacks = (): Stack[] => (motion?.room?.stacks ?? []) as Stack[];
  function ruleOf(): RiserRule {
    const r = motion?.room?.rule;
    return r ? { up: r.rim_under_mm, side: r.side_mm } : RISER_RULE;
  }
  /** The best stack for the vessel by the rule; null until the room is known. */
  function defaultStack(): Stack | null {
    const room = motion?.room;
    if (!room?.room.length || !room.stacks?.length) return null;
    return riserStack(bestRiser(vessel.profile, room.room, RISER_HIGH, ruleOf()), stacks());
  }
  function setStack(s: Stack): void {
    stack = s;
    const all = stacks();
    const i = all.findIndex((x) => x[0] === s[0]);
    riserIn.max = String(Math.max(0, all.length - 1));
    riserIn.value = String(Math.max(0, i));
    riserIn.disabled = !all.length;
    riserVal.textContent = s[0] > 0 ? `${s[0].toFixed(0)} mm` : 'none';
  }
  riserIn.addEventListener('input', () => {
    const s = stacks()[Number(riserIn.value)];
    if (!s) return;
    riserByHand = true;
    setStack(s);
    buildStand();
    pose(lastF);
    onChange();
  });
  let watchPending = params.get('camera') === 'pour';   // a deep link: after the page frames the cradle

  /** The camera on the pour: from the mouth's side (the cradle's -X end), a little above,
   * the vessel and the bottle's neck in frame. */
  function framePour(): void {
    if (!loaded || !frame) return;
    loaded.root.updateMatrixWorld(true);
    const mouth = frame.getWorldPosition(new THREE.Vector3());
    const bottom = Math.min(0, anchor.z);
    const target = new THREE.Vector3(anchor.x + 70, anchor.y, (mouth.z + bottom) * 0.55);
    const span = Math.max(mouth.z - bottom, 2 * maxRadiusOf(vessel)) * 0.5 + 150;
    const dir = new THREE.Vector3(-1, 0.55, 0.38).normalize();
    const cam = stage.camera;
    const dist = span / Math.tan(THREE.MathUtils.degToRad(cam.fov) / 2) * 1.05;
    stage.controls.target.copy(target);
    cam.position.copy(target).addScaledVector(dir, dist);
    cam.near = Math.max(0.5, dist / 100);
    cam.far = dist * 20;
    cam.updateProjectionMatrix();
    stage.controls.update();
    onChange();
  }
  watch.addEventListener('click', framePour);

  function buildStand(): void {
    if (!loaded) return;
    if (stand) {
      loaded.root.remove(stand);
      stand.traverse((o) => { if ((o as THREE.Mesh).isMesh) (o as THREE.Mesh).geometry.dispose(); });
    }
    stand = new THREE.Group();
    stand.name = 'vessel';
    const h0 = riser();
    if (h0 > 0) stand.add(riserStackMesh(stack, look.materials.print));
    const glass = new THREE.Mesh(glassSolid(vessel.profile, cavityOf(vessel), 'z'), look.materials.glassClear);
    glass.position.z = h0;
    glass.castShadow = false;
    glass.renderOrder = 2;
    glass.name = 'vessel_glass';
    stand.add(glass);
    vesselWine = new THREE.Mesh(new THREE.BufferGeometry(), look.materials.wine);
    vesselWine.position.z = h0;
    vesselWine.castShadow = true;
    vesselWine.visible = false;
    vesselWineVol = -1;
    stand.add(vesselWine);
    stand.position.copy(anchor);
    loaded.root.add(stand);
    look.footShadow(anchor.x, anchor.y, h0 > 0 ? maxRadiusOf(vessel) + 15 : vessel.profile[0]![1]);
    const cap = brimMl(vessel);
    const ml = (x: number): string => (x >= 995 ? `${(x / 1000).toFixed(2)} L` : `${Math.round(x)} mL`);
    facts.textContent = `${vessel.style}: ${heightOf(vessel).toFixed(0)} mm tall, Ø${(2 * maxRadiusOf(vessel)).toFixed(0)} mm,`
      + ` rim Ø${(2 * vessel.profile[vessel.profile.length - 1]![1]).toFixed(0)} mm; rated ${ml(vessel.capacity_ml)},`
      + ` ${ml(cap)} to the brim` + (h0 > 0 ? `; on a ${h0.toFixed(0)} mm riser` : '') + ' ';
    const src = el('a', '', 'sizes ↗');
    src.href = vessel.source;
    src.target = '_blank';
    src.rel = 'noopener';
    facts.appendChild(src);
    const room = motion?.room;
    if (room?.room.length) {
      // the one rule (round 2's stand review: the page had kept 3 mm, in the toggle dwell)
      const rule = ruleOf();
      const ok = riserFits(vessel.profile, h0, room.room, rule);
      const fit = fitsRoom(vessel.profile, h0, room.room);
      const rimZ = h0 + heightOf(vessel);
      const low = room.lowest;
      const under = low ? low.mm - rimZ : NaN;
      badge.hidden = false;
      badge.className = `badge ${ok ? 'ok' : 'bad'}`;
      badge.textContent = ok ? 'clears the whole swing'
        : !fit.fits ? `hit (${(-fit.margin).toFixed(0)} mm at ${fit.at.toFixed(0)} mm)`
          : `too high: ${under.toFixed(0)} mm under the clip (${rule.up} at least)`;
      const who = low?.part === 'led_clip' ? 'the light’s printed clip'
        : (low?.part ?? 'the lowest part').replace(/_/g, ' ');
      riserNote.textContent = (h0 > 0 ? `Riser: ${stackWords(stack)}. ` : 'No riser: on the table, in the locator. ')
        + (low ? `Its rim ${under.toFixed(0)} mm under ${who} (the lowest part over it, `
          + `${low.mm.toFixed(0)} mm over the table at rest; the rule: ${rule.up} mm or more). ` : '')
        + 'Set it at rest and leave it the whole pour: never raise the decanter during a pour.';
    } else {
      badge.hidden = true;
      riserNote.textContent = 'Measuring the room under the mouth (a minute the first time): '
        + 'the riser comes once it is known.';
    }
  }

  function setVesselWine(volume: number): void {
    if (!vesselWine) return;
    const v = Math.max(0, volume);
    if (Math.abs(v - vesselWineVol) < 0.3 * ML) return;
    vesselWineVol = v;
    const cav = inset(cavityOf(vessel), WINE_INSET * 0.6);
    if (v < 0.5 * ML) { vesselWine.visible = false; return; }
    const h = heightFor(cav, v);
    const prof: [number, number][] = cav.filter(([s]) => s < h).map(([s, r]) => [s, r]);
    prof.push([h, radiusAt(cav, h)]);
    vesselWine.geometry.dispose();
    vesselWine.geometry = solid(prof, 'z', 72);
    vesselWine.visible = true;
  }

  /** The kept volume at ``turns`` from rest (linear between the samples). */
  function keptAt(turns: number): number {
    if (!motion || !kept.length) return fill;
    const u = (turns / motion.ratio) * motion.n;
    const k0 = Math.max(0, Math.min(kept.length - 1, Math.floor(u)));
    const k1 = Math.min(kept.length - 1, k0 + 1), w = u - Math.floor(u);
    return kept[k0]! + (kept[k1]! - kept[k0]!) * w;
  }

  /** Turns from rest at which ``volume`` has poured (Infinity if it never does). */
  function turnsWhenPoured(volume: number): number {
    if (!motion) return Infinity;
    for (let k = 0; k < kept.length; k++) {
      if (fill - kept[k]! >= volume) return (k / motion.n) * motion.ratio;
    }
    return Infinity;
  }

  const tmpQ = new THREE.Quaternion(), up = new THREE.Vector3(), p0 = new THREE.Vector3();

  function pose(f: number): void {
    lastF = f;
    if (!loaded || !motion || !frame || !wineBody || !wineTop) return;
    loaded.root.updateMatrixWorld(true);
    let turns = turnsAt(motion, f);
    if (motion.ratio - turns < 1e-6) turns = 0;      // at rest from either side: full again
    const inBottle = keptAt(turns);
    const poured = Math.max(0, fill - inBottle);

    // the wine in the bottle: the level for what it keeps at this tilt, level in the world
    frame.getWorldQuaternion(tmpQ);
    up.set(0, 0, 1).applyQuaternion(tmpQ.invert());
    const a = Math.atan2(up.x, up.y);
    if (inBottle > 0.5 * ML) {
      const c = levelFor(inside, a, inBottle);
      p0.set(c * Math.sin(a), c * Math.cos(a), 0);
      frame.localToWorld(p0);
      plane.constant = p0.z;
      const runs = surfaceRuns(wineProfile, a, c, 1.5);
      const pos: number[] = [];
      for (const run of runs) {
        for (let i = 1; i < run.length; i++) {
          const [sa, ua, wa] = run[i - 1]!, [sb, ub, wb] = run[i]!;
          pos.push(sa, ua, -wa, sb, ub, -wb, sb, ub, wb, sa, ua, -wa, sb, ub, wb, sa, ua, wa);
        }
      }
      const g = wineTop.geometry;
      g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
      const nrm = new Float32Array(pos.length);
      for (let i = 0; i < nrm.length; i += 3) { nrm[i] = Math.sin(a); nrm[i + 1] = Math.cos(a); nrm[i + 2] = 0; }
      g.setAttribute('normal', new THREE.BufferAttribute(nrm, 3));
      g.computeBoundingSphere();
      wineBody.visible = wineTop.visible = true;
    } else {
      wineBody.visible = wineTop.visible = false;
    }

    // the vessel fills
    const brim = brimMl(vessel) * ML;
    setVesselWine(Math.min(poured, brim));
    pourVal.textContent = litres(poured);
    barIn.style.width = `${Math.min(100, (poured / brim) * 100)}%`;
    bar.classList.toggle('over', poured > brim);
    keptRow.textContent = `${litres(inBottle)} left in the bottle (${litres(fill)} at the start, ${ULLAGE_MM / 10} cm ullage)`;
    if (fill > brim) {      // a bigger bottle than its decanter: poured in turns (the swap)
      overflow.hidden = false;
      overflow.className = `badge ${poured > brim + 1 * ML ? 'bad' : 'info'}`;
      overflow.textContent = `full at ${turnsWhenPoured(brim).toFixed(1)} turns: turn back a `
        + 'turn and swap the decanter';
      overflow.title = 'Turn back about a turn until the stream stops, slide the decanter off '
        + 'its riser and out under the raised cradle box, and set the next one on the same '
        + 'riser (one whose stack is the same).';
    } else {
      overflow.hidden = true;
    }

    // the mouth: its lowest point on the glass, its lowest on the bore (where the wine leaves)
    const lowOuter = new THREE.Vector3(0, -mouthOuter * Math.sign(Math.cos(a) || 1), 0);
    frame.localToWorld(lowOuter);
    const bore = inside[0]![1];
    const lip = new THREE.Vector3(-0.5, -bore * Math.sign(Math.cos(a) || 1), 0);
    frame.localToWorld(lip);
    const rimZ = riser() + heightOf(vessel);
    const gap = lowOuter.z - rimZ;
    gapVal.textContent = `${gap.toFixed(0)} mm`;
    gapVal.classList.toggle('neg', gap < 0);
    if (gapLine) {
      const arr = gapLine.geometry.attributes.position as THREE.BufferAttribute;
      const x = lowOuter.x, y = lowOuter.y, t = 9;
      const v = [x, y, rimZ, x, y, lowOuter.z, x - t, y, rimZ, x + t, y, rimZ, x - t, y, lowOuter.z, x + t, y, lowOuter.z];
      for (let i = 0; i < v.length; i++) (arr.array as Float32Array)[i] = v[i]!;
      arr.needsUpdate = true;
      gapLine.geometry.computeBoundingSphere();
      const mid = new THREE.Vector3(x, y, (rimZ + lowOuter.z) / 2);
      stage.camera.updateMatrixWorld();
      mid.project(stage.camera);
      const r = stage.canvas.getBoundingClientRect();
      const visible = mid.z < 1 && Math.abs(mid.x) < 1 && Math.abs(mid.y) < 1;
      gapLabel.hidden = !visible;
      gapLabel.style.left = `${r.left + ((mid.x + 1) / 2) * r.width + 12}px`;
      gapLabel.style.top = `${r.top + ((1 - mid.y) / 2) * r.height - 10}px`;
      gapLabel.textContent = `${gap.toFixed(0)} mm`;
    }

    // the stream: when the bottle is giving wine at this turn (over the last ~half turn)
    const rate = (keptAt(Math.max(0, turns - 0.45)) - inBottle) / 0.45;   // mm³ per turn
    if (stream && rate > 2 * ML && turns <= motion.turns_to_full + 0.5) {
      const surfaceZ = stand ? riser() + (vesselWine?.visible
        ? heightFor(cavityOf(vessel), Math.min(poured, brim)) : cavityOf(vessel)[0]![0]) : 0;
      const axisOut = new THREE.Vector3(-1, 0, 0).transformDirection(frame.matrixWorld);   // out of the mouth
      const land = new THREE.Vector3(anchor.x, anchor.y, Math.min(surfaceZ, lip.z - 5));
      const horiz = new THREE.Vector3(axisOut.x, axisOut.y, 0);
      const rim = vessel.profile[vessel.profile.length - 1]![1];
      if (horiz.lengthSq() > 1e-6) land.addScaledVector(horiz.normalize(), Math.min(rim * 0.3, 14));
      const drop = lip.z - land.z;
      const curve = new THREE.CubicBezierCurve3(
        lip.clone(),
        lip.clone().addScaledVector(axisOut, 10),
        new THREE.Vector3(land.x, land.y, land.z + drop * 0.55),
        land,
      );
      const r0 = Math.min(6.5, Math.max(1.6, Math.sqrt(rate / ML) * 1.1));
      taperedTube(curve, (t) => r0 / Math.sqrt(1 + 1.2 * t), stream.geometry);
      stream.visible = true;
    } else if (stream) {
      stream.visible = false;
    }
  }

  function attach(l: LoadedScene, m: CradleMotion): void {
    detach();
    loaded = l;
    motion = m as PourMotion;
    look.dress(l);
    const bottle = l.root.getObjectByName('bottle') as THREE.Mesh | undefined;
    modelRiser = [];                                       // the picked vessel instead
    l.root.traverse((o) => {
      if (o.name === 'decanter') o.visible = false;
      else if (o.name.startsWith('decanter_riser')) { o.visible = false; modelRiser.push(o); }
    });
    if (!bottle || !m.bottle.outline.length) { onChange(); return; }

    // the cradle frame at clip time 0, in the bottle node's frame
    const time = l.action.time;
    l.action.time = 0;
    l.mixer.update(0);
    l.root.updateMatrixWorld(true);
    const lip0 = m.points[m.mouth]![0]!, tilt0 = (m.tilt_deg[0]! * Math.PI) / 180;
    bottle.geometry.computeBoundingBox();
    const centre = bottle.geometry.boundingBox!.getCenter(new THREE.Vector3()).applyMatrix4(bottle.matrixWorld);
    const model = l.walker.worldToLocal(centre.clone());
    const c0 = new THREE.Matrix4().makeRotationZ(tilt0).setPosition(lip0[0], lip0[1], model.z);
    const inWorld = l.walker.matrixWorld.clone().multiply(c0);
    const inBottle = bottle.matrixWorld.clone().invert().multiply(inWorld);
    frame = new THREE.Group();
    frame.name = 'cradle_frame';
    inBottle.decompose(frame.position, frame.quaternion, frame.scale);
    bottle.add(frame);
    const hidden = new THREE.MeshBasicMaterial({ visible: false });
    (bottle.material as THREE.Material).dispose?.();
    bottle.material = hidden;                    // drawn again below, as glass round the wine
    bottle.castShadow = false;

    // the meshes a bottle's setting moves, and the cradle's frame in each (at clip time 0)
    adjusted = [];
    drawn = m.bottle.setting;
    baseSaddles = m.saddles.map((sd) => ({ ...sd }));
    for (const group of ['front', 'rear', 'stop'] as const) {
      for (const n of m.adjust?.[group] ?? []) {
        l.root.getObjectByName(THREE.PropertyBinding.sanitizeNodeName(n))?.traverse((o) => {
          const mesh = o as THREE.Mesh;
          if (!mesh.isMesh) return;
          const inMesh = new THREE.Matrix3().setFromMatrix4(mesh.matrixWorld.clone().invert().multiply(inWorld));
          adjusted.push({ mesh, base: mesh.geometry, group, inMesh });
        });
      }
    }

    bottlePick.replaceChildren();
    for (const b of m.bottles ?? []) {
      const o = el('option', '', b.name);
      o.value = b.key;
      bottlePick.appendChild(o);
    }
    bottleRow.hidden = !bottlePick.options.length;
    const wanted = m.bottles?.find((b) => b.key === (params.get('bottle') ?? DEFAULT_BOTTLE));
    if (wanted) m.bottle = { ...wanted };
    if (!vesselByHand) pickVessel(m.bottle.name);       // a decanter that holds it
    bottlePick.value = m.bottles?.find((b) => b.name === m.bottle.name)?.key ?? '';
    applySetting(m.bottle.setting);
    buildBottle(m);

    // the vessel: under the mouth (posed at full tilt), on the table
    l.action.time = fractionAtTurns(m, m.turns_to_full) * l.clipDuration;
    l.mixer.update(0);
    l.root.updateMatrixWorld(true);
    const mouth = frame.getWorldPosition(new THREE.Vector3());
    anchor = new THREE.Vector3(mouth.x, mouth.y, 0);
    mouthZ = mouth.z;                            // the mouth's centre over the table, from the posed scene
    riserByHand = false;
    setStack(defaultStack() ?? [0, []]);
    buildStand();
    stream = new THREE.Mesh(new THREE.BufferGeometry(), streamMat);
    stream.name = 'pour_stream';
    stream.visible = false;
    stream.castShadow = true;
    stage.scene.add(stream);        // (outside the cradle's root: not in its framing box)
    gapLine = new THREE.LineSegments(
      new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(new Float32Array(18), 3)),
      new THREE.LineBasicMaterial({ color: '#ffd27a', depthTest: false, transparent: true, opacity: 0.9 }),
    );
    gapLine.renderOrder = 10;
    gapLine.frustumCulled = false;
    stage.scene.add(gapLine);

    // the table, the light and the shadows round the cradle (at rest)
    l.action.time = fractionAtTurns(m, 0) * l.clipDuration;
    l.mixer.update(0);
    l.root.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(l.root);
    look.fit(box);
    l.action.time = time;
    l.mixer.update(0);
    pose(lastF);
    onChange();
  }

  function detach(): void {
    gapLabel.hidden = true;
    for (const o of [stream, gapLine]) {
      if (!o) continue;
      stage.scene.remove(o);
      o.geometry.dispose();
    }
    if (!loaded) return;
    // the rest goes with the scene (``teardown`` disposes everything under its root)
    for (const a of adjusted) if (a.mesh.geometry !== a.base) a.mesh.geometry.dispose();
    adjusted = [];
    loaded = null; motion = null; frame = null; wineBody = null; wineTop = null;
    stand = null; vesselWine = null; stream = null; gapLine = null;
  }

  bottlePick.addEventListener('change', () => setBottle(bottlePick.value));
  pick.addEventListener('change', () => {
    vessel = VESSELS.find((v) => v.key === pick.value) ?? vessel;
    vesselByHand = true;
    riserByHand = false;
    setStack(defaultStack() ?? [0, []]);
    const url = new URL(location.href);
    url.searchParams.set('vessel', vessel.key);
    history.replaceState(null, '', url);
    buildStand();
    pose(lastF);
    onChange();
  });
  printSel.addEventListener('change', () => {
    look.setPrintColor(printSel.value);
    const url = new URL(location.href);
    url.searchParams.set('print', printSel.value);
    history.replaceState(null, '', url);
  });

  function update(f: number): void {
    pose(f);
    if (watchPending && frame) { watchPending = false; framePour(); }
  }

  /** The bottle drawn round the wine (``m.bottle``), its wine full, and what it keeps
   * over the swing; again when the bottle's size changes (:func:`setBottle`). */
  function buildBottle(m: CradleMotion): void {
    if (!frame) return;
    for (const o of [...frame.children]) {
      frame.remove(o);
      (o as THREE.Mesh).geometry?.dispose();
    }
    const outside = bottleOutside(m.bottle.outline);
    mouthOuter = outside[0]![1];
    inside = bottleInside(m.bottle.outline);
    wineProfile = inset(inside, WINE_INSET);
    fill = startingFill(inside, nominalVolume(m.bottle.name, 750), ULLAGE_MM);
    const glass = new THREE.Mesh(glassSolid(outside, inside, 'x', 96, 'end'), look.materials.glassGreen);
    glass.castShadow = true;
    glass.renderOrder = 2;
    glass.name = 'bottle_glass';
    frame.add(glass);
    const body = Math.max(...outside.map(([, r]) => r));
    const sEnd = outside[outside.length - 1]![0];
    const span = Math.min(130, sEnd * 0.45);
    frame.add(labelMesh(body + 0.35, sEnd - span - 45, sEnd - 45, sizeLine(m.bottle.name)));
    wineBody = new THREE.Mesh(solid(wineProfile, 'x', 72), wineClipped);
    wineBody.name = 'wine';
    wineTop = new THREE.Mesh(new THREE.BufferGeometry(), topMat);
    wineTop.name = 'wine_surface';
    frame.add(wineBody, wineTop);
    const tilts: number[] = [];
    for (let k = 0; k < m.n; k++) tilts.push(m.tilt_deg[(m.rest + k) % m.n]!);
    kept = keptFromRest(inside, tilts, fill);
  }

  /** The V blocks and the base stop moved to setting ``to`` (from the one the model is drawn
   * at), and the diagram's saddles with them. */
  function applySetting(to: Setting | undefined): void {
    if (!motion) return;
    const o = settingOffsets(drawn, to);
    for (const a of adjusted) {
      if (a.mesh.geometry !== a.base) a.mesh.geometry.dispose();
      const [dx, dy] = adjustShift(a.group, o);
      if (!dx && !dy) { a.mesh.geometry = a.base; continue; }
      const v = new THREE.Vector3(dx, dy, 0).applyMatrix3(a.inMesh);
      a.mesh.geometry = a.base.clone().translate(v.x, v.y, v.z);
    }
    motion.saddles = baseSaddles.map((sd) => saddleAt(sd, o));
  }

  /** Swap the bottle in the cradle (``key`` one of ``motion.bottles``). */
  function setBottle(key: string): void {
    const b = motion?.bottles?.find((x) => x.key === key);
    if (!motion || !b) return;
    motion.bottle = { ...b };
    applySetting(b.setting);
    buildBottle(motion);
    if (!vesselByHand && pickVessel(b.name)) {
      riserByHand = false;
      setStack(defaultStack() ?? [0, []]);
      buildStand();
    }
    pose(lastF);
    onChange();
  }

  function setRoom(room: RoomData): void {
    if (!motion) return;
    motion.room = room;
    if (!riserByHand) setStack(defaultStack() ?? [0, []]);
    buildStand();
    pose(lastF);
    onChange();
  }

  function setVisible(v: boolean): void {
    for (const o of [stand, stream, gapLine, frame]) if (o) o.visible = v;
    for (const o of modelRiser) o.visible = !v;          // (it packs: the flat pack shows it)
    gapLabel.hidden = !v || gapLabel.hidden;
    onChange();
  }

  return { look, attach, detach, update, setRoom, setVisible, setBottle };
}
