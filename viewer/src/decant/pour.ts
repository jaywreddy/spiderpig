/**
 * The pour on the cradle page: the bottle drawn as glass round the wine it holds, the
 * wine's surface level in the world as the cradle tips it, the stream from the mouth once
 * the wine reaches it, and the vessel under the mouth (a decanter or a glass picked from
 * ``vessels.ts``, on the table or a riser) filling as it pours. A small panel picks the
 * vessel and the print colour and reads out the gap from the mouth to the rim, whether
 * the vessel fits the room under the mouth (``motion.room``, when the API sends it) and
 * how much has poured.
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
import { fractionAtTurns, turnsAt, type CradleMotion } from './motion';
import {
  bestRiser, bottleInside, fitsRoom, heightFor, keptFromRest, levelFor, nominalVolume, radiusAt,
  startingFill, surfaceRuns, ML, type Profile, type Room,
} from './liquid';
import {
  DEFAULT_VESSEL, VESSELS, brimMl, cavityOf, heightOf, maxRadiusOf, type Vessel,
} from './vessels';
import { createLook, PRINT_COLORS, type Look } from './realism';
import './pour.css';

/** What the API may add to the swing for the pour (optional). */
interface PourMotion extends CradleMotion {
  /** a riser under the vessel, mm */
  riser_mm?: number;
  /** the room under the mouth: ``[[height_over_table_mm, max_radius_mm], ...]`` */
  room?: [number, number][];
  /** the room once the cradle is past -20° (the pour), the same form */
  room_pouring?: [number, number][];
}

export interface Pour {
  look: Look;
  attach(loaded: LoadedScene, motion: CradleMotion): void;
  detach(): void;
  update(f: number): void;
  /** The room under the mouth arrived (``/api/cradle/{key}/room``): re-pick the riser. */
  setRoom(room: [number, number][], pouring: [number, number][]): void;
  /** Show or hide the vessel, the wine and the stream (the flat pack hides them). */
  setVisible(v: boolean): void;
}

const ULLAGE_MM = 60;
/** The riser's range, and where it puts a vessel's rim by default: under the mouth by a
 * little more than the front saddle comes down at rest (~70 mm). */
const RISER_MAX = 160, RIM_UNDER_MOUTH = 85;
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
function labelMesh(radius: number, s0: number, s1: number): THREE.Mesh {
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
  ctx.fillText('MAGNUM · 1,5 L', 256, 445);
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

/** A lab jack ``h`` mm tall: a 100 × 100 mm platform on a scissor over a base plate, the
 * screw across the scissor's middle and its knob (a plain plate when too low to scissor). */
function labJack(h: number, steel: THREE.Material, alu: THREE.Material): THREE.Group {
  const g = new THREE.Group();
  g.name = 'lab_jack';
  const W = 100, plate = 4;
  const add = (m: THREE.Mesh): void => { m.castShadow = m.receiveShadow = true; g.add(m); };
  const top = new THREE.Mesh(new RoundedBoxGeometry(W, W, plate, 2, 1.2), alu);
  top.position.z = h - plate / 2;
  add(top);
  if (h < 3 * plate + 6) { top.scale.z = h / plate; top.position.z = h / 2; return g; }
  const base = new THREE.Mesh(new RoundedBoxGeometry(W, W * 0.9, plate, 2, 1.2), alu);
  base.position.z = plate / 2;
  add(base);
  // the scissor: an X each side (y = ±38), its bars pinned at the plates' edges
  const z0 = plate + 3, z1 = h - plate - 3, span = W - 18, rise = z1 - z0;
  const len = Math.hypot(span, rise), ang = Math.atan2(rise, span);
  for (const y of [-38, 38]) {
    for (const sgn of [1, -1]) {
      const bar = new THREE.Mesh(new THREE.BoxGeometry(len, 3, 9), steel);
      bar.position.set(0, y + sgn * 2, (z0 + z1) / 2);
      bar.rotation.y = -sgn * ang;
      add(bar);
    }
    for (const [x, z] of [[-span / 2, z0], [span / 2, z0], [-span / 2, z1], [span / 2, z1]] as const) {
      const pin = new THREE.Mesh(new THREE.CylinderGeometry(3, 3, 12, 16), steel);
      pin.position.set(x, y, z);
      add(pin);
    }
  }
  // the screw through the scissor's middle and its knurled knob
  const screw = new THREE.Mesh(new THREE.CylinderGeometry(3, 3, W + 30, 16), steel);
  screw.rotation.z = Math.PI / 2;
  screw.position.set(-15, 0, (z0 + z1) / 2);
  add(screw);
  const knob = new THREE.Mesh(new THREE.CylinderGeometry(11, 11, 12, 24), alu);
  knob.rotation.z = Math.PI / 2;
  knob.position.set(-(W / 2 + 30), 0, (z0 + z1) / 2);   // out to the cradle's open end
  add(knob);
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
  riserIn.max = String(RISER_MAX);
  riserIn.step = '1';
  riserIn.title = 'The lab jack under the vessel (0: on the table)';
  const riserVal = el('span', 'v');
  riserRow.append(el('span', 'k', 'Lab jack'), riserIn, riserVal);
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
  const printRow = el('label', 'pick');
  printRow.append(el('span', 'k', 'Prints'), printSel);
  const watch = el('button', 'watch', 'Watch the pour');
  watch.title = 'Frame the camera on the mouth and the vessel (from the mouth’s side)';
  printRow.appendChild(watch);
  panel.append(pickRow, facts, riserRow, gapRow, pourRow, bar, keptRow, printRow);
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

  // the riser under the vessel: the API's, else the rim RIM_UNDER_MOUTH under the mouth;
  // the panel's slider sets it by hand (kept across vessels once set)
  let riserMm = 0, riserByHand = false, mouthZ = 0;
  const riser = (): number => riserMm;
  function defaultRiser(): number {
    if (motion?.riser_mm !== undefined) return Math.max(0, motion.riser_mm);
    if (motion?.room?.length) return bestRiser(vessel.profile, motion.room, RISER_MAX) ?? 0;
    return Math.min(RISER_MAX, Math.max(0, mouthZ - RIM_UNDER_MOUTH - heightOf(vessel)));
  }
  /** How high the jack may go once the cradle is past -20° (the pour), if higher. */
  function pourRiser(): number | null {
    const room = motion?.room_pouring;
    return room?.length ? bestRiser(vessel.profile, room, RISER_MAX) : null;
  }
  function setRiser(mm: number): void {
    riserMm = Math.round(Math.min(RISER_MAX, Math.max(0, mm)));
    riserIn.value = String(riserMm);
    riserVal.textContent = riserMm > 0 ? `${riserMm} mm` : 'none';
  }
  riserIn.addEventListener('input', () => {
    riserByHand = true;
    setRiser(Number(riserIn.value));
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
    if (h0 > 0) stand.add(labJack(h0, look.materials.steel, look.materials.alu));
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
    if (room && room.length) {
      const fit = fitsRoom(vessel.profile, h0, room);
      badge.hidden = false;
      badge.className = `badge ${fit.fits ? 'ok' : 'bad'}`;
      badge.textContent = fit.fits ? 'fits the whole swing'
        : `hit at rest (${(-fit.margin).toFixed(0)} mm at ${fit.at.toFixed(0)} mm)`;
      const up = pourRiser();
      if (up !== null && up > h0 + 5) {
        const gap = mouthZ - (up + heightOf(vessel));
        facts.appendChild(el('div', 'note', `Pouring closer: once the cradle is past −20°, `
          + `raise the jack to ${up} mm (the rim ${gap.toFixed(0)} mm under the mouth); lower it `
          + `before you crank back to rest, where the cradle box hangs over it.`));
      }
    } else {
      badge.hidden = true;
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
    if (poured > brim + 1 * ML) {
      overflow.hidden = false;
      overflow.className = 'badge bad';
      overflow.textContent = `overflows: full at ${turnsWhenPoured(brim).toFixed(1)} turns`;
    } else if (fill > brim) {
      overflow.hidden = false;
      overflow.className = 'badge info';
      overflow.textContent = `full at ${turnsWhenPoured(brim).toFixed(1)} turns`;
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
    for (const n of ['decanter', 'decanter_riser']) {    // the picked vessel instead
      const o = l.root.getObjectByName(n);
      if (o) o.visible = false;
    }
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

    // the bottle, the wine
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
    frame.add(labelMesh(body + 0.35, sEnd - 175, sEnd - 45));
    wineBody = new THREE.Mesh(solid(wineProfile, 'x', 72), wineClipped);
    wineBody.name = 'wine';
    wineTop = new THREE.Mesh(new THREE.BufferGeometry(), topMat);
    wineTop.name = 'wine_surface';
    frame.add(wineBody, wineTop);

    // what it keeps over the swing, from rest
    const tilts: number[] = [];
    for (let k = 0; k < m.n; k++) tilts.push(m.tilt_deg[(m.rest + k) % m.n]!);
    kept = keptFromRest(inside, tilts, fill);

    // the vessel: under the mouth (posed at full tilt), on the table
    l.action.time = fractionAtTurns(m, m.turns_to_full) * l.clipDuration;
    l.mixer.update(0);
    l.root.updateMatrixWorld(true);
    const mouth = frame.getWorldPosition(new THREE.Vector3());
    anchor = new THREE.Vector3(mouth.x, mouth.y, 0);
    mouthZ = mouth.z;                            // the mouth's centre over the table, from the posed scene
    const asked = Number(params.get('riser'));
    riserByHand = params.has('riser') && Number.isFinite(asked);
    setRiser(riserByHand ? asked : defaultRiser());
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
    loaded = null; motion = null; frame = null; wineBody = null; wineTop = null;
    stand = null; vesselWine = null; stream = null; gapLine = null;
  }

  pick.addEventListener('change', () => {
    vessel = VESSELS.find((v) => v.key === pick.value) ?? vessel;
    if (!riserByHand) setRiser(defaultRiser());
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

  function setRoom(room: [number, number][], pouring: [number, number][]): void {
    if (!motion) return;
    motion.room = room;
    motion.room_pouring = pouring;
    if (!riserByHand) setRiser(defaultRiser());
    buildStand();
    pose(lastF);
    onChange();
  }

  function setVisible(v: boolean): void {
    for (const o of [stand, stream, gapLine, frame]) if (o) o.visible = v;
    gapLabel.hidden = !v || gapLabel.hidden;
    onChange();
  }

  return { look, attach, detach, update, setRoom, setVisible };
}
