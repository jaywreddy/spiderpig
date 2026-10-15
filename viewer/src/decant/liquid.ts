/**
 * The wine's maths, pure (no three.js): solids of revolution given as a profile, how much
 * of one lies under a level plane at a tilt, the level that holds a given volume, what a
 * tipping bottle keeps and what it pours, and how high the wine stands in an upright
 * vessel.
 *
 * Frames. A bottle's profile runs along its axis from the mouth (``s = 0``) to the base;
 * a point of the solid is ``(s, u, w)``: ``s`` along the axis, ``u`` across it in the
 * linkage's plane, ``w`` out of the plane (the cradle frame of ``motion.ts``, ``x = s``,
 * ``y = u``). At a tilt ``a`` (radians, neck down positive) the cradle frame is turned by
 * ``a`` in the plane, so a point's height over the mouth is ``sin(a) s + cos(a) u``: the
 * level of a plane "``height = c``" is ``c``, millimetres over the mouth's centre.
 *
 * Volumes are in mm³ (1 mL = 1000 mm³).
 */

/** ``[[axial_mm, radius_mm], ...]``, the axial coordinate increasing. */
export type Profile = readonly (readonly [number, number])[];

export const ML = 1000;

/** The radius at ``s`` (linear between the profile's points; 0 outside it). */
export function radiusAt(p: Profile, s: number): number {
  const n = p.length;
  if (n === 0 || s < p[0]![0] || s > p[n - 1]![0]) return 0;
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (p[mid]![0] <= s) lo = mid; else hi = mid;
  }
  const [s0, r0] = p[lo]!, [s1, r1] = p[hi]!;
  return s1 > s0 ? r0 + ((r1 - r0) * (s - s0)) / (s1 - s0) : Math.max(r0, r1);
}

/** The solid's volume, frustum by frustum. */
export function profileVolume(p: Profile): number {
  let v = 0;
  for (let i = 1; i < p.length; i++) {
    const [s0, r0] = p[i - 1]!, [s1, r1] = p[i]!;
    v += (Math.PI * (s1 - s0) * (r0 * r0 + r0 * r1 + r1 * r1)) / 3;
  }
  return v;
}

/** The volume of the profile's solid from its start up to axial ``s`` (an upright vessel
 * filled to height ``s``). */
export function volumeTo(p: Profile, s: number): number {
  let v = 0;
  for (let i = 1; i < p.length; i++) {
    const [s0, r0] = p[i - 1]!;
    if (s <= s0) break;
    const s1 = Math.min(s, p[i]![0]);
    const r1 = radiusAt(p, s1);
    v += (Math.PI * (s1 - s0) * (r0 * r0 + r0 * r1 + r1 * r1)) / 3;
  }
  return v;
}

/** The height an upright vessel's cavity (``[[height, radius], ...]`` from its floor) is
 * filled to by ``volume``; its top if it overflows. */
export function heightFor(p: Profile, volume: number): number {
  const n = p.length;
  if (n < 2 || volume <= 0) return n ? p[0]![0] : 0;
  let lo = p[0]![0], hi = p[n - 1]![0];
  if (volumeTo(p, hi) <= volume) return hi;
  for (let k = 0; k < 50; k++) {
    const mid = (lo + hi) / 2;
    if (volumeTo(p, mid) < volume) lo = mid; else hi = mid;
  }
  return (lo + hi) / 2;
}

/** The area of a disc of radius ``r`` (centre at ``u = 0``) where ``u < d``. */
export function discBelow(r: number, d: number): number {
  if (r <= 0 || d <= -r) return 0;
  if (d >= r) return Math.PI * r * r;
  return r * r * Math.acos(-d / r) + d * Math.sqrt(r * r - d * d);
}

/** Slices along the profile: their mid ``s``, length and mid radius (at most ``step`` mm). */
function slices(p: Profile, step: number): { s: number; ds: number; r: number }[] {
  const out: { s: number; ds: number; r: number }[] = [];
  for (let i = 1; i < p.length; i++) {
    const [s0, r0] = p[i - 1]!, [s1, r1] = p[i]!;
    const len = s1 - s0;
    if (len <= 0) continue;
    const k = Math.max(1, Math.ceil(len / step));
    const ds = len / k;
    for (let j = 0; j < k; j++) {
      const t = (j + 0.5) / k;
      // the slice's mean r² as the frustum's (exact for a linear radius)
      const ra = r0 + (r1 - r0) * (j / k), rb = r0 + (r1 - r0) * ((j + 1) / k);
      out.push({ s: s0 + len * t, ds, r: Math.sqrt((ra * ra + ra * rb + rb * rb) / 3) });
    }
  }
  return out;
}

/** The part of the solid under the plane ``sin(a) s + cos(a) u = c``. */
export function volumeBelow(p: Profile, a: number, c: number, step = 1.5): number {
  const sa = Math.sin(a), ca = Math.cos(a);
  let v = 0;
  const aca = Math.abs(ca);
  for (const { s, ds, r } of slices(p, step)) {
    if (aca < 1e-9) {
      // the axis upright: the plane crosses the slice across, at s = c / sin(a)
      const cut = c / sa, s0 = s - ds / 2;
      const inside = Math.min(1, Math.max(0, (cut - s0) / ds));
      v += Math.PI * r * r * ds * (sa > 0 ? inside : 1 - inside);
      continue;
    }
    // u < h / cos(a) (cos > 0), u > h / cos(a) (cos < 0: the disc's mirror image); the
    // chord's offset runs (sin a / cos a) ds over the slice: sample it finely enough
    const h = c - sa * s;                 // the plane's height over this slice's centre
    const dd = (Math.abs(sa) * ds) / aca;
    const m = Math.min(64, Math.max(1, Math.ceil(dd / Math.max(0.5, r * 0.25))));
    let area = 0;
    for (let j = 0; j < m; j++) {
      const off = ((j + 0.5) / m - 0.5) * ds;
      area += discBelow(r, (h - sa * off) / aca);
    }
    v += (area / m) * ds;
  }
  return v;
}

/** The lowest and highest level over the solid at tilt ``a``. */
export function levelRange(p: Profile, a: number): [number, number] {
  const sa = Math.sin(a), ca = Math.abs(Math.cos(a));
  let lo = Infinity, hi = -Infinity;
  for (const [s, r] of p) {
    lo = Math.min(lo, sa * s - ca * r);
    hi = Math.max(hi, sa * s + ca * r);
  }
  return [lo, hi];
}

/** The level ``c`` whose plane leaves ``volume`` under it at tilt ``a`` (the free surface
 * of that much wine). */
export function levelFor(p: Profile, a: number, volume: number): number {
  let [lo, hi] = levelRange(p, a);
  if (volume <= 0) return lo;
  for (let k = 0; k < 44; k++) {
    const mid = (lo + hi) / 2;
    if (volumeBelow(p, a, mid) < volume) lo = mid; else hi = mid;
  }
  return (lo + hi) / 2;
}

/** The level of the opening's lowest point: the bore's rim at the profile's start. */
export function mouthLevel(p: Profile, a: number): number {
  const [s0, r0] = p[0]!;
  return Math.sin(a) * s0 - Math.abs(Math.cos(a)) * r0;
}

/** What the bottle holds at tilt ``a``: everything above its mouth's lowest point pours. */
export function heldAt(p: Profile, a: number): number {
  return volumeBelow(p, a, mouthLevel(p, a));
}

/** A bottle's inside from its outline (``motion.bottle.outline``: a closed polygon in the
 * cradle frame, ``x`` from the mouth to the base, ``y`` = ±radius): the upper half as a
 * profile, less the glass (``wall`` mm on the body, thicker in the neck, where the bore is
 * ~18.5 mm across) and ``base`` mm of base and punt. */
export function bottleInside(
  outline: readonly (readonly [number, number])[], wall = 3.6, base = 10,
): Profile {
  const upper = outline.filter(([, y]) => y >= 0).map(([x, y]) => [x, y] as [number, number]);
  upper.sort((a, b) => a[0] - b[0]);
  const prof: [number, number][] = [];
  for (const [x, r] of upper) {
    const last = prof[prof.length - 1];
    if (last && Math.abs(last[0] - x) < 1e-6) { last[1] = Math.max(last[1], r); continue; }
    prof.push([x, r]);
  }
  if (prof.length < 2) return [];
  const end = prof[prof.length - 1]![0] - base;
  const neckWall = (r: number): number => wall + 2.4 * Math.min(1, Math.max(0, (32 - r) / 14));
  const out: [number, number][] = [];
  for (const [x, r] of prof) {
    if (x > end) break;
    out.push([x, Math.max(1, r - neckWall(r))]);
  }
  out.push([end, radiusAt(prof, end) - neckWall(radiusAt(prof, end))]);
  return out;
}

/** The nominal volume in a bottle's name ("1.5 L Bordeaux magnum": 1500 mL), in mm³. */
export function nominalVolume(name: string, fallbackMl = 750): number {
  const m = /(\d+(?:\.\d+)?)\s*(m?l)\b/i.exec(name);
  if (!m) return fallbackMl * ML;
  const v = Number(m[1]);
  return (m[2]!.toLowerCase() === 'ml' ? v : v * 1000) * ML;
}

/** The fill: ``nominal`` if the bottle holds it below an ``ullage`` (mm, from the mouth
 * along the axis, upright); else what does hold there. */
export function startingFill(inside: Profile, nominal: number, ullage = 60): number {
  const below = profileVolume(inside) - volumeTo(inside, ullage);
  return Math.min(nominal, below);
}

/** What a swing keeps in the bottle, sample by sample from rest: the wine poured so far
 * is gone (``held`` is the most it can hold at each tilt), so the bottle keeps the least
 * it held at any tilt since rest. ``tilts`` in degrees, ``tilts[0]`` at rest. */
export function keptFromRest(p: Profile, tiltsDeg: readonly number[], fill: number): Float64Array {
  const out = new Float64Array(tiltsDeg.length);
  let kept = fill;
  tiltsDeg.forEach((t, k) => {
    kept = Math.min(kept, heldAt(p, (t * Math.PI) / 180));
    out[k] = kept;
  });
  return out;
}

/** The surface of the wine at tilt ``a`` and level ``c``: the plane's cut through the
 * solid as runs of chords. A chord ``[s, u, half]`` spans ``w`` in ``±half`` at ``(s, u)``;
 * consecutive chords of a run bound one strip of the surface. */
export function surfaceRuns(p: Profile, a: number, c: number, step = 1.5): [number, number, number][][] {
  if (p.length < 2) return [];
  const sa = Math.sin(a), ca = Math.cos(a);
  const s0 = p[0]![0], s1 = p[p.length - 1]![0];
  const rmax = Math.max(...p.map(([, r]) => r));
  // walk the plane along its line in (s, u): (s, u) = c (sa, ca) + t (ca, -sa)
  const at = (t: number): [number, number] => [c * sa + t * ca, c * ca - t * sa];
  const half = (t: number): number => {
    const [s, u] = at(t);
    if (s < s0 || s > s1) return -1;
    const r = radiusAt(p, s);
    return r * r - u * u;          // > 0 inside
  };
  const span = Math.abs(s1) + Math.abs(s0) + rmax + Math.abs(c) + 1;
  const n = Math.ceil((2 * span) / step);
  const runs: [number, number, number][][] = [];
  let run: [number, number, number][] | null = null;
  let prevT = -span, prevIn = half(prevT) > 0;
  const chord = (t: number): [number, number, number] => {
    const [s, u] = at(t);
    return [s, u, Math.sqrt(Math.max(0, half(t)))];
  };
  /** the boundary between ``tOut`` (outside) and ``tIn`` (inside) */
  const edge = (tOut: number, tIn: number): number => {
    for (let k = 0; k < 30; k++) {
      const mid = (tOut + tIn) / 2;
      if (half(mid) > 0) tIn = mid; else tOut = mid;
    }
    return tIn;
  };
  for (let i = 1; i <= n; i++) {
    const t = -span + (2 * span * i) / n;
    const inside = half(t) > 0;
    if (inside && !prevIn) {
      run = [chord(edge(prevT, t))];
      runs.push(run);
    }
    if (inside && run) run.push(chord(t));
    if (!inside && prevIn && run) {
      run.push(chord(edge(t, prevT)));
      run = null;
    }
    prevT = t; prevIn = inside;
  }
  return runs.filter((r) => r.length >= 2);
}

/** A room under the mouth: ``[[height_over_table_mm, max_radius_mm], ...]``. */
export type Room = readonly (readonly [number, number])[];

/** Whether a vessel (``[[height, radius], ...]``, base to rim) on a riser of ``riser`` mm
 * fits the room under the mouth: every point of it within the room's radius at its
 * height, and nothing above the room's top. The worst point's margin (mm; negative: it
 * doesn't fit) comes back too. */
export function fitsRoom(
  vessel: Profile, riser: number, room: Room,
): { fits: boolean; margin: number; at: number } {
  const rows = [...room].sort((a, b) => a[0] - b[0]);
  if (!rows.length || !vessel.length) return { fits: true, margin: Infinity, at: 0 };
  const top = rows[rows.length - 1]![0];
  const roomAt = (h: number): number => {
    if (h > top + 1e-6) return -Infinity;
    if (h <= rows[0]![0]) return rows[0]![1];
    return radiusAt(rows, h);
  };
  let margin = Infinity, at = 0;
  const probe = (h: number, r: number): void => {
    const room = roomAt(h + riser);
    const m = room === -Infinity ? top - (h + riser) : room - r;
    if (m < margin) { margin = m; at = h + riser; }
  };
  for (let i = 0; i < vessel.length; i++) {
    const [h, r] = vessel[i]!;
    probe(h, r);
    const next = vessel[i + 1];
    if (next) for (let k = 1; k < 4; k++) {    // between the points, against the room's corners
      const t = k / 4;
      probe(h + (next[0] - h) * t, r + (next[1] - r) * t);
    }
  }
  for (const [h] of rows) {                     // the room's own corners, over the vessel
    const hv = h - riser;
    if (hv >= vessel[0]![0] && hv <= vessel[vessel.length - 1]![0]) probe(hv, radiusAt(vessel, hv));
  }
  return { fits: margin >= 0, margin, at };
}

/** The one rule for a vessel's height (``construction.cradle.RISER_CLEAR``, the API's
 * ``room.rule``; ``cradle_report.riser_fits`` is its Python twin): on ``riser`` the vessel
 * keeps ``side`` mm off the room's side at every height, and raised ``up`` mm more it still
 * fits, so its rim stands ``up`` mm under the lowest part over it (the light's clip, at
 * rest). Rest is a toggle dwell: whatever the box comes down on there, it crushes. */
export interface RiserRule { up: number; side: number }

export const RISER_RULE: RiserRule = { up: 20, side: 10 };

export function riserFits(vessel: Profile, riser: number, room: Room,
  rule: RiserRule = RISER_RULE): boolean {
  return fitsRoom(vessel, riser, room).margin >= rule.side
    && fitsRoom(vessel, riser + rule.up, room).margin >= 0;
}

/** The highest riser (whole mm, ``0..max``) the rule (:func:`riserFits`) lets ``vessel``
 * stand on; ``null`` when not even the table does. */
export function bestRiser(vessel: Profile, room: Room, max: number,
  rule: RiserRule = RISER_RULE): number | null {
  for (let r = Math.floor(max); r >= 0; r--) {
    if (riserFits(vessel, r, room, rule)) return r;
  }
  return null;
}

/** A printed riser stack: its height (mm) and its pieces bottom up (``"blocks"``, a plate's
 * thickness), as ``cradle_report.riser_stacks`` lists them. */
export type Stack = readonly [number, readonly string[]];

/** The highest of ``stacks`` no taller than ``mm`` (none: 0 mm, on the table). */
export function riserStack(mm: number | null, stacks: readonly Stack[]): Stack {
  let best: Stack = [0, []];
  if (mm === null) return best;
  for (const s of stacks) if (s[0] <= mm + 1e-9 && s[0] >= best[0]) best = s;
  return best;
}

/** A stack in words: ``the two blocks, 20, 10 and 5 mm plates (122 mm)``. */
export function stackWords(s: Stack): string {
  if (!s[1].length) return 'none: on the table, in the locator';
  const plates = s[1].filter((p) => p !== 'blocks');
  const parts: string[] = [];
  if (s[1].includes('blocks')) parts.push('the two blocks');
  if (plates.length) {
    const list = plates.length > 1 ? `${plates.slice(0, -1).join(', ')} and ${plates[plates.length - 1]}` : plates[0]!;
    parts.push(`the ${list} mm plate${plates.length > 1 ? 's' : ''}`);
  }
  return `${parts.join(', ')} (${s[0].toFixed(0)} mm)`;
}
