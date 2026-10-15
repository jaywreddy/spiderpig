/**
 * A decanting cradle's swing as ``/api/cradle/{key}`` sends it
 * (``spiderpig.cradle_report.motion``): ``n`` crank angles over one crank turn, sampled as
 * the cradle's baked clip is (frame ``i`` at clip time ``i / n`` of its length). Pure
 * helpers: a clip fraction ``f`` (0..1) to points, tilt, handle turns and phase, and the
 * cradle's frame (a point ``(x, y)`` there is at ``mouth + R(tilt) (x, y)``).
 */

export type XY = readonly [number, number];

export interface CradleMotion {
  key: string;
  name: string;
  notes: string;
  source: string;
  verdict: 'build' | 'study';
  why: string;
  labels: Record<string, string>;
  frame: string[];
  crank: string[];
  mouth: string;
  axis: string;
  output_link: string | null;
  links: Record<string, { joints: string[]; edges: [string, string][] }>;
  points: Record<string, XY[]>;
  tilt_deg: number[];
  handle_turns: number[];
  rest: number;
  full: number;
  n: number;
  rest_deg: number;
  full_deg: number;
  turns_to_full: number;
  ratio: number;
  mouth_wander_mm: number;
  /** the bottle the model draws (its V blocks at its setting) */
  bottle: { key?: string; name: string; outline: XY[]; setting?: Setting };
  /** every bottle the cradle takes, each with its setting */
  bottles?: { key: string; name: string; outline: XY[]; setting?: Setting }[];
  /** the model's nodes a setting moves: with the front V block, with the rear one, with the
   * rear one and along the axis (the base stop's gate) */
  adjust?: { front: string[]; rear: string[]; stop: string[] };
  /** the V blocks (cradle frame, at the drawn bottle's setting) */
  saddles: { name: string; x: [number, number]; y: [number, number]; moves?: 'front' | 'rear' }[];
  ties: Record<string, XY>;
  /** how it's used: the guide's use steps, each its title and lines (``cradle.use_steps``;
   * ``limits``'s first five lines are the card) */
  use?: { tag: string; title: string; lines: string[] }[];
}

/** A bottle's setting (``cradle_report._setting_row``, mm): the front V block's rise over its
 * lowest, the rear one's, the base stop's face along the axis from the mouth. */
export type Setting = readonly [number, number, number];

/** How far each adjustable part moves going from setting ``from`` to ``to``. */
export interface Offsets { front: number; rear: number; stop: number }

export function settingOffsets(from: Setting | undefined, to: Setting | undefined): Offsets {
  if (!from || !to) return { front: 0, rear: 0, stop: 0 };
  return { front: to[0] - from[0], rear: to[1] - from[1], stop: to[2] - from[2] };
}

/** A moved part's shift in the cradle's frame, ``[along the axis, up]``: a V block's straight
 * up; the base stop's gate up with the rear one and along to its place. */
export function adjustShift(group: 'front' | 'rear' | 'stop', o: Offsets): XY {
  if (group === 'front') return [0, o.front];
  if (group === 'rear') return [0, o.rear];
  return [o.stop, o.rear];
}

/** A V block's outline set ``o`` from the drawn bottle's setting. */
export function saddleAt(s: CradleMotion['saddles'][number], o: Offsets): CradleMotion['saddles'][number] {
  const dy = s.moves === 'front' ? o.front : s.moves === 'rear' ? o.rear : 0;
  return { ...s, y: [s.y[0] + dy, s.y[1] + dy] };
}

/** The two samples around clip fraction ``f`` and the weight of the second. */
export function bracket(n: number, f: number): { i0: number; i1: number; w: number } {
  const u = (((f % 1) + 1) % 1) * n;
  const i0 = Math.floor(u) % n;
  return { i0, i1: (i0 + 1) % n, w: u - Math.floor(u) };
}

/** Joint ``name`` at clip fraction ``f`` (linear between the samples). */
export function pointAt(m: CradleMotion, name: string, f: number): XY {
  const path = m.points[name];
  if (!path) throw new Error(`${m.key} has no joint ${name}`);
  const { i0, i1, w } = bracket(m.n, f);
  const a = path[i0]!, b = path[i1]!;
  return [a[0] + (b[0] - a[0]) * w, a[1] + (b[1] - a[1]) * w];
}

/** The bottle's tilt at ``f``, degrees (neck down positive: pouring). */
export function tiltAt(m: CradleMotion, f: number): number {
  const { i0, i1, w } = bracket(m.n, f);
  const a = m.tilt_deg[i0]!, b = m.tilt_deg[i1]!;
  return a + (b - a) * w;
}

/** Handle turns from rest at ``f``, in the clip's direction (0 at rest, ``ratio`` a crank turn). */
export function turnsAt(m: CradleMotion, f: number): number {
  const u = (((f % 1) + 1) % 1) * m.n;
  return ((((u - m.rest) % m.n) + m.n) % m.n) / m.n * m.ratio;
}

/** The clip fraction ``turns`` handle turns past rest. */
export function fractionAtTurns(m: CradleMotion, turns: number): number {
  const u = m.rest + (turns / m.ratio) * m.n;
  return (((u / m.n) % 1) + 1) % 1;
}

/** Rest to full tilt (``tipping``), or back (``returning``), at ``f``. */
export function phaseAt(m: CradleMotion, f: number): 'tipping' | 'returning' {
  const turns = turnsAt(m, f);
  return turns <= m.turns_to_full ? 'tipping' : 'returning';
}

/** A cradle-frame point in the linkage's plane, the mouth at ``lip``, the cradle at ``tiltDeg``. */
export function toPlane(lip: XY, tiltDeg: number, p: XY): XY {
  const a = (tiltDeg * Math.PI) / 180, c = Math.cos(a), s = Math.sin(a);
  return [lip[0] + c * p[0] - s * p[1], lip[1] + s * p[0] + c * p[1]];
}

/** A saddle's outline (a rectangle in the cradle's frame). */
export function saddleOutline(s: CradleMotion['saddles'][number]): XY[] {
  const [x0, x1] = s.x, [y0, y1] = s.y;
  return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
}

export interface Box { x0: number; y0: number; x1: number; y1: number }

/** Everything the diagram draws over the whole swing (the joints, the bottle, the saddles),
 * with ``pad`` mm round it. */
export function swingBounds(m: CradleMotion, pad = 20): Box {
  const b: Box = { x0: Infinity, y0: Infinity, x1: -Infinity, y1: -Infinity };
  const add = (p: XY): void => {
    b.x0 = Math.min(b.x0, p[0]); b.x1 = Math.max(b.x1, p[0]);
    b.y0 = Math.min(b.y0, p[1]); b.y1 = Math.max(b.y1, p[1]);
  };
  for (const path of Object.values(m.points)) path.forEach(add);
  const lip = m.points[m.mouth];
  const step = Math.max(1, Math.floor(m.n / 72));
  for (let i = 0; lip && i < m.n; i += step) {
    const t = m.tilt_deg[i]!;
    for (const p of m.bottle.outline) add(toPlane(lip[i]!, t, p));
    for (const s of m.saddles) for (const p of saddleOutline(s)) add(toPlane(lip[i]!, t, p));
  }
  return { x0: b.x0 - pad, y0: b.y0 - pad, x1: b.x1 + pad, y1: b.y1 + pad };
}

/** "neck up 25°", "level", "neck down 40°": the tilt in words. */
export function tiltWords(deg: number): string {
  const a = Math.abs(deg);
  if (a < 0.5) return 'level';
  return `${deg < 0 ? 'neck up' : 'neck down'} ${a.toFixed(1)}°`;
}
