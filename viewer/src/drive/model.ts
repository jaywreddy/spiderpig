/**
 * The walking model of SPEC.md (shared with the Python side, ``/api/walk``):
 * quasi-static support on flat ground and no-slip body motion from the feet's
 * kinematics. Deliberately no physics engine: the robot is slow and its feet
 * are driven, so "rest on the lower hull under the COM, feet don't slide" is
 * the whole story, and it is cheap enough to evaluate many times per frame.
 *
 * Mech frame (the glb ``walker`` node's local frame): x along the walking
 * axis, y up, z lateral (left side at -z). The feet are whatever the data
 * lists: every foot of the design's linkage on both sides (a leg may have
 * several), each with its path over the crank cycle.
 */
import { Line3, Triangle, Vector3 } from 'three';

export type Side = 'L' | 'R';
export interface Foot { side: Side; leg: number; z: number; xy: Float64Array }   // xy: x0, y0, x1, ...
export interface DriveData { n: number; feet: Foot[]; com: Vector3; rpmMax: number }

/** The SPEC-shaped walking data of the glb ``drive`` extras or an ``/api/walk`` response. */
export interface WalkJson {
  theta_samples: number;
  feet: { body?: string; side: Side; leg: number; z: number; xy: [number, number][] }[];
  com?: [number, number, number];
  servo?: { key?: string; rpm_max?: number };
}

export function parseDrive(src: WalkJson): DriveData {
  const n = src.theta_samples;
  const feet = src.feet.map((f) => {
    const xy = Float64Array.from(f.xy.flat());
    if (xy.length !== 2 * n || !xy.every(Number.isFinite)) throw new Error(`bad foot track ${f.body ?? f.side + f.leg}`);
    return { side: f.side, leg: f.leg, z: f.z, xy };
  });
  return { n, feet, com: new Vector3(...(src.com ?? [0, 0, 0])), rpmMax: src.servo?.rpm_max ?? 50 };
}

const TAU = 2 * Math.PI;

/** Crank angle ``theta`` on a closed grid of ``n`` samples: the sample below it and the weight of the next. */
export function gridAt(theta: number, n: number): [number, number] {
  const u = ((((theta / TAU) * n) % n) + n) % n;
  return [Math.floor(u) % n, u - Math.floor(u)];
}

/** Foot position and velocity (mech frame) at crank angle ``theta`` turning at ``omega``. */
function footAt(f: Foot, n: number, theta: number, omega: number): [Vector3, Vector3] {
  const [i, t] = gridAt(theta, n);
  const at = (k: number, c: number): number => f.xy[2 * ((k + n) % n) + c]!;
  const lerp = (a: number, b: number): number => a + (b - a) * t;
  // dp/dtheta: central difference at the samples, interpolated like p.
  const d = (k: number, c: number): number => (at(k + 1, c) - at(k - 1, c)) * (n / (2 * TAU));
  return [
    new Vector3(lerp(at(i, 0), at(i + 1, 0)), lerp(at(i, 1), at(i + 1, 1)), f.z),
    new Vector3(lerp(d(i, 0), d(i + 1, 0)) * omega, lerp(d(i, 1), d(i + 1, 1)) * omega, 0),
  ];
}

export interface WalkState {
  feet: Vector3[]; vel: Vector3[];
  normal: Vector3; offset: number;           // support plane: normal . p = offset
  height: number; pitch: number; roll: number; // mm, deg, deg
  contacts: boolean[]; nContacts: number;
  tipping: boolean; degenerate: boolean;
  margin: number;                             // mm, positive while the COM is over the polygon
  comOnPlane: Vector3;
  edges: [Vector3, Vector3][];                // support polygon edges on the plane
  vx: number; vz: number; w: number; slip: number;   // mm/s (body frame), rad/s, mm/s
}

/** Support plane, attitude, contacts and stability margin (SPEC "Support"). */
export function support(feet: Vector3[], com: Vector3): Omit<WalkState, 'vel' | 'vx' | 'vz' | 'w' | 'slip'> {
  // Every foot triple, not a ConvexHull: coplanar feet (a flat stance, the
  // double's mirrored pairs) make QuickHull degenerate, and 8 feet are 56 triples.
  const tri = new Triangle(), n = new Vector3(), cp = new Vector3(), q = new Vector3();
  let best: { dist: number; n: Vector3; d: number } | null = null;
  const m = feet.length;
  for (let i = 0; i < m; i++) for (let j = i + 1; j < m; j++) for (let k = j + 1; k < m; k++) {
    tri.set(feet[i]!, feet[j]!, feet[k]!);
    if (tri.getArea() <= 1) continue;
    tri.getNormal(n);
    if (n.y < 0) n.negate();
    const d = n.dot(feet[i]!);
    if (n.y <= 0 || feet.some((p) => n.dot(p) - d < -1e-6)) continue;   // not a lower-hull face
    cp.copy(com).addScaledVector(n, d - n.dot(com));
    let dist = tri.closestPointToPoint(cp, q).distanceTo(cp);
    if (dist < 1e-9) dist = 0;                  // contains the COM's projection
    if (!best || dist < best.dist || (dist === best.dist && n.y > best.n.y)) best = { dist, n: n.clone(), d };
  }
  const degenerate = !best;
  const normal = best?.n ?? new Vector3(0, 1, 0);
  const offset = best?.d ?? Math.min(...feet.map((p) => p.y));
  const contacts = feet.map((p) => normal.dot(p) - offset <= 0.5);
  const onPlane = (p: Vector3): Vector3 => p.clone().addScaledVector(normal, offset - normal.dot(p));
  const comOnPlane = onPlane(com);
  const P = feet.filter((_, i) => contacts[i]).map(onPlane);
  // Support polygon edges: contact pairs with every other contact on one side.
  const edges: [Vector3, Vector3][] = [];
  let inside = P.length >= 3, lineMin = Infinity, segMin = Math.min(...P.map((p) => p.distanceTo(comOnPlane)));
  const ab = new Vector3(), r = new Vector3(), seg = new Line3();
  for (let a = 0; a < P.length; a++) for (let b = a + 1; b < P.length; b++) {
    ab.subVectors(P[b]!, P[a]!);
    if (ab.lengthSq() < 1e-12) continue;
    let pos = 0, neg = 0;
    for (const p of P) {
      const s = r.subVectors(p, P[a]!).cross(ab).dot(normal);
      if (s > 1e-9) pos++; else if (s < -1e-9) neg++;
    }
    if (pos && neg) continue;
    edges.push([P[a]!, P[b]!]);
    segMin = Math.min(segMin, seg.set(P[a]!, P[b]!).closestPointToPoint(comOnPlane, true, q).distanceTo(comOnPlane));
    const s = r.subVectors(comOnPlane, P[a]!).cross(ab).dot(normal);
    if (!pos && !neg) inside = false;           // all contacts on one line
    else if ((pos && s < 0) || (neg && s > 0)) inside = false;
    lineMin = Math.min(lineMin, Math.abs(s) / ab.length());
  }
  const deg = 180 / Math.PI;
  return {
    feet, normal, offset, height: -offset,
    pitch: Math.atan2(normal.x, normal.y) * deg, roll: Math.atan2(normal.z, normal.y) * deg,
    contacts, nContacts: P.length, tipping: !!best && best.dist > 0, degenerate,
    margin: inside ? lineMin : -segMin, comOnPlane, edges,
  };
}

/** No-slip least squares about the contacts' centroid (SPEC "Motion"). */
export function motion(feet: Vector3[], vel: Vector3[], contacts: boolean[]):
  Pick<WalkState, 'vx' | 'vz' | 'w' | 'slip'> {
  const idx = contacts.flatMap((c, i) => (c ? [i] : []));
  const m = idx.length;
  if (!m) return { vx: 0, vz: 0, w: 0, slip: 0 };
  const mean = (f: (i: number) => number): number => idx.reduce((s, i) => s + f(i), 0) / m;
  const xc = mean((i) => feet[i]!.x), zc = mean((i) => feet[i]!.z);
  const ux = -mean((i) => vel[i]!.x), uz = -mean((i) => vel[i]!.z);
  let num = 0, den = 0;
  for (const i of idx) {
    const dx = feet[i]!.x - xc, dz = feet[i]!.z - zc;
    num += dx * vel[i]!.z - dz * vel[i]!.x;
    den += dx * dx + dz * dz;
  }
  const w = den < 1e-9 ? 0 : num / den;
  const vx = ux - w * zc, vz = uz + w * xc;
  const ss = idx.reduce((s, i) =>
    s + (vx + w * feet[i]!.z + vel[i]!.x) ** 2 + (vz - w * feet[i]!.x + vel[i]!.z) ** 2, 0);
  return { vx, vz, w, slip: Math.sqrt(ss / (2 * m)) };
}

/** The whole robot with each side's crank at ``theta[S]`` turning at ``omega[S]``. */
export function evaluate(data: DriveData, theta: Record<Side, number>, omega: Record<Side, number>): WalkState {
  const pv = data.feet.map((f) => footAt(f, data.n, theta[f.side], omega[f.side]));
  const feet = pv.map((x) => x[0]), vel = pv.map((x) => x[1]);
  const s = support(feet, data.com);
  return { ...s, vel, ...motion(feet, vel, s.contacts) };
}

/** Per-revolution metrics of a straight walk (SPEC "Metrics"), when the data source has none. */
export function straightWalk(data: DriveData): Record<string, number | number[] | string> {
  const n = data.n, dth = TAU / n;
  let x = 0, yaw = 0, slip2 = 0, tip = 0, degen = 0, minMargin = Infinity;
  const h: number[] = [], pitch: number[] = [], roll: number[] = [], duty = data.feet.map(() => 0);
  for (let i = 0; i < n; i++) {
    const s = evaluate(data, { L: i * dth, R: i * dth }, { L: 1, R: 1 });
    x += (Math.cos(yaw) * s.vx + Math.sin(yaw) * s.vz) * dth;
    yaw += s.w * dth;
    h.push(s.height); pitch.push(s.pitch); roll.push(s.roll);
    slip2 += s.slip ** 2; minMargin = Math.min(minMargin, s.margin);
    tip += +s.tipping; degen += +s.degenerate;
    s.contacts.forEach((c, k) => { duty[k]! += +c / n; });
  }
  const range = (a: number[]): number[] => [Math.min(...a), Math.max(...a)];
  const slip = Math.sqrt(slip2 / n);
  return {
    stride_mm: Math.abs(x), stride_signed_mm: x, direction: x >= 0 ? '+x' : '-x',
    speed_mm_s: (Math.abs(x) * data.rpmMax) / 60, bob_mm: Math.max(...h) - Math.min(...h),
    pitch_deg: range(pitch), roll_deg: range(roll),
    slip_rms_mm_per_rad: slip, slip_rms_mm_per_rev: slip * TAU, min_margin_mm: minMargin,
    tipping_fraction: tip / n, degenerate_fraction: degen / n, duty,
  };
}
