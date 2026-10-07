/**
 * The drive's walking model (``model.ts``) against the Python one (``spiderpig/walk.py``),
 * on the reference Klann quad both are pinned to: ``tests/fixtures/linkage/walk_reference.json``
 * holds its feet and centre of mass (as ``/api/walk`` sends them), the Python model's
 * straight-walk metrics and the reference numbers ``tests/test_walk.py`` checks too.
 */
import { describe, expect, it } from 'vitest';
import { Vector3 } from 'three';
import raw from '../../../tests/fixtures/linkage/walk_reference.json?raw';
import striderRaw from '../../../tests/fixtures/linkage/walk_reference_strider.json?raw';
import { evaluate, gridAt, motion, parseDrive, straightWalk, support, type WalkJson } from './model';

type Tol = [number, number];   // [value, absolute tolerance]
interface Reference {
  contacts_135: boolean[]; pitch_deg_max_abs: Tol; bob_mm: Tol; stride_mm: Tol;
  min_margin_mm: Tol; direction: string; tipping_fraction: number;
  degenerate_fraction: number; roll_deg: [number, number]; rpm_max: number;
}
interface Doc {
  data: { walk: WalkJson; metrics: Record<string, number | number[] | string | boolean>; reference: Reference };
}

const { walk, metrics: py, reference: ref } = (JSON.parse(raw) as Doc).data;
const data = parseDrive(walk);
const metrics = straightWalk(data);
const num = (k: string): number => metrics[k] as number;
const pair = (k: string): number[] => metrics[k] as number[];
const near = (got: number, [want, tol]: Tol): void => { expect(Math.abs(got - want)).toBeLessThanOrEqual(tol); };

describe('the reference Klann quad', () => {
  it('reads the walk data', () => {
    expect(data.n).toBe(360);
    expect(data.feet).toHaveLength(8);
    expect(data.rpmMax).toBe(ref.rpm_max);
    expect(data.feet.map((f) => f.side)).toEqual(['L', 'L', 'L', 'L', 'R', 'R', 'R', 'R']);
  });

  it('walks the reference numbers', () => {
    near(Math.max(...pair('pitch_deg').map(Math.abs)), ref.pitch_deg_max_abs);
    near(num('bob_mm'), ref.bob_mm);
    near(num('stride_mm'), ref.stride_mm);
    near(num('min_margin_mm'), ref.min_margin_mm);
    expect(metrics.direction).toBe(ref.direction);
    expect(num('tipping_fraction')).toBe(ref.tipping_fraction);
    expect(num('degenerate_fraction')).toBe(ref.degenerate_fraction);
    pair('roll_deg').forEach((r, i) => { expect(Math.abs(r - ref.roll_deg[i]!)).toBeLessThan(1e-9); });
    expect(num('speed_mm_s')).toBeCloseTo((num('stride_mm') * ref.rpm_max) / 60, 9);
  });

  it('stands on legs 0 and 3 of both sides at 135 degrees', () => {
    const th = (135 * Math.PI) / 180;
    const s = evaluate(data, { L: th, R: th }, { L: 1, R: 1 });
    expect(s.contacts).toEqual(ref.contacts_135);
    expect(Math.abs(s.pitch)).toBeLessThan(0.01);
    expect(s.tipping).toBe(false);
  });

  it('gives the Python model\'s metrics', () => {
    for (const k of ['stride_mm', 'stride_signed_mm', 'bob_mm', 'min_margin_mm', 'speed_mm_s',
      'slip_rms_mm_per_rad', 'slip_rms_mm_per_rev', 'tipping_fraction', 'degenerate_fraction']) {
      expect(num(k), k).toBeCloseTo(py[k] as number, 6);
    }
    for (const k of ['pitch_deg', 'roll_deg', 'duty']) {
      const want = py[k] as number[];
      pair(k).forEach((v, i) => { expect(v, `${k}[${i}]`).toBeCloseTo(want[i]!, 6); });
    }
    expect(metrics.direction).toBe(py.direction);
  });
});

// The default design, the Strider double (``walk_reference_strider.json``: its feet and centre of
// mass at its planned foot z, and the Python model's metrics of them).
describe('the reference Strider double', () => {
  const doc = (JSON.parse(striderRaw) as { data: { walk: WalkJson; metrics: Record<string, number | number[] | string | boolean> } }).data;
  const sd = parseDrive(doc.walk);
  const sm = straightWalk(sd);
  const spy = doc.metrics;

  it('reads the walk data', () => {
    expect(sd.n).toBe(doc.walk.theta_samples);
    expect(sd.feet).toHaveLength(doc.walk.feet.length);
    expect(sd.feet.map((f) => f.side)).toEqual(doc.walk.feet.map((f) => f.side));
    expect(new Set(sd.feet.map((f) => f.side))).toEqual(new Set(['L', 'R']));
  });

  it('gives the Python model\'s metrics', () => {
    for (const k of ['stride_mm', 'stride_signed_mm', 'bob_mm', 'min_margin_mm', 'speed_mm_s',
      'slip_rms_mm_per_rad', 'slip_rms_mm_per_rev', 'tipping_fraction', 'degenerate_fraction']) {
      expect(sm[k] as number, k).toBeCloseTo(spy[k] as number, 6);
    }
    for (const k of ['pitch_deg', 'roll_deg', 'duty']) {
      const want = spy[k] as number[];
      (sm[k] as number[]).forEach((v, i) => { expect(v, `${k}[${i}]`).toBeCloseTo(want[i]!, 6); });
    }
    expect(sm.direction).toBe(spy.direction);
  });
});

describe('the model on synthetic feet', () => {
  const square = (y = -100, hx = 50, hz = 40): Vector3[] =>
    [[-hx, y, -hz], [-hx, y, hz], [hx, y, -hz], [hx, y, hz]].map(([x, yy, z]) => new Vector3(x, yy, z));

  it('stands level on four coplanar feet', () => {
    const s = support(square(), new Vector3(0, 0, 0));
    expect(s.degenerate).toBe(false);
    expect(s.tipping).toBe(false);
    expect(s.height).toBeCloseTo(100, 9);
    expect(s.pitch).toBeCloseTo(0, 9);
    expect(s.roll).toBeCloseTo(0, 9);
    expect(s.contacts).toEqual([true, true, true, true]);
    expect(s.margin).toBeCloseTo(40, 9);           // nearest edges: z = +-40
  });

  it('tips with the centre of mass outside', () => {
    const s = support(square(), new Vector3(200, 0, 0));
    expect(s.tipping).toBe(true);
    expect(s.margin).toBeCloseTo(-150, 9);
  });

  it('is degenerate on two feet', () => {
    const s = support([new Vector3(0, -100, 0), new Vector3(30, -95, 10)], new Vector3(5, 0, 0));
    expect(s.degenerate).toBe(true);
    expect(s.height).toBeCloseTo(100, 9);
    expect(s.margin).toBeLessThanOrEqual(0);
  });

  it('recovers a rigid motion with no slip', () => {
    const feet = square();
    const [V, w] = [[3, -2], 0.5] as const;
    const vel = feet.map((p) => new Vector3(-(V[0] + w * p.z), 0, -(V[1] - w * p.x)));
    const m = motion(feet, vel, [true, true, true, true]);
    expect(m.vx).toBeCloseTo(V[0], 9);
    expect(m.vz).toBeCloseTo(V[1], 9);
    expect(m.w).toBeCloseTo(w, 9);
    expect(m.slip).toBeCloseTo(0, 9);
  });

  it('finds the sample below a crank angle', () => {
    expect(gridAt(0, 360)).toEqual([0, 0]);
    const [i, t] = gridAt(-Math.PI / 360, 360);     // half a sample before 0: wraps
    expect(i).toBe(359);
    expect(t).toBeCloseTo(0.5, 9);
  });
});
