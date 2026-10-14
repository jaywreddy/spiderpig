import { describe, expect, it } from 'vitest';
import { packFraction, stagger, type PackData } from './pack';

function data(pose?: PackData['pose']): PackData {
  return {
    modules: [], steps: [], pose,
    box: { inside_mm: [1, 1, 1], outside_mm: [11, 11, 11], floor: [0, 0, 0], volume_l: 0 },
  };
}

describe('the pack', () => {
  it('starts from the pose the cradle is cranked to (B: 4 turns from rest)', () => {
    const b = data({ crank_t: 0.5454, clip_fraction: 0.0868, tilt_deg: -34.8, turns_from_rest: 4 });
    expect(packFraction(b)).toBeCloseTo(0.0868, 6);
  });

  it("starts from the clip's first frame when the data names no pose (a flat pack of old)", () => {
    expect(packFraction(data())).toBe(0);
  });

  it('flies the modules one after another, each from 0 to 1', () => {
    const n = 4;
    for (let i = 0; i < n; i++) {
      expect(stagger(0, i, n)).toBe(0);
      expect(stagger(1, i, n)).toBe(1);
    }
    expect(stagger(0.3, 0, n)).toBeGreaterThan(stagger(0.3, 3, n));
  });
});
