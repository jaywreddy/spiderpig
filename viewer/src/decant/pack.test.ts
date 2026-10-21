import { describe, expect, it } from 'vitest';
import { matmul, movedModule, packFraction, packOrder, snappedDelta, stagger, type PackData } from './pack';

const I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];

function apply(m: number[][], p: number[]): number[] {
  return [0, 1, 2].map((i) => m[i]![0]! * p[0]! + m[i]![1]! * p[1]! + m[i]![2]! * p[2]! + m[i]![3]!);
}

describe('editing the pack', () => {
  it('moves a piece in 3 mm steps, its parts with it', () => {
    const d = snappedDelta([10, 20, 30], [4.4, -1.4, 16.6]);
    expect(apply(d, [10, 20, 30])).toEqual([13, 20, 48]);         // 4.4 -> 3, -1.4 -> 0, 16.6 -> 18
    const m = movedModule({ key: 'k', label: '', how: '', parts: ['a', 'b'], matrix: I,
                            part_matrix: { b: [[1, 0, 0, 5], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]] } }, d);
    expect(apply(m.matrix, [0, 0, 0])).toEqual([3, 0, 18]);
    expect(apply(m.part_matrix!['b']!, [0, 0, 0])).toEqual([8, 0, 18]);
  });

  it('turns it a quarter turn about the vertical through its centre, or tips it', () => {
    const yaw = snappedDelta([10, 0, 0], [0, 0, 0], 1);
    expect(apply(yaw, [10, 0, 0])).toEqual([10, 0, 0]);            // the centre stays
    expect(apply(yaw, [11, 0, 0])).toEqual([10, 0, -1]);           // +x to -z (about +y)
    const tip = snappedDelta([0, 0, 0], [0, 0, 0], 0, 1);
    expect(apply(tip, [0, 1, 0])).toEqual([0, 0, 1]);              // up to +z (about +x)
    expect(matmul(snappedDelta([0, 0, 0], [0, 0, 0], 4), I)).toEqual(I);   // four: none
  });
});

function data(pose?: PackData['pose']): PackData {
  return {
    modules: [], steps: [], pose,
    box: { inside_mm: [1, 1, 1], outside_mm: [11, 11, 11], floor: [0, 0, 0], volume_l: 0 },
  };
}

describe('the pack', () => {
  it('starts from the pose the cradle is cranked to (B: its arm along the rear leg)', () => {
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

describe('what goes in the box', () => {
  it('lists the printed spacers among the pieces in the order they go in, as the guide does', () => {
    const m = (key: string) => ({ key, label: key, how: `the ${key}`, parts: [], matrix: [] });
    const d: PackData = {
      ...data(), modules: [m('board'), m('box'), m('drive')],
      spacers: [{ key: 'spacer_02a', label: 'Spacer 2a', rank: 1, words: 'under the drive' },
                { key: 'spacer_01a', label: 'Spacer 1a', rank: 0 },
                { key: 'spacer_02b', label: 'Spacer 2b', rank: 1 }],
    };
    const order = packOrder(d);
    expect(order.map((o) => o.label)).toEqual(
      ['board', 'Spacer 1a', 'box', 'Spacer 2a', 'Spacer 2b', 'drive']);
    expect(order.filter((o) => o.spacer)).toHaveLength(3);
    expect(order[3]!.how).toBe('under the drive');
    expect(packOrder(data())).toEqual([]);                    // (no spacers: the pieces alone)
  });
});
