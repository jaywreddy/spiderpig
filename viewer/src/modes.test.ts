import { describe, expect, it } from 'vitest';
import { machines, pickMode, withMode, type ModeCatalogue } from './modes';

const walker: ModeCatalogue = {
  linkage: 'strider', kind: 'walker', default: 'robot',
  modes: ['robot', 'klann', 'double', 'decker', 'double_double'],
  labels: { robot: 'robot', klann: 'single (one leg)' },
};
const arm: ModeCatalogue = {
  linkage: 'palletizer_reach', kind: 'arm', default: 'robot', modes: ['robot', 'side'],
  labels: { robot: 'robot (on its turntable)', side: 'the side alone' },
};
const mechanism: ModeCatalogue = {
  linkage: 'crank_rocker', kind: 'mechanism', default: 'side', modes: ['side'],
  labels: { side: 'one side' },
};

describe('the mode picker follows the linkage', () => {
  it('offers what the linkage declares', () => {
    expect(withMode(arm).modes).toEqual(['robot', 'side']);
    expect(withMode(mechanism).modes).toEqual(['side']);
    expect(withMode(walker).modes).toContain('double_double');
  });

  it("adds a stored design's own mode when the catalogue lacks it", () => {
    const { modes, labels } = withMode(walker, 'side');
    expect(modes.at(-1)).toBe('side');
    expect(labels.side).toBe('side (this design)');
    expect(withMode(arm, 'side').modes).toEqual(['robot', 'side']);   // already offered
  });

  it("falls back to the linkage's default view", () => {
    expect(pickMode(mechanism, 'robot')).toBe('side');     // a mechanism makes no robot
    expect(pickMode(arm, 'side')).toBe('side');
    expect(pickMode(walker, 'klann')).toBe('klann');
    expect(pickMode(walker, null)).toBe('robot');
    expect(pickMode(walker, 'side', 'side')).toBe('side');
  });
});

describe('the machine picker', () => {
  it('lists every linkage without feet, arms and mechanisms alike', () => {
    const opts = machines([
      { key: 'strider', name: 'Strider', kind: 'walker', feet: 1 },
      { key: 'palletizer_reach', name: 'Palletizer', kind: 'arm', feet: 0 },
      { key: 'crank_rocker', name: 'Crank-rocker', kind: 'mechanism', feet: 0 },
    ]);
    expect(Object.values(opts)).toEqual(['palletizer_reach', 'crank_rocker']);
    expect(Object.keys(opts)).toEqual(['arm: Palletizer', 'mechanism: Crank-rocker']);
  });
});
