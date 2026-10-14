import { describe, expect, it } from 'vitest';
import {
  groupSelect, linkageGroups, linkageOptions, pickMode, withMode, type ModeCatalogue,
} from './modes';

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

describe('one linkage picker for every kind', () => {
  const linkages = [
    { key: 'strider', name: 'Strider', kind: 'walker' },
    { key: 'crank_rocker', name: 'Crank-rocker', kind: 'mechanism' },
    { key: 'klann', name: 'Klann', kind: 'walker' },
    { key: 'palletizer_reach', name: 'Palletizer', kind: 'arm' },
    { key: 'palletizer', name: 'Palletizer', kind: 'arm' },
  ];

  it('lists every linkage, grouped by kind in the order the server lists the kinds', () => {
    const groups = linkageGroups(linkages);
    expect(groups.map((g) => g.label)).toEqual(['walkers', 'mechanisms', 'arms']);
    expect(groups[0]!.options).toEqual([['Strider', 'strider'], ['Klann', 'klann']]);
    // a name two linkages share is told apart by its key
    expect(groups[2]!.options.map((o) => o[0]))
      .toEqual(['Palletizer (palletizer_reach)', 'Palletizer (palletizer)']);
    const opts = linkageOptions(groups);
    expect(Object.values(opts))
      .toEqual(['strider', 'klann', 'crank_rocker', 'palletizer_reach', 'palletizer']);
  });

  it("wraps the dropdown's options in a group per kind, keeping their order", () => {
    const groups = linkageGroups(linkages);
    type Node = { label?: string; value?: string; kids: Node[]; append(...n: Node[]): void };
    const node = (extra: Partial<Node> = {}): Node => {
      const n: Node = { kids: [], append: (...k) => { n.kids.push(...k); }, ...extra };
      return n;
    };
    const options = Object.values(linkageOptions(groups)).map((value) => node({ value }));
    const select = { options, ownerDocument: { createElement: () => node() }, ...node() };
    groupSelect(select as unknown as HTMLSelectElement, groups);
    expect(select.kids.map((g) => g.label)).toEqual(['walkers', 'mechanisms', 'arms']);
    expect(select.kids.flatMap((g) => g.kids.map((o) => o.value)))
      .toEqual(options.map((o) => o.value));
  });
});
