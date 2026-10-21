import { describe, expect, it } from 'vitest';
import {
  adjustShift, bracket, fractionAtTurns, inToggleZone, phaseAt, pointAt, saddleAt, settingOffsets,
  swingBounds, tiltAt, tiltWords, toggleZones, toPlane, turnsAt, type CradleMotion,
} from './motion';

/** A toy cradle: 4 samples, its tilt -20, 0, 40, 0; rest at 0, full tilt at 2. */
function toy(): CradleMotion {
  return {
    key: 'toy', name: 'Toy', notes: '', source: '', verdict: 'study', why: '', labels: {},
    frame: ['O'], crank: ['O', 'M'], mouth: 'G', axis: 'E', output_link: 'b2',
    links: { b2: { joints: ['G', 'E'], edges: [['G', 'E']] } },
    points: {
      O: [[0, 0], [0, 0], [0, 0], [0, 0]],
      M: [[10, 0], [0, 10], [-10, 0], [0, -10]],
      G: [[100, 50], [100, 50], [100, 50], [100, 50]],
      E: [[200, 50], [200, 60], [200, 70], [200, 60]],
    },
    tilt_deg: [-20, 0, 40, 0], handle_turns: [0, 7.5, 15, 22.5],
    rest: 0, full: 2, n: 4, rest_deg: -20, full_deg: 40, turns_to_full: 15, ratio: 30,
    mouth_wander_mm: 0,
    bottle: { name: 'b', outline: [[0, -10], [50, -10], [50, 10], [0, 10]] },
    saddles: [{ name: 's', x: [10, 20], y: [-30, -5] }],
    ties: {},
  };
}

describe('the cradle page\'s swing', () => {
  it('brackets a clip fraction between two samples, wrapping at the end', () => {
    expect(bracket(4, 0)).toEqual({ i0: 0, i1: 1, w: 0 });
    expect(bracket(4, 0.375)).toEqual({ i0: 1, i1: 2, w: 0.5 });
    expect(bracket(4, 0.875)).toEqual({ i0: 3, i1: 0, w: 0.5 });
    expect(bracket(4, 1.25)).toEqual({ i0: 1, i1: 2, w: 0 });
  });

  it('interpolates points and the tilt between the samples', () => {
    const m = toy();
    expect(pointAt(m, 'M', 0.125)).toEqual([5, 5]);
    expect(tiltAt(m, 0.375)).toBe(20);
    expect(() => pointAt(m, 'Z', 0)).toThrow(/no joint Z/);
  });

  it('counts handle turns from rest and maps them back to the clip', () => {
    const m = { ...toy(), rest: 1 };
    expect(turnsAt(m, 0.25)).toBe(0);
    expect(turnsAt(m, 0.5)).toBe(7.5);
    expect(turnsAt(m, 0)).toBe(22.5);          // three quarters of a turn past rest
    for (const t of [0, 3, 15, 29]) expect(turnsAt(m, fractionAtTurns(m, t))).toBeCloseTo(t);
  });

  it('tips from rest to full tilt, then returns', () => {
    const m = toy();
    expect(phaseAt(m, 0.25)).toBe('tipping');
    expect(phaseAt(m, 0.75)).toBe('returning');
  });

  it('places a cradle-frame point about the mouth at the tilt', () => {
    const [x, y] = toPlane([100, 50], 90, [10, 0]);
    expect(x).toBeCloseTo(100);
    expect(y).toBeCloseTo(60);
  });

  it('bounds the joints, the bottle and the saddles over the swing', () => {
    const b = swingBounds(toy(), 0);
    expect(b.x0).toBeLessThanOrEqual(-10);
    expect(b.x1).toBeGreaterThanOrEqual(200);
    expect(b.y0).toBeLessThan(40);             // the saddle hangs under the mouth
  });

  it('moves the V blocks and the base stop from the drawn bottle\'s setting to another', () => {
    // the 6 L drawn (its V blocks at their lowest), a 750 ml picked
    const o = settingOffsets([0, 0, 571.5], [12.73, 68.87, 300.5]);
    expect(o.front).toBeCloseTo(12.73);
    expect(o.rear).toBeCloseTo(68.87);
    expect(o.stop).toBeCloseTo(-271);
    expect(adjustShift('front', o)).toEqual([0, o.front]);
    expect(adjustShift('rear', o)).toEqual([0, o.rear]);
    expect(adjustShift('stop', o)).toEqual([o.stop, o.rear]);   // up with the rear V, along
    expect(settingOffsets(undefined, [1, 2, 3])).toEqual({ front: 0, rear: 0, stop: 0 });
    const rear = saddleAt({ name: 'r', x: [255, 380], y: [-131, -55], moves: 'rear' }, o);
    expect(rear.y[0]).toBeCloseTo(-131 + 68.87);
    expect(rear.x).toEqual([255, 380]);
    expect(saddleAt({ name: 's', x: [0, 1], y: [0, 1] }, o).y).toEqual([0, 1]);
  });

  it('says the tilt in words', () => {
    expect(tiltWords(-25)).toBe('neck up 25.0°');
    expect(tiltWords(0.2)).toBe('level');
    expect(tiltWords(55)).toBe('neck down 55.0°');
  });

  it('marks the toggle zones either side of rest and of full tilt', () => {
    const m = toy();
    expect(toggleZones(m)).toEqual([]);                  // (none sent: none drawn)
    expect(inToggleZone(m, 1)).toBe(false);
    m.toggle_turns = 2.5;
    expect(toggleZones(m)).toEqual([[0, 2.5], [12.5, 17.5], [27.5, 30]]);
    for (const t of [0, 2.4, 13, 15, 17.5, 28, 30, 31]) expect(inToggleZone(m, t)).toBe(true);
    for (const t of [2.6, 7.5, 12.4, 17.6, 27.4]) expect(inToggleZone(m, t)).toBe(false);
  });
});
