import { describe, expect, it } from 'vitest';
import {
  bottleInside, discBelow, fitsRoom, heightFor, heldAt, keptFromRest, levelFor, levelRange,
  mouthLevel, nominalVolume, profileVolume, radiusAt, startingFill, surfaceRuns, volumeBelow,
  volumeTo, ML, type Profile,
} from './liquid';
import { DEFAULT_VESSEL, VESSELS, brimMl, cavityOf, floorOf, heightOf } from './vessels';

const deg = (d: number): number => (d * Math.PI) / 180;
const cyl = (r: number, len: number): Profile => [[0, r], [len, r]];

describe('profiles: radius and volume', () => {
  it('interpolates the radius, 0 outside', () => {
    const p: Profile = [[0, 10], [10, 20], [20, 20]];
    expect(radiusAt(p, 5)).toBe(15);
    expect(radiusAt(p, 15)).toBe(20);
    expect(radiusAt(p, -1)).toBe(0);
    expect(radiusAt(p, 21)).toBe(0);
  });

  it('a cylinder holds pi r^2 h, a cone a third of it', () => {
    expect(profileVolume(cyl(10, 50))).toBeCloseTo(Math.PI * 100 * 50, 6);
    expect(profileVolume([[0, 0], [30, 10]])).toBeCloseTo((Math.PI * 100 * 30) / 3, 6);
  });

  it('fills an upright vessel to a height and back', () => {
    const p: Profile = [[0, 0], [30, 10], [80, 25]];
    expect(volumeTo(p, 30)).toBeCloseTo((Math.PI * 100 * 30) / 3, 6);
    expect(volumeTo(p, 80)).toBeCloseTo(profileVolume(p), 6);
    for (const h of [5, 30, 55, 79]) expect(heightFor(p, volumeTo(p, h))).toBeCloseTo(h, 4);
    expect(heightFor(p, 1e12)).toBe(80);       // overflowing: the rim
    expect(heightFor(p, 0)).toBe(0);
  });
});

describe('a plane through a solid of revolution', () => {
  it('cuts a disc into the circular segment', () => {
    expect(discBelow(10, -10)).toBe(0);
    expect(discBelow(10, 10)).toBeCloseTo(Math.PI * 100);
    expect(discBelow(10, 0)).toBeCloseTo(Math.PI * 50);
    // a numeric check off the middle
    let a = 0;
    const n = 20000;
    for (let i = 0; i < n; i++) {
      const u = -10 + ((i + 0.5) * 20) / n;
      if (u < 4) a += 2 * Math.sqrt(100 - u * u) * (20 / n);
    }
    expect(discBelow(10, 4)).toBeCloseTo(a, 2);
  });

  it('halves a lying cylinder through its axis, and fills a standing one to the level', () => {
    const p = cyl(20, 100), v = profileVolume(p);
    expect(volumeBelow(p, 0, 0) / v).toBeCloseTo(0.5, 4);
    expect(volumeBelow(p, 0, 20) / v).toBeCloseTo(1, 4);
    expect(volumeBelow(p, 0, -20)).toBe(0);
    // axis up (a = 90°: the height is s): a level at 30 holds 30 mm of it
    expect(volumeBelow(p, deg(90), 30)).toBeCloseTo(Math.PI * 400 * 30, 0);
    // a tilted cylinder cut through its centre: half, by symmetry
    const a = deg(35), centre = Math.sin(a) * 50;
    expect(volumeBelow(p, a, centre) / v).toBeCloseTo(0.5, 3);
  });

  it('solves the level that holds a volume, at any tilt', () => {
    const p: Profile = [[0, 9], [80, 9], [130, 45], [340, 45]];
    for (const t of [-60, -20, 0, 15, 50]) {
      const [lo, hi] = levelRange(p, deg(t));
      expect(lo).toBeLessThan(hi);
      for (const frac of [0.1, 0.5, 0.9]) {
        const v = profileVolume(p) * frac;
        const c = levelFor(p, deg(t), v);
        expect(volumeBelow(p, deg(t), c) / v).toBeCloseTo(1, 3);
      }
    }
  });

  it('draws the free surface as chords inside the solid, covering its area', () => {
    const p = cyl(20, 100);
    const runs = surfaceRuns(p, 0, 0, 1);         // a lying cylinder, half full: a 40 x 100 rectangle
    expect(runs.length).toBe(1);
    let area = 0;
    const run = runs[0]!;
    for (let i = 1; i < run.length; i++) {
      const [s0, , w0] = run[i - 1]!, [s1, , w1] = run[i]!;
      area += Math.abs(s1 - s0) * (w0 + w1);
    }
    expect(area / 4000).toBeCloseTo(1, 2);
    for (const [s, u, w] of run) {
      expect(s).toBeGreaterThanOrEqual(-1e-6);
      expect(s).toBeLessThanOrEqual(100 + 1e-6);
      expect(u * u + w * w).toBeLessThanOrEqual(400 + 1e-6);
    }
    // a tilted surface: every chord on the plane
    const a = deg(30), c = 12;
    for (const r of surfaceRuns(p, a, c)) for (const [s, u] of r) expect(Math.sin(a) * s + Math.cos(a) * u).toBeCloseTo(c, 6);
    expect(surfaceRuns(p, 0, 50)).toEqual([]);   // over the top: no surface
  });
});

describe('a bottle tipping about its mouth', () => {
  // a magnum-ish inside: an 18.5 mm bore, a shoulder, a 91 mm body
  const bottle: Profile = [[0, 9.25], [85, 9.25], [150, 45.5], [346, 45.5]];

  it('keeps everything with the neck up, nothing with the neck well down', () => {
    expect(heldAt(bottle, deg(-80)) / profileVolume(bottle)).toBeCloseTo(1, 2);
    expect(heldAt(bottle, deg(60))).toBeLessThan(1 * ML);
  });

  it('pours what lies above the mouth\'s lowest point', () => {
    const a = deg(0);
    expect(mouthLevel(bottle, a)).toBeCloseTo(-9.25);
    // lying flat it keeps only what is under the bore's bottom
    expect(heldAt(bottle, a)).toBeCloseTo(volumeBelow(bottle, a, -9.25), 6);
    expect(heldAt(bottle, a)).toBeLessThan(profileVolume(bottle) / 2);
  });

  it('keeps the least it held since rest, sample by sample', () => {
    const fill = 1500 * ML;
    const tilts = [-25, -10, 0, 10, 30, 55, 30, 0, -25];
    const kept = keptFromRest(bottle, tilts, fill);
    expect(kept[0]).toBeCloseTo(Math.min(fill, heldAt(bottle, deg(-25))));
    for (let k = 1; k < kept.length; k++) expect(kept[k]!).toBeLessThanOrEqual(kept[k - 1]!);
    // back at rest (the swing's end) it is still empty: the page restarts it at rest
    expect(kept[kept.length - 1]!).toBe(kept[5]!);
    expect(kept[2]!).toBeLessThan(fill);
  });

  it('starts a magnum at 1.5 L with about 6 cm of ullage', () => {
    const outline: [number, number][] = [[0, 15.5], [85, 15.5], [150, 49.5], [356, 49.5], [356, -49.5], [150, -49.5], [85, -15.5], [0, -15.5]];
    const inside = bottleInside(outline);
    expect(inside[0]![1]).toBeGreaterThan(8);      // the bore, ~18.5 mm across
    expect(inside[0]![1]).toBeLessThan(10.5);
    expect(inside[inside.length - 1]![0]).toBeLessThan(356);
    const nominal = nominalVolume('1.5 L Bordeaux magnum');
    expect(nominal).toBe(1500 * ML);
    const fill = startingFill(inside, nominal, 60);
    expect(fill / ML).toBeGreaterThan(1350);
    expect(fill).toBeLessThanOrEqual(nominal);
    const ullage = -levelFor(inside, deg(-90), fill);   // standing up: the level under the mouth
    expect(ullage).toBeGreaterThanOrEqual(59);
    expect(nominalVolume('750 ml Bordeaux')).toBe(750 * ML);
    expect(nominalVolume('jeroboam', 3000)).toBe(3000 * ML);
  });
});

describe('vessels under the mouth', () => {
  it('fits a vessel to the room under the mouth', () => {
    const room: [number, number][] = [[0, 120], [200, 120], [260, 60]];
    const narrow: Profile = [[0, 40], [150, 50], [240, 35]];
    expect(fitsRoom(narrow, 0, room).fits).toBe(true);
    expect(fitsRoom(narrow, 30, room).fits).toBe(false);     // on a riser it pokes out of the top
    const wide: Profile = [[0, 130], [100, 60]];
    const r = fitsRoom(wide, 0, room);
    expect(r.fits).toBe(false);
    expect(r.margin).toBeCloseTo(-10);
    expect(fitsRoom(narrow, 0, []).fits).toBe(true);           // no room data: no verdict against it
  });

  it('every listed vessel has a sane profile and a cavity near its stated brim', () => {
    expect(VESSELS.some((v) => v.key === DEFAULT_VESSEL)).toBe(true);
    for (const v of VESSELS) {
      for (let i = 1; i < v.profile.length; i++) expect(v.profile[i]![0]).toBeGreaterThan(v.profile[i - 1]![0]);
      expect(heightOf(v)).toBeGreaterThan(100);
      const floor = floorOf(v);
      expect(floor).toBeGreaterThan(0);
      expect(floor).toBeLessThan(heightOf(v) * 0.7);
      expect(cavityOf(v)[0]![0]).toBe(floor);
      const brim = brimMl(v);
      if (v.brim_ml || v.kind === 'glass') {
        // traced profiles: within a third of the maker's brim figure
        const stated = v.brim_ml ?? v.capacity_ml;
        expect(brim / stated).toBeGreaterThan(0.66);
        expect(brim / stated).toBeLessThan(1.5);
      } else {
        // a decanter rated for a bottle holds it with room to swirl
        expect(brim).toBeGreaterThan(v.capacity_ml);
        expect(brim).toBeLessThan(v.capacity_ml * 3);
      }
    }
  });

  it('a glass\'s wine starts in its bowl, not its stem', () => {
    const iso = VESSELS.find((v) => v.key === 'iso_3591')!;
    expect(floorOf(iso)).toBeGreaterThan(50);
    expect(floorOf(iso)).toBeLessThan(62);
  });
});
