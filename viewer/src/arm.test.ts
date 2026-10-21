import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import {
  crankCommand, jointsOf, liveCapable, liveQuery, liveReadout, liveTargets, posedMatrix, poseQuery,
} from './arm';

describe('arm joint sliders', () => {
  const info = { key: 'palletizer', kind: 'arm', ranged: true, walks: false, inputs: ['q2', 'q3'],
    ranges: { q2: [40, 115] as [number, number], q3: [-45, 20] as [number, number] } };
  const tree = { joints: [{ name: 'q1', range_deg: [-150, 150] as [number, number], label: 'yaw' }] };

  it('lists the inputs, then the spatial joints, each mid-range', () => {
    const js = jointsOf(info, tree);
    expect(js.map((j) => j.name)).toEqual(['q2', 'q3', 'q1']);
    expect(js[0]!.value).toBeCloseTo(77.5);
    expect(js[2]!.label).toBe('yaw');
    const seeded = jointsOf(info, tree, { q2: 40, q3: -45 });   // the bake's first frame
    expect([seeded[0]!.value, seeded[1]!.value]).toEqual([40, -45]);
  });

  it('offers joints by the ranged capability, never the kind name', () => {
    expect(jointsOf({ key: 'strider', kind: 'walker', ranged: false, walks: true }, undefined)).toEqual([]);
    expect(jointsOf({ key: 'hoecken', kind: 'mechanism', ranged: false, walks: false, inputs: ['t'] }, tree))
      .toEqual([]);
    expect(jointsOf({ ...info, kind: 'anything' }, undefined).map((j) => j.name)).toEqual(['q2', 'q3']);
  });

  it('poses a node: the server pose times its first-frame matrix', () => {
    const frame0 = new THREE.Matrix4().makeTranslation(10, 0, 0);
    const pose = new THREE.Matrix4().makeRotationY(Math.PI / 2);
    const m = posedMatrix(pose.toArray(), frame0);
    const p = new THREE.Vector3().setFromMatrixPosition(m);
    expect(p.x).toBeCloseTo(0);
    expect(p.z).toBeCloseTo(-10);
  });

  it('asks for the pose with the design query and each joint in degrees', () => {
    const q = new URLSearchParams(poseQuery('linkage=palletizer', jointsOf(info, tree)));
    expect(q.get('linkage')).toBe('palletizer');
    expect(Number(q.get('j.q1'))).toBe(0);
    expect(Number(q.get('j.q3'))).toBeCloseTo(-12.5);
  });
});

describe('the live sim of a design that does not walk', () => {
  const hello = {
    drives: [] as string[], sites: ['tool0'],
    joints: { q1: { lo: -120, hi: 120, value: 0, effort: 1.5 }, q2: { lo: -10, hi: 110, value: -10, effort: 2 } },
  };

  it('goes live for what does not walk (a walker drives from the drive panel)', () => {
    expect(liveCapable({ key: 'p', kind: 'arm', walks: false })).toBe(true);
    expect(liveCapable({ key: 'h', kind: 'mechanism', walks: false })).toBe(true);
    expect(liveCapable({ key: 's', kind: 'walker', walks: true })).toBe(false);
    expect(liveCapable({ key: 'old', kind: 'x', feet: 0 })).toBe(true);     // an older catalogue: by its feet
    expect(liveCapable({ key: 'old', kind: 'x', feet: 2 })).toBe(false);
    expect(liveCapable(undefined)).toBe(false);
  });

  it("asks for the view on screen: its mode beside the design's query", () => {
    const q = new URLSearchParams(liveQuery('side', 'linkage=palletizer_reach&p.L1=0.4'));
    expect(q.get('mode')).toBe('side');
    expect(q.get('linkage')).toBe('palletizer_reach');
    expect(q.get('p.L1')).toBe('0.4');
  });

  it("sends each slider the session takes, clipped to the model's range", () => {
    const joints = [
      { name: 'q2', label: 'q2', lo: -10, hi: 110, value: 200 },
      { name: 'q1', label: 'yaw', lo: -150, hi: 150, value: -150 },
      { name: 'q4', label: 'wrist', lo: -90, hi: 90, value: 5 },          // not in the model
    ];
    expect(liveTargets(joints, hello)).toEqual({ q2: 110, q1: -120 });
    expect(liveTargets(joints, { joints: undefined })).toEqual({});
  });

  it("turns each crank the hello declares at the slider's speed", () => {
    expect(crankCommand(0.5, { drives: ['drive'] })).toEqual([0.5]);
    expect(crankCommand(3, { drives: ['L.drive', 'R.drive'] })).toEqual([1, 1]);
    expect(crankCommand(Number.NaN, { drives: ['drive'] })).toEqual([0]);
    expect(crankCommand(0.5, hello)).toEqual([]);                          // an arm: no crank
  });

  it('reads out what the session streams', () => {
    const arm = liveReadout({ 'tool0.x': 163.4, 'tool0.y': -16, 'tool0.z': 300.2, 'q2.angle': Math.PI / 4,
      'q2.torque': 0.04 }, hello);
    expect(arm).toContain('tool0 (163, -16, 300) mm');
    expect(arm).toContain('q2 45.0° 0.04 N·m');
    const mech = liveReadout({ 'drive.angle': 4 * Math.PI, 'drive.torque': 0.006 },
      { drives: ['drive'], sites: [], joints: {} });
    expect(mech).toBe('drive 2.00 rev 0.006 N·m');
  });
});
