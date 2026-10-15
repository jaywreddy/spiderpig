import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { jointsOf, posedMatrix, poseQuery } from './arm';

describe('arm joint sliders', () => {
  const info = { key: 'palletizer', kind: 'arm', inputs: ['q2', 'q3'],
    ranges: { q2: [40, 115] as [number, number], q3: [-45, 20] as [number, number] } };
  const tree = { joints: [{ name: 'q1', range_deg: [-150, 150] as [number, number], label: 'yaw' }] };

  it('lists the inputs, then the spatial joints, each mid-range', () => {
    const js = jointsOf(info, tree);
    expect(js.map((j) => j.name)).toEqual(['q2', 'q3', 'q1']);
    expect(js[0]!.value).toBeCloseTo(77.5);
    expect(js[2]!.label).toBe('yaw');
    expect(jointsOf({ key: 'strider', kind: 'walker' }, undefined)).toEqual([]);
    const seeded = jointsOf(info, tree, { q2: 40, q3: -45 });   // the bake's first frame
    expect([seeded[0]!.value, seeded[1]!.value]).toEqual([40, -45]);
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
