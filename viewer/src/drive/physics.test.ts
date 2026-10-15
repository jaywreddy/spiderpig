/**
 * The live sim's client (``physics.ts``): the frame read by the hello's ``columns`` (a walker's
 * legacy 17 too), the commands of what drives a design (a walker's two cranks, a mechanism's one,
 * an arm's joints), and the parts posed with a fixed base left where the glb stands it. A fake
 * WebSocket stands in for ``/ws/sim``.
 */
import { afterEach, beforeEach, describe, expect, it } from 'vitest';
import * as THREE from 'three';
import type { LoadedScene } from '../loader';
import { LEGACY_COLUMNS, PhysicsLink, columnIndex, readValues, type Hello } from './physics';

class FakeSocket {
  static readonly OPEN = 1;
  static last: FakeSocket | null = null;
  readyState = FakeSocket.OPEN;
  binaryType = '';
  sent: string[] = [];
  onmessage: ((ev: { data: unknown }) => void) | null = null;
  onerror: (() => void) | null = null;
  onclose: (() => void) | null = null;
  constructor(readonly url: string) { FakeSocket.last = this; }
  send(m: string): void { this.sent.push(m); }
  close(): void { this.readyState = 3; }
  receive(data: unknown): void { this.onmessage?.({ data }); }
}

const g = globalThis as unknown as Record<string, unknown>;
let saved: Record<string, unknown>;
beforeEach(() => {
  saved = { WebSocket: g.WebSocket, location: g.location };
  g.WebSocket = FakeSocket;
  g.location = { protocol: 'http:', host: 'localhost:8000' };
});
afterEach(() => { Object.assign(g, saved); });

const WALKER_COLUMNS = ['t', 'base.x', 'base.y', 'base.z', 'base.qw', 'base.qx', 'base.qy', 'base.qz',
  'L.drive.angle', 'R.drive.angle', 'feet_down', 'L.drive.torque', 'R.drive.torque', 'body_down',
  'side_phase', 'az', 'pin_load'];
const ARM_COLUMNS = ['t', 'base.x', 'base.y', 'base.z', 'base.qw', 'base.qx', 'base.qy', 'base.qz', 'pin_load',
  'q1.angle', 'q2.angle', 'q1.torque', 'q2.torque', 'tool0.x', 'tool0.y', 'tool0.z'];
const MECH_COLUMNS = ['t', 'base.x', 'base.y', 'base.z', 'base.qw', 'base.qx', 'base.qy', 'base.qz',
  'drive.angle', 'drive.torque', 'pin_load', 'output.x', 'output.y', 'output.z'];

function hello(columns: string[], extra: Partial<Hello>): Hello {
  return {
    bodies: ['base', 'b1'], nodes: { frame: 'base', link: 'b1' }, header: columns.length, columns, vmax: 5,
    feet: 0, crank_sign: 1, rest_height: 0.1, design: { linkage: 'x', module: 'single', key: 'x' },
    torque_stall: 1, torque_rated: null, ...extra,
  };
}

/** A frame: the header's values (by column), then each body's 3x4 (identity, b1 moved +10 x). */
function frame(h: Hello, values: Record<string, number>): ArrayBuffer {
  const f = new Float32Array(h.header + 12 * h.bodies.length);
  h.columns!.forEach((c, i) => { f[i] = values[c] ?? 0; });
  h.bodies.forEach((_, b) => {
    const o = h.header + 12 * b;
    f.set([1, 0, 0, b ? 10 : 0, 0, 1, 0, 0, 0, 0, 1, 0], o);
  });
  return f.buffer;
}

function scene(): LoadedScene {
  const root = new THREE.Group();
  const walker = new THREE.Object3D();
  walker.position.set(0, 0, 123);                 // where the glb stands it
  walker.updateMatrix();
  root.add(walker);
  for (const name of ['frame', 'link']) {
    const n = new THREE.Object3D();
    n.name = name;
    n.userData.name = name;
    walker.add(n);
  }
  const mixer = new THREE.AnimationMixer(root);
  const clip = new THREE.AnimationClip('c', 1, []);
  return { root, walker, mixer, action: mixer.clipAction(clip), clipDuration: 1, clip, footLine: null, nodeCount: 2 };
}

async function connected(h: Hello, query = ''): Promise<{ link: PhysicsLink; ws: FakeSocket }> {
  const link = new PhysicsLink();
  const p = link.connect(query, () => {});
  const ws = FakeSocket.last!;
  ws.receive(JSON.stringify({ status: 'building', elapsed: 0 }));
  ws.receive(JSON.stringify({ hello: h }));
  await p;
  return { link, ws };
}

describe('the frame is read by its columns', () => {
  it("names a walker's header and aliases its drives to crank.k / torque.k", () => {
    const ix = columnIndex({ columns: WALKER_COLUMNS, header: 17, drives: ['L.drive', 'R.drive'] });
    expect(ix.get('crank.0')).toBe(8);
    expect(ix.get('crank.1')).toBe(9);
    expect(ix.get('torque.1')).toBe(12);
    expect(ix.get('pin_load')).toBe(16);
  });

  it("reads an older server's walker frame by the legacy layout", () => {
    expect(LEGACY_COLUMNS).toHaveLength(17);
    const ix = columnIndex({ header: 17, drives: ['L.drive', 'R.drive'] });
    expect(ix.get('feet_down')).toBe(10);
    expect(ix.get('crank.0')).toBe(8);
    const older = columnIndex({ header: 11 });           // before the torque columns
    expect(older.has('torque.0')).toBe(false);
    expect(readValues(new Float32Array([1, 2, 3]), older).feet_down).toBe(0);
  });

  it("reads an arm's joints and sites and a mechanism's one crank", () => {
    const arm = readValues(Float32Array.from(ARM_COLUMNS, (_, i) => i), columnIndex({ columns: ARM_COLUMNS, header: 16 }));
    expect(arm['q2.angle']).toBe(10);
    expect(arm['tool0.z']).toBe(15);
    expect(arm['crank.0']).toBeUndefined();
    const mech = columnIndex({ columns: MECH_COLUMNS, header: 14, drives: ['drive'] });
    expect(mech.get('crank.0')).toBe(8);
    expect(mech.get('torque.0')).toBe(9);
    expect(mech.has('crank.1')).toBe(false);
  });
});

describe('the session is driven by what the hello declares', () => {
  it("sends a walker's two crank speeds and a mechanism's one, on change only", async () => {
    const { link, ws } = await connected(hello(MECH_COLUMNS, { walks: false, drives: ['drive'] }), 'linkage=m&mode=side');
    expect(ws.url).toBe('ws://localhost:8000/ws/sim?linkage=m&mode=side');
    link.command(0.5);
    link.command(0.5);
    expect(ws.sent).toEqual(['{"cmd":[0.5]}']);
    const w = await connected(hello(WALKER_COLUMNS, { drives: ['L.drive', 'R.drive'] }));
    w.link.command(1, -1);
    expect(w.ws.sent).toEqual(['{"cmd":[1,-1]}']);
    expect(w.link.walks).toBe(true);                    // an older hello without walks: a walker
  });

  it("sends an arm's joint targets in degrees, on change only", async () => {
    const h = hello(ARM_COLUMNS, { walks: false, drives: [],
      joints: { q1: { lo: -120, hi: 120, value: 0, effort: 1.5 }, q2: { lo: -10, hi: 110, value: -10, effort: 2 } } });
    const { link, ws } = await connected(h);
    link.joints({ q2: 40 });
    link.joints({ q2: 40 });
    link.joints({});
    link.joints({ q1: 30, q2: 40 });
    expect(ws.sent).toEqual(['{"joints":{"q2":40}}', '{"joints":{"q1":30,"q2":40}}']);
    expect(link.walks).toBe(false);
    expect(link.info?.joints?.q2?.lo).toBe(-10);
  });

  it("poses a fixed design's parts and leaves its base where the glb stands it", async () => {
    const h = hello(ARM_COLUMNS, { walks: false, drives: [] });
    const { link, ws } = await connected(h);
    const l = scene();
    const before = l.walker.matrix.clone();
    ws.receive(frame(h, { t: 0.1, 'base.z': 500, 'base.qw': 1, 'q2.angle': 0.5, 'tool0.z': 321 }));
    expect(link.apply(l)).toBe(true);
    expect(l.walker.matrix.equals(before)).toBe(true);
    expect(l.walker.matrixAutoUpdate).toBe(true);
    const link1 = l.walker.children.find((n) => n.name === 'link')!;
    expect(new THREE.Vector3().setFromMatrixPosition(link1.matrix).x).toBeCloseTo(10);
    expect(link.frame?.values['tool0.z']).toBeCloseTo(321);
    expect(link.frame?.values['q2.angle']).toBeCloseTo(0.5);
    expect(link.frame?.crank).toEqual([0, 0]);
    link.unbind();
    expect(link1.matrixAutoUpdate).toBe(true);
  });

  it("moves a walker's root to its free base's world pose", async () => {
    const h = hello(WALKER_COLUMNS, { walks: true, drives: ['L.drive', 'R.drive'], feet: 4 });
    const { link, ws } = await connected(h);
    const l = scene();
    ws.receive(frame(h, { t: 0.1, 'base.x': 7, 'base.z': 500, 'base.qw': 1, 'L.drive.angle': 1, feet_down: 3,
      'R.drive.torque': 0.25 }));
    expect(link.apply(l)).toBe(true);
    expect(new THREE.Vector3().setFromMatrixPosition(l.walker.matrix).toArray()).toEqual([7, 0, 500]);
    expect(link.frame?.crank[0]).toBeCloseTo(1);
    expect(link.frame?.feetDown).toBe(3);
    expect(link.frame?.torque[1]).toBeCloseTo(0.25);
  });
});
