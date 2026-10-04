/**
 * Physics drive: the robot in MuJoCo on the server (``/ws/sim``,
 * ``spiderpig.sim.live``), driven live. Commands go up as crank-speed
 * fractions; frames come back in real time and are drawn with the node
 * contract of ``sim.mjcf``: the ``walker`` root at the base's world pose, each
 * glb node at ``D_body @ M0`` (``M0`` its frame-0 matrix). Each draw
 * interpolates between the last two frames by wall time (frames arrive at
 * 60 Hz with jitter; rAF runs at the display's rate).
 */
import * as THREE from 'three';
import type { LoadedScene } from '../loader';

/** The server's steering check of the model (``sim.run.steering_check``): the proven-safe
 * ``|L − R|`` while walking (the sides drifting apart without bound) and the proven-safe
 * speed of a turn in place, 0 when that run fell or tilted past 20°; ``step_deg``, the
 * largest side offset it walked with (the server's phase lock bounds a steering excursion
 * to it and re-locks: 90, 45 or 0); ``forward``, the straight run's verdict. */
export interface Steering {
  turn: number; spin: number; step_deg: number; seconds: number;
  forward: { fell: boolean; max_tilt: number; side_support_low: number; speed: number; walks: boolean;
    body_contact: number; stride: number } | Record<string, never>;
  tests: Record<string, { cmd: [number, number]; fell: boolean; fell_at_s: number | null; max_tilt: number;
    yaw_rate: number; side_support_low: number; side_phase_max?: number }>;
}

export interface Hello {
  bodies: string[]; nodes: Record<string, string>; header: number; vmax: number; feet: number;
  crank_sign: number; rest_height: number; design: { linkage: string; module: string; key: string };
  /** The servo's stall and rated (continuous, null when the catalog has none) torque, N·m. */
  torque_stall: number; torque_rated: number | null; steering: Steering;
  /** The straight run's verdict (``steering.forward``): the physics gate. */
  forward?: Steering['forward'];
  /** The no-slip stride the kinematics promise (mm per revolution): the measured stride is it minus the skating. */
  kinematic_stride_mm?: number;
  mass?: number; payload_g?: number;
  phase_lock?: { kp: number; ki: number; servo_mismatch: number; step_deg: number | null };
}

export interface PhysicsFrame {
  t: number; pos: THREE.Vector3; quat: THREE.Quaternion;
  /** Crank angles turned since the start, the design's sign (``crank_sign`` applied). */
  crank: [number, number]; feetDown: number;
  /** Drive torque on each crank, N·m (the actuator force; 0 from a server that doesn't send it). */
  torque: [number, number];
  /** A body that carries no foot is on the floor (the frame or a link away from the feet). */
  bodyDown: boolean;
  /** The sides' crank difference L − R (rad: what the server's phase lock holds). */
  sidePhase: number;
  /** The base's vertical acceleration (m/s²) and the largest in-plane force a loop's pin carries (N). */
  az: number; pinLoad: number;
}

/** The frame header's columns past the feet count (``sim.live.HEADER``); an older server's header ends
 * before them, and the frame reads 0 there. */
const TORQUE_AT = 11, BODY_DOWN_AT = 13, SIDE_PHASE_AT = 14, AZ_AT = 15, PIN_AT = 16;
const RATE_WINDOW_MS = 3000;   // of frame arrivals behind ``rate`` and ``fps``

interface Bound { node: THREE.Object3D; body: number; m0: THREE.Matrix4 }

/** Nodes of the loaded glb the model doesn't know (``hello.nodes`` must cover them all). */
export function unboundNodes(l: LoadedScene, h: Hello): string[] {
  return l.walker.children
    .map((o) => o.userData.name as string | undefined)
    .filter((n): n is string => !!n && !(n in h.nodes));
}

const _qa = new THREE.Quaternion(), _qb = new THREE.Quaternion(), _va = new THREE.Vector3(), _vb = new THREE.Vector3();
const _ma = new THREE.Matrix4(), _mb = new THREE.Matrix4(), _one = new THREE.Vector3(1, 1, 1);

/** ``a`` and ``b`` as 3x4 row-major blocks of a frame, blended by ``s`` into ``out`` (rotation slerped). */
function blend(a: Float32Array, b: Float32Array, o: number, s: number, out: THREE.Matrix4): void {
  if (s >= 1) {
    out.set(b[o]!, b[o + 1]!, b[o + 2]!, b[o + 3]!, b[o + 4]!, b[o + 5]!, b[o + 6]!, b[o + 7]!,
      b[o + 8]!, b[o + 9]!, b[o + 10]!, b[o + 11]!, 0, 0, 0, 1);
    return;
  }
  _ma.set(a[o]!, a[o + 1]!, a[o + 2]!, 0, a[o + 4]!, a[o + 5]!, a[o + 6]!, 0, a[o + 8]!, a[o + 9]!, a[o + 10]!, 0, 0, 0, 0, 1);
  _mb.set(b[o]!, b[o + 1]!, b[o + 2]!, 0, b[o + 4]!, b[o + 5]!, b[o + 6]!, 0, b[o + 8]!, b[o + 9]!, b[o + 10]!, 0, 0, 0, 0, 1);
  _qa.setFromRotationMatrix(_ma).slerp(_qb.setFromRotationMatrix(_mb), s);
  _va.set(a[o + 3]!, a[o + 7]!, a[o + 11]!).lerp(_vb.set(b[o + 3]!, b[o + 7]!, b[o + 11]!), s);
  out.compose(_va, _qa, _one);
}

export class PhysicsLink {
  private ws: WebSocket | null = null;
  private hello: Hello | null = null;
  private gen = 0;                      // the socket connect() last opened; older ones are ignored
  private prev: Float32Array | null = null;
  private cur: Float32Array | null = null;
  private curAt = 0;                    // performance.now() when ``cur`` arrived
  private arrivals: { t: number; at: number }[] = [];   // the last seconds of frames: sim t vs wall
  /** Bumped when the sim clock went back (a reset landed): the owner's histories start over. */
  epoch = 0;
  private bound: Bound[] = [];
  private boundTo: LoadedScene | null = null;
  private sent = '';
  private readonly d = new THREE.Matrix4();
  frame: PhysicsFrame | null = null;
  /** The session ended after its hello (the socket dropped, or the server's model went
   * stale after a source change): the owner turns physics off or reconnects. */
  onLost: ((reason: string, stale: boolean) => void) | null = null;
  /** A JSON ``error`` after the hello (a rejected command). */
  onError: ((message: string) => void) | null = null;
  get feet(): number { return this.hello?.feet ?? 0; }
  get design(): Hello['design'] | null { return this.hello?.design ?? null; }
  get steering(): Steering | null { return this.hello?.steering ?? null; }
  get torqueStall(): number { return this.hello?.torque_stall ?? 0; }
  get torqueRated(): number | null { return this.hello?.torque_rated ?? null; }
  /** The no-slip stride the kinematics promise (mm per revolution), null from an older server. */
  get kinematicStride(): number | null { return this.hello?.kinematic_stride_mm ?? null; }
  /** Sim seconds per wall second over the last ``RATE_WINDOW_MS`` of frames (1 = real time; under
   * it the server fell behind: a loaded machine); NaN before two frames. Reads a little over 1 with
   * arrival jitter: the owner clamps what it shows. */
  get rate(): number {
    const a = this.arrivals[0], b = this.arrivals[this.arrivals.length - 1];
    return a && b && b.at > a.at + 100 ? ((b.t - a.t) * 1000) / (b.at - a.at) : NaN;
  }

  /** Frames received per wall second over the same window (the server sends 60). */
  get fps(): number {
    const a = this.arrivals[0], b = this.arrivals[this.arrivals.length - 1];
    return a && b && b.at > a.at + 100 ? ((this.arrivals.length - 1) * 1000) / (b.at - a.at) : NaN;
  }
  get open(): boolean { return this.ws?.readyState === WebSocket.OPEN && !!this.hello; }

  /** Connect for the design ``query`` ('' = default); resolves with the hello once the model is
   * built. With ``scene``, refuses (closing the socket) when the model doesn't name every node
   * of that glb: parts would move under the wrong model. A connect superseded by a newer
   * connect() or close() rejects with ``null`` (nothing to show). */
  connect(query: string, status: (s: string) => void, scene: LoadedScene | null = null): Promise<Hello> {
    this.close();
    const gen = ++this.gen;
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    const ws = (this.ws = new WebSocket(`${proto}://${location.host}/ws/sim${query ? `?${query}` : ''}`));
    ws.binaryType = 'arraybuffer';
    const live = (): boolean => gen === this.gen && this.ws === ws;
    return new Promise<Hello>((resolve, reject) => {
      let settled = false;
      const finish = (ok: Hello | null, err?: Error): void => {
        if (settled) return;
        settled = true;
        if (ok) resolve(ok); else reject(err ?? null);
      };
      ws.onmessage = (ev) => {
        if (!live()) { ws.close(); return; }           // superseded: this socket's frames are noise
        if (ev.data instanceof ArrayBuffer) {
          const next = new Float32Array(ev.data);
          if (this.cur && next[0]! < this.cur[0]!) {   // the clock went back: a reset landed
            this.prev = null;
            this.arrivals = [];
            this.epoch++;
          } else {
            this.prev = this.cur;
          }
          this.cur = next;
          this.curAt = performance.now();
          this.arrivals.push({ t: this.cur[0]!, at: this.curAt });
          while (this.arrivals.length > 2 && this.arrivals[0]!.at < this.curAt - RATE_WINDOW_MS) this.arrivals.shift();
          return;
        }
        const msg = JSON.parse(ev.data as string) as { status?: string; elapsed?: number; error?: string; hello?: Hello };
        if (msg.status === 'building') status(`physics: building the MuJoCo model… ${msg.elapsed ?? 0} s`);
        if (msg.status === 'stale' && this.hello) { this.onLost?.('the model changed (sources edited)', true); return; }
        if (msg.error) {
          if (this.hello) this.onError?.(msg.error); else finish(null, new Error(`physics: ${msg.error}`));
        }
        if (msg.hello) {
          const h = msg.hello;
          if (scene) {
            const missing = unboundNodes(scene, h);
            if (missing.length) {
              ws.close();
              finish(null, new Error(`physics: the server simulates ${h.design.linkage} ${h.design.module} but the glb `
                + `on screen has ${missing.length} node(s) it doesn't know (${missing.slice(0, 3).join(', ')}…): `
                + 'the model and the parts disagree'));
              return;
            }
          }
          this.hello = h;
          this.sent = '';
          finish(h);
        }
      };
      ws.onerror = () => finish(null, live() ? new Error('physics: connection failed') : undefined);
      ws.onclose = () => {
        if (!settled) { finish(null, live() ? new Error('physics: connection closed') : undefined); return; }
        if (live() && this.hello) this.onLost?.('connection lost', false);
      };
    });
  }

  close(): void {
    this.gen++;                           // anything still in flight is superseded
    const ws = this.ws;
    Object.assign(this, { ws: null, hello: null, prev: null, cur: null, frame: null, arrivals: [] });
    ws?.close();
  }

  /** Crank commands, fractions of the servo's speed (robot left, right); sent on change. */
  command(left: number, right: number): void {
    const msg = JSON.stringify({ cmd: [left, right] });
    if (this.open && msg !== this.sent) { this.ws!.send(msg); this.sent = msg; }
  }

  reset(): void { if (this.open) this.ws!.send(JSON.stringify({ reset: true })); }

  /** Pose ``l`` from the frames (interpolated to now); false until one has arrived. */
  apply(l: LoadedScene): boolean {
    const h = this.hello, f = this.cur;
    if (!h || !f) return false;
    if (this.boundTo !== l) this.bind(l, h);
    const p = this.prev ?? f, dt = (f[0]! - p[0]!) * 1000;
    const s = dt > 0 ? Math.min(1, Math.max(0, (performance.now() - this.curAt) / dt)) : 1;
    const n = h.header;
    const pos = _va.set(p[1]!, p[2]!, p[3]!).lerp(_vb.set(f[1]!, f[2]!, f[3]!), s).clone();
    const quat = _qa.set(p[5]!, p[6]!, p[7]!, p[4]!).slerp(_qb.set(f[5]!, f[6]!, f[7]!, f[4]!), s).clone();
    const k = h.crank_sign, col = (i: number): number => (n > i ? f[i]! : 0);
    this.frame = {
      t: p[0]! + (f[0]! - p[0]!) * s, pos, quat,
      crank: [k * (p[8]! + (f[8]! - p[8]!) * s), k * (p[9]! + (f[9]! - p[9]!) * s)], feetDown: f[10]!,
      torque: [col(TORQUE_AT), col(TORQUE_AT + 1)],
      bodyDown: col(BODY_DOWN_AT) > 0.5, sidePhase: col(SIDE_PHASE_AT), az: col(AZ_AT), pinLoad: col(PIN_AT),
    };
    l.walker.matrix.compose(pos, quat, _one);
    for (const b of this.bound) {
      blend(p, f, n + 12 * b.body, s, this.d);
      b.node.matrix.multiplyMatrices(this.d, b.m0);
    }
    return true;
  }

  /** Take over ``l``'s nodes: their frame-0 matrices, then hand-set matrices from here on. */
  private bind(l: LoadedScene, h: Hello): void {
    this.unbind();
    const missing = unboundNodes(l, h);
    if (missing.length) throw new Error(`physics: ${missing.length} glb node(s) aren't in the model (${missing[0]}…)`);
    l.mixer.stopAllAction();
    l.action.reset().play();
    l.action.paused = true;
    l.mixer.setTime(0);
    const index = new Map(h.bodies.map((b, i) => [b, i]));
    this.bound = [];
    for (const node of l.walker.children) {
      const body = index.get(h.nodes[node.userData.name as string] ?? '');
      if (body === undefined) { node.visible = node.name !== 'foot_path' && node.visible; continue; }
      node.updateMatrix();
      this.bound.push({ node, body, m0: node.matrix.clone() });
    }
    l.mixer.stopAllAction();
    for (const b of this.bound) b.node.matrixAutoUpdate = false;
    l.walker.matrixAutoUpdate = false;
    this.boundTo = l;
  }

  /** Give the nodes back to the animation (TRS-driven again). */
  unbind(): void {
    const l = this.boundTo;
    if (!l) return;
    for (const b of this.bound) {
      b.node.matrixAutoUpdate = true;
      b.node.matrix.decompose(b.node.position, b.node.quaternion, b.node.scale);
    }
    l.walker.matrixAutoUpdate = true;
    l.walker.traverse((o) => { if (o.name === 'foot_path') o.visible = true; });
    this.bound = [];
    this.boundTo = null;
  }
}
