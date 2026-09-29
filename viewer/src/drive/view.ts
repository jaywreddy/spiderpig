/**
 * What driving adds to the scene: the body pose on the ground, the two sides
 * animated independently, trail, feet / support polygon / COM overlay, the
 * stick-figure preview of an ``/api/walk`` design, and a camera that follows.
 */
import * as THREE from 'three';
import type { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { gridAt, type Side, type WalkState } from './model';
import type { Sim } from './sim';

/** The baked root's standing orientation: mech +y (up) -> world +z. */
const STAND = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2);
const UP = new THREE.Vector3(0, 1, 0);

/** World pose of the body: ground frame T(x, h, z) Ry(yaw) R(n -> +y), stood up by STAND. */
export function bodyMatrix(sim: Sim, s: WalkState): THREE.Matrix4 {
  const q = STAND.clone()
    .multiply(new THREE.Quaternion().setFromAxisAngle(UP, sim.yaw))
    .multiply(new THREE.Quaternion().setFromUnitVectors(s.normal, UP));
  const p = new THREE.Vector3(sim.x, s.height, sim.z).applyQuaternion(STAND);
  return new THREE.Matrix4().compose(p, q, new THREE.Vector3(1, 1, 1));
}

/** The baked clip split by node side (``extras.body`` "L." / "R."), one action per side. */
export function sideActions(mixer: THREE.AnimationMixer, clip: THREE.AnimationClip, root: THREE.Object3D) {
  const side = new Map<string, Side>();
  root.traverse((o) => { const b = o.userData.body; if (/^[LR]\./.test(b)) side.set(o.name, b[0]); });
  const t = clip.tracks[0]!.times;
  const period = t[t.length - 1]! + (t[1]! - t[0]!);     // keys cover [0, D) evenly
  const action = (s: Side): THREE.AnimationAction => mixer.clipAction(new THREE.AnimationClip(`walk_${s}`, period,
    clip.tracks.filter((tr) => side.get(THREE.PropertyBinding.parseTrackName(tr.name).nodeName) === s).map((tr) => {
      // Close the loop (a key at D repeating the first) so every crank angle interpolates.
      const k = tr.getValueSize(), v = new Float32Array(tr.values.length + k);
      v.set(tr.values); v.set(tr.values.subarray(0, k), tr.values.length);
      type Ctor = new (n: string, t: ArrayLike<number>, v: ArrayLike<number>) => THREE.KeyframeTrack;
      return new (tr.constructor as Ctor)(tr.name, [...tr.times, period], v);
    })));
  return { L: action('L'), R: action('R'), period, usable: side.size > 0 };
}

/** Write a vec3 attribute in place (growing it when needed) and draw that many vertices. */
function put(g: THREE.BufferGeometry, name: string, values: number[]): void {
  let a = g.getAttribute(name) as THREE.BufferAttribute | undefined;
  if (!a || a.array.length < values.length) {
    g.dispose();
    g.setAttribute(name, (a = new THREE.BufferAttribute(new Float32Array(Math.max(values.length, 96)), 3)));
  }
  (a.array as Float32Array).set(values);
  a.needsUpdate = true;
  g.setDrawRange(0, values.length / 3);
}

const flat = (pts: THREE.Vector3[]): number[] => pts.flatMap((p) => p.toArray());

/** A 20 m ground plane with a 100 mm grid texture (a mesh: a GridHelper's long lines through the camera
 * plane don't clip everywhere, software GL drops them). */
function ground(): THREE.Mesh {
  const c = Object.assign(document.createElement('canvas'), { width: 512, height: 512 }), g = c.getContext('2d')!;
  g.fillStyle = '#151a20';
  g.fillRect(0, 0, 512, 512);
  g.strokeStyle = '#34404b';
  for (let i = 0; i < 10; i++) {
    g.lineWidth = i ? 2 : 6;
    g.strokeRect(i * 51.2, -10, 0, 532);
    g.strokeRect(-10, i * 51.2, 532, 0);
  }
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.repeat.set(20, 20);           // 1 m per tile
  tex.anisotropy = 8;
  tex.colorSpace = THREE.SRGBColorSpace;
  return new THREE.Mesh(new THREE.PlaneGeometry(20000, 20000), new THREE.MeshBasicMaterial({ map: tex }));
}

export class DriveView {
  readonly overlay = new THREE.Group();
  readonly stick = new THREE.LineSegments(new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ vertexColors: true }));
  private readonly feet = new THREE.Points(new THREE.BufferGeometry(),
    new THREE.PointsMaterial({ size: 9, sizeAttenuation: false, vertexColors: true, depthTest: false }));
  private readonly poly = new THREE.LineSegments(new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ depthTest: false }));
  private readonly com = new THREE.Mesh(new THREE.SphereGeometry(6), new THREE.MeshBasicMaterial({ color: 0xffffff }));
  private readonly grid = ground();
  private readonly trail = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xffc14d }));
  private trailPts: number[] = [];
  private stickLegs: { side: Side; z: number; joints: Float64Array[] }[] = [];
  private stickLinks: [number, number][] = [];
  private last: { p: THREE.Vector3; heading: number } | null = null;

  constructor(scene: THREE.Scene) {
    for (const o of [this.overlay, this.stick]) { o.matrixAutoUpdate = false; o.visible = false; scene.add(o); }
    this.overlay.add(this.feet, this.poly, this.com);
    this.feet.renderOrder = this.poly.renderOrder = 2;
    for (const o of [this.feet, this.poly, this.stick, this.trail]) o.frustumCulled = false;
    scene.add(this.trail, this.grid);
    this.visible = false;
  }

  set visible(on: boolean) {
    this.overlay.visible = this.trail.visible = this.grid.visible = on;
    if (!on) this.stick.visible = false;
  }

  clearTrail(): void { this.trailPts = []; this.trail.geometry.setDrawRange(0, 0); this.last = null; }

  /** Place the overlay (and the stick figure) at the body pose ``m``; update feet, polygon, COM, trail. */
  update(m: THREE.Matrix4, s: WalkState, com: THREE.Vector3, sim: Sim, showSupport: boolean): void {
    this.overlay.matrix.copy(m);
    this.stick.matrix.copy(m);
    put(this.feet.geometry, 'position', flat(s.feet));
    put(this.feet.geometry, 'color', s.contacts.flatMap((c) => (c ? [0.22, 0.85, 0.54] : [0.5, 0.55, 0.6])));
    const lift = s.normal.clone().multiplyScalar(0.5);
    put(this.poly.geometry, 'position', flat([...s.edges.flat(), com, s.comOnPlane].map((p) => p.clone().add(lift))));
    const stable = s.margin >= 0 && !s.tipping && !s.degenerate;
    (this.poly.material as THREE.LineBasicMaterial).color.set(stable ? 0x39d98a : 0xff4d4d);
    this.poly.visible = this.com.visible = showSupport;
    this.com.position.copy(com);
    this.stickPose(sim.modelTheta);
    this.grid.position.set(Math.round(m.elements[12]! / 1000) * 1000, Math.round(m.elements[13]! / 1000) * 1000, -0.5);
    // Trail: the body origin on the ground (world), a point every 4 mm, the last 4000.
    const p = new THREE.Vector3(sim.x, 0.5, sim.z).applyQuaternion(STAND), n = this.trailPts.length;
    if (n && Math.hypot(this.trailPts[n - 3]! - p.x, this.trailPts[n - 2]! - p.y) < 4) return;
    this.trailPts.push(p.x, p.y, p.z);
    if (n > 12000) this.trailPts.splice(0, 6000);
    put(this.trail.geometry, 'position', this.trailPts);
  }

  /** Stick figure of an ``/api/walk`` design: every link of every leg, both sides (R = z-mirror of L). */
  setStick(walk: { legs: { leg: number; joints: Record<string, [number, number][]> }[]; links: [string, string][];
    feet: { side: Side; leg: number; z: number }[]; side_z?: Partial<Record<Side, number>> }): void {
    const names = Object.keys(walk.legs[0]?.joints ?? {});
    this.stickLinks = walk.links.map(([a, b]): [number, number] => [names.indexOf(a), names.indexOf(b)])
      .filter(([a, b]) => a >= 0 && b >= 0);
    this.stickLegs = (['L', 'R'] as const).flatMap((side) => walk.legs.map((leg) => ({
      side, z: walk.feet.find((f) => f.side === side && f.leg === leg.leg)?.z ?? walk.side_z?.[side] ?? 0,
      joints: names.map((j) => Float64Array.from(leg.joints[j]!.flat())),
    })));
    this.stick.geometry.dispose();
    this.stick.geometry = new THREE.BufferGeometry();
    put(this.stick.geometry, 'color', this.stickLegs.flatMap((l) => this.stickLinks.flatMap(() =>
      (l.side === 'L' ? [0.35, 0.66, 1] : [1, 0.64, 0.3]).concat(l.side === 'L' ? [0.35, 0.66, 1] : [1, 0.64, 0.3]))));
  }

  private stickPose(theta: Record<Side, number>): void {
    if (!this.stick.visible) return;
    const pts: number[] = [];
    for (const leg of this.stickLegs) {
      const n = leg.joints[0]!.length / 2, [i, t] = gridAt(theta[leg.side], n), i1 = (i + 1) % n;
      const at = (a: Float64Array): number[] =>
        [a[2 * i]! + (a[2 * i1]! - a[2 * i]!) * t, a[2 * i + 1]! + (a[2 * i1 + 1]! - a[2 * i + 1]!) * t, leg.z];
      for (const [a, b] of this.stickLinks) pts.push(...at(leg.joints[a]!), ...at(leg.joints[b]!));
    }
    put(this.stick.geometry, 'position', pts);
  }

  /** Keep the camera with the robot (orbit still works): translate with it; in chase, turn with it too. */
  follow(camera: THREE.Camera, controls: OrbitControls, m: THREE.Matrix4, heading: number, chase: boolean): void {
    const p = new THREE.Vector3().setFromMatrixPosition(m).setZ(80);
    if (!this.last) {   // start behind the robot, a little to its left and above
      const back = new THREE.Vector3(-Math.cos(heading), -Math.sin(heading), 0);
      controls.target.copy(p);
      camera.position.copy(p).addScaledVector(back, 900).add(new THREE.Vector3(back.y * 400, -back.x * 400, 380));
    } else {
      if (chase) {
        const off = camera.position.clone().sub(controls.target)
          .applyAxisAngle(new THREE.Vector3(0, 0, 1), heading - this.last.heading);
        camera.position.copy(controls.target).add(off);
      }
      const d = p.clone().sub(this.last.p);
      camera.position.add(d);
      controls.target.add(d);
    }
    this.last = { p, heading };
  }
}
