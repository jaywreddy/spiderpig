import type * as THREE from 'three';
import type { Drive } from './drive';

/** A server mode id (see ``/api/modes``): ``robot``, ``klann``, ``double``, … */
export type Mode = string;

/** Camera presets (see ``scene.ts``): world Z is up, the robot walks along X. */
export type View = 'three-quarter' | 'side' | 'front' | 'top';

export interface ViewerHandle {
  readonly mixer: THREE.AnimationMixer | null;
  readonly action: THREE.AnimationAction | null;
  readonly clipDuration: number;
  readonly playing: boolean;
  /** The mode whose GLB is on screen (not merely requested). */
  readonly mode: Mode;
  /** The ``walker`` root node of the loaded GLB (stands the model up). */
  readonly walker: THREE.Object3D | null;
  readonly camera: THREE.PerspectiveCamera;
  step(dt: number): void;
  /** Pause and pose the animation at clip time ``t`` (seconds). */
  seek(t: number): void;
  /** Move the camera to a preset view framing the loaded model. */
  setView(view: View): void;
  loadMode(m: Mode, query?: string): Promise<void>;
  /** Drive mode + tune panel (``drive/index.ts``). */
  readonly drive: Drive;
  ready: boolean;
}

declare global {
  interface Window {
    __viewer?: ViewerHandle;
  }
}
