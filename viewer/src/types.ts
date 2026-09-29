import type * as THREE from 'three';

export type Mode = 'klann' | 'double' | 'decker' | 'double_double';

export interface ViewerHandle {
  readonly mixer: THREE.AnimationMixer | null;
  readonly action: THREE.AnimationAction | null;
  readonly clipDuration: number;
  readonly playing: boolean;
  /** The mode whose GLB is on screen (not merely requested). */
  readonly mode: Mode;
  step(dt: number): void;
  loadMode(m: Mode): Promise<void>;
  ready: boolean;
}

declare global {
  interface Window {
    __viewer?: ViewerHandle;
  }
}
