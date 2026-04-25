import type * as THREE from 'three';

export type Mode = 'klann' | 'double' | 'double_double';

export interface ViewerHandle {
  readonly mixer: THREE.AnimationMixer | null;
  readonly action: THREE.AnimationAction | null;
  readonly clipDuration: number;
  readonly playing: boolean;
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
