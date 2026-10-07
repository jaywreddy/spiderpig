/**
 * Loads superseded by newer ones (``loads.ts``, ``loader.ts``): the latest wins, the one before is
 * aborted at its fetch and never touches the scene.
 */
import { afterEach, describe, expect, it, vi } from 'vitest';
import * as THREE from 'three';
import { loadGlb } from './loader';
import { isAbort, latestOnly } from './loads';

afterEach(() => { vi.unstubAllGlobals(); });

describe('latestOnly', () => {
  it('aborts the load before and makes it stale', () => {
    const loads = latestOnly();
    const a = loads.begin();
    expect(a.current).toBe(true);
    expect(a.signal.aborted).toBe(false);
    const b = loads.begin();
    expect(a.signal.aborted).toBe(true);
    expect(a.current).toBe(false);
    expect(b.current).toBe(true);
    expect(b.signal.aborted).toBe(false);
    expect(loads.generation).toBe(2);
  });

  it('lands only the latest of two loads that finish out of order', async () => {
    const loads = latestOnly();
    const shown: string[] = [];
    const load = async (name: string, ms: number): Promise<void> => {
      const t = loads.begin();
      await new Promise((r) => setTimeout(r, ms));
      if (t.current) shown.push(name);
    };
    await Promise.all([load('slow, first', 30), load('fast, second', 5)]);
    expect(shown).toEqual(['fast, second']);
  });
});

describe('loadGlb', () => {
  it('fetches with the signal and leaves the scene alone when aborted', async () => {
    const seen: (AbortSignal | null | undefined)[] = [];
    vi.stubGlobal('fetch', vi.fn((_url: string, init?: RequestInit) => {
      seen.push(init?.signal);
      return new Promise<Response>((_resolve, reject) => {
        init?.signal?.addEventListener('abort', () => {
          reject(new DOMException('The operation was aborted.', 'AbortError'));
        });
      });
    }));
    const scene = new THREE.Scene();
    const ctrl = new AbortController();
    const pending = loadGlb(scene, 'robot', 'linkage=klann', ctrl.signal);
    ctrl.abort();
    const err = await pending.catch((e: unknown) => e);
    expect(seen).toHaveLength(1);
    expect(seen[0]).toBe(ctrl.signal);
    expect(isAbort(err)).toBe(true);
    expect(scene.children).toHaveLength(0);
  });
});
