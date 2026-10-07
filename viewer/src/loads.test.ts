/**
 * Loads superseded by newer ones (``loads.ts``, ``loader.ts``): the latest wins, the one before is
 * aborted at its fetch and never touches the scene.
 */
import { afterEach, describe, expect, it, vi } from 'vitest';
import * as THREE from 'three';
import { loadGlb } from './loader';
import { isAbort, latestLoader, latestOnly } from './loads';

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

describe('latestLoader (main.ts loadMode)', () => {
  /** A host whose fetches the test settles, recording what reached the screen. */
  function harness() {
    const pending: { mode: string; signal: AbortSignal; resolve(s: string): void; reject(e: unknown): void }[] = [];
    const log: string[] = [];
    const load = latestLoader<string>({
      begin: (mode) => { log.push(`begin ${mode}`); },
      fetch: (mode, _q, signal) => new Promise<string>((resolve, reject) => {
        pending.push({ mode, signal, resolve, reject });
      }),
      show: async (scene) => { log.push(`show ${scene}`); },
      discard: (scene) => { log.push(`discard ${scene}`); },
      end: () => { log.push('end'); },
    });
    return { pending, log, load };
  }

  it('shows only the latest of two loads that land out of order', async () => {
    const { pending, log, load } = harness();
    const first = load('robot', 'linkage=klann');
    const second = load('robot', 'linkage=strider');
    expect(pending[0]!.signal.aborted).toBe(true);     // the first fetch is aborted
    pending[1]!.resolve('strider');
    await second;
    pending[0]!.resolve('klann');                       // (a fetch that ignored its abort)
    await first;
    expect(log).toEqual(['begin robot', 'begin robot', 'show strider', 'end', 'discard klann']);
  });

  it('swallows the superseded load\'s abort but reports the latest one\'s failure', async () => {
    const { pending, log, load } = harness();
    const first = load('robot');
    const second = load('side');
    pending[0]!.reject(new DOMException('aborted', 'AbortError'));
    await expect(first).resolves.toBeUndefined();
    pending[1]!.reject(new Error('422 no plan'));
    await expect(second).rejects.toThrow('422 no plan');
    expect(log).toEqual(['begin robot', 'begin side', 'end']);
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
