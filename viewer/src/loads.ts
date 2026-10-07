/**
 * One load at a time, the latest wins: ``begin()`` aborts the load before it (its fetch, through
 * the ``AbortSignal``) and hands out a ticket whose ``current`` turns false once a newer load has
 * begun. ``main.ts``'s ``loadMode`` takes one per GLB, so a slow bake of a design the user has
 * already left can't land on screen over the newer one (the generation counter ``physics.ts``'s
 * ``connect`` keeps for its sockets).
 */
export interface LoadTicket {
  /** Aborted when a newer load begins: pass it to ``fetch``. */
  readonly signal: AbortSignal;
  /** Is this still the latest load? */
  readonly current: boolean;
}

export interface LatestOnly {
  begin(): LoadTicket;
  /** The generation: how many loads have begun. */
  readonly generation: number;
}

export function latestOnly(): LatestOnly {
  let gen = 0;
  let ctrl: AbortController | null = null;
  return {
    begin(): LoadTicket {
      ctrl?.abort();
      const mine = ++gen;
      const c = (ctrl = new AbortController());
      return {
        signal: c.signal,
        get current() { return mine === gen; },
      };
    },
    get generation() { return gen; },
  };
}

/** Was ``err`` an abort (a load superseded by a newer one)? */
export function isAbort(err: unknown): boolean {
  return (err as { name?: string } | null)?.name === 'AbortError';
}
