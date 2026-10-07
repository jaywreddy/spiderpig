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

/** What :func:`latestLoader` does with a load: ``begin`` (the UI says it is loading),
 * ``fetch`` it (with the ticket's signal), ``show`` the latest one's result, ``discard`` a
 * result a newer load superseded, ``end`` once the latest one is done (either way). */
export interface LoaderHost<S, M extends string = string> {
  begin(mode: M, query: string): void;
  fetch(mode: M, query: string, signal: AbortSignal): Promise<S>;
  show(scene: S, mode: M, query: string): Promise<void>;
  discard(scene: S): void;
  end(): void;
}

/** ``loadMode``: the latest call wins. A call superseded by a newer one resolves without
 * showing anything (its fetch aborted, or its result discarded if it had already arrived);
 * a failure of the latest one rejects. */
export function latestLoader<S, M extends string = string>(
  host: LoaderHost<S, M>,
): (mode: M, query?: string) => Promise<void> {
  const loads = latestOnly();
  return async (mode: M, query = ''): Promise<void> => {
    const ticket = loads.begin();
    host.begin(mode, query);
    try {
      let next: S;
      try {
        next = await host.fetch(mode, query, ticket.signal);
      } catch (err) {
        if (!ticket.current || isAbort(err)) return;   // superseded: the newer load reports
        throw err;
      }
      if (!ticket.current) {        // a newer load began while this one arrived: drop it
        host.discard(next);
        return;
      }
      await host.show(next, mode, query);
    } finally {
      if (ticket.current) host.end();
    }
  };
}
