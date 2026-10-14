import type { Mode } from './types';

/** The views a linkage declares (``/api/modes?linkage=``): its robot when it makes one (a
 * walker's two sides, an arm on its turntable), then one side of each leg module (a
 * walker's) or its one side; the ids in the dropdown's order, their labels. */
export interface ModeCatalogue {
  linkage?: string;
  kind?: string;
  modes: Mode[];
  default: Mode;
  labels?: Record<string, string>;
}

/** The catalogue for ``linkage`` ('' or undefined: the server's default linkage), or null
 * when the server can't say. */
export async function fetchModes(linkage?: string): Promise<ModeCatalogue | null> {
  try {
    const q = linkage ? `?${new URLSearchParams({ linkage }).toString()}` : '';
    const res = await fetch(`/api/modes${q}`);
    if (!res.ok) return null;
    return (await res.json()) as ModeCatalogue;
  } catch {
    return null;
  }
}

/** The dropdown's options: the catalogue's, plus ``mode`` when it isn't one of them (a
 * stored one-sided design's ``side``: an id for URLs, offered then as "this design"). */
export function withMode(cat: ModeCatalogue, mode?: Mode | null): { modes: Mode[]; labels: Record<string, string> } {
  const labels = { ...(cat.labels ?? {}) };
  if (!mode || cat.modes.includes(mode)) return { modes: [...cat.modes], labels };
  return { modes: [...cat.modes, mode], labels: { ...labels, [mode]: `${mode} (this design)` } };
}

/** The mode to show: ``wanted`` when the catalogue (or the design) offers it, else the
 * catalogue's default. */
export function pickMode(cat: ModeCatalogue, wanted?: Mode | null, own?: Mode | null): Mode {
  const { modes } = withMode(cat, own);
  return wanted && modes.includes(wanted) ? wanted : cat.default;
}

/** One entry of ``/api/linkages`` as the machine picker reads it. */
export interface MachineInfo { key: string; name: string; kind: string; feet: number }

/** The machine picker's options (label -> key): every linkage without feet (an arm, a
 * mechanism: no walk to tune), labelled by what the server calls it. */
export function machines(linkages: readonly MachineInfo[]): Record<string, string> {
  return Object.fromEntries(linkages.filter((l) => l.feet === 0)
    .map((l) => [`${l.kind}: ${l.name}`, l.key]));
}
