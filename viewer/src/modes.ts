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

/** One entry of ``/api/linkages`` as the linkage picker reads it. */
export interface LinkageEntry { key: string; name: string; kind: string }

/** A group of the linkage picker: one kind's linkages, each option ``[label, key]``. */
export interface LinkageGroup { kind: string; label: string; options: [string, string][] }

/** The linkage picker's groups: every registered linkage, one group per kind in the order
 * the server lists them (``/api/linkages``), each option labelled by its name (a name two
 * linkages share gets its key). One picker for every kind of machine: what the views and
 * the tune panel then offer follows the linkage's declared capabilities (its modes from
 * ``/api/modes``, its feet), never its kind's name. */
export function linkageGroups(linkages: readonly LinkageEntry[]): LinkageGroup[] {
  const groups: LinkageGroup[] = [];
  const named = new Map<string, number>();
  for (const l of linkages) named.set(l.name, (named.get(l.name) ?? 0) + 1);
  for (const l of linkages) {
    let g = groups.find((x) => x.kind === l.kind);
    if (!g) groups.push((g = { kind: l.kind, label: `${l.kind}s`, options: [] }));
    g.options.push([(named.get(l.name) ?? 0) > 1 ? `${l.name} (${l.key})` : l.name, l.key]);
  }
  return groups;
}

/** The dropdown's options (label -> key), the groups' in their order: what lil-gui takes. */
export function linkageOptions(groups: readonly LinkageGroup[]): Record<string, string> {
  return Object.fromEntries(groups.flatMap((g) => g.options));
}

/** Wrap a ``<select>``'s options (made from :func:`linkageOptions`, in that order) into one
 * ``<optgroup>`` per kind. The options keep their order, so the select's ``selectedIndex``
 * still maps onto the controller's values. */
export function groupSelect(select: HTMLSelectElement, groups: readonly LinkageGroup[]): void {
  const options = Array.from(select.options);
  let i = 0;
  for (const g of groups) {
    const og = select.ownerDocument.createElement('optgroup');
    og.label = g.label;
    for (let k = 0; k < g.options.length && i < options.length; k++) og.append(options[i++]!);
    select.append(og);
  }
}
