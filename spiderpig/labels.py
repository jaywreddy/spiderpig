"""Part labels: one identifier per part type, shared by ``spiderpig build``'s print and cut
files and the assembly guide (``spiderpig guide``).

A label says what the part is, so no reordering of the steps, no edit of a hook and no new
part can change what a label means, and two near-identical parts can't swap labels:

- **printed** (a shape group of :func:`hardware.bom.group_made`, split by filament as the
  print files are): a family code and its defining sizes, ``SP8-0.7`` (a spacer 8 mm
  across, 0.7 mm high), ``RG9.3-3.0`` (a ring), ``RR11.9-2.6`` (a crank rider ring),
  ``CL8.9-0.4`` (a collar), ``SL8.5x23.9`` (a sleeve, across x long), ``TS8.5x8.9`` (a
  thrust sleeve), ``HS19.9-1.8`` (the horn spacer), ``FS14.9-3.0`` (a foot sock), ``DR``
  and ``BC`` (the deck rail, the battery cradle) by their sizes; another filament than the
  design's adds it (``-PETG``), a mirror image ``M``;
- **laser-cut**: ``LK`` a link, ``FR`` a frame plate, ``CW`` a crank web, ``CP`` a centre
  plate, ``DK`` the deck plate, ``LC`` another, with its outline as the per-part DXF
  measures it (``LK75x27``);
- **bought**: what the BOM buys (:func:`hardware.bom.bought_lines`: a shim stack is bought
  as DIN 433 washers, so its label is theirs), by catalog key, shortened (``M3-BH-8``,
  ``CHI-M3-16``, ``M2.5-NY-S-5``); the servo's own horn ``HORN-<servo>``; the supplies
  (threadlocker, epoxy, filament) are no type (:func:`consumables`).

Every size in a label is tie-stable (:func:`spiderpig.rounding.rounded`): float noise
never decides it. Two types whose labels still agree are told apart by their geometry
(volume, then area): ``a``, ``b``, ..., and their names say what differs (the holes, their
spacing, the outline). Each type's file: ``<label>_<what>.stl`` for a print
(``SP8-0.7_top_spacer.stl``), ``<label>_<part>_x<qty>.dxf`` for a cut. ``part_types``'
order (the guide's parts pages) is the order the steps first need each type; the labels
don't depend on it.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

KINDS = ("printed", "laser", "purchased")

_WHAT = (           # (pattern in a printed body's bare name, what it is, family), first wins
    (r"horn_spacer", "horn spacer", "HS"), (r"spacer_hi", "top spacer", "SP"),
    (r"spacer_lo", "head spacer", "SP"), (r"crank_ring", "rider ring", "RR"),
    (r"_ring\d", "ring", "RG"), (r"spacer", "spacer", "SP"), (r"_sock", "foot sock", "FS"),
    (r"thrust", "thrust sleeve", "TS"), (r"sleeve", "sleeve", "SL"),
    (r"collar", "collar", "CL"), (r"deck_rail", "deck rail", "DR"),
    (r"cradle", "battery cradle", "BC"),
)
_ROUND = {"HS", "SP", "RR", "RG", "CL", "FS"}      # across - high
_LONG = {"SL", "TS"}                               # across x long
_LASER = ((r"b\d+(_leg\d+)?$", "LK", "link"), (r"(frame_outer|torso)$", "FR", "frame plate"),
          (r"crank_plate\d+$", "CW", "crank web"), (r"centre_plate\d+$", "CP", "centre plate"),
          (r"deck_plate$", "DK", "deck plate"))
_PHRASES = (("heat_set_insert", "ins"), ("self_tap", "st"), ("set_screw", "set"),
            ("pillar_shaft", "shaft"), ("servo_driver", "drv"), ("shim_din988", "shim"))
_TOKENS = {"bhcs": "bh", "washer": "w", "chicago": "chi", "nut": "n", "standoff": "so",
           "round": "r", "hex": "hx", "nylon": "ny", "screw": "s", "m25": "m2.5"}
CONSUMABLE = re.compile(r"threadlocker|epoxy|glue|cement|filament")
"""Shop supplies, not parts: a step's sentence names them, the cover lists them."""


@dataclass
class PartType:
    label: str
    kind: str                       # "printed", "laser" or "purchased"
    name: str                       # what it is, with its size
    ref: str                        # a body of the type (its pattern, its thumbnail)
    names: list[str]
    file: str | None = None         # a printed type's print STL
    detail: str = ""                # the bag label's size line
    filament: str | None = None     # a printed type's (catalog key)
    key: str | None = None          # a bought type's catalog key
    mirrored: bool = False          # a printed mirror image (printed from ``_mirrored``)
    extra: dict = field(default_factory=dict)

    count: float | None = None      # a bought type's quantity (the BOM's lines)

    @property
    def qty(self) -> int:
        return round(self.count) if self.count is not None else len(self.names)


def _bare(name: str) -> str:
    return re.sub(r"^[LR]\.", "", name)


def what(name: str) -> tuple[str, str]:
    """A printed body's kind and family code, from its name: ("top spacer", "SP")."""
    n = _bare(name)
    return next(((w, f) for pat, w, f in _WHAT if re.search(pat, n)), ("part", "PT"))


def _n(v: float) -> str:
    """A size in a label: 0.1 mm, tie-stable, no trailing ``.0`` (``8``, ``9.3``)."""
    from spiderpig.rounding import rounded

    return f"{rounded(v, 1):g}"


def _h(v: float) -> str:
    """A height in a label: 0.1 mm, tie-stable, always one decimal (``0.7``, ``3.0``)."""
    from spiderpig.rounding import fixed

    return fixed(v, 1)


def _half(v: float) -> str:
    """A laser outline's side: the nearest 0.5 mm, tie-stable (``96.5``, ``12``)."""
    from spiderpig.rounding import rounded

    return f"{rounded(2 * rounded(v, 3), 0) / 2:g}"


def _size(part) -> tuple[float, float, float]:
    bb = part.bounding_box()
    return float(bb.size.X), float(bb.size.Y), float(bb.size.Z)


def _slug(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9.]+", "_", text).strip("_")


def bought_label(key: str) -> str:
    """A catalog key, shortened: ``m3_bhcs_8`` -> ``M3-BH-8``."""
    s = key
    for a, b in _PHRASES:
        s = s.replace(a, b)
    return "-".join(_TOKENS.get(t, t) for t in s.split("_")).upper()


def _printed_label(fam: str, x: float, y: float, z: float) -> str:
    across = max(x, y)
    if fam in _ROUND:
        return f"{fam}{_n(across)}-{_h(z)}"
    if fam in _LONG:
        return f"{fam}{_n(across)}x{_h(z)}"
    dims = sorted((x, y, z), reverse=True)
    return f"{fam}{_half(dims[0])}x{_half(dims[1])}x{_half(dims[2])}"


def part_types(mech, order: list[str] | None = None, groups: dict | None = None,
               filament: str | None = None) -> list[PartType]:
    """Every part type of ``mech``, labelled, in the order ``order`` (the body names as the
    assembly adds them; else the mechanism's) first needs each, the kinds in turn.
    ``groups``: the made groups by method (``"printed"``, ``"laser"``) when the caller has
    them."""
    from spiderpig.hardware import catalog
    from spiderpig.hardware.bom import _filament_name, _split_by, group_made, printed_filaments
    from spiderpig.layout import outline_size

    groups = dict(groups or {})
    for method in ("printed", "laser"):
        if method not in groups:
            groups[method] = group_made(mech.bodies, method)
    filament = filament or mech.meta.get("filament", "pla_filament")
    fil_of = printed_filaments(mech, filament)
    by_name = {b.name: b for b in mech.bodies}
    found: list[PartType] = []
    for g0 in groups["printed"]:
        for g in _split_by(g0, fil_of, by_name):
            plain = [n for n in g.names if n not in g.mirrored]
            ref = g.ref if g.ref.name in plain else by_name[plain[0]]
            x, y, z = _size(ref.part)
            kind, fam = what(ref.name)
            fil = fil_of.get(plain[0])
            label = _printed_label(fam, x, y, z)
            name = f"printed {kind}, {_n(max(x, y))} mm across, {_h(z)} mm high"
            if fil and fil != filament:
                short = _filament_name(fil).split()[0].upper()
                label += f"-{short}"
                name += f", {_filament_name(fil)}"
            t = PartType(label, "printed", name, ref.name, plain, filament=fil,
                         extra={"what": kind, "geo": _geo(ref.part), "body": ref})
            found.append(t)
            if g.mirrored:
                found.append(PartType(label + "M", "printed", name + ", mirror image",
                                      g.mirrored[0], list(g.mirrored), filament=fil,
                                      detail="MIRRORED", mirrored=True,
                                      extra={"what": kind, "geo": t.extra["geo"],
                                             "twin": t}))
    default_sheet = mech.meta.get("sheet") or "acrylic_3mm"
    for g in groups["laser"]:
        ref = g.ref
        sheet = ref.sheet or default_sheet
        w, h = outline_size(ref, default_sheet)
        t_mm = catalog.sheet_thickness(sheet)
        fam, role = next(((f, r) for pat, f, r in _LASER if re.match(pat, _bare(ref.name))),
                         ("LC", "laser-cut part"))
        found.append(PartType(f"{fam}{_half(w)}x{_half(h)}", "laser",
                              f"{role}, laser-cut, {_half(w)} x {_half(h)} mm, "
                              f"{catalog.sheet_name(sheet).split(',')[0]}", ref.name,
                              list(g.names),
                              extra={"geo": _geo(ref.part), "role": role, "body": ref,
                                     "plate_mm2": float(ref.part.volume) / t_mm}))
    found += _bought_types(mech)
    _disambiguate(found)
    for t in found:
        if t.kind == "printed" and not t.mirrored:
            t.file = f"{t.label}_{_slug(t.extra['what'])}.stl"
    for t in found:
        if t.mirrored:
            t.file = t.extra["twin"].file.removesuffix(".stl") + "_mirrored.stl"
    rank = {n: i for i, n in enumerate(order or [b.name for b in mech.bodies])}
    big = len(rank)
    return sorted(found, key=lambda t: (KINDS.index(t.kind),
                                        min((rank.get(n, big) for n in t.names), default=big),
                                        t.label))


def _geo(part) -> tuple[float, float]:
    """What tells two parts of one label apart: volume, then area (0.001 mm)."""
    return (round(float(part.volume), 3), round(float(part.area), 3))


def consumables(mech) -> dict[str, float]:
    """The shop supplies the BOM lists (threadlocker, epoxy; filament is the prints'), by
    catalog key: no step counts them, the guide's cover names them."""
    from spiderpig.hardware.bom import bought_lines

    out: dict[str, float] = {}
    for line in bought_lines(mech):
        if CONSUMABLE.search(line.key):
            out[line.key] = out.get(line.key, 0.0) + line.qty
    return out


def bought_by_body(mech) -> tuple[dict[str, list[tuple[str, float, str]]], dict[str, float]]:
    """What the BOM buys for each body, ``body -> [(key, qty, where)]`` (a shim stack: its
    washers), and what it buys for no body, ``key -> qty`` (a harness's resistors: the
    steps that name the key in their ``extras`` list it); supplies left out."""
    from spiderpig.hardware.bom import bought_lines, line_body

    names = {b.name for b in mech.bodies}
    per: dict[str, list[tuple[str, float, str]]] = {}
    loose: dict[str, float] = {}
    for line in bought_lines(mech):
        if CONSUMABLE.search(line.key):
            continue
        body = line_body(line, names)
        if body is None:
            loose[line.key] = loose.get(line.key, 0.0) + line.qty
        else:
            per.setdefault(body, []).append((line.key, line.qty, line.where))
    return per, loose


def _bought_types(mech) -> list[PartType]:
    """The bought types: each catalog key the BOM buys (its quantity the BOM's), and a
    bought body with no catalog item (the servo's stock horn)."""
    per, loose = bought_by_body(mech)
    out: dict[str, PartType] = {}
    for body, lines in per.items():
        for key, qty, _ in lines:
            if key not in out:
                out[key] = PartType(bought_label(key), "purchased", _catalog_name(key), body,
                                    [], key=key, count=0.0)
            t = out[key]
            t.count = (t.count or 0.0) + qty
            if body not in t.names:
                t.names.append(body)
    for key, qty in loose.items():
        t = out.setdefault(key, PartType(bought_label(key), "purchased", _catalog_name(key),
                                         "", [], key=key, count=0.0))
        t.count = (t.count or 0.0) + qty
    for b in mech.bodies:
        if b.fab == "purchased" and b.part is not None and not b.bom_key:
            key = _bare(b.name)
            if key not in out:
                label, name = _bought(b, key, mech)
                out[key] = PartType(label, "purchased", name, b.name, [])
            out[key].names.append(b.name)
    return list(out.values())


def _catalog_name(key: str) -> str:
    from spiderpig.hardware import catalog

    try:
        return catalog.get(key).name
    except KeyError:
        return key                      # (no catalog item: tests/test_guide.py says so)


def _bought(body, key: str, mech) -> tuple[str, str]:
    """A bought body's label and name: its catalog item's, or (a stock part with none,
    the servo's horn) the servo's."""
    from spiderpig.hardware import catalog

    if body.bom_key:
        try:
            return bought_label(key), catalog.get(key).name
        except KeyError:
            pass
    if "servo_horn" in key:
        from spiderpig import servos

        servo = str(mech.meta.get("servo", ""))
        try:
            horn = servos.get(servo).horn.name
        except (KeyError, ValueError, AttributeError):
            horn = "servo horn"
        return f"HORN-{servo.upper()}", horn if "box" in horn else f"{horn} (in the servo's box)"
    return bought_label(key), key          # (no catalog item: tests/test_guide.py says so)


def _disambiguate(found: list[PartType]) -> None:
    """Labels that agree: ``a``, ``b``, ... by geometry (volume, then area), the mirror
    images following their twins."""
    clash: dict[str, list[PartType]] = {}
    for t in found:
        if not t.mirrored:
            clash.setdefault(t.label, []).append(t)
    for same in clash.values():
        if len(same) < 2:
            continue
        ordered = sorted(same, key=lambda t: (t.extra.get("geo", (0, 0)), t.key or ""))
        for i, t in enumerate(ordered):
            t.label += "abcdefghijklmnopqrstuvwxyz"[i]
        # the names say what tells them apart: the first feature that differs for each
        told = _apart([t.extra["body"] for t in ordered]) if all(
            "body" in t.extra for t in ordered) else [""] * len(ordered)
        for t, why in zip(ordered, told, strict=True):
            if why:         # first, where a bag label's two lines show it
                head, _, rest = t.name.partition(", ")
                t.name = f"{head} ({why})" + (f", {rest}" if rest else "")
    for t in found:
        if t.mirrored:
            t.label = t.extra["twin"].label + "M"


def _features(body) -> list[str]:
    """What can tell a part from one of its size, coarse to fine: its holes, their sizes,
    their spread, its outline's length, its plate."""
    import itertools
    import math

    from spiderpig.layout import section_of
    from spiderpig.rounding import fixed, rounded

    sk = section_of(body)
    holes, outline = [], 0.0
    for f in sk.faces():
        outline += f.outer_wire().length
        for w in f.inner_wires():
            bb = w.bounding_box()
            holes.append(((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2,
                          max(bb.size.X, bb.size.Y)))
    n = len(holes)
    sizes = sorted(f"{rounded(d, 2):g}" for *_, d in holes)     # (4.15: a bonded barrel's)
    span = max((math.dist(a[:2], b[:2]) for a, b in itertools.combinations(holes, 2)),
               default=0.0)
    return [f"{n} hole{'s' if n != 1 else ''}",
            "holes " + ", ".join(sizes) + " mm" if sizes else "no holes",
            f"holes up to {fixed(span, 1)} mm apart" if n > 1 else f"{n} hole",
            f"its outline {fixed(outline, 1)} mm round",
            f"{fixed(float(body.part.area), 1)} mm² of surface"]


def _apart(bodies) -> list[str]:
    """For each of ``bodies`` (of one label), the coarsest description that differs from
    every other's (several joined when one doesn't)."""
    feats = [_features(b) for b in bodies]
    for k in range(len(feats[0])):
        col = [f[k] for f in feats]
        if len(set(col)) == len(col):
            return col
    joined = ["; ".join(f) for f in feats]
    return joined if len(set(joined)) == len(joined) else [
        f"{j}; variant {i + 1}" for i, j in enumerate(joined)]


def print_stems(types: list[PartType]) -> dict[str, str]:
    """Each printed body's print-file stem (its type's file, less ``.stl``; a mirror
    image's is its twin's file, the ``_mirrored`` one written beside it)."""
    return {n: t.file.removesuffix(".stl") for t in types
            if t.kind == "printed" and t.file and not t.mirrored for n in t.names}


def laser_labels(types: list[PartType]) -> dict[str, str]:
    """Each laser-cut body's label (the cut files' names, the sheets' parts list)."""
    return {n: t.label for t in types if t.kind == "laser" for n in t.names}


def by_body(types: list[PartType]) -> dict[str, PartType]:
    """Each body's type."""
    return {n: t for t in types for n in t.names}


def assembly_order(mech, design) -> list[str] | None:
    """The body names in the order the guide's steps add them (None: no steps)."""
    from spiderpig.construction.assembly import assembly_steps, body_of

    try:
        st = assembly_steps(mech, design)
    except ValueError:
        return None
    seen: dict[str, None] = {}
    for s in st:
        for p in s.adds:
            seen.setdefault(body_of(p), None)
    return list(seen)


def step_counts(steps, types: list[PartType], mech) -> list[dict[str, float]]:
    """Each step's parts list, ``label -> qty``: a made body counts once for its type, a
    bought body for what the BOM buys for it (its shim stack's washers), and a step that
    names a key in its ``extras`` takes the BOM's lines of that key that are no body's
    (several steps naming one key share them out evenly: each servo's bus cable in its own
    side's step). Over all steps every type's quantity is its BOM quantity
    (``tests/test_guide.py``)."""
    per, loose = bought_by_body(mech)
    of = by_body(types)
    of_key = {t.key: t for t in types if t.kind == "purchased" and t.key}
    left = dict(loose)
    naming: dict[str, int] = {}
    for st in steps:
        for key in dict.fromkeys(st.extras):
            naming[key] = naming.get(key, 0) + 1
    share = {k: left[k] / n for k, n in naming.items() if left.get(k)}
    out: list[dict[str, float]] = []
    for st in steps:
        counts: dict[str, float] = {}
        for n in st.counted:
            if n in per:
                for key, qty, _ in per[n]:
                    lab = of_key[key].label
                    counts[lab] = counts.get(lab, 0.0) + qty
            elif n in of:
                counts[of[n].label] = counts.get(of[n].label, 0.0) + 1
        for key in dict.fromkeys(st.extras):
            if left.get(key):
                lab = of_key[key].label
                naming[key] -= 1
                take = left.pop(key) if naming[key] == 0 else min(left[key], share[key])
                if key in left:
                    left[key] -= take
                counts[lab] = counts.get(lab, 0.0) + take
        out.append(counts)
    return out


def tag_labels(types: list[PartType], mech) -> dict[str, str]:
    """The label a picture's tag gives each body: its bought line's (a shim stack: the
    washers'), else its type's."""
    per, _ = bought_by_body(mech)
    of_key = {t.key: t for t in types if t.kind == "purchased" and t.key}
    out = {n: t.label for n, t in by_body(types).items()}
    out.update({n: of_key[lines[0][0]].label for n, lines in per.items()})
    return out


def shim_sentences(counted: list[str], mech) -> list[str]:
    """What a step's shim stacks are bought as (:data:`hardware.bom.SHIM_AS`): ``Each
    1.5 mm shim stack is 3 x M3 washer (DIN 433) ...``, once per stack."""
    per, _ = bought_by_body(mech)
    out: list[str] = []
    for n in counted:
        lines = per.get(n, [])
        stack = re.search(r"\(([\d.]+(?: \+ [\d.]+)*) mm\)", " ".join(w for *_, w in lines))
        if not lines or stack is None or len({k for k, *_ in lines}) != 1:
            continue
        total = sum(float(t) for t in stack.group(1).split(" + "))
        qty = sum(q for _, q, _ in lines)
        text = (f"Each {total:g} mm shim stack is {qty:g} x {bought_label(lines[0][0])}, "
                f"stacked: {_catalog_name(lines[0][0])}.")
        if text not in out:
            out.append(text)
    return out
