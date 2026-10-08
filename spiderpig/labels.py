"""Part labels: one identifier per part type, shared by ``spiderpig build``'s print files
and the assembly guide (``spiderpig guide``).

``P`` a printed type (a shape group of :func:`hardware.bom.group_made`, split by filament
as the print files are; a printed part's mirror image is another print, ``P07M``), ``C`` a
laser-cut type (a cut: a mirror image is the same cut, the sheet flipped), ``H`` a bought
item (its catalog key). Each kind is numbered in the order the assembly first needs it
(:func:`construction.assembly.assembly_steps`), so the guide's parts pages read in that order; a
design whose steps can't be made (no plan) numbers in body order. The same design always
gets the same labels.

A printed type's print file is ``<label>_<what>_<height>mm.stl`` (``P13_top_spacer_0.7mm``:
the part's height as printed, on its face), its mirror image's the same with
``_mirrored``: the bag label and the print batch say the same thing.
"""

from __future__ import annotations

import re
from collections import Counter
from dataclasses import dataclass, field

PREFIX = {"printed": "P", "laser": "C", "purchased": "H"}

_WHAT = (           # (pattern in a printed body's bare name, what it is), first match wins
    (r"horn_spacer", "horn spacer"), (r"spacer_hi", "top spacer"),
    (r"spacer_lo", "head spacer"), (r"crank_ring", "rider ring"), (r"_ring\d", "ring"),
    (r"spacer", "spacer"), (r"_sock", "foot sock"), (r"thrust", "thrust sleeve"),
    (r"sleeve", "sleeve"), (r"collar", "collar"), (r"deck_rail", "deck rail"),
    (r"cradle", "battery cradle"),
)


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

    @property
    def qty(self) -> int:
        return len(self.names)


def what(name: str) -> str:
    """A printed body's kind, from its name: "top spacer", "ring", "foot sock"."""
    n = re.sub(r"^[LR]\.", "", name)
    return next((w for pat, w in _WHAT if re.search(pat, n)), "part")


def _size(part) -> tuple[float, float, float]:
    bb = part.bounding_box()
    return float(bb.size.X), float(bb.size.Y), float(bb.size.Z)


def _slug(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9.]+", "_", text).strip("_")


def part_types(mech, order: list[str] | None = None, groups: dict | None = None,
               filament: str | None = None) -> list[PartType]:
    """Every part type of ``mech``, labelled, in label order. ``order``: the body names in
    the order the assembly adds them (else the mechanism's order); ``groups``: the made
    groups by method (``"printed"``, ``"laser"``) when the caller has them."""
    from spiderpig.hardware import catalog
    from spiderpig.hardware.bom import _filament_name, _split_by, group_made, printed_filaments

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
            kind = what(ref.name)
            fil = fil_of.get(plain[0])
            name = f"printed {kind}, {max(x, y):.1f} mm across, {z:.1f} mm high"
            if fil and fil != filament:
                name += f", {_filament_name(fil)}"
            t = PartType("", "printed", name, ref.name, plain, filament=fil,
                         detail=f"{z:.1f} mm high, {max(x, y):.1f} mm across",
                         extra={"what": kind, "z": z})
            found.append(t)
            if g.mirrored:
                found.append(PartType("", "printed", name + ", mirror image",
                                      g.mirrored[0], list(g.mirrored), filament=fil,
                                      detail=t.detail + ", MIRRORED", mirrored=True,
                                      extra={"what": kind, "z": z, "twin": t}))
    for g in groups["laser"]:
        x, y, z = _size(g.ref.part)
        sheet = g.ref.sheet or mech.meta.get("sheet") or ""
        found.append(PartType("", "laser", f"laser-cut, {max(x, y):.1f} x {min(x, y):.1f} mm, "
                              f"{z:.2f} mm {catalog.sheet_name(sheet) if sheet else ''}"
                              .rstrip(), g.ref.name, list(g.names),
                              detail=f"{max(x, y):.0f} x {min(x, y):.0f} mm"))
    bought: dict[str, PartType] = {}
    for b in mech.bodies:
        if b.fab != "purchased" or b.part is None:
            continue
        key = b.bom_key or re.sub(r"^[LR]\.", "", b.name)
        if key not in bought:
            try:
                name = catalog.get(key).name
            except KeyError:
                name = key
            bought[key] = PartType("", "purchased", name, b.name, [], key=key)
        bought[key].names.append(b.name)
    found += list(bought.values())
    # numbered by first use
    rank = {n: i for i, n in enumerate(order or [b.name for b in mech.bodies])}
    big = len(rank)

    def first(t: PartType) -> tuple:
        return (min((rank.get(n, big) for n in t.names), default=big), t.ref)

    count: Counter = Counter()
    out: list[PartType] = []
    for t in sorted((t for t in found if not t.mirrored), key=first):
        count[t.kind] += 1
        t.label = f"{PREFIX[t.kind]}{count[t.kind]:02d}"
        out.append(t)
        if t.kind == "printed":
            t.file = f"{t.label}_{_slug(t.extra['what'])}_{t.extra['z']:.1f}mm.stl"
    for t in found:
        if t.mirrored:
            twin = t.extra["twin"]
            t.label = twin.label + "M"
            t.file = twin.file.removesuffix(".stl") + "_mirrored.stl"
            out.insert(out.index(twin) + 1, t)
    return out


def print_stems(types: list[PartType]) -> dict[str, str]:
    """Each printed body's print-file stem (its type's file, less ``.stl``; a mirror
    image's is its twin's file, the ``_mirrored`` one written beside it)."""
    return {n: t.file.removesuffix(".stl") for t in types
            if t.kind == "printed" and t.file and not t.mirrored for n in t.names}


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
