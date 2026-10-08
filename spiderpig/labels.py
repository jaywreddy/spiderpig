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
- **bought**: the catalog key, shortened (``M3-BH-8``, ``CHI-M3-16``); the servo's own horn
  ``HORN-<servo>``.

Two types whose labels still agree are told apart by their geometry (volume, then area):
``a``, ``b``, ... Each type's file: ``<label>_<what>.stl`` for a print
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
           "round": "r", "hex": "hx", "nylon": "ny", "screw": "s"}


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


def _bare(name: str) -> str:
    return re.sub(r"^[LR]\.", "", name)


def what(name: str) -> tuple[str, str]:
    """A printed body's kind and family code, from its name: ("top spacer", "SP")."""
    n = _bare(name)
    return next(((w, f) for pat, w, f in _WHAT if re.search(pat, n)), ("part", "PT"))


def _n(v: float) -> str:
    """A size in a label: 0.1 mm, no trailing ``.0`` on an across (``8``, ``9.3``)."""
    return f"{round(v, 1):g}"


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
        return f"{fam}{_n(across)}-{z:.1f}"
    if fam in _LONG:
        return f"{fam}{_n(across)}x{z:.1f}"
    dims = sorted((x, y, z), reverse=True)
    return f"{fam}{dims[0]:.0f}x{dims[1]:.0f}x{dims[2]:.0f}"


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
            name = f"printed {kind}, {max(x, y):.1f} mm across, {z:.1f} mm high"
            if fil and fil != filament:
                short = _filament_name(fil).split()[0].upper()
                label += f"-{short}"
                name += f", {_filament_name(fil)}"
            t = PartType(label, "printed", name, ref.name, plain, filament=fil,
                         detail=f"{z:.1f} mm high, {max(x, y):.1f} mm across",
                         extra={"what": kind, "geo": _geo(ref.part)})
            found.append(t)
            if g.mirrored:
                found.append(PartType(label + "M", "printed", name + ", mirror image",
                                      g.mirrored[0], list(g.mirrored), filament=fil,
                                      detail=t.detail + ", MIRRORED", mirrored=True,
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
        found.append(PartType(f"{fam}{w:.0f}x{h:.0f}", "laser",
                              f"{role}, laser-cut, {w:.1f} x {h:.1f} mm, "
                              f"{catalog.sheet_name(sheet).split(',')[0]}", ref.name,
                              list(g.names), detail=f"{w:.0f} x {h:.0f} x {t_mm:g} mm",
                              extra={"geo": _geo(ref.part), "role": role,
                                     "plate_mm2": float(ref.part.volume) / t_mm}))
    bought: dict[str, PartType] = {}
    for b in mech.bodies:
        if b.fab != "purchased" or b.part is None:
            continue
        key = b.bom_key or _bare(b.name)
        if key not in bought:
            label, name = _bought(b, key, mech)
            bought[key] = PartType(label, "purchased", name, b.name, [], key=b.bom_key)
        bought[key].names.append(b.name)
    found += list(bought.values())
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
        return f"HORN-{servo.upper()}", f"{horn} (in the servo's box)"
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
        for i, t in enumerate(sorted(same, key=lambda t: (t.extra.get("geo", (0, 0)),
                                                          t.key or ""))):
            t.label += "abcdefghijklmnopqrstuvwxyz"[i]
            # the name says what tells them apart
            if t.kind == "laser":
                t.name += f", {t.extra['plate_mm2']:.0f} mm² of plate"
                t.detail += f", {t.extra['plate_mm2']:.0f} mm²"
            elif "geo" in t.extra:
                t.name += f", {t.extra['geo'][0]:.1f} mm³"
                t.detail += f", {t.extra['geo'][0]:.1f} mm³"
    for t in found:
        if t.mirrored:
            t.label = t.extra["twin"].label + "M"


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
