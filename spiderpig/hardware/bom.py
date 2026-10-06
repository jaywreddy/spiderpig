"""Bill of materials for a fabricated mechanism.

Everything is derived from the mechanism :func:`fabricate.fabricate` returns:

* bodies with ``fab == "purchased"`` count one each of their ``bom_key``;
* bodies with ``fab == "laser"`` are listed as cut parts (with the sheet
  stock they need, from ``mech.meta["sheet"]``);
* bodies with ``fab == "printed"`` are listed with their volume and filament
  (:func:`part_filament`: TPU 95A for the feet's socks, PETG for a printed part pressed
  on (the capped crankpin's sleeve), else ``mech.meta["filament"]`` or the ``filament``
  argument), and add a filament line per filament (grams at 100 % infill from its catalog
  item's density, as a fraction of a spool);
* ``mech.bom_extras`` adds purchases that aren't modelled as bodies
  (glue, threadlocker, a shim stack's other rings, sheet stock); a line whose ``where`` ends
  in ``cut X
  mm`` (the metal pivots' rod, :class:`construction.pivots.common.RodShaft`)
  also goes on the **cut list** (:func:`cut_list`): identical lengths
  grouped, the total, so the buyer knows how many rods to cut them from.

Made parts that are the same shape are one row with a quantity
(:func:`group_made`): a laser-cut plate and its mirror image are the same cut
(flip it over), a printed part and its mirror image are not ("print
mirrored").

Purchased lines are grouped by catalog key and rounded up to whole packs of
the first (preferred) offer. Lines whose preferred offer is the same product
(same vendor and SKU, e.g. one screw assortment for several lengths) are
bought once.

**Shims** are ordered per thickness (:func:`split_shims`, the items of
:mod:`hardware.shims`): a stack's rings from its height, thickest first. Every shim in
the robot is clamped (under a horn screw's head, at a pillar's end, in a frame tie; the
unclamped spacers are printed); the 1.0 and 0.5 mm ones are bought as DIN 433 washers
(:data:`SHIM_AS`: two make 1 mm), the thinner steps as DIN 988 shims.

What the constructions don't say but the parts do (:func:`fitting_lines`): each horn
screw's shims as the stack under its head (e.g. ``1 mm``, from the shim body's height),
threadlocker 222 on the horn screws where they thread into a metal horn (metal to metal
only: none in a plastic horn), and threadlocker 243 (or 263) on each splice's stud of a
spliced pillar (``--pillar standoff_hand``: the splice hand-tightened at 0.4 N·m, a dab of
threadlocker on the stud, metal to metal, kept off the acrylic; the default one-piece
pillars have none).
"""

from __future__ import annotations

import csv
import json
import math
import re
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from spiderpig.hardware.catalog import get, sheet_name
from spiderpig.hardware.mass import filament_density, surface_props, volume_props
from spiderpig.hardware.mass import volume as part_volume


@dataclass(frozen=True)
class BomLine:
    """``qty`` of catalog item ``key``, used at ``where``."""

    key: str
    qty: float
    where: str = ""


@dataclass
class PurchaseRow:
    key: str
    name: str
    category: str
    qty: float
    where: list[str]
    vendor: str
    url: str
    sku: str
    pack_qty: int
    packs: int
    pack_price_usd: float | None
    verified: bool
    alternatives: list[str]
    same_pack_as: str = ""     # bought with another row's pack (same vendor and SKU)

    @property
    def cost_usd(self) -> float | None:
        if self.same_pack_as:
            return 0.0
        return None if self.pack_price_usd is None else self.packs * self.pack_price_usd


@dataclass
class MadeRow:
    name: str
    method: str          # "laser" | "printed"
    material: str
    size_mm: str         # footprint of a laser part / bbox of a printed part
    volume_cm3: float    # one part
    qty: int = 1
    names: list[str] = field(default_factory=list)
    mirrored: int = 0    # of qty, how many are mirror images (printed parts: "print mirrored")


SAW_KERF = 1.0     # mm lost to each cut of rod stock (a hacksaw or a cut-off disc)


@dataclass(frozen=True)
class CutList:
    """Pieces cut from one stock item (a rod): ``pieces`` is ``(length mm, qty)`` longest
    first, ``total_mm`` their sum, ``stock_mm`` one piece of stock."""

    key: str
    name: str
    pieces: tuple[tuple[float, int], ...]
    stock_mm: float

    @property
    def count(self) -> int:
        return sum(q for _, q in self.pieces)

    @property
    def total_mm(self) -> float:
        return sum(L * q for L, q in self.pieces)

    def stock_pieces(self, kerf: float = SAW_KERF) -> int:
        """How many stock pieces the cuts take: first-fit decreasing, ``kerf`` lost per cut
        (a piece no stock length holds counts one stock piece of its own: the constructions
        refuse such a rod, so it doesn't arise)."""
        if self.stock_mm <= 0:
            return self.count
        free: list[float] = []
        for L, q in self.pieces:            # longest first
            for _ in range(q):
                for i, f in enumerate(free):
                    if f + 1e-9 >= L:
                        free[i] = f - L - kerf
                        break
                else:
                    free.append(self.stock_mm - L - kerf)
        return len(free)

    def describe(self) -> str:
        """``24 pieces of 3 mm stainless rod, 100 mm (456 mm in all): 8 x 21.0, ...``"""
        runs = ", ".join(f"{q} x {L:.1f}" for L, q in self.pieces)
        return (f"{self.count} pieces of {self.name} ({self.total_mm:.0f} mm in all, from "
                f"{self.stock_pieces()} x {self.stock_mm:g} mm stock): {runs} mm")


_CUT = re.compile(r"cut ([\d.]+) mm$")


def cut_list(lines: list[BomLine]) -> list[CutList]:
    """The cut lists the ``cut X mm`` lines ask for, one per stock item, by key."""
    by_key: dict[str, dict[float, int]] = {}
    for line in lines:
        m = _CUT.search(line.where)
        if m is None:
            continue
        pieces = by_key.setdefault(line.key, {})
        L = round(float(m.group(1)), 1)
        pieces[L] = pieces.get(L, 0) + 1
    out = []
    for key, pieces in sorted(by_key.items()):
        item = get(key)
        out.append(CutList(key, item.name, tuple(sorted(pieces.items(), reverse=True)),
                           float(item.dims.get("length", 0.0))))
    return out


ON_HAND = ("pla_filament", "petg_filament", "tpu95a_filament", "threadlocker_222",
           "threadlocker_243")
"""Shop supplies taken as on hand (the user's, 2026-10-05): listed, not ordered (ORDER.md)
and not in any total (:attr:`Bom.cost_usd`, verify's cost floor)."""


@dataclass
class Bom:
    purchased: list[PurchaseRow]
    made: list[MadeRow]
    title: str = ""
    notes: list[str] = field(default_factory=list)
    printed_g: float | None = None     # filament for the printed parts at 100 % infill
    filament: str = "PLA"
    cuts: list[CutList] = field(default_factory=list)   # stock cut to length (the rod pins)
    filaments: dict[str, float] = field(default_factory=dict)   # grams per filament (name)

    @property
    def cost_usd(self) -> float:
        """What the purchases cost (the shop supplies on hand, :data:`ON_HAND`, left out)."""
        return sum(r.cost_usd or 0.0 for r in self.purchased if r.key not in ON_HAND)

    @property
    def unpriced(self) -> list[PurchaseRow]:
        return [r for r in self.purchased if r.cost_usd is None and r.key not in ON_HAND]

    # -- writers ----------------------------------------------------------

    def write(self, out_dir: Path, stem: str = "bom") -> list[Path]:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        paths = [out_dir / f"{stem}.csv", out_dir / f"{stem}.md", out_dir / f"{stem}.json"]
        self.write_csv(paths[0])
        paths[1].write_text(self.markdown())
        paths[2].write_text(json.dumps(self.as_dict(), indent=1))
        return paths

    def write_csv(self, path: Path) -> None:
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["section", "item", "qty", "packs", "pack_qty", "vendor", "sku",
                        "est_cost_usd", "url", "link_verified", "used_at"])
            for r in self.purchased:
                packs = 0 if r.same_pack_as else r.packs
                cost = "" if r.cost_usd is None or r.key in ON_HAND else f"{r.cost_usd:.2f}"
                where = "; ".join(r.where)
                if r.same_pack_as:
                    where = f"(in the same pack as {r.same_pack_as}) {where}"
                w.writerow(["on hand" if r.key in ON_HAND else "buy", r.name, _num(r.qty),
                            packs, r.pack_qty, r.vendor, r.sku,
                            cost, r.url, "yes" if r.verified else "no", where])
            for m in self.made:
                note = f"{m.mirrored} mirrored" if m.mirrored else ""
                w.writerow([m.method, m.name, m.qty, "", "", "", m.material, "", "", note,
                            f"{m.size_mm}; {', '.join(m.names or [m.name])}"])
            for c in self.cuts:
                for L, q in c.pieces:
                    w.writerow(["cut", c.name, q, "", "", "", "", "", "", f"{L:.1f} mm",
                                f"from {c.stock_mm:g} mm stock; deburr"])

    def markdown(self) -> str:
        lines = [f"# Bill of materials{': ' + self.title if self.title else ''}", ""]
        lines += ["## Buy", "",
                  "| qty | item | buy | packs | est. cost | used at |",
                  "|---:|---|---|---:|---:|---|"]
        for r in self.purchased:
            link = f"[{r.vendor}{' ' + r.sku if r.sku else ''}]({r.url})" if r.url else r.vendor
            if not r.verified and r.url:
                link += " (unverified link)"
            cost = "" if r.cost_usd is None else f"${r.cost_usd:.2f}"
            packs = f"{r.packs} × {r.pack_qty}"
            if r.same_pack_as:
                packs, cost = f"with {r.same_pack_as}", "–"
            elif r.key in ON_HAND:
                cost = f"on hand ({cost})" if cost else "on hand"
            where = ", ".join(sorted(set(r.where)))[:120]
            lines.append(f"| {_num(r.qty)} | {r.name} | {link} | {packs} | {cost} | {where} |")
        lines += ["", f"Estimated purchase total: **${self.cost_usd:.2f}** "
                  "(pack prices at the listed vendor; excludes shipping and the shop "
                  "supplies on hand)."]
        if self.unpriced:
            lines.append(f"{len(self.unpriced)} item(s) have no listed price and are not in "
                         "the total: " + ", ".join(r.name for r in self.unpriced) + ".")
        lines.append("")
        for method, title in (("laser", "Laser-cut"), ("printed", "3D printed")):
            rows = [m for m in self.made if m.method == method]
            if not rows:
                continue
            count = sum(m.qty for m in rows)
            lines += [f"## {title} ({count} parts, {len(rows)} different)", "",
                      "| qty | part | material | size | same part |", "|---:|---|---|---|---|"]
            for m in rows:
                qty = str(m.qty)
                if m.mirrored:
                    qty = f"{m.qty - m.mirrored} + {m.mirrored} mirrored"
                others = ", ".join(n for n in m.names if n != m.name)
                lines.append(f"| {qty} | {m.name} | {m.material} | {m.size_mm} | {others} |")
            if method == "printed":
                grams = self.printed_g
                if grams is None:
                    grams = sum(m.volume_cm3 * m.qty for m in rows) * filament_density(None)
                if len(self.filaments) > 1:
                    each = ", ".join(f"{g:.0f} g of {n}" for n, g in self.filaments.items())
                    lines += ["", f"About {grams:.0f} g at 100% infill: {each}."]
                else:
                    lines += ["", f"About {grams:.0f} g of {self.filament} at 100% infill."]
            lines.append("")
        if self.cuts:
            lines += ["## Cut to length", ""]
            for c in self.cuts:
                lines += [f"* {c.describe()}; deburr every cut end."]
            lines.append("")
        if self.notes:
            lines += ["## Notes", ""] + [f"* {n}" for n in self.notes] + [""]
        return "\n".join(lines)

    def as_dict(self) -> dict:
        return {
            "title": self.title,
            # a shop supply on hand (ON_HAND) costs this build nothing: listed, its pack
            # price kept, ``on_hand`` set
            "purchased": [dict(r.__dict__, on_hand=r.key in ON_HAND,
                               cost_usd=0.0 if r.key in ON_HAND else r.cost_usd)
                          for r in self.purchased],
            "made": [m.__dict__ for m in self.made],
            "cost_usd": self.cost_usd,
            "printed_g": self.printed_g,
            "filaments": {n: round(g, 1) for n, g in self.filaments.items()},
            "notes": self.notes,
            "cuts": [{"key": c.key, "name": c.name, "stock_mm": c.stock_mm,
                      "pieces": [list(p) for p in c.pieces], "count": c.count,
                      "total_mm": round(c.total_mm, 1)} for c in self.cuts],
        }


def _num(q: float) -> str:
    return str(int(q)) if float(q).is_integer() else f"{q:g}"


def _footprint(part) -> str:
    bb = part.bounding_box()
    return f"{bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.1f} mm"


# ---------------------------------------------------------------------------
# Identical made parts
# ---------------------------------------------------------------------------


def _frame(part, vol=None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(centroid, principal axes as columns, principal moments), moments ascending.

    ``vol``: the part's volume properties (``GProp_GProps``) when the caller has them:
    build123d's ``center(CenterOf.MASS)`` and ``principal_properties`` each integrate
    them again, to the same numbers."""
    if vol is None:
        vol = volume_props(part)
    c = vol.CentreOfMass()
    pp = vol.PrincipalProperties()
    moments = pp.Moments()
    props = sorted(zip((pp.FirstAxisOfInertia(), pp.SecondAxisOfInertia(),
                        pp.ThirdAxisOfInertia()), moments, strict=True), key=lambda am: am[1])
    axes = np.array([[a.X(), a.Y(), a.Z()] for a, _ in props]).T
    return np.array([c.X(), c.Y(), c.Z()]), axes, np.array([m for _, m in props])


@dataclass(frozen=True)
class _Sig:
    """What :func:`congruent` compares before it moves a part: the invariants a rigid motion
    keeps (volume, area, the principal moments), and what it maps (the centroid and axes,
    the surface centroid). Measured once per part (:func:`group_made` keeps them)."""

    volume: float
    area: float
    frame: tuple[np.ndarray, np.ndarray, np.ndarray]
    surf: np.ndarray             # the surface's centroid


def _sig(part) -> _Sig:
    """One volume and one surface integration (the frame and the area from them, as
    build123d's ``center``, ``principal_properties`` and ``area`` compute them), shared with
    the part's other consumers (:func:`hardware.mass.volume_props`); the volume is
    build123d's (a compound's is the sum of its solids': :func:`hardware.mass.volume`)."""
    vol, surf = volume_props(part), surface_props(part)
    sc = surf.CentreOfMass()
    return _Sig(part_volume(part), surf.Mass(), _frame(part, vol),
                np.array([sc.X(), sc.Y(), sc.Z()]))


_SIGNS = ((1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1),
          (-1, -1, -1), (-1, 1, 1), (1, -1, 1), (1, 1, -1))
_CENTROID_TOL = 1e-3     # mm the mapped surface centroid may miss by before a boolean is run


def _shared_volume(a, b) -> float:
    """The volume ``a`` and ``b`` have in common (one boolean, the parts left as they are:
    OCCT otherwise widens the arguments' tolerances in place, and what a part is later
    meshed as would depend on how often it was compared)."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.TopTools import TopTools_ListOfShape

    args, tools = TopTools_ListOfShape(), TopTools_ListOfShape()
    args.Append(a.wrapped)
    tools.Append(b.wrapped)
    op = BRepAlgoAPI_Common()
    op.SetArguments(args)
    op.SetTools(tools)
    op.SetNonDestructive(True)
    op.SetRunParallel(True)
    op.Build()
    if not op.IsDone():
        return 0.0
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(op.Shape(), props)
    return float(props.Mass())


def _proper_fit(a, sa: _Sig, b, sb: _Sig, tol: float) -> bool:
    """Is ``b`` the image of ``a`` under a rotation + translation (principal frames matched)?

    Each matching of the frames (a sign per axis, proper rotations only) is tried in
    turn; the motion must take ``a``'s surface centroid onto ``b``'s before the proof,
    one boolean: the volume ``a`` moved and ``b`` don't share is less than ``tol``.
    """
    from build123d import Location, Plane

    from spiderpig.shapes import moved as _moved

    ca, ea, _ = sa.frame
    cb, eb, _ = sb.frame
    for signs in _SIGNS:
        r = eb @ np.diag(signs) @ ea.T
        if np.linalg.det(r) < 0:
            continue
        t = cb - r @ ca
        if np.abs(r @ sa.surf + t - sb.surf).max() > _CENTROID_TOL:
            continue
        moved = _moved(a, Location(Plane(tuple(t), tuple(r[:, 0]), tuple(r[:, 2]))))
        if sa.volume + sb.volume - 2.0 * _shared_volume(moved, b) < tol:
            return True
    return False


def congruent(a, b, rel: float = 1e-4, *, sa: _Sig | None = None, sb: _Sig | None = None,
              mirror: Callable[[], tuple] | None = None) -> str | None:
    """``"same"`` if ``b`` is ``a`` moved, ``"mirror"`` if it is ``a``'s mirror image, else None.

    ``sa`` / ``sb`` are the parts' measurements when the caller has them; ``mirror()``
    gives ``a``'s mirror image with its measurements (:func:`group_made` keeps both per
    group, so a reference is measured and mirrored once).
    """
    from build123d import Plane

    sa = sa or _sig(a)
    sb = sb or _sig(b)
    va, vb = sa.volume, sb.volume
    if abs(va - vb) > rel * max(va, vb) or abs(sa.area - sb.area) > rel * max(sa.area, sb.area):
        return None
    fa, fb = sa.frame, sb.frame
    if not np.allclose(fa[2], fb[2], rtol=1e-3, atol=1e-6 * max(fa[2].max(), 1.0)):
        return None
    tol = max(1e-3, 1e-4 * vb)
    if _proper_fit(a, sa, b, sb, tol):
        return "same"
    if mirror is None:
        m = a.mirror(Plane.XY)
        mirrored = (m, _sig(m))
    else:
        mirrored = mirror()
    if _proper_fit(mirrored[0], mirrored[1], b, sb, tol):
        return "mirror"
    return None


@dataclass
class MadeGroup:
    """Made parts of one shape: ``ref`` is the body whose part is the pattern."""

    method: str
    ref: object                                  # mechanism.Body
    names: list[str]
    mirrored: list[str] = field(default_factory=list)

    @property
    def qty(self) -> int:
        return len(self.names)


def _twin(name: str) -> str | None:
    """``"R.x"`` -> ``"L.x"``: the left-side part a robot's right-side part mirrors
    (:func:`construction.robot.assemble_robot` mirrors one side about z = 0)."""
    return "L." + name[2:] if len(name) > 2 and name.startswith("R.") else None


def _mirrors(sa: _Sig, sb: _Sig, rel: float = 1e-9) -> bool:
    """Do ``sb``'s measurements mirror ``sa``'s about z = 0 (the twins of an assembly that
    mirrors exactly; anything else, an edited part, fails and is compared as usual)?"""
    def close(x, y, scale):
        return bool(np.all(np.abs(np.asarray(x) - np.asarray(y)) <= rel * max(scale, 1.0)))

    flip = np.array([1.0, 1.0, -1.0])
    size = max(abs(sa.volume) ** (1 / 3), 1.0)
    return (close(sa.volume, sb.volume, abs(sa.volume)) and close(sa.area, sb.area, sa.area)
            and close(sa.frame[2], sb.frame[2], float(np.abs(sa.frame[2]).max()))
            and close(sa.frame[0] * flip, sb.frame[0], size * 1e3)
            and close(sa.surf * flip, sb.surf, size * 1e3))


def group_made(bodies, method: str) -> list[MadeGroup]:
    """Group ``method`` bodies by shape. For laser parts a mirror image is the same cut.

    A robot's right-side part whose measurements mirror its left twin's (the assembly
    mirrors one side, so they always do unless a part was edited) joins the twin's group
    without a comparison: the right part is congruent to the group's reference when the
    twin is its mirror image, or the reference is its own mirror image (one comparison per
    printed group, made once); else it is a mirror image (what :func:`congruent` finds).
    """
    from build123d import Plane

    groups: list[MadeGroup] = []
    sigs: dict[int, _Sig] = {}               # id(group) -> the reference's measurements
    mirrors: dict[int, tuple] = {}           # id(group) -> (its mirror image, measurements)
    achiral: dict[int, bool] = {}            # id(group) -> is the reference its own mirror
    placed: dict[str, tuple[MadeGroup, str, _Sig]] = {}   # name -> (group, relation, sig)

    def mirror_of(g: MadeGroup) -> Callable[[], tuple]:
        def get() -> tuple:
            if id(g) not in mirrors:
                m = g.ref.part.mirror(Plane.XY)
                mirrors[id(g)] = (m, _sig(m))
            return mirrors[id(g)]
        return get

    def is_achiral(g: MadeGroup, sb: _Sig) -> bool:
        if id(g) not in achiral:
            m, sm = mirror_of(g)()
            sg = sigs[id(g)]
            achiral[id(g)] = _proper_fit(m, sm, g.ref.part, sg, max(1e-3, 1e-4 * sb.volume))
        return achiral[id(g)]

    for b in bodies:
        if b.fab != method or b.part is None:
            continue
        sb = _sig(b.part)
        twin = placed.get(_twin(b.name) or "")
        if twin is not None and getattr(twin[0].ref, "sheet", None) != getattr(b, "sheet", None):
            twin = None
        if twin is not None and _mirrors(twin[2], sb):
            g, rel_twin, _ = twin
            # mirror(twin) is congruent to the reference iff the twin is the reference's
            # mirror image, or the reference is achiral
            rel = "same" if rel_twin == "mirror" or (method == "printed"
                                                     and is_achiral(g, sb)) else "mirror"
            g.names.append(b.name)
            if rel == "mirror" and method == "printed":
                g.mirrored.append(b.name)
            placed[b.name] = (g, rel, sb)
            continue
        for g in groups:
            if getattr(g.ref, "sheet", None) != getattr(b, "sheet", None):
                continue        # the same shape from another sheet is another part
            rel = congruent(g.ref.part, b.part, sa=sigs[id(g)], sb=sb, mirror=mirror_of(g))
            if rel is None:
                continue
            g.names.append(b.name)
            if rel == "mirror" and method == "printed":
                g.mirrored.append(b.name)
            placed[b.name] = (g, rel, sb)
            break
        else:
            g = MadeGroup(method, b, [b.name])
            groups.append(g)
            sigs[id(g)] = sb
            placed[b.name] = (g, "same", sb)
    return groups


# ---------------------------------------------------------------------------
# Filament per printed part
# ---------------------------------------------------------------------------

TPU_FILAMENT = "tpu95a_filament"
"""The feet's socks (:func:`construction.plates.foot_sock`: TPU 95A, the joinery plan)."""
PRESS_FILAMENT = "petg_filament"
"""A printed part pressed onto metal: the capped crankpin's sleeve, a light press on its
hex standoff (``BoltCrank.capped_press``, the crank note's ``sleeve_press_mm``). PETG
takes a press with less creep and cracking than PLA (``construction.printed`` prints its
snap axles in PETG for the same strain); ``None`` here leaves it in the build's filament."""
FILAMENT_USE = {TPU_FILAMENT: "the feet's TPU socks",
                PRESS_FILAMENT: "printed parts pressed on metal (the capped crankpin sleeves)"}
_SOCK = re.compile(r"_sock$")
_SLEEVE = re.compile(r"crank_pin_sleeve_(.+)$")


def _filament_name(key: str) -> str:
    """``"PLA filament"`` from the catalog item's name (``key`` when it isn't one)."""
    try:
        return get(key).name.split(",")[0]
    except KeyError:
        return key


def pressed_sleeves(meta: dict) -> set[str]:
    """The crankpins (``at`` tags) whose printed sleeve is pressed on its standoff: the
    crank note's chains with ``sleeve_press_mm`` > 0."""
    chains = ((meta or {}).get("crank_bolt") or {}).get("chains") or []
    return {c["at"] for c in chains if (c.get("sleeve_press_mm") or 0) > 0}


def part_filament(body, meta: dict, default: str | None) -> str | None:
    """The filament (catalog key) a printed ``body`` is printed in: TPU 95A for a foot's
    sock, :data:`PRESS_FILAMENT` for a sleeve pressed on its crankpin, else ``default``."""
    if _SOCK.search(body.name):
        return TPU_FILAMENT
    m = _SLEEVE.search(body.name)
    if PRESS_FILAMENT and m and m.group(1) in pressed_sleeves(meta):
        return PRESS_FILAMENT
    return default


def printed_filaments(mech, default: str | None) -> dict[str, str | None]:
    """:func:`part_filament` of every printed body, by name."""
    return {b.name: part_filament(b, mech.meta, default)
            for b in mech.bodies if b.fab == "printed"}


def _split_by(g: MadeGroup, fil_of: dict, by_name: dict) -> list[MadeGroup]:
    """A printed group split where its parts take different filaments (the same shape in
    two filaments is two print jobs); one group, as it was, when they all agree."""
    keys = {fil_of.get(n) for n in g.names}
    if len(keys) <= 1:
        return [g]
    out = []
    for k in sorted(keys, key=str):
        names = [n for n in g.names if fil_of.get(n) == k]
        ref = g.ref if g.ref.name in names else by_name.get(names[0], g.ref)
        out.append(MadeGroup(g.method, ref, names, [n for n in g.mirrored if n in names]))
    return out


# ---------------------------------------------------------------------------
# What the fitting needs (shim stacks, threadlocker)
# ---------------------------------------------------------------------------

_HORN_SHIMS = re.compile(r"crank_horn_shims(\d+)$")
_HORN_SCREW = re.compile(r"crank_horn_screw(\d+)$")
_SPLICE_STUD = re.compile(r"pillar_(.+)_stud(\d+)$")
HORN_LOCK = "threadlocker_222"      # low strength: an M2 / M3 horn screw comes out again
SPLICE_LOCK = "threadlocker_243"    # medium (or 263): the hand-tight splice's retention
LOCK_PER_THREAD = 0.01              # of a 10 ml bottle, a drop per thread


def shim_breakdown(total: float, sizes) -> list[float]:
    """DIN 988 shims making up ``total`` mm (0.1 mm steps), thickest first (the crank's
    ``shim_stack``, on the item's sizes)."""
    left, out = round(total, 3), []
    for s in sorted((float(v) for v in sizes), reverse=True):
        while left >= s - 1e-6:
            out.append(s)
            left = round(left - s, 3)
    return out


SHIM_FAMILIES = ("shim_din988_3x6", "shim_din988_4x8", "shim_din988_6x12")
_GAP_SHIM = re.compile(r"(\d+(?:\.\d+)?) mm in the gap")
_STACK_SHIMS = re.compile(r"\bshims ([\d.]+(?: \+ [\d.]+)*) mm")


SHIM_AS: dict[str, tuple[str, int]] = {
    "shim_din988_3x6_t1": ("m3_washer_433", 2),
    "shim_din988_3x6_t0p5": ("m3_washer_433", 1),
    "shim_din988_4x8_t1": ("m4_washer_433", 2),
    "shim_din988_4x8_t0p5": ("m4_washer_433", 1),
}
"""A thickness bought as stock washers instead: a DIN 433 M3 washer (3.2 x 6 x 0.5, +-0.05)
is 0.5 mm of the same ring for $0.05 where a DIN 988 shim sold singly is $5-13 (Accu,
2026-10-05); two make the 1 mm shim; the M4 one (4.3 x 8 x 0.5) the same for the 4 x 8
family. Clamped shims only (a horn screw's head, a pillar splice or end, a frame tie): the
unclamped ones are printed (construction.pivots.common.gap_washers, the Chicago pins' head
spacers)."""


def shim_key(family: str, t: float) -> str:
    """The catalog item of one thickness of a DIN 988 family: ``shim_din988_4x8_t0p5``."""
    return f"{family}_t{t:g}".replace(".", "p")


def shim_as_bought(family: str, t: float) -> str:
    """One shim of a stack as the BOM orders it: ``two DIN 433 washers`` for a 1 mm M3
    shim (:data:`SHIM_AS`), else ``a 0.2 mm DIN 988 shim``."""
    key = shim_key(family, t)
    if key in SHIM_AS:
        washer, n = SHIM_AS[key]
        return f"{n} x {get(washer).name}"
    return f"a {t:g} mm DIN 988 shim"


def split_shims(lines: list[BomLine], by_name: dict,
                stacks: dict[str, list[float]] | None = None
                ) -> tuple[list[BomLine], list[str]]:
    """Each DIN 988 family's lines as lines per thickness (what can be ordered).

    The constructions model a shim stack as one ring as thick as the stack (its line
    names the body) plus a line for the stack's other rings (``n - 1``, its ``where`` a
    description), or list a single shim with its thickness (``0.3 mm in the gap over
    layer 4``), or name the stack (the horn screws': ``DIN 988 shims 1 + 0.2 mm``). Every
    stack is built thickest first (:func:`shim_breakdown`), so the ring's height gives its
    shims. A family with described rings that no stack accounts for keeps its lines as
    they were, with a note."""
    out: list[BomLine] = []
    notes: list[str] = []
    fam_lines: dict[str, list[BomLine]] = {}
    for line in lines:
        (fam_lines.setdefault(line.key, []) if line.key in SHIM_FAMILIES else out).append(line)
    for fam, fl in fam_lines.items():
        sizes = stack_steps(fam)
        split: list[BomLine] = []
        rest = 0.0           # the stacks' other rings, which the ring bodies account for
        unmatched = False
        others = 0           # rings beyond the first that the ring bodies stand for
        for line in fl:
            body = by_name.get(line.where)
            name = line.where or ""
            told = (stacks or {}).get(name) or (stacks or {}).get(
                name[2:] if name[:2] in ("L.", "R.") else None)
            if body is not None and told:
                # the construction said what it stacked (``Realized.notes["shim_stacks"]``)
                stack = [float(t) for t in told]
                others += max(len(stack) - 1, 0)
                what = " + ".join(f"{t:g}" for t in stack)
                split += [BomLine(shim_key(fam, t), line.qty, f"{line.where} ({what} mm)")
                          for t in stack]
                continue
            if body is not None and body.part is not None:
                bb = body.part.bounding_box()
                total = round(min(bb.size.X, bb.size.Y, bb.size.Z), 2)
                stack = shim_breakdown(total, sizes)
                if abs(total - sum(stack)) > 0.02:      # finer than the steps (a clamp's
                    stack = shim_breakdown(total, get(fam).dims.get("t") or ())  # take-up)
                if abs(total - sum(stack)) > 0.02:     # (heights are 0.1 mm multiples)
                    unmatched = True          # a height no step makes: don't drop it
                others += max(len(stack) - 1, 0)
                what = " + ".join(f"{t:g}" for t in stack)
                split += [BomLine(shim_key(fam, t), line.qty, f"{line.where} ({what} mm)")
                          for t in stack]
            elif (m := _STACK_SHIMS.search(line.where or "")):
                stack = m.group(1).split(" + ")
                others += len(stack) - 1
                split += [BomLine(shim_key(fam, float(t)), line.qty, line.where) for t in stack]
            elif (m := _GAP_SHIM.search(line.where or "")):
                split.append(BomLine(shim_key(fam, float(m.group(1))), line.qty, line.where))
            else:
                rest += line.qty
        # ``rest`` may fall short of ``others`` (a construction that doesn't list a stack's
        # other rings: the split counts them); more is shims no stack accounts for
        split = [BomLine(SHIM_AS[x.key][0], x.qty * SHIM_AS[x.key][1], x.where)
                 if x.key in SHIM_AS else x for x in split]
        if unmatched or rest > others + 1e-6 or not all(_known(x.key) for x in split):
            notes.append(f"{get(fam).name}: listed as one line (the thicknesses could not be "
                         f"accounted for: {rest:g} rings described, {others} in the stacks).")
            out += fl
        else:
            out += split
    return out, notes


def stack_steps(family: str) -> tuple[float, ...]:
    """The thicknesses the constructions stack a family's shims from: for the M3 and M4
    families the 1.0 mm shim and the thin step (0.5 mm: a DIN 433 washer,
    :data:`construction.pivots.standoff.SHIM_STEP`), else its catalog ``t``."""
    if family in ("shim_din988_3x6", "shim_din988_4x8"):
        from spiderpig.construction.pivots.standoff import SHIM_STEP

        return (1.0, SHIM_STEP)
    return tuple(get(family).dims.get("t") or ())


def _known(key: str) -> bool:
    try:
        get(key)
    except KeyError:
        return False
    return True


def _metal_horn(meta: dict) -> bool | None:
    """Is the build's servo horn metal (the horn screws thread metal into metal)? Its name
    says so (the STS3215's aluminium disc), or its holes are machine threads (tapped in
    metal: the XL430's HN11-N101); a self-tapping pattern is a plastic horn (the XL330's).
    ``None``: no servo named."""
    key = (meta or {}).get("servo")
    if not key:
        return None
    try:
        from spiderpig import servos

        horn = servos.get(key).horn
    except (KeyError, AttributeError, ValueError):
        return None
    name = horn.name.lower()
    if any(m in name for m in ("alumin", "steel", "brass", "metal")):
        return True
    return not getattr(horn.pattern, "tapping", True)


def _side(name: str) -> str:
    return name[:2] if name[:2] in ("L.", "R.") else ""


def fitting_lines(mech) -> tuple[list[BomLine], list[str], set[str]]:
    """What the fitting needs that the constructions don't list per part: ``(lines, notes,
    bodies whose own line these replace)``.

    * each horn screw's shims (``crank_horn_shims<i>``, a stack modelled as one ring):
      the ring's line says the stack, e.g. ``horn screw 0: DIN 988 shims 0.5 + 0.2 mm``
      (its height is the stack, :func:`shim_breakdown`; the crank lists the stack's other
      rings itself), and a note lists every screw's;
    * threadlocker 222 on each horn screw when the horn is metal (aluminium: the STS3215's
      stock horn), a drop each; none in a plastic horn (metal to metal only);
    * threadlocker 243 on each pillar splice's stud (``pillar_<joint>_stud<k>``), a dab
      each, metal to metal, kept off the acrylic (the user's decision of 2026-10-05; 263
      holds as well), unless the pillar construction already listed them (its
      ``splice_lock_key`` line in ``mech.bom_extras``); the note is written either way.
    """
    lines: list[BomLine] = []
    notes: list[str] = []
    replaced: set[str] = set()
    stacks: dict[str, list[str]] = {}
    horn_screws: list[str] = []
    studs: list[str] = []
    for b in mech.bodies:
        if b.fab != "purchased" or not b.bom_key:
            continue
        if (m := _HORN_SHIMS.search(b.name)) and b.part is not None:
            try:
                sizes = get(b.bom_key).dims.get("t")
            except KeyError:
                sizes = None
            if not sizes:
                continue
            bb = b.part.bounding_box()
            total = round(min(bb.size.X, bb.size.Y, bb.size.Z), 1)
            stack = shim_breakdown(total, sizes)
            what = " + ".join(f"{s:g}" for s in stack)
            where = (f"{b.name}: horn screw {m.group(1)}, shims {what} mm "
                     f"({total:g} mm) under its head")
            lines.append(BomLine(b.bom_key, 1, where))
            replaced.add(b.name)
            bought = " + ".join(shim_as_bought(b.bom_key, s) for s in stack)
            stacks.setdefault(f"{what} mm ({bought})", []).append(
                f"{_side(b.name)}{m.group(1)}")
        elif _HORN_SCREW.search(b.name):
            horn_screws.append(b.name)
        elif _SPLICE_STUD.search(b.name):
            studs.append(b.name)
    if stacks:
        notes.append("Horn screw shims (under each head): " + "; ".join(
            f"screws {', '.join(s)}: {k}" for k, s in stacks.items()) + ".")
    metal = _metal_horn(mech.meta)
    if horn_screws and metal:
        for n in horn_screws:
            lines.append(BomLine(HORN_LOCK, LOCK_PER_THREAD,
                                 f"{n}: into the metal horn (a drop, metal to metal)"))
        notes.append(f"Horn screws: a drop of low-strength threadlocker (Loctite 222) each "
                     f"({len(horn_screws)}), steel into the metal horn; none in a plastic horn.")
    # the pillar construction lists its splice studs' threadlocker itself
    # (StandoffAxle.splice_lock_key, a line per pillar in bom_extras): don't count them twice
    listed = any("splice stud" in (x.where or "") for x in getattr(mech, "bom_extras", ()))
    for n in ([] if listed else studs):
        lines.append(BomLine(SPLICE_LOCK, LOCK_PER_THREAD,
                             f"{n}: splice stud (243 or 263; metal to metal, off the acrylic)"))
    gaps = {name: n.get("bond_gap_mm", 0.0)
            for name, n in ((mech.meta or {}).get("chicago") or {}).items() if n.get("bond_gap_mm")}
    if gaps:
        notes.append("Chicago pins bonded off the barrel head (no printed spacer under it, "
                     "thinner than a print): " + ", ".join(
                         f"{k} {v:g} mm" for k, v in sorted(gaps.items()))
                     + "; set the gap with a feeler gauge while the epoxy cures.")
    if studs:
        notes.append(f"Pillar splices ({len(studs)}): a dab of medium threadlocker "
                     "(Loctite 243, or 263) on each splice stud for retention, metal to "
                     "metal only: keep it off the acrylic.")
    return lines, notes, replaced


# ---------------------------------------------------------------------------
# The BOM
# ---------------------------------------------------------------------------


def bom_from_mechanism(mech, title: str = "", filament: str | None = None,
                       group: bool = True,
                       groups: dict[str, list[MadeGroup]] | None = None) -> Bom:
    """Collect purchases and made parts from a fabricated mechanism.

    ``group=False`` lists every made part on its own row (faster: no shape
    comparison); ``groups`` passes :func:`group_made` results already computed
    (method -> groups).
    """
    lines: list[BomLine] = []
    made: list[MadeRow] = []
    notes: list[str] = []
    sheet = mech.meta.get("sheet_name", "sheet")
    filament = filament or mech.meta.get("filament")
    fil_name = _filament_name(filament) if filament else "PLA/PETG"
    fitted, fit_notes, replaced = fitting_lines(mech)
    for body in mech.bodies:
        if body.fab == "purchased" and body.bom_key and body.name not in replaced:
            lines.append(BomLine(body.bom_key, 1, body.name))
    lines += fitted
    notes += fit_notes
    by_name = {b.name: b for b in mech.bodies}
    fil_of = printed_filaments(mech, filament)
    grams: dict[str | None, float] = {}         # filament key -> grams at 100 % infill
    for method in ("laser", "printed"):
        bodies = [b for b in mech.bodies if b.fab == method and b.part is not None]
        if groups is not None and method in groups:
            found = groups[method]
        elif group:
            found = group_made(bodies, method)
        else:
            found = [MadeGroup(method, b, [b.name]) for b in bodies]
        if method == "printed":
            found = [part for g in found for part in _split_by(g, fil_of, by_name)]
        for g in found:
            fil = fil_of.get(g.ref.name, filament) if method == "printed" else None
            made.append(MadeRow(
                name=g.ref.name, method=method,
                material=(sheet_name(g.ref.sheet) if getattr(g.ref, "sheet", None) else sheet)
                if method == "laser" else (_filament_name(fil) if fil else fil_name),
                size_mm=_footprint(g.ref.part), volume_cm3=part_volume(g.ref.part) / 1000.0,
                qty=g.qty, names=list(g.names), mirrored=len(g.mirrored),
            ))
            if method == "printed":
                grams[fil] = grams.get(fil, 0.0) + (made[-1].volume_cm3 * g.qty
                                                    * filament_density(fil))
    total = sum(grams.values())
    for fil, g in grams.items():
        if not fil or g <= 0:
            continue
        spool = float(get(fil).dims.get("spool_g", 1000.0))
        what = "printed parts" if fil == filament else \
            ", ".join(sorted({FILAMENT_USE.get(fil, "printed parts")}))
        lines.append(BomLine(fil, round(g / spool, 3), f"{what}: {g:.0f} g at 100 % infill"))
    if filament and total > 0:
        each = "; ".join(f"{g:.0f} g of {_filament_name(f)} ({filament_density(f)} g/cm3)"
                         for f, g in grams.items() if f and g > 0)
        notes.append(f"Printed parts need about {total:.0f} g of filament at 100 % infill "
                     f"({each}); less with sparse infill.")
    lines.extend(mech.bom_extras)
    lines, shim_notes = split_shims(lines, by_name, (mech.meta or {}).get("shim_stacks"))
    notes += shim_notes

    grouped: dict[str, PurchaseRow] = {}
    for line in lines:
        item = get(line.key)
        row = grouped.get(line.key)
        if row is None:
            offer = item.offer
            row = grouped[line.key] = PurchaseRow(
                key=item.key, name=item.name, category=item.category, qty=0.0, where=[],
                vendor=offer.vendor if offer else "", url=offer.url if offer else "",
                sku=(offer.sku or "") if offer else "",
                pack_qty=offer.pack_qty if offer else 1, packs=0,
                pack_price_usd=offer.price_usd if offer else None,
                verified=bool(offer and offer.verified),
                alternatives=[o.url for o in item.offers[1:]],
            )
        row.qty += line.qty
        if line.where:
            row.where.append(line.where)
    for cut in cut_list(lines):          # stock cut to length: whole pieces, packed
        if cut.key in grouped:
            grouped[cut.key].qty = float(cut.stock_pieces())
    for row in grouped.values():
        row.qty = round(row.qty, 6)
        offer = get(row.key).offer
        if offer is None:
            row.packs = max(1, math.ceil(row.qty / max(row.pack_qty, 1) - 1e-9))
            continue
        row.packs, usd = offer.buy(row.qty)        # at its price break, where it has them
        if usd is not None:
            row.pack_price_usd = round(usd / row.packs, 4)
    order = ["servo", "horn", "fastener", "insert", "nut", "washer", "standoff", "spacer",
             "bearing", "bushing", "dowel", "clip", "sheet", "filament", "adhesive", "misc"]
    purchased = sorted(
        grouped.values(),
        key=lambda r: (order.index(r.category) if r.category in order else len(order), r.name),
    )
    first: dict[tuple[str, str], PurchaseRow] = {}
    for row in purchased:           # one product (vendor + SKU) covering several rows
        if not row.sku:
            continue
        lead = first.setdefault((row.vendor, row.sku), row)
        if lead is not row:
            row.same_pack_as = lead.name
            lead.packs = max(lead.packs, row.packs)
    made.sort(key=lambda m: (m.method, m.name))
    return Bom(purchased=purchased, made=made, title=title, notes=notes,
               printed_g=total if filament else None,
               filament=fil_name.split()[0] if filament else "PLA", cuts=cut_list(lines),
               filaments={(_filament_name(f) if f else fil_name): g
                          for f, g in grams.items() if g > 0})
