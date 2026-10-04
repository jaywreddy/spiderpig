"""Bill of materials for a fabricated mechanism.

Everything is derived from the mechanism :func:`fabricate.fabricate` returns:

* bodies with ``fab == "purchased"`` count one each of their ``bom_key``;
* bodies with ``fab == "laser"`` are listed as cut parts (with the sheet
  stock they need, from ``mech.meta["sheet"]``);
* bodies with ``fab == "printed"`` are listed with their volume, and add a
  filament line (``mech.meta["filament"]`` or the ``filament`` argument: grams
  at 100 % infill from the catalog item's density, as a fraction of a spool);
* ``mech.bom_extras`` adds purchases that aren't modelled as bodies
  (washers, glue, shims, sheet stock); a line whose ``where`` ends in ``cut X
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
from spiderpig.hardware.mass import filament_density


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

    def describe(self) -> str:
        """``24 pieces of 3 mm stainless rod, 100 mm (456 mm in all): 8 x 21.0, ...``"""
        runs = ", ".join(f"{q} x {L:.1f}" for L, q in self.pieces)
        return (f"{self.count} pieces of {self.name} ({self.total_mm:.0f} mm in all, from "
                f"{self.stock_mm:g} mm stock): {runs} mm")


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


@dataclass
class Bom:
    purchased: list[PurchaseRow]
    made: list[MadeRow]
    title: str = ""
    notes: list[str] = field(default_factory=list)
    printed_g: float | None = None     # filament for the printed parts at 100 % infill
    filament: str = "PLA"
    cuts: list[CutList] = field(default_factory=list)   # stock cut to length (the rod pins)

    @property
    def cost_usd(self) -> float:
        return sum(r.cost_usd or 0.0 for r in self.purchased)

    @property
    def unpriced(self) -> list[PurchaseRow]:
        return [r for r in self.purchased if r.cost_usd is None]

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
                cost = "" if r.cost_usd is None else f"{r.cost_usd:.2f}"
                where = "; ".join(r.where)
                if r.same_pack_as:
                    where = f"(in the same pack as {r.same_pack_as}) {where}"
                w.writerow(["buy", r.name, _num(r.qty), packs, r.pack_qty, r.vendor, r.sku,
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
            where = ", ".join(sorted(set(r.where)))[:120]
            lines.append(f"| {_num(r.qty)} | {r.name} | {link} | {packs} | {cost} | {where} |")
        lines += ["", f"Estimated purchase total: **${self.cost_usd:.2f}** "
                  "(pack prices at the listed vendor; excludes shipping)."]
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
            "purchased": [dict(r.__dict__, cost_usd=r.cost_usd) for r in self.purchased],
            "made": [m.__dict__ for m in self.made],
            "cost_usd": self.cost_usd,
            "printed_g": self.printed_g,
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
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    if vol is None:
        vol = GProp_GProps()
        BRepGProp.VolumeProperties_s(part.wrapped, vol)
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
    build123d's ``center``, ``principal_properties`` and ``area`` compute them); the volume
    is build123d's (a compound's is the sum of its solids')."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    vol, surf = GProp_GProps(), GProp_GProps()
    BRepGProp.VolumeProperties_s(part.wrapped, vol)
    BRepGProp.SurfaceProperties_s(part.wrapped, surf)
    sc = surf.CentreOfMass()
    return _Sig(part.volume, surf.Mass(), _frame(part, vol), np.array([sc.X(), sc.Y(), sc.Z()]))


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
    density = filament_density(filament)
    fil_name = get(filament).name.split(",")[0] if filament else "PLA/PETG"
    for body in mech.bodies:
        if body.fab == "purchased" and body.bom_key:
            lines.append(BomLine(body.bom_key, 1, body.name))
    for method in ("laser", "printed"):
        bodies = [b for b in mech.bodies if b.fab == method and b.part is not None]
        if groups is not None and method in groups:
            found = groups[method]
        elif group:
            found = group_made(bodies, method)
        else:
            found = [MadeGroup(method, b, [b.name]) for b in bodies]
        for g in found:
            made.append(MadeRow(
                name=g.ref.name, method=method,
                material=(sheet_name(g.ref.sheet) if getattr(g.ref, "sheet", None) else sheet)
                if method == "laser" else fil_name,
                size_mm=_footprint(g.ref.part), volume_cm3=g.ref.part.volume / 1000.0,
                qty=g.qty, names=list(g.names), mirrored=len(g.mirrored),
            ))
    grams = sum(m.volume_cm3 * m.qty for m in made if m.method == "printed") * density
    if filament and grams > 0:
        spool = float(get(filament).dims.get("spool_g", 1000.0))
        lines.append(BomLine(filament, round(grams / spool, 3),
                             f"printed parts: {grams:.0f} g at 100 % infill"))
        notes.append(f"Printed parts need about {grams:.0f} g of filament at 100 % infill "
                     f"({density} g/cm3); less with sparse infill.")
    lines.extend(mech.bom_extras)

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
    for row in grouped.values():
        row.qty = round(row.qty, 6)
        row.packs = max(1, math.ceil(row.qty / max(row.pack_qty, 1) - 1e-9))
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
               printed_g=grams if filament else None,
               filament=fil_name.split()[0] if filament else "PLA", cuts=cut_list(lines))
