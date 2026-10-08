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
  in ``cut X mm`` (stock cut to length) also goes on the **cut list** (:func:`cut_list`):
  identical lengths grouped, the total, so the buyer knows how many to cut them from.

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
unclamped spacers are printed); the 1.0 and 0.5 mm ones are bought as 0.5 mm washers
(:data:`SHIM_AS`: two make 1 mm; DIN 125 for M3, DIN 433 for M4), the thinner steps as
DIN 988 shims.

What the constructions don't say but the parts do (:func:`fitting_lines`): each horn
screw's shims as the stack under its head (e.g. ``1 mm``, from the shim body's height),
and threadlocker 222 on the horn screws where they thread into a metal horn (metal to metal
only: none in a plastic horn).
"""

from __future__ import annotations

import csv
import json
import math
import re
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, cast

import numpy as np

from spiderpig.hardware.catalog import get, sheet_name
from spiderpig.hardware.mass import filament_density, surface_props, volume_props
from spiderpig.hardware.mass import volume as part_volume

if TYPE_CHECKING:
    from build123d import Shape

    from spiderpig.mechanism import Body


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
    sheet: str = ""      # a laser part's sheet item (its key)


@dataclass
class CutRow:
    """One service's cutting of the parts on one of its sheets (the material included):
    ``usd`` its estimate (:func:`cut_estimate`; ``None``: no rates for that sheet)."""

    service: str
    sheet: str
    name: str
    parts: int
    area_cm2: float
    usd: float | None


CUT_SOURCE = ("an area estimate calibrated to SendCutSend's live quotes of 2026-10-08 (the "
              "Strider double's 47 parts, USD 117.14 in all, free US shipping over USD 39: "
              "bom-study/evidence/sheet/prices.json); upload the files for the real quote")
"""Where :func:`cut_estimate`'s rates come from (what the BOM and ORDER.md say of it)."""


def cut_estimate(key: str, parts: list[tuple[float, int]]) -> float | None:
    """What a service charges to cut ``parts`` (``(area cm^2, qty)`` each) from sheet ``key``,
    the material included: each part ``max(cut_min_usd, cut_usd_cm2 x area)`` (the sheet
    item's rates); ``None`` without them. Calibrated on SendCutSend's per-file live quotes of
    2026-10-08 (:data:`CUT_SOURCE`): in 3 mm acrylic a small part is USD 1.26-1.40 whatever
    its size (1.33 at 4 off), the 93 cm^2 deck plate 6.88; in 5052 0.080-0.090 in the
    plates come to about 0.20 per cm^2, in 6061 0.100 in 0.40, a small one at least 2.20.
    On the Strider double: acrylic 49.45 (quoted 49.44), aluminium 68.48 (67.70)."""
    d = get(key).dims
    rate, least = d.get("cut_usd_cm2"), d.get("cut_min_usd")
    if rate is None or least is None:
        return None
    return round(sum(q * max(float(least), float(rate) * a) for a, q in parts), 2)


def cutting_rows(made: list[MadeRow]) -> list[CutRow]:
    """A :class:`CutRow` per service and sheet the laser-cut ``made`` rows are cut from."""
    by: dict[str, list[MadeRow]] = {}
    for m in made:
        if m.method == "laser" and m.sheet and cut_by(m.sheet):
            by.setdefault(m.sheet, []).append(m)
    out = []
    for key, rows in by.items():
        t_cm = float(get(key).dims.get("thickness", 3.0)) / 10.0
        parts = [(m.volume_cm3 / t_cm, m.qty) for m in rows]
        out.append(CutRow(cut_by(key), key, get(key).name, sum(m.qty for m in rows),
                          round(sum(a * q for a, q in parts), 1), cut_estimate(key, parts)))
    return sorted(out, key=lambda c: (c.service, c.sheet))


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
           "threadlocker_243", "m2_self_tap_6")
"""Shop supplies taken as on hand (the user's, 2026-10-05), and the M2 x 6 self-tappers
that come in every STS3215's box (Seeed's ST3215-C001 part list: 2 horns and 18 screws a
servo; Waveshare's package photo shows the pointed self-tappers; BOM study 2026-10-08):
listed, not ordered (ORDER.md) and not in any total (:attr:`Bom.cost_usd`, verify's cost
floor)."""


def cut_by(key: str) -> str:
    """The cutting service that supplies sheet ``key`` with its parts (the sheet item's
    ``service``; ``""``: none, or not a sheet). Such a sheet's row is not bought: the
    service supplies the material with the cut, priced by the BOM's cutting line
    (:class:`CutRow`, :func:`cut_estimate`), so a stock seller's raw sheet price
    (Inventables' acrylic, before 2026-10-08) never counts."""
    try:
        item = get(key)
    except KeyError:
        return ""
    return str(item.dims.get("service") or "") if item.category == "sheet" else ""


def bought(key: str) -> bool:
    """Whether a row of ``key`` is ordered and totalled as a purchase: not on hand
    (:data:`ON_HAND`) and not the raw sheet of a service that supplies it (:func:`cut_by`:
    its cutting line holds the material)."""
    return key not in ON_HAND and not cut_by(key)


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
    cutting: list[CutRow] = field(default_factory=list)   # per service and sheet

    @property
    def purchases_usd(self) -> float:
        """What the purchases cost (the shop supplies on hand, :data:`ON_HAND`, and the raw
        sheets a service supplies, :func:`cut_by`, left out: ORDER.md's carts add up to
        it)."""
        return sum(r.cost_usd or 0.0 for r in self.purchased if bought(r.key))

    @property
    def cutting_usd(self) -> float:
        """The cut parts, material included, at their services' estimates (:class:`CutRow`;
        an unpriced one adds nothing)."""
        return round(sum(c.usd or 0.0 for c in self.cutting), 2)

    @property
    def cost_usd(self) -> float:
        """The build's estimated cost: the purchases and the cutting (shipping apart)."""
        return self.purchases_usd + self.cutting_usd

    @property
    def unpriced(self) -> list[PurchaseRow]:
        return [r for r in self.purchased if r.cost_usd is None and bought(r.key)]

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
                cost = "" if r.cost_usd is None or not bought(r.key) else f"{r.cost_usd:.2f}"
                where = "; ".join(r.where)
                if r.same_pack_as:
                    where = f"(in the same pack as {r.same_pack_as}) {where}"
                section = ("on hand" if r.key in ON_HAND else
                           f"cut by {cut_by(r.key)}" if cut_by(r.key) else "buy")
                w.writerow([section, r.name, _num(r.qty),
                            packs, r.pack_qty, r.vendor, r.sku,
                            cost, r.url, "yes" if r.verified else "no", where])
            for m in self.made:
                note = f"{m.mirrored} mirrored" if m.mirrored else ""
                w.writerow([m.method, m.name, m.qty, "", "", "", m.material, "", "", note,
                            f"{m.size_mm}; {', '.join(m.names or [m.name])}"])
            for c in self.cutting:
                w.writerow([f"cut by {c.service}", c.name, c.parts, "", "", c.service, c.sheet,
                            "" if c.usd is None else f"{c.usd:.2f}", "", "",
                            f"{c.area_cm2:g} cm^2 of parts; {CUT_SOURCE}"])
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
            elif cut_by(r.key):
                cost = f"with the cutting ({cut_by(r.key)})"
            where = ", ".join(sorted(set(r.where)))[:120]
            lines.append(f"| {_num(r.qty)} | {r.name} | {link} | {packs} | {cost} | {where} |")
        if self.cutting:
            lines += ["", "## Cut by a service (the material included)", "",
                      "| service | sheet | parts | area cm² | est. cost |",
                      "|---|---|---:|---:|---:|"]
            lines += [f"| {c.service} | {c.name} | {c.parts} | {c.area_cm2:g} | "
                      f"{'' if c.usd is None else f'${c.usd:.2f}'} |" for c in self.cutting]
            lines += ["", f"The cutting is {CUT_SOURCE}."]
        lines += ["", f"Estimated total: **${self.cost_usd:.2f}**: purchases "
                  f"${self.purchases_usd:.2f} (pack prices at the listed vendor) and cutting "
                  f"${self.cutting_usd:.2f}; excludes shipping and the shop supplies on hand."]
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
            # price kept, ``on_hand`` set; a sheet a service cuts is its upload (``cut_by``)
            "purchased": [dict(r.__dict__, on_hand=r.key in ON_HAND, cut_by=cut_by(r.key),
                               cost_usd=r.cost_usd if bought(r.key) else 0.0)
                          for r in self.purchased],
            "made": [m.__dict__ for m in self.made],
            "cutting": [dict(c.__dict__) for c in self.cutting],
            "purchases_usd": round(self.purchases_usd, 2),
            "cutting_usd": self.cutting_usd,
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


_COINCIDE = 1e-9
"""mm: how far one part's boundary geometry may lie from the other's for :func:`_coincide`
(a part's exact copy, moved: ~1e-14)."""
_GRID = 1e-6     # mm: the faces and edges are paired in the order of their numbers on this grid


def _canon(d) -> tuple[float, float, float]:
    """A direction up to its sign: the first component that isn't ~0 made positive."""
    v = (d.X(), d.Y(), d.Z())
    for c in v:
        if abs(c) > 1e-12:
            return v if c > 0 else (-v[0], -v[1], -v[2])
    return v


def _edge_numbers(e) -> list[float] | None:
    """An edge as numbers that fix it as a point set: a line by its two ends; a circular
    arc by its centre, axis (up to sign), radius, ends and the point halfway along (which
    of the two arcs between those ends); ``None`` for any other curve."""
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.GeomAbs import GeomAbs_Circle, GeomAbs_Line

    c = BRepAdaptor_Curve(e)
    u0, u1 = c.FirstParameter(), c.LastParameter()
    ends = sorted((p.X(), p.Y(), p.Z()) for p in (c.Value(u0), c.Value(u1)))
    kind = c.GetType()
    if kind == GeomAbs_Line:
        return [0.0, *ends[0], *ends[1]]
    if kind == GeomAbs_Circle:
        circ = c.Circle()
        o, m = circ.Location(), c.Value(0.5 * (u0 + u1))
        return [1.0, o.X(), o.Y(), o.Z(), *_canon(circ.Axis().Direction()), circ.Radius(),
                *ends[0], *ends[1], m.X(), m.Y(), m.Z()]
    return None


def _face_numbers(f) -> list[float] | None:
    """A face as numbers that fix it as a point set: its surface (a plane by its normal up
    to sign and offset, a cylinder by its axis line and radius) and every edge bounding it
    (:func:`_edge_numbers`, in a fixed order); ``None`` for any other surface or edge."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
    from OCP.TopAbs import TopAbs_EDGE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS

    s = BRepAdaptor_Surface(f)
    kind = s.GetType()
    if kind == GeomAbs_Plane:
        pl = s.Plane()
        n, p = _canon(pl.Axis().Direction()), pl.Location()
        head = [0.0, *n, n[0] * p.X() + n[1] * p.Y() + n[2] * p.Z()]
    elif kind == GeomAbs_Cylinder:
        cy = s.Cylinder()
        d, p = _canon(cy.Axis().Direction()), cy.Axis().Location()
        along = d[0] * p.X() + d[1] * p.Y() + d[2] * p.Z()
        head = [1.0, *d, p.X() - along * d[0], p.Y() - along * d[1], p.Z() - along * d[2],
                cy.Radius()]
    else:
        return None
    edges = []
    ex = TopExp_Explorer(f, TopAbs_EDGE)
    while ex.More():
        e = _edge_numbers(TopoDS.Edge_s(ex.Current()))
        if e is None:
            return None
        edges.append(e)
        ex.Next()
    edges.sort(key=lambda v: (len(v), [round(x / _GRID) for x in v]))
    return head + [float(len(edges))] + [x for e in edges for x in [float(len(e)), *e]]


def _boundary(part) -> list[list[float]] | None:
    """Every face of ``part`` as :func:`_face_numbers`, in a fixed order (``None``: a face
    or an edge of a kind not read). Read only: the part is left as it is."""
    out = []
    for face in part.faces():
        f = _face_numbers(face.wrapped)
        if f is None:
            return None
        out.append(f)
    out.sort(key=lambda v: (len(v), [round(x / _GRID) for x in v]))
    return out


def _coincide(a: list[list[float]] | None, b: list[list[float]] | None) -> bool:
    """Are two parts' :func:`_boundary` the same, face for face, every number within
    :data:`_COINCIDE`? Then the parts are one solid, and this is a proof, not a sample:

    - a face is the piece of its surface its edges bound, and the numbers fix both: the
      plane or the cylinder (its axis line and radius), and each edge as a point set (a
      line's ends; an arc's circle, ends and middle, which of the two arcs it is), so on a
      plane the loops bound one region and on a cylinder the arcs pick the side;
    - so two parts whose faces pair up so have the same boundary, and a closed solid is
      the region its boundary encloses: they are the same solid, within 1e-9 mm (the
      volume they don't share is under 1e-9 mm times their area).

    Faces and edges are paired in the order of their numbers rounded to :data:`_GRID`:
    numbers that round apart, a surface or curve of another kind, a different count, only
    make this ``False``, and the caller runs the boolean."""
    if a is None or b is None or len(a) != len(b):
        return False
    for fa, fb in zip(a, b, strict=True):
        if len(fa) != len(fb) or max((abs(x - y) for x, y in zip(fa, fb, strict=True)),
                                     default=0.0) > _COINCIDE:
            return False
    return True


def _proper_fit(a, sa: _Sig, b, sb: _Sig, tol: float) -> bool:
    """Is ``b`` the image of ``a`` under a rotation + translation (principal frames matched)?

    A pure translation first when the world-frame inertia tensors agree (a part with two
    equal moments, a ring, has no definite principal frame: a mirror-symmetric twin was
    otherwise found only as a mirror image), then each matching of the frames (a sign per
    axis, proper rotations only) in turn; the motion must take ``a``'s surface centroid
    onto ``b``'s before the proof, one boolean: the volume ``a`` moved and ``b`` don't share
    is less than ``tol``. Unless ``a`` moved and ``b`` are one solid (:func:`_coincide`:
    face for face the same planes and cylinders bounded by the same edges, within 1e-9 mm,
    so that volume is under 1e-9 mm times their area, far below ``tol``): the copy of a
    part, moved, which most twins are, needs no boolean (its answer would be the same:
    yes). Either way the motion is a proper one (``det r = +1``): ``a`` moved is ``a``'s
    own handedness, so a chiral part and its mirror image are never "same" here (they meet
    only through :func:`congruent`'s mirrored candidate, as "mirror").
    """
    from build123d import Location, Plane

    from spiderpig.shapes import moved as _moved

    ca, ea, ma = sa.frame
    cb, eb, mb = sb.frame
    # A translation first, when the inertia tensors agree in world axes (a part placed
    # without turning it, the common case): its boolean meets exactly coincident faces,
    # ~5-10x faster than one at the principal frames' rotation, which for a part with two
    # equal moments (a ring, a disc) turns it about its axis by an arbitrary angle.
    ia, ib = ea @ np.diag(ma) @ ea.T, eb @ np.diag(mb) @ eb.T
    eye = (np.eye(3),) if np.abs(ia - ib).max() <= 1e-6 * max(np.abs(ib).max(), 1.0) else ()
    on_b: list | None | bool = False        # b's boundary, once a motion is tried
    for r in (*eye, *(eb @ np.diag(signs) @ ea.T for signs in _SIGNS)):
        if np.linalg.det(r) < 0:
            continue
        t = cb - r @ ca
        if np.abs(r @ sa.surf + t - sb.surf).max() > _CENTROID_TOL:
            continue
        moved = _moved(a, Location(Plane(tuple(t), tuple(r[:, 0]), tuple(r[:, 2]))))
        if _COINCIDE * (sa.area + sb.area) < 0.01 * tol:
            on_b = _boundary(b) if on_b is False else on_b
            if on_b is not None and _coincide(_boundary(moved), on_b):
                return True
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
    ref: Body                                    # its part is the pattern
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
                part = cast("Shape", g.ref.part)  # a made group's bodies all have a part
                m = part.mirror(Plane.XY)
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
HORN_LOCK = "threadlocker_222"      # low strength: an M2 / M3 horn screw comes out again
LOCK_PER_THREAD = 0.01              # of a 10 ml bottle, a drop per thread


def stack(total: float, steps, round_to: float | None = None) -> tuple[list[float], float]:
    """The one greedy shim loop: thicknesses from ``steps`` (any order) making up ``total``
    mm, thickest first, and what is left under the thinnest (never negative); with
    ``round_to``, ``total`` rounded to that step first. Every construction's shims and
    washers stack through it (the crank's horn shims and clamp take-up, the frame ties, a
    pillar's end shims, :func:`materials.washer_stack`, a Chicago pin's head spacer)."""
    if round_to:
        total = round(total / round_to) * round_to
    left, out = round(total, 3), []
    for s in sorted((float(v) for v in steps), reverse=True):
        while left >= s - 1e-6:
            out.append(s)
            left = round(left - s, 3)
    return out, max(left, 0.0)


def shim_breakdown(total: float, sizes) -> list[float]:
    """DIN 988 shims making up ``total`` mm (0.1 mm steps), thickest first (:func:`stack`
    on the item's sizes)."""
    return stack(total, sizes)[0]


SHIM_FAMILIES = ("shim_din988_3x6", "shim_din988_4x8", "shim_din988_6x12")
_GAP_SHIM = re.compile(r"(\d+(?:\.\d+)?) mm in the gap")
_STACK_SHIMS = re.compile(r"\bshims ([\d.]+(?: \+ [\d.]+)*) mm")


SHIM_STEP = 0.5         # the thin step stacked under a column's end: one 0.5 mm washer
#                         (M3 DIN 125 3.2 x 7 x 0.5, M4 DIN 433 4.3 x 8 x 0.5: $0.05-0.06
#                         where a DIN 988 shim is $5-13 sold singly, 2026-10-05: SHIM_AS)

SHIM_AS: dict[str, tuple[str, int]] = {
    "shim_din988_3x6_t1": ("m3_washer", 2),
    "shim_din988_3x6_t0p5": ("m3_washer", 1),
    "shim_din988_4x8_t1": ("m4_washer_433", 2),
    "shim_din988_4x8_t0p5": ("m4_washer_433", 1),
}
"""A thickness bought as stock washers instead: a DIN 125 M3 washer (3.2 x 7 x 0.5, Bolt
Depot 4513, $0.05 each, 2026-10-08) is 0.5 mm of the ring for $0.05 where a DIN 988 shim
sold singly is $5-13; two make the 1 mm shim; the M4 DIN 433 one (4.3 x 8 x 0.5) the same
for the 4 x 8 family. The M3 washer is 1 mm wider than the 3 x 6 shim it stands for (Accu's
DIN 433, 6 mm, was its own cart until 2026-10-08): the constructions model and claim a
clamped stack at :func:`shim_od`. Clamped shims only (a horn screw's head, a pillar's end,
a frame tie): the unclamped ones are printed (construction.pivots.common.gap_washers, the
Chicago pins' head spacers)."""


def shim_od(family: str) -> float:
    """The widest ring a clamped stack of ``family`` holds: its DIN 988 shims, or the washers
    some thicknesses are bought as (:data:`SHIM_AS`: the M3 family's DIN 125, 7 mm)."""
    ods = [float(get(family).dims["od"])]
    ods += [float(get(w).dims["od"]) for k, (w, _) in SHIM_AS.items()
            if k.startswith(family + "_t")]
    return max(ods)


def shim_key(family: str, t: float) -> str:
    """The catalog item of one thickness of a DIN 988 family: ``shim_din988_4x8_t0p5``."""
    return f"{family}_t{t:g}".replace(".", "p")


def shim_as_bought(family: str, t: float) -> str:
    """One shim of a stack as the BOM orders it: ``two DIN 125 washers`` for a 1 mm M3
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
            told = (stacks or {}).get(name) or (
                (stacks or {}).get(name[2:]) if name[:2] in ("L.", "R.") else None)
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
    families the 1.0 mm shim and the thin step (0.5 mm: a stock washer,
    :data:`SHIM_STEP`), else its catalog ``t``."""
    if family in ("shim_din988_3x6", "shim_din988_4x8"):
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
      stock horn), a drop each; none in a plastic horn (metal to metal only).
    """
    lines: list[BomLine] = []
    notes: list[str] = []
    replaced: set[str] = set()
    stacks: dict[str, list[str]] = {}
    horn_screws: list[str] = []
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
    if stacks:
        notes.append("Horn screw shims (under each head): " + "; ".join(
            f"screws {', '.join(s)}: {k}" for k, s in stacks.items()) + ".")
    metal = _metal_horn(mech.meta)
    if horn_screws and metal:
        lines.extend(BomLine(HORN_LOCK, LOCK_PER_THREAD,
                             f"{n}: into the metal horn (a drop, metal to metal)")
                     for n in horn_screws)
        notes.append(f"Horn screws: a drop of low-strength threadlocker (Loctite 222) each "
                     f"({len(horn_screws)}), steel into the metal horn; none in a plastic horn.")
    gaps = {name: n.get("bond_gap_mm", 0.0)
            for name, n in ((mech.meta or {}).get("chicago") or {}).items() if n.get("bond_gap_mm")}
    if gaps:
        notes.append("Chicago pins bonded off the barrel head (no printed spacer under it, "
                     "thinner than a print): " + ", ".join(
                         f"{k} {v:g} mm" for k, v in sorted(gaps.items()))
                     + "; set the gap with a feeler gauge while the epoxy cures.")
    return lines, notes, replaced


# ---------------------------------------------------------------------------
# The BOM
# ---------------------------------------------------------------------------


def bought_lines(mech) -> list[BomLine]:
    """What :func:`bom_from_mechanism` buys, line by line, before it groups them: each
    purchased body's item, the fitting's lines, the extras, the shim stacks as what is
    bought (:func:`split_shims`: stock washers). No filament, no sheets (the build adds
    those). A line whose ``where`` starts with a body's name is that body's
    (:func:`line_body`): the assembly guide lists it where the body goes on."""
    fitted, _, replaced = fitting_lines(mech)
    lines = [BomLine(b.bom_key, 1, b.name) for b in mech.bodies
             if b.fab == "purchased" and b.bom_key and b.name not in replaced]
    lines += fitted
    lines += list(mech.bom_extras)
    by_name = {b.name: b for b in mech.bodies}
    lines, _ = split_shims(lines, by_name, (mech.meta or {}).get("shim_stacks"))
    return lines


def line_body(line: BomLine, names) -> str | None:
    """The body a line is for, when its ``where`` starts with one of ``names``."""
    m = re.match(r"[A-Za-z0-9_.]+", line.where or "")
    if m is None:
        return None
    w = m.group(0).rstrip(".:")
    return w if w in names else None


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
    lines.extend(BomLine(body.bom_key, 1, body.name) for body in mech.bodies
                 if body.fab == "purchased" and body.bom_key and body.name not in replaced)
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
                material=(sheet_name(g.ref.sheet) if g.ref.sheet else sheet)
                if method == "laser" else (_filament_name(fil) if fil else fil_name),
                size_mm=_footprint(g.ref.part), volume_cm3=part_volume(g.ref.part) / 1000.0,
                qty=g.qty, names=list(g.names), mirrored=len(g.mirrored),
                sheet=(g.ref.sheet or str(mech.meta.get("sheet") or "")) if method == "laser"
                else "",
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
                          for f, g in grams.items() if g > 0},
               cutting=cutting_rows(made))
