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
import itertools
import json
import math
import re
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, NamedTuple, cast

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
    # a decanting cradle's laser parts (:func:`cut_features`; ``None`` elsewhere, and left
    # out of the BOM's JSON then): what its cutting fit prices a file by
    cut_mm: float | None = None      # the cut's length: every contour, outline and holes
    pierces: int | None = None       # its closed contours (a pierce each)
    blank_mm: tuple[float, float] | None = None   # its blank, long side first, as its DXF lies


CUT_KEYS = ("cut_mm", "pierces", "blank_mm")
""":class:`MadeRow`'s cutting features: in the JSON only where set (a walker's BOM is as it
was before them)."""


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


class CutPart(NamedTuple):
    """One part (or a file of ``qty`` of it) as a cutting fit prices it: its area (cm^2, the
    outline less its holes), how many, the cut's length (mm), its contours, its blank's long
    and short side (mm). A part given by its area alone (``(area, qty)``: verify's floor) has
    the rest 0, which :meth:`CutFit.one` lifts to the service's smallest part's."""

    area_cm2: float
    qty: int
    cut_mm: float = 0.0
    pierces: int = 0
    long_mm: float = 0.0
    short_mm: float = 0.0


@dataclass(frozen=True)
class CutFit:
    """A decanting cradle's sheet's cutting price, fitted on a service's live quotes: one
    part costs ``base + k x area^area_exp x cut^cut_exp x blank^blank_exp x long^long_exp x
    short^short_exp`` (area and blank in cm^2, the cut in m, the blank's sides in mm), and
    each of a file of ``q`` that times :meth:`factor` ``(q)``; ``quoted`` says on what."""

    k: float
    area_exp: float = 0.0
    cut_exp: float = 0.0
    blank_exp: float = 0.0
    long_exp: float = 0.0
    short_exp: float = 0.0
    base: float = 0.0
    qty: tuple[tuple[int, float], ...] = ()
    quoted: str = ""

    def one(self, p: CutPart, least: tuple[float, float] = (0.0, 0.0)) -> float:
        """One part off (USD); ``least``, the service's smallest part (short, long side mm),
        bounds what a part given by its area alone is priced as."""
        lo, hi = least
        short = max(p.short_mm, lo, 1e-3)
        long_ = max(p.long_mm, hi, short)
        blank = long_ * short / 100.0
        area = max(p.area_cm2, min(blank, lo * hi / 100.0), 1e-6)
        cut = max(p.cut_mm, 2.0 * (lo + hi), 1e-3) / 1000.0
        return self.base + self.k * (area ** self.area_exp * cut ** self.cut_exp
                                     * blank ** self.blank_exp * long_ ** self.long_exp
                                     * short ** self.short_exp)

    def factor(self, q: int) -> float:
        """Each of ``q`` off as a fraction of one off: linear in ``q`` between the fit's
        tiers (``qty``: ``(q, factor)``; 1 at one off), the last one held past it."""
        pts = [(1, 1.0), *self.qty]
        if q <= 1:
            return 1.0
        for (q0, f0), (q1, f1) in itertools.pairwise(pts):
            if q <= q1:
                return f0 + (q - q0) / (q1 - q0) * (f1 - f0)
        return pts[-1][1]


_SCS_2026_10_14 = "SendCutSend's live quotes of 2026-10-14"
_AL_EXP = {"area_exp": 0.2488, "cut_exp": 0.2293, "blank_exp": 0.3255}
CRADLE_CUT_FITS: dict[str, CutFit] = {
    "al5052_4p8mm": CutFit(1.3457, **_AL_EXP, qty=((2, 0.673), (10, 0.489)),
                           quoted=_SCS_2026_10_14 + " (14 files)"),
    "al7075_4p8mm": CutFit(2.5197, **_AL_EXP, qty=((2, 0.777), (10, 0.571)),
                           quoted=_SCS_2026_10_14 + " (4 files)"),
    "al5052_6p35mm": CutFit(1.6400, **_AL_EXP, qty=((2, 0.759), (10, 0.547)),
                            quoted=_SCS_2026_10_14 + " (3 files)"),
    "al6061_3p2mm": CutFit(1.3024, **_AL_EXP, qty=((2, 0.719), (10, 0.612)),
                           quoted=_SCS_2026_10_14 + " (7 files)"),
    "birch_9mm": CutFit(0.009984, long_exp=0.9227, short_exp=0.5536,
                        qty=((2, 0.751), (10, 0.495)), quoted=_SCS_2026_10_14 + " (6 files)"),
    # (no cradle cuts it since 2026-10-13: A's old fit, on its area alone, kept for one that
    # takes it again)
    "al5052_3p2mm": CutFit(0.0323, area_exp=1.0, base=12.57, qty=((2, 0.72),),
                           quoted="SendCutSend's live quotes of 2026-10-10 (3 files of A)"),
}
"""A decanting cradle's cutting, per sheet: what SendCutSend charges for a file, the material
and its default deburring included (its parts are big: the walkers' per-area rates priced a
base plate at USD 488 where SendCutSend quoted 91), **fitted from SendCutSend's live quotes
of 2026-10-14** (design review round 2's 40 quotes, 22:51 to 23:51: each file of both
cradles at its quantity with its 2, 10, 50 ... tiers, and five reshaped ones; 34 different
files and sheets). The aluminium, one off: ``k x area^0.2488 x cut^0.2293 x
blank^0.3255`` (a power law in the part's area cm^2, its cut's length m and its blank
cm^2, the exponents shared, ``k`` the sheet's: 7075 at 1.87 x 5052's at about the same
thickness), one off within -6 to +11 % of every file quoted, at its quantity within -6 to
+10 % (but the 0.250 in frame plates, an option no order has, +15 % at two); the pierces
add nothing the cut's length doesn't say (their term came out at or under 0). The birch
(CNC routed): ``0.009984 x long^0.9227 x short^0.5536``, its blank's sides in mm, within
0.6 %. Each of ``q`` off: the quotes' median factor at the 2 and 10 tiers, linear in ``q``
between (a file of 4, 6 or 8 within 4 %); SendCutSend's discount at two runs x0.65-0.72 in
0.187 in 5052, 0.77-0.78 in 7075, 0.72 in 6061, 0.70-0.78 in the birch. On the reviewed
snapshot's files (4583390) B's come to USD 760.28 (quoted 758.19) and A's to 580.03
(573.82). A sheet with no fit here takes the walkers' per-area rate.

Checked again on SendCutSend's live quotes of 2026-10-21 (simplification round 1's stand
check, two at a time in 0.187 in 5052): B's outer frame plate FR576x477 quotes USD 175.72 the
pair where the fit says 190.12 (+8.2 %), its inner plate FR525x312 104.54 where it says 97.00
(-7.2 %): the pair 280.26 against 287.12 (+2.4 %). Not refitted: the fit is one law per sheet
(its 14 files of 2026-10-14 within -6 to +11 %), and these two err either way, so a ``k``
taken down to the outer plate's quote would take the inner one 15 % under its own (a file's
quote, not its fit, is what an order pays: upload the files)."""
CRADLE_LIVE_QUOTES = (
    ("decanter_rcm", "al5052_4p8mm", "frame_outer", 2, 1304.7, 3564, 16, 576.0, 476.8, 175.72),
    ("decanter_rcm", "al5052_4p8mm", "torso", 2, 360.4, 1593, 6, 524.0, 312.6, 104.54),
)
"""SendCutSend's live quotes of 2026-10-21 that :data:`CRADLE_CUT_FITS` is checked on, not
fitted to: (cradle, sheet, part, quantity, area cm^2, cut mm, pierces, blank long, short mm,
USD for the file); ``tests/test_cradle.py`` holds the fit within 11 % of each."""
CRADLE_CUT_SOURCE = ("a fit, per sheet, to SendCutSend's live quotes of 2026-10-14 for both "
                     "cradles' files (each file's price from its area, its cut's length and "
                     "its blank, the birch's from its blank, at its quantity's discount: on "
                     "the files quoted it says USD 760 for B's, quoted 758.19, and 580 for "
                     "A's, quoted 573.82, a file off by up to 10 %), the material and "
                     "SendCutSend's default deburring included, US shipping free over USD 39; "
                     "upload the files for the real quote")
"""Where a cradle's cutting estimate comes from (its BOM and ORDER.md print it)."""
CRADLE_CUT_PROXIES: dict[str, tuple[str, float, float, str]] = {}
"""A cradle's sheets that have no fitted price, and what bounds an estimate of their cutting:
``(the nearest fitted sheet, its price's low and high factor, the basis)``. Only an estimate
(the BOM and ORDER.md say so, and leave it out of the total); a sheet not here stays
unpriced, its area given. Empty since the quotes of 2026-10-14 priced the 7075 and the birch
(the 7075 had been "x1 to x2 of the 0.187 in 5052 fit", USD 74-148, quoted 170.40; the
birch unpriced, quoted 236.09)."""


def _least(key: str) -> tuple[float, float]:
    """The service's smallest part on sheet ``key`` (short, long side mm; 0 without one)."""
    try:
        lo, hi = sorted(float(v) for v in get(key).dims.get("min_part") or (0.0, 0.0))
    except KeyError:
        return (0.0, 0.0)
    return lo, hi


def cut_estimate(key: str, parts: list, fit: CutFit | None = None) -> float | None:
    """What a service charges to cut ``parts`` (``(area cm^2, qty)`` each, or a
    :class:`CutPart`) from sheet ``key``, the material included: each part
    ``max(cut_min_usd, cut_usd_cm2 x area)`` (the sheet item's rates); ``None`` without them.
    Calibrated on SendCutSend's per-file live quotes of 2026-10-08 (:data:`CUT_SOURCE`): in
    3 mm acrylic a small part is USD 1.26-1.40 whatever its size (1.33 at 4 off), the 93
    cm^2 deck plate 6.88; in 5052 0.080-0.090 in the plates come to about 0.20 per cm^2, in
    6061 0.100 in 0.40, a small one at least 2.20. On the Strider double: acrylic 49.45
    (quoted 49.44), aluminium 68.48 (67.70). ``fit``: a decanting cradle's sheet
    (:data:`CRADLE_CUT_FITS`) instead, each file at its quantity's discount: the parts are
    merged into files by their features first, so a grouped and an ungrouped BOM (verify's,
    audit's) come to the same."""
    if fit is not None:
        files: dict[tuple, int] = {}
        for p in parts:
            c = CutPart(*p)
            k = (round(c.area_cm2, 1), round(c.cut_mm), c.pierces, round(c.long_mm, 1),
                 round(c.short_mm, 1))
            files[k] = files.get(k, 0) + c.qty
        least = _least(key)
        return round(sum(q * round(fit.one(CutPart(a, q, cut, n, lng, sht), least)
                                   * fit.factor(q), 2)
                         for (a, cut, n, lng, sht), q in files.items()), 2)
    d = get(key).dims
    rate, least = d.get("cut_usd_cm2"), d.get("cut_min_usd")
    if rate is None or least is None:
        return None
    return round(sum(q * max(float(least), float(rate) * a) for a, q in parts), 2)


def cut_parts(rows: list[MadeRow], key: str) -> list[CutPart]:
    """``rows``' parts as a cutting fit prices them, on sheet ``key`` (the area: the volume
    over its thickness)."""
    t_cm = float(get(key).dims.get("thickness", 3.0)) / 10.0
    return [CutPart(m.volume_cm3 / t_cm, m.qty, m.cut_mm or 0.0, m.pierces or 0,
                    *(m.blank_mm or (0.0, 0.0))) for m in rows]


def cutting_rows(made: list[MadeRow], fits: dict[str, CutFit] | None = None) -> list[CutRow]:
    """A :class:`CutRow` per service and sheet the laser-cut ``made`` rows are cut from
    (``fits``: per sheet, a product's own estimate, :data:`CRADLE_CUT_FITS`)."""
    by: dict[str, list[MadeRow]] = {}
    for m in made:
        if m.method == "laser" and m.sheet and cut_by(m.sheet):
            by.setdefault(m.sheet, []).append(m)
    out = []
    for key, rows in by.items():
        t_cm = float(get(key).dims.get("thickness", 3.0)) / 10.0
        parts = [(m.volume_cm3 / t_cm, m.qty) for m in rows]
        fit = (fits or {}).get(key)
        out.append(CutRow(cut_by(key), key, get(key).name, sum(m.qty for m in rows),
                          round(sum(a * q for a, q in parts), 1),
                          cut_estimate(key, cut_parts(rows, key) if fit else parts, fit)))
    return sorted(out, key=lambda c: (c.service, c.sheet))


def file_estimates(made: list[MadeRow], fits: dict[str, CutFit] | None = None
                   ) -> dict[str, float]:
    """Each fitted laser-cut file's estimate (USD, at its quantity), by its part's name (a
    cradle's: ``fits``), for ORDER.md's uploads."""
    out: dict[str, float] = {}
    for m in made:
        fit = (fits or {}).get(m.sheet)
        if m.method != "laser" or fit is None:
            continue
        got = cut_estimate(m.sheet, cut_parts([m], m.sheet), fit)
        if got is not None:
            out[m.name] = got
    return out


def cut_features(body) -> tuple[float, int, tuple[float, float]]:
    """A laser-cut ``body``'s cutting features as its per-part DXF has them (the contours
    :func:`layout.save_parts` writes; SendCutSend's files carry no kerf): the cut's length
    (mm, every contour), its contours (a pierce each), its blank (long side first, mm)."""
    from spiderpig.layout import section_of
    from spiderpig.manufacture import blank_of

    sketch = section_of(body)
    wires = list(sketch.wires())
    short, long_ = blank_of(body, sketch)
    return (round(sum(float(w.length) for w in wires), 2), len(wires),
            (round(long_, 2), round(short, 2)))


def deburr_least(sheet: str) -> tuple[float, float] | None:
    """The smallest part (short, long side mm) the service deburrs on ``sheet``: its item's
    ``deburr_mm``, else SendCutSend's for its metal (the rule every aluminium spec table
    gives, :data:`hardware.sheet_catalog.SCS_DEBURR_MM`); ``None``: not deburred at all (a
    routed wood, an acrylic)."""
    from spiderpig.hardware.sheet_catalog import SCS_DEBURR_MM

    try:
        d = get(sheet).dims
    except KeyError:
        return None
    least = d.get("deburr_mm") or (SCS_DEBURR_MM if d.get("metal")
                                   and d.get("service") == "SendCutSend" else None)
    if not least:
        return None
    lo, hi = sorted(float(v) for v in least)
    return lo, hi


def hand_deburred(made: list[MadeRow]) -> list[MadeRow]:
    """The laser-cut rows that arrive as cut: a metal part whose blank is under the
    service's deburring minimum (:func:`deburr_least`, the rule of
    :func:`manufacture.deburr_issue`): to deburr by hand."""
    out = []
    for m in made:
        if m.method != "laser" or not m.sheet or not m.blank_mm:
            continue
        least = deburr_least(m.sheet)
        if least is None:
            continue
        lo, hi = least
        long_, short = m.blank_mm
        if short < lo - 1e-3 or long_ < hi - 1e-3:
            out.append(m)
    return out


def deburr_note(rows: list[MadeRow]) -> str:
    """The BOM's and ORDER.md's line for :func:`hand_deburred`'s rows: which parts arrive as
    cut, why, and what to do (the assembly guide's first stack step says it too)."""
    each = ", ".join(f"{m.name} x {m.qty} ({m.blank_mm[1]:.1f} mm wide)" for m in rows
                     if m.blank_mm)
    lo, hi = deburr_least(rows[0].sheet) or (25.4, 76.2)
    return (f"Arrive as cut, not deburred (SendCutSend deburrs a part only from "
            f"{lo / 25.4:g} x {hi / 25.4:g} in, {lo:g} x {hi:g} mm): {each}. Deburr each by "
            "hand before it goes on (a deburring tool or a file, then fine sandpaper on a flat "
            "block): both faces and each pin hole's edges; the dross is on the laser's exit "
            "face.")


def cut_ranges(cutting: list[CutRow], made: list[MadeRow],
               proxies: dict[str, tuple[str, float, float, str]] | None = None
               ) -> dict[str, tuple[float, float, str] | None]:
    """Each unpriced cutting line's estimate (``cutting``'s rows with no ``usd``), by sheet:
    ``(low, high, basis)`` from its proxy (:data:`CRADLE_CUT_PROXIES`: the nearest fitted
    sheet's price for the same parts, times its factors), ``None`` where none bounds it."""
    out: dict[str, tuple[float, float, str] | None] = {}
    for c in cutting:
        if c.usd is not None:
            continue
        proxy = (proxies or {}).get(c.sheet)
        fit = CRADLE_CUT_FITS.get(proxy[0]) if proxy else None
        if proxy is None or fit is None:
            out[c.sheet] = None
            continue
        rows = [m for m in made if m.method == "laser" and m.sheet == c.sheet]
        usd = cut_estimate(proxy[0], cut_parts(rows, c.sheet), fit) or 0.0
        out[c.sheet] = (round(usd * proxy[1], 2), round(usd * proxy[2], 2), proxy[3])
    return out


def unpriced_cut_note(cutting: list[CutRow], ranges: dict[str, tuple[float, float, str] | None]
                      ) -> str:
    """What a total leaves out (a cradle's BOM and ORDER.md): "excludes N unpriced cut
    lines: ...", each with its area and, where it has one, its estimate and the estimate's
    basis; empty when every line is priced."""
    rows = [c for c in cutting if c.usd is None]
    if not rows:
        return ""
    each = []
    for c in rows:
        got = ranges.get(c.sheet)
        what = f"{c.name} ({c.parts} part{'s' if c.parts != 1 else ''}, {c.area_cm2:g} cm²"
        if got is None:
            each.append(what + ": no fitted price, quoted at upload)")
        else:
            each.append(what + f": an estimate of USD {got[0]:.0f}-{got[1]:.0f}, from {got[2]})")
    low = sum(g[0] for c in rows if (g := ranges.get(c.sheet)))
    high = sum(g[1] for c in rows if (g := ranges.get(c.sheet)))
    still = [c.name for c in rows if ranges.get(c.sheet) is None]
    tail = ""
    if low or high:
        tail = (f" The estimates add USD {low:.0f}-{high:.0f}"
                + (f"; {', '.join(still)} still unpriced." if still else "."))
    return (f"The total excludes {len(rows)} unpriced cut line{'s' if len(rows) > 1 else ''}: "
            + "; ".join(each) + "." + tail)


SAW_KERF = 1.0    # mm lost to each cut of rod stock (a hacksaw or a cut-off disc)


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
           "threadlocker_243", "m2_self_tap_6", "servo_bus_cable_5264")
"""Shop supplies taken as on hand (the user's, 2026-10-05), and the M2 x 6 self-tappers
and the bus cable that come in every STS3215's box (Seeed's ST3215-C001 part list: 2 horns,
18 screws and a "JST Wire" a servo; Waveshare's package photo shows the pointed
self-tappers; BOM study 2026-10-08):
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
    cut_source: str = CUT_SOURCE        # where the cutting's estimate comes from
    cut_ranges: dict[str, tuple[float, float, str] | None] | None = None
    """A cradle's unpriced cutting lines' estimates (:func:`cut_ranges`; ``None``: a walker's
    BOM, which doesn't say what its total leaves out)."""

    @property
    def unpriced_cuts(self) -> str:
        """What the total leaves out of the cutting (:func:`unpriced_cut_note`; a cradle's
        only)."""
        if self.cut_ranges is None:
            return ""
        return unpriced_cut_note(self.cutting, self.cut_ranges)

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
                            f"{c.area_cm2:g} cm^2 of parts; {self.cut_source}"])
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
                      f"{self._cut_cost(c)} |" for c in self.cutting]
            lines += ["", f"The cutting is {self.cut_source}."]
        lines += ["", f"Estimated total: **${self.cost_usd:.2f}**: purchases "
                  f"${self.purchases_usd:.2f} (pack prices at the listed vendor) and cutting "
                  f"${self.cutting_usd:.2f}; excludes shipping and the shop supplies on hand."]
        if self.unpriced_cuts:
            lines.append(f"**{self.unpriced_cuts}**")
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

    def _cut_cost(self, c: CutRow) -> str:
        """A cutting line's cost cell: its price; unpriced, empty (a walker's) or said so,
        with its estimate where it has one (a cradle's)."""
        if c.usd is not None:
            return f"${c.usd:.2f}"
        if self.cut_ranges is None:
            return ""
        got = self.cut_ranges.get(c.sheet)
        return "unpriced" + (f" (≈${got[0]:.0f}-{got[1]:.0f}, an estimate)" if got else "")

    def as_dict(self) -> dict:
        if self.cut_ranges is not None:         # (a cradle's: what its total leaves out)
            return dict(self._as_dict(), unpriced_cutting=[
                {"sheet": c.sheet, "name": c.name, "parts": c.parts, "area_cm2": c.area_cm2,
                 "estimate_usd": (list(got[:2]) if (got := self.cut_ranges.get(c.sheet))
                                  else None),
                 "basis": got[2] if got else "no fitted price: quoted at upload"}
                for c in self.cutting if c.usd is None],
                unpriced_cutting_note=self.unpriced_cuts)
        return self._as_dict()

    def _as_dict(self) -> dict:
        return {
            "title": self.title,
            # a shop supply on hand (ON_HAND) costs this build nothing: listed, its pack
            # price kept, ``on_hand`` set; a sheet a service cuts is its upload (``cut_by``)
            "purchased": [dict(r.__dict__, on_hand=r.key in ON_HAND, cut_by=cut_by(r.key),
                               cost_usd=r.cost_usd if bought(r.key) else 0.0)
                          for r in self.purchased],
            "made": [{k: v for k, v in m.__dict__.items() if k not in CUT_KEYS or v is not None}
                     for m in self.made],
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
    """Its box's size as it stands in the assembly (a walker's laser parts and every printed
    part; a cradle's laser part gives its blank instead, as its DXF lies: design review
    round 2, b1 had read "356 x 79" against its 360.8 x 25.2 blank)."""
    bb = part.bounding_box()
    x, y, z = bb.size.X, bb.size.Y, bb.size.Z
    return f"{x:.0f} x {y:.0f} x {z:.1f} mm"


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
CRADLE_FILAMENT_USE = {TPU_FILAMENT: "the TPU liners, the brake washer, the feet, the "
                                      "pack's spacers and (B) the quick-release pins' keepers",
                       PRESS_FILAMENT: "the saddles' carriers and V blocks, the rear knob and "
                                       "its hub, the base stop's rails and gate and the lip "
                                       "gauge, solid; the hand crank's hub, arm, cap and grip "
                                       "(its clutch's plungers push on them always)"}
CRADLE_PRINT_NOTE = (
    "Print the saddles' carriers and V blocks, the rear knob and its hub, the rear lead "
    "screw, its nut and its thrust ring, the base stop's rails and gate and the lip gauge in "
    "PETG at 100 % infill (the lead screw standing on its foot, a brim, on the rear "
    "carrier's plate: alone, a slender part waits on each layer): they carry the bottle or hold "
    "it where it is, and their stresses are rated solid (up to 11 MPa as long as a bottle "
    "rests: PETG's resin deflects at 64 °C under 1.8 MPa, Eastman's Eastar 6763, so the "
    "cradle stays out of the sun, under 50 °C; PLA softens at 55). The rails stand on their "
    "flat bottoms (their T-slot's lips then bend in their layers), 249 mm long: a bed of "
    "250 mm or more (a Prusa MK4's 250 x 210; not a 220 mm Ender-3). The base stop's gate "
    "lies on its top (its ears' roots and its pointer in its layers), the lip gauge on its "
    "side (support under its bar and arm), the rear "
    "knob's hub on its flange; a brim under the rear carrier (it stands 142 mm tall), "
    "support under its tower's arm and under the rear V block's flats and bosses. The hand "
    "crank in PETG too, its hub solid and its arm, cap and grip at 40 % (its clutch's "
    "plungers push on them always; check the clutch after a hot trip: the guide). The rest "
    "in PLA: the brackets at 20 % infill with 4 walls (B's in PETG: they stay on its "
    "board, which rides in the car, clamped for good under their nuts), the other knobs (the "
    "base board's thumb knobs, or B's studs' knobs, too) and the light's clip (on its tube's "
    "foot, a brim) at 40 %; the TPU parts at 15-20 %, the pack's spacers (pack_spacer_*.stl, "
    "each lying as it goes in the box) at 15 % gyroid with two walls. Each STL is posed as "
    "it should print. For a cradle that travels in a car in summer, print the knobs in PETG "
    "too.")
"""A decanting cradle's (:mod:`construction.cradle`; review round 2, N6 and m9; the drive's
m7)."""
CRADLE_GREASE_NOTE = (
    "Grease (the food-grade one bought, NSF H1: Super Lube 21030, since the saddles sit in "
    "the wine's path; any PTFE or lithium grease only away from it): the front M6 "
    "thumbscrew as it goes in (stainless in a stainless nut galls dry) and the front V "
    "block's sides in its box (it slides dry too, smoother greased). The rear lead screw is "
    "printed and runs dry: keep grease off its thread, its nut, its thrust ring, the TPU "
    "brake washer under it and the rear knob's teeth (the knob's teeth lock the screw, and "
    "the washer only slows it with the knob lifted, dry). The same grease on the hand "
    "crank's clutch: its seat ring's face and "
    "six holes and its thrust washer (the clutch is set greased; dry, it can lock), and "
    "again at its yearly check. And wherever a link turns, on its hard-anodised journal: "
    "the pivot posts (B's O1 and O2, A's G), every link pin and the crankpin, wiped with a "
    "thin film as each goes in and again at each set-up (aluminium on aluminium galls dry; "
    "on the hard coat, greased, the bore turns on a surface harder than itself). The bare "
    "6061 tie rods and columns carry no link and take no grease.")
"""A decanting cradle's (:mod:`construction.cradle`; review round 1, m9; round 2, N5 and the
drive's B-1)."""
CRADLE_CARTS_NOTE = (
    "The carts (simplification round 1, 2026-10-21: folded to seven and the upload). Their "
    "shipping isn't in the totals; priced only where a page or a cart gave it that day: "
    "SendCutSend free in the US over USD 39; Woodcraft standard USD 13.99 to 94103 for the "
    "panels and the glue (its cart's rates). Unpriced: McMaster-Carr (\"applicable shipping "
    "and tax will be added\", no order minimum), Bolt Depot, Fastenal, Phidgets (from "
    "Calgary: shipping, duties and brokerage at checkout), Battery Junction, and Speedy "
    "Metals (FedEx, calculated at checkout with an account). Speedy takes no order under USD "
    "20 (one cradle's rods come to less: order both cradles' in one cart, or add a stick) and "
    "adds a process fee of about USD 8 to a line of more than ten pieces (A's twelve posts: "
    "two lines of six). One cart fewer: McMaster's multipurpose 6061 rod (8974K28, 1/2 in, "
    "+-0.006 in, 6 ft USD 13.75; 8974K48, 5/8 in, 1/2 ft USD 3.25) in place of Speedy's "
    "pieces, every piece sawn at home before it is faced.")
"""A decanting cradle's ORDER.md note on its carts (simplification round 1: McMaster's pages
rendered 2026-10-21, Woodcraft's cart's shipping rates, Speedy's cart; ``order.py`` lists
each cart)."""
# a foot's sock; a decanting cradle's liners, its collar, its lead screw's brake washer, its
# base plate's feet
_SOCK = re.compile(r"_sock$|_liner$|_collar$|_brake$|foot_boot_")
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
    sock (and a decanting cradle's saddle liners and foot boots, and the keepers on its
    quick-release pins: ``meta["keepers"]``, ``construction.cradle.KEEPER``),
    :data:`PRESS_FILAMENT` (PETG) for a sleeve pressed on its crankpin, for a decanting
    cradle's parts that carry the bottle (``meta["petg"]``, ``construction.cradle.PETG_PARTS``)
    and for its hand crank's parts its notes name (``meta["hand_crank"]["petg"]``: its
    clutch's plungers push on them always), else ``default``."""
    if _SOCK.search(body.name) or body.name in (meta or {}).get("keepers", ()):
        return TPU_FILAMENT
    petg = ((meta or {}).get("hand_crank") or {}).get("petg", ())
    if PRESS_FILAMENT and body.name.split(".", 1)[-1] in petg:
        return PRESS_FILAMENT
    m = _SLEEVE.search(body.name)
    if PRESS_FILAMENT and m and m.group(1) in pressed_sleeves(meta):
        return PRESS_FILAMENT
    if PRESS_FILAMENT and body.name in (meta or {}).get("petg", ()):
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
a frame tie): the unclamped ones are printed (construction.pivots.common.build_sleeves, the
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


def body_line(body) -> BomLine:
    """A purchased body's line: one of its item, or, for an item cut from stock at home
    (``dims["cut_from"]``: a cradle's pivot posts, sawn from a stick of rod), the stock with
    the piece's ``cut X mm`` (its length and ``dims["cut_over"]``), so the cut list counts the
    sticks (:func:`cut_list`)."""
    try:
        dims = get(body.bom_key).dims
    except KeyError:
        return BomLine(body.bom_key, 1, body.name)
    stock = dims.get("cut_from")
    if not stock:
        return BomLine(body.bom_key, 1, body.name)
    cut = float(dims["length"]) + float(dims.get("cut_over") or 0.0)
    return BomLine(str(stock), 1, f"{body.name}: cut {cut:.1f} mm")


def bought_lines(mech) -> list[BomLine]:
    """What :func:`bom_from_mechanism` buys, line by line, before it groups them: each
    purchased body's item, the fitting's lines, the extras, the shim stacks as what is
    bought (:func:`split_shims`: stock washers). No filament, no sheets (the build adds
    those). A line whose ``where`` starts with a body's name is that body's
    (:func:`line_body`): the assembly guide lists it where the body goes on."""
    fitted, _, replaced = fitting_lines(mech)
    lines = [body_line(b) for b in mech.bodies
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
                       groups: dict[str, list[MadeGroup]] | None = None,
                       extra: Sequence[tuple[MadeRow, str]] = ()) -> Bom:
    """Collect purchases and made parts from a fabricated mechanism.

    ``group=False`` lists every made part on its own row (faster: no shape
    comparison); ``groups`` passes :func:`group_made` results already computed
    (method -> groups). ``extra``: printed parts that aren't the mechanism's bodies (a
    decanting cradle's pack spacers, ``cradle_pack.pack_kit``), each a row and its filament
    (catalog key): listed, and their filament counted.
    """
    lines: list[BomLine] = []
    made: list[MadeRow] = []
    notes: list[str] = []
    cradle = bool((mech.meta or {}).get("cradle"))     # a decanting cradle's own estimates
    sheet = mech.meta.get("sheet_name", "sheet")
    filament = filament or mech.meta.get("filament")
    fil_name = _filament_name(filament) if filament else "PLA/PETG"
    fitted, fit_notes, replaced = fitting_lines(mech)
    lines.extend(body_line(body) for body in mech.bodies
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
            key = (g.ref.sheet or str(mech.meta.get("sheet") or "")) if method == "laser" \
                else ""
            size = _footprint(g.ref.part)
            cut_mm = pierces = blank = None
            if method == "laser" and cradle:    # its blank as cut, and what its fit reads
                cut_mm, pierces, blank = cut_features(g.ref)
                t = float(get(key).dims.get("thickness", 0.0)) if key else 0.0
                size = f"{blank[0]:.1f} x {blank[1]:.1f} x {t:g} mm"
            made.append(MadeRow(
                name=g.ref.name, method=method,
                material=(sheet_name(g.ref.sheet) if g.ref.sheet else sheet)
                if method == "laser" else (_filament_name(fil) if fil else fil_name),
                size_mm=size, volume_cm3=part_volume(g.ref.part) / 1000.0,
                qty=g.qty, names=list(g.names), mirrored=len(g.mirrored), sheet=key,
                cut_mm=cut_mm, pierces=pierces, blank_mm=blank,
            ))
            if method == "printed":
                grams[fil] = grams.get(fil, 0.0) + (made[-1].volume_cm3 * g.qty
                                                    * filament_density(fil))
    for row, fil in extra:              # (printed, not bodies: a cradle's pack spacers)
        made.append(row)
        grams[fil] = grams.get(fil, 0.0) + row.volume_cm3 * row.qty * filament_density(fil)
    total = sum(grams.values())
    for fil, g in grams.items():
        if not fil or g <= 0:
            continue
        spool = float(get(fil).dims.get("spool_g", 1000.0))
        use = (CRADLE_FILAMENT_USE if (mech.meta or {}).get("cradle") else FILAMENT_USE)
        what = "printed parts" if fil == filament else \
            ", ".join(sorted({use.get(fil, "printed parts")}))
        lines.append(BomLine(fil, round(g / spool, 3), f"{what}: {g:.0f} g at 100 % infill"))
    if filament and total > 0:
        each = "; ".join(f"{g:.0f} g of {_filament_name(f)} ({filament_density(f)} g/cm3)"
                         for f, g in grams.items() if f and g > 0)
        notes.append(f"Printed parts need about {total:.0f} g of filament at 100 % infill "
                     f"({each}); less with sparse infill.")
        if (mech.meta or {}).get("cradle"):
            notes += [CRADLE_PRINT_NOTE, CRADLE_GREASE_NOTE, CRADLE_CARTS_NOTE]
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
    if cradle and (rough := hand_deburred(made)):
        notes.append(deburr_note(rough))
    cutting = cutting_rows(made, CRADLE_CUT_FITS if cradle else None)
    return Bom(purchased=purchased, made=made, title=title, notes=notes,
               printed_g=total if filament else None,
               filament=fil_name.split()[0] if filament else "PLA", cuts=cut_list(lines),
               filaments={(_filament_name(f) if f else fil_name): g
                          for f, g in grams.items() if g > 0},
               cutting=cutting,
               cut_source=CRADLE_CUT_SOURCE if cradle else CUT_SOURCE,
               cut_ranges=cut_ranges(cutting, made, CRADLE_CUT_PROXIES) if cradle else None)
