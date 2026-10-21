"""A packed cradle's carton and postage (packing review 2's M5): the stock cartons it may ship
in, the cheapest that fits its pack and is rated for its weight, and what USPS Ground
Advantage charges for it. ORDER.md's "Ship it" section (``order.order_markdown``'s
``shipping``).

Every number here was read from the page named with it on 2026-10-21 (the packing's turn 3;
the pages and quotes: the turn's record, DECISIONS.md):

* **Cartons**, Uline's product pages (``https://www.uline.com/Product/Detail/<item>``): the
  inside, the outside (+/- 1/8 in), the board, Uline's "UPS guidelines" weight for it (its
  Box Strength Guidelines table: 200# single wall 40 lb, 275# single wall 65 lb, 275# double
  wall 80 lb), the price per box at its first quantity break (its bundle), and a multi-depth
  box's score lines (cut down to one: "Sides are creased at intervals - just cut to the
  creased height needed, fold down"). Uline's double-wall list has one carton in the 30 x 24
  in footprint, S-12613, at 12 in.
* **Postage**, USPS Ground Advantage Retail, Notice 123 effective 2026-10-04
  (https://pe.usps.com/text/dmm300/Notice123.htm), and DMM 133.1.4.2 and 133.1.8
  (https://pe.usps.com/text/dmm300/133.htm): the dimensional weight is the outside's sides
  each rounded up to the inch, multiplied, over 139, rounded up (over 70 lb the 70 lb price);
  the length fees (over 22 in, over 30 in) and the 2 cubic feet fee (the sides rounded off)
  apply but to an oversized parcel (length and girth over 108 in, up to 130).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

IN = 25.4


@dataclass(frozen=True)
class Carton:
    """A stock carton: ``item`` (Uline's), its ``inside`` and ``outside`` (in: length, width,
    depth), its ``board``, the weight it is rated for (lb, UPS's guideline as Uline prints
    it), the price per box (USD) at its first break (``per``: that break's count), its score
    lines (in, the depths it cuts down to; none: as it is) and its page."""

    item: str
    inside: tuple[float, float, float]
    outside: tuple[float, float, float]
    board: str
    rated_lb: float
    usd: float
    per: int
    scores: tuple[float, ...] = ()
    url: str = ""
    lb: float = 4.0
    """A box's weight (lb: its page's bundle weight over the boxes in it; 4 where the page
    gives none)."""

    def at(self, depth: float) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """Its inside and outside cut down to ``depth`` (in, a score line): the walls'
        thickness off its outside's depth as at full depth."""
        wall = self.outside[2] - self.inside[2]
        return ((self.inside[0], self.inside[1], depth),
                (self.outside[0], self.outside[1], depth + wall))


ULINE = "https://www.uline.com/Product/Detail/"
CARTONS: tuple[Carton, ...] = (
    Carton("S-12613", (30, 24, 12), (30.75, 24.75, 13), "275# double wall", 80.0, 11.71, 10,
           url=ULINE + "S-12613", lb=5.4),
    Carton("S-4417", (30, 24, 12), (30.375, 24.375, 12.625), "200# single wall", 40.0, 6.23,
           15, url=ULINE + "S-4417", lb=3.4),
    Carton("S-13330", (30, 24, 10), (30.375, 24.375, 10.625), "200# single wall", 40.0, 5.94,
           15, url=ULINE + "S-13330", lb=3.1),
    Carton("S-11248", (30, 24, 12), (30.375, 24.375, 12.625), "200# single wall", 40.0, 6.90,
           15, (12, 10, 8, 6, 4), ULINE + "S-11248"),
    Carton("S-4509", (30, 24, 24), (30.5, 24.5, 24.75), "275# single wall", 65.0, 11.64, 10,
           (24, 22, 20, 18, 16, 14), ULINE + "S-4509", lb=6.5),
)
"""The stock cartons in the cradles' 30 x 24 in footprint (Uline, fetched 2026-10-21)."""

USPS_GA_RETAIL: dict[int, tuple[float, float, float, float]] = {
    30: (41.15, 60.45, 76.30, 120.30), 31: (42.15, 61.90, 78.20, 123.30),
    32: (42.90, 63.35, 80.10, 126.20), 33: (43.75, 64.80, 81.95, 129.05),
    34: (44.60, 66.10, 83.70, 131.80), 35: (45.50, 67.55, 85.60, 134.65),
    36: (46.25, 68.80, 87.25, 137.30), 37: (47.10, 70.20, 89.05, 140.10),
    38: (47.85, 71.50, 90.75, 142.75), 39: (48.65, 72.80, 92.55, 145.30),
    40: (49.40, 74.10, 94.15, 147.90), 41: (50.25, 75.40, 95.95, 150.50),
    42: (51.05, 76.60, 97.55, 152.90), 43: (51.70, 77.75, 99.25, 155.40),
    44: (52.50, 78.95, 100.80, 157.75), 45: (53.20, 80.20, 102.45, 160.10),
    46: (53.95, 81.35, 104.00, 162.45), 47: (54.60, 82.45, 105.50, 164.70),
    48: (55.30, 83.55, 107.10, 166.85), 49: (56.00, 84.65, 108.60, 169.10),
    50: (56.65, 85.70, 110.05, 171.25), 51: (57.35, 86.75, 111.50, 173.25),
    52: (58.05, 87.80, 112.95, 175.30), 53: (58.70, 88.85, 114.30, 177.30),
    54: (59.30, 89.75, 115.70, 179.25), 55: (59.90, 90.80, 117.05, 181.15),
    56: (60.55, 91.75, 118.40, 183.00), 57: (61.15, 92.50, 119.75, 184.85),
    58: (61.75, 93.50, 121.05, 186.60), 59: (62.35, 94.30, 122.30, 188.25),
    60: (62.85, 95.20, 123.55, 190.00), 61: (63.40, 96.00, 124.80, 191.60),
    62: (64.00, 96.85, 126.00, 193.25), 63: (64.60, 97.70, 127.05, 194.70),
    64: (65.05, 98.35, 128.30, 196.30), 65: (65.55, 99.10, 129.45, 197.70),
    66: (66.05, 99.85, 130.50, 199.20), 67: (66.55, 100.50, 131.60, 200.50),
    68: (67.10, 101.20, 132.65, 201.75), 69: (67.50, 101.80, 133.70, 203.05),
    70: (68.00, 102.50, 134.70, 204.30),
}
"""USPS Ground Advantage Retail (Notice 123, effective 2026-10-04): lb (not over) -> USD at
zones 1, 4, 5 and 8."""
USPS_OVERSIZED = (116.00, 177.35, 211.30, 301.75)
USPS_ZONES = (1, 4, 5, 8)
FEE_OVER_22, FEE_OVER_30, FEE_OVER_2FT3 = 4.50, 10.00, 21.00
DIM_DIVISOR = 139


def usps(outside_in: tuple[float, float, float], lb: float) -> dict:
    """USPS Ground Advantage Retail for a box ``outside_in`` (in) weighing ``lb``: the
    dimensional weight, the weight billed, the fees, and the price at each of
    :data:`USPS_ZONES` (``usd``)."""
    up = sorted((math.ceil(v - 1e-9) for v in outside_in), reverse=True)
    dim = math.ceil(up[0] * up[1] * up[2] / DIM_DIVISOR - 1e-9)
    billed = max(dim, math.ceil(lb - 1e-9))
    girth = up[0] + 2 * (up[1] + up[2])
    if girth > 108:
        return {"dim_lb": dim, "billed_lb": billed, "oversized": True, "fees": 0.0,
                "usd": dict(zip(USPS_ZONES, USPS_OVERSIZED, strict=True))}
    longest = max(outside_in)
    fees = FEE_OVER_30 if longest > 30 else FEE_OVER_22 if longest > 22 else 0.0
    if math.prod(round(v) for v in outside_in) > 3456:
        fees += FEE_OVER_2FT3
    row = USPS_GA_RETAIL[min(max(billed, 30), 70)]
    return {"dim_lb": dim, "billed_lb": billed, "oversized": False, "fees": fees,
            "usd": {z: round(p + fees, 2) for z, p in zip(USPS_ZONES, row, strict=True)}}


CARTON_LB = 4.0
"""lb a 30 x 24 in carton weighs where its page gives no bundle weight (``Carton.lb``:
S-12613's 10 boxes 54 lb, S-4417's 15 51 lb, S-13330's 15 47 lb, S-4509's 10 65 lb)."""


def carton_for(inside_mm: tuple[float, float, float], kg: float) -> dict | None:
    """The cheapest of :data:`CARTONS` whose inside takes a pack ``inside_mm`` (x, height, z:
    its parts' bounds and their clearance; any way round on the floor, the height its depth),
    cut down to the least score line that does, rated for its weight (``kg`` and
    :data:`CARTON_LB`); None when none is. Its carton, the depth it is cut to (None: as it
    is), the spare room each way (mm), the weight (lb, its own box's: ``Carton.lb``) and
    the postage (:func:`usps`)."""
    x, h, z = inside_mm
    best = None
    for c in CARTONS:
        lb = kg / 0.45359237 + c.lb
        plan = sorted((x, z), reverse=True)
        if plan[0] > c.inside[0] * IN or plan[1] > c.inside[1] * IN or lb > c.rated_lb:
            continue
        depths = sorted(d for d in c.scores if d * IN >= h) if c.scores else (
            [c.inside[2]] if c.inside[2] * IN >= h else [])
        if not depths:
            continue
        depth = depths[0]
        inside, outside = c.at(depth) if c.scores else (c.inside, c.outside)
        post = usps(outside, lb)
        key = (post["usd"][5] + c.usd, c.usd)
        if best is None or key < best[0]:
            best = (key, {"carton": c, "cut_to_in": depth if c.scores and depth < c.inside[2]
                          else None, "inside_in": inside, "outside_in": outside,
                          "spare_mm": [round(c.inside[0] * IN - plan[0], 0),
                                       round(depth * IN - h, 0),
                                       round(c.inside[1] * IN - plan[1], 0)],
                          "lb": round(lb, 1), "postage": post})
    return best[1] if best else None


FIT_MM = 1.0
"""mm a carton's inside clears the pack's parts' bounds each way (the review's m5: the
model's 3 mm all round cost an inch)."""


def carton_of(pack: dict, kg: float) -> dict | None:
    """:func:`carton_for` of a cradle's pack payload ``pack`` (``cradle_pack``'s: its parts'
    bounds ``packed_mm`` and :data:`FIT_MM` all round, else its box's inside) weighing
    ``kg``: the carton ORDER.md names and the guide's box steps fill and close."""
    parts = pack.get("packed_mm")
    fit = (tuple(float(v) + 2 * FIT_MM for v in parts) if parts
           else tuple(float(v) for v in pack["box"]["inside_mm"]))
    return carton_for(fit, kg)          # type: ignore[arg-type]


def board_mm(c: Carton) -> float:
    """The carton's board (mm): a wall, from its page's inside and outside lengths (S-4509:
    30 and 30 1/2 in, 6.4 mm), how thick a layer of it folded flat is."""
    return (c.outside[0] - c.inside[0]) / 2 * IN


def fill_words(got: dict, kept: str = "") -> dict[str, str]:
    """How the spare inside a carton (:func:`carton_for`'s ``got``) is filled with the
    bundle's spare cartons (packing review 3's M5: unfilled, the whole pack shifts in it,
    and A's caps had 67 mm to rise): ``"sides"``, the end and the side; ``"top"``, the layers
    over the pack under the lid (``kept``: the parts a hole is cut round, nothing pressing on
    them); each "" where there is no spare that way."""
    spare = got["spare_mm"]
    t = board_mm(got["carton"])
    ways = []
    if spare[0] >= t / 2:
        ways.append(f"{spare[0]:.0f} mm across one end")
    if spare[2] >= t / 2:
        ways.append(f"{spare[2]:.0f} mm along one side")
    sides = ("Fill the carton's spare first: " + " and ".join(ways) + ", the bundle's spare "
             "cartons cut to the carton's depth and folded flat, layer on layer "
             f"(each about {t:.0f} mm), taped into a slab, so the pack can't shift in it."
             if ways else "")
    hole = f", a hole cut through it round {kept}" if kept else ""
    n = max(1, round(spare[1] / t))
    top = ((f"Lay {'one flat sheet' if n == 1 else f'{n} flat sheets'} cut from a spare "
            f"carton over the whole top ({spare[1]:.0f} mm to the carton's top edge){hole}: "
            "the lid then presses it down onto the pack" + (", never on what the hole is round"
                                                            if kept else "") + ".")
           if spare[1] > 0.5 else "")
    return {"sides": sides, "top": top}


def _made_to_size(pack: dict, kg: float, stock: dict) -> list[str]:
    """A line for a carton made to the pack's own box (its outside, ``pack["box"]``), where
    USPS bills it less than the stock one (``stock``: its :func:`usps`): an option only, the
    stock carton being the choice (the user's, 2026-10-21: packing review 3's M6)."""
    if "outside_mm" not in pack["box"]:
        return []
    out = tuple(float(v) / IN for v in pack["box"]["outside_mm"])
    post = usps(out, kg / 0.45359237 + CARTON_LB)       # type: ignore[arg-type]
    if post["usd"][5] >= stock["usd"][5] - 0.005:
        return []
    o = " x ".join(f"{v:.1f}" for v in out)
    return [f"An option only, not the plan: a carton made to the pack's own box ({o} in "
            "outside; no stock size is, so a box maker's: cutting a stock carton down past its "
            "scores re-makes its glued joint and two panels, and its rating no longer holds) "
            "would bill " + " / ".join(f"zone {z} ${p:.2f}" for z, p in post["usd"].items())
            + f", ${stock['usd'][5] - post['usd'][5]:.2f} less a shipment at zone 5"
            + (", the stock one being Oversized (over 108 in of length and girth)"
               if stock["oversized"] else "") + "."]


USPS_MAX_IN = 130
"""USPS Ground Advantage Retail's most length and girth (DMM 101 3.2.1, fetched 2026-10-21:
"must not measure more than 130 inches in length and girth combined")."""
FOAM_IN = (2.0, 1.5, 1.0)
"""in of foam round a carton inside an outer one (a carrier's option): the most that mails."""


def carrier_option(outside_in: tuple[float, float, float], wall_in: float) -> dict | None:
    """The carton (``outside_in``) inside an outer one of ``wall_in`` board with foam all
    round: the most foam of :data:`FOAM_IN` whose length and girth (each side rounded up to
    the inch) stays within :data:`USPS_MAX_IN` (packing review 3's m7: 2 in round S-4509 at
    14 in is 131.5 in even unrounded, past it); None when none does."""
    for f in FOAM_IN:
        o = sorted((v + 2 * f + 2 * wall_in for v in outside_in), reverse=True)
        up = [math.ceil(v - 1e-9) for v in o]
        lg = up[0] + 2 * (up[1] + up[2])
        if lg <= USPS_MAX_IN:
            return {"foam_in": f, "outside_in": tuple(round(v, 2) for v in o),
                    "length_girth_in": lg}
    return None


def ship_lines(name: str, pack: dict, kg: float, why_stock: str = "", kept: str = ""
               ) -> list[str]:
    """ORDER.md's "Ship it" lines for a cradle's pack payload ``pack`` (``cradle_pack``'s:
    its box, its parts' bounds ``packed_mm``) weighing ``kg`` with its spacers: the stock
    carton (:func:`carton_of`: the parts' bounds and :data:`FIT_MM` all round, packing
    review 2's m5) and why it is the choice (``why_stock``, packing review 3's M6), how it
    is cut, filled (:func:`fill_words`; ``kept``: what the top sheet's hole is round) and
    marked, its rating, the postage, a carton made to the pack's own box as an option where
    USPS bills that less, and what the pack is for (a carrier's: :func:`carrier_option`)."""
    inside = tuple(float(v) for v in pack["box"]["inside_mm"])
    got = carton_of(pack, kg)
    lines = ["## Ship it (the carton)", ""]
    dims = " x ".join(f"{v:.0f}" for v in inside)
    if got is None:
        return [*lines, f"No stock carton in the 30 x 24 in footprint takes {name}'s pack "
                f"({dims} mm inside) at {kg / 0.45359237 + CARTON_LB:.1f} lb: a box cut to "
                "it.", ""]
    c: Carton = got["carton"]
    cut = (f", cut down at its {got['cut_to_in']:g} in score (the creased corners cut to it, "
           "the flaps folded at the crease)" if got["cut_to_in"] else "")
    spare = got["spare_mm"]
    post = got["postage"]
    o = got["outside_in"]
    fill = fill_words(got, kept)
    carrier = carrier_option(o, board_mm(c) / IN)
    lines += [
        f"{name} packs in {dims} mm (inside: its parts and their clearance; "
        f"{kg:.1f} kg with its spacers, {got['lb']:.1f} lb in its carton).", "",
        "| buy | item | board | rated (UPS, Uline's table) | each | link |",
        "|---:|---|---|---|---:|---|",
        f"| 1 of {c.per} | {c.item}, {c.inside[0]:g} x {c.inside[1]:g} x {c.inside[2]:g} in"
        f"{cut} | {c.board} | {c.rated_lb:g} lb | ${c.usd:.2f} | "
        f"[product page]({c.url}) |", "",
        f"The stock carton is the one to ship in{why_stock}.",
        f"Spare inside: {spare[0]:.0f} mm along, {spare[1]:.0f} mm over the top, {spare[2]:.0f} "
        "mm across (the pack's spacers hold every piece; the filler stops the pack shifting "
        "in the carton). " + " ".join(t for t in (fill["sides"], fill["top"]) if t)
        + " The bundle is bought whole (Uline sells by the bundle): ship one, keep the rest "
        "for the trips and the filler.",
        f"USPS Ground Advantage Retail ({o[0]:g} x {o[1]:g} x {o[2]:g} in outside: "
        f"{post['dim_lb']} lb dimensional, billed {min(post['billed_lb'], 70)} lb"
        + (", oversized" if post["oversized"] else f", fees ${post['fees']:.2f}") + "): "
        + " / ".join(f"zone {z} ${p:.2f}" for z, p in post["usd"].items())
        + " (Notice 123, 2026-10-04; the shipping isn't in the totals).",
        *_made_to_size(pack, kg, post),
        "The pack is for the car and for hand-carrying, where it rides upright and is set "
        "down: every piece rests on its plates on printed PETG spacers and is held each way "
        "(up too) to a cell, 3 mm. Close the flaps and tape them shut, and mark two opposite "
        "sides THIS SIDE UP, with arrows, and CAR OR HAND-CARRY ONLY: NOT FOR A CARRIER. A "
        "carrier drops a parcel this heavy from 30 in (FedEx's ISTA 6-FedEx-A); the spacers "
        "locate, they don't cushion, so for a carrier insure it, or "
        + (f"pack the carton inside an outer one with {carrier['foam_in']:g} in of foam all "
           "round (" + " x ".join(f"{v:g}" for v in carrier["outside_in"]) + " in outside, "
           f"{carrier['length_girth_in']} in of length and girth, within USPS's "
           f"{USPS_MAX_IN} in"
           + (f", where {FOAM_IN[0]:g} in would be past it" if carrier["foam_in"] < FOAM_IN[0]
              else "") + "; no stock Uline carton is that size: a custom one, Oversized at "
           "USPS)."
           if carrier else "ship a carton made to size inside an outer one with foam: this "
           f"one in foam is past USPS's {USPS_MAX_IN} in of length and girth."), ""]
    return lines
