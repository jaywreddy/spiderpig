"""Catalog data for the ball-bearing pivots (:mod:`construction.pivots.ball`): the shielded
miniature deep-groove bearings bonded into the links, and their vendor models.

Appended to the catalog from the end of :mod:`hardware.parts` (new items only).

The dimensions are the ISO 15 boundary dimensions of each size (bore x OD x width), the
same from every maker; ``radial_clearance_mm`` is ISO 5753-1's normal group (CN) for a bore
of 10 mm or less, 2-13 um, the range a bearing bought without a clearance suffix comes in
(an MC3 miniature, the Japanese makers' normal, is 5-10 um: inside it). ``mass_g`` is an
estimate (:data:`FILL` of the steel annulus), for the mass model. No price is quoted
from memory: McMaster-Carr's listings refuse a scripted fetch from here, so each offer
points at McMaster's listing of that trade number, unpriced and ``verified=False``, until a
fetch with the user's own session (``spiderpig cad fetch``) or an order confirms it.

Each bearing has a vendor model registered (:func:`hardware.cad.register_part_cad`):
McMaster's STEP, unpinned (``sha256 == ""``) until a file is imported into the CAD cache
(``spiderpig cad import`` / ``fetch``). Until then, and whenever a file's bounding box
isn't the catalog's size, the bearing is drawn from the catalog dims
(:func:`construction.pivots.ball.bearing_solid`).
"""

from __future__ import annotations

from spiderpig.hardware.cad import CadRef, register_part_cad
from spiderpig.hardware.catalog import Item, Offer, register

_MCM = "McMaster-Carr"
ISO_CN_SMALL = (0.002, 0.013)
"""ISO 5753-1 normal-group radial internal clearance for a bore up to 10 mm (mm)."""


def _mcm_listing(trade: str) -> Offer:
    """McMaster's listing of one bearing trade number (the part number on it is picked by a
    fetch with the user's session; unpriced until then)."""
    return Offer(_MCM, f"https://www.mcmaster.com/products/bearings/bearing-trade-number~"
                       f"{trade.lower()}/", None, note=f"McMaster's listing of trade no. "
                       f"{trade}; part number and price not fetched (McMaster refuses a "
                       "scripted fetch)")


BEARINGS = {
    # key: (trade name, shaft (bore), OD, width, McMaster trade number)
    "bearing_mr84zz": ("MR84ZZ", 4.0, 8.0, 3.0, "MR84-2Z"),
    "bearing_mr106zz": ("MR106ZZ", 6.0, 10.0, 3.0, "MR106-2Z"),
    "bearing_608zz": ("608ZZ", 8.0, 22.0, 7.0, "608-2Z"),
    "bearing_mr84": ("MR84", 4.0, 8.0, 2.0, "MR84"),
    "bearing_mr106": ("MR106", 6.0, 10.0, 2.5, "MR106"),
}
"""The bearings the ball pivots seat: MR84ZZ on the Chicago screw's 4 mm barrel
(``ball``), MR106ZZ on the 6 mm standoff (``ball_pillar``); both 3 mm wide, a layer of
the default 3 mm sheet; 608ZZ, the skateboard bearing, a pair under an arm's turntable
(:mod:`construction.rotary`); the open MR84 and MR106 (2 and 2.5 mm), the hub's half of a
ball pair (:mod:`construction.pivots.pair`)."""

GENERIC_PACK = {"bearing_mr84zz": (10, 9.0), "bearing_mr106zz": (10, 10.0),
                "bearing_608zz": (10, 9.0), "bearing_mr84": (10, 8.0),
                "bearing_mr106": (10, 9.0)}
"""A generic pack's (quantity, USD) per bearing (uxcell and similar on Amazon, ABEC-1 to 3,
the prices an estimate of 2026-10-12, unverified): what the BOM prices them at."""


def _generic(key: str, trade: str) -> tuple[Offer, ...]:
    if key not in GENERIC_PACK:
        return ()
    n, usd = GENERIC_PACK[key]
    return (Offer("Amazon", f"https://www.amazon.com/s?k={trade}+bearing", pack_qty=n,
                  price_usd=usd, note=f"search; a generic pack of {n}, an estimate "
                                      "(unverified)"),)


FILL = 0.7
"""A bearing's mass over its solid annulus' (rings, balls, cage and shields: about 0.6 g for
an MR84ZZ against the annulus' 0.89 g): an estimate, for the mass model only."""


def _mass_g(d: float, od: float, w: float) -> float:
    import math

    return round(FILL * 7.85e-3 * math.pi / 4 * (od * od - d * d) * w, 2)


for _key, (_trade, _d, _od, _w, _mcm) in BEARINGS.items():
    register(Item(
        _key, f"{_trade} {'shielded' if _trade.endswith('ZZ') else 'open'} ball bearing "
              f"{_d:g} x {_od:g} x {_w:g} mm", "bearing",
        (*_generic(_key, _trade), _mcm_listing(_mcm)),     # the priced offer first: the BOM's
        dims={"id": _d, "od": _od, "w": _w, "radial_clearance_mm": list(ISO_CN_SMALL),
              "material": "steel", "trade": _trade, "mass_g": _mass_g(_d, _od, _w)},
        notes="ISO 15 boundary dimensions; deep groove, "
              + ("both sides metal-shielded (ZZ / 2Z)" if _trade.endswith("ZZ") else "open")
              + f". Radial clearance: ISO 5753-1 CN, {ISO_CN_SMALL[0] * 1000:g}-"
              f"{ISO_CN_SMALL[1] * 1000:g} um."))
    register_part_cad(_key, CadRef(url="", sha256="", filename=f"{_key}.step",
                                   license="McMaster-Carr CAD (for designing with the part;"
                                           " never committed)",
                                   source=f"McMaster-Carr, trade no. {_mcm}",
                                   vendor="mcmaster"))
