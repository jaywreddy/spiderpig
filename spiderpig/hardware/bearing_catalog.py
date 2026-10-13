"""Catalog data for the ball-bearing pivots (:mod:`construction.pivots.ball`): the shielded
miniature deep-groove bearings bonded into the links, and their vendor models.

Appended to the catalog from the end of :mod:`hardware.parts` (new items only).

The dimensions are the ISO 15 boundary dimensions of each size (bore x OD x width), the
same from every maker; ``radial_clearance_mm`` is ISO 5753-1's normal group (CN) for a bore
of 10 mm or less, 2-13 um, the range a bearing bought without a clearance suffix comes in
(an MC3 miniature, the Japanese makers' normal, is 5-10 um: inside it). ``mass_g`` is an
estimate (:data:`FILL` of the steel annulus), for the mass model. No price is quoted
from memory. The BOM's offer (the first) is a generic 10-pack on Amazon (:data:`AMAZON`,
read 2026-10-12: P0 / ABEC-1, the clearance not stated), then VXB's (:data:`VXB`), then
McMaster's listing of the trade number, unpriced and ``verified=False`` (its listings refuse
a scripted fetch) until a fetch with the user's own session (``spiderpig cad fetch``) or an
order confirms it.

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

VXB = {
    # key: (url, SKU, pack, USD): VXB's listings as read on 2026-10-12 (a "sale" and a
    # "regular" price both shown: the sale one)
    "bearing_mr84zz": ("https://vxb.com/products/4x8-shielded-4x8x3-mm-miniature-bearing-pack",
                       "MR84ZZ", 10, 29.99),
    "bearing_mr84": ("https://vxb.com/products/mr84-open-bearing-4x8x2-miniature", "Kit12299",
                     1, 7.77),
    "bearing_mr106zz": ("https://vxb.com/products/6x10-shielded-6x10x3-miniature-bearing-pack-"
                        "of-10", "MR106ZZ", 10, 29.99),
    "bearing_mr106": ("https://vxb.com/products/10-pack-6x10x2-5mm-ball-bearing-miniature",
                      "kit848", 10, 29.99),
    "bearing_608zz": ("https://vxb.com/products/608zz-bearing-8x22x7-shielded", "608ZZ10", 10,
                      8.88),
}
"""VXB's offer per bearing (a bearing house; the open MR106 its 2.5 mm one, not the 2 mm
listed as "MR106"): the alternative to the generic packs (:data:`AMAZON`), which run several
times cheaper at the cost of a stated clearance class."""

AMAZON = {
    # key: ((ASIN, pack, USD, title), ...): Amazon search results read 2026-10-12 (delivered
    # to Pittsburgh), the first the BOM's
    "bearing_mr84zz": (("B0GWRGY86Z", 10, 7.49, "uxcell MR84ZZ 4 x 8 x 3 mm"),),
    "bearing_mr84": (("B0FGJMHY3Y", 10, 7.59, "MR84 open 4 x 8 x 2 mm"),),
    "bearing_mr106": (("B07RVJZZH9", 5, 7.69, "uxcell MR106 open 6 x 10 x 2.5 mm"),
                      ("B0HKNYLBXN", 10, 10.84, "MR106 (L-1060) open 6 x 10 x 2.5 mm")),
    "bearing_mr106zz": (("B0C4PGLSCY", 10, 7.49, "ACROPIX MR106ZZ 6 x 10 x 3 mm"),),
}
"""The BOM's offer per miniature bearing (the first): a generic pack (uxcell-type, P0 /
ABEC-1; the radial clearance not stated: a pair preloaded by its disc spring takes up the
play, measure a sample's)."""

FREE_CAD = {
    # key: (url, sha256, license): a model anyone can download (pinned: a changed file is
    # not used); the rest only through McMaster's CAD and the user's session
    "bearing_mr84zz": ("https://raw.githubusercontent.com/tom-ilan/cycloidal_gearbox/main/"
                       "cad_models/dependencies/Micro_Steel_Deep_Groove_Ball_Bearings_3D_Model/"
                       "MR84zz-EA010/MR84zz-EA010.stp",
                       "b07f947bc1d600993db52f77692b68db12a281ad2c6eb477f70e4ce8a7462746",
                       "no license stated (a public repo): for designing with only"),
    "bearing_mr106zz": ("https://raw.githubusercontent.com/tom-ilan/cycloidal_gearbox/main/"
                        "cad_models/dependencies/Micro_Steel_Deep_Groove_Ball_Bearings_3D_Model/"
                        "MR106-EA017/MR106-EA017.stp",
                        "21aa6deb3f55ab2097df7e154bf9d0d4fe5ba1fbb62c9d31594ebba194872397",
                        "no license stated (a public repo): for designing with only"),
    "bearing_608zz": ("https://raw.githubusercontent.com/FreeCAD/FreeCAD-library/master/"
                      "Mechanical%20Parts/Bearings/608ZZ_Ball_Bearing.step",
                      "45246a4cf1bc63c8186ad8de22a74f44a63aecb450294356592c321bf281e3d3",
                      "CC-BY 3.0 (the FreeCAD library)"),
}
"""Free models of the bearings, verified against the catalog's size by
:func:`hardware.cad.load_axisymmetric` (2026-10-12); ``spiderpig cad download`` fetches
them."""


def _generic(key: str, trade: str) -> tuple[Offer, ...]:
    amazon = tuple(Offer("Amazon", f"https://www.amazon.com/dp/{asin}", asin, pack_qty=n,
                         price_usd=usd, note=f"{title}, {n} a pack (P0 / ABEC-1, clearance "
                                             "not stated); search-result price 2026-10-12")
                   for asin, n, usd, title in AMAZON.get(key, ()))
    if key not in VXB:
        return amazon
    url, sku, n, usd = VXB[key]
    return (*amazon, Offer("VXB", url, sku, pack_qty=n, price_usd=usd,
                           note=f"{trade}, {n} a pack; read on 2026-10-12"))


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
    if _key in FREE_CAD:
        _url, _sha, _lic = FREE_CAD[_key]
        register_part_cad(_key, CadRef(url=_url, sha256=_sha, filename=f"{_key}.step",
                                       license=_lic, source=_url))
        continue
    register_part_cad(_key, CadRef(url="", sha256="", filename=f"{_key}.step",
                                   license="McMaster-Carr CAD (for designing with the part;"
                                           " never committed)",
                                   source=f"McMaster-Carr, trade no. {_mcm}",
                                   vendor="mcmaster"))
