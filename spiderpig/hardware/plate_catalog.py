"""Plate stock for the user's mill (:mod:`spiderpig.milling`, the fab method ``milled``).

Catalogued as ``sheet`` items (:func:`materials.sheet` reads their material, thickness and
density) with no cutting service (``service`` empty): the BOM buys each as raw stock, by
the blank (:data:`BLANK_MM`, a common offcut size) the milled parts' blanks pack into. The
thicknesses are the imperial plate sizes stocked in 6061-T6 (1/8, 1/4, 3/8 in) and the
6 mm cast acrylic and polycarbonate. No price is quoted from memory: each offer names a
listing, unpriced, until a fetch or an order confirms it.

Appended to the catalog from the end of :mod:`hardware.parts` (new items only).
"""

from __future__ import annotations

from spiderpig.hardware.catalog import Item, Offer, register

IN = 25.4
BLANK_MM = (6 * IN, 6 * IN)
"""The piece each plate item is sold by (mm): a 6 x 6 in offcut."""
_OM = "https://www.onlinemetals.com/en/buy/aluminum/aluminum-plate/pid/1266"
_MCM = "https://www.mcmaster.com/products/plastic-sheets/"


def _plate(key: str, name: str, t: float, material: str, url: str, note: str,
           **dims) -> None:
    register(Item(
        key, f"{name}, {BLANK_MM[0] / IN:g} x {BLANK_MM[1] / IN:g} in piece", "sheet",
        (Offer("OnlineMetals" if "onlinemetals" in url else "McMaster-Carr", url, None,
               note=f"{note}; not fetched: unpriced"),),
        dims={"thickness": t, "material": material, "service": "", "milled": True,
              "sheet_mm": list(BLANK_MM), **dims},
        notes="Plate for the user's mill (spiderpig.milling): bought by the piece, the "
              "milled parts' blanks cut from it."))


for _frac, _t in (("1/8", IN / 8), ("1/4", IN / 4), ("3/8", 3 * IN / 8)):
    _plate(f"plate_6061_{_t:.3f}".replace(".", "p"), f"6061-T6 aluminium plate {_frac} in "
           f"({_t:.3f} mm)", round(_t, 3), "aluminium", _OM, f"6061-T6 plate {_frac} in",
           alloy="6061-T6", yield_mpa=240.0, metal=True)
_plate("plate_acrylic_6", "Cast acrylic plate 6 mm", 6.0, "acrylic", _MCM, "cast acrylic 6 mm",
       yield_mpa=50.0)
_plate("plate_pc_6", "Polycarbonate plate 6 mm", 6.0, "polycarbonate", _MCM,
       "polycarbonate 6 mm", yield_mpa=60.0)

register(Item(
    "retaining_compound_641", "Retaining compound, medium strength (Loctite 641 or "
    "equivalent), 10 ml", "adhesive",
    (Offer("McMaster-Carr", "https://www.mcmaster.com/products/retaining-compounds/", None,
           note="medium-strength retaining compound; not fetched: unpriced"),),
    dims={"ml": 10.0},
    notes="A bearing in a milled seat's slip fit: holds it, and lets it be pressed out again "
          "(641 is the removable grade)."))
