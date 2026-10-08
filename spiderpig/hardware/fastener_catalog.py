"""Catalog data for the link pins (:mod:`construction.pivots.chicago`): the Chicago screws,
their washers and shims, the threadlocker.

Appended to the catalog from the end of :mod:`hardware.parts` (new items only). (The
rod, bolt, bushing and PTFE-lined pins' items went with them on 2026-10-07.)

Sources: the 2026-09-29 research notes (``joinery.json``) and the pages
cited per offer. Offers are ``verified=True`` only where the page was
fetched and showed the product; McMaster pages don't render to a fetcher.
"""

from __future__ import annotations

from spiderpig.hardware.catalog import Item, Offer, register

# ---------------------------------------------------------------------------
# Chicago screws (binding barrels and screws, "sex bolts") and what goes with them
# (construction.pivots.chicago). Searched 2026-10-03; vendor pages that refuse a
# fetch are quoted as the search result showed them and stay ``verified=False``.
# ---------------------------------------------------------------------------

CHICAGO_LENGTHS: tuple[float, ...] = (4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 18, 20, 22, 23,
                                      25, 28, 30, 32, 33, 35, 38, 40, 43, 45, 48, 50, 55, 60,
                                      65, 70, 75, 80)
"""Barrel lengths under the head (mm) of the M3 sets: Harfington's black zinc-plated 18-8
series (p-1788004; every M3 variant fetched 2026-10-05), whose plain 18-8 series
(p-1528133, preferred: hardware.sources) has a subset: 1 mm steps from 4 to 16 mm, then 18,
20, 22, 23, 25, 28 and on to 80."""




def chicago(length: float) -> str:
    return f"chicago_m3_{length:g}"


for _L in CHICAGO_LENGTHS:
    register(Item(
        chicago(_L), f"M3 Chicago screw (binding barrel + screw), 4 mm barrel x {_L:g} mm",
        "fastener",
        (Offer("Harfington", "https://www.harfington.com/products/p-1528133", verified=True,
               note="uxcell's 18-8 M3 binding barrels and screws (the per-length variants: "
                    "hardware.sources)"),),
        dims={"thread": 3.0, "barrel_d": 4.0, "head_d": 8.5, "head_h": 1.9,
              "screw_head_h": 1.4, "length": float(_L)},
        notes="Harfington's drawings (2026-10-05): barrel 4.0 OD, M3 inside, heads 8.5 mm; the "
              "plain series' barrel head 1.9 mm and screw head 1.4 mm tall, the black series' "
              "1.3 and 1.3 (the item models the taller); the screw's thread 5 mm. The heads "
              "are domed at the rim: measure a sample. McMaster sells no M3 (M4 and up).",
    ))

register(
    Item("ptfe_washer_4x8x0p5", "PTFE flat washer 4.2 x 7 x 0.5 mm", "washer",
         (Offer("McMaster-Carr", "https://www.mcmaster.com/products/ptfe-washers/",
                note="PTFE washers for M4 / #8, 0.5 mm; part number not confirmed"),
          Offer("Amazon", "https://www.amazon.com/s?k=PTFE+flat+washer+M4+0.5mm",
                note="search; uxcell lists nylon 8 x 4 mm washers (B07MXB78ZN) and PTFE in "
                     "other sizes; acetal (POM) 4 x 8 x 0.5 shims are an equal substitute")),
         dims={"id": 4.2, "od": 7.0, "t": 0.5},   # MISUMI TT-0407-05: no 4.2 x 8 x 0.5 exists
         notes="The thrust face between a link and a Chicago screw's head: PTFE on acrylic "
               "and steel, mu about 0.05-0.1."),
    Item("shim_din988_4x8", "DIN 988 shim ring 4 x 8 mm (0.1 / 0.2 / 0.3 / 0.5 / 1.0 mm)",
         "washer",
         (Offer("Accu", "https://accu-components.com/us/shim-washers/", note="DIN 988 shim "
                "rings, 4 x 8 in 0.1-1.0 mm; part number per thickness not confirmed"),
          Offer("McMaster-Carr", "https://www.mcmaster.com/products/shims/",
                note="ring shims for 4 mm shafts")),
         dims={"id": 4.0, "od": 8.0, "t": (0.1, 0.2, 0.3, 0.5, 1.0)},
         notes="The 4 x 8 family's steps: a pillar's clamped shims (bought per thickness, "
               "the 1.0 and 0.5 mm as stock 0.5 mm washers: bom.SHIM_AS); a Chicago screw's "
               "take-up uses its steps for a printed spacer."),
    Item("threadlocker_222", "Low-strength threadlocker (Loctite 222 or equivalent), 10 ml",
         "adhesive",
         (Offer("Amazon", "https://www.amazon.com/s?k=Loctite+222+10ml",
                note="search; purple, removable by hand tools on M3"),),
         notes="One drop per Chicago screw; anaerobic, cures without preload."),
)

