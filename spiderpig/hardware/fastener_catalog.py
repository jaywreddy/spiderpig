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

DISC_SPRINGS = {
    # key: (OD, ID, material t, free height h0, load at 75 % of h0 - t (N, estimate))
    "disc_spring_din2093_c8": (8.0, 4.2, 0.2, 0.45, 39.0),
    "disc_spring_din2093_c12p5": (12.5, 6.2, 0.35, 0.8, 151.0),
}
"""DIN 2093 series C disc springs: the preload of a ball pair's column
(:mod:`construction.pivots.pair`): the C8 on a 4 mm pin, the C12.5 on a 6 mm shaft,
compressed by the shims under them to the column's preload. Loads at 75 % of the cone's
travel (``h0`` is the free height, the cone ``h0 - t``): DIN 2093's table (39 N and 151 N;
Lesjöfors' stainless C8 36 N), checked 2026-10-12."""

for _k, (_od, _id, _t, _h0, _f) in DISC_SPRINGS.items():
    register(Item(
        _k, f"DIN 2093 series C disc spring {_od:g} x {_id:g} x {_t:g} mm", "spring",
        (Offer("Lesjöfors", "https://shop.lesjofors.com/fi/en-US/all-products/disc-springs-"
               "washers/", f"DSS {_od:g}X{_id:g}X{_t:g}", pack_qty=1,
               price_usd=2.6 if _od < 10 else 3.5,
               note="stainless 1.4310; EUR 5.16 each, 2.35 from 25 (the C8, read 2026-10-12); "
                    "the C12.5 an estimate"),
         Offer("Amazon", f"https://www.amazon.com/s?k=DIN+2093+disc+spring+{_od:g}x{_id:g}",
               pack_qty=10, price_usd=8.0, note="search; packs of 10-20, an estimate"),
         Offer("McMaster-Carr", "https://www.mcmaster.com/metric-belleville-springs",
               note=f"metric Belleville (disc) springs {_id:g} mm ID; part number not "
                    "confirmed")),
        # "t": its installed height with a 0.1 mm shim over it (the slot a ground pin's
        # end keeps for it: construction.pivots.pair.GroundShaft)
        dims={"od": _od, "id": _id, "t_mat": _t, "h0": _h0, "load_n": _f,
              "t": round(_t + 0.25 * (_h0 - _t) + 0.1 + 0.15, 2)},
        notes=f"About {_f:g} N at 75 % of its {_h0 - _t:.2f} mm travel (DIN 2093 series C, "
              "an estimate)."))

GROUND_LENGTHS = tuple(round(5.0 + 0.5 * i, 1) for i in range(111))
"""The ground pin's lengths (mm): cut to order in 0.5 mm steps (5 to 60)."""


def ground_pin(length: float) -> str:
    return f"shaft_g6_4mm_{length:g}"


for _L in GROUND_LENGTHS:
    register(Item(
        ground_pin(_L), f"Ground shaft 4 mm g6 x {_L:g} mm, both ends tapped M2", "fastener",
        (Offer("MISUMI", "https://us.misumi-ec.com/vona2/selector/rotary-shafts/"
               "tapped-rotary-shafts", price_usd=6.0,
               note="rotary shaft, one or both ends tapped, length to order; whether a 4 mm "
                    "shaft takes M2 both ends and the price are estimates (unverified)"),),
        dims={"thread": 2.0, "barrel_d": 4.0, "head_d": 5.0, "head_h": 2.3,
              "screw_head_h": 2.3, "length": float(_L), "tolerance": "g6 (-4/-12 um)"},
        notes="The pin of a preloaded ball pair (construction.pivots.pair): an M2 x 4 socket "
              "head and a 2.2 x 5 washer at each end (head 2.0 + washer 0.3 mm); the "
              "washer's rim holds the stack, on the inner rings' land. MISUMI SFJW4 / "
              "PSFJW4-L-M2-N2 (a 4 mm shaft takes M2 both ends, its 2019 catalogue); its "
              "standard lengths start at 20 mm: a shorter pin is a special, or cut from a "
              "20 mm one and tapped. Price only in MISUMI's configurator: $6 an estimate."))

for _k, (_od, _id) in {"collar_5p5x4p1": (5.5, 4.1), "collar_7x6p1": (7.0, 6.1)}.items():
    register(Item(
        _k, f"Precision collar (spacer tube) {_od:g} OD x {_id:g} ID, cut to length", "spacer",
        (Offer("MISUMI", "https://my.misumi-ec.com/vona2/detail/110310886139/", price_usd=4.0,
               note="precision collar, OD in 0.5 mm and length in 0.1 mm steps; the price an "
                    "estimate (MISUMI's configurator)"),),
        dims={"od": _od, "id": _id},
        notes="A ball pair's column spacer (construction.pivots.pair.stock_tubes): one per "
              "gap between neighbouring links (or a hub), cut to the faces' distance; its OD "
              "bears on the bearings' inner rings only (the 7 mm one passes the drive "
              "pulleys' 8 mm bores on the shoulder's pillar)."))

for _k, (_id, _od) in {"shim_4x6x0p5": (4.0, 6.0), "shim_6x8x0p5": (6.0, 8.0)}.items():
    register(Item(
        _k, f"Stainless shim washer {_id:g} x {_od:g} x 0.5 mm", "washer",
        (Offer("Amazon", f"https://www.amazon.com/s?k={_id:g}x{_od:g}x0.5mm+shim+washer",
               pack_qty=50, price_usd=7.0, note="search; precision shim washers in packs, an "
                                               "estimate"),),
        dims={"id": _id, "od": _od, "t": 0.5},
        notes="A ball pair's inner-ring land (construction.pivots.pair): it bears on the "
              "bearing's inner ring only, so a wider ring or spacer beside the link doesn't "
              "carry the column's preload around the balls."))

register(Item("m2_washer_din125", "DIN 125 flat washer M2 (2.2 x 5 x 0.3 mm)", "washer",
              (Offer("Amazon", "https://www.amazon.com/s?k=M2+flat+washer+DIN+125",
                     pack_qty=100, price_usd=6.0, note="search; an estimate"),),
              dims={"id": 2.2, "od": 5.0, "t": 0.3}))

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

