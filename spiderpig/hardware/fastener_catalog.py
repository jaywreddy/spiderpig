"""Catalog data for the metal-shaft pivot constructions (:mod:`construction.pivots`).

Appended to the catalog from the end of :mod:`hardware.parts` (new items
only; the screw tables there are unchanged). Keys follow the same helpers:
``shcs("3", 45) -> "m3_shcs_45"``.

* ``m3_shcs_45`` / ``m3_shcs_50``: the lengths a bolt pillar needs through a
  quad's 36 mm stack (:data:`BOLT_LENGTHS` is every M3 SHCS length the bolt
  construction may pick).
* ``e_clip_din6799_2p3``: the grooved alternative to a push-on Starlock for
  a 3 mm shaft; listed for reference, not used (a hobbyist won't groove a
  3 mm rod).
* ``m3_nylock_thin``: ISO 10511 low nut (3.9 mm; DIN 985 is 4.0), the
  same claim.

Sources: the 2026-09-29 research notes (``joinery.json``) and the pages
cited per offer. Offers are ``verified=True`` only where the page was
fetched and showed the product; McMaster pages don't render to a fetcher.
"""

from __future__ import annotations

from spiderpig.hardware.catalog import Item, Offer, register
from spiderpig.hardware.fasteners import CLEARANCE, screw, shcs

LONG_SHCS_LENGTHS: tuple[float, ...] = (45, 50)
BOLT_LENGTHS: tuple[float, ...] = tuple(
    sorted(set(screw("shcs", "3").lengths) | set(LONG_SHCS_LENGTHS)))

_dk, _k = screw("shcs", "3").head_d, screw("shcs", "3").head_h
for _L in LONG_SHCS_LENGTHS:
    from spiderpig.hardware.parts import bolt_depot_shcs

    _bd = bolt_depot_shcs(_L)          # 45 mm is priced (Bolt Depot 6388); 50 mm is not
    register(Item(
        shcs("3", _L), f"M3 x {_L:g} mm socket head cap screw", "fastener",
        (*([_bd] if _bd is not None else []),
         Offer("McMaster-Carr", "https://www.mcmaster.com/products/socket-head-screws/",
               pack_qty=50, note=f"M3 x {_L:g} mm, pick from the listing (91290A series); part "
               "number not confirmed"),
         Offer("Amazon", f"https://www.amazon.com/s?k=M3+x+{_L:g}+mm+socket+head+cap+screw",
               note="search: M3 ISO 4762, 50-packs")),
        dims={"d": 3.0, "length": float(_L), "head_d": _dk, "head_h": _k,
              "clearance_d": CLEARANCE["3"]},
        notes="ISO 4762 / DIN 912; thread length 18 mm, plain shank above",
    ))

register(
    Item("e_clip_din6799_2p3", "E-clip DIN 6799 size 2.3 (for 3-4 mm shafts)", "clip",
         (Offer("Accu", "https://accu-components.com/us/e-clips/69319-HETC-2-3-A2", "HETC-2-3-A2",
                verified=True, note="A2 stainless; sold singly"),
          Offer("McMaster-Carr", "https://www.mcmaster.com/products/e-style-retaining-rings/",
                note="E-style external retaining ring for 3 mm shafts")),
         dims={"shaft_d": 3.0, "groove_d": 2.3, "od": 6.3, "t": 0.6},
         notes="Needs a 2.3 mm groove in the shaft: not for cut-to-length rod. OD from the "
               "DIN 6799 table for size 2.3 (Accu lists it as 2.3 x 1.94); not fetched."),
    Item("m3_nylock_thin", "M3 low nylon-insert lock nut (ISO 10511)", "nut",
         (Offer("McMaster-Carr", "https://www.mcmaster.com/products/locknuts/specifications-met~"
                "iso-10511/", note="pick M3 from the ISO 10511 listing"),
          Offer("BelMetric", "https://belmetric.com/nylon-locking-nut-low-class-4-steel-iso-10511/",
                note="class 4 steel; size picked on the page")),
         dims={"af": 5.5, "h": 3.9, "d": 3.0},
         notes="Only 0.1 mm lower than DIN 985: it needs the same two layers."),
)

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
         notes="Takes up a Chicago screw's fixed barrel length against a 3 mm layer stack."),
    Item("threadlocker_222", "Low-strength threadlocker (Loctite 222 or equivalent), 10 ml",
         "adhesive",
         (Offer("Amazon", "https://www.amazon.com/s?k=Loctite+222+10ml",
                note="search; purple, removable by hand tools on M3"),),
         notes="One drop per Chicago screw; anaerobic, cures without preload."),
    Item("bushing_gfm0405_03", "igus iglide G flange bushing 4 x 5.5 x 3 (GFM-0405-03)",
         "bushing",
         (Offer("TME", "https://www.tme.com/us/en-us/details/gfm-0405-03/plain-bearings/igus/",
                "GFM-0405-03", pack_qty=10, price_usd=23.0,
                note="$2.30 each at 10 or more, as a 2026-10-03 search quoted the page"),
          Offer("igus", "https://www.igus.com/iglide-ibh/flange-bearings/product-details/"
                "iglidur-g-m?artnr=GFM-0405-03", "GFM-0405-03", verified=True,
                note="d1 4, d2 5.5, d3 9.5, b1 3, b2 0.75 (fetched 2026-10-03)")),
         dims={"id": 4.0, "od": 5.5, "l": 3.0, "flange_d": 9.5, "flange_t": 0.75}),
)

# The PTFE-lined rod pin (construction.pivots.ptfe): a 3 x 4 mm PTFE tube cut into
# sheet-thick liners, one pressed into each link.
register(
    Item("ptfe_tube_3x4_1m", "PTFE tube 3 mm ID x 4 mm OD, 1 m", "bushing",
         (Offer("West3D", "https://west3d.com/products/bowden-ptfe-tube-4mm-od-3mm-id",
                pack_qty=1, price_usd=2.50,
                note="$2.50 per metre, as a 2026-10-03 web search quoted the page"),
          Offer("Walmart Business", "https://business.walmart.com/ip/Uxcell-3mm-ID-4mm-OD-"
                "PTFE-Tubing-Tube-Pipe-1-Meter-3-3ft-Lengh-For-3D-Printer-RepRap/565280381",
                "565280381", pack_qty=1, price_usd=7.86,
                note="uxcell 1 m, $7.86 as a 2026-10-03 web search quoted the page")),
         dims={"id": 3.0, "od": 4.0, "length": 1000.0},
         notes="Bowden tube for 1.75 mm filament; the bore runs on a 3 mm h9 rod (the tube's "
               "ID tolerance, about +/-0.05, is the liner's clearance)."),
)
