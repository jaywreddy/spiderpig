"""Catalog data: generic purchasable hardware (screws, nut, insert, sheet stock, glue, filament).

Screw keys come from :mod:`hardware.fasteners` (``shcs("3", 12) ->
"m3_shcs_12"``; every family's every stock length is registered here, as
``tests/test_robot.py`` checks).

Sources: the research notes of 2026-09-29 (``joinery.json``: ISO/DIN
dimension tables, vendor pages) plus the pages cited per offer. An offer is
``verified=True`` only if its page was fetched and showed the product;
McMaster-Carr pages don't render to a fetcher, so every McMaster part number
here is unverified (most were confirmed on a mirror or category listing).
Prices are per pack and only where a page showed one.

Servo-specific items (servos, horns sold separately) live in
:mod:`servos.catalog`.

Prices added 2026-09-30 (test drive round 3): the fastener pages that render to a
fetcher (Woodcraft, Woodpeckers Crafts) are ``verified=True`` with the price seen;
Bolt Depot, TME and Amazon refuse a fetch, so their prices are what a web search's
result quoted from that page on that day, ``verified=False`` and said so in the note.
Nothing is priced from memory; an item no page priced stays unpriced (the 3 mm rod,
the M2 tapping kit, the starlock kit, M3 x 18 and x 50 SHCS).
"""

from __future__ import annotations

from spiderpig.hardware.catalog import Item, Offer, register
from spiderpig.hardware.fasteners import CLEARANCE, SCREWS, SHCS_LENGTHS, shcs

SEARCHED = ("price as a web search quoted it from this page on 2026-09-30 (the page "
            "refuses a fetch)")

# ---------------------------------------------------------------------------
# Socket head cap screws (ISO 4762 / DIN 912)
# ---------------------------------------------------------------------------

_MCM_M3_SHCS = {6: "91290A111", 8: "91290A113", 10: "91290A115", 12: "91290A117",
                16: "91290A120", 20: "91290A123", 25: "91290A125", 30: "91290A130"}
_KIT_M3 = {6: 50, 8: 30, 10: 25, 12: 20, 16: 15, 20: 10, 25: 10, 30: 10}  # B0DNM4HK5Q
_KIT_M2 = (4, 6, 8, 10, 12, 16, 20)                           # lengths in B08MT9WWCT
_MCM_SHCS_PAGE = "https://www.mcmaster.com/products/socket-head-screws/"
# Bolt Depot, metric socket cap, stainless 18-8 (A-2), 3 mm x 0.5: (product, USD per 100)
BOLT_DEPOT_M3_SHCS: dict[int, tuple[str | None, float]] = {
    6: (None, 3.83), 8: ("6379", 4.76), 12: ("6381", 5.18), 16: ("6382", 4.50),
    20: ("6383", 4.84), 30: ("6385", 6.89), 45: ("6388", 12.25),
}
_BD_LIST_M3_SHCS = ("https://boltdepot.com/Metric_socket_cap_Stainless_steel_18-8_(A-2)"
                    "_3mm_x_0.5mm")


def bolt_depot_shcs(length: float) -> Offer | None:
    """Bolt Depot's M3 socket cap of this length with the price its page quoted, or None."""
    entry = BOLT_DEPOT_M3_SHCS.get(int(length))
    if entry is None:
        return None
    product, price = entry
    url = (f"https://boltdepot.com/Product-Details?product={product}" if product
           else _BD_LIST_M3_SHCS)
    return Offer("Bolt Depot", url, product, pack_qty=100, price_usd=price,
                 note=f"stainless 18-8 (A-2), DIN 912; {SEARCHED}")


def _shcs_offers(size: str, length: float) -> tuple[Offer, ...]:
    L = int(length)
    offers: list[Offer] = []
    if size == "3":
        if (bd := bolt_depot_shcs(L)) is not None:
            offers.append(bd)
        if L in _MCM_M3_SHCS:
            pn = _MCM_M3_SHCS[L]
            offers.append(Offer("McMaster-Carr", f"https://www.mcmaster.com/{pn}/", pn,
                                pack_qty=100, note="black-oxide alloy steel 12.9; part number "
                                "confirmed on the reli-tool.com mirror"))
        else:
            offers.append(Offer("McMaster-Carr", _MCM_SHCS_PAGE, pack_qty=100,
                                note=f"M3 x {L} mm, pick from the listing"))
        if L in _KIT_M3:
            offers.append(Offer("Amazon", "https://www.amazon.com/dp/B0DNM4HK5Q", "B0DNM4HK5Q",
                                pack_qty=_KIT_M3[L],
                                note="631-pc M3 kit (6-30 mm SHCS, nuts, washers)"))
    elif size == "2":
        offers.append(Offer("McMaster-Carr", _MCM_SHCS_PAGE, pack_qty=100,
                            note=f"M2 x {L} mm, pick from the listing"))
        if L in _KIT_M2:
            offers.append(Offer("Amazon", "https://www.amazon.com/dp/B08MT9WWCT", "B08MT9WWCT",
                                pack_qty=660, note="660-pc M2 assortment (4-30 mm SHCS, nuts, "
                                "washers); count per length not stated"))
    else:  # M2.5
        if L == 8:
            offers.append(Offer("AndyMark", "https://andymark.com/products/"
                                "servo-horn-replacement-screw-m2-5-0-45-x-8-mm", "am-1496",
                                price_usd=0.25, verified=True, note="stainless, sold singly"))
        offers.append(Offer("McMaster-Carr", _MCM_SHCS_PAGE, pack_qty=100,
                            note=f"M2.5 x {L} mm, pick from the listing"))
        offers.append(Offer("Amazon", "https://www.amazon.com/clp/B07C7V6MCK", "B07C7V6MCK",
                            pack_qty=135, note="135-pc M2.5 SHCS assortment; count per "
                            "length not stated"))
    return tuple(offers)


for _size, _lengths in SHCS_LENGTHS.items():
    _sk = SCREWS["shcs", _size]
    for _L in _lengths:
        register(Item(
            shcs(_size, _L), f"M{_sk.d:g} x {_L:g} mm socket head cap screw", "fastener",
            _shcs_offers(_size, _L),
            dims={"d": _sk.d, "length": float(_L), "head_d": _sk.head_d, "head_h": _sk.head_h,
                  "clearance_d": CLEARANCE[_size]},
            notes="ISO 4762 / DIN 912",
        ))

# ---------------------------------------------------------------------------
# Button head socket screws (ISO 7380-1): short heads that fit a 3 mm layer
# ---------------------------------------------------------------------------

_sk = SCREWS["bhcs", "3"]
# Bolt Depot, metric socket button head, stainless 18-8 (A-2), 3 mm x 0.5: (product, USD/100)
BOLT_DEPOT_M3_BHCS: dict[int, tuple[str, float]] = {6: ("7218", 3.97), 10: ("7220", 4.30)}
for _L in _sk.lengths:
    _bd = BOLT_DEPOT_M3_BHCS.get(int(_L))
    register(Item(
        _sk.key(_L), f"M{_sk.d:g} x {_L:g} mm button head socket screw", "fastener",
        (*([Offer("Bolt Depot", f"https://boltdepot.com/Product-Details?product={_bd[0]}",
                  _bd[0], pack_qty=100, price_usd=_bd[1],
                  note=f"stainless 18-8 (A-2), ISO 7380; {SEARCHED}")] if _bd else []),
         Offer("McMaster-Carr", "https://www.mcmaster.com/products/button-head-screws/",
               pack_qty=100, note=f"pick M{_sk.d:g} x {_L:g} mm, ISO 7380; part "
               "number not confirmed"),
         Offer("Amazon", "https://www.amazon.com/s?k=M3+button+head+socket+screw+assortment",
               note="search: M3 ISO 7380 assortment")),
        dims={"d": _sk.d, "length": float(_L), "head_d": _sk.head_d, "head_h": _sk.head_h,
              "clearance_d": CLEARANCE["3"]},
        notes="ISO 7380-1",
    ))

# ---------------------------------------------------------------------------
# Self-tapping screws for plastic (the STS3215's case pilots are 1.6 mm, for PA2.0)
# ---------------------------------------------------------------------------

_sk = SCREWS["self_tap", "2"]
for _L in _sk.lengths:
    register(Item(
        _sk.key(_L), f"M2 x {_L:g} mm pan-head self-tapping screw (for plastic)",
        "fastener",
        (
            Offer("Amazon", "https://www.amazon.com/dp/B0GV338JJK", "B0GV338JJK", pack_qty=100,
                  verified=True, note="800-pc M2 pan-head self-tapping kit, 8 lengths x 100 "
                  "(which lengths was not visible on the page)"),
            Offer("McMaster-Carr", "https://www.mcmaster.com/products/pan-head-thread-forming-screws",
                  pack_qty=100, note="thread-forming screws for plastic, M2"),
            Offer("Amazon", "https://www.amazon.com/dp/B07N79RKTH", "B07N79RKTH", pack_qty=400,
                  verified=True, note="400-pc M2/M2.6 cross pan-head self-tapping assortment; "
                  "count per length not stated"),
        ),
        dims={"d": 2.0, "length": float(_L), "head_d": _sk.head_d, "head_h": _sk.head_h,
              "pilot_d": 1.6},
        notes="Pan head ~4.0 x 1.6 mm (ISO 7049 ST2.2 maximum); measure yours.",
    ))

# ---------------------------------------------------------------------------
# Nut and insert
# ---------------------------------------------------------------------------

register(
    Item("m3_nut", "M3 hex nut (ISO 4032)", "nut",
         (Offer("Bolt Depot", "https://www.boltdepot.com/Product-Details.aspx?product=4773", "4773",
                pack_qty=100, price_usd=2.39,
                note=f"stainless 18-8 (A-2), DIN 934; {SEARCHED}"),
          Offer("McMaster-Carr", "https://www.mcmaster.com/90592A085/", "90592A085", pack_qty=100,
                note="part number from a search result only"),
          Offer("Amazon", "https://www.amazon.com/dp/B0DNM4HK5Q", "B0DNM4HK5Q", pack_qty=140,
                note="in the 631-pc M3 kit")),
         dims={"af": 5.5, "h": 2.4, "d": 3.0}),
    Item("m3_heat_set_insert", "M3 x 5.7 brass heat-set insert (for printed parts)", "insert",
         (Offer("3DJake", "https://www.3djake.com/cnc-kitchen/threaded-inserts-m3-standard",
                "CNC Kitchen M3 standard", pack_qty=100, price_usd=11.37, verified=True),
          Offer("CNC Kitchen", "https://cnckitchen.store/products/heat-set-insert-m3-x-5-7-100-"
                "pieces", pack_qty=100, verified=True, note="EUR 9.40"),
          Offer("ruthex", "https://www.ruthex.de/en/products/ruthex-gewindeeinsatz-m3-100-stuck-"
                "rx-m3x5-7-messing-gewindebuchsen", "RX-M3x5.7", pack_qty=100, verified=True),
          Offer("McMaster-Carr", "https://www.mcmaster.com/94180A331/", "94180A331",
                note="tapered insert, 3.8 mm installed length: a different size")),
         dims={"od": 4.6, "length": 5.7, "hole_d": 4.0, "min_wall": 1.6, "d": 3.0},
         notes="Hole 4.0 mm, at least 1.6 mm of plastic around it (CNC Kitchen). "
               "Press in with a soldering iron; not for laser-cut sheet."),
)

# ---------------------------------------------------------------------------
# Sheet stock (one "sheet" = a nominal 12 x 12 in blank; layout packs onto sheet_mm)
# ---------------------------------------------------------------------------

register(
    Item("acrylic_3mm", "3 mm (1/8 in) cast acrylic sheet, 12 x 12 in", "sheet",
         (Offer("Inventables", "https://www.inventables.com/products/clear-acrylic-sheet-cast",
                "12 x 24 in 1/8 in cast", pack_qty=2, price_usd=10.99, verified=True,
                note="one 12 x 24 in sheet = two 12 x 12 in; thickness +/-8 %"),
          Offer("Amazon", "https://www.amazon.com/dp/B0DTSG32FM", "B0DTSG32FM", pack_qty=13,
                note="13 coloured 12 x 12 in cast sheets"),
          Offer("Ponoko", "https://www.ponoko.com/materials/clear-acrylic", verified=True,
                note="laser-cutting service; kerf 0.2 mm"),
          Offer("SendCutSend", "https://sendcutsend.com/materials/acrylic/", verified=True,
                note="laser-cutting service, 0.118 in acrylic")),
         dims={"thickness": 3.0, "sheet_mm": (300.0, 300.0)},
         notes="Nominal 3 mm; real sheets vary by up to about 8 %. "
               "Measure yours and pass --thickness."),
    Item("plywood_3mm", "3 mm (1/8 in) Baltic birch plywood, 12 x 12 in", "sheet",
         (Offer("Woodpeckers Crafts", "https://woodpeckerscrafts.com/products/baltic-birch-"
                "plywood-1-8-x-12-x-12", pack_qty=1, price_usd=3.10, verified=True,
                note="B/BB, sold per sheet ($2.57 each at 8 or more); page fetched "
                     "2026-09-30"),
          Offer("Amazon", "https://www.amazon.com/dp/B01N5CHME9", "B01N5CHME9", pack_qty=8,
                verified=True, note="Woodpeckers B/BB, box of 8"),
          Offer("Amazon", "https://www.amazon.com/dp/B08429ZRB1", "B08429ZRB1", pack_qty=20,
                verified=True, note="Woodpeckers 12 x 20 in, box of 20"),
          Offer("SendCutSend", "https://sendcutsend.com/materials/baltic-birch-plywood/",
                verified=True, note="laser-cutting service, 0.125 in (3.18 mm)")),
         dims={"thickness": 3.0, "sheet_mm": (300.0, 300.0)},
         notes="Real thickness 2.8-3.3 mm; measure and pass --thickness."),
)

# ---------------------------------------------------------------------------
# Adhesives and filament
# ---------------------------------------------------------------------------

register(
    Item("acrylic_cement", "SCIGRIP (Weld-On) 4 acrylic solvent cement, 4 oz", "adhesive",
         (Offer("U.S. Plastic Corp.", "https://www.usplastic.com/catalog/item.aspx?itemid=165510",
                "97559", price_usd=12.84, verified=True),
          Offer("Amazon", "https://www.amazon.com/dp/B0096T6P1Y", "B0096T6P1Y",
                note="pint with needle applicator bottle"),
          Offer("TAP Plastics", "https://www.tapplastics.com/product/repair_products/"
                "plastic_adhesives/weld_on_4_cement/465")),
         notes="Water-thin, applied by capillary with a needle bottle: 1-2 min working, "
               "3 min fixture (SCIGRIP 4 TDS). Acrylic to acrylic only."),
    Item("wood_glue", "Titebond II Premium wood glue, 8 oz (5003)", "adhesive",
         (Offer("The Home Depot", "https://homedepot.com/p/Titebond-8-oz-Titebond-II-Premium-"
                "Wood-Glue-5003/202180087", "202180087", price_usd=5.49, note=SEARCHED),
          Offer("Amazon", "https://www.amazon.com/dp/B0000223US", "B0000223US"),
          Offer("Titebond", "https://www.titebond.com/product/glues/"
                "2ef3e95d-48d2-43bc-8e1b-217a38930fa2", "5003", verified=True,
                note="manufacturer page (sizes, where to buy)"))),
    Item("ca_glue", "Medium CA (cyanoacrylate) glue, 2 oz", "adhesive",
         (Offer("Woodcraft", "https://www.woodcraft.com/products/starbond-em-150-multi-purpose-"
                "ca-glue-medium-2-oz", price_usd=13.99, verified=True,
                note="Starbond EM-150 medium, 2 oz; page fetched 2026-09-30"),
          Offer("Amazon", "https://www.amazon.com/dp/B00C32MHJU", "B00C32MHJU", verified=True,
                note="Starbond EM-150 medium, 2 oz"),
          Offer("Amazon", "https://www.amazon.com/dp/B0B4PNW7CC", "B0B4PNW7CC",
                note="Starbond thin/medium/thick + activator bundle")),
         notes="Bonds printed PLA/PETG to acrylic or plywood."),
    Item("pla_filament", "PLA filament, 1.75 mm, 1 kg spool", "filament",
         (Offer("Prusa Research", "https://www.prusa3d.com/product/prusament-pla-jet-black-1kg-nfc/",
                "Prusament PLA Jet Black 1kg", price_usd=25.49, verified=True),
          Offer("MatterHackers", "https://www.matterhackers.com/store/l/"
                "polymaker-polylite-pla-black-175mm-1kg", verified=True,
                note="Polymaker PolyLite PLA"),
          Offer("Amazon", "https://www.amazon.com/dp/B01IAVQP2C", "B01IAVQP2C",
                note="Polymaker PolyLite PLA")),
         dims={"density": 1.24, "spool_g": 1000.0, "diameter": 1.75}),
    Item("petg_filament", "PETG filament, 1.75 mm, 1 kg spool", "filament",
         (Offer("Prusa Research", "https://www.prusa3d.com/product/prusament-petg-jet-black-1kg/",
                "Prusament PETG Jet Black 1kg", price_usd=25.49, verified=True),
          Offer("MatterHackers", "https://www.matterhackers.com/store/l/"
                "polymaker-polylite-petg-black-175mm-1kg/sk/MAW5MH42", verified=True,
                note="Polymaker PolyLite PETG (1.25 g/cm3)"),
          Offer("Amazon", "https://www.amazon.com/dp/B09Q28K49W", "B09Q28K49W",
                note="Polymaker PETG")),
         dims={"density": 1.27, "spool_g": 1000.0, "diameter": 1.75}),
)

# Pivot hardware the metal-shaft constructions use (construction/pivots).
register(
    Item("bearing_mf63zz", "MF63ZZ flanged ball bearing 3 x 6 x 2.5", "bearing",
         (Offer("Amazon", "https://www.amazon.com/dp/B00GGQ62PO", "B00GGQ62PO", pack_qty=10,
                price_usd=17.67, note=f"10-pack MF63-ZZ (99MF63-ZZ-X10); {SEARCHED}"),
          Offer("Amazon", "https://www.amazon.com/dp/B08H27NJ5N", "B08H27NJ5N", pack_qty=10,
                verified=True, note="uxcell 10-pack"),
          Offer("McMaster-Carr", "https://www.mcmaster.com/57155K538/", "57155K538")),
         dims={"id": 3.0, "od": 6.0, "w": 2.5, "flange_d": 7.2, "flange_t": 0.6}),
    Item("bearing_f683zz", "F683ZZ flanged ball bearing 3 x 7 x 3", "bearing",
         (Offer("Amazon", "https://www.amazon.com/dp/B08CKJ3NMW", "B08CKJ3NMW", pack_qty=10,
                verified=True, note="uxcell 10-pack"),),
         dims={"id": 3.0, "od": 7.0, "w": 3.0, "flange_d": 8.1, "flange_t": 0.8}),
    Item("bearing_f623zz", "F623ZZ flanged ball bearing 3 x 10 x 4", "bearing",
         (Offer("Amazon", "https://www.amazon.com/dp/B07Z3CHXT5", "B07Z3CHXT5", pack_qty=10,
                verified=True, note="uxcell 10-pack"),),
         dims={"id": 3.0, "od": 10.0, "w": 4.0, "flange_d": 11.5, "flange_t": 1.0}),
    Item("bushing_gfm0304_03", "igus iglide G flange bushing 3 x 4.5 x 3 (GFM-0304-03)",
         "bushing",
         (Offer("TME", "https://www.tme.com/us/en-us/details/gfm-0304-03/plain-bearings/igus/",
                "GFM-0304-03", pack_qty=10, price_usd=5.3,
                note=f"$0.53 each at 10 or more; {SEARCHED}"),
          Offer("igus", "https://www.igus.com/iglide-ibh/flange-bearings/product-details/"
                "iglidur-g-m?artnr=GFM-0304-03", "GFM-0304-03", verified=True),
          Offer("McMaster-Carr", "https://www.mcmaster.com/2705T111/", "2705T111")),
         dims={"id": 3.0, "od": 4.5, "l": 3.0, "flange_d": 7.5, "flange_t": 0.75}),
    Item("m3_nylock", "M3 nylon-insert lock nut (DIN 985)", "nut",
         (Offer("Bolt Depot", "https://boltdepot.com/Product-Details?product=4792", "4792",
                pack_qty=100, price_usd=4.22,
                note=f"stainless 18-8 (A-2), DIN 985; {SEARCHED}"),
          Offer("McMaster-Carr", "https://www.mcmaster.com/93625A100/", "93625A100", pack_qty=100,
                note="18-8 stainless; seen on the McMaster M3 locknut listing"),
          Offer("Aspen Fasteners", "https://www.aspenfasteners.com/m3-0-5-din-985-metric-hex-"
                "nylon-insert-stop-lock-nuts-a2-stainless-steel/", "ME223", pack_qty=250,
                price_usd=36.55, verified=True, note="A2 stainless, bag of 250"),
          Offer("Amazon", "https://www.amazon.com/dp/B07KSPTYNZ", "B07KSPTYNZ", pack_qty=100)),
         dims={"af": 5.5, "h": 4.0, "d": 3.0}),
    Item("m3_washer", "M3 flat washer (DIN 125-A, 3.2 x 7 x 0.5)", "washer",
         (Offer("Bolt Depot", "https://boltdepot.com/Product-Details?product=4513", "4513",
                pack_qty=100, price_usd=1.45,
                note=f"stainless 18-8 (A-2), 3 mm metric flat washer; {SEARCHED}"),
          Offer("McMaster-Carr", "https://www.mcmaster.com/91166A210/", "91166A210", pack_qty=100,
                note="18-8 stainless, per the RepRap McMaster BOM"),
          Offer("Amazon", "https://www.amazon.com/dp/B08HJVF49P", "B08HJVF49P", pack_qty=200),
          Offer("Aspen Fasteners", "https://www.aspenfasteners.com/m3-din-125-type-a-iso-7089-"
                "7090-metric-standard-flat-washers-a2-stainless-steel/", pack_qty=6300,
                price_usd=272.85, verified=True, note="bulk only")),
         dims={"id": 3.2, "od": 7.0, "t": 0.5}),
    Item("rod_3mm_100", "3 mm stainless rod, 100 mm", "dowel",
         (Offer("Amazon", "https://www.amazon.com/dp/B082ZP313B", "B082ZP313B", pack_qty=5,
                verified=True, note="uxcell 5-pack"),),
         dims={"d": 3.0, "length": 100.0}),
    Item("starlock_3mm", "Push-on lock washer for a 3 mm shaft", "clip",
         (Offer("Amazon", "https://www.amazon.com/dp/B0B219S4FW", "B0B219S4FW", pack_qty=60,
                verified=True, note="300-pc starlock kit M3-M12 (60 x 3 mm)"),),
         dims={"shaft_d": 3.0, "od": 9.7, "h": 1.3}),
)

# Items the metal-shaft pivot constructions need (long bolts, clips); registered on import.
from spiderpig.hardware import fastener_catalog  # noqa: E402, F401  (appends to the catalog)
