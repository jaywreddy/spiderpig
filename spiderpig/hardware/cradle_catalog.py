"""Catalog data: what a decanting cradle buys beyond the walkers' hardware.

The hand-cranked worm drive (:mod:`construction.handcrank`): the NRV040 gear reducer and its
output shaft, the riser's screws, the handle; the
cradle's light (:mod:`construction.cradle`); and its standoffs: every pillar, stand column
and tie rod is a 10 mm steel standoff made to length, MISUMI NETRF10 (M5 both ends,
``pillar_shaft_10_m5_<L>``, :func:`netrf10`), with M5 button heads and washers.
The sourcing study: ``/root/scratch/decant/sourcing2.md`` on the development box
(2026-10-10).

House rules as in :mod:`hardware.parts`: nothing priced from memory; ``verified`` only for a
page that was fetched and showed the product.
"""

from __future__ import annotations

from spiderpig.hardware.catalog import CATALOG, Item, Offer, register, register_factory

HARF_CHICAGO_M6 = {   # 18-8 plain, p-1806096: barrel length -> (variant id, SKU, sets, USD)
    10: (46703106162937, "a25051500ux1916", 20, 8.69),
    12: (46703105999097, "a25051500ux1917", 20, 8.50),
    14: (46703106687225, "a25051500ux1918", 20, 8.80),
    15: (46703106031865, "a25051500ux1919", 20, 9.97),
    16: (46703105343737, "a25051500ux1920", 20, 9.46),
    18: (46703106752761, "a25051500ux1921", 20, 9.90),
    20: (46703105868025, "a25051500ux1922", 20, 10.34),
    22: (46703106719993, "a25051500ux1923", 20, 10.64),
    25: (46703105147129, "a25051500ux1924", 20, 11.08),
    28: (46703105179897, "a25051500ux1925", 20, 11.43),
    30: (46703106195705, "a25051500ux1926", 20, 11.56),
}
"""Harfington's (uxcell) 18-8 M6 binding barrels and screws (p-1806096; the product JSON and
the variant drawing ux1920_d02 fetched 2026-10-10): barrel 8 mm, heads 13.5 mm, the barrel's
2.0 and the screw's 1.7 tall, the screw M6 x 10; 20 sets a pack. A decanting cradle's pins
(construction.pivots.chicago.CHICAGO_M6)."""
CHICAGO_M6_LENGTHS = tuple(float(L) for L in HARF_CHICAGO_M6)

BD = "https://boltdepot.com/Product-Details?product="
BD_NOTE = "18-8 (A-2); page rendered 2026-10-10 (WebFetch refused); sold singly"
M6_BHCS_BD = {8: ("6642", 0.17, 11.27), 10: ("6643", 0.18, 12.06), 12: ("6644", 0.19, 12.84),
              16: ("6645", 0.20, 13.54), 20: ("6646", 0.23, None), 25: ("6647", 0.28, None),
              30: ("6648", 0.35, 23.66), 35: ("6649", 0.38, 25.56),
              40: ("6650", 0.41, 27.44), 45: ("6651", 0.46, 30.62)}
"""Bolt Depot's 18-8 M6 button heads (the list page rendered 2026-10-10; the 20 and 25 mm
pages 2026-10-10, each price only; the 8, 10 and 12 mm from the list page rendered
2026-10-13)."""
M5_BHCS_BD = {8: ("6631", 0.12, 7.76), 10: ("6632", 0.12, 8.03), 12: ("6633", 0.13, 8.60),
              16: ("6634", 0.14, 9.60)}
"""Bolt Depot's 18-8 M5 button heads (pages rendered 2026-10-10: head 9.14-9.50 x 2.50-2.75,
3 mm hex; the pages name no standard, the head is ISO 7380's): the NETRF10 standoffs' ends."""

NETRF10_LENGTHS: tuple[float, ...] = tuple(L / 10 for L in range(150, 6001))
"""MISUMI NETRF10's lengths: 15-600 mm in 0.1 mm steps (+-0.1)."""
NETRF10_ID = 4.2            # the strength check's bore: the M5 tap drill, as if tapped through
NETRF10_YIELD = 220.0       # MPa: 1018 at its hot-rolled minimum, as NETRF6 (conservative)
NETRF10_THREAD = 10.0       # its M5 taps' depth (M x 2)
NETRF10_TIERS: tuple[tuple[float, tuple[tuple[int, float], ...]], ...] = (
    (50.0, ((1, 13.46), (5, 9.76), (10, 4.91))),
    (100.0, ((1, 13.77), (5, 9.98), (10, 5.02))),
    (150.0, ((1, 14.08), (5, 10.20), (10, 5.13))),
    (200.4, ((1, 14.38), (5, 10.42), (10, 5.24))),
    (300.0, ((1, 15.30), (5, 11.08), (10, 5.57))),
    (600.0, ((1, 15.60),)),
)
"""USD each by quantity of one length, per length band (up to the band's length): NETRF10-45,
-58, -160, -175, -200 and -255 pages rendered 2026-10-10 with their quantity tables, the
100-150 band's from the NETRF10-125, -128 and -130 pages rendered 2026-10-13 (the rear
saddle's post); the band edges and the 300-600 band's unit price from MISUMI's price API
(unverified). The discount is per part number: five of a length cost less than four."""
NETRF10_SEEN: tuple[float, ...] = (45.0, 55.5, 58.0, 125.0, 128.0, 130.0, 160.0, 175.0,
                                   200.0, 255.0, 260.0)


NETRF12_LENGTHS: tuple[float, ...] = tuple(L / 10 for L in range(250, 1001))
"""MISUMI NETRF12's lengths: 25-700 mm in 0.1 mm steps (+-0.1); stocked here to 100 mm, the
lengths priced."""
NETRF12_ID = 5.0            # the strength check's bore: the M6 tap drill, as if tapped through
NETRF12_THREAD = 12.0       # its M6 taps' depth (M x 2)
NETRF12_TIERS: tuple[tuple[float, tuple[tuple[int, float], ...]], ...] = (
    (50.0, ((1, 15.62), (5, 11.37), (10, 5.74))),
    (100.0, ((1, 16.30), (5, 11.86), (10, 5.99))),
    (130.0, ((1, 16.98), (5, 12.36), (10, 6.24), (20, 6.08))),
)
"""USD each by quantity of one length, per length band: NETRF12-30 and NETRF12-65 pages
rendered 2026-10-13 with their quantity tables (the sourcing study
``/root/scratch/decant/six/sourcing.md``); NETRF12-125, -128 and -130 the same day (the rear
saddle's post: :data:`NETRF12_POST_MAX`)."""
NETRF12_SEEN: tuple[float, ...] = (30.0, 65.0, 125.0, 128.0, 130.0)
NETRF12_POST_MAX = 130.0
"""The longest NETRF12 priced: the rear saddle's post (the pillars stay within
:data:`NETRF12_LENGTHS`)."""


def netrf12(length: float) -> str:
    """The catalog key of the NETRF12 standoff ``length`` mm long (made on first use)."""
    key = f"pillar_shaft_12_m6_{round(length, 1):g}"
    CATALOG.get(key)
    return key


def _netrf12_item(L: float) -> Item:
    pn = f"NETRF12-{L:g}"
    tiers = next(t for top, t in NETRF12_TIERS if top + 1e-9 >= L)
    return Item(
        f"pillar_shaft_12_m6_{L:g}", f"12 mm round steel standoff, {L:g} mm, tapped M6 both "
        f"ends (MISUMI {pn})", "standoff",
        (Offer("MISUMI", "https://us.misumi-ec.com/vona2/detail/110300208270/?HissuCode="
               + pn, pn, price_usd=tiers[0][1], tiers=tiers, verified=L in NETRF12_SEEN,
               note="MISUMI circular standoff, tapped both ends, configurable length: 1018 "
                    "steel, bare (oil it, keep it dry), 12 mm OD, M6 x 12 deep each end, "
                    "length +-0.1 mm in 0.1 mm steps; the discount is per length"),),
        dims={"d": 6.0, "od": 12.0, "length": L, "thread_depth": NETRF12_THREAD,
              "id": NETRF12_ID, "yield_mpa": NETRF10_YIELD},
        notes="A decanting cradle's pivot pillars and crankpin since it carries a 6 L bottle "
              "(2026-10-13): the strength check takes it as a 12 x 5.0 tube of 1018 at 220 "
              "MPa, 1.73 x the NETRF10's bending section.")


def _netrf12_of(key: str) -> Item | None:
    try:
        n = round(float(key.removeprefix("pillar_shaft_12_m6_")) * 10)
    except ValueError:
        return None
    L = n / 10
    if not 250 <= n <= round(NETRF12_POST_MAX * 10) or f"pillar_shaft_12_m6_{L:g}" != key:
        return None
    return _netrf12_item(L)


def netrf10(length: float) -> str:
    """The catalog key of the NETRF10 standoff ``length`` mm long (made on first use)."""
    key = f"pillar_shaft_10_m5_{round(length, 1):g}"
    CATALOG.get(key)
    return key


def _netrf10_item(L: float) -> Item:
    pn = f"NETRF10-{L:g}"
    tiers = next(t for top, t in NETRF10_TIERS if top + 1e-9 >= L)
    return Item(
        f"pillar_shaft_10_m5_{L:g}", f"10 mm round steel standoff, {L:g} mm, tapped M5 both "
        f"ends (MISUMI {pn})", "standoff",
        (Offer("MISUMI", "https://us.misumi-ec.com/vona2/detail/110300208270/?HissuCode="
               + pn, pn, price_usd=tiers[0][1], tiers=tiers, verified=L in NETRF10_SEEN,
               note="MISUMI circular standoff, tapped both ends, configurable length: 1018 "
                    "steel, bare (oil it, keep it dry), 10 mm OD (0/-0.1), M5 x 10 deep each "
                    "end, length +-0.1 mm in 0.1 mm steps; the discount is per length: "
                    "order the quantity the BOM says (5 cost less than 4)"),),
        dims={"d": 5.0, "od": 10.0, "length": L, "thread_depth": NETRF10_THREAD,
              "id": NETRF10_ID, "yield_mpa": NETRF10_YIELD},
        notes="A decanting cradle's pillars, stand columns and tie rods: one piece each, made "
              "to its length. The strength check takes it as a 10 x 4.2 tube of 1018 at "
              "220 MPa.")


def _netrf10_of(key: str) -> Item | None:
    try:
        n = round(float(key.removeprefix("pillar_shaft_10_m5_")) * 10)
    except ValueError:
        return None
    L = n / 10
    if not 150 <= n <= 6000 or f"pillar_shaft_10_m5_{L:g}" != key:
        return None
    return _netrf10_item(L)


register_factory("pillar_shaft_10_m5_", _netrf10_of)
register_factory("pillar_shaft_12_m6_", _netrf12_of)

register(
    Item("nrv030_30", "NRV030 worm gear reducer, 30:1, 9 mm input shaft, 14 mm hollow output "
         "(Phidgets 20 N·m 30 Series)", "gearbox",
         (Offer("Phidgets", "https://www.phidgets.com/?prodid=868", "TRM4400_0",
                price_usd=48.10, verified=True, tiers=((1, 48.10), (5, 46.18), (10, 44.25)),
                note="30:1 only. 9 mm input with 3 mm keyway; 14 mm bore with 5 mm keyway; "
                     "20 N·m; 1.8 kN radial; backlash 1/2°; in stock. Drawing: productfiles/"
                     "TRM4400/TRM4400_0/Documentation/TRM4400_0Mechanical.pdf. Ships from "
                     "Calgary, Canada: check the shipping, duties and brokerage at checkout "
                     "(not in the total); page fetched 2026-10-10"),),
         dims={"ratio": 30, "centre_distance": 30.0, "input_d": 9.0, "input_len": 20.0,
               "input_shoulder": 45.0, "bore": 14.0, "pcd": 65.0, "face_thread": "M6x11",
               "spigot_d": 55.0, "width": 58.0, "mass_g": 1200.0},
         notes="Motovario's NRV030 mesh data at 30:1: static efficiency 0.39, under the 0.5 "
               "of static irreversibility: a load on the cradle can't turn the handle. Mass "
               "about 1.2 kg (a search snippet: weigh it)."),
    Item("nrv040_30", "NRV040 worm gear reducer, 30:1, 11 mm input shaft, 19 mm hollow output "
         "(Phidgets 45 N·m 40 Series)", "gearbox",
         (Offer("Phidgets", "https://www.phidgets.com/?prodid=869", "TRM4401_0",
                price_usd=62.90, verified=True, tiers=((1, 62.90), (5, 60.38), (10, 57.87)),
                note="30:1 only. 11 mm input with 4 mm keyway; 19 mm bore with 6 mm keyway; "
                     "45 N·m; 3.5 kN radial; backlash 1/2°; 27 in stock. Ships from "
                     "Calgary, Canada: check the shipping, duties and brokerage at checkout "
                     "(not in the total); page fetched 2026-10-13"),),
         dims={"ratio": 30, "centre_distance": 40.0, "input_d": 11.0, "input_len": 23.0,
               "input_shoulder": 53.0, "bore": 19.0, "pcd": 75.0, "face_thread": "M6x11",
               "spigot_d": 60.0, "width": 73.0, "mass_g": 2100.0},
         notes="Motovario's NRV040 mesh data at 30:1: static efficiency 0.45, under the 0.5 "
               "of static irreversibility: a load on the cradle can't turn the handle. "
               "Mass: Motovario's 4.4 lb (2.0 kg) dry, about 2.1 kg with its oil. A decanting "
               "cradle's since it carries a 6 L bottle (2026-10-13; the NRV030 held a 3 L)."),
    Item("nrv040_output_shaft_19", "Single output shaft for a 40 series worm gearbox, 19 mm "
         "keyed (keys, washers, snap ring)", "misc",
         (Offer("Phidgets", "https://www.phidgets.com/?prodid=736", "TRM4006_0", price_usd=6.15,
                verified=True, tiers=((1, 6.15), (5, 5.90), (10, 5.66)),
                note="'keys, washers, and a snap ring'; 6 mm keyway; page fetched 2026-10-13"),),
         dims={"d": 19.0, "fit": "h6", "length": 128.0, "protrusion": 43.0, "collar_d": 25.0,
               "collar_t": 3.0, "key": (6.0, 6.0, 30.0), "centre_tap": "M6x15"},
         notes="Lengths as Motovario's NRV040 low-speed shaft (18 h6 x 128, B1 43); its "
               "collar and key length assumed: measure them."),
    Item("nrv030_output_shaft_14", "Single output shaft for NRV030, 14 mm keyed (keys, washers, "
         "snap ring)", "misc",
         (Offer("Phidgets", "https://www.phidgets.com/?prodid=734", "TRM4004_0", price_usd=4.45,
                verified=True, tiers=((1, 4.45), (5, 4.27), (10, 4.09)),
                note="'packaged with keys, washers, and a snap ring'; page fetched 2026-10-10"),),
         dims={"d": 14.0, "fit": "h6", "length": 102.0, "protrusion": 32.5, "collar_d": 20.0,
               "collar_t": 2.5, "key": (5.0, 5.0, 20.0), "centre_tap": "M6x15"}),
    *(Item(f"chicago_m6_{L}", f"M6 Chicago screw (binding barrel + screw), 8 mm barrel x {L} mm",
           "fastener",
           (Offer("Harfington (uxcell)",
                  f"https://www.harfington.com/products/p-1806096?variant={vid}", sku,
                  pack_qty=n, price_usd=usd, verified=True,
                  note="18-8 stainless, Phillips/slotted; product JSON and drawing fetched "
                       "2026-10-10"),),
           dims={"thread": 6.0, "barrel_d": 8.0, "head_d": 13.5, "head_h": 2.0,
                 "screw_head_h": 1.7, "screw_len": 10.0, "length": float(L)})
      for L, (vid, sku, n, usd) in HARF_CHICAGO_M6.items()),
    Item("m6_carriage_20", "M6 x 20 carriage bolt (DIN 603), zinc-plated class 4.6", "fastener",
         (Offer("Bolt Depot", BD + "18220", "18220", price_usd=0.23, verified=True,
                tiers=((1, 0.23), (100, 0.1764)),
                note="zinc-plated class 4.6 steel (Bolt Depot has no M5 and no stainless "
                     "metric carriage bolt); page rendered 2026-10-13"),),
         dims={"d": 6.0, "length": 20.0, "head_d": 16.55, "head_h": 3.88, "neck": 6.48,
               "neck_h": 4.6},
         notes="A decanting cradle's base-board bolt: through a round hole in the plywood, "
               "its square neck pulled into the wood holds it while its nut goes on from "
               "above (round 3 review: a screw's head under the assembled cradle can't be "
               "held)."),
    Item("m6_thin_nut", "M6 thin hex nut (DIN 439, 3.2 mm)", "nut",
         (Offer("Bolt Depot", BD + "13572", "13572", price_usd=0.10,
                tiers=((1, 0.10), (100, 0.0641)), verified=True,
                note="18-8 hex jam nut, 10 AF x 3.2; page rendered 2026-10-10"),),
         dims={"af": 10.0, "h": 3.2}),
    *(Item(f"m6_bhcs_{n}", f"M6 x {n} button head socket screw (ISO 7380)", "fastener",
           (Offer("Bolt Depot", BD + pn, pn, price_usd=each, verified=True,
                  tiers=((1, each),) + (((100, per100 / 100),) if per100 else ()),
                  note=BD_NOTE),),
           dims={"d": 6.0, "length": float(n), "head_d": 10.5, "head_h": 3.3})
      for n, (pn, each, per100) in M6_BHCS_BD.items()),
    *(Item(f"m5_bhcs_{n}", f"M5 x {n} button head socket screw (ISO 7380)", "fastener",
           (Offer("Bolt Depot", BD + pn, pn, price_usd=each, verified=True,
                  tiers=((1, each), (100, per100 / 100)), note=BD_NOTE),),
           dims={"d": 5.0, "length": float(n), "head_d": 9.5, "head_h": 2.75})
      for n, (pn, each, per100) in M5_BHCS_BD.items()),
    Item("m5_nut", "M5 hex nut (DIN 934), 18-8", "nut",
         (Offer("Bolt Depot", BD + "4775", "4775", price_usd=0.07,
                tiers=((1, 0.07), (100, 0.0369)), verified=True, note=BD_NOTE),),
         dims={"af": 8.0, "h": 4.0}),
    Item("m5_washer", "M5 flat washer (DIN 125, 5.3 x 10 x 1.0)", "washer",
         (Offer("Bolt Depot", BD + "4515", "4515", price_usd=0.05, verified=True,
                tiers=((1, 0.05), (100, 0.0234)), note=BD_NOTE),),
         dims={"id": 5.3, "od": 10.0, "t": 1.0}),
    Item("m6_washer", "M6 flat washer (DIN 125, 6.4 x 12 x 1.6), 18-8", "washer",
         (Offer("Bolt Depot", BD + "4516", "4516", price_usd=0.06, verified=True,
                tiers=((1, 0.06), (100, 0.0395)),
                note=BD_NOTE + "; page rendered 2026-10-13"),),
         dims={"id": 6.4, "od": 12.0, "t": 1.6}),
    Item("m6_washer_wide", "M6 fender washer (6 x 18), 18-8", "washer",
         (Offer("Bolt Depot", BD + "7315", "7315", price_usd=0.12, verified=True,
                tiers=((1, 0.12), (100, 0.0782)),
                note=BD_NOTE + "; list page rendered 2026-10-13 (no standard named)"),),
         dims={"id": 6.4, "od": 18.0, "t": 1.6},
         notes="Over the 12 mm crankpin's end, holding the rod on (as the M5's on a NETRF10)."),
    Item("m5_washer_wide", "M5 fender washer (5 x 15)", "washer",
         (Offer("Bolt Depot", BD + "7314", "7314", price_usd=0.10, verified=True,
                tiers=((1, 0.10), (100, 0.068)),
                note=BD_NOTE + "; 0.9-1.4 mm thick, ID not stated (no standard named)"),),
         dims={"id": 5.3, "od": 15.0, "t": 1.2},
         notes="Under a standoff pillar's end screw outside a frame plate, as the walkers' "
               "DIN 9021 (the only 18-8 M5 wide washer Bolt Depot lists)."),
    # the saddles' thumbscrews (construction.cradle: each V block rides up and down on one;
    # pages rendered 2026-10-13, the sourcing in /root/scratch/decant/saddles/sourcing/)
    Item("m6_tap_bolt_60", "M6 x 60 hex tap bolt, fully threaded, 18-8", "fastener",
         (Offer("Bolt Depot", BD + "5973", "5973", price_usd=0.60, verified=True,
                tiers=((1, 0.60), (100, 0.3991)),
                note="18-8 (A-2), M6-1.0 x 60, fully threaded, 10 mm across flats, head "
                     "3.85-4.15 (no standard named: DIN 933's); page rendered 2026-10-13"),),
         dims={"d": 6.0, "length": 60.0, "af": 10.0, "head_h": 4.0},
         notes="The front saddle's thumbscrew: it stands in its carrier's blind bore, a "
               "printed knob on its head, its nut in the V block."),
    Item("m6_nut", "M6 hex nut (DIN 934), 18-8", "nut",
         (Offer("Bolt Depot", BD + "4776", "4776", price_usd=0.11, verified=True,
                tiers=((1, 0.11), (100, 0.0647)),
                note="18-8, M6-1.0, DIN 934, 10 mm across flats, 4.7-5.0 high; page "
                     "rendered 2026-10-13"),),
         dims={"af": 10.0, "h": 5.0}),
    Item("m8_tap_bolt_150", "M8 x 150 hex tap bolt, fully threaded, 18-8", "fastener",
         (Offer("Bolt Depot", BD + "11110", "11110", price_usd=2.87, verified=True,
                tiers=((1, 2.87), (25, 2.208), (250, 1.944)),
                note="18-8 (A-2), M8-1.25 x 150, fully threaded, 13 mm across flats, head "
                     "5.15-5.45 (no standard named: DIN 933's); the longest Bolt Depot "
                     "stocks fully threaded; page rendered 2026-10-13"),),
         dims={"d": 8.0, "length": 150.0, "af": 13.0, "head_h": 5.3},
         notes="The rear saddle's thumbscrew: it stands in its carrier's blind bore, a "
               "printed knob on its head, the coupling nut in the V block riding it."),
    Item("m8_coupling_nut_24", "M8 coupling nut, 24 mm long (DIN 6334), 18-8", "nut",
         (Offer("Bolt Depot", BD + "27224", "27224", price_usd=2.52, verified=True,
                tiers=((1, 2.52), (10, 1.641), (100, 1.48)),
                note="18-8, M8-1.25 x 24, 13 mm across flats, DIN 6334 (the only M8 length "
                     "stocked); page rendered 2026-10-13"),),
         dims={"af": 13.0, "h": 24.0},
         notes="In the rear V block's boss: 24 mm of thread on the lead screw guides it as "
               "well as lifting it."),
    # the M6 Chicago pins' washers' sizes: what their printed head spacers and gap rings are
    # drawn to (materials.WASHERS; they are printed, not bought)
    Item("ptfe_washer_8x14x0p5", "PTFE flat washer 8.2 x 14 x 0.5 mm", "washer",
         (Offer("McMaster-Carr", "https://www.mcmaster.com/95630A899/", "95630A899",
                pack_qty=10, price_usd=8.50, verified=True,
                note="8.05 x 14.3 x 0.7-0.9 mm, the nearest stocked (page rendered "
                     "2026-10-10); MISUMI TT-0814-05 is 8.1 x 14 x 0.5 in 50s ($16.00). Not "
                     "bought by the build: it prints this face as the head spacer"),),
         dims={"id": 8.2, "od": 14.0, "t": 0.5}),
    Item("shim_din988_8x14", "DIN 988 shim ring 8 x 14 mm (0.1 / 0.2 / 0.3 / 0.5 / 1.0 mm)",
         "washer",
         (Offer("Accu", "https://accu-components.com/us/shim-washers/", "62891-62895",
                price_usd=1.85, verified=True,
                note="A2, sold singly, product ids 62891-62895 (0.1 $3.03, 0.2 $2.69, 0.3 "
                     "$3.02, 0.5 $1.85, 1.0 $1.97 each; pages rendered 2026-10-10). Not "
                     "bought by the build: its printed rings take these steps"),),
         dims={"id": 8.0, "od": 14.0, "t": (0.1, 0.2, 0.3, 0.5, 1.0)},
         notes="Sizes the printed spacers on an M6 Chicago pin's 8 mm barrel (steps, OD)."),
    # the 10 mm pillars' (NETRF10) printed gap rings' sizes: never bought
    Item("ptfe_washer_10x16x0p5", "PTFE flat washer 10.2 x 16 x 0.5 mm", "washer",
         (Offer("McMaster-Carr", "https://www.mcmaster.com/products/ptfe-washers/",
                note="not bought by the build: it prints this face as a gap ring"),),
         dims={"id": 10.2, "od": 16.0, "t": 0.5}),
    Item("shim_din988_10x16", "DIN 988 shim ring 10 x 16 mm (0.1 / 0.2 / 0.3 / 0.5 / 1.0 mm)",
         "washer",
         (Offer("Accu", "https://accu-components.com/us/shim-washers/",
                note="not bought by the build: its printed rings take these steps"),),
         dims={"id": 10.0, "od": 16.0, "t": (0.1, 0.2, 0.3, 0.5, 1.0)},
         notes="Sizes the printed rings on a cradle's 10 mm pillar (steps, OD)."),
    Item("ptfe_washer_12x18x0p5", "PTFE flat washer 12.2 x 18 x 0.5 mm", "washer",
         (Offer("McMaster-Carr", "https://www.mcmaster.com/products/ptfe-washers/",
                note="not bought by the build: it prints this face as a gap ring"),),
         dims={"id": 12.2, "od": 18.0, "t": 0.5}),
    Item("shim_din988_12x18", "DIN 988 shim ring 12 x 18 mm (0.1 / 0.2 / 0.3 / 0.5 / 1.0 mm)",
         "washer",
         (Offer("Accu", "https://accu-components.com/us/shim-washers/",
                note="not bought by the build: its printed rings take these steps"),),
         dims={"id": 12.0, "od": 18.0, "t": (0.1, 0.2, 0.3, 0.5, 1.0)},
         notes="Sizes the printed rings on a cradle's 12 mm pillar and crankpin (steps, OD)."),
    Item("penlight_aaa", "AAA LED keychain light, twist switch (Olight i3E EOS), 14 x 60.5 mm",
         "electronics",
         (Offer("Battery Junction", "https://www.batteryjunction.com/products/olight-i3e-eos",
                "I3E-EOS", price_usd=12.99, verified=True,
                note="Olight i3E EOS, 14 x 60.5 mm, 90 lm, twist head, any finish; page "
                     "fetched 2026-10-10 (any 14 mm AAA twist light of about 60 mm fits the "
                     "clip: buy it locally)"),),
         dims={"d": 14.0, "length": 60.5, "mass_g": 20.0},
         notes="The sommelier's candle: it stands in its clip behind the front saddle under "
               "the shoulder, its head (which twists to switch it on) up. A longer light's "
               "tail reaches a decanter's shoulder at rest (the Maglite Solitaire's 81 mm "
               "did). Mass estimated."),
)


# -- the steel pivot pins (construction.pivots.pivot_pin, 2026-10-13) -----------------------

PIVOT8_LENGTHS: tuple[float, ...] = tuple(L / 10 for L in range(100, 1001))
"""MISUMI's precision pivot pins, straight, retaining rings, 8 mm (CCGH8-L): the length L
between the ring grooves, 10.0-100.0 mm in 0.1 mm steps (+0.1/-0; the catalog page, 2019 US
p. 939, and the series page rendered 2026-10-13)."""
PIVOT8_NC = 2.4
"""mm: the pin past each groove's inner edge, the ring and its stub: the alteration NC2.4, the
least for D 8 (2.4-5; the standard 3 mm end, a hair over a 3.175 mm layer with the play, put
every end of B's pins in a clearance gap: 94.5 mm of stack against 77)."""
PIVOT8_TIERS: tuple[tuple[int, float], ...] = (
    (1, 24.05), (2, 22.42), (5, 21.36), (20, 20.35), (50, 19.26))
"""USD each by quantity of one part number, the NC2.4 alteration included (CCGH8-12.8-NC2.4's
page rendered 2026-10-13; CCGH8-12.8's and CCGH8-16's, without it, both 21.74 at one: the
price doesn't follow L)."""
PIVOT8_SEEN: tuple[float, ...] = (12.8,)
PIVOT8_YIELD = 640.0
"""MPa: the yield the strength check takes for CCGH8 (1045 equivalent, hardened to 40-45 HRC
min.): ISO 898-1 class 8.8's minimum (Bolt Depot's grade chart, rendered 2026-10-13), as for
any steel at its hardness or more (8.8 spans 22-32 HRC; 10.9, 940 MPa, 32-39; 12.9, 1100 MPa,
39-44): conservative, whether the pin is hardened through or only at its skin."""


def pivot_pin8(length: float) -> str:
    """The catalog key of the 8 mm pivot pin ``length`` mm between its grooves (made on first
    use)."""
    key = f"pivot_pin8_{round(length, 1):g}"
    CATALOG.get(key)
    return key


def _pivot8_item(L: float) -> Item:
    pn = f"CCGH8-{L:g}-NC{PIVOT8_NC:g}"
    return Item(
        f"pivot_pin8_{L:g}", f"8 mm hardened steel pivot pin, {L:g} mm between its retaining "
        f"rings (MISUMI {pn}, 2 E-rings included)", "dowel",
        (Offer("MISUMI", "https://us.misumi-ec.com/vona2/detail/110300095320/?HissuCode=" + pn,
               pn, price_usd=PIVOT8_TIERS[0][1], tiers=PIVOT8_TIERS,
               verified=L in PIVOT8_SEEN,
               note="MISUMI precision pivot pin, straight, retaining rings: 1045 carbon steel "
                    "equivalent hardened to 40-45 HRC, black oxide, 8 mm g6 (-0.005/-0.014), L "
                    "between the grooves +0.1/-0 in 0.1 mm steps, 2.4 mm past each (the "
                    "alteration NC2.4; 3 mm standard), two E-type retaining rings No. 7 "
                    "(spring steel) included; the discount is per part number"),),
        dims={"barrel_d": 8.0, "head_d": 14.0, "head_h": PIVOT8_NC,
              "screw_head_h": PIVOT8_NC, "thread": 0.0, "length": L, "ring_t": 0.8,
              "groove_w": 0.9, "groove_d": 7.0, "yield_mpa": PIVOT8_YIELD},
        notes="A decanting cradle's link pin: the links turn on the pin between its two "
              "E-rings (construction.pivots.pivot_pin).")


def _pivot8_of(key: str) -> Item | None:
    try:
        n = round(float(key.removeprefix("pivot_pin8_")) * 10)
    except ValueError:
        return None
    L = n / 10
    if not 100 <= n <= 1000 or f"pivot_pin8_{L:g}" != key:
        return None
    return _pivot8_item(L)


register_factory("pivot_pin8_", _pivot8_of)

# -- the quick-release pins (construction.pivots.clevis, 2026-10-13) --------------------------

MCM = "https://www.mcmaster.com/"
CLEVIS8 = {15.0: ("90378A701", 5.47), 25.0: ("90378A704", 9.76), 30.0: ("90378A705", 10.31),
           44.0: ("90378A706", 11.00)}
"""McMaster's 18-8 clevis pins, 8 mm (-0.050/-0.010), head 14 x 3, a cross hole for a 2 mm
cotter at the usable length (the head's underside to the hole's near edge), 5 mm short of
the end: usable length -> (part, USD each) (the 8 mm listing rendered 2026-10-13)."""
CLEVIS8_LENGTHS = tuple(CLEVIS8)
CLEVIS8_YIELD = 205.0
"""MPa: the yield the strength check takes for an 18-8 clevis pin: ASTM A276's minimum for
annealed 304 (McMaster states only Rockwell B80, a cold-worked 18-8's hardness: more)."""


def clevis_pin8(usable: float) -> str:
    """The catalog key of the 8 mm clevis pin ``usable`` mm under its head to its hole."""
    return f"clevis_pin8_{usable:g}"


register(
    *(Item(clevis_pin8(U), f"8 mm clevis pin, 18-8 stainless, {U + 5:g} mm long, {U:g} mm "
           "usable to its 2 mm cotter hole (head 14 x 3)", "dowel",
           (Offer("McMaster-Carr", MCM + pn + "/", pn, price_usd=usd, verified=True,
                  note="18-8 stainless clevis pin, 8 mm (-0.050/-0.010), head 14 x 3 mm, for "
                       "a cotter up to 2 mm; Rockwell B80; the 8 mm listing rendered "
                       "2026-10-13 (https://www.mcmaster.com/products/clevis-pins/"
                       "diameter~8-mm/)"),),
           dims={"barrel_d": 8.0, "head_d": 14.0, "head_h": 3.0, "screw_head_h": 0.0,
                 "thread": 0.0, "length": U, "overall": U + 5.0, "hole_d": 2.0,
                 "yield_mpa": CLEVIS8_YIELD},
           notes="A decanting cradle's quick-release link pin (construction.pivots.clevis): "
                 "its head inside the stack, a hairpin cotter through its hole from the bay.")
      for U, (pn, usd) in CLEVIS8.items()),
    Item("hairpin_cotter_2x50", "Hairpin cotter pin (R-clip), 2 mm wire, 50 mm, for a 6-11 mm "
         "clevis pin", "clip",
         (Offer("McMaster-Carr", MCM + "98350A131/", "98350A131", pack_qty=5, price_usd=3.68,
                verified=True,
                note="zinc-plated spring steel hairpin cotter (R-clip): for a 6 to 11 mm "
                     "clevis pin, 2 mm wire, 10 mm eye, 50 mm long; pack of 5; the hairpin "
                     "cotter listing rendered 2026-10-13 (https://www.mcmaster.com/products/"
                     "hitch-pin-clips/)"),),
         dims={"wire": 2.0, "eye": 10.0, "length": 50.0},
         notes="Through a clevis pin's cross hole by hand, its bent leg over the pin: the "
               "quick-release pins' retainer (construction.pivots.clevis)."),
    Item("m6_carriage_25", "M6 x 25 carriage bolt (DIN 603), zinc-plated class 4.6", "fastener",
         (Offer("Bolt Depot", BD + "18221", "18221", price_usd=0.26, verified=True,
                tiers=((1, 0.26), (100, 0.1982)),
                note="zinc-plated class 4.6 steel (Bolt Depot has no stainless metric "
                     "carriage bolt); page rendered 2026-10-13"),),
         dims={"d": 6.0, "length": 25.0, "head_d": 16.55, "head_h": 3.88, "neck": 6.48,
               "neck_h": 4.6},
         notes="A decanting cradle's base-board bolt (as m6_carriage_20), long enough for a "
               "thumb nut and a thread past it."),
    Item("m6_thumb_nut", "M6 low-profile knurled-head thumb nut (DIN 467 form), head 24 x 5, "
         "black-oxide steel", "nut",
         (Offer("McMaster-Carr", MCM + "92815A203/", "92815A203", price_usd=1.69, verified=True,
                note="black-oxide steel, M6 x 1, knurled head 24 mm x 6 mm overall, collar 12 x "
                     "1; the metric thumb-nut listing rendered 2026-10-13 (Bolt Depot lists no "
                     "metric thumb nut; its M6 wing nuts, 6898 and 29213, span 28-33 mm and "
                     "would hit the bracket's upright 14 mm off the bolt)"),),
         dims={"d": 6.0, "head_d": 24.0, "h": 6.0, "collar_d": 12.0, "collar_h": 1.0},
         notes="On a decanting cradle's base-board bolts, by hand: the side panels lift off "
               "the board with no tool (cradle._bracket)."),
    Item("m3_thumb_screw_9", "M3 x 9 knurled-head thumb screw, 18-8 (head 8 x 5)", "fastener",
         (Offer("McMaster-Carr", MCM + "92545A115/", "92545A115", price_usd=1.76,
                verified=True,
                note="18-8 stainless, M3 x 0.5, 9 mm under the knurled 8 x 5 mm head, "
                     "partially threaded; the metric stainless knurled-head thumb screw "
                     "listing rendered 2026-10-13"),),
         dims={"d": 3.0, "length": 9.0, "head_d": 8.0, "head_h": 5.0},
         notes="Down through a decanting cradle's hand crank's hub onto the input shaft's key: "
               "the hand crank comes off with no tool (construction.handcrank)."),
)

register(
    Item("e_ring_7", "E-type retaining ring No. 7 (JIS B 2804, for an 8-11 mm shaft), spring "
         "steel", "clip",
         (Offer("MISUMI", "https://us.misumi-ec.com/vona2/detail/110300258420/?HissuCode=NETW7",
                "NETW7", price_usd=0.34, verified=True,
                note="spring steel 44-53 HRC, trivalent chromate; groove 7 mm, 14 mm across, "
                     "0.8 mm thick; page rendered 2026-10-13 (two come with each pivot pin: a "
                     "spare only)"),),
         dims={"d": 7.0, "od": 14.0, "t": 0.8}),
)
