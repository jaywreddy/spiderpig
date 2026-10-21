"""Catalog data: what a decanting cradle buys beyond the walkers' hardware.

The hand-cranked worm drive (:mod:`construction.handcrank`): the NRV050 gear reducer and its
output shaft (the NRV040's kept for the record), the riser's studs and nuts, the handle and
its clutch; the
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
SERVOCITY = "https://www.servocity.com/"
BD_NOTE = "18-8 (A-2); page rendered 2026-10-10 (WebFetch refused); sold singly"
M6_BHCS_BD = {8: ("6642", 0.17, 11.27), 10: ("6643", 0.18, 12.06), 12: ("6644", 0.19, 12.84),
              16: ("6645", 0.20, 13.54), 20: ("6646", 0.23, None), 25: ("6647", 0.28, None),
              30: ("6648", 0.35, 23.66), 35: ("6649", 0.38, 25.56),
              40: ("6650", 0.41, 27.44), 45: ("6651", 0.46, 30.62)}
"""Bolt Depot's 18-8 M6 button heads (the list page rendered 2026-10-10; the 20 and 25 mm
pages 2026-10-10, each price only; the 8, 10 and 12 mm from the list page rendered
2026-10-13). Their pages name no property class ("Grade 18-8 / Also known as A-2"; the 6644
and 6645 pages rendered 2026-10-15), and Bolt Depot's grade chart gives A-2's yield as "210
Min.": A2-50's. The cradle's posts and tie rods take :data:`M6_BHCS_A270` instead."""
M6_BHCS_A270 = {12: ("MB2550012A20000", 0.29), 16: ("MB2550016A20000", 0.37),
                20: ("MB2550020A20000", 0.40)}
"""Fastenal's A2-70 M6 button heads (each page rendered 2026-10-15: "Grade A2-70",
"Standards ISO 7380-1, ISO 3506-1", head 10.07-10.50 x 3.00-3.30 mm, 4 mm drive; USD each):
the class the posts' and the tie rods' end joints need once a jam gaps them (design review
round 2's m2: prying at 4-6 kN, A2-70's proof 9.0 kN; an unclassed 18-8's 210 MPa would yield).
Their M6 lengths in the cradle (12, 16, 20) buy here; the others stay Bolt Depot's."""
FASTENAL = "https://www.fastenal.com/product/details/"
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
    (150.0, ((1, 16.98), (5, 12.36), (10, 6.24), (20, 6.08))),
    (200.4, ((1, 18.00), (5, 13.11), (10, 6.62))),
    (300.0, ((1, 21.07), (5, 15.33), (10, 7.73), (20, 7.54), (51, 7.08))),
)
"""USD each by quantity of one length, per length band: NETRF12-30 and NETRF12-65 pages
rendered 2026-10-13 with their quantity tables (the sourcing study
``/root/scratch/decant/six/sourcing.md``); NETRF12-125, -128 and -130 the same day (the rear
saddle's post: :data:`NETRF12_POST_MAX`); the 100.1-150, 150.1-200.4 and 200.5-300 bands
from the NETRF12-129.9, -130, -150, -150.1, -200, -200.5, -274.9 and -300 pages rendered
2026-10-15 (the tie rods, design review round 2's M1: NETRF12-274.9 USD 21.07 at 1-4, 15.33
at 5-9, 7.73 at 10-19, against NETRF10-274.9's 15.30, 11.08 and 5.57; the band edges at 100
and 200.4 from MISUMI's price API)."""
NETRF12_SEEN: tuple[float, ...] = (30.0, 65.0, 125.0, 128.0, 129.9, 130.0, 150.0, 150.1, 200.0,
                                   200.5, 274.9, 300.0)
NETRF12_POST_MAX = 130.0
"""The rear saddle's post's longest NETRF12 (the pillars stay within
:data:`NETRF12_LENGTHS`)."""
NETRF12_PRICED_MAX = 300.0
"""The longest NETRF12 priced (:data:`NETRF12_TIERS`): the tie rods' 274.9 mm."""


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
              "(2026-10-13), its cradle box's tie rods since design review round 2 "
              "(2026-10-15): the strength check takes it as a 12 x 5.0 tube of 1018 at 220 "
              "MPa, 1.73 x the NETRF10's bending section.")


def _netrf12_of(key: str) -> Item | None:
    try:
        n = round(float(key.removeprefix("pillar_shaft_12_m6_")) * 10)
    except ValueError:
        return None
    L = n / 10
    if not 250 <= n <= round(NETRF12_PRICED_MAX * 10) or f"pillar_shaft_12_m6_{L:g}" != key:
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
    Item("standoff_15_m8_nickel_150", "15 mm round steel standoff, 150 mm, tapped M8 both "
         "ends, electroless nickel plated (MISUMI PETR15-150)", "standoff",
         (Offer("MISUMI", "https://us.misumi-ec.com/vona2/detail/110300208270/?HissuCode="
                "PETR15-150", "PETR15-150", price_usd=19.25, verified=True,
                tiers=((1, 19.25), (5, 14.00), (10, 7.05), (20, 6.88), (50, 6.47)),
                note="the NETRF12's series (circular standoffs, tapped both ends), a stocked "
                     "length: low carbon steel, electroless nickel plating, M8 both ends, "
                     "wrench flats 13 mm across at its ends (SC 10); page rendered 2026-10-15 "
                     "(the same day: PETRF15-130, no flats, 29.43; PETRF12-130 21.79; the bare "
                     "NETRF12-130, round 1's post, 16.98)"),),
        dims={"d": 8.0, "od": 15.0, "length": 150.0, "flats": 13.0, "flat_length": 10.0,
              "yield_mpa": NETRF10_YIELD},
        notes="The rear saddle's post (construction.cradle.POST; review round 2): the V "
              "block's boss slides on it in a wine cradle, so plated (bare steel rusts, and "
              "rust raises the boss's friction: m3); 15 mm, as the post now takes a hung 6 "
              "L's roll with its share of the moving parts at 5.7 x its yield (12 mm had "
              "been 2.99 x). Its steel taken as the NETRF12's, 1018 or equivalent at 220 "
              "MPa."),
    Item("m8_bhcs_12", "M8 x 12 button head socket screw (ISO 7380), 18-8", "fastener",
         (Offer("Bolt Depot", BD + "6654", "6654", price_usd=0.40, verified=True,
                tiers=((1, 0.40), (100, 0.2688)),
                note="18-8 (A-2), M8-1.25 x 12, head 13.57-14.00 x 4.10-4.40, 5 mm hex; page "
                     "rendered 2026-10-15"),),
         dims={"d": 8.0, "length": 12.0, "head_d": 14.0, "head_h": 4.4},
         notes="Up into the rear saddle's post's foot from under its carrier."),
)

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
                     "45 N·m; 3.5 kN radial; backlash 1/2°; 27 in stock; country of origin "
                     "China, US HTS 8483.40.50.10 (page re-rendered 2026-10-14, the price "
                     "unchanged). Ships from Calgary, Canada by UPS; since 2025-05-02 Phidgets "
                     "collects US duties, tariffs and brokerage at checkout (UPS Global "
                     "Checkout, a fee of its own): the duty on this code from China is 40 % "
                     "(2.5 % + 25 % Section 301 + 12.5 % since 2026-07; tariffs.wove.com, "
                     "2026-10-14), the shaft's 50 %, no de minimis: not in the total"),),
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
    *(Item(f"m6_bhcs_{n}", f"M6 x {n} button head socket screw (ISO 7380)"
           + (", A2-70" if n in M6_BHCS_A270 else ""), "fastener",
           ((Offer("Fastenal", FASTENAL + M6_BHCS_A270[n][0], M6_BHCS_A270[n][0],
                   price_usd=M6_BHCS_A270[n][1], verified=True,
                   note="A2-70 stainless (ISO 3506-1), ISO 7380-1; page rendered 2026-10-15; "
                        "sold singly"),) if n in M6_BHCS_A270 else ())
           + (Offer("Bolt Depot", BD + pn, pn, price_usd=each, verified=True,
                    tiers=((1, each),) + (((100, per100 / 100),) if per100 else ()),
                    note=BD_NOTE + ("; no property class stated (A2-50's 210 MPa yield "
                                    "at least): not for the posts' and tie rods' joints"
                                    if n in M6_BHCS_A270 else "")),),
           dims={"d": 6.0, "length": float(n), "head_d": 10.5, "head_h": 3.3,
                 **({"property_class": "A2-70", "proof_n": 9045.0}
                    if n in M6_BHCS_A270 else {})})
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
    # the rear saddle's lead screw (construction.cradle: its V block rides it at 8 mm a turn;
    # ServoCity pages rendered 2026-10-14, /root/scratch/decant/review5/r1_mounting/pages/ on
    # the development box)
    Item("lead_screw_8x8_150", "Lead screw, 8 mm, 8 mm lead (2 mm pitch, 4 starts), 150 mm, "
         "stainless steel", "fastener",
         (Offer("ServoCity", SERVOCITY + "3501-series-lead-screw-8mm-lead-4-start-150mm-length/",
                "3501-0804-0150", price_usd=6.99, verified=True,
                note="3501 Series lead screw: stainless steel, 8 mm, 2 mm pitch, 4 starts, "
                     "150 mm, 46 g; page rendered 2026-10-14 (the common 3D printer T8 x 8; "
                     "any 150 mm one with its nut fits)"),),
         dims={"d": 8.0, "lead": 8.0, "pitch": 2.0, "starts": 4, "length": 150.0,
               "root": 5.5},
         notes="The rear saddle's thumbscrew: 9 turns over its V block's 72 mm. It isn't "
               "self-locking: its clamping collar, on a TPU brake washer under the block's "
               "load, holds it (cradle_report.saddles)."),
    Item("lead_nut_8x8_barrel", "Lead screw barrel nut, 8 mm lead, 4 starts, 12 mm OD x 16 mm, "
         "bronze", "nut",
         (Offer("ServoCity", SERVOCITY + "3500-series-lead-screw-barrel-nut-8mm-lead-4-start-"
                "12mm-od-16mm-length/", "3500-0804-1216", price_usd=4.99, verified=True,
                note="3500 Series barrel nut: bronze, 2 mm pitch, 4 starts, 12 mm OD x 16 mm, "
                     "10 g; page rendered 2026-10-14"),),
         dims={"od": 12.0, "length": 16.0, "material": "brass"},
         notes="In the rear V block's boss from below, an M3 set screw on it: the block bears "
               "down on it."),
    Item("lead_collar_8x8", "Lead screw clamping collar, 8 mm lead, 4 starts, 21 mm OD x 9 mm, "
         "aluminium", "fastener",
         (Offer("ServoCity", SERVOCITY + "3504-series-lead-screw-clamping-collar-8mm-lead-4-"
                "start-21mm-od-9mm-length/", "3504-0804-2109", price_usd=6.99, verified=True,
                note="3504 Series clamping collar: aluminium, clear anodized, its bore threaded "
                     "to the lead screw, a pinch bolt; 21 mm OD x 9 mm, 8.2 g; page rendered "
                     "2026-10-14"),),
         dims={"od": 21.0, "length": 9.0},
         notes="On the rear lead screw's foot in the carrier's pocket, on the TPU brake "
               "washer: the block's load on it holds the coarse screw from turning back."),
    *(Item(f"m5_tap_bolt_{n}", f"M5 x {n} hex tap bolt, fully threaded, 18-8", "fastener",
           (Offer("Bolt Depot", BD + pn, pn, price_usd=each, verified=True,
                  tiers=((1, each), (100, per100 / 100)),
                  note="18-8 (A-2), M5-0.8, fully threaded, 8 mm across flats; the M5 tap "
                       "bolt list page rendered 2026-10-14"),),
           dims={"d": 5.0, "length": float(n), "af": 8.0, "head_h": 3.5})
      for n, (pn, each, per100) in {25: ("5952", 0.22, 14.52),
                                    40: ("5955", 0.34, 22.84)}.items()),
    Item("m5_nylon_nut", "M5 nylon-insert lock nut (DIN 985), 18-8", "nut",
         (Offer("Bolt Depot", BD + "4794", "4794", price_usd=0.09,
                tiers=((1, 0.09), (100, 0.0554)), verified=True,
                note="18-8, M5-0.8, nylon insert; listed with the M5 tap bolts, rendered "
                     "2026-10-14"),),
         dims={"af": 8.0, "h": 5.0},
         notes="On the lip gauge's pin behind its back knuckle: snug, it stays where it's set."),
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
               "fender washer, an M6 nut in a printed thumb knob and a thread past it "
               "(cradle._bracket). (McMaster's 18-8 M6 carriage bolts, 97248A311-316, list "
               "no price to a scripted page: the zinc one is kept, its head under the "
               "board.)"),
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


# -- the drive's round 1 (design review round 1, 2026-10-14; construction.handcrank): the
# hand crank's inserts and grip pin's nut and washers (its riser's spacers, the seat's shims and
# the ball-detent clutch's balls, springs and set screws went in round 2). Pages rendered
# 2026-10-14, saved under /root/.cache/spiderpig/tmp/r1drive/pages/ on the development box -----

register(
    Item("m3_heat_set_insert", "M3 heat-set threaded insert for plastic, brass, 5.7 mm long",
         "insert",
         (Offer("Accu", "https://accu-components.com/us/brass-threaded-inserts/"
                "845137-HTBI-M3-BR", "HTBI-M3-BR", price_usd=2.35, verified=True,
                tiers=((1, 2.35), (2, 1.68), (4, 1.36), (6, 1.20), (12, 1.09), (100, 0.69)),
                note="brass, M3, 5.7-5.8 mm long, about 4.4 mm over the knurls, for a 4.0 mm "
                     "hole (+0.1: a search snippet; the page shows the insert and its prices); "
                     "sold singly; page rendered 2026-10-14"),),
         dims={"d": 3.0, "length": 5.7, "hole": 4.0},
         notes="The hand crank's hub (its thumb screw onto the input key) and its journal's "
               "end (the arm's cap): brass threads where the crank comes off at every pack."),
    Item("m8_nyloc_nut", "M8 nylon-insert lock nut, 18-8", "nut",
         (Offer("Bolt Depot", BD + "4797", "4797", price_usd=0.17, verified=True,
                tiers=((1, 0.17), (100, 0.1187)),
                note="18-8, M8 x 1.25, 13 mm across flats, 8 mm high; page rendered "
                     "2026-10-14"),),
         dims={"af": 13.0, "h": 8.0}),
    Item("m8_washer", "M8 flat washer (DIN 125, 8.4 x 16 x 1.6), 18-8", "washer",
         (Offer("Bolt Depot", BD + "4518", "4518", price_usd=0.08, verified=True,
                tiers=((1, 0.08), (100, 0.0529)),
                note="18-8, M8, 16 mm OD; page rendered 2026-10-14"),),
         dims={"id": 8.4, "od": 16.0, "t": 1.6}),
)


# -- the drive's round 2 (design review round 2, 2026-10-14; construction.handcrank): the
# NRV050 and its shaft (the user's call: the 50 series), the riser's studs and nuts, the
# shaft's M10 end screw, the clutch's plungers, nuts and thrust washer, the grip bolts. Pages
# rendered 2026-10-14 and 2026-10-15, saved under /root/.cache/spiderpig/tmp/r2-fix-drive/pages/
# on the development box -------------------------------------------------------------------

PH_DUTY_NOTE = (
    "Ships from Calgary, Canada by UPS; since 2025-05-02 Phidgets collects US duties, tariffs "
    "and brokerage at checkout (UPS Global Checkout, a fee of its own): the duty on HTS "
    "8483.40.50.10 from China is 40 % (2.5 % + 25 % Section 301 + 12.5 % since 2026-07; "
    "tariffs.wove.com, 2026-10-14), on the shaft's 8483.10.50.00 50 %, no de minimis: not in "
    "the total (ORDER.md's landed note)")
BD_2 = "18-8 (A-2); Bolt Depot page rendered 2026-10-15"

register(
    Item("nrv050_30", "NRV050 worm gear reducer, 30:1, 14 mm input shaft, 25 mm hollow output "
         "(Phidgets 85 N·m 50 Series)", "gearbox",
         (Offer("Phidgets", "https://www.phidgets.com/?prodid=870", "TRM4402_0",
                price_usd=92.50, verified=True, tiers=((1, 92.50), (5, 88.80), (10, 85.10)),
                note="30:1 only. 14 mm input with 5 mm keyway; 25 mm bore with 8 mm keyway; "
                     "'Maximum Strength of Gears 85 N·m'; 'Shaft Maximum Radial Load 4.8 kN'; "
                     "backlash 1/2°; 15 in stock; country of origin China, US HTS "
                     "8483.40.50.10 (its specification tab rendered 2026-10-14). Drawing: "
                     "productfiles/TRM4400/TRM4400_0/Documentation/TRM4400_0Mechanical.pdf. "
                     + PH_DUTY_NOTE),),
         dims={"ratio": 30, "centre_distance": 50.0, "input_d": 14.0, "input_len": 30.0,
               "input_shoulder": 64.0, "bore": 25.0, "pcd": 85.0, "face_thread": "M8x10",
               "spigot_d": 70.0, "width": 87.0, "mass_g": 3450.0},
         notes="Motovario's NRV050 at 30:1 (its VSF catalogue): static efficiency 0.44, under "
               "the 0.5 of static irreversibility: a load on the cradle can't turn the handle. "
               "Mass: Motovario's 3.3 kg dry, about 3.45 kg with its oil. The cradles' since "
               "design review round 2 (the user's call of 2026-10-14; the NRV040's 45 N·m "
               "maximum was under its clutch's cap)."),
    Item("nrv050_output_shaft_25", "Single output shaft for a 50 series worm gearbox, 25 mm "
         "keyed (keys, washers, snap ring)", "misc",
         (Offer("Phidgets", "https://www.phidgets.com/?prodid=738", "TRM4008_0", price_usd=6.85,
                verified=True, tiers=((1, 6.85), (5, 6.58), (10, 6.30)),
                note="25 mm, 8 mm keyway, HTS 8483.10.50.00, China; page rendered 2026-10-14. "
                     + PH_DUTY_NOTE),),
         dims={"d": 25.0, "fit": "h6", "length": 153.0, "protrusion": 53.5, "collar_d": 32.0,
               "collar_t": 3.5, "key": (8.0, 7.0, 40.0), "centre_tap": "M10x16"},
         notes="Phidgets' drawing (TRM4004_0Mechanical.pdf, the 050 row): 25 h6 x 153, a 32 x "
               "3.5 collar, 50 past it, the keyway 40 long ending 5 short of its end and 4 "
               "deep, an M10 x 16 end tap; C45 quenched and tempered, HB 220-260."),
    Item("m8_stud_88_45", "M8 x 45 stud, class 8.8 zinc-plated, cut from a 200 mm DIN 976 "
         "threaded rod (four to a rod)", "fastener",
         (Offer("McMaster-Carr", MCM + "99067A279/", "99067A279", pack_qty=4, price_usd=5.11,
                verified=True,
                note="M8 x 1.25 x 200 mm zinc-plated class 8.8 steel threaded rod, DIN 976, "
                     "110,000 psi; the class 8.8 threaded studs listing rendered 2026-10-20; "
                     "cut into four 45 mm studs by hand (a hacksaw)"),),
         dims={"d": 8.0, "length": 45.0, "proof_mpa": 580.0},
         notes="The gearbox's riser studs (construction.handcrank.RISER_STUD, design review "
               "round 3's M2): threaded into the face's tapped holes with threadlocker, each "
               "the tension of one clamped column; an 18-8 set screw's class is for a "
               "headless screw not loaded in tension (TR Fastenings)."),
    Item("m8_sleeve_12x20", "304 stainless spacer 8 mm ID x 12 mm OD x 20 mm (pack of 5)",
         "misc",
         (Offer("Harfington (uxcell)", "https://www.harfington.com/products/p-1579935",
                "a24101800ux1328", pack_qty=5, price_usd=7.53, verified=True,
                note="5 pcs, 304 stainless, 8 ID x 12 OD x 20 mm, 'for 5/16\" or M8 screw'; "
                     "the product JSON fetched 2026-10-20"),),
         dims={"id": 8.0, "od": 12.0, "length": 20.0},
         notes="The gearbox riser's sleeves (construction.handcrank.RISER_SLEEVE): one on "
               "each stud, the ring to the shims, clamped."),
    Item("shim_8x14x0p5", "304 stainless flat shim 8 x 14 x 0.5 mm (pack of 20)", "washer",
         (Offer("Harfington (uxcell)", "https://www.harfington.com/products/p-1378778",
                "a23040700ux0239", pack_qty=20, price_usd=6.14, verified=True,
                note="20 pcs M8 304 ultra-thin flat washers 8 x 14 x 0.5 mm; the product JSON "
                     "fetched 2026-10-20 (2026-10-10 the same)"),),
         dims={"id": 8.0, "od": 14.0, "t": 0.5},
         notes="The gearbox riser's go/no-go shims (construction.handcrank.RISER_SHIM), over "
               "each sleeve under the outer plate."),
    Item("m8_thin_nut", "M8 hex jam nut, 18-8 (13 x 4 mm)", "nut",
         (Offer("Bolt Depot", BD + "13573", "13573", price_usd=0.15, verified=True,
                tiers=((1, 0.15), (100, 0.0979)), note=BD_2 + ": 13 mm AF, 4 mm"),),
         dims={"af": 13.0, "h": 4.0}),
    Item("m10_bhcs_16", "M10 x 16 button head socket screw (ISO 7380), 18-8", "fastener",
         (Offer("Bolt Depot", BD + "13370", "13370", price_usd=1.14, verified=True,
                tiers=((1, 1.14), (25, 19.03 / 25)),
                note=BD_2 + " (the M10 18-8 button head list)"),),
         dims={"d": 10.0, "length": 16.0, "head_d": 17.5, "head_h": 5.5},
         notes="The NRV050 output shaft's end screw (construction.handcrank.END_SCREW_NM)."),
    Item("m16_washer", "M16 flat washer (DIN 125, 17 x 30 x 3), 18-8", "washer",
         (Offer("Bolt Depot", BD + "4522", "4522", price_usd=0.48, verified=True,
                tiers=((1, 0.48), (25, 8.04 / 25)),
                note=BD_2 + " (the 18-8 16 mm flat washer list); its size DIN 125's"),),
         dims={"id": 17.0, "od": 30.0, "t": 3.0},
         notes="The hand crank's thrust washer under the journal's cap (handcrank.THRUST)."),
    Item("spring_plunger_half13", "Ball-nose spring plunger, 1/2\"-13, 0.281 in 440C ball, 6-12 "
         "lbf, black-oxide steel body, hex drive", "misc",
         (Offer("McMaster-Carr", MCM + "5901A17/", "5901A17", price_usd=4.59, verified=True,
                note="1/2\"-13, extended nose 0.072 in, 0.281 in 440C stainless ball, 6 lbf "
                     "extended, 12 lbf compressed, black-oxide steel body 1.05 in long, hex "
                     "drive; the ball-nose spring plunger listing rendered 2026-10-14 "
                     "(https://www.mcmaster.com/products/ball-nose-spring-plungers/)"),),
         dims={"thread": "1/2-13", "ball_d": 7.14, "length": 26.67, "travel": 1.83,
               "force_n": (26.7, 53.4)},
         notes="The hand crank's slip clutch (construction.handcrank.PLUNGER): three in the "
               "arm's boss, their balls on the seat ring's holes."),
    Item("half13_nut", "1/2\"-13 hex nut, 18-8", "nut",
         (Offer("Bolt Depot", BD + "2566", "2566", price_usd=0.43, verified=True,
                tiers=((1, 0.43), (50, 13.56 / 50)), note=BD_2 + ": 3/4 in AF, 7/16 in"),),
         dims={"af": 19.05, "h": 11.11}),
    Item("half13_jam_nut", "1/2\"-13 hex jam nut, 18-8", "nut",
         (Offer("Bolt Depot", BD + "4124", "4124", price_usd=0.43, verified=True,
                tiers=((1, 0.43), (100, 0.3304)), note=BD_2 + ": 3/4 in AF, 5/16 in"),),
         dims={"af": 19.05, "h": 7.94}),
    *(Item(f"m8_hex_bolt_{L}", f"M8 x {L} hex bolt, partially threaded, 18-8", "fastener",
           (Offer("Bolt Depot", BD + pn, pn, price_usd=each, verified=True,
                  tiers=((1, each), (n, bag / n)),
                  note=BD_2 + " (the M8 18-8 hex bolt list)"),),
           dims={"d": 8.0, "length": float(L), "thread": 22.0, "af": 13.0, "head_h": 5.3},
           notes="A hand crank's grip pin (construction.handcrank.HANDLES): the printed grip "
                 "spins on its plain shank.")
      for L, (pn, each, n, bag) in {90: ("5827", 1.68, 100, 109.06)}.items()),
)
