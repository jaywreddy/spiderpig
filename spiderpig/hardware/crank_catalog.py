"""Catalog data for the bolt crank (:class:`construction.crank.BoltCrank`) and the standoff
pillar (:mod:`construction.pivots.standoff`).

Appended to the catalog from the end of :mod:`hardware.parts` (new items only). Searched
2026-10-03; an offer is ``verified=True`` only where the page was fetched and showed the
product and its price. Prices a search result quoted (not fetched) say so in their note.

* ``m6_hex_bolt_<L>``: ISO 4014 (DIN 931) M6 partially threaded hex bolts, class 8.8
  zinc plated, every stock length the bolt crank may pick (:data:`M6_BOLT_LENGTHS`: 30-80
  mm in 5 mm steps, FMW Fasteners' DIN 931 listing fetched 2026-10-04 with prices; M6 x 25
  and shorter are sold only fully threaded, DIN 933, which would put the riders on thread).
  Thread length ``b`` is 18 mm for every length here (ISO 4014: ``b = 2d + 6`` up to
  125 mm), so the plain shank is ``L - 18`` (``lg``); the incomplete thread (ISO 3508
  runout, at most 2.5 pitches) sits above that, which the crank keeps clear of its riders.
* ``m6_nylock``: DIN 985 M6 (10 AF, 6.0 mm; the nylon ring takes about 1.5 mm).
* ``threadlocker_243``: medium strength, on each crank bolt's thread under its nut.
* ``m3_round_standoff_ff_<L>``: 6 mm OD round aluminium M3 female-female standoffs, the
  bolt crank's journal stub.
* ``gobilda_1501_<L>``: goBILDA 1501 series M4 x 0.7 round aluminium standoffs, 6 mm OD,
  the standoff pillar's segments: only the lengths goBILDA sells (:data:`GOBILDA_LENGTHS`,
  their M4 standoff listing fetched 2026-10-04 with each 4-pack's price: 3-12, then 14-60
  mm in mostly 2 mm steps, plus 19, 27 and 43; 13, 15, 17, 21, 23, 25, 29, 31, 33, 35, 37,
  39, 41, 45, 47, 49, 51, 53, 55, 57 and 59 mm don't exist: the 33, 39 and 45 mm product
  pages answer 404).
* ``m4_bhcs_<L>``, ``m4_washer``, ``m4_set_screw_<L>``: the standoff pillar's end screws
  (an ISO 7380 button head and a DIN 125 washer, 3.0 mm together: one layer outside each
  frame plate) and the splices' studs (ISO 4026 set screws threaded into both segments).
* ``ptfe_washer_6x12x0p5``: the crank study's thrust washer for the riders, listed but not
  built: in a 3 mm layer pitch the plates touch, so a 0.5 mm washer between a crank plate
  and a rider has no room (the riders turn against the crank's acrylic plates and the bolt
  head and nut faces, as every link on a pin turns against its neighbours).
"""

from __future__ import annotations

from spiderpig.hardware.catalog import Item, Offer, register

SEARCHED = "as a 2026-10-03 web search quoted the page (not fetched)"
FETCHED = "fetched 2026-10-04"

M6_BOLT_PRICES: dict[int, float] = {30: 0.35, 35: 0.52, 40: 0.58, 45: 0.63, 50: 0.69, 55: 0.74,
                                    60: 0.80, 65: 1.25, 70: 1.00, 75: 1.49, 80: 1.15}
"""FMW Fasteners' M6-1.0 DIN 931 8.8 zinc listing (fetched 2026-10-04), USD each."""
M6_BOLT_LENGTHS: tuple[float, ...] = tuple(float(L) for L in M6_BOLT_PRICES)
"""ISO 4014 M6 stock lengths (mm under the head): 30 mm is the shortest sold partially
threaded (ISO 4014's M6 range starts there; FMW sells M6 x 25 only as DIN 933, fully
threaded), plain shank 12 mm."""
M6_THREAD_B = 18.0


def m6_bolt(length: float) -> str:
    return f"m6_hex_bolt_{length:g}"


for _L in M6_BOLT_LENGTHS:
    register(Item(
        m6_bolt(_L), f"M6 x {_L:g} mm hex bolt, partially threaded (ISO 4014 / DIN 931), 8.8 "
        "zinc", "fastener",
        (Offer("FMW Fasteners",
               "https://www.fmwfasteners.com/search?q=M6-1.0+hex+cap+screw+8.8+DIN+931",
               price_usd=M6_BOLT_PRICES[int(_L)], verified=True,
               note=f"M6-1.0 x {_L:g} DIN 931 8.8 zinc, ${M6_BOLT_PRICES[int(_L)]:.2f} each "
                    f"(the listing {FETCHED})"),
         Offer("Fastenal", "https://www.fastenal.com/products/details/M72550030A20000",
               "M72550030A20000", note=f"A2-70 30 mm: $0.29 each, $5.84 per 100 ({SEARCHED})"),
         Offer("McMaster-Carr", "https://www.mcmaster.com/products/hex-head-screws/",
               note="pick M6 x 1 mm, partially threaded, from the listing; part number not "
                    "confirmed")),
        dims={"d": 6.0, "pitch": 1.0, "length": float(_L), "head_af": 10.0,
              "head_af_min": 9.78, "head_h": 4.0, "b": M6_THREAD_B, "stress_d": 4.92,
              "yield_mpa": 640.0},
        notes="ISO 4014 M6: s 10 (9.78 min), k 4.0, b 18; class 8.8 (640 MPa proof). A2-70 "
              "(450 MPa) is the stainless alternative: the strength check's thread torsion "
              "drops by 30 %, the hex pockets don't change.",
    ))

register(
    Item("m6_nylock", "M6 nylon-insert lock nut (DIN 985), 8 zinc", "nut",
         (Offer("Bolt Depot", "https://www.boltdepot.com/Metric_nylon_insert_lock_nuts.aspx",
                note="M6-1.0 nylon insert lock nut; part number not confirmed"),
          Offer("Amazon", "https://www.amazon.com/s?k=M6+nylon+insert+lock+nut+DIN+985",
                note="search; 50-100 packs")),
         dims={"d": 6.0, "af": 10.0, "h": 6.0, "metal_h": 4.5, "prevailing_nm": 0.4},
         notes="DIN 985 M6: s 10, m 6.0 (the nylon ring about 1.5 mm of it). Prevailing "
               "torque: ISO 2320's least on removal for M6, 0.4 N·m (the strength check's "
               "nut lock counts it)."),
    Item("threadlocker_243", "Medium-strength threadlocker (Loctite 243 or equivalent), 10 ml",
         "adhesive",
         (Offer("Amazon", "https://www.amazon.com/s?k=Loctite+243+10ml",
                note="search; blue, removable with hand tools; oil tolerant"),),
         dims={"breakaway_m10_nm": 26.0},
         notes="On each crank bolt's thread where the nylock sits. The TDS gives a breakaway "
               "torque of about 26 N·m on M10 steel nuts and bolts; on plated or stainless "
               "(passive) surfaces less: the strength check takes half, scaled to the M6 "
               "nut's thread (UNVERIFIED for this nut: a test of one joint settles it)."),
    Item("ptfe_washer_6x12x0p5", "PTFE flat washer 6.4 x 12 x 0.5 mm", "washer",
         (Offer("eBay", "https://www.ebay.com/itm/121716489464",
                note=f"PTFE M6 6.4 x 12 mm, 50 pcs ({SEARCHED}; thickness not stated: 1.5-2 "
                     "mm is the common size, 0.5 mm is rare)"),),
         dims={"id": 6.4, "od": 12.0, "t": 0.5},
         notes="Listed, not built: no room for it in a 3 mm layer pitch (module docstring)."),
)

M3_ROUND_STANDOFF_LENGTHS: tuple[float, ...] = (5, 6, 8, 10, 12, 15, 18, 20, 25, 30)


def m3_round_standoff(length: float) -> str:
    return f"m3_round_standoff_ff_{length:g}"


for _L in M3_ROUND_STANDOFF_LENGTHS:
    register(Item(
        m3_round_standoff(_L), f"M3 x {_L:g} mm round aluminium standoff, female-female, 6 mm "
        "OD", "standoff",
        (Offer("Amazon", "https://www.amazon.com/s?k=M3+round+aluminum+standoff+female+female"
               "+6mm+OD", note="search; uxcell and others sell 6 mm OD M3 F-F round aluminium "
                               "standoffs in 5-30 mm, 10-20 packs (length steps unverified)"),
         Offer("AliExpress", "https://www.aliexpress.com/w/wholesale-m3-round-aluminum-"
               "standoff.html", note="search; 6 mm OD, 5-40 mm")),
        dims={"d": 3.0, "od": 6.0, "length": float(_L), "thread_depth": min(6.0, _L / 2)},
        notes="The bolt crank's journal stub: screwed to the lowest web stack, turning in the "
              "outer frame plate. OD and lengths UNVERIFIED (measure).",
    ))

GOBILDA_PRICES: dict[int, float] = {
    3: 2.29, 4: 2.39, 5: 2.49, 6: 2.49, 7: 2.59, 8: 2.69, 9: 2.69, 10: 2.79, 11: 2.89,
    12: 2.89, 14: 3.09, 16: 3.19, 18: 3.29, 19: 3.39, 20: 3.49, 22: 3.59, 24: 3.69, 26: 3.89,
    27: 3.89, 28: 3.99, 30: 4.09, 32: 4.29, 34: 4.39, 36: 4.49, 38: 4.69, 40: 4.79, 42: 4.89,
    43: 4.99, 44: 5.09, 46: 5.19, 48: 5.29, 50: 5.49, 52: 5.59, 54: 5.69, 56: 5.89, 58: 5.99,
    60: 6.09}
"""goBILDA's M4 standoff listing (https://www.gobilda.com/m4-standoffs/, fetched 2026-10-04):
every 1501 (6 mm OD) length sold, USD per 4-pack."""
GOBILDA_LENGTHS: tuple[float, ...] = tuple(float(L) for L in GOBILDA_PRICES)


def gobilda_1501(length: float) -> str:
    return f"gobilda_1501_{length:g}"


for _L in GOBILDA_LENGTHS:
    _n = int(_L)
    _price = GOBILDA_PRICES[_n]
    register(Item(
        gobilda_1501(_L), f"goBILDA 1501 M4 x 0.7 round standoff, 6 mm OD, {_L:g} mm "
        "(4-pack)", "standoff",
        (Offer("goBILDA", f"https://www.gobilda.com/1501-series-m4-x-0-7mm-standoff-6mm-od-"
               f"{_n}mm-length-4-pack/", f"1501-0006-{_n * 10:04d}", pack_qty=4,
               price_usd=_price, verified=True,
               note=f"${_price:.2f} per 4, on the M4 standoff listing ({FETCHED})"),),
        dims={"d": 4.0, "od": 6.0, "length": _L, "thread_depth": min(8.0, _L / 2),
              "id": 3.3, "yield_mpa": 240.0},
        notes="Aluminium, clear anodised, M4 x 0.7 female both ends. The strength check takes "
              "it as a 6 x 3.3 mm tube (the tap drill's bore, as if tapped through) of 6061-T6 "
              "(240 MPa): conservative for a part tapped only at its ends.",
    ))

M4_BHCS_LENGTHS: tuple[float, ...] = (6, 8, 10, 12, 16)
M4_SET_LENGTHS: tuple[float, ...] = (8, 10, 12, 16)


def m4_bhcs(length: float) -> str:
    return f"m4_bhcs_{length:g}"


def m4_set_screw(length: float) -> str:
    return f"m4_set_screw_{length:g}"


for _L in M4_BHCS_LENGTHS:
    register(Item(
        m4_bhcs(_L), f"M4 x {_L:g} mm button head socket screw (ISO 7380)", "fastener",
        (Offer("Bolt Depot", "https://www.boltdepot.com/Metric_button_head_socket_cap_screws_"
               "Stainless_steel_18-8_M4_x_0.7.aspx", note="18-8 stainless; length per page"),
         Offer("Amazon", f"https://www.amazon.com/s?k=M4+x+{_L:g}mm+button+head+socket+screw",
               note="search")),
        dims={"d": 4.0, "length": float(_L), "head_d": 7.6, "head_h": 2.2},
        notes="ISO 7380-1 M4: dk 7.6, k 2.2.",
    ))
for _L in M4_SET_LENGTHS:
    register(Item(
        m4_set_screw(_L), f"M4 x {_L:g} mm set screw (ISO 4026, flat point)", "fastener",
        (Offer("Bolt Depot", "https://www.boltdepot.com/Metric_set_screws.aspx",
               note="M4-0.7 socket set screw; length per page"),
         Offer("Amazon", f"https://www.amazon.com/s?k=M4+x+{_L:g}mm+set+screw", note="search")),
        dims={"d": 4.0, "length": float(_L)},
        notes="A splice's stud: threaded half into each standoff segment through the splice "
              "plate, threadlocked.",
    ))

register(
    Item("m4_washer", "M4 flat washer (DIN 125-A, 4.3 x 9 x 0.8)", "washer",
         (Offer("Bolt Depot", "https://www.boltdepot.com/Metric_flat_washers.aspx",
                note="M4 DIN 125 flat washer"),
          Offer("Amazon", "https://www.amazon.com/s?k=M4+DIN+125+flat+washer", note="search")),
         dims={"id": 4.3, "od": 9.0, "t": 0.8}),
)

