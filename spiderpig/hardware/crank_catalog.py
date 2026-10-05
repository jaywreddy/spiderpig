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
    Item("ptfe_washer_6x12x0p5", "PTFE flat washer 6.3 x 12 x 0.5 mm", "washer",
         (Offer("eBay", "https://www.ebay.com/itm/121716489464",
                note=f"PTFE M6 6.4 x 12 mm, 50 pcs ({SEARCHED}; thickness not stated: 1.5-2 "
                     "mm is the common size, 0.5 mm is rare)"),),
         dims={"id": 6.3, "od": 12.0, "t": 0.5},   # MISUMI TT-0612-05
         notes="Listed, not built: no room for it in a 3 mm layer pitch (module docstring)."),
)

M3_ROUND_STANDOFF_LENGTHS: tuple[float, ...] = (6, 8, 10, 12, 15, 18, 20, 25, 30)
"""The 6 mm OD lengths uxcell sells threaded (hardware.sources, 2026-10-05): no threaded
6 mm OD x 5 mm exists."""


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
        dims={"d": 3.0, "od": 6.0, "length": float(_L), "thread_depth": float(_L),
              "id": 2.5, "yield_mpa": 240.0},
        notes="uxcell (Harfington), black anodised aluminium, threaded through (coupling-nut "
              "style): the bolt crank's journal stub, the frame ties' segments and those of "
              "--pillar standoff_m3. The strength check takes a 6 x 2.5 tube (the "
              "M3 tap drill) at 240 MPa: the alloy is not stated (UNVERIFIED). No length "
              "tolerance stated: measure one against its gap.",
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

PILLAR_SHAFT_LENGTHS: tuple[float, ...] = tuple(L / 10 for L in range(80, 3001))
"""The one-piece pillar's lengths (:meth:`construction.pivots.standoff.StandoffAxle.one_piece`):
MISUMI makes it to any length from 8 to 300 mm in 0.1 mm steps (+-0.1)."""
PILLAR_SHAFT_ID = 2.5          # the strength check's bore: the M3 tap drill, as if tapped through
PILLAR_SHAFT_YIELD = 220.0     # MPa: 1018 steel at its hot-rolled minimum (cold drawn: ~370);
#                                MISUMI states the grade, not the temper (conservative)
PILLAR_SHAFT_PRICES = (15.32, 11.12, 5.61)
"""USD each at 1-4, 5-9 and 10-19 pieces (NETRF6-128; NETRF6-62 $14.97 / 10.86 / 5.47): the
MISUMI page rendered 2026-10-05. The discount is per line: five cost less than four."""


PILLAR_SHAFT_SEEN: tuple[float, ...] = (62.0, 62.4, 127.5, 128.0, 128.1)
"""The NETRF6 lengths whose MISUMI page was rendered (2026-10-05) with the part and its price
(62.4 and 128.1: the Strider double's and quad's columns, unit price as 62 and 128)."""


def pillar_shaft(length: float) -> str:
    return f"pillar_shaft_6_m3_{length:g}"


for _L in PILLAR_SHAFT_LENGTHS:
    _pn = f"NETRF6-{_L:g}"
    register(Item(
        pillar_shaft(_L), f"6 mm round steel standoff, {_L:g} mm, tapped M3 both ends "
        f"(MISUMI {_pn})", "standoff",
        (Offer("MISUMI", "https://us.misumi-ec.com/vona2/detail/110300208270/?HissuCode="
               + _pn, _pn, pack_qty=5,
               price_usd=round(5 * (10.86 if _L < 95 else PILLAR_SHAFT_PRICES[1]), 2),
               verified=_L in PILLAR_SHAFT_SEEN,
               note="MISUMI circular standoff, tapped both ends, configurable length: 1018 "
                    "steel, oiled (no plating), 6 mm OD (0/-0.1), M3 x 6 deep each end, length "
                    "+-0.1 mm in 0.1 mm steps (the part number's number). Sold singly: USD "
                    "15.32 each at 1-4, 11.12 at 5-9, 5.61 at 10-19 (NETRF6-128; NETRF6-62 "
                    "14.97 / 10.86 / 5.47; 2026-10-05), the discount per line, so it is listed "
                    "as 5 (USD 55.60, less than 4 at 61.28): order 5 of the length (one "
                    "spare)"),),
        dims={"d": 3.0, "od": 6.0, "length": _L, "thread_depth": 6.0, "id": PILLAR_SHAFT_ID,
              "yield_mpa": PILLAR_SHAFT_YIELD},
        notes="A standoff pillar's column no single goBILDA length fills (longer than 60 mm, or "
              "a length goBILDA lacks): one piece made to its length, never spliced. The "
              "strength check takes it as a 6 x 2.5 tube (the M3 tap drill, as if tapped "
              "through) of 1018 at 220 MPa. Oiled bare steel: wipe it, and keep it dry.",
    ))

M4_BHCS_LENGTHS: tuple[float, ...] = (5, 6, 8, 10, 12, 16)
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

# -- the hex-standoff crankpin (the user's decision of 2026-10-04) ----------------------
#
# ``hex_standoff_m3_<L>``: M3 x 5.5 mm AF hexagon spacers, female-female, zinc-plated steel,
# tapped through (Accu's HHTPS-M3-5.5-<L>-S-Z range). The crank's crankpins and journals:
# each hex end sits in a hex pocket of an aluminium crank plate, an M3 button head and a
# DIN 9021 washer screwed into each end retain the plates; the riders turn on a printed
# sleeve over the hex (:class:`construction.crank.BoltCrank`, ``pin="hex"``). Lengths: the
# 10, 12, 35, 40, 50 and 60 mm parts were seen on Accu's site in a 2026-10-04 web search
# (product pages not fetched: Accu answers 403 to a fetch); the rest are the series every
# hex-spacer vendor stocks (Accu, Vital Parts, McMaster), UNVERIFIED per length. No prices:
# Accu quotes per pack size (estimate $0.30-0.60 each).

HEX_M3_SEEN: tuple[float, ...] = (10, 12, 35, 40, 50, 60)
HEX_M3_LENGTHS: tuple[float, ...] = (5, 6, 8, 10, 12, 15, 16, 18, 20, 22, 25, 30, 35, 40, 45,
                                     50, 60)
"""Stock lengths of the M3 x 5.5 AF F-F steel hex standoff (mm)."""


def hex_standoff_m3(length: float) -> str:
    return f"hex_standoff_m3_{length:g}"


WURTH_HEX_LENGTHS: tuple[float, ...] = (5, 6, 8, 10, 12, 15, 16, 18, 20, 22, 25, 30, 35, 40)
"""The lengths of Wurth Elektronik's WA-SSTII M3 x 5.5 AF steel F-F spacer (part 970<LL>0321;
each datasheet fetched 2026-10-05): tapped through up to 20 mm, from 22 mm a 7 mm blind thread
at each end (an M3 x 6 button head through a crank plate and washer engages about 2.7 mm).
The 45, 50 and 60 mm parts are Accu's or Vital Parts' (A1 stainless). McMaster sells M3 hex
standoffs only 5 and 6 mm across flats; Accu's series has no 22 mm."""


WURTH_HEX_PRICES: dict[int, float] = {16: 0.51, 18: 0.63, 20: 0.48, 22: 0.49, 25: 0.57,
                                      30: 0.53}
"""USD each at Mouser (one piece), the lengths the default designs use: 22, 30 from Octopart;
16, 20, 18, 25 from Findchips' Mouser rows (rendered 2026-10-05; Wurth's datasheets: M3,
5.5 AF, steel gloss zinc, status Valid)."""
LONG_HEX_PAGES: dict[int, tuple[Offer, ...]] = {
    50: (Offer("Vital Parts", "https://www.vital-parts.co.uk/threaded-hex-standoffs-female-"
               "female/7886-hff-m3-50-s55-a1", "HFF-M3-50-S55-A1", verified=True,
               note="A1 stainless, 5.5 AF, 12 mm thread each end; GBP 2.02 (fetched "
                    "2026-10-05)"),
         Offer("Accu", "https://accu-components.com/us/threaded-standoffs/464767-HHTPS-M3-5-5-"
               "50-S-Z", "HHTPS-M3-5.5-50-S-Z", price_usd=13.60, verified=True,
               note="zinc-plated steel, made to order (123 days)")),
    60: (Offer("Vital Parts", "https://www.vital-parts.co.uk/threaded-hex-standoffs-female-"
               "female/7902-hff-m3-60-s55-a1", "HFF-M3-60-S55-A1", verified=True,
               note="A1 stainless, 5.5 AF, 12 mm thread each end; GBP 2.18 (fetched "
                    "2026-10-05)"),
         Offer("Accu", "https://accu-components.com/us/threaded-standoffs/464770-HHTPS-M3-5-5-"
               "60-S-Z", "HHTPS-M3-5.5-60-S-Z", price_usd=29.52, verified=True)),
}


def _hex_offers(length: float) -> tuple[Offer, ...]:
    if length in WURTH_HEX_LENGTHS:
        pn = f"970{int(length):02d}0321"
        return (Offer("Mouser", f"https://www.mouser.com/ProductDetail/Wurth-Elektronik/{pn}",
                      f"710-{pn}", price_usd=WURTH_HEX_PRICES.get(int(length)), verified=True,
                      note=f"Wurth WA-SSTII {pn}, steel, gloss zinc, 5.5 AF: Active in "
                           "Wurth's catalog (rendered 2026-10-05) and its datasheet; price "
                           "and stock via Octopart (about USD 0.50 each). Mouser and DigiKey "
                           "block automated browsers. DigiKey stocks the same part"),)
    return LONG_HEX_PAGES.get(int(length), (
        Offer("Accu", "https://accu-components.com/us/threaded-standoffs/",
              f"HHTPS-M3-5.5-{length:g}-S-Z",
              note="in the length selector of Accu's series (page not loaded): pick it there"),))


for _L in HEX_M3_LENGTHS:
    register(Item(
        hex_standoff_m3(_L), f"M3 x {_L:g} mm hex standoff, 5.5 mm AF, female-female, "
        f"zinc-plated steel ({'tapped through' if _L <= 20 else '7 mm thread each end'})",
        "standoff", _hex_offers(_L),
        dims={"d": 3.0, "af": 5.5, "length": float(_L),
              "thread_depth": float(_L) if _L <= 20 else 7.0,
              "yield_mpa": 300.0},
        notes="Free-cutting steel (11SMnPb30 or similar), zinc plated; the strength check "
              "takes 300 MPa for its flats in bearing. A brass part (CuZn39Pb3, about 250 MPa) "
              "is a drop-in: the aluminium pocket governs either way.",
    ))

register(
    Item("m3_washer_9021", "M3 wide flat washer (DIN 9021, 3.2 x 9 x 0.8)", "washer",
         (Offer("Accu", "https://accu-components.com/us/flat-washers/",
                note="DIN 9021 M3, A2 stainless or zinc-plated steel; part number not "
                     "confirmed"),
          Offer("Amazon", "https://www.amazon.com/s?k=M3+DIN+9021+washer", note="search")),
         dims={"id": 3.2, "od": 9.0, "t": 0.8},
         notes="Under each hex crankpin's screw head: it spans the hex pocket and bears on "
               "the standoff's end and the crank plate round it."),
)


# -- the M3 hardware study (SIMPLIFY.md, 2026-10-05) ------------------------------------
#
# ``arl_m3_<L>``: Hirosugi-Keiki ARL-3<L>BE, lead-free free-cutting aluminium (KS26), black
# anodised, round 6 mm OD, M3 female both ends, L +/-0.1 (the maker's page and drawing
# M_AR-30.gif fetched 2026-10-05: http://hirosugi.jp/products/A/ARL-BE.html). Tapped through
# up to 15 mm, from 16 mm a 6 mm thread each end. Lengths 4-12.5 mm in 0.5 mm steps
# (no 10.5 / 11.5), 13-30 mm in 1 mm steps, 35-60 mm in 5 mm steps; USD 0.61-1.40 each
# (MOQ 50 direct; MISUMI resells Hirosugi in small quantities, as it does the PTFE washers).
# ``m3_set_screw_<L>``: ISO 4026 M3 flat point set screws (the splices' and ties' studs).

ARL_M3_PRICES: dict[float, float] = {
    4: 0.61, 4.5: 0.61, 5: 0.61, 5.5: 0.61, 6: 0.61, 6.5: 0.62, 7: 0.62, 7.5: 0.62, 8: 0.63,
    8.5: 0.63, 9: 0.64, 9.5: 0.66, 10: 0.66, 11: 0.67, 12: 0.68, 12.5: 0.68, 13: 0.68,
    14: 0.70, 15: 0.70, 16: 0.71, 16.5: 0.72, 17: 0.72, 17.5: 0.72, 18: 0.73, 19: 0.77,
    20: 0.78, 21: 0.79, 22: 0.79, 23: 0.80, 24: 0.82, 25: 0.96, 26: 0.96, 27: 0.97, 28: 0.98,
    29: 0.99, 30: 1.01, 35: 1.06, 40: 1.10, 45: 1.29, 50: 1.34, 55: 1.40, 60: 1.40}
ARL_M3_LENGTHS: tuple[float, ...] = tuple(float(L) for L in ARL_M3_PRICES)


def arl_m3(length: float) -> str:
    return f"arl_m3_{length:g}"


for _L, _p in ARL_M3_PRICES.items():
    register(Item(
        arl_m3(_L), f"M3 x {_L:g} mm round aluminium standoff, 6 mm OD, female-female "
        f"(Hirosugi ARL-3{_L:g}BE)", "standoff",
        (Offer("Hirosugi-Keiki (MISUMI)", "https://hirosugi.jp/products/A/ARL-BE.html",
               f"ARL-3{_L:g}BE", price_usd=_p, verified=True,
               note=f"USD {_p:.2f} each on the maker's table (fetched 2026-10-05; MOQ 50 "
                    "direct, MISUMI resells)"),),
        dims={"d": 3.0, "od": 6.0, "length": float(_L),
              "thread_depth": float(_L) if _L <= 15 else 6.0, "id": 2.5, "yield_mpa": 240.0},
        notes="KS26 lead-free free-cutting aluminium, black anodised, L +/-0.1. The strength "
              "check takes a 6 x 2.5 tube (the M3 tap drill) at 240 MPa (UNVERIFIED for KS26).",
    ))

M3_SET_LENGTHS: tuple[float, ...] = (6, 8, 10, 12, 16)


def m3_set_screw(length: float) -> str:
    return f"m3_set_screw_{length:g}"


for _L in M3_SET_LENGTHS:
    register(Item(
        m3_set_screw(_L), f"M3 x {_L:g} mm set screw (ISO 4026, flat point)", "fastener",
        (Offer("McMaster-Carr", "https://www.mcmaster.com/products/set-screws/",
               pack_qty=50, note="M3 x 0.5 flat point, 18-8; part number not confirmed"),),
        dims={"d": 3.0, "length": float(_L)},
        notes="A splice's or a frame tie's stud.",
    ))
