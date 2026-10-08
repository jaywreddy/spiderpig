"""Catalog data for the bolt crank (:class:`construction.crank.BoltCrank`), the standoff
pillar (:mod:`construction.pivots.standoff`) and the frame ties (:mod:`construction.chassis`).

Appended to the catalog from the end of :mod:`hardware.parts` (new items only). First
searched 2026-10-03; an offer is ``verified=True`` only where the page was fetched or
rendered and showed the product. Prices a search result quoted (not fetched) say so in their
note. The default build's items take their first offer from :mod:`hardware.sources`.

What the default robot uses:

* ``hex_standoff_m3_<L>``: M3 x 5.5 AF female-female steel hex standoffs, the hex crank's
  crankpins and journals (:data:`HEX_M3_LENGTHS`; Wurth WA-SSTII at Mouser up to 40 mm),
  with ``m3_washer_9021`` under each end's M3 button head.
* ``pillar_shaft_6_m3_<L>``: MISUMI NETRF6, a 6 mm round 1018 steel standoff tapped M3 both
  ends, made to any length from 8 to 300 mm in 0.1 mm steps: a pillar column no single
  goBILDA length fills, in one piece (:data:`PILLAR_SHAFT_LENGTHS`).
* ``m3_round_standoff_ff_<L>``: uxcell 6 mm OD round aluminium M3 female-female standoffs
  (:data:`M3_ROUND_STANDOFF_LENGTHS`): the crank's journal stub, the frame ties' chains and
  ``m3_set_screw_<L>`` (ISO 4026) joins the ties' chains.
* ``gobilda_1501_<L>``: goBILDA 1501 series M4 x 0.7 round aluminium standoffs, 6 mm OD: a
  pillar column one stock length fills, and the round crankpin; only the lengths
  goBILDA sells (:data:`GOBILDA_LENGTHS`, their M4 standoff listing fetched 2026-10-04 with
  each 4-pack's price: 3-12, then 14-60 mm in mostly 2 mm steps, plus 19, 27 and 43; 13, 15,
  17, 21, 23, 25, 29, 31, 33, 35, 37, 39, 41, 45, 47, 49, 51, 53, 55, 57 and 59 mm don't
  exist: the 33, 39 and 45 mm product pages answer 404).
* ``m4_bhcs_<L>``, ``m4_washer``, ``m4_set_screw_<L>``: a goBILDA column's end screws (an
  ISO 7380 button head and a DIN 125 washer, 3.0 mm together: one layer outside each frame
  plate), and the stud joining a round crankpin's two standoffs (ISO 4026).

Kept for the comparisons, and the round crankpin's washers:

* ``threadlocker_243``: medium strength, on the crankpins' screws.
* ``ptfe_washer_6x12x0p5``: the thrust washer the round crankpin's runs carry through a
  clearance gap (:func:`materials.washer_stack`).
"""

from __future__ import annotations

from spiderpig.hardware.catalog import CATALOG, Item, Offer, register, register_factory

SEARCHED = "as a 2026-10-03 web search quoted the page (not fetched)"
FETCHED = "fetched 2026-10-04"

register(
    Item("threadlocker_243", "Medium-strength threadlocker (Loctite 243 or equivalent), 10 ml",
         "adhesive",
         (Offer("Amazon", "https://www.amazon.com/s?k=Loctite+243+10ml",
                note="search; blue, removable with hand tools; oil tolerant"),),
         dims={"breakaway_m10_nm": 26.0},
         notes="On the crankpins' screws. The TDS gives a breakaway torque of about 26 N·m "
               "on M10 steel nuts and bolts; on plated or stainless (passive) surfaces less: "
               "the strength check takes half, scaled to the thread (UNVERIFIED: a test of one "
               "joint settles it)."),
    Item("ptfe_washer_6x12x0p5", "PTFE flat washer 6.3 x 12 x 0.5 mm", "washer",
         (Offer("eBay", "https://www.ebay.com/itm/121716489464",
                note=f"PTFE M6 6.4 x 12 mm, 50 pcs ({SEARCHED}; thickness not stated: 1.5-2 "
                     "mm is the common size, 0.5 mm is rare)"),),
         dims={"id": 6.3, "od": 12.0, "t": 0.5},   # MISUMI TT-0612-05
         notes="On the round crankpin's runs through a clearance gap (materials.washer_stack)."),
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
              "material": "aluminium",
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
PILLAR_SHAFT_TIERS = {62: ((1, 14.97), (5, 10.86), (10, 5.47), (20, 5.36)),
                      128: ((1, 15.32), (5, 11.12), (10, 5.61))}
"""USD each from 1, 5, 10 (and 20) pieces of one length (NETRF6-62 and NETRF6-128): the
MISUMI page rendered 2026-10-05. The discount is per line, so five cost less than four and
ten less than six: the BOM buys at the break (:meth:`catalog.Offer.buy`). A length under
95 mm is priced as the 62, a longer one as the 128."""


PILLAR_SHAFT_SEEN: tuple[float, ...] = (62.0, 62.4, 127.5, 128.0, 128.1)
"""The NETRF6 lengths whose MISUMI page was rendered (2026-10-05) with the part and its price
(62.4 and 128.1: the Strider double's and quad's columns, unit price as 62 and 128)."""


def pillar_shaft(length: float) -> str:
    """The catalog key of the NETRF6 standoff ``length`` mm long (registered on first use)."""
    key = f"pillar_shaft_6_m3_{length:g}"
    CATALOG.get(key)            # (made through the catalog's factory: sourced, locked)
    return key


def _pillar_shaft_item(L: float) -> Item:
    pn = f"NETRF6-{L:g}"
    return Item(
        f"pillar_shaft_6_m3_{L:g}", f"6 mm round steel standoff, {L:g} mm, tapped M3 both ends "
        f"(MISUMI {pn})", "standoff",
        (Offer("MISUMI", "https://us.misumi-ec.com/vona2/detail/110300208270/?HissuCode="
               + pn, pn,
               price_usd=PILLAR_SHAFT_TIERS[62 if L < 95 else 128][0][1],
               tiers=PILLAR_SHAFT_TIERS[62 if L < 95 else 128],
               verified=L in PILLAR_SHAFT_SEEN,
               note="MISUMI circular standoff, tapped both ends, configurable length: 1018 "
                    "steel, oiled (no plating), 6 mm OD (0/-0.1), M3 x 6 deep each end, length "
                    "+-0.1 mm in 0.1 mm steps (the part number's number). Sold singly: USD "
                    "15.32 each at 1-4, 11.12 at 5-9, 5.61 at 10-19 (NETRF6-128; NETRF6-62 "
                    "14.97 / 10.86 / 5.47, 5.36 at 20-50; 2026-10-05), the discount per "
                    "line: order the quantity the BOM says (4 needed: buy 5, at USD 55.60 "
                    "less than 4 at 61.28; 6-9 needed: buy 10)"),),
        dims={"d": 3.0, "od": 6.0, "length": L, "thread_depth": 6.0, "id": PILLAR_SHAFT_ID,
              "yield_mpa": PILLAR_SHAFT_YIELD},
        notes="A standoff pillar's column no single goBILDA length fills (longer than 60 mm, or "
              "a length goBILDA lacks): one piece made to its length, never spliced. The "
              "strength check takes it as a 6 x 2.5 tube (the M3 tap drill, as if tapped "
              "through) of 1018 at 220 MPa. Oiled bare steel: wipe it, and keep it dry.",
    )


def _pillar_shaft_of(key: str) -> Item | None:
    """The item of a ``pillar_shaft_6_m3_<L>`` key: one of :data:`PILLAR_SHAFT_LENGTHS`
    written as :func:`pillar_shaft` writes it, else ``None``. The 2,921 lengths are made on
    demand (:func:`catalog.register_factory`), not registered at import."""
    try:
        n = round(float(key.removeprefix("pillar_shaft_6_m3_")) * 10)
    except ValueError:
        return None
    L = n / 10
    if not 80 <= n <= 3000 or f"pillar_shaft_6_m3_{L:g}" != key:
        return None
    return _pillar_shaft_item(L)


register_factory("pillar_shaft_6_m3_", _pillar_shaft_of)
register(*(_pillar_shaft_item(L) for L in PILLAR_SHAFT_SEEN))   # the lengths priced

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
        notes="The stud joining a round crankpin's two goBILDA standoffs (bolt_round), "
              "threaded half into each.",
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
# series every hex-spacer vendor stocks; 5-40 mm are Wurth's WA-SSTII parts at Mouser
# (:data:`WURTH_HEX_LENGTHS`, datasheets fetched 2026-10-05; priced where the default designs
# use them, :data:`WURTH_HEX_PRICES`), 50 and 60 mm Vital Parts' or Accu's pages
# (:data:`LONG_HEX_PAGES`), 45 mm only in Accu's length selector.

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


# -- M3 set screws (the M3 hardware comparison, 2026-10-05) ------------------------------
#
# ``m3_set_screw_<L>``: ISO 4026 M3 flat point set screws (the frame ties' studs).

M3_SET_LENGTHS: tuple[float, ...] = (6, 8, 10, 12, 16)


def m3_set_screw(length: float) -> str:
    return f"m3_set_screw_{length:g}"


for _L in M3_SET_LENGTHS:
    register(Item(
        m3_set_screw(_L), f"M3 x {_L:g} mm set screw (ISO 4026, flat point)", "fastener",
        (Offer("McMaster-Carr", "https://www.mcmaster.com/products/set-screws/",
               pack_qty=50, note="M3 x 0.5 flat point, 18-8; part number not confirmed"),),
        dims={"d": 3.0, "length": float(_L)},
        notes="A frame tie's stud.",
    ))
