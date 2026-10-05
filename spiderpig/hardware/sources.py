"""Where to buy every item the default robot's BOM lists: the sourcing round of 2026-10-05.

The default build (``spiderpig build``: the Strider double, Chicago pins, standoff pillars,
the bolt crank, the electronics deck) was sourced item by item for a **direct product page**
of the exact part (no search or category pages), preferring McMaster-Carr, DigiKey /
Mouser, Accu, MISUMI and the manufacturers' own stores over marketplaces. Each entry here
becomes its item's first (preferred) offer, ahead of what the item registered before
(:func:`apply`, called once the catalog has loaded); an item's modelling dimensions stay
with the item (they were corrected there where the part found differs: the Chicago screws'
heads, the PTFE washers, the M2.5 standoff's stud, the deck's switch).

``verified=True``: the page (or the maker's own datasheet for it) was fetched and showed
that part. McMaster-Carr, DigiKey and Mouser refuse a scripted fetch, so their part numbers
were confirmed from the vendor's own listing tables, a mirror of McMaster's spec table, the
manufacturer's datasheet or a search result quoting the page, and stay ``verified=False``
with how in the note. A price is per pack and only where a page showed one.

What has no better source than a marketplace: the IP2326 2S USB-C charger module (a generic
board, no distributor stocks one; measure the one you get) and the M3 Chicago screws (no
distributor sells M3 with a 4 mm barrel; uxcell's own store, Harfington, has every length).
"""

from __future__ import annotations

from dataclasses import replace

from spiderpig.hardware.catalog import CATALOG, Offer

_MCM = "McMaster-Carr"
_MCM_NOTE = ("part number from McMaster's spec table (reli-tool.com mirror) or listing; "
             "McMaster pages refuse a scripted fetch")
_ACCU_SHIM = {   # (id, od) -> {thickness: Accu product id}; SKU HSHN-<id>-<od>-<t>-A2
    (3, 6): {0.1: 62835, 0.2: 62836, 0.3: 62837, 0.5: 62839, 1.0: 62840},
    (4, 8): {0.1: 62854, 0.2: 62855, 0.3: 62856, 0.5: 399092, 1.0: 399093},
    (6, 12): {0.1: 62876, 0.2: 62878, 0.3: 62880, 0.5: 62881, 1.0: 62882},
}
_HARFINGTON_CHICAGO = {   # 18-8 plain: barrel length -> (variant id, SKU, sets per pack, USD)
    4: (47148414468345, "a24041700ux0598", 20, 6.84),
    5: (47148414501113, "a24041700ux0599", 20, 6.84),
    6: (47148413845753, "a24041700ux0600", 20, 7.06),
    8: (47148414533881, "a24041700ux0601", 20, 7.12),
    10: (47148414566649, "a24041700ux0602", 20, 7.27),
    12: (47148413812985, "a24041700ux0603", 20, 7.48),
    14: (47148414599417, "a24041700ux0604", 20, 7.18),
    16: (47148414632185, "a24041700ux0605", 10, 6.37),
    18: (47148414664953, "a24041700ux0606", 10, 6.62),
    20: (47148413944057, "a24041700ux0607", 10, 6.32),
    22: (47148414697721, "a24041700ux0608", 10, 6.76),
    30: (47148414730489, "a24041700ux0610", 10, 7.04),
    35: (47148414763257, "a24041700ux0611", 10, 6.87),
    40: (47148414796025, "a24041700ux0612", 10, 7.38),
    45: (47148414828793, "a24041700ux0613", 10, 7.70),
    50: (47148414861561, "a24041700ux0614", 10, 7.99),
    55: (47148414894329, "a24041700ux0615", 5, 6.66),
    60: (47148414927097, "a24041700ux0616", 5, 7.27),
    70: (47148414959865, "a24041700ux0617", 5, 7.00),
    80: (47148414992633, "a24041700ux0618", 5, 7.06),
}
_HARFINGTON_CHICAGO_BLACK = {   # 18-8 black zinc plated (p-1788004), every M3 length
    4: (46564350066937, "a25051200ux2380", 50, 8.79),
    5: (46564350099705, "a25051200ux2381", 50, 9.23),
    6: (46564350132473, "a25051200ux2382", 50, 9.20),
    7: (46564350165241, "a25051200ux2383", 50, 9.68),
    8: (46564350198009, "a25051200ux2384", 50, 9.80),
    9: (46564350230777, "a25051200ux2385", 50, 10.14),
    10: (46564350263545, "a25051200ux2386", 50, 9.59),
    11: (46564350296313, "a25051200ux2387", 50, 10.14),
    12: (46564350329081, "a25051200ux2388", 50, 10.25),
    13: (46564350361849, "a25051200ux2389", 50, 10.58),
    14: (46564350394617, "a25051200ux2390", 50, 10.80),
    15: (46564350427385, "a25051200ux2391", 50, 10.91),
    16: (46564350460153, "a25051200ux2392", 50, 11.56),
    18: (46564350492921, "a25051200ux2393", 50, 12.11),
    20: (46564350525689, "a25051200ux2394", 50, 12.23),
    22: (46564350558457, "a25051200ux2395", 50, 13.09),
    23: (46564350591225, "a25051200ux2396", 50, 13.74),
    25: (46564350623993, "a25051200ux2397", 50, 14.39),
    28: (46564350656761, "a25051200ux2398", 50, 15.71),
    30: (46564350689529, "a25051200ux2399", 50, 16.24),
    32: (46564350722297, "a25051200ux2400", 50, 16.77),
    33: (46564350755065, "a25051200ux2401", 50, 17.42),
    35: (46564350787833, "a25051200ux2402", 50, 17.96),
    38: (46564350820601, "a25051200ux2403", 50, 18.93),
    40: (46564350853369, "a25051200ux2404", 50, 18.49),
    43: (46564351836409, "a25051200ux2405", 50, 20.12),
    45: (46564350886137, "a25051200ux2406", 50, 20.86),
    48: (46564350918905, "a25051200ux2407", 50, 21.39),
    50: (46564350951673, "a25051200ux2408", 50, 20.83),
    55: (46564350984441, "a25051200ux2409", 50, 22.45),
    60: (46564351017209, "a25051200ux2410", 50, 23.31),
    65: (46564351049977, "a25051200ux2411", 20, 12.61),
    70: (46564351082745, "a25051200ux2412", 20, 13.09),
    75: (46564351115513, "a25051200ux2413", 20, 13.52),
    80: (46564351148281, "a25051200ux2414", 20, 14.09),
}
"""Harfington's two M3 Chicago series (product data fetched 2026-10-05). The plain one is
preferred (no plating on the 4 mm surface the links turn on); the black one has the
lengths it lacks (1 mm steps to 16, 23, 25, 28, ...): fastener_catalog.CHICAGO_LENGTHS is
its set. Heads 8.5 mm, barrel head 1.9 (plain) / 1.3 (black) tall, screw head 1.4 / 1.3:
the item models the taller."""

# McMaster's own listing pages (18-8 stainless ISO 7380 button heads, ISO 4026 flat-point set
# screws) rendered with each part number, length and pack: length -> (part number, pack)
_MCM_BHCS = {
    "3": {6: ("92095A179", 100), 8: ("92095A181", 100), 10: ("92095A182", 100),
          12: ("92095A183", 100), 16: ("92095A184", 100), 20: ("92095A185", 50),
          25: ("92095A186", 50), 30: ("92095A187", 50)},
    "4": {5: ("92095A477", 100), 6: ("92095A188", 100), 8: ("92095A189", 100),
          10: ("92095A190", 100), 12: ("92095A192", 100), 16: ("92095A194", 100)},
}
_MCM_M4_SET = {8: ("92605A113", 25), 10: ("92605A115", 50), 12: ("92605A117", 50),
               16: ("92605A121", 50)}
# black-oxide alloy steel 12.9 socket heads (McMaster's 18-8 M3 series stops at 20 mm), from
# the reli-tool.com mirror of McMaster's tables (not sequential past 25 mm); 25 mm and up are
# partially threaded (about 18 mm of thread)
_MCM_M3_SHCS = {6: "91290A111", 8: "91290A113", 10: "91290A115", 12: "91290A117",
                14: "91290A119", 16: "91290A120", 18: "91290A121", 20: "91290A123",
                25: "91290A125", 30: "91290A130", 35: "91290A135", 40: "91290A136",
                45: "91290A079", 50: "91290A137"}
# uxcell's black anodised 6 mm OD round M3 F-F aluminium standoffs, threaded through, 6 per
# pack (Harfington's product data, fetched 2026-10-05): length -> (handle, SKU, USD per pack).
# No threaded 6 mm OD x 5 mm exists (crank_catalog drops it).
_HARF_ROUND = {6: ("p-1633422", "a24061600ux0019", 7.11), 8: ("p-1633423", "a24061600ux0020", 7.35),
               10: ("p-1633424", "a24061600ux0021", 7.38),
               12: ("p-1633425", "a24061600ux0022", 8.26),
               15: ("p-1633426", "a24061600ux0023", 8.48),
               18: ("p-1633427", "a24061600ux0024", 8.92),
               20: ("p-1633428", "a24061600ux0025", 8.72),
               25: ("p-1633430", "a24061600ux0027", 9.11),
               30: ("p-1633431", "a24061600ux0028", 9.31)}


def _chicago_offers(length: int) -> tuple[Offer, ...]:
    """The plain M3 Chicago screw of this barrel length where Harfington has it, else (and as
    the alternative) the black zinc-plated one."""
    def offer(table, handle, finish, note):
        v, sku, n, usd = table[length]
        return Offer("Harfington (uxcell)", f"https://www.harfington.com/products/{handle}"
                     f"?variant={v}", sku, pack_qty=n, price_usd=usd, verified=True,
                     note=f"{finish} binding barrel and screw, M3, barrel 4.0 OD, heads 8.5 mm; "
                          f"{note}. No distributor (McMaster, Misumi, Accu) sells M3 Chicago "
                          "screws")
    out = []
    if length in _HARFINGTON_CHICAGO:
        out.append(offer(_HARFINGTON_CHICAGO, "p-1528133", "18-8 stainless",
                         f"select Thread Size M3, 4mm x {length}mm"))
    out.append(offer(_HARFINGTON_CHICAGO_BLACK, "p-1788004", "18-8 black zinc-plated",
                     f"select M3, {length}mm"))
    return tuple(out)


SOURCES: dict[str, tuple[Offer, ...]] = {
    **{f"m{d}_bhcs_{L}": (Offer(_MCM, f"https://www.mcmaster.com/{pn}/", pn, pack_qty=n,
                                verified=True,
                                note=f"18-8 stainless ISO 7380 M{d} x {L}; seen with its "
                                     "length and pack on McMaster's own listing (price "
                                     "shown only on the product page)"),)
       for d, by_l in _MCM_BHCS.items() for L, (pn, n) in by_l.items()},
    **{f"m4_set_screw_{L}": (Offer(_MCM, f"https://www.mcmaster.com/{pn}/", pn, pack_qty=n,
                                   verified=True,
                                   note=f"18-8 stainless ISO 4026 flat point M4 x {L}; seen on "
                                        "McMaster's own ISO 4026 listing"),)
       for L, (pn, n) in _MCM_M4_SET.items()},
    **{f"m3_shcs_{L}": (Offer(_MCM, f"https://www.mcmaster.com/{pn}/", pn, pack_qty=100,
                              note=f"black-oxide alloy steel 12.9 M3 x {L}"
                                   + (" (partially threaded)" if L >= 25 else "")
                                   + f"; {_MCM_NOTE}; pack size not confirmed"),)
       for L, pn in _MCM_M3_SHCS.items()},
    **{f"m3_round_standoff_ff_{L}": (Offer(
        "Harfington (uxcell)", f"https://www.harfington.com/products/{handle}", sku, pack_qty=6,
        price_usd=usd, verified=True,
        note=f"black anodised aluminium round 6 mm OD x {L} mm, M3 threaded through; OD "
             "tolerance not stated: check one in the 6.6 mm (+-0.3) hole"),)
       for L, (handle, sku, usd) in _HARF_ROUND.items()},
    # -- servo -------------------------------------------------------------------------
    "servo_sts3215": (
        Offer("Seeed Studio", "https://www.seeedstudio.com/STS3215-19kg-cm-7-4V-Serial-Servo-"
              "p-6338.html", "108090023", price_usd=21.99, verified=True,
              note="in stock 2026-10-05; the box has the aluminium disc horn, the idler horn, "
                   "case screws and the M3 horn screw (Waveshare's package photo; Feetech's "
                   "own datasheet says no accessories: buy where the horn kit is shown)"),
        Offer("Waveshare", "https://www.waveshare.com/st3215-servo.htm?sku=33014", "33014",
              price_usd=16.99, verified=True, note="the 7.4 V ST3215, in stock"),
    ),
    # -- screws, nuts, washers, inserts ------------------------------------------------
    "m3_nut": (Offer(_MCM, "https://www.mcmaster.com/91828A211/", "91828A211", pack_qty=100,
                     note="18-8 stainless, 5.5 AF x 2.4 mm; part number from NSN and "
                          "distributor records"),),
    "m3_washer_9021": (Offer(_MCM, "https://www.mcmaster.com/91116A120/", "91116A120",
                             pack_qty=100, note="18-8 stainless DIN 9021, 3.2 x 9 x 0.8; seen "
                                                "on McMaster's own DIN 9021 listing"),),
    "m4_washer": (Offer(_MCM, "https://www.mcmaster.com/93475A230/", "93475A230", pack_qty=100,
                        note="18-8 stainless DIN 125, 4.3 x 9 x 0.8; part number from NSN "
                             "and distributor records"),),
    "m3_heat_set_insert": (Offer("CNC Kitchen (US store)", "https://cnckitchenus.store/"
                                 "products/heat-set-insert-m3-x-5-7-100-pieces", "TC-M3x5.7",
                                 pack_qty=100, price_usd=10.90, verified=True,
                                 note="4.6 OD x 5.7 long, for a 4.0 mm hole"),),
    "m2_self_tap_6": (
        Offer("Accu", "https://accu-components.com/us/torx-pan-head-thread-forming-screws/"
              "993377-SHPRC-M2-6-CS-BZP", "SHPRC-M2-6-CS-BZP", pack_qty=1,
              note="M2 x 6 Torx pan head thread-former for plastics, head 4.0 x 1.72; buy only "
                   "if the servo's bag runs short: every STS3215 ships with its M2 x 6 case "
                   "screws (the SO-101 arm fastens its STS3215s with them and lists none)"),),
    "m25_nylon_screw_5": (
        Offer("DigiKey", "https://www.digikey.com/en/products/detail/essentra-components/"
              "50M025045N005/11638495", "50M025045N005", pack_qty=1,
              note="Essentra nylon 6/6 slotted pan head M2.5 x 5, head 5.0 mm (ISO 7045) "
                   "against the modelled 4.5: the board's pad takes it"),),
    "m25_nylon_nut": (
        Offer("DigiKey", "https://www.digikey.com/en/products/detail/essentra-components/"
              "04M025045HN/9677099", "04M025045HN", pack_qty=1,
              note="Essentra nylon 6/6 M2.5 hex nut, 5.0 AF x 2.0"),),
    "m25_nylon_standoff_mf_6": (
        Offer("Mouser", "https://www.mouser.com/ProductDetail/Wurth-Elektronik/971060155",
              "971060155", pack_qty=1, verified=True,
              note="Wurth WA-SPAIE nylon M2.5 hex 5 AF, 6 mm body, male-female, 8 mm stud "
                   "(the only one of this size confirmed: the item models its 8 mm stud)"),),
    # -- Chicago screws ----------------------------------------------------------------
    **{f"chicago_m3_{L}": _chicago_offers(L) for L in _HARFINGTON_CHICAGO_BLACK},
    # -- PTFE thrust washers -----------------------------------------------------------
    "ptfe_washer_4x8x0p5": (
        Offer("MISUMI (Hirosugi-Keiki)", "https://us.misumi-ec.com/vona2/detail/221006205109/"
              "?HissuCode=TT-0407-05", "TT-0407-05", pack_qty=50, price_usd=2.00, verified=True,
              note="PTFE 4.2 x 7.0 x 0.5, minimum order 50 (USD 0.04 each in the maker's "
                   "TT-0000-00 table); no 4.2 x 8 x 0.5 exists, TT-0408-05 is 4.5 x 8"),),
    "ptfe_washer_6x12x0p5": (
        Offer("MISUMI (Hirosugi-Keiki)", "https://us.misumi-ec.com/vona2/detail/221006205109/"
              "?HissuCode=TT-0612-05", "TT-0612-05", pack_qty=50, price_usd=4.00, verified=True,
              note="PTFE 6.3 x 12 x 0.5, minimum order 50 (USD 0.08 each in the maker's "
                   "TT-0000-00 table)"),),
    # -- DIN 988 shims, per thickness (hardware.shims) -----------------------------------
    **{f"shim_din988_{i}x{o}_t{t:g}".replace(".", "p"): (Offer(
        "Accu", f"https://accu-components.com/us/shim-washers/{pid}-HSHN-{i}-{o}-"
        f"{f'{t:g}'.replace('.', '-')}-A2", f"HSHN-{i}-{o}-{t:g}-A2", pack_qty=1, verified=True,
        note=f"A2 stainless DIN 988 shim {i} x {o} x {t:g} (the maker's datasheet), sold "
             "singly; tolerance +-0.05 mm (+-0.2 at 1 mm). True spring steel only in boxes of "
             "300+ (Aspen Fasteners ME313)"),)
       for (i, o), by_t in _ACCU_SHIM.items() for t, pid in by_t.items()},
    # -- adhesives and filament ----------------------------------------------------------
    "threadlocker_222": (
        Offer("Ellsworth Adhesives", "https://www.ellsworth.com/products/adhesives/anaerobic/"
              "henkel-loctite-222ms-threadlocker-anaerobic-adhesive-purple-10-ml-bottle/",
              "Loctite 222MS 10 ml (IDH 135333)", price_usd=21.05, verified=True,
              note="Henkel's distributor; 222 (IDH 231125) the same strength, backordered"),),
    "threadlocker_243": (
        Offer("Ellsworth Adhesives", "https://www.ellsworth.com/products/adhesives/anaerobic/"
              "henkel-loctite-243-threadlocker-anaerobic-adhesive-blue-10-ml-bottle/",
              "Loctite 243 10 ml (IDH 1329837)", price_usd=21.45, verified=True),
        Offer("DigiKey", "https://www.digikey.com/en/products/detail/loctite/1329837/"
              "10272563", "2275-1329837-ND", note="part number from a search result"),
    ),
    "epoxy_2part": (
        Offer("J-B Weld", "https://www.jbweld.com/product/j-b-weld-twin-tube", "8265S",
              price_usd=7.99, verified=True,
              note="J-B Weld Original, 2 x 1 oz, slow cure (dark grey)"),),
    "pla_filament": (
        Offer("Prusa Research", "https://www.prusa3d.com/product/prusament-pla-jet-black-1kg-"
              "nfc/", "Prusament PLA Jet Black 1kg", price_usd=29.99, verified=True),),
    "petg_filament": (
        Offer("Prusa Research", "https://www.prusa3d.com/product/prusament-petg-jet-black-1kg/",
              "Prusament PETG Jet Black 1kg", price_usd=29.99, verified=True),),
    "tpu95a_filament": (
        Offer("Polymaker", "https://shop.polymaker.com/products/polyflex-tpu95?variant="
              "39574341681209", "PD01001", price_usd=29.99, verified=True,
              note="PolyFlex TPU95, black, 1.75 mm, 0.75 kg"),),
    # -- electronics -------------------------------------------------------------------
    "esp32_servo_driver": (
        Offer("Waveshare", "https://www.waveshare.com/servo-driver-with-esp32.htm", "21593",
              price_usd=15.99, verified=True,
              note="65 x 30 mm, 2.75 mm holes at 58 x 23 (measured on Waveshare's STEP)"),),
    "lipo_2s_450": (
        Offer("Ovonic (maker's store)", "https://us.ovonicshop.com/products/4-x-ovonic-7-4v-80c-"
              "450mah-2s-lipo-battery-long-size-with-xt30-plug-for-fpv-freestyle-racing-drones-"
              "tiny-whoop-drones-quadcopter", "O-80C-450-2S1P-L-XT30-4P", pack_qty=4,
              price_usd=30.74, verified=True, note="61.9 x 16.3 x 13.4 mm, 28 g: the cradle's "
                                                     "size; four per pack"),),
    "ip2326_charger": (
        Offer("Amazon", "https://www.amazon.com/dp/B0GTNBCCQM", "B0GTNBCCQM",
              note="generic IP2326 2S USB-C module (no distributor stocks one); sizes run 30-40 x "
                   "20 x 5-6.9 mm: measure yours against the deck's 30 x 20 x 5 box"),),
    "bms_hx_2s_jh20": (
        Offer("diymore (brand store)", "https://www.diymore.cc/products/2s-10a-8-4v-7-4v-18650-"
              "lithium-protection-board-bms-pcm-pcb-li-ion-lipo-2-cell-pack-with-balance-"
              "function-charger-protect-module", "012759", price_usd=4.99, verified=True,
              note="HX-2S-JH20, 46.7 x 23 x 3.15 mm"),),
    "toggle_mts102": (
        Offer("DigiKey", "https://www.digikey.com/en/products/detail/e-switch/100SP1T1B1M1QEH/"
              "378819", "EG2350-ND", price_usd=3.36,
              note="price: the top of Octopart's distributor range ($2.31-3.36); "
                   "E-Switch 100SP1T1B1M1QEH SPDT mini toggle, 5 A at 28 VDC, 1/4-40 bushing "
                   "(6.35 mm, 8.89 long): a 6.5 mm hole. Two STS3215 stall at about 5 A, at the "
                   "design's 45 % torque limit about 2.4 A: don't switch with the servos "
                   "stalled. Part number from the datasheet and Octopart"),),
    "resistor_100k": (
        Offer("DigiKey", "https://www.digikey.com/en/products/detail/yageo/CFR-25JB-52-100K/245",
              "100KQBK-ND", price_usd=0.10, note="Yageo CFR-25JB-52-100K"),),
    "resistor_33k": (
        Offer("DigiKey", "https://www.digikey.com/en/products/detail/yageo/CFR-25JB-52-33K/1686",
              "33KQBK-ND", price_usd=0.10, note="Yageo CFR-25JB-52-33K"),),
    "xt30_pigtail_pair": (
        Offer("Rotor Riot", "https://rotorriot.com/products/xt30-pigtail", "RR1630",
              price_usd=1.49, verified=True,
              note="XT30 pigtail, 16 AWG, ~10 cm: one is enough (the battery has its own XT30)"),),
    "dc_plug_5521_pigtail": (
        Offer("DigiKey", "https://www.digikey.com/en/products/detail/tensility-international-"
              "corp/CA-2189/568580", "839-053-0188R-ND",
              note="Tensility CA-2189, right-angle 5.5 x 2.1 mm plug, 24 AWG pigtail: the deck "
                   "turns the board so its jack faces forward, out of the bay"),),
    "lipo_strap_10mm": (
        Offer("GetFPV", "https://www.getfpv.com/lumenier-indestructible-kevlar-lipo-strap-"
              "10x180mm-3pcs.html", "8872", pack_qty=3, price_usd=8.49, verified=True,
              note="Lumenier 10 x 180 mm Kevlar strap, 3 per pack"),),
    "foam_tape": (
        Offer("Ellsworth Adhesives", "https://www.ellsworth.com/products/by-manufacturer/3m/"
              "tapes/double-coated/structural-vhb/3m-vhb-tape-5952-gray-0.5-in-x-5-yd-roll/",
              "3M VHB 5952 1/2 in x 5 yd", verified=True,
              note="1.14 mm thick; price shown only to an account"),),
}


def apply() -> None:
    """Put each sourced offer first on its item (the item's own offers follow, minus any
    with the same URL)."""
    for key, offers in SOURCES.items():
        item = CATALOG.get(key)
        if item is None:
            continue
        urls = {o.url for o in offers}
        CATALOG[key] = replace(item, offers=offers + tuple(o for o in item.offers
                                                           if o.url not in urls))
