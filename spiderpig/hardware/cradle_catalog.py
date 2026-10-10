"""Catalog data: what a decanting cradle buys beyond the walkers' hardware.

The hand-cranked worm drive (:mod:`construction.handcrank`): the NRV030 gear reducer and its
output shaft, the keyed flange coupling the crank arm bolts to, the riser's spacers and
screws, the crankpin's shoulder screw and nut, the arm's screws; and the cradle's light
(:mod:`construction.cradle`). The tie rods and columns are the NETRF6 standoffs of
:mod:`hardware.crank_catalog`, their screws the M3 button heads of :mod:`hardware.parts`.

House rules as in :mod:`hardware.parts`: nothing priced from memory; ``verified`` only for a
page that was fetched and showed the product.
"""

from __future__ import annotations

from spiderpig.hardware.catalog import Item, Offer, register

SEARCH = "a search page, not a product: pick a listing that matches the dims"

MCM_SHOULDER_8_M6 = {6: ("92981A196", 4.16), 8: ("92981A198", 2.91),
                     10: ("92981A200", 2.81), 12: ("92981A201", 2.89),
                     16: ("92981A", None), 20: ("92981A", None), 25: ("92981A", None)}
"""McMaster 92981A, ISO 7379 8 mm shoulder, M6 x 11, head 13 x 6, black-oxide alloy steel
(140 ksi): shoulder length -> (part, USD each), the listing rendered 2026-10-10."""
SHOULDER_LENGTHS = tuple(MCM_SHOULDER_8_M6)
M6_SHCS_BD = {10: ("6416", 0.20, 14.20), 12: ("6417", 0.21, 14.81), 16: ("6418", 0.22, 16.06)}
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
M6_BHCS_BD = {16: ("6645", 0.20, 13.54), 30: ("6648", 0.35, 23.66), 35: ("6649", 0.38, 25.56),
              40: ("6650", 0.41, 27.44), 45: ("6651", 0.46, 30.62)}
"""Bolt Depot's 18-8 M6 button heads (the list page rendered 2026-10-10). The 25 mm is on
the same list (product number not read): ``m6_bhcs_25``."""

register(
    Item("nrv030_30", "NRV030 worm gear reducer, 30:1, 9 mm input shaft, 14 mm hollow output "
         "(Phidgets 20 N·m 30 Series)", "gearbox",
         (Offer("Phidgets", "https://www.phidgets.com/?prodid=868", "TRM4400_0",
                price_usd=48.10, verified=True, tiers=((1, 48.10), (5, 46.18), (10, 44.25)),
                note="30:1 only. 9 mm input with 3 mm keyway; 14 mm bore with 5 mm keyway; "
                     "20 N·m; 1.8 kN radial; backlash 1/2°; in stock. Drawing: productfiles/"
                     "TRM4400/TRM4400_0/Documentation/TRM4400_0Mechanical.pdf. Ships from "
                     "Calgary (shipping not checked). Page fetched 2026-10-10."),),
         dims={"ratio": 30, "centre_distance": 30.0, "input_d": 9.0, "input_len": 20.0,
               "input_shoulder": 45.0, "bore": 14.0, "pcd": 65.0, "face_thread": "M6x11",
               "spigot_d": 55.0, "width": 58.0, "mass_g": 1200.0},
         notes="Motovario's NRV030 mesh data at 30:1: static efficiency 0.39, under the 0.5 "
               "of static irreversibility: a load on the cradle can't turn the handle. Mass "
               "about 1.2 kg (a search snippet: weigh it)."),
    Item("nrv030_40", "NRV030 worm gear reducer, 40:1, 9 mm input shaft (NRV030-VS)", "gearbox",
         (Offer("eBay (foreign trade, Shenzhen)", "https://www.ebay.com/itm/355207733951",
                "355207733951", price_usd=95.00, verified=True,
                note="the 9 mm worm shaft out both ends; ratio by message to the seller "
                     "(no selector); +US$22.00 SpeedPAK, about 3 weeks; no output shaft. "
                     "Page rendered 2026-10-10."),),
         dims={"ratio": 40, "mass_g": 1200.0},
         notes="The finer alternative (20 turns a pour, not 15; static efficiency 0.35), "
               "not built by default: only marketplaces sell it."),
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
    *(Item(f"shoulder_m6x8_{n}", f"Shoulder screw ISO 7379, 8 mm shoulder x {n} mm, M6 thread",
           "fastener",
           (Offer("McMaster-Carr", f"https://www.mcmaster.com/{pn}/", pn, price_usd=usd,
                  verified=usd is not None,
                  note="black-oxide alloy steel, 140 ksi; sold each; listing rendered "
                       "2026-10-10" if usd is not None else "the 92981A family (8 mm "
                       "shoulder, M6) in this length: part number and price not read"),),
           dims={"d": 8.0, "length": float(n), "thread": "M6", "thread_len": 11.0,
                 "head_d": 13.0, "head_h": 6.0, "hex": 4.0})
      for n, (pn, usd) in MCM_SHOULDER_8_M6.items()),
    Item("m6_thin_nut", "M6 thin hex nut (DIN 439, 3.2 mm)", "nut",
         (Offer("Bolt Depot", BD + "13572", "13572", price_usd=0.10,
                tiers=((1, 0.10), (100, 0.0641)), verified=True,
                note="18-8 hex jam nut, 10 AF x 3.2; page rendered 2026-10-10"),),
         dims={"af": 10.0, "h": 3.2}),
    Item("m8_nut", "M8 hex nut (ISO 4032)", "nut",
         (Offer("Bolt Depot", BD + "4778", "4778", price_usd=0.17,
                tiers=((1, 0.17), (100, 0.1177)), verified=True,
                note="18-8; list page rendered 2026-10-10"),),
         dims={"af": 13.0, "h": 6.8}, notes="af/h nominal ISO 4032, not read off the page."),
    *(Item(f"m6_shcs_{n}", f"M6 x {n} mm socket head cap screw (ISO 4762)", "fastener",
           (Offer("Bolt Depot", BD + pn, pn, price_usd=each, verified=True,
                  tiers=((1, each), (100, per100 / 100)), note="DIN 912; " + BD_NOTE),),
           dims={"d": 6.0, "length": float(n), "head_d": 10.0, "head_h": 6.0})
      for n, (pn, each, per100) in M6_SHCS_BD.items()),
    *(Item(f"m6_bhcs_{n}", f"M6 x {n} button head socket screw (ISO 7380)", "fastener",
           (Offer("Bolt Depot", BD + pn, pn, price_usd=each, verified=True,
                  tiers=((1, each), (100, per100 / 100)), note=BD_NOTE),),
           dims={"d": 6.0, "length": float(n), "head_d": 10.5, "head_h": 3.3})
      for n, (pn, each, per100) in M6_BHCS_BD.items()),
    # the M6 Chicago pins' washers' sizes: what their printed head spacers and gap rings are
    # drawn to (materials.WASHERS; they are printed, not bought)
    Item("ptfe_washer_8x14x0p5", "PTFE flat washer 8.2 x 14 x 0.5 mm", "washer",
         (Offer("McMaster-Carr", "https://www.mcmaster.com/products/ptfe-washers/",
                note="PTFE washers for M8 / an 8 mm barrel; part number not confirmed (the "
                     "build prints this face as the head spacer)"),),
         dims={"id": 8.2, "od": 14.0, "t": 0.5}),
    Item("shim_din988_8x14", "DIN 988 shim ring 8 x 14 mm (0.1 / 0.2 / 0.3 / 0.5 / 1.0 mm)",
         "washer",
         (Offer("Accu", "https://accu-components.com/us/shim-washers/",
                note="DIN 988 shim rings 8 x 14; part number per thickness not confirmed"),),
         dims={"id": 8.0, "od": 14.0, "t": (0.1, 0.2, 0.3, 0.5, 1.0)},
         notes="Sizes the printed spacers on an M6 Chicago pin's 8 mm barrel (steps, OD)."),
    *(Item(f"m6_bhcs_{n}", f"M6 x {n} button head socket screw (ISO 7380)", "fastener",
           (Offer("Bolt Depot", "https://boltdepot.com/Metric_socket_button_head_Stainless_"
                  "steel_18-8_(A-2)_6mm_x_1.0mm", note=f"the 18-8 M6 button head list; the "
                  f"{n} mm row's product number and price not read (16 mm $0.20, 30 mm "
                  "$0.35)"),),
           dims={"d": 6.0, "length": float(n), "head_d": 10.5, "head_h": 3.3})
      for n in (20, 25)),
    Item("revolving_handle_m8", "Revolving machine handle, M8 stud (Ganter GN 798, 24 mm grip)",
         "misc",
         (Offer("JW Winco", "https://jwwinco.com/en-us/products/1.1-Operating-with-handles-and-"
                "knobs/revolving-handles/GN-798-Technopolymer-Plastic-or-Aluminum-Stepped-"
                "Cylindrical-Revolving-Handles-with-Steel-Threaded-Spindle", "GN 798-KT-24-M8",
                price_usd=6.08, verified=True,
                note="1-99 price; page fetched 2026-10-10. The grip spins: hold its collar "
                     "(or the stud's end) while tightening the nut behind the arm"),),
         dims={"thread": "M8", "thread_len": 16.0, "grip_d": 24.0, "grip_len": 74.0,
               "collar_d": 18.5, "length": 90.0},
         notes="GN 798 dims from Ganter's table as JW Winco's page gave it; overall length "
               "inferred: measure on arrival."),
    Item("penlight_aaa", "AAA LED penlight, twist switch (Maglite Solitaire LED)", "electronics",
         (Offer("Maglite", "https://maglite.com/products/solitaire-led-flashlight", "SJ3A",
                price_usd=8.99, verified=True, note="sale price (regular $9.99), page fetched "
                "2026-10-10; 47 lm; AAA included; shipping extra under $50"),
          Offer("Battery Junction", "https://www.batteryjunction.com/products/olight-i3e-eos",
                "I3E-EOS-19TH-ANNIVERSAR", price_usd=12.99, verified=True,
                note="Olight i3E EOS, 14 x 60.5 mm, 90 lm, twist (a shorter clip fits it); "
                     "page fetched 2026-10-10")),
         dims={"d": 12.7, "length": 81.0, "mass_g": 25.0},
         notes="The sommelier's candle: it rides the front saddle under the shoulder, its "
               "head (which twists to switch it on) up at the shoulder. Mass not on the page."),
)
