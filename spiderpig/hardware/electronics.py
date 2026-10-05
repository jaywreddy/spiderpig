"""Catalog data: the electronics the deck carries (:mod:`construction.deck`).

Option 1 of the electronics review (2026-10-03): a Waveshare "Servo Driver with ESP32"
(drives both STS3215s over their serial bus), a 2S 450 mAh LiPo, an IP2326 2S USB-C
charger, an HX-2S-JH20 protection board, a toggle switch and a 100k / 33k divider into
an ADC pin for the battery voltage. ``dims`` holds what the deck is drawn from (mm) and
``mass_g`` the mass :func:`hardware.mass.material_of` gives the part (spread over its
modelled box) instead of volume x density.

Sources (research agents' web lookups of 2026-10-03, so ``verified=False`` unless the
page itself was fetched): the Waveshare wiki and product page (65 x 30 mm, 21 g, 2.75 mm
holes on 58 x 23 mm, 5.5 x 2.1 mm DC jack 6-12 V, USB-C, $15.99); its photo puts the DC
jack on one short end and the USB-C at the other; the component heights are NOT on the
wiki (its STEP model, ``Servo_Driver_with_ESP32_STEP.zip``, has them): the barrel jack is
taken as 11 mm, the usual height of a 5.5 x 2.1 mm PCB jack. The LiPo is the Ovonic 2S
450 mAh 80C "long" (61.9 x 16.3 x 13.4 mm, 28 g, XT30; Amazon 4-pack $29.69 with a
coupon, search snippet). The IP2326 module's listings say 30-32 x 18-20 x 5 mm (the 5 mm
may exclude the USB-C receptacle; mass unlisted, a bare module of this size is 2-4 g);
the HX-2S-JH20 is 46.7 x 23 x 3.15 mm, rated 10 A by most listings (20 A peak), mass
unlisted. The MTS-102 toggle (SPDT, 6 mm bushing, 2-3 A at 250 V AC: fine at 8.4 V DC)
is about 4.4 g (a 5-pack's listed 22 g). Unverified numbers say so in ``notes``; weigh
and measure the parts on arrival.
"""

from __future__ import annotations

from spiderpig.hardware.catalog import Item, Offer, register

SEARCHED = "price as a web search quoted it from this page on 2026-10-03 (not fetched)"

register(
    Item("esp32_servo_driver", "Waveshare Servo Driver with ESP32 (serial bus servo driver)",
         "electronics",
         (Offer("Waveshare", "https://www.waveshare.com/servo-driver-with-esp32.htm",
                "Servo Driver with ESP32", price_usd=15.99,
                note="price from the product page as a research agent read it on 2026-10-03"),
          Offer("Waveshare wiki", "https://www.waveshare.com/wiki/Servo_Driver_with_ESP32",
                verified=True, note="65 x 30 mm, holes 2.75 mm on 58 x 23 mm; STEP model "
                "and schematic under Resources")),
         dims={"length": 65.0, "width": 30.0, "pcb": 1.6, "hole_d": 2.75,
               "hole_pitch": (58.0, 23.0), "jack_h": 7.3, "parts_h": 5.7, "mass_g": 21.0},
         notes="Measured on Waveshare's STEP model (2026-10-05): the DC jack (5.5 x 2.1 mm, "
               "6-12 V) on one short end, 7.3 mm tall, 0.9 mm past the board's end; the servo "
               "headers 5.7 mm tall across the board; the USB-C on a long edge 47-57 mm from "
               "the jack end (not the other short end); the ESP32 module under the board, "
               "2.3 mm down. M2.5 screws."),
    Item("lipo_2s_450", "2S 7.4 V 450 mAh LiPo, XT30 (Ovonic 80C long size)", "electronics",
         (Offer("Amazon", "https://www.amazon.com/OVONIC-Battery-Package-Including-Batteries/"
                "dp/B0D3F6BRB9", "B0D3F6BRB9", pack_qty=4, price_usd=29.69,
                note=f"4-pack, 61.9 x 16.3 x 13.4 mm, 28 g; {SEARCHED} (with a 10 % coupon)"),
          Offer("Ampow", "https://www.ampow.com/products/ovonic-70c-7-4v-450mah-2s1p-xt30-"
                "4pcs-lipo-battery", note="Ovonic 70C 61 x 16.5 x 13.4 mm, 30.6 g: fits the "
                "same cradle")),
         dims={"length": 61.9, "width": 16.3, "height": 13.4, "mass_g": 28.0},
         notes="The cradle is drawn for 61.9 x 16.3 mm plus 0.3 mm; the 100C pack (62 x 17 x "
               "14 mm, 31 g) needs the cradle's clearance raised. Leads exit the inner end."),
    Item("ip2326_charger", "IP2326 2S USB-C charger module (5 V in, 8.4 V / 1.5 A out)",
         "electronics",
         (Offer("AliExpress", "https://www.aliexpress.us/item/3256808840546226.html",
                price_usd=1.09, note=SEARCHED),
          Offer("AAENICS", "https://store.aaenics.com/product/ip2326-lithium-battery-fast-"
                "charging-module-2s-3s-15w/", note="32 x 18 x 5 mm per the listing")),
         dims={"length": 30.0, "width": 20.0, "height": 5.0, "mass_g": 3.0},
         notes="Listings say 30-32 x 18-20 x 5 mm (UNVERIFIED; 5 mm may exclude the USB-C "
               "receptacle); mass unlisted, 3 g assumed. Its USB-C end sits flush with the "
               "deck's front edge. Set it to 2S (8.4 V)."),
    Item("bms_hx_2s_jh20", "HX-2S-JH20 2S protection board with balancing (10 A)",
         "electronics",
         (Offer("Banggood", "https://usa.banggood.com/HX-2S-JH20-2S-7_4V-8_4V-18650-Lithium-"
                "Battery-Protection-Board-with-Equalization-Overcharged-Protection-p-1816422."
                "html", price_usd=3.99, note=SEARCHED),
          Offer("Amazon", "https://www.amazon.com/HX-2S-JH20-Protection-Balanced-Function-"
                "Overcharged/dp/B0DTPHVHLL", "B0DTPHVHLL", note="46.7 x 23 x 3.15 mm")),
         dims={"length": 46.7, "width": 23.0, "height": 3.15, "mass_g": 3.0},
         notes="Over-discharge 2.9 V/cell, overcharge 4.25 V/cell. Mass unlisted (3 g "
               "assumed). The ESP32 also cuts the drives at 3.3 V/cell from the divider."),
    Item("toggle_mts102", "Mini toggle switch, SPDT, 1/4-40 bushing (E-Switch 100SP1T1B1M1QEH)",
         "electronics",
         (Offer("eBay", "https://www.ebay.com/itm/335475840428", pack_qty=20, price_usd=5.50,
                note=SEARCHED),
          Offer("Amazon", "https://www.amazon.com/MTS-102-Toggle-Switch-Position-125VAC/dp/"
                "B07TS92M8H", "B07TS92M8H", pack_qty=10, note="5-6 A at 125 V AC; price not "
                "seen")),
         dims={"bushing_d": 6.35, "hole_d": 6.5, "body": (13.0, 8.0, 10.0), "lugs": 6.0,
               "bushing_h": 8.89, "lever_h": 10.0, "mass_g": 4.4},
         notes="E-Switch 100 series (datasheet, 2026-10-05): 1/4-40 bushing 6.35 x 8.89 mm, "
               "body 12.70 x 6.86 x 8.89 mm plus 3.96 mm lugs (the model keeps the larger "
               "13 x 8 x 10 + 6 envelope of the MTS-102 it replaced), 5 A at 28 VDC; 4.4 g "
               "taken from an MTS-102."),
    Item("resistor_100k", "100 kOhm 1/4 W resistor (battery divider, top)", "electronics",
         (Offer("DigiKey", "https://www.digikey.com/en/products/detail/yageo/CFR-25JB-52-100K"
                "/245", "CFR-25JB-52-100K", price_usd=0.10, note=SEARCHED),)),
    Item("resistor_33k", "33 kOhm 1/4 W resistor (battery divider, bottom)", "electronics",
         (Offer("DigiKey", "https://www.digikey.com/en/products/detail/yageo/CFR-25JB-52-33K/"
                "1686", "CFR-25JB-52-33K", price_usd=0.10, note=SEARCHED),)),
    Item("lipo_strap_10mm", "Hook-and-loop battery strap, 10 x 130 mm (iFlight 10-pack)",
         "electronics",
         (Offer("Amazon", "https://www.amazon.com/iFlight-Rubberized-Non-Slip-Fastening-"
                "Quadcopter/dp/B07XL8NLLZ", "B07XL8NLLZ", pack_qty=10,
                note="rubberized, metal buckle; price not seen"),),
         notes="Loops through the deck's two strap slots under the deck and over the "
               "battery: 2 x (13.4 + 3) + 2 x 17 + 10 = ~77 mm of a 130 mm strap."),
    Item("xt30_pigtail_pair", "XT30 male + female pigtail pair, 16 AWG, 10 cm", "electronics",
         (Offer("Amazon", "https://www.amazon.com/Female-Connector-Extension-Silicone-Battery/"
                "dp/B0D2V8ZN9V", "B0D2V8ZN9V", pack_qty=5, price_usd=8.99, note=SEARCHED),)),
    Item("dc_plug_5521_pigtail", "5.5 x 2.1 mm DC plug pigtail (male), 15 cm", "electronics",
         (Offer("Amazon", "https://www.amazon.com/JacobsParts-Pigtail-Security-Voltage-"
                "Applications/dp/B00R1XZ09K", "B00R1XZ09K", pack_qty=5,
                note="price not seen"),),
         notes="Switched battery to the driver board's DC jack."),
    Item("foam_tape", "Double-sided foam mounting tape, 1 mm (3M VHB-style), 19 mm roll",
         "adhesive",
         (Offer("Amazon", "https://www.amazon.com/s?k=double+sided+foam+tape+1mm",
                note="search; any 1 mm foam tape"),),
         notes="The charger and the protection board stick under the deck (0.3 mm modelled "
               "gap; 1 mm tape lowers them 0.7 mm more, still clear of the servos)."),
)

# The board's M2.5 nylon hardware: one kit (vendor + SKU shared, so the BOM buys one pack).
_KIT = Offer("Amazon", "https://www.amazon.com/clp/B08XLHQVWM", "B08XLHQVWM", pack_qty=360,
             note="Heayzoki 360 pc M2 / M2.5 / M3 nylon standoff kit; price not seen")
register(
    Item("m25_nylon_standoff_mf_6", "M2.5 x 6 mm nylon hex standoff, male-female", "standoff",
         (_KIT,), dims={"af": 5.0, "length": 6.0, "thread": 8.0,   # Wurth 971060155
                        "mass_g": 0.15}),
    Item("m25_nylon_nut", "M2.5 nylon hex nut", "nut", (_KIT,),
         dims={"af": 5.0, "h": 2.0, "mass_g": 0.05}),
    Item("m25_nylon_screw_5", "M2.5 x 5 mm nylon pan head screw", "fastener", (_KIT,),
         dims={"d": 2.5, "length": 5.0, "head_d": 5.0, "head_h": 1.7, "mass_g": 0.05}),  # ISO 7045
)
