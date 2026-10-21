"""Catalog data: an arm's controller electronics (:mod:`actuators.controllers`, the role
``controller``): the board that drives its steppers, the computer that runs Klipper's host
and the camera, the power and the wiring.

The sourcing round of 2026-10-13 (loop 2's electronics): a BTT SKR Pico V1.0 (an RP2040
and four TMC2209 drivers on one UART, 12/24 V in, a 5 A 5 V buck for the computer), a
Raspberry Pi Zero 2 W on its header, a Mean Well 12 V 5 A brick through a DC jack's screw
terminal, three stepper cables (the motors' PH2.0-6 to the board's XH2.54-4), a 1N5819
flyback diode across the pump and the valve, and a 40 mm fan on the drivers (BTT's manual:
over 0.8 A a motor's driver wants a fan; the arm's NEMA 17s run at about 1 A).

``verified=True``: the page was fetched and showed the part and the price; the rest are a
web search's quote of the page (``verified=False``, said in the note).
"""

from __future__ import annotations

from spiderpig.hardware.catalog import Item, Offer, register

DATE = "2026-10-13"
FETCHED = f"product page fetched {DATE}"
QUOTED = f"price as a web search quoted this page on {DATE} (not fetched)"


def _jameco(pid: str, usd: float, pack: int = 1, note: str = "") -> Offer:
    return Offer("Jameco", "https://www.jameco.com/webapp/wcs/stores/servlet/ProductDisplay"
                 f"?langId=-1&storeId=10001&productId={pid}", pid, pack_qty=pack,
                 price_usd=usd, note=f"{note}; {QUOTED}" if note else QUOTED)


register(
    Item("skr_pico_v1", "BIGTREETECH SKR Pico V1.0 (RP2040, 4x TMC2209 UART, 12/24 V)",
         "electronics",
         (Offer("BIQU", "https://biqu.equipment/products/btt-skr-pico-v1-0", "SKR Pico V1.0",
                price_usd=39.85, verified=True,
                note=f"sale price (regular 55.68); {FETCHED}"),
          Offer("GitHub (BTT)", "https://github.com/bigtreetech/SKR-Pico", verified=True,
                note="the manual, the pin map and the schematic")),
         dims={"length": 85.0, "width": 56.0},
         notes="Four TMC2209s on one UART (gpio9 rx, gpio8 tx, addresses 0-3), each DIAG on "
               "a jumper to its endstop input; two MOSFET outputs (HE gpio23, HB gpio21) and "
               "three fan outputs; a TPS5450 5 A buck feeds the Pi's 5 V on its header. "
               "Flash Klipper's RP2040 build by its BOOT jumper and USB-C."),
    Item("pi_zero_2w_header", "Raspberry Pi Zero 2 W, header soldered", "electronics",
         (Offer("PiShop.us", "https://www.pishop.us/product/raspberry-pi-zero-2-w-with-header/",
                "Raspberry Pi Zero 2 W with header", price_usd=20.75, verified=True,
                note=FETCHED),),
         dims={"length": 65.0, "width": 30.0, "mass_g": 11.0},
         notes="Runs Klipper's host, Moonraker and the camera (its 22-pin CSI port: "
               "csi_cable_pi_zero_30); powered from the SKR Pico's 5 V, talks to it over the "
               "UART on the Pico's Pi header (/dev/ttyAMA0: enable_uart=1, "
               "dtoverlay=disable-bt)."),
    Item("microsd_32gb", "microSD card 32 GB, high endurance (SanDisk High Endurance)",
         "electronics",
         (Offer("Amazon", "https://www.amazon.com/s?k=SanDisk+High+Endurance+32GB+microSD",
                price_usd=10.99, note=f"search-result price {DATE}; the Raspberry Pi "
                "official 32 GB card is 19.95 at PiShop.us (fetched)"),),
         notes="The Pi's OS (Raspberry Pi OS Lite, KIAUH's Klipper and Moonraker)."),
    Item("psu_12v_5a_brick", "Mean Well GST60A12-P1J 12 V 5 A desktop supply, 5.5 x 2.1 mm "
         "plug, C14 inlet", "electronics",
         (_jameco("2224382", 19.40, note="the C13 mains cord is not included"),),
         notes="An enclosed brick: no mains terminals on the bench. The mains cord (IEC C13) "
               "is not included: use one on hand. 60 W covers the three motors' drivers, "
               "the pump and the Pi."),
    Item("dc_jack_terminal", "DC barrel jack 5.5 x 2.1 mm to screw terminal adapter (CUI "
         "CD022-JVP type)", "electronics",
         (_jameco("2153916", 3.95),),
         notes="The brick's plug to the SKR Pico's power screw terminal."),
    Item("stepper_cable_1m", "Stepper motor cable 1 m, JST PH2.0 6-pin to XH2.54 4-pin",
         "electronics",
         (_jameco("2333607", 1.95, note="JST-PH20-XH4P"),),
         notes="A 17HS4401's PH2.0-6 socket to the SKR Pico's XH2.54-4 motor header; check "
               "the coil pairs against the motor's (A+ A- B+ B-) before powering."),
    Item("diode_1n5819", "Schottky diode 1N5819, 40 V 1 A (DO-41)", "electronics",
         (_jameco("177965", 1.90, pack=10, note="0.19 each, in multiples of 10"),),
         notes="Across the pump's and the valve's leads (cathode to +12 V): their flyback "
               "beside the board's own SS14s."),
    Item("fan_4010_12v", "Fan 40 x 40 x 10 mm, 12 V (Sunon HA40101V4-0000-999)",
         "electronics",
         (_jameco("2167630", 6.95),),
         dims={"size": 40.0, "t": 10.0, "mass_g": 10.0},
         notes="On the drivers, from the 12 V input (always on): BTT's manual asks for a "
               "fan over 0.8 A a driver."),
)
