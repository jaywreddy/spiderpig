"""Catalog data for an arm's robot-level parts (:mod:`construction.rotary`,
:mod:`construction.tools`): the turntable's shaft, bearings, collars, shims and wave washer,
the joints' homing sensors, the suction tool and its vacuum, the wrist's stepper.

Appended to the catalog from the end of :mod:`hardware.parts` (new items only). No price
is quoted from memory: the parts the arm buys have a marketplace listing first (Amazon
search-result prices of 2026-10-12, ``verified=False``; :func:`_amazon`), a price marked an
estimate where none was seen; the rest name a listing, unpriced, until a fetch or an order
confirms it (McMaster's through ``spiderpig cad``). A dimension a listing doesn't state
is marked so in the item's notes (measure the part).
"""

from __future__ import annotations

from spiderpig.hardware.catalog import Item, Offer, register
from spiderpig.hardware.catalog import amazon_offer as _amazon
from spiderpig.hardware.catalog import unpriced_offer as _unpriced

_MCM = "McMaster-Carr"


register(Item(
    "shaft_8mm_h6_tapped", "8 mm precision steel shaft, h6, one end tapped M4 x 10, cut to "
    "length (a MISUMI linear shaft, one end tapped, configured)", "shaft",
    (Offer("Misumi", "https://us.misumi-ec.com/vona2/detail/110302634310/", price_usd=10.0,
           note="the one-end-tapped family (the plain PSFJ has no taps): length and the M4 "
                "tap configured on the page; part number unconfirmed, the price an estimate "
                "(about USD 10, not seen: the configurator quotes it)"),),
    dims={"d": 8.0, "length": 100.0, "tap": "M4", "tap_depth": 10.0,
          "tolerance": "h6 (0 / -0.009)", "material": "steel"},
    notes="The turret's turning shaft (an h6 shaft runs from a 9 um gap to an 8 um "
          "interference in a 608's bore: the lower, floating bearing must slide on it for "
          "the wave washer to preload the pair; try the bearings on it): cut to length and "
          "tapped at the top by the seller (a stock configured part), an M4 screw and "
          "washer in its end clamping the pulley, the spacer and the turntable down on the "
          "over collar (round 7's review: a collar pushed down by hand clamps the stack with "
          "some 50-100 N, too little to hold the arm's moment without rocking in the "
          "bore)."))
register(Item(
    "m4_spacer_6", "Round aluminium spacer 4.2 x 8 x 6 mm (unthreaded)", "standoff",
    (Offer("Amazon", "https://www.amazon.com/s?k=M4+aluminum+round+spacer+6mm", pack_qty=20,
           price_usd=8.0, note="search; an estimate"),),
    dims={"id": 4.2, "od": 8.0, "length": 6.0, "material": "aluminium"},
    notes="Under a base_pair turret's bottom plate at its bench screws: the lower bearing "
          "and the column screws' heads stand under the plate."))
register(Item(
    "spiral_wrap_6mm", "Spiral cable wrap, 6 mm, per metre", "cable",
    (_amazon("B0918XNP8J", 3, 5.99, "CSZD spiral wrap 1/4 in (6 mm), 10 ft (3 m) a pack"),
     _unpriced("Amazon", "https://www.amazon.com/s?k=spiral+cable+wrap+6mm", "spiral wrap")),
    dims={"d": 6.0},
    notes="The turntable's cables (the steppers', the endstop's, the vacuum tube) bundled in "
          "a loose loop round the base: the fixed shaft is solid, nothing passes through it, "
          "so the loop takes the yaw's turn (no slip ring)."))
register(Item(
    "collar_8mm", "One-piece clamping shaft collar, 8 mm bore, 18 mm OD x 9 mm", "collar",
    (_amazon("B0DDC7JQXL", 4, 8.29, "PATIKIL two-piece clamping collar, 8 mm bore, steel, "
             "18 mm OD x 9 mm, 4 pcs (round 3 of the cost loop: the 25 mm one below clashes "
             "in the base_pair turret)"),
     _amazon("B0FFLT81TK", 6, 9.39, "uxcell one-piece clamping collar, 8 mm bore, 6 pcs: "
             "aluminium, 25 mm OD x 8 mm (the model's 18 x 9 steel collar is the clearance "
             "it keeps: check the 25 mm clears the plates and pulley before ordering)"),
     _unpriced(_MCM, "https://www.mcmaster.com/products/shaft-collars/", "8 mm bore "
               "one-piece clamping collars")),
    dims={"id": 8.0, "od": 18.0, "w": 9.0, "material": "steel"},
    notes="Clamping, not set-screw: a precision shaft is hardened and a set screw's cup "
          "point won't bite it; the clamp's face squares the shaft against the boss. Sizes "
          "are a typical listing's (UNVERIFIED: check the one bought)."))
register(Item(
    "shim_8x12x0p5", "Shim washer 8.2 x 12 x 0.5 mm, steel", "washer",
    (_unpriced(_MCM, "https://www.mcmaster.com/products/shims/", "8 mm shaft shims"),),
    dims={"id": 8.2, "od": 12.0, "t": 0.5, "material": "steel"},
    notes="Between a collar and a 608's inner ring: it bears on the inner ring only (its "
          "land is 12.1 mm), never on the shields."))
register(Item(
    "wave_washer_8mm", "Wave spring washer for 8 mm shaft, 8.4 x 12 mm", "washer",
    (_amazon("B00I2VV4VI", 25, 6.93, "Hillman M8 wave spring washer, zinc-plated steel, 25 "
             "pcs: its OD, height and load not listed (check it against this item's 12 mm "
             "OD and 30 N)"),
     _unpriced(_MCM, "https://www.mcmaster.com/products/wave-washers/", "8 mm wave "
               "washers")),
    dims={"id": 8.4, "od": 12.0, "t": 0.3, "free_h": 1.2, "work_h": 0.6,
          "load_n": 30.0, "material": "steel"},
    notes="The preloaded pair's spring, over the lower inner ring under the mid collar "
          "(:mod:`construction.rotary.base_pair`). Its load at the working height (30 N) is "
          "a typical catalogue figure, UNVERIFIED: check the listing."))
register(Item(
    "optical_endstop", "Slotted optical endstop module (TCST2103 type, 3D-printer board)",
    "electronics",
    (_amazon("B09BQ8SNMK", 6, 9.99, "optical endstop modules with 1 m cables, 6 pcs"),
     _unpriced("Amazon", "https://www.amazon.com/s?k=optical+endstop+tcst2103", "optical "
               "endstop boards")),
    dims={"board": [33.0, 10.5, 1.6], "slot_w": 3.0, "slot_h": 6.0, "body": [12.0, 6.0, 11.0],
          "hole_pitch": 19.0, "mass_g": 3.0},
    notes="The homing sensor: a flag through its slot breaks the beam. Board and body sizes "
          "are nominal (UNVERIFIED: measure the board bought)."))
register(Item(
    "suction_cup_m5_10", "Vacuum suction cup, 10 mm NBR, M5 male (Festo VAS-10-M5-NBR)",
    "pneumatic",
    (_amazon("B07MB8P3B2", 4, 8.18, "uxcell 10 mm suction cup, M5 male joint, silicone (not "
             "NBR: fine on clean glass), 4 pcs"),
     _unpriced("Festo", "https://www.festo.com/us/en/a/173440/", "VAS-10-M5-NBR")),
    dims={"d": 10.0, "h": 11.0, "thread": "M5", "mass_g": 2.0, "material": "nylon"},
    notes="The tool: picks glass slides by vacuum. Height is nominal (UNVERIFIED)."))
register(Item(
    "vacuum_pump_12v", "12 V diaphragm vacuum pump (micro 370, about -55 to -60 kPa)",
    "pneumatic",
    (Offer("Amazon", "https://www.amazon.com/dp/B0867NLHCF", "B0867NLHCF", pack_qty=2,
           price_usd=14.20, note="12 V micro 370 vacuum pump with inlet and vent, 2 a pack: "
           "0 to -55 kPa in a 100 cc tank in under 10 s at under 650 mA (its listing); "
           "product-page price read 2026-10-13"),
     _amazon("B0GQ9C828K", 1, 17.03, "CJVP28-AB12C 12 V micro 370 vacuum pump"),
     _unpriced("Amazon", "https://www.amazon.com/s?k=12v+micro+vacuum+pump", "micro "
               "vacuum pumps")),
    dims={"mass_g": 60.0},
    notes="Sits off the arm, on the bench; feeds the suction cup through the valve. At "
          "-55 kPa a 10 mm cup holds about 4 N (a glass slide weighs some 5 g)."))
register(Item(
    "solenoid_valve_12v_3way", "12 V 3-way solenoid valve (vents the cup to release)",
    "pneumatic",
    (_amazon("B07XCV799G", 2, 9.99, "uxcell 2-position 3-way 12 V 0.2 A air valve, 2 pcs"),
     _unpriced("Amazon", "https://www.amazon.com/s?k=12v+3+way+mini+solenoid+valve",
               "mini 3-way valves")),
    dims={"mass_g": 20.0}))
register(Item(
    "pu_tube_4mm", "PU tubing 4 mm OD x 2.5 mm ID, per metre", "pneumatic",
    (_amazon("B0DJZ1CRCC", 5, 6.89, "HARFINGTON PU tube 4 mm OD x 2.5 mm ID, 5 m a pack"),
     _unpriced("Amazon", "https://www.amazon.com/s?k=4mm+pu+tubing", "4 mm PU tube")),
    dims={"od": 4.0, "id": 2.5}))
register(Item(
    "barb_m5_4mm", "Push-in fitting, M5 male to 4 mm tube", "pneumatic",
    (_amazon("B07KGMBZZJ", 10, 5.83, "uxcell straight push-in fitting M5 male x 4 mm tube, "
             "10 pcs"),
     _unpriced("Amazon", "https://www.amazon.com/s?k=m5+push+in+fitting+4mm", "M5 to 4 mm "
               "fittings")),
    dims={"thread": "M5", "tube": 4.0, "mass_g": 3.0, "od": 10.0, "h": 14.0},
    notes="On the tool's shelf, its thread down into the cup's coupling nut (clamping the "
          "shelf). Body diameter and height over the shelf nominal (UNVERIFIED: measure)."))
register(Item(
    "m5_coupling_nut_15", "M5 coupling nut, 8 mm hex x 15 mm, threaded through (DIN 6334)",
    "nut",
    (_unpriced(_MCM, "https://www.mcmaster.com/products/coupling-nuts/", "M5 x 0.8 "
               "coupling nuts, 8 mm hex, 15 mm long"),),
    dims={"thread": "M5", "af": 8.0, "length": 15.0, "mass_g": 4.0, "material": "steel"},
    notes="The suction cup's column under the tool's shelf: the push-in fitting screwed into "
          "its top through the shelf, the cup into its bottom; the vacuum through its bore. "
          "The cup's and the fitting's threads each need about 3 mm in it (UNVERIFIED: the "
          "cup's thread length)."))
register(Item(
    "stepper_11hs20", "NEMA 11 stepper 11HS20-0674 (0.12 N·m, 0.67 A)", "motor",
    (_unpriced("StepperOnline", "https://www.omc-stepperonline.com/nema-11-stepper-motor",
               "11HS20-0674 (12 N·cm, 0.67 A)"),),
    dims={"mass_g": 200.0, "material": "steel"},
    notes="The wrist's motor. Holding torque and current from StepperOnline's listing; the "
          "51 mm length and 200 g are UNVERIFIED (measure)."))
register(Item(
    "inclinometer_scl3300", "Murata SCL3300-D01 inclinometer on a breakout board (SPI)",
    "sensor",
    (Offer("Mouser", "https://www.mouser.com/c/?q=SCL3300-D01", price_usd=30.0,
           note="the bare SCL3300-D01 about $17-25; on a small carrier board with its "
                "decoupling, about $30 (an estimate; Murata's eval board $77)"),),
    dims={"mass_g": 3.0, "resolution_deg": 0.0055, "noise_deg_rthz": 0.001,
          "offset_drift_deg_k": 0.005},
    notes="Bonded flat on the link, its x along the link, y in the link's plane: the "
          "link's angle from horizontal is atan2(x, y) of its accelerations "
          "(spiderpig.control.incline). Wired by a 4-wire SPI flex to the host."))
register(Item(
    "omron_ee_sx670", "Omron EE-SX670-WR slot-type photomicrosensor, 5 mm slot, pre-wired 1 m",
    "electronics",
    (Offer("Digi-Key", "https://www.digikey.com/en/products/result?keywords=EE-SX670-WR",
           price_usd=34.0, note="EE-SX670-WR 1M; the price from a search summary, unverified"),),
    dims={"slot_w": 5.0, "aperture_mm": 0.5, "repeat_um": 2.0, "mass_g": 25.0},
    notes="The precise homing switch (actuators.sensors, construction.rotary): its datasheet's "
          "reference graph repeats to 2 um over 20 tries at 25 C (5 um budgeted; 20-40 um of "
          "drift from 25 C to -25 / 55 C). Datasheet: https://www.farnell.com/datasheets/"
          "2574484.pdf. CAD: Omron's site needs an account (no free model found); a "
          "laser-cut 6061 flag through its slot (PLA passes infrared and creeps)."))
register(Item(
    "camera_rpi_module3_wide", "Raspberry Pi Camera Module 3 Wide (12 MP, autofocus from "
    "5 cm, 120 deg), CSI cable", "electronics",
    (Offer("Raspberry Pi", "https://www.raspberrypi.com/products/camera-module-3/",
           price_usd=35.0, note="the Wide; the list price"),
     _amazon("B07G9VLPZH", 1, 16.00, "an alternative, not the same camera: OV5647 5 MP with "
             "an M12 lens (90 deg), manual focus (set it once at the working height), no "
             "autofocus; 25 x 24 mm board (check its mounting)")),
    dims={"board": [25.0, 24.0, 12.4], "pixels": [4608, 2592], "min_focus_mm": 50.0,
          "mass_g": 4.0},
    notes="The homing reference's camera (control.vision): under the tool's shelf about 50 mm "
          "over the reference board, its lens locked once focused; on the Klipper host's "
          "(a Raspberry Pi's) CSI port. The standard module focuses only from 10 cm. "
          "Mechanical drawing and STEP: Raspberry Pi's product information portal "
          "(pip.raspberrypi.com)."))
register(Item(
    "camera_ov5647_m12", "OV5647 5 MP camera for the Raspberry Pi, M12 lens (about 90 deg), "
    "manual focus, CSI cable", "electronics",
    (_amazon("B07G9VLPZH", 1, 16.00, "OV5647 with an M12 lens; set its focus once at the "
             "working height and lock the lens"),),
    dims={"board": [25.0, 24.0, 12.4], "pixels": [2592, 1944], "fov_deg": 90.0,
          "mass_g": 4.0, "hole_pitch": [21.0, 12.5], "hole_d": 2.2, "lens_d": 14.0,
          "board_t": 1.0},
    notes="The cheaper homing camera (round 2 of the cost loop): about 39 um a pixel 50 mm "
          "over the board (the Camera Module 3 Wide's 27); a dot's centroid to a tenth of a "
          "pixel all the same, its focus fixed by hand (no autofocus to drift). Hung under "
          "the tool's shelf (construction.tools) on four M2 screws; its holes taken as the "
          "Raspberry Pi camera's 21 x 12.5 mm, the lens on their centre, its holder 14 mm "
          "across (UNVERIFIED: measure the board bought). Its cable: the kit's 15-pin one "
          "doesn't fit the Pi Zero 2 W's 22-pin port (csi_cable_pi_zero_30)."))
register(Item(
    "csi_cable_pi_zero_30", "Raspberry Pi Zero camera cable, 15-pin 1 mm to 22-pin 0.5 mm "
    "FFC, 30 cm", "electronics",
    (Offer("Amazon", "https://www.amazon.com/dp/B07H3L3LM2", "B07H3L3LM2", price_usd=4.99,
           note="DORHEA 30 cm Pi Zero camera cable, 15 pin 1.0 mm to 22 pin 0.5 mm; "
                "product-page price read 2026-10-13"),
     Offer("Amazon", "https://www.amazon.com/dp/B0DQBHNT9C", "B0DQBHNT9C", pack_qty=2,
           price_usd=8.99, note="ZDE, two 30 cm cables; product-page price read "
                                "2026-10-13")),
    dims={"length_mm": 300.0, "pins": [15, 22]},
    notes="The homing camera on the tool to the controller's Pi Zero 2 W (a 22-pin port). "
          "30 cm from the tool along the arm: check it reaches the Pi where it sits (a "
          "longer one, or a 15-pin extension through a joiner, if not)."))
register(Item(
    "fiducial_pcb", "Reference board: FR-4 1.6 mm, 100 x 100 mm, a grid of copper dots "
    "(ENIG), screwed to the base plate", "electronics",
    (Offer("JLCPCB", "https://jlcpcb.com/", price_usd=5.0, pack_qty=5,
           note="a two-layer board's minimum order (5), an estimate; the dots its copper"),),
    dims={"size_mm": [100.0, 100.0], "t": 1.6},
    notes="The homing reference's sheet (control.vision): copper dots etched in FR-4 hold "
          "their places against the air's humidity, where paper swells tens of um from day "
          "to day; the board's etch places them (its accuracy is the arm's after "
          "calibration; a chrome-on-glass target is finer)."))
register(Item(
    "bearing_f608zz", "F608ZZ flanged shielded ball bearing 8 x 22 x 7 mm, flange 25 x 1.5",
    "bearing",
    (Offer("AndyMark", "https://www.andymark.com/products/8mm-id-22mm-od-shielded-flanged-"
           "bearing-f608zz", price_usd=4.40, note="read 2026-10-12"),),
    dims={"id": 8.0, "od": 22.0, "w": 7.0, "flange_d": 25.0, "flange_t": 1.5,
          "radial_clearance_mm": [0.003, 0.018], "material": "steel", "mass_g": 13.0},
    notes="The base_pair turret's bearings (construction.rotary.base_pair): one bonded into "
          "each base plate, its flange on the plate's top face (it takes the arm's weight), "
          "the shaft through both as the jig while the epoxy cures. CN clearance of a 608 "
          "(3-18 um)."))
M4_HEX_STANDOFF_LENGTHS = (35, 40, 45, 50)
for _L in M4_HEX_STANDOFF_LENGTHS:
    register(Item(
        f"m4_hex_standoff_{_L}", f"M4 x {_L} mm hex standoff, female-female, 7 mm across "
        "flats, aluminium", "standoff",
        (Offer("Amazon", "https://www.amazon.com/s?k=M4+hex+standoff+female+female",
               pack_qty=10, price_usd=9.0, note="search; packs of 10, an estimate"),),
        dims={"length": float(_L), "af": 7.0, "thread": "M4", "material": "aluminium"},
        notes="A base_pair turret's column: seven of them between its plates carry the arm's "
              "tipping moment (a 7 mm hex racks about six times less than the M3 rounds)."))
register(Item(
    "angle_bracket_16", "Steel L angle bracket 16 x 16 x 12 mm, black (corner brace)",
    "bracket",
    (Offer("Amazon", "https://www.amazon.com/s?k=uxcell+16x16mm+angle+bracket+black",
           pack_qty=12, price_usd=8.0, note="search: uxcell 16 x 16 mm black L brackets, "
           "12 pcs; the price an estimate (2026-10-14)"),),
    dims={"w": 12.0, "leg": 16.0, "t": 1.6, "hole": 3.4, "hole_at": 8.8},
    notes="A frame plate's foot held to the turntable, the compact turret's (its turntable "
          "four millimetres less round than 20 mm brackets'). Width, thickness and the "
          "holes UNVERIFIED (a listing's photo): measure; a hole under 3.4 mm drilled out for "
          "M3."))
