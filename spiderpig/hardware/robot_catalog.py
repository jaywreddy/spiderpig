"""Catalog data for an arm's robot-level parts (:mod:`construction.rotary`,
:mod:`construction.tools`): the turntable's shaft, collars, shims and wave washer, the
homing sensor, the suction tool and its vacuum, the wrist's stepper.

Appended to the catalog from the end of :mod:`hardware.parts` (new items only). No price
is quoted from memory: every offer names a listing, unpriced, until a fetch or an order
confirms it (McMaster's through ``spiderpig cad``). A dimension a listing doesn't state
is marked so in the item's notes (measure the part).
"""

from __future__ import annotations

from spiderpig.hardware.catalog import Item, Offer, register

_MCM = "McMaster-Carr"


def _unpriced(vendor: str, url: str, note: str) -> Offer:
    return Offer(vendor, url, None, note=f"{note}; not fetched: unpriced")


register(Item(
    "shaft_8mm_h6_100", "8 mm precision steel shaft, h6, 100 mm (cut to length)", "shaft",
    (_unpriced(_MCM, "https://www.mcmaster.com/products/rotary-shafts/", "8 mm h6 "
               "precision shafts"),),
    dims={"d": 8.0, "length": 100.0, "tolerance": "h6 (0 / -0.009)", "material": "steel"},
    notes="The turntable's fixed shaft: clamped by collars either side of the base's boss, "
          "the hub's two 608 bearings turn on it. An h6 shaft runs from a 9 um gap to an 8 um "
          "interference in a 608's bore: the upper bearing must slide on it for the wave "
          "washer to preload the pair (try the bearings on it; polish, or pick another). In "
          "the base_pair turret it turns, in the base's two bearings: the lower (floating) one "
          "must slide on it."))
register(Item(
    "shaft_8mm_h6_tapped", "8 mm precision steel shaft, h6, one end tapped M4 x 10, cut to "
    "length (Misumi PSFJ type, configured)", "shaft",
    (_unpriced("Misumi", "https://us.misumi-ec.com/vona2/detail/110302634310/",
               "PSFJ8, length and the M4 tap configured on the page"),),
    dims={"d": 8.0, "length": 100.0, "tap": "M4", "tap_depth": 10.0,
          "tolerance": "h6 (0 / -0.009)", "material": "steel"},
    notes="The base_pair turret's turning shaft: cut to length and tapped at the top by the "
          "seller (a stock configured part), an M4 screw and washer in its end clamping the "
          "pulley, the spacer and the turntable down on the over collar (round 7's review: a "
          "collar pushed down by hand clamps the stack with some 50-100 N, too little to "
          "hold the arm's moment without rocking in the bore)."))
register(Item(
    "m4_spacer_6", "Round aluminium spacer 4.2 x 8 x 6 mm (unthreaded)", "standoff",
    (Offer("Amazon", "https://www.amazon.com/s?k=M4+aluminum+round+spacer+6mm", pack_qty=20,
           price_usd=8.0, note="search; an estimate"),),
    dims={"id": 4.2, "od": 8.0, "length": 6.0, "material": "aluminium"},
    notes="Under a base_pair turret's bottom plate at its bench screws: the lower bearing "
          "and the column screws' heads stand under the plate."))
register(Item(
    "spiral_wrap_6mm", "Spiral cable wrap, 6 mm, per metre", "cable",
    (_unpriced("Amazon", "https://www.amazon.com/s?k=spiral+cable+wrap+6mm", "spiral wrap"),),
    dims={"d": 6.0},
    notes="The turntable's cables (the steppers', the endstop's, the vacuum tube) bundled in "
          "a loose loop round the base: the fixed shaft is solid, nothing passes through it, "
          "so the loop takes the yaw's turn (no slip ring)."))
register(Item(
    "collar_8mm", "One-piece clamping shaft collar, 8 mm bore, 18 mm OD x 9 mm", "collar",
    (_unpriced(_MCM, "https://www.mcmaster.com/products/shaft-collars/", "8 mm bore "
               "one-piece clamping collars"),),
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
    (_unpriced(_MCM, "https://www.mcmaster.com/products/wave-washers/", "8 mm wave "
               "washers"),),
    dims={"id": 8.4, "od": 12.0, "t": 0.3, "free_h": 1.2, "work_h": 0.6,
          "load_n": 30.0, "material": "steel"},
    notes="The preloaded pair's spring, under the top collar on the upper inner ring (in "
          "the base_pair turret over the lower inner ring, under the mid collar). Its "
          "load at the working height (30 N) is a typical catalogue figure, UNVERIFIED: "
          "check the listing."))
register(Item(
    "optical_endstop", "Slotted optical endstop module (TCST2103 type, 3D-printer board)",
    "electronics",
    (_unpriced("Amazon", "https://www.amazon.com/s?k=optical+endstop+tcst2103", "optical "
               "endstop boards"),),
    dims={"board": [33.0, 10.5, 1.6], "slot_w": 3.0, "slot_h": 6.0, "body": [12.0, 6.0, 11.0],
          "hole_pitch": 19.0, "mass_g": 3.0},
    notes="The homing sensor: a flag through its slot breaks the beam. Board and body sizes "
          "are nominal (UNVERIFIED: measure the board bought)."))
register(Item(
    "suction_cup_m5_10", "Vacuum suction cup, 10 mm NBR, M5 male (Festo VAS-10-M5-NBR)",
    "pneumatic",
    (_unpriced("Festo", "https://www.festo.com/us/en/a/173440/", "VAS-10-M5-NBR"),),
    dims={"d": 10.0, "h": 11.0, "thread": "M5", "mass_g": 2.0, "material": "nylon"},
    notes="The tool: picks glass slides by vacuum. Height is nominal (UNVERIFIED)."))
register(Item(
    "vacuum_pump_12v", "12 V diaphragm vacuum pump (micro, about -60 kPa)", "pneumatic",
    (_unpriced("Amazon", "https://www.amazon.com/s?k=12v+micro+vacuum+pump", "micro "
               "vacuum pumps"),),
    dims={"mass_g": 60.0},
    notes="Sits off the arm, on the bench; feeds the suction cup through the valve."))
register(Item(
    "solenoid_valve_12v_3way", "12 V 3-way solenoid valve (vents the cup to release)",
    "pneumatic",
    (_unpriced("Amazon", "https://www.amazon.com/s?k=12v+3+way+mini+solenoid+valve",
               "mini 3-way valves"),),
    dims={"mass_g": 20.0}))
register(Item(
    "pu_tube_4mm", "PU tubing 4 mm OD x 2.5 mm ID, per metre", "pneumatic",
    (_unpriced("Amazon", "https://www.amazon.com/s?k=4mm+pu+tubing", "4 mm PU tube"),),
    dims={"od": 4.0, "id": 2.5}))
register(Item(
    "barb_m5_4mm", "Push-in fitting, M5 male to 4 mm tube", "pneumatic",
    (_unpriced("Amazon", "https://www.amazon.com/s?k=m5+push+in+fitting+4mm", "M5 to 4 mm "
               "fittings"),),
    dims={"thread": "M5", "tube": 4.0, "mass_g": 3.0}))
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
           price_usd=35.0, note="the Wide; the list price"),),
    dims={"board": [25.0, 24.0, 12.4], "pixels": [4608, 2592], "min_focus_mm": 50.0,
          "mass_g": 4.0},
    notes="The homing reference's camera (control.vision): on the tool holder about 50 mm "
          "over the reference board, its lens locked once focused; on the Klipper host's "
          "(a Raspberry Pi's) CSI port. The standard module focuses only from 10 cm. "
          "Mechanical drawing and STEP: Raspberry Pi's product information portal "
          "(pip.raspberrypi.com)."))
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
        notes="A base_pair turret's column: four of them between its plates carry the arm's "
              "tipping moment (a 7 mm hex racks about six times less than the M3 rounds)."))
register(Item(
    "angle_bracket_20", "Steel L angle bracket 20 x 20 x 16 mm, 3.4 mm holes (corner brace)",
    "bracket",
    (Offer("Amazon", "https://www.amazon.com/s?k=20mm+L+corner+brace+bracket+M3",
           pack_qty=20, price_usd=8.0, note="search; zinc-plated corner braces, an estimate"),),
    dims={"w": 16.0, "leg": 20.0, "t": 2.0, "hole": 3.4},
    notes="A frame plate's foot held to the turntable (in place of the printed brackets)."))
register(Item(
    "rubber_feet_12mm", "Adhesive rubber bumper feet 12 x 4 mm", "foot",
    (Offer("Amazon", "https://www.amazon.com/s?k=rubber+bumper+feet+12mm", pack_qty=50,
           price_usd=6.0, note="search; an estimate"),),
    dims={"d": 12.0, "h": 4.0},
    notes="Not used: round 7's review screws the base_pair turret to the bench (on "
          "rubber a light base rocks under the arm's moment)."))
