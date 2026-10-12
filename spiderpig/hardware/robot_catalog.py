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
          "washer to preload the pair (try the bearings on it; polish, or pick another)."))
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
    notes="The preloaded pair's spring, under the top collar on the upper inner ring. Its "
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
