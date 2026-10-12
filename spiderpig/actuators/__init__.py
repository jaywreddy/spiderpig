"""Actuators other than the walkers' continuous servos: stepper motors, for an arm's inputs.

An arm's inputs (:attr:`linkage.Linkage.drives`) are each turned by a motor through a
transmission (:mod:`construction.transmissions`); the role ``actuator``
(:mod:`spiderpig.roles`, one per drive) picks the motor from :data:`STEPPERS`.

A stepper is held where it is told by its rotor's teeth: its holding stiffness near a step
is about the holding torque times the rotor's tooth count (``K = T_hold N_r``, N·m/rad: a
hybrid stepper's torque near equilibrium is ``T_hold sin(N_r theta)``), and it moves in
full steps of 360 / (4 N_r) degrees, finer in microsteps (``microsteps``: the driver's
setting; a microstep's place is less exact than a full step's, ±5 % of a full step on a
typical hybrid motor: a systematic error, :meth:`StepperSpec.error_terms`' accuracy). Through
a reduction ``n`` both improve at the joint: the step by ``n``, the stiffness by ``n²``.

Each motor's numbers are its datasheet's (``source``); its body is drawn from them (the
NEMA frame, the pilot boss, the shaft with its flat), its vendor model through
:mod:`hardware.cad` when one is registered.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class StepperSpec:
    """A hybrid stepper motor (dimensions mm, torque N·m)."""

    key: str
    name: str
    frame: float                 # the square body's side (NEMA 17: 42.3)
    length: float                # the body's length, without the boss and shaft
    hole_square: float           # the mounting holes' square (NEMA 17: 31.0)
    hole_thread: str             # "M3"
    hole_depth: float            # the tapped holes' depth
    boss_d: float                # the pilot boss (NEMA 17: 22.0)
    boss_h: float
    shaft_d: float
    shaft_len: float             # from the mounting face
    flat_depth: float            # the D-flat's depth (0: round)
    holding_nm: float
    step_deg: float              # full step
    rotor_teeth: int
    rated_a: float
    mass_g: float
    bom_key: str
    source: str
    microsteps: int = 16         # the driver's setting (TMC2209 and kin: 16 by default)
    rotor_g_cm2: float = 0.0     # the rotor's inertia (g·cm², the datasheet's; 0: unknown)
    microstep_accuracy: float = 0.05    # a microstep's place, as a fraction of a full step

    @property
    def holding_stiffness_nm_per_rad(self) -> float:
        """Near a step: ``T_hold N_r`` (N·m/rad)."""
        return self.holding_nm * self.rotor_teeth

    @property
    def microstep_deg(self) -> float:
        return self.step_deg / self.microsteps

    def error_terms(self, ratio: float):
        """The motor seen at the joint through a reduction ``ratio``: its microstep (the
        joint's resolution), its microstep's inexactness (a systematic error: accuracy, the
        same at the same step), its holding stiffness times ``ratio²``."""
        from spiderpig.construction.joints import ErrorTerms

        return ErrorTerms(resolution_deg=self.microstep_deg / ratio,
                          accuracy_deg=self.microstep_accuracy * self.step_deg / ratio,
                          stiffness_nm_per_rad=self.holding_stiffness_nm_per_rad * ratio ** 2,
                          source=f"{self.key} through {ratio:.3g}:1",
                          notes=(f"{self.microsteps} microsteps of a {self.step_deg:g} deg step "
                                 f"(a microstep's place +-{self.microstep_accuracy:.0%} of a "
                                 "step)", f"holding stiffness {self.holding_nm:g} N·m x "
                                 f"{self.rotor_teeth} teeth"))


STEPPERS: dict[str, StepperSpec] = {}


def register(spec: StepperSpec) -> StepperSpec:
    if spec.key in STEPPERS and STEPPERS[spec.key] != spec:
        raise ValueError(f"stepper {spec.key!r} registered twice")
    STEPPERS[spec.key] = spec
    return spec


def get(key: str) -> StepperSpec:
    try:
        return STEPPERS[key]
    except KeyError:
        raise KeyError(f"unknown stepper {key!r}; have {sorted(STEPPERS)}") from None


def available() -> list[str]:
    return list(STEPPERS)


_SO = "https://www.omc-stepperonline.com"
NEMA17 = {"frame": 42.3, "hole_square": 31.0, "hole_thread": "M3", "hole_depth": 4.5,
          "boss_d": 22.0, "boss_h": 2.0, "shaft_d": 5.0, "shaft_len": 24.0,
          "flat_depth": 0.5, "step_deg": 1.8, "rotor_teeth": 50}
"""The NEMA 17 frame (NEMA ICS 16; the StepperOnline drawings): 42.3 mm square, M3 holes on
a 31 mm square 4.5 mm deep, a 22 mm boss 2 mm proud, a 5 mm shaft 24 mm long with a 0.5 mm
flat; 1.8 deg hybrids (50 rotor teeth)."""

register(StepperSpec(
    key="nema17_17hs4401", name="NEMA 17 stepper, 40 mm, 0.40 N·m (StepperOnline 17HS4401)",
    length=40.0, holding_nm=0.40, rated_a=1.7, mass_g=280.0, bom_key="stepper_17hs4401",
    rotor_g_cm2=54.0,
    source=f"{_SO} 17HS4401 datasheet: 40 N·cm holding, 1.7 A, 40 mm, 280 g; rotor inertia "
           "54 g·cm² (typical, UNVERIFIED)", **NEMA17))
register(replace(STEPPERS["nema17_17hs4401"], key="nema17_17hs4401_x64",
                 name="NEMA 17 stepper 17HS4401, its driver at 1/64 microsteps",
                 microsteps=64))
"""The same motor, its driver (a TMC2209) set to 1/64: a homing lands on a whole microstep,
so a quarter of the 1/16's quantisation (the review's floor: +-24 / 15 / 17 um at the tool
for q1 / q2 / q3 at 1/16); the step rate four times (the turntable about 37 k steps/s at
90 deg/s, within a TMC2209's)."""
register(StepperSpec(
    key="nema17_17hs19", name="NEMA 17 stepper, 48 mm, 0.59 N·m (StepperOnline 17HS19-2004S1)",
    length=48.0, holding_nm=0.59, rated_a=2.0, mass_g=390.0, bom_key="stepper_17hs19",
    rotor_g_cm2=82.0,
    source=f"{_SO} 17HS19-2004S1 datasheet: 59 N·cm holding, 2.0 A, 48 mm, 390 g; rotor "
           "inertia 82 g·cm² (typical, UNVERIFIED)", **NEMA17))

NEMA11 = {"frame": 28.2, "hole_square": 23.0, "hole_thread": "M2.5", "hole_depth": 2.5,
          "boss_d": 22.0, "boss_h": 1.5, "shaft_d": 5.0, "shaft_len": 20.0,
          "flat_depth": 0.5, "step_deg": 1.8, "rotor_teeth": 50}
"""The NEMA 11 frame: 28.2 mm square, M2.5 holes on a 23 mm square, a 22 mm boss, a 5 mm
shaft (the common StepperOnline drawing; the shaft's length varies by model: measure)."""

register(StepperSpec(
    key="nema11_11hs20", name="NEMA 11 stepper, 0.12 N·m (StepperOnline 11HS20-0674)",
    length=51.0, holding_nm=0.12, rated_a=0.67, mass_g=200.0, bom_key="stepper_11hs20",
    rotor_g_cm2=18.0,
    source=f"{_SO} 11HS20-0674 listing: 12 N·cm holding, 0.67 A; 51 mm and 200 g "
           "UNVERIFIED", **NEMA11))

DEFAULT = "nema17_17hs4401"


def step_angle_rad(spec: StepperSpec) -> float:
    return math.radians(spec.step_deg)
