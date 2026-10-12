"""Rotary joints out of the side's plane: an arm's turntable (2026-10-12, phase 4).

A rotary joint is four swappable parts, each from its registry:

* the **support**, what holds the axis (:data:`SUPPORTS`): a pair of 608ZZ bearings on a
  fixed 8 mm shaft, spaced :data:`SPACING` apart in the turning hub, plain
  (``deep_groove_pair``: the MT4's way; their internal clearance and the shaft's fit tilt
  the axis) or preloaded by a wave washer under the top collar (``preloaded_pair``: the
  two act as an angular-contact pair back to back, no play while the washer's force
  exceeds what a tipping moment takes off a row); or the bearings in the base
  (``base_pair``, :mod:`.base_pair`): two flanged F608s bonded into the base's two plates
  about 48 mm apart, the shaft turning in them with the turntable clamped on it;
* the **transmission** (:data:`construction.transmissions.TRANSMISSIONS`, the same as the
  side's drives'): the GT2 belt (16 to 80 teeth, 5:1) or the cord capstan, here on a
  :data:`YAW_DRUM_R` drum (11.6:1), its travel bounded by the motor's shaft (the wraps'
  walk along the sleeve);
* the **actuator** (:data:`spiderpig.actuators.STEPPERS`);
* the **sensor** (:data:`SENSORS`, or ``"none"``): the homing switch, a slotted optical
  endstop on the base and a half-circle flag on the turntable (which side of home the
  turntable is on is the beam's state: homing knows which way to turn).

A **preset** (:data:`PRESETS`, the role ``turntable``) names one of each, and the joint's
range; a combination of its own is one entry more. The parts' error terms add up
(:meth:`RotaryPreset.error_terms`): the support's tilt and play, the transmission's
backlash and stiffness, the motor's step through the reduction.

The turntable itself, the base under it and the side standing on it: :mod:`.turret`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any

from spiderpig.construction.joints import ErrorTerms

SPACING = 25.0
"""The hub's two bearings' centres apart (mm): the support's tilt is its play over this."""
SHAFT_FIT = 0.009
"""The worst diametral gap between an 8 mm h6 shaft (0 / -0.009) and a 608's bore
(0 / -0.008) (mm): a plain pair's inner rings slide on it."""
YAW_DRUM_R = 40.0
"""The turntable's capstan drum radius (mm): 11.6:1 on the 6 mm sleeve. Its travel is the
wraps' walk the motor's 24 mm shaft leaves (:func:`.turret.turret_plan`): a larger drum
turns less (15.9:1 on a 55 mm drum: about 174 deg)."""
ECCENTRICITY_MM = 0.05
"""How far the yaw's pulley or drum sits off the axis once centred on the hub with a sleeve
at assembly (mm): a once-a-turn error of ``e / R`` at the joint, systematic (it calibrates
out). An estimate."""
AXIS_RUNOUT_MM = 0.05
"""A clamping collar's face runout across its diameter (mm): the fixed shaft tilts by it
over the collar, the yaw axis out of plumb, systematic. An estimate."""
SWITCH_REPEAT_MM = 0.02
"""A slotted optical switch's repeatability at its flag's edge (mm): the homed yaw differs
by it over the flag's radius each time. A typical figure, unverified."""
PRELOAD_ROW_FACTOR = 4.0
"""What tipping force (each bearing's radial reaction) a preloaded deep-groove pair takes
before a row unloads, per newton of preload: about ``1 / tan`` of its contact angle (some
14 deg under a light preload). An estimate."""


@dataclass(frozen=True)
class Support:
    """The bearing arrangement holding a rotary axis (module docstring)."""

    key: str
    label: str
    bearing: str = "bearing_608zz"
    preload: bool = False
    spring: str = "wave_washer_8mm"
    row_share: float = 1.0       # the spring's force the more lightly loaded row keeps

    @property
    def slide_note(self) -> str:
        which = "lower (floating)" if self.key == "base_pair" else "upper"
        note = (f"the {which} bearing must slide on the shaft for the washer to load it (an "
                "h6 shaft may hold it by up to 8 um of interference)")
        if self.row_share < 1.0:
            note += ("; the fixed upper row keeps the washer's force less the arm's weight "
                     f"(about {self.row_share:.0%} of it, estimated): a 60-80 N spring "
                     "doubles the moment held")
        return note

    def play(self) -> float:
        """The diametral play each bearing leaves the axis: its internal clearance and the
        shaft's fit (none once preloaded)."""
        if self.preload:
            return 0.0
        from spiderpig.construction.pivots.ball import internal_clearance

        return internal_clearance(self.bearing) + SHAFT_FIT

    def capacity_nm(self, spacing: float) -> float:
        """The tipping moment it holds without play (N·m): a preloaded pair until a row
        unloads, a plain pair none (its play moves under any moment)."""
        if not self.preload:
            return 0.0
        from spiderpig.hardware.catalog import get

        load = float(get(self.spring).dims["load_n"])
        return PRELOAD_ROW_FACTOR * load * self.row_share * spacing / 1000.0

    def error_terms(self, spacing: float) -> ErrorTerms:
        c = self.play()
        notes = (f"two {self.bearing} {spacing:g} mm apart",)
        if self.preload:
            notes += (f"preloaded by a {self.spring}: no play up to about "
                      f"{self.capacity_nm(spacing):.1f} N·m of tipping moment (estimate)",
                      self.slide_note)
        else:
            notes += (f"each bearing's clearance and the shaft's fit, {c * 1000:.0f} um "
                      "diametral: the axis tilts by it over the spacing",)
        return ErrorTerms(radial_play_mm=c, tilt_deg=math.degrees(c / spacing),
                          source=f"{self.key}: {self.label}", notes=notes)


SUPPORTS: dict[str, Support] = {s.key: s for s in (
    Support("deep_groove_pair", "a 608ZZ pair on a fixed 8 mm shaft, not preloaded"),
    Support("preloaded_pair", "a 608ZZ pair on a fixed 8 mm shaft, preloaded by a wave washer",
            preload=True),
    Support("base_pair", "two F608ZZ bonded into the base's plates about 48 mm apart, the "
            "8 mm shaft turning in them, preloaded by a wave washer (the MT4's way: "
            ":mod:`.base_pair`)", bearing="bearing_f608zz", preload=True, row_share=0.5),
)}


@dataclass(frozen=True)
class Sensor:
    """A homing sensor: a switch on the fixed side, a flag on the turning one."""

    key: str
    label: str
    bom_key: str
    repeat_mm: float = SWITCH_REPEAT_MM     # its repeatability at the flag's edge


SENSORS: dict[str, Sensor] = {s.key: s for s in (
    Sensor("optical_flag", "a slotted optical endstop on the base, a half-circle flag under "
           "the turntable", "optical_endstop"),
    Sensor("ee_sx67", "an Omron EE-SX67 slot sensor on the base, an aluminium flag under the "
           "turntable", "omron_ee_sx670", 0.005),
)}
"""The EE-SX67's repeatability: its datasheet's reference graph, 2 um over 20 tries at
25 C, 20-40 um of drift between 25 C and -25 / 55 C (about 0.7 um/K): 5 um budgeted (round
5 of the arm's precision review); the TCST2103 board's 20 um is unverified."""


@dataclass(frozen=True)
class RotaryPreset:
    """A rotary joint's four parts and its range (module docstring)."""

    key: str
    label: str
    support: str
    transmission: str
    actuator: str
    sensor: str
    range_deg: tuple[float, float]

    @property
    def parts(self) -> tuple[Support, Any, Any, Sensor | None]:
        """``(support, transmission, motor, sensor or None)``."""
        from spiderpig import actuators

        return (SUPPORTS[self.support], yaw_transmission(self.transmission),
                actuators.get(self.actuator), SENSORS.get(self.sensor))

    def error_terms(self, spacing: float, drive: ErrorTerms) -> ErrorTerms:
        """The support's terms with the transmission's (``drive``, at its geometry) and the
        motor's through the reduction."""
        support, tr, motor, _ = self.parts
        return support.error_terms(spacing) + drive + motor.error_terms(tr.ratio(None))


def yaw_transmission(key: str) -> Any:
    """The transmission ``key`` as the turntable's: the capstan on its large drum."""
    from spiderpig.construction.transmissions import TRANSMISSIONS

    t: Any = TRANSMISSIONS[key]
    return replace(t, drum_r=YAW_DRUM_R) if key == "capstan" else t


PRESETS: dict[str, RotaryPreset] = {p.key: p for p in (
    RotaryPreset("belt_preload", "GT2 belt 5:1 on a preloaded 608 pair, optical homing",
                 "preloaded_pair", "belt", "nema17_17hs4401", "optical_flag", (-150.0, 150.0)),
    RotaryPreset("mt4", "the MT4's way: GT2 belt 5:1 on a plain 608 pair, optical homing",
                 "deep_groove_pair", "belt", "nema17_17hs4401", "optical_flag",
                 (-150.0, 150.0)),
    RotaryPreset("capstan_preload", "cord capstan 11.6:1 on a preloaded 608 pair, optical "
                 "homing", "preloaded_pair", "capstan", "nema17_17hs4401", "optical_flag",
                 (-115.0, 115.0)),
    RotaryPreset("mt4_base", "the MT4's way: GT2 belt 5:1, the shaft turning in a preloaded "
                 "F608 pair in the base's plates, the driver at 1/64, an M2 screw through a "
                 "slot switch past the range's end (round 7's turret review)", "base_pair", "belt",
                 "nema17_17hs4401_x64", "optical_flag", (-120.0, 120.0)),
    RotaryPreset("capstan_precise", "cord capstan 11.6:1 on a preloaded 608 pair, an Omron "
                 "slot sensor, the driver at 1/64", "preloaded_pair", "capstan",
                 "nema17_17hs4401_x64", "ee_sx67", (-115.0, 115.0)),
)}
DEFAULT = "belt_preload"
"""The user's "start with the belt" (2026-10-11), preloaded: the MT4's tilt play gone."""
