"""Rotary joints out of the side's plane: an arm's turntable (2026-10-12, phase 4).

A rotary joint is four parts, each from its registry:

* the **support**, what holds the axis (:data:`SUPPORTS`): ``base_pair``
  (:mod:`.base_pair`): two flanged F608s bonded into the base's two plates about 48 mm
  apart, the shaft turning in them with the turntable clamped on it, the pair preloaded by
  a wave washer;
* the **transmission** (:func:`yaw_transmission`): ``belt_60``, the side's drives' GT2
  belt (:data:`construction.transmissions.TRANSMISSIONS` ``belt``) on a 60-tooth large
  pulley (3.75:1, a stock 158 mm loop);
* the **actuator** (:data:`spiderpig.actuators.STEPPERS`);
* the **sensor** (:data:`SENSORS`): ``hard_stop``, no switch: a screw under the turntable
  stalls the driver on a post on the base past the range's end, and the camera sets the
  fine zero (:mod:`control.homing`).

A **preset** (:data:`PRESETS`, the role ``turntable``) names one of each, and the joint's
range: ``mt4_compact``, the one the user approved (2026-10-14; the others were pruned
then, ``config.REMOVED_CONSTRUCTIONS``). The parts' error terms add up
(:meth:`RotaryPreset.error_terms`): the support's tilt and play, the transmission's
backlash and stiffness, the motor's step through the reduction.

The turntable itself, the base under it and the side standing on it: :mod:`.turret`.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

from spiderpig.construction.joints import ErrorTerms

ECCENTRICITY_MM = 0.05
"""How far the yaw's pulley sits off the axis once centred on its own bore on the shaft
(mm): a once-a-turn error of ``e / R`` at the joint, systematic (it calibrates out). An
estimate."""
AXIS_RUNOUT_MM = 0.05
"""A clamping collar's face runout across its diameter (mm): the shaft tilts by it over the
collar, the yaw axis out of plumb, systematic. An estimate."""
PRELOAD_ROW_FACTOR = 4.0
"""What tipping force (each bearing's radial reaction) a preloaded deep-groove pair takes
before a row unloads, per newton of preload: about ``1 / tan`` of its contact angle (some
14 deg under a light preload). An estimate."""


@dataclass(frozen=True)
class Support:
    """The bearing arrangement holding a rotary axis (module docstring): a pair preloaded
    by ``spring``, its fixed row keeping ``row_share`` of the spring's force."""

    key: str
    label: str
    bearing: str
    spring: str = "wave_washer_8mm"
    row_share: float = 1.0       # the spring's force the more lightly loaded row keeps

    @property
    def slide_note(self) -> str:
        note = ("the lower (floating) bearing must slide on the shaft for the washer to load "
                "it (an h6 shaft may hold it by up to 8 um of interference)")
        if self.row_share < 1.0:
            note += ("; the fixed upper row keeps the washer's force less the arm's weight "
                     f"(about {self.row_share:.0%} of it, estimated): a 60-80 N spring "
                     "doubles the moment held")
        return note

    def capacity_nm(self, spacing: float) -> float:
        """The tipping moment it holds without play (N·m): until a row unloads."""
        from spiderpig.hardware.catalog import get

        load = float(get(self.spring).dims["load_n"])
        return PRELOAD_ROW_FACTOR * load * self.row_share * spacing / 1000.0

    def error_terms(self, spacing: float) -> ErrorTerms:
        """No play (preloaded) up to :meth:`capacity_nm`."""
        notes = (f"two {self.bearing} {spacing:g} mm apart",
                 f"preloaded by a {self.spring}: no play up to about "
                 f"{self.capacity_nm(spacing):.1f} N·m of tipping moment (estimate)",
                 self.slide_note)
        return ErrorTerms(radial_play_mm=0.0, tilt_deg=0.0, source=f"{self.key}: {self.label}",
                          notes=notes)


SUPPORTS: dict[str, Support] = {s.key: s for s in (
    Support("base_pair", "two F608ZZ bonded into the base's plates about 48 mm apart, the "
            "8 mm shaft turning in them, preloaded by a wave washer (the MT4's way: "
            ":mod:`.base_pair`)", bearing="bearing_f608zz", row_share=0.5),
)}


@dataclass(frozen=True)
class Sensor:
    """A homing sensor: what the fixed side and the turning one do."""

    key: str
    label: str
    bom_key: str
    repeat_mm: float             # its repeatability at its radius (mm)


SENSORS: dict[str, Sensor] = {s.key: s for s in (
    Sensor("hard_stop", "no sensor: an M3 screw under the turntable stalls the driver "
           "(TMC2209 StallGuard) on a post on the base past the range's end", "", 0.75),
)}


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
    def parts(self) -> tuple[Support, Any, Any, Sensor]:
        """``(support, transmission, motor, sensor)``."""
        from spiderpig import actuators

        return (SUPPORTS[self.support], yaw_transmission(self.transmission),
                actuators.get(self.actuator), SENSORS[self.sensor])

    def error_terms(self, spacing: float, drive: ErrorTerms) -> ErrorTerms:
        """The support's terms with the transmission's (``drive``, at its geometry) and the
        motor's through the reduction."""
        support, tr, motor, _ = self.parts
        return support.error_terms(spacing) + drive + motor.error_terms(tr.ratio(None))


def yaw_transmission(key: str) -> Any:
    """The turntable's transmission ``key``: ``belt_60``, the GT2 belt on a 60-tooth large
    pulley (16 to 60 teeth, 3.75:1, a 158 mm loop)."""
    from spiderpig.construction.transmissions import TRANSMISSIONS

    if key != "belt_60":
        raise KeyError(f"unknown turntable transmission {key!r}; have ['belt_60']")
    belt: Any = TRANSMISSIONS["belt"]
    return replace(belt, key=key, large_key="pulley_gt2_60t_8mm")


PRESETS: dict[str, RotaryPreset] = {p.key: p for p in (
    RotaryPreset("mt4_compact", "the MT4's way made small: GT2 16 to 60 teeth (3.75:1) on "
                 "the 158 mm loop, the shaft turning in a preloaded F608 pair in the base's "
                 "plates, the driver at 1/64 homed by its stall on a hard stop past the "
                 "range's end (the camera then sets the zero), the motor 38.5 mm from the "
                 "axis, the base a tight round core under a turntable as wide, a lobe out "
                 "the back round the motor (the user's 'an elegant machine', 2026-10-14)",
                 "base_pair", "belt_60", "nema17_17hs4401_x64", "hard_stop", (-120.0, 120.0)),
)}
DEFAULT = "mt4_compact"
"""The user's approved turntable (2026-10-14): the only one kept."""
