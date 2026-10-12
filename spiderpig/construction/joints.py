"""What each joint part lets its joint do wrong: the error terms every construction reports.

A revolute joint of a robot is four choices, each a construction from a registry: the
**support** that carries it (a plain pin in laser-cut holes, a bearing pair, a crossed
roller), the **transmission** that drives it (a servo horn on the crank, a belt, a capstan),
the **actuator** (a servo, a stepper) and the **sensor** (a homing flag, an encoder). Each
says how far it lets its joint wander through :class:`ErrorTerms`; the precision analysis
(``spiderpig.precision``) adds them up along the kinematic chain, so a part added later is
analysed with no change there.

Two kinds of error are kept apart, because only one of them shows when a move is repeated:

* **lost motion** (play, backlash, deadband): the joint can sit anywhere inside it, so it
  hurts *repeatability*;
* **compliance and systematic error** (stiffness under a load, a resolution step, a
  tolerance): the same at the same pose and load, so it hurts *accuracy* and is
  calibrated out.

Every number is at the joint (an angle about its axis, a play across it), whatever the part
does inside; ``0`` means "none" and ``math.inf`` stiffness "rigid". :func:`plain_hole`
gives a pin in a laser-cut hole its terms from the fit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace


@dataclass(frozen=True)
class ErrorTerms:
    """How a joint part lets its joint wander.

    ``radial_play_mm``: diametral clearance a load reversal moves the joint across, in the
    plane. ``axial_play_mm``: end float along the axis. ``tilt_deg``: how far the joint's
    axis can tilt out of square (a link wobbling on its pin). ``backlash_deg``: lost motion
    about the axis (a gear train's backlash, a servo's deadband). ``resolution_deg``: the
    smallest step the joint can be commanded by. ``accuracy_deg``: a systematic error about
    the axis, the same every time (a stepper's microstep off its place): it calibrates out.
    ``stiffness_nm_per_rad``: torsional
    stiffness about the axis (a belt, a cable, a stepper's holding stiffness, all seen at
    the joint). ``radial_stiffness_n_per_mm``: stiffness across it. ``source``: the
    construction and the basis of the numbers.
    """

    radial_play_mm: float = 0.0
    axial_play_mm: float = 0.0
    tilt_deg: float = 0.0
    backlash_deg: float = 0.0
    resolution_deg: float = 0.0
    accuracy_deg: float = 0.0
    stiffness_nm_per_rad: float = math.inf
    radial_stiffness_n_per_mm: float = math.inf
    source: str = ""
    notes: tuple[str, ...] = field(default=())

    def __add__(self, other: ErrorTerms) -> ErrorTerms:
        """Two parts of one joint in series: plays and backlash add, the coarser resolution
        wins, stiffnesses combine as springs in series."""
        def series(a: float, b: float) -> float:
            if math.isinf(a):
                return b
            if math.isinf(b):
                return a
            return 1.0 / (1.0 / a + 1.0 / b) if a > 0 and b > 0 else 0.0

        return ErrorTerms(
            radial_play_mm=self.radial_play_mm + other.radial_play_mm,
            axial_play_mm=self.axial_play_mm + other.axial_play_mm,
            tilt_deg=self.tilt_deg + other.tilt_deg,
            backlash_deg=self.backlash_deg + other.backlash_deg,
            resolution_deg=max(self.resolution_deg, other.resolution_deg),
            accuracy_deg=self.accuracy_deg + other.accuracy_deg,
            stiffness_nm_per_rad=series(self.stiffness_nm_per_rad, other.stiffness_nm_per_rad),
            radial_stiffness_n_per_mm=series(self.radial_stiffness_n_per_mm,
                                             other.radial_stiffness_n_per_mm),
            source=" + ".join(s for s in (self.source, other.source) if s),
            notes=self.notes + other.notes,
        )

    def lost_motion_deg(self, lever_mm: float) -> float:
        """The joint's lost motion about its axis, with its radial play seen at ``lever_mm``
        (a link that long rocks across the clearance at its far end)."""
        rock = math.degrees(self.radial_play_mm / lever_mm) if lever_mm > 0 else 0.0
        return self.backlash_deg + rock

    def as_dict(self) -> dict:
        d = {k: getattr(self, k) for k in ("radial_play_mm", "axial_play_mm", "tilt_deg",
                                           "backlash_deg", "resolution_deg",
                                           "accuracy_deg")}
        for k in ("stiffness_nm_per_rad", "radial_stiffness_n_per_mm"):
            v = getattr(self, k)
            d[k] = None if math.isinf(v) else v
        return {**d, "source": self.source, "notes": list(self.notes)}


NONE = ErrorTerms(source="rigid")


def plain_hole(clearance_mm: float, axial_play_mm: float, tilt_deg: float = 0.0,
               source: str = "") -> ErrorTerms:
    """A pin turning in a plain hole: the hole's diametral clearance over the pin is the
    radial play, whatever sets the end float is the axial play."""
    return ErrorTerms(radial_play_mm=float(clearance_mm), axial_play_mm=float(axial_play_mm),
                      tilt_deg=float(tilt_deg), source=source,
                      notes=(f"{clearance_mm:g} mm diametral clearance in a plain hole",))


def from_wobble_note(note: dict, base: ErrorTerms) -> ErrorTerms:
    """A realized joint's terms: ``base`` (its construction's: the radial play) with what
    the build measured in its wobble note (:func:`construction.wobble.column_wobble`): the
    column's end float and the worst link's tilt. (The note's per-link clearance is what
    tilts the link, which for a bearing is not its radial play.)"""
    basis = str(note.get("play_basis") or "")
    return replace(base, axial_play_mm=float(note.get("play_mm", base.axial_play_mm)),
                   tilt_deg=float(note.get("worst_deg", base.tilt_deg)),
                   notes=base.notes + ((f"end float: {basis}",) if basis else ()))


def with_source(terms: ErrorTerms, source: str) -> ErrorTerms:
    return replace(terms, source=source)
