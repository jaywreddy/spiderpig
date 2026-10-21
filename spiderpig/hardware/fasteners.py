"""Screws: the families the constructions model, their catalog keys and their solids.

A :class:`Screw` is one family (a head style at one thread size): its head,
its stock lengths and the catalog key of each length (``m3_shcs_12``,
``m2_self_tap_6``; :mod:`hardware.parts` registers every one). A construction
picks a length from ``lengths`` and draws it with :func:`screw_solid`;
:func:`parse` recovers the family from a key a servo spec names.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

from spiderpig.shapes import disc, union

SIZES = {"M2": "2", "M2.5": "2p5", "M3": "3"}       # thread label -> size in a catalog key
CLEARANCE = {"2": 2.4, "2p5": 2.9, "3": 3.4,        # ISO 273 medium clearance holes (mm)
             "5": 5.5, "6": 6.6, "8": 9.0, "10": 11.0}
SHCS_LENGTHS: dict[str, tuple[float, ...]] = {
    "2": (4, 5, 6, 8, 10, 12, 16, 20),
    "2p5": (4, 5, 6, 8, 10, 12, 16, 20),
    "3": (6, 8, 10, 12, 14, 16, 18, 20, 25, 30, 35, 40),
}
BHCS_LENGTHS: dict[str, tuple[float, ...]] = {
    "3": (6, 8, 10, 12, 16, 20, 25, 30),
    # a decanting cradle's (hardware.cradle_catalog): its standoffs' M5 ends, its gearbox's
    # M6 face, the crank arm's end screw and its 12 mm standoffs' M6 ends
    "5": (8, 10, 12, 16),
    "6": (8, 10, 12, 16, 20, 25, 30, 35, 40, 45),
    "8": (12,),         # (the rear saddle's 15 mm post's foot)
    # a decanting cradle's NRV050 output shaft's M10 end tap (round 2's drive)
    "10": (16,),
}
SELF_TAP_LENGTHS: dict[str, tuple[float, ...]] = {"2": (4, 5, 6, 8, 10, 12)}


@dataclass(frozen=True)
class Screw:
    """A screw family: head size (mm), stock lengths, and its catalog key per length."""

    kind: str                   # "shcs" | "bhcs" | "self_tap"
    size: str                   # "2", "2p5", "3"
    head_d: float
    head_h: float
    lengths: tuple[float, ...]

    @property
    def d(self) -> float:
        return float(self.size.replace("p", "."))

    @property
    def shank_d(self) -> float:
        """Modelled shank: about the thread's major diameter (ISO 965 6g: M3 2.874-2.98)."""
        return 0.97 * self.d

    def key(self, length: float) -> str:
        return f"m{self.size}_{self.kind}_{length:g}"


SCREWS: dict[tuple[str, str], Screw] = {(s.kind, s.size): s for s in (
    # ISO 4762 socket heads: (head diameter dk, head height k)
    *(Screw("shcs", size, dk, k, SHCS_LENGTHS[size])
      for size, dk, k in (("2", 3.8, 2.0), ("2p5", 4.5, 2.5), ("3", 5.5, 3.0))),
    # ISO 7380-1 button head: fits a 3 mm layer where a socket head doesn't
    Screw("bhcs", "3", 5.7, 1.65, BHCS_LENGTHS["3"]),
    Screw("bhcs", "5", 9.5, 2.75, BHCS_LENGTHS["5"]),
    Screw("bhcs", "6", 10.5, 3.3, BHCS_LENGTHS["6"]),
    Screw("bhcs", "8", 14.0, 4.4, BHCS_LENGTHS["8"]),
    Screw("bhcs", "10", 17.5, 5.5, BHCS_LENGTHS["10"]),
    # PA2.0 pan-head self-tapping screws for plastic; head within ISO 7049 ST2.2's maximum
    Screw("self_tap", "2", 4.0, 1.6, SELF_TAP_LENGTHS["2"]),
)}


TR_TORQUES = ("https://www.trfastenings.com/knowledge-base/metric-screws/"
              "stainless-steel-pre-load-and-tightening-torques")
BUTTON_A2_70_NM: dict[str, float] = {"6": 4.0, "10": 15.0}
"""An ISO 7380 button head's most tightening torque in A2-70 (N·m): its low head carries less
than the thread (ISO 7380-1:2022: about 80 %), so under A2-70's own M6 5.9 N·m (friction
0.1). TR Fastenings' socket screws page (the drive review's helper, 2026-10-20: A2-70 / A4-70
M6 4 N·m, M10 15; 10.9 M6 8, M10 30). A decanting cradle's M6 button heads (its pillars',
stand columns' and tie rods') are tightened to this (design review round 3's M5)."""
A2_70_PRELOAD: dict[str, tuple[tuple[float, float, float], ...]] = {
    "6": ((0.1, 6850.0, 5.9), (0.2, 5540.0, 8.8), (0.3, 4490.0, 10.4)),
    "10": ((0.1, 20000.0, 30.0), (0.2, 16200.0, 44.0), (0.3, 13100.0, 51.0)),
}
"""A2-70 coarse threads' preload (N) at the tightening torque (N·m) TR Fastenings tabulates
for each friction (:data:`TR_TORQUES`, the drive review's helper's fetch of 2026-10-20):
``(friction, preload, torque)``. :func:`preload_band` scales a row to another torque."""


def preload_band(size: str, nm: float, frictions: tuple[float, float] = (0.1, 0.2)
                 ) -> tuple[float, float]:
    """The least and the most preload (N) an A2-70 screw of ``size`` takes at ``nm`` N·m over
    ``frictions`` (:data:`A2_70_PRELOAD`'s rows, each its preload per N·m times ``nm``: the
    torque's share that stretches the screw stays the same at one friction)."""
    rows = {mu: f / t for mu, f, t in A2_70_PRELOAD[size]}
    got = [rows[mu] * nm for mu in frictions]
    return min(got), max(got)


def screw(kind: str, size: str) -> Screw:
    return SCREWS[kind, size]


def shcs(size: str, length: float) -> str:
    return SCREWS["shcs", size].key(length)


def bhcs(size: str, length: float) -> str:
    return SCREWS["bhcs", size].key(length)


_KEY = re.compile(r"^m(\d+(?:p\d+)?)_(shcs|bhcs|self_tap)_(\d+(?:\.\d+)?)$")


def parse(key: str | None) -> tuple[Screw, float] | None:
    """``"m2_self_tap_6"`` -> (the M2 tapping screw family, 6.0); ``None`` if not modelled."""
    m = _KEY.match(key or "")
    if m is None:
        return None
    size, kind, length = m.groups()
    sk = SCREWS.get((kind, size))
    return None if sk is None else (sk, float(length))


def screw_solid(xy, sk: Screw, bearing_z: float, length: float, up: bool = True):
    """A screw whose head bears at ``bearing_z``, shank pointing up (or down)."""
    s = 1.0 if up else -1.0
    head = disc(xy, sk.head_d / 2, *sorted((bearing_z - s * sk.head_h, bearing_z)))
    shank = disc(xy, sk.shank_d / 2, *sorted((bearing_z, bearing_z + s * length)))
    return union([head, shank])
