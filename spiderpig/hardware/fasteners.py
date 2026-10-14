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
CLEARANCE = {"2": 2.4, "2p5": 2.9, "3": 3.4}         # ISO 273 medium clearance holes (mm)
SHCS_LENGTHS: dict[str, tuple[float, ...]] = {
    "2": (4, 5, 6, 8, 10, 12, 16, 20),
    "2p5": (4, 5, 6, 8, 10, 12, 16, 20),
    "3": (6, 8, 10, 12, 14, 16, 18, 20, 25, 30, 35, 40, 45, 50, 60, 70),
}
BHCS_LENGTHS: dict[str, tuple[float, ...]] = {"3": (6, 8, 10, 12, 16, 20, 25, 30)}
SELF_TAP_LENGTHS: dict[str, tuple[float, ...]] = {"2": (4, 5, 6, 8, 10, 12)}
BUTTON_LENGTHS: dict[str, tuple[float, ...]] = {"2": (3, 4, 5, 6)}


@dataclass(frozen=True)
class Screw:
    """A screw family: head size (mm), stock lengths, and its catalog key per length."""

    kind: str                   # "shcs" | "bhcs" | "button" | "self_tap"
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

    def priced(self, length: float) -> bool:
        """Whether the BOM's first offer for this length shows a price (Bolt Depot has no
        M3 x 14: its first offer is McMaster's, unpriced)."""
        from spiderpig.hardware.catalog import get

        try:
            offer = get(self.key(length)).offer
        except KeyError:
            return False
        return offer is not None and offer.price_usd is not None

    def length_for(self, need: float, *, longest: bool = False) -> float:
        """The shortest length at least ``need`` mm, a priced one first (:meth:`priced`),
        else any listed: the one way a part picks a screw's length (2026-10-14: twenty
        inline picks before). ``longest``: none reaches it, so take the longest instead of
        raising."""
        listed = sorted(float(L) for L in self.lengths)
        for pool in ([L for L in listed if self.priced(L)], listed):
            for L in pool:
                if need - 1e-6 <= L:
                    return L
        if longest and listed:
            return listed[-1]
        raise LookupError(f"no {self.kind} M{self.d:g} reaches {need:.2f} mm "
                          f"(lengths: {listed})")

SCREWS: dict[tuple[str, str], Screw] = {(s.kind, s.size): s for s in (
    # ISO 4762 socket heads: (head diameter dk, head height k)
    *(Screw("shcs", size, dk, k, SHCS_LENGTHS[size])
      for size, dk, k in (("2", 3.8, 2.0), ("2p5", 4.5, 2.5), ("3", 5.5, 3.0))),
    # ISO 7380-1 button head: fits a 3 mm layer where a socket head doesn't
    Screw("bhcs", "3", 5.7, 1.65, BHCS_LENGTHS["3"]),
    # ISO 7380-1 M2 button head (Aspen's drawing: dk 3.5-3.8, k 1.16-1.3; the maxima): the
    # ball pair's ground pin ends only (construction.pivots.pair.GroundShaft), its own kind
    # so the horn screws' choice ("bhcs" before "shcs", construction.crank.web) is unchanged
    Screw("button", "2", 3.8, 1.3, BUTTON_LENGTHS["2"]),
    # PA2.0 pan-head self-tapping screws for plastic; head within ISO 7049 ST2.2's maximum
    Screw("self_tap", "2", 4.0, 1.6, SELF_TAP_LENGTHS["2"]),
)}


def screw(kind: str, size: str) -> Screw:
    return SCREWS[kind, size]


def shcs(size: str, length: float) -> str:
    return SCREWS["shcs", size].key(length)


def bhcs(size: str, length: float) -> str:
    return SCREWS["bhcs", size].key(length)


_KEY = re.compile(r"^m(\d+(?:p\d+)?)_(shcs|bhcs|button|self_tap)_(\d+(?:\.\d+)?)$")


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
