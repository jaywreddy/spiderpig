"""What the bolt crank's joints carry (:class:`BoltCrank` ``capacity``) and the one
hex-in-socket bearing model (:func:`hex_bearing_nm`)."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from spiderpig.construction.crank.hex import HexJoint

if TYPE_CHECKING:
    from spiderpig.config import BuildConfig
    from spiderpig.construction.crank.web import WebJoint


def hex_bearing_nm(af: float, engaged: float, p: float = 50.0, relief: float = 0.0) -> float:
    """The twist (N·m) a hex ``af`` across flats carries in a socket of a plastic (printed PLA,
    acrylic) over ``engaged`` mm when the pressure on each flat's loaded half reaches ``p``
    everywhere (fully plastic): per flat ``p L (a/2)^2 / 2`` about the flat's midpoint, ``a =
    af / sqrt 3`` its length less ``relief`` (the corner reliefs take that off a flat), six
    flats: ``0.75 p a^2 L``. An M3 nut (5.5 AF) in a 2.4 mm printed trap: 0.9 N·m, about
    where printed nut traps are reported to spin out. The one bearing model for every
    hex-in-socket joint (:mod:`spiderpig.strength`)."""
    a = max(af / math.sqrt(3) - relief, 0.0)
    return 0.75 * p * a * a * max(engaged, 0.0) / 1e3




class CapacityMixin:
    """The joints' ratings of :class:`BoltCrank`."""

    if TYPE_CHECKING:       # what it reads of :class:`BoltCrank` (its fields and methods)
        hex: bool
        hex_af: float
        hex_corner_loss: float
        hex_yield: float
        head_mu: float
        lock_key: str | None
        pin_hole: float
        pin_min_engage: float
        pin_mu: float
        pin_od: float
        pin_preload_n: float
        web_t: float
        web_yield: float

        def pin_screw(self) -> tuple[float, float]: ...

    def capacity(self, joint: WebJoint | HexJoint | None = None) -> dict[str, float]:
        """What one crankpin of single aluminium webs holds (N·m), per web (the weaker
        counts; both are alike). The hex pin: :meth:`hex_capacity`. The round one: the
        standoff's end face clamped on the web by the M4
        screw's preload, and beside it the screw's head on the web's other face, which
        reaches the standoff only through the screw's thread (its friction under that
        preload plus the threadlocker's breakaway, half for plated steel). Friction joints:
        the preload and both coefficients are estimates, UNVERIFIED until the test build."""
        from spiderpig.hardware.catalog import get

        if self.hex:
            return self.hex_capacity(joint if isinstance(joint, HexJoint) else None)

        F = self.pin_preload_n
        hd, _ = self.pin_screw()

        def r_eff(ro: float, ri: float) -> float:      # an annulus's friction radius (mm)
            return 2 / 3 * (ro ** 3 - ri ** 3) / (ro ** 2 - ri ** 2)

        face = self.pin_mu * F * r_eff(self.pin_od / 2, 2.0) / 1e3
        head = self.head_mu * F * r_eff(hd / 2, self.pin_hole / 2) / 1e3
        thread = 0.15 * F * (3.545 / 2) / math.cos(math.radians(30)) / 1e3
        e = min(joint.engage_lo, joint.engage_hi) if joint else self.pin_min_engage
        lock = 0.0
        if self.lock_key is not None:
            m10 = float(get(self.lock_key).dims.get("breakaway_m10_nm", 0.0))
            lock = 0.5 * m10 * (4.0 / 10.0) ** 2 * (e / 8.4)
        screw = min(head, thread + lock)
        return {
            f"web clamped on the standoff's end ({F:g} N, mu {self.pin_mu:g}) + the screw "
            f"head (mu {self.head_mu:g}, through the M4 thread)": round(face + screw, 3),
        }

    def hex_capacity(self, joint: HexJoint | None = None) -> dict[str, float]:
        """What one hex crankpin holds (N·m): the hex in each web's pocket (the shallower
        counts), one bearing model (:func:`hex_bearing_nm`) at the weaker of the plate's
        and the standoff's yield (``web_yield`` 193 MPa on 5052-H32, 276 on 6061-T6;
        ``hex_yield`` 300 MPa steel), each flat short by ``hex_corner_loss`` for the
        standoff's rounded corners (the dog-bone reliefs leave the pocket's flats whole);
        and the standoff's own torsion (the tube in its flats round the M3 thread)."""
        eng = min(joint.engaged_lo, joint.engaged_hi) if joint else self.web_t
        p = min(self.web_yield, self.hex_yield)
        bearing = hex_bearing_nm(self.hex_af, eng, p, self.hex_corner_loss)
        zp = math.pi * (self.hex_af ** 4 - 3.0 ** 4) / (16 * self.hex_af)
        torsion = self.hex_yield / math.sqrt(3) * zp / 1e3
        return {
            f"hex {self.hex_af:g} AF in its plate's pocket, {eng:g} mm at {p:g} MPa": round(
                bearing, 3),
            f"hex standoff torsion ({self.hex_yield:g} MPa)": round(torsion, 3),
        }


def crank_capacity(meta: dict, config: BuildConfig) -> dict[str, float] | None:
    """What one crankpin joint of ``config``'s crank holds, per element (N·m), from the
    built crank's notes (``meta``: ``crank_bolt``), else the construction's nominal (no
    build needed: the sim's metrics); ``None`` for a crank this doesn't model."""
    from spiderpig.construction import CRANKS

    construction = CRANKS.get(config.crank)
    if construction is None:
        return None
    bolt = meta.get("crank_bolt")
    if bolt and bolt.get("chains"):
        caps: dict[str, float] = {}
        for ch in bolt["chains"] + bolt.get("journals", []):
            for k, v in ch["capacity_nm"].items():
                caps[k] = min(caps.get(k, math.inf), v)
        return caps
    return construction.for_sheet(config.crank_sheet).capacity()
