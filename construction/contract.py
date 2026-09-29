"""Check the construction contract: every part lies inside the space its group claimed.

The planner guarantees that claims of different groups never meet; this
module checks the other half: that what a construction *builds* stays inside
what it *claimed*. Together they mean the parts can't collide at any crank
angle. Run it for every construction (tests do) and whenever one changes.

Exceptions, by design:

* the frame plates own their layers (0 and ``top``) outright;
* the servo stands outside the stack, above the inner frame plate; the part
  of the drive group below that face (the horn) must lie in the crank's
  "servo horn" claim.
"""

from __future__ import annotations

from build123d import Box, Location

from construction.base import Build, Realized
from construction.envelope import claimed_solid
from construction.plates import FramePlates, LinkPlates

TOL = 0.02        # mm the envelope is grown by (float noise)
MAX_OUTSIDE = 1e-3  # mm^3 a part may poke outside its envelope


def _outside(part, envelope) -> float:
    """Volume of ``part`` outside ``envelope`` (either may be empty)."""
    if part is None or not part.solids():
        return 0.0
    if envelope is None:
        return part.volume
    rest = part - envelope
    return 0.0 if rest is None else sum(s.volume for s in rest.solids())


def _slab(z0: float, z1: float, size: float = 1e4):
    return Box(size, size, z1 - z0).moved(Location((0.0, 0.0, (z0 + z1) / 2)))


def check_side(design, mech) -> list[str]:
    """Violations of the contract for one side at ``mech``'s crank angle (empty if none)."""
    build = Build(design.ctx, design.plan, mech)
    problems: list[str] = []
    done = Realized()
    plate_top = build.z(build.top)[1]
    for g in design.groups:
        plates = isinstance(g, (LinkPlates, FramePlates))
        got = g.realize(build, done) if plates else g.realize(build)
        done.merge(got)
        if isinstance(g, FramePlates):
            for b in got.bodies:
                bb = b.part.bounding_box()
                zs = [build.z(k) for k in (0, build.top)]
                if not any(abs(bb.min.Z - z0) < TOL and abs(bb.max.Z - z1) < TOL for z0, z1 in zs):
                    problems.append(f"{b.name}: frame plate outside its layer")
            continue
        if g.name == "drive":
            envelope = claimed_solid(build, [p for p in build.shapes("crank")
                                             if p.label == "servo horn"], TOL)
            below = _slab(-1e3, plate_top)
            for b in got.bodies:
                inside = b.part & below if b.part is not None else None
                vol = _outside(inside, envelope)
                if vol > MAX_OUTSIDE:
                    problems.append(f"{b.name}: {vol:.3f} mm^3 below the servo face, "
                                    "outside the horn claim")
            continue
        if isinstance(g, LinkPlates):
            for b in got.bodies:
                env = claimed_solid(build, build.shapes(b.name), TOL)
                vol = _outside(b.part, env)
                if vol > MAX_OUTSIDE:
                    problems.append(f"{b.name}: {vol:.3f} mm^3 outside its claim")
            continue
        envelope = claimed_solid(build, build.shapes(g.name), TOL)
        for b in got.bodies:
            vol = _outside(b.part, envelope)
            if vol > MAX_OUTSIDE:
                problems.append(f"{g.name}/{b.name}: {vol:.3f} mm^3 outside its claims")
    return problems


__all__ = ["check_side"]
