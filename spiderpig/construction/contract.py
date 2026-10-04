"""Check the construction contract: every part lies inside the space its group claimed.

The planner guarantees that claims of different groups never meet; this
module checks the other half: that what a construction *builds* stays inside
what it *claimed*. Together they mean the parts can't collide at any crank
angle. Run it for every construction (tests do) and whenever one changes.
:func:`clashes` and :func:`bad_solids` check the fabricated solids themselves
(the audit and the tests).

Exceptions, by design:

* the frame plates own their layers (0 and ``top``) outright;
* the servo stands outside the stack, above the inner frame plate; the part
  of the drive group below that face (the horn, the mounting screws) must
  lie in the crank's "servo horn" claim or the drive's own claims.
"""

from __future__ import annotations

import itertools

from build123d import Box, Location

from spiderpig.construction.base import Build, Realized
from spiderpig.construction.envelope import claimed_solid

TOL = 0.02        # mm the envelope is grown by (float noise)
MAX_OUTSIDE = 1e-3  # mm^3 a part may poke outside its envelope
CLASH_MM3 = 1e-3  # mm^3 two parts may share (float noise)


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
        got = g.realize(build, done)
        done.merge(got)
        if g.name == "frame":
            for b in got.bodies:
                bb = b.part.bounding_box()
                zs = [build.z(k) for k in (0, build.top)]
                if not any(abs(bb.min.Z - z0) < TOL and abs(bb.max.Z - z1) < TOL for z0, z1 in zs):
                    problems.append(f"{b.name}: frame plate outside its layer")
            continue
        if g.name == "drive":
            envelope = claimed_solid(build, [p for p in build.shapes("crank")
                                             if p.label == "servo horn"] + build.shapes(g.name),
                                     TOL)
            below = _slab(-1e3, plate_top)
            for b in got.bodies:
                inside = b.part & below if b.part is not None else None
                vol = _outside(inside, envelope)
                if vol > MAX_OUTSIDE:
                    problems.append(f"{b.name}: {vol:.3f} mm^3 below the servo face, "
                                    "outside the horn claim")
            continue
        if g.name == "links":
            for b in got.bodies:          # a link, or what it carries (a foot's sock)
                env = claimed_solid(build, build.shapes(b.rigid_with or b.name), TOL)
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


def _overlap(a, b, eps: float = 1e-6) -> bool:
    return all(max(getattr(a.min, c), getattr(b.min, c)) < min(getattr(a.max, c),
                                                               getattr(b.max, c)) - eps
               for c in "XYZ")


def clashes(mech) -> list[dict]:
    """Pairs of parts that intersect by more than :data:`CLASH_MM3`.

    A screw in the part it threads into (``mech.meta["fastened"]``) is not a clash.
    """
    allowed = {frozenset(p) for p in mech.meta.get("fastened", [])}
    parts = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    boxes = {n: p.bounding_box() for n, p in parts.items()}
    out = []
    for a, b in itertools.combinations(parts, 2):
        if frozenset((a, b)) in allowed or not _overlap(boxes[a], boxes[b]):
            continue
        inter = parts[a] & parts[b]
        vol = 0.0 if inter is None else sum(s.volume for s in inter.solids())
        if vol > CLASH_MM3:
            out.append({"a": a, "b": b, "mm3": round(vol, 3)})
    return out


def bad_solids(mech) -> list[dict]:
    """Parts that aren't a valid B-rep solid: one valid solid for a part the design makes
    (laser-cut, printed); a purchased part's model (a manufacturer's STEP: a servo) may be
    a compound of several solids, each of which must be valid."""
    out = []
    for b in mech.bodies:
        if b.part is None:
            continue
        solids = b.part.solids()
        n = len(solids)
        purchased = getattr(b, "fab", None) == "purchased"
        ok = (n >= 1 and all(s.is_valid for s in solids)) if purchased else (
            n == 1 and b.part.is_valid)
        if not ok:
            out.append({"part": b.name, "solids": n, "valid": bool(b.part.is_valid),
                        "fab": getattr(b, "fab", None)})
    return out
