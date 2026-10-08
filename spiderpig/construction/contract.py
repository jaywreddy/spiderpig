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
import logging
import math
from typing import TYPE_CHECKING, cast

import numpy as np
from build123d import Box, Location

from spiderpig.construction.base import Build, Realized
from spiderpig.construction.envelope import claimed_solid
from spiderpig.rounding import rounded
from spiderpig.stack import Disc

if TYPE_CHECKING:
    from build123d import Shape

log = logging.getLogger(__name__)

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


INSIDE_MM = 1e-7    # mm a part's z may pass its column's ends by and still count inside


def _round_bounds(part):
    """What bounds ``part`` round any vertical axis, read off its B-rep, or ``None`` when a
    face or an edge is of a kind not handled: ``(arcs, points)``, every point of the part
    lying within ``r`` of an arc's centre ``(x, y, r)`` or at most as far as the furthest of
    ``points`` (the radial distance from an axis is convex, so over a planar face it peaks on
    the face's edges, over a line at an end; a vertical cylinder, a horizontal circle are
    their arcs), and the ``points`` span the part's z (a planar or vertical cylindrical
    face's z peaks on its edges, and a horizontal circle's edges are at their centre's z)."""
    from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Circle, GeomAbs_Cylinder, GeomAbs_Line, GeomAbs_Plane
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS

    arcs: list[tuple[float, float, float]] = []
    points: list[tuple[float, float, float]] = []
    shape = part.wrapped
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        s = BRepAdaptor_Surface(TopoDS.Face_s(ex.Current()))
        kind = s.GetType()
        if kind == GeomAbs_Cylinder:
            c = s.Cylinder()
            ax = c.Axis()
            if abs(abs(ax.Direction().Z()) - 1.0) > 1e-12:
                return None
            arcs.append((ax.Location().X(), ax.Location().Y(), c.Radius()))
        elif kind != GeomAbs_Plane:
            return None
        ex.Next()
    ex = TopExp_Explorer(shape, TopAbs_EDGE)
    while ex.More():
        c = BRepAdaptor_Curve(TopoDS.Edge_s(ex.Current()))
        kind = c.GetType()
        if kind == GeomAbs_Line:
            for u in (c.FirstParameter(), c.LastParameter()):
                p = c.Value(u)
                points.append((p.X(), p.Y(), p.Z()))
        elif kind == GeomAbs_Circle:
            circ = c.Circle()
            ax = circ.Axis()
            if abs(abs(ax.Direction().Z()) - 1.0) > 1e-12:
                return None
            o = ax.Location()
            arcs.append((o.X(), o.Y(), circ.Radius()))
            points.append((o.X(), o.Y(), o.Z()))
        else:
            return None
        ex.Next()
    return (arcs, points) if points else None


def _in_a_column(build: Build, part, shapes) -> bool:
    """Whether ``part`` lies inside the discs among ``shapes`` round one point, grown by
    :data:`TOL`, by its B-rep alone (:func:`_round_bounds`): its z within a run of them
    with no gap and its furthest from their axis within the least of their radii. A
    sufficient test, no boolean: ``False`` says nothing."""
    discs = [p for p in shapes if isinstance(p.shape, Disc)]
    if not discs:
        return False
    bounds = _round_bounds(part)
    if bounds is None:
        return False
    arcs, points = bounds
    z_lo = min(p[2] for p in points)
    z_hi = max(p[2] for p in points)
    for at in {p.shape.at for p in discs}:
        x, y = (float(v) for v in build.xy(at))
        far = max([math.hypot(a - x, b - y) + r for a, b, r in arcs]
                  + [math.hypot(a - x, b - y) for a, b, _ in points])
        slots = sorted((*build.plan.slot_z(p), p.shape.r + TOL) for p in discs
                       if p.shape.at == at)
        z = z_lo
        for z0, z1, r in slots:     # the run of slots from the part's foot to its head
            if z0 <= z + INSIDE_MM and z1 > z and r >= far:
                z = max(z, z1)
        if z >= z_hi - INSIDE_MM:
            return True
    return False


def _outside_claims(build: Build, part, shapes, envelope) -> float:
    """:func:`_outside` of ``part`` and the claims ``shapes`` (``envelope`` makes their
    solid): 0 without a boolean for a part plainly inside a column of discs
    (:func:`_in_a_column`; at most a speck of :data:`INSIDE_MM` past its ends)."""
    if part is not None and part.solids() and _in_a_column(build, part, shapes):
        return 0.0
    return _outside(part, envelope(shapes))


def _slab(z0: float, z1: float, size: float = 1e4):
    return Box(size, size, z1 - z0).moved(Location((0.0, 0.0, (z0 + z1) / 2)))


def _realized_for(design, groups) -> list:
    """The groups :func:`check_side` realizes to check ``groups`` (``None``: every group):
    those named, and every group before a named one that cuts (a plate cuts the holes the
    groups before it asked for), in build order."""
    if groups is None:
        return list(design.groups)
    names = set(groups)
    unknown = names - {g.name for g in design.groups}
    if unknown:
        raise ValueError(f"no such group: {', '.join(sorted(unknown))} (the side's groups: "
                         f"{', '.join(g.name for g in design.groups)})")
    last = max((i for i, g in enumerate(design.groups) if g.cuts and g.name in names),
               default=-1)
    return [g for i, g in enumerate(design.groups) if i <= last or g.name in names]


def _items(build: Build, g, got: Realized) -> list[tuple]:
    """What the contract checks of ``got`` (group ``g`` realized at ``build``'s angle), per
    body: ``(body, the claims its part must lie in, only its part below the servo face)``;
    the frame plates' claims are their layers (``None``)."""
    if g.name == "frame":
        return [(b, None, False) for b in got.bodies]
    if g.name == "drive":
        shapes = [p for p in build.shapes("crank") if p.label == "servo horn"] + build.shapes(
            g.name)
        return [(b, shapes, True) for b in got.bodies]
    if g.name == "links":       # a link, or what it carries (a foot's sock)
        return [(b, build.shapes(b.rigid_with or b.name), False) for b in got.bodies]
    shapes = build.shapes(g.name)
    return [(b, shapes, False) for b in got.bodies]


class _Envelopes:
    """:func:`claimed_solid` of a list of claims at one angle, each list made once."""

    def __init__(self, build: Build):
        self.build = build
        self.made: dict[tuple, tuple] = {}     # the ids -> (the solid, the claims kept alive)

    def __call__(self, shapes):
        key = tuple(id(s) for s in shapes)
        if key not in self.made:
            self.made[key] = (claimed_solid(self.build, shapes, TOL), shapes)
        return self.made[key][0]


def _check_group(build: Build, g, got: Realized, envelope: _Envelopes,
                 measured: list | None = None) -> list[str]:
    """The contract's violations by ``got``, group ``g`` realized at ``build``'s angle;
    ``measured`` gets each checked body's ``(part, volume outside)`` (its part below the
    servo face, for the drive)."""
    problems: list[str] = []
    if g.name == "frame":
        zs = [build.z(k) for k in (0, build.top)]
        for b in got.bodies:
            bb = cast("Shape", b.part).bounding_box()
            if not any(abs(bb.min.Z - z0) < TOL and abs(bb.max.Z - z1) < TOL for z0, z1 in zs):
                problems.append(f"{b.name}: frame plate outside its layer")
        return problems
    below = _slab(-1e3, build.z(build.top)[1])
    for b, shapes, clip in _items(build, g, got):
        part = (b.part & below if b.part is not None else None) if clip else b.part
        vol = _outside_claims(build, part, shapes, envelope)
        if measured is not None:
            measured.append((part, vol))
        if vol <= MAX_OUTSIDE:
            continue
        if g.name == "drive":
            problems.append(f"{b.name}: {vol:.3f} mm^3 below the servo face, "
                            "outside the horn claim")
        elif g.name == "links":
            problems.append(f"{b.name}: {vol:.3f} mm^3 outside its claim")
        else:
            problems.append(f"{g.name}/{b.name}: {vol:.3f} mm^3 outside its claims")
    return problems


def check_side(design, mech, groups=None) -> list[str]:
    """Violations of the contract for one side at ``mech``'s crank angle (empty if none).

    ``groups``: the names of the groups to check (default every group); the others are
    realized only where a named plate group needs their holes, and never checked."""
    build = Build(design.ctx, design.plan, mech)
    problems: list[str] = []
    done = Realized()
    envelope = _Envelopes(build)
    names = None if groups is None else set(groups)
    for g in _realized_for(design, groups):
        got = g.realize(build, done)
        done.merge(got)
        if names is None or g.name in names:
            problems += _check_group(build, g, got, envelope)
    return problems


MOVE_MM = 1e-6    # mm a point may stray from where a group's motion carries it


def _joints_xy(mech, name: str) -> np.ndarray | None:
    try:
        b = mech.body(name)
    except KeyError:
        return None
    return np.array([(b.pose @ j.pose).matrix[:2, 3] for j in b.joints], dtype=float)


def _fit(p: np.ndarray | None, q: np.ndarray | None):
    """The planar rigid motion ``(rotation, translation)`` taking the points ``p`` onto
    ``q``, or ``None`` (fewer than two distinct points, or no motion does it to
    :data:`MOVE_MM`)."""
    if p is None or q is None or len(p) != len(q) or not len(p):
        return None
    pc, qc = p.mean(axis=0), q.mean(axis=0)
    a, b = p - pc, q - qc
    if float(np.abs(a).max()) < 1e-3:
        return None                     # one point: its turn is anyone's guess
    th = math.atan2(float(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])),
                    float(np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1])))
    r = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    t = qc - r @ pc
    if float(np.abs(p @ r.T + t - q).max()) > MOVE_MM:
        return None
    return r, t


def _motion_of(motion, body, ref: Build, other: Build):
    """``body``'s rigid motion from ``ref``'s angle to ``other``'s under the group's
    :class:`construction.base.Motion` (``None``: unknown)."""
    if motion.point is not None:
        return np.eye(2), other.xy(motion.point) - ref.xy(motion.point)
    host = body.rigid_with or body.name
    return _fit(_joints_xy(ref.mech, host), _joints_xy(other.mech, host))


def _carried(shape, move, ref: Build, other: Build) -> bool:
    """Whether the claim ``shape`` at ``other``'s angle is ``move`` applied to it at
    ``ref``'s (each point it is drawn from, to :data:`MOVE_MM`)."""
    s = shape.shape
    r, t = move
    return all(float(np.abs(r @ ref.xy(p) + t - other.xy(p)).max()) <= MOVE_MM
               for p in ((s.at,) if isinstance(s, Disc) else (s.a, s.b)))


def _holds_everywhere(g, got: Realized, ref: Build, others: list[Build],
                      envelope: _Envelopes, measured: list) -> bool:
    """Whether group ``g``'s verdict at ``ref``'s angle (no violation) holds at every one
    of ``others`` without realizing it there.

    Its :meth:`~construction.base.Group.motion` says how its parts move. A frame plate's
    rule (its layer's z) holds under any planar motion; for any other part, the claims
    its motion carries along to every angle (a link's own, the crank's on the crank, an
    axle's on its point; at any angle the envelope holds at least those, moved as the part
    is) must already hold the part here, to half the contract's allowance: the volume
    outside them is the volume outside at every angle, and the envelope there only
    bigger."""
    motion = g.motion(got)
    if motion is None:
        return False
    if g.name == "frame":
        zs = [ref.z(k) for k in (0, ref.top)]
        for b in got.bodies:
            bb = cast("Shape", b.part).bounding_box()
            if not any(abs(bb.min.Z - z0) < TOL / 2 and abs(bb.max.Z - z1) < TOL / 2
                       for z0, z1 in zs):
                return False
        return True
    for (b, shapes, _), (part, vol) in zip(_items(ref, g, got), measured, strict=True):
        moves = [_motion_of(motion, b, ref, o) for o in others]
        if any(m is None for m in moves):
            return False
        kept = [s for s in shapes
                if all(_carried(s, m, ref, o) for m, o in zip(moves, others, strict=True))]
        if len(kept) != len(shapes):
            vol = _outside_claims(ref, part, kept, envelope)
        if vol > MAX_OUTSIDE / 2:
            return False
    return True


GUARD_TURN = (math.sqrt(5.0) - 1.0) / 2.0
"""The guard's crank angle (:func:`check_sides`): this fraction of a turn past the first
angle (irrational: never an angle a caller checks, nor a symmetry of the cycle)."""


class DeclaredMotionWarning(UserWarning):
    """A group's :meth:`~construction.base.Group.motion` doesn't hold: its parts realized at
    the guard angle aren't its first angle's parts moved so (a bug in the group's
    declaration; :func:`check_sides` checks that group exactly at every angle instead)."""


def _mass(part) -> tuple[float, float, np.ndarray, np.ndarray]:
    """``part``'s volume, area, centre of mass and inertia tensor about it: what a rigid
    motion carries along (and none of it where a curve's seam happens to sit)."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    vol, surf = GProp_GProps(), GProp_GProps()
    BRepGProp.VolumeProperties_s(part.wrapped, vol)
    BRepGProp.SurfaceProperties_s(part.wrapped, surf)
    c, m = vol.CentreOfMass(), vol.MatrixOfInertia()
    return (vol.Mass(), surf.Mass(), np.array([c.X(), c.Y(), c.Z()]),
            np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]))


def _moves_as_declared(g, ref_got: Realized, got: Realized, ref: Build, guard: Build) -> str:
    """Why ``got`` (group ``g`` realized at ``guard``'s angle) isn't ``ref_got`` moved by
    the group's declared motion ("": it is): the same bodies, each part's volume and area,
    and its centre of mass and inertia tensor where the motion takes them (1e-5 mm; 1e-6
    relative). A cheap necessary condition, no boolean (``test_each_group_moves_as_it_says``
    compares the solids themselves)."""
    motion = g.motion(ref_got)
    if motion is None:
        return "no motion declared"
    a, b = [x.name for x in ref_got.bodies], [x.name for x in got.bodies]
    if a != b:
        return f"bodies {sorted(set(a) ^ set(b)) or 'reordered'}"
    for pa, pb in zip(ref_got.bodies, got.bodies, strict=True):
        if (pa.part is None) != (pb.part is None):
            return f"{pa.name}: a part at one angle only"
        if pa.part is None:
            continue
        move = _motion_of(motion, pa, ref, guard)
        if move is None:
            return f"{pa.name}: its host's motion is unknown"
        (va, aa, ca, ia), (vb, ab, cb, ib) = _mass(pa.part), _mass(pb.part)
        if abs(va - vb) > 1e-6 * max(abs(va), 1.0):
            return f"{pa.name}: volume {va:.6g} -> {vb:.6g} mm^3"
        if abs(aa - ab) > 1e-6 * max(aa, 1.0):
            return f"{pa.name}: area {aa:.6g} -> {ab:.6g} mm^2"
        r3 = np.eye(3)
        r3[:2, :2] = move[0]
        want_c = r3 @ ca + np.array([*move[1], 0.0])
        if float(np.abs(want_c - cb).max()) > 1e-5:
            return f"{pa.name}: its centre of mass isn't where the motion takes it"
        if float(np.abs(r3 @ ia @ r3.T - ib).max()) > 1e-6 * max(float(np.abs(ib).max()), 1.0):
            return f"{pa.name}: it isn't turned as the motion turns it"
    return ""


def _guard(order, skipped: set[str], refs: dict, ref: Build, guard: Build) -> set[str]:
    """The groups of ``skipped`` (checked once, carried to the other angles by their
    declared motion) whose parts at ``guard``'s angle aren't their parts at ``ref``'s moved
    so, each warned about (:class:`DeclaredMotionWarning`) for :func:`check_sides` to check
    exactly. A side that can't be realized there fails the guard for every skipped group."""
    import warnings

    last = max(i for i, g in enumerate(order) if g.name in skipped)
    done = Realized()
    bad: dict[str, str] = {}
    try:
        for g in order[:last + 1]:
            got = g.realize(guard, done)
            done.merge(got)
            if g.name in skipped:
                why = _moves_as_declared(g, refs[g.name], got, ref, guard)
                if why:
                    bad[g.name] = why
    except Exception as e:      # noqa: BLE001 - then checked exactly at every angle
        bad = dict.fromkeys(skipped, f"unbuildable at the guard angle: {e}")
    for name, why in sorted(bad.items()):
        msg = f"{name}: declared motion doesn't hold ({why}); checked at every angle instead"
        log.error("contract: %s", msg)
        warnings.warn(DeclaredMotionWarning(msg), stacklevel=3)
    return set(bad)


def check_sides(design, tmpl, ts, groups=None) -> list[list[str]]:
    """:func:`check_side` at each crank angle of ``ts`` (``tmpl`` the side's template), in
    order: the same verdicts, realizing the side twice rather than at every angle.

    The side is realized and checked at the first angle exactly as :func:`check_side` does.
    A group whose parts merely move with the angle (:meth:`construction.base.Group.motion`)
    and that holds there with the claims that move with it (:func:`_holds_everywhere`) holds
    at every angle; every other group (one that changes shape with the angle, one with a
    violation or near one) is realized again at each other angle, with the groups before
    it, and checked there exactly.

    The declarations are checked at run time: the side is realized once more at a guard
    angle no caller checks (:data:`GUARD_TURN` of a turn on), and a group whose parts there
    aren't its first angle's moved as it declares is checked exactly at every angle, with a
    :class:`DeclaredMotionWarning` (a bug to fix). The limit: a group is realized at the
    first angle and the guard's only, and trusted to build alike at the others, so a part
    (or a :class:`~construction.base.ConstructionError`) that a construction makes at one
    of the other checked angles alone isn't seen here; :func:`check_side` at that angle
    sees it, and the nightly gate compares the two on its designs."""
    ts = [float(t) for t in ts]
    if not ts:
        return []
    builds = [Build(design.ctx, design.plan, tmpl.freeze_at(t)) for t in ts]
    ref, others = builds[0], builds[1:]
    names = None if groups is None else set(groups)
    order = _realized_for(design, groups)
    checked = [g for g in order if names is None or g.name in names]
    done = Realized()
    envelope = _Envelopes(ref)
    first: list[str] = []
    again: set[str] = set()
    refs: dict[str, Realized] = {}
    for g in order:
        got = g.realize(ref, done)
        done.merge(got)
        if g not in checked:
            continue
        refs[g.name] = got
        measured: list = []
        first += _check_group(ref, g, got, envelope, measured)
        if others and not _holds_everywhere(g, got, ref, others, envelope, measured):
            again.add(g.name)
    skipped = {g.name for g in checked} - again
    if others and skipped:
        t = ts[0] + 2 * math.pi * GUARD_TURN
        while any(abs(t - u) < 1e-3 for u in ts):
            t += 2 * math.pi * GUARD_TURN / 7
        again |= _guard(order, skipped, refs, ref,
                        Build(design.ctx, design.plan, tmpl.freeze_at(t)))
    if again:
        log.debug("contract: realized again at every angle: %s", ", ".join(sorted(again)))
    out = [first]
    for build in others:
        problems: list[str] = []
        if again:
            last = max(i for i, g in enumerate(order) if g.name in again)
            done = Realized()
            envelope = _Envelopes(build)
            for g in order[:last + 1]:
                got = g.realize(build, done)
                done.merge(got)
                if g.name in again:
                    problems += _check_group(build, g, got, envelope)
        out.append(problems)
    return out


def _overlap(a, b, eps: float = 1e-6) -> bool:
    return all(max(getattr(a.min, c), getattr(b.min, c)) < min(getattr(a.max, c),
                                                               getattr(b.max, c)) - eps
               for c in "XYZ")


def clashes(mech, names=None) -> list[dict]:
    """Pairs of parts that intersect by more than :data:`CLASH_MM3`.

    A screw in the part it threads into (``mech.meta["fastened"]``) is not a clash.
    ``names``: only the pairs with at least one of these bodies (default every pair).
    """
    allowed = {frozenset(p) for p in mech.meta.get("fastened", [])}
    parts = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    boxes = {n: p.bounding_box() for n, p in parts.items()}
    some = None if names is None else set(names)
    out = []
    for a, b in itertools.combinations(parts, 2):
        if some is not None and a not in some and b not in some:
            continue
        if frozenset((a, b)) in allowed or not _overlap(boxes[a], boxes[b]):
            continue
        inter = parts[a] & parts[b]
        vol = 0.0 if inter is None else sum(s.volume for s in inter.solids())
        if vol > CLASH_MM3:
            out.append({"a": a, "b": b, "mm3": rounded(vol, 3)})
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
