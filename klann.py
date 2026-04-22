"""Symbolic Klann walking-linkage geometry and mechanism assembly.

The Klann linkage is a 6-bar planar mechanism that converts continuous
rotation of a crank into a foot path well suited to walking robots. This
module keeps the geometry purely symbolic via sympy, with a single scalar
time symbol ``t`` driving the crank. A ``phase`` offset is baked into M's
parametric angle at construction so multi-leg assemblies pre-bake one
:class:`KlannSolution` per leg.

Downstream code (shapes.py, layout.py, the glTF bake) evaluates the
:class:`KlannSolution` at float ``t`` via :meth:`KlannSolution.joints_at`.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from contextlib import contextmanager
from dataclasses import dataclass, replace
from functools import cached_property
from typing import Any

import numpy as np
import sympy as sp
from sympy.geometry import Circle, Point, Segment

from mechanism import (
    Body,
    BodyTemplate,
    Joint,
    JointTemplate,
    Mechanism,
    MechanismTemplate,
    Pose,
    offset_pose_at,
    translation_pose_at,
)


# Optional profiler hook. ``viewer/bake_gltf.py`` sets this to its
# ``_Profiler`` instance at the start of a bake so the per-frame hot path
# (create_klann_geometry, lambdify, joints_at, body wiring) shows up as
# labelled rows in the bake profile summary. Leave ``None`` in normal use.
_BAKE_PROFILER: Any = None


@contextmanager
def _maybe_timed(label: str):
    prof = _BAKE_PROFILER
    if prof is None:
        yield
        return
    with prof.timed(label):
        yield


def custom_intersection(c1, c2):
    """Return the intersection points of two sympy Circles.

    Unlike ``sympy.geometry.Circle.intersection``, this avoids the expensive
    ``simplify()`` path: it inlines the standard two-circle formula and
    returns raw :class:`sympy.geometry.Point` objects. The fast path
    preserves the original semantics for coincident and tangent circles.
    """
    if c1.center == c2.center:
        if c2.radius == c1.radius:
            return c2
        return []

    # Take raw coord differences: constructing the intermediate Point2D
    # via ``c2.center - c1.center`` triggers an expensive normalization
    # pass on symbolic inputs (64s for the E intersection in create_klann_geometry).
    dx = c2.center.x - c1.center.x
    dy = c2.center.y - c1.center.y
    d = (dy**2 + dx**2) ** 0.5
    a = (c1.radius**2 - c2.radius**2 + d**2) / (2 * d)

    x2 = c1.center.x + (dx * a / d)
    y2 = c1.center.y + (dy * a / d)

    h = (c1.radius**2 - a**2) ** 0.5
    rx = -dy * (h / d)
    ry = dx * (h / d)

    xi_1 = x2 + rx
    xi_2 = x2 - rx
    yi_1 = y2 + ry
    yi_2 = y2 - ry

    ret = [Point(xi_1, yi_1)]
    if xi_1 != xi_2 or yi_1 != yi_2:
        ret.append(Point(xi_2, yi_2))
    return ret


@dataclass
class KlannSolution:
    """Symbolic Klann trajectory.

    One scalar time symbol ``t`` drives the crank. The ``phase`` offset is
    baked into M's parametric angle at construction, so each leg of a
    multi-arm assembly carries its own solution. Downstream callers
    evaluate at a concrete ``t`` via :meth:`joints_at`.
    """

    t: sp.Symbol
    orientation: int
    phase: float
    points: dict[str, Any]      # keys: O, A, B, C, D, E, F, M (sympy Point)
    segments: dict[str, Any]    # keys: b1, b2, b3, b4, conn (sympy Segment)

    @cached_property
    def callables(self) -> dict[str, Callable[[float], tuple[float, float]]]:
        """One numpy callable ``(t,) -> (x, y)`` per named point."""
        with _maybe_timed("4.1b_klann.lambdify"):
            return {
                name: sp.lambdify((self.t,), (pt.x, pt.y), modules="numpy", cse=True)
                for name, pt in self.points.items()
            }

    def joints_at(self, t_value: float) -> dict[str, tuple[float, float]]:
        """Evaluate every named point at ``t_value`` and return float (x, y) pairs."""
        with _maybe_timed("4.1c_klann.joints_at_eval"):
            out: dict[str, tuple[float, float]] = {}
            for name, fn in self.callables.items():
                x, y = fn(float(t_value))
                out[name] = (float(x), float(y))
            return out


def create_klann_geometry(orientation: int = 1, phase: float = 0.0) -> KlannSolution:
    """Solve the Klann linkage symbolically in a single time symbol ``t``.

    Parameters match the reference mechanism (Joseph C. Klann's published
    proportions). ``orientation=+1`` yields a right-hand linkage; ``-1`` a
    left-hand mirror. The crank angle is ``t + phase`` — bake a different
    ``phase`` per leg to build a multi-arm walker.
    """
    with _maybe_timed("4.1a_klann.create_geometry"):
        t = sp.Symbol("t", real=True)

        O = Point(0, 0)
        mOA = 60
        OA = Point(0, -mOA)
        rotA = orientation * 66.28 * np.pi / 180
        A = OA.rotate(rotA)

        OB = Point(0, 1.121 * mOA)
        rotB = orientation * -41.76 * np.pi / 180
        B = OB.rotate(rotB)

        M_circ = Circle(O, 0.412 * mOA)
        M = M_circ.arbitrary_point()
        [sym] = M.free_symbols
        M = M.subs(sym, t + phase)
        C_circ1 = Circle(M, 1.143 * mOA)
        C_circ2 = Circle(A, 0.909 * mOA)
        with _maybe_timed("4.1a_klann.create_geometry.intersect_C"):
            C_ints = custom_intersection(C_circ1, C_circ2)
        i = 0
        if orientation == -1:
            i = 1
        C = C_ints[i]

        MC_length = M.distance(C)
        CD_length = 0.726 * mOA
        Dx = C.x + ((C.x - M.x) / MC_length) * CD_length
        Dy = C.y + ((C.y - M.y) / MC_length) * CD_length
        D = Point(Dx, Dy)

        D_circ = Circle(D, 0.93 * mOA)
        B_circ = Circle(B, 0.8 * mOA)
        with _maybe_timed("4.1a_klann.create_geometry.intersect_E"):
            E_ints = custom_intersection(B_circ, D_circ)
        E = E_ints[i]

        ED_length = E.distance(D)
        DF_length = 2.577 * mOA
        Fx = D.x + ((D.x - E.x) / ED_length) * DF_length
        Fy = D.y + ((D.y - E.y) / ED_length) * DF_length
        F = Point(Fx, Fy)

        b1 = Segment(M, D)
        b2 = Segment(B, E)
        b3 = Segment(A, C)
        b4 = Segment(E, F)
        conn = Segment(O, M)

        return KlannSolution(
            t=t,
            orientation=orientation,
            phase=float(phase),
            points={"O": O, "A": A, "B": B, "C": C, "D": D, "E": E, "F": F, "M": M},
            segments={"b1": b1, "b2": b2, "b3": b3, "b4": b4, "conn": conn},
        )


# Per-leg connection template. Body names pick up a ``name_suffix`` in
# :func:`build_klann_mechanism` so multiple legs coexist in one Mechanism.
_CONN_TEMPLATE: list[tuple[tuple[int, str, str], tuple[int, str, str]]] = [
    ((0, "torso", "A"), (0, "b3", "A")),
    ((0, "torso", "B"), (0, "b2", "B")),
    ((0, "torso", "O"), (0, "conn", "O")),
    ((0, "conn", "M"), (0, "b1", "M")),
    ((0, "b1", "C"), (0, "b3", "C")),
    ((0, "b1", "D"), (0, "b4", "D")),
    ((0, "b2", "E"), (0, "b4", "E")),
    ((0, "conn", "O"), (0, "coupler", "O")),
]


def build_klann_mechanism(
    solution: KlannSolution,
    t: float,
    *,
    thickness: float,
    z_base: float = 0.0,
    name_suffix: str = "",
    with_parts: bool = True,
) -> Mechanism:
    """Instantiate one Klann leg from a pre-baked :class:`KlannSolution`.

    Evaluates ``solution`` at ``t`` to get concrete joint XYs, then wires
    up the seven bodies (torso, coupler, b1..b4, conn). Joint Z layering
    is phase-invariant: pins on the positive layer sit at
    ``z = thickness * layer`` and on the negative layer one layer below.

    Stacking multiple legs in Z is done via ``z_base`` — set on the
    ``coupler`` body's initial pose. :meth:`Mechanism.solved` BFS then
    propagates the offset to every body in the leg.

    ``name_suffix`` is appended to every body name and the connection
    tuples so legs can be merged into one :class:`Mechanism`.
    """
    xy = solution.joints_at(t)

    with _maybe_timed("4.1d_klann.assemble_leg"):
        def _jp(name: str, layer: int) -> tuple[Joint, Joint]:
            x, y = xy[name]
            pos = Joint(name=name, pose=Pose.from_translation([x, y, thickness * layer]))
            neg = Joint(
                name=name, pose=Pose.from_translation([x, y, thickness * (layer - 1)])
            )
            return pos, neg

        pos_A, neg_A = _jp("A", 1)
        pos_B, neg_B = _jp("B", 1)
        pos_C, neg_C = _jp("C", 1)
        pos_D, neg_D = _jp("D", 1)
        pos_E, neg_E = _jp("E", 1)
        pos_M, neg_M = _jp("M", 1)
        pos_O, neg_O = _jp("O", 1)
        bt_pos, _ = _jp("B", 2)

        # Mirrored legs own the M joint on the opposite layer so a shared crank
        # couples the right-hand and left-hand legs at the same physical pin.
        if solution.orientation == -1:
            b1_M, conn_M = pos_M, neg_M
        else:
            b1_M, conn_M = neg_M, pos_M

        def _nm(base: str) -> str:
            return f"{base}{name_suffix}"

        if with_parts:
            from shapes import create_shaft_connector, create_torso, rationalize_segment

            def _seg(p: str, q: str) -> Segment:
                return Segment(Point(*xy[p]), Point(*xy[q]))

            b1_body = rationalize_segment(_seg("M", "D"), [b1_M, neg_C, neg_D], _nm("b1"))
            b2_body = rationalize_segment(_seg("B", "E"), [neg_B, neg_E], _nm("b2"))
            b3_body = rationalize_segment(_seg("A", "C"), [neg_A, pos_C], _nm("b3"))
            b4_body = rationalize_segment(_seg("E", "F"), [pos_E, pos_D], _nm("b4"))
            conn_body = rationalize_segment(_seg("O", "M"), [neg_O, conn_M], _nm("conn"))
            torso_part = create_torso([pos_A, pos_O, bt_pos])
            coupler_part = create_shaft_connector()
        else:
            b1_body = Body(name=_nm("b1"), joints=[b1_M, neg_C, neg_D], color="green")
            b2_body = Body(name=_nm("b2"), joints=[neg_B, neg_E], color="green")
            b3_body = Body(name=_nm("b3"), joints=[neg_A, pos_C], color="green")
            b4_body = Body(name=_nm("b4"), joints=[pos_E, pos_D], color="green")
            conn_body = Body(name=_nm("conn"), joints=[neg_O, conn_M], color="green")
            torso_part = None
            coupler_part = None

        torso = Body(
            name=_nm("torso"),
            part=torso_part,
            joints=[pos_A, pos_O, bt_pos],
            color="yellow",
        )
        coupler = Body(
            name=_nm("coupler"),
            part=coupler_part,
            joints=[pos_O],
            color="blue",
            pose=Pose.from_translation([0.0, 0.0, z_base]),
        )

        connections = [
            ((pi, f"{pn}{name_suffix}", pj), (ci, f"{cn}{name_suffix}", cj))
            for (pi, pn, pj), (ci, cn, cj) in _CONN_TEMPLATE
        ]

        return Mechanism(
            name=f"klann{name_suffix}",
            bodies=[coupler, b1_body, b2_body, b3_body, b4_body, conn_body, torso],
            connections=connections,
        )


def build_klann_template(
    solution: KlannSolution,
    *,
    thickness: float,
    z_base: float = 0.0,
    name_suffix: str = "",
) -> MechanismTemplate:
    """Parametric-in-t analogue of :func:`build_klann_mechanism`.

    Builds one Klann leg as a :class:`MechanismTemplate` whose joint poses
    are closures over ``solution.callables[name]``. The template can be
    sampled over an entire ``ts`` array in one vectorized pass; see
    :meth:`MechanismTemplate.sample`. No build123d parts are attached —
    templates drive the frame loop, not the reference build.
    """
    callables = solution.callables

    def _jpt(name: str, layer: int) -> tuple[JointTemplate, JointTemplate]:
        xy_fn = callables[name]
        pos = JointTemplate(
            name=name, pose_at=translation_pose_at(xy_fn, z=thickness * layer)
        )
        neg = JointTemplate(
            name=name, pose_at=translation_pose_at(xy_fn, z=thickness * (layer - 1))
        )
        return pos, neg

    pos_A, neg_A = _jpt("A", 1)
    pos_B, neg_B = _jpt("B", 1)
    pos_C, neg_C = _jpt("C", 1)
    pos_D, neg_D = _jpt("D", 1)
    pos_E, neg_E = _jpt("E", 1)
    pos_M, neg_M = _jpt("M", 1)
    pos_O, neg_O = _jpt("O", 1)
    bt_pos, _ = _jpt("B", 2)

    if solution.orientation == -1:
        b1_M, conn_M = pos_M, neg_M
    else:
        b1_M, conn_M = neg_M, pos_M

    def _nm(base: str) -> str:
        return f"{base}{name_suffix}"

    b1_body = BodyTemplate(name=_nm("b1"), joints=[b1_M, neg_C, neg_D], color="green")
    b2_body = BodyTemplate(name=_nm("b2"), joints=[neg_B, neg_E], color="green")
    b3_body = BodyTemplate(name=_nm("b3"), joints=[neg_A, pos_C], color="green")
    b4_body = BodyTemplate(name=_nm("b4"), joints=[pos_E, pos_D], color="green")
    conn_body = BodyTemplate(name=_nm("conn"), joints=[neg_O, conn_M], color="green")
    torso = BodyTemplate(
        name=_nm("torso"),
        joints=[pos_A, pos_O, bt_pos],
        color="yellow",
    )
    coupler = BodyTemplate(
        name=_nm("coupler"),
        joints=[pos_O],
        color="blue",
        base_pose=Pose.from_translation([0.0, 0.0, z_base]),
    )

    connections = [
        ((pi, f"{pn}{name_suffix}", pj), (ci, f"{cn}{name_suffix}", cj))
        for (pi, pn, pj), (ci, cn, cj) in _CONN_TEMPLATE
    ]

    return MechanismTemplate(
        name=f"klann{name_suffix}",
        bodies=[coupler, b1_body, b2_body, b3_body, b4_body, conn_body, torso],
        connections=connections,
    )


def build_multi_leg_mechanism(
    n_legs: int,
    t: float,
    *,
    thickness: float,
    z_stride: float | None = None,
    with_parts: bool = True,
) -> Mechanism:
    """Stack ``n_legs`` copies of the Klann leg at evenly-spaced phases.

    Leg ``k`` is solved at ``phase = 2π·k/n_legs`` and offset by
    ``k * z_stride`` in Z (default: ``3 * thickness``). Bodies are named
    ``torso_leg0``, ``b1_leg0``, …, so each leg is a disjoint sub-graph;
    :meth:`Mechanism.solved` walks every component independently.
    """
    if n_legs < 1:
        raise ValueError(f"n_legs must be >= 1, got {n_legs}")
    if z_stride is None:
        z_stride = 3.0 * thickness

    bodies: list[Body] = []
    connections: list = []
    for k in range(n_legs):
        phase = 2.0 * math.pi * k / n_legs
        sol = create_klann_geometry(orientation=1, phase=phase)
        leg = build_klann_mechanism(
            sol,
            t,
            thickness=thickness,
            z_base=k * z_stride,
            name_suffix=f"_leg{k}",
            with_parts=with_parts,
        )
        bodies.extend(leg.bodies)
        connections.extend(leg.connections)

    return Mechanism(name="klann_multi", bodies=bodies, connections=connections)


def build_multi_leg_template(
    n_legs: int,
    *,
    thickness: float,
    z_stride: float | None = None,
) -> MechanismTemplate:
    """Parametric-in-t analogue of :func:`build_multi_leg_mechanism`.

    Precomputes one :class:`KlannSolution` per leg (one per
    ``phase = 2π·k/n_legs``) — this is the expensive symbolic work that
    used to be redone per frame. All legs share the template; the frame
    loop only needs :meth:`MechanismTemplate.sample`.
    """
    if n_legs < 1:
        raise ValueError(f"n_legs must be >= 1, got {n_legs}")
    if z_stride is None:
        z_stride = 3.0 * thickness

    bodies: list[BodyTemplate] = []
    connections: list = []
    for k in range(n_legs):
        phase = 2.0 * math.pi * k / n_legs
        sol = create_klann_geometry(orientation=1, phase=phase)
        leg = build_klann_template(
            sol,
            thickness=thickness,
            z_base=k * z_stride,
            name_suffix=f"_leg{k}",
        )
        bodies.extend(leg.bodies)
        connections.extend(leg.connections)

    return MechanismTemplate(
        name="klann_multi", bodies=bodies, connections=connections
    )


# ---------------------------------------------------------------------------
# Composition primitives for multi-linkage assemblies
# ---------------------------------------------------------------------------
#
# The 2016 project stacked 2–4 Klann legs into walker sculptures by sharing
# rigid bodies across legs (a single crank driving a mirrored pair, one
# torso plate carrying pivots for every leg, one shaft coupler spanning
# multiple decks). Each pattern reduces to a small mechanism rewrite: drop
# one or more bodies and redirect the connection edges that used to target
# them. These helpers are pure — they return a new :class:`Mechanism` —
# mirroring the style of :meth:`Mechanism.solved`.


def voffset_joint(j: Joint, dz: float, new_name: str | None = None) -> Joint:
    """Return a copy of ``j`` shifted by ``dz`` in Z, optionally renamed."""
    m = j.pose.matrix.copy()
    m[2, 3] += dz
    return Joint(name=new_name or j.name, pose=Pose.from_matrix(m))


def _merge_mechanisms(name: str, mechs: list[Mechanism]) -> Mechanism:
    bodies: list[Body] = []
    connections: list = []
    for m in mechs:
        bodies.extend(m.bodies)
        connections.extend(m.connections)
    return Mechanism(name=name, bodies=bodies, connections=connections)


def _rewrite_connections(connections, rewriter):
    """Map each (a, b) endpoint via ``rewriter(idx, body, joint)``.

    ``rewriter`` may return a replacement ``(idx, body, joint)`` tuple, or
    ``None`` to drop that entire connection.
    """
    out = []
    for a, b in connections:
        na = rewriter(*a)
        nb = rewriter(*b)
        if na is None or nb is None:
            continue
        out.append((na, nb))
    return out


def combine_connectors(
    mech: Mechanism,
    suffix_a: str,
    suffix_b: str,
    *,
    new_name: str = "conn",
) -> Mechanism:
    """Fuse two ``conn`` crank bodies into one rigid body with joints from both.

    2016 analogue: ``combine_connectors(conn1, conn2)`` (Project/main.py:494).
    The two cranks share the world-origin pivot O, so a single rigid body
    driven by one shaft co-rotates both mirrored legs.
    """
    name_a = f"conn{suffix_a}"
    name_b = f"conn{suffix_b}"
    body_a = mech.body(name_a)
    body_b = mech.body(name_b)

    # Rename joints with the leg suffix so the merged body has unique names.
    joints = [replace(j, name=f"{j.name}{suffix_a}") for j in body_a.joints] + [
        replace(j, name=f"{j.name}{suffix_b}") for j in body_b.joints
    ]

    if body_a.part is not None and body_b.part is not None:
        part = body_a.part + body_b.part
    else:
        part = body_a.part or body_b.part

    fused = Body(name=new_name, part=part, joints=joints, color=body_a.color)

    bodies = []
    for b in mech.bodies:
        if b.name == name_a:
            bodies.append(fused)  # preserve position of the first old body
        elif b.name == name_b:
            continue
        else:
            bodies.append(b)

    def rewrite(idx, body, joint):
        if body == name_a:
            return (idx, new_name, f"{joint}{suffix_a}")
        if body == name_b:
            return (idx, new_name, f"{joint}{suffix_b}")
        return (idx, body, joint)

    return Mechanism(
        name=mech.name,
        bodies=bodies,
        connections=_rewrite_connections(mech.connections, rewrite),
    )


def fuse_couplers(
    mech: Mechanism,
    suffixes: list[str],
    *,
    deck_dzs: list[float] | None = None,
    new_name: str = "coupler",
) -> Mechanism:
    """Collapse per-leg coupler bodies into a single shaft-coupler body.

    The primary coupler keeps its ``part`` and ``pose`` (so the existing
    ``z_base`` anchor still seeds BFS). Each additional leg contributes one
    ``O``-named joint on the shared coupler, renamed with that leg's
    suffix and optionally offset in Z by ``deck_dzs[k]`` to span decks.

    2016 analogue: the single ``coupler`` body with joints ``pos_O`` at layer 1
    and ``O_3`` at layer 5 (Project/main.py:696).
    """
    if deck_dzs is None:
        deck_dzs = [0.0] * len(suffixes)
    if len(deck_dzs) != len(suffixes):
        raise ValueError("deck_dzs length must match suffixes length")

    primary_name = f"coupler{suffixes[0]}"
    primary = mech.body(primary_name)

    # Rebuild the primary coupler's joint list: one O per suffix, suffix-renamed.
    joints: list[Joint] = []
    for suffix, dz in zip(suffixes, deck_dzs):
        base = primary.joints[0]  # every coupler had a single "O" joint
        joints.append(voffset_joint(base, dz, new_name=f"O{suffix}"))

    fused = Body(
        name=new_name,
        part=primary.part,
        joints=joints,
        color=primary.color,
        pose=primary.pose,
    )

    drop_names = {f"coupler{s}" for s in suffixes}
    bodies = []
    for b in mech.bodies:
        if b.name == primary_name:
            bodies.append(fused)
        elif b.name in drop_names:
            continue
        else:
            bodies.append(b)

    def rewrite(idx, body, joint):
        for suffix in suffixes:
            if body == f"coupler{suffix}":
                return (idx, new_name, f"{joint}{suffix}")
        return (idx, body, joint)

    return Mechanism(
        name=mech.name,
        bodies=bodies,
        connections=_rewrite_connections(mech.connections, rewrite),
    )


def fuse_torsos(
    mech: Mechanism,
    suffixes: list[str],
    *,
    a_dz: list[float] | None = None,
    b_dz: list[float] | None = None,
    servo: bool = True,
    new_name: str = "torso",
) -> Mechanism:
    """Collapse per-leg torsos into one torso body carrying every listed leg's pivots.

    Builds a fresh :class:`Body` named ``new_name`` whose joint list is the
    union of per-leg ``A``/``B`` joints (z-shifted by the optional
    ``a_dz``/``b_dz``) plus a single shared ``O`` joint from the primary leg.
    The build123d part is re-created via :func:`shapes.create_torso`.

    Any existing ``torso*`` body whose suffix is **not** in ``suffixes`` is
    dropped along with its incident connection edges — callers wire those
    orphaned legs via :func:`_add_standoffs` or similar.

    2016 analogue: the single hulled ``torso`` carrying ``[pos_O, A2, B2, A1, B1]``
    in ``DoubleKlannLinkage.__init__`` (Project/main.py:587).
    """
    n = len(suffixes)
    if a_dz is None:
        a_dz = [0.0] * n
    if b_dz is None:
        b_dz = [0.0] * n
    if len(a_dz) != n or len(b_dz) != n:
        raise ValueError("a_dz/b_dz length must match suffixes length")

    # Collect suffixed A/B joints from each listed leg's torso, plus one O.
    new_joints: list[Joint] = []
    primary = mech.body(f"torso{suffixes[0]}")
    o_joint = next(j for j in primary.joints if j.name == "O")
    new_joints.append(replace(o_joint))  # O stays unsuffixed (all legs share it)

    for suffix, dza, dzb in zip(suffixes, a_dz, b_dz):
        torso = mech.body(f"torso{suffix}")
        a_j = next(j for j in torso.joints if j.name == "A")
        b_j = next(j for j in torso.joints if j.name == "B")
        new_joints.append(voffset_joint(a_j, dza, new_name=f"A{suffix}"))
        new_joints.append(voffset_joint(b_j, dzb, new_name=f"B{suffix}"))

    # Rebuild the build123d part. If the primary torso had no part (with_parts=False),
    # skip regeneration.
    if primary.part is not None:
        from shapes import create_torso

        part = create_torso(new_joints, servo=servo)
    else:
        part = None

    fused = Body(
        name=new_name,
        part=part,
        joints=new_joints,
        color=primary.color or "yellow",
    )

    listed = {f"torso{s}" for s in suffixes}
    drop_all_torsos = {b.name for b in mech.bodies if b.name.startswith("torso")}

    bodies = []
    inserted = False
    for b in mech.bodies:
        if b.name in drop_all_torsos:
            if not inserted:
                bodies.append(fused)
                inserted = True
        else:
            bodies.append(b)

    def rewrite(idx, body, joint):
        if body in listed:
            # joint "O" stays as "O"; "A"/"B" pick up the leg suffix.
            for suffix in suffixes:
                if body == f"torso{suffix}":
                    if joint == "O":
                        return (idx, new_name, "O")
                    return (idx, new_name, f"{joint}{suffix}")
        if body in drop_all_torsos:
            return None  # unlisted torso — drop this edge entirely
        return (idx, body, joint)

    return Mechanism(
        name=mech.name,
        bodies=bodies,
        connections=_rewrite_connections(mech.connections, rewrite),
    )


def _add_standoffs(
    mech: Mechanism,
    *,
    leg_suffix: str,
    layer_thickness: float,
) -> Mechanism:
    """Ground a leg's A/B pivots via two :func:`shapes.standoff` bodies.

    2016 analogue: ``st_conn3, st_conn4`` in ``DoubleDeckerKlannLinkage``
    (Project/main.py:668). The upper-deck leg has no torso of its own; a pair
    of printed standoffs bridge its b3.A and b2.B pivots down to the torso
    plane.
    """
    from shapes import standoff

    # Allocate standoff indices unique within this mechanism.
    existing = sum(1 for b in mech.bodies if b.name.startswith("standoff"))
    st_a = standoff(layer_thickness, index=existing)
    st_b = standoff(layer_thickness, index=existing + 1)

    bodies = list(mech.bodies) + [st_a, st_b]
    connections = list(mech.connections) + [
        ((0, f"b3{leg_suffix}", "A"), (0, st_a.name, "A")),
        ((0, f"b2{leg_suffix}", "B"), (0, st_b.name, "A")),
    ]
    return Mechanism(name=mech.name, bodies=bodies, connections=connections)


# ---------------------------------------------------------------------------
# Template-layer composition primitives (parametric-in-t siblings of
# combine_connectors / fuse_couplers / fuse_torsos / _add_standoffs).
# They mirror the single-t primitives structurally; the only differences are
# ``JointTemplate`` instead of ``Joint`` (joint-pose renames wrap the
# underlying ``pose_at`` callable) and ``BodyTemplate.base_pose`` / no parts.
# ---------------------------------------------------------------------------


def _merge_templates(name: str, tmpls: list[MechanismTemplate]) -> MechanismTemplate:
    bodies: list[BodyTemplate] = []
    connections: list = []
    for m in tmpls:
        bodies.extend(m.bodies)
        connections.extend(m.connections)
    return MechanismTemplate(name=name, bodies=bodies, connections=connections)


def combine_connectors_template(
    tmpl: MechanismTemplate,
    suffix_a: str,
    suffix_b: str,
    *,
    new_name: str = "conn",
) -> MechanismTemplate:
    name_a = f"conn{suffix_a}"
    name_b = f"conn{suffix_b}"
    body_a = tmpl.body(name_a)
    body_b = tmpl.body(name_b)

    joints = [
        JointTemplate(name=f"{j.name}{suffix_a}", pose_at=j.pose_at) for j in body_a.joints
    ] + [
        JointTemplate(name=f"{j.name}{suffix_b}", pose_at=j.pose_at) for j in body_b.joints
    ]
    fused = BodyTemplate(name=new_name, joints=joints, color=body_a.color)

    bodies: list[BodyTemplate] = []
    for b in tmpl.bodies:
        if b.name == name_a:
            bodies.append(fused)
        elif b.name == name_b:
            continue
        else:
            bodies.append(b)

    def rewrite(idx, body, joint):
        if body == name_a:
            return (idx, new_name, f"{joint}{suffix_a}")
        if body == name_b:
            return (idx, new_name, f"{joint}{suffix_b}")
        return (idx, body, joint)

    return MechanismTemplate(
        name=tmpl.name,
        bodies=bodies,
        connections=_rewrite_connections(tmpl.connections, rewrite),
    )


def fuse_couplers_template(
    tmpl: MechanismTemplate,
    suffixes: list[str],
    *,
    deck_dzs: list[float] | None = None,
    new_name: str = "coupler",
) -> MechanismTemplate:
    if deck_dzs is None:
        deck_dzs = [0.0] * len(suffixes)
    if len(deck_dzs) != len(suffixes):
        raise ValueError("deck_dzs length must match suffixes length")

    primary_name = f"coupler{suffixes[0]}"
    primary = tmpl.body(primary_name)

    joints: list[JointTemplate] = []
    base = primary.joints[0]                    # every coupler has a single "O" joint
    for suffix, dz in zip(suffixes, deck_dzs):
        joints.append(
            JointTemplate(
                name=f"O{suffix}",
                pose_at=base.pose_at if dz == 0.0 else offset_pose_at(base.pose_at, dz=dz),
            )
        )

    fused = BodyTemplate(
        name=new_name,
        joints=joints,
        color=primary.color,
        base_pose=primary.base_pose,
    )

    drop_names = {f"coupler{s}" for s in suffixes}
    bodies: list[BodyTemplate] = []
    for b in tmpl.bodies:
        if b.name == primary_name:
            bodies.append(fused)
        elif b.name in drop_names:
            continue
        else:
            bodies.append(b)

    def rewrite(idx, body, joint):
        for suffix in suffixes:
            if body == f"coupler{suffix}":
                return (idx, new_name, f"{joint}{suffix}")
        return (idx, body, joint)

    return MechanismTemplate(
        name=tmpl.name,
        bodies=bodies,
        connections=_rewrite_connections(tmpl.connections, rewrite),
    )


def fuse_torsos_template(
    tmpl: MechanismTemplate,
    suffixes: list[str],
    *,
    a_dz: list[float] | None = None,
    b_dz: list[float] | None = None,
    new_name: str = "torso",
) -> MechanismTemplate:
    n = len(suffixes)
    if a_dz is None:
        a_dz = [0.0] * n
    if b_dz is None:
        b_dz = [0.0] * n
    if len(a_dz) != n or len(b_dz) != n:
        raise ValueError("a_dz/b_dz length must match suffixes length")

    new_joints: list[JointTemplate] = []
    primary = tmpl.body(f"torso{suffixes[0]}")
    o_joint = next(j for j in primary.joints if j.name == "O")
    new_joints.append(JointTemplate(name="O", pose_at=o_joint.pose_at))

    for suffix, dza, dzb in zip(suffixes, a_dz, b_dz):
        torso = tmpl.body(f"torso{suffix}")
        a_j = next(j for j in torso.joints if j.name == "A")
        b_j = next(j for j in torso.joints if j.name == "B")
        new_joints.append(
            JointTemplate(
                name=f"A{suffix}",
                pose_at=a_j.pose_at if dza == 0.0 else offset_pose_at(a_j.pose_at, dz=dza),
            )
        )
        new_joints.append(
            JointTemplate(
                name=f"B{suffix}",
                pose_at=b_j.pose_at if dzb == 0.0 else offset_pose_at(b_j.pose_at, dz=dzb),
            )
        )

    fused = BodyTemplate(
        name=new_name,
        joints=new_joints,
        color=primary.color or "yellow",
    )

    listed = {f"torso{s}" for s in suffixes}
    drop_all_torsos = {b.name for b in tmpl.bodies if b.name.startswith("torso")}

    bodies: list[BodyTemplate] = []
    inserted = False
    for b in tmpl.bodies:
        if b.name in drop_all_torsos:
            if not inserted:
                bodies.append(fused)
                inserted = True
        else:
            bodies.append(b)

    def rewrite(idx, body, joint):
        if body in listed:
            for suffix in suffixes:
                if body == f"torso{suffix}":
                    if joint == "O":
                        return (idx, new_name, "O")
                    return (idx, new_name, f"{joint}{suffix}")
        if body in drop_all_torsos:
            return None
        return (idx, body, joint)

    return MechanismTemplate(
        name=tmpl.name,
        bodies=bodies,
        connections=_rewrite_connections(tmpl.connections, rewrite),
    )


def _add_standoffs_template(
    tmpl: MechanismTemplate,
    *,
    leg_suffix: str,
    layer_thickness: float,
) -> MechanismTemplate:
    """Template-layer sibling of :func:`_add_standoffs`.

    Standoffs are grounding bodies whose anchor is at the body origin; the
    joint pose is time-invariant :class:`Pose.identity`. Built as a constant
    ``pose_at`` closure that ignores ``ts`` and returns a broadcast identity.
    """
    del layer_thickness  # standoff part geometry isn't needed in the template
    existing = sum(1 for b in tmpl.bodies if b.name.startswith("standoff"))

    def _identity_pose_at(ts: np.ndarray) -> np.ndarray:
        ts = np.asarray(ts, dtype=float)
        return np.broadcast_to(np.eye(4), (ts.shape[0], 4, 4)).copy()

    st_a = BodyTemplate(
        name=f"standoff{existing}",
        joints=[JointTemplate(name="A", pose_at=_identity_pose_at)],
        color="yellow",
    )
    st_b = BodyTemplate(
        name=f"standoff{existing + 1}",
        joints=[JointTemplate(name="A", pose_at=_identity_pose_at)],
        color="yellow",
    )

    bodies = list(tmpl.bodies) + [st_a, st_b]
    connections = list(tmpl.connections) + [
        ((0, f"b3{leg_suffix}", "A"), (0, st_a.name, "A")),
        ((0, f"b2{leg_suffix}", "B"), (0, st_b.name, "A")),
    ]
    return MechanismTemplate(name=tmpl.name, bodies=bodies, connections=connections)


# ---------------------------------------------------------------------------
# Public multi-linkage builders
# ---------------------------------------------------------------------------


def build_double_klann(
    t: float,
    *,
    thickness: float | None = None,
    with_parts: bool = True,
) -> Mechanism:
    """Mirrored pair sharing one crank and one torso.

    2016 analogue: ``DoubleKlannLinkage`` (Project/main.py:519). Two legs of
    opposite chirality at the same phase, their O→M crank arms fused into a
    single rigid body driven by one shaft. A single torso holds both legs'
    A/B pivots, staggered in Z by a 6 mm layer stride.
    """
    from shapes import THICKNESS

    th = THICKNESS if thickness is None else thickness
    sol_r = create_klann_geometry(orientation=+1, phase=0.0)
    sol_l = create_klann_geometry(orientation=-1, phase=0.0)
    leg_r = build_klann_mechanism(sol_r, t, thickness=th, name_suffix="_leg0", with_parts=with_parts)
    leg_l = build_klann_mechanism(sol_l, t, thickness=th, name_suffix="_leg1", with_parts=with_parts)
    mech = _merge_mechanisms("klann_double", [leg_r, leg_l])
    mech = combine_connectors(mech, "_leg0", "_leg1")
    mech = fuse_couplers(mech, ["_leg0", "_leg1"])
    mech = fuse_torsos(mech, ["_leg0", "_leg1"], a_dz=(0.0, -6.0), b_dz=(0.0, -6.0))
    return mech


def build_double_decker_klann(
    t: float,
    *,
    thickness: float | None = None,
    z_deck: float = 15.0,
    with_parts: bool = True,
) -> Mechanism:
    """Two legs of the same chirality stacked in Z at 90° phase offset.

    2016 analogue: ``DoubleDeckerKlannLinkage`` (Project/main.py:633). The
    lower leg grows the torso + servo mount; the upper leg is grounded via
    printed standoffs bridging to the torso plane. A single coupler carries
    one ``O`` joint per deck so a single shaft spans both levels.
    """
    from shapes import THICKNESS

    th = THICKNESS if thickness is None else thickness
    sol0 = create_klann_geometry(orientation=+1, phase=0.0)
    sol1 = create_klann_geometry(orientation=+1, phase=math.pi / 2)
    leg0 = build_klann_mechanism(
        sol0, t, thickness=th, z_base=0.0, name_suffix="_leg0", with_parts=with_parts
    )
    leg1 = build_klann_mechanism(
        sol1, t, thickness=th, z_base=z_deck, name_suffix="_leg1", with_parts=with_parts
    )
    mech = _merge_mechanisms("klann_decker", [leg0, leg1])
    mech = fuse_couplers(mech, ["_leg0", "_leg1"], deck_dzs=[0.0, z_deck])
    mech = fuse_torsos(mech, ["_leg0"])
    mech = _add_standoffs(mech, leg_suffix="_leg1", layer_thickness=z_deck)
    return mech


def build_double_double_decker_klann(
    t: float,
    *,
    thickness: float | None = None,
    z_deck: float = 15.0,
    with_parts: bool = True,
) -> Mechanism:
    """Three-leg walker: mirrored lower pair with shared crank + upper leg at 90° phase.

    2016 analogue: ``DoubleDoubleDeckerKlannLinkage`` (Project/main.py:835).
    The lower deck is a mirrored pair (``+1`` and ``-1`` orientations, phase
    π apart) sharing a single combined crank. The upper deck is one leg at
    90° phase, grounded via standoffs. One shared coupler spans both decks;
    one shared torso carries both lower legs' pivots.
    """
    from shapes import THICKNESS

    th = THICKNESS if thickness is None else thickness
    sol0 = create_klann_geometry(orientation=+1, phase=0.0)
    sol1 = create_klann_geometry(orientation=-1, phase=math.pi)
    sol2 = create_klann_geometry(orientation=+1, phase=math.pi / 2)
    leg0 = build_klann_mechanism(
        sol0, t, thickness=th, z_base=0.0, name_suffix="_leg0", with_parts=with_parts
    )
    leg1 = build_klann_mechanism(
        sol1, t, thickness=th, z_base=0.0, name_suffix="_leg1", with_parts=with_parts
    )
    leg2 = build_klann_mechanism(
        sol2, t, thickness=th, z_base=z_deck, name_suffix="_leg2", with_parts=with_parts
    )
    mech = _merge_mechanisms("klann_quad", [leg0, leg1, leg2])
    mech = combine_connectors(mech, "_leg0", "_leg1")
    mech = fuse_couplers(mech, ["_leg0", "_leg1", "_leg2"], deck_dzs=[0.0, 0.0, z_deck])
    mech = fuse_torsos(mech, ["_leg0", "_leg1"], a_dz=(0.0, -6.0), b_dz=(0.0, -6.0))
    mech = _add_standoffs(mech, leg_suffix="_leg2", layer_thickness=z_deck)
    return mech


def build_double_template(
    *, thickness: float | None = None,
) -> MechanismTemplate:
    """Parametric-in-t sibling of :func:`build_double_klann`."""
    from shapes import THICKNESS

    th = THICKNESS if thickness is None else thickness
    sol_r = create_klann_geometry(orientation=+1, phase=0.0)
    sol_l = create_klann_geometry(orientation=-1, phase=0.0)
    leg_r = build_klann_template(sol_r, thickness=th, name_suffix="_leg0")
    leg_l = build_klann_template(sol_l, thickness=th, name_suffix="_leg1")
    tmpl = _merge_templates("klann_double", [leg_r, leg_l])
    tmpl = combine_connectors_template(tmpl, "_leg0", "_leg1")
    tmpl = fuse_couplers_template(tmpl, ["_leg0", "_leg1"])
    tmpl = fuse_torsos_template(
        tmpl, ["_leg0", "_leg1"], a_dz=[0.0, -6.0], b_dz=[0.0, -6.0]
    )
    return tmpl


def build_double_decker_template(
    *, thickness: float | None = None, z_deck: float = 15.0,
) -> MechanismTemplate:
    """Parametric-in-t sibling of :func:`build_double_decker_klann`."""
    from shapes import THICKNESS

    th = THICKNESS if thickness is None else thickness
    sol0 = create_klann_geometry(orientation=+1, phase=0.0)
    sol1 = create_klann_geometry(orientation=+1, phase=math.pi / 2)
    leg0 = build_klann_template(sol0, thickness=th, z_base=0.0, name_suffix="_leg0")
    leg1 = build_klann_template(sol1, thickness=th, z_base=z_deck, name_suffix="_leg1")
    tmpl = _merge_templates("klann_decker", [leg0, leg1])
    tmpl = fuse_couplers_template(tmpl, ["_leg0", "_leg1"], deck_dzs=[0.0, z_deck])
    tmpl = fuse_torsos_template(tmpl, ["_leg0"])
    tmpl = _add_standoffs_template(tmpl, leg_suffix="_leg1", layer_thickness=z_deck)
    return tmpl


def build_double_double_decker_template(
    *, thickness: float | None = None, z_deck: float = 15.0,
) -> MechanismTemplate:
    """Parametric-in-t sibling of :func:`build_double_double_decker_klann`."""
    from shapes import THICKNESS

    th = THICKNESS if thickness is None else thickness
    sol0 = create_klann_geometry(orientation=+1, phase=0.0)
    sol1 = create_klann_geometry(orientation=-1, phase=math.pi)
    sol2 = create_klann_geometry(orientation=+1, phase=math.pi / 2)
    leg0 = build_klann_template(sol0, thickness=th, z_base=0.0, name_suffix="_leg0")
    leg1 = build_klann_template(sol1, thickness=th, z_base=0.0, name_suffix="_leg1")
    leg2 = build_klann_template(sol2, thickness=th, z_base=z_deck, name_suffix="_leg2")
    tmpl = _merge_templates("klann_quad", [leg0, leg1, leg2])
    tmpl = combine_connectors_template(tmpl, "_leg0", "_leg1")
    tmpl = fuse_couplers_template(
        tmpl, ["_leg0", "_leg1", "_leg2"], deck_dzs=[0.0, 0.0, z_deck]
    )
    tmpl = fuse_torsos_template(
        tmpl, ["_leg0", "_leg1"], a_dz=[0.0, -6.0], b_dz=[0.0, -6.0]
    )
    tmpl = _add_standoffs_template(tmpl, leg_suffix="_leg2", layer_thickness=z_deck)
    return tmpl


def KlannLinkage(name: str = "klann") -> Mechanism:
    """Assemble a single-leg Klann mechanism at the reference time ``t=1.0``.

    Lays out the 5 laser-cut links (``b1``..``b4`` plus ``conn``), a simple
    torso plate carrying the servo-mount pivots, and a shaft coupler. The
    returned :class:`Mechanism` is unsolved: call ``.solved()`` to propagate
    world poses.
    """
    from shapes import THICKNESS

    sol = create_klann_geometry(orientation=1, phase=0.0)
    mech = build_klann_mechanism(sol, t=1.0, thickness=THICKNESS, with_parts=True)
    mech.name = name
    return mech
