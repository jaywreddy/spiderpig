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
from dataclasses import dataclass
from functools import cached_property
from typing import Any

import numpy as np
import sympy as sp
from sympy.geometry import Circle, Point, Segment

from mechanism import Body, Joint, Mechanism, Pose


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
        return {
            name: sp.lambdify((self.t,), (pt.x, pt.y), modules="numpy", cse=True)
            for name, pt in self.points.items()
        }

    def joints_at(self, t_value: float) -> dict[str, tuple[float, float]]:
        """Evaluate every named point at ``t_value`` and return float (x, y) pairs."""
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

    def _nm(base: str) -> str:
        return f"{base}{name_suffix}"

    if with_parts:
        from shapes import create_shaft_connector, create_torso, rationalize_segment

        def _seg(p: str, q: str) -> Segment:
            return Segment(Point(*xy[p]), Point(*xy[q]))

        b1_body = rationalize_segment(_seg("M", "D"), [neg_M, neg_C, neg_D], _nm("b1"))
        b2_body = rationalize_segment(_seg("B", "E"), [neg_B, neg_E], _nm("b2"))
        b3_body = rationalize_segment(_seg("A", "C"), [neg_A, pos_C], _nm("b3"))
        b4_body = rationalize_segment(_seg("E", "F"), [pos_E, pos_D], _nm("b4"))
        conn_body = rationalize_segment(_seg("O", "M"), [neg_O, pos_M], _nm("conn"))
        torso_part = create_torso([pos_A, pos_O, bt_pos])
        coupler_part = create_shaft_connector()
    else:
        b1_body = Body(name=_nm("b1"), joints=[neg_M, neg_C, neg_D], color="green")
        b2_body = Body(name=_nm("b2"), joints=[neg_B, neg_E], color="green")
        b3_body = Body(name=_nm("b3"), joints=[neg_A, pos_C], color="green")
        b4_body = Body(name=_nm("b4"), joints=[pos_E, pos_D], color="green")
        conn_body = Body(name=_nm("conn"), joints=[neg_O, pos_M], color="green")
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
