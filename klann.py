"""Symbolic Klann walking-linkage geometry and mechanism assembly.

The Klann linkage is a 6-bar planar mechanism that converts continuous
rotation of a crank into a foot path well suited to walking robots. This
module keeps the geometry purely symbolic via sympy so the linkage
parameters remain easy to read and modify. Downstream code (shapes.py,
layout.py) evaluates the returned points numerically.
"""

from __future__ import annotations

import numpy as np
from sympy.geometry import Circle, Point, Segment

from mechanism import Body, Mechanism


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

    dx, dy = (c2.center - c1.center).args
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


def create_klann_geometry(orientation=1, phase=1):
    """Solve the Klann linkage at a given crank ``phase``.

    Parameters match the reference mechanism (Joseph C. Klann's published
    proportions). ``orientation=+1`` yields a right-hand linkage; ``-1`` a
    left-hand mirror. ``phase`` advances the crank; a full foot cycle
    corresponds to ``phase`` sweeping 0..2 pi.

    Returns a list ``[O, A, B, C, D, E, F, M, b1, b2, b3, b4, conn]`` where
    the points are the pivots and foot tip and the :class:`Segment` objects
    describe the links connecting them.
    """
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
    st = {M.free_symbols.pop(): phase}
    M = M.evalf(subs=st)
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

    return [O, A, B, C, D, E, F, M, b1, b2, b3, b4, conn]


_CONNECTIONS = [
    ((0, "torso", "A"), (0, "b3", "A")),
    ((0, "torso", "B"), (0, "b2", "B")),
    ((0, "torso", "O"), (0, "conn", "O")),
    ((0, "conn", "M"), (0, "b1", "M")),
    ((0, "b1", "C"), (0, "b3", "C")),
    ((0, "b1", "D"), (0, "b4", "D")),
    ((0, "b2", "E"), (0, "b4", "E")),
    ((0, "conn", "O"), (0, "coupler", "O")),
]


def _build_mechanism(
    name: str,
    O,
    A,
    B,
    C,
    D,
    E,
    _F,
    M,
    b1,
    b2,
    b3,
    b4,
    conn,
    *,
    with_parts: bool,
) -> Mechanism:
    """Wire the seven bodies given the per-phase symbolic points and segments.

    ``with_parts=False`` skips build123d part construction — useful for the
    viewer bake loop where we only need joint poses for ``Mechanism.solved()``.
    """
    from shapes import joint_from_point  # lazy: avoid build123d when unused

    pos_A, neg_A = joint_from_point(A, "A", 1)
    pos_B, neg_B = joint_from_point(B, "B", 1)
    pos_C, neg_C = joint_from_point(C, "C", 1)
    pos_D, neg_D = joint_from_point(D, "D", 1)
    pos_E, neg_E = joint_from_point(E, "E", 1)
    pos_M, neg_M = joint_from_point(M, "M", 1)
    pos_O, neg_O = joint_from_point(O, "O", 1)
    bt_pos, _ = joint_from_point(B, "B", 2)

    if with_parts:
        from shapes import create_shaft_connector, create_torso, rationalize_segment

        b1_body = rationalize_segment(b1, [neg_M, neg_C, neg_D], "b1")
        b2_body = rationalize_segment(b2, [neg_B, neg_E], "b2")
        b3_body = rationalize_segment(b3, [neg_A, pos_C], "b3")
        b4_body = rationalize_segment(b4, [pos_E, pos_D], "b4")
        conn_body = rationalize_segment(conn, [neg_O, pos_M], "conn")
        torso_part = create_torso([pos_A, pos_O, bt_pos])
        coupler_part = create_shaft_connector()
    else:
        b1_body = Body(name="b1", joints=[neg_M, neg_C, neg_D], color="green")
        b2_body = Body(name="b2", joints=[neg_B, neg_E], color="green")
        b3_body = Body(name="b3", joints=[neg_A, pos_C], color="green")
        b4_body = Body(name="b4", joints=[pos_E, pos_D], color="green")
        conn_body = Body(name="conn", joints=[neg_O, pos_M], color="green")
        torso_part = None
        coupler_part = None

    torso = Body(
        name="torso",
        part=torso_part,
        joints=[pos_A, pos_O, bt_pos],
        color="yellow",
    )
    coupler = Body(
        name="coupler",
        part=coupler_part,
        joints=[pos_O],
        color="blue",
    )

    return Mechanism(
        name=name,
        bodies=[coupler, b1_body, b2_body, b3_body, b4_body, conn_body, torso],
        connections=list(_CONNECTIONS),
    )


def solve_klann_at_phase(
    phase: float,
    *,
    name: str = "klann",
    with_parts: bool = False,
) -> Mechanism:
    """Build a Klann mechanism with joints placed at the given crank ``phase``.

    Bodies are wired identically to :func:`KlannLinkage` — only the joint XY
    positions differ per phase. Joint Z layering is phase-invariant (pins
    don't migrate in Z between layers). Returns an unsolved
    :class:`Mechanism`; call ``.solved()`` to propagate world poses.

    ``with_parts=False`` (default) skips build123d part construction for
    speed when only poses are needed (viewer bake loop).
    """
    geom = create_klann_geometry(orientation=1, phase=phase)
    return _build_mechanism(name, *geom, with_parts=with_parts)


def KlannLinkage(name: str = "klann") -> Mechanism:
    """Assemble the full Klann single-leg mechanism at the reference phase.

    Lays out the 5 laser-cut links (``b1``..``b4`` plus ``conn``), a simple
    torso plate carrying the servo-mount pivots, and a shaft coupler. The
    returned :class:`Mechanism` is unsolved: call ``.solved()`` to propagate
    world poses.
    """
    return solve_klann_at_phase(phase=1.0, name=name, with_parts=True)
