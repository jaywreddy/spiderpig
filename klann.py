"""Symbolic Klann walking-linkage geometry.

The Klann linkage is a 6-bar planar mechanism that converts continuous
rotation of a crank into a foot path well suited to walking robots. This
module keeps the geometry purely symbolic via sympy so the linkage
parameters remain easy to read and modify. Downstream code (shapes.py,
layout.py) evaluates the returned points numerically.
"""

from __future__ import annotations

import numpy as np
from sympy.geometry import Circle, Point, Segment


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
