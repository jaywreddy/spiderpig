"""Decanting cradles: a crank tips a wine bottle about its own mouth.

A decanting cradle pours a large-format bottle (a 1.5 L magnum to a 3 L double magnum)
slowly enough that the sediment stays in the bottle. These two hold the bottle's mouth
still while it tips, so the decanter can stand under it for the whole pour (the
prior art and why: ``docs/agentlib/DECANTER.md``). One turn of the crank is one decant:
the cradle swings from rest (the bottle 50° neck up: an opened full bottle goes in) to
full tilt (30° neck down: both bottles' shoulders over the mouth, so they drain) and
back, and the crank-rocker's toggles at both ends are soft stops: cranking on brings
the bottle back, it can't tip further.

The cradle body is the output, a ``rotation`` that promises its mouth point ``L`` stays
put (:attr:`linkage.Output.still`). Its frame is ``(L, X)``: the bottle's axis runs from
the mouth through ``X`` at ``tilt_rest`` (the angle ``axis`` below fixes which way).
Lengths are mm, coordinates relative to the crank centre O (y up).

* ``decanter_hinge`` (A): the cradle is the crank-rocker's rocker, hinged on pins in line
  with the mouth (G = L).
* ``decanter_rcm`` (B): a double parallelogram, the remote centre of motion of surgical
  arms (Taylor et al., the da Vinci's), makes the cradle turn about the mouth with
  nothing there; a crank-rocker drives its rear arm.
"""

from __future__ import annotations

import sympy as sp

from spiderpig.linkage import (
    Linkage,
    Output,
    P,
    circle_x_circle,
    crank,
    extend,
    offset,
    register,
    xy,
)

R = sp.Rational
SOURCE = "https://patents.google.com/patent/US7708241"   # the mouth on the tilt axis

TIES = {"Tf1": (55, -40), "Tf2": (75, -40), "Tr1": (270, -112), "Tr2": (300, -112)}
"""Where the cradle box's tie rods ride on the cradle link, in the cradle's frame (mm: the
mouth at the origin, x along the bottle's axis toward its base, y up with the bottle level):
two under the neck carry the front saddle (the neck's V and the shoulder's stop), two under
the body the rear saddle (the body's V, deep enough for a 3 L bottle, 141 mm). Each link
point sweeps an arc about the mouth: :mod:`construction.cradle` keeps the frame clear of
them."""


def _ties(at):
    """The tie points as steps: ``at(cx, cy)`` places a cradle-frame point."""
    return [(name, at(cx, cy)) for name, (cx, cy) in TIES.items()]


def _two(a, b):
    return (a, b), ((a, b),)


# --- A: a hinge at the mouth ------------------------------------------------------------


def decanter_hinge(p):
    """A crank-rocker whose rocker is the cradle, hinged at the bottle's mouth G. The rod
    pin E sits on the bottle's axis, ``rocker`` behind the mouth; ``crank`` and ``rod``
    are the crank-rocker's for an 80° swing (−50° to +30°) from O, 285 mm under the mouth
    and 85 behind it: the crank torque in the gearbox's rating (SF 1.3), the jam's rod force
    no more than the old −25° to +55° design's, the rod short enough not to buckle under it
    (SF 1.5), transmission 38°."""
    return [
        ("O", xy(0, 0)),
        ("G", xy(p["gx"], p["gy"])),
        ("M", crank(p["crank"])),
        ("E", circle_x_circle(P("M"), p["rod"], P("G"), p["rocker"], -1)),
        # the cradle's frame is G -> E (E on the bottle's axis)
        *_ties(lambda cx, cy: offset(P("G"), P("E"), cx, cy)),
    ]


register(Linkage(
    key="decanter_hinge", name="Decanting cradle A: hinge at the mouth",
    program=decanter_hinge,
    params={"crank": R(8951, 100), "rod": R(26734, 100), "rocker": R(140), "gx": R(-85),
            "gy": R(285)},
    links={"b1": _two("M", "E"),
           "b2": (("G", "E", *TIES), (("G", "E"), ("G", "Tf1"), ("Tf1", "Tf2"), ("Tf2", "E"),
                                     ("Tf2", "Tr1"), ("E", "Tr1"), ("Tr1", "Tr2")))},
    labels={"b1": "connecting rod", "b2": "cradle (hinged at the mouth G; E on the bottle's axis)"},
    frame=("O", "G"), crank=("O", "M"),
    output=Output("body", "b2", "rotation", ("G", "E"), still=("G", 1e-9)),
    source=SOURCE, product="cradle",
    notes="Four bars; the cradle tips the bottle 80° about its mouth once per turn.",
))


# --- B: a virtual hinge (a double parallelogram) --------------------------------------


def decanter_rcm(p):
    """A double-parallelogram remote centre of motion about the mouth L.

    Two ground pivots O1, O2 lie on a ray from L (``r1``, ``r2`` along ``ray`` degrees).
    The driven arm b2 (O2-K, with the rod's pin P on its bell crank) and the copy of it b3
    (O1-J) stay parallel through the bar b4 (X-J-K), which therefore translates; the
    cradle b6 (X-Y) stays parallel to b3 through b5 (Y-J2, parallel to b4). So the cradle
    turns with the arms and its point at -(O2 - L) from K, the mouth, stands still. The
    crank-rocker O-M-P drives b2."""
    ray = p["ray"] * sp.pi / 180
    lx, ly = p["lx"], p["ly"]
    u = sp.Matrix([sp.cos(ray), sp.sin(ray)])
    o1 = sp.Matrix([lx, ly]) + p["r1"] * u
    o2 = sp.Matrix([lx, ly]) + p["r2"] * u
    bell = p["bell"] * sp.pi / 180
    return [
        ("O", xy(0, 0)),
        ("L", xy(lx, ly)),
        ("O1", o1),
        ("O2", o2),
        ("M", crank(p["crank"])),
        ("P", circle_x_circle(P("M"), p["rod"], P("O2"), p["lever"], -1)),
        # K: the arm's end, ``arm`` from O2 at -bell from the lever O2-P
        ("K", offset(P("O2"), P("P"), p["arm"] * sp.cos(bell), -p["arm"] * sp.sin(bell))),
        # J: the copy arm's end (the parallelogram O1-J-K-O2)
        ("J", circle_x_circle(P("O1"), p["arm"], P("K"), p["r2"] - p["r1"], 1)),
        # X: the cradle's pin on the translating bar, r1 beyond J (the bar is parallel to L-O2)
        ("X", extend(P("K"), P("J"), p["r1"])),
        # J2 on the copy arm, Y on the cradle: the parallelogram X-J-J2-Y
        ("J2", offset(P("O1"), P("J"), p["y_along"], p["y_across"])),
        ("Y", circle_x_circle(P("J2"), p["r1"], P("X"), _xy(p), 1)),
        # the mouth, a point of the cradle: -X0 from X in the arm's frame, X0 = (arm, 0),
        # Y0 = (y_along, y_across), written in the cradle's frame X -> Y; Ax on the bottle's
        # axis, ``skew`` from the arm's
        ("Lc", _on_cradle(p, 0, 0)),
        ("Ax", _on_cradle(p, 100, 0)),
        *_ties(lambda cx, cy: _on_cradle(p, cx, cy)),
    ]


def _on_cradle(p, cx, cy):
    """The cradle's point at ``(cx, cy)`` in the bottle's frame (the mouth at the origin, the
    bottle's axis along x). The arm's frame is that turned by ``skew`` (the arms swing that
    much ahead of the bottle: their parallelograms stay clear of folding flat on the
    pivots' ray over an 80° swing); X is ``(arm, 0)`` there and Y ``(y_along, y_across)``,
    so the point is an offset from X along X -> Y."""
    k = p["skew"] * sp.pi / 180
    cx, cy = cx * sp.cos(k) + cy * sp.sin(k), -cx * sp.sin(k) + cy * sp.cos(k)
    s = _xy(p)
    ux, uy = (p["y_along"] - p["arm"]) / s, p["y_across"] / s
    dx, dy = cx - p["arm"], cy
    return offset(P("X"), P("Y"), dx * ux + dy * uy, -dx * uy + dy * ux)


def _xy(p):
    """The cradle's pin spacing |X - Y|."""
    return sp.sqrt((p["y_along"] - p["arm"]) ** 2 + p["y_across"] ** 2)


RCM = {"crank": R(935, 10), "rod": R(3362, 10), "lever": R(147), "bell": R(64),
       "arm": R(174), "ray": R(-181, 2), "r1": R(105), "r2": R(307), "lx": R(-366),
       "ly": R(322), "y_along": R(-110), "y_across": R(0), "skew": R(8)}
"""B (2026-10-10, refined to build: ``docs/agentlib/DECANTER.md``): -50° (rest, an opened
full bottle stands in it) to +30°, the crank-rocker's toggles at both. The pivots O1, O2
hang straight under the mouth (a ray 90.5° down), 105 and 307 mm from it: the bar O1-O2
lies off every tie rod's arc (the front ties run 68-85 mm from the mouth, the rear ones
292-320 and never further round than 72° down), and the cradle link reaches its ties
without sweeping over a pivot. The arms swing 8° ahead of the bottle (``skew``): both
parallelograms stay 48° or more off their fold. Y lies on the copy arm's line through O1
(``y_across`` 0): the bars b4 and b5 never meet, nor the arms b2 and b3 (26 mm apart at the
closest), so each pair shares a layer. The crank O is behind and under it all, its handle
behind every part. The rod (336 mm, 0.250 in) is as short as the gearbox's peak (equal
strokes want a long rod for the crank) and the rod's force at a jam (a long crank) allow
with both at SF 1.35 or more: the crank sits 366 mm behind the mouth, where 480 had made
the stand 827 mm long (2026-10-10, slimmed)."""

register(Linkage(
    key="decanter_rcm", name="Decanting cradle B: virtual hinge (double parallelogram)",
    program=decanter_rcm, params=RCM, angles=frozenset({"ray", "bell", "skew"}),
    links={"b1": _two("M", "P"),
           "b2": (("O2", "P", "K"), (("O2", "P"), ("O2", "K"))),
           "b3": (("O1", "J", "J2"), (("O1", "J"), ("O1", "J2"))),
           "b4": (("X", "J", "K"), (("X", "K"),)),
           "b5": _two("Y", "J2"),
           # (no chord Y-Tr1: 200 mm from the mouth it would sweep across the pivot O1)
           "b6": (("X", "Y", "Lc", "Ax", *TIES),
                  (("X", "Y"), ("Y", "Tf1"), ("Tf1", "Tf2"), ("Tf2", "X"), ("X", "Tr1"),
                   ("Tr1", "Tr2")))},
    labels={"b1": "connecting rod", "b2": "lower arm O2-K (driven at P)",
            "b3": "copy arm O1-J-J2", "b4": "translating bar X-J-K",
            "b5": "cradle link Y-J2", "b6": "cradle (turns about the mouth Lc)"},
    frame=("O", "O1", "O2"), crank=("O", "M"),
    output=Output("body", "b6", "rotation", ("Lc", "Ax"), still=("Lc", 1e-6)),
    source="https://research.buaa.edu.cn/en/publications/design-of-double-parallelogram-remote-center-of-motion-mechanisms/",
    product="cradle",
    notes="Eight bars; the cradle tips 80° about a point with nothing there.",
))
