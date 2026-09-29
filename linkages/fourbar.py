"""The 4-bar walking linkage (diywalkers) and the LEGO Spot Micro legs built from it.

Numbers are the page's Scratch simulator units (``unit`` mm each; the LEGO
builds are 1/10 scale, one LEGO hole per 10 units); the params ``b1``..``b4``
and ``ang`` are its sliders, i.e. the bar map's B1..B4 and "B4's angle". The
crank B1 turns about the crank centre ``O``; the frame's hip joint ``H``
sits at (``-hip_x``, ``hip_y``) from it (the "hip joint X" slider is
negative: the leg reaches out to the left). The coupler B2 runs from the
crankpin ``M`` to the knee ``K``, where the rocker B3 hangs from the hip.
B4, rigid with B2 (the plan's blue triangle: one bar), runs from the knee to
the foot ``F`` at ``ang`` degrees counter-clockwise from B2's extension past
the knee. Link bodies: ``b1`` = B2 + B4, ``b2`` = B3.

The page's linkage (its GIF, and "Spot Micro Ver 2 with longer legs") is
40 / 80 / 50 / 100 at 97 degrees, hip (-65, 47). LEGO Spot Micro Ver 1 is
30 / 60 / 40 / 70 at 90 degrees, hip (-50, 33); Ver 2 shortens B4 to 90,
which puts it at 99 degrees (the page's own equivalent of the 6-bar
simulator's B5 = 0, B6 = 100, B7 = 20: 99.06 degrees).
"""

from __future__ import annotations

import sympy as sp

from linkage import Linkage, P, circle_x_circle, crank, offset, register, xy

R = sp.Rational
SOURCE = "https://www.diywalkers.com/4-bar-walking-linkage.html"

PARAMS = {
    "unit": R(3, 5),      # mm per simulator unit (crank 24 mm)
    "hip_x": R(65),       # hip joint H at (-hip_x, hip_y) from the crank centre
    "hip_y": R(47),
    "b1": R(40),          # B1, crank O - M
    "b2": R(80),          # B2, coupler M - K
    "b3": R(50),          # B3, rocker H - K
    "b4": R(100),         # B4, leg K - F
    "ang": R(97),         # B4's angle: degrees CCW from B2's extension past K
}


def program(p):
    u = p["unit"]
    a = p["ang"] * sp.pi / 180
    return [
        ("O", xy(0, 0)),
        ("H", xy(-p["hip_x"] * u, p["hip_y"] * u)),
        ("M", crank(p["b1"] * u)),
        ("K", circle_x_circle(P("M"), p["b2"] * u, P("H"), p["b3"] * u, 1)),
        ("F", offset(P("M"), P("K"), (p["b2"] + p["b4"] * sp.cos(a)) * u,
                     p["b4"] * sp.sin(a) * u)),
    ]


FOURBAR = register(Linkage(
    key="fourbar",
    name="4-bar walking linkage",
    family="fourbar",
    params=PARAMS,
    angles=frozenset({"ang"}),
    program=program,
    links={
        "b1": (("M", "K", "F"), (("M", "K"), ("K", "F"))),
        "b2": (("H", "K"), (("H", "K"),)),
    },
    labels={"b1": "coupler B2 + leg B4 (M-K-F)", "b2": "rocker B3 (H-K)"},
    frame=("H", "O"),
    crank=("O", "M"),
    feet=(("b1", "F"),),
    source=SOURCE,
    notes="Four bars, one frame pivot; a long, low D-shaped foot path. "
          "The page's GIF (Spot Micro Ver 2 with longer legs).",
))

register(FOURBAR.variant(
    "fourbar_spot_micro", "LEGO Spot Micro Ver 1 (4-bar)",
    unit=R(4, 5), hip_x=R(50), hip_y=R(33), b1=R(30), b2=R(60), b3=R(40), b4=R(70), ang=R(90),
    notes="The LEGO Spot Micro's first legs: a smaller 4-bar with B4 square to B2 "
          "(at 0.8 mm per unit this is the LEGO build's own scale).",
))

register(FOURBAR.variant(
    "fourbar_spot_micro_v2", "LEGO Spot Micro Ver 2 (4-bar)",
    b4=R(90), ang=R(99),
    notes="The page's linkage with legs shortened to 90 so front and rear feet don't collide "
          "with the crank at 12 o'clock.",
))
