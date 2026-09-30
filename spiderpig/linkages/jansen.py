"""Theo Jansen's Strandbeest leg: an 8-bar linkage, his eleven "holy numbers".

Lengths are Jansen's units (``unit`` mm each): the fixed pivot A sits ``a``
left of and ``l`` below the crank centre, the crank is ``m``. Two links (j,
k) ride the crankpin; a rocking triangle (b, d, e) on A and the rocker c
drive the foot triangle (g, h, i) through f. A mirrored pair (the
``double`` module) shares one crankpin, as on the beach animals.
"""

from __future__ import annotations

import sympy as sp

from spiderpig.linkage import Linkage, P, circle_x_circle, crank, register, xy

R = sp.Rational
PARAMS = {
    # mm per Jansen unit. At 1.5 link k passes the fixed pivot A 8.7 mm off, under
    # the 9 mm a necked pillar needs, so legs sharing A can't be layered.
    "unit": R(8, 5),
    "a": R(38),            # A: a left of O ...
    "l": R(78, 10),        # ... and l below it
    "m": R(15),            # crank
    "j": R(50),            # crankpin - B (upper)
    "k": R(619, 10),       # crankpin - C (lower)
    "b": R(83, 2),         # A - B      \
    "d": R(401, 10),       # A - D       } the rocking triangle
    "e": R(279, 5),        # B - D      /
    "c": R(393, 10),       # A - C (lower rocker)
    "f": R(197, 5),        # D - E
    "g": R(367, 10),       # C - E      \
    "h": R(657, 10),       # E - F       } the foot triangle
    "i": R(49),            # C - F      /
}


def program(p):
    u = p["unit"]
    return [
        ("O", xy(0, 0)),
        ("A", xy(-p["a"] * u, -p["l"] * u)),
        ("M", crank(p["m"] * u)),
        ("B", circle_x_circle(P("M"), p["j"] * u, P("A"), p["b"] * u, -1)),
        ("C", circle_x_circle(P("M"), p["k"] * u, P("A"), p["c"] * u, 1)),
        ("D", circle_x_circle(P("A"), p["d"] * u, P("B"), p["e"] * u, 1)),
        ("E", circle_x_circle(P("D"), p["f"] * u, P("C"), p["g"] * u, -1)),
        ("F", circle_x_circle(P("E"), p["h"] * u, P("C"), p["i"] * u, -1)),
    ]


JANSEN = register(Linkage(
    key="jansen",
    name="Jansen (Strandbeest)",
    family="jansen",
    params=PARAMS,
    program=program,
    links={
        "b1": (("M", "B"), (("M", "B"),)),
        "b2": (("M", "C"), (("M", "C"),)),
        "b3": (("A", "B", "D"), (("A", "B"), ("B", "D"), ("D", "A"))),
        "b4": (("A", "C"), (("A", "C"),)),
        "b5": (("D", "E"), (("D", "E"),)),
        "b6": (("C", "E", "F"), (("C", "E"), ("E", "F"), ("F", "C"))),
    },
    labels={"b1": "upper crank link j", "b2": "lower crank link k",
            "b3": "rocking triangle b-d-e", "b4": "lower rocker c",
            "b5": "link f", "b6": "foot triangle g-h-i"},
    frame=("A", "O"),
    crank=("O", "M"),
    feet=(("b6", "F"),),
    source="https://www.diywalkers.com/strandbeest.html",
    notes="Eight bars, one fixed pivot; a flat, smooth stance and a low lift.",
))
