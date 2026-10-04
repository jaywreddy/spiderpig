"""The Strider linkage (Wade and Ben Vagle): a coupled pair of legs on one crankpin.

Numbers are the plan's drawing units (``unit`` mm each), read off the
diywalkers bar map: crank 4, frame pivots 22 apart and 8 above the crank
centre, rockers 5, bars 14 extended 6 past the crankpin, a 1-unit toe at 90°
to that extension, shins 13 and feet 10.

The two legs are coupled: the bar from the left rocker runs straight through
the crankpin and carries the *right* foot link, and vice versa. So one Strider
"leg" here is the whole mirror-symmetric pair, with two feet.
"""

from __future__ import annotations

import math

import sympy as sp

from spiderpig.linkage import (
    Linkage,
    Module,
    P,
    circle_x_circle,
    crank,
    extend,
    offset,
    register,
    xy,
)

PARAMS = {
    "unit": sp.Rational(13, 2),     # mm per drawing unit (6.5: the foot links clear the crank)
    "crank": sp.Integer(4),
    "half_span": sp.Integer(11),    # frame pivots at (±half_span, rise)
    "rise": sp.Integer(8),
    "rocker": sp.Integer(5),
    "bar": sp.Integer(14),
    "tail": sp.Integer(6),          # bar extension past the crankpin
    "toe": sp.Integer(1),           # offset at 90° to the extension
    "shin": sp.Integer(13),
    "foot": sp.Integer(10),
}


def program(p):
    u = p["unit"]
    return [
        ("O", xy(0, 0)),
        ("J2", xy(-p["half_span"] * u, p["rise"] * u)),
        ("J6", xy(p["half_span"] * u, p["rise"] * u)),
        ("J1", crank(p["crank"] * u)),
        ("J3", circle_x_circle(P("J2"), p["rocker"] * u, P("J1"), p["bar"] * u, -1)),
        ("J7", circle_x_circle(P("J6"), p["rocker"] * u, P("J1"), p["bar"] * u, 1)),
        ("J5", extend(P("J7"), P("J1"), p["tail"] * u)),
        ("J9", extend(P("J3"), P("J1"), p["tail"] * u)),
        ("J11", offset(P("J7"), P("J1"), (p["bar"] + p["tail"]) * u, -p["toe"] * u)),
        ("J10", offset(P("J3"), P("J1"), (p["bar"] + p["tail"]) * u, p["toe"] * u)),
        ("J4", circle_x_circle(P("J3"), p["shin"] * u, P("J11"), p["foot"] * u, -1)),
        ("J8", circle_x_circle(P("J7"), p["shin"] * u, P("J10"), p["foot"] * u, 1)),
    ]


STRIDER = register(Linkage(
    key="strider",
    name="Strider (coupled pair)",
    family="strider",
    params=PARAMS,
    program=program,
    links={
        "b1": (("J2", "J3"), (("J2", "J3"),)),
        "b2": (("J3", "J1", "J9", "J10"), (("J3", "J9"), ("J9", "J10"))),
        "b3": (("J3", "J4"), (("J3", "J4"),)),
        "b4": (("J11", "J4"), (("J11", "J4"),)),
        "b5": (("J6", "J7"), (("J6", "J7"),)),
        "b6": (("J7", "J1", "J5", "J11"), (("J7", "J5"), ("J5", "J11"))),
        "b7": (("J7", "J8"), (("J7", "J8"),)),
        "b8": (("J10", "J8"), (("J10", "J8"),)),
    },
    labels={"b1": "left rocker", "b2": "left bar + right tail", "b3": "left shin",
            "b4": "left foot link", "b5": "right rocker", "b6": "right bar + left tail",
            "b7": "right shin", "b8": "right foot link"},
    frame=("J2", "O", "J6"),
    crank=("O", "J1"),
    feet=(("b3", "J4"), ("b7", "J8")),
    # One Strider already is a mirrored pair: modules add pairs out of phase, each pair
    # of a double on one crank body (as Klann's mirrored pair), a decker's on its own.
    # The double (four feet per side, 180° apart) is the project's default walker: it
    # never leaves the ground, bobs 5.5 mm, steers and audits clean (docs/audit); the
    # quad only adds layers.
    default_module="double",
    modules={
        "single": Module(((+1, 0.0),)),
        "double": Module(((+1, 0.0), (+1, math.pi)), cranks=((0, 1),)),
        "decker": Module(((+1, 0.0), (+1, math.pi / 2))),
        "quad": Module(((+1, 0.0), (+1, math.pi), (+1, math.pi / 2), (+1, 3 * math.pi / 2)),
                       cranks=((0, 1), (2, 3))),
    },
    source="https://www.diywalkers.com/strider-linkage-plans.html",
    notes="Two feet per crankpin, 180° apart; long, flat stance.",
))
