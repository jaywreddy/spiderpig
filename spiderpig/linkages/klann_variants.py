"""Klann's linkage as diywalkers.com draws it, and its LEGO builds.

The diywalkers Klann simulator describes a leg by the crank centre's offsets
to the two frame pivots (``HiFrameX/Y``, ``LowFrameX/Y``), bar lengths
``b0, b2 .. b7`` and two fixed bends, in drawing units where the crank is 3
(for the LEGO builds a unit is one hole pitch). The crank rider turns by
``B4 angle`` at the knee C, where the lower rocker meets it, and the leg
turns by ``LegAngle`` at D, where the crank rider drives it. The default
:mod:`linkages.klann` (the 2016 proportions) keeps both bars straight, so
these are a program of their own:

=========  ==========  ==============================================
page       here        what
=========  ==========  ==============================================
b0         OM          crank
b2         MC          crankpin M to the knee C (crank rider)
b4         CD          knee C to D (crank rider, turned by ``bendC``)
b3         AC          lower rocker, frame pivot A to C
b6         BE          upper rocker, frame pivot B to E
b5         DE          upper leg, D to E
b7         DF          lower leg, D to the foot F (turned by ``bendD``)
B4 angle   bendC       degrees; the crank rider turns left at C
LegAngle   bendD       degrees; the leg turns right at D
HiFrameX   hi_x        B = (hi_x, hi_y) from the crank centre
HiFrameY   hi_y
LowFrameX  low_x       A = (low_x, -low_drop): LowFrameY = -low_drop
LowFrameY  low_drop
=========  ==========  ==============================================

``unit`` is mm per drawing unit: 8, the LEGO hole pitch, for every variant,
so all have a 24 mm crank (the default Klann's is 24.7 mm).

Our leg (orientation +1) is the page's right-hand leg, whose frame pivots
lie right of the crank; the page's left-hand leg is its mirror image
(orientation -1), and both ride one crankpin as in the page's drawings.
"""

from __future__ import annotations

import sympy as sp

from spiderpig.linkage import KLANN_QUAD, Linkage, P, circle_x_circle, crank, register, rotate, xy

R = sp.Rational


def bent(frm: sp.Matrix, through: sp.Matrix, length, degrees) -> sp.Matrix:
    """``length`` beyond ``through`` on the ray frm -> through turned ``degrees`` left."""
    u = through - frm
    return through + rotate(u, degrees) * length / sp.sqrt(u.dot(u))


# Klann's patent drawing as diywalkers transcribes it ("Klann's Linkage
# Plans", section 1, scaled to a crank of 3). Bars from the page's table
# "Klann's Standard Bar Lengths, Patent vs LEGO" (the drawing rounds them to
# 6.6, 3.59, 5.84, 10.04, 5.79, 10.04; the simulator's b7 slider says 10.01);
# frame pivots from "Frame Coordinates and Bar Angles": crank (10, 10),
# B (7.4, 16.9), A (3.4, 8.03); bends from the simulator (12.83°, 30°).
PATENT = {
    "unit": R(8),
    "OM": R(3),                # b0
    "MC": R(6604, 1000),       # b2
    "CD": R(5844, 1000),       # b4
    "AC": R(3589, 1000),       # b3
    "BE": R(5795, 1000),       # b6
    "DE": R(10037, 1000),      # b5
    "DF": R(10036, 1000),      # b7
    "bendC": R(1283, 100),     # B4 angle (the drawing: 12.8°)
    "bendD": R(30),            # LegAngle
    "hi_x": R(26, 10),         # HiFrameX: 10 - 7.4
    "hi_y": R(69, 10),         # HiFrameY: 16.9 - 10
    "low_x": R(66, 10),        # LowFrameX: 10 - 3.4
    "low_drop": R(197, 100),   # -LowFrameY: 10 - 8.03
}


def program(p):
    u = p["unit"]
    return [
        ("O", xy(0, 0)),
        ("A", xy(p["low_x"] * u, -p["low_drop"] * u)),
        ("B", xy(p["hi_x"] * u, p["hi_y"] * u)),
        ("M", crank(p["OM"] * u)),
        ("C", circle_x_circle(P("M"), p["MC"] * u, P("A"), p["AC"] * u, 1)),
        ("D", bent(P("M"), P("C"), p["CD"] * u, p["bendC"])),
        ("E", circle_x_circle(P("B"), p["BE"] * u, P("D"), p["DE"] * u, 1)),
        ("F", bent(P("E"), P("D"), p["DF"] * u, -p["bendD"])),
    ]


PLANS = "https://www.diywalkers.com/klanns-linkage-plans.html"
PLANNED = ("Plans with the default constructions (3 mm sheet), stack height: "
           "single {}, double {}, decker {}, quad {} mm.")

KLANN_PATENT = register(Linkage(
    key="klann_patent",
    name="Klann (patent drawing, diywalkers)",
    family="klann",
    params=PATENT,
    angles=frozenset({"bendC", "bendD"}),
    program=program,
    links={
        "b1": (("M", "C", "D"), (("M", "C"), ("C", "D"))),
        "b2": (("B", "E"), (("B", "E"),)),
        "b3": (("A", "C"), (("A", "C"),)),
        "b4": (("E", "D", "F"), (("E", "D"), ("D", "F"))),
    },
    labels={"b1": "crank rider (M-C-D, bent at C)", "b2": "upper rocker (B-E)",
            "b3": "lower rocker (A-C)", "b4": "leg (E-D-F, bent at D)"},
    frame=("A", "O", "B"),
    crank=("O", "M"),
    feet=(("b4", "F"),),
    modules={"quad": KLANN_QUAD},
    source=PLANS,
    notes=(
        "Klann's patent drawing as diywalkers transcribes it. Not the default `klann` "
        "(the 2016 proportions): here the crank rider bends 12.83° at the knee and the leg "
        "30°, where `klann` keeps both straight; in these units (crank 3) `klann` has "
        "A (6.67, -2.93), B (5.44, 6.09), MC 8.32, AC 6.62, CD 5.29, DE 6.77, BE 5.83 and a "
        "straight DF 18.76. Foot path 90 x 47 mm (`klann`: 117 x 86 mm). "
        + PLANNED.format(18, 21, 30, 39)
    ),
))

KLANN_LEGO = register(KLANN_PATENT.variant(
    "klann_lego", "Klann (LEGO, diywalkers ver 2)",
    OM=R(3), MC=R(7), CD=R(6), AC=R(4), BE=R(6), DE=R(10), DF=R(10),
    bendC=R(0), bendD=R(29), hi_x=R(3), hi_y=R(7), low_x=R(7), low_drop=R(2),
    source=PLANS,
    notes=(
        "diywalkers' LEGO approximation of the patent linkage (Klann Ver 2), in hole "
        "pitches: crank 3, frame pivots at (3, 8) and (7, 17) with the crank at (10, 10) "
        "(the page's left leg), a straight crank rider (7 + 6 pitches on one beam), "
        "lower rocker 4, upper rocker 6, upper leg 10 and an 11-hole lower leg (10) bent "
        "29° at the knee. " + PLANNED.format(18, 21, 30, 39)
    ),
))

KLANN_LONG_LEGS = register(KLANN_LEGO.variant(
    "klann_long_legs", "Klann (LEGO long legs, EV3 spider)",
    DF=R(14),
    source="https://www.diywalkers.com/klanns-spider-ev3-long-legs.html",
    notes=(
        "Klann's Spider Ver 3: the LEGO linkage with the lower leg lengthened by 4 holes "
        "(an 11-hole beam becomes a 15-hole one: 14 pitches), still bent 29°, for ground "
        "clearance under the frame. The foot path widens (13.7 x 5.6 pitches vs 11.5 x 5.5). "
        + PLANNED.format(18, 21, 30, 39)
    ),
))

KLANN_HIGH_STEP = register(KLANN_LONG_LEGS.variant(
    "klann_high_step", "Klann (LEGO high-step mod)",
    MC=R(6), CD=R(7), low_x=R(6),
    source="https://www.diywalkers.com/klann-high-step-mod.html",
    notes=(
        "The long-legged LEGO linkage with the lower rocker and its frame pivot moved one "
        "hole toward the crank (b2 7 -> 6, b4 6 -> 7, LowFrameX 7 -> 6; lower pivot at "
        "(4, 8)): the step rises from 5.6 to 8.2 pitches. " + PLANNED.format(18, 30, 30, 42)
    ),
))
