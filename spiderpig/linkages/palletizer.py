"""The palletizer arm: a desktop four-axis arm's planar side (the shoulder and the elbow).

A palletizer arm (the WLKATA MT4, the Dobot Magician, the industrial palletizing robots)
keeps its tool level by two parallelograms and drives its elbow from the base, so both
heavy motors sit on the turntable. Its side view is a planar linkage of two inputs, an arm
(:attr:`linkage.Linkage.kind` ``"arm"``): no crank, each input turning a link about a
frame pivot over a range. The base's yaw and the wrist's roll are the robot's other two
axes, out of this plane (the robot-level turntable and tool).

The links (``b1`` .. ``b8``), with O the shoulder pivot (the frame), y up:

* the **upper arm** ``b1`` O-E, turned by ``q2`` (its angle from +x);
* the **elbow crank** ``b2`` O-C, turned by ``q3`` about the same pivot, and its **rod**
  ``b3`` C-R: with the forearm's rear arm E-R they make the parallelogram O-C-R-E, so the
  **forearm** ``b4`` R-E-W keeps ``q3``'s direction (its angle from +x) whatever ``q2``
  does: the elbow is driven from the shoulder;
* the **levelling rod** ``b5`` P-T from the frame pivot P, with the upper arm a second
  parallelogram O-P-T-E, so the **elbow plate** ``b6`` E-T-U never turns;
* the **wrist rod** ``b7`` U-V, with the forearm the third parallelogram E-U-V-W, so the
  **end plate** ``b8`` W-V-F never turns: the tool flange F stays level (the output, a
  translating platform).

Proportions are the MT4's where its published model gives them (WLKATA's URDF and STEP,
github.com/wlkata: upper arm 130 mm, forearm 150 mm, the flange 35 mm forward and 27.4 mm
under the wrist pivot). The pivots P and U and the elbow crank's radius are chosen: the
crank and the levelling arms as long as the side allows (60, 40 and 40 mm), since every
pin's play on them is magnified by the forearm over their length
(:mod:`spiderpig.precision`). The inputs' box is a rectangle (the tour's), ``q2``
40..115 deg and ``q3`` -45..20 deg: the elbow drive's parallelogram keeps its sides at
least :data:`MIN_PARALLELOGRAM_DEG` apart when ``q2 - q3`` stays in 20..160 deg, and the
rectangle is the largest inside that band. The MT4's own limit is on that relative angle,
so its workspace is the band itself: the rectangle gives up the band's corners (the
forearm pointing well down with the upper arm forward, the reach below the shoulder);
:func:`parallelogram_margins` says how close each comes.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import sympy as sp

from spiderpig.linkage import Linkage, Output, P, crank_at, register, xy

R = sp.Rational
MIN_PARALLELOGRAM_DEG = 20.0


def palletizer(p):
    """The program: the shoulder O, the frame pivot P, the elbow E, the elbow crank's pin C
    and the forearm's rear pin R (parallel, the same radius), the wrist W, the levelling
    plates' corners T, U, V (each a fixed vector from its plate's pivot) and the flange F."""
    return [
        ("O", xy(0, 0)),
        ("P", xy(p["px"], p["py"])),
        ("E", crank_at(P("O"), p["upper"], "q2")),
        ("C", crank_at(P("O"), p["crank"], "q3", 180)),
        ("R", crank_at(P("E"), p["crank"], "q3", 180)),
        ("W", crank_at(P("E"), p["fore"], "q3")),
        ("T", P("E") + xy(p["px"], p["py"])),
        ("U", P("E") + xy(p["ux"], p["uy"])),
        ("V", P("W") + xy(p["ux"], p["uy"])),
        ("F", P("W") + xy(p["fx"], p["fy"])),
    ]


PALLETIZER = register(Linkage(
    key="palletizer", name="Palletizer arm (MT4-like: 130 / 150 mm)", program=palletizer,
    inputs=("q2", "q3"), crank=(),
    ranges={"q2": (40.0, 115.0), "q3": (-45.0, 20.0)},
    drives={"q2": ("b1", "O"), "q3": ("b2", "O")},
    params={"upper": R(130), "fore": R(150), "crank": R(60),
            "px": R(-3939, 100), "py": R(695, 100),       # |OP| 40 mm at 170 deg
            "ux": R(0), "uy": R(40),                      # the elbow plate's third corner
            "fx": R(35), "fy": R(-274, 10)},              # the flange (MT4)
    links={"b1": (("O", "E"), (("O", "E"),)),
           "b2": (("O", "C"), (("O", "C"),)),
           "b3": (("C", "R"), (("C", "R"),)),
           "b4": (("R", "E", "W"), (("R", "E"), ("E", "W"))),
           "b5": (("P", "T"), (("P", "T"),)),
           "b6": (("E", "T", "U"), (("E", "T"), ("E", "U"))),
           "b7": (("U", "V"), (("U", "V"),)),
           "b8": (("W", "V", "F"), (("W", "V"), ("W", "F")))},
    labels={"b1": "upper arm (input q2)", "b2": "elbow crank (input q3)",
            "b3": "elbow rod", "b4": "forearm", "b5": "levelling rod",
            "b6": "elbow plate (never turns)", "b7": "wrist rod",
            "b8": "end plate (the tool flange; never turns)"},
    frame=("O", "P"),
    output=Output("point", "F", "translation_platform", ("W", "F"), body="b8"),
    source="https://github.com/wlkata/Wlkata_MT4_ROS2 (link lengths); "
           "https://github.com/wlkata/MT4-STL",
    notes="Two inputs, both driven from the shoulder; the tool stays level. The robot-level "
          "turntable and wrist add the arm's other two axes.",
))


PALLETIZER_REACH = register(replace(
    PALLETIZER, key="palletizer_reach",
    name="Palletizer arm reaching the table (MT4-like: 130 / 150 mm, crank 90, a band)",
    ranges={"q2": (-10.0, 110.0), "q3": (-90.0, 10.0)}, band=(30.0, 150.0),
    params={**PALLETIZER.params, "crank": R(90),
            "px": R(3064, 100), "py": R(-2571, 100),    # |OP| 40 mm at -40 deg: in front
            "ux": R(3064, 100), "uy": R(2571, 100)},    # the elbow plate's corner at 40 deg
    notes="The round-6 geometry of the arm's precision review (2026-10-12): the elbow crank "
          "90 mm (its parallelogram's gains 9.1 / 7.3 -> 6.0 / 3.3 at O, E / C, R), the "
          "inputs' box widened and held to q2 - q3 within 30..150 deg (every parallelogram "
          "30 deg or more off its dead point), the levelling pivot P and the wrist corner U "
          "moved so: the tip reaches table height over a 168 mm square once the turret keeps "
          "the arm out of the base instead of under it. Round 7 (the turret's, 2026-10-12) "
          "built that turret (``base_pair``) and held q3 to -90 deg: the elbow folded further "
          "with the shoulder low only brought the tool down beside the base (a tenth of the "
          "box), where a 150 mm shoulder can't let it."))
"""The palletizer that reaches its table (module docstring of the review's round 6)."""


def parallelogram_margins(values: tuple[float, ...] = ()) -> dict[str, float]:
    """The least angle (deg) between each parallelogram's two sides over the box: the
    elbow drive O-C-R-E, the levelling O-P-T-E, the wrist E-U-V-W. Near 0 a parallelogram
    flattens and can flip."""
    lk = PALLETIZER
    pts = lk.solve(params=dict(zip(lk.params, values, strict=True)) if values else None
                   ).at(*lk.box())

    def least(a, b, c) -> float:
        u, v = pts[b] - pts[a], pts[c] - pts[a]
        cross = np.abs(u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0])
        ang = np.degrees(np.arctan2(cross, (u * v).sum(-1)))
        return float(np.minimum(ang, 180.0 - ang).min())

    return {"elbow drive": least("O", "C", "E"), "levelling": least("O", "P", "E"),
            "wrist": least("E", "U", "W")}


def reach_mm(values: tuple[float, ...] = ()) -> float:
    """How far forward of the shoulder the flange gets over the box (mm)."""
    lk = PALLETIZER
    pts = lk.solve(params=dict(zip(lk.params, values, strict=True)) if values else None
                   ).at(*lk.box())
    return float(pts["F"][:, 0].max())


