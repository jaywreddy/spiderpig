"""Building-block mechanisms: straight lines, lifts, xy and rotations, driven by a crank.

Not walkers: each declares its :class:`linkage.Output` (the point or link that
does the job, and what it promises) instead of feet. Later, stages stack: one
mechanism's output link carries the next one's frame (see CLAUDE.md,
"Stacking (future)").

Lengths are in a drawing unit (``unit`` mm each; the crank is 1 unit), as
selected and checked numerically in the research (its ``verify.py``; its
numbers are in ``tests/test_mechanisms.py``). Every one plans and builds at
that scale except the five-bar, which needs a second drive. The docstring of
each program gives its source and what its output does. Output bodies carry
``(next stage's frame)`` in their labels.
"""

from __future__ import annotations

import sympy as sp

from linkage import (
    Linkage,
    Output,
    P,
    circle_x_circle,
    crank,
    crank_at,
    extend,
    offset,
    register,
    xy,
)

R = sp.Rational
DOM = "http://www.designofmachinery.com/DOM/Chap_03_3ed_p134.pdf"   # Norton, Table 3-1


def _two(a, b):
    return (a, b), ((a, b),)


# --- straight line -------------------------------------------------------------


def hoecken(p):
    """Hoeckens straight-line four-bar (Norton, Design of Machinery, Table 3-1, the
    180° straightness row; the Chebyshev lambda's cognate). The coupler point P,
    on the coupler's extension (MP = 2 link), runs along y = const at nearly
    constant speed for the half turn centred on t = 180°, then returns over a
    D-shaped arc above the line. The coupler b1 rotates: ``hoecken_table`` is
    the translating version."""
    u = p["unit"]
    return [
        ("O", xy(0, 0)),
        ("Q", xy(p["ground"] * u, 0)),
        ("M", crank(p["crank"] * u)),
        ("B", circle_x_circle(P("M"), p["link"] * u, P("Q"), p["link"] * u, 1)),
        ("P", extend(P("M"), P("B"), p["link"] * u)),
    ]


register(Linkage(
    key="hoecken", name="Hoeckens straight line", program=hoecken,
    params={"unit": R(16), "crank": R(1), "ground": R(11, 5), "link": R(14, 5)},
    links={"b1": (("M", "B", "P"), (("M", "P"),)), "b2": _two("Q", "B")},
    labels={"b1": "coupler M-B-P (P on the extension, MP = 2 link)", "b2": "rocker Q-B"},
    frame=("O", "Q"), crank=("O", "M"),
    output=Output("point", "P", "line", ("P", "B"), body="b1", straight=(90, 270, 0.1)),
    source=DOM, notes="Four bars; P runs a 67 mm straight stroke over half the turn.",
))


def watt_crank(p):
    """A crank driving a Watt linkage's coupler, like a piston (an all-revolute
    slider-crank; Watt's linkage as the guide: en.wikipedia.org/wiki/Watt%27s_linkage).
    P, the middle of the Watt coupler B-D, reciprocates on x = 0: up over one
    half turn, down over the other, 2 crank apart; its link b3 rocks ~5°."""
    u = p["unit"]
    return [
        ("O", xy(0, 0)),
        ("A", xy(p["ax"] * u, p["ay"] * u)),
        ("C", xy(p["cx"] * u, p["cy"] * u)),
        ("M", crank(p["crank"] * u)),
        ("B", circle_x_circle(P("M"), p["rod"] * u, P("A"), p["arm"] * u, -1)),
        ("D", circle_x_circle(P("B"), p["coupler"] * u, P("C"), p["arm"] * u, 1)),
        ("P", offset(P("B"), P("D"), p["coupler"] * u / 2, 0)),
    ]


WATT = {"unit": R(16), "crank": R(1), "rod": R(3), "arm": R(4), "coupler": R(3),
        "ax": R(-4), "ay": R(3), "cx": R(4), "cy": R(6)}
WATT_LINKS = {"b1": _two("M", "B"), "b2": _two("A", "B"),
              "b3": (("B", "P", "D"), (("B", "D"),)), "b4": _two("C", "D")}
WATT_LABELS = {"b1": "connecting rod", "b2": "Watt arm A-B (driven)",
               "b3": "Watt coupler B-D, P at its middle", "b4": "Watt arm C-D"}

register(Linkage(
    key="watt_crank", name="Watt-guided crank (reciprocating line)", program=watt_crank,
    params=WATT, links=WATT_LINKS, labels=WATT_LABELS,
    frame=("O", "A", "C"), crank=("O", "M"),
    output=Output("point", "P", "line", ("P", "D"), body="b3", straight=(0, 360, 0.05)),
    source="https://en.wikipedia.org/wiki/Watt%27s_linkage",
    notes="Six bars; P goes up and down a 32 mm straight line once per turn.",
))


def peaucellier_crank(p):
    """The Peaucellier-Lipkin inversor (en.wikipedia.org/wiki/Peaucellier-Lipkin_linkage)
    driven by a crank-rocker: the bell crank b2 swings Q on a circle through
    the inversor's pivot Z, so X moves on the exact line
    x = Zx + (arm² - side²) / (2 r_in), up and down 2.9 crank once per turn."""
    u = p["unit"]
    return [
        ("O", xy(0, 0)),
        ("Y", xy(p["yx"] * u, p["yy"] * u)),                       # the bell crank's pivot
        ("Z", xy((p["yx"] - p["r_in"]) * u, p["yy"] * u)),         # the inversor's, |YZ| = r_in
        ("M", crank(p["crank"] * u)),
        ("E", circle_x_circle(P("M"), p["rod"] * u, P("Y"), p["pin"] * u, 1)),
        ("Q", offset(P("Y"), P("E"), 0, -p["r_in"] * u)),         # at 90° to E: Q's circle meets Z
        ("R1", circle_x_circle(P("Z"), p["arm"] * u, P("Q"), p["side"] * u, 1)),
        ("R2", circle_x_circle(P("Z"), p["arm"] * u, P("Q"), p["side"] * u, -1)),
        ("X", circle_x_circle(P("R1"), p["side"] * u, P("R2"), p["side"] * u, 1)),
    ]


register(Linkage(
    key="peaucellier_crank", name="Peaucellier-Lipkin inversor (exact line)",
    program=peaucellier_crank,
    params={"unit": R(16), "crank": R(1), "rod": R(3), "arm": R(5), "side": R(3, 2),
            "r_in": R(17, 8), "pin": R(2), "yx": R(3), "yy": R(-7, 4)},
    links={"b1": _two("M", "E"), "b2": (("Y", "E", "Q"), (("Y", "E"), ("Y", "Q"))),
           "b3": _two("Z", "R1"), "b4": _two("Z", "R2"), "b5": _two("R1", "Q"),
           "b6": _two("R2", "Q"), "b7": _two("R1", "X"), "b8": _two("R2", "X")},
    labels={"b1": "connecting rod", "b2": "input bell crank (Y; rod pin E, Q at 90°)",
            "b3": "long arm Z-R1", "b4": "long arm Z-R2", "b5": "rhombus side R1-Q",
            "b6": "rhombus side R2-Q", "b7": "rhombus side R1-X", "b8": "rhombus side R2-X"},
    frame=("O", "Y", "Z"), crank=("O", "M"),
    output=Output("point", "X", "line", ("X", "R1"), body="b7", straight=(0, 360, 1e-6)),
    source="https://en.wikipedia.org/wiki/Peaucellier%E2%80%93Lipkin_linkage",
    notes="Ten bars; X goes up and down an exactly straight 46 mm line once per turn.",
))


# --- lift ------------------------------------------------------------------------


def parallelogram_lift(p):
    """A parallelogram (en.wikipedia.org/wiki/Parallel_motion) driven by a centric
    crank-rocker: the platform b4 never rotates; it rises and falls 2 crank
    once per turn along an arc of radius ``arm`` about G1 (1.6 mm sideways),
    with a toggle (zero speed, full force) at both ends."""
    u = p["unit"]
    return [
        ("O", xy(0, 0)),
        ("G1", xy(p["gx"] * u, p["gy"] * u)),
        ("G2", xy(p["gx"] * u, (p["gy"] + p["spacing"]) * u)),
        ("M", crank(p["crank"] * u)),
        ("T1", circle_x_circle(P("M"), p["rod"] * u, P("G1"), p["arm"] * u, -1)),
        ("T2", circle_x_circle(P("T1"), p["spacing"] * u, P("G2"), p["arm"] * u, -1)),
    ]


register(Linkage(
    key="parallelogram_lift", name="Parallelogram lift", program=parallelogram_lift,
    params={"unit": R(16), "crank": R(1), "rod": R(3), "arm": R(5), "spacing": R(2),
            "gx": R(-49, 10), "gy": R(3)},
    links={"b1": _two("M", "T1"), "b2": _two("G1", "T1"), "b3": _two("G2", "T2"),
           "b4": _two("T1", "T2")},
    labels={"b1": "connecting rod", "b2": "lower arm (driven)", "b3": "upper arm",
            "b4": "platform (next stage's frame)"},
    frame=("O", "G1", "G2"), crank=("O", "M"),
    output=Output("body", "b4", "translation_platform", ("T1", "T2")),
    source="https://en.wikipedia.org/wiki/Parallel_motion",
    notes="Five bars; a platform that lifts 32 mm without turning.",
))


def watt_table_lift(p):
    """``watt_crank`` plus a parallel copy (Sarrus-equivalent, all dyads): the arm
    A-B is copied to A2 = A + (sx, sy) by the parallelogram A-E-E2-A2, the
    half coupler B-P by B2-P2, so the table b8 = P-P2 never rotates and each of
    its points runs a Watt straight line: up and down 2 crank once per turn."""
    u = p["unit"]
    shift = sp.sqrt(p["sx"] ** 2 + p["sy"] ** 2) * u
    e = sp.sqrt(p["e_along"] ** 2 + p["e_across"] ** 2)
    return [
        *watt_crank(p)[:3],
        ("A2", xy((p["ax"] + p["sx"]) * u, (p["ay"] + p["sy"]) * u)),
        *watt_crank(p)[3:],
        # the coupling pin on the arm, 2 from A, 53.13° (3-4-5) off the bar: square to the shift
        ("E", offset(P("A"), P("B"), p["e_along"] * u, p["e_across"] * u)),
        ("E2", circle_x_circle(P("E"), shift, P("A2"), e * u, -1)),
        ("B2", offset(P("A2"), P("E2"), p["arm"] * p["e_along"] / e * u,
                      -p["arm"] * p["e_across"] / e * u)),
        ("P2", circle_x_circle(P("P"), shift, P("B2"), p["coupler"] * u / 2, -1)),
    ]


register(Linkage(
    key="watt_table_lift", name="Watt table lift (straight, level)", program=watt_table_lift,
    params={**WATT, "sx": R(-4), "sy": R(3), "e_along": R(6, 5), "e_across": R(8, 5)},
    links={**WATT_LINKS, "b2": (("A", "E", "B"), (("A", "B"), ("A", "E"))),
           "b5": _two("E", "E2"), "b6": (("A2", "E2", "B2"), (("A2", "B2"), ("A2", "E2"))),
           "b7": _two("B2", "P2"), "b8": _two("P", "P2")},
    labels={**WATT_LABELS, "b2": "Watt arm A-B (driven; coupling pin E)",
            "b5": "coupling rod E-E2 (parallelogram A-E-E2-A2)", "b6": "arm copy A2-E2-B2",
            "b7": "half-coupler copy B2-P2", "b8": "table P-P2 (next stage's frame)"},
    frame=("O", "A", "C", "A2"), crank=("O", "M"),
    output=Output("body", "b8", "translation_platform", ("P", "P2"), straight=(0, 360, 0.05)),
    source="https://en.wikipedia.org/wiki/Sarrus_linkage",
    notes="Nine bars; a level table that lifts 32 mm on a straight line.",
))


# --- xy ----------------------------------------------------------------------------


def five_bar(p):
    """The five-bar parallel xy mechanism (en.wikipedia.org/wiki/Five-bar_linkage):
    two full-turn cranks, at O (input ``t``) and O2 (``t2``), carry the distal
    links that meet at P. Every (t, t2) assembles (base > 2 crank and base +
    2 crank < 2 distal), so both inputs may spin freely; P reaches a
    64 x 53 mm workspace."""
    u = p["unit"]
    return [
        ("O", xy(0, 0)),
        ("O2", xy(p["base"] * u, 0)),
        ("M", crank(p["crank"] * u)),
        ("M2", crank_at(P("O2"), p["crank"] * u, "t2")),
        ("P", circle_x_circle(P("M"), p["distal"] * u, P("M2"), p["distal"] * u, 1)),
    ]


register(Linkage(
    key="five_bar", name="Five-bar xy (two inputs)", program=five_bar, inputs=("t", "t2"),
    params={"unit": R(20), "crank": R(1), "distal": R(4), "base": R(5)},
    links={"b1": _two("M", "P"), "b2": _two("M2", "P"), "b3": _two("O2", "M2")},
    labels={"b1": "distal link 1", "b2": "distal link 2", "b3": "second crank (input t2)"},
    frame=("O", "O2"), crank=("O", "M"),
    output=Output("point", "P", "xy", ("P", "M"), body="b1"),
    source="https://en.wikipedia.org/wiki/Five-bar_linkage",
    notes="Two cranks place P anywhere in its workspace. Needs a second drive.",
))


def hoecken_pantograph(p):
    """A x2 pantograph (en.wikipedia.org/wiki/Pantograph) on the Hoeckens point P:
    the rhombus F-J1-P-J2 pivoted at F makes X = F + 2 (P - F), the Hoeckens
    path scaled twice about F: a 100 mm straight stroke over half the turn, at
    half the force."""
    u = p["unit"]
    return [
        *hoecken(p)[:2],
        ("F", xy(p["fx"] * u, p["fy"] * u)),
        *hoecken(p)[2:],
        ("J1", circle_x_circle(P("F"), p["half"] * u, P("P"), p["half"] * u, 1)),
        ("K", extend(P("F"), P("J1"), p["half"] * u)),
        ("J2", circle_x_circle(P("K"), p["half"] * u, P("P"), p["half"] * u, 1)),
        ("X", extend(P("K"), P("J2"), p["half"] * u)),
    ]


register(Linkage(
    key="hoecken_pantograph", name="Hoeckens x2 pantograph", program=hoecken_pantograph,
    params={"unit": R(12), "crank": R(1), "ground": R(11, 5), "link": R(14, 5),
            "fx": R(11, 5), "fy": R(39, 5), "half": R(11, 5)},
    links={"b1": (("M", "B", "P"), (("M", "P"),)), "b2": _two("Q", "B"),
           "b3": (("F", "J1", "K"), (("F", "K"),)), "b4": (("K", "J2", "X"), (("K", "X"),)),
           "b5": _two("J1", "P"), "b6": _two("J2", "P")},
    labels={"b1": "Hoeckens coupler", "b2": "Hoeckens rocker", "b3": "pantograph arm F-J1-K",
            "b4": "pantograph arm K-J2-X", "b5": "rhombus bar J1-P", "b6": "rhombus bar J2-P"},
    frame=("O", "Q", "F"), crank=("O", "M"),
    output=Output("point", "X", "path", ("X", "K"), body="b4", straight=(90, 270, 0.15)),
    source="https://en.wikipedia.org/wiki/Pantograph",
    notes="Eight bars; the Hoeckens path at twice the size.",
))


# --- rotation --------------------------------------------------------------------


def crank_rocker(p):
    """A centric crank-rocker (en.wikipedia.org/wiki/Four-bar_linkage): the rocker
    b2 swings 2 asin(crank / rocker) = 60° about G, symmetric about straight
    down, equal times each way; at both ends it toggles (stands still, full
    force)."""
    u = p["unit"]
    return [
        ("O", xy(0, 0)),
        ("G", xy(p["coupler"] * u, sp.sqrt(p["rocker"] ** 2 - p["crank"] ** 2) * u)),
        ("M", crank(p["crank"] * u)),
        ("E", circle_x_circle(P("M"), p["coupler"] * u, P("G"), p["rocker"] * u, -1)),
    ]


register(Linkage(
    key="crank_rocker", name="Crank-rocker (60°)", program=crank_rocker,
    params={"unit": R(16), "crank": R(1), "rocker": R(2), "coupler": R(3)},
    links={"b1": _two("M", "E"), "b2": _two("G", "E")},
    labels={"b1": "coupler", "b2": "output rocker (next stage's frame)"},
    frame=("O", "G"), crank=("O", "M"),
    output=Output("body", "b2", "rotation", ("G", "E")),
    source="https://en.wikipedia.org/wiki/Four-bar_linkage",
    notes="Four bars; a rocker swinging 60° once per turn.",
))


def rocker_amplifier(p):
    """``crank_rocker`` plus a dyad (a Watt six-bar,
    en.wikipedia.org/wiki/Six-bar_linkage): the first rocker b2 is a lever
    through G whose far end E2 drives the short output rocker b4 through
    b3, amplifying 60° to 153° about H, once each way per turn."""
    u = p["unit"]
    return [
        *crank_rocker(p)[:2],
        ("H", xy(p["hx"] * u, p["hy"] * u)),
        *crank_rocker(p)[2:],
        ("E2", offset(P("G"), P("E"), p["e_along"] * u, p["e_across"] * u)),
        ("K", circle_x_circle(P("E2"), p["link2"] * u, P("H"), p["out"] * u, -1)),
    ]


register(Linkage(
    key="rocker_amplifier", name="Rocker amplifier (153°)", program=rocker_amplifier,
    params={"unit": R(16), "crank": R(1), "rocker": R(2), "coupler": R(3),
            "e_along": R(-7, 2), "e_across": R(1), "hx": R(2), "hy": R(13, 4),
            "link2": R(11, 4), "out": R(5, 4)},
    links={"b1": _two("M", "E"), "b2": (("G", "E", "E2"), (("E", "G"), ("G", "E2"))),
           "b3": _two("E2", "K"), "b4": _two("H", "K")},
    labels={"b1": "coupler", "b2": "first rocker, a lever through G (E below, E2 above)",
            "b3": "link E2-K", "b4": "output rocker (next stage's frame)"},
    frame=("O", "G", "H"), crank=("O", "M"),
    output=Output("body", "b4", "rotation", ("H", "K")),
    source="https://en.wikipedia.org/wiki/Six-bar_linkage",
    notes="Six bars; an output rocker swinging 153° once per turn.",
))


def dwell_rocker(p):
    """A single-dwell six-bar (Norton, Design of Machinery, ch. 3): the coupler
    point P of a four-bar runs a near-circular arc of radius ``dyad`` about
    the output pin K's rest position, so the rocker b4 stands still (±0.25°)
    for 125° of the turn; the rest of the turn it swings 44° out and back."""
    u = p["unit"]
    return [
        ("O", xy(0, 0)),
        ("Q", xy(p["ground"] * u, 0)),
        ("W", xy(p["wx"] * u, p["wy"] * u)),
        ("M", crank(p["crank"] * u)),
        ("B", circle_x_circle(P("M"), p["coupler"] * u, P("Q"), p["rocker"] * u, 1)),
        ("P", offset(P("M"), P("B"), p["p_along"] * u, p["p_across"] * u)),
        ("K", circle_x_circle(P("P"), p["dyad"] * u, P("W"), p["out"] * u, 1)),
    ]


register(Linkage(
    key="dwell_rocker", name="Dwell rocker (44°, 125° dwell)", program=dwell_rocker,
    params={"unit": R(15), "crank": R(1), "ground": R(19, 4), "coupler": R(11, 4),
            "rocker": R(19, 4), "p_along": R(9, 2), "p_across": R(3, 4), "dyad": R(229, 100),
            "out": R(251, 200), "wx": R(1, 4), "wy": R(27, 4)},
    links={"b1": (("M", "B", "P"), (("M", "B"), ("B", "P"))), "b2": _two("Q", "B"),
           "b3": _two("P", "K"), "b4": _two("W", "K")},
    labels={"b1": "coupler (tracer P)", "b2": "rocker Q-B", "b3": "dyad link P-K",
            "b4": "output rocker (next stage's frame)"},
    frame=("O", "Q", "W"), crank=("O", "M"),
    output=Output("body", "b4", "rotation", ("W", "K"), dwell=(0.25, 120)),
    source=DOM, notes="Six bars; a rocker that rests for a third of the turn.",
))


# --- lift and carry --------------------------------------------------------------


def hoecken_table(p):
    """The Hoeckens linkage with a parallel copy (Chebyshev's plantigrade machine,
    the walking-beam principle: en.wikipedia.org/wiki/Chebyshev_lambda_linkage):
    the rocker Q-B is copied to Q2 = Q + (shift, 0) through the coupling rod
    B-B2, the coupler's B-P by B2-P2, so the table b6 = P-P2 never rotates and
    each of its points runs the Hoeckens D: a 67 mm straight stroke at nearly
    constant speed, then a lifted return."""
    u = p["unit"]
    return [
        *hoecken(p)[:2],
        ("Q2", xy((p["ground"] + p["shift"]) * u, 0)),
        *hoecken(p)[2:],
        ("B2", circle_x_circle(P("B"), p["shift"] * u, P("Q2"), p["link"] * u, 1)),
        ("P2", circle_x_circle(P("P"), p["shift"] * u, P("B2"), p["link"] * u, 1)),
    ]


register(Linkage(
    key="hoecken_table", name="Hoeckens table (lift and carry)", program=hoecken_table,
    params={"unit": R(16), "crank": R(1), "ground": R(11, 5), "link": R(14, 5), "shift": R(5)},
    links={"b1": (("M", "B", "P"), (("M", "P"),)), "b2": _two("Q", "B"), "b3": _two("B", "B2"),
           "b4": _two("Q2", "B2"), "b5": _two("B2", "P2"), "b6": _two("P", "P2")},
    labels={"b1": "Hoeckens coupler", "b2": "Hoeckens rocker", "b3": "coupling rod B-B2",
            "b4": "rocker copy Q2-B2", "b5": "half-coupler copy B2-P2",
            "b6": "table P-P2 (next stage's frame)"},
    frame=("O", "Q", "Q2"), crank=("O", "M"),
    output=Output("body", "b6", "translation_platform", ("P", "P2"), straight=(90, 270, 0.1)),
    source="https://en.wikipedia.org/wiki/Chebyshev_lambda_linkage",
    notes="Seven bars; a level table carried 67 mm straight, then lifted back.",
))
