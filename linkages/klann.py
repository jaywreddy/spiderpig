"""Joseph Klann's linkage (US patent 6,260,862): a 6-bar leg, straight-bar form.

The proportions are the 2016 project's; the patent drawing itself (bent rider
and leg) is ``klann_patent`` in :mod:`linkages.klann_variants`.

Lengths are multiples of ``OA`` (crank centre to the lower frame pivot A, in
mm); ``angA`` / ``angB`` place the frame pivots, in degrees from straight
down / up.
"""

from __future__ import annotations

import sympy as sp

from linkage import Linkage, P, circle_x_circle, crank, extend, register, rotate, xy

PARAMS = {
    "OA": sp.Integer(60),
    "angA": sp.Rational(6628, 100),
    "OB": sp.Rational(1121, 1000),
    "angB": sp.Rational(-4176, 100),
    "OM": sp.Rational(412, 1000),
    "MC": sp.Rational(1143, 1000),
    "AC": sp.Rational(909, 1000),
    "CD": sp.Rational(726, 1000),
    "DE": sp.Rational(93, 100),
    "BE": sp.Rational(8, 10),
    "DF": sp.Rational(2577, 1000),
}


def program(p):
    oa = p["OA"]
    return [
        ("O", xy(0, 0)),
        ("A", rotate(xy(0, -oa), p["angA"])),
        ("B", rotate(xy(0, p["OB"] * oa), p["angB"])),
        ("M", crank(p["OM"] * oa)),
        ("C", circle_x_circle(P("M"), p["MC"] * oa, P("A"), p["AC"] * oa, 1)),
        ("D", extend(P("M"), P("C"), p["CD"] * oa)),
        ("E", circle_x_circle(P("B"), p["BE"] * oa, P("D"), p["DE"] * oa, 1)),
        ("F", extend(P("E"), P("D"), p["DF"] * oa)),
    ]


KLANN = register(Linkage(
    key="klann",
    name="Klann (2016 project proportions)",
    family="klann",
    params=PARAMS,
    angles=frozenset({"angA", "angB"}),
    program=program,
    links={
        "b1": (("M", "C", "D"), (("M", "D"),)),
        "b2": (("B", "E"), (("B", "E"),)),
        "b3": (("A", "C"), (("A", "C"),)),
        "b4": (("E", "D", "F"), (("E", "F"),)),
    },
    labels={"b1": "crank rider (M-C-D)", "b2": "upper rocker (B-E)",
            "b3": "lower rocker (A-C)", "b4": "leg (E-D-F)"},
    frame=("A", "O", "B"),
    crank=("O", "M"),
    feet=(("b4", "F"),),
    source="https://www.diywalkers.com/klanns-linkage-plans.html",
    notes="Six bars, two frame pivots, straight bars; a high lift. The project default.",
))
