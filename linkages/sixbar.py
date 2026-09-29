"""The 6-bar walking linkage (diywalkers): the 4-bar with its rocker extended.

Numbers are the page's Scratch simulator units (``unit`` mm each); the
params ``b1``..``b7`` are its sliders, i.e. the bar map's B1..B7. The crank
B1 turns about the crank centre ``O``; the hip joint ``H`` sits at
(``-hip_x``, ``hip_y``) from it. B2 runs from the crankpin ``M`` to the knee
``K`` on the rocker B3, which hangs from the hip and carries on (B5) past
the knee to ``P``. The leg B4 hangs from ``P``; B6 ties the foot ``F`` back
to B2 at ``Q``, B7 from the crankpin ("B6's upper connection's distance from
the crank"; it doesn't change B2). With B7 = 0, B6 rides the crankpin
itself. Link bodies: ``b1`` = B2, ``b2`` = B3 + B5, ``b3`` = B4, ``b4`` = B6.

The page's versions (hip; B1..B7):

* bar map: (-60, 60); 40, 80, 60, 90, 20, 110, 20
* Ver 1: (-60, 60); 40, 80, 60, 100, 20, 120, 20
* Ver 2: (-55, 60); 40, 80, 50, 100, 10, 110, 0
* Ver 3: (-50, 65); 40, 80, 50, 90, 20, 100, 0 (also its "4-bar to 6-bar"
  figure, where B7 isn't shown: B6 meets the crankpin there)

**Fabrication.** In the bar map and Ver 1, ``Q`` is 20 from the
crankpin, inside the crank circle (40): seen from B6, O circles ``Q`` once
per turn, so B6 sweeps across the crank axis whatever its shape, and the
printed crankshaft occupies O in every layer: the static clearance stage
rejects them until a crank overhung from the servo side exists. In Ver 2,
B5 = 10 puts ``P`` 6 mm from ``K`` at the 24 mm-crank scale; the two pins'
heads and shoulders can't clear each other in any layer order, so Ver 2 is
drawn at 1.0 mm per unit (a 40 mm crank), where it plans.
"""

from __future__ import annotations

import sympy as sp

from linkage import Linkage, P, circle_x_circle, crank, extend, offset, register, xy

R = sp.Rational
SOURCE = "https://www.diywalkers.com/6-bar-walking-linkage.html"


def _params(hip_x, hip_y, b1, b2, b3, b4, b5, b6, b7=None):
    out = {"unit": R(3, 5),     # mm per simulator unit (crank 24 mm)
           "hip_x": R(hip_x), "hip_y": R(hip_y), "b1": R(b1), "b2": R(b2), "b3": R(b3),
           "b4": R(b4), "b5": R(b5), "b6": R(b6)}
    if b7:
        out["b7"] = R(b7)
    return out


def _program(on_b2: bool):
    def program(p):
        u = p["unit"]
        steps = [
            ("O", xy(0, 0)),
            ("H", xy(-p["hip_x"] * u, p["hip_y"] * u)),
            ("M", crank(p["b1"] * u)),
            ("K", circle_x_circle(P("M"), p["b2"] * u, P("H"), p["b3"] * u, 1)),
            ("P", extend(P("H"), P("K"), p["b5"] * u)),
        ]
        if on_b2:
            steps.append(("Q", offset(P("M"), P("K"), p["b7"] * u, 0)))
        q = P("Q") if on_b2 else P("M")
        steps.append(("F", circle_x_circle(P("P"), p["b4"] * u, q, p["b6"] * u, -1)))
        return steps

    return program


def _sixbar(key: str, name: str, params: dict, notes: str) -> Linkage:
    on_b2 = "b7" in params
    q = "Q" if on_b2 else "M"
    return Linkage(
        key=key,
        name=name,
        family="sixbar",
        params=params,
        program=_program(on_b2),
        links={
            "b1": (("M", "Q", "K") if on_b2 else ("M", "K"), (("M", "K"),)),
            "b2": (("H", "K", "P"), (("H", "P"),)),
            "b3": (("P", "F"), (("P", "F"),)),
            "b4": ((q, "F"), ((q, "F"),)),
        },
        labels={"b1": "B2 (M-K)", "b2": "rocker B3 + B5 (H-K-P)", "b3": "leg B4 (P-F)",
                "b4": f"B6 ({q}-F)"},
        frame=("H", "O"),
        crank=("O", "M"),
        feet=(("b3", "F"),),
        source=SOURCE,
        notes=notes,
    )


SIXBAR = register(_sixbar(
    "sixbar", "6-bar walking linkage (bar map)", _params(60, 60, 40, 80, 60, 90, 20, 110, 20),
    notes="Six bars, one frame pivot: the 4-bar's rocker extended for a higher step. "
          "Needs an overhung crank (B6 sweeps across the crank axis).",
))
SIXBAR_V1 = register(_sixbar(
    "sixbar_v1", "6-bar walking linkage, Ver 1", _params(60, 60, 40, 80, 60, 100, 20, 120, 20),
    notes="The bar map with longer legs (B4 100, B6 120). Needs an overhung crank.",
))
SIXBAR_V2 = register(_sixbar(
    "sixbar_v2", "6-bar walking linkage, Ver 2",
    _params(55, 60, 40, 80, 50, 100, 10, 110) | {"unit": R(1)},
    notes="B6 on the crankpin (B7 = 0), a short rocker extension (B5 = 10); drawn at a "
          "40 mm crank so pins K and P clear each other.",
))
SIXBAR_V3 = register(_sixbar(
    "sixbar_v3", "6-bar walking linkage, Ver 3", _params(50, 65, 40, 80, 50, 90, 20, 100),
    notes="Six bars, one frame pivot: the 4-bar's rocker extended for a higher step, "
          "B6 on the crankpin. The page's Ver 3 and its 4-bar-to-6-bar figure.",
))
