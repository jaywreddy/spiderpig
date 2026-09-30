"""TrotBot (Wade and Ben Vagle): an 8-bar leg, plus its heel and retractable toe.

Numbers are the diywalkers plan's drawing units (``unit`` mm each), bar
names as on its "bar numbers for simulator" map, joints as on its joint map
(``J0`` is the crank centre ``O``, at the plan's (0, 10)):

* the frame pivot for the support rod ``J3`` sits 7 left of and 6 above the
  crank centre; the crank ``B0`` is 4;
* the rocker ``B2`` + ``B3`` (8 + 2) turns about ``J3``: ``J2`` at one end,
  ``J4`` at the other;
* the crank rider is a rigid triangle: ``B1`` (6) from the crankpin ``J1``
  to ``J2``, straight on 1 (``B10``) past ``J1`` to ``J9``, and ``B7`` (3)
  to ``J8``, with ``J2``-``J8`` 7.55 (the dashed ``B15``: the plan's
  109.475 degrees at ``J1``; its 90-degree DIY drawing, ``B7`` = 2.83 square
  to ``B10`` at ``J9``, is the same triangle to 0.01);
* ``B6`` (11) from the crankpin to ``J5`` on the bar ``B4`` + ``B5`` (6 + 2)
  hanging from ``J4``; ``B9`` (8) from its end ``J6`` down to the main foot
  ``J7``, which ``B8`` (9) ties back to ``J8``.

The heel (TrotBot Ver 1) adds ``B13`` (7.2) from ``J9`` and the heel bar
``B12`` + ``B14`` (2.55 + 1) from ``J10``, 2.64 (``B11``) up ``B8`` from the
foot; ``J12`` is the heel. The retractable toe (Ver 3) extends the heel bar
1 past ``J10`` (``J14``) and hangs the toe tip ``J15`` from it (3.7) and from
the middle of ``B9`` (``J13``, 4 from ``J6``; 3.8). The toe's joints carry no
number on the plan; ``J13``-``J15`` are ours. Branches are the site's Python
simulator's (``TrotBot Stationary.py``): ``J2`` high, ``J5`` low, ``J8``
left, ``J7`` low, ``J11`` low; the program matches it to 1e-13.

**Fabrication.** ``B8`` and the heel link ``B13`` hang off the crank
triangle at ``J8`` and ``J9``, 3 and 1 from the crankpin: inside the crank
circle (4). Seen from either link, O circles its pin once per crank turn, so
the link sweeps across the crank axis whatever its shape. The crank router
(:mod:`construction.route`) runs the crankshaft along the crankpin's post
through such a link's layer, and the link must clear that post. ``B13``
passes it 6.8 mm off at the drawing's 7 mm unit, under the 10 mm a post
needs (3 post radius + 6 link half-width + 1 margin): that clearance sets
the family's scale, 10.5 mm per unit (x1.5; x1.47 is the least that clears
it).
"""

from __future__ import annotations

import sympy as sp

from spiderpig.linkage import Linkage, P, circle_x_circle, crank, register, xy

R = sp.Rational
SOURCE = "https://www.diywalkers.com/trotbot-linkage-plans.html"

BASE = {
    # mm per drawing unit (crank 42 mm), one scale for the family: the heel
    # link B13 sweeps across O and passes the crankpin's post 6.8 mm off at
    # unit 7, under the 10 it needs (3 post radius + 6 link half-width + 1
    # margin); 10.5 is the least half-unit step that clears it (x1.47 would).
    "unit": R(21, 2),
    "frame_x": R(7),      # frame pivot J3 at (-frame_x, frame_y) from the crank centre
    "frame_y": R(6),
    "B0": R(4),           # crank
    "B1": R(6),           # J1 - J2
    "B2": R(8),           # J2 - J3
    "B3": R(2),           # J3 - J4 (B2 extended past the frame pivot)
    "B4": R(6),           # J4 - J5
    "B5": R(2),           # J5 - J6 (B4 extended)
    "B6": R(11),          # J1 - J5
    "B7": R(3),           # J1 - J8
    "B8": R(9),           # J8 - J7
    "B9": R(8),           # J6 - J7
    "B10": R(1),          # J1 - J9 (B1 extended past the crankpin)
    "B15": R(755, 100),   # J2 - J8 (the triangle's dashed side)
}
HEEL = {
    "B11": R(264, 100),   # J7 - J10, along B8 (B8 = 6.36 + 2.64)
    "B12": R(255, 100),   # J10 - J11
    "B13": R(72, 10),     # J9 - J11
    "B14": R(1),          # J11 - J12 (B12 extended: the heel)
}
TOE = {
    "toe_pivot": R(4),        # J6 - J13, along B9 (B9 = 4 + 4)
    "heel_ext": R(1),         # J10 - J14 (the heel bar extended past J10)
    "toe_front": R(38, 10),   # J13 - J15
    "toe_back": R(37, 10),    # J14 - J15
}


def _on_bar(frm, to, dist, length):
    """The point ``dist`` from ``frm`` on the bar frm -> to of known ``length``.

    Beyond ``to`` when ``dist > length``. The same point as
    :func:`linkage.extend` / :func:`linkage.offset` give, but the bar's
    length is a parameter instead of a square root of the program so far:
    the heel version compiles in about 11 s instead of 4 minutes.
    """
    return frm + (to - frm) * (dist / length)


def _program(heel: bool, toe: bool):
    def program(p):
        u = p["unit"]
        steps = [
            ("O", xy(0, 0)),
            ("J3", xy(-p["frame_x"] * u, p["frame_y"] * u)),
            ("J1", crank(p["B0"] * u)),
            ("J2", circle_x_circle(P("J1"), p["B1"] * u, P("J3"), p["B2"] * u, -1)),
            ("J4", _on_bar(P("J2"), P("J3"), p["B2"] + p["B3"], p["B2"])),
            ("J5", circle_x_circle(P("J1"), p["B6"] * u, P("J4"), p["B4"] * u, 1)),
            ("J6", _on_bar(P("J4"), P("J5"), p["B4"] + p["B5"], p["B4"])),
            ("J9", _on_bar(P("J2"), P("J1"), p["B1"] + p["B10"], p["B1"])),
            ("J8", circle_x_circle(P("J1"), p["B7"] * u, P("J2"), p["B15"] * u, 1)),
            ("J7", circle_x_circle(P("J8"), p["B8"] * u, P("J6"), p["B9"] * u, 1)),
        ]
        if heel:
            steps += [
                ("J10", _on_bar(P("J7"), P("J8"), p["B11"], p["B8"])),
                ("J11", circle_x_circle(P("J9"), p["B13"] * u, P("J10"), p["B12"] * u, 1)),
                ("J12", _on_bar(P("J10"), P("J11"), p["B12"] + p["B14"], p["B12"])),
            ]
        if toe:
            steps += [
                ("J13", _on_bar(P("J6"), P("J7"), p["toe_pivot"], p["B9"])),
                ("J14", _on_bar(P("J11"), P("J10"), p["B12"] + p["heel_ext"], p["B12"])),
                ("J15", circle_x_circle(P("J13"), p["toe_front"] * u,
                                        P("J14"), p["toe_back"] * u, -1)),
            ]
        return steps

    return program


LABELS = {
    "b1": "crank triangle B1+B10, B7 (J2-J1-J9, J8)", "b2": "rocker B2+B3 (J2-J3-J4)",
    "b3": "B4+B5 (J4-J5-J6)", "b4": "B6 (J1-J5)", "b5": "B9 (J6-J7)", "b6": "B8 (J8-J7)",
    "b7": "heel link B13 (J9-J11)", "b8": "heel bar B12+B14 (J10-J11-J12)",
    "b9": "toe link (J13-J15)", "b10": "toe strut (J14-J15)",
}


def _links(heel: bool, toe: bool):
    links = {
        "b1": (("J2", "J1", "J9", "J8"), (("J2", "J9"), ("J1", "J8"))),
        "b2": (("J2", "J3", "J4"), (("J2", "J4"),)),
        "b3": (("J4", "J5", "J6"), (("J4", "J6"),)),
        "b4": (("J1", "J5"), (("J1", "J5"),)),
        "b5": (("J6", "J13", "J7") if toe else ("J6", "J7"), (("J6", "J7"),)),
        "b6": (("J8", "J10", "J7") if heel else ("J8", "J7"), (("J8", "J7"),)),
    }
    if heel:
        links["b7"] = (("J9", "J11"), (("J9", "J11"),))
        end = "J14" if toe else "J10"
        links["b8"] = ((("J14",) if toe else ()) + ("J10", "J11", "J12"), ((end, "J12"),))
    if toe:
        links["b9"] = (("J13", "J15"), (("J13", "J15"),))
        links["b10"] = (("J14", "J15"), (("J14", "J15"),))
    return links


def _trotbot(key: str, name: str, heel: bool, toe: bool, notes: str) -> Linkage:
    links = _links(heel, toe)
    feet = (("b6", "J7"),) + ((("b8", "J12"),) if heel else ()) + ((("b9", "J15"),) if toe else ())
    return Linkage(
        key=key,
        name=name,
        family="trotbot",
        params={**BASE, **(HEEL if heel else {}), **(TOE if toe else {})},
        program=_program(heel, toe),
        links=links,
        labels={b: LABELS[b] for b in links},
        frame=("J3", "O"),
        crank=("O", "J1"),
        feet=feet,
        source=SOURCE,
        notes=notes,
    )


TROTBOT = register(_trotbot(
    "trotbot", "TrotBot", heel=False, toe=False,
    notes="Eight bars, one frame pivot; a tear-drop foot path with a high step.",
))
TROTBOT_HEEL = register(_trotbot(
    "trotbot_heel", "TrotBot with heel (Ver 1)", heel=True, toe=False,
    notes="Ten bars: a heel takes the weight while the main foot is still descending.",
))
TROTBOT_TOE = register(_trotbot(
    "trotbot_toe", "TrotBot with heel and retractable toe (Ver 3)", heel=True, toe=True,
    notes="Twelve bars: heel, plus a toe that paws backward and stays folded as the leg "
          "lifts.",
))
