"""Prototype switches for the hardware-simplification study (SIMPLIFY.md).

``SPIDERPIG_HW`` is a comma-separated list of flags; each changes what one construction
buys, never the claims the planner sees (so a layering plans the same with or without):

* ``printfill``: every unclamped spacer is printed: the washers an axle carries through a
  clearance gap (PTFE + DIN 988 stacks) become one printed ring at the gap's height, and
  a Chicago screw's PTFE washer and take-up shims become one printed head spacer per end.
* ``m3``: the standoff pillars and frame ties on M3 (Hirosugi ARL-3xxBE 6 mm round
  aluminium standoffs, M3 button heads, M3 set screws, DIN 9021 M3 washers, DIN 988 3 x 6
  shims where a shim is clamped).
"""

from __future__ import annotations

import os


def flags() -> frozenset[str]:
    return frozenset(f.strip() for f in os.environ.get("SPIDERPIG_HW", "").split(",") if f.strip())


def on(name: str) -> bool:
    return name in flags()


PRINT_MIN = 0.4      # thinnest printed spacer (two 0.2 mm layers); thinner is left as play
PRINT_TOL = 0.1      # a printed spacer's height tolerance (mm)
