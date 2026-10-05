"""The simplified hardware (the study of 2026-10-05), on by default.

``SPIDERPIG_HW`` is a comma-separated list of flags (default :data:`DEFAULT`; set it empty
for the hardware before the study); each changes what one construction buys, never the
claims the planner sees (so a layering plans the same with or without):

* ``printfill``: every unclamped spacer is printed: the washers an axle carries through a
  clearance gap (PTFE + DIN 988 stacks) become one printed ring at the gap's height, and
  a Chicago screw's PTFE washer and take-up shims become one printed head spacer per end.
* ``m3``: the frame ties and the crank's journal stub on uxcell's 6 mm round M3 aluminium
  standoffs, M3 button heads and set screws, DIN 9021 M3 washers (``--pillar standoff_m3``
  puts the pillars on them too; the default pillar stays goBILDA M4: on uxcell's coarse
  lengths a pillar splices inside a loaded span, jam SF 1.62).
* ``lengths``: fewer screw lengths (the deck's screws button heads, M3 x 10 -> x 8).
* ``oneshim``: clamped shims only in 1.0 mm and :data:`construction.pivots.standoff.SHIM_STEP`
  (0.5 mm) steps, bought as DIN 433 washers (:data:`hardware.bom.SHIM_AS`).
"""

from __future__ import annotations

import os

DEFAULT = "printfill,m3,lengths,oneshim"


def flags() -> frozenset[str]:
    return frozenset(f.strip() for f in os.environ.get("SPIDERPIG_HW", DEFAULT).split(",")
                     if f.strip())


def on(name: str) -> bool:
    return name in flags()


PRINT_MIN = 0.4      # thinnest printed spacer (two 0.2 mm layers); thinner is left as play
PRINT_TOL = 0.1      # a printed spacer's height tolerance (mm)
