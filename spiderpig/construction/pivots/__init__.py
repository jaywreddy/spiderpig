"""Pivot constructions on purchased metal shafts: the pillars' and the link pins'.

Registered in :data:`construction.AXLES` and picked with ``BuildConfig.pillar`` /
``.pin``: ``--pillar standoff --pin chicago``, the only two since 2026-10-07 (the user's
decision D1: the printed, rod, bolt, PTFE-lined, bearing, bushing and spliced pivots were
removed; :data:`config.REMOVED_CONSTRUCTIONS` names each one's replacement, and the pivot
review that chose these two is in :mod:`construction.pivots.chicago`'s docstring). Every
construction reports its links' tilt (:mod:`construction.wobble`).

==================  ================================================================
key                 construction
==================  ================================================================
``standoff``        the pillar: a goBILDA 1501 round 6 mm M4 standoff where one
                    stock length fills the column, else one MISUMI NETRF6 6 mm steel
                    standoff made to its length (M3 ends), never spliced; printed
                    rings, a button head and washer through each frame plate; pillars
                    only (:mod:`construction.pivots.standoff`)
``chicago``         the pin: an M3 Chicago screw (4 mm barrel through the stack),
                    printed rings and a printed head spacer per end, lowest link
                    bonded to the barrel; pins only (:mod:`construction.pivots.chicago`)
==================  ================================================================

Every pivot's spacer rings and gap rings are printed (:func:`common.gap_washers`): an
unclamped spacer only sets play, so it needn't be bought; clamped shims are steel
(:data:`hardware.bom.SHIM_AS`).

Every one states its claims through :class:`construction.axle.AxleDims`: a shaft can't
neck down, so every layer between the ends is a loose spacer at least as wide as the
narrowest ring (``fill``, ``neck``); a purchased retainer is as wide as it is (``head``,
the ``ends`` hook, ``end_h``). A layout that leaves no room for one of those is reported by
the planner as unbuildable, with the link in the way.
"""

from __future__ import annotations

from spiderpig.construction.pivots.chicago import ChicagoAxle
from spiderpig.construction.pivots.standoff import StandoffAxle

PIVOTS = (ChicagoAxle(), StandoffAxle())
