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
``chicago6``        the same with an M6 Chicago screw (8 mm barrel): a decanting
                    cradle's pins until 2026-10-13
``pivot8``          an 8 mm hardened steel pivot pin (MISUMI CCGH8) made to its stack,
                    an E-ring at each end, on the Chicago screw's column: a decanting
                    cradle's pins since 2026-10-13 (``config.PRODUCT_PINS``,
                    :mod:`construction.pivots.pivot_pin`)
``clevis8``         an 8 mm 18-8 clevis pin, its head inside the stack over a printed
                    spacer, a hairpin cotter through its hole from the bay: a decanting
                    cradle's quick-release pins (B's X and Y since 2026-10-13,
                    ``config.PRODUCT_PIN_JOINTS``, :mod:`construction.pivots.clevis`)
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

from spiderpig.construction.pivots.chicago import CHICAGO_M6, ChicagoAxle
from spiderpig.construction.pivots.clevis import CLEVIS8
from spiderpig.construction.pivots.pivot_pin import PIVOT8
from spiderpig.construction.pivots.standoff import StandoffAxle

PIVOTS = (ChicagoAxle(), StandoffAxle(), CHICAGO_M6, PIVOT8, CLEVIS8)
