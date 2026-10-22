"""Pivot constructions on purchased metal shafts: the pillars' and the link pins'.

Registered in :data:`construction.AXLES` and picked with ``BuildConfig.pillar`` /
``.pin``: ``--pillar standoff --pin chicago`` by default; the ball-bearing pivots
``ball`` / ``ball_pillar`` since 2026-10-11 (the robot-arm workstream). The user's
decision D1 of 2026-10-07 removed the printed, rod, bolt, PTFE-lined, bearing, bushing and
spliced pivots (:data:`config.REMOVED_CONSTRUCTIONS` names each one's replacement; the
pivot review that chose chicago and standoff is in :mod:`construction.pivots.chicago`). Every
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
``rod516``          a 5/16 in 6061-T6 rod pin cut to its stack, an M3 button head and
                    a wide washer on each tapped end, on the pivot pin's column: a
                    decanting cradle's pins since 2026-10-21 (the materials rule,
                    :mod:`construction.pivots.rod_pin`)
``rod38``           the same in 3/8 in: B's rod pin P, A's E
``rodclip516``      a 5/16 in 6061-T6 rod pin, its head an M3 button head and washer,
                    a hairpin cotter through its drilled hole, on the clevis pin's
                    column: B's X and Y since 2026-10-21
``ball``            the Chicago pin with an MR84ZZ ball bearing bonded into every link
                    but the host (:mod:`construction.pivots.ball`, 2026-10-11)
``ball_pillar``     the standoff pillar with an MR106ZZ bearing bonded into every link on
                    it (link_radius 6.6 mm)
``ball_pair``       ``ball`` with a preloaded pair in every link thick enough for two, the
                    column clamped by a disc spring through the inner rings
                    (:mod:`construction.pivots.pair`, 2026-10-12)
``ball_pair_pillar`` ``ball_pillar`` likewise
==================  ================================================================

Every pivot's spacer rings and gap rings are printed, one sleeve per run of them between
the parts the axle retains (the sleeve rule of 2026-10-14, :func:`common.sleeve_runs`,
:func:`common.build_sleeves`): an unclamped spacer only sets play, so it needn't be bought;
clamped shims are steel (:data:`hardware.bom.SHIM_AS`). Where a construction declares its
stock spacers (``spacers``, :class:`common.Stock`: a walker's ``chicago`` and ``standoff``,
the arm's pivots, since 2026-10-14) :meth:`construction.axle.AxleGroup.realize` makes them
stock tubes and shims instead, but on a product, whose printed rings fill their layers (a
decanting cradle's: :func:`construction.axle.full_rings`). What an axle prints and the stock
part it buys at a plan's z (``prints``, ``stock_keys``) are the planner's ``tally``.

Every one states its claims through :class:`construction.axle.AxleDims`: a shaft can't
neck down, so every layer between the ends is a loose spacer at least as wide as the
narrowest ring (``fill``, ``neck``); a purchased retainer is as wide as it is (``head``,
the ``ends`` hook, ``end_h``). A layout that leaves no room for one of those is reported by
the planner as unbuildable, with the link in the way.
"""

from __future__ import annotations

from spiderpig.construction.pivots.ball import BallAxle, BallPillar
from spiderpig.construction.pivots.chicago import CHICAGO_M6, ChicagoAxle
from spiderpig.construction.pivots.clevis import CLEVIS8
from spiderpig.construction.pivots.pair import BallPairAxle, BallPairPillar
from spiderpig.construction.pivots.pivot_pin import PIVOT8
from spiderpig.construction.pivots.rod_pin import ROD38, ROD516, RODCLIP516
from spiderpig.construction.pivots.standoff import StandoffAxle

PIVOTS = (ChicagoAxle(), StandoffAxle(), BallAxle(), BallPillar(), BallPairAxle(),
          BallPairPillar(), CHICAGO_M6, PIVOT8, CLEVIS8, ROD516, ROD38, RODCLIP516)
