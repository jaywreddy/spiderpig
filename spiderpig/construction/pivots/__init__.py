"""Pivot constructions on purchased metal shafts, for pillars and pins alike.

Registered in :data:`construction.AXLES` next to the printed axle and picked
with ``BuildConfig.pin`` / ``.pillar`` (``spiderpig build --pin bolt --pillar printed``).
**The default is ``--pin chicago --pillar printed``**: an M3 Chicago screw through each
pin's stack, printed stepped pillars glued into the frame plates (the pivot review's
table, the reasons and the assembly steps: :mod:`construction.pivots.chicago`; the rod
it replaced: :mod:`construction.pivots.rod`; why not ``bolt``:
:mod:`construction.pivots.bolt`). Every construction reports its links' tilt
(:mod:`construction.wobble`). ``--pin printed`` stays the zero-hardware option (on the
Strider its J7 snap lip is relieved to 0.13 mm). Pillars stay printed whichever pin is
chosen: bolt pillars don't plan on either default design (the 50 mm stock-screw bound
and a ring-filled column; 60-70 mm screws plan the Klann only at 21 layers) and rod
pillars fill the whole stack with rings.

=============  ================================================================
key            construction
=============  ================================================================
``rod``        3 mm steel rod cut to length, laser-cut spacer rings, Starlock
               push-on clips; pillars glued into the frame plates
               (:mod:`construction.pivots.rod`)
``bolt``       M3 socket head cap screw as the axle, laser-cut rings, flat
               washer and nylock nut; a pillar clamps the frame plates
               (:mod:`construction.pivots.bolt`)
``bearing``    MF63ZZ flanged ball bearing glued in every link, 3 mm rod,
               printed spacer sleeves, clips (:mod:`construction.pivots.insert`)
``bushing``    igus GFM-0304-03 flange bushing pressed in every link, same
               rod, sleeves and clips (:mod:`construction.pivots.insert`)
``chicago``    M3 Chicago screw (4 mm barrel through the stack), laser-cut
               rings, PTFE washer and DIN 988 shims, lowest link bonded to
               the barrel; pins only (:mod:`construction.pivots.chicago`)
``chicago_``   the same screw with an igus GFM-0405-03 flange bushing in
``bushing``    every link but the lowest, printed sleeves; pins only
``ptfe``       the ``rod`` with a 3 x 4 mm PTFE tube liner pressed in every
               link (:mod:`construction.pivots.ptfe`; the liner's 10 MPa is
               its limit: fine walking, a warning jammed on the test designs)
=============  ================================================================

All four state their claims through :class:`construction.axle.AxleDims`: a
rod can't neck down, so every layer between the ends is a loose spacer at
least as wide as the narrowest ring or sleeve (``fill``, ``neck``); a flange
needs a free face (``flange``); a purchased retainer is as wide as it is
(``head``, the ``ends`` hook). A layout that leaves no room for one of those
is reported by the planner as unbuildable, with the link in the way.

Research: hobbyist pivots for laser-cut walkers (3 mm sheet, M3 hardware)
==========================================================================

From the joinery notes of 2026-09-29 (``joinery.json``: ISO/DIN tables,
vendor pages, community builds) plus a check of the Starlock and E-clip
tables. Prices are hobby-market ballparks unless a page showed one; the
catalog (:mod:`hardware.parts`, :mod:`hardware.fastener_catalog`) marks
which links were verified.

(a) Screw or rod axle with spacers
    *Parts*: M3 SHCS (ISO 4762, head 5.5 x 3.0) or a 3 mm 304 rod cut from
    100 mm stock; DIN 985 nylock (5.5 AF x 4.0; ISO 10511 low: 3.9) or
    Starlock push-on clip (3 mm: OD 9.7, 0.2 thick, 1.3 high, 4 legs;
    push-on about 11 kg, pull-off about 20 kg per the Starlock table); DIN
    125 washers (3.2 x 7 x 0.5); spacers from the same sheet (3 mm rings,
    free with the plates), M3 nylon spacers (REV 3 mm, OD 4.5, 50 for
    $3.25) or steel (McMaster 92871A003).
    *Retention*: head or clip below, nut or clip above, one spacer per
    empty layer, so every link has a face against something on both sides.
    A nylock clamps links and spacers in series: tighten only snug.
    *Fits*: the thread rides in the hole (M3 major 2.874-2.98): links 3.2
    (ISO 273 fine), plates and rings 3.4 (medium); a rod: 3.2 running,
    3.15 glued (CA) in a plate. A laser hole comes out one kerf larger
    than drawn (0.15-0.2 mm), which :mod:`layout` compensates; test-cut a
    coupon. Play 0.2-0.4 mm at the joint; the holes wear.
    *Cost*: about $0.10-0.20 per joint from an M3 kit (Amazon B0DNM4HK5Q
    covers 6-30 mm) plus a nylock; a 5-pack of 100 mm rod (uxcell
    B082ZP313B, $5.49 as searched 2026-10-03) cuts a Strider's 24 pins of
    9.6-15.6 mm (266 mm; the Klann quad's 24 are 9.6 mm each),
    clips about $0.33 each from a 300-pc Starlock kit (MCMASKE, 60 per
    size, $19.99 at mcmaske.com on 2026-10-03; the 280-pc Glarks kit
    B076D3FZM4 is $12.99 for about 40 per size), so a rod pin is about
    $0.90 all in and a Strider's 24 pins (48 clips) one kit and one 5-pack:
    $25.48. *Vendors*: McMaster (91290A1xx, 93625A100, 91166A210), Amazon,
    Aspen Fasteners (nylocks, washers, verified).
    *Verdict*: what hobby builders actually do (the Make: Klann "Spiderbot"
    uses M3 button heads and nylocs; the hackaday Strandbeest "3 mm MDF and
    a ton of M3 screws and nuts"). Implemented as ``bolt`` (screw) and
    ``rod`` (rod + clips: smooth shaft, no clamping, single-use clips); the
    rod is the default pin, the screw's head being unreachable in a
    bottom-up stack (:mod:`.bolt`). The Starlock push-on / pull-off figures
    above are from a Starlock table and not verified here.

(b) Flanged ball bearing in the link, 3 mm shaft
    *Parts*: MF63ZZ 3 x 6 x 2.5 (flange 7.2 x 0.6) or F683ZZ 3 x 7 x 3
    (flange 8.1 x 0.8); a 3 mm m6 dowel or rod as the shaft (a light press
    in the 3.000 bore, so the inner race turns with the shaft); clips or a
    nylock; spacers that bear on the flange.
    *Retention*: the flange lies on one face of the link, in the next
    layer, so that layer's spacer must be shorter (printed sleeves, not
    rings) and wider than the flange; two adjacent links turn their flanges
    outwards, three in a row can't.
    *Fits*: the outer ring wants an H7 housing, which a laser can't hold;
    pressing steel into acrylic splits it (Hackaday), so cut a slip fit
    (6.00-6.03) and glue with CA or SCIGRIP 16, or a light press in plywood
    only (0.025-0.05 mm overlap, UPenn). Shaft h6/m6 (Minebea).
    *Cost*: about $0.70-1.00 each in uxcell 10-packs (Amazon B08H27NJ5N,
    B08CKJ3NMW, verified pages, prices not shown); McMaster 57155K538.
    *Verdict*: the smoothest joint; worth it at the crank, overkill on a
    slow rocking pivot. Implemented as ``bearing`` (MF63ZZ, glued).

(c) igus / bronze flange bushing
    *Parts*: igus iglide G GFM-0304-03 (3 x 4.5 x 3, flange 7.5 x 0.75,
    verified on igus.com; about $0.53 each per a TME snippet; McMaster
    2705T111), or oil-embedded bronze 6659K201 (same size, unverified).
    *Retention*: as (b); the 0.75 mm flange is a thrust washer where
    stacked plates rub.
    *Fits*: igus asks for an H7 housing (the bore then settles to E10) and
    a shaft of h9 or better; in 3 mm acrylic cut 4.50-4.52 and press (a
    polymer, it doesn't split the sheet), CA if it comes out loose.
    *Cost*: about $0.50 each. *Vendors*: igus, TME, McMaster.
    *Verdict*: the most forgiving joint (self-lubricating, shock and dirt
    tolerant) for low-speed oscillation; Klann loads suit it. Implemented as
    ``bushing``.

(d) Shoulder screws
    *Parts*: 4 mm shoulder, M3 thread (McMaster 92981A741 and family; 4-M3
    isn't in ISO 7379, whose shoulders start at 6.5 mm); a bushing per link.
    *Retention*: the shoulder length must equal the clamped stack to its
    +0/+0.25 mm tolerance, so the thread bottoms in a nut or an insert.
    *Fits*: shoulder 3.928-3.990: a 4.05-4.10 hole (bushing) runs on it.
    *Cost*: $1-2 each (unverified). *Verdict*: a 4 mm shoulder doesn't share
    parts with the 3 mm rod and its lengths don't match 3 mm layers with
    two links on a pin; not implemented.

(e) Heat-set inserts and captive nuts (frame to pillar)
    *Parts*: M3 x 5.7 brass inserts (CNC Kitchen, 100 for EUR 9.40,
    verified; hole 4.0 mm, 1.6 mm wall) for printed parts only: acrylic is
    "too brittle to be cut or deformed by screw threads" (Hackaday) and a
    laser can't make the blind hole. An M3 nut in a laser-cut hex hole is
    captive only in the plane: it retains nothing axially without glue.
    *Verdict*: the robot's frame ties already use inserts in printed
    columns (:mod:`construction.robot`); for pillars the ``bolt``
    construction clamps both plates between head and nylock instead, and
    ``rod`` / ``bearing`` / ``bushing`` glue the rod into the plates (CA on
    steel in a 3.15 mm hole) with a clip under the outer plate.

Not implemented, on purpose: E-clips (DIN 6799 size 2.3 for 3-4 mm shafts;
they need a groove nobody cuts in a 3 mm rod; listed in the catalog as
``e_clip_din6799_2p3``), 3 mm D-shaft stock (no large vendor), printed pins
under 5 mm (perimeter-only, weak: Hubs).
"""

from __future__ import annotations

from spiderpig.construction.pivots.bolt import BoltAxle
from spiderpig.construction.pivots.chicago import CHICAGO_BUSHING, ChicagoAxle
from spiderpig.construction.pivots.insert import BEARING, BUSHING
from spiderpig.construction.pivots.insert import InsertAxle as InsertAxle
from spiderpig.construction.pivots.ptfe import PtfeAxle
from spiderpig.construction.pivots.rod import RodAxle
from spiderpig.construction.pivots.standoff import STANDOFF_BENCH, StandoffAxle

PIVOTS = (RodAxle(), BoltAxle(), BEARING, BUSHING, ChicagoAxle(), CHICAGO_BUSHING,
          PtfeAxle(), StandoffAxle(), STANDOFF_BENCH)
