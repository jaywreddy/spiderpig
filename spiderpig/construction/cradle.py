"""A decanting cradle: two side stacks, the cradle box between them, a stand and a bottle.

A cradle linkage (:mod:`linkages.decanters`, ``Linkage.product == "cradle"``) is planned
one side at a time like any mechanism. The bottle lies between two sides, the **bay**, so
:func:`assemble_cradle` makes the cradle from that one side:

* **The drive side** (``L.``, the side as planned; on your right as you stand behind the
  cradle facing the decanter) carries the hand crank (:mod:`construction.handcrank`).
* **The passive side** (``R.``) is the drive side's own parts mirrored, without the crank,
  the rod and the rod's pin (:func:`passive_side`): the cradle box drives it; its plates
  keep the drive's holes, so each plate pair is one part, flipped.
* **The cradle box**: four tie rods (1/2 in 6061-T6511 rounds made to length, faced and tapped
  M6 at home) between the two cradle links, at the linkage's tie points (``decanters.TIES``), each
  screwed through its cradle link from outside. Two printed saddles ride on them, each a
  carrier on its two rods (an M3 set screw up from its underside onto each) and a V block
  that rides up and down in it on a thumbscrew (:class:`Saddle`): the front one under the
  neck, in a box guide; the rear one under the body, on a steel post beside a coarse lead
  screw (its knob drops onto teeth that lock it), the base stop's rails continuing its V
  behind it on their flanges. TPU liners, pegged in, line the Vs. At a 6 L methuselah's
  setting each V block sits on its tie rods.
* **The bottles** (:func:`setting`, :func:`seat`): a 750 ml to a 6 L, each with its mouth
  on the pivot and its axis level, set by their scales with the bottle out, from its own
  measures: the two knobs and the base stop. The **lip gauge** is fixed at the mouth, its
  pad over the rim's top (a bottle is uncorked before it goes in): it checks where the
  mouth is and holds the bottle from sliding forward. The **base stop**'s gate rides the
  rear V block's flats and the
  rails behind it, clamped by hand behind the bottle's base: it holds the bottle from
  sliding back.
* **The light**: an AAA keychain light standing in a clip printed with the front saddle,
  its head under the shoulder (the sommelier's candle; its head twists it on).
* **The stand** (:data:`STANDS`, :class:`CradleStand`): each side's frame plates are the
  stand's members, not arms from O. The outer plate carries the hinge, the gearbox and the
  legs down to a rail. The inner plate holds the pillars and stiffens the side, its posts
  off one line so it holds their ends (A: a block round the hinge and a leg down the front
  to a column on the rail, six columns; B: the bar O1-O2, a triangle and a leg off it and a
  bottom chord to two columns on the rail, four columns), so the tie rods pass it in the
  open.
* **The base board** (:func:`_base_plate`): both sides' rails bolted onto it with printed
  brackets (:func:`_bracket`: thumb knobs, no tool), each rail between two birch cleats
  glued to it (:func:`_cleats`), the decanter's printed riser (:func:`riser_pieces`: a fixed
  height per decanter, :data:`RISER_CLEAR`) in a locator in a notch from its front edge
  (:func:`_riser_locator`), a window under the base stop (:data:`BOARD_WINDOW`), printed TPU
  feet snapped into its corners; B's sits toward its drive side (:data:`BASE_Z`).
* **Display only** (``fab="display"``: drawn, never bought, cut or printed): the bottle (a
  6 L methuselah, :data:`SHOWN`: the V blocks drawn at its setting, their lowest) and a
  decanter under the mouth, on the riser.

The planner sees the side-level parts as claims: the ties' columns through the layers
between the cradle link and the inner plate (:class:`CradleTies`), the stand's columns and
base ties through every layer (:class:`CradleStand`). What it can't see, the tie rods
passing the inner plate's layer and the bottle in the bay, is measured on the assembled
cradle over the swing (:func:`construction.contract.clashes`, by
:mod:`spiderpig.cradle_report` and ``tests/test_cradle.py``).
"""

from __future__ import annotations

import itertools
import math
import sys
import threading
import types
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from typing import cast

import numpy as np
from build123d import Axis as BAxis
from build123d import (
    Box,
    Cylinder,
    Face,
    Helix,
    Line,
    Location,
    Plane,
    Polyline,
    Solid,
    Wire,
    sweep,
)

from spiderpig.construction.base import (
    FRAME_INNER,
    FRAME_OUTER,
    RIDES_HOST,
    Build,
    ConstructionError,
    Context,
    Group,
    Motion,
    Realized,
    hardware,
)
from spiderpig.construction.plates import FramePlates, boss_web
from spiderpig.construction.robot import prefixed
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.fasteners import BUTTON_A2_70_NM, CLEARANCE, Screw, screw
from spiderpig.mechanism import Body, Mechanism
from spiderpig.shapes import (
    Cut,
    Rect,
    Shape3D,
    cut_holes,
    difference,
    disc,
    intersection,
    moved,
    pill,
    pills,
    union,
)
from spiderpig.stack import Claim, Disc, Layout, Placed, Unbuildable, body_class

LIP_HEIGHT = 550.0      # the bottle's mouth over the table (mm): at rest, 50° neck up, the
#                         base stop (set for a 6 L) hangs about 507 mm under it, 26 mm
#                         over the base board's top (17 mm up)
RISER_CLEAR = (20.0, 10.0)
"""The one rule for a vessel's height under the mouth (design review round 2, 2026-10-14):
its rim at least this far (mm) under the lowest part of the cradle box over it over the whole
swing (the light's printed clip at rest: :func:`spiderpig.cradle_report.room`'s ``lowest``),
and every point of it this far from the room's side (the moving box beside it, the bay's
faces). Rest is a toggle dwell: what the box comes down onto there it crushes with a force no
clutch bounds, so the clearance is built into the riser, not left to a ruler. The cradle page
(``viewer/src/decant/liquid.ts``, ``riserFits``), the guide and ``DECANTER.md`` give one stack
per decanter by it (:func:`spiderpig.cradle_report.riser_for`)."""
RISER_BASE = 100.0      # the printed riser's footprint (a 100 x 100 mm square) and the
#                         locator's pocket for it (RISER_PLAY each side)
RISER_PLAY = 0.5
RISER_BLOCK = (49.8, 85.0)
"""The riser's two blocks, side by side (each ``RISER_BASE`` along x by this wide across,
this tall: a printed cup, open underneath), each on its lift (:data:`RISER_LIFT`): also the
drive side panel's lay-flat blocks (the trip's breakdown, :func:`module_steps`), a block on
its lift standing as far as the gearbox does off its outer plate, so the panel lies level on
them and the gearbox."""
RISER_LIFT = 27.2
"""The riser's two lifts (design review round 2's drive, 2026-10-15): half plates this tall
under the blocks, each a block's footprint (a cup open underneath, its boss up inside its
block). Round 2's NRV050 stands 114.7 mm off the outer plate where the NRV040 stood 87.5, so a
block on its lift stands 114.2 with its boss (:data:`RISER_LIP`), half a millimetre under the
gearbox as the NRV040's blocks had stood. Taller blocks instead would have taken the whole
riser past the drawn decanter's rule, or left some listed vessels no stack that clears the
locator (the plates alone stop under it)."""
RISER_PLATES = (5.0, 4.0)       # its printed plates over the blocks, bottom up (mm): the whole
#                                 set the drawn decanter's (DECANTER_RISER) over the lifts (20,
#                                 10 and 5 before them)
RISER_LIP = 2.0         # each piece's boss on top (45°), into the recess under the next
RISER_WALL = 3.0        # the cups' walls and top
DECANTER_RISER = RISER_LIFT + RISER_BLOCK[1] + sum(RISER_PLATES) + RISER_LIP
"""The drawn decanter's riser (a RIEDEL Ultra, :data:`DECANTER_PROFILE`): the whole printed
set, 123.2 mm, its rim 31.8 mm under the light's clip; the most :data:`RISER_CLEAR` allows
it is 129 (``tests/test_cradle.py`` holds it). Any other stack is some of the lifts, the
blocks and the plates in that order (:func:`riser_stacks`)."""
DECANTER_PROFILE = ((0.0, 28.6), (6.3, 38.8), (21.7, 84.2), (30.0, 95.8), (40.7, 102.5),
                    (47.4, 100.8), (55.4, 91.2), (72.4, 52.4), (79.5, 41.3), (89.5, 32.3),
                    (108.5, 22.9), (133.5, 18.3), (165.9, 19.0), (189.9, 23.9), (225.0, 37.1))
"""The decanter drawn (display): a RIEDEL Ultra 2400/14, 750 ml, 225 mm tall, 205 across
(https://www.riedel.com/en-gb/riedel-ultra-decanter/240000014), its outline (height,
radius) traced from RIEDEL's drawing (the vessel study of 2026-10-10, ``DECANTER.md``)."""
BAY = 260.0            # between the two inner plates' bay faces: a 174 mm bottle, and a
#                         decanter up to 240 mm across stands wholly between the side stacks
TIE_D = 12.7            # the cradle box's tie rods: 1/2 in 6061-T6511 round, M6 x 12 deep each
#                         end (hardware.cradle_catalog.al_rod; the simplification round 1,
#                         2026-10-21, the materials rule: MISUMI's 12 mm NETRF12 steel since
#                         design review round 2's M1, 2026-10-15: the passive half's moment
#                         crosses the bay in them racking, and a NETRF10's M5 end joints worked
#                         at their gapping moment walking, its tapped ends yielded jammed:
#                         cradle_report.box_racking). Their centres stayed (decanters.TIES), so
#                         their tops stand 0.35 mm higher than the 12 mm rods'
TIE_THREAD = 12.0
M5 = screw("bhcs", "5")
M6 = screw("bhcs", "6")
M6_109 = screw("bhcs109", "6")  # an M6 ISO 7380 button head in class 10.9 steel
TIE_M = M6_109          # the tie rods' end screws: M6 x 16 button heads, class 10.9 (ISO 7380,
#                         ISO 898-1; A2-70 until 2026-10-20: at ISO 7380's 4 N·m in A2-70 their
#                         joints stood at their gap walking, 0.98 of it, cradle_report's
#                         OPEN_JOINTS). The same 3.3 mm head under the cradle link: a full-
#                         strength A2-70 head (an ISO 4017 hex's 4.15, an ISO 4762's 6) needs a
#                         clearance gap over the planner's 4 mm there, a layer that neither
#                         cradle's plan has (B's pin K's clevis pin would span 19.1 mm)
TIE_ROD_SCREW = 16.0    # through the cradle link's 4.75 mm: 11.25 mm in the rod's 12 mm tap
TIE_ROD_HOLE = 6.4      # their clearance holes in the cradle link (ISO 273 fine)
TIE_TIGHTEN_NM = 6.0    # the tie rods' screws tightened (the guide), and STEEL_FEET's: 3/4 of
#                         ISO 7380's most in 10.9 (fasteners.BUTTON_10_9_NM, 8), 3.8-7.0 kN of
#                         preload (fasteners.preload_band), the end joint closed to 7.3 N·m at the
#                         least (cradle_report.box_racking), at the most 52 % of what its head
#                         carries (BUTTON_10_9_PROOF_N); 8 would put 9.3 kN on the 5052 under
#                         a foot's washer and into the rods' mild-steel taps for no joint that
#                         needs it. (6 N·m on A2-70 button heads until design review round 3;
#                         4 until 2026-10-20)
STEEL_FEET: dict[str, tuple[str, ...]] = {}
"""The pillars whose foot (its end on the outer plate) takes the tie rods' class 10.9 button head
at :data:`TIE_TIGHTEN_NM` in both sides, not an A2-70 one at ISO 7380's 4 N·m: none since the
simplification round 1 (2026-10-21, the materials rule). A's pillar G, whose foot had lifted
walking at the A2-70 head's least preload on its 12 mm steel post (1.06 x its decompression,
2026-10-20), stays shut on the 1/2 in 6061 one (``cradle_report.post_joints``). The tie rods'
heads stay class 10.9 (:data:`TIE_M`): outside the bay the wine runs in (on the cradle links'
outer faces), unplated (an electroplated 10.9 risks hydrogen embrittlement), greased or
threadlocked into the aluminium rods."""
SMALL_TIE_D = 10.0      # a 10 mm standoff (MISUMI NETRF10, M5 x 10 deep each end): a base or
#                         bay tie, a 10 mm column
TIE_SCREW = 12.0        # M5 x 12 button heads through a 3.2-4.8 mm plate: 7.2-8.8 mm in the tie
TIE_HOLE = 5.5          # an M5 screw's clearance hole (ISO 273 medium): a base or bay tie's, a
#                         foot's, a 10 mm column's
M5_NUT = (8.0, 4.0)     # ISO 4032 M5 nut: across flats, height
M5_PITCH = 0.8          # its thread's pitch (ISO 261, coarse): a stud knob's turn
LINER = 2.0            # the V blocks' TPU liners
FRAME_COLOR = "#eb6834"
SADDLE_COLOR = "#6a4fc7"
VEE_COLOR = "#8a6fe0"
KNOB_COLOR = "#d9a521"
LINER_COLOR = "#2b2b2b"
TIE_COLOR = "#4a4a4a"
BOTTLE_COLOR = "#20402a"
DECANTER_COLOR = "#cfe3f5"
LED_COLOR = "#f0f0e0"
BOARD_COLOR = "#9aa3ad"
RISER_COLOR = "#3a3f45"


# --- the bottles (display) and what the saddles are sized for ----------------------------

@dataclass(frozen=True)
class BottleSize:
    """A bottle's outline (mm): overall length, body radius, neck radius, the neck's length
    from the mouth, where the shoulder reaches the body."""

    key: str
    name: str
    length: float
    r: float
    r_neck: float
    neck: float
    shoulder: float
    mass_full_kg: float
    com_from_mouth: float        # its centre of mass, full, along the axis from the mouth
    outline: tuple[tuple[float, float], ...] = ()   # a maker's drawing, traced (x, r): drawn
    #                                                 as it is, not the model's cosine

    def profile(self) -> list[tuple[float, float]]:
        """(x along the axis from the mouth, radius) round the outline: the maker's drawing
        traced (:attr:`outline`, design review round 3, mounting M1 and M2: a real shoulder
        flares earlier and fuller than a cosine), or a cylinder neck, a quarter-cosine
        shoulder and a cylinder body."""
        if self.outline:
            return [(float(x), float(r)) for x, r in self.outline]
        pts = [(0.0, self.r_neck), (self.neck, self.r_neck)]
        n = 8
        for i in range(1, n + 1):           # the shoulder: a quarter cosine
            s = i / n
            x = self.neck + s * (self.shoulder - self.neck)
            pts.append((x, self.r_neck + (self.r - self.r_neck) * (1 - math.cos(s * math.pi)) / 2))
        pts.append((self.length, self.r))
        return pts


SAVERGLASS_4427 = (
    (0, 12.43), (2, 13.44), (5, 13.44), (7, 14.46), (10, 14.71), (15, 14.71), (17, 13.70),
    (20, 13.70), (43, 14.20), (72, 15.47), (75, 16.49), (78, 18.26), (87, 27.92), (94, 32.89),
    (101, 36.02), (108, 37.79), (112, 38.30), (300.5, 38.30))
"""Saverglass's Bordelaise Tradition Premium 75 cl (ref. 4427), its drawing traced (design
review round 3's mounting review, 2026-10-20: 1 mm steps, checked against the sheet's body
diameter and ring; Douglas-Peucker to 0.25 mm): its shoulder flares from 71 mm, where the
model's cosine had it straight to 80."""
UNIVERRE_NGBD06QFR = (
    (0, 20.84), (3, 21.71), (6, 21.91), (8, 23.61), (9, 23.73), (22, 23.73), (24, 21.71),
    (30, 21.71), (81, 24.02), (85, 24.60), (92, 26.39), (102, 30.87), (108, 34.83),
    (112, 38.24), (116, 42.52), (123, 52.40), (129, 59.55), (136, 65.95), (143, 70.91),
    (152, 75.54), (161, 79.02), (170, 81.04), (175, 81.62), (184, 82.20), (215, 82.20),
    (217, 81.91), (243, 81.91), (244, 81.62), (271, 81.62), (272, 81.33), (477, 79.31),
    (487, 78.44), (490, 77.48), (493, 74.96))
"""univerre's 6 L Bordeaux (ref. NGBD06QFR), its drawing traced as :data:`SAVERGLASS_4427`
(round 3's mounting M2): its shoulder flares from about 85 mm (r 33 at 105, 48 at 120, 60 at
130: the model's cosine had 24.8 at 122.5), and its body tapers 82.2 -> 79.3 mm toward its
base."""

STANDARD = BottleSize("standard", "750 ml Bordeaux", 300.5, 38.3, 15.0, 80.0, 115.0,
                      1.3, 192.0, SAVERGLASS_4427)
MAGNUM = BottleSize("magnum", "1.5 L Bordeaux magnum", 356.0, 49.5, 15.5, 85.0, 150.0,
                    2.6, 230.0)
JEROBOAM = BottleSize("jeroboam", "3 L Champagne jeroboam", 470.0, 70.5, 19.0, 105.0, 190.0,
                      5.9, 325.0)
IMPERIAL = BottleSize("imperial", "6 L Bordeaux impériale", 493.0, 82.3, 23.5, 115.0, 195.0,
                      9.1, 319.0, UNIVERRE_NGBD06QFR)
METHUSELAH = BottleSize("methuselah", "6 L Champagne methuselah", 571.0, 87.0, 24.0, 128.0,
                        231.0, 11.1, 390.0)
SIZES = (STANDARD, MAGNUM, JEROBOAM, IMPERIAL, METHUSELAH)
"""What the cradle is built for (``docs/agentlib/DECANTER.md``, Requirements): a standard
750 ml Bordeaux (Saverglass Bordelaise Trad. Premium, ref. 4427: 300.5 x 76.6 mm, 550 g;
its shoulder flaring from 71 mm, drawn as traced, :data:`SAVERGLASS_4427`; the Bordelaise moulds
run 300.5-315 mm long: Saverglass's 1855, Icon and Maestria), a Bordeaux magnum (univerre:
356 x 99 mm, 1.04 kg empty), a Champagne jeroboam (Saverglass Jero Vignoble, ref. 7201:
470.3 x 140.8 mm, 2.9 kg empty; their Standard 300 cl, ref. 0718, is 474.3 x 135.2 mm and
3.03 kg, univerre's NGJEC03Q 475 x 133: no bottle's length is a size's, so the scales set
any bottle by its own measures, :func:`setting`), and the
two 6 L shapes (2026-10-13, the user's "both"): a Bordeaux impériale (Saverglass
Bordelaise Tradition 600 cl, ref. 0714: 492.8 mm tall, 164.5 mm at the shoulder, 156.5 at
the base, 3.12 kg empty; univerre's NGBD06QFR 493.1 x 164.4, 2.95 kg) and a Champagne
methuselah, the largest and heaviest (Saverglass Standard 600 cl, ref. 1742: 571 mm tall,
174 mm at the shoulder, 170 at the base, 5.13 kg empty). Full mass: the glass plus 0.75,
1.5, 3 or 6 kg of wine (``cradle_report.WINE_KG``). The impériale is drawn as univerre draws
it (:data:`UNIVERRE_NGBD06QFR`; design review round 3, mounting M2: its shoulder is 27 mm
fuller at the light than the cosine had it); the methuselah's neck (radius, length to the
shoulder) is still an estimate scaled from the jeroboam's (Saverglass's 1742 sheet gives its
overall sizes only): the front knob is set to the neck in hand, and the light clears a
shoulder as full as the impériale's (:func:`bottle_room`).

The centre of mass, full (``com_from_mouth``; design review round 1, 2026-10-14: it had been
42 % of the length from the base, too far forward), from a model of each bottle: a glass
shell over its :meth:`BottleSize.profile`, the neck's wall 1.5 x the body's and the base 3 x
thick under a punt (a cone 0.8 r across and 0.35 r deep), the wall's thickness solved so the
glass weighs the bottle's empty mass (2.5 g/cm³: 2.9 mm for the 750 ml to 6.3 mm for the
methuselah), and the wine (0.995 g/cm³) filled from the base, upright and sealed (the
headspace in the neck), to its volume. It puts a full bottle's centre 31-36 % of its length
up from the base: a 750 ml's about 11 cm up its 30 cm, the usual figure. Along the axis from
the mouth: 192, 230, 325, 319 and 390 mm (the magnum's and the impériale's wine fills the
whole of their drawn outline: there the wine's centre is the inside's). Weigh a bottle on a
knife edge to refine one.

Checked against the makers' pages rendered 2026-10-14 (design review round 2, N2): each
size is its cited bottle to 0.5 mm and 0.05 kg (the 750 ml, 300 mm before, is the 4427's
300.5)."""


@dataclass(frozen=True)
class Saddle:
    """A printed saddle on two tie rods, in the cradle's frame (mm): a carrier on the rods
    (``x`` its span along the axis, ``y0`` its bottom) and a V block riding up and down in
    it on a thumbscrew (``vee`` its span along the axis, ``bottom`` its underside, ``apex``
    the V's apex under its TPU liner and ``top`` its top edges, each at :data:`DESIGN`'s
    setting: the block's lowest, sitting on the tie rods; ``travel`` how far it rises)."""

    name: str
    ties: tuple[str, str]
    x: tuple[float, float]
    y0: float
    vee: tuple[float, float]
    bottom: float
    apex: float
    top: float
    travel: float


def _apex(r: float) -> float:
    """The V's apex (90° included) under a radius ``r`` lying in it with its axis at y 0."""
    return -(r + LINER) * math.sqrt(2)


DESIGN = METHUSELAH
"""The bottle the saddles' V blocks bottom out for, the fattest neck and body of
:data:`SIZES` (and the longest and heaviest: the load case): at its setting each V block
sits on its tie rods. A thinner bottle's V blocks are raised (:func:`setting`) so that its
axis lies level through the pivot too."""
SHOWN = DESIGN
"""The bottle the model draws in the saddles: the V blocks are drawn at its setting, their
lowest (the envelope the clashes, the room under the mouth and the loads are measured
in); :func:`at_setting` sets the drawn cradle for another."""

FRONT_BEAR = 64.5      # the front V's bearing ends here (cradle x; design review round 3,
#                        mounting M1): behind it the block's V is relieved (FRONT_RELIEF) and
#                        its liner stops, so the V holds a neck only in front of any shoulder
#                        (Saverglass's 4427 flares from 71 mm from its lip: borne to 79, it
#                        had lain on its shoulder, set by its neck 3.9 mm low)
FRONT_RELIEF = 6.0     # the V lowered this much behind its bearing (a 1 mm floor kept under
#                        it): 6.2 mm of radius over the neck it's set for, a 4427's shoulder
#                        4.4 mm over its neck there
FRONT_WALL_RELIEF = 6.0     # the carrier's back wall's notch this much under the lowest
#                             liner's (it had been 1 mm under): a 750's shoulder slid onto
#                             the pad clears it by 3.0 mm

SADDLES = (
    # the neck's V block in a box guide in its carrier (its underside 5.7 mm under the 1/2 in
    # tie rods: an M3 x 5 set screw up to each); it bears on the neck to FRONT_BEAR; at its
    # lowest it sits 0.05 mm over the rods' tops (-41.6: 0.35 mm higher since the 12 mm
    # rods, the simplification round 1, 2026-10-21; its V's apex where it was)
    Saddle("front_saddle", ("Tf1", "Tf2"), (45.0, 85.0), -60.0, (48.3, 79.0), -41.55,
           _apex(DESIGN.r_neck), -10.0, 15.0),
    # the body's V block, under a 750 ml's body (its base at 300); the base stop's rails'
    # flanges continue its V behind it to 588, under a 6 L's whole body, so lying level or
    # tipped its weight centres on the support (cradle_report.balance); it rides a steel
    # post and a lead screw at its ends
    Saddle("rear_saddle", ("Tr1", "Tr2"), (255.0, 380.0), -149.0, (255.0, 380.0), -130.9,
           _apex(DESIGN.r), -55.0, 72.0),
)

STOP_GAP = 2.5          # the base stop's face behind the bottle's base: at rest the mouth
#                         lies 2.7 mm behind the lip gauge's pad (1.7-3.7 for a length
#                         measured to a millimetre) and slides onto the pad past level
#                         (the stream lands at most 3.7 mm off until it does: review round
#                         2, N2; it had been 0.5, which left 0.7 mm for a bottle's length;
#                         a bottle too long can't lie in under the fixed gauge)
NECK_AT = FRONT_BEAR - STOP_GAP     # where the guide measures a neck, from its lip (62 mm): the
#                                     front V's bearing's back end, the bottle at rest on the
#                                     gate, every real neck's widest there over the bearing
#                                     (round 3, mounting M1: measured at 80, a steep Bordeaux
#                                     set 3.9-5.1 mm low)
STOP_RANGE = (285.0, 576.0)     # where the base stop's face can go (cradle x): inside the
#                                 rear V block (the 750 ml's at 303) to short of the
#                                 rails' end (the 6 L's at 573.5)


@dataclass(frozen=True)
class Setting:
    """How the cradle is set for a bottle (mm): each V block's rise over its lowest (the
    :data:`DESIGN` bottle's), the base stop's face along the axis from the mouth."""

    front: float
    rear: float
    stop: float


def setting(size: BottleSize) -> Setting:
    """``size``'s setting: each V block raised by what its neck and body are thinner than
    :data:`DESIGN`'s (times √2: a 90° V), the base stop's face :data:`STOP_GAP` behind its
    base with its mouth at the lip gauge (on the pivot). A bottle off the list sets the same
    from its diameters (each saddle's scale) and the gauge (``docs/agentlib/DECANTER.md``)."""
    front = (DESIGN.r_neck - size.r_neck) * math.sqrt(2)
    rear = (DESIGN.r - size.r) * math.sqrt(2)
    for s, u in zip(SADDLES, (front, rear), strict=True):
        if not -1e-9 <= u <= s.travel:
            raise ValueError(f"{size.name}: the {s.name}'s V block would rise {u:.1f} mm "
                             f"(it travels 0-{s.travel:g})")
    stop = size.length + STOP_GAP
    if not STOP_RANGE[0] <= stop <= STOP_RANGE[1]:
        raise ValueError(f"{size.name}: the base stop can't go {stop:g} mm behind the mouth "
                         f"({STOP_RANGE[0]:g}-{STOP_RANGE[1]:g})")
    return Setting(front, rear, stop)


def offsets(size: BottleSize) -> Setting:
    """How far each adjustable part moves from :data:`SHOWN`'s setting to ``size``'s (the
    V blocks up, the base stop's gate along the axis; cradle frame, mm)."""
    a, b = setting(SHOWN), setting(size)
    return Setting(b.front - a.front, b.rear - a.rear, b.stop - a.stop)


def bottle_room(size: BottleSize, mouth_x: float = STOP_GAP) -> dict:
    """The room round ``size``'s outline (:meth:`BottleSize.profile`: a maker's drawing where
    it has one) lying in the saddles set for it, its axis on the pivot's line, its mouth
    ``mouth_x`` behind the pivot (at rest on the gate, :data:`STOP_GAP`; slid onto the pad,
    the pad's face, ``GAUGE_FRONT[1]``), in mm (design review round 3, mounting M1 and M2: the
    model's cosine shoulder had hidden a real one on the carrier's back wall and on the
    light): ``neck`` the radius the guide measures (:data:`NECK_AT` from the lip) and
    ``bearing`` the widest the front V bears on (to :data:`FRONT_BEAR`: the same, a neck
    widening to its shoulder); the least radial room behind the bearing to the V's relief
    (``relief``) and to the carrier's back wall's notch (``wall``); the least height of the
    glass over the light's head (``light``); how far the lip gauge's pad's bottom edge comes
    down the rim's face (``pad``)."""
    front = SADDLES[0]
    prof = np.asarray(size.profile(), dtype=float)

    def widest(x0: float, x1: float) -> float:
        us = np.linspace(x0, x1, max(int((x1 - x0) * 4), 2)) - mouth_x
        return float(np.max(np.interp(us, prof[:, 0], prof[:, 1])))

    r_m = size.r_neck
    rise = (DESIGN.r_neck - r_m) * math.sqrt(2)
    face = r_m + LINER + FRONT_RELIEF / math.sqrt(2)
    floor = front.bottom + 1.0 + rise
    behind = widest(FRONT_BEAR, front.vee[1] + CLEAR)
    notch = front.apex + LINER * math.sqrt(2) - 1.0 - FRONT_WALL_RELIEF
    d = PENLIGHT[0]
    over = widest(LED_X - d / 2, LED_X + d / 2)
    ring = float(np.max(np.interp(np.linspace(0.0, 3.0, 13), prof[:, 0], prof[:, 1])))
    return {"neck": round(float(np.interp(NECK_AT, prof[:, 0], prof[:, 1])), 2),
            "bearing": round(widest(front.vee[0], FRONT_BEAR), 2),
            "relief": round(min(face - behind, -behind - floor), 2),
            "wall": round(-notch / math.sqrt(2) - widest(front.vee[1] + CLEAR, front.x[1]), 2),
            "light": round(-over - LED_TOP, 2), "pad": round(ring - GAUGE_PAD[0], 2)}


# --- the stand -------------------------------------------------------------------------

@dataclass(frozen=True)
class Stand:
    """A cradle's frame plates as a truss (mm, relative to the bottle's mouth at rest).

    ``nodes``: the free points; a member may also name O or a pillar. ``outer``/``inner``:
    each plate's members (pills of ``member_r``). ``columns``: a post between a side's two
    plates there (``column_d``: 1/2 in 6061 rod since simplification round 1). ``feet``:
    where the outer plates stand on the table (a TPU boot each). ``base_ties``: a tie rod
    across the whole cradle, outer plate to outer plate through both stacks (clear of the
    decanter: never in front). ``bay_ties``: a tie rod across the bay between the two inner
    plates (where nothing of the cradle passes)."""

    nodes: Mapping[str, tuple[float, float]]
    outer: tuple[tuple[str, str], ...]
    inner: tuple[tuple[str, str], ...]
    columns: tuple[str, ...] = ()
    feet: tuple[str, ...] = ()
    base_ties: tuple[str, ...] = ()
    bay_ties: tuple[str, ...] = ()
    member_r: float = 12.0
    wide: Mapping[tuple[str, str], float] = field(default_factory=dict)  # a member's own
    #                                 radius, where it is wider than ``member_r``
    column_d: float = SMALL_TIE_D   # the columns' standoffs: 10 mm (NETRF10, M5) or 12 (NETRF12,
    #                             M6: :func:`column_parts`)
    base: tuple[float, float] = (-200.0, 482.0)    # the base board's front and rear edges
    fillet_r: float = 20.0      # the plates' inside corners rounded this much (their joints
#                                 read as one shape, not crossed bars)


BASE_SHEET = "birch_9mm"        # the base board, both sides' outer plates bolted onto it:
#                                 Baltic birch plywood, routed (2026-10-13, the user's)
BASE_T = 9.0                    # its thickness
WOOD_GLUE = "wood_glue_woodcraft"   # its cleats and the riser locator's layers glued
#                                     (Titebond II, from Woodcraft with the panels)
GREASE = "grease_h1"            # every greased face (NSF H1: the saddles' sit in the wine's
#                                 path; design review round 3's stand and use, m10: "any
#                                 lithium or PTFE grease", unbought, had been a shop supply)
BASE_HALF = 292.0               # its half-width across the bay: a 584 mm track (a full 6 L
#                                 tips sideways past 25° on B: cradle_report.stability)
BASE_Z: dict[str, float] = {"decanter_rcm": -40.0}
"""The base board's middle across the bay (mm, z; the drive side is -z), per cradle (0 if
not named). B's sits 40 mm toward its drive side: the gearbox and the hand crank outboard
put the whole cradle's centre of mass 68 mm that way, so a centred board had 26° to the
drive side's edge and 32° to the other (round 1's stand review). The passive brackets'
knobs reach 244 mm out, 8 mm inside its passive edge."""
BASE_CORNER_R = 35.0            # its four corners rounded (the frames' fillets' language;
#                                 a square plywood corner chips and catches a shin)
NOTCH_MOUTH_R = 8.0             # the notch's two corners at the front edge, rounded too
FOOT_PAD_H = 8.0                # its printed TPU feet, snapped into its corner holes
FOOT_INSET = 25.0               # a foot's centre in from the board's edges
FOOT_PAD_R = 12.0               # its pad's radius (the tipping edges: cradle_report.stability)
BASE_TOP_CLEAR = FOOT_PAD_H + BASE_T - 2.0  # what may hang under it: all but 2 mm of the feet
BASE_TOP = -LIP_HEIGHT + FOOT_PAD_H + BASE_T    # its top, from the mouth
NOTCH_R = 126.0                 # the decanter's notch in it: a 244 mm decanter and play,
#                                 slid in from the front onto the table, the notch's round
#                                 end locating it under the mouth
DECANTER_X = 0.0                # the decanter's axis, under the mouth (a stream lands -15..+16 mm
#                                 from it over the pour: 2026-10-10 review round 2)
LOCATOR = (-70.0, 1.0, 2)       # the riser locator in the notch: its front edge (x from the
#                                 mouth), its play to the notch each side, its layers of the
#                                 board's birch (glued: 18 mm, so its top layer meets the
#                                 board's whole edge; the board stands 8 mm up on its feet)
NOTCH_BAY = (-45.0, 15.0)       # the notch's two side bays (x from the mouth, radius) the
#                                 locator's ears drop into: it can't slide out forward
BAY_POINT_R = 3.0               # the four points a bay leaves on the notch's edges, rounded
#                                 (design review round 2, 2026-10-15: no sharp birch points)
EAR_FILLET_R = 2.0              # the locator's ears' inside corners (SendCutSend's birch page:
#                                 "add fillets to internal corners"; its bit leaves R1.6)
DOG_BONE_R = 1.6                # a routed square corner's relief: the router's 1/8 in bit
#                                 (SendCutSend's least hole in 9 mm birch, 3.175 mm)
CLEAT = (26.0, 0.02, 4.0)       # a rail's two birch cleats, glued to the board either side of
#                                 it: their width across (SendCutSend's least birch part is
#                                 25.4 mm), the play each side of the rail (glued against the
#                                 rail itself, nothing between: what the glue-up leaves; 0.15,
#                                 a sheet of paper's, until design review round 3's M6: the
#                                 side rocked +-1.3 deg on it, +-9.7 mm at B's O1), the gap to a
#                                 bracket
CLEAT_BEAR = BASE_T - 2.0 - 0.5     # the rail's height bearing on its cleats (mm): the cleat's
#                                 less its lead-in and the rail's 0.5 mm over the board; the
#                                 side rocks 2 x play over it (cradle_report.side_stiffness)
CLEAT_LEAD = 2.0                # a lead-in sanded at 45° on each cleat's top edge by the rail,
#                                 so the rails find their slots blind (round 2: 0.15 mm a side)
CLEAT_END_R = 5.0               # a cleat's four corners rounded (design review round 2,
#                                 2026-10-15: no square birch ends)
HOME_SHEET = "birch_3_8_panel"  # the cleats and the riser locator: Woodcraft's 3/8 in (9 mm)
#                                 Baltic birch panels, cut at home on the Makera Carvera
#                                 (simplification round 1, 2026-10-21, the user's choice;
#                                 SendCutSend's routed BASE_SHEET before: the board stays)
CARVERA_LONGEST = 285.0         # a part's longest side cut on the Carvera: a 12 in panel's
#                                 300 mm nest (sheet_catalog.CARVERA_PANEL) less the packer's
#                                 3 mm margins, with room
CLEAT_JOINT = 1.0               # a cleat longer than that comes in equal pieces butted end to
#                                 end on its pencil marks, this far apart, each glued along its
#                                 length (B's 452 mm ones in two), their joint ends square
BOARD_WINDOW = (285.0, 390.0, 105.0, 20.0)
"""A window through the base board under the base stop's lowest sweep (design review round
2, 2026-10-14): from and to (x behind the mouth), its half-width across the bay's middle and
its corners' radius. At rest, set for a 6 L, the base stop comes down to 26 mm over the
board, a finger's 25 mm (ISO 13854) and less than the cork just pulled; over the window
whatever lies there lies on the table, 43 mm under it (:func:`finger_gap`). It cuts no cleat,
bolt or foot, and the board stays whole round it: 159 mm of birch between it and the notch,
134 mm behind it on B (A: 92)."""
_FOOT_Y = BASE_TOP + 12.5       # a foot's centre: its screw's boss (2 t) on the base board
BRACKET = (36.0, 30.0, 5.0)     # a foot's printed bracket: its length, legs' height, wall
"""A stand foot is a printed L-bracket outside each outer plate, an M5 button head through
it and the plate (its nut in the layer over the plate) and a carriage bolt up through the
base board and its foot's oversize hole, an 18-8 fender washer and a printed thumb knob (an
18-8 nut pressed in it) on it (:func:`_bracket`). A rail raised over :data:`_FOOT_Y`
(:func:`rail_y`) raises its feet' screws with it and their brackets' uprights as much. On a
cradle of :data:`BOARD_BRACKETS` the bracket stays bolted to the board and the plate's M5 is
a stud with a knob on it, dropping into the upright's open-topped slot."""
BOARD_BRACKETS: frozenset[str] = frozenset({"decanter_rcm"})
"""The cradles whose foot brackets stay bolted to the base board (the pack's turn 2,
2026-10-20; the packing review's L1): B's. Each outer plate's foot M5 is then a stud, an M5
button head from the plate's inside (its head where the nut was, in the layer over the
plate: the same claim) held by a jam nut on the plate's outer face (threadlocker, once), and
a printed knob on it (an 18-8 nut pressed in) clamps the bracket's upright to the plate by
hand. The upright is slotted open at its top (:data:`STUD_SLOT`), so the cradle lifts
straight off its board with the knobs loosened a few turns, the studs, jam nuts and knobs
with it, and drops back with each stud in its slot (the slots and the cleats guide it). The
bracket's foot is bolted down by its carriage bolt, washer and a plain M6 nut (a 10 mm
wrench, at the first build: no knob turns there again). Its passive side's brackets had set
the pack's height, 30 mm proud of the panel lying on top (B's box 149.4 L; with them on the
board 141 L, :mod:`spiderpig.cradle_pack`)."""
STUD_SLOT = 12.0        # the upright's open-topped slot: the jam nut's corners (9.24 mm) and
#                         1.3 mm of play either side (the side settles where the linkage
#                         puts it, as the foot's oversize hole let it: :data:`BRACKET_HOLE`)
STUD_KNOB = (26.0, 1.5, 12)     # the stud's printed knob: across, its floor (between its
#                                 pressed M5 nut and the upright), flutes round it
SLOT_LEAD = 2.0         # the slot's mouth chamfered this far at 45° either side (design
#                         review round 3's stand and use, m11: a square mouth left the four
#                         studs to be aimed to its 1.4 mm of play by eye)
KNOB_BACK_TURNS = 2
_WORDS = ("no", "one", "two", "three", "four")
"""How far the trip's way back backs each stud's knob off its jam nut before the cradle is
lowered onto its board (design review round 3's stand and use, M1): snug on its jam nut for
the trip, the knob's floor stands :data:`M5_NUT`'s 4.0 mm off the plate, where the
bracket's 5.0 mm upright must pass, so a cradle lowered so hung on the uprights' top edges
by its four knobs, 29 mm up and out of its cleats (one turn back, 4.8 mm, still 0.2 short);
two turns of :data:`M5_PITCH` leave the upright 0.6 mm and the knob's nut still all on its
stud (``tests/test_cradle.py``)."""


def rail_y(rail_r: float) -> float:
    """The y of a rail ``rail_r`` wide each side of its line (and of its feet, the screws on
    that line), from the mouth: its bottom edge where a 24 mm rail's is, 0.5 mm over the base
    board, between its cleats (:data:`CLEAT`). A wider rail (the cradle side frames' redesign,
    2026-10-20) stands higher on the same bottom edge: its columns' bodies and heads then
    clear the cleats' tops (:data:`BASE_T` over the board)."""
    return _FOOT_Y + rail_r - 12.0


_RAIL_B = 30.0          # B's rail: 60 mm (its columns E1 and E2 on it; 44 before design
#                         review round 3, whose model put the rail at E2's foot at 1.16)
_RAIL_A = 30.0          # A's rail: 60 mm (its columns E1 and E2 on it; the inner leg to them
#                         60 mm wide, its edge 0.5 mm over the board as the rail's; 40 before
#                         round 3, the rail at E1's foot 0.39 on its model)

STANDS: dict[str, Stand] = {
    # A: the hinge G at the mouth, the crank O 173 behind and 202 under it. The outer plate
    # (outside every moving part's layer: any truss) from G down in front of the decanter
    # to the front foot and back to O, O's legs down and back, a rail on the base board.
    # The inner plate is a small block round G, tied to the outer by two columns in front of
    # the hinge, where nothing of the cradle ever goes; nothing crosses the bay over the
    # mouth (the base board and the cradle box hold the sides).
    "decanter_hinge": Stand(
        # (2026-10-10, the user's −50° rest: O 300 under the mouth and 65 behind it; the
        # mouth 460 over the table) the feet: in front of the decanter, under O, at the rear
        nodes={"Ff": (-150.0, rail_y(_RAIL_A)), "Fm": (140.0, rail_y(_RAIL_A)),
               "Fr": (380.0, rail_y(_RAIL_A)),
               "C1": (-24.0, 20.0), "C2": (-24.0, -22.0),
               # on the front leg G-Ff, the inner plate's leg over it down to about 200 mm
               # over the table: its lowest column D3 halves the outer leg's free length (design
               # review round 1, 2026-10-14: the leg takes 2 kN jammed, and from D2 to the
               # foot it buckled out of the plate at 0.6)
               **{n: (-150.0 * f, rail_y(_RAIL_A) * f) for n, f in (("D1", 0.175),
                                                                    ("D2", 0.36),
                                                                    ("D3", 0.68))},
               # (the side frames' redesign, 2026-10-20) a column on the rail under D3, the
               # inner plate's leg down to it: the posts no longer stand on one line, so the
               # inner plate holds their ends across it (DECANTER.md, "The side frames");
               # (design review round 3, 2026-10-20) a second, E2, under the mouth, the
               # inner chord along the rail to it: the side buckled out of its plates at
               # 1.35 x the clutch's cap, and its rail at E1 held 0.39
               "E1": (-80.0, rail_y(_RAIL_A)), "E2": (10.0, rail_y(_RAIL_A))},
        outer=(("G", "Ff"), ("G", "O"), ("Ff", "O"), ("O", "Fm"), ("O", "Fr"),
               ("Ff", "Fm"), ("Fm", "Fr"), ("G", "C1"), ("G", "C2"), ("C1", "C2"),
               ("C2", "D1")),
        inner=(("G", "C1"), ("G", "C2"), ("C1", "C2"), ("C2", "D1"), ("D1", "D2"),
               ("G", "D1"), ("D2", "D3"), ("D3", "E1"), ("E1", "E2")),
        # (the simplification round 1, 2026-10-21: no column at C2, idle at 5.4 x the clutch's
        # cap on steel, 9 on 6061; its node stays both plates' corner)
        columns=("C1", "D1", "D2", "D3", "E1", "E2"),
        feet=("Ff", "Fm", "Fr"),
        # the strut from the hinge back to the gearbox: the gearbox's node O leans on it
        # across the plate (the legs from O buckled sideways at walking SF 3.0), and with the
        # legs from O it takes the gearbox's turn out of the plate there: 96 mm wide and the
        # legs 40 (the side frames' redesign, 2026-10-20: at 48 and 24 the strut held the
        # clutch's cap at 0.63 at O on the side as one frame); the rail 40 mm; the inner
        # plate's G-D1 23.6 mm, so the 12 mm front tie rod Tf1 passes it by the plan's margin
        # (design review round 2's M1, 2026-10-15: at 24 it came 6.9 mm from it). Design
        # review round 3 (2026-10-20: on 5052's minimum yield, the side as one frame second
        # order, its buckling): the strut 140 mm at O, the legs from O 56, the front leg 48,
        # the triangles round G 40 and their leg to D1 34 (both plates: as wide as the front
        # tie rod Tf1 passing it leaves), the inner leg 40 down
        # to D3, 60 to E1 and 40 along the rail to E2 (its bottom edge clear of the riser locator
        # in the board's notch), the rail 60
        wide={("G", "O"): 70.0, ("Ff", "O"): 28.0, ("O", "Fm"): 28.0, ("O", "Fr"): 28.0,
              ("Ff", "Fm"): _RAIL_A, ("Fm", "Fr"): _RAIL_A, ("G", "D1"): 11.3,
              ("G", "Ff"): 24.0, ("G", "C1"): 20.0, ("G", "C2"): 20.0, ("C1", "C2"): 20.0,
              ("C2", "D1"): 17.0, ("D1", "D2"): 20.0, ("D2", "D3"): 20.0,
              ("D3", "E1"): 30.0, ("E1", "E2"): 20.0},
        column_d=12.0,
        base=(-200.0, 482.0),
    ),
    # B: the arms' pivots O1, O2 hang straight under the mouth, the crank O 366 behind it
    # and low (its handle behind everything). The outer plate is two triangles on the
    # pivots' bar, O1-O over O2-O, a front leg from O2, a rear leg straight down from O and
    # a rail; O2-O, the strut that takes the rod's push back to the gearbox, is 64 mm wide
    # (1.45 kN jammed; at 24 mm it buckled out of the plate at 0.7, and the gearbox's node O
    # leans on it across the plate). The inner plate is the bar O1-O2, a triangle
    # O1-O2-D1 off it and a leg D1-D2 down over the outer plate's front leg, on two 12 mm
    # columns (design review round 1, 2026-10-14: the bar alone held its pillars only along
    # itself, so across it O2 was a cantilever, jam SF 0.4; now each side is a box from the
    # pivots down, as A's is; a column between O1 and D1 would stand in the lever b2's
    # sweep). The tie rods pass behind it (the rear ones never further round than 72° down,
    # the bar at 90.5°, the triangle and the leg in front of it) and the front ones above
    # it. Nothing crosses the bay: the decanter stands under the mouth, between the bars.
    "decanter_rcm": Stand(
        # (simplification round 2, 2026-10-21: the front foot Ff 50 mm back, from x -110, and
        # the board's front with it, its width kept; E1 and E2 60 mm forward, from 220 and
        # 280: the outer plate's blank 13 % smaller, the inner plate's chord 60 mm shorter)
        nodes={"Ff": (-60.0, rail_y(_RAIL_B)), "Fr": (386.0, rail_y(_RAIL_B)),
               # on the front leg O2-Ff, a quarter and 0.6 of the way down it from O2 (O2 is
               # (-2.7, -307))
               **{n: (-2.7 + (-60.0 + 2.7) * f, -307.0 + (rail_y(_RAIL_B) + 307.0) * f)
                  for n, f in (("D1", 0.25), ("D2", 0.6))},
               # (the side frames' redesign, 2026-10-20) two columns on the rail toward the
               # rear, the inner plate's bottom chord from D2 to them: a box toward the rear,
               # so the posts no longer stand on one line and the inner plate holds their ends
               # across it (DECANTER.md, "The side frames")
               "E1": (160.0, rail_y(_RAIL_B)), "E2": (220.0, rail_y(_RAIL_B))},
        outer=(("O1", "O2"), ("O2", "Ff"), ("O2", "O"), ("O1", "O"), ("O", "Fr"),
               ("Ff", "Fr"), ("O2", "Fr")),
        inner=(("O1", "O2"), ("O2", "D1"), ("O1", "D1"), ("D1", "D2"), ("D2", "E1"),
               ("E1", "E2")),
        # (the simplification round 1, 2026-10-21: no column at D1, idle at 8.9 x the clutch's
        # cap; its node stays the inner plate's corner)
        columns=("D2", "E1", "E2"),
        feet=("Ff", "Fr"),
        # (design review round 2's B1, 2026-10-15: the front leg 40 mm wide, O2 to the foot;
        # at 24 its strip at the columns' holes and down to the foot held the clutch's cap
        # out of its plane at 0.44-0.8; layer 0 only: cradle_report.structure) The side
        # frames' redesign (2026-10-20): the rail 44 mm (the rear column's foot), the rear
        # leg 56 and O1-O 40 (the gearbox's turn at O), O2-Fr 32 (its buckling), the inner
        # chord 40 and its leg D1-D2 32. Design review round 3 (2026-10-20: on 5052's minimum
        # yield, the side as one frame second order, its buckling): the rail 60, the rear leg
        # 80, O1-O 72, O2-O 88, O2-Fr 40, the inner chord 60, its leg 44, O1-D1 27 (as wide as
        # the front tie rod Tf2 passing it leaves)
        wide={("O2", "O"): 44.0, ("O2", "Ff"): 20.0, ("Ff", "Fr"): _RAIL_B, ("O", "Fr"): 40.0,
              ("O1", "O"): 36.0, ("O2", "Fr"): 20.0, ("D2", "E1"): 30.0, ("E1", "E2"): 30.0,
              ("D1", "D2"): 22.0, ("O1", "D1"): 13.0},
        column_d=12.0,
        # (round 1, 2026-10-14: with the corrected centre of mass and the board 40 mm
        # toward the drive side, BASE_Z, the rear edge binds: 682 mm is the least board
        # that keeps a full 6 L past 25° over it since round 1's drive, whose parts took the
        # 680 mm board's 25.0° to 24.9°: 25.1° at 682. Round 2's drive (the NRV050 and its
        # hand crank) took 682's to 24.9°: 684; with round 2's window under the base stop as
        # well, 684's rear stood at 25.0° to the tenth: 686, 2 mm more at the rear. That bound
        # counts a full 6 L at every tilt; with the wine leaving as it pours the rear holds
        # 28°: cradle_report.stability's "pouring"). Simplification round 2: its front 50 mm
        # back with the front foot (-160 before), so the front pad keeps its carriage bolt's
        # spacing; the rear stays (a 10 mm trim took the rear to 25.2°)
        base=(-110.0, 526.0),
    ),
}


def stand_of(lk) -> Stand:
    try:
        return STANDS[lk.key]
    except KeyError:
        raise ConstructionError(f"{lk.key} is a cradle with no stand (construction.cradle."
                                "STANDS)") from None


def base_z(lk) -> float:
    """The base board's middle across the bay (:data:`BASE_Z`)."""
    return BASE_Z.get(lk.key, 0.0)


def mouth_frame(lk) -> tuple[str, str]:
    """The cradle link's frame (``Output.frame``): the mouth and a point along the bottle."""
    assert lk.output is not None
    return lk.output.frame


def mouth_name(lk) -> str:
    """The cradle link's still point, the bottle's mouth (``Output.still``)."""
    assert lk.output is not None
    assert lk.output.still is not None
    return lk.output.still[0]


def cradle_link(lk) -> str:
    assert lk.output is not None
    return lk.output.link


def tie_points(topo, lk) -> dict[str, str]:
    """Tie name (``Tf1``...) -> its topology point (the cradle link's lone joints)."""
    link = cradle_link(lk)
    return {joint: p for (body, joint), p in sorted(topo.point_of.items())
            if body_class(body) == link and joint.startswith("T")}


def _point_of(topo, lk, joint: str) -> str:
    link = cradle_link(lk)
    for (body, j), p in topo.point_of.items():
        if j == joint and (body_class(body) == link or p == joint):
            return p
    return joint


def _node_xy(get, lk, topo, stand: Stand, name: str) -> np.ndarray:
    """A stand node's world XY: a named point (O, a pillar) or mouth + its offset."""
    if name in stand.nodes:
        return get(_point_of(topo, lk, mouth_name(lk))) + np.asarray(stand.nodes[name], float)
    return get(name)


MOUTH_DECIMALS = 9
"""The mm decimals :func:`mouth_xy` rounds the mouth to, 1e-9 mm (B's sits 1.1e-13 mm off
its design's -366, 322 in its frozen mechanism, differently at each crank angle)."""


def mouth_xy(build: Build) -> np.ndarray:
    """The mouth's world XY (:func:`mouth_name`), the same at every crank angle: its planned
    geometry's first sample (where the stand's claims stand it) rounded to
    :data:`MOUTH_DECIMALS`, not the frozen mechanism's. The mouth stands still over the turn
    (the linkage's ``Output.still`` promises it, to 1e-6 mm for B and 1e-9 for A), but B's
    mechanism puts it up to 1.1e-13 mm off from one crank angle to the next (its cradle
    link's pose there; A's hinge is exact), so the stand, the base board and the parts off
    the mouth moved by that between angles and no memo of them (:func:`plate_part`,
    :func:`_part_memo`) hit at another angle (measured 2026-10-20). Snapped when the
    mechanism's is within that promise of the plan's (always, while the output check holds:
    the parts move by 1e-13 mm), else the mechanism's."""
    lk = build.ctx.config.lk
    assert lk.output is not None            # (a cradle's: mouth_name)
    assert lk.output.still is not None
    name = _point_of(build.plan.topo, lk, mouth_name(lk))
    at = build.xy(name)
    planned = np.asarray(build.plan.topo.geometry.points[name], float).reshape(-1, 2)[0]
    if float(np.abs(at - planned).max()) <= lk.output.still[1]:
        return np.round(planned, MOUTH_DECIMALS)
    return at


def _ground_xy(build: Build):
    """``build.xy`` but the mouth's, :func:`mouth_xy` (the stand's nodes stand off it)."""
    lk = build.ctx.config.lk
    mouth = _point_of(build.plan.topo, lk, mouth_name(lk))
    lip = mouth_xy(build)
    return lambda name: lip.copy() if name == mouth else build.xy(name)


# --- the side-level groups ------------------------------------------------------------

class CradleStand(Group):
    """The stand in one side: its plates' members (pads), its columns, and the base ties'
    holes; the columns and base ties claimed through every layer between the plates."""

    name = "stand"
    stack_only = True           # the columns' screw heads stand outside the plates

    def _points(self, ctx: Context) -> dict[str, str]:
        """Each column and foot as a fixed point of the side's geometry."""
        lk = ctx.config.lk
        stand = stand_of(lk)
        geo = ctx.topo.geometry
        pts = geo.points

        def get(n):
            return np.asarray(pts[n][0], dtype=float)

        out = {}
        for n in (*stand.columns, *stand.base_ties, *stand.bay_ties, *stand.feet):
            name = f"stand.{n}"
            if name not in pts:
                geo.points[name] = np.broadcast_to(_node_xy(get, lk, ctx.topo, stand, n),
                                                   (geo.samples, 2))
            out[n] = name
        geo.__dict__.pop("step", None)
        return out

    def claims(self, ctx: Context) -> list[Claim]:
        pts = self._points(ctx)
        stand = stand_of(ctx.config.lk)
        bay = {pts[n] for n in stand.bay_ties}
        feet = {pts[n] for n in stand.feet}
        col_ = column_parts(stand)
        r = col_.d / 2 + 0.3
        r_face = (face_washer()[0] / 2 + 0.3) if col_.face else r   # its end washers
        head = M5.head_d / 2 + 0.3
        nut = M5_NUT[0] / math.sqrt(3) + 0.3

        def make(L: Layout):
            out = []
            for name in feet:       # a foot bracket's screw's nut, over the outer plate
                out.append(Placed(0, Disc(name, TIE_HOLE / 2), self.name, "foot screw",
                                  seat=True))
                out.append(Placed(1, Disc(name, nut), self.name, "foot screw nut",
                                  sheet=M5_NUT[1]))
            for name in bay:        # a bay tie's screw: up through the inner plate
                out.append(Placed(L.top, Disc(name, TIE_HOLE / 2), self.name, "bay tie screw",
                                  seat=True))
                out.append(Placed(L.top - 1, Disc(name, head), self.name, "bay tie screw head",
                                  gap=True, height=M5.head_h + 0.25, toward=-1))
            for name in (p for n, p in pts.items() if p not in bay | feet):
                out += [Placed(k, Disc(name, r), self.name, "stand column", seat=True)
                        for k in (0, L.top)]
                for k in range(1, L.top):       # its end washers in the layers (and the gaps)
                    rk = r_face if k in (1, L.top - 1) else r       # on the plates' faces
                    out.append(Placed(k, Disc(name, rk), self.name, "stand column"))
                    out.append(Placed(k, Disc(name, r_face if k == L.top - 1 else r),
                                      self.name, "stand column", gap=True))
                out.append(Placed(0, Disc(name, r_face), self.name, "stand column",
                                  gap=True))
            return out

        return [Claim("stand", frozenset(), make)]

    def realize(self, build: Build, done: Realized) -> Realized:
        ctx = build.ctx
        lk = ctx.config.lk
        stand = stand_of(lk)
        out = Realized()
        topo = build.plan.topo

        ground = _ground_xy(build)

        def xy(n):
            return tuple(_node_xy(ground, lk, topo, stand, n))

        for key, members in ((FRAME_OUTER, stand.outer), (FRAME_INNER, stand.inner)):
            for a, b in members:
                out.pad(key, xy(a), xy(b), stand.wide.get((a, b), stand.member_r))
        # nothing of the outer plate below the board's top: a member wider than the rail
        # ending at a foot (B's 40 mm front leg) is cut off there, not rounded into the board
        # (its section through the foot stays whole; the rail and the feet's bosses end
        # 0.5 mm and less over the board, untouched)
        feet = [np.asarray(xy(n), float) for n in stand.feet]
        if feet:
            nodes = [np.asarray(xy(n), float) for pr in stand.outer for n in pr]
            x0 = min(p[0] for p in nodes) - 100.0
            x1 = max(p[0] for p in nodes) + 100.0
            board = float(mouth_xy(build)[1]) + BASE_TOP
            out.cut(FRAME_OUTER, Rect(((x0 + x1) / 2, board - 50.0), (x1 - x0, 100.0)))
        frame = topo.frame_bodies[0]
        hole = TIE_HOLE
        z0, z1 = build.z(0)[1], build.z(build.top)[0]      # between the plates
        for n in (*stand.base_ties, *stand.feet):
            out.cut(FRAME_OUTER, Cut(xy(n), hole))
        for n in stand.bay_ties:
            out.cut(FRAME_INNER, Cut(xy(n), hole))
        col = column_parts(stand)
        w_od, w_id, w_t = face_washer() if col.face else (0.0, 0.0, 0.0)
        for n in stand.columns:
            p = xy(n)
            out.cut(FRAME_OUTER, Cut(p, col.hole))
            out.cut(FRAME_INNER, Cut(p, col.hole))
            if col.face:            # a fender washer on each plate's face under its end
                for tag, lo in (("foot", z0), ("end", z1 - w_t)):
                    out.bodies.append(hardware(
                        f"stand_column_{n}_{tag}washer",
                        difference(disc(p, w_od / 2, lo, lo + w_t),
                                   disc(p, w_id / 2, lo - 1.0, lo + w_t + 1.0)),
                        frame, fab="purchased", bom_key=col.face, color=TIE_COLOR))
            c0, c1 = z0 + w_t, z1 - w_t
            out.bodies.append(hardware(f"stand_column_{n}", _tie(p, c0, c1, col.d, col.thread),
                                       frame, fab="purchased", bom_key=col.key(c1 - c0),
                                       color=TIE_COLOR))
            for side, z in ((-1, build.z(0)[0]), (1, build.z(build.top)[1])):
                if col.washer is not None:
                    from spiderpig.hardware.catalog import get

                    w = get(col.washer).dims
                    wt = float(w["t"])
                    out.bodies.append(hardware(
                        f"stand_column_{n}_washer{'io'[side > 0]}",
                        difference(disc(p, float(w["od"]) / 2, *sorted((z, z + side * wt))),
                                   disc(p, float(w["id"]) / 2, z - 3.0, z + 3.0)),
                        frame, fab="purchased", bom_key=col.washer, color=TIE_COLOR))
                    z += side * wt
                out.bodies.append(hardware(f"stand_column_{n}_screw{'io'[side > 0]}",
                                           _screw(p, z, side, col.screw_len, col.screw),
                                           frame, fab="purchased",
                                           bom_key=col.screw.key(col.screw_len),
                                           color=TIE_COLOR))
        out.notes["stand"] = {"columns": list(stand.columns), "feet": list(stand.feet),
                              "base_ties": list(stand.base_ties),
                              "bay_ties": list(stand.bay_ties),
                              "column_mm": round(z1 - z0 - 2 * w_t, 2),
                              "plates_gap_mm": round(z1 - z0, 2)}
        return out

    def assembly(self, view) -> list:
        """The stand's columns (:mod:`construction.assembly`): each screwed to the bare
        outer plate from outside first, as the pillars are; its inner screw from the bay
        once the inner plate is on."""
        from spiderpig.construction.assembly import JOIN, STACK, Op, whole
        from spiderpig.hardware.fasteners import BUTTON_A2_70_NM

        columns = view.named(r"stand_column_[A-Za-z0-9]+")
        faces = view.named(r"stand_column_\w+_(?:foot|end)washer")
        outer = view.named(r"stand_column_\w+_(?:screw|washer)i")
        inner = view.named(r"stand_column_\w+_(?:screw|washer)o")
        nm = BUTTON_A2_70_NM["6"]
        washed = bool(view.named(r"stand_column_\w+_washer[io]"))
        head = ("(and its washer, on a 12 mm column)" if washed else
                "(straight on the deburred plate: no washer under it)")
        ops = []
        if columns:
            ops.append(Op(STACK, (-1, 1), whole(*columns, *faces, *outer), "",
                          f"Screw the stand's columns to the outer plate, a button head {head} "
                          "from outside into each "
                          + ("through an 18 mm fender washer on the plate's inner face under "
                             "the column's end, another on its top end for the inner plate "
                             "(all of one thickness: caliper them from the bag, within 0.05 "
                             "mm)" if faces else "")
                          + f" (threadlocker; {nm:g} N·m, ISO 7380's most in A2-70: TR "
                          "Fastenings' table).", "columns"))
        if inner:
            ops.append(Op(JOIN, (2,), whole(*inner), "",
                          "The columns' inner button heads "
                          + ("(and washers) " if washed else "") + "from the bay side "
                          f"(threadlocker, {nm:g} N·m). After a clutch slip (a blocked cradle "
                          f"cranked until the clutch clicks), take every post's screws back to "
                          f"{nm:g} N·m: the slip may have set the plates under the posts' "
                          "ends.", "column_screws"))
        return ops

    def motion(self, got: Realized) -> Motion:
        return RIDES_HOST


class CradleTies(Group):
    """The cradle box's tie rods in one side: their columns from the cradle link through
    every layer to the inner plate (whose layer they pass in its open), and the screw
    heads under the link."""

    name = "ties"

    def claims(self, ctx: Context) -> list[Claim]:
        lk = ctx.config.lk
        link = next(n for n in ctx.topo.links if body_class(n) == cradle_link(lk))
        ties = list(tie_points(ctx.topo, lk).values())
        r = TIE_D / 2 + 0.3
        head = TIE_M.head_d / 2 + 0.3

        def make(L: Layout):
            k = L.layers[link]
            if k != L.top - 1:
                # the cradle link rides next to the inner plate, so its tie rods leave the
                # stack through that plate's layer alone (and short)
                raise Unbuildable(f"the cradle link {link} rides under the inner plate "
                                  f"(layer {L.top - 1}), not in layer {k}")
            out = []
            for name in ties:
                out.extend(Placed(j, Disc(name, r), self.name, "tie rod")
                           for j in range(k + 1, L.top))
                out.extend(Placed(j, Disc(name, r), self.name, "tie rod", gap=True)
                           for j in range(k, L.top))
                out.append(Placed(L.top, Disc(name, r), self.name, "tie rod", seat=True))
                # its screw's head under the link: in the gap there, or the layer under it
                # (the rod under it is held off it by its washer on the crankpin: its play,
                # handcrank.CRANKPIN_PLAY, is less than the 0.4 mm left)
                out.append(Placed(k - 1, Disc(name, head), self.name, "tie screw head",
                                  gap=True, height=TIE_M.head_h + 0.25, toward=-1))
            return out

        return [Claim("tie rods", frozenset((link,)), make)]

    def realize(self, build: Build, done: Realized) -> Realized:
        lk = build.ctx.config.lk
        out = Realized()
        topo = build.plan.topo
        link = next(n for n in topo.links if body_class(n) == cradle_link(lk))
        ties = tie_points(topo, lk)
        for name in ties.values():
            out.cut(link, Cut(tuple(build.xy(name)), TIE_ROD_HOLE))
        _ties_clear_the_inner_plate(build, done, ties)
        return out

    def assembly(self, view) -> list:
        """The cradle link (:mod:`construction.assembly`): the drive side's with its tie
        rods screwed on, on the bench, before it goes on its pins (their screws' heads end
        up under it); the passive side's goes on the tie rods' other ends on the cradle
        (``CRADLE_ORDER`` moves it), not in its stack. Placed here, ahead of the links'
        hook."""
        import re

        from spiderpig.construction.assembly import STACK, Op, whole

        link = cradle_link(view.ctx.config.lk) if view.ctx is not None else None
        if link is None or link not in view.z:
            return []
        k = view.slot(view.z[link][0])
        lk = view.ctx.config.lk
        clipped = sorted(j for j in lk.links[link][0] if f"pin_{j}_clip" in view.z)
        keepers = sorted(n for n in view.z for j in clipped
                         if re.fullmatch(rf"pin_{re.escape(j)}_(?:gap\d+_spacer|ring\d+)", n))
        ops = []
        if keepers:     # onto their pins over the links under the cradle link
            low = min(view.slot(view.z[n][0]) for n in keepers) - 1
            ops.append(Op(STACK, (low, 4), whole(*keepers), "", keeper_text(clipped),
                          "keepers"))
        if view.side == "R":
            return [Op(STACK, (k, 1), whole(link), "", "", "cradle_link"), *ops]
        return [Op(STACK, (k, 0, "ties"), whole(link), "Cradle link and tie rods",
                   f"The four tie rods onto the cradle link, an M{TIE_M.d:g} x "
                   f"{TIE_ROD_SCREW:g} steel button head (class 10.9, from its own bag: not "
                   "a stainless one) through the link from its leg side into each "
                   f"(threadlocker, {TIE_TIGHTEN_NM:g} N·m on the key: closed, the joint holds "
                   "the box's walking moment without working); the rods "
                   "stand up from its bay side. It goes onto the stack rods up, its bores "
                   "down over the posts and the pins it rides.", "cradle_link",
                   sub=True), *ops]

    def motion(self, got: Realized) -> Motion:
        return RIDES_HOST


_PLATE_MEMO: dict = {}


class CradlePlates(FramePlates):
    """A cradle's frame plates: the stand's members (the pads :class:`CradleStand` asks
    for) and every other group's pads, with a boss round every round hole (the cut rules'
    two thicknesses), minus the holes; no arms from O (the stand's members replace them)."""

    def assembly(self, view) -> list:
        """The outer plate first, its outside down (on the drive side, onto the gearbox's
        riser); the inner plate the first part of its unit, its bay face down."""
        from spiderpig.construction.assembly import STACK, UNIT, Op, whole

        topo = getattr(view.ctx, "topo", None)
        inner = topo.frame_bodies[0] if topo is not None and topo.frame_bodies else None
        ops = []
        if "frame_outer" in view.z:
            ops.append(Op(STACK, (-1, 0), whole("frame_outer"), "Outer frame plate",
                          "Lay the outer frame plate down, its outside down.", "plate"))
        if inner is not None and inner in view.z:
            ops.append(Op(UNIT, (0,), whole(inner), "Inner plate",
                          "Lay the inner plate down, its bay face down.", "plate"))
        return ops

    def realize(self, build: Build, done: Realized) -> Realized:
        out = Realized()
        frame = build.plan.topo.frame_bodies[0]
        web = boss_web(build.ctx.sheet("frame"))
        for key, layer, name in ((FRAME_INNER, build.top, frame),
                                 (FRAME_OUTER, 0, "frame_outer")):
            z0, z1 = build.z(layer)
            pads = done.pads.get(key, [])
            if not pads:
                raise ConstructionError(f"the cradle's {key} plate has no members")
            cuts = done.cuts.get(key, [])
            fillet = stand_of(build.ctx.config.lk).fillet_r
            keep_out = ()
            if key == FRAME_INNER:
                _tips_clear_the_inner_plate(build, done)
                keep_out = _keep_out(build)
            part = plate_part(pads, cuts, z0, z1, web, fillet, keep_out,
                              taper=build.ctx.config.linkage in TAPERED)
            out.bodies.append(hardware(name, part, frame, fab="laser", color=FRAME_COLOR,
                                       sheet=build.ctx.sheet("frame")))
        return out


STEP_MAX = 4.5          # a boss or a member's end this far or less proud of a strip: tapered
STEP_TURN = 30.0        # two members this far or less off a straight line run straight on
TAPER_RUN = 10.0        # a taper from a proud station into its plain strip: 1 in this
EDGE_R = 3.0            # the outline's convex corners (where the board cut meets a member)
TAPERED: frozenset[str] = frozenset({"decanter_rcm"})
"""The cradles whose frame plates' members are tapered station to station (:func:`_tapered`):
B's. A's keep the closing's S-steps for now (cosmetic: none is sharp): its flat pack lays both
panels face down on the board with the loose parts in their open triangles, and the tapers
(601 mm² more on its outer plate) closed what its printed spacers stood in, one growing to a
327 mm print and another left loose 66 mm (``tests/test_cradle.py``'s pack test, 2026-10-20);
the pack's turn 3 takes them up. Every convex corner is rounded on both (:data:`EDGE_R`)."""


def plate_part(pads, cuts, z0: float, z1: float, web: float, fillet: float = 0.0,
               keep_out=(), taper: bool = True) -> Shape3D:
    """A cradle frame plate: its members (pills, tapered where a boss or a straight-on
    member stands a little proud of them: :func:`_tapered`), a boss round every round
    hole, the inside corners rounded ``fillet`` (a closing: the outline grown and shrunk
    back, so the joints read as one shape), clear of ``keep_out`` (solids: what passes the
    plate's layer), its convex corners rounded :data:`EDGE_R` (an opening: where the board
    cut meets a member, or a keep-out the edge), less the holes. Memoized (the keep-outs by
    identity: :func:`_keep_out` gives the same solid for the same paths; each entry holds its
    keep-outs, so their ids stay theirs)."""
    memo = (tuple(pads), tuple(cuts), z0, z1, web, fillet, tuple(map(id, keep_out)), taper)
    hit = _PLATE_MEMO.get(memo)
    if hit is not None:
        return hit[0]
    bosses = [(c.xy, c.d / 2 + web) for c in cuts if isinstance(c, Cut) and web > 0]
    away = union(list(keep_out)) if keep_out else None
    parts = (_tapered(pads, bosses, z0, z1, away) if taper      # (not: :data:`TAPERED`)
             else [pill(a, b, r, z0, z1) for a, b, r in pads])
    parts += [disc(xy, r, z0, z1) for xy, r in bosses]
    part = union(parts)
    if fillet > 0:
        part = _closed(part, z0, z1, fillet)
    if away is not None:
        part = difference(part, away)
    edges = [c for c in cuts if isinstance(c, Rect)]
    if edges:
        part = cut_holes(part, edges, z0, z1)
    part = _opened(part, z0, z1, EDGE_R)
    part = cut_holes(part, [c for c in cuts if not isinstance(c, Rect)], z0, z1)
    if len(_PLATE_MEMO) > 16:
        _PLATE_MEMO.clear()
    _PLATE_MEMO[memo] = (part, tuple(keep_out))
    return part


def _tapered(pads, bosses, z0: float, z1: float, away=None) -> list[Shape3D]:
    """The plate's members (``pads``: ``(a, b, r)``), each from station to station along
    it (its ends, and every boss centred on it between) the convex hull of the two
    stations' discs (design review round 3, laser-cut parts minor 5: where strips of
    different widths met, or a boss stood proud of its strip, the closing left an S-shaped
    step of 2.5-4 mm in a straight edge); between a proud station and a plain one, a straight
    taper of 1 in :data:`TAPER_RUN` into the plain strip. A station's disc is the widest of
    the member,
    the bosses there and every member running straight on from it (within
    :data:`STEP_TURN`), if that is at most :data:`STEP_MAX` proud of the member, else the
    member's own (a designed widening, a gusset, a corner keep their shape). A taper that
    would reach into ``away`` (the inner plate's keep-outs) keeps the step."""
    out = []
    for i, (a, b, r) in enumerate(pads):
        a, b = (float(a[0]), float(a[1])), (float(b[0]), float(b[1]))
        length = math.dist(a, b)
        if length < 1e-6:
            out.append(pill(a, b, r, z0, z1))
            continue
        u = ((b[0] - a[0]) / length, (b[1] - a[1]) / length)
        stations = [(0.0, _station(pads, bosses, i, a, b))]
        for xy, rb in bosses:           # a boss on the member between its ends
            s = (xy[0] - a[0]) * u[0] + (xy[1] - a[1]) * u[1]
            off = abs((xy[0] - a[0]) * u[1] - (xy[1] - a[1]) * u[0])
            if off < 1e-3 and 1e-6 < s < length - 1e-6 and r < rb <= r + STEP_MAX + 1e-9:
                stations.append((s, rb))
        stations.append((length, _station(pads, bosses, i, b, a)))
        stations.sort()
        for (s0, r0), (s1, r1) in itertools.pairwise(stations):
            if max(r0, r1) <= r + 1e-9:
                out.append(pill(_along(a, u, s0), _along(a, u, s1), r, z0, z1))
                continue
            if min(r0, r1) <= r + 1e-9:     # one end proud: a taper of TAPER_RUN to 1
                run = min(s1 - s0, TAPER_RUN * (max(r0, r1) - r))
                if r1 > r0:
                    plain, s0 = (s0, s1 - run), s1 - run
                else:
                    plain, s1 = (s0 + run, s1), s0 + run
                if plain[1] - plain[0] > 1e-6:
                    out.append(pill(_along(a, u, plain[0]), _along(a, u, plain[1]), r, z0, z1))
            p, q = _along(a, u, s0), _along(a, u, s1)
            if s1 - s0 <= abs(r1 - r0) + 1e-6:
                out.append(pill(p, q, r, z0, z1))
                continue
            hull = _hull_pill(p, r0, q, r1, z0, z1)
            if away is not None and difference(hull, pill(p, q, r, z0, z1)).distance_to(
                    away) < 1e-6:
                out.append(pill(p, q, r, z0, z1))
            else:
                out.append(hull)
    return out


def _station(pads, bosses, i: int, n, far) -> float:
    """Member ``pads[i]``'s station disc at its end ``n`` (its other end ``far``): the widest
    of it, the bosses at ``n`` and the members running straight on from it there, if at
    most :data:`STEP_MAX` proud of it, else its own (:func:`_tapered`)."""
    r = pads[i][2]

    def at(p, q) -> bool:
        return math.dist(p, q) < 1e-6

    def heading(p) -> float:
        return math.atan2(p[1] - n[1], p[0] - n[0])

    wide = r
    for j, (p, q, w) in enumerate(pads):
        for end, other in ((p, q), (q, p)):
            if j != i and at(end, n) and not at(other, n):
                turn = abs(math.remainder(heading(other) - heading(far), 2 * math.pi))
                if math.degrees(turn) >= 180.0 - STEP_TURN:
                    wide = max(wide, w)
    wide = max([wide] + [rb for xy, rb in bosses if at(xy, n)])
    return wide if wide - r <= STEP_MAX + 1e-9 else r


def _along(a, u, s: float) -> tuple[float, float]:
    """The point ``s`` along the unit direction ``u`` from ``a``."""
    return (a[0] + s * u[0], a[1] + s * u[1])


def _hull_pill(p, rp: float, q, rq: float, z0: float, z1: float) -> Shape3D:
    """The convex hull of a disc of radius ``rp`` at ``p`` and one of ``rq`` at ``q`` (a
    tapered stadium: two tangent lines and two arcs, no boolean), extruded ``z0..z1``."""
    from build123d import Edge, Vector

    length = math.dist(p, q)
    u = np.array([q[0] - p[0], q[1] - p[1]]) / length
    n = np.array([-u[1], u[0]])
    s = (rp - rq) / length
    c = math.sqrt(max(0.0, 1.0 - s * s))
    lo, hi = s * u - c * n, s * u + c * n
    pa, qa = np.asarray(p, float), np.asarray(q, float)

    def v(xy):
        return Vector(float(xy[0]), float(xy[1]), z0)

    wire = Wire([Edge.make_line(v(pa + rp * lo), v(qa + rq * lo)),
                 Edge.make_three_point_arc(v(qa + rq * lo), v(qa + rq * u), v(qa + rq * hi)),
                 Edge.make_line(v(qa + rq * hi), v(pa + rp * hi)),
                 Edge.make_three_point_arc(v(pa + rp * hi), v(pa - rp * u), v(pa + rp * lo))])
    return Solid.extrude(Face(wire), Vector(0, 0, z1 - z0))


def _opened(part: Shape3D, z0: float, z1: float, r: float) -> Shape3D:
    """``part`` (a plate from ``z0`` to ``z1``) with its convex corners rounded ``r``: its top
    outline shrunk ``r`` and grown back (round joins), extruded back down (a corner the
    board's cut leaves on a member, design review round 3's laser-cut parts minor 5: a
    sharp point a hand finds)."""
    from build123d import Kind, extrude, offset

    tops = [f for f in part.faces() if abs(f.center().Z - z1) < 1e-6
            and abs(abs(f.normal_at().Z) - 1.0) < 1e-6]
    if not tops:
        return part
    grown = offset(offset(tops, -r, kind=Kind.ARC), r, kind=Kind.ARC)
    faces = [f for f in grown.faces() if isinstance(f, Face)]
    return union([extrude(f, amount=-(z1 - z0)) for f in faces])


def _closed(part: Shape3D, z0: float, z1: float, r: float) -> Shape3D:
    """``part`` (a plate from ``z0`` to ``z1``) with its inside corners rounded ``r``: its top
    outline grown ``r`` and shrunk back (round joins), extruded back down."""
    from build123d import Kind, extrude, offset

    tops = [f for f in part.faces() if abs(f.center().Z - z1) < 1e-6
            and abs(abs(f.normal_at().Z) - 1.0) < 1e-6]
    if not tops:
        return part
    shrunk = offset(offset(tops, r, kind=Kind.ARC), -r, kind=Kind.ARC)
    faces = [f for f in shrunk.faces() if isinstance(f, Face)]
    return union([extrude(f, amount=-(z1 - z0)) for f in faces])


_KEEP_OUT_MEMO: dict = {}


def _keep_out(build: Build) -> tuple[Shape3D, ...]:
    """The inner plate's keep-outs (:func:`_tie_paths`, :func:`_tip_paths`) fused into one
    tool (none: ``()``): their sections fused as one planar region and extruded once
    (:func:`shapes.pills`; as a fuse of the prisms it was 62-70 of B's 95-100 s first
    fabrication, measured 2026-10-20, the same point set). The same at every crank angle
    (the paths are over the whole turn, the layer the plan's): memoized by every pill's
    exact ends, radius and z extent, so a process's next fabrication of the design (another
    angle, a report's clash angles) fuses nothing."""
    specs = tuple(_tie_specs(build) + _tip_specs(build))
    if not specs:
        return ()
    hit = _KEEP_OUT_MEMO.get(specs)
    if hit is None:
        hit = pills(specs)
        if len(_KEEP_OUT_MEMO) > 8:
            _KEEP_OUT_MEMO.clear()
        _KEEP_OUT_MEMO[specs] = hit
    return (hit,)


def _tie_specs(build: Build) -> list[tuple]:
    """:func:`_tie_paths`' pills as :func:`pill`'s arguments."""
    ties = tie_points(build.plan.topo, build.ctx.config.lk)
    z0, z1 = build.z(build.top)
    r = TIE_D / 2 + build.plan.spec.margin
    out = []
    for name in ties.values():
        path = np.asarray(build.plan.topo.geometry.points[name], float).reshape(-1, 2)
        step = max(1, len(path) // 48)
        pts = [*path[::step], path[0]]
        out += [(tuple(a), tuple(b), r, z0 - 1.0, z1 + 1.0)
                for a, b in itertools.pairwise(pts)]
    return out


def _tie_paths(build: Build) -> list[Shape3D]:
    """What the inner plate's layer must stay clear of: each tie rod's path over the turn
    (its radius and the plan's margin), as solids through the layer."""
    return [pill(*s) for s in _tie_specs(build)]


def _tips(build: Build) -> list[Placed]:
    """What else passes the inner plate's layer in its open: a pin's end past its clip (a
    seat the pin claims there, ``tips``: :mod:`construction.pivots.clevis`)."""
    return [p for p in build.plan.shapes(layer=build.top, gaps=False)
            if p.seat and p.label.endswith(" tip") and isinstance(p.shape, Disc)]


def _tip_specs(build: Build) -> list[tuple]:
    """:func:`_tip_paths`' pills as :func:`pill`'s arguments."""
    z0, z1 = build.z(build.top)
    out = []
    for p in _tips(build):
        assert isinstance(p.shape, Disc)
        path = np.asarray(build.plan.topo.geometry.points[p.shape.at], float).reshape(-1, 2)
        step = max(1, len(path) // 48)
        pts = [*path[::step], path[0]]
        r = p.shape.r + build.plan.spec.margin
        out += [(tuple(a), tuple(b), r, z0 - 1.0, z1 + 1.0)
                for a, b in itertools.pairwise(pts)]
    return out


def _tip_paths(build: Build) -> list[Shape3D]:
    """The inner plate's keep-outs for :func:`_tips`: each one's path over the turn (its
    radius and the plan's margin), as solids through the layer."""
    return [pill(*s) for s in _tip_specs(build)]


def _tips_clear_the_inner_plate(build: Build, done: Realized):
    """Each of :func:`_tips` clears the inner plate's members and bosses over the turn by the
    plan's margin (as the tie rods do: :func:`_ties_clear_the_inner_plate`)."""
    for p in _tips(build):
        assert isinstance(p.shape, Disc)
        _clear_the_inner_plate(build, done, p.label, p.shape.at,
                               p.shape.r + build.plan.spec.margin, "pin end")


def _ties_clear_the_inner_plate(build: Build, done: Realized, ties: Mapping[str, str]):
    """Each tie rod passes the inner plate's layer in the plate's open, where the planner
    checks nothing (a frame plate's layer holds only what sits in its holes): its path over
    the whole turn must clear the plate's members and the bosses round its holes (as built
    so far: the stand's and the axles') by the plan's margin, or the rod would run into the
    plate."""
    reach = TIE_D / 2 + build.plan.spec.margin
    for tie, name in ties.items():
        _clear_the_inner_plate(build, done, tie, name, reach, "tie rod")


def _clear_the_inner_plate(build: Build, done: Realized, label: str, name: str,
                           reach: float, what: str):
    """Point ``name``'s path over the turn (``what`` ``label``'s) clears the inner plate's
    members and the bosses round its holes by ``reach``: :class:`ConstructionError`
    otherwise."""
    pads = done.pads.get(FRAME_INNER, [])
    web = boss_web(build.ctx.sheet("frame"))
    holes = [c for c in done.cuts.get(FRAME_INNER, []) if isinstance(c, Cut)]
    path = np.asarray(build.plan.topo.geometry.points[name], float).reshape(-1, 2)
    for a, b, r in pads:
        a_, b_ = np.asarray(a[:2], float), np.asarray(b[:2], float)
        ab = b_ - a_
        u = np.clip(((path - a_) @ ab) / max(float(ab @ ab), 1e-18), 0.0, 1.0)
        gap = float(np.linalg.norm(path - (a_ + u[:, None] * ab), axis=1).min()) - r
        if gap < reach:
            raise ConstructionError(f"{what} {label} comes within {gap:.1f} mm of the inner "
                                    f"plate's member {a[:2]}-{b[:2]} over the turn (it "
                                    f"needs {reach:.1f})")
    for c in holes:
        gap = float(np.linalg.norm(path - np.asarray(c.xy[:2], float), axis=1).min())
        if gap - c.d / 2 - web < reach:
            raise ConstructionError(f"{what} {label} comes within {gap - c.d / 2 - web:.1f} "
                                    f"mm of the inner plate's boss at {c.xy[:2]} over the "
                                    f"turn (it needs {reach:.1f})")


# --- parts -----------------------------------------------------------------------------

def _standoff(length: float) -> str:
    """The catalog key of a base or bay tie's or a 10 mm column's NETRF10 standoff
    ``length`` mm long."""
    from spiderpig.hardware.cradle_catalog import netrf10

    return netrf10(round(length, 1))


def _rod(length: float, d: float = TIE_D) -> str:
    """The catalog key of a 6061-T6511 round ``d`` mm, ``length`` mm long, tapped M6 both ends
    (a tie rod, a post: ``hardware.cradle_catalog.al_rod``)."""
    from spiderpig.hardware.cradle_catalog import al_rod

    return al_rod(d, round(length, 1))


def _tie(xy, z0: float, z1: float, d: float = TIE_D, thread: float = TIE_THREAD) -> Shape3D:
    """A tie rod or post (``d``: 12.7 mm a 1/2 in 6061 round tapped M6, 10 a NETRF10 steel
    standoff tapped M5) from z0 to z1, its ends tapped (modelled as holes the screw's size,
    ``thread`` deep)."""
    tap = 2.5 if d <= SMALL_TIE_D else 3.0
    return difference(disc(xy, d / 2, z0, z1),
                      *[disc(xy, tap, a, b)
                        for a, b in ((z0 - 1, z0 + thread), (z1 - thread, z1 + 1))])


def _screw(xy, face: float, side: int, length: float = TIE_ROD_SCREW, s=TIE_M) -> Shape3D:
    """A button head (``s``: a tie rod's M6) bearing on ``face``, its head on the ``side``
    (+1 over it, -1 under it) and its shank into the standoff beyond."""
    head = disc(xy, s.head_d / 2, *sorted((face, face + side * s.head_h)))
    shank = disc(xy, s.shank_d / 2, *sorted((face, face - side * length)))
    return union([head, shank])


@dataclass(frozen=True)
class ColumnParts:
    """A stand column's bought parts (:attr:`Stand.column_d`): its standoff's diameter and
    tapped depth, the button heads through the plates (size, length, the plates' hole) and
    the standoff's catalog key for a length."""

    d: float
    thread: float
    screw: Screw
    screw_len: float
    hole: float
    washer: str | None = None   # a flat washer under each head (its catalog key)
    face: str | None = None     # a fender washer between each end and its plate (its key)

    def key(self, length: float) -> str:
        """The post's catalog key for a ``length`` mm gap: a 1/2 in 6061 one by the pillars'
        own rule (``StandoffAxle.segment``: the same length as a pillar's in the same gap, so
        the side's posts are one part number, faced to one length)."""
        if self.d > SMALL_TIE_D:
            from spiderpig.construction.pivots.standoff import StandoffAxle

            got = StandoffAxle().cradle_shaft().segment(length)
            return _rod(got if got is not None else length, self.d)
        return _standoff(length)


POST_HOLE = 6.4         # a 12 mm post's M6 end screw's hole in a frame plate (ISO 273 fine; 6.6
#                         until design review round 2's m2: a post shifted 0.3 mm in it once its
#                         joint had gapped)
FACE_WASHER = "m6_washer_wide"  # the 18-8 M6 fender washer (6.4 x 18 x 1.6, Bolt Depot 7315)
#                         between a 12 mm post's end and its frame plate: every stand column's
#                         both ends, every pillar's foot (design review round 3, laser-cut parts
#                         M1: the 12 mm end's edge bore on the bare 5052 at the clutch's cap past
#                         VDI 2230's limiting pressure; spread over 18 mm it holds,
#                         cradle_report.post_joints). Matched by caliper within 0.05 mm (the
#                         guide: a post's length scatter is the inner plate's locked-in stress)


HEAD_WASHED: dict[str, tuple[str, ...]] = {"pillar": ("foot",), "column": ()}
"""The post ends whose M6 button head keeps a DIN 125 washer under it, by kind (simplification
round 2, 2026-10-21: every end had one since design review round 2's m2): a pillar's foot on
the outer plate, where the walking moment opens the face at the least preload a bare head's
friction on the 5052 leaves (``cradle_report.post_joints``: 1.07 x its decompression on B's
O1 and O2, 1.05 on A's G, at an under-head friction of 0.3; 0.85 and 0.83 washered), and where
B's lay-flat block takes O1's head in its relief (:data:`RISER_RELIEF`). Elsewhere the head
bears on the deburred plate (``cradle_report``'s head rows, VDI 2230's limiting pressure)."""


def face_washer() -> tuple[float, float, float]:
    """:data:`FACE_WASHER`'s OD, ID and thickness (mm)."""
    from spiderpig.hardware.catalog import get

    d = get(FACE_WASHER).dims
    return float(d["od"]), float(d["id"]), float(d["t"])


def column_parts(stand: Stand) -> ColumnParts:
    """:class:`ColumnParts` of ``stand``'s columns: a NETRF10 on M5 x 12 button heads in
    5.5 mm holes, or a 1/2 in 6061-T6511 round (M6 x 12 deep each end, tapped at home: the
    simplification round 1, 2026-10-21; MISUMI's 12 mm NETRF12 steel before) on M6 x 16
    button heads (A2-70) in 6.4 mm holes, their heads on the plate (:data:`HEAD_WASHED`:
    simplification round 2; DIN 125 washers under them since design review round 2's m2, when
    the heads had borne on the bare 5052 through 6.6 mm holes, on 18-8 screws of no stated
    class)."""
    if stand.column_d > SMALL_TIE_D:
        from spiderpig.hardware.cradle_catalog import ROD_HALF, ROD_THREAD

        return ColumnParts(ROD_HALF, ROD_THREAD, M6, 16.0, POST_HOLE,
                           "m6_washer" if HEAD_WASHED["column"] else None, FACE_WASHER)
    return ColumnParts(SMALL_TIE_D, 10.0, M5, TIE_SCREW, TIE_HOLE)


_PART_MEMO: dict = {}
_PART_MEMO_SIZE = 256
_PART_LOCK = threading.Lock()   # (:func:`_own` reads a memo's shape through shapes.moved)
_STORES = 0
"""How many times this module's namespace has been set from outside it since import (a
test's monkeypatch or its undo: :class:`_Module`); :func:`_part_memo`'s key carries it."""


class _Module(types.ModuleType):
    """This module, counting each store into its namespace from outside (``setattr(cr, ...)``,
    a monkeypatch and its undo, ``mock.patch``): a builder :func:`_part_memo` keeps reads
    this module's constants, so one set in place keys afresh. (Not ``globals()`` in the key:
    that reads every name the module binds, the assembly hooks among them, into the
    fabrication's code closure, :mod:`spiderpig.keys`.) The module's own assignments go
    straight to its namespace, uncounted."""

    def __setattr__(self, name: str, value) -> None:
        global _STORES
        _STORES += 1
        super().__setattr__(name, value)

    def __delattr__(self, name: str) -> None:
        global _STORES
        _STORES += 1
        super().__delattr__(name)


sys.modules[__name__].__class__ = _Module


def _exact(v):
    """``v`` as a key equal only to an equal ``v``: floats by their bits (``-0.0`` isn't
    ``0.0``), sequences and mappings in their order, dataclasses field by field, arrays with
    their dtype and shape; anything else (a shape) is a ``TypeError``."""
    import dataclasses

    if v is None or isinstance(v, str):
        return v
    if isinstance(v, bool):
        return ("b", v)
    if isinstance(v, (int, np.integer)):
        return ("i", int(v))
    if isinstance(v, (float, np.floating)):
        return ("f", float(v).hex())
    if isinstance(v, np.ndarray):
        return ("a", v.dtype.str, v.shape, tuple(_exact(x) for x in v.ravel().tolist()))
    if isinstance(v, (tuple, list)):
        return (type(v).__name__, tuple(_exact(x) for x in v))
    if isinstance(v, Mapping):
        return ("m", tuple((_exact(k), _exact(x)) for k, x in v.items()))
    if dataclasses.is_dataclass(v) and not isinstance(v, type):
        return ("d", type(v).__qualname__,
                tuple((f.name, _exact(getattr(v, f.name))) for f in dataclasses.fields(v)))
    raise TypeError(f"no exact key for a {type(v).__name__}")


def _own(v):
    """A memo's result as the caller's own: each shape a wrapper of its own (the same B-rep:
    :func:`shapes.moved` by nothing, so moving it in place leaves the memo's be), each
    container a new one."""
    import dataclasses

    from build123d import Shape

    if isinstance(v, Shape):
        return moved(v, Location())
    if isinstance(v, (tuple, list)):
        return type(v)(_own(x) for x in v)
    if isinstance(v, dict):
        return {k: _own(x) for k, x in v.items()}
    if dataclasses.is_dataclass(v) and not isinstance(v, type):
        return dataclasses.replace(v, **{f.name: _own(getattr(v, f.name))
                                         for f in dataclasses.fields(v) if f.init})
    return v


def _part_memo(fn):
    """``fn`` (a builder of a cradle's parts that is the same at every crank angle: the
    cradle box's parts in the cradle's frame, the stand's base board, brackets, cleats, feet,
    riser and decanter) memoized in the process, so a design's next fabrication (another
    angle: the clash angles, a report's, a sweep) builds none of them again (B's 19 s of
    its 27 s second fabrication, measured 2026-10-20). Keyed by everything that shapes the
    parts: ``fn``, every argument exactly (:func:`_exact`: an argument it can't key is
    built afresh), and this module's constants as they are (:data:`_STORES`: a constant or a
    helper set in its place, as a test's ``monkeypatch`` does, is another key; nothing
    changes one in place). Each caller gets parts of its own (:func:`_own`)."""
    import functools

    @functools.wraps(fn)
    def memoized(*args, **kwargs):
        try:
            key = (fn.__qualname__, _STORES, _exact(args), _exact(sorted(kwargs.items())))
        except TypeError:
            return fn(*args, **kwargs)
        with _PART_LOCK:
            hit = _PART_MEMO.get(key)
            if hit is not None:
                return _own(hit)
        made = fn(*args, **kwargs)
        with _PART_LOCK:
            if len(_PART_MEMO) >= _PART_MEMO_SIZE:
                _PART_MEMO.clear()
            _PART_MEMO.setdefault(key, made)
            return _own(made)

    return memoized


def _place(part, lip, tilt: float, z: float = 0.0):
    """A part drawn in the cradle's frame (the mouth at the origin, the axis along +x)
    turned by ``tilt`` about z and put at ``lip`` (world XY, z)."""
    loc = Location((float(lip[0]), float(lip[1]), z), (0, 0, math.degrees(tilt)))
    return moved(part, loc)


@_part_memo
def _bottle(size: BottleSize, axis_y0: float, slope: float, x0: float) -> Shape3D:
    """The bottle in the cradle's frame: its mouth at (x0, axis_y0 + slope x0), its axis
    sloped ``slope`` (radians, nose up positive)."""
    prof = size.profile()
    pts = [(0.0, 0.0), *prof, (size.length, 0.0)]
    face = Face(Wire.make_polygon([(x, r, 0.0) for x, r in pts], close=True))
    solid = Solid.revolve(face, 360.0, BAxis.X)
    loc = Location((x0, axis_y0, 0.0), (0, 0, math.degrees(slope)))
    return moved(solid, loc)


def seat(size: BottleSize, at: Setting | None = None) -> tuple[float, float, float]:
    """Where ``size`` lies in the saddles set for it (``at``: set for another), its mouth at
    the lip gauge: (its axis's y at the mouth's x, the axis's slope in radians, the mouth's
    x), in the cradle's frame."""
    s = setting(size) if at is None else at
    front, rear = SADDLES
    xf, xr = sum(front.vee) / 2, sum(rear.vee) / 2
    yf = front.apex + s.front + (size.r_neck + LINER) * math.sqrt(2)
    yr = rear.apex + s.rear + (size.r + LINER) * math.sqrt(2)
    slope = math.atan2(yr - yf, xr - xf)
    return yf - xf * math.tan(slope), slope, 0.0


SET_SCREW = 6.0            # an M3 cup-point set screw (the light's clip, the rear knob's hub,
#                            the barrel nut's)
CARRIER_SET_SCREW = 5.0    # the M3 cup-point set screw locking a carrier on each tie: 5 mm
#                            since the 1/2 in tie rods (the simplification round 1,
#                            2026-10-21: an M3 x 6 left 5.7 mm under a 12.7 mm rod)
SET_PILOT = 2.6            # its printed pilot (the screw cuts its own thread in PLA)
CLEAR = 0.3                # a sliding fit's clearance, each side


def _box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Shape3D:
    """A box between those planes (cradle frame)."""
    return Box(x1 - x0, y1 - y0, z1 - z0).moved(
        Location(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)))


def _prism_x(zy, x0: float, x1: float) -> Shape3D:
    """A polygon in the (z, y) plane extruded along the axis from ``x0`` to ``x1``."""
    face = Face(Wire.make_polygon([(x0, y, z) for z, y in zy], close=True))
    return Solid.extrude(face, (x1 - x0, 0, 0))


def _rod_y(x: float, z: float, r: float, y0: float, y1: float) -> Shape3D:
    """A cylinder up the cradle frame's y from ``y0`` to ``y1``."""
    return Cylinder(r, y1 - y0, rotation=(90, 0, 0)).moved(Location((x, (y0 + y1) / 2, z)))


def _rod_x(y: float, z: float, r: float, x0: float, x1: float) -> Shape3D:
    """A cylinder along the axis from ``x0`` to ``x1``."""
    return Cylinder(r, x1 - x0, rotation=(0, 90, 0)).moved(Location(((x0 + x1) / 2, y, z)))


def _hex_y(x: float, z: float, af: float, y0: float, y1: float) -> Shape3D:
    """A hexagonal prism up y (a nut, a hex head), ``af`` across its flats (facing ±z)."""
    r = af / math.sqrt(3)
    pts = [(x + r * math.cos(math.pi / 3 * i), y0, z + r * math.sin(math.pi / 3 * i))
           for i in range(6)]
    return Solid.extrude(Face(Wire.make_polygon(pts, close=True)), (0, y1 - y0, 0))


def _star_y(x: float, z: float, n: int, r_out: float, r_in: float, y0: float, y1: float
            ) -> Shape3D:
    """A star of ``n`` points up y (a ring of teeth): its points at ``r_out``, its notches
    at ``r_in``, a point toward +x."""
    pts = [(x + (r_out if i % 2 == 0 else r_in) * math.cos(math.pi / n * i), y0,
            z + (r_out if i % 2 == 0 else r_in) * math.sin(math.pi / n * i))
           for i in range(2 * n)]
    return Solid.extrude(Face(Wire.make_polygon(pts, close=True)), (0, y1 - y0, 0))


def _vee(x0: float, x1: float, apex: float) -> Shape3D:
    """What lies over a 90° V along the axis from ``x0`` to ``x1``, its apex at ``apex``
    (cradle frame)."""
    size = 400.0
    return Box(x1 - x0 + 2, size, size).rotate(BAxis.X, 45).moved(
        Location(((x0 + x1) / 2, apex + size / math.sqrt(2), 0.0)))


def _rod_z(x: float, y: float, r: float, z0: float, z1: float) -> Shape3D:
    """A cylinder across the bay (along z) from ``z0`` to ``z1``."""
    return Cylinder(r, z1 - z0).moved(Location((x, y, (z0 + z1) / 2)))


PEG = (1.75, 3.0)          # a TPU liner's pegs: radius, length into its V (they key it there)


def _peg(x: float, y: float, sign: int, apex: float, r: float, length: float,
         back: float = 0.0) -> Shape3D:
    """A peg square to a 90° V's face (cradle frame; the V's apex at ``apex``): on its
    ``sign`` side (+1: +z) at ``x`` and the face's height ``y``, ``length`` into the face's
    material (out of the V), ``back`` the other way (into a liner on the face)."""
    n = np.array([0.0, -1.0, float(sign)]) / math.sqrt(2)          # into the material
    c = np.array([x, y, sign * (y - apex)]) + n * (length - back) / 2
    return Cylinder(r, length + back, rotation=(sign * 45.0, 0, 0)).moved(
        Location(tuple(float(v) for v in c)))


def _liner_pegs(apex: float, pegs, sides=(-1, 1)) -> tuple[list[Shape3D], list[Shape3D]]:
    """A liner's pegs at ``pegs`` ((x, y) on its V's face, on each of ``sides``) and their
    holes in the V (a hair larger and deeper)."""
    keys, holes = [], []
    for x, y in pegs:
        for sign in sides:
            keys.append(_peg(x, y, sign, apex, PEG[0], PEG[1], back=0.3))
            holes.append(_peg(x, y, sign, apex, PEG[0] + 0.15, PEG[1] + 0.5))
    return keys, holes


def _vee_block(s: Saddle, block: Shape3D, pegs=()) -> tuple[Shape3D, Shape3D]:
    """A V block's V cut into ``block`` and the TPU liner lining it (cradle frame, at the
    block's lowest), keyed into the V by a peg each side at each of ``pegs`` (x, the face's
    y): pressed in, it can't walk out of the V as a bottle slides on it."""
    x0, x1 = s.vee
    outer = _vee(x0, x1, s.apex)                 # the liner's back: the block's V
    body = _box(x0, x1, s.bottom, s.top, -150.0, 150.0)
    liner = difference(cast(Shape3D, body & outer),
                       _vee(x0, x1, s.apex + LINER * math.sqrt(2)))
    keys, holes = _liner_pegs(s.apex, pegs)
    return difference(block, outer, *holes), union([liner, *keys])


def _ticks(x0: float, x1: float, face: float, sign: int, ys, width: float = 0.5,
           depth: float = 0.8) -> list[Shape3D]:
    """A scale's grooves (cradle frame): at each of ``ys``, ``width`` tall, along x from
    ``x0`` to ``x1``, ``depth`` into a wall whose face is at z ``face`` (``sign``: into +z
    or -z)."""
    z0, z1 = sorted((face - sign * 0.01, face + sign * depth))
    return [_box(x0, x1, y - width / 2, y + width / 2, z0, z1) for y in ys]


DOT = (0.5, 0.6)            # a scale's dots beside a named groove: radius, depth
DOT_PITCH = 1.3             # along the groove


def _graded(s: Saddle, r_design: float, every: float, lo: float, hi: float) -> list[float]:
    """A saddle's graded marks (its V block's reading edge at each, as its top edge's cradle
    y): one each ``every`` mm of diameter from ``lo`` to ``hi`` within its travel (a bottle
    off the list: its neck's or its body's diameter)."""
    sizes = [lo + every * k for k in range(round((hi - lo) / every) + 1)]
    graded = [s.top + (r_design - d / 2) * math.sqrt(2) for d in sizes]
    return [y for y in graded if s.top - 1e-9 <= y <= s.top + s.travel + 1e-9]


FRONT_GROUPS = (("standard", "magnum"), ("jeroboam",), ("imperial", "methuselah"))
"""The front scale's named grooves, from the top: the 750 ml's and the magnum's necks rise
the front V block 0.7 mm apart, the two 6 L's too, so each pair shares one groove at their
mean (0.36 mm off for each: 0.45 mm at the mouth; review round 1, m1)."""


def front_named() -> list[float]:
    """The front scale's named grooves (:data:`FRONT_GROUPS`, from the top): the front V
    block's rise for each."""
    by = {z.key: (DESIGN.r_neck - z.r_neck) * math.sqrt(2) for z in SIZES}
    return [sum(by[k] for k in g) / len(g) for g in FRONT_GROUPS]


# --- the front saddle: a V block in a box guide, its thumbscrew at its +z end -----------

FRONT_W = (-48.0, 52.0)       # the front carrier's -z and +z faces
FRONT_HALF = 34.0             # its V block's half-width (the V's top edges at 26.8)
FRONT_TOP = 6.0               # the carrier's walls' tops (the -z one's: FRONT_GUIDE)
FRONT_GUIDE = (18.0, 3.0, 3.7)      # the box guide's tall side (review round 2, m5): the -z
#                                     wall's top, and the V block's cheek up its -z face (its
#                                     top at the block's lowest, its thickness): the knob's
#                                     push at +z, 40 mm off the box's middle, cocks the block
#                                     onto that wall's top and the +z wall's foot, and the
#                                     taller that pair the less it binds (front_drawer)
FRONT_CHEEK = 13.0            # the cheek's top over the lip gauge's post (at the block's
#                               lowest; design review round 3, mounting m9): 10 mm taller
#                               between the gauge's cheeks, against the post's face (to its
#                               36.5 top), so the guide holds dry (the spill washes its
#                               grease out); 8.5 mm under the gauge's bridge at full travel
FRONT_FLOOR = -43.0           # its floor's top (the tie rods' tops, at -41.6, stand proud)
FRONT_TAB = (56.0, 72.0, -28.0)     # the V block's tab into the +z wall: x span, its top
FRONT_JACK = (65.0, 40.0)     # the thumbscrew (x, z): an M6 x 60 hex screw, fully threaded;
#                               midway between the front tie rods, its tip in the carrier's
#                               floor 1 mm from each 12 mm rod (design review round 2's M1:
#                               at 63, beside a 10 mm rod, it met the 12 mm Tf1; the tab 2 mm
#                               longer for its nut's pocket)
FRONT_NUT_Y = -40.0           # its nut's underside in the tab, the block at its lowest
FRONT_HEAD_Y = 9.5            # the screw's head's underside
FRONT_SCREW = 60.0
FRONT_DRAIN = (18.0, 26.0, 3.0)     # the drains through the back wall at the floor: |z|, height
M6_HEX = (10.0, 5.0, 4.0)     # DIN 934 M6 nut and its tap bolt's head: across flats, nut, head
M5_HEX = (8.0, 3.5, 5.0)      # an M5 tap bolt's head and a nylon-insert nut: flats, head, nut
FRONT_NUT_KEY, FRONT_JACK_KEY = "m6_nut", "m6_tap_bolt_60"     # (hardware.cradle_catalog)
KNOB_R = (11.0, 12.0)         # the printed knobs' radii (front, rear)
GAUGE_PIVOT = (30.0, -41.0)   # the lip gauge's bolt along x (cradle y, z)
GAUGE_POST = (55.8, 74.2, 36.5)     # its post on the carrier's -z wall: x span, its flat top
#                                     (the gauge's bridge sits on it); 18.4 mm along the bolt
#                                     (review round 1, M3)
GAUGE_POST_OUT = -56.0        # the post's outer face (|z| 8 mm past the carrier's -z face:
#                               design review round 3, mounting m1: 21.7 mm across, so it
#                               takes a wet bottle's slide onto the pad, cradle_report.saddles)
GAUGE_KNUCKLES = ((49.5, 55.5), (74.5, 80.5))   # its cheeks either side of the post
GAUGE_PAD = (10.5, 7.0)       # its pad's bottom (over a cork's bore) and half-width: 3.25 mm
#                               down a 750 ml's rim (Saverglass 4427: its ring 27.5 mm
#                               across), clear of a methuselah's 20.5 mm bore (round 2, m4)
GAUGE_FRONT = (-14.0, -0.2)   # its pad and arm along x: the back face in the mouth's plane
GAUGE_BAR = (20.0, 38.0, -36.0, -27.0)  # its arm in front of the mouth and its bar beside
#                                         the neck (a 6 L's at |z| 24): y0, y1; the bar's z
GAUGE_BRIDGE = (36.5, 50.0, -48.0, -30.0)   # over the post, cheek to cheek, flat on the
#                                             post's top: y0, y1, z
GAUGE_PIN = 40.0              # M5 x 40 tap bolt: front cheek, post, back cheek, lock nut
GAUGE_PIN_KEY, GAUGE_NUT_KEY = "m5_tap_bolt_40", "m5_nylon_nut"


@_part_memo
def _front_saddle(s: Saddle, ties: Mapping[str, tuple[float, float]]
                  ) -> tuple[Shape3D, list[Shape3D]]:
    """The front saddle's carrier (cradle frame): a floor on the two tie rods (an M3 set screw up
    from its underside onto each, in a printed pilot), walls round its V block (the block's
    guide, the -z one taller for the block's cheek, :data:`FRONT_GUIDE`; the front one
    notched a millimetre under the lowest liner, the back one :data:`FRONT_WALL_RELIEF`
    more, under a shoulder behind the V's bearing; two drains through the back one at the
    floor, its low end at rest), a pocket in the +z wall for the block's tab and the
    thumbscrew's foot, the lip gauge's post on the -z wall (its top flat: the gauge's bridge
    sits on it; widened outboard to :data:`GAUGE_POST_OUT`), the scale's grooves on the +z
    wall's inside (the block's top edge reads them: three named in front of the pocket,
    :func:`front_named`, a dot each per place from the top; the neck's diameter every 4 mm
    behind it). (The carrier, its set screws.)"""
    x0, x1 = s.x
    h = FRONT_HALF + CLEAR
    v0, v1 = s.vee[0] - CLEAR, s.vee[1] + CLEAR
    floor = _box(x0, x1, s.y0, FRONT_FLOOR, *FRONT_W)
    walls = [_box(x0, x1, FRONT_FLOOR - 1.0, FRONT_GUIDE[0], FRONT_W[0], -h),
             _box(x0, x1, FRONT_FLOOR - 1.0, FRONT_TOP, h, FRONT_W[1])]
    def notched(notch: float) -> list[tuple[float, float]]:
        return [(-h, FRONT_FLOOR - 1.0), (h, FRONT_FLOOR - 1.0), (h, notch + h), (0.0, notch),
                (-h, notch + h)]

    notch = s.apex + LINER * math.sqrt(2) - 1.0
    walls += [_prism_x(notched(notch), x0, v0),
              _prism_x(notched(notch - FRONT_WALL_RELIEF), v1, x1)]
    yp, zp = GAUGE_PIVOT
    px0, px1, ptop = GAUGE_POST
    post = [_box(px0, px1, FRONT_GUIDE[0] - 1.0, ptop, GAUGE_POST_OUT, -h),
            _box(px0, px1, FRONT_FLOOR + 3.0, FRONT_GUIDE[0], GAUGE_POST_OUT,   # its root
                 FRONT_W[0] + 0.01)]                                          # (over the rods)
    part = union([floor, *walls, *post])
    xj, zj = FRONT_JACK
    named = [s.top + u for u in front_named()]
    graded = _graded(s, DESIGN.r_neck, 4.0, 28.0, 48.0)
    t0, t1 = v0 + 1.0, FRONT_TAB[0] - CLEAR - 1.0          # the named row's span along x
    g1 = t0 + 1.2                                          # its grooves; their dots after
    dots = [_rod_z(g1 + DOT_PITCH * (k + 0.75), y, DOT[0], h - 0.01, h + DOT[1])
            for i, y in enumerate(named) for k in range(i + 1)]
    if g1 + DOT_PITCH * (len(named) + 0.25) > t1:
        raise ValueError("the front scale's dots don't fit in front of the knob's pocket")
    d0, d1, dh = FRONT_DRAIN
    cuts = [Box(30.0, 30.0, 2 * 60.0 + 2).rotate(BAxis.Z, 45).moved(   # over a decanter's
        Location((x0, s.y0, 0.0))),                                    # neck at rest
            _box(FRONT_TAB[0] - CLEAR, FRONT_TAB[1] + CLEAR, FRONT_FLOOR - 0.5,
                 FRONT_TOP + 1.0, h - 1.0, zj + M6_HEX[0] / 2 + 1.0 + CLEAR),
            _rod_y(xj, zj, 3.2, FRONT_HEAD_Y - FRONT_SCREW - 0.5, FRONT_FLOOR),
            _rod_x(yp, zp, TIE_HOLE / 2 - 0.1, px0 - 0.1, px1 + 0.1),
            *_ticks(t0, g1, h, 1, named),
            *dots,
            *_ticks(FRONT_TAB[1] + CLEAR + 1.0, v1 - 1.0, h, 1, graded),
            *[_box(v1 - 0.5, x1 + 1.0, FRONT_FLOOR - 0.01, FRONT_FLOOR + dh, *sorted(
                (sign * d0, sign * d1))) for sign in (-1, 1)]]
    screws = []
    for n in s.ties:
        x, y = ties[n]
        cuts.append(Cylinder(TIE_D / 2 + 0.15, 2 * 60.0 + 4).moved(Location((x, y, 0.0))))
        under = y - TIE_D / 2
        if under - CARRIER_SET_SCREW - 0.05 < s.y0:
            raise ValueError(f"{s.name}: an M3 x {CARRIER_SET_SCREW:g} set screw doesn't fit "
                             f"under "
                             f"tie {n} ({under - s.y0:.1f} mm)")
        cuts.append(_rod_y(x, 0.0, SET_PILOT / 2, s.y0 - 2.0, under))
        screws.append(_rod_y(x, 0.0, 1.25, under - 0.05 - CARRIER_SET_SCREW,
                             under - 0.05))
    return difference(part, *cuts), screws


FRONT_PEGS = ((56.4, -24.0),)       # the front liner's pegs (x, the V face's y), each side:
#                                     mid-bearing (the liner ends at FRONT_BEAR)


@_part_memo
def _front_vee(s: Saddle) -> tuple[Shape3D, Shape3D, Shape3D]:
    """The front V block at its lowest (cradle frame): the neck's V, sitting on the tie
    rods, its tab into the carrier's +z wall holding the thumbscrew's M6 nut (in a hex
    pocket from below: the block bears down on it), a cheek up its -z face along the
    carrier's taller wall (:data:`FRONT_GUIDE`; taller still over the lip gauge's post,
    :data:`FRONT_CHEEK`), its TPU liner pegged into the V. The V bears on the neck only
    to :data:`FRONT_BEAR` (its liner ends there): behind it the V is
    :data:`FRONT_RELIEF` lower, a millimetre of floor kept under it, so no shoulder rests
    on the block (design review round 3, mounting M1). (The block, its liner, the nut.)"""
    x0, x1 = s.vee
    xj, zj = FRONT_JACK
    _, cheek, ct = FRONT_GUIDE
    block = union([_box(x0, x1, s.bottom, s.top, -FRONT_HALF, FRONT_HALF),
                   _box(FRONT_TAB[0], FRONT_TAB[1], s.bottom, FRONT_TAB[2], FRONT_HALF - 0.5,
                        zj + M6_HEX[0] / 2 + 1.0)])
    block = difference(block, _hex_y(xj, zj, M6_HEX[0] + CLEAR, s.bottom - 1.0,
                                     FRONT_NUT_Y + M6_HEX[1] + 0.2),
                       _rod_y(xj, zj, 3.3, s.bottom - 1.0, FRONT_TAB[2] + 1.0))
    vee, liner = _vee_block(replace(s, vee=(x0, FRONT_BEAR)), block, FRONT_PEGS)
    relief = cast(Shape3D, _vee(FRONT_BEAR, x1 + 1.0, s.apex - FRONT_RELIEF)
                  & _box(FRONT_BEAR, x1 + 1.0, s.bottom + 1.0, s.top + 1.0, -FRONT_HALF - 1.0,
                         FRONT_HALF + 1.0))
    px0, px1, _ = GAUGE_POST
    vee = union([difference(vee, relief),
                 _box(x0, x1, s.top - 0.01, cheek, -FRONT_HALF, -FRONT_HALF + ct),
                 _box(px0 + CLEAR, px1 - CLEAR, cheek - 0.01, FRONT_CHEEK, -FRONT_HALF,
                      -FRONT_HALF + ct)])
    nut = difference(_hex_y(xj, zj, M6_HEX[0], FRONT_NUT_Y, FRONT_NUT_Y + M6_HEX[1]),
                     _rod_y(xj, zj, 3.0, FRONT_NUT_Y - 1.0, FRONT_NUT_Y + M6_HEX[1] + 1.0))
    return vee, liner, nut


@_part_memo
def _jack(xz: tuple[float, float], d: float, length: float, head: tuple[float, float],
          head_y: float, knob_r: float) -> tuple[Shape3D, Shape3D]:
    """A thumbscrew (cradle frame): a hex screw ``d`` x ``length`` standing in its carrier's
    blind bore, its head (across flats, height) up at ``head_y``, a printed knob pressed on
    the head (its hex pocket from below). (The screw, the knob.)"""
    x, z = xz
    screw_ = union([_hex_y(x, z, head[0], head_y, head_y + head[1]),
                    _rod_y(x, z, 0.97 * d / 2, head_y - length, head_y)])
    knob = difference(_rod_y(x, z, knob_r, head_y - 0.5, head_y + head[1] + 6.5),
                      _hex_y(x, z, head[0] + CLEAR, head_y - 1.0, head_y + head[1] + 0.2),
                      _rod_y(x, z, d / 2 + 0.3, head_y - 1.0, head_y + 0.1))
    return screw_, knob


@_part_memo
def _gauge() -> tuple[Shape3D, Shape3D, Shape3D]:
    """The lip gauge (cradle frame), fixed: two cheeks either side of the front carrier's
    post, an M5 tap bolt through them and the post along the axis, a nylon-insert nut
    behind, tight; a bridge joining the cheeks over the post, flat on its flat top (the
    bolt is 6.5 mm under it, so the gauge can't turn about it: one edge would have to sink
    into the post); a bar forward beside the neck (clear of a 6 L's as it's laid in) to an
    arm across in front of the mouth and its pad, their back face in the pivot's plane: the
    top of the mouth's rim bears on it, above the cork's bore. A bottle pushing it forward
    bears its back cheek on the post's back face; the bolt takes the turn of a push off its
    line. (Review round 2, N4 and m2: it had hinged up to uncork and fallen back by its
    own weight, against its hinge's friction by 1.3 x; a bottle is uncorked standing on
    the table before it goes in.) (The gauge, its bolt, the bolt's nut.)"""
    yp, zp = GAUGE_PIVOT
    (f0, f1), (b0, b1) = GAUGE_KNUCKLES
    y0, y1, bz0, bz1 = GAUGE_BAR
    by0, by1, rz0, rz1 = GAUGE_BRIDGE
    ax0, ax1 = GAUGE_FRONT
    bottom, half = GAUGE_PAD
    r = 6.5
    gauge = union([_rod_x(yp, zp, r, f0, f1), _rod_x(yp, zp, r, b0, b1),
                   _box(f0, f1, y0, by1, zp - r, bz1),                   # front cheek
                   _box(b0, b1, yp, by1, zp - r, zp + r),                # back cheek
                   _box(f0, b1, by0, by1, rz0, rz1),                     # bridge
                   _box(ax0, f0 + 0.01, y0, y1, bz0, bz1),               # bar
                   _box(ax0, ax1, y0, y1, bz0, half),                    # arm
                   _box(ax0, ax1, bottom, y0 + 0.01, -half, half)])      # pad
    gauge = difference(gauge, _rod_x(yp, zp, TIE_HOLE / 2, f0 - 1.0, b1 + 1.0))
    af, head_h, nut_h = M5_HEX
    x = f0 + GAUGE_PIN
    pin = union([_rod_x(yp, zp, af / 2, f0 - head_h, f0), _rod_x(yp, zp, 2.4, f0, x)])
    nut = difference(_rod_x(yp, zp, af / 2, b1, b1 + nut_h), _rod_x(yp, zp, 2.5, b1 - 1.0,
                                                                     b1 + nut_h + 1.0))
    if x < b1 + nut_h + 0.8:
        raise ValueError("the lip gauge's pin doesn't reach through its lock nut")
    return gauge, pin, nut


def gauge_fixed() -> dict:
    """The lip gauge held on its post (:func:`_gauge`): its bridge's underside on the post's
    flat top (``seated``: no gap), the bolt under that top (``bolt_under_top_mm``) and
    inside its width, so that the gauge can turn about the bolt only by sinking an edge
    of the bridge into the post (``held``); the post top's width across and each edge's
    lever from the bolt (mm)."""
    yp, zp = GAUGE_PIVOT
    _, _, ptop = GAUGE_POST
    z0, z1 = max(GAUGE_POST_OUT, GAUGE_BRIDGE[2]), -(FRONT_HALF + CLEAR)   # (under the bridge)
    under = ptop - yp
    levers = (zp - z0, z1 - zp)
    seated = abs(GAUGE_BRIDGE[0] - ptop) < 1e-9
    return {"seated": seated, "bolt_under_top_mm": round(under, 2),
            "top_width_mm": round(z1 - z0, 2), "edge_levers_mm": [round(v, 2) for v in levers],
            "held": seated and under > 0.0 and min(levers) > 0.0}


# --- the rear saddle: a V block on a steel post and a coarse lead screw ------------------

REAR_POST = (290.5, 110.0)    # the post (+z, 27 mm in front of the lead screw): (x, z)
REAR_LEAD = (317.5, 110.0)    # the lead screw (+z): (x, z), its knob outboard of a 6 L (|z|
#                               87) by 10 mm as it's laid in (review round 1, M2)
"""The rear V block is pushed up at the lead screw and guided by the post: a push off the
guide cocks the block on it (a drawer), and the post's friction grows with the push. The
post was at -z, 213 mm across from the screw, and the block jammed raising it whenever the
boss's friction on the post was over 0.149 (0.095 at a 750 ml's setting), under printed
plastic's 0.2-0.3 on steel (review round 2, N1); beside the screw it jams only past
``engagement / 54`` (:func:`cradle_report.drawers`)."""
REAR_FOOT_Z = (88.5, 103.5, 126.0)  # the carrier's side blocks across: inner faces, the -z
#                                     one's outer (it only rides the rods since the post
#                                     left it: 15 mm), the +z one's (the post and the screw)
REAR_FOOT_TOP = -121.0        # their tops
REAR_BRIDGE = (15.0, 6.0)     # the bridge joining them under the V block: along x, its height
#                               (10 until simplification round 1: the racking box forces its
#                               ends against each other, cradle_report._carriers, and a
#                               forced bar's stress grows with its depth: 1/2 in 6061 tie rods
#                               had taken a 10 mm one to 2.33 x across its layers, bar 2.5)
REAR_SET_Z = 103.0            # the M3 set screws up from the +z block onto the tie rods
REAR_BOSS = (12.0, -110.0, -57.0)   # the V block's boss round the post: radius, bottom, top
#                                     (at its lowest: over the carrier's collar)
LEAD_BOSS = (14.0, -120.5, -98.0)   # its boss round the lead screw, low beside the flats: the
#                                     knob over it stays under the axis's height, and the
#                                     knob's hub as low as the old knob's top (the cradle box
#                                     turned over rests on the base stop's knob, not on it:
#                                     cradle_pack.rest_poses)
LEAD_WEB = (7.0, 88.0, 96.0, 100.0, -98.0)  # that boss's web to the flats: half-length,
#                                             |z| from, to the flats' edge, to the boss, the
#                                             outer part's top (under the knob at full rise)
POST_WEB = (7.0, 6.0)         # the post's boss's web to the flats (half-length; |z| from and
#                               to as the lead screw's), and the two bosses' web between them
#                               (half-thickness across)
REAR_FLANGE = (-67.0, -55.0, 96.0)  # its flats along its top: bottom, top, half-width
VEE_T = 7.5                   # its V shell, square to the V (9 before review round 2: the
#                               lighter block keeps the stand's rear tip over 25°)
LEAD = 8.0                    # the lead screw: 4 starts, 2 mm apart, 8 mm a turn
LEAD_SCREW = (13.0, 150.0, 11.4, -146.0)   # its thread's diameter, length, root, foot
#                               (cradle y): printed in PETG, standing (simplification round
#                               1, 2026-10-21: a ServoCity Tr8 x 8 stainless screw, bronze
#                               barrel nut and aluminium collar until then). 13 mm passes the
#                               tower arm's bushing and the knob's teeth as the 8 mm screw
#                               did (16 mm would not: the arm, its teeth and the knob would
#                               all grow, and the screw couldn't go in through the arm)
LEAD_THREAD = (4, 0.8, 0.6, 30.0)   # its thread: starts, depth, crest (along the axis),
#                               flank angle off the radial plane (60° included: each flank
#                               overhangs 60° standing, 0.8 mm deep, as printed threads do)
LEAD_SPIGOT = 8.0             # its foot's spigot, in the carrier's bore under the pocket
LEAD_TOP = 10.0               # its top in the knob's hub (from the hub's bottom up)
LEAD_FLAT = (1.0, 12.0)       # a flat on its top (design review round 3, mounting m3; printed
#                               since simplification round 1): depth, length; the knob's hub's
#                               set screw bears on it, so the hub holds the screw by its shape
LEAD_NUT = (19.0, 16.0)       # its printed nut (PETG), up into the V block's boss from below:
#                               across its flats, length. It floats there (LEAD_FLOAT), so the
#                               screw carries the block's weight only and the post its roll
LEAD_FLOAT = 0.5              # the nut's play in its hex pocket, each side (its flats key it)
LEAD_PLAY = 0.25              # the nut's thread's clearance on the screw's, each side
THRUST = (21.0, 9.0)          # the printed thrust ring in the carrier's pocket, the screw's
#                               shoulder on it (the old collar's place): OD, height
BRAKE_T = 1.5                 # the TPU brake washer under the thrust ring
TOWER = (333.0, 351.0, 101.0, 119.0)    # the carrier's tower beside the screw: x, z spans
ARM = (307.0, -25.0, -17.0)   # its arm over the screw's top, a bushing round it: front x,
#                               bottom, top (over the boss at full rise)
ARM_PAD = 9.8                 # the arm's round pad round the screw (radius), under the teeth
#                               (clear of the lead screw boss's web at full rise)
LOCK_TEETH = (12, 10.0, 8.3, 3.0)   # the knob's lock (review round 2, N5): a star of teeth on
#                                     the arm's pad round the screw (points, outer and inner
#                                     radius, height); the knob drops over it
REAR_HUB = (16.0, -13.5, 4.0, (11.0, 2.0))  # the knob's hub on the screw's top (a hex across
#                                             its flats; bottom, top; its top flange: radius,
#                                             height), an M3 set screw onto the screw
REAR_KNOB = (13.0, -17.0, 0.0, 3.6)     # the knob: radius, bottom and top (down, on the arm),
#                                         its lift off the teeth (to turn it)
POST = (150.0, -141.0)        # the post: 5/8 in 6061-T6511 rod (simplification round 1,
#                               2026-10-21: no steel; a 15 mm MISUMI PETR15-150 nickel-plated
#                               steel standoff until then), cut 6 in by the supplier, faced to
#                               length, its flats milled and its foot tapped M6 on the Carvera:
#                               its length and foot. It takes the roll of a load hung at the
#                               screw, all of it since the nut floats
POST_KEY = "rod_6061_5_8_150"       # (hardware.cradle_catalog: Speedy Metals 2439)
POST_D = 15.875
POST_YIELD = 241.3            # MPa: 6061-T6511 rod's least (the Aluminum Association's Table
#                               11.1, extruded rod 0.250 in and over: 35 ksi)
POST_FLATS = (13.0, 10.0)     # its wrench flats at each end: across, along it (the boss rides
#                               the round between them)
POST_COLLAR = (4.0, -111.0)   # the carrier's collar round the post's socket: wall, top (the
#                               socket 30 mm deep, from the post's foot)
POST_M = M6                   # its foot's screw (an M8 until simplification round 1: one tap,
#                               the tie rods' M6, for every rod end)
POST_SCREW = 12.0             # M6 x 12 up into it from under the carrier, 8.6 mm in it
SEAT_R, SEAT_DEPTH = TIE_D / 2 + 0.15, 2.0  # the rear V block's seats on the tie rods
GUSSET = (45.0, -90.0)        # under its +z flat beside the bosses (from 22.5 in front of
#                               the post's to 22.5 behind the screw's): along x, its bottom
GROOVE = (77.0, 83.0, 3.0)    # the base stop's guide grooves along the flats and the rails:
#                               |z| span, depth
TSLOT = (293.0, 5.5, 5.5, (16.0, 1.6), 8.4, -74.0, -78.0)
"""The base stop's T-slot under the +z groove (the flats from its first x, then the +z rail):
its first x, the neck's width and the lips' thickness under the groove (5.5 mm: they take
the lock's preload at a firm hand's 1 N·m, review round 2, N6; they had been 5), the
washer's slot (width, height), the nut's width (the M5 nut can't turn in it) down to the
floor, the floor, the keel's bottom under it (the rails' tongues' tops too)."""
REAR_PEGS = ((280.0, -95.0), (355.0, -95.0))    # the rear liner's pegs (x, the face's y)
STOP_MARKS = (87.0, 88.0, 92.0)     # the base stop's scale across the +z flat and rail's top:
#                                     a 10 mm groove's inner end, a 5 mm one's, the outer
STOP_EVERY = 5.0              # its grooves, every 5 mm of a bottle's length (a long one every
#                               10); one dot in front of each 50 mm one, three, four and five
#                               in front of the 300, 400 and 500 mm ones
VERNIER = (6.0, 4, 4.0)       # the gate's pointer's vernier (review round 2, N2): its lines'
#                               pitch along the axis (the scale's and a millimetre: the k-th
#                               line from the gate's face meets a groove at a length k mm
#                               past one), how many, the pointer's width across (4 mm since
#                               round 3's mounting m2: 2 mm, stood on end to print, snapped
#                               at 3 N), its inner edge where it was, its outer over the
#                               rail's outer face; the lines are notches in that inner edge,
#                               full height, so each meets its groove at the rail's top (m4:
#                               cut in the pointer's top, 6 mm over the grooves, they read
#                               across a step, a millimetre's parallax at 10° off square)
POINTER_IN = 89.8             # the pointer's inner edge (|z|): the grooves show inside it


def stop_marks() -> list[tuple[float, int, bool]]:
    """The base stop's scale (:data:`STOP_EVERY`): each groove's x (cradle frame: a
    bottle's length plus :data:`STOP_GAP`, where the gate's face goes for it), its dots and
    whether it's a long one (every 10 mm). A plain length scale, the same for any bottle: a
    size's own length is just one reading of it (review round 2, N2: the grooves named for
    one maker's bottle of each size didn't fit another's). It starts as far in front of the
    gate's range as its vernier reaches."""
    pitch, lines, _ = VERNIER
    lo = math.floor((STOP_RANGE[0] - pitch * lines - STOP_GAP) / STOP_EVERY)
    hi = math.floor((STOP_RANGE[1] - STOP_GAP) / STOP_EVERY)
    out = []
    for k in range(lo, hi + 1):
        length = k * STOP_EVERY
        dots = round(length / 100) if length % 100 == 0 else 1 if length % 50 == 0 else 0
        out.append((length + STOP_GAP, dots, length % 10 == 0))
    return out


def _stop_scale(x0: float, x1: float, top: float) -> list[Shape3D]:
    """The base stop's scale (cradle frame) on the +z flat or rail's top (``top``, the V
    block at its lowest) from ``x0`` to ``x1``: :func:`stop_marks`' grooves across, their
    dots in front of them between the guide groove and the scale."""
    zl, zs, zo = STOP_MARKS
    w = 0.3
    out = []
    for x, dots, long in stop_marks():
        if not x0 + 1.0 <= x <= x1 - 1.0:
            continue
        out.append(_box(x - w, x + w, top - 0.6, top + 0.01, zl if long else zs, zo + 0.01))
        for k in range(dots):
            xd = x - 1.5 - DOT_PITCH * k
            if x0 + 1.0 <= xd <= x1 - 1.0:
                out.append(_rod_y(xd, (GROOVE[1] + zl) / 2, DOT[0], top - DOT[1], top + 0.01))
    return out


def _tslot(x0: float, x1: float) -> list[Shape3D]:
    """The base stop's T-slot (:data:`TSLOT`) along the +z groove from ``x0`` to ``x1``
    (cradle frame, the V block at its lowest)."""
    hi = REAR_FLANGE[1]
    g0, g1, depth = GROOVE
    zc = (g0 + g1) / 2
    _, neck, lip, (ww, wh), nw, _, _ = TSLOT
    y_lip = hi - depth - lip
    return [_box(x0, x1, y_lip - 0.01, hi - depth + 0.01, zc - neck / 2, zc + neck / 2),
            _box(x0, x1, y_lip - wh, y_lip, zc - ww / 2, zc + ww / 2),
            _box(x0, x1, TSLOT[5], y_lip - wh + 0.01, zc - nw / 2, zc + nw / 2)]


def _tslot_lead_in(x: float, inward: int) -> list[Shape3D]:
    """The T-slot's end at the face ``x`` chamfered :data:`SLOT_LEAD_IN` at 45° (cradle
    frame, the V block at its lowest; ``inward``: the part's side of the face, -1 or +1),
    each of its three channels widened by that at the face, tapering to its own size."""
    hi = REAR_FLANGE[1]
    g0, g1, depth = GROOVE
    zc = (g0 + g1) / 2
    _, neck, lip, (ww, wh), nw, floor, _ = TSLOT
    y_lip = hi - depth - lip
    c = SLOT_LEAD_IN
    out = []
    for y0, y1, w in ((y_lip, hi - depth, neck), (y_lip - wh, y_lip, ww),
                      (floor, y_lip - wh, nw)):
        rects = []
        for dx, grow in ((-inward * 0.01, c + 0.01), (inward * c, 0.0)):
            rects.append(Wire.make_polygon(
                [(x + dx, y0 - grow, zc - w / 2 - grow), (x + dx, y1 + grow, zc - w / 2 - grow),
                 (x + dx, y1 + grow, zc + w / 2 + grow), (x + dx, y0 - grow, zc + w / 2 + grow)],
                close=True))
        out.append(Solid.make_loft(rects))
    return out


@_part_memo
def _rear_saddle(s: Saddle, ties: Mapping[str, tuple[float, float]]
                 ) -> tuple[Shape3D, list[Shape3D]]:
    """The rear saddle's carrier (cradle frame): two side blocks on the two tie rods, joined
    under the V block by a bridge; the +z one holds the aluminium post (in a socket 30 mm
    deep, a collar round its top, an M6 button head up into it from below, its head sunk)
    and beside it the lead screw (a pocket for its printed thrust ring on a TPU brake
    washer, and a blind bore for its foot's spigot), and a tower beside the screw up to an
    arm over its top: a bushing round it (a knock at the knob goes into the tower, not the
    screw), a star of teeth on the arm round it that the knob drops over (its lock,
    :func:`_lead_screw`), and the scale on the tower's face, read by the lead screw boss's
    top edge (one groove per bottle, a dot each per place from the top, and the body's
    diameter every 10 mm); an M3 set screw up from the +z block onto each rod. (The
    carrier, its set screws.)"""
    x0, x1 = s.x
    xp, zp = REAR_POST
    xs, zs = REAR_LEAD
    za, zb, zc = REAR_FOOT_Z
    tx0, tx1, tz0, tz1 = TOWER
    ax, ab, at = ARM
    n_t, r_t, r_i, h_t = LOCK_TEETH
    part = union([_box(x0, x1, s.y0, REAR_FOOT_TOP, za, zc),
                  _box(x0, x1, s.y0, REAR_FOOT_TOP, -zb, -za),
                  _box(xs - REAR_BRIDGE[0] / 2, xs + REAR_BRIDGE[0] / 2, s.y0,
                       s.y0 + REAR_BRIDGE[1], -za - 0.5, za + 0.5),
                  _box(tx0, tx1, REAR_FOOT_TOP - 1.0, at, tz0, tz1),
                  _box(ax, tx1, ab, at, tz0, tz1),
                  _rod_y(xs, zs, ARM_PAD, ab, at),
                  _star_y(xs, zs, n_t, r_t, r_i, at - 0.01, at + h_t),
                  _rod_y(xp, zp, POST_D / 2 + 0.15 + POST_COLLAR[0], REAR_FOOT_TOP - 0.01,
                         POST_COLLAR[1])])
    top0 = LEAD_BOSS[2]                       # the reading edge, the block at its lowest
    named = sorted(top0 + (DESIGN.r - z.r) * math.sqrt(2) for z in SIZES)[::-1]
    graded = [top0 + y - s.top for y in _graded(s, DESIGN.r, 10.0, 70.0, 170.0)]
    graded = [y for y in graded if all(abs(y - n) > 1.5 for n in named)]
    floor = REAR_FOOT_TOP - 0.5 - THRUST[1] - BRAKE_T     # the thrust ring's pocket's floor
    gz = tz0 + 10.0                           # the named grooves' end; their dots after
    cuts = [_rod_y(xp, zp, POST_D / 2 + 0.15, POST[1], POST_COLLAR[1] + 1.0),
            _rod_y(xp, zp, POST_M.head_d / 2 + 0.3, s.y0 - 1.0, s.y0 + POST_M.head_h + 0.25),
            _rod_y(xp, zp, CLEARANCE[f"{POST_M.d:g}"] / 2, s.y0, POST[1] + 0.5),
            _rod_y(xs, zs, LEAD_SPIGOT / 2 + 0.2, LEAD_SCREW[3] - 0.5, floor + 0.01),
            _rod_y(xs, zs, THRUST[0] / 2 + 0.4, floor, REAR_FOOT_TOP + 1.0),
            _rod_y(xs, zs, LEAD_SCREW[0] / 2 + 0.2, ab - 1.0, at + h_t + 1.0),
            *[_box(tx0 - 0.01, tx0 + 0.8, y - 0.25, y + 0.25, tz0 - 1.0, gz) for y in named],
            *[_rod_x(y, gz + DOT_PITCH * (k + 0.75), DOT[0], tx0 - 0.01, tx0 + DOT[1])
              for i, y in enumerate(named) for k in range(i + 1)],
            *[_box(tx0 - 0.01, tx0 + 0.8, y - 0.25, y + 0.25, tz0 - 1.0, tz0 + 6.0)
              for y in graded]]
    if gz + DOT_PITCH * (len(named) + 0.25) > tz1:
        raise ValueError("the rear scale's dots don't fit on the tower's face")
    screws = []
    for n in s.ties:
        x, y = ties[n]
        cuts.append(Cylinder(TIE_D / 2 + 0.15, 2 * zc + 4).moved(Location((x, y, 0.0))))
        under = y - TIE_D / 2
        if under - CARRIER_SET_SCREW - 0.05 < s.y0:
            raise ValueError(f"{s.name}: an M3 x {CARRIER_SET_SCREW:g} set screw doesn't fit "
                             f"under "
                             f"tie {n} ({under - s.y0:.1f} mm)")
        cuts.append(_rod_y(x, REAR_SET_Z, SET_PILOT / 2, s.y0 - 2.0, under))
        screws.append(_rod_y(x, REAR_SET_Z, 1.25, under - 0.05 - CARRIER_SET_SCREW,
                             under - 0.05))
    return difference(part, *cuts), screws


@_part_memo
def _rear_vee(s: Saddle, ties: Mapping[str, tuple[float, float]]
              ) -> tuple[Shape3D, Shape3D, Shape3D]:
    """The rear V block at its lowest (cradle frame): a V shell (:data:`VEE_T`) sitting on
    the tie rods in a seat over each, its flats along the top (the base stop's guide groove
    along each, its T-slot under the +z one, its scale across the +z one), at the +z end a
    boss round the post (a sliding fit) and beside it a low one round the lead screw (its
    printed nut in a hex pocket from below, floating in it, :data:`LEAD_FLOAT`: its flats
    keep it from turning, the block bears down on it), webbed to each other and to the
    flats, a gusset under the +z
    flat beside them (they carry the roll of a load at the axis held at the screw), a
    socket under each flat at its back end for a rail's tongue, its TPU liner pegged into
    the V. (The block, its liner, the nut.)"""
    x0, x1 = s.vee
    xp, zp = REAR_POST
    xs, zs = REAR_LEAD
    lo, hi, w = REAR_FLANGE
    shell = cast(Shape3D, _box(x0, x1, s.bottom, hi, -w, w)
                 & _vee(x0, x1, s.apex - VEE_T * math.sqrt(2)))
    edge = lo - (s.apex - VEE_T * math.sqrt(2))         # (where the shell meets the flats)
    rp, rs = REAR_BOSS[0], LEAD_BOSS[0]
    gusset = _prism_x([(w, lo + 0.01), (w, GUSSET[1]), (edge, lo + 0.01)],
                      xp - GUSSET[0] / 2, xs + GUSSET[0] / 2)
    seats = [_box(ties[n][0] - 12.0, ties[n][0] + 12.0, ties[n][1] + TIE_D / 2 - SEAT_DEPTH,
                  s.bottom + 6.0, -12.0, 12.0) for n in s.ties]
    wx, wz0, wz1, wz2, wtop = LEAD_WEB
    px, pz = POST_WEB
    keel = TSLOT[6]
    sx0, sz0, sz1, _ = SOCKET
    sockets = [_box(sx0, x1, SOCKET_BOTTOM, lo + 0.01, *sorted((sign * sz0, sign * sz1)))
               for sign in (-1, 1)]
    block = union([shell, _box(x0, x1, lo, hi, -w, w), gusset, *seats,
                   _rod_y(xp, zp, rp, REAR_BOSS[1], REAR_BOSS[2]),
                   _rod_y(xs, zs, rs, LEAD_BOSS[1], LEAD_BOSS[2]),
                   _box(xs - wx, xs + wx, LEAD_BOSS[1], lo + 0.01, wz0, wz1),
                   _box(xs - wx, xs + wx, LEAD_BOSS[1], wtop, wz1 - 0.01, wz2),
                   _box(xp - px, xp + px, REAR_BOSS[1], REAR_BOSS[2], wz0, wz2),
                   _box(xp, xs, REAR_BOSS[1], LEAD_BOSS[2], zp - pz, zp + pz),
                   _box(x0, x1, keel, lo + 0.01, GROOVE[0] - 5.0, GROOVE[1] + 5.0),
                   *sockets])
    g0, g1, depth = GROOVE
    nd, nl = LEAD_NUT
    tx0, tz0, tz1, th = TONGUE
    cuts = [_rod_y(xp, zp, POST_D / 2 + 0.2, REAR_BOSS[1] - 1.0, REAR_BOSS[2] + 1.0),
            *[Cylinder(SEAT_R, 40.0).moved(Location((*ties[n], 0.0))) for n in s.ties],
            _rod_y(xs, zs, LEAD_SCREW[0] / 2 + 0.5, LEAD_BOSS[1] - 1.0, LEAD_BOSS[2] + 1.0),
            _hex_y(xs, zs, nd + 2 * LEAD_FLOAT, LEAD_BOSS[1] - 1.0, LEAD_BOSS[1] + nl),
            _box(x0 - 1.0, x1 + 1.0, hi - depth, hi + 1.0, g0, g1),
            _box(x0 - 1.0, x1 + 1.0, hi - depth, hi + 1.0, -g1, -g0),
            *_tslot(TSLOT[0], x1 + 1.0),
            *_tslot_lead_in(x1, -1),
            *_stop_scale(x0, x1, hi),
            *[_box(tx0 - CLEAR, x1 + 1.0, TONGUE_TOP - th, TONGUE_TOP + CLEAR,
                   *sorted((sign * (tz0 - CLEAR), sign * (tz1 + CLEAR))))
              for sign in (-1, 1)],
            *[_rod_y(x, sign * (tz0 + tz1) / 2, TIE_HOLE / 2, SOCKET_BOTTOM - 1.0,
                     TONGUE_TOP - th) for x in RAIL_SCREW_X for sign in (-1, 1)],
            *[_rod_y(x, sign * (tz0 + tz1) / 2, M5.head_d / 2 + 0.5,
                     SOCKET_BOTTOM - 1.0, SOCKET_BOTTOM + RAIL_HEAD_SINK)
              for x in RAIL_SCREW_X for sign in (-1, 1)]]
    vee, liner = _vee_block(s, difference(block, *cuts), REAR_PEGS)
    nut = difference(_hex_y(xs, zs, nd, LEAD_BOSS[1], LEAD_BOSS[1] + nl),
                     _rod_y(xs, zs, LEAD_SCREW[0] / 2 + LEAD_PLAY, LEAD_BOSS[1] - 1.0,
                            LEAD_BOSS[1] + nl + 1.0))
    return vee, liner, nut


@_part_memo
def _rear_nut_threaded() -> Shape3D:
    """The rear V block's printed nut as it prints (:func:`_rear_vee`'s, the block at its
    lowest): its bore threaded (:func:`_lead_thread`, :data:`LEAD_PLAY` out). The assembly
    holds its plain bore (``meta["print_shapes"]``: only its print file is threaded)."""
    xs, zs = REAR_LEAD
    nd, nl = LEAD_NUT
    return difference(_hex_y(xs, zs, nd, LEAD_BOSS[1], LEAD_BOSS[1] + nl),
                      _lead_thread(LEAD_BOSS[1] - 1.0, LEAD_BOSS[1] + nl + 1.0, LEAD_PLAY))


def _thread_profile(u: float, r_root: float, r_crest: float) -> float:
    """The lead screw's thread's radius ``u`` mm along the axis from a root's middle
    (:data:`LEAD_THREAD`: a symmetric trapezoid, one pitch long)."""
    starts, _, crest, flank_deg = LEAD_THREAD
    pitch = LEAD / starts
    flank = (r_crest - r_root) * math.tan(math.radians(flank_deg))
    root = pitch - crest - 2 * flank
    u = u % pitch
    for run, a, b in ((root / 2, r_root, r_root), (flank, r_root, r_crest),
                      (crest, r_crest, r_crest), (flank, r_crest, r_root)):
        if u < run:
            return a + (b - a) * u / run
        u -= run
    return r_root


LEAD_SECTION = 48             # the thread's cross-section's points round a turn (7.5° apart)


def _lead_thread(y0: float, y1: float, grow: float = 0.0) -> Shape3D:
    """The rear lead screw's threaded length up y from ``y0`` to ``y1`` at :data:`REAR_LEAD`
    (``grow``: its surface that much out all round, a nut's threaded bore): the thread's
    cross-section square to the axis (its radius round a turn, :func:`_thread_profile`,
    :data:`LEAD_THREAD`'s starts) swept up the axis turning a turn per :data:`LEAD` (a
    helical spine beside it): one sweep, no booleans, a right-hand thread. Its phase is
    the same at every ``y`` whatever ``y0``."""
    xs, zs = REAR_LEAD
    d, _, root_d, _ = LEAD_SCREW
    r_root, r_crest = root_d / 2 + grow, d / 2 + grow
    turn0 = 2 * math.pi * y0 / LEAD
    pts = []
    for i in range(LEAD_SECTION):
        th = 2 * math.pi * i / LEAD_SECTION
        r = _thread_profile(-LEAD * th / (2 * math.pi), r_root, r_crest)
        pts.append((r * math.cos(th + turn0), r * math.sin(th + turn0), 0.0))
    section = Face(Wire(Polyline(*pts, close=True).edges()))
    length = y1 - y0
    spine = Helix(pitch=LEAD, height=length, radius=r_crest + 5.0)
    spine = spine.rotate(BAxis.Z, math.degrees(turn0))
    solid = sweep(section, path=Wire([Line((0, 0, 0), (0, 0, length))]), binormal=spine)
    solid = solid if isinstance(solid, Solid) else solid.solids()[0]
    # its axis (z) up the cradle frame's y, at the screw
    return solid.rotate(BAxis.X, -90.0).moved(Location((xs, y0, zs)))


@dataclass(frozen=True)
class LeadScrew:
    """The rear thumbscrew's parts (:func:`_lead_screw`, cradle frame)."""

    screw: Shape3D
    thrust: Shape3D
    brake: Shape3D
    hub: Shape3D
    knob: Shape3D
    set_screw: Shape3D
    threaded: Shape3D           # the screw as it prints (``screw``: its thread's envelope)


@_part_memo
def _lead_screw() -> LeadScrew:
    """The rear thumbscrew (cradle frame): the printed lead screw (simplification round 1:
    PETG, standing; :data:`LEAD_SCREW`, :data:`LEAD_THREAD`), its foot's spigot
    (:data:`LEAD_SPIGOT`) in the carrier's bore, its shoulder on a printed thrust ring in
    the carrier's pocket on a TPU brake washer (the block's load on it slows the screw
    turning back), threaded up through the V block's floating nut to under the tower's arm,
    plain through the arm's bushing, and its top (:data:`LEAD_TOP`) in a printed hub (an M3
    set screw onto a flat printed on it, :data:`LEAD_FLAT`, out through the hub's +z flat)
    and the knob, which slides up and down the hub's hex: down, its underside's star socket
    sits over the star of teeth on the tower's arm and the screw can't turn (review round
    2, N5: an 8 mm lead turns back under the block's load); lifted :data:`REAR_KNOB`'s lift
    off the teeth, it turns the screw. The set screw's end rides a slot in the knob's wall,
    which reaches it. (:class:`LeadScrew`.)"""
    xs, zs = REAR_LEAD
    d, length, _, foot = LEAD_SCREW
    od, cl = THRUST
    top = REAR_FOOT_TOP - 0.5                  # the thrust ring's top: the screw's shoulder
    af, h0, h1, (fr, fh) = REAR_HUB
    kr, k0, k1, lift = REAR_KNOB
    n_t, r_t, r_i, h_t = LOCK_TEETH
    screw_top = foot + length
    thread_top = ARM[1] - 1.0                  # under the arm: the nut's highest reach
    rise = max(setting(z).rear for z in SIZES)
    if LEAD_BOSS[1] + LEAD_NUT[1] + rise > thread_top - 1.0:
        raise ValueError("the rear lead screw's thread ends under its nut's highest reach")
    ends = [_rod_y(xs, zs, LEAD_SPIGOT / 2, foot, top + 0.01),
            _rod_y(xs, zs, d / 2, thread_top - 0.01, h0 + 0.01),
            _rod_y(xs, zs, LEAD_TOP / 2, h0, screw_top)]
    screw_ = union([ends[0], _rod_y(xs, zs, d / 2, top, thread_top), *ends[1:]])
    threaded = union([ends[0], _lead_thread(top, thread_top), *ends[1:]])
    thrust = difference(_rod_y(xs, zs, od / 2, top - cl, top),
                        _rod_y(xs, zs, LEAD_SPIGOT / 2 + 0.2, top - cl - 1.0, top + 1.0))
    brake = difference(_rod_y(xs, zs, od / 2, top - cl - BRAKE_T, top - cl),
                       _rod_y(xs, zs, LEAD_SPIGOT / 2 + 0.3, top - cl - BRAKE_T - 1.0,
                              top - cl + 1.0))
    flat_d, flat_l = LEAD_FLAT
    z_flat = zs + LEAD_TOP / 2 - flat_d         # the flat's face, the set screw's tip on it
    flat = _box(xs - d, xs + d, screw_top - flat_l, screw_top + 1.0, z_flat, zs + d)
    screw_, threaded = difference(screw_, flat), difference(threaded, flat)
    y_set = (h0 + screw_top) / 2               # the set screw's height, on the screw's flat
    if y_set < screw_top - flat_l + 2.0:
        raise ValueError("the rear knob hub's set screw misses its lead screw's flat")
    hub = difference(union([_hex_y(xs, zs, af, h0, h1), _rod_y(xs, zs, fr, h1 - 0.01,
                                                                h1 + fh)]),
                     _rod_y(xs, zs, LEAD_TOP / 2 + 0.05, h0 - 1.0, screw_top + 0.3),
                     _rod_z(xs, y_set, SET_PILOT / 2, zs + LEAD_TOP / 2 - 0.5,
                            zs + af / 2 + 1.0))
    set_screw = _rod_z(xs, y_set, 1.25, z_flat + 0.05, z_flat + 0.05 + SET_SCREW)
    slot_r = 1.6                               # the slot round the set screw's end
    knob = difference(
        _rod_y(xs, zs, kr, k0, k1),
        _star_y(xs, zs, n_t, r_t + CLEAR, r_i + CLEAR, k0 - 0.01, k0 + h_t + CLEAR),
        _hex_y(xs, zs, af + 2 * CLEAR, k0 + h_t, k1 + 0.01),
        _box(xs - slot_r, xs + slot_r, y_set - lift - slot_r - 0.2, y_set + slot_r + 0.2,
             zs, zs + kr + 1.0))
    if not h0 + 6.0 <= screw_top <= h1 + fh - 1.0:
        raise ValueError("the rear lead screw's top isn't inside its knob's hub")
    if k1 + lift > h1 - 0.2:
        raise ValueError("the rear knob can't lift off its teeth under its hub's flange")
    if lift < h_t + 0.3:
        raise ValueError("the rear knob's lift doesn't clear its teeth")
    if z_flat + 0.05 + SET_SCREW > zs + kr - 1.0:
        raise ValueError("the rear knob hub's set screw stands out of the knob")
    return LeadScrew(screw_, thrust, brake, hub, knob, set_screw, threaded)


def _post_y(xz: tuple[float, float], y0: float, y1: float) -> Shape3D:
    """The rear post up y from ``y0`` to ``y1`` (:data:`POST`): a :data:`POST_D` 6061 rod,
    its foot tapped for :data:`POST_M`, its wrench flats at each end (:data:`POST_FLATS`,
    facing ±x)."""
    x, z = xz
    across, along = POST_FLATS
    r = POST_D / 2
    flats = [_box(min(xa, xb), max(xa, xb), a, b, z - r - 1.0, z + r + 1.0)
             for xa, xb in ((x - across / 2, x - r - 1.0), (x + across / 2, x + r + 1.0))
             for a, b in ((y0 - 1.0, y0 + along), (y1 - along, y1 + 1.0))]
    return difference(_rod_y(x, z, r, y0, y1), *flats,
                      _rod_y(x, z, POST_M.d * 0.42, y0 - 1, y0 + 12.0))


# --- the base stop: a gate on the rear V block's flats and the rails behind it -----------

RAIL_END = 600.0              # the rails' end (cradle x): the 6 L's gate locked short of it
RAIL_Z = 92.0                 # the rails' outer faces (their inner the V's edge)
FLANGE = (-91.0, 7.0)         # each rail's 45° flange continuing the V behind the block (the
#                               body of a 3 L or more rests on it: review round 1, B1): its
#                               bottom (cradle y; the rail's keel's and tongue's too: each
#                               rail stands on it on the print bed) and its thickness across
FLANGE_END = 580.0            # where the flanges, their liners and the keels end (cradle x):
#                               past a 6 L's heel (561), short of the rails' end, so that at
#                               rest nothing of the base stop comes nearer the base board than
#                               a finger's 25 mm (ISO 13854: the end of the flange had been
#                               18.8 mm over it)
RAIL_KEEL = 4.0               # each rail's outer keel (review round 2, N3): a web down its
#                               outer face to the flange's bottom, twice the rail's stiffness
RAIL_ROOT = 12.0              # the channel between flange and keel filled this far behind the
#                               block: the tongue's root
KEEL_CHAMFER = 5.0            # the +z keel's bottom end cut back at 45° this far, for the same
FINGER_GAP = 25.0             # ISO 13854's least gap for a finger
TONGUE = (351.0, 73.5, 90.5, 13.0)      # each rail's tongue into its socket under the flats:
#                                         its front x (the rail 249 mm long: a 250 mm bed),
#                                         |z| span, height
TONGUE_TOP = -78.0            # the tongues' tops (the V block at its lowest): under the +z
#                               flat's keel round the T-slot, the -z one level with it
RIBS = (3, 1.0, 0.5)          # crush ribs along a tongue's top (review round 2, N3): how many,
#                               their width and height, over the socket's 0.3 mm clearance
#                               above it: pressed in, they crush 0.2 mm, so the tongue sits in
#                               its socket with no play (it had 0.3 mm, 0.6° of rock)
SOCKET = (340.0, 70.5, 93.5, 10.7)      # the sockets: front x, |z| span, the floor under the
#                                         tongue (it sits on it: the rail's top is the flats')
SOCKET_BOTTOM = TONGUE_TOP - TONGUE[3] - SOCKET[3]      # a socket's underside
RAIL_SCREW_X = (357.0, 372.0)           # the two M5 x 16 up through each socket into its tongue
RAIL_SCREW = 16.0
RAIL_HEAD_SINK = 3.5                    # their heads sunk this far into the socket's floor
RAIL_DRAIN = 1.5                        # the drain's radius from the +z rail's T-slot's back end
#                                         down through its keel (spilt wine runs down the slot
#                                         at rest: review round 2, m6)
SLOT_LEAD_IN = 0.5            # the T-slot's ends at the block-to-rail joint chamfered this
#                               much, so the washer and nut cross it (review round 2, m8)
FLANGE_PEGS = ((430.0, -70.0), (540.0, -70.0))  # the flange liners' pegs (x, the face's y)
GATE_T = 10.0                 # the gate's plate along the axis
GATE_BOTTOM = -92.0           # its bottom edge (at the lowest): 5 mm under a 6 L's base
GATE_TOP = 9.0                # its top bar, ears and pointer over the rails (design review
#                               round 3, mounting M3: 6 mm, the clamped ear's root took the
#                               push's lever at 8.6-12 MPa, across its layers as it printed;
#                               9 mm, and the gate prints on its top, the root in its layers:
#                               12, the reviewer's, would want an M5 x 30 lock bolt)
EAR_END = 24.0                # its ears on the rails, behind its face
EAR_Z = 72.0                  # their inner faces (|z|; their outer the top bar's, STOP_MARKS)
EAR_FILLET = 4.0              # each ear's inside corner at the plate's back face, filleted
LOCK_X = 17.0                 # the lock's bolt through the +z ear, behind the face
LOCK_KNOB = (13.0, 9.5)       # its printed knob on the bolt's head: radius, height
LOCK_BOLT = 25.0              # M5 x 25 tap bolt: knob, ear, key, lips, washer, nut
LOCK_KEYS = ("m5_tap_bolt_25", "m5_washer_wide", "m5_nut")
LOCK_WASHER = (15.0, 1.2)     # the M5 fender washers (under the bolt's head, on the lips)


@_part_memo
def _rails(s: Saddle) -> tuple[list[Shape3D], list[Shape3D], list[Shape3D]]:
    """The base stop's rails (cradle frame, the V block at its lowest): two bars, one each
    side, continuing the rear V block's flats behind it, their guide grooves (the +z one's
    T-slot and scale too: its slot chamfered where it meets the block's, and drained at its
    back end), each with a 45° flange under it continuing the V to :data:`FLANGE`'s bottom
    (its TPU liner pegged on), a keel down its outer face (:data:`RAIL_KEEL`), the channel
    between them filled at its root, and a tongue forward into its socket under the block's
    flats (crush ribs along its top: no play), two M5 button heads up through the socket's
    floor into it. (The rails (-z, +z), their liners, their four screws.)"""
    hi = REAR_FLANGE[1]
    g0, g1, depth = GROOVE
    yb, ft = FLANGE
    keel = TSLOT[6]
    lo = REAR_FLANGE[0]
    xr = s.vee[1]
    tx0, tz0, tz1, th = TONGUE
    n_r, rw, rh = RIBS

    def zv(y: float) -> float:                 # the V's face (the liners' backs) at y
        return y - s.apex

    bar_zy = [(zv(hi), hi), (RAIL_Z, hi), (RAIL_Z, lo), (zv(lo), lo)]
    deep_zy = [(zv(lo), lo + 0.01), (RAIL_Z, lo + 0.01), (RAIL_Z, yb), (RAIL_Z - RAIL_KEEL, yb),
               (RAIL_Z - RAIL_KEEL, lo), (zv(lo) + ft, lo), (zv(yb) + ft, yb), (zv(yb), yb)]
    rails, liners, screws = [], [], []
    for sign in (-1, 1):
        sz = sorted((sign * tz0, sign * tz1))
        parts = [_prism_x([(sign * z, y) for z, y in bar_zy], xr, RAIL_END),
                 _prism_x([(sign * z, y) for z, y in deep_zy], xr, FLANGE_END),
                 _box(xr, xr + RAIL_ROOT, yb, lo + 0.01, *sorted((sign * zv(lo),
                                                                   sign * RAIL_Z))),
                 _box(tx0, xr + 0.5, TONGUE_TOP - th, TONGUE_TOP + 0.01, *sz)]
        pitch = (tz1 - tz0 - rw) / (n_r - 1)
        parts += [_box(tx0, xr + 0.5, TONGUE_TOP, TONGUE_TOP + rh,
                       *sorted((sign * (tz0 + i * pitch), sign * (tz0 + i * pitch + rw))))
                  for i in range(n_r)]
        zk = sorted((sign * (GROOVE[0] - 5.0), sign * (GROOVE[1] + 5.0)))
        if sign > 0:                           # the keel round the T-slot
            parts.append(_box(xr, RAIL_END, keel, lo + 0.01, *zk))
        k = KEEL_CHAMFER
        cuts = [_box(xr - 1.0, RAIL_END + 1.0, hi - depth, hi + 1.0,
                     *sorted((sign * g0, sign * g1)))]
        if sign > 0:
            chamfer = Solid.extrude(Face(Wire.make_polygon(
                [(RAIL_END - k, keel - 0.01, zk[0] - 1.0), (RAIL_END + 0.01, keel - 0.01,
                                                             zk[0] - 1.0),
                 (RAIL_END + 0.01, keel + k, zk[0] - 1.0)], close=True)),
                (0, 0, zk[1] - zk[0] + 2.0))
            x_end = RAIL_END - 3.0             # the T-slot's closed back end, low at rest
            cuts += [*_tslot(xr - 1.0, x_end), *_tslot_lead_in(xr, 1),
                     *_stop_scale(xr, RAIL_END, hi), chamfer,
                     _rod_y(x_end - RAIL_DRAIN, (g0 + g1) / 2, RAIL_DRAIN, keel - 6.0,
                            TSLOT[5] + 0.01)]
        zt = sign * (tz0 + tz1) / 2
        y_head = SOCKET_BOTTOM + RAIL_HEAD_SINK    # the screws' heads' tops, sunk in the floor
        cuts += [_rod_y(x, zt, M5_PILOT / 2, TONGUE_TOP - th - 1.0, y_head + RAIL_SCREW + 1.0)
                 for x in RAIL_SCREW_X]
        band = [(sign * zv(hi), hi), (sign * (zv(hi) - LINER * math.sqrt(2)), hi),
                (sign * (zv(yb) - LINER * math.sqrt(2)), yb), (sign * zv(yb), yb)]
        liner = _prism_x(band, xr + CLEAR, FLANGE_END - CLEAR)
        keys, holes = _liner_pegs(s.apex, FLANGE_PEGS, (sign,))
        rails.append(difference(union(parts), *cuts, *holes))
        liners.append(union([liner, *keys]))
        screws.extend(union([_rod_y(x, zt, M5.head_d / 2, y_head - M5.head_h, y_head),
                             _rod_y(x, zt, 2.4, y_head, y_head + RAIL_SCREW)])
                      for x in RAIL_SCREW_X)
    return rails, liners, screws


@_part_memo
def _gate(s: Saddle, stop: float) -> tuple[Shape3D, list[Shape3D]]:
    """The base stop's gate, its face at ``stop`` (cradle frame, the V block at its lowest):
    a plate across the V behind the bottle's base (a millimetre inside the liners' face,
    from :data:`GATE_BOTTOM` up), its top on the flats or the rails (:data:`GATE_TOP` over
    them) with a pointer forward along the scale's outer half (its back end the face's
    plane reads the scale; a vernier of :data:`VERNIER` lines notched in its inner edge
    reads the millimetres), an ear on each behind it keyed into its groove, its root's
    inside corner filleted (:data:`EAR_FILLET`); the lock through the +z ear: an M5 tap
    bolt down through a printed knob (its head and a fender washer in the knob), the ear
    and its key into a fender washer and an M5 nut in the T-slot under the groove. Turned
    by hand it clamps the gate anywhere along the slot: no tool, no thread in the plastic.
    (The gate; the lock's bolt, knob, washers (on the ear, under the lips) and nut.)"""
    hi = REAR_FLANGE[1]
    g0, g1, depth = GROOVE
    face = s.apex + LINER * math.sqrt(2)

    def zf(y: float) -> float:
        return y - face - 1.0

    top = hi + GATE_TOP
    ze = STOP_MARKS[0]                         # the ears' and the top bar's outer edges
    zy = [(-zf(GATE_BOTTOM), GATE_BOTTOM), (zf(GATE_BOTTOM), GATE_BOTTOM), (zf(hi), hi),
          (ze, hi), (ze, top), (-ze, top), (-ze, hi), (-zf(hi), hi)]
    z_ear = EAR_Z
    ears = [_box(stop + GATE_T - 0.01, stop + EAR_END, hi, top, za, zb)
            for za, zb in ((z_ear, ze), (-ze, -z_ear))]
    f = EAR_FILLET                             # each ear's root's inside corner, filleted
    x_r = stop + GATE_T
    for sign in (-1, 1):
        zc = sign * (z_ear - f)
        corner = _box(x_r - 0.01, x_r + f, hi, top, *sorted((zc, sign * (z_ear + 0.01))))
        ears.append(difference(corner, _rod_y(x_r + f, zc, f, hi - 1.0, top + 1.0)))
    keys = [_box(stop, stop + EAR_END, hi - depth + CLEAR, hi + 0.01, za, zb)
            for za, zb in ((g0 + CLEAR, g1 - CLEAR), (-g1 + CLEAR, -g0 - CLEAR))]
    pitch, lines, pw = VERNIER
    z_in, z_out = POINTER_IN, POINTER_IN + pw
    reach = pitch * lines + 1.5                # its arm forward of the face
    pointer = union([_box(stop, stop + 3.0, hi, top, ze - 0.01, z_out),
                     _box(stop - reach, stop + 0.01, hi, top, z_in, z_out)])
    vernier = [_box(stop - pitch * k - 0.25, stop - pitch * k + 0.25, hi - 0.01, top + 0.01,
                    z_in - 0.01, z_in + 0.6) for k in range(1, lines + 1)]
    xl, zl = stop + LOCK_X, (g0 + g1) / 2
    gate = difference(union([_prism_x(zy, stop, stop + GATE_T), *ears, *keys, pointer]),
                      _rod_y(xl, zl, TIE_HOLE / 2, hi - depth - 1.0, top + 1.0), *vernier)
    kr, kh = LOCK_KNOB
    wd, wt = LOCK_WASHER
    af, head_h, _ = M5_HEX
    y_h = top + wt                             # the bolt head's underside, on its washer
    washer0 = difference(_rod_y(xl, zl, wd / 2, top, y_h),
                         _rod_y(xl, zl, 2.65, top - 1.0, y_h + 1.0))
    bolt = union([_hex_y(xl, zl, af, y_h, y_h + head_h),
                  _rod_y(xl, zl, 2.4, y_h - LOCK_BOLT, y_h)])
    knob = difference(_rod_y(xl, zl, kr, y_h, y_h + kh),     # (pressed down onto the head)
                      _hex_y(xl, zl, af + CLEAR, y_h - 1.0, y_h + head_h + 0.2))
    _, _, lip, _, _, floor_, _ = TSLOT
    y_lip = hi - depth - lip
    washer1 = difference(_rod_y(xl, zl, wd / 2, y_lip - wt, y_lip),
                         _rod_y(xl, zl, 2.65, y_lip - wt - 1.0, y_lip + 1.0))
    y_nut = y_lip - wt - M5_NUT[1]
    nut = difference(_hex_y(xl, zl, M5_NUT[0], y_nut, y_lip - wt),
                     _rod_y(xl, zl, 2.5, y_nut - 1.0, y_lip - wt + 1.0))
    end = y_h - LOCK_BOLT
    if not floor_ + 0.3 <= end <= y_nut - 0.8:
        raise ValueError("the base stop's lock bolt doesn't end past its nut in the T-slot")
    return gate, [bolt, knob, washer0, washer1, nut]


@_part_memo
def _led(front: Saddle) -> tuple[Shape3D, Shape3D, Shape3D, list[Shape3D], list[Shape3D]]:
    """The penlight under the shoulder and its clip, in the cradle's frame: the light stands
    behind the front saddle, its head up under the shoulder; the clip, a part of its own
    (printed standing, its tube upright), holds its body below the head (which twists to
    switch it on) on a lip at the bottom, an M3 set screw through a boss on its back holding
    it from turning. A plate behind the saddle carries it, its flange under the carrier's
    flat underside (behind the front tie's set screw) held by two M5 button heads up into
    printed pilots. (The light, the clip, the set screw, the clip's screws, the saddle's
    pilots for them.)"""
    d, length = PENLIGHT
    x, head = LED_X, LED_TOP
    up = (90, 0, 0)                          # a cylinder's axis along y
    light = Cylinder(d / 2, length, rotation=up).moved(Location((x, head - length / 2, 0.0)))
    clip_top, clip_bot = head - CLIP_GAP, head - length     # (the lip under the light's tail)
    tube = Cylinder(d / 2 + 2.5, clip_top - clip_bot + 2.0, rotation=up).moved(
        Location((x, (clip_top + clip_bot - 2.0) / 2, 0.0)))
    tube = tube - Cylinder(d / 2 + 0.2, clip_top - clip_bot + 0.01, rotation=up).moved(
        Location((x, (clip_top + clip_bot) / 2 + 0.005, 0.0)))
    tube = tube - Cylinder(d / 2 - 2.0, 10.0, rotation=up).moved(     # the lip's hole
        Location((x, clip_bot - 2.0, 0.0)))
    boss_y = (clip_top + clip_bot) / 2
    rib = boss_y + 4.0 - (clip_bot - 2.0)        # the set screw's boss, a rib down to the foot
    boss = Box(SET_SCREW + 1.0, rib, 8.0).moved(
        Location((x + d / 2 + (SET_SCREW + 1.0) / 2 - 0.5, boss_y + 4.0 - rib / 2, 0.0)))
    pilot = Cylinder(SET_PILOT / 2, SET_SCREW + 4.0, rotation=(0, 90, 0)).moved(
        Location((x + d / 2 + SET_SCREW / 2, boss_y, 0.0)))
    # the plate: from the saddle's back face to the tube, its lower front cut away at 45°
    # (over a decanter's shoulder at rest), its flange forward under the saddle on a 45°
    # gusset
    x1, y0, half = front.x[1], front.y0, CLIP_W / 2
    xs = x - d / 2 - 1.5                     # (short of the light's head)
    yb = clip_bot - 2.0
    prof = [(x1, y0), (xs, y0), (xs, yb + CLIP_CUT - (xs - x1)), (x1, yb + CLIP_CUT),
            (x1, y0 - CLIP_FLANGE - (x1 - CLIP_X0)), (CLIP_X0, y0 - CLIP_FLANGE),
            (CLIP_X0, y0)]
    plate = Solid.extrude(Face(Wire.make_polygon([(px, py, -half) for px, py in prof],
                                                 close=True)), (0, 0, 2 * half))
    screws, pilots, bores = [], [], []
    for z in (-CLIP_SCREW_Z, CLIP_SCREW_Z):
        at = Location((CLIP_SCREW_X, 0.0, z))
        down = (90, 0, 0)                    # a cylinder along y
        through = CLIP_FLANGE + CLIP_BITE     # (a stock M5 length: CLIP_SCREW)
        screws.append(union([
            Cylinder(M5.head_d / 2, M5.head_h, rotation=down).moved(
                at * Location((0, y0 - CLIP_FLANGE - M5.head_h / 2, 0))),
            Cylinder(2.4, through, rotation=down).moved(
                at * Location((0, y0 - CLIP_FLANGE + through / 2, 0)))]))
        pilots.append(Cylinder(M5_PILOT / 2, CLIP_BITE + 1.0, rotation=down).moved(
            at * Location((0, y0 + (CLIP_BITE + 1.0) / 2 - 0.01, 0))))
        bores.append(Cylinder(TIE_HOLE / 2, CLIP_FLANGE + 2, rotation=down).moved(
            at * Location((0, y0 - CLIP_FLANGE / 2, 0))))
        bores.append(Cylinder(M5.head_d / 2 + 0.5, 80.0, rotation=down).moved(  # its head's
            at * Location((0, y0 - CLIP_FLANGE - 40.0, 0))))                    # way in
    screw = Cylinder(1.25, SET_SCREW, rotation=(0, 90, 0)).moved(
        Location((x + d / 2 + 0.05 + SET_SCREW / 2, boss_y, 0.0)))
    clip = difference(union([tube, boss, plate]), pilot, *bores, light)
    return light, clip, screw, screws, pilots


CLIP_W = 28.0               # the light clip's plate, across the bay
CLIP_X0 = 77.0              # its flange's front edge (the front tie's set screw at 75: open)
CLIP_FLANGE = 5.0           # the flange's thickness under the saddle
CLIP_CUT = 33.0             # the plate's lower front, cut at 45° this far up from the tube's
#                             foot at the saddle's back face (clear of a decanter at rest)
CLIP_SCREW_X, CLIP_SCREW_Z = 82.5, 7.5   # its two M5 screws (their pilots 1.2 mm clear of
#                                          the front tie's bore)
CLIP_BITE = 7.0             # their thread in the saddle
CLIP_SCREW = M5.key(CLIP_FLANGE + CLIP_BITE)    # M5 x 12
M5_PILOT = 4.2              # a printed pilot an M5 cuts its own thread in


PENLIGHT = (14.0, 60.5)     # the light: Olight i3E EOS, a AAA keychain light (d, length):
#                             short, so its tail clears a decanter's shoulder at rest
LED_X = 100.0               # its axis, in the cradle's frame: under the shoulders' start (a
#                             750's shoulder, a 6 L's neck); design review round 3, mounting
#                             M2: at 118 univerre's real 6 L impériale, its shoulder 27 mm
#                             fuller there than the model's, lay on its head (:func:`bottle_room`)
LED_TOP = -45.0             # and its head, 11-27 mm under the glass there
CLIP_GAP = 16.0             # the clip starts this far under the head (the head twists)


@_part_memo
def _bracket(xy, y_top: float, face: float, sign: int, t_frame: float, board: bool = False
             ) -> tuple[Shape3D, list[tuple[Shape3D, str | None, str]], tuple[float, float]]:
    """A foot's printed L-bracket outside an outer plate (world): its upright leg on the
    plate's outer face (``face``; ``sign`` outward along z), its foot leg on the base board
    (its top ``y_top``); an M5 button head through the upright and the plate into a nut over
    the plate, a carriage bolt up from under the base board through the foot's oversize hole
    (:data:`BRACKET_HOLE`), an 18-8 fender washer and a printed thumb knob on it, an 18-8 M6
    nut pressed into the knob (:data:`KNOB`: by hand, so the side lifts off the board with no
    tool; the bolt's head in the feet's clearance, :data:`BOLT_OUT` off the plate so the knob
    clears the upright's screw head). (The bracket; its screws, nuts, washer and knob, each
    with its catalog key (the knob's ``None``: printed) and its name's suffix; the base
    board's hole (x, z). A foot on a raised rail, :func:`rail_y`, raises its upright as much
    over its screw.)

    ``board`` (:data:`BOARD_BRACKETS`): the bracket stays on the board. The plate's M5 is a
    stud, its button head in the layer over the plate (where the nut was) and its jam nut
    on the plate's outer face, in the upright's slot, open at the top (:data:`STUD_SLOT`);
    a printed knob on the stud (:data:`STUD_KNOB`, an M5 nut pressed into it from outside,
    its floor bearing on the upright either side of the slot) clamps the upright to the
    plate; the bracket's foot is held down by a plain M6 nut on its washer, no knob."""
    length, height, wall = BRACKET
    fx, fy = xy
    tall = height + max(0.0, fy - (y_top + _FOOT_Y - BASE_TOP))
    zs = sorted((face, face + sign * wall))
    up = Box(length, tall, wall).moved(Location((fx, y_top + tall / 2, sum(zs) / 2)))
    zf = sorted((face, face + sign * height))
    foot = Box(length, wall, height).moved(Location((fx, y_top + wall / 2, sum(zf) / 2)))
    zb = face + sign * BOLT_OUT                      # the foot leg's bolt
    hole_up = (_stud_slot(fx, fy, y_top + tall, zs) if board
               else disc((fx, fy), TIE_HOLE / 2, zs[0] - 1, zs[1] + 1))
    hole_down = _rod_y(fx, zb, BRACKET_HOLE / 2, y_top - 1.0, y_top + wall + 1.0)
    bracket = difference(union([up, foot]), hole_up, hole_down)
    inner = face - sign * t_frame
    side: list[tuple[Shape3D, str | None, str]]
    if board:       # a stud from the plate's inside, its jam nut and its knob outside
        k_d, floor, _ = STUD_KNOB
        l_side = _m5_length(t_frame + wall + floor + M5_NUT[1])
        z_k = face + sign * wall                    # the knob's floor on the upright
        z_n = z_k + sign * floor                    # its nut, pressed in from outside
        if (BOLT_OUT - M6_HEX[0] / math.sqrt(3) < wall + floor + M5_NUT[1] + 1.0
                or fy - k_d / 2 < y_top + wall + 1.0 or fy + k_d / 2 > y_top + tall):
            raise ValueError("the stud's knob doesn't clear the foot's nut, its foot leg or "
                             "the upright's top")
        side = [(_screw((fx, fy), inner, -sign, l_side, M5), M5.key(l_side), "screw0"),
                (_nut((fx, fy), *sorted((face, face + sign * M5_NUT[1]))), "m5_nut", "nut0"),
                (_stud_knob(fx, fy, z_k, sign), None, "knob0"),
                (_nut((fx, fy), *sorted((z_n, z_n + sign * M5_NUT[1]))), "m5_nut", "nut2")]
    else:           # through the upright and the plate, the nut over the plate's inner face
        l_side = _m5_length(wall + t_frame + M5_NUT[1])
        nut_z = sorted((inner, inner - sign * M5_NUT[1]))
        side = [(_screw((fx, fy), face + sign * wall, sign, l_side, M5), M5.key(l_side),
                 "screw0"), (_nut((fx, fy), *nut_z), "m5_nut", "nut0")]
    # a carriage bolt up from under the base board (its square neck in the board's square
    # hole: it can't turn), through the foot, its washer and knob on the foot from above
    d, blen, head_d, neck_h, head_h = CARRIAGE
    y_under = y_top - BASE_T
    if y_under - head_h < y_top - BASE_TOP_CLEAR:
        raise ValueError("the base board's carriage bolts' heads reach the table")
    w_id, w_od, w_t = WASHER
    af, nut_h = M6_HEX[0], (FOOT_NUT[1] if board else M6_HEX[1])   # (B's: nylon-insert)
    knob_d = KNOB[0]
    if neck_h > BASE_T or BASE_T + wall + w_t + nut_h + 1.0 > blen:
        raise ValueError("the carriage bolt doesn't reach through the board, the foot, its "
                         "washer and its nut, a thread past it")
    if not board and BOLT_OUT - knob_d / 2 < wall + M5.head_h + 1.0:
        raise ValueError("the thumb knob would hit the upright's screw head")
    if w_od / 2 - (BRACKET_HOLE - d) / 2 < BRACKET_HOLE / 2 + 1.0:
        raise ValueError("the washer doesn't cover the bracket's oversize hole")
    head = _rod_y(fx, zb, head_d / 2, y_under - head_h, y_under)
    shank = _rod_y(fx, zb, d / 2 * 0.97, y_under, y_under + blen)
    y_w = y_top + wall                              # the washer, on the foot
    washer = difference(_rod_y(fx, zb, w_od / 2, y_w, y_w + w_t),
                        _rod_y(fx, zb, w_id / 2, y_w - 1.0, y_w + w_t + 1.0))
    y_n = y_w + w_t                                 # the nut and the knob, on the washer
    nut = difference(_hex_y(fx, zb, af, y_n, y_n + nut_h),
                     _rod_y(fx, zb, d / 2, y_n - 1.0, y_n + nut_h + 1.0))
    base: list[tuple[Shape3D, str | None, str]] = [
        (union([head, shank]), f"m6_carriage_{blen:g}", "screw1"),
        (washer, WASHER_KEY, "washer1"), (nut, FOOT_NUT[0] if board else "m6_nut", "nut1")]
    if not board:
        base.append((_knob(fx, zb, y_n), None, "knob"))
    return bracket, side + base, (fx, zb)


def _stud_slot(x: float, y: float, top: float, zs) -> Shape3D:
    """A board bracket's slot (world): :data:`STUD_SLOT` wide through the upright (z
    ``zs``) from over its top down to a round bottom under the stud's jam nut at ``(x,
    y)``, its corners 0.3 mm over it (the rail then 0.2 mm over the board); its mouth
    chamfered :data:`SLOT_LEAD` at 45° either side, so a jam nut lowered off by that much
    finds its slot (design review round 3's stand and use, M1 and m11)."""
    r = STUD_SLOT / 2
    c = y - M5_NUT[0] / math.sqrt(3) - 0.3 + r     # the bottom's centre
    z0, z1 = zs[0] - 1.0, zs[1] + 1.0
    lead = r + SLOT_LEAD
    mouth = Solid.extrude(Face(Wire.make_polygon(
        [(x - r, top - SLOT_LEAD, z0), (x + r, top - SLOT_LEAD, z0), (x + lead + 1.0, top + 1.0,
         z0), (x - lead - 1.0, top + 1.0, z0)], close=True)), (0, 0, z1 - z0))
    return union([disc((x, c), r, z0, z1),
                  _box(x - r, x + r, c, top + 1.0, z0, z1), mouth])


def _stud_knob(x: float, y: float, z0: float, sign: int) -> Shape3D:
    """A board bracket's stud's printed knob (world, along z from ``z0``, the upright's
    outer face, ``sign`` outward): fluted, :data:`STUD_KNOB`'s floor on the upright, then a
    hex pocket open outward that an 18-8 M5 nut is pressed into, the stud's bore through."""
    d, floor, flutes = STUD_KNOB
    z1 = z0 + sign * (floor + M5_NUT[1])
    zk = sorted((z0, z1))
    zp = sorted((z0 + sign * floor, z1 + sign))
    r = (M5_NUT[0] + 2 * KNOB_PRESS) / math.sqrt(3)
    pts = [(x + r * math.cos(math.pi / 3 * i), y + r * math.sin(math.pi / 3 * i), zp[0])
           for i in range(6)]
    pocket = Solid.extrude(Face(Wire.make_polygon(pts, close=True)), (0, 0, zp[1] - zp[0]))
    cuts = [pocket, disc((x, y), 2.75, zk[0] - 1.0, zk[1] + 1.0)]
    cuts += [disc((x + d / 2 * math.cos(a), y + d / 2 * math.sin(a)), 1.8, zk[0] - 1.0,
                  zk[1] + 1.0) for a in np.linspace(0.0, 2 * math.pi, flutes, endpoint=False)]
    return difference(disc((x, y), d / 2, *zk), union(cuts))


def _knob(x: float, z: float, y0: float) -> Shape3D:
    """A base board bolt's printed thumb knob (world): fluted, standing on its washer at
    ``y0``, its 18-8 M6 nut pressed up into a hex pocket from its underside (the nut bears on
    the washer; the knob only turns it), the bolt's end in its bore."""
    d, h, flutes = KNOB
    cuts = [_hex_y(x, z, M6_HEX[0] + 2 * KNOB_PRESS, y0 - 1.0, y0 + M6_HEX[1]),
            _rod_y(x, z, 3.3, y0 - 1.0, y0 + h + 1.0)]
    cuts += [_rod_y(x + d / 2 * math.cos(a), z + d / 2 * math.sin(a), 1.6, y0 - 1.0,
                    y0 + h + 1.0) for a in np.linspace(0.0, 2 * math.pi, flutes, endpoint=False)]
    return difference(_rod_y(x, z, d / 2, y0, y0 + h), union(cuts))


CARRIAGE = (6.0, 25.0, 16.55, 4.6, 3.88)   # the base board's bolts (DIN 603 M6 x 25, Bolt
#                                            Depot 18221): d, length, head d, neck h, head h
#                                            (a washer, a nut and a thread past it)
WASHER = (6.4, 18.0, 1.6)  # an 18-8 M6 fender washer under each knob (Bolt Depot 7315): id,
#                            od, t; it spreads the nut's clamp over 200 mm^2 of the printed
#                            foot (the thumb nut's 12 mm collar had 81 mm^2 on bare PLA)
WASHER_KEY = "m6_washer_wide"
KNOB = (24.0, 10.5, 12)    # the thumb knob: printed, across, tall (its top a millimetre over
#                            the bolt's end), flutes round it; an 18-8 M6 nut (DIN 934, Bolt
#                            Depot 4776) in it, so no thread is plastic and nothing rusts in a
#                            wine bar (the black-oxide thumb nut it replaced)
KNOB_PRESS = 0.05          # its hex pocket's clearance each flat: the nut presses in
FOOT_NUT = ("m6_nylon_nut", 6.0, 1.5)
"""A board bracket's foot nut (:data:`BOARD_BRACKETS`; design review round 3's stand and use,
m4): an 18-8 M6 nylon-insert nut (Bolt Depot 4795, 6 mm high, the plain DIN 934's USD 0.11),
snugged once to this torque (N·m) and never again: 1.25 kN at a nut factor of 0.2, 6.3 MPa
on the printed foot under the 18 mm fender washer, half the sustained PLA bearing rating (a
wrench's "snug", 2-5 N·m, had been 8-21 MPa, and nothing held the nut: a bracket turned on
its one bolt misses its stud next trip). Set with the luggage scale on the 10 mm wrench."""
BOLT_OUT = 22.0            # the bolt this far out from the outer plate's face (19 under the
#                            thin hex nut it had): the knob clears the upright's M5 screw head
#                            by 2.25 mm (a wing nut's 28-33 mm wings would hit it)
BRACKET_HOLE = 8.5         # the bolt's hole in a bracket's foot: 1.25 mm of play round it, so
#                            a side settles where the linkage puts it before its knobs are
#                            snug (B's two remote centres must agree: round 1's linkage review)
CARRIAGE_HOLE = 6.7        # the board's hole for its square neck (6.48 across its flats): a
#                            routed square, a dog-bone in each corner (DOG_BONE_R), so the neck
#                            locks by its shape (a round hole had the neck's corners crush 1.4
#                            mm of birch each: a mallet, and split face veneers)


def _m5_length(grip: float) -> float:
    """The shortest stock M5 button head through ``grip`` mm with a turn of thread to spare."""
    return next(n for n in M5.lengths if n >= grip + 0.5)


def _nut(xy, z0: float, z1: float) -> Shape3D:
    """An M5 nut along z (drawn round, its across-flats)."""
    return difference(disc(xy, M5_NUT[0] / 2, z0, z1), disc(xy, 2.5, z0 - 1, z1 + 1))


def _rounded(x0: float, x1: float, z0: float, z1: float, y0: float, y1: float, r: float
             ) -> Shape3D:
    """A plate from ``x0`` to ``x1`` and ``z0`` to ``z1`` (world), ``y0`` to ``y1`` thick,
    its four corners rounded ``r``."""
    return union([_box(x0 + r, x1 - r, y0, y1, z0, z1), _box(x0, x1, y0, y1, z0 + r, z1 - r),
                  *(_rod_y(x, z, r, y0, y1) for x in (x0 + r, x1 - r) for z in (z0 + r, z1 - r))])


def _square_hole(x: float, z: float, side: float, y0: float, y1: float) -> Shape3D:
    """A routed square hole's cutter (world, up y): ``side`` across, square to x and z, a
    dog-bone relief of the bit's radius (:data:`DOG_BONE_R`) in each corner, the corner on
    its circle, so a square part seats in it."""
    h = side / 2
    c = h - DOG_BONE_R / math.sqrt(2)
    return union([_box(x - h, x + h, y0, y1, z - h, z + h),
                  *(_rod_y(x + sx * c, z + sz * c, DOG_BONE_R, y0, y1)
                    for sx in (-1, 1) for sz in (-1, 1))])


@_part_memo
def _base_plate(lip, stand: Stand, y_top: float, holes: list[tuple[float, float]],
                z_mid: float = 0.0) -> tuple[Shape3D, list[tuple[float, float]]]:
    """The base board (world): from the stand's front to its rear edge, :data:`BASE_HALF`
    each side of ``z_mid`` (:func:`base_z`), under both sides' outer plates, its corners
    rounded (:data:`BASE_CORNER_R`); a notch from its front edge round the decanter (its
    round end under the mouth, its mouth's corners rounded, a bay each side for the jack
    locator's ears, the bays' points rounded: :data:`BAY_POINT_R`), a window under the base
    stop (:data:`BOARD_WINDOW`, about the bay's
    middle), the brackets' carriage bolts' square holes, a hole near each corner for a TPU
    foot. (The board, the feet's (x, z).)"""
    t = BASE_T
    x0, x1 = float(lip[0]) + stand.base[0], float(lip[0]) + stand.base[1]
    za, zb = z_mid - BASE_HALF, z_mid + BASE_HALF
    y0, y1 = y_top - t, y_top
    plate = _rounded(x0, x1, za, zb, y0, y1, BASE_CORNER_R)
    xd = float(lip[0]) + DECANTER_X
    c0, c1 = y0 - 1.0, y1 + 1.0
    m = NOTCH_MOUTH_R
    w0, w1, w_half, w_r = BOARD_WINDOW
    cuts = [_box(x0 - 1.0, xd, c0, c1, -NOTCH_R, NOTCH_R), _rod_y(xd, 0.0, NOTCH_R, c0, c1),
            _rounded(float(lip[0]) + w0, float(lip[0]) + w1, -w_half, w_half, c0, c1, w_r)]
    cuts += [_rod_y(xd + NOTCH_BAY[0], s * NOTCH_R, NOTCH_BAY[1], c0, c1) for s in (-1, 1)]
    cuts += _bay_fillets(xd + NOTCH_BAY[0], NOTCH_R, NOTCH_R, NOTCH_BAY[1], BAY_POINT_R,
                         c0, c1)
    cuts += [difference(_box(x0 - 1.0, x0 + m, c0, c1, *sorted((s * (NOTCH_R - 1.0),
                                                               s * (NOTCH_R + m)))),
                        _rod_y(x0 + m, s * (NOTCH_R + m), m, c0 - 1.0, c1 + 1.0))
             for s in (-1, 1)]
    cuts += [_square_hole(x, z, CARRIAGE_HOLE, c0, c1) for x, z in holes]
    feet = [(x, z) for x in (x0 + FOOT_INSET, x1 - FOOT_INSET)
            for z in (za + FOOT_INSET, zb - FOOT_INSET)]
    cuts += [_rod_y(x, z, FOOT_STEM / 2 + 0.25, c0, c1) for x, z in feet]
    return difference(plate, union(cuts)), feet


def _bay_fillets(xb: float, zc: float, edge: float, rb: float, r: float, y0: float, y1: float
                 ) -> list[Shape3D]:
    """Where a bay's circle (radius ``rb``, centred at ``x = xb``, ``z = ±zc``) meets the
    straight edge ``z = ±edge`` either side of the notch, at both ends of each bay: the four
    regions (world, through ``y0``..``y1``) between the edge, the circle and a fillet of
    radius ``r`` tangent to both beyond the edge and outside the circle. The board's points
    cut by them are rounded; the locator's inside corners filled by them are."""
    out = []
    ze = edge + r                               # the fillet's centre: r beyond the edge
    rc = rb + r                                 # and r outside the circle
    dx = math.sqrt(rc * rc - (ze - zc) ** 2)
    xt, zt = dx * rb / rc, zc + (ze - zc) * rb / rc     # its tangent point on the circle
    for s in (-1, 1):
        for e in (-1, 1):
            xa, xc = sorted((xb + e * (xt - 0.5), xb + e * dx))
            za, zb = sorted((s * (edge - 1.0), s * zt))
            region = _box(xa, xc, y0, y1, za, zb)
            out.append(difference(region, _rod_y(xb + e * dx, s * ze, r, y0 - 1.0, y1 + 1.0)))
    return out


@_part_memo
def _riser_locator(lip, y_table: float) -> list[Shape3D]:
    """The riser locator (world): :data:`LOCATOR`'s layers of the board's birch, glued, on the
    table in the board's notch (its outline the notch's, :data:`LOCATOR`'s play in, from its
    front edge to the round end; an ear into each of the notch's bays, its inside corners
    rounded :data:`EAR_FILLET_R`), a square pocket for
    the printed riser's base (:data:`RISER_BASE`) under the mouth. It puts the riser under the
    mouth (a lab jack had floated 76 mm either way in the bare notch: round 1's stand review)
    and lifts out to set a vessel on the table."""
    x0, play, n = LOCATOR
    x0 = float(lip[0]) + x0
    xd = float(lip[0]) + DECANTER_X
    r = NOTCH_R - play
    out = []
    for k in range(n):
        y0 = y_table + k * BASE_T
        y1 = y0 + BASE_T
        body = union([_box(x0, xd, y0, y1, -r, r),
                      cast(Shape3D, _rod_y(xd, 0.0, r, y0, y1)
                           & _box(xd, xd + r + 1.0, y0 - 1.0, y1 + 1.0, -r - 1.0, r + 1.0)),
                      *(_rod_y(xd + NOTCH_BAY[0], s * NOTCH_R, NOTCH_BAY[1] - play, y0, y1)
                        for s in (-1, 1)),
                      *_bay_fillets(xd + NOTCH_BAY[0], NOTCH_R, r, NOTCH_BAY[1] - play,
                                    EAR_FILLET_R, y0, y1)])
        out.append(difference(body, _square_hole(xd, 0.0, RISER_BASE + 2 * RISER_PLAY,
                                                 y0 - 1.0, y1 + 1.0)))
    return out


def _frustum(x0: float, x1: float, z0: float, z1: float, y0: float, h: float, inset: float
             ) -> Shape3D:
    """A slab from ``x0``-``x1`` by ``z0``-``z1`` (world) standing on ``y0``, ``h`` tall, its
    sides drawn in by ``inset`` over its height (45° for an equal one): a printed riser's boss
    and the recess it fits, both printable with no support."""
    face = Face(Wire.make_polygon([(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],
                                  close=True))
    return Solid.extrude_taper(face, (0.0, h, 0.0), math.degrees(math.atan2(inset, h)))


RISER_BOSS_IN = 3.5     # a riser piece's boss's base, in from its edges (its top 2 mm more)


def riser_pieces(x: float, z: float, y_table: float) -> list[tuple[str, Shape3D]]:
    """The printed riser stacked on the table at ``(x, z)`` (world), bottom up: its two lifts
    side by side across z, its two blocks on them, then its plates (:data:`RISER_LIFT`,
    :data:`RISER_BLOCK`, :data:`RISER_PLATES`), each piece's 45° boss (:data:`RISER_LIP`) up
    into a recess (or a cup) over it (0.5 mm of play each side), so the stack can't slide
    apart, and the whole of it :data:`DECANTER_RISER` tall. Each lift and block and a plate
    as tall as three walls are cups open underneath (:data:`RISER_WALL`), the thinner plates
    solid; each prints upside down, its boss on the bed. (``(name, part)`` each:
    ``decanter_riser_lift0``, ``1``, ``decanter_riser_block0``, ``1``, then
    ``decanter_riser_plate5``, ``4``.)"""
    half = RISER_BASE / 2.0
    w, hb = RISER_BLOCK
    lip, wall, b = RISER_LIP, RISER_WALL, RISER_BOSS_IN
    out: list[tuple[str, Shape3D]] = []

    def piece(xa, xb, za, zb, y0, h, cup):
        body = _rounded(xa, xb, za, zb, y0, y0 + h, 3.0)
        boss = _frustum(xa + b, xb - b, za + b, zb - b, y0 + h - 0.01, lip + 0.01, lip)
        part = union([body, boss])
        if cup:     # open underneath: its walls and top; the boss below rides inside them
            return difference(part, _box(xa + wall, xb - wall, y0 - 1.0, y0 + h - wall,
                                         za + wall, zb - wall))
        recess = _frustum(xa + b - 0.5, xb - b + 0.5, za + b - 0.5, zb - b + 0.5, y0 - 0.01,
                          lip + 0.41, lip)
        return difference(part, recess)

    from build123d import Cone

    bore, mouth, deep = RISER_RELIEF
    halves = ((z - half, z - half + w), (z + half - w, z + half))
    for i, (za, zb) in enumerate(halves):       # the lifts, under the blocks
        out.append((f"decanter_riser_lift{i}", piece(x - half, x + half, za, zb, y_table,
                                                     RISER_LIFT, True)))
    y0 = y_table + RISER_LIFT
    for i, (za, zb) in enumerate(halves):
        block = piece(x - half, x + half, za, zb, y0, hb, True)
        top = y0 + hb + lip                     # a relief in its boss over a screw head (it
        zc = (za + zb) / 2                      # lies under a post): a cone into a bore
        cone = Cone(bore / 2, mouth / 2, deep, rotation=(-90, 0, 0)).moved(
            Location((x, top - deep / 2, zc)))
        relief = union([cone, _rod_y(x, zc, mouth / 2, top - 0.01, top + 1.0),
                        _rod_y(x, zc, bore / 2, y0 + hb - wall - 1.0, top - deep + 0.01)])
        out.append((f"decanter_riser_block{i}", difference(block, relief)))
    y = y0 + hb
    for h in RISER_PLATES:
        out.append((f"decanter_riser_plate{h:g}", piece(x - half, x + half, z - half,
                                                         z + half, y, h, h >= 3 * wall)))
        y += h
    return out


@_part_memo
def _cleats(xs: list[float], y_top: float, face: float, sign: int, t_frame: float
            ) -> list[Shape3D]:
    """A rail's birch cleats (world), glued to the base board either side of the outer
    plate's rail, between its brackets (``xs``: their x, along the rail), :data:`CLEAT`'s
    play off its faces, their corners rounded (:data:`CLEAT_END_R`): they hold the side
    upright on the board (the brackets alone had let it sway several mm at a 20 N push:
    round 1's stand review) and drop it into place when it goes back on."""
    width, play, gap = CLEAT
    inner = face - sign * t_frame
    out = []
    xs = sorted(xs)
    for xa, xb in itertools.pairwise(xs):
        a, b = xa + BRACKET[0] / 2 + gap, xb - BRACKET[0] / 2 - gap
        n = math.ceil((b - a) / CARVERA_LONGEST - 1e-9)       # its pieces (the Carvera's bed)
        each = (b - a - (n - 1) * CLEAT_JOINT) / n
        for z0, z1 in ((face + sign * play, face + sign * (play + width)),
                       (inner - sign * play, inner - sign * (play + width))):
            za, zb = sorted((z0, z1))
            whole = _rounded(a, b, za, zb, y_top, y_top + BASE_T, CLEAT_END_R)
            if n == 1:
                out.append(whole)
                continue
            for i in range(n):          # the whole cleat's outline, cut square between
                x0 = a + i * (each + CLEAT_JOINT)
                out.append(intersection(whole, _box(x0, x0 + each, y_top - 1.0,
                                                    y_top + BASE_T + 1.0, za - 1.0, zb + 1.0)))
    return out


FOOT_STEM = 10.0            # a TPU foot's stem through the base board's corner hole


@_part_memo
def _tpu_foot(x: float, z: float, y_top: float) -> Shape3D:
    """A printed TPU foot (world): a pad under the base board's corner, a stem up through
    its hole and a bulb over it that snaps it in: a cone (a 30° lead-in) on a ledge, the
    stem bored up the middle so it squeezes through the hole."""
    from build123d import Cone

    up = (-90, 0, 0)                         # a solid's axis along +y
    t = BASE_T
    pad = Cylinder(FOOT_PAD_R, FOOT_PAD_H, rotation=up).moved(
        Location((x, y_top - t - FOOT_PAD_H / 2, z)))
    stem = Cylinder(FOOT_STEM / 2, t, rotation=up).moved(Location((x, y_top - t / 2, z)))
    lead = 1.25 / math.tan(math.radians(30.0)) + 0.6        # the cone's height
    bulb = Cone(FOOT_STEM / 2 + 1.25, FOOT_STEM / 2 - 0.6, lead, rotation=up).moved(
        Location((x, y_top + lead / 2, z)))
    bore = Cylinder(FOOT_BORE / 2, t + lead + 2.0, rotation=up).moved(
        Location((x, y_top + (lead - t) / 2 + 1.0, z)))
    return difference(union([pad, stem, bulb]), bore)


FOOT_BORE = 4.0             # up a foot's stem and bulb: the TPU squeezes into the hole


# --- the assembly ----------------------------------------------------------------------

STUDIES: dict[str, str] = {}
"""The cradles that are studies (their outputs say so): none since B was refined to build
(2026-10-10, ``docs/agentlib/DECANTER.md``)."""

DRIVE_PARTS = ("gearbox", "riser_", "crank_", "crankpin", "handle")
"""The hand crank's parts by name (:mod:`construction.handcrank`): the drive side's only."""
DRIVE_WHERE = "the drive"       # the drive's BOM lines' ``where`` (handcrank's extras)

GROUPS = {"drive": ("gearbox", "riser_", "handle"), "crank": ("conn",),
          "box": ("tie_", "front_saddle", "rear_saddle", "led", "base_stop"),
          "stand": ("base_plate", "base_cleat", "riser_locator", "foot_", "decanter_riser")}
"""A cradle's parts its sides' plan doesn't name, by name: the gearbox and the hand crank,
the arm's plates, the cradle box, the stand's base (the records' groups:
``api.building.group_of``)."""


def group_of(name: str) -> str | None:
    """The :data:`GROUPS` group of a cradle part's (side-less) name, or None."""
    return next((g for g, stems in GROUPS.items() if name.startswith(stems)), None)


def passive_side(side: Mechanism, design) -> Mechanism:
    """The cradle's other side: the drive side's own parts (:func:`assemble_cradle` mirrors
    them) less the hand crank's, the rod and the rod's pin, and the connections to them.
    Its plates keep the drive's holes, so each plate pair is one part, flipped: two of a
    file at SendCutSend, and spares that fit either side."""
    from spiderpig.construction.axle import AxleGroup
    from spiderpig.construction.handcrank import HandCrank

    rider = HandCrank.rider(design.ctx)
    crank = set(design.plan.topo.crank_bodies)
    pins = tuple(g.name.replace(":", "_") + "_" for g in design.groups
                 if isinstance(g, AxleGroup) and rider in g.axis.members)

    arm = tuple(f"{c}_plate" for c in crank)     # (the arm's other plates)

    def drive(name: str) -> bool:
        return (name in crank or name == rider or name.startswith(DRIVE_PARTS + arm)
                or (bool(pins) and name.startswith(pins)))

    keep = [b for b in side.bodies if not drive(b.name)]
    names = {b.name for b in keep}
    # (the drive's bought-by-the-pack lines too: its threadlocker, the riser's shims had
    # been counted twice)
    extras = [x for x in side.bom_extras if not x.where.startswith(DRIVE_WHERE)]
    return replace(side, bodies=keep, bom_extras=extras, connections=[
        c for c in side.connections if c[0][1] in names and c[1][1] in names])


KEEPER = (7.38, 0.36)
"""The keepers of the cradle link's quick-release pins (B's X and Y, both sides; the pack's
turn 1, 2026-10-14): the printed ring each clip pin's plan puts in the gap between its
lowest link (b4, b5) and the cradle link, printed in TPU 95A with its bore (mm) under the
pin, so it grips the pin (:func:`spiderpig.cradle_report.keepers`); and the least
interference a print may come out with (mm, on the diameter: the 5/16 in hard-anodized 6061
stick as calipered, within 0.038 mm, the keeper printed to it, ``rod_pin.MATCH_TOL``, since
simplification round 1, 2026-10-21, and the bore printed to within 0.15 mm; it was 7.4 and 0.4
on McMaster's 8 mm clevis pin, 7.95-7.99: the rod's wider tolerance keeps the tight end's push
on under 120 N). A side lying down with its pins' clips
out holds them by their keepers: each pin's head is under its link with nothing under it,
and a bare pin drops out. So the cradle box lifts off the drive side as it lies
(:func:`module_steps`), and the passive panel lifts off with its pins and their keepers on
them: no spacer is left loose on the cradle link. The bore was 7.6 mm (0.2 mm at least)
until design review round 2 (2026-10-14): the reassembly can rest half the cradle box on a
pin's tip, so each keeper is pushed with :data:`KEEPER_PUSH_N` on the bench, and the steps
hold a piece's weight until both its pins are in."""
KEEPER_PUSH_N = 30.0
"""The bench check of each keeper (the guide's): a 30 N thumb push on its pin's tip (3 kg on
a kitchen scale, or the luggage scale's hook), the pin mustn't slide; one that does is
reprinted 0.2 mm tighter. 30 N is 1.7 x the most the reassembly can put on a pin's tip
(half the 3.6 kg cradle box, ``cradle_report.keepers``; 3.2 kg before round 2's 12 mm tie
rods)."""
KEEPER_COLOR = LINER_COLOR


def keeper_text(pins) -> str:
    """The guide's sentence for the keepers of ``pins`` (each side's stack, over the links
    under the cradle link): on, and the bench check (:data:`KEEPER_PUSH_N`)."""
    many = len(pins) > 1
    kg = KEEPER_PUSH_N / 9.81
    each = "each" if many else "the"
    return (f"The TPU keeper of pin{'s' if many else ''} {' and '.join(pins)} (the ring "
            f"printed in TPU, its bore {KEEPER[0]:g} mm) pushed down over {each} pin onto "
            "its link, with a short tube over the pin (a firm push, about 100 N; "
            "deburr the pin's cross-hole first, it goes past it): it grips the pin, and it is "
            "what holds the pin in its link when the side lies down with its clip out (the "
            "trip's breakdown). Check each: hold its link, the pin's tip down on a kitchen "
            f"scale, and press until the scale reads {kg:.0f} kg ({KEEPER_PUSH_N:g} N, about "
            "twice what a piece of the cradle resting on a pin's tip could put on it): the pin "
            "mustn't slide in its keeper. One that slides is reprinted 0.2 mm tighter; one "
            "that won't go on by hand, 0.1 mm looser.")


KEEPER_FILL = 9.0
"""The keeper's bore is filled out to this diameter (mm) from :data:`KEEPER`'s: over any
ring's own bore (the pin's 8 mm and its print clearance), under its outline."""


def keeper_names(side: Mechanism, lk) -> list[str]:
    """The drive side ``side``'s printed rings (bare names) that are keepers (:data:`KEEPER`):
    on each pin of the cradle link's with a clip (a quick-release clevis pin), every ring or
    sleeve the pin's plan prints between its links (a gap's ring, a layer's ring, or one
    sleeve of both: not the take-up spacer under its head); none on a cradle without such
    pins (A). A clipped pin with no ring is an error: nothing would hold it, its side down."""
    import re

    link = cradle_link(lk)
    names = {b.name for b in side.bodies}
    pins = sorted(j for j in lk.links[link][0]
                  if any(j in js for b, (js, _) in lk.links.items() if b != link)
                  and f"pin_{j}_clip" in names)
    out = []
    for j in pins:
        rings = sorted(n for n in names
                       if re.fullmatch(rf"pin_{re.escape(j)}_(?:gap\d+_spacer|ring\d+)", n))
        if not rings:
            raise ConstructionError(f"pin {j}: a quick-release pin with no ring between its "
                                    "links to keep it (construction.cradle.KEEPER)")
        out += rings
    return out


def keeper_pin(name: str) -> str:
    """The pin a keeper (:func:`keeper_names`, bare or a side's) keeps: ``L.pin_X_gap14_spacer``
    -> ``L.pin_X_pin``."""
    import re

    return re.sub(r"_(?:gap\d+_spacer|ring\d+)$", "", name) + "_pin"


def _keeper(part):
    """A ring ``part`` on a pin (a side's frame, along z) as a keeper: its outline as it is,
    its bore filled in to :data:`KEEPER`'s."""
    from spiderpig.shapes import ring

    bb = part.bounding_box()
    xy = ((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2)
    return union([part, ring(xy, KEEPER_FILL, KEEPER[0], bb.min.Z, bb.max.Z)])


def _moved(part, dz: float, mirror: bool):
    if part is None:
        return None
    if mirror:
        part = part.mirror(Plane.XY)
        dz = -dz
    return moved(part, Location((0.0, 0.0, dz)))


@dataclass
class CradleFrame:
    """Where the cradle is at one crank angle: the mouth (world XY), the cradle's tilt
    (rad), the mid-plane (side z), the tie points in the cradle's frame."""

    lip: np.ndarray
    tilt: float
    z_mid: float
    ties: dict[str, tuple[float, float]] = field(default_factory=dict)


def cradle_frame(design, side: Mechanism) -> CradleFrame:
    lk = design.config.lk
    build = Build(design.ctx, design.plan, side)
    topo = design.plan.topo
    o, a = lk.output.frame
    lip = mouth_xy(build)
    d = build.xy(_point_of(topo, lk, a)) - build.xy(_point_of(topo, lk, o))
    tilt = math.atan2(d[1], d[0])
    from spiderpig.linkages.decanters import TIES

    return CradleFrame(lip=lip, tilt=tilt, z_mid=design.plan.z(design.plan.top)[1] + BAY / 2,
                       ties=dict(TIES))


def assemble_cradle(side: Mechanism, design) -> Mechanism:
    """The whole cradle from its drive side ``side`` at one crank angle: that side, its
    mirror image less the drive (:func:`passive_side`), the cradle box, the stand's base
    plate (the module's docstring)."""
    lk = design.config.lk
    cf = cradle_frame(design, side)
    z_mid = cf.z_mid
    passive = passive_side(side, design)
    keepers = set(keeper_names(side, lk))
    bodies: list[Body] = []
    connections = []
    for s, mech in (("L", side), ("R", passive)):
        mirror = s == "R"
        bodies.extend(Body(
            name=prefixed(b.name, s),
            part=_moved(_keeper(b.part) if b.name in keepers else b.part, -z_mid, mirror),
            joints=b.joints, color=KEEPER_COLOR if b.name in keepers else b.color,
            pose=b.pose, outline=b.outline,
            rigid_with=prefixed(b.rigid_with, s), fab=b.fab, bom_key=b.bom_key, sheet=b.sheet,
        ) for b in mech.bodies)
        connections += [((i, prefixed(pb, s), pj), (k, prefixed(cb, s), cj))
                        for (i, pb, pj), (k, cb, cj) in mech.connections]
    plan = design.plan
    topo = plan.topo
    link = next(n for n in topo.links if body_class(n) == cradle_link(lk))
    host = prefixed(link, "L")
    frame_host = prefixed(topo.frame_bodies[0], "L")
    t_link = design.ctx.sheet_t("link", link)
    link_bottom = plan.z(plan.layers[link])[0]
    half = z_mid - (link_bottom + t_link)           # a tie rod's half-length
    build = Build(design.ctx, plan, side)
    ground = _ground_xy(build)
    extras: list[Body] = []
    fastened = []
    tie_len = round(2 * half, 1)
    for n, p in tie_points(topo, lk).items():
        xy = tuple(build.xy(p))
        extras.append(hardware(f"tie_{n}", _tie(xy, -half, half), host, fab="purchased",
                               bom_key=_rod(tie_len), color=TIE_COLOR))
        for s, sign in (("L", -1), ("R", 1)):
            face = sign * (z_mid - link_bottom)      # the link's outer face
            extras.append(hardware(f"tie_{n}_screw{s}", _screw(xy, face, sign), host,
                                   fab="purchased", bom_key=TIE_M.key(TIE_ROD_SCREW),
                                   color=TIE_COLOR))
    stand = stand_of(lk)
    z0 = plan.z(0)[1]
    foot_half = z_mid - z0                            # a base tie between the outer plates
    foot_len = round(2 * foot_half, 1)
    for n in stand.base_ties:
        xy = tuple(_node_xy(ground, lk, topo, stand, n))
        extras.append(hardware(f"base_tie_{n}", _tie(xy, -foot_half, foot_half, SMALL_TIE_D, 10.0),
                               frame_host,
                               fab="purchased", bom_key=_standoff(foot_len), color=TIE_COLOR))
        for s, sign in (("L", -1), ("R", 1)):
            face = sign * (z_mid - plan.z(0)[0])
            extras.append(hardware(f"base_tie_{n}_screw{s}",
                                   _screw(xy, face, sign, TIE_SCREW, M5), frame_host,
                                   fab="purchased", bom_key=M5.key(TIE_SCREW),
                                   color=TIE_COLOR))
    bay_half = BAY / 2                  # a bay tie between the inner plates' bay faces
    bay_len = round(2 * bay_half, 1)
    for n in stand.bay_ties:
        xy = tuple(_node_xy(ground, lk, topo, stand, n))
        extras.append(hardware(f"bay_tie_{n}", _tie(xy, -bay_half, bay_half, SMALL_TIE_D, 10.0),
                               frame_host,
                               fab="purchased", bom_key=_standoff(bay_len), color=TIE_COLOR))
        for s, sign in (("L", -1), ("R", 1)):
            face = sign * (z_mid - plan.z(plan.top)[0])     # the inner plate's leg face
            extras.append(hardware(f"bay_tie_{n}_screw{s}",
                                   _screw(xy, face, sign, TIE_SCREW, M5), frame_host,
                                   fab="purchased", bom_key=M5.key(TIE_SCREW),
                                   color=TIE_COLOR))
    # the saddles (each a carrier, its V block at the drawn bottle's setting, its
    # thumbscrew), the lip gauge, the light and the base stop: drawn in the cradle's frame
    front, rear = SADDLES
    shown = SHOWN                       # the bottle drawn: the V blocks at their lowest
    at = setting(shown)
    light, clip, led_screw, clip_screws, clip_pilots = _led(front)
    xp, zp = REAR_POST

    def put(name: str, part, fab: str, color: str, key: str | None = None,
            dx: float = 0.0, dy: float = 0.0) -> None:
        if dx or dy:
            part = moved(part, Location((dx, dy, 0.0)))
        extras.append(hardware(name, _place(part, cf.lip, cf.tilt), host, fab=fab,
                               bom_key=key, color=color))

    lead = _lead_screw()
    rear_vee, rear_liner, rear_nut = _rear_vee(rear, cf.ties)
    for s, (carrier, sets), (vee, liner, nut), (jack, knob), du, keys in (
            (front, _front_saddle(front, cf.ties), _front_vee(front),
             _jack(FRONT_JACK, 6.0, FRONT_SCREW, (M6_HEX[0], M6_HEX[2]), FRONT_HEAD_Y,
                   KNOB_R[0]), at.front, (FRONT_NUT_KEY, FRONT_JACK_KEY)),
            (rear, _rear_saddle(rear, cf.ties), (rear_vee, rear_liner, rear_nut),
             (lead.screw, lead.knob), at.rear, (None, None))):
        if s is front:                  # the light's clip's screws' pilots
            carrier = difference(carrier, *clip_pilots)
        put(s.name, carrier, "printed", SADDLE_COLOR)
        put(f"{s.name}_vee", vee, "printed", VEE_COLOR, dy=du)
        put(f"{s.name}_liner", liner, "printed", LINER_COLOR, dy=du)
        # (the rear's nut and screw printed since simplification round 1)
        put(f"{s.name}_nut", nut, "purchased" if keys[0] else "printed",
            TIE_COLOR if keys[0] else KNOB_COLOR, keys[0], dy=du)
        put(f"{s.name}_jack", jack, "purchased" if keys[1] else "printed",
            TIE_COLOR if keys[1] else KNOB_COLOR, keys[1])
        put(f"{s.name}_knob", knob, "printed", KNOB_COLOR)
        # a set screw up from its underside holds the carrier on each tie
        fastened.extend((s.name, f"tie_{n}") for n in s.ties)
        for n, set_screw in zip(s.ties, sets, strict=True):
            name = f"{s.name}_set_screw_{n}"
            put(name, set_screw, "purchased", TIE_COLOR,
                f"m3_set_screw_{CARRIER_SET_SCREW:g}")
            fastened += [(name, s.name), (name, f"tie_{n}")]
        fastened += [(f"{s.name}_vee", f"{s.name}_liner"), (f"{s.name}_nut", f"{s.name}_vee"),
                     (f"{s.name}_nut", f"{s.name}_jack"), (f"{s.name}_knob", f"{s.name}_jack"),
                     (f"{s.name}_jack", s.name), ("bottle", f"{s.name}_liner")]
    # the rear lead screw's thrust ring on its brake washer, its knob's hub, the set screw
    # in the hub
    set_key = f"m3_set_screw_{SET_SCREW:g}"
    put("rear_saddle_thrust", lead.thrust, "printed", KNOB_COLOR)
    put("rear_saddle_brake", lead.brake, "printed", LINER_COLOR)
    put("rear_saddle_hub", lead.hub, "printed", KNOB_COLOR)
    put("rear_saddle_hub_set_screw", lead.set_screw, "purchased", TIE_COLOR, set_key)
    fastened += [("rear_saddle_thrust", "rear_saddle_jack"),
                 ("rear_saddle_thrust", "rear_saddle"), ("rear_saddle_brake", "rear_saddle"),
                 ("rear_saddle_brake", "rear_saddle_jack"),
                 ("rear_saddle_brake", "rear_saddle_thrust"),
                 ("rear_saddle_hub", "rear_saddle_jack"), ("rear_saddle_hub", "rear_saddle_knob"),
                 ("rear_saddle_hub_set_screw", "rear_saddle_hub"),
                 ("rear_saddle_hub_set_screw", "rear_saddle_knob"),
                 ("rear_saddle_hub_set_screw", "rear_saddle_jack")]
    # the rear V block's post, screwed up into from under the carrier
    y_head = rear.y0 + 0.25
    put("rear_saddle_post", _post_y((xp, zp), POST[1], POST[1] + POST[0]), "purchased",
        TIE_COLOR, POST_KEY)
    put("rear_saddle_post_screw",
        union([_rod_y(xp, zp, POST_M.head_d / 2, y_head, y_head + POST_M.head_h),
               _rod_y(xp, zp, POST_M.shank_d / 2, y_head + POST_M.head_h,
                      y_head + POST_M.head_h + POST_SCREW)]),
        "purchased", TIE_COLOR, POST_M.key(POST_SCREW))
    fastened += [("rear_saddle_post", "rear_saddle"), ("rear_saddle_post", "rear_saddle_vee"),
                 ("rear_saddle_post_screw", "rear_saddle"),
                 ("rear_saddle_post_screw", "rear_saddle_post")]
    # the lip gauge, fixed on its post: its pad at the mouth
    gauge, pin, gauge_nut = _gauge()
    put("front_saddle_gauge", gauge, "printed", KNOB_COLOR)
    put("front_saddle_gauge_pin", pin, "purchased", TIE_COLOR, GAUGE_PIN_KEY)
    put("front_saddle_gauge_nut", gauge_nut, "purchased", TIE_COLOR, GAUGE_NUT_KEY)
    fastened += [("front_saddle_gauge_pin", "front_saddle_gauge"),
                 ("front_saddle_gauge_pin", "front_saddle"),
                 ("front_saddle_gauge_nut", "front_saddle_gauge_pin"),
                 ("front_saddle_gauge_nut", "front_saddle_gauge")]
    extras.append(hardware("led", _place(light, cf.lip, cf.tilt), host, fab="purchased",
                           bom_key="penlight_aaa", color=LED_COLOR))
    extras.append(hardware("led_set_screw", _place(led_screw, cf.lip, cf.tilt), host,
                           fab="purchased", bom_key=f"m3_set_screw_{SET_SCREW:g}",
                           color=TIE_COLOR))
    extras.append(hardware("led_clip", _place(clip, cf.lip, cf.tilt), host, fab="printed",
                           color=SADDLE_COLOR))
    for i, part in enumerate(clip_screws):
        extras.append(hardware(f"led_clip_screw{i}", _place(part, cf.lip, cf.tilt), host,
                               fab="purchased", bom_key=CLIP_SCREW, color=TIE_COLOR))
    # the base stop: its rails on the rear V block (their flanges carry the body behind
    # it), its gate set behind the drawn bottle
    rails, rail_liners, rail_screws = _rails(rear)
    gate, (bolt, lock_knob, washer0, washer1, lock_nut) = _gate(rear, at.stop)
    for i, (rail, rail_liner) in enumerate(zip(rails, rail_liners, strict=True)):
        put(f"base_stop_rail{i}", rail, "printed", SADDLE_COLOR, dy=at.rear)
        put(f"base_stop_rail{i}_liner", rail_liner, "printed", LINER_COLOR, dy=at.rear)
        fastened += [(f"base_stop_rail{i}", "rear_saddle_vee"),
                     (f"base_stop_rail{i}", f"base_stop_rail{i}_liner"),
                     ("bottle", f"base_stop_rail{i}_liner"),
                     ("base_stop_gate", f"base_stop_rail{i}")]
    for i, part in enumerate(rail_screws):
        put(f"base_stop_screw{i}", part, "purchased", TIE_COLOR, M5.key(RAIL_SCREW),
            dy=at.rear)
        fastened += [(f"base_stop_screw{i}", f"base_stop_rail{i // 2}"),
                     (f"base_stop_screw{i}", "rear_saddle_vee")]
    put("base_stop_gate", gate, "printed", SADDLE_COLOR, dx=0.0, dy=at.rear)
    for name, part, fab, color, key in (
            ("base_stop_lock", bolt, "purchased", TIE_COLOR, LOCK_KEYS[0]),
            ("base_stop_knob", lock_knob, "printed", KNOB_COLOR, None),
            ("base_stop_washer0", washer0, "purchased", TIE_COLOR, LOCK_KEYS[1]),
            ("base_stop_washer1", washer1, "purchased", TIE_COLOR, LOCK_KEYS[1]),
            ("base_stop_nut", lock_nut, "purchased", TIE_COLOR, LOCK_KEYS[2])):
        put(name, part, fab, color, key, dy=at.rear)
    fastened += [("base_stop_gate", "rear_saddle_vee"), ("base_stop_lock", "base_stop_gate"),
                 ("base_stop_lock", "base_stop_knob"), ("base_stop_lock", "base_stop_washer0"),
                 ("base_stop_lock", "base_stop_washer1"), ("base_stop_lock", "base_stop_nut"),
                 ("base_stop_lock", "base_stop_rail1"), ("base_stop_lock", "rear_saddle_vee"),
                 ("base_stop_washer0", "base_stop_gate"), ("base_stop_washer0", "base_stop_knob"),
                 ("base_stop_washer1", "base_stop_rail1"),
                 ("base_stop_washer1", "rear_saddle_vee"), ("base_stop_nut", "base_stop_rail1"),
                 ("base_stop_nut", "rear_saddle_vee"), ("base_stop_nut", "base_stop_washer1")]
    # the base board, its TPU feet, its cleats, the riser locator in its notch, and each foot's
    # bracket bolting an outer plate onto it
    y_top = float(cf.lip[1]) + BASE_TOP
    t_frame = plan.t(0)
    base_holes = []
    cleats = []
    for s, sign in (("L", -1), ("R", 1)):
        face = sign * (z_mid - plan.z(0)[0])          # the outer plate's outer face
        host_s = prefixed(topo.frame_bodies[0], s)
        feet_xy = {n: tuple(float(v) for v in _node_xy(ground, lk, topo, stand, n))
                   for n in stand.feet}
        for n, xy in feet_xy.items():
            bracket, screws, hole = _bracket(xy, y_top, face, sign, t_frame,
                                             lk.key in BOARD_BRACKETS)
            base_holes.append(hole)
            extras.append(hardware(f"foot_bracket_{n}{s}", bracket, host_s, fab="printed",
                                   color=SADDLE_COLOR))
            for part, key, suffix in screws:
                extras.append(hardware(f"foot_bracket_{n}{s}_{suffix}", part, host_s,
                                       fab="purchased" if key else "printed", bom_key=key,
                                       color=TIE_COLOR if key else KNOB_COLOR))
        cleats += [hardware(f"base_cleat_{s}{i}", part, frame_host, fab="laser",
                            sheet=HOME_SHEET, color=BOARD_COLOR)
                   for i, part in enumerate(_cleats([xy[0] for xy in feet_xy.values()], y_top,
                                                    face, sign, t_frame))]
    plate_part, foot_xz = _base_plate(cf.lip, stand, y_top, base_holes, base_z(lk))
    extras.append(hardware("base_plate", plate_part, frame_host, fab="laser",
                           sheet=BASE_SHEET, color=BOARD_COLOR))
    extras += cleats
    extras += [hardware(f"riser_locator_{k}", part, frame_host, fab="laser", sheet=HOME_SHEET,
                        color=BOARD_COLOR)
               for k, part in enumerate(_riser_locator(cf.lip, y_top - BASE_T - FOOT_PAD_H))]
    glue = [BomLine(WOOD_GLUE, 1, "the base board's cleats and the riser locator's layers"),
            BomLine(GREASE, 1, "every greased face (food grade: the saddles' in the wine's "
                               "path)")]
    # each keeper grips its pin (its bore under the pin's: an interference fit, not a clash)
    fastened += [(prefixed(n, s), prefixed(keeper_pin(n), s))
                 for n in sorted(keepers) for s in SIDES_LR]
    # the crank block's keyed plates, cut under the shaft's key and filed to it at assembly
    fastened += [(prefixed(a, "L"), prefixed(b, "L"))
                 for a, b in (side.meta.get("hand_crank") or {}).get("filed", ())]
    for i, (x, z) in enumerate(foot_xz):
        extras.append(hardware(f"foot_boot_{i}", _tpu_foot(x, z, y_top), frame_host,
                               fab="printed", color=LINER_COLOR))
    y0, slope, x0 = seat(shown)
    extras.append(hardware("bottle", _place(_bottle(shown, y0, slope, x0), cf.lip, cf.tilt),
                           host, fab="display", color=BOTTLE_COLOR))
    vessel, risers = _decanter(cf.lip)
    extras.append(hardware("decanter", vessel, frame_host, fab="display",
                           color=DECANTER_COLOR))
    extras += [hardware(n, part, frame_host, fab="printed", color=RISER_COLOR)
               for n, part in risers]
    fastened += [("led_clip", SADDLES[0].name), ("led", "led_clip"),
                 ("led_set_screw", "led_clip"), ("led_set_screw", "led")]
    fastened += [(f"led_clip_screw{i}", n) for i in range(len(clip_screws))
                 for n in ("led_clip", SADDLES[0].name)]
    # the printed thread only in the print files (the assembly holds its envelope)
    print_shapes = {"rear_saddle_jack": _place(lead.threaded, cf.lip, cf.tilt),
                    "rear_saddle_nut": _place(moved(_rear_nut_threaded(),
                                                    Location((0.0, at.rear, 0.0))),
                                              cf.lip, cf.tilt)}
    meta = dict(side.meta, cradle=True, cradle_layer=plan.layers[cradle_link(lk)],
                print_poses=print_poses([b.name for b in extras + bodies], cf.tilt),
                print_shapes=print_shapes,
                cradle_lip=[float(cf.lip[0]), float(cf.lip[1])], cradle_tilt=float(cf.tilt),
                mid_plane=z_mid, bay_mm=BAY, lip_height_mm=LIP_HEIGHT,
                tie_mm=tie_len, base_tie_mm=foot_len, bay_tie_mm=bay_len, fastened=fastened,
                bottle=shown.name, cradle_linkage=lk.key, cradle_modules=lk.key in PACK_POSES,
                keepers=sorted(prefixed(n, s) for n in keepers for s in SIDES_LR),
                petg=sorted([n for n in PETG_PARTS if any(b.name == n for b in extras)]
                            + [b.name for b in extras if lk.key in BOARD_BRACKETS
                               and b.name.startswith("foot_bracket_")
                               and "_" not in b.name.removeprefix("foot_bracket_")]),
                board_window=[float(cf.lip[0]) + BOARD_WINDOW[0],
                              float(cf.lip[0]) + BOARD_WINDOW[1], -BOARD_WINDOW[2],
                              BOARD_WINDOW[2], BOARD_WINDOW[3]])
    for b in bodies + extras:
        # every cut file turned to its least-area box (simplification round 2's C1:
        # SendCutSend prices a part by its box; the walkers' files lie as they did)
        if b.fab == "laser":
            b.turn = "least"
    return Mechanism(name=f"{side.name}_cradle", bodies=bodies + extras,
                     connections=connections, meta=meta,
                     bom_extras=list(side.bom_extras) + list(passive.bom_extras) + glue)


@_part_memo
def _decanter(lip) -> tuple[Shape3D, list[tuple[str, Shape3D]]]:
    """A standard decanter under the mouth (display, :data:`DECANTER_PROFILE`) on the
    printed riser (:func:`riser_pieces`, :data:`DECANTER_RISER` tall) in the riser locator's
    pocket: (the decanter, the riser's pieces)."""
    table = float(lip[1]) - LIP_HEIGHT
    height = DECANTER_PROFILE[-1][0]
    base = table + DECANTER_RISER
    prof = [(0.0, base), *((r, base + h) for h, r in DECANTER_PROFILE), (0.0, base + height)]
    face = Face(Wire.make_polygon([(x, y, 0.0) for x, y in prof], close=True))
    xd = float(lip[0]) + DECANTER_X
    vessel = moved(Solid.revolve(face, 360.0, BAxis.Y), Location((xd, 0.0, 0.0)))
    return vessel, riser_pieces(xd, 0.0, table)


def assembly(view) -> list:
    """How the cradle box goes on (:mod:`construction.assembly`, ``CRADLE_ORDER``; ``view``
    a ``RobotView``): the tie rods with the drive side's cradle link (a bench sub-assembly
    of the drive side's stack), the bay tie with the drive side's inner plate; the base
    plate, its feet and the drive side's brackets; the saddles (each V block in its carrier
    on its thumbscrew, the lip gauge on the front one) slid onto the rods, their set
    screws, the light, the base stop; the passive side's cradle link onto the rods and its
    screws; the passive side's brackets and cleats (its knobs snug last); and the use:
    setting it for a bottle, the display bottle and decanter, and for a cradle that packs as
    modules (:data:`PACK_POSES`) taking it apart and putting it back (:func:`module_steps`)."""
    from spiderpig.construction.assembly import (
        CRADLE,
        FINISH,
        JOIN,
        STACK,
        UNIT,
        USE,
        Op,
        Scene,
        whole,
    )

    ops = []
    k = int(view.meta.get("cradle_layer", 0))
    ties = view.named(r"tie_[A-Za-z0-9]+")
    if ties:
        ops.append(Op(STACK, (k, 0, "ties"), whole(*ties, *view.named(r"tie_\w+_screwL")),
                      "Cradle link and tie rods", "", "cradle_link", sub=True, side="L"))
    key = str(view.meta.get("cradle_linkage") or "")
    modules = bool(view.meta.get("cradle_modules"))
    lying = (Scene("pack", "drive", show=("drive", "box", "passive", "blocks")) if modules
             else None)                 # (B's first build: lying down, off its board)
    right = view.named(r"tie_\w+_screwR")
    if right:
        ops.append(Op(UNIT, (99,), whole(*right), "",      # (after the link: its key)
                      f"The cradle link's M{TIE_M.d:g} steel button heads (class 10.9) from "
                      f"its outside into the rods' ends (threadlocker, {TIE_TIGHTEN_NM:g} N·m "
                      "on the key).",
                      "tie_screws", side="R", scene=lying))
    if modules and key in PACK_POSES:   # its passive panel onto the cradle link (its pins'
        #                                 clips: ``over_link``), as the trip's way back
        from spiderpig import linkage

        lk = linkage.get(key)
        text, lines = lower_first(lk)
        pins = trip_pins(lk)
        ops.append(Op(JOIN, (0,), (), "Passive side panel onto its cradle link", text,
                      "over_link", side="R", count=False, lines=lines,
                      scene=Scene("pack", "drive", show=("drive", "box", "passive", "blocks"),
                                  marks=(rf"R\.pin_{pins[0]}_pin",) if pins else ())))
    for s in SIDES_LR:
        bay = view.named(r"bay_tie_[A-Za-z0-9]+") if s == "L" else []
        screws = view.named(rf"bay_tie_\w+_screw{s}")
        if bay or screws:
            ops.append(Op(UNIT, (1,), whole(*bay, *screws), "",
                          "The bay tie onto the inner plate's bay face, an M5 button head "
                          "through the plate from its leg side (threadlocker)." if s == "L"
                          else "The inner plate onto the bay tie's free end too, its M5 "
                               "button head from the plate's leg side (threadlocker).",
                          "bay_tie", side=s))
    saddles = [n for s in SADDLES for n in (s.name, *(f"{s.name}_{p}" for p in (
        "vee", "liner", "nut", "jack", "knob"))) if n in view.z]
    saddles += [n for n in ("rear_saddle_post", "rear_saddle_post_screw", "rear_saddle_thrust",
                            "rear_saddle_brake", "rear_saddle_hub", "rear_saddle_hub_set_screw",
                            "front_saddle_gauge",
                            "front_saddle_gauge_pin", "front_saddle_gauge_nut") if n in view.z]
    sets = view.named(r"\w+_saddle_set_screw_\w+")
    if saddles:
        ops.append(Op(CRADLE, (0,), whole(*saddles, *sets), "Saddles",
                      "On the bench, each saddle's V block into its carrier, its TPU liner "
                      "pressed into its V, its pegs into the V's holes. The front one: its M6 "
                      "nut up into the hex pocket under its tab, a thin smear of the "
                      "food-grade grease on the block's sides and the carrier's walls (it "
                      "slides dry too, but smoother greased; wine runs through there), the "
                      "block down into the carrier's box, its tall cheek against the tall "
                      "wall, the M6 thumbscrew down through the tab's nut into its foot's bore "
                      "(a dab of grease on its thread: stainless in stainless galls dry), the "
                      "knob pressed onto its head. The rear one: the aluminium post into "
                      "the carrier's socket beside the lead screw's pocket, its milled flats "
                      "at the top, an M6 button head up into its tapped foot from below "
                      "(threadlocker); the printed nut up into the block's lead-screw boss, "
                      "its hex in the hex pocket (it floats there: loose is right); the TPU "
                      "brake washer and the printed thrust ring into the carrier's pocket; the "
                      "block's post boss down over the post; the printed lead screw down "
                      "through the tower's arm and turned down through the nut, its foot's "
                      "spigot into its bore and its shoulder down on the thrust ring. The "
                      "knob's hub up into the knob from below (its flange holds the knob on), "
                      "the hub onto the screw's top, its M3 set screw snug through the slot "
                      "in the knob's side onto the flat printed on the screw's top "
                      "(threadlocker: the set screw holds the screw to the knob by that "
                      "flat). Run the block up and down once, dry: the printed thread runs "
                      "dry, never greased, nor the thrust ring, its washer or the knob's "
                      "teeth. The knob turns the screw only lifted off its teeth; let go, it "
                      "drops back onto them (press it down until it seats) and the screw "
                      "can't turn: try it, the knob down, by pressing the block down hard (it "
                      "may not move). The lip gauge onto the front "
                      "carrier's post, a cheek either side, its bridge flat on the post's "
                      "top: the M5 tap bolt through them and the post from the front, its "
                      "nylon-insert nut behind, tight (it doesn't move: a bottle is uncorked "
                      "before it goes in). Then the saddles slid onto the tie rods' free "
                      "ends, the front one (the small V) on the rods nearer the hinge, the "
                      "knobs to the passive side, centred across the bay; the M3 set screws "
                      "up from their undersides into the printed pilots, snug on the rods "
                      "(they cut their own thread: no more than snug). Turn each knob down "
                      "until its V block sits on the rods.",
                      "saddles"))
    stop = [n for n in ("base_stop_rail0", "base_stop_rail1", "base_stop_rail0_liner",
                        "base_stop_rail1_liner", "base_stop_screw0", "base_stop_screw1",
                        "base_stop_screw2", "base_stop_screw3", "base_stop_gate",
                        "base_stop_lock", "base_stop_knob", "base_stop_washer0",
                        "base_stop_washer1", "base_stop_nut") if n in view.z]
    if stop:
        ops.append(Op(CRADLE, (2,), whole(*stop), "Base stop",
                      "The base stop: an M5 nut and a fender washer (the washer on top) into "
                      "the T-slot of the rail with the scale, from its front end; then each "
                      "rail's tongue pressed into its socket under the rear V block's flats "
                      "(the ribs along its top crush: a few taps of a mallet, until the rail "
                      "meets the block), the rail's top flush with the flats and its flange "
                      "with the block's V, two M5 button heads up through the socket's floor "
                      "into the tongue (snug: the tongue sits down on the floor); "
                      "each TPU liner pressed onto its rail's flange, its pegs into the holes. "
                      "The gate onto the rails from their end, its keys in the grooves, its "
                      "face toward the saddle; the nut slid along under the scale side's ear, "
                      "a fender washer on the ear, the M5 tap bolt down through it, the ear "
                      "and the washer below into the nut, its knob pressed onto its head. It "
                      "is set for each bottle (the use's setting step).", "base_stop"))
    light = [n for n in ("led_clip", "led_clip_screw0", "led_clip_screw1", "led",
                         "led_set_screw") if n in view.z]
    if light:
        ops.append(Op(CRADLE, (1,), whole(*light), "Light",
                      "The light's printed clip up under the front saddle's back end, its "
                      "flange on the saddle's underside, two M5 button heads up through it "
                      "into the saddle's pilots (snug: they cut their own thread). The "
                      "penlight down into the clip, onto its lip, its head up under where a "
                      "bottle's neck meets its shoulder; the M3 set screw into the clip's "
                      "boss, snug on the light's body (it holds the light while you twist "
                      "its head to switch it on).", "light"))
    base = [n for n in ("base_plate",) if n in view.z]
    feet = view.named(r"foot_boot_\w+")
    locator = view.named(r"riser_locator_\d+")
    cleats = [n for s in SIDES_LR for n in view.named(rf"base_cleat_{s}\d+")]
    brackets = {s: view.named(rf"foot_bracket_[A-Za-z0-9]+{s}(_\w+)?") for s in SIDES_LR}
    blocks = view.named(r"decanter_riser_(block|lift)\d+")
    plates = view.named(r"decanter_riser_plate\d+")
    # every carriage bolt goes in with the base board (from under it), the passive side's too;
    # the passive side's bracket (its screw and nut) goes on that side, its washer, nut and
    # knob onto its bolt once it stands on the board
    bolts = [n for n in brackets["R"] if n.endswith("_screw1")]
    on_bolt = [n for n in brackets["R"] if n.endswith(("_washer1", "_nut1", "_knob"))]
    on_board = key in BOARD_BRACKETS
    if on_board:        # every bracket bolted to the board; each plate's studs and knobs
        studs = ("_screw0", "_nut0", "_knob0", "_nut2")
        bolts = [n for s in SIDES_LR for n in brackets[s] if not n.endswith(studs)]
        on_bolt = []
        brackets = {s: [n for n in brackets[s] if n.endswith(studs)] for s in SIDES_LR}
    brackets["R"] = [n for n in brackets["R"] if n not in bolts and n not in on_bolt]
    if on_board and brackets["L"]:      # the drive side's studs on the bench first (round
        #                                     3's stand and use, M3): it drops into the slots
        ops.append(Op(FINISH, (0,), whole(*brackets["L"]), "Drive side's studs",
                      studs_first(key), "studs_L",
                      scene=Scene("built", "drive", show=("drive", "blocks"),
                                  marks=(r"foot_bracket_\w+L_(screw0|nut0)",))))
    if base or feet or brackets["L"]:
        text = board_text(key)
        ops.append(Op(FINISH, (0,), whole(*base, *feet, *locator, *bolts,
                                          *(() if on_board else brackets["L"])),
                      "Base board", text[0], "base", lines=text[1:]))
    if modules:
        if blocks:
            ops.append(Op(FINISH, (3,), whole(*blocks), "Laid on its drive side",
                          *_lay_first(key), scene=lying))
        if brackets["R"]:
            ops.append(Op(FINISH, (4,), whole(*brackets["R"]),
                          "Studs" if on_board else "Brackets",
                          STUDS if on_board else BRACKET_SIDE, "brackets_R",
                          scene=Scene("built", "passive", show=("passive",)),
                          lines=() if on_board else (
                              "On the bench, the panel on its outer plate: the washers and "
                              "knobs go onto the board's bolts once it stands there.",)))
        board = (*cleats, *on_bolt)
    else:
        if brackets["R"] or on_bolt:
            ops.append(Op(FINISH, (1,), whole(*brackets["R"], *on_bolt), "",
                          BRACKETS + " The passive side's knobs only started: they're "
                          "snugged last, at level, once the board is finished.", "base_R"))
        board = (*cleats, *(blocks if key in LAY_BLOCKS else ()))
    if board and key:
        text, tag, lines = _board_finish(key, modules)
        ops.append(Op(FINISH, (5,), whole(*board), "Cleats, finish, knobs", text, tag,
                      lines=lines))
    for i, u in enumerate(use_steps(key, bool(saddles)), 1):
        parts = {"setting": [n for n in ("bottle",) if n in view.z],
                 "riser": [*(() if key in LAY_BLOCKS else blocks), *plates,   # (A: no lay)
                           *(n for n in ("decanter",) if n in view.z)]}
        ops.append(Op(USE, (i,), whole(*parts.get(u.tag, ())), u.title, u.lines[0], u.tag,
                      count=False, lines=u.lines[1:], scene=cast(Scene | None, u.scene)))
    if key in PACK_POSES:
        from spiderpig import linkage

        lk = linkage.get(key)
        breakdown, reassembly = module_steps(lk, trip_pins(lk))
        intro = ("Four pieces, the riser and the hand crank, no tool: the base board, the two "
                 "side panels and the cradle box. ")
        for tag, trip in (("pack", breakdown), ("unpack", reassembly)):
            head = "Taking it apart" if tag == "pack" else "Back together"
            for i, m in enumerate(trip, 1):
                ops.append(Op(USE, (tag, i), (), f"{head} {i}/{len(trip)}: {m.title}",
                              (intro if tag == "pack" and i == 1 else "") + m.text,
                              f"{tag}_{i:02d}", count=False, lines=m.lines,
                              scene=cast(Scene | None, m.scene)))
    return ops + shop_ops(view)


ARRIVAL_LEAST_T: dict[str, dict[str, float]] = {"decanter_rcm": {"b1": 5.79}}
"""Per cradle, a link's least thickness on arrival (mm, calipered at its middle): B's rod b1,
0.250 in 5052, buckles at the clutch's cap at 1.72 on its nominal thickness, and Euler's load
goes as its cube, so 1.3 needs 0.756 of it cubed (SendCutSend's 0.250 in may come 0.027 in
thin: 1.22 there; simplification round 1's drive check). Thinner: ask SendCutSend for a
remake."""
POST_OVER = (12.85, 13.10, "33/64 in")
"""A pivot post calipered over the first (mm: ANSI H35.2's +-0.009 in on the 1/2 in rod, and
the laser's +-0.127 on its link's 13.05 mm bore: a post at its top won't go into a bore at
its bottom) has its link's bore opened with this drill (mm, its size; simplification round
1's stand check, C3)."""
FACE_SQUARE_DEG = 0.1
"""Each rod's ends faced square on the Carvera to this (degrees: 0.02 mm across a 12.7 mm
face, a machinist's square shows no light under it): an end face 0.25° out takes most of a
tie rod's end joint's gap moment (simplification round 1's box review)."""


def shop_ops(view) -> list:
    """The cradle's shop work before its first part goes on (``construction.assembly``'s
    ``SHOP`` stage; simplification round 1, 2026-10-21: the posts, tie rods, pins and the
    crankpin are 6061 rod made at home, and the cleats and the riser locator are cut on the
    Carvera): the arrival checks, the rods cut to length and faced, drilled and tapped, the
    pins cut to their calipered stacks, the Carvera's birch. No part goes on in them: each
    says what it makes, from the bodies' catalog keys (``view.keys``)."""
    import re

    from spiderpig import linkage
    from spiderpig.construction.assembly import SHOP, Op, Scene
    from spiderpig.construction.pivots import rod_pin as rp
    from spiderpig.hardware.order import PIN_TAKE_UP_MM

    key = str(view.meta.get("cradle_linkage") or "")
    if not key:
        return []
    lk = linkage.get(key)
    keys = {n: str(k or "") for n, k in view.keys.items()}
    crankpin = [n for n in keys if n.endswith(".crankpin")]
    hard = sorted(n for n, k in keys.items() if k.startswith("rod6061ha_12p7_m6_")
                  and n not in crankpin)
    plain = sorted(n for n, k in keys.items() if k.startswith("rod6061_"))
    pins = sorted(n for n, k in keys.items() if k in rp.ROD_KEY.values()
                  and re.fullmatch(r"[LR]\.pin_\w+_pin", n))
    clips = [n for n in pins if f"{n[:-4]}_clip" in view.z]
    ops: list = []
    # the arrival checks
    least = ARRIVAL_LEAST_T.get(key, {})
    lines = [f"{link}: caliper its thickness at its middle, away from its bosses: at least "
             f"{t:g} mm (its buckling at the clutch's cap goes as its thickness cubed; "
             "SendCutSend's 0.250 in may come 0.69 mm thin). Thinner: ask SendCutSend for a "
             "remake before anything is drilled to it." for link, t in least.items()]
    lines.append(rp.ARRIVAL_WORDS)
    ops.append(Op(SHOP, (0,), (), "Arrival checks", "Caliper the rods as they arrive (three "
                  "places along each, a micrometer or the caliper's jaws square on it): "
                  f"{rp.STOCK_WORDS}", "arrival", count=False, lines=tuple(lines)))
    # the rods to length, faced
    pivots = {j: [n for n, (js, _) in lk.links.items() if j in js] for j in lk.frame
              if j != lk.crank[0]}
    lo, drill_mm, drill = POST_OVER
    bores = "; ".join(f"{j}'s {' and '.join(ls)}" for j, ls in pivots.items())
    rods_text = (
        f"Saw the pivot posts ({len(hard)}: the pillars a link turns on, {', '.join(pivots)}) "
        "from the hard-anodised 1/2 in rod, each 1.5 mm over its length (the BOM's cut list), "
        "a fine-tooth hacksaw in the vise, the rod wrapped in tape where the jaws hold it. "
        f"The plain posts and tie rods ({len(plain)}) and the riser's sleeves come cut, 1.5 "
        "mm over (Speedy Metals, +1/8 -0 in). The sleeves are only drilled here: each is "
        "faced at the gearbox's step, to the gearbox's calipered height, once it has come.")
    rod_lines = (
        "Face every piece but the riser's sleeves square to its length on the Carvera: the "
        "pieces lying along X in "
        "the vise against one stop, a batch a setup, an end mill's flank across the end, then "
        "re-clamped once for the other end. Each to its length within 0.05 mm (a side's "
        "posts all one length: they set its plates' gap), each end square to "
        f"{FACE_SQUARE_DEG:g}° (a machinist's square on it shows no light under it; 0.02 mm "
        "across the face). Break the edges with a file.",
        f"Caliper each pivot post at its journal, where its link turns: over {lo:g} mm, open "
        f"that link's bore with a {drill} ({drill_mm:.2f} mm) drill, in the drill press, the "
        f"link clamped flat on scrap ({bores}). Its sawn and faced ends are bare (the hard "
        "coat is on its journal), and so are its taps. The crankpin likewise (sawn from the "
        f"same stick, the pins' step): over {lo:g} mm, open the rod b1's bore and the crank "
        "block's plates' holes for it with the same drill.",
        "The riser's studs: saw the heads off four M8 x 60 A2-70 hex screws (Fastenal's), "
        "each cut to 45 mm, its end filed and a nut run off it; drill the four sleeves 8.0 mm "
        "through, square, in the drill press.")
    ops.append(Op(SHOP, (1,), (), "Rods to length, faced", rods_text, "rods", count=False,
                  lines=rod_lines,
                  scene=Scene("built", show=("*",),
                              marks=(r"[LR]\.pillar_\w+", r"[LR]\.stand_column_\w+",
                                     r"tie_T[a-z]+\d"))))
    # drilled and tapped
    m6 = 2 * (len(hard) + len(plain)) + 2 * len(crankpin) + 1      # (+ the rear post's foot)
    m3 = 2 * (len(pins) - len(clips)) + len(clips)
    tap_lines = (
        f"M6 ({m6} ends: each post and tie rod both ends, the crankpin both ends, the rear "
        "saddle's post's foot): a 5.0 mm drill 14 mm deep, then the M6 x 1 plug tap and the "
        "bottoming tap 12 mm deep, by the T-handle wrench, Tap Magic for aluminium, backed "
        "off half a turn each turn. The drill press square to the face (the rod in a "
        "V-block), or a printed drilling guide over the end.",
        f"M3 ({m3} ends: each screw-ended pin both ends, each clip pin its head end): a 2.5 mm "
        "drill 8 mm deep, the M3 x 0.5 tap 6 mm; blow the chips out. Each clip pin's cross "
        "hole: 2 mm through, at the length the pins' step gives, in a V-block.",
        "The rear saddle's post: its two wrench flats milled on the Carvera, 13 mm across, "
        "10 mm long at its top.")
    ops.append(Op(SHOP, (3,), (), "Drilled and tapped", "Every rod's tapped ends, on the "
                  "bench drill and by hand, once each piece is to its length:", "taps",
                  count=False, lines=tap_lines))
    # the pins to their stacks
    if pins or crankpin:
        stock = rp.PIN_STOCK_WORDS
        ops.append(Op(SHOP, (2,), (), "Pins cut to their stacks",
                      f"Each link pin cut from {stock} to its own stack: caliper its links "
                      "and the printed spacers between them, as the stack steps put them on "
                      "(the arrival's figures), add them up, and cut the pin at that stack + "
                      f"{PIN_TAKE_UP_MM:g} mm (+0.1/-0: a hacksaw 1 mm over, then faced on "
                      "the Carvera in the vise, or filed to the caliper), its ends square; "
                      f"{len(pins)} pins" + (f", {len(clips)} of them clip pins" if clips
                                             else "") + ". The crankpin likewise, to its "
                      "drawn length (its plates' stack and its washer).", "pins",
                      count=False, lines=rp.PIN_LINES,
                      scene=Scene("built", show=("*",),
                                  marks=(r"[LR]\.pin_\w+_pin", r"L\.crankpin"))))
    # the birch on the Carvera
    homes = sorted(n for n in view.z if re.fullmatch(r"base_cleat_\w+|riser_locator_\d+", n))
    if homes:
        ops.append(Op(SHOP, (4,), (), "The cleats and the riser locator, on the Carvera",
                      CARVERA_TEXT[0], "carvera", count=False, lines=CARVERA_TEXT[1:],
                      scene=Scene("built", show=("board",),
                                  marks=(r"base_cleat_\w+", r"riser_locator_\d+"))))
    return ops


CARVERA_TEXT = (
    "From the 3/8 in Baltic birch panels (ORDER.md: Woodcraft): saw each 12 x 12 in panel to "
    "about 238 mm wide, square, to go under the Carvera's 240 mm (its bed is 360 x 240 mm).",
    "Workholding: the panel flat on the wasteboard, double-sided tape over its whole underside "
    "and pressed down with a roller or a block (or four screws through it outside the parts); "
    "zero the bit on the panel's top, its X and Y on the panel's sawn corner.",
    "Each panel's nest is ORDER.md's Carvera list's sheet (`*_sheet_Carvera_*.dxf`: the parts "
    "laid out on 300 x 235 mm, 3 mm in from its edges): the outlines with a 1/8 in (3.175 mm) "
    "single-flute end mill "
    "for wood, through in three or four passes, a tab or two on each part (tape alone can let "
    "a narrow cleat shift on the last pass); the files are the parts' nominal size, Makera CAM "
    "offsets the bit.",
    f"The lead-in: a 90° chamfer bit along each cleat's long top edges, {CLEAT_LEAD:g} mm (the "
    "rail finds its slot blind on the way back; either edge can go to the rail), before the "
    "parts are cut free.",
    "Cut the tabs and sand them flush, each cleat's long faces before all (a nub there takes "
    "up the rail's play). A cleat longer than the bed comes in pieces (B's 452 mm ones in "
    f"two): they go onto the board end to end on its pencil line, {CLEAT_JOINT:g} mm apart, "
    "each glued along its length.",
)
"""The Carvera's step (``carvera``: :func:`shop_ops`), the cleats and the riser locator cut at
home from Woodcraft's 3/8 in Baltic birch (simplification round 1, 2026-10-21, the user's
choice; SendCutSend's routed birch before, its tabs sanded)."""


SIDES_LR = ("L", "R")


@dataclass(frozen=True)
class UseStep:
    """One of the guide's use steps (:func:`use_steps`; the cradle page's "Using it"): its
    tag (its place in ``construction.assembly.USE_TAGS``), title, sentences (each its own
    item) and picture (``construction.assembly.Scene``)."""

    tag: str
    title: str
    lines: tuple[str, ...]
    scene: object = None      # a construction.assembly.Scene


def grip_at_slip_n(key: str) -> float:
    """The force at cradle ``key``'s hand crank's grip where its clutch slips as drawn: set at
    ``handcrank.CLUTCH_SET`` x a full 6 L's worst start (``handcrank.drawn_clutch``, over
    ``handcrank.handle_r``'s throw). The user sets their own on a measured start."""
    from spiderpig.construction.handcrank import drawn_clutch, handle_r

    return drawn_clutch(key)["set_nm"] * 1000.0 / handle_r(key)


def use_steps(key: str, saddles: bool = True) -> list[UseStep]:
    """How cradle ``key`` is used (design review round 2's M4, 2026-10-14: the guide's use
    steps had been four paragraphs, the use limits 626 words in one, in an alphabetical
    order that took it apart before it poured): where it stands, setting it for a bottle,
    the decanter's riser, decanting, and the use limits as a five-line card and the rest
    for reference. The guide's steps and the cradle page's "Using it" panel
    (``cradle_report.motion``'s ``use``) read the same lines."""
    from spiderpig import linkage
    from spiderpig.construction.assembly import Scene
    from spiderpig.construction.handcrank import (
        CLUTCH_SET,
        TOGGLE_TURNS,
        drawn_clutch,
        handle_r,
    )

    r = handle_r(key)
    slip = grip_at_slip_n(key)
    ceil = drawn_clutch(key)["hi_grip_n"]
    turns = f"{TOGGLE_TURNS:g}"
    room = f"{ROOM_FREE:g} mm free behind the base board and beside its drive side"
    out = [UseStep("where", "Before you use it", (
        "Headroom: 950 mm over the table (a 6 L's base at full tilt reaches about 910 mm).",
        f"{room}: the base stop's rails reach past its rear edge at full tilt, and the hand "
        "crank's grip turns past " + ("both edges" if key == "decanter_rcm" else
                                      "its drive-side edge") + " (a hand on it reaches about "
        f"{HAND_REACH.get(key, ROOM_FREE - 20.0):g} mm out): against a wall the grip's bolt "
        "would rub it all the way round.",
        "A flat counter: the TPU feet take up about 1 mm; if it rocks, print a thicker foot "
        "for the corner that lifts.",
        ("The four stud knobs snug by hand; wiggle each bracket: it mustn't move on the "
         "board (one that does: its nylon-insert nut snugged again, as the base board's last "
         "step says)." if key in BOARD_BRACKETS else
         "The thumb knobs snug by hand. One that keeps working loose has let its "
         "bracket's printed foot creep under the washer: snug it, and reprint that bracket in "
         "PETG."),
        "Nothing on the board round the riser: under the base stop the board has a window, so "
        "at rest the base stop comes down to 43 mm over the table there, not 26 over a cork "
        "left on the board.",
        "The clutch checked on its stub, and the gearbox's red arcs painted (the last build "
        "step, \"Before the first pour\"): at every set-up, and every month or ten pours "
        "while it stands set up."),
        Scene("rest", show=("*", "bottle", "decanter"),
              marks=(r"foot_bracket_\w+_knob", "base_plate")))]
    if saddles:
        out.append(UseStep("setting", "Setting it for a bottle", (
            "Measure the bottle first, every one: two bottles of a size differ (750 ml "
            "Bordeaux run 300 to 315 mm long, 3 L ones 470 to 475), so the listed sizes' "
            "grooves are only examples. Its length standing on the table, base to lip, to the "
            f"millimetre; its neck's diameter {NECK_AT:g} mm down from the lip, where the front "
            "V's back end holds it, in front of any shoulder (calipers, or a strip of paper "
            "round it: the length round over 3.14); its body's 255 mm from the lip, where it "
            "lies on the rear V's front edge.",
            "Uncork it there, standing on the table: the lip gauge is fixed over the mouth, so "
            "nothing goes in or out of it in the cradle. A sparkling wine under pressure the "
            "same, its cork eased out by hand, the mouth pointed away from people.",
            "Wipe the bottle dry (a cold one sweats) and the V blocks' liners (after a wash, or "
            "wine from the last pour): dry, their TPU holds the bottle on the gate the whole "
            "pour; wet, it slides forward onto the lip gauge's pad mid-pour, with a jolt.",
            "Then at rest (neck up), the bottle out, set all three.",
            "The base stop: loosen its knob and slide the gate until its face reads the "
            "bottle's length. The grooves across the rail or flat read a length every 5 mm (a "
            "long one every 10; three, four and five dots in front of the 300, 400 and 500 mm "
            "ones, one in front of the 50s).",
            "The pointer reaching forward from the gate's face reads the millimetres: set the "
            "face just behind the groove of the 5 mm at or under the length, then on until "
            "the notch in the pointer's inner edge counted from the face that is the extra "
            "millimetres (the first for 1 mm, the fourth for 4) lines up with a groove beside "
            "it, where both meet the rail's top (none, and the face on the groove, for 0).",
            "Turn its knob tight by hand, then tug the gate toward the saddle: it must not move "
            "(a gate left loose lets a bottle slide down the rails).",
            "The front V block: turn its knob until the block's top edge is on the bottle's "
            "neck diameter on the scale inside the carrier's wall (every 4 mm, behind the "
            "knob), or on a listed size's groove in front of it (one dot a 750 ml or a magnum, "
            "two a 3 L, three a 6 L).",
            "The rear V block: its knob turns only lifted off its teeth. Lift it and turn it "
            "until the screw boss is on the body's diameter on the tower's face (every 10 mm), "
            "or a listed size's groove (one dot a 750 ml to five a methuselah, whose V blocks "
            "sit on their rods; 9 turns end to end); let it drop and press it down until it "
            "seats: it holds the block where it is.",
            "Now and then, the rear knob down, try to turn the lead screw by its collar under "
            "the block and press the block down hard: neither may move (a set screw that has "
            "let go: snug it, threadlocked).",
            "Lay the bottle in base first, its label up: hold it by the neck and the body, its "
            "base low, set its heel on the rails (or the rear V) just in front of the gate and "
            "slide it back against the gate, then lower the neck into the front V, the rim "
            "passing behind the lip gauge's pad. If the neck won't go down, the bottle is "
            "longer than the gate is set: move the gate back. Then tug the gate again, and "
            "again before the first turn if the bottle has rested for its sediment (its "
            "lock's printed lips creep under the preload).",
            "Look along the neck: the pad's bottom edge covers the top of the rim, 2 to 4 mm "
            "in front of it (more, and the gate is too far back by the extra; the pad not over "
            "the rim, and the front V is set too low). Past level the pad holds the bottle "
            "forward, as the gate holds it back.",
            "And under the neck: a strip of paper between it and the front V's liner, either "
            "side, must be pinched. Slid in freely, the neck is up over its V: a body that "
            "narrows toward its base (some 3 and 6 L ones do) lies tipped back on it, its "
            "mouth up to 4 mm high; set the rear V for a body 5 mm wider (half a step on its "
            "scale, the block lower) and it comes down."),
            Scene("rest", show=("*", "bottle"),
                  marks=(r"front_saddle_knob", r"rear_saddle_knob", r"base_stop_(gate|knob)",
                         r"front_saddle_gauge"))))
    stack = f"{RISER_LIFT + RISER_BLOCK[1]:g} mm"
    plates = ", ".join(f"{p:g}" for p in RISER_PLATES)
    out.append(UseStep("riser", "The decanter's riser", (
        "The decanter stands on the printed riser in the riser locator's pocket, under the "
        "mouth: one height for each decanter, set at rest and left the whole pour.",
        f"The rule: its rim at least {RISER_CLEAR[0]:g} mm under the lowest part of the "
        "cradle box over it, the light's printed clip (394 mm over the table, near rest), and "
        f"{RISER_CLEAR[1]:g} mm clear of the box beside it over the whole swing.",
        f"The riser's pieces, bottom up: its two lifts side by side, its two blocks on them "
        f"({stack} together: the drive panel's lay-flat blocks on the trip, each on its "
        f"lift), then its {plates} mm plates, each piece's boss up into the one over it; the "
        f"whole of them {DECANTER_RISER:g} mm.",
        f"A RIEDEL Ultra decanter (225 mm tall) takes all of them, {DECANTER_RISER:g} mm: "
        f"its rim {DECANTER_RISER + DECANTER_PROFILE[-1][0]:g} mm over the table.",
        "Another decanter or glass: the cradle page (decant.html, Pour into) gives its stack "
        "by the same rule: some of the lifts, the blocks and the plates, in that order. "
        "Lower is always safe; higher, never.",
        "Never raise the decanter during a pour (a book, a coaster, anything): coming back "
        "to rest the worm drives the cradle box down onto the glass with a force no clutch "
        "bounds."),
        Scene("rest", show=("board", "risers", "decanter", "box"),
              marks=(r"decanter_riser_\w+", "decanter"))))
    out.append(UseStep("decant", "Decanting", (
        "Crank it to rest (neck up), set it for the bottle and lay the bottle in (the step "
        "before); the decanter on its riser.",
        "Switch the light on (twist its head).",
        "Turn the handle slowly, either way: the mouth stays where it is while the bottle "
        f"tips (rest to full tilt is {full_turns(linkage.get(key)):.1f} turns).",
        "Stop when the sediment, lit from under by the light, reaches the neck; let go and "
        "it holds. "
        "Letting go stops the bottle, not the stream: the wine already over the lip runs on; "
        "to stop it, turn back about a turn.",
        "A bottle bigger than its decanter (a 6 L fills a 1.5 L decanter four times): when "
        "the decanter is full (the cradle page says at how many turns), turn back a turn "
        "until the stream stops, slide the decanter off its riser and out under the raised "
        "cradle box, and set the next one on the same riser (a decanter whose stack is the "
        "same; another's, only with the cradle back at rest).",
        "Then crank back to rest with the decanter and its riser where they are: its rim "
        f"keeps its {RISER_CLEAR[0]:g} mm under the clip the whole way. At rest, take the "
        "decanter away and wipe the neck, the front saddle, the tie rods and the light (the "
        "front saddle's drains let out what runs into it)."),
        Scene("full", show=("*", "bottle", "decanter", "risers"),
              marks=("decanter", "bottle"))))
    zone = (f"while the gearbox hub's arrow is in a red arc (the {turns} turns either side of "
            "rest and of full tilt)")
    out.append(UseStep("limits", "The use limits", (
        f"1. Room: 950 mm of headroom and {ROOM_FREE:g} mm free behind the board and beside "
        "its drive side.",
        f"2. The decanter on its riser, set once at rest, its rim {RISER_CLEAR[0]:g} mm or "
        "more under the light's clip; once set, nothing raised or added under the cradle "
        "box.",
        f"3. Hands out, and nothing added under the box, {zone}; the decanter stays put.",
        "4. The bottle uncorked before it goes in, and the pad over its rim (look along the "
        "neck); the gate's knob tight (tug the gate) and the rear knob down on its teeth "
        "whenever a bottle is in.",
        "5. The handle off (its thumb screw) when you leave it: nothing turns without it.",
        "Where it pinches while the handle turns: both side stacks, their bay faces included "
        "(each return to rest the cradle links shear past the inner bars' front ends, at the "
        "bay's edge); the crank and the rod's end, turning in the open above and behind the "
        "gearbox, in front of the hand crank. A trapped finger: the worm holds what it has "
        "caught, so turn the handle back the way it came; never pull.",
        "Never force the handle: if it stiffens, stop and look. Its clutch clicks round past "
        f"its setting ({CLUTCH_SET:g} times the start you measured: about {slip:.0f} N at the "
        f"grip on its {r:g} mm crank for a full 6 L, and never over {ceil:g} N), so, set and "
        "checked, a hard push can't break the drive away from the ends of the swing; a click "
        "means something is in the way.",
        "Check the setting on its stub in the riser "
        "locator's pocket (the last build step, \"Before the first pour\") at every set-up, "
        "after greasing it, after a trip in a hot car, and every month or ten pours while it "
        "stands set up: unchecked, the grease's drift can carry the slip over what the parts "
        f"are rated at. Within {turns} turns of rest and of full tilt (the gearbox's red "
        "arcs) no clutch bounds the force: there only the clearance protects.",
        "Turn it, don't spin it: about a turn a second at most, empty or full (spun fast, "
        "the worm passes more of the clutch's slip to the cradle).",
        "Only the hand crank's arm may turn it: never a drill on its input once it is built, "
        "nor a hand twisting the hub or the clutch's flange (either turns the cradle with no "
        "clutch between).",
        "Uncork every bottle standing on the table before it goes in: the lip gauge is fixed "
        "over the mouth, and nothing goes in or out of it in the cradle. A sparkling wine "
        "under pressure the same, its cork eased out by hand, the mouth pointed away from "
        "people (the -50° rest takes an opened full bottle).",
        "The rear knob lifted, only the brake washer slows the lead screw: lift it only "
        "with the bottle out, and press it down on its teeth after.",
        "Keep your hands off the decanter and the bottle while the handle turns.",
        "Lift the cradle by its base board, never by the hand crank or the cradle box.",
        f"After the clutch has slipped against a block (a click under load), take every "
        f"post's stainless M6 button head back to {BUTTON_A2_70_NM['6']:g} N·m and every steel "
        "one (the tie rods'" + (", pillar " + " and ".join(STEEL_FEET[key]) + "'s foot"
                                if key in STEEL_FEET else "")
        + f") to {TIE_TIGHTEN_NM:g}: the slip may have set the plates under the posts' ends.",
        "Keep it out of the sun and under 50 °C (a hot car too), and hand-wash the printed "
        "parts: the PETG ones that carry the bottle bear up to 11 MPa as long as a bottle "
        "rests in it, and PETG's resin deflects at 64 °C under 1.8 MPa (Eastman's Eastar "
        "6763); PLA softens at about 55."),
        Scene("rest", show=("*", "bottle", "decanter"),
              marks=("L.gearbox", r"L\.handle\w*"))))
    return out


TOOLS = (
    "Turning: a torque wrench (4 to 15 N·m: the drive's M6 screws, the posts' and tie rods' "
    "screws, the gearbox's riser, its end screw), hex keys (2 to 6 mm, and one for the "
    "clutch plungers' backs), 8, 10 and two 13 mm wrenches and a 3/4 in one (the plungers' jam "
    "nuts)",
    "Measuring: a caliper with a depth rod (the links' thickness on arrival, b1's least, each "
    "rod stick and pivot post, the pins' stacks, the gearbox's go/no-go), a machinist's "
    "square (the rods' faced ends), a luggage scale (the clutch's start and setting, the "
    "empty grip, the board brackets' nuts on a wrench), a kitchen scale (the keepers)",
    "Cutting and tapping the rod: a fine-tooth hacksaw (the pivot posts, the pins and the "
    "crankpin from their hard-anodized sticks, the riser's studs from A2-70 screws), the "
    "Carvera with its vise and an aluminium end mill (every rod's ends faced square, the rear "
    "post's flats), M3 x 0.5 and M6 x 1 taps (plug and bottoming) in a T-handle tap wrench, "
    "Tap Magic for aluminium, drills 2.0 mm (the clip pins' cross holes), 2.5 and 5.0 mm (the "
    "taps), 8.0 mm (the riser's sleeves), 21/64 and 25/64 in (a pin stick over its size) and "
    "33/64 in (a pivot post over its size), a drill press with a V-block (or a printed "
    "drilling guide)",
    "The Carvera's birch: a saw to take each 12 in panel under the bed's 240 mm, double-sided "
    "tape, a 1/8 in single-flute end mill for wood and a 90° chamfer bit (the cleats' "
    "lead-ins)",
    "Shaping: a deburring tool with a countersink blade, a flat file and fine sandpaper on a "
    "flat block (the links that arrive as cut, the laser-cut edges, the router's tabs on the "
    "board, the Carvera's tabs, the keyed plates' slots), a 14 mm drill bit (the hand crank's "
    "hub, by hand), a drill (the gearbox's run-in, before it goes on the cradle, never after)",
    "A vise (the knobs' nuts, the hand crank's knobs onto their screws' heads, the clutch "
    "plungers' nuts)",
    "Clamps or weights for the glue; a short tube over a 5/16 in pin (the keepers)",
    "Marking and handling: paper, a pencil, a paint pen and red paint (the pack marks, the "
    "gearbox's toggle arcs, each stick's reading), masking tape, a folded towel",
    "Also: the food-grade grease (the BOM's: the saddles, the clutch, and wiped thin on every "
    "hard-anodized journal a link turns on), a clear polyurethane and a brush (the base "
    "board)",
)
"""The tools the cradle's guide names, grouped (design review round 2's m10: some 24, named
only inside the steps): the guide's cover and ``ASSEMBLY.md`` list them."""

BOARD_TEXT = (
    "The board stays bare until both sides stand on it: wood glue holds bare wood, not a "
    "finish, so its cleats are glued first and it is finished after (the cleats and its "
    "finish: the last step before it's used).",
    "First sand the router's tabs flush on the board's routed edges: SendCutSend's router "
    "leaves small tabs where it held the board (about 4.8 mm wide, half the birch deep; it "
    "can't detab birch). The cleats and the riser locator's layers come off the Carvera "
    "with their own tabs sanded already (the shop step), the cleats' long faces before all: "
    f"a nub there takes up the rail's {CLEAT[1]:g} mm of play.",
    "The riser locator's two layers glued face to face, bare, their outlines flush (wood "
    "glue, weighted 30 minutes); it drops into the notch, its ears in the notch's bays.",
    "The TPU feet pushed up into the board's corner holes (a twist helps them through).")
"""The base board's step's first sentences (:func:`board_text`; design review round 3's
laser-cut parts minor 7: the router's tabs, SendCutSend's birch page and sheet-cutting
guidelines, fetched 2026-10-20)."""
BRACKET_SIDE = (
    "At each foot a printed bracket outside the outer plate: an M5 button head through the "
    "bracket and the plate, its nut over the plate's inside (an 8 mm wrench, threadlocker: "
    "the bracket stays on its side).")
BRACKETS = (
    BRACKET_SIDE + " Over its carriage bolt, on the bracket's foot, an 18-8 fender washer, "
    "then a printed thumb knob with an 18-8 M6 nut pressed up into its hex pocket (once, on "
    "the bench, in a vise), turned down by hand (snug: the foot is printed; no threadlocker, "
    "it comes off to pack). The foot's hole is 2.5 mm over the bolt: the side settles where "
    "the linkage puts it.")
BRACKETS_BOARD = (
    "The four printed brackets onto the board, each foot over its carriage bolt, an 18-8 "
    "fender washer and an 18-8 M6 nylon-insert nut on it, run down with the 10 mm wrench "
    "until it meets the washer and no further for now, the bracket still free to turn (its "
    "insert grips: it won't spin on by hand). They're snugged once the cradle has set them "
    "where its sides want them, and stay on the board: no knob turns there again. The "
    "foot's hole is 2.5 mm over the bolt.")
STUD_WORDS = (
    "At each foot of the outer plate a stud: an M5 button head from the plate's inside out "
    "through its foot hole (a 3 mm hex key on its head), its jam nut on the plate's outer "
    "face, an 8 mm wrench, threadlocker (it stays). Then an 18-8 M5 nut pressed into each "
    "stud's printed knob from outside (once, in a vise)")
STUDS = (
    STUD_WORDS.replace("At each foot of the outer plate", "The panel's feet over the bench's "
                       "edge, at each foot of its outer plate", 1)
    + " and the knob onto its stud, a few turns: it clamps the bracket's upright to the "
    "plate, by hand.")
DROP_DRIVE = (
    "Then the drive side onto the board, upright: lower it straight down, its outer plate's "
    "rail on the board and the notch at the front, each stud's jam nut dropping into its "
    "bracket's open slot (the slots' chamfered mouths guide it), and a knob onto each stud, "
    "snug by hand: its two brackets hold it standing from here on (it can't stand alone: "
    "the gearbox outboard, the cradle link and the tie rods inboard).")
"""B's base board step (:data:`BOARD_BRACKETS`; design review round 3's stand and use, M3):
the drive side's studs go on at the bench (:func:`studs_first`), its brackets on the board,
and the side drops into their open slots, so nobody holds 8 kg upright on a 4.75 mm edge
with one hand while a hex key and a wrench work either side of the plate (and the jam nut
needn't be turned inside the 12 mm slot, where a wrench can't reach)."""


def board_text(key: str) -> tuple[str, ...]:
    """The base board's step's sentences (``base``): :data:`BOARD_TEXT`, the carriage bolts,
    then for a cradle whose brackets stay on its board (:data:`BOARD_BRACKETS`) the brackets
    and the drive side dropped into their slots on its studs (:func:`studs_first`), for one
    whose brackets come off with it the drive side, then its brackets and thumb knobs."""
    board = key in BOARD_BRACKETS
    bolts = ("The M6 carriage bolts up through its square holes from under, each square neck "
             "in its hole (it can't turn: nothing to reach under the board later" + (
                 "; each holds its bracket for good)." if board else
                 "; it hangs there until its knob is on, so the knobs go back on whenever the "
                 "cradle is off)."))
    if board:
        return (*BOARD_TEXT, bolts, BRACKETS_BOARD, DROP_DRIVE)
    return (*BOARD_TEXT, bolts, "The drive side onto the board, its outer plate's rail on "
            "it, the notch at the front.", BRACKETS)


def studs_first(key: str) -> str:
    """The drive side's studs, on the bench before it stands on the board (``studs_L``: B's,
    :data:`DROP_DRIVE`), the side lying on the riser's blocks as the trip lays it."""
    return ("On the bench first, before it stands: lay the drive side outer plate down on "
            "the riser's two blocks, each on its lift, and its gearbox, as the trip lays it, "
            f"on a folded towel: {_blocks_words(key)} (the cradle link's tie rods stand up). "
            + STUD_WORDS.replace("its jam nut on the plate's outer face", "its jam nut on "
                                 "the plate's outer face, reached under it (the blocks stand "
                                 f"it {RISER_LIFT + RISER_BLOCK[1] + RISER_LIP:.0f} mm off the "
                                 "bench)", 1)
            + ": the knobs go on once it stands on the board.")
TITEBOND_FAQ = "https://www.titebond.com/resources/use/all/faqs"


def _blocks_words(key: str) -> str:
    """Where the riser's blocks go under the drive side's outer plate (:data:`LAY_BLOCKS`)."""
    nodes = LAY_BLOCKS.get(key, ())
    words = []
    for n in nodes:
        if not isinstance(n, str):
            words.append("one under the rear leg just over its foot's bracket")
        elif n.startswith("O"):
            words.append(f"one under pillar {n}'s screw head (the cone in the block's top "
                         "over the head)")
        elif n.startswith("E"):
            words.append(f"one under column {n}'s screw head on the rail")
        else:
            words.append(f"one under column {n}'s screw head on the front leg")
    return ", ".join(words)


def _block_names(key: str) -> list[str]:
    """Each lay-flat block's place in words (:data:`LAY_BLOCKS`): ``"pillar O1"``."""
    out = []
    for n in LAY_BLOCKS.get(key, ()):
        if not isinstance(n, str):
            out.append("the rear leg")
        elif n.startswith("O"):
            out.append(f"pillar {n}")
        else:
            out.append(f"column {n}")
    return out


def _block_xy(key: str) -> tuple[np.ndarray, list[np.ndarray]]:
    """The gearbox's axis (the crank's centre) and each lay-flat block's middle
    (:data:`LAY_BLOCKS`, as ``cradle_pack.lay_blocks`` stands them), in the side's plane."""
    from spiderpig import linkage

    lk = linkage.get(key)
    pts = lk.solve().evaluate(np.array([0.0]))
    stand = stand_of(lk)
    mouth = pts[mouth_name(lk)][0]
    at = []
    for n in LAY_BLOCKS.get(key, ()):
        if not isinstance(n, str):
            at.append(mouth + np.asarray(n, float))
        elif n in stand.nodes:
            at.append(mouth + np.asarray(stand.nodes[n], float))
        else:
            at.append(pts[n][0])
    return pts[lk.crank[0]][0], at


LIFTS: dict[str, dict[str, float]] = {"decanter_rcm": {"cradle": 12.7, "drive_box": 9.9},
                                      "decanter_hinge": {"cradle": 12.0}}
"""What a person lifts onto and off the lay-flat blocks, or off the board (kg;
``cradle_pack.rest_poses``' masses, held to them in ``tests/test_cradle.py``): B's cradle off
its board, and its drive side with the cradle box (the first build's lay); A's cradle off its
board (its first build: the board's finish). Two people, one each side, or one in the order
:func:`lay_down` gives (design review round 3's stand and use, M5: lifted by its outer
plates' 4.75 mm top edges 430 mm apart, 465 mm over a counter, the NIOSH lifting equation's
recommended limit is 11.7 kg at most)."""


def lay_down(key: str, what: str = "cradle") -> str:
    """How ``what`` (a key of :data:`LIFTS`) goes down onto the lay-flat blocks: two people,
    one lowering, one setting each block under its head; or one, the gearbox's end down
    first, the other end held up while the blocks slide in under it, the one nearer the
    gearbox first (design review round 3's stand and use, M5)."""
    names = _block_names(key)
    o, at = _block_xy(key)
    order = sorted(range(len(at)), key=lambda i: float(np.linalg.norm(at[i][:2] - o[:2])))
    kg = LIFTS.get(key, {}).get(what)
    apart = (round(float(np.linalg.norm(at[0][:2] - at[1][:2])) / 5.0) * 5.0
             if len(at) == 2 else 0.0)
    slide = ", then ".join(f"{names[i]}'s" for i in order)
    heads = all(isinstance(n, str) for n in LAY_BLOCKS.get(key, ()))
    washer = 12.0                       # a post's DIN 125 M6 washer under its screw head
    lands = (f"a head lands in the cone in its block's top within "
             f"{(RISER_RELIEF[1] - washer) / 2:g} mm of its middle"
             + ("; " if apart else "")) if heads else ""
    return ((f"About {kg:g} kg: two" if kg else "Two") + " people if you can, one lowering "
            "it level, the other setting each block in its place (" + lands + (
                f"their middles about {apart:g} mm apart, each block's long side along the "
                "plate" if apart else "") + "). Alone: lower its gearbox's end onto the "
            "towel first, holding the other end up; with your other hand slide in the blocks, "
            f"{slide}, and lower that end onto them.")


def pin_words(lk, pin: str) -> str:
    """Where pin ``pin`` of the cradle link (:func:`trip_pins`) is, in words: the link it
    joins (the longer or the shorter of the two) and which of them lies nearer the gearbox
    (design review round 3's stand and use, m1: X and Y are the same clevis pin with the
    same keeper, and nothing had told them apart)."""
    pins = trip_pins(lk)
    link = cradle_link(lk)
    pts = lk.solve().evaluate(np.array([0.0]))
    o = pts[lk.crank[0]][0]

    def other(p: str) -> str:
        return next(b for b, (js, _) in lk.links.items() if b != link and p in js)

    def length(b: str) -> float:
        js = lk.links[b][0]
        return max(float(np.linalg.norm(pts[a][0] - pts[c][0])) for a in js for c in js)

    def near(p: str) -> float:
        return float(np.linalg.norm(pts[p][0] - o))

    b = other(pin)
    rest = [q for q in pins if q != pin]
    if not rest:
        return f"pin {pin}, in the link {b}"
    longer = all(length(b) > length(other(q)) for q in rest)
    nearer = all(near(pin) < near(q) for q in rest)
    return (f"pin {pin}, in the {'long' if longer else 'short'} link {b}, "
            f"{'nearer' if nearer else 'farther from'} the gearbox")


def _lay_first(key: str) -> tuple[str, str]:
    """The first build's step laying B's drive side and its cradle box down (``lay``)."""
    from spiderpig import linkage

    words = PACK_POSES[key].words if key in PACK_POSES else ""
    marks = PACK_POSES[key].marks if key in PACK_POSES else ""
    lk = linkage.get(key)
    pins = trip_pins(lk)
    first = pin_words(lk, pins[0]) if pins else "the first pin"
    return ("Crank it to the pack pose: from rest, about 5½ turns the way that swings the "
            f"crank's arm down, until the arm lies along {words} (seen from outside, the leg "
            "hides it). Mark the pose for the trip with a paint pen across the drive side's "
            f"links: {marks}; and paint an X beside pin X's hole on each cradle link, the "
            f"passive one too before it goes on ({first}): it goes in first on every way "
            "back. Take the hand crank off (its thumb screw: lying down, its arc would meet "
            "the towel). " + (
                "Loosen the drive side's two knobs a few turns (two at least) and lift it "
                "straight up off the board (its studs rise out of the brackets' slots) by its "
                "outer plate's top member" if key in BOARD_BRACKETS else
                "Undo the drive side's two thumb knobs and lift it off the board by its outer "
                "plate's top member") + ", a hand under the cradle box; lay it on its "
            "drive side on a folded towel, on the riser's two blocks, each on its lift (the "
            f"riser's bottom pieces): {_blocks_words(key)}. {lay_down(key, 'drive_box')} It "
            "lies level on them and the gearbox, the tie rods standing up; keep a hand on the "
            "cradle box while you work on it.", "lay",)


def lower_first(lk) -> tuple[str, tuple[str, ...]]:
    """The first build's passive side panel onto its cradle link, the cradle lying on its
    drive side (design review round 2's M1: as the trip's way back, :func:`module_steps`):
    ``(text, lines)``."""
    pose = PACK_POSES[lk.key]
    pins = trip_pins(lk)
    names = " and ".join(pins)
    second = pins[1] if len(pins) > 1 else ""
    nudge = pose.nudge or "its link"
    return ("Set the panel's links to the drive side's pose, the mirror image: b3 at the same "
            "angle to its inner bar, b5 to b3 (the drive side's paint lines show it), and "
            "tape b3 to the inner bar with a strip of masking tape (" + nudge + " stays free: "
            "it is the one nudged into place).", (
                f"Its pins {names} stand in their links on their keepers, their clips out; "
                "turn the panel over and hold it level, its outer plate up: each pin hangs in "
                "its link by its head, its keeper holding it.",
                "Lower the panel level onto the cradle link, holding its weight until both pins "
                f"are in their holes: {pin_words(lk, pins[0])} (its paint X), into its hole "
                "first (look in through the bay)" + (
                    f"; if pin {second} doesn't line up, nudge {nudge} until it does"
                    if second else "") + ". Never let it rest on a pin's tip: the pin's "
                "keeper would let it slide back up its link; if one has, lift the panel and "
                "push the pin home by its head.",
                "Peel the tape off; mark this side's pose with the paint pen too, as the drive "
                "side's."))


ROOM_FREE = 150.0
"""The room a cradle needs free behind its base board and beside its drive side (mm; design
review round 3's stand and use, m5: "100 mm behind" had been the base stop's rails' reach, 42
mm past B's rear edge at full tilt). The hand crank's grip turns about the input shaft, at
the same reach at every crank angle; measured on the fabricated cradles 2026-10-20 (their
pack input's parts): B's grip bolt's head 103 mm past the board's rear edge and the grip 76 mm
past its drive-side edge at its outermost, a hand on it 118 and 104 mm
(``cradle_report.HAND_OUT``, ``HAND_AROUND``); A's grip inside the rear edge, 102 mm past the
drive-side edge, a hand 130 (:data:`HAND_REACH`)."""
HAND_REACH: dict[str, float] = {"decanter_rcm": 120.0, "decanter_hinge": 130.0}


def full_turns(lk) -> float:
    """Handle turns from rest to full tilt (the bottle's least tilt to its most), the way
    the cradle page counts them (``cradle_report.motion``'s ``turns_to_full``)."""
    from spiderpig.construction.handcrank import NRV050

    n = 2880
    ts = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    pts = lk.solve().evaluate(ts)
    o, ax = mouth_frame(lk)
    d = pts[ax] - pts[o]
    tilt = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    rest, full = int(np.argmin(tilt)), int(np.argmax(tilt))
    return (full - rest) % n / n * NRV050.ratio


def level_turns(lk) -> float:
    """Handle turns from rest to where the bottle lies level (tilt 0), the shorter way."""
    from spiderpig.construction.handcrank import NRV050

    n = 2880
    ts = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    pts = lk.solve().evaluate(ts)
    o, ax = mouth_frame(lk)
    d = pts[ax] - pts[o]
    tilt = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    k = int(np.argmin(tilt))
    order = np.roll(np.arange(n), -k)
    j = next(i for i, m in enumerate(order) if tilt[m] >= 0.0)
    return j / n * NRV050.ratio


def _board_finish(key: str, modules: bool) -> tuple[str, str, tuple[str, ...]]:
    """The last build step: both sides on the board, its cleats glued and its finish on
    (design review round 2's M6, 2026-10-14: it had been oiled and sealed, then glued), the
    passive side's knobs snugged at level (its m6), the checks: ``(text, tag, lines)``."""
    from spiderpig import linkage

    lk = linkage.get(key)
    turns = _turns_words(level_turns(lk))
    board = key in BOARD_BRACKETS
    lines = []
    if modules:
        lines.append((
            "Stand the cradle up by its two outer plates' top members (a hand under the cradle "
            f"box; {LIFTS[key]['cradle']:g} kg, two people if you can, one each side) and "
            if key in LIFTS else
            "Stand the cradle up by its two outer plates' top members (a hand under the cradle "
            "box) and ") + (
                "lower it onto the board, each stud's jam nut down into its bracket's slot (the "
                "brackets' nuts still loose: they move to where the sides put them); the knobs "
                "snug by hand, the passive side's only started." if board else
                "set it on the board, each bracket's foot over its carriage bolt (its hole is "
                "2.5 mm over the bolt); the washers and the thumb knobs on, by hand, the "
                "passive side's only started."))
    lines += [
        "Each rail's cleats (a pair between each two of its brackets), their chamfered "
        f"lead-in ({CLEAT_LEAD:g} mm, cut on the Carvera) up and to the rail, so the rails find "
        "their slots blind on the way back: slide one against each face of the rail, nothing "
        "between (the rail is their gauge: it drops back in under a hand's press), and pencil "
        "round them. A cleat in pieces (B's, two each) goes end to end on its line, "
        f"{CLEAT_JOINT:g} mm apart.",
        "The hand crank off (its thumb screw); " + (
            "loosen the knobs a few turns (two at least) and lift the cradle straight up off "
            "its brackets (they stay on the board, finished round)" if board else
            "undo the thumb knobs and lift the cradle off") + " by "
        "its outer plates' top members" + (
            f" ({LIFTS[key]['cradle']:g} kg, two people if you can, one each side)"
            if key in LIFTS and key not in LAY_BLOCKS else "") + (
            "; lay it on its drive side on a folded towel, on the riser's two blocks, each on "
            f"its lift: {_blocks_words(key)}, a hand on it until it settles. {lay_down(key)}"
            if key in LAY_BLOCKS else
            "; stand it on the bench or the floor beside the board on its brackets' feet, as "
            "it stood on the board: it stands there while the glue and the finish cure."),
        "Glue the cleats on their marks to the bare board, each piece along its whole length "
        "(Titebond II, a thin bead, weighted or clamped 30 minutes); wipe off the "
        "squeeze-out, out of the lead-in above all.",
        "Then the finish, on bare wood only after the glue (Titebond's FAQ: its glues bond "
        "bare wood, and over a finish a bond holds only as well as the finish holds the "
        f"wood; {TITEBOND_FAQ}): polyurethane every face and edge of the board, the cleats "
        "and the riser locator (the notch's end grain above all: it would wick wine; the "
        "locator stands on the table, in any spill). Two or three coats, sanded lightly "
        "between, as its can says; let it cure before the cradle goes back on.",
        "Stand the cradle back on the board, each rail between its cleats; " + (
            "each stud's jam nut down into its bracket's slot, the knobs" if board else
            "the washers and knobs on") + ", the drive side's snug by hand, the passive "
        "side's only started; the hand crank back on.",
        f"Turn the handle to level ({turns} turns from rest: the bottle would lie level) "
        "with the passive side's knobs loose, then snug them there: that side settles where "
        "the linkage puts it" + (", and B's two remote centres agree best at level (round "
        "2's linkage review: snugged at rest, the forced misfit had been nearly twice)."
        if modules else ".") + (
            " Then the four brackets' nylon-insert nuts snug, the cradle on them, to "
            f"{FOOT_NUT[2]:g} N·m: the 10 mm wrench pulled square by the luggage scale hooked "
            f"100 mm from the nut, to {FOOT_NUT[2] / 0.1 / 9.81:.1f} kg (no more: the foot is "
            "printed; the insert holds it there). They stay so, and every trip after drops the "
            "studs into the same slots." if board else ""),
        "Then a whole pour and back, the bottle out: both cradle links swing freely and the "
        f"handle turns evenly, under {EMPTY_GRIP_N:g} N at the grip (the luggage scale "
        f"hooked on the grip: under {EMPTY_GRIP_N / 9.81:.1f} kg); and "
        "the rod's paper check again (the hand crank's step), now that the tie rods are held "
        "at both ends."]
    return lines[0], "board", tuple(lines[1:])



@dataclass(frozen=True)
class PackPose:
    """Where a cradle that packs as modules is cranked to first (:data:`PACK_POSES`): the
    crank's arm (O to its crankpin) along ``member`` of the drive side's outer plate (a
    member of :class:`Stand`'s ``outer``), a mark seen without a tool or a count; ``words``
    names it, ``marks`` the paint-pen lines that set the passive side's links back to it."""

    member: tuple[str, str]
    words: str
    marks: str
    nudge: str = ""             # the link the second pin stands in (swung to drop it in)


PACK_POSES: dict[str, PackPose] = {
    # B: the arm straight down along the rear leg O-Fr, about 5 1/2 turns from rest the way
    # that swings it down: the bottle 31° neck up, inside the 44-20° where the passive cradle
    # link clears its inner bar and the box doesn't grow (round 1's stand review; "4 turns"
    # had been 35° one way, 39° the other, and 44-47° for a rest called early)
    "decanter_rcm": PackPose(
        ("O", "Fr"), "the drive side's outer plate's rear leg, straight down from the gearbox",
        "a line across the arm b3 and the inner bar at its O1 end, and one across the link b5 "
        "and b3 by its pin J2", "b5"),
}
"""The cradles that pack as modules (``cradle_pack.module_pieces``, the user's of
2026-10-13), each one's pack pose (:func:`pack_pose`). A (not here) packs flat: its cradle
link rides pillar G between its side's plates."""


@dataclass(frozen=True)
class Rest:
    """A way the guide sets a piece, or pieces together, down on a table
    (:data:`REST_POSES`; rated by ``cradle_pack.rest_poses``: the face of their hull they
    settle on, its tilt, the tip angle and the push at their top that tips them). ``pieces``:
    ``cradle_pack``'s keys (``drive``, ``box``, ``passive``, ...; ``"*"``: all but the board,
    the hand crank and the riser, the cradle off its board); ``down``: ``"drive"`` the drive
    side's outer plate down, ``"passive"`` the passive side's, ``"top"`` the cradle box
    levelled and turned over, its saddles' open tops down (on its two cradle links' long
    edges), ``"feet"`` standing as on its board (on its brackets' feet); ``blocks``: on the
    riser's two blocks under the drive side's outer plate (:data:`LAY_BLOCKS`)."""

    key: str
    words: str
    pieces: tuple[str, ...]
    down: str
    blocks: bool = False


REST_POSES: dict[str, tuple[Rest, ...]] = {
    "decanter_rcm": (
        Rest("cradle", "the cradle off its board, lying on its drive side on the riser's two "
                       "blocks", ("*",), "drive", True),
        Rest("drive_box", "the drive side and the cradle box lying on the blocks, the passive "
                          "panel off", ("drive", "box"), "drive", True),
        Rest("drive", "the drive side panel alone on the blocks (the way back starts here)",
             ("drive",), "drive", True),
        Rest("box", "the cradle box on its two cradle links' long edges, its saddles' open "
                    "tops down", ("box",), "top"),
        Rest("passive", "the passive side panel on its outer plate", ("passive",),
             "passive"),
    ),
    "decanter_hinge": (
        Rest("cradle", "the cradle off its board (the board's finish curing), standing on its "
                       "six brackets' feet on the bench", ("*",), "feet"),
    ),
}
"""Each cradle's resting poses in the guide (design review round 2's M2, 2026-10-14), rated
in ``tests/test_cradle.py``. Round 1's had rested B's cradle and its drive panel on the
gearbox, a screw head and a printed bracket, 14° tilted (the drive panel alone tipped at 6.8°
and 2.5 N), and its box "as it was, on its other cradle link", on two tie screws' heads
(10.0°, 2.8 N; its "41 mm margin" was to the link's outline, which it never sat on). A's had
lain on its drive side on the blocks too, under column C1 and the rear leg; design review round
3's frames took its centre of mass toward its gearbox, and it tipped at 13.4° under the 15° bar
wherever the blocks went: it stands on its brackets' feet instead (2026-10-20), as on its
board."""

LAY_BLOCKS: dict[str, tuple[str | tuple[float, float], ...]] = {
    "decanter_rcm": ("O1", "E1"),
}
"""Where the riser's two blocks (:data:`RISER_BLOCK`) stand under the drive side's outer plate
when the cradle lies on its drive side: each centred on a post's screw head on the plate's
outer face (a pillar's or a stand column's, by its node; the head in the relief in the
block's boss, :data:`RISER_RELIEF`), or at a point (x, y from the mouth), the block's length
along x. With the gearbox at O they make a triangle round the centre of mass
(:data:`REST_POSES`). B's: under pillar O1 (the plate's top front) and the rail's front
column E1 (since the side frames' redesign, 2026-10-20: under the front leg's column D2 the
drive panel alone, heavier low on the rail, tipped at 25.3°; under E1 at 39.7°). A has none: off
its board it stands on its brackets' feet (:data:`REST_POSES`)."""
RISER_RELIEF = (13.5, 26.0, 3.6)
"""The blocks' bosses' relief over a post's screw head (mm; design review round 3's stand and
use, M5 and m2): a bore this far across through the block's top into its cup (E1's and O1's
heads stand 4.9 mm proud on their 12 mm washers, deeper than the boss and top's 5 mm), under
a cone from this far across at the boss's top, this deep (30° off the boss's face): a head
set down within 6 mm of its block's middle slides down the cone into the bore, and the plate
bears on the boss round it, not on the washer (the 12 x 3.6 mm relief it replaced had
taken neither, the plate 1.3-1.6 mm off the boss)."""


def pack_pose(lk) -> tuple[float, float, float] | None:
    """The crank angle (radians) cradle ``lk`` packs at (:data:`PACK_POSES`: its arm along the
    member), the bottle's tilt there (degrees) and the handle turns from rest to it (the
    shorter way: its sign the crank's; rest, the bottle's least tilt); ``None``: it packs
    flat. The arm's direction is the crank angle's own, so the pose is one crank angle."""
    from spiderpig.construction.handcrank import NRV050

    pose = PACK_POSES.get(lk.key)
    if pose is None:
        return None
    n = 2880
    ts = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    pts = lk.solve().evaluate(ts)
    o, ax = mouth_frame(lk)
    stand = stand_of(lk)

    def node(name: str) -> np.ndarray:
        if name in stand.nodes:
            return pts[o][0] + np.asarray(stand.nodes[name], float)
        return pts[name][0]

    a, b = (node(m) for m in pose.member)
    want = math.atan2(b[1] - a[1], b[0] - a[0])
    c0, c1 = lk.crank
    arm = pts[c1] - pts[c0]
    off = np.angle(np.exp(1j * (np.arctan2(arm[:, 1], arm[:, 0]) - want)))
    k = int(np.argmin(np.abs(off)))
    # the crank turns uniformly with t: refine to the exact angle
    rate = np.sign(np.gradient(np.unwrap(np.arctan2(arm[:, 1], arm[:, 0])))[k])
    t = float((ts[k] - rate * off[k]) % (2.0 * math.pi))
    d = pts[ax] - pts[o]
    tilt = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    rest = float(ts[int(np.argmin(tilt))])
    at = lk.solve().evaluate(np.array([t]))
    dt = at[ax][0] - at[o][0]
    turn = (t - rest + math.pi) % (2.0 * math.pi) - math.pi        # the shorter way
    return t, math.degrees(math.atan2(dt[1], dt[0])), turn / (2.0 * math.pi) * NRV050.ratio


def _arm_way(lk, turns: float) -> str:
    """Which way the crank's arm swings from rest the way ``turns`` goes: ``down`` or ``up``
    (its crankpin's height)."""
    n = 2880
    ts = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    pts = lk.solve().evaluate(ts)
    o, ax = mouth_frame(lk)
    d = pts[ax] - pts[o]
    k = int(np.argmin(np.unwrap(np.arctan2(d[:, 1], d[:, 0]))))
    y = pts[lk.crank[1]][:, 1]
    step = 1 if turns > 0 else -1
    return "down" if y[(k + step * 40) % n] < y[k] else "up"


EMPTY_GRIP_N = 5.0
"""The bench check after the cradle goes back together (:func:`module_steps`): the bottle
out, the handle turns with under this at its grip (``cradle_report.report``'s
``empty_handle_force_n``: B 3.6 N on 2026-10-14); more, and the two sides fight each other
(B's two remote centres don't agree: round 1's linkage review)."""


def _turns_words(turns: float) -> str:
    """``5.6`` -> ``about 5½``: to the half turn."""
    halves = round(abs(turns) * 2)
    whole, half = divmod(halves, 2)
    return f"about {whole}{'½' if half else ''}" if whole else "about ½"


@dataclass(frozen=True)
class ModuleStep:
    """One step of a cradle's trip apart or back together (:func:`module_steps`): a short
    ``title``, its ``text`` and more ``lines`` (each its own item in the guide), a ``tag``
    (the pieces the breakdown's step frees, ``cradle_pack.module_pieces``: ``hand_crank``,
    ``base``, ``lift``, ``box``; the others name what they do) and its picture (``scene``, a
    ``construction.assembly.Scene``). To add a step, add one here: the guide numbers and
    draws it, the pack's payload and the cradle page list its text."""

    title: str
    text: str
    tag: str = ""
    lines: tuple[str, ...] = ()
    scene: object = None      # a construction.assembly.Scene

    @property
    def words(self) -> str:
        """Its text and lines as one paragraph (the pack's payload, the cradle page)."""
        return " ".join((self.text, *self.lines))


def trip_pins(lk) -> list[str]:
    """The cradle link's pins shared with another link on a side (B's X and Y): those the trip
    pulls (:func:`module_steps`)."""
    link = cradle_link(lk)
    return sorted(j for j in lk.links[link][0]
                  if any(j in js for b, (js, _) in lk.links.items() if b != link))


def module_steps(lk, pins) -> tuple[list[ModuleStep], list[ModuleStep]]:
    """How cradle ``lk`` (:data:`PACK_POSES`) comes apart into its modules and goes back
    together, for ``cradle_pack.module_pieces`` and the guide (:func:`assembly`), each step a
    :class:`ModuleStep`. ``pins``: the cradle link's pins on each side (B's X and Y:
    :func:`trip_pins`).

    Laid on its drive side (the gearbox and the riser's two blocks under it,
    :data:`LAY_BLOCKS`), the passive side panel lifts straight up off the pins, level, its
    links unable to swing; then the cradle box lifts straight up off the drive side's pins (the
    pack's turn 1, 2026-10-14: on its own the box packs in about two thirds of the box it took
    on the drive side) and is set on its cradle links' long edges, its saddles' tops down.
    Each pin of a side lying down is held by its TPU keeper (:data:`KEEPER`): its head is on
    its link's outer face (:mod:`construction.pivots.clevis`), under the link with nothing
    under it, and a bare pin drops out once its clip is out. The way back holds each piece's
    weight until both its pins are in their holes: a link left resting on a pin's tip can
    push it out through its keeper (design review round 2's M3). Standing, the 6.8 kg drive
    module tipped onto its cradle box once the pins cleared, and the panel fell the other way
    (round 1's stand review); the box can't slide off a side that stands on the board (its
    bolts and cleats hold the side), and of the two sides only the drive side holds the box
    (the worm): the passive side's links are free, so the passive panel comes off first.
    Every resting pose is rated (:data:`REST_POSES`, ``cradle_pack.rest_poses``)."""
    from spiderpig.construction.assembly import Scene

    pose = PACK_POSES[lk.key]
    got = pack_pose(lk)
    assert got is not None
    _, tilt, turns = got
    names = " and ".join(pins)
    which = "Pin" + ("s " if len(pins) > 1 else " ") + names
    each = "each one's R-clip" if len(pins) > 1 else "its R-clip"
    way = _arm_way(lk, turns)
    second = pins[1] if len(pins) > 1 else ""
    nudge = pose.nudge or "its link"
    blocks = _blocks_words(lk.key)
    level = _turns_words(level_turns(lk))
    clips = {s: (rf"{s}\.pin_({'|'.join(pins)})_clip",) for s in SIDES_LR}
    pin_x = {s: rf"{s}\.pin_{pins[0]}_pin" for s in SIDES_LR}       # (its callout)
    pins_re = {s: rf"{s}\.pin_({'|'.join(pins)})_pin" for s in SIDES_LR}
    three = ("drive", "box", "passive", "blocks")
    hold = ("Never let it rest on a pin's tip: the pin's keeper would let it slide back out "
            "of its link; if one has, lift the piece off and push the pin home by its head.")
    knobs = 2 * len(stand_of(lk).feet)
    kg = LIFTS.get(lk.key, {}).get("cradle")
    lift = f"{kg:g} kg: " if kg else ""
    first = pin_words(lk, pins[0]) if pins else ""
    if lk.key in BOARD_BRACKETS:        # (the brackets stay on the board: L1)
        back = f"{KNOB_BACK_TURNS * M5_PITCH:.1f}"
        off = (f"Base board: loosen the {knobs} knobs on the outer plates' studs by hand, a "
               "few turns each (two at least), and lift the cradle straight up off the board "
               f"by its two outer plates' top members ({lift}two people if you can, one each "
               "side), never by the cradle box (only the worm holds it): each stud and its jam "
               "nut rise out of its bracket's open slot, the rails out of their cleats. The "
               "brackets stay bolted to the board. Turn each knob back down onto its jam nut, "
               "snug, for the trip.")
        on = (f"First back each knob off its jam nut {_WORDS[KNOB_BACK_TURNS]} full turns: it "
              f"then stands {back} mm off the nut, a fingernail, and the bracket's "
              f"{BRACKET[2]:g} mm upright passes between it and the plate (snug on its nut it "
              f"leaves {M5_NUT[1]:g} mm: the cradle would hang on the uprights by its knobs).",
              f"Then stand the cradle up by its two outer plates' top members ({lift}two "
              "people if you can, one each side) and lower it straight onto the base board: "
              "each rail between its two cleats, each stud's jam nut down into its bracket's "
              "slot (the cleats' lead-ins and the slots' chamfered mouths guide it down).",
              "The drive side's knobs snug by hand, the passive side's only started.")
    else:
        off = ("Base board: undo the brackets' thumb knobs by hand and lift the cradle off by "
               "its two outer plates' top members, never by the cradle box (only the worm "
               "holds it). The carriage bolts stay in the board, their washers and knobs back "
               "on them.")
        on = ("Stand the cradle up by its two outer plates' top members and set it on the base "
              "board: each rail between its two cleats (their lead-ins guide it down), each "
              "bracket's foot over its carriage bolt (its hole is 2.5 mm oversize).",
              "The washers and the thumb knobs on: the drive side's snug by hand, the passive "
              "side's only started.")
    breakdown = [
        ModuleStep(
            "the pack pose",
            "Crank it to the pack pose: from rest (the bottle neck up, where the cradle stops "
            f"coming back), turn the handle the way that swings the crank's arm {way}, "
            f"{_turns_words(turns)} turns, until the arm lies along {pose.words} (seen from "
            f"outside, the leg hides it). The bottle is then {abs(tilt):.0f}° neck up, and "
            "each side's cradle link clears that side's inner bar.",
            "pose",
            (f"The paint pen's lines from the first build line up there ({pose.marks}); if "
             "they have worn, mark them again.",
             "The decanter away, take the riser out of its locator: its plates, then its two "
             "blocks, each on its lift (the next steps lay the cradle on them)."),
            Scene("pack", show=("*", "risers"), marks=(r"L\.conn\w*", r"L\.crankpin\w*"))),
        ModuleStep(
            "the hand crank off",
            "Hand crank: loosen its thumb screw by hand and pull it off the gearbox's input "
            "shaft; the screw goes back into its hub. Nothing turns without it.", "hand_crank",
            scene=Scene("pack", show=("*",), moves=(("hand_crank", 160.0, 0.0, 0.0),),
                        marks=(r"L\.handle\w*",))),
        ModuleStep(
            "off the board", off, "base",
            scene=Scene("pack", show=("board", "drive", "box", "passive"),
                        moves=tuple((k, 0.0, 140.0, 0.0) for k in ("drive", "box", "passive")),
                        marks=(r"foot_bracket_\w+_knob\d?",))),
        ModuleStep(
            "laid on its drive side",
            "Stand the riser's two blocks, each on its lift, on a folded towel and lay the "
            "cradle on its drive "
            f"side on them and its gearbox: {blocks}. It lies level, the passive side up: "
            "gravity runs along the pins, so no link can swing.", "lay",
            (lay_down(lk.key),
             "It tips if pushed at its top: keep a hand on the cradle box while the other "
             "works, down to the box's own step.",),
            Scene("pack", "drive", show=three, marks=(r"decanter_riser_block\d",))),
        ModuleStep(
            "the passive side's clips",
            f"{which} (the cradle link's, on the passive side): reach into the bay and pull "
            f"{each} by hand from under the passive side's cradle link. Each pin stays in the "
            "panel's link, standing on its head, its TPU keeper on it.", "clips",
            scene=Scene("pack", "drive", show=three, marks=clips["R"])),
        ModuleStep(
            "the passive side panel off",
            "Passive side panel: lift it straight up off the pins by its outer plate, level, "
            "the pins and their keepers coming with it, and push each pin's clip back through "
            "its hole; then set the panel down on its outer plate on the folded towel (its "
            "studs' ends stand past their knobs: they'd scratch a counter). The drive side "
            "with the cradle box stays lying as it is, the worm holding it.", "lift",
            scene=Scene("pack", "drive", show=three, moves=(("passive", 0.0, 170.0, 0.0),),
                        marks=(r"R\.frame_outer",))),
        ModuleStep(
            "the drive side's clips",
            f"{which} on the drive side: pull {each} by hand, reaching in from the side "
            "between the drive side's cradle link and its inner plate (the cradle box is open "
            "at its sides), a hand on the box. Each pin stays in its link: its head is under "
            "the link, and its TPU keeper on the link holds it.", "box_clips",
            scene=Scene("pack", "drive", show=("drive", "box", "blocks"), marks=clips["L"])),
        ModuleStep(
            "the cradle box off",
            "Cradle box: lift it straight up off the drive side's pins, level, by its top "
            "cradle link and a tie rod (about 3.5 kg). Turn it a quarter about its tie rods "
            "and set it down on its two cradle links' long edges, its saddles' open tops down "
            "(the base stop's knob is the third point; the tie rods level, so nothing weighs "
            "on the saddles' set screws). Push each pin's clip back through its hole.", "box",
            (f"The drive side stays on its blocks; its link {pose.nudge or 'b5'} now turns "
             "free on its one pin, flat. The riser's blocks and lifts go back with its "
             "plates for the trip.",),
            Scene("pack", "drive", show=("drive", "box", "blocks"),
                  moves=(("box", 0.0, 200.0, 0.0),), marks=(r"[LR]\.b6",))),
    ]
    reassembly = [
        ModuleStep(
            "the drive side on its blocks",
            "Stand the riser's two blocks, each on its lift, on a folded towel and lay the "
            "drive side panel on "
            f"them and its gearbox, outer plate down: {blocks}.", "drive",
            (f"Its pins {names} stand in their links on their keepers, their clips out: push "
             "each down by its head, fully home. Then press each one's tip with a thumb, "
             f"hard (about {KEEPER_PUSH_N / 9.81:.0f} kg): it mustn't slide; a keeper that "
             "lets its pin slide has relaxed (reprint it 0.2 mm tighter at home; meanwhile "
             "hold the box well off that pin's tip).",),
            Scene("pack", "drive", show=("drive", "blocks"),
                  marks=(r"decanter_riser_block\d", pins_re["L"]))),
        ModuleStep(
            "the cradle box on",
            "Lower the cradle box onto it, level, its knobs' side up (its other cradle link "
            "down), holding its weight until both pins are in their holes: "
            f"{first} (its paint X), into its hole first (look in from the side)"
            + (f", then turn the box about it and swing the drive side's {nudge} until pin "
               f"{second} lines up under its hole" if second else "") + ". " + hold,
            "box_on", (*((f"Pin {pins[0]} first, always: the box's centre of mass lies past it "
                         f"from pin {second}, so with {pins[0]} in its hole the box's weight "
                         f"lifts it off {second}'s tip (the other way round its weight would "
                         f"rest on {pins[0]}'s tip and push it back up its link).",)
                        if second else ()),
                       "Both clips in, from the side, all the way."),
            Scene("pack", "drive", show=("drive", "box", "blocks"),
                  moves=(("box", 0.0, 45.0, 0.0),), marks=(pin_x["L"],))),
        ModuleStep(
            "the passive panel set",
            f"Lay the passive side panel down on its outer plate on the folded towel, its pins "
            f"{names} held in its links by their clips, and set its links to the pack pose by "
            f"the paint marks: {pose.marks}.", "set",
            (f"A strip of masking tape across b3 and the inner bar holds them while the panel "
             f"turns over ({nudge} stays free: it is the one nudged into place).",),
            Scene("pack", "passive", show=("passive",), marks=(r"R\.b[35]",))),
        ModuleStep(
            "turned over",
            "Pull each pin's clip first, the panel still lying on its outer plate: each pin "
            "stays standing on its head in its link, its keeper on it. Then turn the panel "
            "over and hold it level, its outer plate up: each pin hangs in its link by its "
            "head, its keeper holding it.", "turn",
            scene=Scene("pack", "drive", show=("passive",), marks=(pins_re["R"],))),
        ModuleStep(
            "the passive panel on",
            "Lower the panel level onto the cradle link, holding its weight until both pins "
            f"are in their holes: {first} (its paint X), into its hole first (look in through "
            "the bay)" + (f"; if pin {second} doesn't line up, nudge {nudge} until it does"
                          if second else "") + ". " + hold, "lower", ("Peel the tape off.",),
            Scene("pack", "drive", show=three, moves=(("passive", 0.0, 45.0, 0.0),),
                  marks=(pin_x["R"], r"R\.b5"))),
        ModuleStep(
            "its clips in",
            "Both R-clips in from under the cradle link, through the pins' holes, all the way: "
            "a pin without its clip is held by its keeper's grip alone.", "clips_in",
            scene=Scene("pack", "drive", show=three, marks=clips["R"])),
        ModuleStep(
            "onto the board", on[0], "stand", on[1:],
            Scene("pack", show=("board", "drive", "box", "passive"),
                  moves=tuple((k, 0.0, 60.0, 0.0) for k in ("drive", "box", "passive")),
                  marks=(r"foot_bracket_\w+_knob\d?",))),
        ModuleStep(
            "the riser back",
            "The riser back in its locator, stacked for your decanter (the cradle page's "
            "Riser line: write it on the locator's underside with the paint pen), not the "
            "whole of it unless your decanter's stack is: its lifts side by side, its blocks "
            "on them, then its plates, as many of each as the stack says, each boss in the "
            "piece over it.", "riser",
            scene=Scene("pack", show=("*", "risers"), marks=(r"decanter_riser_\w+",))),
        ModuleStep(
            "the hand crank on",
            "The hand crank on, its thumb screw onto the input shaft's key.", "hand_crank_on",
            scene=Scene("pack", show=("*",), marks=(r"L\.handle\w*",))),
        ModuleStep(
            "snugged at level",
            f"Turn the handle to level ({level} turns from rest: the bottle would lie level) "
            "with the passive side's knobs loose, then snug them there: that side settles "
            "where the linkage puts it, and B's two remote centres agree best at level.",
            "snug", scene=Scene("level", show=("*",), marks=(r"foot_bracket_\w+R_knob\d?",))),
        ModuleStep(
            "the checks",
            "The first build's checks again, the bottle out: turn a whole pour and back; both "
            "cradle links swing freely and the handle turns evenly with a fingertip, under "
            f"{EMPTY_GRIP_N:g} N at the grip with no stiff spot (the luggage scale hooked on "
            f"the grip: under {EMPTY_GRIP_N / 9.81:.1f} kg).", "checks",
            ("Stiffer, the two sides fight each other: loosen the passive side's knobs and "
             "settle them again at level.",
             "Then the rod's paper check (the hand crank's step). All four clips in, the four "
             "knobs snug.",
             "Then the clutch's check on its stub and the worm's hold, with the heaviest bottle "
             "you'll pour (the last build step, \"Before the first pour\"): every set-up, "
             "before the first pour."),
            Scene("rest", show=("*",), marks=(r"L\.handle_grip",))),
    ]
    return breakdown, reassembly


PETG_PARTS = ("front_saddle", "front_saddle_vee", "front_saddle_gauge", "rear_saddle",
              "rear_saddle_vee", "rear_saddle_hub", "rear_saddle_knob", "rear_saddle_jack",
              "rear_saddle_nut", "rear_saddle_thrust", "base_stop_rail0", "base_stop_rail1",
              "base_stop_gate")
"""The printed parts that carry the bottle or hold it where it is (``meta["petg"]``,
``hardware.bom.part_filament``): printed solid in PETG, whose layers bond about as well as
PLA's (Prusament's datasheets: 18 and 17 MPa across them) and whose resin deflects at 64 °C
under 1.8 MPa, 70 under 0.46 (Eastman's Eastar 6763; Tg 80; PLA softens at 55): their
stresses, rated on solid sections (``cradle_report.saddles``; review round 2, N6 and m9),
reach 11 MPa for as long as a bottle rests, so the guide keeps the cradle under 50 °C
(design review round 3, mounting m5: "holds its shape to 68 °C" had been the heat a part
unloaded takes)."""
FRONT_MOVES = ("front_saddle_vee", "front_saddle_liner", "front_saddle_nut")
REAR_MOVES = ("rear_saddle_vee", "rear_saddle_liner", "rear_saddle_nut",
              "base_stop_rail0", "base_stop_rail1",
              "base_stop_rail0_liner", "base_stop_rail1_liner", *(f"base_stop_screw{i}"
                                                                   for i in range(4)))
STOP_MOVES = ("base_stop_gate", "base_stop_lock", "base_stop_knob", "base_stop_washer0",
              "base_stop_washer1", "base_stop_nut")
"""The parts a bottle's setting moves (:func:`at_setting`): up with the front V block, up
with the rear one, and up with the rear one and along the axis (the base stop's gate)."""


def at_setting(mech, size: BottleSize):
    """The assembled cradle ``mech`` (drawn set for :data:`SHOWN`) set for ``size``: its V
    blocks and base stop moved by :func:`offsets`, ``size``'s bottle in the saddles."""
    lip = np.asarray(mech.meta["cradle_lip"], dtype=float)
    tilt = float(mech.meta["cradle_tilt"])
    d = offsets(size)
    c, s = math.cos(tilt), math.sin(tilt)
    shift = {**dict.fromkeys(FRONT_MOVES, (0.0, d.front)),
             **dict.fromkeys(REAR_MOVES, (0.0, d.rear)),
             **dict.fromkeys(STOP_MOVES, (d.stop, d.rear))}
    bodies = []
    for b in mech.bodies:
        if b.part is not None and b.name == "bottle":
            y0, slope, x0 = seat(size)
            b = replace(b, part=_place(_bottle(size, y0, slope, x0), lip, tilt))
        elif b.part is not None and b.name in shift:
            dx, dy = shift[b.name]
            b = replace(b, part=moved(b.part, Location((c * dx - s * dy, s * dx + c * dy,
                                                         0.0))))
        bodies.append(b)
    return replace(mech, bodies=bodies, meta=dict(mech.meta, bottle=size.name))


def finger_gap(mech) -> float:
    """The least gap (mm) between the base stop (its rails, their liners, its gate and lock)
    and what is under it in the assembled cradle ``mech`` as drawn (its V blocks at their
    lowest): the base board's top, or the table where the board's window is
    (:data:`BOARD_WINDOW`, ``meta["board_window"]``). At rest, the lowest it comes over the
    swing, at least :data:`FINGER_GAP` (43 mm since round 2's window; 26 to the board)."""
    by = {b.name: b for b in mech.bodies if b.part is not None}
    top = by["base_plate"].placed_part().bounding_box().max.Y
    table = top - BASE_T - FOOT_PAD_H
    x0, x1, z0, z1, r = mech.meta.get("board_window") or (0.0, 0.0, 0.0, 0.0, 0.0)
    gap = math.inf
    for n, b in by.items():
        if not n.startswith("base_stop"):
            continue
        verts, _ = b.placed_part().tessellate(0.5, 0.5)
        p = np.array([[v.X, v.Y, v.Z] for v in verts])
        # over the window: inside its rectangle less its corners' rounding
        cx = np.clip(p[:, 0], x0 + r, x1 - r)
        cz = np.clip(p[:, 2], z0 + r, z1 - r)
        over = np.hypot(p[:, 0] - cx, p[:, 2] - cz) < r
        gap = min(gap, float(np.min(p[:, 1] - np.where(over, table, top))))
    return gap


def bottle_hits(mech, size: BottleSize = DESIGN) -> list[str]:
    """What meets what in the assembled cradle ``mech`` set for ``size``
    (:func:`at_setting`): its bottle and every part its setting moves, against every other
    part (``"a / b"``: none for a bottle it takes); by default the largest, the 6 L
    methuselah."""
    from spiderpig.construction.contract import clashes

    found = clashes(at_setting(mech, size), ("bottle", *FRONT_MOVES, *REAR_MOVES,
                                             *STOP_MOVES))
    return [f"{c['a']} / {c['b']}" for c in found]


def print_poses(names, tilt: float) -> dict[str, list[tuple[str, float]]]:
    """How a cradle's printed parts that don't lie in a layer go on the print bed
    (:func:`build.print_pose`: turns about the global axes, ``tilt`` the cradle's in the
    drawn pose, radians): the carriers, the V blocks (their post's bore and nut's upright;
    support under the rear one's flats and bosses), the base stop's rails (on their flat
    bottoms: the T-slot's lips then bend in their layers; review round 2, N6), the knobs
    and the brake washer as they stand (the rear carrier's tower: support under its arm),
    the rear knob's hub on its flange, the V blocks' liners on their two edges (their apex
    up, TPU's 45° faces), the rails' liners flat, the lip gauge on its side (its bar bends
    in its layers; support under the bar and the arm), the base stop's gate on its top (its
    top bar, ears and pointer flat on the bed, its plate narrowing up at 45°: its ears'
    roots and its pointer in its layers; design review round 3, mounting M3 and m2: on its
    face, the ears' roots were across them and the pointer a 2 x 6 mm column),
    the light's clip on its tube's foot (upright, round), the brackets on their L (its
    corner in every layer), the base board's thumb knobs on their tops, the feet as they
    stand, the hand crank's hub standing on its end (its tube up) and its arm, grip and cap
    on their outer faces (review round 1, m4); the riser's pieces upside down, their
    bosses on the bed (round 2)."""
    t = -math.degrees(tilt)
    out: dict[str, list[tuple[str, float]]] = {}
    carriers = {s.name for s in SADDLES}
    for n in names:
        if n.startswith("foot_bracket_") and n.endswith("_knob"):
            out[n] = [("X", -90.0)]          # on its top: the nut's pocket open upward
        elif n in carriers or n.endswith(("_knob", "_vee")) or n in (
                "led_clip", "rear_saddle_brake", "base_stop_rail0", "base_stop_rail1",
                "rear_saddle_jack", "rear_saddle_nut", "rear_saddle_thrust"):
            out[n] = [("Z", t), ("X", 90.0)]
        elif n == "rear_saddle_hub":
            out[n] = [("Z", t), ("X", -90.0)]
        elif n in ("base_stop_rail0_liner", "base_stop_rail1_liner"):
            # its sheet on the bed, its pegs up (simplification round 1, 2026-10-21: turned
            # the other way, 45°, it stood on its pegs, 19 mm² on the bed, the rest on
            # supports: 2:52 to print, not 1:38, BambuStudio 2.08)
            out[n] = [("Z", t), ("X", -135.0 if n == "base_stop_rail0_liner" else 135.0)]
        elif n.endswith("_liner"):
            out[n] = [("Z", t), ("X", -90.0)]
        elif n == "base_stop_gate":          # on its top: its ears' roots and its pointer
            out[n] = [("Z", t), ("X", -90.0)]   # in its layers (round 3, mounting M3, m2)
        elif n == "front_saddle_gauge":
            out[n] = [("Z", t)]
        elif n.startswith("foot_bracket_") and n.count("_") <= 2:   # its L on the bed: the
            out[n] = [("Y", 90.0)]                                   # corner in every layer
        elif n.startswith("foot_boot_"):
            out[n] = [("X", 90.0)]
        elif n.startswith("decanter_riser_"):      # upside down: its boss on the bed, its
            out[n] = [("X", -90.0)]                 # recess or cup open upward
        elif n in ("L.handle", "L.handle_tube"):   # its hub (or its tube's end) on the
            out[n] = [("Y", -90.0)]          # bed, the cone under the clutch's flange (the
            #                                  shaft along +x)
        elif n in ("L.handle_arm", "L.handle_grip", "L.handle_cap"):
            out[n] = [("Y", 90.0)]           # the arm's outer face on the bed
    return out
