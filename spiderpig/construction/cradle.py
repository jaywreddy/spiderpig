"""A decanting cradle: two side stacks, the cradle box between them, a stand and a bottle.

A cradle linkage (:mod:`linkages.decanters`, ``Linkage.product == "cradle"``) is planned
one side at a time like any mechanism. The bottle lies between two sides, the **bay**, so
:func:`assemble_cradle` makes the cradle from that one side:

* **The drive side** (``L.``, the side as planned; on your right as you stand behind the
  cradle facing the decanter) carries the hand crank (:mod:`construction.handcrank`).
* **The passive side** (``R.``) is the drive side's own parts mirrored, without the crank,
  the rod and the rod's pin (:func:`passive_side`): the cradle box drives it; its plates
  keep the drive's holes, so each plate pair is one part, flipped.
* **The cradle box**: four tie rods (10 mm steel standoffs made to length, MISUMI NETRF10)
  between the two cradle links, at the linkage's tie points (``decanters.TIES``), each
  screwed through its cradle link from outside. Two printed saddles ride on them, each a
  carrier on its two rods (an M3 set screw up from its underside onto each) and a V block
  that rides up and down in it on a thumbscrew (:class:`Saddle`): the front one under the
  neck, in a box guide; the rear one under the body, on a steel post and a coarse lead
  screw, the base stop's rails continuing its V behind it on their flanges. TPU liners,
  pegged in, line the Vs. At a 6 L methuselah's setting each V block sits on its tie rods.
* **The bottles** (:func:`setting`, :func:`seat`): a 750 ml to a 6 L, each with its mouth
  on the pivot and its axis level, set by their scales with the bottle out: the two knobs
  and the base stop. The **lip gauge** stands at the mouth, its pad on the rim's top (lifted
  to uncork, it falls back by itself): it checks where the mouth is and holds the bottle
  from sliding forward. The **base stop**'s gate rides the rear V block's flats and the
  rails behind it, clamped by hand behind the bottle's base: it holds the bottle from
  sliding back.
* **The light**: an AAA keychain light standing in a clip printed with the front saddle,
  its head under the shoulder (the sommelier's candle; its head twists it on).
* **The stand** (:data:`STANDS`, :class:`CradleStand`): each side's frame plates are the
  stand's members, not arms from O. The outer plate carries the hinge, the gearbox and the
  legs down to a rail. The inner plate holds the pillars and stiffens the side (A: a block
  round the hinge and a leg down the front, four columns; B: the bar O1-O2), so the tie
  rods pass it in the open.
* **The base board** (:func:`_base_plate`): both sides' rails bolted onto it with printed
  brackets (:func:`_bracket`: thumb knobs, no tool), each rail between two birch cleats
  glued to it (:func:`_cleats`), the decanter's lab jack in a locator in a notch from its
  front edge (:func:`_jack_locator`), printed TPU feet snapped into its corners; B's sits
  toward its drive side (:data:`BASE_Z`).
* **Display only** (``fab="display"``: drawn, never bought, cut or printed): the bottle (a
  6 L methuselah, :data:`SHOWN`: the V blocks drawn at its setting, their lowest) and a
  decanter under the mouth.

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
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from typing import cast

import numpy as np
from build123d import Axis as BAxis
from build123d import Box, Cylinder, Face, Location, Plane, Solid, Wire

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
from spiderpig.hardware.fasteners import Screw, screw
from spiderpig.mechanism import Body, Mechanism
from spiderpig.shapes import Cut, Shape3D, cut_holes, difference, disc, moved, pill, union
from spiderpig.stack import Claim, Disc, Layout, Placed, Unbuildable, body_class

LIP_HEIGHT = 550.0      # the bottle's mouth over the table (mm): at rest, 50° neck up, the
#                         base stop (set for a 6 L) hangs about 507 mm under it, 26 mm
#                         over the base board's top (17 mm up)
DECANTER_RISER = 135.0  # the drawn decanter's riser: the highest that keeps it clear of the
#                         cradle box over the whole swing (cradle_report.room: at rest, 50°
#                         neck up, the front saddle, the tie rods and the light hang across
#                         the axis from 385 mm over the table up)
DECANTER_PROFILE = ((0.0, 28.6), (6.3, 38.8), (21.7, 84.2), (30.0, 95.8), (40.7, 102.5),
                    (47.4, 100.8), (55.4, 91.2), (72.4, 52.4), (79.5, 41.3), (89.5, 32.3),
                    (108.5, 22.9), (133.5, 18.3), (165.9, 19.0), (189.9, 23.9), (225.0, 37.1))
"""The decanter drawn (display): a RIEDEL Ultra 2400/14, 750 ml, 225 mm tall, 205 across
(https://www.riedel.com/en-gb/riedel-ultra-decanter/240000014), its outline (height,
radius) traced from RIEDEL's drawing (the vessel study of 2026-10-10, ``DECANTER.md``)."""
RISER_W = 100.0         # the riser under it (display): a lab jack's 100 mm platform
BAY = 260.0             # between the two inner plates' bay faces: a 174 mm bottle, and a
#                         decanter up to 240 mm across stands wholly between the side stacks
TIE_D = 10.0            # the tie rods and columns: MISUMI NETRF10 (10 mm, M5 x 10 deep each end)
TIE_THREAD = 10.0
M5 = screw("bhcs", "5")
TIE_SCREW = 12.0        # M5 x 12 button heads through a 3.2-4.8 mm plate: 7.2-8.8 mm in the tie
TIE_HOLE = 5.5          # their clearance holes (ISO 273 medium)
M5_NUT = (8.0, 4.0)     # ISO 4032 M5 nut: across flats, height
LINER = 2.0             # the V blocks' TPU liners
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

    def profile(self) -> list[tuple[float, float]]:
        """(x along the axis from the mouth, radius) round the outline."""
        pts = [(0.0, self.r_neck), (self.neck, self.r_neck)]
        n = 8
        for i in range(1, n + 1):           # the shoulder: a quarter cosine
            s = i / n
            x = self.neck + s * (self.shoulder - self.neck)
            pts.append((x, self.r_neck + (self.r - self.r_neck) * (1 - math.cos(s * math.pi)) / 2))
        pts.append((self.length, self.r))
        return pts


STANDARD = BottleSize("standard", "750 ml Bordeaux", 300.0, 38.3, 15.0, 80.0, 115.0,
                      1.3, 191.0)
MAGNUM = BottleSize("magnum", "1.5 L Bordeaux magnum", 356.0, 49.5, 15.5, 85.0, 150.0,
                    2.6, 230.0)
JEROBOAM = BottleSize("jeroboam", "3 L Champagne jeroboam", 470.0, 70.5, 19.0, 105.0, 190.0,
                      5.9, 325.0)
IMPERIAL = BottleSize("imperial", "6 L Bordeaux impériale", 493.0, 82.3, 23.5, 115.0, 195.0,
                      9.1, 319.0)
METHUSELAH = BottleSize("methuselah", "6 L Champagne methuselah", 571.0, 87.0, 24.0, 128.0,
                        231.0, 11.1, 390.0)
SIZES = (STANDARD, MAGNUM, JEROBOAM, IMPERIAL, METHUSELAH)
"""What the cradle is built for (``docs/agentlib/DECANTER.md``, Requirements): a standard
750 ml Bordeaux (the Bordelaise moulds run about 295-310 x 74-77 mm; its neck to the
shoulder about 80 mm, the front V block's back end), a Bordeaux magnum (univerre: 356 x 99
mm, 1.04 kg empty), a Champagne jeroboam (Saverglass: 470 x 141 mm, 2.9 kg empty), and the
two 6 L shapes (2026-10-13, the user's "both"): a Bordeaux impériale (Saverglass
Bordelaise Tradition 600 cl, ref. 0714: 492.8 mm tall, 164.5 mm at the shoulder, 156.5 at
the base, 3.12 kg empty; univerre's NGBD06QFR 493.1 x 164.4, 2.95 kg) and a Champagne
methuselah, the largest and heaviest (Saverglass Standard 600 cl, ref. 1742: 571 mm tall,
174 mm at the shoulder, 170 at the base, 5.13 kg empty). Full mass: the glass plus 0.75,
1.5, 3 or 6 kg of wine (``cradle_report.WINE_KG``). The 6 L necks (radius, length to the
shoulder) are estimates scaled from the jeroboam's (no maker draws them): the front knob is
set to the neck in hand.

The centre of mass, full (``com_from_mouth``; design review round 1, 2026-10-14: it had been
42 % of the length from the base, too far forward), from a model of each bottle: a glass
shell over its :meth:`BottleSize.profile`, the neck's wall 1.5 x the body's and the base 3 x
thick under a punt (a cone 0.8 r across and 0.35 r deep), the wall's thickness solved so the
glass weighs the bottle's empty mass (2.5 g/cm³: 2.9 mm for the 750 ml to 6.3 mm for the
methuselah), and the wine (0.995 g/cm³) filled from the base, upright and sealed (the
headspace in the neck), to its volume. It puts a full bottle's centre 31-36 % of its length
up from the base: a 750 ml's about 11 cm up its 30 cm, the usual figure. Along the axis from
the mouth: 191, 230, 325, 319 and 390 mm (the magnum's and the impériale's wine fills the
whole of their drawn outline: there the wine's centre is the inside's). Weigh a bottle on a
knife edge to refine one."""


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

SADDLES = (
    # the neck's V block in a box guide in its carrier (its underside 8 mm under the 10 mm
    # tie rods: an M3 x 6 set screw up to each); it ends at a 750 ml's shoulder (80 mm)
    Saddle("front_saddle", ("Tf1", "Tf2"), (45.0, 85.0), -60.0, (48.3, 79.0), -41.9,
           _apex(DESIGN.r_neck), -10.0, 15.0),
    # the body's V block, under a 750 ml's body (its base at 300); the base stop's rails'
    # flanges continue its V behind it to 588, under a 6 L's whole body, so lying level or
    # tipped its weight centres on the support (cradle_report.balance); it rides a steel
    # post and a lead screw at its ends
    Saddle("rear_saddle", ("Tr1", "Tr2"), (255.0, 380.0), -149.0, (255.0, 380.0), -130.9,
           _apex(DESIGN.r), -55.0, 72.0),
)

STOP_GAP = 0.5          # the base stop's face behind the bottle's base
STOP_RANGE = (285.0, 576.0)     # where the base stop's face can go (cradle x): inside the
#                                 rear V block (the 750 ml's at 300.5) to short of the
#                                 rails' end (the 6 L's at 571.5)


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


# --- the stand -------------------------------------------------------------------------

@dataclass(frozen=True)
class Stand:
    """A cradle's frame plates as a truss (mm, relative to the bottle's mouth at rest).

    ``nodes``: the free points; a member may also name O or a pillar. ``outer``/``inner``:
    each plate's members (pills of ``member_r``). ``columns``: a 10 mm steel standoff between
    a side's two plates there. ``feet``: where the outer plates stand on the table (a TPU
    boot each). ``base_ties``: a tie rod across the whole cradle, outer plate to outer plate
    through both stacks (clear of the decanter: never in front). ``bay_ties``: a tie rod
    across the bay between the two inner plates (where nothing of the cradle passes)."""

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
    column_d: float = TIE_D     # the columns' standoffs: 10 mm (NETRF10, M5) or 12 (NETRF12,
    #                             M6: :func:`column_parts`)
    base: tuple[float, float] = (-200.0, 482.0)    # the base board's front and rear edges
    fillet_r: float = 20.0      # the plates' inside corners rounded this much (their joints
#                                 read as one shape, not crossed bars)


BASE_SHEET = "birch_9mm"        # the base board, both sides' outer plates bolted onto it:
#                                 Baltic birch plywood, routed (2026-10-13, the user's)
BASE_T = 9.0                    # its thickness
WOOD_GLUE = "wood_glue"         # its cleats and the jack locator's layers glued (Titebond II)
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
JACK_BASE = 100.0               # the lab jack under the decanter: a 100 x 100 mm base and
#                                 platform (the common 4 x 4 in stainless jack)
JACK_PLAY = 0.5                 # its pocket's play each side
LOCATOR = (-70.0, 1.0, 2)       # the jack locator in the notch: its front edge (x from the
#                                 mouth), its play to the notch each side, its layers of the
#                                 board's birch (glued: 18 mm, so its top layer meets the
#                                 board's whole edge; the board stands 8 mm up on its feet)
NOTCH_BAY = (-45.0, 15.0)       # the notch's two side bays (x from the mouth, radius) the
#                                 locator's ears drop into: it can't slide out forward
DOG_BONE_R = 1.6                # a routed square corner's relief: the router's 1/8 in bit
#                                 (SendCutSend's least hole in 9 mm birch, 3.175 mm)
CLEAT = (26.0, 0.15, 4.0)       # a rail's two birch cleats, glued to the board either side of
#                                 it: their width across (SendCutSend's least birch part is
#                                 25.4 mm), the play each side of the rail, the gap to a bracket
_FOOT_Y = BASE_TOP + 12.5       # a foot's centre: its screw's boss (2 t) on the base board
BRACKET = (36.0, 30.0, 5.0)     # a foot's printed bracket: its length, legs' height, wall
"""A stand foot is a printed L-bracket outside each outer plate, an M5 button head through
it and the plate (its nut in the layer over the plate) and a carriage bolt up through the
base board and its foot's oversize hole, an 18-8 fender washer and a printed thumb knob (an
18-8 nut pressed in it) on it (:func:`_bracket`)."""

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
        nodes={"Ff": (-150.0, _FOOT_Y), "Fm": (140.0, _FOOT_Y), "Fr": (380.0, _FOOT_Y),
               "C1": (-24.0, 20.0), "C2": (-24.0, -22.0),
               # on the front leg G-Ff, the inner plate's leg over it down to about 200 mm
               # over the table: its lowest column D3 halves the outer leg's free length (design
               # review round 1, 2026-10-14: the leg takes 2 kN jammed, and from D2 to the
               # foot it buckled out of the plate at 0.6)
               **{n: (-150.0 * f, (_FOOT_Y - 0.0) * f) for n, f in (("D1", 0.175),
                                                                     ("D2", 0.36),
                                                                     ("D3", 0.68))}},
        outer=(("G", "Ff"), ("G", "O"), ("Ff", "O"), ("O", "Fm"), ("O", "Fr"),
               ("Ff", "Fm"), ("Fm", "Fr"), ("G", "C1"), ("G", "C2"), ("C1", "C2"),
               ("C2", "D1")),
        inner=(("G", "C1"), ("G", "C2"), ("C1", "C2"), ("C2", "D1"), ("D1", "D2"),
               ("G", "D1"), ("D2", "D3")),
        columns=("C1", "C2", "D1", "D2", "D3"),
        feet=("Ff", "Fm", "Fr"),
        # the strut from the hinge back to the gearbox, 48 mm wide: the gearbox's node O
        # leans on it across the plate (the legs from O buckled sideways at walking SF 3.0)
        wide={("G", "O"): 24.0},
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
        nodes={"Ff": (-110.0, _FOOT_Y), "Fr": (386.0, _FOOT_Y),
               # on the front leg O2-Ff, 60 and 150 mm down it from O2 (O2 is (-2.7, -307))
               **{n: (-2.7 + (-110.0 + 2.7) * f, -307.0 + (_FOOT_Y + 307.0) * f)
                  for n, f in (("D1", 0.25), ("D2", 0.6))}},
        outer=(("O1", "O2"), ("O2", "Ff"), ("O2", "O"), ("O1", "O"), ("O", "Fr"),
               ("Ff", "Fr"), ("O2", "Fr")),
        inner=(("O1", "O2"), ("O2", "D1"), ("O1", "D1"), ("D1", "D2")),
        columns=("D1", "D2"),
        feet=("Ff", "Fr"),
        wide={("O2", "O"): 32.0},
        column_d=12.0,
        # (round 1, 2026-10-14: with the corrected centre of mass and the board 40 mm
        # toward the drive side, BASE_Z, the rear edge binds: 682 mm is the least board
        # that keeps a full 6 L past 25° over it since round 1's drive, whose parts took the
        # 680 mm board's 25.0° to 24.9°: 25.1° at 682)
        base=(-160.0, 522.0),
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
        r = column_parts(stand).d / 2 + 0.3
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
                for k in range(1, L.top):
                    out.append(Placed(k, Disc(name, r), self.name, "stand column"))
                    out.append(Placed(k, Disc(name, r), self.name, "stand column", gap=True))
                out.append(Placed(0, Disc(name, r), self.name, "stand column", gap=True))
            return out

        return [Claim("stand", frozenset(), make)]

    def realize(self, build: Build, done: Realized) -> Realized:
        ctx = build.ctx
        lk = ctx.config.lk
        stand = stand_of(lk)
        out = Realized()
        topo = build.plan.topo

        def xy(n):
            return tuple(_node_xy(build.xy, lk, topo, stand, n))

        for key, members in ((FRAME_OUTER, stand.outer), (FRAME_INNER, stand.inner)):
            for a, b in members:
                out.pad(key, xy(a), xy(b), stand.wide.get((a, b), stand.member_r))
        frame = topo.frame_bodies[0]
        hole = TIE_HOLE
        z0, z1 = build.z(0)[1], build.z(build.top)[0]      # between the plates
        for n in (*stand.base_ties, *stand.feet):
            out.cut(FRAME_OUTER, Cut(xy(n), hole))
        for n in stand.bay_ties:
            out.cut(FRAME_INNER, Cut(xy(n), hole))
        col = column_parts(stand)
        for n in stand.columns:
            p = xy(n)
            out.cut(FRAME_OUTER, Cut(p, col.hole))
            out.cut(FRAME_INNER, Cut(p, col.hole))
            out.bodies.append(hardware(f"stand_column_{n}", _tie(p, z0, z1, col.d, col.thread),
                                       frame, fab="purchased", bom_key=col.key(z1 - z0),
                                       color=TIE_COLOR))
            for side, z in ((-1, build.z(0)[0]), (1, build.z(build.top)[1])):
                out.bodies.append(hardware(f"stand_column_{n}_screw{'io'[side > 0]}",
                                           _screw(p, z, side, col.screw_len, col.screw),
                                           frame, fab="purchased",
                                           bom_key=col.screw.key(col.screw_len),
                                           color=TIE_COLOR))
        out.notes["stand"] = {"columns": list(stand.columns), "feet": list(stand.feet),
                              "base_ties": list(stand.base_ties),
                              "bay_ties": list(stand.bay_ties), "column_mm": round(z1 - z0, 2)}
        return out

    def assembly(self, view) -> list:
        """The stand's columns (:mod:`construction.assembly`): each screwed to the bare
        outer plate from outside first, as the pillars are; its inner screw from the bay
        once the inner plate is on."""
        from spiderpig.construction.assembly import JOIN, STACK, Op, whole

        columns = view.named(r"stand_column_[A-Za-z0-9]+")
        outer = view.named(r"stand_column_\w+_screwi")
        inner = view.named(r"stand_column_\w+_screwo")
        ops = []
        if columns:
            ops.append(Op(STACK, (-1, 1), whole(*columns, *outer), "",
                          "Screw the stand's columns to the outer plate, a button head from "
                          "outside into each (threadlocker).", "columns"))
        if inner:
            ops.append(Op(JOIN, (2,), whole(*inner), "",
                          "The columns' inner button heads from the bay side (threadlocker).",
                          "column_screws"))
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
        head = M5.head_d / 2 + 0.3

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
                                  gap=True, height=M5.head_h + 0.25, toward=-1))
            return out

        return [Claim("tie rods", frozenset((link,)), make)]

    def realize(self, build: Build, done: Realized) -> Realized:
        lk = build.ctx.config.lk
        out = Realized()
        topo = build.plan.topo
        link = next(n for n in topo.links if body_class(n) == cradle_link(lk))
        ties = tie_points(topo, lk)
        for name in ties.values():
            out.cut(link, Cut(tuple(build.xy(name)), TIE_HOLE))
        _ties_clear_the_inner_plate(build, done, ties)
        return out

    def assembly(self, view) -> list:
        """The cradle link (:mod:`construction.assembly`): the drive side's with its tie
        rods screwed on, on the bench, before it goes on its pins (their screws' heads end
        up under it); the passive side's goes on the tie rods' other ends on the cradle
        (``CRADLE_ORDER`` moves it), not in its stack. Placed here, ahead of the links'
        hook."""
        from spiderpig.construction.assembly import STACK, Op, whole

        link = cradle_link(view.ctx.config.lk) if view.ctx is not None else None
        if link is None or link not in view.z:
            return []
        k = view.slot(view.z[link][0])
        if view.side == "R":
            return [Op(STACK, (k, 1), whole(link), "", "", "cradle_link")]
        lk = view.ctx.config.lk
        clipped = sorted(j for j in lk.links[link][0] if f"pin_{j}_clip" in view.z)
        keep = (f" Before it goes on, pin{'s' if len(clipped) > 1 else ''} "
                f"{' and '.join(clipped)}' TPU keeper (the ring printed in TPU over the link "
                "the pin stands in) pushed down onto that link: firm, it grips the pin, and it "
                "is what holds the pin in its link when the side lies down with its clip out "
                "(the trip's breakdown; both sides' keepers)." if clipped else "")
        return [Op(STACK, (k, 0, "ties"), whole(link), "Cradle link and tie rods",
                   "The four tie rods onto the cradle link, an M5 button head "
                   "through the link from its leg side into each (threadlocker); the rods "
                   "stand up from its bay side. It goes onto the stack rods up, its bores "
                   "down over the steel and the pins it rides." + keep, "cradle_link",
                   sub=True)]

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
                keep_out = _tie_paths(build) + _tip_paths(build)
            part = plate_part(pads, cuts, z0, z1, web, fillet, keep_out)
            out.bodies.append(hardware(name, part, frame, fab="laser", color=FRAME_COLOR,
                                       sheet=build.ctx.sheet("frame")))
        return out


def plate_part(pads, cuts, z0: float, z1: float, web: float, fillet: float = 0.0,
               keep_out=()) -> Shape3D:
    """A cradle frame plate: its members (pills), a boss round every round hole, the inside
    corners rounded ``fillet`` (a closing: the outline grown and shrunk back, so the joints
    read as one shape), clear of ``keep_out`` (solids: what passes the plate's layer), less
    the holes. Memoized."""
    memo = (tuple(pads), tuple(cuts), z0, z1, web, fillet, len(keep_out))
    part = _PLATE_MEMO.get(memo)
    if part is not None:
        return part
    parts = [pill(a, b, r, z0, z1) for a, b, r in pads]
    parts += [disc(c.xy, c.d / 2 + web, z0, z1) for c in cuts if isinstance(c, Cut) and web > 0]
    part = union(parts)
    if fillet > 0:
        part = _closed(part, z0, z1, fillet)
    if keep_out:
        part = difference(part, union(list(keep_out)))
    part = cut_holes(part, cuts, z0, z1)
    if len(_PLATE_MEMO) > 16:
        _PLATE_MEMO.clear()
    _PLATE_MEMO[memo] = part
    return part


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


def _tie_paths(build: Build) -> list[Shape3D]:
    """What the inner plate's layer must stay clear of: each tie rod's path over the turn
    (its radius and the plan's margin), as solids through the layer."""
    ties = tie_points(build.plan.topo, build.ctx.config.lk)
    z0, z1 = build.z(build.top)
    r = TIE_D / 2 + build.plan.spec.margin
    out = []
    for name in ties.values():
        path = np.asarray(build.plan.topo.geometry.points[name], float).reshape(-1, 2)
        step = max(1, len(path) // 48)
        pts = [*path[::step], path[0]]
        out += [pill(tuple(a), tuple(b), r, z0 - 1.0, z1 + 1.0)
                for a, b in itertools.pairwise(pts)]
    return out


def _tips(build: Build) -> list[Placed]:
    """What else passes the inner plate's layer in its open: a pin's end past its clip (a
    seat the pin claims there, ``tips``: :mod:`construction.pivots.clevis`)."""
    return [p for p in build.plan.shapes(layer=build.top, gaps=False)
            if p.seat and p.label.endswith(" tip") and isinstance(p.shape, Disc)]


def _tip_paths(build: Build) -> list[Shape3D]:
    """The inner plate's keep-outs for :func:`_tips`: each one's path over the turn (its
    radius and the plan's margin), as solids through the layer."""
    z0, z1 = build.z(build.top)
    out = []
    for p in _tips(build):
        assert isinstance(p.shape, Disc)
        path = np.asarray(build.plan.topo.geometry.points[p.shape.at], float).reshape(-1, 2)
        step = max(1, len(path) // 48)
        pts = [*path[::step], path[0]]
        r = p.shape.r + build.plan.spec.margin
        out += [pill(tuple(a), tuple(b), r, z0 - 1.0, z1 + 1.0)
                for a, b in itertools.pairwise(pts)]
    return out


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
    """The catalog key of a tie rod's or column's NETRF10 standoff ``length`` mm long."""
    from spiderpig.hardware.cradle_catalog import netrf10

    return netrf10(round(length, 1))


def _standoff12(length: float) -> str:
    """The catalog key of a 12 mm NETRF12 standoff ``length`` mm long (the rear post)."""
    from spiderpig.hardware.cradle_catalog import netrf12

    return netrf12(round(length, 1))


def _tie(xy, z0: float, z1: float, d: float = TIE_D, thread: float = TIE_THREAD) -> Shape3D:
    """A steel standoff (``d``, 10 mm: a NETRF10) from z0 to z1, its ends tapped (modelled
    as holes the screw's size, ``thread`` deep: M5 in a 10 mm, M6 in a 12 mm)."""
    tap = 2.5 if d <= TIE_D else 3.0
    return difference(disc(xy, d / 2, z0, z1),
                      *[disc(xy, tap, a, b)
                        for a, b in ((z0 - 1, z0 + thread), (z1 - thread, z1 + 1))])


def _screw(xy, face: float, side: int, length: float = TIE_SCREW, s=M5) -> Shape3D:
    """A button head (``s``: M5) bearing on ``face``, its head on the ``side`` (+1 over it,
    -1 under it) and its shank into the standoff beyond."""
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

    def key(self, length: float) -> str:
        return _standoff12(length) if self.d > TIE_D else _standoff(length)


def column_parts(stand: Stand) -> ColumnParts:
    """:class:`ColumnParts` of ``stand``'s columns: a NETRF10 on M5 x 12 button heads in
    5.5 mm holes, or a NETRF12 (12 mm, M6 x 12 deep each end) on M6 x 12 in 6.6 mm."""
    if stand.column_d > TIE_D:
        return ColumnParts(12.0, 12.0, M6, 12.0, 6.6)
    return ColumnParts(TIE_D, TIE_THREAD, M5, TIE_SCREW, TIE_HOLE)


def _place(part, lip, tilt: float, z: float = 0.0):
    """A part drawn in the cradle's frame (the mouth at the origin, the axis along +x)
    turned by ``tilt`` about z and put at ``lip`` (world XY, z)."""
    loc = Location((float(lip[0]), float(lip[1]), z), (0, 0, math.degrees(tilt)))
    return moved(part, loc)


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


SET_SCREW = 6.0            # the M3 cup-point set screw locking a carrier on each tie
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
FRONT_TOP = 6.0               # the carrier's walls' tops
FRONT_FLOOR = -43.0           # its floor's top (the tie rods' tops, at -42, stand proud)
FRONT_TAB = (56.0, 70.0, -28.0)     # the V block's tab into the +z wall: x span, its top
FRONT_JACK = (63.0, 40.0)     # the thumbscrew (x, z): an M6 x 60 hex screw, fully threaded
FRONT_NUT_Y = -40.0           # its nut's underside in the tab, the block at its lowest
FRONT_HEAD_Y = 9.5            # the screw's head's underside
FRONT_SCREW = 60.0
FRONT_DRAIN = (18.0, 26.0, 3.0)     # the drains through the back wall at the floor: |z|, height
M6_HEX = (10.0, 5.0, 4.0)     # DIN 934 M6 nut and its tap bolt's head: across flats, nut, head
M5_HEX = (8.0, 3.5, 5.0)      # an M5 tap bolt's head and a nylon-insert nut: flats, head, nut
FRONT_NUT_KEY, FRONT_JACK_KEY = "m6_nut", "m6_tap_bolt_60"     # (hardware.cradle_catalog)
KNOB_R = (11.0, 12.0)         # the printed knobs' radii (front, rear)
GAUGE_PIVOT = (30.0, -41.0)   # the lip gauge's hinge axis along x (cradle y, z)
GAUGE_POST = (55.8, 74.2, 36.5)     # its post on the carrier's -z wall: x span, top (its down
#                                     stop); 18.4 mm along the pin (review round 1, M3)
GAUGE_KNUCKLES = ((49.5, 55.5), (74.5, 80.5))   # its knuckles either side of the post
GAUGE_PAD = (11.0, 7.0)       # its pad's bottom (over a cork's bore) and half-width
GAUGE_FRONT = (-14.0, -0.2)   # its pad and arm along x: the back face in the mouth's plane
GAUGE_BAR = (20.0, 38.0, -36.0, -27.0)  # its arm in front of the mouth and its bar beside
#                                         the neck (a 6 L's at |z| 24): y0, y1; the bar's z
GAUGE_BRIDGE = (36.9, 50.0, -48.0, -30.0)   # over the post, knuckle to knuckle: y0, y1, z
GAUGE_FLIP = 70.0             # it opens this far, its lug on the wall's top: its centre of
#                               mass stays inboard of the hinge, so it falls back shut
GAUGE_LUG = (2.5, -100.0)     # the lug's round end: its radius, and its direction from the
#                               hinge where it lands (10° short of its arc's lowest: opening
#                               further drives it into the wall's top)
GAUGE_PIN = 40.0              # M5 x 40 tap bolt: front knuckle, post, back knuckle, lock nut
GAUGE_PIN_KEY, GAUGE_NUT_KEY = "m5_tap_bolt_40", "m5_nylon_nut"


def _front_saddle(s: Saddle, ties: Mapping[str, tuple[float, float]]
                  ) -> tuple[Shape3D, list[Shape3D]]:
    """The front saddle's carrier (cradle frame): a floor on the two tie rods (an M3 set
    screw up from its underside onto each, in a printed pilot), walls round its V block
    (the block's guide; the front and back ones notched a millimetre under the lowest
    liner; two drains through the back one at the floor, its low end at rest), a pocket in
    the +z wall for the block's tab and the thumbscrew's foot, the lip gauge's post on the
    -z wall, the scale's grooves on the +z wall's inside (the block's top edge reads them:
    three named in front of the pocket, :func:`front_named`, a dot each per place from the
    top; the neck's diameter every 4 mm behind it). (The carrier, its set screws.)"""
    x0, x1 = s.x
    h = FRONT_HALF + CLEAR
    v0, v1 = s.vee[0] - CLEAR, s.vee[1] + CLEAR
    floor = _box(x0, x1, s.y0, FRONT_FLOOR, *FRONT_W)
    walls = [_box(x0, x1, FRONT_FLOOR - 1.0, FRONT_TOP, FRONT_W[0], -h),
             _box(x0, x1, FRONT_FLOOR - 1.0, FRONT_TOP, h, FRONT_W[1])]
    notch = s.apex + LINER * math.sqrt(2) - 1.0
    zy = [(-h, FRONT_FLOOR - 1.0), (h, FRONT_FLOOR - 1.0), (h, notch + h), (0.0, notch),
          (-h, notch + h)]
    walls += [_prism_x(zy, x0, v0), _prism_x(zy, v1, x1)]
    yp, zp = GAUGE_PIVOT
    px0, px1, ptop = GAUGE_POST
    post = [_box(px0, px1, FRONT_TOP - 1.0, yp, FRONT_W[0], -h),      # (round over the pin:
            _rod_x(yp, zp, ptop - yp, px0, px1)]                    # the bridge swings past)
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
        if under - SET_SCREW - 0.05 < s.y0:
            raise ValueError(f"{s.name}: an M3 x {SET_SCREW:g} set screw doesn't fit under "
                             f"tie {n} ({under - s.y0:.1f} mm)")
        cuts.append(_rod_y(x, 0.0, SET_PILOT / 2, s.y0 - 2.0, under))
        screws.append(_rod_y(x, 0.0, 1.25, under - 0.05 - SET_SCREW, under - 0.05))
    return difference(part, *cuts), screws


FRONT_PEGS = ((63.65, -24.0),)      # the front liner's pegs (x, the V face's y), each side


def _front_vee(s: Saddle) -> tuple[Shape3D, Shape3D, Shape3D]:
    """The front V block at its lowest (cradle frame): the neck's V, sitting on the tie
    rods, its tab into the carrier's +z wall holding the thumbscrew's M6 nut (in a hex
    pocket from below: the block bears down on it), its TPU liner pegged into the V. (The
    block, its liner, the nut.)"""
    x0, x1 = s.vee
    xj, zj = FRONT_JACK
    block = union([_box(x0, x1, s.bottom, s.top, -FRONT_HALF, FRONT_HALF),
                   _box(FRONT_TAB[0], FRONT_TAB[1], s.bottom, FRONT_TAB[2], FRONT_HALF - 0.5,
                        zj + M6_HEX[0] / 2 + 1.0)])
    block = difference(block, _hex_y(xj, zj, M6_HEX[0] + CLEAR, s.bottom - 1.0,
                                     FRONT_NUT_Y + M6_HEX[1] + 0.2),
                       _rod_y(xj, zj, 3.3, s.bottom - 1.0, FRONT_TAB[2] + 1.0))
    vee, liner = _vee_block(s, block, FRONT_PEGS)
    nut = difference(_hex_y(xj, zj, M6_HEX[0], FRONT_NUT_Y, FRONT_NUT_Y + M6_HEX[1]),
                     _rod_y(xj, zj, 3.0, FRONT_NUT_Y - 1.0, FRONT_NUT_Y + M6_HEX[1] + 1.0))
    return vee, liner, nut


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


def _gauge() -> tuple[Shape3D, Shape3D, Shape3D]:
    """The lip gauge, down (cradle frame): two knuckles on a pin along the axis either side
    of the front carrier's post (an M5 tap bolt through them and the post, a nylon-insert
    nut behind, snug: it swings under its own weight), a bridge over the post joining them
    (resting on the post's top: its down stop), a bar forward beside the neck (clear of a
    6 L's as it's laid in) to an arm across in front of the mouth and its pad, their back
    face in the pivot's plane: the top of the mouth's rim bears on it, above the cork's
    bore. A bottle pushing it forward bears its back knuckle on the post's back face; the
    pin takes the turn of a push off its line. A lug on the back knuckle lands on the
    carrier's wall when it's flipped up :data:`GAUGE_FLIP` (to uncork), short of where its
    centre of mass would pass over the hinge: let go, it falls back onto the mouth. (The
    gauge, its pin, the pin's nut.)"""
    yp, zp = GAUGE_PIVOT
    (f0, f1), (b0, b1) = GAUGE_KNUCKLES
    y0, y1, bz0, bz1 = GAUGE_BAR
    by0, by1, rz0, rz1 = GAUGE_BRIDGE
    ax0, ax1 = GAUGE_FRONT
    bottom, half = GAUGE_PAD
    r = 6.5
    lug_end, land = GAUGE_LUG
    lug_r = (yp - FRONT_TOP - lug_end) / -math.sin(math.radians(land))
    a = math.radians(land - GAUGE_FLIP)       # the lug's direction from the hinge, down
    lug = union([_rod_x(yp + lug_r * math.sin(a), zp + lug_r * math.cos(a), lug_end, b0, b1),
                 _box(b0, b1, -lug_end, lug_end, 0.0, lug_r).rotate(
                     BAxis.X, -math.degrees(a)).moved(Location((0.0, yp, zp)))])
    gauge = union([_rod_x(yp, zp, r, f0, f1), _rod_x(yp, zp, r, b0, b1),
                   _box(f0, f1, y0, by1, zp - r, bz1),                   # front knuckle
                   _box(b0, b1, yp, by1, zp - r, zp + r),                # back knuckle
                   _box(f0, b1, by0, by1, rz0, rz1),                     # bridge
                   _box(ax0, f0 + 0.01, y0, y1, bz0, bz1),               # bar
                   _box(ax0, ax1, y0, y1, bz0, half),                    # arm
                   _box(ax0, ax1, bottom, y0 + 0.01, -half, half),       # pad
                   lug])
    gauge = difference(gauge, _rod_x(yp, zp, TIE_HOLE / 2, f0 - 1.0, b1 + 1.0))
    af, head_h, nut_h = M5_HEX
    x = f0 + GAUGE_PIN
    pin = union([_rod_x(yp, zp, af / 2, f0 - head_h, f0), _rod_x(yp, zp, 2.4, f0, x)])
    nut = difference(_rod_x(yp, zp, af / 2, b1, b1 + nut_h), _rod_x(yp, zp, 2.5, b1 - 1.0,
                                                                     b1 + nut_h + 1.0))
    if x < b1 + nut_h + 0.8:
        raise ValueError("the lip gauge's pin doesn't reach through its lock nut")
    return gauge, pin, nut


def gauge_return() -> dict:
    """The lip gauge's balance about its hinge: its centre of mass's angle over the hinge
    (degrees from the horizontal, inboard positive) down and at its stop
    (:data:`GAUGE_FLIP`): under 90° at the stop, it falls back shut; and the lug's gap to
    the wall's top just short of the stop (none at it)."""
    gauge, _, _ = _gauge()
    yp, zp = GAUGE_PIVOT
    c = gauge.center()
    down = math.degrees(math.atan2(c.Y - yp, c.Z - zp))
    lug_end, land = GAUGE_LUG
    lug_r = (yp - FRONT_TOP - lug_end) / -math.sin(math.radians(land))
    low = yp + lug_r * math.sin(math.radians(land)) - lug_end      # the lug's at the stop
    return {"com_deg_down": round(down, 1), "com_deg_open": round(down + GAUGE_FLIP, 1),
            "com_mm": round(math.hypot(c.Y - yp, c.Z - zp), 1),
            "falls_back": down + GAUGE_FLIP < 85.0,
            "stop_on_wall_mm": round(low - FRONT_TOP, 2)}


# --- the rear saddle: a V block on a steel post and a coarse lead screw ------------------

REAR_POST = (317.5, -103.0)   # the post (-z): (x, z)
REAR_LEAD = (317.5, 110.0)    # the lead screw (+z): (x, z), its knob outboard of a 6 L (|z|
#                               87) by 11 mm as it's laid in (review round 1, M2)
REAR_FOOT_Z = (88.5, 117.5, 126.0)  # the carrier's side blocks across: inner faces, the -z
#                                     one's outer, the +z one's (the lead screw's collar)
REAR_FOOT_TOP = -121.0        # their tops
REAR_BOSS = (12.0, -120.5, -57.0)   # the V block's boss round the post: radius, bottom, top
#                                     (at its lowest)
LEAD_BOSS = (14.0, -120.5, -88.0)   # its boss round the lead screw, low beside the flats: the
#                                     knob over it stays under the axis's height
LEAD_WEB = (7.0, 88.0, 96.0, 100.0, -79.0)  # that boss's web to the flats: half-length,
#                                             |z| from, to the flats' edge, to the boss, the
#                                             outer part's top (under the knob at full rise)
REAR_FLANGE = (-67.0, -55.0, 96.0)  # its flats along its top: bottom, top, half-width
VEE_T = 9.0                   # its V shell, square to the V
LEAD = 8.0                    # the lead screw: Tr8 x 8 (2 mm pitch, 4 starts), 8 mm a turn
LEAD_SCREW = (8.0, 150.0, 5.5, -146.0)   # its diameter, length, root, foot (cradle y)
LEAD_NUT = (12.0, 16.0)       # its bronze barrel nut in the boss from below: OD, length
COLLAR = (21.0, 9.0)          # its clamping collar in the carrier's pocket: OD, length
BRAKE_T = 1.5                 # the TPU brake washer under the collar
TOWER = (333.0, 351.0, 101.0, 119.0)    # the carrier's tower beside the screw: x, z spans
ARM = (303.0, -15.0, -7.0)    # its arm over the screw's top, a bushing round it: front x,
#                               bottom, top (over the boss at full rise)
REAR_KNOB = (-6.5, 7.5)       # the knob on the screw's top: bottom, top (cradle y)
LEAD_KEYS = ("lead_screw_8x8_150", "lead_nut_8x8_barrel", "lead_collar_8x8")
"""(hardware.cradle_catalog: ServoCity 3501, 3500 and 3504 series)"""
POST = (130.0, -141.0)        # the post: a NETRF12 standoff (12 mm, M6 ends), its length
#                               and foot (12 mm: it takes the roll of a load hung at the screw)
POST_D = 12.0
M6 = screw("bhcs", "6")
POST_SCREW = 12.0             # M6 x 12 up into it from under the carrier, 6.7 mm in it
SEAT_R, SEAT_DEPTH = TIE_D / 2 + 0.15, 2.0  # the rear V block's seats on the tie rods
GUSSET = (45.0, -90.0)        # under its flats beside each boss: along x, its bottom
GROOVE = (77.0, 83.0, 3.0)    # the base stop's guide grooves along the flats and the rails:
#                               |z| span, depth
TSLOT = (293.0, 5.5, 5.0, (16.0, 1.6), 8.4, -74.0, -78.0)
"""The base stop's T-slot under the +z groove (the flats from its first x, then the +z rail):
its first x, the neck's width and the lips' thickness under the groove, the washer's slot
(width, height), the nut's width (the M5 nut can't turn in it) down to the floor, the floor,
the keel's bottom under it (the rails' bars' too)."""
REAR_PEGS = ((280.0, -95.0), (355.0, -95.0))    # the rear liner's pegs (x, the face's y)
STOP_MARKS = (87.0, 89.5, 92.0)     # the base stop's scale across the +z flat and rail's top:
#                                     a named groove's inner end, a graded one's, the outer
STOP_EVERY = 10.0             # its graded grooves, every 10 mm of a bottle's length


def _stop_scale(x0: float, x1: float, top: float) -> list[Shape3D]:
    """The base stop's scale (cradle frame) on the +z flat or rail's top (``top``, the V
    block at its lowest) from ``x0`` to ``x1``: a groove across at each listed bottle's
    setting (:func:`setting`'s ``stop``: the gate's face goes there), a dot each per place
    from the shortest in front of it, and one each :data:`STOP_EVERY` mm of a bottle's
    length (another bottle: its length plus :data:`STOP_GAP`)."""
    zn, zg, zo = STOP_MARKS
    named = sorted(z.length + STOP_GAP for z in SIZES)
    k0 = math.ceil((STOP_RANGE[0] - STOP_GAP) / STOP_EVERY)
    graded = [STOP_EVERY * k + STOP_GAP for k in range(k0, int(STOP_RANGE[1] // STOP_EVERY) + 1)]
    graded = [x for x in graded if all(abs(x - n) > 2.0 for n in named)]
    w = 0.3
    out = [_box(x - w, x + w, top - 0.6, top + 0.01, zn if x in named else zg, zo + 0.01)
           for x in [*named, *graded] if x0 + 1.0 <= x <= x1 - 1.0]
    for i, x in enumerate(named):
        for k in range(i + 1):
            xd = x - 1.5 - DOT_PITCH * k
            if x0 + 1.0 <= xd <= x1 - 1.0:
                out.append(_rod_y(xd, (GROOVE[1] + zn) / 2, DOT[0], top - DOT[1], top + 0.01))
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


def _rear_saddle(s: Saddle, ties: Mapping[str, tuple[float, float]]
                 ) -> tuple[Shape3D, list[Shape3D]]:
    """The rear saddle's carrier (cradle frame): two side blocks on the two tie rods, joined
    under the V block by a bridge; the -z one holds the steel post (in a socket, an M6
    button head up into it from below, its head sunk), the +z one the lead screw (a pocket
    for its clamping collar on a TPU brake washer, open outboard to reach the collar's
    screw, and a blind bore for its foot), and a tower beside it up to an arm over the
    screw's top: a bushing round it (a knock at the knob goes into the tower, not the
    screw) and the scale on the tower's face, read by the lead screw boss's top edge (one
    groove per bottle, a dot each per place from the top, and the body's diameter every 10
    mm); an M3 set screw up from the +z block onto each rod. (The carrier, its set screws.)"""
    x0, x1 = s.x
    xp, zp = REAR_POST
    xs, zs = REAR_LEAD
    za, zb, zc = REAR_FOOT_Z
    tx0, tx1, tz0, tz1 = TOWER
    ax, ab, at = ARM
    part = union([_box(x0, x1, s.y0, REAR_FOOT_TOP, za, zc),
                  _box(x0, x1, s.y0, REAR_FOOT_TOP, -zb, -za),
                  _box(xp - 7.5, xp + 7.5, s.y0, s.y0 + 10.0, -za - 0.5, za + 0.5),
                  _box(tx0, tx1, REAR_FOOT_TOP - 1.0, at, tz0, tz1),
                  _box(ax, tx1, ab, at, tz0, tz1)])
    top0 = LEAD_BOSS[2]                       # the reading edge, the block at its lowest
    named = sorted(top0 + (DESIGN.r - z.r) * math.sqrt(2) for z in SIZES)[::-1]
    graded = [top0 + y - s.top for y in _graded(s, DESIGN.r, 10.0, 70.0, 170.0)]
    graded = [y for y in graded if all(abs(y - n) > 1.5 for n in named)]
    floor = REAR_FOOT_TOP - 0.5 - COLLAR[1] - BRAKE_T     # the collar pocket's floor
    gz = tz0 + 10.0                           # the named grooves' end; their dots after
    cuts = [_rod_y(xp, zp, POST_D / 2 + 0.15, POST[1], REAR_FOOT_TOP + 1.0),
            _rod_y(xp, zp, M6.head_d / 2 + 0.3, s.y0 - 1.0, s.y0 + M6.head_h + 0.25),
            _rod_y(xp, zp, 3.3, s.y0, POST[1] + 0.5),
            _rod_y(xs, zs, LEAD_SCREW[0] / 2 + 0.2, LEAD_SCREW[3] - 0.5, floor + 0.01),
            _rod_y(xs, zs, COLLAR[0] / 2 + 0.4, floor, REAR_FOOT_TOP + 1.0),
            _box(xs - 6.0, xs + 6.0, floor, REAR_FOOT_TOP + 1.0, zs, zc + 1.0),
            _rod_y(xs, zs, LEAD_SCREW[0] / 2 + 0.2, ab - 1.0, at + 1.0),
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
        if under - SET_SCREW - 0.05 < s.y0:
            raise ValueError(f"{s.name}: an M3 x {SET_SCREW:g} set screw doesn't fit under "
                             f"tie {n} ({under - s.y0:.1f} mm)")
        cuts.append(_rod_y(x, -zp, SET_PILOT / 2, s.y0 - 2.0, under))
        screws.append(_rod_y(x, -zp, 1.25, under - 0.05 - SET_SCREW, under - 0.05))
    return difference(part, *cuts), screws


def _rear_vee(s: Saddle, ties: Mapping[str, tuple[float, float]]
              ) -> tuple[Shape3D, Shape3D, Shape3D, Shape3D]:
    """The rear V block at its lowest (cradle frame): a V shell (:data:`VEE_T`) sitting on
    the tie rods in a seat over each, its flats along the top (the base stop's guide groove
    along each, its T-slot under the +z one, its scale across the +z one), a boss at the
    -z end round the post (a sliding fit) and a low one at the +z end round the lead screw
    (its bronze barrel nut in a bore from below, an M3 set screw holding it from turning:
    the block bears down on it), a gusset under the flats beside each (they carry the roll
    of a load at the axis held at the screw), a socket under each flat at its back end for
    a rail's tongue, its TPU liner pegged into the V. (The block, its liner, the barrel
    nut, its set screw.)"""
    x0, x1 = s.vee
    xp, zp = REAR_POST
    xs, zs = REAR_LEAD
    lo, hi, w = REAR_FLANGE
    shell = cast(Shape3D, _box(x0, x1, s.bottom, hi, -w, w)
                 & _vee(x0, x1, s.apex - VEE_T * math.sqrt(2)))
    edge = lo - (s.apex - VEE_T * math.sqrt(2))         # (where the shell meets the flats)
    rp, rs = REAR_BOSS[0], LEAD_BOSS[0]
    gussets = [_prism_x([(-w, lo + 0.01), (-w, GUSSET[1]), (zp + rp, GUSSET[1]),
                         (-edge, lo + 0.01)], xp - GUSSET[0] / 2, xp + GUSSET[0] / 2),
               _prism_x([(w, lo + 0.01), (w, GUSSET[1]), (edge, lo + 0.01)],
                        xs - GUSSET[0] / 2, xs + GUSSET[0] / 2)]
    seats = [_box(ties[n][0] - 12.0, ties[n][0] + 12.0, ties[n][1] + TIE_D / 2 - SEAT_DEPTH,
                  s.bottom + 6.0, -12.0, 12.0) for n in s.ties]
    wx, wz0, wz1, wz2, wtop = LEAD_WEB
    keel = TSLOT[6]
    sx0, sz0, sz1, _ = SOCKET
    sockets = [_box(sx0, x1, _socket_bottom(sign), lo + 0.01, *sorted((sign * sz0, sign * sz1)))
               for sign in (-1, 1)]
    block = union([shell, _box(x0, x1, lo, hi, -w, w), *gussets, *seats,
                   _rod_y(xp, zp, rp, REAR_BOSS[1], REAR_BOSS[2]),
                   _rod_y(xs, zs, rs, LEAD_BOSS[1], LEAD_BOSS[2]),
                   _box(xs - wx, xs + wx, LEAD_BOSS[1], lo + 0.01, wz0, wz1),
                   _box(xs - wx, xs + wx, LEAD_BOSS[1], wtop, wz1 - 0.01, wz2),
                   _box(x0, x1, keel, lo + 0.01, GROOVE[0] - 5.0, GROOVE[1] + 5.0),
                   *sockets])
    g0, g1, depth = GROOVE
    nd, nl = LEAD_NUT
    tx0, tz0, tz1, th = TONGUE
    cuts = [_rod_y(xp, zp, POST_D / 2 + 0.2, REAR_BOSS[1] - 1.0, REAR_BOSS[2] + 1.0),
            *[Cylinder(SEAT_R, 40.0).moved(Location((*ties[n], 0.0))) for n in s.ties],
            _rod_y(xs, zs, LEAD_SCREW[0] / 2 + 0.5, LEAD_BOSS[1] - 1.0, LEAD_BOSS[2] + 1.0),
            _rod_y(xs, zs, nd / 2 + 0.1, LEAD_BOSS[1] - 1.0, LEAD_BOSS[1] + nl),
            _rod_z(xs, LEAD_BOSS[1] + nl / 2, SET_PILOT / 2, zs + nd / 2 - 0.5, zs + rs + 1.0),
            _box(x0 - 1.0, x1 + 1.0, hi - depth, hi + 1.0, g0, g1),
            _box(x0 - 1.0, x1 + 1.0, hi - depth, hi + 1.0, -g1, -g0),
            *_tslot(TSLOT[0], x1 + 1.0),
            *_stop_scale(x0, x1, hi),
            *[_box(tx0 - CLEAR, x1 + 1.0, _tongue_top(sign) - th - CLEAR,
                   _tongue_top(sign), *sorted((sign * (tz0 - CLEAR), sign * (tz1 + CLEAR))))
              for sign in (-1, 1)],
            *[_rod_y(x, sign * (tz0 + tz1) / 2, TIE_HOLE / 2, _socket_bottom(sign) - 1.0,
                     _tongue_top(sign) - th) for x in RAIL_SCREW_X for sign in (-1, 1)],
            *[_rod_y(x, sign * (tz0 + tz1) / 2, M5.head_d / 2 + 0.5,
                     _socket_bottom(sign) - 1.0, _socket_bottom(sign) + RAIL_HEAD_SINK)
              for x in RAIL_SCREW_X for sign in (-1, 1)]]
    vee, liner = _vee_block(s, difference(block, *cuts), REAR_PEGS)
    nut = difference(_rod_y(xs, zs, nd / 2, LEAD_BOSS[1], LEAD_BOSS[1] + nl),
                     _rod_y(xs, zs, LEAD_SCREW[0] / 2, LEAD_BOSS[1] - 1.0,
                            LEAD_BOSS[1] + nl + 1.0))
    z0 = zs + nd / 2 + 0.05
    set_screw = _rod_z(xs, LEAD_BOSS[1] + nl / 2, 1.25, z0, z0 + SET_SCREW)
    return vee, liner, nut, set_screw


def _lead_screw() -> tuple[Shape3D, Shape3D, Shape3D, Shape3D, Shape3D]:
    """The rear thumbscrew (cradle frame): the Tr8 x 8 lead screw from its foot in the
    carrier's bore up through its clamping collar (in the carrier's pocket on the TPU brake
    washer: the block's load on the collar's face holds the screw from being turned back
    by it), the V block's barrel nut and the tower's bushing, the printed knob on its top
    (an M3 set screw onto the screw). (The screw, the collar, the brake washer, the knob,
    the knob's set screw.)"""
    xs, zs = REAR_LEAD
    d, length, _, foot = LEAD_SCREW
    od, cl = COLLAR
    top = REAR_FOOT_TOP - 0.5
    screw_ = _rod_y(xs, zs, 0.97 * d / 2, foot, foot + length)
    collar = difference(_rod_y(xs, zs, od / 2, top - cl, top),
                        _rod_y(xs, zs, d / 2, top - cl - 1.0, top + 1.0))
    brake = difference(_rod_y(xs, zs, od / 2, top - cl - BRAKE_T, top - cl),
                       _rod_y(xs, zs, d / 2 + 0.3, top - cl - BRAKE_T - 1.0, top - cl + 1.0))
    k0, k1 = REAR_KNOB
    knob = difference(_rod_y(xs, zs, KNOB_R[1], k0, k1),
                      _rod_y(xs, zs, d / 2 + 0.1, k0 - 1.0, foot + length + 0.3),
                      _rod_x(k0 + 4.0, zs, SET_PILOT / 2, xs - KNOB_R[1] - 1.0, xs))
    if foot + length < k0 + 8.0 or foot + length > k1 - 1.5:
        raise ValueError("the rear lead screw's top isn't inside its knob")
    set_screw = _rod_x(k0 + 4.0, zs, 1.25, xs - d / 2 - SET_SCREW + 0.05, xs - d / 2 + 0.05)
    return screw_, collar, brake, knob, set_screw


def _post_y(xz: tuple[float, float], y0: float, y1: float) -> Shape3D:
    """A 12 mm steel standoff up y from ``y0`` to ``y1``, its ends tapped M6."""
    x, z = xz
    return difference(_rod_y(x, z, POST_D / 2, y0, y1),
                      *[_rod_y(x, z, 3.0, a, b) for a, b in ((y0 - 1, y0 + 12.0),
                                                             (y1 - 12.0, y1 + 1))])


# --- the base stop: a gate on the rear V block's flats and the rails behind it -----------

RAIL_END = 600.0              # the rails' end (cradle x): the 6 L's gate locked short of it
RAIL_Z = 92.0                 # the rails' outer faces (their inner the V's edge)
FLANGE = (-85.0, 8.5)         # each rail's 45° flange continuing the V behind the block (the
#                               body of a 3 L or more rests on it: review round 1, B1): its
#                               bottom (cradle y) and its thickness across
FLANGE_END = 588.0            # where the flanges and their liners end (cradle x): past a 6 L's
#                               heel (561), short of the rails' end, so that at rest nothing of
#                               the base stop comes nearer the base board than a finger's 25 mm
#                               (ISO 13854: the end of the flange had been 18.8 mm over it)
KEEL_CHAMFER = 5.0            # the +z keel's bottom end cut back at 45° this far, for the same
FINGER_GAP = 25.0             # ISO 13854's least gap for a finger
TONGUE = (352.0, 74.0, 90.0, 12.0)      # each rail's tongue into its socket under the flats:
#                                         its front x, |z| span, height (its top the flats'
#                                         underside, the +z one's keel's)
SOCKET = (340.0, 71.0, 93.0, 10.7)      # the sockets: front x, |z| span, the floor under the
#                                         tongue's cavity


def _tongue_top(sign: int) -> float:
    """A rail's tongue's top (cradle y, the V block at its lowest): under the flats, or on the
    +z side (``sign`` > 0) under the keel round the T-slot."""
    return TSLOT[6] if sign > 0 else REAR_FLANGE[0]


def _socket_bottom(sign: int) -> float:
    """A socket's bottom under its tongue's cavity (:data:`SOCKET`'s floor)."""
    return _tongue_top(sign) - TONGUE[3] - CLEAR - SOCKET[3]
RAIL_SCREW_X = (358.0, 374.0)           # the two M5 x 16 up through each socket into its tongue
RAIL_SCREW = 16.0
RAIL_HEAD_SINK = 3.5                    # their heads sunk this far into the socket's floor
FLANGE_PEGS = ((430.0, -70.0), (540.0, -70.0))  # the flange liners' pegs (x, the face's y)
GATE_T = 10.0                 # the gate's plate along the axis
GATE_BOTTOM = -92.0           # its bottom edge (at the lowest): 5 mm under a 6 L's base
EAR_END = 24.0                # its ears on the rails, behind its face
LOCK_X = 17.0                 # the lock's bolt through the +z ear, behind the face
LOCK_KNOB = (13.0, 9.5)       # its printed knob on the bolt's head: radius, height
LOCK_BOLT = 25.0              # M5 x 25 tap bolt: knob, ear, key, lips, washer, nut
LOCK_KEYS = ("m5_tap_bolt_25", "m5_washer_wide", "m5_nut")
LOCK_WASHER = (15.0, 1.2)     # the M5 fender washers (under the bolt's head, on the lips)


def _rails(s: Saddle) -> tuple[list[Shape3D], list[Shape3D], list[Shape3D]]:
    """The base stop's rails (cradle frame, the V block at its lowest): two bars, one each
    side, continuing the rear V block's flats behind it, their guide grooves (the +z one's
    T-slot and scale too), each with a 45° flange under it continuing the V to
    :data:`FLANGE`'s bottom (its TPU liner pegged on) and a tongue forward into its socket
    under the block's flats, two M5 button heads up through the socket's floor into it.
    (The rails (-z, +z), their liners, their four screws.)"""
    hi = REAR_FLANGE[1]
    g0, g1, depth = GROOVE
    yb, ft = FLANGE
    keel = TSLOT[6]
    lo = REAR_FLANGE[0]
    xr = s.vee[1]
    tx0, tz0, tz1, th = TONGUE

    def zv(y: float) -> float:                 # the V's face (the liners' backs) at y
        return y - s.apex

    zy = [(zv(hi), hi), (RAIL_Z, hi), (RAIL_Z, lo), (zv(lo) + ft, lo), (zv(yb) + ft, yb),
          (zv(yb), yb)]
    rails, liners, screws = [], [], []
    for sign in (-1, 1):
        bar = _prism_x([(sign * z, y) for z, y in zy], xr, RAIL_END)
        top = _tongue_top(sign)
        tongue = _box(tx0, xr + 0.5, top - th, top + 0.01, *sorted((sign * tz0, sign * tz1)))
        zk = sorted((sign * (GROOVE[0] - 5.0), sign * (GROOVE[1] + 5.0)))
        if sign > 0:                           # the keel round the T-slot
            tongue = union([tongue, _box(xr, RAIL_END, keel, lo + 0.01, *zk)])
        sbot = _socket_bottom(sign)
        k = KEEL_CHAMFER
        cuts = [_box(xr - 1.0, RAIL_END + 1.0, hi - depth, hi + 1.0,
                     *sorted((sign * g0, sign * g1))),
                _box(FLANGE_END, RAIL_END + 1.0, yb - 1.0, lo, *sorted(        # (the flange's
                    (0.0, sign * (GROOVE[0] - 5.5))))]                          # end)
        if sign > 0:
            chamfer = Solid.extrude(Face(Wire.make_polygon(
                [(RAIL_END - k, keel - 0.01, zk[0] - 1.0), (RAIL_END + 0.01, keel - 0.01,
                                                             zk[0] - 1.0),
                 (RAIL_END + 0.01, keel + k, zk[0] - 1.0)], close=True)),
                (0, 0, zk[1] - zk[0] + 2.0))
            cuts += [*_tslot(xr - 1.0, RAIL_END - 3.0), *_stop_scale(xr, RAIL_END, hi), chamfer]
        zt = sign * (tz0 + tz1) / 2
        y_head = sbot + RAIL_HEAD_SINK             # the screws' heads' tops, sunk in the floor
        cuts += [_rod_y(x, zt, M5_PILOT / 2, top - th - 1.0, y_head + RAIL_SCREW + 1.0)
                 for x in RAIL_SCREW_X]
        band = [(sign * zv(hi), hi), (sign * (zv(hi) - LINER * math.sqrt(2)), hi),
                (sign * (zv(yb) - LINER * math.sqrt(2)), yb), (sign * zv(yb), yb)]
        liner = _prism_x(band, xr + CLEAR, FLANGE_END - CLEAR)
        keys, holes = _liner_pegs(s.apex, FLANGE_PEGS, (sign,))
        rails.append(difference(union([bar, tongue]), *cuts, *holes))
        liners.append(union([liner, *keys]))
        screws.extend(union([_rod_y(x, zt, M5.head_d / 2, y_head - M5.head_h, y_head),
                             _rod_y(x, zt, 2.4, y_head, y_head + RAIL_SCREW)])
                      for x in RAIL_SCREW_X)
    return rails, liners, screws


def _gate(s: Saddle, stop: float) -> tuple[Shape3D, list[Shape3D]]:
    """The base stop's gate, its face at ``stop`` (cradle frame, the V block at its lowest):
    a plate across the V behind the bottle's base (a millimetre inside the liners' face,
    from :data:`GATE_BOTTOM` up), its top on the flats or the rails with a pointer over the
    scale (its front edge reads it), an ear on each behind it keyed into its groove; the
    lock through the +z ear: an M5 tap bolt down through a printed knob (its head and a
    fender washer in the knob), the ear and its key into a fender washer and an M5 nut in
    the T-slot under the groove. Turned by hand it clamps the gate anywhere along the
    slot: no tool, no thread in the plastic. (The gate; the lock's bolt, knob, washers (on
    the ear, under the lips) and nut.)"""
    hi = REAR_FLANGE[1]
    g0, g1, depth = GROOVE
    face = s.apex + LINER * math.sqrt(2)

    def zf(y: float) -> float:
        return y - face - 1.0

    top = hi + 6.0
    ze = 88.0                                  # the ears' and the top bar's outer edges
    zy = [(-zf(GATE_BOTTOM), GATE_BOTTOM), (zf(GATE_BOTTOM), GATE_BOTTOM), (zf(hi), hi),
          (ze, hi), (ze, top), (-ze, top), (-ze, hi), (-zf(hi), hi)]
    ears = [_box(stop + GATE_T - 0.01, stop + EAR_END, hi, top, za, zb)
            for za, zb in ((72.0, ze), (-ze, -72.0))]
    keys = [_box(stop, stop + EAR_END, hi - depth + CLEAR, hi + 0.01, za, zb)
            for za, zb in ((g0 + CLEAR, g1 - CLEAR), (-g1 + CLEAR, -g0 - CLEAR))]
    pointer = _box(stop, stop + 3.0, hi, top, ze - 0.01, RAIL_Z - 0.5)
    xl, zl = stop + LOCK_X, (g0 + g1) / 2
    gate = difference(union([_prism_x(zy, stop, stop + GATE_T), *ears, *keys, pointer]),
                      _rod_y(xl, zl, TIE_HOLE / 2, hi - depth - 1.0, top + 1.0))
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
LED_X = 118.0               # its axis, in the cradle's frame: under either bottle's shoulder
LED_TOP = -45.0             # and its head, 10-25 mm under the glass there
CLIP_GAP = 16.0             # the clip starts this far under the head (the head twists)


def _bracket(xy, y_top: float, face: float, sign: int, t_frame: float
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
    board's hole (x, z).)"""
    length, height, wall = BRACKET
    fx, fy = xy
    zs = sorted((face, face + sign * wall))
    up = Box(length, height, wall).moved(Location((fx, y_top + height / 2, sum(zs) / 2)))
    zf = sorted((face, face + sign * height))
    foot = Box(length, wall, height).moved(Location((fx, y_top + wall / 2, sum(zf) / 2)))
    zb = face + sign * BOLT_OUT                      # the foot leg's bolt
    hole_up = disc((fx, fy), TIE_HOLE / 2, zs[0] - 1, zs[1] + 1)
    hole_down = _rod_y(fx, zb, BRACKET_HOLE / 2, y_top - 1.0, y_top + wall + 1.0)
    bracket = difference(union([up, foot]), hole_up, hole_down)
    # through the upright and the plate, the nut over the plate's inner face
    l_side = _m5_length(wall + t_frame + M5_NUT[1])
    inner = face - sign * t_frame
    nut_z = sorted((inner, inner - sign * M5_NUT[1]))
    side_screw = _screw((fx, fy), face + sign * wall, sign, l_side)
    side_nut = _nut((fx, fy), *nut_z)
    # a carriage bolt up from under the base board (its square neck in the board's square
    # hole: it can't turn), through the foot, its washer and knob on the foot from above
    d, blen, head_d, neck_h, head_h = CARRIAGE
    y_under = y_top - BASE_T
    if y_under - head_h < y_top - BASE_TOP_CLEAR:
        raise ValueError("the base board's carriage bolts' heads reach the table")
    w_id, w_od, w_t = WASHER
    af, nut_h = M6_HEX[0], M6_HEX[1]
    knob_d = KNOB[0]
    if neck_h > BASE_T or BASE_T + wall + w_t + nut_h + 1.0 > blen:
        raise ValueError("the carriage bolt doesn't reach through the board, the foot, its "
                         "washer and its nut, a thread past it")
    if BOLT_OUT - knob_d / 2 < wall + M5.head_h + 1.0:
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
    return bracket, [(side_screw, M5.key(l_side), "screw0"), (side_nut, "m5_nut", "nut0"),
                     (union([head, shank]), f"m6_carriage_{blen:g}", "screw1"),
                     (washer, WASHER_KEY, "washer1"), (nut, "m6_nut", "nut1"),
                     (_knob(fx, zb, y_n), None, "knob")], (fx, zb)


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


def _base_plate(lip, stand: Stand, y_top: float, holes: list[tuple[float, float]],
                z_mid: float = 0.0) -> tuple[Shape3D, list[tuple[float, float]]]:
    """The base board (world): from the stand's front to its rear edge, :data:`BASE_HALF`
    each side of ``z_mid`` (:func:`base_z`), under both sides' outer plates, its corners
    rounded (:data:`BASE_CORNER_R`); a notch from its front edge round the decanter (its
    round end under the mouth, its mouth's corners rounded, a bay each side for the jack
    locator's ears), the brackets' carriage bolts' square holes, a hole near each corner for
    a TPU foot. (The board, the feet's (x, z).)"""
    t = BASE_T
    x0, x1 = float(lip[0]) + stand.base[0], float(lip[0]) + stand.base[1]
    za, zb = z_mid - BASE_HALF, z_mid + BASE_HALF
    y0, y1 = y_top - t, y_top
    plate = _rounded(x0, x1, za, zb, y0, y1, BASE_CORNER_R)
    xd = float(lip[0]) + DECANTER_X
    c0, c1 = y0 - 1.0, y1 + 1.0
    m = NOTCH_MOUTH_R
    cuts = [_box(x0 - 1.0, xd, c0, c1, -NOTCH_R, NOTCH_R), _rod_y(xd, 0.0, NOTCH_R, c0, c1)]
    cuts += [_rod_y(xd + NOTCH_BAY[0], s * NOTCH_R, NOTCH_BAY[1], c0, c1) for s in (-1, 1)]
    cuts += [difference(_box(x0 - 1.0, x0 + m, c0, c1, *sorted((s * (NOTCH_R - 1.0),
                                                               s * (NOTCH_R + m)))),
                        _rod_y(x0 + m, s * (NOTCH_R + m), m, c0 - 1.0, c1 + 1.0))
             for s in (-1, 1)]
    cuts += [_square_hole(x, z, CARRIAGE_HOLE, c0, c1) for x, z in holes]
    feet = [(x, z) for x in (x0 + FOOT_INSET, x1 - FOOT_INSET)
            for z in (za + FOOT_INSET, zb - FOOT_INSET)]
    cuts += [_rod_y(x, z, FOOT_STEM / 2 + 0.25, c0, c1) for x, z in feet]
    return difference(plate, union(cuts)), feet


def _jack_locator(lip, y_table: float) -> list[Shape3D]:
    """The jack locator (world): :data:`LOCATOR`'s layers of the board's birch, glued, on the
    table in the board's notch (its outline the notch's, :data:`LOCATOR`'s play in, from its
    front edge to the round end; an ear into each of the notch's bays), a square pocket for
    the lab jack's base (:data:`JACK_BASE`) under the mouth. It puts the jack's platform
    under the mouth (it had floated 76 mm either way in the bare notch: round 1's stand
    review) and lifts out to set a vessel on the table."""
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
                        for s in (-1, 1))])
        out.append(difference(body, _square_hole(xd, 0.0, JACK_BASE + 2 * JACK_PLAY,
                                                 y0 - 1.0, y1 + 1.0)))
    return out


def _cleats(xs: list[float], y_top: float, face: float, sign: int, t_frame: float
            ) -> list[Shape3D]:
    """A rail's birch cleats (world), glued to the base board either side of the outer
    plate's rail, between its brackets (``xs``: their x, along the rail), :data:`CLEAT`'s
    play off its faces: they hold the side upright on the board (the brackets alone had let
    it sway several mm at a 20 N push: round 1's stand review) and drop it into place when
    it goes back on."""
    width, play, gap = CLEAT
    inner = face - sign * t_frame
    out = []
    xs = sorted(xs)
    for xa, xb in itertools.pairwise(xs):
        a, b = xa + BRACKET[0] / 2 + gap, xb - BRACKET[0] / 2 - gap
        for z0, z1 in ((face + sign * play, face + sign * (play + width)),
                       (inner - sign * play, inner - sign * (play + width))):
            out.append(_box(a, b, y_top, y_top + BASE_T, *sorted((z0, z1))))
    return out


FOOT_STEM = 10.0            # a TPU foot's stem through the base board's corner hole


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

DRIVE_PARTS = ("gearbox", "riser_spacer", "crank_", "crankpin", "handle")
"""The hand crank's parts by name (:mod:`construction.handcrank`): the drive side's only."""

GROUPS = {"drive": ("gearbox", "riser_spacer", "handle"), "crank": ("conn",),
          "box": ("tie_", "front_saddle", "rear_saddle", "led", "base_stop"),
          "stand": ("base_plate", "base_cleat", "jack_locator", "foot_")}
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
    return replace(side, bodies=keep, connections=[
        c for c in side.connections if c[0][1] in names and c[1][1] in names])


KEEPER = (7.6, 0.2)
"""The keepers of the cradle link's quick-release pins (B's X and Y, both sides; the pack's
turn 1, 2026-10-14): the printed ring each clevis pin's plan puts in the gap between its
lowest link (b4, b5) and the cradle link, printed in TPU 95A with its bore (mm) under the
8 mm pin, so it grips the pin (:func:`spiderpig.cradle_report.keepers`); and the least
interference a print may come out with (mm, on the diameter: a printed hole runs small,
and TPU's run smaller still). A side lying down with its pins' clips out holds them by
their keepers: each pin's head is under its link with nothing under it, and a bare pin
drops out. So the cradle box lifts off the drive side as it lies (:func:`module_steps`),
and the passive panel lifts off with its pins and their keepers on them: no spacer is
left loose on the cradle link."""
KEEPER_COLOR = LINER_COLOR


def keeper_names(side: Mechanism, lk) -> list[str]:
    """The drive side ``side``'s gap rings (bare names) that are keepers (:data:`KEEPER`): on
    each pin of the cradle link's with a clip (a quick-release clevis pin), every ring the
    pin's plan puts in a gap between its links; none on a cradle without such pins (A). A
    clipped pin with no ring is an error: nothing would hold it with its side down."""
    import re

    link = cradle_link(lk)
    names = {b.name for b in side.bodies}
    pins = sorted(j for j in lk.links[link][0]
                  if any(j in js for b, (js, _) in lk.links.items() if b != link)
                  and f"pin_{j}_clip" in names)
    out = []
    for j in pins:
        rings = sorted(n for n in names if re.fullmatch(rf"pin_{re.escape(j)}_gap\d+_spacer", n))
        if not rings:
            raise ConstructionError(f"pin {j}: a quick-release pin with no ring in a gap to "
                                    "keep it (construction.cradle.KEEPER)")
        out += rings
    return out


def _keeper(part):
    """A gap ring ``part`` (a side's frame, along z) as a keeper: its outline, its bore
    :data:`KEEPER`'s."""
    from spiderpig.shapes import ring

    bb = part.bounding_box()
    xy = ((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2)
    return ring(xy, bb.max.X - bb.min.X, KEEPER[0], bb.min.Z, bb.max.Z)


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
    lip = build.xy(_point_of(topo, lk, mouth_name(lk)))
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
    extras: list[Body] = []
    fastened = []
    tie_len = round(2 * half, 1)
    for n, p in tie_points(topo, lk).items():
        xy = tuple(build.xy(p))
        extras.append(hardware(f"tie_{n}", _tie(xy, -half, half), host, fab="purchased",
                               bom_key=_standoff(tie_len), color=TIE_COLOR))
        for s, sign in (("L", -1), ("R", 1)):
            face = sign * (z_mid - link_bottom)      # the link's outer face
            extras.append(hardware(f"tie_{n}_screw{s}", _screw(xy, face, sign), host,
                                   fab="purchased", bom_key=M5.key(TIE_SCREW),
                                   color=TIE_COLOR))
    stand = stand_of(lk)
    z0 = plan.z(0)[1]
    foot_half = z_mid - z0                            # a base tie between the outer plates
    foot_len = round(2 * foot_half, 1)
    for n in stand.base_ties:
        xy = tuple(_node_xy(build.xy, lk, topo, stand, n))
        extras.append(hardware(f"base_tie_{n}", _tie(xy, -foot_half, foot_half), frame_host,
                               fab="purchased", bom_key=_standoff(foot_len), color=TIE_COLOR))
        for s, sign in (("L", -1), ("R", 1)):
            face = sign * (z_mid - plan.z(0)[0])
            extras.append(hardware(f"base_tie_{n}_screw{s}", _screw(xy, face, sign), frame_host,
                                   fab="purchased", bom_key=M5.key(TIE_SCREW),
                                   color=TIE_COLOR))
    bay_half = BAY / 2                  # a bay tie between the inner plates' bay faces
    bay_len = round(2 * bay_half, 1)
    for n in stand.bay_ties:
        xy = tuple(_node_xy(build.xy, lk, topo, stand, n))
        extras.append(hardware(f"bay_tie_{n}", _tie(xy, -bay_half, bay_half), frame_host,
                               fab="purchased", bom_key=_standoff(bay_len), color=TIE_COLOR))
        for s, sign in (("L", -1), ("R", 1)):
            face = sign * (z_mid - plan.z(plan.top)[0])     # the inner plate's leg face
            extras.append(hardware(f"bay_tie_{n}_screw{s}", _screw(xy, face, sign), frame_host,
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

    lead, collar, brake, lead_knob, lead_set = _lead_screw()
    rear_vee, rear_liner, rear_nut, rear_nut_set = _rear_vee(rear, cf.ties)
    for s, (carrier, sets), (vee, liner, nut), (jack, knob), du, keys in (
            (front, _front_saddle(front, cf.ties), _front_vee(front),
             _jack(FRONT_JACK, 6.0, FRONT_SCREW, (M6_HEX[0], M6_HEX[2]), FRONT_HEAD_Y,
                   KNOB_R[0]), at.front, (FRONT_NUT_KEY, FRONT_JACK_KEY)),
            (rear, _rear_saddle(rear, cf.ties), (rear_vee, rear_liner, rear_nut),
             (lead, lead_knob), at.rear, (LEAD_KEYS[1], LEAD_KEYS[0]))):
        if s is front:                  # the light's clip's screws' pilots
            carrier = difference(carrier, *clip_pilots)
        put(s.name, carrier, "printed", SADDLE_COLOR)
        put(f"{s.name}_vee", vee, "printed", VEE_COLOR, dy=du)
        put(f"{s.name}_liner", liner, "printed", LINER_COLOR, dy=du)
        put(f"{s.name}_nut", nut, "purchased", TIE_COLOR, keys[0], dy=du)
        put(f"{s.name}_jack", jack, "purchased", TIE_COLOR, keys[1])
        put(f"{s.name}_knob", knob, "printed", KNOB_COLOR)
        # a set screw up from its underside holds the carrier on each tie
        fastened.extend((s.name, f"tie_{n}") for n in s.ties)
        for n, set_screw in zip(s.ties, sets, strict=True):
            name = f"{s.name}_set_screw_{n}"
            put(name, set_screw, "purchased", TIE_COLOR, f"m3_set_screw_{SET_SCREW:g}")
            fastened += [(name, s.name), (name, f"tie_{n}")]
        fastened += [(f"{s.name}_vee", f"{s.name}_liner"), (f"{s.name}_nut", f"{s.name}_vee"),
                     (f"{s.name}_nut", f"{s.name}_jack"), (f"{s.name}_knob", f"{s.name}_jack"),
                     (f"{s.name}_jack", s.name), ("bottle", f"{s.name}_liner")]
    # the rear lead screw's collar on its brake washer, the set screws in its knob and on
    # its barrel nut
    set_key = f"m3_set_screw_{SET_SCREW:g}"
    put("rear_saddle_lead_clamp", collar, "purchased", TIE_COLOR, LEAD_KEYS[2])
    put("rear_saddle_brake", brake, "printed", LINER_COLOR)
    put("rear_saddle_knob_set_screw", lead_set, "purchased", TIE_COLOR, set_key)
    put("rear_saddle_nut_set_screw", rear_nut_set, "purchased", TIE_COLOR, set_key, dy=at.rear)
    fastened += [("rear_saddle_lead_clamp", "rear_saddle_jack"),
                 ("rear_saddle_lead_clamp", "rear_saddle"), ("rear_saddle_brake", "rear_saddle"),
                 ("rear_saddle_brake", "rear_saddle_jack"),
                 ("rear_saddle_brake", "rear_saddle_lead_clamp"),
                 ("rear_saddle_knob_set_screw", "rear_saddle_knob"),
                 ("rear_saddle_knob_set_screw", "rear_saddle_jack"),
                 ("rear_saddle_nut_set_screw", "rear_saddle_vee"),
                 ("rear_saddle_nut_set_screw", "rear_saddle_nut")]
    # the rear V block's post, screwed up into from under the carrier
    y_head = rear.y0 + 0.25
    put("rear_saddle_post", _post_y((xp, zp), POST[1], POST[1] + POST[0]), "purchased",
        TIE_COLOR, _standoff12(POST[0]))
    put("rear_saddle_post_screw",
        union([_rod_y(xp, zp, M6.head_d / 2, y_head, y_head + M6.head_h),
               _rod_y(xp, zp, 2.9, y_head + M6.head_h, y_head + M6.head_h + POST_SCREW)]),
        "purchased", TIE_COLOR, M6.key(POST_SCREW))
    fastened += [("rear_saddle_post", "rear_saddle"), ("rear_saddle_post", "rear_saddle_vee"),
                 ("rear_saddle_post_screw", "rear_saddle"),
                 ("rear_saddle_post_screw", "rear_saddle_post")]
    # the lip gauge, down: its pad at the mouth
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
    # the base board, its TPU feet, its cleats, the jack locator in its notch, and each foot's
    # bracket bolting an outer plate onto it
    y_top = float(cf.lip[1]) + BASE_TOP
    t_frame = plan.t(0)
    base_holes = []
    cleats = []
    for s, sign in (("L", -1), ("R", 1)):
        face = sign * (z_mid - plan.z(0)[0])          # the outer plate's outer face
        host_s = prefixed(topo.frame_bodies[0], s)
        feet_xy = {n: tuple(float(v) for v in _node_xy(build.xy, lk, topo, stand, n))
                   for n in stand.feet}
        for n, xy in feet_xy.items():
            bracket, screws, hole = _bracket(xy, y_top, face, sign, t_frame)
            base_holes.append(hole)
            extras.append(hardware(f"foot_bracket_{n}{s}", bracket, host_s, fab="printed",
                                   color=SADDLE_COLOR))
            for part, key, suffix in screws:
                extras.append(hardware(f"foot_bracket_{n}{s}_{suffix}", part, host_s,
                                       fab="purchased" if key else "printed", bom_key=key,
                                       color=TIE_COLOR if key else KNOB_COLOR))
        cleats += [hardware(f"base_cleat_{s}{i}", part, frame_host, fab="laser",
                            sheet=BASE_SHEET, color=BOARD_COLOR)
                   for i, part in enumerate(_cleats([xy[0] for xy in feet_xy.values()], y_top,
                                                    face, sign, t_frame))]
    plate_part, foot_xz = _base_plate(cf.lip, stand, y_top, base_holes, base_z(lk))
    extras.append(hardware("base_plate", plate_part, frame_host, fab="laser",
                           sheet=BASE_SHEET, color=BOARD_COLOR))
    extras += cleats
    extras += [hardware(f"jack_locator_{k}", part, frame_host, fab="laser", sheet=BASE_SHEET,
                        color=BOARD_COLOR)
               for k, part in enumerate(_jack_locator(cf.lip, y_top - BASE_T - FOOT_PAD_H))]
    glue = [BomLine(WOOD_GLUE, 1, "the base board's cleats and the jack locator's layers")]
    # each keeper grips its pin (its bore under the pin's: an interference fit, not a clash)
    fastened += [(prefixed(n, s), prefixed(n.split("_gap")[0] + "_pin", s))
                 for n in sorted(keepers) for s in SIDES_LR]
    for i, (x, z) in enumerate(foot_xz):
        extras.append(hardware(f"foot_boot_{i}", _tpu_foot(x, z, y_top), frame_host,
                               fab="printed", color=LINER_COLOR))
    y0, slope, x0 = seat(shown)
    extras.append(hardware("bottle", _place(_bottle(shown, y0, slope, x0), cf.lip, cf.tilt),
                           host, fab="display", color=BOTTLE_COLOR))
    vessel, riser = _decanter(cf.lip)
    extras.append(hardware("decanter", vessel, frame_host, fab="display",
                           color=DECANTER_COLOR))
    extras.append(hardware("decanter_riser", riser, frame_host, fab="display",
                           color="#3a3f45"))
    fastened += [("led_clip", SADDLES[0].name), ("led", "led_clip"),
                 ("led_set_screw", "led_clip"), ("led_set_screw", "led")]
    fastened += [(f"led_clip_screw{i}", n) for i in range(len(clip_screws))
                 for n in ("led_clip", SADDLES[0].name)]
    meta = dict(side.meta, cradle=True, cradle_layer=plan.layers[cradle_link(lk)],
                print_poses=print_poses([b.name for b in extras + bodies], cf.tilt),
                cradle_lip=[float(cf.lip[0]), float(cf.lip[1])], cradle_tilt=float(cf.tilt),
                mid_plane=z_mid, bay_mm=BAY, lip_height_mm=LIP_HEIGHT,
                tie_mm=tie_len, base_tie_mm=foot_len, bay_tie_mm=bay_len, fastened=fastened,
                bottle=shown.name, cradle_linkage=lk.key,
                keepers=sorted(prefixed(n, s) for n in keepers for s in SIDES_LR))
    return Mechanism(name=f"{side.name}_cradle", bodies=bodies + extras,
                     connections=connections, meta=meta,
                     bom_extras=list(side.bom_extras) + list(passive.bom_extras) + glue)


def _decanter(lip) -> tuple[Shape3D, Shape3D]:
    """A standard decanter under the mouth (display, :data:`DECANTER_PROFILE`) on its
    riser (:data:`DECANTER_RISER`): (the decanter, the riser)."""
    table = float(lip[1]) - LIP_HEIGHT
    height = DECANTER_PROFILE[-1][0]
    lift = DECANTER_RISER
    base = table + lift
    prof = [(0.0, base), *((r, base + h) for h, r in DECANTER_PROFILE), (0.0, base + height)]
    face = Face(Wire.make_polygon([(x, y, 0.0) for x, y in prof], close=True))
    vessel = moved(Solid.revolve(face, 360.0, BAxis.Y),
                   Location((float(lip[0]) + DECANTER_X, 0.0, 0.0)))
    riser = Box(RISER_W, max(lift, 1.0), RISER_W).moved(
        Location((float(lip[0]) + DECANTER_X, table + max(lift, 1.0) / 2, 0.0)))
    return vessel, riser


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
    from spiderpig.construction.assembly import CRADLE, FINISH, STACK, UNIT, USE, Op, whole

    ops = []
    k = int(view.meta.get("cradle_layer", 0))
    ties = view.named(r"tie_[A-Za-z0-9]+")
    if ties:
        ops.append(Op(STACK, (k, 0, "ties"), whole(*ties, *view.named(r"tie_\w+_screwL")),
                      "Cradle link and tie rods", "", "cradle_link", sub=True, side="L"))
    right = view.named(r"tie_\w+_screwR")
    if right:
        ops.append(Op(UNIT, (99,), whole(*right), "",      # (after the link: its key)
                      "The cradle link's M5 button heads from its outside into the rods' "
                      "ends (threadlocker).", "tie_screws", side="R"))
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
    saddles += [n for n in ("rear_saddle_post", "rear_saddle_post_screw", "rear_saddle_lead_clamp",
                            "rear_saddle_brake", "rear_saddle_knob_set_screw",
                            "rear_saddle_nut_set_screw", "front_saddle_gauge",
                            "front_saddle_gauge_pin", "front_saddle_gauge_nut") if n in view.z]
    sets = view.named(r"\w+_saddle_set_screw_\w+")
    if saddles:
        ops.append(Op(CRADLE, (0,), whole(*saddles, *sets), "Saddles",
                      "On the bench, each saddle's V block into its carrier, its TPU liner "
                      "pressed into its V, its pegs into the V's holes. The front one: its M6 "
                      "nut up into the hex pocket under its tab, the block down into the "
                      "carrier's box, the M6 thumbscrew down through the tab's nut into its "
                      "foot's bore (a dab of grease on its thread: stainless in stainless "
                      "galls dry), the knob pressed onto its head. The rear one: the steel "
                      "post into the carrier's socket, an M6 button head up into it from "
                      "below (threadlocker); the bronze barrel nut up into the block's "
                      "lead-screw boss, its M3 set screw snug on it; the TPU brake washer and "
                      "the clamping collar into the carrier's pocket, the collar's pinch screw "
                      "toward the pocket's open side; the block down over the post; the lead "
                      "screw (a little grease on its thread) down through the tower's arm, "
                      "the barrel nut and the collar into its foot's bore, and the collar's "
                      "pinch screw tight with the collar down on its washer (the block's "
                      "load on that washer is what holds the coarse screw from turning back); "
                      "the knob onto the screw's top, its M3 set screw snug. The lip gauge "
                      "onto the front carrier's post, a knuckle either side: the M5 tap bolt "
                      "through them and the post from the front, its nylon-insert nut behind, "
                      "snug: lifted to its stop and let go, it must fall shut. Then the "
                      "saddles slid onto the tie rods' free ends, the front one (the small V) "
                      "on the rods nearer the hinge, the knobs to the passive side, centred "
                      "across the bay; the M3 set screws up from their undersides into the "
                      "printed pilots, snug on the rods (they cut their own thread: no more "
                      "than snug). Turn each knob down until its V block sits on the rods.",
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
                      "rail's tongue into its socket under the rear V block's flats, the "
                      "rail's top flush with the flats and its flange with the block's V, two "
                      "M5 button heads up through the socket's floor into the tongue (snug); "
                      "each TPU liner pressed onto its rail's flange, its pegs into the holes. "
                      "The gate onto the rails from their end, its keys in the grooves, its "
                      "face toward the saddle; the nut slid along under the scale side's ear, "
                      "a fender washer on the ear, the M5 tap bolt down through it, the ear "
                      "and the washer below into the nut, its knob pressed onto its head. It "
                      "is set for each bottle (the use's first step).", "base_stop"))
    light = [n for n in ("led_clip", "led_clip_screw0", "led_clip_screw1", "led",
                         "led_set_screw") if n in view.z]
    if light:
        ops.append(Op(CRADLE, (1,), whole(*light), "Light",
                      "The light's printed clip up under the front saddle's back end, its "
                      "flange on the saddle's underside, two M5 button heads up through it "
                      "into the saddle's pilots (snug: they cut their own thread). The "
                      "penlight down into the clip, onto its lip, its head up under where the "
                      "bottle's shoulder lies; the M3 set screw into the clip's boss, snug on "
                      "the light's body (it holds the light while you twist its head to "
                      "switch it on).", "light"))
    base = [n for n in ("base_plate",) if n in view.z]
    feet = view.named(r"foot_boot_\w+")
    locator = view.named(r"jack_locator_\d+")
    cleats = {s: view.named(rf"base_cleat_{s}\d+") for s in SIDES_LR}
    brackets = {s: view.named(rf"foot_bracket_[A-Za-z0-9]+{s}(_\w+)?") for s in SIDES_LR}
    # every carriage bolt goes in with the base board (from under it), the passive side's too
    bolts = [n for n in brackets["R"] if n.endswith("_screw1")]
    brackets["R"] = [n for n in brackets["R"] if n not in bolts]
    how = ("At each foot a printed bracket outside the outer plate, over its carriage bolt: "
           "an M5 button head through the bracket and the plate, its nut over the plate's "
           "inside (an 8 mm wrench, threadlocker: the bracket stays on its side); on the "
           "carriage bolt over the bracket's foot an 18-8 fender washer, then a printed thumb "
           "knob with an 18-8 M6 nut pressed up into its hex pocket (once, on the bench, in a "
           "vise), turned down by hand (snug: the foot is printed; no threadlocker, it comes "
           "off to pack). The foot's hole is 2.5 mm over the bolt: the side settles where the "
           "linkage puts it.")
    glue = ("Then its rail's two cleats: slide one against each face of the rail, a sheet of "
            "paper between (their play), and pencil round them on the board; lift the side "
            "off, glue the cleats on their marks (wood glue, a thin bead, weighted or clamped "
            "30 minutes) and set the side back between them, its knobs on again.")
    if base or feet or brackets["L"]:
        ops.append(Op(FINISH, (0,), whole(*base, *feet, *locator, *cleats["L"], *brackets["L"],
                                          *bolts), "Base board",
                      "Seal the base board's edges (the notch's above all: its end grain would "
                      "wick wine) and the jack locator's with a clear finish such as "
                      "polyurethane; oil the faces. The TPU feet pushed up into the board's "
                      "corner holes (a twist helps them through); the M6 carriage bolts up "
                      "through its square holes from under, each square neck in its hole (it "
                      "can't turn: nothing to reach under the board later; it hangs there "
                      "until its knob is on, so the knobs go back on whenever the cradle is "
                      "off). The "
                      "jack locator's two layers glued face to face, their outlines flush "
                      "(weighted 30 minutes); it drops into the notch, its ears in the notch's "
                      "bays. The drive side onto the base board, its outer plate's rail on it, "
                      "the notch at the front. " + how + " " + glue, "base"))
    if brackets["R"] or cleats["R"]:
        ops.append(Op(FINISH, (1,), whole(*cleats["R"], *brackets["R"]), "",
                      how + " " + glue + " Tighten the passive side's knobs last: turn the "
                      "crank through a whole pour and back to rest with them loose, then snug "
                      "them. Then turn it through a whole pour again: both cradle links must "
                      "still swing freely, the handle turning evenly; and the rod's paper check "
                      "again (the hand crank's step), now that the tie rods are held at both "
                      "ends.", "base_R"))
    if saddles:
        ops.append(Op(USE, (0,), (), "Setting it for a bottle",
                      "At rest (neck up), the bottle out, set all three by their scales. The "
                      "base stop: loosen its knob, slide the gate until its pointer's front "
                      "edge is on the bottle's groove across the rail or flat (a dot per "
                      "place in front of each, from the shortest: one a 750 ml, two a magnum, "
                      "three a 3 L, four a 6 L impériale, five a methuselah; another bottle: "
                      "its length, on the grooves every 10 mm, plus half a millimetre), and "
                      "turn the knob tight by hand. Each V block: turn its knob until the "
                      "block's reading edge is at the bottle's groove (the front block's top "
                      "edge on the scale inside the carrier's wall, in front of the knob: one "
                      "dot a 750 ml or a magnum, two a 3 L, three a 6 L; the rear block's "
                      "screw boss on the tower's face: one dot a 750 ml to five a methuselah, "
                      "whose V blocks sit on their rods; 9 turns end to end). Another bottle: "
                      "the other rows, its neck's diameter every 4 mm in front, measured 50 to "
                      "80 mm from its lip where the front V holds it (a Port bottle's bulb or "
                      "a Burgundy or Champagne magnum's tapered neck sets by that, not by the "
                      "lip), its body's every 10 mm at the rear. Lay the bottle in, its label "
                      "up, its base to the gate: it slides back onto it. The lip gauge stays "
                      "down; its pad stands just in front of the mouth, a millimetre or less "
                      "from the rim (more, and the gate or a knob is off). To uncork, lift "
                      "the gauge onto "
                      "its stop and hold it there, or let it rest on the corkscrew; let go "
                      "and it falls back onto the mouth. It must be down before the handle "
                      "turns: past level it holds the bottle forward, as the gate holds it "
                      "back. After the pour, wipe the neck and take the decanter away before "
                      "cranking back to rest (the front saddle's drains let out what runs "
                      "into it).", "setting", count=False))
    shown = [n for n in ("bottle", "decanter", "decanter_riser") if n in view.z]
    if shown:
        ops.append(Op(USE, (1,), whole(*shown), "Decanting",
                      "Not bought: crank the cradle to rest (neck up), set it for the bottle "
                      "and lay the bottle in (the step before); set a lab jack (100 x 100 mm) "
                      "into the jack locator's square, the decanter on it, and raise it until "
                      "the decanter's rim is a finger's width under the front saddle: that "
                      "one height clears the whole swing, so leave it there until the bottle "
                      "is back at rest. Switch the light on. Turn the "
                      "handle slowly, either way: the mouth stays where it is while the "
                      "bottle tips (rest to full tilt is 15 turns). Stop when the sediment "
                      "reaches the shoulder over the light; let go and it holds. " + USE_LIMITS,
                      "use", count=False))
    key = view.meta.get("cradle_linkage")
    if key in PACK_POSES:
        from spiderpig import linkage

        lk = linkage.get(key)
        link = cradle_link(lk)
        pins = sorted(j for j in lk.links[link][0]           # (shared with another link)
                      if any(j in js for b, (js, _) in lk.links.items() if b != link))
        breakdown, reassembly = module_steps(lk, pins)
        ops.append(Op(USE, (2,), (), "Taking it apart for the trip",
                      "Four pieces and the hand crank, no tool: the base board, the two "
                      "side panels and the cradle box. "
                      + " ".join(f"{i}. {text}" for i, (text, _) in
                                 enumerate(breakdown, 1)), "pack", count=False))
        ops.append(Op(USE, (3,), (), "Putting it back together",
                      " ".join(f"{i}. {text}" for i, text in enumerate(reassembly, 1)),
                      "unpack", count=False))
    return ops


SIDES_LR = ("L", "R")
USE_LIMITS = (
    "Before you use it: give it 950 mm of headroom (a 6 L's base at full tilt) and 100 mm "
    "behind the base board (the base stop's rails reach past its rear edge). On a counter "
    "that isn't flat the TPU feet take up about 1 mm; if it rocks, print a thicker foot for "
    "the corner that lifts. Check the four thumb knobs are snug by hand before each use; one "
    "that works loose again has let its bracket's printed foot creep under the washer: "
    "snug it again, and if it keeps loosening reprint that bracket in PETG. "
    "The decanter: set the jack's height once, at rest, and leave it for the whole swing: "
    "its rim no higher than 380 mm over the table (250 mm across up to 165 mm, 125 mm "
    "across above that; the cradle page gives each decanter's height). Never raise it "
    "during a pour: on the way back to rest the worm would drive the cradle box down onto "
    "the glass, and the handle would only feel a little stiff. "
    "While the handle turns, keep fingers clear of where it pinches: both side stacks, their "
    "bay faces included (each time the cradle comes back to rest the cradle links shear past "
    "the inner bars' front ends, at the bay's edge); the crank and the rod's end, which turn "
    "in the open above and behind the gearbox, right in front of the hand crank. (At rest the "
    "base stop comes down to 26 mm over the base board behind the decanter, a finger's 25 mm "
    "by ISO 13854: keep the board clear there.) A trapped finger: the worm holds "
    "whatever it has caught, so turn the handle back the way it came; never pull. When you "
    "leave it, take the handle off (its thumb screw): nothing turns without it. "
    "Never force the handle: if it stiffens, stop and look. Its clutch clicks round past "
    "what its bench setting says (27 N at the grip on B's 125 mm crank, 40 on A's 85; 10 % "
    "more at most), so a hard push can't break the drive: a click means something is "
    "in the way (with nothing in the way, a full 6 L started from a stop near level: turn "
    "back a little and on, steadily). It doesn't protect what is caught: in the last two or "
    "three handle turns before rest and before full tilt the cradle comes in with a force no "
    "clutch bounds, so nothing may be under the cradle box or the bottle there. Letting go "
    "stops the bottle, "
    "not the stream: the wine already over the lip runs on; to stop it, turn back about a "
    "turn. The lip gauge falls back onto the mouth by itself: check it is down before the "
    "bottle passes level, nothing holding it up (a corkscrew left in the cork included); with "
    "it up, a wet bottle slides forward at full tilt, its mouth off the pivot over the "
    "decanter. Uncork at rest, the base stop set: lift the lip gauge onto its stop and hold "
    "it there, or let it rest on the corkscrew, holding the neck, the corkscrew's lever lifted "
    "straight up; let go and the gauge falls back onto the mouth. Not a sparkling wine under "
    "pressure: at rest the mouth points 50° up toward the front; ease such a cork out by "
    "hand before the bottle goes in, the mouth pointed away from people. Keep your hands off "
    "the decanter while the handle turns (near rest the cradle box comes down over it, and "
    "the worm doesn't stop for a hand). Lift the cradle by its base board, never by the hand "
    "crank or the cradle box. Hand-wash the printed parts (PLA softens at 55 °C).")
"""The guide's use limits (the room it needs, the vessel, where it pinches: round 1's stand
review, ``docs/agentlib/DECANTER.md``)."""


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


def pack_pose(lk) -> tuple[float, float, float] | None:
    """The crank angle (radians) cradle ``lk`` packs at (:data:`PACK_POSES`: its arm along the
    member), the bottle's tilt there (degrees) and the handle turns from rest to it (the
    shorter way: its sign the crank's; rest, the bottle's least tilt); ``None``: it packs
    flat. The arm's direction is the crank angle's own, so the pose is one crank angle."""
    from spiderpig.construction.handcrank import NRV040

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
    return t, math.degrees(math.atan2(dt[1], dt[0])), turn / (2.0 * math.pi) * NRV040.ratio


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


def module_steps(lk, pins) -> tuple[list[tuple[str, str]], list[str]]:
    """How cradle ``lk`` (:data:`PACK_POSES`) comes apart into its modules and goes back
    together, for ``cradle_pack.module_pieces`` and the guide (:func:`assembly`): the
    breakdown's steps, each ``(text, tag)`` (tags ``pose``, ``hand_crank``, ``base``,
    ``lay``, ``clips``, ``lift``, ``box_clips``, ``box``: the pieces each frees), and the
    reassembly's texts. ``pins``: the cradle link's pins on each side (B's X and Y).

    Laid on its drive side (the gearbox down), the passive side panel lifts straight up off
    the pins, level, its links unable to swing; then the cradle box lifts straight up off
    the drive side's pins (the pack's turn 1, 2026-10-14: on its own the box packs in about
    two thirds of the box it took on the drive side). Each pin of a side lying down is held
    by its TPU keeper (:data:`KEEPER`): its head is on its link's outer face
    (:mod:`construction.pivots.clevis`), under the link with nothing under it, and a bare pin
    drops out once its clip is out. Standing, the 6.8 kg drive module tipped onto its cradle
    box once the pins cleared, and the panel fell the other way (round 1's stand review);
    the box can't slide off a side that stands on the board (its bolts and cleats hold the
    side), and of the two sides only the drive side holds the box (the worm): the passive
    side's links are free, so the passive panel comes off first."""
    pose = PACK_POSES[lk.key]
    got = pack_pose(lk)
    assert got is not None
    _, tilt, turns = got
    names = " and ".join(pins)
    which = "Pin" + ("s " if len(pins) > 1 else " ") + names
    each = "each one's R-clip" if len(pins) > 1 else "its R-clip"
    way = _arm_way(lk, turns)
    nudge = (f", then nudge {pose.nudge or 'its link'} until pin {pins[1]} drops into its"
             if len(pins) > 1 else "")
    swing = (f", then turn the box about it and swing the drive side's {pose.nudge or 'link'} "
             f"until pin {pins[1]} drops into its" if len(pins) > 1 else "")
    breakdown = [
        (f"Crank it to the pack pose: from rest (the bottle neck up, where the cradle stops "
         f"coming back), turn the handle the way that swings the crank's arm {way}, "
         f"{_turns_words(turns)} turns, until the arm lies along {pose.words} (seen from "
         f"outside, the leg hides it). The bottle is then {abs(tilt):.0f}° neck up, and each "
         "side's cradle link clears that side's inner bar. The first time, mark the "
         f"pose for the way back with a paint pen: {pose.marks}.", "pose"),
        ("Hand crank: loosen its thumb screw by hand and pull it off the gearbox's input "
         "shaft; the screw goes back into its hub. Nothing turns without it.", "hand_crank"),
        ("Base board: undo the brackets' thumb knobs by hand and lift the cradle off by its "
         "two outer plates' top members, never by the cradle box (only the worm holds it). "
         "The carriage bolts stay in the board, their washers and knobs back on them.",
         "base"),
        ("Lay the cradle on its drive side on a folded towel, the gearbox down (it rests a "
         "little tilted on the gearbox and that side's brackets), the passive side up. Lying "
         "so, gravity runs along the pins: no link can swing.", "lay"),
        (f"{which} (the cradle link's, on the passive side): reach into the bay and pull "
         f"{each} by hand from under the passive side's cradle link. Each pin stays in the "
         "panel's link, standing on its head, its TPU keeper on it.", "clips"),
        ("Passive side panel: lift it straight up off the pins by its outer plate, level, "
         "the pins and their keepers coming with it, and push each pin's clip back through "
         "its hole; then set the panel down any way up. The drive side with the cradle box "
         "stays lying as it is, the worm holding it.", "lift"),
        (f"{which} on the drive side: pull {each} by hand, reaching in from the side "
         "between the drive side's cradle link and its inner plate (the cradle box is open "
         "at its sides). Each pin stays in its link: its head is under the link, and its TPU "
         "keeper on the link holds it.", "box_clips"),
        ("Cradle box: lift it straight up off the drive side's pins, level, by its top "
         "cradle link and a tie rod (about 3 kg), and set it down as it was, on its other "
         "cradle link. The pins and their keepers stay in the drive side's links: push each "
         "pin's clip back through its hole. The drive side stays lying as it is (its link "
         f"{pose.nudge or 'b5'} now turns free on its one pin, flat): so it goes in the box.",
         "box"),
    ]
    reassembly = [
        f"Lay the drive side on its drive side on a folded towel, the gearbox down, its pins "
        f"{names} standing in their links on their keepers, their clips out.",
        "Lower the cradle box onto it, level, its knobs' side up (its other cradle link "
        f"down): pin {pins[0]} into its hole first (look in from the side){swing} hole. Both "
        "clips in, from the side, all the way.",
        f"Lay the passive side panel down on its outer plate, its pins {names} held in its "
        "links by their clips, and set the links to the pack pose by the paint marks: "
        f"{pose.marks}. A strip of masking tape across each pair of marks holds them while "
        "the panel turns over.",
        "Turn the panel over and hold it level, its outer plate up: pull each pin's clip; "
        "each pin hangs in its link by its head, its keeper on it.",
        f"Lower the panel level onto the cradle link: pin {pins[0]} into its hole first "
        f"(look in through the bay){nudge}. Peel the tape off.",
        "Both R-clips in from under the cradle link, through the pins' holes, all the way: a "
        "pin without its clip is held by its keeper's grip alone.",
        "Stand the cradle up by its two outer plates' top members and set it on the base "
        "board: each rail between its two cleats, each bracket's foot over its carriage bolt "
        "(its hole is 2.5 mm oversize). The washers and the thumb knobs on: the drive side's "
        "snug by hand, the passive side's only started.",
        "The hand crank on, its thumb screw onto the input shaft's key.",
        "Turn a whole pour and back to rest with the passive side's knobs loose: that side "
        "settles where the linkage puts it. Then snug them, at rest.",
        "The first build's checks again, the bottle out: turn a whole pour and back; both "
        "cradle links swing freely and the handle turns evenly with a fingertip, under "
        f"{EMPTY_GRIP_N:g} N at the grip, with no stiff spot (stiffer, the two sides fight "
        "each other: loosen the passive side's knobs and turn again). Then the rod's paper "
        "check (the hand crank's step). All four clips in, the four knobs snug.",
    ]
    return breakdown, reassembly


FRONT_MOVES = ("front_saddle_vee", "front_saddle_liner", "front_saddle_nut")
REAR_MOVES = ("rear_saddle_vee", "rear_saddle_liner", "rear_saddle_nut",
              "rear_saddle_nut_set_screw", "base_stop_rail0", "base_stop_rail1",
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
    and the base board's top in the assembled cradle ``mech`` as drawn (its V blocks at their
    lowest): at rest, the lowest it comes over the swing, at least :data:`FINGER_GAP`."""
    by = {b.name: b for b in mech.bodies if b.part is not None}
    board = by["base_plate"].placed_part().bounding_box().max.Y
    return min(b.placed_part().bounding_box().min.Y for n, b in by.items()
               if n.startswith("base_stop")) - board


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
    support under the rear one's flats and bosses), the knobs and the brake washer as they
    stand (the rear carrier's tower: support under its arm), the V blocks' liners on their
    two edges (their apex up, TPU's 45° faces), the rails' liners flat, the rails on their
    outer faces, the lip gauge on its pad's face (support under its back knuckle and lug),
    the base stop's gate on its face, the light's clip on its tube's foot (upright, round),
    the brackets on their L (its corner in every layer), the base board's thumb knobs on
    their tops, the feet as they stand, the hand crank's hub standing on its end (its tube up)
    and its arm, grip and cap on their outer faces. (Review round 1, m4.)"""
    t = -math.degrees(tilt)
    out: dict[str, list[tuple[str, float]]] = {}
    carriers = {s.name for s in SADDLES}
    for n in names:
        if n.startswith("foot_bracket_") and n.endswith("_knob"):
            out[n] = [("X", -90.0)]          # on its top: the nut's pocket open upward
        elif n in carriers or n.endswith(("_knob", "_vee")) or n in ("led_clip",
                                                                    "rear_saddle_brake"):
            out[n] = [("Z", t), ("X", 90.0)]
        elif n in ("base_stop_rail0_liner", "base_stop_rail1_liner"):
            out[n] = [("Z", t), ("X", 45.0 if n == "base_stop_rail0_liner" else -45.0)]
        elif n.endswith("_liner"):
            out[n] = [("Z", t), ("X", -90.0)]
        elif n in ("base_stop_gate", "front_saddle_gauge"):
            out[n] = [("Z", t), ("Y", -90.0)]
        elif n in ("base_stop_rail0", "base_stop_rail1"):
            out[n] = [("Z", t)] if n == "base_stop_rail0" else [("Z", t), ("X", 180.0)]
        elif n.startswith("foot_bracket_") and n.count("_") <= 2:   # its L on the bed: the
            out[n] = [("Y", 90.0)]                                   # corner in every layer
        elif n.startswith("foot_boot_"):
            out[n] = [("X", 90.0)]
        elif n == "L.handle":               # its hub on the bed, the cone under the clutch's
            out[n] = [("Y", -90.0)]          # flange, the seats up (the shaft along +x)
        elif n in ("L.handle_arm", "L.handle_grip", "L.handle_cap"):
            out[n] = [("Y", 90.0)]           # the arm's outer face on the bed
    return out
