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
  neck, in a box guide; the rear one under the body, on a steel post and its lead screw.
  TPU liners line both Vs. At a 6 L methuselah's setting each V block sits on its tie rods.
* **The bottles** (:func:`setting`, :func:`seat`): a 750 ml to a 6 L, each with its mouth
  on the pivot and its axis level, set by the two knobs (each saddle's scale), the lip
  gauge and the base stop. The **lip gauge** flips down from the front saddle onto the top
  of the mouth's rim: it marks where the mouth goes and holds the bottle from sliding
  forward. The **base stop**'s gate rides the rear V block's top edges and the rails
  behind it, locked behind the bottle's base: it holds the bottle from sliding back.
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
from spiderpig.hardware.fasteners import screw
from spiderpig.mechanism import Body, Mechanism
from spiderpig.shapes import Cut, Shape3D, cut_holes, difference, disc, moved, pill, union
from spiderpig.stack import Claim, Disc, Layout, Placed, Unbuildable, body_class

LIP_HEIGHT = 550.0      # the bottle's mouth over the table (mm): at rest, 50° neck up, the
#                         base stop's rails' end (set for a 6 L) hangs about 514 mm under
#                         it, over the base board's top (17 mm up)
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
                      1.3, 174.0)
MAGNUM = BottleSize("magnum", "1.5 L Bordeaux magnum", 356.0, 49.5, 15.5, 85.0, 150.0,
                    2.6, 205.0)
JEROBOAM = BottleSize("jeroboam", "3 L Champagne jeroboam", 470.0, 70.5, 19.0, 105.0, 190.0,
                      5.9, 265.0)
IMPERIAL = BottleSize("imperial", "6 L Bordeaux impériale", 493.0, 82.3, 23.5, 115.0, 195.0,
                      9.1, 286.0)
METHUSELAH = BottleSize("methuselah", "6 L Champagne methuselah", 571.0, 87.0, 24.0, 128.0,
                        231.0, 11.1, 331.0)
SIZES = (STANDARD, MAGNUM, JEROBOAM, IMPERIAL, METHUSELAH)
"""What the cradle is built for (``docs/agentlib/DECANTER.md``, Requirements): a standard
750 ml Bordeaux (the Bordelaise moulds run about 295-310 x 74-77 mm; its neck to the
shoulder about 80 mm, the front V block's back end), a Bordeaux magnum (univerre: 356 x 99
mm, 1.04 kg empty), a Champagne jeroboam (Saverglass: 470 x 141 mm, 2.9 kg empty), and the
two 6 L shapes (2026-10-13, the user's "both"): a Bordeaux impériale (Saverglass
Bordelaise Tradition 600 cl, ref. 0714: 492.8 mm tall, 164.5 mm at the shoulder, 156.5 at
the base, 3.12 kg empty; univerre's NGBD06QFR 493.1 x 164.4, 2.95 kg) and a Champagne
methuselah, the largest and heaviest (Saverglass Standard 600 cl, ref. 1742: 571 mm tall,
174 mm at the shoulder, 170 at the base, 5.13 kg empty). Full mass and centre of mass
estimated (glass plus 0.75, 1.5, 3 or 6 kg of wine, the centre about 42 % of the length
from the base); the 6 L necks (radius, length to the shoulder) are estimates scaled from
the jeroboam's (no maker draws them): the front knob is set to the neck in hand."""


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
    # the body's V block, long enough to reach behind a 6 L's centre of mass (331 mm from
    # the mouth: lying level or tipped, the bottle bears on both saddles) and still under a
    # 750 ml's body (its base at 300); it rides a steel post and its lead screw at its ends
    Saddle("rear_saddle", ("Tr1", "Tr2"), (255.0, 380.0), -149.0, (255.0, 380.0), -130.9,
           _apex(DESIGN.r), -55.0, 72.0),
)

STOP_GAP = 0.5          # the base stop's face behind the bottle's base
STOP_RANGE = (285.0, 576.0)     # where the base stop's face can go (cradle x): inside the
#                                 rear V block (the 750 ml's at 300.5) to the rails' end
#                                 (the 6 L's at 571.5)


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
               # on the front leg G-Ff, the inner plate's leg over it down to 300 mm over
               # the table (a decanter wider than the bay passes under it)
               **{n: (-150.0 * f, (_FOOT_Y - 0.0) * f) for n, f in (("D1", 0.175),
                                                                     ("D2", 0.36))}},
        outer=(("G", "Ff"), ("G", "O"), ("Ff", "O"), ("O", "Fm"), ("O", "Fr"),
               ("Ff", "Fm"), ("Fm", "Fr"), ("G", "C1"), ("G", "C2"), ("C1", "C2"),
               ("C2", "D1")),
        inner=(("G", "C1"), ("G", "C2"), ("C1", "C2"), ("C2", "D1"), ("D1", "D2"),
               ("G", "D1")),
        columns=("C1", "C2", "D1", "D2"),
        feet=("Ff", "Fm", "Fr"),
        base=(-200.0, 482.0),
    ),
    # B: the arms' pivots O1, O2 hang straight under the mouth, the crank O 366 behind it
    # and low (its handle behind everything). The outer plate is two triangles on the
    # pivots' bar, O1-O over O2-O, a front leg from O2, a rear leg straight down from O and
    # a rail. The inner plate is the bar O1-O2: the tie rods pass behind it (the rear ones
    # never further round than 72° down, the bar at 90.5°) and the front ones above it.
    # Nothing crosses the bay: the decanter stands under the mouth, between the bars. The
    # base board ends 158 mm behind O (a full 6 L tips over its rear edge past 25°).
    "decanter_rcm": Stand(
        nodes={"Ff": (-110.0, _FOOT_Y), "Fr": (386.0, _FOOT_Y)},
        outer=(("O1", "O2"), ("O2", "Ff"), ("O2", "O"), ("O1", "O"), ("O", "Fr"),
               ("Ff", "Fr"), ("O2", "Fr")),
        inner=(("O1", "O2"),),
        feet=("Ff", "Fr"),
        base=(-160.0, 524.0),
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
        r = TIE_D / 2 + 0.3
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
                out.pad(key, xy(a), xy(b), stand.member_r)
        frame = topo.frame_bodies[0]
        hole = TIE_HOLE
        z0, z1 = build.z(0)[1], build.z(build.top)[0]      # between the plates
        for n in (*stand.base_ties, *stand.feet):
            out.cut(FRAME_OUTER, Cut(xy(n), hole))
        for n in stand.bay_ties:
            out.cut(FRAME_INNER, Cut(xy(n), hole))
        for n in stand.columns:
            p = xy(n)
            out.cut(FRAME_OUTER, Cut(p, hole))
            out.cut(FRAME_INNER, Cut(p, hole))
            out.bodies.append(hardware(f"stand_column_{n}", _tie(p, z0, z1), frame,
                                       fab="purchased", bom_key=_standoff(z1 - z0),
                                       color=TIE_COLOR))
            for side, z in ((-1, build.z(0)[0]), (1, build.z(build.top)[1])):
                out.bodies.append(hardware(f"stand_column_{n}_screw{'io'[side > 0]}",
                                           _screw(p, z, side), frame, fab="purchased",
                                           bom_key=M5.key(TIE_SCREW), color=TIE_COLOR))
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
        return [Op(STACK, (k, 0, "ties"), whole(link), "Cradle link and tie rods",
                   "The four tie rods onto the cradle link, an M5 button head "
                   "through the link from its leg side into each (threadlocker); the rods "
                   "stand up from its bay side. It goes onto the stack rods up, its bores "
                   "down over the steel and the pins it rides.", "cradle_link",
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


def _tie(xy, z0: float, z1: float) -> Shape3D:
    """A 10 mm steel standoff from z0 to z1, its ends tapped (modelled as M5 holes)."""
    return difference(disc(xy, TIE_D / 2, z0, z1),
                      *[disc(xy, 2.5, a, b)
                        for a, b in ((z0 - 1, z0 + TIE_THREAD), (z1 - TIE_THREAD, z1 + 1))])


def _screw(xy, face: float, side: int, length: float = TIE_SCREW) -> Shape3D:
    """An M5 button head bearing on ``face``, its head on the ``side`` (+1 over it, -1
    under it) and its shank into the standoff beyond."""
    head = disc(xy, M5.head_d / 2, *sorted((face, face + side * M5.head_h)))
    shank = disc(xy, M5.shank_d / 2, *sorted((face, face - side * length)))
    return union([head, shank])


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


def _vee_block(s: Saddle, block: Shape3D) -> tuple[Shape3D, Shape3D]:
    """A V block's V cut into ``block`` and the TPU liner lining it (cradle frame, at the
    block's lowest)."""
    x0, x1 = s.vee
    outer = _vee(x0, x1, s.apex)                 # the liner's back: the block's V
    body = _box(x0, x1, s.bottom, s.top, -150.0, 150.0)
    liner = difference(cast(Shape3D, body & outer),
                       _vee(x0, x1, s.apex + LINER * math.sqrt(2)))
    return difference(block, outer), liner


def _ticks(x0: float, x1: float, face: float, sign: int, ys, width: float = 0.5,
           depth: float = 0.8) -> list[Shape3D]:
    """A scale's grooves (cradle frame): at each of ``ys``, ``width`` tall, along x from
    ``x0`` to ``x1``, ``depth`` into a wall whose face is at z ``face`` (``sign``: into +z
    or -z)."""
    z0, z1 = sorted((face - sign * 0.01, face + sign * depth))
    return [_box(x0, x1, y - width / 2, y + width / 2, z0, z1) for y in ys]


def _marks(s: Saddle, r_design: float, radius_of, every: float, lo: float, hi: float
           ) -> tuple[list[float], list[float]]:
    """A saddle's scale (its V block's top edge at each mark, cradle y, mm): one mark per
    bottle of :data:`SIZES`, and one each ``every`` mm of diameter from ``lo`` to ``hi``
    (a bottle off the list: its neck's or its body's diameter)."""
    named = [s.top + (r_design - radius_of(z)) * math.sqrt(2) for z in SIZES]
    sizes = [lo + every * k for k in range(round((hi - lo) / every) + 1)]
    graded = [s.top + (r_design - d / 2) * math.sqrt(2) for d in sizes]
    keep = [y for y in graded if s.top - 1e-9 <= y <= s.top + s.travel + 1e-9]
    return named, keep


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
M6_HEX = (10.0, 5.0, 4.0)     # DIN 934 M6 nut and its tap bolt's head: across flats, nut, head
M8_HEX = (13.0, 5.3)          # the M8 tap bolt's head: across flats, height
FRONT_NUT_KEY, FRONT_JACK_KEY = "m6_nut", "m6_tap_bolt_60"     # (hardware.cradle_catalog)
COUPLER_KEY, REAR_JACK_KEY = "m8_coupling_nut_24", "m8_tap_bolt_150"
KNOB_R = (11.0, 12.0)         # the printed knobs' radii (front, rear)
GAUGE_PIVOT = (30.0, -41.0)   # the lip gauge's hinge axis along x (cradle y, z)
GAUGE_POST_X = (60.0, 68.0)   # its post on the carrier's -z wall
GAUGE_CHEEK_X = (53.0, 59.7)  # its cheek round the pin, in front of the post
GAUGE_PAD = (11.0, 7.0)       # its pad's bottom (over a cork's bore) and half-width
GAUGE_BAR = (33.0, 46.0)      # its arm and bar over the neck (a 6 L's top at 24): bottom, top


def _front_saddle(s: Saddle, ties: Mapping[str, tuple[float, float]]
                  ) -> tuple[Shape3D, list[Shape3D]]:
    """The front saddle's carrier (cradle frame): a floor on the two tie rods (an M3 set
    screw up from its underside onto each, in a printed pilot), walls round its V block
    (the block's guide; the front and back ones notched a millimetre under the lowest
    liner), a pocket in the +z wall for the block's tab and the thumbscrew's foot, the lip
    gauge's post on the -z wall, the scale's grooves on the +z wall's inside (the block's
    top edge reads them: one per bottle in front of the pocket, the neck's diameter every 4
    mm behind it). (The carrier, its set screws.)"""
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
    post = [_box(*GAUGE_POST_X, FRONT_TOP - 1.0, yp + 6.0, zp, -h - 0.3),
            _box(*GAUGE_POST_X, FRONT_TOP - 1.0, yp, zp - 4.5, zp),
            _rod_x(yp, zp, 4.5, *GAUGE_POST_X)]
    part = union([floor, *walls, *post])
    xj, zj = FRONT_JACK
    named, graded = _marks(s, DESIGN.r_neck, lambda z: z.r_neck, 4.0, 28.0, 48.0)
    cuts = [Box(30.0, 30.0, 2 * 60.0 + 2).rotate(BAxis.Z, 45).moved(   # over a decanter's
        Location((x0, s.y0, 0.0))),                                    # neck at rest
            _box(FRONT_TAB[0] - CLEAR, FRONT_TAB[1] + CLEAR, FRONT_FLOOR - 0.5,
                 FRONT_TOP + 1.0, h - 1.0, zj + M6_HEX[0] / 2 + 1.0 + CLEAR),
            _rod_y(xj, zj, 3.2, FRONT_HEAD_Y - FRONT_SCREW - 0.5, FRONT_FLOOR),
            _rod_x(yp, zp, M5_PILOT / 2, GAUGE_POST_X[0] - 0.1, GAUGE_POST_X[1] + 0.1),
            *_ticks(v0 + 1.0, FRONT_TAB[0] - CLEAR - 1.0, h, 1, named),
            *_ticks(FRONT_TAB[1] + CLEAR + 1.0, v1 - 1.0, h, 1, graded)]
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


def _front_vee(s: Saddle) -> tuple[Shape3D, Shape3D, Shape3D]:
    """The front V block at its lowest (cradle frame): the neck's V, sitting on the tie
    rods, its tab into the carrier's +z wall holding the thumbscrew's M6 nut (in a hex
    pocket from below: the block bears down on it). (The block, its liner, the nut.)"""
    x0, x1 = s.vee
    xj, zj = FRONT_JACK
    block = union([_box(x0, x1, s.bottom, s.top, -FRONT_HALF, FRONT_HALF),
                   _box(FRONT_TAB[0], FRONT_TAB[1], s.bottom, FRONT_TAB[2], FRONT_HALF - 0.5,
                        zj + M6_HEX[0] / 2 + 1.0)])
    block = difference(block, _hex_y(xj, zj, M6_HEX[0] + CLEAR, s.bottom - 1.0,
                                     FRONT_NUT_Y + M6_HEX[1] + 0.2),
                       _rod_y(xj, zj, 3.3, s.bottom - 1.0, FRONT_TAB[2] + 1.0))
    vee, liner = _vee_block(s, block)
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


def _gauge() -> tuple[Shape3D, Shape3D]:
    """The lip gauge, flipped down (cradle frame): a cheek on a pin along the axis through
    the front carrier's post (an M5 x 16 button head into the post's pilot, snug: it swings
    under its own weight), a bridge resting on the post's top (its down stop), an arm across
    over the neck and a bar forward over it to a pad in front of the mouth, its back face at
    the pivot's plane: the top of the mouth's rim bears on it, above the cork's bore. It
    flips up and out (over the -z wall) to lay a bottle in or to uncork. (The gauge, its
    pin.)"""
    yp, zp = GAUGE_PIVOT
    xb = GAUGE_POST_X[1] + 0.5
    cheek = union([_rod_x(yp, zp, 6.5, *GAUGE_CHEEK_X),
                   _box(*GAUGE_CHEEK_X, yp, yp + 6.5, zp - 6.5, zp + 6.5)])
    bottom, half = GAUGE_PAD
    y0, y1 = GAUGE_BAR
    gauge = union([cheek,
                   _box(GAUGE_CHEEK_X[0], xb, yp + 6.5, y1, zp - 6.5, -30.0),     # bridge
                   _box(GAUGE_CHEEK_X[0], xb, y0, y1, -30.0, half),               # arm
                   _box(-10.0, xb, y0, y1, -half, half),                          # bar
                   _box(-10.0, -0.2, bottom, y1, -half, half)])                   # pad
    gauge = difference(gauge, _rod_x(yp, zp, TIE_HOLE / 2, GAUGE_CHEEK_X[0] - 1.0,
                                     GAUGE_CHEEK_X[1] + 1.0))
    x = GAUGE_CHEEK_X[0]
    pin = union([_rod_x(yp, zp, M5.head_d / 2, x - M5.head_h, x),
                 _rod_x(yp, zp, 2.4, x, x + GAUGE_PIN)])
    return gauge, pin


GAUGE_PIN = 16.0           # M5 x 16: through the cheek, 9 mm into the post's pilot


# --- the rear saddle: a V block on a steel post and its lead screw -----------------------

REAR_GUIDE = (317.5, 103.0)   # the post (-z) and the lead screw (+z): (x, |z|)
REAR_FOOT_Z = (88.5, 117.5)   # the carrier's two side blocks, each side
REAR_FOOT_TOP = -121.0        # their tops
REAR_BOSS = (12.0, -120.5, -57.0)   # the V block's bosses round the post and the screw:
#                                     radius, bottom, top (at its lowest)
REAR_FLANGE = (-67.0, -55.0, 96.0)  # its flats along its top: bottom, top, half-width
VEE_T = 9.0                   # its V shell, square to the V
REAR_SCREW = 150.0            # the lead screw: an M8 x 150 hex screw, fully threaded
REAR_HEAD_Y = 18.0            # its head's underside (the knob over the bosses' highest)
COUPLER = (13.0, 24.0, -110.0)      # the M8 coupling nut in the +z boss: across flats,
#                                     length, its underside (at the lowest)
POST = (130.0, -141.0)        # the post: a NETRF12 standoff (12 mm, M6 ends), its length
#                               and foot (12 mm: it takes the roll of a load hung at the screw)
POST_D = 12.0
M6 = screw("bhcs", "6")
POST_SCREW = 12.0             # M6 x 12 up into it from under the carrier, 6.7 mm in it
SEAT_R, SEAT_DEPTH = TIE_D / 2 + 0.15, 2.0  # the rear V block's seats on the tie rods
GUSSET = (45.0, -90.0)        # under its flats beside each boss: along x, its bottom
SCALE_X = (331.0, 337.0)      # the rear scale's bar, beside the screw's boss
GROOVE = (77.5, 82.5, 3.0)    # the base stop's guide grooves along the flats and the rails:
#                               |z| span, depth
PILOT_PITCH = 8.0             # the base stop's lock pilots along the +z groove
PILOT_X0 = 288.0              # the first


def _rear_saddle(s: Saddle, ties: Mapping[str, tuple[float, float]]
                 ) -> tuple[Shape3D, list[Shape3D]]:
    """The rear saddle's carrier (cradle frame): two side blocks on the two tie rods, joined
    under the V block by a bridge; the -z one holds the steel post (in a socket, an M5
    button head up into it from below, its head sunk), the +z one the lead screw's foot (a
    blind bore it turns in, the block's load on it) and the scale's bar (one groove per
    bottle on its +z half, the body's diameter every 10 mm on the other: the V block's
    boss's top edge reads them); an M3 set screw up from the +z block onto each rod. (The
    carrier, its set screws.)"""
    x0, x1 = s.x
    xg, zg = REAR_GUIDE
    za, zb = REAR_FOOT_Z
    top = REAR_BOSS[2] + s.travel + 1.0
    part = union([_box(x0, x1, s.y0, REAR_FOOT_TOP, za, zb),
                  _box(x0, x1, s.y0, REAR_FOOT_TOP, -zb, -za),
                  _box(xg - 7.5, xg + 7.5, s.y0, s.y0 + 10.0, -za - 0.5, za + 0.5),
                  _box(*SCALE_X, REAR_FOOT_TOP - 1.0, top, zg - 3.0, zg + 9.0)])
    named, graded = _marks(s, DESIGN.r, lambda z: z.r, 10.0, 70.0, 170.0)
    shift = REAR_BOSS[2] - s.top            # (the boss's top edge reads, not the flats')
    cuts = [_rod_y(xg, -zg, POST_D / 2 + 0.15, POST[1], REAR_FOOT_TOP + 1.0),
            _rod_y(xg, -zg, M6.head_d / 2 + 0.3, s.y0 - 1.0, s.y0 + M6.head_h + 0.25),
            _rod_y(xg, -zg, 3.3, s.y0, POST[1] + 0.5),
            _rod_y(xg, zg, 4.2, REAR_HEAD_Y - REAR_SCREW - 0.5, REAR_FOOT_TOP + 1.0),
            *[_box(SCALE_X[0] - 0.01, SCALE_X[0] + 0.8, y + shift - 0.25, y + shift + 0.25,
                   zg - 3.0, zg + 3.0) for y in named],
            *[_box(SCALE_X[0] - 0.01, SCALE_X[0] + 0.8, y + shift - 0.25, y + shift + 0.25,
                   zg + 3.0, zg + 9.0) for y in graded]]
    screws = []
    for n in s.ties:
        x, y = ties[n]
        cuts.append(Cylinder(TIE_D / 2 + 0.15, 2 * zb + 4).moved(Location((x, y, 0.0))))
        under = y - TIE_D / 2
        if under - SET_SCREW - 0.05 < s.y0:
            raise ValueError(f"{s.name}: an M3 x {SET_SCREW:g} set screw doesn't fit under "
                             f"tie {n} ({under - s.y0:.1f} mm)")
        cuts.append(_rod_y(x, zg, SET_PILOT / 2, s.y0 - 2.0, under))
        screws.append(_rod_y(x, zg, 1.25, under - 0.05 - SET_SCREW, under - 0.05))
    return difference(part, *cuts), screws


def _lock_pilots(x0: float, x1: float) -> list[float]:
    """The base stop's lock pilots' x along the +z groove from ``x0`` to ``x1``."""
    k0 = math.ceil((x0 - PILOT_X0) / PILOT_PITCH - 1e-9)
    k1 = math.floor((x1 - PILOT_X0) / PILOT_PITCH + 1e-9)
    return [PILOT_X0 + PILOT_PITCH * k for k in range(k0, k1 + 1)]


def _rear_vee(s: Saddle, ties: Mapping[str, tuple[float, float]]
              ) -> tuple[Shape3D, Shape3D, Shape3D]:
    """The rear V block at its lowest (cradle frame): a V shell (:data:`VEE_T`) sitting on
    the tie rods in a seat over each, its flats along the top (the base stop's rails, a
    guide groove along each, the lock's pilots along the +z one, the rails' two screws'
    pilots under them at its back), a boss at each end round the post (-z, a sliding fit)
    and the lead screw (+z, the M8 coupling nut in a hex pocket from below: the block bears
    down on it), a gusset under the flats beside each (they carry the roll of a load at the
    axis held at the screw). (The block, its liner, the coupling nut.)"""
    x0, x1 = s.vee
    xg, zg = REAR_GUIDE
    lo, hi, w = REAR_FLANGE
    shell = cast(Shape3D, _box(x0, x1, s.bottom, hi, -w, w)
                 & _vee(x0, x1, s.apex - VEE_T * math.sqrt(2)))
    edge = s.apex - VEE_T * math.sqrt(2) - lo           # (where the shell meets the flats)
    gussets = [_prism_x([(sign * w, lo + 0.01), (sign * w, GUSSET[1]),
                         (sign * (zg - REAR_BOSS[0]), GUSSET[1]), (sign * -edge, lo + 0.01)],
                        xg - GUSSET[0] / 2, xg + GUSSET[0] / 2) for sign in (-1, 1)]
    seats = [_box(ties[n][0] - 12.0, ties[n][0] + 12.0, ties[n][1] + TIE_D / 2 - SEAT_DEPTH,
                  s.bottom + 6.0, -12.0, 12.0) for n in s.ties]
    block = union([shell, _box(x0, x1, lo, hi, -w, w), *gussets, *seats,
                   *[_rod_y(xg, z, REAR_BOSS[0], REAR_BOSS[1], REAR_BOSS[2])
                     for z in (-zg, zg)]])
    af, length, y_nut = COUPLER
    g0, g1, depth = GROOVE
    cuts = [_rod_y(xg, -zg, POST_D / 2 + 0.2, REAR_BOSS[1] - 1.0, REAR_BOSS[2] + 1.0),
            *[Cylinder(SEAT_R, 40.0).moved(Location((*ties[n], 0.0))) for n in s.ties],
            _rod_y(xg, zg, 4.3, REAR_BOSS[1] - 1.0, REAR_BOSS[2] + 1.0),
            _hex_y(xg, zg, af + CLEAR, REAR_BOSS[1] - 1.0, y_nut + length + 0.2),
            _box(x0 - 1.0, x1 + 1.0, hi - depth, hi + 1.0, g0, g1),
            _box(x0 - 1.0, x1 + 1.0, hi - depth, hi + 1.0, -g1, -g0),
            *[_rod_y(x, (g0 + g1) / 2, M5_PILOT / 2, hi - depth - 8.5, hi - depth + 0.5)
              for x in _lock_pilots(x0 + 5.0, x1 - 4.0)],
            *[_rod_y(sum(LAP_X) / 2, z, M5_PILOT / 2, lo - 1.0, lo + 10.5)
              for z in (-LAP_Z, LAP_Z)]]
    vee, liner = _vee_block(s, difference(block, *cuts))
    nut = difference(_hex_y(xg, zg, af, y_nut, y_nut + length),
                     _rod_y(xg, zg, 4.0, y_nut - 1.0, y_nut + length + 1.0))
    return vee, liner, nut


def _post_y(xz: tuple[float, float], y0: float, y1: float) -> Shape3D:
    """A 12 mm steel standoff up y from ``y0`` to ``y1``, its ends tapped M6."""
    x, z = xz
    return difference(_rod_y(x, z, POST_D / 2, y0, y1),
                      *[_rod_y(x, z, 3.0, a, b) for a, b in ((y0 - 1, y0 + 12.0),
                                                             (y1 - 12.0, y1 + 1))])


# --- the base stop: a gate on the rear V block's flats and the rails behind it -----------

RAIL_END = 615.0              # the rails' end (cradle x): the 6 L's gate locked short of it
RAIL_Z = (70.9, 90.0)         # the rails across: from the V's top edge out
LAP_X = (366.0, 380.0)        # the rails' tongues under the V block's flats
LAP_Z = 87.0                  # their screws (M5 x 16 up into the flats)
GATE_T = 10.0                 # the gate's plate along the axis
GATE_BOTTOM = -92.0           # its bottom edge (at the lowest): 5 mm under a 6 L's base
EAR_END = 28.0                # its ears on the rails, behind its face
SLOT = (12.0, 26.0)           # the lock's slot in its +z ear, behind its face


def _rails(s: Saddle) -> tuple[Shape3D, list[Shape3D]]:
    """The base stop's rails (cradle frame, the V block at its lowest): two bars continuing
    the rear V block's flats behind it, its guide grooves and the lock's pilots, a cross
    bar at their end, each bar's tongue under the block's flats and an M5 button head up
    through it into the block. (The rails, their two screws.)"""
    lo, hi, _ = REAR_FLANGE
    g0, g1, depth = GROOVE
    za, zb = RAIL_Z
    x0 = s.vee[1]
    part = union([_box(x0, RAIL_END, lo, hi, za, zb), _box(x0, RAIL_END, lo, hi, -zb, -za),
                  _box(LAP_X[0], x0 + 10.0, lo - 6.0, lo + 0.01, za + 1.0, LAP_Z + 5.0),
                  _box(LAP_X[0], x0 + 10.0, lo - 6.0, lo + 0.01, -LAP_Z - 5.0, -za - 1.0),
                  _box(RAIL_END - 10.0, RAIL_END, lo, hi, -zb, zb)])
    xl = sum(LAP_X) / 2
    cuts = [_box(x0 - 1.0, RAIL_END - 10.0, hi - depth, hi + 1.0, g0, g1),
            _box(x0 - 1.0, RAIL_END - 10.0, hi - depth, hi + 1.0, -g1, -g0),
            *[_rod_y(x, (g0 + g1) / 2, M5_PILOT / 2, hi - depth - 8.5, hi - depth + 0.5)
              for x in _lock_pilots(x0 + 4.0, RAIL_END - 14.0)],
            *[_rod_y(xl, z, TIE_HOLE / 2, lo - 7.0, lo + 1.0) for z in (-LAP_Z, LAP_Z)]]
    screws = [union([_rod_y(xl, z, M5.head_d / 2, lo - 6.0 - M5.head_h, lo - 6.0),
                     _rod_y(xl, z, 2.4, lo - 6.0, lo - 6.0 + RAIL_SCREW)])
              for z in (-LAP_Z, LAP_Z)]
    return difference(part, *cuts), screws


RAIL_SCREW = 16.0          # M5 x 16: through the tongue, 10 mm into the flats


def _gate(s: Saddle, stop: float) -> tuple[Shape3D, Shape3D]:
    """The base stop's gate, its face at ``stop`` (cradle frame, the V block at its lowest):
    a plate across the V behind the bottle's base (a millimetre inside the liner's face,
    from :data:`GATE_BOTTOM` up), its top on the flats or the rails, an ear on each behind
    it keyed into its groove; the +z ear's slot over a pilot, an M5 button head through it
    (it clamps the gate there; should it slip, the slot's end stops it within 8 mm). (The
    gate, its lock.)"""
    hi = REAR_FLANGE[1]
    g0, g1, depth = GROOVE
    face = s.apex + LINER * math.sqrt(2)

    def zf(y: float) -> float:
        return y - face - 1.0

    top = hi + 6.0
    zy = [(-zf(GATE_BOTTOM), GATE_BOTTOM), (zf(GATE_BOTTOM), GATE_BOTTOM), (zf(hi), hi),
          (RAIL_Z[1] - 2.0, hi), (RAIL_Z[1] - 2.0, top), (2.0 - RAIL_Z[1], top),
          (2.0 - RAIL_Z[1], hi), (-zf(hi), hi)]
    ears = [_box(stop + GATE_T - 0.01, stop + EAR_END, hi, top, za, zb)
            for za, zb in ((72.0, RAIL_Z[1] - 2.0), (2.0 - RAIL_Z[1], -72.0))]
    keys = [_box(stop, stop + EAR_END, hi - depth + CLEAR, hi + 0.01, za, zb)
            for za, zb in ((g0 + CLEAR, g1 - CLEAR), (-g1 + CLEAR, -g0 - CLEAR))]
    zl = (g0 + g1) / 2
    r = TIE_HOLE / 2
    slot = union([_box(stop + SLOT[0] + r, stop + SLOT[1] - r, hi - depth - 1.0, top + 1.0,
                       zl - r, zl + r),
                  *[_rod_y(stop + x, zl, r, hi - depth - 1.0, top + 1.0)
                    for x in (SLOT[0] + r, SLOT[1] - r)]])
    gate = difference(union([_prism_x(zy, stop, stop + GATE_T), *ears, *keys]), slot)
    x = min(_lock_pilots(stop + SLOT[0] + r, stop + SLOT[1] - r),
            key=lambda p: abs(p - stop - sum(SLOT) / 2))
    lock = union([_rod_y(x, zl, M5.head_d / 2, top, top + M5.head_h),
                  _rod_y(x, zl, 2.4, top - LOCK_SCREW, top)])
    return gate, lock


LOCK_SCREW = 16.0          # M5 x 16: through the ear and its key, 7 mm into the pilot


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

DRIVE_PARTS = ("gearbox", "riser_disc", "crank_", "crankpin", "handle")
"""The hand crank's parts by name (:mod:`construction.handcrank`): the drive side's only."""

GROUPS = {"drive": ("gearbox", "riser_disc", "handle"), "crank": ("conn",),
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
    bodies: list[Body] = []
    connections = []
    for s, mech in (("L", side), ("R", passive)):
        mirror = s == "R"
        bodies.extend(Body(
            name=prefixed(b.name, s), part=_moved(b.part, -z_mid, mirror), joints=b.joints,
            color=b.color, pose=b.pose, outline=b.outline,
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
    xg, zg = REAR_GUIDE

    def put(name: str, part, fab: str, color: str, key: str | None = None,
            dx: float = 0.0, dy: float = 0.0) -> None:
        if dx or dy:
            part = moved(part, Location((dx, dy, 0.0)))
        extras.append(hardware(name, _place(part, cf.lip, cf.tilt), host, fab=fab,
                               bom_key=key, color=color))

    for s, (carrier, sets), (vee, liner, nut), (jack, knob), du, keys in (
            (front, _front_saddle(front, cf.ties), _front_vee(front),
             _jack(FRONT_JACK, 6.0, FRONT_SCREW, (M6_HEX[0], M6_HEX[2]), FRONT_HEAD_Y,
                   KNOB_R[0]), at.front, (FRONT_NUT_KEY, FRONT_JACK_KEY)),
            (rear, _rear_saddle(rear, cf.ties), _rear_vee(rear, cf.ties),
             _jack(REAR_GUIDE, 8.0, REAR_SCREW, M8_HEX, REAR_HEAD_Y, KNOB_R[1]), at.rear,
             (COUPLER_KEY, REAR_JACK_KEY))):
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
    # the rear V block's post, screwed up into from under the carrier
    y_head = rear.y0 + 0.25
    put("rear_saddle_post", _post_y((xg, -zg), POST[1], POST[1] + POST[0]), "purchased",
        TIE_COLOR, _standoff12(POST[0]))
    put("rear_saddle_post_screw",
        union([_rod_y(xg, -zg, M6.head_d / 2, y_head, y_head + M6.head_h),
               _rod_y(xg, -zg, 2.9, y_head + M6.head_h, y_head + M6.head_h + POST_SCREW)]),
        "purchased", TIE_COLOR, M6.key(POST_SCREW))
    fastened += [("rear_saddle_post", "rear_saddle"), ("rear_saddle_post", "rear_saddle_vee"),
                 ("rear_saddle_post_screw", "rear_saddle"),
                 ("rear_saddle_post_screw", "rear_saddle_post")]
    # the lip gauge, flipped down onto the mouth
    gauge, pin = _gauge()
    put("front_saddle_gauge", gauge, "printed", KNOB_COLOR)
    put("front_saddle_gauge_pin", pin, "purchased", TIE_COLOR, M5.key(GAUGE_PIN))
    fastened += [("front_saddle_gauge_pin", "front_saddle_gauge"),
                 ("front_saddle_gauge_pin", "front_saddle")]
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
    # the base stop: its rails on the rear V block, its gate set behind the drawn bottle
    rails, rail_screws = _rails(rear)
    gate, lock = _gate(rear, at.stop)
    put("base_stop_rails", rails, "printed", SADDLE_COLOR, dy=at.rear)
    for i, part in enumerate(rail_screws):
        put(f"base_stop_screw{i}", part, "purchased", TIE_COLOR, M5.key(RAIL_SCREW),
            dy=at.rear)
        fastened += [(f"base_stop_screw{i}", "base_stop_rails"),
                     (f"base_stop_screw{i}", "rear_saddle_vee")]
    put("base_stop_gate", gate, "printed", SADDLE_COLOR, dy=at.rear)
    put("base_stop_lock", lock, "purchased", TIE_COLOR, M5.key(LOCK_SCREW), dy=at.rear)
    fastened += [("base_stop_rails", "rear_saddle_vee"), ("base_stop_gate", "base_stop_rails"),
                 ("base_stop_gate", "rear_saddle_vee"), ("base_stop_lock", "base_stop_gate"),
                 ("base_stop_lock", "base_stop_rails"), ("base_stop_lock", "rear_saddle_vee")]
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
                bottle=shown.name, cradle_linkage=lk.key)
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
    saddles += [n for n in ("rear_saddle_post", "rear_saddle_post_screw", "front_saddle_gauge",
                            "front_saddle_gauge_pin") if n in view.z]
    sets = view.named(r"\w+_saddle_set_screw_\w+")
    if saddles:
        ops.append(Op(CRADLE, (0,), whole(*saddles, *sets), "Saddles",
                      "On the bench, each saddle's V block into its carrier, its TPU liner "
                      "pressed into its V. The front one: its M6 nut up into the hex pocket "
                      "under its tab, the block down into the carrier's box, the M6 "
                      "thumbscrew down through the tab's nut into its foot's bore, the knob "
                      "pressed onto its head. The rear one: the steel post into the "
                      "carrier's socket, an M5 button head up into it from below "
                      "(threadlocker); the M8 coupling nut up into the block's other boss, the "
                      "block down over the post, the M8 thumbscrew down through the nut into "
                      "its foot, its knob on. The lip gauge onto the front carrier's post, an "
                      "M5 button head through its cheek into the post's pilot, snug: it must "
                      "flip under its own weight. Then the saddles slid onto the tie rods' "
                      "free ends, the front one (the small V) on the rods nearer the hinge, "
                      "the knobs to the passive side, centred across the bay; the M3 set "
                      "screws up from their undersides into the printed pilots, snug on the "
                      "rods (they cut their own thread: no more than snug). Turn each knob "
                      "down until its V block sits on the rods.", "saddles"))
    stop = [n for n in ("base_stop_rails", "base_stop_screw0", "base_stop_screw1",
                        "base_stop_gate", "base_stop_lock") if n in view.z]
    if stop:
        ops.append(Op(CRADLE, (2,), whole(*stop), "Base stop",
                      "The base stop's printed rails behind the rear V block, flush with its "
                      "flats, their tongues under the flats, an M5 button head up through each "
                      "into its pilot (snug); the gate onto the rails from their end, its keys "
                      "in the grooves, its face toward the saddle, its M5 button head through "
                      "its slot into a pilot. It is set for each bottle (the use's first "
                      "step).", "base_stop"))
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
                      "At rest (neck up), the bottle out and the lip gauge flipped up: turn "
                      "each knob until its V block's top edge is at the bottle's groove on "
                      "the scale beside it (the front one's on its carrier's wall, in front of "
                      "the knob; the rear one's on its bar, read by the screw's boss's top "
                      "edge). One groove each, from the top: a 750 ml, a magnum, a 3 L, a 6 L "
                      "impériale; a 6 L methuselah's V blocks sit on their rods. Another "
                      "bottle: the other row, its neck's diameter every 4 mm in front, its "
                      "body's every 10 mm at the rear. Lay the bottle in, its label up; flip "
                      "the gauge down onto the mouth and slide the bottle forward until the "
                      "top of its rim meets the gauge's pad; slide the base stop's gate up to "
                      "its base and turn its M5 button head tight through the slot into the "
                      "pilot under it. The gauge stays down while it pours: it holds the "
                      "bottle forward, the gate holds it back. Flip the gauge up to uncork or "
                      "to lift the bottle out.", "setting", count=False))
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
                      "Three modules and the hand crank, no tool. "
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
    "in the open above and behind the gearbox, right in front of the hand crank; and the base "
    "board behind the decanter, where at rest the base stop's rails come down to 18 mm over "
    "it, less than the 25 mm a finger needs (ISO 13854). A trapped finger: the worm holds "
    "whatever it has caught, so turn the handle back the way it came; never pull. When you "
    "leave it, take the handle off (its thumb screw): nothing turns without it. "
    "Never force the handle: if it stiffens, stop and look (the drive holds a firm 40 N push "
    "at the grip against a blocked cradle, no more; if it stays stiff after being forced, "
    "look for a bent pin between the rod and the cradle link). Letting go stops the bottle, "
    "not the stream: the wine already over the lip runs on; to stop it, turn back about a "
    "turn. Check the lip gauge is down before the bottle passes level: with it up, a wet "
    "bottle slides forward at full tilt, its mouth off the pivot over the decanter. Uncork "
    "at rest, the base stop set and the lip gauge up, holding the neck, a corkscrew's lever "
    "lifted straight up; then flip the gauge down onto the mouth. Not a sparkling wine under "
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
    ``lay``, ``clips``, ``lift``: the pieces each frees), and the reassembly's texts.
    ``pins``: the cradle link's pins on the passive side (B's X and Y).

    Laid on its drive side (the gearbox down), the passive side panel lifts straight up off
    the pins, level, its links unable to swing. Standing, the 6.8 kg drive module tipped
    onto its cradle box once the pins cleared, and the panel fell the other way (round 1's
    stand review). Not on the passive side, as that review had it: each quick-release pin's
    head is on its link's outer face (:mod:`construction.pivots.clevis`), so with that side
    down and its clip out the pin drops out of its link, nothing under its head."""
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
    breakdown = [
        (f"Crank it to the pack pose: from rest (the bottle neck up, where the cradle stops "
         f"coming back), turn the handle the way that swings the crank's arm {way}, "
         f"{_turns_words(turns)} turns, until the arm lies along {pose.words} (seen from "
         f"outside, the leg hides it). The bottle is then {abs(tilt):.0f}° neck up, and the "
         "passive side's cradle link clears that side's inner bar. The first time, mark the "
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
         "panel's link, standing on its head.", "clips"),
        ("Passive side panel: lift it straight up off the pins by its outer plate, level, "
         "the pins coming with it, and hold it level. The printed spacer that sat on each "
         "pin over the cradle link stays lying on the link: slip it back onto its pin's end "
         "from below and push the pin's clip through its hole; then set the panel down any "
         "way up. The drive side with the cradle box stays lying as it is, the worm holding "
         "it: so it goes in the box.", "lift"),
    ]
    reassembly = [
        "Lay the drive side with the cradle box on its drive side on a folded towel, the "
        "gearbox down, the passive cradle link up.",
        f"Lay the passive side panel down on its outer plate, its pins {names} held in its "
        "links by their clips, and set the links to the pack pose by the paint marks: "
        f"{pose.marks}. A strip of masking tape across each pair of marks holds them while "
        "the panel turns over.",
        "Turn the panel over and hold it level, its outer plate up: pull each pin's clip, "
        "take its spacer off the pin's end and set the spacer over its hole in the cradle "
        "link.",
        f"Lower the panel level onto the cradle link: pin {pins[0]} through its spacer into "
        f"its hole first (look in through the bay){nudge}. Peel the tape off.",
        "Both R-clips in from under the cradle link, through the pins' holes, all the way: a "
        "pin without its clip is held by nothing.",
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
        "check (the hand crank's step). Both clips in, the four knobs snug.",
    ]
    return breakdown, reassembly


FRONT_MOVES = ("front_saddle_vee", "front_saddle_liner", "front_saddle_nut")
REAR_MOVES = ("rear_saddle_vee", "rear_saddle_liner", "rear_saddle_nut", "base_stop_rails",
              "base_stop_screw0", "base_stop_screw1")
STOP_MOVES = ("base_stop_gate", "base_stop_lock")
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
    drawn pose, radians): the carriers as they stand, the V blocks and the liners on an end
    (the V's section on the bed: no overhang), the knobs on their faces, the lip gauge on
    its side, the base stop's rails on their top and its gate on its face, the light's clip
    on its tube's foot (upright, round), the brackets on their L (its corner in every
    layer), the base board's thumb knobs on their tops, the feet as they stand, the hand
    crank on its arm (its tube up)."""
    t = -math.degrees(tilt)
    out: dict[str, list[tuple[str, float]]] = {}
    carriers = {s.name for s in SADDLES}
    for n in names:
        if n in carriers or n.endswith("_knob") or n == "led_clip":
            out[n] = [("Z", t), ("X", 90.0)]
        elif n.endswith("_vee"):
            out[n] = [("Z", t), ("Y", 90.0)]
        elif n.endswith("_liner") or n == "base_stop_gate":
            out[n] = [("Z", t), ("Y", -90.0)]
        elif n == "front_saddle_gauge":
            out[n] = [("Z", t)]
        elif n == "base_stop_rails":
            out[n] = [("Z", t), ("X", -90.0)]
        elif n.startswith("foot_bracket_") and n.count("_") <= 2:   # its L on the bed: the
            out[n] = [("Y", 90.0)]                                   # corner in every layer
        elif n.startswith("foot_bracket_") and n.endswith("_knob"):
            out[n] = [("X", -90.0)]          # on its top: the nut's pocket open upward
        elif n.startswith("foot_boot_"):
            out[n] = [("X", 90.0)]
        elif n in ("L.handle", "L.handle_grip"):
            out[n] = [("Y", 90.0)]           # (the input shaft along +x: the arm down)
    return out
