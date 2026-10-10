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
  screwed through its cradle link from outside. Two printed saddles ride on them: the
  front one under the neck, its V for the neck and its back face the shoulder's stop; the
  rear one under the body, its V deep enough for a 3 L bottle. TPU liners line both Vs. An
  M3 set screw up from each saddle's underside, in a printed pilot, locks it on each tie.
* **The light**: an AAA keychain light standing in a clip printed with the front saddle,
  its head under the shoulder (the sommelier's candle; its head twists it on).
* **The bottles** (:func:`adapter`, :func:`seat`): a magnum on printed adapter liners, a
  3 L in a printed collar, each with its mouth on the pivot and its axis level.
* **The stand** (:data:`STANDS`, :class:`CradleStand`): each side's frame plates are the
  stand's members, not arms from O. The outer plate carries the hinge, the gearbox and the
  legs down to a rail. The inner plate holds the pillars and stiffens the side (A: a block
  round the hinge and a leg down the front, four columns; B: the bar O1-O2), so the tie
  rods pass it in the open.
* **The base plate** (:func:`_base_plate`): both sides' rails bolted onto it with printed
  brackets (:func:`_bracket`), the decanter in a notch from its front edge, printed TPU
  feet snapped into its corners.
* **Display only** (``fab="display"``: drawn, never bought, cut or printed): the bottle (a
  1.5 L Bordeaux magnum) and a decanter under the mouth.

The planner sees the side-level parts as claims: the ties' columns through the layers
between the cradle link and the inner plate (:class:`CradleTies`), the stand's columns and
base ties through every layer (:class:`CradleStand`). What it can't see, the tie rods
passing the inner plate's layer and the bottle in the bay, is measured on the assembled
cradle over the swing (:func:`construction.contract.clashes`, by
:mod:`spiderpig.cradle_report` and ``tests/test_cradle.py``).
"""

from __future__ import annotations

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
from spiderpig.hardware.fasteners import screw
from spiderpig.mechanism import Body, Mechanism
from spiderpig.shapes import Cut, Shape3D, cut_holes, difference, disc, moved, pill, union
from spiderpig.stack import Claim, Disc, Layout, Placed, Unbuildable, body_class

LIP_HEIGHT = 350.0      # the bottle's mouth over the table (mm): a decanter up to 310 mm tall
#                         stands under it, its lip 29 mm or more under a 3 L's at full tilt
DECANTER_GAP = 40.0     # the decanter's mouth at least this far under the bottle's
BAY = 160.0             # between the two inner plates' bay faces: a 141 mm bottle and room
TIE_D = 10.0            # the tie rods and columns: MISUMI NETRF10 (10 mm, M5 x 10 deep each end)
TIE_THREAD = 10.0
M5 = screw("bhcs", "5")
TIE_SCREW = 12.0        # M5 x 12 button heads through a 3.2-4.8 mm plate: 7.2-8.8 mm in the tie
TIE_HOLE = 5.5          # their clearance holes (ISO 273 medium)
M5_NUT = (8.0, 4.0)     # ISO 4032 M5 nut: across flats, height
LINER = 2.0             # the saddles' TPU liners
SADDLE_CLEAR = 3.0      # a saddle's end to an inner plate's bay face
FRAME_COLOR = "#eb6834"
SADDLE_COLOR = "#6a4fc7"
ADAPTER_COLOR = "#8a8a8a"
PARK_X = 170.0              # a spare bottle size's collar, lying on the table this far in
#                             front of the mouth (clear of the decanter and the front feet)
LINER_COLOR = "#2b2b2b"
TIE_COLOR = "#4a4a4a"
BOTTLE_COLOR = "#20402a"
DECANTER_COLOR = "#cfe3f5"
LED_COLOR = "#f0f0e0"


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


MAGNUM = BottleSize("magnum", "1.5 L Bordeaux magnum", 356.0, 49.5, 15.5, 85.0, 150.0,
                    2.6, 205.0)
JEROBOAM = BottleSize("jeroboam", "3 L Champagne jeroboam", 470.0, 70.5, 19.0, 105.0, 190.0,
                      5.9, 265.0)
SIZES = (MAGNUM, JEROBOAM)
"""What the cradle is built for (``docs/agentlib/DECANTER.md``, Requirements): a Bordeaux
magnum (univerre: 356 x 99 mm, 1.04 kg empty) to a Champagne jeroboam (Saverglass: 470 x
141 mm, 2.9 kg empty); full mass and centre of mass estimated (glass plus 1.5 or 3 kg of
wine, the centre about 42 % of the length from the base)."""


@dataclass(frozen=True)
class Saddle:
    """A printed saddle on two tie rods, in the cradle's frame (mm): ``x`` its span along
    the axis, ``y0`` its bottom, ``apex`` the V's apex (under its TPU liner), ``top`` the V's
    top edges."""

    name: str
    ties: tuple[str, str]
    x: tuple[float, float]
    y0: float
    apex: float
    top: float


def _apex(r: float) -> float:
    """The V's apex (90° included) under a radius ``r`` lying in it with its axis at y 0."""
    return -(r + LINER) * math.sqrt(2)


DESIGN = JEROBOAM
"""The bottle the saddles' Vs are sized for: it lies in them with its axis level through
the pivot. A thinner one lies on adapter liners (:func:`adapter`) that put its axis there
too."""
STOP_X = MAGNUM.neck
"""The front saddle's back face, the shoulder stop (cradle frame): the shortest neck's
length, so the magnum's mouth is on the pivot; a longer neck takes a collar round it
between the stop and its shoulder (:func:`adapter`)."""

SADDLES = (
    # the neck's V sized for the 3 L's neck; its back face the shoulder stop
    # (its underside 7 mm under the 10 mm tie rods: an M3 x 6 set screw up to each)
    Saddle("front_saddle", ("Tf1", "Tf2"), (45.0, STOP_X), -53.0,
           _apex(DESIGN.r_neck), -8.0),
    # the body's V sized for the 3 L
    Saddle("rear_saddle", ("Tr1", "Tr2"), (255.0, 315.0), -125.0, _apex(DESIGN.r), -45.0),
)


def adapter(size: BottleSize) -> tuple[float, float, float]:
    """What ``size`` needs to lie with its mouth on the pivot and its axis level: (the
    front adapter liner's wall, the rear's: what its neck and body are thinner than the
    3 L's; the collar round its neck between the stop and its shoulder: what its neck is
    longer than :data:`STOP_X`), mm. The magnum: the two liners; the 3 L: the collar. A
    builder with another bottle prints these to its own (``docs/agentlib/DECANTER.md``)."""
    return (DESIGN.r_neck - size.r_neck, DESIGN.r - size.r, size.neck - STOP_X)


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
    base: tuple[float, float] = (-200.0, 440.0)    # the base plate's front and rear edges


BASE_SHEET = "al5052_3p2mm"     # the base plate, both sides' outer plates bolted onto it
BASE_HALF = 250.0               # its half-width across the bay: a 500 mm track (a 30° tip
#                                 and a 26 N push at a bottle's top, the worst over the swing)
FOOT_PAD_H = 8.0                # its printed TPU feet, snapped into its corner holes
BASE_TOP_CLEAR = FOOT_PAD_H + 3.175 - 2.0   # what may hang under it: all but 2 mm of the feet
BASE_TOP = -LIP_HEIGHT + FOOT_PAD_H + 3.175     # its top, from the mouth
NOTCH_R = 126.0                 # the decanter's notch in it: a 244 mm decanter and play,
#                                 slid in from the front onto the table, the notch's round
#                                 end locating it under the mouth
DECANTER_X = 0.0                # the decanter's axis, under the mouth (a stream lands -15..+16 mm
#                                 from it over the pour: 2026-10-10 review round 2)
_FOOT_Y = BASE_TOP + 12.5       # a foot's centre: its screw's boss (2 t) on the base plate
BRACKET = (36.0, 30.0, 5.0)     # a foot's printed bracket: its length, legs' height, wall
"""A stand foot is a printed L-bracket outside each outer plate, an M5 button head through
it and the plate (its nut in the layer over the plate) and one down through it and the
base plate (its nut under the base plate, in the feet's height)."""

STANDS: dict[str, Stand] = {
    # A: the hinge G at the mouth, the crank O 173 behind and 202 under it. The outer plate
    # (outside every moving part's layer: any truss) from G down in front of the decanter
    # to the front foot and back to O, O's legs down and back, a rail on the base plate.
    # The inner plate is a small block round G, tied to the outer by two columns in front of
    # the hinge, where nothing of the cradle ever goes; nothing crosses the bay over the
    # mouth (the base plate and the cradle box hold the sides).
    "decanter_hinge": Stand(
        nodes={"Ff": (-150.0, _FOOT_Y), "Fm": (173.0, _FOOT_Y), "Fr": (380.0, _FOOT_Y),
               "C1": (-24.0, 20.0), "C2": (-24.0, -22.0),
               # on the front leg G-Ff, the inner plate's leg over it down to 200 mm over
               # the table (a decanter is 120 mm across there: it slides in under it)
               "D1": (-35.0, -76.3), "D2": (-69.0, -150.4)},
        outer=(("G", "Ff"), ("G", "O"), ("Ff", "O"), ("O", "Fm"), ("O", "Fr"),
               ("Ff", "Fm"), ("Fm", "Fr"), ("G", "C1"), ("G", "C2"), ("C1", "C2"),
               ("C2", "D1")),
        inner=(("G", "C1"), ("G", "C2"), ("C1", "C2"), ("C2", "D1"), ("D1", "D2"),
               ("G", "D1")),
        columns=("C1", "C2", "D1", "D2"),
        feet=("Ff", "Fm", "Fr"),
        base=(-200.0, 440.0),
    ),
    # B: the arms' pivots O1, O2 on a ray down and back from the mouth, the crank O behind
    # them. The inner plate is the bar O1-O2 (the tie rods pass above and in front of it).
    "decanter_rcm": Stand(
        nodes={"Ff": (105.0, _FOOT_Y), "Fr": (490.0, _FOOT_Y), "Bt": (172.4, -144.7)},
        outer=(("O1", "O2"), ("O2", "O"), ("O1", "Ff"), ("O2", "Ff"), ("O", "Fr"),
               ("Ff", "Fr"), ("O2", "Fr")),
        inner=(("O1", "O2"),),
        feet=("Ff", "Fr"),
        # on the O1-O2 bar, 25 mm from O1: under every part of the cradle's sweep
        bay_ties=("Bt",),
        base=(-200.0, 545.0),
    ),
}


def stand_of(lk) -> Stand:
    try:
        return STANDS[lk.key]
    except KeyError:
        raise ConstructionError(f"{lk.key} is a cradle with no stand (construction.cradle."
                                "STANDS)") from None


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
                   "stand up from its bay side.", "cradle_link", sub=True)]

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
            memo = (tuple(pads), tuple(cuts), z0, z1, web)
            part = _PLATE_MEMO.get(memo)
            if part is None:
                parts = [pill(a, b, r, z0, z1) for a, b, r in pads]
                parts += [disc(c.xy, c.d / 2 + web, z0, z1) for c in cuts
                          if isinstance(c, Cut) and web > 0]
                part = cut_holes(union(parts), cuts, z0, z1)
                if len(_PLATE_MEMO) > 16:
                    _PLATE_MEMO.clear()
                _PLATE_MEMO[memo] = part
            out.bodies.append(hardware(name, part, frame, fab="laser", color=FRAME_COLOR,
                                       sheet=build.ctx.sheet("frame")))
        return out


def _ties_clear_the_inner_plate(build: Build, done: Realized, ties: Mapping[str, str]):
    """Each tie rod passes the inner plate's layer in the plate's open, where the planner
    checks nothing (a frame plate's layer holds only what sits in its holes): its path over
    the whole turn must clear the plate's members and the bosses round its holes (as built
    so far: the stand's and the axles') by the plan's margin, or the rod would run into the
    plate."""
    pads = done.pads.get(FRAME_INNER, [])
    web = boss_web(build.ctx.sheet("frame"))
    holes = [c for c in done.cuts.get(FRAME_INNER, []) if isinstance(c, Cut)]
    reach = TIE_D / 2 + build.plan.spec.margin
    for tie, name in ties.items():
        path = np.asarray(build.plan.topo.geometry.points[name], float).reshape(-1, 2)
        for a, b, r in pads:
            a_, b_ = np.asarray(a[:2], float), np.asarray(b[:2], float)
            ab = b_ - a_
            u = np.clip(((path - a_) @ ab) / max(float(ab @ ab), 1e-18), 0.0, 1.0)
            gap = float(np.linalg.norm(path - (a_ + u[:, None] * ab), axis=1).min()) - r
            if gap < reach:
                raise ConstructionError(f"tie rod {tie} comes within {gap:.1f} mm of the inner "
                                        f"plate's member {a[:2]}-{b[:2]} over the turn (it "
                                        f"needs {reach:.1f})")
        for c in holes:
            gap = float(np.linalg.norm(path - np.asarray(c.xy[:2], float), axis=1).min())
            if gap - c.d / 2 - web < reach:
                raise ConstructionError(f"tie rod {tie} comes within {gap - c.d / 2 - web:.1f} "
                                        f"mm of the inner plate's boss at {c.xy[:2]} over the "
                                        f"turn (it needs {reach:.1f})")


# --- parts -----------------------------------------------------------------------------

def _standoff(length: float) -> str:
    """The catalog key of a tie rod's or column's NETRF10 standoff ``length`` mm long."""
    from spiderpig.hardware.cradle_catalog import netrf10

    return netrf10(round(length, 1))


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


def seat(size: BottleSize, adapted: bool = True) -> tuple[float, float, float]:
    """Where ``size`` lies in the saddles, on its adapter liners (``adapted``) or straight in
    the 3 L-sized ones: (its axis's y at the mouth's x, the axis's slope in radians, the
    mouth's x), in the cradle's frame."""
    front, rear = SADDLES
    a_f, a_r, collar = adapter(size) if adapted else (0.0, 0.0, 0.0)
    xf, xr = sum(front.x) / 2, sum(rear.x) / 2
    yf = front.apex + (size.r_neck + LINER + a_f) * math.sqrt(2)
    yr = rear.apex + (size.r + LINER + a_r) * math.sqrt(2)
    slope = math.atan2(yr - yf, xr - xf)
    x0 = front.x[1] + collar - size.neck     # its shoulder on the stop (or its collar)
    return yf + (x0 - xf) * math.tan(slope), slope, x0


SET_SCREW = 6.0            # the M3 cup-point set screw locking a saddle on each tie
SET_PILOT = 2.6            # its printed pilot (the screw cuts its own thread in PLA)


def _saddle(s: Saddle, ties: Mapping[str, tuple[float, float]]
            ) -> tuple[Shape3D, Shape3D, list[Shape3D]]:
    """A saddle, its TPU liner and its set screws, in the cradle's frame: a block across the
    bay, a 90° V cut along the axis (its apex ``s.apex`` under the liner), the tie holes,
    and up from its underside to each tie an M3 set screw in a printed pilot, its cup point
    on the tie (the saddle can't then slide along the ties)."""
    half = BAY / 2 - SADDLE_CLEAR
    x0, x1 = s.x
    block = _block(x0, x1, s.y0, s.top)
    outer = _vee(x0, x1, s.apex)             # the liner's back: the saddle's V
    inner = _vee(x0, x1, s.apex + LINER * math.sqrt(2))
    saddle = difference(block, outer)
    if s is SADDLES[0]:         # its front-bottom corner: over the decanter's neck at rest
        saddle = difference(saddle, Box(30.0, 30.0, 2 * half + 2).rotate(BAxis.Z, 45).moved(
            Location((x0, s.y0, 0.0))))
    liner = difference(cast(Shape3D, block & outer), inner)
    screws = []
    for n in s.ties:
        x, y = ties[n]
        hole = Cylinder(TIE_D / 2 + 0.15, 2 * half + 4).moved(Location((x, y, 0.0)))
        under = y - TIE_D / 2                    # the tie's underside
        if under - SET_SCREW - 0.05 < s.y0:
            raise ValueError(f"{s.name}: an M3 x {SET_SCREW:g} set screw doesn't fit under "
                             f"tie {n} ({under - s.y0:.1f} mm)")
        pilot = Cylinder(SET_PILOT / 2, under - s.y0 + 2.0, rotation=(90, 0, 0)).moved(
            Location((x, (under + s.y0 - 2.0) / 2, 0.0)))
        saddle = difference(saddle, hole, pilot)
        screws.append(Cylinder(1.25, SET_SCREW, rotation=(90, 0, 0)).moved(
            Location((x, under - 0.05 - SET_SCREW / 2, 0.0))))
    return saddle, liner, screws


def _block(x0: float, x1: float, y0: float, top: float) -> Shape3D:
    """A saddle's block across the bay (cradle frame), from ``y0`` to ``top``."""
    half = BAY / 2 - SADDLE_CLEAR
    return Box(x1 - x0, top - y0, 2 * half).moved(Location(((x0 + x1) / 2, (y0 + top) / 2, 0.0)))


def _vee(x0: float, x1: float, apex: float) -> Shape3D:
    """What lies over a 90° V along the axis from ``x0`` to ``x1``, its apex at ``apex``
    (cradle frame)."""
    size = 400.0
    return Box(x1 - x0 + 2, size, size).rotate(BAxis.X, 45).moved(
        Location(((x0 + x1) / 2, apex + size / math.sqrt(2), 0.0)))


def _adapter(s: Saddle, wall: float, size: BottleSize) -> Shape3D:
    """A thinner bottle's TPU adapter liner on a saddle's liner (cradle frame): a V shell
    ``wall`` thick, its walls up past where ``size`` touches them."""
    x0, x1 = s.x
    lo = s.apex + LINER * math.sqrt(2)                   # the liner's face
    r = size.r_neck if s is SADDLES[0] else size.r
    top = -r / math.sqrt(2) + 8.0                        # past its tangent points
    return difference(cast(Shape3D, _block(x0, x1, s.y0, top) & _vee(x0, x1, lo)),
                      _vee(x0, x1, lo + wall * math.sqrt(2)))


def _collar(size: BottleSize, length: float) -> Shape3D:
    """A longer-necked bottle's printed TPU collar (cradle frame, on its neck from the
    stop): a C round the neck ``length`` long, its opening narrower than the neck so it
    snaps on."""
    r_in, r_out = size.r_neck + 0.3, size.r_neck + 10.0
    ring = Cylinder(r_out, length, rotation=(0, 90, 0)) - Cylinder(
        r_in, length + 2, rotation=(0, 90, 0))
    gap = Box(length + 2, r_out, 1.6 * size.r_neck).moved(Location((0.0, r_out / 2 + 4, 0.0)))
    return moved(ring - gap, Location((STOP_X + length / 2, 0.0, 0.0)))


def _led(front: Saddle) -> tuple[Shape3D, Shape3D, Shape3D, list[Shape3D], list[Shape3D]]:
    """The penlight under the shoulder and its clip, in the cradle's frame: the light stands
    behind the shoulder stop, its head up under the shoulder; the clip, a part of its own
    (printed standing, its tube upright), holds its body below the head (which twists to
    switch it on) on a lip at the bottom, an M3 set screw through a boss on its back holding
    it from turning. A plate behind the saddle carries it, its flange under the saddle's
    flat underside (behind the front tie's set screw) held by two M5 button heads up into
    printed pilots. (The light, the clip, the set screw, the clip's screws, the saddle's
    pilots for them.)"""
    d, length = PENLIGHT
    x, head = LED_X, LED_TOP
    up = (90, 0, 0)                          # a cylinder's axis along y
    light = Cylinder(d / 2, length, rotation=up).moved(Location((x, head - length / 2, 0.0)))
    clip_top, clip_bot = head - CLIP_GAP, head - length + 2.0
    tube = Cylinder(d / 2 + 2.5, clip_top - clip_bot + 2.0, rotation=up).moved(
        Location((x, (clip_top + clip_bot - 2.0) / 2, 0.0)))
    tube = tube - Cylinder(d / 2 + 0.2, clip_top - clip_bot + 0.01, rotation=up).moved(
        Location((x, (clip_top + clip_bot) / 2 + 0.005, 0.0)))
    tube = tube - Cylinder(d / 2 - 2.0, 10.0, rotation=up).moved(     # the lip's hole
        Location((x, clip_bot - 2.0, 0.0)))
    boss_y = (clip_top + clip_bot) / 2
    boss = Box(SET_SCREW + 1.0, 8.0, 8.0).moved(
        Location((x + d / 2 + (SET_SCREW + 1.0) / 2 - 0.5, boss_y, 0.0)))
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


def _base_stop(rear: Saddle, size: BottleSize) -> tuple[Shape3D, Shape3D, list[Shape3D],
                                                        Shape3D, list[Shape3D]]:
    """The bottle's base stop, in the cradle's frame: a printed spine under the bottle from
    the rear saddle's back face (its flange there, two M5 button heads into the saddle's V
    walls under the liner), a row of pilots along it, and a printed cup sliding on it, its
    face a millimetre or two behind the base of the bottle ``size``, an M5 button head
    through its side into the pilot there. At rest the bottle lies 25° nose up and only the
    liners' friction held it (a corkscrew's push slid it back): the cup holds it, and it
    stays in the cradle whatever the drive does. (The spine, the cup, the flange's screws,
    the cup's screw, the saddle's pilots.)"""
    x1 = rear.x[1]
    y_top = -DESIGN.r - SPINE_UNDER                  # the spine's top, under a 3 L's body
    y_bot = y_top - SPINE_H
    half = SPINE_W / 2
    flange = Box(SPINE_FLANGE[0], -DESIGN.r - 2.0 - y_bot, SPINE_FLANGE[1]).moved(
        Location((x1 + SPINE_FLANGE[0] / 2, (-DESIGN.r - 2.0 + y_bot) / 2, 0.0)))
    r_end = (y_top - y_bot) / 2                      # its end rounded (it comes down low)
    bar = union([Box(SPINE_END - r_end - x1, y_top - y_bot, 2 * half).moved(
        Location(((x1 + SPINE_END - r_end) / 2, (y_top + y_bot) / 2, 0.0))),
        Cylinder(r_end, 2 * half).moved(Location((SPINE_END - r_end, (y_top + y_bot) / 2,
                                                  0.0)))])
    y_mid = (y_top + y_bot) / 2
    across = (0, 0, 0)                               # a cylinder's axis along z
    holes = [Cylinder(M5_PILOT / 2, half + 1.0, rotation=across).moved(
        Location((x, y_mid, half - (half + 1.0) / 2 + 0.5))) for x in STOP_HOLES]
    screws, pilots, bores = [], [], []
    along = (0, 90, 0)                               # a cylinder's axis along x
    for z in (-SPINE_SCREW_Z, SPINE_SCREW_Z):
        at = Location((x1, y_mid, z))
        screws.append(union([
            Cylinder(M5.head_d / 2, M5.head_h, rotation=along).moved(
                at * Location((SPINE_FLANGE[0] + M5.head_h / 2, 0, 0))),
            Cylinder(2.4, SPINE_SCREW, rotation=along).moved(
                at * Location((SPINE_FLANGE[0] - SPINE_SCREW / 2, 0, 0)))]))
        bite = SPINE_SCREW - SPINE_FLANGE[0]
        pilots.append(Cylinder(M5_PILOT / 2, bite + 1.0, rotation=along).moved(
            at * Location((-(bite + 1.0) / 2 + 0.01, 0, 0))))
        bores.append(Cylinder(TIE_HOLE / 2, SPINE_FLANGE[0] + 2, rotation=along).moved(
            at * Location((SPINE_FLANGE[0] / 2, 0, 0))))
    spine = difference(union([flange, bar]), *holes, *bores)
    # the cup: a channel round the spine and the stop over it, its face at x_c, its two
    # holes 5 mm apart so the row's 10 mm pitch sets it in 5 mm steps
    base = size.length
    hole, face = next((h, h + d) for h in STOP_HOLES for d in CUP_HOLES if h + d >= base + 1.0)
    w = CUP_WALL
    chan = difference(
        Box(CUP_LEN + CUP_STOP, y_top - y_bot + 0.4 + 2 * w, 2 * (half + 0.2 + w)).moved(
            Location((face - (CUP_LEN - CUP_STOP) / 2, y_mid, 0.0))),
        Box(CUP_LEN + CUP_STOP + 2, y_top - y_bot + 0.4, 2 * (half + 0.2)).moved(
            Location((face - (CUP_LEN - CUP_STOP) / 2, y_mid, 0.0))))
    stop = Box(CUP_STOP, CUP_TOP - (y_top + 0.2 + w), 2 * CUP_HALF).moved(
        Location((face + CUP_STOP / 2, (CUP_TOP + y_top + 0.2 + w) / 2, 0.0)))
    cup_holes = [Cylinder(TIE_HOLE / 2, w + 2, rotation=across).moved(
        Location((face - d, y_mid, half + 0.2 + w / 2))) for d in CUP_HOLES]
    cup = difference(union([chan, stop]), *cup_holes)
    lock = union([Cylinder(M5.head_d / 2, M5.head_h, rotation=across).moved(
        Location((hole, y_mid, half + 0.2 + w + M5.head_h / 2))),
        Cylinder(2.4, CUP_SCREW, rotation=across).moved(
        Location((hole, y_mid, half + 0.2 + w - CUP_SCREW / 2)))])
    return spine, cup, screws, lock, pilots


SPINE_W, SPINE_H = 30.0, 20.0   # the base stop's printed spine: across, deep
SPINE_UNDER = 8.0               # its top under a 3 L's body (in the saddles' Vs)
SPINE_END = JEROBOAM.length + 10.0     # it ends at the longest bottle's cup (at rest it comes
#                                         down to about 40 mm over the base plate)
SPINE_FLANGE = (6.0, 64.0)      # its flange on the rear saddle's back face: thick, across
SPINE_SCREW_Z = 22.0            # the flange's screws, into the saddle's V walls under the liner
SPINE_SCREW = 16.0              # M5 x 16: 10 mm in the saddle
STOP_HOLES = tuple(float(x) for x in range(340, 471, 10))   # the spine's pilots (cradle x)
CUP_HOLES = (12.0, 17.0)        # the cup's two holes, this far in front of its face
CUP_LEN, CUP_STOP, CUP_WALL = 25.0, 6.0, 3.0   # its channel, its stop plate, its walls
CUP_TOP = -30.0                 # the stop's top (cradle y): over both bottles' lower rims
CUP_HALF = 30.0                 # its half-width across
CUP_SCREW = 12.0                # M5 x 12 through its wall into the spine's pilot


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
             ) -> tuple[Shape3D, list[tuple[Shape3D, str]], tuple[float, float]]:
    """A foot's printed L-bracket outside an outer plate (world): its upright leg on the
    plate's outer face (``face``; ``sign`` outward along z), its foot leg on the base plate
    (its top ``y_top``); an M5 button head through the upright and the plate into a nut over
    the plate, one up from under the base plate through the foot into a nut on it (its head
    in the feet's clearance, its nut in reach from above). (The bracket, its screws and nuts
    with their catalog keys, the base plate's hole (x, z).)"""
    length, height, wall = BRACKET
    fx, fy = xy
    zs = sorted((face, face + sign * wall))
    up = Box(length, height, wall).moved(Location((fx, y_top + height / 2, sum(zs) / 2)))
    zf = sorted((face, face + sign * height))
    foot = Box(length, wall, height).moved(Location((fx, y_top + wall / 2, sum(zf) / 2)))
    zb = face + sign * (height - 11.0)               # the foot leg's screw
    hole_up = disc((fx, fy), TIE_HOLE / 2, zs[0] - 1, zs[1] + 1)
    hole_down = Cylinder(CARRIAGE[0] / 2 + 0.4, wall + 2, rotation=(90, 0, 0)).moved(
        Location((fx, y_top + wall / 2, zb)))
    neck = Cylinder(CARRIAGE_NECK_HOLE / 2, 2 * (CARRIAGE[3] - 3.175), rotation=(90, 0, 0)).moved(
        Location((fx, y_top, zb)))              # (the bolt's square neck stands this far up)
    bracket = difference(union([up, foot]), hole_up, hole_down, neck)
    # through the upright and the plate, the nut over the plate's inner face
    l_side = _m5_length(wall + t_frame + M5_NUT[1])
    inner = face - sign * t_frame
    nut_z = sorted((inner, inner - sign * M5_NUT[1]))
    side_screw = _screw((fx, fy), face + sign * wall, sign, l_side)
    side_nut = _nut((fx, fy), *nut_z)
    # a carriage bolt up from under the base plate (its square neck in the plate's square
    # hole: it can't turn), through the foot, its nut on the foot from above
    d, length, head_d, _, head_h = CARRIAGE
    y_under = y_top - 3.175
    if y_under - head_h < y_top - BASE_TOP_CLEAR:
        raise ValueError("the base plate's carriage bolts' heads reach the table")
    if 3.175 + wall + 3.2 > length:
        raise ValueError("the carriage bolt doesn't reach through the foot and its nut")
    up = (90, 0, 0)
    head = Cylinder(head_d / 2, head_h, rotation=up).moved(
        Location((fx, y_under - head_h / 2, zb)))
    shank = Cylinder(d / 2 * 0.97, length, rotation=up).moved(
        Location((fx, y_under + length / 2, zb)))
    y_nut = y_top + wall
    nut = Cylinder(5.0, 3.2, rotation=up).moved(Location((fx, y_nut + 1.6, zb))) - Cylinder(
        d / 2, 5.2, rotation=up).moved(Location((fx, y_nut + 1.6, zb)))
    return bracket, [(side_screw, M5.key(l_side)), (side_nut, "m5_nut"),
                     (union([head, shank]), "m6_carriage_16"), (nut, "m6_thin_nut")], (fx, zb)


CARRIAGE = (6.0, 16.0, 16.55, 4.6, 3.88)   # the base plate's bolts (DIN 603 M6 x 16, Bolt
#                                            Depot 18219): d, length, head d, neck h, head h
CARRIAGE_SQUARE = 7.0      # the base plate's square hole for its 6.48 mm neck (SendCutSend
#                            rounds inside corners 0.8 mm: a 7 mm square clears its corners)
CARRIAGE_NECK_HOLE = 9.6   # the bracket foot's relief over the neck's 6.48 mm square


def _m5_length(grip: float) -> float:
    """The shortest stock M5 button head through ``grip`` mm with a turn of thread to spare."""
    return next(n for n in M5.lengths if n >= grip + 0.5)


def _nut(xy, z0: float, z1: float) -> Shape3D:
    """An M5 nut along z (drawn round, its across-flats)."""
    return difference(disc(xy, M5_NUT[0] / 2, z0, z1), disc(xy, 2.5, z0 - 1, z1 + 1))


def _base_plate(lip, stand: Stand, y_top: float, holes: list[tuple[float, float]]
                ) -> tuple[Shape3D, list[tuple[float, float]]]:
    """The base plate (world): from the stand's front to its rear edge, :data:`BASE_HALF`
    each side of the bay's middle, under both sides' outer plates; a notch from its front
    edge round the decanter (its round end under the mouth locating it), the brackets'
    screw holes, a hole in each corner for a TPU foot. (The plate, the feet's (x, z).)"""
    t = 3.175
    x0, x1 = float(lip[0]) + stand.base[0], float(lip[0]) + stand.base[1]
    plate = Box(x1 - x0, t, 2 * BASE_HALF).moved(Location(((x0 + x1) / 2, y_top - t / 2, 0.0)))
    xd = float(lip[0]) + DECANTER_X
    up = (90, 0, 0)
    notch = union([Box(xd - x0 + 1.0, t + 2, 2 * NOTCH_R).moved(
                       Location(((x0 - 1.0 + xd) / 2, y_top - t / 2, 0.0))),
                   Cylinder(NOTCH_R, t + 2, rotation=up).moved(Location((xd, y_top - t / 2,
                                                                          0.0)))])
    inset = 25.0
    feet = [(x, z) for x in (x0 + inset, x1 - inset) for z in (-BASE_HALF + inset,
                                                                BASE_HALF - inset)]
    cuts = [Box(CARRIAGE_SQUARE, t + 2, CARRIAGE_SQUARE).moved(Location((x, y_top - t / 2, z)))
            for x, z in holes]          # the brackets' carriage bolts' square necks
    cuts += [Cylinder(FOOT_STEM / 2 + 0.25, t + 2, rotation=up).moved(
        Location((x, y_top - t / 2, z))) for x, z in feet]
    return difference(plate, notch, *cuts), feet


FOOT_STEM = 10.0            # a TPU foot's stem through the base plate's corner hole


def _tpu_foot(x: float, z: float, y_top: float) -> Shape3D:
    """A printed TPU foot (world): a pad under the base plate's corner, a stem up through
    its hole and a bulb over it that snaps it in: a cone (a 30° lead-in) on a ledge, the
    stem bored up the middle so it squeezes through the hole."""
    from build123d import Cone

    up = (-90, 0, 0)                         # a solid's axis along +y
    t = 3.175
    pad = Cylinder(12.0, FOOT_PAD_H, rotation=up).moved(
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

STUDIES = {"decanter_rcm": "a design study, not the one to build (docs/agentlib/DECANTER.md)"}
"""The cradles that are studies (their outputs say so)."""

DRIVE_PARTS = ("gearbox", "riser_disc", "crank_", "crankpin", "handle")
"""The hand crank's parts by name (:mod:`construction.handcrank`): the drive side's only."""

GROUPS = {"drive": ("gearbox", "riser_disc", "handle"), "crank": ("conn",),
          "box": ("tie_", "front_saddle", "rear_saddle", "led", "jeroboam_collar", "base_stop"),
          "stand": ("base_plate", "foot_")}
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
    # the saddles, their liners, the light, the bottle: drawn in the cradle's frame
    light, clip, led_screw, clip_screws, clip_pilots = _led(SADDLES[0])
    shown = MAGNUM                          # the bottle drawn: a magnum on its adapters
    spine, cup, spine_screws, cup_lock, spine_pilots = _base_stop(SADDLES[1], shown)
    a_f, a_r, _ = adapter(shown)
    for s in SADDLES:
        saddle, liner, screws = _saddle(s, cf.ties)
        if s is SADDLES[0]:                 # the light's clip's screws' pilots
            saddle = difference(saddle, *clip_pilots)
        else:                               # the base stop's spine's
            saddle = difference(saddle, *spine_pilots)
        extras.append(hardware(s.name, _place(saddle, cf.lip, cf.tilt), host, fab="printed",
                               color=SADDLE_COLOR))
        wall = a_f if s is SADDLES[0] else a_r
        if wall > 0:
            name = f"{s.name}_magnum_liner"
            part = _adapter(s, wall, shown)
            extras.append(hardware(name, _place(part, cf.lip, cf.tilt), host, fab="printed",
                                   color=ADAPTER_COLOR))
            fastened += [(name, f"{s.name}_liner"), ("bottle", name)]
        extras.append(hardware(f"{s.name}_liner", _place(liner, cf.lip, cf.tilt), host,
                               fab="printed", color=LINER_COLOR))
        # a set screw up from its underside holds it on each tie
        fastened.extend((s.name, f"tie_{n}") for n in s.ties)
        for n, set_screw in zip(s.ties, screws, strict=True):
            name = f"{s.name}_set_screw_{n}"
            extras.append(hardware(name, _place(set_screw, cf.lip, cf.tilt), host,
                                   fab="purchased", bom_key=f"m3_set_screw_{SET_SCREW:g}",
                                   color=TIE_COLOR))
            fastened += [(name, s.name), (name, f"tie_{n}")]
    extras.append(hardware("led", _place(light, cf.lip, cf.tilt), host, fab="purchased",
                           bom_key="penlight_aaa", color=LED_COLOR))
    extras.append(hardware("led_set_screw", _place(led_screw, cf.lip, cf.tilt), host,
                           fab="purchased", bom_key=f"m3_set_screw_{SET_SCREW:g}",
                           color=TIE_COLOR))
    extras.append(hardware("led_clip", _place(clip, cf.lip, cf.tilt), host, fab="printed",
                           color=SADDLE_COLOR))
    # the base stop: its spine on the rear saddle, its cup set behind the magnum's base
    extras.append(hardware("base_stop_spine", _place(spine, cf.lip, cf.tilt), host,
                           fab="printed", color=SADDLE_COLOR))
    extras.append(hardware("base_stop_cup", _place(cup, cf.lip, cf.tilt), host,
                           fab="printed", color=SADDLE_COLOR))
    for i, part in enumerate(spine_screws):
        extras.append(hardware(f"base_stop_screw{i}", _place(part, cf.lip, cf.tilt), host,
                               fab="purchased", bom_key=M5.key(SPINE_SCREW), color=TIE_COLOR))
    extras.append(hardware("base_stop_lock", _place(cup_lock, cf.lip, cf.tilt), host,
                           fab="purchased", bom_key=M5.key(CUP_SCREW), color=TIE_COLOR))
    for i, part in enumerate(clip_screws):
        extras.append(hardware(f"led_clip_screw{i}", _place(part, cf.lip, cf.tilt), host,
                               fab="purchased", bom_key=CLIP_SCREW, color=TIE_COLOR))
    # the base plate, its TPU feet, and each foot's bracket bolting an outer plate onto it
    y_top = float(cf.lip[1]) + BASE_TOP
    t_frame = plan.t(0)
    base_holes = []
    for s, sign in (("L", -1), ("R", 1)):
        face = sign * (z_mid - plan.z(0)[0])          # the outer plate's outer face
        host_s = prefixed(topo.frame_bodies[0], s)
        for n in stand.feet:
            xy = tuple(float(v) for v in _node_xy(build.xy, lk, topo, stand, n))
            bracket, screws, hole = _bracket(xy, y_top, face, sign, t_frame)
            base_holes.append(hole)
            extras.append(hardware(f"foot_bracket_{n}{s}", bracket, host_s, fab="printed",
                                   color=SADDLE_COLOR))
            for i, (part, key) in enumerate(screws):
                extras.append(hardware(f"foot_bracket_{n}{s}_{'screw' if i % 2 == 0 else 'nut'}"
                                       f"{i // 2}", part, host_s, fab="purchased",
                                       bom_key=key, color=TIE_COLOR))
    plate_part, foot_xz = _base_plate(cf.lip, stand, y_top, base_holes)
    extras.append(hardware("base_plate", plate_part, frame_host, fab="laser",
                           sheet=BASE_SHEET, color="#9aa3ad"))
    for i, (x, z) in enumerate(foot_xz):
        extras.append(hardware(f"foot_boot_{i}", _tpu_foot(x, z, y_top), frame_host,
                               fab="printed", color=LINER_COLOR))
    y0, slope, x0 = seat(shown)
    extras.append(hardware("bottle", _place(_bottle(shown, y0, slope, x0), cf.lip, cf.tilt),
                           host, fab="display", color=BOTTLE_COLOR))
    # the 3 L's neck collar, not on the magnum shown: drawn lying on the table in front
    for size in SIZES:
        length = adapter(size)[2]
        if size is not shown and length > 0:
            ring = _collar(size, length).moved(Location((-STOP_X - length / 2, 0.0, 0.0)))
            ring = ring.rotate(BAxis.Z, 90).moved(Location(
                (float(cf.lip[0]) - PARK_X, float(cf.lip[1]) - LIP_HEIGHT + length / 2, 0.0)))
            extras.append(hardware(f"{size.key}_collar", ring, frame_host, fab="printed",
                                   color=ADAPTER_COLOR))
    extras.append(hardware("decanter", _decanter(cf.lip), frame_host, fab="display",
                           color=DECANTER_COLOR))
    fastened += [("bottle", s.name + "_liner") for s in SADDLES]
    fastened += [("led_clip", SADDLES[0].name), ("led", "led_clip"),
                 ("led_set_screw", "led_clip"), ("led_set_screw", "led")]
    fastened += [(f"led_clip_screw{i}", n) for i in range(len(clip_screws))
                 for n in ("led_clip", SADDLES[0].name)]
    fastened += [("base_stop_spine", SADDLES[1].name), ("base_stop_cup", "base_stop_spine"),
                 ("base_stop_lock", "base_stop_cup"), ("base_stop_lock", "base_stop_spine")]
    fastened += [(f"base_stop_screw{i}", n) for i in range(len(spine_screws))
                 for n in ("base_stop_spine", SADDLES[1].name)]
    meta = dict(side.meta, cradle=True, cradle_layer=plan.layers[cradle_link(lk)],
                print_poses=print_poses([b.name for b in extras + bodies], cf.tilt),
                cradle_lip=[float(cf.lip[0]), float(cf.lip[1])], cradle_tilt=float(cf.tilt),
                mid_plane=z_mid, bay_mm=BAY, lip_height_mm=LIP_HEIGHT,
                tie_mm=tie_len, base_tie_mm=foot_len, bay_tie_mm=bay_len, fastened=fastened,
                bottle=shown.name)
    return Mechanism(name=f"{side.name}_cradle", bodies=bodies + extras,
                     connections=connections, meta=meta,
                     bom_extras=list(side.bom_extras) + list(passive.bom_extras))


def _decanter(lip) -> Shape3D:
    """A large-format decanter under the mouth (display): a conical neck to a wide body on
    the table, its mouth 30 mm under the bottle's (a bottle's neck at rest, 10° nose up,
    hangs 20 mm under its mouth)."""
    top = float(lip[1]) - DECANTER_GAP
    table = float(lip[1]) - LIP_HEIGHT
    prof = [(0, table), (95, table), (122, table + 20), (118, table + 70), (26, top - 40),
            (24, top), (0, top)]
    face = Face(Wire.make_polygon([(x, y, 0.0) for x, y in prof], close=True))
    solid = Solid.revolve(face, 360.0, BAxis.Y)
    return moved(solid, Location((float(lip[0]) - 5.0, 0.0, 0.0)))


def assembly(view) -> list:
    """How the cradle box goes on (:mod:`construction.assembly`, ``CRADLE_ORDER``; ``view``
    a ``RobotView``): the tie rods with the drive side's cradle link (a bench sub-assembly
    of the drive side's stack), the bay tie with the drive side's inner plate; the base
    plate, its feet and the drive side's brackets; the saddles slid onto the rods, their
    set screws, the light, the base stop; the passive side's cradle link onto the rods and
    its screws; the passive side's brackets; and the display bottle and decanter, the
    use."""
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
    saddles = [n for s in SADDLES for n in (s.name, f"{s.name}_liner")
               if n in view.z]
    adapters = [n for s in SADDLES if (n := f"{s.name}_magnum_liner") in view.z]
    sets = view.named(r"\w+_saddle_set_screw_\w+")
    if saddles:
        ops.append(Op(CRADLE, (0,), whole(*saddles, *sets), "Saddles",
                      "Each saddle's TPU liner pressed into its V; the saddles slid onto the "
                      "tie rods' free ends, the front one (the small V) on the rods nearer "
                      "the hinge, centred across the bay; the M3 set screws up from their "
                      "undersides into the printed pilots, snug on the rods (they cut their "
                      "own thread: no more than snug).", "saddles"))
    stop = [n for n in ("base_stop_spine", "base_stop_screw0", "base_stop_screw1",
                        "base_stop_cup", "base_stop_lock") if n in view.z]
    if stop:
        ops.append(Op(CRADLE, (2,), whole(*stop), "Base stop",
                      "The base stop's printed spine against the rear saddle's back face, "
                      "under the V, its flange's two M5 button heads into the saddle's pilots "
                      "(snug); the cup slid onto the spine from its end, its stop toward the "
                      "saddle. Set it for the bottle in use: lay the bottle in with its "
                      "shoulder on the stop (on its collar, for a 3 L), slide the cup up to "
                      "its base and back to the first hole that lines up (the cup has two, "
                      "5 mm apart), and the M5 button head through it into the spine. The "
                      "bottle can't then slide back off the pivot when you uncork it.",
                      "base_stop"))
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
    brackets = {s: view.named(rf"foot_bracket_[A-Za-z0-9]+{s}(_\w+)?") for s in SIDES_LR}
    how = ("At each foot a printed bracket outside the outer plate, over its carriage bolt: "
           "an M5 button head through the bracket and the plate, its nut over the plate's "
           "inside (an 8 mm wrench); the carriage bolt's M6 thin nut on the bracket's foot "
           "(a 10 mm wrench; threadlocker on both).")
    if base or feet or brackets["L"]:
        ops.append(Op(FINISH, (0,), whole(*base, *feet, *brackets["L"]), "Base plate",
                      "The TPU feet pushed up into the base plate's corner holes (a twist "
                      "helps them through); the six M6 carriage bolts up through its square "
                      "holes from under (their necks hold them: nothing to reach under the "
                      "plate later). The drive side onto the base plate, its outer plate's "
                      "rail on it, the notch at the front. " + how, "base"))
    if brackets["R"]:
        ops.append(Op(FINISH, (1,), whole(*brackets["R"]), "", how, "base_R"))
    collars = view.named(r"\w+_collar")
    if adapters or collars:
        ops.append(Op(USE, (0,), whole(*adapters, *collars), "Bottle adapters",
                      "A magnum lies on its two TPU adapter liners, laid in the saddles' "
                      "liners, its shoulder against the front saddle's back face. A 3 L "
                      "lies in the liners alone, its TPU collar snapped round its neck "
                      "against that face. Either way its mouth is on the pivot and its axis "
                      "level. Another bottle: print adapters to its neck and body "
                      "(construction.cradle.adapter).", "adapters"))
    shown = [n for n in ("bottle", "decanter") if n in view.z]
    if shown:
        ops.append(Op(USE, (1,), whole(*shown), "Decanting",
                      "Not bought: crank the cradle to rest (neck up), set the base stop for "
                      "the bottle, lay the bottle in the saddles, its shoulder against the "
                      "front saddle's back face and its label up; stand the decanter in the "
                      "base plate's notch, under the mouth; switch the light on. Turn the "
                      "handle slowly, either way: the mouth stays where it is while the "
                      "bottle tips (rest to full tilt is 15 turns). Stop when the sediment "
                      "reaches the shoulder over the light; let go and it holds. " + USE_LIMITS,
                      "use", count=False))
    return ops


SIDES_LR = ("L", "R")
USE_LIMITS = (
    "Before you use it: give it 800 mm of headroom (a 3 L's base at full tilt) and 50 mm "
    "behind the base plate (a 3 L reaches past its rear edge); keep the base plate clear "
    "behind the decanter (at rest the base stop comes down to 40 mm over it). A decanter fits "
    "up to 180 mm tall at 260 mm across, 190 mm across up to 200 mm, 90 mm across up to 270 "
    "mm (the light's clip at rest) and 56 mm across (its neck) up to 310 mm, its mouth 55 mm "
    "across or more. Never force the handle: if it stiffens, stop and look (the drive holds a "
    "firm 40 N push at the grip against a blocked cradle, no more; if it stays stiff after "
    "being forced, look for a bent pin between the rod and the cradle link). Keep your "
    "fingers out of the drive side's plates while it turns. Uncork at rest, the base stop "
    "set, holding the neck, a corkscrew's lever lifted straight up; then slide the bottle "
    "forward onto its stop. Lift the cradle by its base plate, never by the hand crank. "
    "Hand-wash the printed parts (PLA softens at 55 °C).")
"""The guide's use limits (the room it needs: ``docs/agentlib/DECANTER.md``)."""


def jeroboam_hits(mech) -> list[str]:
    """The parts a 3 L bottle lying in the saddles (on their liners, in its collar, without
    the magnum's adapters) would meet in the assembled cradle ``mech`` (the magnum is the
    bottle drawn)."""
    lip = np.asarray(mech.meta["cradle_lip"], dtype=float)
    y0, slope, x0 = seat(JEROBOAM)
    part = _place(_bottle(JEROBOAM, y0, slope, x0), lip, float(mech.meta["cradle_tilt"]))
    box = part.bounding_box()
    out = []
    for b in mech.bodies:
        if b.part is None or b.name in ("bottle", "base_stop_cup") or b.name.endswith(
                ("_liner", "_collar")):         # (the cup is set behind the bottle in use)
            continue
        q = b.placed_part()
        bb = q.bounding_box()
        if not all(max(getattr(box.min, c), getattr(bb.min, c))
                   < min(getattr(box.max, c), getattr(bb.max, c)) for c in "XYZ"):
            continue
        inter = part & q
        if inter is not None and sum(x.volume for x in inter.solids()) > 1.0:
            out.append(b.name)
    return out


def print_poses(names, tilt: float) -> dict[str, list[tuple[str, float]]]:
    """How a cradle's printed parts that don't lie in a layer go on the print bed
    (:func:`build.print_pose`: turns about the global axes, ``tilt`` the cradle's in the
    drawn pose, radians): the saddles, the liners and adapters on an end (the V's section on
    the bed: no overhang), the light's clip on its tube's foot (upright, round), the base
    stop's spine on its underside and its cup on its channel's end, the collar on an end,
    the brackets on their L (its corner in every layer), the feet as they stand, the hand
    crank on its arm (its tube up)."""
    t = -math.degrees(tilt)
    out: dict[str, list[tuple[str, float]]] = {}
    for n in names:
        if n in {s.name for s in SADDLES}:
            out[n] = [("Z", t)]
        elif n in ("base_stop_spine", "led_clip"):  # on its underside; on its tube's foot
            out[n] = [("Z", t), ("X", 90.0)]
        elif n == "base_stop_cup":                  # on its channel's open end
            out[n] = [("Z", t), ("Y", 90.0)]
        elif n.endswith("_liner"):
            out[n] = [("Z", t), ("Y", -90.0)]
        elif n.endswith("_collar"):
            out[n] = [("X", -90.0)]
        elif n.startswith("foot_bracket_") and n.count("_") <= 2:   # its L on the bed: the
            out[n] = [("Y", 90.0)]                                   # corner in every layer
        elif n.startswith("foot_boot_"):
            out[n] = [("X", 90.0)]
        elif n in ("L.handle", "L.handle_grip"):
            out[n] = [("Y", 90.0)]           # (the input shaft along +x: the arm down)
    return out
