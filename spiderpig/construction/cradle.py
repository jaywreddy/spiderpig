"""A decanting cradle: two side stacks, the cradle box between them, a stand and a bottle.

A cradle linkage (:mod:`linkages.decanters`, ``Linkage.product == "cradle"``) is planned
one side at a time like any mechanism. The bottle lies between two sides, the **bay**, so
:func:`assemble_cradle` makes the cradle from that one side:

* **The drive side** (``L.``, the side as planned; on your right as you stand behind the
  cradle facing the decanter) carries the hand crank (:mod:`construction.handcrank`).
* **The passive side** (``R.``) is the same plan mirrored, without the crank, the rod and
  the rod's pin (:func:`passive_design`): the cradle box drives it.
* **The cradle box**: four tie rods (6 mm steel standoffs made to length, MISUMI NETRF6)
  between the two cradle links, at the linkage's tie points (``decanters.TIES``), each
  screwed through its cradle link from outside. Two printed saddles ride on them: the
  front one under the neck, its V for the neck and its back face the shoulder's stop; the
  rear one under the body, its V deep enough for a 3 L bottle. TPU liners line both Vs. An
  M3 grub screw in a nut trap holds each saddle on each tie.
* **The light**: an AAA LED penlight standing in a printed clip on the front saddle's back,
  its head under the shoulder (the sommelier's candle; its head twists it on).
* **The stand** (:data:`STANDS`, :class:`CradleStand`): each side's frame plates are the
  stand's members, not arms from O. The outer plate carries the hinge, the gearbox and the
  legs to the feet behind the decanter. The inner plate is only what holds the pillars
  (A: a small block round the hinge with two columns; B: the bar O1-O2), so the tie rods
  pass it in the open. Base ties across the cradle at the feet hold the two sides apart.
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

import numpy as np
from build123d import Axis as BAxis
from build123d import Box, Cylinder, Location, Plane, Polyline, make_face, revolve

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

LIP_HEIGHT = 340.0      # the bottle's mouth over the table (mm): a decanter up to ~310 mm
DECANTER_GAP = 30.0     # the decanter's mouth at least this far under the bottle's
BAY = 160.0             # between the two inner plates' bay faces: a 141 mm bottle and room
TIE_D = 6.0             # the tie rods: MISUMI NETRF6 (6 mm, M3 x 6 deep each end)
TIE_THREAD = 6.0
M3 = screw("bhcs", "3")
TIE_SCREW = 8.0         # M3 x 8 button heads through 3.175 mm plates: 4.8 mm in the tie
LINER = 2.0             # the saddles' TPU liners
SADDLE_CLEAR = 3.0      # a saddle's end to an inner plate's bay face
FRAME_COLOR = "#eb6834"
SADDLE_COLOR = "#6a4fc7"
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


MAGNUM = BottleSize("1.5 L Bordeaux magnum", 356.0, 49.5, 15.5, 85.0, 150.0, 2.6, 205.0)
JEROBOAM = BottleSize("3 L Champagne jeroboam", 470.0, 70.5, 19.0, 105.0, 190.0, 5.9, 265.0)
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


SADDLES = (
    # the neck's V sized for the jeroboam's neck; its back face (x 87) the shoulder's stop:
    # a magnum's mouth lands 2 mm behind the hinge (its neck 85 mm and the liner)
    Saddle("front_saddle", ("Tf1", "Tf2"), (45.0, 87.0), -50.0, _apex(JEROBOAM.r_neck), -8.0),
    # the body's V sized for the jeroboam: a magnum sits nose up a few degrees
    Saddle("rear_saddle", ("Tr1", "Tr2"), (255.0, 315.0), -122.0, _apex(JEROBOAM.r), -45.0),
)


# --- the stand -------------------------------------------------------------------------

@dataclass(frozen=True)
class Stand:
    """A cradle's frame plates as a truss (mm, relative to the bottle's mouth at rest).

    ``nodes``: the free points; a member may also name O or a pillar. ``outer``/``inner``:
    each plate's members (pills of ``member_r``). ``columns``: a 6 mm steel standoff between
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


_FOOT_Y = -LIP_HEIGHT + 12.0 + 3.0      # a foot's centre: its member's edge on a 3 mm boot sole
FRONT_FOOT = 105.0
"""The front feet behind the mouth (mm): as far forward as the decanter lets them (a 244 mm
decanter is 92 mm off its axis at the side plates, 80 mm out), so a full 3 L bottle's
weight at full tilt (152 mm behind the mouth) stays over the feet."""

STANDS: dict[str, Stand] = {
    # A: the hinge G at the mouth, the crank O 220 behind and 200 under it. The outer plate
    # goes from G back over the decanter's shoulder (K) and down behind it to the front
    # foot; O's legs to both feet. The inner plate is a small block round G, tied to the
    # outer by two columns in front of the hinge, where nothing of the cradle ever goes.
    "decanter_hinge": Stand(
        nodes={"K": (125.0, -115.0), "Ff": (FRONT_FOOT, _FOOT_Y), "Fm": (220.0, _FOOT_Y),
               "Fr": (440.0, _FOOT_Y), "C1": (-24.0, 20.0), "C2": (-24.0, -22.0),
               "Bt": (-20.0, 38.0)},
        # O's leg straight down to a foot, and a rail on the table back to the rear foot:
        # nothing of the plate behind O over the rail, where the hand crank turns
        outer=(("G", "K"), ("K", "Ff"), ("K", "O"), ("O", "Ff"), ("O", "Fm"), ("Ff", "Fm"),
               ("Fm", "Fr"), ("G", "C1"), ("G", "C2"), ("C1", "C2")),
        inner=(("G", "C1"), ("G", "C2"), ("C1", "C2"), ("C1", "Bt"), ("G", "Bt")),
        columns=("C1", "C2"),
        feet=("Ff", "Fm", "Fr"),
        base_ties=("Fr",),
        # over the mouth and in front of it: a bottle's neck is always behind its mouth
        bay_ties=("Bt",),
    ),
    # B: the arms' pivots O1, O2 on a ray down and back from the mouth, the crank O behind
    # them. The inner plate is the bar O1-O2 (the tie rods pass above and in front of it).
    "decanter_rcm": Stand(
        nodes={"Ff": (FRONT_FOOT, _FOOT_Y), "Fr": (490.0, _FOOT_Y), "Bt": (172.4, -144.7)},
        outer=(("O1", "O2"), ("O2", "O"), ("O1", "Ff"), ("O2", "Ff"), ("O", "Fr"),
               ("Ff", "Fr"), ("O2", "Fr")),
        inner=(("O1", "O2"),),
        feet=("Ff", "Fr"),
        base_ties=("Fr",),
        # on the O1-O2 bar, 25 mm from O1: under every part of the cradle's sweep
        bay_ties=("Bt",),
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
        for n in (*stand.columns, *stand.base_ties, *stand.bay_ties):
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
        r = TIE_D / 2 + 0.3
        head = M3.head_d / 2 + 0.3

        def make(L: Layout):
            out = []
            for name in bay:        # a bay tie's screw: up through the inner plate
                out.append(Placed(L.top, Disc(name, 1.7), self.name, "bay tie screw",
                                  seat=True))
                out.append(Placed(L.top - 1, Disc(name, head), self.name, "bay tie screw head",
                                  gap=True, height=M3.head_h + 0.25, toward=-1))
            for name in (p for n, p in pts.items() if p not in bay):
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
        hole = 3.4
        z0, z1 = build.z(0)[1], build.z(build.top)[0]      # between the plates
        for n in stand.base_ties:
            out.cut(FRAME_OUTER, Cut(xy(n), hole))
        for n in stand.bay_ties:
            out.cut(FRAME_INNER, Cut(xy(n), hole))
        for n in stand.columns:
            p = xy(n)
            out.cut(FRAME_OUTER, Cut(p, hole))
            out.cut(FRAME_INNER, Cut(p, hole))
            out.bodies.append(hardware(f"stand_column_{n}", _tie(p, z0, z1), frame,
                                       fab="purchased", bom_key=_netrf6(z1 - z0),
                                       color=TIE_COLOR))
            for side, z in ((-1, build.z(0)[0]), (1, build.z(build.top)[1])):
                out.bodies.append(hardware(f"stand_column_{n}_screw{'io'[side > 0]}",
                                           _m3(p, z, side), frame, fab="purchased",
                                           bom_key=f"m3_bhcs_{TIE_SCREW:g}", color=TIE_COLOR))
        out.notes["stand"] = {"columns": list(stand.columns), "feet": list(stand.feet),
                              "base_ties": list(stand.base_ties),
                              "bay_ties": list(stand.bay_ties), "column_mm": round(z1 - z0, 2)}
        return out

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
        head = M3.head_d / 2 + 0.3

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
                                  gap=True, height=M3.head_h + 0.25, toward=-1))
            return out

        return [Claim("tie rods", frozenset((link,)), make)]

    def realize(self, build: Build, done: Realized) -> Realized:
        lk = build.ctx.config.lk
        out = Realized()
        link = next(n for n in build.plan.topo.links if body_class(n) == cradle_link(lk))
        for name in tie_points(build.plan.topo, lk).values():
            out.cut(link, Cut(tuple(build.xy(name)), 3.4))
        return out

    def motion(self, got: Realized) -> Motion:
        return RIDES_HOST


_PLATE_MEMO: dict = {}


class CradlePlates(FramePlates):
    """A cradle's frame plates: the stand's members (the pads :class:`CradleStand` asks
    for) and every other group's pads, with a boss round every round hole (the cut rules'
    two thicknesses), minus the holes; no arms from O (the stand's members replace them)."""

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


# --- parts -----------------------------------------------------------------------------

def _netrf6(length: float) -> str:
    from spiderpig.hardware.crank_catalog import pillar_shaft

    return pillar_shaft(round(length, 1))


def _tie(xy, z0: float, z1: float) -> Shape3D:
    """A 6 mm steel standoff from z0 to z1, its ends tapped (modelled as M3 holes)."""
    return difference(disc(xy, TIE_D / 2, z0, z1),
                      *[disc(xy, 1.5, a, b)
                        for a, b in ((z0 - 1, z0 + TIE_THREAD), (z1 - TIE_THREAD, z1 + 1))])


def _m3(xy, face: float, side: int, length: float = TIE_SCREW) -> Shape3D:
    """An M3 button head bearing on ``face``, its head on the ``side`` (+1 over it, -1
    under it) and its shank into the part beyond."""
    head = disc(xy, M3.head_d / 2, *sorted((face, face + side * M3.head_h)))
    shank = disc(xy, M3.shank_d / 2, *sorted((face, face - side * length)))
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
    face = make_face(Polyline(*[(x, r) for x, r in pts], close=True).edges())
    solid = revolve(face.faces(), BAxis.X)
    loc = Location((x0, axis_y0, 0.0), (0, 0, math.degrees(slope)))
    return moved(solid, loc)


def seat(size: BottleSize) -> tuple[float, float, float]:
    """Where ``size`` lies in the saddles: (its axis's y at the mouth's x, the axis's slope
    in radians, the mouth's x), in the cradle's frame."""
    front, rear = SADDLES
    xf, xr = sum(front.x) / 2, sum(rear.x) / 2
    yf = front.apex + (size.r_neck + LINER) * math.sqrt(2)
    yr = rear.apex + (size.r + LINER) * math.sqrt(2)
    slope = math.atan2(yr - yf, xr - xf)
    x0 = front.x[1] - size.neck - LINER        # its shoulder on the front saddle's back face
    return yf + (x0 - xf) * math.tan(slope), slope, x0


def _saddle(s: Saddle, ties: Mapping[str, tuple[float, float]]) -> tuple[Shape3D, Shape3D]:
    """A saddle and its TPU liner, in the cradle's frame: a block across the bay, a 90° V
    cut along the axis (its apex ``s.apex`` under the liner), the tie holes."""
    half = BAY / 2 - SADDLE_CLEAR
    x0, x1 = s.x
    block = Box(x1 - x0, s.top - s.y0 + 30.0, 2 * half).moved(
        Location(((x0 + x1) / 2, (s.y0 + s.top + 30.0) / 2, 0.0)))
    block = block - Box(x1 - x0 + 2, 60.0, 2 * half + 2).moved(
        Location(((x0 + x1) / 2, s.top + 30.0, 0.0)))     # trim at the V's top edges

    def vee(apex: float):
        size = 400.0
        return Box(x1 - x0 + 2, size, size).rotate(BAxis.X, 45).moved(
            Location(((x0 + x1) / 2, apex - LINER * math.sqrt(2) + size / math.sqrt(2), 0.0)))

    outer = vee(s.apex)                      # the liner's back: the saddle's V
    inner = vee(s.apex + LINER * math.sqrt(2))
    saddle = block - outer
    liner = (block & outer) - inner
    for n in s.ties:
        x, y = ties[n]
        hole = Cylinder(TIE_D / 2 + 0.15, 2 * half + 4).moved(Location((x, y, 0.0)))
        saddle = saddle - hole
    return saddle, liner


def _led(front: Saddle) -> tuple[Shape3D, Shape3D]:
    """The penlight under the shoulder and its printed clip, in the cradle's frame: the
    light stands under the neck's end, its head up at the shoulder; the clip holds its body
    below the head (which twists to switch it on) and joins the front saddle's back face."""
    d, length = PENLIGHT
    x, head = front.x[1] + 13.0, LED_TOP
    up = (90, 0, 0)                          # a cylinder's axis along y
    light = Cylinder(d / 2, length, rotation=up).moved(Location((x, head - length / 2, 0.0)))
    clip_top, clip_bot = head - CLIP_GAP, head - CLIP_GAP - 36.0
    tube = Cylinder(d / 2 + 2.5, clip_top - clip_bot, rotation=up).moved(
        Location((x, (clip_top + clip_bot) / 2, 0.0)))
    tube = tube - Cylinder(d / 2 + 0.2, clip_top - clip_bot + 2, rotation=up).moved(
        Location((x, (clip_top + clip_bot) / 2, 0.0)))
    arm = Box(x - front.x[1] - d / 2 - 1.0, 10.0, 12.0).moved(
        Location(((front.x[1] + x - d / 2 - 1.0) / 2, front.y0 + 5.0 - 0.01, 0.0)))
    web = Box(4.0, front.y0 + 10.0 - clip_bot, 12.0).moved(
        Location((x - d / 2 - 2.0, (front.y0 + 10.0 + clip_bot) / 2, 0.0)))
    return light, union([tube, arm, web])


PENLIGHT = (12.7, 81.0)     # the light: Maglite Solitaire LED, a AAA penlight (d, length)
LED_TOP = -58.0             # its head, in the cradle's frame: 10 mm under a magnum's shoulder
CLIP_GAP = 16.0             # the clip starts this far under the head (the head twists)


def _boot(xy, z0: float, z1: float, plate_z: tuple[float, float]) -> Shape3D:
    """A printed TPU boot on a stand foot (world, the plate's own z): a sole under the
    plate's end and two cheeks either side of it."""
    x, y = float(xy[0]), float(xy[1])
    sole_top = y - FOOT_R
    t0, t1 = plate_z
    w = (t1 - t0) + 2 * BOOT_WALL
    sole = Box(BOOT_LEN, BOOT_SOLE, w).moved(
        Location((x, sole_top - BOOT_SOLE / 2, (t0 + t1) / 2)))
    cheek = Box(BOOT_LEN, BOOT_H, BOOT_WALL)
    cheeks = [cheek.moved(Location((x, sole_top + BOOT_H / 2, zc)))
              for zc in (t0 - BOOT_WALL / 2, t1 + BOOT_WALL / 2)]
    return union([sole, *cheeks])


FOOT_R = 12.0               # the stand's members' half-width (Stand.member_r)
BOOT_SOLE, BOOT_WALL, BOOT_H, BOOT_LEN = 3.0, 2.0, 8.0, 44.0


# --- the assembly ----------------------------------------------------------------------

def passive_design(design):
    """The cradle's other side's design: the side's without the hand crank and the rod's
    pin (so its plates get no holes for them); :func:`fabricate.fabricate` builds it and
    :func:`assemble_cradle` leaves its rod out."""
    from spiderpig.construction.axle import AxleGroup
    from spiderpig.construction.handcrank import HandCrank

    rider = HandCrank.rider(design.ctx)
    groups = [g for g in design.groups
              if not isinstance(g, HandCrank)
              and not (isinstance(g, AxleGroup) and rider in g.axis.members)]
    return replace(design, groups=groups)


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


def assemble_cradle(side: Mechanism, passive: Mechanism, design) -> Mechanism:
    """The whole cradle from its drive side ``side`` and its other side ``passive`` (made
    with :func:`passive_design`), both at one crank angle (the module's docstring)."""
    from spiderpig.construction.handcrank import HandCrank

    lk = design.config.lk
    cf = cradle_frame(design, side)
    z_mid = cf.z_mid
    rider = HandCrank.rider(design.ctx)
    passive = replace(passive, bodies=[b for b in passive.bodies if b.name != rider])
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
                               bom_key=_netrf6(tie_len), color=TIE_COLOR))
        for s, sign in (("L", -1), ("R", 1)):
            face = sign * (z_mid - link_bottom)      # the link's outer face
            extras.append(hardware(f"tie_{n}_screw{s}", _m3(xy, face, sign), host,
                                   fab="purchased", bom_key=f"m3_bhcs_{TIE_SCREW:g}",
                                   color=TIE_COLOR))
    stand = stand_of(lk)
    z0 = plan.z(0)[1]
    foot_half = z_mid - z0                            # a base tie between the outer plates
    foot_len = round(2 * foot_half, 1)
    for n in stand.base_ties:
        xy = tuple(_node_xy(build.xy, lk, topo, stand, n))
        extras.append(hardware(f"base_tie_{n}", _tie(xy, -foot_half, foot_half), frame_host,
                               fab="purchased", bom_key=_netrf6(foot_len), color=TIE_COLOR))
        for s, sign in (("L", -1), ("R", 1)):
            face = sign * (z_mid - plan.z(0)[0])
            extras.append(hardware(f"base_tie_{n}_screw{s}", _m3(xy, face, sign), frame_host,
                                   fab="purchased", bom_key=f"m3_bhcs_{TIE_SCREW:g}",
                                   color=TIE_COLOR))
    bay_half = BAY / 2                  # a bay tie between the inner plates' bay faces
    bay_len = round(2 * bay_half, 1)
    for n in stand.bay_ties:
        xy = tuple(_node_xy(build.xy, lk, topo, stand, n))
        extras.append(hardware(f"bay_tie_{n}", _tie(xy, -bay_half, bay_half), frame_host,
                               fab="purchased", bom_key=_netrf6(bay_len), color=TIE_COLOR))
        for s, sign in (("L", -1), ("R", 1)):
            face = sign * (z_mid - plan.z(plan.top)[0])     # the inner plate's leg face
            extras.append(hardware(f"bay_tie_{n}_screw{s}", _m3(xy, face, sign), frame_host,
                                   fab="purchased", bom_key=f"m3_bhcs_{TIE_SCREW:g}",
                                   color=TIE_COLOR))
    # the saddles, their liners, the light, the bottle: drawn in the cradle's frame
    for s in SADDLES:
        saddle, liner = _saddle(s, cf.ties)
        extras.append(hardware(s.name, _place(saddle, cf.lip, cf.tilt), host, fab="printed",
                               color=SADDLE_COLOR))
        extras.append(hardware(f"{s.name}_liner", _place(liner, cf.lip, cf.tilt), host,
                               fab="printed", color=LINER_COLOR))
        # a grub screw in a nut trap holds it on each tie
        fastened.extend((s.name, f"tie_{n}") for n in s.ties)
    light, clip = _led(SADDLES[0])
    extras.append(hardware("led_clip", _place(clip, cf.lip, cf.tilt), host, fab="printed",
                           color=SADDLE_COLOR))
    extras.append(hardware("led", _place(light, cf.lip, cf.tilt), host, fab="purchased",
                           bom_key="penlight_aaa", color=LED_COLOR))
    # the feet's TPU boots, on each side's outer plate
    for s, sign in (("L", -1), ("R", 1)):
        pz = sorted((sign * (z_mid - plan.z(0)[0]), sign * (z_mid - plan.z(0)[1])))
        for n in stand.feet:
            xy = _node_xy(build.xy, lk, topo, stand, n)
            extras.append(hardware(f"foot_boot_{n}{s}", _boot(xy, 0, 0, tuple(pz)),
                                   prefixed(topo.frame_bodies[0], s), fab="printed",
                                   color=LINER_COLOR))
    y0, slope, x0 = seat(MAGNUM)
    extras.append(hardware("bottle", _place(_bottle(MAGNUM, y0, slope, x0), cf.lip, cf.tilt),
                           host, fab="display", color=BOTTLE_COLOR))
    extras.append(hardware("decanter", _decanter(cf.lip), frame_host, fab="display",
                           color=DECANTER_COLOR))
    fastened += [("bottle", s.name + "_liner") for s in SADDLES]
    fastened += [("led", "led_clip"), ("led_clip", SADDLES[0].name)]
    meta = dict(side.meta, cradle=True, mid_plane=z_mid, bay_mm=BAY, lip_height_mm=LIP_HEIGHT,
                tie_mm=tie_len, base_tie_mm=foot_len, bay_tie_mm=bay_len, fastened=fastened,
                bottle=MAGNUM.name)
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
    face = make_face(Polyline(*prof, close=True).edges())
    solid = revolve(face.faces(), BAxis.Y)
    return moved(solid, Location((float(lip[0]) - 5.0, 0.0, 0.0)))
