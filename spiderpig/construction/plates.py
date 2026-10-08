"""Laser-cut plates: the leg links and the two frame plates.

These groups run last: they cut the holes every other group asked for
(:attr:`construction.base.Realized.cuts`) and grow the frame plates by the
pads other groups need (the servo footprint, chassis tabs).
"""

from __future__ import annotations

import math

import numpy as np

from spiderpig.construction.base import (
    FRAME_INNER,
    FRAME_OUTER,
    RIDES_HOST,
    Build,
    Context,
    Group,
    Motion,
    Realized,
    hardware,
)
from spiderpig.shapes import Cut, Rect, box, cut_holes, disc, pill, plate, share, union
from spiderpig.stack import Claim, Disc, Layout, Pill, Placed, body_class

SOCK_T = 1.5            # a TPU foot sock's wall round the toe (mm)
SOCK_REACH = 2.5        # its claim past the link's edge: the wall and its legs' corners
NOTCH = (2.2, 0.8)      # the toe's flank notches its lugs snap into (along, deep)
NOTCH_BACK = 2.0        # a notch's centre behind the toe's centre (mm)
SOCK_COLOR = "#2b2b2b"


CHORD_MAX_DEG = 120.0   # pillars at most this far apart about O get a chord between them


def chords(o, pillars) -> list[tuple]:
    """The frame plates' chords (the joinery plan): a bar between pillars next to each other
    about O (at most ``CHORD_MAX_DEG`` apart), which closes each arm pair into a triangle;
    the plates are in their own layers, so it costs the legs nothing."""
    if len(pillars) < 2:
        return []
    ang = sorted((math.atan2(q[1] - o[1], q[0] - o[0]), tuple(q)) for q in pillars)
    if len(ang) == 2:           # one pair: one chord, across the smaller of its two gaps
        (a, q), (b, r) = ang
        gap = min((b - a) % (2 * math.pi), (a - b) % (2 * math.pi))
        return [(q, r)] if 1e-6 < math.degrees(gap) <= CHORD_MAX_DEG else []
    out = []
    for i, (a, q) in enumerate(ang):
        b, r = ang[(i + 1) % len(ang)]
        gap = (b - a) % (2 * math.pi)
        if 1e-6 < math.degrees(gap) <= CHORD_MAX_DEG:
            out.append((q, r))
    return out


CORNER_R = 1.5          # a lightening pocket's inside corners (SendCutSend cuts 0.8 mm)


def boss_web(key: str | None) -> float:
    """The plate a frame plate keeps round a hole (mm): the service's least hole-to-edge
    distance in its sheet (:attr:`materials.Sheet.min_edge`: 2 x t in aluminium), and a
    little over; 0 for a sheet with no rule."""
    if key is None:
        return 0.0
    from spiderpig.materials import sheet

    edge = sheet(key).min_edge
    return edge + 0.1 if edge > 0 else 0.0


def window(part, tri, r: float, z0: float, z1: float):
    """``part`` with the triangle ``tri`` (O and two pillars) filled and a lightening pocket
    cut in it: the triangle shrunk by ``r`` (the arms' and the chord's half-width), its
    corners rounded to :data:`CORNER_R`. A pocket too small for that stays filled."""
    from build123d import Polygon, offset

    pts = [tuple(map(float, q)) for q in tri]
    area = abs((pts[1][0] - pts[0][0]) * (pts[2][1] - pts[0][1])
               - (pts[2][0] - pts[0][0]) * (pts[1][1] - pts[0][1])) / 2
    if area < 1e-6:
        return part
    face = Polygon(*pts, align=None)
    filled = union([part, _prism(face, z0, z1)])
    try:
        inner = offset(face, -(r + CORNER_R))
        inner = offset(inner, CORNER_R)
    except Exception:          # noqa: BLE001 - nothing left inside: the window stays filled
        return filled
    if not getattr(inner, "area", 0) or inner.area < 4 * CORNER_R ** 2:
        return filled
    return filled - _prism(inner, z0, z1)


def _prism(face, z0: float, z1: float):
    from build123d import Pos, extrude

    return Pos(0, 0, z0) * extrude(face, z1 - z0)


def foot_links(topo, lk) -> list[tuple[str, str, str]]:
    """``(link, foot point, the link's other end)`` of every foot link of the side: the
    linkage's feet (``lk.feet``: link class and point), per leg, where the foot is a free
    pill end (one segment of the link ends there: the sock's legs and notches run along
    it). A foot at a corner of the link's outline (two segments meet there: Jansen's
    triangle b6) gets no sock: one along either side would sit in the other's material."""
    out = []
    for cls, point in getattr(lk, "feet", ()):
        for name, segs in topo.links.items():
            if body_class(name) != cls:
                continue
            suffix = name[len(cls):]
            for cand in (f"{point}{suffix}", f"{name}.{point}"):
                at = [sg for sg in segs if cand in sg]
                if len(at) > 1:             # a corner: no sock
                    break
                seg = at[0] if at else None
                if seg is not None:
                    other = seg[0] if seg[1] == cand else seg[1]
                    out.append((name, cand, other))
                    break
    return out


RIDER_BOSS_T = 1.0      # a metal link keeps this many x its thickness round its crank bore


def rider_bosses(ctx: Context) -> dict[str, tuple[str, float]]:
    """Per aluminium link riding a crankpin: (the crankpin, the radius of the boss its end
    grows round the bore), where the link's own width leaves less than ``RIDER_BOSS_T`` x
    its thickness of web there (the cut rules' error, 2026-10-04: the hex crankpin's 8.5 mm
    sleeve in klann_lego's 6061 b1 left 1.57 mm). The bore is the crank's rider hole."""
    from spiderpig import construction
    from spiderpig.materials import sheet

    crank = construction.CRANKS.get(getattr(ctx.config, "crank", ""), None)
    if crank is not None:             # the crank this sheet makes it (the hex pin's sleeve:
        crank = crank.resolve(ctx)    # unresolved, a BoltCrank reads the round 6 mm pin)
    d = crank.rider_d(ctx.params) if crank is not None else ctx.params.crankpin_d
    hole = ctx.params.hole(d)
    out = {}
    for link, at in ctx.topo.riders.items():
        key = ctx.sheet("link", link)
        if key is None or sheet(key).min_edge <= 0:
            continue
        r = hole / 2 + RIDER_BOSS_T * ctx.sheet_t("link", link) + 0.1
        if r > ctx.params.link_radius + 1e-9:
            out[link] = (at, r)
    return out


class LinkPlates(Group):
    """Every leg link, cut from the sheet, in the layer the plan gives it (an aluminium
    link's end grown round a crank bore its width can't hold: :func:`rider_bosses`)."""

    name = "links"
    cuts = True

    def claims(self, ctx: Context) -> list[Claim]:
        r = ctx.params.link_radius
        bosses = rider_bosses(ctx)

        feet = {name: foot for name, foot, _ in foot_links(ctx.topo, ctx.config.lk)} \
            if hasattr(ctx.config, "lk") else {}

        def make(link: str, segs):
            def f(L: Layout):
                out = [Placed(L.layers[link], Pill(a, b, r), link, link) for a, b in segs]
                if link in bosses:      # its end grown round the crank bore
                    at, rb = bosses[link]
                    out.append(Placed(L.layers[link], Disc(at, rb), link, f"{link} boss"))
                if link in feet:        # its TPU sock round the toe, in its own layer
                    out.append(Placed(L.layers[link], Disc(feet[link], r + SOCK_REACH), link,
                                      f"{link} sock"))
                return out
            return f

        return [Claim(n, frozenset((n,)), make(n, segs)) for n, segs in ctx.topo.links.items()]

    def realize(self, build: Build, done: Realized) -> Realized:
        out = Realized()
        r = build.ctx.params.link_radius
        ctx = build.ctx
        bosses = rider_bosses(ctx)
        feet = {name: (foot, other) for name, foot, other in
                (foot_links(build.plan.topo, ctx.config.lk) if hasattr(ctx.config, "lk")
                 else ())}
        for name, segs in build.plan.topo.links.items():
            z0, z1 = build.z(build.layers[name])
            z1 = min(z1, z0 + ctx.sheet_t("link", name))     # its own sheet, on the layer's floor
            cuts = list(done.cuts.get(name, []))
            if name in feet:
                sock, notches = foot_sock(build.xy(feet[name][0]), build.xy(feet[name][1]), r,
                                          z0, z1)
                cuts += notches
                out.bodies.append(hardware(f"{name}_sock", sock, name, fab="printed",
                                           color=SOCK_COLOR))
                out.notes.setdefault("feet", {})[name] = {
                    "sock": "TPU 95A", "wall_mm": SOCK_T, "point": feet[name][0]}
            boss = bosses.get(name)
            part = plate([(build.xy(a), build.xy(b), r) for a, b in segs], z0, z1, cuts,
                         discs=[(build.xy(boss[0]), boss[1])] if boss else ())
            out.bodies.append(hardware(name, part, name, fab="laser",
                                       sheet=ctx.sheet("link", name)))
        return out

    def motion(self, got: Realized) -> Motion:
        """A link, its boss and its sock are drawn from its own joints, and every hole the
        others ask of it is round at one of them: each rides its link."""
        return RIDES_HOST


def foot_sock(foot, other, r: float, z0: float, z1: float):
    """A printed TPU 95A sock on a foot link's toe (the joinery plan): a 1.5 mm wall round
    the toe's rounded end, in the link's own layer (nothing stands into the layers beside
    it), its two legs along the flanks with a lug each snapped into a notch laser-cut in
    the flank; replaceable as it wears, and the grip the sim's floor friction assumes
    (:attr:`sim.mjcf.SimParams.friction`). Returns (the sock, the link's notch cuts)."""
    f, o = np.asarray(foot, float), np.asarray(other, float)
    d = f - o
    d = d / max(float(np.linalg.norm(d)), 1e-9)
    n = np.array([-d[1], d[0]])
    ang = math.atan2(d[1], d[0])
    ring = disc(tuple(f), r + SOCK_T, z0, z1) - disc(tuple(f), r, z0 - 1, z1 + 1)
    half = box(tuple(f + d * (r + SOCK_T) / 2), (r + SOCK_T + 0.01, 2 * (r + SOCK_T) + 1,
                                                z1 - z0), z0, ang)
    legs = [box(tuple(f - d * (NOTCH_BACK + 0.5) / 2 + s * n * (r + SOCK_T / 2)),
                (NOTCH_BACK + 0.5 + 0.02, SOCK_T, z1 - z0), z0, ang) for s in (-1, 1)]
    lugs = [box(tuple(f - d * NOTCH_BACK + s * n * (r - NOTCH[1] / 2 + 0.05)),
                (NOTCH[0] - 0.2, NOTCH[1], z1 - z0), z0, ang) for s in (-1, 1)]
    sock = union([ring & half, *legs, *lugs])
    notches = [Rect(tuple(f - d * NOTCH_BACK + s * n * (r - NOTCH[1] / 2 + 0.05)),
                    (NOTCH[0], NOTCH[1] + 0.1), ang) for s in (-1, 1)]
    return sock, notches


# a frame plate made from exactly these inputs before (the plates don't move: every crank
# angle of a check or a build makes the same one); the key is every input of the plate,
# the value a wrapper of its B-rep that is never handed out: each fabrication gets a
# wrapper of its own (shapes.share), so moving one in place moves no other
_FRAME_MEMO: dict = {}


class FramePlates(Group):
    """The inner (servo) and outer frame plates: arms from O out to every pillar."""

    name = "frame"
    cuts = True

    def claims(self, ctx: Context) -> list[Claim]:
        return []   # layers 0 and top are reserved for these plates by the planner

    def realize(self, build: Build, done: Realized) -> Realized:
        out = Realized()
        p = build.ctx.params
        topo = build.plan.topo
        o = tuple(build.xy("O"))
        pillars = [tuple(build.xy(a.name)) for a in topo.axes_of("frame")]
        arms = [(o, q, p.frame_radius) for q in pillars]
        arms += [(a, b, p.frame_radius) for a, b in chords(o, pillars)]
        frame = topo.frame_bodies[0]
        tris = [(o, a, b) for a, b in chords(o, pillars)]
        web = boss_web(build.ctx.sheet("frame"))
        for key, layer, name in ((FRAME_INNER, build.top, frame),
                                 (FRAME_OUTER, 0, "frame_outer")):
            z0, z1 = build.z(layer)
            memo = (tuple(arms), tuple(tris), z0, z1, p.frame_radius, web,
                    tuple(done.pads.get(key, [])), tuple(done.cuts.get(key, [])))
            hit = _FRAME_MEMO.get(memo)
            if hit is not None:         # the same plate at another crank angle: static
                out.bodies.append(hardware(name, share(hit), frame, fab="laser",
                                           color="#eb6834", sheet=build.ctx.sheet("frame")))
                continue
            part = plate(arms, z0, z1, discs=[(o, p.frame_radius)])
            for tri in tris:        # the window an arm pair and its chord close: a lightening
                part = window(part, tri, p.frame_radius, z0, z1)    # pocket, corners rounded
            extra = [pill(a, b, r, z0, z1) for a, b, r in done.pads.get(key, [])]
            cuts = done.cuts.get(key, [])
            # a boss round every round hole: the service's two thicknesses of plate to the
            # edge (the design review's warning level; under one an error), the plate's own
            # layer, so it costs the legs nothing
            extra += [disc(c.xy, c.d / 2 + web, z0, z1) for c in cuts
                      if isinstance(c, Cut) and web > 0]
            if extra:
                part = union([part, *extra])
            part = cut_holes(part, cuts, z0, z1)
            if len(_FRAME_MEMO) > 32:
                _FRAME_MEMO.clear()
            _FRAME_MEMO[memo] = share(part)
            out.bodies.append(hardware(name, part, frame, fab="laser", color="#eb6834",
                                       sheet=build.ctx.sheet("frame")))
        return out

    def motion(self, got: Realized) -> Motion:
        """The plates don't move: O, the pillars and what the others ask of them (the
        servo's holes and pad, the pillars', the journal's) stand with the frame."""
        return RIDES_HOST
