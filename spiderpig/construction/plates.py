"""Laser-cut plates: the leg links and the two frame plates.

These groups run last: they cut the holes every other group asked for
(:attr:`construction.base.Realized.cuts`) and grow the frame plates by the
pads other groups need (the servo footprint, chassis tabs).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import cast

import numpy as np
from build123d import Face, Location, Vector

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
from spiderpig.shapes import (
    Cut,
    Rect,
    Shape3D,
    box,
    cut_holes,
    difference,
    disc,
    intersection,
    pill,
    plate,
    share,
    union,
)
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
    return difference(filled, _prism(inner, z0, z1))


def _prism(face, z0: float, z1: float) -> Shape3D:
    from build123d import Pos, extrude

    # a Part moved is a Part (build123d types ``Location * Shape`` as a bare Shape)
    return cast("Shape3D", Pos(0, 0, z0) * extrude(face, z1 - z0))


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


def _product(ctx: Context) -> bool:
    """Whether ``ctx`` builds a product (a decanting cradle: :attr:`linkage.Linkage.product`)."""
    return bool(getattr(getattr(ctx.config, "lk", None), "product", ""))


def _chord_r(ctx: Context, link: str, a: str, b: str) -> float:
    """A link chord's half-width: a product's own (:func:`config.chord_r`), else
    ``Params.link_radius``."""
    if not getattr(getattr(ctx.config, "lk", None), "product", ""):
        return ctx.params.link_radius
    from spiderpig.config import chord_r

    return chord_r(ctx.config, link, a, b)


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


# --- a product's links: each outline closed -------------------------------------------

PRODUCT_FILLET_R = 15.0
"""A product's links (a decanting cradle's): each link's outline, its chords and bosses,
closed by this radius (grown by it and shrunk back: every inside corner, an elbow, a boss's
cusp, a window's corner, rounded to it, a window narrower than twice it filled), so no cut
aluminium link has a sharp inside corner (design review round 2, 2026-10-15, laser-cut parts
M4: SendCutSend's 7075 page, "avoid tight internal corners"; nothing sweeps where the
closing adds, measured, and the planner checks it: :func:`product_outline`'s claims)."""
PRODUCT_FILLETS: dict[str, dict[str, float]] = {"decanter_rcm": {"b6": 18.0},
                                              "decanter_hinge": {"b2": 18.0}}
"""Per product linkage, per link: its own closing radius, where not
:data:`PRODUCT_FILLET_R`: B's cradle link b6 R18 (the review's priced shape, its sliver
window gone); A's cradle link b2 R18, both its windows gone (R15 would have left a
15 mm hole of its triangle's and that triangle's acute corner filled as a sliver)."""
FILL_SAG = 1.0          # mm: the most a fillet's claim stands off its arc into the air
FILL_SAG_WINDOW = 3.0   # mm: the same into a window the closing leaves open (the link's own)
FILL_PAD = 0.01         # mm: a fill's sampled reach from its fillet's centre, rounded up


def fillet_r(ctx: Context, link: str) -> float:
    """The radius a product's link ``link`` is closed by (:data:`PRODUCT_FILLETS`, else
    :data:`PRODUCT_FILLET_R`)."""
    return PRODUCT_FILLETS.get(ctx.config.linkage, {}).get(body_class(link), PRODUCT_FILLET_R)


def product_bosses(ctx: Context, link: str, segs) -> dict[str, float]:
    """A product's link ``link``: the radius of each boss it grows round a hole, by point:
    its crank bore's (:func:`rider_bosses`) and its own (:func:`config.boss_r`: the cut rules'
    hole-to-edge), each only where wider than the chords meeting there."""
    from spiderpig.config import boss_r

    out: dict[str, float] = {}
    rider = rider_bosses(ctx).get(link)
    if rider is not None:
        out[rider[0]] = rider[1]
    for joint, r in boss_r(ctx.config, link).items():
        p = ctx.topo.point_of.get((link, joint))
        if p is None:
            raise ConstructionError(f"{link} has no joint {joint} for its boss")
        out[p] = max(out.get(p, 0.0), r)
    chords = {p: max(_chord_r(ctx, link, a, b) for a, b in segs if p in (a, b))
              for p in out if any(p in sg for sg in segs)}
    return {p: r for p, r in sorted(out.items()) if r > chords.get(p, 0.0) + 1e-9}


Cover = tuple[tuple[float, ...], tuple[float, ...], float]   # a pill: its ends, its radius


@dataclass(frozen=True)
class Outline:
    """A product link's outline (:func:`product_outline`) in the link's own frame: the
    origin its point ``frame[0]``, x toward ``frame[1]`` (``length`` away). ``face``: its
    chords and bosses closed (:func:`fillet_r`); ``covers``: pills ``(p, q, r)`` whose union
    holds everything the closing added, standing off the closed outline into the air by at
    most about :data:`FILL_SAG` (:data:`FILL_SAG_WINDOW` into a window it leaves open), else
    into the link (the claims' points, :meth:`stack.Topology.add_link_points`, are their
    ends)."""

    frame: tuple[str, str]
    length: float
    face: Face
    covers: tuple[Cover, ...]

    def points(self, link: str) -> dict[str, tuple[str, str, float, float]]:
        """The covers' ends as points fixed to the link (name -> ``(a, b, u, v)``)."""
        a, b = self.frame
        ends = sorted({e for p, q, _ in self.covers for e in (p, q)})
        return {f"{link}.fill{i}": (a, b, x / self.length, y / self.length)
                for i, (x, y) in enumerate(ends)}

    def pills(self, link: str) -> list[tuple[str, str, float]]:
        """The covers as pills between the link's points (:meth:`points`)."""
        ends = sorted({e for p, q, _ in self.covers for e in (p, q)})
        name = {e: f"{link}.fill{i}" for i, e in enumerate(ends)}
        return [(name[p], name[q], r) for p, q, r in self.covers]


_OUTLINES: dict[tuple, tuple] = {}     # a closing's inputs -> (its face, its covers)


def product_outline(ctx: Context, link: str, segs) -> Outline:
    """A product's link ``link`` (outline ``segs``), closed: its chords
    (:func:`config.chord_r`) and bosses (:func:`product_bosses`) in its own frame, drawn
    where the plan's geometry has its points at the first sample, grown by
    :func:`fillet_r` and shrunk back (round joins). Memoized on those inputs: the claims
    and every build of the link make it from the same numbers, so each fabrication places
    one shape (it moves with its link exactly)."""
    g = ctx.topo.geometry.points
    a, b = segs[0]
    pa = np.asarray(g[a][0], dtype=float)
    d = np.asarray(g[b][0], dtype=float) - pa
    length = float(np.hypot(*d))
    ex = d / length
    ey = np.array([-ex[1], ex[0]])

    def local(p: str) -> tuple[float, float]:
        w = np.asarray(g[p][0], dtype=float) - pa
        return float(w @ ex), float(w @ ey)

    pills = tuple((local(p), local(q), _chord_r(ctx, link, p, q)) for p, q in segs)
    discs = tuple((local(p), r) for p, r in product_bosses(ctx, link, segs).items())
    cut = product_band(ctx, link, local)
    key = (pills, discs, fillet_r(ctx, link), cut)
    got = _OUTLINES.get(key)
    if got is None:
        try:
            got = _close(*key[:3])
            if cut is not None:
                got = (_band_cut(got[0], *cut), got[1])
        except ConstructionError as e:
            raise ConstructionError(f"{link}: {e}") from None
        if len(_OUTLINES) > 64:
            _OUTLINES.clear()
        _OUTLINES[key] = got
    return Outline((a, b), length, got[0], got[1])


def product_band(ctx: Context, link: str, local) -> tuple | None:
    """A product's link ``link``'s band (:func:`config.band`) in the link's own frame
    (``local``: a point name to its local x, y): ``(origin, along, lo, hi)``, the band between
    ``lo`` and ``hi`` across ``along`` from ``origin``; ``None`` if it has none."""
    from spiderpig.config import band

    got = band(ctx.config, link) if _product(ctx) else None
    if got is None:
        return None
    o, x, lo, hi = got
    po, px = (ctx.topo.point_of.get((link, j)) for j in (o, x))
    if po is None or px is None:
        raise ConstructionError(f"{link} has no joints {o} and {x} for its band")
    a, b = np.asarray(local(po), float), np.asarray(local(px), float)
    u = (b - a) / float(np.hypot(*(b - a)))
    return (tuple(map(float, a)), tuple(map(float, u)), float(lo), float(hi))


BAND_CORNER_R = 5.0
"""A product link's corners where its band cuts it (:func:`product_band`), rounded to this
radius (opened: shrunk by it and grown back), so its outline stays smooth all round; under half
its narrowest chord, so nothing else changes."""


def _band_cut(face, origin, along, lo: float, hi: float):
    """``face`` cut to the band between ``lo`` and ``hi`` across ``along`` from ``origin``
    (:func:`product_band`), the cut's corners rounded (:data:`BAND_CORNER_R`): one face."""
    from build123d import Kind, Wire, offset

    o, u = np.asarray(origin, float), np.asarray(along, float)
    v = np.array([-u[1], u[0]])
    big = 1.0e4
    ring = [o + s * big * u + y * v for s, y in ((-1, lo), (1, lo), (1, hi), (-1, hi))]
    strip = Face(Wire.make_polygon([_xy0(p) for p in ring], close=True))
    got = [f for f in _faces(face & strip) if f.area > 1e-6]
    if len(got) != 1:
        raise ConstructionError(f"its outline cut to its band is {len(got)} pieces")
    opened = offset(offset(got[0], -BAND_CORNER_R, kind=Kind.ARC), BAND_CORNER_R,
                    kind=Kind.ARC).faces()
    if len(opened) != 1:
        raise ConstructionError(f"its outline cut to its band and rounded is {len(opened)} "
                                "pieces")
    out = opened[0]
    if out.normal_at().Z < 0:
        out = -out
    return out


CRANK_FILLET_R = 10.0
"""A hand crank's block plates (:mod:`construction.handcrank`): the arm's pill on its hub's
disc, closed by this radius (design review round 3, laser-cut parts minor 1: they met in two
55.6° cusps at the neck, the arm's most bent section, on all eight plates; round 2 had priced
the R10 neck at USD 0.24 for six). What the closing adds lies inside the crank's own sweep
and is claimed (:func:`arm_fills`)."""


def _arm(hub_r: float, web_r: float, throw: float) -> tuple:
    """The crank block plate's closed outline in its own frame (O at the origin, x toward
    its crankpin ``throw`` away) and the pills covering what the closing added
    (:func:`_close`), memoized as :func:`product_outline`'s."""
    key = ((((0.0, 0.0), (throw, 0.0), web_r),), (((0.0, 0.0), hub_r),), CRANK_FILLET_R)
    got = _OUTLINES.get(key)
    if got is None:
        got = _close(*key)
        if len(_OUTLINES) > 64:
            _OUTLINES.clear()
        _OUTLINES[key] = got
    return got


def arm_fills(ctx: Context, hub_r: float, web_r: float, pin: str, throw: float
              ) -> list[tuple[str, str, float]]:
    """What closing a crank block plate's outline adds (:func:`_arm`), as pills between
    points fixed to the crank (O and its crankpin ``pin``), added to the geometry: the
    claims' share of :data:`CRANK_FILLET_R`."""
    covers = _arm(hub_r, web_r, throw)[1]
    ends = sorted({e for p, q, _ in covers for e in (p, q)})
    name = {e: f"crank.fill{i}" for i, e in enumerate(ends)}
    ctx.topo.add_link_points({name[e]: ("O", pin, e[0] / throw, e[1] / throw) for e in ends})
    return [(name[p], name[q], w) for p, q, w in covers]


def arm_plate(hub_r: float, web_r: float, throw: float, o, toward, z0: float, z1: float,
              cuts) -> Shape3D:
    """A crank block plate: :func:`_arm`'s closed outline on O at ``o``, its arm toward
    ``toward`` (the crankpin's centre), from ``z0`` to ``z1``, less ``cuts``."""
    from build123d import Solid

    turn = math.degrees(math.atan2(toward[1] - o[1], toward[0] - o[0]))
    face = _arm(hub_r, web_r, throw)[0].moved(
        Location((float(o[0]), float(o[1]), z0), (0, 0, 1), turn))
    part = cast("Shape3D", Solid.extrude(face, Vector(0, 0, z1 - z0)))
    return cut_holes(part, cuts, z0, z1)


def _xy0(p) -> Vector:
    return Vector(float(p[0]), float(p[1]), 0.0)


def flat_pill(p, q, r: float):
    """The face (in the XY plane) of the stadium ``p``-``q`` of radius ``r``; of a disc where
    ``p`` and ``q`` meet."""
    from build123d import Edge, Plane, Wire

    p, q = np.asarray(p, dtype=float), np.asarray(q, dtype=float)
    length = float(np.hypot(*(q - p)))
    if length < 1e-9:
        return Face(Wire([Edge.make_circle(r, Plane(origin=_xy0(p)))]))
    u = (q - p) / length
    n = np.array([-u[1], u[0]])
    return Face(Wire([
        Edge.make_line(_xy0(p - r * n), _xy0(q - r * n)),
        Edge.make_three_point_arc(_xy0(q - r * n), _xy0(q + r * u), _xy0(q + r * n)),
        Edge.make_line(_xy0(q + r * n), _xy0(p + r * n)),
        Edge.make_three_point_arc(_xy0(p + r * n), _xy0(p - r * u), _xy0(p - r * n))]))


def _close(pills, discs, r: float) -> tuple:
    """The face of ``pills`` and ``discs`` (in a plane) closed by ``r``, and pills covering
    what the closing added (:func:`_cover`)."""
    from build123d import Kind, offset

    faces = [flat_pill(p, q, w) for p, q, w in pills] + [flat_pill(c, c, w) for c, w in discs]
    joined = cast("Face", faces[0].fuse(*faces[1:])).clean() if len(faces) > 1 else faces[0]
    union = joined.faces()
    if len(union) != 1:
        raise ConstructionError(f"its chords and bosses make {len(union)} pieces, not one")
    closed = offset(offset(union[0], r, kind=Kind.ARC), -r, kind=Kind.ARC).faces()
    if len(closed) != 1:
        raise ConstructionError(f"its outline closed by {r:g} mm is {len(closed)} pieces")
    face = closed[0]
    if face.normal_at().Z < 0:
        face = -face
    fills = [f for f in _faces(face - union[0]) if f.area > 1e-6]
    deep = 2 * min(w for *_, w in [*pills, *discs])
    # a window the closing leaves open: its corners' claims may stand off further into it
    kept = [Face(w) for w in face.inner_wires()]
    covers = tuple(c for f in fills for c in _cover(
        f, union[0], r, deep, FILL_SAG_WINDOW if any(_meets(f, k) for k in kept) else FILL_SAG))
    return face, covers


def _faces(shape) -> list[Face]:
    """The faces of a 2D boolean's result (a face, a compound or a list of them)."""
    if hasattr(shape, "faces"):
        return list(shape.faces())
    return [f for s in shape for f in s.faces()]


def _meets(f, window) -> bool:
    """Whether the fill ``f`` borders the open window ``window`` (shares an edge's length)."""
    return f.distance_to(window) < 1e-6 and any(
        window.distance_to(e.position_at(0.5)) < 1e-6 for e in f.edges())


def _cover(f, union, r: float, deep: float, sag: float = FILL_SAG) -> list[Cover]:
    """Pills whose union holds ``f``, a piece the closing by ``r`` added to ``union``.

    A piece along one fillet (an arc of radius ``r`` round a centre outside the union: a
    corner's, a cusp's) lies in the annular sector about that centre its boundary spans
    (every point of it at least ``r`` out, at most the furthest its boundary reaches): a
    chain of pills along the sector's middle arc, each over its share of the sector, its
    radius the most any point of that share stands off it, holds it, and stands off the
    fillet into the air by at most ``sag``; its far side runs into the link, so only where
    the sector is no deeper than ``deep`` (the link's narrowest chord). Any other piece (a
    window filled whole, a narrow window's sharp corner) is held by its triangles (its
    tessellation: every edge of the piece bulges into it, so the chords hold it), each by
    three pills from its corners to its incentre."""
    from build123d import GeomType

    arcs = [e for e in f.edges() if e.geom_type == GeomType.CIRCLE
            and abs(e.radius - r) < 1e-6 and union.distance_to(e.arc_center) > 1e-6]
    centres = {(round(e.arc_center.X, 6), round(e.arc_center.Y, 6)) for e in arcs}
    pts = np.array([(v.X, v.Y) for e in f.edges() for v in e.positions(np.linspace(0, 1, 257))])
    if len(centres) == 1:
        o = np.array([arcs[0].arc_center.X, arcs[0].arc_center.Y])
        w = pts - o
        rho = np.hypot(w[:, 0], w[:, 1])
        if rho.min() > r - 1e-6 and rho.max() + FILL_PAD - r <= deep:
            mid = sum((np.array([(m := e.position_at(0.5)).X, m.Y]) - o for e in arcs),
                      np.zeros(2))
            m0 = math.atan2(mid[1], mid[0])
            phi = (np.arctan2(w[:, 1], w[:, 0]) - m0 + math.pi) % (2 * math.pi) - math.pi
            lo, hi = float(phi.min()) - 2e-3, float(phi.max()) + 2e-3
            far = float(rho.max()) + FILL_PAD
            rm = (r + far) / 2
            step = 4 * math.asin(min(1.0, math.sqrt(sag / (4 * rm))))
            n = max(1, math.ceil((hi - lo) / step))
            dl = (hi - lo) / n
            width = (far - r) / 2 + rm * (1 - math.cos(dl / 2)) + 1e-3
            q = [tuple(map(float, o + rm * np.array([math.cos(m0 + lo + j * dl),
                                                      math.sin(m0 + lo + j * dl)])))
                 for j in range(n + 1)]
            return [(q[j], q[j + 1], width) for j in range(n)]
    verts, tris = f.tessellate(0.02, 0.1)
    xy = np.array([(v.X, v.Y) for v in verts])
    out = []
    for i, j, k in tris:
        a, b, c = xy[i], xy[j], xy[k]
        la, lb, lc = (float(np.hypot(*(y - x))) for x, y in ((b, c), (c, a), (a, b)))
        area = abs(float((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]))) / 2
        if area < 1e-9:
            continue
        inc = tuple(map(float, (la * a + lb * b + lc * c) / (la + lb + lc)))
        rin = 2 * area / (la + lb + lc) + 1e-3
        out += [(tuple(map(float, x)), inc, rin) for x in (a, b, c)]
    return out


def _placed_outline(build: Build, ol: Outline, z0: float, z1: float) -> Shape3D:
    """``ol``'s face where its frame's points are at ``build``'s angle, from ``z0`` to
    ``z1``."""
    from build123d import Solid

    pa, pb = build.xy(ol.frame[0]), build.xy(ol.frame[1])
    turn = math.degrees(math.atan2(pb[1] - pa[1], pb[0] - pa[0]))
    face = ol.face.moved(Location((float(pa[0]), float(pa[1]), z0), (0, 0, 1), turn))
    return cast("Shape3D", Solid.extrude(face, Vector(0, 0, z1 - z0)))


class LinkPlates(Group):
    """Every leg link, cut from the sheet, in the layer the plan gives it (an aluminium
    link's end grown round a crank bore its width can't hold: :func:`rider_bosses`); a
    product's with its own bosses and its outline closed (:func:`product_outline`)."""

    name = "links"
    cuts = True

    def claims(self, ctx: Context) -> list[Claim]:
        r = ctx.params.link_radius
        bosses = {n: [boss] for n, boss in rider_bosses(ctx).items()}
        fills: dict[str, list[tuple[str, str, float]]] = {}
        if _product(ctx):       # its bosses, and what closing its outline adds, on its points
            bosses = {n: list(product_bosses(ctx, n, segs).items())
                      for n, segs in ctx.topo.links.items()}
            for n, segs in ctx.topo.links.items():
                outline = product_outline(ctx, n, segs)
                ctx.topo.add_link_points(outline.points(n))
                fills[n] = outline.pills(n)

        feet = {name: foot for name, foot, _ in foot_links(ctx.topo, ctx.config.lk)} \
            if hasattr(ctx.config, "lk") else {}

        def make(link: str, segs):
            def f(L: Layout):
                out = [Placed(L.layers[link], Pill(a, b, _chord_r(ctx, link, a, b)), link,
                              link) for a, b in segs]
                for at, rb in bosses.get(link, ()):     # its end grown round a bore
                    out.append(Placed(L.layers[link], Disc(at, rb), link, f"{link} boss"))
                out += [Placed(L.layers[link], Pill(p, q, w), link, f"{link} fillet")
                        for p, q, w in fills.get(link, ())]
                if link in feet:        # its TPU sock round the toe, in its own layer
                    out.append(Placed(L.layers[link], Disc(feet[link], r + SOCK_REACH), link,
                                      f"{link} sock"))
                return out
            return f

        return [Claim(n, frozenset((n,)), make(n, segs)) for n, segs in ctx.topo.links.items()]

    def assembly(self, view) -> list:
        """Each link with the layer it sits in (:mod:`construction.assembly`), a foot
        link's TPU sock on it first; not a decanting cradle's cradle link (its tie rods'
        hook places it)."""
        from spiderpig.construction.assembly import STACK, Op, whole
        from spiderpig.stack import body_class

        cfg = getattr(view.ctx, "config", None)
        lk = getattr(cfg, "lk", None)
        skip = lk.output.link if lk is not None and lk.product and lk.output else None
        ops = []
        for link in getattr(getattr(view.ctx, "topo", None), "links", {}):
            if link not in view.z or (skip is not None and body_class(link) == skip):
                continue
            k = view.slot(view.z[link][0])
            ops.append(Op(STACK, (k, 1), whole(link), "",
                          "Place the layer's links on their pillars and pins.", "layer"))
            if f"{link}_sock" in view.z:
                ops.append(Op(STACK, (k, 1), whole(f"{link}_sock"), "",
                              "Each foot link takes its TPU sock, slid on, before it goes on.",
                              "layer"))
        return ops

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
            if _product(ctx):   # its outline closed, placed where its points are
                part = cut_holes(_placed_outline(build, product_outline(ctx, name, segs),
                                                 z0, z1), cuts, z0, z1)
            else:
                boss = bosses.get(name)
                part = plate([(build.xy(a), build.xy(b), _chord_r(ctx, name, a, b))
                              for a, b in segs], z0, z1, cuts,
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
    ring = difference(disc(tuple(f), r + SOCK_T, z0, z1), disc(tuple(f), r, z0 - 1, z1 + 1))
    half = box(tuple(f + d * (r + SOCK_T) / 2), (r + SOCK_T + 0.01, 2 * (r + SOCK_T) + 1,
                                                z1 - z0), z0, ang)
    legs = [box(tuple(f - d * (NOTCH_BACK + 0.5) / 2 + s * n * (r + SOCK_T / 2)),
                (NOTCH_BACK + 0.5 + 0.02, SOCK_T, z1 - z0), z0, ang) for s in (-1, 1)]
    lugs = [box(tuple(f - d * NOTCH_BACK + s * n * (r - NOTCH[1] / 2 + 0.05)),
                (NOTCH[0] - 0.2, NOTCH[1], z1 - z0), z0, ang) for s in (-1, 1)]
    sock = union([intersection(ring, half), *legs, *lugs])
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

    def assembly(self, view) -> list:
        """The outer plate first, bare, leg side up; the inner plate the first part of its
        unit, servo side up (:mod:`construction.assembly`)."""
        from spiderpig.construction.assembly import STACK, UNIT, Op, whole

        topo = getattr(view.ctx, "topo", None)
        inner = topo.frame_bodies[0] if topo is not None and topo.frame_bodies else None
        ops = []
        if "frame_outer" in view.z:
            ops.append(Op(STACK, (-1, 0), whole("frame_outer"),
                          "Outer frame plate and pillar columns",
                          "Lay the outer frame plate down, leg side up.", "plate"))
        if inner is not None and inner in view.z:
            ops.append(Op(UNIT, (0,), whole(inner), "Inner plate and servo",
                          "Lay the inner frame plate down, servo side up.", "plate"))
        return ops

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
