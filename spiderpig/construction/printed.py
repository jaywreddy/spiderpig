"""Printed stepped axles: segments that snap together.

An axle's claim (:meth:`construction.axle.AxleGroup.claims`) is a column of
discs, one per layer: a head or cap at a free end, an anchor through a frame
plate, a bearing (``axle``) through each link, a shoulder beside each link
and a neck elsewhere. :func:`plan_segments` turns that column into a radius
profile along the axis and splits it into printable segments;
:func:`segment_solid` builds one segment as a solid of revolution.

Profile
    Each layer gets its claimed radius. A shoulder, head or cap beside a
    link stops ``play`` short of the link's face, and the bearing radius
    runs on into that gap, so the link turns freely between the two
    shoulders that hold it. Where the radius grows going up (a neck into a
    shoulder), it grows along a 45 degree cone so it prints without support.

Segments
    The column is split just above every run of adjacent links that has a
    shoulder or cap above it (at ``z(top of the run) + play``). Each segment
    therefore ends in the bearing its links turn on, and its links are
    threaded on from above before the next segment snaps on. A run with a
    frame plate above it isn't split: the plate retains its links and the
    bearing carries on through the plate as the anchor.

Snap joint (:class:`Snap`)
    A split peg stands on top of every segment but the last; the next segment
    has a socket in the bottom of its shoulder (or cap). Going up the peg:
    a round shank (``shank`` radius, ``shank_h`` tall) that locates the
    segments sideways in the socket's throat, a barb (``barb`` radius,
    ``land_h`` tall) and a 45 degree lead-in back to the shank radius. The
    barb is trimmed by two flats (``flats`` half-width) so both prongs can
    pass the throat. The socket is the peg grown by ``clearance`` all round
    (radially and axially): a throat ``engage`` narrower than the barb, a
    chamber the barb clicks into, a 45 degree roof and a flat ceiling. So the
    barb hooks under a ledge ``shank_h - clearance`` thick, and the segments
    can separate by at most ``clearance`` once snapped.

    The slot that splits the peg into two prongs runs on down through the
    bearing and whatever is below it, up to ``slot_max`` below the peg's tip,
    so the prongs are long enough to flex: each prong's tip moves
    :meth:`Snap.deflection` (about 0.3 mm) while the barb passes the throat.
    Below the slot's rounded root the segment stays stiff, or the prongs
    would hinge there and the bearing would give under its links: ``base``
    of solid above a bottom face, or the whole layer holding a socket. The
    slot never enters an anchor (glued into its plate).

    :meth:`Segment.strain` estimates the peak strain in the prongs while
    snapping (a stepped cantilever from the slot's root). The wide shoulders
    and heads barely bend, so a short bearing over one of them strains most:
    about 2 % for a pin through two links, up to 4 % for one link over a
    shoulder. Print axles in PETG (about 4 % is fine for one-time assembly;
    6-7 % is past the yield of printed PLA or PETG prongs). Above
    ``max_strain`` :func:`plan_segments` first runs the slot deeper (past
    ``slot_max``, down to what the segment allows), then relieves that
    joint's lip (a smaller ``engage``, down to ``min_engage``: the barb has
    less to climb, so it holds less too) and records the joint's own
    :class:`Snap` on the two segments it joins (``peg_snap`` / ``socket_snap``);
    when neither brings the strain down it raises ``ValueError`` (the pin
    can't be snapped without breaking). ``Segment.strain_pct`` and
    ``engage`` are what :mod:`tools.audit` reports per pin.

Printing (every segment is modelled z-up, as printed)
    Each segment stands on a flat bottom face: a head, a shoulder or a cap,
    with the socket opening in it. Steps in going up are ledges, steps out
    are 45 degree cones, the socket roof is a 45 degree cone. Two small
    overhangs remain: the underside of the barb (``engage + clearance``,
    0.4 mm, the face that holds the joint) and the socket's flat ceiling, a
    bridge about 5.5 mm across.
"""

from __future__ import annotations

import dataclasses
import logging
import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from functools import cache

import numpy as np
from build123d import Axis, Box, Cylinder, Face, Location, Solid, Vector, Wire

from spiderpig.shapes import moved

log = logging.getLogger(__name__)

TAN_22_5 = math.sqrt(2.0) - 1.0   # offsets a 45 degree corner by a normal distance
EPS = 1e-9

Outline = list[tuple[float, float]]   # (r, z) points of a half profile


@dataclass(frozen=True)
class Snap:
    """A split snap peg and its socket (radii about the axis, heights from the split).

    ``barb``: barb radius; ``engage``: how far the barb overlaps the socket's
    ledge (radial); ``clearance``: gap between peg and socket, radial and
    axial; ``shank_h``: shank height; ``land_h``: height of the barb's
    cylindrical land; ``flats``: half-width across the barb flats; ``slot``:
    width of the slot that splits the peg.
    """

    barb: float
    engage: float
    clearance: float
    shank_h: float
    land_h: float
    flats: float
    slot: float

    @property
    def throat(self) -> float:
        """Radius of the socket's mouth, the narrowest the barb must pass."""
        return self.barb - self.engage

    @property
    def shank(self) -> float:
        return self.throat - self.clearance

    @property
    def chamber(self) -> float:
        """Radius of the chamber the barb clicks into."""
        return self.barb + self.clearance

    @property
    def tip(self) -> float:
        """Radius at the top of the 45 degree lead-in (the shank radius)."""
        return self.shank

    @property
    def lead_h(self) -> float:
        return self.barb - self.tip

    @property
    def height(self) -> float:
        """How far the peg stands above the split."""
        return self.shank_h + self.land_h + self.lead_h

    @property
    def depth(self) -> float:
        """How deep the socket goes above the split."""
        return self.height + self.clearance

    def deflection(self) -> float:
        """Radial deflection each prong needs while the barb passes the throat.

        The prongs close towards the slot; the corners of the barb at the
        flats are the last to clear the throat.
        """
        f = min(self.flats, self.throat)
        return math.sqrt(self.barb**2 - f**2) - math.sqrt(self.throat**2 - f**2)

    def peg(self, z: float) -> Outline:
        """The peg's outline from the shank's root at ``z`` up to the axis at its tip."""
        zb = z + self.shank_h
        return [(self.shank, z), (self.shank, zb), (self.barb, zb),
                (self.barb, zb + self.land_h), (self.tip, z + self.height),
                (0.0, z + self.height)]

    def socket(self, z: float) -> Outline:
        """The socket's outline from the axis at its ceiling down to its mouth at ``z``.

        It is :meth:`peg` grown by ``clearance`` (normal to every face).
        """
        c, k = self.clearance, self.clearance * TAN_22_5
        zb = z + self.shank_h
        return [(0.0, z + self.depth), (self.tip + k, z + self.depth),
                (self.chamber, zb + self.land_h + k), (self.chamber, zb - c),
                (self.throat, zb - c), (self.throat, z)]


@dataclass(frozen=True)
class Piece:
    """A stretch of an axle's profile: radius ``r`` from ``z0`` to ``z1``.

    ``role`` is the claim it lies in (``axle``, ``anchor``, ``head``, ``cap``,
    ``shoulder``, ``neck``), or ``play`` for the bearing running on into the
    gap between a link and the shoulder beside it.
    """

    z0: float
    z1: float
    r: float
    role: str
    layer: int


@dataclass
class Segment:
    """One printed piece of an axle, bottom to top."""

    index: int
    pieces: list[Piece]
    links: tuple[str, ...] = ()          # links that turn on its bearing
    socket: float | None = None          # split below: its socket opens at this z
    peg: float | None = None             # split above: its peg stands at this z
    slot_root: float | None = None       # lowest point of the peg's slot
    anchors: tuple[int, ...] = field(default=())   # frame-plate layers it is glued in
    peg_snap: Snap | None = None         # this joint's snap when relieved (else the axle's)
    socket_snap: Snap | None = None      # the joint below's, likewise
    strain_pct: float | None = None      # peak prong strain while snapping, % (pegs only)

    @property
    def z0(self) -> float:
        return self.pieces[0].z0

    @property
    def z1(self) -> float:
        return self.pieces[-1].z1

    def flex(self, snap: Snap) -> float | None:
        """Length of the peg's prongs, from the slot's root to the barb."""
        if self.peg is None or self.slot_root is None:
            return None
        return self.peg + (self.peg_snap or snap).shank_h - self.slot_root

    def strain(self, snap: Snap) -> float | None:
        """Peak bending strain in the prongs while the barb passes the throat.

        Each prong is a cantilever from the slot's root, stepped like the
        axle (the shank above the split), pushed at the barb until it has
        moved :meth:`Snap.deflection`. The wide parts (shoulders, heads)
        barely bend, so the strain peaks where the prong narrows.
        """
        if self.peg is None or self.slot_root is None:
            return None
        snap = self.peg_snap or snap
        root, zb = self.slot_root, self.peg + snap.shank_h
        spans = [(max(p.z0, root), p.z1, p.r) for p in self.pieces if p.z1 > root + EPS]
        spans.append((self.peg, zb, snap.shank))
        compliance, peak = 0.0, 0.0
        for z0, z1, r in spans:
            i, c = prong_section(r, snap.slot)
            compliance += ((zb - z0) ** 3 - (zb - z1) ** 3) / (3 * i)
            peak = max(peak, (zb - z0) * c / i)
        return snap.deflection() / compliance * peak


@cache
def prong_section(r: float, slot: float) -> tuple[float, float]:
    """(second moment of area, extreme fibre) of one prong bending towards the slot.

    A prong's section is the disc of radius ``r`` beyond the slot
    (``x >= slot / 2``).
    """
    x = np.linspace(slot / 2, r, 513)
    h = 2 * np.sqrt(np.clip(r * r - x * x, 0.0, None))
    area = np.trapezoid(h, x)
    xc = np.trapezoid(x * h, x) / area
    return float(np.trapezoid((x - xc) ** 2 * h, x)), float(max(xc - slot / 2, r - xc))


RETAINERS = ("shoulder", "head", "cap")


def plan_segments(
    column: Mapping[int, tuple[str, float]],
    z: Callable[[int], tuple[float, float]],
    links: Mapping[int, tuple[str, ...]],
    *,
    axle: float,
    snap: Snap,
    play: float,
    bridge: float,
    base: float,
    slot_max: float,
    min_prong: float = 0.8,
    max_strain: float = 1.0,
    min_engage: float | None = None,
    name: str = "axle",
    strain_target: float | None = None,
) -> list[Segment]:
    """Split an axle's claimed column into snap-together segments.

    ``column`` maps each layer to its claim's (role, radius); ``z`` gives a
    layer's Z range; ``links`` the links that turn on the axle, per layer;
    ``axle`` is the bearing radius. A prong strained more than
    ``max_strain`` while snapping gets a deeper slot, then a relieved lip
    (``engage`` down to ``min_engage``; ``None``: the axle's); past both it is
    a ``ValueError`` (see the module doc). ``strain_target`` (at most ``max_strain``)
    is the margin the planner aims for: a prong over it is relieved the same way, down to
    the target when the lip allows, else to just under ``max_strain``.
    """
    ks = sorted(column)
    if ks != list(range(ks[0], ks[-1] + 1)):
        raise ValueError(f"{name}: its claim skips a layer ({ks})")
    member = {k for k in ks if column[k][0] == "axle"}
    pieces: list[Piece] = []
    for k in ks:
        role, r = column[k]
        z0, z1 = z(k)
        if role in RETAINERS:      # stop short of the links beside it
            lo = z0 + play if k - 1 in member else z0
            hi = z1 - play if k + 1 in member else z1
            if lo > z0:
                pieces.append(Piece(z0, lo, min(axle, r), "play", k))
            pieces.append(Piece(lo, hi, r, role, k))
            if hi < z1:
                pieces.append(Piece(hi, z1, min(axle, r), "play", k))
        else:
            pieces.append(Piece(z0, z1, r, role, k))

    # split above every run of links that a shoulder or cap retains
    splits = sorted(z(k + 1)[0] + play for k in member
                    if k + 1 in column and column[k + 1][0] in ("shoulder", "cap"))
    groups: list[list[Piece]] = [[]]
    for p in pieces:
        if groups[-1] and any(abs(p.z0 - s) < EPS for s in splits):
            groups.append([])
        groups[-1].append(p)
    if len(groups) != len(splits) + 1:
        raise ValueError(f"{name}: a split doesn't fall between two pieces")

    segs: list[Segment] = []
    for i, g in enumerate(groups):
        layers = {p.layer for p in g if p.role == "axle"}
        seg = Segment(
            index=i, pieces=_merge(g),
            links=tuple(n for k in sorted(layers) for n in links.get(k, ())),
            socket=g[0].z0 if i > 0 else None,
            peg=g[-1].z1 if i < len(groups) - 1 else None,
            anchors=tuple(sorted({p.layer for p in g if p.role == "anchor"})),
        )
        if seg.peg is not None:
            thin = snap.slot / 2 + min_prong
            seg.slot_root = _slot_root(seg, g[0].z1, snap, bridge, base, slot_max, thin)
            strain = was = seg.strain(snap)
            goal = max_strain if strain_target is None else min(strain_target, max_strain)
            if strain > goal:
                low = snap.engage if min_engage is None else min_engage
                root = seg.slot_root
                strain = _relieve(seg, g[0].z1, snap, bridge, base, thin, goal, low, name,
                                  limit=max_strain)
                if strain > goal + EPS:         # the target is out of reach: the limit will do
                    seg.slot_root, seg.peg_snap, strain = root, None, was
                    if was > max_strain:
                        strain = _relieve(seg, g[0].z1, snap, bridge, base, thin, max_strain,
                                          low, name)
            seg.strain_pct = 100.0 * strain
        if i and segs[-1].peg_snap is not None:
            seg.socket_snap = segs[-1].peg_snap
        segs.append(seg)
    return segs


def _relieve(seg: Segment, first: float, snap: Snap, bridge: float, base: float, thin: float,
             max_strain: float, min_engage: float, name: str,
             limit: float | None = None) -> float:
    """Bring a peg's snapping strain under ``max_strain``: the slot as deep as the segment
    allows, then the joint's lip relieved in 0.01 mm steps down to ``min_engage`` (the
    joint's own :class:`Snap` goes on ``seg.peg_snap``). The strain reached; ``ValueError``
    when even that is over ``limit`` (``max_strain`` when not given)."""
    limit = max_strain if limit is None else limit
    was = seg.strain(snap)
    seg.slot_root = _slot_root(seg, first, snap, bridge, base, math.inf, thin)
    strain = seg.strain(snap)
    if strain <= max_strain:
        log.info("%s seg%d: snap prongs run deeper (%.1f mm): strain %.1f %% -> %.1f %%",
                 name, seg.index, seg.flex(snap), 100 * was, 100 * strain)
        return strain
    engage = snap.engage
    while strain > max_strain and engage - 0.01 >= min_engage - EPS:
        engage = round(engage - 0.01, 6)
        eased = dataclasses.replace(snap, engage=engage)
        seg.slot_root = _slot_root(seg, first, eased, bridge, base, math.inf, thin)
        strain = seg.strain(eased)
        seg.peg_snap = eased
    if strain > limit:
        raise ValueError(
            f"{name} seg{seg.index}: its snap prongs ({seg.flex(snap):.1f} mm) would strain "
            f"{100 * strain:.1f} % while snapping even with the lip relieved to "
            f"{engage:.2f} mm (want at most {100 * max_strain:.1f} %): the pin can't be "
            "snapped together without breaking (a metal pin, --pin rod or bolt, would do)")
    log.info("%s seg%d: snap lip relieved to %.2f mm engage: strain %.1f %% -> %.1f %%", name,
             seg.index, engage, 100 * was, 100 * strain)
    return strain


def _merge(pieces: list[Piece]) -> list[Piece]:
    out: list[Piece] = []
    for p in pieces:
        if out and abs(out[-1].r - p.r) < EPS and out[-1].role == p.role:
            q = out.pop()
            p = Piece(q.z0, p.z1, p.r, p.role, q.layer)
        out.append(p)
    return out


def _slot_root(seg: Segment, first: float, snap: Snap, bridge: float, base: float,
               slot_max: float, thin: float) -> float:
    """The slot runs as deep as allowed, at most ``slot_max`` below the peg's tip.

    Below its root the segment must be stiff, or the prongs would hinge
    there and the bearing would give under the links: ``base`` of solid above
    the bottom face, or the whole layer holding the socket (up to ``first``,
    at least ``bridge`` above the socket). Never into an anchor (glued in its
    plate) or anything thinner than ``thin``.
    """
    socketed = seg.socket is not None
    floor = max(seg.z0 + snap.depth + bridge, first) if socketed else seg.z0 + base
    for p in seg.pieces:
        if p.role == "anchor" or p.r < thin - EPS:
            floor = max(floor, p.z1)
    return max(floor, seg.peg + snap.height - slot_max)


def outline(seg: Segment, snap: Snap) -> Outline:
    """The segment's closed half profile (r, z), counter-clockwise from its bottom face."""
    pts: Outline = []
    cur = None
    for p in seg.pieces:
        if cur is None:
            cur = p.r
            pts.append((cur, p.z0))
        elif p.r < cur:                     # step in going up: a ledge
            pts.append((cur, p.z0))
            cur = p.r
            pts.append((cur, p.z0))
        elif p.r > cur:                     # step out going up: a 45 degree cone
            rise = min(p.r - cur, p.z1 - p.z0)
            pts.append((cur, p.z0))
            cur += rise
            pts.append((cur, p.z0 + rise))
        pts.append((cur, p.z1))
    if seg.peg is not None:
        pts += (seg.peg_snap or snap).peg(seg.z1)
    else:
        pts.append((0.0, seg.z1))
    if seg.socket is not None:
        pts += (seg.socket_snap or snap).socket(seg.z0)
    else:
        pts.append((0.0, seg.z0))
    return _clean(pts)


def _clean(pts: Outline) -> Outline:
    """Drop repeated points and the middle one of three collinear points."""
    out: Outline = []
    for p in pts:
        if out and math.dist(out[-1], p) < 1e-7:
            continue
        out.append(p)
        while len(out) >= 3 and _collinear(*out[-3:]):
            del out[-2]
    while len(out) >= 3 and (math.dist(out[0], out[-1]) < 1e-7
                             or _collinear(out[-2], out[-1], out[0])):
        out.pop()
    while len(out) >= 3 and _collinear(out[-1], out[0], out[1]):
        del out[0]
    return out


def _collinear(a, b, c) -> bool:
    return abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) < 1e-9


def segment_solid(seg: Segment, snap: Snap, xy, angle: float = 0.0) -> Solid:
    """The segment in world coordinates: its axis vertical through ``xy``.

    The peg's slot runs along direction ``angle`` (radians, world XY).
    """
    pts = outline(seg, snap)
    snap = seg.peg_snap or snap             # the peg's own joint when relieved
    wire = Wire.make_polygon([Vector(r, 0.0, zz) for r, zz in pts], close=True)
    solid = Solid.revolve(Face(wire), 360.0, Axis.Z)
    if seg.peg is not None:
        span = 2.0 * max(r for r, _ in pts) + 2.0
        top = seg.peg + snap.height + 1.0
        # the slot (along local X) that splits the peg into two prongs, with a round root
        lo = seg.slot_root + snap.slot / 2
        cutters = [
            Box(span, snap.slot, top - lo).moved(Location((0.0, 0.0, (lo + top) / 2))),
            Cylinder(snap.slot / 2, span).rotate(Axis.Y, 90).moved(Location((0.0, 0.0, lo))),
        ]
        # flats across the barb, so the prongs' corners clear the throat
        zb = seg.peg + snap.shank_h
        for side in (-1, 1):
            x = side * (snap.flats + span / 2)
            cutters.append(Box(span, span, top - zb).moved(Location((x, 0.0, (zb + top) / 2))))
        solids = solid.cut(*cutters).solids()
        if len(solids) != 1:
            raise ValueError(f"segment {seg.index}: the slot splits it into {len(solids)} parts")
        solid = solids[0]
    if angle:
        solid = solid.rotate(Axis.Z, math.degrees(angle))
    return moved(solid, Location((float(xy[0]), float(xy[1]), 0.0)))
