"""The body's underside: what a stair edge meets before the feet do.

The **body** is everything on a side that isn't a leg link: the frame plates
(:class:`construction.plates.FramePlates`: a disc on O, arms to the pillars,
the pad under the servo), the crank's sweep (a circle about O, crankpin
distance + web radius), the servo's footprint (:mod:`servos.mount`: mounted
at O on the inner plate, pointing away from the pillars) and the robot's
centre plates (:mod:`construction.chassis`: the servo footprint grown by the
rear screws and the frame ties). :class:`Underside` is its profile in the
plane of motion: for each x, the lowest y it reaches over a crank cycle. The
planner keeps what it adds to the crank above it (:meth:`Underside.allows`),
and a design reports how high it rides (:meth:`Underside.clearance`).

Claims-level geometry: shapes are discs, capsules and the servo's and centre
plates' rectangles, as the groups claim or outline them, not the fabricated
solids.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from spiderpig.construction.base import Context
from spiderpig.construction.chassis import _footprint, centre_plates, rear_screws, tie_dims

STEP = 0.25     # mm between profile samples along x


@dataclass(frozen=True)
class Underside:
    """``ys[i]``: the lowest y the body reaches at ``xs[i]`` (``inf`` where it has nothing);
    ``lowest_part``: which of the body's shapes reaches lowest (what a designer would
    move to gain ground clearance)."""

    xs: np.ndarray
    ys: np.ndarray
    o: tuple[float, float]      # the crank axis O
    lowest_part: str = ""

    @property
    def lowest(self) -> float:
        return float(self.ys.min())

    def allows(self, margin: float) -> float:
        """The largest circle about O that stays ``margin`` above the profile, within the
        body's x-extent: an added crank feature turns with the crank, so it sweeps a full
        circle about O, and must fit in it."""
        dx = self.xs - self.o[0]
        depth = self.o[1] - self.ys - margin          # how far below O it may reach at x
        reach = np.where(depth >= 0, np.hypot(dx, np.maximum(depth, 0.0)), np.abs(dx))
        edge = min(self.o[0] - self.xs[0], self.xs[-1] - self.o[0])
        return float(min(reach.min(), edge))

    def clearance(self, lowest_foot: float) -> float:
        """How far the body's lowest point rides above the lowest foot point (mm)."""
        return self.lowest - lowest_foot


def _disc(xs, c, r):
    dx = xs - c[0]
    inside = np.abs(dx) <= r
    return np.where(inside, c[1] - np.sqrt(np.maximum(r * r - dx * dx, 0.0)), np.inf)


def _pill(xs, a, b, r, n: int = 64):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return np.min([_disc(xs, a + (b - a) * t, r) for t in np.linspace(0.0, 1.0, n)], axis=0)


def _polygon(xs, pts):
    """Lower boundary of a convex polygon."""
    out = np.full_like(xs, np.inf)
    for p, q in zip(pts, pts[1:] + pts[:1], strict=True):
        (x0, y0), (x1, y1) = p, q
        if abs(x1 - x0) < 1e-12:
            continue
        lo, hi = min(x0, x1), max(x0, x1)
        on = (xs >= lo) & (xs <= hi)
        y = y0 + (y1 - y0) * (xs - x0) / (x1 - x0)
        out = np.where(on, np.minimum(out, y), out)
    return out


def body_shapes(ctx: Context, crank_reach: float | None) -> list[tuple]:
    """The body's shapes in side coordinates, each named: ``(label, "disc", c, r)``,
    ``(label, "pill", a, b, r)`` and ``(label, "poly", [corners])``. ``crank_reach``: the
    radius the crank sweeps about O."""
    from spiderpig.servos.mount import away_from_pillars

    topo, p, spec = ctx.topo, ctx.params, ctx.servo
    pts = topo.geometry.points
    o = np.asarray(pts["O"][0], float)
    frame = topo.axes_of("frame")
    pillars = [np.asarray(pts[a.name][0], float) for a in frame]
    out: list[tuple] = [("the frame plates' disc at O", "disc", o, p.frame_radius)]
    out += [(f"the frame plates' arm to pillar {a.name}", "pill", o, q, p.frame_radius)
            for a, q in zip(frame, pillars, strict=True)]
    if crank_reach:
        out.append(("the crank's sweep", "disc", o, crank_reach))
    u = away_from_pillars(o, pillars)
    v = np.array([u[1], -u[0]])

    def world(x, y):
        return o + x * u + y * v

    def rect(label, x0, x1, y0, y1):
        return (label, "poly",
                [tuple(world(x, y)) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))])

    x0, x1, y0, y1 = _footprint(spec)
    out.append(rect(f"the servo's body ({spec.key})", x0, x1, y0, y1))
    half = (y1 - y0) / 2 + p.min_wall                             # its pad on the inner plate
    out.append(("the servo's pad on the inner frame plate", "pill",
                world(x0 + half, 0), world(x1 - half, 0), half * math.sqrt(2)))
    xs, ys = [x0, x1], [y0, y1]                                   # the centre plates
    try:
        rs = rear_screws(spec, centre_plates(spec, ctx.pitch, p.margin), ctx.pitch)
        d = tie_dims(ctx)
    except ValueError:          # no chassis for this servo: the servo alone
        return out
    if rs is not None:
        rr = rs.head_d / 2 + 0.3 + p.min_wall
        for h in rs.holes:     # each servo's own holes: +y on the left, -y on the right
            xs += [h.x - rr, h.x + rr]
            ys += [h.y + rr, -h.y - rr]
    tr, yt = d.column + 1.0, y1 + p.margin + d.column
    tx = (x0 + d.column, x1 - d.column) if x1 - x0 > 2 * d.column else ((x0 + x1) / 2,)
    xs += [x - tr for x in tx] + [x + tr for x in tx]
    ys += [yt + tr, -yt - tr]
    out.append(rect("the centre plates (the chassis between the servos)",
                    min(xs), max(xs), min(ys), max(ys)))
    return out


def underside(ctx: Context, crank_reach: float | None) -> Underside:
    """The body's underside profile (see the module docstring)."""
    shapes = body_shapes(ctx, crank_reach)
    lo, hi = math.inf, -math.inf
    for _label, kind, *g in shapes:
        if kind == "disc":
            lo, hi = min(lo, g[0][0] - g[1]), max(hi, g[0][0] + g[1])
        elif kind == "pill":
            lo = min(lo, g[0][0] - g[2], g[1][0] - g[2])
            hi = max(hi, g[0][0] + g[2], g[1][0] + g[2])
        else:
            lo, hi = min([lo] + [x for x, _ in g[0]]), max([hi] + [x for x, _ in g[0]])
    xs = np.arange(lo, hi + STEP, STEP)
    ys = np.full_like(xs, np.inf)
    lowest_part, lowest = "", math.inf
    for label, kind, *g in shapes:
        f = {"disc": lambda g: _disc(xs, g[0], g[1]), "pill": lambda g: _pill(xs, *g),
             "poly": lambda g: _polygon(xs, g[0])}[kind]
        prof = f(g)
        ys = np.minimum(ys, prof)
        if (low := float(prof.min())) < lowest - 1e-9:
            lowest_part, lowest = label, low
    o = ctx.topo.geometry.points["O"][0]
    return Underside(xs, ys, (float(o[0]), float(o[1])), lowest_part)
