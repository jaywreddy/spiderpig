"""A reference layer planner for small designs: every layering, every crank route.

Independent of :mod:`stack`'s search and :mod:`construction.route`'s router:
it enumerates link layers (skipping only pairs of links whose own shapes
collide, which no route can change), builds every claim for each, and tries
every crank route: sets of runs (a run point and a layer range, webs in the
layers either side, ``1..`` below the hub) that put each rider in a run of
its own crankpin. A layering and route count when no two shapes of different
groups come too close in a layer (the planner's own distances) and nothing but
seats sits in a frame plate's layer, and the printed crank can build the route
(:func:`buildable`, its joints as ``realize`` makes them). Cost: the planner's objective, in order:
added crank features (run layers no rider of their point is in, detour runs),
the detours' sweep, a dropped bearing.
"""

from __future__ import annotations

import itertools
import math
from collections.abc import Iterator

import numpy as np

from spiderpig import construction
from spiderpig.construction.crank import NUT_AF, POST_SCREWS, CrankRoute, Run
from spiderpig.stack import Layout, Placed, StackProblem, made


def routes(points: list[str], lo: int, hi: int, need: dict[int, str],
           bearing: tuple[bool, ...] = (True,)) -> Iterator[CrankRoute]:
    """Every route whose runs sit in layers ``lo..hi``, one layer apart at least, with the
    layers in ``need`` (layer -> point) inside a run along their point."""

    def rec(k: int) -> Iterator[tuple[Run, ...]]:
        # every layer below k that needs a run is in one; the next run starts at k or later
        if all(j < k for j in need):
            yield ()
        for a in range(k, hi + 1):
            if any(k <= j < a for j in need):
                break                       # a layer that needs a run would be left out
            for b in range(a, hi + 1):
                pts = {need[j] for j in need if a <= j <= b}
                if len(pts) > 1:
                    break
                if b + 1 in need:           # its upper web can't sit in a rider's layer
                    continue
                for p in pts or points:
                    for rest in rec(b + 2):
                        yield (Run(p, a, b), *rest)

    if any(not lo <= j <= hi for j in need):
        return
    for runs in rec(lo):
        if runs:
            for keep in bearing:
                yield CrankRoute(runs, keep)


def cost(route: CrankRoute, layers: dict[str, int], riders: dict[str, str],
         detours: dict[str, float]) -> tuple[int, float, int]:
    ridden = {(k, riders[n]) for n, k in layers.items() if n in riders}
    extra = sum(1 for r in route.runs for k in range(r.lo, r.hi + 1) if (k, r.at) not in ridden)
    extra += sum(1 for r in route.runs if r.at in detours)
    return extra, sum(detours[r.at] for r in route.runs if r.at in detours), int(not route.bearing)


def buildable(route: CrankRoute, layers: dict[str, int], problem: StackProblem, ctx,
              h0: int) -> bool:
    """The printed crank's joints, as :meth:`construction.crank.PrintedCrank.realize` makes
    them: runs along one point whose webs meet are one chain (one point, one chain), its
    screw must fit between its outer webs' faces (end play set back where a rider turns
    against them), a web set back in the hub's lowest layer (``h0``) must leave the hub its
    horn screws, and pockets in one segment (consecutive chains, the last chain and the
    horn screws) must not meet."""
    crank, drive = construction.crank(ctx.config.crank), ctx.interfaces["drive"]
    geo, pitch, play = problem.topo.geometry, problem.spec.pitch, crank.axial_play
    riders: dict[str, set[int]] = {}
    for n, pin in problem.topo.riders.items():
        riders.setdefault(pin, set()).add(layers[n])
    runs = sorted(route.runs, key=lambda r: (r.lo, r.at))
    below, above = set(), set()
    for r in runs:
        ridden = riders.get(r.at, set()) & set(range(r.lo, r.hi + 1))
        if r.lo in ridden:
            below.add(r.lo - 1)
        if r.hi in ridden and len(ridden) <= r.hi - r.lo:
            above.add(r.hi + 1)
    chains: list[list] = []
    for r in runs:
        if chains and chains[-1][-1].at == r.at and r.lo - chains[-1][-1].hi <= 3:
            chains[-1].append(r)
        else:
            chains.append([r])
    if len({c[0].at for c in chains}) < len(chains):
        return False
    two = getattr(crank, "two_layer_top", False)     # the keyed crank's two-layer top web

    def face(k: int, top: bool) -> float:
        return k * pitch + play * (k in above) if top else (k + 1) * pitch - play * (k in below)

    for c in chains:
        lo, hi = c[0].lo - 1, c[-1].hi + 1
        faces = [face(lo, True), face(lo, False), face(hi, True),
                 face(hi + 1 if two else hi, False)]
        if two:
            if hi + 1 in {k for r in runs for k in range(r.lo, r.hi + 1)} or hi + 1 > h0:
                return False
            faces.append(face(c[0].hi + 1, True))       # the first post's top
        if crank.post_joint(*faces) is None:
            return False
    if h0 in above and crank.hub_joint(ctx, problem.router.dims.hub_thickness, play) is None:
        return False
    xy = {p: geo.points[p][0] for p in {r.at for r in runs}}
    head = max(sk.head_d for sk in POST_SCREWS) / 2 + crank.screw_fit / 2
    nut = (NUT_AF + crank.nut_fit) / math.sqrt(3)
    if two:
        nut = max(nut, (crank.pocket_af() + 2 * crank.pocket_chamfer) / math.sqrt(3))
    post = problem.router.dims.post
    for x, y in itertools.pairwise(chains):
        if math.dist(xy[x[0].at], xy[y[0].at]) < nut + max(head, post):
            return False
    o, pin = geo.points["O"][0], geo.points[problem.topo.axes_of("crankpin")[0].name][0]
    theta = math.atan2(pin[1] - o[1], pin[0] - o[0]) + drive.pattern_angle
    horn = [(o + drive.screw_pcd / 2 * np.array([math.cos(a), math.sin(a)]), drive.screw_head_d / 2)
            for a in (theta + 2 * math.pi * k / drive.screw_count
                      for k in range(drive.screw_count))]
    if drive.center_head_d > 0:
        horn.append((o, (drive.center_head_d + ctx.params.print_fit) / 2))
    return all(math.dist(xy[chains[-1][0].at], h) >= nut + r for h, r in horn)


def solve(problem: StackProblem, top: int, ctx,
          only=None) -> tuple[tuple, dict[str, int], CrankRoute] | None:
    """The cheapest layering and route in ``top + 1`` layers (``ctx``: the side's context, for
    the crank's construction and the drive), or ``None``. ``only(layers, h0)``: the routes
    to try instead of every route (e.g. one run per crankpin over the whole stack)."""
    geo, m, pitch = problem.topo.geometry, problem.spec.margin, problem.spec.pitch
    router = problem.router
    links = list(problem.links)
    riders = problem.topo.riders
    h0 = router.hub_bottom(top)
    detours = {d.name: d.sweep for d in router.facts.detours}
    points = [a.name for a in problem.topo.axes_of("crankpin")] + list(detours)
    fixed = [c for c in problem.claims if c.choice is None]
    routed = [c for c in problem.claims if c.choice is not None]

    def clear(a: Placed, b: Placed) -> bool:
        return (a.seat or b.seat or a.layer != b.layer or a.group == b.group
                or geo.dist(a.shape.core, b.shape.core) >= a.shape.r + b.shape.r + m)

    def own(n: str, k: int) -> list[Placed]:
        return [p for c in fixed if c.deps == {n}
                for p in (made(c, Layout({n: k}, top, pitch))[0] or ())]

    solo = {(n, k): own(n, k) for n in links for k in range(1, top)}
    apart = {(a, b, k): all(clear(p, q) for p in solo[(a, k)] for q in solo[(b, k)])
             for a, b in itertools.combinations(links, 2) for k in range(1, top)}
    memo: dict[tuple, list[Placed] | None] = {}

    def built(c, layers) -> list[Placed] | None:
        key = (id(c), *(layers[d] for d in sorted(c.deps)))
        if key not in memo:
            memo[key] = made(c, Layout(layers, top, pitch))[0]
        return memo[key]

    def layerings(i: int, layers: dict[str, int]) -> Iterator[dict[str, int]]:
        """Every layering, skipping only links whose own shapes collide in a shared layer."""
        if i == len(links):
            yield dict(layers)
            return
        n = links[i]
        for k in range(1, top):
            if all(layers[m] != k or apart[(m, n, k)] for m in links[:i]):
                layers[n] = k
                yield from layerings(i + 1, layers)
                del layers[n]

    best = None
    for layers in layerings(0, {}):
        shapes: list[Placed] = []
        for c in fixed:
            out = built(c, layers)
            if out is None:
                break
            shapes += out
        else:
            if any(not p.seat and p.layer in (0, top) for p in shapes):
                continue
            by_layer: dict[int, list[Placed]] = {}
            for p in shapes:
                by_layer.setdefault(p.layer, []).append(p)
            if not all(clear(a, b) for ps in by_layer.values()
                       for a, b in itertools.combinations(ps, 2)):
                continue
            need = {k: riders[n] for n, k in layers.items() if n in riders}
            keep = (True, False) if problem.spec.drop_bearing else (True,)
            for route in (only(layers, h0) if only else routes(points, 2, h0 - 1, need, keep)):
                c = cost(route, layers, riders, detours)
                if best is not None and c >= best[0]:
                    continue
                crank: list[Placed] = []
                for claim in routed:
                    out, _ = made(claim, Layout(layers, top, pitch, {router.group: route}))
                    if out is None:
                        break
                    crank += out
                else:
                    if any(not p.seat and p.layer in (0, top) for p in crank):
                        continue
                    if all(clear(a, b) for a in crank for b in by_layer.get(a.layer, ())) \
                            and buildable(route, layers, problem, ctx, h0):
                        best = (c, dict(layers), route)
    return best
