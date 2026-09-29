"""A reference layer planner for small designs: every layering, every crank route.

Independent of :mod:`stack`'s search and :mod:`construction.route`'s router:
it enumerates link layers (skipping only pairs of links whose own shapes
collide, which no route can change), builds every claim for each, and tries
every crank route: sets of runs (a run point and a layer range, webs in the
layers either side, ``1..`` below the hub) that put each rider in a run of
its own crankpin. A layering and route count when no two shapes of different
groups come too close in a layer (the planner's own distances) and nothing but
seats sits in a frame plate's layer. Cost: the planner's objective, in order:
added crank features (run layers no rider of their point is in, detour runs),
the detours' sweep, a dropped bearing.
"""

from __future__ import annotations

import itertools
from collections.abc import Iterator

from construction.crank import CrankRoute, Run
from stack import Layout, Placed, StackProblem, made


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


def solve(problem: StackProblem, top: int) -> tuple[tuple, dict[str, int], CrankRoute] | None:
    """The cheapest layering and route in ``top + 1`` layers, or ``None``."""
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
    apart = {(a, ka, b, kb): all(clear(p, q) for p in solo[(a, ka)] for q in solo[(b, kb)])
             for a, b in itertools.combinations(links, 2)
             for ka in range(1, top) for kb in range(1, top) if ka == kb}
    best = None
    for combo in itertools.product(range(1, top), repeat=len(links)):
        layers = dict(zip(links, combo, strict=True))
        if not all(apart.get((a, layers[a], b, layers[b]), True)
                   for a, b in itertools.combinations(links, 2)):
            continue
        layout = Layout(layers, top, pitch)
        shapes: list[Placed] = []
        for c in fixed:
            out, _ = made(c, layout)
            if out is None:
                break
            shapes += out
        else:
            if any(not p.seat and p.layer in (0, top) for p in shapes):
                continue
            if not all(clear(a, b) for a, b in itertools.combinations(shapes, 2)):
                continue
            need = {k: riders[n] for n, k in layers.items() if n in riders}
            keep = (True, False) if problem.spec.drop_bearing else (True,)
            for route in routes(points, 2, h0 - 1, need, keep):
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
                    if all(clear(a, b) for a in crank for b in shapes):
                        best = (c, dict(layers), route)
    return best


__all__ = ["cost", "routes", "solve"]
