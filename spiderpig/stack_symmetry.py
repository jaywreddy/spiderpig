"""Symmetries of a multi-leg module's layer problem (a prototype behind ``StackSpec.symmetry``).

A module's legs run at phases of one crank. When, for some re-timing of the cycle (a shift
by a fraction of a turn, possibly run backwards: a distance over the whole cycle doesn't
care), every point's path is another point's path, the links, the router's pieces and rules
and the axles' facts map onto each other, and every claim gives the mapped shapes for the
mapped layering, then that relabelling maps every layering to one exactly as good: the
search needs only one of each orbit. :func:`symmetries` finds such maps and checks them
(the claims on sampled layerings, per stack size: they are closures, so that check is a
sample, not a proof); the search then keeps ``layer(x) <= layer(g(x))`` for every map ``g``
and the first link ``x`` it places (a lex-leader: in every orbit the layering with the
lowest ``x`` satisfies them all), which cuts what it explores without losing any plan's cost.
"""

from __future__ import annotations

import random
from dataclasses import replace

import numpy as np

from spiderpig import stack

TOL = 1e-6          # mm: the legs' paths are computed apart, so they agree to ~1e-13
RADIUS = 9          # decimals a shape's radius is compared to


def _point_map(names, arrs, idx, onto) -> dict[str, str] | None:
    """Each point to the one whose path (in ``onto``) is its own path (in ``arrs``: possibly
    mirrored) re-timed by ``idx``; None unless that is one-to-one."""
    out = {}
    for p, a in zip(names, arrs, strict=True):
        moved = a[idx]
        match = [q for q, b in zip(names, onto, strict=True) if np.abs(moved - b).max() < TOL]
        if len(match) != 1:
            return None
        out[p] = match[0]
    return out if len(set(out.values())) == len(names) else None


def symmetries(prob: stack.StackProblem) -> list[tuple[dict[str, str], dict[str, str]]]:
    """Every non-trivial relabelling of the legs that is a symmetry of ``prob`` (geometry,
    links, riders, router, axle facts; the claims are checked per stack size,
    :func:`claims_commute`), as (link map, point map)."""
    legs = sorted(set(prob.leg.values()))
    if len(legs) < 2 or not all("_leg" in n for n in prob.links):
        return []
    geo = prob.topo.geometry
    names = list(geo.points)
    arrs = [np.asarray(geo.points[p]) for p in names]
    T = geo.samples
    t = np.arange(T)
    out = []
    shifts = sorted({(k * T) // d for d in (len(legs), 2) for k in range(d) if (k * T) % d == 0})
    ox = float(np.asarray(geo.points["O"]).reshape(-1, 2)[0, 0])
    mirrored = [a * [-1.0, 1.0] + [2 * ox, 0.0] for a in arrs]     # reflected across x = O.x
    for s, back, mirror in ((s, b, m) for s in shifts for b in (False, True)
                            for m in (False, True)):
        if s == 0 and not back and not mirror:
            continue
        sig = _point_map(names, mirrored if mirror else arrs, (s - t) % T if back else (s + t) % T,
                         arrs)
        if sig is not None:
            g = _links(prob, sig)
            if g is not None and any(g[n] != n for n in g) and _router_ok(prob, sig) \
                    and _facts_ok(prob, g):
                out.append((g, sig))
    return out


def _links(prob, sig) -> dict[str, str] | None:
    outline = {n: frozenset(frozenset(s) for s in segs) for n, segs in prob.topo.links.items()}
    by_outline = {o: n for n, o in outline.items()}
    g = {}
    for n, o in outline.items():
        img = frozenset(frozenset(sig[p] for p in s) for s in o)
        if img not in by_outline:
            return None
        g[n] = by_outline[img]
    if len(set(g.values())) != len(g):
        return None
    riders = prob.topo.riders
    if any((n in riders) != (g[n] in riders) or (n in riders and sig[riders[n]] != riders[g[n]])
           for n in g):
        return None
    return g


def _router_ok(prob, sig) -> bool:
    r = prob.router
    if r is None:
        return True
    if not hasattr(r, "points"):
        return False
    idx = {p: j for j, p in enumerate(r.points)}
    if any(sig.get(p) not in idx for p in r.points):
        return False
    perm = [idx[sig[p]] for p in r.points]
    n = len(perm)
    return (all(r.last[j] == r.last[perm[j]] and r.sweep[j] == r.sweep[perm[j]]
                for j in range(n))
            and all(r.after[i][j] == r.after[perm[i]][perm[j]]
                    for i in range(n) for j in range(n)))


def _facts_ok(prob, g) -> bool:
    facts = {(m, lk, a) for v in prob.spans.values() for (m, lk, a) in v}
    img = {(tuple(sorted(g[x] for x in m)), tuple(sorted(g[x] for x in lk)), a)
           for m, lk, a in facts}
    return img == facts


def _shape(s, sig=None):
    if isinstance(s, stack.Disc):
        return ("d", sig[s.at] if sig else s.at, round(s.r, RADIUS))
    a, b = (sig[s.a], sig[s.b]) if sig else (s.a, s.b)
    return ("p", *sorted((a, b)), round(s.r, RADIUS))


def claims_commute(prob: stack.StackProblem, g: dict[str, str], sig: dict[str, str], top: int,
                   samples: int = 8) -> bool:
    """Every claim has an image claim giving the mapped shapes (same layers, seats and a
    one-to-one map of groups) for the mapped layering, on ``samples`` random layerings of
    its links in a stack of ``top + 1`` layers (seeded: the same answer every run)."""
    rnd = random.Random(top)
    claims = [c for c in prob.claims if c.choice is None]
    by_deps: dict[frozenset, list] = {}
    for c in claims:
        by_deps.setdefault(frozenset(c.deps), []).append(c)
    groups: dict[str, str] = {}
    for c in claims:
        img = frozenset(g[d] for d in c.deps)
        for c2 in by_deps.get(img, ()):
            if (c2.final != c.final or (c.early is None) != (c2.early is None)
                    or frozenset(g[d] for d in c.early_deps) != c2.early_deps):
                continue
            seen: dict[str, str] = {}
            ok = True
            parts = [(c, c2, c.deps)] + ([(replace(c, make=c.early), replace(c2, make=c2.early),
                                           c.early_deps)] if c.early is not None else [])
            for one, two, deps in parts:
                for _ in range(samples if deps else 1):
                    lay = {d: rnd.randrange(1, top) for d in deps}
                    out1, _ = stack.made(one, stack.Layout(lay, top, prob.spec.pitch))
                    out2, _ = stack.made(two, stack.Layout({g[d]: v for d, v in lay.items()},
                                                           top, prob.spec.pitch))
                    if (out1 is None) != (out2 is None):
                        ok = False
                        break
                    if out1 is None:
                        continue
                    a = sorted((p.layer, _shape(p.shape, sig), p.seat) for p in out1)
                    b = sorted((p.layer, _shape(p.shape), p.seat) for p in out2)
                    if a != b:
                        ok = False
                        break
                    for p in out1:
                        key = (p.layer, _shape(p.shape, sig), p.seat)
                        for q in out2:
                            if (q.layer, _shape(q.shape), q.seat) == key:
                                seen.setdefault(p.group, q.group)
                if not ok:
                    break
            if ok:
                for a_, b_ in seen.items():
                    if groups.setdefault(a_, b_) != b_:
                        return False
                break
        else:
            return False
    return len(set(groups.values())) == len(groups)       # one-to-one
