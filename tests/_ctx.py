"""Synthetic inputs for the seam tests (ROADMAP W4, item 2): a :class:`stack.Topology`, a
:class:`construction.base.Context`, a :class:`stack.Layout` and a few claims, hand-made in a
few lines, so a construction's or the planner's rule is tested on numbers the test states,
without a linkage, a plan search or a fabrication.

```python
from tests import _ctx

topo = _ctx.topology()                    # O, a crankpin M turning about it, a rider b1
ctx = _ctx.context(topo)                  # the default build config (Strider double)
L = _ctx.layout(top=8, gaps={3: 1.5})     # every layer 3 mm, a 1.5 mm gap over layer 3
```

Every helper returns a fresh object: a test may change what it gets.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

import numpy as np

from spiderpig import servos
from spiderpig.config import BuildConfig
from spiderpig.construction.base import Context
from spiderpig.stack import Axis, Claim, Disc, Geometry, Layout, Pill, Placed, Topology

SAMPLES = 24
"""Samples of the crank cycle in a turning geometry."""


def turning(r: float, phase_deg: float = 0.0, centre=(0.0, 0.0), n: int = SAMPLES) -> np.ndarray:
    """A point ``r`` from ``centre`` turning once round it (``(n, 2)``)."""
    t = np.linspace(0.0, 2 * math.pi, n, endpoint=False) + math.radians(phase_deg)
    return np.c_[centre[0] + r * np.cos(t), centre[1] + r * np.sin(t)]


def still(xy, n: int = SAMPLES) -> np.ndarray:
    """A fixed point, sampled ``n`` times."""
    return np.tile(np.asarray(xy, dtype=float), (n, 1))


def topology(points: Mapping[str, object] | None = None,
             links: Mapping[str, Sequence[tuple[str, str]]] | None = None,
             axles: Sequence[tuple[str, str, Sequence[str]]] | None = None,
             name: str = "toy") -> Topology:
    """A topology of ``points`` (name -> ``(x, y)`` or an ``(n, 2)`` array), ``links``
    (name -> outline segments as point pairs) and ``axles`` (``(point, kind, links)``,
    kind ``"center"`` / ``"crankpin"`` / ``"frame"`` / ``"pin"``). The default: O at the
    origin, a crankpin M turning 12 mm about it, a rider b1 from M to a point P beside it,
    and b2 from P to a frame pillar F; axles O (centre), M (crankpin, b1), P (pin, b1 b2) and
    F (frame, b2)."""
    if points is None:
        m = turning(12.0)
        points = {"O": (0.0, 0.0), "M": m, "P": m + np.array([40.0, 0.0]),
                  "F": (60.0, -30.0)}
    if links is None:
        links = {"b1": (("M", "P"),), "b2": (("P", "F"),)}
    if axles is None:
        axles = (("O", "center", ()), ("M", "crankpin", ("b1",)), ("P", "pin", ("b1", "b2")),
                 ("F", "frame", ("b2",)))
    arrays = {k: (np.asarray(v, dtype=float) if np.ndim(v) == 2 else still(v))
              for k, v in points.items()}
    axes = tuple(Axis(p, kind, tuple(ms), tuple((m, p) for m in ms)) for p, kind, ms in axles)
    point_of = {(m, p): p for p, _, ms in axles for m in ms}
    return Topology(name, Geometry(arrays), {k: tuple(v) for k, v in links.items()}, axes,
                    point_of)


def context(topo: Topology | None = None, config: BuildConfig | None = None, *,
            servo: str | None = None, pitch: float | None = None,
            interfaces: Mapping[str, object] | None = None) -> Context:
    """A context on ``topo`` (default: :func:`topology`) for ``config`` (default: the
    project's default design, a side), ``servo`` and ``pitch`` overriding the config's."""
    cfg = config or BuildConfig(robot=False)
    if servo is not None:
        cfg = BuildConfig(**{**_fields(cfg), "servo": servo})
    return Context(topo=topo if topo is not None else topology(), params=cfg.params,
                   pitch=cfg.pitch if pitch is None else pitch,
                   servo=servos.get(cfg.servo), config=cfg,
                   interfaces=dict(interfaces or {}))


def _fields(cfg: BuildConfig) -> dict:
    from dataclasses import fields

    return {f.name: getattr(cfg, f.name) for f in fields(cfg) if f.init}


def layout(layers: Mapping[str, int] | None = None, top: int = 8, pitch: float = 3.0, *,
           gaps: Mapping[int, float] | None = None, thick: Mapping[int, float] | None = None,
           final: bool = True, choices: Mapping[str, object] | None = None) -> Layout:
    """A layout of ``top + 1`` layers ``pitch`` thick (``thick`` per layer otherwise), the
    clearance gaps ``gaps`` (layer -> mm over it); ``final`` (the plan's z) by default."""
    return Layout(dict(layers or {}), top, pitch, dict(choices or {}), dict(gaps or {}),
                  dict(thick or {}), final)


def link_claim(name: str, r: float = 3.0, topo: Topology | None = None) -> Claim:
    """A link's own claim: a pill of radius ``r`` round each of its outline segments, in its
    layer (the shape :mod:`construction.plates` claims for a laser-cut link)."""
    segs = topo.links[name] if topo is not None else None

    def make(L: Layout) -> list[Placed]:
        k = L.layers[name]
        out = segs if segs is not None else ()
        return [Placed(k, Pill(a, b, r), name, name) for a, b in out]

    return Claim(name, frozenset((name,)), make)


def disc_claim(group: str, at: str, r: float, layers: Sequence[int],
               deps: Sequence[str] = ()) -> Claim:
    """A group's disc of radius ``r`` round ``at`` in fixed ``layers`` (a pillar's ring, a
    printed spacer: what does not move with the links' layers)."""
    return Claim(group, frozenset(deps),
                 lambda L: [Placed(k, Disc(at, r), group, group) for k in layers])
