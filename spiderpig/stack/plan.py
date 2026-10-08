"""The planner's settings (:class:`StackSpec`), its CPU deadline and the plan it returns."""


from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field
from functools import cached_property
from typing import TYPE_CHECKING

from spiderpig.stack.geometry import Layout

if TYPE_CHECKING:
    from spiderpig.stack.geometry import Claim, Placed
    from spiderpig.stack.topology import Topology


# ---------------------------------------------------------------------------
# The problem and the plan
# ---------------------------------------------------------------------------


MAX_SECONDS = float(os.environ.get("SPIDERPIG_PLAN_SECONDS", "60"))
"""The planner's default CPU-seconds budget (:attr:`StackSpec.max_seconds`, and the
shared deadline of the recommendation checks, :mod:`recommend`). ``math.inf`` leaves the
node budgets as the only bound, which makes how far a search gets the same on every
machine (``tests/test_stage_checks.py`` does that)."""


@dataclass(frozen=True)
class StackSpec:
    """Stack dimensions (mm): ``pitch`` = sheet thickness; ``margin`` = clearance.

    Search effort, in nodes: ``quick_nodes`` per stack size while looking for
    the first plan, ``max_nodes`` per size left open while none is found and
    for a cheaper route, ``max_nodes // 2`` per thinner size the proof pass
    tries to rule out, ``max_total_nodes`` in all; and ``max_seconds`` of
    wall clock for the whole search, the safety net (a node's cost grows
    with the stack size). A plan found is always valid; only the proof that
    it is the thinnest depends on them: when they run out, a plan found is
    returned unproven (``StackPlan.optimal`` false, ``proof`` naming the
    sizes left open) and none found raises :class:`PlanError` with the tally.
    ``drop_bearing`` lets the crank lose its bottom bearing as the last resort.
    ``workers`` (a prototype): search the stack sizes in that many forked processes
    (:mod:`stack_pool`), with the serial search's answer. ``max_seconds`` is CPU time of
    this process (:class:`Deadline`), so how far a search gets doesn't depend on what else
    the machine is doing (a 60 s wall-clock budget made the same design plan at load 3 and
    fail at load 25, measured); a forked pool worker counts its own CPU from zero, so it
    gets about that much on top. ``prove`` (a prototype, on by
    default): with it off the search stops at the thinnest stack its quick pass finds a
    plan in (the plan it would return before the proof), and says what is left unproven.
    ``quick_first`` (a prototype, off by default): a short search stops at its first plan
    instead of spending the rest of its budget on a cheaper route in a size the next,
    thinner one may well beat (the size kept gets the cheaper route's full search).
    ``symmetry`` (a prototype, off by default): a two-leg module whose leg swap is a
    symmetry of the problem (:mod:`stack_symmetry`) searches one of each mirror pair.
    """

    pitch: float = 3.0
    margin: float = 1.0
    min_top: int = 2
    max_top: int = 60
    quick_nodes: int = 1500
    max_nodes: int = 20000
    max_total_nodes: int = 60000
    # of CPU time (Deadline): a loaded machine doesn't shorten it; read from MAX_SECONDS when
    # a spec is made, so a test can take the clock out and bound a search by its nodes alone
    max_seconds: float = field(default_factory=lambda: MAX_SECONDS)
    drop_bearing: bool = False
    workers: int = 1
    prove: bool = True
    quick_first: bool = False
    symmetry: bool = False
    # z of the plan (:meth:`StackProblem.plan`): the frame plates' thickness (``None``: the
    # pitch), each link's sheet thickness (a link not named: the pitch; a layer is as thick
    # as the thickest plate in it), and the thicknesses a clearance gap may have (the thin
    # sheet a filler plate is cut from, and what an axle's washers stack to), thinnest first
    frame_t: float | None = None
    link_t: tuple[tuple[str, float], ...] = ()
    gaps: tuple[float, ...] = (1.0, 1.5, 2.0, 2.29, 2.54)
    # Where a fastener's head (a clearance shape that may sink, :attr:`Placed.toward`) goes:
    # "sink" claims the layer beside its link, as a full-layer head (no gap ever); "gap"
    # puts it in a thin clearance gap unless it fits that layer; "best" plans them sunk, and
    # in gaps only when that fails; "gap_sink" (a router with its own heads in gaps: the
    # single-plate crank) in gaps, and only when that fails the pivots' heads sunk with the
    # router's (and :data:`GAP_GROUPS`') still in their gaps. :attr:`StackPlan.heads` says
    # which ("sink" or "gap").
    heads: str = "best"


def thread_time() -> float:
    """CPU seconds of the calling thread (:data:`time.CLOCK_THREAD_CPUTIME_ID`; the process's
    where a platform lacks it): what the planner's deadline counts, so neither the machine's
    load nor another thread of the same process (a server baking a glb) shortens a search."""
    try:
        return time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
    except (AttributeError, OSError):
        return time.process_time()


class Deadline:
    """A CPU-time deadline ``seconds`` from its creation (``clock``: :func:`thread_time`, the
    calling thread's CPU; a wall clock would make a plan's reach depend on the machine's
    load), shared by nested planner runs (a design's plan, the leg hint's, the checks of a
    recommendation): each takes what is left of it. ``math.inf``: none."""

    def __init__(self, seconds: float = math.inf, clock=thread_time):
        self.seconds = seconds
        self.clock = clock
        self.start = clock()
        self.at = self.start + seconds

    @property
    def remaining(self) -> float:
        return max(self.at - self.clock(), 0.0)

    @property
    def expired(self) -> bool:
        return self.clock() >= self.at

    @property
    def elapsed(self) -> float:
        return self.clock() - self.start


@dataclass
class StackPlan:
    """A solved stack: every link's layer, the stack size and every claimed shape.

    ``optimal``: no thinner stack exists and no cheaper route for the routed
    group in this one (``proof`` says how that was established, or why not).
    """

    spec: StackSpec
    layers: dict[str, int]
    top: int
    topo: Topology
    claims: tuple[Claim, ...]
    placed: tuple[Placed, ...] = ()
    choices: dict[str, object] = field(default_factory=dict)
    optimal: bool = False
    proof: str = ""
    cost: int = 0
    gaps: dict[int, float] = field(default_factory=dict)    # layer -> clearance gap above it
    thick: dict[int, float] = field(default_factory=dict)   # layer -> thickness (not pitch)
    sunk: frozenset = frozenset()       # the heads sunk into a layer (``sunk_key``)
    heads: str = "gap"                  # how the plan placed heads (StackSpec.heads)

    @cached_property
    def layout(self) -> Layout:
        return Layout(dict(self.layers), self.top, self.spec.pitch, dict(self.choices),
                      dict(self.gaps), dict(self.thick), final=True)

    def z(self, layer: int) -> tuple[float, float]:
        return self.layout.z(layer)

    def gap_z(self, layer: int) -> tuple[float, float]:
        return self.layout.gap_z(layer)

    def slot_z(self, p: Placed) -> tuple[float, float]:
        return self.layout.slot_z(p)

    def t(self, layer: int) -> float:
        return self.layout.t(layer)

    @property
    def height(self) -> float:
        """Total thickness of the stack, both frame plates and every gap included (mm)."""
        return self.layout.height()

    def shapes(self, group: str | None = None, layer: int | None = None,
               gaps: bool = True) -> list[Placed]:
        """Placed shapes, of ``group`` and in ``layer`` (the gap above it too, unless
        ``gaps`` is false)."""
        return [p for p in self.placed
                if (group is None or p.group == group) and (layer is None or p.layer == layer)
                and (gaps or not p.gap)]

    def describe(self) -> str:
        lo = min((p.layer for p in self.placed), default=0)
        hi = max((p.layer for p in self.placed), default=self.top)
        rows = []
        for k in range(max(hi, self.top), min(lo, 0) - 1, -1):
            if self.gaps.get(k):
                held = sorted({p.label or p.group for p in self.placed
                               if p.gap and p.layer == k and p.height > 0})
                rows.append(f"  gap      z {self.gap_z(k)[0]:6.1f}  {self.gaps[k]:g} mm "
                            f"clearance: {', '.join(held)}")
            names = sorted(n for n, s in self.layers.items() if s == k)
            groups = sorted({p.label or p.group for p in self.placed
                             if p.layer == k and not p.gap and not p.seat
                             and p.group not in self.layers})
            if k == self.top:
                label = "inner frame plate"
            elif k == 0:
                label = "outer frame plate"
            else:
                label = ", ".join(names + groups) or "·"
            if k < 0 or k > self.top:
                label = "(outside) " + (", ".join(groups) or "·")
            t = self.t(k)
            rows.append(f"  layer {k:2d}  z {self.z(k)[0]:6.1f}  "
                        + (f"{t:g} mm  " if abs(t - self.spec.pitch) > 1e-9 else "") + label)
        return "\n".join(rows)
