"""Layer planning: which laser-cut link sits in which Z slot, and the hardware between.

The Klann mechanism is planar; Z only matters for fabrication. This module
decides it explicitly. A stack is a sequence of equal-height *slots*
(``pitch`` = sheet thickness), with the frame plate in the top slot and the
servo on top of that. Every leg link (b1..b4) occupies one slot. The rest is
derived from the link slots:

* **Pins** (C, D, E) carry a head flange one slot below their lowest link, a
  cap one slot above their highest link, and pass bare through any slots in
  between.
* **Frame pivots** (A, B) carry a head flange below their lowest link, then
  rise through every slot above it to the plate.
* **The crank is a built-up crankshaft.** Every b1 passes within 0.05 mm of
  the crank axis O at some point in the cycle, and because the crank turns a
  full circle relative to b1, *no* crank-attached point other than b1's own
  crankpin M can pass through b1's slot. So the crank reaches each b1 slot
  only along that b1's crankpin, with a **web** (arms from O out to the
  crankpins) in each neighbouring slot, and runs along O as a **journal**
  through the other slots up to the servo hub.

A plan is valid when, in every slot, every pair of objects clears by
``margin`` over the **whole crank cycle**. :meth:`StackProblem.solve`
searches link slots (lowest stack first) against clearance tables computed
once from vectorized samples of the mechanism.
"""

from __future__ import annotations

import itertools
import re
from collections.abc import Iterable
from dataclasses import dataclass, field
from functools import cached_property
from typing import Literal

import numpy as np

AxisKind = Literal["pin", "frame"]


@dataclass(frozen=True)
class Envelope:
    """Room a pivot's hardware needs around its axis (radii in mm, heights in slots).

    ``head_*``: directly below the lowest plate on the axis. ``tail_*``:
    directly above the highest (unused for frame pivots, which end above the
    frame plate). ``gap_radius``: slots the axle crosses that hold other
    parts. A *flush* envelope needs nothing outside its plates.
    """

    head_radius: float = 4.0
    head_slots: int = 1
    tail_radius: float = 4.0
    tail_slots: int = 1
    gap_radius: float = 2.0

    @property
    def flush(self) -> bool:
        return self.head_slots == 0 and self.tail_slots == 0


@dataclass(frozen=True)
class StackSpec:
    """Physical dimensions (mm) the plan must respect.

    Joinery options supply the three envelopes; the servo supplies the hub
    (its horn and screw heads, in ``hub_slots`` slots right under the frame
    plate) and how many crank plates must sit right under the hub
    (``adapter_slots``: the horn adapter plus the plate its screw heads sink
    into).
    """

    pitch: float = 3.0          # slot height = sheet thickness
    link_radius: float = 6.0    # half-width of a laser-cut link / crank web arm
    margin: float = 1.0         # clearance between objects sharing a slot
    pin: Envelope = Envelope()                                  # link-to-link pivots
    frame: Envelope = Envelope(tail_radius=0.0, tail_slots=0)   # link-to-frame pivots
    crankpin: Envelope = Envelope(tail_radius=0.0, tail_slots=0)  # b1-to-crank pivots
    journal_radius: float = 4.0  # crank plates on the axis O
    hub_radius: float = 4.0      # servo horn + screw heads under the frame plate
    hub_slots: int = 0
    adapter_radius: float = 4.0  # crank plates directly under the hub
    adapter_slots: int = 0

    def envelope(self, kind: str) -> Envelope:
        return {"pin": self.pin, "frame": self.frame, "crankpin": self.crankpin}[kind]

    @property
    def top_rider_gap(self) -> int:
        """Slots between the highest b1 and the frame plate (hub + adapter + web)."""
        return self.hub_slots + max(self.adapter_slots, 1) + 1


@dataclass(frozen=True)
class Link:
    """A laser-cut link: the union of pills (radius ``link_radius``) over ``segments``.

    Each segment is a pair of ``(T, 2)`` XY samples over the crank cycle.
    """

    name: str
    segments: tuple[tuple[np.ndarray, np.ndarray], ...]


@dataclass(frozen=True)
class Axis:
    """Coincident joints between links (or a link and the frame): one physical pin."""

    name: str
    kind: AxisKind
    xy: np.ndarray                          # (T, 2)
    members: tuple[str, ...]                # links riding on this axis
    joints: tuple[tuple[str, str], ...] = ()  # every (body, joint) on the axis


@dataclass(frozen=True)
class Crank:
    """The crankshaft: axis ``center`` (O) and the crankpins its webs reach."""

    center: np.ndarray                       # (T, 2)
    pins: dict[str, np.ndarray]              # crankpin name -> (T, 2)
    riders: dict[str, str]                   # b1 link name -> crankpin name
    joints: dict[str, tuple[tuple[str, str], ...]] = field(default_factory=dict)
    center_joints: tuple[tuple[str, str], ...] = ()


# ---------------------------------------------------------------------------
# Batched planar distances (exact, vectorized over the crank samples)
# ---------------------------------------------------------------------------


def _point_seg(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ab = b - a
    denom = np.maximum((ab * ab).sum(-1), 1e-18)
    u = np.clip(((p - a) * ab).sum(-1) / denom, 0.0, 1.0)
    return np.linalg.norm(p - (a + ab * u[..., None]), axis=-1)


def _cross(o: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return (a[..., 0] - o[..., 0]) * (b[..., 1] - o[..., 1]) - (
        a[..., 1] - o[..., 1]
    ) * (b[..., 0] - o[..., 0])


def seg_seg(p1, q1, p2, q2) -> np.ndarray:
    """Distance between two segments per sample (0 where they cross)."""
    d = np.minimum.reduce([
        _point_seg(p1, p2, q2), _point_seg(q1, p2, q2),
        _point_seg(p2, p1, q1), _point_seg(q2, p1, q1),
    ])
    d1, d2 = _cross(p2, q2, p1), _cross(p2, q2, q1)
    d3, d4 = _cross(p1, q1, p2), _cross(p1, q1, q2)
    crossing = (d1 * d2 < 0) & (d3 * d4 < 0)
    return np.where(crossing, 0.0, d)


def _link_link(a: Link, b: Link) -> float:
    return min(float(seg_seg(p, q, r, s).min()) for (p, q) in a.segments for (r, s) in b.segments)


def _point_link(xy: np.ndarray, b: Link) -> float:
    return min(float(_point_seg(xy, p, q).min()) for (p, q) in b.segments)


def _point_point(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b, axis=-1).min())


# ---------------------------------------------------------------------------
# The plan
# ---------------------------------------------------------------------------


@dataclass
class StackPlan:
    """Slot assignment for every link, plus the frame plate and crank layout."""

    spec: StackSpec
    slots: dict[str, int]           # link name -> slot
    plate: int                      # frame plate slot (top of the stack)
    axes: tuple[Axis, ...]
    crank: Crank | None
    problem: StackProblem | None = None

    def z(self, slot: int) -> tuple[float, float]:
        return slot * self.spec.pitch, (slot + 1) * self.spec.pitch

    def span(self, axis: Axis) -> tuple[int, int]:
        ss = [self.slots[m] for m in axis.members]
        return min(ss), max(ss)

    @property
    def height(self) -> float:
        return (self.plate + 1) * self.spec.pitch

    # -- crank layout ---------------------------------------------------------

    @cached_property
    def rider_slots(self) -> dict[int, set[str]]:
        """slot -> crankpins whose riders (b1 links) sit in it."""
        out: dict[int, set[str]] = {}
        if self.crank is None:
            return out
        for link, pin in self.crank.riders.items():
            out.setdefault(self.slots[link], set()).add(pin)
        return out

    @cached_property
    def webs(self) -> dict[int, tuple[str, ...]]:
        """slot -> crankpins its web reaches. A web sits beside every rider slot.

        Below the lowest b1 a flush crankpin gets a cheek web; a headed one
        ends in its head instead (see :attr:`pin_flanges`).
        """
        out: dict[int, set[str]] = {}
        rs = self.rider_slots
        lowest = min(rs, default=self.plate)
        flush = self.spec.crankpin.flush
        for s, pins in rs.items():
            for k in (s - 1, s + 1):
                if k in rs or (k < lowest and not flush):
                    continue
                out.setdefault(k, set()).update(pins)
        return {k: tuple(sorted(v)) for k, v in sorted(out.items())}

    @cached_property
    def pin_flanges(self) -> dict[str, tuple[int, ...]]:
        """Headed crankpin -> the slots its head fills (below the lowest b1)."""
        out: dict[str, tuple[int, ...]] = {}
        env = self.spec.crankpin
        if self.crank is None or env.flush:
            return out
        rs = self.rider_slots
        lowest = min(rs, default=0)
        for pin in self.crank.pins:
            ss = [s for s, ps in rs.items() if pin in ps]
            if ss and min(ss) == lowest:
                out[pin] = tuple(lowest - k for k in range(1, env.head_slots + 1))
        return out

    @cached_property
    def hub_slots(self) -> tuple[int, ...]:
        """Slots right under the frame plate that hold the servo horn (not crank plates)."""
        return tuple(range(self.plate - self.spec.hub_slots, self.plate))

    @cached_property
    def adapter_slots(self) -> tuple[int, ...]:
        """Crank plates right under the hub: the horn adapter and its screw-head plate."""
        top = self.plate - self.spec.hub_slots
        return tuple(range(top - self.spec.adapter_slots, top))

    @cached_property
    def journal_slots(self) -> tuple[int, ...]:
        """Slots with a crank plate on the axis O (webs included), below the hub."""
        rs = self.rider_slots
        if not rs:
            return ()
        bottom = min(rs) - 1 if self.spec.crankpin.flush else min(rs) + 1
        top = self.plate - self.spec.hub_slots
        return tuple(k for k in range(bottom, top) if k not in rs)

    def crank_radius(self, slot: int) -> float:
        """Radius the crank (or servo hub) occupies around O in ``slot``."""
        if slot in self.hub_slots:
            return self.spec.hub_radius
        if slot in self.adapter_slots:
            return self.spec.adapter_radius
        return self.spec.journal_radius

    @cached_property
    def occupancy(self) -> dict[int, dict[int, float]]:
        """Axis index -> {slot: disc radius} for the flanges and bare shafts it needs."""
        return {
            i: self.problem.occupancy(i, self.slots, self.plate)
            for i in range(len(self.axes))
        }

    def disc_fits(self, i: int, slot: int, radius: float,
                  extra: dict[int, dict[int, float]] | None = None) -> bool:
        """Would a disc of ``radius`` on axis ``i`` in ``slot`` clear everything there?

        Used to add optional sleeves and frame posts. ``extra`` holds discs
        already added the same way (axis -> {slot: radius}).
        """
        pr, sp = self.problem, self.spec
        m = sp.margin
        ax = self.axes[i]
        for n, s in self.slots.items():
            if s == slot and n not in ax.members and (
                pr.jl[(i, n)] < sp.link_radius + radius + m
            ):
                return False
        for source in (self.occupancy, extra or {}):
            for j, occ in source.items():
                r2 = occ.get(slot)
                if j != i and r2 is not None and pr.jj[frozenset((i, j))] < radius + r2 + m:
                    return False
        if slot in self.webs and any(
            pr.aj[(p, i)] < sp.link_radius + radius + m for p in self.webs[slot]
        ):
            return False
        on_axis = slot in self.journal_slots or slot in self.hub_slots
        return not (on_axis and pr.oj[i] < self.crank_radius(slot) + radius + m)

    def describe(self) -> str:
        rows = []
        for k in range(self.plate, -1, -1):
            names = sorted(n for n, s in self.slots.items() if s == k)
            if k == self.plate:
                label = "frame plate"
            else:
                parts = names[:]
                if k in self.hub_slots:
                    parts.append("servo horn")
                if k in self.webs:
                    parts.append(f"web({'+'.join(self.webs[k])})")
                elif k in self.adapter_slots:
                    parts.append("horn adapter" if k == max(self.adapter_slots) else "crank plate")
                elif k in self.journal_slots:
                    parts.append("journal")
                label = ", ".join(parts) or "·"
            rows.append(f"  slot {k:2d}  z {k * self.spec.pitch:5.1f}  {label}")
        return "\n".join(rows)


class StackProblem:
    """Clearance tables for one assembly, and a search over link slots."""

    def __init__(
        self,
        links: Iterable[Link],
        axes: Iterable[Axis] = (),
        crank: Crank | None = None,
        spec: StackSpec | None = None,
    ):
        self.spec = spec or StackSpec()
        self.links = {lk.name: lk for lk in links}
        self.axes = tuple(axes)
        self.crank = crank
        self.rider = dict(crank.riders) if crank else {}

    # -- clearance tables (min distance over the whole cycle) ---------------

    @cached_property
    def ll(self) -> dict[frozenset, float]:
        return {
            frozenset((a, b)): _link_link(self.links[a], self.links[b])
            for a, b in itertools.combinations(self.links, 2)
        }

    @cached_property
    def jl(self) -> dict[tuple[int, str], float]:
        return {
            (i, n): _point_link(ax.xy, lk)
            for i, ax in enumerate(self.axes)
            for n, lk in self.links.items()
            if n not in ax.members
        }

    @cached_property
    def jj(self) -> dict[frozenset, float]:
        return {
            frozenset((i, k)): _point_point(self.axes[i].xy, self.axes[k].xy)
            for i, k in itertools.combinations(range(len(self.axes)), 2)
        }

    @cached_property
    def arms(self) -> dict[str, Link]:
        if self.crank is None:
            return {}
        o = self.crank.center
        return {p: Link(f"arm:{p}", ((o, xy),)) for p, xy in self.crank.pins.items()}

    @cached_property
    def al(self) -> dict[tuple[str, str], float]:
        """Crank arm (O -> crankpin) vs non-b1 link."""
        return {
            (p, n): _link_link(arm, lk)
            for p, arm in self.arms.items()
            for n, lk in self.links.items()
            if n not in self.rider
        }

    @cached_property
    def aj(self) -> dict[tuple[str, int], float]:
        """Crank arm vs axis point."""
        return {
            (p, i): _point_link(ax.xy, arm)
            for p, arm in self.arms.items()
            for i, ax in enumerate(self.axes)
        }

    @cached_property
    def ol(self) -> dict[str, float]:
        """Crank axis O vs link."""
        if self.crank is None:
            return {}
        return {n: _point_link(self.crank.center, lk) for n, lk in self.links.items()}

    @cached_property
    def oj(self) -> dict[int, float]:
        if self.crank is None:
            return {}
        return {i: _point_point(self.crank.center, ax.xy) for i, ax in enumerate(self.axes)}

    def _clear_ll(self, a: str, b: str) -> bool:
        return self.ll[frozenset((a, b))] >= 2 * self.spec.link_radius + self.spec.margin

    # -- what each axis occupies ------------------------------------------------

    def occupancy(self, i: int, slots: dict[str, int], plate: int) -> dict[int, float] | None:
        """slot -> disc radius this axis puts there (``None`` if members unassigned)."""
        ax, sp = self.axes[i], self.spec
        if any(m not in slots for m in ax.members):
            return None
        ms = {slots[m] for m in ax.members}
        lo, hi = min(ms), max(ms)
        env = sp.envelope(ax.kind)
        occ: dict[int, float] = {lo - k: env.head_radius for k in range(1, env.head_slots + 1)}
        top = hi if ax.kind == "pin" else plate
        occ.update({k: env.gap_radius for k in range(lo + 1, top) if k not in ms})
        if ax.kind == "pin":
            occ.update({hi + k: env.tail_radius for k in range(1, env.tail_slots + 1)})
        return occ

    # -- search ---------------------------------------------------------------

    def solve(self, max_plate: int = 30) -> StackPlan:
        member_of: dict[str, list[int]] = {n: [] for n in self.links}
        for i, ax in enumerate(self.axes):
            for m in ax.members:
                member_of[m].append(i)
        order = self._order()
        for plate in range(3, max_plate + 1):
            found = self._search(order, member_of, plate)
            if found is not None:
                return StackPlan(self.spec, found, plate, self.axes, self.crank, self)
        raise ValueError(f"no feasible stack up to {max_plate} slots")

    def _order(self) -> list[str]:
        """b1s first (they constrain the crank), then most-conflicted links."""
        deg = {n: 0 for n in self.links}
        for pair, d in self.ll.items():
            if d < 2 * self.spec.link_radius + self.spec.margin:
                for n in pair:
                    deg[n] += 1
        for ax in self.axes:
            for m in ax.members:
                deg[m] += 1
        return sorted(self.links, key=lambda n: (n not in self.rider, -deg[n], n))

    def _search(self, order, member_of, plate) -> dict[str, int] | None:
        sp = self.spec
        r_link, m = sp.link_radius, sp.margin
        slots: dict[str, int] = {}
        occs: dict[int, dict[int, float]] = {}
        riders_at: dict[int, list[str]] = {}   # slot -> b1 links in it

        def arm_ok_vs_links(pin: str, s: int) -> bool:
            return all(
                self.al[(pin, n)] >= 2 * r_link + m
                for n, sn in slots.items()
                if sn == s and n not in self.rider
            )

        def arm_ok_vs_discs(pin: str, s: int) -> bool:
            for i, occ in occs.items():
                r = occ.get(s)
                if r is not None and self.aj[(pin, i)] < r_link + r + m:
                    return False
            return True

        def place(n: str, s: int) -> list[int] | None:
            if any(sn == s and not self._clear_ll(n, o) for o, sn in slots.items()):
                return None
            for i, occ in occs.items():
                r = occ.get(s)
                if r is not None and self.jl[(i, n)] < r_link + r + m:
                    return None
            pin = self.rider.get(n)
            if pin is not None:
                lowest_ok = 1 if sp.crankpin.flush else sp.crankpin.head_slots
                if not lowest_ok <= s <= plate - sp.top_rider_gap:
                    return None
                for k in (s - 1, s + 1):
                    near = riders_at.get(k, [])
                    if any(self.rider[b] != pin for b in near):
                        return None
                    if not near and not (arm_ok_vs_links(pin, k) and arm_ok_vs_discs(pin, k)):
                        return None
            else:
                for k in (s - 1, s + 1):
                    for b in riders_at.get(k, []):
                        if self.al[(self.rider[b], n)] < 2 * r_link + m:
                            return None
            slots[n] = s
            if pin is not None:
                riders_at.setdefault(s, []).append(n)
            added: list[int] = []
            for i in member_of[n]:
                occ = self.occupancy(i, slots, plate)
                if occ is None:
                    continue
                if not self._axis_ok(i, occ, slots, occs, riders_at, plate):
                    unplace(n, added)
                    return None
                occs[i] = occ
                added.append(i)
            return added

        def unplace(n: str, added: list[int]) -> None:
            for i in added:
                del occs[i]
            if n in self.rider:
                riders_at[slots[n]].remove(n)
            del slots[n]

        def rec(k: int) -> bool:
            if k == len(order):
                return self._journal_ok(slots, occs, riders_at, plate)
            n = order[k]
            for s in range(plate - 1):
                added = place(n, s)
                if added is None:
                    continue
                if rec(k + 1):
                    return True
                unplace(n, added)
            return False

        return dict(slots) if rec(0) else None

    def _axis_ok(self, i, occ, slots, occs, riders_at, plate) -> bool:
        sp = self.spec
        r_link, m = sp.link_radius, sp.margin
        if occ and (min(occ) < 0 or max(occ) >= plate):
            return False
        for n, s in slots.items():
            r = occ.get(s)
            if r is not None and self.jl[(i, n)] < r_link + r + m:
                return False
        for k, other in occs.items():
            d = self.jj[frozenset((i, k))]
            for s, r in occ.items():
                r2 = other.get(s)
                if r2 is not None and d < r + r2 + m:
                    return False
        for s, r in occ.items():  # webs beside b1 slots
            if riders_at.get(s):
                continue
            for k in (s - 1, s + 1):
                for b in riders_at.get(k, []):
                    if self.aj[(self.rider[b], i)] < r_link + r + m:
                        return False
        return True

    def _journal_ok(self, slots, occs, riders_at, plate) -> bool:
        """Checks that need the whole assignment: the crank on O, the servo hub,
        and a headed crankpin's head under the lowest b1."""
        if self.crank is None:
            return True
        plan = StackPlan(self.spec, dict(slots), plate, self.axes, self.crank, self)
        sp, m = self.spec, self.spec.margin
        for k in (*plan.journal_slots, *plan.hub_slots):
            r = plan.crank_radius(k)
            for n, s in slots.items():
                if s == k and self.ol[n] < sp.link_radius + r + m:
                    return False
            for i, occ in occs.items():
                r2 = occ.get(k)
                if r2 is not None and self.oj[i] < r + r2 + m:
                    return False
        head = sp.crankpin.head_radius
        for p, ks in plan.pin_flanges.items():
            for k in ks:
                for n, s in slots.items():
                    if s == k and self.al[(p, n)] < sp.link_radius + head + m:
                        return False
                for i, occ in occs.items():
                    r2 = occ.get(k)
                    if r2 is not None and self.aj[(p, i)] < head + r2 + m:
                        return False
        return True


def group_axes(
    joint_xy: dict[tuple[str, str], np.ndarray],
    connections: Iterable[tuple[tuple[str, str], tuple[str, str]]],
    tol: float = 1e-6,
) -> list[list[tuple[str, str]]]:
    """Group ``(body, joint)`` nodes into physical axes.

    Nodes joined by a connection are one axis; so are nodes whose XY coincide
    over every sample (e.g. the A pivots of two same-chirality decks).
    """
    parent = {n: n for n in joint_xy}

    def find(n):
        while parent[n] != n:
            parent[n] = parent[parent[n]]
            n = parent[n]
        return n

    for a, b in connections:
        parent[find(a)] = find(b)
    nodes = list(joint_xy)
    for a, b in itertools.combinations(nodes, 2):
        if find(a) != find(b) and np.abs(joint_xy[a] - joint_xy[b]).max() < tol:
            parent[find(a)] = find(b)
    groups: dict[tuple[str, str], list[tuple[str, str]]] = {}
    for n in nodes:
        groups.setdefault(find(n), []).append(n)
    return list(groups.values())


# ---------------------------------------------------------------------------
# From a kinematic template to a stack problem
# ---------------------------------------------------------------------------

LINK_CLASSES = frozenset({"b1", "b2", "b3", "b4"})


def body_class(name: str) -> str:
    """``"b1_leg3"`` -> ``"b1"``; ``"conn_upper"`` stays ``"conn_upper"``."""
    return re.sub(r"_leg\d+$", "", name)


def _is_crank(name: str) -> bool:
    return body_class(name).startswith("conn")


def _axis_name(nodes: list[tuple[str, str]]) -> str:
    """Name an axis after its first link joint: ``("b1_leg0", "C")`` -> ``"C_leg0"``."""
    for body, joint in sorted(nodes):
        if body_class(body) in LINK_CLASSES:
            m = re.search(r"(_leg\d+)$", body)
            return f"{joint}{m.group(1) if m else ''}"
    return "_".join(j for _, j in sorted(nodes))


def problem_from_template(
    tmpl, spec: StackSpec | None = None, samples: int = 720,
) -> StackProblem:
    """Classify a template's bodies and joints into links, pin axes and the crank.

    Links are the b1..b4 bodies. Coincident joints form axes: the one on the
    crank centre (frame + crank + coupler) is O; axes joining a crank to b1s
    are crankpins; axes touching the frame are fixed pivots; the rest are pins.
    """
    ts = np.linspace(0.0, 2.0 * np.pi, samples, endpoint=False)
    sampled = tmpl.sample(ts)
    xy = {
        (b.name, j.name): sampled.joint_world[b.name][j.name][:, :2]
        for b in tmpl.bodies for j in b.joints
    }
    links = [
        Link(b.name, tuple((xy[(b.name, p)], xy[(b.name, q)]) for p, q in b.outline))
        for b in tmpl.bodies if body_class(b.name) in LINK_CLASSES
    ]
    edges = [((pn, pj), (cn, cj)) for (_, pn, pj), (_, cn, cj) in tmpl.connections]
    axes: list[Axis] = []
    center = None
    center_joints: tuple[tuple[str, str], ...] = ()
    pins: dict[str, np.ndarray] = {}
    riders: dict[str, str] = {}
    pin_joints: dict[str, tuple[tuple[str, str], ...]] = {}
    for nodes in group_axes(xy, edges):
        bodies = {b for b, _ in nodes}
        members = tuple(sorted(b for b in bodies if body_class(b) in LINK_CLASSES))
        point = xy[nodes[0]]
        cranked = any(_is_crank(b) for b in bodies)
        framed = any(body_class(b) == "torso" for b in bodies)
        if cranked and not members:
            center = point
            center_joints = tuple(sorted(nodes))
        elif cranked:
            name = _axis_name(nodes)
            pins[name] = point
            pin_joints[name] = tuple(sorted(nodes))
            riders.update({m: name for m in members})
        elif framed and members:
            axes.append(Axis(_axis_name(nodes), "frame", point, members, tuple(sorted(nodes))))
        elif len(members) >= 2:
            axes.append(Axis(_axis_name(nodes), "pin", point, members, tuple(sorted(nodes))))
    crank = None
    if center is not None:
        crank = Crank(center=center, pins=pins, riders=riders, joints=pin_joints,
                      center_joints=center_joints)
    return StackProblem(links, axes, crank, spec)


# ---------------------------------------------------------------------------
# Independent verification
# ---------------------------------------------------------------------------


def verify_plan(plan: StackPlan, tmpl, samples: int = 1440, tol: float = 0.05) -> list[str]:
    """Re-check a plan from scratch against a fresh (denser) sampling of ``tmpl``.

    Unlike the solver's incremental tables, this enumerates every physical
    object slot by slot (links, pin flanges and shafts, frame pins, crank
    webs, journal, crankpin flanges) and tests every pair sharing a slot.
    Pairs within one rigid group (the crank; one pin with itself) are
    skipped. Returns human-readable violations (empty when valid).
    """
    fresh = problem_from_template(tmpl, plan.spec, samples)
    sp = plan.spec
    links, crank = fresh.links, fresh.crank
    # object = (label, group, slot, kind, geometry, radius)
    objs: list[tuple[str, str, int, str, object, float]] = []
    for name, slot in plan.slots.items():
        objs.append((name, name, slot, "segs", links[name].segments, sp.link_radius))
    by_name = {ax.name: ax for ax in fresh.axes}
    for ax in plan.axes:
        xy = by_name[ax.name].xy
        env = sp.envelope(ax.kind)
        ms = {plan.slots[m] for m in ax.members}
        lo, hi = min(ms), max(ms)
        for k in range(1, env.head_slots + 1):
            objs.append((f"head:{ax.name}@{lo - k}", ax.name, lo - k, "pt", xy, env.head_radius))
        top = plan.plate if ax.kind == "frame" else hi
        for k in range(lo + 1, top):
            if k not in ms:
                objs.append((f"shaft:{ax.name}@{k}", ax.name, k, "pt", xy, env.gap_radius))
        if ax.kind == "pin":
            for k in range(1, env.tail_slots + 1):
                objs.append((f"tail:{ax.name}@{hi + k}", ax.name, hi + k, "pt", xy,
                             env.tail_radius))
    if crank is not None:
        o = crank.center
        for k in (*plan.journal_slots, *plan.hub_slots):
            objs.append((f"crank@{k}", "crank", k, "pt", o, plan.crank_radius(k)))
        for k, pins in plan.webs.items():
            for p in pins:
                objs.append((f"web@{k}:{p}", "crank", k, "segs", ((o, crank.pins[p]),),
                             sp.link_radius))
        for p, ks in plan.pin_flanges.items():
            for k in ks:
                objs.append((f"head:{p}@{k}", "crank", k, "pt", crank.pins[p],
                             sp.crankpin.head_radius))

    def dist(a, b) -> float:
        (_, _, _, ka, ga, _), (_, _, _, kb, gb, _) = a, b
        if ka == "pt" and kb == "pt":
            return _point_point(ga, gb)
        if ka == "pt":
            return _point_link(ga, Link("", gb))
        if kb == "pt":
            return _point_link(gb, Link("", ga))
        return _link_link(Link("", ga), Link("", gb))

    bad = []
    for a, b in itertools.combinations(objs, 2):
        if a[2] != b[2] or a[1] == b[1]:
            continue
        need = a[5] + b[5] + sp.margin - tol
        d = dist(a, b)
        if d < need:
            bad.append(f"slot {a[2]}: {a[0]} x {b[0]} clear {d - a[5] - b[5]:.2f} mm "
                       f"(need {sp.margin:.2f})")
    if min(o[2] for o in objs) < 0:
        bad.append("an object sits below slot 0")
    if max(o[2] for o in objs) >= plan.plate:
        bad.append("an object reaches the frame plate slot")
    return bad
