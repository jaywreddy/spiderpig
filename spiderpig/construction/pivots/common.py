"""Pieces the metal-shaft pivots share: the claimed column, printed sleeves, small hardware
solids.

**The sleeve rule** (2026-10-14, :func:`sleeve_runs`): what an axle prints between the parts
it retains (its links, the frame plates it is anchored in, its heads and clips) is one
sleeve per contiguous run of rings (a spacer or shoulder layer) and gap rings (the washer
through a clearance gap), stepped where their radii differ, and split only where printing
needs it: a sleeve prints without support when its radius never grows away from the bed, so
each piece is cut down to the running least from its bed end (whichever end keeps the
most), and a run splits where that would cut a ring under what it must keep (a shoulder's
overlap of its link's hole). The parts an axle prints are then a property of the layout,
not one per layer and one per gap (:func:`sleeve_count`, the planner's ``tally``).
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

from spiderpig.construction.axle import AxleGroup, full_rings
from spiderpig.construction.base import Build, Realized, hardware
from spiderpig.shapes import disc, ring, union

STEEL = "#4a4a4a"
RING_COLOR = "#a9cbe0"       # the laser-cut spacer rings' colour (rings are printed now)
SLEEVE_COLOR = "#1baf7a"     # printed spacer sleeves (the printed axle's green)
EPS = 1e-9

RETAINED = ("axle", "anchor")            # column roles the ends clamp between
SPACER_ROLES = ("shoulder", "spacer", "neck")



PRINT_MIN = 0.4      # thinnest printed spacer (two 0.2 mm layers); thinner is left as play
PRINT_TOL = 0.1      # a printed spacer's height tolerance (mm), counted as a pin's play

def xy_of(build: Build, group: AxleGroup) -> tuple[float, float]:
    xy = build.xy(group.axis.name)
    return float(xy[0]), float(xy[1])


def host_of(build: Build, group: AxleGroup) -> str:
    """What a pin's loose parts move with: its lowest link (a pillar's: the frame,
    ``topo.frame_bodies[0]``)."""
    return min(group.axis.members, key=lambda m: (build.layers[m], m))


def stem_of(group: AxleGroup) -> str:
    return group.name.replace(":", "_")


@dataclass(frozen=True)
class Column:
    """An axle group's claimed column at the solved plan.

    ``roles[layer] = (role, radius)``: ``axle`` (seated in a link), ``anchor``
    (seated in a frame plate), ``shoulder`` / ``spacer`` / ``neck`` between,
    and the retainers beyond each end (``head``, ``cap``, ``nut``).
    """

    roles: dict[int, tuple[str, float]]
    links: dict[int, tuple[str, ...]]        # layer -> the links turning on the axle there
    # the clearance gaps the column has (the plan's): layer under the gap -> (role, radius,
    # height): its end retainers ("head", "cap", "nut", ... with the height they need) and
    # the washers it carries through a gap ("washer", height 0)
    gaps: dict[int, tuple[str, float, float]] = field(default_factory=dict)

    @classmethod
    def of(cls, build: Build, group: AxleGroup) -> Column:
        roles, gaps = {}, {}
        for s in build.shapes(group.name):
            role = s.label.rsplit(" ", 1)[1]
            if s.gap:
                gaps[s.layer] = (role, s.shape.r, s.height)
            else:
                roles[s.layer] = (role, s.shape.r)
        links: dict[int, tuple[str, ...]] = {}
        for m in group.axis.members:
            k = build.layers[m]
            links[k] = (*links.get(k, ()), m)
        return cls(roles, links, gaps)

    @property
    def lo_gap(self) -> bool:
        """The bottom end's retainer sits in the clearance gap under the stack."""
        return self.gaps.get(self.k0 - 1, ("",))[0] not in ("", "washer")

    @property
    def hi_gap(self) -> bool:
        return self.gaps.get(self.k1, ("",))[0] not in ("", "washer")

    def end_z(self, build: Build, side: str) -> tuple[float, float] | None:
        """Where the ``"lo"`` / ``"hi"`` end's retainer may go: its clearance gap, or the
        layer beyond the stack (``None``: it has none)."""
        if side == "lo":
            if self.lo_gap:
                return build.plan.gap_z(self.k0 - 1)
            return build.z(self.k0 - 1) if self.k0 - 1 in self.roles else None
        if self.hi_gap:
            return build.plan.gap_z(self.k1)
        return build.z(self.k1 + 1) if self.k1 + 1 in self.roles else None

    @property
    def washers(self) -> list[int]:
        """The clearance gaps the column crosses (the layer under each)."""
        return sorted(k for k, (role, _, _) in self.gaps.items() if role == "washer")

    def role(self, k: int) -> str:
        return self.roles[k][0]

    @property
    def k0(self) -> int:
        """The lowest retained layer: the outer anchor, or the lowest link."""
        return min(k for k, (role, _) in self.roles.items() if role in RETAINED)

    @property
    def k1(self) -> int:
        return max(k for k, (role, _) in self.roles.items() if role in RETAINED)

    @property
    def between(self) -> list[int]:
        """The spacer layers between the ends (every non-link layer, for a filled column)."""
        return [k for k in range(self.k0 + 1, self.k1) if self.role(k) in SPACER_ROLES]

    @property
    def anchors(self) -> list[int]:
        return sorted(k for k, (role, _) in self.roles.items() if role == "anchor")


def bored(part, xy, d: float, z0: float, z1: float):
    """``part`` with a through-bore of diameter ``d``."""
    return part - disc(xy, d / 2, z0 - 1.0, z1 + 1.0)


HEAD_CLEARANCE = 0.25    # a retainer in a clearance gap stays this far off the next layer (mm)


def ring_z(build: Build, k: int) -> tuple[float, float]:
    """A spacer ring's z in layer ``k``: the default sheet's thickness on the layer's
    floor (a layer an aluminium plate thickens is a little taller than a ring), or the
    whole layer (:func:`construction.axle.full_rings`)."""
    z0, z1 = build.z(k)
    return (z0, z1) if full_rings(build.ctx) else (z0, min(z1, z0 + build.ctx.pitch))


@dataclass(frozen=True)
class Slot:
    """One ring of an axle's printed run: a spacer ring in a layer or the ring through a
    clearance gap (``name``: the body it was before the sleeve rule), ``z0``..``z1``, the
    radius its claim gives it (``r``) and the least it may be cut to and still do its job
    (``least``: a shoulder's overlap of its link's hole)."""

    name: str
    z0: float
    z1: float
    r: float
    least: float = 0.0


def _printable(seg: Sequence[Slot]) -> list[float] | None:
    """The radii a run of touching slots prints at as one piece without support (each its
    claim's at most, never growing away from the bed: the running least from the bottom, or
    from the top with the piece printed upside down; the one that keeps more), ``None``
    when either would cut a slot under its ``least``."""
    best: tuple[float, list[float]] | None = None
    for order in (range(len(seg)), range(len(seg) - 1, -1, -1)):
        radii = [0.0] * len(seg)
        least = float("inf")
        for i in order:
            least = min(least, seg[i].r)
            radii[i] = least
        if any(r < s.least - 1e-9 for r, s in zip(radii, seg, strict=True)):
            continue
        kept = sum(r * r * (s.z1 - s.z0) for r, s in zip(radii, seg, strict=True))
        if best is None or kept > best[0] + 1e-12:
            best = (kept, radii)
    return None if best is None else best[1]


def sleeve_runs(slots: Sequence[Slot], eps: float = 1e-6) -> list[list[tuple[Slot, float]]]:
    """The printed sleeves of an axle's slots (any order): slots that touch (one's top the
    next's bottom, within ``eps``) are one run, each run one sleeve (its slots with the
    radius each prints at: :func:`_printable`), split only where one piece can't print
    without cutting a slot under its ``least`` (a slot alone always prints, at its own
    radius). Bottom up."""
    order = sorted(slots, key=lambda s: (s.z0, s.z1))
    runs: list[list[Slot]] = []
    for s in order:
        if runs and abs(runs[-1][-1].z1 - s.z0) <= eps:
            runs[-1].append(s)
        else:
            runs.append([s])
    pieces: list[list[tuple[Slot, float]]] = []
    for run in runs:
        i = 0
        while i < len(run):
            j, radii = i, [run[i].r]
            while j + 1 < len(run):
                got = _printable(run[i:j + 2])
                if got is None:
                    break
                j, radii = j + 1, got
            pieces.append(list(zip(run[i:j + 1], radii, strict=True)))
            i = j + 1
    return pieces


def column_slots(build: Build, group: AxleGroup, col: Column, *,
                 fill: dict[int, float] | None = None, trim: dict[int, float] | None = None,
                 skip: Sequence[int] = ()) -> list[Slot]:
    """The slots of a column (:class:`Slot`): a ring in each spacer layer between its ends
    (:func:`ring_z`, ``fill`` more on top) and one through each clearance gap it crosses
    (``trim[k]``: what something else of the axle's takes of gap ``k``'s top: a standoff's
    end shims; ``skip``: gaps another part of the axle fills, a clevis pin's take-up
    spacer)."""
    from spiderpig.construction.axle import STOP_OVERLAP

    p, d = build.ctx.params, group.dims(build.ctx)
    stop = p.hole(2 * d.axle) / 2 + STOP_OVERLAP
    out = []
    for k in col.between:
        role, r = col.roles[k]
        z0, z1 = ring_z(build, k)
        out.append(Slot(f"ring{k}", z0, z1 + (fill or {}).get(k, 0.0), r,
                        stop if role == "shoulder" else d.neck))
    for k in col.washers:
        if k in skip:
            continue
        z0, z1 = build.plan.gap_z(k)
        z1 -= (trim or {}).get(k, 0.0)
        if z1 - z0 > 1e-6:
            out.append(Slot(f"gap{k}_spacer", z0, z1, col.gaps[k][1], d.neck))
    return out


def build_sleeves(build: Build, group: AxleGroup, slots: Sequence[Slot], out: Realized,
                  bore: float, host: str, stem: str) -> tuple[float, dict[str, str]]:
    """The printed sleeves of ``slots`` (:func:`sleeve_runs`), bored ``bore`` mm, into
    ``out``: each named after its lowest slot (``{stem}_ring{k}``, ``{stem}_gap{k}_spacer``);
    one under :data:`PRINT_MIN` tall is left as play. Returns (the play left, mm; each
    slot's sleeve by the slot's name)."""
    xy = xy_of(build, group)
    play = 0.0
    of: dict[str, str] = {}
    for piece in sleeve_runs(slots):
        z0, z1 = piece[0][0].z0, piece[-1][0].z1
        if z1 - z0 < PRINT_MIN - 1e-6:
            play += z1 - z0
            continue
        name = f"{stem}_{piece[0][0].name}"
        part = union([ring(xy, 2 * r, bore, s.z0, s.z1) for s, r in piece]) \
            if len(piece) > 1 else ring(xy, 2 * piece[0][1], bore, z0, z1)
        out.bodies.append(hardware(name, part, host, fab="printed", color=SLEEVE_COLOR))
        of.update(dict.fromkeys((s.name for s, _ in piece), name))
    return play, of


def sleeve_count(slots: Sequence[Slot]) -> int:
    """The sleeves :func:`build_sleeves` prints of ``slots`` (the planner's ``tally``)."""
    return sum(1 for piece in sleeve_runs(slots)
               if piece[-1][0].z1 - piece[0][0].z0 >= PRINT_MIN - 1e-6)


def unit_usd(key: str) -> float:
    """What one of catalog item ``key`` costs (its first offer: the pack's price over the
    pack; 0 unpriced): the planner's ``tally`` of a stock part."""
    from spiderpig.hardware.catalog import get

    offer = get(key).offer
    if offer is None or offer.price_usd is None:
        return 0.0
    return offer.price_usd / max(offer.pack_qty, 1)


def column_air(build: Build, group: AxleGroup, a: int, b: int) -> float:
    """What the plan's z leaves free around an axle's own parts in layers ``a``..``b`` (its
    links at their sheets' thickness, a default-sheet ring elsewhere, in layers an aluminium
    plate made thicker): the axle's stack closes it up (the planner's ``air``,
    :meth:`construction.axle.AxleGroup.claims`)."""
    plan, ctx = build.plan, build.ctx
    full = full_rings(ctx)
    own: dict[int, float] = {}
    for m in group.axis.members:
        k = plan.layers[m]
        own[k] = max(own.get(k, 0.0), ctx.sheet_t("link", m))
    return sum(max(0.0, plan.t(k) - own.get(k, plan.t(k) if full else ctx.pitch))
               for k in range(a, b + 1) if 0 < k < plan.top)
