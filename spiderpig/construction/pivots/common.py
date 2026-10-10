"""Pieces the metal-shaft pivots share: the claimed column, printed rings, small hardware
solids."""

from __future__ import annotations

from dataclasses import dataclass, field

from spiderpig.construction.axle import AxleGroup, full_rings
from spiderpig.construction.base import Build, Realized, hardware
from spiderpig.shapes import disc

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


def ring_runs(build: Build, col, fill: dict[int, float] | None = None
              ) -> list[tuple[list[int], float, float, float]]:
    """The printed rings of a column's spacer layers ``col.between``: ``(layers, radius, z0,
    z1)`` each. One ring per layer; where the rings fill their layers
    (:func:`construction.axle.full_rings`, a cradle's), one sleeve for each run of touching
    layers with the same ring (fewer parts, one step to put on). ``fill``: extra height on
    a layer's ring (:meth:`StandoffAxle.ring_fill`)."""
    fill = fill or {}
    out: list[tuple[list[int], float, float, float]] = []
    merge = full_rings(build.ctx)
    for k in col.between:
        _, r = col.roles[k]
        z0, z1 = ring_z(build, k)
        z1 += fill.get(k, 0.0)
        if (merge and out and out[-1][0][-1] == k - 1 and out[-1][1] == r
                and abs(out[-1][3] - z0) < 1e-6):
            ks, _, a, _ = out[-1]
            out[-1] = ([*ks, k], r, a, z1)
        else:
            out.append(([k], r, z0, z1))
    return out


def gap_washers(build: Build, group: AxleGroup, col: Column, out: Realized, shaft_d: float,
                host: str, stem: str, color: str = "#f2f2f2",
                trim: dict[int, float] | None = None) -> float:
    """What an axle carries through every clearance gap of its column (the plan's): one
    printed ring at the gap's height (unclamped: it only locates the links; under
    :data:`PRINT_MIN` the gap is left as play). ``trim[k]``: the height at the top of gap
    ``k`` something else of the axle's takes (a standoff column's end shims), so its ring
    stops under it. Returns the play left in all (mm)."""
    from spiderpig.materials import washer_od

    xy = xy_of(build, group)
    play = 0.0
    for k in col.washers:
        g = build.plan.gaps.get(k, 0.0) - (trim or {}).get(k, 0.0)
        if g <= 1e-6:
            continue
        if g < PRINT_MIN - 1e-6:
            play += g
            continue
        z0, _ = build.plan.gap_z(k)
        od = washer_od(shaft_d)
        part = bored(disc(xy, od / 2, z0, z0 + g), xy, shaft_d + 0.3, z0, z0 + g)
        out.bodies.append(hardware(f"{stem}_gap{k}_spacer", part, host, fab="printed",
                                   color=SLEEVE_COLOR))
    return play


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
