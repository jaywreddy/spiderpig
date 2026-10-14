"""Pieces the metal-shaft pivots share: the claimed column, printed rings, small hardware
solids."""

from __future__ import annotations

import itertools
from dataclasses import dataclass, field
from typing import Any, cast

from spiderpig.construction.axle import AxleGroup
from spiderpig.construction.base import Build, Realized, hardware
from spiderpig.hardware.bom import BomLine
from spiderpig.shapes import disc, ring

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


def link_z(build: Build, m: str) -> tuple[float, float]:
    """Link ``m``'s z: its own sheet's thickness on its layer's floor, as
    :class:`construction.plates.LinkPlates` cuts it (a layer a thicker plate thickens leaves
    air over a thinner link: what sits on or beside the link reads its faces here, not the
    layer's middle, 2026-10-14)."""
    z0, z1 = build.z(build.layers[m])
    return z0, min(z1, z0 + build.ctx.sheet_t("link", m))


def ring_z(build: Build, k: int) -> tuple[float, float]:
    """A spacer ring's z in layer ``k``: the default sheet's thickness on the layer's
    floor (a layer an aluminium plate thickens is a little taller than a ring)."""
    z0, z1 = build.z(k)
    return z0, min(z1, z0 + build.ctx.pitch)


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
    own: dict[int, float] = {}
    for m in group.axis.members:
        k = plan.layers[m]
        own[k] = max(own.get(k, 0.0), ctx.sheet_t("link", m))
    return sum(max(0.0, plan.t(k) - own.get(k, ctx.pitch)) for k in range(a, b + 1)
               if 0 < k < plan.top)


@dataclass(frozen=True)
class Stock:
    """What a pivot's column holds between its links, from stock and cut plates only
    (:meth:`construction.axle.AxleGroup.realize` applies it after the construction builds):
    ``tube`` cut to length between neighbours (:func:`stock_tubes`), ``shim`` stacked for
    any thinner spacer and a full-layer ring cut from the default sheet
    (:func:`stock_spacers`); ``land``: the land washers a short span takes instead of a
    tube; ``plates``: the column's tubes run to the frame plates it is anchored in (a
    pillar). The arm's of 2026-10-12, the walkers' since 2026-10-14 (the user's "fewer
    printed spacers")."""

    tube: str
    shim: str
    land: str = ""
    plates: bool = False


TUBE_MIN = 0.2          # a gap this short between neighbours is left as the column's play


def stock_tubes(out: Realized, build: Build, group: AxleGroup, tube_key: str,
                members, plates: bool = False, land_key: str = "") -> int:
    """The column between its links made of stock tubes (precision collars cut to length):
    one per gap between neighbouring links or hubs, face to face, in place of the rings and
    gap spacers there (a ring is one default-sheet layer tall: in a layer a thick link
    thickens it left a gap, and the column's preload stopped at it: the review of
    2026-10-12). Their OD bears on the bearings' inner rings only. A span the land washers
    (``land_key``) stack to, a clearance gap :func:`land_gaps` kept between two neighbours,
    takes those instead. Returns how many tubes."""
    from spiderpig.hardware.catalog import get as _get

    xy = xy_of(build, group)
    d = _get(tube_key).dims
    od, bore = float(d["od"]), float(d["id"])
    dims = group.dims(build.ctx)
    # a tube stands in a link's layer, over a thin link's face, only inside the seat the axle
    # claims there (a bearing's); else from the next layer's floor (2026-10-14)
    inside = (dims.seat or dims.axle) + EPS >= od / 2
    elements = [link_z(build, m) if inside else build.z(build.layers[m]) for m in members]
    col = Column.of(build, group)           # where the column claims less than the tube (a
    mine = {build.layers[m] for m in members}   # drive's or a crank's hub round it, a neck)
    elements += [build.z(k) for k, (_, r) in col.roles.items()
                 if k not in mine and r + EPS < od / 2]
    elements += [build.plan.gap_z(k) for k, (_, r, _) in col.gaps.items() if r + EPS < od / 2]
    lo_k, hi_k = min(col.roles), max(col.roles)
    elements += [build.z(k) for k in range(lo_k, hi_k + 1)
                 if k not in col.roles and k not in mine]       # nothing of its own there
    for b in out.bodies:            # a hub; a pillar's end shims
        if b.part is not None and ("_hub_" in b.name or "_shims" in b.name):
            bb = cast(Any, b.part).bounding_box()
            elements.append((bb.min.Z, bb.max.Z))
    if plates:                      # a pillar's column runs to the frame plates it's in
        elements += [build.z(k) for k in Column.of(build, group).anchors]
    elements.sort()
    spans = [(a[1], b[0]) for a, b in itertools.pairwise(elements) if b[0] - a[1] > TUBE_MIN]
    host = None
    keep = []
    for b in out.bodies:            # the printed rings and gap spacers inside a span go
        if b.fab == "printed" and b.part is not None:
            bb = cast(Any, b.part).bounding_box()
            zc = (bb.min.Z + bb.max.Z) / 2
            if any(lo - EPS <= zc <= hi + EPS for lo, hi in spans):
                host = host or b.rigid_with
                continue
        keep.append(b)
    out.bodies[:] = keep
    stem = stem_of(group)
    host = host or sorted(members, key=lambda m: build.layers[m])[0]
    land = _get(land_key).dims if land_key else None
    tubes = []
    for i, (lo, hi) in enumerate(spans):
        n = round((hi - lo) / float(land["t"])) if land else 0
        if land and 1 <= n <= LAND_MAX and abs(hi - lo - n * float(land["t"])) <= LAND_TOL:
            out.bodies.append(hardware(f"{stem}_land{i}", ring(
                xy, float(land["od"]), float(land["id"]), lo, lo + n * float(land["t"])),
                host, fab="purchased", bom_key=land_key, color=STEEL))
            if n > 1:
                out.extras.append(BomLine(land_key, n - 1, f"{group.name}: {n} lands stacked"))
            continue
        tubes.append((lo, hi))
    for i, (lo, hi) in enumerate(tubes):
        out.bodies.append(hardware(f"{stem}_tube{i}", ring(xy, od, bore, lo, hi), host,
                                   fab="purchased", bom_key=tube_key, color=STEEL))
    out.notes.setdefault("tubes", {})[group.name] = [round(hi - lo, 1) for lo, hi in tubes]
    return len(tubes)


LAND_MAX = 2            # a span up to this many land washers takes them, not a tube
LAND_TOL = 0.15         # ... within this (the disc spring takes it up)


def stock_spacers(out: Realized, build: Build, shim_key: str, land_key: str,
                  riders: int) -> None:
    """What the column holds between its links made from stock and cut plates only (the
    user's of 2026-10-12: "only stock parts except for the plates"): a printed ring a full
    default-sheet layer tall is cut from that sheet (``laser``: a cut plate); any other
    printed spacer (a head spacer, a gap ring) is a stack of DIN 988 shims (``shim_key``,
    :func:`hardware.bom.stack`). And a land washer (``land_key``) each side of ``riders``
    links (0: the column's tubes are its lands, :func:`stock_tubes`)."""
    from dataclasses import replace as _replace

    from spiderpig.hardware.bom import stack
    from spiderpig.hardware.catalog import get as _get

    steps = tuple(float(t) for t in _get(shim_key).dims["t"])
    pitch = build.ctx.pitch
    for i, b in enumerate(out.bodies):
        if b.fab != "printed" or b.part is None:
            continue
        bb = cast(Any, b.part).bounding_box()
        h = bb.max.Z - bb.min.Z
        if abs(h - pitch) < 0.05:
            out.bodies[i] = _replace(b, fab="laser", sheet=None)
            continue
        shims, _ = stack(h, steps)
        if not shims:
            continue
        out.bodies[i] = _replace(b, fab="purchased", bom_key=shim_key, color=STEEL)
        if len(shims) > 1:
            out.extras.append(BomLine(shim_key, len(shims) - 1,
                                      f"{b.name}: {h:.1f} mm of shims"))
    if riders:
        out.extras.append(BomLine(land_key, 2 * riders, "the riders' inner-ring lands"))
