"""Pieces the metal-shaft pivots share: the claimed column, the rod with its push-on
clips, laser-cut rings, printed sleeves and small hardware solids."""

from __future__ import annotations

from dataclasses import dataclass, field

from build123d import Axis, Box, Face, Location, Solid, Vector, Wire

from spiderpig.construction.axle import AxleGroup
from spiderpig.construction.base import (
    FRAME_INNER,
    FRAME_OUTER,
    Build,
    ConstructionError,
    Realized,
    hardware,
)
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.catalog import get
from spiderpig.shapes import Cut, disc, moved

STEEL = "#4a4a4a"
RING_COLOR = "#a9cbe0"       # laser-cut spacer rings (the same sheet as the links)
SLEEVE_COLOR = "#1baf7a"     # printed spacer sleeves (the printed axle's green)
EPS = 1e-9

ROD_KEY = "rod_3mm_100"
CLIP_KEY = "starlock_3mm"
RETAINED = ("axle", "anchor")            # column roles the ends clamp between
SPACER_ROLES = ("shoulder", "spacer", "neck")
FLANGE_ROOM = ("shoulder", "spacer", "head", "cap")


def xy_of(build: Build, group: AxleGroup) -> tuple[float, float]:
    xy = build.xy(group.axis.name)
    return float(xy[0]), float(xy[1])


def host_of(build: Build, group: AxleGroup) -> str:
    """What the axle's loose parts move with: the frame (pillars) or the lowest link (pins)."""
    if group.pillar:
        return build.plan.topo.frame_bodies[0]
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
            links[k] = links.get(k, ()) + (m,)
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

    def r(self, k: int) -> float:
        return self.roles[k][1]

    @property
    def k0(self) -> int:
        """The lowest retained layer: the outer anchor, or the lowest link."""
        return min(k for k, (role, _) in self.roles.items() if role in RETAINED)

    @property
    def k1(self) -> int:
        return max(k for k, (role, _) in self.roles.items() if role in RETAINED)

    @property
    def below(self) -> list[int]:
        """Layers claimed beyond the bottom end, nearest first (a retainer in the clearance
        gap under the stack: that gap's layer, ``k0 - 1``)."""
        out = sorted((k for k in self.roles if k < self.k0), reverse=True)
        return [self.k0 - 1] + out if self.lo_gap else out

    @property
    def above(self) -> list[int]:
        out = sorted(k for k in self.roles if k > self.k1)
        return [self.k1 + 1] + out if self.hi_gap else out

    @property
    def between(self) -> list[int]:
        """The spacer layers between the ends (every non-link layer, for a filled column)."""
        return [k for k in range(self.k0 + 1, self.k1) if self.role(k) in SPACER_ROLES]

    @property
    def runs(self) -> list[list[int]]:
        """Maximal runs of adjacent spacer layers: one sleeve each."""
        out: list[list[int]] = []
        for k in self.between:
            if out and out[-1][-1] == k - 1:
                out[-1].append(k)
            else:
                out.append([k])
        return out

    @property
    def anchors(self) -> list[int]:
        return sorted(k for k, (role, _) in self.roles.items() if role == "anchor")

    def room(self, k: int) -> float:
        """Radius free for a link's flange in layer ``k`` (0 at a link or a frame plate)."""
        role, r = self.roles.get(k, ("", 0.0))
        return r if role in FLANGE_ROOM else 0.0


def hex_prism(xy, af: float, z0: float, z1: float, angle: float = 0.0):
    """A hexagonal prism ``af`` across flats, one pair of flats facing ``angle`` (degrees)."""
    boxes = [Box(af, 4 * af, z1 - z0).rotate(Axis.Z, angle + a) for a in (0.0, 60.0, 120.0)]
    prism = boxes[0] & boxes[1] & boxes[2]
    return moved(prism, Location((float(xy[0]), float(xy[1]), (z0 + z1) / 2)))


def bored(part, xy, d: float, z0: float, z1: float):
    """``part`` with a through-bore of diameter ``d``."""
    return part - disc(xy, d / 2, z0 - 1.0, z1 + 1.0)


def sleeve_solid(xy, pieces: list[tuple[float, float, float]], bore_d: float) -> Solid:
    """A printed spacer sleeve on a rod: cylinders ``(z0, z1, r)`` stacked bottom to top.

    Where the radius changes the outside runs along a 45 degree cone inside
    the wider piece, so the sleeve prints on either end without support and
    never leaves its claimed layers.
    """
    pts: list[tuple[float, float]] = []
    for i, (z0, z1, r) in enumerate(pieces):
        if i == 0:
            pts.append((r, z0))
        else:
            _, _, r_prev = pieces[i - 1]
            step = abs(r - r_prev)
            if step > EPS:
                if step > min(z1 - z0, pieces[i - 1][1] - pieces[i - 1][0]) + EPS:
                    raise ConstructionError(f"a sleeve steps by {step:.2f} mm between pieces "
                                            f"shorter than that")
                if r > r_prev:                   # step out going up: cone into this piece
                    pts += [(r_prev, z0), (r, z0 + step)]
                else:                            # step in going up: cone into the piece below
                    pts += [(r_prev, z0 - step), (r, z0)]
        pts.append((r, z1))
    pts += [(bore_d / 2, pieces[-1][1]), (bore_d / 2, pieces[0][0])]
    clean: list[tuple[float, float]] = []
    for p in pts:
        if not clean or abs(clean[-1][0] - p[0]) > 1e-9 or abs(clean[-1][1] - p[1]) > 1e-9:
            clean.append(p)
    wire = Wire.make_polygon([Vector(r, 0.0, z) for r, z in clean], close=True)
    solid = Solid.revolve(Face(wire), 360.0, Axis.Z)
    return moved(solid, Location((float(xy[0]), float(xy[1]), 0.0)))


@dataclass(frozen=True)
class RodShaft:
    """A 3 mm rod cut to length: glued into the frame plates it reaches, a push-on clip
    (Starlock) against the stack at every free end, the rod's end just proud of the clip.

    The rod is modelled ``model_gap`` under size so it clears the bores it
    runs in (the clip's, a bearing's), which are modelled at size.
    """

    rod_key: str = ROD_KEY
    clip_key: str = CLIP_KEY
    protrude: float = 0.5            # rod beyond a clip
    set_play: float = 0.1            # axial play a clip pushed on "until it just touches" leaves
    #                                  (an assumption: a push-on clip is set by feel)
    model_gap: float = 0.01
    glue_per_anchor: float = 0.02    # CA glue per plate anchor, as a fraction of a bottle

    @property
    def d(self) -> float:
        return float(get(self.rod_key).dims["d"])

    @property
    def stock(self) -> float:
        return float(get(self.rod_key).dims["length"])

    def clip(self) -> tuple[float, float]:
        """(outside diameter, height) of the push-on clip."""
        c = get(self.clip_key).dims
        return float(c["od"]), float(c["h"])

    def end_height(self) -> float:
        """What a clip and the rod's end need in a clearance gap."""
        _, h = self.clip()
        return h + self.protrude + HEAD_CLEARANCE

    def check(self, ctx, pillar: bool, extra: float = 0.0) -> None:
        """The clip and the rod's end fit an end layer (``extra``: a flange under the clip)."""
        p = ctx.params
        _, h = self.clip()
        if h + self.protrude + extra > ctx.pitch + EPS:
            raise ConstructionError(f"a {h:g} mm push-on clip and the rod's end don't fit a "
                                    f"{ctx.pitch:g} mm layer")
        if pillar and p.hole(self.d, "glue") / 2 + p.min_wall > p.frame_radius:
            raise ConstructionError(f"a {self.d:g} mm rod doesn't fit the frame plate arms")

    def realize(self, build: Build, group: AxleGroup, col: Column, out: Realized, *,
                faces: dict[str, float] | None = None) -> None:
        """Add the rod, its clips, the plate holes and the glue to ``out``.

        ``faces["lo"]`` / ``["hi"]``: how far a link's flange holds the bottom /
        top clip off the retained stack's face.
        """
        faces = faces or {}
        p = build.ctx.params
        xy, host, stem = xy_of(build, group), host_of(build, group), stem_of(group)
        od, h = self.clip()
        d = self.d
        z0, z1 = build.z(col.k0)[0], build.z(col.k1)[1]
        if col.below:
            face = z0 - faces.get("lo", 0.0)
            clip = bored(disc(xy, od / 2, face - h, face), xy, d, face - h, face)
            out.bodies.append(hardware(f"{stem}_clip_lo", clip, host, fab="purchased",
                                       bom_key=self.clip_key, color=STEEL))
            z0 = face - h - self.protrude
        if col.above:
            face = z1 + faces.get("hi", 0.0)
            clip = bored(disc(xy, od / 2, face, face + h), xy, d, face, face + h)
            out.bodies.append(hardware(f"{stem}_clip_hi", clip, host, fab="purchased",
                                       bom_key=self.clip_key, color=STEEL))
            z1 = face + h + self.protrude
        rod = disc(xy, (d - self.model_gap) / 2, z0, z1)
        out.bodies.append(hardware(f"{stem}_rod", rod, host, fab="purchased", color=STEEL))
        out.extras.append(BomLine(self.rod_key, (z1 - z0) / self.stock,
                                  f"{group.name}: cut {z1 - z0:.1f} mm"))
        plates = {0: FRAME_OUTER, build.top: FRAME_INNER}
        for k in col.anchors:
            out.cut(plates[k], Cut(xy, p.hole(d, "glue")))
        if col.anchors:
            out.extras.append(BomLine("ca_glue", self.glue_per_anchor * len(col.anchors),
                                      f"{group.name} anchors"))


HEAD_CLEARANCE = 0.25    # a retainer in a clearance gap stays this far off the next layer (mm)


def ring_z(build: Build, k: int) -> tuple[float, float]:
    """A laser-cut ring's z in layer ``k``: the default sheet's thickness on the layer's
    floor (a layer an aluminium plate thickens is a little taller than a ring)."""
    z0, z1 = build.z(k)
    return z0, min(z1, z0 + build.ctx.pitch)


def gap_washers(build: Build, group: AxleGroup, col: Column, out: Realized, shaft_d: float,
                host: str, stem: str, color: str = "#f2f2f2") -> float:
    """The washers an axle carries through every clearance gap of its column (the plan's):
    a PTFE washer and DIN 988 shims stacked to the gap (:func:`materials.washer_stack`),
    one body per gap (its first washer's BOM line, the rest as extras). Returns the play
    the stacks leave in all (mm)."""
    from spiderpig.materials import washer_od, washer_stack

    xy = xy_of(build, group)
    play = 0.0
    for k in col.washers:
        g = build.plan.gaps.get(k, 0.0)
        if g <= 0:
            continue
        items, left = washer_stack(shaft_d, g)
        play += left
        if not items:
            continue
        z0, _ = build.plan.gap_z(k)
        t = sum(x[1] for x in items)
        od = washer_od(shaft_d)
        part = bored(disc(xy, od / 2, z0, z0 + t), xy, shaft_d + 0.2, z0, z0 + t)
        out.bodies.append(hardware(f"{stem}_gap{k}_washers", part, host, fab="purchased",
                                   bom_key=items[0][0], color=color))
        for key, tt in items[1:]:
            out.extras.append(BomLine(key, 1, f"{group.name}: {tt:g} mm in the gap over "
                                              f"layer {k}"))
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
