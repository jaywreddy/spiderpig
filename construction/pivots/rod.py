"""``rod``: a 3 mm steel rod with laser-cut spacer rings and push-on clips.

The rod is cut to length from stock. Every layer between the ends that holds
no link holds a **ring** cut from the same sheet as the links (one layer
thick by construction, bore a running fit on the rod, outside diameter the
claim's), so each link is held by a ring, a clip or a frame plate on each
face. A **pillar**'s rod is glued (CA) into both frame plates it reaches,
with a clip under the outer plate; a **pin**'s rod carries a clip against
its lowest and its highest link. The rod can't neck down, so a link passing
closer than the narrowest ring the laser should cut blocks the layer
(``neck`` is that ring's radius) and the planner has to route around it.

Assembly: push the bottom clip on, thread rings and links on in layer
order, push the top clip on until it just touches; a pillar's rod goes
through the outer plate first (glue), the inner plate last.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from construction.axle import AxleDims, AxleGroup
from construction.base import Build, ConstructionError, Context, Realized, hardware
from construction.pivots.common import RING_COLOR, Column, RodShaft, host_of, stem_of, xy_of
from shapes import Cut, ring


@dataclass(frozen=True)
class RodAxle:
    """3 mm rod, laser-cut spacer rings, Starlock push-on clips."""

    key: str = "rod"
    label: str = "3 mm steel rod, laser-cut spacer rings, push-on clips (glued into the frame)"
    running_fit: float = 0.2      # a link's and a ring's hole over the rod (3.2 mm: ISO 273 fine)
    shaft: RodShaft = field(default_factory=RodShaft)

    def hole(self) -> float:
        return self.shaft.d + self.running_fit

    def dims(self, ctx: Context, pillar: bool) -> AxleDims:
        p = ctx.params
        d = self.shaft.d
        if self.hole() / 2 + p.min_wall > p.link_radius:
            raise ConstructionError(f"a {d:g} mm rod's hole leaves less than {p.min_wall} mm of "
                                    f"link around it (link radius {p.link_radius})")
        self.shaft.check(ctx, pillar)
        ring_min = self.hole() / 2 + p.min_wall        # the narrowest ring worth cutting
        if ring_min > p.spacer_d / 2:
            raise ConstructionError(f"a {p.spacer_d} mm spacer ring leaves less than {p.min_wall} "
                                    f"mm around a {self.hole():.2f} mm hole")
        od, _ = self.shaft.clip()
        return AxleDims(axle=d / 2, spacer=p.spacer_d / 2, head=od / 2, neck=ring_min, fill=True)

    def realize(self, group: AxleGroup, build: Build) -> Realized:
        """Rings in every spacer layer, the rod with its clips, running-fit holes in the links."""
        out = Realized()
        col = Column.of(build, group)
        xy, host, stem = xy_of(build, group), host_of(build, group), stem_of(group)
        for k in col.between:
            role, r = col.roles[k]
            if role == "neck":
                raise ConstructionError(f"{group.name}: a rod can't neck down (layer {k})")
            z0, z1 = build.z(k)
            out.bodies.append(hardware(f"{stem}_ring{k}", ring(xy, 2 * r, self.hole(), z0, z1),
                                       host, fab="laser", color=RING_COLOR))
        self.shaft.realize(build, group, col, out)
        for m in group.axis.members:
            out.cut(m, Cut(xy, self.hole()))
        return out
