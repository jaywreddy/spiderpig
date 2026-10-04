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

**The default link pin until the pivot review of 2026-10-03** (now :mod:`.chicago`: the
same stacks, play set by the barrel instead of by feel, a stronger 4 mm barrel; the
review's table is there). Note from that review, at each design's own loads
(the strength check of 2026-10-03, ``docs/audit/STRENGTH.md``: MuJoCo, jammed at the
servo's torque limit, a two-link pin bending by ``F s / 2``): the demo Klann
(``--linkage klann``) quad at phases 0,0,180,180 puts pin E across 9 mm (four layers),
where the rod jams at SF 0.53 (the Chicago screw 0.80; both fail, 245 N jammed);
``klann_lego`` quad at the same phases spans every pin 3 mm, 13 layers, SF 2.02 (the
Chicago screw 2.62), $227.58 against the Chicago screw's $206.09; the Strider double 2.23
against 3.55. (The review
first had 1.57 / 3.71 at a family-wide 155 N and ``F s / 4``.) Reviewed against the
other pins on the Strider double and the demo Klann quad (the pin review of 2026-10): rod
pins with printed pillars plan at 16 layers (Strider) and 12 (Klann) in seconds, the
same stacks as the printed snap pin, where a ``bolt`` pin's nut claims two layers and
needs a head nobody can reach once the lowest link is on (:mod:`.bolt`); the smooth h9
rod has the least play of the plain pivots (0.20-0.23 mm, the thread of a bolt rides the
holes) and holds every pin of the test designs at their own loads (above, the demo Klann's pin E
with a warning); and only
the pillars and crank snap now, so the audit's worst snap strain drops from 3.8 % (a
relieved lip) to 1.6 %. Its costs: 24 cut pieces on a Strider (9.6, 12.6 and 15.6 mm:
two to four layers plus the clips, 266 mm of rod; the BOM carries the cut list), a clip
per end pushed on with a tube, +24 parts over printed pins. Rod *pillars* are not the
default: a rod can't neck, so a pillar's column fills the whole stack with rings.

Assembly, bottom up (the frame's outer plate down, as for the printed
axles): deburr every cut end; push the **lower clip** onto the cut rod
first, flat side to the stack; thread the rings and links on in layer order
(the layer plan in ``build/audit`` or ``spiderpig build`` says which); when
the pin's top link is the top of the stack push the **upper clip** on with
a 5.5 mm socket or a short tube over the rod until it just touches the
link: the links must still turn freely. A clip pushed too far clamps the
links and has to be pried off and replaced (the kit has spares; a clip
doesn't back off). A pillar's rod goes through the outer plate first (CA
glue), its clip under that plate, the inner plate last.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from spiderpig.construction.axle import AxleDims, AxleGroup
from spiderpig.construction.base import Build, ConstructionError, Context, Realized, hardware
from spiderpig.construction.pivots.common import (
    SLEEVE_COLOR,
    Column,
    RodShaft,
    gap_washers,
    host_of,
    ring_z,
    stem_of,
    xy_of,
)
from spiderpig.construction.wobble import Section, column_wobble
from spiderpig.materials import washer_od
from spiderpig.shapes import Cut, ring


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
        return AxleDims(axle=d / 2, spacer=p.spacer_d / 2, head=od / 2, neck=ring_min, fill=True,
                        end_h=(self.shaft.end_height(),) * 2, washer=washer_od(d) / 2)

    def realize(self, group: AxleGroup, build: Build) -> Realized:
        """Rings in every spacer layer, the rod with its clips, running-fit holes in the links."""
        out = Realized()
        col = Column.of(build, group)
        xy, host, stem = xy_of(build, group), host_of(build, group), stem_of(group)
        for k in col.between:
            role, r = col.roles[k]
            if role == "neck":
                raise ConstructionError(f"{group.name}: a rod can't neck down (layer {k})")
            z0, z1 = ring_z(build, k)
            out.bodies.append(hardware(f"{stem}_ring{k}", ring(xy, 2 * r, self.hole(), z0, z1),
                                       host, fab="printed", color=SLEEVE_COLOR))
        self.shaft.realize(build, group, col, out)
        gap_washers(build, group, col, out, self.shaft.d, host, stem)
        for m in group.axis.members:
            out.cut(m, Cut(xy, self.hole()))
        out.notes["wobble"] = {group.name: column_wobble(
            build, group, col, clearance=self.running_fit, length=build.ctx.pitch,
            play=self.shaft.set_play,
            play_basis=f"push-on clip set to touch ({self.shaft.set_play:g} mm assumed)",
            section=Section.rod(self.shaft.d))}
        return out
