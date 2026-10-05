"""``ptfe``: the ``rod`` pin with a PTFE tube liner pressed into every link.

A 3 x 4 mm PTFE tube (Bowden tube, :data:`TUBE_KEY`) is cut into liners one sheet
long, and each link gets one, pressed into a 4 mm hole cut ``seat_fit`` under the
tube's outside diameter; the 3 mm rod, its printed spacer rings and its Starlock
clips are the ``rod`` construction's (:class:`construction.pivots.common.RodShaft`).
Each link then turns PTFE on steel (mu about 0.05-0.1, no lubricant) instead of acrylic
on steel, with the liner's ID tolerance as the bore clearance.

Its limit is the liner, not the rod: virgin PTFE yields in compression at about 10-12
MPa and creeps well below that, so the strength check takes :data:`PTFE_LIMIT_MPA`
(10 MPa) over the bearing pressure ``F / (d L)`` on the 3 x 3 mm bore as the liner's
safety factor (:attr:`construction.wobble.Section.bearing_limit_mpa`): the walking
warning at SF 3 keeps the walking pressure under 3.3 MPa, about PTFE's long-term
creep limit, and the jam check at SF 2 keeps a stall at the torque limit under 5 MPa.
With each design's own loads (:mod:`sim.loads`, ``docs/audit/STRENGTH.md``, 2026-10-03)
it is viable walking (0.6-0.9 MPa, SF 11-18), but jammed at the 0.85 N·m torque limit
the Strider double's 50 N on a pin is 5.6 MPa (SF 1.79, a warning) and ``klann_lego``
quad at 0,0,180,180's 172 N 19.2 MPa (SF 0.52, an error), where the Chicago screw holds
3.55 and 2.62. It buys free tilt (0.96 deg against the plain holes' 3.8) for $21-24 and
24-42 parts more than the Chicago screw. Selectable, not the default, for designs whose
pins jam under ~90 N; lowering the servo's torque limit to 0.45 N·m roughly halves the
jam loads (SF ~3.4 on the Strider double).

Assembly: cut the liners square to the sheet thickness (a razor in a mitre jig), press
one into each link's hole flush both faces (a flat block), then as the rod: lower
clip, rings and links in layer order, upper clip until it just touches.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from spiderpig.construction.axle import AxleDims, AxleGroup
from spiderpig.construction.base import Build, ConstructionError, Context, Realized, hardware
from spiderpig.construction.pivots.common import (
    SLEEVE_COLOR,
    Column,
    RodShaft,
    bored,
    gap_washers,
    host_of,
    ring_z,
    stem_of,
    xy_of,
)
from spiderpig.construction.wobble import Section, column_wobble
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.catalog import get
from spiderpig.materials import washer_od
from spiderpig.shapes import Cut, disc, ring

TUBE_KEY = "ptfe_tube_3x4_1m"
PTFE_LIMIT_MPA = 10.0
PTFE_COLOR = "#f2f2f2"


def ptfe_section(d: float = 3.0) -> Section:
    """The 3 mm rod, its links bearing on PTFE liners (:data:`PTFE_LIMIT_MPA`)."""
    return Section(f"{d:g} mm rod in PTFE liners", d, Section.rod(d).z_bend,
                   Section.rod(d).a_shear, 215.0, bearing_limit_mpa=PTFE_LIMIT_MPA)


@dataclass(frozen=True)
class PtfeAxle:
    """3 mm rod, a PTFE liner pressed in every link, printed rings, push-on clips."""

    key: str = "ptfe"
    label: str = ("3 mm steel rod in PTFE tube liners (3 x 4 mm, one pressed in each link), "
                  "printed spacer rings, push-on clips")
    running_fit: float = 0.2      # a ring's hole over the rod
    seat_fit: float = -0.05       # a link's hole over the tube's OD: a light press
    bore_clearance: float = 0.05  # the liner's bore over the rod (the tube's ID tolerance)
    tube_key: str = TUBE_KEY
    model_gap: float = 0.01
    shaft: RodShaft = field(default_factory=RodShaft)

    def tube(self) -> tuple[float, float]:
        d = get(self.tube_key).dims
        return float(d["id"]), float(d["od"])

    def hole(self) -> float:
        return self.shaft.d + self.running_fit

    def seat(self) -> float:
        return self.tube()[1] + self.seat_fit

    def dims(self, ctx: Context, pillar: bool) -> AxleDims:
        p = ctx.params
        d = self.shaft.d
        tid, _ = self.tube()
        if abs(tid - d) > 1e-9:
            raise ConstructionError(f"a {tid:g} mm PTFE liner doesn't take the {d:g} mm rod")
        if self.seat() / 2 + p.min_wall > p.link_radius:
            raise ConstructionError(f"a {self.seat():.2f} mm liner seat leaves less than "
                                    f"{p.min_wall} mm of link around it")
        self.shaft.check(ctx, pillar)
        ring_min = self.hole() / 2 + p.min_wall
        if ring_min > p.spacer_d / 2:
            raise ConstructionError(f"a {p.spacer_d} mm spacer ring leaves less than {p.min_wall} "
                                    f"mm around a {self.hole():.2f} mm hole")
        od, _ = self.shaft.clip()
        return AxleDims(axle=d / 2, spacer=p.spacer_d / 2, head=od / 2, neck=ring_min, fill=True,
                        end_h=(self.shaft.end_height(),) * 2, washer=washer_od(d) / 2,
                        seat=self.tube()[1] / 2)

    def realize(self, group: AxleGroup, build: Build) -> Realized:
        """Rings, the rod and clips (the rod's), a liner in every link."""
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
        tid, tod = self.tube()
        for k, members in col.links.items():
            z0, z1 = build.z(k)
            for m in members:
                # modelled at the seat (the press closes the 0.05 mm), a hair under
                liner = bored(disc(xy, (min(tod, self.seat()) - self.model_gap) / 2, z0, z1),
                              xy, tid, z0 - 1, z1 + 1)
                out.bodies.append(hardware(f"{stem}_{m}_liner", liner, m, fab="purchased",
                                           color=PTFE_COLOR))
                out.extras.append(BomLine(self.tube_key, (z1 - z0) / float(
                    get(self.tube_key).dims["length"]), f"{group.name} {m}: cut {z1 - z0:.1f} mm"))
                out.cut(m, Cut(xy, self.seat()))
        out.notes["wobble"] = {group.name: column_wobble(
            build, group, col, clearance=self.bore_clearance, length=build.ctx.pitch,
            play=self.shaft.set_play,
            play_basis=f"push-on clip set to touch ({self.shaft.set_play:g} mm assumed)",
            section=ptfe_section(self.shaft.d))}
        return out
