"""``bearing`` / ``bushing``: a flanged insert in each link on a 3 mm rod, printed sleeves.

Each link gets a flanged insert seated in its hole: a shielded miniature
ball bearing (``bearing``, MF63ZZ 3 x 6 x 2.5, flange 7.2 x 0.6, glued in
with CA since steel can't be pressed into acrylic) or a polymer flange
bushing (``bushing``, igus GFM-0304-03, 3 x 4.5 x 3, flange 7.5 x 0.75,
pressed). The insert's body is shorter than the sheet, so the whole link
turns on it; its flange lies on one face of the link, in the neighbouring
layer, and the **sleeve** or clip there bears on the flange. Two links in
adjacent layers turn their flanges outwards; three in a row, or a link
against a frame plate with a link on its other face, have no room for the
middle flange and are unbuildable (:func:`construction.axle.flange_sides`).

Between links the rod carries **printed sleeves**: one per run of spacer
layers, stepped to the claimed radius of each layer, shortened by a flange
where one reaches into the run. The shaft, its clips and its plate anchors
are the ``rod`` construction's (:class:`construction.pivots.common.RodShaft`).
The rod should be a snug fit in the bearing's bore (an m6 dowel pin is) so
the inner race turns with the rod and the balls, not the rod, take the
motion.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from spiderpig.construction.axle import AxleDims, AxleGroup, flange_sides
from spiderpig.construction.base import Build, ConstructionError, Context, Realized, hardware
from spiderpig.construction.pivots.common import (
    SLEEVE_COLOR,
    STEEL,
    Column,
    RodShaft,
    bored,
    gap_washers,
    host_of,
    sleeve_solid,
    stem_of,
    xy_of,
)
from spiderpig.construction.wobble import Section, column_wobble
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.catalog import get
from spiderpig.shapes import Cut, disc, union


@dataclass(frozen=True)
class InsertDims:
    id: float
    od: float
    width: float          # over all, flange included
    flange_d: float
    flange_t: float

    @property
    def body(self) -> float:
        """Length of the part that sits in the link's hole."""
        return self.width - self.flange_t


@dataclass(frozen=True)
class InsertAxle:
    """A flanged bearing or bushing in every link, on a 3 mm rod with printed sleeves."""

    gaps: ClassVar[bool] = False    # its retainers assume full layers (no clearance gaps)

    key: str
    label: str
    insert: str                   # catalog key of the flanged insert
    seat_fit: float               # a link's hole over the insert's outside diameter
    glued: bool                   # CA-glue each insert into its hole
    shaft: RodShaft = field(default_factory=RodShaft)   # or a ChicagoShaft (.chicago)
    spacer_d: float | None = None   # sleeves' and spacers' diameter when not Params.spacer_d
    bore_clearance: float = 0.05    # the insert's bore over the shaft, diametral (wobble)
    sleeve_fit: float = 0.3       # sleeve bore over the rod (a printed part sliding on)
    sleeve_wall: float = 0.85     # thinnest printed wall of a sleeve
    flange_play: float = 0.1      # gap between a flange and the sleeve beside it
    min_sleeve: float = 1.0       # least sleeve left in a layer a flange reaches into
    glue_per_insert: float = 0.005

    def column(self, *args, **kw) -> None:
        """The shaft's own rule over the column, if it has one (a Chicago screw's stock
        barrel lengths)."""
        rule = getattr(self.shaft, "column", None)
        if rule is not None:
            rule(*args, **kw)

    @property
    def roles(self) -> tuple[str, ...]:
        """What it may build: pillars and pins, unless its shaft is a pin only."""
        return getattr(self.shaft, "roles", ("pillar", "pin"))

    def insert_dims(self) -> InsertDims:
        item = get(self.insert)
        d = item.dims
        try:
            return InsertDims(id=float(d["id"]), od=float(d["od"]),
                              width=float(d["w"] if "w" in d else d["l"]),
                              flange_d=float(d["flange_d"]), flange_t=float(d["flange_t"]))
        except KeyError as e:
            raise ConstructionError(f"{item.name} isn't a flanged insert: its catalog entry has "
                                    f"no {e.args[0]!r}") from None

    def seat(self) -> float:
        """Diameter of the hole cut in a link for the insert."""
        return self.insert_dims().od + self.seat_fit

    def dims(self, ctx: Context, pillar: bool) -> AxleDims:
        p = ctx.params
        ins = self.insert_dims()
        name = get(self.insert).name
        rod = self.shaft.d
        if abs(ins.id - rod) > 1e-9:
            raise ConstructionError(f"{name} takes a {ins.id:g} mm shaft, not the {rod:g} mm rod")
        if self.seat() / 2 + p.min_wall > p.link_radius:
            raise ConstructionError(f"a {self.seat():.2f} mm hole for {name} leaves less than "
                                    f"{p.min_wall} mm of link around it")
        if ins.body > ctx.pitch + 1e-9:
            raise ConstructionError(f"{name} is {ins.body:g} mm long under its flange, more than "
                                    f"the {ctx.pitch:g} mm sheet")
        if ins.flange_t + self.flange_play + self.min_sleeve > ctx.pitch + 1e-9:
            raise ConstructionError(f"the {ins.flange_t:g} mm flange of {name} leaves no room "
                                    f"for a sleeve in a {ctx.pitch:g} mm layer")
        self.shaft.check(ctx, pillar, extra=ins.flange_t)
        spacer = (p.spacer_d if self.spacer_d is None else self.spacer_d) / 2
        clip_od, _ = self.shaft.clip()
        if ins.flange_d / 2 > spacer or (self.spacer_d is None and ins.flange_d > clip_od):
            raise ConstructionError(f"the {ins.flange_d:g} mm flange of {name} is wider than a "
                                    f"{2 * spacer:g} mm spacer or a {clip_od:g} mm clip")
        neck = (rod + self.sleeve_fit) / 2 + self.sleeve_wall
        if neck > spacer:
            raise ConstructionError("a sleeve's wall doesn't fit inside a spacer")
        return AxleDims(axle=rod / 2, spacer=spacer, head=max(clip_od, ins.flange_d) / 2,
                        neck=neck, fill=True,
                        flange=ins.flange_d / 2, seat=ins.od / 2)

    def realize(self, group: AxleGroup, build: Build) -> Realized:
        out = Realized()
        col = Column.of(build, group)
        xy, host, stem = xy_of(build, group), host_of(build, group), stem_of(group)
        ins = self.insert_dims()
        names = {k: ", ".join(ms) for k, ms in col.links.items()}
        sides = flange_sides(sorted(col.links), col.room, ins.flange_d / 2, names)
        bonded = host if hasattr(self.shaft, "host_hole") else None   # a Chicago barrel's host
        # the inserts, each glued or pressed into its link, flange on the free face
        for k, members in col.links.items():
            s = sides[k]
            z0, z1 = build.z(k)
            face = z1 if s > 0 else z0
            body = disc(xy, ins.od / 2, *sorted((face, face - s * ins.body)))
            flange = disc(xy, ins.flange_d / 2, *sorted((face, face + s * ins.flange_t)))
            for m in members:
                if m == bonded:
                    continue
                part = bored(union([body, flange]), xy, ins.id, z0 - 1, z1 + 1)
                out.bodies.append(hardware(f"{stem}_{m}_{self.key}", part, m, fab="purchased",
                                           bom_key=self.insert, color=STEEL))
                out.cut(m, Cut(xy, self.seat()))
        # one printed sleeve per run of spacer layers, kept off the flanges beside it
        reach = ins.flange_t + self.flange_play
        for run in col.runs:
            z0, z1 = build.z(run[0])[0], build.z(run[-1])[1]
            if sides.get(run[0] - 1) == +1:
                z0 += reach
            if sides.get(run[-1] + 1) == -1:
                z1 -= reach
            pieces = [(max(build.z(k)[0], z0), min(build.z(k)[1], z1), col.r(k)) for k in run]
            sleeve = sleeve_solid(xy, pieces, self.shaft.d + self.sleeve_fit)
            out.bodies.append(hardware(f"{stem}_sleeve{run[0]}", sleeve, host, fab="printed",
                                       color=SLEEVE_COLOR))
        # the rod and its clips: a clip bears on a flange where the end link's points its way
        faces = {}
        if sides.get(col.k0) == -1 and not (bonded and col.links[col.k0] == (bonded,)):
            faces["lo"] = ins.flange_t
        if sides.get(col.k1) == +1:
            faces["hi"] = ins.flange_t
        fit = self.shaft.realize(build, group, col, out, faces=faces)
        gap_washers(build, group, col, out, self.shaft.d, host, stem)
        if bonded:
            play = fit.play + self.flange_play * sum(
                1 for run in col.runs for k in (run[0] - 1, run[-1] + 1)
                if sides.get(k) == (1 if k < run[0] else -1))
            basis = f"barrel length less stack and shims ({fit.length:g} mm barrel)"
            section = Section.tube(self.shaft.d, 3.0, name="chicago barrel 4 x 3 tube")
        else:
            play = self.shaft.set_play + self.flange_play * sum(
                1 for run in col.runs for k in (run[0] - 1, run[-1] + 1)
                if sides.get(k) == (1 if k < run[0] else -1))
            basis = (f"push-on clip set to touch ({self.shaft.set_play:g} mm assumed) plus "
                     "the flange-to-sleeve gaps")
            section = Section.rod(self.shaft.d)
        out.notes["wobble"] = {group.name: column_wobble(
            build, group, col,
            clearance=lambda m: 0.0 if m == bonded else self.bore_clearance,
            length=lambda m: build.ctx.pitch if m == bonded else ins.body,
            play=play, play_basis=basis, section=section, bearing_len=ins.body)}
        if self.glued:
            n = len(group.axis.members)
            out.extras.append(BomLine("ca_glue", self.glue_per_insert * n,
                                      f"{group.name}: {get(self.insert).name} into its links"))
        return out


BEARING = InsertAxle(
    key="bearing",
    label="MF63ZZ flanged ball bearing glued in each link, 3 mm rod, printed sleeves, clips",
    insert="bearing_mf63zz", seat_fit=0.03, glued=True,
)
BUSHING = InsertAxle(
    key="bushing",
    label="igus GFM-0304-03 flange bushing pressed in each link, 3 mm rod, printed sleeves, clips",
    insert="bushing_gfm0304_03", seat_fit=0.02, glued=False,
)
