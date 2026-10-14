"""``ball`` (pins) and ``ball_pillar`` (pillars): a ball bearing bonded into every link that
turns on the axle.

The plain pivots (``chicago``, ``standoff``) turn each link in a laser-cut hole: 0.2 and
0.35 mm of diametral clearance, the play a load reversal moves the joint across, and the
running fit the link's tilt follows (:mod:`construction.wobble`). These seat a shielded
miniature deep-groove bearing in each link instead, so the link turns on balls and its play
is the bearing's internal clearance (ISO 5753-1 CN, 2-13 um at these sizes) plus how its
bore sits on the shaft:

==============  =======================================================================
key             construction
==============  =======================================================================
``ball``        pins: an MR84ZZ (4 x 8 x 3) in every link but the host, on the M3 Chicago
                screw's 4 mm barrel (:mod:`construction.pivots.chicago`: the barrel, its
                lengths, head spacers and the host link bonded to it, unchanged)
``ball_pillar`` pillars: an MR106ZZ (6 x 10 x 3) in every link on the 6 mm standoff
                (:mod:`construction.pivots.standoff`: the column, its stock lengths and
                end screws, unchanged); a link needs ``link_radius`` 6.6 mm around it
==============  =======================================================================

**The seat.** The bearing is bonded into the link's bore (slow two-part epoxy on the outer
ring only, kept off the shields; acrylic crazes under CA and cracks under a press fit), the
bore ``seat_fit`` (0.2 mm) over the bearing's OD: more than a laser-cut hole's tolerance
(about +-0.13 mm at SendCutSend), so the bore is never a press fit, and a gap the epoxy
fills (centre the bearing on a 4 mm pin while it cures). The bearing is as wide as the link
is thick (3 mm on 3 mm sheet): a link thinner than it as built (the plan's z: give the
measured sheet, ``--thickness``) is refused, since a bearing proud of its link would be
clamped by the rings beside it. The printed spacer rings between links are the plain pins'
(8.5 mm, a 1.5 mm wall a printer holds): they bear on the inner rings, on the outer rings
and on the link faces, so thrust and tilt still pass through faces that slide, as on the
plain pins. A ring that touched the inner ring only (under about 6 mm across for an MR84,
its pitch diameter) would be a bought shim; the robot arm's precision joints, not these,
are where that is worth it.

**The shaft fit.** The bore (4 mm, tolerance +0 / -8 um) on the shaft: the Chicago barrel
is stainless tube, not ground, taken at 4.00 to 3.95 mm (``shaft_fit`` 0.05 mm, the worst
diametral gap); the goBILDA standoff (and the one-piece steel shaft) at 6.00 to 5.97 (0.03).
Both are assumptions, unmeasured: a barrel over 4.00 mm doesn't take the bearing (sand it,
or ream nothing: choose another screw). That and the bearing's own clearance are the
joint's radial play (:meth:`error_terms`), against 0.2 / 0.35 mm plain.

**Tilt.** A link on one deep-groove bearing tilts as far as its bore rocks on the shaft
over the bearing's width (the shaft fit over the width) plus the rings' own free
misalignment, about ``G_a / d_m`` (the axial play its radial clearance allows,
``G_a = sqrt(4 (f_i + f_o - 1) D_w G_r)``, over its pitch diameter; :func:`internal_tilt`:
0.5 deg for an MR84 at CN's worst): :func:`construction.wobble.column_wobble` over the
bearing's width with the clearance that tilts as far, and the faces either side bounding
it (the supported tilt) as on the plain pins.

The vendor model (McMaster-Carr's STEP, :mod:`hardware.bearing_catalog`) is drawn when it
is in the CAD cache and matches the catalog's size; otherwise :func:`bearing_solid`.
Clearances always use the catalog dims.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import ClassVar

from spiderpig.construction.axle import AxleDims, AxleGroup
from spiderpig.construction.base import Build, ConstructionError, Context, Realized, hardware
from spiderpig.construction.joints import ErrorTerms
from spiderpig.construction.pivots.chicago import ChicagoAxle, _hw_tol, chicago_section
from spiderpig.construction.pivots.common import (
    EPS,
    SLEEVE_COLOR,
    STEEL,
    Column,
    gap_washers,
    host_of,
    link_z,
    ring_z,
    stem_of,
    xy_of,
)
from spiderpig.construction.pivots.standoff import StandoffAxle
from spiderpig.construction.wobble import column_wobble
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.catalog import get
from spiderpig.shapes import Cut, ring

BEARING_COLOR = "#b8bcc2"
GLUE_PER_BEARING = 0.003        # of the epoxy kit, per bearing bonded (the BOM's share)
OSCULATION = (0.52, 0.53)       # inner and outer groove radius over the ball's (deep groove,
#                                 the usual values)
BALL_FRACTION = 0.3             # a ball's diameter over the bearing's section (OD - bore)/2 x
#                                 2: the usual proportion, when the catalog gives no ball size


def bearing_dims(key: str) -> dict:
    return get(key).dims


def bearing_solid(key: str, xy, z0: float):
    """Bearing ``key`` standing on ``z0`` at ``xy``: the vendor's model when the CAD cache
    holds one of the catalog's size (:func:`hardware.cad.load_axisymmetric`), else a ring of
    the catalog's bore, OD and width."""
    from build123d import Location

    from spiderpig.hardware.cad import load_axisymmetric, part_cad

    d = bearing_dims(key)
    od, bore, w = float(d["od"]), float(d["id"]), float(d["w"])
    ref = part_cad(key)
    model = load_axisymmetric(ref, od, w) if ref is not None else None
    if model is not None:
        return model.moved(Location((float(xy[0]), float(xy[1]), z0)))
    return ring(xy, od, bore, z0, z0 + w)


def internal_clearance(key: str) -> float:
    """The worst radial internal clearance of bearing ``key`` (mm)."""
    return float(max(bearing_dims(key).get("radial_clearance_mm") or (0.0,)))


def internal_tilt(key: str) -> float:
    """How far bearing ``key``'s rings misalign freely at its worst radial clearance
    (radians): the axial play that clearance allows over the pitch diameter,
    ``sqrt(4 (f_i + f_o - 1) D_w G_r) / d_m`` (the ball ``D_w`` the catalog's ``ball_d``, else
    :data:`BALL_FRACTION` of the section)."""
    import math

    d = bearing_dims(key)
    od, bore = float(d["od"]), float(d["id"])
    ball = float(d.get("ball_d") or BALL_FRACTION * (od - bore))
    fi, fo = OSCULATION
    axial = math.sqrt(4.0 * (fi + fo - 1.0) * ball * internal_clearance(key))
    return axial / ((od + bore) / 2)


@dataclass(frozen=True)
class BearingSeat:
    """What the two ball pivots share: the bearing, its seat in the link and its fit on the
    shaft. ``seat_fit``: the link's bore over the bearing's OD (bonded); ``shaft_fit``: the
    worst diametral gap between the bearing's bore and the shaft."""

    bearing_key: str
    shaft_fit: float
    seat_fit: float = 0.2

    @property
    def od(self) -> float:
        return float(bearing_dims(self.bearing_key)["od"])

    @property
    def bore(self) -> float:
        return float(bearing_dims(self.bearing_key)["id"])

    @property
    def width(self) -> float:
        return float(bearing_dims(self.bearing_key)["w"])

    def seat_d(self) -> float:
        return self.od + self.seat_fit

    def clearance(self) -> float:
        """The joint's diametral play: the bore on the shaft and the balls' clearance."""
        return round(self.shaft_fit + internal_clearance(self.bearing_key), 4)

    def tilt_clearance(self) -> float:
        """The diametral clearance over the bearing's width that tilts a link as far as the
        bore rocking on the shaft plus the rings' free misalignment (:func:`internal_tilt`)."""
        import math

        rock = math.atan2(self.shaft_fit, self.width) + internal_tilt(self.bearing_key)
        return round(self.width * math.tan(rock), 4)

    def check(self, ctx: Context, shaft_d: float, what: str) -> float:
        """The seat radius, or :class:`ConstructionError` when the shaft isn't the bearing's
        bore or the link can't hold the seat."""
        p = ctx.params
        if abs(shaft_d - self.bore) > EPS:
            raise ConstructionError(f"{what}: a {self.bore:g} mm bore bearing on a "
                                    f"{shaft_d:g} mm shaft")
        r = self.seat_d() / 2
        if r + p.min_wall > p.link_radius + EPS:
            need = round(r + p.min_wall, 2)
            raise ConstructionError(
                f"{what}: a {self.od:g} mm bearing's seat leaves less than {p.min_wall:g} mm "
                f"of a {p.link_radius:g} mm-radius link around it (link_radius {need:g} would)",
                changes=[("link_radius", p.link_radius, need)], lever="link_radius")
        if self.width > ctx.pitch + EPS:
            raise ConstructionError(f"{what}: a {self.width:g} mm bearing is wider than the "
                                    f"{ctx.pitch:g} mm links it seats in")
        return r

    def seat_bearings(self, build: Build, group: AxleGroup, out: Realized, riders,
                      stem: str) -> None:
        """A bearing bonded into each rider link's bore, centred in its layer; the bore cut."""
        xy = xy_of(build, group)
        for m in sorted(riders):
            k = build.layers[m]
            z0, z1 = build.z(k)
            if self.width > z1 - z0 + EPS:
                raise ConstructionError(f"{group.name}: {m} is {z1 - z0:g} mm thick, under the "
                                        f"{self.width:g} mm bearing it seats")
            l0, l1 = link_z(build, m)               # centred in the link
            zb = (l0 + l1 - self.width) / 2
            out.bodies.append(hardware(f"{stem}_bearing_{m}", bearing_solid(
                self.bearing_key, xy, zb), m, fab="purchased", bom_key=self.bearing_key,
                color=BEARING_COLOR))
            out.cut(m, Cut(xy, self.seat_d()))
            out.extras.append(BomLine("epoxy_2part", GLUE_PER_BEARING,
                                      f"{group.name}: bearing into {m} (slow epoxy on the "
                                      "outer ring: CA crazes acrylic)"))

    def terms(self, base: ErrorTerms, key: str) -> ErrorTerms:
        """``base`` (the shaft's end float) with the bearing's radial play."""
        import math

        return replace(base, radial_play_mm=self.clearance(),
                       tilt_deg=round(math.degrees(math.atan2(self.tilt_clearance(),
                                                              self.width)), 3),
                       source=f"{key}: {self.bearing_key} on the shaft",
                       notes=(f"{self.shaft_fit:g} mm bore-on-shaft gap (assumed, unmeasured) "
                              f"+ {internal_clearance(self.bearing_key) * 1000:g} um internal "
                              "clearance (ISO 5753-1 CN, worst)",))

    def assembly(self, group: AxleGroup, view) -> list:
        """On the bench, before each layer: the bearings of that layer's links bonded in
        (one bench step per layer, whatever pins they ride; the link and its bearing then go
        on together in the layer's step)."""
        from spiderpig.construction.assembly import STACK, Op, whole

        stem = stem_of(group)
        by_layer: dict[int, list[str]] = {}
        for n in view.z:
            if n.startswith(f"{stem}_bearing_"):
                by_layer.setdefault(view.slot(view.z[n][0]), []).append(n)
        return [Op(STACK, (k, 0, 0), whole(*sorted(names)), f"Bearings for layer {k}",
                   "Bond a bearing into each of these links' bores, flush and square (slow "
                   "epoxy on the outer ring only, none on the shields or the bore; centre it "
                   "on a pin while it cures), and let it cure before the link goes on.",
                   "layer", sub=True)
                for k, names in sorted(by_layer.items())]


@dataclass(frozen=True)
class BallAxle(ChicagoAxle):
    """MR84ZZ ball bearings in the rider links of a Chicago-screw pin (pins only)."""

    key: str = "ball"
    label: str = ("MR84ZZ ball bearings bonded into the links, on an M3 Chicago screw's 4 mm "
                  "barrel; printed rings and head spacers; host link bonded (pins only)")
    seat: BearingSeat = field(default_factory=lambda: BearingSeat("bearing_mr84zz", 0.05))
    cad_keys: ClassVar[tuple[str, ...]] = ("bearing_mr84zz",)

    def dims(self, ctx: Context, pillar: bool) -> AxleDims:
        d = super().dims(ctx, pillar)
        return replace(d, seat=self.seat.check(ctx, self.shaft.d, self.key))

    def error_terms(self, ctx: Context) -> ErrorTerms:
        return self.seat.terms(super().error_terms(ctx), self.key)

    def assembly(self, group: AxleGroup, view) -> list:
        return [*self.seat.assembly(group, view), *super().assembly(group, view)]

    def realize(self, group: AxleGroup, build: Build) -> Realized:
        """As the Chicago pin (rings, the screw and its spacers, the host bonded), but each
        rider seats a bearing: its bore is the bearing's seat, and its tilt and play are the
        bearing's on the barrel."""
        out = Realized()
        col = Column.of(build, group)
        xy, host, stem = xy_of(build, group), host_of(build, group), stem_of(group)
        for k in col.between:
            role, r = col.roles[k]
            if role == "neck":
                raise ConstructionError(f"{group.name}: a barrel can't neck down (layer {k})")
            z0, z1 = ring_z(build, k)
            out.bodies.append(hardware(f"{stem}_ring{k}", ring(xy, 2 * r, self.hole(), z0, z1),
                                       host, fab="printed", color=SLEEVE_COLOR))
        f = self.shaft.realize(build, group, col, out)
        gap_washers(build, group, col, out, self.shaft.d, host, stem)
        riders = [m for m in group.axis.members if m != host]
        self.seat.seat_bearings(build, group, out, riders, stem)
        out.notes["wobble"] = {group.name: column_wobble(
            build, group, col,
            clearance=lambda m: 0.0 if m == host else self.seat.tilt_clearance(),
            length=self.seat.width, play=f.play + _hw_tol(),
            play_basis=f"barrel length less stack, washer and shims ({f.length:g} mm barrel)",
            section=chicago_section(self.shaft),
            bearing_len=self.seat.width)}
        return out


@dataclass(frozen=True)
class BallPillar(StandoffAxle):
    """MR106ZZ ball bearings in the links on a 6 mm standoff pillar (pillars only)."""

    key: str = "ball_pillar"
    label: str = ("MR106ZZ ball bearings bonded into the links on a 6 mm standoff column "
                  "(goBILDA stock, else one steel shaft made to length); printed rings, button "
                  "heads through both frame plates (pillars only; link_radius 6.6 mm)")
    roles: ClassVar[tuple[str, ...]] = ("pillar",)
    seat: BearingSeat = field(default_factory=lambda: BearingSeat("bearing_mr106zz", 0.03))
    cad_keys: ClassVar[tuple[str, ...]] = ("bearing_mr106zz",)

    def dims(self, ctx: Context, pillar: bool) -> AxleDims:
        d = super().dims(ctx, pillar)
        return replace(d, seat=self.seat.check(ctx, self.od, self.key))

    def error_terms(self, ctx: Context) -> ErrorTerms:
        return self.seat.terms(super().error_terms(ctx), self.key)

    def assembly(self, group: AxleGroup, view) -> list:
        return [*self.seat.assembly(group, view), *super().assembly(group, view)]

    def realize(self, group: AxleGroup, build: Build) -> Realized:
        """The standoff column as built (its rings, shims, end screws and plate holes), then a
        bearing in every link on it: the link's running hole becomes the bearing's seat and
        the wobble note the bearing's."""
        out = super().realize(group, build)
        xy, stem = xy_of(build, group), stem_of(group)
        members = set(group.axis.members)
        for m in members:     # the plain running hole the column cut: the seat replaces it
            out.cuts[m] = [c for c in out.cuts.get(m, []) if not (
                isinstance(c, Cut) and abs(c.xy[0] - xy[0]) < 1e-9
                and abs(c.xy[1] - xy[1]) < 1e-9)]
        self.seat.seat_bearings(build, group, out, members, stem)
        note = out.notes["wobble"][group.name]
        out.notes["wobble"][group.name] = _rebuilt(build, group, note,
                                                   self.seat.tilt_clearance(),
                                                   self.seat.width, self.section())
        return out


def _rebuilt(build: Build, group: AxleGroup, note: dict, clearance: float, width: float,
             section) -> dict:
    """The standoff's wobble note recomputed for links on bearings (the bearing's clearance
    over its width), keeping the column's measured play, supports and shims."""
    col = Column.of(build, group)
    keep = {k: note[k] for k in ("supports", "segments_mm", "shims_mm") if k in note}
    out = column_wobble(build, group, col, clearance=clearance, length=width,
                        play=note["play_mm"], play_basis=note["play_basis"], section=section,
                        bearing_len=width)
    return {**out, **keep}


__all__ = ["STEEL", "BallAxle", "BallPillar", "BearingSeat", "bearing_solid"]
