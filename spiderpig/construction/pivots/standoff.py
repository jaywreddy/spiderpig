"""``standoff``: round 6 mm standoffs as pillars; a long column is one piece made to length.

A **pillar** (frame pivot; pins are not built this way) is a column from the outer frame
plate's inner face to the inner plate's, screwed through each plate from outside (no glue):
a button head and a washer (3.0 mm together, one layer outside the plate; the inner one's
stands in the space between the robot's inner plates, clear of the chassis, and the deck
lowers past it), tightened to at most ``tighten_nm``. Both ends fixed in the plates: a
beam. A pillar a link's sweep stops short of one plate is a cantilever from the other, its
free end over its last link closed by the same screw and washer in the next layer (a
clearance gap between that link and that layer is claimed and takes a printed gap ring, as
every gap of the column does, so the link can't slide: W8, 2026-10-07). The
links turn on the standoff's 6 mm OD (a running fit, ``Params.running_fit``); every other
layer between the plates holds a printed spacer **ring** (an 8.5 mm laser-cut ring is
under both services' smallest part), so every link has a face on both sides. Where an
aluminium link made a layer thicker than the sheet and a stock standoff stands longer than
its stack, the rings in those layers grow to take the air up
(:meth:`StandoffAxle.ring_fill`), so the links keep their play.

**The column** (:meth:`StandoffAxle.column_axle`):

* where one stock length fills it: a goBILDA 1501 round aluminium standoff (6 mm OD,
  M4 x 0.7 female both ends; :data:`hardware.crank_catalog.GOBILDA_LENGTHS`: not every mm,
  in 3 mm layers 12, 18, 24, 27, 30, 36, 42, 48, 54 and 60 mm), an M4 button head and a
  DIN 125 washer (9 mm) through each plate;
* else (longer than 60 mm, or a length goBILDA lacks): **one** 6 mm round 1018 steel
  standoff made to the column's length, tapped M3 both ends (MISUMI NETRF6, 0.1 mm steps,
  +-0.1; :meth:`StandoffAxle.one_piece`), an M3 button head and a DIN 9021 washer through
  each plate. Never spliced. (The default Strider double's four pillars are NETRF6-62.4.)
  A column no longer than one goBILDA standoff that no stock length fills is refused unless
  goBILDA standoffs spliced at link-free layers would have filled it
  (:meth:`StandoffAxle._spliceable`): the rule the spliced pillar left behind, kept so the
  plans stay where they were (a short odd column, 15 mm, stays refused).

Why one piece: a splice is a joint mid-span, and rated at the plan's own z (its clearance
gaps included: a stack is up to twice its layers x pitch) the hand-tight splices opened at
jam SF 1.43 on the Strider double and 0.51 on the quad; even the bench-built 1.0 N·m splice
can't reach SF 2 on the quad's 128 mm column, wherever the splices go. One piece rates as a
beam: SF 10.4 and 5.1 there. (The spliced pillars were removed on 2026-10-07:
:data:`config.REMOVED_CONSTRUCTIONS`.)

**Strength** (:mod:`spiderpig.strength`, :func:`construction.wobble.stresses`): the section
is the standoff taken as a tube bored to its tap drill (as if tapped through: conservative
for a part tapped at its ends): goBILDA's 6 x 3.3 mm of 6061-T6 at 240 MPa, the steel
shaft's 6 x 2.5 mm of 1018 at 220 MPa. The column is a beam **per bay** between its
supports, the frame plates' faces (``supports`` in the note), at the plan's own z.
``docs/audit/STRENGTH.md`` has it per design.

**Shims.** A column may be up to ``max_shims`` shorter than its gap where its upper end is
under a spacer layer: steel shims sit between its end and the face over it, so the column
stays one contiguous stack (in the clearance gap under the face when there is one, its
gap ring trimmed to make room; else in the spacer layer, whose ring they shorten), and the
end screw is chosen for the plate plus the shims. They come in whole 1 mm and
:data:`SHIM_STEP` (0.5 mm) steps, bought as DIN 433 washers (:data:`hardware.bom.SHIM_AS`:
two make 1 mm).

Assembly, bottom up (:meth:`StandoffAxle.assembly`; :data:`construction.assembly.ROBOT_ORDER`
has the whole robot's order): the
outer plate down; per pillar, its column onto the plate's hole with the button head and
washer from outside (threadlocker, to ``tighten_nm`` while the column is still bare to
hold), then the links and printed rings in layer order (the plan says which); the inner
plate last, as part of its unit (the servo, horn and hub plate on it), its screws from the
servo bay, each to ``tighten_nm``.
"""

from __future__ import annotations

import functools
import itertools
from dataclasses import dataclass, replace
from typing import ClassVar

from spiderpig.construction.axle import AxleDims, AxleGroup, End
from spiderpig.construction.base import (
    FRAME_INNER,
    FRAME_OUTER,
    Build,
    ConstructionError,
    Context,
    Realized,
    hardware,
)
from spiderpig.construction.pivots.common import (
    EPS,
    SLEEVE_COLOR,
    STEEL,
    Column,
    bored,
    column_air,
    gap_washers,
    ring_z,
    xy_of,
)
from spiderpig.construction.wobble import Section, column_wobble
from spiderpig.hardware.bom import SHIM_STEP, BomLine
from spiderpig.hardware.catalog import get
from spiderpig.shapes import Cut, disc, ring, union
from spiderpig.stack import Unbuildable

ALU = "#c8ccd0"
COLUMN_TOL = 0.25    # the column within this of its gap (half the step)


@dataclass(frozen=True)
class StandoffAxle:
    """A 6 mm round standoff column as a pillar: one stock goBILDA 1501 standoff, else one
    steel standoff made to length (:meth:`one_piece`)."""

    key: str = "standoff"
    label: str = ("6 mm round aluminium standoffs (goBILDA M4) where one stock length fills the "
                  "column, else one 6 mm steel shaft made to its length (tapped M3), printed "
                  "rings, button heads through both frame plates")
    roles: ClassVar[tuple[str, ...]] = ("pillar",)
    od: float = 6.0
    id_: float = 3.3                 # the strength check's bore (M4 tap drill)
    yield_mpa: float = 240.0         # 6061-T6
    ring_fit: float = 0.35           # a ring's bore over the standoff
    end_hole: float = 4.5            # the frame plates' hole for the M4 screw (ISO 273 medium)
    stud_hole: float = 4.3           # the end shims' hole
    min_segment: float = 12.0        # thread for the end screws in each end
    min_engage: float = 4.0          # M4 thread in a standoff's end
    tighten_nm: float = 0.8          # the end screws
    shim_key: str = "shim_din988_4x8"   # the end shims (bought as DIN 433 washers)
    set_play: float = 0.1            # axial play of the column (plates touch; assumed)
    washer_key: str = "m4_washer"
    lock_key: str | None = "threadlocker_222"
    lock_per_screw: float = 0.01
    size: str = "M4"                 # "M3": the one-piece shaft's M3 ends (one_piece)
    stock: str = ""                  # "": goBILDA M4 stock lengths; "shaft": a 6 mm round
    #                                  steel standoff tapped M3 both ends, made to length
    #                                  (MISUMI NETRF6, one_piece)

    @property
    def screw_d(self) -> float:
        return 3.0 if self.size == "M3" else 4.0

    @property
    def thread_max(self) -> float:
        """The deepest thread a standoff's end has (goBILDA M4: 8; the shaft's M3 taps, 2 x M
        deep: 6)."""
        return 6.0 if self.size == "M3" else 8.0

    def segment_key(self, length: float) -> str:
        if self.stock == "shaft":
            from spiderpig.hardware.crank_catalog import pillar_shaft

            return pillar_shaft(length)
        from spiderpig.hardware.crank_catalog import gobilda_1501

        return gobilda_1501(length)

    # -- catalog ----------------------------------------------------------------------

    def lengths(self) -> tuple[float, ...]:
        from spiderpig.hardware.crank_catalog import GOBILDA_LENGTHS, PILLAR_SHAFT_LENGTHS

        if self.stock == "shaft":
            return PILLAR_SHAFT_LENGTHS
        return GOBILDA_LENGTHS

    @property
    def max_segment(self) -> float:
        return max(self.lengths())

    def one_piece(self) -> StandoffAxle:
        """What a column no single stock standoff fills becomes: one 6 mm round steel
        standoff made to the column's length (MISUMI NETRF6, 0.1 mm steps;
        :data:`hardware.crank_catalog.PILLAR_SHAFT_LENGTHS`), tapped M3 both ends, an M3
        button head and DIN 9021 washer through each plate, never spliced. (A splice is a
        joint mid-span: at the plan's own z, its clearance gaps included, the hand-tight
        splices of the Strider double and quad opened at jam SF 1.43 and 0.51.)"""
        from spiderpig.hardware.crank_catalog import PILLAR_SHAFT_ID, PILLAR_SHAFT_YIELD

        return replace(self, stock="shaft", size="M3",
                       id_=PILLAR_SHAFT_ID, yield_mpa=PILLAR_SHAFT_YIELD, end_hole=3.4,
                       stud_hole=3.2, min_engage=3.0, min_segment=12.0,
                       washer_key="m3_washer_9021", shim_key="shim_din988_3x6")

    def washer(self) -> tuple[float, float, float]:
        w = get(self.washer_key).dims
        return float(w["od"]), float(w["id"]), float(w["t"])

    def end_screw(self, pitch: float, plate: bool = True
                  ) -> tuple[str, float, float, float] | None:
        """(key, length, head diameter, head height) of the button head (M4, or M3 for an
        M3 column) through a frame plate (``plate``; else straight into the column's free
        end, over the last link) and its washer into the standoff's end: the most thread up
        to its depth (taken as the shortest standoff's)."""
        from spiderpig.hardware.crank_catalog import M4_BHCS_LENGTHS, m4_bhcs
        from spiderpig.hardware.fasteners import SCREWS

        _, _, wt = self.washer()
        depth = min(self.thread_max, self.min_segment / 2)
        best = None
        lengths = SCREWS["bhcs", "3"].lengths if self.size == "M3" else M4_BHCS_LENGTHS
        for L in lengths:
            e = L - (pitch if plate else 0.0) - wt
            if self.min_engage - EPS <= e <= depth + EPS and (best is None or e > best[1]):
                best = (L, e)
        if best is None:
            return None
        key = SCREWS["bhcs", "3"].key(best[0]) if self.size == "M3" else m4_bhcs(best[0])
        d = get(key).dims
        return key, best[0], float(d["head_d"]), float(d["head_h"])

    # -- dimensions and rules ---------------------------------------------------------------

    def dims(self, ctx: Context, pillar: bool) -> AxleDims:
        p = ctx.params
        if not pillar:
            raise ConstructionError("a standoff is a pillar only: a link pin has no plate to "
                                    "screw its ends to")
        if p.hole(self.od) / 2 + p.min_wall > p.link_radius + EPS:
            raise ConstructionError(f"a {self.od:g} mm standoff's hole leaves less than "
                                    f"{p.min_wall} mm of link around it (link radius "
                                    f"{p.link_radius})")
        if self.end_hole / 2 + p.min_wall > p.frame_radius + EPS:
            raise ConstructionError("an M4 pillar screw doesn't fit the frame plate arms")
        ring_min = (self.od + self.ring_fit) / 2 + p.min_wall   # the narrowest ring to cut
        screw = self.end_screw(ctx.sheet_t("frame"))
        if screw is None or self.end_screw(ctx.pitch, False) is None:
            raise ConstructionError(f"no stock M4 screw fits a standoff pillar's ends in "
                                    f"{ctx.pitch:g} mm layers")
        w_od, _, w_t = self.washer()
        if screw[3] + w_t > ctx.pitch + EPS:
            from spiderpig.hardware.catalog import sheet_thickness

            need = screw[3] + w_t
            nominal = sheet_thickness(ctx.config.sheet) if ctx.config is not None else None
            after = nominal if nominal is not None and nominal >= need - EPS else need
            raise ConstructionError(
                f"an M4 button head and washer ({need:g} mm) don't fit a {ctx.pitch:g} mm "
                f"layer outside the plate; the layer pitch is the sheet's thickness: "
                f"materials.thickness_mm {ctx.pitch:g} -> {after:g}, or a thicker sheet",
                changes=(("thickness_mm", ctx.pitch, after),),
                lever=f"a standoff pillar's end screw needs layers of at least {need:g} mm",
                numbers={"pitch_mm": ctx.pitch, "least_pitch_mm": need})
        head = max(w_od, screw[2]) / 2
        from spiderpig.materials import washer_od

        return AxleDims(axle=self.od / 2, spacer=max(p.spacer_d / 2, ring_min), head=head,
                        neck=ring_min, washer=washer_od(self.od) / 2)

    def ends(self, d: AxleDims, pillar: bool, anchored: tuple[bool, bool], n_layers: int,
             pitch: float, span: float | None = None
             ) -> tuple[tuple[End, ...], tuple[End, ...]]:
        """A button head and washer outside each frame plate it reaches (over the inner
        plate: in the chassis' space between the robot's two inner plates), or over its last
        link at a free end (a cantilever from the other plate)."""
        return (("head", d.head),), (("head", d.head),)

    @staticmethod
    def faces(links, top: int, anchored: tuple[bool, bool]) -> tuple[int, int]:
        """The layers bounding the column's standoff: the frame plates (it ends on their
        inner faces, screwed through each), else the end layer over its last link (its
        screw head's)."""
        return (0 if anchored[0] else min(links) - 1,
                top if anchored[1] else max(links) + 1)

    max_long: float = 0.8            # a standoff may be this much longer than its gap (the
    #                                  column then holds the plates that far apart: axial play)
    max_short: float = 0.1           # or this much shorter (the plates' and rings' tolerance)
    max_shims: float = 2.0           # or shorter by shims, where its upper end's layer is a
    #                                  spacer (its ring shortened for them), not a link

    def column_axle(self, links, top: int, pitch: float, lo: int = 0, layout=None,
                    air=None) -> StandoffAxle | None:
        """The construction the column between faces ``lo`` and ``top`` is built with: this
        one where one stock goBILDA standoff fills it, else :meth:`one_piece` where a shaft
        length does (and the column is longer than one goBILDA standoff, or
        :meth:`_spliceable`); ``None``: neither. ``layout``: at its z (a plan's, its gaps and
        thicker plates included), else every layer ``pitch``; ``air(a, b)``: what the z leaves
        free around the column's own parts in layers ``a``..``b`` (its standoff spans its
        parts' stack, which closes it up)."""
        zs = None
        if layout is not None and (layout.thick or layout.gaps):
            zs = tuple((round(layout.z(k)[0], 6), round(layout.z(k)[1], 6),
                        round(air(k, k) if air is not None else 0.0, 6))
                       for k in range(lo, top + 1))
        return _column_axle(self, frozenset(links), top, pitch, lo, zs)

    def _column_axle(self, links: frozenset[int], top: int, pitch: float, lo: int,
                     zs: tuple | None) -> StandoffAxle | None:
        if self._fits(links, top, pitch, lo, zs):
            return self
        if not self._spliceable(links, top, pitch, lo, zs):
            z_lo = zs[0][1] if zs is not None else (lo + 1) * pitch
            z_hi = zs[top - lo][0] if zs is not None else top * pitch
            air = sum(z[2] for z in zs[1:top - lo]) if zs is not None else 0.0
            if z_hi - z_lo - air <= self.max_segment + self.max_long + EPS:
                return None
        shaft = self.one_piece()
        return shaft if shaft._fits(links, top, pitch, lo, zs) else None

    def segment(self, gap: float, shims: bool = False) -> float | None:
        """The stock length for a ``gap`` mm between two faces (``max_short`` under it to
        ``max_long`` over, the nearest; with ``shims``, up to ``max_shims`` under it, steel
        shims (DIN 433 washers) taking the rest up at its upper end), ``None`` when none is."""
        if self.size == "M3":
            # (the shaft: its lengths, end shims in SHIM_STEP steps only (DIN 433 washers,
            # two to a 1 mm shim), the column within COLUMN_TOL of its gap; goBILDA M4 keeps
            # its rule below, its take-up rounded to the step in shims(): at most half a step
            # over, inside max_long)
            def resid(L: float) -> float:
                d = gap - L
                return abs(d - round(d / SHIM_STEP) * SHIM_STEP) if d > EPS else abs(d)
            hi = self.max_shims if shims else self.max_short
            ok = [L for L in self.lengths() if self.min_segment - EPS <= L
                  and gap - hi - EPS <= L <= gap + 0.1 + EPS
                  and resid(L) <= COLUMN_TOL + EPS]
            return min(ok, key=lambda L: (round(resid(L), 3), abs(L - gap), L)) if ok else None
        short = self.max_shims if shims else self.max_short
        ok = [L for L in self.lengths() if self.min_segment - EPS <= L
              and gap - short - EPS <= L <= gap + self.max_long + EPS]
        return min(ok, key=lambda L: (abs(L - gap), L)) if ok else None

    @staticmethod
    def _gap(a: int, b: int, pitch: float, lo: int, zs: tuple | None) -> float:
        """Between faces ``a`` and ``b``: the z (each layer's from ``lo``, else ``pitch``),
        less the air its parts close."""
        if zs is None:
            return b * pitch - (a + 1) * pitch
        return zs[b - lo][0] - zs[a - lo][1] - sum(zs[k - lo][2] for k in range(a + 1, b))

    def _fits(self, links, top: int, pitch: float, lo: int, zs: tuple | None) -> bool:
        """One standoff of this construction's lengths fills the column between faces
        ``lo`` and ``top``."""
        return self.segment(self._gap(lo, top, pitch, lo, zs),
                            top - 1 > lo and top - 1 not in links) is not None

    def _spliceable(self, links, top: int, pitch: float, lo: int, zs: tuple | None) -> bool:
        """Whether up to three splices at link-free layers between the faces (the removed
        spliced pillar's rule: each segment a stock goBILDA length) would fill the column:
        kept as the one-piece column's rule for a short column, so the planner refuses what
        it always refused."""
        cand = [k for k in range(lo + 2, top - 1) if k not in links]

        def fits(sp) -> bool:
            faces = [lo, *sp, top]
            return all(self.segment(self._gap(a, b, pitch, lo, zs),
                                    b - 1 > a and b - 1 not in links) is not None
                       for a, b in itertools.pairwise(faces))

        return any(fits(sp) for n in range(1, 4) for sp in itertools.combinations(cand, n))

    def column(self, pillar: bool, links: list[int], top: int,
               anchored: tuple[bool, bool], pitch: float, layout=None, air=None) -> None:
        """The planner's rule: a standoff must fill the column (at the plan's own z when
        ``layout`` is final)."""
        lo, hi = self.faces(links, top, anchored)
        final = layout if layout is not None and layout.final else None
        if pillar and self.column_axle(links, hi, pitch, lo, final, air) is None:
            raise Unbuildable(
                f"no stock standoff ({self.min_segment:g}-{self.max_segment:g} mm) nor a "
                f"shaft made to length fills its {(hi - lo - 1) * pitch:g} mm column")

    # -- strength ---------------------------------------------------------------------------

    def section(self) -> Section:
        if self.stock == "shaft":
            return Section.tube(self.od, self.id_, self.yield_mpa,
                                name=f"6 mm 1018 steel standoff (as a 6 x {self.id_:g} tube)")
        return Section.tube(self.od, self.id_, self.yield_mpa,
                            name=f"6 mm Al standoff (6061, as a 6 x {self.id_:g} tube)")

    def shim_od(self) -> float:
        return float(get(self.shim_key).dims["od"])

    def shims(self, t: float) -> list[float]:
        """The shims that stack to an end's take-up ``t``, thickest first: whole 1 mm shims
        and :data:`SHIM_STEP` (bought as DIN 433 washers, :data:`hardware.bom.SHIM_AS`), the
        thickness rounded to the step."""
        from spiderpig.hardware.bom import stack

        return stack(t, (1.0, SHIM_STEP), round_to=SHIM_STEP)[0]

    # -- parts ----------------------------------------------------------------------------

    def ring_fill(self, build: Build, group: AxleGroup, col, faces: list[int]
                  ) -> dict[int, float]:
        """Per ring layer, how much taller than the default sheet its printed ring is made
        (on top of :func:`ring_z`), where a stock standoff stands longer than its gap: a
        layer an aluminium plate elsewhere made thicker leaves the column air
        (:func:`column_air`), the plates then held apart by the standoff, and that much
        axial play for the links (``klann_lego``'s pillars: 0.70 mm, 5.0 deg of tilt, before
        2026-10-05). Each ring takes up to its layer's air, lowest first, until the column's
        stack is the standoff's length; a link's layer keeps its air (the link is its own
        sheet)."""
        fill: dict[int, float] = {}
        pitch = build.ctx.pitch
        for a, b in itertools.pairwise(faces):
            z0, z1 = build.z(a)[1], build.z(b)[0]
            span = z1 - z0 - column_air(build, group, a + 1, b - 1)
            length = self.segment(span, b - 1 > a and b - 1 not in col.links)
            over = 0.0 if length is None else length - span
            for k in range(a + 1, b):
                if over <= EPS:
                    break
                if k in col.links or k not in col.between or col.roles[k][0] == "neck":
                    continue
                lo, hi = ring_z(build, k)
                room = build.z(k)[1] - hi
                if room > EPS and hi - lo >= pitch - EPS:
                    fill[k] = min(room, over)
                    over -= fill[k]
        return fill

    def assembly(self, group: AxleGroup, view) -> list:
        """How a pillar goes on (:mod:`construction.assembly`): its column (the standoff,
        the button head and washer from outside) on the bare outer plate first; its rings,
        spacers and shims with the layers they sit in; its inner screw and washer from the
        servo bay once the inner plate is on (the join)."""
        import re

        from spiderpig.construction.assembly import JOIN, STACK, Op, whole
        from spiderpig.construction.pivots.common import stem_of

        stem = re.escape(stem_of(group))
        bottom, top = view.layers[0][0], view.layers[view.top][1]
        column = view.named(rf"{stem}_standoff\d+")
        ends = view.named(rf"{stem}_(screw|washer)\d+")
        column += [n for n in ends if view.z[n][0] < bottom - 1e-3]
        inner = [n for n in ends if n not in column and view.z[n][1] > top + 1e-3]
        ops = []
        if column:
            ops.append(Op(STACK, (-1, 1), whole(*column), "",
                          "Screw each pillar's standoff column to the outer plate: its button "
                          "head and washer from outside, threadlocker, to "
                          f"{self.tighten_nm:g} N·m while the column is bare to hold.",
                          "columns"))
        if inner:
            ops.append(Op(JOIN, (1,), whole(*inner), "",
                          "Each pillar's inner screw and washer from the servo bay (a "
                          f"ball-end key), threadlocker, to {self.tighten_nm:g} N·m.",
                          "pillar_screws"))
        ops.extend(Op(STACK, (view.slot(view.z[n][0]), 3), whole(n), "",
                      "The take-up shims on their column's top.", "layer")
                   for n in view.named(rf"{stem}_shims\d+"))
        return ops

    def realize(self, group: AxleGroup, build: Build) -> Realized:
        col = Column.of(build, group)
        top = build.top
        anchored = (0 in col.anchors, top in col.anchors)
        lo_face, hi_face = self.faces(col.links, top, anchored)
        axle = self.column_axle(set(col.links), hi_face, build.ctx.pitch, lo_face,
                                build.plan.layout, lambda a, b: column_air(build, group, a, b))
        if axle is None:
            raise ConstructionError(f"{group.name}: no stock standoff nor a shaft made to "
                                    "length fills its column")
        return axle._realize(group, build, col, anchored, (lo_face, hi_face))

    def _realize(self, group: AxleGroup, build: Build, col: Column,
                 anchored: tuple[bool, bool], faces: tuple[int, int]) -> Realized:
        """The column between ``faces``, built with this construction (the one
        :meth:`column_axle` picked)."""
        out = Realized()
        p = build.ctx.params
        pitch = build.ctx.pitch
        xy = xy_of(build, group)
        host = build.plan.topo.frame_bodies[0]
        stem = group.name.replace(":", "_")
        top = build.top
        a, b = faces
        fill = self.ring_fill(build, group, col, [a, b])
        for k in col.between:
            role, r = col.roles[k]
            if role == "neck":
                raise ConstructionError(f"{group.name}: a standoff can't neck down (layer {k})")
            z0, z1 = ring_z(build, k)
            z1 += fill.get(k, 0.0)
            out.bodies.append(hardware(f"{stem}_ring{k}",
                                       ring(xy, 2 * r, self.od + self.ring_fit, z0, z1), host,
                                       fab="printed", color=SLEEVE_COLOR))
        shimmed: dict[int, float] = {}      # face -> the shims' thickness under it
        trim: dict[int, float] = {}         # gap layer -> the height the shims take of it
        long = 0.0          # how far the stock standoff (and take-up) is off its gap
        z0, z1 = build.z(a)[1], build.z(b)[0]
        span = z1 - z0 - column_air(build, group, a + 1, b - 1)
        length = self.segment(span, b - 1 > a and b - 1 not in col.links)
        if length is None:
            raise ConstructionError(f"{group.name}: no stock standoff fits {span:.2f} mm")
        long += max(0.0, length - span - sum(fill.get(k, 0.0) for k in range(a + 1, b)))
        short = span - length
        if short > self.max_short + EPS:
            # steel shims between the standoff's upper end and the face over it (the column
            # stays one contiguous stack up to the face): in the clearance gap under the face
            # where there is one (its gap ring trimmed to make room), else, or for what the
            # gap can't take, in the spacer layer under it, whose ring they shorten
            sh = self.shims(round(short, 1))
            # what the stock length and the rounded take-up leave off the gap (under half
            # a step, either way) is the column's play, not lost
            long += abs(short - sum(sh))
            if sh:              # (under half the thin step, 0.25 mm: left as play)
                t = sum(sh)
                zs0 = z1 - t
                g = build.plan.gaps.get(b - 1, 0.0) if b - 1 in col.washers else 0.0
                if g > 0:
                    trim[b - 1] = min(t, g)
                out.bodies.append(hardware(f"{stem}_shims{b}", ring(xy, self.shim_od(),
                                                                    self.stud_hole, zs0, z1),
                                           host, fab="purchased", bom_key=self.shim_key,
                                           color=STEEL))
                if len(sh) > 1:
                    out.extras.append(BomLine(self.shim_key, len(sh) - 1,
                                              f"{group.name}: {t:.1f} mm under layer {b}"))
                sleeve = f"{stem}_ring{b - 1}"
                for i, body in enumerate(out.bodies):
                    if body.name == sleeve:
                        bb = body.part.bounding_box()
                        if zs0 + EPS < bb.max.Z:
                            r = (bb.max.X - bb.min.X) / 2
                            out.bodies[i] = hardware(
                                sleeve, ring(xy, 2 * r, self.od + self.ring_fit, bb.min.Z,
                                             zs0), host, fab="printed", color=SLEEVE_COLOR)
                shimmed[b] = t
                z1 = zs0
        seg = disc(xy, self.od / 2 - 0.01, z0, z1) - disc(xy, 2.0, z0 - 1, z1 + 1)
        out.bodies.append(hardware(f"{stem}_standoff{a}", seg, host, fab="purchased",
                                   bom_key=self.segment_key(length), color=ALU))
        long += gap_washers(build, group, col, out, self.od, host, stem, trim=trim)
        w_od, w_id, w_t = self.washer()
        ends = [(a, -1.0, anchored[0]), (b, 1.0, anchored[1])]
        for k, sign, plate in ends:
            # through the plate (and any shims under it) into the standoff's end
            grip = (build.plan.t(k) if plate else pitch) + shimmed.get(k, 0.0)
            got = self.end_screw(grip, plate)
            if got is None:
                raise ConstructionError(f"{group.name}: no stock M4 screw takes a {grip:.2f} mm "
                                        f"grip at layer {k}")
            key, L, hd, hh = got
            if plate:            # outside the frame plate
                face = build.z(k)[0] if sign < 0 else build.z(k)[1]
            else:                # in the end layer, on the column's free end
                face = build.z(k)[1] if sign < 0 else build.z(k)[0]
            washer = bored(disc(xy, w_od / 2, *sorted((face, face + sign * w_t))), xy, w_id,
                           *sorted((face, face + sign * w_t)))
            bear = face + sign * w_t
            head = disc(xy, hd / 2, *sorted((bear, bear + sign * hh)))
            shank = disc(xy, 0.97 * self.screw_d / 2, *sorted((bear, bear - sign * L)))
            out.bodies += [
                hardware(f"{stem}_washer{k}", washer, host, fab="purchased",
                         bom_key=self.washer_key, color=STEEL),
                hardware(f"{stem}_screw{k}", union([head, shank]), host, fab="purchased",
                         bom_key=key, color=STEEL)]
        if self.lock_key is not None:
            out.extras.append(BomLine(self.lock_key, self.lock_per_screw * len(ends),
                                      f"{group.name} end screws"))
        for m in group.axis.members:
            out.cut(m, Cut(xy, p.hole(self.od)))
        if anchored[0]:
            out.cut(FRAME_OUTER, Cut(xy, self.end_hole))
        if anchored[1]:        # screwed through the inner plate as through the outer
            out.cut(FRAME_INNER, Cut(xy, self.end_hole))
        note = column_wobble(
            build, group, col, clearance=p.running_fit, length=pitch,
            play=self.set_play + long,
            play_basis=(f"plates and rings touching ({self.set_play:g} mm assumed)"
                        + (f", the stock segments and shims {long:.2f} mm off their gaps"
                           if long else "")),
            section=self.section())
        note["supports"] = [k for k, on in ((0, anchored[0]), (top, anchored[1])) if on]
        note["segments_mm"] = [length]
        note["shims_mm"] = {int(k): round(t, 3) for k, t in shimmed.items()}
        out.notes["wobble"] = {group.name: note}
        return out


@functools.lru_cache(maxsize=4096)
def _column_axle(axle: StandoffAxle, links: frozenset[int], top: int, pitch: float,
                 lo: int, zs: tuple | None = None) -> StandoffAxle | None:
    """:meth:`StandoffAxle.column_axle`, remembered (the planner asks per layout)."""
    return axle._column_axle(links, top, pitch, lo, zs)
