"""``standoff``: round 6 mm aluminium standoffs as pillars, spliced at supported layers.

A **pillar** (frame pivot; pins are not built this way) is a column of goBILDA 1501 round
aluminium standoffs (6 mm OD, M4 x 0.7 female both ends; :mod:`hardware.crank_catalog`)
from the outer frame plate's inner face to the inner plate's: an M4 button head and a
DIN 125 washer (9 mm) outside each plate screw into the column's ends (3.0 mm together,
one layer outside the plate, as the printed pillar's head; the inner one's stands in the
space between the robot's inner plates, clear of the chassis and the deck), tightened to at
most ``tighten_nm``; no glue (until 2026-10-04 the top end was glued flush in the inner
plate). Both ends fixed in the plates: a beam. A
pillar a link's sweep stops short of one plate is a cantilever from the other, its free
end over its last link closed by the same screw and washer in the next layer.
The links turn on the standoff's 6 mm OD (a running fit, ``Params.running_fit``); every
other layer between the plates holds a printed spacer **sleeve** (``fill``; an 8.5 mm
laser-cut ring is under both services' smallest part), so every link has a face on both
sides.

**Long gaps are spliced.** A stock standoff is at most ``max_segment`` long (60 mm), so a
column longer than that is a chain of **segments**, each a stock length, joined end to
end through a **splice plate**: in a layer no link of the pillar sits in, a stack of DIN 988
steel shims (4 x 8) to the layer's thickness, clamped between the two segments' end faces
by an M4 set screw threaded half into each (threadlocked), the segments turned together by
hand (``splice_nm``: round standoffs have no flats). Splices go only there (a supported layer: a
plate, never a bare joint mid-span), and :meth:`StandoffAxle.splices` picks the fewest,
each segment at least ``min_segment`` long (thread for the stud and the end screws); a
column no choice fits is :class:`stack.Unbuildable` for the planner (the ``column``
hook of :class:`construction.axle.AxleGroup`). The plate rings are one layer, so a
segment spans whole layers, ``n`` x the layer pitch, and must be a length goBILDA sells
(:data:`hardware.crank_catalog.GOBILDA_LENGTHS`: not every mm; in 3 mm layers 12, 18, 24,
27, 30, 36, 42, 48, 54 and 60 mm, so a 15, 21, 33, 39, 45, 51 or 57 mm column is
spliced too, e.g. 57 = 30 + a 3 mm splice plate + 24).

**Strength** (:mod:`spiderpig.strength`, :func:`construction.wobble.stresses`): the
section is the standoff taken as a 6 x 3.3 mm tube (the M4 tap drill, as if tapped
through: conservative for a part tapped at its ends) of 6061-T6, 240 MPa; the column is
a beam **per bay** between its supports, the frame plates' faces (``supports`` in the
note). A splice plate is a joint, not a support: nothing ties it sideways to the frame,
so it doesn't shorten the span. Its capacity, the moment that starts to open the
clamped end faces (the hand-tight preload x ``(ro^2 + ri^2) / 4 ro`` of the 6 / 4.3 mm
annulus, on steel shims: an honest clamp; until 2026-10-04 it took the end screws' preload
on an acrylic ring, which the end screws don't load and acrylic creeps out of), is
reported per splice (``splices`` in the note) and checked against the bay's moment
there. (A splice layer tied to the frame, a mid frame plate, would make it a support and
halve the span; no design here has a layer free for one.)

Against the printed 6 mm PETG pillar (50 MPa) the section holds about 4.4 x the
moment; the pillar review's numbers (``docs/audit/STRENGTH.md``) have it per design.

Assembly, bottom up: the outer plate down; per pillar, its lowest segment onto the plate's
hole with the M4 screw and washer from outside (finger tight, threadlocker), then the
links and rings in layer order (the plan says which), at each splice the splice plate and
the stud (threadlocker) into the next segment; the inner plate last, its M4 screws from
above, each to ``tighten_nm``.
"""

from __future__ import annotations

import functools
import itertools
from dataclasses import dataclass
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
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.catalog import get
from spiderpig.shapes import Cut, disc, ring, union
from spiderpig.stack import Unbuildable

ALU = "#c8ccd0"


@dataclass(frozen=True)
class StandoffAxle:
    """goBILDA 1501 round standoffs (6 mm OD) as a pillar, spliced at plate rings."""

    key: str = "standoff"
    label: str = ("6 mm round aluminium standoffs (M4, spliced at plate rings), laser-cut "
                  "rings, M4 button heads through both frame plates")
    roles: ClassVar[tuple[str, ...]] = ("pillar",)
    od: float = 6.0
    id_: float = 3.3                 # the strength check's bore (M4 tap drill)
    yield_mpa: float = 240.0         # 6061-T6
    ring_fit: float = 0.35           # a ring's bore over the standoff
    end_hole: float = 4.5            # the frame plates' hole for the M4 screw (ISO 273 medium)
    stud_hole: float = 4.3           # a splice plate's hole for the stud
    min_segment: float = 12.0        # thread for the end screw and the stud in each end
    min_engage: float = 4.0          # M4 thread in a standoff's end
    tighten_nm: float = 0.8          # the end screws
    splice_nm: float = 1.0           # a splice's segments turned together on the stud, each
    #                                  held in soft-jaw pliers (round standoffs have no flats):
    #                                  the "supported splice" of 2026-10-04 (0.4, finger tight,
    #                                  failed the Strider quad's jam); UNVERIFIED
    shim_key: str = "shim_din988_4x8"   # a splice plate: steel shims stacked to the layer
    set_play: float = 0.1            # axial play of the column (plates touch; assumed)
    washer_key: str = "m4_washer"
    lock_key: str | None = "threadlocker_222"
    lock_per_screw: float = 0.01

    # -- catalog ----------------------------------------------------------------------

    def lengths(self) -> tuple[float, ...]:
        from spiderpig.hardware.crank_catalog import GOBILDA_LENGTHS

        return GOBILDA_LENGTHS

    @property
    def max_segment(self) -> float:
        return max(self.lengths())

    def washer(self) -> tuple[float, float, float]:
        w = get(self.washer_key).dims
        return float(w["od"]), float(w["id"]), float(w["t"])

    def end_screw(self, pitch: float, plate: bool = True
                  ) -> tuple[str, float, float, float] | None:
        """(key, length, head diameter, head height) of the M4 button head through a frame
        plate (``plate``; else straight into the column's free end, over the last link)
        and its washer into a segment's end: the most thread up to the segment's depth
        (taken as the shortest segment's)."""
        from spiderpig.hardware.crank_catalog import M4_BHCS_LENGTHS, m4_bhcs

        _, _, wt = self.washer()
        depth = min(8.0, self.min_segment / 2)
        best = None
        for L in M4_BHCS_LENGTHS:
            e = L - (pitch if plate else 0.0) - wt
            if self.min_engage - EPS <= e <= depth + EPS and (best is None or e > best[1]):
                best = (L, e)
        if best is None:
            return None
        d = get(m4_bhcs(best[0])).dims
        return m4_bhcs(best[0]), best[0], float(d["head_d"]), float(d["head_h"])

    def stud(self, pitch: float) -> tuple[str, float] | None:
        """(key, length) of a splice's set screw: one layer of plate and at least
        ``min_engage`` in each segment."""
        from spiderpig.hardware.crank_catalog import M4_SET_LENGTHS, m4_set_screw

        depth = min(8.0, self.min_segment / 2)
        for L in M4_SET_LENGTHS:
            e = (L - pitch) / 2
            if self.min_engage - EPS <= e <= depth + EPS:
                return m4_set_screw(L), L
        return None

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
        if screw is None or self.end_screw(ctx.pitch, False) is None or self.stud(
                ctx.pitch) is None:
            raise ConstructionError(f"no stock M4 screw fits a standoff pillar's ends or splices "
                                    f"in {ctx.pitch:g} mm layers")
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
                        neck=ring_min, fill=True, washer=washer_od(self.od) / 2)

    def ends(self, d: AxleDims, pillar: bool, anchored: tuple[bool, bool], n_layers: int,
             pitch: float, span: float | None = None
             ) -> tuple[tuple[End, ...], tuple[End, ...]]:
        """An M4 screw head and washer outside each frame plate it reaches (over the inner
        plate: in the chassis' space between the robot's two inner plates), or over its last
        link at a free end (a cantilever from the other plate)."""
        return (("head", d.head),), (("head", d.head),)

    @staticmethod
    def faces(links, top: int, anchored: tuple[bool, bool]) -> tuple[int, int]:
        """The layers bounding the column's standoffs: the frame plates (they end on their
        inner faces, screwed through each), else the end layer over its last link (its
        screw head's)."""
        return (0 if anchored[0] else min(links) - 1,
                top if anchored[1] else max(links) + 1)

    max_long: float = 0.8            # a segment may be this much longer than its gap (the
    #                                  column then holds the plates that far apart: axial play)
    max_short: float = 0.1           # or this much shorter (the plates' and rings' tolerance)
    max_shims: float = 2.0           # or shorter by shims, where its upper end's layer is a
    #                                  spacer (its sleeve shortened for them), not a link

    def splices(self, links: list[int] | set[int], top: int, pitch: float,
                lo: int = 0, layout=None, air=None) -> list[int] | None:
        """The splice layers (:meth:`_splices`); ``layout``: at its z (a plan's, its gaps
        and thicker plates included), else every layer ``pitch``; ``air(a, b)``: what the
        z leaves free around the column's own parts in layers ``a``..``b`` (its segments
        span its parts' stack, which closes it up)."""
        zs = None
        if layout is not None and (layout.thick or layout.gaps):
            zs = [(round(layout.z(k)[0], 6), round(layout.z(k)[1], 6),
                   round(air(k, k) if air is not None else 0.0, 6))
                  for k in range(lo, top + 1)]
            zs = tuple(zs)
        got = _splices(self, frozenset(links), top, pitch, lo, zs)
        return None if got is None else list(got)

    def segment(self, gap: float, shims: bool = False) -> float | None:
        """The stock length for a ``gap`` mm between two faces (``max_short`` under it to
        ``max_long`` over, the nearest; with ``shims``, up to ``max_shims`` under it, DIN 988
        shims taking the rest up at its upper end), ``None`` when none is."""
        short = self.max_shims if shims else self.max_short
        ok = [L for L in self.lengths() if self.min_segment - EPS <= L
              and gap - short - EPS <= L <= gap + self.max_long + EPS]
        return min(ok, key=lambda L: (abs(L - gap), L)) if ok else None

    def _splices(self, links, top: int, pitch: float, lo: int,
                 zs: tuple | None = None) -> tuple[int, ...] | None:
        """The fewest splice layers (each a layer no link of the pillar sits in) cutting
        layers ``lo + 1..top - 1`` (between the column's end faces: the plates, or a free
        end's head layer) into segments of stock lengths at least ``min_segment`` long;
        ``None`` when none does. ``zs``: each layer's z from ``lo`` to ``top`` (else every
        layer ``pitch``)."""
        links = set(links)
        cand = [k for k in range(lo + 2, top - 1) if k not in links]
        mid = (lo + top) / 2

        def z(k: int) -> tuple[float, float]:
            return zs[k - lo][:2] if zs is not None else (k * pitch, (k + 1) * pitch)

        def gap(a: int, b: int) -> float:
            """Between faces ``a`` and ``b``: the z, less the air its parts close."""
            g = z(b)[0] - z(a)[1]
            if zs is not None:
                g -= sum(zs[k - lo][2] for k in range(a + 1, b))
            return g

        def fits(sp) -> bool:
            faces = [lo, *sp, top]
            return all(self.segment(gap(a, b), b - 1 > a and b - 1 not in links) is not None
                       for a, b in itertools.pairwise(faces))

        za, zb = lo + 0.5, top - 0.5                # the column's ends (layer units)
        loads = sorted(links)

        def moment(z: float, pattern) -> float:
            """|M| at ``z`` of a beam on the column's ends under unit loads at ``pattern``."""
            rb = sum(k - za for k in pattern) / (zb - za)
            ra = len(pattern) - rb
            return abs(ra * (z - za) - sum(z - k for k in pattern if k < z))

        patterns = [[k] for k in loads] + [loads]   # each link alone, every link at once

        def worst(sp) -> float:
            return max((moment(k, pat) for k in sp for pat in patterns), default=0.0)

        for n in range(0, 4):
            # the fewest splices; among them, the one whose worst splice sees the least moment
            # (the strength check's unit patterns on a beam between the column's ends), then
            # the farthest from the column's middle
            best = min((sp for sp in itertools.combinations(cand, n) if fits(sp)),
                       key=lambda sp: (round(worst(sp), 6),
                                       -min((abs(k - mid) for k in sp), default=0.0)),
                       default=None)
            if best is not None:
                return tuple(best)
        return None

    def column(self, pillar: bool, links: list[int], top: int,
               anchored: tuple[bool, bool], pitch: float, layout=None, air=None) -> None:
        """The planner's rule: stock segments and splices must fit the column (at the
        plan's own z when ``layout`` is final)."""
        lo, hi = self.faces(links, top, anchored)
        final = layout if layout is not None and layout.final else None
        if pillar and self.splices(links, hi, pitch, lo, final, air) is None:
            raise Unbuildable(
                f"no stock standoffs ({self.min_segment:g}-{self.max_segment:g} mm) and splice "
                f"plates (at layers no link of it sits in) fill its {(hi - lo - 1) * pitch:g} mm "
                "column")

    # -- strength ---------------------------------------------------------------------------

    def section(self) -> Section:
        return Section.tube(self.od, self.id_, self.yield_mpa,
                            name="6 mm Al standoff (6061, as a 6 x 3.3 tube)")

    def splice_preload_n(self) -> float:
        """The splice's clamp: the two segments screwed together on the stud, each held in
        soft-jaw pliers (round standoffs have no flats for a spanner: ``splice_nm``, ``T /
        0.2 d``)."""
        return self.splice_nm / (0.2 * 0.004)

    def splice_basis(self) -> str:
        return (f"{self.splice_preload_n():.0f} N: the segments turned together to "
                f"{self.splice_nm:g} N·m in soft-jaw pliers on steel shims, threadlocked "
                "(UNVERIFIED: the test build)")

    def splice_capacity_nmm(self) -> float:
        """The moment (N·mm) that starts to open a splice: the clamp's preload
        (:meth:`splice_preload_n`) times ``(ro^2 + ri^2) / 4 ro`` of the clamped annulus.
        (Until 2026-10-04 the end screws' 0.8 N·m preload on an acrylic ring: the end screws
        don't load a splice, and acrylic creeps out of such a clamp.)"""
        f = self.splice_preload_n()
        ro, ri = self.od / 2, self.stud_hole / 2
        return f * (ro * ro + ri * ri) / (4 * ro)

    def shim_od(self) -> float:
        return float(get(self.shim_key).dims["od"])

    def splice_shims(self, pitch: float) -> list[float]:
        """The DIN 988 shims that stack to a splice layer's thickness, thickest first."""
        steps = sorted((float(t) for t in get(self.shim_key).dims["t"]), reverse=True)
        out, left = [], round(pitch, 3)
        for t in steps:
            k = int(left / t + 1e-6)
            out += [t] * k
            left = round(left - k * t, 3)
        return out

    # -- parts ----------------------------------------------------------------------------

    def realize(self, group: AxleGroup, build: Build) -> Realized:
        out = Realized()
        col = Column.of(build, group)
        p = build.ctx.params
        pitch = build.ctx.pitch
        xy = xy_of(build, group)
        host = build.plan.topo.frame_bodies[0]
        stem = group.name.replace(":", "_")
        top = build.top
        anchored = (0 in col.anchors, top in col.anchors)
        lo_face, hi_face = self.faces(col.links, top, anchored)
        splices = self.splices(set(col.links), hi_face, pitch, lo_face, build.plan.layout,
                               lambda a, b: column_air(build, group, a, b))
        if splices is None:
            raise ConstructionError(f"{group.name}: no stock standoffs and splices fill its "
                                    "column")
        shims = self.splice_shims(pitch)
        for k in col.between:
            role, r = col.roles[k]
            if role == "neck":
                raise ConstructionError(f"{group.name}: a standoff can't neck down (layer {k})")
            z0, z1 = ring_z(build, k)
            if k in splices:
                # the splice plate: a stack of steel shims (stock), clamped between the
                # segments' end faces (an acrylic ring would creep out of the clamp)
                t = sum(shims)
                part = ring(xy, self.shim_od(), self.stud_hole, z0, z0 + t)
                out.bodies.append(hardware(f"{stem}_splice{k}", part, host, fab="purchased",
                                           bom_key=self.shim_key, color=STEEL))
                if len(shims) > 1:
                    out.extras.append(BomLine(self.shim_key, len(shims) - 1,
                                              f"{group.name}: splice at layer {k}"))
                continue
            out.bodies.append(hardware(f"{stem}_ring{k}",
                                       ring(xy, 2 * r, self.od + self.ring_fit, z0, z1), host,
                                       fab="printed", color=SLEEVE_COLOR))
        from spiderpig.hardware.crank_catalog import gobilda_1501

        faces = [lo_face, *splices, hi_face]
        segments = []
        long = 0.0          # how much longer the stock segments are than their gaps
        for a, b in zip(faces, faces[1:], strict=False):
            z0, z1 = build.z(a)[1], build.z(b)[0]
            span = z1 - z0 - column_air(build, group, a + 1, b - 1)
            free = b - 1 > a and b - 1 not in col.links
            length = self.segment(span, free)
            if length is None:
                raise ConstructionError(f"{group.name}: no stock standoff fits {span:.2f} mm")
            long += max(0.0, length - span)
            short = span - length
            if short > self.max_short + EPS:
                # shims under the face over it, in a spacer layer whose sleeve they shorten
                sh = self.splice_shims(round(short, 1))
                t = sum(sh)
                z1 = min(z1, build.z(b - 1)[1])     # in the spacer layer, under any gap
                zs0 = z1 - t
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
                        r = (bb.max.X - bb.min.X) / 2
                        out.bodies[i] = hardware(sleeve, ring(xy, 2 * r, self.od + self.ring_fit,
                                                              bb.min.Z, min(bb.max.Z, zs0)),
                                                 host, fab="printed", color=SLEEVE_COLOR)
                z1 = zs0
            seg = disc(xy, self.od / 2 - 0.01, z0, z1) - disc(xy, 2.0, z0 - 1, z1 + 1)
            out.bodies.append(hardware(f"{stem}_standoff{a}", seg, host, fab="purchased",
                                       bom_key=gobilda_1501(length), color=ALU))
            segments.append(length)
        long += gap_washers(build, group, col, out, self.od, host, stem)
        w_od, w_id, w_t = self.washer()
        ends = [(lo_face, -1.0, anchored[0]), (hi_face, 1.0, anchored[1])]
        for k, sign, plate in ends:
            key, L, hd, hh = self.end_screw(build.plan.t(k) if plate else pitch, plate)
            if plate:            # outside the frame plate
                face = build.z(k)[0] if sign < 0 else build.z(k)[1]
            else:                # in the end layer, on the column's free end
                face = build.z(k)[1] if sign < 0 else build.z(k)[0]
            washer = bored(disc(xy, w_od / 2, *sorted((face, face + sign * w_t))), xy, w_id,
                           *sorted((face, face + sign * w_t)))
            bear = face + sign * w_t
            head = disc(xy, hd / 2, *sorted((bear, bear + sign * hh)))
            shank = disc(xy, 3.9 / 2, *sorted((bear, bear - sign * L)))
            out.bodies += [
                hardware(f"{stem}_washer{k}", washer, host, fab="purchased",
                         bom_key=self.washer_key, color=STEEL),
                hardware(f"{stem}_screw{k}", union([head, shank]), host, fab="purchased",
                         bom_key=key, color=STEEL)]
        stud_key, stud_len = self.stud(pitch)
        for k in splices:
            z0, z1 = build.z(k)
            zm = (z0 + z1) / 2
            out.bodies.append(hardware(f"{stem}_stud{k}", disc(xy, 3.9 / 2, zm - stud_len / 2,
                                                               zm + stud_len / 2),
                                       host, fab="purchased", bom_key=stud_key, color=STEEL))
        if self.lock_key is not None:
            out.extras.append(BomLine(self.lock_key, self.lock_per_screw * (len(ends)
                                                                            + len(splices)),
                                      f"{group.name} screws and studs"))
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
                        + (f", the stock segments {long:.2f} mm over their gaps" if long else "")),
            section=self.section())
        note["supports"] = [k for k, a in ((0, anchored[0]), (top, anchored[1])) if a]
        cap = self.splice_capacity_nmm()
        note["splices"] = [{"layer": k, "capacity_nmm": round(cap, 1),
                            "basis": self.splice_basis()} for k in splices]
        note["segments_mm"] = segments
        out.notes["wobble"] = {group.name: note}
        return out



@functools.lru_cache(maxsize=4096)
def _splices(axle: StandoffAxle, links: frozenset[int], top: int, pitch: float,
             lo: int, zs: tuple | None = None) -> tuple[int, ...] | None:
    """:meth:`StandoffAxle.splices`, remembered (the planner asks per layout)."""
    return axle._splices(links, top, pitch, lo, zs)
