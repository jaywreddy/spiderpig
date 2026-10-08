"""The bolt crank (:class:`BoltCrank`): its parameters, dims, rules, claims and parts."""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, replace

import numpy as np

from spiderpig.construction.base import (
    Build,
    ConstructionError,
    Context,
    DriveInterface,
    Params,
    Realized,
)
from spiderpig.construction.crank.base import (
    EPS,
    GROUP,
    CrankDims,
    CrankGroup,
    CrankRoute,
    chains_of,
)
from spiderpig.construction.crank.capacity import CapacityMixin
from spiderpig.construction.crank.hex import HexFitMixin, HexJoint
from spiderpig.construction.crank.plates import _WebPlates
from spiderpig.construction.crank.web import WebFitMixin
from spiderpig.hardware.fasteners import SCREWS
from spiderpig.stack import Disc, Layout, Placed, Unbuildable


@dataclass(frozen=True)
class BoltCrank(HexFitMixin, WebFitMixin, CapacityMixin):
    """The laser-cut crank (``bolt``, the default; ``bolt_round``): single aluminium web plates.

    Every web is **one plate** cut from the crank's sheet (the default 0.100 in 6061-T6,
    :data:`config.CRANK_SHEET`; :meth:`resolve` reads its thickness and yield), every
    crankpin and journal a stock **hex standoff** (``pin="hex"``, ``bolt``): an M3 x 5.5 AF
    steel standoff whose ends sit in hex pockets of its two webs, an M3 button head and DIN
    9021 washer into each end, the riders turning on a printed sleeve over the hex; or
    (``pin="round"``, ``bolt_round``) a goBILDA 1501 round standoff clamped between its webs
    by an M4 button head into each end (friction). :class:`_WebPlates` builds it; the
    planner's rules for it are in :mod:`construction.route` (:meth:`joint_rules`).

    Assembly: bottom up with the legs, the hub chain capped by the hub plate, which comes
    on with the horn, the servo and the inner plate as one unit
    (:data:`construction.robot.ASSEMBLY`, steps 2 to 4). Axially (the assembly audit of
    2026-10-04): the capped standoff is carried by its sleeve, a light press on the hex
    caught between its two plates (``capped_press``), and the crank body (stub, webs,
    standoffs) stops toward the outer plate on the stub's printed thrust sleeve and toward
    the hub on the capped sleeve (``sleeve_play``): it floats 0.1 + 0.2 mm, and the capped hex
    is rated in the hub's depth less the 0.1. The round standoff's clamp needs a screw over
    the hub plate as well, so a ``bolt_round`` chain that ends in the hub plate has no
    assembly order (:meth:`_WebPlates.assembly_issue`, an audit error).
    """

    key: str = "bolt"
    label: str = ("laser-cut aluminium crank: one plate per web, each crankpin a steel "
                  "hex standoff keyed in hex pockets of its webs (screws and wide washers "
                  "retain them), the riders on a printed sleeve, a round standoff journal "
                  "stub, bolted to the horn through its top plate")
    bolt_d: float = 6.0           # (round) the standoff's washers in a gap: their size
    bore: float = 6.4             # (round) those washers' bore
    lock_key: str | None = "threadlocker_243"
    lock_per_bolt: float = 0.01
    stub_screw_engage: float = 2.5   # least thread of the stub screw in the standoff (5
    #                                  turns of M3; 3.0 before the 3.175 mm aluminium plates)
    stub_seat: float = 1.5           # least of the stub in the outer frame plate
    washer_r: float = 6.0            # PTFE washers / shims (6 x 12) on a crankpin through a gap
    web_t: float = 3.175             # the crank sheet's thickness, set by resolve
    stub_below: float = 2.5          # the stub may stand this far out under the
    #                                  outer frame plate (a thin plate leaves a stock length
    #                                  too little room to end inside it); claimed in layer -1
    pin_od: float = 6.0              # the single webs' crankpin: a goBILDA 1501 round standoff
    pin_hole: float = 4.5            # a web's hole for its M4 screw
    pin_min_engage: float = 2.8      # least M4 thread in a standoff's end (4 turns)
    pin_preload_n: float = 2200.0    # an M4 button head at about 2 N·m into the standoff
    pin_mu: float = 0.3              # the standoff's end face on a web (anodised on bare
    #                                  aluminium, dry; UNVERIFIED: the test-build checklist)
    head_mu: float = 0.2             # the screw head on the web
    web_edge_t: float = 1.0          # a web's wall round a hole, in sheet thicknesses
    head_clear: float = 0.3          # z clearance over a head in its gap
    # -- the crankpin: a hex standoff (the default) or the round one ---------
    pin: str = "hex"                 # "hex": a steel hex standoff keyed in hex pockets (the
    #                                  user's decision of 2026-10-04); "round": the goBILDA
    #                                  round standoff clamped by friction (``bolt_round``)
    hex_af: float = 5.5              # the M3 hex standoff's across-flats (crank_catalog)
    hex_fit: float = 0.1             # a hex pocket over the AF (laser-cut, a slide fit)
    dogbone_r: float = 0.8           # the pocket's corner reliefs: at least the service's
    #                                  inside radius (SendCutSend 0.8 mm), each a circle
    #                                  through the hex's corner, centred out along its bisector,
    #                                  so the flats keep their whole length
    hex_corner_loss: float = 0.3     # off each flat for the standoff's rounded corners (mm)
    hex_yield: float = 300.0         # the standoff's steel in bearing (MPa; brass 250)
    web_yield: float = 276.0         # the crank sheet's (6061-T6; set by for_sheet)
    sleeve_od: float = 8.5           # the printed sleeve the riders turn on (its wall 0.9 mm
    #                                  round the bore's corners; a rider's hole + wall within
    #                                  link_radius 6)
    sleeve_fit: float = 0.3          # its hex bore over the standoff's AF (printed)
    sleeve_play: float = 0.2         # its length short of the plates' inner faces
    protrude_max: float = 1.2        # a standoff end past its plate's outer face, at most
    collar_min: float = 0.4          # a printed collar's thinnest (under it the plate floats)
    recess_max: float = 0.3          # a standoff end inside its pocket, at most (the hex in
    #                                  2.24 of 0.100 in 6061-T6: 3.83 N·m, SF 2.25 at 1.7 N·m;
    #                                  materials.ROLES["crank"] counts it)
    hex_min_engage: float = 2.5      # least M3 thread of a screw in the standoff (5 turns)
    hex_tip_gap: float = 0.3         # between the two screws' tips inside the standoff
    hex_engage_max: float = 6.0      # the screws' thread in the standoff, at most
    capped_press: float = 0.1        # (hex) the capped chain's printed sleeve: its hex bore
    #                                  printed this much under the standoff's AF, a light
    #                                  press (the assembly audit of 2026-10-04): the sleeve,
    #                                  captured between its two plates, then carries the
    #                                  standoff, which nothing else holds axially (no screw
    #                                  over the hub plate)
    thrust_od: float = 8.5           # its outside (it bears on the plate round the 6.6 hole)
    thrust_play: float = 0.1         # its end short of the outer plate's inner face

    # -- the sheet ------------------------------------------------------------------------

    horn_shim_max: float = 2.0       # DIN 988 shims under a horn screw's head, at most (mm)

    def resolve(self, ctx: Context) -> BoltCrank:
        """This crank for ``ctx``'s crank sheet (:meth:`for_sheet`, its thickness as the
        design measures it)."""
        return replace(self.for_sheet(ctx.sheet("crank")), web_t=ctx.sheet_t("crank"))

    @property
    def hex(self) -> bool:
        """Hex standoff crankpins (the default; ``bolt_round``: the round standoff)."""
        return self.pin == "hex"

    def for_sheet(self, key: str | None) -> BoltCrank:
        """This crank cut from sheet ``key``: its plates' thickness and yield (``None``: as
        it stands). A non-metal sheet raises :class:`ConstructionError`: the single plates
        are aluminium (an acrylic crank sheet built the two-plate M6 stack crank, removed on
        2026-10-07; :class:`config.BuildConfig` refuses it first)."""
        if key is None:
            return self
        from spiderpig.materials import sheet

        sh = sheet(key)
        if not sh.metal:
            raise ConstructionError(
                f"the bolt crank's single web plates need a metal crank sheet, not {key!r} "
                "(the acrylic two-plate crank was removed on 2026-10-07)")
        return replace(self, web_t=sh.thickness, web_yield=sh.yield_mpa)

    # -- dimensions and rules -----------------------------------------------------------

    def dims(self, ctx: Context) -> CrankDims:
        p: Params = ctx.params
        drive: DriveInterface = ctx.interfaces["drive"]
        t = ctx.sheet_t("crank")
        wall = max(p.min_wall, self.web_edge_t * t)
        hub = max(drive.horn_radius, drive.screw_pcd / 2 + drive.screw_head_d / 2 + p.min_wall)
        sh = ctx.sheet("crank")
        if sh is not None:
            from spiderpig.materials import sheet

            # the horn screws' holes at the service's edge distance from the hub's rim
            hub = max(hub, drive.screw_pcd / 2 + self.horn_hole(ctx) / 2 + sheet(sh).min_edge)
        if self.hex:
            web = max(p.web_radius, math.ceil((self.hex_reach() + wall) * 10) / 10,
                      self.head_r(),
                      # the stub screw's hole at O, the service's edge distance from the rim
                      math.ceil((3.4 / 2 + (sheet(sh).min_edge if sh else wall)) * 10) / 10)
            journal = math.ceil((self.hex_pocket_af() / math.sqrt(3) + 0.2) * 10) / 10
        else:
            # its screw holes the service's edge distance (2 t) from the web's rim
            edge = max(wall, sheet(sh).min_edge) if sh is not None else wall
            web = max(p.web_radius, math.ceil((self.pin_hole / 2 + edge) * 10) / 10,
                      self.head_r())
            journal = self.pin_od / 2
        rd = self.rider_d()
        dims = CrankDims(web=web, journal=journal, stub=self.stub_od() / 2,
                         post=rd / 2, hub=hub, hub_thickness=t - 1e-3)
        if p.hole(rd) / 2 + p.min_wall > p.link_radius + EPS:
            raise ConstructionError(
                f"a {rd:g} mm crankpin's hole leaves less than {p.min_wall} mm of the "
                f"rider around it (link radius {p.link_radius})")
        if dims.journal > dims.web:
            raise ConstructionError("the crank body on O must not be wider than a web")
        face = (drive.horn_face_depth - ctx.sheet_t("frame")) / ctx.pitch
        if abs(face - round(face)) > 1e-6:
            raise ConstructionError(
                f"the bolt crank's hub plate needs the horn's face on a layer boundary; it is "
                f"{drive.horn_face_depth:g} mm under the inner plate's top face in "
                f"{ctx.pitch:g} mm layers (the drive's horn spacer sizes for it: "
                "DriveGroup.spacer)")
        if self.horn_joint_web(ctx, t, drive.spacer_t) is None:
            raise ConstructionError(f"no screw fits between the bolt crank's hub plate and the "
                                    f"{ctx.servo.key} horn")
        if not self.stub_layers_web(ctx.sheet_t("frame"), ctx.pitch, t):
            raise ConstructionError(f"no stock stub standoff fits {ctx.pitch:g} mm layers")
        return dims

    def check_route(self, L: Layout, route: CrankRoute, ridden: dict[str, set[int]],
                    plates: set[int], drive: DriveInterface | None = None) -> None:
        """The plan's own z (:attr:`stack.Layout.final`): the stub standoff reaches the
        outer frame plate (the chains' standoffs and the horn screws: :meth:`web_claims`;
        :class:`stack.Unbuildable` if not)."""
        if route.bearing and plates:
            a = min(ch[0].lo for ch in chains_of(route.runs)) - 1
            if self.stub_z(L.z(a)[0] - L.z(0)[0], L.t(0), self.web_t) is None:
                raise Unbuildable(f"at the plan's z no stock stub standoff reaches the outer "
                                  f"frame plate from the lowest web (layer {a})")

    def joint_rules(self, ctx: Context, dims: CrankDims):
        """The router's rules (:class:`construction.route.JointRules`): every span a stock
        standoff fits, the screw heads' and washers' gap pieces, the stub's layers."""
        from spiderpig.construction.route import JointRules, horn_pockets

        p, t = ctx.pitch, ctx.sheet_t("crank")
        spans: dict[int, int] = {}
        for n in range(3, 64):
            m = 0
            if self._web_span_ok(n - 2, 0, p, t):
                for a in (0, 1):
                    for low in range(4):
                        for c in (0, 1):
                            for d in (0, 1):
                                m |= 1 << (16 * a + 4 * low + 2 * c + d)
            spans[n] = m
        r = self.head_r()
        hr = self.horn_head_r(ctx)
        return JointRules(spans, head=r, nut=r, post=dims.post, horn=horn_pockets(ctx),
                          bottom_layers=self.stub_layers_web(ctx.sheet_t("frame"), p, t),
                          gap_head=r, horn_heads=tuple((h, hr) for h in self.horn_points(ctx)),
                          j_spans=tuple((n, self._web_span_ok(n, 0, p, t)) for n in range(64)),
                          gap_washer=self.washer_r)

    # -- the stub and the strength check's capacities -------------------------------

    def stub_od(self) -> float:
        from spiderpig.hardware.catalog import get
        from spiderpig.hardware.crank_catalog import M3_ROUND_STANDOFF_LENGTHS, m3_round_standoff

        return float(get(m3_round_standoff(M3_ROUND_STANDOFF_LENGTHS[0])).dims["od"])

    def stub_z(self, top: float, plate: float, upper: float
               ) -> tuple[str, float, float, float] | None:
        """The stub standoff under the lowest web, at a plan's own z: ``top`` that web's
        bottom face over the outer frame plate's bottom face, ``plate`` that plate's
        thickness, ``upper`` the web the screw passes. (key, length, its bottom end's z, the
        screw's length): the longest stock length that ends at most ``stub_below`` under the
        outer plate and at least ``stub_seat`` inside it, and a button head that engages it
        ``stub_screw_engage`` (its head on the lowest web, from above)."""
        from spiderpig.hardware.catalog import get
        from spiderpig.hardware.crank_catalog import M3_ROUND_STANDOFF_LENGTHS, m3_round_standoff

        below = self.stub_below
        for S in sorted(M3_ROUND_STANDOFF_LENGTHS, reverse=True):
            z0 = top - S
            if not -below - EPS <= z0 <= plate - self.stub_seat + EPS:
                continue
            depth = float(get(m3_round_standoff(S)).dims["thread_depth"])
            for L in SCREWS["bhcs", "3"].lengths:
                e = L - upper
                if self.stub_screw_engage - EPS <= e <= depth + EPS:
                    return m3_round_standoff(S), S, z0, L
        return None

    @staticmethod
    def plate_z(L: Layout, k: int, t: float, hub: int | None = None) -> tuple[float, float]:
        """A single crank plate's z in layer ``k``: on the layer's floor, the hub plate at its
        top (against the horn spacer). A plate thinner than its layer leaves air over (or
        under) it, which the crankpins' shims and the screws' heads use."""
        z0, z1 = L.z(k)
        return (z1 - t, z1) if k == hub else (z0, z0 + t)

    def hub_capped(self, ctx: Context, at: str) -> bool:
        """Whether a crankpin whose chain ends in the hub plate takes no screw over the
        hub plate, only the one from below: every hex one (the hub plate, held to the horn
        by the horn screws, caps its upper end; the hex keeps its whole depth in the hub
        plate\'s pocket); the round standoff\'s clamp needs both screws."""
        return self.hex

    def stub_thrust_r(self, ctx: Context, route: CrankRoute, hub: int) -> float:
        """The radius of the stub's thrust sleeve  when the route has
        a chain capped in the hub plate (0: none): with no screw over the hub plate, the hex
        only slides in its pocket, and nothing else stops the crank body (stub, webs,
        standoffs) moving toward the outer plate (the assembly audit of 2026-10-04: the hex
        would leave the hub plate's pocket after 2.54 mm, the screw heads meet the links
        first)."""
        if not (self.hex and route.bearing):
            return 0.0
        for ch in chains_of(route.runs):
            if ch[-1].hi + 1 == hub and self.hub_capped(ctx, ch[0].at):
                return self.thrust_od / 2
        return 0.0

    def hub_head_need(self, ctx: Context, horn_radius: float) -> float:
        """How thick the printed horn spacer must be for a crankpin\'s screw head over the hub
        plate (it stands in a pocket of the spacer, under the horn): a round standoff\'s
        crankpin within a head\'s reach of the horn\'s rim (a hex one is capped by the hub
        plate, :meth:`hub_capped`); 0 when none is (:meth:`servos.mount.DriveGroup.spacer`
        adds a layer for it)."""
        if self.hex:
            return 0.0              # capped by the hub plate: no head over it
        hr = self.head_r()
        g = ctx.topo.geometry.points
        head = self.pin_screw()[1] + self.head_clear
        need = 0.0
        for a in ctx.topo.axes_of("crankpin"):
            R = float(np.linalg.norm(g[a.name][0] - g["O"][0]))
            if horn_radius + hr + ctx.params.margin <= R:
                continue
            need = max(need, head)
        return need

    def web_claims(self, ctx: Context, L: Layout, route: CrankRoute, ridden, d: CrankDims,
                   plate_t: float, drive: DriveInterface, hub: int, horn_pts: list[str]
                   ) -> list[Placed]:
        """What single-plate webs add to the route's shapes: per chain the tip under its
        lowest web, the jam nut over it and the head over its top web (clearance shapes:
        each in the gap beside its plate, or sunk into the layer there when nothing is in
        its way); the stub plate under the first chain; the plates between the last chain
        and the hub as wide as the hub (the horn screws pass them); the horn screws' heads
        under the lowest of those."""
        out: list[Placed] = []
        hr = self.head_r()
        chains = chains_of(route.runs)
        clear = self.head_clear
        head = self.pin_screw()[1] + clear
        if self.hex:
            head = self.hex_stack(0.0, 0) + clear
        t = plate_t
        spacer_r = (self.rider_d() / 2 + 0.5) if self.hex else self.pin_od / 2 + 1.0

        def heights(j, w1: int) -> tuple[float, float]:
            """What hangs under the lowest web and over the top web (in layer ``w1``; with
            clearance): the hex pin's upper stack in the gap over that web's layer less the
            air over the plate in it."""
            if isinstance(j, HexJoint):
                up = max(0.0, j.stack_hi + clear - self.air_over(L, w1, t, hub))
                return round(j.stack_lo + clear, 3), round(up, 3)
            return head, head

        for ch in chains:
            at, lo, hi = ch[0].at, ch[0].lo, ch[-1].hi
            capped = hi + 1 == hub and self.hub_capped(ctx, at)
            j = self.chain_fit_web(L, lo, hi, t=t, hub=hub, capped=capped)
            if j is None and L.final:
                raise Unbuildable(f"at the plan's z no stock standoff fits the crankpin at {at} "
                                  f"(webs in layers {lo - 1} and {hi + 1})")
            if lo - 2 < 0:
                raise Unbuildable(f"the screw under crankpin {at} needs layer {lo - 2}")
            h_lo, h_hi = heights(j, hi + 1)
            out.append(Placed(lo - 2, Disc(at, hr), GROUP, f"crankpin screw {at}", gap=True,
                              height=h_lo, toward=-1))
            if self.hex and j is not None and (j.gap > 0 or self.hex_gap_needed(
                    L, lo, hi, t, hub, capped)):
                # the gap over the lowest web the standoff's length opened (hex_gap_fit): a
                # printed ring on the sleeve fills it, so the lowest rider keeps its layer
                # (_WebPlates.gap_parts prints it, as the run's rings)
                out.append(Placed(lo - 1, Disc(at, self.washer_r), GROUP,
                                  f"crankpin {at} washer", gap=True, height=j.gap))
            if isinstance(j, HexJoint):
                # the gaps along the run it needs (their rings: the run's washer claims)
                out += [Placed(k, Disc(at, spacer_r), GROUP, f"crankpin {at} gap", gap=True,
                               height=h) for k, h in j.gaps if k != lo - 1]
            if capped:
                # under the horn spacer, which caps its upper end: no screw there
                out.append(Placed(lo - 1, Disc(at, spacer_r), GROUP,
                                  f"crankpin {at} spacer", gap=True, height=0.0))
                continue
            out.append(Placed(hi + 1, Disc(at, hr), GROUP, f"crankpin screw {at}", gap=True,
                              height=h_hi, toward=1))
            # the standoff and its shims through the gap over the lowest web (the height:
            # its length past the layers and the air over the plate); the hex pin's sleeve
            out.append(Placed(lo - 1, Disc(at, spacer_r), GROUP,
                              f"crankpin {at} spacer", gap=True,
                              height=j.gap if j is not None else 0.0))
            g = ctx.topo.geometry.points
            R = float(np.linalg.norm(g[at][0] - g["O"][0]))
            if hi + 1 == hub and drive.horn_radius + hr + ctx.params.margin > R:
                # its head over the hub plate stands in a pocket of the printed horn spacer
                # (servos.mount.DriveGroup.realize cuts it), which must be that thick
                spacer = (self.horn_spacer(L, drive, hub) if L.final else drive.spacer_t)
                if spacer < h_hi - 1e-6:
                    raise Unbuildable(f"the screw head of crankpin {at} over the hub plate "
                                      f"needs {h_hi:.2f} mm of the horn spacer, which is "
                                      f"{spacer:.2f} mm")
                if hr + (drive.center_head_d + ctx.params.print_fit) / 2 + 0.5 > R:
                    raise Unbuildable(f"the screw head of crankpin {at} would meet the horn's "
                                      "centre screw")
        # journal standoffs on O between chains that don't share a web
        order = sorted(chains, key=lambda ch: ch[0].lo)
        for c0, c1 in itertools.pairwise(order):
            e, f = c0[-1].hi + 1, c1[0].lo - 1
            if f <= e:
                continue
            j = self.chain_fit_web(L, e + 1, f - 1, t=t, hub=hub, sleeve=False)
            if j is None and L.final:
                raise Unbuildable(f"at the plan's z no stock standoff joins the crank's webs "
                                  f"in layers {e} and {f} on O")
            h_lo, h_hi = heights(j, f)
            out += [Placed(k, Disc("O", d.journal), GROUP, "crank journal")
                    for k in range(e + 1, f)]
            out.append(Placed(e - 1, Disc("O", hr), GROUP, "journal screw", gap=True,
                              height=h_lo, toward=-1))
            out.append(Placed(f, Disc("O", hr), GROUP, "journal screw", gap=True,
                              height=h_hi, toward=1))
            out.append(Placed(e, Disc("O", d.journal if self.hex else self.pin_od / 2 + 1.0),
                              GROUP,
                              "crank journal spacer", gap=True,
                              height=j.gap if j is not None else 0.0))
        first = min(chains, key=lambda ch: ch[0].lo)
        a = first[0].lo - 1
        if route.bearing:
            # the stub standoff's screw from above the lowest web: its head in the air over
            # that plate, the rest in the gap there
            sk = SCREWS["bhcs", "3"]
            air = L.t(a) - t
            h = sk.head_h + clear - air
            out.append(Placed(a, Disc("O", sk.head_d / 2 + 0.3), GROUP, "stub screw head",
                              gap=True, height=round(max(h, 0.0), 3)))
        top_web = max(ch[-1].hi + 1 for ch in chains)
        out += [Placed(k, Disc("O", d.hub), GROUP, "crank body", sheet=plate_t)
                for k in range(top_web + 1, hub)]
        seg = top_web
        sk = self.horn_kind(ctx)
        got = self.horn_fit_web(ctx, t, self.horn_spacer(L, drive, hub) if L.final
                                else drive.spacer_t)
        if L.final and got is None:
            raise Unbuildable(f"at the plan's z no stock horn screw fits the crank's top "
                              f"plates (layers {seg}..{hub}) and the horn spacer")
        shim = got[3] if got is not None else 0.0
        if got is not None and got[0].head_h > sk.head_h:
            sk = got[0]
        for name in horn_pts:
            out.append(Placed(seg - 1, Disc(name, self.horn_head_r(ctx, shim)), GROUP,
                              "horn screw head", gap=True, height=sk.head_h + shim + clear,
                              toward=-1))
        return out

    # -- parts ----------------------------------------------------------------------------

    def realize(self, group: CrankGroup, build: Build) -> Realized:
        return _WebPlates(self, group, build).make()


BOLT_ROUND = BoltCrank(
    key="bolt_round", pin="round",
    label=("laser-cut crank, its crankpins round goBILDA standoffs clamped between single "
           "aluminium webs by M4 screws (friction; the default before the hex standoff of "
           "2026-10-04)"))
"""The friction-clamped round-standoff crankpin (``--crank bolt_round``): TrotBot's heel and
toe (:data:`config.LINKAGE_CRANKS`), where the hex crankpin's 8.5 mm sleeve doesn't clear b7."""
