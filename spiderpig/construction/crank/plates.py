"""The bolt crank's parts (:class:`_WebPlates`): plates, standoffs, screws, sleeves."""

from __future__ import annotations

import itertools
import math
from typing import TYPE_CHECKING

import numpy as np

from spiderpig.construction.base import (
    FRAME_OUTER,
    Build,
    ConstructionError,
    DriveInterface,
    Realized,
    hardware,
)
from spiderpig.construction.crank.base import (
    BOLT_COLOR,
    EPS,
    GROUP,
    PRESS_DRAWN,
    SEGMENT_COLOR,
    STEEL,
    CrankGroup,
    _hex,
    chains_of,
    hex_play,
    route_of,
)
from spiderpig.construction.crank.hex import HexJoint
from spiderpig.construction.crank.web import SHIM_KEY, WebJoint, shim_stack
from spiderpig.construction.envelope import shape_solid
from spiderpig.hardware.bom import BomLine
from spiderpig.hardware.fasteners import SCREWS, parse, screw_solid
from spiderpig.shapes import Cut, disc, union

if TYPE_CHECKING:
    from spiderpig.construction.crank.bolt import BoltCrank


class _WebPlates:
    """The bolt crank of one side (:meth:`BoltCrank.realize`): every crank layer one
    aluminium plate (on its layer's floor; the hub plate at its layer's top, against the horn
    spacer), every crankpin and journal a hex standoff in hex pockets of its two webs (or a
    round standoff clamped between them by M4 screws), the stub standoff screwed to the
    lowest web from above, the horn screws up through the hub plate."""

    def __init__(self, c: BoltCrank, group: CrankGroup, build: Build):
        self.c, self.build = c, build
        ctx = build.ctx
        self.p = ctx.params
        self.d = group.dims(ctx)
        topo = build.plan.topo
        self.host = topo.crank_bodies[0]
        self.drive: DriveInterface = ctx.interfaces["drive"]
        self.pins = topo.axes_of("crankpin")
        self.route = route_of(build.plan.layout, self.pins)
        self.chains = chains_of(sorted(self.route.runs, key=lambda r: (r.lo, r.at)))
        self.plates: dict[int, list] = {}
        self.washers: list = []                 # crankpin washers in a gap
        for pl in build.shapes(GROUP):
            if pl.gap:
                if pl.label.endswith(" washer"):
                    self.washers.append(pl)
                continue
            if pl.label in ("crank body", "crank hub") or pl.label.startswith("web "):
                self.plates.setdefault(pl.layer, []).append(pl)
        self.sheet = ctx.sheet("crank")
        self.t = ctx.sheet_t("crank")
        self.cuts: dict[int, list] = {k: [] for k in self.plates}
        self.out = Realized()
        self.notes: list[dict] = []
        hub = [pl.layer for pl in build.shapes(GROUP) if pl.label == "crank hub"]
        self.hub_layer = max(hub) if hub else None

    def xy(self, point: str) -> tuple:
        return tuple(self.build.xy(point))

    def buy(self, name: str, part, key: str, color: str = STEEL) -> None:
        self.out.bodies.append(hardware(name, part, self.host, fab="purchased", bom_key=key,
                                        color=color))

    def gap_parts(self, out: Realized) -> None:
        """What fills a clearance gap a crankpin's run crosses between its riders: a printed
        ring on the hex pin's sleeve, a stack of washers on the round standoff (none where a
        horn screw's head is in that gap: the rider turns on air)."""
        from spiderpig.materials import washer_stack

        b = self.build
        horn = [pl for pl in b.shapes(GROUP) if pl.label == "horn screw head" and pl.gap]
        for pl in self.washers:
            g = b.plan.gaps.get(pl.layer, 0.0)
            if g <= 0:
                continue
            # (horn screw heads and crankpin washers are discs)
            if any(h.layer == pl.layer and math.dist(self.xy(h.shape.at),  # pyright: ignore[reportAttributeAccessIssue]
                                                     self.xy(pl.shape.at))  # pyright: ignore[reportAttributeAccessIssue]
                   < h.shape.r + pl.shape.r for h in horn):
                continue        # a horn screw's head is there: the rider turns on air
            z0, _ = b.plan.gap_z(pl.layer)
            if self.c.hex:
                # a printed ring on the riders' sleeve (no stock washer fits its 8.5 mm)
                xy = self.xy(pl.shape.at)
                ring = (disc(xy, pl.shape.r - 0.05, z0 + 0.05, z0 + g - 0.05)
                        - disc(xy, (self.c.sleeve_od + self.p.print_fit) / 2, z0 - 1,
                               z0 + g + 1))
                out.bodies.append(hardware(f"crank_ring_{pl.shape.at}_{pl.layer}", ring,
                                           self.host, fab="printed", color=SEGMENT_COLOR))
                continue
            items, _ = washer_stack(self.c.bolt_d, g)
            if not items:
                continue
            t = sum(x[1] for x in items)
            xy = self.xy(pl.shape.at)
            od = 2 * pl.shape.r
            part = disc(xy, od / 2, z0, z0 + t) - disc(xy, self.c.bore / 2, z0 - 1, z0 + t + 1)
            tag = f"{pl.shape.at}_{pl.layer}"
            self.buy(f"crank_washers_{tag}", part, items[0][0], "#f2f2f2")
            for key, tt in items[1:]:     # each shim by its thickness (the BOM orders them so)
                out.extras.append(BomLine(key, 1, f"crankpin {tag}: {tt:g} mm in the gap"))

    def pz(self, k: int) -> tuple[float, float]:
        return self.c.plate_z(self.build.plan.layout, k, self.t, self.hub_layer)

    def layer_cut(self, k: int, xy, r: float, z0: float | None = None,
                  z1: float | None = None) -> None:
        b = self.build
        if k in self.cuts:
            lo, hi = b.z(k)
            self.cuts[k].append(disc(xy, r, lo - 3 if z0 is None else z0,
                                     hi + 3 if z1 is None else z1))

    def make(self) -> Realized:
        self.journals: list[dict] = []
        for ch in self.chains:
            at, xy = ch[0].at, self.xy(ch[0].at)
            tag = at if sum(x[0].at == at for x in self.chains) == 1 else f"{at}_{ch[0].lo}"
            lo, hi = ch[0].lo, ch[-1].hi
            if len(ch) > 1:
                raise ConstructionError(
                    f"the crank's route returns to O between runs of {at} (layers "
                    f"{ch[0].hi + 1}..{ch[1].lo - 1}): a plate there would turn loose on the "
                    "standoff (one run per point: the router's rule)")
            self.notes.append(self.pin(at, xy, tag, lo, hi))
        order = sorted(self.chains, key=lambda ch: ch[0].lo)
        for c0, c1 in itertools.pairwise(order):
            e, f = c0[-1].hi + 1, c1[0].lo - 1
            if f > e:
                note = self.pin("O", self.xy("O"), f"journal{e}", e + 1, f - 1)
                self.journals.append(note)
        self.stub()
        self.hub()
        return self.finish()

    def pin(self, at: str, xy, tag: str, lo: int, hi: int) -> dict:
        """A round standoff between the webs in layers ``lo - 1`` and ``hi + 1`` at ``xy``,
        an M4 button head into each end, shims under its lower end; its note."""
        from spiderpig.hardware.catalog import get

        c, b = self.c, self.build
        if c.hex:
            return self.hex_pin(at, xy, tag, lo, hi)
        hd, hh = c.pin_screw()
        j = c.chain_fit_web(b.plan.layout, lo, hi, t=self.t, hub=self.hub_layer)
        if j is None:
            raise ConstructionError(f"no stock standoff fits the crankpin at {at} (webs in "
                                    f"layers {lo - 1} and {hi + 1})")
        w0, w1 = lo - 1, hi + 1
        z_lo, z_hi = self.pz(w0)[1], self.pz(w1)[0]
        for k in (w0, w1):
            self.layer_cut(k, xy, c.pin_hole / 2)
        # a crank plate under or over the chain at the pin (the stub plate, a hub plate)
        self.layer_cut(w0 - 1, xy, c.head_r())
        self.layer_cut(w1 + 1, xy, c.head_r())
        from spiderpig.hardware.crank_catalog import gobilda_1501, m4_set_screw

        assert isinstance(j, WebJoint)     # a round crank's chain fit (chain_fit_web)
        z0 = z_hi - j.length
        za = z0
        for i, seg in enumerate(j.segments or (j.length,)):
            st = disc(xy, c.pin_od / 2 - 0.01, za, za + seg) - disc(xy, 2.0, za - 1,
                                                                    za + seg + 1)
            self.buy(f"crank_pin_{tag}" + (f"_{i}" if i else ""), st, gobilda_1501(seg),
                     "#c8ccd0")
            za += seg
            if i + 1 < len(j.segments):
                self.buy(f"crank_pin_stud_{tag}", disc(xy, 1.95, za - 6, za + 6),
                         m4_set_screw(12))
        if j.shims > 0:
            # DIN 988 4 x 8 shims to the clamp's take-up, thickest first (the BOM orders
            # them per thickness, hardware.bom.split_shims); no PTFE washer in a clamp
            from spiderpig.hardware.bom import shim_breakdown

            key = "shim_din988_4x8"
            items = [(key, t) for t in shim_breakdown(j.shims, get(key).dims["t"])]
            t = sum(x[1] for x in items)
            if 0 < z_lo + t - z0 <= 0.1:
                # the stack's rounding (stock shim steps and the fit's tolerance) leaves the
                # standoff a hair onto the shims: they are what gives, drawn up to its end
                t = z0 - z_lo
            shim = disc(xy, 3.95, z_lo, z_lo + t) - disc(xy, 2.05, z_lo - 1, z_lo + t + 1)
            self.buy(f"crank_pin_shims_{tag}", shim, items[0][0], "#9a9a9a")
            # the stack as bought (the body may be drawn a hair short of it, above): the BOM
            # orders these thicknesses, not a breakdown of the drawn height
            self.out.notes.setdefault("shim_stacks", {})[f"crank_pin_shims_{tag}"] = [
                x[1] for x in items]
            for key, _ in items[1:]:
                self.out.extras.append(BomLine(key, 1, f"crankpin {tag}: shims"))
        sk_lo = get(j.screw_lo).dims
        sk_hi = get(j.screw_hi).dims
        for name, key, d, bearing, up in (
                (f"crank_pin_screw_lo_{tag}", j.screw_lo, sk_lo, self.pz(w0)[0], True),
                (f"crank_pin_screw_hi_{tag}", j.screw_hi, sk_hi, self.pz(w1)[1], False)):
            s_ = 1.0 if up else -1.0
            body = union([disc(xy, hd / 2, *sorted((bearing - s_ * hh, bearing))),
                          disc(xy, 1.95, *sorted((bearing, bearing + s_ * d["length"])))])
            self.buy(name, body, key)
        if c.lock_key is not None:
            self.out.extras.append(BomLine(c.lock_key, 2 * c.lock_per_bolt,
                                           f"crankpin {tag} screws"))
        return {
            "at": tag, "standoff": j.standoff, "length_mm": j.length,
            "segments_mm": list(j.segments),
            "shims_mm": j.shims, "gap_mm": j.gap, "screws": [j.screw_lo, j.screw_hi],
            "engage_mm": [j.engage_lo, j.engage_hi], "run_layers": hi - lo + 1,
            "layers": [lo - 2, hi + 1], "capacity_nm": c.capacity(j)}

    def hex_pin(self, at: str, xy, tag: str, lo: int, hi: int) -> dict:
        """A hex standoff between the webs in layers ``lo - 1`` and ``hi + 1`` at ``xy``
        (``at`` "O": a journal, no riders): its hex ends in the webs' hex pockets, an M3
        button head and a DIN 9021 washer into each end, printed hex collars where it stands
        past a plate, the riders' printed sleeve between the plates; its note."""
        c, b = self.c, self.build
        journal = at == "O"
        capped = hi + 1 == self.hub_layer and not journal and c.hub_capped(b.ctx, at)
        j = c.chain_fit_web(b.plan.layout, lo, hi, t=self.t, hub=self.hub_layer,
                            sleeve=not journal, capped=capped)
        if j is None:
            raise ConstructionError(f"no stock hex standoff fits the crankpin at {at} (webs in "
                                    f"layers {lo - 1} and {hi + 1})")
        w0, w1 = lo - 1, hi + 1
        ang = 0.0 if journal else b.angle("O", at)
        z_lo, z_hi = self.pz(w0)[0], self.pz(w1)[1]          # the plates' outer faces
        for k in (w0, w1):
            if k in self.cuts:
                z0, z1 = self.pz(k)
                self.cuts[k].append(c.hex_cut(xy, z0 - 1, z1 + 1, ang))
        # a crank plate under or over the chain at the pin (the stub plate, a hub plate)
        self.layer_cut(w0 - 1, xy, c.head_r())
        self.layer_cut(w1 + 1, xy, c.head_r())
        assert isinstance(j, HexJoint)     # a hex crank's chain fit (chain_fit_web)
        e0, e1 = z_lo - j.out_lo, z_hi + j.out_hi            # the standoff's ends
        st = _hex(xy, c.hex_af, e0, e1, ang) - disc(xy, 1.5, e0 - 1, e1 + 1)   # M3, through
        self.buy(f"crank_pin_{tag}", st, j.standoff, "#b9b9b9")
        _hd, _hh = c.hex_screw()
        _, wod, wt = c.hex_washer()
        et = c.extra_washer_t()
        pocket = c.hex_pocket_af() + c.sleeve_fit - c.hex_fit
        for side, end, collar, key, k, s_ in (("lo", e0, j.collar_lo, j.screw_lo, j.washers_lo,
                                                -1.0),
                                               ("hi", e1, j.collar_hi, j.screw_hi, j.washers_hi,
                                                1.0)):
            face = z_lo if side == "lo" else z_hi
            if not key:
                continue                    # capped by the horn spacer: no screw
            # the washer on the standoff's end, or on the plate when the end is recessed
            end = min(end, face) if side == "lo" else max(end, face)
            if collar > 0:
                zc = sorted((face, face + s_ * collar))
                ring = disc(xy, wod / 2 - 0.05, *zc) - _hex(xy, pocket, zc[0] - 1, zc[1] + 1,
                                                            ang)
                self.out.bodies.append(hardware(f"crank_pin_collar_{side}_{tag}", ring,
                                                self.host, fab="printed", color=SEGMENT_COLOR))
            zw = sorted((end, end + s_ * wt))
            washer = disc(xy, wod / 2, *zw) - disc(xy, 1.6, zw[0] - 1, zw[1] + 1)
            self.buy(f"crank_pin_washer_{side}_{tag}", washer, "m3_washer_9021", "#d0d0d0")
            bearing = end + s_ * wt
            if k:
                zx = sorted((bearing, bearing + s_ * k * et))
                extra = disc(xy, 3.5, *zx) - disc(xy, 1.6, zx[0] - 1, zx[1] + 1)
                self.buy(f"crank_pin_washers_{side}_{tag}", extra, "m3_washer", "#d0d0d0")
                if k > 1:
                    self.out.extras.append(BomLine("m3_washer", k - 1, f"crankpin {tag}"))
                bearing += s_ * k * et
            got = parse(key)
            assert got is not None          # the fit's keys are modelled screws (fit_hex)
            sk, length = got
            self.buy(f"crank_pin_screw_{side}_{tag}",
                     screw_solid(xy, sk, bearing, length, up=side == "lo"), key)
        press = capped
        if j.sleeve > 0:
            z0 = self.pz(w0)[1] + (self.pz(w1)[0] - self.pz(w0)[1] - j.sleeve) / 2
            bore = pocket
            if press:
                # pressed on the hex resting on the lower plate (the washer under that plate
                # drawn up against it), so the play is all over it; drawn a hair over the AF
                # (the print is ``capped_press`` under it)
                z0, bore = self.pz(w0)[1], c.hex_af + PRESS_DRAWN
            sleeve = (disc(xy, c.sleeve_od / 2, z0, z0 + j.sleeve)
                      - _hex(xy, bore, z0 - 1, z0 + j.sleeve + 1, ang))
            self.out.bodies.append(hardware(f"crank_pin_sleeve_{tag}", sleeve, self.host,
                                            fab="printed", color=SEGMENT_COLOR))
        if c.lock_key is not None:
            self.out.extras.append(BomLine(c.lock_key, 2 * c.lock_per_bolt,
                                           f"crankpin {tag} screws"))
        return {
            "at": tag, "standoff": j.standoff, "length_mm": j.length, "span_mm": j.span,
            "capped": capped,
            "segments_mm": [j.length], "out_mm": [j.out_lo, j.out_hi],
            "collars_mm": [j.collar_lo, j.collar_hi],
            "screws": [x for x in (j.screw_lo, j.screw_hi) if x],
            "engage_mm": [j.engage_lo, j.engage_hi], "washers": [j.washers_lo, j.washers_hi],
            "hex_engaged_mm": [j.engaged_lo, j.engaged_hi], "sleeve_mm": j.sleeve,
            "sleeve_press_mm": c.capped_press if press and j.sleeve > 0 else 0.0,
            "run_layers": hi - lo + 1, "layers": [lo - 2, hi + 1],
            "capacity_nm": c.capacity(j)}

    def assembly_issue(self) -> str | None:
        """Why this crank can't be put together, or ``None``: a chain that ends in the hub
        plate with a screw over it (the round standoff's clamp, ``bolt_round``).
        The horn screws come up through the hub plate from below, so the hub plate, the horn,
        the servo and the inner plate go on as one unit (:data:`construction.robot.ASSEMBLY`)
        once the stack under the hub plate is closed, which buries that screw's head under
        the horn spacer; and built the other way round (the hub plate screwed to the chain
        first), the horn screws' heads are inside the closed stack. The assembly audit of
        2026-10-04 (the Strider quad's J1_leg3, ``klann_lego``'s M_leg2)."""
        for n in self.notes:
            if (self.hub_layer is not None and n["layers"][1] == self.hub_layer
                    and not n.get("capped") and len(n["screws"]) > 1):
                return (f"crankpin {n['at']}'s chain ends in the hub plate (layer "
                        f"{self.hub_layer}) with a screw over it ({n['screws'][-1]}), and the "
                        "horn screws come up through the hub plate from below: no order "
                        "drives both (the hub plate, horn, servo and inner plate go on as one "
                        "unit once the stack under it is closed; construction.robot.ASSEMBLY)"
                        "; a crank that caps that chain (the hex crank, --crank bolt) has an "
                        "order, the round standoff's friction clamp needs both screws")
        return None

    def stub(self) -> None:
        c, b = self.c, self.build
        o = self.xy("O")
        if not self.route.bearing or not self.chains:
            return
        k = min(ch[0].lo for ch in self.chains) - 1           # the lowest web
        got = c.stub_z(b.z(k)[0] - b.z(0)[0], b.plan.t(0), self.t)
        if got is None:
            raise ConstructionError(f"no stock stub standoff reaches the outer frame plate from "
                                    f"the lowest web (layer {k})")
        key, _S, z0, L = got
        z0 += b.z(0)[0]
        top = self.pz(k)[0]
        od = c.stub_od()
        st = disc(o, od / 2 - 0.01, z0, top) - disc(o, 1.5, z0 - 1, top + 1)   # M3 thread
        self.buy("crank_stub", st, key, "#c0c0c0")
        sk = SCREWS["bhcs", "3"]
        bearing = self.pz(k)[1]          # the head on the lowest web, from above
        self.buy("crank_stub_screw", screw_solid(o, sk, bearing, L, up=False), sk.key(L))
        self.layer_cut(k, o, 3.4 / 2)
        self.out.cut(FRAME_OUTER, Cut(o, od + 0.6))      # +/-0.3 mm: the journal's clearance
        if c.hex and any(n.get("capped") for n in self.notes):
            # the crank body's stop toward the outer plate: a printed sleeve round the stub
            # from the lowest web's underside to just over the outer plate's inner face,
            # bearing on the plate round the journal's hole
            zb = b.z(0)[1] + c.thrust_play
            ring = (disc(o, c.thrust_od / 2, zb, top)
                    - disc(o, (od + self.p.print_fit) / 2, zb - 1, top + 1))
            self.out.bodies.append(hardware("crank_stub_thrust", ring, self.host,
                                            fab="printed", color=SEGMENT_COLOR))
            self.thrust = {"od_mm": c.thrust_od, "length_mm": round(top - zb, 3),
                           "play_mm": c.thrust_play}

    def hub(self) -> None:
        c, b, drive = self.c, self.build, self.drive
        hub = sorted(pl.layer for pl in b.shapes(GROUP) if pl.label == "crank hub")
        if not hub:
            return
        top = hub[-1]
        seg = top
        spacer = c.horn_spacer(b.plan.layout, drive, top)
        got = c.horn_fit_web(b.ctx, self.t, spacer)
        if got is None:
            raise ConstructionError(f"no screw fits the bolt crank's top plates and the "
                                    f"{b.ctx.servo.key} horn")
        sk, L, _, shim = got
        o = np.asarray(self.xy("O"))
        theta = b.angle("O", self.pins[0].name) + drive.pattern_angle
        face = self.pz(top)[0]
        bearing = face - shim
        shims = shim_stack(shim)
        for i in range(drive.screw_count):
            a = theta + 2 * math.pi * i / drive.screw_count
            xy = tuple(o + drive.screw_pcd / 2 * np.array([math.cos(a), math.sin(a)]))
            for k in range(seg, top + 1):
                self.layer_cut(k, xy, c.horn_hole(b.ctx) / 2)
            self.buy(f"crank_horn_screw{i}", screw_solid(xy, sk, bearing, L), sk.key(L))
            if shims:
                # DIN 988 shims under the head: a stock length too long for the hub plate
                ring = disc(xy, 2.95, bearing, face) - disc(xy, 1.55, bearing - 1, face + 1)
                self.buy(f"crank_horn_shims{i}", ring, SHIM_KEY, "#9a9a9a")
                if len(shims) > 1:
                    self.out.extras.append(BomLine(SHIM_KEY, len(shims) - 1,
                                                   f"horn screw {i}: shims"))
        if drive.center_head_d > 0 and drive.center_head_h > EPS:
            r = (drive.center_head_d + self.p.print_fit) / 2
            face = b.z(top)[1] + spacer
            for k in range(seg, top + 1):
                if b.z(k)[1] > face - drive.center_head_h - 0.2:
                    self.layer_cut(k, tuple(o), r)

    def finish(self) -> Realized:
        c, b = self.c, self.build
        out = self.out
        n_plates = 0
        for k, shapes in sorted(self.plates.items()):
            z = self.pz(k)
            solid = union([shape_solid(b, pl, z=z) for pl in shapes])
            if self.cuts[k]:
                solid = solid - union(self.cuts[k])
            parts = solid.solids()
            solid = parts[0] if len(parts) == 1 else solid
            n_plates += 1
            out.bodies.append(hardware(f"crank_plate{k}", solid, self.host, fab="laser",
                                       color=BOLT_COLOR, sheet=self.sheet))
        self.gap_parts(out)
        for pin in self.pins:
            for m in pin.members:
                out.cut(m, Cut(self.xy(pin.name), self.p.hole(self.c.rider_d())))
        out.notes["crank_bolt"] = {
            "webs": "single",
            "crankpin": ("hex standoff in hex pockets, the riders on a printed sleeve"
                         if c.hex else "round standoff clamped by M4 screws"),
            "chains": self.notes, "journals": self.journals, "plates": n_plates,
            "stub_thrust": getattr(self, "thrust", None),
            "assembly": self.assembly_issue(),
            # (the acrylic stacks' keys, kept empty so stored audits compare)
            "segments": [], "sheet_mm": self.t,
            "weakest_bond": None, "threadlocker": c.lock_key}
        if c.hex:
            out.notes["crank_bolt"].update({
                "pocket_af_mm": round(c.hex_pocket_af(), 3), "dogbone_r_mm": c.dogbone_r,
                "play_deg": round(2 * hex_play(c.hex_af, c.hex_pocket_af()), 2)})
        return out
