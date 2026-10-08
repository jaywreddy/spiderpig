"""The single-plate webs' fit (:class:`BoltCrank`): the round standoff's clamp, a
chain's spans, the stub and the horn screws through the hub plate."""

from __future__ import annotations

import functools
import math
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

from spiderpig.construction.base import (
    ConstructionError,
    Context,
    DriveInterface,
)
from spiderpig.construction.crank.base import EPS
from spiderpig.hardware.fasteners import SCREWS, SIZES, Screw
from spiderpig.stack import Layout

if TYPE_CHECKING:
    from spiderpig.construction.crank.hex import HexJoint


SHIM_KEY = "shim_din988_3x6"     # under a horn screw's head (M2 and M3: the head bears on it)
HORN_TIP_CLEAR = 0.3             # a horn screw's tip under the inner plate's top face (mm)


def shim_stack(t: float) -> list[float]:
    """DIN 988 3 x 6 shims making up ``t`` mm (0.1 mm steps), thickest first."""
    from spiderpig.hardware.bom import stack
    from spiderpig.hardware.catalog import get

    return stack(t, get(SHIM_KEY).dims["t"])[0]


@functools.cache
def _pin_lengths() -> tuple[tuple[float, ...], ...]:
    """A crankpin's standoffs, shortest first: one stock goBILDA 1501 length, or two (each
    at least 12 mm: thread for the stud and a screw) joined by an M4 set screw."""
    from spiderpig.hardware.crank_catalog import GOBILDA_LENGTHS

    one = [(float(a),) for a in GOBILDA_LENGTHS]
    two = [(float(a), float(b)) for a in GOBILDA_LENGTHS for b in GOBILDA_LENGTHS
           if a >= b >= 12 and a + b > max(GOBILDA_LENGTHS)]
    return tuple(sorted(one + two, key=lambda c: (sum(c), len(c))))


@dataclass(frozen=True)
class WebJoint:
    """One crankpin between single-plate webs (:meth:`BoltCrank.fit_web`): a round standoff
    clamped between its two webs by an M4 screw into each end."""

    standoff: str        # catalog key (a stock length)
    length: float
    shims: float         # DIN 988 shims under its lower end (mm)
    gap: float           # the clearance gap over the lowest web its length needs (mm)
    screw_lo: str        # the M4 screw up through the lowest web (and the shims)
    engage_lo: float
    screw_hi: str        # the M4 screw down through the top web
    engage_hi: float
    segments: tuple[float, ...] = ()   # stock lengths, bottom up (two: joined by a stud)




class WebFitMixin:
    """The webs', stub's and horn screws' fit, for :class:`BoltCrank`."""

    if TYPE_CHECKING:       # what it reads of :class:`BoltCrank` (its fields and methods)
        hex: bool
        head_clear: float
        horn_shim_max: float
        pin_min_engage: float
        pin_od: float
        sleeve_od: float

        @staticmethod
        def plate_z(L: Layout, k: int, t: float, hub: int | None = None
                    ) -> tuple[float, float]: ...

        def stub_z(self, top: float, plate: float, upper: float
                   ) -> tuple[str, float, float, float] | None: ...

        # (HexFitMixin's)
        @staticmethod
        def hex_screw() -> tuple[float, float]: ...

        @staticmethod
        def hex_washer() -> tuple[float, float, float]: ...

        def air_over(self, L: Layout, k: int, t: float, hub: int | None = None) -> float: ...

        def fit_hex(self, span: float, t_lo: float, t_hi: float, sleeve: bool = True,
                    out_hi_max: float | None = None, capped: bool = False,
                    air_hi: float = 0.0) -> HexJoint | None: ...

        def hex_gap_fit(self, span: float, slots: list[tuple[int, float]], t_lo: float,
                        t_hi: float, sleeve: bool = True, out_hi_max: float | None = None,
                        capped: bool = False, air_hi: float = 0.0) -> HexJoint | None: ...

    def pin_screw(self) -> tuple[float, float]:
        """(head diameter, head height) of the crankpins' M4 button heads."""
        from spiderpig.hardware.catalog import get
        from spiderpig.hardware.crank_catalog import M4_BHCS_LENGTHS, m4_bhcs

        d = get(m4_bhcs(M4_BHCS_LENGTHS[0])).dims
        return float(d["head_d"]), float(d["head_h"])

    def head_r(self) -> float:
        """A crankpin screw head's clearance shape (the hex pin's: its wide washer)."""
        if self.hex:
            return max(self.hex_screw()[0], self.hex_washer()[1]) / 2 + 0.3
        return self.pin_screw()[0] / 2 + 0.3

    def rider_d(self, params=None) -> float:
        """What the riders turn on: the hex pin's printed sleeve, else the round standoff
        (``params`` unused: the crank's own)."""
        return self.sleeve_od if self.hex else self.pin_od

    def _pin_screw(self, through: float, depth: float) -> tuple[str, float] | None:
        """The M4 button head through ``through`` mm (a web, and shims) into a standoff's
        end tapped ``depth`` deep: the most thread, (key, engagement)."""
        from spiderpig.hardware.crank_catalog import M4_BHCS_LENGTHS, m4_bhcs

        best = None
        for L in M4_BHCS_LENGTHS:
            e = L - through
            if self.pin_min_engage - EPS <= e <= depth + EPS and (best is None or e > best[1]):
                best = (m4_bhcs(L), round(e, 3))
        return best

    def fit_web(self, free: float, gap: float, t_lo: float, t_top: float) -> WebJoint | None:
        """The crankpin of a chain of single-plate webs: ``free`` mm between the lowest web's
        top face and the top web's bottom face, not counting the clearance gap over the
        lowest web (``gap``). The shortest stock standoff at least ``free`` long; the gap
        over the lowest web it needs (its length past ``free``), the rest of that gap taken
        up by DIN 988 shims under its end (0.1 mm steps); an M4 button head into each end
        with at least ``pin_min_engage`` of thread."""
        from spiderpig.hardware.catalog import get
        from spiderpig.hardware.crank_catalog import gobilda_1501
        from spiderpig.stack import GAP_MAX

        for segs in _pin_lengths():
            S = sum(segs)
            if free - EPS > S:
                continue
            need = max(0.0, S - free)
            if need > GAP_MAX + EPS:
                return None
            shims = round(max(0.0, free + gap - S), 1)
            d_lo = float(get(gobilda_1501(segs[0])).dims["thread_depth"])
            d_hi = float(get(gobilda_1501(segs[-1])).dims["thread_depth"])
            lo = self._pin_screw(t_lo + shims, d_lo)
            hi = self._pin_screw(t_top, d_hi)
            if lo is None or hi is None:
                continue
            return WebJoint(gobilda_1501(segs[0]), S, shims, round(need, 3), lo[0], lo[1],
                            hi[0], hi[1], segs)
        return None

    def chain_fit_web(self, L: Layout, lo: int, hi: int, low: int = 0, t: float | None = None,
                      hub: int | None = None, sleeve: bool = True, capped: bool = False
                      ) -> WebJoint | HexJoint | None:
        """:meth:`fit_web` for a chain over runs ``lo``..``hi`` at the layout's z: its webs
        in ``lo - 1`` and ``hi + 1``, plates ``t`` thick (default: the layers'). Its
        ``gap`` is what the standoff needs of the clearance gap over the lowest web once the
        air over that plate is used."""
        w0, w1 = lo - 1, hi + 1
        t0 = L.t(w0) if t is None else t
        t1 = L.t(w1) if t is None else t
        if self.hex:
            span = self.plate_z(L, w1, t1, hub)[1] - self.plate_z(L, w0, t0, hub)[0]
            # over the hub plate the upper end stands in the horn spacer's pocket, or (capped)
            # ends in the hub plate under the spacer
            hi_max = 0.0 if capped or w1 == hub else None
            air = self.air_over(L, w1, t1, hub)
            j = self.fit_hex(span, t0, t1, sleeve=sleeve, out_hi_max=hi_max, capped=capped,
                             air_hi=air)
            if j is not None:
                return j
            # the gaps over the lowest web and along the run (a journal: over its lowest
            # web only), those the plan has first
            ks = [w0] + (list(range(lo, hi + 1)) if sleeve else [])
            ks.sort(key=lambda k: (L.gap(k) <= 0, k))
            return self.hex_gap_fit(span, [(k, L.gap(k)) for k in ks], t0, t1, sleeve,
                                    hi_max, capped, air)
        top_face = self.plate_z(L, w0, t0, hub)[1]
        bottom = self.plate_z(L, w1, t1, hub)[0]
        floor = L.z(lo)[0]
        j = self.fit_web(bottom - floor, floor - top_face, t0, t1)
        if j is None:
            return None
        air = L.z(w0)[1] - top_face
        return replace(j, gap=round(max(0.0, j.gap - air), 3))

    def _web_span_ok(self, run: int, low: int, pitch: float, t: float) -> bool:
        """Whether a chain whose webs are ``run`` layers apart takes a stock standoff at the
        nominal z or up to an aluminium plate's thickness more (the exact z decides)."""
        air = max(0.0, pitch - t)
        if self.hex:
            # outer faces: the lower web on its layer's floor, the upper's top face, the run's
            # layers between (and up to two clearance gaps the plan may put among them)
            span = max(pitch, t) + run * pitch + t
            # (two webs in adjacent layers: no gap between them, nothing short enough joins
            # them, as the screws into a 5-6 mm standoff's ends would meet)
            return any(self.fit_hex(span + dz, t, t) is not None
                       for dz in ((0.0, 1.0, 1.6, 2.0, 2.6, 3.2) if run else (0.0,)))
        for dz in (0.0, run * 0.175, run * 0.175 + 1.0, air):
            if self.fit_web(run * pitch + dz, air, t, t) is not None:
                return True
        return False

    def stub_layers_web(self, t0: float, pitch: float, t: float, most: int = 64
                        ) -> frozenset[int]:
        """The layers the first chain's lowest web may sit in: the stub standoff from its
        underside down into the outer frame plate (screwed to it from above)."""
        return frozenset(a for a in range(2, most)
                         if self.stub_z(t0 + (a - 1) * pitch, t0, t) is not None)

    def horn_joint_web(self, ctx: Context, seg: float, spacer: float
                       ) -> tuple[Screw, float, float] | None:
        """:meth:`horn_fit_web` without its shims: (kind, length, thread in the horn)."""
        got = self.horn_fit_web(ctx, seg, spacer)
        return None if got is None else got[:3]

    def horn_fit_web(self, ctx: Context, seg: float, spacer: float
                     ) -> tuple[Screw, float, float, float] | None:
        """The horn screws up through the crank's top plates (``seg`` mm: the hub and what
        is under it, their heads under the lowest) and the horn spacer (``spacer``) into
        the horn: (kind, length, thread in the horn, DIN 988 shims under the head). Most
        thread, then no shims, then shortest. A stock length too long for the plates (a
        horn whose thread window is short, the XL430's 1.5-2.0 mm, against a thin hub
        plate) takes 0.1 mm steps of shims under its head (up to ``horn_shim_max``, in the
        gap under the hub where its head hangs)."""
        from spiderpig.stack import GAP_MAX

        spec = ctx.servo
        pat = spec.horn.pattern
        size = SIZES.get(pat.thread, "")        # (no size: no screw of the kinds below)
        order = ("self_tap",) if pat.tapping else ("bhcs", "shcs")
        kinds = [SCREWS[(k, size)] for k in order if (k, size) in SCREWS]
        reach = pat.reach if pat.reach is not None else spec.horn.thickness
        # the tip stays HORN_TIP_CLEAR under the inner plate's top face (inside the horn)
        e_max = min(reach, spec.horn_face_depth - HORN_TIP_CLEAR)
        e_want = min(pat.thread_depth if pat.thread_depth is not None else e_max, e_max)
        e_min = min(e_want, 1.5)
        for sk in kinds:
            best = None
            for L in sk.lengths:
                e = L - seg - spacer
                shim = 0.0
                if e > e_max + EPS:
                    # shims to the wanted thread, else to the most the horn takes; no more
                    # than the gap under the hub holds over the head
                    most = min(self.horn_shim_max,
                               GAP_MAX - sk.head_h - self.head_clear)
                    shim = math.ceil((e - e_want) * 10 - 1e-6) / 10
                    whole = float(math.ceil(e - e_max - 1e-6))
                    if e - whole >= e_min - EPS:
                        shim = whole       # whole 1 mm shims (DIN 433 pairs), thread to e_max
                    if shim > most + EPS:
                        shim = math.ceil((e - e_max) * 10 - 1e-6) / 10
                    if shim > most + EPS:
                        continue
                    e -= shim
                if not e_min - EPS <= e <= e_max + EPS:
                    continue
                rank = (-min(e, e_want), shim, L)
                if best is None or rank < best[0]:
                    best = (rank, (sk, L, round(e, 3), round(shim, 2)))
            if best is not None:
                return best[1]
        return None

    @staticmethod
    def horn_hole(ctx: Context) -> float:
        """A horn screw's hole in the hub plate: its clearance hole, or the least hole the
        service cuts in the crank's sheet (SendCutSend: the thickness; an M2 head still
        bears on the ring round it)."""
        drive: DriveInterface = ctx.interfaces["drive"]
        key = ctx.sheet("crank")
        if key is None:
            return drive.screw_clearance_d
        from spiderpig.materials import sheet

        return max(drive.screw_clearance_d, sheet(key).min_hole + 0.025)

    def horn_head_r(self, ctx: Context, shim: float | None = None) -> float:
        """A horn screw head's clearance shape under the hub: the head, or with ``shim`` mm
        of DIN 988 shims under it (6 mm OD) the wider of them; ``None``: whether the plan's
        nominal z needs shims."""
        from spiderpig.hardware.catalog import get

        if shim is None:
            drive: DriveInterface = ctx.interfaces["drive"]
            got = self.horn_fit_web(ctx, ctx.sheet_t("crank"), drive.spacer_t)
            shim = got[3] if got is not None else 0.0
        d = self.horn_kind(ctx).head_d
        if shim > 0:
            d = max(d, float(get(SHIM_KEY).dims["od"]))
        return d / 2 + 0.3

    def horn_kind(self, ctx: Context) -> Screw:
        """The horn screws' kind (its head is what hangs under the crank's top plates)."""
        pat = ctx.servo.horn.pattern
        size = SIZES.get(pat.thread, "")        # (no size: no screw of the kinds below)
        order = ("self_tap",) if pat.tapping else ("bhcs", "shcs")
        kinds = [SCREWS[(k, size)] for k in order if (k, size) in SCREWS]
        if not kinds:
            raise ConstructionError(f"no screw for the {ctx.servo.key} horn's {pat.thread}")
        return kinds[0]

    def horn_points(self, ctx: Context) -> list[str]:
        """The horn screws as points fixed to the crank (added to the side's geometry)."""
        drive: DriveInterface = ctx.interfaces["drive"]
        topo = ctx.topo
        out = []
        for i in range(drive.screw_count):
            name = f"crank.horn{i}"
            if name not in topo.crank_points:
                a = math.degrees(drive.pattern_angle + 2 * math.pi * i / drive.screw_count)
                topo.add_crank_point(name, drive.screw_pcd / 2, a)
            out.append(name)
        return out

    def horn_spacer(self, L: Layout, drive: DriveInterface, hub: int) -> float:
        """The printed horn spacer at the plan's z: over the hub plate up to the horn's face."""
        return L.z(L.top)[1] - L.z(hub)[1] - (drive.horn_face_depth - drive.spacer_t)
