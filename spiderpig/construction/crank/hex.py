"""The hex-standoff crankpin's fit (:class:`BoltCrank`, ``pin="hex"``): its stock
length, pockets, screws, washers, collars and the gaps it needs."""

from __future__ import annotations

import functools
import math
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

from spiderpig.construction.crank.base import EPS, _hex
from spiderpig.hardware.fasteners import SCREWS
from spiderpig.shapes import disc
from spiderpig.stack import Layout

if TYPE_CHECKING:
    from spiderpig.construction.crank.bolt import BoltCrank


@dataclass(frozen=True)
class HexJoint:
    """One crankpin (or journal) of single-plate webs on a hex standoff
    (:meth:`BoltCrank.fit_hex`): its hex ends in hex pockets through its two webs, an M3
    button head and a wide washer screwed into each end. ``span`` is the distance between
    the two plates' outer faces; the standoff is at least that long, and what it stands past
    a plate's outer face (``out_lo`` / ``out_hi``, into the clearance gap where that end's
    screw head is) carries a printed hex collar of that thickness (``collar_*``, none under
    ``BoltCrank.collar_min``: the plate floats that much), so the washer bears on the
    standoff's end and the collar (or the plate) together."""

    standoff: str        # catalog key (a stock length)
    length: float
    span: float          # the plates' outer faces apart (mm)
    out_lo: float        # the standoff past the lower plate's outer face (mm)
    out_hi: float        # ... past the upper plate's
    collar_lo: float     # printed hex collars taking those up (0: none)
    collar_hi: float
    screw_lo: str        # the M3 button head into the lower end
    engage_lo: float
    washers_lo: int      # DIN 125 washers under the head besides the DIN 9021 one
    screw_hi: str
    engage_hi: float
    washers_hi: int
    engaged_lo: float    # the hex in each plate's pocket (mm)
    engaged_hi: float
    sleeve: float        # the printed sleeve the riders turn on (0: a journal, none)
    gap: float = 0.0     # the clearance gap over the lowest web its length needs (mm; 0:
    #                      a stock length fits as the plan stands, BoltCrank.hex_gap_fit)
    stack_lo: float = 0.0   # what hangs under the lower plate: standoff end, washers, head
    stack_hi: float = 0.0   # ... over the upper plate
    segments: tuple[float, ...] = ()
    gaps: tuple[tuple[int, float], ...] = ()   # (gap slot, mm) the length needs in all




class HexFitMixin:
    """The hex crankpin's fit, for :class:`BoltCrank`."""

    def hex_lengths(self) -> tuple[float, ...]:
        from spiderpig.hardware.crank_catalog import HEX_M3_LENGTHS

        return tuple(float(x) for x in HEX_M3_LENGTHS)

    @staticmethod
    def hex_key(length: float) -> str:
        from spiderpig.hardware.crank_catalog import hex_standoff_m3

        return hex_standoff_m3(length)

    @staticmethod
    def hex_screw() -> tuple[float, float]:
        """(head diameter, head height) of the hex pins' M3 button heads (ISO 7380)."""
        return SCREWS["bhcs", "3"].head_d, SCREWS["bhcs", "3"].head_h

    @staticmethod
    def hex_washer() -> tuple[float, float, float]:
        """(id, od, t) of the wide washer under each hex pin screw (DIN 9021 M3)."""
        from spiderpig.hardware.catalog import get

        d = get("m3_washer_9021").dims
        return float(d["id"]), float(d["od"]), float(d["t"])

    @staticmethod
    def extra_washer_t() -> float:
        from spiderpig.hardware.catalog import get

        return float(get("m3_washer").dims["t"])

    def hex_pocket_af(self) -> float:
        return self.hex_af + self.hex_fit

    def hex_reach(self) -> float:
        """A hex pocket's reach from its centre: a corner plus its dog-bone relief."""
        return self.hex_pocket_af() / math.sqrt(3) + 2 * self.dogbone_r

    def hex_cut(self, xy, z0: float, z1: float, angle: float):
        """A hex pocket with a dog-bone relief at each corner: a ``dogbone_r`` circle through
        the corner, centred out along its bisector (the flats keep their whole length; the
        service's inside radius is the circle's)."""
        af = self.hex_pocket_af()
        cut = _hex(xy, af, z0, z1, angle)
        rc = af / math.sqrt(3)
        for i in range(6):
            a = angle + math.pi / 6 + i * math.pi / 3
            # 0.05 mm over the corner, so the pocket is one outline (a circle through the
            # corner only touches it): each flat 0.09 mm shorter at each end, within the
            # rating's corner loss
            r = rc + self.dogbone_r - 0.05
            cut = cut + disc((float(xy[0]) + r * math.cos(a), float(xy[1]) + r * math.sin(a)),
                             self.dogbone_r, z0, z1)
        return cut

    def hex_screws(self, length: float) -> tuple[float, int, float] | None:
        """The screw into each end of a ``length`` mm standoff (tapped through): (its stock
        length, DIN 125 washers under its head besides the wide one, thread in the
        standoff): the most thread up to ``hex_engage_max`` that leaves ``hex_tip_gap``
        between the two tips, fewest washers first."""
        _, _, wt = self.hex_washer()
        et = self.extra_washer_t()
        cap = min(self.hex_engage_max, (length - self.hex_tip_gap) / 2)
        best = None
        for k in (0, 1, 2):
            for L in SCREWS["bhcs", "3"].lengths:
                e = L - wt - k * et
                if (self.hex_min_engage - EPS <= e <= cap + EPS
                        and (best is None or e > best[2] + EPS)):
                    best = (float(L), k, round(e, 3))
            if best is not None:
                return best
        return None

    def hex_stack(self, out: float, washers: int) -> float:
        """What a hex pin's end holds past its plate's outer face: the standoff's end (or its
        collar), the washers and the screw head."""
        return out + self.hex_washer()[2] + washers * self.extra_washer_t() + self.hex_screw()[1]

    def fit_hex(self, span: float, t_lo: float, t_hi: float, sleeve: bool = True,
                out_hi_max: float | None = None, capped: bool = False,
                air_hi: float = 0.0) -> HexJoint | None:
        """:meth:`_fit_hex`, remembered (the planner asks it at every node's layouts)."""
        return _fit_hex(self, round(span, 6), round(t_lo, 6), round(t_hi, 6), bool(sleeve),
                        out_hi_max, bool(capped), round(max(air_hi, 0.0), 6))

    def _fit_hex(self, span: float, t_lo: float, t_hi: float, sleeve: bool = True,
                 out_hi_max: float | None = None, capped: bool = False,
                 air_hi: float = 0.0) -> HexJoint | None:
        """The hex standoff between two single-plate webs whose outer faces are ``span``
        apart (plates ``t_lo`` and ``t_hi`` thick): the stock length nearest ``span`` that
        either stands past the plates by what the two ends' stacks take (``protrude_max``
        each, the lower end first, ``out_hi_max`` over the upper plate; each stack within a
        clearance gap: the washer then bears on the standoff's end and a printed collar)
        or ends inside both pockets by at most ``recess_max`` (the washers then clamp the
        plates on the printed sleeve, as long as the inner faces are apart, and the hex is
        in that much less of each pocket). Flush or standing past first, then shortest.
        What stands past is split between the ends so the taller of the two stacks' gaps is
        the least (``air_hi``: the air over the upper plate in its own layer, which its stack
        uses first; since the debug of 2026-10-05, before: the lower end first, which made
        every crankpin gap under a chain 4 mm and pushed the Strider decker's Chicago pins
        off their stock barrels)."""
        from spiderpig.stack import GAP_MAX

        room = GAP_MAX - self.head_clear
        hi_max = self.protrude_max if out_hi_max is None else out_hi_max
        best: tuple | None = None
        for S in self.hex_lengths():
            extra = S - span
            if extra < -2 * self.recess_max - EPS:
                continue
            if extra > self.protrude_max + hi_max + EPS:
                break
            scr = self.hex_screws(S)
            if scr is None:
                continue
            L, k, e = scr
            if extra >= -EPS:
                x = max(extra, 0.0)
                lo = min(x, self.protrude_max)
                splits = [(lo, x - lo)]
                if hi_max > 0:
                    # the upper end as far as evens the two gaps' needs (its stack has the
                    # air over its plate first), else shared
                    up = min(max((x + air_hi) / 2, 0.0), x, hi_max)
                    splits += [(x - up, up), (x / 2, x / 2)]
            else:
                splits = [(extra / 2, extra / 2)]       # recessed in both pockets
            ok = [(max(o_lo, o_hi - air_hi), i, o_lo, o_hi)
                  for i, (o_lo, o_hi) in enumerate(splits)
                  if not (o_lo > self.protrude_max + EPS or o_hi > hi_max + EPS
                          or self.hex_stack(max(o_lo, 0.0), k) > room + EPS
                          or self.hex_stack(max(o_hi, 0.0), k) > room + EPS)]
            if ok:
                _, _, out_lo, out_hi = min(ok)
                rank = (extra < -EPS, abs(extra))
                if best is None or rank < best[0]:
                    best = (rank, S, out_lo, out_hi, L, k, e)
        if best is None:
            return None
        _, S, out_lo, out_hi, L, k, e = best

        def collar(o: float) -> float:
            # to 0.01 mm, never past the standoff's end (rounded up, it stood into the
            # washer on that end: klann_lego double's 0.165 mm^3 clash, r4 2026-10-05)
            return math.floor(o * 100 + 1e-6) / 100 if o >= self.collar_min - EPS else 0.0

        inner = span - t_lo - t_hi
        # standing past: the plates captured between the washers and the sleeve, which is
        # short of them by its play; recessed: the washers clamp the plates on the sleeve
        sl = inner - (self.sleeve_play if out_lo >= -EPS else 0.0)
        key = SCREWS["bhcs", "3"].key(L)
        return HexJoint(
            self.hex_key(S), S, round(span, 3), round(out_lo, 3), round(out_hi, 3),
            collar(out_lo), collar(out_hi), key, e, k,
            "" if capped else key, 0.0 if capped else e, 0 if capped else k,
            round(t_lo + min(out_lo, 0.0), 3),
            # capped: the crank body's float toward the outer plate (the thrust sleeve's
            # play) draws the hex that much out of the hub plate's pocket
            round(t_hi + min(out_hi, 0.0) - (self.thrust_play if capped else 0.0), 3),
            round(sl, 3) if sleeve else 0.0,
            stack_lo=round(self.hex_stack(max(out_lo, 0.0), k), 3),
            stack_hi=0.0 if capped else round(self.hex_stack(max(out_hi, 0.0), k), 3),
            segments=(S,))

    def hex_gap_fit(self, span: float, slots: list[tuple[int, float]], t_lo: float,
                    t_hi: float, sleeve: bool = True, out_hi_max: float | None = None,
                    capped: bool = False, air_hi: float = 0.0) -> HexJoint | None:
        """:meth:`_hex_gap_fit`, remembered."""
        return _hex_gap_fit(self, round(span, 6), tuple((k, round(h, 6)) for k, h in slots),
                            round(t_lo, 6), round(t_hi, 6), bool(sleeve), out_hi_max,
                            bool(capped), round(air_hi, 6))

    def _hex_gap_fit(self, span: float, slots: tuple[tuple[int, float], ...], t_lo: float,
                     t_hi: float, sleeve: bool = True, out_hi_max: float | None = None,
                     capped: bool = False, air_hi: float = 0.0) -> HexJoint | None:
        """A hex standoff for a chain no stock length fits at ``span``: the clearance gaps
        ``slots`` (``(slot, mm now)``, in the order to use them: the chain's own gap slots,
        from its lowest web's up) opened or thickened, in 0.1 mm steps each up to
        :data:`stack.GAP_MAX`, by the least that one fits (its ``gaps``: the heights it
        claims there; printed rings on the sleeve fill them, :meth:`web_claims`; ``gap``:
        the one over the lowest web). The round standoff's rule (:meth:`fit_web`'s ``gap``)
        for the hex, whose stock lengths are 5 mm apart past 25 mm: no length fits a span
        in 25.7-27.6 mm (25.7-28.8 capped in the hub plate), which kept the Strider decker
        and quad off the hex crank (the debug of 2026-10-05)."""
        from spiderpig.stack import GAP_MAX, GAP_STEP

        room = sum(max(0.0, GAP_MAX - h) for _, h in slots)
        for i in range(1, int(room / GAP_STEP + 1e-6) + 1):
            dz = i * GAP_STEP
            j = self.fit_hex(span + dz, t_lo, t_hi, sleeve=sleeve, out_hi_max=out_hi_max,
                             capped=capped, air_hi=air_hi)
            if j is None:
                continue
            left, gaps = dz, []
            for k, h in slots:
                take = min(left, max(0.0, GAP_MAX - h))
                if take > EPS:
                    gaps.append((k, round(h + take, 3)))
                    left -= take
            w0 = slots[0][0]
            return replace(j, gap=dict(gaps).get(w0, 0.0), gaps=tuple(sorted(gaps)))
        return None

    def hex_gap_needed(self, L: Layout, lo: int, hi: int, t: float, hub: int | None,
                       capped: bool) -> bool:
        """Whether the clearance gap the plan has over a hex chain's lowest web (layer
        ``lo - 1``) is one its standoff needs: no stock length fits the span without it
        (:meth:`hex_gap_fit` opened it)."""
        w0, w1 = lo - 1, hi + 1
        g = L.gap(w0)
        if not self.hex or g <= 0:
            return False
        span = self.plate_z(L, w1, t, hub)[1] - self.plate_z(L, w0, t, hub)[0] - g
        hi_max = 0.0 if capped or w1 == hub else None
        return self.fit_hex(span, t, t, out_hi_max=hi_max, capped=capped,
                            air_hi=self.air_over(L, w1, t, hub)) is None

    def air_over(self, L: Layout, k: int, t: float, hub: int | None = None) -> float:
        """The air over the crank plate in layer ``k`` inside its own layer (a plate thinner
        than its layer sits on the floor; the hub plate at the top: none): a crankpin's
        stack over that plate uses it before the clearance gap above."""
        return max(0.0, L.z(k)[1] - self.plate_z(L, k, t, hub)[1])


@functools.lru_cache(maxsize=1 << 16)
def _fit_hex(c: BoltCrank, *args) -> HexJoint | None:
    return c._fit_hex(*args)


@functools.lru_cache(maxsize=1 << 14)
def _hex_gap_fit(c: BoltCrank, *args) -> HexJoint | None:
    return c._hex_gap_fit(*args)
