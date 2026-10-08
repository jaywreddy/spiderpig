"""The plan's z: clearance gaps, sunk heads, layer thicknesses (:func:`finalize`)."""


from __future__ import annotations

import heapq
import itertools
import math
from collections.abc import Iterable, Mapping
from dataclasses import replace
from typing import TYPE_CHECKING

from spiderpig.stack.geometry import Disc, Layout, Pill, Placed, Unbuildable, made
from spiderpig.stack.plan import StackPlan

if TYPE_CHECKING:
    from spiderpig.stack.geometry import Claim, Geometry, Shape
    from spiderpig.stack.plan import StackSpec
    from spiderpig.stack.topology import Topology


# ---------------------------------------------------------------------------
# The plan's z: clearance gaps, sunk heads, layer thicknesses
# ---------------------------------------------------------------------------


class PlanReject(ValueError):
    """A layering whose plan can't be built at its own z (:func:`finalize`): a claim a
    clearance gap or a thicker plate moved can't be built, or a head needs a gap no stock
    sheet is thick enough for. The search takes it as a dead end."""


EPS_Z = 1e-6
SunkKey = tuple


def sunk_key(p: Placed) -> SunkKey:
    return (p.group, p.label, p.layer, p.shape.core, p.toward)


def _hit(geo: Geometry, a: Shape, b: Shape, margin: float) -> bool:
    return geo.dist(a.core, b.core) < a.r + b.r + margin - 1e-9


def bridges(shapes: Iterable[Placed], gaps: Mapping[int, float] | None = None) -> list[Placed]:
    """What runs on through a clearance gap: a group's piece (one core) in the layers on
    both sides of it is in the gap too, at the narrower of the two (a crank stack, a
    hub, a post); a group that put shapes of its own at that core in the gap (an axle's
    washers) is left as it said. Only the gaps in ``gaps`` (all, when ``None``)."""
    cores: dict[tuple, dict[int, float]] = {}
    own = set()
    for p in shapes:
        if p.gap:
            own.add((p.group, p.shape.core, p.layer))
            continue
        d = cores.setdefault((p.group, p.shape.core), {})
        r, t = d.get(p.layer, (0.0, math.inf))
        d[p.layer] = (max(r, p.shape.r), min(t, p.sheet))
    out = []
    for (g, core), d in cores.items():
        for k, (r, t) in sorted(d.items()):
            if k + 1 not in d or (g, core, k) in own:
                continue
            if gaps is not None and not gaps.get(k):
                continue
            rr = min(r, d[k + 1][0])
            shape = Disc(core[1], rr) if core[0] == "pt" else Pill(core[1], core[2], rr)
            # a plate stack on both sides (``sheet``: the thinner): a filler plate fills it
            out.append(Placed(k, shape, g, f"{g} through the gap", gap=True,
                              sheet=min(t, d[k + 1][1])))
    return out


def settle(shapes: Iterable[Placed], sunk: Iterable[SunkKey], layout: Layout) -> list[Placed]:
    """The claims' shapes as the plan builds them: the heads in ``sunk`` moved into the layer
    they sink into, the shapes of a gap the plan doesn't have dropped, and what runs on
    through the gaps it has added (:func:`bridges`)."""
    sunk = set(sunk)
    out = []
    for p in shapes:
        if p.gap and p.toward and sunk_key(p) in sunk:
            out.append(replace(p, layer=p.layer + (1 if p.toward > 0 else 0), gap=False))
        elif p.gap and not layout.gap(p.layer):
            continue
        else:
            out.append(p)
    return out + bridges(out, layout.gaps)


def _sinkable(shapes: list[Placed], layout: Layout, geo: Geometry, margin: float
              ) -> set[SunkKey]:
    """The heads that go into the layer beside their gap instead (the full-layer head):
    a gap goes when every head in it fits the layer it would sink into (not a frame
    plate, tall enough, clear of every other group's shape there and of the heads
    already sunk into it), going up the stack."""
    top = layout.top
    plate: dict[int, list[Placed]] = {}
    heads: dict[int, list[Placed]] = {}
    for p in shapes:
        if p.seat:
            continue
        if not p.gap:
            plate.setdefault(p.layer, []).append(p)
        elif p.height > 0:
            heads.setdefault(p.layer, []).append(p)
    sunk: set[SunkKey] = set()
    for b in sorted(heads):
        moves = []
        for h in heads[b]:
            tgt = b + 1 if h.toward > 0 else b if h.toward < 0 else None
            if tgt is None or not 0 < tgt < top or h.height > layout.t(tgt) + EPS_Z:
                break
            if any(q.group != h.group and _hit(geo, h.shape, q.shape, margin)
                   for q in [*plate.get(tgt, ()), *(m for t, m in moves if t == tgt)]):
                break
            moves.append((tgt, h))
        else:
            for tgt, h in moves:
                plate.setdefault(tgt, []).append(h)
                sunk.add(sunk_key(h))
    return sunk


def _thicknesses(layers: Mapping[str, int], shapes: Iterable[Placed], spec: StackSpec,
                 top: int) -> dict[int, float]:
    """Each layer's thickness where it isn't the pitch: the frame plates', the thickest
    link's or plate's in it."""
    t: dict[int, float] = {}
    if spec.frame_t is not None:
        t[0] = t[top] = spec.frame_t
    lt = dict(spec.link_t)
    for n, k in layers.items():
        if n in lt:
            t[k] = max(t.get(k, spec.pitch), lt[n])
    for p in shapes:
        if p.sheet > 0 and not p.gap and 0 < p.layer < top:
            t[p.layer] = max(t.get(p.layer, spec.pitch), p.sheet)
    return {k: v for k, v in t.items() if abs(v - spec.pitch) > EPS_Z}


def _gap_options(k: int, h: float, spec: StackSpec, bridged: set[int]) -> list[float]:
    """The thicknesses gap ``k`` may have for heads ``h`` tall, thinnest first: a thin
    sheet's where plates go on through it (``bridged``), else any :data:`GAP_STEP` up to
    :data:`GAP_MORE` over the need."""
    if k in bridged:
        return [o for o in sorted(spec.gaps) if o >= h - EPS_Z]
    if h > GAP_MAX + EPS_Z:
        return []
    g = math.ceil(h / GAP_STEP - 1e-6) * GAP_STEP
    return [round(g + i * GAP_STEP, 3) for i in range(int(GAP_MORE / GAP_STEP) + 1)
            if g + i * GAP_STEP <= max(GAP_MAX, g) + EPS_Z]


def _gap_sizes(shapes: Iterable[Placed], spec: StackSpec, top: int,
               bridged: set[int] = frozenset()) -> dict[int, float]:
    """Each clearance gap the heads in it need: the thinnest it may be over the tallest
    (:func:`_gap_options`; :class:`PlanReject` when none is tall enough)."""
    need: dict[int, float] = {}
    for p in shapes:
        if p.gap and p.height > 0:
            need[p.layer] = max(need.get(p.layer, 0.0), p.height)
    out = {}
    for k, h in need.items():
        if not 0 <= k < top:
            raise PlanReject(f"a clearance gap over layer {k} is outside the frame plates")
        g = next(iter(_gap_options(k, h, spec, bridged)), None)
        if g is None:
            who = sorted({p.label or p.group for p in shapes
                          if p.gap and p.layer == k and p.height > h - EPS_Z})
            most = max(spec.gaps) if k in bridged else GAP_MAX
            raise PlanReject(f"{', '.join(who)} needs a {h:.2f} mm clearance gap over layer "
                             f"{k}, more than the thickest it may have ({most:g} mm)")
        out[k] = g
    return out


MEMO_MAKES = 200_000     # claim makes a problem's plans remember before the memo is dropped


def _make_all(claims: Iterable[Claim], layout: Layout, memo: dict | None = None
              ) -> list[Placed]:
    """Every claim made in ``layout`` (:class:`PlanReject` naming the first that can't be).
    ``memo``: what each claim made before, by what it reads of a layout (its deps' layers,
    or every link's for a final claim, the stack size, the z and the choices), shared by
    the plans of one problem (whose claims it is only given for: they outlive it, so
    their ids are theirs), as the search's leaves re-make mostly the same claims."""
    out: list[Placed] = []
    if memo is not None:
        if len(memo) >= MEMO_MAKES:
            memo.clear()
        zs = memo.setdefault("z", {})
        lay = layout.layers
        z = (layout.top, layout.pitch, layout.final, tuple(sorted(layout.gaps.items())),
             tuple(sorted(layout.thick.items())), tuple(sorted(layout.choices.items())))
        z = zs.setdefault(z, len(zs))           # the z, as a small int
        every = tuple(sorted(lay.items()))
        deps = memo.setdefault("deps", {})
    for c in claims:
        if memo is None:
            got, why = made(c, layout)
        else:
            i = id(c)
            order = deps.get(i)
            if order is None:
                order = deps[i] = tuple(sorted(c.deps))
            key = (i, z, every if c.final else tuple([lay.get(d) for d in order]))
            hit = memo.get(key)
            if hit is None:
                hit = memo[key] = made(c, layout)
            got, why = hit
        if got is None:
            e = PlanReject(why)
            e.claim = c
            raise e
        out.extend(got)
    return out


def plate_bridged(shapes: Iterable[Placed]) -> set[int]:
    """The gaps a stack of plates runs on through (a group's plates at one core in the
    layers on both sides): a filler plate fills such a gap, so it is a thin sheet's
    thickness; any other gap is a stack of washers and shims, so any 0.1 mm."""
    plates: dict[tuple, set[int]] = {}
    for p in shapes:
        if p.sheet > 0 and not p.gap:
            plates.setdefault((p.group, p.shape.core), set()).add(p.layer)
    return {k for ks in plates.values() for k in ks if k + 1 in ks}


GAP_STEP = 0.1        # a gap only washers and shims fill: any multiple of the thinnest shim
GAP_MAX = 4.0         # ... up to this (a head with its shims; a taller one keeps a layer)
GAP_TRIES = 60        # thicker gaps a plan's z tries for a claim that fails (finalize)
GAP_MORE = 3.0        # the most a plan's z thickens a gap past its heads' need (a layer's worth)


class _ReadGaps(Mapping):
    """A layout's gaps that note every gap a claim read (``read``: layer -> thickness).
    Only values are noted: the keys, and so ``len``, iteration and membership, are the
    same in every try of :func:`_thicker_gaps`."""

    __slots__ = ("_gaps", "read")

    def __init__(self, gaps: Mapping[int, float]):
        self._gaps = gaps
        self.read: dict[int, float] = {}

    def __getitem__(self, k: int) -> float:
        v = self._gaps[k]
        self.read[k] = v
        return v

    def get(self, k, default=None):
        return self[k] if k in self._gaps else default

    def __contains__(self, k) -> bool:
        return k in self._gaps

    def __iter__(self):
        return iter(self._gaps)

    def __len__(self) -> int:
        return len(self._gaps)


def _thicker_gaps(err: PlanReject, spec: StackSpec, layers, top: int, choices,
                  gaps: dict[int, float], thick: dict[int, float],
                  bridged: set[int]) -> dict[int, float]:
    """The gaps thickened the least (one gap at a time, then two) at which the claim that
    failed (``err.claim``) builds, among the :data:`GAP_TRIES` least thickenings; ``err``
    again when none does. Bounded on purpose: the search takes the first layering that
    builds at a size, so a layering let through on a far thicker gap would end it on a
    taller plan than the next layering gives (measured: a 37.6 mm single module went 44.2
    mm when every single thickening was tried)."""
    claim = getattr(err, "claim", None)
    if claim is None:
        raise err
    ks = sorted(gaps)
    more = {k: [o for o in _gap_options(k, gaps[k], spec, bridged) if o > gaps[k] + EPS_Z]
            for k in ks}
    # every thickening (its total, then what it changes), in the order of sorting the gap
    # dicts by (rounded total, sorted items) and keeping the first GAP_TRIES: only those
    # whose total is within rounding (1e-5) of the GAP_TRIES-th least can be among them, and
    # every gap dict has the keys ``ks``, so its items compare as its values in that order
    cands: list[tuple[float, tuple]] = []
    for k in ks:
        cands += [(o - gaps[k], ((k, o),)) for o in more[k]]
    for a, b in itertools.combinations(ks, 2):
        cands += [(oa + ob - gaps[a] - gaps[b], ((a, oa), (b, ob)))
                  for oa in more[a][:8] for ob in more[b][:8]]
    if len(cands) > GAP_TRIES:
        cut = heapq.nsmallest(GAP_TRIES, (c[0] for c in cands))[-1] + 1e-5
        cands = [c for c in cands if c[0] <= cut]
    keyed = []
    for i, (d, how) in enumerate(cands):
        g = {**gaps, **dict(how)}
        keyed.append(((round(d, 6), tuple(g[k] for k in ks), i), d, g))
    keyed.sort(key=lambda t: t[0])
    tries = [(d, g) for _, d, g in keyed[:GAP_TRIES]]
    # A claim's ``make`` is a function of what it reads of its layout: a try that agrees
    # with a failed one on every gap that one read fails too, and is skipped (the gaps
    # read through :class:`_ReadGaps`; every try has the same keys, layers and thicknesses)
    failed: list[dict[int, float]] = []
    seen = _ReadGaps(gaps)
    if made(claim, Layout(layers, top, spec.pitch, choices, seen, dict(thick),
                          final=True))[0] is None:
        failed.append(seen.read)
    for _, g in tries:
        if any(all(g[k] == v for k, v in r.items()) for r in failed):
            continue
        seen = _ReadGaps(g)
        layout = Layout(layers, top, spec.pitch, choices, seen, dict(thick), final=True)
        if made(claim, layout)[0] is not None:
            return g
        failed.append(seen.read)
    most = max((t[0] for t in tries), default=0.0)
    raise PlanReject(f"{err} (nor with its clearance gaps thickened by up to {most:.2g} mm "
                     f"in all: the {len(tries)} least thickenings of one or two gaps)")


HEADS_ORDER = {"best": ("sink", "gap"), "gap_sink": ("gap", "sink")}

GIVE_UP = 200
"""(``gap_sink``) The first gap search gives up for the sunk one after this many layerings
failed on the crank's washers in the plan's gaps with no plan found (TrotBot's heel meets
~65 a CPU second; the designs that plan in gaps meet a handful first)."""
"""The heads searches :meth:`StackProblem.solve_heads` tries in turn, per
:attr:`StackSpec.heads`."""

GAP_GROUPS = ("drive",)
"""Groups whose heads stay in their clearance gaps beside a router's (the servo's, the
frame ties' and the deck rails' screws under the inner plate: the crank's horn screws
share that gap), when the other heads sink (:func:`heads_claims`)."""


def heads_claims(claims: Iterable[Claim], heads: str, keep: frozenset[str] = frozenset()
                 ) -> tuple[Claim, ...]:
    """The claims as the search sees them for ``heads`` (:attr:`StackSpec.heads`): "sink"
    puts every head that may sink into the layer beside its link (and drops what would
    only be in a gap: an axle's washers), "gap" leaves them. ``keep``: groups whose shapes
    stay as they are either way (a single-plate crank's, whose router places its heads in
    gaps: sunk, they would stand in a rider's layer; :data:`GAP_GROUPS`); the plan's z
    gives what crosses their gaps its washers (:meth:`StackProblem.plan`)."""
    claims = tuple(claims)
    if heads != "sink":
        return claims

    def sunk(make):
        if make is None:
            return None

        def f(L: Layout):
            out = make(L)
            if out is None:
                return None
            got = [p if p.group in keep else
                   replace(p, layer=p.layer + (1 if p.toward > 0 else 0), gap=False)
                   if p.gap and p.toward else p
                   for p in out if not p.gap or p.toward or p.group in keep]
            for p in got:
                if p.height > L.t(p.layer) + EPS_Z:
                    raise Unbuildable(f"{p.label or p.group} needs {p.height:.2f} mm, more "
                                      f"than layer {p.layer} ({L.t(p.layer):g} mm)")
            return got
        return f

    return tuple(replace(c, make=sunk(c.make), early=sunk(c.early)) for c in claims)


def finalize(topo: Topology, claims: Iterable[Claim], spec: StackSpec,
             layers: Mapping[str, int], top: int,
             choices: Mapping[str, object] | None = None,
             sink_all_but: frozenset[str] | None = None, memo: dict | None = None
             ) -> StackPlan:
    """The plan of a layering: which heads sink into a layer and which keep a clearance
    gap (:func:`_sinkable`), each gap's stock thickness and each layer's (the plates'
    sheets), and every claim made again at the z those give, until they settle (a
    claim's head may need more at its real z: a Chicago screw's shims). Deterministic in
    its arguments, so a stored layering re-makes the same plan. ``sink_all_but``: every
    head but those groups' sinks into its layer (the search placed them there:
    :meth:`StackProblem.plan` with a router's heads in gaps), unless the plan's z or a gap
    kept beside it says otherwise."""
    claims = tuple(claims)
    choices = dict(choices or {})
    layers = dict(layers)
    nominal = Layout(layers, top, spec.pitch, choices)
    shapes = _make_all(claims, nominal, memo)
    sunk = _sinkable(shapes, nominal, topo.geometry, spec.margin)
    if sink_all_but is not None:
        sunk |= {sunk_key(p) for p in shapes
                 if p.gap and p.toward and p.group not in sink_all_but}
    gaps: dict[int, float] = {}
    thick: dict[int, float] = {}
    layout = nominal
    for _ in range(8):
        # a head that needs more at the plan's z than the layer it sank into keeps its gap
        sunk -= {sunk_key(p) for p in shapes if p.gap and p.toward and sunk_key(p) in sunk
                 and p.height > layout.t(p.layer + (1 if p.toward > 0 else 0)) + EPS_Z}
        bridged = plate_bridged(shapes)
        while True:
            kept = [p for p in shapes if not (p.gap and p.toward and sunk_key(p) in sunk)]
            want = _gap_sizes(kept, spec, top, bridged)
            # a head can't sink past a gap the plan keeps there (it hangs off its link's
            # face): every head in a gap the others keep stays in it
            have = set(want) | {k for k, v in gaps.items() if v}
            back = {sunk_key(p) for p in shapes if p.gap and p.toward
                    and sunk_key(p) in sunk and p.layer in have}
            if not back:
                break
            sunk -= back
        new_gaps = {k: max(v, gaps.get(k, 0.0)) for k, v in {**gaps, **want}.items()}
        new_thick = _thicknesses(layers, shapes, spec, top)
        new_thick = {k: max(v, thick.get(k, 0.0)) for k, v in {**thick, **new_thick}.items()}
        if layout is not nominal and new_gaps == gaps and new_thick == thick:
            break
        gaps, thick = new_gaps, new_thick
        layout = Layout(layers, top, spec.pitch, choices, dict(gaps), dict(thick), final=True)
        try:
            shapes = _make_all(claims, layout, memo)
        except PlanReject as e:
            if not gaps:
                raise
            # a stock part (a crank bolt, a standoff) that misses at these gaps may fit at
            # thicker ones: the least thickening that builds the claim, then all of them
            gaps = _thicker_gaps(e, spec, layers, top, choices, gaps, thick, bridged)
            layout = Layout(layers, top, spec.pitch, choices, dict(gaps), dict(thick),
                            final=True)
            shapes = _make_all(claims, layout, memo)
    else:
        raise PlanReject("the clearance gaps and the claims made at their z don't settle")
    placed = settle(shapes, sunk, layout)
    return StackPlan(spec, layers, top, topo, claims, tuple(placed), choices,
                     gaps=dict(gaps), thick=dict(thick), sunk=frozenset(sunk))
