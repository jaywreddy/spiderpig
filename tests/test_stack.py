"""Tests for :mod:`stack`: the claims-based layer planner."""

from __future__ import annotations

import dataclasses
import itertools
import json
import random

import numpy as np
import pytest

from spiderpig import fabricate, stack
from spiderpig.config import BuildConfig
from spiderpig.construction.crank import CrankRoute, Run
from spiderpig.design import engine_version
from spiderpig.fabricate import side_problem, template_for
from spiderpig.stack import (
    Claim,
    Disc,
    Geometry,
    Layout,
    Pill,
    Placed,
    StackProblem,
    StackSpec,
    Topology,
    seg_seg,
    verify_plan,
)
from tests import cache, tiers


@pytest.fixture(params=["single", "double", "decker", "quad"])
def planned(request, design):
    """``(side template, SideDesign)`` per module."""
    return design(request.param)


def test_seg_seg_is_exact():
    a, b = np.array([[0.0, 0.0]]), np.array([[10.0, 0.0]])
    assert seg_seg(a, b, np.array([[5.0, -1.0]]), np.array([[5.0, 1.0]]))[0] == 0.0  # crossing
    assert seg_seg(a, b, np.array([[0.0, 3.0]]), np.array([[10.0, 3.0]]))[0] == 3.0  # parallel
    assert seg_seg(a, b, np.array([[13.0, 4.0]]), np.array([[20.0, 4.0]]))[0] == 5.0  # end-to-end


def test_distance_is_a_lower_bound_between_samples():
    """A point on a circle passing a fixed point: coarse samples must not overshoot."""
    fine = np.linspace(0, 2 * np.pi, 20000, endpoint=False)
    coarse = fine[::500]

    def geo(ts):
        return Geometry({"p": np.c_[10 * np.cos(ts), 10 * np.sin(ts)], "q": [10.0, 0.4]})

    truth = geo(fine).dist(("pt", "p"), ("pt", "q"))
    assert geo(coarse).dist(("pt", "p"), ("pt", "q")) <= truth + 1e-9


def _toy(links: dict[str, tuple[np.ndarray, np.ndarray]]):
    points, segs = {}, {}
    for n, (p, q) in links.items():
        points[f"{n}.a"], points[f"{n}.b"] = p, q
        segs[n] = ((f"{n}.a", f"{n}.b"),)
    topo = Topology("toy", Geometry(points), segs, (), {})

    def claim(n):
        return Claim(n, frozenset((n,)),
                     lambda L: [Placed(L.layers[n], Pill(f"{n}.a", f"{n}.b", 3.0), n, n)])

    return topo, [claim(n) for n in links]


def test_links_that_collide_get_different_layers():
    ts = np.zeros((4, 1))
    bar = (np.c_[ts * 0, ts * 0], np.c_[ts * 0 + 50, ts * 0])
    crossing = (np.c_[ts * 0 + 25, ts * 0 - 20], np.c_[ts * 0 + 25, ts * 0 + 20])
    far = (np.c_[ts * 0, ts * 0 + 90], np.c_[ts * 0 + 50, ts * 0 + 90])
    topo, claims = _toy({"a": bar, "b": crossing, "c": far})
    plan = StackProblem(topo, claims).solve()
    assert plan.layers["a"] != plan.layers["b"]
    assert plan.top == 3                                 # two link layers between the plates
    assert all(0 < k < plan.top for k in plan.layers.values())


def test_a_claim_that_cannot_be_built_forces_another_layout():
    ts = np.zeros((2, 1))
    topo, claims = _toy({"a": (np.c_[ts * 0, ts * 0], np.c_[ts * 0 + 9, ts * 0])})
    claims.append(Claim("fussy", frozenset("a"), lambda L: None if L.layers["a"] < 3 else []))
    plan = StackProblem(topo, claims).solve()
    assert plan.layers["a"] >= 3


def test_plan_clears_everything_over_the_full_cycle(planned):
    tmpl, design = planned
    assert verify_plan(design.plan, tmpl) == []


def test_verifier_catches_a_bad_plan(design):
    """Negative control: b1 and b2 in one layer is the original layering bug."""
    tmpl, d = design("single")
    plan = d.plan
    broken = dataclasses.replace(plan, layers={**plan.layers, "b2": plan.layers["b1"]})
    assert any("b1" in v and "b2" in v for v in verify_plan(broken, tmpl))


def test_every_pillar_is_held_by_a_frame_plate(planned):
    """Every pillar is anchored in a frame plate, both where its links let it reach them (a
    standoff pillar, the default, can't neck down past a link that sweeps close: then it is
    a cantilever from the plate it reaches)."""
    _, design = planned
    plan = design.plan
    for ax in plan.topo.axes_of("frame"):
        labels = {p.label for p in plan.shapes(f"pillar:{ax.name}")}
        anchors = [p.layer for p in plan.shapes(f"pillar:{ax.name}") if p.label.endswith("anchor")]
        assert anchors, (ax.name, labels)
        assert set(anchors) <= {0, plan.top}, (ax.name, labels)


def test_crank_crosses_a_b1_layer_only_along_its_crankpin(planned):
    _, design = planned
    plan = design.plan
    riders = {plan.layers[b] for b in plan.topo.riders}
    for p in plan.shapes("crank", gaps=False):
        if p.layer in riders:
            assert p.seat, p
            assert p.label.startswith("crankpin"), p


def test_every_link_is_held_on_its_axles(planned):
    """Each link on an axle has a shoulder, head, cap, plate or another link either side."""
    _, design = planned
    plan = design.plan
    for ax in plan.topo.axes:
        if ax.kind not in ("pin", "frame"):
            continue
        group = ("pillar:" if ax.kind == "frame" else "pin:") + ax.name
        held = {p.layer for p in plan.shapes(group, gaps=False)
                if not p.label.endswith("neck")} | {plan.layers[m] for m in ax.members}
        # a head or cap in the clearance gap beside a link holds it there
        held |= {p.layer for p in plan.shapes(group) if p.gap and p.height > 0
                 and p.toward < 0}                   # under the link over the gap
        held |= {p.layer + 1 for p in plan.shapes(group) if p.gap and p.height > 0
                 and p.toward > 0}                   # over the link under it
        for m in ax.members:
            k = plan.layers[m]
            assert {k - 1, k + 1} <= held, (ax.name, m)


def test_layout_layers_between():
    L = Layout({}, 5, 3.0)
    assert list(L.layers_between(2.9, 6.1)) == [0, 1, 2]
    assert list(L.layers_between(3.0, 6.0)) == [1]


# -- the plan's z (stack.finalize) on hand-made claims ------------------------------------


def _still(**pts) -> Geometry:
    """Fixed points (two samples each)."""
    return Geometry({n: np.array([xy, xy], dtype=float) for n, xy in pts.items()})


def _z_problem(*claims) -> Topology:
    topo = Topology("z", _still(O=(0, 0), P=(40, 0), Q=(80, 0)), {"a": (("O", "P"),)}, (), {})
    return topo, (Claim("a", frozenset("a"),
                        lambda L: [Placed(L.layers["a"], Pill("O", "P", 3.0), "a", "a")]),
                  *claims)


def _head(layer: int, h: float, at: str = "Q", group: str = "pin", toward: int = 0) -> Claim:
    return Claim(group, frozenset(), lambda L: [
        Placed(layer, Disc(at, 3.0), group, f"{group} head", gap=True, height=h,
               toward=toward)])


def test_a_gap_is_a_stock_sheet_where_plates_run_through_it_else_any_tenth():
    """A head in the gap over layer 2, where the crank's plates stand in layers 2 and 3 (a
    filler plate fills it: the thinnest stock sheet over the head), and one over layer 4
    (washers and shims: the next 0.1 mm); the plates run on through their gap."""
    plates = Claim("crank", frozenset(), lambda L: [
        Placed(k, Disc("O", 5.0), "crank", "crank web", sheet=3.175) for k in (2, 3)])
    topo, claims = _z_problem(plates, _head(2, 1.2), _head(4, 1.23, group="pillar"))
    plan = stack.finalize(topo, claims, StackSpec(), {"a": 5}, 7)
    assert plan.gaps == {2: 1.5, 4: 1.3}
    assert plan.thick == {2: 3.175, 3: 3.175}         # a plate thicker than its layer
    (through,) = [p for p in plan.placed if p.label == "crank through the gap"]
    assert (through.layer, through.gap, through.sheet) == (2, True, 3.175)
    assert plan.z(3)[0] == pytest.approx(3.0 + 3.0 + 3.175 + 1.5)
    assert verify_plan(plan) == []


def test_a_head_with_room_beside_it_sinks_and_one_without_keeps_its_gap():
    """``toward``: a head may sink into the layer over (+1) or under (-1) its gap when
    nothing of another group is there; link ``a`` in layer 3 sweeps P."""
    topo, claims = _z_problem(_head(1, 2.0, at="Q", toward=+1),
                              _head(2, 2.0, at="P", group="cap", toward=+1))
    plan = stack.finalize(topo, claims, StackSpec(), {"a": 3}, 6)
    assert plan.gaps == {2: 2.0}                       # P's head can't go into a's layer
    sunk = {k[0]: k[2] for k in plan.sunk}
    assert sunk == {"pin": 1}
    (pin,) = plan.shapes("pin")
    assert (pin.layer, pin.gap) == (2, False)          # in layer 2 now, at its full height
    # every head sunk but the drive's, as the search placed them (``sink_all_but``)
    forced = stack.finalize(topo, claims, StackSpec(), {"a": 3}, 6,
                            sink_all_but=frozenset({"cap"}))
    assert {k[0] for k in forced.sunk} == {"pin"}


def _counted(make, read: bool = True):
    """``make``, noting the gap over layer 4 of every final layout it is made at (what it
    reads anyway: noting more would be reading more)."""
    calls = []

    def f(L):
        calls.append((L.gap(4) if read else 0.0) if L.final else None)
        return make(L)
    return f, calls


def test_a_stock_part_that_misses_at_its_gaps_gets_the_least_thickening():
    """A claim that only builds once the gap over layer 4 is 1.6 mm: the plan's z thickens
    that gap the least (one gap, then two), and skips the tries that change only gaps the
    claim never read (the gap over layer 2 here)."""
    def need(L):
        if L.final and L.gap(4) < 1.6 - 1e-9:
            raise stack.Unbuildable("its stock length needs 1.6 mm over layer 4")
        return []

    make, calls = _counted(need)
    topo, claims = _z_problem(_head(2, 1.2), _head(4, 1.2, group="pillar"),
                              Claim("stock", frozenset(), make))
    plan = stack.finalize(topo, claims, StackSpec(), {"a": 5}, 7)
    assert plan.gaps == {2: 1.2, 4: 1.6}
    # the final z, then (in the thickening) once more at it, then only the thickenings of
    # the gap over layer 4: 1.3 .. 1.6 (layer 2's, alone or with layer 4's at a value it
    # failed at, skipped), then the plan's z made again
    assert [c for c in calls if c is not None] == [1.2, 1.2, 1.3, 1.4, 1.5, 1.6, 1.6]


def test_a_claim_no_thickening_helps_is_refused_after_one_more_try():
    """A claim that fails at the plan's z whatever its gaps (it reads none): every
    thickening agrees with the failed try on what it read, so none is made, and the
    refusal is the one the 60 tries would give."""
    make, calls = _counted(lambda L: None if L.final else [], read=False)
    topo, claims = _z_problem(_head(2, 1.2), _head(4, 1.2, group="pillar"),
                              Claim("stock", frozenset(), make))
    with pytest.raises(stack.PlanReject, match=r"stock can't be built in this layout \(nor "
                       r"with its clearance gaps thickened by up to 1 mm in all: the 60 "
                       r"least thickenings of one or two gaps\)"):
        stack.finalize(topo, claims, StackSpec(), {"a": 5}, 7)
    assert len([c for c in calls if c is not None]) == 2      # the plan's z, and once more


def _reference_tries(gaps, spec, bridged):
    """The thickenings ``_thicker_gaps`` tries, in its order, as first written: every one
    made, sorted whole by (total rounded, sorted items), the first GAP_TRIES kept."""
    ks = sorted(gaps)
    more = {k: [o for o in stack._gap_options(k, gaps[k], spec, bridged)
                if o > gaps[k] + stack.EPS_Z] for k in ks}
    tries = []
    for k in ks:
        tries += [(o - gaps[k], {**gaps, k: o}) for o in more[k]]
    for a, b in itertools.combinations(ks, 2):
        tries += [(oa + ob - gaps[a] - gaps[b], {**gaps, a: oa, b: ob})
                  for oa in more[a][:8] for ob in more[b][:8]]
    tries.sort(key=lambda t: (round(t[0], 6), sorted(t[1].items())))
    return tries[:stack.GAP_TRIES]


def test_the_thickenings_are_tried_in_the_order_of_sorting_them_all():
    """``_thicker_gaps`` sorts only the thickenings within rounding of the GAP_TRIES-th
    least: the same tries, in the same order, as sorting them all; with a claim that reads
    every gap, none is skipped."""
    rng = random.Random(7)
    spec = StackSpec()
    for _ in range(150):
        ks = rng.sample(range(1, 19), rng.randint(1, 7))
        gaps = {k: rng.choice([0.3, 0.5, 0.8128, 1.0, 1.016, 1.3, 2.0, 2.4, 3.6, 3.9, 4.0])
                for k in ks}
        bridged = {k for k in ks if rng.random() < 0.3}
        seen = []

        def make(L, seen=seen):
            seen.append(dict(L.gaps.items()))

        err = stack.PlanReject("x")
        err.claim = Claim("x", frozenset(), make)
        with pytest.raises(stack.PlanReject):
            stack._thicker_gaps(err, spec, {}, 20, {}, dict(gaps), {}, bridged)
        assert seen[1:] == [g for _, g in _reference_tries(gaps, spec, bridged)]


def test_the_plans_z_is_the_sum_of_its_layers_and_gaps_in_order():
    """``Layout.z``: each layer's bottom is ``sum`` over the layers and gaps under it, in
    order, to the last bit (float sums are compensated: no running total), and
    ``layers_between`` (remembered per z and interval) is the layers overlapping it."""
    rng = random.Random(3)
    for _ in range(300):
        top = rng.randint(6, 30)
        thick = {k: rng.choice([3.0, 2.032, 2.54, 2.286, 3.175, 1.5])
                 for k in rng.sample(range(-3, top + 3), rng.randint(0, 6))}
        gaps = {k: round(rng.uniform(0.1, 4.0), 1)
                for k in rng.sample(range(0, top), rng.randint(0, 6))}
        L = Layout({}, top, 3.0, {}, gaps, thick, final=True)
        ks = list(range(-20, top + 20))
        rng.shuffle(ks)
        for k in ks:
            z0 = (k * L.pitch if not thick and not gaps
                  else sum(L.t(j) + L.gap(j) for j in range(k)) if k >= 0
                  else -sum(L.t(j) + L.gap(j) for j in range(k, 0)))
            assert repr(L.z(k)) == repr((z0, z0 + L.t(k)))
        again = Layout({}, top, 3.0, {}, dict(gaps), dict(thick))
        for _ in range(10):
            a = rng.uniform(-10, 100)
            b = a + rng.uniform(0, 20)
            got = L.layers_between(a, b)
            assert again.layers_between(a, b) == got
            want = [k for k in range(-40, top + 40)
                    if L.z(k)[0] < b - 1e-9 and L.z(k)[1] > a + 1e-9]
            assert list(got) == (want if want else [])


# -- recorded plans: made again from their layering, and the cache's seeded plans ---------

PLANS: dict[str, dict] = {
    "klann-single": {"module": "single"},
    "klann-double": {"module": "double"},
    "klann-decker": {"module": "decker"},
    "klann-quad": {"module": "quad"},
    "strider-double": {"linkage": "strider", "module": "double"},
    "klann_lego-quad": {"linkage": "klann_lego", "module": "quad"},
    "hoecken": {"linkage": "hoecken", "module": "single"},
    "hoecken_pantograph": {"linkage": "hoecken_pantograph", "module": "single"},
    "dwell_rocker": {"linkage": "dwell_rocker", "module": "single"},
}
"""The designs whose plans ``tests/fixtures/planner/plans/<name>.json`` record: the order
designs and the mechanisms."""


def _plan_cfg(name: str) -> BuildConfig:
    return BuildConfig(**{"linkage": "klann", "robot": False, **PLANS[name]})


def _plan_record(name: str) -> dict:
    """A recorded plan: the layering, the crank's route and the plan's z, as the planner
    solves it now (not its proof, whose budget is the machine's)."""
    cfg = _plan_cfg(name)
    tmpl = template_for(cfg)
    _, _, problem = side_problem(tmpl, cfg)
    plan = problem.solve()
    doc = cache.plan_doc(plan)
    for k in ("optimal", "proof", "cost"):
        doc.pop(k)
    return {**doc, "height": plan.height}


def _choices(doc: dict) -> dict:
    route = doc["route"]
    return {} if route is None else {"crank": CrankRoute(
        tuple(Run(r["at"], r["lo"], r["hi"]) for r in route["runs"]), route["bearing"])}


@pytest.mark.parametrize("name", list(PLANS))
def test_a_recorded_plan_is_made_again_and_verifies(name):
    """The plan of a recorded layering and route (``StackProblem.plan``, what the store and
    the robot's side do instead of searching): it builds at its own z, verifies on a denser
    sampling, and (on the engine that recorded it) has the recorded gaps, thicknesses and
    height."""
    doc = cache.recorded("planner", f"plans/{name}", lambda: _plan_record(name))
    cfg = _plan_cfg(name)
    tmpl = template_for(cfg)
    _, _, problem = side_problem(tmpl, cfg, hint=False)
    plan = problem.plan(doc["layers"], doc["top"], _choices(doc), doc["heads"])
    assert verify_plan(plan, tmpl) == []
    assert plan.heads == doc["heads"]
    if cache.read_fixture("planner", f"plans/{name}")["engine_version"] == engine_version():
        got = cache.plan_doc(plan)
        assert (got["gaps"], got["thick"]) == (doc["gaps"], doc["thick"])
        assert plan.height == pytest.approx(doc["height"], abs=1e-9)


@pytest.mark.slow
@pytest.mark.fixture_regen
@pytest.mark.parametrize("name", list(PLANS))
def test_the_recorded_plans_are_current(name):
    """The planner still solves each recorded design to its recorded plan."""
    cache.assert_current("planner", f"plans/{name}", lambda: _plan_record(name))


@pytest.mark.parametrize("name", ["klann-single", "strider-double"])
def test_a_seeded_plan_is_the_solved_plan(name):
    """The cache's seed (``tests.cache``: the plan written as JSON, read back as a ``_Seed``)
    re-made the way ``design_side`` re-makes a known layout (``fabricate._reuse``: the plan
    of its layering, verified) is the solved plan: every placed shape, the gaps, the
    thicknesses, the sunk heads, the route, and the proof it carries."""
    cfg = _plan_cfg(name)
    tmpl = template_for(cfg)
    _, _, problem = side_problem(tmpl, cfg)
    solved = problem.solve()
    doc = json.loads(json.dumps(cache.plan_doc(solved)))
    _, _, again = side_problem(tmpl, cfg, hint=False)
    seeded = fabricate._reuse(again, cache._Seed(doc))
    assert seeded is not None
    for field in ("layers", "top", "choices", "gaps", "thick", "sunk", "heads", "optimal",
                  "proof", "cost"):
        assert getattr(seeded, field) == getattr(solved, field), field
    assert sorted(map(repr, seeded.placed)) == sorted(map(repr, solved.placed))
    assert seeded.height == solved.height
    assert verify_plan(seeded, tmpl) == verify_plan(solved, tmpl) == []
    # a seed whose layering doesn't verify (every link in one layer) is never reused:
    # design_side solves instead
    bad = {**doc, "layers": dict.fromkeys(doc["layers"], 2)}
    assert fabricate._reuse(again, cache._Seed(bad)) is None



def test_a_shape_in_a_frame_plates_layer_is_a_blocker_the_failure_parses():
    """A shape the search found in a frame plate's layer is tallied against the plate, and
    the blocker line :func:`failure.parse_blocker` reads back."""
    from spiderpig.failure import parse_blocker

    cfg = BuildConfig(linkage="hoecken_pantograph", robot=False)
    _, _, problem = side_problem(template_for(cfg), cfg)
    p = stack.Placed(0, stack.Disc("O", 3.0), "probe", "a probe shape")
    problem._tally(p, None)
    problem._tally(p, None)
    (line,) = [b for b in problem.blockers() if "probe" in b]
    assert line == "      2 x a probe shape vs a frame plate: it would sit in a frame plate's layer"
    assert parse_blocker(line) == {"count": 2, "a": "a probe shape", "b": "a frame plate",
                                   "why": "it would sit in a frame plate's layer",
                                   "text": line.strip()}


def test_a_claim_that_cant_be_built_at_the_plans_z_rejects_a_plan_with_no_gaps():
    """``finalize``: a claim that refuses the plan's own z with no clearance gap to thicken
    rejects the layering (``PlanReject``) as it is."""
    cfg = BuildConfig(linkage="hoecken_pantograph", robot=False)
    _, _, problem = side_problem(template_for(cfg), cfg)

    def make(L):
        if L.final:
            raise stack.PlanReject("no stock part fits at this z")
        return []

    claims = [stack.Claim("probe", frozenset(), make)]
    with pytest.raises(stack.PlanReject, match="no stock part fits at this z"):
        stack.finalize(problem.topo, claims, problem.spec, {}, 4)



def test_the_gaps_a_claim_reads_are_a_mapping_of_the_layouts():
    gaps = stack._ReadGaps({3: 2.4, 5: 4.0})
    assert len(gaps) == 2
    assert sorted(gaps) == [3, 5]
    assert 3 in gaps
    assert gaps.read == {}
    assert gaps[5] == 4.0
    assert gaps.get(7, 0.0) == 0.0
    assert gaps.read == {5: 4.0}


MEMO_DESIGNS = {
    "klann-single": {"linkage": "klann", "module": "single"},
    "jansen-single": {"linkage": "jansen", "module": "single"},
    "klann-quad": {"linkage": "klann", "module": "quad"},
    "trotbot_heel": {"linkage": "trotbot_heel", "module": "single"},    # heads gap_sink
}
"""Designs whose solves re-make claims in ``finalize`` (the Strider double and the
mechanisms make each once)."""


@pytest.mark.parametrize("name", tiers.quick(list(MEMO_DESIGNS), keep=["klann-single"]))
def test_a_remembered_claim_make_equals_a_fresh_one(name, monkeypatch):
    """``finalize``'s memo of claim makes (``StackProblem._makes``, keyed by what a make
    reads: its deps' layers, the stack, the z and the choices) gives exactly what making the
    claims afresh gives, for every ``_make_all`` of a whole solve (both heads searches on
    TrotBot's heel, whose sunk one makes the raw claims)."""
    cfg = BuildConfig(**{"robot": False, **MEMO_DESIGNS[name]})
    _, _, problem = side_problem(template_for(cfg), cfg, hint=False)
    original = stack._make_all
    calls = {"memo": 0, "makes": 0}
    memos: dict[int, dict] = {}         # (each heads search's problem has its own)

    def outcome(claims, layout, memo):
        try:
            return original(claims, layout, memo), None
        except stack.PlanReject as e:
            return None, (str(e), e.claim)

    def checked(claims, layout, memo=None):
        claims = tuple(claims)
        got = outcome(claims, layout, memo)
        if memo is not None:
            calls["memo"] += 1
            calls["makes"] += len(claims)
            memos[id(memo)] = memo
            assert got == outcome(claims, layout, None), layout
        if got[1] is not None:
            e = stack.PlanReject(got[1][0])
            e.claim = got[1][1]
            raise e
        return got[0]

    monkeypatch.setattr(stack.plan_z, "_make_all", checked)
    problem.solve()
    entries = sum(len(m) - 2 for m in memos.values())   # (less their "z" and "deps")
    assert calls["memo"] > 1
    assert 0 < entries < calls["makes"]         # the memo was hit
