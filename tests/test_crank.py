"""Tests for the crank group (:mod:`construction.crank`) on the bolt crank: single aluminium
web plates on hex standoff crankpins (``bolt``, the default) or round ones (``bolt_round``),
with every servo; the crank's routes (hand-made plans of what the planner may choose).

The Klann single sides they look at come from the fabrication cache (:mod:`tests.cache`);
a test of the crank alone realizes the crank group alone (:func:`tests._construction.
realize_groups`), and the contract's fast cases check the crank and the drive
(``check_side(groups=)``), the whole side in the slow tier (every servo's whole side:
``tests/test_contract.py``)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from spiderpig import servos
from spiderpig.config import BuildConfig
from spiderpig.construction import CRANKS as CRANK_REGISTRY
from spiderpig.construction.base import FRAME_OUTER, Build
from spiderpig.construction.contract import bad_solids, check_side, clashes
from spiderpig.construction.crank import (
    CrankRoute,
    Run,
    chains_of,
    default_route,
    hex_play,
    route_of,
)
from spiderpig.fabricate import (
    SideDesign,
    design_side,
    fabricate_side,
    side_problem,
    template_for,
)
from spiderpig.hardware.bom import bom_from_mechanism
from spiderpig.servos import cad as cadlib
from spiderpig.servos import model
from spiderpig.shapes import disc
from spiderpig.stack import Axis, Layout, verify_plan
from tests import cache
from tests._construction import overlapping_pairs, realize_groups
from tests.tiers import quick

MODES = ("single", "double", "decker", "quad")   # the Klann's (the Strider's crank differs)
SERVOS = servos.available()
OTHERS = [s for s in SERVOS if s != servos.DEFAULT]


@pytest.fixture(scope="module", autouse=True)
def _offline(tmp_path_factory):
    """No downloads in tests: servos are drawn parametrically."""
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv(cadlib.OFFLINE_ENV, "1")
        mp.setenv(cadlib.CACHE_ENV, str(tmp_path_factory.mktemp("cad")))
        for f in (model.cad_servo, model._servo_part, cadlib._load_cached):
            f.cache_clear()
        yield
        for f in (model.cad_servo, model._servo_part, cadlib._load_cached):
            f.cache_clear()


CRANKS = ["bolt", "bolt_round"]


def _cfg(mode="single", servo=servos.DEFAULT, crank="bolt") -> BuildConfig:
    return BuildConfig(linkage="klann", module=mode, robot=False, servo=servo, crank=crank)


def _design(mode="single", servo=servos.DEFAULT, crank="bolt"):
    """``(template, design)`` of the Klann ``mode`` side (the plan seeded from the cache)."""
    return cache.cached_design(_cfg(mode, servo, crank))


def _side(t=1.0, servo=servos.DEFAULT, crank="bolt", mode="single"):
    """``(design, the fabricated side at t)`` of the Klann ``mode`` side, from the cache."""
    cfg = _cfg(mode, servo, crank)
    return _design(mode, servo, crank)[1], cache.cached_side(cfg, t)


def _clashes(mech) -> list[tuple[str, str, float]]:
    """Every pair of parts sharing more than 1e-3 mm^3 (a pair whose boxes don't meet
    shares none)."""
    parts = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    out = []
    for a, b in overlapping_pairs(parts):
        inter = parts[a] & parts[b]
        vol = 0.0 if inter is None else sum(s.volume for s in inter.solids())
        if vol > 1e-3:
            out.append((a, b, round(vol, 4)))
    return out


def _volume(shape) -> float:
    return 0.0 if shape is None else sum(s.volume for s in shape.solids())


def _runs(design) -> tuple[Run, ...]:
    return route_of(design.plan.layout, design.plan.topo.axes_of("crankpin")).runs


def _hub_layer(plan) -> int:
    """The hub plate's layer (the top crank plate, the horn screws through it)."""
    return max(p.layer for p in plan.shapes("crank") if p.label == "crank hub")


def _bbox(mech, name):
    return mech.body(name).part.bounding_box()


def _part(mech, name: str):
    return next(b.part for b in mech.bodies if b.name == name)


# -- the contract ---------------------------------------------------------------------


@pytest.mark.slow
@pytest.mark.parametrize("crank", CRANKS)
@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize("t", [0.0, 2.2, 4.38])
def test_crank_stays_inside_its_claims(mode, t, crank):
    """The whole side (the fast tier checks the crank and the drive alone, below)."""
    tmpl, design = _design(mode, crank=crank)
    assert check_side(design, tmpl.freeze_at(t)) == []


@pytest.mark.parametrize("crank", CRANKS)
def test_the_crank_alone_stays_inside_its_claims(crank):
    tmpl, design = _design(crank=crank)
    assert check_side(design, tmpl.freeze_at(2.2), groups=("drive", "crank")) == []


@pytest.mark.parametrize("servo", OTHERS)
def test_every_servo_couples_the_crank_inside_the_claims(servo):
    tmpl, design = _design(servo=servo)
    assert check_side(design, tmpl.freeze_at(2.2), groups=("drive", "crank")) == []


# -- the parts ------------------------------------------------------------------------


@pytest.mark.parametrize("crank", CRANKS)
@pytest.mark.parametrize("mode", quick(["single", "double", "quad"], ["single"]))
def test_every_crank_part_is_one_valid_solid(mode, crank):
    """One aluminium plate per crank layer, one standoff per chain (and per journal between
    chains), each part one valid solid turning with the crank body; the bought ones with a
    BOM key, the plates on the laser sheets."""
    design, mech = _side(1.0, crank=crank, mode=mode)
    note = mech.meta["crank_bolt"]
    plates = [b for b in mech.bodies if b.name.startswith("crank_plate")]
    assert len(plates) == note["plates"]
    assert {int(b.name.removeprefix("crank_plate")) for b in plates} == {
        p.layer for p in design.plan.shapes("crank", gaps=False)
        if p.label in ("crank body", "crank hub") or p.label.startswith("web ")}
    chains = chains_of(_runs(design))
    assert len(note["chains"]) == len(chains)
    pins = [b for b in mech.bodies if b.name.startswith("crank_pin_")
            and b.name.removeprefix("crank_pin_") in
            {n["at"] for n in note["chains"] + note["journals"]}]
    assert len(pins) == len(chains) + len(note["journals"])
    for b in mech.bodies:
        if b.name.startswith("crank"):
            assert len(b.part.solids()) == 1, b.name
            assert b.part.is_valid, b.name
            assert b.rigid_with == design.plan.topo.crank_bodies[0]
            assert b.fab in ("laser", "printed", "purchased"), b.name
            assert (b.fab == "laser") == b.name.startswith("crank_plate"), b.name
            if b.fab == "purchased":
                assert b.bom_key, b.name


@pytest.mark.parametrize("servo", quick(SERVOS, [servos.DEFAULT]))
@pytest.mark.parametrize("t", quick([1.0, 4.38], [4.38]))
def test_single_side_parts_do_not_intersect(servo, t):
    """Every body, screws, standoffs and sleeves included."""
    _, mech = _side(t, servo)
    names = {b.name for b in mech.bodies}
    for prefix in ("servo_screw", "crank_pin_screw", "crank_pin_sleeve", "crank_plate",
                   "crank_stub", "crank_horn_screw"):
        assert any(n.startswith(prefix) for n in names), prefix
    assert _clashes(mech) == []


@pytest.mark.parametrize("servo", SERVOS)
def test_horn_screws_land_in_the_horn_holes(servo):
    tmpl = _design(servo=servo)[0]
    design, mech = _side(1.0, servo)
    frozen = tmpl.freeze_at(1.0)
    build = Build(design.ctx, design.plan, frozen)
    spec = servos.get(servo)
    pat = spec.horn.pattern
    iface = design.ctx.interfaces["drive"]
    o = build.xy("O")
    theta = design.drive.horn_angle(build)
    holes = [o + pat.pcd / 2 * np.array([math.cos(theta + 2 * math.pi * k / pat.count),
                                         math.sin(theta + 2 * math.pi * k / pat.count)])
             for k in range(pat.count)]
    plate_top = design.plan.z(design.plan.top)[1]
    horn_face = plate_top - spec.horn_face_depth
    horn = mech.body("servo_horn").part
    screws = [b for b in mech.bodies if b.name.startswith("crank_horn_screw")]
    assert len(screws) == pat.count
    used = set()
    for b in screws:
        bb = b.part.bounding_box()
        c = np.array([(bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2])
        k = min(range(len(holes)), key=lambda i: np.linalg.norm(holes[i] - c))
        assert np.linalg.norm(holes[k] - c) < 1e-3
        used.add(k)
        engage = bb.max.Z - horn_face
        assert engage >= min(pat.thread_depth, 1.5) - 1e-6
        assert engage <= min(pat.reach, spec.horn_face_depth) + 1e-6
        assert plate_top + 1e-6 >= bb.max.Z
        # the shank is in the hole, not in the horn's metal
        hole = disc(tuple(holes[k]), pat.thread_d / 2, horn_face, bb.max.Z)
        assert _volume(b.part & hole) > 0.5 * math.pi * (0.97 * pat.thread_d / 2) ** 2 * engage
        assert _volume(b.part & horn) < 1e-3
        assert b.bom_key.startswith("m" + pat.thread[1:].replace(".", "p") + "_")
    assert used == set(range(pat.count))
    # the hub plate bolts on at the coupling face and never rises above it
    top = _bbox(mech, f"crank_plate{_hub_layer(design.plan)}")
    assert pytest.approx(plate_top - iface.horn_face_depth) == top.max.Z


@pytest.mark.parametrize("servo", SERVOS)
def test_the_hub_turns_clear_of_the_inner_plate(servo):
    tmpl = _design(servo=servo)[0]
    design, mech = _side(1.0, servo)
    hub = _part(mech, f"crank_plate{_hub_layer(design.plan)}")
    plate_bottom = design.plan.z(design.plan.top)[0]
    margin = design.ctx.params.margin
    r = design.ctx.interfaces["drive"].horn_radius
    o = tuple(Build(design.ctx, design.plan, tmpl.freeze_at(1.0)).xy("O"))
    outside = disc(o, 60, plate_bottom - margin + 1e-3, plate_bottom + 3) - disc(
        o, r + 1e-3, plate_bottom - margin, plate_bottom + 4)
    assert _volume(hub & outside) < 1e-6


# -- the crankpin joints ----------------------------------------------------------------


def test_default_route_merges_adjacent_riders_of_one_pin():
    pins = [Axis("M", "crankpin", ("a", "b"), ()), Axis("N", "crankpin", ("c",), ())]
    route = default_route(Layout({"a": 2, "b": 3, "c": 6}, 8, 3.0), pins)
    assert route.runs == (Run("M", 2, 3), Run("N", 6, 6))
    assert route.bearing
    route = default_route(Layout({"a": 2, "b": 5, "c": 3}, 8, 3.0), pins)
    assert route.runs == (Run("M", 2, 2), Run("M", 5, 5), Run("N", 3, 3))


def test_chains_of_joins_runs_along_one_point():
    """A chain: runs along one point whose webs meet, in one layer or two adjacent ones (a
    run ending in layer 3 has its web in 4: one starting in 5 or 6 joins it); a run along
    another point, or one a layer further on, starts a new one."""
    a, b = Run("M", 2, 3), Run("M", 5, 6)
    assert chains_of((a, b)) == [[a, b]]
    assert chains_of((a, Run("M", 6, 6))) == [[a, Run("M", 6, 6)]]
    assert chains_of((a, Run("M", 7, 7))) == [[a], [Run("M", 7, 7)]]
    assert chains_of((a, Run("N", 5, 6))) == [[a], [Run("N", 5, 6)]]


@pytest.mark.parametrize("crank", CRANKS)
@pytest.mark.parametrize("mode", quick(["single", "double", "quad"], ["single"]))
def test_crankpins_are_standoffs_screwed_into_their_webs(mode, crank):
    """Every chain's standoff passes through its two web plates (the hex in their pockets,
    at most ``recess_max`` short of their outer faces; the round one clamped between them),
    a button head up into its lower end from under the lower web, one down into its upper
    end unless the hub plate caps it (the hex's hub chain); the riders turn on its sleeve
    (the hex's) between the plates."""
    design, mech = _side(1.0, crank=crank, mode=mode)
    c = CRANK_REGISTRY[crank].resolve(design.ctx)
    hub = _hub_layer(design.plan)
    by_at = {n["at"]: n for n in mech.meta["crank_bolt"]["chains"]}
    chains = chains_of(sorted(_runs(design), key=lambda r: (r.lo, r.at)))
    for ch in chains:
        at = ch[0].at
        tag = at if sum(x[0].at == at for x in chains) == 1 else f"{at}_{ch[0].lo}"
        note = by_at[tag]
        lo = _bbox(mech, f"crank_plate{ch[0].lo - 1}")
        hi = _bbox(mech, f"crank_plate{ch[-1].hi + 1}")
        pin = _bbox(mech, f"crank_pin_{tag}")
        screw_lo = _bbox(mech, f"crank_pin_screw_lo_{tag}")
        assert screw_lo.min.Z < lo.min.Z                  # its head under the lower web
        assert pin.min.Z - 1e-6 <= screw_lo.max.Z <= pin.max.Z + 1e-6
        if c.hex:
            assert lo.min.Z + c.recess_max + 1e-6 >= pin.min.Z
            assert hi.max.Z - c.recess_max - 1e-6 <= pin.max.Z
            sleeve = _bbox(mech, f"crank_pin_sleeve_{tag}")
            assert lo.max.Z - 1e-6 <= sleeve.min.Z
            assert sleeve.max.Z <= hi.min.Z + 1e-6
            for r in ch:                                   # its riders' layers on the sleeve
                for k in range(r.lo, r.hi + 1):
                    z0, z1 = design.plan.z(k)
                    if any(design.plan.layers[m] == k for m in design.plan.topo.riders):
                        assert z0 + 1e-6 >= sleeve.min.Z
                        assert z1 <= sleeve.max.Z + 1e-6
        else:                                              # clamped between the webs
            assert lo.max.Z - 1e-6 <= pin.min.Z
            assert pin.max.Z <= hi.min.Z + 1e-6
        capped = ch[-1].hi + 1 == hub and c.hub_capped(design.ctx, at)
        assert note.get("capped", False) == capped      # (the round one's note has none)
        names = {b.name for b in mech.bodies}
        assert (f"crank_pin_screw_hi_{tag}" in names) == (not capped)
        if not capped:
            assert _bbox(mech, f"crank_pin_screw_hi_{tag}").max.Z > hi.max.Z
    if mode == "double":                               # both legs on one standoff: one joint
        assert len(chains) == 1
        riders = {design.plan.layers[m] for m in design.plan.topo.riders}
        assert len(riders) == 2
        assert riders <= set(range(chains[0][0].lo, chains[0][-1].hi + 1))


def test_hex_play_of_a_standoff_in_its_pockets():
    """A hex turns until its corners meet the pocket's flats: a 5.0 AF one 3.13 deg in a
    5.15 pocket, 18.1 deg in a 5.65 one, none when the pocket is no wider than the hex; the
    crank's note gives the hex standoff's in its 0.1 mm over pockets, twice (two pockets)."""
    assert hex_play(5.0, 5.15) == pytest.approx(3.13, abs=0.01)
    assert hex_play(5.0, 5.65) == pytest.approx(18.13, abs=0.01)
    assert hex_play(5.0, 5.0) == hex_play(5.0, 4.9) == 0.0
    design, mech = _side(1.0)
    c = CRANK_REGISTRY["bolt"].resolve(design.ctx)
    note = mech.meta["crank_bolt"]
    assert note["pocket_af_mm"] == pytest.approx(c.hex_af + c.hex_fit)
    assert note["play_deg"] == pytest.approx(2 * hex_play(c.hex_af, c.hex_pocket_af()), abs=0.01)
    assert 0 < note["play_deg"] < 2 * hex_play(5.0, 5.15)


# -- crank routes: hand-made plans of what the planner may choose ---------------------------

KLANN = BuildConfig(linkage="klann", robot=False, module="single")
ROUTES = {  # link layers, top, route, detour point (name, r, degrees)
    # b1's run goes on over an empty layer above it (below it): its sleeve through that layer
    "past b1": ({"b1": 6, "b2": 4, "b3": 5, "b4": 5}, 10, CrankRoute((Run("M", 6, 7),)), None),
    "under b1": ({"b1": 6, "b2": 4, "b3": 3, "b4": 3}, 9, CrankRoute((Run("M", 5, 6),)), None),
    # a detour opposite the crankpin in an empty layer, on a standoff of its own capped by
    # the hub plate, a journal standoff on O between the two chains
    "detour": ({"b1": 2, "b2": 4, "b3": 3, "b4": 3}, 9,
               CrankRoute((Run("M", 2, 2), Run("X", 6, 6))), ("X", 14.0, 180.0)),
    "no bearing": ({"b1": 6, "b2": 4, "b3": 5, "b4": 5}, 9,
                   CrankRoute((Run("M", 6, 6),), bearing=False), None),
}
_ROUTED: dict[str, tuple] = {}


def _routed(config, layers, top, route, point=None):
    """One side planned by hand along ``route`` (``point``: a detour's (name, r, degrees))."""
    tmpl = template_for(config)
    ctx, groups, problem = side_problem(tmpl, config)
    if point is not None:
        ctx.topo.add_crank_point(*point)
    plan = problem.plan(layers, top, {"crank": route})
    assert verify_plan(plan) == []
    return tmpl, SideDesign(config, ctx, groups, plan)


def _route_case(name: str):
    """(template, design, the side fabricated at t = 4.38) for a case of :data:`ROUTES`."""
    if name not in _ROUTED:
        layers, top, route, point = ROUTES[name]
        tmpl, design = _routed(KLANN, layers, top, route, point)
        _ROUTED[name] = tmpl, design, fabricate_side(design, tmpl.freeze_at(4.38))
    return _ROUTED[name]


_CRANK_ONLY: dict[str, tuple] = {}


def _route_crank(name: str):
    """(template, design, the crank group's :class:`Realized` at t = 4.38) for a case of
    :data:`ROUTES`: the crank realized alone, as :func:`fabricate_side` builds it."""
    if name not in _CRANK_ONLY:
        if name in _ROUTED:
            tmpl, design, _ = _ROUTED[name]
        else:
            layers, top, route, point = ROUTES[name]
            tmpl, design = _routed(KLANN, layers, top, route, point)
        _CRANK_ONLY[name] = tmpl, design, realize_groups(design, 4.38, ["crank"],
                                                         tmpl.freeze_at(4.38))
    return _CRANK_ONLY[name]


@pytest.mark.slow
@pytest.mark.parametrize("case", sorted(ROUTES))
def test_every_route_builds_inside_its_claims(case):
    tmpl, design, mech = _route_case(case)
    assert check_side(design, tmpl.freeze_at(2.2)) == []
    assert clashes(mech) == []
    assert bad_solids(mech) == []
    crank = [b for b in mech.bodies if b.name.startswith("crank")]
    for b in crank:
        assert len(b.part.solids()) == 1, b.name
        assert b.part.is_valid, b.name
    runs = ROUTES[case][2].runs
    # one standoff per chain (a chain of runs along one point shares it), one journal
    # standoff between two chains a layer or more apart
    note = mech.meta["crank_bolt"]
    assert sorted(n["at"] for n in note["chains"]) == sorted({r.at for r in runs})
    assert len(note["journals"]) == (1 if case == "detour" else 0)
    bought = {r.key for r in bom_from_mechanism(mech).purchased}
    keys = {b.bom_key for b in crank if b.fab == "purchased"}
    # (the horn screws' DIN 988 shims are bought per thickness: hardware.bom.split_shims)
    assert {k for k in keys if not k.startswith("shim_")} <= bought
    assert {n["standoff"] for n in note["chains"] + note["journals"]} <= keys


@pytest.mark.parametrize(("case", "rider", "bare"), [("past b1", 6, 7), ("under b1", 6, 5)])
def test_a_run_past_its_rider_carries_the_sleeve_through_the_bare_layer(case, rider, bare):
    """No plate in a run's layer: the standoff and its sleeve go on through the layer the
    rider isn't in, the sleeve between the two webs, the rider's layer on it."""
    tmpl, design, got = _route_crank(case)
    z = design.plan.z
    names = {b.name for b in got.bodies}
    assert f"crank_plate{bare}" not in names
    assert f"crank_plate{rider}" not in names
    run = ROUTES[case][2].runs[0]
    lo, hi = (_part(got, f"crank_plate{k}").bounding_box()
              for k in (run.lo - 1, run.hi + 1))
    sleeve = _part(got, "crank_pin_sleeve_M").bounding_box()
    pin = _part(got, "crank_pin_M").bounding_box()
    assert lo.max.Z - 1e-6 <= sleeve.min.Z
    assert sleeve.max.Z <= hi.min.Z + 1e-6
    for k in (rider, bare):
        assert z(k)[0] + 1e-6 >= sleeve.min.Z
        assert z(k)[1] <= sleeve.max.Z + 1e-6
        assert z(k)[0] >= pin.min.Z
        assert z(k)[1] <= pin.max.Z
    # the sleeve fills the webs' gap but for the riders' end play (none: pressed, capped)
    c = CRANK_REGISTRY["bolt"].resolve(design.ctx)
    length = sleeve.max.Z - sleeve.min.Z
    assert hi.min.Z - lo.max.Z - c.sleeve_play - 1e-3 <= length


def test_a_crank_point_turns_with_the_crank():
    layers, top, route, point = ROUTES["detour"]
    tmpl, design = _routed(KLANN, layers, top, route, point)
    for t in (0.0, 1.0, 4.38):
        build = Build(design.ctx, design.plan, tmpl.freeze_at(t))
        x, m = build.xy("X") - build.xy("O"), build.xy("M") - build.xy("O")
        assert np.linalg.norm(x) == pytest.approx(14.0)
        assert np.dot(x, m) / np.linalg.norm(m) == pytest.approx(-14.0)   # opposite M


def test_without_the_bearing_the_crank_ends_at_its_lowest_web():
    tmpl, design, got = _route_crank("no bearing")
    names = {b.name for b in got.bodies}
    assert not any(n.startswith("crank_stub") for n in names)   # no journal stub
    run = ROUTES["no bearing"][2].runs[0]
    bottom = min(b.part.bounding_box().min.Z for b in got.bodies
                 if b.name.startswith("crank_plate"))
    assert pytest.approx(design.plan.z(run.lo - 1)[0]) == bottom   # the web on its floor
    assert FRAME_OUTER not in got.cuts                 # no journal hole in the outer plate


def test_a_lowest_web_no_stock_stub_reaches_is_reported():
    """The stub standoff from the lowest web down into the outer frame plate comes in stock
    lengths (M3 round, 6-30 mm): a web in layer 11 is out of its reach. The plan's z checks
    it (the crank's ``check_route``), so the layering doesn't even make a plan."""
    from spiderpig.stack import PlanReject

    with pytest.raises(PlanReject, match=r"no stock stub standoff reaches the outer frame plate "
                                         r"from the lowest web \(layer 11\)"):
        _routed(KLANN, {"b1": 12, "b2": 10, "b3": 11, "b4": 11}, 15,
                CrankRoute((Run("M", 12, 12),)))


# -- the Strider decker and quad on the hex crank (the debug of 2026-10-05) -----------------

def _hex_crank():
    from spiderpig.construction.crank import BoltCrank

    return BoltCrank().for_sheet("al6061_2p5mm")


def test_a_span_between_stock_hex_lengths_opens_the_chain_gaps():
    """No stock hex standoff fits a span of 26.5 mm capped in the hub plate (25 mm is 1.5
    short, 30 mm stands 3.5 past the lower plate): the gaps along the chain open, the one
    over the lowest web first, each up to the 4 mm a gap holds, to the next stock length;
    what is already in a gap counts."""
    from spiderpig.stack import GAP_MAX

    c = _hex_crank()
    t = c.web_t
    assert c.fit_hex(26.5, t, t, out_hi_max=0.0, capped=True) is None
    j = c.hex_gap_fit(26.5, [(9, 2.8), (10, 2.5)], t, t, out_hi_max=0.0, capped=True)
    assert j is not None
    assert j.length == 30.0
    gaps = dict(j.gaps)
    assert gaps[9] == pytest.approx(GAP_MAX)                 # the lowest web's gap first
    assert sum(g - h for g, h in ((gaps[9], 2.8), (gaps.get(10, 2.5), 2.5))) == pytest.approx(
        j.span - 26.5)
    assert j.gap == gaps[9]
    assert j.out_lo <= c.protrude_max + 1e-9
    assert j.out_hi == 0.0
    # no room left in the gaps: none
    assert c.hex_gap_fit(26.5, [(9, GAP_MAX)], t, t, out_hi_max=0.0, capped=True) is None


def test_the_upper_end_of_a_hex_pin_uses_the_air_over_its_plate():
    """What a standoff stands past its plates goes where the two gaps' needs come out even,
    the upper stack using the air over its plate in its layer first (a 0.100 in plate in a
    3 mm layer: 0.46 mm)."""
    c = _hex_crank()
    t = c.web_t
    span = 3.0 + 2 * 3.0 + t + 0.5                    # a 12 mm standoff stands ~0.46 past
    plain = c.fit_hex(span, t, t)
    air = c.fit_hex(span, t, t, air_hi=3.0 - t)
    extra = plain.length - span
    assert plain.out_lo + plain.out_hi == pytest.approx(extra)
    assert air.out_lo + air.out_hi == pytest.approx(extra)
    assert max(air.out_lo, air.out_hi - (3.0 - t)) <= max(plain.out_lo, plain.out_hi) + 1e-9
    assert air.out_hi >= air.out_lo


@pytest.mark.slow
@pytest.mark.parametrize("module", ["decker", "quad"])
@pytest.mark.usefixtures("fresh_plan_memo")
def test_the_strider_decker_and_quad_plan_on_the_hex_crank(module):
    """The Strider's decker and quad default to the hex-standoff crank (they were on the
    round friction crank, whose hub chain screw no assembly order drives): each plans, every
    chain's standoff a stock length at the plan's z, the hub chain capped (no screw over the
    hub plate), and the side builds inside its claims."""
    from spiderpig.construction.crank import chains_of as chains

    cfg = BuildConfig(module=module)
    assert cfg.crank == "bolt"
    tmpl = template_for(cfg)
    design = design_side(tmpl, cfg, advise=False)
    plan = design.plan
    assert verify_plan(plan, tmpl) == []
    crank = next(g for g in design.groups if g.name == "crank")
    c = crank.construction.resolve(design.ctx)
    assert c.hex
    route = route_of(plan.layout, [a.name for a in design.ctx.topo.axes_of("crankpin")])
    hub = max(k for k in range(plan.top) if any(
        p.label == "crank hub" for p in plan.shapes("crank") if p.layer == k))
    for ch in chains(route.runs):
        capped = ch[-1].hi + 1 == hub and c.hub_capped(design.ctx, ch[0].at)
        j = c.chain_fit_web(plan.layout, ch[0].lo, ch[-1].hi, t=c.web_t, hub=hub,
                            capped=capped)
        assert j is not None, ch
        assert j.gap == 0.0, ch                       # a stock length as the plan stands
        if ch[-1].hi + 1 == hub:
            assert capped
            assert not j.screw_hi



def test_a_screw_key_reads_back_as_its_kind_and_length():
    from spiderpig.hardware.fasteners import SCREWS
    from spiderpig.hardware.fasteners import parse as screw_from_key

    for key, kind in (("m3_shcs_10", SCREWS["shcs", "3"]), ("m3_bhcs_8", SCREWS["bhcs", "3"])):
        sk, length = screw_from_key(key)
        assert (sk, length) == (kind, float(key.rsplit("_", 1)[1]))
        assert sk.key(length) == key
    assert screw_from_key("m3_nut") is None
    assert screw_from_key("m5_shcs_10") is None        # no M5 kind modelled


def test_the_hub_sits_right_under_the_inner_plate_when_the_horn_needs_no_layer():
    """The XL430's horn face is 3.175 mm under the inner plate's top face, which is the
    0.125 in plate's own bottom face: the horn takes no layer of the stack
    (``horn_layers`` 0), its layers are the plate's and the hub's is the one under it."""
    from spiderpig.construction.crank import hub_layers
    from spiderpig.fabricate import side_problem, template_for
    from spiderpig.stack import Layout

    cfg = BuildConfig(linkage="hoecken_pantograph", robot=False, frame_sheet="al5052_3p2mm",
                      servo="xl430_w250")
    ctx, groups, _ = side_problem(template_for(cfg), cfg)
    drive = ctx.interfaces["drive"]
    assert drive.horn_layers == 0
    assert drive.horn_face_depth == pytest.approx(drive.plate_t)
    hub_t = next(g for g in groups if g.name == "crank").dims(ctx).hub_thickness
    horn, hub = hub_layers(Layout({}, 8, ctx.pitch), drive, hub_t)
    assert horn.start == 8          # from the inner plate up into the servo
    assert hub == range(7, 8)
