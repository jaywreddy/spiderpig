"""Tests for the metal-shaft pivots (:mod:`construction.pivots`): the Chicago screw pin and
the standoff pillar, the two left (the rod, bolt, bearing, bushing, PTFE and bushed-Chicago
pivots were removed on 2026-10-07: :data:`config.REMOVED_CONSTRUCTIONS`).

Checked on the Klann single (one fabrication, from the fabrication cache, :mod:`tests.cache`)
and on the Jansen single: the plan, the contract, clashes, the holes, a continuous stack on
every axle, the parts and the BOM; and the ways they refuse a design. The contract's fast
case checks the axle groups alone (``check_side(groups=)``), the whole side in the slow tier.
Their own fits: ``test_wobble.py`` (Chicago), ``test_standoff.py``.
"""

from __future__ import annotations

from dataclasses import replace

import pytest

from spiderpig import construction
from spiderpig.config import BuildConfig, ParamError
from spiderpig.construction import ConstructionError
from spiderpig.construction.axle import AxleGroup
from spiderpig.construction.base import FRAME_INNER, FRAME_OUTER, Build, Params, Realized
from spiderpig.construction.contract import check_side
from spiderpig.construction.pivots.common import Column
from spiderpig.fabricate import fabricate
from spiderpig.hardware.bom import bom_from_mechanism
from spiderpig.hardware.catalog import get
from spiderpig.materials import washer_od
from spiderpig.stack import verify_plan
from tests import cache
from tests._construction import overlapping_pairs

KEYS = ("chicago", "standoff")
T = 1.0


def _config(**kw) -> BuildConfig:
    return BuildConfig(**{"linkage": "klann", "module": "single", "robot": False, **kw})


def _vol(a, b) -> float:
    inter = a & b
    return 0.0 if inter is None else sum(s.volume for s in inter.solids())


def _clashes(mech, allowed=()) -> list[tuple[str, str, float]]:
    allowed = {frozenset(p) for p in allowed}
    parts = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    out = []
    for a, b in overlapping_pairs(parts):     # a pair whose boxes don't meet shares nothing
        if frozenset((a, b)) in allowed:
            continue
        vol = _vol(parts[a], parts[b])
        if vol > 1e-3:
            out.append((a, b, round(vol, 4)))
    return out


@pytest.fixture(scope="module")
def side():
    """The Klann single, one side, Chicago pins on standoff pillars (from the cache)."""
    cfg = _config()
    tmpl, design = cache.cached_design(cfg)
    build = Build(design.ctx, design.plan, tmpl.freeze_at(T))
    fab = cache.cached_side(cfg, T)
    axles = [g for g in design.groups if isinstance(g, AxleGroup)]
    return tmpl, design, build, fab, axles


@pytest.fixture(scope="module")
def jansen():
    return cache.cached_design(_config(linkage="jansen"))


def _group_bodies(fab, group):
    stem = group.name.replace(":", "_") + "_"
    return [b for b in fab.bodies if b.name.startswith(stem)]


# -- dims -----------------------------------------------------------------------


def test_registry_and_cli_list_the_constructions(capsys):
    from spiderpig.build import _list_options

    assert set(construction.AXLES) == set(KEYS)
    for key in KEYS:
        assert construction.axle(key).key == key
    assert construction.axle("chicago").roles == ("pin",)
    assert construction.axle("standoff").roles == ("pillar",)
    _list_options()
    out = capsys.readouterr().out
    for key in KEYS:
        assert f"  {key:14} " in out


def test_dims_are_honest(side):
    _, design, *_ = side
    ctx = design.ctx
    p = ctx.params
    pin = construction.axle("chicago").dims(ctx, False)
    assert pin.axle == pytest.approx(2.0)                 # the 4 mm barrel
    assert pin.neck == pytest.approx((4.0 + 0.2) / 2 + p.min_wall)   # a printed ring round it
    assert pin.neck <= pin.spacer == pytest.approx(p.spacer_d / 2)
    assert pin.head == pytest.approx(8.5 / 2)             # the barrel's and screw's heads
    assert all(h > 0 for h in pin.end_h)                  # each end fits a clearance gap
    assert pin.washer == pytest.approx(washer_od(4.0) / 2)
    pillar = construction.axle("standoff").dims(ctx, True)
    assert pillar.axle == pytest.approx(3.0)              # the 6 mm standoff
    assert pillar.neck == pytest.approx((6.0 + 0.35) / 2 + p.min_wall)
    assert pillar.neck <= pillar.spacer
    assert pillar.head == pytest.approx(9.0 / 2)          # the M4 washer under the button head
    assert pillar.end_h == (0.0, 0.0)                     # a head outside each plate
    assert pillar.washer == pytest.approx(washer_od(6.0) / 2)


def test_constructions_refuse_what_they_cannot_build(side):
    _, design, *_ = side
    ctx = design.ctx
    chicago, standoff = construction.axle("chicago"), construction.axle("standoff")
    with pytest.raises(ConstructionError, match="link pin only"):
        chicago.dims(ctx, True)
    with pytest.raises(ConstructionError, match="pillar only"):
        standoff.dims(ctx, False)
    thin = replace(ctx, params=Params(link_radius=3.0))
    with pytest.raises(ConstructionError, match="leaves less than"):
        chicago.dims(thin, False)                        # no link left around the hole
    with pytest.raises(ConstructionError, match="leaves less than"):
        standoff.dims(thin, True)
    with pytest.raises(ConstructionError, match="don't fit a 1.5 mm layer"):
        chicago.dims(replace(ctx, pitch=1.5), False)     # a head doesn't fit a layer
    with pytest.raises(ConstructionError, match="don't fit a 1.5 mm layer"):
        standoff.dims(replace(ctx, pitch=1.5), True)
    with pytest.raises(ConstructionError, match="spacer ring"):
        chicago.dims(replace(ctx, params=Params(spacer_d=5.0)), False)
    with pytest.raises(ConstructionError, match="frame plate arms"):
        standoff.dims(replace(ctx, params=Params(frame_radius=2.5)), True)
    chicago.dims(ctx, False)
    standoff.dims(ctx, True)


# -- the contract, the plan, the clashes ---------------------------------------------


@pytest.mark.slow
@pytest.mark.parametrize("t", [0.0, 4.38])
def test_parts_stay_inside_their_claims(side, t):
    """The whole side (the fast tier checks the axles alone, below)."""
    tmpl, design, *_ = side
    assert check_side(design, tmpl.freeze_at(t)) == []


def test_the_axles_stay_inside_their_claims(side):
    tmpl, design, *_, axles = side
    assert check_side(design, tmpl.freeze_at(4.38), groups=[g.name for g in axles]) == []


def test_plan_verifies(side):
    tmpl, design, *_ = side
    assert verify_plan(design.plan, tmpl) == []


@pytest.mark.slow
def test_jansen_single_builds_with_the_default_pivots(jansen):
    tmpl, design = jansen
    assert verify_plan(design.plan, tmpl) == []
    assert check_side(design, tmpl.freeze_at(T)) == []


def test_single_side_parts_do_not_intersect(side):
    *_, fab, _ = side
    assert _clashes(fab) == []


def test_every_part_is_one_valid_solid(side):
    *_, fab, _ = side
    for b in fab.bodies:
        if b.part is not None:
            assert len(b.part.solids()) == 1, b.name
            assert b.part.is_valid, b.name


# -- what is built ---------------------------------------------------------------------


def test_holes_cut_for_each_construction(side):
    """A pin: its running fit in every link but the lowest, which is bonded to the barrel
    (a closer hole); a pillar: its running fit in its links and the end screw's clearance
    hole in each frame plate it is anchored in."""
    _, design, build, _, axles = side
    p = design.ctx.params
    plates = {0: FRAME_OUTER, build.top: FRAME_INNER}
    for g in axles:
        got = g.realize(build, Realized())
        c = g.construction
        if g.pillar:
            anchors = Column.of(build, g).anchors
            assert anchors
            assert set(got.cuts) == set(g.axis.members) | {plates[k] for k in anchors}
            for m in g.axis.members:
                assert [x.d for x in got.cuts[m]] == [pytest.approx(p.hole(6.0))]
            col = Column.of(build, g)
            lo, hi = c.faces(col.links, build.top, (0 in anchors, build.top in anchors))
            axle = c.column_axle(set(col.links), hi, build.ctx.pitch, lo)
            for k in anchors:
                assert [x.d for x in got.cuts[plates[k]]] == [pytest.approx(axle.end_hole)]
        else:
            host = min(g.axis.members, key=lambda m: (build.layers[m], m))
            assert set(got.cuts) == set(g.axis.members)
            assert [x.d for x in got.cuts[host]] == [pytest.approx(c.shaft.host_hole())]
            for m in set(g.axis.members) - {host}:
                assert [x.d for x in got.cuts[m]] == [pytest.approx(c.hole())]
            assert {e.key for e in got.extras} == {"epoxy_2part", "threadlocker_222"}


def _assert_nothing_slides(side, which) -> list[str]:
    """From the bottom retainer to the top one each axle's stack is continuous: links,
    rings, printed spacers, gap spacers, heads and washers, and the plates it is anchored
    in; the shaft (the Chicago screw, the standoff with its end screws) runs through all of
    it. ``which(group, column)`` picks the axles; returns their names."""
    _, design, build, fab, axles = side
    # a 3.0 mm acrylic part in a layer an aluminium plate thickens to 3.175 mm leaves the
    # difference; the columns' play (a printed spacer's tolerance, a barrel's take-up)
    plan = design.plan
    play = 0.25 + max([plan.t(k) - plan.spec.pitch for k in range(plan.top + 1)] + [0.0]) + 1e-6
    seen = []
    for g in axles:
        col = Column.of(build, g)
        if not which(g, col):
            continue
        seen.append(g.name)
        spans = [build.z(k) for k in col.links] + [build.z(k) for k in col.anchors]
        shafts = []
        for b in _group_bodies(fab, g):
            bb = b.part.bounding_box()
            if "_screw" in b.name or "_standoff" in b.name:
                shafts.append((bb.min.Z, bb.max.Z))
            else:
                spans.append((bb.min.Z, bb.max.Z))
        spans.sort()
        lo, hi = spans[0][0], max(z1 for _, z1 in spans)
        reach = lo
        for z0, z1 in spans:
            assert z0 <= reach + play, (g.name, z0, reach)
            reach = max(reach, z1)
        assert reach == pytest.approx(hi)
        # the shaft runs through: nothing of the stack is off it
        s0, s1 = min(z0 for z0, _ in shafts), max(z1 for _, z1 in shafts)
        assert s0 < lo + 1e-6, g.name
        assert s1 > hi - 1e-6, g.name
    return seen


def test_nothing_on_a_pin_or_a_two_plate_pillar_can_slide(side):
    """The Chicago pins, and the standoff pillars anchored in both frame plates."""
    seen = _assert_nothing_slides(
        side, lambda g, col: not g.pillar or len(col.anchors) == 2)
    assert any(n.startswith("pin:") for n in seen)
    assert any(n.startswith("pillar:") for n in seen)


@pytest.mark.xfail(strict=True, reason=(
    "engine bug found by W2's re-pointing (in 93dfe31 too; the removed pivots' designs hid "
    "it): a cantilever standoff pillar leaves the clearance gap over its last link empty "
    "(AxleGroup.claims puts washers in range(k0, k1) only): Klann single pillar B's b2 "
    "slides 4.0 mm. Fixing it changes the kept designs' parts: a user decision"))
def test_nothing_on_a_cantilever_pillar_can_slide(side):
    """A standoff pillar a link's sweep stops short of one frame plate."""
    assert _assert_nothing_slides(side, lambda g, col: g.pillar and len(col.anchors) == 1)


def test_spacers_and_retainers_are_what_the_key_says(side):
    _, design, build, fab, axles = side
    for g in axles:
        bodies = {b.name.split("_", 2)[2]: b for b in _group_bodies(fab, g)}
        col = Column.of(build, g)
        rings = {n for n in bodies if n.startswith("ring")}
        assert rings == {f"ring{k}" for k in col.between}, g.name
        printed = {n for n, b in bodies.items() if b.fab == "printed"}
        assert rings <= printed
        assert {f"gap{k}_spacer" for k in col.washers} <= set(bodies), g.name
        if g.pillar:
            (standoff,) = [b for n, b in bodies.items() if n.startswith("standoff")]
            assert standoff.bom_key.startswith(("gobilda_1501_", "pillar_shaft_"))
            # a button head and washer at each end, on the plates or the free end's layer
            ends = sorted(int(n[len("screw"):]) for n in bodies if n.startswith("screw"))
            assert ends == sorted(int(n[len("washer"):]) for n in bodies
                                  if n.startswith("washer"))
            assert len(ends) == 2
            assert set(col.anchors) <= set(ends)
        else:
            screw = bodies["screw"]
            assert screw.bom_key.startswith("chicago_m3_")
            host = min(g.axis.members, key=lambda m: (build.layers[m], m))
            assert screw.rigid_with == host                  # bonded into the lowest link
            assert {n for n in bodies if n.startswith("spacer_")} <= printed


def test_bom_counts_per_construction(side):
    _, design, build, fab, axles = side
    bom = bom_from_mechanism(fab, group=False)
    rows = {r.key: r for r in bom.purchased}
    for r in bom.purchased:
        assert get(r.key).offers, r.key
        assert r.url.startswith("https://")
    pins = [g for g in axles if not g.pillar]
    pillars = [g for g in axles if g.pillar]
    assert pins
    assert pillars
    assert sum(r.qty for k, r in rows.items() if k.startswith("chicago_m3_")) == len(pins)
    standoffs = sum(r.qty for k, r in rows.items()
                    if k.startswith(("gobilda_1501_", "pillar_shaft_")))
    assert standoffs == len(pillars)                         # one piece each, never spliced
    washers = sum(1 for b in fab.bodies if b.name.startswith("pillar_")
                  and b.bom_key in ("m4_washer", "m3_washer_9021"))
    assert washers == 2 * len(pillars)
    for key in ("epoxy_2part", "threadlocker_222"):
        assert key in rows, key


@pytest.mark.slow
def test_robot_bom_doubles_the_side(side):
    tmpl, design, *_ = side
    robot = fabricate(tmpl, replace(design.config, robot=True), T)
    assert _clashes(robot, robot.meta["fastened"]) == []
    rows = {r.key: r for r in bom_from_mechanism(robot, group=False).purchased}
    side_rows = {r.key: r for r in bom_from_mechanism(fabricate(tmpl, design.config, T),
                                                      group=False).purchased}
    chicago = [k for k in side_rows if k.startswith("chicago_m3_")]
    assert chicago
    for k in chicago:
        assert rows[k].qty == 2 * side_rows[k].qty, k


# -- the default pin --------------------------------------------------------------------------


def test_the_default_pin_is_the_chicago_screw_on_standoff_pillars():
    """The pivot review's decision (:mod:`construction.pivots.chicago`): Chicago screw pins
    between the links; since 2026-10-03 standoff pillars and the bolt crank (the crank
    study, :class:`construction.crank.BoltCrank`); the default's key carries no hash. The
    rod, printed pillars and the keyed crank were removed on 2026-10-07: asking for one
    names its replacement."""
    cfg = BuildConfig()
    assert (cfg.linkage, cfg.module) == ("strider", "double")
    assert (cfg.pin, cfg.pillar, cfg.crank) == ("chicago", "standoff", "bolt")
    assert cfg.key == "strider_double_robot"
    assert BuildConfig(crank="bolt_round").key != cfg.key
    assert BuildConfig(linkage="klann", module="quad").pin == "chicago"
    with pytest.raises(ParamError, match="chicago"):
        BuildConfig(pin="rod")
    with pytest.raises(ParamError, match="standoff"):
        BuildConfig(pillar="printed")
