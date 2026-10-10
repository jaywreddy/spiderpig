"""The decanting cradles (``linkages/decanters.py``, ``construction/cradle.py``,
``construction/handcrank.py``, ``cradle_report.py``): the bottle's mouth stands still while
it tips (A from -25°, where an opened full bottle holds, to +55°; the study B from -10°),
both bottles lie with their mouths on the pivot, both sides plan with the hand crank, every
part stays in its claims, the whole cradle is clash-free over the swing (the hand crank's
sweep and a 3 L bottle too), it stands, the hand turns it, the pins hold, the cut files
pass the cut rules, and the guide has an order for every part."""

from __future__ import annotations

import math

import numpy as np
import pytest

from spiderpig import linkage
from spiderpig.config import BuildConfig, ParamError, design_from_query, is_cradle
from spiderpig.construction import cradle as cr
from spiderpig.construction.handcrank import NRV030, HandCrank
from spiderpig.linkages.decanters import TIES
from tests.tiers import quick

CRADLES = ("decanter_hinge", "decanter_rcm")
REST = {"decanter_hinge": -25.0, "decanter_rcm": -10.0}
TRANSMISSION = {"decanter_hinge": 40.0, "decanter_rcm": 30.0}   # least, from a toggle
HEIGHT = {"decanter_hinge": 70.0, "decanter_rcm": 80.0}         # a side's stack, mm


def _tilt(key: str, ts) -> np.ndarray:
    lk = linkage.get(key)
    pts = lk.solve().evaluate(ts)
    o, ax = lk.output.frame
    d = pts[ax] - pts[o]
    return np.degrees(np.unwrap(np.arctan2(d[:, 1], d[:, 0])))


@pytest.mark.linkage
@pytest.mark.parametrize("key", CRADLES)
def test_the_mouth_stands_still_while_the_bottle_tips(key):
    lk = linkage.get(key)
    assert lk.product == "cradle"
    assert lk.kind == "mechanism"
    got = lk.assert_output()
    assert got.wander_mm is not None
    assert got.wander_mm < 1e-6
    tilt = _tilt(key, np.linspace(0, 2 * math.pi, 1441))
    assert tilt.min() == pytest.approx(REST[key], abs=0.1)
    assert tilt.max() == pytest.approx(55.0, abs=0.1)
    # every loop closes with room and away from a toggle
    for s in lk.check():
        if s.kind == "closure":
            assert s.margin_mm > 5.0, s.describe()
            assert min(s.angle_deg[0], 180 - s.angle_deg[1]) >= TRANSMISSION[key] - 1e-6, \
                s.describe()


@pytest.mark.linkage
@pytest.mark.parametrize("key", CRADLES)
def test_the_ties_ride_the_cradle_about_the_mouth(key):
    """Each tie point is where its cradle-frame coordinates put it, turned about the mouth."""
    lk = linkage.get(key)
    ts = np.linspace(0, 2 * math.pi, 73)
    pts = lk.solve().evaluate(ts)
    lip = pts[lk.output.still[0]]
    a = np.radians(_tilt(key, ts))
    for name, (cx, cy) in TIES.items():
        want = lip + np.stack([np.cos(a) * cx - np.sin(a) * cy,
                               np.sin(a) * cx + np.cos(a) * cy], -1)
        assert np.abs(pts[name] - want).max() < 1e-9


@pytest.mark.linkage
def test_a_still_point_is_a_promise(monkeypatch):
    """``Output.still`` breaks loudly when the point wanders (the B cradle's still point
    moved from the mouth to a corner of the cradle), and must be a joint of the output."""
    from spiderpig.linkage import Output, engine

    lk = linkage.get("decanter_rcm")

    def moved(key: str, still: tuple[str, float]) -> linkage.Linkage:
        return linkage.Linkage(key=key, name=key, params=lk.params, program=lk.program,
                               links=lk.links, frame=lk.frame, crank=lk.crank,
                               angles=lk.angles,
                               output=Output("body", "b6", "rotation", ("Lc", "X"),
                                             still=still))

    corner = moved("x_still", ("X", 1e-6))
    monkeypatch.setitem(engine.REGISTRY, corner.key, corner)
    c = corner.output_check()
    assert c.wander_mm > 50.0
    assert "X" in c.broken
    with pytest.raises(ValueError, match="must be joints"):
        moved("x_still2", ("O1", 1))


@pytest.mark.linkage
def test_cradle_defaults_and_the_hand_crank_only_for_cradles():
    cfg = BuildConfig(linkage="decanter_hinge", robot=False)
    assert is_cradle(cfg)
    assert (cfg.crank, cfg.sheet, cfg.frame_sheet, cfg.crank_sheet) == (
        "hand", "al5052_3p2mm", "al5052_4p8mm", "al6061_3p2mm")
    assert (cfg.pin, cfg.pillar, cfg.params.link_radius) == ("chicago6", "standoff", 11.0)
    from spiderpig.materials import link_sheets

    assert link_sheets(cfg) == {"b1": "al5052_4p8mm"}      # (b2: the default sheet's)
    with pytest.raises(ParamError, match="the hand crank builds a decanting cradle"):
        BuildConfig(linkage="hoecken", robot=False, crank="hand")
    with pytest.raises(ParamError, match="the hand crank builds a decanting cradle"):
        BuildConfig(linkage="decanter_hinge", robot=False, crank="bolt")
    with pytest.raises(ParamError, match="it builds one side"):
        BuildConfig(linkage="decanter_hinge", robot=True)


@pytest.mark.linkage
def test_a_fit_override_keeps_the_rest_of_the_products_and_a_walker_drops_them():
    """The product's dimensions merge per field (one override keeps the rest); a design
    moved from a cradle to a walker gets the walker's own sheets, pin and dimensions."""
    from spiderpig.fit import Params

    cfg = BuildConfig(linkage="decanter_hinge", robot=False, params=Params(margin=0.6))
    assert cfg.params.margin == 0.6
    assert (cfg.params.link_radius, cfg.params.frame_radius) == (11.0, 12.0)
    walker = design_from_query({"linkage": "strider"}, base=cfg)
    plain = BuildConfig()
    assert (walker.pin, walker.sheet, walker.frame_sheet) == (plain.pin, plain.sheet,
                                                               plain.frame_sheet)
    assert walker.params.link_radius == plain.params.link_radius
    assert walker.params.margin == 0.6                  # (the override is the design's own)


@pytest.mark.linkage
def test_the_worm_holds_and_both_bottles_lie_with_their_mouths_on_the_pivot():
    assert NRV030.irreversible()
    assert NRV030.ratio == 30
    for size in cr.SIZES:
        assert cr.seat(size) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9), size.name
    # the magnum on its adapters, the 3 L in its collar; a magnum straight in the 3 L's
    # saddles would lie nose up, its mouth off the pivot
    assert cr.adapter(cr.MAGNUM)[:2] == pytest.approx((3.5, 21.0))
    assert cr.adapter(cr.JEROBOAM) == pytest.approx((0.0, 0.0, 20.0))
    y0, slope, x0 = cr.seat(cr.MAGNUM, adapted=False)
    assert math.degrees(-slope) > 5.0
    assert math.hypot(x0, y0) > 1.0


@pytest.mark.planner
@pytest.mark.parametrize("key", CRADLES)
@pytest.mark.usefixtures("fresh_plan_memo")
def test_a_cradle_side_plans_with_its_link_under_the_inner_plate(key):
    from spiderpig.fabricate import design_side, template_for

    cfg = BuildConfig(linkage=key, robot=False)
    design = design_side(template_for(cfg), cfg)
    plan = design.plan
    assert plan.optimal, plan.proof
    assert plan.layers[cr.cradle_link(cfg.lk)] == plan.top - 1
    assert HandCrank.rider(design.ctx) == "b1"
    assert plan.height < HEIGHT[key]     # each side; the bay between them is cradle.BAY


@pytest.mark.construction
@pytest.mark.parametrize(("key", "t"), quick(
    [(k, t) for k in CRADLES for t in (0.0, 2.2)], [("decanter_hinge", 2.2)]))
def test_a_cradle_side_stays_inside_its_claims(key, t):
    from spiderpig.construction.contract import check_side
    from spiderpig.fabricate import design_side, template_for

    cfg = BuildConfig(linkage=key, robot=False)
    tmpl = template_for(cfg)
    design = design_side(tmpl, cfg)
    assert check_side(design, tmpl.freeze_at(t)) == []


@pytest.mark.slow
@pytest.mark.construction
@pytest.mark.parametrize("key", CRADLES)
def test_the_whole_cradle_is_clash_free_over_the_swing(key):
    """Both sides, the box, the stand on its base plate, the bottle and the decanter at
    rest, at full tilt and across the turn: no two parts meet, the hand crank's sweep meets
    nothing, and a 3 L bottle in the saddles (its collar, no adapters) meets nothing."""
    from spiderpig.construction.contract import bad_solids, clashes
    from spiderpig.cradle_report import wheel_sweep
    from spiderpig.fabricate import fabricate, template_for

    cfg = BuildConfig(linkage=key, robot=False)
    tmpl = template_for(cfg)
    ts = np.linspace(0, 2 * math.pi, 721)
    tilt = _tilt(key, ts)
    angles = [ts[int(np.argmin(tilt))], ts[int(np.argmax(tilt))], 1.0, 3.5, 5.2]
    for t in angles:
        mech = fabricate(tmpl, cfg, float(t))
        assert clashes(mech) == [], t
        assert wheel_sweep(mech) == [], t
        assert cr.jeroboam_hits(mech) == [], t
    assert bad_solids(mech) == []
    names = {b.name for b in mech.bodies}
    assert {"L.gearbox", "bottle", "decanter", "led", "front_saddle", "rear_saddle",
            "base_plate", "jeroboam_collar"} <= names
    assert not any(n.startswith("R.") and ("gearbox" in n or n == "R.b1") for n in names)


@pytest.mark.slow
@pytest.mark.construction
@pytest.mark.parametrize("key", CRADLES)
def test_the_cradle_passes_the_cut_rules(key):
    """Every laser-cut part against SendCutSend's rules for its sheet: no errors (the least
    hole a sheet's thickness, the hole-to-edge web, the part sizes)."""
    from spiderpig import manufacture
    from spiderpig.fabricate import fabricate, template_for

    cfg = BuildConfig(linkage=key, robot=False)
    mech = fabricate(template_for(cfg), cfg, 1.0)
    got = manufacture.summary(manufacture.check(mech, cfg.sheet, dxf=False))
    assert got["ok"], got


@pytest.mark.slow
@pytest.mark.strength
def test_the_cradle_stands_turns_easily_and_holds():
    """A, the one to build: the mouth still for both bottles, it stands at 15° and more,
    the hand turns it with under 25 N, the worm holds, the rod, the crankpin and every pin
    and plate hold a sealed full 3 L bottle 3 x over and the gearbox's rating jammed."""
    from spiderpig.cradle_report import report

    r = report(BuildConfig(linkage="decanter_hinge", robot=False), clash_angles=0)
    assert r["swing"]["mouth_wander_mm"] < 1e-6
    assert max(b["mouth_wander_mm"] for b in r["bottles"]) < 0.5
    assert r["stand"]["worst_tip_deg"] >= 15.0
    assert min(c["push_at_top_n"] for c in r["stand"]["cases"]) >= 25.0
    assert r["drive"]["irreversible"]
    # (against its 20 N·m continuous rating at 1400 rpm input; the peak is a sealed full 3 L
    # at one tilt, at hand speed)
    assert r["drive"]["gearbox_sf"] >= 1.3
    assert r["drive"]["handle_force_n"] < 25.0
    assert r["rod"]["sf_bottle"] >= 3.0
    assert r["rod"]["sf_jam"] >= 1.5
    assert r["crankpin"]["sf_bottle"] >= 3.0
    assert r["crankpin"]["sf_jam"] >= 1.5
    rows = r["strength"]["rows"]
    walk = [x["walk_sf"] for x in rows if x["walk_sf"] is not None and x["joint"] != "crank"]
    assert min(walk) >= 3.0, [(x["joint"], x["walk_sf"]) for x in rows]
    # the arm's keyway on the shaft's key: twice the gearbox's rating (its jam row)
    (crank,) = [x for x in rows if x["joint"] == "crank"]
    assert crank["walk_sf"] >= 2.5
    jam = [x["jam_sf"] for x in rows if x["jam_sf"] is not None]
    assert min(jam) >= 1.4, [(x["joint"], x["jam_sf"]) for x in rows]


@pytest.mark.slow
@pytest.mark.strength
def test_the_study_b_stands_and_holds_a_bottle():
    """B, a study (``docs/agentlib/DECANTER.md``): it stands, the hand turns it and every
    joint holds a sealed full 3 L bottle; its jam case is below A's (the record says)."""
    from spiderpig.cradle_report import report

    r = report(BuildConfig(linkage="decanter_rcm", robot=False), clash_angles=0)
    assert r["stand"]["worst_tip_deg"] >= 15.0
    assert r["drive"]["handle_force_n"] < 25.0
    walk = [x["walk_sf"] for x in r["strength"]["rows"] if x["walk_sf"] is not None]
    assert min(walk) >= 2.5


@pytest.mark.slow
@pytest.mark.construction
@pytest.mark.parametrize("key", CRADLES)
def test_the_cradle_has_an_assembly_order(key):
    """The guide's order (``CRADLE_ORDER``) covers every part once: the gearbox under the
    drive side's outer plate first, the drive side's cradle link with its tie rods on the
    bench, the passive side's cradle link onto the rods on the cradle (not in its stack),
    the base plate and the hand crank at the end, the bottle and decanter last."""
    from spiderpig.construction.assembly import FINISH, UNIT, USE, assembly_steps
    from spiderpig.fabricate import design_side, product, template_for

    cfg = BuildConfig(linkage=key, robot=False)
    tmpl = template_for(cfg)
    design = design_side(tmpl, cfg)
    steps = assembly_steps(product(design, tmpl, 1.0, robot=False), design)
    link = cr.cradle_link(cfg.lk)
    first = steps[0]
    assert first.sub
    assert {"L.gearbox", "L.crank_shaft"} <= set(first.adds)
    (ties,) = [s for s in steps if f"L.{link}" in s.adds]
    assert ties.sub
    assert {"tie_Tf1", "tie_Tr2"} <= set(ties.adds)
    (passive,) = [s for s in steps if f"R.{link}" in s.adds]
    assert passive.stage == UNIT
    assert passive.side == "R"
    (base,) = [s for s in steps if "base_plate" in s.adds]
    assert base.stage == FINISH
    assert steps[-1].stage == USE
    assert set(steps[-1].adds) == {"bottle", "decanter"}
    assert steps.index(passive) > steps.index(ties)
