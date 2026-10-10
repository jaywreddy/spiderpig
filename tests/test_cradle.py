"""The decanting cradles (``linkages/decanters.py``, ``construction/cradle.py``,
``construction/handcrank.py``, ``cradle_report.py``): the bottle's mouth stands still while
it tips -10° to +55°, both sides plan with the hand crank, every part stays in its claims,
the whole cradle is clash-free over the swing, it stands, the hand turns it, the pins hold."""

from __future__ import annotations

import math

import numpy as np
import pytest

from spiderpig import linkage
from spiderpig.config import BuildConfig, ParamError, is_cradle
from spiderpig.construction import cradle as cr
from spiderpig.construction.handcrank import NRV030, HandCrank
from spiderpig.linkages.decanters import TIES
from tests.tiers import quick

CRADLES = ("decanter_hinge", "decanter_rcm")


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
    assert tilt.min() == pytest.approx(-10.0, abs=0.1)
    assert tilt.max() == pytest.approx(55.0, abs=0.1)
    # every loop closes with room and away from a toggle
    for s in lk.check():
        if s.kind == "closure":
            assert s.margin_mm > 5.0, s.describe()
            assert min(s.angle_deg[0], 180 - s.angle_deg[1]) >= 30.0 - 1e-6, s.describe()


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
    assert (cfg.crank, cfg.frame_sheet, cfg.crank_sheet) == ("hand", "al5052_3p2mm",
                                                             "al6061_6p35mm")
    assert (cfg.pin, cfg.pillar, cfg.params.link_radius) == ("chicago6", "standoff", 11.0)
    from spiderpig.materials import link_sheets

    assert link_sheets(cfg) == {"b1": "al5052_4p8mm", "b2": "al5052_3p2mm"}
    with pytest.raises(ParamError, match="the hand crank builds a decanting cradle"):
        BuildConfig(linkage="hoecken", robot=False, crank="hand")
    with pytest.raises(ParamError, match="the hand crank builds a decanting cradle"):
        BuildConfig(linkage="decanter_hinge", robot=False, crank="bolt")
    with pytest.raises(ParamError, match="it builds one side"):
        BuildConfig(linkage="decanter_hinge", robot=True)


@pytest.mark.linkage
def test_the_worm_holds_and_the_bottles_sit():
    assert NRV030.irreversible()
    assert NRV030.ratio == 30
    magnum = cr.seat(cr.MAGNUM)
    jeroboam = cr.seat(cr.JEROBOAM)
    # the jeroboam's axis through the pivot; a magnum's within a few mm of it
    assert abs(jeroboam[0]) < 0.1
    assert abs(jeroboam[1]) < 0.01
    assert math.hypot(magnum[2], magnum[0]) < 4.0


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
    assert plan.height < 70.0            # each side; the bay between them is cradle.BAY


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
    """Both sides, the box, the stand, the bottle and the decanter at rest, at full tilt
    and across the turn: no two parts meet."""
    from spiderpig.construction.contract import bad_solids, clashes
    from spiderpig.fabricate import fabricate, template_for

    cfg = BuildConfig(linkage=key, robot=False)
    tmpl = template_for(cfg)
    ts = np.linspace(0, 2 * math.pi, 721)
    tilt = _tilt(key, ts)
    angles = [ts[int(np.argmin(tilt))], ts[int(np.argmax(tilt))], 1.0, 3.5, 5.2]
    for t in angles:
        mech = fabricate(tmpl, cfg, float(t))
        assert clashes(mech) == [], t
    assert bad_solids(mech) == []
    names = {b.name for b in mech.bodies}
    assert {"L.gearbox", "bottle", "decanter", "led", "front_saddle", "rear_saddle"} <= names
    assert not any(n.startswith("R.") and ("gearbox" in n or n == "R.b1") for n in names)


@pytest.mark.slow
@pytest.mark.strength
@pytest.mark.parametrize("key", CRADLES)
def test_the_cradle_stands_turns_easily_and_holds(key):
    from spiderpig.cradle_report import report

    r = report(BuildConfig(linkage=key, robot=False), clash_angles=0)
    assert r["swing"]["mouth_wander_mm"] < 1e-6
    assert r["stand"]["worst_margin_mm"] > 20.0
    assert r["drive"]["irreversible"]
    assert r["drive"]["gearbox_sf"] >= 2.0
    assert r["drive"]["handle_force_n"] < 25.0
    assert r["rod"]["sf_bottle"] >= 3.0
    assert r["rod"]["sf_jam"] >= 1.5
    rows = r["strength"]["rows"]
    walk = [x["walk_sf"] for x in rows if x["walk_sf"] is not None]
    assert min(walk) >= 3.0, [(x["joint"], x["walk_sf"]) for x in rows]
    jam = [x["jam_sf"] for x in rows if x["jam_sf"] is not None]
    assert min(jam) >= 1.0, [(x["joint"], x["jam_sf"]) for x in rows]
