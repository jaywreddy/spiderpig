"""The decanting cradles (``linkages/decanters.py``, ``construction/cradle.py``,
``construction/handcrank.py``, ``cradle_report.py``): the bottle's mouth stands still while
it tips (both from -50°, nearly upright, where an opened full bottle goes in, to +30°),
both bottles lie with their mouths on the pivot, both sides plan with the hand crank, every
part stays in its claims, the whole cradle is clash-free over the swing (the hand crank's
sweep and a 6 L bottle too), it stands, the hand turns it, the pins hold, the cut files
pass the cut rules, and the guide has an order for every part."""

from __future__ import annotations

import math

import numpy as np
import pytest

from spiderpig import linkage
from spiderpig.config import BuildConfig, ParamError, design_from_query, is_cradle
from spiderpig.construction import cradle as cr
from spiderpig.construction.handcrank import HandCrank
from spiderpig.linkages.decanters import TIES
from tests.tiers import quick

CRADLES = ("decanter_hinge", "decanter_rcm")
REST = {"decanter_hinge": -50.0, "decanter_rcm": -50.0}
FULL = {"decanter_hinge": 30.0, "decanter_rcm": 30.0}
TRANSMISSION = {"decanter_hinge": 38.0, "decanter_rcm": 36.0}   # least, from a toggle
HEIGHT = {"decanter_hinge": 70.0, "decanter_rcm": 85.0}         # a side's stack, mm


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
    assert tilt.max() == pytest.approx(FULL[key], abs=0.1)
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
    assert (cfg.pin, cfg.pillar, cfg.params.link_radius) == ("pivot8", "standoff", 12.0)
    assert BuildConfig(linkage="decanter_hinge", robot=False, pin="chicago6").pin == "chicago6"
    from spiderpig.materials import link_sheets

    assert link_sheets(cfg) == {"b1": "al5052_6p35mm", "b2": "al5052_4p8mm"}
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
    assert (cfg.params.link_radius, cfg.params.frame_radius) == (12.0, 12.0)
    walker = design_from_query({"linkage": "strider"}, base=cfg)
    plain = BuildConfig()
    assert (walker.pin, walker.sheet, walker.frame_sheet) == (plain.pin, plain.sheet,
                                                               plain.frame_sheet)
    assert walker.params.link_radius == plain.params.link_radius
    assert walker.params.margin == 0.6                  # (the override is the design's own)


@pytest.mark.linkage
def test_the_worm_holds_and_every_bottle_lies_with_its_mouth_on_the_pivot():
    from spiderpig.construction.handcrank import NRV040

    assert NRV040.irreversible()
    assert NRV040.ratio == 30
    assert [z.key for z in cr.SIZES] == ["standard", "magnum", "jeroboam", "imperial",
                                         "methuselah"]
    # the saddles sized for the fattest neck and body, the longest bottle the base stop's
    assert cr.DESIGN is cr.METHUSELAH
    assert all(cr.DESIGN.r >= z.r and cr.DESIGN.r_neck >= z.r_neck for z in cr.SIZES)
    assert max(z.length for z in cr.SIZES) < cr.SPINE_END
    for size in cr.SIZES:
        assert cr.seat(size) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9), size.name
    # every bottle but the 6 L methuselah on its adapters, the longer necks in collars; a
    # magnum straight in the 6 L's saddles would lie nose up, its mouth off the pivot
    assert cr.adapter(cr.MAGNUM)[:2] == pytest.approx((8.5, 37.5))
    assert cr.adapter(cr.METHUSELAH) == pytest.approx((0.0, 0.0, 43.0))
    # the 6 L's centre of mass between the saddles: lying level or tipped, it bears on both
    front, rear = cr.SADDLES
    for size in cr.SIZES:
        assert front.x[1] < size.com_from_mouth < rear.x[1], size.name
    # the 750 ml: its neck short of the stop, so its front liner ends at its shoulder (the
    # stop); the rear one closed behind its base, inside the rear saddle
    a_f, a_r, collar = cr.adapter(cr.STANDARD)
    assert collar < 0
    bb = cr._adapter(front, a_f, cr.STANDARD).bounding_box()
    assert pytest.approx(cr.STANDARD.neck) == bb.max.X
    assert cr.STANDARD.length < rear.x[1]
    shell = cr._adapter(rear, a_r, cr.STANDARD)
    assert pytest.approx(rear.x[1]) == shell.bounding_box().max.X
    y0, slope, x0 = cr.seat(cr.MAGNUM, adapted=False)
    assert math.degrees(-slope) > 5.0
    assert math.hypot(x0, y0) > 1.0


@pytest.mark.planner
@pytest.mark.parametrize("key", quick(CRADLES, ["decanter_hinge"]))
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


@pytest.mark.planner
@pytest.mark.usefixtures("fresh_plan_memo")
def test_a_thicker_link_sheet_plans_too():
    """The hand crank's search estimates the layers over its keyed plates at the pitch (no
    thinner than the plan's: the search's claims are never stricter than the final
    layout's), so A in 0.187 in links plans."""
    from spiderpig.fabricate import design_side, template_for

    cfg = BuildConfig(linkage="decanter_hinge", robot=False, sheet="al5052_4p8mm")
    design = design_side(template_for(cfg), cfg)
    assert design.plan.optimal, design.plan.proof


@pytest.mark.api
@pytest.mark.no_fabricate
def test_a_cradle_never_takes_another_engines_plan(monkeypatch):
    """A product re-plans rather than take a stored plan of another engine, its own or a
    same-spec design's (``stages.planning._reuse_plan``): a valid but unproven, thicker
    stack once built a cradle."""
    from types import SimpleNamespace

    from spiderpig.stages import planning

    def never(*_a, **_k):
        raise AssertionError("a product took another engine's plan")

    monkeypatch.setattr(planning, "_remake_plan", never)
    cfg = BuildConfig(linkage="decanter_hinge", robot=False)
    for own in (None, {"ok": True, "engine_version": "0.1.0+old"}):
        store = SimpleNamespace(read_report=lambda _id, _kind, own=own: own, find_plans=never)
        design = SimpleNamespace(store=store, id="x", engine_version="0.1.0+new", config=cfg)
        assert planning._reuse_plan(design) is None     # type: ignore[arg-type]


@pytest.mark.hardware
def test_a_cradles_cutting_estimate_is_the_same_grouped_or_not():
    """The cradle's cutting fit discounts a file cut twice or more (``CRADLE_QTY_FACTOR``):
    the same whether the BOM groups its parts (build) or not (verify, audit)."""
    from spiderpig.hardware.bom import CRADLE_CUT_FITS, cut_estimate

    for key, fit in CRADLE_CUT_FITS.items():
        grouped = cut_estimate(key, [(120.0, 2), (35.5, 3), (60.0, 1)], fit)
        assert cut_estimate(key, [(120.0, 1), (35.5, 1), (60.0, 1), (120.0, 1), (35.5, 1),
                                  (35.5, 1)], fit) == grouped


def test_a_cradle_warns_when_its_own_default_replaces_a_named_one():
    """A spec that names a walker's default the cradle replaces is told so."""
    from spiderpig.stages.resolve import resolve

    d = resolve({"kind": "mechanism", "linkage": {"key": "decanter_hinge"},
                 "materials": {"sheet": "acrylic_3mm"}}, store=None)
    assert d.config.sheet == "al5052_3p2mm"
    assert any("materials.sheet acrylic_3mm" in w for w in d.warnings)


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


@pytest.mark.construction
def test_a_cradle_pin_is_a_hardened_pivot_pin_made_to_its_stack():
    """The pins (``PIVOT8``): MISUMI's hardened 8 mm pivot pin, an E-ring at each end, L
    made to its stack to the 0.1 mm (no take-up, its play the least and under a step more);
    every link's hole the running fit (the M6 Chicago barrel's 8.2 mm), nothing bonded,
    threaded or glued; rated as a solid 8 mm rod at class 8.8's yield."""
    import math

    from spiderpig.construction.axle import AxleGroup
    from spiderpig.construction.base import Build, Realized
    from spiderpig.construction.pivots.chicago import CHICAGO_M6
    from spiderpig.construction.pivots.pivot_pin import PIVOT8
    from spiderpig.fabricate import design_side, template_for
    from spiderpig.hardware.catalog import get

    cfg = BuildConfig(linkage="decanter_hinge", robot=False)
    tmpl = template_for(cfg)
    design = design_side(tmpl, cfg)
    build = Build(design.ctx, design.plan, tmpl.freeze_at(1.0))
    (pin,) = [g for g in design.groups if isinstance(g, AxleGroup) and not g.pillar]
    assert pin.construction.key == PIVOT8.key
    assert PIVOT8.hole() == pytest.approx(CHICAGO_M6.hole())
    got = pin.realize(build, Realized())
    assert set(got.cuts) == set(pin.axis.members)
    for m in pin.axis.members:
        assert [x.d for x in got.cuts[m]] == [pytest.approx(PIVOT8.hole())]
    assert got.extras == []                     # no epoxy, no threadlocker
    note = got.notes["pivot_pin"][pin.name]
    shaft = pin.construction.shaft
    assert shaft.min_play - 1e-9 <= note["play_mm"] < shaft.min_play + 0.1
    assert note["length_mm"] == pytest.approx(note["stack_mm"] + note["play_mm"])
    (body,) = [b for b in got.bodies if b.name.endswith("_pin")]
    assert body.bom_key == note["item"]
    assert get(body.bom_key).offer.sku == f"CCGH8-{note['length_mm']:g}-NC2.4"
    section = got.notes["wobble"][pin.name]["section"]
    assert section["yield_mpa"] == 640.0
    assert section["z_bend_mm3"] == pytest.approx(math.pi * 8 ** 3 / 32, abs=1e-3)


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_a_pivot_pin_fits_its_stack_and_its_ends_a_layer():
    """A pivot pin takes the length its stack and least play need, to the 0.1 mm, and no
    shims; its ends (2.4 mm, the play over the top one) sink into a 3.175 mm layer; it is a
    pin only, and an end taller than a clearance gap is refused."""
    from dataclasses import replace

    from spiderpig.construction import ConstructionError
    from spiderpig.construction.pivots.pivot_pin import PIVOT8
    from spiderpig.stack import GAP_MAX

    shaft = PIVOT8.shaft
    f = shaft.fit(12.83, 3.175)
    assert (f.length, f.shims) == (pytest.approx(13.1), 0.0)
    assert f.play == pytest.approx(0.27)
    assert shaft.item()["screw_head_h"] + f.play < 3.175
    assert max(shaft.base_heights()) < 3.175    # (with the clearance a gap keeps)
    ctx = type("Ctx", (), {"pitch": 3.175})()
    shaft.check(ctx, False)
    with pytest.raises(ConstructionError, match="link pin only"):
        shaft.check(ctx, True)
    with pytest.raises(ConstructionError, match="clearance gap"):
        replace(shaft, min_play=GAP_MAX).check(ctx, False)


@pytest.mark.construction
def test_a_tie_rod_that_would_sweep_into_the_inner_plate_fails():
    """The planner checks nothing in the inner plate's layer (a frame plate's), so the ties
    check their paths over the turn against its members: one across a tie's arc fails."""
    from spiderpig.construction.base import FRAME_INNER, Build, ConstructionError, Realized
    from spiderpig.fabricate import design_side, template_for

    cfg = BuildConfig(linkage="decanter_hinge", robot=False)
    tmpl = template_for(cfg)
    design = design_side(tmpl, cfg)
    build = Build(design.ctx, design.plan, tmpl.freeze_at(1.0))
    ties = cr.tie_points(design.plan.topo, cfg.lk)
    done = Realized()
    cr._ties_clear_the_inner_plate(build, done, ties)       # no plate: nothing to meet
    path = np.asarray(design.plan.topo.geometry.points[ties["Tf1"]], float)
    a, b = path[0], path[len(path) // 3]
    done.pad(FRAME_INNER, (*a, 0.0), (*b, 0.0), 5.0)
    with pytest.raises(ConstructionError, match="tie rod Tf1"):
        cr._ties_clear_the_inner_plate(build, done, ties)


@pytest.mark.slow
@pytest.mark.construction
@pytest.mark.parametrize("key", CRADLES)
def test_the_whole_cradle_is_clash_free_over_the_swing(key):
    """Both sides, the box, the stand on its base plate, the bottle and the decanter at
    rest, at full tilt and across the turn: no two parts meet, the hand crank's sweep meets
    nothing, and a 6 L methuselah in the saddles (its collar, no adapters) meets nothing."""
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
        assert cr.bottle_hits(mech) == [], t
    assert bad_solids(mech) == []
    names = {b.name for b in mech.bodies}
    assert {"L.gearbox", "bottle", "decanter", "led", "front_saddle", "rear_saddle",
            "base_plate", "jeroboam_collar", "methuselah_collar"} <= names
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
    """A, the one to build: the mouth still for both bottles, it stands at 15° and more and
    takes a 25 N push at the bottle's top (the worst over the swing), the hand turns it with
    under 25 N, the worm holds, the rod, the crankpin and every pin and plate hold a sealed
    full 6 L bottle 3 x over and a firm 40 N push at the grip on a blocked cradle (the jam:
    ``cradle_report.jam_nm``, ``config.PRODUCT_JAM_NM``)."""
    from spiderpig.config import PRODUCT_JAM_NM
    from spiderpig.construction.handcrank import handle_r
    from spiderpig.cradle_report import JAM_HAND_N, jam_nm, report

    for key in CRADLES:
        cfg = BuildConfig(linkage=key, robot=False)
        assert PRODUCT_JAM_NM[key] == pytest.approx(jam_nm(grip_r=handle_r(cfg)), abs=0.05)

    r = report(BuildConfig(linkage="decanter_hinge", robot=False), clash_angles=0)
    assert r["swing"]["mouth_wander_mm"] < 1e-6
    assert max(b["mouth_wander_mm"] for b in r["bottles"]) < 0.5
    assert r["stand"]["worst_tip_deg"] >= 15.0
    assert min(c["push_at_top_n"] for c in r["stand"]["cases"]) >= 25.0
    assert r["drive"]["irreversible"]
    # (against its 45 N·m rating; the peak is a sealed full 6 L at one tilt, at hand speed)
    assert r["drive"]["gearbox_sf"] >= 1.3
    assert r["drive"]["handle_force_n"] < 25.0
    assert r["rod"]["sf_bottle"] >= 3.0
    assert r["rod"]["sf_jam"] >= 1.5
    assert r["crankpin"]["sf_bottle"] >= 3.0
    assert r["crankpin"]["sf_jam"] >= 1.5
    rows = r["strength"]["rows"]
    walk = [x["walk_sf"] for x in rows if x["walk_sf"] is not None and x["joint"] != "crank"]
    assert min(walk) >= 3.0, [(x["joint"], x["walk_sf"]) for x in rows]
    # the keyed plates on the shaft's key (the crank's row)
    (crank,) = [x for x in rows if x["joint"] == "crank"]
    assert crank["walk_sf"] >= 2.5
    assert crank["jam_sf"] >= 1.5
    assert r["drive"]["jam_hand_force_n"] == JAM_HAND_N
    # every joint holds the jam; pin E, the M6 Chicago screw between the rod and the cradle
    # link, least (it yields at about 1.3 x the jam's push: 54 N at the grip)
    jam = [x["jam_sf"] for x in rows if x["jam_sf"] is not None]
    assert min(jam) >= 1.3, [(x["joint"], x["jam_sf"]) for x in rows]


@pytest.mark.slow
@pytest.mark.strength
def test_b_stands_turns_easily_and_holds():
    """B, the virtual hinge (``docs/agentlib/DECANTER.md``): the mouth still for both
    bottles, it stands at 25° and takes a 25 N push at the bottle's top (the worst over the
    swing), the hand turns it with under 25 N, the worm holds within its rating, the rod and
    the crankpin hold the jam, every pin, pillar and plate holds a sealed full 6 L bottle 2 x
    over and a firm 40 N push at the grip on a blocked cradle 1.3 x over."""
    from spiderpig.cradle_report import report

    r = report(BuildConfig(linkage="decanter_rcm", robot=False), clash_angles=0)
    assert r["swing"]["mouth_wander_mm"] < 1e-6
    assert max(b["mouth_wander_mm"] for b in r["bottles"]) < 0.5
    assert r["stand"]["worst_tip_deg"] >= 25.0
    assert min(c["push_at_top_n"] for c in r["stand"]["cases"]) >= 25.0
    assert r["drive"]["irreversible"]
    assert r["drive"]["gearbox_sf"] >= 1.3
    assert r["drive"]["handle_force_n"] < 25.0
    assert r["rod"]["sf_jam"] >= 1.5
    assert r["crankpin"]["sf_jam"] >= 1.5
    rows = r["strength"]["rows"]
    walk = [x["walk_sf"] for x in rows if x["walk_sf"] is not None]
    assert min(walk) >= 2.0, [(x["joint"], x["walk_sf"]) for x in rows]
    jam = [x["jam_sf"] for x in rows if x["jam_sf"] is not None]
    assert min(jam) >= 1.3, [(x["joint"], x["jam_sf"]) for x in rows]


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
    # every carriage bolt goes in from under the base plate, the passive side's too
    bolts = [a for s in steps for a in s.adds if a.startswith("foot_bracket") and
             a.endswith("_screw1")]
    assert len(bolts) == len(set(bolts)) > 3
    assert set(bolts) <= set(base.adds)
    # the hand crank right after the base plate (to turn the cradle for the checks), all
    # of it in its step
    (hand,) = [s for s in steps if "L.handle" in s.adds]
    assert steps.index(hand) == steps.index(base) + 1
    assert {b for s in steps for b in s.adds if b.startswith("L.handle")} <= set(hand.adds)
    assert steps[-1].stage == USE
    assert set(steps[-1].adds) == {"bottle", "decanter", "decanter_riser"}
    assert steps.index(passive) > steps.index(ties)
    # the drive side on the base plate before the box; the passive inner plate after the
    # passive stack has met its cradle link; the tie screws after the link
    (saddles,) = [s for s in steps if "front_saddle" in s.adds]
    assert steps.index(base) < steps.index(saddles)
    inner = next(s for s in steps if s.side == "R" and any(a.startswith("R.") and
                                                           a.endswith("torso") for a in s.adds))
    stack = [s for s in steps if s.side == "R" and s.stage == "stack"]
    assert stack
    assert steps.index(inner) > max(steps.index(s) for s in stack)
    (screws,) = [s for s in steps if any(a.startswith("tie_") and a.endswith("screwR")
                                         for a in s.adds)]
    assert steps.index(screws) >= steps.index(passive)
    text = " ".join(t for s in steps for t in s.text)
    if not cr.stand_of(cfg.lk).bay_ties:        # (A has none)
        assert "bay tie" not in text, text


@pytest.mark.slow
@pytest.mark.construction
def test_the_cradle_builds_an_order_without_a_walkers_words(tmp_path):
    """``spiderpig build`` of A (slow: the whole machine's STEP): its BOM's title is the
    cradle's, ORDER.md has no acrylic note, no servo boxes, the TPU parts are the cradle's,
    and nothing is bought for a Chicago pin's epoxy."""
    from spiderpig.build import main

    assert main(["--linkage", "decanter_hinge", "--side-only", "--out", str(tmp_path),
                 "--force"]) == 0
    order = (tmp_path / "ORDER.md").read_text()
    assert "decanting cradle" in order.splitlines()[0]
    for walker in ("acrylic arrives", "servo boxes", "TPU socks", "epoxy"):
        assert walker not in order, walker


@pytest.mark.slow
@pytest.mark.api
def test_verify_rates_a_cradles_joints_at_its_own_loads():
    """``api.verify`` rates a cradle's joints at its own statics (cradle_report.loads), as an
    audit does: A and B pass, every joint at jam SF 1.3 or more."""
    from spiderpig import api

    d = api.resolve({"kind": "mechanism", "linkage": {"key": "decanter_hinge"}}, store=None)
    q = api.verify(d, "quick")      # the default level: a catalog floor with no servo in it
    (floor,) = [r for r in q.rows if r.requirement == "budget.cost_floor_usd"]
    assert "NRV040" in floor.detail
    assert "servo" not in floor.detail.lower()
    assert not [r for r in q.rows if r.requirement == "size.mass_g"]   # a build weighs it
    a = api.verify(d, "standard")
    assert a.ok, [f.message for f in a.failures]
    (joints,) = [r for r in a.rows if r.requirement == "strength.joints"]
    assert joints.value >= 1.3
    b = api.verify(api.resolve({"kind": "mechanism", "linkage": {"key": "decanter_rcm"}},
                               store=None), "standard")
    assert b.ok, [f.message for f in b.failures]
    (joints,) = [r for r in b.rows if r.requirement == "strength.joints"]
    assert joints.value >= 1.3


@pytest.mark.slow
@pytest.mark.construction
def test_every_printed_part_fits_a_print_bed_in_its_pose():
    """Each printed part in its print pose (``meta["print_poses"]``, :func:`build.print_pose`)
    fits a 250 x 210 x 210 mm bed, and the parts not lying in a layer have a pose."""
    from spiderpig.build import print_pose
    from spiderpig.fabricate import fabricate, template_for

    cfg = BuildConfig(linkage="decanter_hinge", robot=False)
    mech = fabricate(template_for(cfg), cfg, 1.0)
    poses = mech.meta["print_poses"]
    for name in ("front_saddle", "rear_saddle", "front_saddle_liner", "L.handle",
                 "jeroboam_collar", "foot_boot_0"):
        assert name in poses, name
    for b in mech.bodies:
        if b.fab != "printed" or b.part is None:
            continue
        bb = print_pose(b.part, poses.get(b.name)).bounding_box()
        x, y = sorted((bb.size.X, bb.size.Y))
        assert y <= 250.0, (b.name, y, x, bb.size.Z)
        assert x <= 210.0, (b.name, y, x, bb.size.Z)
        assert bb.size.Z <= 210.0, (b.name, y, x, bb.size.Z)


@pytest.mark.linkage
@pytest.mark.parametrize("key", CRADLES)
def test_the_cradle_pages_swing_is_the_linkages(key):
    """The cradle page's swing (``cradle_report.motion``, ``viewer/decant.html``): rest and
    full tilt where the linkage puts them, half the worm's turns between them, the mouth
    still, and the cradle's frame as the ties ride it."""
    from spiderpig.cradle_report import motion

    m = motion(BuildConfig(linkage=key, robot=False), 72)
    assert m["rest_deg"] == pytest.approx(REST[key], abs=0.5)
    assert m["full_deg"] == pytest.approx(FULL[key], abs=0.5)
    assert m["turns_to_full"] == pytest.approx(m["ratio"] / 2, abs=1.0)
    assert m["mouth_wander_mm"] < 1e-6
    assert len(m["tilt_deg"]) == len(m["points"][m["mouth"]]) == 72
    i = m["full"]
    a = math.radians(m["tilt_deg"][i])
    lip = np.asarray(m["points"][m["mouth"]][i])
    for name, (cx, cy) in m["ties"].items():
        want = lip + np.array([math.cos(a) * cx - math.sin(a) * cy,
                               math.sin(a) * cx + math.cos(a) * cy])
        assert np.abs(np.asarray(m["points"][name][i]) - want).max() < 0.05, name


@pytest.mark.server
def test_the_cradle_page_lists_the_cradles_and_serves_their_swing(monkeypatch):
    from fastapi.testclient import TestClient

    from spiderpig.server import app as srv

    monkeypatch.delenv("VITE_ALLOWED_HOSTS", raising=False)
    client = TestClient(srv.app, base_url="http://localhost")
    cards = client.get("/api/cradles").json()["cradles"]
    assert [c["key"] for c in cards] == list(CRADLES)
    assert {c["key"]: c["verdict"] for c in cards} == {"decanter_hinge": "build",
                                                        "decanter_rcm": "build"}
    r = client.get("/api/cradle/decanter_hinge?n=48")
    assert r.status_code == 200
    assert r.json()["n"] == 48
    assert client.get("/api/cradle/strider").status_code == 404


def _corners(lo, hi) -> np.ndarray:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return np.array([[(lo, hi)[i & 1][0], (lo, hi)[(i >> 1) & 1][1], (lo, hi)[(i >> 2) & 1][2]]
                     for i in range(8)])


def _packed_ok(boxes: list[tuple[int, str, np.ndarray, np.ndarray]], lo, hi,
               gap: float = 0.0) -> None:
    """Packed parts' boxes (piece index in insertion order, name, min, max): inside
    ``lo``..``hi``, two of different pieces at least ``gap`` apart along some axis, and of
    two that overlap in plan the lower one's piece in first."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    for _, n, a, b in boxes:
        assert (a >= lo - 0.05).all(), (n, a, lo)
        assert (b <= hi + 0.05).all(), (n, b, hi)
    for x, (i, n, a, b) in enumerate(boxes):
        for j, m, c, d in boxes[x + 1:]:
            if i == j:
                continue
            ov = np.minimum(b, d) - np.maximum(a, c)
            assert (ov <= -gap + 0.05).any(), (n, m, ov)
            if ov[0] > 0.05 and ov[2] > 0.05:          # over each other: the lower one first
                first, second = (i, j) if b[1] <= c[1] + 0.05 else (j, i)
                assert first < second, (n, m)


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_packer_levels_drops_and_fills_in_order():
    """``cradle_pack.pack`` on boxes: a base board stays at the bottom, as it is; two panels
    (a plate on posts) lie flat, one on the other; a bar turned 30° in its plane lies
    levelled (its own size, not its turned box's) and flat; small blocks fill
    in; no two pieces meet (``CLEAR`` apart), all inside the box, each piece in after what
    is under it. ``level`` finds the bar's turn; ``tuck`` swings a link hanging off its
    panel in; the insertion order refuses a piece both over and under another."""
    from spiderpig.cradle_pack import CLEAR, Placed, _Order, level, pack, tuck

    turn = np.array([[math.cos(math.radians(30)), -math.sin(math.radians(30)), 0.0],
                     [math.sin(math.radians(30)), math.cos(math.radians(30)), 0.0],
                     [0.0, 0.0, 1.0]])
    bar = [_corners((-100, -10, -2), (100, 10, 2)) @ turn.T]
    assert level(bar) == pytest.approx(-30.0)
    assert level(bar, tilt=30.0) == pytest.approx(-30.0)

    def panel(z0: float) -> list[np.ndarray]:          # a plate in xy, four posts off it
        posts = [_corners((x, y, z0 + 5), (x + 10, y + 10, z0 + 45))
                 for x, y in ((-140, 10), (130, 10), (-140, 180), (60, 120))]
        return [_corners((-150, 0, z0), (150, 200, z0 + 5)), *posts]

    items = [("base", "base", [_corners((-200, -3, -160), (200, 0, 160)),
                               _corners((-190, 0, -150), (-180, 15, -140))]),
             ("p1", "panel", panel(0.0)), ("p2", "panel", panel(80.0)), ("bar", "fill", bar)]
    items += [(f"block{i}", "fill", [_corners((0, 0, 0), (40, 25, 12))]) for i in range(5)]
    res = pack(items)
    assert sorted(res.order) == sorted(k for k, _, _ in items)
    assert res.order[0] == "base"
    assert np.allclose(res.placed["base"].matrix(), np.eye(4))    # the board stays put
    for k in ("p1", "p2"):                                         # flat: the plate level
        plate = res.placed[k].boxes[0]
        assert plate[1, 1] - plate[0, 1] == pytest.approx(5.0)
    b = res.placed["bar"].boxes[0]
    assert sorted(np.round(b[1] - b[0], 6)) == pytest.approx([4.0, 20.0, 200.0])
    assert b[1, 1] - b[0, 1] == pytest.approx(4.0)                 # flat
    for i in range(5):
        blk = res.placed[f"block{i}"].boxes[0]
        assert blk[1, 1] - blk[0, 1] == pytest.approx(12.0)
    rank = {k: i for i, k in enumerate(res.order)}
    _packed_ok([(rank[k], f"{k}[{j}]", bx[0], bx[1]) for k, p in res.placed.items()
                for j, bx in enumerate(p.boxes)], *res.inside, gap=CLEAR - 0.1)
    # the box: the board (3 + the 15 mm post on it), two 50 mm panels, the gaps
    assert res.inside[1, 1] - res.inside[0, 1] <= 15 + 2 * 50 + 4 * CLEAR + 2 * CLEAR + 4.0

    # a link hanging off its panel's top swings in, about its pin, clear of the plate's posts
    plate = panel(0.0)
    link = [_corners((95, 150, 10), (105, 300, 15))]
    deg = tuck(link, (100.0, 155.0), plate)
    assert deg == pytest.approx(90.0)          # in, square to the plate, the least swing
    assert tuck([_corners((95, 50, 10), (105, 150, 15))], (100.0, 55.0), plate) == 0.0

    # a piece can't go in between two parts of another
    order = _Order()
    two = np.array([[[0, 0, 0], [100, 10, 100]], [[0, 50, 0], [100, 60, 100]]], float)
    order.add(Placed("u", np.eye(3), np.zeros(3), two), (set(), set()))
    assert order.relations(np.array([[[10, 20, 10], [50, 30, 50]]], float)) is None
    assert order.relations(np.array([[[10, 70, 10], [50, 80, 50]]], float)) == ({0}, set())


PACK_BOX = {"decanter_hinge": (665.0, 310.0, 590.0), "decanter_rcm": (650.0, 325.0, 555.0)}
"""Each cradle's flat-pack box, outside (x, height, z; mm): at most (since the 6 L: its rear
saddle and two rear liners, about 104 mm thick, lie over the panels)."""
PACK_JOINTS = {"decanter_hinge": ("Pin E ", "pillar G"), "decanter_rcm": ("Pins X and Y ", None)}
"""Where each cradle link comes off its side: its pins and its pillar (the steps name them)."""


@pytest.mark.slow
@pytest.mark.construction
@pytest.mark.parametrize("key", CRADLES)
def test_the_cradle_packs_flat_in_one_box(key):
    """``cradle_pack.flat_pack`` (a fabrication at crank angle 0): every part but the display
    ones in exactly one piece; each part's packed bounding box (the part moved by its piece's
    transform, or its own) inside the box and clear of every other piece's (a levelled piece
    turns by any angle, so its parts' boxes are the moved parts', not their assembled boxes'
    corners moved); of two pieces over each other the lower one goes in first; the box no
    bigger than :data:`PACK_BOX`; the steps name where the cradle link comes off its side."""
    from spiderpig.cradle_pack import flat_pack
    from spiderpig.fabricate import fabricate, template_for
    from spiderpig.mechanism import Pose
    from spiderpig.shapes import moved

    cfg = BuildConfig(linkage=key, robot=False)
    mech = fabricate(template_for(cfg), cfg, 0.0)
    p = flat_pack(mech, cfg.lk)
    shipped = {b.name: b for b in mech.bodies if b.part is not None and b.fab != "display"}
    names = [n for m in p["modules"] for n in m["parts"]]
    assert sorted(names) == sorted(shipped)                 # each in exactly one piece
    assert len({m["key"] for m in p["modules"]}) == len(p["modules"])
    boxes = []
    for i, m in enumerate(p["modules"]):
        for n in m["parts"]:
            mat = np.array(m.get("part_matrix", {}).get(n, m["matrix"]), float)
            assert np.allclose(mat[:3, :3] @ mat[:3, :3].T, np.eye(3), atol=1e-5)
            bb = moved(shipped[n].placed_part(), Pose(mat).to_location()).bounding_box()
            boxes.append((i, n, np.array([bb.min.X, bb.min.Y, bb.min.Z]),
                          np.array([bb.max.X, bb.max.Y, bb.max.Z])))
    lo = np.asarray(p["box"]["floor"], float)
    _packed_ok(boxes, lo, lo + np.asarray(p["box"]["inside_mm"], float))
    assert all(np.asarray(p["box"]["outside_mm"]) <= PACK_BOX[key]), p["box"]
    assert p["modules"][0]["key"] == "base"
    pins, pillar = PACK_JOINTS[key]
    text = " ".join(p["steps"])
    assert pins in text, p["steps"]
    assert (pillar in text) if pillar else ("pillar" not in text), p["steps"]
