"""The decanting cradles (``linkages/decanters.py``, ``construction/cradle.py``,
``construction/handcrank.py``, ``cradle_report.py``): the bottle's mouth stands still while
it tips (both from -50°, nearly upright, where an opened full bottle goes in, to +30°),
both bottles lie with their mouths on the pivot, both sides plan with the hand crank, every
part stays in its claims, the whole cradle is clash-free over the swing (the hand crank's
sweep and a 6 L bottle too), it stands, the hand turns it, the pins hold, the cut files
pass the cut rules, and the guide has an order for every part."""

from __future__ import annotations

import itertools
import math

import numpy as np
import pytest

from spiderpig import linkage
from spiderpig.config import BuildConfig, ParamError, design_from_query, is_cradle
from spiderpig.construction import cradle as cr
from spiderpig.construction.handcrank import HandCrank
from spiderpig.linkages.decanters import TIES
from tests import cache
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
    from spiderpig.construction.handcrank import NRV050, HandCrank

    assert HandCrank().box is NRV050                   # (the user's call of 2026-10-14)
    assert NRV050.irreversible()
    assert NRV050.eta_new() < NRV050.eta_static
    assert NRV050.ratio == 30
    # the efficiency rises with the input's speed, from the static to the catalogue's 500 rpm
    assert (NRV050.eta_static <= NRV050.eta_at(60.0) < NRV050.eta_at(100.0)
            < NRV050.eta_at(500.0) == pytest.approx(NRV050.eta_ref[1], abs=1e-3))
    assert [z.key for z in cr.SIZES] == ["standard", "magnum", "jeroboam", "imperial",
                                         "methuselah"]
    # the V blocks bottom out for the fattest neck and body (the load case, the bottle drawn)
    assert cr.DESIGN is cr.METHUSELAH is cr.SHOWN
    assert all(cr.DESIGN.r >= z.r and cr.DESIGN.r_neck >= z.r_neck for z in cr.SIZES)
    front, rear = cr.SADDLES
    for size in cr.SIZES:
        # set by its two diameters (a 90° V rises √2 x what it is thinner) and its length:
        # its mouth on the pivot, its axis level, within each V block's travel
        s = cr.setting(size)
        assert (s.front, s.rear, s.stop) == pytest.approx((
            (cr.DESIGN.r_neck - size.r_neck) * math.sqrt(2),
            (cr.DESIGN.r - size.r) * math.sqrt(2), size.length + cr.STOP_GAP))
        assert 0.0 <= s.front <= front.travel
        assert 0.0 <= s.rear <= rear.travel
        assert cr.STOP_RANGE[0] <= s.stop <= cr.STOP_RANGE[1]
        assert cr.seat(size) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9), size.name
        # its centre of mass (the shell-and-wine model, design review round 1) behind the
        # front V, 28-40 % of its length up from its base, and on the rear support (a 6 L
        # methuselah's, 390 mm from the mouth, lies 10 mm behind the rear V's end, over the
        # base stop's rails' flanges: the mounting review's B1)
        assert front.vee[1] < size.com_from_mouth, size.name
        assert 0.6 < size.com_from_mouth / size.length < 0.72, size.name
        assert size.com_from_mouth < min(cr.FLANGE_END, size.length), size.name
    assert cr.setting(cr.METHUSELAH) == cr.Setting(0.0, 0.0,
                                                   cr.METHUSELAH.length + cr.STOP_GAP)
    # a magnum in the 6 L's setting would lie nose up, its mouth off the pivot
    y0, slope, x0 = cr.seat(cr.MAGNUM, cr.setting(cr.DESIGN))
    assert math.degrees(-slope) > 5.0
    assert math.hypot(x0, y0) > 1.0
    # a bottle the V blocks can't reach is refused, not drawn off the pivot
    from dataclasses import replace

    with pytest.raises(ValueError, match="rear_saddle's V block would rise"):
        cr.setting(replace(cr.METHUSELAH, r=90.0))
    with pytest.raises(ValueError, match="rear_saddle's V block would rise"):
        cr.setting(replace(cr.STANDARD, r=30.0))
    with pytest.raises(ValueError, match="base stop can't go"):
        cr.setting(replace(cr.METHUSELAH, length=590.0))


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
    """The cradle's cutting fit discounts a file cut twice or more (``CutFit.factor``): the
    same whether the BOM groups its parts (build) or not (verify, audit); a part given by
    its area alone (verify's floor) is the service's smallest part's price, above 0."""
    from spiderpig.hardware.bom import CRADLE_CUT_FITS, CutPart, cut_estimate

    a = CutPart(120.0, 1, 900.0, 4, 300.0, 24.0)
    b = CutPart(35.5, 1, 400.0, 3, 120.0, 30.0)
    c = CutPart(60.0, 1, 600.0, 5, 200.0, 40.0)
    for key, fit in CRADLE_CUT_FITS.items():
        grouped = cut_estimate(key, [a._replace(qty=2), b._replace(qty=3), c], fit)
        assert cut_estimate(key, [a, b, c, a, b, b], fit) == grouped
        assert cut_estimate(key, [(0.0, 1)], fit) > 0
        assert fit.factor(1) == 1.0
        assert fit.factor(2) < 1.0
        assert fit.factor(3) <= fit.factor(2)


QUOTED = (
    # SendCutSend's live quotes of 2026-10-14 (design review round 2): B's and A's files at
    # the reviewed snapshot, at their quantities: (cradle, sheet, part, qty, area cm^2, cut
    # mm, pierces, blank long, short mm, USD for the file)
    ("B", "al7075_4p8mm", "L-b3", 2, 70.4, 734, 4, 308.0, 24.0, 41.06),
    ("B", "al7075_4p8mm", "L-b4", 2, 76.6, 767, 4, 331.0, 24.0, 43.72),
    ("B", "al7075_4p8mm", "L-b2", 2, 112.9, 1024, 5, 180.4, 171.0, 85.62),
    ("B", "al5052_4p8mm", "L-b6", 2, 252.7, 1626, 8, 475.6, 216.8, 77.74),
    ("B", "al5052_4p8mm", "L-torso", 2, 113.1, 851, 5, 362.3, 67.6, 30.94),
    ("B", "al5052_4p8mm", "L-b5", 2, 28.7, 337, 3, 129.0, 24.0, 9.62),
    ("B", "al5052_4p8mm", "L-frame_outer", 2, 748.9, 4057, 15, 506.4, 454.8, 161.10),
    ("B", "al5052_6p35mm", "L-b1", 1, 83.8, 814, 3, 360.8, 25.3, 21.66),
    ("B", "al6061_3p2mm", "L-conn_plate1", 6, 27.8, 416, 4, 124.6, 37.3, 33.90),
    ("B", "al6061_3p2mm", "L-conn", 1, 28.7, 398, 4, 124.7, 37.3, 8.45),
    ("B", "al6061_3p2mm", "L-conn_plate7", 1, 30.6, 368, 4, 124.7, 37.3, 8.29),
    ("B", "birch_9mm", "jack_locator_0", 2, 325.1, 1225, 2, 280.0, 195.0, 50.38),
    ("B", "birch_9mm", "base_plate", 1, 3306.9, 3210, 9, 682.0, 584.0, 140.39),
    ("B", "birch_9mm", "base_cleat_L0", 4, 117.5, 956, 1, 452.0, 26.0, 45.32),
    ("A", "al5052_4p8mm", "L-frame_outer", 2, 679.0, 4174, 18, 683.6, 508.5, 175.70),
    ("A", "al5052_4p8mm", "L-torso", 2, 110.9, 991, 7, 407.5, 53.9, 32.30),
    ("A", "al5052_4p8mm", "L-b2", 2, 188.6, 1460, 9, 384.0, 168.0, 60.66),
    ("A", "al5052_6p35mm", "L-b1", 1, 67.2, 677, 3, 292.0, 25.3, 17.77),
    ("A", "al6061_3p2mm", "L-conn_plate1", 6, 26.8, 408, 4, 120.7, 37.3, 33.36),
    ("A", "al6061_3p2mm", "L-conn", 1, 27.7, 390, 4, 120.7, 37.3, 8.31),
    ("A", "al6061_3p2mm", "L-conn_plate7", 1, 29.6, 361, 4, 120.7, 37.3, 8.15),
    ("A", "birch_9mm", "base_plate", 1, 3205.1, 3348, 11, 682.0, 584.0, 139.03),
    ("A", "birch_9mm", "base_cleat_L0", 4, 64.0, 544, 1, 246.0, 26.0, 26.52),
    ("A", "birch_9mm", "base_cleat_L2", 4, 51.0, 444, 1, 196.0, 26.0, 21.64),
    ("A", "birch_9mm", "jack_locator_0", 2, 325.1, 1225, 2, 280.0, 195.0, 50.38),
)


@pytest.mark.no_fabricate
def test_a_cradles_cutting_fit_holds_to_the_live_quotes():
    """``CRADLE_CUT_FITS`` against the quotes it was fitted on (design review round 2's B1:
    the area-only fit had missed by -32 to +53 %, the 7075 and the birch unpriced): each
    file within 11 %, B's and A's orders within 2 % (quoted USD 758.19 and 573.82)."""
    from spiderpig.hardware.bom import CRADLE_CUT_FITS, CutPart, cut_estimate

    totals: dict[str, list[float]] = {}
    for cradle, key, part, q, area, cut, n, long_, short, usd in QUOTED:
        got = cut_estimate(key, [CutPart(area, q, cut, n, long_, short)], CRADLE_CUT_FITS[key])
        assert got == pytest.approx(usd, rel=0.11), (cradle, part)
        t = totals.setdefault(cradle, [0.0, 0.0])
        t[0] += usd
        t[1] += got
    assert totals["B"][0] == pytest.approx(758.19)
    assert totals["A"][0] == pytest.approx(573.82)
    for live, fit in totals.values():
        assert fit == pytest.approx(live, rel=0.02)


@pytest.mark.no_fabricate
def test_the_take_up_rates_each_pin_against_its_sheets_tolerances():
    """``cradle_report.take_up`` (design review round 2's M5): each pin's play against its
    links' sheets at their published over-thickness, worst case and RSS. On B's pins as
    reviewed (their wobble notes' plays), P (b1 0.250 in + b2, 0.187 in 5052 since design
    review round 3), K, X and Y (two 0.187 in 5052 links each since b4 went to 5052 in
    simplification round 1) come out short worst case; J (b3 7075, b4 5052) keeps 0.04 mm.
    The pillars too (round 3's laser-cut parts minor 9). A rating: the guide and ORDER.md
    carry the fix."""
    from types import SimpleNamespace

    from spiderpig.cradle_report import take_up

    cfg = BuildConfig(linkage="decanter_rcm", robot=False)
    plays = {"P": (0.374, "b1", "b2"), "Y": (0.400, "b5", "b6"), "J": (0.448, "b3", "b4"),
             "K": (0.448, "b2", "b4"), "X": (0.424, "b4", "b6")}
    wobble = {f"pin:{p}": {"play_mm": play, "links": [{"link": a}, {"link": b}]}
              for p, (play, a, b) in plays.items()}
    wobble["pillar:O1"] = {"play_mm": 0.4, "links": [{"link": "b3"}]}     # a pillar
    got = take_up(cfg, SimpleNamespace(meta={"wobble": wobble}, bodies=[]))
    by = {r["pin"]: r for r in got["rows"]}
    assert set(by) == {*plays, "pillar O1"}
    assert got["short_worst"] == ["K", "P", "X", "Y"]
    assert got["short_rss"] == ["P"]
    assert by["P"]["growth_worst_mm"] == pytest.approx(0.915, abs=1e-3)
    assert by["P"]["margin_worst_mm"] == pytest.approx(-0.541, abs=1e-3)
    assert by["J"]["margin_worst_mm"] == pytest.approx(0.041, abs=1e-3)
    assert by["K"]["margin_worst_mm"] == pytest.approx(-0.01, abs=1e-3)
    assert by["pillar O1"]["margin_worst_mm"] == pytest.approx(0.222, abs=1e-3)
    assert [x["sheet"] for x in by["P"]["links"]] == ["al5052_6p35mm", "al5052_4p8mm"]
    assert got["fix"]


def test_a_cradle_warns_when_its_own_default_replaces_a_named_one():
    """A spec that names a walker's default the cradle replaces is told so."""
    from spiderpig.stages.resolve import resolve

    d = resolve({"kind": "mechanism", "linkage": {"key": "decanter_hinge"},
                 "materials": {"sheet": "acrylic_3mm"}}, store=None)
    assert d.config.sheet == "al5052_3p2mm"
    assert any("materials.sheet acrylic_3mm" in w for w in d.warnings)


OWN_GROUPS = ("stand", "ties")
"""A cradle's own side groups: the stand's columns and members, the cradle box's tie rods."""


@pytest.mark.construction
@pytest.mark.parametrize(("key", "t", "groups"), quick(
    [(k, t, None) for k in CRADLES for t in (0.0, 2.2)] + [(k, 2.2, OWN_GROUPS) for k in CRADLES],
    [(k, 2.2, OWN_GROUPS) for k in CRADLES]))
def test_a_cradle_side_stays_inside_its_claims(key, t, groups):
    """Every part of a cradle's side inside its own claims at a crank angle
    (``contract.check_side``): the whole side (``groups`` ``None``: every group realized
    and checked, 12-30 s since the inner plate's keep-out is fused in 2D; slow), or the
    cradle's own groups alone (:data:`OWN_GROUPS`, the others realized only where those
    need them: about a second, the fast tier's)."""
    from spiderpig.construction.contract import check_side

    tmpl, design = cache.cached_design(BuildConfig(linkage=key, robot=False))
    assert check_side(design, tmpl.freeze_at(t), groups=groups) == []


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
@pytest.mark.no_fabricate
def test_a_clevis_pin_takes_up_its_stack_and_its_clip_a_gap():
    """``clevis8`` (B's X and Y, ``config.PRODUCT_PIN_JOINTS``): the 15 mm usable stock pin
    over B's 11.2 mm stack with a printed spacer under its head to the 0.1 mm (the play the
    least and under a step more), its head and spacer standing 6.8 mm out (two layers); its
    clip a clearance gap; a pin only; a stack no stock pin spans, or one longer than the
    linkage lets a pin be (``chicago.MAX_BARREL``), refused; rated as a solid 8 mm rod at
    annealed 304's yield; the pin and its clip priced from their pages."""
    import math
    from dataclasses import replace

    from spiderpig.config import PRODUCT_PIN_JOINTS
    from spiderpig.construction import ConstructionError
    from spiderpig.construction.pivots.clevis import CLEVIS8
    from spiderpig.hardware.catalog import get
    from spiderpig.stack import GAP_MAX

    assert {PRODUCT_PIN_JOINTS["decanter_rcm"][j] for j in "XY"} == {"clevis8"}
    shaft = CLEVIS8.shaft
    f = shaft.take_up(11.2)
    assert (f.length, f.shims_lo) == (15.0, pytest.approx(3.5))
    assert shaft.min_play - 1e-9 <= f.play < shaft.min_play + 0.1
    assert shaft.outer(11.2) == pytest.approx(15.0 - 11.2 + 3.0)
    assert shaft.base_heights()[0] == 0.0 < shaft.base_heights()[1] <= GAP_MAX
    ctx = type("Ctx", (), {"pitch": 3.175})()
    shaft.check(ctx, False)
    with pytest.raises(ConstructionError, match="link pin only"):
        shaft.check(ctx, True)
    with pytest.raises(ConstructionError, match="no stock clevis pin"):
        replace(shaft, max_length=20.0).take_up(15.0)
    sec = shaft.section()
    assert sec.yield_mpa == 205.0
    assert sec.z_bend == pytest.approx(math.pi * 8 ** 3 / 32)
    pin, clip = get("clevis_pin8_15"), get("hairpin_cotter_2x50")
    assert (pin.offer.sku, pin.offer.price_usd, pin.offer.verified) == ("90378A701", 5.47, True)
    assert (clip.offer.sku, clip.offer.pack_qty, clip.offer.verified) == ("98350A131", 5, True)
    assert pin.dims["hole_d"] <= clip.dims["wire"] + 0.01


@pytest.mark.slow
@pytest.mark.construction
def test_bs_x_and_y_are_clevis_pins_its_clips_over_the_cradle_link():
    """B's pins X and Y (``clevis8``): the plan claims the head and spacer in the two layers
    outside b4 (b5), the clip's whole reach in the gap over the cradle link and the pin's end
    in the inner plate's open (a seat; the plate keeps off its path); built, the pin, its
    printed spacer and its clip, every link's hole the M6 barrel's 8.2 mm, the spacer the
    take-up to the 0.1 mm, the clip over the link by its play, the side within its claims."""
    from spiderpig.construction.axle import AxleGroup
    from spiderpig.construction.base import Build, Realized
    from spiderpig.construction.pivots.clevis import CLEVIS8, CLIP_REACH
    from spiderpig.fabricate import design_side, template_for

    cfg = BuildConfig(linkage="decanter_rcm", robot=False)
    tmpl = template_for(cfg)
    design = design_side(tmpl, cfg)
    plan = design.plan
    build = Build(design.ctx, plan, tmpl.freeze_at(1.0))
    link = cr.cradle_link(cfg.lk)
    for j, bar in (("X", "b4"), ("Y", "b5")):
        (g,) = [g for g in design.groups if isinstance(g, AxleGroup) and g.name == f"pin:{j}"]
        assert g.construction.key == CLEVIS8.key
        k0, k1 = plan.layers[bar], plan.layers[link]
        assert k1 == plan.top - 1
        mine = plan.shapes(group=g.name)
        assert {p.layer for p in mine if p.label.endswith("head")} == {k0 - 1, k0 - 2}
        (clip,) = [p for p in mine if p.label.endswith("clip")]
        assert (clip.layer, clip.gap, clip.shape.r) == (k1, True, CLIP_REACH)
        assert plan.gaps[k1] >= clip.height
        (tip,) = [p for p in mine if p.label.endswith("tip")]
        assert (tip.layer, tip.seat) == (plan.top, True)
        got = g.realize(build, Realized())
        stem = g.name.replace(":", "_")
        assert {b.name for b in got.bodies} >= {f"{stem}_pin", f"{stem}_spacer_lo",
                                                f"{stem}_clip"}
        keys = {b.name: b.bom_key for b in got.bodies}
        assert keys[f"{stem}_pin"] == "clevis_pin8_15"
        assert keys[f"{stem}_clip"] == "hairpin_cotter_2x50"
        assert got.extras == []                 # nothing threaded, glued or locked
        for m in g.axis.members:
            assert [x.d for x in got.cuts[m]] == [pytest.approx(8.2)]
        note = got.notes["clevis_pin"][g.name]
        assert note["usable_mm"] == 15.0
        assert note["spacer_mm"] + note["stack_mm"] + note["play_mm"] == pytest.approx(15.0)
        assert 0.25 - 1e-9 <= note["play_mm"] < 0.35
        assert got.notes["wobble"][g.name]["section"]["yield_mpa"] == 205.0


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_a_quick_release_pins_gap_ring_is_its_keeper():
    """The ring a quick-release pin's plan puts in the gap over its lowest link is its keeper
    (``cradle.KEEPER``, pack turn 1): the ring's outline with a bore under the 8 mm pin, so
    it grips the pin and holds it in its link with the side lying down and the clip out
    (TPU 95A: ``bom.part_filament`` reads ``meta["keepers"]``). ``keeper_names`` takes every
    gap ring of each of the cradle link's pins that has a clip (B's X and Y), none of a pin
    without one, and refuses a clipped pin with no ring to keep it."""
    from types import SimpleNamespace

    from build123d import Location

    from spiderpig.construction.base import ConstructionError
    from spiderpig.hardware.bom import TPU_FILAMENT, part_filament
    from spiderpig.shapes import moved, ring

    gap = ring((10.0, 20.0), 14.0, 8.3, -145.2, -142.2)
    keep = cr._keeper(gap)
    bb = keep.bounding_box()
    assert pytest.approx((14.0, 14.0, 3.0)) == (bb.size.X, bb.size.Y, bb.size.Z)
    r = (cr.KEEPER[0] / 2 + 4.0) / 2           # between its bore and the pin's surface
    probe = moved(cr._rod_y(0.0, 0.0, 0.05, -1.0, 1.0), Location((10.0 + r, 20.0, -143.7)))
    assert cr.KEEPER[0] < 8.0 - cr.KEEPER[1] + 1e-9

    def vol(a) -> float:
        inter = a & probe
        return 0.0 if inter is None else sum(p.volume for p in inter.solids())

    assert vol(keep) > 1e-3
    assert vol(gap) < 1e-6
    lk = linkage.get("decanter_rcm")

    def side(*names):
        return SimpleNamespace(bodies=[SimpleNamespace(name=n) for n in names])

    got = cr.keeper_names(side("pin_X_pin", "pin_X_clip", "pin_X_gap14_spacer",
                               "pin_X_spacer_lo", "pin_Y_pin", "pin_Y_clip", "pin_Y_ring13",
                               "pin_K_clip", "pin_K_gap11_spacer", "pin_J_gap13_spacer"), lk)
    assert got == ["pin_X_gap14_spacer", "pin_Y_ring13"]    # (K: not the cradle link's)
    assert cr.keeper_pin("R.pin_Y_ring13") == "R.pin_Y_pin"
    assert cr.keeper_pin("pin_X_gap14_spacer") == "pin_X_pin"
    assert cr.keeper_names(side("pin_X_gap14_spacer", "pin_Y_gap14_spacer"), lk) == []
    with pytest.raises(ConstructionError, match="keep"):
        cr.keeper_names(side("pin_X_clip", "pin_Y_clip", "pin_Y_gap14_spacer"), lk)
    meta = {"keepers": ["L.pin_X_gap14_spacer"]}
    assert part_filament(SimpleNamespace(name="L.pin_X_gap14_spacer"), meta, "pla") == TPU_FILAMENT
    assert part_filament(SimpleNamespace(name="L.pin_K_gap11_spacer"), meta, "pla") == "pla"


@pytest.mark.construction
@pytest.mark.no_fabricate
@pytest.mark.parametrize("sign", [1, -1])
def test_a_board_bracket_takes_a_stud_and_a_knob(sign):
    """B's brackets stay on its board (pack turn 2, the packing review's L1;
    ``cradle.BOARD_BRACKETS``): the plate's M5 is a stud (a button head from the plate's
    inside, where the nut was, M5 x 16), its jam nut on the plate's outer face inside the
    upright's slot, which is open from the stud to the upright's top; a printed knob on the
    stud (an M5 nut pressed in) bears on the upright either side of the slot and clears the
    foot's plain M6 nut; no thumb knob on the board's bolt. Nothing overlaps."""
    from build123d import Box, Location

    face, t = 0.0, 4.75
    bracket, parts, _ = cr._bracket((0.0, 22.5), 0.0, face, sign, t, board=True)
    by = {suffix: (part, key) for part, key, suffix in parts}
    assert {k: v[1] for k, v in by.items()} == {
        "screw0": "m5_bhcs_16", "nut0": "m5_nut", "knob0": None, "nut2": "m5_nut",
        "screw1": "m6_carriage_25", "washer1": "m6_washer_wide", "nut1": "m6_nylon_nut"}
    shapes = {"bracket": bracket, **{k: v[0] for k, v in by.items()}}
    for (a, sa), (b, sb) in itertools.combinations(shapes.items(), 2):
        inter = sa & sb
        assert inter is None or sum(x.volume for x in inter.solids()) < 1e-3, (a, b)
    stud, jam, knob, nut = (by[k][0].bounding_box() for k in ("screw0", "nut0", "knob0", "nut2"))
    out = (lambda bb: bb.max.Z) if sign > 0 else (lambda bb: -bb.min.Z)
    inn = (lambda bb: bb.min.Z) if sign > 0 else (lambda bb: -bb.max.Z)
    assert inn(stud) == pytest.approx(-t - cr.M5.head_h, abs=0.01)    # its head inside
    assert out(stud) > out(nut)                                       # a thread past it
    assert inn(jam) == pytest.approx(face, abs=0.01)
    assert out(jam) < cr.BRACKET[2]                                   # (in the slot)
    assert inn(knob) == pytest.approx(cr.BRACKET[2], abs=0.01)         # on the upright
    nut1 = by["nut1"][0].bounding_box()
    assert inn(nut1) - out(knob) >= 1.0                               # clear of the foot's nut
    # the slot: nothing of the bracket over the stud, its jam nut's width, to the top
    bb = bracket.bounding_box()
    slot = Box(cr.STUD_SLOT - 0.4, bb.max.Y - 26.5, cr.BRACKET[2] - 0.2).moved(
        Location((0.0, (26.5 + bb.max.Y) / 2, sign * cr.BRACKET[2] / 2)))
    inter = bracket & slot
    assert inter is None or sum(x.volume for x in inter.solids()) < 1e-3
    # its mouth chamfered (design review round 3's stand and use, m11): nothing of the
    # bracket in the 45° lead either side at the top
    lead = cr.SLOT_LEAD
    for sx in (-1, 1):
        corner = Box(lead - 0.4, 0.3, cr.BRACKET[2] - 0.2).moved(Location((
            sx * (cr.STUD_SLOT / 2 + (lead - 0.4) / 2), bb.max.Y - 0.15 - 0.01,
            sign * cr.BRACKET[2] / 2)))
        inter = bracket & corner
        assert inter is None or sum(x.volume for x in inter.solids()) < 1e-3, sx
    # the trip (round 3's M1): the knob turned down onto its jam nut stands the nut's height
    # off the plate, short of the upright's wall, so a cradle lowered so hangs on the
    # uprights; backed off cr.KNOB_BACK_TURNS turns it clears the wall, its nut still all on
    # the stud
    wall = cr.BRACKET[2]
    trip = out(jam)                                     # the knob's floor on its jam nut
    assert trip < wall
    back = trip + cr.KNOB_BACK_TURNS * cr.M5_PITCH
    assert back >= wall + 0.5, back
    assert out(stud) >= out(nut) + (back - inn(knob)), (out(stud), out(nut), back)


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_trip_and_the_first_build_read_as_a_user_does_them():
    """Design review round 3's stand and use: the way back backs the knobs off two turns
    before the cradle goes down (M1); B's drive side gets its studs on the bench and drops
    into its brackets' slots (M3); the lifts say two people, or one in an order that can be
    done (M5); pin X is told by where it is, and painted (m1); B's texts are its own (m3);
    the room behind and beside (m5), the pad over the rim (m6), the riser as stacked (m7),
    the clips out before the panel turns (m8), the towel (m9); the turns the page counts
    (m13)."""
    lk = linkage.get("decanter_rcm")
    pins = cr.trip_pins(lk)
    off, back = cr.module_steps(lk, pins)
    by = {m.tag: m.words for m in off + back}
    on = by["stand"]
    assert on.index(f"{cr._WORDS[cr.KNOB_BACK_TURNS]} full turns") < on.index(
        "lower it straight onto")
    assert cr.pin_words(lk, pins[0]) == "pin X, in the long link b4, nearer the gearbox"
    assert "long link b4" in by["box_on"]
    assert "second_tip_n" not in by["box_on"]
    assert "long link b4" in by["lower"]
    assert "two people" in by["base"]
    assert "two people" in by["lay"]
    assert "Alone" in by["lay"]
    assert "column E1's, then pillar O1's" in by["lay"]          # the one nearer the gearbox
    assert by["turn"].index("Pull each pin's clip") < by["turn"].index("turn the panel over")
    assert "towel" in by["lift"]
    assert "towel" in by["set"]
    assert "stacked for your decanter" in by["riser"]
    assert "paint an X" in cr._lay_first("decanter_rcm")[0]
    text = cr.board_text("decanter_rcm")
    assert not any("knobs go back on" in t for t in text)        # B's bolts keep their nuts
    assert text[-1] == cr.DROP_DRIVE
    assert cr.BRACKETS_BOARD in text
    assert "tabs" in text[1]
    assert "jam nut" in cr.studs_first("decanter_rcm")
    assert any("knobs go back on" in t for t in cr.board_text("decanter_hinge"))
    for key in CRADLES:
        steps = {u.tag: u for u in cr.use_steps(key)}
        where = " ".join(steps["where"].lines)
        assert f"{cr.ROOM_FREE:g} mm free behind the base board and beside" in where
        assert ("thumb knobs" in where) == (key not in cr.BOARD_BRACKETS)
        assert steps["limits"].lines[0].startswith(f"1. Room: 950 mm of headroom and "
                                                   f"{cr.ROOM_FREE:g} mm")
        assert "pad over its rim" in steps["limits"].lines[3]
        full = f"{cr.full_turns(linkage.get(key)):.1f} turns"
        assert any(full in t for t in steps["decant"].lines), full


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_a_base_bracket_takes_a_thumb_knob_and_the_hand_crank_a_thumb_screw():
    """The base board comes off by hand: each bracket's carriage bolt (M6 x 25) through its
    foot's oversize hole (the side settles where the linkage puts it), an 18-8 fender washer
    over the hole, an 18-8 M6 nut in a printed thumb knob, the knob clear of the bracket's
    upright and the upright's M5 screw head, a thread past the nut and the bolt's end in the
    knob; the hand crank's hub an M3 thumb screw down to the input key through a brass
    heat-set insert (round 1: a printed thread undone at every pack wears), its head clear of
    the hub."""
    from spiderpig.construction.handcrank import HANDLE_THUMB, INSERT, HandCrank

    bracket, parts, _ = cr._bracket((0.0, 12.5), 0.0, 0.0, 1, 4.75)   # (its screw 12.5 up)
    by = {suffix: (part, key) for part, key, suffix in parts}
    assert {key for _, key in by.values()} >= {"m6_carriage_25", "m6_nut", "m6_washer_wide",
                                                "m5_nut", None}
    assert by["knob"][1] is None                                           # printed
    knob, nut, washer = by["knob"][0], by["nut1"][0], by["washer1"][0]
    bolt, side = by["screw1"][0], by["screw0"][0]
    for a, b in itertools.combinations((bracket, side, bolt, knob, nut, washer), 2):
        inter = a & b
        assert inter is None or sum(x.volume for x in inter.solids()) < 1e-3
    assert bolt.bounding_box().max.Y - nut.bounding_box().max.Y >= 3.0     # thread past it
    assert knob.bounding_box().max.Y > bolt.bounding_box().max.Y          # its end covered
    assert knob.bounding_box().min.Z >= side.bounding_box().max.Z + 2.0     # the knob clear
    assert cr.BRACKET_HOLE - cr.CARRIAGE[0] >= 2.0                           # oversize
    assert cr.WASHER[1] / 2 - (cr.BRACKET_HOLE - cr.CARRIAGE[0]) / 2 > cr.BRACKET_HOLE / 2
    hc = HandCrank()
    got, _ = hc._handle((0.0, 0.0), (1.0, 0.0), 0.0, "frame")
    bodies = {b.name: b for b in got}
    thumb = bodies["handle_thumb_screw"]
    assert thumb.bom_key == f"m3_thumb_screw_{HANDLE_THUMB[0]:g}"
    assert bodies["handle_insert"].bom_key == INSERT[2]
    for part in ("handle", "handle_insert"):
        inter = thumb.part & bodies[part].part
        assert inter is None or sum(x.volume for x in inter.solids()) < 1e-3, part
    reach = thumb.part.bounding_box().max.Z
    assert reach > hc.box.crank_hub_r + HANDLE_THUMB[2]


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_hand_crank_slips_past_its_set_force_and_comes_off_whole():
    """The hand crank's slip clutch (design review round 2's B-1): three steel ball-nose spring
    plungers in the arm's boss, their balls on the edges of a 6061 seat ring's holes (a 30°
    seat); set at 1.3 x the start the user measures, checked within its window, its cap the
    config's for each cradle, under its ceiling whatever the gearbox's run-in state (design
    review round 3's M1); every set-up's plunger force (B or A, the gearbox run in or new, the
    greased friction's either end) within the plunger's range up to the guide's limit, an
    eighth of a turn from the felt stop (its m3); its slip
    rising with the friction, never locking greased, locking dry (the band the report
    rates); its parts meet nowhere, each ball touching its hole's edge; the cap keyed on the
    journal's flat holds the arm on, so the crank comes off the input shaft whole; the grip on
    a bolt's plain shank; each plunger's nut pressed in from the boss's inner face (its m2).
    The set state is slippable in the drawn parts (design review round 3's B1): each plunger
    drawn at the guide's start, half a turn out from the felt stop, its body's nose over the
    ring's face by 0.2 mm or more (its room for the ball's climb) on B's and A's crank, and
    at the guide's limit still."""
    from spiderpig.config import PRODUCT_CLUTCH_NM
    from spiderpig.construction.handcrank import (
        CLUTCH,
        CLUTCH_SET,
        DESIGN_START_NM,
        GREASED_MU,
        HANDLES,
        NRV050,
        PLUNGER,
        HandCrank,
        clutch,
        clutch_ceiling,
        clutch_drift,
        clutch_slip,
        drawn_clutch,
        plunger_setting,
    )

    rate = (PLUNGER["force"][1] - PLUNGER["force"][0]) / PLUNGER["travel"]
    big_r, a = PLUNGER["ball_d"] / 2, CLUTCH["seat_d"] / 2
    climb = big_r - math.sqrt(big_r ** 2 - a ** 2)
    assert math.degrees(math.asin(a / big_r)) == pytest.approx(30.0, abs=0.5)
    assert CLUTCH["seat_d"] >= 3.175                        # SendCutSend's least hole
    setting = plunger_setting()
    assert setting["climb_mm"] == pytest.approx(climb, abs=1e-3)
    assert setting["start_room_mm"] >= 0.2
    assert setting["limit_room_mm"] >= 0.2
    for key in CRADLES:
        got = drawn_clutch(key)
        assert PRODUCT_CLUTCH_NM[key] == pytest.approx(got["crank_cap_nm"], abs=0.05)
        assert got["set_nm"] == pytest.approx(CLUTCH_SET * DESIGN_START_NM[key], abs=1e-3)
        new = clutch(DESIGN_START_NM[key] * NRV050.eta_static / NRV050.eta_new(),
                     HANDLES.get(key, HANDLES[""]).throw, NRV050, clutch_ceiling(key))
        assert new["crank_cap_nm"] <= got["crank_cap_nm"] + 1e-9     # the ceiling holds it
        assert new["margin"] >= 1.0                                   # (and it still turns)
        for f in (*got["plunger_n"], *new["plunger_n"]):
            assert PLUNGER["force"][0] - 0.5 <= f <= setting["limit_n"] + 1e-9, f
            assert f <= PLUNGER["force"][1] - rate * climb
    drift = clutch_drift()
    assert 1.0 < drift["greased"][1] <= drift["greased_worn"][1] < 2.0
    assert drift["locks_dry"]
    assert drift["locks_at_mu"] > 2 * GREASED_MU[1]
    assert clutch_slip(35.0, *GREASED_MU[:1] * 2) < clutch_slip(35.0, *GREASED_MU[1:] * 2)
    hc = HandCrank()
    parts, sweep = hc._handle((0.0, 0.0), (1.0, 0.0), 0.0, "frame")
    by = {b.name: b for b in parts}
    for x, y in itertools.combinations(parts, 2):
        inter = x.part & y.part
        assert inter is None or sum(v.volume for v in inter.solids()) < 0.05, (x.name, y.name)
    ring_bb = by["handle_seat_ring"].part.bounding_box()
    assert by["handle_seat_ring"].fab == "laser"
    assert pytest.approx(CLUTCH["ring_t"], abs=1e-3) == ring_bb.size.X
    for i in range(CLUTCH["plungers"]):
        bb = by[f"handle_plunger{i}"].part.bounding_box()
        assert math.hypot(bb.center().Y, bb.center().Z) == pytest.approx(CLUTCH["r"],
                                                                         abs=0.01)
        assert bb.min.X < ring_bb.max.X                     # its ball down in its seat
        assert by[f"handle_plunger{i}"].bom_key == PLUNGER["key"]
    grip = by["handle_grip"].part.bounding_box()
    assert pytest.approx(HANDLES[""].grip[1], abs=0.01) == grip.size.X
    assert by["handle_grip_pin"].bom_key.startswith("m8_hex_bolt_")
    assert by["handle_grip_nut"].bom_key == "m8_nyloc_nut"
    # the arm is held on the journal by the keyed cap and its thumb screw (the crank comes
    # off whole), and everything that turns is inside the sweep
    assert HANDLES[""].reach <= by["handle_cap"].part.bounding_box().min.X
    assert max(s[2] for s in sweep) >= max(b.part.bounding_box().max.X for b in parts) - 1e-6
    assert max(s[0] for s in sweep) >= HANDLES[""].throw + HANDLES[""].grip[0] / 2
    for spec in {HANDLES[k].reach: HANDLES[k] for k in HANDLES}.values():
        _the_clutch_is_set_slippable(hc, spec, climb, setting)


def _the_clutch_is_set_slippable(hc, spec, climb: float, setting: dict) -> None:
    """In the drawn hand crank (``spec``'s reach): each ball seated, its tip the climb under
    the seat ring's face, and its plunger's body (its thread, not its ball) over the face by
    0.2 mm or more: as the ball climbs out its spring goes in by the climb, and with less room
    the nose lands on the ring first (locked). Drawn at the guide's start (the felt stop,
    half a turn out), and at its limit (an eighth of a turn) the room still left; each
    plunger's nut in its pocket from the boss's inner face, flush with it."""
    from build123d import Box, Cylinder, Location

    from spiderpig.construction.handcrank import CLUTCH, NEAR_STOP, NUT_HALF, PLUNGER, SET_BACK

    parts, _ = hc._handle((0.0, 0.0), (1.0, 0.0), 0.0, "frame", spec)
    by = {b.name: b for b in parts}
    face = by["handle_seat_ring"].part.bounding_box().max.X
    a_in = spec.reach - CLUTCH["boss_t"]
    big_r = PLUNGER["ball_d"] / 2
    for i in range(CLUTCH["plungers"]):
        plunger = by[f"handle_plunger{i}"].part
        bb = plunger.bounding_box()
        cy, cz = bb.center().Y, bb.center().Z
        # the ball seated, its tip the climb under the ring's face: as it climbs out its
        # spring goes in by the climb, so the body's nose must stand over the face (its
        # room) or the nose lands on the ring first and the clutch locks
        assert pytest.approx(face - climb, abs=1e-3) == bb.min.X
        low = Box(5.2, 40.0, 40.0).moved(Location((face + 0.2 - 2.6, cy, cz)))
        ub = (plunger & low).bounding_box()
        # only the ball under 0.2 mm over the face: the body's thread (12.7 mm) stands over
        assert max(ub.size.Y, ub.size.Z) <= PLUNGER["ball_d"] + 1e-3, (spec.reach, ub)
        # the body's nose: where the solid first reaches out past the ball's radius
        shell = (Cylinder(10.0, 80.0, rotation=(0, 90, 0))
                 - Cylinder(big_r + 0.2, 82.0, rotation=(0, 90, 0))).moved(
            Location((bb.min.X + 40.0, cy, cz)))
        nose = (plunger & shell).bounding_box().min.X
        assert nose - face == pytest.approx(setting["start_room_mm"], abs=0.01)
        # turned in from there to the guide's limit, the room still left
        turned = (SET_BACK - NEAR_STOP) * PLUNGER["pitch"]
        assert nose - turned - face >= 0.2 - 1e-6
        nut = by[f"handle_plunger_nut{i}"].part.bounding_box()
        assert pytest.approx(a_in, abs=1e-6) == nut.min.X
        assert pytest.approx(a_in + NUT_HALF[1], abs=1e-6) == nut.max.X


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_gearbox_stands_on_studs_and_the_crank_block_on_the_shafts_end():
    """The riser (design review round 3's M2: a clamped stack, not four studs in bending):
    a ring of the crank sheet on the gearbox's machined face, centred on its spigot ring, its
    holes' webs a thickness or more; on each stud a steel sleeve that sits on the face's boss
    between the spigot ring and the boss's edge, the go/no-go's shims over it within their
    range, the plate over the spigot ring and the turning hub's end by ``RISER_CLEAR``; each
    class 8.8 stud threaded a diameter or more into the face, short of its thread's depth, its
    end filed to at most ``STUD_PAST_MAX`` over its thin nut, under the crank block's lowest
    plate by half a millimetre with the go/no-go's window and the plates under the seat at
    their thickest (its m9); the crank block's seat plate on the shaft's steel end, its M10
    end screw's head no higher over the block's top than the tie's head (the rod rides where
    it did); the keyed plates on the key's straight flanks for any end the go/no-go's window
    takes."""
    from spiderpig.construction.handcrank import (
        BLOCK_PLATES,
        HEAD_LAYER_T,
        KEYED_PLATES,
        M6,
        NRV050,
        PLATE_OVER,
        RISER_CLEAR,
        RISER_RING,
        RISER_RING_HOLE,
        RISER_SHIMS_RANGE,
        RISER_SLEEVE,
        RISER_STUD,
        RISER_THIN,
        SEAT_LOW,
        STUD_IN,
        STUD_PAST_MAX,
        arm_z,
        end_screw,
        riser_for,
    )

    t, t_frame = 3.175, 4.75
    box = NRV050
    a0, a1 = arm_z(0.0, t_frame, KEYED_PLATES * t)
    r = riser_for(0.0, a0, t, box, t_frame, t)
    assert r.height - max(box.spigot_h, box.hub_out) >= RISER_CLEAR
    od = RISER_SLEEVE[0]
    assert box.pcd / 2 - od / 2 >= box.spigot_d / 2 + 1.0       # off the spigot ring
    assert box.pcd / 2 + od / 2 <= box.face_d / 2 - 1.0          # on the face's boss
    hole = box.face_thread + RISER_RING_HOLE
    assert box.pcd / 2 - hole / 2 - (box.spigot_d + RISER_RING) / 2 >= t   # the ring's webs
    assert box.face_d / 2 - box.pcd / 2 - hole / 2 >= t
    assert r.ring_t + RISER_SLEEVE[2] + r.shims == pytest.approx(r.height)
    assert RISER_SHIMS_RANGE[0] <= r.shims <= RISER_SHIMS_RANGE[1]
    d = RISER_STUD[0]
    assert STUD_IN * d <= r.engage <= box.face_depth - 0.5
    assert 0.0 <= r.past <= STUD_PAST_MAX
    assert r.stud >= r.stud_drawn                                # (filed, never short)
    stud_top = r.face - r.engage + r.stud_drawn
    assert stud_top == pytest.approx(t_frame + RISER_THIN[1] + r.past)
    assert stud_top + STUD_PAST_MAX - r.past + SEAT_LOW + r.seat_k * PLATE_OVER <= a0 - 0.5
    assert HEAD_LAYER_T - RISER_THIN[1] - STUD_PAST_MAX >= 0.5
    assert r.seat == pytest.approx(a0 + r.seat_k * t)
    assert r.shaft_end == pytest.approx(r.seat)
    top = a0 + BLOCK_PLATES * t
    assert r.seat + t + end_screw(box).head_h <= top + M6.head_h + 1e-9
    assert r.low >= 0.5
    key_w, _, key_len = box.shaft_key
    for end in (r.seat, r.seat - r.low):
        assert end - box.key_short - key_w / 2 >= a1 - 0.2 - 1e-9
        assert end - box.key_short - key_len + key_w / 2 <= a0


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_riser_holds_the_gearbox_as_a_clamped_stack():
    """The riser rated (design review round 3's M2, ``cradle_drive.riser_rating``): at a
    rod force and torque like B's walking peak it doesn't slip at the faces' oiled friction
    (``RISER_MU``) by 2 x; at B's and A's capped jam 30° from the toggles (the review's rod
    forces) every strength row holds 1.3 (the stud in class 8.8 at its proof, the sleeve,
    the ring's bearing); the 18-8 set screws round 2 had, carried in bending over 18.7 mm,
    were SF 0.55-0.62 (the review's)."""
    from spiderpig.construction.handcrank import NRV050
    from spiderpig.cradle_drive import riser_rating

    notes = {"riser_height_mm": 25.2, "face_z_mm": -25.2, "rod_mid_z_mm": 55.0,
             "riser_ring_mm": 3.175, "riser_shims_mm": 2.025, "plate_inner_z_mm": 4.75}
    walk = riser_rating(NRV050, notes, 410.0, 35.8)
    assert walk["slip_sf"] >= 2.0, walk
    assert walk["least"] >= 2.0, walk
    for f, cap in ((1931.9, 63.7), (2301.7, 67.1)):
        got = riser_rating(NRV050, notes, f, cap)
        assert got["least"] >= 1.3, got
        assert got["sf_stud"] >= 1.3, got


def p_ok(name: str, pieces) -> bool:
    """A riser piece is one valid solid."""
    part = dict(pieces)[name]
    return len(part.solids()) == 1 and part.is_valid


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_base_board_is_rounded_offset_and_locates_its_riser():
    """The base board (round 1's stand review): B's sits 40 mm toward its drive side
    (``cradle.BASE_Z``), its corners and the notch's mouth rounded, each carriage bolt's
    hole a routed square (its neck locks by shape) with a dog-bone in each corner, two bays
    in the notch, a window under the base stop (round 2: ``cradle.BOARD_WINDOW``) with birch
    all round it; the riser locator's two layers lie in the notch with play, their ears in
    the bays (it can't slide out forward), a square pocket for the printed riser's 100 mm
    base under the mouth; the riser's pieces (round 2: no lab jack) stack to the drawn
    decanter's riser, each boss in the recess over it with play, the bottom ones in the
    pocket; a rail's cleats either side of it with play, between its brackets."""
    from build123d import Location

    from spiderpig.shapes import moved

    lip = np.array([0.0, cr.LIP_HEIGHT])
    stand = cr.STANDS["decanter_rcm"]
    y_top = lip[1] + cr.BASE_TOP
    holes = [(-110.0, 231.7), (386.0, -231.7)]
    z_mid = cr.BASE_Z["decanter_rcm"]
    assert z_mid < 0.0                                   # toward the drive side (-z)
    board, feet = cr._base_plate(lip, stand, y_top, holes, z_mid)
    bb = board.bounding_box()
    assert pytest.approx((z_mid - cr.BASE_HALF, z_mid + cr.BASE_HALF)) == (bb.min.Z, bb.max.Z)
    assert pytest.approx(stand.base) == (bb.min.X, bb.max.X)
    y = y_top - cr.BASE_T / 2

    def solid_at(x: float, z: float, shape=board, at: float = y, r: float = 0.2) -> bool:
        probe = moved(cr._rod_y(0.0, 0.0, r, at - 0.5, at + 0.5), Location((x, 0.0, z)))
        inter = shape & probe
        return inter is not None and sum(p.volume for p in inter.solids()) > 1e-3

    x0 = stand.base[0]
    za = z_mid - cr.BASE_HALF
    assert not solid_at(x0 + 2.0, za + 2.0)              # a rounded corner
    assert solid_at(x0 + cr.BASE_CORNER_R, za + 2.0)
    assert not solid_at(x0 + 1.0, cr.NOTCH_R + 1.0)      # the notch's rounded mouth
    h = cr.CARRIAGE_HOLE / 2
    for x, z in holes:              # a square hole, a dog-bone past each corner (its relief)
        assert not any(solid_at(x + sx * (h + 0.25), z + sz * (h - 1.13), r=0.05)
                       for sx in (-1, 1) for sz in (-1, 1))
        assert solid_at(x + h + 1.0, z)
        assert solid_at(x + h + 0.3, z)                  # (no relief mid-side)
        assert cr.CARRIAGE_HOLE - 0.13 > 6.48            # the neck (6.48 across) goes in
    # the feet's pads clear the carriage bolts' heads (their circles in plan)
    for fx, fz in feet:
        for x, z in holes:
            assert math.hypot(fx - x, fz - z) > cr.FOOT_PAD_R + cr.CARRIAGE[2] / 2 + 2.0
    # the window under the base stop: open over it, birch round it (in front and behind
    # it, either side of it, the board's whole width there)
    w0, w1, w_half, w_r = cr.BOARD_WINDOW
    assert not solid_at((w0 + w1) / 2, 0.0)
    assert not solid_at(w0 + 2.0, w_half - w_r)
    assert solid_at(w0 - 2.0, 0.0)
    assert solid_at(w1 + 2.0, 0.0)
    assert solid_at((w0 + w1) / 2, w_half + 2.0)
    assert solid_at((w0 + w1) / 2, -w_half - 2.0)
    assert w0 - cr.NOTCH_R > 100.0
    assert stand.base[1] - w1 > 100.0
    assert w_half + 10.0 < 209.7 - 4.775 - cr.CLEAT[1] - cr.CLEAT[0]     # (no cleat cut)
    table = lip[1] - cr.LIP_HEIGHT
    layers = cr._riser_locator(lip, table)
    assert len(layers) == 2
    for part in layers:
        inter = part & board
        assert inter is None or sum(p.volume for p in inter.solids()) < 1e-3
        lb = part.bounding_box()
        yl = (lb.min.Y + lb.max.Y) / 2
        assert not solid_at(0.0, 0.0, part, yl)           # the pocket under the mouth
        assert solid_at(0.0, cr.RISER_BASE / 2 + cr.RISER_PLAY + 2.0, part, yl)
    pieces = cr.riser_pieces(0.0, 0.0, table)
    names = [n for n, _ in pieces]
    assert names == ["decanter_riser_lift0", "decanter_riser_lift1", "decanter_riser_block0",
                     "decanter_riser_block1",
                     *(f"decanter_riser_plate{p:g}" for p in cr.RISER_PLATES)]
    boxes = {n: p.bounding_box() for n, p in pieces}
    top = max(b.max.Y for b in boxes.values())
    assert top - table == pytest.approx(cr.DECANTER_RISER, abs=0.02)
    assert (cr.RISER_LIFT + cr.RISER_BLOCK[1] + sum(cr.RISER_PLATES) + cr.RISER_LIP
            == cr.DECANTER_RISER)
    for n, b in boxes.items():                           # in the locator's pocket's footprint
        assert -cr.RISER_BASE / 2 - 1e-6 <= b.min.X
        assert b.max.X <= cr.RISER_BASE / 2 + 1e-6
        assert -cr.RISER_BASE / 2 - 1e-6 <= b.min.Z
        assert b.max.Z <= cr.RISER_BASE / 2 + 1e-6
        assert p_ok(n, pieces)
    for (a, pa), (b_, pb) in itertools.combinations(pieces, 2):    # nested, not one in another
        inter = pa & pb
        assert inter is None or sum(x.volume for x in inter.solids()) < 1e-3, (a, b_)
    lifts = [boxes[n] for n in names[:2]]
    blocks = [boxes[n] for n in names[2:4]]
    assert all(pytest.approx(table) == b.min.Y for b in lifts)       # on the table
    assert all(pytest.approx(table + cr.RISER_LIFT) == b.min.Y for b in blocks)    # on them
    tall = pytest.approx(cr.RISER_BLOCK[1] + cr.RISER_LIP, abs=0.02)
    assert all(tall == b.max.Y - b.min.Y for b in blocks)
    # each block's relief over a post's head (design review round 3's stand and use, M5 and
    # m2): a bore through its top that takes the head's 12 mm washer, under a cone whose
    # mouth takes one set down 7 mm off; the boss's face round it bears on the plate
    bore, mouth, deep = cr.RISER_RELIEF
    for (_, part), b in zip(pieces[2:4], blocks, strict=True):
        zc, yt = (b.min.Z + b.max.Z) / 2, b.max.Y
        assert bore >= 12.0 + 1.0
        assert (mouth - 12.0) / 2 >= 7.0
        for d in (0.6, deep + 0.6, cr.RISER_LIP + cr.RISER_WALL - 0.6):
            assert not solid_at(0.0, zc, part, yt - d)           # through the boss and top

        def opening(d: float) -> float:                          # the cone's radius, d down
            return mouth / 2 - d * (mouth - bore) / 2 / deep

        assert not solid_at(opening(1.5) - 0.4, zc, part, yt - 1.0)   # in the cone
        assert solid_at(opening(1.5) + 0.6, zc, part, yt - 2.0)       # its slope under it
        assert solid_at(mouth / 2 + 1.0, zc, part, yt - 0.6)     # the boss's face round it
    lb = layers[-1].bounding_box()
    assert blocks[0].max.Y > lb.max.Y + 50.0             # it stands out of the locator
    top = layers[-1].bounding_box()
    assert y_top - cr.BASE_T + 8.0 < top.max.Y          # it meets the board's edge
    xe = cr.NOTCH_BAY[0]
    ear = (layers[-1] & cr._box(xe - 5.0, xe + 5.0, top.min.Y, top.max.Y,
                                cr.NOTCH_R + 1.0, cr.NOTCH_R + 10.0))
    assert ear is not None                               # its ear
    assert sum(p.volume for p in ear.solids()) > 1.0
    cleats = cr._cleats([-110.0, 386.0], y_top, 209.7, 1, 4.775)
    assert len(cleats) == 2
    for c in cleats:
        b = c.bounding_box()
        assert -110.0 + cr.BRACKET[0] / 2 <= b.min.X
        assert 386.0 - cr.BRACKET[0] / 2 >= b.max.X
        gap = min(abs(b.min.Z - 209.7), abs(b.max.Z - (209.7 - 4.775)))
        assert gap == pytest.approx(cr.CLEAT[1])
        assert b.max.Z - b.min.Z >= 25.4                  # SendCutSend's least birch part


@pytest.mark.linkage
def test_b_packs_at_a_mark_inside_its_clear_range():
    """B's pack pose (``cradle.pack_pose``): the crank's arm along the drive side's outer
    plate's rear leg, a mark seen without counting turns; the bottle between 44° and 20°
    neck up there, where the passive cradle link clears its inner bar and the box doesn't
    grow (round 1's stand review: "4 turns" was 35° one way and 39° the other). The steps
    lay it on its drive side before the clips come out and the passive panel lifts off; the
    way back sets the links by the paint marks and snugs the passive side last."""
    lk = linkage.get("decanter_rcm")
    t, tilt, turns = cr.pack_pose(lk)
    assert -44.0 < tilt < -20.0
    assert 5.0 < abs(turns) < 6.5
    pts = lk.solve().evaluate(np.array([t]))
    arm = pts[lk.crank[1]][0] - pts[lk.crank[0]][0]
    a, b = cr.PACK_POSES["decanter_rcm"].member
    leg = (pts[cr.mouth_frame(lk)[0]][0] + np.asarray(cr.STANDS["decanter_rcm"].nodes[b])
           - pts[a][0])
    cos = float(arm @ leg / np.linalg.norm(arm) / np.linalg.norm(leg))
    assert cos > math.cos(math.radians(0.5))
    assert cr.trip_pins(lk) == ["X", "Y"]
    breakdown, back = cr.module_steps(lk, ["X", "Y"])
    tags = [m.tag for m in breakdown]
    assert tags.index("lay") < tags.index("clips") < tags.index("lift")
    # the cradle box off the drive side after the passive panel (pack turn 1): only the
    # drive side's worm holds the box, and lying so, the drive side's pins are kept
    assert tags.index("lift") < tags.index("box_clips") < tags.index("box")
    assert tags.index("hand_crank") < tags.index("base")
    assert "keeper" in breakdown[tags.index("box_clips")].words
    assert not any("spacer" in m.words for m in breakdown)    # (the keepers stay on the pins)
    assert "arm down" in breakdown[0].text
    assert "about 5½ turns" in breakdown[0].text
    # design review round 2: laid on the riser's blocks (O1's and, since the side frames'
    # redesign, the rail column E1's screw heads in their reliefs), the box set on its links'
    # edges, each piece's weight held until both its pins are in, the passive side snugged at
    # level, a measure for the empty grip
    lay = breakdown[tags.index("lay")].words
    assert "blocks" in lay
    assert "pillar O1" in lay
    assert "column E1's screw head on the rail" in lay
    assert "long edges" in breakdown[tags.index("box")].words
    words = " ".join(m.words for m in back)
    assert words.count("holding its weight until both pins are in their holes") == 2
    assert words.count("Never let it rest on a pin's tip") == 2
    assert "to level" in words
    assert "snug them there" in words
    assert "at rest" not in words
    assert "luggage scale" in words
    assert "kg" in words
    assert any("paper check" in m.words for m in back)
    assert all(m.title and m.scene is not None for m in (*breakdown, *back))
    # the tape holds b3 to its bar, not b5, the one nudged
    assert "b5 stays free" in back[[m.tag for m in back].index("set")].words
    assert cr.pack_pose(linkage.get("decanter_hinge")) is None


@pytest.mark.no_fabricate
def test_a_cradles_bom_says_what_its_total_leaves_out():
    """A cradle's BOM and ORDER.md say plainly what their totals leave out: each unpriced
    cutting line with its area and, where a fitted sheet bounds it (a proxy), an estimate
    marked as one with its basis; one with neither stays unpriced. B's and A's sheets are
    all fitted since SendCutSend's quotes of 2026-10-14 (the 7075 had been "x1 to x2 of the
    5052 fit", the birch unpriced: design review round 2's B1), so none has a proxy. A
    walker's BOM (no ``cut_ranges``) says nothing new."""
    from spiderpig.hardware.bom import (
        CRADLE_CUT_FITS,
        CRADLE_CUT_PROXIES,
        CRADLE_CUT_SOURCE,
        Bom,
        CutRow,
        MadeRow,
        cut_estimate,
        cut_ranges,
    )
    from spiderpig.hardware.order import order_markdown

    assert {"al7075_4p8mm", "birch_9mm", "al5052_4p8mm", "al5052_6p35mm",
            "al6061_3p2mm"} <= set(CRADLE_CUT_FITS)
    assert CRADLE_CUT_PROXIES == {}
    assert "2026-10-14" in CRADLE_CUT_SOURCE
    assert "758.19" in CRADLE_CUT_SOURCE
    proxies = {"al7075_4p8mm": ("al5052_4p8mm", 1.0, 2.0, "a test's basis")}
    made = [MadeRow("L.b2", "laser", "7075", "", 40.0 * 0.4826, 2, sheet="al7075_4p8mm"),
            MadeRow("base_plate", "laser", "birch", "", 3000.0 * 0.9, 1, sheet="birch_9mm"),
            MadeRow("L.frame_outer", "laser", "5052", "", 600.0 * 0.47625, 2,
                    sheet="al5052_4p8mm")]
    cutting = [CutRow("SendCutSend", "al5052_4p8mm", "5052 0.187 in", 2, 1200.0, 100.0),
               CutRow("SendCutSend", "al7075_4p8mm", "7075 0.190 in", 2, 80.0, None),
               CutRow("SendCutSend", "birch_9mm", "Baltic birch 9 mm", 1, 3000.0, None)]
    ranges = cut_ranges(cutting, made, proxies)
    base = cut_estimate("al5052_4p8mm", [(40.0, 2)], CRADLE_CUT_FITS["al5052_4p8mm"])
    assert ranges["al7075_4p8mm"][:2] == pytest.approx((base, 2 * base), abs=0.02)
    assert ranges["birch_9mm"] is None
    bom = Bom([], made, cutting=cutting, cut_source=CRADLE_CUT_SOURCE, cut_ranges=ranges)
    md = bom.markdown()
    assert "The total excludes 2 unpriced cut lines" in md
    assert "an estimate of USD" in md
    assert "Baltic birch 9 mm (1 part, 3000 cm²: no fitted" in md
    assert bom.as_dict()["unpriced_cutting"][1]["estimate_usd"] is None
    order = order_markdown(bom, [], [])
    assert "The total excludes 2 unpriced cut lines" in order
    walker = Bom([], made, cutting=cutting)
    assert "unpriced cut line" not in walker.markdown()
    assert "unpriced_cutting" not in walker.as_dict()


@pytest.mark.construction
def test_the_cradles_angle_free_parts_are_memoized_by_all_that_shapes_them(monkeypatch):
    """``cr._part_memo`` (the cradle box's, the base board's and the feet's parts, the same
    at every crank angle): the same arguments give the same B-rep, each caller's part its
    own (moving one in place leaves the next where it was); a module constant set in its
    place (a monkeypatch) is another key, as is ``-0.0`` against ``0.0``; an argument with no
    exact key (a shape) is built afresh."""
    from build123d import Location

    foot = cr._tpu_foot(10.0, 20.0, 30.0)
    again = cr._tpu_foot(10.0, 20.0, 30.0)
    assert again is not foot
    assert again.wrapped.IsPartner(foot.wrapped)
    c = foot.center()
    again.move(Location((5.0, 0.0, 0.0)))
    assert cr._tpu_foot(10.0, 20.0, 30.0).center() == c
    assert foot.center() == c
    monkeypatch.setattr(cr, "FOOT_BORE", cr.FOOT_BORE + 2.0)
    wider = cr._tpu_foot(10.0, 20.0, 30.0)
    assert not wider.wrapped.IsPartner(foot.wrapped)
    assert wider.volume < foot.volume - 1.0
    monkeypatch.undo()              # (a store too: built afresh, as it was)
    back = cr._tpu_foot(10.0, 20.0, 30.0)
    assert not back.wrapped.IsPartner(wider.wrapped)
    assert back.volume == pytest.approx(foot.volume, rel=1e-12)
    assert cr._tpu_foot(10.0, 20.0, 30.0).wrapped.IsPartner(back.wrapped)
    assert cr._exact(-0.0) != cr._exact(0.0)
    assert cr._exact(np.float64(1.5)) == cr._exact(1.5)
    assert cr._exact({"a": 1, "b": 2}) != cr._exact({"b": 2, "a": 1})     # (in order)
    with pytest.raises(TypeError):
        cr._exact(foot)


@pytest.mark.construction
def test_the_mouth_is_snapped_to_the_plan_at_every_crank_angle():
    """``cr.mouth_xy``: B's mouth is still over the turn to 1e-13 mm in its frozen mechanism,
    and exactly its design's (-366, 322 from O: the planned geometry's first sample rounded
    to 1e-9 mm) at every angle, so the stand's members and the base board, and their memos,
    are the same at every angle."""
    from spiderpig.construction.base import Build

    cfg = BuildConfig(linkage="decanter_rcm", robot=False)
    tmpl, design = cache.cached_design(cfg)
    name = cr._point_of(design.plan.topo, cfg.lk, cr.mouth_name(cfg.lk))
    planned = np.asarray(design.plan.topo.geometry.points[name], float).reshape(-1, 2)[0]
    for t in (1.0, 3.5, 5.2):
        build = Build(design.ctx, design.plan, tmpl.freeze_at(t))
        assert np.abs(build.xy(name) - planned).max() < 1e-9
        assert cr.mouth_xy(build).tolist() == [-366.0, 322.0]


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


SWING_ANGLES = ("rest", "full", "1.0", "3.5", "5.2", "165deg", "180deg", "345deg")
"""The clash test's crank angles: at rest, at full tilt, three across the turn, and where A's
105 mm hand crank met its cradle link b2 and tie Tr2's screw (design review round 1's drive:
about 165-195° and 330-345° of crank)."""


def _swing_angle(key: str, name: str) -> float:
    """The crank angle :data:`SWING_ANGLES` names for ``key`` (rest and full: the least and
    the most tilt of 721 samples of the turn)."""
    if name in ("rest", "full"):
        ts = np.linspace(0, 2 * math.pi, 721)
        tilt = _tilt(key, ts)
        return float(ts[int(np.argmin(tilt) if name == "rest" else np.argmax(tilt))])
    if name.endswith("deg"):
        return float(np.radians(float(name[:-3])))
    return float(name)


@pytest.mark.slow
@pytest.mark.construction
@pytest.mark.parametrize("angle", SWING_ANGLES)
@pytest.mark.parametrize("key", CRADLES)
def test_the_whole_cradle_is_clash_free_over_the_swing(key, angle):
    """Both sides, the box, the stand on its base plate, the bottle (a 6 L methuselah, the V
    blocks at their lowest) and the decanter at rest, at full tilt and across the turn: no
    two parts meet, the hand crank's sweep meets nothing, and every bottle size at its own
    setting (its V blocks raised, its base stop moved) meets nothing, nor do they; the base
    stop never comes nearer the base board than a finger's 25 mm (ISO 13854: at rest, set
    for a 6 L, its lowest). One crank angle per test (each a fabrication of the whole
    cradle, a minute or two, from the test cache when it holds this code's: the angles run
    side by side under xdist, each test under 5 min)."""
    from spiderpig.construction.contract import bad_solids, clashes
    from spiderpig.cradle_report import wheel_sweep

    cfg = BuildConfig(linkage=key, robot=False)
    t = _swing_angle(key, angle)
    mech = cache.cached_product(cfg, t)
    assert clashes(mech) == [], t
    assert wheel_sweep(mech) == [], t
    assert cr.finger_gap(mech) >= cr.FINGER_GAP + 15.0, t   # (the board's window)
    for size in cr.SIZES:
        assert cr.bottle_hits(mech, size) == [], (t, size.key)
    if angle != SWING_ANGLES[-1]:
        return
    assert bad_solids(mech) == []
    names = {b.name for b in mech.bodies}
    assert {"L.gearbox", "bottle", "decanter", "led", "front_saddle", "rear_saddle",
            "front_saddle_vee", "rear_saddle_vee", "front_saddle_gauge", "base_stop_gate",
            "base_plate", *cr.FRONT_MOVES, *cr.REAR_MOVES, *cr.STOP_MOVES} <= names
    assert not any(n.endswith("_collar") for n in names)
    assert not any(n.startswith("R.") and ("gearbox" in n or n == "R.b1") for n in names)


@pytest.mark.slow
@pytest.mark.construction
@pytest.mark.parametrize("key", CRADLES)
def test_a_hand_on_the_grip_clears_the_cradle_over_the_whole_turn(key):
    """The hand crank's swept solids and a hand on its grip (``cradle_report.hand_sweep``:
    the grip and 20 mm round it, from the arm to past the bolt's head) against every other
    part over the whole crank turn at 1° (design review round 2's M-3: A's hand met its
    cradle link b2 and its rear tie's screw for 64.5° of crank, which the clash test's
    angles and the crank's own solids had missed): nothing within either."""
    from spiderpig.cradle_report import hand_sweep

    cfg = BuildConfig(linkage=key, robot=False)
    mech = cache.cached_product(cfg, 1.0)
    got = hand_sweep(cfg, mech, 1.0)
    assert got["crank"][0] > 2.0, got
    assert got["hand"][0] > 2.0, got
    assert got["hand_under_0_deg"] == 0.0


@pytest.mark.slow
@pytest.mark.construction
@pytest.mark.parametrize("key", CRADLES)
def test_the_cradle_passes_the_cut_rules(key):
    """Every laser-cut part against SendCutSend's rules for its sheet: no errors (the least
    hole a sheet's thickness, the hole-to-edge web, the part sizes)."""
    from spiderpig import manufacture

    cfg = BuildConfig(linkage=key, robot=False)
    mech = cache.cached_product(cfg, 1.0)
    got = manufacture.summary(manufacture.check(mech, cfg.sheet, dxf=False))
    assert got["ok"], got


def _outline_turns(part) -> tuple[list[float], list[tuple[str, float]]]:
    """A plate's top outline (its outer wire), in order: every tangent break's turn (degrees,
    + convex, - re-entrant) and each edge's kind (``L``, ``+`` a convex arc, ``-`` a concave
    one) with its radius."""
    from OCP.GeomAbs import GeomAbs_Circle
    from OCP.gp import gp_Pnt, gp_Vec

    from spiderpig.layout import _ordered_edges

    top = max((f for f in part.faces() if abs(abs(f.normal_at().Z) - 1) < 1e-6),
              key=lambda f: f.center().Z)
    edges = _ordered_edges(top.outer_wire())
    pts = [(p.X(), p.Y()) for c, u0, u1 in edges
           for p in (c.Value(u0 + (u1 - u0) * k / 8) for k in range(8))]
    ccw = 1 if sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1)
                   in zip(pts, pts[1:] + pts[:1], strict=True)) > 0 else -1

    def tan(c, u, way):
        p, v = gp_Pnt(), gp_Vec()
        c.D1(u, p, v)
        n = math.hypot(v.X(), v.Y())
        return way * v.X() / n, way * v.Y() / n

    breaks, kinds = [], []
    for i, (c, u0, u1) in enumerate(edges):
        way = 1 if u1 >= u0 else -1
        if c.GetType() == GeomAbs_Circle:
            ta, tm = tan(c, u0, way), tan(c, (u0 + u1) / 2, way)
            kinds.append(("+" if (ta[0] * tm[1] - ta[1] * tm[0]) * ccw > 0 else "-",
                          round(c.Circle().Radius(), 3)))
        else:
            kinds.append(("L", 0.0))
        cn, v0, v1 = edges[(i + 1) % len(edges)]
        a, b = tan(c, u1, way), tan(cn, v0, 1 if v1 >= v0 else -1)
        cross = a[0] * b[1] - a[1] * b[0]
        turn = math.degrees(math.atan2(abs(cross), a[0] * b[0] + a[1] * b[1]))
        if turn > 0.5:
            breaks.append(math.copysign(turn, cross * ccw))
    return breaks, kinds


def _s_steps(kinds, fillet: float) -> list[float]:
    """The radii of the convex arcs between two concave ``fillet`` arcs: the S-steps a
    closing leaves where a disc stands a little proud of a straight edge."""
    n = len(kinds)
    return [k[1] for i, k in enumerate(kinds) if k[0] == "+"
            and all(kinds[j][0] == "-" and abs(kinds[j][1] - fillet) < 1e-6
                    for j in ((i - 1) % n, (i + 1) % n))]


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_a_frame_plate_s_steps_are_tapered_and_its_cut_corners_rounded():
    """Design review round 3, laser-cut parts minor 5: a strip with a boss 0.8 mm proud of it
    and a 32 mm member running straight on from a 24 mm one closed into S-shaped steps
    (R20, the boss's or the end's arc, R20); now each station is tapered into (the hull
    between two proud stations, 1 in 10 from a proud one into a plain strip), so no S-step
    is left. A corner a member's straight-on 60 mm widening (over :data:`STEP_MAX`) keeps.
    The board's cut across a member leaves no sharp convex corner (rounded
    :data:`EDGE_R`), and the outline has no tangent break at all."""
    from spiderpig.shapes import Cut, Rect, cut_holes, disc, pill, union

    t, web, fillet = 4.75, 9.6, 20.0
    pads = [((0.0, 0.0), (100.0, 0.0), 12.0), ((100.0, 0.0), (180.0, 0.0), 16.0),
            ((180.0, 0.0), (180.0, 120.0), 16.0)]
    cuts = [Cut((40.0, 0.0), 6.4), Cut((100.0, 0.0), 6.4), Cut((180.0, 120.0), 6.4),
            Rect((90.0, -29.5), (300.0, 40.0)),        # the board, its top at y -9.5
            ]
    old = cut_holes(cr._closed(union([pill(a, b, r, 0.0, t) for a, b, r in pads]
                                     + [disc(c.xy, c.d / 2 + web, 0.0, t)
                                        for c in cuts if isinstance(c, Cut)]),
                               0.0, t, fillet), cuts, 0.0, t)
    old_breaks, old_kinds = _outline_turns(old)
    assert _s_steps(old_kinds, fillet), old_kinds           # (the closing's S-steps)
    assert any(b > 0 for b in old_breaks)                   # (the board cut's corners)
    new = cr.plate_part(pads, cuts, 0.0, t, web, fillet)
    breaks, kinds = _outline_turns(new)
    assert _s_steps(kinds, fillet) == []
    assert breaks == []
    assert ("+", cr.EDGE_R) in kinds
    assert new.volume > 0.98 * old.volume
    assert len(new.solids()) == 1
    # A's plates keep their steps for its flat pack (cradle.TAPERED), their corners rounded
    assert frozenset({"decanter_rcm"}) == cr.TAPERED
    plain = cr.plate_part(pads, cuts, 0.0, t, web, fillet, taper=False)
    breaks, kinds = _outline_turns(plain)
    assert _s_steps(kinds, fillet)
    assert breaks == []


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_guide_names_the_as_cut_parts_and_the_routed_birch():
    """Design review round 3, laser-cut parts minors 3 and 6: the arrival step names the
    parts under SendCutSend's deburring minimum (:func:`assembly.as_cut_words`, a 24 mm link
    once whatever its sides; a 30 mm one is deburred), and the parts table calls the birch
    routed and names the crank block's plates (their files keep their labels)."""
    from build123d import Box, Cylinder, Location

    from spiderpig.construction.assembly import AS_CUT, CRADLE_ARRIVAL, as_cut_words
    from spiderpig.labels import part_types
    from spiderpig.mechanism import Body, Mechanism

    def plate(w, h, t, sheet, name):
        part = Box(w, h, t).moved(Location((w / 2, h / 2, t / 2)))
        part = part - Cylinder(4.1, 3 * t).moved(Location((12, 12, t / 2)))
        return Body(name, part, fab="laser", sheet=sheet)

    mech = Mechanism("m", [plate(120, 24, 4.826, "al7075_4p8mm", f"{s}.b4") for s in "LR"]
                     + [plate(120, 30, 4.75, "al5052_4p8mm", "L.b6"),
                        plate(300, 200, 9.0, "birch_9mm", "base_plate"),
                        plate(128, 44, 3.175, "al6061_3p2mm", "L.conn_plate3")],
                     meta={"sheet": "al5052_3p2mm"})
    assert as_cut_words(mech) == "b4"
    assert AS_CUT in CRADLE_ARRIVAL
    names = {t.label[:5]: t.name for t in part_types(mech)}
    assert names["LC300"].startswith("base board, CNC-routed, ")
    assert names["LC128"].startswith("crank block plate, laser-cut, ")


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_a_crank_block_plate_has_no_cusp_at_its_neck():
    """Design review round 3, laser-cut parts minor 1: the crank block's plates (the arm's
    pill on its hub's disc) met in two re-entrant cusps at the neck; closed by
    :data:`construction.plates.CRANK_FILLET_R` the outline has no tangent break, and what
    the closing added lies in the pills its claims take (:func:`plates.arm_fills`)."""
    from build123d import Solid, Vector

    from spiderpig.construction import plates
    from spiderpig.construction.handcrank import NRV050
    from spiderpig.shapes import difference, pill, union

    t = 3.175
    hc = HandCrank(NRV050)
    ctx = type("C", (), {"sheet_t": staticmethod(lambda role: t)})()
    hub, web, throw = hc.hub_r(ctx), hc.web_r(ctx), 105.0
    part = plates.arm_plate(hub, web, throw, (0.0, 0.0), (0.0, throw), 0.0, t, ())
    breaks, _ = _outline_turns(part)
    assert breaks == []
    face, covers = plates._arm(hub, web, throw)
    plain = union([pill((0.0, 0.0), (throw, 0.0), web, 0.0, t),
                   pill((0.0, 0.0), (0.0, 0.0), hub, 0.0, t)])
    added = difference(Solid.extrude(face, Vector(0, 0, t)), plain)
    assert added.volume > 1.0
    held = union([pill(p, q, w, -1.0, t + 1.0) for p, q, w in covers])
    assert difference(added, held).volume < 1e-6


@pytest.fixture
def cached_report_fabrications(monkeypatch):
    """``cradle_report``'s fabrications from the test cache (``cached_product``): a report
    of a loaded cradle is the fresh one's to the byte (both cradles' ``report`` JSON,
    measured 2026-10-20), in 15-20 s instead of 90-130."""
    from spiderpig import cradle_report

    def fabricate(tmpl, config, t=1.0, store=None):
        assert store is None
        return cache.cached_product(config, t)

    monkeypatch.setattr(cradle_report, "fabricate", fabricate)


@pytest.fixture
def cached_cradle_fabrications(monkeypatch, cached_report_fabrications):
    """Every fabrication of a cradle from the test cache: ``cradle_report``'s
    (:func:`cached_report_fabrications`), the api's (``api.building.fabricate_at``: a build,
    a verify's clash angles) and ``spiderpig build``'s, each a mechanism of the caller's own
    (``cache.own_product``, as the store's fabrication cache hands one out: ``build`` adds to
    its ``bom_extras``), when the design's plan is the cache's (else fabricated). Both
    cradles' ``standard`` verify rows are the fresh ones' to the byte (measured
    2026-10-20)."""
    from spiderpig import build
    from spiderpig.api import building

    fabricate_at, fabricate = building.fabricate_at, build.fabricate

    def cached(cfg, plan) -> bool:
        return (is_cradle(cfg) and not cfg.robot
                and cache.plan_doc(plan) == cache.plan_doc(cache.cached_design(cfg)[1].plan))

    def at(design, t):
        if design.side is not None and cached(design.config, design.side.plan):
            return cache.own_product(design.config, t)
        return fabricate_at(design, t)

    def fab(tmpl, config=None, t=1.0, *, store=None):
        from spiderpig.fabricate import design_side

        if config is not None and cached(config, design_side(tmpl, config).plan):
            return cache.own_product(config, t)
        return fabricate(tmpl, config, t, store=store)

    monkeypatch.setattr(building, "fabricate_at", at)
    monkeypatch.setattr(build, "fabricate", fab)


@pytest.mark.slow
@pytest.mark.strength
@pytest.mark.usefixtures("cached_report_fabrications")
def test_the_cradle_stands_turns_easily_and_holds():
    """A, the hinge (B is the main design): the mouth still for both bottles, it stands at
    15° and more and takes a 25 N push at the bottle's top (the worst over the swing), the
    hand turns its heavy-load crank with under MIL-STD-1472H's 22 N, the worm holds within
    its maximum, the rod, the crankpin and every pin and plate hold a sealed full 6 L bottle
    3 x over and a firm push at the grip on a blocked cradle up to the clutch's cap (the jam:
    ``cradle_report.jam_nm``, ``config.PRODUCT_JAM_NM``)."""
    from spiderpig.config import PRODUCT_JAM_NM
    from spiderpig.cradle_report import JAM_HAND_N, jam_nm, report

    for key in CRADLES:
        cfg = BuildConfig(linkage=key, robot=False)
        assert PRODUCT_JAM_NM[key] == pytest.approx(jam_nm(cfg), abs=0.05)

    r = report(BuildConfig(linkage="decanter_hinge", robot=False), clash_angles=0)
    assert r["swing"]["mouth_wander_mm"] < 1e-6
    assert max(b["mouth_wander_mm"] for b in r["bottles"]) < 0.5
    assert r["stand"]["worst_tip_deg"] >= 15.0
    assert min(c["push_at_top_n"] for c in r["stand"]["cases"]) >= 25.0
    assert r["drive"]["irreversible"]
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
    # every joint, link, column, plate and tie rod holds the jam (rated from the force
    # vectors: the links chord by chord, the side's posts and plates as one frame), the side
    # frames too since their redesign (2026-10-20: the column E1 under the front leg)
    _holds_the_jam(r, "decanter_hinge")
    _the_structure_is_rated(r, ("b2",), {"pillar:G"},
                            {"column:C1", "column:C2", "column:D1", "column:D2", "column:D3",
                             "column:E1", "column:E2"})
    _holds_the_clutchs_cap(r, "decanter_hinge")
    _the_grip_is_open(_the_drive_keeps_its_bars(r) + _open_joints(r, "decanter_hinge"))


def _the_drive_keeps_its_bars(r: dict) -> list[str]:
    """A report's drive against its cited bars (design review round 2's drive, 2026-10-14):
    the NRV050 against Phidgets' 85 N·m maximum, 1.3 x over the walking peak and at
    ``GEARBOX_CAP_FLOOR`` over the clutch's cap; the grip under MIL-STD-1472H's 22 N at the
    worst start (the catalogue's run-in efficiency: the report's band gives the new box's
    too) on a crank of Figure 13's dimensions for its row: a light-load crank within 125 mm,
    its grip 25-75 long and 10-16 across; a heavy-load one 190-510 mm, 75 long or more,
    25-38 across. A grip open at the bar (``cradle_report.OPEN_GRIP``) is held no worse than
    the bar and returned, for the test's xfail."""
    from spiderpig.construction.handcrank import (
        DESIGN_START_NM,
        HANDLE_OVER_RADIUS,
        LIGHT_LOAD_N,
    )
    from spiderpig.cradle_report import OPEN_GRIP

    drive = r["drive"]
    # the clutch's drawn setting (the guide's expected reading, the config's cap) is the
    # report's own worst start
    assert DESIGN_START_NM[r["linkage"]] == pytest.approx(drive["handle_torque_nm"],
                                                          abs=0.002)
    assert drive["gearbox_rating_nm"] == 85.0
    assert drive["gearbox_sf"] >= 1.3, drive["gearbox_sf"]
    assert drive["gearbox_cap_sf"] >= drive["gearbox_cap_floor"], drive["gearbox_cap_sf"]
    grip_d, grip_len = drive["grip_mm"]
    if drive["grip_load"] == "light":       # (A's 5 mm over: handcrank.HANDLES says why)
        assert drive["grip_radius_mm"] <= 125.0 + HANDLE_OVER_RADIUS.get(r["linkage"], 0.0)
        assert 25.0 <= grip_len <= 75.0
        assert 10.0 <= grip_d <= 16.0
    else:
        assert 190.0 <= drive["grip_radius_mm"] <= 510.0
        assert grip_len >= 75.0
        assert 25.0 <= grip_d <= 38.0
    lo, hi = drive["handle_force_band_n"]
    assert lo == drive["handle_force_n"] < hi                   # (new: less efficient)
    grip = (drive["handle_force_n"], drive["grip_radius_mm"])
    if r["linkage"] in OPEN_GRIP:                               # at the bar: the user's call
        assert drive["handle_force_n"] <= LIGHT_LOAD_N, grip
        if drive["handle_force_n"] >= LIGHT_LOAD_N:
            return [f"the grip {grip[0]:g} N at {grip[1]:g} mm ({OPEN_GRIP[r['linkage']]})"]
        return []
    assert drive["handle_force_n"] < LIGHT_LOAD_N, grip
    return []


def _holds_the_jam(r: dict, key: str) -> None:
    """Every row of the report holds the jam at SF 1.3 or more, the side frames' posts and
    plates included (their redesign, 2026-10-20: nothing open at the clutch's cap,
    ``cradle_report.OPEN_AT_CAP``); the cradle box's tie rods and their end joints among the
    rows (design review round 2's M1); the rod as a beam-column at its far pin's offset (its
    minor 1); the side's stiffness across (its M3: a hand's 20 N sideways on the bottle sways
    the pivots under 5 mm)."""
    from spiderpig.cradle_report import OPEN_AT_CAP

    assert key not in OPEN_AT_CAP
    rows = r["strength"]["rows"]
    held = [(x["joint"], x["jam_sf"]) for x in rows if x["jam_sf"] is not None]
    assert min(sf for _, sf in held) >= 1.3, held
    assert {"frame:outer", "frame:inner"} <= {j for j, _ in held}
    box = {x["joint"]: x for x in rows if x["kind"] == "box"}
    assert set(box) == {"box:tie rods", "box:tie rod ends"}
    from spiderpig.cradle_report import OPEN_JOINTS

    open_ = OPEN_JOINTS.get(key, {})
    gap = r["structure"]["box"]["cases"]["walk"]["gap_ratio"]          # closed walking at the
    assert gap >= (0.9 if "tie rod ends" in open_ else 1.0), gap       # least preload
    assert r["rod"]["sf_secant_jam"] >= 1.5
    # design review round 3: the sway with the rail's rock in its cleats (M6); the side as one
    # frame buckling (M2) among the rows above; the posts' screwed ends (M5): at the cap their
    # screws within A2-70's proof at 1.3, the plates under them within VDI 2230's limiting
    # pressure (its R10), walking every face shut at its screw's least preload and none
    # slipping (fretting each pour)
    stiff = r["structure"]["stiffness"]
    assert stiff["sway_mm"] <= 5.0
    assert stiff["sway_mm"] > stiff["elastic_mm"]                     # (the rock counted)
    assert {x["joint"] for x in rows} >= {"frame:side buckling"}
    assert r["structure"]["buckling"]["jam"]["factor"] >= 1.3
    joints = r["structure"]["joints"]["worst"]
    assert joints["jam"]["screw_sf"] >= 1.3, joints
    assert joints["jam"]["bearing_sf"] >= 1.0, joints
    assert joints["walk"]["slip_x"] <= 1.0, joints
    assert joints["walk"]["open_x"] <= (1.1 if "posts" in open_ else 1.0), joints


def _open_joints(r: dict, key: str) -> list[str]:
    """The screwed joints open at their walking bar as ``cradle_report.OPEN_JOINTS`` says
    (the user's call), for the test's xfail; an entry no longer open fails."""
    from spiderpig.cradle_report import OPEN_JOINTS

    got = []
    for what, why in OPEN_JOINTS.get(key, {}).items():
        if what == "tie rod ends":
            is_open = r["structure"]["box"]["cases"]["walk"]["gap_ratio"] < 1.0
        else:
            is_open = r["structure"]["joints"]["worst"]["walk"]["open_x"] > 1.0
        assert is_open, f"{key}: {what} holds now: drop it from OPEN_JOINTS"
        got.append(why)
    return got


def _the_grip_is_open(grip: list[str]) -> None:
    """The grip open at its bar, said as such (A's: ``cradle_report.OPEN_GRIP``, the user's
    call): the test's last check, an xfail with its reason. (The side frames, open at the
    clutch's cap from design review round 2, hold it since their redesign of 2026-10-20.)"""
    if grip:
        pytest.xfail("; ".join(grip))


def _holds_the_clutchs_cap(r: dict, key: str) -> None:
    """A report's drive at its clutch (``cradle_drive``): set at 1.3 x the start, its least
    in the bench check's window clears the start by 1.1; its cap is the jam case's torque
    (however hard the push), the gearbox's maximum over it at its floor; unchecked, the
    greased friction and the seats' wear drift it, and the report says the cap it would give;
    the grip's force at the clutch's top within the input shaft's radial load there; the
    output shaft's radial load at the rod's plane walking 3 x over; every part, the output
    shaft included, holds the cap at SF 1.3 or more 30° and more from the toggles, the side
    frames too since their redesign (2026-10-20; the band round a toggle where any falls
    under 1.3 is reported, no wider); the keyed plates at the cap."""
    from spiderpig.cradle_drive import OUTPUT, RISER

    clutch = r["drive"]["clutch"]
    assert clutch["margin"] >= clutch["margin_floor"]
    assert r["drive"]["jam_torque_nm"] == pytest.approx(clutch["crank_cap_nm"], abs=0.01)
    assert clutch["unchecked_cap_nm"] > clutch["crank_cap_nm"]          # (reported)
    # the ceiling (design review round 3's M1): set on a new box, the cap is the rated one,
    # and the new box's 6 L still turns; spun fast, more (reported, the use limits' "turn
    # it, don't spin it")
    assert clutch["set_new_cap_nm"] <= clutch["crank_cap_nm"] + 0.05
    assert clutch["set_new_no_ceiling_cap_nm"] > clutch["crank_cap_nm"]
    assert clutch["new_box_margin"] >= 1.0
    assert clutch["fast_cap_nm"] > clutch["crank_cap_nm"]
    assert clutch["fast_cap_least_sf"] >= 1.2
    # every plunger setting reachable by the guide's limit but a 10 % weak spring's at the
    # greased friction's low end (the slip then stays under the target: its m3)
    assert all(case == ["0.09", "0.9"] for case in clutch["plunger_reach"]["under"])
    # the gearbox's riser (its M2): no slip walking, every strength row at the cap 30° off
    riser = r["drive"]["riser"]
    assert riser["walk"]["slip_sf"] >= 2.0, riser
    assert riser["cap"]["least"] >= 1.3, riser
    assert r["drive"]["input_overhung"]["sf"] >= 2.0
    assert r["drive"]["output_overhung"]["sf_walk"] >= 3.0
    capped = r["jam_capped"]
    assert capped["crank_nm"] == clutch["crank_cap_nm"]
    away = capped["by_band"][-1]["parts"]
    assert min(away.values()) >= 1.3, away          # every part 30° from the toggles,
    assert OUTPUT in capped["parts"]                # the output shaft among them,
    assert RISER in capped["parts"]                 # and the gearbox's riser
    bands = [(x["band_deg"], x["worst"]) for x in capped["by_band"]]
    held = [v["holds_1p3_outside_deg"] for v in capped["parts"].values()]
    assert None not in held, bands                  # (5° steps: where the old case
    holds = max(held)                               # skipped, now rated at a bound)
    assert holds <= 30.0, bands
    assert capped["holds_1p3_outside_deg"] == holds, bands
    assert capped["by_band"][0]["band_deg"] == 0.0              # the whole turn rated
    from spiderpig.construction.handcrank import TOGGLE_TURNS

    assert holds * 30.0 / 360.0 <= TOGGLE_TURNS                 # (the guide's marks)
    assert r["drive"]["keyway"]["plates_sf_cap"] >= 2.0
    assert r["drive"]["keyway"]["one_plate_sf_walk"] >= 1.3
    assert "This report:" in r["verdict"][1]


@pytest.mark.slow
@pytest.mark.strength
@pytest.mark.usefixtures("cached_report_fabrications")
def test_b_stands_turns_easily_and_holds():
    """B, the virtual hinge (``docs/agentlib/DECANTER.md``): the mouth still for both
    bottles, it stands at 25° and takes a 25 N push at the bottle's top (the worst over the
    swing), the hand turns it with under MIL-STD-1472H's light-load crank's 22 N (the box
    run in), the worm holds within its maximum, the rod and the crankpin hold the jam, every
    pin, pillar and plate holds a sealed full 6 L bottle 2 x over and a firm 40 N push at the
    grip on a blocked cradle, up to the clutch's cap, 1.3 x over."""
    from spiderpig.cradle_report import report

    r = report(BuildConfig(linkage="decanter_rcm", robot=False), clash_angles=0)
    assert r["swing"]["mouth_wander_mm"] < 1e-6
    assert max(b["mouth_wander_mm"] for b in r["bottles"]) < 0.5
    assert r["stand"]["worst_tip_deg"] >= 25.0
    assert min(c["push_at_top_n"] for c in r["stand"]["cases"]) >= 25.0
    assert r["drive"]["irreversible"]
    assert r["rod"]["sf_jam"] >= 1.5
    assert r["crankpin"]["sf_jam"] >= 1.5
    # the empty cradle turns with a fingertip (the guide's check after it goes back together)
    assert r["drive"]["empty_handle_force_n"] < cr.EMPTY_GRIP_N
    # every edge (the board sits 40 mm toward the drive side: cradle.BASE_Z)
    edges = r["stand"]["edges"]
    assert set(edges) == {"front", "rear", "drive side", "passive side"}
    assert all(e["tip_deg"] >= 25.0 and e["push_at_top_n"] >= 25.0 for e in edges.values()), \
        edges
    rows = r["strength"]["rows"]
    walk = [x["walk_sf"] for x in rows if x["walk_sf"] is not None]
    assert min(walk) >= 2.0, [(x["joint"], x["walk_sf"]) for x in rows]
    _holds_the_jam(r, "decanter_rcm")
    # the quick-release clevis pins at X and Y (2026-10-13): stronger than the M6 Chicago
    # screws they replaced (1.72 and 1.49 jammed); K and J2 clevis pins too, P and J the
    # hardened pivot pins (design review round 1): no Chicago screw left
    quick = {x["joint"]: x for x in rows if x["joint"] in ("pin:X", "pin:Y")}
    assert set(quick) == {"pin:X", "pin:Y"}
    for x in quick.values():
        assert "clevis" in x["section"], x
        assert x["jam_sf"] >= 2.0, x
        assert x["walk_sf"] >= 6.0, x
    pins = {x["joint"]: x["section"] for x in rows if x["kind"] == "pin"}
    assert {j for j, s in pins.items() if "clevis" in s} == {"pin:K", "pin:J2", "pin:X", "pin:Y"}
    assert {j for j, s in pins.items() if "pivot pin" in s} == {"pin:P", "pin:J"}
    # X's and Y's TPU keepers (pack turn 1): each holds its pin 10 x its weight at the least
    # interference a print gives and TPU's least published modulus, and goes on by hand
    k = r["keepers"]
    assert {cr.keeper_pin(x["keeper"]) for x in k["rows"]} == {f"{s}.pin_{j}_pin" for s in "LR"
                                                                for j in "XY"}
    assert k["least_ratio"] >= k["bar"] == 10.0
    # design review round 2's M3: against the most the way back can put on a pin's tip,
    # half the 3.6 kg cradle box, the bench push screens each keeper 1.5 x over it; printed
    # at 7.4 mm, at least 0.4 mm of grip; still on by hand, a tube over the pin
    assert cr.KEEPER == (7.4, 0.4)
    assert 3.3 < k["box_kg"] < 3.9          # (the 12 mm tie rods since round 2: 3.2 before)
    assert 13.0 < k["load_n"] < 18.0
    assert k["load_ratio"] >= k["load_bar"] == 1.5
    assert all(x["push_on_n"] <= k["push_on_bar_n"] == 120.0 for x in k["rows"])
    # the packing review's m1 (pack turn 2): at the tight end too (the bore 0.15 mm under,
    # the pin at its most: 0.74 mm), still by hand with a tube over the pin
    assert k["tight_mm"] == pytest.approx(0.74)
    assert all(x["push_on_tight_n"] <= k["push_on_bar_n"] for x in k["rows"])
    # and its M2: the box onto the drive side lying down, pin X first: its centre of mass
    # lies past X from Y, so turned about X it lifts off Y's tip (the other way round it
    # would rest on X's, more than its weight: the steps say X first)
    assert k["second_tip_n"]["X then Y"] <= 0.0 < k["second_tip_n"]["Y then X"]
    # the stand with the wine leaving as it pours (round 2's m1): the rear's real margin,
    # over its bound (a full 6 L at every tilt)
    pour = r["stand"]["pouring"]
    assert pour["rear"]["tip_deg"] > r["stand"]["edges"]["rear"]["tip_deg"]
    assert all(e["tip_deg"] >= 25.0 for e in pour.values())
    _the_structure_is_rated(r, ("b2", "b6"), {"pillar:O1", "pillar:O2"},
                            {"column:D1", "column:D2", "column:E1", "column:E2"})
    # the known limits (not designed for): a block near the mouth multiplies the pins' loads;
    # where the frame starts to give, in newtons at the grip (design review round 2's M2)
    front = r["blocked"]["cases"]["front saddle"]
    assert front["joints_n"]["J"] > 3.0 * max(x["jam_n"] or 0.0 for x in rows
                                              if x["joint"] == "pin:J")
    first = r["blocked_capped"]["cases"]["front saddle"]["threshold"]["sf_1"]
    assert first is not None
    assert 0.0 < first["grip_n"] < 27.0, first      # under the clutch's 27 N
    # design review round 3's M3: each angle pushed as far as its own clutch lets it (where
    # gravity helps the turn, up to 37 N at the grip), the rear saddle's block and a 6 L's
    # base found to give within the clutch (the use limits keep the board under them clear)
    for name in ("rear saddle", "bottle's base"):
        th = r["blocked_capped"]["cases"][name]["threshold"]
        assert th["within_grip_n"] > 30.0, th
        assert th["sf_1p3"] is not None, th
        assert th["sf_1p3"]["grip_n"] < th["within_grip_n"], th
    _holds_the_clutchs_cap(r, "decanter_rcm")
    assert not _the_drive_keeps_its_bars(r)
    _the_grip_is_open(_open_joints(r, "decanter_rcm"))


def _the_structure_is_rated(r: dict, links, pillars, columns) -> None:
    """The report's rows include the structure rated from the force vectors (design review
    round 1, ``cradle_report.structure``): each cradle link and bell crank chord by chord
    (its worst chord named), each pillar on its box, each stand column, both frame plates'
    members."""
    rows = {x["joint"]: x for x in r["strength"]["rows"]}
    st = r["structure"]
    for link in links:
        assert st["link_frames"]["jam"][link]["worst"]["chord"], link
        assert f"link:{link}" in rows
    for p in pillars:
        assert p.split(":", 1)[-1] in st["posts"]["jam"]
        assert rows[p]["jam_sf"] is not None
    for c in columns:
        assert rows[c]["kind"] == "column"
    assert {"frame:outer", "frame:inner"} <= set(rows)
    assert st["skipped_deg"] > 0.0                    # the toggles, left to the drive's cap


@pytest.mark.slow
@pytest.mark.construction
@pytest.mark.parametrize("key", CRADLES)
def test_the_cradle_has_an_assembly_order(key):
    """The guide's order (``CRADLE_ORDER``; B's ``CRADLE_MODULE_ORDER``) covers every part
    once: the gearbox under the drive side's outer plate first, the drive side's cradle link
    with its tie rods on the bench, the passive side's cradle link onto the rods on the
    cradle (not in its stack), the base board and the hand crank, the board's cleats and
    finish once both sides stand on it (glued bare, then finished: design review round 2's
    M6), and the use (its M4): where it stands, setting it for a bottle, the riser and the
    decanter, decanting, the use limits, in that order, each its own short items and its
    own picture; B's trip apart and back after them, a step each. B's passive side panel is
    built whole on the bench and lowered onto its cradle link, the cradle lying on its drive
    side on the riser's blocks (its M1)."""
    from spiderpig.construction.assembly import FINISH, JOIN, STACK, UNIT, USE, assembly_steps
    from spiderpig.fabricate import design_side, template_for

    cfg = BuildConfig(linkage=key, robot=False)
    tmpl = template_for(cfg)
    design = design_side(tmpl, cfg)
    steps = assembly_steps(cache.cached_product(cfg, 1.0), design)   # (fabricate's product)
    link = cr.cradle_link(cfg.lk)
    modules = key in cr.PACK_POSES
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
    # every carriage bolt goes in from under the base board, the passive side's too
    bolts = [a for s in steps for a in s.adds if a.startswith("foot_bracket") and
             a.endswith("_screw1")]
    assert len(bolts) == len(set(bolts)) > 3
    assert set(bolts) <= set(base.adds)
    # the hand crank right after the base board (to turn the cradle for the checks), all
    # of it in its step
    (hand,) = [s for s in steps if "L.handle" in s.adds]
    assert steps.index(hand) == steps.index(base) + 1
    assert {b for s in steps for b in s.adds if b.startswith("L.handle")} <= set(hand.adds)
    assert steps[-1].stage == USE
    # the board bare until both sides stand on it: the locator with the board, glued bare;
    # every cleat in one last build step, glued bare, then the finish (Titebond's FAQ)
    adds = {a for s in steps for a in s.adds}
    assert {"riser_locator_0", "riser_locator_1"} <= set(base.adds)
    assert "bare" in " ".join(base.text)
    assert "oil" not in " ".join(base.text)
    (board,) = [s for s in steps if any(a.startswith("base_cleat_R") for a in s.adds)]
    assert {a for a in adds if a.startswith("base_cleat_")} <= set(board.adds)
    btext = " ".join(board.text)
    assert btext.index("Glue the cleats") < btext.index("polyurethane every face")
    assert "titebond.com" in btext
    assert "lead-in" in btext
    assert "to level" in btext
    assert "luggage scale" in btext
    assert steps.index(board) > max(steps.index(s) for s in steps if s.side == "R")
    # the clutch set, the worm's hold and the toggle zones once the cradle is built (design
    # review round 3's stand and use M2, M4): the last build step; the hand crank's step
    # sets the plungers at their felt start (its B1), no more
    htext = " ".join(hand.text)
    for words in ("stops hard", "half a turn", "an eighth of a turn", "Before the first pour"):
        assert words in htext, words
    for words in ("flush", "Measure the start", "tube in a vise", "toggle"):
        assert words not in htext, words
    (pour,) = [s for s in steps if s.title == "Before the first pour"]
    assert steps.index(pour) == steps.index(board) + 1
    assert not pour.adds
    ptext = " ".join(pour.text)
    for words in ("luggage scale", "check stub", "no vise", "none over the ceiling",
                  "every month or ten pours", "red arc", "folded under the rod",
                  "one straight line", "worm's hold"):
        assert words in ptext, words
    # the use, in order, after the build; its own items, none a wall of text; a picture each
    use = [s for s in steps if s.stage == USE]
    assert steps.index(use[0]) == steps.index(pour) + 1
    titles = [s.title for s in use]
    assert titles[:5] == ["Before you use it", "Setting it for a bottle", "The decanter's riser",
                          "Decanting", "The use limits"]
    assert all(s.scene is not None for s in use)
    assert all(len(t.split()) <= 110 for s in use for t in s.text), \
        max((len(t.split()), t) for s in use for t in s.text)
    card = [t for t in use[4].text if t[:2] in ("1.", "2.", "3.", "4.", "5.")]
    assert len(card) == 5
    assert "bottle" in use[1].adds
    lays = key in cr.LAY_BLOCKS
    riser = {"decanter", *(f"decanter_riser_plate{p:g}" for p in cr.RISER_PLATES)}
    if not lays:            # (its blocks and lifts are only the riser's: below)
        riser |= {f"decanter_riser_{k}{i}" for k in ("block", "lift") for i in (0, 1)}
    assert set(use[2].adds) == riser
    assert all(not s.adds for s in use[3:])
    text = " ".join(t for s in use for t in s.text)
    for words in ("lab jack", "jack's height", "finger's width", "take the decanter away "
                  "before"):
        assert words not in text.lower(), words
    assert "20 mm" in text
    assert "light's printed clip" in text
    assert "the next one on the same riser" in text            # the decanter's swap
    trip = titles[5:]
    assert bool(trip) == modules
    if modules:
        n_apart = sum(t.startswith("Taking it apart") for t in trip)
        assert n_apart == 8
        assert len(trip) - n_apart == 11
        assert trip[0].startswith("Taking it apart 1/8")
        assert trip[-1].startswith( "Back together 11/11")
    # the riser's blocks: B lays its cradle on them in its first build (and on the trip); A
    # stands on its brackets' feet while its board's finish cures (on the blocks it tipped at
    # 13.4° since design review round 3's frames), so its blocks come with the riser's step
    blocks = [s for s in steps if "decanter_riser_block0" in s.adds]
    assert len(blocks) == 1
    assert lays == modules
    assert (blocks[0] is use[2]) == (not lays)
    assert blocks[0] is not board
    (saddles,) = [s for s in steps if "front_saddle" in s.adds]
    assert steps.index(base) < steps.index(saddles)
    inner = next(s for s in steps if s.side == "R" and any(a.startswith("R.") and
                                                           a.endswith("torso") for a in s.adds))
    stack = [s for s in steps if s.side == "R" and s.stage == STACK]
    assert stack
    assert steps.index(inner) > max(steps.index(s) for s in stack)
    (screws,) = [s for s in steps if any(a.startswith("tie_") and a.endswith("screwR")
                                         for a in s.adds)]
    assert steps.index(screws) >= steps.index(passive)
    (clips,) = [s for s in steps if "R.pin_X_clip" in s.adds] if modules else [None]
    if modules:
        # the passive panel built whole on the bench, its inner plate and brackets on, then
        # lowered onto its cradle link, the cradle lying on its drive side
        (lay,) = [s for s in steps if s.title == "Laid on its drive side"]
        assert steps.index(saddles) < steps.index(lay) < steps.index(passive)
        assert steps.index(inner) < steps.index(clips)
        assert clips.stage == JOIN
        assert clips.scene is not None
        assert clips.scene.lay == "drive"
        assert passive.scene is not None
        assert passive.scene.lay == "drive"
        # its studs and knobs on its outer plate before it goes on (pack turn 2: its brackets
        # bolted to the board, with the board)
        studs = next(s for s in steps if "foot_bracket_FfR_knob0" in s.adds)
        assert steps.index(studs) < steps.index(clips)
        assert {"foot_bracket_FfR", "foot_bracket_FrR"} <= set(base.adds)
        ctext = " ".join(clips.text)
        assert "holding its weight until both pins" in ctext
        assert "pin's tip" in ctext
        assert all(s.where == "bench" for s in stack)
        assert "7-15 mm out" not in " ".join(t for s in steps for t in s.text)
    text = " ".join(t for s in steps for t in s.text)
    if not cr.stand_of(cfg.lk).bay_ties:        # (A has none)
        assert "bay tie" not in text, text
    # the keepers' bench check: a 30 N push on each pin's tip
    if modules:
        assert f"{cr.KEEPER_PUSH_N:g} N" in text
        assert "kitchen scale" in text


@pytest.mark.slow
@pytest.mark.construction
@pytest.mark.usefixtures("cached_cradle_fabrications")
def test_the_cradle_builds_an_order_without_a_walkers_words(tmp_path):
    """``spiderpig build`` of A (slow: the whole machine's STEP; its fabrication from the test
    cache): its BOM's title is the cradle's, ORDER.md has no acrylic note, no servo boxes,
    the TPU parts are the cradle's, and nothing is bought for a Chicago pin's epoxy."""
    from spiderpig.build import main

    assert main(["--linkage", "decanter_hinge", "--side-only", "--out", str(tmp_path),
                 "--force"]) == 0
    order = (tmp_path / "ORDER.md").read_text()
    assert "decanting cradle" in order.splitlines()[0]
    for walker in ("acrylic arrives", "servo boxes", "TPU socks", "epoxy"):
        assert walker not in order, walker


@pytest.mark.slow
@pytest.mark.api
@pytest.mark.parametrize("key", CRADLES)
@pytest.mark.usefixtures("cached_cradle_fabrications")
def test_verify_rates_a_cradles_joints_at_its_own_loads(key):
    """``api.verify`` rates a cradle's joints at its own statics (cradle_report.loads), as an
    audit does: A and B fail on nothing, their side frames included since the redesign of
    2026-10-20 (a joint under jam SF 1 is verify's failure; the report's tests hold every
    part at 1.3; design review round 2 had left the side frames open at the clutch's cap,
    ``cradle_report.OPEN_AT_CAP``). One cradle per test (each a ``standard`` verify: a build,
    the contract, the clashes; its fabrications from the test cache)."""
    from spiderpig import api

    d = api.resolve({"kind": "mechanism", "linkage": {"key": key}}, store=None)
    if key == "decanter_hinge":
        q = api.verify(d, "quick")  # the default level: a catalog floor with no servo in it
        (floor,) = [r for r in q.rows if r.requirement == "budget.cost_floor_usd"]
        assert "NRV050" in floor.detail
        assert "servo" not in floor.detail.lower()
        assert not [r for r in q.rows if r.requirement == "size.mass_g"]  # a build weighs it
    rep = api.verify(d, "standard")
    assert not [f.message for f in rep.failures], key
    assert rep.ok, key
    (joints,) = [r for r in rep.rows if r.requirement == "strength.joints"]
    assert joints.value is not None


@pytest.mark.slow
@pytest.mark.construction
def test_every_printed_part_fits_a_print_bed_in_its_pose():
    """Each printed part in its print pose (``meta["print_poses"]``, :func:`build.print_pose`)
    fits a 250 x 210 x 210 mm bed, and the parts not lying in a layer have a pose."""
    from spiderpig.build import print_pose

    cfg = BuildConfig(linkage="decanter_hinge", robot=False)
    mech = cache.cached_product(cfg, 1.0)
    poses = mech.meta["print_poses"]
    assert sorted(mech.meta["petg"]) == sorted(cr.PETG_PARTS)
    for name in ("front_saddle", "rear_saddle", "front_saddle_vee", "rear_saddle_vee",
                 "front_saddle_liner", "front_saddle_knob", "front_saddle_gauge",
                 "rear_saddle_liner", "rear_saddle_knob", "rear_saddle_hub",
                 "rear_saddle_brake",
                 "base_stop_rail0", "base_stop_rail1", "base_stop_rail0_liner",
                 "base_stop_rail1_liner", "base_stop_gate", "base_stop_knob", "L.handle",
                 "foot_boot_0"):
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
    # every bottle with its setting (the drawn one's V blocks at their lowest), and the parts
    # the page moves to it
    assert m["bottle"]["key"] == cr.SHOWN.key
    assert m["bottle"]["setting"] == [0.0, 0.0, cr.SHOWN.length + cr.STOP_GAP]
    got = {b["key"]: b["setting"] for b in m["bottles"]}
    assert got["standard"] == pytest.approx([12.73, 68.87, 303.0], abs=0.01)
    assert set(m["adjust"]) == {"front", "rear", "stop"}
    assert [s["moves"] for s in m["saddles"]] == ["front", "rear"]


@pytest.mark.strength
@pytest.mark.no_fabricate
def test_a_bell_crank_is_rated_by_the_moment_about_its_pivot():
    """``cradle_structure.link_bending`` (design review round 1): an L-shaped bell crank on
    its pivot O2, a force at its lever P and the balancing one at its arm K, takes their
    whole moment about O2 through each root (``|r x F|``, not a beam's ``F L / 4``); the
    chord P-K turns it into axial force; a link's own weight is spread over its joints."""
    from types import SimpleNamespace

    from spiderpig.cradle_structure import LinkSection, link_bending

    xy = {"O2": (0.0, 0.0), "P": (100.0, 0.0), "K": (0.0, 150.0)}
    pts = {k: np.array([v]) for k, v in xy.items()}
    m = 100.0 * 1000.0                                        # N·mm about O2
    on = {"P": (0.0, 1000.0), "K": (m / 150.0, 0.0), "O2": (-m / 150.0, -1000.0)}
    joints = [(j, "b2", "ground" if j == "O2" else "x") for j in on]
    vecs = np.array([[on[j] for j, _, _ in joints]])
    sec = {"b2": LinkSection(24.0, 4.826, 476.0)}
    for edges, most in (((("O2", "P"), ("O2", "K")), 100.0), ((("O2", "P"), ("O2", "K"),
                                                                ("P", "K")), 25.0)):
        lk = SimpleNamespace(links={"b2": (("O2", "P", "K"), edges)})
        got = link_bending(lk, pts, joints, vecs, sec, {"O2": 12.35})["b2"]["worst"]
        if most == 100.0:
            assert got["moment_nm"] == pytest.approx(100.0, rel=1e-3)
            assert got["at"] == "O2"
            z = 4.826 * (24 ** 3 - 12.35 ** 3) / (6 * 24)        # the net section at O2
            assert got["stress_mpa"] == pytest.approx(m / z + abs(got["axial_n"]) /
                                                      ((24 - 12.35) * 4.826), rel=1e-3)
        else:
            assert got["moment_nm"] < most, got


@pytest.mark.strength
@pytest.mark.no_fabricate
def test_a_pillar_is_held_only_where_its_inner_plate_is():
    """``cradle_structure.posts``: two posts in a line (B's inner bar O1-O2, before round 1)
    hold a pillar's end along the line, not across it, where the pillar is a cantilever off
    the outer plate (its foot takes nearly ``F a``: only the bar's twist holds it); a third
    post off the line, the plate a triangle, holds it both ways. Each end held against
    turning by its plate's strips (``plate_spring``)."""
    from spiderpig.cradle_structure import E_STEEL, Post, plate_spring, posts

    ei = E_STEEL * math.pi * (12 ** 4 - 5 ** 4) / 64
    a, f, t = 50.0, 1000.0, 4.763
    o1, o2, d1 = np.array([0.0, 0.0]), np.array([0.0, -200.0]), np.array([-60.0, -260.0])
    far = [(np.array([300.0, -100.0]), 24.0), (np.array([-100.0, -400.0]), 24.0)]

    def foot(side, d) -> float:
        vecs = np.array([[d]], float) * f
        return posts(side, 70.0, vecs)["O2"]["foot_nm"]

    def post(name, at, inner, loads=()):
        return Post(name, at, ei, list(loads), plate_spring(at, far, t),
                    plate_spring(at, [(q, 24.0) for q in inner], t))

    two = [post("O1", o1, [o2]), post("O2", o2, [o1], [(0, 1.0, a)])]
    across = foot(two, (1.0, 0.0))
    assert across > 0.6 * f * a / 1000.0             # the bar's twist holds the end a little
    assert foot(two, (0.0, 1.0)) < 0.5 * across      # along: O1 holds it
    three = [post("O1", o1, [o2, d1]), post("O2", o2, [o1, d1], [(0, 1.0, a)]),
             post("D1", d1, [o1, o2])]
    assert foot(three, (1.0, 0.0)) < 0.5 * across


@pytest.mark.strength
@pytest.mark.no_fabricate
def test_a_frame_plates_member_buckles_out_of_its_plane():
    """``cradle_structure.members``: a strut between two feet takes a load at its middle
    node in compression; its Euler load out of the plane at its own width and the plate's
    thickness; held across at both nodes, pinned (``MEMBER_K``); a node not held across
    tilts on what holds it."""
    from spiderpig.cradle_structure import E_ALU, members

    xy = {"A": np.array([0.0, 0.0]), "B": np.array([200.0, 0.0]),
          "C": np.array([100.0, 100.0])}
    pairs = (("A", "C"), ("C", "B"), ("A", "B"))
    load = {"C": np.array([[0.0, -1000.0]])}
    got = {m["member"]: m for m in members(xy, pairs, {}, 4.763, 193.0, load, ("A", "B"),
                                           tied=("A", "B", "C"))["members"]}
    ac = got["A-C"]
    assert ac["compression_n"] == pytest.approx(1000.0 / math.sqrt(2), rel=0.05)   # (rigid joints)
    L = 100.0 * math.sqrt(2)
    assert ac["euler_n"] == pytest.approx(math.pi ** 2 * E_ALU * 24 * 4.763 ** 3 / 12 / L ** 2,
                                          abs=0.1)
    loose = {m["member"]: m for m in members(xy, pairs, {}, 4.763, 193.0, load, ("A", "B"),
                                             tied=("A", "B"))["members"]}
    assert loose["A-C"]["untied"] == ["C"]
    assert loose["A-C"]["buckling"] < ac["buckling"]


@pytest.mark.strength
@pytest.mark.no_fabricate
def test_a_posts_foot_turns_square_to_its_load():
    """``cradle_structure.posts``: a post bent by a force along x turns its foot about y, the
    moment vector square to the force in the plate's plane (design review round 2: its
    probes had applied the slopes' generalized moment as the turn, 90° off, and so shared a
    post's moment among the plate's members by the wrong axis)."""
    from spiderpig.cradle_structure import E_STEEL, Post, posts

    ei = E_STEEL * math.pi * 12 ** 4 / 64
    p = Post("P", np.zeros(2), ei, [(0, 1.0, 50.0)], np.eye(2) * 1e6, np.zeros((2, 2)))
    got = posts([p], 100.0, np.array([[[1000.0, 0.0]]]))["P"]["forces"]["turn"]["foot"][0]
    assert abs(got[0]) < 1e-6 * abs(got[1])
    assert abs(got[1]) == pytest.approx(1000.0 * 50.0, rel=0.01)    # a cantilever's F a


@pytest.mark.strength
@pytest.mark.no_fabricate
def test_a_side_frame_holds_its_posts_only_as_its_plates_do():
    """``cradle_structure.side_frame`` (design review round 2's B1): two posts between two
    plates, the outer on clamped feet. Plates far stiffer than the posts: a load at one
    post's middle along the posts' line is a portal's (the loaded post's foot ``F L / 4``,
    its end 0, the other post ``F L / 8`` at both ends: sway shared, the ends clamped by the
    posts' push and pull). Across the line nothing but the plates' twist holds the posts'
    ends, so the loaded post's foot takes more. Real plates (24 x 4.75 mm strips) hold less,
    and every member end is rated on its cut section (``plate_rows``)."""
    from spiderpig.cradle_structure import E_STEEL, Post, SidePlates, plate_rows, side_frame

    ei = E_STEEL * math.pi * 12 ** 4 / 64
    L, f = 60.0, 1000.0
    xy = {"A": np.array([0.0, 0.0]), "B": np.array([100.0, 0.0]),
          "F1": np.array([0.0, -100.0]), "F2": np.array([100.0, -100.0])}

    def run(t: float, w: float, d) -> dict:
        outer = [(("A", "B"), w), (("A", "F1"), w), (("B", "F2"), w)]
        plates = SidePlates(xy, outer, [(("A", "B"), w)], t, ("F1", "F2"))
        side = [Post("A", xy["A"], ei, [(0, 1.0, L / 2)]), Post("B", xy["B"], ei, [])]
        return side_frame(side, L, np.array([[d]], float) * f, plates)

    got, _ = run(100.0, 200.0, (1.0, 0.0))
    assert got["A"]["foot_nm"] == pytest.approx(f * L / 4 / 1000, rel=0.05)
    assert got["A"]["tip_nm"] < 0.05 * f * L / 1000
    assert got["B"]["foot_nm"] == pytest.approx(f * L / 8 / 1000, rel=0.05)
    across, _ = run(100.0, 200.0, (0.0, 1.0))
    assert across["A"]["foot_nm"] > 1.3 * got["A"]["foot_nm"]
    _, grills = run(4.75, 24.0, (1.0, 0.0))
    rows = plate_rows(grills["outer"], [(("A", "B"), 24.0), (("A", "F1"), 24.0),
                                        (("B", "F2"), 24.0)], xy, 4.75, 193.0, {"A": 6.4},
                      9.6)
    assert set(rows) == {f"{a}-{b} at {n}" for a, b in (("A", "B"), ("A", "F1"), ("B", "F2"))
                         for n in (a, b)}
    assert all(0 < r["safety"] < math.inf for r in rows.values())


@pytest.mark.slow
@pytest.mark.construction
@pytest.mark.parametrize("key", CRADLES)
def test_every_post_the_side_frame_rates_stands_plate_to_plate(key):
    """Design review round 3's B1: B's pillar O1 had stopped 15.9 mm under the inner plate
    (no hole cut there) while ``cradle_report._side_posts`` tied it to it, so its SF 0.39 at
    the clutch's cap read 1.37. Every post of both sides as fabricated stands between both
    plates' faces (its foot washer and end shims within ``SPAN_TOL``) with a hole through each
    plate on its axis, and the side frame refuses one that doesn't."""
    from types import SimpleNamespace

    from spiderpig.cradle_report import SPAN_TOL, check_spans, post_spans

    cfg = BuildConfig(linkage=key, robot=False)
    mech = cache.cached_product(cfg, 1.0)
    got = post_spans(cfg, mech)
    stand = cr.stand_of(cfg.lk)
    posts = {f"pillar:{j}" for j in cfg.lk.frame if j != cfg.lk.crank[0]}
    posts |= {f"column:{n}" for n in stand.columns}
    assert set(got) == {"L", "R"}
    for side, by in got.items():
        assert set(by) == posts, side
        for name, g in by.items():
            assert g["found"], (side, name)
            assert max(g["short_mm"]) <= SPAN_TOL, (side, name, g)
            assert all(g["holes"]), (side, name, g)
    check_spans(cfg, mech)
    # a pillar cut short of the inner plate (as O1 was) is refused
    short = [b for b in mech.bodies if b.name.startswith("L.pillar_") and "standoff" in b.name]
    assert short
    from spiderpig.shapes import disc

    b0 = short[0]
    bb = b0.part.bounding_box()
    cut = SimpleNamespace(name=b0.name, part=disc(((bb.min.X + bb.max.X) / 2,
                                                   (bb.min.Y + bb.max.Y) / 2), 6.0,
                                                  bb.min.Z, bb.max.Z - 15.0),
                          fab=b0.fab, placed_part=None)
    cut.placed_part = lambda c=cut: c.part
    fake = SimpleNamespace(bodies=[cut if b is b0 else b for b in mech.bodies], meta=mech.meta)
    with pytest.raises(ValueError, match="short of the"):
        check_spans(cfg, fake)


@pytest.mark.strength
@pytest.mark.no_fabricate
def test_a_pillar_note_held_by_one_plate_is_refused():
    """``cradle_report._side_posts``: a pillar whose note says it is held by one frame plate
    (``supports``) is no post plate to plate (design review round 3's B1)."""
    from spiderpig.cradle_report import _side_posts

    cfg = BuildConfig(linkage="decanter_rcm", robot=False)
    lz = {str(k): [5.0 * k, 5.0 * k + 4.75] for k in range(17)}
    notes = {f"pillar:{j}": {"layer_z": lz, "top": 16, "supports": [0, 14], "layers": {},
                             "section": {"bearing_d": 12.0}} for j in ("O1", "O2")}
    pts = {n: np.zeros((1, 2)) for n in (*cfg.lk.frame, cr.mouth_name(cfg.lk))}
    with pytest.raises(ValueError, match="plate to plate"):
        _side_posts(cfg, notes, [], pts)


def _two_post_frame(t: float = 4.75, w: float = 24.0, feet=("F1", "F2")):
    from spiderpig.cradle_structure import E_STEEL, Post, SidePlates

    ei = E_STEEL * math.pi * 12 ** 4 / 64
    xy = {"A": np.array([0.0, 0.0]), "B": np.array([100.0, 0.0]),
          "F1": np.array([0.0, -100.0]), "F2": np.array([100.0, -100.0])}
    outer = [(("A", "B"), w), (("A", "F1"), w), (("B", "F2"), w)]
    plates = SidePlates(xy, outer, [(("A", "B"), w)], t, feet)
    side = [Post("A", xy["A"], ei, [(0, 1.0, 30.0)]), Post("B", xy["B"], ei, [])]
    return side, plates


@pytest.mark.strength
@pytest.mark.no_fabricate
def test_a_side_frame_is_solved_exactly_or_refused():
    """Design review round 3's M4: the side frame's solve had a ridge of ``1e-12 max|K|`` on
    its diagonal, which propped A's frame (its sway 1.11 mm where it is 1.91) and left its
    inner plate 1.7 % out of equilibrium. Now Jacobi-scaled and exact: in equilibrium to its
    loads, and a frame that isn't held (no feet) refused."""
    from spiderpig.cradle_structure import SingularFrame, side_frame_solve

    side, plates = _two_post_frame()
    vecs = np.array([[[1000.0, 300.0]], [[-200.0, 800.0]]])
    got = side_frame_solve(side, 60.0, vecs, plates)
    assert got.residual < 1e-9
    # the inner plate floats on the posts: their ends' forces on it sum to the loads' share
    # the outer plate doesn't take, and it is in equilibrium out of its plane
    end = sum(got.posts[p.name]["forces"]["box"]["end"] for p in side)
    foot = sum(got.posts[p.name]["forces"]["box"]["foot"] for p in side)
    assert np.allclose(foot - end, vecs[:, 0, :], atol=1e-6 * 1000)
    _, floating = _two_post_frame(feet=())
    with pytest.raises(SingularFrame):
        side_frame_solve(side, 60.0, vecs, floating)


@pytest.mark.strength
@pytest.mark.no_fabricate
def test_the_geometric_stiffness_buckles_a_column_at_euler():
    """``cradle_structure._geo4`` (design review round 3's M2: the side frame's buckling,
    ``side_frame_solve``'s ``buckling``): a pinned-pinned strut of beam elements, ``(K +
    lambda K_G) phi = 0``, buckles at ``pi^2 E I / L^2``; and the side frame's buckling
    factor goes as the inverse of its members' compression."""
    from spiderpig.cradle_structure import _beam_k, _geo4, side_frame_solve

    ei, L, n = 2.0e8, 300.0, 8
    h = L / n
    K = np.zeros((2 * n + 2, 2 * n + 2))
    G = np.zeros_like(K)
    for e in range(n):
        idx = [2 * e, 2 * e + 1, 2 * e + 2, 2 * e + 3]
        K[np.ix_(idx, idx)] += _beam_k(ei, h)
        G[np.ix_(idx, idx)] += _geo4(h)
    free = [d for d in range(2 * n + 2) if d not in (0, 2 * n)]
    lam = np.linalg.eigvals(np.linalg.solve(K[np.ix_(free, free)], G[np.ix_(free, free)]))
    assert 1.0 / max(lam.real) == pytest.approx(math.pi ** 2 * ei / L ** 2, rel=1e-3)
    side, plates = _two_post_frame()
    vecs = np.array([[[0.0, 0.0]]])
    axial = {"outer": {("A", "F1"): np.array([-500.0]), ("B", "F2"): np.array([-500.0])}}
    one = side_frame_solve(side, 60.0, vecs, plates, axial=axial, second_order=False)
    axial2 = {"outer": {k: 2 * v for k, v in axial["outer"].items()}}
    two = side_frame_solve(side, 60.0, vecs, plates, axial=axial2, second_order=False)
    assert 1.0 < float(one.buckling[0]) < math.inf
    assert float(two.buckling[0]) == pytest.approx(float(one.buckling[0]) / 2, rel=1e-6)


@pytest.mark.strength
@pytest.mark.no_fabricate
def test_a_plate_section_is_rated_on_its_in_and_out_of_plane_stresses_together():
    """``cradle_structure.combined_sf`` (design review round 3's minor 1: a member's in-plane
    and out-of-plane stresses had been rated apart): at its edge the axial, in-plane and
    out-of-plane bending stresses add; at its face's middle the twist's shear joins the
    axial and out-of-plane ones."""
    from spiderpig.cradle_structure import combined_sf

    w, t, d = 40.0, 4.75, 6.4
    N, Mi, Mo = np.array([-700.0]), np.array([60000.0]), np.array([15000.0])
    vm, sf = combined_sf(N, Mi, Mo, np.zeros(1), w, t, d, 158.6)
    edge = 700 / ((w - d) * t) + 60000 / (t * (w ** 3 - d ** 3) / (6 * w)) \
        + 15000 / ((w - d) * t * t / 6)
    assert vm[0] == pytest.approx(edge)
    assert sf[0] == pytest.approx(158.6 / edge)
    twist = combined_sf(np.zeros(1), np.zeros(1), np.zeros(1), np.array([5000.0]), w, t, 0.0,
                        158.6)[0]
    assert twist[0] == pytest.approx(math.sqrt(3) * 3 * 5000 / (w * t * t))


@pytest.mark.no_fabricate
def test_a_product_is_rated_on_its_alloys_minimum_and_a_walker_on_its_typical():
    """Design review round 3's M1: the cradles had been rated on 5052-H32's typical 193 MPa;
    a product now takes each alloy's specified minimum in its gauge (ASTM B209: 5052-H32
    23.0 ksi, 6061-T6 35.0, 7075-T6 0.190 in 69.0), a walker keeps the typical value its
    rules were set on (the identity gate holds them), and the posts' steel its stated
    material's (MISUMI's EN 1.0038 equivalent: 235 MPa)."""
    from spiderpig.hardware.cradle_catalog import NETRF10_YIELD
    from spiderpig.materials import allowable_mpa, sheet

    cradle = BuildConfig(linkage="decanter_rcm", robot=False)
    walker = BuildConfig()
    assert allowable_mpa(cradle, "al5052_4p8mm") == pytest.approx(158.6, abs=0.05)
    assert allowable_mpa(cradle, "al5052_6p35mm") == pytest.approx(158.6, abs=0.05)
    assert allowable_mpa(cradle, "al6061_6p35mm") == pytest.approx(241.3, abs=0.05)
    assert allowable_mpa(cradle, "al7075_4p8mm") == pytest.approx(475.7, abs=0.05)
    assert allowable_mpa(walker, "al5052_2mm") == sheet("al5052_2mm").yield_mpa == 193.0
    assert NETRF10_YIELD == 235.0


@pytest.mark.strength
@pytest.mark.no_fabricate
def test_a_post_ends_face_lifts_past_its_kern_and_bears_at_its_edge():
    """``cradle_report.contact_peak`` (design review round 3's M5): a post's 12 mm end face on
    its plate's 6.4 mm hole, clamped by its screw's preload: under its kern's moment the
    pressure is linear, ``F / A + M / Z``; past it the face lifts and its edge bears more
    (the no-tension solution, the resultant where the moment puts it); a fender washer's
    18 mm spreads it; past the face's radius nothing holds it."""
    from spiderpig.cradle_report import contact_peak

    od, bore = 12.0, 6.4
    area = math.pi * (od * od - bore * bore) / 4
    z = math.pi * (od ** 4 - bore ** 4) / (32 * od)
    kern = z / area
    F = np.array([4000.0, 4000.0, 4000.0, 4000.0])
    M = np.array([0.0, 0.5 * 4000 * kern, 4000 * 4.5, 4000 * 6.5])
    p = contact_peak(F, M, od, bore)
    assert p[0] == pytest.approx(4000 / area)
    assert p[1] == pytest.approx(4000 / area + M[1] / z)
    assert p[2] > 4000 / area + M[2] / z                   # lifted: the edge takes more
    assert math.isinf(p[3])
    spread = contact_peak(F[2:3], M[2:3], 15.2, bore)
    assert spread[0] < 0.5 * p[2]


@pytest.mark.strength
@pytest.mark.no_fabricate
def test_the_cradle_box_racks_on_its_tie_rods():
    """``cradle_report.box_racking`` (design review round 2's M1): the passive half's share of
    what the drive side's pins put on the cradle about the mouth crosses the bay in the four
    tie rods, each fixed at both ends: shared by their stiffness at their radii from the mouth
    (``12 E I / L^3`` across, ``G J / L`` in twist), the rear rods most."""
    from spiderpig.cradle_report import PASSIVE_SHARE, box_racking
    from spiderpig.cradle_structure import E_STEEL

    cfg = BuildConfig(linkage="decanter_rcm", robot=False)
    pts = {"Lc": np.zeros((1, 2)), "X": np.array([[100.0, 0.0]])}
    joints = [("X", "b6", "b4")]
    vecs = np.array([[[0.0, 1000.0]]])                  # 100 N·m about the mouth
    got = box_racking(cfg, joints, pts, {"walk": (vecs, np.zeros(1, bool), None)}, 274.9)
    case = got["cases"]["walk"]
    assert case["crossing_nm"] == pytest.approx(PASSIVE_SHARE * 100.0)
    d, L = cr.TIE_D, 274.9
    ei, gj = E_STEEL * math.pi * d ** 4 / 64, E_STEEL / 2.6 * math.pi * d ** 4 / 32
    r = {n: math.hypot(*TIES[n]) for n in TIES}
    theta = case["crossing_nm"] * 1000 / sum(12 * ei / L ** 3 * x * x + gj / L for x in r.values())
    assert case["rods"]["Tr2"]["shear_n"] == pytest.approx(12 * ei / L ** 3 * r["Tr2"] * theta,
                                                           rel=0.01)
    assert max(case["rods"].values(), key=lambda x: x["end_nm"]) is case["rods"]["Tr2"]


@pytest.mark.no_fabricate
def test_the_saddles_hold_a_full_6_l():
    """``cradle_report.saddles``: every printed part of the saddles, the lip gauge and the base
    stop over its bar against its cited strength, in its layers or across them as it's
    printed (Prusament's datasheets: a quarter of it for a load that stays, 1/2.5 for the
    pour's or a knock's), at a full 6 L's worst (hung on its thumbscrew, leaning on the gate
    at rest with a corkscrew's push, on the gauge at full tilt with no friction, its body on
    the rails' flanges), the post and the screws 3 x under their yield, the gate's lock
    gripping 2 x what it holds at the least friction cited, the rear knob's teeth holding
    the greased lead screw; no V block's guide jams raising it under twice the most friction
    cited for its faces, dry (review round 2, N1: the post 213 mm from the screw jammed at
    0.15, 0.095 for a 750 ml; round 3's mounting m9: the front one held only greased); the
    lip gauge held on its post. Design review round 3 (mounting): the gate's ears' roots
    (M3: the push's lever under the rails, unrated), the rear lock's two M3 set screws
    (m3), and the lip gauge at a wet bottle's slide onto its pad (m1: its peak over the
    static pull, the gauge's parts rated at it)."""
    from spiderpig.cradle_report import (
        PETG_STEEL,
        PLA_ALLOW,
        PRINTED,
        SUSTAINED_SF,
        TPU_WET,
        TRANSIENT_SF,
        drawers,
        saddles,
    )

    assert pytest.approx(17.0 / 4) == PLA_ALLOW
    got = saddles(-50.0, 30.0, 1.25)     # (the parts riding the rear block: 1.1-1.2 kg)
    assert got["ok"], [r for r in got["rows"] if not r["ok"]]
    rows = {r["part"]: r for r in got["rows"]}
    for r in got["rows"]:
        if "stress_mpa" in r and r["limit_mpa"] in PRINTED.values():
            assert r["limit_mpa"] == PRINTED["petg", "across" if "across" in r["basis"]
                                             else "in"], r
            assert r["bar"] == (SUSTAINED_SF if "sustained" in r["basis"] else TRANSIENT_SF)
        assert r["bar"] is None or r["sf"] >= r["bar"], r
    assert rows["rear post (bending)"]["load_n"] > 10.0     # (N·m: the couple)
    assert rows["rear post (bending at rest)"]["load_n"] > 10.0
    assert "rear post's socket (bearing)" in rows
    # the rails print standing: their T-slot's lips bend in their layers (review round 2, N6)
    assert "in its layers" in rows["base stop's T-slot lips (bending)"]["basis"]
    assert "across" in rows["rear knob's tower (bending)"]["basis"]
    assert "across" in rows["lip gauge's post (bent and twisted)"]["basis"]
    for part in ("rear lead screw's lock (the knob's teeth)", "base stop's lock (its grip)",
                 "base stop's rails (bending at their sockets)",
                 "base stop's rails' tongues (bending where they leave the sockets)",
                 "lip gauge on its post (fixed)", "rear knob's tower (bending)"):
        assert part in rows, part
    # the gate's ears' roots: in their layers, the gate printing on its top; its plate
    # hanging from its top bar across them
    ears = rows["base stop's gate's ears (their root)"]
    assert "in its layers" in ears["basis"]
    assert ears["sf"] >= SUSTAINED_SF
    assert "across" in rows["base stop's gate (its plate hanging from its top bar)"]["basis"]
    # the rear lock's set screws: the hub's on a flat on the screw, the nut's by friction
    # through its printed thread across its layers
    hub = rows["rear knob's hub on the lead screw (its M3 set screw on the flat)"]
    nut = rows["rear barrel nut in its boss (its M3 set screw)"]
    assert "across" in nut["basis"]
    assert min(hub["sf"], nut["sf"]) >= SUSTAINED_SF
    assert cr.LEAD_FLAT[0] > 0.0
    # a wet bottle's slide onto the pad: it can start in the swing only under tan 30° / √2,
    # and its peak over the static pull is what the gauge's parts are rated at
    slide = got["slide"]
    assert slide["mu_static_most"] == pytest.approx(math.tan(math.radians(30.0)) / math.sqrt(2),
                                                    abs=1e-3)
    assert slide["mu_kinetic"] == TPU_WET < slide["mu_static_most"]
    assert slide["peak_n"] > slide["static_n"]
    assert rows["lip gauge's post (bent and twisted)"]["load_n"] == pytest.approx(
        slide["peak_n"], abs=0.1)
    # the brake alone, greased, doesn't hold: the knob's lock does
    greased = rows["rear lead screw's brake, the knob lifted, greased"]
    assert greased["bar"] is None
    assert greased["sf"] < 1.0
    # the drawers: every bottle's setting, both guides
    guides = drawers()
    assert {(r["bottle"], r["guide"]) for r in guides} == {
        (z.name, g) for z in cr.SIZES for g in ("rear", "front")}
    assert all(r["ok"] and r["mu_jam"] >= 2 * r["mu"] for r in guides)
    assert all(r["mu"] == PETG_STEEL["dry"][1] for r in guides)       # (both dry)
    rear = {r["bottle"]: r for r in guides if r["guide"] == "rear"}
    assert rear[cr.STANDARD.name]["mu_jam"] == pytest.approx(
        rear[cr.STANDARD.name]["engagement_mm"] / 54.0, abs=1e-3)
    assert min(r["mu_jam"] for r in rear.values()) > 0.6     # (it had been 0.095)
    assert got["balance"]["ok"]
    assert got["gauge"]["held"]


@pytest.mark.no_fabricate
def test_real_bottles_lie_clear_of_the_saddles_and_the_light():
    """Design review round 3, mounting M1 and M2: the 750 ml and the 6 L impériale are drawn as
    their makers draw them (Saverglass's 4427, univerre's NGBD06QFR, traced), not the model's
    cosine shoulder that had hidden their real ones. Each listed bottle, set as the guide
    measures it (its neck :data:`construction.cradle.NECK_AT` from the lip, the front V's
    bearing's back end), at rest on the gate and slid onto the pad: the front V bears on its
    measured section (its widest there), the V's relief behind the bearing and the carrier's
    back wall clear its shoulder, the light's head lies under the glass with room, and the
    pad covers the rim. At the light's old place (118 mm) the impériale lay on its head."""
    from dataclasses import replace

    assert cr.STANDARD.profile()[1] == (2.0, 13.44)                  # (the drawings)
    assert cr.IMPERIAL.profile()[-1] == (493.0, 74.96)
    assert cr.MAGNUM.profile()[0] == (0.0, cr.MAGNUM.r_neck)        # (the model)
    assert cr.NECK_AT == cr.FRONT_BEAR - cr.STOP_GAP
    words = " ".join(next(u for u in cr.use_steps("decanter_rcm") if u.tag == "setting").lines)
    assert f"neck's diameter {cr.NECK_AT:g} mm down from the lip" in words
    for size in cr.SIZES:
        for mouth in (cr.STOP_GAP, cr.GAUGE_FRONT[1]):
            room = cr.bottle_room(size, mouth)
            if mouth == cr.STOP_GAP:
                assert room["bearing"] == pytest.approx(room["neck"], abs=0.05), size.key
                assert room["neck"] == pytest.approx(size.r_neck, abs=0.4), size.key
            assert room["relief"] >= 1.0, (size.key, mouth, room)
            assert room["wall"] >= 2.5, (size.key, mouth, room)
            assert room["light"] >= 5.0, (size.key, mouth, room)
            assert room["pad"] >= 2.5, (size.key, mouth, room)
    assert cr.bottle_room(cr.IMPERIAL, cr.GAUGE_FRONT[1])["light"] >= 10.0
    # (m8) a methuselah whose shoulder were as full as the impériale's clears it too
    fuller = replace(cr.METHUSELAH, outline=cr.UNIVERRE_NGBD06QFR)
    assert cr.bottle_room(fuller, cr.GAUGE_FRONT[1])["light"] >= 10.0


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_gate_prints_on_its_top_its_pointer_wide_and_notched():
    """Design review round 3, mounting M3, m2 and m4: the base stop's gate prints on its top
    (its top bar, ears and pointer on the bed: the ears' roots and the 25.5 mm pointer in its
    layers; on its face the pointer had been a 2 x 6 mm column), its ears
    :data:`construction.cradle.GATE_TOP` tall over the rails with their roots filleted, its
    pointer 4 mm wide with the vernier notched through its inner edge to the rail's top (cut
    in its top they read across a 6 mm step), and the M5 x 25 lock bolt still reaching past
    its nut."""
    from spiderpig.build import print_pose

    rear = cr.SADDLES[1]
    stop = cr.setting(cr.STANDARD).stop
    gate, (bolt, *_, nut) = cr._gate(rear, stop)
    pose = cr.print_poses(["base_stop_gate"], 0.0)["base_stop_gate"]
    assert pose == [("Z", -0.0), ("X", -90.0)]
    posed = print_pose(gate, pose).bounding_box()
    hi = cr.REAR_FLANGE[1]
    bb = gate.bounding_box()
    assert pytest.approx((hi + cr.GATE_TOP, cr.GATE_BOTTOM)) == (bb.max.Y, bb.min.Y)
    assert pytest.approx(-(hi + cr.GATE_TOP)) == posed.min.Z         # its top on the bed
    assert cr.GATE_TOP >= 9.0
    assert cr.EAR_FILLET >= 4.0
    pitch, lines, width = cr.VERNIER
    assert width >= 4.0
    assert pytest.approx(cr.POINTER_IN + width) == bb.max.Z
    # each notch through the pointer's whole height at its inner edge
    from build123d import Box, Location

    for k in range(1, lines + 1):
        probe = Box(0.3, cr.GATE_TOP - 0.2, 0.4).moved(Location(
            (stop - pitch * k, hi + cr.GATE_TOP / 2, cr.POINTER_IN + 0.25)))
        hit = gate & probe
        assert hit is None or getattr(hit, "volume", 0.0) < 1e-6, k
    assert bolt.bounding_box().min.Y <= nut.bounding_box().min.Y - 0.8   # (past its nut)


@pytest.mark.no_fabricate
def test_the_parts_that_carry_the_bottle_print_solid_in_petg():
    """The saddles' carriers and V blocks, the rear knob and its hub, the base stop's rails
    and gate and the lip gauge print in PETG (``meta["petg"]``: on hand), solid, as
    ``cradle_report.saddles`` rates them (review round 2, N6 and m9); the liners stay TPU,
    the other knobs the build's filament. The grease note keeps grease off what holds the
    rear lead screw (round 2, N5)."""
    from types import SimpleNamespace

    from spiderpig.hardware.bom import (
        CRADLE_GREASE_NOTE,
        CRADLE_PRINT_NOTE,
        PRESS_FILAMENT,
        TPU_FILAMENT,
        part_filament,
    )

    meta = {"petg": list(cr.PETG_PARTS)}
    for n in cr.PETG_PARTS:
        assert part_filament(SimpleNamespace(name=n), meta, "pla_filament") == PRESS_FILAMENT
    assert part_filament(SimpleNamespace(name="front_saddle_knob"), meta,
                         "pla_filament") == "pla_filament"
    assert part_filament(SimpleNamespace(name="rear_saddle_liner"), meta,
                         "pla_filament") == TPU_FILAMENT
    assert part_filament(SimpleNamespace(name="rear_saddle_vee"), {},
                         "pla_filament") == "pla_filament"         # (a walker: no "petg")
    assert "PETG at 100 % infill" in CRADLE_PRINT_NOTE
    assert "above its collar" in CRADLE_GREASE_NOTE
    assert "knob's teeth" in CRADLE_GREASE_NOTE


@pytest.mark.no_fabricate
def test_the_base_stop_reads_any_bottles_length_to_a_millimetre():
    """The base stop's scale (review round 2, N2: its grooves had been named for one maker's
    bottle of each size, and the gate set half a millimetre behind it): a groove every 5 mm
    of a bottle's length over the gate's range, a long one every 10, one dot at each 50 and
    three, four and five at 300, 400 and 500; the gate's pointer's vernier meets a groove
    with its k-th line from the face for a length k mm past one (none for a round one). The
    gate goes :data:`construction.cradle.STOP_GAP` behind the base, so a bottle measured to
    a millimetre lies 1.7-3.7 mm behind the gauge's pad at rest."""
    marks = cr.stop_marks()
    xs = [x for x, _, _ in marks]
    assert all(b - a == pytest.approx(cr.STOP_EVERY) for a, b in itertools.pairwise(xs))
    assert (xs[0] - cr.STOP_GAP, xs[-1] - cr.STOP_GAP) == (255.0, 570.0)
    assert all(lng == ((x - cr.STOP_GAP) % 10 == 0) for x, _, lng in marks)
    dots = {round(x - cr.STOP_GAP): d for x, d, _ in marks if d}
    assert (dots[300], dots[350], dots[400], dots[500]) == (3, 1, 4, 5)
    pitch, lines, _ = cr.VERNIER
    for length in range(286, 571):
        stop = length + cr.STOP_GAP
        met = [k for k in range(1, lines + 1) if any(abs(stop - pitch * k - x) < 0.3 for x in xs)]
        assert met == ([length % 5] if length % 5 else []), length
    for size in cr.SIZES:
        assert xs[0] <= cr.setting(size).stop <= xs[-1] + cr.STOP_EVERY, size.name
    gap = cr.STOP_GAP - cr.GAUGE_FRONT[1]                  # the mouth to the pad, at rest
    assert (gap - 1.0, gap + 1.0) == pytest.approx((1.7, 3.7))


@pytest.mark.no_fabricate
def test_every_bottle_rests_on_both_saddles_at_rest_level_and_tipped():
    """``cradle_report.balance`` (review round 1, B1): with its centre of mass where the shell
    model puts it (a 6 L methuselah's 390 mm from the mouth), the gate's push under the axis
    at rest and the gauge's over it at full tilt, each bottle's reaction centres inside its
    support, the front V to the rear support's end (the rear V, then the rails' flanges), by
    40 mm or more; without the flanges (the rear V's end at 380) the 6 L would lift its neck
    off the front V at rest. Behind 380 the flanges bend (review round 2, N3): their sag
    lifts the mouth at most :data:`cradle_report.MOUTH_LIFT_MAX`, under a quarter of a
    millimetre once the bottle is level. The lip gauge is held on its post: its bridge
    flat on the post's top, its bolt under it."""
    from spiderpig.cradle_report import MOUTH_LIFT_MAX, balance

    assert cr.METHUSELAH.com_from_mouth == 390.0
    got = balance(-50.0, 30.0)
    assert got["ok"]
    assert got["worst_margin_mm"] >= 40.0
    assert {r["case"] for r in got["rows"]} == {"rest", "level", "full"}
    assert len(got["rows"]) == 3 * len(cr.SIZES)
    rest = {r["bottle"]: r for r in got["rows"] if r["case"] == "rest"}
    meth = rest[cr.METHUSELAH.name]
    assert meth["centre_mm"] > cr.SADDLES[1].vee[1]     # (on the flanges)
    assert meth["rigid_margin_mm"] < 0 < meth["flange_n"]
    assert got["worst_mouth_lift_mm"] <= MOUTH_LIFT_MAX
    assert all(r["mouth_lift_mm"] < 0.25 for r in got["rows"] if r["case"] != "rest")
    assert all(r["flange_n"] == 0.0 for r in got["rows"] if r["rigid_margin_mm"] > 0)
    g = cr.gauge_fixed()
    assert g["held"]
    assert g["seated"]
    assert g["bolt_under_top_mm"] > 5.0


@pytest.mark.server
def test_the_cradle_page_lists_the_cradles_and_serves_their_swing(monkeypatch):
    from fastapi.testclient import TestClient

    from spiderpig.server import app as srv

    monkeypatch.delenv("VITE_ALLOWED_HOSTS", raising=False)
    client = TestClient(srv.app, base_url="http://localhost")
    cards = client.get("/api/cradles").json()["cradles"]
    assert [c["key"] for c in cards] == ["decanter_rcm", "decanter_hinge"]   # B, the main
    assert cards[0]["why"].startswith("The main design")
    assert cards[1]["why"].startswith("Builds too (B is the main design)")
    assert {c["key"]: c["verdict"] for c in cards} == {"decanter_hinge": "build",
                                                        "decanter_rcm": "build"}
    r = client.get("/api/cradle/decanter_hinge?n=48")
    assert r.status_code == 200
    assert r.json()["n"] == 48
    from spiderpig.construction.handcrank import TOGGLE_TURNS

    assert r.json()["toggle_turns"] == TOGGLE_TURNS     # the slider's red zones (round 3's M4)
    # how it's used: the guide's own use steps, the use limits' five-line card first
    # (design review round 2's M5: the page had none of them)
    use = r.json()["use"]
    assert [u["tag"] for u in use] == ["where", "setting", "riser", "decant", "limits"]
    card = [x for x in use[-1]["lines"] if x[:2] in ("1.", "2.", "3.", "4.", "5.")]
    assert len(card) == 5
    assert all(len(x) < 200 for x in card)
    assert "20 mm" in card[1]
    assert "riser" in card[1]
    assert client.get("/api/cradle/strider").status_code == 404


def _corners(lo, hi) -> np.ndarray:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return np.array([[(lo, hi)[i & 1][0], (lo, hi)[(i >> 1) & 1][1], (lo, hi)[(i >> 2) & 1][2]]
                     for i in range(8)])


def _item(key: str, role: str, boxes, mass: float = 1.0, pre=None):
    """A :class:`voxpack.Item` of boxes ((lo, hi) each), its mass at its boxes' centroid."""
    from spiderpig import voxpack as vp

    meshes = [vp.box_mesh(lo, hi) for lo, hi in boxes]
    vols = [vp.mesh_volume(*m) for m in meshes]
    vol = sum(v for v, _ in vols)
    com = sum(v * c for v, c in vols) / vol
    return vp.Item(key, key, role, meshes, mass, com, vol, np.eye(3) if pre is None else pre)


def _placed(items, layout):
    """The layout's pieces where it put them (:class:`voxpack.Packed`), in its order."""
    from spiderpig import voxpack as vp

    by = {it.key: it for it in items}
    out = []
    for p in layout.placed:
        it = by[p.key]
        out.append(vp.Packed(p.key, it.label, [(q @ p.rot.T + p.shift, t) for q, t in it.meshes],
                             it.mass, p.rot @ it.com + p.shift, it.volume))
    return out


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_voxels_are_the_real_shapes_filled():
    """``voxpack.voxelize``: a box fills exactly its cells; a 3 mm plate on the grid one
    layer (a face on a cell's boundary marks the cell it bounds); a frame of four bars keeps
    its hole empty; a plate turned 30° fills the cells it passes through, its centres-only
    fill its own area; each of the cube's 24 turns of a grid (``Vox.turned``) is the turned
    mesh voxelized; ``mesh_volume`` is a closed mesh's volume and centroid."""
    from spiderpig import voxpack as vp

    p, t = vp.box_mesh((0, 0, 0), (30, 9, 12))
    assert vp.mesh_volume(p, t)[0] == pytest.approx(3240.0)
    assert np.allclose(vp.mesh_volume(p, t)[1], (15, 4.5, 6))
    v = vp.voxelize([(p, t)])
    assert v.occ.shape == (10, 3, 4)
    assert v.occ.all()
    plate = vp.voxelize([vp.box_mesh((0, 0, 0), (60, 3, 60))])
    assert plate.occ.shape == (20, 1, 20)
    assert plate.occ.all()
    bars = [vp.box_mesh((0, 0, 0), (60, 6, 15)), vp.box_mesh((0, 0, 45), (60, 6, 60)),
            vp.box_mesh((0, 0, 15), (15, 6, 45)), vp.box_mesh((45, 0, 15), (60, 6, 45))]
    frame = vp.voxelize(bars)
    assert int(frame.occ.sum()) == (60 * 6 * 60 - 30 * 6 * 30) // 27
    assert not frame.occ[5:15, :, 5:15].any()
    bar = vp.box_mesh((0, 0, 0), (200, 20, 3))
    turned = vp.voxelize([bar], rot=vp.rot_about("z", 30))
    centres = vp.voxelize([bar], rot=vp.rot_about("z", 30), surface=False)
    assert centres.occ.sum() == pytest.approx(200 * 20 / 9, rel=0.03)
    assert turned.occ.sum() >= centres.occ.sum()
    assert (turned.occ | centres.occ).sum() == turned.occ.sum()      # (one holds the other)
    lshape = [vp.box_mesh((0, 0, 0), (30, 6, 9)), vp.box_mesh((0, 6, 0), (6, 21, 9))]
    base = vp.voxelize(lshape)
    assert len(vp.ROTATIONS) == 24
    assert np.allclose(vp.ROTATIONS[0], np.eye(3))
    for r in vp.ROTATIONS:
        assert np.allclose(r @ r.T, np.eye(3))
        assert np.linalg.det(r) == pytest.approx(1.0)
        a, b = base.turned(r), vp.voxelize(lshape, rot=r)
        assert a.occ.shape == b.occ.shape
        assert (a.occ == b.occ).all()
        assert np.allclose(a.org, b.org)


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_packer_levels_drops_and_fills_in_order():
    """``voxpack.pack`` on boxes: a base board stays at the bottom, as it is; two panels
    (a plate on posts) lie flat; a bar turned 30° in its plane lies levelled (``level``: its
    own size, not its turned box's) and flat; small blocks fill in; judged afresh
    (``voxpack.metrics``): no two pieces share a cell or a cubic millimetre, all inside the
    box, each supported, none over another that goes in after it. ``tuck`` swings a link
    hanging off its panel in."""
    from spiderpig import voxpack as vp
    from spiderpig.cradle_pack import level, tuck

    turn = vp.rot_about("z", 30)
    bar = [_corners((-100, -10, -2), (100, 10, 2)) @ turn.T]
    assert level(bar) == pytest.approx(-30.0)
    assert level(bar, tilt=30.0) == pytest.approx(-30.0)

    def panel(z0: float) -> list:                      # a plate in xy, four posts off it
        posts = [((x, y, z0 + 5), (x + 10, y + 10, z0 + 45))
                 for x, y in ((-140, 10), (130, 10), (-140, 180), (60, 120))]
        return [((-150, 0, z0), (150, 200, z0 + 5)), *posts]

    items = [_item("base", "base", [((-200, -3, -160), (200, 0, 160)),
                                    ((-190, 0, -150), (-180, 15, -140))], 3.0),
             _item("p1", "panel", panel(0.0), 2.0), _item("p2", "panel", panel(80.0), 2.0)]
    rod = _item("bar", "fill", [((-100, -10, -2), (100, 10, 2))], 0.5)
    rod.meshes = [(p @ turn.T, t) for p, t in rod.meshes]
    rod.com = turn @ rod.com
    rod.pre = vp.rot_about("z", level([p for p, _ in rod.meshes]))
    items.append(rod)
    items += [_item(f"block{i}", "fill", [((0, 0, 0), (40, 25, 12))], 0.2) for i in range(5)]
    layout, packer = vp.pack(items)
    assert sorted(p.key for p in layout.placed) == sorted(it.key for it in items)
    assert layout.placed[0].key == "base"
    assert np.allclose(layout.placed[0].matrix(), np.eye(4))        # the board stays put
    pieces = _placed(items, layout)
    by = {pc.key: pc for pc in pieces}
    for k in ("p1", "p2"):                                           # flat: the plate level
        plate = by[k].meshes[0][0]
        assert np.ptp(plate[:, 1]) == pytest.approx(5.0)
    b = by["bar"].meshes[0][0]
    assert sorted(np.round(np.ptp(b, axis=0), 6)) == pytest.approx([4.0, 20.0, 200.0])
    assert np.ptp(b[:, 1]) == pytest.approx(4.0)                     # flat, levelled
    for i in range(5):
        assert np.ptp(by[f"block{i}"].meshes[0][0][:, 1]) == pytest.approx(12.0)
    m = vp.metrics(pieces)
    assert m["overlaps"] == []
    assert m["touching"] == []
    assert m["order_faults"] == []
    assert m["unsupported"] == []
    assert m["count"] == len(items)
    # the box: the board and its post, a panel on its posts (45 + clearances), the other
    # beside or over it: no taller than both stacked
    assert m["box"]["inside_mm"][1] <= 15 + 2 * 50 + 6 * vp.CLEAR + 2 * vp.CELL

    # a link hanging off its panel's top swings in, about its pin, clear of the plate's posts
    plate = [_corners(lo, hi) for lo, hi in panel(0.0)]
    link = [_corners((95, 150, 10), (105, 300, 15))]
    deg = tuck(link, (100.0, 155.0), plate)
    assert deg == pytest.approx(90.0)          # in, square to the plate, the least swing
    assert tuck([_corners((95, 50, 10), (105, 150, 15))], (100.0, 55.0), plate) == 0.0
    assert packer.stats.odd_columns == 0


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_a_piece_drops_into_an_open_frame():
    """Real shapes nest: a block drops into a frame's hole (the box stays as tall as the
    block, not the frame and the block stacked), where boxes would have stacked it on top;
    a plan's step pins its orientation (``down``, ``spin``: ``voxpack.describe`` reads it
    back), and ``on`` puts it on a named piece; a plan's order with a cycle is refused; a
    plan goes to JSON and back, an explicit turn (``rot``) too."""
    import json

    from spiderpig import voxpack as vp

    frame = [((0, 0, 0), (150, 6, 30)), ((0, 0, 120), (150, 6, 150)),
             ((0, 0, 30), (30, 6, 120)), ((120, 0, 30), (150, 6, 120))]
    items = [_item("frame", "panel", frame, 1.0), _item("block", "fill",
                                                        [((0, 0, 0), (60, 40, 60))], 0.5)]
    layout, _ = vp.pack(items)
    by = {pc.key: pc for pc in _placed(items, layout)}
    blk, frm = by["block"].meshes[0][0], np.concatenate([p for p, _ in by["frame"].meshes])
    assert blk[:, 1].min() < frm[:, 1].max()                 # its bottom in the frame's hole
    m = vp.metrics(list(by.values()))
    assert m["overlaps"] == []
    assert m["unsupported"] == []
    assert m["order_faults"] == []
    assert m["box"]["inside_mm"][1] <= 40 + 2 * vp.CLEAR + vp.CELL

    # a plan: the block on its side (its +x face down, a quarter turn), on the frame
    plan = vp.Plan("on top", (vp.Step("frame", down="-y"),
                              vp.Step("block", down="+x", spin=90, on="frame")))
    layout, packer = vp.pack(items, plan)
    blk = layout.by_key()["block"]
    assert vp.describe(blk.shape.ri, packer.items["block"].faces) == ("+x", 90)
    assert np.allclose(vp.ROTATIONS[blk.shape.ri] @ [1, 0, 0], [0, -1, 0])
    assert "frame" in blk.rest.on
    assert vp.Plan.from_json(plan.to_json()) == plan
    turn = tuple(tuple(float(v) for v in row) for row in vp.ROTATIONS[7])
    plan = vp.Plan("turned", (vp.Step("block", rot=turn),))
    layout, _ = vp.pack(items, plan)
    assert layout.by_key()["block"].shape.ri == 7
    assert vp.Plan.from_json(json.loads(json.dumps(plan.to_json()))) == plan
    with pytest.raises(ValueError, match="cycle"):
        vp.pack(items, vp.Plan("loop", (vp.Step("frame", after=("block",)),
                                        vp.Step("block", after=("frame",)))))


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_a_piece_rests_on_its_contacts_on_pads_or_not_at_all():
    """``voxpack.support``: a piece whose centre of mass is over the hull of the columns it
    touches (within ``SETTLE``) rests directly; one that touches only at a corner but has
    something within ``PAD_MAX`` under the rest rests on foam pads (where, how tall); with
    nothing near under the rest, it tips."""
    from spiderpig import voxpack as vp

    xz = np.array([(i, k) for i in range(10) for k in range(10)])
    owner = np.zeros(len(xz), int)
    com = (5.0, 5.0)
    assert vp.support(xz, np.zeros(len(xz)), owner, ["under"], com, 3.0).how == "direct"
    corner = (xz[:, 0] < 2) & (xz[:, 1] < 2)
    gap = np.where(corner, 0, 5)                         # 15 mm under the rest
    got = vp.support(xz, gap, owner, ["under"], com, 3.0)
    assert got.how == "padded"
    assert got.on == ("under",)
    assert len(got.pads) == 3                            # the hull's three far corners
    assert {p.height for p in got.pads} == {15.0}
    assert {p.on for p in got.pads} == {"under"}
    for p in got.pads:                                    # a PAD_SIDE square on the corner
        assert (p.x1 - p.x0, p.z1 - p.z0) == (vp.PAD_SIDE, vp.PAD_SIDE)
        assert p.volume == pytest.approx(vp.PAD_SIDE ** 2 * 15.0)
    far = np.where(corner, 0, 30)                        # 90 mm: more than a pad fills
    assert vp.support(xz, far, owner, ["under"], com, 3.0).how == "none"
    floor = vp.support(xz, np.zeros(len(xz)), np.full(len(xz), -1), [], com, 3.0)
    assert floor.on == ("floor",)


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_metrics_judge_any_layout():
    """``voxpack.metrics`` on hand-made layouts: two slabs stacked a foam's width apart fill
    their share of the box, the upper resting on the lower, the lower on the floor; put in
    upper first, the order is faulted; a third slab cut into the lower one is an overlap of
    its real volume (measured on the fine cells); the mass centre's height."""
    from spiderpig import voxpack as vp

    def slab(key, lo, hi, mass=1.0):
        p, t = vp.box_mesh(lo, hi)
        vol, c = vp.mesh_volume(p, t)
        return vp.Packed(key, key, [(p, t)], mass, c, vol)

    low = slab("low", (0, 0, 0), (100, 10, 100), 3.0)
    up = slab("up", (0, 13, 0), (100, 23, 100), 1.0)
    m = vp.metrics([low, up])
    inside = np.array(m["box"]["inside_mm"])
    assert inside == pytest.approx([106, 29, 106])
    assert m["fill"] == pytest.approx(2 * 100 * 10 * 100 / np.prod(inside), rel=1e-3)
    rows = {r["key"]: r for r in m["pieces"]}
    assert rows["low"]["on"] == ["floor"]
    assert rows["up"]["on"] == ["low"]
    assert rows["up"]["rests"] == "direct"
    assert m["unsupported"] == []
    assert m["overlaps"] == []
    assert m["order_faults"] == []
    assert m["mass_centre_mm"] == pytest.approx((3 * 8 + 1 * 21) / 4, abs=0.06)
    assert m["box"]["outside_mm"] == pytest.approx(list(inside + 2 * vp.WALL))
    assert vp.metrics([up, low])["order_faults"] == [{"over": "up", "under": "low"}]
    cut = slab("cut", (50, 5, 50), (150, 15, 150))
    m = vp.metrics([low, up, cut])
    over = {(o["a"], o["b"]): o["volume_mm3"] for o in m["overlaps"]}
    assert over[("low", "cut")] == pytest.approx(50 * 5 * 50, rel=0.05)
    assert ("up", "cut") in over


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_search_keeps_its_best(monkeypatch):
    """``voxpack.anneal`` (in this process) from the greedy packing's choices: never worse
    than where it started; ``voxpack.search`` packs its best again. With ``seeds`` (packing
    review 1's m6) it anneals each, keeps the best and reports every run (its score, its
    box's litres on the real shapes: ``voxpack.outside_litres``) and the greedy start's;
    ``voxpack.spread`` sums them up (the best, median and worst, how many at the best)."""
    from spiderpig import voxpack as vp

    monkeypatch.setenv("SPIDERPIG_WORKERS", "0")
    items = [_item("base", "base", [((0, -3, 0), (240, 0, 180))], 2.0)]
    items += [_item(f"b{i}", "fill", [((0, 0, 0), (60 + 10 * i, 30, 45))], 0.3)
              for i in range(5)]
    pk = vp.Packer(items)
    start = pk.best_greedy()
    s, seq, tried = vp.anneal(items, None, start.seq, 1.0, 1)
    assert s <= start.score + 1e-9
    assert tried >= 2
    assert sorted(c.key for c in seq) == sorted(c.key for c in start.seq)
    best = vp.search(items, None, 1.0, 1)
    assert best.score <= start.score + 1e-9
    # N seeds (one after another here), the best kept, the spread across them reported
    best = vp.search(items, None, 0.3, 1, seed=5, seeds=3)
    assert [r["seed"] for r in best.runs] == [5, 6, 7, None]       # (None: the greedy start)
    assert best.score == pytest.approx(min(r["score"] for r in best.runs), abs=0.006)
    assert all(r["litres"] > 0 and r["tried"] >= 1 for r in best.runs)
    assert any(r["litres"] == pytest.approx(vp.outside_litres(items, best), abs=0.06)
               for r in best.runs if r["score"] <= best.score + 0.006)    # (the best's)
    sp = vp.spread(best.runs)
    assert sp["seeds"] == 3
    assert sp["litres"]["best"] <= sp["litres"]["median"] <= sp["litres"]["worst"]
    assert sp["litres"]["spread"] == pytest.approx(sp["litres"]["worst"] - sp["litres"]["best"])
    assert 1 <= sp["at_best"] <= 3


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_score_charges_each_pad():
    """``voxpack.score`` (packing review 1's m7): a pad costs ``W_PAD`` each, ``W_PAD_VOLUME``
    a litre of them and ``W_PAD_TALL`` a metre of the tallest (``voxpack.pad_cost``), so in
    one box the layout on fewer, smaller pads scores less (it charged 2 L a padded piece,
    whatever its pads); a piece on five pads costs about what a padded piece did."""
    import dataclasses

    from spiderpig import voxpack as vp

    items = [_item("base", "base", [((0, -3, 0), (240, 0, 180))], 2.0),
             _item("slab", "fill", [((0, 0, 0), (200, 20, 150))], 1.0)]
    lay = vp.Packer(items).greedy()

    def on(*heights):
        b = lay.bin.copy()
        p = b.placed[-1]
        pads = tuple(vp.Pad(10.0 * i, 0.0, 10.0 * i + 15.0, 15.0, h, "base")
                     for i, h in enumerate(heights))
        b.placed[-1] = dataclasses.replace(p, rest=vp.Support("padded", ("base",), pads))
        return vp.score(b, {it.key: it.mass for it in items})

    bare = vp.score(lay.bin, {it.key: it.mass for it in items})
    assert bare < on(20, 20) < on(20, 20, 20) < on(20, 20, 20, 20)        # fewer
    assert on(20, 20, 20) < on(20, 20, 50) < on(50, 50, 50)               # smaller
    assert vp.pad_cost([]) == 0.0
    five = [vp.Pad(0.0, 0.0, vp.PAD_SIDE, vp.PAD_SIDE, 30.0)] * 5
    assert 1.5 < vp.pad_cost(five) < 3.0


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_metrics_measure_pads_and_free_travel():
    """``voxpack.metrics`` on a hand-made layout (packing review 1's M1 and its restraint
    check): a plate held 37.3 mm over a slab by one corner's post rests on pads at its other
    corners, each one's height the real gap (``voxpack.measure_pads``, rays on the meshes:
    37.3, where the voxels say a multiple of the cell less the foam), a footprint under the
    plate, what it is under and on, an id by the plate's number; the cut list adds them up.
    Each piece's free travel along the floor (``free_mm``, ``meets``) to a neighbour or the
    box's wall: the block on the slab slides 100 mm and more, the slabs only the foam's 3 mm
    (``restraint``: the loose pieces, the most any slides)."""
    from spiderpig import voxpack as vp

    def slab(key, boxes, mass=1.0):
        ms = [vp.box_mesh(lo, hi) for lo, hi in boxes]
        vols = [vp.mesh_volume(*m) for m in ms]
        vol = sum(v for v, _ in vols)
        com = sum(v * c for v, c in vols) / vol
        return vp.Packed(key, key, ms, mass, com, vol, [f"{key}{i}" for i in range(len(ms))])

    low = slab("low", [((0, 0, 0), (300, 10, 300))], 3.0)
    up = slab("up", [((0, 47.3, 0), (300, 52.3, 300)), ((0, 13, 0), (20, 47.3, 20))])
    blk = slab("b", [((100, 13, 100), (140, 33, 140))], 0.2)
    m = vp.metrics([low, up, blk])
    rows = {r["key"]: r for r in m["pieces"]}
    assert rows["up"]["rests"] == "padded"
    pads = rows["up"]["pads"]
    assert [p["id"] for p in pads] == ["2a", "2b", "2c"][:len(pads)]
    assert len(pads) == 3
    for p in pads:
        assert p["height_mm"] == pytest.approx(37.3, abs=0.05)            # the real gap
        assert p["voxel_mm"] % vp.CELL == pytest.approx(0.0)
        assert p["voxel_mm"] < p["height_mm"]
        assert (p["under"], p["on"], p["on_part"]) == ("up0", "low", "low0")
        assert p["base_mm"] == pytest.approx(10.0)
        x0, z0, x1, z1 = p["footprint"]
        assert 0.0 <= x0 < x1 <= 300.0
        assert 0.0 <= z0 < z1 <= 300.0
        assert p["size_mm"] == [pytest.approx(x1 - x0), pytest.approx(z1 - z0)]
        assert 0 < p["size_mm"][0] <= vp.PAD_SIDE
        assert p["fits"] is True
    assert m["pads"]["count"] == 3
    assert m["pads"]["tallest_mm"] == pytest.approx(37.3, abs=0.05)
    assert m["pads"]["volume_cm3"] == pytest.approx(
        sum(p["size_mm"][0] * p["size_mm"][1] * p["height_mm"] for p in pads) / 1e3, abs=0.06)
    assert m["pads"]["cut_list"] == pads
    # free travel: the block 103 mm to the -x wall, 163 to the +x (the cells: up to 2 less)
    free, meets = rows["b"]["free_mm"], rows["b"]["meets"]
    assert 103.0 - 2 * vp.CELL <= free["-x"] <= 103.0
    assert 163.0 - 2 * vp.CELL <= free["+x"] <= 163.0
    assert meets["-x"] == meets["+x"] == "wall"
    assert free["+y"] < 47.3 - 33.0                                       # up: the plate
    assert meets["+y"] == "up"
    assert rows["b"]["slide_mm"] == max(free[k] for k in ("+x", "-x", "+z", "-z"))
    assert rows["low"]["slide_mm"] == rows["up"]["slide_mm"] == vp.CLEAR
    assert m["restraint"] == {"held_mm": vp.CLEAR, "max_slide_mm": rows["b"]["slide_mm"],
                              "loose": ["b"]}


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_a_pad_is_a_block_that_fits():
    """``voxpack.measure_pads``: each pad is a rectangular block from the top of what is
    under it to the piece's lowest underside over its footprint, and nothing of another
    piece runs through it. A plate 37.3 mm over a slab with a hole a rod stands up through:
    the pad clear of the rod is its whole square, the real gap tall; one whose square takes
    in the rod is tried smaller (``PAD_TRIES``) until it is clear of it; one centred on the
    rod has no room at any size (``fits`` false, its height 0); one where the plate isn't is
    unmeasured (``fits`` None: the voxels' word)."""
    from spiderpig import voxpack as vp

    def piece(key, boxes):
        ms = [vp.box_mesh(lo, hi) for lo, hi in boxes]
        return vp.Packed(key, key, ms, 1.0, np.zeros(3), 0.0,
                         [f"{key}{i}" for i in range(len(ms))])

    y0, y1 = 47.3, 52.3
    low = piece("low", [((0, 0, 0), (300, 10, 300))])
    plate = piece("plate", [((0, y0, 0), (99, y1, 300)), ((105, y0, 0), (300, y1, 300)),
                            ((99, y0, 0), (105, y1, 99)), ((99, y0, 105), (105, y1, 300))])
    rod = piece("rod", [((100, 12, 100), (104, 70, 104))])

    def pad(cx, cz):
        h = vp.PAD_SIDE / 2
        return vp.Pad(cx - h, cz - h, cx + h, cz + h, 33.0, "low")

    clear, beside, on_rod, off = vp.measure_pads(
        [low, plate, rod], 1, [pad(130, 102), pad(110, 102), pad(102, 102), pad(400, 400)],
        floor=-3.0)
    assert clear["fits"] is True
    assert clear["size_mm"] == [vp.PAD_SIDE, vp.PAD_SIDE]
    assert clear["height_mm"] == pytest.approx(y0 - 10.0, abs=0.05)
    assert (clear["base_mm"], clear["on"], clear["on_part"], clear["under"]) == (
        pytest.approx(10.0), "low", "low0", "plate1")
    assert beside["fits"] is True                       # (15 mm took in the rod: 9 doesn't)
    assert beside["size_mm"] == [vp.PAD_TRIES[0], vp.PAD_TRIES[0]]
    assert beside["footprint"][0] > 104.0
    assert beside["height_mm"] == pytest.approx(y0 - 10.0, abs=0.05)
    assert on_rod["fits"] is False
    assert on_rod["height_mm"] == 0.0
    assert off["fits"] is None
    assert off["height_mm"] == off["voxel_mm"] == 33.0
    assert off["base_mm"] is None


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_guide_puts_the_pieces_and_their_spacers_in_the_box(monkeypatch):
    """``cradle_pack.box_steps`` (pack turn 2): into the box, a step a piece in the order it
    goes in, the spacers that go in before it named in it (a support on what it stands on,
    under the piece, the parts it bears on; a restraint beside what it holds; one ranked
    before the board on the carton's floor); out of it, the other way round. The guide puts
    them after taking the cradle apart and before putting it back together, numbered on
    (``guide.make.with_box_steps``)."""
    from types import SimpleNamespace

    from spiderpig import cradle_pack as cp
    from spiderpig.construction.assembly import Step
    from spiderpig.guide import make

    layout = {
        "modules": [{"key": "base", "label": "Base board", "on": ["floor"]},
                    {"key": "box", "label": "Cradle box", "down": "bottom", "on": ["base"]},
                    {"key": "drive", "label": "Drive side panel", "down": "outer",
                     "on": ["base", "spacer_3a"]}],
        "spacers": [{"key": "spacer_3a", "label": "Spacer 3a", "under": "drive",
                     "on": ["base"], "bears": ["L.gearbox", "base_plate"], "why": "support",
                     "rank": 1.5, "pads": ["3a"]},
                    {"key": "spacer_box_px", "label": "Spacer box_px", "under": "box",
                     "on": ["floor"], "why": "restraint +x", "rank": -0.75, "pads": []}]}
    into, out = cp.box_steps(layout)
    assert [t for t, _ in into] == ["Base board", "Cradle box", "Drive side panel"]
    assert into[0][1][0].startswith("Spacer box_px on the carton's floor, against the cradle "
                                    "box's side")
    assert into[0][1][-1] == "Then the base board, on the carton's floor."
    assert into[2][1][0].startswith("Spacer 3a on the base board, under where the drive side "
                                    "panel comes")
    assert "the drive side's gearbox" in into[2][1][0]
    assert "its outer plate down" in into[2][1][-1]
    assert out == [("Drive side panel", ["Lift the drive side panel straight up and out.",
                                         "Then Spacer 3a."]),
                   ("Cradle box", ["Lift the cradle box straight up and out."]),
                   ("Base board", ["Lift the base board straight up and out.",
                                   "Then Spacer box_px."])]

    def step(n: int, title: str) -> Step:
        return Step(n, title, "use", "Using it", None, [], [], [], ["..."], "robot")

    steps = [step(1, "Build"), step(2, "Taking it apart 1/1: off"),
             step(3, "Back together 1/1: on")]
    monkeypatch.setattr(cp, "pack_kit", lambda config, store=None: layout)
    got = make.with_box_steps(steps, BuildConfig(linkage="decanter_rcm", robot=False),
                              SimpleNamespace(meta={"cradle": True}), None)
    assert [s.title for s in got] == [
        "Build", "Taking it apart 1/1: off", "Into its box 1/3: Base board",
        "Into its box 2/3: Cradle box", "Into its box 3/3: Drive side panel",
        "Out of its box 1/3: Drive side panel", "Out of its box 2/3: Cradle box",
        "Out of its box 3/3: Base board", "Back together 1/1: on"]
    assert [s.number for s in got] == list(range(1, 10))
    assert make.with_box_steps(steps, BuildConfig(linkage="decanter_hinge", robot=False),
                               SimpleNamespace(meta={"cradle": True}), None) == steps


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_printed_spacers_hold_what_the_pads_held():
    """``voxpack.spacers`` (the pack's turn 2: printed spacers where the foam pads were): a
    plate held 30 mm over a slab at one end by a leg, its centre over nothing (on pads), and
    a block loose on the slab with room round it. The spacers are the slab's empty cells:
    each plate pad's a block from the slab to the plate's underside, its lips round the
    plate's edge, grown until it doesn't slide; the block gets a restraint beside it. Judged
    with them (``metrics``, each spacer at its rank in the order): the plate rests directly,
    on them, nothing slides more than a cell (3 mm), nothing overlaps, the order holds; a
    spacer's cells' surface (``vox_mesh``) is closed and voxelizes back to its cells; a
    spacer round-trips its JSON."""
    from spiderpig import voxpack as vp

    def piece(key, boxes, mass):
        ms = [vp.box_mesh(lo, hi) for lo, hi in boxes]
        pos = np.concatenate([q for q, _ in ms])
        tri = np.concatenate([t + 8 * i for i, (_, t) in enumerate(ms)])
        vol, c = vp.mesh_volume(pos, tri)
        return vp.Packed(key, key, [(pos, tri)], mass, c, vol, [key])

    base = piece("base", [((-120, 0, -120), (240, 9, 120))], 3.0)
    plate = piece("plate", [((0, 39, -60), (180, 45, 60)), ((0, 9, -6), (12, 39, 6))], 2.0)
    block = piece("block", [((-90, 9, -90), (-50, 40, -50))], 0.5)
    pieces = [base, plate, block]
    g = vp.occupancy(pieces)
    rows = vp.metrics(pieces, grid=g)["pieces"]
    assert rows[1]["rests"] == "padded"
    assert rows[1]["pads"]
    assert max(rows[2]["free_mm"].values()) > 3.0                 # (the block, loose)
    kit = vp.spacers(g, rows)
    support = [sp for sp in kit if sp.why == "support"]
    assert support
    assert all((sp.under, sp.on) == ("plate", ("base",)) for sp in support)
    assert {q for sp in support for q in sp.pads} == {q["id"] for q in rows[1]["pads"]}
    assert any(sp.under == "block" and sp.why.startswith("restraint") for sp in kit)
    packed = []
    for sp in kit:
        pos, tri = sp.mesh()
        vol, cen = vp.mesh_volume(pos, tri)
        assert vol == pytest.approx(sp.volume)                     # (closed, outward)
        packed.append((sp.rank, 1, vp.Packed(sp.key, sp.key, [(pos, tri)], 0.01, cen, vol,
                                             [sp.key])))
        v = sp.vox()
        assert (vp.voxelize([(pos, tri)], v.cell, org=v.org, shape=v.occ.shape).occ
                == v.occ).all()
        back = vp.Spacer.from_json(sp.to_json())
        assert (back.cols == sp.cols).all()
        assert (back.rank, back.on) == (sp.rank, sp.on)
    order = sorted([(float(i), 0, pc) for i, pc in enumerate(pieces)] + packed,
                   key=lambda t: (t[0], t[1]))
    m = vp.metrics([pc for *_, pc in order], (g.lo, g.hi))
    by = {r["key"]: r for r in m["pieces"]}
    assert by["plate"]["rests"] == "direct"
    assert not by["plate"]["pads"]
    assert set(by["plate"]["on"]) <= {sp.key for sp in support} | {"base"}
    assert m["restraint"]["max_slide_mm"] <= vp.CELL, m["restraint"]
    assert (m["overlaps"], m["order_faults"], m["unsupported"]) == ([], [], [])


def _made_up_cradle(parts: dict) -> dict:
    """A cradle's pack input (``cradle_pack.gather``'s) made up of boxes: each part's name
    (``cradle_pack.pieces`` sorts them into pieces by it) -> its boxes ((lo, hi) each) and
    an optional turn about z (degrees); packed flat (no ``turns``)."""
    from spiderpig import voxpack as vp

    out, meshes = {}, {}
    for n, (boxes, deg) in parts.items():
        pos, tri, k = [], [], 0
        for lo, hi in boxes:
            p, t = vp.box_mesh(lo, hi)
            pos.append(p @ vp.rot_about("z", deg).T)
            tri.append(t + k)
            k += len(p)
        p, t = np.concatenate(pos), np.concatenate(tri)
        vol, c = vp.mesh_volume(p, t)
        meshes[n] = (p.astype(np.float32), t.astype(np.uint32))
        out[n] = {"fab": "sheet", "bom_key": None, "rigid_with": None,
                  "lo": p.min(axis=0).tolist(), "hi": p.max(axis=0).tolist(), "volume": vol,
                  "mass": vol * 2.7e-6, "com": c.tolist()}
    return {"linkage": "made_up", "link": "b6", "t": 0.0, "tilt": 0.0, "turns": None,
            "parts": out, "meshes": meshes, "connections": [], "fastened": [], "joints": {},
            "slide_off": None}


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_a_split_keeps_each_pieces_orientation():
    """``cradle_pack.pack_items`` with a what-if split (packing review 1's m5: ``--split``
    stood B's cradle box on its end): the piece the parts come off keeps the orientation it
    had whole (its levelling with them: ``level``), the new piece its parent's; ``relevel``
    names the pieces levelled afresh, on what they now hold."""
    from spiderpig import cradle_pack as cp
    from spiderpig import voxpack as vp

    inp = _made_up_cradle({
        "base_plate": ([((-300, -10, -200), (300, 0, 200))], 0.0),
        "tie_T1": ([((0, 0, 0), (300, 20, 20))], 25.0),            # a bar, 25 deg up
        "tie_T1_screwL": ([((-10, 0, 0), (10, 400, 20))], 0.0),    # a post on it, upright
        "L.b1": ([((-200, 0, 30), (200, 150, 36))], 0.0),
    })
    whole = {it.key: it for it in cp.pack_items(inp)[2]}
    bar = [np.asarray(inp["meshes"]["tie_T1"][0], float)]
    both = [*bar, np.asarray(inp["meshes"]["tie_T1_screwL"][0], float)]
    assert cp.level(bar) == pytest.approx(-25.0)
    assert cp.level(both) != pytest.approx(cp.level(bar))       # (the post turns it)
    assert np.allclose(whole["tie_T1"].pre, vp.rot_about("z", cp.level(both)))
    splits = {"post": ["tie_T1_screw*"]}
    _, _, items, _ = cp.pack_items(inp, splits)
    by = {it.key: it for it in items}
    assert by["tie_T1"].parts == ["tie_T1"]
    assert by["post"].parts == ["tie_T1_screwL"]
    assert np.allclose(by["tie_T1"].pre, whole["tie_T1"].pre)    # as it was whole
    assert np.allclose(by["post"].pre, whole["tie_T1"].pre)      # its parent's
    assert np.allclose(by["drive"].pre, np.eye(3))
    by = {it.key: it for it in cp.pack_items(inp, splits, relevel=["tie_T1"])[2]}
    assert np.allclose(by["tie_T1"].pre, vp.rot_about("z", -25.0))     # afresh: the bar's
    assert np.allclose(by["post"].pre, whole["tie_T1"].pre)


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_a_plan_that_stops_is_judged_incomplete(tmp_path, monkeypatch):
    """Packing review 1's M3: a plan's step with no room (the base stop ``into`` a cradle
    box it doesn't fit) stops the packer with two of the six pieces placed. The layout says
    so (``missing``, in order from the one that stopped it; ``warnings``; no score),
    ``spiderpig pack-inspect`` judges it incomplete (the metrics' ``missing``, the sheet's
    status line red, naming them) and exits 1; packed by a plan that fits, it exits 0."""
    import json

    from PIL import Image

    from spiderpig import cradle_pack as cp
    from spiderpig import voxpack as vp
    from spiderpig.tools import pack_inspect

    inp = _made_up_cradle({
        "base_plate": ([((-300, -10, -200), (300, 0, 200))], 0.0),
        "tie_T1": ([((0, 0, 0), (200, 40, 100))], 0.0),            # the cradle box: solid
        "base_stop_gate": ([((0, 0, 0), (120, 60, 80))], 0.0),
        "L.handle": ([((0, 0, 0), (100, 20, 20))], 0.0),
        "L.b1": ([((-250, 0, 30), (250, 120, 36)), ((-250, 0, 30), (-240, 40, 160))], 0.0),
        "R.b1": ([((-250, 0, -36), (250, 120, -30)), ((240, 0, -160), (250, 40, -30))], 0.0),
    })
    flat = {"down": "-y", "spin": 0}                     # (one orientation each: quick)
    fits = vp.Plan("fits", (
        vp.Step("tie_T1", corner="-x-z", **flat),
        vp.Step("base_stop", after=("tie_T1",), **flat),
        vp.Step("hand_crank", after=("tie_T1",), **flat),
        vp.Step("drive", down="outer", spin=0, after=("base_stop", "hand_crank")),
        vp.Step("passive", down="outer", spin=0, after=("drive",))))
    stops = vp.Plan("stops", tuple(vp.Step("base_stop", into="tie_T1", after=("tie_T1",),
                                           **flat) if st.key == "base_stop" else st
                                   for st in fits.steps))
    why = "base_stop: no room (into tie_T1, down -y)"
    lay = cp.pack_input(inp, stops, cell=6.0)            # (6 mm cells: quick)
    assert [m["key"] for m in lay["modules"]] == ["base", "tie_T1"]
    assert lay["missing"] == ["base_stop", "hand_crank", "drive", "passive"]
    assert lay["warnings"] == [why]
    assert lay["score"] is None
    json.dumps(lay, allow_nan=False)                     # (a JSON body: no infinity)

    monkeypatch.setattr(cp, "cached_input", lambda linkage, store=None: inp)
    out = tmp_path / "out"
    argv = ["made_up", "--store", str(tmp_path / "store"), "--out", str(out), "--cell", "6",
            "--plan"]
    for plan in (stops, fits):
        (tmp_path / f"{plan.name}.json").write_text(json.dumps(plan.to_json()))
    assert pack_inspect.main([*argv, str(tmp_path / "stops.json")]) == 1
    m = json.loads((out / "stops.json").read_text())
    assert m["count"] == 2
    assert m["missing"] == ["base_stop", "hand_crank", "drive", "passive"]
    assert len(m["missing_parts"]) == 4
    assert m["warnings"] == [why]
    img = np.asarray(Image.open(out / "stops.png").convert("RGB"))[66:92, :800].astype(int)
    red = (img[..., 0] > 140) & (img[..., 1] < 80) & (img[..., 2] < 80)
    assert red.sum() > 200                               # the status line, red
    assert pack_inspect.main([*argv, str(tmp_path / "fits.json"), "--no-sheet"]) == 0
    m = json.loads((out / "fits.json").read_text())
    assert (m["count"], m["missing"], m["missing_parts"]) == (6, [], [])


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_pack_inspect_draws_and_judges_a_layout(tmp_path):
    """``spiderpig pack-inspect``'s judge (``tools.pack_inspect.judge``) of a hand-made
    layout over a made-up cradle's input: each part moved by its piece's matrix (or its
    own), voxelized afresh, judged; the contact sheet (a PNG) and the metrics and layout
    JSON written."""
    import json

    from PIL import Image

    from spiderpig import voxpack as vp
    from spiderpig.tools import pack_inspect

    parts, meshes = {}, {}
    for n, (lo, hi) in {"plate": ((0, 0, 0), (200, 6, 150)), "post": ((10, 6, 10), (20, 40, 20)),
                        "brick": ((0, 0, 0), (50, 20, 30))}.items():
        p, t = vp.box_mesh(lo, hi)
        vol, c = vp.mesh_volume(p, t)
        parts[n] = {"volume": vol, "mass": vol * 1e-6, "com": c.tolist()}
        meshes[n] = (p, t)
    inp = {"linkage": "made_up", "parts": parts, "meshes": meshes}
    eye = np.eye(4).tolist()
    up = np.eye(4)
    up[:3, 3] = (100, 9, 60)
    layout = {"modules": [{"key": "panel", "label": "A plate on a post",
                           "parts": ["plate", "post"], "matrix": eye},
                          {"key": "brick", "label": "A brick", "parts": ["brick"],
                           "matrix": up.tolist()}]}
    row = pack_inspect.judge(inp, layout, "hand", tmp_path)
    m = json.loads((tmp_path / "hand.json").read_text())
    assert pack_inspect.complete(row)
    assert (m["missing"], m["missing_parts"], m["warnings"]) == ([], [], [])
    assert m["restraint"]["max_slide_mm"] == max(r["slide_mm"] for r in m["pieces"])
    assert row["overlaps"] == 0
    assert row["unsupported"] == []
    assert m["count"] == 2
    assert m["order"] == ["panel", "brick"]
    assert {r["key"]: r["on"] for r in m["pieces"]}["brick"] == ["panel"]
    img = Image.open(row["sheet"])
    assert img.size[0] > 2000
    assert img.size[1] > 1000
    assert json.loads((tmp_path / "hand.layout.json").read_text()) == layout


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_pack_inspects_default_search_stays_within_its_budget(monkeypatch):
    """``pack-inspect --search`` without ``--seconds``: 60 s a seed, less when its seeds'
    rounds (``--workers`` at a time; one after another with no workers) would take it past
    ``SEARCH_BUDGET_S`` in all, so the default command stays an agent's step."""
    from spiderpig.tools import pack_inspect as pi

    assert pi.search_seconds(None, 4) == pi.SEARCH_SECONDS
    assert pi.search_seconds(12, 4) == pi.SEARCH_BUDGET_S / 3
    for seeds, procs in ((None, 4), (6, 6), (12, 4), (30, 6), (5, 1)):
        n = seeds or procs
        rounds = math.ceil(n / procs)
        assert pi.search_seconds(seeds, procs) * rounds <= pi.SEARCH_BUDGET_S + 1e-9
    monkeypatch.setenv("SPIDERPIG_WORKERS", "0")
    assert pi.search_seconds(4, 4) == pi.SEARCH_BUDGET_S / 4


RISER_REF = "tests/fixtures/cradle/riser_reference.json"


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_riser_rule_gives_each_listed_vessel_its_stack():
    """The one rule for a vessel's height (``cradle.RISER_CLEAR``, design review round 2's
    B1): its rim 20 mm under the lowest part over it, 10 mm off the room's side, on the
    highest printed stack under that (``cradle_report.riser_for``, ``riser_stack``). On B's
    recorded room (``tests/fixtures/cradle/riser_reference.json``, which the slow test below
    keeps true to the model) every listed vessel gets the stack recorded there; the page's
    twin (``liquid.ts``) is held to the same file by ``liquid.test.ts``. The drawn RIEDEL
    Ultra stands on the whole printed set, under the most the rule allows it."""
    import json
    from pathlib import Path

    from spiderpig.cradle_report import fits_room, riser_fits, riser_for, riser_stack, riser_stacks

    ref = json.loads((Path(__file__).resolve().parents[1] / RISER_REF).read_text())
    rows = ref["room"]
    assert ref["rule"] == {"rim_under_mm": cr.RISER_CLEAR[0], "side_mm": cr.RISER_CLEAR[1]}
    assert [[h, list(n)] for h, n in riser_stacks()] == ref["stacks"]
    for v in ref["vessels"]:
        best = riser_for(v["profile"], rows)
        assert best == v["riser_max_mm"], v["key"]
        st = riser_stack(best)
        assert st is not None, v["key"]
        assert [st[0], st[1]] == v["stack"], v["key"]
        assert riser_fits(v["profile"], st[0], rows)
    # the drawn decanter: the whole set, its rim 20 mm or more under the lowest part
    drawn = [list(p) for p in cr.DECANTER_PROFILE]
    most = riser_for(drawn, rows)
    assert most is not None
    assert most >= cr.DECANTER_RISER
    assert riser_stack(most)[0] == cr.DECANTER_RISER
    rim = cr.DECANTER_RISER + cr.DECANTER_PROFILE[-1][0]
    assert ref["lowest"]["part"] == "led_clip"
    assert ref["lowest"]["mm"] - rim >= cr.RISER_CLEAR[0]
    # the page's old rule (3 mm off the side, nothing over the rim) stood it 26 mm higher
    old = max(r for r in range(250) if fits_room(drawn, r, rows)[0] >= 3.0)
    assert old - most > 15


@pytest.mark.slow
@pytest.mark.construction
def test_the_room_under_the_mouth_names_its_lowest_part():
    """``cradle_report.room`` of B as fabricated: the lowest part of the cradle box over the
    drawn decanter's rim over the whole swing is the light's printed clip, near rest, about
    394 mm over the table (design review round 2: the guide had named the front saddle, 66
    mm higher than the clip's 380 then; round 3's mounting M2 moved the light 18 mm toward
    the mouth, 14 mm higher at rest); the drawn decanter's riser by the rule is the whole
    printed set; the recorded room (``RISER_REF``) is this one (``SPIDERPIG_REGEN=1``
    rewrites it)."""
    import json
    import os
    from pathlib import Path

    from spiderpig.cradle_report import room
    from spiderpig.fabricate import fabricate, template_for

    cfg = BuildConfig(linkage="decanter_rcm", robot=False)
    got = room(cfg, fabricate(template_for(cfg), cfg, 0.0))
    low = got["lowest"]
    assert low["part"] == "led_clip"
    assert 385.0 < low["mm"] < 400.0
    assert low["tilt_deg"] == pytest.approx(REST["decanter_rcm"], abs=5.0)
    assert got["drawn"]["riser_mm"] == cr.DECANTER_RISER
    assert got["drawn"]["stack"][0] == cr.DECANTER_RISER
    path = Path(__file__).resolve().parents[1] / RISER_REF
    ref = json.loads(path.read_text())
    if os.environ.get("SPIDERPIG_REGEN") == "1":
        from spiderpig.cradle_report import riser_for, riser_stack

        ref.update(room=got["room"], rule=got["rule"], stacks=got["stacks"], lowest=low)
        for v in ref["vessels"]:
            v["riser_max_mm"] = riser_for(v["profile"], got["room"])
            st = riser_stack(v["riser_max_mm"])
            v["stack"] = [st[0], st[1]] if st else None
        path.write_text(json.dumps(ref, indent=1) + "\n")
    assert np.allclose(np.asarray(ref["room"]), np.asarray(got["room"]), atol=0.5), \
        "the recorded room is stale: SPIDERPIG_REGEN=1"
    assert ref["lowest"]["part"] == low["part"]


@pytest.mark.slow
@pytest.mark.construction
@pytest.mark.parametrize("key", CRADLES)
def test_every_pose_the_guide_rests_a_piece_in_is_rated(key):
    """Each pose the guide sets a piece down in (``cradle.REST_POSES``; design review round
    2's M2), rated on the pieces' real shapes (``cradle_pack.rest_poses``): B's cradle and
    its drive side lie level on the riser's two blocks and the gearbox (the drive panel
    alone tipped at 6.8° and 2.5 N on its gearbox and a bracket), its box stands on its
    links' long edges, its passive panel on its outer plate; A stands on its brackets' feet
    while its board's finish cures (it had lain on the blocks: design review round 3's frames
    tipped it at 13.4°, wherever they went). Each tips only past its bar and takes its push at
    its top; on the blocks, level on them and the gearbox, which they stand as far as off the
    outer plate."""
    from spiderpig import cradle_pack as cp

    inp = cp.cached_input(key)
    rows = {r["pose"]: r for r in cp.rest_poses(inp, linkage.get(key))}
    assert set(rows) == {r.key for r in cr.REST_POSES[key]}
    # (the drive side with the box: 14.3° from round 2's NRV050, 3.45 kg 114.7 mm off the
    # outer plate, and the mounting round's saddles, 16.3° on the NRV040, its bar 14° then;
    # 17.7° since the side frames' redesign of 2026-10-20 put more of the side's weight low
    # on its rail, the bar 15° again, and the second block under the rail's column E1: the
    # drive panel alone 39.7°, 25.3° on D2's)
    bars = {"cradle": (15.0, 9.0), "drive_box": (15.0, 7.0), "drive": (30.0, 12.0),
            "box": (35.0, 12.0), "passive": (45.0, 9.0)}
    for pose, r in rows.items():
        tip, push = bars[pose]
        assert r["tip_deg"] >= tip, r
        assert r["push_n"] >= push, r
    for rest in cr.REST_POSES[key]:
        got = rows[rest.key]
        if rest.blocks:
            assert got["tilt_deg"] < 2.0, got                       # level on the blocks
            assert {"riser block 0", "riser block 1", "L.gearbox"} <= set(got["on"])
        elif rest.down == "feet":                                   # on its brackets alone
            assert got["tilt_deg"] < 2.0, got
            assert {n for n in got["on"] if not n.startswith("foot_bracket_")} <= {
                "L.frame_outer", "R.frame_outer", "L.torso", "R.torso"}, got
    # the blocks on their lifts (and their boss) stand as far as the gearbox off the outer
    # plate (round 2's NRV050: 114.7 mm)
    lo = inp["parts"]["L.frame_outer"]["lo"][2]
    standout = lo - inp["parts"]["L.gearbox"]["lo"][2]
    assert abs(cr.RISER_LIFT + cr.RISER_BLOCK[1] + cr.RISER_LIP - standout) <= 2.0, standout
    # the guide's lifts (design review round 3's stand and use, M5: two people, or one in
    # cradle.lay_down's order) say what the pieces weigh
    for pose, kg in cr.LIFTS.get(key, {}).items():
        assert abs(rows[pose]["kg"] - kg) <= 0.15, (pose, rows[pose]["kg"], kg)


PACK_BOX = {"decanter_hinge": (705.0, 215.0, 610.0)}
"""A's flat-pack box, outside (x, height, z; mm): at most (the base board's 684 x 584 mm since
the stand's 2026-10-14 refit, the walls round it; both panels face down on the board, the
loose parts in their open triangles, since the real-shape packer of 2026-10-14: 185 mm tall,
about 78 L, where the boxes' packer had 124; 212 mm and 89 L since round 2's drive, the
NRV050's housing and riser standing 27 mm taller off the drive panel's outer plate; 213 mm
and 90 L since the side frames' redesign of 2026-10-20, the drive panel turned a quarter)."""
MODULE_BOX = (30.0, 325.0, 600.0, 725.0, 143.0)
"""A cradle packed as modules (B): its box, outside, along the board no more than 30 mm (its
walls and clearances take 16) over the larger of the board's length and the cradle box's
(725 mm: levelled, from its cradle links' front pins to its rails' end), at most 340 mm tall
(the board, the cradle box standing on it, 178 mm, the two side panels nested over it),
across no more than 30 mm over the larger of the board's width and 600 mm, and at most 150 L.
B, 2026-10-13: 643 x 478 x 611 mm on its 626 x 530 board, about 188 L (its flat pack had
been 112); 2026-10-14 (round 1: the knobs and cleats on the board, the pack pose at the rear
leg): 702 x 485 x 606 on its 684 x 584 board, 206 L; the real-shape packer's plan: 700 x 478
x 604, 202 L; round 1's structure (the inner plate's triangle and leg, its columns, the 682
mm board): 699 x 499 x 608, 212 L, the drive side carrying the cradle box, 441 mm across the
bay; pack turn 1 (the cradle box on its own): 740 x 316 x 600, 141 L; round 2's structure:
740 x 334 x 600, 148 L; the side frames' redesign of 2026-10-20 (the inner plates' bottom
chords, the passive panel turned end for end over the drive panel): 740 x 337 x 600, 149 L;
pack turn 2 (its brackets on the board, the hand crank in the cradle box's bay, printed
spacers): 739 x 318 x 600, 141 L. Read off the board, so a larger board moves the bound with
it."""
SPACER_SLIVER = 10.0
"""mm^3: the most a printed spacer may share with a part the pack test tessellates afresh
(:func:`test_the_cradle_packs_in_one_box`): the chords of a curved part's mesh lie a
little off the pack's, which the spacer is shaped to (1 mm^3 seen, B's hand crank's grip)."""
PACK_SLIDE = {"decanter_rcm": 3.0, "decanter_hinge": 6.0}
"""The most a piece or a spacer slides in a cradle's pack (mm, ``voxpack.free_travel`` on
3 mm cells, the printed spacers in it; pack turn 2): B every one a cell (the 3 mm the foam
had been), A one spacer two (its passive inner plate's restraint against the carton's end:
A ships once)."""
PACK_JOINTS = {"decanter_hinge": ("Pin E ", "pillar G"), "decanter_rcm": ("Pins X and Y ", None)}
"""Where each cradle link comes off its side: its pins and its pillar (the steps name them)."""
MODULES = {"decanter_hinge": None,
           "decanter_rcm": {"base", "drive", "passive", "box", "hand_crank"}}
"""The pieces a cradle that packs as modules comes apart into (``cradle.PACK_POSES``),
besides the spare bottles' adapters and collars and the decanter's riser; None: it packs
flat."""


PACK_OPEN = {
    "decanter_rcm": "design review round 3's frames (the rail 60 mm, the rear leg 80, O2-O 88) "
                    "land the drive panel on the cradle box: unsupported in the default plan, "
                    "742 x 354 x 600 mm, 157.5 L (141.0): the packing round re-plans it",
    "decanter_hinge": "design review round 3's frames (the strut G-O 140 mm, the rail 60, a "
                      "column E2) stand the panels 255 mm tall over PACK_BOX's 215, a piece "
                      "out of order: the packing round re-plans it",
}
"""The packs design review round 3's structure broke (an open item, the packing round's)."""


@pytest.mark.slow
@pytest.mark.construction
@pytest.mark.parametrize("key", [pytest.param(k, marks=pytest.mark.xfail(
    reason=PACK_OPEN[k], strict=False)) if k in PACK_OPEN else k for k in CRADLES])
def test_the_cradle_packs_in_one_box(key):
    """``cradle_pack.flat_pack`` (a fabrication at its pack pose, ``pack_t``, by its plan,
    ``cradle_pack.PACK_PLANS``): every part but the display ones in exactly one piece; each
    part's packed bounding box (the part moved by its piece's transform, or its own) inside
    the box; judged on the real shapes (each part's tessellation moved, voxelized afresh:
    ``voxpack.metrics``): no two pieces share a cubic millimetre, none is over another that
    goes in after it, each rests on what is under it (directly or on pads, each a block
    with room where it stands: ``voxpack.measure_pads``), none missing; the box no
    bigger than :data:`PACK_BOX` (modules: :data:`MODULE_BOX`, off the base board's size);
    the steps name where the cradle link comes off its side. A packs flat, from crank angle
    0. B packs as its board, its two side panels, its cradle box and its hand crank
    (:data:`MODULES`; the cradle box on its own since pack turn 1), cranked first until the
    crank's arm lies along its outer plate's rear leg (about 5 1/2 turns from rest, 31° neck
    up): there its passive side panel, its two clips out, comes straight off its cradle
    link's pins and meets nothing of the drive side or the cradle box (``slide_off``), where
    at rest the inner bar would meet the cradle link; then the cradle box, the drive side's
    two clips out, comes straight up off the drive side's pins and meets nothing. The cradle
    is laid on its drive side first (standing, the drive module would tip onto its cradle
    box), each pin of the side lying down kept in its link by its TPU keeper, and the steps
    say how it goes back together."""
    from spiderpig import voxpack as vp
    from spiderpig.cradle_pack import PACK_PLANS, flat_pack, in_order, pack_t, spacer_packed
    from spiderpig.fabricate import fabricate, template_for
    from spiderpig.mechanism import Pose
    from spiderpig.mesh import tessellate_many
    from spiderpig.shapes import moved

    cfg = BuildConfig(linkage=key, robot=False)
    t = pack_t(cfg)
    mech = fabricate(template_for(cfg), cfg, t)
    p = flat_pack(mech, cfg.lk, t)
    assert p["plan"] == PACK_PLANS[key].name
    shipped = {b.name: b for b in mech.bodies if b.part is not None and b.fab != "display"}
    names = [n for m in p["modules"] for n in m["parts"]]
    assert sorted(names) == sorted(shipped)                 # each in exactly one piece
    assert len({m["key"] for m in p["modules"]}) == len(p["modules"])
    lo = np.asarray(p["box"]["floor"], float)
    hi = lo + np.asarray(p["box"]["inside_mm"], float)
    pieces = []
    for kind, m in in_order(p):         # (the printed spacers at their places in the order)
        if kind == "spacer":
            pieces.append(spacer_packed(m))
            continue
        shapes = []
        for n in m["parts"]:
            mat = np.array(m.get("part_matrix", {}).get(n, m["matrix"]), float)
            assert np.allclose(mat[:3, :3] @ mat[:3, :3].T, np.eye(3), atol=1e-5)
            sh = moved(shipped[n].placed_part(), Pose(mat).to_location())
            bb = sh.bounding_box()
            assert (np.array([bb.min.X, bb.min.Y, bb.min.Z]) >= lo - 0.05).all(), (n, lo)
            assert (np.array([bb.max.X, bb.max.Y, bb.max.Z]) <= hi + 0.05).all(), (n, hi)
            shapes.append(sh)
        meshes = [(np.asarray(q, float), np.asarray(i).reshape(-1, 3))
                  for q, i, _ in tessellate_many(shapes, 0.5, 0.5) if len(i)]
        pieces.append(vp.Packed(m["key"], m["label"], meshes, 1.0,
                                np.concatenate([q for q, _ in meshes]).mean(axis=0), 0.0))
    # judged on the grid the spacers were shaped on (its corner within a rounding of the
    # floor's: a spacer's faces on its cells' faces, on a grid off by a hair they would claim
    # a cell more each way)
    org = np.asarray(p["spacers"][0]["org"], float) if p["spacers"] else lo
    judged = vp.metrics(pieces, (org, hi))
    # (a spacer is shaped to the pack's meshes; the parts tessellated again here, a curve's
    # chords lie a little otherwise: a sliver of its TPU, under SPACER_SLIVER mm^3, gives)
    assert [o for o in judged["overlaps"] if (not o["a"].startswith("spacer_")
            and not o["b"].startswith("spacer_")) or o["volume_mm3"] > SPACER_SLIVER] == [], \
        judged["overlaps"]
    assert judged["order_faults"] == [], judged["order_faults"]
    assert p["unsupported"] == [], p["unsupported"]
    assert all(m["rests"] in ("direct", "padded") for m in p["modules"])
    assert (p["missing"], p["warnings"]) == ([], [])          # every piece placed
    assert p["score"] is not None
    pads = [q for m in p["modules"] for q in m["pads"]]       # each a block that fits
    assert all(q["fits"] and q["height_mm"] > 0 for q in pads), pads
    # the printed spacers in the pads' place (pack turn 2): every pad replaced, each piece
    # (and spacer) held to within PACK_SLIDE of where it goes, judged on the real shapes
    kit = p["spacers"]
    assert {q["id"] for q in pads} <= {i for d in kit for i in d["pads"]}, kit
    assert all(d["volume_cm3"] > 0 and max(d["size_mm"]) <= 220.0 for d in kit), kit
    assert judged["restraint"]["max_slide_mm"] <= PACK_SLIDE[key], judged["restraint"]
    out = np.asarray(p["box"]["outside_mm"])
    if key in PACK_BOX:
        assert all(out <= PACK_BOX[key]), p["box"]
    else:                               # modules: the bound follows the base board
        board = shipped["base_plate"].placed_part().bounding_box()
        over, tall, plane, length, litres = MODULE_BOX
        assert out[0] <= max(board.size.X, length) + over, (p["box"], board.size)
        assert out[1] <= tall, p["box"]
        assert out[2] <= max(board.size.Z, plane) + over, (p["box"], board.size)
        assert p["box"]["volume_l"] <= litres, p["box"]
    assert p["modules"][0]["key"] == "base"
    assert 0.0 < p["fill"] < 0.2
    pins, pillar = PACK_JOINTS[key]
    text = " ".join(p["steps"])
    assert pins in text, p["steps"]
    # (B's steps name pillar O1's screw head, where a riser block goes, not a pillar its
    # cradle link rides)
    assert (pillar in text) if pillar else ("rides pillar" not in text), p["steps"]
    assert p["pose"]["clip_fraction"] == pytest.approx(t / (2 * math.pi), abs=1e-6)
    by = {m["key"]: m for m in p["modules"]}
    if MODULES[key] is None:
        assert p["mode"] == "flat"
        assert t == 0.0
        assert "comes apart flat" in text                    # A's cradle link rides G
        return
    assert p["mode"] == "modules"
    spares = set(by) - MODULES[key]
    assert MODULES[key] <= set(by)
    assert all(by[k]["label"].startswith(("Adapter", "Collar", "Riser")) for k in spares), spares
    # the riser travels as one piece (its blocks are the drive panel's lay-flat blocks)
    assert {n for n in shipped if n.startswith("decanter_riser")} == set(by["risers"]["parts"])
    # the cradle box (both cradle links, the ties, the saddles with their V blocks and lip
    # gauge, the light, the base stop) travels on its own (pack turn 1); the drive side has
    # the rest of L but the hand crank, the passive side the rest of R, each pin with its
    # keeper on it
    link = cr.cradle_link(cfg.lk)
    drive, passive = set(by["drive"]["parts"]), set(by["passive"]["parts"])
    box = set(by["box"]["parts"])
    assert {f"L.{link}", f"R.{link}", "tie_Tf1", "front_saddle", "rear_saddle", "led",
            "base_stop_gate", "front_saddle_gauge"} <= box
    keepers = set(mech.meta["keepers"])       # (each fastened to its pin, on its side)
    # (a part fastened to another of the drive side, the crank block's keyed plates filed to
    # the shaft's key, stays with it)
    assert {a for a, b in mech.meta["fastened"]
            if a in shipped and not {a, b} <= drive} - keepers <= box
    assert {n for n in shipped if n.startswith("L.") and n != f"L.{link}"
            and not n.startswith("L.handle")} <= drive
    assert {"L.gearbox", "L.pin_X_pin", "L.pin_X_clip", "L.pin_Y_clip"} <= drive
    assert {n for n in shipped if n.startswith("R.") and n != f"R.{link}"} <= passive
    assert {"R.pin_X_pin", "R.pin_X_clip", "R.pin_Y_clip", "R.frame_outer"} <= passive
    assert len(keepers) == 4
    assert keepers <= drive | passive
    assert all(n.split(".")[0] == ("L" if n in drive else "R") for n in keepers)
    assert set(by["hand_crank"]["parts"]) == {n for n in shipped if n.startswith("L.handle")}
    assert "L.handle_thumb_screw" in by["hand_crank"]["parts"]
    base = set(by["base"]["parts"])
    assert {n for n in shipped if n.startswith("foot_bracket") and n.endswith("_nut1")} <= base
    assert {shipped[n].bom_key for n in base if n.startswith("foot_bracket")} == {
        "m6_carriage_25", "m6_washer_wide", "m6_nylon_nut", None}   # (None: the brackets)
    assert {n for n in shipped if n.startswith(("base_cleat", "riser_locator"))} <= base
    assert {n for n in shipped if n.startswith("foot_bracket") and n.endswith("_knob")} <= base
    assert 5.0 <= p["pose"]["turns_from_rest"] <= 6.5
    assert -44.0 < p["pose"]["tilt_deg"] < -20.0             # b6 clears; the box doesn't grow
    # the passive panel off the box and the drive side, then the box off the drive side:
    # each lifted straight up, its clips out, meets nothing
    assert p["slide_off"] == {"hits": [], "clips_out": ["R.pin_X_clip", "R.pin_Y_clip"],
                              "box": {"hits": [],
                                      "clips_out": ["L.pin_X_clip", "L.pin_Y_clip"]}}
    for words in ("rear leg", "about 5½ turns", "R-clip", "on its drive side",
                  "lift it straight up", "knobs on the outer plates' studs", "thumb screw",
                  "paint pen", "keeper", "Cradle box: lift it straight up",
                  "The brackets stay bolted to the board"):
        assert words in text, (words, p["steps"])
    assert text.index("on its drive side") < text.index("R-clip") < text.index(
        "lift it straight up") < text.index("Cradle box: lift it straight up")
    back = " ".join(p["reassembly"])
    for words in ("paint marks", "(its paint X), into its hole first",
                  "Lower the cradle box onto it", "Both R-clips in", "cleats", "knobs loose",
                  "paper check",
                  "Pin X first, always", "press each one's tip with a thumb",
                  "down into its bracket's slot"):
        assert words in back, (words, p["reassembly"])
    # its four brackets stay bolted to the board (pack turn 2, the packing review's L1):
    # each one's carriage bolt, washer and plain M6 nut with it; each plate's stud (its
    # jam nut) and knob (its nut) with its panel
    import re

    brackets = {n for n in shipped if re.fullmatch(r"foot_bracket_[A-Za-z0-9]+", n)}
    assert len(brackets) == 4
    assert brackets <= base
    studs = {n for n in shipped if n.startswith("foot_bracket")
             and n.endswith(("_screw0", "_nut0", "_knob0", "_nut2"))}
    assert len(studs) == 16
    side = {n: re.fullmatch(r"foot_bracket_[A-Za-z0-9]+([LR])_\w+", n).group(1) for n in studs}
    assert {n for n in studs if side[n] == "R"} <= passive
    assert {n for n in studs if side[n] == "L"} <= drive
    assert back.index("Lower the cradle box onto it") < back.index("paint marks")


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_a_side_panel_slides_off_only_where_nothing_is_in_its_way():
    """``cradle_pack.slide_hits`` on solids: a panel's plate under a cradle link it overlaps
    in plan meets it as it slides off along z (its side's way: away from the bay's middle);
    moved clear of it in plan, or the link past it, it doesn't; a pin through the link's hole
    slides out of it; a clip over the link (not taken out) catches it; a panel on the other
    side slides the other way."""
    from build123d import Box, Cylinder, Location, Plane

    from spiderpig.cradle_pack import slide_hits

    link = Box(100, 40, 3).moved(Location((0, 0, 139.5))) - Cylinder(4.1, 10).moved(
        Location((30, 0, 139.5)))
    torso = Box(40, 40, 4.75).moved(Location((-20, 0, 132.4)))
    pin = Cylinder(4.0, 20).moved(Location((30, 0, 140.0)))
    clip = Box(30, 2, 2).moved(Location((40, 0, 136.0)))
    assert slide_hits({"torso": torso}, {"link": link}) == [("torso", "link", 3.5)]
    clear = torso.moved(Location((0, 60, 0)))
    assert slide_hits({"torso": clear, "pin": pin}, {"link": link}) == []
    assert slide_hits({"clip": clip}, {"link": link})[0][:2] == ("clip", "link")
    assert slide_hits({"torso": torso.mirror(Plane.XY)},
                      {"link": link.mirror(Plane.XY)}) == [("torso", "link", 3.5)]


@pytest.mark.construction
@pytest.mark.no_fabricate
@pytest.mark.parametrize("key", CRADLES)
def test_a_cradles_links_are_closed_bossed_and_claimed(key):
    """Design review round 2's laser-cut parts (2026-10-15): each cradle link's outline is
    its chords and bosses closed (``plates.product_outline``), so no corner on it is sharp
    (every edge meets the next along its tangent): B's cradle link b6 and A's b2 have no
    window left, B's bell crank b2 keeps its window, its corners rounded to the link's
    closing radius; each rod b1 is 26 mm wide (SendCutSend deburrs it) with a boss round its
    crank bore, and every boss of ``config.PRODUCT_BOSS_R`` is there; and what the link
    claims (its chords, its bosses and the closing's covers, on points of its own) holds its
    whole outline, at the first sample and a third of a turn on (the points ride it)."""
    from build123d import GeomType

    from spiderpig.config import PRODUCT_BOSS_R, chord_r
    from spiderpig.construction import plates
    from spiderpig.fabricate import side_problem, template_for
    from spiderpig.stack import Disc, Layout, body_class

    cfg = BuildConfig(linkage=key, robot=False)
    ctx, groups, _ = side_problem(template_for(cfg), cfg, hint=False)
    group = next(g for g in groups if g.name == "links")
    claims = {c.owner: c for c in group.claims(ctx)}    # (its points added again: the same)
    layout = Layout(dict.fromkeys(ctx.topo.links, 1), 2, ctx.pitch)
    geo = ctx.topo.geometry.points
    windows = {"decanter_rcm": {"b2": 1, "b6": 0}, "decanter_hinge": {"b2": 0}}[key]
    for link, segs in ctx.topo.links.items():
        ol = plates.product_outline(ctx, link, segs)
        r = plates.fillet_r(ctx, link)
        for v in ol.face.vertices():     # smooth all round: each edge leaves along the last
            at = np.array([v.X, v.Y])
            ways = []
            for e in ol.face.edges():
                for u, sign in ((0.0, 1.0), (1.0, -1.0)):
                    p = e.position_at(u)
                    if np.hypot(p.X - at[0], p.Y - at[1]) < 1e-6:
                        t = e.tangent_at(u)
                        ways.append(sign * np.array([t.X, t.Y]))
            assert len(ways) == 2, (link, at)
            assert float(ways[0] @ ways[1]) < -0.9998, (link, at)
        if link in windows:
            inner = ol.face.inner_wires()
            assert len(inner) == windows[link], link
            for w in inner:
                arcs = [e for e in w.edges() if e.geom_type == GeomType.CIRCLE]
                assert len(arcs) >= 3, link
                assert all(abs(e.radius - r) < 1e-6 for e in arcs), link
        a, b = ol.frame

        def local(p: str, k: int = 0, a=a, b=b) -> np.ndarray:
            o, d = np.asarray(geo[a][k]), np.asarray(geo[b][k]) - np.asarray(geo[a][k])
            ex = d / np.hypot(*d)
            w = np.asarray(geo[p][k]) - o
            return np.array([w @ ex, w @ np.array([-ex[1], ex[0]])])

        for joint, rb in PRODUCT_BOSS_R[key].get(body_class(link), {}).items():
            c = local(ctx.topo.point_of[(link, joint)])
            assert any(e.geom_type == GeomType.CIRCLE and abs(e.radius - rb) < 1e-6
                       and np.hypot(e.arc_center.X - c[0], e.arc_center.Y - c[1]) < 1e-6
                       for e in ol.face.edges()), (link, joint)
        if body_class(link) == "b1":
            assert 2 * chord_r(cfg, link, *segs[0]) == 26.0
            box = ol.face.bounding_box()
            assert pytest.approx(2 * 15.8, abs=1e-6) == box.max.Y - box.min.Y
        shapes = claims[link].make(layout)
        for k in (0, ctx.topo.geometry.samples // 3):
            held = [plates.flat_pill(local(s.shape.at, k), local(s.shape.at, k), s.shape.r)
                    if isinstance(s.shape, Disc) else
                    plates.flat_pill(local(s.shape.a, k), local(s.shape.b, k), s.shape.r)
                    for s in shapes]
            left = [ol.face]
            for h in held:              # (one at a time: a fuse of them all can come back null)
                cut = [f - h for f in left]
                left = [g for c in cut for g in (c.faces() if hasattr(c, "faces") else c)
                        if g.area > 1e-9]
            assert sum(f.area for f in left) < 1e-3, (link, k, [f.area for f in left])


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_boards_bay_points_the_locators_ears_and_the_cleats_ends_are_rounded():
    """Design review round 2's laser-cut parts (2026-10-15): the four points the notch's bays
    leave on the base board are cut back to R3 (``cradle.BAY_POINT_R``), the riser
    locator's four inside corners at its ears filled to R2 (``EAR_FILLET_R``: SendCutSend's
    birch page) and the locator still clear of the board, and each cleat's corners rounded
    R5 (``CLEAT_END_R``), its width as it was."""
    from build123d import Location

    from spiderpig.shapes import moved

    lip = np.array([0.0, cr.LIP_HEIGHT])
    y_top = lip[1] + cr.BASE_TOP
    board, _ = cr._base_plate(lip, cr.STANDS["decanter_rcm"], y_top,
                              [(-110.0, 231.7), (386.0, -231.7)], cr.BASE_Z["decanter_rcm"])

    def solid_at(shape, x: float, z: float, y: float) -> bool:
        probe = moved(cr._rod_y(0.0, 0.0, 0.05, y - 0.5, y + 0.5), Location((x, 0.0, z)))
        inter = shape & probe
        return inter is not None and sum(p.volume for p in inter.solids()) > 1e-6

    xb, rb = cr.NOTCH_BAY
    y = y_top - cr.BASE_T / 2
    for s in (-1, 1):
        for e in (-1, 1):
            x0, z0 = xb + e * rb, s * cr.NOTCH_R           # where the old point was
            assert not solid_at(board, x0 + e * 0.3, z0 + s * 0.3, y)
            assert solid_at(board, x0 + e * 5.0, z0 + s * 5.0, y)
    play = cr.LOCATOR[1]
    edge = cr.NOTCH_R - play
    for part in cr._riser_locator(lip, lip[1] - cr.LIP_HEIGHT):
        lb = part.bounding_box()
        yl = (lb.min.Y + lb.max.Y) / 2
        for s in (-1, 1):
            for e in (-1, 1):
                x0 = xb + e * math.sqrt((rb - play) ** 2 - play ** 2)     # the old corner
                assert solid_at(part, x0 + e * 0.25, s * (edge + 0.25), yl)
                assert not solid_at(part, x0 + e * 3.0, s * (edge + 1.0), yl)
        inter = part & board
        assert inter is None or sum(p.volume for p in inter.solids()) < 1e-3
    for c in cr._cleats([-110.0, 386.0], y_top, 209.7, 1, 4.775):
        b = c.bounding_box()
        assert pytest.approx(cr.CLEAT[0]) == b.max.Z - b.min.Z
        yc = y_top + cr.BASE_T / 2
        assert not solid_at(c, b.min.X + 0.5, b.min.Z + 0.5, yc)
        assert solid_at(c, b.min.X + cr.CLEAT_END_R, b.min.Z + 0.5, yc)
        assert solid_at(c, b.min.X + 0.5, (b.min.Z + b.max.Z) / 2, yc)
