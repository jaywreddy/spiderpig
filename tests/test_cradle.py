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
        # its centre of mass between the saddles: lying level or tipped, it bears on both
        assert front.vee[1] < size.com_from_mouth < rear.vee[1], size.name
    assert cr.setting(cr.METHUSELAH) == cr.Setting(0.0, 0.0, cr.METHUSELAH.length + 0.5)
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
def test_a_base_bracket_takes_a_thumb_knob_and_the_hand_crank_a_thumb_screw():
    """The base board comes off by hand: each bracket's carriage bolt (M6 x 25) through its
    foot's oversize hole (the side settles where the linkage puts it), an 18-8 fender washer
    over the hole, an 18-8 M6 nut in a printed thumb knob, the knob clear of the bracket's
    upright and the upright's M5 screw head, a thread past the nut and the bolt's end in the
    knob; the hand crank's hub an M3 thumb screw down to the input key, its head clear of
    the hub."""
    from spiderpig.construction.handcrank import HANDLE_THUMB, HandCrank

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
    bodies = {b.name: b for b in hc._handle((0.0, 0.0), (1.0, 0.0), 0.0, "frame")}
    thumb = bodies["handle_thumb_screw"]
    assert thumb.bom_key == f"m3_thumb_screw_{HANDLE_THUMB[0]:g}"
    inter = thumb.part & bodies["handle"].part
    assert inter is None or sum(x.volume for x in inter.solids()) < 1e-3
    reach = thumb.part.bounding_box().max.Z
    assert reach > hc.box.crank_hub_r + HANDLE_THUMB[2]


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_base_board_is_rounded_offset_and_locates_its_jack():
    """The base board (round 1's stand review): B's sits 40 mm toward its drive side
    (``cradle.BASE_Z``), its corners and the notch's mouth rounded, each carriage bolt's
    hole a routed square (its neck locks by shape) with a dog-bone in each corner, two bays
    in the notch; the jack locator's two layers lie in the notch with play, their ears in
    the bays (it can't slide out forward), a square pocket for a 100 mm jack's base under
    the mouth; a rail's cleats either side of it with play, between its brackets."""
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
    layers = cr._jack_locator(lip, lip[1] - cr.LIP_HEIGHT)
    assert len(layers) == 2
    for part in layers:
        inter = part & board
        assert inter is None or sum(p.volume for p in inter.solids()) < 1e-3
        lb = part.bounding_box()
        yl = (lb.min.Y + lb.max.Y) / 2
        assert not solid_at(0.0, 0.0, part, yl)           # the pocket under the mouth
        assert solid_at(0.0, cr.JACK_BASE / 2 + cr.JACK_PLAY + 2.0, part, yl)
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
    breakdown, back = cr.module_steps(lk, ["X", "Y"])
    tags = [tag for _, tag in breakdown]
    assert tags.index("lay") < tags.index("clips") < tags.index("lift")
    assert tags.index("hand_crank") < tags.index("base")
    assert "arm down" in breakdown[0][0]
    assert "about 5½ turns" in breakdown[0][0]
    assert any("knobs loose" in s for s in back)
    assert any("paper check" in s for s in back)
    assert cr.pack_pose(linkage.get("decanter_hinge")) is None


@pytest.mark.no_fabricate
def test_a_cradles_bom_says_what_its_total_leaves_out():
    """A cradle's BOM and ORDER.md say plainly what their totals leave out: each unpriced
    cutting line with its area and, where a fitted sheet bounds it, an estimate marked as
    one with its basis (the 7075 off the 0.187 in 5052 fit, x1 to x2); the birch stays
    unpriced. A walker's BOM (no ``cut_ranges``) says nothing new."""
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

    made = [MadeRow("L.b2", "laser", "7075", "", 40.0 * 0.4826, 2, sheet="al7075_4p8mm"),
            MadeRow("base_plate", "laser", "birch", "", 3000.0 * 0.9, 1, sheet="birch_9mm"),
            MadeRow("L.frame_outer", "laser", "5052", "", 600.0 * 0.47625, 2,
                    sheet="al5052_4p8mm")]
    cutting = [CutRow("SendCutSend", "al5052_4p8mm", "5052 0.187 in", 2, 1200.0, 100.0),
               CutRow("SendCutSend", "al7075_4p8mm", "7075 0.190 in", 2, 80.0, None),
               CutRow("SendCutSend", "birch_9mm", "Baltic birch 9 mm", 1, 3000.0, None)]
    ranges = cut_ranges(cutting, made, CRADLE_CUT_PROXIES)
    base = cut_estimate("al5052_4p8mm", [(40.0, 2)], CRADLE_CUT_FITS["al5052_4p8mm"])
    assert ranges["al7075_4p8mm"][:2] == pytest.approx((base, 2 * base), abs=0.02)
    assert ranges["birch_9mm"] is None
    bom = Bom([], made, cutting=cutting, cut_source=CRADLE_CUT_SOURCE, cut_ranges=ranges)
    md = bom.markdown()
    assert "The total excludes 2 unpriced cut lines" in md
    assert "an estimate of USD" in md
    assert "Baltic birch 9 mm (1 part, 3000 cm²: no fitted" in md
    assert "cradle A's files" in CRADLE_CUT_SOURCE
    assert "359-368" not in CRADLE_CUT_SOURCE
    assert bom.as_dict()["unpriced_cutting"][1]["estimate_usd"] is None
    order = order_markdown(bom, [], [])
    assert "The total excludes 2 unpriced cut lines" in order
    walker = Bom([], made, cutting=cutting)
    assert "unpriced cut line" not in walker.markdown()
    assert "unpriced_cutting" not in walker.as_dict()


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
    """Both sides, the box, the stand on its base plate, the bottle (a 6 L methuselah, the V
    blocks at their lowest) and the decanter at rest, at full tilt and across the turn: no
    two parts meet, the hand crank's sweep meets nothing, and every bottle size at its own
    setting (its V blocks raised, its base stop moved) meets nothing, nor do they."""
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
        for size in cr.SIZES:
            assert cr.bottle_hits(mech, size) == [], (t, size.key)
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
    jam = [x["jam_sf"] for x in rows if x["jam_sf"] is not None]
    assert min(jam) >= 1.3, [(x["joint"], x["jam_sf"]) for x in rows]
    # the quick-release clevis pins at X and Y (2026-10-13): stronger than the M6 Chicago
    # screws they replaced (1.72 and 1.49 jammed)
    quick = {x["joint"]: x for x in rows if x["joint"] in ("pin:X", "pin:Y")}
    assert set(quick) == {"pin:X", "pin:Y"}
    for x in quick.values():
        assert "clevis" in x["section"], x
        assert x["jam_sf"] >= 2.0, x
        assert x["walk_sf"] >= 6.0, x


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
    (use,) = [s for s in steps if "bottle" in s.adds]
    assert use.stage == USE
    assert set(use.adds) == {"bottle", "decanter", "decanter_riser"}
    assert all(not s.adds for s in steps[steps.index(use) + 1:])
    assert steps.index(passive) > steps.index(ties)
    # the board's cleats with their side's brackets, the jack locator with the board
    adds = {a for s in steps for a in s.adds}
    assert {"jack_locator_0", "jack_locator_1"} <= set(base.adds)
    assert {a for a in adds if a.startswith("base_cleat_L")} <= set(base.adds)
    (base_r,) = [s for s in steps if any(a.startswith("base_cleat_R") for a in s.adds)]
    assert steps.index(base_r) > steps.index(base)
    assert "knobs last" in " ".join(base_r.text)
    titles = [s.title for s in steps]
    packs = key in cr.PACK_POSES
    assert ("Taking it apart for the trip" in titles) == packs
    assert ("Putting it back together" in titles) == packs
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
    for name in ("front_saddle", "rear_saddle", "front_saddle_vee", "rear_saddle_vee",
                 "front_saddle_liner", "front_saddle_knob", "front_saddle_gauge",
                 "base_stop_rails", "base_stop_gate", "L.handle", "foot_boot_0"):
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
    assert got["standard"] == pytest.approx([12.73, 68.87, 300.5], abs=0.01)
    assert set(m["adjust"]) == {"front", "rear", "stop"}
    assert [s["moves"] for s in m["saddles"]] == ["front", "rear"]


@pytest.mark.no_fabricate
def test_the_saddles_hold_a_full_6_l():
    """``cradle_report.saddles``: every printed part of the saddles, the lip gauge and the base
    stop under its allowable at a full 6 L's worst (hung on its thumbscrew, leaning on the
    gate at rest with a corkscrew's push, on the gauge at full tilt with no friction), the
    post and the thumbscrew 3 x under their yield, the gate's lock gripping 3 x its load."""
    from spiderpig.cradle_report import PLA_ALLOW, saddles

    got = saddles(-50.0, 30.0, 0.75)
    rows = {r["part"]: r for r in got["rows"]}
    for r in got["rows"]:
        if r.get("limit_mpa") == PLA_ALLOW:
            assert r["stress_mpa"] <= PLA_ALLOW, r
        else:
            assert r["sf"] >= 3.0, r
    assert rows["rear post (NETRF12, bending)"]["load_n"] > 10.0     # (N·m: the couple)
    assert rows["base stop's lock (its grip)"]["backstop_mm"] < 10.0


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


PACK_BOX = {"decanter_hinge": (705.0, 305.0, 610.0)}
"""A's flat-pack box, outside (x, height, z; mm): at most (the base board's 684 x 584 mm since
the stand's 2026-10-14 refit, the walls round it)."""
MODULE_BOX = (30.0, 500.0, 600.0)
"""A cradle packed as modules (B): its box, outside, over its base board's length and width
(x, z: the board lies at the bottom, the other pieces over it) at most 30 mm (its walls and
clearances take 16), at most 500 mm tall (the drive side with the cradle box lies flat, 442
mm, on the 28 mm board and its bolts), and across no more than 30 mm over the larger of the
board's width and 600 mm (the drive side's height in its plane at the pack pose, 596).
B, 2026-10-13: 643 x 478 x 611 mm on its 626 x 530 board, about 188 L (its flat pack had
been 112); 2026-10-14 (round 1: the knobs and cleats on the board, the pack pose at the rear
leg): 702 x 485 x 606 on its 684 x 584 board, 206 L. Read off the board, so a larger board
moves the bound with it."""
PACK_JOINTS = {"decanter_hinge": ("Pin E ", "pillar G"), "decanter_rcm": ("Pins X and Y ", None)}
"""Where each cradle link comes off its side: its pins and its pillar (the steps name them)."""
MODULES = {"decanter_hinge": None,
           "decanter_rcm": {"base", "drive", "passive", "hand_crank"}}
"""The pieces a cradle that packs as modules comes apart into (``cradle.PACK_POSES``),
besides the spare bottles' adapters and collars; None: it packs flat."""


@pytest.mark.slow
@pytest.mark.construction
@pytest.mark.parametrize("key", CRADLES)
def test_the_cradle_packs_in_one_box(key):
    """``cradle_pack.flat_pack`` (a fabrication at its pack pose, ``pack_t``): every part but
    the display ones in exactly one piece; each part's packed bounding box (the part moved by
    its piece's transform, or its own) inside the box and clear of every other piece's (a
    levelled piece turns by any angle, so its parts' boxes are the moved parts', not their
    assembled boxes' corners moved); of two pieces over each other the lower one goes in
    first; the box no bigger than :data:`PACK_BOX` (modules: :data:`MODULE_BOX`, off the base
    board's size); the steps name where the cradle link comes
    off its side. A packs flat, from crank angle 0. B packs as three modules and its hand crank
    (:data:`MODULES`, the spare adapters loose), cranked first until the crank's arm lies
    along its outer plate's rear leg (about 5 1/2 turns from rest, 31° neck up): there its
    passive side panel, its two clips out, comes straight off its cradle link's pins and
    meets nothing of the drive side or the cradle box (``slide_off``), where at rest the
    inner bar would meet the cradle link; the cradle is laid on its drive side first and the
    panel lifted off (standing, the drive module would tip onto its cradle box; on its
    passive side, the pins would drop out of their links), and the steps say how it goes back
    together."""
    from spiderpig.cradle_pack import flat_pack, pack_t
    from spiderpig.fabricate import fabricate, template_for
    from spiderpig.mechanism import Pose
    from spiderpig.shapes import moved

    cfg = BuildConfig(linkage=key, robot=False)
    t = pack_t(cfg)
    mech = fabricate(template_for(cfg), cfg, t)
    p = flat_pack(mech, cfg.lk, t)
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
    out = np.asarray(p["box"]["outside_mm"])
    if key in PACK_BOX:
        assert all(out <= PACK_BOX[key]), p["box"]
    else:                               # modules: the bound follows the base board
        board = shipped["base_plate"].placed_part().bounding_box()
        over, tall, plane = MODULE_BOX
        assert out[0] <= board.size.X + over, (p["box"], board.size)
        assert out[1] <= tall, p["box"]
        assert out[2] <= max(board.size.Z, plane) + over, (p["box"], board.size)
    assert p["modules"][0]["key"] == "base"
    pins, pillar = PACK_JOINTS[key]
    text = " ".join(p["steps"])
    assert pins in text, p["steps"]
    assert (pillar in text) if pillar else ("pillar" not in text), p["steps"]
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
    assert all(by[k]["label"].startswith(("Adapter", "Collar")) for k in spares), spares
    # the cradle box (both cradle links, the ties, the saddles with their V blocks and lip
    # gauge, the light, the base stop) travels on the drive side; the passive side has the
    # rest of R
    link = cr.cradle_link(cfg.lk)
    drive, passive = set(by["drive"]["parts"]), set(by["passive"]["parts"])
    assert {"L.gearbox", f"L.{link}", f"R.{link}", "tie_Tf1", "front_saddle", "rear_saddle",
            "led", "base_stop_gate", "front_saddle_gauge"} <= drive
    assert {a for a, _ in mech.meta["fastened"] if a in shipped} <= drive
    assert {n for n in shipped if n.startswith("R.") and n != f"R.{link}"} <= passive
    assert {"R.pin_X_pin", "R.pin_X_clip", "R.pin_Y_clip", "R.frame_outer"} <= passive
    assert set(by["hand_crank"]["parts"]) == {n for n in shipped if n.startswith("L.handle")}
    assert "L.handle_thumb_screw" in by["hand_crank"]["parts"]
    base = set(by["base"]["parts"])
    assert {n for n in shipped if n.startswith("foot_bracket") and n.endswith("_nut1")} <= base
    assert {shipped[n].bom_key for n in base if n.startswith("foot_bracket")} == {
        "m6_carriage_25", "m6_washer_wide", "m6_nut", None}         # (None: the knobs)
    assert {n for n in shipped if n.startswith(("base_cleat", "jack_locator"))} <= base
    assert {n for n in shipped if n.endswith("_knob")} <= base
    assert 5.0 <= p["pose"]["turns_from_rest"] <= 6.5
    assert -44.0 < p["pose"]["tilt_deg"] < -20.0             # b6 clears; the box doesn't grow
    assert p["slide_off"] == {"hits": [], "clips_out": ["R.pin_X_clip", "R.pin_Y_clip"]}
    for words in ("rear leg", "about 5½ turns", "R-clip", "on its drive side",
                  "lift it straight up", "thumb knobs", "thumb screw", "paint pen"):
        assert words in text, (words, p["steps"])
    assert text.index("on its drive side") < text.index("R-clip") < text.index(
        "lift it straight up")
    back = " ".join(p["reassembly"])
    for words in ("paint marks", "pin X through its spacer into its hole first",
                  "Both R-clips in", "cleats", "knobs loose", "paper check"):
        assert words in back, (words, p["reassembly"])


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
