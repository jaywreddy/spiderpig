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
        # its centre of mass (the shell-and-wine model, design review round 1) behind the
        # front V, 28-40 % of its length up from its base, and on the rear support (a 6 L
        # methuselah's, 390 mm from the mouth, lies 10 mm behind the rear V's end, over the
        # base stop's rails' flanges: the mounting review's B1)
        assert front.vee[1] < size.com_from_mouth, size.name
        assert 0.6 < size.com_from_mouth / size.length < 0.72, size.name
        assert size.com_from_mouth < min(cr.FLANGE_END, size.length), size.name
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
    """The hand crank's slip clutch (round 1's drive review: a 40 N push was no bound): its
    arm turns on the hub's journal, three steel balls on springs and set screws in its boss
    seated in the flange's conical seats; its parts meet nowhere, each ball sits in a seat,
    the springs stay off solid as set, the set screws inside the boss; the cap it puts on the
    crank is ``config.PRODUCT_CLUTCH_NM``'s; the cap and the journal's thumb screw keep the arm
    on, so it comes off the input shaft whole; the grip is 60 mm long on a bolt's plain
    shank."""
    from spiderpig.config import PRODUCT_CLUTCH_NM
    from spiderpig.construction.handcrank import (
        CLUTCH,
        CLUTCH_SPRING,
        GRIP,
        NRV040,
        REACH,
        HandCrank,
        clutch,
        handle_r,
    )

    for key in CRADLES:
        cfg = BuildConfig(linkage=key, robot=False)
        got = clutch(handle_r(cfg), NRV040.ratio, NRV040.eta_hand)
        assert PRODUCT_CLUTCH_NM[key] == pytest.approx(got["crank_cap_nm"], abs=0.05)
        assert got["slip_lo_nm"] < got["slip_nm"] < got["slip_hi_nm"]
        # each spring as set: under its solid with room for the ball to climb out of its seat
        _, free, rate, max_len, _, _ = CLUTCH_SPRING
        assert free - got["spring_n"] / rate > max_len           # (at the top of its climb)
    hc = HandCrank()
    parts, sweep = hc._handle((0.0, 0.0), (1.0, 0.0), 0.0, "frame", 100.0)
    by = {b.name: b for b in parts}
    for a, b in itertools.combinations(parts, 2):
        inter = a.part & b.part
        assert inter is None or sum(x.volume for x in inter.solids()) < 0.05, (a.name, b.name)
    balls = [by[f"handle_ball{i}"] for i in range(CLUTCH["balls"])]
    flange = (REACH - CLUTCH["boss_t"] - CLUTCH["gap"])          # the seats' face (x)
    for ball in balls:
        bb = ball.part.bounding_box()
        assert flange - 1.0 > bb.min.X                             # down in its seat
        assert math.hypot(bb.center().Y, bb.center().Z) == pytest.approx(CLUTCH["r"], abs=0.01)
    for i in range(CLUTCH["balls"]):
        assert by[f"handle_set_screw{i}"].part.bounding_box().max.X < REACH
    grip = by["handle_grip"].part.bounding_box()
    assert pytest.approx(GRIP[1], abs=0.01) == grip.size.X
    assert by["handle_grip_pin"].bom_key.startswith("m8_hex_bolt_")
    assert by["handle_grip_nut"].bom_key == "m8_nyloc_nut"
    # the arm is held on the journal by the cap and its thumb screw (the crank comes off
    # whole), and everything that turns is inside the sweep
    assert by["handle_cap"].part.bounding_box().min.X >= REACH
    assert max(s[2] for s in sweep) >= max(b.part.bounding_box().max.X for b in parts) - 1e-6
    assert max(s[0] for s in sweep) >= 100.0 + GRIP[0] / 2


@pytest.mark.construction
@pytest.mark.no_fabricate
def test_the_gearbox_stands_on_spacers_and_the_crank_block_on_shims():
    """The riser (round 1's drive review, B1): four round spacers on the face screws, on the
    gearbox's machined face, the plate over the spigot ring and the turning hub's end by
    ``RISER_CLEAR`` whatever their unmeasured heights; the crank block's top plate seated on
    the shaft's steel end over DIN 988 shims (M2: no printed sleeve in the clamp), the drawn
    shim leaving room either way; the keyed plates on the key's straight flanks for any shim
    the guide may call for; the shims bought fill any gap to it within a ring."""
    from spiderpig.construction.handcrank import (
        KEYED_PLATES,
        NRV040,
        RISER_CLEAR,
        SHIM,
        SPACER,
        arm_z,
        riser_for,
        shims_for,
    )

    t, t_frame = 3.175, 4.75
    a0, a1 = arm_z(0.0, t_frame, KEYED_PLATES * t)
    r = riser_for(0.0, a0, a1, t, NRV040, t_frame)
    assert r.spacer in SPACER[2]
    assert r.face == pytest.approx(-r.spacer)
    assert r.spacer - max(NRV040.spigot_h, NRV040.hub_out) >= RISER_CLEAR
    assert NRV040.pcd / 2 - SPACER[0] / 2 >= NRV040.spigot_d / 2 + 1.0    # off the spigot
    assert 0.3 <= r.shim <= r.shim_max - 0.3
    k = (r.seat - a0) / t
    assert k == pytest.approx(round(k))
    assert round(k) > KEYED_PLATES
    key_w, _, key_len = NRV040.shaft_key
    for shim in (0.0, r.shim_max):
        end = r.seat - shim
        assert end - NRV040.key_short - key_w / 2 >= a1 - 0.2 - 1e-9
        assert end - NRV040.key_short - key_len + key_w / 2 <= a0
    assert r.shim_max <= SHIM[0] * SHIM[1]
    # the rings to the one below the gap: the block a ring at most low, never high
    t_shim = SHIM[0]
    for gap in (0.0, 0.19, 0.2, r.shim, r.shim_max):
        got = sum(shims_for(gap))
        assert gap - t_shim < got <= gap + 1e-9


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
    setting (its V blocks raised, its base stop moved) meets nothing, nor do they; the base
    stop never comes nearer the base board than a finger's 25 mm (ISO 13854: at rest, set
    for a 6 L, its lowest)."""
    from spiderpig.construction.contract import bad_solids, clashes
    from spiderpig.cradle_report import wheel_sweep
    from spiderpig.fabricate import fabricate, template_for

    cfg = BuildConfig(linkage=key, robot=False)
    tmpl = template_for(cfg)
    ts = np.linspace(0, 2 * math.pi, 721)
    tilt = _tilt(key, ts)
    # (and where A's 105 mm hand crank met its cradle link b2 and tie Tr2's screw, design
    # review round 1's drive: about 165-195° and 330-345° of crank)
    angles = [ts[int(np.argmin(tilt))], ts[int(np.argmax(tilt))], 1.0, 3.5, 5.2,
              *np.radians([165.0, 180.0, 345.0])]
    for t in angles:
        mech = fabricate(tmpl, cfg, float(t))
        assert clashes(mech) == [], t
        assert wheel_sweep(mech) == [], t
        assert cr.finger_gap(mech) >= cr.FINGER_GAP, t
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
    under MIL-STD-1472H's light-load crank's 22 N, the worm holds within its rating at a
    hand's speed, the rod, the crankpin and every pin and plate hold a sealed full 6 L bottle
    3 x over and a firm 40 N push at the grip on a blocked cradle up to the clutch's cap (the
    jam: ``cradle_report.jam_nm``, ``config.PRODUCT_JAM_NM``)."""
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
    # every joint, link, column and plate holds the jam (rated from the force vectors:
    # the links chord by chord, the pillar and the columns on their box)
    jam = [x["jam_sf"] for x in rows if x["jam_sf"] is not None]
    assert min(jam) >= 1.3, [(x["joint"], x["jam_sf"]) for x in rows]
    _the_structure_is_rated(r, ("b2",), {"pillar:G"},
                            {"column:C1", "column:C2", "column:D1", "column:D2"})
    _holds_the_clutchs_cap(r)
    _the_drive_keeps_its_bars(r)       # (last: its grip misses the bar, open)


def _the_drive_keeps_its_bars(r: dict) -> None:
    """A report's drive against its cited bars (design review round 1's drive, 2026-10-14):
    the NRV040 at a hand's speed (Motovario's 30:1 rating at its lowest listed input speed,
    500 rpm, scaled to Phidgets' 45 N·m: ``handcrank.NRV040``) 1.3 x over the walking peak,
    and within the 45 N·m row itself; the grip under MIL-STD-1472H's light-load crank's 22 N
    (Figure 13: ``handcrank.LIGHT_LOAD_N``) at a throw within its 125 mm."""
    from spiderpig.construction.handcrank import LIGHT_LOAD_N

    drive = r["drive"]
    assert drive["gearbox_sf_hand"] >= 1.3, (drive["gearbox_sf_hand"], drive["gearbox_sf"])
    assert drive["gearbox_sf"] >= 1.0, drive["gearbox_sf"]
    assert drive["handle_force_n"] < LIGHT_LOAD_N, (drive["handle_force_n"],
                                                     drive["grip_radius_mm"])
    assert drive["grip_radius_mm"] <= 125.0


def _holds_the_clutchs_cap(r: dict) -> None:
    """A report's drive at its clutch (``cradle_drive``): its least slip clears the worst
    starting turn by the margin; its cap is the jam case's torque (however hard the push),
    within the gearbox's momentary limit (2 x its rating); the grip's force at the clutch's
    top within the input shaft's radial load there; every part holds the cap at SF 1.3 or
    more 30° and more from the toggles (where the old 40 N push's case skipped, every part
    is now rated: the band round a toggle where any falls under 1.3 is reported, no wider);
    the gearbox's and the keyed plates' rows at the jams."""
    clutch = r["drive"]["clutch"]
    assert clutch["margin"] >= clutch["margin_floor"]
    assert r["drive"]["jam_torque_nm"] == pytest.approx(clutch["crank_cap_nm"], abs=0.01)
    assert r["drive"]["gearbox_jam_x_peak_limit"] <= 1.0
    assert r["drive"]["input_overhung"]["sf"] >= 2.0
    capped = r["jam_capped"]
    assert capped["crank_nm"] == clutch["crank_cap_nm"]
    assert capped["away"][0] >= 1.3, capped["away"]
    holds = capped["holds_1p3_outside_deg"]
    bands = [(x["band_deg"], x["worst"]) for x in capped["by_band"]]
    assert holds is not None, bands                 # (5° steps: where the old case
    assert holds <= 30.0, bands                     # skipped, now rated at a bound)
    assert capped["by_band"][0]["band_deg"] == 0.0              # the whole turn rated
    assert r["drive"]["gearbox_jam_x_rating"] > 1.0             # (reported, not a bar)
    assert r["drive"]["keyway"]["four_plates_sf_cap"] >= 2.0
    assert "This report:" in r["verdict"][1]


@pytest.mark.slow
@pytest.mark.strength
def test_b_stands_turns_easily_and_holds():
    """B, the virtual hinge (``docs/agentlib/DECANTER.md``): the mouth still for both
    bottles, it stands at 25° and takes a 25 N push at the bottle's top (the worst over the
    swing), the hand turns it with under MIL-STD-1472H's light-load crank's 22 N, the worm
    holds within its rating at a hand's speed, the rod and the crankpin hold the jam, every
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
    jam = [x["jam_sf"] for x in rows if x["jam_sf"] is not None]
    assert min(jam) >= 1.3, [(x["joint"], x["jam_sf"]) for x in rows]
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
    _the_structure_is_rated(r, ("b2", "b6"), {"pillar:O1", "pillar:O2"},
                            {"column:D1", "column:D2"})
    # the known limits (not designed for): a block near the mouth multiplies the pins' loads
    front = r["blocked"]["cases"]["front saddle"]
    assert front["joints_n"]["J"] > 3.0 * max(x["jam_n"] or 0.0 for x in rows
                                              if x["joint"] == "pin:J")
    _holds_the_clutchs_cap(r)
    _the_drive_keeps_its_bars(r)


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
                 "rear_saddle_liner", "rear_saddle_knob", "rear_saddle_brake",
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
    assert got["standard"] == pytest.approx([12.73, 68.87, 300.5], abs=0.01)
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


@pytest.mark.no_fabricate
def test_the_saddles_hold_a_full_6_l():
    """``cradle_report.saddles``: every printed part of the saddles, the lip gauge and the base
    stop under its allowable at a full 6 L's worst (hung on its thumbscrew, leaning on the
    gate at rest with a corkscrew's push, on the gauge at full tilt with no friction, its
    body on the rails' flanges), the post and the screws 3 x under their yield, the gate's
    lock gripping and the lead screw's brake holding 3 x what they hold."""
    from spiderpig.cradle_report import PLA_ALLOW, saddles

    got = saddles(-50.0, 30.0, 0.75)
    rows = {r["part"]: r for r in got["rows"]}
    for r in got["rows"]:
        if r.get("limit_mpa") == PLA_ALLOW:
            assert r["stress_mpa"] <= PLA_ALLOW, r
        else:
            assert r["sf"] >= 3.0, r
    assert rows["rear post (NETRF12, bending)"]["load_n"] > 10.0     # (N·m: the couple)
    assert rows["rear post (NETRF12, bending at rest)"]["load_n"] > 10.0
    for part in ("rear lead screw's brake (no turning back)", "base stop's lock (its grip)",
                 "base stop's rails (bending at their sockets)", "lip gauge's post (bent and "
                 "twisted)", "rear knob's tower (bending)", "base stop's T-slot lips (bending)"):
        assert part in rows, part
    assert got["balance"]["ok"]
    assert got["gauge"]["falls_back"]


@pytest.mark.no_fabricate
def test_every_bottle_rests_on_both_saddles_at_rest_level_and_tipped():
    """``cradle_report.balance`` (review round 1, B1): with its centre of mass where the shell
    model puts it (a 6 L methuselah's 390 mm from the mouth), the gate's push under the axis
    at rest and the gauge's over it at full tilt, each bottle's reaction centres inside its
    support, the front V to the rear support's end (the rear V, then the rails' flanges), by
    40 mm or more; without the flanges (the rear V's end at 380) the 6 L would lift its neck
    off the front V at rest. The lip gauge, lifted to its stop, falls back shut."""
    from spiderpig.cradle_report import balance

    assert cr.METHUSELAH.com_from_mouth == 390.0
    got = balance(-50.0, 30.0)
    assert got["ok"]
    assert got["worst_margin_mm"] >= 40.0
    assert {r["case"] for r in got["rows"]} == {"rest", "level", "full"}
    assert len(got["rows"]) == 3 * len(cr.SIZES)
    rest = {r["bottle"]: r for r in got["rows"] if r["case"] == "rest"}
    assert rest[cr.METHUSELAH.name]["centre_mm"] > cr.SADDLES[1].vee[1]     # (on the flanges)
    g = cr.gauge_return()
    assert g["falls_back"]
    assert g["com_deg_open"] < 85.0
    assert abs(g["stop_on_wall_mm"]) < 0.05


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
    assert got.pads
    assert max(h for _, _, h in got.pads) == pytest.approx(15.0)
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
    than where it started; ``voxpack.search`` packs its best again."""
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
    assert row["overlaps"] == 0
    assert row["unsupported"] == []
    assert m["count"] == 2
    assert m["order"] == ["panel", "brick"]
    assert {r["key"]: r["on"] for r in m["pieces"]}["brick"] == ["panel"]
    img = Image.open(row["sheet"])
    assert img.size[0] > 2000
    assert img.size[1] > 1000
    assert json.loads((tmp_path / "hand.layout.json").read_text()) == layout


PACK_BOX = {"decanter_hinge": (705.0, 200.0, 610.0)}
"""A's flat-pack box, outside (x, height, z; mm): at most (the base board's 684 x 584 mm since
the stand's 2026-10-14 refit, the walls round it; 185 mm tall since the real-shape packer of
2026-10-14: both panels face down on the board, the loose parts in their open triangles,
about 78 L where the boxes' packer had 124)."""
MODULE_BOX = (30.0, 500.0, 600.0)
"""A cradle packed as modules (B): its box, outside, over its base board's length and width
(x, z: the board lies at the bottom, the other pieces over it) at most 30 mm (its walls and
clearances take 16), at most 500 mm tall (the drive side with the cradle box lies on its
drive side, the gearbox down, 441 mm across the bay, over the passive panel on the board),
and across no more than 30 mm over the larger of the board's width and 600 mm (the drive
side's height in its plane at the pack pose, 596). B, 2026-10-13: 643 x 478 x 611 mm on its
626 x 530 board, about 188 L (its flat pack had been 112); 2026-10-14 (round 1: the knobs
and cleats on the board, the pack pose at the rear leg): 702 x 485 x 606 on its 684 x 584
board, 206 L; the real-shape packer's plan: 700 x 478 x 604, 202 L; round 1's structure (the
inner plate's triangle and leg, its columns, the 680 mm board): 696 x 499 x 608, 211 L. Read
off the board, so a larger board moves the bound with it."""
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
    """``cradle_pack.flat_pack`` (a fabrication at its pack pose, ``pack_t``, by its plan,
    ``cradle_pack.PACK_PLANS``): every part but the display ones in exactly one piece; each
    part's packed bounding box (the part moved by its piece's transform, or its own) inside
    the box; judged on the real shapes (each part's tessellation moved, voxelized afresh:
    ``voxpack.metrics``): no two pieces share a cubic millimetre, none is over another that
    goes in after it, each rests on what is under it (directly or on foam pads); the box no
    bigger than :data:`PACK_BOX` (modules: :data:`MODULE_BOX`, off the base board's size);
    the steps name where the cradle link comes off its side. A packs flat, from crank angle
    0. B packs as three modules and its hand crank (:data:`MODULES`, the spare adapters
    loose), cranked first until the crank's arm lies along its outer plate's rear leg (about
    5 1/2 turns from rest, 31° neck up): there its passive side panel, its two clips out,
    comes straight off its cradle link's pins and meets nothing of the drive side or the
    cradle box (``slide_off``), where at rest the inner bar would meet the cradle link; the
    cradle is laid on its drive side first and the panel lifted off (standing, the drive
    module would tip onto its cradle box; on its passive side, the pins would drop out of
    their links), and the steps say how it goes back together."""
    from spiderpig import voxpack as vp
    from spiderpig.cradle_pack import PACK_PLANS, flat_pack, pack_t
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
    for m in p["modules"]:
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
    judged = vp.metrics(pieces, (lo, hi))
    assert judged["overlaps"] == [], judged["overlaps"]
    assert judged["order_faults"] == [], judged["order_faults"]
    assert p["unsupported"] == [], p["unsupported"]
    assert all(m["rests"] in ("direct", "padded") for m in p["modules"])
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
    assert 0.0 < p["fill"] < 0.2
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
    assert {n for n in shipped if n.startswith("foot_bracket") and n.endswith("_knob")} <= base
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
