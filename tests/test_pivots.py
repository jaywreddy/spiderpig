"""Tests for the metal-shaft pivots (:mod:`construction.pivots`): rod, bolt, bearing, bushing.

Each is checked as both pillar and pin on the Klann single (one fabrication
per construction, shared by the tests) and on the Jansen single, plus the
Klann quad's plan; mixed builds; and the ways they refuse a design.
"""

from __future__ import annotations

import itertools
from dataclasses import replace

import pytest

import construction
from construction import ConstructionError
from construction.axle import AxleGroup, flange_sides
from construction.base import FRAME_INNER, FRAME_OUTER, Build, Params
from construction.contract import check_side
from construction.pivots import BEARING, BUSHING, BoltAxle, InsertAxle, RodAxle
from construction.pivots.common import Column
from fabricate import BuildConfig, design_side, fabricate, fabricate_side, template_for
from hardware.bom import bom_from_mechanism
from hardware.catalog import get
from stack import Layout, Unbuildable, verify_plan

KEYS = ("rod", "bolt", "bearing", "bushing")
T = 1.0


def _config(key: str, **kw) -> BuildConfig:
    return BuildConfig(module="single", robot=False, pin=key, pillar=key, **kw)


def _vol(a, b) -> float:
    inter = a & b
    return 0.0 if inter is None else sum(s.volume for s in inter.solids())


def _clashes(mech, allowed=()) -> list[tuple[str, str, float]]:
    allowed = {frozenset(p) for p in allowed}
    parts = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    out = []
    for a, b in itertools.combinations(parts, 2):
        if frozenset((a, b)) in allowed:
            continue
        vol = _vol(parts[a], parts[b])
        if vol > 1e-3:
            out.append((a, b, round(vol, 4)))
    return out


@pytest.fixture(scope="module", params=KEYS)
def side(request):
    """The Klann single, one side, ``key`` for both pillars and pins."""
    key = request.param
    cfg = _config(key)
    tmpl = template_for(cfg)
    design = design_side(tmpl, cfg)
    mech = tmpl.freeze_at(T)
    build = Build(design.ctx, design.plan, mech)
    fab = fabricate_side(design, mech)
    axles = [g for g in design.groups if isinstance(g, AxleGroup)]
    return key, tmpl, design, build, fab, axles


@pytest.fixture(scope="module")
def jansen():
    out = {}
    for key in KEYS:
        cfg = _config(key, linkage="jansen")
        tmpl = template_for(cfg)
        out[key] = (tmpl, design_side(tmpl, cfg))
    return out


def _group_bodies(fab, group):
    stem = group.name.replace(":", "_") + "_"
    return [b for b in fab.bodies if b.name.startswith(stem)]


# -- dims -----------------------------------------------------------------------


def test_registry_and_cli_list_the_constructions(capsys):
    from main import _list_options

    assert set(KEYS) <= set(construction.AXLES)
    for key in KEYS:
        assert construction.axle(key).key == key
    _list_options()
    out = capsys.readouterr().out
    for key in KEYS:
        assert f"  {key:14} " in out


def test_dims_are_honest(side):
    key, _, design, *_ = side
    p = design.ctx.params
    for pillar in (True, False):
        d = construction.axle(key).dims(design.ctx, pillar)
        assert d.fill                                   # every layer between the ends is filled
        assert d.axle == pytest.approx(1.5)             # a 3 mm shaft
        assert d.neck >= (3.0 + 0.2) / 2 + min(p.min_wall, 0.85)   # a ring or a sleeve, not a rod
        assert d.neck <= d.spacer
        if key in ("bearing", "bushing"):
            ins = get(construction.axle(key).insert).dims
            assert d.flange == pytest.approx(ins["flange_d"] / 2)
            assert d.seat == pytest.approx(ins["od"] / 2)
            assert d.head >= d.flange
        else:
            assert d.flange == 0
            assert d.seat is None
        if key == "bolt":
            assert d.head == pytest.approx(7.0 / 2)     # the washer is the widest retainer
        else:
            assert d.head == pytest.approx(9.7 / 2)     # a Starlock clip


def test_constructions_refuse_what_they_cannot_build():
    cfg = _config("rod")
    ctx = design_side(template_for(cfg), cfg).ctx
    thin = replace(ctx, params=Params(link_radius=3.0))
    for c in (RodAxle(), BoltAxle(), BEARING, BUSHING):
        with pytest.raises(ConstructionError):
            c.dims(thin, False)                          # no link left around the hole
    with pytest.raises(ConstructionError, match="clip"):
        RodAxle().dims(replace(ctx, pitch=1.5), False)    # a clip doesn't fit a layer
    with pytest.raises(ConstructionError, match="ring"):
        RodAxle().dims(replace(ctx, params=Params(spacer_d=5.0)), False)
    with pytest.raises(ConstructionError, match="don't fit"):
        BoltAxle(pin_nut_layers=1).dims(ctx, False)      # a 4 mm nylock needs two layers
    with pytest.raises(ConstructionError, match="F623ZZ"):   # 10 mm wide, 4 mm long: no
        replace(BEARING, insert="bearing_f623zz").dims(ctx, False)
    with pytest.raises(ConstructionError, match="more than the"):   # 2.25 mm body, 2.2 mm sheet
        BUSHING.dims(replace(ctx, pitch=2.2), False)
    with pytest.raises(ConstructionError, match="flange"):
        BEARING.dims(replace(ctx, params=Params(spacer_d=6.0)), False)
    with pytest.raises(ConstructionError, match="isn't a flanged insert"):
        InsertAxle(key="x", label="", insert="m3_heat_set_insert", seat_fit=0.0,
                   glued=False).dims(ctx, False)
    for c in (RodAxle(), BoltAxle(), BEARING, BUSHING):
        c.dims(ctx, True)
        c.dims(ctx, False)


# -- the contract, the plan, the clashes ---------------------------------------------


@pytest.mark.parametrize("t", [0.0, 4.38])
def test_parts_stay_inside_their_claims(side, t):
    _, tmpl, design, *_ = side
    assert check_side(design, tmpl.freeze_at(t)) == []


def test_plan_verifies(side):
    _, tmpl, design, *_ = side
    assert verify_plan(design.plan, tmpl) == []


def test_jansen_single_builds_with_every_construction(jansen):
    for key, (tmpl, design) in jansen.items():
        assert verify_plan(design.plan, tmpl) == [], key
        assert check_side(design, tmpl.freeze_at(T)) == [], key


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
    key, _, design, build, _, axles = side
    p = design.ctx.params
    plates = {0: FRAME_OUTER, build.top: FRAME_INNER}
    link_hole = {"rod": 3.2, "bolt": 3.2, "bearing": 6.03, "bushing": 4.52}[key]
    plate_hole = 3.4 if key == "bolt" else p.hole(3.0, "glue")     # clamped, or glued
    for g in axles:
        got = g.realize(build)
        anchors = [s.layer for s in build.shapes(g.name) if s.label.endswith("anchor")]
        assert set(got.cuts) == set(g.axis.members) | {plates[k] for k in anchors}
        for m in g.axis.members:
            assert [c.d for c in got.cuts[m]] == [pytest.approx(link_hole)]
        for k in anchors:
            assert [c.d for c in got.cuts[plates[k]]] == [pytest.approx(plate_hole)]
        glue = [e for e in got.extras if e.key == "ca_glue"]
        if key == "bolt":
            assert glue == []
        else:
            assert bool(glue) == bool(anchors or key == "bearing"), g.name


def test_nothing_on_an_axle_can_slide(side):
    """From the bottom retainer to the top one the stack is continuous: links, rings or
    sleeves, flanges, clips, nuts, heads and the plates it is anchored in."""
    key, _, design, build, fab, axles = side
    play = 0.1 + 1e-6
    for g in axles:
        col = Column.of(build, g)
        spans = [build.z(k) for k in col.links] + [build.z(k) for k in col.anchors]
        for b in _group_bodies(fab, g):
            if b.name.endswith(("_rod", "_screw")):
                continue
            bb = b.part.bounding_box()
            spans.append((bb.min.Z, bb.max.Z))
        spans.sort()
        lo, hi = spans[0][0], max(z1 for _, z1 in spans)
        reach = lo
        for z0, z1 in spans:
            assert z0 <= reach + play, (g.name, z0, reach)
            reach = max(reach, z1)
        assert reach == pytest.approx(hi)
        shaft = next(b for b in _group_bodies(fab, g) if b.name.endswith(("_rod", "_screw")))
        sb = shaft.part.bounding_box()
        s0, s1 = sb.min.Z, sb.max.Z
        assert s0 < lo + 1e-6, g.name          # the shaft runs through
        assert s1 > hi - 1e-6, g.name


def test_spacers_and_retainers_are_what_the_key_says(side):
    key, _, design, build, fab, axles = side
    for g in axles:
        bodies = _group_bodies(fab, g)
        fabs = {b.name.split("_", 2)[2]: b.fab for b in bodies}
        col = Column.of(build, g)
        if key in ("rod", "bolt"):
            assert {f"ring{k}" for k in col.between} == {n for n in fabs if n.startswith("ring")}
            assert all(fabs[n] == "laser" for n in fabs if n.startswith("ring"))
        else:
            assert {f"sleeve{r[0]}" for r in col.runs} == {n for n in fabs if "sleeve" in n}
            assert all(fabs[n] == "printed" for n in fabs if "sleeve" in n)
            for m in g.axis.members:
                b = next(b for b in bodies if b.name.endswith(f"{m}_{key}"))
                assert b.rigid_with == m
                assert b.bom_key == construction.axle(key).insert
        if key == "bolt":
            assert {"screw", "washer", "nut"} <= set(fabs)
            screw = next(b for b in bodies if b.name.endswith("_screw"))
            assert screw.bom_key.startswith("m3_shcs_")
        else:
            assert "clip_lo" in fabs or "clip_hi" in fabs
            assert ("clip_lo" in fabs) == bool(col.below)
            assert ("clip_hi" in fabs) == bool(col.above)


def test_bom_counts_per_construction(side):
    key, _, design, build, fab, axles = side
    bom = bom_from_mechanism(fab, group=False)
    rows = {r.key: r for r in bom.purchased}
    for r in bom.purchased:
        assert get(r.key).offers, r.key
        assert r.url.startswith("https://")
    ends = sum(len(Column.of(build, g).below) + len(Column.of(build, g).above) for g in axles)
    if key == "bolt":
        assert rows["m3_nylock"].qty == rows["m3_washer"].qty == len(axles)
        screws = sum(r.qty for k, r in rows.items() if k.startswith("m3_shcs_") and r.qty)
        assert screws >= len(axles) + 4      # the axles' and the servo's screws
        assert "rod_3mm_100" not in rows
    else:
        assert rows["starlock_3mm"].qty == ends
        length = sum(e.qty for e in fab.bom_extras if e.key == "rod_3mm_100") * 100
        stems = [b for b in fab.bodies if b.name.endswith("_rod")]
        assert len(stems) == len(axles)
        assert length == pytest.approx(sum(b.part.bounding_box().size.Z for b in stems))
        assert rows["rod_3mm_100"].packs == 1
    if key in ("bearing", "bushing"):
        assert rows[construction.axle(key).insert].qty == sum(len(g.axis.members) for g in axles)
    if key == "bearing":
        assert any("into its links" in w for w in rows["ca_glue"].where)


def test_flanges_point_at_free_faces(side):
    key, _, design, build, fab, axles = side
    if key not in ("bearing", "bushing"):
        pytest.skip("no flanges")
    ins = get(construction.axle(key).insert).dims
    for g in axles:
        col = Column.of(build, g)
        for k, members in col.links.items():
            z0, z1 = build.z(k)
            for m in members:
                b = next(b for b in _group_bodies(fab, g) if b.name.endswith(f"{m}_{key}"))
                bb = b.part.bounding_box()
                b0, b1, height = bb.min.Z, bb.max.Z, bb.size.Z
                assert height == pytest.approx(ins.get("w", ins.get("l")))
                if b1 > z1 + 1e-6:                # flange up: the layer above is not a link
                    assert b1 == pytest.approx(z1 + ins["flange_t"])
                    assert k + 1 not in col.links
                    assert col.room(k + 1) >= ins["flange_d"] / 2
                else:
                    assert b0 == pytest.approx(z0 - ins["flange_t"])
                    assert k - 1 not in col.links
                    assert col.room(k - 1) >= ins["flange_d"] / 2


# -- mixed builds and the quad -----------------------------------------------------------


@pytest.mark.parametrize(("pin", "pillar"), [("bearing", "printed"), ("rod", "bolt")])
def test_mixed_constructions_plan_and_build(pin, pillar):
    cfg = BuildConfig(module="single", robot=False, pin=pin, pillar=pillar)
    tmpl = template_for(cfg)
    design = design_side(tmpl, cfg)
    assert verify_plan(design.plan, tmpl) == []
    assert check_side(design, tmpl.freeze_at(T)) == []
    mech = fabricate_side(design, tmpl.freeze_at(T))
    assert mech.meta["pin"] == pin
    assert mech.meta["pillar"] == pillar
    kinds = {b.name.split("_")[0] for b in mech.bodies if b.name.startswith(("pin_", "pillar_"))}
    assert kinds == {"pin", "pillar"}
    assert _clashes(mech) == []


def test_quad_plans_with_every_construction():
    heights = {}
    for key in ("printed", *KEYS):
        cfg = BuildConfig(module="quad", robot=False, pin=key, pillar="printed")
        tmpl = template_for(cfg)
        plan = design_side(tmpl, cfg).plan
        assert verify_plan(plan, tmpl) == [], key
        heights[key] = plan.height
    for key in KEYS:            # a nut end costs at most a layer over a printed cap
        assert heights[key] <= heights["printed"] + 3.0, heights


def test_robot_bom_doubles_the_side(side):
    key, tmpl, design, *_ = side
    if key != "rod":
        pytest.skip("one robot build is enough")
    robot = fabricate(tmpl, replace(design.config, robot=True), T)
    assert _clashes(robot, robot.meta["fastened"]) == []
    bom = bom_from_mechanism(robot, group=False)
    rows = {r.key: r for r in bom.purchased}
    side_bom = {r.key: r for r in bom_from_mechanism(fabricate(tmpl, design.config, T),
                                                     group=False).purchased}
    assert rows["starlock_3mm"].qty == 2 * side_bom["starlock_3mm"].qty
    assert rows["rod_3mm_100"].qty == pytest.approx(2 * side_bom["rod_3mm_100"].qty)


# -- refusals the planner reports ----------------------------------------------------------


def test_flange_sides_rules():
    room = {1: 4.0, 4: 4.0, 6: 4.0, 7: 2.0}.get
    assert flange_sides([2, 3], lambda k: room(k, 0.0), 3.6) == {2: -1, 3: +1}
    assert flange_sides([5], lambda k: room(k, 0.0), 3.6) == {5: -1}        # the roomier side
    assert flange_sides([5], lambda k: {4: 3.0, 6: 4.0}.get(k, 0.0), 3.6) == {5: +1}
    with pytest.raises(Unbuildable, match="adjacent layers 2..4"):
        flange_sides([2, 3, 4], lambda k: 5.0, 3.6, {3: "b2"})
    with pytest.raises(Unbuildable, match="no room for the 7.2 mm flange of b9"):
        flange_sides([5], lambda k: 3.0, 3.6, {5: "b9"})


def test_bolt_refuses_a_stack_no_standard_screw_spans():
    cfg = _config("bolt")
    design = design_side(template_for(cfg), cfg)
    pin = next(g for g in design.groups if isinstance(g, AxleGroup) and not g.pillar)
    claim = pin.claims(design.ctx)[0]
    a, b = pin.axis.members
    layout = Layout({**design.plan.layers, a: 2, b: 8}, 12, design.ctx.pitch)   # a 21 mm stack
    with pytest.raises(Unbuildable, match="no standard M3 screw for its 21 mm stack"):
        claim.make(layout)
    layout = Layout({**design.plan.layers, a: 2, b: 3}, 12, design.ctx.pitch)
    labels = {p.layer: p.label.rsplit(" ", 1)[1] for p in claim.make(layout)}
    assert labels == {1: "head", 2: "axle", 3: "axle", 4: "nut", 5: "nut"}


def test_new_catalog_items():
    for key in ("m3_shcs_45", "m3_shcs_50", "e_clip_din6799_2p3", "m3_nylock_thin"):
        item = get(key)
        assert item.offers
        assert all(o.url.startswith("https://") for o in item.offers)
    assert get("m3_shcs_45").dims["length"] == 45.0
