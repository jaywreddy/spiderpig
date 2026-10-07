"""The bolt crank (:class:`construction.crank.BoltCrank`): single aluminium web plates on
hex standoff crankpins (``bolt``, the default since 2026-10-04) or round standoff ones
(``bolt_round``); its route rules and strength."""

from __future__ import annotations

import math

import pytest

from spiderpig.config import BuildConfig
from spiderpig.construction import CRANKS
from spiderpig.construction.base import ConstructionError
from spiderpig.construction.contract import bad_solids, check_side, clashes
from spiderpig.construction.crank import BoltCrank, CrankRoute, HexJoint, Run
from spiderpig.hardware.catalog import get
from spiderpig.hardware.crank_catalog import HEX_M3_LENGTHS
from spiderpig.stack import Layout, Unbuildable
from spiderpig.strength import crank_capacity

PITCH = 3.0


def test_registered_and_selectable():
    assert set(CRANKS) == {"bolt", "bolt_round"}
    assert isinstance(CRANKS["bolt"], BoltCrank)
    # the hex standoff crankpin is the default; the friction-clamped round one selectable
    assert CRANKS["bolt"].for_sheet(BuildConfig().crank_sheet).hex
    round_ = CRANKS["bolt_round"].for_sheet(BuildConfig().crank_sheet)
    assert isinstance(round_, BoltCrank)
    assert not round_.hex
    # the crankpins' screws are threadlocked
    assert get(CRANKS["bolt"].lock_key).category == "adhesive"


def test_the_single_plates_need_a_metal_crank_sheet():
    """:meth:`BoltCrank.for_sheet` takes the sheet's thickness and yield; a non-metal
    sheet (the removed acrylic two-plate crank's) raises."""
    from spiderpig.materials import sheet

    c = CRANKS["bolt"].for_sheet("al6061_2mm")
    assert (c.web_t, c.web_yield) == (sheet("al6061_2mm").thickness,
                                      sheet("al6061_2mm").yield_mpa)
    assert CRANKS["bolt"].for_sheet(None) is CRANKS["bolt"]
    for key in ("bolt", "bolt_round"):
        with pytest.raises(ConstructionError, match="need a metal crank sheet"):
            CRANKS[key].for_sheet("acrylic_3mm")


def test_the_hub_chain_is_capped_by_default():
    """The assembly audit of 2026-10-04: no order drives a screw over the hub plate once
    the hub plate, horn, servo and inner plate go on as one unit, so the hex chain that
    ends in the hub plate has none (the hub plate caps it); the round standoff's clamp needs
    both. The capped standoff is carried by its pressed sleeve and the crank body stopped by
    the stub's thrust sleeve, rated in what the crank body's float leaves of the hub's
    pocket."""
    bolt = CRANKS["bolt"].for_sheet(BuildConfig().crank_sheet)
    round_ = CRANKS["bolt_round"].for_sheet(BuildConfig().crank_sheet)
    assert bolt.hub_capped(None, "M")
    assert not round_.hub_capped(None, "M")
    assert bolt.capped_press > 0
    span, t = 29.9, bolt.web_t
    assert bolt.fit_hex(span, t, t, capped=True).engaged_hi == pytest.approx(
        t - bolt.thrust_play)
    assert bolt.fit_hex(span, t, t).engaged_hi == pytest.approx(t)


def test_the_stub_thrust_sleeve_comes_with_a_capped_chain():
    """:meth:`BoltCrank.stub_thrust_r`: the thrust sleeve's radius when a chain ends capped
    in the hub plate's layer; none when no chain reaches the hub, for the round standoff
    (screwed over the hub plate), or with no journal stub (no bearing)."""
    bolt = CRANKS["bolt"]
    route = CrankRoute((Run("M", 2, 4),))
    assert bolt.stub_thrust_r(None, route, 5) == pytest.approx(bolt.thrust_od / 2)
    assert bolt.stub_thrust_r(None, route, 7) == 0.0
    assert CRANKS["bolt_round"].stub_thrust_r(None, route, 5) == 0.0
    assert bolt.stub_thrust_r(None, CrankRoute(route.runs, bearing=False), 5) == 0.0


def test_the_stub_must_reach_the_outer_plate_at_the_plans_z():
    """:meth:`BoltCrank.check_route`: at the plan's z the stub standoff (M3 round, stock
    6-30 mm) runs from the lowest web down into the outer frame plate; a lowest web 30 mm
    over the outer plate's bottom (layer 10 of 3 mm layers) has a stock 30 mm standoff, one
    33 mm over it (layer 11) none, and the route is refused."""
    c = CRANKS["bolt"].for_sheet(BuildConfig().crank_sheet)
    L = Layout({}, 20, PITCH, final=True)
    assert c.stub_z(30.0, PITCH, c.web_t)[1] == 30
    assert c.stub_z(33.0, PITCH, c.web_t) is None
    c.check_route(L, CrankRoute((Run("M", 11, 12),)), {}, {10, 13})
    with pytest.raises(Unbuildable, match="no stock stub standoff reaches"):
        c.check_route(L, CrankRoute((Run("M", 12, 13),)), {}, {11, 14})
    # no journal stub, nothing to reach
    c.check_route(L, CrankRoute((Run("M", 12, 13),), bearing=False), {}, {11, 14})


@pytest.mark.slow
def test_a_screw_over_the_hub_plate_has_no_assembly_order(side):
    """The round standoff's friction clamp screws over the hub plate, which no order can
    drive with the horn screws coming up through it from below: the crank's note says so
    and the audit fails it (the Strider quad's and klann_lego's, 2026-10-04)."""
    note = side("single", crank="bolt_round").meta["crank_bolt"]
    assert "no order drives both" in note["assembly"]
    assert side("single", crank="bolt").meta["crank_bolt"]["assembly"] is None


def test_single_web_rules_for_the_router():
    """The bolt crank's single plates (2026-10-04): chains share a web or a journal standoff
    joins them, never a journal plate; its screw heads are gap pieces the router keeps clear
    (one per crank point, the horn screws', one at O); a span is buildable whatever the
    riders' faces (no end play to set back), and the first web sits where a stock stub
    reaches the outer plate."""
    from spiderpig.construction.route import joint_rules
    from spiderpig.fabricate import side_problem, template_for

    cfg = BuildConfig(linkage="klann", module="single", robot=False, crank="bolt")
    ctx, groups, problem = side_problem(template_for(cfg), cfg)
    crank = next(g for g in groups if g.name == "crank")
    c = crank.construction.resolve(ctx)
    rules = joint_rules(crank.construction, ctx, crank.dims(ctx))
    assert rules.gap_head == pytest.approx(c.head_r())
    assert rules.gap_head > 0
    assert rules.horn_heads
    assert problem.spec.heads == "gap_sink"     # in gaps, else the pivots' heads sunk
    assert rules.gap_washer == c.washer_r > 0      # its run washers: the leaf routes round
    assert len(problem.router.gap_pieces) == problem.router.n + len(rules.horn_heads) + 1
    t, frame = ctx.sheet_t("crank"), ctx.sheet_t("frame")
    for n, m in rules.spans.items():
        assert m in (0, (1 << 32) - 1), n          # every set of faces alike
        assert bool(m) == c._web_span_ok(n - 2, 0, ctx.pitch, t), n
    assert any(rules.spans.values())
    assert dict(rules.j_spans) == {n: c._web_span_ok(n, 0, ctx.pitch, t) for n in range(64)}
    assert rules.bottom_layers
    assert min(rules.bottom_layers) >= 2
    for a in rules.bottom_layers:
        assert c.stub_z(frame + (a - 1) * ctx.pitch, frame, t) is not None, a


def test_the_round_standoffs_capacity_is_its_friction_clamp():
    """``bolt_round``: one friction joint per web (UNVERIFIED coefficients), the face under
    the M4 screw's preload and the head through the thread; the hex: its pockets and its
    torsion."""
    round_ = CRANKS["bolt_round"].for_sheet(BuildConfig().crank_sheet)
    (key, nm), = round_.capacity().items()
    assert key.startswith("web clamped on the standoff's end (2200 N, mu 0.3)")
    assert nm > 0.85                          # holds the jam twist at the 0.85 N·m limit
    hexed = CRANKS["bolt"].for_sheet(BuildConfig().crank_sheet).capacity()
    assert any("pocket" in k for k in hexed)
    assert any("torsion" in k for k in hexed)


@pytest.mark.slow
def test_the_klann_single_builds_clean_with_the_bolt_crank(design, side):
    """The default (hex standoff) crank: clean, every crankpin a stock hex standoff whose
    hex fills its plates' pockets (or all but ``recess_max``), its screws, washers, sleeve."""
    tmpl, d = design("single", crank="bolt")
    mech = side("single", crank="bolt")
    assert check_side(d, mech) == []
    assert clashes(mech) == []
    assert bad_solids(mech) == []
    plates = [b for b in mech.bodies if b.name.startswith("crank_plate")]
    assert plates
    assert all(b.fab == "laser" for b in plates)
    note = mech.meta["crank_bolt"]
    assert note["chains"]
    assert note["plates"] == len(plates)
    c = BoltCrank().for_sheet(d.config.crank_sheet)
    for ch in note["chains"]:
        assert get(ch["standoff"]).category == "standoff"
        assert ch["length_mm"] in HEX_M3_LENGTHS
        assert min(ch["hex_engaged_mm"]) >= c.web_t - c.recess_max - 1e-9
        assert ch["sleeve_mm"] > 0
    keys = {b.bom_key for b in mech.bodies if b.name.startswith("crank_")}
    assert "m3_washer_9021" in keys
    assert any(k and k.startswith("hex_standoff_m3_") for k in keys)
    sleeves = [b for b in mech.bodies if b.name.startswith("crank_pin_sleeve")]
    assert sleeves
    assert all(b.fab == "printed" for b in sleeves)
    # every plate is on the sheets
    from spiderpig.layout import laser_bodies

    assert {b.name for b in plates} <= {b.name for b in laser_bodies(mech)}
    # the horn's face on a layer boundary (the drive's spacer)
    drive = d.ctx.interfaces["drive"]
    n = (drive.horn_face_depth - d.ctx.sheet_t("frame")) / d.ctx.pitch   # under the Al plate
    assert n == pytest.approx(round(n))
    # the service's rules on every crank plate: no error, and no edge or corner warning;
    # the hex pockets stand 1 x t (web_edge_t) inside the webs' rims, which the web rule
    # (since 2026-10-04) reports as a warning
    from spiderpig.manufacture import check

    issues = [i for i in check(mech, d.config.sheet)["issues"]
              if i["part"].startswith("crank_plate")]
    assert all(i["rule"] == "web" and i["level"] == "warning" for i in issues)
    assert all(i["value"] >= c.web_t - 1e-6 for i in issues)
    # the chain capped by the hub plate: its sleeve pressed on the hex, the stub's thrust
    # sleeve the crank body's stop toward the outer plate (the assembly audit of 2026-10-04)
    capped = [ch for ch in note["chains"] if ch["capped"]]
    assert capped
    assert all(ch["sleeve_press_mm"] == c.capped_press > 0 for ch in capped)
    assert all(ch["hex_engaged_mm"][1] == pytest.approx(c.web_t - c.thrust_play)
               for ch in capped)
    assert note["stub_thrust"]["play_mm"] == c.thrust_play
    thrust = [b for b in mech.bodies if b.name == "crank_stub_thrust"]
    assert [b.fab for b in thrust] == ["printed"]
    assert note["assembly"] is None


@pytest.mark.slow
def test_the_bolt_crank_plates_pack_onto_dxf_sheets(side, tmp_path):
    """Each plate is cut on the sheets, its hex pockets among its holes."""
    import ezdxf

    from spiderpig.layout import save_sheets

    mech = side("single", crank="bolt")
    files = save_sheets(mech, tmp_path / "sheet")
    assert files
    parts = (tmp_path / "sheet_parts.csv").read_text()
    plates = [b.name for b in mech.bodies if b.name.startswith("crank_plate")]
    assert all(n in parts for n in plates)
    polys = sum(len(ezdxf.readfile(str(f)).modelspace().query("LWPOLYLINE")) for f in files)
    assert polys > len(plates)        # outlines, and the pockets' (non-circular) holes




# -- the hex standoff crankpin (the user's decision of 2026-10-04) -------------------------

HEX = BoltCrank().for_sheet("al6061_2p5mm")


@pytest.mark.parametrize("span", [x / 4 for x in range(30, 200)])
def test_every_hex_fit_is_stock_and_holds_its_plates(span):
    """What :meth:`BoltCrank.fit_hex` promises, re-derived: a stock length; its ends past
    the plates by at most ``protrude_max`` (each stack within a clearance gap, a printed
    collar from ``collar_min``), or inside both pockets by at most ``recess_max``; the two
    screws' tips apart inside the standoff, each with its least thread; the sleeve between
    the plates."""
    from spiderpig.stack import GAP_MAX

    t = HEX.web_t
    j = HEX.fit_hex(span, t, t)
    if j is None:
        return
    assert isinstance(j, HexJoint)
    assert j.length in HEX_M3_LENGTHS
    assert j.length == pytest.approx(span + j.out_lo + j.out_hi)
    for out, collar, stack in ((j.out_lo, j.collar_lo, j.stack_lo),
                               (j.out_hi, j.collar_hi, j.stack_hi)):
        assert -HEX.recess_max - 1e-9 <= out <= HEX.protrude_max + 1e-9
        assert collar == (pytest.approx(out, abs=0.01) if out >= HEX.collar_min else 0.0)
        assert stack + HEX.head_clear <= GAP_MAX + 1e-9
    assert (j.out_lo < 0) == (j.out_hi < 0)                     # recessed: both ends
    assert min(j.engaged_lo, j.engaged_hi) >= t - HEX.recess_max - 1e-9
    assert j.engage_lo + j.engage_hi <= j.length - HEX.hex_tip_gap + 1e-9
    assert min(j.engage_lo, j.engage_hi) >= HEX.hex_min_engage - 1e-9
    assert 0 < j.sleeve <= span - 2 * t + 1e-9


def test_the_hex_fits_cover_short_spans():
    """Every span of one to five 3 mm run layers between 0.100 in plates takes a stock
    standoff (the stock lengths step 1-3 mm there)."""
    t = HEX.web_t
    for run in range(1, 6):
        assert HEX.fit_hex(3.0 + run * 3.0 + t, t, t) is not None, run


def test_the_crank_sheet_holds_the_hex_at_sf_2():
    """The default crank sheet (0.100 in 6061-T6) is the thinnest whose hex pockets carry
    twice the jam twist of two crankpins 180 deg apart (2 x 0.85 N·m) with the standoff
    recessed its most; 0.063 in 5052, the sheet before, holds 1.9 N·m whole."""
    from spiderpig.materials import thinnest_sheet

    assert BuildConfig().crank_sheet == thinnest_sheet("crank") == "al6061_2p5mm"
    caps = HEX.capacity()
    key = next(k for k in caps if "pocket" in k)
    a = 5.5 / math.sqrt(3) - HEX.hex_corner_loss
    assert caps[key] == pytest.approx(0.75 * 276 * a * a * 2.54 / 1e3, abs=1e-3)
    worst = HEX.hex_capacity(HexJoint("", 0, 0, 0, 0, 0, 0, "", 0, 0, "", 0, 0,
                                      2.54 - HEX.recess_max, 2.54, 0))
    assert min(worst.values()) >= 2 * 2 * 0.85
    thin = BoltCrank().for_sheet("al5052_1p6mm")
    assert min(thin.capacity().values()) < 2 * 2 * 0.85
    nominal = crank_capacity({}, BuildConfig())
    assert min(nominal.values()) == pytest.approx(min(caps.values()))


def test_the_hex_pocket_has_dogbones_at_the_service_radius():
    """A relief circle of the service's inside radius through each corner: one outline,
    each flat whole but for 0.09 mm at each end."""
    from spiderpig.materials import sheet

    assert HEX.dogbone_r >= sheet("al6061_2p5mm").corner_r
    cut = HEX.hex_cut((0.0, 0.0), 0.0, 1.0, 0.0)
    assert len(cut.solids()) == 1
    af = HEX.hex_pocket_af()
    plain = math.sqrt(3) / 2 * af ** 2
    assert plain < cut.volume < plain + 6 * math.pi * HEX.dogbone_r ** 2
    assert HEX.hex_reach() == pytest.approx(af / math.sqrt(3) + 2 * HEX.dogbone_r)


def test_horn_screws_take_shims_where_the_hub_plate_is_thin():
    """The XL430's M2 horn screws engage 1.5-2.0 mm of its horn: under a thin hub plate a
    stock length is too long, and DIN 988 shims under the head take up the rest."""
    from spiderpig.fabricate import side_problem, template_for

    cfg = BuildConfig(linkage="klann", module="single", robot=False, servo="xl430_w250")
    ctx, _, _ = side_problem(template_for(cfg), cfg)
    c = BoltCrank().resolve(ctx)
    for seg in (1.6, 2.032, 2.54, 3.175):
        got = c.horn_fit_web(ctx, seg, 3.032)
        assert got is not None, seg
        sk, length, e, shim = got
        assert 1.5 - 1e-9 <= e <= 2.0 + 1e-9
        assert length == pytest.approx(seg + 3.032 + shim + e)
        assert 0 <= shim <= c.horn_shim_max
    assert c.horn_hole(ctx) >= get(ctx.sheet("crank")).dims["min_hole"]


def test_the_crank_defaults_per_module_and_linkage():
    """The hex-standoff crank is the default; the tables name where it doesn't plan or
    breaks a cut rule (the merge of 2026-10-04, config.MODULE_CRANKS / LINKAGE_CRANKS)."""
    from spiderpig.config import BuildConfig as Cfg

    assert Cfg().crank == "bolt"                                        # the Strider double
    assert Cfg(module="single", robot=False).crank == "bolt"
    assert Cfg(module="decker").crank == Cfg(module="quad").crank == "bolt"    # 2026-10-05
    assert Cfg(linkage="klann_lego").crank == "bolt"   # r4: b1 grown round the hex bore
    assert Cfg(linkage="trotbot_heel").crank == "bolt_round"
    assert Cfg(linkage="hoecken_pantograph", robot=False).crank == "bolt"
    assert Cfg(module="quad", crank="bolt").crank == "bolt"               # named: as named
