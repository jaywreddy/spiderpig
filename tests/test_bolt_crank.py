"""The bolt crank (:class:`construction.crank.BoltCrank`): on the default aluminium crank
sheet single plates on hex standoff crankpins (2026-10-04), on an acrylic one plate stacks
keyed on M6 hex-bolt crankpins; its route rules and strength."""

from __future__ import annotations

import math

import pytest

from spiderpig.config import BuildConfig
from spiderpig.construction import CRANKS
from spiderpig.construction.contract import bad_solids, check_side, clashes
from spiderpig.construction.crank import BoltCrank, HexJoint, hex_bearing_nm, hex_pocket
from spiderpig.hardware.catalog import get
from spiderpig.hardware.crank_catalog import (
    HEX_M3_LENGTHS,
    M6_BOLT_LENGTHS,
    M6_THREAD_B,
    m6_bolt,
)
from spiderpig.strength import crank_capacity

PITCH = 3.0


def test_registered_and_selectable():
    assert isinstance(CRANKS["bolt"], BoltCrank)
    assert {"keyed", "keyed_float", "printed", "bolt", "bolt_round"} <= set(CRANKS)
    # the hex standoff crankpin is the default; the friction-clamped round one selectable
    assert CRANKS["bolt"].for_sheet(BuildConfig().crank_sheet).hex
    round_ = CRANKS["bolt_round"].for_sheet(BuildConfig().crank_sheet)
    assert round_.single
    assert not round_.hex


def test_the_hub_chain_is_capped_by_default():
    """The assembly audit of 2026-10-04: no order drives a screw over the hub plate once
    the hub plate, horn, servo and inner plate go on as one unit, so the chain that ends
    in the hub plate has none (the hub plate caps it); ``bolt_hub_screw`` keeps it. The
    capped standoff is carried by its pressed sleeve and the crank body stopped by the
    stub's thrust sleeve; ``bolt_unretained`` keeps the build before."""
    assert not CRANKS["bolt"].hub_screw
    assert CRANKS["bolt_hub_screw"].hub_screw
    assert CRANKS["bolt_hub_screw"].for_sheet(BuildConfig().crank_sheet).hex
    bolt = CRANKS["bolt"].for_sheet(BuildConfig().crank_sheet)
    assert bolt.capped_press > 0
    assert bolt.stub_thrust
    free = CRANKS["bolt_unretained"].for_sheet(BuildConfig().crank_sheet)
    assert free.hex
    assert free.capped_press == 0
    assert not free.stub_thrust
    # the capped hex is rated in what the crank body's float leaves of the hub's pocket
    span, t = 29.9, bolt.web_t
    assert bolt.fit_hex(span, t, t, capped=True).engaged_hi == pytest.approx(
        t - bolt.thrust_play)
    assert free.fit_hex(span, t, t, capped=True).engaged_hi == pytest.approx(t)


@pytest.mark.slow
def test_a_screw_over_the_hub_plate_has_no_assembly_order(side):
    """The round standoff's friction clamp screws over the hub plate, which no order can
    drive with the horn screws coming up through it from below: the crank's note says so
    and the audit fails it (the Strider quad's and klann_lego's, 2026-10-04)."""
    note = side("single", crank="bolt_round").meta["crank_bolt"]
    assert "no order drives both" in note["assembly"]
    assert side("single", crank="bolt").meta["crank_bolt"]["assembly"] is None


@pytest.mark.parametrize("run_layers", range(1, 14))
@pytest.mark.parametrize("low", range(4))
def test_every_fit_keeps_the_riders_on_the_plain_shank(run_layers, low):
    """What :meth:`BoltCrank.fit` promises, re-derived: the riders on the full-diameter
    shank, the whole nut on complete thread, the tip past the nylock inside the tip layer,
    a stock length (cut only shorter)."""
    c = BoltCrank()
    j = c.fit(run_layers, low, PITCH)
    if j is None:
        return
    head_af, head_h = c.head()
    _, nut_h = c.nut()
    z_h = (2 + run_layers) * PITCH + c.head_gap          # the head's underside
    assert j.length in M6_BOLT_LENGTHS
    assert j.cut <= j.length
    assert j.plain == j.length - M6_THREAD_B
    if run_layers > low:                                 # riders on the plain shank
        assert z_h - (j.plain - c.runout) <= (2 + low) * PITCH + 1e-9
    nut_top = nut_h - j.sink
    assert z_h - j.plain >= nut_top - 1e-9               # the nut on complete thread
    assert 0.0 <= j.sink <= c.max_sink
    tip = z_h - j.cut
    assert tip <= -j.sink - c.min_tip + 1e-9             # past the nylock
    assert tip >= -PITCH + c.tip_recess - 1e-9           # inside the tip layer
    assert j.head_engaged == head_h


def test_a_rider_cant_sit_on_the_nut_stack():
    """The thread's runout and the nut's complete thread exclude it at every span."""
    c = BoltCrank()
    assert all(c.fit(r, 0, PITCH) is None for r in range(1, 20))
    assert any(c.fit(r, 1, PITCH) for r in range(1, 20))
    assert any(c.fit(r, 2, PITCH) for r in range(1, 20))


def test_joint_rules_for_the_router(design):
    tmpl, d = design("single", crank="bolt")
    from spiderpig.construction.route import joint_rules
    from spiderpig.fabricate import side_problem

    # the two-plate stack crank: an acrylic crank sheet (the single aluminium webs' rules:
    # test_single_web_rules_for_the_router)
    ctx, groups, problem = side_problem(tmpl, BuildConfig(linkage="klann", module="single",
                                                          robot=False, crank="bolt",
                                                          crank_sheet="acrylic_3mm"))
    crank = next(g for g in groups if g.name == "crank")
    rules = joint_rules(crank.construction, ctx, crank.dims(ctx))
    for flag in ("two_layer_top", "two_layer_bottom", "tip", "low_count", "share_stack"):
        assert getattr(rules, flag), flag
    assert not rules.inner_webs
    assert rules.bottom_layers
    assert min(rules.bottom_layers) >= 2
    c = crank.construction
    for n, m in rules.spans.items():
        for low in range(4):
            assert bool(m >> (4 * low) & 1) == (c.fit(n - 4, low, ctx.pitch) is not None)


def test_single_web_rules_for_the_router():
    """On an aluminium crank sheet the bolt crank is single plates (2026-10-04): chains share
    a web or a journal standoff joins them, never a journal plate; its screw heads are gap
    pieces the router keeps clear (one per crank point, the horn screws', one at O)."""
    from spiderpig.construction.route import joint_rules
    from spiderpig.fabricate import side_problem, template_for

    cfg = BuildConfig(linkage="klann", module="single", robot=False, crank="bolt")
    ctx, groups, problem = side_problem(template_for(cfg), cfg)
    crank = next(g for g in groups if g.name == "crank")
    assert crank.construction.single
    rules = joint_rules(crank.construction, ctx, crank.dims(ctx))
    assert rules.j_last
    assert not rules.two_layer_top
    assert not rules.two_layer_bottom
    assert rules.gap_head > 0
    assert rules.horn_heads
    assert problem.spec.heads == "gap_sink"     # in gaps, else the pivots' heads sunk
    assert rules.gap_washer > 0                    # its run washers: the leaf routes round
    assert len(problem.router.gap_pieces) == problem.router.n + len(rules.horn_heads) + 1


def test_capacity_per_element_and_no_post_shell():
    caps = BoltCrank().capacity()
    assert set(caps) == {
        "head pocket, 4 mm of 10 AF in acrylic", "nut pocket, 6 mm of 10 AF in acrylic",
        "M6 thread torsion (640 MPa)",
        "nut lock (nylock prevailing + threadlocker breakaway (half: plated steel))"}
    a = 10.0 / math.sqrt(3) - 0.5
    assert caps["head pocket, 4 mm of 10 AF in acrylic"] == pytest.approx(
        0.75 * 50 * a * a * 4 / 1e3, abs=1e-3)
    assert caps["M6 thread torsion (640 MPa)"] == pytest.approx(
        640 / math.sqrt(3) * math.pi * 4.92 ** 3 / 16 / 1e3, abs=1e-3)
    assert min(caps.values()) > 2 * 0.85      # holds twice the jam twist at the 0.85 N·m limit
    keyed = crank_capacity({}, BuildConfig(crank="keyed"))
    assert not any("post shell" in k for k in keyed)
    assert min(keyed.values()) == pytest.approx(
        hex_bearing_nm(5.0, 1.6 - 0.4) + 0.17, abs=1e-3)


def test_hex_pocket_has_its_corner_reliefs():
    cut = hex_pocket((0.0, 0.0), 10.1, 0.0, 3.0, 0.0)
    plain = math.sqrt(3) / 2 * 10.1 ** 2 * 3.0      # a hexagon: (sqrt 3 / 2) AF^2
    assert cut.volume > plain
    assert cut.volume < plain + 6 * math.pi * 0.25 ** 2 * 3.0


def test_catalog_has_every_bolt_and_the_nut():
    for L in M6_BOLT_LENGTHS:
        d = get(m6_bolt(L)).dims
        assert (d["b"], d["head_af"], d["head_h"]) == (18.0, 10.0, 4.0)
    # partially threaded M6 starts at 30 mm (M6 x 25 is sold only fully threaded, DIN 933)
    assert min(M6_BOLT_LENGTHS) == 30.0
    assert get("m6_nylock").dims["h"] == 6.0
    assert get("threadlocker_243").category == "adhesive"


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

    mech = side("single", crank="bolt", crank_sheet="acrylic_3mm")
    files = save_sheets(mech, tmp_path / "sheet")
    assert files
    parts = (tmp_path / "sheet_parts.csv").read_text()
    plates = [b.name for b in mech.bodies if b.name.startswith("crank_plate")]
    assert all(n in parts for n in plates)
    polys = sum(len(ezdxf.readfile(str(f)).modelspace().query("LWPOLYLINE")) for f in files)
    assert polys > len(plates)        # outlines, and the pockets' (non-circular) holes


def test_the_chains_rule_the_thinner_sizes_out():
    """The bolt crank's lower bound on the stack (:meth:`CrankRouter.min_top`): four chains
    of at least 8 layers between their outer webs, a two-layer mid stack shared, over the
    tip layer and up to the hub: 30 layers at least on the Klann quad (it plans 31), so the
    planner starts there; the keyed crank's rules give none."""
    from spiderpig.fabricate import side_problem, template_for

    cfg = BuildConfig(linkage="klann", module="quad", robot=False, crank_sheet="acrylic_3mm")
    _, _, problem = side_problem(template_for(cfg), cfg)
    assert problem.spec.min_top == 29
    assert "4 crankpin chains, each at least 8 layers" in problem.floor
    keyed = BuildConfig(linkage="klann", module="quad", robot=False, crank="keyed")
    _, _, problem = side_problem(template_for(keyed), keyed)
    assert problem.spec.min_top == 2
    assert problem.floor == ""


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
    assert Cfg(linkage="klann_lego").crank == "bolt_round"
    assert Cfg(linkage="trotbot_heel").crank == "bolt_round"
    assert Cfg(linkage="hoecken_pantograph", robot=False).crank == "bolt"
    assert Cfg(module="quad", crank="bolt").crank == "bolt"               # named: as named
