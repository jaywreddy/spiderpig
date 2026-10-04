"""The bolt crank (:class:`construction.crank.BoltCrank`): laser-cut plate stacks keyed on
M6 hex-bolt crankpins, and its route rules and strength."""

from __future__ import annotations

import math

import pytest

from spiderpig.config import BuildConfig
from spiderpig.construction import CRANKS
from spiderpig.construction.contract import bad_solids, check_side, clashes
from spiderpig.construction.crank import BoltCrank, hex_bearing_nm, hex_pocket
from spiderpig.hardware.catalog import get
from spiderpig.hardware.crank_catalog import M6_BOLT_LENGTHS, M6_THREAD_B, m6_bolt
from spiderpig.strength import crank_capacity

PITCH = 3.0


def test_registered_and_selectable():
    assert isinstance(CRANKS["bolt"], BoltCrank)
    assert {"keyed", "keyed_float", "printed", "bolt"} <= set(CRANKS)


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
    assert problem.spec.heads == "gap"
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
    for ch in note["chains"]:
        assert ch["bare_layers"] >= 1
        assert get(ch["bolt"]).category == "fastener"
    keys = {b.bom_key for b in mech.bodies if b.name.startswith("crank_")}
    assert "m6_nylock" in keys
    assert any(k and k.startswith("m6_hex_bolt_") for k in keys)
    # every plate is on the sheets
    from spiderpig.layout import laser_bodies

    assert {b.name for b in plates} <= {b.name for b in laser_bodies(mech)}
    # the horn's face on a layer boundary (the drive's spacer)
    drive = d.ctx.interfaces["drive"]
    n = (drive.horn_face_depth - d.ctx.sheet_t("frame")) / d.ctx.pitch   # under the Al plate
    assert n == pytest.approx(round(n))


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
