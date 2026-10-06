"""Regressions from the adversarial review of 2026-10-05: the frame ties on every servo, shim
heights the BOM can't split, and Chicago pins whose lower spacer is thinner than a print."""

from __future__ import annotations

import pytest
from build123d import Location

from spiderpig.construction.chassis import _chain, _shims
from spiderpig.hardware.bom import BomLine, split_shims
from spiderpig.mechanism import Body
from spiderpig.shapes import disc


@pytest.mark.parametrize(("span", "segs", "take"), [
    (32.0, [30], 2.0),          # the STS3215's chain
    (34.0, [15, 18], 1.0),      # the XL430's
    (23.0, [20], 3.0),          # the XL330's: no M3 pair fills it, 3 mm of shims
    (30.1, [30], 0.0),          # within the plates' 0.1 mm: no 0.1 mm shim
    (31.1, [30], 1.0),
    (30.6, [30], 0.5),
])
def test_tie_chains_take_up_in_whole_steps(span, segs, take):
    got = _chain(span)
    assert got == (segs, take)
    assert sum(_shims(take)) == pytest.approx(take)
    assert all(s in (1.0, 0.5) for s in _shims(take))


def _ring(name: str, t: float) -> Body:
    part = (disc((0, 0), 3.0, 0.0, t) - disc((0, 0), 1.55, -1.0, t + 1.0)).moved(Location())
    return Body(name, part=part, fab="purchased", bom_key="shim_din988_3x6")


def test_a_shim_height_the_steps_cant_make_is_not_dropped():
    ok = _ring("L.tie_shims0", 1.5)                # 1.0 + 0.5: two DIN 433 lines worth
    lines, notes = split_shims([BomLine("shim_din988_3x6", 1, ok.name),
                                BomLine("shim_din988_3x6", 1, "frame tie 0, L")],
                               {ok.name: ok})
    assert not notes
    assert sum(x.qty for x in lines if x.key == "m3_washer_433") == 3   # 2 for 1 mm, 1 for 0.5
    thin = _ring("L.tie_shims1", 0.1)              # finer than 1.0/0.5: the family's 0.1
    lines, notes = split_shims([BomLine("shim_din988_3x6", 1, thin.name)], {thin.name: thin})
    assert not notes
    assert [x.key for x in lines] == ["shim_din988_3x6_t0p1"]
    odd = _ring("L.tie_shims2", 0.05)              # no shim makes 0.05 mm
    lines, notes = split_shims([BomLine("shim_din988_3x6", 1, odd.name)], {odd.name: odd})
    assert notes                                   # said, and kept as the family's line
    assert [x.key for x in lines] == ["shim_din988_3x6"]


@pytest.mark.slow
def test_an_xl330_robot_builds_its_frame_ties():
    from spiderpig.config import BuildConfig
    from spiderpig.fabricate import design_side, fabricate, template_for

    cfg = BuildConfig(linkage="strider", module="single", servo="xl330_m288", robot=True)
    tmpl = template_for(cfg)
    design_side(tmpl, cfg)
    mech = fabricate(tmpl, cfg, 1.0)
    assert mech.meta["ties"] > 0
    assert mech.meta["tie_shims_mm"] == pytest.approx(3.0)



# -- round 2 -----------------------------------------------------------------------------


def test_the_engine_version_follows_code_not_docstrings():
    from spiderpig.design import _code_digest

    a = 'def f():\n    """Old words."""\n    return 1\n'
    assert _code_digest(a) == _code_digest(a.replace("Old words.", "New words."))
    assert _code_digest(a) != _code_digest(a.replace("return 1", "return 2"))


def test_the_rail_screw_reaches_through_its_nut_on_any_frame_plate():
    from spiderpig.construction.deck import NUT_DEPTH, RAIL_NUT_H, rail_screw_length

    for t in (2.032, 2.286, 3.175):
        assert rail_screw_length(t) >= t + NUT_DEPTH + RAIL_NUT_H
    assert rail_screw_length(2.032) == 8
    assert rail_screw_length(3.175) == 10


@pytest.mark.slow
@pytest.mark.parametrize("kw", [dict(linkage="trotbot_heel", module="single"),
                                dict(linkage="strider", module="double", crank="bolt_round")])
def test_a_round_crankpins_shims_are_its_take_up_and_ordered_exactly(kw):
    from spiderpig.config import BuildConfig
    from spiderpig.fabricate import fabricate, template_for
    from spiderpig.hardware.bom import SHIM_AS, bom_from_mechanism, shim_key

    cfg = BuildConfig(**kw)                     # the robot: its bodies are L./R. prefixed
    mech = fabricate(template_for(cfg), cfg, 1.0)
    shims = [b for b in mech.bodies if "crank_pin_shims_" in b.name]
    assert shims
    notes = mech.meta["crank_bolt"]["chains"]
    want = {c["at"]: c["shims_mm"] for c in notes if c.get("shims_mm")}
    for b in shims:
        tag = b.name.split("crank_pin_shims_")[1]
        assert abs(b.part.bounding_box().size.Z - want[tag]) <= 0.11
    bom = bom_from_mechanism(mech, group=False)
    assert not any(r.key in ("shim_din988_4x8", "shim_din988_6x12") for r in bom.purchased)
    # every crankpin's stack is ordered: its thicknesses add up to it, each on its own line
    for side in ("L.", "R."):
        for tag, mm in want.items():
            name = f"{side}crank_pin_shims_{tag} ("
            rows = [(r.key, w) for r in bom.purchased for w in r.where if w.startswith(name)]
            stack = [float(t) for t in rows[0][1][len(name):].split(" mm)")[0].split(" + ")]
            assert abs(sum(stack) - mm) <= 0.11, (side, tag, stack, mm)
            keys = {k for k, _ in rows}
            for t in stack:
                assert (SHIM_AS.get(shim_key("shim_din988_4x8", t), (shim_key(
                    "shim_din988_4x8", t),))[0]) in keys, (t, keys)


@pytest.mark.slow
def test_a_build_leaves_no_cut_or_print_files_from_before(tmp_path):
    from spiderpig import build

    stale = tmp_path / "laser" / "parts" / "Ponoko_acrylic_3mm" / "old_x9.dxf"
    stale.parent.mkdir(parents=True)
    stale.write_text("")
    (tmp_path / "print").mkdir()
    (tmp_path / "print" / "old.stl").write_text("")
    (tmp_path / "manifest.json").write_text('{"design": "an earlier export"}')
    assert build.main(["--linkage", "klann", "--module", "single", "--side-only",
                       "--out", str(tmp_path)]) == 0
    assert not stale.exists()
    assert not (tmp_path / "print" / "old.stl").exists()
    assert list((tmp_path / "laser" / "parts").rglob("*.dxf"))
    # (api.export reuses and clears by it): the build's own, naming its design
    import json

    from spiderpig import api
    from spiderpig.config import BuildConfig

    cfg = BuildConfig(linkage="klann", module="single", robot=False)
    got = json.loads((tmp_path / "manifest.json").read_text())["design"]
    assert got == api.resolve(api.spec_of(cfg), store=None).id


def test_a_stored_plan_that_ran_out_of_time_is_searched_again(tmp_path):
    import json

    from spiderpig import api
    from spiderpig.store import Store

    store = Store(tmp_path)
    heel = api.resolve({"kind": "walker", "linkage": {"key": "trotbot_heel",
                                                      "params": {"unit": 7}},
                        "legs": {"module": "single"},
                        "constructions": {"crank": "keyed", "pillar": "printed"}}, store)
    assert not api.plan(heel).ok
    path = store.report_path(heel.id, "plan")
    doc = json.loads(path.read_text())
    assert api._reuse_plan(api.load(heel.id, store)) is not None   # a real failure: cached
    for f in doc["failures"]:
        f["code"] = "no_plan_in_time"
    path.write_text(json.dumps(doc))
    assert api._reuse_plan(api.load(heel.id, store)) is None        # CPU time: not cached


def test_an_export_into_a_folder_another_design_wrote_last_is_not_reused(tmp_path):
    import json

    from spiderpig import api

    out = tmp_path / "out"
    out.mkdir()
    (out / "manifest.json").write_text(json.dumps({"design": "someone-else"}))
    assert api._manifest_design(out) == "someone-else"
    assert api._manifest_design(tmp_path) is None


def test_clearing_generated_files_keeps_the_users_own(tmp_path):
    from spiderpig.build import clear_generated

    (tmp_path / "laser" / "parts" / "x").mkdir(parents=True)
    (tmp_path / "laser" / "parts" / "x" / "a.dxf").write_text("")
    (tmp_path / "laser" / "notes.txt").write_text("mine")
    (tmp_path / "print").mkdir()
    (tmp_path / "print" / "a.stl").write_text("")
    clear_generated(tmp_path / "laser")
    clear_generated(tmp_path / "print")
    assert (tmp_path / "laser" / "notes.txt").read_text() == "mine"
    assert not (tmp_path / "laser" / "parts").exists()
    assert not (tmp_path / "print").exists()


def test_the_engine_digest_reads_a_source_in_its_declared_encoding():
    from spiderpig.design import _code_digest

    src = "# -*- coding: latin-1 -*-\ns = '\xe9'\n"
    assert _code_digest(src.encode("latin-1")) == _code_digest(src.encode("latin-1"))
    assert _code_digest(b"x = 1\n") == _code_digest("x = 1\n")


def test_a_made_to_length_pillar_is_bought_at_its_price_break():
    from spiderpig.hardware.catalog import get

    offer = get("pillar_shaft_6_m3_62.4").offer
    assert offer.buy(1) == (1, 14.97)
    assert offer.buy(4) == (5, 54.3)            # five at 10.86 cost less than four at 14.97
    assert offer.buy(6) == (10, 54.7)           # ten at 5.47 less than six at 10.86
    assert offer.buy(12) == (12, 65.64)


def test_a_report_behind_a_plan_that_ran_out_of_time_is_never_stored(tmp_path):
    import json

    from spiderpig import api
    from spiderpig.failure import Failure
    from spiderpig.store import Store

    store = Store(tmp_path)
    d = api.resolve({"kind": "walker", "linkage": {"key": "klann"},
                     "legs": {"module": "single", "sides": 1}}, store)
    late = Failure(stage="plan", code="no_plan_in_time", message="out of time")
    api._finish(d, "build", api.BuildReport(failures=[late], t=1.0), 0.0)
    assert store.read_report(d.id, "build") is None
    # one stored before this rule: not served
    path = store.report_path(d.id, "verify")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"stage": "verify", "design": d.id, "level": "quick",
                                "engine_version": d.engine_version, "ok": False,
                                "failures": [late.to_dict()], "rows": []}))
    from spiderpig.verify import VerifyReport

    back = api.load(d.id, store)
    assert api._cached(back, "verify", VerifyReport, level="quick") is None


def test_view_and_export_take_every_build_option_and_the_spec_keeps_link_sheets(tmp_path):
    import argparse

    from spiderpig import api
    from spiderpig.config import BuildConfig, add_build_args, add_design_args
    from spiderpig.store import Store
    from spiderpig.view import resolve_args

    ap = argparse.ArgumentParser()
    add_design_args(ap)
    add_build_args(ap)
    ap.add_argument("--side-only", action="store_true")
    ap.set_defaults(linkage=None, module=None, servo=None, pillar=None, pin=None, crank=None,
                    sheet=None, frame_sheet=None, heads=None)
    store = Store(tmp_path)
    assert resolve_args(ap.parse_args([]), store) is None
    c = resolve_args(ap.parse_args(["--frame-sheet", "al5052_3p2mm", "--crank-sheet",
                                    "al6061_3p2mm", "--heads", "gap", "--link-sheet",
                                    "b1=al6061_3p2mm"]), store).config
    assert (c.frame_sheet, c.crank_sheet, c.heads) == ("al5052_3p2mm", "al6061_3p2mm", "gap")
    assert c.link_sheets == (("b1", "al6061_3p2mm"),)
    assert resolve_args(ap.parse_args(["--linkage", "strider"]), store).config == BuildConfig()
    cfg = BuildConfig(link_sheets=(("b1", "al6061_3p2mm"),))
    assert api.resolve(api.spec_of(cfg), store).config == cfg


def test_every_bolt_crank_is_rated():
    from spiderpig.config import BuildConfig
    from spiderpig.construction import CRANKS
    from spiderpig.construction.crank import BoltCrank
    from spiderpig.strength import crank_capacity

    keys = [k for k, c in CRANKS.items() if isinstance(c, BoltCrank)]
    assert {"bolt", "bolt_round", "bolt_hub_screw", "bolt_unretained"} <= set(keys)
    for k in keys:
        assert crank_capacity({}, BuildConfig(crank=k)), k


def test_purchased_parts_are_massed_in_their_own_material():
    from spiderpig.hardware.mass import item_material

    assert item_material("gobilda_1501_10") == "aluminium"
    assert item_material("m3_round_standoff_ff_6") == "aluminium"
    assert item_material("hex_standoff_m3_5") is None                  # steel
    assert item_material("pillar_shaft_6_m3_62.4") is None             # steel
    assert item_material("m3_nylock") is None                          # a steel nut
    assert item_material("ptfe_washer_6x12x0p5") == "ptfe"


def test_a_horn_screws_shims_say_what_is_bought():
    from spiderpig.hardware.bom import shim_as_bought

    assert "DIN 433" in shim_as_bought("shim_din988_3x6", 1.0)
    assert shim_as_bought("shim_din988_3x6", 0.2) == "a 0.2 mm DIN 988 shim"


# -- round 5 -----------------------------------------------------------------------------


@pytest.mark.slow
def test_an_xl330_robots_centre_plates_can_be_cut():
    from spiderpig import manufacture
    from spiderpig.config import BuildConfig
    from spiderpig.fabricate import fabricate, template_for

    cfg = BuildConfig(servo="xl330_m288")
    mech = fabricate(template_for(cfg), cfg, 1.0)
    got = manufacture.check(mech, cfg.sheet, dxf=False)
    assert not [i for i in got["issues"] if i["level"] == "error"], got["issues"]


def test_the_bom_files_mark_the_shop_supplies_on_hand(tmp_path):
    import csv
    import json

    from spiderpig.hardware.bom import Bom, PurchaseRow

    def row(key, price):
        return PurchaseRow(key=key, name=key, category="misc", qty=1, where=["x"], vendor="v",
                           url="u", sku="s", pack_qty=1, packs=1, pack_price_usd=price,
                           verified=True, alternatives=[])
    bom = Bom(purchased=[row("m3_nut", 2.0), row("pla_filament", 29.99)], made=[])
    bom.write(tmp_path)
    with open(tmp_path / "bom.csv") as f:
        rows = {r["item"]: r for r in csv.DictReader(f)}
    assert rows["pla_filament"]["section"] == "on hand"
    assert rows["pla_filament"]["est_cost_usd"] == ""
    doc = json.loads((tmp_path / "bom.json").read_text())
    pla = next(r for r in doc["purchased"] if r["key"] == "pla_filament")
    assert pla["on_hand"]
    assert pla["cost_usd"] == 0.0
    assert doc["cost_usd"] == pytest.approx(sum(r["cost_usd"] or 0 for r in doc["purchased"]))


def test_the_kerf_notes_follow_the_files():
    from spiderpig.hardware.order import kerf_note

    assert "nominal size" in kerf_note("SendCutSend", {0.0})
    assert "offset twice" in kerf_note("SendCutSend", {0.1})
    assert "0.2 mm" in kerf_note("Ponoko", {0.2})
    assert "a kerf small" in kerf_note("Ponoko", {0.0})


def test_a_ptfe_liner_is_massed_as_ptfe():
    from types import SimpleNamespace

    from spiderpig import servos
    from spiderpig.hardware.mass import DENSITY, material_of

    body = SimpleNamespace(name="pin_J3_liner", fab="purchased", bom_key="ptfe_tube_3x4_1m",
                           sheet=None)
    assert material_of(body, "acrylic_3mm", None, servos.get("sts3215"))[:2] == (
        "ptfe", DENSITY["ptfe"])


def test_the_server_keeps_a_designs_materials_for_another_linkage(tmp_path):
    from spiderpig import api
    from spiderpig.config import BuildConfig
    from spiderpig.server import app as srv

    srv.configure(str(tmp_path), prebake_default=False)
    try:
        c = BuildConfig(frame_sheet="al5052_2p5mm", heads="gap", crank_sheet="al6061_3p2mm",
                        servo="xl430_w250")
        d = api.resolve(api.spec_of(c), str(tmp_path))
        got = srv._config_from_query({"design": d.id, "linkage": "klann"}, robot=True)
        assert (got.frame_sheet, got.heads, got.crank_sheet, got.servo) == (
            "al5052_2p5mm", "gap", "al6061_3p2mm", "xl430_w250")
    finally:
        srv.configure(None, prebake_default=False)


def test_the_server_bakes_in_a_worker_once_its_sources_changed(monkeypatch):
    from spiderpig.server import app as srv

    monkeypatch.setattr(srv, "_LOADED_AT", srv._sources_mtime() - 1.0)
    monkeypatch.delenv("SPIDERPIG_WORKERS", raising=False)
    assert srv._stale_code()
    monkeypatch.setenv("SPIDERPIG_WORKERS", "0")
    assert not srv._stale_code()


def test_a_worker_reads_the_callers_path_before_its_arguments(tmp_path):
    from spiderpig import workers
    from spiderpig.config import BuildConfig

    assert workers.submit(_config_key, BuildConfig()).result() == BuildConfig().key


def _config_key(config):
    return config.key


@pytest.mark.slow
def test_explain_strength_takes_a_mechanism(tmp_path):
    from spiderpig import explain

    assert explain.main(["--linkage", "hoecken", "--strength", "--no-sim",
                         "--store", str(tmp_path)]) in (0, None)


@pytest.mark.slow
def test_an_export_into_a_build_folder_leaves_no_other_designs_files(tmp_path):
    from spiderpig import api, build

    assert build.main(["--linkage", "klann", "--module", "single", "--side-only",
                       "--out", str(tmp_path)]) == 0
    assert (tmp_path / "ORDER.md").exists()
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}}, store=None)
    assert api.export(d, ["bom"], tmp_path).ok
    assert not (tmp_path / "ORDER.md").exists()
    assert not list(tmp_path.glob("laser/**/*.dxf"))          # the klann's cut files


@pytest.mark.slow
def test_the_nominal_mass_is_within_two_percent_of_the_built_robot():
    from spiderpig import walk
    from spiderpig.config import BuildConfig
    from spiderpig.fabricate import fabricate, template_for

    for cfg in (BuildConfig(), BuildConfig(linkage="klann_lego")):
        nom = walk.nominal_mass_breakdown(cfg, walk.side_legs(cfg))["total"]
        fab = sum(g for g, _ in walk.body_masses(fabricate(template_for(cfg), cfg, 1.0),
                                                 cfg).values())
        assert abs(nom - fab) / fab < 0.02, (cfg.linkage, nom, fab)


def test_the_protection_board_finds_its_place_under_the_deck_or_says_it_has_none():
    from spiderpig.construction.base import ConstructionError
    from spiderpig.construction.deck import _bms_place

    bms = {"length": 46.7, "width": 23.0}
    strap = (-40.2, -30.2, -12.1, 12.1)
    # the STS3215's bay: across it, where it always sat
    assert _bms_place(bms, -6.5, 25.07, -68.0, [strap]) == (23.0, 23.35, -6.5, 0.0)
    # a narrower bay: turned along it, clear of a nut in its way
    nut = (-20.0, -15.0, -12.0, -6.0)
    b_x, b_z, x1, z = _bms_place(bms, -6.5, 15.3, -68.0, [nut])
    assert (b_x, b_z) == (46.7, 11.5)
    assert z - b_z >= -6.0 or x1 - b_x >= -15.0 or x1 <= -20.0
    with pytest.raises(ConstructionError):
        _bms_place(bms, -6.5, 15.3, -68.0, [strap])      # (the strap's run blocks it)


@pytest.mark.slow
def test_an_export_into_its_own_build_folder_keeps_the_builds_files(tmp_path):
    from spiderpig import api, build
    from spiderpig.config import BuildConfig

    assert build.main(["--linkage", "klann", "--module", "single", "--side-only",
                       "--out", str(tmp_path)]) == 0
    parts = sorted(tmp_path.glob("laser/parts/**/*.dxf"))
    prints = sorted(tmp_path.glob("print/*.stl"))
    assert parts
    cfg = BuildConfig(linkage="klann", module="single", robot=False)
    d = api.resolve(api.spec_of(cfg), store=None)
    assert api.export(d, ["bom", "dxf"], tmp_path).ok
    assert (tmp_path / "ORDER.md").exists()                       # the same design's
    assert sorted(tmp_path.glob("laser/parts/**/*.dxf")) == parts
    assert sorted(tmp_path.glob("print/*.stl")) == prints


@pytest.mark.slow
def test_the_protection_board_is_clear_of_the_decks_wire_and_cable_tie_slots(robot):
    from spiderpig.construction.deck import CABLE_TIE_SLOT, WIRE_SLOT, WIRE_SLOT_Z

    mech = robot("quad")                     # the demo Klann robot
    bb = next(b for b in mech.bodies if b.name == "deck_bms").part.bounding_box()
    x_c = mech.meta["deck"]["x_c"]
    tie_dx = WIRE_SLOT[0] / 2 + 3.0 + CABLE_TIE_SLOT[0] / 2
    for z in (-WIRE_SLOT_Z, WIRE_SLOT_Z):
        over_x = x_c + tie_dx > bb.min.X and x_c - tie_dx < bb.max.X
        over_z = z + WIRE_SLOT[1] / 2 > bb.min.Z and z - WIRE_SLOT[1] / 2 < bb.max.Z
        assert not (over_x and over_z), (bb.min.X, bb.max.X, z)
