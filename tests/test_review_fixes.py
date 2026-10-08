"""Regressions from the adversarial review of 2026-10-05: the frame ties on every servo, shim
heights the BOM can't split, and Chicago pins whose lower spacer is thinner than a print."""

from __future__ import annotations

import pytest
from build123d import Location

from spiderpig.construction.chassis import _chain, _shims
from spiderpig.hardware.bom import BomLine, split_shims
from spiderpig.mechanism import Body
from spiderpig.shapes import disc
from tests import _api


@pytest.mark.parametrize(("span", "segs", "take"), [
    (32.0, [30], 2.0),          # the STS3215's chain
    (34.0, [15, 18], 1.0),      # the XL430's
    (23.0, [20], 3.0),          # the XL330's: no M3 pair fills it, 3 mm of shims
    (30.1, [30], 0.0),          # within the plates' 0.1 mm: no 0.1 mm shim
    (31.1, [30], 1.0),
    (30.6, [30], 0.5),
])
@pytest.mark.construction
def test_tie_chains_take_up_in_whole_steps(span, segs, take):
    got = _chain(span)
    assert got == (segs, take)
    assert sum(_shims(take)) == pytest.approx(take)
    assert all(s in (1.0, 0.5) for s in _shims(take))


def _ring(name: str, t: float) -> Body:
    part = (disc((0, 0), 3.0, 0.0, t) - disc((0, 0), 1.55, -1.0, t + 1.0)).moved(Location())
    return Body(name, part=part, fab="purchased", bom_key="shim_din988_3x6")


@pytest.mark.hardware
def test_a_shim_height_the_steps_cant_make_is_not_dropped():
    ok = _ring("L.tie_shims0", 1.5)                # 1.0 + 0.5: two DIN 125 lines worth
    lines, notes = split_shims([BomLine("shim_din988_3x6", 1, ok.name),
                                BomLine("shim_din988_3x6", 1, "frame tie 0, L")],
                               {ok.name: ok})
    assert not notes
    assert sum(x.qty for x in lines if x.key == "m3_washer") == 3   # 2 for 1 mm, 1 for 0.5
    thin = _ring("L.tie_shims1", 0.1)              # finer than 1.0/0.5: the family's 0.1
    lines, notes = split_shims([BomLine("shim_din988_3x6", 1, thin.name)], {thin.name: thin})
    assert not notes
    assert [x.key for x in lines] == ["shim_din988_3x6_t0p1"]
    odd = _ring("L.tie_shims2", 0.05)              # no shim makes 0.05 mm
    lines, notes = split_shims([BomLine("shim_din988_3x6", 1, odd.name)], {odd.name: odd})
    assert notes                                   # said, and kept as the family's line
    assert [x.key for x in lines] == ["shim_din988_3x6"]


@pytest.mark.construction
def test_an_xl330_robot_builds_its_frame_ties():
    from spiderpig.config import BuildConfig

    cfg = BuildConfig(linkage="strider", module="single", servo="xl330_m288", robot=True)
    mech = _api.fabricated(cfg)                  # fabricate(), from the test cache
    assert mech.meta["ties"] > 0
    assert mech.meta["tie_shims_mm"] == pytest.approx(3.0)



# -- round 2 -----------------------------------------------------------------------------


def test_the_engine_version_follows_code_not_docstrings():
    from spiderpig.design import _code_digest

    a = 'def f():\n    """Old words."""\n    return 1\n'
    assert _code_digest(a) == _code_digest(a.replace("Old words.", "New words."))
    assert _code_digest(a) != _code_digest(a.replace("return 1", "return 2"))


@pytest.mark.construction
def test_the_rail_screw_reaches_through_its_nut_on_any_frame_plate():
    from spiderpig.construction.deck import NUT_DEPTH, RAIL_NUT_H, rail_screw_length

    for t in (2.032, 2.286, 3.175):
        assert rail_screw_length(t) >= t + NUT_DEPTH + RAIL_NUT_H
    assert rail_screw_length(2.032) == 8
    assert rail_screw_length(3.175) == 10


@pytest.mark.hardware
@pytest.mark.slow
@pytest.mark.parametrize("kw", [{"linkage": "trotbot_heel", "module": "single"},
                                {"linkage": "strider", "module": "double", "crank": "bolt_round"}])
def test_a_round_crankpins_shims_are_its_take_up_and_ordered_exactly(kw):
    from spiderpig.config import BuildConfig
    from spiderpig.hardware.bom import SHIM_AS, bom_from_mechanism, shim_key

    cfg = BuildConfig(**kw)                     # the robot: its bodies are L./R. prefixed
    mech = _api.fabricated(cfg)                 # fabricate(), from the test cache
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
    from spiderpig.config import BuildConfig
    from spiderpig.store import Store

    # what its static failure's recommendation checks (unit 12), from the test cache
    _api.seed(BuildConfig(linkage="trotbot_heel", module="single",
                          proportions=(("unit", 12.0),)))
    store = Store(tmp_path)
    heel = api.resolve({"kind": "walker", "linkage": {"key": "trotbot_heel",
                                                      "params": {"unit": 7}},
                        "legs": {"module": "single"}}, store)
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


@pytest.mark.hardware
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


@pytest.mark.strength
def test_every_bolt_crank_is_rated():
    from spiderpig.config import BuildConfig
    from spiderpig.construction import CRANKS
    from spiderpig.construction.crank import BoltCrank
    from spiderpig.strength import crank_capacity

    keys = [k for k, c in CRANKS.items() if isinstance(c, BoltCrank)]
    assert set(keys) == {"bolt", "bolt_round"}         # every crank left (2026-10-07)
    for k in keys:
        assert crank_capacity({}, BuildConfig(crank=k)), k


@pytest.mark.hardware
def test_purchased_parts_are_massed_in_their_own_material():
    from spiderpig.hardware.mass import item_material

    assert item_material("gobilda_1501_10") == "aluminium"
    assert item_material("m3_round_standoff_ff_6") == "aluminium"
    assert item_material("hex_standoff_m3_5") is None                  # steel
    assert item_material("pillar_shaft_6_m3_62.4") is None             # steel
    assert item_material("m3_nut") is None                             # a steel nut
    assert item_material("ptfe_washer_6x12x0p5") == "ptfe"


@pytest.mark.hardware
def test_a_horn_screws_shims_say_what_is_bought():
    from spiderpig.hardware.bom import shim_as_bought

    assert "DIN 125" in shim_as_bought("shim_din988_3x6", 1.0)
    assert "DIN 433" in shim_as_bought("shim_din988_4x8", 1.0)
    assert shim_as_bought("shim_din988_3x6", 0.2) == "a 0.2 mm DIN 988 shim"


# -- round 5 -----------------------------------------------------------------------------


@pytest.mark.hardware
@pytest.mark.slow
def test_an_xl330_robots_centre_plates_can_be_cut():
    from spiderpig import manufacture
    from spiderpig.config import BuildConfig

    cfg = BuildConfig(servo="xl330_m288")
    mech = _api.fabricated(cfg)                  # fabricate(), from the test cache
    got = manufacture.check(mech, cfg.sheet, dxf=False)
    assert not [i for i in got["issues"] if i["level"] == "error"], got["issues"]


@pytest.mark.hardware
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


@pytest.mark.hardware
def test_the_kerf_notes_follow_the_files():
    from spiderpig.hardware.order import kerf_note

    assert "nominal size" in kerf_note("SendCutSend", {0.0})
    assert "offset twice" in kerf_note("SendCutSend", {0.1})
    assert "0.2 mm" in kerf_note("Ponoko", {0.2})
    assert "a kerf small" in kerf_note("Ponoko", {0.0})


@pytest.mark.hardware
def test_a_ptfe_part_is_massed_as_ptfe():
    """(On the PTFE tube liner of the removed ``ptfe`` pin before 2026-10-07; the PTFE
    washer the Chicago screw's catalog item keeps is the PTFE part left.)"""
    from types import SimpleNamespace

    from spiderpig import servos
    from spiderpig.hardware.mass import DENSITY, material_of

    body = SimpleNamespace(name="pin_J3_washer", fab="purchased",
                           bom_key="ptfe_washer_4x8x0p5", sheet=None)
    assert material_of(body, "acrylic_3mm", None, servos.get("sts3215"))[:2] == (
        "ptfe", DENSITY["ptfe"])


@pytest.mark.server
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


@pytest.mark.server
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


@pytest.mark.strength
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


@pytest.mark.linkage
@pytest.mark.slow
def test_the_nominal_mass_is_within_two_percent_of_the_built_robot():
    from spiderpig import walk
    from spiderpig.config import BuildConfig

    for cfg in (BuildConfig(), BuildConfig(linkage="klann_lego")):
        nom = walk.nominal_mass_breakdown(cfg, walk.side_legs(cfg))["total"]
        fab = sum(g for g, _ in walk.body_masses(_api.fabricated(cfg), cfg).values())
        assert abs(nom - fab) / fab < 0.02, (cfg.linkage, nom, fab)


@pytest.mark.construction
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


@pytest.mark.construction
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


# -- round 7 -----------------------------------------------------------------------------


@pytest.mark.hardware
@pytest.mark.slow
@pytest.mark.parametrize("kerf", [0.2, 0.5])
def test_the_hex_crank_plates_lay_out_at_ponokos_kerf(tmp_path, kerf):
    from build123d import Face

    from spiderpig.config import BuildConfig
    from spiderpig.layout import _outer, _wire_is_circle, offset_wires, save_sheets, section_of

    cfg = BuildConfig(robot=False)
    mech = _api.fabricated(cfg)                  # fabricate(), from the test cache
    assert save_sheets(mech, tmp_path / "s", kerf=kerf, default=cfg.sheet)
    plate = next(b for b in mech.bodies if b.name == "crank_plate2")
    wires = list(section_of(plate).wires())
    pocket = next(w for w in wires if w is not _outer(wires) and _wire_is_circle(w) is None)
    got = offset_wires(pocket, -kerf / 2)            # the hex and its dog-bone lobes, apart
    area = sum(Face(w).area for w in got)
    assert area < Face(pocket).area - 0.5 * kerf / 2 * pocket.length


@pytest.mark.slow
def test_an_export_is_not_reused_once_a_build_wrote_its_folder(tmp_path):
    import json

    from spiderpig import api, build
    from spiderpig.config import BuildConfig
    from spiderpig.store import Store

    cfg = BuildConfig(linkage="klann", module="single", robot=False)
    d = api.resolve(api.spec_of(cfg), Store(tmp_path / "store"))
    assert _api.built(d).ok                      # its build from the test cache
    out = tmp_path / "out"
    first = api.export(d, ["bom"], out)
    assert first.ok
    assert build.main(["--linkage", "klann", "--module", "single", "--side-only", "--kerf",
                       "0.1", "--no-dxf", "--out", str(out)]) == 0
    m = json.loads((out / "manifest.json").read_text())
    assert m["design"] != d.id                        # the kerf shapes the cut files: its own
    again = api.export(api.load(d.id, d.store), ["bom"], out)
    assert again.ok
    assert json.loads((out / "manifest.json").read_text())["formats"] == ["bom"]


# -- round 8 -----------------------------------------------------------------------------


@pytest.mark.hardware
@pytest.mark.parametrize("ccw", [True, False])
def test_a_grown_contour_that_falls_back_grows_every_edge(monkeypatch, ccw):
    import math

    from build123d import Edge, Face, Plane, Wire

    from spiderpig import layout

    def fails(self, *a, **k):
        raise RuntimeError("Unexpected result type")
    # a 10 x 6 slot (two lines, two half circles), either way round
    lines = [Edge.make_line((0, 0), (10, 0)), Edge.make_line((10, 6), (0, 6))]
    arcs = [Edge.make_circle(3, Plane(origin=(10, 3, 0)), start_angle=-90, end_angle=90),
            Edge.make_circle(3, Plane(origin=(0, 3, 0)), start_angle=90, end_angle=270)]
    wire = Wire([lines[0], arcs[0], lines[1], arcs[1]])
    if not ccw:
        wire = Wire([e.reversed() for e in reversed(wire.edges())])
    a, perim, g = Face(wire).area, wire.length, 0.25
    monkeypatch.setattr(Wire, "offset_2d", fails)
    (got,) = layout.offset_wires(wire, g)
    assert Face(got).area == pytest.approx(a + perim * g + math.pi * g * g, rel=1e-4)


def test_a_fit_of_zero_servo_screw_web_keeps_every_front_screw():
    from dataclasses import replace

    from spiderpig import api
    from spiderpig.config import BuildConfig, Params

    cfg = BuildConfig(params=replace(Params(), servo_screw_web_t=0.0))
    assert api.resolve(api.spec_of(cfg), store=None).config == cfg


# -- round 9 -----------------------------------------------------------------------------


@pytest.mark.strength
def test_a_crank_rider_is_rated_at_the_crank_bore():
    from spiderpig import strength
    from spiderpig.config import BuildConfig

    cfg = BuildConfig(linkage="jansen", module="double")
    bore = strength.rider_hole(cfg)
    assert bore > strength.LINK_HOLE                  # the hex crankpin's sleeve: 8.85 mm
    loads = {"source": "sim", "joints": [
        {"stem": "M", "links": ["b1_leg0"], "crank": True,
         "walk": {"n": 10.0}, "jam": {"n": 133.0}},
        {"stem": "B", "links": ["b1_leg0"], "crank": False,
         "walk": {"n": 10.0}, "jam": {"n": 133.0}}]}
    (row,) = [r for r in strength.link_rows(cfg, loads) if r["joint"] == "link:b1"]
    w, t = 2 * cfg.params.link_radius, cfg.pitch
    assert row["jam"]["stress_mpa"] == pytest.approx(strength.LINK_KT * 133.0 / ((w - bore) * t),
                                                     abs=0.01)


def test_the_mcp_plan_output_carries_every_plan_report_field():
    from dataclasses import fields

    from spiderpig.api import PlanReport
    from spiderpig.mcp.outputs import PlanOut

    have = set(PlanOut.__annotations__) | {"ok", "failures"}
    for k in PlanOut.__mro__:
        have |= set(getattr(k, "__annotations__", {}))
    assert {f.name for f in fields(PlanReport)} <= have


def test_the_mcp_build_job_reports_its_own_failed_build(tmp_path, monkeypatch):
    from spiderpig import api
    from spiderpig.config import BuildConfig
    from spiderpig.failure import Failure
    from spiderpig.mcp.jobs import run_op
    from tests import cache

    # a store holding the Klann single side's build (the test cache's prebuilt store)
    store = cache.prebuilt_store(BuildConfig(linkage="klann", module="single", robot=False),
                                 tmp_path)
    (design_id,) = store.ids()
    assert run_op(str(store.root), "build", design_id, {"t": 1.0})["ok"]   # stored
    late = Failure(stage="plan", code="no_plan_in_time", message="out of time")
    monkeypatch.setattr(api, "build", lambda d, t=1.0, force=False:
                        api.BuildReport(failures=[late], t=t, ok=False))
    got = run_op(str(store.root), "build", design_id, {"t": 1.0})
    assert not got["ok"]
    assert [f["code"] for f in got["failures"]] == ["no_plan_in_time"]


@pytest.mark.strength
def test_the_anchor_both_plates_fix_is_the_two_plate_beam():
    from spiderpig import strength
    from spiderpig.config import BuildConfig

    cfg = BuildConfig(robot=False)
    note = _api.fabricated(cfg).meta["wobble"]["pillar:J2_leg0"]       # from the test cache
    loads = {"walk_n": 60.0, "jam_n": 155.0, "source": "family"}
    both = strength.joint_strength("pillar:J2_leg0", note, loads)
    cant = dict(note, anchors=[0])
    row = strength.joint_strength("pillar:J2_leg0", cant, loads)
    (fix,) = [f for f in strength.fixes(row, cant, loads, cfg) if "both frame plates" in f]
    assert f"jam SF {both['jam']['safety']:g}" in fix


# -- round 10 ----------------------------------------------------------------------------


@pytest.mark.strength
def test_every_crank_states_the_bore_its_riders_are_cut_to():
    from spiderpig import strength
    from spiderpig.config import BuildConfig

    assert strength.rider_hole(BuildConfig()) == pytest.approx(8.85)          # the hex pin
    # the round standoff's 6 mm, the riders turning on it at the running fit (0.35)
    assert strength.rider_hole(BuildConfig(crank="bolt_round")) == pytest.approx(6.35)


def test_a_range_target_is_scored_against_the_bound_it_misses():
    from spiderpig.spec import Target

    t = Target(min=100.0, max=1000.0)
    assert t.scale_at(66.9) == 100.0
    assert t.scale_at(1200.0) == 1000.0
    assert Target(value=50.0, tol=1.0).scale_at(10.0) == 50.0


@pytest.mark.sim
def test_the_phase_lock_steers_about_the_revolution_it_is_locked_at():
    import math

    import numpy as np

    from spiderpig.sim.run import PhaseLock

    lock = PhaseLock(5.0, max_offset=0.3)
    lock.ref[:] = [6 * math.pi + 10.0, 10.0]          # three revolutions apart, after a spin
    lock.ctrl(np.array([5.0, 5.0]), np.array([6 * math.pi + 10.0, 10.0]), 0.01)   # walking
    lock.ctrl(np.array([5.0, 4.5]), np.array([6 * math.pi + 10.0, 10.0]), 0.01)   # steering
    d = lock.ref[0] - lock.ref[1]
    assert abs(d - 6 * math.pi) <= 0.3 + 1e-9         # not unwound to zero


def test_a_construction_change_is_patched_under_constructions():
    from spiderpig import linkage
    from spiderpig.failure import Recommendation
    from spiderpig.stack import Recommendation as EngineRec

    rec = EngineRec((("crank", "bolt", "bolt_round"),), why="w")
    got = Recommendation.from_engine(rec, linkage.get("strider"))    # (its ``crank`` param)
    assert got.patch == {"constructions": {"crank": "bolt_round"}}


def test_a_value_target_under_the_scale_is_met_from_the_band():
    from spiderpig import api

    st = api.measure_config(api.resolve({"kind": "mechanism",
                                         "linkage": {"key": "hoecken"}}, None).config)
    stroke = st["motion.stroke_mm"]
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"},
                     "motion": {"stroke_mm": {"value": round(0.8 * stroke, 1),
                                              "tol": round(0.016 * stroke, 2)}}}, None)
    assert api.advise(d).recommendations


# -- round 11 ----------------------------------------------------------------------------


@pytest.mark.hardware
def test_stock_cut_to_length_is_bought_in_whole_pieces():
    """(The rod pivots' stock bound, ``max_stack``, went with them on 2026-10-07.)"""
    from spiderpig.hardware.bom import CutList

    # three 60 mm pieces don't come out of two 100 mm rods (1.8 rods by length)
    assert CutList("rod_3mm_100", "rod", ((60.0, 3),), 100.0).stock_pieces() == 3
    assert CutList("rod_3mm_100", "rod", ((40.0, 4),), 100.0).stock_pieces() == 2


@pytest.mark.strength
def test_the_crank_advice_names_what_would_hold_it():
    """A weak hex crank: the torque limit that holds it and a thicker crank sheet (deeper
    pockets) where one is stocked; a weak round-standoff clamp: the clamp's screws. (Before
    2026-10-07 it named the hex crank for the keyed one.)"""
    from spiderpig import strength
    from spiderpig.config import BuildConfig

    row = {"kind": "crank", "factor": 1.0, "capacity_nm": {"hex": 0.5},
           "weakest": "hex 5.5 AF in its plate's pocket", "construction": "bolt",
           "jam": {"torque_nm": 0.85, "safety": 0.5}}
    got = strength.fixes(row, None, {}, BuildConfig())
    assert got[0].startswith("set the servo's torque limit to 0.25 N·m")   # 0.5 / (SF 2 x 1)
    assert any("--crank-sheet al6061_3p2mm" in f for f in got)
    thick = strength.fixes(row, None, {}, BuildConfig(crank_sheet="al6061_3p2mm"))
    assert not any("--crank-sheet" in f for f in thick)                   # already the thickest
    clamp = dict(row, construction="bolt_round", capacity_nm={"clamp": 0.5},
                 weakest="web clamped on the standoff")
    got = strength.fixes(clamp, None, {}, BuildConfig(crank="bolt_round"))
    assert any("threadlocker on the crankpin screws" in f for f in got)
    assert not any("--crank-sheet" in f for f in got)


@pytest.mark.slow       # a store's STEP round trip and a recheck: 4-8 s, its parts cached
def test_an_export_after_an_accepted_edit_is_written_afresh(tmp_path, monkeypatch):
    import numpy as np
    from build123d import Cylinder

    from spiderpig import api

    monkeypatch.setattr(api.building, "fabricate_at", _api.fabricate_from_cache)   # cached parts
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}}, None)
    assert api.build(d).ok
    first = api.export(d, ["dxf"], tmp_path)
    name = next(n for n, p in d.parts.items() if p.group == "links")
    part, body = d.parts[name], d.mech.body(name)
    a, b = ((body.pose @ j.pose).matrix[:2, 3] for j in body.joints[:2])
    part.solid = part.solid - Cylinder(1.0, 10).moved(part.locate((np.asarray(a) + b) / 2))
    assert api.recheck(d).ok
    again = api.export(d, ["dxf"], tmp_path)
    assert again is not first


def test_scale_advice_keeps_a_met_target_met():
    from spiderpig import api

    base = api.measure_config(api.resolve({"kind": "mechanism",
                                           "linkage": {"key": "hoecken"}}, None).config)
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}, "motion": {
        "stroke_mm": {"min": round(1.3 * base["motion.stroke_mm"], 2)},
        "straightness_mm": {"max": round(1.1 * base["motion.straightness_mm"], 4)}}}, None)
    rep = api.advise(d)
    assert not rep.recommendations
    assert any("together" in n for n in rep.notes)


def test_a_pre_build_estimate_doesnt_fail_a_hard_target(tmp_path):
    from spiderpig import api

    d = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"},
                     "size": {"mass_g": {"max": 120}}}, tmp_path)
    rep = api.verify(d, "quick")
    row = next(r for r in rep.rows if r.requirement == "size.mass_g")
    assert row.tier == "estimated"
    assert not row.hard


# -- round 12 ----------------------------------------------------------------------------


@pytest.mark.construction
def test_two_pillars_get_their_chord_either_side_of_the_crank():
    from spiderpig.construction.plates import chords

    assert len(chords((0, 0), [(10, 1), (10, -1)])) == 1
    assert len(chords((0, 0), [(-10, 1), (-10, -1)])) == 1             # across +-180 deg
    assert chords((0, 0), [(10, 0), (-10, 0)]) == []                   # 180 deg apart


@pytest.mark.construction
def test_a_foot_at_a_links_corner_gets_no_sock():
    from types import SimpleNamespace

    from spiderpig.construction.plates import foot_links

    lk = SimpleNamespace(feet=[("b6", "F"), ("b4", "G")])
    topo = SimpleNamespace(links={"b6": [("C", "E"), ("E", "F"), ("F", "C")],
                                  "b4": [("D", "G")]})
    assert foot_links(topo, lk) == [("b4", "G", "D")]


def test_a_hard_target_only_estimated_is_unverified_and_unscored():
    from spiderpig import api
    from spiderpig.config import BuildConfig

    _api.seed(BuildConfig())                    # the Strider double's plan, cached
    d = api.resolve({"kind": "walker", "linkage": {"key": "strider"},
                     "size": {"mass_g": {"max": 50}},
                     "motion": {"stride_mm": {"min": 10, "hard": False}}}, store=None)
    rep = api.verify(d, "quick")
    assert "size.mass_g" in rep.unverified
    assert rep.score == 1.0                        # the soft stride only
    row = next(r for r in rep.rows if r.requirement == "size.mass_g")
    assert row.score is None
    assert not row.hard


@pytest.mark.slow       # a store's STEP round trip and a recheck: 4-8 s, its parts cached
def test_an_edited_handles_export_stays_off_the_store(tmp_path, monkeypatch):
    import numpy as np
    from build123d import Cylinder

    from spiderpig import api
    from spiderpig.store import Store

    monkeypatch.setattr(api.building, "fabricate_at", _api.fabricate_from_cache)   # cached parts
    store = Store(tmp_path)
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}}, store)
    assert api.build(d).ok
    name = next(n for n, p in d.parts.items() if p.group == "links")
    part, body = d.parts[name], d.mech.body(name)
    a, b = ((body.pose @ j.pose).matrix[:2, 3] for j in body.joints[:2])
    part.solid = part.solid - Cylinder(1.0, 10).moved(part.locate((np.asarray(a) + b) / 2))
    assert api.recheck(d).ok
    edited = api.export(d, ["bom"])
    assert "edited" in edited.out_dir
    back = api.load(d.id, store)
    plain = api.export(back, ["bom"])
    assert plain.out_dir != edited.out_dir
    assert plain.manifest["mass_g"] != edited.manifest["mass_g"]


# -- round 13 ----------------------------------------------------------------------------


@pytest.mark.slow       # a store's STEP round trip and a recheck: 4-8 s, its parts cached
def test_a_rejected_edit_leaves_the_built_parts_in_the_mechanism(tmp_path, monkeypatch):
    from build123d import Box, Location

    from spiderpig import api
    from spiderpig.store import Store

    monkeypatch.setattr(api.building, "fabricate_at", _api.fabricate_from_cache)   # cached parts
    store = Store(tmp_path)
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}}, store)
    assert api.build(d).ok
    name = next(n for n, p in d.parts.items() if p.group == "links")
    part = d.parts[name]
    part.solid = part.solid + Box(200, 200, 2).moved(Location(part.solid.bounding_box().center()))
    assert not api.recheck(d).ok
    assert d.mech.body(name).part is part.built        # what export and verify read
    assert not d.edited
    assert store.read_report(d.id, "recheck") is None  # handle-local edits: not stored


@pytest.mark.slow
def test_verify_keeps_an_edited_handles_crank_angle_and_edits(monkeypatch):
    import numpy as np
    from build123d import Cylinder

    from spiderpig import api

    monkeypatch.setattr(api.building, "fabricate_at", _api.fabricate_from_cache)   # cached parts
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}}, None)
    assert api.build(d, 0.5).ok
    name = next(n for n, p in d.parts.items() if p.group == "links")
    part, body = d.parts[name], d.mech.body(name)
    a, b = ((body.pose @ j.pose).matrix[:2, 3] for j in body.joints[:2])
    part.solid = part.solid - Cylinder(1.0, 10).moved(part.locate((np.asarray(a) + b) / 2))
    edited = part.volume_mm3
    assert api.recheck(d).ok
    api.verify(d, "standard")
    assert d.build_t == 0.5
    assert d.parts[name].volume_mm3 == pytest.approx(edited)
    assert api.build(d, 1.0).ok
    assert not d.edited                                 # a fresh build: the edits are gone


# -- round 14 ----------------------------------------------------------------------------


@pytest.mark.slow       # a store's STEP round trip and a recheck: 4-8 s, its parts cached
def test_an_accepted_edit_is_graded_and_forgotten_as_itself(tmp_path, monkeypatch):
    import numpy as np
    from build123d import Cylinder

    from spiderpig import api
    from spiderpig.store import Store

    monkeypatch.setattr(api.building, "fabricate_at", _api.fabricate_from_cache)   # cached parts
    store = Store(tmp_path)
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}}, store)
    assert api.build(d).ok
    plate = d.parts["crank_plate2"]
    bb = plate.solid.bounding_box()
    c = np.array([bb.center().X, bb.center().Y])
    hole = Cylinder(0.25, 50).moved(plate.locate(c))
    plate.solid = plate.solid - hole                     # a 0.5 mm hole: under the minimum
    assert api.recheck(d).ok
    assert not d.reports["build"].cut_rules["ok"]       # graded as edited, not as built
    assert api.recheck(d).ok                            # a second recheck of the edits:
    assert store.read_report(d.id, "recheck") is None   # still not the store's
    api.verify(d, "quick")
    assert api.build(d, 1.0, force=True).ok             # a fresh build: the edits gone
    assert "verify" not in d.reports                    # and what was verified of them


def test_a_recheck_that_raises_leaves_the_built_parts_in_the_mechanism():
    from types import SimpleNamespace

    from spiderpig import api

    built = object()
    body = SimpleNamespace(part=None)
    mech = SimpleNamespace(body=lambda n: body)
    part = SimpleNamespace(solid="not a shape", built=built, edited=True)
    d = SimpleNamespace(mech=mech, parts={"b1": part}, side=None)
    with pytest.raises(TypeError):
        api.recheck(d)
    assert body.part is None                            # nothing taken before the check


# -- round 15 ----------------------------------------------------------------------------


@pytest.mark.planner
def test_the_gap_fallback_says_what_it_tried():
    from spiderpig.stack import Claim, PlanReject, StackSpec, Unbuildable, _thicker_gaps

    def make(L):          # a stock part that fits only once gap 3 is a whole 1 mm thicker
        if L.gap(3) < 2.0 - 1e-6:
            raise Unbuildable("misses its stock length")
        return []

    err = PlanReject("crank: misses its stock length")
    err.claim = Claim("crank", frozenset(), make)
    assert _thicker_gaps(err, StackSpec(), {}, 10, {}, {3: 1.0}, {}, set())[3] == \
        pytest.approx(2.0)
    with pytest.raises(PlanReject, match="least thickenings of one or two gaps"):
        _thicker_gaps(err, StackSpec(), {}, 10, {}, dict.fromkeys(range(1, 7), 1.0), {}, set())


@pytest.mark.slow       # a store's STEP round trip and a recheck: 4-8 s, its parts cached
def test_an_edited_export_into_a_folder_is_never_reused(tmp_path, monkeypatch):
    import json

    import numpy as np
    from build123d import Cylinder

    from spiderpig import api
    from spiderpig.store import Store

    monkeypatch.setattr(api.building, "fabricate_at", _api.fabricate_from_cache)   # cached parts
    store, out = Store(tmp_path / "s"), tmp_path / "out"
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}}, store)
    assert api.export(d, ["bom"], out).ok
    name = next(n for n, p in d.parts.items() if p.group == "links")
    part, body = d.parts[name], d.mech.body(name)
    a, b = ((body.pose @ j.pose).matrix[:2, 3] for j in body.joints[:2])
    part.solid = part.solid - Cylinder(1.0, 10).moved(part.locate((np.asarray(a) + b) / 2))
    assert api.recheck(d).ok
    assert api.export(d, ["bom"], out).ok
    assert json.loads((out / "manifest.json").read_text())["edited"]
    fresh = api.load(d.id, store)
    api.export(fresh, ["bom"], out)
    assert not fresh.log[-1]["cached"]
    assert not json.loads((out / "manifest.json").read_text())["edited"]


def test_a_build_at_another_angle_forgets_the_other_angles_export(tmp_path, monkeypatch):
    from spiderpig import api

    monkeypatch.setattr(api.building, "fabricate_at", _api.fabricate_from_cache)   # cached parts
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}}, None)
    assert api.build(d, 1.0).ok
    first = api.export(d, ["bom"], tmp_path)
    assert api.build(d, 2.0).ok
    again = api.export(d, ["bom"], tmp_path)
    assert again is not first
    assert again.manifest["t_ref"] == 2.0
