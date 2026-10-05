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
    assert build.main(["--linkage", "klann", "--module", "single", "--side-only",
                       "--out", str(tmp_path)]) == 0
    assert not stale.exists()
    assert not (tmp_path / "print" / "old.stl").exists()
    assert list((tmp_path / "laser" / "parts").rglob("*.dxf"))


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
