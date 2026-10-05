"""The agent-facing surface (:mod:`spiderpig`): the Spec validates and resolves to a stable
design id, the operations return reports (failures as data, never raised), the harness
verifies with tiers, parts expose live solids and ``recheck`` catches an edited one."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest
from build123d import Box, Location

from spiderpig import api
from spiderpig.config import BuildConfig
from spiderpig.failure import Failure, apply_patch, parse_blocker
from spiderpig.spec import TARGET_FIELDS, Spec, SpecErrors, Target, spec_schema, validate

# The numbers below are the keyed crank's and the printed pillars' (the defaults before the
# bolt crank and the standoff pillars of 2026-10-03): the specs pin them, so a design's
# height, parts and cost stay what these tests check
OLD = {"constructions": {"crank": "keyed", "pillar": "printed"}}
KLANN_QUAD = {"kind": "walker", "linkage": {"key": "klann"}}      # the default quad


def _errors(doc: dict) -> dict[str, list]:
    out: dict[str, list] = {}
    for e in validate(doc):
        out.setdefault(e.path, []).append(e)
    return out


# ---------------------------------------------------------------------------
# Spec validation
# ---------------------------------------------------------------------------


def test_unknown_fields_are_rejected_with_paths_and_nearest_keys():
    errs = _errors({"kind": "walker", "linkage": {"key": "klann", "params": {"oa": 60}},
                    "motion": {"stride": {"min": 100}}, "colour": "red",
                    "legs": {"modul": "quad"}})
    assert errs["colour"][0].message == "unknown field"
    assert errs["linkage.params.oa"][0].nearest == "OA"
    assert errs["motion.stride"][0].nearest == "stride_mm"
    assert "stride_mm" in errs["motion.stride"][0].allowed
    assert errs["legs.modul"][0].nearest == "module"
    assert set(errs) == {"colour", "linkage.params.oa", "motion.stride", "legs.modul"}


def test_out_of_vocabulary_keys_name_the_allowed_values():
    errs = _errors({"kind": "walker", "linkage": {"key": "klan"},
                    "materials": {"servo": "sg90", "sheet": "acrylic"},
                    "constructions": {"pin": "bearings"}, "outputs": ["step", "pdf"]})
    assert errs["linkage.key"][0].nearest == "klann"
    assert "klann" in errs["linkage.key"][0].allowed
    assert errs["materials.servo"][0].allowed == ("sts3215", "xl330_m288", "xl430_w250")
    assert errs["materials.sheet"][0].nearest == "acrylic_3mm"
    assert errs["constructions.pin"][0].nearest == "bearing"
    assert errs["outputs[1]"][0].allowed == ("step", "stl", "print", "dxf", "bom", "glb", "mjcf")


def test_wildcards_are_rejected():
    errs = _errors({"kind": "walker", "linkage": {"key": "any"}, "materials": {"servo": "*"}})
    assert "wildcard" in errs["linkage.key"][0].message
    assert "wildcard" in errs["materials.servo"][0].message


def test_kind_module_phases_sides_and_targets_are_checked_against_the_linkage():
    errs = _errors({"kind": "mechanism", "linkage": {"key": "klann"},
                    "legs": {"module": "hex", "phases_deg": [0, 90], "sides": 3},
                    "size": {"stack_mm": 30}, "motion": {"bob_mm": {"min": 1},
                                                          "stack_mm": {"max": 1},
                                                          "stroke_mm": {"min": 1, "max": 0}}})
    assert "klann is a walker, not a mechanism" in errs["linkage.key"][0].message
    assert errs["legs.module"][0].allowed == ("single", "double", "decker", "quad")
    assert "4 legs per side, got 2" in errs["legs.phases_deg"][0].message
    assert errs["legs.sides"][0].allowed == (1, 2)
    assert "not a bare number" in errs["size.stack_mm"][0].message
    assert "bob_mm is a metric of a walker, not a mechanism" in errs["motion.bob_mm"][0].message
    assert "put it under size" in errs["motion.stack_mm"][0].message
    assert "min 1 is above max 0" in errs["motion.stroke_mm"][0].message
    with pytest.raises(SpecErrors) as e:
        Spec.from_dict({"kind": "walker"})
    assert [x.path for x in e.value.errors] == ["linkage"]


def test_hard_and_soft_defaults_and_explicit_flips():
    spec = Spec.from_dict({**KLANN_QUAD,
                           "motion": {"stride_mm": {"min": 90}, "bob_mm": {"max": 30, "hard": True},
                                      "ground_clearance_mm": {"min": 20}},
                           "size": {"stack_mm": {"max": 40, "hard": False}},
                           "budget": {"cost_usd": {"max": 100}}})
    hard = {f.path: t.hard for f, t in spec.targets()}
    assert hard == {"motion.stride_mm": False, "motion.bob_mm": True,
                    "motion.ground_clearance_mm": True, "size.stack_mm": False,
                    "budget.cost_usd": True}
    assert not TARGET_FIELDS["motion"]["speed_mm_s"].hard
    assert TARGET_FIELDS["size"]["mass_g"].hard
    assert TARGET_FIELDS["motion"]["speed_mm_s"].tier == "estimated"
    assert Spec.from_dict(spec.to_dict()) == spec


def test_target_semantics():
    assert Target(min=90).check(102.4) == (True, 0.0)
    assert Target(max=40).check(36.0) == (True, 0.0)
    met, miss = Target(min=150).check(88.7)
    assert not met
    assert miss == pytest.approx(61.3)
    assert Target(value=20).check(20.9) == (True, 0.0)            # within 5 %
    assert Target(value=20, tol=0.5).check(20.9)[1] == pytest.approx(0.4)
    assert Target(min=1, max=2).describe() == "1..2"


def test_schema_is_json_and_lists_the_vocabularies():
    schema = spec_schema()
    text = json.dumps(schema)
    assert schema["required"] == ["kind", "linkage"]
    assert schema["properties"]["linkage"]["properties"]["key"]["enum"][0] == "strider"
    assert schema["properties"]["size"]["properties"]["stack_mm"]["properties"]["hard"]["default"]
    assert not schema["properties"]["motion"]["properties"]["bob_mm"]["properties"]["hard"][
        "default"]
    assert "kerf_mm" in schema["properties"]["fit"]["properties"]
    assert "additionalProperties" in text


# ---------------------------------------------------------------------------
# resolve
# ---------------------------------------------------------------------------


def test_resolve_infers_the_rest_and_the_id_is_stable_with_defaults_dropped():
    a = api.resolve(KLANN_QUAD)
    b = api.resolve({"kind": "walker", "linkage": {"key": "klann", "params": {"OA": 60}},
                     "legs": {"module": "quad", "sides": 2, "phases_deg": [0, 180, 90, 270]},
                     "materials": {"sheet": "acrylic_3mm", "servo": "sts3215"},
                     "constructions": {"pillar": "standoff", "pin": "chicago", "crank": "bolt"},
                     "fit": {"link_radius": 6.0}})
    assert a.id == b.id == api.resolve(KLANN_QUAD).id
    assert len(a.id) == 16
    assert a.config.is_default
    assert a.config.key == "klann_quad_robot"
    r = a.resolved
    assert r["legs"] == {"module": "quad", "phases_deg": [0.0, 180.0, 90.0, 270.0], "sides": 2}
    assert r["linkage"]["params"]["OA"] == 60.0
    assert r["materials"]["pitch_mm"] == 3.0
    assert r["fit"]["link_radius"] == 6.0
    assert r["fit"]["kerf_mm"] is None          # each sheet's service kerf (layout.sheet_kerf)
    assert r["outputs"] == ["step", "stl", "print", "dxf", "bom"]
    assert a.engine_version.startswith("0.1.0+")
    c = api.resolve({**KLANN_QUAD, "linkage": {"key": "klann", "params": {"OA": 50}}})
    assert c.id != a.id
    assert c.config.proportions == (("OA", 50.0),)
    m = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}})
    assert m.resolved["legs"] == {"module": "single", "phases_deg": [0.0], "sides": 1}
    assert not m.config.robot
    with pytest.raises(SpecErrors):
        api.resolve({"kind": "walker", "linkage": {"key": "hoecken"}})


def test_linkage_cards():
    keys = [c["key"] for c in api.list_linkages("walker")]
    assert keys[0] == "strider"
    assert "klann" in keys
    assert "hoecken" not in keys
    card = api.describe("klann")
    assert card["scale_params"] == ["OA"]
    assert card["modules"]["quad"]["legs"] == 4
    assert card["foot_path"]["lift_mm"] == pytest.approx(86.5, abs=0.1)
    assert all(c["margin_mm"] > 0 for c in card["closures"])
    with pytest.raises(KeyError, match="did you mean 'klann'"):
        api.describe("klan")


# ---------------------------------------------------------------------------
# The default Klann quad
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def quad(design):
    """The default quad's handle; its plan is the session's (``design_side`` is cached per
    template and config)."""
    design("quad")
    return api.resolve(KLANN_QUAD)


def test_check_plan_walk_on_the_default_quad(quad):
    cr = api.check(quad)
    assert cr.ok
    assert cr.failures == []
    assert [s["point"] for s in cr.steps if s["kind"] == "closure"] == ["C", "E"]
    assert cr.foot_path["lift_mm"] == pytest.approx(86.5, abs=0.1)
    assert cr.drive == {"servo": "sts3215", "rpm_max": 52.0, "inputs": ["t"]}
    assert cr.ground_clearance_mm == pytest.approx(64.15, abs=0.05)
    assert cr.crank_facts["hosts"]["b1_leg0"] == ["M_leg0"]
    pr = api.plan(quad)
    assert pr.ok
    # the bolt crank's single aluminium webs (2026-10-04): a one-layer run per crankpin, the
    # screw heads in clearance gaps (11 of them), 0.080 in frame plates: 14 layers, 77.164
    # mm (the two-plate stacks took 31 layers, 95.975 mm; the keyed crank's were 16)
    assert pr.n_layers == 14
    # the hex-standoff crankpins on 0.100 in 6061 webs (2026-10-04): 76.789 mm (77.164 on
    # the round standoff); 74.289 with the hub chain capped (no screw over the hub plate);
    # 73.789 since a hex pin's upper stack uses the air over its plate (2026-10-05)
    assert pr.height_mm == pytest.approx(73.789)
    assert len(pr.gaps_mm) == 9       # the hex crank's, hub chain capped (10 with the
    #                                   screw over the hub plate; the round standoff's: 11)
    assert pr.route == {"runs": [{"at": f"M_leg{k}", "lo": lo, "hi": lo}
                                 for k, lo in ((0, 4), (2, 6), (1, 8), (3, 10))],
                        "bearing": True}
    assert pr.layers["b1_leg0"] == 4
    assert "inner frame plate" in pr.table
    wr = api.walk(quad)
    assert wr.ok
    assert wr.feet_z_planned
    assert wr.mass_nominal
    assert wr.metrics["stride_mm"] == pytest.approx(102.4, abs=0.1)
    assert api.recommend(quad) == []
    assert "3. plan" in api.explain(quad)
    ops = [e["op"] for e in quad.log]     # the handle is the module's: other tests of it log
    assert {"check", "plan", "walk"} <= set(ops)   # their ops too, in xdist's order
    json.dumps(quad.to_dict())     # every report is JSON-able


def test_verify_quick_passes_with_tiers(quad):
    rep = api.verify(quad, "quick")
    assert rep.ok
    assert rep.score == 1.0
    assert rep.failures == []
    rows = {r.requirement: r for r in rep.rows}
    assert rows["program.loops_close"].tier == "proven"
    assert rows["program.loops_close"].passed
    assert rows["size.stack_mm"].value == pytest.approx(73.789)     # the hex crank's
    assert rows["size.stack_mm"].tier == "proven"
    assert rows["motion.speed_mm_s"].tier == "estimated"
    assert rows["motion.stride_mm"].tier == "measured"
    assert rows["size.mass_g"].tier == "estimated"
    assert rows["motion.ground_clearance_mm"].value == pytest.approx(64.15, abs=0.05)
    assert rep.unverified == []
    json.dumps(rep.to_dict())


def test_a_hard_size_miss_fails_and_the_same_target_soft_only_lowers_the_score():
    hard = api.resolve({**KLANN_QUAD, **OLD, "size": {"stack_mm": {"max": 30}},
                        "motion": {"stride_mm": {"min": 90}}})
    rep = api.verify(hard, "quick")
    assert not rep.ok
    row = next(r for r in rep.rows if r.requirement == "size.stack_mm")
    assert not row.passed
    assert row.hard
    # 17 layers: the keyed crank's top webs (0.080 in frame plates; 3.175 mm ones: 16, 49.05)
    assert row.value == pytest.approx(49.764)
    assert row.target == "<= 30"
    assert rep.score == 1.0                        # the stride, the only soft target, is met
    soft = api.resolve({**KLANN_QUAD, **OLD, "size": {"stack_mm": {"max": 30, "hard": False}},
                        "motion": {"stride_mm": {"min": 90}}})
    rep = api.verify(soft, "quick")
    assert rep.ok
    row = next(r for r in rep.rows if r.requirement == "size.stack_mm")
    assert not row.passed
    assert not row.hard
    assert row.score == pytest.approx(0.3412)      # 49.764 mm against 30: 1 - 19.764 / 30
    assert rep.score == pytest.approx(0.6706)      # the mean of stride 1.0 and stack 0.3412
    assert hard.id != soft.id


# ---------------------------------------------------------------------------
# Failures as data
# ---------------------------------------------------------------------------


def test_the_heel_at_the_drawings_unit_fails_the_static_stage_with_a_patch_that_plans():
    heel = api.resolve({"kind": "walker", "linkage": {"key": "trotbot_heel",
                                                      "params": {"unit": 7}},
                        "legs": {"module": "single"}, **OLD})      # the keyed crank's post
    cr = api.check(heel)
    assert not cr.ok
    (f,) = cr.failures
    assert (f.stage, f.code) == ("static", "link_no_layer")
    assert f.culprits[0]["body"] == "b7"
    assert f.culprits[0]["point"] == "J1"
    assert f.numbers["dist_mm"] == pytest.approx(6.8, abs=0.05)
    assert f.numbers["need_mm"] == 11.25          # the keyed crank's 8.5 mm post: 4.25 + 6 + 1
    (rec,) = f.recommendations
    assert rec.patch == {"linkage": {"params": {"unit": 12.0}}}
    assert rec.changes == [{"name": "unit", "before": 7.0, "after": 12.0}]
    # 15 layers on the 0.080 in frame plates of 2026-10-04 (14 on 0.125 in)
    assert rec.verified.startswith("checked: the static stage passes, and it plans in 15 ")
    assert api.recommend(heel) == [rec]
    assert api.plan(heel).failures == cr.failures
    rep = api.verify(heel, "quick")
    assert not rep.ok
    assert rep.failures[0].code == "link_no_layer"
    fixed = api.resolve(apply_patch(heel.spec.to_dict(), rec.patch))
    assert fixed.resolved["linkage"]["params"]["unit"] == 12.0
    assert fixed.id != heel.id
    pr = api.plan(fixed)
    assert pr.ok
    assert pr.n_layers == 15


def test_a_broken_loop_is_a_program_failure_with_its_numbers():
    d = api.resolve({**KLANN_QUAD, "linkage": {"key": "klann", "params": {"MC": 0.3}}})
    cr = api.check(d)
    (f,) = cr.failures
    assert (f.stage, f.code) == ("program", "loop_cannot_close")
    assert f.culprits == [{"joint": "C", "refs": ["A", "M"]}]
    assert f.numbers["margin_mm"] < 0
    assert f.numbers["fails_deg"] is not None
    assert api.walk(d).failures[0].code == "linkage_invalid"
    rep = api.verify(d, "quick")
    assert not rep.ok
    assert not next(r for r in rep.rows
                                    if r.requirement == "program.loops_close").passed


def test_exceptions_map_to_stages():
    from spiderpig import linkage
    from spiderpig.config import ParamError
    from spiderpig.construction.base import ConstructionError
    from spiderpig.stack import PlanError

    assert Failure.from_exception(ParamError("x")).stage == "spec"
    assert Failure.from_exception(linkage.OutputError("k: y")).code == "promise_broken"
    assert Failure.from_exception(linkage.AssemblyError("k: joint C can't be placed")
                                  ).culprits == [{"joint": "C"}]
    f = Failure.from_exception(ConstructionError("x: the drive turns one input"))
    assert (f.stage, f.code) == ("drive", "second_input_no_drive")
    assert Failure.from_exception(ConstructionError("too thin"), stage="fabricate").stage == \
        "fabricate"
    f = Failure.from_exception(PlanError("j: no layer plan found with up to 5 layers after "
                                         "60000 search steps",
                                         ["  12 x pin:D head vs b2: -8.7 mm apart in one "
                                          "layer, need 1.0"]))
    assert f.code == "no_plan_in_budget"
    assert f.numbers == {"search_steps": 60000}
    assert f.blockers == [{"count": 12, "a": "pin:D head", "b": "b2", "gap_mm": -8.7,
                           "need_mm": 1.0, "text": "12 x pin:D head vs b2: -8.7 mm apart in "
                                                   "one layer, need 1.0"}]
    assert parse_blocker("  3 x crank route: no crank route passes")["why"] == \
        "crank route: no crank route passes"
    assert Failure.from_exception(ValueError("sheet packing dropped ['L.torso']")
                                  ).culprits == [{"body": "L.torso"}]
    assert Failure.from_exception(KeyError("no catalog item 'm9'")).code == "unknown_catalog_key"
    json.dumps(f.to_dict())


# ---------------------------------------------------------------------------
# Solids, recheck, the standard level and export
# ---------------------------------------------------------------------------


def test_parts_expose_live_solids_and_recheck_passes(quad, robot):
    rep = api.attach_build(quad, robot("quad", 1.0), 1.0)
    assert rep.ok
    assert rep.n_parts == sum(1 for b in robot("quad", 1.0).bodies if b.part is not None)
    assert rep.counts["laser"] > 0
    assert rep.mass_g == pytest.approx(sum(p.mass_g for p in quad.parts.values()), abs=0.01)
    b1 = quad.parts["L.b1_leg0"]
    assert b1.solid is robot("quad", 1.0).body("L.b1_leg0").part
    assert (b1.group, b1.side, b1.fab, b1.material) == ("links", "L", "laser", "sheet")
    assert b1.layers == (4,)
    assert not b1.edited
    assert quad.parts["R.b1_leg0"].layers == (4,)
    assert quad.parts["R.b1_leg0"].side == "R"
    assert quad.parts["L.servo"].group == "drive"
    assert quad.parts["L.servo"].mass_g == 55.0
    standoff = next(k for k in quad.parts if k.startswith("L.pillar_A_leg0_standoff"))
    assert quad.parts[standoff].group == "pillar:A_leg0"
    plate = next(k for k in quad.parts if k.startswith("L.crank_plate"))
    assert quad.parts[plate].group == "crank"
    assert quad.parts[plate].fab == "laser"
    assert quad.parts["centre_plate0"].group == "chassis"
    assert quad.parts["centre_plate0"].side \
        is None
    rr = api.recheck(quad)      # nothing edited: solids and clashes only, no mutation
    assert rr.ok
    assert rr.edited == []
    assert rr.checked == []
    assert rr.clashes == []
    assert quad.parts["L.b1_leg0"].solid is robot("quad", 1.0).body("L.b1_leg0").part


@pytest.mark.slow
def test_an_edited_solid_that_leaves_its_claim_or_clashes_is_caught():
    d = api.resolve({"kind": "walker", "linkage": {"key": "klann"},
                     "legs": {"module": "single", "sides": 1}})
    assert api.build(d).ok
    plan = d.side.plan
    z0, z1 = plan.z(plan.layers["b2"])
    b2 = d.parts["b2"]
    bb = b2.solid.bounding_box()
    c = ((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2)
    # a boss on the link reaching up through the next three layers: outside its claim
    b2.solid = b2.solid + Box(12, 12, 4 * (z1 - z0)).moved(Location((*c, z0 + 2 * (z1 - z0))))
    assert b2.edited
    rr = api.recheck(d)
    assert not rr.ok
    assert rr.edited == ["b2"]
    assert rr.checked == ["b2"]
    assert rr.contract[0]["part"] == "b2"
    assert rr.contract[0]["mm3_outside"] > 1000
    assert [f.code for f in rr.failures] == ["part_outside_claim"]
    # the link one layer thicker: it runs into the shoulders holding it and its neighbours
    b2.solid = b2.built + Box(bb.size.X, bb.size.Y, z1 - z0).moved(
        Location((*c, z1 + (z1 - z0) / 2)))
    rr = api.recheck(d)
    assert not rr.ok
    assert {f.code for f in rr.failures} >= {"parts_clash"}
    assert any(c["a"] == "b2" or c["b"] == "b2" for c in rr.clashes)
    b2.solid = b2.built                      # restored: the guarantee holds again
    assert api.recheck(d).ok
    assert not b2.edited
    b2.solid = "not a solid"
    with pytest.raises(TypeError, match="build123d Shape"):
        api.recheck(d)


@pytest.mark.slow
def test_verify_standard_passes_on_the_default_quad(quad, robot):
    api.attach_build(quad, robot("quad", 1.0), 1.0)
    rep = api.verify(quad, "standard")
    assert rep.ok
    assert rep.failures == []
    rows = {r.requirement: r for r in rep.rows}
    assert rows["plan.verified"].tier == "proven"
    assert rows["plan.verified"].value == 0
    assert rows["contract@t=0"].passed
    assert rows["contract@t=3.2"].passed
    assert rows["clash@t=1"].value == 0
    assert rows["solids@t=1"].value == 0
    assert rows["size.mass_g"].tier == "measured"
    assert rows["size.mass_g"].value > 400
    assert rows["size.envelope_z_mm"].tier == "measured"
    assert rows["budget.sheets"].value >= 1
    assert rows["budget.cost_usd"].tier == "estimated"
    assert rows["budget.print_g"].value > 0


@pytest.mark.slow
def test_export_writes_what_the_cli_writes(tmp_path):
    d = api.resolve({"kind": "walker", "linkage": {"key": "klann"},
                     "legs": {"module": "single", "sides": 1},
                     "outputs": ["step", "dxf", "bom"]})
    rep = api.export(d, out_dir=tmp_path)
    assert rep.ok
    names = {str(p.relative_to(tmp_path)) for p in tmp_path.rglob("*") if p.is_file()}
    # a set per service and sheet, each part on its thinnest (2026-10-04): the links in
    # acrylic (Ponoko), the
    # frame plates 0.080 in 5052, the crank's webs 0.100 in 6061 (the hex crankpins'
    # pockets), the foot link 6061
    assert {"klann.step", "laser/klann_sheet_Ponoko_acrylic_3mm_0.dxf",
            "laser/klann_sheet_SendCutSend_al5052_2mm_0.dxf",
            "laser/klann_sheet_SendCutSend_al6061_2p5mm_0.dxf",
            "laser/klann_sheet_SendCutSend_al6061_3p2mm_0.dxf", "laser/klann_sheet_parts.csv",
            "bom.csv",
            "bom.md", "bom.json", "manifest.json"} <= names
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert manifest["design"] == d.id
    assert manifest["plan"]["layers"] == 10          # the single-plate bolt crank's (keyed: 8)
    assert manifest["bom"]["items"] > 0
    # the bolt crank's webs, standoff crankpin, screws, shims and stub, the standoff
    # pillars' segments, sleeves, screws and washers, the Chicago pins (keyed and printed:
    # 36; the two-plate bolt crank's 13 layers had 58); with the hex-standoff crankpins
    # (2026-10-04) 75 (the round standoff's: 64); 72 with the hub chain capped (no screw,
    # washer or collar over the hub plate); 71 since the second assembly audit (the
    # stub's thrust sleeve; the STS3215's two near front screws left out); 69 since the hex
    # crank's gap rules of 2026-10-05 (another 10-layer layering: fewer pivot washers/shims)
    assert len(manifest["parts"]) == 69
    with pytest.raises(ValueError, match="unknown formats"):
        api.export(d, ["pdf"], tmp_path)


# ---------------------------------------------------------------------------
# Test drive, round 1 (docs/agentlib/TESTDRIVE.md): what the reports must say
# ---------------------------------------------------------------------------

KLANN_SINGLE = {"kind": "walker", "linkage": {"key": "klann"},
                "legs": {"module": "single", "sides": 1}, **OLD}


def test_a_walkers_card_says_which_modules_walk_and_how_each_parameter_moves_the_foot():
    card = api.describe("klann")
    mods = card["modules"]
    assert mods["quad"]["walks"]
    assert mods["quad"]["stride_mm"] > 50
    assert not mods["double"]["walks"]
    assert mods["double"]["stride_mm"] < 1
    sens = card["sensitivity"]
    assert set(sens) == {p["name"] for p in card["params"]}
    oa = sens["OA"]                              # the scale parameter: everything grows 10 %
    assert oa["step"] == "+10%"
    assert all(oa[k] == pytest.approx(10.0, abs=0.5)
               for k in ("lift_mm", "height_mm", "width_mm"))
    assert sens["angA"]["step"] == "+5°"
    assert sens["MC"] is None                    # +10 % of MC: the loops no longer close
    assert api.describe("strider")["modules"]["double"]["walks"]
    # a mechanism's card has its own sensitivity: the output's numbers (round 3)
    assert "stroke_mm" in api.describe("hoecken")["sensitivity"]["unit"]
    text = json.dumps(card)
    assert json.loads(text)["sensitivity"] == card["sensitivity"]
    assert "NaN" not in text                     # a null, never NaN, where a loop can't close


def test_a_module_with_no_net_travel_says_why_and_which_module_walks():
    d = api.resolve({"kind": "walker", "linkage": {"key": "klann"},
                     "legs": {"module": "double"}}, store=None)
    w = api.walk(d)
    assert w.ok
    assert w.metrics["stride_mm"] < 1
    (note,) = w.notes
    assert note.startswith("no net travel: klann's double module")
    assert "quad (" in note
    assert "describe(linkage)" in note
    stride = next(r for r in w.rows if r.requirement == "motion.stride_mm")
    speed = next(r for r in w.rows if r.requirement == "motion.speed_mm_s")
    assert stride.detail == speed.detail == note
    assert api.no_travel_note(d.config, {"stride_mm": 100.0}) is None


def test_check_names_the_body_part_that_sets_the_ground_clearance_and_z_counts_the_heads():
    d = api.resolve(KLANN_SINGLE, store=None)
    cr = api.check(d)
    assert cr.ok
    assert cr.lowest_body_part.startswith("the ")                    # entry 7
    rep = api.verify(d, "quick")
    gc = next(r for r in rep.rows if r.requirement == "motion.ground_clearance_mm")
    assert cr.lowest_body_part in gc.detail
    z = next(r for r in rep.rows if r.requirement == "size.envelope_z_mm")
    assert "axle heads outside" in z.detail                            # entry 4
    x = next(r for r in rep.rows if r.requirement == "size.envelope_x_mm")
    assert "sweep over the cycle" in x.detail
    assert all(isinstance(w, str) for w in api.plan(d).warnings)


def test_explain_prints_a_recorded_failure_without_solving_or_advising_again(monkeypatch):
    heel = api.resolve({"kind": "walker", "linkage": {"key": "trotbot_heel",
                                                      "params": {"unit": 7}},
                        "legs": {"module": "single"}})
    assert not api.plan(heel).ok
    import spiderpig.fabricate as fabricate
    import spiderpig.recommend as recommend

    def never(*a, **k):
        raise AssertionError("the engine ran again")

    monkeypatch.setattr(fabricate, "design_side", never)
    monkeypatch.setattr(recommend, "recommend", never)
    text = api.explain(heel)                                           # entry 10
    assert "STOP:" in text
    assert "b7" in text
    assert "what would clear it" in text          # the recorded failure's recommendations


def test_export_returns_a_prior_export_that_covers_the_formats(tmp_path):
    from spiderpig.store import Store

    store = Store(tmp_path / "store")
    d = api.resolve(KLANN_SINGLE, store=store)
    out = tmp_path / "out"
    out.mkdir()
    files = [out / "klann.step", out / "bom.csv", out / "manifest.json"]
    for f in files:
        f.write_text("x")
    prior = api.ExportReport(out_dir=str(out.resolve()), formats=["step", "bom"],
                             files=[str(f) for f in files])
    store.write_report(d, "export", prior)
    rep = api.export(d, ["bom"], out)                                   # entry 19
    assert rep.formats == ["step", "bom"]
    assert rep.files == prior.files
    assert d.mech is None                                              # served, not built


def test_bad_solids_accepts_a_purchased_compound_of_valid_solids():
    from types import SimpleNamespace

    from build123d import Compound

    from spiderpig.construction.contract import bad_solids

    two = Compound([Box(1, 1, 1), Box(1, 1, 1).moved(Location((3, 0, 0)))])
    assert len(two.solids()) == 2
    mech = SimpleNamespace(bodies=[
        SimpleNamespace(name="servo", part=two, fab="purchased"),      # entry 14
        SimpleNamespace(name="link", part=two, fab="laser"),
        SimpleNamespace(name="pin", part=Box(1, 1, 1), fab="printed"),
        SimpleNamespace(name="none", part=None, fab="laser")])
    (bad,) = bad_solids(mech)
    assert (bad["part"], bad["solids"], bad["fab"]) == ("link", 2, "laser")


def test_the_bake_tessellates_face_by_face():
    from spiderpig.bake import _tessellate

    pos, tri = _tessellate(Box(2, 3, 4))
    assert pos.shape == (24, 3)
    assert tri.shape == (36,)
    assert pos.dtype.name == "float32"
    assert tri.dtype.name == "uint32"
    assert tri.max() == 23


def test_recommend_offers_the_default_scale_when_a_scaled_down_plan_ran_out_of_room(
        monkeypatch):
    from spiderpig import recommend as rec
    from spiderpig.config import BuildConfig

    checked = "checked: the static stage passes, and it plans in 16 layers (48 mm)"
    monkeypatch.setattr(rec, "_verify", lambda config, plan, deadline=None: checked)
    monkeypatch.setattr(rec, "_DONE", {})
    small = BuildConfig(linkage="jansen", module="quad", proportions=(("unit", 1.1),))
    recs, notes = rec.recommend(small, plan=True)                     # entry 9
    r, printed = recs           # the scale first; printed pillars (the standoffs' rings fill)
    assert printed.changes == (("pillar", "standoff", "printed"),)
    assert r.changes == (("unit", 1.1, 1.6),)
    assert r.verified == checked
    assert "default scale" in r.why
    assert notes == []
    recs, notes = rec.recommend(BuildConfig(linkage="jansen", module="quad"), plan=True)
    assert [r.changes for r in recs] == [(("pillar", "standoff", "printed"),)]   # no scale
    assert notes == []                        # a lever left: printed pillars
    recs, notes = rec.recommend(BuildConfig(linkage="jansen", module="quad", pillar="printed"),
                                plan=True)
    assert recs == []
    assert "stack's own room" in notes[0]
    assert "levers left" in notes[0]
    assert rec.recommend(small) == ([], [])          # a static failure without gaps: nothing


def test_capture_warnings_collects_the_constructions_warnings_once():
    import logging

    with api.capture_warnings() as seen:
        logging.getLogger("spiderpig.construction.printed").warning("pin seg0: strains 6 %")
        logging.getLogger("spiderpig.construction.printed").warning("pin seg0: strains 6 %")
        logging.getLogger("spiderpig.construction.printed").info("not a warning")
        logging.getLogger("spiderpig.other").warning("not collected")
    assert seen == ["pin seg0: strains 6 %"]                            # entry 18


# ---------------------------------------------------------------------------
# Test drive, round 2 (docs/agentlib/TESTDRIVE.md): the thin sheet, the budget, the handles
# ---------------------------------------------------------------------------


THIN = {"kind": "walker", "linkage": {"key": "klann"}, "legs": {"module": "single", "sides": 1},
        "materials": {"thickness_mm": 2}, **OLD}


def test_a_thin_sheet_warns_and_the_crank_names_the_least_pitch_with_a_checked_patch():
    thin = api.resolve(THIN, store=None)                                   # entry 2
    (warning,) = [w for w in thin.warnings if w.startswith("materials.")]
    assert warning.startswith("materials.thickness_mm 2 is 33% under acrylic_3mm's nominal 3 mm")
    cr = api.check(thin)
    assert not cr.ok
    (f,) = cr.failures
    assert (f.stage, f.code) == ("construction", "unbuildable")
    assert f.message.startswith("no M3 screw, nut and 4 mm hex key fit a keyed crankpin joint "
                                "in 2 mm layers: ")
    assert "the least layer pitch that fits is 3 mm" in f.message
    assert "materials.thickness_mm 2 -> 3, or a thicker sheet" in f.message    # 3 is nominal
    assert f.numbers == {"pitch_mm": 2, "least_pitch_mm": 3.0}
    (rec,) = f.recommendations
    assert rec.patch == {"materials": {"thickness_mm": 3.0}}
    assert rec.changes == [{"name": "thickness_mm", "before": 2, "after": 3.0}]
    assert rec.why.startswith("the keyed crank's crankpin joints need layers of at least 3 mm")
    # 8 layers (22.239 mm) on the 0.080 in frame plates of 2026-10-04 (7 on 0.125 in)
    assert rec.verified.startswith("checked: the static stage passes, and it plans in 8 layers")
    assert api.recommend(thin) == [rec]
    assert api.plan(thin).failures == cr.failures
    rep = api.verify(thin, "quick")
    rows = {r.requirement: r for r in rep.rows}
    assert not rep.ok
    assert rows["drive.one_servo"].passed                      # the drive is fine: the crank isn't
    assert not rows["construction.buildable"].passed
    assert rows["construction.buildable"].value == "unbuildable"
    fixed = api.derive(thin, rec.patch, store=None)
    assert [w for w in fixed.warnings if w.startswith("materials.")] == []
    assert api.check(fixed).ok
    assert api.plan(fixed).n_layers == 8
    near = api.resolve({**THIN, "materials": {"thickness_mm": 3.2}}, store=None)
    assert [w for w in near.warnings if w.startswith("materials.")] == []


def test_a_thickness_at_the_nominal_or_as_an_int_is_the_same_design():
    plain = api.resolve(KLANN_QUAD, store=None)                            # entry 12
    nominal = api.resolve({**KLANN_QUAD, "materials": {"thickness_mm": 3}}, store=None)
    assert nominal.id == plain.id
    assert nominal.resolved["materials"]["thickness_mm"] is None
    thin = api.resolve({**KLANN_QUAD, "materials": {"thickness_mm": 2}}, store=None)
    assert thin.id == api.resolve({**KLANN_QUAD, "materials": {"thickness_mm": 2.0}},
                                  store=None).id
    assert thin.resolved["materials"]["thickness_mm"] == 2.0
    assert thin.config.pitch == 2.0


def test_the_least_pitch_of_the_printed_crank():
    from spiderpig.construction.crank import PrintedCrank

    crank = PrintedCrank()
    assert crank.least_pitch(2.0) == 2.9
    assert crank.least_pitch(3.0) == 3.0
    assert crank.least_pitch(2.0, limit=2.5) is None


def test_the_module_error_says_legs_per_side():
    errs = _errors({"kind": "walker", "linkage": {"key": "klann"}, "legs": {"module": "hex"}})
    (e,) = errs["legs.module"]                                             # entry 1
    assert "a module is the legs per side" in e.message
    assert "quad 4 a side (8 on the robot)" in e.message
    assert "no linkage has a three-leg module" in e.message
    assert e.allowed == ("single", "double", "decker", "quad")


def _bom(*rows):
    from spiderpig.hardware.bom import Bom, PurchaseRow

    out = []
    for key, qty, pack_qty, price in rows:
        packs = -(-qty // pack_qty)
        out.append(PurchaseRow(key=key, name=key.replace("_", " "), category="misc", qty=qty,
                               where=[], vendor="Shop", url="", sku="", pack_qty=pack_qty,
                               packs=packs, pack_price_usd=price, verified=True,
                               alternatives=[]))
    return Bom(purchased=out, made=[])


def test_the_cost_row_cannot_pass_a_max_on_a_lower_bound_and_names_every_unpriced_item():
    from spiderpig.verify import cost_row

    bom = _bom(("servo", 2, 1, 25.0), ("bearing_mf63zz", 64, 10, None),
               ("m3_nut", 8, 100, None))
    hard = api.resolve({**KLANN_QUAD, "budget": {"cost_usd": {"max": 100}}}, store=None)
    row = cost_row(hard, bom)                                              # entry 3
    assert row.value == 50.0
    assert row.hard
    assert not row.passed
    assert row.detail.startswith("at least; the target can't be verified while items are unpriced")
    assert "2 unpriced, so the total is a lower bound: 64 x bearing mf63zz (7 packs of 10 at " \
           "Shop); 8 x m3 nut (1 pack of 100 at Shop)" in row.detail
    soft = api.resolve({**KLANN_QUAD, "budget": {"cost_usd": {"max": 100, "hard": False}}},
                       store=None)
    row = cost_row(soft, bom)
    assert row.passed
    assert not row.hard
    assert row.detail.startswith("at least (the verdict is on the priced part)")
    floor = api.resolve({**KLANN_QUAD, "budget": {"cost_usd": {"min": 40}}}, store=None)
    assert cost_row(floor, bom).passed                     # a lower bound can confirm a floor
    over = cost_row(hard, _bom(("servo", 2, 1, 60.0), ("m3_nut", 8, 100, None)))
    assert not over.passed
    assert over.detail.startswith("2 items")
    priced = cost_row(hard, _bom(("servo", 2, 1, 25.0), ("m3_nut", 8, 100, 2.0)))
    assert priced.passed
    assert "unpriced" not in priced.detail


def test_verify_quick_prices_a_floor_from_the_catalog():
    from spiderpig.verify import cost_floor

    one = api.resolve({"kind": "walker", "linkage": {"key": "klann"},
                       "legs": {"module": "single", "sides": 1},
                       "budget": {"cost_usd": {"max": 200}}, **OLD}, store=None)
    total, priced, unpriced = cost_floor(one)                              # entry 4
    from spiderpig import servos
    from spiderpig.hardware.catalog import get as item

    servo = item(servos.get("sts3215").bom_key).offer.price_usd
    glue, nuts = 13.99, 2.39     # round 4: a bottle of CA for the anchors, the crank's nuts
    # a blank of each aluminium sheet: the frame (0.080 in, 2026-10-04), a Klann's feet
    al, al6061 = 18.0, 32.0
    assert total == pytest.approx(servo + 25.49 + 10.99 + al + al6061 + glue + nuts, abs=0.01)
    assert unpriced == ["Two-part slow-cure structural epoxy (e.g. J-B Weld Original or "
                        "Loctite EA E-30CL), 2 x 25 ml",          # the Chicago barrels' bond
                        "M3 x 4 mm brass hex standoff, female-female, 5.0 mm A/F",
                        "Low-strength threadlocker (Loctite 222 or equivalent), 10 ml"]
    assert priced[0].startswith("Feetech STS3215")
    rep = api.verify(one, "quick")
    rows = {r.requirement: r for r in rep.rows}
    assert rows["budget.cost_floor_usd"].value == total
    assert rows["budget.cost_floor_usd"].detail.startswith("before a build, from the catalog: ")
    assert "counted after a build (verify standard)" in rows["budget.cost_floor_usd"].detail
    assert rep.unverified == ["budget.cost_usd"]
    assert rep.ok
    robot = api.resolve({**KLANN_QUAD, **OLD, "materials": {"servo": "xl330_m288"},
                         "budget": {"cost_usd": {"max": 100}}}, store=None)
    total, priced, _ = cost_floor(robot)
    xl330 = item(servos.get("xl330_m288").bom_key).offer.price_usd
    # (no acrylic cement since the glue-free joinery of 2026-10-04: it was $12.84)
    assert total == pytest.approx(2 * xl330 + 25.49 + 10.99 + al + al6061 + 11.37
                                  + glue + nuts, abs=0.01)
    rep = api.verify(robot, "quick")
    rows = {r.requirement: r for r in rep.rows}
    assert "budget.cost_floor_usd" not in rows
    assert rows["budget.cost_usd"].value == total
    assert not rows["budget.cost_usd"].passed
    assert rows["budget.cost_usd"].detail.startswith("a lower bound already over the target")
    assert not rep.ok


def test_the_recommendation_says_which_module_it_checked():
    from spiderpig import recommend as rec
    from spiderpig.config import BuildConfig

    text = rec._verify(BuildConfig(linkage="klann", module="quad", robot=False),
                       plan=False)                                                # entry 5
    # the bolt crank's single webs, heads in gaps, 0.080 in frame plates (2026-10-04); the
    # hex crankpins' screw stacks in their gaps: 39.339 mm (the round standoff's 34.939);
    # 36.839 with the hub chain capped (no screw over the hub plate, 2026-10-04); 37.439 since
    # the hex crank's gap rules of 2026-10-05 (another 10-layer layering found first)
    assert text.startswith("checked: the static stage passes, and its single module plans in "
                           "10 layers (37.439 mm); the quad module's own plan is not checked "
                           "here")
    assert "the planner's deadline is 60 s" in text
    assert rec._verify(BuildConfig(linkage="klann", module="single", robot=False), plan=False) \
        == "checked: the static stage passes, and it plans in 10 layers (37.439 mm)"


def test_a_parts_mass_and_volume_follow_its_edited_solid(quad, robot):
    api.attach_build(quad, robot("quad", 1.0), 1.0)
    part = quad.parts["L.b2_leg0"]                                         # entry 7
    mass, volume = part.mass_g, part.volume_mm3
    assert part.density == pytest.approx(1.19, abs=0.05)
    assert mass == pytest.approx(volume / 1000 * part.density)
    try:
        c = part.solid.center()
        part.solid = part.solid - Box(4, 4, 20).moved(Location((c.X, c.Y, c.Z)))
        assert part.edited
        assert part.volume_mm3 < volume - 40
        assert part.mass_g == pytest.approx(part.volume_mm3 / 1000 * part.density)
        assert part.to_dict()["mass_g"] == pytest.approx(part.mass_g, abs=1e-3)
    finally:
        part.solid = part.built                       # the session's robot: never mutated
    assert part.mass_g == mass
    assert part.volume_mm3 == volume
    servo = quad.parts["L.servo"]
    assert servo.fixed_mass_g == 55.0
    assert servo.mass_g == 55.0


@pytest.mark.slow
def test_a_reloaded_build_carries_the_links_joints_and_outlines_and_takes_the_example_edit(
        tmp_path):
    from build123d import Cylinder, Location

    spec = {"kind": "walker", "linkage": {"key": "klann"},
            "legs": {"module": "single", "sides": 1}}
    first = api.resolve(spec, store=tmp_path)
    assert api.build(first).ok
    again = api.load(first.id, store=tmp_path)
    rep = api.build(again)                                                 # entry 6
    assert rep.ok
    assert again.log[-1]["cached"]
    body, fresh = again.mech.body("b2"), first.mech.body("b2")
    assert body.outline == fresh.outline != ()
    assert [j.name for j in body.joints] == [j.name for j in fresh.joints]
    link = again.parts["b2"]
    a, b = (body.joint(j).pose.matrix[:2, 3] for j in body.outline[0])
    z = again.side.plan.z(link.layers[0])
    mass, total = link.mass_g, rep.mass_g
    link.solid = link.solid - Cylinder(1.5, 10).moved(Location((*((a + b) / 2), sum(z) / 2)))
    assert link.mass_g < mass
    rr = api.recheck(again)
    assert rr.ok
    assert rr.edited == ["b2"]
    assert rr.checked == ["b2"]
    assert again.reports["build"].mass_g == pytest.approx(
        sum(p.mass_g for p in again.parts.values()), abs=0.01)
    assert again.reports["build"].mass_g < total


def test_list_designs_cards_say_what_a_design_is_made_of(tmp_path):
    base = {"kind": "walker", "linkage": {"key": "strider", "params": {"unit": 6.3}},
            "legs": {"module": "double"}, "materials": {"servo": "xl330_m288"},
            "budget": {"cost_usd": {"max": 120}}}
    a = api.resolve({**base, "constructions": {"pin": "bearing", "pillar": "bearing"}},
                    store=tmp_path)
    b = api.resolve({**base, "constructions": {"pin": "bushing", "pillar": "bushing"}},
                    store=tmp_path)
    cards = {c["id"]: c for c in api.list_designs(store=tmp_path)}         # entry 9
    assert cards[a.id]["constructions"] == {"pillar": "bearing", "pin": "bearing",
                                            "crank": "bolt", "heads": "best"}
    assert cards[b.id]["constructions"]["pin"] == "bushing"
    for c in (cards[a.id], cards[b.id]):
        assert c["servo"] == "xl330_m288"
        assert c["sheet"] == "acrylic_3mm"
        assert c["thickness_mm"] is None
        assert c["params"] == {"unit": 6.3}
        assert c["targets"] == {"budget": ["cost_usd"]}


@pytest.mark.slow
def test_export_reports_the_bakes_warnings(quad, robot, tmp_path, monkeypatch):
    import logging

    api.attach_build(quad, robot("quad", 1.0), 1.0)

    def fake_bake(path, cfg, profile=False, **kw):         # fabricated=, side=
        logging.getLogger("bake_gltf").warning("tessellate: 3 of 511 faces have no "
                                               "triangulation; skipped")
        Path(path).write_bytes(b"glTF")

    monkeypatch.setattr("spiderpig.bake.bake_gltf", fake_bake)
    rep = api.export(quad, ["glb"], tmp_path)                              # entry 10
    assert rep.ok
    assert rep.warnings == ["tessellate: 3 of 511 faces have no triangulation; skipped"]
    assert rep.manifest["warnings"] == rep.warnings
    assert json.loads((tmp_path / "manifest.json").read_text())["warnings"] == rep.warnings


# ---------------------------------------------------------------------------
# Test drive, round 3 (docs/agentlib/TESTDRIVE.md): signed coordinates, a missed target's
# scale, the bolt pillars' bound, the stack's floor, the mass estimate, the CLI's designs
# ---------------------------------------------------------------------------


def test_a_mechanism_with_signed_coordinates_resolves_checks_and_reloads():
    from spiderpig import linkage
    from spiderpig.spec import validate

    lk = linkage.get("peaucellier_crank")
    assert "yy" in lk.signed                       # entry 1
    assert "arm" not in lk.signed
    d = api.resolve({"kind": "mechanism",
                     "linkage": {"key": "peaucellier_crank", "params": {"unit": 18}}})
    cr = api.check(d)
    assert cr.ok
    assert cr.output["stroke_mm"] == pytest.approx(51.64, abs=0.01)
    assert api.load(d.id).config == d.config              # the resolved yy = -1.75 reloads
    assert api.check(api.resolve({"kind": "mechanism", "linkage": {"key": "watt_crank"}})).ok
    explicit = api.resolve({"kind": "mechanism",
                            "linkage": {"key": "peaucellier_crank", "params": {"yy": -1.75}}})
    assert explicit.config.proportions == ()             # the default, however written
    errors = validate({"kind": "mechanism",
                       "linkage": {"key": "peaucellier_crank", "params": {"arm": 0}}})
    assert [e.path for e in errors] == ["linkage.params.arm"]      # a length stays a length
    card = api.describe("peaucellier_crank")
    assert [p["name"] for p in card["params"] if p["signed"]] == ["yy"]


def test_a_missed_output_target_gets_a_checked_scale_and_the_card_a_sensitivity():
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"},
                     "motion": {"stroke_mm": {"min": 80, "hard": True}}})
    assert not api.verify(d, "quick").ok
    adv = api.advise(d)                                                       # entry 2
    assert adv.stage == "target"
    (rec,) = adv.recommendations
    assert rec.patch == {"linkage": {"params": {"unit": 19.5}}}
    assert "stroke_mm scales with unit" in rec.why
    assert rec.verified.startswith("checked: stroke_mm 81.5")
    assert api.verify(api.derive(d, rec.patch), "quick").ok
    assert [r.patch for r in api.recommend(d)] == [rec.patch]
    text = api.explain(d)
    assert "4. targets" in text
    assert "motion.stroke_mm: 66.89 mm vs >= 80: MISSED (hard)" in text
    assert "unit 16 -> 19.5" in text
    s = api.describe("hoecken")["sensitivity"]
    assert s["unit"]["stroke_mm"] == pytest.approx(10.0)
    assert s["unit"]["straightness_mm"] == pytest.approx(10.0)
    assert {"step", "stroke_mm", "straightness_mm", "extent_x_mm", "extent_y_mm",
            "on_line_fraction"} <= set(s["crank"])


def test_a_target_the_output_lacks_is_refused_and_a_mechanism_names_no_lowest_part():
    from spiderpig.spec import validate

    (err,) = validate({"kind": "mechanism", "linkage": {"key": "hoecken"},
                       "motion": {"dwell_deg": {"min": 90}}})                 # entry 3
    assert err.path == "motion.dwell_deg"
    assert "line output has no dwell_deg" in err.message
    assert list(err.allowed) == ["stroke_mm", "straightness_mm", "on_line_fraction",
                                 "transmission_angle_deg"]      # round 5: the angle too
    cr = api.check(api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}}))
    assert cr.ground_clearance_mm is None       # entry 4
    assert cr.lowest_body_part == ""


def test_bolt_pillars_bound_the_stack_and_a_failed_plan_says_so_and_offers_printed_ones():
    from spiderpig.construction.pivots.bolt import BoltAxle
    from spiderpig.fabricate import side_problem, template_for

    assert BoltAxle().max_stack(3.0) == pytest.approx(45.0)                   # entry 7
    # (the keyed crank: bolt pivots assume full layers, which the default single-plate
    # crank's clearance gaps don't give, fabricate.side_problem says so)
    cfg = BuildConfig(linkage="klann", module="single", pillar="bolt", crank="keyed",
                      robot=False)
    _, _, problem = side_problem(template_for(cfg), cfg, hint=False)
    assert problem.spec.max_top == 14
    assert problem.notes == ["pillar:A: the longest stock M3 screw (50 mm) clamps at most 15 "
                             "layers of 3 mm (45 mm), so no taller stack was searched"]
    printed = BuildConfig(linkage="klann", module="single", robot=False)
    assert side_problem(template_for(printed), printed, hint=False)[2].spec.max_top == 60
    d = api.resolve({"kind": "walker", "linkage": {"key": "klann"},
                     "constructions": {"pillar": "bolt", "crank": "keyed"}})
    pr = api.plan(d)
    assert not pr.ok
    (f,) = pr.failures
    assert "with up to 15 layers (the most a group allows)" in f.message
    assert any("clamps at most 15 layers" in n for n in f.notes)
    rec = next(r for r in f.recommendations
               if r.patch == {"constructions": {"pillar": "printed"}})
    # 17 layers on the 0.080 in frame plates of 2026-10-04 (16 on 0.125 in)
    assert "plans (quad module, the design's own) in 17 layers" in rec.verified


def test_a_proven_stack_miss_names_the_floor_and_advise_notes_it(quad):
    d = api.resolve({"kind": "walker", "linkage": {"key": "klann"},
                     "size": {"stack_mm": {"max": 30}}, **OLD})
    row = next(r for r in api.verify(d, "quick").rows if r.requirement == "size.stack_mm")
    assert not row.passed                            # entry 11
    assert row.tier == "proven"
    assert row.detail.startswith("49.764 mm is proven the thinnest for klann's quad module "
                                 "on 3 mm layers (17 layers")       # 0.080 in frame plates
    assert "no module of klann with fewer walks" in row.detail
    adv = api.advise(d)
    assert adv.stage == "target"
    assert adv.recommendations == []
    assert adv.notes[0].startswith("size.stack_mm 49.764 vs <= 30: 49.764 mm is proven the "
                                   "thinnest")


def test_the_mass_estimate_says_what_it_counts_and_the_measured_row_lists_groups(quad, robot):
    from spiderpig import verify as verify_module
    from spiderpig import walk as walk_model

    cfg = quad.config
    b = walk_model.nominal_mass_breakdown(cfg, walk_model.side_legs(cfg))     # entry 12
    # the default quad + its deck: 0.080 in frame, 0.100 in 6061 crank (the hex crankpins'
    # pockets; 975 g on 0.063 in) and 0.090 in centre plates (the thinnest per part,
    # 2026-10-04; 1088 g on 0.125 in aluminium)
    assert b["total"] == pytest.approx(1009.6, rel=0.02)
    assert (b["links"] + b["servos"] + b["plates"] + b["printed"] + b["deck"]
            == pytest.approx(b["total"]))
    row = next(r for r in api.verify(quad, "quick").rows if r.requirement == "size.mass_g")
    assert row.tier == "estimated"
    assert row.detail.startswith("estimated before a build: links ")
    assert "2 servos 110 g" in row.detail
    br = api.attach_build(quad, robot("quad", 1.0), 1.0)
    text = verify_module.mass_by_group(br)
    assert text.startswith("by group: links ")
    assert "drive " in text
    assert "chassis " in text
    one = BuildConfig(linkage="jansen", module="single", robot=False)
    side = walk_model.nominal_mass_breakdown(one, walk_model.side_legs(one), robot=False)
    both = walk_model.nominal_mass_breakdown(one, walk_model.side_legs(one))
    assert side["servos"] == pytest.approx(both["servos"] / 2)
    assert side["plates"] < both["plates"] / 2                   # no centre plates on one side


def test_captured_warnings_stay_off_the_terminal(caplog):
    import logging

    lg = logging.getLogger("spiderpig.construction.printed")
    with api.capture_warnings() as seen:
        lg.warning("pin:X seg0: its snap prongs strain")
    assert seen == ["pin:X seg0: its snap prongs strain"]
    assert caplog.records == []                              # entry 14
    assert lg.propagate


def test_spec_of_a_config_round_trips():
    cfg = BuildConfig(linkage="klann", module="single", pin="bolt", thickness=2.9,
                      servo="xl330_m288")
    doc = api.spec_of(cfg)                                                    # entry 10
    assert doc["constructions"]["pin"] == "bolt"
    assert doc["materials"]["thickness_mm"] == 2.9
    assert doc["legs"] == {"module": "single", "sides": 2}
    assert api.resolve(doc).config == cfg
    mech = api.spec_of(BuildConfig(linkage="hoecken", module="single", robot=False))
    assert mech["kind"] == "mechanism"
    assert mech["legs"]["sides"] == 1
    assert api.resolve(mech).config == BuildConfig(linkage="hoecken", module="single",
                                                   robot=False)


# ---------------------------------------------------------------------------
# Test drive, round 4 (docs/agentlib/TESTDRIVE.md): the cost floor, the glue, the BOM's
# sheets, a robot part's frame, the sim's rows and mesher, the walks flag, the warnings
# ---------------------------------------------------------------------------

STRIDER_DOUBLE_PLY = {"kind": "walker", "linkage": {"key": "strider"},
                      "legs": {"module": "double"}, "materials": {"sheet": "plywood_3mm"}}


def test_the_cost_floor_counts_the_glue_and_the_nuts_and_says_what_a_build_adds():
    from spiderpig import verify as verify_module

    d = api.resolve(STRIDER_DOUBLE_PLY, store=None)
    total, priced, unpriced = verify_module.cost_floor(d)                     # entry 1
    # the threadlockers (the Chicago pins and the standoff pillars' screws take the
    # low-strength one) and the Chicago barrels' epoxy are bought whatever the sizes, but the
    # catalog has no verified price for them yet. The thin sheets of 2026-10-04: a 0.080 in
    # frame blank ($18, the 0.125 in was $28) and, since the hex-standoff crankpins, a 0.100
    # in 6061 crank blank ($21; the 0.063 in 5052 was $18); the single-plate crank has no
    # nylocks, and nothing is glued to a plate any more (no wood glue: the glue-free
    # joinery), and no CA since the battery cradle is screwed to the deck (2026-10-05)
    assert total == pytest.approx(40.0 + 25.49 + 3.10 + 18.0 + 21.0 + 11.37)
    assert unpriced == ["Two-part slow-cure structural epoxy (e.g. J-B Weld Original or "
                        "Loctite EA E-30CL), 2 x 25 ml",          # the Chicago barrels
                        "Low-strength threadlocker (Loctite 222 or equivalent), 10 ml",
                        "Medium-strength threadlocker (Loctite 243 or equivalent), 10 ml"]
    assert not any(line.startswith("Medium CA (cyanoacrylate) glue") for line in priced)
    assert sum(line.startswith("Titebond II") for line in priced) == 0
    # the frame blank in 5052, the crank's in 6061 (the hex crankpins' pockets)
    assert sum(line.startswith("5052 aluminium sheet") for line in priced) == 1
    assert sum(line.startswith("6061 aluminium sheet") for line in priced) == 1
    lift = api.resolve({"kind": "mechanism", "linkage": {"key": "parallelogram_lift"}, **OLD},
                       store=None)
    assert verify_module.cost_floor(lift)[0] == pytest.approx(72.86 + 18.0)   # + its Al frame
    bolted = api.resolve({"kind": "mechanism", "linkage": {"key": "parallelogram_lift"},
                          "constructions": {"pillar": "bolt", "pin": "bolt", "crank": "keyed"}},
                         store=None)
    assert verify_module.cost_floor(bolted)[0] == pytest.approx(72.86 + 18.0 - 13.99)   # unglued
    row = next(r for r in api.verify(lift, "quick").rows
               if r.requirement == "budget.cost_floor_usd")
    assert row.value == pytest.approx(72.86 + 18.0)
    assert row.detail.endswith(verify_module.FLOOR_LEAVES_OUT)
    assert "the sheets' count, the crank's screws" in row.detail


def test_a_bom_exported_without_a_dxf_still_buys_the_sheets(tmp_path):
    d = api.resolve(KLANN_SINGLE, store=None)
    rep = api.export(d, ["bom"], tmp_path)                                    # entry 3
    assert rep.ok
    bom = json.loads((tmp_path / "bom.json").read_text())
    sheet = next(r for r in bom["purchased"] if r["key"] == "acrylic_3mm")
    assert sheet["qty"] >= 1
    assert sheet["cost_usd"] == pytest.approx(10.99)
    assert bom["cost_usd"] == pytest.approx(sum(r["cost_usd"] or 0 for r in bom["purchased"]))


@pytest.mark.slow
def test_a_robot_part_locates_a_cut_by_the_sides_coordinates_and_recheck_notes_a_miss(design):
    from build123d import Cylinder

    design("single")                                # the side's plan is the session's
    d = api.resolve({"kind": "walker", "linkage": {"key": "klann"},
                     "legs": {"module": "single"}}, store=None)
    api.build(d)
    z_mid = d.mech.meta["mid_plane"]
    links = sorted(n for n, p in d.parts.items() if p.group == "links" and p.side == "L")
    first, second = links[0], links[1]                # the Klann single's L.b1, L.b2
    edited = {}
    for name in (first, "R." + first[2:]):                                    # entry 5
        link, body = d.parts[name], d.mech.body(name)
        assert link.z_mid == z_mid
        assert link.z_side == pytest.approx(d.side.plan.z(link.layers[0]))
        a, b = (body.joint(j).pose.matrix[:2, 3] for j in body.outline[0])
        loc = link.locate((a + b) / 2)
        z = sum(link.z_side) / 2
        z_world = loc.position.Z
        assert z_world == pytest.approx(z - z_mid if name[0] == "L" else z_mid - z)
        bb = link.solid.bounding_box()
        assert bb.min.Z < z_world < bb.max.Z                # inside the solid, whichever side
        before = link.volume_mm3
        link.solid = link.solid - Cylinder(1.5, 10).moved(loc)
        edited[name] = before - link.volume_mm3
        assert edited[name] == pytest.approx(math.pi * 1.5 ** 2 * 3.0, rel=0.02)
    missed = d.parts[second]                        # the old way: the plan's z, no frame
    zz = d.side.plan.z(missed.layers[0])
    missed.solid = missed.solid - Cylinder(1.5, 10).moved(Location((0.0, 0.0, sum(zz) / 2)))
    rc = api.recheck(d)
    assert rc.ok
    assert sorted(rc.edited) == sorted([first, second, "R." + first[2:]])
    assert rc.contract == []
    assert rc.clashes == []
    (note,) = rc.notes
    assert note.startswith(f"{second}: the edited solid has the build's volume")
    assert "Part.locate" in note
    from spiderpig.design import jsonable

    assert api.RecheckReport.from_dict(json.loads(json.dumps(jsonable(rc)))).notes == [note]
    one = api.resolve(KLANN_SINGLE, store=None)      # one side: the side's frame as is
    api.build(one)
    part = one.parts[first[2:]]                  # the same link, unprefixed on one side
    assert part.z_mid is None
    z_default, z_given = part.locate((1.0, 2.0)).position.Z, part.locate((1, 2), 4.5).position.Z
    assert z_default == pytest.approx(sum(part.z_side) / 2)
    assert z_given == pytest.approx(4.5)


def test_the_sim_rows_have_their_own_names(monkeypatch):
    from spiderpig import verify as verify_module

    pytest.importorskip("mujoco")
    from spiderpig.sim import run as sim_run

    fake = {"speed": 164.9, "stride": 190.4, "fell": False, "max_tilt": 4.2,
            "torque_peak": 0.21, "torque_limit": 1.91, "saturates": False}
    monkeypatch.setattr(sim_run, "simulate", lambda config, seconds: None)
    monkeypatch.setattr(sim_run, "walk_metrics", lambda result: fake)
    d = api.resolve({**KLANN_QUAD, "motion": {"speed_mm_s": {"min": 100}}}, store=None)
    rows = verify_module._sim_rows(d, verify_module.VerifyReport(level="full"))   # entry 7
    assert [r.requirement for r in rows] == ["sim.speed_mm_s", "sim.stride_mm", "sim.stays_up",
                                             "sim.torque"]
    assert rows[0].source == "sim"
    assert rows[0].value == pytest.approx(164.9)
    assert rows[0].target == ">= 100"
    assert rows[0].passed
    assert not rows[0].hard
    assert "the walk model's motion.speed_mm_s row is the spec's" in rows[0].detail


def test_the_meshes_read_by_occts_gltf_writer_are_the_loops():
    """mesh.read_meshes takes the triangles out with RWGltf_CafWriter; the node-by-node
    loop it replaced (mesh._read_faces) must give the very same arrays, located, mirrored
    (reversed faces), curved and multi-solid parts alike, and parts sharing faces (a part
    placed twice) each their own."""
    from build123d import Compound, Cylinder, Plane, Pos, Rot

    from spiderpig import mesh
    from spiderpig.shapes import moved

    holed = Box(20, 8, 3) - Cylinder(2, 10).moved(Pos(5, 0, 0))
    parts = [Box(2, 3, 4), holed, moved(holed, Pos(30, 2, 1) * Rot(0, 0, 37)),
             holed.mirror(Plane.XY), Compound([Box(1, 1, 1), Box(1, 1, 1).moved(Pos(3, 0, 0))])]
    got = mesh.tessellate_many(parts)
    assert mesh._read_gltf(parts)          # the writer's path, located parts too: no fallback
    for part, (pos, tri, skipped) in zip(parts, got, strict=True):
        mesh.mesh_part(part)
        want_pos, want_tri = mesh._read_faces(part)
        assert skipped == 0
        assert (pos.dtype, tri.dtype) == (want_pos.dtype, want_tri.dtype)
        assert pos.shape == want_pos.shape
        assert (pos == want_pos).all()
        assert (tri == want_tri).all()


def test_the_sim_meshes_face_by_face_as_the_bake_does():
    from spiderpig.bake import _tessellate
    from spiderpig.mesh import tessellate
    from spiderpig.sim.mjcf import _hull

    box = Box(2, 3, 4)
    pos, tri, skipped = tessellate(box)                                       # entry 6
    assert skipped == 0
    assert (len(pos), len(tri)) == (24, 36)                 # 6 faces x 4 corners, 12 triangles
    bake_pos, bake_tri = _tessellate(box)
    assert (bake_pos == pos).all()
    assert (bake_tri == tri).all()
    hull = _hull(box, 0.1)
    assert hull.shape == (8, 3)
    assert sorted(map(tuple, hull))[0] == pytest.approx((-1.0, -1.5, -2.0))


def test_walks_means_a_stride_of_twenty_millimetres():
    assert api.WALKS_MM == 20.0
    card = api.describe("trotbot_toe")
    single = card["modules"]["single"]                                       # entry 14
    assert 1 < single["stride_mm"] < 20
    assert not single["walks"]
    assert card["modules"]["decker"]["walks"]
    d = api.resolve({"kind": "walker", "linkage": {"key": "trotbot_toe"},
                     "legs": {"module": "single"}}, store=None)
    (note,) = api.walk(d).notes
    assert note.startswith("a shuffle, not a walk: trotbot_toe's single module")
    assert "decker" in note
    assert "quad" in note
    assert "single (" not in note                       # a 4 mm shuffle isn't offered as a walk


@pytest.mark.slow
def test_the_static_stage_and_the_standard_verify_keep_the_warnings_off_the_terminal(caplog):
    import logging

    d = api.resolve({"kind": "walker", "linkage": {"key": "strider"},
                     "legs": {"module": "single", "sides": 1},
                     "constructions": {"pin": "printed"}}, store=None)     # not the rod default
    with caplog.at_level(logging.WARNING):
        cr = api.check(d)
        rep = api.verify(d, "standard")                                     # entry 12
    assert cr.ok
    assert isinstance(cr.warnings, list)
    assert rep.ok
    br = d.reports["build"]
    # the planner relieves a snap lip it would once have warned about (the audit's snap
    # column reports it), so a printed pin builds without a warning either way
    assert isinstance(br.warnings, list)
    assert not any("snap prongs" in w for w in br.warnings)
    assert [r for r in caplog.records if r.name.startswith("spiderpig.construction")] == []


# ---------------------------------------------------------------------------
# Test drive, round 5 (docs/agentlib/TESTDRIVE.md): the budget's allowance for unpriced
# items, the transmission angle as a target, a point off the number line, the two-input
# mechanism's words, the spec messages per kind, a one-sided envelope, the fall's detail
# ---------------------------------------------------------------------------


def test_r5_a_budget_allowance_accepts_the_unpriced_items():
    from types import SimpleNamespace

    from spiderpig import verify as verify_module
    from spiderpig.spec import ALLOWANCE

    spec = {"kind": "walker", "linkage": {"key": "klann"}, "legs": {"module": "single"},
            "budget": {"cost_usd": {"max": 150}, ALLOWANCE: 15}}
    assert validate(spec) == []                                               # entry 1
    (e,) = validate({**spec, "budget": {"cost_usd": {"max": 150}, ALLOWANCE: -1}})
    assert (e.path, e.message) == (f"budget.{ALLOWANCE}", "must be >= 0, got -1")
    assert ALLOWANCE in spec_schema()["properties"]["budget"]["properties"]
    d = api.resolve(spec, store=None)
    assert d.spec.allowance_usd == 15.0
    assert d.resolved["budget"][ALLOWANCE] == 15.0          # part of the id
    assert d.spec.to_dict()["budget"][ALLOWANCE] == 15.0
    plain = api.resolve({**spec, "budget": {"cost_usd": {"max": 150}}}, store=None)
    assert plain.id != d.id
    row = SimpleNamespace(key="m2_self_tap_6", name="M2 x 6 mm screw", qty=8, pack_qty=100,
                          packs=1, vendor="Amazon", cost_usd=None, verified=True,
                          same_pack_as=None)
    servo = SimpleNamespace(key="sts3215", name="Feetech STS3215", qty=1, pack_qty=1, packs=1,
                            vendor="Seeed", cost_usd=100.0, verified=True, same_pack_as=None)
    bom = SimpleNamespace(purchased=[servo, row], unpriced=[row], cost_usd=100.0)
    r = verify_module.cost_row(plain, bom)             # no allowance: a lower bound, FAIL
    assert not r.passed
    assert r.value == 100.0
    assert r.detail.startswith("at least; the target can't be verified while items are "
                               "unpriced (price them in the catalog, or accept them with an "
                               "allowance: budget.allowance_usd")
    r = verify_module.cost_row(d, bom)                  # the allowance: added, and it verifies
    assert r.passed
    assert r.value == 115.0
    assert r.detail.startswith("$100.00 priced + $15.00 allowed (budget.allowance_usd) for the "
                               "1 unpriced item: ")
    assert "8 x M2 x 6 mm screw (1 pack of 100 at Amazon)" in r.detail
    over = api.resolve({**spec, "budget": {"cost_usd": {"max": 110}, ALLOWANCE: 15}},
                       store=None)
    assert not verify_module.cost_row(over, bom).passed


def test_r5_the_transmission_angle_is_a_target_and_a_row():
    from spiderpig import verify as verify_module

    f = TARGET_FIELDS["motion"]["transmission_angle_deg"]                    # entry 2
    assert f.kinds == ("walker", "mechanism")
    assert not f.hard
    assert f.source == "check"
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "rocker_amplifier"},
                     "motion": {"swing_deg": {"min": 120},
                                "transmission_angle_deg": {"min": 40}}}, store=None)
    rep = api.verify(d, "quick")
    assert rep.ok
    row = next(r for r in rep.rows if r.requirement == "motion.transmission_angle_deg")
    assert row.value == pytest.approx(42.6, abs=0.1)    # K: 43°..136° folded about 90°
    assert row.passed
    assert row.tier == "measured"
    assert not row.hard
    assert row.detail.startswith("the least over the closures is at K")
    closures = [s for s in d.reports["check"].steps if s["kind"] == "closure"]
    angle, at = verify_module.least_transmission_angle(closures)
    assert (round(angle, 1), at) == (42.6, "K")
    assert verify_module.least_transmission_angle([]) == (None, "")
    assert "motion.transmission_angle_deg: 42.62 deg vs >= 40: ok" in api.explain(d)
    tight = api.resolve({"kind": "mechanism", "linkage": {"key": "rocker_amplifier"},
                         "motion": {"transmission_angle_deg": {"min": 45, "hard": True}}},
                        store=None)
    assert not api.verify(tight, "quick").ok
    adv = api.advise(tight)                 # a missed target check can read: said, no lever
    assert adv.stage == "target"
    assert adv.notes[0].startswith("motion.transmission_angle_deg 42.6")
    walker = api.resolve({"kind": "walker", "linkage": {"key": "klann"},
                          "legs": {"module": "single"}}, store=None)
    row = next(r for r in api.verify(walker, "quick").rows
               if r.requirement == "motion.transmission_angle_deg")
    assert row.target is None
    assert row.value == pytest.approx(24.7, abs=0.1)
    # a misspelling is nearest; a metric of another name (a "rotation") is not offered
    (e,) = validate({"kind": "mechanism", "linkage": {"key": "hoecken"},
                     "motion": {"transmision_angle_deg": {"min": 40}}})
    assert (e.path, e.message, e.nearest) == ("motion.transmision_angle_deg", "unknown metric",
                                              "transmission_angle_deg")
    (e,) = validate({"kind": "walker", "linkage": {"key": "klann"},
                     "motion": {"obstacle_mm": {"min": 25}}})
    assert e.nearest is None
    assert "transmission_angle_deg" in e.allowed


def test_r5_a_point_off_the_number_line_fails_the_program_stage_and_names_the_root():
    from spiderpig import linkage as linkage_module

    d = api.resolve({"kind": "mechanism", "linkage": {"key": "crank_rocker",
                                                       "params": {"crank": 1.6, "rocker": 1.2}}},
                    store=None)
    cr = api.check(d)                                                          # entry 3
    assert not cr.ok
    (f,) = cr.failures
    assert (f.stage, f.code) == ("program", "point_undefined")
    assert f.message.startswith("crank_rocker: G (fixed) can't be placed at these parameters: "
                                "its coordinates are not a number (sqrt(-crank**2 + rocker**2) "
                                "with -crank**2 + rocker**2 = -1.12 at these parameters")
    assert f.culprits == [{"joint": "G", "refs": []}]
    assert f.notes
    assert "square root" in f.notes[0]
    step = next(s for s in cr.steps if s["point"] == "G")
    assert step["invalid"]
    assert step["fails_deg"] == (0.0, 360.0)
    assert step["fail_fraction"] == 1.0
    assert cr.output is None
    assert cr.foot_path is None
    with pytest.raises(linkage_module.AssemblyError, match="not a number"):
        d.lk.assert_assembles(dict(d.config.proportions))
    assert api.advise(d).stage == "program"
    assert "STOP: crank_rocker: G (fixed) can't be placed" in api.explain(d)
    assert not api.verify(d, "quick").ok
    ok = api.resolve({"kind": "mechanism", "linkage": {"key": "crank_rocker",
                                                        "params": {"rocker": 1.2}}}, store=None)
    assert api.check(ok).ok                          # the same point, a number again


def test_r5_a_two_input_mechanism_is_told_at_resolve_and_by_advise():
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "five_bar"}}, store=None)
    (w,) = d.warnings                                                          # entry 4
    assert w.startswith("five_bar has 2 inputs (t, t2) and v1 builds one drive")
    assert "second_input_no_drive" in w
    assert "a limit of v1, not of the spec" in w
    assert "the one-input mechanisms are hoecken, " in w
    assert "five_bar" not in w.split("are ")[1]
    cr = api.check(d)
    (f,) = cr.failures
    assert (f.stage, f.code) == ("drive", "second_input_no_drive")
    assert "a limit of v1 (one servo per machine), not of this spec" in f.message
    assert "a one-input mechanism builds (hoecken, " in f.message
    adv = api.advise(d)
    assert adv.stage == "drive"
    assert adv.recommendations == []
    assert adv.notes[-1].startswith("no fix: five_bar has 2 inputs")
    assert api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}}, store=None
                       ).warnings == []


def test_r5_the_spec_messages_fit_the_kind():
    from spiderpig import linkage as linkage_module

    (e,) = validate({"kind": "mechanism", "linkage": {"key": "five_bar"},
                     "motion": {"stroke_mm": {"min": 50}}})                     # entry 8
    assert e.message == ("five_bar's xy output has no stroke_mm; an xy output is measured by "
                         "its extent alone (extent_mm on the card), which is no target")
    assert e.allowed == ("transmission_angle_deg",)
    (e,) = validate({"kind": "mechanism", "linkage": {"key": "hoecken"},
                     "legs": {"module": "quad"}})                               # entry 9
    assert e.message == ("unknown value 'quad'; hoecken is a mechanism: its one module is single "
                         "(one side, no legs; the leg modules double, decker and quad are a "
                         "walker's)")
    assert e.allowed == ("single",)
    (e,) = validate({"kind": "walker", "linkage": {"key": "klan"}})            # entry 10
    assert e.message == "unknown value 'klan'"
    assert e.nearest == "klann"
    assert set(e.allowed) == set(linkage_module.available("walker"))
    (e,) = validate({"kind": "walker", "linkage": {"key": "hoeken"}})
    assert e.message == "unknown value 'hoeken'; 'hoecken' is a mechanism, not a walker"
    assert e.nearest == "hoecken"
    assert "hoecken" not in e.allowed
    errs = validate({"linkage": {"key": "klan"}})            # no kind: every key is allowed
    assert [x.path for x in errs] == ["kind", "linkage.key"]
    assert set(errs[1].allowed) == set(linkage_module.available())


def test_r5_a_one_sided_envelope_row_says_the_stack_not_the_stacks():
    d = api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken"}}, store=None)
    row = next(r for r in api.verify(d, "quick").rows if r.requirement == "size.envelope_z_mm")
    assert row.detail.startswith("the stack + the servo on the inner plate + ")    # entry 12
    robot = api.resolve(KLANN_QUAD, store=None)
    row = next(r for r in api.verify(robot, "quick").rows
               if r.requirement == "size.envelope_z_mm")
    assert row.detail.startswith("the two stacks + the chassis + ")


def test_r5_the_fall_detail_says_when_how_and_what_the_model_saw():
    from types import SimpleNamespace

    import numpy as np

    from spiderpig import verify as verify_module
    from spiderpig.sim.run import _fall

    t = np.linspace(0.0, 4.0, 41)
    tilt = np.where(t >= 0.9, math.radians(92.0), 0.05)
    attitude = np.zeros((41, 3))
    attitude[:, 2] = np.where(t >= 0.9, math.radians(92.0), 0.0)        # rolling
    r = SimpleNamespace(t=t, tilt=tilt, attitude=attitude)
    fall = _fall(r, 0.5)                                                       # entry 5
    assert fall == {"fell_at_s": pytest.approx(0.9), "fell_axis": "rolling"}
    assert _fall(SimpleNamespace(t=t, tilt=np.zeros(41), attitude=attitude), 0.5) == {
        "fell_at_s": None, "fell_axis": None}
    m = {"fell": True, "max_tilt": 92.2, **fall}
    walk = SimpleNamespace(ok=True, metrics={"tipping_fraction": 0.0})
    design = SimpleNamespace(reports={"walk": walk})
    text = verify_module.fall_detail(m, design)
    assert text.startswith("max tilt 92.2 deg; fell over rolling at 0.9 s into the 4 s run")
    assert "the quasi-static model's tipping fraction is 0.00 (it saw no tipping" in text
    assert verify_module.fall_detail({"fell": False, "max_tilt": 4.2}) == "max tilt 4.2 deg"


def test_r5_the_guide_names_the_allowance_the_angle_and_the_second_input():
    from spiderpig.mcp import render_guide

    guide = render_guide()
    assert "`budget.allowance_usd`" in guide                                   # entry 11
    assert "have vendor links but no price" not in guide
    assert "transmission_angle_deg" in guide
    assert "second_input_no_drive" in guide
