"""The agent-facing surface (:mod:`spiderpig`): the Spec validates and resolves to a stable
design id, the operations return reports (failures as data, never raised), the harness
verifies with tiers, parts expose live solids and ``recheck`` catches an edited one."""

from __future__ import annotations

import json

import pytest
from build123d import Box, Location

from spiderpig import api
from spiderpig.failure import Failure, apply_patch, parse_blocker
from spiderpig.spec import TARGET_FIELDS, Spec, SpecErrors, Target, spec_schema, validate

KLANN_QUAD = {"kind": "walker", "linkage": {"key": "klann"}}


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
    assert schema["properties"]["linkage"]["properties"]["key"]["enum"][0] == "klann"
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
                     "constructions": {"pillar": "printed", "pin": "printed", "crank": "printed"},
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
    assert r["fit"]["kerf_mm"] == 0.15
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
    assert keys[0] == "klann"
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
    assert pr.n_layers == 12
    assert pr.height_mm == 36.0
    assert pr.optimal
    assert pr.route == {"runs": [{"at": f"M_leg{k}", "lo": 2 + 2 * k, "hi": 2 + 2 * k}
                                 for k in range(4)], "bearing": True}
    assert pr.layers["b1_leg0"] == 2
    assert "inner frame plate" in pr.table
    wr = api.walk(quad)
    assert wr.ok
    assert wr.feet_z_planned
    assert wr.mass_nominal
    assert wr.metrics["stride_mm"] == pytest.approx(102.4, abs=0.1)
    assert api.recommend(quad) == []
    assert "3. plan" in api.explain(quad)
    assert [e["op"] for e in quad.log][:3] == ["check", "plan", "walk"]
    json.dumps(quad.to_dict())     # every report is JSON-able


def test_verify_quick_passes_with_tiers(quad):
    rep = api.verify(quad, "quick")
    assert rep.ok
    assert rep.score == 1.0
    assert rep.failures == []
    rows = {r.requirement: r for r in rep.rows}
    assert rows["program.loops_close"].tier == "proven"
    assert rows["program.loops_close"].passed
    assert rows["size.stack_mm"].value == 36.0
    assert rows["size.stack_mm"].tier == "proven"
    assert rows["motion.speed_mm_s"].tier == "estimated"
    assert rows["motion.stride_mm"].tier == "measured"
    assert rows["size.mass_g"].tier == "estimated"
    assert rows["motion.ground_clearance_mm"].value == pytest.approx(64.15, abs=0.05)
    assert rep.unverified == []
    json.dumps(rep.to_dict())


def test_a_hard_size_miss_fails_and_the_same_target_soft_only_lowers_the_score():
    hard = api.resolve({**KLANN_QUAD, "size": {"stack_mm": {"max": 30}},
                        "motion": {"stride_mm": {"min": 90}}})
    rep = api.verify(hard, "quick")
    assert not rep.ok
    row = next(r for r in rep.rows if r.requirement == "size.stack_mm")
    assert not row.passed
    assert row.hard
    assert row.value == 36.0
    assert row.target == "<= 30"
    assert rep.score == 1.0                        # the stride, the only soft target, is met
    soft = api.resolve({**KLANN_QUAD, "size": {"stack_mm": {"max": 30, "hard": False}},
                        "motion": {"stride_mm": {"min": 90}}})
    rep = api.verify(soft, "quick")
    assert rep.ok
    row = next(r for r in rep.rows if r.requirement == "size.stack_mm")
    assert not row.passed
    assert not row.hard
    assert row.score == pytest.approx(0.8)
    assert rep.score == pytest.approx(0.9)         # the mean of stride 1.0 and stack 0.8
    assert hard.id != soft.id


# ---------------------------------------------------------------------------
# Failures as data
# ---------------------------------------------------------------------------


def test_the_heel_at_the_drawings_unit_fails_the_static_stage_with_a_patch_that_plans():
    heel = api.resolve({"kind": "walker", "linkage": {"key": "trotbot_heel",
                                                      "params": {"unit": 7}},
                        "legs": {"module": "single"}})
    cr = api.check(heel)
    assert not cr.ok
    (f,) = cr.failures
    assert (f.stage, f.code) == ("static", "link_no_layer")
    assert f.culprits[0]["body"] == "b7"
    assert f.culprits[0]["point"] == "J1"
    assert f.numbers["dist_mm"] == pytest.approx(6.8, abs=0.05)
    assert f.numbers["need_mm"] == 10.0
    (rec,) = f.recommendations
    assert rec.patch == {"linkage": {"params": {"unit": 10.5}}}
    assert rec.changes == [{"name": "unit", "before": 7.0, "after": 10.5}]
    assert rec.verified.startswith("checked: the static stage passes, and it plans in 12 ")
    assert api.recommend(heel) == [rec]
    assert api.plan(heel).failures == cr.failures
    rep = api.verify(heel, "quick")
    assert not rep.ok
    assert rep.failures[0].code == "link_no_layer"
    fixed = api.resolve(apply_patch(heel.spec.to_dict(), rec.patch))
    assert fixed.resolved["linkage"]["params"]["unit"] == 10.5
    assert fixed.id != heel.id
    pr = api.plan(fixed)
    assert pr.ok
    assert pr.n_layers == 12


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
    assert b1.layers == (2,)
    assert not b1.edited
    assert quad.parts["R.b1_leg0"].layers == (2,)
    assert quad.parts["R.b1_leg0"].side == "R"
    assert quad.parts["L.servo"].group == "drive"
    assert quad.parts["L.servo"].mass_g == 55.0
    assert quad.parts["L.pillar_A_leg0_seg0"].group == "pillar:A_leg0"
    assert quad.parts["L.crank_seg0"].group == "crank"
    assert quad.parts["centre_plate0"].group == "chassis"
    assert quad.parts["centre_plate0"].side \
        is None
    rr = api.recheck(quad)      # nothing edited: solids and clashes only, no mutation
    assert rr.ok
    assert rr.edited == []
    assert rr.checked == []
    assert rr.clashes == []
    assert quad.parts["L.b1_leg0"].solid is robot("quad", 1.0).body("L.b1_leg0").part


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
    assert {"klann.step", "laser/klann_sheet_0.dxf", "laser/klann_sheet_parts.csv", "bom.csv",
            "bom.md", "bom.json", "manifest.json"} <= names
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert manifest["design"] == d.id
    assert manifest["plan"]["layers"] == 7
    assert manifest["bom"]["items"] > 0
    assert len(manifest["parts"]) == 28
    with pytest.raises(ValueError, match="unknown formats"):
        api.export(d, ["pdf"], tmp_path)


# ---------------------------------------------------------------------------
# Test drive, round 1 (docs/agentlib/TESTDRIVE.md): what the reports must say
# ---------------------------------------------------------------------------

KLANN_SINGLE = {"kind": "walker", "linkage": {"key": "klann"},
                "legs": {"module": "single", "sides": 1}}


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
    assert "sensitivity" not in api.describe("hoecken")
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
    (r,) = recs
    assert r.changes == (("unit", 1.1, 1.6),)
    assert r.verified == checked
    assert "default scale" in r.why
    assert notes == []
    recs, notes = rec.recommend(BuildConfig(linkage="jansen", module="quad"), plan=True)
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
