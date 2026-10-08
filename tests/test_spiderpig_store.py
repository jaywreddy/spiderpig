"""The per-project design store (:mod:`spiderpig.store`): a design and every stage's report
round-trip through files with their id intact, a reloaded plan is verified and identical,
parts come back from STEP, stale engine versions re-verify or rebuild, gc removes only
what it should, compare and derive work off the record, and the cache is hit."""

from __future__ import annotations

import json
from datetime import UTC, datetime, timedelta

import numpy as np
import pytest

from spiderpig import api
from spiderpig.config import BuildConfig
from spiderpig.design import jsonable
from spiderpig.failure import apply_patch, merge_patch
from spiderpig.stack import verify_plan
from spiderpig.stages import planning as stages_planning
from spiderpig.stages import resolve as stages_resolve
from spiderpig.store import STORE_ENV, Store, StoreError, diff_json
from tests import _api

# The numbers below are the default constructions' (the hex crank, ``bolt_round`` on
# TrotBot's heel, the standoff pillars and Chicago pins; the keyed crank and the printed
# pillars these tests pinned until 2026-10-07 are removed). The default Klann quad stacks
# 72.989 mm (13 layers), under the 80 its spec asks
KLANN_QUAD = {"kind": "walker", "linkage": {"key": "klann"}, "size": {"stack_mm": {"max": 80}}}
KLANN_SINGLE = {"kind": "walker", "linkage": {"key": "klann"},
                "legs": {"module": "single", "sides": 1}}
HEEL = {"kind": "walker", "linkage": {"key": "trotbot_heel", "params": {"unit": 7}},
        "legs": {"module": "single"}}
# the plan the heel's checked patch (its default unit, 10.5) has
HEEL_DEFAULT = BuildConfig(linkage="trotbot_heel", module="single")


def _doc(rep) -> dict:
    """A report's JSON form without its timing."""
    d = rep.to_dict() if hasattr(rep, "to_dict") else jsonable(rep)
    d.pop("seconds", None)
    d.pop("reused", None)
    return d


def _stored(store: Store, id: str, stage: str) -> dict:
    doc = store.read_report(id, stage)
    assert doc is not None, f"{stage}.json missing"
    for k in ("stage", "design", "engine_version", "written_at", "seconds", "reused"):
        doc.pop(k, None)
    return doc


def _count(monkeypatch, module, name, instead=None):
    """Replace ``module.name`` with a counting wrapper (calling ``instead`` when given,
    else the original); returns the call list."""
    calls: list[tuple] = []
    original = getattr(module, name) if instead is None else instead

    def wrapper(*args, **kwargs):
        calls.append(args)
        return original(*args, **kwargs)

    monkeypatch.setattr(module, name, wrapper)
    return calls


# ---------------------------------------------------------------------------
# The record
# ---------------------------------------------------------------------------


def test_resolve_records_the_design_and_load_gives_it_back(tmp_path):
    store = Store(tmp_path)
    d = api.resolve(KLANN_QUAD, store)
    assert d.store == store
    folder = store.dir(d.id)
    assert {p.name for p in folder.iterdir()} == {"spec.json", "resolved.json"}
    rec = json.loads((folder / "resolved.json").read_text())
    assert rec["id"] == d.id
    assert rec["engine_version"] == d.engine_version
    assert rec["resolved"] == json.loads(json.dumps(d.resolved))
    assert rec["derived_from"] is None
    assert json.loads((folder / "spec.json").read_text()) == d.spec.to_dict()
    back = api.load(d.id, store)
    assert back.id == d.id
    assert back.resolved == json.loads(json.dumps(d.resolved))
    assert back.config == d.config
    assert back.spec == d.spec
    assert back.warnings == d.warnings
    assert back.created_at == d.created_at
    assert back.reports == {}
    listed = api.list_designs(store)
    assert [x["id"] for x in listed] == [d.id]
    assert listed[0]["linkage"] == "klann"
    assert listed[0]["stages"] == {}
    # the first record wins: resolving again keeps it
    again = api.resolve(KLANN_QUAD, tmp_path)
    assert again.created_at == d.created_at
    with pytest.raises(KeyError, match="no design"):
        api.load("0" * 16, store)
    with pytest.raises(ValueError, match="not a design id"):
        store.dir("../etc")


def test_store_none_keeps_everything_in_memory_and_the_env_var_picks_the_root(
        tmp_path, monkeypatch):
    monkeypatch.setenv(STORE_ENV, str(tmp_path / "env"))
    d = api.resolve(KLANN_QUAD)
    assert d.store == Store(tmp_path / "env")
    assert (tmp_path / "env" / "designs" / d.id / "resolved.json").is_file()
    m = api.resolve(KLANN_QUAD, store=None)
    assert m.store is None
    assert m.id == d.id
    api.check(m)
    assert not (tmp_path / "env" / "designs" / d.id / "check.json").exists()
    assert Store.of(None) is None
    assert Store.of(str(tmp_path)) == Store(tmp_path)


def test_a_record_that_does_not_hash_to_its_id_is_refused(tmp_path):
    store = Store(tmp_path)
    d = api.resolve(KLANN_QUAD, store)
    path = store.dir(d.id) / "resolved.json"
    rec = json.loads(path.read_text())
    rec["resolved"]["linkage"]["params"]["OA"] = 61.0
    path.write_text(json.dumps(rec))
    with pytest.raises(StoreError, match="hashes to"):
        api.load(d.id, store)


# ---------------------------------------------------------------------------
# Reports round-trip and the cache
# ---------------------------------------------------------------------------


def test_reports_round_trip_and_the_second_run_hits_the_cache(tmp_path, design, monkeypatch):
    design("quad")       # the session's plan (cached)
    store = Store(tmp_path)
    d = api.resolve(KLANN_QUAD, store)
    reps = {"check": api.check(d), "plan": api.plan(d), "walk": api.walk(d),
            "verify": api.verify(d, "quick")}
    assert all(r.ok for r in reps.values())
    for stage, rep in reps.items():
        assert _stored(store, d.id, stage) == _doc(rep)
    assert not any(e["cached"] for e in d.log)
    assert [e["op"] for e in store.read_log(d.id)] == ["check", "plan", "walk", "verify:quick"]

    # a fresh handle: everything comes from the store, the engine's stages aren't run
    solved = _count(monkeypatch, stages_planning, "design_side")
    problems = _count(monkeypatch, stages_planning, "side_problem")
    verified = _count(monkeypatch, stages_planning, "verify_plan")
    payloads = _count(monkeypatch, api.walk_model, "api_payload")
    back = api.load(d.id, store)
    for stage, rep in reps.items():
        got = api.verify(back, "quick") if stage == "verify" else getattr(api, stage)(back)
        assert _doc(got) == _doc(rep), stage
    assert [(e["op"], e["cached"]) for e in back.log] == [
        ("check", True), ("plan", True), ("walk", True), ("verify:quick", True)]
    assert solved == []                      # no plan search
    assert len(problems) == 1                # the side rebuilt once for the plan
    assert len(verified) == 1
    assert payloads == []                    # the walk model wasn't run
    assert back.side is not None
    assert back.reports["plan"].reused == "store"
    assert back.reports["plan"].optimal
    assert api.verify(back, "quick") is back.reports["verify"]
    # force recomputes and rewrites
    fresh = api.check(back, force=True)
    assert _doc(fresh) == _doc(reps["check"])
    assert back.log[-1]["cached"] is False


def test_a_reloaded_plan_is_verified_and_identical(tmp_path, design):
    tmpl, side = design("quad")     # KLANN_QUAD's
    store = Store(tmp_path)
    d = api.resolve(KLANN_QUAD, store)
    api.plan(d)
    back = api.load(d.id, store)
    pr = api.plan(back)
    plan = back.side.plan
    assert plan.layers == side.plan.layers
    assert plan.top == side.plan.top
    assert plan.choices["crank"] == side.plan.choices["crank"]
    assert len(plan.placed) == len(side.plan.placed)
    assert (plan.optimal, plan.proof, plan.cost) == (side.plan.optimal, side.plan.proof,
                                                     side.plan.cost)
    assert pr.route == {"runs": [{"at": r.at, "lo": r.lo, "hi": r.hi}
                                 for r in side.plan.choices["crank"].runs], "bearing": True}
    assert verify_plan(plan, tmpl) == []
    assert back.side.ground_clearance_mm == pytest.approx(side.ground_clearance_mm)
    assert api.walk(back).feet_z_planned


def test_a_failing_check_and_plan_are_cached_as_failures(tmp_path):
    _api.seed(HEEL_DEFAULT)            # what its recommendation checks, cached
    store = Store(tmp_path)
    heel = api.resolve(HEEL, store)
    cr, pr = api.check(heel), api.plan(heel)
    assert not cr.ok
    assert not pr.ok
    assert _stored(store, heel.id, "plan")["failures"][0]["code"] == "link_no_layer"
    back = api.load(heel.id, store)
    assert _doc(api.plan(back)) == _doc(pr)
    assert back.log[-1]["cached"]
    (rec,) = api.recommend(back)
    assert rec.patch == {"linkage": {"params": {"unit": 10.5}}}       # bolt_round's 6 mm post


# ---------------------------------------------------------------------------
# Parts
# ---------------------------------------------------------------------------


@pytest.mark.slow
def test_parts_reload_from_step_and_recheck_passes(tmp_path, design, robot, monkeypatch):
    design("quad")
    mech = robot("quad", 1.0)
    store = Store(tmp_path)
    d = api.resolve(KLANN_QUAD, store)
    rep = api.attach_build(d, mech, 1.0)
    assert rep.ok
    manifest = store.read_report(d.id, "build")
    assert manifest["t"] == 1.0
    assert manifest["n_parts"] == len(manifest["parts"]) == len(d.parts)
    files = [e for e in manifest["parts"] if e["file"]]
    refs = [e for e in manifest["parts"] if e["same_as"]]
    assert manifest["files"] == len(files) == len({e["file"] for e in files})
    assert len(files) + len(refs) == len(d.parts)
    assert len(refs) > 40                                    # the whole right side
    assert all(e["name"].startswith("R.") and e["same_as"] == "L." + e["name"][2:]
               and e["mirror"] for e in refs)
    assert all((store.dir(d.id) / "build" / e["file"]).is_file() for e in files)
    assert manifest["fastened"]
    assert "epoxy_2part" in {b["key"] for b in manifest["bom_extras"]}   # the barrels' (no CA)

    fabricated = _count(monkeypatch, api.building, "fabricate_side")
    back = api.load(d.id, store)
    br = api.build(back)
    assert br.ok
    assert fabricated == []
    assert next(e for e in back.log if e["op"] == "build")["cached"]
    assert back.build_t == 1.0
    assert set(back.parts) == set(d.parts)
    assert br.n_parts == rep.n_parts
    assert br.counts == rep.counts
    assert br.mass_g == pytest.approx(rep.mass_g, abs=0.01)
    assert br.envelope_mm == pytest.approx(rep.envelope_mm, abs=1e-3)
    for name, part in d.parts.items():
        got = back.parts[name]
        assert got.mass_g == pytest.approx(part.mass_g, rel=1e-6), name
        assert got.volume_mm3 == pytest.approx(part.volume_mm3, rel=1e-6), name
        assert np.allclose(got.pose, part.pose)
        assert got.layers == part.layers
        assert (got.group, got.side, got.fab, got.material, got.bom_key, got.rigid_with) == \
            (part.group, part.side, part.fab, part.material, part.bom_key, part.rigid_with)
        assert not got.edited
    r = back.mech.body("R.b1_leg0").placed_part().bounding_box()
    left = back.mech.body("L.b1_leg0").placed_part().bounding_box()
    assert pytest.approx(-left.max.Z) == r.min.Z
    assert back.mech.meta["mid_plane"] == pytest.approx(mech.meta["mid_plane"])
    assert back.mech.body("L.servo").color == mech.body("L.servo").color
    rr = api.recheck(back)
    assert rr.ok
    assert rr.clashes == []
    assert rr.bad_solids == []


def test_a_build_at_another_angle_replaces_the_stored_one(tmp_path, side):
    store = Store(tmp_path)
    d = api.resolve(KLANN_SINGLE, store)
    api.attach_build(d, side("single", 1.0), 1.0)
    assert store.read_report(d.id, "build")["t"] == 1.0
    n = len(list((store.dir(d.id) / "build" / "parts").iterdir()))
    assert n == len(d.parts)                                 # one side: no mirrors
    back = api.load(d.id, store)
    api.attach_build(back, side("single", 2.0), 2.0)
    assert store.read_report(d.id, "build")["t"] == 2.0
    assert len(list((store.dir(d.id) / "build" / "parts").iterdir())) == n
    again = api.load(d.id, store)
    assert api.build(again, 2.0).ok
    assert again.log[-1]["cached"]


# ---------------------------------------------------------------------------
# Another engine version
# ---------------------------------------------------------------------------


def test_a_store_from_another_engine_reverifies_the_plan_and_rebuilds_the_rest(
        tmp_path, design, side, monkeypatch):
    design("single")
    store = Store(tmp_path)
    d = api.resolve(KLANN_SINGLE, store)
    api.check(d)
    pr = api.plan(d)
    api.attach_build(d, side("single", 1.0), 1.0)
    real = d.engine_version
    # (load reads it in api.store_ops, resolve in stages.resolve)
    monkeypatch.setattr(api.store_ops, "engine_version", lambda: "0.0.0+fake")
    monkeypatch.setattr(stages_resolve, "engine_version", lambda: "0.0.0+fake")

    solved = _count(monkeypatch, stages_planning, "design_side")
    verified = _count(monkeypatch, stages_planning, "verify_plan")
    # every fabrication counted; its parts the test cache's (the same design at t = 1)
    fabricated = _count(monkeypatch, api.building, "fabricate_side",
                        instead=lambda d, mech, ties=(): _api.own(
                            side("single", 1.0)))
    back = api.load(d.id, store)
    assert back.engine_version == "0.0.0+fake"
    assert any("recorded under engine " + real in w for w in back.warnings)
    cr = api.check(back)
    assert cr.ok
    assert not back.log[-1]["cached"]                       # recomputed, not trusted
    got = api.plan(back)
    assert got.ok
    assert solved == []                                      # re-verified, not re-solved
    assert len(verified) == 1
    assert len(verified[0]) == 2                             # fresh sampling: (plan, tmpl)
    assert got.layers == pr.layers
    assert got.top == pr.top
    assert got.route == pr.route
    assert got.reused == "store"
    assert got.optimal is False
    assert "re-verified under engine 0.0.0+fake" in got.proof
    assert not back.log[-1]["cached"]
    assert store.read_report(d.id, "plan")["engine_version"] == "0.0.0+fake"
    br = api.build(back)                                     # parts of another engine: rebuilt
    assert br.ok
    assert len(fabricated) == 1
    assert store.read_report(d.id, "build")["engine_version"] == "0.0.0+fake"

    # the same spec resolved under the new engine is a new design; it seeds its plan
    # from the old one's record and verifies it
    new = api.resolve(KLANN_SINGLE, store)
    assert new.id != d.id
    got = api.plan(new)
    assert got.ok
    assert got.reused == d.id
    assert solved == []
    assert store.read_report(new.id, "plan")["layers"] == pr.layers


def test_a_stored_plan_that_no_longer_holds_is_solved_again(tmp_path, design, monkeypatch):
    design("single")
    store = Store(tmp_path)
    d = api.resolve(KLANN_SINGLE, store)
    pr = api.plan(d)
    path = store.report_path(d.id, "plan")
    doc = json.loads(path.read_text())
    doc["layers"] = dict.fromkeys(doc["layers"], 0)           # every link in the frame plate
    path.write_text(json.dumps(doc))
    solved = _count(monkeypatch, stages_planning, "design_side")
    back = api.load(d.id, store)
    got = api.plan(back)
    assert got.ok
    assert len(solved) == 1
    assert got.layers == pr.layers
    assert got.reused is None
    assert not back.log[-1]["cached"]
    assert json.loads(path.read_text())["layers"] == pr.layers


# ---------------------------------------------------------------------------
# gc, compare, derive
# ---------------------------------------------------------------------------


def test_gc_removes_what_it_should_and_nothing_else(tmp_path):
    store = Store(tmp_path)
    a = api.resolve(KLANN_QUAD, store)
    b = api.resolve(KLANN_SINGLE, store)
    c = api.resolve({**KLANN_SINGLE, "outputs": ["step"]}, store)
    api.check(a)
    stray = store.designs / "not-a-design"
    stray.mkdir()
    (tmp_path / "notes.txt").write_text("keep me")
    with pytest.raises(ValueError, match="says what to remove"):
        api.gc(store=store)
    assert api.gc(older_than=timedelta(hours=1), store=store) == []
    assert api.gc(keep=[a, b.id], older_than=datetime.now(UTC) + timedelta(days=1),
                  store=store) == [c.id]
    assert set(store.ids()) == {a.id, b.id}
    assert api.gc(keep=[a.id], store=store) == [b.id]
    assert store.ids() == [a.id]
    assert store.read_report(a.id, "check") is not None
    assert stray.is_dir()
    assert (tmp_path / "notes.txt").read_text() == "keep me"
    assert store.gc(older_than=0) == [a.id]
    assert store.ids() == []


def test_compare_and_derive(tmp_path):
    _api.seed(HEEL_DEFAULT)            # the patch's plan, cached
    store = Store(tmp_path)
    heel = api.resolve(HEEL, store)
    (rec,) = api.recommend(heel)
    api.plan(heel)                       # recommend stops at the failing check
    child = api.derive(heel, rec.patch)
    assert child.store == store
    assert child.derived_from == heel.id
    assert child.patch == rec.patch
    assert child.resolved["linkage"]["params"]["unit"] == 10.5
    record = store.read_design(child.id)
    assert record["derived_from"] == heel.id
    assert record["patch"] == rec.patch
    assert api.load(child.id, store).derived_from == heel.id
    cards = {c["id"]: c for c in api.list_designs(store)}
    assert cards[child.id]["derived_from"] == heel.id
    assert cards[heel.id]["derived_from"] is None
    assert api.plan(child).ok
    cmp = api.compare(heel, child)
    assert cmp["spec_patch"] == rec.patch
    assert cmp["resolved_patch"] == {"linkage": {"params": {"unit": 10.5}}}
    assert cmp["derived"] == f"{child.id} derives from {heel.id}"
    assert cmp["engine_version"] is None
    assert cmp["reports"]["check"]["ok"] == {"a": False, "b": True}
    # the heel's default design: 13 layers (the keyed crank's at unit 12, removed: 15)
    assert cmp["reports"]["plan"]["n_layers"] == {"a": None, "b": 13}
    assert cmp["only_in"] == {"a": [], "b": []}
    by_id = api.compare(heel.id, child.id, store)
    assert by_id["spec_patch"] == cmp["spec_patch"]
    assert by_id["reports"]["plan"]["ok"] == {"a": False, "b": True}
    same = api.compare(child, child)
    assert same["spec_patch"] == {}
    assert same["reports"]["plan"] == {}
    # derive with a no-op patch is the same design
    assert api.derive(child, {}).id == child.id
    # merge patches: null removes, and the smallest patch round-trips
    a, b = heel.spec.to_dict(), child.spec.to_dict()
    assert apply_patch(a, merge_patch(a, b)) == b
    assert apply_patch(b, merge_patch(b, a)) == a
    assert merge_patch({"x": {"y": 1, "z": 2}}, {"x": {"y": 1}}) == {"x": {"z": None}}
    assert apply_patch({"x": {"y": 1, "z": 2}}, {"x": {"z": None}}) == {"x": {"y": 1}}
    assert diff_json({"rows": [{"requirement": "a", "value": 1}, {"requirement": "b", "value": 2}],
                      "seconds": 1},
                     {"rows": [{"requirement": "b", "value": 3}], "seconds": 2}) == {
        "rows.a": {"a": {"requirement": "a", "value": 1}, "b": None},
        "rows.b.value": {"a": 2, "b": 3}}


@pytest.mark.slow
def test_export_lands_in_the_store_and_is_reused(tmp_path, side, monkeypatch):
    store = Store(tmp_path)
    d = api.resolve({**KLANN_SINGLE, "outputs": ["step", "bom"]}, store)
    api.attach_build(d, side("single", 1.0), 1.0)
    rep = api.export(d)
    assert rep.ok
    assert rep.out_dir == str(store.exports_dir(d.id).resolve())
    assert (store.exports_dir(d.id) / "klann.step").is_file()
    assert (store.exports_dir(d.id) / "manifest.json").is_file()
    assert _stored(store, d.id, "export")["files"] == rep.files
    steps = _count(monkeypatch, type(d.mech), "export_step")
    again = api.export(d)                                    # the handle's memo
    assert again is rep
    assert steps == []
    back = api.load(d.id, store)
    assert api.export(back).files == rep.files
    assert back.log[-1]["cached"]
    assert back.mech is None                                 # no build needed for a hit


def test_verify_levels_do_not_evict_each_other(tmp_path):
    """The store keeps one verify report per level beside the latest: a quick verify after
    a standard one doesn't cost the standard one again."""
    from spiderpig.verify import VerifyReport

    store = Store(tmp_path)
    d = api.resolve(KLANN_QUAD, store)
    std = VerifyReport("standard", seconds=1.0)
    api._commit(d, "verify", std, op="verify:standard")
    quick = VerifyReport("quick", seconds=0.1)
    api._commit(d, "verify", quick, op="verify:quick")
    assert store.read_report(d.id, "verify")["level"] == "quick"                  # the latest
    assert store.read_report(d.id, "verify", "standard")["level"] == "standard"
    assert store.read_report(d.id, "verify", "quick")["level"] == "quick"
    assert store.stages(d.id)["verify"]["level"] == "quick"
    back = api.load(d.id, store)
    got = api._cached(back, "verify", VerifyReport, op="verify:standard", level="standard")
    assert got is not None
    assert got.level == "standard"
    assert back.log[-1] == dict(back.log[-1], op="verify:standard", cached=True)
    got = api._cached(back, "verify", VerifyReport, op="verify:quick", level="quick")
    assert got is not None
    assert got.level == "quick"
    assert api._cached(back, "verify", VerifyReport, op="verify:full", level="full") is None
    # a store written before the per-level copies: the latest still serves its own level
    store.report_path(d.id, "verify", "quick").unlink()
    fresh = api.load(d.id, store)
    assert api._cached(fresh, "verify", VerifyReport, level="quick").level == "quick"


def test_the_engine_version_is_read_back_from_its_digest_cache_only_while_it_holds(
        tmp_path, monkeypatch):
    """``engine_version`` keeps its digest on disk under a signature of the sources it
    hashes: a later process reads the very string it computed, and an entry whose
    signature isn't this tree's is computed afresh (and rewritten)."""
    from spiderpig import design

    code_digest = design._code_digest
    monkeypatch.setenv(design.DIGEST_CACHE_ENV, str(tmp_path))
    monkeypatch.setattr(design, "_ENGINE_VERSION", [])
    computed = design.engine_version()                     # parsed, then written
    (entry,) = tmp_path.glob("*.json")
    assert json.loads(entry.read_text())["version"] == computed
    monkeypatch.setattr(design, "_ENGINE_VERSION", [])
    monkeypatch.setattr(design, "_code_digest", lambda src: pytest.fail("parsed again"))
    assert design.engine_version() == computed             # read back
    doc = json.loads(entry.read_text())
    entry.write_text(json.dumps({"signature": doc["signature"][::-1], "version": "0.0.0+x"}))
    monkeypatch.setattr(design, "_ENGINE_VERSION", [])
    monkeypatch.setattr(design, "_code_digest", code_digest)
    assert design.engine_version() == computed             # not this tree's: computed again
    assert json.loads(entry.read_text())["version"] == computed
    monkeypatch.setenv(design.DIGEST_CACHE_ENV, "off")
    assert design._digest_cache([], "", "") is None
