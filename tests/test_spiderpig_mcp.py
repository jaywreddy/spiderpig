"""The MCP server (:mod:`spiderpig.mcp`), driven through the SDK's in-memory client: every
tool is listed with schemas and hints, results are JSON with failures as data and no
solids, ``resolve`` agrees with the Python API, a stage failure carries the patch that
``derive`` applies, the long-operation path returns a manifest of files in the store,
resources and prompts read, and ``gc`` refuses to run bare."""

from __future__ import annotations

import asyncio
import json
import time
from pathlib import Path

import pytest
from mcp import Client

from spiderpig import api
from spiderpig.mcp import make_server
from spiderpig.store import Store

KLANN_SINGLE = {"kind": "walker", "linkage": {"key": "klann"},
                "legs": {"module": "single", "sides": 1}}
HEEL = {"kind": "walker", "linkage": {"key": "trotbot_heel", "params": {"unit": 7}},
        "legs": {"module": "single"}}
TOOLS = {"list_linkages", "describe", "catalog", "resolve", "check", "plan", "explain",
         "recommend", "walk", "build", "verify", "export", "compare", "derive", "get_design",
         "list_designs", "gc", "get_job", "wait_job"}
MUTATING = {"export", "gc"}


def run(coro):
    return asyncio.run(coro)


async def _call(server, name: str, **args):
    async with Client(server) as client:
        return await client.call_tool(name, args)


def call(server, name: str, **args) -> dict:
    """A tool's structured result (asserting it is JSON and not an error)."""
    result = run(_call(server, name, **args))
    assert not result.is_error, result.content
    doc = result.structured_content
    assert json.loads(json.dumps(doc)) == doc
    return doc


def misuse(server, name: str, **args) -> dict:
    """A misused tool's failure (``isError`` set, the same envelope)."""
    result = run(_call(server, name, **args))
    assert result.is_error
    doc = result.structured_content
    assert doc["ok"] is False
    assert json.loads(result.content[0].text) == doc
    (failure,) = doc["failures"]
    return failure


def _no_solids(doc) -> None:
    text = json.dumps(doc)
    assert "solid" not in text
    assert "build123d" not in text
    assert "<" not in text.replace("<=", "")           # no repr of an object


@pytest.fixture(scope="module")
def server():
    """The server over the session's store (``$SPIDERPIG_STORE``)."""
    s = make_server()
    yield s
    s.spiderpig.jobs.shutdown()


@pytest.fixture(scope="module")
def single(server, design) -> str:
    """The Klann single's id, its plan the session's."""
    design("single")
    return call(server, "resolve", spec=KLANN_SINGLE)["design"]


# ---------------------------------------------------------------------------
# Listing
# ---------------------------------------------------------------------------


def test_every_tool_is_listed_with_schemas_and_hints(server):
    async def go():
        async with Client(server) as client:
            return (await client.list_tools()).tools

    tools = {t.name: t for t in run(go())}
    assert set(tools) == TOOLS
    for name, t in tools.items():
        assert t.input_schema["type"] == "object", name
        assert t.output_schema is not None, name
        assert {"ok", "failures"} <= set(t.output_schema["properties"]), name
        assert t.description, name
        hints = t.annotations
        if name in MUTATING:
            assert hints.read_only_hint is False, name
        else:
            assert hints.read_only_hint is True, name
            assert hints.idempotent_hint is True, name
    assert tools["gc"].annotations.destructive_hint is True
    assert tools["export"].annotations.destructive_hint is False
    spec = tools["resolve"].input_schema["properties"]["spec"]
    assert spec["required"] == ["kind", "linkage"]
    assert spec["properties"]["linkage"]["properties"]["key"]["enum"][0] == "klann"
    assert spec["additionalProperties"] is False
    assert tools["verify"].input_schema["properties"]["level"]["enum"] == ["quick", "standard",
                                                                           "full"]
    failure = tools["check"].output_schema["$defs"]["FailureOut"]["properties"]
    assert {"stage", "code", "message", "culprits", "numbers", "recommendations"} <= set(failure)


def test_the_guide_says_what_is_not_in_v1(server):
    async def go():
        async with Client(server) as client:
            return (await client.read_resource("spiderpig://guide")).contents[0].text

    guide = run(go())
    assert "No `tune` and no `search`" in guide
    assert "| `motion.stride_mm` | walker | mm/rev | soft |" in guide
    assert "| `size.stack_mm` | walker/mechanism | mm | **hard** |" in guide
    assert "| `klann` | walker |" in guide
    assert "| servo | `sts3215` |" in guide
    assert str(server.spiderpig.store.root.resolve()) in guide


# ---------------------------------------------------------------------------
# resolve
# ---------------------------------------------------------------------------


def test_resolve_agrees_with_the_python_api_and_an_invalid_spec_lists_its_errors(server):
    out = call(server, "resolve", spec=KLANN_SINGLE)
    d = api.resolve(KLANN_SINGLE)
    assert out["design"] == d.id
    assert out["resolved"] == json.loads(json.dumps(d.resolved))
    assert out["module"] == "single"
    assert out["sides"] == 1
    assert out["store"] == str(Store.default().root.resolve())
    assert out["warnings"] == d.warnings
    bad = call(server, "resolve", spec={"kind": "walker", "linkage": {"key": "klan"},
                                        "motion": {"stride": {"min": 100}}, "colour": "red"})
    assert bad["ok"] is False
    (f,) = bad["failures"]
    assert (f["stage"], f["code"]) == ("spec", "invalid_spec")
    errors = {e["path"]: e for e in bad["errors"]}
    assert set(errors) == {"linkage.key", "motion.stride", "colour"}
    assert errors["linkage.key"]["nearest"] == "klann"
    assert errors["motion.stride"]["nearest"] == "stride_mm"
    assert "klann" in errors["linkage.key"]["allowed"]
    assert "design" not in bad


def test_cards(server):
    keys = [c["key"] for c in call(server, "list_linkages", kind="walker")["linkages"]]
    assert keys[0] == "klann"
    card = call(server, "describe", key="klann")["card"]
    assert card["scale_params"] == ["OA"]
    assert card["foot_path"]["lift_mm"] == pytest.approx(86.5, abs=0.1)
    f = misuse(server, "describe", key="klan")
    assert f["code"] == "unknown_linkage"
    assert "did you mean 'klann'" in f["message"]
    cat = call(server, "catalog")
    servo = next(s for s in cat["servos"] if s["key"] == "sts3215")
    assert servo["rpm_max"] == 52.0
    assert servo["mass_g"] == 55.0
    assert servo["price_usd"] is not None
    sheet = next(s for s in cat["sheets"] if s["key"] == "acrylic_3mm")
    assert sheet["thickness_mm"] == 3.0
    assert sheet["sheet_mm"] == [300.0, 300.0]
    assert sheet["price_usd"] == 10.99
    axles = {a["key"]: a for a in cat["constructions"]["axles"]}
    assert set(axles) == {"printed", "rod", "bolt", "bearing", "bushing"}
    assert axles["bolt"]["hardware"]["nut_key"]["key"] == "m3_nylock"
    assert axles["printed"]["roles"] == ["pillar", "pin"]
    assert [c["key"] for c in cat["constructions"]["cranks"]] == ["printed"]
    assert set(call(server, "catalog", category="sheets")) == {"ok", "failures", "sheets"}


# ---------------------------------------------------------------------------
# The stages on the Klann single: JSON, no solids
# ---------------------------------------------------------------------------


def test_check_plan_walk_and_verify_quick_return_json_reports(server, single):
    cr = call(server, "check", design=single)
    assert cr["ok"]
    assert cr["design"] == single
    assert [s["point"] for s in cr["steps"] if s["kind"] == "closure"] == ["C", "E"]
    assert cr["drive"]["servo"] == "sts3215"
    assert cr["crank_facts"]["hosts"]["b1"] == ["M"]
    _no_solids(cr)
    pr = call(server, "plan", design=single)
    assert pr["ok"]
    assert pr["n_layers"] == 7
    assert pr["layers"]["b1"] == 2
    assert pr["route"]["runs"] == [{"at": "M", "lo": 2, "hi": 2}]
    assert pr["optimal"] is True
    assert "inner frame plate" in pr["table"]
    _no_solids(pr)
    wr = call(server, "walk", design=single)
    assert wr["ok"]
    assert wr["feet_z_planned"]
    assert wr["metrics"]["stride_mm"] > 0
    assert wr["rows"] == [] or "pass" in wr["rows"][0]
    _no_solids(wr)
    vr = call(server, "verify", design=single, level="quick")
    assert vr["ok"]
    assert vr["level"] == "quick"
    assert "job" not in vr
    rows = {r["requirement"]: r for r in vr["rows"]}
    assert rows["program.loops_close"]["tier"] == "proven"
    assert rows["program.loops_close"]["pass"] is True
    assert rows["size.stack_mm"]["value"] == 21.0
    assert rows["motion.speed_mm_s"]["tier"] == "estimated"
    _no_solids(vr)
    assert call(server, "recommend", design=single) == {
        "ok": True, "failures": [], "design": single, "stage": None, "recommendations": []}
    assert "3. plan" in call(server, "explain", design=single)["text"]
    summary = call(server, "get_design", design=single, stage="summary")["report"]
    assert {"check", "plan", "walk", "verify"} <= set(summary["stages"])
    assert summary["verify"]["ok"] is True
    plan_doc = call(server, "get_design", design=single, stage="plan")["report"]
    assert plan_doc["layers"] == pr["layers"]
    log = call(server, "get_design", design=single, stage="log")["report"]["entries"]
    assert log[0]["op"] == "check"
    # every call loads a fresh handle, so cached reads land in the log too
    assert {"check", "plan", "walk", "verify:quick"} <= {e["op"] for e in log}
    assert any(e["cached"] for e in log)
    listed = call(server, "list_designs")
    assert single in [d["id"] for d in listed["designs"]]
    assert listed["store"] == str(Store.default().root.resolve())


def test_misuse_is_an_error_result_carrying_a_failure(server):
    assert misuse(server, "check", design="0" * 16)["code"] == "no_such_design"
    assert misuse(server, "plan", design="nope")["code"] == "bad_design_id"
    assert misuse(server, "get_design", design="0" * 16, stage="plan")["code"] == "no_such_design"
    assert misuse(server, "get_job", job="nope")["code"] == "no_such_job"
    assert misuse(server, "gc")["code"] == "gc_needs_arguments"
    f = misuse(server, "compare", a="0" * 16, b="1" * 16)
    assert f["code"] == "no_such_design"


# ---------------------------------------------------------------------------
# Failures as data: the heel at 7 mm, and the patch that plans
# ---------------------------------------------------------------------------


def test_a_stage_failure_is_a_failure_dict_whose_patch_derives_a_design_that_plans(server):
    heel = call(server, "resolve", spec=HEEL)["design"]
    cr = call(server, "check", design=heel)
    assert cr["ok"] is False
    (f,) = cr["failures"]
    assert (f["stage"], f["code"]) == ("static", "link_no_layer")
    assert f["culprits"][0]["body"] == "b7"
    assert f["numbers"]["need_mm"] == 10.0
    (rec,) = f["recommendations"]
    assert rec["patch"] == {"linkage": {"params": {"unit": 10.5}}}
    assert rec["verified"].startswith("checked: the static stage passes")
    recs = call(server, "recommend", design=heel)
    assert recs["stage"] == "static"
    assert recs["recommendations"] == [rec]
    assert call(server, "verify", design=heel, level="quick")["failures"][0]["code"] == \
        "link_no_layer"
    assert "what would clear it" in call(server, "explain", design=heel)["text"]
    child = call(server, "derive", design=heel, patch=rec["patch"])
    assert child["ok"]
    assert child["design"] != heel
    assert child["derived_from"] == heel
    assert child["patch"] == rec["patch"]
    assert child["resolved"]["linkage"]["params"]["unit"] == 10.5
    pr = call(server, "plan", design=child["design"])
    assert pr["ok"]
    assert pr["n_layers"] == 12
    cmp = call(server, "compare", a=heel, b=child["design"])
    assert cmp["spec_patch"] == rec["patch"]
    assert cmp["derived"] == f"{child['design']} derives from {heel}"
    assert cmp["reports"]["check"]["ok"] == {"a": False, "b": True}
    assert call(server, "derive", design=heel, patch={})["design"] == heel
    bad = call(server, "derive", design=heel, patch={"legs": {"module": "hex"}})
    assert bad["ok"] is False
    assert bad["errors"][0]["path"] == "legs.module"


# ---------------------------------------------------------------------------
# Long operations: build as a job, its manifest of files in the store
# ---------------------------------------------------------------------------


def test_build_through_the_long_op_path_returns_a_manifest_of_files_in_the_store(
        server, single):
    out = call(server, "build", design=single, wait_seconds=0)
    assert out["ok"]
    job = out["job"]
    assert job["state"] in ("queued", "running")
    assert job["op"] == "build"
    assert job["design"] == single
    assert "parts" not in out
    got = call(server, "wait_job", job=job["job"], seconds=300)
    assert got["ok"], got
    assert got["job"]["state"] == "done"
    assert got["job"]["seconds"] > 0
    manifest = got["job"]["result"]
    assert manifest["ok"]
    assert manifest["design"] == single
    assert manifest["t"] == 1.0
    assert manifest["n_parts"] == len(manifest["parts"]) == 28
    assert manifest["mass_g"] > 0
    assert len(manifest["envelope_mm"]) == 3
    store = Store.default()
    assert manifest["dir"] == str(store.dir(single) / "build")
    files = [p for p in manifest["parts"] if p["path"]]
    assert manifest["files"] == len(files) == 28                     # one side: no mirrors
    assert all(Path(p["path"]).is_file() and p["path"].endswith(".step") for p in files)
    b1 = next(p for p in manifest["parts"] if p["name"] == "b1")
    assert (b1["group"], b1["fab"], b1["layers"]) == ("links", "laser", [2])
    _no_solids(manifest)
    assert call(server, "get_job", job=job["job"])["job"]["state"] == "done"
    stored = call(server, "get_design", design=single, stage="build")["report"]
    assert stored["n_parts"] == 28
    assert stored["parts"][0]["path"] == manifest["parts"][0]["path"]
    # the same build again: served from the store's STEP files within the grace period
    again = call(server, "build", design=single, wait_seconds=120)
    assert again["ok"]
    assert again["job"]["state"] == "done"
    assert again["n_parts"] == 28
    assert "result" not in again["job"]


@pytest.mark.slow
def test_export_as_a_job_writes_the_files(server, single, tmp_path):
    out = call(server, "export", design=single, formats=["step", "bom"], out_dir=str(tmp_path),
               wait_seconds=0)
    got = call(server, "wait_job", job=out["job"]["job"], seconds=600)
    assert got["ok"], got
    rep = got["job"]["result"]
    assert rep["ok"]
    assert rep["out_dir"] == str(tmp_path.resolve())
    names = {Path(f).name for f in rep["files"]}
    assert {"klann.step", "bom.csv", "bom.md", "bom.json", "manifest.json"} <= names
    assert all(Path(f).is_file() for f in rep["files"])
    assert rep["manifest"]["design"] == single
    assert rep["manifest"]["plan"]["layers"] == 7
    bad = run(_call(server, "export", design=single, formats=["pdf"]))   # the input schema
    assert bad.is_error
    assert "pdf" in bad.content[0].text


# ---------------------------------------------------------------------------
# Resources, prompts, gc
# ---------------------------------------------------------------------------


def test_resources_and_prompts_read(server, single):
    async def go():
        async with Client(server) as client:
            listed = await client.list_resources()
            templates = await client.list_resource_templates()
            reads = {uri: (await client.read_resource(uri)).contents[0]
                     for uri in ("spiderpig://schema/spec", "spiderpig://linkages/klann",
                                 "spiderpig://catalog/servos",
                                 f"spiderpig://designs/{single}/plan",
                                 f"spiderpig://designs/{single}/summary")}
            prompts = await client.list_prompts()
            walker = await client.get_prompt("design_walker", {"goal": "a 60 mm stride"})
            diag = await client.get_prompt("diagnose", {"design": single})
            it = await client.get_prompt("iterate", {"design": single, "metric": "bob_mm"})
            try:
                await client.read_resource(f"spiderpig://designs/{single}/nope")
            except Exception as e:  # noqa: BLE001 - the SDK's protocol error
                missing = str(e)
            else:
                missing = None
            return listed, templates, reads, prompts, walker, diag, it, missing

    listed, templates, reads, prompts, walker, diag, it, missing = run(go())
    uris = {str(r.uri) for r in listed.resources}
    assert {"spiderpig://guide", "spiderpig://schema/spec", "spiderpig://linkages/klann",
            "spiderpig://catalog/servos", "spiderpig://catalog/constructions"} <= uris
    assert {str(t.uri_template) for t in templates.resource_templates} == {
        "spiderpig://linkages/{key}", "spiderpig://catalog/{category}",
        "spiderpig://designs/{design}/{stage}"}
    schema = json.loads(reads["spiderpig://schema/spec"].text)
    assert schema["required"] == ["kind", "linkage"]
    assert reads["spiderpig://schema/spec"].mime_type == "application/json"
    assert json.loads(reads["spiderpig://linkages/klann"].text)["key"] == "klann"
    assert json.loads(reads["spiderpig://catalog/servos"].text)["servos"][0]["key"] == "sts3215"
    assert json.loads(reads[f"spiderpig://designs/{single}/plan"].text)["n_layers"] == 7
    assert json.loads(reads[f"spiderpig://designs/{single}/summary"].text)["id"] == single
    assert missing is not None
    assert "unknown stage" in missing
    assert {p.name for p in prompts.prompts} == {"design_walker", "diagnose", "iterate"}
    assert "a 60 mm stride" in walker.messages[0].content.text
    assert walker.messages[0].role == "user"
    assert single in diag.messages[0].content.text
    assert "bob_mm" in it.messages[0].content.text


def test_gc_refuses_bare_and_removes_what_it_is_told(tmp_path):
    server = make_server(tmp_path)
    try:
        assert server.spiderpig.store == Store(tmp_path)
        a = call(server, "resolve", spec=KLANN_SINGLE)["design"]
        b = call(server, "resolve", spec={**KLANN_SINGLE, "outputs": ["step"]})["design"]
        assert misuse(server, "gc")["code"] == "gc_needs_arguments"
        assert call(server, "gc", older_than_seconds=3600) == {"ok": True, "failures": [],
                                                                "removed": []}
        assert call(server, "gc", keep=[a])["removed"] == [b]
        assert [d["id"] for d in call(server, "list_designs")["designs"]] == [a]
    finally:
        server.spiderpig.jobs.shutdown()


def test_a_cold_resolve_to_verify_quick_round_trip_is_fast(tmp_path, design):
    design("single")
    server = make_server(tmp_path)
    try:
        t0 = time.time()
        d = call(server, "resolve", spec=KLANN_SINGLE)["design"]
        vr = call(server, "verify", design=d, level="quick")
        seconds = time.time() - t0
        assert vr["ok"]
        assert seconds < 30, seconds
    finally:
        server.spiderpig.jobs.shutdown()
