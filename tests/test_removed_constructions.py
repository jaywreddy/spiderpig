"""The constructions removed on 2026-10-07 (W2, the user's decision D1) fail clearly: a
config, a spec or a stored design naming one says what replaces it, and the API and the
MCP return it as a ``Failure`` (``spec`` / ``bad_parameter``), never a traceback."""

from __future__ import annotations

import asyncio
import json

import pytest
from mcp import Client

from spiderpig import api, construction
from spiderpig.config import (
    CRANK_SHEET,
    REMOVED_CONSTRUCTIONS,
    REMOVED_PARAMS,
    BuildConfig,
    ParamError,
    removed_construction,
)
from spiderpig.design import design_id
from spiderpig.failure import Failure
from spiderpig.mcp import make_server
from spiderpig.spec import SpecErrors
from spiderpig.store import Store

KEPT = {"crank": {"bolt", "bolt_round"}, "pin": {"chicago"}, "pillar": {"standoff"}}

REMOVED = [(field, key) for field, table in REMOVED_CONSTRUCTIONS.items() for key in table]


def test_the_registries_are_the_kept_constructions():
    """D1: the cranks ``bolt`` and ``bolt_round``, the ``chicago`` pin, the one-piece
    ``standoff`` pillar; no removed key is registered, every replacement is."""
    assert set(construction.CRANKS) == KEPT["crank"]
    assert set(construction.AXLES) == KEPT["pin"] | KEPT["pillar"]
    for field, table in REMOVED_CONSTRUCTIONS.items():
        registry = construction.CRANKS if field == "crank" else construction.AXLES
        assert not set(table) & set(registry), field
        for key, (replacement, when, what) in table.items():
            assert replacement in KEPT[field], (field, key)
            assert when == "2026-10-07"
            assert what


@pytest.mark.parametrize(("field", "key"), REMOVED)
def test_a_config_naming_a_removed_construction_names_its_replacement(field, key):
    replacement = REMOVED_CONSTRUCTIONS[field][key][0]
    with pytest.raises(ParamError) as e:
        BuildConfig(**{field: key})
    msg = str(e.value)
    assert removed_construction(field, key) == (msg, replacement)
    assert f"{field} {key!r}" in msg
    assert "removed on 2026-10-07" in msg
    assert f"use {field}={replacement!r}" in msg


def test_the_kept_constructions_still_build_a_config():
    for field, keys in KEPT.items():
        for key in keys:
            assert removed_construction(field, key) is None
            assert getattr(BuildConfig(**{field: key}), field) == key
    assert BuildConfig(linkage="trotbot_heel").crank == "bolt_round"     # its own


def test_an_acrylic_crank_sheet_names_an_aluminium_one():
    """The acrylic two-plate crank went with D1: the bolt crank's plates are aluminium."""
    with pytest.raises(ParamError, match=f"crank_sheet 'acrylic_3mm' is not metal.*{CRANK_SHEET}"):
        BuildConfig(crank_sheet="acrylic_3mm")
    from spiderpig.construction.base import ConstructionError
    from spiderpig.construction.crank import BoltCrank

    with pytest.raises(ConstructionError, match="need a metal crank sheet"):
        BoltCrank().for_sheet("acrylic_3mm")


def test_a_spec_naming_a_removed_construction_is_invalid_with_the_replacement():
    spec = {"kind": "mechanism", "linkage": {"key": "hoecken_pantograph"},
            "constructions": {"crank": "keyed", "pillar": "printed", "pin": "rod"}}
    with pytest.raises(SpecErrors) as e:
        api.resolve(spec, store=None)
    errors = {err.path: err for err in e.value.errors}
    assert errors["constructions.crank"].nearest == "bolt"
    assert errors["constructions.pillar"].nearest == "standoff"
    assert errors["constructions.pin"].nearest == "chicago"
    assert "removed on 2026-10-07" in errors["constructions.crank"].message
    failure = Failure.from_exception(e.value)
    assert (failure.stage, failure.code) == ("spec", "invalid_spec")
    assert "use crank='bolt'" in failure.message
    # an acrylic crank sheet: the validator refuses it at its path, the default its nearest
    with pytest.raises(SpecErrors) as e:
        api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken_pantograph"},
                     "materials": {"crank_sheet": "acrylic_3mm"}}, store=None)
    (err,) = e.value.errors
    assert (err.path, err.nearest) == ("materials.crank_sheet", CRANK_SHEET)
    assert "'acrylic_3mm' is not metal" in err.message
    assert "acrylic_3mm" not in err.allowed
    assert set(err.allowed) >= {CRANK_SHEET, "al6061_2mm", "al5052_2mm"}


def _store_with_a_keyed_design(root, in_spec: bool) -> tuple[Store, str]:
    """A store holding a design recorded with ``crank="keyed"`` (as a store written before
    2026-10-07 holds one): a hoecken_pantograph side resolved now, its resolved spec naming
    the keyed crank (and its spec too, ``in_spec``: a spec that named it; else it was the
    linkage's default then), re-recorded under the id they hash to."""
    store = Store(root)
    design = api.resolve(api.spec_of(BuildConfig(linkage="hoecken_pantograph", robot=False)),
                         store)
    rec = store.read_design(design.id)
    spec = store.read_spec(design.id)
    rec["resolved"]["constructions"]["crank"] = "keyed"
    if in_spec:
        spec.setdefault("constructions", {})["crank"] = "keyed"
    else:
        (spec.get("constructions") or {}).pop("crank", None)
    old = design_id(rec["resolved"], rec["engine_version"])
    rec["id"] = old
    d = store.dir(old)
    d.mkdir(parents=True, exist_ok=True)
    (d / "resolved.json").write_text(json.dumps(rec))
    (d / "spec.json").write_text(json.dumps(spec))
    store.remove(design.id)
    return store, old


def test_a_spec_or_stored_design_naming_a_removed_fit_field_says_it_was_removed(tmp_path):
    """The Params fields nothing read (W5, :data:`config.REMOVED_PARAMS`): a spec giving one
    is invalid at its path with the removal; a stored design's resolved record (every field
    written in) loads at the field's last default and fails with the message off it."""
    for name in REMOVED_PARAMS:
        with pytest.raises(SpecErrors) as e:
            api.resolve({"kind": "mechanism", "linkage": {"key": "hoecken_pantograph"},
                         "fit": {name: 1.0}}, store=None)
        (err,) = e.value.errors
        assert err.path == f"fit.{name}"
        assert "was removed on 2026-10-07" in err.message
        assert Failure.from_exception(e.value).code == "invalid_spec"
    store = Store(tmp_path / "store")
    design = api.resolve(api.spec_of(BuildConfig(linkage="hoecken_pantograph", robot=False)),
                         store)
    rec = store.read_design(design.id)
    for name, (_, default, _) in REMOVED_PARAMS.items():
        rec["resolved"]["fit"][name] = default      # as a store written before W5 holds it
    assert api._config_from_resolved(rec["resolved"]) == design.config
    rec["resolved"]["fit"]["neck_d"] = 3.0
    with pytest.raises(ParamError, match=r"fit\.neck_d .* was removed on 2026-10-07"):
        api._config_from_resolved(rec["resolved"])


@pytest.mark.parametrize("in_spec", [True, False], ids=["spec", "resolved"])
def test_a_stored_keyed_design_loads_as_a_failure_naming_bolt(tmp_path, in_spec):
    store, old = _store_with_a_keyed_design(tmp_path / "store", in_spec)
    with pytest.raises(ParamError) as e:
        api.load(old, store)
    failure = Failure.from_exception(e.value)
    assert (failure.stage, failure.code) == ("spec", "bad_parameter")
    assert "crank 'keyed'" in failure.message
    assert "use crank='bolt'" in failure.message
    assert "2026-10-07" in failure.message


@pytest.mark.parametrize("in_spec", [True, False], ids=["spec", "resolved"])
def test_the_mcp_answers_a_stored_keyed_design_with_the_failure(tmp_path, in_spec):
    """Through the MCP's tools (the SDK's in-memory client): the tool is misused (``isError``)
    with the ``Failure`` document, not a traceback."""
    store, old = _store_with_a_keyed_design(tmp_path / "store", in_spec)
    server = make_server(store.root)

    async def call():
        async with Client(server) as client:
            return await client.call_tool("check", {"design": old})

    try:
        result = asyncio.run(call())
    finally:
        server.spiderpig.jobs.shutdown()
    assert result.is_error
    doc = result.structured_content
    assert doc["ok"] is False
    (failure,) = doc["failures"]
    assert (failure["stage"], failure["code"]) == ("spec", "bad_parameter")
    assert "use crank='bolt'" in failure["message"]
    assert "Traceback" not in json.dumps(doc)


@pytest.mark.parametrize(("argv", "field", "replacement"), [
    (["--crank", "keyed"], "crank", "bolt"), (["--pin", "rod"], "pin", "chicago"),
    (["--pillar", "printed"], "pillar", "standoff")])
def test_the_cli_names_a_removed_constructions_replacement(capsys, argv, field, replacement):
    """``spiderpig build --crank keyed`` and the like: argparse's usage error (exit 2) with
    the replacement and the date, not ``invalid choice``."""
    from spiderpig import cli

    with pytest.raises(SystemExit) as e:
        cli.main(["build", *argv])
    assert e.value.code == 2
    err = capsys.readouterr().err
    assert f"use {field}={replacement!r}" in err
    assert "removed on 2026-10-07" in err
    assert "invalid choice" not in err



# -- the small kept paths the removed constructions used to reach (W2 review round 1) ----


@pytest.mark.construction
def test_an_unknown_construction_names_the_registry():
    from spiderpig import construction
    from spiderpig.construction.base import ConstructionError

    with pytest.raises(ConstructionError, match=r"no crank construction 'nope'; have \['bolt', "
                                                r"'bolt_round'\]"):
        construction.crank("nope")
    with pytest.raises(ConstructionError, match=r"no axle construction 'nope'"):
        construction.axle("nope")


@pytest.mark.construction
def test_a_context_whose_config_names_no_part_sheets_cuts_everything_from_its_sheet():
    """``Context.sheet``: a config without per-part sheets (no ``frame_sheet``) cuts every
    role from its one ``sheet``."""
    import dataclasses
    import types

    from spiderpig.fabricate import side_problem, template_for

    cfg = BuildConfig(linkage="hoecken_pantograph", robot=False)
    ctx, _, _ = side_problem(template_for(cfg), cfg)
    assert ctx.sheet("frame") == cfg.frame_sheet != cfg.sheet
    bare = dataclasses.replace(ctx, config=types.SimpleNamespace(sheet="acrylic_3mm"))
    assert bare.sheet("crank") == bare.sheet("frame") == bare.sheet("link") == "acrylic_3mm"


@pytest.mark.planner
def test_a_claim_on_an_unknown_link_is_refused():
    from spiderpig.fabricate import side_problem, template_for
    from spiderpig.stack import Claim, StackProblem

    cfg = BuildConfig(linkage="hoecken_pantograph", robot=False)
    _, _, problem = side_problem(template_for(cfg), cfg)
    probe = Claim("probe", frozenset({"no_such_link"}), lambda L: [])
    with pytest.raises(ValueError, match=r"depends on unknown links \['no_such_link'\]"):
        StackProblem(problem.topo, [*problem.raw_claims, probe], problem.spec)


@pytest.mark.construction
def test_the_side_design_reports_its_linkage_checks():
    """``SideDesign.checks``: the linkage's loop checks at the design's proportions: the
    Klann's two loops (closing at C and E) close all round, each with a margin and a transmission
    angle short of a toggle."""
    from tests import cache

    _, design = cache.cached_design(BuildConfig(linkage="klann", module="single", robot=False))
    closures = [c for c in design.checks if c.kind == "closure"]
    assert [c.point for c in closures] == ["C", "E"]
    margins = [round(c.margin_mm, 2) for c in closures]
    assert margins == [21.24, 9.45]          # how far each loop stays from failing to close
    for c in closures:
        assert c.fail_fraction == 0.0, c
        assert 0 < c.angle_deg[0] <= c.angle_deg[1] < 180, c


def test_a_misshapen_spec_section_is_reported_at_its_path():
    from spiderpig.spec import validate

    errs = validate({"kind": "mechanism", "linkage": {"key": "hoecken_pantograph"},
                     "materials": {"link_sheets": ["b1"]}, "outputs": "step"})
    assert {e.path for e in errs} == {"materials.link_sheets", "outputs"}


@pytest.mark.hardware
def test_unknown_catalog_keys_read_as_themselves():
    from spiderpig.hardware.bom import _filament_name, _known

    assert _filament_name("no_such_filament") == "no_such_filament"
    assert _filament_name("pla_filament") == "PLA filament"
    assert not _known("no_such_item")


@pytest.mark.linkage
def test_the_walk_model_wants_a_foot_z_per_foot():
    from spiderpig import walk

    cfg = BuildConfig(linkage="hoecken_pantograph", robot=False)
    with pytest.raises(ValueError, match="foot z values"):
        walk.make_feet(cfg, [], [0.0])
