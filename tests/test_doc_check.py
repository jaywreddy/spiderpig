"""The doc check (``tests/doc_check.py``): it resolves real names, finds the drift the docs
audit found, and rejects a made-up name of every kind."""

from __future__ import annotations

import pytest

from tests import doc_check


@pytest.fixture(scope="module")
def index():
    return doc_check.default_index()


@pytest.mark.parametrize("token", [
    "stack.StackProblem.solve", "spiderpig.config.BuildConfig", "config.default_module",
    "construction.robot.ASSEMBLY", "BoltCrank.for_sheet", "pivots.standoff",
    "api.plan_config(config, store)", "tests.cache.cached_design", "os.replace",
    "mise run test-quick", "mise run gate", "spiderpig build", "--profile", "--no-profile",
    "--regen", "SPIDERPIG_STORE", "$SPIDERPIG_STORE", "SPIDERPIG_WORKERS=0",
    "spiderpig/stack/search.py", "tests/cache.py", "construction/crank/bolt.py", "docs/agentlib/",
    "tests/fixtures/<module>/<name>.json", "design_side", "BuildConfig()", "gap_sink",
    "/api/glb/{mode}",
])
def test_real_names_resolve(index, token):
    kind, status, why = doc_check.check_token(index, token)
    assert status in ("ok", "lenient", "skip"), (token, kind, why)


@pytest.mark.parametrize("token", [
    "stack.frobnicate_layers",              # a module's missing function
    "BoltCrank.frobnicate",                 # a class's missing member
    "spiderpig.nosuchmodule.thing",
    "frobnicate_widget_zz",                 # a made-up identifier
    "FrobnicatorZz()",
    "mise run no-such-task",
    "spiderpig frobnicate",
    "--no-such-flag-zz",
    "$SPIDERPIG_NOPE_ZZ",
    "spiderpig/nope_zz.py",
    "tests/test_stack.py::test_no_such_test_zz",
    "/api/nope_zz",
])
def test_a_made_up_name_is_a_miss(index, token):
    kind, status, why = doc_check.check_token(index, token)
    assert status == "miss", (token, kind, why)
    assert why


def test_the_known_drift_is_found(index):
    """The docs audit's misses (2026-10-07): SCOPE.md's dead parsers and cache variable,
    API.md's ``mcp.Client``."""
    misses = {r.token for r in doc_check.check(
        ["docs/agentlib/SCOPE.md", "docs/agentlib/API.md"], index, allow=set())}
    for token in ("walk.make_config", "bake.build_config", "mcp.Client(server)",
                  "$SPIDERPIG_CACHE"):
        assert token in misses, token


def test_report_only_unless_strict(tmp_path, monkeypatch, capsys):
    doc = tmp_path / "doc.md"
    doc.write_text("Uses `stack.frobnicate_layers` and `mise run test-quick`.\n\n"
                   "```bash\nmise run no-such-task-zz\n```\n")
    monkeypatch.setattr(doc_check, "ALLOW_FILE", tmp_path / "none.txt")
    assert doc_check.main([str(doc)]) == 0
    out = capsys.readouterr().out
    assert "stack.frobnicate_layers" in out
    assert "mise run no-such-task-zz" in out          # code blocks' tasks are checked too
    assert doc_check.main([str(doc), "--strict"]) == 1


def test_the_allow_list_holds_a_miss(tmp_path, index):
    doc = tmp_path / "doc.md"
    doc.write_text("`frobnicate_widget_zz` and `stack.frobnicate_layers`\n")
    allow = tmp_path / "allow.txt"
    allow.write_text("# comment\nfrobnicate_widget_zz\n")
    results = doc_check.check([str(doc)], index, doc_check.load_allow(allow), every=True)
    status = {r.token: r.status for r in results}
    assert status == {"frobnicate_widget_zz": "allowed", "stack.frobnicate_layers": "miss"}


@pytest.mark.parametrize("token", ["use_recorded_foot_z", "compute_mass", "bolt_captive",
                                   "pdot_ix"])
def test_docstrings_and_comments_name_nothing(index, token):
    """Each word stands only in a docstring or a comment of the code (W0 review, round 1):
    the doc check doesn't take it for a name."""
    kind, status, why = doc_check.check_token(index, token)
    assert status == "miss", (token, kind, why)
