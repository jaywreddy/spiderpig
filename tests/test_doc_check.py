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
    "construction.assembly.ROBOT_ORDER", "BoltCrank.for_sheet", "pivots.standoff",
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


def test_the_known_drift_is_found(tmp_path, index):
    """The docs audit's misses (2026-10-07, fixed in the docs by W7): SCOPE.md's dead parsers
    and cache variable, API.md's ``mcp.Client``, CLAUDE.md's ``j_last``."""
    tokens = ("walk.make_config", "bake.build_config", "mcp.Client(server)",
              "$SPIDERPIG_CACHE", "j_last", "spiderpig/stack.py")
    doc = tmp_path / "drift.md"
    doc.write_text("".join(f"- `{t}`\n" for t in tokens))
    misses = {r.token for r in doc_check.check([str(doc)], index, allow=set())}
    assert misses == set(tokens)


def test_history_is_not_checked(index):
    """The dated records under ``docs/history/`` keep the names of their day (W7)."""
    assert doc_check.check(["docs/history/AUDIT.md"], index, allow=set(), every=True) == []
    assert not any(d.startswith(doc_check.HISTORY) for d in doc_check.DEFAULT_DOCS)


def test_the_default_docs_have_no_miss(index):
    """``mise run doc-check -- --strict`` (CI's blocking step, W7)."""
    misses = doc_check.check(list(doc_check.DEFAULT_DOCS), index)
    assert not misses, [f"{r.doc}:{r.line} `{r.token}`: {r.why}" for r in misses]


def test_designs_doc_from_a_fake_snapshot(tmp_path):
    """``mise run gate -- doc DIR`` (docs/agentlib/DESIGNS.md) on a hand-made snapshot."""
    import json

    from tests.gate import identity_gate

    (tmp_path / "snapshot.json").write_text(json.dumps({
        "commit": "abcdef0123", "branch": "b", "spiderpig_dirty": False,
        "designs": ["strider_double", "dwell_rocker"], "written_at": "2026-10-08T00:00:00"}))
    config = ("BuildConfig(linkage='{l}', module='{m}', robot={r}, phases=None, "
              "sheet='acrylic_3mm', frame_sheet='al5052_2mm', crank_sheet='al6061_2p5mm', "
              "link_sheets=None, servo='sts3215', pillar='standoff', pin='chicago', "
              "crank='bolt', params=Params(margin=1.0, crank='not_this'))")
    for name, linkage, module, robot, problems in (
            ("strider_double", "strider", "double", True, []),
            ("dwell_rocker", "dwell_rocker", "single", False,
             ["strength: pin:X (two-link pin (inner), 3 mm span): jam SF 0.3 (fix: shorter)"])):
        (tmp_path / f"{name}.json").write_text(json.dumps({
            "argv": ["--linkage", linkage], "config": config.format(l=linkage, m=module, r=robot),
            "plan": {"top": 13, "height": 66.464, "optimal": True, "heads": "gap",
                     "gaps": {"0": 2.7}},
            "audit": {"layers": 14, "problems": problems, "warnings": ["w (detail)"],
                      "sheets": {"acrylic_3mm": 1}, "manufacture": {"parts": 47},
                      "bom": {"items": 45, "cost_usd": 347.65, "unpriced": ["a", "b"]}},
            "parts": {"t=1": [{}] * 387}, "bom": {}}))
    text = identity_gate.designs_doc(tmp_path)
    assert "`abcdef0`" in text
    assert "2026-10-08T00:00:00" in text
    row = next(line for line in text.splitlines() if line.startswith("| `strider_double`"))
    cells = [c.strip() for c in row.strip("|").split("|")]
    assert cells == ["`strider_double`", "strider / double", "14", "66.5", "yes", "bolt",
                     "standoff", "chicago", "387", "47", "OK, 1 warning", "$347.65"]
    assert "| `dwell_rocker` | dwell_rocker / single (one side) |" in text
    assert "FAIL (1), 1 warning" in text
    assert "  - strength: pin:X: jam SF 0.3" in text      # parentheses (the fixes) dropped
    assert "(2 lines unpriced, not in the total)" in text
    out = tmp_path / "DESIGNS.md"
    assert identity_gate.main(["doc", str(tmp_path), "--out", str(out)]) == 0
    assert out.read_text() == text
    assert identity_gate.main(["doc", str(tmp_path / "nope")]) == 2


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
