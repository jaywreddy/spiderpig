"""Every gate design passes its own audit (2026-10-14): the default build of each design the
identity gate holds (:data:`tests.gate.identity_gate.DESIGNS`: the walkers, the
mechanisms, the arms) has no audit problem, the gate's way (``spiderpig audit --no-sim``:
the family's pin loads). The gate reports it too (``identity_gate.report_audits``: a
snapshot's failing designs, exit 3); the slow test runs the audits themselves."""

from __future__ import annotations

import argparse
import json

import pytest

from tests.gate import identity_gate


def _snapshot(tmp_path, problems: dict[str, list[str]]):
    for name, probs in problems.items():
        (tmp_path / f"{name}.json").write_text(json.dumps({"audit": {"problems": probs}}))
    (tmp_path / "snapshot.json").write_text(json.dumps({"designs": list(problems)}))
    return tmp_path


def test_the_gate_reports_a_design_that_fails_its_audit(tmp_path, capsys):
    snap = _snapshot(tmp_path, {"strider_double": [], "klann_quad": [
        "strength: pin:C_leg1 (b3_leg1, b1_leg1; two-link pin): jam SF 0.3 at 155 N"]})
    assert identity_gate.audit_failures(snap, ["strider_double", "klann_quad"]) == {
        "klann_quad": ["strength: pin:C_leg1 (b3_leg1, b1_leg1; two-link pin): jam SF 0.3 "
                       "at 155 N"]}
    assert identity_gate.report_audits(snap, ["strider_double", "klann_quad"], known={}) == 3
    out = capsys.readouterr().out
    assert "klann_quad: FAILS ITS AUDIT (1 problems)" in out
    assert "strider_double" not in out
    why = {"klann_quad": "its long pins"}
    assert identity_gate.report_audits(snap, ["strider_double", "klann_quad"], why) == 0
    assert "klann_quad: fails its audit, known (1 problems): its long pins" in (
        capsys.readouterr().out)
    (snap / "klann_quad.json").write_text(json.dumps({"audit": {"problems": []}}))
    assert identity_gate.report_audits(snap, ["strider_double", "klann_quad"], known={}) == 0
    assert "every design passes" in capsys.readouterr().out
    assert identity_gate.report_audits(snap, ["strider_double", "klann_quad"], why) == 3
    assert "drop it from KNOWN_AUDIT_FAILURES" in capsys.readouterr().out
    assert set(identity_gate.KNOWN_AUDIT_FAILURES) <= set(identity_gate.DESIGNS)


def test_compares_exit_status_masks_neither_verdict():
    """An identity difference never hides an audit failure, nor the other way round."""
    status = identity_gate.exit_status
    assert [status(i, 0) for i in (0, 1, 2)] == [0, 1, 2]
    assert [status(i, 3) for i in (0, 1, 2)] == [3, 4, 5]


@pytest.mark.slow
@pytest.mark.parametrize("name", [
    pytest.param(n, marks=pytest.mark.xfail(reason=identity_gate.KNOWN_AUDIT_FAILURES[n],
                                            strict=True))
    if n in identity_gate.KNOWN_AUDIT_FAILURES else n for n in identity_gate.DESIGNS])
def test_every_gate_design_passes_its_own_audit(name, monkeypatch):
    """The audit of the gate's design ``name`` as the gate runs it (its options, the
    family's loads, the planner held by its node budgets alone), problems none; a design
    of ``identity_gate.KNOWN_AUDIT_FAILURES`` is expected to fail (strictly: once it
    passes, the table must lose it)."""
    import spiderpig.stack.plan as stack_plan
    from spiderpig.config import add_build_args, add_design_args, config_from_args
    from spiderpig.tools.audit import audit_module

    monkeypatch.setattr(stack_plan, "MAX_SECONDS", 3600.0)
    p = argparse.ArgumentParser()
    add_design_args(p)
    add_build_args(p)
    config = config_from_args(p.parse_args(identity_gate.DESIGNS[name]))
    rep = audit_module(config.module, config, identity_gate.TS_CONTRACT,
                       identity_gate.TS_CLASH, sim=False)
    assert rep["problems"] == [], rep["problems"]
