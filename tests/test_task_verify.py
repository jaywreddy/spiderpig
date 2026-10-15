"""A task carried through to verify (2026-10-14): a candidate's spec carries its task (the
Spec's ``task`` section, in the design's id; a spec without one keeps its id), and
``verify`` re-checks the task's geometry on the design, the ``task.*`` rows: a place task's
area and height against the turret plan's bench reach, a trace task's path by an arm's
inverse kinematics (its band plus the budget's lost motion) or a cranked output's check,
each at an honest tier (a met bound confirms nothing)."""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any, cast

import numpy as np
import pytest

from spiderpig import api, linkage, urdf, verify
from spiderpig.api import tasks
from spiderpig.api.reports import PrecisionReport
from spiderpig.spec import Spec, SpecErrors, spec_schema, validate
from spiderpig.stages.resolve import second_input_note, undriven_inputs

ARM = "palletizer_reach"
LINE = {"do": "trace", "path_mm": [[0, 0], [50, 0.2], [100, 0]], "tol_mm": 0.5}


def _rows(rows) -> dict[str, verify.Row]:
    return {r.requirement: r for r in rows}


# -- the spec carries its task ----------------------------------------------------------


@pytest.mark.no_fabricate
def test_a_spec_carries_its_task_into_the_id():
    base = {"kind": "arm", "linkage": {"key": ARM}}
    task = {"do": "place", "area_mm": [100, 60], "repeatability_mm": 0.1}
    plain, carried = api.resolve(base, None), api.resolve({**base, "task": task}, None)
    assert "task" not in plain.resolved                     # (no task: the id it had)
    assert carried.resolved["task"]["repeatability_mm"] == {"max": 0.1}   # one document
    assert carried.id != plain.id
    assert {k: v for k, v in carried.resolved.items() if k != "task"} == plain.resolved
    assert api.resolve({**base, "task": {**task, "repeatability_mm": {"max": 0.1}}},
                       None).id == carried.id               # a bare number is its target
    assert Spec.from_dict(carried.spec.to_dict()) == carried.spec
    assert "task" in spec_schema()["properties"]
    from spiderpig.design import side_hash
    assert side_hash(carried.resolved) == side_hash(plain.resolved)   # one plan to share


@pytest.mark.no_fabricate
def test_a_task_the_linkage_cant_do_is_refused():
    errs = {e.path: e for e in validate({"kind": "walker", "linkage": {"key": "strider"},
                                         "task": {"do": "place", "area_mm": [10, 10]}})}
    assert "has feet, no output" in errs["task.do"].message
    assert errs["task.do"].allowed == ("walk",)
    errs = {e.path: e for e in validate({"linkage": {"key": "hoecken"},
                                         "task": {"do": "trace", "strok_mm": 3}})}
    assert errs["task.strok_mm"].nearest == "stroke_mm"


@pytest.mark.no_fabricate
def test_a_candidates_spec_carries_the_task():
    from spiderpig.task import Task

    task = Task.from_dict(LINE)
    for key in (ARM, "hoecken"):
        lk = linkage.get(key)
        spec = tasks._spec(tasks.Candidate(key, lk.name, lk.kind, "single"), task)
        assert spec["task"] == task.to_dict()
        assert not validate(spec)
        same = {"linkage": {"key": key}}           # the same machine, its controller named
        if "constructions" in spec:                # (a mechanism's board, 2026-10-14)
            same["constructions"] = spec["constructions"]
        assert tasks._body(api.resolve(spec, None).resolved) == tasks._body(
            api.resolve(same, None).resolved)                         # the store's lend


# -- the URDF and the five-bar's second input -----------------------------------------


@pytest.mark.no_fabricate
def test_resolve_refuses_a_urdf_its_linkage_cant_have():
    with pytest.raises(SpecErrors) as e:
        api.resolve({"kind": "walker", "linkage": {"key": "strider"},
                     "outputs": ["step", "urdf"]}, None)
    (err,) = e.value.errors
    assert err.path == "outputs[1]"
    assert err.message == urdf.unsupported(linkage.get("strider"))
    assert "urdf" not in (err.allowed or ())
    assert api.resolve({"kind": "arm", "linkage": {"key": ARM}, "outputs": ["urdf"]}, None)


@pytest.mark.no_fabricate
def test_a_second_input_is_driven_where_it_declares_its_drive():
    """The registered five-bar's ``t2`` declares no drive: the screen excludes it from a
    place task for its axes, a trace task's candidate fails at ``drive``, and resolve warns
    with the same words; a five-bar whose ``t2`` declares one has nothing undriven (its
    JointDrive is built: construction._joint_drives)."""
    from dataclasses import replace

    five = linkage.get("five_bar")
    assert undriven_inputs(five) == ("t2",)
    note = second_input_note(five)
    assert note.startswith("five_bar has 2 inputs (t, t2) and t2 declares no drive")
    assert "five_bar" not in note.split("mechanisms are ")[1]
    assert "t2 declaring no drive" in tasks._drivable(five)
    driven = replace(five, key="five_bar_task_driven", drives={"t2": ("b3", "O2")})
    assert undriven_inputs(driven) == ()
    assert tasks._drivable(driven) == ""
    assert undriven_inputs(linkage.get(ARM)) == ()           # (an arm's: all its drives)


# -- the rows: place ------------------------------------------------------------------


def _bench(lo: float, hi: float, yaw=(-150.0, 150.0), h_top: float = 80.0) -> dict:
    r = np.linspace(lo - 20, hi + 20, 400)
    return {"lo": lo, "hi": hi, "yaw_deg": yaw, "keep_r": lo - 10, "note": "a note",
            "r_mm": r, "h_mm": np.where(r < (lo + hi) / 2, h_top, h_top / 4)}


@pytest.mark.no_fabricate
def test_a_place_tasks_area_and_height_against_the_bench(monkeypatch):
    task = {"do": "place", "area_mm": [100, 60], "height_mm": 30}
    d = api.resolve({"kind": "arm", "linkage": {"key": ARM}, "task": task}, None)
    assert verify.task_requirements(d) == {"task.area_mm", "task.height_mm"}
    monkeypatch.setattr(tasks, "bench_reach", lambda design: _bench(120.0, 260.0))
    rows = _rows(verify.task_rows(d))
    area, height = rows["task.area_mm"], rows["task.height_mm"]
    assert (area.passed, area.tier, area.hard) == (True, "estimated", True)
    assert cast(float, area.value) > 0
    assert "from 120 to 260 mm out" in area.detail
    assert height.passed                     # the area sits where the tool lifts 80 mm
    monkeypatch.setattr(tasks, "bench_reach",
                        lambda design: _bench(120.0, 260.0, h_top=20.0))
    assert not _rows(verify.task_rows(d))["task.height_mm"].passed
    monkeypatch.setattr(tasks, "bench_reach", lambda design: _bench(120.0, 150.0))
    small = _rows(verify.task_rows(d))["task.area_mm"]
    assert (small.passed, small.hard) == (False, True)
    assert "misses by" in small.detail
    never = {**_bench(0, 0), "lo": float("nan"), "hi": float("nan"),
             "note": "the tool's tip stops 12.0 mm over the bench (a riser under the slides)"}
    monkeypatch.setattr(tasks, "bench_reach", lambda design: never)
    rows = _rows(verify.task_rows(d))
    assert not rows["task.area_mm"].passed
    assert "riser" in rows["task.area_mm"].detail
    pts = api.resolve({"kind": "arm", "linkage": {"key": ARM},
                       "task": {"do": "place", "points_mm": [[0, 0], [80, 0], [0, 50, 10]]}},
                      None)
    assert verify.task_requirements(pts) == {"task.points_mm", "task.height_mm"}


@pytest.mark.no_fabricate
def test_an_arms_side_alone_places_nothing_over_an_area(monkeypatch):
    """No turret under it: its output's reach about its shoulder is a bound, and with no
    yaw it sweeps a plane: the area is refuted."""
    monkeypatch.setattr(tasks, "bench_reach", lambda design: None)
    d = api.resolve({"kind": "arm", "linkage": {"key": ARM}, "legs": {"robot": False},
                     "task": {"do": "place", "area_mm": [100, 60]}}, None)
    (row,) = verify.task_rows(d)
    assert (row.passed, row.tier, row.hard) == (False, "bound", True)
    assert "no yaw" in row.detail


# -- the rows: trace ------------------------------------------------------------------


@pytest.mark.no_fabricate
def test_a_cranked_outputs_check_against_the_path():
    lk = next(k for k in linkage.available("mechanism")
              if getattr(linkage.get(k).output, "motion", "") == "line"
              and not undriven_inputs(linkage.get(k)))
    d = api.resolve({"linkage": {"key": lk}, "task": LINE}, None)
    assert verify.task_requirements(d) == {"task.path_mm"}
    check = SimpleNamespace(output={"stroke_mm": 120.0, "straightness_mm": 0.3})
    (row,) = verify.task_rows(d, check)
    assert (row.passed, row.tier, row.hard) == (True, "measured", True)
    bent = SimpleNamespace(output={"stroke_mm": 120.0, "straightness_mm": 0.8})
    assert not verify.task_rows(d, bent)[0].passed            # past the tol
    short = SimpleNamespace(output={"stroke_mm": 60.0, "straightness_mm": 0.1})
    assert not verify.task_rows(d, short)[0].passed
    curve = api.resolve({"linkage": {"key": lk},
                         "task": {"do": "trace", "path_mm": [[0, 0], [50, 30], [100, 0]]}},
                        None)
    (row,) = verify.task_rows(curve, check)
    assert (row.tier, row.passed, row.hard) == ("bound", True, False)   # its length only


@pytest.mark.no_fabricate
def test_an_arm_traces_the_tasks_path_its_lost_motion_added():
    d = api.resolve({"kind": "arm", "linkage": {"key": ARM}, "task": LINE}, None)
    assert verify.task_requirements(d) == {"task.path_mm", "task.straightness_mm"}
    assert "motion" not in d.spec.to_dict()            # (an arm's line is its drives')
    rows = _rows(verify.task_rows(d))
    path, line = rows["task.path_mm"], rows["task.straightness_mm"]
    assert (path.passed, path.tier, path.hard) == (True, "measured", True)
    assert (line.tier, line.passed, line.hard) == ("bound", True, False)  # confirms nothing
    band = cast(float, line.value)
    d.reports["precision"] = PrecisionReport(complete=True,
                                             metrics={"repeatability_mm": 0.1})
    line = _rows(verify.task_rows(d))["task.straightness_mm"]
    assert (line.tier, line.passed, line.hard) == ("estimated", True, True)
    assert line.value == pytest.approx(band + 0.2, abs=1e-3)
    d.reports["precision"] = PrecisionReport(complete=True, metrics={"repeatability_mm": 0.3})
    assert not _rows(verify.task_rows(d))["task.straightness_mm"].passed
    far = api.resolve({"kind": "arm", "linkage": {"key": ARM},
                       "task": {"do": "trace", "stroke_mm": 2000}}, None)
    (row,) = verify.task_rows(far)
    assert (row.requirement, row.passed, row.value) == ("task.stroke_mm", False, None)


@pytest.mark.no_fabricate
def test_a_met_bound_leaves_the_task_unverified():
    d = api.resolve({"kind": "arm", "linkage": {"key": ARM}, "task": LINE}, None)
    rep = verify.VerifyReport("quick")
    rep.rows = verify.task_rows(d)
    done = verify._done(d, rep, 0.0)
    assert "task.straightness_mm" in done.unverified        # a bound: informational
    assert "task.path_mm" not in done.unverified
    assert done.ok


# -- end to end: candidates -> resolve -> verify -----------------------------------------


@pytest.mark.slow     # (the arm's plan and its side fabricated: minutes cold, the caches')
def test_a_place_task_is_confirmed_on_the_approved_arm(tmp_path):
    from spiderpig.config import BuildConfig
    from tests import _api

    _api.seed(BuildConfig(linkage=ARM))         # (the plan from the test cache)
    rep = api.candidates({"do": "place", "area_mm": [100, 60], "height_mm": 20}, tmp_path)
    best = next(c for c in rep.candidates if c.linkage == ARM)
    d = api.resolve(best.spec, tmp_path)
    assert d.spec.task is not None
    v = api.verify(d, "standard")
    rows = _rows(v.rows)
    for name in ("task.area_mm", "task.height_mm"):
        assert rows[name].passed, rows[name].describe()
        assert rows[name].tier == "estimated"
    assert not {"task.area_mm", "task.height_mm"} & set(v.unverified)
    assert v.ok, v.describe()


@pytest.mark.slow     # (a mechanism's plan, build and budget: ~20 s, the caches')
def test_a_trace_task_is_confirmed_on_a_mechanism(tmp_path):
    from tests import _api

    _api.seed(_api.HOECKEN_CFG)
    task = {"do": "trace", "path_mm": [[0, 0], [60, 0]], "tol_mm": 0.5}
    rep = api.candidates(task, tmp_path)
    best = rep.candidates[0]
    assert best.linkage == "hoecken"
    d = api.resolve(best.spec, tmp_path)
    v = api.verify(d, "standard")
    row = _rows(v.rows)["task.path_mm"]
    assert (row.passed, row.tier, row.hard) == (True, "measured", True), row.describe()
    assert "task.path_mm" not in v.unverified
    assert v.ok, v.describe()
    assert cast(Any, api.load(d.id, tmp_path)).spec.task == d.spec.task   # stored with it


@pytest.mark.no_fabricate
def test_a_place_task_needs_three_axes_and_a_mechanism_candidate_names_its_board():
    """A one-input mechanism has an output but not the three positional axes a place task
    takes: resolve refuses it with that reason (review round 3); a trace task's mechanism
    candidate names the board that speaks its servo's protocol (its kind leaves it none)."""
    errs = validate({"linkage": {"key": "hoecken"}, "task": {"do": "place",
                                                            "area_mm": [50, 50]}})
    assert any("positional axes" in e.message for e in errs)
    walker = validate({"linkage": {"key": "strider"}, "task": {"do": "place",
                                                              "area_mm": [50, 50]}})
    assert any("no output" in e.message for e in walker)
    from spiderpig.task import Task

    lk = linkage.get("hoecken")
    spec = tasks._spec(tasks.Candidate("hoecken", lk.name, lk.kind, "single"),
                       Task.from_dict(LINE))
    assert spec["constructions"]["controller"] == "esp32_bus_servo"
    arm = linkage.get(ARM)
    spec = tasks._spec(tasks.Candidate(ARM, arm.name, arm.kind, "single"),
                       Task.from_dict(LINE))
    assert "controller" not in spec.get("constructions", {})      # an arm's default stands
