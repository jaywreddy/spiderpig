"""Tasks and their candidates (:mod:`spiderpig.task`, :func:`spiderpig.api.candidates`): a
task validates whole, its numbers become a spec's targets, and the linkages are read by what
they declare (feet, an output and its motion, ranged inputs, a turntable's yaw), ranked with
the numbers behind each verdict, the store's measurements read where it has them."""

from __future__ import annotations

import math

import pytest

from spiderpig import api, linkage
from spiderpig.api import tasks
from spiderpig.api.reports import PrecisionReport
from spiderpig.spec import SpecErrors, validate
from spiderpig.task import TASK_FIELDS, Task, task_schema
from spiderpig.task import validate as validate_task

PLACE = {"do": "place", "area_mm": [200, 150], "repeatability_mm": 0.1}

pytestmark = pytest.mark.no_fabricate


def _cards(monkeypatch, cards: dict[str, dict]) -> None:
    """The linkages' cards the screen reads, made up (``describe`` computes the walk model
    over every module: seconds per linkage), and no scale parameters."""
    def describe(key, store=None):
        return cards[key]

    monkeypatch.setattr(tasks, "describe", describe)
    monkeypatch.setattr(tasks, "scale_params_table", lambda store=None: {})


def _walker_card(strides: dict[str, float], lift: float) -> dict:
    return {"modules": {m: {"legs": 1, "stride_mm": s, "walks": s >= 20.0}
                        for m, s in strides.items()},
            "foot_path": {"lift_mm": lift}}


# ---------------------------------------------------------------------------
# the task
# ---------------------------------------------------------------------------


def test_a_task_validates_whole_and_names_the_nearest_field():
    errs = validate_task({"do": "walk", "strid_mm": 40, "area_mm": [1, 2]})
    by = {e.path: e for e in errs}
    assert by["strid_mm"].nearest == "stride_mm"
    assert "a place task's" in by["area_mm"].message
    assert validate_task({"do": "fly"})[0].nearest is None
    assert validate_task({"do": "place"})[0].path == "area_mm"      # an area or points
    assert validate_task({"do": "trace", "motion": "lin"})[0].nearest == "line"
    assert validate_task({"do": "walk", "stride_mm": {"mn": 3}})[0].nearest == "min"
    with pytest.raises(SpecErrors):
        Task.from_dict({"do": "walk", "stride_mm": -1})
    assert set(task_schema()["properties"]) == {"do", *TASK_FIELDS}


def test_a_bare_number_is_the_bound_the_task_means():
    t = Task.from_dict({"do": "walk", "stride_mm": 40, "obstacle_mm": 15})
    needs = t.needs()
    assert (needs["stride_mm"].min, needs["stride_mm"].max) == (40.0, None)
    assert needs["lift_mm"].min == 15.0                      # the obstacle, as a lift
    p = Task.from_dict(PLACE)
    assert p.needs()["repeatability_mm"].max == 0.1
    assert p.area() == (200.0, 150.0)
    pts = Task.from_dict({"do": "place", "points_mm": [[0, 0, 5], [120, 0], [0, 80, 20]]})
    assert pts.area() == (120.0, 80.0)
    assert pts.height() == 20.0
    line = Task.from_dict({"do": "trace", "path_mm": [[0, 0], [50, 0.2], [100, 0]]})
    path = line.path()
    assert path is not None
    assert path["line"]
    assert math.isclose(line.needs()["stroke_mm"].min or 0.0, 100.0)
    assert line.needs()["straightness_mm"].max == 1.0       # tol_mm's default
    assert Task.from_dict(t.to_dict()) == t


def test_an_area_fits_a_band_round_an_axis():
    slack, how = tasks.sector_fit(200, 150, 150, 330, 240)
    assert slack > 0
    assert "far corners" in how
    assert tasks.sector_fit(200, 150, 149.2, 287.8, 240)[0] < 0     # 139 mm deep: too shallow
    assert tasks.sector_fit(100, 50, 100, 200, None)[0] < 0          # no yaw: a plane, no area


# ---------------------------------------------------------------------------
# candidates
# ---------------------------------------------------------------------------


def test_place_finds_the_linkages_with_three_axes_and_says_why_the_rest_cant(monkeypatch):
    _cards(monkeypatch, {})
    rep = api.candidates(PLACE, None)
    assert {c.linkage for c in rep.candidates} == set(linkage.available("arm"))
    why = {x["linkage"]: x["why"] for x in rep.excluded}
    assert "no output" in why["strider"]
    assert "1 axis" in why["hoecken"]
    assert "takes 3" in why["hoecken"]
    assert "2 axes" in why["five_bar"]
    for c in rep.candidates:
        checks = {k.requirement: k for k in c.checks}
        assert checks["repeatability_mm"].ok is None       # nothing fabricated: unmeasured
        assert "precision(design)" in checks["repeatability_mm"].source
        assert checks["area_mm"].tier == "bound"           # the bench is the plan's
        assert checks["area_mm"].ok in (None, False)       # a bound refutes, never confirms
        assert c.spec["precision"]["rp_mm"] == {"max": 0.1}    # what an arm's maker quotes
        assert validate(c.spec) == []
        assert c.numbers["yaw_deg"]
        assert c.numbers["axes"] == 3
    assert [c.rank for c in rep.candidates] == list(range(1, len(rep.candidates) + 1))


def test_a_stored_precision_report_is_read_and_ranks(monkeypatch, tmp_path):
    """A design of the candidate's spec in the store (other targets, the same machine)
    lends its precision budget: the arm's ISO RP becomes the repeatability's value."""
    _cards(monkeypatch, {})
    d = api.resolve({"kind": "arm", "linkage": {"key": "palletizer_reach"},
                     "motion": {"transmission_angle_deg": {"min": 10}}}, tmp_path)
    assert d.store is not None
    d.store.write_report(d, "precision", PrecisionReport(
        metrics={"rp_mm": 0.047, "repeatability_mm": 1.2, "accuracy_mm": 9.1}, complete=True))
    rep = api.candidates(PLACE, tmp_path)
    best = rep.candidates[0]
    assert best.linkage == "palletizer_reach"
    assert best.design == d.id
    rp = next(k for k in best.checks if k.requirement == "repeatability_mm")
    assert (rp.value, rp.ok) == (0.047, True)
    assert d.id in rp.source
    assert "ISO 9283" in rp.detail
    loose = api.candidates({**PLACE, "repeatability_mm": 0.01}, tmp_path)
    assert next(c for c in loose.candidates if c.design == d.id).ok is False


def test_walk_ranks_the_modules_that_walk_and_scales_a_short_stride(monkeypatch):
    cards = {k: _walker_card({"single": 0.0, "quad": 60.0}, 25.0)
             for k in linkage.available("walker")}
    cards["strider"] = _walker_card({"single": 0.0, "double": 170.0, "quad": 175.0}, 33.0)
    _cards(monkeypatch, cards)
    rep = api.candidates({"do": "walk", "stride_mm": 150, "obstacle_mm": 30}, None)
    best = rep.candidates[0]
    assert (best.linkage, best.module) == ("strider", "quad")
    assert all(c.module != "single" for c in rep.candidates)     # a single doesn't walk
    checks = {k.requirement: k for k in best.checks}
    assert checks["stride_mm"].ok
    assert checks["lift_mm"].ok
    assert checks["ground_clearance_mm"].ok is None           # the plan's: unmeasured
    assert best.spec["motion"]["ground_clearance_mm"] == {"min": 30.0}
    assert best.spec["motion"]["lift_mm"] == {"min": 30.0}
    assert best.ok is None
    assert best.why.startswith("meets what is measured")
    short = next(c for c in rep.candidates if c.linkage == "klann")
    assert short.ok is False
    assert short.scale is not None
    assert short.scale["meets"]
    assert math.isclose(short.scale["factor"], 2.5)
    assert {x["linkage"] for x in rep.excluded} == set(linkage.available()) - set(cards)
    assert len(api.candidates({"do": "walk"}, None, limit=2).candidates) == 2


def test_trace_matches_the_output_motion_and_its_metrics(monkeypatch):
    from spiderpig import spec

    cards = {k: {"output_check": {"stroke_mm": 50.0, "straightness_mm": 0.2,
                                  "on_line_fraction": 0.5, "extent_mm": [50, 1]}}
             for k in linkage.available()}
    _cards(monkeypatch, cards)

    def measures(lk):           # (the output checks themselves: a second each)
        return (("stroke_mm", "straightness_mm", "on_line_fraction")
                if lk.output.motion == "line" else ("swing_deg",))

    monkeypatch.setattr(tasks, "output_metrics", measures)
    monkeypatch.setattr(spec, "output_metrics", measures)
    rep = api.candidates({"do": "trace", "motion": "line", "stroke_mm": 40,
                          "straightness_mm": 0.5}, None)
    lines = {k for k in linkage.available("mechanism")
             if getattr(linkage.get(k).output, "motion", "") == "line"}
    assert {c.linkage for c in rep.candidates} == lines
    why = {x["linkage"]: x["why"] for x in rep.excluded}
    assert "not a line" in why["crank_rocker"]
    assert "under control" in why["palletizer"]        # an arm traces by its drives
    assert all(c.ok for c in rep.candidates)
    arm = api.candidates({"do": "trace", "stroke_mm": 40}, None)
    reach = next(c for c in arm.candidates if c.linkage == "palletizer_reach")
    assert next(k for k in reach.checks if k.requirement == "stroke_mm").tier == "bound"
    assert "motion" not in reach.spec          # an arm's stroke is its drives', no target


@pytest.mark.slow
def test_candidates_read_the_real_cards(tmp_path):
    """The screen on the linkages' own cards (computed into the store: seconds each)."""
    walk = api.candidates({"do": "walk", "stride_mm": 150}, tmp_path, limit=3)
    assert walk.candidates[0].ok
    assert walk.candidates[0].numbers["stride_mm"] >= 150
    trace = api.candidates({"do": "trace", "motion": "line", "stroke_mm": 60,
                            "straightness_mm": 0.5}, tmp_path)
    assert trace.candidates[0].linkage == "hoecken"
    assert trace.candidates[0].ok
    assert "peaucellier_crank" in {c.linkage for c in trace.candidates}
