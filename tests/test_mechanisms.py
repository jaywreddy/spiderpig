"""Building-block mechanisms (``linkages/mechanisms.py``): each output measures what the
research measured, the template stage enforces what it promises, and each one-input
mechanism plans (the planner's tier) and builds inside its claims (the constructions'),
or the pipeline says why."""

from __future__ import annotations

import math
from dataclasses import replace

import pytest

from spiderpig import linkage
from spiderpig.config import BuildConfig, ParamError
from spiderpig.construction import ConstructionError
from spiderpig.construction.contract import check_side
from spiderpig.fabricate import design_side, template_for
from tests.tiers import quick

MECHANISMS = linkage.available("mechanism")

# The research's numbers (verify.py) for the selected parameters, in mm and degrees.
VERIFY = {
    "hoecken": {"stroke_mm": 66.89, "straightness_mm": 0.0644, "on_line": 0.5125},
    "watt_crank": {"stroke_mm": 32.04, "straightness_mm": 0.0167, "on_line": 1.0},
    "peaucellier_crank": {"stroke_mm": 45.9, "straightness_mm": 0.0, "on_line": 1.0},
    "parallelogram_lift": {"extent_mm": (1.616, 32.0), "rotation_deg": 0.0},
    "watt_table_lift": {"extent_mm": (0.04483, 32.04), "stroke_mm": 32.04,
                        "straightness_mm": 0.0167, "rotation_deg": 0.0},
    "five_bar": {"extent_mm": (64.0, 53.4)},
    "hoecken_pantograph": {"stroke_mm": 100.34, "straightness_mm": 0.0966, "on_line": 0.2403},
    "crank_rocker": {"swing_deg": 60.0},
    "rocker_amplifier": {"swing_deg": 153.263},
    "dwell_rocker": {"swing_deg": 43.876, "dwell_deg": 125.0},
    "hoecken_table": {"stroke_mm": 66.89, "straightness_mm": 0.0644, "on_line": 0.5125,
                      "rotation_deg": 0.0},
}

# Mechanisms the pipeline stops, the stage error and what it says.
EXPECTED = {
    "five_bar": (ConstructionError, "five_bar: the drive turns one input, the crank at O "
                                    "(one servo); five_bar has 2 (t, t2), and a drive for t2 "
                                    "isn't built"),
}


def test_walkers_have_feet_mechanisms_an_output():
    assert set(VERIFY) == set(MECHANISMS)
    assert sorted(linkage.available()) == sorted(linkage.available("walker") + MECHANISMS)
    for key in linkage.available():
        lk = linkage.get(key)
        assert (lk.kind == "walker") == bool(lk.feet) == (lk.output is None)
    assert linkage.get("hoecken").leg_modules == {"single": ((1, 0.0),)}


@pytest.mark.parametrize("key", MECHANISMS)
def test_output_measures_what_the_research_measured(key):
    got = linkage.get(key).assert_output()
    assert got.broken is None
    for name, want in VERIFY[key].items():
        assert getattr(got, name) == pytest.approx(want, rel=5e-3, abs=5e-3 if name == "on_line"
                                                   else 1e-4), name
    assert got.describe().startswith(f"{key}: ")


def test_a_broken_promise_stops_the_template_stage():
    with pytest.raises(linkage.OutputError, match=r"^watt_crank: output P strays across a "
                       r"0\.514 mm band about its line over crank 0°\.\.360°, wider than the "
                       r"0\.05 mm it promises$"):
        linkage.build_module_template("single", proportions={"coupler": 2}, linkage="watt_crank")
    with pytest.raises(linkage.OutputError, match=r"^dwell_rocker: output b4 stands still "
                       r"\(±0\.25°\) for only 82\.0° of the turn, less than the 120° it promises$"):
        linkage.build_module_template("single", proportions={"dyad": 2.2}, linkage="dwell_rocker")


def test_a_platform_that_turns_is_refused(monkeypatch):
    lift = linkage.get("parallelogram_lift")

    def bent(p):        # the upper arm 2 mm longer: no longer a parallelogram
        *steps, _ = lift.program(p)
        return [*steps, ("T2", linkage.circle_x_circle(
            linkage.P("T1"), p["spacing"] * p["unit"], linkage.P("G2"), p["arm"] * p["unit"] + 2,
            -1))]

    monkeypatch.setitem(linkage.REGISTRY, "bent_lift", replace(lift, key="bent_lift",
                                                               program=bent))
    with pytest.raises(linkage.OutputError, match=r"^bent_lift: platform b4 turns [\d.]+° over "
                       r"the cycle \(most at crank \d+°\): a translating platform must not "
                       r"rotate$"):
        linkage.build_module_template("single", linkage="bent_lift")


def test_two_inputs_are_checked_over_their_torus():
    lk = linkage.get("five_bar")
    p = {s.point: s for s in lk.check()}["P"]
    assert p.margin_mm == pytest.approx(20.0)                      # verify.py: 1.000 unit
    assert (p.worst_deg, p.worst_t2_deg) == (180.0, 0.0)          # cranks pointing apart
    assert p.angle_deg == pytest.approx((44.05, 122.09), abs=0.1)
    with pytest.raises(linkage.AssemblyError, match=r"joint P can't be placed .* miss each other "
                       r"by up to 20\.00 mm \(worst at 180°, t2 0°\)"):
        lk.assert_assembles({"distal": 3})
    pts = lk.solve().evaluate([0.0, math.pi], [math.pi, 0.0])
    assert pts["M2"][:, 0] == pytest.approx([80.0, 120.0])        # t2 turns the second crank


@pytest.mark.planner
@pytest.mark.parametrize("key", MECHANISMS)
@pytest.mark.usefixtures("fresh_plan_memo")
def test_one_side_plans_or_the_pipeline_says_why(key):
    """One side lays out, or a stage says why (its parts: the test below)."""
    cfg = BuildConfig(linkage=key, module="single", robot=False)
    tmpl = template_for(cfg)
    if key in EXPECTED:
        err, msg = EXPECTED[key]
        with pytest.raises(err) as e:
            design_side(tmpl, cfg)
        assert str(e.value).startswith(msg)
        # round 5: and what to do about it (a limit of v1; the one-input mechanisms)
        assert "a limit of v1 (one servo per machine), not of this spec" in str(e.value)
        assert "a one-input mechanism builds (hoecken, " in str(e.value)
        return
    design = design_side(tmpl, cfg)
    assert design.plan.top >= 2
    if key == "peaucellier_crank":      # the long arms sweep over Y: its pillar holds one side
        assert ("b3 passes pillar:Y at 0.1 mm, under the 11.7 mm its thinnest part needs, so it "
                "can't be in any layer pillar:Y spans") in [c.describe() for c in design.clearances]


@pytest.mark.construction
@pytest.mark.parametrize(("key", "t"), quick(
    [(k, t) for k in MECHANISMS if k not in EXPECTED for t in (0.0, 2.2)], [("hoecken", 2.2)]))
def test_one_side_stays_inside_its_claims(key, t):
    """Every part of each one-input mechanism's side stays inside its claims (at two crank
    angles, each its own case: ~4.5 s each on the Hoecken; the plan is the test above's,
    cached by the engine)."""
    cfg = BuildConfig(linkage=key, module="single", robot=False)
    tmpl = template_for(cfg)
    design = design_side(tmpl, cfg)
    assert check_side(design, tmpl.freeze_at(t)) == []


def test_walking_takes_walkers_only():
    with pytest.raises(ParamError, match="hoecken is a mechanism, not a walker"):
        BuildConfig(linkage="hoecken", module="single", robot=True)
