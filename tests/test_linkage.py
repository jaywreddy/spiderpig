"""The generic linkage engine and every registered linkage definition."""

from __future__ import annotations

import itertools
import math

import numpy as np
import pytest

import linkage
from fabricate import BuildConfig, design_side, template_for
from stack import ClearanceError

TS = np.linspace(0.0, 2.0 * math.pi, 720, endpoint=False)
ALL = linkage.available()
WALKERS = linkage.available("walker")


def test_registry_has_klann_first_and_the_others():
    assert ALL[0] == linkage.DEFAULT == "klann"
    assert "strider" in ALL


@pytest.mark.parametrize("key", ALL)
def test_mirror_is_the_reflection_at_the_mirrored_crank_angle(key):
    lk = linkage.get(key)
    right = lk.solve(+1, 0.0).evaluate(math.pi - (TS + 0.4))
    left = lk.solve(-1, 0.4).evaluate(TS)
    for name in lk.points:
        np.testing.assert_allclose(left[name][:, 0], -right[name][:, 0], atol=1e-9)
        np.testing.assert_allclose(left[name][:, 1], right[name][:, 1], atol=1e-9)


@pytest.mark.parametrize("key", ALL)
def test_assembles_over_the_whole_cycle(key):
    pts = linkage.get(key).solve().evaluate(TS)
    broken = sorted(n for n, v in pts.items() if not np.isfinite(v).all())
    assert not broken, f"{key}: {broken} don't assemble at every crank angle"


@pytest.mark.parametrize("key", ALL)
def test_bodies_are_rigid_and_the_crank_turns_about_o(key):
    lk = linkage.get(key)
    pts = lk.solve().evaluate(TS)
    np.testing.assert_allclose(pts["O"], 0.0, atol=1e-12)
    bodies = {**{b: js for b, (js, _) in lk.links.items()}, "conn": lk.crank}
    for body, joints in bodies.items():
        for a, b in itertools.combinations(joints, 2):
            d = np.linalg.norm(pts[a] - pts[b], axis=-1)
            assert np.ptp(d) < 1e-6, f"{key} {body}: |{a}{b}| drifts by {np.ptp(d)}"
    for pin in lk.crank[1:]:
        r = np.linalg.norm(pts[pin], axis=-1)
        assert np.ptp(r) < 1e-9
        ang = np.unwrap(np.arctan2(pts[pin][:, 1], pts[pin][:, 0]))
        assert ang[-1] - ang[0] == pytest.approx(TS[-1] - TS[0], abs=1e-6), "crank turns with t"
    frame = lk.solve().evaluate(TS[:2])
    for j in lk.frame:
        np.testing.assert_allclose(frame[j][0], frame[j][1], atol=1e-12, err_msg=f"{j} moves")


@pytest.mark.parametrize("key", WALKERS)
def test_feet_are_the_lowest_points(key):
    lk = linkage.get(key)
    pts = lk.solve().evaluate(TS)
    feet = {j for _, j in lk.feet}
    low = min(pts[j][:, 1].min() for j in feet)
    others = min(pts[j][:, 1].min() for j in lk.points if j not in feet)
    assert low < others, f"{key}: a non-foot point reaches lower than the feet"


@pytest.mark.parametrize("key", ALL)
def test_leg_template_pins_every_shared_joint(key):
    lk = linkage.get(key)
    tmpl = linkage.build_leg_template(lk.solve())
    sampled = tmpl.sample(TS[::30])
    for (_, a, ja), (_, b, jb) in tmpl.connections:
        pa = sampled.joint_world[a][ja]
        pb = sampled.joint_world[b][jb]
        np.testing.assert_allclose(pa, pb, atol=1e-9)
    on = {}
    for b in tmpl.bodies:
        for j in b.joints:
            on.setdefault(j.name, []).append(b.name)
    ends = {j for _, j in lk.feet} | ({lk.output.point, *lk.output.frame} if lk.output else set())
    lonely = sorted(j for j, bs in on.items() if len(bs) < 2 and j not in ends
                    and j not in {p for _, segs in lk.links.values() for s in segs for p in s})
    assert not lonely, f"{key}: joints {lonely} pin nothing"


@pytest.mark.parametrize("key", ALL)
def test_meta_records_the_linkage(key):
    tmpl = linkage.build_module_template("single", linkage=key)
    assert tmpl.meta["linkage"] == key
    assert tmpl.meta["proportions"] == linkage.get(key).defaults
    assert len(linkage.feet_of(tmpl)) == len(linkage.get(key).feet)


def test_parameters_tune_the_program():
    lk = linkage.get("strider")
    a = lk.solve().evaluate(TS)["J4"]
    b = lk.solve(params={"unit": 2 * float(lk.params["unit"])}).evaluate(TS)["J4"]
    np.testing.assert_allclose(b, 2 * a, atol=1e-9)
    with pytest.raises(KeyError, match="unknown strider parameters"):
        lk.values({"nope": 1.0})


def test_strider_matches_its_plan_drawing():
    """Joint positions read off the diywalkers joint map (drawing units, crank centre (10, 10))."""
    lk = linkage.get("strider")
    unit = float(lk.params["unit"])
    at = lk.solve().joints_at(math.atan2(7.3 - 10, 12.9 - 10))
    plan = {"J3": (0.2, 13.1), "J7": (24.8, 14.7), "J5": (7.8, 4.1), "J9": (18.3, 4.7),
            "J11": (7.3, 4.9), "J10": (18.7, 5.6), "J4": (-1.5, 0.3), "J8": (28.1, 2.1)}
    for j, xy in plan.items():
        got = np.asarray(at[j]) / unit + 10.0
        assert np.hypot(*(got - xy)) < 0.2, f"{j}: {got} vs plan {xy}"


# A link pinned to the crank rider inside the crank circle sweeps across O, which the
# through crankshaft fills in every layer: these need a crank overhung from the servo side.
NEEDS_OVERHUNG_CRANK = {"trotbot", "trotbot_heel", "trotbot_toe", "sixbar", "sixbar_v1"}


@pytest.mark.parametrize("key", WALKERS)     # mechanisms: tests/test_mechanisms.py
def test_one_side_plans_or_the_pipeline_says_why(key):
    """A single-module side lays out with the default constructions, or the static
    clearance stage names the link no layer can hold."""
    cfg = BuildConfig(linkage=key, module="single", robot=False)
    if key in NEEDS_OVERHUNG_CRANK:
        with pytest.raises(ClearanceError, match="sweeps right across the crank at O, so it "
                                                 "can't be in any layer but its riders'"):
            design_side(template_for(cfg), cfg)
        return
    design = design_side(template_for(cfg), cfg)
    assert design.plan.top >= 2
