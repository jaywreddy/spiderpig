"""The generic linkage engine and every registered linkage definition."""

from __future__ import annotations

import itertools
import math

import numpy as np
import pytest
import sympy as sp

from spiderpig import linkage
from spiderpig.config import BuildConfig
from spiderpig.fabricate import design_side, template_for
from spiderpig.stack import ClearanceError

TS = np.linspace(0.0, 2.0 * math.pi, 720, endpoint=False)
ALL = linkage.available()
WALKERS = linkage.available("walker")


def test_registry_has_the_default_first_and_the_others():
    assert ALL[0] == linkage.DEFAULT == "strider"
    assert linkage.get("strider").default_module == "double"
    assert "klann" in ALL                     # the wobbly demo stays registered
    assert not linkage.get("klann").default_module


def test_klann_reference_foot_value():
    """Captured once from the 2016 project's numerical output; drift here is deliberate."""
    sol = linkage.get("klann").solve()
    fx, fy = sol.joints_at(1.0)["F"]
    assert fx == pytest.approx(224.87767188289433, abs=1e-9)
    assert fy == pytest.approx(-93.54456977054052, abs=1e-9)
    ox, oy = linkage.get("klann").solve(phase=0.7).joints_at(0.0)["O"]
    assert (ox, oy) == pytest.approx((0.0, 0.0), abs=1e-12)


def test_circle_x_circle_two_unit_circles_is_exact():
    c1, c2 = sp.Matrix([0, 0]), sp.Matrix([1, 0])
    up = linkage.circle_x_circle(c1, 1, c2, 1, +1)
    down = linkage.circle_x_circle(c1, 1, c2, 1, -1)
    assert list(up) == [sp.Rational(1, 2), sp.sqrt(3) / 2]
    assert list(down) == [sp.Rational(1, 2), -sp.sqrt(3) / 2]


@pytest.mark.parametrize("key", ALL)
def test_program_steps_stay_small(key):
    """Each step is a short expression over earlier points' symbols, not a
    substituted tree (the old chain grew Klann's F to ~34k ops)."""
    for name, expr in linkage.get(key).steps:
        ops = sum(sp.count_ops(c) for c in expr)
        assert ops < 200, f"{key} step {name} has {ops} ops"


@pytest.mark.parametrize("key", ALL)
def test_phase_is_a_time_shift(key):
    lk = linkage.get(key)
    base, shifted = lk.solve(-1, 0.0), lk.solve(-1, 0.9)
    ts = np.linspace(0.0, 2.0 * math.pi, 50)
    for name in lk.points:
        np.testing.assert_allclose(shifted.evaluate(ts)[name], base.evaluate(ts + 0.9)[name],
                                   atol=1e-12)


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


# TrotBot's heel link (the heel and toe variants) passes one plan unit from the crankpin:
# the family's 10.5 mm unit clears the bolt crank's 6 mm shank (the default) and the printed
# crank's 6 mm post, but the keyed crank's 8.5 mm post needs a 12 mm unit
KEYED_UNIT = {"trotbot_heel": 12.0, "trotbot_toe": 12.0}


@pytest.mark.parametrize("key", WALKERS)     # mechanisms: tests/test_mechanisms.py
def test_one_side_plans(key):
    """A single-module side lays out with the default constructions (TrotBot's heel link
    at its family's 10.5 mm unit: the bolt crank's shank clears it; with the keyed crank's
    post it stops at the static stage and is sent to 12)."""
    cfg = BuildConfig(linkage=key, module="single", robot=False)
    if key in KEYED_UNIT:
        keyed = BuildConfig(linkage=key, module="single", robot=False, crank="keyed",
                            pillar="printed")
        with pytest.raises(ClearanceError, match="what would clear it:\n  unit 10.5 -> 12"):
            design_side(template_for(keyed), keyed)
    design = design_side(template_for(cfg), cfg)
    assert design.plan.top >= 2
