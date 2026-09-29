"""Tests for :mod:`fabricate` and the construction contract: do the parts go together?"""

from __future__ import annotations

import itertools

import pytest

from construction.contract import check_side
from fabricate import BuildConfig, design_side, fabricate, fabricate_side
from klann import (
    build_double_double_decker_template,
    build_klann_template,
    create_klann_geometry,
)

SIDE = BuildConfig(robot=False)


def _clashes(mech) -> list[tuple[str, str, float]]:
    parts = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    out = []
    for a, b in itertools.combinations(parts, 2):
        inter = parts[a] & parts[b]
        vol = 0.0 if inter is None else sum(s.volume for s in inter.solids())
        if vol > 1e-3:
            out.append((a, b, round(vol, 4)))
    return out


@pytest.fixture(scope="module")
def single():
    tmpl = build_klann_template(create_klann_geometry())
    return tmpl, design_side(tmpl, SIDE)


@pytest.fixture(scope="module")
def quad():
    tmpl = build_double_double_decker_template()
    return tmpl, design_side(tmpl, SIDE)


@pytest.mark.parametrize("t", [0.0, 2.2, 4.38])
def test_every_part_stays_inside_its_claim(single, quad, t):
    for tmpl, design in (single, quad):
        assert check_side(design, tmpl.freeze_at(t)) == []


@pytest.mark.parametrize("t", [1.0, 4.38])  # 4.38: where b1 and b2 used to collide
def test_single_side_parts_do_not_intersect(single, t):
    tmpl, design = single
    assert _clashes(fabricate_side(design, tmpl.freeze_at(t))) == []


def test_every_part_is_one_valid_solid(single):
    tmpl, design = single
    mech = fabricate_side(design, tmpl.freeze_at(1.0))
    for body in mech.bodies:
        if body.part is not None:
            assert len(body.part.solids()) == 1, body.name
            assert body.part.is_valid, body.name


def test_links_sit_in_their_planned_layers(single):
    tmpl, design = single
    mech = fabricate_side(design, tmpl.freeze_at(1.0))
    for name, k in design.plan.layers.items():
        bb = mech.body(name).part.bounding_box()
        z = (bb.min.Z, bb.max.Z)
        assert z == pytest.approx(design.plan.z(k))
        assert mech.body(name).fab == "laser"


def test_hardware_rides_a_real_body(single):
    tmpl, design = single
    mech = fabricate_side(design, tmpl.freeze_at(1.0))
    names = {b.name for b in mech.bodies}
    riders = [b for b in mech.bodies if b.rigid_with is not None]
    assert riders
    for b in riders:
        assert b.rigid_with in names
        assert not b.joints


def test_every_body_says_how_it_is_made(quad):
    tmpl, design = quad
    mech = fabricate_side(design, tmpl.freeze_at(1.0))
    for b in mech.bodies:
        if b.part is not None:
            assert b.fab in ("laser", "printed", "purchased"), b.name
            if b.fab == "purchased" and b.name != "servo_horn":
                assert b.bom_key, b.name


def test_robot_is_two_mirrored_sides(single):
    tmpl, _ = single
    robot = fabricate(tmpl, BuildConfig(), 1.0)
    for name in ("b1", "torso", "servo"):
        left = robot.body(f"L.{name}").part.bounding_box()
        right = robot.body(f"R.{name}").part.bounding_box()
        assert left.max.Z <= 0 <= right.min.Z
        z_left, xy_left = (left.min.Z, left.max.Z), (left.min.X, left.min.Y)
        assert z_left == pytest.approx((-right.max.Z, -right.min.Z))
        assert xy_left == pytest.approx((right.min.X, right.min.Y))
