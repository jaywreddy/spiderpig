"""Tests for :mod:`fabricate`: do the parts physically go together?"""

from __future__ import annotations

import itertools

import pytest

from fabricate import fabricate, plan_for
from klann import (
    build_double_double_decker_template,
    build_klann_template,
    create_klann_geometry,
)
from shapes import FLANGE_R, HOLE_R, PIN_R


def _z(part) -> tuple[float, float]:
    bb = part.bounding_box()
    return bb.min.Z, bb.max.Z


def _valid(part) -> bool:
    return part.is_valid() if callable(part.is_valid) else part.is_valid


def _clashes(mech) -> list[tuple[str, str, float]]:
    parts = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    out = []
    for a, b in itertools.combinations(parts, 2):
        inter = parts[a] & parts[b]  # build123d returns None for an empty result
        vol = 0.0 if inter is None else inter.volume
        if vol > 1e-3:
            out.append((a, b, vol))
    return out


@pytest.fixture(scope="module")
def single():
    tmpl = build_klann_template(create_klann_geometry())
    return tmpl, plan_for(tmpl)


@pytest.mark.parametrize("t", [1.0, 4.38])  # 4.38: where b1 and b2 used to collide
def test_single_leg_parts_do_not_intersect(single, t):
    tmpl, plan = single
    mech = fabricate(tmpl.freeze_at(t), plan).solved()
    assert _clashes(mech) == []


def test_every_part_is_one_valid_solid(single):
    tmpl, plan = single
    mech = fabricate(tmpl.freeze_at(1.0), plan)
    for body in mech.bodies:
        if body.part is not None:
            assert len(body.part.solids()) == 1, f"{body.name} is {len(body.part.solids())} solids"
            assert _valid(body.part), body.name


def test_links_sit_in_their_planned_slots(single):
    tmpl, plan = single
    mech = fabricate(tmpl.freeze_at(1.0), plan)
    for name, slot in plan.slots.items():
        assert _z(mech.body(name).part) == pytest.approx(plan.z(slot))


def test_pins_pass_through_every_link_they_join(single):
    """A pin's shaft spans all its links' slots, head below and cap above."""
    tmpl, plan = single
    mech = fabricate(tmpl.freeze_at(1.0), plan)
    for ax in plan.axes:
        prefix = "frame_" if ax.kind == "frame" else ""
        pin = mech.body(f"{prefix}pin_{ax.name}").part
        pin_bottom, pin_top = _z(pin)
        cap_bottom, _ = _z(mech.body(f"{prefix}cap_{ax.name}").part)
        lo, hi = plan.span(ax)
        lowest_bottom, highest_top = plan.z(lo)[0], plan.z(hi)[1]
        assert pin_bottom == pytest.approx(lowest_bottom - plan.spec.pitch)  # head underneath
        assert pin_top >= highest_top                                     # shaft through all
        assert cap_bottom >= highest_top - 1e-9                           # cap on top
        width = pin.bounding_box().size.X
        assert width == pytest.approx(2 * FLANGE_R)
    for name in plan.slots:
        assert mech.body(name).rigid_with is None


def test_hardware_rides_a_real_body(single):
    tmpl, plan = single
    mech = fabricate(tmpl.freeze_at(1.0), plan)
    names = {b.name for b in mech.bodies}
    hardware = [b for b in mech.bodies if b.rigid_with is not None]
    assert hardware
    for b in hardware:
        assert b.rigid_with in names
        assert not b.joints


def test_crankpin_is_press_fit_in_webs_and_free_in_b1(single):
    tmpl, plan = single
    mech = fabricate(tmpl.freeze_at(1.0), plan)
    crankpin = mech.body("crankpin_M").part
    bottom, top = _z(crankpin)
    b1_bottom, b1_top = plan.z(plan.slots["b1"])
    assert bottom <= b1_bottom
    assert top >= b1_top
    # bore == shaft in the web (press fit), clearance hole in b1 (running fit)
    for host in ("conn", "b1"):
        inter = crankpin & mech.body(host).part
        assert inter is None or inter.volume < 1e-3, host
    assert PIN_R < HOLE_R


def test_joinery_off_leaves_structure_only():
    tmpl = build_double_double_decker_template()
    mech = fabricate(tmpl.freeze_at(0.5), plan_for(tmpl), joinery=False)
    names = [b.name for b in mech.bodies]
    assert not any(n.startswith(("pin_", "cap_", "sleeve_", "frame_")) for n in names)
    assert sum(n.startswith("crankpin_") for n in names) == 4
    frame = mech.body("torso").part
    assert len(frame.solids()) == 1
