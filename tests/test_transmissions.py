"""An arm's drives (:mod:`construction.transmissions`, :mod:`spiderpig.actuators`,
2026-10-11): a stepper and a GT2 belt or a cord capstan per input, the roles that pick
them per drive, where the motors go, the hub's layers the axle nests in, what they build
and what they cost the output's precision."""

from __future__ import annotations

import math

import numpy as np
import pytest

from spiderpig import actuators, construction
from spiderpig import roles as rolemod
from spiderpig.config import BuildConfig
from spiderpig.construction.transmissions import TRANSMISSIONS, DriveGroup, DriveHub
from spiderpig.construction.transmissions.base import motor_ends, place_motor
from spiderpig.construction.transmissions.belt import BeltDrive
from spiderpig.construction.transmissions.capstan import CapstanDrive
from tests import cache

ARM = {"linkage": "palletizer", "robot": False}


def _arm(**kw) -> BuildConfig:
    return BuildConfig(**ARM, **kw)


@pytest.fixture(scope="module")
def belt():
    tmpl, design = cache.cached_design(_arm())
    return tmpl, design


# -- the parts' numbers ---------------------------------------------------------------


@pytest.mark.no_fabricate
def test_a_stepper_seen_through_a_reduction():
    m = actuators.get("nema17_17hs4401")
    assert m.holding_stiffness_nm_per_rad == pytest.approx(0.40 * 50)
    t = m.error_terms(5.0)
    # 1/16 of a 1.8 deg step (0.1125) beats the +-5 % a microstep's place is (0.09): the
    # coarser of the two, through 5:1
    assert t.resolution_deg == pytest.approx(1.8 / 16 / 5)
    assert t.stiffness_nm_per_rad == pytest.approx(20.0 * 25)
    assert actuators.DEFAULT in actuators.available()
    with pytest.raises(KeyError, match="unknown stepper"):
        actuators.get("nema99")


@pytest.mark.no_fabricate
def test_a_gt2_loop_sits_at_its_centre_distance():
    b = BeltDrive()
    assert b.ratio(None) == 5.0
    big, small = b.big["pitch_d"], b.small["pitch_d"]
    assert big == pytest.approx(80 * 2 / math.pi, abs=1e-3)
    for length in (200, 280):
        c = b.centre_distance(length)
        back = 2 * c + math.pi * (big + small) / 2 + (big - small) ** 2 / (4 * c)
        assert back == pytest.approx(length)
    assert b.centre_distance(100) is None                   # too short to go round


@pytest.mark.no_fabricate
def test_a_capstans_ratio_is_its_radii():
    c = CapstanDrive()
    assert c.ratio(None) == pytest.approx((30 + 0.5) / (2.5 + 0.5))   # the bare 5 mm shaft
    assert c.distances(None, None) == [(30 + 2.5 + 1.0, None)]


@pytest.mark.no_fabricate
def test_a_motor_keeps_clear_of_the_heads_and_the_other_motor():
    m = actuators.get(actuators.DEFAULT)
    o = np.zeros(2)
    head = (np.array([20.0, -30.0]), 5.0)                    # in the first angle's way
    xy, th, _, _ = place_motor(o, [(50.0, None)], m, [head], [])
    assert math.degrees(th) != -30.0
    ends = motor_ends(xy, th, m)
    assert min(np.linalg.norm(head[0] - e) for e in ends) > 0
    with pytest.raises(construction.ConstructionError, match="no place round the pivot"):
        place_motor(o, [(10.0, None)], m, [(o, 5.0)], [])   # on the pillar head itself


@pytest.mark.no_fabricate
def test_the_roles_pick_each_drives_parts():
    assert rolemod.get("transmission").per == "drive"
    assert rolemod.choices("transmission") == ["belt", "capstan"]
    assert set(TRANSMISSIONS) == {"belt", "capstan"}
    c = _arm(roles=(("transmission@q3", "capstan"),))
    assert (c.role("transmission", "q2"), c.role("transmission", "q3")) == ("belt", "capstan")
    assert c.role("actuator", "q2") == actuators.DEFAULT
    assert BuildConfig().applied_roles() == {}               # a walker has no drives


# -- a planned side -------------------------------------------------------------------


@pytest.mark.no_fabricate
def test_the_drives_alternate_sides_and_the_axle_nests(belt):
    _, design = belt
    hubs = [design.ctx.interfaces[f"drive:{k}"] for k in ("q2", "q3")]
    assert all(isinstance(h, DriveHub) for h in hubs)
    assert [h.side for h in hubs] == [-1, 1]                 # outer plate, inner plate
    assert [h.link for h in hubs] == ["b1", "b2"]
    assert all(h.axis == "O" for h in hubs)
    plan = design.plan
    nested = {s.layer for s in plan.shapes("pillar:O") if s.label.endswith("nested")}
    want = set(hubs[0].layers(plan.layers["b1"])) | set(hubs[1].layers(plan.layers["b2"]))
    assert nested == want
    groups = [g for g in design.groups if isinstance(g, DriveGroup)]
    assert [g.transmission.key for g in groups] == ["belt", "belt"]


@pytest.mark.no_fabricate
def test_the_drives_error_terms_are_the_belts_and_the_motors(belt):
    _, design = belt
    g = next(g for g in design.groups if isinstance(g, DriveGroup))
    t = g.error_terms(design.ctx)
    assert t.backlash_deg > 0                                # the teeth's clearance
    assert t.resolution_deg == pytest.approx(1.8 / 16 / 5)
    assert t.stiffness_nm_per_rad > 0
    assert t.stiffness_nm_per_rad < 0.40 * 50 * 25           # the belt in series: softer
    assert "belt" in t.source
    assert "nema17" in t.source


def test_the_belt_arm_builds_inside_its_claims_and_buys_its_drives(belt):
    from spiderpig.construction.contract import check_side
    from spiderpig.hardware.bom import bom_from_mechanism

    tmpl, design = belt
    drives = [g.name for g in design.groups if isinstance(g, DriveGroup)]
    assert check_side(design, tmpl.freeze_at(2.0), groups=drives) == []
    fab = cache.cached_side(_arm(), 1.0)
    rows = {r.key: r.qty for r in bom_from_mechanism(fab).purchased}
    assert rows["stepper_17hs4401"] == 2
    assert rows["pulley_gt2_16t_5mm"] == rows["pulley_gt2_80t_8mm"] == 2
    assert any(k.startswith("belt_gt2_6mm_") for k in rows)
    notes = fab.meta["drive"]
    assert {n["side"] for n in notes.values()} == {"outer", "inner"}


def test_the_guide_mounts_each_motor_and_its_drive(belt):
    from spiderpig.construction.assembly import assembly_steps

    _, design = belt
    fab = cache.cached_side(_arm(), 1.0)
    steps = assembly_steps(fab, design)
    titles = [s.title for s in steps]
    assert any("drive for q2" in t for t in titles)
    assert any("Motor for q3" in t for t in titles)          # the inner plate's unit
    text = " ".join(" ".join(s.text) for s in steps)
    assert "Cut its shaft to" in text
    assert "over its link" in text
    assert "under its link" in text


@pytest.mark.slow
def test_the_capstan_arm_plans_and_holds_its_contract():
    from spiderpig.construction.contract import check_side

    cfg = _arm(roles=(("transmission", "capstan"),))
    tmpl, design = cache.cached_design(cfg)
    for t in (0.0, 4.4):
        assert check_side(design, tmpl.freeze_at(t)) == []
    g = next(g for g in design.groups if isinstance(g, DriveGroup))
    assert g.error_terms(design.ctx).backlash_deg == 0.0     # the cord never slips


@pytest.mark.slow
def test_the_drives_join_the_precision_budget():
    from spiderpig.precision import precision

    belt_r = precision(_arm(), n=9)
    cap_r = precision(_arm(roles=(("transmission", "capstan"),)), n=9)
    assert set(belt_r.inputs) == {"q2", "q3"}
    assert belt_r.inputs["q2"].backlash_mm > cap_r.inputs["q2"].backlash_mm == 0.0
    assert cap_r.inputs["q2"].resolution_mm < belt_r.inputs["q2"].resolution_mm
    assert belt_r.repeatability_mm == pytest.approx(
        belt_r.plays.worst_mm + sum(b.backlash_mm for b in belt_r.inputs.values()))
    assert not any("no transmission" in n for n in belt_r.notes)
