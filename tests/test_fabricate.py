"""Tests for :mod:`fabricate`: what a fabricated side and robot carry.

(The contract, valid solids and clashes: ``test_contract.py``.)
"""

from __future__ import annotations

import pytest

from spiderpig.config import BuildConfig
from spiderpig.fabricate import design_side


def test_links_sit_in_their_planned_layers(design, side):
    _, d = design("single")
    mech = side("single", 1.0)
    for name, k in d.plan.layers.items():
        bb = mech.body(name).part.bounding_box()
        z = (bb.min.Z, bb.max.Z)
        z0 = d.plan.z(k)[0]          # on its layer's floor, its own sheet thick (a layer an
        assert z == pytest.approx((z0, z0 + d.ctx.sheet_t("link", name)))   # Al plate thickens)
        assert z[1] <= d.plan.z(k)[1] + 1e-9
        assert mech.body(name).fab == "laser"


def test_hardware_rides_a_real_body(side):
    mech = side("single", 1.0)
    names = {b.name for b in mech.bodies}
    riders = [b for b in mech.bodies if b.rigid_with is not None]
    assert riders
    for b in riders:
        assert b.rigid_with in names
        assert not b.joints


def test_every_body_says_how_it_is_made(side):
    mech = side("quad", 1.0)
    for b in mech.bodies:
        if b.part is not None:
            assert b.fab in ("laser", "printed", "purchased"), b.name
            if b.fab == "purchased" and b.name != "servo_horn":
                assert b.bom_key, b.name       # (the rod pins' cut rods, removed, had none)


def test_robot_is_two_mirrored_sides(robot):
    mech = robot("single", 1.0)
    for name in ("b1", "torso", "servo"):
        left = mech.body(f"L.{name}").part.bounding_box()
        right = mech.body(f"R.{name}").part.bounding_box()
        assert left.max.Z <= 0 <= right.min.Z
        z_left, xy_left = (left.min.Z, left.max.Z), (left.min.X, left.min.Y)
        assert z_left == pytest.approx((-right.max.Z, -right.min.Z))
        assert xy_left == pytest.approx((right.min.X, right.min.Y))


def test_the_robot_shares_the_sides_design(design):
    """One design per side however it's asked for: the frame ties join at build time."""
    tmpl, d = design("single")
    assert design_side(tmpl, BuildConfig(linkage="klann", module="single", robot=True)) is d
    assert not d.config.robot


@pytest.mark.slow           # (two fabrications and every export: ~15 s)
def test_a_static_frame_plate_is_made_once_and_never_changed(tmp_path):
    """The frame plates don't move: a fabrication at another crank angle takes the part an
    earlier one made (``plates._FRAME_MEMO``), one part in both mechanisms, and nothing a
    build, a check or an export does to a mechanism changes it in place."""
    from spiderpig.construction import plates
    from spiderpig.hardware.bom import bom_from_mechanism, group_made
    from spiderpig.layout import save_parts, save_sheets, sheet_lines
    from spiderpig.manufacture import check
    from tests import cache

    cfg = BuildConfig(linkage="hoecken_pantograph", robot=False)
    cache.seed_plan(cfg)
    a = cache.cached_side(cfg, 1.0, fresh=True)
    b = cache.cached_side(cfg, 4.38, fresh=True)
    memo = [part for part, _ in plates._FRAME_MEMO.values()]
    shared = {x.name: x.part for x in a.bodies if any(x.part is p for p in memo)}
    assert len(shared) == 2
    assert "frame_outer" in shared
    for n, part in shared.items():
        assert b.body(n).part is part
    before = {n: (p.wrapped, p.volume, p.bounding_box().min.Z) for n, p in shared.items()}
    a.export_stl(tmp_path / "a.stl")
    a.export_step(tmp_path / "a.step")
    save_sheets(a, tmp_path / "sheet", default=cfg.sheet)
    save_parts(group_made(a.bodies, "laser"), tmp_path / "parts", cfg.sheet)
    sheet_lines(a, cfg.sheet)
    check(a, cfg.sheet)
    bom_from_mechanism(a)
    for n, p in shared.items():
        assert (p.wrapped, p.volume, p.bounding_box().min.Z) == before[n]
        assert b.body(n).part is p
