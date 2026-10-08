"""The construction contract for every module and servo: every part a group builds stays
inside its claims (:func:`construction.contract.check_side`), is one valid solid, and no
two parts of a side ever intersect (the planner's guarantee, checked on the real solids)."""

from __future__ import annotations

import pytest

from spiderpig import servos
from spiderpig.construction.contract import bad_solids, check_side, clashes
from tests.tiers import quick

MODULES = ("single", "double", "decker", "quad")
OTHERS = [s for s in servos.available() if s != servos.DEFAULT]


@pytest.mark.parametrize("t", quick([0.0, 2.2, 4.38], []))
@pytest.mark.parametrize("module", MODULES)
def test_every_part_stays_inside_its_claim(design, module, t):
    """Every group of every module at three angles: the full tier only (the single at 2.2,
    the cheapest case, is ~8 s). The quick tier's whole-side contract check is the Hoecken's
    (test_mechanisms.py::test_one_side_stays_inside_its_claims[hoecken-2.2], ~4.4 s: its
    drive, crank, pillars, pins, links and frame plates through the same check_side)."""
    tmpl, d = design(module)
    assert check_side(d, tmpl.freeze_at(t)) == []


# the full tier only: the quick tier couples each servo's drive and crank
# (test_crank.py::test_every_servo_couples_the_crank_inside_the_claims, 3 s each), and
# checks every group on the default servo (above); the whole side per servo is ~8 s
@pytest.mark.parametrize("servo", quick(OTHERS, []))
@pytest.mark.parametrize("module", MODULES)
def test_every_servo_couples_inside_the_claims(design, module, servo, monkeypatch):
    if (module, servo) == ("quad", "xl330_m288"):
        # the demo Klann quad on the XL330 with the hex-standoff crank (2026-10-04): its hub
        # chain must end flush in the hub plate, and the 2-3 mm steps of the hex standoff's
        # stock lengths reject most layerings at their z, so the planner finds its 15-layer
        # plan only after ~6 CPU minutes, past the 60 s default (the STS3215 plans in 3 s)
        from spiderpig import stack

        monkeypatch.setattr(stack.plan, "MAX_SECONDS", 900.0)
    tmpl, d = design(module, servo)
    assert check_side(d, tmpl.freeze_at(2.2)) == []


# 4.38: where b1 and b2 used to collide
@pytest.mark.parametrize("t", quick([0.0, 1.0, 2.2, 4.38], [1.0, 4.38]))
@pytest.mark.parametrize("servo", quick(servos.available(), [servos.DEFAULT]))
def test_single_side_parts_do_not_intersect(side, servo, t):
    """Every body, screws and nuts included."""
    mech = side("single", t, servo)
    names = {b.name for b in mech.bodies if b.part is not None}
    for prefix in ("pin_", "pillar_", "servo_screw", "crank_pin"):
        assert any(n.startswith(prefix) for n in names), prefix
    assert clashes(mech) == []


@pytest.mark.parametrize("module", quick(["single", "quad"], ["single"]))
def test_every_part_is_one_valid_solid(side, module):
    assert bad_solids(side(module, 1.0)) == []
