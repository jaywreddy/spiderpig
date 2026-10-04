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


@pytest.mark.parametrize("t", quick([0.0, 2.2, 4.38], [2.2]))
@pytest.mark.parametrize("module", quick(MODULES, ["single"]))
def test_every_part_stays_inside_its_claim(design, module, t):
    tmpl, d = design(module)
    assert check_side(d, tmpl.freeze_at(t)) == []


@pytest.mark.parametrize("servo", OTHERS)
@pytest.mark.parametrize("module", quick(MODULES, ["single"]))
def test_every_servo_couples_inside_the_claims(design, module, servo):
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
