"""What would clear a stage's failure (:mod:`recommend`): checked before it is said."""

from __future__ import annotations

import pytest

from spiderpig import explain
from spiderpig.config import BuildConfig
from spiderpig.construction.base import Params
from spiderpig.fabricate import design_side, side_problem, static_stage, template_for
from spiderpig.recommend import Gap, scale
from spiderpig.stack import ClearanceError


def _heel(unit: float, crank: str = "printed", **params) -> BuildConfig:
    """The heel with the printed crank, whose 3 mm post radius these numbers are (the keyed
    crank's 8.5 mm post asks more of b7, and sizes the post itself: test_the_keyed_crank_
    post_sends_the_heel_up_a_scale)."""
    return BuildConfig(linkage="trotbot_heel", module="single", robot=False, crank=crank,
                       pillar="printed", proportions=(("unit", unit),), params=Params(**params))


def _fail(cfg) -> ClearanceError:
    with pytest.raises(ClearanceError) as e:
        design_side(template_for(cfg), cfg)
    return e.value


def test_the_heel_is_told_the_scale_that_clears_it():
    """At the drawing's 7 mm unit the heel link passes the crankpin's post 6.8 mm off, under
    the 10 mm it needs: x1.47 clears it, rounded up to a 10.5 mm unit, and checked."""
    e = _fail(_heel(7.0))
    (rec,) = e.recommendations
    assert rec.changes == (("unit", 7.0, 10.5),)
    assert rec.why.startswith("scale trotbot_heel x1.50 (the least that clears it is x1.47; "
                              "b7 past crankpin J1 is 6.8 mm, needs 10.0)")
    assert rec.effects.startswith("crank 28.0 -> 42.0 mm; about 1.5x the crank torque")
    assert rec.verified.startswith("checked: the static stage passes, and it plans in 12 ")
    assert "what would clear it:\n  unit 7 -> 10.5: scale trotbot_heel x1.50" in str(e)
    # no thinner parts do: the crankpin takes an M3 screw, and b1 a wall round it
    assert any(n.startswith("no part sizes at this scale clear it within the constructions' "
                            "limits (the least: ") for n in e.notes)


def test_thinner_parts_are_checked_against_the_constructions():
    """At a 9 mm unit b7 passes 8.7 mm off: thinner links and crankpin clear it, and a
    thinner link gets a thinner axle (min_wall round its hole)."""
    e = _fail(_heel(9.0))
    thin = next(r for r in e.recommendations if r.why == "thinner parts at this scale")
    assert thin.changes == (("link_radius", 6.0, 4.5), ("crankpin_d", 6.0, 5.5),
                            ("axle_d", 6.0, 5.5))
    assert thin.verified.startswith("checked: the static stage passes")
    assert e.recommendations[0].changes == (("unit", 9.0, 10.5),)


@pytest.mark.parametrize(("unit", "params", "dist", "need"), [
    (8.0, {"link_radius": 4.5, "axle_d": 4.0, "crankpin_d": 5.0}, 7.8, 8.0),
    (10.0, {}, 9.7, 10.0),
])
def test_the_heel_just_misses_below_its_scale(unit, params, dist, need):
    cfg = _heel(unit, **params)
    tmpl = template_for(cfg)
    _, _, problem = side_problem(tmpl, cfg)
    with pytest.raises(ClearanceError, match=f"it passes crankpin J1 at {dist} mm, under the "
                                             f"{need} mm a post there needs"):
        static_stage(tmpl, problem)


def test_the_keyed_crank_post_sends_the_heel_up_a_scale():
    """With the keyed crank (the default) the heel at its own 10.5 mm unit passes J1 at 10.2
    mm of the 11.2 a 4.25 mm post needs; thinner parts can't help (the keyed crank sizes
    its post from the key), so the one recommendation is the scale, checked."""
    e = _fail(_heel(10.5, crank="keyed"))
    assert "it passes crankpin J1 at 10.2 mm, under the 11.2 mm a post there needs" in str(e)
    (rec,) = e.recommendations
    assert rec.changes == (("unit", 10.5, 12.0),)
    assert rec.verified.startswith("checked: the static stage passes, and it plans in 14 layers")


def test_gap_arithmetic():
    gap = Gap("b7 past J1", 6.8, (("crankpin_d", 0.5), ("link_radius", 1.0)), 1.0)
    assert gap.need(Params()) == pytest.approx(10.0)
    assert gap.need(Params(link_radius=4.5, crankpin_d=5.0)) == pytest.approx(8.0)
    # a link across the part itself: no scale clears it
    assert scale(_heel(7.0), [Gap("x", -1.0, (), 1.0)]) is None


def test_explain_prints_what_would_clear_it():
    out = explain.explain("trotbot_heel", params={"unit": 7.0})
    assert "STOP: trotbot_heel: b7 sweeps right across the crank at O" in out
    assert "what would clear it:" in out
    assert "unit 7 -> 10.5" in out    # the bolt crank's 6 mm shank (default; keyed: -> 12)
