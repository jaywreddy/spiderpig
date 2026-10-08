"""The construction contract for every module and servo: every part a group builds stays
inside its claims (:func:`construction.contract.check_side`), is one valid solid, and no
two parts of a side ever intersect (the planner's guarantee, checked on the real solids)."""

from __future__ import annotations

import copy
import logging
import math
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from build123d import Box, Location, Sphere

from spiderpig import servos
from spiderpig.construction import contract
from spiderpig.construction.base import Build, Realized, hardware
from spiderpig.construction.contract import bad_solids, check_side, check_sides, clashes
from spiderpig.shapes import box, disc, union
from spiderpig.stack import Disc, Placed
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


# -- check_sides: the side realized once for every angle ----------------------------------

ANGLES = (0.0, 2.2, 4.38)


@pytest.mark.parametrize(("linkage", "module"), quick(
    [("klann", m) for m in MODULES] + [("strider", "double"), ("klann_lego", "quad")], []))
def test_check_sides_is_check_side_at_every_angle(design, linkage, module):
    """:func:`check_sides` gives :func:`check_side`'s verdict at every angle (the full tier:
    each case is the exact check at three angles, ~20-60 s; the Strider quad's crank has a
    hex journal, so its crank is realized again at each angle)."""
    tmpl, d = design(module, linkage=linkage)
    assert check_sides(d, tmpl, ANGLES) == [check_side(d, tmpl.freeze_at(t)) for t in ANGLES]


def _moved(part, move):
    r, t = move
    deg = math.degrees(math.atan2(r[1, 0], r[0, 0]))
    return part.moved(Location((float(t[0]), float(t[1]), 0.0), (0.0, 0.0, deg)))


def _volume(part) -> float:
    return 0.0 if part is None else sum(s.volume for s in part.solids())


@pytest.mark.parametrize(("linkage", "module"), quick(
    [("klann", m) for m in MODULES] + [("strider", "double"), ("klann_lego", "quad"),
                                       ("hoecken", "single")], []))
def test_each_group_moves_as_it_says(design, linkage, module):
    """What :func:`check_sides` takes on trust, shown on the real parts: every group that
    declares a :meth:`~spiderpig.construction.base.Group.motion` builds, at another crank
    angle, exactly its parts at the first moved by it (the same bodies, volume, box, and
    nothing of either outside the other)."""
    tmpl, d = design(module, linkage=linkage)
    a, b = (Build(d.ctx, d.plan, tmpl.freeze_at(t)) for t in (0.0, 2.2))
    done_a, done_b = Realized(), Realized()
    moving = 0
    for g in d.groups:
        got_a, got_b = g.realize(a, done_a), g.realize(b, done_b)
        done_a.merge(got_a)
        done_b.merge(got_b)
        motion = g.motion(got_a)
        if motion is None:
            continue
        assert [x.name for x in got_a.bodies] == [x.name for x in got_b.bodies], g.name
        for pa, pb in zip(got_a.bodies, got_b.bodies, strict=True):
            move = contract._motion_of(motion, pa, a, b)
            assert move is not None, (g.name, pa.name)
            want, got = _moved(pa.part, move), pb.part
            assert got.volume == pytest.approx(want.volume, rel=1e-9, abs=1e-9), pa.name
            wb, gb = want.bounding_box(), got.bounding_box()
            box = [wb.min.X, wb.min.Y, wb.min.Z, wb.max.X, wb.max.Y, wb.max.Z]
            assert pytest.approx(box, abs=1e-5) == [gb.min.X, gb.min.Y, gb.min.Z,
                                                    gb.max.X, gb.max.Y, gb.max.Z], pa.name
            assert _volume(got - want) + _volume(want - got) < 1e-4, pa.name
            moving += 1
    assert moving > 0


def _poking(d, mm3: float, *, at=lambda build: True, motion=True):
    """A copy of the side ``d`` (the factory's own is never touched) whose first pin also
    builds a cube of ``mm3`` outside its claims at the crank angles ``at`` picks, its
    ``motion`` declared as the pin's (else ``None``): ``(that side, the pin's name)``."""
    pin = next(g for g in d.groups if g.name.startswith("pin:"))
    g = copy.copy(pin)

    def poke(build, done):
        got = pin.realize(build, done)
        if at(build):
            x, y = build.xy(pin.axis.name)
            col = [s for s in build.shapes(pin.name) if not s.gap]
            z0, z1 = build.plan.slot_z(col[0])
            c = mm3 ** (1 / 3)
            part = Box(c, c, c).moved(Location((x + 3 * max(s.shape.r for s in col), y,
                                                (z0 + z1) / 2)))
            got.bodies.append(hardware(f"{pin.name}_poke", part, pin.axis.members[0],
                                       fab="printed"))
        return got

    g.realize = poke
    if not motion:
        g.motion = lambda got: None
    return replace(d, groups=[g if x is pin else x for x in d.groups]), pin.name


def test_check_sides_rechecks_a_group_that_changes_with_the_angle(design):
    """A group that doesn't say how it moves (``motion`` ``None``) is realized and checked
    again at every angle: a part outside its claims at 2.2 only is found there."""
    tmpl, d = design("single", linkage="hoecken")
    pin = d.plan.topo.axes_of("crankpin")[0].name
    at0 = tuple(Build(d.ctx, d.plan, tmpl.freeze_at(0.0)).xy(pin))
    d, name = _poking(d, 0.5, motion=False, at=lambda build: tuple(build.xy(pin)) != at0)
    ts = (0.0, 2.2)
    got = check_sides(d, tmpl, ts, groups=[name])
    assert got == [check_side(d, tmpl.freeze_at(t), groups=[name]) for t in ts]
    assert got[0] == []
    assert len(got[1]) == 1
    assert f"{name}_poke" in got[1][0]


@pytest.mark.parametrize(("mm3", "again"), [(0.0006, True), (0.0002, False), (0.5, True)])
def test_check_sides_carries_only_a_clear_verdict(design, caplog, mm3, again):
    """A part that moves as its group says but is near the contract's allowance (more than
    half of it outside the claims that move with it) or past it is checked again at every
    angle; one clear of it is not."""
    tmpl, d = design("single", linkage="hoecken")
    d, name = _poking(d, mm3)
    ts = (0.0, 2.2)
    with caplog.at_level(logging.DEBUG, logger=contract.__name__):
        got = check_sides(d, tmpl, ts, groups=[name])
    assert got == [check_side(d, tmpl.freeze_at(t), groups=[name]) for t in ts]
    assert (name in caplog.text) == again
    assert all(len(p) == (mm3 > contract.MAX_OUTSIDE) for p in got)


# -- the boolean-free inside test (contract._in_a_column) ------------------------------------

def _column(radii):
    """A fake build with discs at ``A`` = (10, 5) in layers 0.. of 3 mm, one radius each."""
    shapes = [Placed(k, Disc("A", r), "pin:A") for k, r in enumerate(radii)]
    build = SimpleNamespace(xy=lambda at: np.array([10.0, 5.0]),
                            plan=SimpleNamespace(slot_z=lambda p: (3.0 * p.layer,
                                                                   3.0 * p.layer + 3.0)))
    return build, shapes


@pytest.mark.no_fabricate
@pytest.mark.parametrize(("part", "inside"), [
    # a tube in layers 0-1 (r 3 in claims of 3): inside
    (lambda: disc((10, 5), 3.0, 0, 6) - disc((10, 5), 1.5, -1, 7), True),
    # its axis off by 0.03 mm: past the 0.02 mm the envelope grows by
    (lambda: disc((10.03, 5), 3.0, 0, 6), False),
    # into layer 2, whose claim is narrower
    (lambda: disc((10, 5), 3.0, 0, 7), False),
    # a speck past the column's top
    (lambda: disc((10, 5), 1.0, 1, 12.001), False),
    # a box whose corners are inside (planar faces: their edges' ends)
    (lambda: box((10, 5), (2.0, 2.0, 5.0), 0.5, 0.0), True),
    # a box whose corners are 2.83 mm out in layer 2 (r 2.5)
    (lambda: box((10, 5), (4.0, 4.0, 2.0), 6.5, 0.0), False),
    # a sphere: not a face the test reads, so it says nothing
    (lambda: Sphere(1.0).moved(Location((10, 5, 4))), False),
])
def test_in_a_column_only_says_inside_for_a_part_inside(part, inside):
    """:func:`contract._in_a_column` says a part is inside its column of disc claims only
    when it is (and the boolean agrees); otherwise the contract runs the boolean."""
    build, shapes = _column([3.0, 3.0, 2.5, 3.0])
    p = part()
    assert contract._in_a_column(build, p, shapes) == inside
    if inside:
        env = union([disc((10, 5), s.shape.r + contract.TOL, *build.plan.slot_z(s))
                     for s in shapes])
        assert contract._outside(p, env) < 1e-9


@pytest.mark.parametrize(("linkage", "module"), quick(
    [("hoecken", "single"), ("klann", "single"), ("strider", "double")], [("hoecken", "single")]))
def test_in_a_column_agrees_with_the_boolean(design, linkage, module):
    """On a real side (the quick tier: the Hoecken's axles; ~1 s): every part the
    boolean-free test calls inside its claims is inside by the boolean too."""
    tmpl, d = design(module, linkage=linkage)
    build = Build(d.ctx, d.plan, tmpl.freeze_at(2.2))
    envelope = contract._Envelopes(build)
    groups = None if linkage != "hoecken" else [
        g.name for g in d.groups if g.name.startswith(("pin:", "pillar:"))]
    done, n = Realized(), 0
    for g in contract._realized_for(d, groups):
        got = g.realize(build, done)
        done.merge(got)
        if g.name == "frame":
            continue
        for b, shapes, clip in contract._items(build, g, got):
            if not clip and contract._in_a_column(build, b.part, shapes):
                assert contract._outside(b.part, envelope(shapes)) < 1e-6, b.name
                n += 1
    assert n > 0
