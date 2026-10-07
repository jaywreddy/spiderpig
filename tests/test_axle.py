"""Tests for the generic axle claims (:class:`construction.axle.AxleGroup`).

Whatever the construction (the Chicago screw pin, the standoff pillar), an axle claims its
seat in each of its links' layers, a shoulder beside each link, a spacer in every other
layer of its retained stack, its anchors in the frame plates it reaches and its retainers
beyond each end; its early claim (once its own links have layers) is a lower bound of all
that. (Inside their claims and clash-free: ``test_contract.py``; each construction's own
parts: ``test_standoff.py``, ``test_wobble.py``. The printed stepped axle these tests were
written on was removed on 2026-10-07.)
"""

from __future__ import annotations

import pytest

from spiderpig.construction.axle import STOP_OVERLAP, AxleGroup
from tests.tiers import quick

T = 1.0


def _label(p) -> str:
    return p.label.rsplit(" ", 1)[1]


@pytest.fixture(params=quick(["single", "double", "decker", "quad"], ["single", "double"]))
def axles(request, design):
    """``(design, its axle groups)`` per Klann module, the default constructions (the
    decker and the quad, the same rules on more axles, in the slow tier)."""
    _, d = design(request.param)
    return d, [g for g in d.groups if isinstance(g, AxleGroup)]


def test_the_default_axles_are_chicago_pins_and_standoff_pillars(axles):
    _, groups = axles
    assert groups
    for g in groups:
        assert g.construction.key == ("standoff" if g.pillar else "chicago"), g.name


def test_every_link_sits_on_its_axle(axles):
    design, groups = axles
    plan = design.plan
    for g in groups:
        d = g.dims(design.ctx)
        seats = {p.layer: p for p in plan.shapes(g.name) if _label(p) == "axle"}
        assert set(seats) == {plan.layers[m] for m in g.axis.members}, g.name
        for p in seats.values():
            assert p.seat
            assert not p.gap
            assert p.shape.r == pytest.approx(d.axle)


def test_a_shoulder_beside_each_link_and_a_spacer_elsewhere(axles):
    """Every layer of the retained stack between its ends is the axle's: a link's, a
    shoulder right beside a link (wide enough to overlap its hole), else a loose spacer,
    each at most ``spacer`` and at least the ``neck``."""
    design, groups = axles
    plan, p = design.plan, design.ctx.params
    for g in groups:
        d = g.dims(design.ctx)
        stop = p.hole(2 * d.axle) / 2 + STOP_OVERLAP
        shapes = [s for s in plan.shapes(g.name) if not s.gap]
        links = {plan.layers[m] for m in g.axis.members}
        anchors = {s.layer for s in shapes if _label(s) == "anchor"}
        k0 = 0 if 0 in anchors else min(links)
        k1 = plan.top if plan.top in anchors else max(links)
        for k in range(k0 + 1, k1):
            if k in links:
                continue
            (s,) = [s for s in shapes if s.layer == k]
            want = "shoulder" if {k - 1, k + 1} & links else "spacer"
            assert _label(s) == want, (g.name, k, s.label)
            assert d.neck - 1e-9 <= s.shape.r <= d.spacer + 1e-9, (g.name, k)
            if want == "shoulder":
                assert s.shape.r >= stop - 1e-9, (g.name, k)


def test_pillars_are_anchored_and_pins_retained_at_both_ends(axles):
    """A pillar is anchored in a frame plate it reaches (both where it can: a beam; else a
    cantilever, its free end retained over its last link); a pin has a head below its
    lowest link and a cap above its highest, each in the clearance gap beside its link or
    in the layer beyond."""
    design, groups = axles
    plan = design.plan
    beams = 0
    for g in groups:
        shapes = plan.shapes(g.name)
        links = sorted(plan.layers[m] for m in g.axis.members)
        lo, hi = links[0], links[-1]
        if g.pillar:
            anchors = sorted(s.layer for s in shapes if _label(s) == "anchor")
            assert anchors in ([0], [plan.top], [0, plan.top]), (g.name, anchors)
            assert all(s.seat for s in shapes if _label(s) == "anchor")
            beams += anchors == [0, plan.top]
            heads = sorted(s.layer for s in shapes if _label(s) == "head" and not s.gap)
            # a screw head outside each plate it is anchored in, or over its free end
            want = [-1 if 0 in anchors else lo - 1, plan.top + 1 if plan.top in anchors
                    else hi + 1]
            assert heads == want, (g.name, heads, want)
        else:
            assert not [s for s in shapes if _label(s) == "anchor"], g.name
            (head,) = [s for s in shapes if _label(s) == "head"]
            (cap,) = [s for s in shapes if _label(s) == "cap"]
            assert head.layer == lo - 1, g.name          # in the gap under lo, or sunk
            assert head.toward == -1 if head.gap else True
            assert cap.layer == (hi if cap.gap else hi + 1), g.name
            assert cap.toward == +1 if cap.gap else True
    assert beams                    # the Klann's frame pivots reach both plates


def test_washers_only_in_the_plans_gaps_inside_the_retained_stack(axles):
    design, groups = axles
    plan = design.plan
    for g in groups:
        washers = [s for s in plan.shapes(g.name) if _label(s) == "washer"]
        links = sorted(plan.layers[m] for m in g.axis.members)
        for w in washers:
            assert w.gap
            assert plan.gaps.get(w.layer, 0.0) > 0, (g.name, w.layer)
            if not g.pillar:
                assert links[0] <= w.layer < links[-1], (g.name, w.layer)


def test_the_early_claim_is_a_lower_bound(axles):
    """What the search places once an axle's own links have layers never claims more than
    the axle finally does: each early shape is covered by the final claim's in its layer (a
    retainer the search puts in the clearance gap beside a link may finally sink into the
    layer beyond the gap: a gap ``k`` is over layer ``k``)."""
    design, groups = axles
    plan = design.plan
    for g in groups:
        (claim,) = g.claims(design.ctx)
        assert claim.early_deps == frozenset(g.axis.members)
        final = plan.shapes(g.name)
        for e in claim.early(plan.layout):
            if e.gap:
                sunk = e.layer + (1 if e.toward > 0 else 0)
                cover = [s for s in final if (s.gap and s.layer == e.layer)
                         or (not s.gap and s.layer == sunk)]
            else:
                cover = [s for s in final if not s.gap and s.layer == e.layer]
            r = max((s.shape.r for s in cover), default=0.0)
            assert r >= e.shape.r - 1e-9, (g.name, e.layer, e.label)
