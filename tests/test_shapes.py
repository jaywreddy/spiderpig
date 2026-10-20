"""The build123d primitives' booleans (``spiderpig/shapes.py``): every one returns one
shape. build123d returns a ``ShapeList`` (a list) when a boolean whose operands are all
``Solid`` leaves several pieces; pyright's ``ShapeList`` findings of 2026-10-08
(``plates.py``, ``chassis.py``, ``deck.py``) are those operators' declared types."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from spiderpig.construction.contract import bad_solids
from spiderpig.shapes import (
    Rect,
    cut_holes,
    difference,
    disc,
    intersection,
    pill,
    pills,
    plate,
    union,
)

pytestmark = pytest.mark.no_fabricate


def test_holes_that_split_a_part_leave_one_shape():
    """A cut across a one-pill plate (a ``Solid``) leaves two pieces: before 2026-10-08
    ``cut_holes`` returned them as a ``ShapeList``, which the next boolean (``union``: no
    ``fuse``) and ``contract.bad_solids`` (no ``is_valid``) failed on with an
    ``AttributeError``. Now a ``Compound`` of both, reported as a part of two solids."""
    part = plate([((0.0, 0.0), (40.0, 0.0), 5.0)], 0.0, 3.0)
    split = cut_holes(part, [Rect((20.0, 0.0), (4.0, 20.0))], 0.0, 3.0)
    assert not isinstance(split, list)
    assert len(split.solids()) == 2
    assert len(union([split, disc((0.0, 0.0), 1.0, 0.0, 3.0)]).solids()) == 2
    mech = SimpleNamespace(bodies=[SimpleNamespace(name="link", part=split, fab="laser")])
    assert bad_solids(mech) == [{"part": "link", "solids": 2, "valid": True, "fab": "laser"}]


def test_difference_and_intersection_of_solids_are_one_shape():
    """Two ``Solid`` operands that leave two pieces: build123d's operator gives a list,
    :func:`shapes.difference` and :func:`shapes.intersection` one ``Compound``."""
    a, b = pill((0, 0), (30, 0), 5, 0, 3), pill((15, -10), (15, 10), 2, -1, 4)
    raw = a - b
    assert isinstance(raw, list)
    assert len(raw) == 2
    cut = difference(a, b)
    assert not isinstance(cut, list)
    assert len(cut.solids()) == 2
    both = intersection(cut, pill((0, 0), (30, 0), 5, 1, 2))
    assert not isinstance(both, list)
    assert len(both.solids()) == 2


def test_the_primitives_booleans_never_split_into_a_list():
    """Why ``plates.foot_sock``'s ``ring & half`` (pyright: ``&`` on a ``ShapeList``) never
    failed: :func:`shapes.disc` is a ``Cylinder`` (a ``Compound``), so ``disc - disc`` is a
    ``Compound`` whatever it leaves; the sock is one valid solid."""
    from spiderpig.construction.plates import foot_sock

    ring = disc((0, 0), 6, 0, 3) - disc((0, 0), 5, -1, 4)
    assert not isinstance(ring, list)
    apart = disc((0, 0), 6, 0, 3) - disc((0, 0), 30, 1, 2)     # two discs, one Compound
    assert not isinstance(apart, list)
    assert len(apart.solids()) == 2
    sock, notches = foot_sock((0.0, 0.0), (30.0, 0.0), 5.0, 0.0, 3.0)
    assert len(sock.solids()) == 1
    assert sock.is_valid
    assert len(notches) == 2


def test_pills_fused_in_2d_are_the_pills_union():
    """:func:`shapes.pills` (the sections fused in 2D, extruded once: the cradle's inner
    plate's keep-outs) is the point set :func:`shapes.union` of the :func:`shapes.pill` s
    makes: overlapping and disjoint runs along a closed path, a point pill, a second z
    extent; one shape, never a list, and each region one solid."""
    import itertools
    import math

    ring = [(40 * math.cos(a), 25 * math.sin(a))
            for a in (2 * math.pi * k / 24 for k in range(25))]
    specs = [(p, q, 3.0, -1.0, 4.0) for p, q in itertools.pairwise(ring)]
    specs += [((90.0, 0.0), (110.0, 5.0), 2.5, -1.0, 4.0),      # apart from the ring
              ((90.0, 30.0), (90.0, 30.0), 2.0, -1.0, 4.0),     # a point: a disc
              ((40.0, 0.0), (60.0, 0.0), 2.0, 4.0, 6.0)]        # another z extent
    got = pills(specs)
    want = union([pill(*s) for s in specs])
    assert not isinstance(got, list)
    assert got.volume == pytest.approx(want.volume, rel=1e-9)
    assert got.area == pytest.approx(want.area, rel=1e-9)
    left = [s for s in (got - want, want - got) if s is not None]
    assert sum(sum(x.volume for x in s) if isinstance(s, list) else s.volume
               for s in left) < 1e-6 * want.volume
    assert len(got.solids()) == len(want.solids())
    one = pills(specs[:-3])                 # the ring alone: one z extent, one region
    assert len(one.solids()) == 1
    assert one.is_valid
