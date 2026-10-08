"""The build123d primitives' booleans (``spiderpig/shapes.py``): every one returns one
shape. build123d returns a ``ShapeList`` (a list) when a boolean whose operands are all
``Solid`` leaves several pieces; pyright's ``ShapeList`` findings of 2026-10-08
(``plates.py``, ``chassis.py``, ``deck.py``) are those operators' declared types."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from spiderpig.construction.contract import bad_solids
from spiderpig.shapes import Rect, cut_holes, difference, disc, intersection, pill, plate, union

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
