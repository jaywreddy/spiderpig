"""The assembly guide (prototype, docs/agentlib/GUIDE.md): the step model's structure on the
default robot, and the renderer's determinism."""

from __future__ import annotations

import hashlib
import io

import numpy as np
import pytest

from spiderpig.config import BuildConfig
from spiderpig.guide.model import layers_of, rows_of, slot_of, steps
from spiderpig.guide.render import HIGHLIGHT, Item, View, render


def _box(lo, hi):
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    p = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)], float)
    q = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    t = np.array([(a, b, c) for a, b, c, d in q] + [(a, c, d) for a, b, c, d in q])
    return p, t.ravel()


@pytest.mark.no_fabricate
def test_render_draws_the_highlight_and_is_byte_stable():
    items = [Item("base", *_box((0, 0, 0), (40, 30, 2))),
             Item("new", *_box((5, 5, 2), (15, 25, 8)), fill=HIGHLIGHT, label="P01")]
    view = View(eye=(1.0, 0.9, 1.4), size=(160, 120))   # from the +z side, over the box
    a, b = render(items, view), render(items, view)
    img = np.asarray(a)
    assert img.shape == (120, 160, 3)
    orange = (img[..., 0] > 150) & (img[..., 2] < 90)
    assert orange.sum() > 100                       # the added part shows, highlighted
    assert (img.min(-1) < 60).sum() > 100           # with ink lines
    digest = [hashlib.sha256(_png(x)).hexdigest() for x in (a, b)]
    assert digest[0] == digest[1]


def _png(img) -> bytes:
    buf = io.BytesIO()
    img.save(buf, "PNG")
    return buf.getvalue()


@pytest.mark.no_fabricate
def test_a_gap_goes_with_the_layer_above_it():
    layers = {-1: (-3.0, 0.0), 0: (0.0, 2.0), 1: (4.7, 7.7), 2: (11.0, 14.0)}
    assert slot_of(-2.5, layers) == -1
    assert slot_of(2.0, layers) == 1       # a spacer on the plate's top: the gap under 1
    assert slot_of(7.7, layers) == 2
    assert slot_of(20.0, layers) == 3      # over the stack


@pytest.fixture(scope="module")
def default_steps():
    from tests import cache

    cfg = BuildConfig()
    mech = cache.cached_robot(cfg, 1.0)
    _, design = cache.cached_design(cfg)
    return mech, steps(rows_of(mech), layers_of(design.plan), mech.meta["mid_plane"])


@pytest.mark.slow
def test_every_part_is_added_exactly_once(default_steps):
    mech, st = default_steps
    added = [n for s in st for n in s.adds]
    assert len(added) == len(set(added))
    assert set(added) == {b.name for b in mech.bodies if b.part is not None}
    assert [s.number for s in st] == list(range(1, len(st) + 1))
    assert all(s.adds and s.text for s in st)


@pytest.mark.slow
def test_steps_build_each_side_bottom_up_and_place_their_sub_assemblies(default_steps):
    _mech, st = default_steps
    for s in st:
        assert not set(s.context) & set(s.adds)
        if s.sub:                               # drawn alone, then put on next step
            assert not s.context
            nxt = st[s.number]                  # (numbers are 1-based)
            assert set(s.adds) <= set(nxt.places)
    for side in "LR":
        layers = [int(s.title.rsplit(" ", 1)[-1]) for s in st
                  if s.title.startswith(f"{'Left' if side == 'L' else 'Right'} side: layer")]
        assert layers == sorted(layers)
        assert len(layers) >= 3
