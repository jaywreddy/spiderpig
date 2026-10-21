"""An arm's relative limit (``Linkage.band``, 2026-10-12): the box, the tour and the ISO
poses stay in it; the reaching palletizer keeps its parallelograms off their dead points
and its gains under the base palletizer's."""

from __future__ import annotations

import math

import numpy as np
import pytest

from spiderpig import linkage
from spiderpig.linkage.constraints import check_conditioning, gains, sample_inputs
from spiderpig.repeatability import iso_inputs

pytestmark = pytest.mark.no_fabricate


def test_the_box_tour_and_iso_poses_keep_to_the_band():
    lk = linkage.get("palletizer_reach")
    lo, hi = lk.band
    for qs in (lk.box(15), lk.tour(np.linspace(0, 2 * math.pi, 400)), iso_inputs(lk)):
        d = np.degrees(qs[0] - qs[1])
        assert d.min() >= lo - 1e-9
        assert d.max() <= hi + 1e-9
    assert lk.in_band(math.radians(60), math.radians(-20))
    assert not lk.in_band(math.radians(100), math.radians(-60))        # 160 deg apart
    base = linkage.get("palletizer")                    # no band: the box as it was
    a, _ = base.box(3)
    assert np.degrees(a).tolist() == [40.0, 40.0, 40.0, 77.5, 77.5, 77.5, 115.0, 115.0,
                                      115.0]


@pytest.mark.slow     # (gains over two boxes: ~14 s)
def test_the_reaching_palletizer_is_better_conditioned_and_geared():
    reach, base = linkage.get("palletizer_reach"), linkage.get("palletizer")
    cond, _ = check_conditioning(reach, reach.values())
    assert cond < 500

    def worst(lk, joint):
        g = gains(lk, lk.values(), sample_inputs(lk, 9))
        return max(float(np.nanmax(np.linalg.norm(m, ord=2, axis=(1, 2))))
                   for (j, _), m in g.per.items() if j == joint)

    for joint in ("C", "R"):
        assert worst(reach, joint) < 0.6 * worst(base, joint)
