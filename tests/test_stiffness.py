"""The links' bending (:mod:`spiderpig.linkage.stiffness`, 2026-10-12): the beam against the
cantilever's formulas, a load crossing between layers twisting a thin link, and the
palletizer's acrylic against aluminium."""

from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
import pytest

from spiderpig import linkage
from spiderpig.linkage.stiffness import Rod, Section, _beam, _pose, compliance

pytestmark = pytest.mark.no_fabricate

ACRYLIC = Section(13.2, 3.0, 3200.0)


def test_a_beam_is_a_cantilever_both_ways_and_a_rod_along_z():
    k = _beam(np.zeros(3), np.array([100.0, 0, 0]), ACRYLIC)[6:, 6:]
    w, t, e = 13.2, 3.0, 3200.0
    for axis, inertia in ((1, t * w ** 3 / 12), (2, w * t ** 3 / 12)):
        f = np.zeros(6)
        f[axis] = 1.0
        assert np.linalg.solve(k, f)[axis] == pytest.approx(100 ** 3 / (3 * e * inertia))
    rod = Rod(4.0)
    k = _beam(np.zeros(3), np.array([0, 0, 50.0]), rod)[6:, 6:]
    f = np.zeros(6)
    f[0] = 1.0
    assert np.linalg.solve(k, f)[0] == pytest.approx(
        50 ** 3 / (3 * rod.modulus_mpa * math.pi * 4 ** 4 / 64))


def test_a_load_crossing_layers_twists_the_thin_link_under_it():
    """A link on a fixed pivot, a pin at its end standing 20 mm up to the link that carries
    the load: the first link bends in its plane and twists by the load times 20 mm."""
    lay = SimpleNamespace(bodies=("torso", "A", "B"),
                          joints={"torso": ("G",), "A": ("G", "J"), "B": ("J", "T")},
                          axles=("G", "J"), output_body="B", output_point="T")
    at = {"G": np.array([0.0, 0.0]), "J": np.array([100.0, 0.0]), "T": np.array([101.0, 0.0])}
    stiff = {"G": Rod(4.0, 2e9), "J": Rod(4.0, 2e9)}
    fr, o = _pose(lay, at, ACRYLIC, {"A": 0.0, "B": 20.0}, stiff, None, {"A": "G"})
    fr.spring(2, None, [5], 1e9)                   # B held square on the pin (its drive)
    u = fr.tip(o, np.array([0.0, -1.0, 0.0]), np.zeros(3))
    w, t, e = 13.2, 3.0, 3200.0
    g = e / (2 * 1.35)
    j = w * t ** 3 / 3 * (1 - 0.63 * t / w)
    bend = 100 ** 3 / (3 * e * t * w ** 3 / 12)
    twist = 20 * 100 / (g * j) * 20
    assert -u[1] == pytest.approx(bend + twist, rel=1e-3)


def test_aluminium_links_sag_far_less_than_acrylic():
    pal = linkage.get("palletizer")
    acrylic = compliance(pal, pal.values(), ACRYLIC, payload_n=0.49)
    al = compliance(pal, pal.values(), Section(13.2, 3.175, 69000.0), payload_n=0.49)
    assert al.sag_mm < acrylic.sag_mm / 15
    assert al.across_mm_per_n < acrylic.across_mm_per_n / 15
