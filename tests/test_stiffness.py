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


def test_where_the_load_comes_in_and_whats_driven_is_by_capability():
    """stiffness.outputs and drives (2026-10-14): an arm's tool point and its inputs' drives,
    a mechanism's output and a walker's feet, each with its crank held at its centre; no
    kind's name asked."""
    from spiderpig.linkage.stiffness import drives, outputs

    pal = linkage.get("palletizer")
    assert pal.output is not None
    assert outputs(pal) == ((pal.output.link, pal.output.point),)
    assert drives(pal) == {pal.drives[k][0]: pal.drives[k][1] for k in pal.inputs}
    strider = linkage.get("strider")
    assert outputs(strider) == (("b3", "J4"), ("b7", "J8"))
    assert drives(strider) == {"conn": "O"}
    hp = linkage.get("hoecken_pantograph")
    assert hp.output is not None
    assert outputs(hp) == ((hp.output.link, hp.output.point),)
    assert drives(hp) == {"conn": hp.crank[0]}


def test_a_pin_partway_along_a_bar_splits_its_beam():
    """Strider's b2 is a bar J3-J9 with the crankpin J1 partway along it: its beams run
    J3-J1-J9 (and J9-J10), so the crank's load reaches the link."""
    from spiderpig.linkage.engine import LegSolution
    from spiderpig.linkage.stiffness import _beams

    lk = linkage.get("strider")
    js, segs = lk.links["b2"]
    at = {k: v.reshape(-1, 2)[0] for k, v in LegSolution(1, 0.0, lk.values(), lk.key)
          .at(np.array([0.3])).items()}
    assert sorted(_beams(list(js), segs, at)) == sorted(
        [("J3", "J1"), ("J1", "J9"), ("J9", "J10")])
    assert len(_beams(["A", "B", "C"], (), at={})) == 3          # no outline: every pair


def test_a_walkers_feet_across_the_plane():
    """stiffness.across of a walker in one plane: each foot's compliance across it, finite
    and the same for the Strider's two feet (its leg is symmetric in them), the foot link
    the most tilted; aluminium links (69 GPa) over 15 times stiffer than acrylic (3.2)."""
    from spiderpig.linkage.stiffness import across

    lk = linkage.get("strider")
    acrylic = across(lk, lk.values(), Section(12.0, 3.0, 3200.0))
    assert [(a.body, a.point) for a in acrylic] == [("b3", "J4"), ("b7", "J8")]
    for a in acrylic:
        assert 0.1 < a.mm_per_n < 100
        assert math.isfinite(a.tilt_deg_per_n)
        assert a.tilt_link == a.body
        assert 0 <= a.at < 2 * math.pi
    assert acrylic[0].mm_per_n == pytest.approx(acrylic[1].mm_per_n, rel=1e-6)
    al = across(lk, lk.values(), Section(12.0, 3.175, 69000.0))
    assert al[0].mm_per_n < acrylic[0].mm_per_n / 15
