"""Tests for 2016-style multi-linkage assemblies.

Covers the mirrored pair (``build_double_klann``), the 90°-phase Z-stack
(``build_double_decker_klann``), and the 4-leg walker combining both patterns
(``build_double_double_decker_klann``). Also exercises the small primitives
those builders share — ``voffset_joint``, ``combine_connectors``,
``fuse_couplers``, ``fuse_torsos``.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

# viewer/ is a sibling of the package modules; make it importable for the
# ``_class_of`` tests below.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "viewer"))

from klann import (
    build_double_decker_klann,
    build_double_double_decker_klann,
    build_double_klann,
    build_klann_mechanism,
    create_klann_geometry,
    voffset_joint,
)
from mechanism import Joint, Pose


# ---------------------------------------------------------------------------
# Primitive unit tests
# ---------------------------------------------------------------------------


def test_voffset_joint_pure():
    j = Joint("pin", Pose.from_translation([1.0, 2.0, 3.0]))
    shifted = voffset_joint(j, 5.0)
    assert shifted.name == "pin"
    assert shifted.pose.matrix[2, 3] == pytest.approx(8.0)
    # original untouched
    assert j.pose.matrix[2, 3] == pytest.approx(3.0)


def test_voffset_joint_renames():
    j = Joint("A", Pose.identity())
    renamed = voffset_joint(j, -6.0, new_name="A_leg1")
    assert renamed.name == "A_leg1"
    assert renamed.pose.matrix[2, 3] == pytest.approx(-6.0)


def test_mirror_orientation_swaps_M_layer():
    """Mirrored legs own the M joint on the opposite layer from the primary leg.

    A standalone mirrored leg can't close every connection without a
    torso-level Z-voffset (that's what ``fuse_torsos`` applies). The
    invariant we enforce here is the *structural* one: b1 / conn must trade
    their M joint Z heights when ``orientation=-1``. Full joint closure is
    covered by ``test_double_klann_all_connections_close`` once the torso
    offsets are applied.
    """
    sol_p = create_klann_geometry(orientation=+1, phase=0.0)
    sol_m = create_klann_geometry(orientation=-1, phase=0.0)
    mech_p = build_klann_mechanism(sol_p, t=0.7, thickness=3.0, with_parts=False)
    mech_m = build_klann_mechanism(sol_m, t=0.7, thickness=3.0, with_parts=False)

    def _m_z(mech, body_name):
        b = mech.body(body_name)
        return float(b.joint("M").pose.matrix[2, 3])

    # +1 leg: b1.M lives on the neg layer (z=0), conn.M on pos (z=3).
    # -1 leg: the ownership swaps.
    assert _m_z(mech_p, "b1") == pytest.approx(0.0)
    assert _m_z(mech_p, "conn") == pytest.approx(3.0)
    assert _m_z(mech_m, "b1") == pytest.approx(3.0)
    assert _m_z(mech_m, "conn") == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# build_double_klann — mirrored pair
# ---------------------------------------------------------------------------


def test_double_klann_body_count():
    mech = build_double_klann(t=1.0, with_parts=False)
    names = [b.name for b in mech.bodies]
    # Exactly one of each shared body + four link bodies per leg
    assert names.count("torso") == 1
    assert names.count("coupler") == 1
    assert names.count("conn") == 1  # combined crank
    # Four per-leg laser-cut link bodies per leg × 2 legs = 8
    for prefix in ("b1", "b2", "b3", "b4"):
        assert sum(1 for n in names if n.startswith(f"{prefix}_leg")) == 2
    assert len(mech.bodies) == 11


def test_double_klann_all_connections_close():
    mech = build_double_klann(t=1.0, with_parts=False).solved()
    for (_, a_name, a_joint), (_, b_name, b_joint) in mech.connections:
        a = mech.body(a_name)
        b = mech.body(b_name)
        pa = (a.pose @ a.joint(a_joint).pose).matrix
        pb = (b.pose @ b.joint(b_joint).pose).matrix
        np.testing.assert_allclose(
            pa[:3, 3], pb[:3, 3], atol=1e-6,
            err_msg=f"{a_name}.{a_joint} ≠ {b_name}.{b_joint}",
        )


def test_double_klann_mirror_produces_distinct_foot():
    """The mirrored leg's foot trace must differ from the primary leg's.

    (M lies on a circle at O that is *not* reflected — only A/B rotation
    signs flip and the opposite circle-intersection is picked — so F is
    not a simple X-negation; we just assert the two traces are meaningfully
    distinct at a handful of crank angles.)
    """
    sol_r = create_klann_geometry(orientation=+1, phase=0.0)
    sol_l = create_klann_geometry(orientation=-1, phase=0.0)
    for t in (0.1, 1.0, 2.5):
        fr = sol_r.joints_at(t)["F"]
        fl = sol_l.joints_at(t)["F"]
        dist = math.hypot(fr[0] - fl[0], fr[1] - fl[1])
        assert dist > 50.0  # many mm apart at every sampled t


def test_double_klann_shared_crank_is_single_body():
    mech = build_double_klann(t=1.0, with_parts=False)
    conn_bodies = [b for b in mech.bodies if b.name.startswith("conn")]
    assert len(conn_bodies) == 1
    # The combined crank carries both legs' O/M joints, renamed with suffixes.
    names = {j.name for j in conn_bodies[0].joints}
    assert names == {"O_leg0", "M_leg0", "O_leg1", "M_leg1"}


def test_double_klann_shared_torso_pivot_stagger():
    """The shared torso's per-leg A/B pivots must be offset in Z by the 6mm stride."""
    mech = build_double_klann(t=1.0, with_parts=False)
    torso = mech.body("torso")
    joint_z = {j.name: float(j.pose.matrix[2, 3]) for j in torso.joints}
    # leg0 keeps its native z (thickness=3), leg1 pins are shifted down 6mm.
    assert joint_z["A_leg0"] - joint_z["A_leg1"] == pytest.approx(6.0, abs=1e-9)
    assert joint_z["B_leg0"] - joint_z["B_leg1"] == pytest.approx(6.0, abs=1e-9)


# ---------------------------------------------------------------------------
# build_double_decker_klann — 90° phase, Z-stacked
# ---------------------------------------------------------------------------


def test_double_decker_body_count():
    mech = build_double_decker_klann(t=1.0, with_parts=False)
    names = [b.name for b in mech.bodies]
    assert names.count("torso") == 1
    assert names.count("coupler") == 1
    # Each leg keeps its own conn (no shared crank in the decker).
    assert sum(1 for n in names if n.startswith("conn_leg")) == 2
    assert sum(1 for n in names if n.startswith("standoff")) == 2
    # Four links per leg × 2 legs = 8
    for prefix in ("b1", "b2", "b3", "b4"):
        assert sum(1 for n in names if n.startswith(f"{prefix}_leg")) == 2
    assert len(mech.bodies) == 14


def test_double_decker_phase_offset():
    """Leg 1's foot at time t matches leg 0's foot at t + π/2."""
    sol0 = create_klann_geometry(orientation=+1, phase=0.0)
    sol1 = create_klann_geometry(orientation=+1, phase=math.pi / 2)
    for t in (0.0, 0.5, 2.0 * math.pi - 0.3):
        f1_at_t = sol1.joints_at(t)["F"]
        f0_shifted = sol0.joints_at(t + math.pi / 2)["F"]
        np.testing.assert_allclose(f1_at_t, f0_shifted, atol=1e-9)


def test_double_decker_z_stacking():
    mech = build_double_decker_klann(t=0.3, with_parts=False, z_deck=15.0).solved()
    # Upper leg conn originates at z ≈ z_deck + thickness (layer offset from coupler).
    conn_leg1_z = mech.body("conn_leg1").pose.matrix[2, 3]
    conn_leg0_z = mech.body("conn_leg0").pose.matrix[2, 3]
    assert conn_leg1_z - conn_leg0_z == pytest.approx(15.0, abs=1e-6)


def test_double_decker_shared_coupler_has_two_O_joints():
    mech = build_double_decker_klann(t=0.3, with_parts=False, z_deck=15.0)
    coupler = mech.body("coupler")
    names = sorted(j.name for j in coupler.joints)
    assert names == ["O_leg0", "O_leg1"]
    z = {j.name: float(j.pose.matrix[2, 3]) for j in coupler.joints}
    assert z["O_leg1"] - z["O_leg0"] == pytest.approx(15.0, abs=1e-9)


def test_double_decker_all_connections_close():
    mech = build_double_decker_klann(t=1.0, with_parts=False).solved()
    for (_, a_name, a_joint), (_, b_name, b_joint) in mech.connections:
        a = mech.body(a_name)
        b = mech.body(b_name)
        pa = (a.pose @ a.joint(a_joint).pose).matrix
        pb = (b.pose @ b.joint(b_joint).pose).matrix
        np.testing.assert_allclose(
            pa[:3, 3], pb[:3, 3], atol=1e-6,
            err_msg=f"{a_name}.{a_joint} ≠ {b_name}.{b_joint}",
        )


# ---------------------------------------------------------------------------
# build_double_double_decker_klann — 4-leg walker
# ---------------------------------------------------------------------------


def test_quad_body_count():
    mech = build_double_double_decker_klann(t=1.0, with_parts=False)
    names = [b.name for b in mech.bodies]
    assert names.count("torso") == 1
    assert names.count("coupler") == 1
    # One combined crank per deck — lower (conn) and upper (conn_upper).
    assert names.count("conn") == 1
    assert names.count("conn_upper") == 1
    # Both combined cranks absorbed their per-leg originals.
    assert sum(1 for n in names if n.startswith("conn_leg")) == 0
    # Two standoffs per upper-deck leg × 2 upper legs = 4
    assert sum(1 for n in names if n.startswith("standoff")) == 4
    # Four per-leg laser-cut links × 4 legs = 16
    for prefix in ("b1", "b2", "b3", "b4"):
        assert sum(1 for n in names if n.startswith(f"{prefix}_leg")) == 4
    assert len(mech.bodies) == 24


def test_quad_all_connections_close():
    mech = build_double_double_decker_klann(t=1.0, with_parts=False).solved()
    for (_, a_name, a_joint), (_, b_name, b_joint) in mech.connections:
        a = mech.body(a_name)
        b = mech.body(b_name)
        pa = (a.pose @ a.joint(a_joint).pose).matrix
        pb = (b.pose @ b.joint(b_joint).pose).matrix
        np.testing.assert_allclose(
            pa[:3, 3], pb[:3, 3], atol=1e-6,
            err_msg=f"{a_name}.{a_joint} ≠ {b_name}.{b_joint}",
        )


def test_quad_four_coupler_O_joints():
    """Coupler carries one O joint per leg — lower pair coplanar, upper pair at z_deck."""
    mech = build_double_double_decker_klann(t=0.5, with_parts=False, z_deck=15.0)
    coupler = mech.body("coupler")
    z = {j.name: float(j.pose.matrix[2, 3]) for j in coupler.joints}
    assert set(z) == {"O_leg0", "O_leg1", "O_leg2", "O_leg3"}
    assert z["O_leg0"] == pytest.approx(z["O_leg1"], abs=1e-9)
    assert z["O_leg2"] == pytest.approx(z["O_leg3"], abs=1e-9)
    assert z["O_leg2"] - z["O_leg0"] == pytest.approx(15.0, abs=1e-9)


def test_quad_upper_crank_is_combined_body():
    """Upper-deck mirrored pair fuses into a single rigid body driven by the shared shaft."""
    mech = build_double_double_decker_klann(t=1.0, with_parts=False)
    conn_upper = mech.body("conn_upper")
    names = {j.name for j in conn_upper.joints}
    assert names == {"O_leg2", "M_leg2", "O_leg3", "M_leg3"}


def test_quad_upper_legs_grounded_via_standoffs():
    mech = build_double_double_decker_klann(t=1.0, with_parts=False)
    edges = mech.connections
    for suffix in ("_leg2", "_leg3"):
        a_edges = [
            (a, b) for a, b in edges if a[1] == f"b3{suffix}" and a[2] == "A"
        ]
        b_edges = [
            (a, b) for a, b in edges if a[1] == f"b2{suffix}" and a[2] == "B"
        ]
        assert len(a_edges) == 1, f"{suffix}: expected one standoff edge at b3.A"
        assert a_edges[0][1][1].startswith("standoff")
        assert len(b_edges) == 1, f"{suffix}: expected one standoff edge at b2.B"
        assert b_edges[0][1][1].startswith("standoff")


def test_quad_upper_crank_lifts_to_z_deck():
    """Once solved, the upper combined crank sits at z_deck so its mesh doesn't
    render at the lower-deck origin (the pre-fix 'hanging crankshaft' symptom)."""
    mech = build_double_double_decker_klann(t=0.3, with_parts=False, z_deck=15.0).solved()
    lower = mech.body("conn").pose.matrix[2, 3]
    upper = mech.body("conn_upper").pose.matrix[2, 3]
    assert upper - lower == pytest.approx(15.0, abs=1e-6)


# ---------------------------------------------------------------------------
# bake_gltf class-name compatibility
# ---------------------------------------------------------------------------


def test_class_of_survives_new_body_names():
    from bake_gltf import _class_of

    assert _class_of("b1_leg3") == "b1"
    assert _class_of("coupler_leg0") == "coupler"
    assert _class_of("conn") == "conn"                # lower combined crank
    assert _class_of("conn_upper") == "conn_upper"    # upper combined crank — distinct class
    assert _class_of("torso") == "torso"
    assert _class_of("coupler") == "coupler"          # fused coupler
    assert _class_of("standoff0") == "standoff"
    assert _class_of("standoff12") == "standoff"
    assert _class_of("conn_leg2") == "conn"
