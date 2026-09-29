"""Tests for 2016-style multi-linkage assemblies.

Covers the mirrored pair (``build_double_klann``), two legs 90° apart on one
crankshaft (``build_double_decker_klann``), and the 4-leg walker combining
both (``build_double_double_decker_klann``), plus the template rewrites they
share (``combine_connectors``, ``fuse_couplers``, ``fuse_torsos``).

Kinematics only (``with_parts=False``): Z placement belongs to the stack
plan and is covered by ``test_stack.py``.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "viewer"))

from klann import (  # noqa: E402
    build_double_decker_klann,
    build_double_double_decker_klann,
    build_double_klann,
    build_double_template,
    build_klann_template,
    combine_connectors,
    create_klann_geometry,
    fuse_torsos,
)


def _assert_connections_close(mech):
    solved = mech.solved()
    for (_, a_name, a_joint), (_, b_name, b_joint) in solved.connections:
        a, b = solved.body(a_name), solved.body(b_name)
        pa = (a.pose @ a.joint(a_joint).pose).matrix[:3, 3]
        pb = (b.pose @ b.joint(b_joint).pose).matrix[:3, 3]
        np.testing.assert_allclose(
            pa, pb, atol=1e-6, err_msg=f"{a_name}.{a_joint} ≠ {b_name}.{b_joint}"
        )


# ---------------------------------------------------------------------------
# Template rewrites
# ---------------------------------------------------------------------------


def test_combine_connectors_merges_joints_and_outline():
    legs = [
        build_klann_template(create_klann_geometry(+1), name_suffix="_leg0"),
        build_klann_template(create_klann_geometry(-1), name_suffix="_leg1"),
    ]
    from klann import _merge

    tmpl = combine_connectors(_merge("pair", legs), "_leg0", "_leg1")
    conn = tmpl.body("conn")
    assert {j.name for j in conn.joints} == {"O_leg0", "M_leg0", "O_leg1", "M_leg1"}
    assert conn.outline == (("O_leg0", "M_leg0"), ("O_leg1", "M_leg1"))
    assert not any(b.name.startswith("conn_leg") for b in tmpl.bodies)
    # every edge that targeted a per-leg crank now targets the fused one
    ends = {(b, j) for edge in tmpl.connections for (_, b, j) in edge}
    assert ("conn", "M_leg1") in ends
    assert not any(b.startswith("conn_leg") for b, _ in ends)


def test_fuse_torsos_shares_O_and_suffixes_pivots():
    tmpl = build_double_template()
    torso = tmpl.body("torso")
    assert sorted(j.name for j in torso.joints) == ["A_leg0", "A_leg1", "B_leg0", "B_leg1", "O"]
    with pytest.raises(KeyError):
        fuse_torsos(tmpl, ["_leg0"])  # already fused: no torso_leg0 left


# ---------------------------------------------------------------------------
# build_double_klann — mirrored pair
# ---------------------------------------------------------------------------


def test_double_klann_body_count():
    names = [b.name for b in build_double_klann(t=1.0, with_parts=False).bodies]
    assert names.count("torso") == names.count("coupler") == names.count("conn") == 1
    for prefix in ("b1", "b2", "b3", "b4"):
        assert sum(1 for n in names if n.startswith(f"{prefix}_leg")) == 2
    assert len(names) == 11


def test_double_klann_all_connections_close():
    _assert_connections_close(build_double_klann(t=1.0, with_parts=False))


def test_double_klann_mirror_produces_distinct_foot():
    """M is not reflected (only A/B and the intersection branch flip), so the
    mirrored foot is not a simple X-negation; check the traces differ."""
    sol_r = create_klann_geometry(orientation=+1, phase=0.0)
    sol_l = create_klann_geometry(orientation=-1, phase=0.0)
    for t in (0.1, 1.0, 2.5):
        fr, fl = sol_r.joints_at(t)["F"], sol_l.joints_at(t)["F"]
        assert math.hypot(fr[0] - fl[0], fr[1] - fl[1]) > 50.0


def test_double_klann_mirrored_legs_share_the_crankpin():
    mech = build_double_klann(t=0.7, with_parts=False)
    conn = mech.body("conn")
    m0 = conn.joint("M_leg0").pose.matrix[:2, 3]
    m1 = conn.joint("M_leg1").pose.matrix[:2, 3]
    np.testing.assert_allclose(m0, m1, atol=1e-12)


# ---------------------------------------------------------------------------
# build_double_decker_klann — two legs 90° apart on one crankshaft
# ---------------------------------------------------------------------------


def test_double_decker_body_count():
    names = [b.name for b in build_double_decker_klann(t=1.0, with_parts=False).bodies]
    assert names.count("torso") == names.count("coupler") == 1
    assert sum(1 for n in names if n.startswith("conn_leg")) == 2
    for prefix in ("b1", "b2", "b3", "b4"):
        assert sum(1 for n in names if n.startswith(f"{prefix}_leg")) == 2
    assert len(names) == 12


def test_double_decker_phase_offset():
    """Leg 1's foot at time t matches leg 0's foot at t + π/2."""
    sol0 = create_klann_geometry(orientation=+1, phase=0.0)
    sol1 = create_klann_geometry(orientation=+1, phase=math.pi / 2)
    for t in (0.0, 0.5, 2.0 * math.pi - 0.3):
        np.testing.assert_allclose(
            sol1.joints_at(t)["F"], sol0.joints_at(t + math.pi / 2)["F"], atol=1e-9
        )


def test_double_decker_frame_holds_both_decks():
    mech = build_double_decker_klann(t=1.0, with_parts=False)
    torso = mech.body("torso")
    assert sorted(j.name for j in torso.joints) == ["A_leg0", "A_leg1", "B_leg0", "B_leg1", "O"]
    # same chirality: both decks pivot on the same A and B
    np.testing.assert_allclose(
        torso.joint("A_leg0").pose.matrix, torso.joint("A_leg1").pose.matrix, atol=1e-12
    )
    _assert_connections_close(mech)


# ---------------------------------------------------------------------------
# build_double_double_decker_klann — 4-leg walker
# ---------------------------------------------------------------------------


def test_quad_body_count():
    names = [b.name for b in build_double_double_decker_klann(t=1.0, with_parts=False).bodies]
    assert names.count("torso") == names.count("coupler") == 1
    assert names.count("conn") == names.count("conn_upper") == 1
    assert not any(n.startswith(("conn_leg", "standoff")) for n in names)
    for prefix in ("b1", "b2", "b3", "b4"):
        assert sum(1 for n in names if n.startswith(f"{prefix}_leg")) == 4
    assert len(names) == 20


def test_quad_all_connections_close():
    _assert_connections_close(build_double_double_decker_klann(t=1.0, with_parts=False))


def test_quad_cranks_are_arms_across_the_centre():
    """Each mirrored pair's crank is one bar through O with a crankpin at each end."""
    mech = build_double_double_decker_klann(t=0.4, with_parts=False)
    for name, (a, b) in {"conn": ("_leg0", "_leg1"), "conn_upper": ("_leg2", "_leg3")}.items():
        crank = mech.body(name)
        ma = crank.joint(f"M{a}").pose.matrix[:2, 3]
        mb = crank.joint(f"M{b}").pose.matrix[:2, 3]
        np.testing.assert_allclose(ma, -mb, atol=1e-9)


def test_quad_every_leg_pivots_on_the_frame():
    mech = build_double_double_decker_klann(t=1.0, with_parts=False)
    torso_joints = {j.name for j in mech.body("torso").joints}
    for k in range(4):
        assert {f"A_leg{k}", f"B_leg{k}"} <= torso_joints
