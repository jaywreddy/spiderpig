"""Tests for the leg modules (``linkage.build_module_template``): the mirrored pair
(``double``), two legs 90° apart on one crankshaft (``decker``), the 4-leg walker
combining both (``quad``), and the template rewrites they share
(``combine_connectors``, ``fuse_couplers``, ``fuse_torsos``).

Kinematics only: Z placement belongs to the stack plan (``test_stack.py``).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

import linkage
from linkage import build_module_template, combine_connectors, fuse_torsos

TS = np.linspace(0.0, 2.0 * math.pi, 24, endpoint=False)
KLANN = linkage.get("klann")


def _assert_connections_close(tmpl):
    """Every connection pins two joints that coincide at every crank angle, in the plane."""
    sampled = tmpl.sample(TS)
    for (_, a, ja), (_, b, jb) in tmpl.connections:
        np.testing.assert_allclose(sampled.joint_world[a][ja], sampled.joint_world[b][jb],
                                   atol=1e-6, err_msg=f"{a}.{ja} ≠ {b}.{jb}")
    for body, joints in sampled.joint_world.items():      # planar: Z is the stack plan's job
        for name, xyz in joints.items():
            assert np.all(xyz[:, 2] == 0.0), f"{body}.{name} has a Z offset"


def _body_names(module: str) -> list[str]:
    return [b.name for b in build_module_template(module).bodies]


# ---------------------------------------------------------------------------
# Template rewrites
# ---------------------------------------------------------------------------


def test_combine_connectors_merges_joints_and_outline():
    tmpl = combine_connectors(linkage.legs_template("pair", [(+1, 0.0), (-1, 0.0)]),
                              "_leg0", "_leg1")
    conn = tmpl.body("conn")
    assert {j.name for j in conn.joints} == {"O_leg0", "M_leg0", "O_leg1", "M_leg1"}
    assert conn.outline == (("O_leg0", "M_leg0"), ("O_leg1", "M_leg1"))
    assert not any(b.name.startswith("conn_leg") for b in tmpl.bodies)
    # every edge that targeted a per-leg crank now targets the fused one
    ends = {(b, j) for edge in tmpl.connections for (_, b, j) in edge}
    assert ("conn", "M_leg1") in ends
    assert not any(b.startswith("conn_leg") for b, _ in ends)


def test_fuse_torsos_shares_O_and_suffixes_pivots():
    tmpl = build_module_template("double")
    torso = tmpl.body("torso")
    assert sorted(j.name for j in torso.joints) == ["A_leg0", "A_leg1", "B_leg0", "B_leg1", "O"]
    with pytest.raises(KeyError):
        fuse_torsos(tmpl, ["_leg0"])  # already fused: no torso_leg0 left


# ---------------------------------------------------------------------------
# double: the mirrored pair
# ---------------------------------------------------------------------------


def test_double_body_count():
    names = _body_names("double")
    assert names.count("torso") == names.count("coupler") == names.count("conn") == 1
    for prefix in ("b1", "b2", "b3", "b4"):
        assert sum(1 for n in names if n.startswith(f"{prefix}_leg")) == 2
    assert len(names) == 11


def test_double_all_connections_close():
    _assert_connections_close(build_module_template("double"))


def test_double_mirror_produces_distinct_foot():
    """M is not reflected (only A/B and the intersection branch flip), so the
    mirrored foot is not a simple X-negation; check the traces differ."""
    sol_r, sol_l = KLANN.solve(+1), KLANN.solve(-1)
    for t in (0.1, 1.0, 2.5):
        fr, fl = sol_r.joints_at(t)["F"], sol_l.joints_at(t)["F"]
        assert math.hypot(fr[0] - fl[0], fr[1] - fl[1]) > 50.0


def test_double_mirrored_legs_share_the_crankpin():
    conn = build_module_template("double").body("conn")
    ts = np.array([0.7])
    np.testing.assert_allclose(conn.joint("M_leg0").eval(ts)[0, :2, 3],
                               conn.joint("M_leg1").eval(ts)[0, :2, 3], atol=1e-12)


# ---------------------------------------------------------------------------
# decker: two legs 90° apart on one crankshaft
# ---------------------------------------------------------------------------


def test_decker_body_count():
    names = _body_names("decker")
    assert names.count("torso") == names.count("coupler") == 1
    assert sum(1 for n in names if n.startswith("conn_leg")) == 2
    for prefix in ("b1", "b2", "b3", "b4"):
        assert sum(1 for n in names if n.startswith(f"{prefix}_leg")) == 2
    assert len(names) == 12


def test_decker_frame_holds_both_decks():
    tmpl = build_module_template("decker")
    torso = tmpl.body("torso")
    assert sorted(j.name for j in torso.joints) == ["A_leg0", "A_leg1", "B_leg0", "B_leg1", "O"]
    # same chirality: both decks pivot on the same A and B
    np.testing.assert_allclose(torso.joint("A_leg0").eval(TS), torso.joint("A_leg1").eval(TS),
                               atol=1e-12)
    _assert_connections_close(tmpl)


# ---------------------------------------------------------------------------
# quad: the 4-leg walker
# ---------------------------------------------------------------------------


def test_quad_body_count():
    names = _body_names("quad")
    assert names.count("torso") == names.count("coupler") == 1
    assert names.count("conn") == names.count("conn_upper") == 1
    assert not any(n.startswith(("conn_leg", "standoff")) for n in names)
    for prefix in ("b1", "b2", "b3", "b4"):
        assert sum(1 for n in names if n.startswith(f"{prefix}_leg")) == 4
    assert len(names) == 20


def test_quad_all_connections_close():
    _assert_connections_close(build_module_template("quad"))


def test_quad_cranks_are_arms_across_the_centre():
    """Each mirrored pair's crank is one bar through O with a crankpin at each end."""
    tmpl = build_module_template("quad")
    ts = np.array([0.4])
    for name, (a, b) in {"conn": ("_leg0", "_leg1"), "conn_upper": ("_leg2", "_leg3")}.items():
        crank = tmpl.body(name)
        ma = crank.joint(f"M{a}").eval(ts)[0, :2, 3]
        mb = crank.joint(f"M{b}").eval(ts)[0, :2, 3]
        np.testing.assert_allclose(ma, -mb, atol=1e-9)


def test_quad_every_leg_pivots_on_the_frame():
    torso_joints = {j.name for j in build_module_template("quad").body("torso").joints}
    for k in range(4):
        assert {f"A_leg{k}", f"B_leg{k}"} <= torso_joints
