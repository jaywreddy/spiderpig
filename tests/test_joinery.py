"""Tests for the :mod:`joinery` rewrite primitive."""

from __future__ import annotations

import numpy as np

from joinery import (
    ClevisHole,
    Joinery,
    JoineryPiece,
    LayerStaggerPin,
    constant_pose_at,
)
from mechanism import (
    Body,
    BodyTemplate,
    Joint,
    JointTemplate,
    Mechanism,
    MechanismTemplate,
    Pose,
)


def _two_body() -> Mechanism:
    """Two bodies sharing a joint at the origin (no Z stagger yet)."""
    a = Body(name="a", joints=[Joint("p", Pose.identity())])
    b = Body(name="b", joints=[Joint("p", Pose.identity())])
    return Mechanism(
        name="pair",
        bodies=[a, b],
        connections=[((0, "a", "p"), (1, "b", "p"))],
    )


def _two_body_template() -> MechanismTemplate:
    a = BodyTemplate(
        name="a",
        joints=[JointTemplate("p", pose_at=constant_pose_at(Pose.identity()))],
    )
    b = BodyTemplate(
        name="b",
        joints=[JointTemplate("p", pose_at=constant_pose_at(Pose.identity()))],
    )
    return MechanismTemplate(
        name="pair",
        bodies=[a, b],
        connections=[((0, "a", "p"), (1, "b", "p"))],
    )


def test_layer_stagger_pin_introduces_z_gap():
    """LayerStaggerPin(spacing=3) should make child sit 3 above parent in Z."""
    mech = _two_body()
    mech = LayerStaggerPin(spacing=3.0).apply(
        mech, parent=("a", "p"), child=("b", "p")
    )
    solved = mech.solved()
    np.testing.assert_allclose(solved.body("a").pose.matrix[:3, 3], [0, 0, 0])
    np.testing.assert_allclose(solved.body("b").pose.matrix[:3, 3], [0, 0, 3])


def test_apply_drops_original_connection_and_inserts_chain():
    mech = LayerStaggerPin(spacing=2.0).apply(
        _two_body(), parent=("a", "p"), child=("b", "p"), index=0
    )
    # Original a↔b edge should be gone; spacer should chain a→spacer→b.
    edges = {((pn, pj), (cn, cj)) for (_, pn, pj), (_, cn, cj) in mech.connections}
    assert (("a", "p"), ("b", "p")) not in edges
    assert (("b", "p"), ("a", "p")) not in edges
    spacer = "stagger0_spacer"
    assert (("a", "p"), (spacer, "tail")) in edges
    assert ((spacer, "head"), ("b", "p")) in edges


def test_apply_preserves_all_other_connections():
    a = Body(name="a", joints=[Joint("p", Pose.identity())])
    b = Body(name="b", joints=[Joint("p", Pose.identity()), Joint("q", Pose.identity())])
    c = Body(name="c", joints=[Joint("q", Pose.identity())])
    mech = Mechanism(
        name="trip",
        bodies=[a, b, c],
        connections=[
            ((0, "a", "p"), (1, "b", "p")),
            ((1, "b", "q"), (2, "c", "q")),
        ],
    )
    mech = LayerStaggerPin(spacing=1.0).apply(
        mech, parent=("a", "p"), child=("b", "p")
    )
    edges = {((pn, pj), (cn, cj)) for (_, pn, pj), (_, cn, cj) in mech.connections}
    # b↔c connection must survive untouched.
    assert (("b", "q"), ("c", "q")) in edges


def test_multi_piece_joinery_emits_one_body_per_piece():
    parent_part = object()  # opaque sentinel — None would skip placement
    child_part = object()
    pieces = (
        JoineryPiece(name="head", part=parent_part, z_offset=0.0, bind="parent"),
        JoineryPiece(name="nut", part=child_part, z_offset=4.0, bind="child"),
    )
    # Substitute None for the actual build123d parts so we don't need OCCT
    # for this structural test.
    pieces = tuple(JoineryPiece(p.name, None, p.z_offset, p.bind) for p in pieces)
    j = Joinery(name_prefix="bolt", spacing=4.0, pieces=pieces)
    mech = j.apply(_two_body(), parent=("a", "p"), child=("b", "p"))

    body_names = {b.name for b in mech.bodies}
    assert "bolt0_spacer" in body_names
    assert "bolt0_head" in body_names
    assert "bolt0_nut" in body_names

    edges = {((pn, pj), (cn, cj)) for (_, pn, pj), (_, cn, cj) in mech.connections}
    # parent-bound piece mates with parent
    assert (("a", "p"), ("bolt0_head", "mate")) in edges
    # child-bound piece mates with child
    assert (("b", "p"), ("bolt0_nut", "mate")) in edges


def test_solve_closes_for_multi_piece_joinery():
    """Every connection in a joinery'd mechanism should align in world space."""
    pieces = (
        JoineryPiece(name="head", part=None, z_offset=0.0, bind="parent"),
        JoineryPiece(name="washer", part=None, z_offset=2.0, bind="parent"),
        JoineryPiece(name="nut", part=None, z_offset=5.0, bind="child"),
    )
    j = Joinery(name_prefix="bolt", spacing=5.0, pieces=pieces)
    mech = j.apply(_two_body(), parent=("a", "p"), child=("b", "p"))
    solved = mech.solved()
    for (_, an, aj), (_, bn, bj) in solved.connections:
        a = solved.body(an)
        b = solved.body(bn)
        pa = (a.pose @ a.joint(aj).pose).matrix[:3, 3]
        pb = (b.pose @ b.joint(bj).pose).matrix[:3, 3]
        np.testing.assert_allclose(
            pa, pb, atol=1e-9,
            err_msg=f"{an}.{aj} vs {bn}.{bj} did not align",
        )


def test_bracket_offset_shifts_child_in_xy():
    j = Joinery(spacing=3.0, bracket_offset_xyz=(5.0, -2.0, 0.0))
    mech = j.apply(_two_body(), parent=("a", "p"), child=("b", "p"))
    solved = mech.solved()
    np.testing.assert_allclose(
        solved.body("b").pose.matrix[:3, 3], [5.0, -2.0, 3.0]
    )


def test_template_apply_matches_mechanism_at_t0():
    """apply_template + sample(ts=[0]) should put bodies at the same world
    positions as apply + solved() at t=0."""
    spacing = 4.0
    mech = LayerStaggerPin(spacing=spacing).apply(
        _two_body(), parent=("a", "p"), child=("b", "p")
    )
    solved = mech.solved()

    tmpl = LayerStaggerPin(spacing=spacing).apply_template(
        _two_body_template(), parent=("a", "p"), child=("b", "p")
    )
    sampled = tmpl.sample(np.array([0.0]))

    for body in solved.bodies:
        np.testing.assert_allclose(
            sampled.world[body.name][0],
            body.pose.matrix,
            atol=1e-12,
            err_msg=f"template and mechanism diverge for body {body.name!r}",
        )


def test_clevis_hole_cuts_into_host_part():
    """ClevisHole.host_negative removes volume from a unit-cube host part."""
    from build123d import Align, Box

    cube = Box(20, 20, 20, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    original_volume = cube.volume

    a = Body(name="a", part=cube, joints=[Joint("p", Pose.identity())])
    b = Body(name="b", joints=[Joint("p", Pose.identity())])
    mech = Mechanism(
        name="cut",
        bodies=[a, b],
        connections=[((0, "a", "p"), (1, "b", "p"))],
    )
    mech = ClevisHole(spacing=3.0, hole_radius=2.0, hole_height=20.0).apply(
        mech, parent=("a", "p"), child=("b", "p")
    )
    new_part = mech.body("a").part
    assert new_part is not None
    assert new_part.volume < original_volume
    # Hole volume = pi * r^2 * h = pi * 4 * 20 ~ 251.3
    expected_removed = np.pi * 4.0 * 20.0
    np.testing.assert_allclose(
        original_volume - new_part.volume, expected_removed, rtol=0.05
    )


def test_default_joinery_does_not_cut_hosts():
    """LayerStaggerPin has no host_negative — host parts pass through."""
    sentinel = object()

    class _DummyPart:
        def __init__(self, marker):
            self.marker = marker
        def __sub__(self, other):
            raise AssertionError("LayerStaggerPin should not cut hosts")

    a = Body(name="a", part=_DummyPart(sentinel), joints=[Joint("p", Pose.identity())])
    b = Body(name="b", joints=[Joint("p", Pose.identity())])
    mech = Mechanism(
        name="nocut",
        bodies=[a, b],
        connections=[((0, "a", "p"), (1, "b", "p"))],
    )
    mech = LayerStaggerPin(spacing=3.0).apply(
        mech, parent=("a", "p"), child=("b", "p")
    )
    assert mech.body("a").part.marker is sentinel


def test_clevis_pin_factory_builds_one_piece_joinery():
    """ClevisPin produces a Joinery with a single parent-bound piece."""
    from joinery import ClevisPin

    j = ClevisPin()
    assert len(j.pieces) == 1
    assert j.pieces[0].name == "pin"
    assert j.pieces[0].bind == "parent"
    assert j.pieces[0].part is not None
    assert j.spacing == 0.0  # layered atop existing _jp Z stagger
    assert j.name_prefix == "clevis_pin"


def test_standoff_factory_binds_to_child():
    """Standoff defaults to bind=child (anchored ground while parent rotates)."""
    from joinery import Standoff

    j = Standoff()
    assert len(j.pieces) == 1
    assert j.pieces[0].bind == "child"
    assert j.pieces[0].part is not None


def test_clevis_pin_apply_adds_pin_body_to_mechanism():
    """Applying ClevisPin inserts a pin body that survives BFS solve."""
    from joinery import ClevisPin

    mech = ClevisPin().apply(_two_body(), parent=("a", "p"), child=("b", "p"))

    body_names = {b.name for b in mech.bodies}
    assert "clevis_pin0_pin" in body_names
    pin_body = mech.body("clevis_pin0_pin")
    assert pin_body.part is not None

    # Solve closes — every connection's joints align in world space.
    solved = mech.solved()
    for (_, an, aj), (_, bn, bj) in solved.connections:
        a_w = (solved.body(an).pose @ solved.body(an).joint(aj).pose).matrix[:3, 3]
        b_w = (solved.body(bn).pose @ solved.body(bn).joint(bj).pose).matrix[:3, 3]
        np.testing.assert_allclose(a_w, b_w, atol=1e-9)


def test_clevis_pin_applied_to_real_klann_assembly():
    """End-to-end: apply ClevisPin to the O joint of a real Klann linkage,
    confirm the pin body lands at the world position of O, and BFS solve
    still closes every connection (including the new joinery edges)."""
    from joinery import ClevisPin
    from klann import KlannLinkage

    mech = KlannLinkage()
    pin = ClevisPin()
    mech = pin.apply(
        mech, parent=("conn", "O"), child=("coupler", "O"), index=0
    )

    body_names = {b.name for b in mech.bodies}
    assert "clevis_pin0_pin" in body_names
    assert "clevis_pin0_spacer" in body_names

    solved = mech.solved()
    # Closure: every connection's joints align in world space.
    for (_, an, aj), (_, bn, bj) in solved.connections:
        a_w = (solved.body(an).pose @ solved.body(an).joint(aj).pose).matrix[:3, 3]
        b_w = (solved.body(bn).pose @ solved.body(bn).joint(bj).pose).matrix[:3, 3]
        np.testing.assert_allclose(
            a_w, b_w, atol=1e-6,
            err_msg=f"{an}.{aj} vs {bn}.{bj} did not align in solved KlannLinkage",
        )

    # Pin body world position should match the parent (conn) joint world position
    # (because the pin is bind="parent" with z_offset=0, mating at conn.O).
    conn = solved.body("conn")
    pin_body = solved.body("clevis_pin0_pin")
    conn_O_world = (conn.pose @ conn.joint("O").pose).matrix[:3, 3]
    pin_origin_world = pin_body.pose.matrix[:3, 3]
    np.testing.assert_allclose(pin_origin_world, conn_O_world, atol=1e-6)


def test_stacked_pin_with_mixed_binds_separates_pieces_per_side():
    """Worked example: a 4-piece pin (head + washer parent-side, washer + nut
    child-side) produces four distinct bodies, two mated to parent, two to child."""
    pieces = (
        JoineryPiece(name="head", part=None, z_offset=0.0, bind="parent"),
        JoineryPiece(name="washer_top", part=None, z_offset=2.0, bind="parent"),
        JoineryPiece(name="washer_bot", part=None, z_offset=4.0, bind="child"),
        JoineryPiece(name="nut", part=None, z_offset=6.0, bind="child"),
    )
    j = Joinery(name_prefix="bolt", spacing=6.0, pieces=pieces)
    mech = j.apply(_two_body(), parent=("a", "p"), child=("b", "p"), index=7)

    edges = {((pn, pj), (cn, cj)) for (_, pn, pj), (_, cn, cj) in mech.connections}
    # head + washer_top mate with parent
    assert (("a", "p"), ("bolt7_head", "mate")) in edges
    assert (("a", "p"), ("bolt7_washer_top", "mate")) in edges
    # washer_bot + nut mate with child
    assert (("b", "p"), ("bolt7_washer_bot", "mate")) in edges
    assert (("b", "p"), ("bolt7_nut", "mate")) in edges
