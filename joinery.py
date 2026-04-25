"""Joinery: explicit hardware (pins, screws, bushings, brackets) at joints.

A :class:`Joinery` is a mechanism-rewrite primitive in the same style as
:func:`klann.combine_connectors` / :func:`klann.fuse_couplers`: given a
single connection between two bodies, :meth:`Joinery.apply` (and its
template sibling :meth:`Joinery.apply_template`) inserts hardware bodies,
rewires the connection, and optionally cuts negatives into the host
bodies.

Two responsibilities:

1. **Spacing** — the Z gap between the parent and child joint contacts is
   declared on the joinery, replacing the implicit ``layer * thickness``
   stagger that lived in :func:`klann._jp` / :func:`klann._jpt`.
2. **Geometry** — each :class:`JoineryPiece` along the pin axis becomes a
   :class:`mechanism.Body` (single-t) or :class:`mechanism.BodyTemplate`
   (parametric-in-t) bound to either the parent or child host. Stacked
   hardware (screw + washer + nut) is just multiple pieces with mixed
   binds.

Plus an optional ``host_negative`` hook so a joinery can subtract clearance
holes / counterbores / keyway slots from its hosts.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Literal

import numpy as np

from mechanism import (
    Body,
    BodyTemplate,
    Joint,
    JointTemplate,
    Mechanism,
    MechanismTemplate,
    Pose,
)

Bind = Literal["parent", "child"]


@dataclass(frozen=True)
class JoineryPiece:
    """One rigid component along a pin axis.

    ``z_offset`` is along the pin axis from the joinery's tail (the parent
    contact). ``bind`` selects which host the piece rides with in the
    animation: a screw head bound to ``"parent"`` rotates with the parent
    link; a nut bound to ``"child"`` rotates with the child link.
    """

    name: str
    part: object | None = None
    z_offset: float = 0.0
    bind: Bind = "parent"
    color: str | None = None


@dataclass(frozen=True)
class Joinery:
    """A pin assembly between two joint frames.

    ``spacing`` is the Z gap between the joinery's tail (parent contact)
    and head (child contact). ``pieces`` are the rigid components along
    the axis. ``bracket_offset_xyz`` lets a clevis or bracket introduce a
    fixed XY shift between the parent and child contacts so the child
    joint isn't co-axial with the parent's pin.

    Subclass and override :meth:`host_negative` to cut clearance into the
    parent/child bodies. Built-in :class:`LayerStaggerPin`,
    :class:`ClevisHole`, :class:`Standoff` cover the patterns the existing
    Klann assembly uses.
    """

    name_prefix: str = "joinery"
    spacing: float = 0.0
    pieces: tuple[JoineryPiece, ...] = ()
    bracket_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)

    # ---- host coupling hooks ----

    def host_negative(
        self,
        side: Bind,
        host_body: Body,
        joint_pose_in_host: Pose,
    ) -> object | None:
        """Return a build123d Part to subtract from ``host_body.part``,
        positioned in ``host_body``'s local frame. Default: no cut.

        Subclasses override to declare clevis clearance holes, counterbores,
        keyway slots, etc. Branch on ``side`` for asymmetric cuts (e.g.,
        clearance hole on the parent, threaded hole on the child).
        """
        return None

    # ---- rewrite primitives ----

    def apply(
        self,
        mech: Mechanism,
        *,
        parent: tuple[str, str],
        child: tuple[str, str],
        index: int = 0,
    ) -> Mechanism:
        """Rewrite ``mech`` to insert this joinery between two bodies.

        Removes the connection ``((parent_body, parent_joint),
        (child_body, child_joint))``, applies host negatives, and inserts
        a kinematic spacer body (carrying the parent→child Z spacing
        constraint) plus one body per :class:`JoineryPiece`.
        """
        return _apply_to_mechanism(
            self, mech, parent=parent, child=child, index=index
        )

    def apply_template(
        self,
        tmpl: MechanismTemplate,
        *,
        parent: tuple[str, str],
        child: tuple[str, str],
        index: int = 0,
    ) -> MechanismTemplate:
        """Parametric-in-t sibling of :meth:`apply`.

        All joinery joint poses are time-invariant — joinery doesn't move
        relative to its host's local frame — so spacer/piece body
        templates use :func:`constant_pose_at`. The ``host_negative`` step
        is a no-op for templates (they carry no parts; the bake picks up
        the host-cut version of each part from the single-t reference
        build).
        """
        return _apply_to_template(
            self, tmpl, parent=parent, child=child, index=index
        )


# ---------------------------------------------------------------------------
# Module-level helpers (promoted from klann.py:947 _identity_pose_at).
# ---------------------------------------------------------------------------


def constant_pose_at(pose: Pose) -> Callable[[np.ndarray], np.ndarray]:
    """Return a ``pose_at`` callable that yields ``pose`` for every t.

    Joinery bodies' joint poses are time-invariant (joinery itself doesn't
    move relative to its host body's local frame), so all joinery
    :class:`JointTemplate` instances use this.
    """
    matrix = pose.matrix

    def _fn(ts: np.ndarray) -> np.ndarray:
        ts = np.asarray(ts, dtype=float)
        return np.broadcast_to(matrix, (ts.shape[0], 4, 4)).copy()

    return _fn


# ---------------------------------------------------------------------------
# Internal: shared apply logic.
# ---------------------------------------------------------------------------


def _spacer_head_pose(
    spacing: float, bracket_xyz: tuple[float, float, float]
) -> Pose:
    """Pose of the spacer body's ``head`` joint in its own local frame."""
    bx, by, _ = bracket_xyz
    return Pose.from_translation([bx, by, spacing])


def _piece_part_offset(
    piece: JoineryPiece,
    spacing: float,
    bracket_xyz: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Where to place ``piece.part`` in its own body's local frame.

    The piece body's mate joint sits at the body origin; the part is
    shifted along the pin axis so its world position lands at
    ``host_joint_world + (0, 0, z_offset)``:

    - ``bind="parent"``: host is parent at world ``parent_joint``; place
      part at ``(0, 0, z_offset)``.
    - ``bind="child"``: host is child at world ``parent_joint +
      bracket_xyz + (0, 0, spacing)``; place part at
      ``(-bx, -by, z_offset - spacing)`` so it lands at
      ``parent_joint + (0, 0, z_offset)`` after BFS aligns the mate joint.
    """
    if piece.bind == "parent":
        return (0.0, 0.0, piece.z_offset)
    bx, by, _ = bracket_xyz
    return (-bx, -by, piece.z_offset - spacing)


def _edges_match(
    a: tuple[str, str],
    b: tuple[str, str],
    target: tuple[tuple[str, str], tuple[str, str]],
) -> bool:
    """True iff ``(a, b)`` matches ``target`` in either direction."""
    return (a, b) == target or (b, a) == target


def _drop_target_connection(connections, target):
    """Yield connection tuples from ``connections`` minus those matching ``target``."""
    out = []
    for (pi, pn, pj), (ci, cn, cj) in connections:
        if _edges_match((pn, pj), (cn, cj), target):
            continue
        out.append(((pi, pn, pj), (ci, cn, cj)))
    return out


def _apply_host_negative(
    joinery: Joinery, side: Bind, host: Body, joint_pose: Pose
) -> object | None:
    """If the joinery declares a cut for this side, subtract it from the host part.

    Returns the new host part, or ``None`` if no change. The cut is
    supplied by ``host_negative`` in host-local coordinates (the joinery
    is responsible for positioning it at the joint location).
    """
    if host.part is None:
        return None
    cut = joinery.host_negative(side, host, joint_pose)
    if cut is None:
        return None
    return host.part - cut


def _apply_to_mechanism(
    joinery: Joinery,
    mech: Mechanism,
    *,
    parent: tuple[str, str],
    child: tuple[str, str],
    index: int,
) -> Mechanism:
    parent_body_name, parent_joint_name = parent
    child_body_name, child_joint_name = child

    parent_body = mech.body(parent_body_name)
    child_body = mech.body(child_body_name)
    parent_joint_pose = parent_body.joint(parent_joint_name).pose
    child_joint_pose = child_body.joint(child_joint_name).pose

    new_parent_part = _apply_host_negative(
        joinery, "parent", parent_body, parent_joint_pose
    )
    new_child_part = _apply_host_negative(
        joinery, "child", child_body, child_joint_pose
    )

    # Spacer body: pure kinematic carrier of the parent→child Z gap.
    spacer_name = f"{joinery.name_prefix}{index}_spacer"
    spacer = Body(
        name=spacer_name,
        part=None,
        joints=[
            Joint(name="tail", pose=Pose.identity()),
            Joint(
                name="head",
                pose=_spacer_head_pose(joinery.spacing, joinery.bracket_offset_xyz),
            ),
        ],
        color=None,
    )

    # One body per piece, mated to its bind side via a single "mate" joint.
    piece_bodies: list[Body] = []
    piece_connections: list = []
    for piece in joinery.pieces:
        piece_body_name = f"{joinery.name_prefix}{index}_{piece.name}"
        if piece.part is not None:
            from build123d import Pos  # lazy build123d import

            ox, oy, oz = _piece_part_offset(
                piece, joinery.spacing, joinery.bracket_offset_xyz
            )
            placed = (
                piece.part.moved(Pos(ox, oy, oz)) if (ox or oy or oz) else piece.part
            )
        else:
            placed = None

        piece_bodies.append(
            Body(
                name=piece_body_name,
                part=placed,
                joints=[
                    Joint(name="mate", pose=Pose.identity()),
                    # Witness joint at (1, 0, 0) — not part of any connection,
                    # but Procrustes in viewer/bake_gltf.py needs ≥2 joints to
                    # recover rotation. Without this, rotationally asymmetric
                    # hardware (hex nuts, brackets, keyed pins) would translate
                    # but not rotate with the host link.
                    Joint(name="_witness", pose=Pose.from_translation([1.0, 0.0, 0.0])),
                ],
                color=piece.color,
            )
        )
        host_name = parent_body_name if piece.bind == "parent" else child_body_name
        host_joint = (
            parent_joint_name if piece.bind == "parent" else child_joint_name
        )
        piece_connections.append(
            ((0, host_name, host_joint), (0, piece_body_name, "mate"))
        )

    # Rebuild bodies with replaced parent/child + appended joinery bodies.
    bodies: list[Body] = []
    for b in mech.bodies:
        if b.name == parent_body_name and new_parent_part is not None:
            bodies.append(replace(b, part=new_parent_part))
        elif b.name == child_body_name and new_child_part is not None:
            bodies.append(replace(b, part=new_child_part))
        else:
            bodies.append(b)
    bodies.append(spacer)
    bodies.extend(piece_bodies)

    target = ((parent_body_name, parent_joint_name), (child_body_name, child_joint_name))
    new_connections = _drop_target_connection(mech.connections, target)
    new_connections.append(
        ((0, parent_body_name, parent_joint_name), (0, spacer_name, "tail"))
    )
    new_connections.append(
        ((0, spacer_name, "head"), (0, child_body_name, child_joint_name))
    )
    new_connections.extend(piece_connections)

    return Mechanism(name=mech.name, bodies=bodies, connections=new_connections)


def _apply_to_template(
    joinery: Joinery,
    tmpl: MechanismTemplate,
    *,
    parent: tuple[str, str],
    child: tuple[str, str],
    index: int,
) -> MechanismTemplate:
    parent_body_name, parent_joint_name = parent
    child_body_name, child_joint_name = child

    spacer_name = f"{joinery.name_prefix}{index}_spacer"
    spacer = BodyTemplate(
        name=spacer_name,
        joints=[
            JointTemplate(name="tail", pose_at=constant_pose_at(Pose.identity())),
            JointTemplate(
                name="head",
                pose_at=constant_pose_at(
                    _spacer_head_pose(joinery.spacing, joinery.bracket_offset_xyz)
                ),
            ),
        ],
        color=None,
    )

    piece_bodies: list[BodyTemplate] = []
    piece_connections: list = []
    for piece in joinery.pieces:
        piece_body_name = f"{joinery.name_prefix}{index}_{piece.name}"
        piece_bodies.append(
            BodyTemplate(
                name=piece_body_name,
                joints=[
                    JointTemplate(
                        name="mate", pose_at=constant_pose_at(Pose.identity())
                    ),
                    # Witness joint — see comment in _apply_to_mechanism.
                    JointTemplate(
                        name="_witness",
                        pose_at=constant_pose_at(
                            Pose.from_translation([1.0, 0.0, 0.0])
                        ),
                    ),
                ],
                color=piece.color,
            )
        )
        host_name = parent_body_name if piece.bind == "parent" else child_body_name
        host_joint = (
            parent_joint_name if piece.bind == "parent" else child_joint_name
        )
        piece_connections.append(
            ((0, host_name, host_joint), (0, piece_body_name, "mate"))
        )

    bodies = list(tmpl.bodies) + [spacer] + piece_bodies

    target = ((parent_body_name, parent_joint_name), (child_body_name, child_joint_name))
    new_connections = _drop_target_connection(tmpl.connections, target)
    new_connections.append(
        ((0, parent_body_name, parent_joint_name), (0, spacer_name, "tail"))
    )
    new_connections.append(
        ((0, spacer_name, "head"), (0, child_body_name, child_joint_name))
    )
    new_connections.extend(piece_connections)

    return MechanismTemplate(name=tmpl.name, bodies=bodies, connections=new_connections)


# ---------------------------------------------------------------------------
# Built-in joineries.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class LayerStaggerPin(Joinery):
    """Pure-spacing joinery: replaces the implicit ``layer * thickness`` Z stagger.

    No positive geometry, no host cut — equivalent to the original
    :func:`klann._jp` pos/neg layer math, just declared as joinery. Used
    everywhere the original mechanism builder would have offset two joints
    in Z to keep adjacent links from intersecting.
    """

    name_prefix: str = "stagger"


@dataclass(frozen=True)
class ClevisHole(Joinery):
    """Pure-spacing joinery that also cuts a 4 mm clearance hole on each side.

    Replaces the per-joint ``clevis_neg`` call inside
    :func:`shapes.rationalize_segment`: the hole is no longer subtracted
    blindly when the link is built, but only at the joints that actually
    have a pin going through them. This is what every layered Klann joint
    used to look like before the joinery refactor.
    """

    name_prefix: str = "clevis_hole"
    hole_radius: float = 2.0
    hole_height: float = 20.0

    def host_negative(
        self,
        side: Bind,
        host_body: Body,
        joint_pose_in_host: Pose,
    ) -> object | None:
        from build123d import Align, Cylinder  # lazy

        cyl = Cylinder(
            radius=self.hole_radius,
            height=self.hole_height,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
        return cyl.moved(joint_pose_in_host.to_location())


# ---------------------------------------------------------------------------
# Built-in joineries that carry positive printed-pin geometry.
#
# These wrap the build123d primitives that lived in :func:`shapes.connector`
# and :func:`shapes.standoff` so callers get a uniform API: declare a
# joinery, apply it to a connection, and the hardware shows up as bodies
# in the mechanism (bake-aware, animated by the existing per-body Procrustes
# pipeline).
#
# Both default to ``spacing=0`` so they can be layered on top of the existing
# ``_jp`` pos/neg Z-stagger system without disrupting the canonical layout.
# Set ``spacing`` explicitly to make joinery own the parent→child Z gap.
# ---------------------------------------------------------------------------


def ClevisPin(
    *,
    spacing: float = 0.0,
    base_height: float = 4.0,
    shaft_radius: float = 1.9,
    shaft_height: float = 8.0,
    cap_height: float = 4.0,
    cap_gap: float = 6.4,
    pin_anchor_z: float | None = None,
    bind: Bind = "parent",
    color: str = "blue",
    name_prefix: str = "clevis_pin",
) -> Joinery:
    """Factory: 3D-printed pin + cap that pins two layered links together.

    Geometry mirrors :func:`shapes.connector`: a 4 mm-radius base disc
    (height ``base_height``) with a ``shaft_radius`` mm-radius shaft
    (height ``shaft_height``) capped by a second 4 mm-radius disc.
    The whole pin is one rigid piece bound to ``bind`` (default
    ``"parent"`` — the pin rides with the parent link).

    ``pin_anchor_z`` controls where along the pin the kinematic mate
    point sits (0 = base of the pin, ``shaft_height/2 + base_height`` =
    middle of the shaft). The default centers the mate at the shaft
    midpoint so the pin spans both layered links above and below the
    joint, matching how :func:`shapes.connector` wired its ``"pin"``
    joint.

    Defaults to ``spacing=0`` so it can be layered on top of the existing
    ``_jp`` Z-stagger system without disrupting the canonical layout. Set
    ``spacing`` explicitly for joinery to own the parent→child Z gap.

    For stacked hardware (two-piece pin where the head rotates with one
    side and the nut with the other), construct :class:`Joinery` directly
    with multiple :class:`JoineryPiece` entries.
    """
    from build123d import Cylinder, Pos  # lazy

    if pin_anchor_z is None:
        pin_anchor_z = base_height + shaft_height / 2
    base = Cylinder(radius=4.0, height=base_height).move(Pos(0, 0, base_height / 2))
    shaft = Cylinder(radius=shaft_radius, height=shaft_height).move(
        Pos(0, 0, base_height + shaft_height / 2)
    )
    cap = Cylinder(radius=4.0, height=cap_height).move(
        Pos(0, 0, base_height + cap_gap + cap_height / 2)
    )
    bottom = base + shaft
    dimple_cap = cap - bottom
    part = bottom + dimple_cap

    return Joinery(
        name_prefix=name_prefix,
        spacing=spacing,
        pieces=(
            JoineryPiece(
                name="pin",
                part=part,
                z_offset=-pin_anchor_z,
                bind=bind,
                color=color,
            ),
        ),
    )


def Standoff(
    *,
    thickness: float = 6.0,
    peg_height: float = 5.5,
    clearance: float = 3.2,
    spacing: float = 0.0,
    bind: Bind = "child",
    color: str = "yellow",
    name_prefix: str = "standoff",
) -> Joinery:
    """Factory: stackable 3D-printed standoff bridging an upper-deck pivot to ground.

    Geometry mirrors :func:`shapes.standoff`: a ``thickness`` mm-tall
    base disc with a 1.9 mm pin protruding upward and a clearance dimple
    on the far side. Bound to ``"child"`` by default — the standoff is
    fixed to ground (anchored at the joint world position) while the
    parent link rotates above it.

    Used in the original ``_add_standoffs`` flow to ground upper-deck
    legs whose torso has been dropped by ``fuse_torsos``.
    """
    from build123d import Cylinder, Pos  # lazy

    base = Cylinder(radius=4.0, height=thickness).move(Pos(0, 0, thickness / 2))
    b_placed = base.move(Pos(0, 0, -thickness))
    peg = Cylinder(radius=1.9, height=peg_height).move(Pos(0, 0, peg_height / 2))
    peg_neg = peg.move(Pos(0, 0, -(clearance + thickness)))
    unit = b_placed + peg - peg_neg

    return Joinery(
        name_prefix=name_prefix,
        spacing=spacing,
        pieces=(JoineryPiece(name="post", part=unit, z_offset=0.0, bind=bind, color=color),),
    )
