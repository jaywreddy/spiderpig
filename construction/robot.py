"""The robot: two mirror-image sides with their servos back to back in one frame.

In side coordinates (:mod:`fabricate`) the outer frame plate starts at
``z = 0``, the inner frame plate is on top and the servo stands on it. The
robot's mid-plane is where the two servos' rear faces meet a stack of
**centre plates** that the servos screw into. The left side ("L.") is its
side moved so that mid-plane is at ``z = 0``; the right side ("R.") is the
mirror image (``z -> -z``). The mechanism is planar, so both sides move
identically in XY; only their parts are mirrored. The servos therefore turn
in opposite senses about their own axes, which drives both sides forward.

The chassis (the frame the two servos live in) belongs to the robot, not a side:

**Centre plates** (laser-cut, laminated with the sheet's adhesive). There are
:func:`centre_plates` of them, enough that the two servos' rear bumps clear
each other; each plate has a relief cut-out only where a bump reaches it.
Both servos are real servos (the right one is not the left one's mirror
image), so the right servo's servo-frame ``+y`` points the other way in the
world. Each servo uses its rear holes on its own ``+y`` side, so the two
screw sets never share a hole position. The first ``n // 2`` plates on a
servo's side are its own:
its screws pass through them into the pilot holes (``REAR_ENGAGE`` of thread
in the case), and their heads bear on the last own plate, recessed in the
plates beyond it (for three plates: the middle one). Assembly: screw each
servo to its own plate(s), then laminate the stack with the middle plate(s)
between them, over the heads.

**Frame ties** (printed): four columns between the two inner frame plates,
beside the servo's long sides, make the frame rigid in bending and torsion.
Nothing may go below an inner plate (the leg stack is there, and the
planner doesn't know about the ties), so a column can't be clamped to a plate
by a screw head on the leg side. Instead each column is two halves that meet
the centre plates: a *screw half* on the left and an *insert half* on the
right, each seated in its inner plate with a spigot (flush with the plate's
leg-side face, bonded with CA glue) and bearing on the plate's top face. One
M3 socket head screw per tie runs down the screw half's bore, through the
centre plates, into an M3 heat-set insert in the insert half: it clamps both
halves to the centre plates and ties the two inner plates together through
them. Heat-set inserts rather than nuts: they are fused into the printed
half, so the screw pulls the half itself (a nut in an end pocket would only
bear on the centre plates), they need no side nut slot that weakens a
column, and the thread is reusable. The screw goes in from the leg side of
the left inner plate, through the spigot, before the legs are fitted.

:class:`FrameTies` is the side-level part of the ties: the spigot holes and
pads it adds to the inner frame plate (no claims: nothing it adds is below
the plate's top face). :func:`assemble_robot` places the ties themselves.

:mod:`construction.chassis` builds the chassis parts themselves (centre plates, rear
screws, tie columns) for :func:`assemble_robot`.
"""

from __future__ import annotations

from dataclasses import replace

from build123d import Location, Plane

from construction.base import FRAME_INNER, Build, Context, Group, Realized
from construction.chassis import centre_plates, chassis, servo_frame, tie_dims, tie_points
from mechanism import Body, Mechanism, MechanismTemplate
from shapes import Cut

SIDES = ("L", "R")


def prefixed(name: str | None, side: str) -> str | None:
    return None if name is None else f"{side}.{name}"


def robot_template(tmpl: MechanismTemplate) -> MechanismTemplate:
    """Both sides' kinematics: the side template twice, prefixed ``L.`` and ``R.``."""
    bodies, connections = [], []
    for side in SIDES:
        bodies += [replace(b, name=prefixed(b.name, side)) for b in tmpl.bodies]
        connections += [((i, prefixed(pb, side), pj), (k, prefixed(cb, side), cj))
                        for (i, pb, pj), (k, cb, cj) in tmpl.connections]
    return MechanismTemplate(name=f"{tmpl.name}_robot", bodies=bodies, connections=connections,
                             meta=dict(tmpl.meta))


def _rear_face(spec) -> float:
    return spec.rear_face_z if spec.rear_face_z is not None else spec.mount_face_z - spec.body[2]


def mid_plane_z(spec, top: int, pitch: float, margin: float) -> float:
    """Side-coordinate z of the robot's mid-plane for a stack of ``top + 1`` layers."""
    n = centre_plates(spec, pitch, margin)
    return (top + 1) * pitch + (spec.mount_face_z - _rear_face(spec)) + n * pitch / 2


def mid_plane(design) -> float:
    """Side-coordinate z of the robot's mid-plane."""
    return mid_plane_z(design.ctx.servo, design.plan.top, design.ctx.pitch,
                       design.ctx.params.margin)


class FrameTies(Group):
    """Side-level part of the frame ties: spigot holes and pads in the inner frame plate."""

    name = "frame ties"

    def __init__(self, drive):
        self.drive = drive

    def claims(self, ctx: Context) -> list:
        return []   # nothing below the inner plate's top face

    def realize(self, build: Build, done: Realized) -> Realized:
        out = Realized()
        p, d = build.ctx.params, tie_dims(build.ctx)
        frame = servo_frame(build, self.drive)
        for xy in tie_points(build, self.drive):
            x, _ = frame.local(xy)
            out.cut(FRAME_INNER, Cut(xy, p.hole(d.spigot_d, "glue")))
            out.pad(FRAME_INNER, xy, frame.xy(x, 0.0), d.column + p.min_wall)
        return out


# ---------------------------------------------------------------------------
# Assembly
# ---------------------------------------------------------------------------


def _moved(part, dz: float, mirror: bool):
    if part is None:
        return None
    if mirror:
        part = part.mirror(Plane.XY)
        dz = -dz
    return part.moved(Location((0.0, 0.0, dz)))


def assemble_robot(side: Mechanism, design) -> Mechanism:
    """Both sides of the robot and the frame between their servos."""
    z_mid = mid_plane(design)
    bodies: list[Body] = []
    connections = []
    for s in SIDES:
        mirror = s == "R"
        for b in side.bodies:
            bodies.append(Body(
                name=prefixed(b.name, s), part=_moved(b.part, -z_mid, mirror), joints=b.joints,
                color=b.color, pose=b.pose, outline=b.outline,
                rigid_with=prefixed(b.rigid_with, s), fab=b.fab, bom_key=b.bom_key,
            ))
        connections += [((i, prefixed(pb, s), pj), (k, prefixed(cb, s), cj))
                        for (i, pb, pj), (k, cb, cj) in side.connections]
    host = {s: prefixed(design.plan.topo.frame_bodies[0], s) for s in SIDES}
    parts, extras, info = chassis(side, design, z_mid, host)
    bodies += parts
    meta = dict(side.meta, robot=True, mid_plane=z_mid, filament="pla_filament", **info)
    return Mechanism(
        name=f"{side.name}_robot", bodies=bodies, connections=connections, meta=meta,
        bom_extras=list(side.bom_extras) * 2 + extras,
    )
