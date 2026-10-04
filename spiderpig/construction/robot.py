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

**Centre plates** (laser-cut from the frame's aluminium; no adhesive: the ties' studs
clamp the stack). There are :func:`centre_plates` of them, enough that the two servos' rear
bumps clear each other; each plate has a relief cut-out only where a bump reaches it.
Both servos are real servos (the right one is not the left one's mirror
image), so the right servo's servo-frame ``+y`` points the other way in the
world. Each servo uses its rear holes on its own ``+y`` side, so the two
screw sets never share a hole position. The first ``n // 2`` plates on a
servo's side are its own: its screws pass through them into the pilot holes
(``REAR_ENGAGE`` of thread in the case), and their heads bear on the last own
plate, recessed in the plates beyond it.

**Frame ties** (2026-10-04, no glue, no insert, no tapped plate): four columns between the
two inner frame plates, beside the servo's long sides, each a chain of goBILDA 1501 round
standoffs per side, from the inner plate's servo-side face to the centre plates (stock
lengths, DIN 988 shims at the plate). An M4 button head comes up through each inner plate
from the leg side into the chain (its head in the clearance gap under the plate, which the
drive group claims, so the planner keeps the legs clear of it), and an M4 set screw through
the centre plates joins the two chains and clamps the centre plates between them.
Assembly: the screws through the inner plates before the legs, the chains on them, the
centre plates and studs when the sides join.

:class:`FrameTies` is the side-level part of the ties: the spigot holes and
pads it adds to the inner frame plate (no claims: nothing it adds is below
the plate's top face). :func:`assemble_robot` places the ties themselves.

:mod:`construction.chassis` builds the chassis parts themselves (centre plates, rear
screws, tie columns) for :func:`assemble_robot`.
"""

from __future__ import annotations

import math
from dataclasses import replace

from build123d import Location, Plane

from spiderpig.construction.base import (
    FRAME_INNER,
    Build,
    ConstructionError,
    Context,
    Group,
    Realized,
)
from spiderpig.construction.chassis import (
    centre_plates,
    centre_t,
    chassis,
    servo_frame,
    tie_dims,
    tie_points,
)
from spiderpig.construction.deck import FLOOR_MARGIN as DECK_FLOOR_MARGIN
from spiderpig.construction.deck import (
    RAIL_HOLE,
    RAIL_SCREW_R,
    deck_parts,
    deck_place,
    rail_screw_points,
)
from spiderpig.mechanism import Body, Mechanism, MechanismTemplate
from spiderpig.shapes import Cut, moved

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


def mid_plane_z(spec, top: int, pitch: float, margin: float,
                height: float | None = None, centre: float | None = None) -> float:
    """Side-coordinate z of the robot's mid-plane for a stack of ``top + 1`` layers
    (``height``: the stack's own, its gaps and thicker plates included; else
    ``top + 1`` pitches; ``centre``: the centre plates' thickness, else the pitch)."""
    c = pitch if centre is None else centre
    n = centre_plates(spec, c, margin)
    h = (top + 1) * pitch if height is None else height
    return h + (spec.mount_face_z - _rear_face(spec)) + n * c / 2


def mid_plane(design) -> float:
    """Side-coordinate z of the robot's mid-plane."""
    return mid_plane_z(design.ctx.servo, design.plan.top, design.ctx.pitch,
                       design.ctx.params.margin, design.plan.height,
                       centre=centre_t(design.ctx))


class FrameTies(Group):
    """Side-level part of the frame ties and the deck rails: their screws' holes and pads in
    the inner frame plate (the screws' heads under the plate are the drive group's claims,
    :meth:`servos.mount.DriveGroup.claims`)."""

    name = "frame ties"

    def __init__(self, drive):
        self.drive = drive

    def claims(self, ctx: Context) -> list:
        return []   # the drive group claims the heads under the plate

    def realize(self, build: Build, done: Realized) -> Realized:
        out = Realized()
        p, d = build.ctx.params, tie_dims(build.ctx)
        frame = servo_frame(build, self.drive)
        for xy in tie_points(build, self.drive):
            x, _ = frame.local(xy)
            out.cut(FRAME_INNER, Cut(xy, d.hole_d))
            out.pad(FRAME_INNER, xy, frame.xy(x, 0.0), d.head_r + p.min_wall)
        try:                     # the electronics deck's rails (construction.deck)
            screws = rail_screw_points(build.ctx)
        except ConstructionError:
            screws = []          # no deck on this frame: assemble_robot says why
        for xy in screws:
            out.cut(FRAME_INNER, Cut(xy, RAIL_HOLE))
            out.pad(FRAME_INNER, xy, build.xy("O"), RAIL_SCREW_R + 2 * p.min_wall)
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
    return moved(part, Location((0.0, 0.0, dz)))


def _deck(side: Mechanism, design, z_mid: float, host, bodies) -> tuple[list, list, dict]:
    """The electronics deck (:mod:`construction.deck`): its bodies and purchases, and what
    ``mech.meta["deck"]`` says (``fitted: False`` and why when it doesn't fit). The rails
    must clear everything the chassis put between the inner plates: checked here on the
    real parts, not only on :func:`deck.deck_floor`'s estimate."""
    build = Build(design.ctx, design.plan, side)
    try:
        place = deck_place(build, design.drive)
    except ConstructionError as e:
        return [], [], {"fitted": False, "why": str(e)}
    z_in = abs(design.plan.z(design.plan.top)[1] - z_mid)
    top = max((bb.max.Y for b in bodies if b.part is not None
               for bb in (b.part.bounding_box(),)
               if z_in + 1e-6 > bb.max.Z and -z_in - 1e-6 < bb.min.Z), default=-math.inf)
    if top > place.rail_y0 - 0.5 * DECK_FLOOR_MARGIN:      # deck_floor missed a part
        return [], [], {"fitted": False,
                        "why": f"the chassis reaches y = {top:.1f} mm between the inner plates, "
                               f"over the deck rails' underside at {place.rail_y0:.1f} mm"}
    parts, extras, info, fastened = deck_parts(design, z_mid, place, host)
    info["chassis_top_y"] = round(top, 2)
    info["fastened"] = fastened
    return parts, extras, info


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
                rigid_with=prefixed(b.rigid_with, s), fab=b.fab, bom_key=b.bom_key, sheet=b.sheet,
            ))
        connections += [((i, prefixed(pb, s), pj), (k, prefixed(cb, s), cj))
                        for (i, pb, pj), (k, cb, cj) in side.connections]
    host = {s: prefixed(design.plan.topo.frame_bodies[0], s) for s in SIDES}
    parts, extras, info = chassis(side, design, z_mid, host)
    bodies += parts
    deck, deck_extras, info["deck"] = _deck(side, design, z_mid, host, bodies)
    if deck:
        bodies += deck
        extras += deck_extras
        info["fastened"] = list(info["fastened"]) + info["deck"].pop("fastened")
    meta = dict(side.meta, robot=True, mid_plane=z_mid, filament="pla_filament", **info)
    return Mechanism(
        name=f"{side.name}_robot", bodies=bodies, connections=connections, meta=meta,
        bom_extras=list(side.bom_extras) * 2 + extras,
    )
