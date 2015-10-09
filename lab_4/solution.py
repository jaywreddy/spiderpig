#!/usr/bin/python

from digifab import *
import numpy as np


def snap_shaft():
    """Return snap shaft and hole geometry in a list of PolyMeshes."""

    s_pts = solid.polygon(
        points=[
            [0.0, 0.0], [4.95, 0.0], [4.95, 3.0], [2.45, 3.0], [1.95, 3.5],
            [1.95, 5.675], [2.45, 6.175], [2.45, 7.425], [1.95, 7.925],
            [0.75, 7.925], [0.75, 3.5], [0.0, 3.0], [0.0, 0.0]
        ]
    )

    sh_gen = (
        solid.rotate_extrude(convexity=10)(s_pts)
        - solid.translate([0, 0, 8.5])(solid.cube([1.5, 10, 10], center=True))
        - solid.translate([0, 0, 3.75])(
            solid.rotate([90, 0, 0])(
                solid.cylinder(r=0.75, h=10, center=True)
            )
        )
    )

    h_pts = solid.polygon(
        points=[
            [2.5, 0], [2, 0.5], [2.0, 2.675], [2.5, 3.175], [
                4.95, 3.175], [4.95, 0], [2.5, 0]
        ]
    )

    # added hole so it will subtract geometry from arm as necessary
    h_gen = solid.translate([0, 0, 3])(
        solid.rotate_extrude(convexity=10)(h_pts))

    return [PolyMesh(generator=sh_gen), PolyMesh(generator=h_gen)]


def add_snap_joint_pos(poly, joint):
    """
    Adds the knob side of the snap joint
    """
    transform = matrix_pose(joint.pose)
    s, h = snap_shaft()
    tf_knob = transform * s
    return poly + tf_knob


def add_snap_joint_neg(poly, joint):
    """
    Adds the hole side of the snap joint
    """
    transform = matrix_pose(joint.pose)
    s, h = snap_shaft()
    tf_hole = transform * h

    # manually remove cylinder center
    h_hole = PolyMesh(generator=solid.cylinder(r=3, h=8, segments=10))
    tf_snap_sub = transform * h_hole
    return (poly - tf_snap_sub) + tf_hole


def add_mortice(
        mortice_poly, mortice_joint, tenon_poly, tenon_joint,
        margin=0.5):
    """Add a mortice slot between two PolyMeshes.

    This algorithm will place mortice_poly and tenon_poly such that
    mortice_joint and tenon_joint are coincident, and then remove a slot along
    the positive Z-axis from mortice_poly so that the tenon_poly can slide into
    joint alignment. When aligned, the polys must have intersecting geometry.

    Args:
      mortice_poly (PolyMesh): PolyMesh to remove mortice from.
      mortice_joint (Joint): Joint on mortice_poly.
      tenon_poly (PolyMesh): PolyMesh that will slide into mortice_poly.
      tenon_joint (Joint): Joint on tenon_poly.
      margin (float): grow mortice slot outward by this margin.

    Returns:
      A modified version of mortice_poly with mortice slot removed.

    """

    mortice_inv = matrix_pose(pose_inverse(mortice_joint.pose))
    tenon_inv = matrix_pose(pose_inverse(tenon_joint.pose))

    # Transform mortice and tenon bodies so that the joints are at ORIGIN_POSE
    tf_mortice = mortice_inv * mortice_poly
    tf_tenon = tenon_inv * tenon_poly

    # Find intersecting volume at joint
    intersection = tf_mortice & tf_tenon

    # Project the outline to the XY plane and add margin
    mortice_outline = solid.minkowski()(
        solid.projection()(intersection.get_generator()),
        solid.circle(margin)
    )

    # Create a hole volume that goes all the way through the mortice body in the
    # positive Z direction
    max_z = mortice_poly.bounds()[1, 2] + margin
    mortice_volume = solid.linear_extrude(max_z)(mortice_outline)

    # Subtract the mortice volume from the mortice body, and return it in the
    # original orientation
    tf_mortice -= PolyMesh(generator=mortice_volume)

    return mortice_joint.pose * tf_mortice


def dummy_arm(name):
    """Simple placeholder arm.

    Args:
      name (str): Set Body name to this.

    Returns:
      A Body with a joint at ORIGIN_POSE, and a rectangular prism in blue layer 'print'

    """

    return Body(
        joints=[Joint(name='shoulder')],
        layers=Layer(
            (-5, -5, 0) * PolyMesh(generator=solid.cube([30, 10, 3])),
            name='print',
            color='blue'
        ),
        name=name
    )


def quat_from_zx(z_axis, x_axis):
    zn = numpy.array(z_axis)
    zn = zn / (zn.T.dot(zn)**0.5)
    xn = numpy.array(x_axis)
    xn = xn / (xn.T.dot(xn)**0.5)
    yn = numpy.cross(zn, xn)
    H = numpy.eye(4)
    H[0:3, 0:3] = numpy.vstack([xn, yn, zn]).T
    return quaternion_from_matrix(H)


class Robot(Mechanism):

    def __init__(self, **kwargs):

        if 'name' not in kwargs.keys():
            kwargs['name'] = 'robot'

        if 'elts' not in kwargs.keys():
            head = Body(
                joints=[
                    Joint(
                        ((20.0, 5.0, -10.0), Y_JOINT_POSE[1]),
                        name='neck'
                    )
                ],
                layers=Layer(
                    PolyMesh(filename='../lab_2/LOGOROBOT.stl'),
                    name='prefab',
                    color='yellow'
                ),
                name='head'
            )

            sq2_2 = (2.0**0.5) / 2.0
            neck_joint = Joint(
                ((21.0, 24.0, 15.0), NZ_JOINT_POSE[1]),
                name='neck'
            )
            torso_geom = PolyMesh(generator=solid.cube((42.0, 30.0, 20.0)))

            # Add mortice joinery between head and torso

            with_mortise = add_mortice(
                torso_geom, neck_joint, head[0][0], head.joints[0])

            # define shoulder joints
            right_shoulder_joint = Joint(
                ((2.0, 25.0, 10.0), quat_from_zx((-1, 0, 0), Z_AXIS)),
                name='right_shoulder'
            )
            left_shoulder_joint = Joint(
                ((40.0, 25.0, 10.0), quat_from_zx((1, 0, 0), Z_AXIS)),
                name='left_shoulder'
            )

            # add snap joints
            joined_torso = add_snap_joint_pos(
                with_mortise, right_shoulder_joint)
            final_torso = add_snap_joint_pos(joined_torso, left_shoulder_joint)

            torso = Body(
                joints=[
                    neck_joint,
                    right_shoulder_joint,
                    left_shoulder_joint
                ],
                layers=Layer(
                    final_torso,
                    name='print',
                    color='blue',
                ),
                name='torso'
            )
            kwargs['elts'] = [torso, head]
            kwargs['children'] = [Arm(name='right_arm'), Arm(name='left_arm')]

        if 'connections' not in kwargs.keys():
            kwargs['connections'] = [
                (('robot', 'torso', 'neck'), ('robot', 'head', 'neck')),
                (('robot', 'torso', 'right_shoulder'),
                    ('right_arm', 'upper_arm', 'shoulder')),
                (('robot', 'torso', 'left_shoulder'),
                    ('left_arm', 'upper_arm', 'shoulder'))
            ]

        super(Robot, self).__init__(**kwargs)


class Arm(Mechanism):

    ARM_COUNT = 0

    def __init__(self, **kwargs):

        if 'name' not in kwargs.keys():
            kwargs['name'] = 'arm_%d' % Arm.ARM_COUNT
            Arm.ARM_COUNT = Arm.ARM_COUNT + 1

        thickness = 3
        length = 40
        width = 8
        arm_geom = PolyMesh(generator=solid.cube((length, width, thickness)))
        offset = 6
        upper_arm_joint_pos = [offset, width / 2, 0]
        lower_arm_joint_pos = [offset, width / 2, -thickness]
        shoulder_joint_pos = [length - offset, width / 2, -thickness]
        OT = (0, 0, 0)
        OQ = (0, 0, 1, 0)
        OP = (OT, OQ)
        upper_arm_joint_pose = (upper_arm_joint_pos, OQ)
        lower_arm_joint_pose = (lower_arm_joint_pos, OQ)
        shoulder_joint_pose = (shoulder_joint_pos, OQ)
        upper_arm_joint = Joint(
            pose=upper_arm_joint_pose, name="upper_arm_joint")
        lower_arm_joint = Joint(
            pose=lower_arm_joint_pose, name="lower_arm_joint")
        shoulder_joint = Joint(pose=shoulder_joint_pose, name="shoulder")

        # add extra joinery
        joined_upper = add_snap_joint_pos(arm_geom, upper_arm_joint)
        double_joined_upper = add_snap_joint_neg(joined_upper, shoulder_joint)
        joined_lower = add_snap_joint_neg(arm_geom, lower_arm_joint)
        upper_arm_layer = Layer(
            double_joined_upper,
            color='blue'
        )
        lower_arm_layer = Layer(
            joined_lower,
            color='blue'
        )
        upper_arm = Body(pose=OP, elts=[upper_arm_layer],
                         joints=[upper_arm_joint, shoulder_joint], name="upper_arm")

        lower_arm = Body(pose=OP, elts=[lower_arm_layer],
                         joints=[lower_arm_joint], name="lower_arm")
        arm_rot = [2]
        arm_connection = ((0, 'upper_arm', 'upper_arm_joint'),
                          (0, 'lower_arm', 'lower_arm_joint'))
        kwargs['elts'] = [upper_arm, lower_arm]
        kwargs['connections'] = [arm_connection]
        kwargs['state'] = arm_rot
        super(Arm, self).__init__(**kwargs)


class Pantograph(Mechanism):

    def __init__(self, scale=2.0, **kwargs):

        if 'name' not in kwargs.keys():
            kwargs['name'] = 'pantograph'

        #choose ground_length
        ground_length = 20
        link1_length = (scale - 1)*ground_length

        #choose Trace length
        trace_length = 10
        link2_length = ground_length = trace_length
        arm_length = (ground_length + link1_length) - link2_length

        # create the 4 bars.
        width = 8
        thickness = 3
        offset = 4
        ground_geom = PolyMesh(generator=solid.cube(
            [ground_length + link1_length + 2 * offset, width, thickness]))
        trace_geom = PolyMesh(generator=solid.cube(
            [link2_length + trace_length + 2 * offset, width, thickness]))
        link_geom = PolyMesh(generator=solid.cube(
            [link1_length + 2 * offset, width, thickness]))
        arm_geom = PolyMesh(generator=solid.cube(
            [link2_length + arm_length + 2 * offset, width, thickness]))

        # create joints and poses
        OT = (0, 0, 0)
        OQ = (0, 0, 0, 1)
        OP = (OT, OQ)
        conns = []

        # add ground joint
        base_pos = [offset, width / 2, -thickness]
        base_joint = Joint(pose=(base_pos, OQ), name="base_joint")
        ground_geom = add_snap_joint_neg(ground_geom, base_joint)

        # add ground/trace connection
        ground_trace_pos = [offset + ground_length, width / 2, 0]
        ground_trace_joint = Joint(
            pose=(ground_trace_pos, OQ), name="ground_trace")
        ground_geom = add_snap_joint_pos(ground_geom, ground_trace_joint)

        trace_ground_pos = [offset, width / 2, -thickness]
        trace_ground_joint = Joint(
            pose=(trace_ground_pos, OQ), name="trace_ground")
        trace_geom = add_snap_joint_neg(trace_geom, trace_ground_joint)
        conns.append(((0, "ground", "ground_trace"),
                      (0, "trace", "trace_ground")))

        # add ground/arm joint
        ground_arm_pos = [offset + ground_length + link1_length, width / 2, 0]
        ground_arm_joint = Joint(pose=(ground_arm_pos, OQ), name="ground_arm")
        ground_geom = add_snap_joint_pos(ground_geom, ground_arm_joint)

        arm_ground_pos = [offset, width / 2, -thickness]
        arm_ground_joint = Joint(pose=(arm_ground_pos, OQ), name="arm_ground")
        arm_geom = add_snap_joint_neg(arm_geom, arm_ground_joint)
        conns.append(((0, "ground", "ground_arm"), (0, "arm", "arm_ground")))

        # add link/trace joint
        trace_link_pos = [offset + link2_length, width / 2, -thickness]
        trace_link_joint = Joint(pose=(trace_link_pos, OQ), name="trace_link")
        trace_geom = add_snap_joint_neg(trace_geom, trace_link_joint)

        link_trace_pos = [offset, width / 2, 0]
        link_trace_joint = Joint(pose=(link_trace_pos, OQ), name="link_trace")
        link_geom = add_snap_joint_pos(link_geom, link_trace_joint)
        conns.append(((0, "link", "link_trace"), (0, "trace", "trace_link")))

        # add link/arm joint
        link_arm_pos = [offset + link1_length, width / 2, 0]
        link_arm_joint = Joint(pose=(link_arm_pos, OQ), name="link_arm")
        link_geom = add_snap_joint_pos(link_geom, link_arm_joint)

        arm_link_pos = [offset + link2_length, width / 2, -thickness]
        arm_link_joint = Joint(pose=(arm_link_pos, OQ), name="arm_link")
        arm_geom = add_snap_joint_neg(arm_geom, arm_link_joint)
        conns.append(((0, "link", "link_arm"), (0, "arm", "arm_link")))

        # add trace and arm end joints
        arm_end_pos = [offset + link2_length +
                       arm_length, width / 2, -thickness]
        arm_end_joint = Joint(pose=(arm_end_pos, OQ), name="arm_end_joint")
        arm_geom = add_snap_joint_neg(arm_geom, arm_end_joint)

        trace_end_pos = [offset + link2_length +
                         trace_length, width / 2, -thickness]
        trace_end_joint = Joint(
            pose=(trace_end_pos, OQ), name="trace_end_joint")
        trace_geom = add_snap_joint_neg(trace_geom, trace_end_joint)

        ground = Body(name="ground",
                    joints=[ground_arm_joint, ground_trace_joint],
                    elts=[Layer(ground_geom, color="blue")])
        trace = Body(name="trace",
                    joints=[trace_ground_joint, trace_link_joint],
                    elts=[Layer(trace_geom, color="blue")])
        link = Body(name="link",
                    joints=[link_arm_joint, link_trace_joint],
                    elts=[Layer(link_geom, color="blue")])
        arm = Body(name="arm",
                    joints=[arm_ground_joint, arm_link_joint],
                    elts=[Layer(arm_geom, color="blue")])

        kwargs['elts'] = [ground, trace, link, arm]
        kwargs['connections'] = conns
        super(Pantograph, self).__init__(**kwargs)

def do_lab():
    p = Pantograph()
    p.show()

    r = Robot()
    r.show()
