#!/usr/bin/python

from digifab import *
import numpy
import solid


###############
# Geom Utils
###############
def quat_from_zx(z_axis, x_axis):
    zn = numpy.array(z_axis)
    zn = zn / (zn.T.dot(zn)**0.5)
    xn = numpy.array(x_axis)
    xn = xn / (xn.T.dot(xn)**0.5)
    yn = numpy.cross(zn, xn)
    H = numpy.eye(4)
    H[0:3, 0:3] = numpy.vstack([xn, yn, zn]).T
    return quaternion_from_matrix(H)


def rationalize_fourbar(fourbar, thickness):
    rationalized = []
    for bar in fourbar.elts:
        trace, label = bar.elts
        thickened = solid.linear_extrude(thickness)(trace.get_generator())
        pl_thick = PolyMesh(generator = thickened)
        new_bar = Body(
            joints=bar.joints,
            layers=Layer(
                pl_thick,
                name=trace.name,
                color='blue'
            ),
            name=bar.name
        )
        rationalized.append(new_bar)
    rationalized_fourbar = fourbar.clone()
    rationalized_fourbar.elts = rationalized
    return rationalized_fourbar

def create_walker_leg(name):
    x1 = -131 + -112j
    x2 = -123 + -129j
    x3 = -87 + -132j
    x4 = -125 + -91j
    x5 = -104 + -95j
    p = (x1,x2,x3,x4,x5)
    b = 30 + -20j
    d = -20 + 30j
    bar = SynthFourBar(B = b, D = d, P = p)
    design = bar.children[0]
    design.name=name
    thickbar  = rationalize_fourbar(design, 3)
    return thickbar


    a = Walker()
    a.get_generator()
    a.solved()
    a.solved().get_generator()
    a.get_generator()
    ab.save_scad("test.scad")
    a.solved().children[1].get_generator()
    b = create_walker_leg("haha")
    b.solved().get_generator()
    c = b.solved()
    c.


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

class Walker(Mechanism):

    def __init__(self, **kwargs):

        if 'name' not in kwargs.keys():
            kwargs['name'] = 'walker'

        if 'elts' not in kwargs.keys():
            #Create body w/joints
            body_geom = PolyMesh(generator = solid.cube([100,100,100]) )

            # define shoulder joints
            right_shoulder_joint = Joint(
                ((0.0, 50.0, 50.0), quat_from_zx((-1, 0, 0), Z_AXIS)),
                name='right_shoulder'
            )
            left_shoulder_joint = Joint(
                ((100.0, 50.0, 50.0), quat_from_zx((1, 0, 0), Z_AXIS)),
                name='left_shoulder'
            )

            # add snap joints
            joined_torso = add_snap_joint_pos(
                body_geom, right_shoulder_joint)
            final_torso = add_snap_joint_pos(joined_torso, left_shoulder_joint)

            torso = Body(
                joints=[
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
            kwargs['elts'] = [torso]#[torso]
            right_leg = create_walker_leg('right_leg').solved()
            left_leg =  create_walker_leg('left_leg').solved()
            right_solved = PolyMesh(generator=right_leg.get_generator())
            left_solved = PolyMesh(generator=left_leg.get_generator())

            kwargs['children'] = [right_solved, left_solved]

        if 'connections' not in kwargs.keys():
            kwargs['connections'] = [
                (('walker', 'torso', 'right_shoulder'),
                    ('right_leg', 3, 1)),
                (('walker', 'torso', 'left_shoulder'),
                    ('left_leg', 3, 1))
            ]

        super(Walker, self).__init__(**kwargs)

class WalkerLeg(Mechanism):
    ARM_COUNT = 0

    def __init__(self, **kwargs):

        if 'name' not in kwargs.keys():
            kwargs['name'] = 'arm_%d' % Arm.ARM_COUNT
            Arm.ARM_COUNT = Arm.ARM_COUNT + 1

        leg_mechanism = create_walker_leg()
        anchor = leg_mechanism.joint((0,0))
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
        kwargs['elts'] = [design]
        super(WalkerLeg, self).__init__(**kwargs)

def make_walker():
    # define leg parameters
    x1 = -131 + -112j
    x2 = -123 + -129j
    x3 = -87 + -132j
    x4 = -125 + -91j
    x5 = -104 + -95j
    p = (x1,x2,x3,x4,x5)
    b = 30 + -20j
    d = -20 + 30j
    bar = SynthFourBar(B=b, D=d, P= p)

    #Elaborate Solids

    #Create body w/joints
    body_geom = PolyMesh(generator = solid.cube([10,10,10]) )

    # define shoulder joints
    right_shoulder_joint = Joint(
        ((0.0, 5.0, 5.0), quat_from_zx((-1, 0, 0), Z_AXIS)),
        name='right_shoulder'
    )
    left_shoulder_joint = Joint(
        ((10.0, 5.0, 5.0), quat_from_zx((1, 0, 0), Z_AXIS)),
        name='left_shoulder'
    )


class SynthFourBar(Mechanism):
  def __init__(self, B = -9.65+96.6j, D = -73.5-91.3j,
    P = (-100+44j, -110+21j, -114-1.5j, -113-23j, -110-41j), signs=(1,1,1,1,1),
    origins=None, **kwargs):

    """Show all solutions of a synthesis problem showing output points.

    Args:
      B,D,P : synthesis arguments, see fourbar_synthesis.py
      origins: list of positions to put generated mechanisms at. By default
        will be spaced apart to not overlap
    """

    if 'name' not in kwargs.keys():
      kwargs['name'] = 'synth_four_bar'


    if 'children' not in kwargs.keys():
      # Get synthesis solutions, and remove those that don't have A/Ac and C/Cc
      # as complex conjugates
      solns = filter(is_consistent, synthesis(B, D, P, signs))

      # Remove 2R solutions
      solns = [s for s in solns if abs(s[0]-B) > 0.01]

      if not len(solns):
        raise Exception('No consistent solution found for synthesis')

      children = []
      constraints = []
      child_offset = 0.0
      soln_count = 0
      origins = []

      for A,_,C,_ in solns:
        # Create an elbow up and elbow down mechanism for each synthesis solution
        vectors = [B-A,D-B,(C-D,P[0]-D),A-C]
        up_child = FourBar(
          vectors=vectors, elbow=0, name='soln_%d_up' % soln_count,
        )

        down_child = up_child.clone(
          elbow=1, name='soln_%d_down' % soln_count,
        )

        # space children out by twice the maximum link length to guarantee no
        # overlapping
        offset = 2 * max(up_child.lengths)
        up_pos = (child_offset + A.real, A.imag, 0.0)
        down_pos = (child_offset + A.real, offset + A.imag, 0.0)

        up_child.constraints = [('body',(0,0,0),(up_pos, ORIGIN_POSE[1]))]
        down_child.constraints = [('body',(0,0,0),(down_pos, ORIGIN_POSE[1]))]

        origins.extend([(child_offset, 0.0),(child_offset,offset)])

        children.extend([up_child,down_child])

        constraints.extend([
          ('body', (up_child.name,0,0), (up_pos,ORIGIN_POSE[1])),
          ('body', (down_child.name,0,0), (down_pos,ORIGIN_POSE[1]))
        ])

        soln_count += 1
        child_offset += offset

      kwargs['children'] = children
      kwargs['constraints'] = constraints

    if type(B) is complex:
      self.B = (B.real, B.imag)
    else:
      self.B = B

    if type(D) is complex:
      self.D = (D.real, D.imag)
    else:
      self.D = D

    if type(P[0]) is complex:
      self.P = [(p.real,p.imag) for p in P]
    else:
      self.P = P

    self.origins = origins
    self.signs = signs

    super(SynthFourBar, self).__init__(**kwargs)

  def plot(self, plotter, **kwargs):
    x,y = zip(*self.P)
    x,y = numpy.array(x), numpy.array(y)
    for x_o, y_o in self.origins:
      plotter.ax.plot(x+x_o,y+y_o,'k*')

    super(SynthFourBar, self).plot(plotter, **kwargs)

  def synth_angle(self, synth_idx, child_idx):
    """Given a synthesis point index, return the angle that should be between
      body 0 and body 1

    Args:
      synth_idx (int): index into P from synthesis problem
      child_idx (int): which of self.children to get angle for
    """

    P = [complex(*pi) for pi in self.P]
    S,T = inv_kin_2R(complex(*self.B), complex(*self.D), P[0], P[synth_idx])[self.signs[synth_idx]]
    return self.children[child_idx].init_angle + numpy.angle(S)

  def show(self, state=None, **kwargs):
    """Show collection of synthesized mechanisms

    Args:
      state (int|float): if int, use synth_angle to assign mechanism to pose
        that reaches output point P[state]. If float, assign all children
        mechanism
    """

    if type(state) is int:
      for i in range(len(self.children)):
        self.children[i].state[0] = self.synth_angle(state,i)

    elif type(state) is float:
      for child in self.children:
        child.state[0] = state

    super(SynthFourBar, self).show(**kwargs)
