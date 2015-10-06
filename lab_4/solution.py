#!/usr/bin/python

from digifab import *


def snap_shaft():
  """Return snap shaft and hole geometry in a list of PolyMeshes."""

  s_pts = solid.polygon(
    points=[
      [0.0,0.0],[4.95,0.0],[4.95,3.0],[2.45,3.0],[1.95,3.5],
      [1.95,5.675],[2.45,6.175],[2.45,7.425],[1.95,7.925],
      [0.75,7.925],[0.75,3.5],[0.0,3.0],[0.0,0.0]
    ]
  )
  
  sh_gen = (
    solid.rotate_extrude(convexity=10)(s_pts)
    - solid.translate([0,0,8.5])(solid.cube([1.5,10,10],center=True))
    - solid.translate([0,0,3.75])(
      solid.rotate([90,0,0])(
        solid.cylinder(r=0.75,h=10,center=True)
      )
    )
  )
  
  h_pts = solid.polygon(
    points = [
      [2.5,0],[2,0.5],[2.0,2.675],[2.5,3.175],[4.95,3.175],[4.95,0],[2.5,0]
    ]
  )
  
  h_gen = solid.translate([0,0,3])(solid.rotate_extrude(convexity=10)(h_pts))
  
  return [PolyMesh(generator=sh_gen),PolyMesh(generator=h_gen)]


def add_mortice(
    mortice_poly, mortice_joint, tenon_poly, tenon_joint,
    margin = 0.5):
  
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
  max_z = mortice_poly.bounds()[1,2] + margin
  mortice_volume = solid.linear_extrude(max_z)(mortice_outline)

  # Subtract the mortice volume from the mortice body, and return it in the
  # original orientation
  tf_mortice -= PolyMesh(generator = mortice_volume)

  return mortice_joint.pose * tf_mortice

def dummy_arm(name):
  """Simple placeholder arm.

  Args:
    name (str): Set Body name to this.  

  Returns:
    A Body with a joint at ORIGIN_POSE, and a rectangular prism in blue layer 'print'
  
  """
  
  return Body(
    joints = [Joint(name='shoulder')],
    layers = Layer(
      (-5,-5,0) * PolyMesh(generator=solid.cube([30,10,3])),
      name = 'print',
      color = 'blue'
    ),
    name = name
  )

def quat_from_zx(z_axis,x_axis):
  zn = numpy.array(z_axis)
  zn = zn/(zn.T.dot(zn)**0.5)
  xn = numpy.array(x_axis)
  xn = xn/(xn.T.dot(xn)**0.5)
  yn = numpy.cross(zn,xn)
  H = numpy.eye(4)
  H[0:3,0:3] = numpy.vstack([xn,yn,zn]).T
  return quaternion_from_matrix(H)
  
class Robot(Mechanism):
  def __init__(self, **kwargs):
    
    if 'name' not in kwargs.keys():
      kwargs['name'] = 'robot'
    
    if 'elts' not in kwargs.keys():
      
      head = Body(
        joints = [
          Joint(
            ((20.0,5.0,-10.0),Y_JOINT_POSE[1]),
            name = 'neck'
          )
        ],
        layers = Layer(
          PolyMesh(filename = '../lab_2/LOGOROBOT.stl'),
          name = 'prefab',
          color = 'yellow'
        ),
        name = 'head'
      )
     
      sq2_2 = (2.0**0.5)/2.0

      torso = Body(
        joints = [
          Joint(
            ((21.0,24.0,15.0),NZ_JOINT_POSE[1]),
            name = 'neck'
          ),
          Joint(
            ((4.0,25.0,10.0), quat_from_zx((-sq2_2,sq2_2,0.0),Z_AXIS)),
            name = 'right_shoulder'
          ),
          Joint(
            ((38.0,25.0,10.0), quat_from_zx((sq2_2,sq2_2,0.0),Z_AXIS)),
            name = 'left_shoulder'
          )
        ],
        layers = Layer(
          PolyMesh(generator = solid.cube((42.0,30.0,20.0))),
          name = 'print',
          color = 'blue',
        ),
        name = 'torso'
      )

      # Add mortice joinery between head and torso
      torso[0][0] = add_mortice(
        torso[0][0], torso.joints[0],
        head[0][0], head.joints[0]
      )

      kwargs['elts'] = [torso, head, dummy_arm('right_arm'), dummy_arm('left_arm')]

    if 'connections' not in kwargs.keys():
      kwargs['connections'] = [
        (('robot','torso','neck'),('robot','head','neck')),
        (('robot','torso','right_shoulder'),('robot','right_arm','shoulder')),
        (('robot','torso','left_shoulder'),('robot','left_arm','shoulder'))
      ]

    super(Robot, self).__init__(**kwargs)

class Arm(Mechanism):
  
  ARM_COUNT = 0
  
  def __init__(self, **kwargs):
    
    if 'name' not in kwargs.keys():
      kwargs['name'] = 'arm_%d' % Arm.ARM_COUNT
      Arm.ARM_COUNT = Arm.ARM_COUNT + 1

    super(Arm, self).__init__(**kwargs)

class Pantograph(Mechanism):
  def __init__(self, scale = 2.0, **kwargs):

    if 'name' not in kwargs.keys():
      kwargs['name'] = 'pantograph'

    super(Pantograph, self).__init__(**kwargs)

