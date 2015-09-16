#!/usr/bin/python

from digifab import *
import solid
import solid.utils
import math

def triangle(length=40.0, r = 3.175/2):
  """
  Generate list of points corresponding to triangular side of a tetrahedron.
  Also generate points for same trianglular side, offset to allow joinery in
  the tetrahedron.

  Args:
    Side length of tetrahedron, radius of wire to use

  Returns:
    Triangle points, Offset points
  """

  return tri_pts, offs_pts


def tetrahedron(length=40.0, r=3.175/2):
  """
  Simulate a tetrahedron made out of 4 bent wire triangles. To transform a
  list of points, create a rotation_matrix (defined in transformations.py),
  and use as follows:

  r = rotation_matrix(ang,direction, point)
  transformed_points = [compose([r,points.T])[0:3].T]

  Those transposes are necessary for the homogeneous transformations to work
  properly.

  Args:
    Side length of tetrahedron, radius of wire to use

  Returns:
    Solid Python object
  """
  t_pts,o_pts = triangle()

  return tet_mesh

def unfold_wire(pl):
  """
  Unfold a list of 3D points such that they are planar, while maintaining the
  open angle between the points.

  Args:
    List of points

  Returns:
    Unfolded list of points
  """

  return u_pl

def diwire_dfm(pl):
  """
  Check that a polyline is manufacturable by the DI-WIRE. No points closer
  than 12mm, no angle sharper than 135 degrees

  Args:
    PolyLine

  Returns:
    True or False
  """

  if valid:
    return True
  else:
    return False


def sim_wire(pl, r=3.175/2):
  """
  Simulate the output of the DI-WIRE given an ordered list of 3D coordinates
  corresponding to bend points. If you want to get really fancy, simulate the
  bend radius.

  Args:
    3D Point list

  Returns:
    SolidPython object
  """
  return wire_sim

if __name__ == '__main__':
  # Create equilateral triangle geometry
  tri = PolyLine(points=triangle()[1])

  # Check triangle for manufacturability
  assert(diwire_dfm(tri))

  # Create Layout and write to DXF file for use with DIWIRE
  lo = Layout(
    blocks = Block(
      layers =  Layer(name='bend', color='blue', geometries = tri)
    )
  )

  lo.save('lab_1.dxf')

  tet_mesh = PolyMesh(generator=tetrahedron()))
  
  tet_mesh.save('lab_1.scad')
  tet_mesh.save('lab_1.stl')



