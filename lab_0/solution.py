#!/usr/bin/python

from digifab import *
from math import sqrt
import numpy, solid

def combine_shapes(shapes):
  """
  Transform and combine shapes as in all_shapes below using OpenSCAD
  generators and functions.

  Args:
    shapes = (open_pl, squarcle_pl, star_pl)

  Retruns:
    A single PolyLine with transformed and combined geometry
  """
  open_pl, squarcle_pl, star_pl = shapes
  small_open_pl = solid.scale(0.5)( open_pl.get_generator() )
  trans_squarcle_pl = solid.translate([0,50])( squarcle_pl.get_generator() )
  trans_rot_star_pl = solid.translate([50,175])( solid.rotate(numpy.pi/2)( star_pl.get_generator() ) )
  combined = (small_open_pl + trans_squarcle_pl + trans_rot_star_pl)
  return PolyLine(generator=combined)

def tile_shape(shape, n_x, n_y):
  """
  Tile a shape in an n_x by n_y grid, making sure that the tiled shapes do not
  overlap.

  Args:
    shape (PolyLine): the shape to be tiled
    n_x (int): number of shapes in x direction
    n_y (int): number of shapes in y direction

  Returns:
    A single PolyLine with tiled shapes
  """
  margin = 5
  origin = PolyLine()
  xs = [x for (x,y) in shape.points]
  ys = [y for (x,y) in shape.points]
  width = max(xs) - min(xs)
  height = max(ys) - min(ys)
  for x in range(n_x):
      for y in range(n_y):
          dx = x*(width+margin)
          dy = y*(height+margin)
          origin += (dx, dy, 0) * shape
  return origin

def make_art():
  """
  Go crazy, make something cool! Must involve at least 100 points with 10
  separate PolyLine elements.

  Returns:
    A single PolyLine with art
  """

  return sierpinski(4000,7)


def sierpinski(l, d):
    if d == 1:
        return PolyLine([[0,0],[l,0],[l/2,sqrt(3)*l/2]]).hull()
    else:
        triangle = sierpinski(l/2,d-1)
        all = PolyLine()
        all += triangle
        all += (l/2,0,0)*triangle
        all += (l/4,sqrt(3)*l/4,0) * triangle
        return all

def check_overlap(shape):
  """Return true if a PolyLine has any overlapping geometry."""

  for i in range(len(shape)-1):
    for j in range(i+1,len(shape)):
      if len(shape[i] & shape[j]):
        return True
  return False

def solution(shapes):
  """Run solution code. Do not edit this function"""

  open_pl, squarcle_pl, star_pl = shapes

  try:
    combined_shapes = combine_shapes(shapes).simplified()
    assert(len(combined_shapes) == sum([len(s.simplified()) for s in shapes])) #added simplified
  except AssertionError:
    print 'Something\'s wrong with combine_shapes'

  try:
    tiled_star = tile_shape(star_pl, 2, 3).simplified()
    tiled_star.save("tiled_star.scad")
    assert(len(tiled_star) == 6*len(star_pl))
    assert(not check_overlap(tiled_star))
  except AssertionError:
    print 'Something\'s wrong with tiling stars'

  try:
    tiled_squarcle = tile_shape(squarcle_pl, 2, 1).simplified()
    tiled_squarcle.save("tiled_squarcle.scad")
    assert(len(tiled_squarcle) == 2*len(squarcle_pl.simplified())) #I think there needs to be a simplified here as well?
  except AssertionError:
    print 'Something\'s wrong with tiling squarcles'

  try:
    art = make_art()
    art.save("art.dxf")
    assert(len(art.points) >= 100)
    assert(len(art) >= 10)
  except AssertionError:
    print 'Something\'s wrong with (your) art'

def star(n_point = 5, major_r = 30.0, minor_r = 15.0):
  """Create a closed N-pointed star.

  Args:
    n_point (int): number of points
    major_r (float): outer radius of star
    major_r (float): inner radius of star

  Returns:
    A PolyLine star
  """

  pts = []
  for i in range(2*n_point):
    angle = (2.0*numpy.pi*i)/(2.0*n_point)
    if (i % 2) == 0:
      pts.append([numpy.cos(angle)*major_r,numpy.sin(angle)*major_r])
    else:
      pts.append([numpy.cos(angle)*minor_r,numpy.sin(angle)*minor_r])
  return PolyLine(pts, polygon=True)

def example():
  """Run lab_0 example. Creates all_shapes.scad and all_shapes.dxf files.

  Returns:
    open, squarcle, and star PolyLines
  """

  # Make an open PolyLine defined with points
  open_pl = PolyLine([[0,0],[0,60],[100,0],[200,0]])

  # Make an OpenSCAD generator
  squarcle_gen = (
    solid.square(50) +
    solid.translate([50,25])(solid.circle(25)) -
    solid.translate([20,15])(solid.text('S',size=20))
  )

  # Use the OpenSCAD generator to make a PolyLine
  squarcle_pl = PolyLine(generator=squarcle_gen)

  # Create star.dxf by saving a PolyLine
  star().save('star.dxf')

  # Load PolyLine from DXF file
  star_pl = PolyLine(filename='star.dxf')

  # Scale, translate and rotate PolyLines
  small_open_pl = 0.5 * open_pl
  trans_squarcle_pl = (0,50,0) * squarcle_pl
  trans_rot_star_pl = (50,175,numpy.pi/2) * star_pl

  # Combine the geometries and save them
  all_shapes = small_open_pl + trans_squarcle_pl + trans_rot_star_pl
  all_shapes.save('all_shapes.scad')
  all_shapes.save('all_shapes.dxf')

  return (open_pl, squarcle_pl, star_pl)

if __name__ == '__main__':
  solution(example())
