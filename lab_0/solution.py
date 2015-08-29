#!/usr/bin/python

from digifab import *
from star import save_star

import solid, glob

def example():

  # Create Layout with one Block with 3 Layers
  lo = Layout(
    Block(
      [
        Layer(name='R', color='red'),
        Layer(name='G', color='green'),
        Layer(name='B', color='blue')
      ],
      name='main'
    )
  )
  
  # Add an open PolyLine defined with points to main block, red layer
  lo['main']['R'] += PolyLine([[0,0],[0,30],[50,0],[100,0]])

  # Add PolyLines using OpenSCAD generators to main block, green layer
  squarcle = solid.translate([0,75,0]) (
    solid.square(50) + solid.translate([50,25,0])(solid.circle(25))
  )
  lo['main']['G'] += PolyLine(generator=squarcle)
  
  # If the star.dxf file does not exist, create it by running save_star()
  if not glob.glob('star.dxf'):
    save_star()
    
  # Load Layout from DXF file
  star_lo = Layout(filename='star.dxf')
  
  # Get the first PolyLine from the first Layer from the first Block
  star_pl = star_lo[0][0][0]

  # Add the transformed polyline to the blue layer
  # 2D pose is (x,y,theta)
  lo['main']['B'] += (50,175,numpy.pi/2) ** star_pl

  # Save the layout in several 2D formats
  lo.save('lab_0.scad')
  lo.save('lab_0.dxf')

  return lo

if __name__ == '__main__':
  example()

