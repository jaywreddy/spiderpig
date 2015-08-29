#!/usr/bin/python

from digifab import *
import numpy

def star_polyline(n_point = 5, major_r = 30.0, minor_r = 15.0):
  pts = []
  for i in range(2*n_point):
    angle = (2.0*numpy.pi*i)/(2.0*n_point)
    if (i % 2) == 0:
      pts.append([numpy.cos(angle)*major_r,numpy.sin(angle)*major_r])
    else:
      pts.append([numpy.cos(angle)*minor_r,numpy.sin(angle)*minor_r])
  return PolyLine(pts, polygon=True)
  
def save_star(n_point = 5, major_r = 30.0, minor_r = 15.0):
  star_polyline().save('star.dxf')

if __name__ == '__main__':
  save_star()
