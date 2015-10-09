#!/usr/bin/python

from digifab import *
import numpy

class SynthFourBar(Mechanism):
  def __init__(self, B = 70-40j, D = 110+90j, 
    P = (190+120j, 185+107j, 185+97j, 187+83j, 190+65j), signs=None, **kwargs):
    
    if 'name' not in kwargs.keys():
      kwargs['name'] = 'synth_four_bar'


    if 'children' not in kwargs.keys():
      solns = filter(is_consistent, synthesis(B, D, P, signs))
      
      # Remove 2R solutions
      solns = [s for s in solns if abs(s[0]-B) > 0.01]

      if not len(solns):
        raise Exception('No consistent solution found for synthesis')
      
      children = []
      constraints = []
      child_offset = 0.0
      soln_count = 0

      for A,_,C,_ in solns:
        vectors = [B-A,D-B,(C-D,P[0]-D),A-C]
        up_child = FourBar(vectors=vectors, name='soln_%d_up' % soln_count)
        #down_child = FourBar(vectors=vectors, name='soln_%d_down' % soln_count)
        
        offset = 2 * max(up_child.lengths) # + down_child.lengths)
        
        children.extend([up_child])#,down_child])
        
        up_pos = (child_offset + A.real, A.imag, 0.0)
        #down_pos = (child_offset + A.real, offset + A.imag, 0.0)
        #ori = quaternion_about_axis(numpy.angle(B-A),Z_AXIS)

        constraints.extend([
          ('body', (up_child.name,0,0), (up_pos,ORIGIN_POSE[1])),
          #('body', (down_child.name,0,0), (down_pos,ori))
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
    
    super(SynthFourBar, self).__init__(**kwargs)
    
  def plot(self, plotter, **kwargs):
    x,y = zip(*self.P)
    plotter.ax.plot(x,y,'k*')

    super(SynthFourBar, self).plot(plotter, **kwargs)

  def show(self, state=None, **kwargs):
    if type(state) is int:
      P0 = complex(*self.points[0])
      P = complex(*self.points[state])
      S = inv_kin_2R(complex(*self.B), complex(*self.D),P0,P)
    
    if type(state) is float:
      for child in self.children:
        child.state = [state] + 3*[numpy.nan]
    
    super(SynthFourBar, self).show(**kwargs)
