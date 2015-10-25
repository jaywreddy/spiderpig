#!/usr/bin/python

from digifab import *
import numpy

def test_synth():
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

    bar.show()

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
