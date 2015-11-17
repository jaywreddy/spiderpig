#!/usr/local/bin/python

#symbolic/numerical math
import numpy as np
import sympy
from sympy.geometry import *
from sympy.plotting import plot_implicit, plot_parametric, plot
from sympy.simplify import simplify
from sympy.utilities.autowrap import ufuncify
#rationalization
from digifab import *
import solid

def funcify_all(items, param):
    return [ufuncify([param],item) for item in items]

def eval_all(items, state):
    return [item.subs(state) for item in items]

def plot_geom(items):
    plots = []
    #create plots from implicit equations
    for item in items:
        if isinstance(item,Circle):
            cc = Circle(item.center.evalf(), item.radius)
            pl = plot_implicit(cc.equation(),show=False)
            plots.append(pl)
        elif isinstance(item,Segment):
            limits = item.plot_interval()
            var, a0,a1 = limits
            xeq,yeq = item.arbitrary_point().args

            #introduce numerical precision
            xeq = xeq.evalf()
            yeq = yeq.evalf()
            new_limits = (xeq.free_symbols.pop(),a0,a1)
            pl = plot_parametric(xeq,yeq,new_limits, show=False, line_color='r')
            plots.append(pl)
        elif isinstance(item,Line):
            pl = plot_implicit(item.equation().evalf(),show=False)
            plots.append(pl)
        elif isinstance(item,Point):
            pc = Circle(item.evalf(), .2)
            plots.append(plot_geom([pc]))
        else:
            raise TypeError("item does not have recognized geometry type")
    #combine plots
    p = plots.pop()
    for e in plots:
        p.extend(e)
    return p

def custom_intersection(c1,c2):
    #check if they are the same circle
    if c1.center == c2.center:
        if c2.radius == c1.radius:
            return c2
        return []

    dx, dy = (c2.center - c1.center).args
    d = (dy**2 + dx**2)**(.5)
    #R = x2.radius + c1.radius

    #check if they are out of range - unparameterized
    # if d > R or d < abs(self.radius - o.radius):
    #     return []
    #actually solve
    a = (c1.radius**2 - c2.radius**2 + d**2) / (2*d)

    x2 = c1.center.x + (dx * a/d)
    y2 = c1.center.y + (dy * a/d)

    h = (c1.radius**2 - a**2)**(.5)
    rx = -dy * (h/d)
    ry = dx * (h/d)

    xi_1 = x2 + rx
    xi_2 = x2 - rx
    yi_1 = y2 + ry
    yi_2 = y2 - ry

    ret = [Point(xi_1, yi_1)]
    if xi_1 != xi_2 or yi_1 != yi_2:
        ret.append(Point(xi_2, yi_2))
    return ret


def frange(start, stop, step):
     r = start
     while r < stop:
     	yield r
     	r += step



def create_klann_geometry():
    O = Point(0,0)
    mOA = 1
    OA = Point(0,-mOA)
    rotA = 66.28*np.pi/180
    A = OA.rotate(rotA)

    OB = Point(0,.611*mOA)
    rotB = -63.76*np.pi/180
    B = OB.rotate(rotB)

    M_circ = Circle(O, .412*mOA)
    M = M_circ.arbitrary_point()
    C_circ1 = Circle(M,1.143*mOA)
    C_circ2 = Circle(A, .909*mOA)
    C_ints = custom_intersection(C_circ1,C_circ2)
    C = C_ints[0] #the circles have two intersections, check its the right value

    MC_length = M.distance(C)
    CD_length = .726*mOA
    Dx = C.x + ((C.x - M.x)/ MC_length)*CD_length
    Dy = C.y + ((C.y - M.y)/ MC_length)*CD_length
    D = Point(Dx, Dy)

    D_circ = Circle(D, .93*mOA)
    B_circ = Circle(B, 1.323*mOA)
    E_ints = custom_intersection(B_circ, D_circ)

    E=E_ints[0] #same deal
    ED_length = E.distance(D)
    DF_length = 2.577*mOA
    Fx = D.x + ((D.x - E.x)/ ED_length)*DF_length
    Fy = D.y + ((D.y - E.y)/ ED_length)*DF_length
    F = Point(Fx, Fy)

    b1 = Segment(M,D)
    b2 = Segment(B,E)
    b3 = Segment(A,C)
    b4 = Segment(E,F)

    #evalf
    xf, yf = F.args
    t = xf.free_symbols.pop()
    state = {t:3.3}

    items = [A,B,M_circ, C_circ2, B_circ, F, b1,b2,b3,b4]
    #funcified = funcify_all(items, t)
    evaluated = eval_all(items, state)
    total_plot = plot_geom(evaluated)

    #evaluate mechanism at multiple points
    mechanism = [b1,b2,b3,b4]
    #func_mech = funcify_all(mechanism, t)
    steps = 7
    t_plot = total_plot
    for p in frange(0.0, 2*np.pi, 2*np.pi/steps):
        p_state = {t:p}
        evaluated = eval_all(mechanism,p_state)
        p_plot = plot_geom(evaluated)
        t_plot.extend(p_plot)

    #trace out foot path
    path_plot = plot_parametric(xf,yf, (xf.free_symbols.pop(),0.0,2*np.pi),show=False)
    t_plot.extend(path_plot)

    t_plot.show()
    total_plot.show()
    t_plot.save("klann2.png")

create_klann_geometry()

mech1 = items #decide which items I need and rationalize the shit out of them

#TODO: Create elaborate linkage function
