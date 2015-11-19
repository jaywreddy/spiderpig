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

def quat_from_zx(z_axis, x_axis):
    zn = numpy.array(z_axis)
    zn = zn / (zn.T.dot(zn)**0.5)
    xn = numpy.array(x_axis)
    xn = xn / (xn.T.dot(xn)**0.5)
    yn = numpy.cross(zn, xn)
    H = numpy.eye(4)
    H[0:3, 0:3] = numpy.vstack([xn, yn, zn]).T
    return quaternion_from_matrix(H)

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
    mOA = 60
    OA = Point(0,-mOA)
    rotA = 66.28*np.pi/180
    A = OA.rotate(rotA)

    OB = Point(0,.761*mOA)
    rotB = -52.76*np.pi/180
    B = OB.rotate(rotB)

    M_circ = Circle(O, .412*mOA)
    M = M_circ.arbitrary_point()
    #st = {M.free_symbols.pop():1}
    #M = M.evalf(subs=st)
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
    conn = Segment(O,M)

    items = [O,A,B, C, D, E, F,M, b1,b2,b3,b4,conn]
    return items

def plot_stuff():
    #evalf
    xf, yf = F.args
    t = xf.free_symbols.pop()
    state = {t:3.3}
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

def add_servo_mount(poly):
    """
    add shaft mounting hole
    """
    hole = solid.cube([4,4,50], center = True)
    pm = PolyMesh(generator = hole)
    return poly - pm

def joint_from_point(p,name, layer):
    x = float(p.x.evalf())
    y = float(p.y.evalf())
    z = 0
    thickness = 3

    pos = Joint(
        ((x, y, z + thickness*layer),Z_JOINT_POSE[1]),
        name=name
    )
    neg = Joint(
        ((x, y, z + thickness*(layer-1)), Z_JOINT_POSE[1]),
        name=name
    )
    return pos,neg

def clevis_neg(poly, joint):
    transform = matrix_pose(joint.pose)
    h_hole = PolyMesh(generator=solid.cylinder(r=2, h=20, segments=100,center =True))
    tf_snap_sub = transform * h_hole
    return poly - tf_snap_sub

def rationalize_segment(seg,joints,name, state= {}):
    p1 = seg.p1
    p2 = seg.p2
    buff = 6
    thickness = 3

    p1x = p1.x.evalf(subs=state)
    p1y = p1.y.evalf(subs=state)
    p2x = p2.x.evalf(subs=state)
    p2y = p2.y.evalf(subs=state)

    c = solid.cylinder(r= buff, h =thickness, segments =100)

    c1 = solid.translate([p1x, p1y, 0])(c)
    c2 = solid.translate([p2x, p2y, 0])(c)

    link = solid.hull()(c1,c2)

    OT = (0, 0, 0)
    OQ = (0, 0, 1, 0)
    OP = (OT, OQ)
    pm = PolyMesh(generator=link)

    for joint in joints:
        pm = clevis_neg(pm,joint)
    if name =="conn":
        #this is a connector joint
        pm = add_servo_mount(pm)
    layers=Layer(
        pm,
        name="lol",
        color='green'
    )
    link_body = Body(pose=OP, elts=[layers],
                     joints=joints, name=name)
    return link_body

def create_servo_mount():
    """
    right now designed to fit Jacobs institute model
    """
    width = 6.5
    length = 20.0
    depth = 2.3
    voffset = 18.5

    left_bar = solid.cube([width,length,depth], center = True)
    hole = solid.cylinder(r=2,h=10, center =True,segments = 100)
    hole1 = solid.translate([0,4,0])(hole)
    hole2 = solid.translate([0,-4,0])(hole)
    left_bar = solid.difference()(left_bar, hole1)
    left_bar = solid.difference()(left_bar, hole2)

    right_bar = solid.cube([width,length,depth],center = True)
    right_bar = solid.difference()(right_bar, hole1)
    right_bar = solid.difference()(right_bar, hole2)

    left_spread = -30.0
    right_spread = 17.0
    left_bar = solid.translate([left_spread, 0,-(depth/2 + voffset)])(left_bar)
    right_bar = solid.translate([right_spread, 0 , -(depth/2+voffset)])(right_bar)
    connector = solid.cube([(right_spread - left_spread) + width,width,depth],center=True)
    placed_connector = solid.translate([(left_spread+right_spread)/2,-(length/2 +width/2), -(depth/2+voffset)])(connector)
    total_mount = left_bar + placed_connector + right_bar
    pl = PolyMesh(generator= total_mount)

    attach_point = PolyMesh(generator =solid.translate([width, 0,0])(right_bar))
    return pl, attach_point

def create_shaft_connector():
    """
    Oriented relative to the shaft that will be driven at the origin in
    the +Z direction.

    TODO: Consider refactoring into mechanism with joint at O.
    """
    mount_plate = solid.cylinder(r = 10, h= 3)
    shaft = solid.translate([-2,-2,0])(solid.cube([4,4,15]))
    shifted_shaft = solid.translate([0,0,3])(shaft)
    total = mount_plate+shaft
    pl = PolyMesh(generator=total)
    return pl

def save_layout(pls, filename):
    """ Given a list of polylines, make a layout and save it to DXF files for laser cutting

    Args:
      pls ([PolyLine]): list of polylines
      filname (str): beginning of file name (sheet number and DXF extension will be added)

    """

    for pl in pls:
        pl.generator = None
    lo = Layout([Block(Layer(s, color='red')) for s in pls], size=(200, 200))

    sheets = lo.solved(5)

    for i in range(len(sheets)):
        sheets[i].save('%s_sheet_%d.dxf' % (filename, i), version='AC1021')

def create_support_bar(pt, offset):
    """
    create a support bar to the point from the origin XY plane
    account for default klann spacing for now.

    plan on gluing two segments with acentone

    """
    base = solid.cylinder(r = 4, h = offset)
    clevis_pin = solid.cylinder(r=2, h = 3.5,segments=300)
    placed_pin = solid.translate([0,0,offset])(clevis_pin)
    dx = pt.x
    dy = pt.y
    total = base + placed_pin
    translated = solid.translate([dx,dy,0])(total)
    pl = PolyMesh(generator=translated)
    attach = PolyMesh(generator = solid.translate([dx,dy,0])(base))
    return pl, attach


class KlannLinkage(Mechanism):
    ARM_COUNT = 0
    def __init__(self, **kwargs):
        if 'name' not in kwargs.keys():
            kwargs['name'] = 'klann'

        if 'elts' not in kwargs.keys():
            #create
            O, A,B, C, D, E, F,M, b1,b2,b3,b4,conn = create_klann_geometry()
            pos_A, neg_A = joint_from_point(A,"A",1)
            pos_B, neg_B = joint_from_point(B,"B",1)
            pos_C, neg_C = joint_from_point(C,"C",1)
            pos_D, neg_D = joint_from_point(D,"D",1)
            pos_E, neg_E = joint_from_point(E,"E",1)
            pos_F, neg_F = joint_from_point(F,"F",1)
            pos_M, neg_M = joint_from_point(M,"M",1)
            pos_O, neg_O = joint_from_point(O,"O",1)


            b1_body = rationalize_segment(b1,[neg_M, neg_C, neg_D],"b1")
            b2_body = rationalize_segment(b2,[neg_B, neg_E],"b2")
            b3_body = rationalize_segment(b3, [neg_A,pos_C],"b3")
            b4_body = rationalize_segment(b4, [pos_E,pos_D], "b4")
            conn = rationalize_segment(conn, [neg_O, pos_M], "conn" )

            # #create servo mount that connects to shaft connector
            mt, m_attach = create_servo_mount()
                        # #add support for A,B
            mount_thickness= 3 # shaft coupler
            layer_thickness = 3
            #O, A,B, C, D, E, F,M, b1,b2,b3,b4,conn = create_klann_geometry()

            ##create the attacher thing, with joints
            A_bar, a_attach = create_support_bar(A,mount_thickness)
            B_bar, b_attach = create_support_bar(B,mount_thickness + layer_thickness)

            m_attach_gen = m_attach.get_generator()
            a_gen = a_attach.get_generator()
            b_gen = b_attach.get_generator()

            a_join = solid.hull()(m_attach_gen, a_gen)
            b_join = solid.hull()(m_attach_gen, b_gen)

            cronenberg = PolyMesh(generator= solid.union()(a_join,b_join))
            aparatus = cronenberg + A_bar + B_bar + mt

            shaft_conn = create_shaft_connector()

            body_geom = PolyMesh(generator = solid.translate([-50,-50,-100])(solid.cube([100,100,100])))


            OT = (0, 0, 0)
            OQ = (0, 0, 1, 0)
            OP = (OT, OQ)
            bt_pos, bt_neg = joint_from_point(B,"B",2)
            torso = Body(
                pose = OP,
                joints=[
                    pos_A,
                    pos_O, bt_pos
                ],
                elts=[Layer(
                    aparatus,
                    name='lol',
                    color='yellow',
                )],
                name='torso'
            )
            coupler = Body(
                pose = OP,
                joints = [pos_O],
                elts = [Layer(shaft_conn, name='lol', color = 'blue')],
                name = 'coupler'
            )
            kwargs['elts'] = [coupler, b1_body, b2_body, b3_body, b4_body, conn,torso]#[torso]
            #kwargs['children'] = [torso]

        if 'connections' not in kwargs.keys():
            kwargs['connections'] = [
                ((0, 'torso', 'A'),(0, 'b3', 'A')),
                ((0, 'torso', 'B'),(0, 'b2', 'B')),
                ((0,'torso', 'O'),(0, 'conn', 'O')),
                ((0, 'conn','M'),(0,'b1', 'M')),
                ((0,'b1','C'),(0,'b3','C')),
                ((0,'b1','D'), (0,'b4','D')),
                ((0,'b2','E'),(0,'b4','E')),
                ((0,'conn', 'O'), (0,'coupler', 'O'))
            ]
        super(KlannLinkage, self).__init__(**kwargs)

    def get_layouts(self):
        """Return layouts for making this sculpture.

        Returns:
          cut_layout: 2D Layout for laser cutting base
          print_layout: 3D Layout for 3D printing connectors
        """
        links = self.children
        pls = []
        for link in links:
            pm= link.elts[0].elts[0]
            base = pm.get_generator()
            pl = PolyLine(generator=solid.projection()(base) )
            pls.append(pl)
        return pls

#mech = KlannLinkage()
#mech.save("linkage.scad")

O, A,B, C, D, E, F,M, b1,b2,b3,b4,conn = create_klann_geometry()
xf, yf = F.args
path_plot = plot_parametric(xf,yf, (xf.free_symbols.pop(),0.0,2*np.pi),show=False)
path_plot.show()
