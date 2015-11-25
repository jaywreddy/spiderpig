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

def create_klann_geometry(orientation = 1, phase = 1):
    O = Point(0,0)
    mOA = 60
    OA = Point(0,-mOA)
    rotA = orientation*66.28*np.pi/180
    A = OA.rotate(rotA)

    OB = Point(0,1.121*mOA) #.611
    rotB = orientation*-41.76*np.pi/180 # -63.76
    B = OB.rotate(rotB)

    M_circ = Circle(O, .412*mOA)
    M = M_circ.arbitrary_point()
    st = {M.free_symbols.pop():phase}
    M = M.evalf(subs=st)
    C_circ1 = Circle(M,1.143*mOA)
    C_circ2 = Circle(A, .909*mOA)
    C_ints = custom_intersection(C_circ1,C_circ2)
    i= 0
    if orientation == -1:
        i = 1
    C = C_ints[i] #the circles have two intersections, check its the right value


    MC_length = M.distance(C)
    CD_length = .726*mOA
    Dx = C.x + ((C.x - M.x)/ MC_length)*CD_length
    Dy = C.y + ((C.y - M.y)/ MC_length)*CD_length
    D = Point(Dx, Dy)


    D_circ = Circle(D, .93*mOA)
    B_circ = Circle(B, .8*mOA) # 1.323
    E_ints = custom_intersection(B_circ, D_circ)

    E=E_ints[i] #same deal

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
    path_plot = plot_parametric(xf.evalf(),yf.evalf(), (xf.free_symbols.pop(),0.0,2*np.pi),show=False)
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
    if "conn" in name:
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

    attach_point1 = PolyMesh(generator =solid.translate([width, 0,0])(right_bar))
    attach_point2 = PolyMesh(generator =solid.translate([-width, 0,0])(left_bar))
    return pl, attach_point1, attach_point2

def create_shaft_connector():
    """
    Oriented relative to the shaft that will be driven at the origin in
    the +Z direction.

    TODO: Consider refactoring into mechanism with joint at O.
    """
    mount_plate = solid.cylinder(r = 10, h= 3)
    shaft = solid.translate([-2,-2,0])(solid.cube([4,4,20]))
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

def create_support_bar(jt, offset):
    """
    create a support bar to the point from the origin XY plane
    account for default klann spacing for now.

    plan on gluing two segments with acetone

    """
    ((dx,dy,_),_) = jt.pose
    base = solid.cylinder(r = 4, h = offset)
    clevis_pin = solid.cylinder(r=2, h = 3.5,segments=300)
    placed_pin = solid.translate([0,0,offset])(clevis_pin)
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
            mt, m_attach, _ = create_servo_mount()
                        # #add support for A,B
            mount_thickness= 3 # shaft coupler
            layer_thickness = 3
            #O, A,B, C, D, E, F,M, b1,b2,b3,b4,conn = create_klann_geometry()

            ##create the attacher thing, with joints
            A_bar, a_attach = create_support_bar(pos_A,mount_thickness)
            B_bar, b_attach = create_support_bar(pos_B,mount_thickness + layer_thickness)

            m_attach_gen = m_attach.get_generator()
            a_gen = a_attach.get_generator()
            b_gen = b_attach.get_generator()

            a_join = solid.hull()(m_attach_gen, a_gen)
            b_join = solid.hull()(m_attach_gen, b_gen)

            cronenberg = PolyMesh(generator= solid.union()(a_join,b_join))
            aparatus = cronenberg + A_bar + B_bar + mt

            shaft_conn = create_shaft_connector()

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

    def save_layouts(self):
        """Return layouts for making this sculpture.

        Returns:
          cut_layout: 2D Layout for laser cutting base
          print_layout: 3D Layout for 3D printing connectors
        """
        links = self.elts
        pls = []
        for link in links:
            if link.name == "coupler" or link.name == "torso":
                #save as stl
                link.elts[0].elts[0].save("%s.stl"%(link.name))
            else:
                #create 2D outline
                pm= link.elts[0].elts[0]
                base = pm.get_generator()
                pl = PolyLine(generator=solid.projection()(base) )
                pls.append(pl)
        save_layout(pls, "linkages")

def create_klann_part(orientation, phase, p= ""):
    #return 3 lists:
    #set of bodies, joints, connections
    O, A,B, C, D, E, F,M, b1,b2,b3,b4,conn = create_klann_geometry(orientation,phase)
    pos_A, neg_A = joint_from_point(A,p+"A",1)
    pos_B, neg_B = joint_from_point(B,p+"B",1)
    pos_C, neg_C = joint_from_point(C,p+"C",1)
    pos_D, neg_D = joint_from_point(D,p+"D",1)
    pos_E, neg_E = joint_from_point(E,p+"E",1)
    pos_F, neg_F = joint_from_point(F,p+"F",1)
    pos_M, neg_M = joint_from_point(M,p+"M",1)
    pos_O, neg_O = joint_from_point(O,p+"O",1)


    b1_body = rationalize_segment(b1,[neg_M, neg_C, neg_D],p+"b1")
    b2_body = rationalize_segment(b2,[neg_B, neg_E],p+"b2")
    b3_body = rationalize_segment(b3, [neg_A,pos_C],p+"b3")
    b4_body = rationalize_segment(b4, [pos_E,pos_D], p+"b4")
    conn = rationalize_segment(conn, [neg_O, pos_M], p+"conn" )


    ######
    # Connect bottom
    #######
    need_connectors = [(neg_C,b1_body), (neg_D,b1_body), (neg_E,b2_body), (neg_M,b1_body)]
    c_conns = []
    pins = []
    for anchor, bod in need_connectors:
        pin = connector()
        pin_base = pin.joints[0]
        p_conn = ((0,bod.name,anchor.name),(0,pin.name,"pin"))
        pins.append(pin)
        c_conns.append(p_conn)



    bodies = [b1_body, b2_body, b3_body, b4_body, conn] + pins
    bt_pos, bt_neg = joint_from_point(B,p+"B",2)
    joints = [pos_A, pos_O, bt_pos]
    conns = [
        ((0,p+'conn',p+'M'),(0,p+'b1', p+'M')),
        ((0,p+'b1',p+'C'),(0,p+'b3',p+'C')),
        ((0,p+'b1',p+'D'), (0,p+'b4',p+'D')),
        ((0,p+'b2',p+'E'),(0,p+'b4',p+'E')),
    ] + c_conns
    return bodies, joints, conns

def combine_connectors(conn1, conn2):
    pm1 = conn1.elts[0].elts[0]
    pm2 = conn2.elts[0].elts[0]
    joints = conn1.joints+conn2.joints
    pm = pm1+pm2
    OT = (0, 0, 0)
    OQ = (0, 0, 1, 0)
    OP = (OT, OQ)
    layers=Layer(
        pm,
        name="lol",
        color='green'
    )
    conn_body = Body(pose=OP, elts=[layers],
                     joints=joints, name="conn")
    return conn_body

class DoubleKlannLinkage(Mechanism):
    def __init__(self, **kwargs):
        if 'name' not in kwargs.keys():
            kwargs['name'] = 'klann'

        if 'elts' not in kwargs.keys():
            #create

            b1, j1, c1 = create_klann_part(1,1, "1")
            b2, j2, c2 = create_klann_part(-1, 1+ np.pi, "2")
            conn1 = b1.pop()
            conn2 = b2.pop()
            conn = combine_connectors(conn1,conn2)
            # print(conn.joints)
            A1,_,B1 = j1
            A2,_,B2 = j2
            link_bodies = b1+b2 +[conn]
            link_conns = c1+c2
            # #create servo mount that connects to shaft connector
            mt, m_attach1, m_attach2 = create_servo_mount()
                        # #add support for A,B
            mount_thickness= 3 # shaft coupler
            layer_thickness = 3

            ##create the attacher thing, with joints
            A1_bar, a1_attach = create_support_bar(A1,mount_thickness)
            B1_bar, b1_attach = create_support_bar(B1,mount_thickness + layer_thickness)

            m_attach1_gen = m_attach1.get_generator()
            a1_gen = a1_attach.get_generator()
            b1_gen = b1_attach.get_generator()

            a1_join = solid.hull()(m_attach1_gen, a1_gen)
            b1_join = solid.hull()(m_attach1_gen, b1_gen)

            cronenberg1 = PolyMesh(generator= solid.union()(a1_join,b1_join))

            A2_bar, a2_attach = create_support_bar(A2,mount_thickness)
            B2_bar, b2_attach = create_support_bar(B2,mount_thickness + layer_thickness)

            m_attach2_gen = m_attach2.get_generator()
            a2_gen = a2_attach.get_generator()
            b2_gen = b2_attach.get_generator()

            a2_join = solid.hull()(m_attach2_gen, a2_gen)
            b2_join = solid.hull()(m_attach2_gen, b2_gen)

            cronenberg2 = PolyMesh(generator= solid.union()(a2_join,b2_join))
            aparatus = cronenberg1 + A1_bar + B1_bar + mt + cronenberg2 + A2_bar + B2_bar

            shaft_conn = create_shaft_connector()

            OT = (0, 0, 0)
            OQ = (0, 0, 1, 0)
            OP = (OT, OQ)

            pos_O, neg_O = joint_from_point(Point(0,0), "1O", 1)

            torso = Body(
                pose = OP,
                joints = j1+j2,
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
            kwargs['elts'] = [conn,torso]+ link_bodies#[torso] , conn coupler,
            #kwargs['children'] = [torso]

        if 'connections' not in kwargs.keys():
            kwargs['connections'] = [
                #((0, 'conn', '1O'),(0,'coupler','1O')),
                ((0, 'torso', '1O'),(0, 'conn','1O')),
                ((0, 'torso', '2O'),(0, 'conn','2O')),
                ((0, 'torso', '1A'),(0, '1b3', '1A')),
                ((0, 'torso', '1B'),(0, '1b2', '1B')),
                ((0, 'torso', '2A'),(0, '2b3', '2A')),
                ((0, 'torso', '2B'),(0, '2b2', '2B'))
            ] + link_conns
        super(DoubleKlannLinkage, self).__init__(**kwargs)

    def save_layouts(self):
        """Return layouts for making this sculpture.

        Returns:
          cut_layout: 2D Layout for laser cutting base
          print_layout: 3D Layout for 3D printing connectors
        """
        links = self.elts
        pls = []
        for link in links:
            if link.name == "coupler" or link.name == "torso":
                #save as stl
                link.elts[0].elts[0].save("%s.stl"%(link.name))
            else:
                #create 2D outline
                pm= link.elts[0].elts[0]
                base = pm.get_generator()
                pl = PolyLine(generator=solid.projection()(base) )
                pls.append(pl)
        save_layout(pls, "linkages")


def voffset_joint(jt, offset, name):
    ((dx,dy,dz),quat) = jt.pose
    dz = dz + offest
    pos = Joint(
        ((dx, dy, dz + offset),Z_JOINT_POSE[1]),
        name=name
    )
    return joint


class DoubleDeckerKlannLinkage(Mechanism):
    def __init__(self, **kwargs):
        if 'name' not in kwargs.keys():
            kwargs['name'] = 'klann'

        if 'elts' not in kwargs.keys():
            #create

            b1, j1, c1 = create_klann_part(1,1, "1")
            b2, j2, c2 = create_klann_part(1, 1+ np.pi/2, "2")

            # print(conn.joints)
            A1,_,B1 = j1
            A2,_,B2 = j2
            link_bodies = b1+b2
            link_conns = c1+c2
            # # #create servo mount that connects to shaft connector
            mt, m_attach1, m_attach2 = create_servo_mount()
            #             # #add support for A,B
            mount_thickness= 6 # shaft coupler
            layer_thickness = 15
            #
            # ##create the attacher thing, with joints

            #first layer of standoffs
            st_body1,anch1 = standoff(mount_thickness) #contains joint "anchor"
            st_body2,anch2 = standoff(mount_thickness)
            st_body3,_ = standoff(layer_thickness) #contains joint "anchor"
            st_body4,_ = standoff(layer_thickness)
            st_conn1 = ((0,"1b3","1A"),(0,st_body1.name,"A"))
            st_conn2 = ((0,"1b2","1B"),(0,st_body2.name,"A"))

            #second layer:
            st_conn3 = ((0,"2b3", "2A"),(0,st_body3.name, "A"))
            st_conn4 = ((0,"2b2", "2B"),(0,st_body4.name, "A"))

            #A1_bar, a1_attach = create_support_bar(A1,mount_thickness)
            #B1_bar, b1_attach = create_support_bar(B1,mount_thickness + layer_thickness)
            #
            m_attach1_gen = m_attach1.get_generator()
            # b1_gen = b1_attach.get_generator()
            #
            ((dxa,dya,_),_) = A1.pose
            ((dxb,dyb,_),_) = B1.pose
            placed_anch1 = solid.translate([dxa,dya,0])(anch1)
            placed_anch2 = solid.translate([dxb,dyb,0])(anch2)

            a1_join = solid.hull()(m_attach1_gen, placed_anch1)
            b1_join = solid.hull()(m_attach1_gen, placed_anch2)
            #
            cronenberg1 = PolyMesh(generator= solid.union()(a1_join,b1_join))

            aparatus = cronenberg1 +mt

            shaft_conn = create_shaft_connector()

            OT = (0, 0, 0)
            OQ = (0, 0, 1, 0)
            OP = (OT, OQ)

            pos_O, neg_O = joint_from_point(Point(0,0), "1O", 1)
            O_3, _ = joint_from_point(Point(0,0),"3O",5)

            torso = Body(
                pose = OP,
                joints = j1+j2,
                elts=[Layer(
                    aparatus,
                    name='lol',
                    color='yellow',
                )],
                name='torso'
            )
            coupler = Body(
                 pose = OP,
                 joints = [pos_O,O_3],
                 elts = [Layer(shaft_conn, name='lol', color = 'blue')],
                 name = 'coupler'
             )
            kwargs['elts'] = link_bodies+ [coupler, st_body1, st_body2, st_body3, st_body4,torso]#[torso] [conn,torso]+ , [coupler]+ conn coupler,
            #kwargs['children'] = [torso]

        if 'connections' not in kwargs.keys():
            kwargs['connections'] = [
                ((0, '1conn', '1O'),(0,'coupler','1O')),
                ((0, 'coupler', '3O'),(0, '2conn','2O')),
                st_conn1,
                st_conn2,
                st_conn3,
                st_conn4,
                #((0, 'torso', '2O'),(0, 'conn','2O')),
                ((0, 'torso', '1A'),(0, '1b3', '1A')),
                ((0, 'torso', '1B'),(0, '1b2', '1B')),
                #((0, 'torso', '2A'),(0, '2b3', '2A')),
                #((0, 'torso', '2B'),(0, '2b2', '2B'))
            ] + link_conns
        super(DoubleDeckerKlannLinkage, self).__init__(**kwargs)

    def save_layouts(self):
        """
        Return layouts for making this sculpture.

        Returns:
          cut_layout: 2D Layout for laser cutting base
          print_layout: 3D Layout for 3D printing connectors
        """
        links = self.elts
        pls = []
        for link in links:
            if "coupler" in link.name or link.name == "torso" or "pin" in link.name or "":
                #save as stl
                link.elts[0].elts[0].save("%s.stl"%(link.name))
            else:
                #create 2D outline
                pm= link.elts[0].elts[0]
                base = pm.get_generator()
                pl = PolyLine(generator=solid.projection()(base) )
                pls.append(pl)
        save_layout(pls, "linkages")

#######
# Global bad....
#######
standnumber=0
pinnumber=0
def connector():
    """
    Connector 2.0!!! comes in two better sized segments, now with indent!

    """
    global pinnumber
    OT = (0, 0, 0)
    OQ = (0, 0, 1, 0)
    OP = (OT, OQ)
    base = solid.cylinder(r=4, h = 2, segments = 100)
    conn = solid.cylinder(r = 1.8, h = 6.4, segments = 100 )

    anchor = Joint(
            ((0, 0, 5.2),Z_JOINT_POSE[1]),
            name="pin"
        )
    bottom = base+ solid.translate([0,0,2])(conn)
    cap = solid.translate([0,0,2+6.25])(base)
    dimple_cap  = cap - conn

    p1 = PolyMesh(generator = bottom)
    p2 = PolyMesh(generator  = dimple_cap)
    # p1.save("pin.stl")
    # p2.save("cap.stl")
    poly = p1+p2
    pin = Body(
         pose = OP,
         joints = [anchor],
         elts = [Layer(poly, name='lol', color = 'blue')],
         name = 'coupler'+str(pinnumber)
    )
    pinnumber = pinnumber + 1
    return pin
def standoff(thickness):
    """
    Stackable, gluable 3D printable standoff connector w/pivot axis.
    """
    global standnumber
    OT = (0, 0, 0)
    OQ = (0, 0, 1, 0)
    OP = (OT, OQ)
    base = solid.cylinder(r=4, h=thickness, segments = 100)
    conn = solid.cylinder(r=1.8, h=4, segments = 100)
    #wtf??? can't get z axis on Joint to work, no matter what I put it just
    #defaults to 0, so I'm going to orient everything at the origin.
    anchor = Joint((0,0, -100),Z_JOINT_POSE[1],name="A")
    b_placed = solid.translate([0,0,-thickness])(base)
    unit = b_placed+ conn - solid.translate([0,0,-(5+ thickness)])(conn)
    pm = PolyMesh(generator = unit)
    stand = Body(
        pose = OP,
        joints = [anchor],
        elts = [Layer(pm, name="lol", color = 'yellow')],
        name = 'standoff'+str(standnumber)
    )
    standnumber = standnumber + 1
    return stand, b_placed

def make_stupid_connector():
    """
    Make "smarter" by having a recessed point in the cap for
    a glue hole, make more flush with bars.
    """
    base = solid.cylinder(r=4, h = 2, segments = 100)
    conn = solid.cylinder(r = 1.8, h = 6.8, segments = 100 )

    bottom = base+ solid.translate([0,0,2])(conn)

    p1 = PolyMesh(generator = bottom)
    p2 = PolyMesh(generator  = base)
    p1.save("pin.stl")
    p2.save("cap.stl")

#make_stupid_connector()


mech = DoubleDeckerKlannLinkage()
#mech.save("decker.scad")
mech.save_layouts()

# O, A,B, C, D, E, F,M, b1,b2,b3,b4,conn = create_klann_geometry()
# xf, yf = F.args
# t = xf.free_symbols.pop()
#
# path_plot = plot_parametric(xf,yf, (xf.free_symbols.pop(),0.0,2*np.pi))
