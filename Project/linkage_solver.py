#!/usr/local/bin/python
import numpy as np
import percache
import sympy
import math
from sympy.geometry import *
from sympy.plotting import plot_implicit, plot_parametric, plot
from sympy.simplify import simplify
from sympy.utilities.autowrap import ufuncify

from digifab import *
import solid

#rationalization
cache = percache.Cache("/tmp/my-cache13")

def create_servo_mount():
    """
    right now designed to fit Jacobs institute model
    """
    width = 6.5
    length = 20.0
    depth = 2.3
    voffset = -18.5 - 9

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

def create_support_bar(jt, offset):
    """
    create a support bar to the point from the origin XY plane
    account for default klann spacing for now.

    plan on gluing two segments with acetone

    """
    ((dx,dy,dz),_) = jt.pose
    placed_base = solid.translate([0,0,-offset])(solid.cylinder(r = 4, h = offset))
    clevis_pin = solid.translate([0,0,-(offset+5.5)])(solid.cylinder(r=1.9, h = 5.5,segments=300))
    total = placed_base + clevis_pin
    translated = solid.translate([dx,dy,dz])(total)
    pl = PolyMesh(generator=translated)
    attach = PolyMesh(generator = solid.translate([dx,dy,dz])(placed_base))
    return pl, attach

shaft_count = 0
def create_crank_shaft():
    """
    Oriented relative to the shaft that will be driven at the origin in
    the +Z direction.

    TODO: Consider refactoring into mechanism with joint at O.
    """
    global shaft_count
    name = "shaft_" + str(shaft_count)
    j_name = "shaft_joint_" + str(shaft_count)
    mount_plate = solid.cylinder(r = 10, h= 3)
    shaft = solid.cube([4/math.sqrt(2),4/math.sqrt(2),6], center = True)
    shifted_shaft = solid.translate([0,0,3])(shaft)
    total = mount_plate+shaft
    pl = PolyMesh(generator=total)

    j1 = Joint(
        ((0, 0, 0),Z_JOINT_POSE[1]),
        name=j_name
    )

    OT = (0, 0, 0)
    OQ = (0, 0, 1, 0)
    OP = (OT, OQ)

    layers=Layer(
        pl,
        name="lol",
        color='blue'
    )
    b = Body(pose=OP, elts=[layers],
                     joints=[j1], name=name)
    return b, name, j_name
offset_count = 0
def create_offset():
    """
    Oriented relative to the shaft that will be driven at the origin in
    the +Z direction.

    TODO: Consider refactoring into mechanism with joint at O.
    """
    global offset_count
    name = "offset_" + str(offset_count)
    offset_count +=1
    j_name = "offset_joint_" + str(offset_count)
    thickness = 6 #span 2 layers
    peg_height = 5.5
    clearance = 3.2
    base = solid.cylinder(r=4, h=thickness, segments = 100)
    conn = solid.cylinder(r=1.9, h=peg_height, segments = 100)
    #wtf??? can't get z axis on Joint to work, no matter what I put it just
    #defaults to 0, so I'm going to orient everything at the origin.
    b_placed = solid.translate([0,0,-thickness])(base)
    unit = b_placed+ conn - solid.translate([0,0,-(clearance+ thickness)])(conn)
    pm = PolyMesh(generator = unit)

    j1 = Joint(
        ((0, 0, 0),NZ_JOINT_POSE[1]),
        name=j_name
    )

    OT = (0, 0, 0)
    OQ = (0, 0, 1, 0)
    OP = (OT, OQ)

    layers=Layer(
        pm,
        name="lol",
        color='blue'
    )
    b = Body(pose=OP, elts=[layers],
                     joints=[j1], name=name)
    return b, name, j_name


locked_count = 0
def locked_offset():
    global locked_count
    name = "locked_" + str(locked_count)
    locked_count +=1
    j_name = "locked_joint_" + str(locked_count)
    thickness = 6 #span 2 layers
    peg_height = 3.2
    clearance = 3.2
    base = solid.cylinder(r=1.9, h=thickness, segments = 100)
    conn1 = solid.cube([4/math.sqrt(2),4/math.sqrt(2),3],center=True)
    conn2 = solid.cube([4/math.sqrt(2),4/math.sqrt(2),3],center=True)
    #wtf??? can't get z axis on Joint to work, no matter what I put it just
    #defaults to 0, so I'm going to orient everything at the origin.
    b_placed = solid.translate([0,0,3])(base)
    unit = b_placed + solid.translate([0,0,1.5])(conn1) + solid.translate([0,0,10.5])(conn2)
    pm = PolyMesh(generator = unit)
    # pm.save("lol.stl")
    j1 = Joint(
        ((0, 0, 12),NZ_JOINT_POSE[1]), #9
        name=j_name
    )

    OT = (0, 0, 0)
    OQ = (0, 0, 1, 0)
    OP = (OT, OQ)

    layers=Layer(
        pm,
        name="lol",
        color='blue'
    )
    b = Body(pose=OP, elts=[layers],
                     joints=[j1], name=name)
    return b, name, j_name


pinnumber=0
def connector():
    """
    Connector 2.0!!! comes in two better sized segments, now with indent!

    """
    global pinnumber
    OT = (0, 0, 0)
    OQ = (0, 0, 1, 0)
    OP = (OT, OQ)
    base = solid.cylinder(r=4, h = 4, segments = 100)
    conn = solid.cylinder(r = 1.9, h = 8, segments = 100 )

    #8.2
    anchor = Joint(
            ((0, 0, 8),Z_JOINT_POSE[1]),
            name="pin"
        )
    bottom = base+ solid.translate([0,0,4])(conn)
    cap = solid.translate([0,0,4+6.4])(base)
    dimple_cap  = cap - bottom

    p1 = PolyMesh(generator = bottom)
    p2 = PolyMesh(generator  = dimple_cap)
    #p1.save("pin.stl")
    #p2.save("cap.stl")
    poly = p1+p2
    pin = Body(
         pose = OP,
         joints = [anchor],
         elts = [Layer(poly, name='lol', color = 'blue')],
         name = 'coupler'+str(pinnumber)
    )
    pinnumber = pinnumber + 1
    return pin

def rationalize_linkage(linkage_dicts, layer_dict, lock_joints = [], anchor_pts = []):
    """
    linkage_dicts:[{seg:joint}]
    layer_dict:{seg:layer}
    """
    #invert linkage_dicts
    joint_dict = {}
    for d in linkage_dicts:
        s = list(d.keys())[0]
        js = list(d.values())[0]
        for j in js:
            if j in joint_dict:
                joint_dict[j]+=[s]
            else:
                joint_dict[j] = [s]
    seg_joints = {}
    joint_names = {}
    all_segs = list(layer_dict.keys())

    seg_names = {}
    segnum = 0
    for s in all_segs:
        seg_names[s] = "segment_"+str(segnum)
        segnum+=1

    for s in all_segs:
        seg_joints[s]=[]


    rats = []
    counter = 0
    conns = []
    for (pt, segs) in joint_dict.items():
        bottom_seg = min(segs, key = lambda x: layer_dict[x])
        top_seg = max(segs, key = lambda x: layer_dict[x])
        for (i,s1) in enumerate(segs):
            for s2 in segs[i+1:]:
                    l1,l2 = layer_dict[s1],layer_dict[s2]
                    if abs(l1-l2)==1:
                        name = "joint_" + str(counter)
                        joint_names[pt] = name
                        #create conns entry
                        conns += [((0,seg_names[s1],name),(0,seg_names[s2],name))]
                        counter +=1
                        j_pos, j_neg = joint_from_point(pt,name,1)
                        if l1>l2:
                            seg_joints[s1] += [j_neg]
                            seg_joints[s2] += [j_pos]
                        else:
                            seg_joints[s1] += [j_pos]
                            seg_joints[s2] += [j_neg]
                        #add joinery
                        if len(segs)==2:
                            pin = connector()
                            pin_base = pin.joints[0]
                            p_conn = ((0,seg_names[bottom_seg],name),(0,pin.name,"pin"))
                            rats += [pin]
                            conns += [p_conn]
            if pt in anchor_pts:
                name = "joint_" + str(counter)
                if name not in joint_names:
                  joint_names[pt] = name
                counter+=1
                j_pos, j_neg = joint_from_point(pt,name,1)
                seg_joints[s1] += [j_pos]
                if s1 in lock_joints:
                    #add a crank shaft
                    cs, p_name, j_name = create_crank_shaft()
                    conns += [((0, seg_names[s1], name),(0, p_name, j_name))]
                    rats +=[cs]
                else:
                    #add a standard offset
                    off, p_name, j_name = create_offset()
                    conns += [((0, seg_names[s1], name),(0, p_name, j_name))]
                    rats+=[off]

        if len(segs)>2:
            #this is a locked joint
            name = "joint_" + str(counter)
            print(name)
            counter+=1
            j_pos, j_neg = joint_from_point(pt,name,1)
            seg_joints[bottom_seg] += [j_neg]

            lock, l_name, j_name = locked_offset()
            l_conn = ((0,l_name, j_name),(0,seg_names[bottom_seg],name))
            conns += [l_conn]
            rats += [lock]

    #print(seg_joints)
    rats += [rationalize_segment(s,seg_joints[s],seg_names[s], is_locked = s in lock_joints) for s in all_segs]
    return rats, conns, seg_names, joint_names


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

def square_neg(poly,joint):
    transform = matrix_pose(joint.pose)
    w, l, h = 4/math.sqrt(2),4/math.sqrt(2),20
    h_hole = PolyMesh(generator=solid.cube([w,l,h],center =True))
    tf_snap_sub = transform * h_hole
    return poly - tf_snap_sub


def rationalize_segment(seg,joints,name, state= {}, is_locked = False):

    p1 = seg
    p2 = seg
    if type(seg) != Point:
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
        if is_locked:
            pm = square_neg(pm,joint)
        else:
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
    a = (c1.radius**2 - c2.radius**2 + d**2) / (2*d)

    x2 = c1.center.x + (dx * a/d)
    y2 = c1.center.y + (dy * a/d)

    h = (c1.radius**2 - a**2)**(.5)
    rx = -dy * (h/d)
    ry = dx * (h/d)

    xi_1 = simplify(x2 + rx)
    xi_2 = simplify(x2 - rx)
    yi_1 = simplify(y2 + ry)
    yi_2 = simplify(y2 - ry)

    ret = [Point(xi_1, yi_1)]
    if xi_1 != xi_2 or yi_1 != yi_2:
        ret.append(Point(xi_2, yi_2))
    return ret

def frange(start, stop, step):
     r = start
     while r < stop:
     	yield r
     	r += step
@cache
def create_klann_geometry(orientation = 1, phase = 1, evaluate= True):
    O = Point(0,0)
    mOA = 60
    OA = Point(0,-mOA)
    rotA = orientation*66.28*np.pi/180
    A = OA.rotate(rotA)

    OB = Point(0,1.121*mOA) #.611
    rotB = orientation*-41.76*np.pi/180 # -63.76
    B = simplify(OB.rotate(rotB))

    M_circ = Circle(O, .412*mOA)
    M = M_circ.arbitrary_point()
    if evaluate:
        st = {M.free_symbols.pop():phase}
        M = M.evalf(subs=st)
    C_circ1 = Circle(M,1.143*mOA)
    C_circ2 = Circle(A, .909*mOA)
    C_ints = custom_intersection(C_circ1,C_circ2)
    i= 0
    if orientation == -1:
        i = 1
    C = C_ints[i] #the circles have two intersections, check its the right value
    C = simplify(C)

    MC_length = M.distance(C)
    CD_length = .726*mOA
    Dx = C.x + ((C.x - M.x)/ MC_length)*CD_length
    Dy = C.y + ((C.y - M.y)/ MC_length)*CD_length
    D = Point(Dx, Dy)
    D = simplify(D)

    D_circ = Circle(D, .93*mOA) #.93
    B_circ = Circle(B, .8*mOA) # 1.323 #.8
    E_ints = custom_intersection(B_circ, D_circ)

    E=E_ints[i] #same deal
    E = simplify(E)

    ED_length = E.distance(D)
    DF_length = 2.577*mOA
    Fx = D.x + ((D.x - E.x)/ ED_length)*DF_length
    Fy = D.y + ((D.y - E.y)/ ED_length)*DF_length
    F = Point(Fx, Fy)
    F = simplify(F)

    b1 = Segment(M,D)
    b2 = Segment(B,E)
    b3 = Segment(A,C)
    b4 = Segment(E,F)
    conn = Segment(O,M)

    items = [O,A,B, C, D, E, F,M, b1,b2,b3,b4,conn]
    return items

@cache
def simulate_linkage_path(orient,default):
    a0 = 0
    a1 = 2*np.pi
    samps = 100
    Od,Ad,Bd, Cd, Dd, Ed, Fd,Md, b1d,b2d,b3d,b4d,connd = create_klann_geometry(orient, default)
    states = frange(a0,a1, float(a1-a0)/samps)
    seg_paths = {b1d:[],b2d:[],b3d:[],b4d:[],connd:[]}
    for st in states:
        O,A,B, C, D, E, F,M, b1,b2,b3,b4,conn = create_klann_geometry(orient, st)
        seg_paths[b1d] += list([p.evalf() for p in b1.points])
        seg_paths[b2d] += list([p.evalf() for p in b2.points])
        seg_paths[b3d] += list([p.evalf() for p in b3.points])
        seg_paths[b4d] += list([p.evalf() for p in b4.points])
    return seg_paths

@cache
def seg_conflicts(seg_paths):
    segs = list(seg_paths.keys())
    conflict_dict = {k:[] for (k,v) in seg_paths.items()}
    for (k,s1) in enumerate(segs):
        for s2 in segs[k+1:]:
            s1_pts = seg_paths[s1]
            s2_pts = seg_paths[s2]
            d1_hull = util.convex_hull(*s1_pts)
            d2_hull = util.convex_hull(*s2_pts)
            overlap = intersection(d1_hull, d2_hull)
            if len(overlap) > 0:
                conflict_dict[s1]+=[s2]
                conflict_dict[s2]+=[s1]
    return conflict_dict

def test_linearizer():
    O,A,B,C,D,E,F,M,b1,b2,b3,b4,conn = create_klann_geometry(1,1)
    b1_dict = {b1: [M,D,C]}
    b2_dict = {b2: [B,E]}
    b3_dict = {b3: [A,C]}
    b4_dict = {b4:[E,D,F]}
    conn_dict = {conn: [M,O]}

    O2,A2,B2,C2,D2,E2,F2,M2,b12,b22,b32,b42,conn2 = create_klann_geometry(-1,1)
    b1_dict2 = {b12: [M2,D2,C2]}
    b2_dict2 = {b22: [B2,E2]}
    b3_dict2 = {b32: [A2,C2]}
    b4_dict2 = {b42:[E2,D2,F2]}

    O3,A3,B3,C3,D3,E3,F3,M3,b13,b23,b33,b43,conn3 = create_klann_geometry(1,1 + np.pi)
    b1_dict3 = {b13: [M3,D3,C3]}
    b2_dict3 = {b23: [B3,E3]}
    b3_dict3 = {b33: [A3,C3]}
    b4_dict3 = {b43:[E3,D3,F3]}

    O4,A4,B4,C4,D4,E4,F4,M4,b14,b24,b34,b44,conn4 = create_klann_geometry(-1,1 + np.pi)
    b1_dict4 = {b14: [M4,D4,C4]}
    b2_dict4 = {b24: [B4,E4]}
    b3_dict4 = {b34: [A4,C4]}
    b4_dict4 = {b44:[E4,D4,F4]}
    conn_dict4 = {conn4:[M4,O4]}
    # seg_paths = simulate_linkage_path(1,1)
    # seg_paths2 = simulate_linkage_path(-1,1)
    # seg_paths3 = simulate_linkage_path(1,1+np.pi)
    # seg_paths4 = simulate_linkage_path(-1,1+np.pi)
    # seg_paths.update(seg_paths2)
    # seg_paths.update(seg_paths3)
    # seg_paths.update(seg_paths4)
    # print("generated Geometry")
    #
    # seg_confs = seg_conflicts(seg_paths)
    # print("calculated conflicts")
    #
    # #add gear conflicts and joints
    gear = Segment(M,M3)
    cap = Segment(M3,M3)
    # seg_confs[b1]+=[gear]
    # seg_confs[b12]+=[gear]
    # seg_confs[b13]+=[gear]
    # seg_confs[b14]+=[gear]
    # seg_confs[gear] = [b1,b12,b13,b14]
    gear_dict = {gear:[M,M3]}
    cap_dict = {cap:[M3]}
    #
    seg_dicts = [b1_dict, b2_dict, b3_dict, b4_dict]
    seg_dicts2 = [b1_dict2, b2_dict2, b3_dict2, b4_dict2]
    seg_dicts3 = [b1_dict3, b2_dict3, b3_dict3, b4_dict3]
    seg_dicts4 = [b1_dict4, b2_dict4, b3_dict4, b4_dict4]
    all_dicts = [gear_dict, conn_dict, cap_dict] + seg_dicts + seg_dicts2 + seg_dicts3 + seg_dicts4
    # print("added gear")
    #
    # neighbor_dicts = {list(seg.keys())[0]: [list(s.keys())[0] for s in all_dicts if shares_joint(s,seg)]
    #                             for seg in all_dicts}

    print("created neighbors")

    #optimal = create_layer_assignment(neighbor_dicts,seg_confs)
    optimal = {}
    optimal[gear] = 0
    optimal[b1] = 1
    optimal[b2] = 1
    optimal[b3] = 2
    optimal[b4] = 2

    optimal[b12] = 2
    optimal[b22] = 2
    optimal[b32] = 1
    optimal[b42] = 1

    optimal[b13] = -2
    optimal[b23] = -2
    optimal[b33] = -1
    optimal[b43] = -1

    optimal[b14] = -1
    optimal[b24] = -1
    optimal[b34] = -2
    optimal[b44] = -2

    optimal[conn] = 3
    optimal[cap] = -3

    lock_joints = [conn,conn2,conn3,conn4,gear, cap]
    anchor_pts = [A,B,O,A4,B4]
    ratts, conns, seg_names, joint_names = rationalize_linkage(all_dicts, optimal, lock_joints, anchor_pts)

    pos_O, neg_O = joint_from_point(Point(0,0), "1O", 1)
    #Create torso
    mt, m_attach1, m_attach2 = create_servo_mount()
                # #add support for A,B
    mount_thickness= 3 # shaft coupler
    layer_thickness = 3

    ##create the attacher thing, with joints
    A1_bar, a1_attach = create_support_bar(joint_from_point(A,"A",3)[0],mount_thickness)
    B1_bar, b1_attach = create_support_bar(joint_from_point(B,"B",2)[0],mount_thickness)

    m_attach1_gen = m_attach1.get_generator()
    a1_gen = a1_attach.get_generator()
    b1_gen = b1_attach.get_generator()

    a1_join = solid.hull()(m_attach1_gen, a1_gen)
    b1_join = solid.hull()(m_attach1_gen, b1_gen)

    cronenberg1 = PolyMesh(generator= solid.union()(a1_join,b1_join))

    A2_bar, a2_attach = create_support_bar(joint_from_point(A2,"A2",2)[0],mount_thickness)
    B2_bar, b2_attach = create_support_bar(joint_from_point(B2,"B2",3)[0],mount_thickness)

    m_attach2_gen = m_attach2.get_generator()
    a2_gen = a2_attach.get_generator()
    b2_gen = b2_attach.get_generator()

    a2_join = solid.hull()(m_attach2_gen, a2_gen)
    b2_join = solid.hull()(m_attach2_gen, b2_gen)

    cronenberg2 = PolyMesh(generator= solid.union()(a2_join,b2_join))
    aparatus = cronenberg1 + A1_bar + B1_bar + mt + cronenberg2 + A2_bar + B2_bar

    OT = (0, 0, 0)
    OQ = Z_JOINT_POSE[1]
    OP = (OT, OQ)


    torso = Body(
        pose = OP,
        joints = [pos_O],
        elts=[Layer(
            aparatus,
            name='lol',
            color='yellow',
        )],
        name='torso'
    )

    ratts += [torso]
    conns += [((0,'torso',"1O"),(0,seg_names[conn], joint_names[O]))]
    #print ratts
    #print conns
    return ratts, conns

class KlannLinkage(Mechanism):
    def __init__(self, **kwargs):
        ratts, conns = test_linearizer()
        if 'name' not in kwargs.keys():
            kwargs['name'] = 'klann'

        if 'elts' not in kwargs.keys():
            kwargs['elts'] = ratts

        if 'connections' not in kwargs.keys():
            kwargs['connections'] = conns
        super(KlannLinkage, self).__init__(**kwargs)

def shares_joint(seg_d1, seg_d2):
    """
    Returns whether or not the two segments share a joint in common
    False if they are the same segment.
    """
    if seg_d1 == seg_d2:
        return False
    for p1 in list(seg_d1.values())[0]:
        for p2 in list(seg_d2.values())[0]:
            if p1.distance(p2)<.2:
                return True
    return False

evals = 0
def evalp(point, state):
    global evals
    print("%d evals"%evals)
    evals = evals+1
    if len(point.free_symbols)!=0:
        st = {point.free_symbols.pop():state}
        try:
            evalled = point.evalf(subs=st)
            return evalled
        except ValueError:
            print("Evaluation failed, returning origin")
            return Point(0,0)

    return point.evalf()

@cache
def conflicts(seg_d1, seg_d2):
    """
    Returns whether or not the two segments conflict in their paths
    False if they are equal
    """
    if seg_d1 == seg_d2:
        return False

    seg1 = list(seg_d1.keys())[0]
    seg2 = list(seg_d2.keys())[0]
    #See if the convex hulls of the intersections intersect
    samps = 20
    limits = seg1.plot_interval()
    print(limits)
    var, a0,a1 = limits
    states = frange(a0,a1, float(a1-a0)/samps)
    p1,p2 = seg1.points
    p3,p4 = seg2.points
    p1_vals = [evalp(p1,s) for s in states]
    p2_vals = [evalp(p2,s) for s in states]
    p3_vals = [evalp(p3,s) for s in states]
    p4_vals = [evalp(p4,s) for s in states]
    d1_vals = p1_vals + p2_vals
    d2_vals = p1_vals + p2_vals
    d1_hull = util.convex_hull(*d1_vals)
    d2_hull = util.convex_hull(*d2_vals)
    overlap = intersection(d1_hull, d2_hull)
    return len(overlap) > 0
    #dist = d1_hull.distance(d2_hull) #both are convex, its chill
    #return dist > 6 #adjustable parameter
def count_layers(d):
    return len(set(d))

def is_valid(assigned_state, conflict_dicts):
    for n1 in list(assigned_state.keys()):
        st1 = assigned_state[n1]
        for n2 in conflict_dicts[n1]:
            if(n2 in assigned_state and st1==assigned_state[n2]):
                return False
    return True

def augment_states(states,double):
    possible_states = []
    for n in states:
        if double:
            possible_states += [n+2,n+1,n-1, n-2]
        else:
            possible_states += [n+1,n-1]
    return set(possible_states)

def create_layer_assignment(neighbor_dicts, conflict_dicts):
    start_node = list(neighbor_dicts.keys())[0] #choose random segment to start

    all_states = {} #populate state possibilities
    all_states[start_node] = [0]
    to_populate = neighbor_dicts[start_node]
    while len(to_populate) > 0:
        node = to_populate.pop(0)
        neighbors = neighbor_dicts[node]
        needs_state = [n for n in neighbors if n not in all_states]
        for n in needs_state:
            double = False
            if node == start_node:
                double = True
            all_states[n] = augment_states(states[node],double)
        to_populate += needs_state

    def explore(node, assigned_state, to_explore):
        neighbors = neighbor_dicts[node]
        to_explore += [n for n in neighbors if n not in assigned_state]
        all_solns = []
        for st in all_states[node]:
            assigned_state[node] = st
            assignment_check = is_valid(assigned_state, conflict_dicts)
            #bound_check = len(list(assigned_state.values())) < best_size
            if(assignment_check):
                solns = [explore(n,assigned_state,to_explore) for n in neighbors]
                all_solns += solns

        #filter out bad solns
        all_solns = [s for s in all_solns if s != 0]
        if len(all_solns) == 0:
            #no solutions found in this root
            return 0
        return min(all_solns, key = lambda x: len(list(x)))

    return explore(start_node,{},[])

def linearizer(seg_dicts):
    """
    Takes in a list of dictionaries of {Segment: [Point]}
    Assigns them to layers and does stuff
    """
    #First, construct neighbors.
    neighbor_dicts = {list(seg.keys())[0]: [list(s.keys())[0] for s in seg_dicts if shares_joint(s,seg)]
                                for seg in seg_dicts}
    #print(neighbor_dicts.values())
    print("Calculated Adjacencies")

    #conflicts(seg_dicts[0],seg_dicts[3])
    conflict_dicts = {list(seg.keys())[0]: [list(s.keys())[0] for s in seg_dicts if conflicts(s,seg)]
                                for seg in seg_dicts}

    print("calculated conflicts")
    optimal = create_layer_assignment(neighbor_dicts,conflict_dicts)
    print(optimal.values())
    return optimal
# locked_offset()[0].save("lock.stl")
A = KlannLinkage()
A.save("blah2.scad")
#simulate_linkage_path()
