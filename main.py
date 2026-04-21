"""Klann walking-linkage mechanism generator.

This module wires a single :class:`KlannLinkage` and writes its assembled form
to ``single.scad``. The port to build123d + pytransform3d (see plan) is staged
across subsequent phases; this phase only strips dead code.
"""

import numpy as np
from sympy.geometry import Circle, Point, Segment

import solid
from digifab import (
    Body,
    Joint,
    Layer,
    Mechanism,
    PolyMesh,
    Z_JOINT_POSE,
    matrix_pose,
)


def custom_intersection(c1, c2):
    if c1.center == c2.center:
        if c2.radius == c1.radius:
            return c2
        return []

    dx, dy = (c2.center - c1.center).args
    d = (dy**2 + dx**2) ** 0.5
    a = (c1.radius**2 - c2.radius**2 + d**2) / (2 * d)

    x2 = c1.center.x + (dx * a / d)
    y2 = c1.center.y + (dy * a / d)

    h = (c1.radius**2 - a**2) ** 0.5
    rx = -dy * (h / d)
    ry = dx * (h / d)

    xi_1 = x2 + rx
    xi_2 = x2 - rx
    yi_1 = y2 + ry
    yi_2 = y2 - ry

    ret = [Point(xi_1, yi_1)]
    if xi_1 != xi_2 or yi_1 != yi_2:
        ret.append(Point(xi_2, yi_2))
    return ret


def create_klann_geometry(orientation=1, phase=1):
    O = Point(0, 0)
    mOA = 60
    OA = Point(0, -mOA)
    rotA = orientation * 66.28 * np.pi / 180
    A = OA.rotate(rotA)

    OB = Point(0, 1.121 * mOA)
    rotB = orientation * -41.76 * np.pi / 180
    B = OB.rotate(rotB)

    M_circ = Circle(O, 0.412 * mOA)
    M = M_circ.arbitrary_point()
    st = {M.free_symbols.pop(): phase}
    M = M.evalf(subs=st)
    C_circ1 = Circle(M, 1.143 * mOA)
    C_circ2 = Circle(A, 0.909 * mOA)
    C_ints = custom_intersection(C_circ1, C_circ2)
    i = 0
    if orientation == -1:
        i = 1
    C = C_ints[i]

    MC_length = M.distance(C)
    CD_length = 0.726 * mOA
    Dx = C.x + ((C.x - M.x) / MC_length) * CD_length
    Dy = C.y + ((C.y - M.y) / MC_length) * CD_length
    D = Point(Dx, Dy)

    D_circ = Circle(D, 0.93 * mOA)
    B_circ = Circle(B, 0.8 * mOA)
    E_ints = custom_intersection(B_circ, D_circ)
    E = E_ints[i]

    ED_length = E.distance(D)
    DF_length = 2.577 * mOA
    Fx = D.x + ((D.x - E.x) / ED_length) * DF_length
    Fy = D.y + ((D.y - E.y) / ED_length) * DF_length
    F = Point(Fx, Fy)

    b1 = Segment(M, D)
    b2 = Segment(B, E)
    b3 = Segment(A, C)
    b4 = Segment(E, F)
    conn = Segment(O, M)

    return [O, A, B, C, D, E, F, M, b1, b2, b3, b4, conn]


def add_servo_mount(poly):
    hole = solid.cube([4, 4, 50], center=True)
    pm = PolyMesh(generator=hole)
    return poly - pm


def joint_from_point(p, name, layer):
    x = float(p.x.evalf())
    y = float(p.y.evalf())
    z = 0
    thickness = 3
    pos = Joint(((x, y, z + thickness * layer), Z_JOINT_POSE[1]), name=name)
    neg = Joint(((x, y, z + thickness * (layer - 1)), Z_JOINT_POSE[1]), name=name)
    return pos, neg


def clevis_neg(poly, joint):
    transform = matrix_pose(joint.pose)
    h_hole = PolyMesh(generator=solid.cylinder(r=2, h=20, segments=100, center=True))
    tf_snap_sub = transform * h_hole
    return poly - tf_snap_sub


def rationalize_segment(seg, joints, name, state=None):
    state = state or {}
    p1 = seg.p1
    p2 = seg.p2
    buff = 6
    thickness = 3

    p1x = p1.x.evalf(subs=state)
    p1y = p1.y.evalf(subs=state)
    p2x = p2.x.evalf(subs=state)
    p2y = p2.y.evalf(subs=state)

    c = solid.cylinder(r=buff, h=thickness, segments=100)
    c1 = solid.translate([p1x, p1y, 0])(c)
    c2 = solid.translate([p2x, p2y, 0])(c)
    link = solid.hull()(c1, c2)

    OP = ((0, 0, 0), (0, 0, 1, 0))
    pm = PolyMesh(generator=link)
    for joint in joints:
        pm = clevis_neg(pm, joint)
    if "conn" in name:
        pm = add_servo_mount(pm)

    layers = Layer(pm, name="lol", color="green")
    return Body(pose=OP, elts=[layers], joints=joints, name=name)


def create_servo_mount():
    """Servo mount sized for the Jacobs Institute model."""
    width = 6.5
    length = 20.0
    depth = 2.3
    voffset = 18.5

    left_bar = solid.cube([width, length, depth], center=True)
    hole = solid.cylinder(r=2, h=10, center=True, segments=100)
    hole1 = solid.translate([0, 4, 0])(hole)
    hole2 = solid.translate([0, -4, 0])(hole)
    left_bar = solid.difference()(left_bar, hole1)
    left_bar = solid.difference()(left_bar, hole2)

    right_bar = solid.cube([width, length, depth], center=True)
    right_bar = solid.difference()(right_bar, hole1)
    right_bar = solid.difference()(right_bar, hole2)

    left_spread = -30.0
    right_spread = 17.0
    left_bar = solid.translate([left_spread, 0, -(depth / 2 + voffset)])(left_bar)
    right_bar = solid.translate([right_spread, 0, -(depth / 2 + voffset)])(right_bar)
    connector_bar = solid.cube(
        [(right_spread - left_spread) + width, width, depth], center=True
    )
    placed_connector = solid.translate(
        [(left_spread + right_spread) / 2, -(length / 2 + width / 2), -(depth / 2 + voffset)]
    )(connector_bar)
    total_mount = left_bar + placed_connector + right_bar

    pl = PolyMesh(generator=total_mount)
    attach_point1 = PolyMesh(generator=solid.translate([width, 0, 0])(right_bar))
    attach_point2 = PolyMesh(generator=solid.translate([-width, 0, 0])(left_bar))
    return pl, attach_point1, attach_point2


def create_shaft_connector():
    """Shaft coupler oriented along +Z at the origin."""
    mount_plate = solid.cylinder(r=10, h=3)
    shaft = solid.translate([-2, -2, 0])(solid.cube([4, 4, 20]))
    total = mount_plate + shaft
    return PolyMesh(generator=total)


def create_support_bar(jt, offset):
    """Cylindrical standoff from the XY plane down to a joint, ending in a clevis pin."""
    ((dx, dy, dz), _) = jt.pose
    placed_base = solid.translate([0, 0, -offset])(solid.cylinder(r=4, h=offset))
    clevis_pin = solid.translate([0, 0, -(offset + 2.5)])(
        solid.cylinder(r=1.9, h=5.5, segments=300)
    )
    total = placed_base + clevis_pin
    translated = solid.translate([dx, dy, dz])(total)
    pl = PolyMesh(generator=translated)
    attach = PolyMesh(generator=solid.translate([dx, dy, dz])(placed_base))
    return pl, attach


# Module-level counters used by connector() and standoff() for unique body names.
# These are parameterized out in the build123d port (phase 5).
pinnumber = 0
standnumber = 0


def connector():
    """3D-printed pin + cap clevis joiner."""
    global pinnumber
    OP = ((0, 0, 0), (0, 0, 1, 0))
    base = solid.cylinder(r=4, h=4, segments=100)
    conn = solid.cylinder(r=1.9, h=8, segments=100)

    anchor = Joint(((0, 0, 5.2), Z_JOINT_POSE[1]), name="pin")
    bottom = base + solid.translate([0, 0, 4])(conn)
    cap = solid.translate([0, 0, 4 + 6.4])(base)
    dimple_cap = cap - bottom

    p1 = PolyMesh(generator=bottom)
    p2 = PolyMesh(generator=dimple_cap)
    poly = p1 + p2
    pin = Body(
        pose=OP,
        joints=[anchor],
        elts=[Layer(poly, name="lol", color="blue")],
        name="coupler" + str(pinnumber),
    )
    pinnumber += 1
    return pin


def standoff(thickness):
    """Stackable 3D-printable standoff with a pivot axis."""
    global standnumber
    OP = ((0, 0, 0), (0, 0, 1, 0))
    peg_height = 5.5
    clearance = 3.2
    base = solid.cylinder(r=4, h=thickness, segments=100)
    conn = solid.cylinder(r=1.9, h=peg_height, segments=100)
    anchor = Joint((0, 0, -100), Z_JOINT_POSE[1], name="A")
    b_placed = solid.translate([0, 0, -thickness])(base)
    unit = b_placed + conn - solid.translate([0, 0, -(clearance + thickness)])(conn)
    pm = PolyMesh(generator=unit)
    stand = Body(
        pose=OP,
        joints=[anchor],
        elts=[Layer(pm, name="lol", color="yellow")],
        name="standoff" + str(standnumber),
    )
    standnumber += 1
    return stand, b_placed


class KlannLinkage(Mechanism):
    def __init__(self, **kwargs):
        if "name" not in kwargs:
            kwargs["name"] = "klann"

        if "elts" not in kwargs:
            O, A, B, C, D, E, F, M, b1, b2, b3, b4, conn = create_klann_geometry()
            pos_A, neg_A = joint_from_point(A, "A", 1)
            pos_B, neg_B = joint_from_point(B, "B", 1)
            pos_C, neg_C = joint_from_point(C, "C", 1)
            pos_D, neg_D = joint_from_point(D, "D", 1)
            pos_E, neg_E = joint_from_point(E, "E", 1)
            pos_M, neg_M = joint_from_point(M, "M", 1)
            pos_O, neg_O = joint_from_point(O, "O", 1)

            b1_body = rationalize_segment(b1, [neg_M, neg_C, neg_D], "b1")
            b2_body = rationalize_segment(b2, [neg_B, neg_E], "b2")
            b3_body = rationalize_segment(b3, [neg_A, pos_C], "b3")
            b4_body = rationalize_segment(b4, [pos_E, pos_D], "b4")
            conn = rationalize_segment(conn, [neg_O, pos_M], "conn")

            _, j, _ = create_klann_part(1, 1, "1")
            A, _, B = j
            mt, m_attach, _ = create_servo_mount()
            mount_thickness = 3
            layer_thickness = 3

            A_bar, a_attach = create_support_bar(A, mount_thickness)
            B_bar, b_attach = create_support_bar(B, mount_thickness + layer_thickness)

            m_attach_gen = m_attach.get_generator()
            a_gen = a_attach.get_generator()
            b_gen = b_attach.get_generator()

            a_join = solid.hull()(m_attach_gen, a_gen)
            b_join = solid.hull()(m_attach_gen, b_gen)

            cronenberg = PolyMesh(generator=solid.union()(a_join, b_join))
            aparatus = cronenberg + A_bar + B_bar + mt

            shaft_conn = create_shaft_connector()

            OP = ((0, 0, 0), (0, 0, 1, 0))
            bt_pos, _ = joint_from_point(B, "B", 2)
            torso = Body(
                pose=OP,
                joints=[pos_A, pos_O, bt_pos],
                elts=[Layer(aparatus, name="lol", color="yellow")],
                name="torso",
            )
            coupler = Body(
                pose=OP,
                joints=[pos_O],
                elts=[Layer(shaft_conn, name="lol", color="blue")],
                name="coupler",
            )
            kwargs["elts"] = [coupler, b1_body, b2_body, b3_body, b4_body, conn, torso]

        if "connections" not in kwargs:
            kwargs["connections"] = [
                ((0, "torso", "A"), (0, "b3", "A")),
                ((0, "torso", "B"), (0, "b2", "B")),
                ((0, "torso", "O"), (0, "conn", "O")),
                ((0, "conn", "M"), (0, "b1", "M")),
                ((0, "b1", "C"), (0, "b3", "C")),
                ((0, "b1", "D"), (0, "b4", "D")),
                ((0, "b2", "E"), (0, "b4", "E")),
                ((0, "conn", "O"), (0, "coupler", "O")),
            ]
        super().__init__(**kwargs)


def create_klann_part(orientation, phase, p="", special=False):
    O, A, B, C, D, E, F, M, b1, b2, b3, b4, conn = create_klann_geometry(orientation, phase)
    pos_A, neg_A = joint_from_point(A, p + "A", 1)
    pos_B, neg_B = joint_from_point(B, p + "B", 1)
    pos_C, neg_C = joint_from_point(C, p + "C", 1)
    pos_D, neg_D = joint_from_point(D, p + "D", 1)
    pos_E, neg_E = joint_from_point(E, p + "E", 1)
    pos_M, neg_M = joint_from_point(M, p + "M", 1)
    pos_O, neg_O = joint_from_point(O, p + "O", 1)

    b_M = neg_M if orientation != -1 else pos_M
    b1_body = rationalize_segment(b1, [b_M, neg_C, neg_D], p + "b1")
    b2_body = rationalize_segment(b2, [neg_B, neg_E], p + "b2")
    b3_body = rationalize_segment(b3, [neg_A, pos_C], p + "b3")
    b4_body = rationalize_segment(b4, [pos_E, pos_D], p + "b4")
    conn_M = pos_M if orientation != -1 else neg_M
    conn = rationalize_segment(conn, [neg_O, conn_M], p + "conn")

    need_connectors = [(neg_C, b1_body), (neg_D, b1_body), (neg_E, b2_body), (neg_M, b1_body)]
    c_conns = []
    pins = []
    for anchor, bod in need_connectors:
        pin = connector()
        p_conn = ((0, bod.name, anchor.name), (0, pin.name, "pin"))
        pins.append(pin)
        c_conns.append(p_conn)

    center_conn = ((0, "conn", p + "M"), (0, p + "b1", p + "M"))
    if special:
        center_conn = (((0, p + "conn", p + "M"), (0, p + "b1", p + "M")),)

    bodies = pins + [b1_body, b2_body, b3_body, b4_body, conn]
    _, bt_neg = joint_from_point(B, p + "B", 2)
    joints = [pos_A, pos_O, bt_neg]
    conns = [
        center_conn,
        ((0, p + "b1", p + "C"), (0, p + "b3", p + "C")),
        ((0, p + "b1", p + "D"), (0, p + "b4", p + "D")),
        ((0, p + "b2", p + "E"), (0, p + "b4", p + "E")),
    ] + c_conns
    return bodies, joints, conns


if __name__ == "__main__":
    mech = KlannLinkage()
    mech.save("single.scad")
