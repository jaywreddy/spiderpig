#!/usr/bin/python

from digifab import *
import solid as sl
import os
import solid.utils
import numpy as np
from numpy import linalg as la
import math


def mesh_extremum(mesh, axis):
    """
    Identifies the extreme points of the mesh bounded by the halfspaces:
    {(t * v), v in [alpha_min, alpha_max]}
    assumes axis is normalized.
    """
    dots = [np.dot(p, axis) for p in mesh.points]
    alpha_max = max(dots)
    alpha_min = min(dots)
    return (alpha_min, alpha_max)


def frange(start, stop, step):
    """
    A range iterator that permits float values
    """
    x = start
    while x < stop:
        yield x
        x += step


def planar_slice(mesh, slice_args, slice_width=5):
    """
    Returns a 2D projection of where the mesh intersects the plane.

    Args:
        mesh (PolyMesh): 3D PolyMesh object to be sliced
        plane (PolyLine): a 3D PolyLine of coplanar points
    """
    (plane, offset) = slice_args
    slice_gen = sl.linear_extrude(slice_width)(plane.get_generator())
    slice_gen = sl.translate(offset)(slice_gen)
    slice = PolyMesh(generator=slice_gen)
    intersection = (slice.intersected(mesh)).get_generator()
    return PolyLine(generator=sl.projection()(intersection))

def planar_slice_3d(mesh, slice_args, slice_width = 1):
    (plane, offset) = slice_args
    slice_gen = sl.linear_extrude(slice_width)(plane.get_generator())
    slice_gen = sl.translate(offset)(slice_gen)
    slice = PolyMesh(generator=slice_gen)
    intersection = (slice.intersected(mesh))
    return intersection

def slice_mesh(fn='LOGOROBOT.stl', t_m=3.0):
    """ Return list of slices from stl modifed with through holes for alignment.

    Args:
      filename (str): STL file with 3D solid to turn into slices
      t_m (float): material thickness

    Returns:
      list of PolyLines that are the 2D profiles of the slices

    """
    poly = PolyMesh(filename=fn)
    #custom rotation for LOGOROBOT
    rot = sl.rotate(-90, [1, 0, 0])(poly.get_generator())
    poly = PolyMesh(generator=rot)
    slices = []
    plane = sl.square(1000, True)
    axis = np.array([0, 0, 1])  # slice in Z direction
    (alpha_min, alpha_max) = mesh_extremum(poly, axis)
    for alpha in frange(alpha_min, alpha_max, t_m):
        norm = list(axis * alpha)
        slice_polyline = PolyLine(generator=plane)
        slice_args = (slice_polyline, norm)
        slice = planar_slice(poly, slice_args, t_m)
        slices.append(slice)
    return slices


def slice_sim(slices, t_m=3.0):
    """ Returns simulation of assembled slice-formed solid.

    Args:
      slices ([PolyLine]): list of PolyLines to simulate.
      t_m (float): material thickness

    Returns:
      A PolyMesh representing the assembled slices.

    """
    # assume stacked in the z dimension
    axis = np.array([0, 0, 1])
    slice_solids = []
    for (i, slice) in enumerate(slices):
        translation = list(t_m * axis * i)
        slice_solid = sl.linear_extrude(t_m)(slice.get_generator())
        positioned_solid = sl.translate(translation)(slice_solid)
        slice_solids.append(positioned_solid)
    return PolyMesh(generator=sl.union()(slice_solids))


def assemble_chair(polygons, tf_xyz_rpy, t=3.0):
    """ Create a 3D rendering of a chair from part outlines.

    Args:
      polygons ([PolyLine]): list of PolyLines representing the outlines of the chair parts
      tf_xyz_rpy ([[x,y,z][r,p,y]]): List of transformations as x,y,z offsets and roll, pitch,
        yaw rotations in degrees

      t (int): material thickness

    Returns:
      A list of PolyMeshes, with one PolyMesh per input polygon, transformed by corresponding
      tf input

    """
    polys = []
    for (p,r) in zip(polygons, tf_xyz_rpy):
        translation, rotation = r
        solid_p = p.get_generator()
        thick_p = sl.linear_extrude(t)(solid_p)
        rotated_p = sl.rotate(rotation)(thick_p)
        translated_p = sl.translate(translation)(rotated_p)
        poly = PolyMesh(generator = translated_p)
        polys.append(poly)
    return polys


def join_butt(solids):
    """Remove intersecting material of solids to allow for butt joints

    Args:
      solids ([PolyMesh]): List of PolyMeshes that are intersecting at butt joints

    Returns:
      [PolyMesh] list of PolyMeshes modified with butt joints

    """
    for i1 in range(len(solids)):
        for i2 in range(i1):
            s1 = solids[i1]
            s2 = solids[i2]
            intersection = s1.intersected(s2)
            solids[i1]= s1.differenced(intersection)
    return solids

def plane_from_normal(norm):
    plane = sl.square(1000,True)
    plane_poly = PolyLine(generator=plane)
    plane_axis = np.array([0,0,1])
    if norm == plane_axis:
        return plane
    rot_axis = np.cross(norm, plane_axis)
    cos_theta = np.dot(norm, plane_axis)
    theta = math.acos(cos_theta)
    quaternion = quaternion_about_axis(theta, rot_axis)

    z = np.array([0,0,1])
    x = np.array([1,0,0])
    rot_axis = np.cross(x,z)
    cos_theta= np.dot(z,x)
    theta = math.acos(cos_theta)
    quat = quaternion_about_axis(theta, rot_axis)
    quat_mat = quaternion_matrix(quat)
    x * quat
    I = identity_matrix()
    plane_poly.points
    plane_poly * e
    e = euler_from_quaternion(quat)
    np.dot(quat,np.array(z))



def join_box(solids):
    """
    Args:
      List of elaborated solids
    Returns:
      List of solids modified for joinery
    """
    butt_join_thickness = 5
    for i1 in range(len(solids)):
        for i2 in range(i1):
            s1 = solids[i1]
            s2 = solids[i2]
            intersection = s1.intersected(s2)

            #identify cut axis:
            points = intersection.points
            maxx = None
            maxy = None
            maxz = None
            for j1 in range(len(points)):
                for j2 in range(j1):
                    p1 = points[j1]
                    p2 = points[j2]
                    dp = p2-p1
                    dx,dy,dz = dp
                    maxx = max(la.norm(maxx), dx) * np.array[1,0,0]
                    maxy = max(la.norm(maxy), dy) * np.array[0,1,0]
                    maxy = max(la.norm(maxz), dz) * np.array[0,0,1]

            #only slice if we find splitting axis
            if(maxx):
                split_axis = max([maxx,maxy,maxz], key=la.norm)
                norm_axis = split_axis/la.norm(split_axis)
                start_point = min(points, key= lambda x: np.dot(norm_axis, x))
                end_val = max([np.dot(norm_axis, p) for p in ponits])
                for l in frange(0,end_val, box_join_thickness):
                    slice_norm = start_point + norm_axis*l
                    plane = plane_from_normal(slice_norm)
                    slice_args = (plane, slice_norm)
                    planar_slice_3d(intersection, slice_args, butt_join_thickness)

                #segmented intersection 1
                seg1 = intersection
                #segmented intersection 2
                seg2 = intersection

                solids[i1]= s1.differenced(seg1)
                solids[i2]= s2.differenced(seg2)
    return solids


def rationalize_planar_solids(solids, tf_xyz_rpy, offset):
    """
    Args:
      List of solids modified for joinery
    Returns:
      List of PolyLines projected from solids, offset for laser kerf
    """
    final_list = []
    #reverse the transformation, then call ayout
    for (p,r) in zip(solids, tf_xyz_rpy):
        translation, rotation = r
        #create reverse
        translate*=-1
        rotation*=-1
        solid_p = p.get_generator()
        translated_p = sl.translate(translation)(solid_p)
        rotated_p = sl.rotate(rotation)(translated_p)
        p_2d = sl.projection(rotated_p)
        polyline = PolyLine(generator=p_2d)
        offset = offset_polygon(polyline, solid)
        final_list.append(offset)
    return final_list


def offset_polygon(pl, offset):
    """ A work around to get offsets working nicely with solid python functions

    Args:
      pl (PolyLine): polygon geometry
      offset (float): distance to offset points

    Returns:
      offset PolyLine

    """
    pp = [p + [0] for p in pl.points.tolist()]
    off_pts = numpy.array([p[0:2] for p in solid.utils.offset_points(
        pp[0:-1], offset, closed_poly=True)])
    return PolyLine(points=numpy.vstack([off_pts, off_pts[0]]))


def join_with_fold(bA, fpA, plB, fpB):
    """
    The first polyline in Block A needs to be the part outline
    Args:
      bA (Block): Block to add new polyline to
      fpA (numpy.array([p0], p1])): numpy array of two points on block A to mate to polyline
      plB (PolyLine): Polyline to join to Block bA
      fpA (numpy.array([p0], p1])): numpy array of two points on polyline B to mate to Block
    Returns:
      Combined Block with fold and cut layers
    """

    return


def add_tab(b, tp, right_handed=True):
    """
    Args:
      b (Block): Block to which tab should be added
      tp (numpy.array([p0], p1])): numpy array of two points on block to add tab
      right_handed (bool): Boolean to indicated handedness of tab
    Returns:
      Block with added tab
    """

    return


def save_layout(pls, filename):
    """ Given a list of polylines, make a layout and save it to DXF files for laser cutting

    Args:
      pls ([PolyLine]): list of polylines
      filname (str): beginning of file name (sheet number and DXF extension will be added)

    """

    for pl in pls:
        pl.generator = None
    lo = Layout([Block(Layer(s, color='red')) for s in pls], size=(250, 100))

    sheets = lo.solved(5)

    for i in range(len(sheets)):
        sheets[i].save('%s_sheet_%d.dxf' % (filename, i), version='AC1021')


def slice_forms():
    ##################
    # SLICE FORMS
    ##################
    # Create slices from mesh
    slices = slice_mesh()

    save_layout(slices, 'lab_2_slice')

    # Save simulation of sliced solid
    slice_sim(slices).save('lab_2_slice_sim.scad')


def joinery():
    ##################
    # 2D JOINERY
    ##################

    # Assemble chair from polygons
    seat_pl = PolyLine(
        [[0.0, 0.0], [50.0, 0.0], [50.0, 50.0], [0.0, 50.0]], polygon=True)
    back_pl = PolyLine(
        [[0.0, 0.0], [85.0, 0.0], [85.0, 50.0], [0.0, 50.0]], polygon=True)
    side_a_pl = PolyLine(
        [[0.0, 0.0], [10.0, 0.0], [10.0, 35.0], [40.0, 35.0],
         [40.0, 0.0], [50.0, 0.0], [50.0, 50.0], [0.0, 50.0], [0.0, 0.0]], polygon=True)
    side_b_pl = PolyLine(
        [[0.0, 0.0], [10.0, 0.0], [10.0, 35.0], [40.0, 35.0],
         [40.0, 0.0], [50.0, 0.0], [50.0, 50.0], [0.0, 50.0], [0.0, 0.0]], polygon=True)

    chair_pl = [seat_pl, back_pl, side_a_pl, side_b_pl]

    # Material thickness
    t = 3

    # Transformations to extrude chair from pieces
    tfs = [
        [[0, 0, 50 - t], [0, 0, 0]],
        [[t, 0, 35], [0, -90, 0]],
        [[0, t, 0], [90, 0, 0]],
        [[0, 50, 0], [90, 0, 0]]
    ]

    s = assemble_chair(chair_pl, tfs, t)

    j_s = join_butt(s)

    r_s = rationalize_planar_solids(j_s, tfs, 0.1)

    save_layout(r_s, 'chair_pieces')


def folding():
    ##################
    # FOLDING
    ##################

    # Here's an example of how to use your join_with_fold function

    b = Block([Layer(name='cut', color='red'),
               Layer(name='fold', color='black')])

    p1 = PolyLine(generator=solid.square()).simplified()
    p2 = (1, 3, 2) * p1
    b['cut'] += p1
    fpA = [p1.points[2], p1.points[3]]
    fpB = [p2.points[3], p2.points[2]]

    f = join_with_fold(b, fpA, p2, fpB)

    f.show()

    # Create the add tab function, and add some tabs to this thing

    add_tab(f, numpy.array([[1, 0], [1, 0.25]]))
    add_tab(f, numpy.array([[0, 0.25], [0, 0.7]]), right_handed=False)

    f.show()

    # Yaaay tabs

    # Now assemble the chair polygons by sequentially applying the
    # join_with_fold function, adding tabs where needed. I'll do the first one

    c = Block([Layer(name='cut', color='red'),
               Layer(name='fold', color='black')])
    c['cut'] += PolyLine(points=seat)

    c_pts = numpy.array([seat[3], seat[2]])
    s_pts = numpy.array([side_a[-3], side_a[-2]])
    c = join_with_fold(c, c_pts, PolyLine(points=side_a), s_pts)

    l = Layout(c)
    l.save('folded_chair.dxf')


if __name__ == '__main__':
    slice_forms()

    print 'Slice Forms done'

    joinery()

    print 'Joinery done'

    folding()

    print 'Solution Done'
