#!/usr/bin/python

from digifab import *
import solid
import solid.utils
import math
import numpy as np
import numpy.linalg as la


def addOffset(tri, offset):
	"""
	generate larger triangle formed by adding offset to triangle
	"""
	pts = tri.points
	L = getDistance(*pts[0:2])
	l = offset / math.sin(math.pi / 3)
	scale = float((L + offset + l) / L)
	withOffset = scale * tri
	return withOffset


def centerObjects(parent, child):
	p1 = center_point(parent)
	p2 = center_point(child)
	dx, dy = p1 - p2
	return (dx, dy, 0) * child

def centerObjects3D(parent, child):
	p1 = center_point(parent)
	p2 = center_point(child)
	dx, dy, dz = p1 - p2
	return (dx, dy, dz) * child


def triangle(length=40.0, r=3.175 / 2):
	"""
	Generate list of points corresponding to triangular side of a tetrahedron.
	Also generate points for same trianglular side, offset to allow joinery in
	the tetrahedron.

	Args:
	  Side length of tetrahedron, radius of wire to use

	Returns:
	  Triangle points, Offset points
	"""
	# equilateral triangle:
	a = np.array([0, 0])
	b = np.array([length, 0])
	c = np.array([length / 2, length * math.sin(math.pi / 3)])
	tri_pts = PolyLine([a, b, c, a])
	offs_pts = addOffset(tri_pts, r)
	tri_pts = centerObjects(offs_pts, tri_pts)
	return tri_pts, offs_pts


def circle(r, n):
	s = np.pi / n
	circle_pts = []
	t = 0.0
	while t <= 2 * np.pi:
		circle_pts.append([np.sin(t), np.cos(t), 0])
		t += s
	return circle_pts


def polyline_to_3D(polylines):
	pl3d = PolyLine([[x, y, 0] for (x, y) in polylines.points])
	return pl3d


def polyline_to_solidlist(polyline):
	pl3d = PolyLine([[x, y, z] for (x, y, z) in polyline.points])
	aspath = [list(p) for p in pl3d.points]
	return aspath


def mesh_wire(polyline, r):
	ds = circle(2, 50)
	path = polyline_to_solidlist(polyline)
	mesh = solid.utils.extrude_along_path(circle(2, 50), path)
	return mesh


def unique(points):
	hashable_pts = [tuple(p) for p in points]
	unique = np.unique(hashable_pts)
	np_pts = [np.array(x) for x in unique]
	return np_pts


def center_point(polyline):
	"""
	Find center point of polyline
	"""
	pts = unique(polyline.points)
	return sum(pts) / len(pts)


def rotate_polyline(polyline, matrix):
	points = np.array(polyline.clean().points)
	return PolyLine(compose([r, points.T])[0:3].T)


def tetrahedron(length=40.0, r=3.175 / 2):
	"""
	Simulate a tetrahedron made out of 4 ebnt wire triangles. To transform a
	list of points, create a rotation_matrix (defined in transformations.py),
	and use as follows:

	r = rotation_matrix(ang,direction, point)
	transformed_points = [compose([r,points.T])[0:3].T]

	Those transposes are necessary for the homogeneous transformations to work
	properly.

	Args:
	  Side length of tetrahedron, radius of wire to use

	Returns:
	  Solid Python object
	"""
	t_pts, o_pts = triangle(length, r)
	# todo: find proper rotation matrix to make tetrahedron.
	t3 = polyline_to_3D(t_pts)
	o3 = polyline_to_3D(o_pts)
	sideIter = takeNGenerator(o_pts.simplified().points, 2)
	sides = [(t3, o3)]
	for side in sideIter:
		side3 = polyline_to_3D(PolyLine(side))
		rot_pt = center_point(side3)
		(p1, p2) = side3.points
		direction = p2 - p1
		ang =math.acos(1.0/3.0)
		r = rotation_matrix(ang, direction, rot_pt)
		new_t = rotate_polyline(t3, r)
		new_o = rotate_polyline(o3, r)
		sides.append((new_t, new_o))
	meshed = [(mesh_wire(t, r), o) for (t, o) in sides]
	# make into solid object
	parts = [mesh for (mesh, outline) in meshed]
	total = parts[0]
	for i in range(1, len(parts)):
		total += parts[i]
	return total

def fast_forward(iter, num):
	for i in range(num):
		iter.next()

def create_transform(start, stop):
    # make sure they are np arrays
    start = np.array(start)
    stop = np.array(stop)
    # check for numerical instability at flip:
    if np.array_equal(stop, -1 * start):
        return quaternion_about_axis(np.pi, [0, -1, 0])
    rot_axis = np.cross(start, stop)
    cos_theta = np.dot(stop, start)
    #correct for numerical errors:
    if cos_theta >= 1:
        cos_theta = 1.0
    if cos_theta <= -1:
        cose_theta = -1.0
    theta = math.acos(cos_theta)
    quaternion = quaternion_about_axis(theta, rot_axis)
    return quaternion


def unfold_wire(pl):
	"""
	Unfold a list of 3D points such that they are planar, while maintaining the
	open angle between the points.

	Args:
	  List of points

	Returns:
	  Unfolded list of points
	"""
	pl = phone_pl
	shape_points = [np.array(p) for p in pl.points]
	pointIter = takeNGenerator(shape_points, 4)
	d0 = getDistance(*shape_points[0:2])
	points = [np.array([0, 0]), np.array([0, d0])]
	for i in range(len(shape_points)-3):
		(p1,p2,p3,p4) = pointIter.next()
		v1 =p1-p2
		v2 = p3-p2
		v3 = p2-p3
		v4 = p4-p3
		old_normal = np.cross(v1,v2)
		new_normal = np.cross(v3,v4)
		norm_old = old_normal/la.norm(old_normal)
		norm_new = old_normal/la.norm(new_normal)


		#check if we need to transform:
		if any(norm_old != norm_new):
			print norm_old, norm_new
			#create a transform that will rotate the next points to the old orientation
			transform = create_transform(norm_new, norm_old)
			rot_pot = p2
			pose = (rot_pot, transform)
			poly = PolyLine(shape_points[i:])
			translated = poly.transformed(pose)
			new_pts = [np.array(p) for p in translated.points]

			if len(shape_points[:i]) is 0:
				shape_points = new_pts
			else:
				shape_points = np.vstack((shape_points[:i], new_pts))
			pointIter = takeNGenerator(shape_points, 4)
			fast_forward(pointIter, i)
	return PolyLine(shape_points)


def getDistance(p1, p2):
	"""
	Measures distance between p1, p2.
	Type: p1,p2 := Np Array, dim: 3
	"""
	dist = la.norm(p2 - p1)
	return dist


def getAngle(p1, p2, p3):
	"""
	Measures the interior angle formed by p1, p2, p3.
	Type: dim 3 np array,
	Returns: angle in degrees
	"""
	v1 = p1 - p2
	v2 = p3 - p2
	mag = la.norm(v1) * la.norm(v2)
	c = np.dot(v1, v2) / mag
	cross = np.cross(v1,v2)
	s = la.norm(cross)/mag
	atang = math.atan2(s,c)
	ang = atang * 180 / math.pi
	return ang


def takeNGenerator(seq, n):
	"""
	returns a generator of all subsequences of length n in the list, in order.
	"""
	index = 0
	while index + n <= len(seq):
		yield seq[index:index + n]
		index = index + 1


def diwire_dfm(pl):
	"""
	Check that a polyline is manufacturable by the DI-WIRE. No points closer
	than 12mm, no angle sharper than 135 degrees

	Args:
	  PolyLine

	Returns:
	  True or False
	"""
	# check angle
	take3 = takeNGenerator(pl.points, 3)
	valid = reduce(lambda flag, p: flag and (getAngle(*p) <= 135), take3, True)
	# check distance
	take2 = takeNGenerator(pl.points, 2)
	valid = reduce(lambda flag, p: flag and (
		getDistance(*p) >= 12), take2, valid)
	if valid:
		return True
	else:
		return False


def phone_holder():
	points = [[3.0568, 0.0, 0.4915],
			  [3.4, 0.0, 0.0],
			  [2.8, 0.0, 0.0],
			  [1.6405, 0, 1.6405],
			  [0.0, 0.0, 0.0],
			  [0.0, 2.0, 0.0],
			  [1.6405, 2.0, 1.6405],
			  [2.8, 2.0, 0.0],
			  [3.4, 2.0, 0.0],
			  [3.0568, 2.0, 0.4915]]
	pl = PolyLine(points)
	return 25*pl


def sim_wire(pl, r=3.175 / 2):
	"""
	Simulate the output of the DI-WIRE given an ordered list of 3D coordinates
	corresponding to bend points. If you want to get really fancy, simulate the
	bend radius.

	Args:
	  3D Point list

	Returns:
	  SolidPython object
	"""
	return wire_sim


if __name__ == '__main__':
	# Create equilateral triangle geometry
	tri = PolyLine(points=triangle()[1].points)

	# Check triangle for manufacturability
	assert(diwire_dfm(tri))

	#make phone case
	phone_pl = phone_holder()
	mesh_wire(phone_pl,2)
	phone_2d = unfold_wire(phone_pl)
	mesh_wire(phone_2d,2)
	assert(diwire_dfm(phone_2d))
	phone_2d.show()
	phone_2d.save('holder.dxf')

	# Create Layout and write to DXF file for use with DIWIRE
	lo = Layout(
		blocks=Block(
			layers=Layer(name='bend', color='blue', geometries=tri)
		)
	)

	lo.save('lab_1.dxf')

	tet_mesh = PolyMesh(generator=tetrahedron())
	tet_mesh.save('lab_1.scad')
	tet_mesh.save('lab_1.stl')
