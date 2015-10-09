#!/usr/bin/python

from digifab import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import Voronoi, Delaunay


def create_transform(start, stop):
    # make sure they are np arrays
    start = np.array(start)
    stop = np.array(stop)
    # check for numerical instability at flip:
    if np.array_equal(stop, -1 * start):
        return quaternion_about_axis(np.pi, [0, -1, 0])
    rot_axis = np.cross(start, stop)
    cos_theta = np.dot(stop, start)
    theta = math.acos(cos_theta)
    quaternion = quaternion_about_axis(theta, rot_axis)
    return quaternion


class Stick(Body):

    STICK_COUNT = 0

    def __init__(self, length=50.0, **kwargs):
        """Make a 3mm diameter cylinder of specified length."""
        if 'diameter' not in kwargs.keys():
            kwargs['diameter'] = 3.0

        if 'name' not in kwargs.keys():
            kwargs['name'] = 'stick_%d' % Stick.STICK_COUNT
            Stick.STICK_COUNT = Stick.STICK_COUNT + 1

        self.length = length

        if 'joints' not in kwargs.keys():
            kwargs['joints'] = [Joint(ORIGIN_POSE), Joint(((0, 0, length), ORIGIN_POSE[1]))]

        dia = float(kwargs['diameter'])
        if 'layers' not in kwargs.keys():
            kwargs['layers'] = Layer(
                PolyMesh(generator=solid.cylinder(d=dia, h=length)),
                name='stick', color='white')

        super(Stick, self).__init__(**kwargs)

    def guide(self):
        """Return guide PolyLine with name label for this stick."""

        bar_pl = PolyLine([[0, -2], [0, 2]])

        stick_pl = PolyLine([[0, 0], [self.length, 0]])

        stick_num = self.name.split('_')[-1]
        label_pl = (2, -2, 0) * PolyLine(generator=solid.text(stick_num, 4))

        len_pose = (self.length, 0, 0)

        guide_pl = bar_pl + stick_pl + len_pose * (bar_pl + label_pl)

        return guide_pl


def make_tenon():
    base = solid.square(15, center=True)
    base_thick = solid.linear_extrude(2)(base)
    top = solid.square(9, center=True)
    top_thick = solid.linear_extrude(8.1)(top)
    total = base_thick + top_thick
    # offset to be embedded in plate:
    tf = solid.translate([0, 0, -8])(total)
    return PolyMesh(generator=tf)


def make_mortise(label, axis):
    """
    Make mortise to connect to rod oriented with axis.
    """
    height = 9.0
    thickness = 2.0
    hole_depth = 6.0
    hole_width = 3.4
    offset = 3.0
    shell = solid.cylinder(d=hole_width + 2 * thickness, h=height)
    label = "A"  # TODO: fix this shit
    # Add hole
    hole = solid.translate([0, 0, offset])(
        solid.hole()(solid.cylinder(d=3.4, h=6.5, segments=10)))
    mortise = shell + hole

    # Add label
    lb = solid.text(label)
    lb_scale = solid.scale(.3)(lb)
    lb_thick = solid.linear_extrude(1.5)(lb_scale)
    lb_rot = solid.rotate(90, [1, 0, 0])(lb_thick)
    lb_offset = hole_width / 2 + thickness / 2  # offset so it intersects surface
    lb_trans = solid.translate([0, -lb_offset, 5])(lb_rot)
    with_label = mortise + lb_trans
    lb_mesh = PolyMesh(generator=with_label)

    # rotate to match axis
    z_axis = np.array([0, 0, 1])
    rot = create_transform(z_axis, axis)
    pt = [0, 0, 0]  # rotate to match center of R= 3mm sphere
    pose = (pt, rot)
    oriented_mesh = lb_mesh.transformed(pose)
    return oriented_mesh


class Connector(Body):

    CONNECTOR_COUNT = 0

    def __init__(self, axes=None, **kwargs):
        """Make a connector with specified connection axes.

        Args:
          axes { str: [float] } : Dictionary mapping the name of a connected
            block (Stick or Plate) to a list of float representing the unit axis
            in the direction of the connection.
        """

        if axes is None:
            axes = []

        self.axes = axes

        if 'name' not in kwargs.keys():
            kwargs['name'] = 'connector_%d' % Connector.CONNECTOR_COUNT
            Connector.CONNECTOR_COUNT = Connector.CONNECTOR_COUNT + 1

        total_mesh = PolyMesh(generator=solid.sphere(6.0))

        # if this is a plate connector, we don't need the center
        if any(["plate" in name for (name, axis) in axes.items()]):
            total_mesh = make_tenon()

        for (name, axis) in axes.items():
            if "plate" not in name:
                mort = make_mortise(name, axis)
                total_mesh = total_mesh + mort

        if 'layers' not in kwargs.keys():
            kwargs['layers'] = Layer(
                total_mesh,
                name='print', color='blue')

        super(Connector, self).__init__(**kwargs)


class Plate(Body):

    PLATE_COUNT = 0

    def __init__(self, base_pts=None, **kwargs):
        """Make a support plate with specified connection holes at base_pts

        Args:
          base_pts [[float]]: list of 3D coordinates of support connectors (the
            z coordinate will be 0).
        """

        if base_pts is None:
            base_pts = []

        self.base_pts = base_pts

        if 'name' not in kwargs.keys():
            kwargs['name'] = 'plate_%d' % Plate.PLATE_COUNT
            Plate.PLATE_COUNT = Plate.PLATE_COUNT + 1

        if 'layers' not in kwargs.keys():
            gen = solid.translate([0, 0, -6])(solid.cube([150, 150, 6]))
            kwargs['layers'] = Layer(
                PolyMesh(generator=gen), name='cut', color='red')

        super(Plate, self).__init__(**kwargs)

    def cut(self):
        """Return cut geometry for this support plate."""
        thick_plate = self[0][0].get_generator()
        subtracted_plate = thick_plate
        for pt in self.base_pts:
            tenon = make_tenon().get_generator()
            placed_tenon = solid.translate(list(pt))(tenon)
            subtracted_plate = subtracted_plate - placed_tenon
        return PolyLine(
            generator=solid.projection(cut=True)(subtracted_plate)
        )


def make_sparse_cloud(n_points=6, scale=150.0):
    """Return a 3D point cloud with minimum distance between points."""

    cloud = scale * numpy.random.rand(1, 3)

    while cloud.shape[0] < n_points:
        new_pt = scale * numpy.random.rand(1, 3)
        dists = (((cloud - new_pt)**2).sum(axis=1))**0.5
        if all(dists > scale / 5.0):
            cloud = numpy.vstack([cloud, new_pt])

    return cloud + [0, 0, 50.0]


class Sculpture(Layout):

    def __init__(self, cloud=None, **kwargs):

        if cloud is None:
            cloud = make_sparse_cloud()

        self.cloud = cloud

        self.edges = []

        delaunay = Delaunay(self.cloud)

        for simplex in delaunay.simplices:
            sl = simplex.tolist()
            simplex_edges = []
            for i in range(len(sl)):
                for j in range(i + 1, len(sl)):
                    simplex_edges.append((sl[i], sl[j]))
            for edge in simplex_edges:
                if edge not in self.edges and edge[::-1] not in self.edges:
                    self.edges.append(edge)

        min_z = self.cloud[:, 2].argsort()[0:3]
        base_pts = self.cloud[min_z, :]
        base_pts[:, 2] = 0.0

        self.edges.extend([(self.cloud.shape[0] + i, min_z[i])
                           for i in range(3)])
        self.cloud = numpy.vstack([self.cloud, base_pts])

        if 'blocks' not in kwargs.keys():
            blocks = []
            connector_axes = [dict() for i in range(self.cloud.shape[0])]

            # Iterate through edges, create sticks for each, and record axes
            # for connector
            for edge in self.edges:
                pt0, pt1 = self.cloud[edge, :]
                vector = pt1 - pt0
                length = ((vector)**2).sum()**0.5
                axis = vector / length

                # If the current axis is very nearly vertical, just coerce to vertical to
                # avoid numerical instability problems
                if (1.0 - abs(axis[2])) < 0.001:
                    if axis[2] > 0:
                        quat = ORIGIN_POSE[1]
                    else:
                        quat = NZ_JOINT_POSE[1]

                # Otherwise, use cross product to find axis of rotation to get z axis
                # aligned with stick direction, and dot product to find magnitude of
                # rotation angle
                else:
                    rot_axis = numpy.cross([0, 0, 1], axis)
                    rot_angle = numpy.arccos(axis[2])
                    quat = quaternion_about_axis(rot_angle, rot_axis)

                # Add Sticks
                length = length - 6.0  # leave room for joinery
                new_stick = Stick(length, pose=(pt0, quat), diameter=3.4)
                blocks.append(new_stick)

                # Update connector axes list with mapping from stick name to the unit
                # vector axis  along that stick
                connector_axes[edge[0]][new_stick.name] = axis.tolist()
                connector_axes[edge[1]][new_stick.name] = (-axis).tolist()

            # Add Plate body
            plate = Plate(base_pts)
            blocks.append(plate)

            # Add an axis for connection to plate for the last three connectors
            for axes in connector_axes[-3:]:
                axes[plate.name] = [0.0, 0.0, -1.0]

            # Add Connector bodies with connection axes and correct translation
            # pose
            for i in range(len(connector_axes)):
                conn_axes=connector_axes[i]
                conn_pose=(self.cloud[i, :].tolist(), ORIGIN_POSE[1])
                blocks.append(Connector(conn_axes, pose = conn_pose))
            kwargs['blocks'] = blocks

        super(Sculpture, self).__init__(**kwargs)

    def show_wireframe(self):
        """Show matplotlib 3D plot of wireframe of sculpture."""

        fig = plt.figure(2)
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(*self.cloud.T)

        for edge in self.edges:
            ax.plot(*self.cloud[edge, :].T)

        ax.axis('equal')

        plt.show()

    def show(self):
        super(Sculpture, self).show(is_2d=False)

    def get_layouts(self):
        """Return layouts for making this sculpture.

        Returns:
          guide_layout: 2D Layout for printing, use to cut sticks
          cut_layout: 2D Layout for laser cutting base
          print_layout: 3D Layout for 3D printing connectors

        """
        sticks = [body.clone() for body in self if type(body) is Stick]
        guide_layout = Layout(
            [Block(Layer(stick.guide())) for stick in sticks],
            size=(215, 279)  # Letter paper size
        )
        for i in range(len(sticks)):
            guide_layout[i] *= (0, 2.5 + i * 5.0, 0)

        plates = [body for body in self if type(body) is Plate]
        cut_layout = Layout(
            Block(Layer(plates[0].cut(), color='red')),
            size=(300, 300)  # Laser Cutter bed is larger, but use 30x30cm for now
        )

        connectors = [
            body.clone(pose=ORIGIN_POSE) for body in self if type(body) is Connector
        ]
        max_dims = numpy.vstack([conn.dims(2)
                                 for conn in connectors]).max(axis=0)
        n_x = numpy.floor(200 / max_dims[0])
        i = 0
        j = 0
        print_layout = Layout(size=(200, 200))
        for conn in connectors:
            z_min = conn.bounds()[0, 2]
            conn.pose = (
                (i * max_dims[0], j * max_dims[1], -z_min), ORIGIN_POSE[1])
            print_layout += conn.transformed()
            if i == n_x - 1:
                i = 0
                j += 1
            else:
                i += 1
        return guide_layout, cut_layout, print_layout


def seeded_solution(seed=1):
    numpy.random.seed(seed)
    sculpture = Sculpture()
    guide_layout, cut_layout, print_layout = sculpture.get_layouts()
    return sculpture, guide_layout, cut_layout, print_layout


def test_connector():
    axes = {"a": np.array([0, 0, 1]), "b": np.array([1, 0, 0])}
    conn = Connector(axes)
    return conn.get_generator()


def solid_hole_example():
    h_g = solid.translate([0, 0, -3])(solid.cylinder(3, 20)) + \
        solid.hole()(solid.cylinder(1.5, 20))
    h_g += solid.sphere(16)
    PolyMesh(generator=h_g).show()
    # Holes always get added at the end, so the geometry is ensured to be empty
    # This ALWAYS happens, so if you want to restrict the scope of holes
    h_g = solid.part()(solid.translate(
        [0, 0, -3])(solid.cylinder(3, 20)) + solid.hole()(solid.cylinder(1.5, 20)))
    h_g += solid.sphere(16)
    PolyMesh(generator=h_g).show()


def export_sculpture():
    sculpture = Sculpture()
    lol = sculpture.get_generator()
    return solid.scad_render_to_file(lol, "test.scad")

if __name__ == '__main__':
    sculpture, guide_layout, cut_layout, print_layout = seeded_solution()
    sculpture.save('sculpture.scad')
    guide_layout.save('guide_layout.dxf')
    cut_layout.save('cut_layout.dxf')
    print_layout.save('print_layout.stl')
