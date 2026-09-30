"""Triangle meshes of build123d shapes, face by face.

OCCT's incremental mesher triangulates a shape; a face it leaves without a
triangulation (some manufacturers' STEP models, the XL330's among them, have a
few such faces) would make build123d's ``Shape.tessellate`` raise on the whole
part. :func:`tessellate` walks the faces instead and skips what the mesher left
out, counting it, so the viewer's bake (:mod:`spiderpig.bake`) and the MuJoCo
model's hulls (:mod:`spiderpig.sim.mjcf`) both mesh a purchased model the same
way and neither fails on it.
"""

from __future__ import annotations

import numpy as np


def tessellate(part, tolerance: float = 0.1, angular: float = 0.1
               ) -> tuple[np.ndarray, np.ndarray, int]:
    """``(positions (nv, 3) float32, indices (nt * 3,) uint32, faces skipped)``.

    No normals: a consumer shades flat or takes the hull. The mesh is OCCT's
    incremental mesh of the whole shape at ``tolerance`` (mm) and ``angular``
    (rad); each face's triangles are appended in the face's own orientation.
    """
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_Orientation
    from OCP.TopLoc import TopLoc_Location

    BRepMesh_IncrementalMesh(part.wrapped, tolerance, True, angular, True)
    positions: list[tuple[float, float, float]] = []
    tris: list[tuple[int, int, int]] = []
    skipped = 0
    for face in part.faces():
        loc = TopLoc_Location()
        poly = BRep_Tool.Triangulation_s(face.wrapped, loc)
        if poly is None:
            skipped += 1
            continue
        trsf = loc.Transformation()
        reverse = face.wrapped.Orientation() == TopAbs_Orientation.TopAbs_REVERSED
        base = len(positions)
        for i in range(1, poly.NbNodes() + 1):
            p = poly.Node(i).Transformed(trsf)
            positions.append((p.X(), p.Y(), p.Z()))
        for i in range(1, poly.NbTriangles() + 1):
            t = poly.Triangle(i)
            a, b, c = t.Value(1) + base - 1, t.Value(2) + base - 1, t.Value(3) + base - 1
            tris.append((a, c, b) if reverse else (a, b, c))
    return (np.array(positions, dtype=np.float32).reshape(-1, 3),
            np.array(tris, dtype=np.uint32).flatten(), skipped)
