"""Triangle meshes of build123d shapes, face by face.

OCCT's incremental mesher triangulates a shape; a face it leaves without a
triangulation (some manufacturers' STEP models, the XL330's among them, have a
few such faces) would make build123d's ``Shape.tessellate`` raise on the whole
part. :func:`tessellate` walks the faces instead and skips what the mesher left
out, counting it, so the viewer's bake (:mod:`spiderpig.bake`) and the MuJoCo
model's hulls (:mod:`spiderpig.sim.mjcf`) both mesh a purchased model the same
way and neither fails on it.

The triangles are read out of the meshed shapes by OCCT's glTF writer
(``RWGltf_CafWriter``: the faces in the shape's order, each face's nodes in order, its
location applied, a reversed face's triangles flipped), which does in C++ what reading
them node by node from Python did, to the same arrays; :func:`tessellate_many` reads many
parts in one pass.
"""

from __future__ import annotations

import json
import logging
import struct
import tempfile
from pathlib import Path

import numpy as np

log = logging.getLogger("spiderpig.mesh")

Mesh = tuple[np.ndarray, np.ndarray, int]     # positions (nv, 3) f32, indices (3 nt,) u32, skipped


def tessellate(part, tolerance: float = 0.1, angular: float = 0.1) -> Mesh:
    """``(positions (nv, 3) float32, indices (nt * 3,) uint32, faces skipped)``.

    No normals: a consumer shades flat or takes the hull. The mesh is OCCT's
    incremental mesh of the whole shape at ``tolerance`` (mm) and ``angular``
    (rad); each face's triangles are appended in the face's own orientation.
    """
    return tessellate_many([part], tolerance, angular)[0]


def tessellate_many(parts, tolerance: float = 0.1, angular: float = 0.1) -> list[Mesh]:
    """:func:`tessellate` of every part (each meshed on its own: the mesher's relative
    deflection depends on the shape's size), the triangles read out in one pass. Parts
    that share a face (a hardware model placed twice) are read before the next one is
    meshed again, as one after the other would."""
    out: list[Mesh] = []
    batch: list = []
    seen: set[int] = set()
    alive: list = []                 # the faces' TShapes, kept so their ids stay theirs
    for part in parts:
        faces = [f.wrapped.TShape() for f in part.faces()]
        alive += faces
        ids = {id(f) for f in faces}
        if ids & seen:
            out += read_meshes(batch)
            batch, seen = [], set()
        mesh_part(part, tolerance, angular)
        batch.append(part)
        seen |= ids
    return out + read_meshes(batch)


def mesh_part(part, tolerance: float = 0.1, angular: float = 0.1) -> None:
    """OCCT's incremental mesh of ``part`` at ``tolerance`` (relative) and ``angular``."""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepTools import BRepTools

    # a finer mesh left on the shape by an earlier call (the bake's, then the hulls') would
    # be reused by the mesher: start from the geometry, so the mesh is the parameters' alone
    BRepTools.Clean_s(part.wrapped)
    BRepMesh_IncrementalMesh(part.wrapped, tolerance, True, angular, True)


def read_meshes(parts) -> list[Mesh]:
    """The triangulation already on each of ``parts`` (see :func:`mesh_part`), faces in
    order, a face the mesher left without one skipped and counted."""
    skipped = [_untriangulated(p) for p in parts]
    try:
        arrays = _read_gltf(parts)
    except _Unread as e:
        log.debug("read_meshes: %s; reading %d parts node by node", e, len(parts))
        arrays = [_read_faces(p) for p in parts]
    return [(pos, idx, k) for (pos, idx), k in zip(arrays, skipped, strict=True)]


def _untriangulated(part) -> int:
    from OCP.BRep import BRep_Tool
    from OCP.TopLoc import TopLoc_Location

    return sum(BRep_Tool.Triangulation_s(f.wrapped, TopLoc_Location()) is None
               for f in part.faces())


def _read_faces(part) -> tuple[np.ndarray, np.ndarray]:
    """The triangulation read node by node (what :func:`_read_gltf` falls back to)."""
    from OCP.BRep import BRep_Tool
    from OCP.TopAbs import TopAbs_Orientation
    from OCP.TopLoc import TopLoc_Location

    positions: list[tuple[float, float, float]] = []
    tris: list[tuple[int, int, int]] = []
    for face in part.faces():
        loc = TopLoc_Location()
        poly = BRep_Tool.Triangulation_s(face.wrapped, loc)
        if poly is None:
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
            np.array(tris, dtype=np.uint32).flatten())


class _Unread(Exception):
    """The glTF writer's file isn't one mesh per part as written: read the faces instead."""


def _read_gltf(parts) -> list[tuple[np.ndarray, np.ndarray]]:
    """Every part's triangles through ``RWGltf_CafWriter``: one XCAF document with a free
    shape per part (each in a compound of its own, so its location stays on its faces),
    written as a binary glTF (faces merged into one primitive per part, no unit or axis
    conversion) and read back. :class:`_Unread` unless that gives one untransformed root
    node per part, in order, with a mesh of its own and one primitive each."""
    from OCP.BRep import BRep_Builder
    from OCP.Message import Message_ProgressRange
    from OCP.RWGltf import RWGltf_CafWriter
    from OCP.TCollection import TCollection_AsciiString, TCollection_ExtendedString
    from OCP.TColStd import TColStd_IndexedDataMapOfStringString
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopoDS import TopoDS_Compound
    from OCP.XCAFApp import XCAFApp_Application
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    if not parts:
        return []
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    app = XCAFApp_Application.GetApplication_s()
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    builder = BRep_Builder()
    for part in parts:
        # XCAF turns a located shape into a reference (the location on the glTF node, the
        # nodes left local); inside a compound of its own the location is the faces', and
        # the writer applies it to the nodes as the loop does
        wrapper = TopoDS_Compound()
        builder.MakeCompound(wrapper)
        builder.Add(wrapper, part.wrapped)
        tool.AddShape(wrapper, False)
    with tempfile.TemporaryDirectory(prefix="spiderpig-mesh-") as tmp:
        path = Path(tmp) / "parts.glb"
        writer = RWGltf_CafWriter(TCollection_AsciiString(str(path)), True)
        writer.SetMergeFaces(True)
        writer.SetParallel(False)
        if not writer.Perform(doc, TColStd_IndexedDataMapOfStringString(),
                              Message_ProgressRange()) or not path.is_file():
            raise _Unread("the glTF writer failed")
        gltf, blob = _glb(path.read_bytes())
    nodes = [gltf["nodes"][i] for i in gltf["scenes"][gltf.get("scene", 0)]["nodes"]]
    if len(nodes) != len(parts):
        raise _Unread(f"{len(nodes)} root nodes for {len(parts)} parts")
    out = []
    used = set()
    for node in nodes:
        if any(k in node for k in ("matrix", "translation", "rotation", "scale", "children")):
            raise _Unread("a node with a transform or children")
        if "mesh" not in node:          # nothing triangulated
            out.append((np.zeros((0, 3), np.float32), np.zeros(0, np.uint32)))
            continue
        if node["mesh"] in used:
            raise _Unread("a mesh instanced twice")
        used.add(node["mesh"])
        prims = gltf["meshes"][node["mesh"]]["primitives"]
        if len(prims) != 1 or prims[0].get("mode", 4) != 4:
            raise _Unread("not one triangle primitive per part")
        pos = _accessor(gltf, blob, prims[0]["attributes"]["POSITION"])
        idx = _accessor(gltf, blob, prims[0]["indices"])
        out.append((np.ascontiguousarray(pos, dtype=np.float32).reshape(-1, 3),
                    np.ascontiguousarray(idx, dtype=np.uint32).reshape(-1)))
    return out


def _glb(data: bytes) -> tuple[dict, bytes]:
    magic, _, length = struct.unpack_from("<4sII", data, 0)
    if magic != b"glTF":
        raise _Unread("not a binary glTF")
    off, chunks = 12, {}
    while off < length:
        n, kind = struct.unpack_from("<I4s", data, off)
        chunks[kind] = data[off + 8:off + 8 + n]
        off += 8 + n
    return json.loads(chunks[b"JSON"]), chunks.get(b"BIN\x00", b"")


_COMPONENTS = {5126: np.float32, 5125: np.uint32, 5123: np.uint16, 5121: np.uint8}
_WIDTH = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


def _accessor(gltf: dict, blob: bytes, i: int) -> np.ndarray:
    acc = gltf["accessors"][i]
    view = gltf["bufferViews"][acc["bufferView"]]
    dtype, k, n = np.dtype(_COMPONENTS[acc["componentType"]]), _WIDTH[acc["type"]], acc["count"]
    start = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
    stride = view.get("byteStride") or k * dtype.itemsize
    rows = np.ndarray((n, k), dtype=dtype, buffer=blob, offset=start,
                      strides=(stride, dtype.itemsize))
    return rows.copy()


def export_stl(shape, path, tolerance: float = 1e-3, angular: float = 0.1) -> bool:
    """A binary STL of ``shape``: build123d's ``export_stl`` (the same mesher parameters and
    writer), meshing once. build123d constructs ``BRepMesh_IncrementalMesh``, which meshes,
    and then calls ``Perform()``, which runs the whole mesher again over the finished mesh
    (a quarter of the time) and keeps it."""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.StlAPI import StlAPI_Writer

    BRepMesh_IncrementalMesh(shape.wrapped, tolerance, True, angular, True)
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    return writer.Write(shape.wrapped, str(path))
