"""An arm's baked model posed at joint angles (the viewer's joint sliders, 2026-10-12).

The bake (:mod:`spiderpig.bake`) animates an arm over its inputs' tour; to pose it anywhere
in its box, the viewer asks for each node's transform relative to the clip's first frame
(:func:`node_poses`): the planar motion of the node's anchor body from the reference pose
to the inputs given (the linkage's points at those inputs, :meth:`linkage.LegSolution.at`,
fitted per body as the bake fits them, :func:`walk.planar_fit`), composed with the spatial
joints at the angles given (:class:`kinematics.RobotTree`), over the same composition at
the first frame. The node's matrix there times it is the node posed: no part is
fabricated, the glb's own node extras (``rigid_with``) and scene metadata (its ``tree``)
say who rides whom.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from spiderpig import linkage
from spiderpig.config import BuildConfig
from spiderpig.kinematics import RobotTree, planar_matrix


def node_poses(config: BuildConfig, glb: Path, q_deg: dict[str, float]) -> dict[str, list[float]]:
    """Each node's 4x4 (column-major, three.js's order) taking it from the clip's first
    frame to the joints ``q_deg`` (degrees: the planar inputs' design angles, the robot's
    ``q1`` / ``q4`` from their zeros; a joint not given stays where the first frame has it)."""
    import pygltflib

    from spiderpig import walk
    from spiderpig.bake import T_REF
    from spiderpig.construction.robot import layout_of
    from spiderpig.fabricate import template_for

    lk = config.lk
    if lk.kind != "arm":
        raise ValueError(f"{lk.key} is a {lk.kind}: joint poses are an arm's")
    g = pygltflib.GLTF2().load(str(glb))
    if g is None:
        raise ValueError(f"{glb}: not a glb")
    scene = g.scenes[g.scene or 0]
    tree = RobotTree.from_meta((scene.extras or {}).get("tree"))
    tmpl = template_for(config)
    if config.robot:
        tmpl = layout_of(config).template(tmpl)
    ts = np.array([T_REF])
    ref = tmpl.sample(ts).joint_world
    q0 = lk.tour(ts)
    qs = [math.radians(q_deg[n]) if n in q_deg else float(q0[i][0])
          for i, n in enumerate(lk.inputs)]
    pts = linkage.LegSolution(1, 0.0, config.values, lk.key).at(*(np.atleast_1d(v) for v in qs))
    planar, planar0 = {}, {}
    for b in tmpl.bodies:
        names = [j.name for j in b.joints if j.name in pts]
        if not names:
            continue
        p0 = np.stack([ref[b.name][n][0] for n in names])[None]
        p1 = np.stack([np.append(np.asarray(pts[n]).reshape(-1, 2)[0], ref[b.name][n][0, 2])
                       for n in names])[None]
        th, tr = walk.planar_fit(p0, p1)
        planar[b.name] = planar_matrix(th, tr)
        planar0[b.name] = np.eye(4)[None]
    joints = joints0 = {}
    if tree is not None:
        first = tree.sweep(np.zeros(1))
        angles = {j.name: (np.array([math.radians(q_deg[j.name])]) if j.name in q_deg
                           else first[j.name]) for j in tree.joints}
        joints = tree.joint_transforms(angles, planar)
        joints0 = tree.joint_transforms(first, planar0)
    nodes = {n.name: n for n in g.nodes if n.name}
    out: dict[str, list[float]] = {}
    for name, node in nodes.items():
        extras = node.extras or {}
        if "body" not in extras:
            continue
        anchor = _anchor(name, nodes, planar)
        if tree is not None:
            t1 = tree.body_transform(name, anchor, joints, planar, 1)[0]
            t0 = tree.body_transform(name, anchor, joints0, planar0, 1)[0]
        else:
            t1 = planar[anchor][0] if anchor else np.eye(4)
            t0 = np.eye(4)
        out[name] = (t1 @ np.linalg.inv(t0)).T.ravel().tolist()
    return out


def _anchor(name: str, nodes: dict, planar: dict) -> str | None:
    """The kinematic body a node rides: itself, else its ``rigid_with`` chain's first."""
    seen = set()
    cur: str | None = name
    while cur is not None and cur not in seen:
        if cur in planar:
            return cur
        seen.add(cur)
        node = nodes.get(cur)
        cur = (node.extras or {}).get("rigid_with") if node is not None else None
    return None
