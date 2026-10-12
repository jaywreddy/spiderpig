"""URDF of a fabricated arm (2026-10-12, phase 4): the design in ROS, MoveIt and RViz.

A URDF is a tree; a linkage is not (its parallelograms close loops). The export is the
tree that keeps most joints a function of one input, and says how to set the rest:

* **links**: one per rigid body of the planar side (a link, the frame) with everything
  riding it (its pins' parts, its drive's pulley, the tool's holder); the turret's static
  parts on ``robot_frame`` (the base), its turning ones on the frame's link (they turn
  with it); the wrist's on ``tool``. Each link's visual is one STL (millimetres, scaled),
  its inertial the parts' mass, centre and inertia (:func:`walk.body_masses`,
  :func:`hardware.mass.part_props`).
* **joints**: ``q1`` (the turntable, about the vertical) and ``q4`` (the wrist), each
  actuated input of the side (``q2``, ``q3``: the drives' links on their pivots), and a
  spanning tree of the side's other pins, chosen so as many as can be are linear in one
  input (Prim's on the number of inputs a joint's angle follows). A joint linear in one
  input is a URDF ``mimic`` of it; one linear in several, or not linear, is a plain
  revolute that the sidecar ``<name>.joints.json`` maps (its coefficients over every
  input, or a table over the box), with the loops' closing pins the tree left out.
* **frames**: ``base_link`` is ROS's (z up); ``robot_frame`` under it is the design's (y
  up, x forward: a fixed joint turns it), and every joint's zero is the pose fabricated
  (an input's URDF angle is its design angle less its value there: the sidecar's
  ``zero_rad``).

:func:`forward` evaluates the URDF itself (the tests check it against the linkage).
"""

from __future__ import annotations

import heapq
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from spiderpig import linkage
from spiderpig.config import BuildConfig
from spiderpig.kinematics import RobotTree, rigid_about
from spiderpig.mechanism import Mechanism

ROOT = "base_link"
FRAME = "robot_frame"
TOOL0 = "tool0"            # ROS-Industrial's name for the tool's tip frame
NOMINAL_RPM = 300.0        # a stepper's speed the joints' velocity limits assume
LINEAR_TOL = 1e-6          # a joint's angle within this of linear (rad) is linear
COEF_TOL = 1e-9


@dataclass
class UrdfJoint:
    name: str
    parent: str
    child: str
    origin: tuple[float, float, float]       # robot coordinates, mm, at the pose fabricated
    axis: tuple[float, float, float]
    kind: str = "revolute"                   # "revolute" | "fixed"
    limits: tuple[float, float] = (-math.pi, math.pi)
    mimic: tuple[str, float] | None = None   # (leader, multiplier)
    coef: dict[str, float] = field(default_factory=dict)   # rad per input rad
    linear: bool = True
    effort: float = 0.0                      # N·m (an actuated joint's: holding x ratio)
    velocity: float = 0.0                    # rad/s (NOMINAL_RPM through the ratio)


@dataclass
class UrdfModel:
    """The tree :func:`model` makes (module docstring): links (name -> bodies), joints
    parents first, each link's origin (robot mm), the inputs' values at the zero pose, and
    the pins the tree left out."""

    links: dict[str, list[str]]
    joints: list[UrdfJoint]
    origins: dict[str, np.ndarray]
    zero: dict[str, float]
    closures: list[tuple[str, str, str]]
    tables: dict[str, dict] = field(default_factory=dict)
    floor: np.ndarray = field(default_factory=lambda: np.zeros(3))   # base_link (robot mm)
    closure_xyz: list[np.ndarray] = field(default_factory=list)      # each closure's pin


def _angles(lk: linkage.Linkage, values: tuple[float, ...], inputs) -> dict[str, np.ndarray]:
    """Each link's angle (its first outline segment's) at the inputs given (rad)."""
    pts = linkage.LegSolution(1, 0.0, values, lk.key).at(*inputs)
    out = {}
    for name, (_, segs) in lk.links.items():
        a, b = segs[0]
        d = pts[b] - pts[a]
        out[name] = np.unwrap(np.arctan2(d[..., 1], d[..., 0]).ravel())
    return out


def _fit(rel: np.ndarray, x: np.ndarray, names: tuple[str, ...]) -> tuple[dict[str, float],
                                                                          float, bool]:
    """``rel ~ x @ coef + c``: the coefficients per input, the constant, whether exact."""
    a = np.column_stack([x, np.ones(len(rel))])
    sol, *_ = np.linalg.lstsq(a, rel, rcond=None)
    res = float(np.abs(a @ sol - rel).max()) if len(rel) else 0.0
    coef = {n: round(float(c), 9) for n, c in zip(names, sol[:-1], strict=True)
            if abs(c) > COEF_TOL}
    return coef, float(sol[-1]), res < LINEAR_TOL


def model(mech: Mechanism, config: BuildConfig, t: float) -> UrdfModel:
    """The URDF tree of ``mech`` (``config`` fabricated at ``t``): module docstring."""
    from spiderpig import walk

    lk = config.lk
    if lk.kind != "arm":
        raise ValueError(f"{lk.key} is a {lk.kind}: the URDF is an arm's (its inputs drive "
                         "links over ranges)")
    values = config.values
    by = {b.name: b for b in mech.bodies}
    tree = RobotTree.from_meta(mech.meta.get("tree"))
    carried = tree.carried if tree is not None else {}
    frame = next(b.name for b in mech.bodies if b.joints and b.name == "torso")
    kin = [b.name for b in mech.bodies if b.joints]
    hosted = {j.name for j in (tree.joints if tree else ()) if j.host is not None}
    links: dict[str, list[str]] = {FRAME: [], **{k: [] for k in kin}}
    if hosted:
        links["tool"] = []
    for b in mech.bodies:
        if b.part is None:
            continue
        j = carried.get(b.name)
        if j in hosted:
            links["tool"].append(b.name)
            continue
        anchor = walk.anchor_of(b, by)
        if anchor is not None:
            links[anchor].append(b.name)
        elif j is not None:
            links[frame].append(b.name)          # turns with the turntable: the frame's
        else:
            links[FRAME].append(b.name)          # the base
    # the inputs at the pose fabricated, and every link's angle over the box
    q0 = lk.tour(np.array([t]))
    zero = {n: float(v[0]) for n, v in zip(lk.inputs, q0, strict=True)}
    grid = lk.box(21)
    ang = _angles(lk, values, grid)
    ang0 = {k: float(v[0]) for k, v in _angles(lk, values, q0).items()}
    x = np.column_stack([np.ravel(g) for g in grid])
    names = tuple(lk.inputs)
    pos = {}
    for b in mech.bodies:
        for jn in b.joints:
            pos[(b.name, jn.name)] = np.asarray((b.pose @ jn.pose).matrix[:3, 3], dtype=float)

    def rel(child: str, parent: str):
        a = ang.get(child, np.zeros(len(x))) - ang.get(parent, np.zeros(len(x)))
        a0 = ang0.get(child, 0.0) - ang0.get(parent, 0.0)
        return a - 2 * math.pi * np.round((a - a0) / (2 * math.pi))

    # the side's pins: every pair of bodies on one axle (a joint of one name at one place)
    at: dict[str, list[str]] = {}
    for (_, pa, ja), (_, pb, jb) in mech.connections:
        for body, jn in ((pa, ja), (pb, jb)):
            if body in links and body not in at.setdefault(jn, []):
                at[jn].append(body)
    edges: dict[tuple[str, str], tuple[np.ndarray, str]] = {}
    for jn, bodies in at.items():
        for i, a in enumerate(bodies):
            for b in bodies[i + 1:]:
                if np.linalg.norm(pos[(a, jn)] - pos[(b, jn)]) < 1e-6:
                    edges.setdefault(_pair(a, b), (pos[(a, jn)], jn))
    drives = {(frame, body): q for q, (body, _) in lk.drives.items()}
    joints: list[UrdfJoint] = []
    origins: dict[str, np.ndarray] = {}
    tables: dict[str, dict] = {}
    closures: list[tuple[str, str, str]] = []
    closure_xyz: list[np.ndarray] = []
    # Prim from the frame: an actuated pin first, then by how many inputs it follows
    done = {frame}
    heap: list = []

    def push(u: str):
        for (a, b), (_p, jn) in edges.items():
            if u in (a, b):
                v = b if a == u else a
                if v in done:
                    continue
                q = drives.get((u, v))
                coef, _, lin = _fit(rel(v, u), x, names)
                w = -1 if q is not None else (len(coef) if lin else 99)
                heapq.heappush(heap, (w, u, v, jn))

    push(frame)
    while heap:
        _, u, v, _ = heapq.heappop(heap)
        if v in done:
            continue
        done.add(v)
        p, _ = edges[_pair(u, v)]
        origins[v] = p
        q = drives.get((u, v))
        r = rel(v, u)
        coef, _, lin = _fit(r, x, names)
        if q is not None:
            lo, hi = (math.radians(a) - zero[q] for a in lk.ranges[q])
            joints.append(UrdfJoint(q, u, v, tuple(p), (0.0, 0.0, 1.0), limits=(lo, hi),
                                    coef={q: 1.0}))
        else:
            name = f"{u}_{v}"
            r0 = ang0.get(v, 0.0) - ang0.get(u, 0.0)        # its angle at the zero pose
            span = (float(r.min()) - r0, float(r.max()) - r0)
            mimic = None
            if lin and len(coef) == 1:
                (lead, mult), = coef.items()
                mimic = (lead, mult)
            if not lin:
                tables[name] = {"inputs": {n: x[:, i].tolist() for i, n in enumerate(names)},
                                "angle_rad": (r - r0).tolist()}
            joints.append(UrdfJoint(name, u, v, tuple(p), (0.0, 0.0, 1.0),
                                    limits=(min(span[0], 0.0) - 1e-3, max(span[1], 0.0) + 1e-3),
                                    mimic=mimic, coef=coef, linear=lin))
        push(v)
    # a pin the tree doesn't keep: its two bodies not joined by tree joints on that axle
    for (a, b), (_, jn) in edges.items():
        group = {a}
        grew = True
        while grew:
            grew = False
            for j in joints:
                if (j.parent in group) != (j.child in group) and \
                        edges.get(_pair(j.parent, j.child), (None, ""))[1] == jn:
                    group |= {j.parent, j.child}
                    grew = True
        if b not in group:
            closures.append((a, b, jn))
            closure_xyz.append(edges[(a, b)][0])
    # the robot's own joints
    origins[frame] = np.zeros(3) if tree is None else np.asarray(tree.joint("q1").origin)
    head: list[UrdfJoint] = []
    if tree is not None:
        j1 = tree.joint("q1")
        head.append(UrdfJoint("q1", FRAME, frame, j1.origin, j1.axis,
                              limits=_radians(j1.range_deg)))
        for j in tree.joints:
            if j.host is not None:
                origins["tool"] = np.asarray(j.origin)
                joints.append(UrdfJoint(j.name, j.host, "tool", j.origin, j.axis,
                                        limits=_radians(j.range_deg)))
    else:
        head.append(UrdfJoint("frame_fixed", FRAME, frame, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                              kind="fixed"))
    origins[FRAME] = np.zeros(3)
    # the tool's tip: a frame for planning (ROS-Industrial's tool0), under the cup
    tool = mech.meta.get("tool")
    if tool:
        x, y, z = tool["axis"]
        tip = (x, y - float(tool["drop_mm"]), z)
        parent = "tool" if "tool" in links else tool["body"]
        links[TOOL0] = []
        origins[TOOL0] = np.asarray(tip)
        joints.append(UrdfJoint("tool0_fixed", parent, TOOL0, tip, (0.0, 0.0, 1.0),
                                kind="fixed"))
    # the actuated joints' effort and speed: each motor's holding torque and a nominal
    # speed through its reduction; a passive joint takes the most of them
    rated = _ratings(mech)
    for j in head + joints:
        if j.name in rated:
            j.effort, j.velocity = rated[j.name]
    top = (max((e for e, _ in rated.values()), default=1.0),
           max((v for _, v in rated.values()), default=1.0))
    for j in head + joints:
        if j.kind != "fixed" and j.name not in rated:
            j.effort, j.velocity = top
    floor = np.zeros(3)
    if tree is not None:          # base_link on the base's floor under the yaw axis
        info = mech.meta.get("turret", {})
        floor = np.asarray(info.get("axis", (0.0, 0.0, 0.0)), dtype=float)
    return UrdfModel(links, head + joints, origins, zero, closures, tables, floor, closure_xyz)


def _ratings(mech: Mechanism) -> dict[str, tuple[float, float]]:
    """``joint -> (effort N·m, velocity rad/s)`` of each motor-driven joint."""
    from spiderpig import actuators

    out = {}

    def rate(name: str, motor: str, ratio: float):
        m = actuators.STEPPERS.get(motor)
        if m is not None and ratio > 0:
            out[name] = (m.holding_nm * ratio, NOMINAL_RPM / 60.0 * 2 * math.pi / ratio)

    for d in (mech.meta.get("drive") or {}).values():
        rate(d["input"], d["motor"], float(d["ratio"]))
    tu = mech.meta.get("turret")
    if tu:
        rate("q1", tu["motor"], float(tu["ratio"]))
    tool = mech.meta.get("tool")
    if tool and tool.get("wrist"):
        rate("q4", tool["wrist"], 1.0)
    return out


def write(mech: Mechanism, config: BuildConfig, t: float, out: Path, name: str,
          props: dict | None = None) -> list[Path]:
    """``<out>/<name>.urdf``, its meshes under ``<out>/meshes/`` and the sidecar
    ``<name>.joints.json`` (module docstring). Returns the files written."""
    from build123d import Compound, Location

    from spiderpig import walk
    from spiderpig.hardware.mass import part_props
    from spiderpig.mesh import export_stl

    m = model(mech, config, t)
    out = Path(out)
    (out / "meshes").mkdir(parents=True, exist_ok=True)
    props = {} if props is None else props
    masses = walk.body_masses(mech, config, props)
    by = {b.name: b for b in mech.bodies}
    inert = {}            # link -> (grams, centre of mass (robot mm), inertia kg m² there)
    for link, bodies in m.links.items():
        grams = sum(masses[n][0] for n in bodies if n in masses)
        if grams <= 0:
            continue
        com = sum(masses[n][0] * masses[n][1] for n in bodies if n in masses) / grams
        inertia = np.zeros((3, 3))
        for n in bodies:
            if n not in masses:
                continue
            g, c = masses[n]
            p = props.get(n) or part_props(by[n].part)
            unit = np.asarray(p.inertia, dtype=float) * (g / abs(p.volume) if p.volume else 0.0)
            d = np.asarray(c) - com
            inertia += unit + g * (float(d @ d) * np.eye(3) - np.outer(d, d))
        inert[link] = (grams, com, inertia * 1e-9)            # g mm² -> kg m²
    root = ET.Element("robot", name=name)
    ET.SubElement(root, "link", name=ROOT)
    files = []
    for link, bodies in m.links.items():
        el = ET.SubElement(root, "link", name=link)
        o = m.origins.get(link, np.zeros(3))
        parts = [pp for n in bodies if (pp := by[n].placed_part()) is not None]
        if parts:
            stl = out / "meshes" / f"{link}.stl"
            comp = Compound(children=[p.moved(Location(tuple(-o))) for p in parts])
            export_stl(comp, stl)
            files.append(stl)
            vis = ET.SubElement(el, "visual")
            ET.SubElement(vis, "origin", xyz="0 0 0", rpy="0 0 0")
            geo = ET.SubElement(vis, "geometry")
            ET.SubElement(geo, "mesh", filename=f"meshes/{link}.stl", scale="0.001 0.001 0.001")
        if link in inert:
            grams, com, inertia = inert[link]
            ine = ET.SubElement(el, "inertial")
            ET.SubElement(ine, "origin", xyz=_m(com - o), rpy="0 0 0")
            ET.SubElement(ine, "mass", value=f"{grams / 1000.0:.6g}")
            ET.SubElement(ine, "inertia", ixx=f"{inertia[0, 0]:.6g}", ixy=f"{inertia[0, 1]:.6g}",
                          ixz=f"{inertia[0, 2]:.6g}", iyy=f"{inertia[1, 1]:.6g}",
                          iyz=f"{inertia[1, 2]:.6g}", izz=f"{inertia[2, 2]:.6g}")
    fix = ET.SubElement(root, "joint", name="design_frame", type="fixed")
    ET.SubElement(fix, "parent", link=ROOT)
    ET.SubElement(fix, "child", link=FRAME)
    fl = m.floor                    # the design's origin seen from the floor, in ROS's axes
    ET.SubElement(fix, "origin", xyz=_m((-fl[0], fl[2], -fl[1])), rpy=f"{math.pi / 2:.9f} 0 0")
    for j in m.joints:
        el = ET.SubElement(root, "joint", name=j.name, type=j.kind)
        ET.SubElement(el, "parent", link=j.parent)
        ET.SubElement(el, "child", link=j.child)
        o = np.asarray(j.origin) - m.origins.get(j.parent, np.zeros(3))
        ET.SubElement(el, "origin", xyz=_m(o), rpy="0 0 0")
        if j.kind != "fixed":
            ET.SubElement(el, "axis", xyz=" ".join(f"{v:g}" for v in j.axis))
            ET.SubElement(el, "limit", lower=f"{j.limits[0]:.6f}", upper=f"{j.limits[1]:.6f}",
                          effort=f"{j.effort:.4g}", velocity=f"{j.velocity:.4g}")
            if j.mimic is None and j.coef and set(j.coef) != {j.name}:
                el.append(ET.Comment(f" passive: follows {', '.join(sorted(j.coef))}; set it "
                                     "from the sidecar's coefficients, not a free joint "))
        if j.mimic is not None:
            ET.SubElement(el, "mimic", joint=j.mimic[0], multiplier=f"{j.mimic[1]:.9g}",
                          offset="0")
    for a, b, jn in m.closures:
        root.append(ET.Comment(f" closes a loop: {a} and {b} meet at {jn} (spiderpig's "
                               f"joints.json) "))
    ET.indent(root)
    path = out / f"{name}.urdf"
    path.write_text(ET.tostring(root, encoding="unicode") + "\n")
    side = {"zero_rad": m.zero, "units": "URDF angles are design angles less zero_rad",
            "base_link": "on the base's floor under the yaw axis (ROS axes: z up, x forward)",
            "joints": {j.name: {"parent": j.parent, "child": j.child,
                                "coefficients": j.coef, "linear": j.linear,
                                "mimic": list(j.mimic) if j.mimic else None}
                       for j in m.joints if j.kind != "fixed"},
            "closures": [{"bodies": [a, b], "joint": jn} for a, b, jn in m.closures],
            "tables": m.tables}
    js = out / f"{name}.joints.json"
    js.write_text(json.dumps(side, indent=1))
    mj = out / f"{name}.mjcf.xml"
    mj.write_text(mjcf(m, inert, name, {p.stem for p in files}))
    return [path, js, mj, *files]


KP = 200.0          # the MJCF's position actuators' stiffness (N·m/rad): the motors holding


def mjcf(m: UrdfModel, inert: dict, name: str, meshes: set[str]) -> str:
    """The arm in MuJoCo from the same tree (the robot's coordinates, y up, metres): a body
    per link nested as the URDF's joints nest them, a hinge each, the loops closed by
    ``connect`` equalities at their pins (so the passive joints follow the inputs as the
    linkage does), a position actuator on each actuated joint (its motor holding), the
    tool's tip a site ``tool0``; the links' meshes visual only (no contacts)."""
    root = ET.Element("mujoco", model=name)
    ET.SubElement(root, "compiler", angle="radian", meshdir="meshes")
    ET.SubElement(root, "option", gravity="0 -9.81 0", timestep="0.0005")
    asset = ET.SubElement(root, "asset")
    for link in sorted(meshes):
        ET.SubElement(asset, "mesh", name=link, file=f"{link}.stl", scale="0.001 0.001 0.001")
    world = ET.SubElement(root, "worldbody")
    if FRAME in meshes:
        ET.SubElement(world, "geom", type="mesh", mesh=FRAME, contype="0", conaffinity="0")
    children: dict[str, list[UrdfJoint]] = {}
    for j in m.joints:
        children.setdefault(j.parent, []).append(j)

    def body(parent_el, j: UrdfJoint):
        o = m.origins.get(j.child, np.asarray(j.origin))
        rel = (o - m.origins.get(j.parent, np.zeros(3))) / 1000.0
        el = ET.SubElement(parent_el, "body", name=j.child,
                           pos=" ".join(f"{v:.9g}" for v in rel))
        if j.kind != "fixed":
            jo = (np.asarray(j.origin) - o) / 1000.0
            ET.SubElement(el, "joint", name=j.name, type="hinge",
                          axis=" ".join(f"{v:g}" for v in j.axis),
                          pos=" ".join(f"{v:.9g}" for v in jo), damping="0.01")
        if j.child in inert:
            g, com, ine = inert[j.child]
            ET.SubElement(el, "inertial", pos=" ".join(f"{v:.9g}" for v in (com - o) / 1000.0),
                          mass=f"{g / 1000.0:.6g}",
                          fullinertia=" ".join(f"{v:.6g}" for v in (
                              ine[0, 0], ine[1, 1], ine[2, 2], ine[0, 1], ine[0, 2], ine[1, 2])))
        else:
            ET.SubElement(el, "inertial", pos="0 0 0", mass="1e-4",
                          diaginertia="1e-9 1e-9 1e-9")
        if j.child in meshes:
            ET.SubElement(el, "geom", type="mesh", mesh=j.child, contype="0", conaffinity="0")
        if j.child == TOOL0:
            ET.SubElement(el, "site", name=TOOL0, pos="0 0 0", size="0.002")
        for k in children.get(j.child, []):
            body(el, k)

    for j in children.get(FRAME, []):
        body(world, j)
    eq = ET.SubElement(root, "equality")
    for (a, b, jn), xyz in zip(m.closures, m.closure_xyz, strict=True):
        anchor = (np.asarray(xyz) - m.origins.get(a, np.zeros(3))) / 1000.0
        ET.SubElement(eq, "connect", name=f"pin_{jn}_{a}_{b}", body1=a, body2=b,
                      anchor=" ".join(f"{v:.9g}" for v in anchor))
    act = ET.SubElement(root, "actuator")
    for j in m.joints:
        if j.kind != "fixed" and j.mimic is None and set(j.coef) <= {j.name} and j.effort > 0 \
                and j.name in {*m.zero, "q1", "q4"}:
            ET.SubElement(act, "position", name=j.name, joint=j.name, kp=f"{KP:g}",
                          ctrlrange=f"{j.limits[0]:.6f} {j.limits[1]:.6f}", ctrllimited="true")
    ET.indent(root)
    return ET.tostring(root, encoding="unicode") + "\n"


def _pair(a: str, b: str) -> tuple[str, str]:
    return (a, b) if a <= b else (b, a)


def _radians(deg: tuple[float, float]) -> tuple[float, float]:
    return (math.radians(deg[0]), math.radians(deg[1]))


def _m(v) -> str:
    return " ".join(f"{float(c) / 1000.0:.9g}" for c in v)


# -- evaluating it -------------------------------------------------------------------------


def forward(model_: UrdfModel, inputs: dict[str, float], extra: dict[str, float] | None = None
            ) -> dict[str, np.ndarray]:
    """Each link's 4x4 pose (robot coordinates, mm) for the inputs' design values (rad)
    and the robot joints' angles (``extra``: ``q1``, ``q4``), every passive joint from its
    coefficients (a non-linear one is not evaluated: :class:`ValueError`)."""
    extra = extra or {}
    d = {n: inputs[n] - model_.zero[n] for n in model_.zero}
    pose = {FRAME: np.eye(4)}
    for j in model_.joints:
        if j.kind == "fixed":
            v = 0.0
        elif j.name in extra:
            v = extra[j.name]
        elif j.name in d:
            v = d[j.name]
        else:
            if not j.linear:
                raise ValueError(f"{j.name} is not linear in the inputs")
            v = sum(c * d[n] for n, c in j.coef.items())
        r = rigid_about(j.axis, j.origin, [v])[0]
        pose[j.child] = pose[j.parent] @ r
    return pose
