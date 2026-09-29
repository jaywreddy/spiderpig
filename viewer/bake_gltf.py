"""Bake the walker as one self-contained, animated ``.glb`` for the three.js viewer.

Modes
-----
``robot`` (default)
    The whole robot (:mod:`construction.robot`): two mirror-image sides,
    bodies prefixed ``L.`` / ``R.``, plus the chassis between the servos.
    ``module`` picks the side (``quad`` by default).
``single`` / ``double`` / ``decker`` / ``quad``
    One side only (:data:`fabricate.MODULES`).

Pipeline
--------
1. Fabricate the walker at the reference crank angle ``t = 0``
   (:func:`fabricate.fabricate`): every part is modelled in world
   coordinates at that angle.
2. Tessellate one mesh per congruence group. Bodies of one class
   (:func:`stack.body_class`: ``L.b1_leg2`` -> ``b1``) share a mesh when a
   planar rigid motion plus a Z shift maps one part onto the other; this is
   *verified* from exact B-rep mass properties, so a right-side part that
   isn't symmetric about its own mid-plane (a stepped pin, the servo) gets
   its own mesh while mirrored link plates (plain extrusions) share.
3. Pack geometry, one material per fabrication kind (laser acrylic, printed,
   purchased servo / metal).
4. Sample the kinematic template (both sides for ``robot``) over one crank
   revolution and fit each body's planar rigid motion to its joints;
   hardware follows its ``rigid_with`` host.
5. Emit a root node ``walker`` that stands the robot on its feet (model +Y
   -> up, the layer stack horizontal, lowest foot on ``z = 0``) with one
   animated child node per body (TRS channels, LINEAR).
6. Foot-path overlay in ``scene.extras`` (a mechanism's, one side only:
   ``output_path`` and its ``output`` spec); for the robot, the walking
   model's data (:mod:`walk`) in the root node's extras under ``"drive"``.
7. Serialize with ``pygltflib``.

Design parameters
-----------------
``linkage`` (a registered :class:`linkage.Linkage`, Klann by default),
``phases`` (one crank phase per leg) and ``proportions`` (overrides of the
linkage's parameters) change the design; the CLI takes ``--linkage``, the
phases in degrees (``--phases 0,180,90,270``) and ``--proportion
NAME=VALUE`` (repeatable). A non-default design is written to
``viewer/data/params/<mode>_<module>_<key>.glb`` unless ``--out`` says
otherwise (:func:`param_glb`; the dev server caches parameter bakes there too).

Usage
-----
    uv run python viewer/bake_gltf.py                       # robot, quad per side
    uv run python viewer/bake_gltf.py --mode robot --module single
    uv run python viewer/bake_gltf.py --mode double --frames 60
    uv run python viewer/bake_gltf.py --phases 0,175,180,355 --proportion DF=2.5
    uv run python viewer/bake_gltf.py --linkage jansen --module double
"""

from __future__ import annotations

import argparse
import hashlib
import logging
import math
import sys
import time
from collections import defaultdict
from collections.abc import Mapping, Sequence
from contextlib import contextmanager
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

import numpy as np
import pygltflib

logger = logging.getLogger("bake_gltf")


@dataclass
class _Profiler:
    """Lightweight perf recorder: nested wall-clock timers + counters + metrics.

    Durations are accumulated per label so the same bracket can be entered
    many times (e.g. once per animation frame) and reported as total / mean /
    p50 / p95.
    """

    enabled: bool = True
    _durations: dict[str, list[float]] = field(
        default_factory=lambda: defaultdict(list)
    )
    _counters: dict[str, int] = field(default_factory=lambda: defaultdict(int))
    _metrics: dict[str, float] = field(default_factory=dict)

    @contextmanager
    def timed(self, label: str):
        if not self.enabled:
            yield
            return
        start = time.perf_counter()
        try:
            yield
        finally:
            self._durations[label].append(time.perf_counter() - start)

    def bump(self, label: str, n: int = 1) -> None:
        if self.enabled:
            self._counters[label] += n

    def set_metric(self, key: str, value: float) -> None:
        if self.enabled:
            self._metrics[key] = float(value)

    def log_summary(self) -> None:
        if not self.enabled:
            return

        bake_total = sum(self._durations.get("bake_total", [])) or None

        rows = []
        for label, times_ in self._durations.items():
            n = len(times_)
            total = sum(times_)
            mean_ms = (total / n) * 1000.0 if n else 0.0
            ts = sorted(times_)
            p50 = ts[n // 2] * 1000.0 if n else 0.0
            p95 = ts[min(n - 1, int(n * 0.95))] * 1000.0 if n else 0.0
            pct = (total / bake_total * 100.0) if bake_total else 0.0
            rows.append((label, n, total, mean_ms, p50, p95, pct))
        rows.sort(key=lambda r: -r[2])

        lines = [
            "bake profile summary:",
            f"  {'label':40} {'calls':>6} {'total_s':>9} "
            f"{'mean_ms':>9} {'p50_ms':>9} {'p95_ms':>9} {'%bake':>6}",
            f"  {'-' * 40} {'-' * 6} {'-' * 9} {'-' * 9} "
            f"{'-' * 9} {'-' * 9} {'-' * 6}",
        ]
        for label, n, total, mean_ms, p50, p95, pct in rows:
            lines.append(
                f"  {label:40} {n:6d} {total:9.3f} "
                f"{mean_ms:9.3f} {p50:9.3f} {p95:9.3f} {pct:6.1f}"
            )
        if self._counters:
            lines.append("  counters:")
            for k, v in sorted(self._counters.items()):
                lines.append(f"    {k}: {v}")
        if self._metrics:
            lines.append("  metrics:")
            for k, v in sorted(self._metrics.items()):
                # Integers stay integers for readability (vert counts, bytes).
                if v == int(v):
                    lines.append(f"    {k}: {int(v)}")
                else:
                    lines.append(f"    {k}: {v:.3f}")

        logger.info("\n".join(lines))

# Make sibling modules importable when invoked as ``viewer/bake_gltf.py``.
_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import linkage as linkage_mod  # noqa: E402
import walk  # noqa: E402
from construction.robot import robot_template  # noqa: E402
from fabricate import MODULES, BuildConfig, design_side, fabricate, template_for  # noqa: E402
from mechanism import Body, Mechanism, MechanismTemplate  # noqa: E402
from stack import body_class, is_link  # noqa: E402

# One side's kinematics per module (see fabricate.MODULES) is
# ``fabricate.template_for(config)``: the module at the config's crank phases
# and proportions.
ROBOT = "robot"
MODES = (ROBOT, *MODULES)
DEFAULT_MODULE = "quad"
PARAMS_DIR = _REPO_ROOT / "viewer" / "data" / "params"

# Crank angle the parts are modelled at; frame 0 of the animation.
_T_REF = 0.0

# Model -> viewer: the linkage moves in XY with +Y up and the layer stack
# along Z. The root node turns +90 deg about X: +Y -> +Z (up), +Z -> -Y.
_ROOT_ROTATION = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))   # xyzw


def _resolve(mode: str, module: str | None = None,
             robot_module: str = DEFAULT_MODULE) -> tuple[str, bool]:
    """``(side module, robot?)`` for a bake mode (:func:`build_config` checks the module);
    the robot's module is ``robot_module`` unless ``module`` says otherwise."""
    if mode == ROBOT:
        return module or robot_module, True
    if mode not in MODULES:
        raise ValueError(f"unknown mode {mode!r}; have {list(MODES)}")
    if module not in (None, mode):
        raise ValueError(f"mode {mode!r} is one side; module {module!r} applies to robot only")
    return mode, False


def _build_template(config: BuildConfig) -> MechanismTemplate:
    """The kinematics the animation samples: one side, or both sides of the robot."""
    tmpl = template_for(config)
    return robot_template(tmpl) if config.robot else tmpl


def build_config(
    mode: str, module: str | None = None, config: BuildConfig | None = None,
    thickness: float | None = None, phases: Sequence[float] | None = None,
    proportions: Mapping[str, float] | None = None, linkage: str | None = None,
) -> BuildConfig:
    """The build for a bake. ``linkage``, ``phases`` (rad, one per leg) and
    ``proportions`` override ``config``'s; they are validated and normalized
    (:func:`walk.normalize_phases`, :func:`walk.normalize_proportions`), so a
    default design has one config however it was asked for.

    Raises ``ValueError`` (:class:`walk.ParamError` for bad design parameters).
    """
    config = config or BuildConfig()
    side, robot = _resolve(mode, module, config.module)     # a robot keeps config's module
    config = replace(config, module=side, robot=robot)
    if linkage is not None:
        config = replace(config, linkage=walk.get_linkage(linkage).key)
    if robot and walk.get_linkage(config.linkage).kind != "walker":
        raise walk.ParamError(f"{config.linkage} is a mechanism: bake one side (mode single), "
                              f"not a robot")
    if phases is not None:
        config = replace(config, phases=tuple(float(p) for p in phases))
    if proportions is not None:
        config = replace(config, proportions=tuple(dict(proportions).items()))
    config = replace(
        config,
        phases=walk.normalize_phases(side, config.phases, degrees=False,
                                     linkage=config.linkage),
        proportions=walk.normalize_proportions(dict(config.proportions), config.linkage),
    )
    return config if thickness is None else replace(config, thickness=thickness)


def config_key(config: BuildConfig) -> str:
    """A short stable hash of a (normalized) build config: names parameter bakes."""
    return hashlib.sha1(repr(config).encode()).hexdigest()[:12]


def is_default(mode: str, config: BuildConfig) -> bool:
    """Is ``config`` what a plain bake of ``mode`` builds?"""
    return config == build_config(mode)


def param_glb(mode: str, config: BuildConfig, root: Path | None = None) -> Path:
    """Where a bake of ``mode`` with a non-default ``config`` is cached."""
    return (root or PARAMS_DIR) / f"{mode}_{config.module}_{config_key(config)}.glb"


def _build_assembly(
    mode: str,
    *,
    t: float,
    module: str | None = None,
    config: BuildConfig | None = None,
    thickness: float | None = None,
    phases: Sequence[float] | None = None,
    proportions: Mapping[str, float] | None = None,
    linkage: str | None = None,
) -> Mechanism:
    """The fabricated walker at crank angle ``t`` (parts in world coordinates)."""
    config = build_config(mode, module, config, thickness, phases, proportions, linkage)
    return fabricate(template_for(config), config, t)


# ---------------------------------------------------------------------------
# Materials: one per fabrication kind
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _Material:
    rgba: tuple[float, float, float, float]
    metallic: float
    roughness: float


_MATERIALS: dict[str, _Material] = {
    # laser-cut leg links: clear acrylic with a cool tint, see-through
    "acrylic": _Material((0.42, 0.74, 0.96, 0.42), 0.0, 0.06),
    # laser-cut frame and chassis plates: orange-tinted acrylic
    "acrylic_frame": _Material((1.00, 0.34, 0.05, 0.70), 0.0, 0.08),
    # FDM prints: axles, pins, crankshaft
    "printed": _Material((0.52, 0.30, 0.95, 1.0), 0.0, 0.6),
    # purchased: the servo body, and metal hardware (screws, horns)
    "servo": _Material((0.05, 0.05, 0.06, 1.0), 0.1, 0.45),
    "metal": _Material((0.78, 0.79, 0.81, 1.0), 0.85, 0.30),
    # anything without a ``fab`` (shouldn't happen)
    "other": _Material((0.55, 0.55, 0.55, 1.0), 0.0, 0.7),
}


def _catalog_category(bom_key: str | None) -> str | None:
    if not bom_key:
        return None
    from hardware.catalog import get

    try:
        return get(bom_key).category
    except KeyError:
        return None


def _material_of(body: Body) -> str:
    """Material key for a body: by ``fab``, then by role."""
    cls = body_class(body.name)
    if body.fab == "laser":
        return "acrylic" if is_link(body.name) else "acrylic_frame"
    if body.fab == "printed":
        return "printed"
    if body.fab == "purchased":
        servo = _catalog_category(body.bom_key) == "servo" or cls == "servo"
        return "servo" if servo else "metal"
    return "other"


# ---------------------------------------------------------------------------
# Planar rigid motions
# ---------------------------------------------------------------------------


def _body_joint_world(body) -> dict[str, np.ndarray]:
    """World-space XYZ of each joint on ``body`` under its current pose."""
    return {
        j.name: np.asarray((body.pose @ j.pose).matrix[:3, 3], dtype=float)
        for j in body.joints
    }


def _quat_z(theta: np.ndarray) -> np.ndarray:
    """``(T, 4)`` xyzw quaternions for rotations by ``theta`` about Z."""
    half = np.asarray(theta, dtype=float) / 2.0
    zeros = np.zeros_like(half)
    return np.stack([zeros, zeros, np.sin(half), np.cos(half)], axis=-1)


def _quat_hemisphere_continuous(q: np.ndarray) -> np.ndarray:
    """Flip signs of ``(T, 4)`` quaternions so adjacent samples stay in the
    same hemisphere (avoids the 2π ambiguity during LINEAR interpolation)."""
    q = q.copy()
    for i in range(1, q.shape[0]):
        if float(np.dot(q[i - 1], q[i])) < 0.0:
            q[i] = -q[i]
    return q


def _rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


# ---------------------------------------------------------------------------
# Mesh sharing: congruence under a planar rigid motion plus a Z shift
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _MassProps:
    """Exact B-rep invariants of a part (volume and surface properties)."""

    volume: float
    area: float
    com: np.ndarray        # volume centroid (3,)
    surf_com: np.ndarray   # surface centroid (3,)
    inertia: np.ndarray    # (3, 3) about the volume centroid
    z_range: tuple[float, float]


def _mass_props(part) -> _MassProps:
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    vol, surf = GProp_GProps(), GProp_GProps()
    BRepGProp.VolumeProperties_s(part.wrapped, vol)
    BRepGProp.SurfaceProperties_s(part.wrapped, surf)
    c, sc = vol.CentreOfMass(), surf.CentreOfMass()
    m = vol.MatrixOfInertia()
    bb = part.bounding_box()
    return _MassProps(
        volume=vol.Mass(), area=surf.Mass(),
        com=np.array([c.X(), c.Y(), c.Z()]), surf_com=np.array([sc.X(), sc.Y(), sc.Z()]),
        inertia=np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]),
        z_range=(bb.min.Z, bb.max.Z),
    )


@dataclass(frozen=True)
class _Planar:
    """``x -> R(theta) x + (tx, ty, dz)``: a planar rigid motion plus a Z shift."""

    theta: float = 0.0
    txy: tuple[float, float] = (0.0, 0.0)
    dz: float = 0.0

    def apply(self, p: np.ndarray) -> np.ndarray:
        p = np.asarray(p, dtype=float)
        xy = p[..., :2] @ _rot2(self.theta).T + np.asarray(self.txy)
        return np.concatenate([xy, p[..., 2:3] + self.dz], axis=-1)


_LEN_TOL = 1e-3    # mm
_REL_TOL = 1e-5


def _congruent(a: _MassProps, b: _MassProps, g: _Planar) -> bool:
    """Does ``g`` (with its ``dz``) map the part with props ``a`` onto ``b``?

    Volume, area, both centroids, the inertia tensor and the Z extent must
    all agree. The Z extent against the centroids is what rejects a Z-mirror
    of a part that isn't symmetric about its own mid-plane (a stepped pin
    has the same inertia either way up, but its centroid sits nearer its head).
    """
    if not math.isclose(a.volume, b.volume, rel_tol=_REL_TOL, abs_tol=1e-9):
        return False
    if not math.isclose(a.area, b.area, rel_tol=_REL_TOL, abs_tol=1e-9):
        return False
    if np.abs(g.apply(a.com) - b.com).max() > _LEN_TOL:
        return False
    if np.abs(g.apply(a.surf_com) - b.surf_com).max() > _LEN_TOL:
        return False
    if abs(a.z_range[0] + g.dz - b.z_range[0]) > _LEN_TOL:
        return False
    if abs(a.z_range[1] + g.dz - b.z_range[1]) > _LEN_TOL:
        return False
    r = np.eye(3)
    r[:2, :2] = _rot2(g.theta)
    scale = max(np.abs(a.inertia).max(), 1e-12)
    return bool(np.abs(r @ a.inertia @ r.T - b.inertia).max() <= _REL_TOL * scale)


def _fit_ref(a: dict[str, np.ndarray], b: dict[str, np.ndarray]) -> _Planar | None:
    """Planar motion taking joints ``a`` onto the same-named joints ``b``, if exact."""
    if set(a) != set(b) or not a:
        return None
    names = sorted(a)
    p0 = np.stack([a[n] for n in names])[None]
    p1 = np.stack([b[n] for n in names])[None]
    theta, trans = walk.planar_fit(p0, p1)
    g = _Planar(float(theta[0]), (float(trans[0, 0]), float(trans[0, 1])))
    if np.abs(g.apply(p0[0])[:, :2] - p1[0][:, :2]).max() > 1e-4:
        return None
    return g


@dataclass
class _MeshPlan:
    """Which mesh each body instances, and how the mesh sits on the body at ``t_ref``."""

    key_of: dict[str, str]          # body -> mesh key
    rep_of: dict[str, Body]         # mesh key -> the body whose part is tessellated
    place: dict[str, _Planar]       # body -> motion taking the mesh onto its part


def _plan_meshes(bodies: list[Body], anchors: dict[str, dict[str, np.ndarray]],
                 owner: dict[str, str | None], prof: _Profiler,
                 props: dict[str, _MassProps] | None = None) -> _MeshPlan:
    """Group bodies into shared meshes (see :func:`_congruent`).

    ``props`` caches each body's mass properties (filled as they're needed).
    """
    by_class: dict[str, list[Body]] = defaultdict(list)
    for b in bodies:
        if b.part is not None:
            by_class[body_class(b.name)].append(b)
    plan = _MeshPlan({}, {}, {})
    props = {} if props is None else props

    def props_of(b: Body) -> _MassProps:
        if b.name not in props:
            props[b.name] = _mass_props(b.part)
        return props[b.name]

    for cls, members in by_class.items():
        reps: list[tuple[str, Body]] = []
        for b in members:
            placed = None
            for key, rep in reps:
                oa, ob = owner[rep.name], owner[b.name]
                if oa is None or ob is None or body_class(oa) != body_class(ob):
                    continue
                g = _fit_ref(anchors[oa], anchors[ob])
                if g is None:
                    continue
                pa, pb = props_of(rep), props_of(b)
                g = replace(g, dz=float(pb.com[2] - pa.com[2]))
                if _congruent(pa, pb, g):
                    placed = (key, g)
                    break
            if placed is None:
                key = cls if not reps else b.name
                while key in plan.rep_of:
                    key = f"{key}~"
                reps.append((key, b))
                plan.rep_of[key] = b
                placed = (key, _Planar())
            else:
                prof.bump("mesh_shared")
            plan.key_of[b.name], plan.place[b.name] = placed
    return plan


# ---------------------------------------------------------------------------
# glTF packing
# ---------------------------------------------------------------------------


def _tessellate(part, tolerance: float = 0.1) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return ``(positions (nv,3) float32, normals (nv,3) float32, indices (nt*3,) uint32)``.

    Normals are smooth-shaded (per-vertex average of adjacent face normals).
    Three.js can override to flat shading client-side if desired.
    """
    verts, tris = part.tessellate(tolerance=tolerance)
    positions = np.array([(v.X, v.Y, v.Z) for v in verts], dtype=np.float32)
    triangles = np.array(tris, dtype=np.uint32)

    # Per-face normal via cross product.
    p0 = positions[triangles[:, 0]]
    p1 = positions[triangles[:, 1]]
    p2 = positions[triangles[:, 2]]
    face_n = np.cross(p1 - p0, p2 - p0)
    face_len = np.linalg.norm(face_n, axis=1, keepdims=True)
    face_n = np.where(face_len > 0, face_n / np.where(face_len > 0, face_len, 1.0), 0.0)

    # Accumulate per-vertex.
    normals = np.zeros_like(positions)
    for i in range(3):
        np.add.at(normals, triangles[:, i], face_n)
    vlen = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.where(vlen > 0, normals / np.where(vlen > 0, vlen, 1.0), 0.0)

    return positions.astype(np.float32), normals.astype(np.float32), triangles.flatten()


class _Packer:
    """Append-only binary blob that hands out :class:`pygltflib.BufferView` indices."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.buffer_views: list[pygltflib.BufferView] = []

    def add(self, data: bytes, *, target: int | None = None) -> int:
        # glTF requires 4-byte-aligned bufferViews.
        pad = (4 - (len(self._buf) % 4)) % 4
        if pad:
            self._buf.extend(b"\x00" * pad)
        byte_offset = len(self._buf)
        self._buf.extend(data)
        bv = pygltflib.BufferView(
            buffer=0, byteOffset=byte_offset, byteLength=len(data)
        )
        if target is not None:
            bv.target = target
        self.buffer_views.append(bv)
        return len(self.buffer_views) - 1

    @property
    def bytes(self) -> bytes:
        return bytes(self._buf)


def _accessor_for(
    bv_index: int,
    count: int,
    *,
    component_type: int,
    accessor_type: str,
    min_vals: list[float] | None = None,
    max_vals: list[float] | None = None,
) -> pygltflib.Accessor:
    acc = pygltflib.Accessor(
        bufferView=bv_index,
        componentType=component_type,
        count=count,
        type=accessor_type,
    )
    if min_vals is not None:
        acc.min = [float(v) for v in min_vals]
    if max_vals is not None:
        acc.max = [float(v) for v in max_vals]
    return acc


def _gltf_material(name: str) -> pygltflib.Material:
    m = _MATERIALS[name]
    translucent = m.rgba[3] < 1.0
    return pygltflib.Material(
        name=name,
        pbrMetallicRoughness=pygltflib.PbrMetallicRoughness(
            baseColorFactor=list(m.rgba), metallicFactor=m.metallic, roughnessFactor=m.roughness,
        ),
        alphaMode=pygltflib.BLEND if translucent else pygltflib.OPAQUE,
        doubleSided=True,
    )


def _json_meta(meta: dict) -> dict:
    return {k: v for k, v in meta.items() if isinstance(v, (str, int, float, bool))}


def _drive_extra(config: BuildConfig, mech: Mechanism,
                 motion: dict[str, tuple[np.ndarray, np.ndarray]],
                 owner: dict[str, str | None], props: dict[str, _MassProps],
                 duration_s: float) -> dict:
    """The root node's ``drive`` extras: :func:`walk.drive_extra` for the fabricated robot.

    Feet z from the side's layer plan; the centre of mass is every part's
    mass at its centroid (:func:`walk.body_masses`, reusing the mass
    properties the mesh sharing measured) averaged over the animation's
    samples (``motion``: each anchor's planar motion per frame).
    """
    design = design_side(template_for(config), config)       # cached by fabricate
    by_name = {b.name: b for b in mech.bodies}

    def volume_centroid(body: Body) -> tuple[float, np.ndarray]:
        if body.name not in props:
            props[body.name] = _mass_props(by_name[body.name].part)
        p = props[body.name]
        return p.volume, p.com

    masses = walk.body_masses(mech, config, volume_centroid=volume_centroid)
    com, mass = walk.cycle_com(masses, motion, owner)
    model = walk.walker(config, feet_z=walk.foot_z_planned(config, design), com=com,
                        mass_g=mass)
    return walk.drive_extra(model, duration_s, metrics=walk.straight_walk_metrics(model))


def bake_gltf(
    out: Path,
    *,
    n_frames: int = 120,
    duration_s: float = 1.0,
    mode: str = ROBOT,
    module: str | None = None,
    thickness: float | None = None,
    config: BuildConfig | None = None,
    phases: Sequence[float] | None = None,
    proportions: Mapping[str, float] | None = None,
    linkage: str | None = None,
    verbose: bool = False,
    profile: bool = True,
    cprofile_out: Path | None = None,
) -> None:
    """Write ``<out>``: the fabricated walker and its animation over one crank revolution.

    ``mode`` is ``"robot"`` (both sides; ``module`` picks the side, ``quad``
    by default) or a side-only module (``single``, ``double``, ``decker``,
    ``quad``). ``config`` sets the build (sheet, servo, constructions); its
    ``module`` / ``robot`` fields follow ``mode``. ``linkage`` (a key of
    :func:`linkage.available`), ``phases`` (radians, one per leg, like
    ``BuildConfig.phases``) and ``proportions`` (overrides of the linkage's
    parameters) set the design; they override ``config``'s.
    Bad parameters raise ``ValueError``; so does a layout the planner can't
    find (:mod:`stack`), and a construction that can't be built raises
    :class:`construction.ConstructionError`.

    A robot's root node ``walker`` carries the walking model's data in its
    extras under ``"drive"`` (:func:`walk.drive_extra`): every foot's path
    (theta_i = 2 pi i / 360, animation time ``tau = theta / 2 pi *
    duration_s``) with its lateral z from the layer plan, the cycle-mean
    centre of mass and total mass of the fabricated parts, the clip
    duration, the servo's key and speed, the design parameters and the
    straight-walk metrics.

    Profiling
    ---------
    When ``profile`` is true, per-stage wall-clock times, call counts and
    output-size metrics are emitted via the ``bake_gltf`` logger at INFO
    level. Set ``cprofile_out`` to a path to additionally dump a
    ``cProfile`` .prof file (plus a ``<path>.txt`` of the top-30 cumulative
    hot functions) for deep dives.
    """
    config = build_config(mode, module, config, thickness, phases, proportions, linkage)
    robot = config.robot
    lk = linkage_mod.get(config.linkage)
    if verbose and logger.level > logging.DEBUG:
        logger.setLevel(logging.DEBUG)

    prof = _Profiler(enabled=profile)

    pr = None
    if cprofile_out is not None:
        import cProfile  # noqa: PLC0415
        pr = cProfile.Profile()
        pr.enable()

    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)

    logger.info(
        "bake: mode=%s linkage=%s module=%s phases=%s proportions=%s n_frames=%d "
        "duration_s=%.3f out=%s",
        mode, config.linkage, config.module,
        "default" if config.phases is None
        else ",".join(f"{math.degrees(p):g}" for p in config.phases),
        dict(config.proportions) or "default", n_frames, duration_s, out,
    )
    prof.set_metric("n_frames", n_frames)

    linkage_mod._BAKE_PROFILER = prof if profile else None
    try:
        with prof.timed("bake_total"):
            # --- stage 1: the fabricated walker at t_ref ---
            with prof.timed("1_reference_build"):
                ref_mech = fabricate(template_for(config), config, _T_REF)
            bodies = ref_mech.bodies
            by_name = {b.name: b for b in bodies}
            # (body, joint): lk.feet per leg, or a mechanism's output point
            feet = linkage_mod.feet_of(ref_mech) or [(lk.output.link, lk.output.point)]
            prof.set_metric("n_bodies", len(bodies))
            prof.set_metric("n_legs", len(feet) // max(len(lk.feet), 1))
            logger.debug("reference mech: %d bodies", len(bodies))

            # Every body moves with the joints of its anchor: its own, or its host's.
            owner = {b.name: walk.anchor_of(b, by_name) for b in bodies}
            anchors = {n: _body_joint_world(by_name[n]) for n in set(owner.values()) if n}

            # --- stage 2: one mesh per congruence group (see _plan_meshes) ---
            mass_props: dict[str, _MassProps] = {}     # shared with the drive extras
            with prof.timed("2_mesh_share"):
                meshes_of = _plan_meshes(bodies, anchors, owner, prof, mass_props)
            logger.debug("%d bodies with parts -> %d meshes",
                         len(meshes_of.key_of), len(meshes_of.rep_of))

            class_mesh: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
            with prof.timed("2_tessellate_total"):
                for key, rep in meshes_of.rep_of.items():
                    with prof.timed(f"2_tessellate.{_material_of(rep)}"):
                        class_mesh[key] = _tessellate(rep.part)
                    nv = len(class_mesh[key][0])
                    nt = len(class_mesh[key][2]) // 3
                    prof.set_metric(f"verts.{key}", nv)
                    prof.set_metric(f"tris.{key}", nt)
                    logger.debug("  %s: %d verts, %d tris", key, nv, nt)
            prof.set_metric("n_meshes", len(class_mesh))

            # --- stage 3: pack per-mesh geometry accessors + materials ---
            packer = _Packer()
            accessors: list[pygltflib.Accessor] = []
            materials: list[pygltflib.Material] = []
            material_idx: dict[str, int] = {}
            meshes: list[pygltflib.Mesh] = []
            class_mesh_idx: dict[str, int] = {}

            with prof.timed("3_gltf_pack_geometry"):
                for key, (positions, normals, indices) in class_mesh.items():
                    mat = _material_of(meshes_of.rep_of[key])
                    if mat not in material_idx:
                        material_idx[mat] = len(materials)
                        materials.append(_gltf_material(mat))
                    pos_bv = packer.add(positions.tobytes(), target=pygltflib.ARRAY_BUFFER)
                    pos_acc = len(accessors)
                    accessors.append(
                        _accessor_for(
                            pos_bv, len(positions),
                            component_type=pygltflib.FLOAT,
                            accessor_type=pygltflib.VEC3,
                            min_vals=positions.min(axis=0).tolist(),
                            max_vals=positions.max(axis=0).tolist(),
                        )
                    )
                    nrm_bv = packer.add(normals.tobytes(), target=pygltflib.ARRAY_BUFFER)
                    nrm_acc = len(accessors)
                    accessors.append(
                        _accessor_for(
                            nrm_bv, len(normals),
                            component_type=pygltflib.FLOAT,
                            accessor_type=pygltflib.VEC3,
                        )
                    )
                    idx_bv = packer.add(indices.tobytes(), target=pygltflib.ELEMENT_ARRAY_BUFFER)
                    idx_acc = len(accessors)
                    accessors.append(
                        _accessor_for(
                            idx_bv, len(indices),
                            component_type=pygltflib.UNSIGNED_INT,
                            accessor_type=pygltflib.SCALAR,
                        )
                    )
                    class_mesh_idx[key] = len(meshes)
                    meshes.append(pygltflib.Mesh(name=key, primitives=[pygltflib.Primitive(
                        attributes=pygltflib.Attributes(POSITION=pos_acc, NORMAL=nrm_acc),
                        indices=idx_acc,
                        material=material_idx[mat],
                        mode=pygltflib.TRIANGLES,
                    )]))

            # --- stage 4: sample animation ---
            #
            # A body's motion is the planar rigid transform taking its anchor
            # joints at t_ref to the same joints at each frame; hardware
            # anchors on its ``rigid_with`` host. A shared mesh was modelled
            # on its representative, so the node first applies the body's
            # placement (``_MeshPlan.place``: planar motion + Z shift) and
            # then the motion.
            logger.debug("sampling %d frames over %.3fs…", n_frames, duration_s)
            ts = _T_REF + np.linspace(0.0, 2.0 * math.pi, n_frames, endpoint=False)
            times = np.linspace(0.0, duration_s, n_frames, endpoint=False, dtype=np.float32)

            translations: dict[str, np.ndarray] = {}
            rotations: dict[str, np.ndarray] = {}
            thetas: dict[str, np.ndarray] = {}

            with prof.timed("4_animation_sample_total"):
                with prof.timed("4.1_template_build"):
                    template = _build_template(config)
                with prof.timed("4.2_template_sample"):
                    sampled = template.sample(ts)
                with prof.timed("4.3_trs_batch"):
                    motion: dict[str, tuple[np.ndarray, np.ndarray]] = {}
                    for name, ref in anchors.items():
                        current = sampled.joint_world[name]
                        names = [n for n in ref if n in current]
                        p0 = np.broadcast_to(
                            np.stack([ref[n] for n in names]), (n_frames, len(names), 3)
                        )
                        p1 = np.stack([current[n] for n in names], axis=1)
                        motion[name] = walk.planar_fit(p0, p1)
                    for body in bodies:
                        prof.bump("body_extract.calls")
                        g = meshes_of.place.get(body.name, _Planar())
                        anchor = owner[body.name]
                        if anchor is None:
                            prof.bump("body_extract.static")
                            theta = np.full(n_frames, g.theta)
                            trans = np.tile([*g.txy, g.dz], (n_frames, 1))
                        else:
                            m_theta, m_trans = motion[anchor]
                            theta = m_theta + g.theta
                            c, s = np.cos(m_theta), np.sin(m_theta)
                            tx, ty = g.txy
                            trans = m_trans + np.stack(
                                [c * tx - s * ty, s * tx + c * ty, np.full(n_frames, g.dz)],
                                axis=1,
                            )
                        thetas[body.name] = theta
                        translations[body.name] = trans.astype(np.float32)
                        rotations[body.name] = _quat_hemisphere_continuous(
                            _quat_z(theta)
                        ).astype(np.float32)

            # --- shared time accessor ---
            time_bv = packer.add(times.tobytes())
            time_acc = len(accessors)
            accessors.append(
                _accessor_for(
                    time_bv, n_frames,
                    component_type=pygltflib.FLOAT,
                    accessor_type=pygltflib.SCALAR,
                    min_vals=[float(times.min())],
                    max_vals=[float(times.max())],
                )
            )

            # --- stage 5: root + per-body nodes, animation samplers/channels ---
            nodes: list[pygltflib.Node] = []
            animation_samplers: list[pygltflib.AnimationSampler] = []
            animation_channels: list[pygltflib.AnimationChannel] = []

            with prof.timed("5_gltf_nodes_channels"):
                root = pygltflib.Node(name="walker", rotation=list(_ROOT_ROTATION))
                nodes.append(root)
                ground = math.inf       # lowest model Y any mesh reaches over the cycle
                for body in bodies:
                    key = meshes_of.key_of.get(body.name)
                    initial_t = translations[body.name][0]
                    initial_q = rotations[body.name][0]

                    node = pygltflib.Node(
                        name=body.name,
                        translation=[float(initial_t[0]), float(initial_t[1]), float(initial_t[2])],
                        rotation=[float(initial_q[0]), float(initial_q[1]),
                                  float(initial_q[2]), float(initial_q[3])],
                    )
                    if key is not None:
                        node.mesh = class_mesh_idx[key]
                        v = class_mesh[key][0]
                        th = thetas[body.name]
                        y = np.outer(np.sin(th), v[:, 0]) + np.outer(np.cos(th), v[:, 1])
                        ground = min(ground, float((y.min(axis=1) + translations[body.name][:, 1])
                                                   .min()))
                    node.extras = {
                        "fab": body.fab, "rigid_with": body.rigid_with, "bom": body.bom_key,
                        "body": body.name,
                    }
                    node_idx = len(nodes)
                    nodes.append(node)

                    for path, data, acc_type in (
                        ("translation", translations[body.name], pygltflib.VEC3),
                        ("rotation", rotations[body.name], pygltflib.VEC4),
                    ):
                        bv = packer.add(data.tobytes())
                        acc = len(accessors)
                        accessors.append(
                            _accessor_for(
                                bv, n_frames,
                                component_type=pygltflib.FLOAT,
                                accessor_type=acc_type,
                            )
                        )
                        sampler_idx = len(animation_samplers)
                        animation_samplers.append(
                            pygltflib.AnimationSampler(
                                input=time_acc, output=acc, interpolation="LINEAR"
                            )
                        )
                        animation_channels.append(
                            pygltflib.AnimationChannel(
                                sampler=sampler_idx,
                                target=pygltflib.AnimationChannelTarget(
                                    node=node_idx, path=path
                                ),
                            )
                        )
                ground = 0.0 if not math.isfinite(ground) else ground
                # Stand it up: model +Y -> +Z, the lowest point of the gait on z = 0.
                root.translation = [0.0, 0.0, -ground]
                root.children = list(range(1, len(nodes)))
                root.extras = {"model_up": [0, 1, 0], "stack_axis": [0, 0, 1],
                               "ground_y": ground}

            animation = pygltflib.Animation(
                name="walk", samplers=animation_samplers, channels=animation_channels
            )

            # --- stage 6: foot-path extra (leg 0's first foot for reference; a
            # mechanism's output point, as ``output_path``) ---
            with prof.timed("6_foot_path_extra"):
                sol0 = lk.solve(1, 0.0, dict(config.proportions))
                foot_samples = 64
                foot = sol0.evaluate(
                    np.linspace(0.0, 2.0 * math.pi, foot_samples, endpoint=False)
                )[(lk.feet or feet)[0][1]]
                foot_path = [[float(x), float(y)] for x, y in foot]
                # Drawn just outside that foot's link (first side), in model Z.
                link = next((by_name[b] for b, _ in feet if by_name[b].part), None)
                foot_z = link.part.bounding_box().min.Z - 0.5 if link is not None else 0.0

            # --- stage 6b: the walking model's data (robot only; see walk.py) ---
            if robot:
                with prof.timed("6b_drive_extra"):
                    root.extras["drive"] = _drive_extra(
                        config, ref_mech, motion, owner, mass_props, duration_s)
                drive = root.extras["drive"]
                prof.set_metric("drive.mass_g", drive["mass_g"])
                prof.set_metric("drive.stride_mm", drive["metrics"]["stride_mm"])
                logger.debug("drive: com %s, %.1f g, stride %.1f mm/rev", drive["com"],
                             drive["mass_g"], drive["metrics"]["stride_mm"])

            scene = pygltflib.Scene(nodes=[0])
            path = "foot_path" if lk.feet else "output_path"
            scene.extras = {
                path: foot_path, f"{path}_z": foot_z,
                "mode": mode, "linkage": config.linkage, "module": config.module, "robot": robot,
                "meta": _json_meta(ref_mech.meta),
            }
            if lk.output:
                scene.extras["output"] = asdict(lk.output)

            # --- stage 7: assemble + binary-serialize glTF ---
            with prof.timed("7_serialize"):
                blob = packer.bytes
                gltf = pygltflib.GLTF2(
                    asset=pygltflib.Asset(version="2.0", generator="spiderpig/bake_gltf"),
                    scene=0,
                    scenes=[scene],
                    nodes=nodes,
                    meshes=meshes,
                    materials=materials,
                    accessors=accessors,
                    bufferViews=packer.buffer_views,
                    buffers=[pygltflib.Buffer(byteLength=len(blob))],
                    animations=[animation],
                )
                gltf.set_binary_blob(blob)
                gltf.save_binary(str(out))

            prof.set_metric("blob_bytes", len(blob))
            prof.set_metric("gltf_bytes", out.stat().st_size)
            prof.set_metric("animation_channels", len(animation_channels))
            prof.set_metric("accessors", len(accessors))
    finally:
        linkage_mod._BAKE_PROFILER = None
        if pr is not None:
            pr.disable()
            cprofile_out = Path(cprofile_out)
            cprofile_out.parent.mkdir(parents=True, exist_ok=True)
            pr.dump_stats(str(cprofile_out))
            txt_path = cprofile_out.with_suffix(cprofile_out.suffix + ".txt")
            import pstats  # noqa: PLC0415
            with open(txt_path, "w") as f:
                pstats.Stats(str(cprofile_out), stream=f).sort_stats(
                    "cumulative"
                ).print_stats(30)
            logger.info("cProfile dumped to %s (top-30 in %s)", cprofile_out, txt_path)

        # Peak resident set (linux: ru_maxrss is KB; mac: bytes — treat as linux here).
        try:
            import resource  # noqa: PLC0415
            ru = resource.getrusage(resource.RUSAGE_SELF)
            prof.set_metric("peak_rss_mb", ru.ru_maxrss / 1024.0)
        except ImportError:
            pass

        prof.log_summary()

    logger.info("wrote %s (%d B)", out, out.stat().st_size)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output .glb path (default: viewer/data/klann_<mode>.glb).",
    )
    p.add_argument("--frames", type=int, default=120, help="Animation frame count.")
    p.add_argument(
        "--duration", type=float, default=1.0, help="Animation duration in seconds."
    )
    p.add_argument(
        "--mode",
        choices=list(MODES),
        default=ROBOT,
        help="robot (both sides, default) or one side: single/double/decker/quad.",
    )
    p.add_argument(
        "--module",
        default=None,
        help=f"Legs per side for --mode robot: one of the linkage's modules, e.g. "
        f"{'/'.join(MODULES)} (default: {DEFAULT_MODULE}).",
    )
    walk.add_design_args(p)
    p.add_argument(
        "--profile",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Emit per-stage wall-clock profile summary (default: on).",
    )
    p.add_argument(
        "--cprofile",
        type=Path,
        default=None,
        metavar="PATH",
        help="Also dump a cProfile .prof file plus <PATH>.txt top-30 report.",
    )
    p.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level (default: INFO).",
    )
    args = p.parse_args()
    design = walk.design_args(args)
    phases_deg = design["phases_deg"]
    try:
        args.config = build_config(
            args.mode, args.module, linkage=design["linkage"],
            phases=None if phases_deg is None else [math.radians(v) for v in phases_deg],
            proportions=design["proportions"])
    except ValueError as e:
        p.error(str(e))
    return args


def main() -> None:
    args = _parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    # build123d logs every builder-less primitive at INFO; keep the profile readable.
    logging.getLogger("build123d").setLevel(max(logging.WARNING, logging.root.level))
    config = args.config
    out = args.out
    if out is None:
        out = (_REPO_ROOT / "viewer" / "data" / f"klann_{args.mode}.glb"
               if is_default(args.mode, config) else param_glb(args.mode, config))
    bake_gltf(
        out,
        n_frames=args.frames,
        duration_s=args.duration,
        mode=args.mode,
        module=args.module,
        config=config,
        profile=args.profile,
        cprofile_out=args.cprofile,
    )


if __name__ == "__main__":
    main()
