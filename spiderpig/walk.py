"""Walking model: the robot's feet, how it stands, how it moves and how well it walks.

This is the Python side of the drive contract shared with the viewer
(``/api/walk``, the ``drive`` extras of a baked robot; the viewer implements
the same model in TypeScript). Everything is quasi-static on flat ground: at
every instant the robot rests on the face of its feet's lower convex hull
under its centre of mass, and moves so that the feet touching the ground
don't slide (as well as they can).

Frames
------
**Mech frame** = body frame = the local frame of the glb root node
``walker``: x along the linkage plane (the walking axis), y up (feet at low
y), z lateral (the layer stack axis). The left side (bodies ``L.``) is at
negative z, the right side (``R.``) is its mirror image at positive z, the
robot's mid-plane is z = 0. Kinematic XY is the linkage's XY unchanged
(:mod:`linkage`: the crank centre O is the origin).

Inputs
------
Per side S in {L, R} a crank angle ``theta_S`` (rad; increasing = the
design's crank direction = the baked animation playing forward) and rate
``omega_S`` (rad/s). The baked clip's time ``tau`` in ``[0, D)`` is
``theta = 2 pi tau / D``. Both sides are the same planar mechanism, so a
leg's foot follows the same XY path on either side, at its own side's
crank angle.

Feet
----
The design's linkage (``BuildConfig.linkage``) names its feet: ``(link,
joint)`` pairs, one or more per leg (Klann: ``b4.F``; a Strider leg is a
coupled pair with two). A foot's path is sampled at ``N_THETA`` crank
angles ``theta_i = 2 pi i / N`` from the linkage's compiled program (the
side template's joints are its points) and interpolated linearly in theta
(wrapping); ``dp/dtheta`` is the central difference on that grid,
interpolated the same way. A foot's lateral ``z`` is the mid-plane of its
link plate's layer: from the layer plan when one is at hand
(:func:`foot_z_planned`), otherwise from :func:`foot_z_nominal`, the
linkage's default design's layers, planned once (exact for the default
designs).

Support (:func:`support`)
-------------------------
Candidate planes are the triangles of feet with area > ``AREA_MIN``,
normal ``n`` with ``n.y > 0``, that no foot is below (``>= -ON_PLANE``):
faces of the lower hull. Of the faces whose triangle contains the
projection (along ``n``) of the centre of mass ``c``, the robot rests on
the most level one (largest ``n.y``: several non-coplanar faces can contain
it). If none does it is ``tipping`` and rests on the face nearest that
projection (ties: the most level). Fewer than three feet, or all collinear:
``degenerate``, horizontal plane through the lowest foot. Attitude: ``pitch
= atan2(n.x, n.y)`` (about z), ``roll = atan2(n.z, n.y)`` (about x),
degrees; ``height`` is the signed distance from the body origin to the
plane. Contacts are the feet within ``CONTACT`` mm of it; ``margin`` is the
signed distance from c's projection to the edges of the contacts' convex
hull in the plane (positive inside).

Motion (:func:`body_velocity`)
------------------------------
A contact foot at ``(x_i, z_i)`` moving at ``pdot_i`` in the body frame
must not move over the ground::

    Vx + w z_i + pdot_ix = 0
    Vz - w x_i + pdot_iz = 0

for the body velocity ``(Vx, Vz)`` (body frame) and yaw rate ``w`` about
+y, by least squares over the contacts, solved about their centroid
``(xc, zc)`` (which decouples it)::

    U = -mean(pdot_i)                                   centroid velocity
    w = sum(dx_i pdot_iz - dz_i pdot_ix) / sum(dx_i^2 + dz_i^2)   (0 if < 1e-9)
    (Vx, Vz) = (Ux - w zc, Uz + w xc)                   body origin velocity

``slip`` is the RMS of the ``2 m`` scalar residuals (mm/s). World update
(explicit Euler): ``(X, Z) += R(yaw) (Vx, Vz) dt`` with ``dX = cos(yaw) Vx
+ sin(yaw) Vz``, ``dZ = -sin(yaw) Vx + cos(yaw) Vz``; ``yaw += w dt``.

Mass
----
A baked robot's centre of mass comes from its fabricated parts
(:func:`body_masses`, :func:`cycle_com`: volumes x densities, the servo's
catalogued mass), averaged over a crank revolution. ``/api/walk`` builds no
parts and uses :func:`nominal_mass` (fitted to the fabricated default
robots, see there).
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
from functools import cache
from itertools import combinations
from typing import overload

import numpy as np

from spiderpig import linkage as lkg
from spiderpig import servos
from spiderpig.config import BuildConfig
from spiderpig.construction.robot import SIDES, mid_plane, mid_plane_z
from spiderpig.fabricate import design_side, template_for
from spiderpig.hardware.mass import PartProps, material_of, part_props, servo_mass_g, sheet_density
from spiderpig.linkage import AssemblyError

N_THETA = 360        # crank-angle samples per revolution
# A design whose stability margin dips under this (mm) tips in MuJoCo (Jansen's quad: 4 mm
# predicted, over within 3 s): ``/api/walk`` says so (``stable``) and the viewer refuses to
# drive it in physics until it is tuned.
MIN_MARGIN_MM = 15.0
# Under this much ground per revolution the design doesn't walk (Klann's double: 2e-13 mm,
# its four feet stay coplanar; MuJoCo crawls it at 11 mm/s with the body on the floor):
# ``/api/walk`` says so (``walks``) and the viewer warns like the margin.
MIN_STRIDE_MM = 5.0
AREA_MIN = 1.0       # mm^2: smaller foot triangles don't define a plane
ON_PLANE = 1e-6      # mm: a foot this far below a candidate plane still counts as on it
CONTACT = 0.5        # mm: feet this close to the support plane are contacts
W_DEN_MIN = 1e-9     # mm^2: below this spread of the contacts the yaw rate is 0
_INSIDE_TOL = 1e-9   # mm: c's projection this close to a triangle counts as inside it
_HULL_TOL = 1e-7     # mm: side-of-line tolerance for the support polygon
DEFAULT_RPM = 50.0   # when the servo spec has no speed


# The linkage can't be assembled at some crank angle (a loop doesn't close): the
# template stage raises it (linkage.Linkage.assert_assembles).
LinkageError = AssemblyError


def links_of(lk: lkg.Linkage) -> list[tuple[str, str]]:
    """A leg's stick figure: the crank, every link's outline, the frame (O to each pivot)."""
    return [*(("O", pin) for pin in lk.crank[1:]),
            *(seg for _, outline in lk.links.values() for seg in outline),
            *(("O", j) for j in lk.frame if j != "O")]


def theta_grid(n: int = N_THETA) -> np.ndarray:
    return 2.0 * math.pi * np.arange(n) / n


# ---------------------------------------------------------------------------
# Leg kinematics and feet
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Leg:
    """One leg of a side over a revolution: its linkage's every point (mech frame) per theta."""

    leg: int
    orientation: int
    phase: float                     # rad
    joints: dict[str, np.ndarray]    # point -> (N, 2)


def side_legs(config: BuildConfig, n: int = N_THETA) -> list[Leg]:
    """The legs of one side sampled at ``n`` crank angles, from the compiled program.

    The side template's joints are the program's points (``tests/test_walk.py``
    checks it). Raises :class:`LinkageError` when a loop can't close somewhere
    in the cycle, with the reason :func:`fabricate.template_for` gives (the
    linkage's assembly check).
    """
    try:
        template_for(config)
    except ValueError as e:
        raise LinkageError(str(e)) from None
    lk = config.lk
    params = dict(config.proportions) or None
    ts = theta_grid(n)
    out = []
    for k, (orient, phase) in enumerate(config.legs):
        with np.errstate(all="ignore"):
            pts = lk.solve(orient, phase, params).evaluate(ts)
        out.append(Leg(k, int(orient), float(phase), pts))
        for name, xy in pts.items():                # a backstop on this grid
            bad = ~np.isfinite(xy).all(axis=-1)
            if bad.any():
                raise LinkageError(f"{lk.key} leg {k}: joint {name} can't be placed at crank "
                                   f"angle {np.degrees(ts[bad][0]):.0f} deg")
    return out


@dataclass(frozen=True)
class Foot:
    """A foot tip: a foot joint of a link (``linkage.Linkage.feet``), on one side."""

    body: str            # "L.b4_leg0"
    side: str            # "L" / "R"
    leg: int
    z: float             # lateral position (mech frame, mm)
    xy: np.ndarray       # (N, 2): XY at theta_i = 2 pi i / N

    def as_json(self, digits: int = 4) -> dict:
        return {"body": self.body, "side": self.side, "leg": self.leg,
                "z": round(float(self.z), digits),
                "xy": np.round(self.xy, digits).tolist()}


def side_feet(config: BuildConfig) -> list[tuple[int, str, str]]:
    """``(leg, body, joint)`` of every foot of one side: each leg's, in the linkage's order."""
    n = len(config.legs)
    return [(k, f"{link}{'' if n == 1 else f'_leg{k}'}", joint)
            for k in range(n) for link, joint in config.lk.feet]


def make_feet(config: BuildConfig, legs: Sequence[Leg], z_left: Sequence[float]) -> list[Foot]:
    """Both sides' feet: the left side's at ``z_left`` (one per foot), the right mirrored."""
    feet = side_feet(config)
    if len(z_left) != len(feet):
        raise ValueError(f"{len(feet)} feet per side, {len(z_left)} foot z values")
    return [Foot(f"{side}.{body}", side, k, sign * float(z), legs[k].joints[joint])
            for side, sign in (("L", 1.0), ("R", -1.0))
            for (k, body, joint), z in zip(feet, z_left, strict=True)]


def foot_z_nominal(config: BuildConfig) -> list[float]:
    """Left-side foot z (one per foot) without planning this design: the default design's.

    The linkage's default design (the module's phases, its proportions) is
    planned once (cached). With no layer plan at all, a guess: the feet one
    layer apart from layer 2 out. Exact for a default design; the right side
    is the mirror (``-z``).
    """
    z = _default_plan_z(replace(config, phases=None, proportions=()))
    if z is not None:
        return list(z)
    return foot_z_guess(config)


def foot_z_guess(config: BuildConfig) -> list[float]:
    """Left-side foot z without any plan (no search at all): the feet one layer apart from
    layer 2 out, in a stack of two layers per foot plus five. What :func:`foot_z_nominal`
    falls back to, and what a linkage card's per-module stride uses."""
    n = len(side_feet(config))
    layers, top = range(2, 2 + n), 2 * n + 5
    pitch = config.pitch
    z_mid = mid_plane_z(servos.get(config.servo), top, pitch, config.params.margin)
    return [(layer + 0.5) * pitch - z_mid for layer in layers]


@cache
def _default_plan_z(config: BuildConfig) -> tuple[float, ...] | None:
    try:
        return tuple(foot_z_planned(config))
    except ValueError:      # no layer plan, or a construction that can't be built
        return None


def foot_z_planned(config: BuildConfig, design=None) -> list[float]:
    """Left-side foot z (one per foot) from the layer plan (plans the side unless ``design``
    given).

    The foot link's layer mid-plane, moved like :func:`construction.robot.assemble_robot`
    moves the left side (mid-plane to z = 0).
    """
    if design is None:
        design = design_side(template_for(config), config)
    plan, z_mid = design.plan, mid_plane(design)
    # the foot link's own mid-plane: its sheet's thickness on its layer's floor
    return [plan.z(plan.layers[body])[0] + design.ctx.sheet_t("link", body) / 2 - z_mid
            for _, body, _ in side_feet(config)]


# ---------------------------------------------------------------------------
# Mass and centre of mass
# ---------------------------------------------------------------------------

def servo_info(key: str) -> dict:
    """``{"key", "rpm_max", "mass_g"}`` of a servo from its spec (``speed_rpm``,
    ``weight_g``); a spec without them gets ``DEFAULT_RPM`` and its body box
    (:func:`hardware.mass.servo_mass_g`)."""
    spec = servos.get(key)
    rpm = float(spec.speed_rpm) if spec.speed_rpm else DEFAULT_RPM
    return {"key": key, "rpm_max": rpm, "mass_g": servo_mass_g(spec)}


def body_masses(mech, config: BuildConfig,
                props: dict[str, PartProps] | None = None) -> dict[str, tuple[float, np.ndarray]]:
    """``body -> (grams, centre of mass)`` for every body with a part (model coordinates).

    Each part's volume at its material's density (:func:`hardware.mass.material_of`;
    the servo at its catalogued mass) at its centroid. ``props`` caches each
    body's :class:`hardware.mass.PartProps` (shared with the bake).
    """
    spec = servos.get(config.servo)
    props = {} if props is None else props
    out = {}
    for b in mech.bodies:
        if b.part is None:
            continue
        if b.name not in props:
            props[b.name] = part_props(b.part)
        p = props[b.name]
        _, density, fixed = material_of(b, config.sheet, mech.meta.get("filament"), spec)
        grams = fixed if fixed is not None else abs(p.volume) / 1000.0 * density
        out[b.name] = (grams, np.asarray(p.com, dtype=float))
    return out


def cycle_com(masses: Mapping[str, tuple[float, np.ndarray]],
              motion: Mapping[str, tuple[np.ndarray, np.ndarray]],
              owner: Mapping[str, str | None]) -> tuple[np.ndarray, float]:
    """The centre of mass averaged over a crank revolution, and the total mass.

    ``motion[anchor] = (theta (T,), translation (T, 3))`` is the planar rigid
    motion of a kinematic body from the reference pose to each sample;
    ``owner[body]`` is the anchor a body moves with (``None``: static).
    """
    total, acc = 0.0, np.zeros(3)
    for name, (grams, com) in masses.items():
        anchor = owner.get(name)
        if anchor is None or anchor not in motion:
            pos = com
        else:
            th, tr = motion[anchor]
            c, s = np.cos(th), np.sin(th)
            pos = np.stack([c * com[0] - s * com[1] + tr[:, 0],
                            s * com[0] + c * com[1] + tr[:, 1],
                            com[2] + tr[:, 2]], axis=1).mean(axis=0)
        total += grams
        acc += grams * pos
    return (acc / total if total > 0 else acc), total


def planar_fit(p0: np.ndarray, p1: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Planar rigid motion (rotation about z, then translation) taking ``p0`` to ``p1``.

    ``p0``, ``p1``: ``(T, K, 3)``. Returns ``(theta (T,), translation (T, 3))``.
    """
    c0, c1 = p0.mean(axis=1), p1.mean(axis=1)
    if p0.shape[1] == 1:
        return np.zeros(p0.shape[0]), c1 - c0
    v0, v1 = p0 - c0[:, None], p1 - c1[:, None]
    th = np.arctan2((v0[..., 0] * v1[..., 1] - v0[..., 1] * v1[..., 0]).sum(axis=1),
                    (v0[..., 0] * v1[..., 0] + v0[..., 1] * v1[..., 1]).sum(axis=1))
    c, s = np.cos(th), np.sin(th)
    rc0 = np.stack([c * c0[:, 0] - s * c0[:, 1], s * c0[:, 0] + c * c0[:, 1], c0[:, 2]], axis=1)
    return th, c1 - rc0


def anchor_of(body, by_name: Mapping) -> str | None:
    """The kinematic body whose joints carry ``body``: itself, or its ``rigid_with`` chain."""
    seen = set()
    while body is not None and body.name not in seen:
        if body.joints:
            return body.name
        seen.add(body.name)
        body = by_name.get(body.rigid_with) if body.rigid_with else None
    return None


# Nominal mass model (no parts), see :func:`nominal_mass`. Everything of a side but its
# link plates and servo is lumped on the crank axis O (its measured centre of mass is
# within a few mm of it): the laser-cut plates (the frame plates, by fixed pivot; the bolt
# crank's plates, by crankpin; half the centre plates), which follow their sheets' density
# and thickness (the constants are per 3 mm of acrylic: :func:`_sheet_scale`), and the
# purchased and printed rest (the crank's bolts and stub, the standoff pillars by pivot,
# the Chicago pins by pin joint, the ties, screws and horn). Refitted 2026-10-04 to the
# fabricated Klann quad and Strider double robots built the default way (aluminium frame
# and crank plates, a Klann's foot links in 6061; 1088.4 and 775.8 g with the deck: within
# 1.2 %). Refitted 2026-10-05 when purchased parts took their own material
# (:func:`hardware.mass.item_material`: the ties' and the stub's aluminium standoffs,
# -9.6 and -1.3 g a side; a pillar -1 g, between the Strider's steel one-piece columns and
# klann_lego's goBILDA ones): the Strider double -1.5 %, the demo Klann quad -0.7 %,
# klann_lego +1.5 %.
_ACRYLIC = 1.19                      # g/cm^3 the plate constants are per (3 mm thick)
_FRAME_BASE_G, _FRAME_PER_PIVOT_G = 16.2, 2.85    # the frame plates (per 3 mm acrylic)
_CENTRE_PLATES_G = 11.05             # half the centre plates (the robot's chassis)
_CHASSIS_REST_G = 17.65              # half the ties and rear screws: aluminium, steel
_DRIVE_EXTRA_G = 3.6                 # the servo's screws and horn
_CRANK_PLATES_BASE_G, _CRANK_PLATES_PER_PIN_G = 3.78, 5.07   # the bolt crank's plates
_CRANK_HW_BASE_G, _CRANK_HW_PER_PIN_G = 9.6, 8.28             # ... its bolts, nuts, stub
_PILLAR_PER_PIVOT_G = 12.0           # a standoff pillar (segments, rings, screws, washers)
_PILLAR_PER_PIVOT_LEG_G = 0.0
_PIN_PER_JOINT_G = 3.3               # a Chicago pin (screw, rings, washer, shims)


def _sheet_scale(config: BuildConfig, key: str) -> float:
    """A sheet's mass per area over 3 mm acrylic's (the plate constants')."""
    from spiderpig.materials import thickness

    return sheet_density(key) * thickness(config, key) / (_ACRYLIC * 3.0)


# The electronics deck (construction.deck), robot only: its fabricated mass on 3 mm acrylic
# (plate 33 g of it, at the sheet's density; the rest the electronics, rails and hardware)
# and its centre of mass from the servo body's centre (x) and the chassis' top, which is
# the servo's highest corner plus 2.36 mm (the centre plates round the ties, 2 thicknesses
# of aluminium to the edge): measured on the Strider double (116.6 g since the rails are
# screwed on, 2026-10-04; 113 g with glued spigots), centre 13.45 mm over the chassis top.
# (+2.5 g: the cradle screwed down; -1.4 g: the simplified hardware's button-head deck
# screws and 8 mm-stud board standoffs, 2026-10-05)
_DECK_PLATE_G, _DECK_REST_G = 33.3, 84.4
_DECK_COM_DX, _DECK_COM_DY, _DECK_FLOOR_DY = -4.0, 13.45, 2.36


def _pin_joints(lk) -> int:
    """Pin joints of one leg: joints of its links that are neither fixed nor on the crank,
    nor a foot or output point."""
    joints = {j for js, _ in lk.links.values() for j in js}
    joints -= set(lk.frame) | set(lk.crank) | {f for _, f in lk.feet}
    if lk.output is not None:
        joints.discard(lk.output.point)
    return len(joints)


def nominal_deck(spec, u: np.ndarray, scale: float = 1.0) -> tuple[float, np.ndarray]:
    """The electronics deck's nominal mass (g) and centre of mass (side XY, mm) for a servo
    ``spec`` whose +x is ``u`` (its frame centred on the crank axis): over the servo body's
    centre, :data:`_DECK_COM_DY` above the chassis' top. ``scale``: the deck sheet's mass
    per area over 3 mm acrylic's."""
    L, W, _ = spec.body
    v = np.array([u[1], -u[0]])
    corners = [(spec.axis_offset + a) * u + b * v for a in (-L / 2, L / 2) for b in (-W / 2, W / 2)]
    floor = max(q[1] for q in corners) + _DECK_FLOOR_DY
    com = np.array([spec.axis_offset * u[0] + _DECK_COM_DX, floor + _DECK_COM_DY])
    return _DECK_PLATE_G * scale + _DECK_REST_G, com


def nominal_mass_breakdown(config: BuildConfig, legs: Sequence[Leg], robot: bool = True
                           ) -> dict:
    """The nominal mass without building parts, by what it is made of (grams): ``links``
    (every outline segment a pill of the link radius, one pitch thick, at the sheet's
    density, the joint discs counted once and the axle holes taken out), ``servos``,
    ``plates`` (the frame plates and, for the robot, the centre plates, at the sheet's
    density), ``printed`` (the crank, pillars, pins, ties: PLA and small hardware),
    ``deck`` (the robot's electronics deck, :mod:`construction.deck`: fitted), plus
    ``total``, the cycle-mean centre of mass ``com`` (side coordinates, z = 0) and a
    ``note`` on how it was made. ``robot``: both sides and the chassis (what the walk
    model always is), else one side alone. On the default robots (Strider, Klann,
    klann_lego) this is within 2 % of the fabricated mass (:func:`body_masses`); a build
    measures it."""
    from spiderpig.materials import link_sheets, thickness

    lk = config.lk
    pitch = config.pitch
    p = config.params
    r = p.link_radius
    dens = sheet_density(config.sheet)
    own = link_sheets(config)          # link class -> its sheet, where not the default

    def mass_per(link: str) -> float:
        """g per mm^3 / 1000 x thickness: a link's sheet's density times its thickness."""
        key = own.get(link)
        if key is None:
            return dens * pitch
        return sheet_density(key) * thickness(config, key)
    spec = servos.get(config.servo)
    servo_g = servo_info(config.servo)["mass_g"]
    hole = p.hole(p.axle_d) / 2
    links, c_acc = 0.0, np.zeros(2)
    for leg in legs:
        for link, (joints, outline) in lk.links.items():
            m = mass_per(link)
            seen: dict[str, int] = {}
            for a_name, b_name in outline:
                a, b = leg.joints[a_name], leg.joints[b_name]
                length = float(np.linalg.norm(b - a, axis=-1).mean())
                grams = m * (2 * r * length + math.pi * r * r) / 1000.0
                links += grams
                c_acc += grams * ((a + b) / 2).mean(axis=0)
                seen[a_name] = seen.get(a_name, 0) + 1
                seen[b_name] = seen.get(b_name, 0) + 1
            for k in seen.values():                     # a joint's disc counted once
                links -= m * (k - 1) * math.pi * r * r / 1000.0
            links -= m * len(joints) * math.pi * hole * hole / 1000.0
    pivots = {tuple(np.round(leg.joints[j][0], 6)) for leg in legs for j in lk.frame if j != "O"}
    # servo +x: away from the frame pillars (the distinct fixed pivots)
    away = -np.sum([np.asarray(q) for q in pivots], axis=0) if pivots else np.zeros(2)
    norm = np.linalg.norm(away)
    u = away / norm if norm > 1e-9 else np.array([1.0, 0.0])
    c_acc += servo_g * (spec.axis_offset * u)
    scale = _sheet_scale(config, config.sheet)
    plates = (_FRAME_BASE_G + _FRAME_PER_PIVOT_G * len(pivots)) * _sheet_scale(
        config, config.frame_sheet)
    # crankpins at distinct positions (a mirrored pair shares one; a decker's are 90° apart)
    crankpins = {tuple(np.round(leg.joints[p][0], 3)) for leg in legs for p in lk.crank[1:]}
    plates += (_CRANK_PLATES_BASE_G + _CRANK_PLATES_PER_PIN_G * len(crankpins)) \
        * _sheet_scale(config, config.crank_sheet)
    crank = _CRANK_HW_BASE_G + _CRANK_HW_PER_PIN_G * len(crankpins)
    printed = (_DRIVE_EXTRA_G + crank
               + (_PILLAR_PER_PIVOT_G + _PILLAR_PER_PIVOT_LEG_G * len(legs)) * len(pivots)
               + _PIN_PER_JOINT_G * _pin_joints(lk) * len(legs))
    if robot:
        plates += _CENTRE_PLATES_G * scale
        printed += _CHASSIS_REST_G
    side = links + servo_g + plates + printed
    sides = 2 if robot else 1
    c = c_acc / side
    deck = 0.0
    if robot:                     # the electronics deck over the servos, between the frames
        deck, dc = nominal_deck(spec, u, scale)
        c = (c * sides * side + deck * dc) / (sides * side + deck)
    return {
        "links": sides * links, "servos": sides * servo_g, "plates": sides * plates,
        "printed": sides * printed, "deck": deck, "total": sides * side + deck,
        "com": np.array([c[0], c[1], 0.0]),
        "note": (f"links as {dens:g} g/cm3 pills of the link radius on {pitch:g} mm layers "
                 f"(holes taken out; a link of another sheet at its own), the frame plates "
                 f"at their sheet ({config.frame_sheet}); the plates and the printed parts "
                 f"from fitted "
                 f"constants (within 2 % on the default robots); a build measures it"),
    }


def nominal_mass(config: BuildConfig, legs: Sequence[Leg]) -> tuple[np.ndarray, float]:
    """Centre of mass (cycle mean) and total mass of the robot without building parts
    (:func:`nominal_mass_breakdown`). The two sides are mirror images, so z = 0."""
    b = nominal_mass_breakdown(config, legs)
    return b["com"], float(b["total"])


# ---------------------------------------------------------------------------
# The walker
# ---------------------------------------------------------------------------


@dataclass
class Walker:
    """Both sides' feet and the centre of mass: everything the walking model needs."""

    feet: list[Foot]
    com: np.ndarray                  # (3,) mech frame
    mass_g: float
    config: BuildConfig
    legs: list[Leg] = field(default_factory=list)   # one side, for display
    z_nominal: bool = True
    com_nominal: bool = True

    def __post_init__(self) -> None:
        self.com = np.asarray(self.com, dtype=float).reshape(3)
        self._xy = np.stack([f.xy for f in self.feet])            # (F, N, 2)
        n = self._xy.shape[1]
        step = 2.0 * math.pi / n
        self._dxy = (np.roll(self._xy, -1, axis=1) - np.roll(self._xy, 1, axis=1)) / (2 * step)
        self._z = np.array([f.z for f in self.feet], dtype=float)
        self._right = np.array([f.side == "R" for f in self.feet])

    @property
    def n(self) -> int:
        return self._xy.shape[1]

    @property
    def side_index(self) -> np.ndarray:
        """Per foot: 0 for the left side, 1 for the right."""
        return self._right.astype(np.int64)

    def feet_at(self, theta_l, theta_r) -> tuple[np.ndarray, np.ndarray]:
        """Feet positions ``(T, F, 3)`` and ``dp/dtheta`` ``(T, F, 3)`` (mm/rad).

        ``theta_l`` / ``theta_r`` are each side's crank angle, scalars or ``(T,)``.
        """
        tl, tr = np.broadcast_arrays(np.atleast_1d(np.asarray(theta_l, dtype=float)),
                                     np.atleast_1d(np.asarray(theta_r, dtype=float)))
        theta = np.where(self._right[None, :], tr[:, None], tl[:, None])      # (T, F)
        n = self.n
        u = theta / (2.0 * math.pi) * n
        fl = np.floor(u)
        frac = (u - fl)[..., None]
        i0 = fl.astype(np.int64) % n
        i1 = (i0 + 1) % n
        f = np.arange(len(self.feet))[None, :]
        xy = self._xy[f, i0] * (1 - frac) + self._xy[f, i1] * frac
        dxy = self._dxy[f, i0] * (1 - frac) + self._dxy[f, i1] * frac
        z = np.broadcast_to(self._z, theta.shape)
        p = np.concatenate([xy, z[..., None]], axis=-1)
        pd = np.concatenate([dxy, np.zeros((*theta.shape, 1))], axis=-1)
        return p, pd


def walker(config: BuildConfig | None = None, *, feet_z: Sequence[float] | None = None,
           com: Sequence[float] | np.ndarray | None = None, mass_g: float | None = None,
           legs: Sequence[Leg] | None = None, n: int = N_THETA) -> Walker:
    """The walking model of the robot ``config`` describes (no parts are built).

    ``feet_z``: left-side foot z per foot (e.g. :func:`foot_z_planned`), else
    :func:`foot_z_nominal`. ``com`` / ``mass_g``: e.g. from a fabricated
    robot (:func:`body_masses`, :func:`cycle_com`), else :func:`nominal_mass`.
    ``legs``: one side's sampled legs (default :func:`side_legs`).
    """
    config = config or BuildConfig()
    legs = list(legs) if legs is not None else side_legs(config, n)
    z_nominal = feet_z is None
    z_left = foot_z_nominal(config) if z_nominal else [float(z) for z in feet_z]
    feet = make_feet(config, legs, z_left)
    com_nominal = com is None
    if com_nominal:
        com, m = nominal_mass(config, legs)
        mass_g = m if mass_g is None else mass_g
    elif mass_g is None:
        mass_g = float("nan")
    return Walker(feet=feet, com=np.asarray(com, dtype=float), mass_g=float(mass_g),
                  config=config, legs=list(legs), z_nominal=z_nominal, com_nominal=com_nominal)


# ---------------------------------------------------------------------------
# Support
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Support:
    """How the robot stands, per sample (leading axes ``...`` of the feet given).

    ``normal`` is the support plane's unit normal in the body frame (n.y > 0)
    and ``point`` a point on it; ``face`` the indices of the three feet
    defining it (-1 when degenerate).
    """

    normal: np.ndarray        # (..., 3)
    point: np.ndarray         # (..., 3)
    height: np.ndarray        # (...,)  signed distance of the body origin above the plane (mm)
    pitch_deg: np.ndarray     # (...,)  atan2(n.x, n.y)
    roll_deg: np.ndarray      # (...,)  atan2(n.z, n.y)
    contacts: np.ndarray      # (..., F) bool
    margin: np.ndarray        # (...,)  mm, positive: c's projection inside the support polygon
    tipping: np.ndarray       # (...,) bool
    degenerate: np.ndarray    # (...,) bool
    face: np.ndarray          # (..., 3) int

    def at(self, i) -> Support:
        return Support(*(getattr(self, f)[i] for f in self.__dataclass_fields__))


@cache
def _combos(n: int, k: int) -> np.ndarray:
    return np.array(list(combinations(range(n), k)), dtype=np.int64).reshape(-1, k)


def _seg_distance(q: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Distance from points ``q`` to segments ``a``-``b`` (broadcast, last axis xyz)."""
    d = b - a
    dd = np.einsum("...i,...i->...", d, d)
    t = np.clip(np.einsum("...i,...i->...", q - a, d) / np.where(dd > 0, dd, 1.0), 0.0, 1.0)
    return np.linalg.norm(q - (a + t[..., None] * d), axis=-1)


def _triangle_distance(q, a, b, c, n) -> np.ndarray:
    """Distance from ``q`` (in the triangle's plane, unit normal ``n``) to triangle abc."""
    orient = np.sign(np.einsum("...i,...i->...", np.cross(b - a, c - a), n))
    inside = np.ones(q.shape[:-1], dtype=bool)
    for p0, p1 in ((a, b), (b, c), (c, a)):
        side = orient * np.einsum("...i,...i->...", np.cross(p1 - p0, q - p0), n)
        inside &= side >= -_INSIDE_TOL * np.linalg.norm(p1 - p0, axis=-1)
    dist = np.minimum(np.minimum(_seg_distance(q, a, b), _seg_distance(q, b, c)),
                      _seg_distance(q, c, a))
    return np.where(inside, 0.0, dist)


def support(feet: np.ndarray, com: Sequence[float] | np.ndarray) -> Support:
    """The support state for feet positions ``(..., F, 3)`` (body frame) and centre of mass.

    See the module docstring; vectorized over the leading axes.
    """
    feet = np.asarray(feet, dtype=float)
    lead, n_feet = feet.shape[:-2], feet.shape[-2]
    P = feet.reshape(-1, n_feet, 3)
    T = P.shape[0]
    c = np.asarray(com, dtype=float).reshape(3)
    rows = np.arange(T)

    normal = np.tile([0.0, 1.0, 0.0], (T, 1))
    point = np.zeros((T, 3))
    face = np.full((T, 3), -1, dtype=np.int64)
    tipping = np.zeros(T, dtype=bool)
    degenerate = np.ones(T, dtype=bool)

    tri = _combos(n_feet, 3)
    if len(tri):
        A, B, C = P[:, tri[:, 0]], P[:, tri[:, 1]], P[:, tri[:, 2]]          # (T, K, 3)
        cr = np.cross(B - A, C - A)
        length = np.linalg.norm(cr, axis=-1)
        sign = np.where(cr[..., 1] < 0, -1.0, 1.0)
        n = cr * (sign / np.where(length > 0, length, 1.0))[..., None]
        ok = (length / 2 > AREA_MIN) & (n[..., 1] > 0.0)
        dist = np.einsum("tkfi,tki->tkf", P[:, None, :, :] - A[:, :, None, :], n)
        valid = ok & (dist >= -ON_PLANE).all(axis=-1)
        q = c - np.einsum("tki,tki->tk", c - A, n)[..., None] * n              # c projected
        dtri = np.where(valid, _triangle_distance(q, A, B, C, n), np.inf)
        dmin = dtri.min(axis=1)
        has = np.isfinite(dmin)
        # the most level of the nearest faces (containing ones are at distance 0)
        near = valid & (dtri <= np.maximum(dmin, 0.0)[:, None] + _INSIDE_TOL)
        best = np.argmax(np.where(near, n[..., 1], -np.inf), axis=1)
        degenerate = ~has
        tipping = has & (dmin > _INSIDE_TOL)
        normal = np.where(has[:, None], n[rows, best], normal)
        point = np.where(has[:, None], A[rows, best], point)
        face = np.where(has[:, None], tri[best], face)

    low = P[..., 1].argmin(axis=1)
    point = np.where(degenerate[:, None], P[rows, low], point)
    dist = np.einsum("tfi,ti->tf", P - point[:, None, :], normal)
    contacts = dist <= CONTACT
    height = -np.einsum("ti,ti->t", point, normal)
    pitch = np.degrees(np.arctan2(normal[:, 0], normal[:, 1]))
    roll = np.degrees(np.arctan2(normal[:, 2], normal[:, 1]))
    margin = _margin(P, c, normal, point, contacts)

    def shaped(a, extra=()):
        return a.reshape(lead + tuple(extra))

    return Support(
        normal=shaped(normal, (3,)), point=shaped(point, (3,)), height=shaped(height),
        pitch_deg=shaped(pitch), roll_deg=shaped(roll),
        contacts=shaped(contacts, (n_feet,)), margin=shaped(margin),
        tipping=shaped(tipping), degenerate=shaped(degenerate), face=shaped(face, (3,)),
    )


def _margin(P, c, normal, point, contacts) -> np.ndarray:
    """Signed distance from c's projection to the contacts' convex hull in the plane.

    Vectorized over the edges of the hull: a pair of contacts is an edge when
    every other contact is on one side of the line through it. Rows whose
    contacts are collinear (or fewer than three) fall back to :func:`_hull_distance`.
    """
    T, n_feet, _ = P.shape
    Pp = P - np.einsum("tfi,ti->tf", P - point[:, None, :], normal)[..., None] * normal[:, None]
    q = c - np.einsum("ti,ti->t", c - point, normal)[:, None] * normal                # (T, 3)
    out = np.full(T, np.nan)
    pairs = _combos(n_feet, 2)
    if len(pairs):
        Pi, Pj = Pp[:, pairs[:, 0]], Pp[:, pairs[:, 1]]                            # (T, M, 3)
        d = Pj - Pi
        length = np.linalg.norm(d, axis=-1)
        m = np.cross(np.broadcast_to(normal[:, None, :], d.shape), d)
        m = m / np.where(length > 0, length, 1.0)[..., None]
        s = np.einsum("tmfi,tmi->tmf", Pp[:, None, :, :] - Pi[:, :, None, :], m)    # (T, M, F)
        others = np.broadcast_to(contacts[:, None, :], s.shape)
        above = (s > _HULL_TOL) & others
        below = (s < -_HULL_TOL) & others
        edge = (contacts[:, pairs[:, 0]] & contacts[:, pairs[:, 1]] & (length > _HULL_TOL)
                & (above.any(-1) ^ below.any(-1)))
        orient = np.where(above.any(-1), 1.0, -1.0)
        line = orient * np.einsum("tmi,tmi->tm", q[:, None, :] - Pi, m)
        inside = np.where(edge, line >= -_HULL_TOL, True).all(axis=1)
        inner = np.where(edge, line, np.inf).min(axis=1)
        outer = np.where(edge, _seg_distance(q[:, None, :], Pi, Pj), np.inf).min(axis=1)
        good = edge.any(axis=1)
        out = np.where(good, np.where(inside, inner, -outer), np.nan)
    for t in np.flatnonzero(np.isnan(out)):
        out[t] = _hull_distance(q[t], Pp[t][contacts[t]])
    return out


def _hull_distance(q: np.ndarray, pts: np.ndarray) -> float:
    """``-distance`` from ``q`` to the hull of fewer-than-3 or collinear points ``pts``."""
    if len(pts) == 0:
        return -math.inf
    if len(pts) == 1:
        return -float(np.linalg.norm(q - pts[0]))
    d = pts[:, None, :] - pts[None, :, :]
    i, j = np.unravel_index(np.argmax(np.einsum("abi,abi->ab", d, d)), d.shape[:2])
    return -float(_seg_distance(q, pts[i], pts[j]))


# ---------------------------------------------------------------------------
# Motion
# ---------------------------------------------------------------------------


def body_velocity(feet: np.ndarray, rates: np.ndarray, contacts: np.ndarray,
                  ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Body velocity making the contact feet not slide, by least squares.

    ``feet`` ``(..., F, 3)`` positions and ``rates`` ``(..., F, 3)`` velocities
    (mm/s) in the body frame, ``contacts`` ``(..., F)``. Returns ``V (..., 2)``
    = (Vx, Vz) (mm/s, body frame), yaw rate ``w (...)`` (rad/s about +y) and
    ``slip (...)`` (mm/s, RMS of the 2m scalar residuals). Solved about the
    contacts' centroid (see the module docstring); no contacts: all zero.
    """
    feet, rates = np.asarray(feet, dtype=float), np.asarray(rates, dtype=float)
    k = np.asarray(contacts, dtype=float)
    m = k.sum(axis=-1)
    msafe = np.where(m > 0, m, 1.0)
    x, z = feet[..., 0], feet[..., 2]
    px, pz = rates[..., 0], rates[..., 2]
    xc = (k * x).sum(-1) / msafe
    zc = (k * z).sum(-1) / msafe
    dx, dz = x - xc[..., None], z - zc[..., None]
    ux = -(k * px).sum(-1) / msafe
    uz = -(k * pz).sum(-1) / msafe
    den = (k * (dx * dx + dz * dz)).sum(-1)
    num = (k * (dx * pz - dz * px)).sum(-1)
    w = np.where(den >= W_DEN_MIN, num / np.where(den >= W_DEN_MIN, den, 1.0), 0.0)
    rx = ux[..., None] + w[..., None] * dz + px          # Vx + w z_i + pdot_ix
    rz = uz[..., None] - w[..., None] * dx + pz          # Vz - w x_i + pdot_iz
    slip = np.sqrt((k * (rx * rx + rz * rz)).sum(-1) / (2 * msafe))
    V = np.stack([ux - w * zc, uz + w * xc], axis=-1)
    return V, w, slip


@dataclass(frozen=True)
class Trace:
    """A simulated drive: states at ``t`` (K+1 samples, the last one after the final step)."""

    t: np.ndarray            # (K+1,) s
    theta: np.ndarray        # (K+1, 2) crank angles (L, R), rad
    omega: np.ndarray        # (K+1, 2) rad/s (the last step's repeated at the end)
    x: np.ndarray            # (K+1,) world position on the ground (mm)
    z: np.ndarray
    yaw: np.ndarray          # (K+1,) rad about +y
    V: np.ndarray            # (K+1, 2) body velocity (mm/s, body frame)
    w: np.ndarray            # (K+1,) yaw rate (rad/s)
    slip: np.ndarray         # (K+1,) mm/s
    support: Support         # per state


def simulate(model: Walker, omega_l, omega_r, dt, *, theta0: tuple[float, float] = (0.0, 0.0),
             pose0: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> Trace:
    """Drive the robot with crank rates ``omega_l[k]``, ``omega_r[k]`` (rad/s) for steps ``dt``.

    Rates and ``dt`` are scalars or length-K sequences (broadcast). Explicit
    Euler: the state after step k uses the velocities at state k. ``theta0``
    is the start ``(theta_L, theta_R)``, ``pose0`` the start ``(x, z, yaw)``
    on the ground.
    """
    wl, wr, dts = np.broadcast_arrays(np.atleast_1d(np.asarray(omega_l, dtype=float)),
                                      np.atleast_1d(np.asarray(omega_r, dtype=float)),
                                      np.atleast_1d(np.asarray(dt, dtype=float)))
    omega = np.stack([wl, wr], axis=1)                                  # (K, 2)
    omega = np.concatenate([omega, omega[-1:]], axis=0)                 # (K+1, 2)
    t = np.concatenate([[0.0], np.cumsum(dts)])
    theta = np.asarray(theta0, dtype=float) + np.concatenate(
        [np.zeros((1, 2)), np.cumsum(omega[:-1] * dts[:, None], axis=0)])
    P, dP = model.feet_at(theta[:, 0], theta[:, 1])
    rates = dP * omega[:, model.side_index][..., None]
    sup = support(P, model.com)
    V, w, slip = body_velocity(P, rates, sup.contacts)
    yaw = pose0[2] + np.concatenate([[0.0], np.cumsum(w[:-1] * dts)])
    c, s = np.cos(yaw[:-1]), np.sin(yaw[:-1])
    dx = (c * V[:-1, 0] + s * V[:-1, 1]) * dts
    dz = (-s * V[:-1, 0] + c * V[:-1, 1]) * dts
    x = pose0[0] + np.concatenate([[0.0], np.cumsum(dx)])
    z = pose0[1] + np.concatenate([[0.0], np.cumsum(dz)])
    return Trace(t=t, theta=theta, omega=omega, x=x, z=z, yaw=yaw, V=V, w=w, slip=slip,
                 support=sup)


# ---------------------------------------------------------------------------
# Straight-walk metrics
# ---------------------------------------------------------------------------


def straight_walk_metrics(model: Walker, *, rpm_max: float | None = None,
                          omega: float = 1.0) -> dict:
    """Gait metrics over one crank revolution, both sides at the same angle and rate.

    One Euler step per sample of the model's grid (``N_THETA``), from theta = 0.

    ``stride_mm`` = |distance per revolution| along body x, ``direction``
    its sign (``"+x"`` / ``"-x"``), ``stride_signed_mm``; ``bob_mm`` = range
    of the body height; ``pitch_deg`` / ``roll_deg`` = [min, max];
    ``slip_rms`` (= ``slip_rms_mm_per_rad``) = RMS slip / omega (mm per rad
    of crank) and ``slip_rms_mm_per_rev`` = that x 2 pi; ``min_margin_mm``;
    ``tipping_fraction`` / ``degenerate_fraction`` of the samples; ``duty`` =
    fraction of the cycle each foot (in ``model.feet`` order) is a contact;
    ``speed_mm_s`` at the servo's ``rpm_max``. Extras: ``drift_mm``
    (lateral per revolution), ``yaw_deg_per_rev``, ``foot_bob_mm`` (range of
    the lowest foot's y), ``mean_contacts``; ``walks``: the stride is at least
    :data:`MIN_STRIDE_MM` and the support isn't degenerate (fewer than three
    feet) half the cycle or more.
    """
    n = model.n
    ts = theta_grid(n)
    dt = 2.0 * math.pi / (n * omega)
    trace = simulate(model, np.full(n, omega), np.full(n, omega), dt)
    sup = trace.support
    h, pitch, roll = sup.height[:n], sup.pitch_deg[:n], sup.roll_deg[:n]
    slip = trace.slip[:n]
    stride = float(trace.x[n])
    if rpm_max is None:
        rpm_max = float(servo_info(model.config.servo)["rpm_max"])     # (a float already)
    P, _ = model.feet_at(ts, ts)
    slip_rad = float(np.sqrt(np.mean(slip ** 2)) / omega)
    return {
        "stride_mm": abs(stride),
        "direction": "+x" if stride >= 0 else "-x",
        "stride_signed_mm": stride,
        "bob_mm": float(h.max() - h.min()),
        "pitch_deg": [float(pitch.min()), float(pitch.max())],
        "roll_deg": [float(roll.min()), float(roll.max())],
        "slip_rms": slip_rad,
        "slip_rms_mm_per_rad": slip_rad,
        "slip_rms_mm_per_rev": slip_rad * 2.0 * math.pi,
        "min_margin_mm": float(sup.margin[:n].min()),
        "tipping_fraction": float(sup.tipping[:n].mean()),
        "degenerate_fraction": float(sup.degenerate[:n].mean()),
        "duty": [float(v) for v in sup.contacts[:n].mean(axis=0)],
        "rpm_max": float(rpm_max),
        "speed_mm_s": abs(stride) * float(rpm_max) / 60.0,
        "drift_mm": float(trace.z[n]),
        "yaw_deg_per_rev": float(np.degrees(trace.yaw[n])),
        "foot_bob_mm": float(np.ptp(P[..., 1].min(axis=-1))),
        "mean_contacts": float(sup.contacts[:n].sum(axis=-1).mean()),
        "walks": bool(abs(stride) >= MIN_STRIDE_MM and sup.degenerate[:n].mean() < 0.5),
    }


# Weights of :func:`objective` (mm-equivalents).
OBJECTIVE_WEIGHTS = {
    "bob": 1.0,          # per mm of body bob
    "attitude": 1.0,     # per degree of pitch range + roll range
    "slip": 0.5,         # per mm/rev of RMS slip
    "tipping": 200.0,    # per unit fraction of the cycle tipping
    "degenerate": 200.0,  # per unit fraction of the cycle on fewer than 3 feet
    "margin": 2.0,       # per mm the margin goes negative
    "stride": 100.0,     # per unit fraction the stride falls below 90 % of the reference
}


def objective(metrics: Mapping, stride_ref: float | None = None,
              weights: Mapping[str, float] | None = None) -> float:
    """A gait score to minimise (mm-equivalents; lower is smoother).

    ``bob + (pitch range + roll range in deg) + 0.5 slip (mm/rev)``, plus
    penalties: 200 per unit tipping / degenerate fraction, 2 per mm of
    negative stability margin, and 100 per unit fraction the stride falls
    short of 90 % of ``stride_ref`` (e.g. the default design's stride; no
    stride term when that is under 1 mm).
    """
    w = dict(OBJECTIVE_WEIGHTS, **(weights or {}))
    m = metrics
    score = (w["bob"] * m["bob_mm"]
             + w["attitude"] * (np.ptp(m["pitch_deg"]) + np.ptp(m["roll_deg"]))
             + w["slip"] * m["slip_rms_mm_per_rev"]
             + w["tipping"] * m["tipping_fraction"]
             + w["degenerate"] * m["degenerate_fraction"]
             + w["margin"] * max(0.0, -m["min_margin_mm"]))
    if stride_ref is not None and stride_ref >= 1.0:
        score += w["stride"] * max(0.0, 0.9 - m["stride_mm"] / stride_ref)
    return float(score)


# ---------------------------------------------------------------------------
# Payloads: /api/walk and the baked robot's "drive" extras
# ---------------------------------------------------------------------------


def _round(a, digits: int = 4):
    return np.round(np.asarray(a, dtype=float), digits).tolist()


@overload
def jsonable(obj: Mapping) -> dict: ...
@overload
def jsonable(obj: object) -> object: ...
def jsonable(obj):
    """``obj`` with numpy scalars/arrays as Python ones and non-finite floats as ``None``.

    The plain types go first, by their exact type, and a list of plain floats (a foot's or
    a joint's track: most of an ``/api/walk`` body) in one comprehension: the values the
    general rules below give, which everything else (numpy, subclasses) goes through."""
    t = type(obj)
    if t is float:
        return obj if math.isfinite(obj) else None
    if t is list or t is tuple:
        if all(type(v) is float for v in obj):
            return [v if math.isfinite(v) else None for v in obj]
        return [jsonable(v) for v in obj]
    if t is dict:
        return {str(k): jsonable(v) for k, v in obj.items()}
    if t is str or t is int or t is bool or obj is None:
        return obj
    if isinstance(obj, Mapping):
        return {str(k): jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return jsonable(obj.tolist())
    if isinstance(obj, (bool, np.bool_)):
        return bool(obj)
    if isinstance(obj, (int, np.integer)):
        return int(obj)
    if isinstance(obj, (float, np.floating)):
        return float(obj) if math.isfinite(obj) else None
    return obj


def drive_extra(model: Walker, clip_duration_s: float, metrics: dict | None = None) -> dict:
    """The ``drive`` extras of a baked robot's root node ``walker``."""
    servo = servo_info(model.config.servo)
    out = {
        "theta_samples": model.n,
        "clip_duration_s": float(clip_duration_s),
        "feet": [f.as_json() for f in model.feet],
        "com": _round(model.com),
        "mass_g": round(float(model.mass_g), 2),
        "servo": {"key": servo["key"], "rpm_max": servo["rpm_max"]},
        "params": model.config.design_json(),
        "z_nominal": model.z_nominal,
        "com_nominal": model.com_nominal,
    }
    if metrics is not None:
        out["metrics"] = metrics
    return jsonable(out)


def api_payload(config: BuildConfig, *, feet_z: Sequence[float] | None = None,
                com: Sequence[float] | None = None, mass_g: float | None = None) -> dict:
    """The ``/api/walk`` response for ``config`` (no parts built).

    An invalid linkage gives ``valid: false`` and the reason in ``error``. A design that
    assembles but whose stability margin dips under :data:`MIN_MARGIN_MM` is ``valid`` (it
    previews and drives on the walking model) but not ``stable``, with the reason in
    ``warning``: MuJoCo may tip it over (the physics drive asks MuJoCo and refuses only
    when it fell). One that covers no ground (under :data:`MIN_STRIDE_MM` per revolution,
    or on fewer than three feet half the time) is not ``walks``, with that in ``warning``.
    """
    servo = servo_info(config.servo)
    base = {
        "valid": True, "error": None, **config.design_json(),
        "theta_samples": N_THETA,
        "links": [list(link) for link in links_of(config.lk)],
        "servo": {"key": servo["key"], "rpm_max": servo["rpm_max"]},
    }
    try:
        model = walker(config, feet_z=feet_z, com=com, mass_g=mass_g)
    except LinkageError as e:
        return jsonable(base | {"valid": False, "error": str(e), "stable": False,
                                "walks": False, "warning": None, "feet": [], "legs": [],
                                "side_z": None, "z_nominal": feet_z is None, "com": None,
                                "mass_g": None, "metrics": None})
    zs = {s: float(np.mean([f.z for f in model.feet if f.side == s])) for s in SIDES}
    metrics = straight_walk_metrics(model, rpm_max=servo["rpm_max"])
    stable = metrics["min_margin_mm"] >= MIN_MARGIN_MM
    walks = metrics["walks"]
    warning = None
    if not walks:
        warning = (f"does not walk: {metrics['stride_mm']:.1f} mm per revolution (under "
                   f"{MIN_STRIDE_MM:g}) and on fewer than three feet "
                   f"{metrics['degenerate_fraction'] * 100:.0f} % of the cycle")
    elif not stable:
        warning = (f"stability margin {metrics['min_margin_mm']:.1f} mm dips under "
                   f"{MIN_MARGIN_MM:g} mm: the robot may tip over (MuJoCo decides)")
    return jsonable(base | {
        "stable": stable,
        "walks": walks,
        "warning": warning,
        "feet": [f.as_json() for f in model.feet],
        "legs": [{"leg": leg.leg, "orientation": leg.orientation,
                  "phase_deg": round(math.degrees(leg.phase), 6),
                  "joints": {j: _round(v) for j, v in leg.joints.items()}}
                 for leg in model.legs],
        "side_z": zs,
        "z_nominal": model.z_nominal,
        "com": _round(model.com),
        "com_nominal": model.com_nominal,
        "mass_g": round(float(model.mass_g), 2),
        "metrics": metrics,
    })
