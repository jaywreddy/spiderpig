"""How a decanting cradle does its job: the swing, the loads, the hand, the stand, the fit.

A cradle (:mod:`construction.cradle`) is judged here, not by the walking model or MuJoCo:

* **The swing**: the tilt range and how still the bottle's mouth stays
  (:meth:`linkage.Linkage.output_check`), each loop's transmission angles.
* **The bottles** (:data:`construction.cradle.SIZES`): each one's setting (the two V
  blocks' rise, the base stop's place: :func:`construction.cradle.setting`) and where its
  mouth sits off the pivot there (:func:`construction.cradle.seat`).
* **The saddles** (:func:`saddles`): the loads on their printed parts and steel (the V
  blocks, the thumbscrews, the post, the lip gauge, the base stop) and the stresses in
  them, a sealed full bottle of each size at its own setting.
* **The loads**: a quasi-static planar solve (:func:`statics`) of the side's bodies, the
  cradle box's mass and a **sealed full 6 L methuselah** (the worst a cradle carries: nothing
  pours out of it) on the cradle, gravity alone, at every crank angle: the crank's torque,
  every pin's force. A **jam** case puts the most the hand crank's slip clutch lets through
  on a blocked cradle (since design review round 1: :func:`jam_nm`; a firm push at the grip,
  :data:`JAM_HAND_N`, had been the case), away from the crank-rocker's toggles (where
  nothing can bound it). The pins are rated by :mod:`spiderpig.strength` at these loads
  ("walk" the bottle's, "jam" the jam's).
* **The clutch's cap** (:func:`clutch_case`): the most the hand crank's slip clutch lets
  onto the crank, however hard the push (``config.PRODUCT_CLUTCH_NM``), on a blocked cradle
  over the whole turn, the toggles included: each part's safety factor as the excluded
  band round the toggles narrows to nothing, and where it falls under 1.3 and 1.
* **The hand**: the crank torque through the worm (its ratio, its static efficiency run in
  and new: the slow end of a hand's speed) at the handle's throw; turns per pour; the slip
  clutch's setting against the worst starting turn, its drift; the gearbox against its
  maximum, its input's and output's radial loads, the keyed plates at the jams; a hand on the
  grip against every part over the whole turn (:func:`hand_sweep`).
* **The rod**: Euler buckling of the connecting rod at its largest compression.
* **The stand**: the centre of mass of the cradle with each bottle, full and empty, at its
  own setting, over the swing, against the feet (the base ties' line along x, the two
  outer plates across).
* **Clashes**: every part against every other at angles across the swing
  (:func:`construction.contract.clashes`), the bottle and the decanter included.
"""

from __future__ import annotations

import itertools
import math
import re
from dataclasses import dataclass, replace

import numpy as np

from spiderpig.config import BuildConfig, is_cradle
from spiderpig.construction import cradle as cr
from spiderpig.construction.handcrank import (
    CRANKPIN_D,
    HAND_RPM,
    HANDLE_R,
    LIGHT_LOAD_N,
    NRV050,
    TOGGLE_TURNS,
    HandCrank,
    WormBox,
    drawn_clutch,
    handle_spec,
)
from spiderpig.cradle_drive import JAM_FLOOR, clutch_case, drive_limits, riser_rating, verdict
from spiderpig.fabricate import design_side, fabricate, template_for
from spiderpig.hardware import mass as mass_mod
from spiderpig.hardware.cradle_catalog import (
    NETRF10_ROOT,
    NETRF10_YIELD,
    NETRF12_ID,
    NETRF12_ROOT,
    ROD_E,
    ROD_ROOT,
    ROD_SHEAR,
    ROD_THREAD,
    ROD_YIELD,
)
from spiderpig.stack import body_class

G = 9.81
SHARE = 1.0             # a pin on both sides of the cradle carries this share of the planar
#                         load: all of it. The rod drives one side, whose pivots carry half
#                         the weight and the rod's whole force (the other side's, half the
#                         weight): the planar sum bounds both (the 2026-10-10 review)
TOGGLE_DEG = 30.0       # the jam case leaves out crank angles this close to a toggle: there
#                         the cradle moves under 1° per 10° of crank (it is at its end), and a
#                         block's force grows without bound
E_ALU = 69000.0         # MPa, 5052 / 6061
WARN_HANDLE_N = LIGHT_LOAD_N    # the grip force a light-load crank takes (MIL-STD-1472H)
JAM_HAND_N = 40.0       # a firm push at the grip against a blocked cradle (about twice the
#                         worst turn's: a hand that feels it stop and pushes on): no bound by
#                         itself, so the jam is the clutch's cap (jam_nm); reported beside it


def push_nm(box: WormBox = NRV050, grip_r: float = HANDLE_R) -> float:
    """The crank torque (N·m) of :data:`JAM_HAND_N` at the grip (``grip_r`` mm) through the
    worm at the hand's fastest (``WormBox.eta_at`` at ``handcrank.HAND_RPM``)."""
    return JAM_HAND_N * grip_r / 1000.0 * box.ratio * box.eta_at(HAND_RPM)


def jam_nm(config: BuildConfig) -> float:
    """The jam case's crank torque (N·m): the most the hand crank's slip clutch lets through
    as drawn (``construction.handcrank.drawn_clutch``'s cap: set on the bench at 1.3 x a full
    6 L's start, checked within its window, through the worm at the hand's fastest; design
    review rounds 1 and 2), however hard the push at the grip: :func:`push_nm`'s firm push is
    no bound (a hand on a crank that feels stuck pushes 60-150 N), the clutch is.
    ``config.PRODUCT_JAM_NM`` is this number."""
    return drawn_clutch(config.linkage)["crank_cap_nm"]


@dataclass
class Rigid:
    """A moving body of the planar model: its joints (side's names) and its mass (kg) with
    the centre of mass written in its frame (``along``, ``across`` from ``a`` toward ``b``)."""

    name: str
    joints: tuple[str, ...]
    a: str
    b: str
    mass: float
    along: float
    across: float


def _frame_coords(p, a, b) -> tuple[float, float]:
    u = (b - a) / max(float(np.linalg.norm(b - a)), 1e-12)
    n = np.array([-u[1], u[0]])
    d = p - a
    return float(d @ u), float(d @ n)


def _from_frame(along, across, a, b):
    u = (b - a) / np.linalg.norm(b - a, axis=-1, keepdims=True)
    n = np.stack([-u[..., 1], u[..., 0]], axis=-1)
    return a + along * u + across * n


def _host(b, by) -> str:
    """The kinematic body a part rides (its ``rigid_with`` chain's end)."""
    seen = 0
    while b.rigid_with is not None and b.rigid_with in by and seen < 20:
        b = by[b.rigid_with]
        seen += 1
    return b.name


def part_mass_kg(config: BuildConfig, b) -> float:
    """A part's mass (kg): :mod:`hardware.mass`'s (a bought part at its catalogued mass; a
    printed one solid, in PETG if it's one of :data:`construction.cradle.PETG_PARTS`)."""
    fil = "petg_filament" if b.name in cr.PETG_PARTS else "pla_filament"
    _, dens, fixed = mass_mod.material_of(b, config.sheet, fil, None)
    if fixed is not None:
        return fixed / 1000.0
    return mass_mod.part_props(b.placed_part()).volume * dens / 1e6


_MASSES: dict = {}


def masses(config: BuildConfig, mech) -> tuple[dict[str, float], dict[str, np.ndarray]]:
    """Per side body class (both sides' parts summed: the motion is planar), its mass (kg)
    and first moment (kg mm, world XY) in the fabricated cradle ``mech`` (its V blocks at
    the drawn bottle's setting, their lowest: the load case's).

    Memoized by everything it reads (the sheet, and each body's name, part object, pose,
    host, fabrication, BOM key and sheet; an entry holds its parts, so their ids stay
    theirs): a report asks it of one cradle for every load case (28 times in B's, 112 of
    its 229 s, measured 2026-10-20), each time integrating every placed part afresh."""
    held = tuple((b.part, getattr(b.part, "wrapped", None)) for b in mech.bodies)
    key = (config.sheet, tuple((b.name, id(p), id(w), b.pose.matrix.tobytes(), b.rigid_with,
                                b.fab, b.bom_key, b.sheet)
                               for b, (p, w) in zip(mech.bodies, held, strict=True)))
    hit = _MASSES.get(key)
    if hit is not None:
        m_of, mom = hit[1]
        return dict(m_of), {k: v.copy() for k, v in mom.items()}
    m_of, mom = _masses(config, mech)
    if len(_MASSES) > 8:
        _MASSES.clear()
    _MASSES[key] = (held, (m_of, mom))
    return dict(m_of), {k: v.copy() for k, v in mom.items()}


def _masses(config: BuildConfig, mech) -> tuple[dict[str, float], dict[str, np.ndarray]]:
    by = {b.name: b for b in mech.bodies}
    m_of: dict[str, float] = {}
    mom: dict[str, np.ndarray] = {}
    for b in mech.bodies:
        if b.part is None or b.fab == "display":
            continue
        cls = body_class(_host(b, by))
        m = part_mass_kg(config, b)
        com = mass_mod.part_props(b.placed_part()).com[:2]
        m_of[cls] = m_of.get(cls, 0.0) + m
        mom[cls] = mom.get(cls, np.zeros(2)) + m * com
    return m_of, mom


def _world_joints(config: BuildConfig, ts) -> dict[str, np.ndarray]:
    return config.lk.solve(1, 0.0, dict(config.proportions)).evaluate(np.asarray(ts, float))


def planar_model(config: BuildConfig, mech, t_ref: float, bottle: cr.BottleSize | None
                 ) -> tuple[list[Rigid], list[tuple[str, str, str]]]:
    """The moving bodies (:class:`Rigid`) and the joints ``(point, body, body-or-ground)``."""
    lk = config.lk
    m_of, moments = masses(config, mech)
    pts = _world_joints(config, [t_ref])
    links = {b: js for b, (js, _) in lk.links.items()}
    links["conn"] = lk.crank
    out = []
    for name, js in links.items():
        a, b = js[0], js[1]
        m = m_of.get(name, 0.0)
        com = moments.get(name, np.zeros(2)) / m if m > 0 else (pts[a][0] + pts[b][0]) / 2
        if name == cr.cradle_link(lk) and bottle is not None:
            y0, slope, x0 = cr.seat(bottle)
            cf_com = np.array([x0 + bottle.com_from_mouth * math.cos(slope),
                               y0 + bottle.com_from_mouth * math.sin(slope)])
            o, ax = cr.mouth_frame(lk)
            lip, d = pts[o][0], pts[ax][0] - pts[o][0]
            tilt = math.atan2(d[1], d[0])
            c, s = math.cos(tilt), math.sin(tilt)
            wcom = lip + np.array([c * cf_com[0] - s * cf_com[1], s * cf_com[0] + c * cf_com[1]])
            com = (com * m + wcom * bottle.mass_full_kg) / (m + bottle.mass_full_kg)
            m += bottle.mass_full_kg
        along, across = _frame_coords(com, pts[a][0], pts[b][0])
        out.append(Rigid(name, tuple(js), a, b, m, along, across))
    ground = set(lk.frame)
    owners: dict[str, list[str]] = {}
    for r in out:
        for j in r.joints:
            owners.setdefault(j, []).append(r.name)
    joints = []
    for j, own in owners.items():
        if j in ground:
            joints += [(j, o, "ground") for o in own]
        elif len(own) >= 2:
            joints += [(j, own[0], o) for o in own[1:]]
    return out, joints


def block_point(config: BuildConfig, ts, at: tuple[float, float]) -> np.ndarray:
    """A point of the cradle at ``at`` in its frame (mm: the mouth at the origin, x along the
    bottle's axis toward its base, y up with the bottle level), in the world per crank angle
    (``decanters.TIES`` are such points)."""
    pts = _world_joints(config, ts)
    o, ax = cr.mouth_frame(config.lk)
    d = pts[ax] - pts[o]
    a = np.arctan2(d[:, 1], d[:, 0])
    c, s = np.cos(a), np.sin(a)
    return pts[o] + np.stack([c * at[0] - s * at[1], s * at[0] + c * at[1]], axis=-1)


def statics(config: BuildConfig, rigid: list[Rigid], joints, ts, jam_nm: float = 0.0, *,
            block: tuple[float, float] | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Per crank angle: the crank torque that holds the cradle under gravity (N·m, positive
    turns the crank forward) and every joint's force (N); with ``jam_nm``, the cradle
    blocked and ``jam_nm`` on the crank instead: its rotation held by a free moment on the
    cradle link (a pure couple: the "torque" returned is that moment, N·m) or, with
    ``block`` (a point in the cradle's frame, :func:`block_point`), by a force there along
    its motion about the mouth (the "torque" returned is that force, N). (Lengths are mm:
    moments are solved in N·mm.)"""
    torques, forces, _ = statics_vectors(config, rigid, joints, ts, jam_nm, block=block)
    return torques, forces


def statics_vectors(config: BuildConfig, rigid: list[Rigid], joints, ts, jam_nm: float = 0.0,
                    *, block: tuple[float, float] | None = None
                    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """:func:`statics` and every joint's force vector, ``(n, joints, 2)``: the force on the
    joint's first body from its second (or from the ground)."""
    lk = config.lk
    pts = _world_joints(config, ts)
    names = [r.name for r in rigid]
    idx = {n: i for i, n in enumerate(names)}
    nj = len(joints)
    n_unk = 2 * nj + 1
    n_eq = 3 * len(rigid)
    crank_i = idx["conn"]
    cradle = cr.cradle_link(lk)
    torques = np.zeros(len(ts))
    forces = np.zeros((len(ts), nj))
    vecs = np.zeros((len(ts), nj, 2))
    q = block_point(config, ts, block) if block is not None else None
    mouth = pts[cr.mouth_name(lk)]
    for k in range(len(ts)):
        A = np.zeros((n_eq, n_unk))
        rhs = np.zeros(n_eq)
        for r in rigid:
            i = idx[r.name]
            a, b = pts[r.a][k], pts[r.b][k]
            com = _from_frame(r.along, r.across, a, b)
            w = -r.mass * G
            rhs[3 * i + 1] -= w                       # sum Fy + w = 0
            rhs[3 * i + 2] -= com[0] * w              # moments about the origin: x Fy - y Fx
        for jk, (j, b1, b2) in enumerate(joints):
            p = pts[j][k]
            for body, sign in ((b1, 1.0), (b2, -1.0)):
                if body == "ground":
                    continue
                i = idx[body]
                A[3 * i, 2 * jk] += sign
                A[3 * i + 1, 2 * jk + 1] += sign
                A[3 * i + 2, 2 * jk] += -sign * p[1]
                A[3 * i + 2, 2 * jk + 1] += sign * p[0]
        if jam_nm:
            # the drive torque is given; the cradle's rotation held by an extra moment, or
            # by a force at the block along the cradle's motion there
            A[3 * crank_i + 2, -1] = 0.0
            rhs[3 * crank_i + 2] -= jam_nm * 1000.0
            c = idx[cradle]
            if q is None:
                A[3 * c + 2, -1] = 1.0
            else:
                r = q[k] - mouth[k]
                n = np.array([-r[1], r[0]]) / max(float(np.linalg.norm(r)), 1e-9)
                A[3 * c, -1], A[3 * c + 1, -1] = n
                A[3 * c + 2, -1] = q[k][0] * n[1] - q[k][1] * n[0]
        else:
            A[3 * crank_i + 2, -1] = 1.0              # the drive's torque on the crank
        sol, *_ = np.linalg.lstsq(A, rhs, rcond=None)
        torques[k] = sol[-1] / (1.0 if block is not None else 1000.0)
        forces[k] = np.hypot(sol[0:2 * nj:2], sol[1:2 * nj:2])
        vecs[k] = sol[:2 * nj].reshape(nj, 2)
    return torques, forces, vecs


def _toggles(config: BuildConfig, ts) -> np.ndarray:
    """Crank angles near a toggle (the tilt's turning points): the jam case skips them."""
    lk = config.lk
    pts = _world_joints(config, ts)
    o, ax = cr.mouth_frame(lk)
    d = pts[ax] - pts[o]
    tilt = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    ends = [int(np.argmin(tilt)), int(np.argmax(tilt))]
    near = np.zeros(len(ts), bool)
    for e in ends:
        diff = np.degrees(np.abs((ts - ts[e] + math.pi) % (2 * math.pi) - math.pi))
        near |= diff < TOGGLE_DEG
    return near


def report(config: BuildConfig, clash_angles: int = 6, store=None) -> dict:
    """The cradle's verdict (the module docstring); ``clash_angles`` 0 skips the clashes."""
    if not is_cradle(config):
        raise ValueError(f"{config.linkage} is not a decanting cradle")
    lk = config.lk
    tmpl = template_for(config)
    design = design_side(tmpl, config)
    ts = np.linspace(0.0, 2.0 * math.pi, 360, endpoint=False)
    out: dict = {"linkage": config.linkage, "name": lk.name}

    # the swing
    oc = lk.output_check(dict(config.proportions))
    pts = _world_joints(config, ts)
    o, ax = cr.mouth_frame(lk)
    d = pts[ax] - pts[o]
    tilt = np.degrees(np.unwrap(np.arctan2(d[:, 1], d[:, 0])))
    steps = [s for s in lk.check(dict(config.proportions)) if s.kind == "closure"]
    out["swing"] = {
        "rest_deg": round(float(tilt.min()), 2), "full_deg": round(float(tilt.max()), 2),
        "swing_deg": round(float(np.ptp(tilt)), 2),
        "mouth_wander_mm": round(float(oc.wander_mm or 0.0), 6),
        "transmission_deg": {s.point: [round(v, 1) for v in s.angle_deg] for s in steps},
        "worst_transmission_deg": round(min(min(s.angle_deg[0], 180 - s.angle_deg[1])
                                            for s in steps), 1),
    }
    # the bottles in the saddles, each at its own setting: how far its mouth moves
    out["bottles"] = []
    swing = math.radians(float(np.ptp(tilt)))
    for size in cr.SIZES:
        y0, slope, x0 = cr.seat(size)
        r = math.hypot(x0, y0)
        out["bottles"].append({"bottle": size.name, "mouth_off_pivot_mm":
                               [round(x0, 2), round(y0, 2)],
                               "mouth_wander_mm": round(2 * r * math.sin(swing / 2), 2),
                               "nose_up_deg": round(-math.degrees(slope), 2),
                               "setting_mm": _setting_row(size),
                               "full_kg": size.mass_full_kg})

    mech = fabricate(tmpl, config, 1.0, store=store)
    # the loads: a sealed full 6 L methuselah; the jam: the hand crank's clutch's cap
    rigid, joints = planar_model(config, mech, 1.0, cr.DESIGN)
    torque, forces = statics(config, rigid, joints, ts)
    near = _toggles(config, ts)
    _, jam_f = statics(config, rigid, joints, ts, jam_nm=jam_nm(config))
    jam_f = np.where(near[:, None], 0.0, jam_f)
    box = HandCrank().box
    peak = float(np.abs(torque).max())
    handle = peak / (box.ratio * box.eta_static)
    notes = mech.meta.get("hand_crank") or {}
    spec = handle_spec(config)
    grip_r = spec.throw
    out["drive"] = {
        "gearbox": box.key, "ratio": box.ratio, "static_efficiency": box.eta_static,
        "irreversible": box.irreversible(),
        "crank_torque_peak_nm": round(peak, 2), "gearbox_rating_nm": box.rated_nm,
        "gearbox_sf": round(box.rated_nm / max(peak, 1e-9), 2),
        "handle_torque_nm": round(handle, 3),
        "handle_force_n": round(handle / (grip_r / 1000.0), 1),
        "grip_radius_mm": grip_r,
        "jam_torque_nm": round(jam_nm(config), 2),
        "handle_turns_per_pour": round(box.ratio / 2.0, 1),
        "deg_per_handle_turn_mean": round(float(np.ptp(tilt)) / (box.ratio / 2.0), 2),
        "load_case": f"a sealed full {cr.DESIGN.name} ({cr.DESIGN.mass_full_kg} kg)",
    }
    # the empty cradle (the bottle out): the bench check after it goes back together
    rigid0, joints0 = planar_model(config, mech, 1.0, None)
    empty, _ = statics(config, rigid0, joints0, ts)
    out["drive"]["empty_handle_force_n"] = round(
        float(np.abs(empty).max()) / (box.ratio * box.eta_static) / (grip_r / 1000.0), 2)
    out["drive"].update(drive_limits(box, spec, peak, handle, notes))
    if out["drive"]["handle_force_n"] > WARN_HANDLE_N:
        out.setdefault("warnings", []).append(
            f"the handle needs {out['drive']['handle_force_n']} N at its grip at worst")
    rider = HandCrank.rider(design.ctx)
    rows = joint_rows(config, rider, joints, forces, jam_f)
    out["joints"] = rows
    # the rod's buckling at its largest force (compression or not: the bound)
    rod_len = float(np.linalg.norm(pts[lk.crank[1]][0] - pts[
        next(j for j in lk.links[rider][0] if j != lk.crank[1])][0]))
    t_rod = design.ctx.sheet_t("link", rider)
    far = next(j for j in lk.links[rider][0] if j != lk.crank[1])
    from spiderpig.config import chord_r
    from spiderpig.materials import allowable_mpa, link_sheets

    w = 2 * chord_r(config, rider, lk.crank[1], far)
    inertia = w * t_rod ** 3 / 12.0
    p_cr = math.pi ** 2 * E_ALU * inertia / rod_len ** 2
    rod_jk = next(jk for jk, (j, b1, b2) in enumerate(joints) if j == lk.crank[1])
    f_rod = float(forces[:, rod_jk].max())
    f_rod_jam = float(jam_f[:, rod_jk].max())
    # its far pin's span: the force comes in half of it off the rod's mid-plane
    span = float(((mech.meta.get("wobble") or {}).get(f"pin:{far}") or {}).get("span_mm", 0.0))
    out["rod"] = {"length_mm": round(rod_len, 1), "section_mm": [round(w, 2), round(t_rod, 3)],
                  "euler_n": round(p_cr, 1), "bottle_n": round(f_rod, 1),
                  "jam_n": round(f_rod_jam, 1), "sf_bottle": round(p_cr / max(f_rod, 1e-9), 2),
                  "sf_jam": round(p_cr / max(f_rod_jam, 1e-9), 2),
                  "eccentricity_mm": round(span / 2, 2),
                  "yield_mpa": allowable_mpa(config, link_sheets(config).get(rider,
                                                                             config.sheet))}
    for tag, f in (("bottle", f_rod), ("jam", f_rod_jam)):
        out["rod"][f"sf_secant_{tag}"] = round(float(rod_secant_sf(np.array([f]),
                                                                    out["rod"])[0]), 2)
    # the gearbox's output shaft: the rod's force at its plane, walking (the toggles' at the
    # cap: jam_capped's rows)
    outp = out["drive"]["output_overhung"]
    outp["walk_n"] = round(f_rod, 1)
    outp["sf_walk"] = round(outp["allowed_n"] / max(f_rod, 1e-9), 2)
    # the gearbox's riser walking (its cap's: jam_capped's rows; design review round 3's M2)
    out["drive"]["riser"] = {
        "walk": riser_rating(box, notes, f_rod, peak),
        "basis": "a clamped stack (ring, sleeve, shims, plate) on each of four class 8.8 M8 "
                 "studs: the stud's tension and shear, the sleeve's bending, the ring's "
                 "bearing; its slip at the faces' friction (cradle_drive.riser_rating)"}
    # the crankpin: a NETRF12 standing in the crank block, the rod's force at the rod's
    # mid-plane over the block's top (a cantilever that long out of its socket): it bends at
    # the block's top, taken as the 12 x 5.0 tube its tapped end may be there, at 1018's
    # hot-rolled minimum yield
    note = mech.meta.get("hand_crank") or {}
    rod_mid = design.plan.z(design.plan.layers[rider])[0] + design.ctx.sheet_t(
        "link", rider) / 2
    lever = float(note.get("crankpin_lever_mm", rod_mid - (note.get("arm_z_mm") or
                                                           [0.0, 0.0])[1]))
    od, bore = CRANKPIN_D, NETRF12_ID
    cap = math.pi * (od ** 4 - bore ** 4) / (32 * od) * NETRF10_YIELD / 1000.0  # N·m
    out["crankpin"] = {"lever_mm": round(lever, 2), "capacity_nm": round(cap, 2),
                       "bottle_nm": round(f_rod * lever / 1000.0, 2),
                       "jam_nm": round(f_rod_jam * lever / 1000.0, 2),
                       "sf_bottle": round(cap * 1000.0 / max(f_rod * lever, 1e-9), 2),
                       "sf_jam": round(cap * 1000.0 / max(f_rod_jam * lever, 1e-9), 2)}
    out["stand"] = stability(config, mech, design, ts)
    struct = structure(config, mech, ts=ts)
    out["strength"] = strength_check(config, mech, rows, peak, struct)
    out["structure"] = {k: v for k, v in struct.items() if k not in ("rated", "pillars")}
    out["blocked"] = blocked(config, mech, ts=ts)
    cap = out["drive"]["clutch"]["crank_cap_nm"]
    out["jam_capped"] = clutch_case(config, mech, ts, tilt, cap, out["rod"], out["crankpin"],
                                    out["drive"]["jam_push_nm"],
                                    out["drive"]["output_overhung"]["allowed_n"],
                                    riser=(box, notes))
    # at a fast spin the same slip puts more on the crank (design review round 3's M1): the
    # parts' least there, scaled from the cap's 30° from the toggles
    clutch_row = out["drive"]["clutch"]
    clutch_row["fast_cap_least_sf"] = round(
        out["jam_capped"]["away"][0] * cap / clutch_row["fast_cap_nm"], 2)
    out["drive"]["riser"]["cap"] = riser_rating(
        box, notes, out["jam_capped"]["by_band"][-1]["rod_n"], cap)
    out["blocked_capped"] = blocked(config, mech, ts=ts, crank_nm=cap, thresholds=True,
                                    rod_euler_n=out["rod"]["euler_n"])
    out["verdict"] = verdict(lk.key, out)
    moving = {*cr.REAR_MOVES, *cr.STOP_MOVES}
    out["saddles"] = saddles(float(tilt.min()), float(tilt.max()), sum(
        part_mass_kg(config, b) for b in mech.bodies if b.name in moving))
    got = keepers(config, mech)
    if got:
        out["keepers"] = got
    out["take_up"] = take_up(config, mech)
    if clash_angles:
        from spiderpig.construction.contract import clashes

        found = []
        ends = [float(ts[int(np.argmin(tilt))]), float(ts[int(np.argmax(tilt))])]
        for t in [*ends, *np.linspace(0.0, 2.0 * math.pi, clash_angles, endpoint=False)]:
            m = fabricate(tmpl, config, float(t), store=store)
            found.extend(dict(c, crank_deg=round(math.degrees(t), 1)) for c in clashes(m))
            found.extend(dict(c, crank_deg=round(math.degrees(t), 1)) for c in wheel_sweep(m))
        out["clashes"] = found
    return out


def rod_secant_sf(f: np.ndarray, rod: dict) -> np.ndarray:
    """The rod's safety factor as a beam-column out of its plane (design review round 2's
    minor 1): its far pin spans two layers, so its force ``f`` (N, per case) comes in
    ``rod["eccentricity_mm"]`` off its mid-plane there, an end moment ``M0 = f e``; pinned at
    the crankpin, its largest moment is ``M0 / sin(k L)`` (``k L = pi sqrt(f / P_cr)``, past
    a quarter wave; else ``M0`` at the end), on its weak axis with its axial stress, against
    its sheet's yield (0 at or past Euler)."""
    w, t = rod["section_mm"]
    e, y, pcr = rod["eccentricity_mm"], rod["yield_mpa"], rod["euler_n"]
    f = np.maximum(np.asarray(f, float), 1e-9)
    ratio = np.minimum(f / pcr, 0.999999)
    kl = math.pi * np.sqrt(ratio)
    amp = np.where(kl > math.pi / 2, 1.0 / np.sin(kl), 1.0)
    sig = f / (w * t) + amp * f * e / (w * t * t / 6.0)
    return np.where(f >= pcr, 0.0, y / sig)


def _setting_row(size: cr.BottleSize) -> list[float]:
    """``size``'s setting (``construction.cradle.setting``): the front V block's rise, the
    rear one's, the base stop's face along the axis from the mouth (mm)."""
    s = cr.setting(size)
    return [round(s.front, 2), round(s.rear, 2), round(s.stop, 1)]


WINE_KG = {"standard": 0.75, "magnum": 1.5, "jeroboam": 3.0, "imperial": 6.0,
           "methuselah": 6.0}
"""The wine in each bottle size full (``construction.cradle.SIZES``), kg: what an empty one
weighs less."""


def stability(config: BuildConfig, mech, design, ts) -> dict:
    """The whole cradle's centre of mass over the swing, each bottle full and empty, against
    the base board's feet (their pads' outer edges; the board :func:`cradle.base_z` off the
    bay's middle): its margin (mm) to the nearest tipping edge, front, rear and each side (the
    gearbox and the hand wheel weigh on the drive side, -z), the tilt it tips at and the push
    at the bottle's highest point that tips it, each the worst over the swing (the box's parts
    measured as ``mech`` was built, turned with it); and per edge (``edges``) its worst tilt
    and least push, with the bottle that sets each."""
    lk = config.lk
    stand_ = cr.stand_of(lk)
    by = {b.name: b for b in mech.bodies}
    link = cr.cradle_link(lk)
    pts = _world_joints(config, ts)
    o, ax = cr.mouth_frame(lk)
    meta = mech.meta or {}
    if "cradle_lip" in meta:                  # the pose the parts were built in
        lip0, t0 = np.asarray(meta["cradle_lip"], float), float(meta["cradle_tilt"])
    else:
        d0 = pts[ax][0] - pts[o][0]
        lip0, t0 = pts[o][0], math.atan2(d0[1], d0[0])
    table = float(lip0[1]) - cr.LIP_HEIGHT
    # every part's mass and centre: what stands, what turns with the box (about the mouth)
    still_m, still_c = 0.0, np.zeros(3)
    box_m, box_c = 0.0, np.zeros(3)
    moves = {**dict.fromkeys(cr.FRONT_MOVES, "front"), **dict.fromkeys(cr.REAR_MOVES, "rear"),
             **dict.fromkeys(cr.STOP_MOVES, "stop")}
    moved_m: dict[str, float] = dict.fromkeys(("front", "rear", "stop"), 0.0)  # (it moves)
    for b in mech.bodies:
        # (the decanter's riser stands on the table in its locator, not on the board)
        if b.part is None or b.fab == "display" or b.name.startswith("decanter_riser"):
            continue
        m = part_mass_kg(config, b)
        c = np.asarray(mass_mod.part_props(b.placed_part()).com, float)
        if body_class(_host(b, by)) == link:
            box_m += m
            box_c += m * c
            if b.name in moves:
                moved_m[moves[b.name]] += m
        else:
            still_m += m
            still_c += m * c
    box_rel0 = (box_c / box_m)[:2] - lip0 if box_m else np.zeros(2)
    box_z = float(box_c[2] / box_m) if box_m else 0.0
    inset = cr.FOOT_INSET - cr.FOOT_PAD_R        # the feet (construction.cradle._base_plate)
    x_front = float(lip0[0]) + stand_.base[0] + inset
    x_rear = float(lip0[0]) + stand_.base[1] - inset
    z_mid = cr.base_z(lk)
    z_drive, z_passive = z_mid - (cr.BASE_HALF - inset), z_mid + (cr.BASE_HALF - inset)
    rows = []
    edges: dict[str, dict] = {}
    pouring_edges: dict[str, dict] = {}
    for size in cr.SIZES:
        # the box's centre with its V blocks and base stop at this bottle's setting
        d = cr.offsets(size)
        shift = (moved_m["front"] * np.array([0.0, d.front])
                 + moved_m["rear"] * np.array([0.0, d.rear])
                 + moved_m["stop"] * np.array([d.stop, d.rear])) / max(box_m, 1e-9)
        box_rel = box_rel0 + np.array([math.cos(t0) * shift[0] - math.sin(t0) * shift[1],
                                       math.sin(t0) * shift[0] + math.cos(t0) * shift[1]])
        y0, slope, x0 = cr.seat(size)
        com_cf = np.array([x0 + size.com_from_mouth * math.cos(slope),
                           y0 + size.com_from_mouth * math.sin(slope)])
        top_cf = np.array([x0 + size.length * math.cos(slope),
                           y0 + size.length * math.sin(slope) + size.r])
        wine = WINE_KG[size.key]
        full, empty = size.mass_full_kg, size.mass_full_kg - wine
        for state in ("full", "empty", "pouring"):
            least: dict[str, tuple] = {}       # tip, push, margin: each its worst (and edge)
            mb = full if state == "full" else empty
            for k in range(len(ts)):
                lip = pts[o][k]
                d = pts[ax][k] - pts[o][k]
                tl = math.atan2(d[1], d[0])
                if state == "pouring":          # full while neck up; past level it has
                    mb = full if tl <= 0.0 else empty   # poured (what it keeps sits forward)
                c, s_ = math.cos(tl - t0), math.sin(tl - t0)
                box = lip + np.array([c * box_rel[0] - s_ * box_rel[1],
                                      s_ * box_rel[0] + c * box_rel[1]])
                c2, s2 = math.cos(tl), math.sin(tl)
                bot = lip + np.array([c2 * com_cf[0] - s2 * com_cf[1],
                                      s2 * com_cf[0] + c2 * com_cf[1]])
                top = lip[1] + s2 * top_cf[0] + c2 * top_cf[1]
                m = still_m + box_m + mb
                cx = (still_c[0] + box_m * box[0] + mb * bot[0]) / m
                cy = (still_c[1] + box_m * box[1] + mb * bot[1]) / m
                cz = (still_c[2] + box_m * box_z) / m        # the bottle on the bay's middle
                h = cy - table
                for edge, margin in (("front", cx - x_front), ("rear", x_rear - cx),
                                     ("drive side", cz - z_drive),
                                     ("passive side", z_passive - cz)):
                    got = {"tip": math.degrees(math.atan2(margin, h)),
                           "push": m * G * margin / max(float(top - table), 1.0),
                           "margin": margin}
                    for key, v in got.items():
                        if key not in least or v < least[key][0]:
                            least[key] = (v, edge)
                    for into, which in ((edges, ("full", "empty")),
                                        (pouring_edges, ("empty", "pouring"))):
                        if state not in which:
                            continue
                        e = into.setdefault(edge, {})
                        for key, v in (("tip_deg", got["tip"]),
                                       ("push_at_top_n", got["push"])):
                            if key not in e or v < e[key][0]:
                                e[key] = (v, f"{size.name}, {state}")
            if state == "pouring":
                continue                        # (its own edges: ``pouring``)
            rows.append({"bottle": size.name, "state": state, "kg": round(mb, 2),
                         "margin_mm": round(least["margin"][0], 1), "edge": least["tip"][1],
                         "tip_deg": round(least["tip"][0], 1),
                         "push_at_top_n": round(least["push"][0], 1),
                         "push_edge": least["push"][1]})
    return {"feet": {"x_mm": [round(x_front, 1), round(x_rear, 1)],
                     "z_mm": [round(z_drive, 1), round(z_passive, 1)]},
            "board_z_mm": z_mid,
            "device_kg": round(still_m + box_m, 3),
            "cases": rows, "worst_margin_mm": min(r["margin_mm"] for r in rows),
            "worst_tip_deg": min(r["tip_deg"] for r in rows),
            "edges": {e: {"tip_deg": round(v["tip_deg"][0], 1), "tip_by": v["tip_deg"][1],
                          "push_at_top_n": round(v["push_at_top_n"][0], 1),
                          "push_by": v["push_at_top_n"][1]} for e, v in edges.items()},
            # the wine leaving as it pours (design review round 2's m1): each bottle full
            # while neck up, emptied past level (what it keeps there sits forward, in the
            # shoulder: it only steadies the rear); "edges" counts a full bottle at every
            # tilt, a bound no pour reaches
            "pouring": {e: {"tip_deg": round(v["tip_deg"][0], 1), "tip_by": v["tip_deg"][1],
                            "push_at_top_n": round(v["push_at_top_n"][0], 1),
                            "push_by": v["push_at_top_n"][1]}
                        for e, v in pouring_edges.items()}}


def stand(linkage: str = "decanter_rcm", store=None) -> dict:
    """:func:`stability` alone, for a quick re-run after a change to the masses, the bottles'
    centres of mass or the board (``python -m spiderpig.cradle_report stand [linkage]``):
    the cradle fabricated as :func:`report` does, its stand's verdict per edge."""
    config = BuildConfig(linkage=linkage, robot=False)
    tmpl = template_for(config)
    design = design_side(tmpl, config)
    mech = fabricate(tmpl, config, 1.0, store=store)
    ts = np.linspace(0.0, 2.0 * math.pi, 360, endpoint=False)
    return stability(config, mech, design, ts)


PRINTED = {
    ("pla", "in"): 51.0, ("pla", "across"): 17.0,
    ("petg", "in"): 47.0, ("petg", "across"): 18.0,
}
"""MPa: a printed part's strength, solid (100 % infill): in its layers (Prusament's
tensile yield of printed specimens, horizontal, ISO 527-1) and across them (their
interlayer adhesion): Prusament PLA's datasheet v1.1 of 2022-07-27 (51 ± 3 and 17 ± 3)
and PETG's v1.1 of 2022-02-16 (47 ± 2 and 18 ± 4), the specimens printed solid. A part
in bearing (pressed, not pulled) takes the in-layer figure."""
SUSTAINED_SF = 4.0      # a load that stays (a bottle at rest, a lock's preload): a quarter of
#                         the cited strength (it creeps, it doesn't break)
TRANSIENT_SF = 2.5      # a load of minutes or less (the pour, a knock): about 2 on the
#                         strength's low end (the mean less its spread)
PLA_ALLOW = PRINTED["pla", "across"] / SUSTAINED_SF
"""MPa: what a printed PLA part takes across its layers under a load that stays, 4.25 (a
quarter of Prusament's 17; review round 2, N6: it had been 10, called a quarter of a ~40
MPa that no datasheet gives). The saddles, the gauge and the base stop are PETG now
(:data:`construction.cradle.PETG_PARTS`), rated by :data:`PRINTED`."""
STAINLESS_YIELD = 210.0     # MPa: the thumbscrews' 18-8, as annealed 304 (no class named)
CORKSCREW_N = 13.0      # a corkscrew's push into the cork (the mechanical review: 6-13 N)
LOCK_TORQUE_NM = (0.5, 1.0)     # the base stop's knob (26 mm) turned tight by hand: the least
#                                 (its grip) and the most (its lips' stress) a hand gives it
NUT_FACTOR = 0.2        # a dry steel screw's preload: torque / (factor x diameter)
PETG_STEEL = {"dry": (0.24, 0.30), "oiled": (0.13, 0.17)}
"""Printed PETG on steel, static friction, the least and the most over 30-70 N: Stoimenov
et al., "Static and Kinetic Friction of 3D Printed Polymers and Composites", Tribology in
Industry 46(1), 2024, figs. 8 and 9 (dry, and with SAE 15W40 oil). A grip is rated at the
least, a jam at the most. (Printed PLA, the same paper: 0.20-0.30 dry, 0.16-0.20 oiled.)"""
GREASED = 0.10          # a face the screw's grease has reached (review round 2, N5: under
#                         the paper's oiled least)
TPU_FRICTION = 0.5      # the rear lead screw's TPU 95A brake washer on aluminium and PLA, dry
TPU_WET = 0.30          # TPU sliding wet (design review round 3, mounting m1): pure TPU in
#                         distilled water on tin bronze, its basalt-fibre composite's 0.088
#                         being "70.57% lower" (Yu, Jiang, Yu and Guo, "Tribological
#                         Performance of Short Fibers Reinforced Thermoplastic Polyurethane
#                         Composite Materials Under Water-Lubricated Condition", Polymers
#                         17(1):30, 2024): a wet liner's least kinetic friction
SLIDE_GAP = 4.0         # mm: the most the setting step's check leaves between the rim and
#                         the lip gauge's pad at rest ("2 to 4 mm in front of it")
LEAD_FRICTION = 0.08    # its bronze nut on the stainless screw, greased: the least it may be
KNOCK_N = 50.0          # a knock sideways on the rear knob (a hand, a bottle going in)
GRIP_BAR = 2.0          # a grip (the base stop's lock) over what it holds, at the least
#                         friction cited
DRAWER_BAR = 2.0        # a guide's jamming friction over the most cited for its faces
E_PETG = 1500.0         # MPa: printed PETG's tensile modulus (Prusament, horizontal)
MOUTH_LIFT_MAX = 1.0    # mm: the most the rails' sag may lift the mouth (balance)
BASE_RING = 10.0        # a bottle's base rests on a ring this far in from its body's radius
HEEL = 10.0             # its body's last 10 mm, rounded into the base, rest on nothing


TPU_E = 26.0            # MPa: TPU 95A's tensile modulus, the least published for it printed
#                         (UltiMaker's TDS v5.00, ASTM D638: 26 MPa; its v2.01 of 2025-09-15,
#                         ASTM D3039: 56-67 MPa by the print's orientation)
TPU_NU = 0.45           # its Poisson's ratio (an elastomer: under 0.5)
TPU_STEEL_FRICTION = 0.3    # a keeper's grip on a stainless pin: under the 0.5 of TPU on
#                             aluminium and PLA dry (TPU_FRICTION), for an oiled pin
KEEPER_LEAST_RATIO = 10.0   # the keepers' bar: the least grip over the pin's own weight
KEEPER_LOAD_RATIO = 1.5     # and the bench push (cradle.KEEPER_PUSH_N) over the most the steps
#                             can put on a pin's tip: half the cradle box's weight
KEEPER_PUSH_MAX_N = 120.0   # its push on, at the bore's nominal interference: by hand, a tube
#                             over the pin (round 2: 80-105 N dry at the 7.4 mm bore); at the
#                             tight end too (the packing review's m1, pack turn 2)
KEEPER_BORE_TOL = 0.15      # the keeper's bore as printed, either way (cradle.KEEPER's least
#                             interference is the pin's least less the bore's most)
PIN_MOST = 7.99             # the clevis pin's most (McMaster's 8 mm, -0.05/-0.01)


def keepers(config: BuildConfig, mech) -> dict:
    """The keepers on the cradle link's quick-release pins (``construction.cradle.KEEPER``,
    ``mech.meta["keepers"]``; B's X and Y, both sides): each a TPU ring over the 8 mm pin,
    its bore under the pin. Its grip is the friction of the pressure an interference fit
    makes (Lamé, a ring on a rigid shaft: ``p = delta E / (d ((D^2 + d^2) / (D^2 - d^2) +
    nu))``) over the bore's area, at the least interference a print may come out with and
    the least published modulus (:data:`TPU_E`). Against the pin's own weight (its side
    lying down, the clip out: the head is under the link) and against the most the
    reassembly can put on a pin's tip (``load_n``: half the cradle box's weight, if a link
    is let rest on one while the other is found: design review round 2), which the guide's
    bench push (``construction.cradle.KEEPER_PUSH_N``) screens every keeper for and the steps
    keep off (they hold a piece's weight until both pins are in). Its push on: the same at
    the bore's nominal interference, TPU's highest modulus and dry friction. Empty for a
    cradle with none (A)."""
    from spiderpig.construction.pivots.clevis import CLEVIS8

    names = list(mech.meta.get("keepers") or [])
    if not names:
        return {}
    by = {b.name: b for b in mech.bodies}
    d = CLEVIS8.shaft.d
    link = cr.cradle_link(config.lk)
    box_kg = sum(part_mass_kg(config, b) for b in mech.bodies
                 if b.part is not None and b.fab != "display"
                 and body_class(_host(b, by)) == link)
    load = box_kg * G / 2.0

    def normal(delta: float, e: float, od: float, h: float) -> float:
        """The bore's normal force (N) at interference ``delta`` (mm, on the diameter)."""
        p = delta * e / (d * ((od ** 2 + d ** 2) / (od ** 2 - d ** 2) + TPU_NU))
        return p * math.pi * d * h

    tight = PIN_MOST - (cr.KEEPER[0] - KEEPER_BORE_TOL)     # (the tightest a print may be)
    rows = []
    for n in names:
        bb = by[n].placed_part().bounding_box()
        h = bb.size.Z
        od = max(bb.size.X, bb.size.Y)
        w = part_mass_kg(config, by[cr.keeper_pin(n)]) * G
        least = TPU_STEEL_FRICTION * normal(cr.KEEPER[1], TPU_E, od, h)
        rows.append({"keeper": n, "od_mm": round(od, 2), "height_mm": round(h, 2),
                     "pin_weight_n": round(w, 3), "grip_least_n": round(least, 2),
                     "push_on_n": round(TPU_FRICTION * normal(d - cr.KEEPER[0], 67.0, od, h),
                                        1),
                     "push_on_tight_n": round(TPU_FRICTION * normal(tight, 67.0, od, h), 1),
                     "ratio": round(least / max(w, 1e-9), 1)})
    # the way back (the packing review's M2): the cradle box lowered onto the drive side lying
    # down, its first pin into its hole, then turned about it until the second lines up:
    # the box's weight on the second pin's tip, about the first (gravity along the pins;
    # the box's centre of mass in their plane); <= 0, the box lifts off it
    box = [b for b in mech.bodies if b.part is not None and b.fab != "display"
           and body_class(_host(b, by)) == link]
    kg = [part_mass_kg(config, b) for b in box]
    com = np.sum([k * np.asarray(mass_mod.part_props(b.placed_part()).com, float)
                  for k, b in zip(kg, box, strict=True)], axis=0) / max(sum(kg), 1e-9)
    pins = cr.trip_pins(config.lk)
    side = next((s for s in "LR" if f"{s}.conn" in by), "L")
    tips = {}
    if len(pins) == 2:
        at = {}
        for j in pins:
            bb = by[f"{side}.pin_{j}_pin"].placed_part().bounding_box()
            at[j] = np.array([(bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2])
        for first, second in (pins, pins[::-1]):
            v = at[second] - at[first]
            gap = float(np.linalg.norm(v))
            tips[f"{first} then {second}"] = round(
                box_kg * G * float((com[:2] - at[first]) @ v) / gap ** 2, 1)
    return {"bore_mm": cr.KEEPER[0], "pin_d_mm": d, "least_interference_mm": cr.KEEPER[1],
            "tpu_e_mpa": TPU_E, "friction": TPU_STEEL_FRICTION, "rows": rows,
            "least_ratio": min(r["ratio"] for r in rows), "bar": KEEPER_LEAST_RATIO,
            "box_kg": round(box_kg, 2), "load_n": round(load, 1),
            "bench_push_n": cr.KEEPER_PUSH_N,
            "load_ratio": round(cr.KEEPER_PUSH_N / load, 2), "load_bar": KEEPER_LOAD_RATIO,
            "least_over_load": round(min(r["grip_least_n"] for r in rows) / load, 2),
            "push_on_bar_n": KEEPER_PUSH_MAX_N, "tight_mm": round(tight, 2),
            "second_tip_n": tips}


def take_up(config: BuildConfig, mech) -> dict:
    """Each pin's axial take-up against its links' sheets as they may arrive (design review
    round 2's M5): a pin (``mech.meta["wobble"]``'s ``pin:`` notes, and since round 3's
    laser-cut parts minor 9 its pillars' ``pillar:``, a row ``"pillar O2"``: its links and its
    play, the barrel or the length between the rings less the stack as drawn, the printed
    parts' tolerance already in) takes its links' thickness over the nominal; each link's
    sheet is over by up to its published tolerance (``thickness_tol``, SendCutSend's spec
    tables: 0.250 in 5052 +0.027 in, 0.187 in +0.009 in, 7075 0.190 in +0.007 in). Per pin:
    the play, the links and their sheets, the growth worst case (every link at its most)
    and root-sum-square, and the margins (the play less each). A negative worst-case margin
    is a ring that may not go into its groove or a clip whose hole is covered, with no
    adjustment in the part: the guide calipers the links on arrival (and sands the pin's
    printed ring or spacer by the excess) and ORDER.md orders the made-to-length pins from
    the measured stack (``hardware.order.MEASURED_PINS``). A rating, not a bar."""
    from spiderpig.hardware.catalog import get
    from spiderpig.materials import sheet_of

    rows = []
    for name, note in sorted((mech.meta.get("wobble") or {}).items()):
        if not name.startswith(("pin:", "pillar:")) or not note.get("links"):
            continue
        links, overs = [], []
        for ln in note["links"]:
            key = sheet_of(config, "link", ln["link"])
            over = float((get(key).dims.get("thickness_tol") or (0.0, 0.0))[0])
            overs.append(over)
            links.append({"link": ln["link"], "sheet": key, "over_mm": over})
        play = float(note.get("play_mm") or 0.0)
        worst, rss = sum(overs), math.sqrt(sum(o * o for o in overs))
        rows.append({"pin": name[4:] if name.startswith("pin:") else name.replace(":", " "),
                     "kind": str((note.get("section") or {}).get("name", "")),
                     "play_mm": round(play, 3), "links": links,
                     "growth_worst_mm": round(worst, 3), "growth_rss_mm": round(rss, 3),
                     "margin_worst_mm": round(play - worst, 3),
                     "margin_rss_mm": round(play - rss, 3)})
    short = [r["pin"] for r in rows if r["margin_worst_mm"] < 0]
    return {"rows": rows, "short_worst": short,
            "short_rss": [r["pin"] for r in rows if r["margin_rss_mm"] < 0],
            "least_margin_mm": min((r["margin_worst_mm"] for r in rows), default=None),
            "basis": "SendCutSend's published thickness tolerances (spec tables, 2026-10-15) "
                     "over each pin's play as drawn",
            "fix": "the guide calipers each link at its pin holes on arrival and sands a "
                   "pin's printed ring or spacer by its excess; ORDER.md orders the "
                   "made-to-length pins after, at the measured stack + 0.3 mm"
                   if short else ""}


def balance(rest_deg: float = -50.0, full_deg: float = 30.0) -> dict:
    """Where the saddles' reaction on each bottle centres along its axis (from the mouth),
    at rest, lying level and at full tilt, against the support it rests on: the front V's
    front end to the rear support's end (the rear V and the base stop's rails' flanges
    behind it, as far as the body's heel). At rest the gate pushes the base back below the
    axis (as low as it can: the base's ring or the gate's bottom edge, or the V's grip on
    the body, whichever is lower), at full tilt the lip gauge pushes the rim's top forward
    above it: either is a couple with the weight's pull along the axis, and moves the
    centre toward the base by its depth x tan(tilt) (review round 1, B1). The centre of mass:
    ``BottleSize.com_from_mouth`` (round 1's shell-and-wine model). A centre past the
    support's end lifts the neck off the front V: ``ok`` false.

    The rear V block is rigid to its back end (``rigid_end_mm``, 380); behind it the rails'
    flanges bend (review round 2, N3: they had been taken as rigid). A centre behind 380
    rests the bottle on that end and the flanges' end, the flanges taking ``flange_n``
    (both, at the end, the bottle pivoting on the block's end: the most they can take
    there); they sag ``sag_mm`` there (each rail a cantilever from its socket, its tongue
    held without play, :func:`rail_section`, printed PETG's modulus) and the bottle tips
    about the block's end, lifting the mouth ``mouth_lift_mm``: ``ok`` false over
    :data:`MOUTH_LIFT_MAX`. As the pour moves the centre in front of 380 the lift goes
    back to nothing, smoothly."""
    front, rear = cr.SADDLES
    x0, xr = front.vee[0], rear.vee[1]
    flange_end = cr.FLANGE_END - cr.CLEAR
    _, rail_i, _ = rail_section()
    rows = []
    for size in cr.SIZES:
        at = cr.setting(size)
        end = min(flange_end, size.length - HEEL)
        com = size.com_from_mouth
        gate = min(size.r - BASE_RING, -(cr.GATE_BOTTOM + at.rear))
        depth_rest = max(gate, size.r / math.sqrt(2))
        rim = (cr.GAUGE_PAD[0] + size.r_neck) / 2
        for case, deg, depth in (("rest", rest_deg, depth_rest), ("level", 0.0, 0.0),
                                 ("full", full_deg, rim)):
            x = com + depth * math.tan(math.radians(abs(deg)))
            wn = size.mass_full_kg * G * math.cos(math.radians(deg))
            on = wn * (x - xr) / (end - xr) if x > xr and end > xr else 0.0
            span = max(end - xr, 0.0)
            sag = on / 2 * span ** 3 / (3 * E_PETG * rail_i)
            lift = sag * xr / span if span else 0.0
            rows.append({"bottle": size.name, "case": case, "tilt_deg": round(deg, 1),
                         "com_mm": round(com, 1), "couple_depth_mm": round(depth, 1),
                         "centre_mm": round(x, 1), "support_mm": [x0, round(end, 1)],
                         "margin_mm": round(end - x, 1), "rigid_end_mm": xr,
                         "rigid_margin_mm": round(xr - x, 1), "flange_n": round(on, 1),
                         "sag_mm": round(sag, 2), "mouth_lift_mm": round(lift, 2),
                         "ok": x0 < x < end and lift <= MOUTH_LIFT_MAX})
    return {"rows": rows, "worst_margin_mm": min(r["margin_mm"] for r in rows),
            "worst_mouth_lift_mm": max(r["mouth_lift_mm"] for r in rows),
            "ok": all(r["ok"] for r in rows)}


def rail_section() -> tuple[float, float, float]:
    """A base stop rail's section behind the V block (the -z one's; the +z one's T-slot keel
    stiffens it): its bar, its outer keel and its flange (:func:`construction.cradle._rails`):
    :func:`_section`'s area, second moment and farthest fibre (mm)."""
    rear = cr.SADDLES[1]
    lo, hi, _ = cr.REAR_FLANGE
    yb, ft = cr.FLANGE
    kz = cr.RAIL_Z - cr.RAIL_KEEL

    def zv(y: float) -> float:
        return y - rear.apex

    return _section([(zv(hi), hi), (cr.RAIL_Z, hi), (cr.RAIL_Z, yb), (kz, yb), (kz, lo),
                     (zv(lo) + ft, lo), (zv(yb) + ft, yb), (zv(yb), yb)])


def _section(zy) -> tuple[float, float, float]:
    """A polygon's (z, y) area, second moment about its horizontal centroidal axis and its
    farthest fibre from that axis (mm): a rail's section under a vertical load."""
    pts = [*zy, zy[0]]
    a = cy = iy = 0.0
    for (z0, y0), (z1, y1) in itertools.pairwise(pts):
        cross = z0 * y1 - z1 * y0
        a += cross
        cy += (y0 + y1) * cross
        iy += (y0 * y0 + y0 * y1 + y1 * y1) * cross
    a /= 2.0
    cy /= 6.0 * a
    i = iy / 12.0 - a * cy * cy
    far = max(abs(y - cy) for _, y in zy)
    return abs(a), abs(i), far


def drawers() -> list[dict]:
    """The V blocks' guides as drawers (review round 2, N1 and m5): a block pushed up off its
    guide's middle cocks on it, and the guide's friction grows with the push, so that past
    ``mu_jam = engagement / (2 x offset)`` no push lifts it. At each bottle's setting: the
    rear block's boss on its post, pushed at the lead screw beside it
    (:data:`construction.cradle.REAR_POST`; it had been 213 mm across the block), and the
    front block in its box, pushed at the thumbscrew 40 mm off the box's middle (it cocks
    onto the -z wall's top and the +z wall's foot, its cheek up the taller wall,
    :data:`construction.cradle.FRONT_GUIDE`, and over the lip gauge's post its taller cheek
    up the post's face, :data:`construction.cradle.FRONT_CHEEK`). Each against the most
    friction cited for its faces (:data:`PETG_STEEL`), dry: the rear boss on its plated
    post; the front block, PETG in PETG (unmeasured; design review round 3, mounting m9:
    greased as the guide says, until a spill washes it out), its oiled figure beside it
    (``mu_oiled``)."""
    front = cr.SADDLES[0]
    e_rear = math.hypot(cr.REAR_LEAD[0] - cr.REAR_POST[0], cr.REAR_LEAD[1] - cr.REAR_POST[1])
    flat = cr.POST_FLATS[1]                   # (the boss rides the round between the flats)
    post0, post1 = cr.POST[1] + flat, cr.POST[1] + cr.POST[0] - flat
    b0, b1 = cr.REAR_BOSS[1], cr.REAR_BOSS[2]
    e_front = cr.FRONT_JACK[1]
    wall, cheek, _ = cr.FRONT_GUIDE
    tall, post_top = cr.FRONT_CHEEK, cr.GAUGE_POST[2]
    dry, oiled = PETG_STEEL["dry"][1], PETG_STEEL["oiled"][1]
    out = []
    for size in cr.SIZES:
        at = cr.setting(size)
        bottom = front.bottom + at.front
        for guide, eng, e, mu in (
                ("rear", min(b1 + at.rear, post1) - max(b0 + at.rear, post0), e_rear, dry),
                ("front", max(min(cheek + at.front, wall), min(tall + at.front, post_top))
                 - bottom, e_front, dry)):
            jam = eng / (2.0 * e)
            out.append({"bottle": size.name, "guide": guide, "engagement_mm": round(eng, 1),
                        "offset_mm": round(e, 1), "mu_jam": round(jam, 3), "mu": mu,
                        "mu_oiled": oiled, "sf": round(jam / mu, 2),
                        "ok": jam / mu >= DRAWER_BAR})
    return out


def saddles(rest_deg: float = -50.0, full_deg: float = 30.0, moving_kg: float = 0.0
            ) -> dict:
    """The saddles' loads and stresses, a sealed full bottle of each size at its own setting
    over the swing (``rest_deg`` to ``full_deg``), its weight the worst way for each part
    (its centre of mass ``BottleSize.com_from_mouth``), and :func:`balance`:

    * the rear V block on its tie rods (a 6 L methuselah's V blocks sit on them);
    * a V block hung on its thumbscrew (the heaviest a 6 L set a hair off the rods): the
      screw's barrel nut in the block, the roll couple of a load at the axis held at the
      block's +z end (the post, stiffer, and the screw beside it share it), and at rest the
      pitch of the bottle's pull along the axis, 121 mm over the post's root, on the post
      too;
    * the rear lead screw: its lock (the knob's teeth) against the thread turning back under
      the block's load with the thread greased, the brake washer without it, dry and
      greased; its span and its tower under a knock at the knob;
    * the guides (:func:`drawers`): the friction at which raising each V block would jam;
    * the base stop at rest (the bottle's pull down the axis and a corkscrew's push): its
      gate's plate in bending between its ears, its lock's grip (the knob's least preload on
      two faces, at the least friction cited) and the T-slot's lips under its most; its
      rails as cantilevers from their sockets, the body's share on their flanges at worst
      (the bottle on the front V and the flanges' end), their tongues where they leave the
      sockets, the tongues' ribs and the sockets' floors;
    * the lip gauge at full tilt (the weight down the axis, no friction counted): its pad,
      its arm across the mouth, its bar beside the neck, its bridge over the post (the push
      is off their lines by the bolt's 41 mm across and the rim's height), the post (bent by
      the push, twisted by its turn), the bolt (the turn's couple on its cheeks); and,
      fixed, a knock down on its pad, its turn on the post's top's edge.

    Each row: the load (N, or N·m or N·mm where the case says), the stress (MPa) and its
    limit (a printed part's cited strength, :data:`PRINTED`, in its layers or across them as
    it's printed; the steel's yield), their ratio ``sf`` against the row's ``bar``
    (:data:`SUSTAINED_SF` for a printed part's load that stays, :data:`TRANSIENT_SF` for the
    pour's or a knock's, 3 for steel); a grip's row its grip over its load at the least
    friction cited (a row with no ``bar`` is told, not rated). Every printed part rated here
    is PETG, solid (:data:`construction.cradle.PETG_PARTS`; review round 2, N6 and m9).
    ``moving_kg``: what rides the rear V block (itself, its liner, the base stop), hung with
    the bottle."""
    g = G
    heavy = max(cr.SIZES, key=lambda z: z.mass_full_kg)
    w_max = heavy.mass_full_kg * g
    front, rear = cr.SADDLES
    rows: list[dict] = []

    def row(part: str, case: str, load: float, stress: float, limit: float, bar: float,
            basis: str) -> None:
        sf = round(limit / max(stress, 1e-9), 2)
        rows.append({"part": part, "case": case, "load_n": round(load, 1),
                     "stress_mpa": round(stress, 2), "limit_mpa": limit, "basis": basis,
                     "bar": bar, "sf": sf, "ok": sf >= bar})

    def printed(part: str, case: str, load: float, stress: float, across: bool,
                lasting: str) -> None:
        row(part, case, load, stress, PRINTED["petg", "across" if across else "in"],
            SUSTAINED_SF if lasting == "sustained" else TRANSIENT_SF,
            f"PETG {'across' if across else 'in'} its layers, {lasting}")

    def steel(part: str, case: str, load: float, stress: float, limit: float) -> None:
        row(part, case, load, stress, limit, 3.0, "steel's yield")

    def grip(part: str, case: str, load: float, held: float, mu: float,
             bar: float | None) -> None:
        sf = round(held / max(load, 1e-9), 2)
        rows.append({"part": part, "case": case, "load_n": round(load, 3),
                     "grip_n": round(held, 3), "mu": mu, "bar": bar, "sf": sf,
                     "ok": bar is None or sf >= bar})

    s_rest = math.sin(math.radians(-rest_deg))
    c_rest = math.cos(math.radians(rest_deg))
    # the rear V block on its two rods (the 6 L, lying level): each seat a 9 mm wide arc
    # over 24 mm of rod
    seat_area = 2.0 * math.sqrt(cr.SEAT_R ** 2 - (cr.SEAT_R - cr.SEAT_DEPTH) ** 2) * 24.0
    printed("rear V block's seats on the tie rods", "a full 6 L methuselah, level", w_max,
            w_max / 2 / seat_area, False, "sustained")
    # hung on its lead screw: the barrel nut bears up on the bore's end over it
    hung = (heavy.mass_full_kg + moving_kg) * g
    nd = cr.LEAD_NUT[0]
    ring = math.pi / 4 * ((nd + 0.2) ** 2 - (cr.LEAD_SCREW[0] + 1.0) ** 2)
    printed("rear barrel nut on the V block", "a full 6 L, hung, level", hung, hung / ring,
            False, "sustained")
    xs, zs = cr.REAR_LEAD
    post_i = math.pi * cr.POST_D ** 4 / 64
    post_s = math.pi * cr.POST_D ** 3 / 32
    root = cr.LEAD_SCREW[2]
    screw_i = math.pi * root ** 4 / 64
    screw_s = math.pi * root ** 3 / 32
    share = post_i / (post_i + screw_i)
    couple = hung * zs                                        # N·mm, about the axis
    steel("rear post (bending)", "the roll of a full 6 L hung at the screw (N·m)",
          couple * share / 1000, couple * share / post_s, NETRF10_YIELD)
    steel("rear lead screw (bending)", "its share of that roll (N·m)",
          couple * (1 - share) / 1000, couple * (1 - share) / screw_s, STAINLESS_YIELD)
    # at rest, a bottle hung on the screw (any but the methuselah, whose block sits on its
    # rods): its pull along the axis, at the axis, pitches the block about the post's root
    worst = (0.0, None)
    for size in cr.SIZES:
        if cr.setting(size).rear <= 1e-9:
            continue
        m = (size.mass_full_kg + moving_kg) * g
        com = size.com_from_mouth
        pitch = m * c_rest * (com - xs) + (m * s_rest + CORKSCREW_N) * -cr.REAR_FOOT_TOP
        roll = m * c_rest * zs
        both = math.hypot(pitch, roll)
        if both > worst[0]:
            worst = (both, size)
    both, size = worst
    if size is not None:
        steel("rear post (bending at rest)", f"a full {size.name} hung: "
              "the pull along the axis and the roll (N·m)", both * share / 1000,
              both * share / post_s, NETRF10_YIELD)
    # the post's socket in the carrier: its couple as a bearing pressure, linear down the
    # socket's depth (the carrier prints standing: in its layers)
    m_post = max(couple, both) * share
    depth_s = cr.POST_COLLAR[1] - cr.POST[1]
    printed("rear post's socket (bearing)", f"the post's couple at worst, {depth_s:g} mm "
            "deep (N·m)", m_post / 1000, 6 * m_post / (cr.POST_D * depth_s ** 2), False,
            "sustained")
    # the V block at the post's boss: the post's share of the roll into the +z flat and the
    # gusset under it, along both bosses
    depth = cr.REAR_FLANGE[1] - cr.GUSSET[1]
    along = xs - cr.REAR_POST[0] + cr.GUSSET[0]
    printed("rear V block at the post's boss", "the post's share of the roll (N·m)",
            couple * share / 1000, couple * share / (along * depth ** 2 / 6), False,
            "sustained")
    # the lead screw's hold: its lead (8 mm) turns it back under the block's load; the
    # knob's teeth hold it (a third of them bearing), the brake washer only slows it
    od, d = cr.COLLAR[0] / 2, cr.LEAD_SCREW[0] / 2 + 0.3
    r_brake = 2.0 / 3.0 * (od ** 3 - d ** 3) / (od ** 2 - d ** 2)
    pitch_d = cr.LEAD_SCREW[0] - 1.0                          # its mean diameter, Tr8 x 2
    lead_a = math.atan(cr.LEAD / (math.pi * pitch_d))
    phi = math.atan(LEAD_FRICTION / math.cos(math.radians(15.0)))
    back = cr.LEAD / (2 * math.pi) * math.tan(lead_a - phi) / math.tan(lead_a)  # mm per N
    n_t, r_t, r_i, h_t = cr.LOCK_TEETH
    torque = hung * back                                       # N·mm
    chord = 2 * r_i * math.sin(math.pi / n_t)                 # a tooth's root, notch to notch
    tau = torque / ((r_t + r_i) / 2) / (n_t / 3 * chord * h_t)
    printed("rear lead screw's lock (the knob's teeth)", "a full 6 L hung, the thread "
            "greased (N·mm)", torque, math.sqrt(3) * tau, False, "sustained")
    # the teeth hold the knob, the knob the hub by its hex, the hub the screw by an M3 set
    # screw, and the barrel nut is kept from turning in its boss by another (design review
    # round 3, mounting m3; both threadlocked, so neither backs out): the hub's bears on a
    # flat filed on the screw (cradle.LEAD_FLAT), so the turn pushes it sideways into its
    # pilot (in bearing); the nut's grips its round by friction at its point and the bore
    # opposite (the least cited for printed PETG on steel at both), a seating force its
    # self-cut PETG thread carries across its layers (its shear on half the thread's
    # cylinder)
    flat_d, _ = cr.LEAD_FLAT
    hub_bite = cr.REAR_HUB[0] / 2 - cr.LEAD_SCREW[0] / 2 - 0.05
    side = torque / (cr.LEAD_SCREW[0] / 2 - flat_d)
    printed("rear knob's hub on the lead screw (its M3 set screw on the flat)",
            "the greased screw's turn back under a full 6 L, hung: the set screw pushed "
            "sideways into its pilot (N)", side, side / (3.0 * hub_bite), False, "sustained")
    mu_set = PETG_STEEL["dry"][0]
    nut_bite = cr.LEAD_BOSS[0] - cr.LEAD_NUT[0] / 2
    seat = torque / (2.0 * mu_set * cr.LEAD_NUT[0] / 2)
    printed("rear barrel nut in its boss (its M3 set screw)", "the seating force that holds "
            "the same turn by friction (N)", seat,
            math.sqrt(3) * seat / (math.pi * 3.0 * nut_bite * 0.5), True, "sustained")
    grip("rear lead screw's brake, the knob lifted", "any load: the TPU washer's grip per "
         "its pull back (N·mm/N), dry", back, TPU_FRICTION * r_brake, TPU_FRICTION, 3.0)
    grip("rear lead screw's brake, the knob lifted, greased", "the same, the washer's "
         "faces greased (the lock holds; told, not rated)", back, GREASED * r_brake,
         GREASED, None)
    # a knock at the rear knob: into the tower's arm (the screw's top in its bushing), the
    # tower bending at its root; the screw between the nut and the bushing, knocked midway
    tx0, tx1, tz0, tz1 = cr.TOWER
    _, k0, k1, _ = cr.REAR_KNOB
    lever = (k0 + k1) / 2 - cr.REAR_FOOT_TOP
    t_s = min((tx1 - tx0) * (tz1 - tz0) ** 2, (tz1 - tz0) * (tx1 - tx0) ** 2) / 6
    printed("rear knob's tower (bending)", "a 50 N knock sideways at the knob", KNOCK_N,
            KNOCK_N * lever / t_s, True, "transient")
    span = cr.ARM[1] - cr.LEAD_BOSS[2]
    steel("rear lead screw (bending)", "a 50 N knock midway between the nut and the bushing",
          KNOCK_N, KNOCK_N * span / 4 / screw_s, STAINLESS_YIELD)
    share_f = max(max((cr.SADDLES[1].vee[0] + cr.SADDLES[1].vee[1]) / 2
                      - z.com_from_mouth, 0.0)
                  / ((cr.SADDLES[1].vee[0] + cr.SADDLES[1].vee[1]) / 2 - sum(front.vee) / 2)
                  * z.mass_full_kg * g for z in cr.SIZES)
    printed("front thumbscrew's nut on the V block", "the heaviest front share, level",
            share_f, share_f / (3 * math.sqrt(3) / 2 * (cr.M6_HEX[0] / math.sqrt(3)) ** 2
                                - math.pi * 3.3 ** 2), False, "sustained")
    rows.extend({"part": f"{r['guide']} V block's guide (raised: the drawer)",
                 "case": f"set for a {r['bottle']}: {r['engagement_mm']:g} mm engaged, "
                         f"pushed {r['offset_mm']:g} mm off it",
                 "mu_jam": r["mu_jam"], "mu": r["mu"], "bar": DRAWER_BAR, "sf": r["sf"],
                 "ok": r["ok"]} for r in drawers())
    # the base stop at rest: down the axis, the gate's plate a beam between its ears
    push = w_max * s_rest + CORKSCREW_N
    span = 2 * (cr.GROOVE[0] + cr.GROOVE[1]) / 2
    height = cr.REAR_FLANGE[1] + cr.GATE_TOP - cr.GATE_BOTTOM
    printed("base stop's gate (bending)", "a full 6 L at rest, a corkscrew's push", push,
            push * span / 4 / (height * cr.GATE_T ** 2 / 6), False, "sustained")
    # the push lies under the rails' tops (the base's ring, or the gate's bottom edge): its
    # lever pitches the gate about its plate's footprint on the rails, held down by the
    # lock's bolt through the +z ear (the -z ear's key only slides), whose root carries it
    # (design review round 3, mounting M3: unrated, and across the layers of the gate as it
    # printed on its face; it prints on its top, the roots in its layers), and the plate
    # hangs from its top bar, across them
    hi = cr.REAR_FLANGE[1]
    lever_m, lever_z, lever_d = 0.0, cr.DESIGN, 0.0
    for z in cr.SIZES:
        at_z = cr.setting(z)
        under = hi + at_z.rear + min(z.r - BASE_RING, -(cr.GATE_BOTTOM + at_z.rear))
        f = z.mass_full_kg * g * s_rest + CORKSCREW_N
        if f * under > lever_m:
            lever_m, lever_z, lever_d = f * under, z, under
    held_down = lever_m / (cr.LOCK_X - cr.GATE_T / 2)     # the bolt's pull, the footprint's
    root = held_down * (cr.LOCK_X - cr.GATE_T)            # middle the pivot
    ear_w = cr.STOP_MARKS[0] - cr.EAR_Z
    printed("base stop's gate's ears (their root)", f"a full {lever_z.name} at rest, a "
            f"corkscrew's push {lever_d:.0f} mm under the rails: the lock's bolt holds the "
            "+z ear down (N)", held_down, root / (ear_w * cr.GATE_TOP ** 2 / 6), False,
            "sustained")
    face = rear.apex + cr.LINER * math.sqrt(2)
    plate_w = 2.0 * (hi - face - 1.0)                     # its width at the rails' tops
    printed("base stop's gate (its plate hanging from its top bar)", "the same push's lever "
            "(N·m)", lever_m / 1000, lever_m / (plate_w * cr.GATE_T ** 2 / 6), True,
            "sustained")
    least, most = (t * 1000 / (NUT_FACTOR * 5.0) for t in LOCK_TORQUE_NM)
    mu = PETG_STEEL["dry"][0]
    grip("base stop's lock (its grip)", f"the same: its knob tight by hand "
         f"({LOCK_TORQUE_NM[0]:g} N·m), the ear on the rail and the washer on the T-slot's "
         "lips", push, 2 * mu * least, mu, GRIP_BAR)
    _, neck, lip, (ww, _), _, _, _ = cr.TSLOT
    wd = cr.LOCK_WASHER[0]
    reach = ww / 2 - (neck / 2 + wd / 2) / 2                   # the load's lever to the root
    width = wd + 2 * lip                                       # along the slot (45° spread)
    printed("base stop's T-slot lips (bending)", f"the lock's preload, its knob at "
            f"{LOCK_TORQUE_NM[1]:g} N·m", most, most / 2 * reach / (width * lip ** 2 / 6),
            False, "sustained")
    # the rails: the body's share on the flanges at worst, each rail a cantilever from its
    # socket; its tongue where it leaves the socket, the tongue's ribs at its front end, the
    # socket's floor under the tongue's back end
    _, rail_i, far = rail_section()
    xr, xf = rear.vee[1], sum(front.vee) / 2
    worst_m = {"sustained": (0.0, ""), "transient": (0.0, "")}  # at rest; level, the pour
    for z in cr.SIZES:
        end = min(cr.FLANGE_END - cr.CLEAR, z.length - HEEL)
        if end <= xr:
            continue
        com = z.com_from_mouth
        for name, lasting, deg, depth in (("at rest", "sustained", rest_deg, max(min(
                z.r - BASE_RING, -(cr.GATE_BOTTOM + cr.setting(z).rear)),
                z.r / math.sqrt(2))), ("level", "transient", 0.0, 0.0)):
            wn = z.mass_full_kg * g * math.cos(math.radians(deg))
            x = com + depth * math.tan(math.radians(abs(deg)))
            on = wn * (x - xf) / (end - xf)                    # on the flanges' end at worst
            m = on * (end - xr) / 2                            # per rail
            if m > worst_m[lasting][0]:
                worst_m[lasting] = (m, f"a full {z.name} {name}: its body on the flanges' "
                                       "end (N·m)")
    tx0_, tz0_, tz1_, th = cr.TONGUE
    n_r, rw, _ = cr.RIBS
    t_floor = cr.SOCKET[3] - cr.RAIL_HEAD_SINK / 2
    span_f = tz1_ - tz0_ + 2 * cr.CLEAR
    checks = (  # each: its name, its load and stress from the rail's moment, its extra case
        ("base stop's rails (bending at their sockets)",
         lambda m: (m / 1000, m * far / rail_i), ""),
        ("base stop's rails' tongues (bending where they leave the sockets)",
         lambda m: (m / 1000, 6 * m / ((tz1_ - tz0_) * th ** 2)), ""),
        ("base stop's tongues' ribs (bearing)",       # the tongue's front end, up (N)
         lambda m: (m / (xr - tx0_), m / (xr - tx0_) / (n_r * rw * 10.0)),
         "; the tongue's front end on its socket's roof, on the ribs' front 10 mm (N)"),
        ("base stop's sockets (their floors under the tongues)",     # its ends' bearing
         lambda m: (m / ((xr - tx0_) * 2 / 3), 3 * m / ((xr - tx0_) * 2 / 3) * span_f
                    / (2 * (xr - tx0_) / 2 * t_floor ** 2)),
         "; the tongue's back end on the floor (N)"))
    for part, of, more in checks:          # the case that governs: at rest, or the pour
        rated = []
        for lasting, (m, case) in worst_m.items():
            load, stress = of(m)
            bar = SUSTAINED_SF if lasting == "sustained" else TRANSIENT_SF
            rated.append((PRINTED["petg", "in"] / max(stress, 1e-9) / bar, lasting, load,
                          stress, case))
        _, lasting, load, stress, case = min(rated)
        printed(part, case.removesuffix(" (N·m)") + (more or " (N·m)"), load, stress, False,
                lasting)
    # the lip gauge at full tilt: the weight down the axis on its pad (on the rim's top); it
    # prints on its side (its layers across z)
    static = w_max * math.sin(math.radians(full_deg))
    bottom, half = cr.GAUGE_PAD
    ax0, ax1 = cr.GAUGE_FRONT
    y0, y1, bz0, bz1 = cr.GAUGE_BAR
    by0, by1, rz0, rz1 = cr.GAUGE_BRIDGE
    yp, zp = cr.GAUGE_PIVOT
    rim = (bottom + max(z.r_neck for z in cr.SIZES)) / 2
    slide = gauge_slide(full_deg)
    pull = max(static, slide["peak_n"])
    gcase = ("a full 6 L at full tilt, no friction" if pull == static else
             f"a full 6 L's slide onto it, wet, from {SLIDE_GAP:g} mm behind it")
    t = ax1 - ax0                                              # their depth along x
    printed("lip gauge's pad (bending)", gcase, pull,
            pull * (y0 - bottom) / ((2 * half) * t ** 2 / 6), False, "transient")
    printed("lip gauge's arm (bending)", "the same", pull,
            pull * -bz1 / ((y1 - y0) * t ** 2 / 6), True, "transient")
    bw, bh = bz1 - bz0, y1 - y0
    printed("lip gauge's bar (bending both ways)", "the same", pull,
            pull * -(bz0 + bz1) / 2 / (bh * bw ** 2 / 6)
            + pull * ((y0 + y1) / 2 - rim) / (bw * bh ** 2 / 6), False, "transient")
    rw_, rh = rz1 - rz0, by1 - by0
    printed("lip gauge's bridge (bending both ways)", "the same", pull,
            pull * -zp / (rh * rw_ ** 2 / 6) + pull * ((by0 + by1) / 2 - rim)
            / (rw_ * rh ** 2 / 6), False, "transient")
    px0, px1, _ = cr.GAUGE_POST
    pl, pw = px1 - px0, -cr.FRONT_HALF - cr.CLEAR - cr.GAUGE_POST_OUT   # along x, across
    turn = pull * -zp                                          # N·mm, about the post's axis
    sigma = pull * (yp - cr.FRONT_GUIDE[0]) / (pw * pl ** 2 / 6)
    a, b = max(pl, pw), min(pl, pw)
    tau = turn / ((1 / 3 - 0.21 * b / a * (1 - (b / a) ** 4 / 12)) * a * b ** 2)
    printed("lip gauge's post (bent and twisted)", "the same (its cheek on the post's back)",
            pull, math.sqrt(sigma ** 2 + 3 * tau ** 2), True, "transient")
    (f0, f1), (b0, b1) = cr.GAUGE_KNUCKLES
    spacing = (b0 + b1) / 2 - (f0 + f1) / 2
    on_pin = turn / spacing                                    # each cheek on the bolt
    m_pin = on_pin * ((f1 - f0) / 2 + (px0 - f1))
    steel("lip gauge's bolt (M5, bending)", "its cheeks' couple, the push's turn", on_pin,
          m_pin / (math.pi * 4.134 ** 3 / 32), STAINLESS_YIELD)
    printed("lip gauge's post's bore (bearing)", "the bolt's couple in it", on_pin,
            6 * turn / ((cr.TIE_HOLE - 0.2) * pl ** 2), False, "transient")
    held = cr.gauge_fixed()
    knock = KNOCK_N * -zp                                      # its turn about the bolt
    edge = knock / min(held["edge_levers_mm"])
    printed("lip gauge on its post (fixed)", "a 50 N knock down on its pad: its turn on the "
            "post top's edge (2 mm of it)", edge, edge / (pl * 2.0), False, "transient")
    rated = [r for r in rows if r.get("bar")]
    worst = min(rated, key=lambda r: r["sf"] / r["bar"])
    return {"rows": rows, "ok": all(r["ok"] for r in rows),
            "worst": {"part": worst["part"], "sf": worst["sf"], "bar": worst["bar"]},
            "balance": balance(rest_deg, full_deg), "gauge": held, "slide": slide}


def gauge_slide(full_deg: float = 30.0) -> dict:
    """A wet bottle's slide onto the lip gauge's pad past level (design review round 3,
    mounting m1), at its worst (a full 6 L methuselah): at rest it lies on the gate, its rim
    up to :data:`SLIDE_GAP` behind the pad. Dry, its TPU liners hold it past level (over 1,
    times √2: a 90° V's two faces press it with √2 its weight); wet, it slides once the tilt's
    tangent passes √2 x its static friction, within the swing only under tan(full) / √2, and
    the friction falls to its kinetic, at least :data:`TPU_WET`: their difference drives it
    the gap into the pad, a spring (the gauge's bar bent by the push off its line, both
    ways: the stiffest it can be, the post's twist and its bolt only soften it), which peaks
    at the drive plus the root of its square and twice the spring's work over the gap.
    ``peak_n`` (N) against ``static_n``, the weight down the axis at full tilt with no
    friction counted."""
    heavy = max(cr.SIZES, key=lambda z: z.mass_full_kg)
    w = heavy.mass_full_kg * G
    wedge = math.sqrt(2.0)
    full = math.radians(full_deg)
    mu_s = math.tan(full) / wedge
    drive = w * math.cos(full) * wedge * max(mu_s - TPU_WET, 0.0)
    y0, y1, bz0, bz1 = cr.GAUGE_BAR
    length = cr.GAUGE_KNUCKLES[0][0] - cr.GAUGE_FRONT[0]
    bw, bh = bz1 - bz0, y1 - y0
    rim = (cr.GAUGE_PAD[0] + max(z.r_neck for z in cr.SIZES)) / 2
    off_z, off_y = -(bz0 + bz1) / 2, (y0 + y1) / 2 - rim
    k = E_PETG / length / (off_z ** 2 * 12 / (bh * bw ** 3) + off_y ** 2 * 12 / (bw * bh ** 3))
    peak = drive + math.sqrt(drive ** 2 + 2.0 * k * drive * SLIDE_GAP)
    return {"mu_static_most": round(mu_s, 3), "mu_kinetic": TPU_WET,
            "drive_n": round(drive, 1), "gauge_n_per_mm": round(k, 1), "gap_mm": SLIDE_GAP,
            "peak_n": round(peak, 1), "static_n": round(w * math.sin(full), 1)}


def wheel_sweep(mech) -> list[dict]:
    """What the hand crank's turning would hit: the solids of revolution its parts sweep
    about the input shaft (``notes["hand_crank"]["handle"]["sweep"]``: its hub and tube, the
    hub's thumb screw's head, the clutch's flange and boss, the disc its arm, grip and the
    grip's bolt and nut sweep), against every part but the crank's own and the gearbox it
    rides (the crank is drawn at rest)."""
    from build123d import Axis, Cylinder, Location

    w = (mech.meta.get("hand_crank") or {}).get("handle")
    if not w:
        return []
    end, d = np.asarray(w["end_xy"], float), np.asarray(w["dir"], float)
    z = float(w["axis_z"]) - float(mech.meta.get("mid_plane", 0.0))
    a = math.degrees(math.atan2(d[1], d[0]))

    def cyl(r: float, t0: float, t1: float):
        # along +x from the shaft's end, then turned to the shaft's way (as it is built)
        return Cylinder(r, t1 - t0, rotation=(0.0, 90.0, 0.0)).moved(
            Location(((t0 + t1) / 2, 0.0, 0.0))).rotate(Axis.Z, a).moved(
            Location((float(end[0]), float(end[1]), z)))

    from spiderpig.shapes import union

    sweep = union([cyl(float(r), float(t0), float(t1)) for r, t0, t1 in w["sweep"]])
    out = []
    box = sweep.bounding_box()
    for b in mech.bodies:
        if b.part is None or "handle" in b.name or b.name.endswith("gearbox"):
            continue
        part = b.placed_part()
        bb = part.bounding_box()
        if not all(max(getattr(box.min, c), getattr(bb.min, c)) < min(getattr(box.max, c),
                                                                       getattr(bb.max, c))
                   for c in "XYZ"):
            continue
        inter = sweep & part
        v = 0.0 if inter is None else sum(x.volume for x in inter.solids())
        if v > 1.0:
            out.append({"a": "hand wheel's sweep", "b": b.name, "mm3": round(v, 1)})
    return out


HAND_AROUND = 20.0      # a hand on the grip: its fingers and knuckles reach this far past the
#                         grip's surface (a 16 mm grip's 28: round 2's reviewer took 25)
HAND_OUT = 15.0         # and this far past the grip bolt's head
SWEEP_EDGE = 1.5        # hand_sweep's surface triangles near an envelope, split to this edge
SWEEP_NEAR = 15.0       # (mm): within this of an envelope


def hand_sweep(config: BuildConfig, mech, step_deg: float = 1.0, t_ref: float = 1.0) -> dict:
    """The hand crank's turning against every other part of the built cradle ``mech``
    (fabricated at ``t_ref``) over the whole crank turn at ``step_deg``: each body's motion
    the planar rigid fit of its anchor's joints (as the bake's), its surface sampled
    adaptively (a triangle whose bound comes within :data:`SWEEP_NEAR` of an envelope split
    until its edges are under :data:`SWEEP_EDGE`), measured about the input shaft's axis
    against (a) the crank's own swept solids (``notes["hand_crank"]["handle"]["sweep"]``,
    drawn at rest: what it sweeps over a handle turn) and (b) a hand on the grip: within the
    grip's radius and :data:`HAND_AROUND` round it, from the arm's outer face to the grip
    bolt's head and :data:`HAND_OUT` (design review round 2's M-3: the crank's own solids
    cleared A's cradle where a hand on its grip didn't). Each handle turn covers every angle
    of the crank's disc, so every crank angle meets the whole envelope. Signed clearances
    (mm, negative inside): the least of each, at which crank angle and what part, and the
    crank degrees where the hand's is under 0."""
    from spiderpig import walk
    from spiderpig.construction.robot import robot_template
    from spiderpig.mesh import tessellate_many

    w = (mech.meta.get("hand_crank") or {}).get("handle")
    if not w:
        return {}
    end, d = np.asarray(w["end_xy"], float), np.asarray(w["dir"], float)
    nrm = np.array([-d[1], d[0]])
    zax = float(w["axis_z"]) - float(mech.meta.get("mid_plane", 0.0))
    sweep = [tuple(map(float, s)) for s in w["sweep"]]
    throw = float(w["grip_r_mm"])
    grip_d = float((w.get("grip_mm") or [16.0])[0])
    disc_row = max(sweep, key=lambda s: s[0])
    hand_x = (float(w["reach_mm"]), disc_row[2] + HAND_OUT)
    hand_r = (throw - grip_d / 2 - HAND_AROUND, throw + grip_d / 2 + HAND_AROUND)
    tmpl = template_for(config)
    by = {b.name: b for b in mech.bodies}
    owner = {b.name: walk.anchor_of(b, by) for b in mech.bodies}
    anchors = {}
    for name in {o for o in owner.values() if o}:
        b = by[name]
        anchors[name] = {j.name: np.asarray((b.pose @ j.pose).matrix[:3, 3], float)
                         for j in b.joints}
    deg = np.arange(0.0, 360.0, step_deg)
    ts = t_ref + np.radians(deg)
    nf = len(ts)
    sampled = robot_template(tmpl).sample(ts)
    motion = {}
    for name, joints in anchors.items():
        cur = sampled.joint_world.get(name)
        if cur is None:
            continue
        names = [j for j in joints if j in cur]
        p0 = np.broadcast_to(np.stack([joints[j] for j in names]), (nf, len(names), 3))
        p1 = np.stack([cur[j] for j in names], axis=1)
        motion[name] = walk.planar_fit(p0, p1)
    bodies = [b for b in mech.bodies if b.part is not None and "handle" not in b.name
              and not b.name.endswith("gearbox")]
    meshes = tessellate_many([b.placed_part() for b in bodies], 0.2, 0.2)

    def clear(x, rho):
        best = np.full(x.shape, np.inf)
        for r, x0, x1 in sweep:
            dx = np.maximum(np.maximum(x0 - x, x - x1), 0.0)
            dr = rho - r
            best = np.minimum(best, np.where(dx == 0.0, dr, np.where(
                dr <= 0, dx, np.hypot(np.maximum(dr, 0), dx))))
        dx = np.maximum(np.maximum(hand_x[0] - x, x - hand_x[1]), 0.0)
        dr = np.maximum(np.maximum(hand_r[0] - rho, rho - hand_r[1]), 0.0)
        inside = (dx == 0) & (dr == 0)
        pen = -np.minimum.reduce([x - hand_x[0], hand_x[1] - x, rho - hand_r[0],
                                  hand_r[1] - rho])
        return best, np.where(inside, pen, np.hypot(dx, dr))

    def measure(pts, th, tr):
        cs, hs = np.empty((nf, len(pts))), np.empty((nf, len(pts)))
        for f0 in range(0, nf, 24):
            fr = slice(f0, min(nf, f0 + 24))
            c, s = np.cos(th[fr]), np.sin(th[fr])
            xw = c[:, None] * pts[None, :, 0] - s[:, None] * pts[None, :, 1] + tr[fr, 0, None]
            yw = s[:, None] * pts[None, :, 0] + c[:, None] * pts[None, :, 1] + tr[fr, 1, None]
            zw = pts[None, :, 2] + tr[fr, 2, None]
            rx, ry = xw - end[0], yw - end[1]
            cs[fr], hs[fr] = clear(rx * d[0] + ry * d[1],
                                   np.hypot(rx * nrm[0] + ry * nrm[1], zw - zax))
        return cs, hs

    def split(tri):
        a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
        ab, bc, ca = (a + b) / 2, (b + c) / 2, (c + a) / 2
        return np.concatenate([np.stack([a, ab, ca], 1), np.stack([ab, b, bc], 1),
                               np.stack([ca, bc, c], 1), np.stack([ab, bc, ca], 1)])

    def take(pts, th, tr, bc, bh):
        """``pts`` measured: the least clearances so far (``bc``, ``bh``, per frame) and
        each point's least over every frame and both envelopes."""
        lb = np.full(len(pts), np.inf)
        for p0 in range(0, len(pts), 3000):
            cc, hh = measure(pts[p0:p0 + 3000], th, tr)
            bc = np.minimum(bc, cc.min(axis=1))
            bh = np.minimum(bh, hh.min(axis=1))
            lb[p0:p0 + 3000] = np.minimum(cc, hh).min(axis=0)
        return lb, bc, bh

    worst = np.full(nf, np.inf)
    hand = np.full(nf, np.inf)
    who_c, who_h = [""] * nf, [""] * nf
    for b, (pos, idx, _) in zip(bodies, meshes, strict=True):
        if len(pos) == 0:
            continue
        o = owner[b.name]
        th, tr = (motion[o] if o in motion else (np.zeros(nf), np.zeros((nf, 3))))
        pos = pos.astype(float)
        tri = pos[idx.reshape(-1, 3)]
        _, bc, bh = take(pos, th, tr, np.full(nf, np.inf), np.full(nf, np.inf))
        for _ in range(14):
            if not len(tri):
                break
            cen = tri.mean(axis=1)
            rad = np.linalg.norm(tri - cen[:, None], axis=2).max(axis=1)
            edge = np.maximum.reduce([np.linalg.norm(tri[:, 1] - tri[:, 0], axis=1),
                                      np.linalg.norm(tri[:, 2] - tri[:, 1], axis=1),
                                      np.linalg.norm(tri[:, 0] - tri[:, 2], axis=1)])
            lb, bc, bh = take(cen, th, tr, bc, bh)
            tri = tri[(lb - rad < SWEEP_NEAR) & (edge > SWEEP_EDGE)]
            if len(tri) > 300000:
                break
            tri = split(tri) if len(tri) else tri
        for arr, who, got in ((worst, who_c, bc), (hand, who_h, bh)):
            upd = got < arr
            arr[upd] = got[upd]
            for i in np.nonzero(upd)[0]:
                who[i] = b.name
    i, j = int(np.argmin(worst)), int(np.argmin(hand))
    return {"step_deg": step_deg, "hand_x_mm": [round(v, 1) for v in hand_x],
            "hand_r_mm": [round(v, 1) for v in hand_r],
            "crank": [round(float(worst[i]), 2), float(deg[i]), who_c[i]],
            "hand": [round(float(hand[j]), 2), float(deg[j]), who_h[j]],
            "hand_under_0_deg": round(float((hand < 0).sum() * step_deg), 1)}


def joint_rows(config: BuildConfig, rider: str, joints, forces, jam_f) -> list[dict]:
    """Each joint's peak force over the turn (N), the bottle's and the jam's, per side
    (:data:`SHARE` on the pins both sides have)."""
    lk = config.lk
    rows = []
    for jk, (j, b1, b2) in enumerate(joints):
        on_drive = j in lk.crank or rider in (b1, b2)
        share = 1.0 if on_drive else SHARE
        rows.append({"joint": j, "bodies": [b1, b2], "drive_side_only": on_drive,
                     "bottle_n": round(float(forces[:, jk].max()) * share, 1),
                     "jam_n": round(float(jam_f[:, jk].max()) * share, 1)})
    return rows


def loads(config: BuildConfig, store=None) -> dict:
    """The joint loads :mod:`spiderpig.strength` rates a cradle at (what
    :func:`strength.design_loads` returns for one: the audit's, ``explain``'s and verify's
    pin and plate rows): a sealed full 6 L methuselah's walking, the hand crank's clutch's
    cap jammed (:func:`jam_nm`)."""
    tmpl = template_for(config)
    design = design_side(tmpl, config)
    mech = fabricate(tmpl, config, 1.0, store=store)
    ts = np.linspace(0.0, 2.0 * math.pi, 360, endpoint=False)
    rigid, joints = planar_model(config, mech, 1.0, cr.DESIGN)
    torque, forces = statics(config, rigid, joints, ts)
    _, jam_f = statics(config, rigid, joints, ts, jam_nm=jam_nm(config))
    jam_f = np.where(_toggles(config, ts)[:, None], 0.0, jam_f)
    rows = joint_rows(config, HandCrank.rider(design.ctx), joints, forces, jam_f)
    return loads_doc(config, rows, float(np.abs(torque).max()),
                     structure(config, mech, ts=ts))


def loads_doc(config: BuildConfig, rows: list[dict], crank_nm: float,
              struct: dict | None = None) -> dict:
    """:func:`joint_rows` as :func:`strength.check`'s loads document (source ``statics``:
    per joint, as a sim's); with ``struct`` (:func:`structure`), what it rated from the
    force vectors: the links' bending (``link_frames``), the pillars' moments (``posts``),
    the columns' and the frame plates' rows (``rated``)."""
    lk = config.lk
    ground = set(lk.frame)
    joints = []
    for r in rows:
        bodies = [b for b in r["bodies"] if b != "ground"]
        joints.append({"stem": r["joint"], "crank": r["joint"] in lk.crank,
                       "frame": r["joint"] in ground, "links": bodies,
                       "walk": {"n": r["bottle_n"], "patterns": None},
                       "jam": {"n": r["jam_n"], "patterns": None}})
    out = {"source": "statics", "note": "a cradle's: a sealed full 6 L methuselah (walk), the "
           "most the hand crank's slip clutch lets onto a blocked cradle "
           f"({jam_nm(config):g} N·m at the crank, jam); cradle_report",
           "walk_n": max((j["walk"]["n"] for j in joints), default=0.0),
           "jam_n": max((j["jam"]["n"] for j in joints), default=0.0), "joints": joints,
           "torque_limit_nm": jam_nm(config),
           "walk_torque_nm": round(crank_nm, 2),
           "joint_moment_factor": 1.0,
           "crank_basis": "statics: the bottle's largest torque over a pour, the hand crank's "
                          "clutch's cap jammed"}
    if struct is not None:
        out["link_frames"] = struct["link_frames"]
        out["posts"] = struct["pillars"]
        out["rated"] = struct["rated"]
    return out


# -- the structure from the force vectors (cradle_structure) ---------------------------

def column_section(stand: cr.Stand) -> tuple[float, float, float]:
    """A stand column's section (mm): its diameter, the bore its tapped ends are rated as
    (the female thread's roots, as a pillar's: a NETRF10's M5 5.0, a NETRF12's M6 6.0;
    design review round 3's minor 3: the tap drill's had been taken) and their depth
    (:func:`construction.cradle.column_parts`)."""
    col = cr.column_parts(stand)
    return col.d, (ROD_ROOT if col.d > 10.0 else NETRF10_ROOT), col.thread


BLOCKS = {"front saddle": (55.0, -47.95), "rear saddle": (317.5, -149.0),
          "bottle's base": (560.0, 60.0)}
"""Where a blocked cradle is held in the known-limit cases (:func:`blocked`), in the cradle's
frame (mm): the front tie Tf1 (what the front saddle, the light and the front ties would
bring down on a raised decanter's rim), the rear saddle's underside, midway along it (the
base stop's rails, anything on the board), and a 6 L's base at its top edge (a shelf or a
cabinet's front over it: design review round 2's M2)."""


def _holes(config: BuildConfig, notes: dict) -> dict[str, float]:
    """Each joint's hole in its links (mm) as the constructions cut it: a pin's or pillar's
    the shaft (``section.bearing_d``) and its links' running clearance (``clearance_mm``:
    the pins' 8.2 mm, the pillars' 12.35), the crankpin's bore, the tie rods' screws'."""
    from spiderpig import strength

    out = {}
    for name, note in notes.items():
        sec = note.get("section") or {}
        if "bearing_d" not in sec:
            continue
        gap = max((float(e.get("clearance_mm", 0.0)) for e in note.get("links") or []),
                  default=0.0)
        out[strength._stem(name)] = round(float(sec["bearing_d"]) + gap, 3)
    out[config.lk.crank[1]] = strength.rider_hole(config)
    from spiderpig.linkages.decanters import TIES

    out.update(dict.fromkeys(TIES, cr.TIE_ROD_HOLE))
    return out


def _stand_xy(config: BuildConfig, pts) -> dict[str, np.ndarray]:
    """Every node of the cradle's stand (its plates' members' ends, its columns), world XY."""
    stand = cr.stand_of(config.lk)
    mouth = pts[cr.mouth_name(config.lk)][0]
    names = ({p for pair in (*stand.outer, *stand.inner) for p in pair} | set(stand.columns)
             | {*stand.feet, *stand.base_ties, *stand.bay_ties})
    return {n: (mouth + np.asarray(stand.nodes[n], float)) if n in stand.nodes
            else np.asarray(pts[n][0], float) for n in names}


def _stand_widths(stand: cr.Stand) -> dict[tuple[str, str], float]:
    widths = dict.fromkeys((*stand.outer, *stand.inner), 2 * stand.member_r)
    widths.update({k: 2 * r for k, r in stand.wide.items()})
    return widths


SPAN_TOL = 2.5          # mm: a post's standoff ends within this of each plate's face (its
#                         end shims, StandoffAxle.max_shims, and the play)
_SPANS: dict[int, tuple[object, dict]] = {}     # post_spans by the mechanism (kept alive)


def post_spans(config: BuildConfig, mech) -> dict[str, dict]:
    """Every post between a side's frame plates as fabricated (design review round 3's B1:
    B's pillar O1 had stopped 15.9 mm under the inner plate, no hole cut there, while the
    side frame tied it to it): per side and post (``"L"``, ``"pillar:O1"`` /
    ``"column:D1"``), its standoff's z extent, the plates' faces it stands between, how far
    short of each it ends (mm) and whether each plate is cut through on its axis (a hole
    there, its screw's). Memoized by the mechanism."""
    from build123d import Vector

    hit = _SPANS.get(id(mech))
    if hit is not None and hit[0] is mech:
        return hit[1]
    lk = config.lk
    stand = cr.stand_of(lk)
    stems = {f"pillar:{j}": rf"pillar_{j}_standoff\d+" for j in lk.frame if j != lk.crank[0]}
    stems.update({f"column:{n}": f"stand_column_{n}" for n in stand.columns})
    by = {b.name: b for b in mech.bodies if b.part is not None}
    out: dict[str, dict] = {}
    for side in sorted({n.split(".", 1)[0] for n in by if "." in n}):
        outer, inner = by.get(f"{side}.frame_outer"), by.get(f"{side}.torso")
        if outer is None or inner is None:
            continue
        po, pi = outer.placed_part(), inner.placed_part()
        ob, ib = po.bounding_box(), pi.bounding_box()
        up = ob.max.Z <= ib.min.Z           # the outer plate under the inner (the L side)
        f_out, f_in = (ob.max.Z, ib.min.Z) if up else (ob.min.Z, ib.max.Z)
        mid_o, mid_i = (ob.min.Z + ob.max.Z) / 2, (ib.min.Z + ib.max.Z) / 2
        got: dict[str, dict] = {}
        for name, stem in stems.items():
            parts = [b for n, b in by.items() if re.fullmatch(f"{side}\\.{stem}", n)]
            if not parts:
                got[name] = {"found": False}
                continue
            bbs = [b.placed_part().bounding_box() for b in parts]
            lo, hi = min(b.min.Z for b in bbs), max(b.max.Z for b in bbs)
            x = (min(b.min.X for b in bbs) + max(b.max.X for b in bbs)) / 2
            y = (min(b.min.Y for b in bbs) + max(b.max.Y for b in bbs)) / 2
            near_o, near_i = (lo, hi) if up else (hi, lo)
            got[name] = {"found": True, "z": [round(lo, 3), round(hi, 3)],
                         "faces": [round(f_out, 3), round(f_in, 3)],
                         "short_mm": [round(abs(near_o - f_out), 3),
                                      round(abs(near_i - f_in), 3)],
                         "holes": [not po.is_inside(Vector(x, y, mid_o)),
                                   not pi.is_inside(Vector(x, y, mid_i))]}
        out[side] = got
    if len(_SPANS) > 3:
        _SPANS.clear()
    _SPANS[id(mech)] = (mech, out)
    return out


def check_spans(config: BuildConfig, mech) -> None:
    """Every post the side frame rates stands plate to plate in ``mech``, as it models them
    (:func:`post_spans`): its standoff within :data:`SPAN_TOL` of both plates' faces, a hole
    through each plate on its axis; else ``ValueError`` (a cantilever the side frame doesn't
    model, or a post with nothing to screw it to)."""
    bad = []
    for side, got in post_spans(config, mech).items():
        for name, g in got.items():
            if not g["found"]:
                bad.append(f"{side}.{name}: no standoff fabricated")
                continue
            for plate, short, hole in zip(("outer", "inner"), g["short_mm"], g["holes"],
                                          strict=True):
                if short > SPAN_TOL:
                    bad.append(f"{side}.{name} stops {short:.1f} mm short of the {plate} "
                               "plate")
                if not hole:
                    bad.append(f"{side}.{name}: the {plate} plate has no hole on its axis")
    if bad:
        raise ValueError(f"{config.lk.key}: the side frame rates every post plate to plate, "
                         "but " + "; ".join(bad))


def _side_posts(config: BuildConfig, notes: dict, joints, pts, mech=None
                ) -> tuple[list, float]:
    """The drive side's posts (:class:`cradle_structure.Post`): each pillar with its links'
    loads along their bores over the outer plate's face (half a quarter of the link's
    thickness either side of its mid-plane: the load spread over the bore), each stand
    column, each with its
    plates' hold on its ends (:func:`cradle_structure.plate_spring`); and the plates' gap
    (mm), between layer 0's top and the plan's top layer's bottom. Each post stands plate to
    plate: a pillar's note must say it reaches both (``supports``), and with ``mech`` the
    fabricated parts must (:func:`check_spans`; design review round 3's B1: B's O1, a
    cantilever, had been rated on the plates' full gap)."""
    from spiderpig import cradle_structure as cs
    from spiderpig.materials import sheet_of, thickness

    lk = config.lk
    stand = cr.stand_of(lk)
    mouth = pts[cr.mouth_name(lk)][0]
    xy = _stand_xy(config, pts)
    widths = _stand_widths(stand)
    t_frame = thickness(config, sheet_of(config, "frame"))
    nodes = [j for j in lk.frame if j != lk.crank[0]] + list(stand.columns)
    if mech is not None:
        check_spans(config, mech)

    def spring(pairs, n: str) -> np.ndarray:
        at = {k: xy[k] for k in xy if k in nodes or any(k in pr for pr in pairs)}
        segs = []
        for pr in pairs:
            w = widths.get(pr, widths.get(pr[::-1], 2 * stand.member_r))
            segs += [(s, w) for s in cs._split(list(at), at, [pr], tol=1.0)]
        return cs.plate_spring(xy[n], [(xy[b if a == n else a], w) for (a, b), w in segs
                                       if n in (a, b)], t_frame)

    out = []
    length = None
    for j in lk.frame:
        if j == lk.crank[0]:
            continue
        note = notes.get(f"pillar:{j}")
        if note is None:
            continue
        lz = note["layer_z"]
        face = float(lz["0"][1])
        top = max(map(int, lz))
        sup = [int(k) for k in note.get("supports") or (0, top)]
        if sorted(sup) != [0, top]:
            raise ValueError(f"{lk.key}: pillar {j} is held at layers {sup}, not by both frame "
                             f"plates (0 and {top}): the side frame rates it plate to plate")
        length = float(lz[str(top)][0]) - face
        od = float(note["section"]["bearing_d"])
        ei = ROD_E * math.pi * od ** 4 / 64        # (solid but for its tapped ends)
        loads = []
        for jk, (jj, body, other) in enumerate(joints):
            if jj != j or other != "ground":
                continue
            k = note["layers"].get(body)
            if k is None:
                raise ValueError(f"pillar {j}: no layer for its link {body}")
            z0 = float(lz[str(k)][0])
            t = thickness(config, sheet_of(config, "link", body))
            # its bore bears along its thickness: half the load a quarter of it either side
            # of its mid-plane (a load spread evenly over t, to the moment between them)
            loads += [(jk, -0.5, z0 + t / 4 - face), (jk, -0.5, z0 + 3 * t / 4 - face)]
        out.append(cs.Post(j, np.asarray(pts[j][0], float), ei, loads,
                           spring(stand.outer, j), spring(stand.inner, j), ROD_THREAD,
                           ea=ROD_E * math.pi * od ** 2 / 4))
    if length is None:
        raise ValueError(f"{lk.key}: no pillar note to read the plates' gap from")
    d, _, thread = column_section(stand)
    for n in stand.columns:
        at = (mouth + np.asarray(stand.nodes[n], float)) if n in stand.nodes else pts[n][0]
        out.append(cs.Post(f"column:{n}", np.asarray(at, float),
                           ROD_E * math.pi * d ** 4 / 64, [],
                           spring(stand.outer, n), spring(stand.inner, n), thread,
                           ea=ROD_E * math.pi * d ** 2 / 4))
    return out, length


def post_bending(got: dict, d: float, bore: float) -> float:
    """A post's worst bending stress (MPa) from :func:`cradle_structure.posts`: its tapped
    ends' moment on the tube the tap leaves, the rest on the solid bar."""
    tube = math.pi * (d ** 4 - bore ** 4) / (32 * d)
    solid = math.pi * d ** 3 / 32
    return max(got["ends_nm"] * 1000.0 / tube, got["mid_nm"] * 1000.0 / solid)


def _post_row(name: str, kind: str, got: dict[str, dict], d: float, bore: float,
              yield_mpa: float, what: str) -> dict:
    """A column's strength row from :func:`cradle_structure.posts`'s results per case."""
    row = {"joint": name, "kind": kind, "links": [], "section": what, "what": what}
    area = math.pi * (d * d - bore * bore) / 4
    for tag in ("walk", "jam"):
        g = got.get(tag)
        if g is None:
            row[tag] = None
            continue
        bend = post_bending(g, d, bore)
        shear = g["shear_n"] / area
        vm = math.hypot(bend, math.sqrt(3) * shear)
        row[tag] = {"load_n": g["end_n"], "moment_nm": g["moment_nm"],
                    "bending_mpa": round(bend, 1), "safety": round(yield_mpa / max(vm, 1e-9), 2)}
    return row


def structure(config: BuildConfig, mech, *, crank_nm: float | None = None, ts=None,
              skip=None, bottle: cr.BottleSize | None = cr.DESIGN,
              block: tuple[float, float] | None = None,
              jam: tuple[np.ndarray, np.ndarray] | None = None) -> dict:
    """The cradle's structure from the statics' force vectors (:mod:`cradle_structure`):
    "walk" ``bottle`` sealed and full, "jam" ``crank_nm`` at the crank (default the jam
    case's, :func:`jam_nm`) on a blocked cradle (a couple; with ``block``, a force at that
    point of the cradle, :func:`statics`; with ``jam``, these crank torques and joint force
    vectors per angle instead, :func:`blocked`'s superposition), over ``ts`` (default 360
    crank angles) less ``skip`` in the jam (default the toggles' :func:`_toggles`; pass an
    empty mask, and a clutch's cap as ``crank_nm``, to rate them). ``mech``: the cradle
    fabricated (its masses, its wobble notes: the plan's z). Returns the links in bending,
    the posts and both frame plates as one frame (:func:`cradle_structure.side_frame`,
    design review round 2: each plate out of its plane as it is, the gearbox's shaft
    turning the outer plate at the crank), the frame plates' members in their plane and out
    of it, the side's stiffness across (``stiffness``), the cradle box across the bay
    (``box``), and the strength rows they add (``rated``: the columns, a row per frame plate,
    the tie rods and their end joints) and the pillars' moments (``pillars``) for
    :func:`loads_doc`. Since design review round 3 (2026-10-20): each post's span from the
    parts (:func:`check_spans`), the solve exact, both idealizations of the plates (faces and
    mid-planes, the worse), second order with the side's buckling per angle (``buckling``,
    a rated row), each plate section's stresses combined, every part on its specified
    minimum yield (:func:`materials.allowable_mpa`), and the posts' screwed ends
    (``joints``, :func:`post_joints`)."""
    from spiderpig import cradle_structure as cs
    from spiderpig.materials import allowable_mpa, link_sheets, thickness

    lk = config.lk
    if ts is None:
        ts = np.linspace(0.0, 2.0 * math.pi, 360, endpoint=False)
    ts = np.asarray(ts, float)
    if crank_nm is None:
        crank_nm = jam_nm(config)
    if skip is None:
        skip = _toggles(config, ts)
    skip = np.asarray(skip, bool)
    rigid, joints = planar_model(config, mech, 1.0, bottle)
    pts = _world_joints(config, ts)
    walk_t, _, walk_v = statics_vectors(config, rigid, joints, ts)
    if jam is None:
        _, _, jam_v = statics_vectors(config, rigid, joints, ts, jam_nm=crank_nm, block=block)
        jam_torque = np.full(len(ts), crank_nm)
    else:
        jam_torque, jam_v = jam
    cases = {"walk": (walk_v, np.zeros(len(ts), bool), walk_t),
             "jam": (jam_v, skip, jam_torque)}
    notes = mech.meta.get("wobble") or {}
    holes = _holes(config, notes)
    w = 2 * config.params.link_radius
    sheets = link_sheets(config)
    sections = {}
    for name in lk.links:
        key = sheets.get(name, config.sheet)
        sections[name] = cs.LinkSection(w, thickness(config, key), allowable_mpa(config, key),
                                        key)
    link = cr.cradle_link(lk)
    ties = [j for j in lk.links[link][0] if j.startswith("T")]
    on = {"walk": ties, "jam": ties}
    if block is not None:       # a block comes in through its saddle's two tie rods
        from spiderpig.linkages.decanters import TIES

        on["jam"] = sorted(ties, key=lambda j: math.dist(TIES[j], block))[:2]
    from spiderpig.config import chord_r

    def width(name: str, a: str, b: str) -> float:
        return 2 * chord_r(config, name, a, b)

    from spiderpig.config import PRODUCT_BOSS_R

    bosses = PRODUCT_BOSS_R.get(lk.key, {})
    frames = {tag: cs.link_bending(lk, pts, joints, v, sections, holes, {link: on[tag]}, sk,
                                   width, bosses) for tag, (v, sk, _) in cases.items()}
    side, length = _side_posts(config, notes, joints, pts, mech)
    faces = side_plates(config, pts)
    o_joint = next(jk for jk, (j, b1, b2) in enumerate(joints)
                   if j == lk.crank[0] and b2 == "ground")
    z_rod = rod_height(config, notes, mech.meta.get("hand_crank"))

    def turns(v: np.ndarray) -> dict[str, np.ndarray]:
        """The gearbox's turn on the outer plate at the crank: its shaft carries the rod's
        force from the crankpin, the rod's mid-plane over the plate (``z_rod``)."""
        f = -v[:, o_joint]
        return {lk.crank[0]: z_rod * np.stack([-f[:, 1], f[:, 0]], -1)}

    stand = cr.stand_of(lk)
    xy = faces.xy
    t_frame = faces.t
    y_frame = allowable_mpa(config, config.frame_sheet)
    widths = _stand_widths(stand)
    post_nodes = [p.name.split(":", 1)[-1] for p in side]
    web = _frame_web(config)
    stiff = side_stiffness(config, side, length, faces, len(joints))
    mid = side_stiffness(config, side, length, replace(faces, mid=True), len(joints))
    for k in ("pivots_n_mm", "across_n_mm", "crank_n_mm"):
        for n, kv in mid[k].items():
            stiff[k][n] = min(stiff[k][n], kv)
    stiff["least_n_mm"] = min(stiff["pivots_n_mm"].values())
    # the side rocks on its rail's play in the cleats before they bear (design review round
    # 3's M6): 2 x the play over the rail's bearing height, at each pivot's height over the
    # board; the reported sway is the elastic one and that rocking
    board = float(pts[cr.mouth_name(lk)][0][1]) + cr.BASE_TOP
    rock = {n: 2 * cr.CLEAT[1] / cr.CLEAT_BEAR * (float(xy[n][1]) - board)
            for n in stiff["pivots_n_mm"]}
    stiff["elastic_mm"] = round(SWAY_PUSH_N / (2 * stiff["least_n_mm"]), 2)
    stiff["rock_mm"] = {n: round(r, 2) for n, r in rock.items()}
    stiff["sway_mm"] = round(max(SWAY_PUSH_N / (2 * k) + rock[n]
                                 for n, k in stiff["pivots_n_mm"].items()), 2)

    def in_plane(got_t: dict, v: np.ndarray, sk: np.ndarray, torque: np.ndarray) -> dict:
        """Both plates in their plane: the outer pinned at its feet, the posts' feet and the
        gearbox on it; the inner floating on its posts' ends."""
        keep = ~sk
        ld_o = {(p.name.split(":", 1)[-1]): got_t[p.name]["forces"]["box"]["foot"]
                for p in side}
        ld_o[lk.crank[0]] = ld_o.get(lk.crank[0], 0.0) - v[keep, o_joint]
        outer = cs.members({k: xy[k] for k in {*(q for pr in stand.outer for q in pr),
                                               *ld_o, *_on_members(stand, "outer")}},
                           stand.outer, widths, t_frame, y_frame, ld_o, stand.feet,
                           (lk.crank[0], -torque[keep] * 1000.0),
                           tied=[*stand.feet, *post_nodes], across_k=stiff["across_n_mm"])
        ld_i = {(p.name.split(":", 1)[-1]): -got_t[p.name]["forces"]["box"]["end"]
                for p in side}
        inner = cs.members({k: xy[k] for k in {*(q for pr in stand.inner for q in pr),
                                               *ld_i, *_on_members(stand, "inner")}},
                           stand.inner, widths, t_frame, y_frame, ld_i, (), tied=post_nodes)
        return {"outer": outer, "inner": inner}

    # each side as one frame, rated both ways its plates may be idealized (design review
    # round 3's minor 2: at their faces, where the posts end, and at their mid-planes, where
    # they bend, each post's end a rigid half-thickness off it; the worse of the two per
    # row): first order, then the plates' members' in-plane forces (and the posts' axial
    # ones) on the frame's bending out of the plates, second order (P-Delta) and its
    # buckling factor per angle (round 3's M2); every member end out of its plane with its
    # in-plane stress (round 3's minor 1)
    got: dict[str, dict] = {}
    plates: dict[str, dict] = {}
    lam: dict[str, tuple[float, float, float]] = {}

    def chain(plates_, v, sk, torque) -> tuple:
        """One case's side frame first order, its plates in their plane, then second order
        (memoized by the mechanism, the idealization and the case's vectors: the walking
        case is the same in every call of a report)."""
        key = (id(mech), plates_.mid, hash(np.ascontiguousarray(v).tobytes()),
               hash(np.ascontiguousarray(sk).tobytes()), hash(np.asarray(torque).tobytes()),
               round(length, 6), z_rod)
        hit = _CHAIN.get(key)
        if hit is not None and hit[0] is mech:
            return hit[1]
        first = cs.side_frame_solve(side, length, v, plates_, sk, turns=turns(v),
                                    second_order=False)
        mem = in_plane(first.posts, v, sk, torque)
        axial = {pl: cs.grillage_axial(getattr(plates_, pl), xy, mem[pl]["forces"])
                 for pl in ("outer", "inner")}
        frame = cs.side_frame_solve(side, length, v, plates_, sk, turns=turns(v),
                                    axial=axial, second_order=True)
        if len(_CHAIN) > 24:
            _CHAIN.clear()
        _CHAIN[key] = (mech, (mem, axial, frame))
        return mem, axial, frame

    for plates_ in (faces, replace(faces, mid=True)):
        for tag, (v, sk, torque) in cases.items():
            mem, axial, frame = chain(plates_, v, sk, torque)
            if frame.buckling is not None and len(frame.buckling):
                i = int(np.argmin(frame.buckling))
                at = float(np.degrees(ts[~sk][i]))
                comp = max((float(-f[i, 0]) for pl in axial.values() for f in pl.values()),
                           default=0.0)
                if tag not in lam or frame.buckling[i] < lam[tag][0]:
                    lam[tag] = (float(frame.buckling[i]), at, max(comp, 0.0))
            have = got.setdefault(tag, {})
            for name, row in frame.posts.items():
                old = have.get(name)
                if old is None:
                    have[name] = {**row, "forces": {k: (dict(f) if isinstance(f, dict) else f)
                                                    for k, f in row["forces"].items()}}
                    continue
                for k in ("moment_nm", "ends_nm", "mid_nm", "shear_n", "foot_nm", "tip_nm",
                          "end_n", "drift_mm"):
                    old[k] = max(old[k], row[k])
                for part in ("foot", "end"):        # the larger end moment per angle
                    a_, b_ = old["forces"]["turn"][part], row["forces"]["turn"][part]
                    bigger = np.hypot(*b_.T) > np.hypot(*a_.T)
                    old["forces"]["turn"][part] = np.where(bigger[:, None], b_, a_)
            for plate, segs in (("outer", plates_.outer), ("inner", plates_.inner)):
                into = plates.setdefault(plate, {}).setdefault(tag, {})
                rows = [*mem[plate]["members"],
                        *cs.plate_rows(frame.grillages[plate], segs, xy, t_frame, y_frame,
                                       plate_holes(config, plate), web,
                                       inplane=axial[plate]).values()]
                for m_ in rows:
                    have_ = into.get(m_["member"])
                    if have_ is None:
                        into[m_["member"]] = dict(m_)
                        continue
                    if m_["safety"] < have_["safety"]:
                        have_.update({k: m_[k] for k in m_ if k not in ("compression_n",
                                                                       "buckling")})
                    if m_["buckling"] is not None and (have_["buckling"] is None
                                                       or m_["buckling"] < have_["buckling"]):
                        have_.update({k: m_[k] for k in ("compression_n", "buckling")})

    # the strength rows: the columns, each frame plate (its worst member), the side's
    # buckling, the pillars' moments for strength.joint_strength, the tie rods across the bay
    rated = []
    d, bore, _ = column_section(stand)
    for p in side:
        if not p.name.startswith("column:"):
            continue
        rated.append(_post_row(p.name, "column", {tag: got[tag][p.name] for tag in got},
                               d, bore, ROD_YIELD,
                               f"{d:g} mm 6061-T6511 rod column (as a {d:g} x {bore:g} tube "
                               "at its tapped ends)"))
    for plate, by in plates.items():
        row = {"joint": f"frame:{plate}", "kind": "frame", "links": [],
               "sheet": config.frame_sheet, "thickness_mm": t_frame,
               "section": f"{plate} frame plate's members", "what":
               f"the {plate} frame plate's members (in-plane stress, Euler out of the plane "
               "between their nodes, and at every member's ends out of the plane with the "
               "in-plane stress there, P-Delta: cradle_structure.plate_rows)"}
        for tag, ms in by.items():
            worst = min(ms.values(), key=lambda m_: min(m_["safety"], m_["buckling"] or
                                                        math.inf))
            sf = min(worst["safety"], worst["buckling"] or math.inf)
            row[tag] = {"safety": round(sf, 2), "governs": worst["member"],
                        "load_n": worst["compression_n"], "stress_mpa": worst["stress_mpa"]}
        rated.append(row)
    if lam:
        row = {"joint": "frame:side buckling", "kind": "frame", "links": [],
               "sheet": config.frame_sheet, "thickness_mm": t_frame,
               "section": "the side frame out of its plates' plane",
               "what": "the drive side's posts and both frame plates buckling as one frame "
                       "(cradle_structure.side_frame_solve: the least load factor over the "
                       "angles rated)"}
        for tag, (lv, at, comp) in lam.items():
            row[tag] = ({"safety": round(lv, 2), "governs": f"at crank {at:.0f}°",
                         "load_n": round(comp, 1)} if math.isfinite(lv) else None)
        rated.append(row)
    box = box_racking(config, joints, pts, cases, float(mech.meta["tie_mm"]))
    rated += box["rows"]
    pillars = {}
    for p in side:
        if p.name.startswith("column:"):
            continue
        od = float(notes[f"pillar:{p.name}"]["section"]["bearing_d"])
        pillars[p.name] = {tag: {"moment_nmm": got[tag][p.name]["moment_nm"] * 1000.0,
                                 "bending_mpa": round(post_bending(got[tag][p.name], od,
                                                                   ROD_ROOT), 1),
                                 "shear_n": got[tag][p.name]["shear_n"],
                                 "foot_nm": got[tag][p.name]["foot_nm"],
                                 "end_n": got[tag][p.name]["end_n"]} for tag in got}
    strip = {tag: {name: {k: v for k, v in g.items() if k != "forces"}
                   for name, g in by.items()} for tag, by in got.items()}
    return {"crank_nm": crank_nm, "skipped_deg": round(float(skip.sum()) * 360.0 / len(ts), 1),
            "plates_gap_mm": round(length, 2), "rod_over_plate_mm": round(z_rod, 2),
            "link_frames": frames, "posts": strip,
            "plates": {plate: {tag: list(ms.values()) for tag, ms in by.items()}
                       for plate, by in plates.items()},
            "buckling": {tag: {"factor": round(lv, 2) if math.isfinite(lv) else None,
                               "crank_deg": round(at, 1), "most_compression_n": round(c, 1)}
                         for tag, (lv, at, c) in lam.items()},
            "joints": post_joints(config, side, {tag: got[tag] for tag in got}),
            "stiffness": {k: v for k, v in stiff.items() if k != "across_n_mm"},
            "box": {k: v for k, v in box.items() if k != "rows"},
            "tie_threads": tie_threads(config, box),
            "rated": rated, "pillars": pillars}


_CHAIN: dict = {}       # structure's side-frame chain per case (its ``chain``)


def _on_members(stand: cr.Stand, plate: str) -> list[str]:
    """The stand's screw points on a plate's members (the feet's on the outer, a bay tie's on
    the inner): nodes there, so a member is rated at their holes."""
    if plate == "outer":
        return [*stand.feet, *stand.base_ties]
    return list(stand.bay_ties)


def _frame_web(config: BuildConfig) -> float:
    from spiderpig.construction.plates import boss_web

    return boss_web(config.frame_sheet)


def side_plates(config: BuildConfig, pts):
    """The drive side's two frame plates out of their plane for
    :func:`cradle_structure.side_frame`: each plate's members split at every node on them
    (posts, feet, screws), their widths, the frame sheet's thickness, the outer plate's
    feet."""
    from spiderpig import cradle_structure as cs
    from spiderpig.materials import sheet_of, thickness

    stand = cr.stand_of(config.lk)
    xy = _stand_xy(config, pts)
    widths = _stand_widths(stand)

    def segs(pairs) -> list:
        out = []
        for pr in pairs:
            w = widths.get(pr, widths.get(pr[::-1], 2 * stand.member_r))
            out += [(s, w) for s in cs._split(list(xy), xy, [pr], tol=1.0)]
        return out

    return cs.SidePlates(xy, segs(stand.outer), segs(stand.inner),
                         thickness(config, sheet_of(config, "frame")), stand.feet)


def plate_holes(config: BuildConfig, plate: str) -> dict[str, float]:
    """The holes a frame plate has at the stand's nodes (mm): the posts' end screws' (a
    pillar's, a column's), the feet's screws (the outer), a bay tie's (the inner)."""
    from spiderpig.construction.pivots.standoff import StandoffAxle

    lk = config.lk
    stand = cr.stand_of(lk)
    out = dict.fromkeys((j for j in lk.frame if j != lk.crank[0]),
                        StandoffAxle().cradle_shaft().end_hole)
    out.update(dict.fromkeys(stand.columns, cr.column_parts(stand).hole))
    out.update(dict.fromkeys(_on_members(stand, plate), cr.TIE_HOLE))
    return out


def rod_height(config: BuildConfig, notes: dict, hand: dict | None = None) -> float:
    """The connecting rod's mid-plane over the outer plate's inner face (mm): where the
    gearbox's shaft takes the rod's force from the crankpin, so the shaft turns the outer
    plate at the crank by the force times this. From the hand crank's own notes where given
    (``hand``: the gearbox on its riser under the plate, the crank block over it, the rod's
    mid-plane, ``rod_mid_z_mm``, over ``plate_inner_z_mm``), else the plan's z from the pins'
    wobble notes (``notes``)."""
    from spiderpig.materials import sheet_of, thickness

    if hand and "rod_mid_z_mm" in hand and "plate_inner_z_mm" in hand:
        return float(hand["rod_mid_z_mm"]) - float(hand["plate_inner_z_mm"])
    lk = config.lk
    rider = next(b for b, (js, _) in lk.links.items() if lk.crank[1] in js)
    face = None
    mid = None
    for note in notes.values():
        lz = note.get("layer_z") or {}
        if face is None and "0" in lz:
            face = float(lz["0"][1])
        k = (note.get("layers") or {}).get(rider)
        if k is not None and str(k) in lz:
            mid = float(lz[str(k)][0]) + thickness(config, sheet_of(config, "link", rider)) / 2
    if face is None or mid is None:
        raise ValueError(f"{lk.key}: no wobble note gives the rod's or the plates' z")
    return mid - face


SWAY_PUSH_N = 20.0      # a hand's sideways push on the bottle (the side's stiffness across)


def side_stiffness(config: BuildConfig, side, length: float, plates, n_joints: int) -> dict:
    """The drive side's stiffness across its plates (design review round 2's M3: the frames
    soft out of their plane): a push across at each pillar's node (the pivots, where the
    cradle box's sideways load comes in through the links) and at every untied node of the
    outer plate (the crank's O: what holds its members' sway, for
    :func:`cradle_structure.members`), the posts, both plates and the feet as
    :func:`cradle_structure.side_frame` takes them (N/mm); the pivots' sway under
    :data:`SWAY_PUSH_N` at the bottle, both sides sharing it (mm)."""
    from spiderpig import cradle_structure as cs

    lk = config.lk
    memo = (lk.key, round(length, 6), n_joints, plates.t, plates.mid, tuple(plates.feet),
            tuple((ab, w) for ab, w in (*plates.outer, *plates.inner)),
            tuple((n, tuple(np.round(np.asarray(p, float), 6))) for n, p in
                  sorted(plates.xy.items())),
            tuple((p.name, tuple(np.round(p.xy, 6)), p.ei, p.ea) for p in side))
    hit = _STIFF_MEMO.get(memo)
    if hit is not None:
        return {k: (dict(v) if isinstance(v, dict) else v) for k, v in hit.items()}
    stand = cr.stand_of(lk)
    zero = np.zeros((1, n_joints, 2))
    outer_nodes = {q for (a, b), _ in plates.outer for q in (a, b)}
    tied = {*stand.feet, *(p.name.split(":", 1)[-1] for p in side)}
    pivots = [j for j in lk.frame if j != lk.crank[0]]
    out: dict = {"across_n_mm": {}, "pivots_n_mm": {}}
    for n in sorted(outer_nodes - set(stand.feet)):
        if n not in pivots and n in tied:
            continue
        _, gr = cs.side_frame(side, length, zero, plates, push={n: 1.0})
        k = 1.0 / max(abs(float(gr["outer"].nodes[n][0, 0])), 1e-12)
        if n in pivots:
            out["pivots_n_mm"][n] = round(k, 2)
        else:
            out["across_n_mm"][n] = k
    least = min(out["pivots_n_mm"].values())
    out["least_n_mm"] = least
    out["push_n"] = SWAY_PUSH_N
    out["sway_mm"] = round(SWAY_PUSH_N / (2 * least), 2)
    out["crank_n_mm"] = {n: round(k, 2) for n, k in out["across_n_mm"].items()}
    out["basis"] = ("a push across the plates at the node, the side's posts and both plates "
                    "out of their plane (cradle_structure.side_frame), the outer plate's feet "
                    "held by their cleats; the sway: the push at the bottle shared by both "
                    "sides, at the least stiff pivot")
    if len(_STIFF_MEMO) > 16:
        _STIFF_MEMO.clear()
    _STIFF_MEMO[memo] = out
    return {k: (dict(v) if isinstance(v, dict) else v) for k, v in out.items()}


_STIFF_MEMO: dict = {}      # side_stiffness by every input (it doesn't move with the loads)


PASSIVE_SHARE = 0.5     # the passive half's share of what the cradle box carries: the bottle
#                         and the box straddle the bay's middle, and a block on a saddle or on
#                         the bottle comes in there (a decanter's rim under the mouth)
TIE_PROOF_N = 20.1 * 450.0     # an M6 A2-70 screw's proof load (stress area 20.1 mm^2, ISO
#                                3506-1's 450 MPa): the posts' button heads (the tie rods'
#                                class 10.9 ones: fasteners.BUTTON_10_9_PROOF_N)


def box_racking(config: BuildConfig, joints, pts, cases: dict, tie_mm: float) -> dict:
    """The cradle box across the bay (design review round 2's M1, 2026-10-15). The passive
    side holds its cradle link in place but lets it turn about the mouth (B's double
    parallelogram, A's hinge at G), so what the passive half of the box carries about the
    mouth (:data:`PASSIVE_SHARE` of what the drive side's pins put on the cradle: half the
    bottle's and the box's weight walking, half the jam's couple or block) crosses the bay
    in the four tie rods racking: the two cradle links turn against each other about the
    mouth's axis, each rod a steel beam fixed at both ends (``12 E I / L^3`` across at its
    radius from the mouth, ``G J / L`` in twist). Per case: the moment crossing, each rod's
    shear, end moment and twist, its tapped end's stress (the tube its tap leaves, 1018 at
    its least yield), the end joint's gapping moment (its preload, ``TIE_TIGHTEN_NM`` on the
    screw, over the end face's kern) and, gapped, the screw's prying tension against what its
    head carries (a class 10.9 button head, ``fasteners.BUTTON_10_9_PROOF_N``); the strength
    rows (``"rows"``: the rods, their end joints)."""
    from spiderpig import cradle_structure as cs
    from spiderpig.linkages.decanters import TIES

    lk = config.lk
    link = cr.cradle_link(lk)
    mouth = pts[cr.mouth_name(lk)]
    d, L = cr.TIE_D, tie_mm
    bore = NETRF12_ROOT if d > 10.0 else NETRF10_ROOT
    ei = cs.E_STEEL * math.pi * d ** 4 / 64
    gj = cs.E_STEEL / 2.6 * math.pi * d ** 4 / 32
    k_s, k_t = 12 * ei / L ** 3, gj / L
    r = {n: math.hypot(*TIES[n]) for n in TIES}
    k_rot = sum(k_s * ri * ri + k_t for ri in r.values())
    z_tube = math.pi * (d ** 4 - bore ** 4) / (32 * d)
    screw_d = cr.TIE_M.d
    from spiderpig.hardware.fasteners import BUTTON_10_9_PROOF_N, preload_band

    proof = BUTTON_10_9_PROOF_N[f"{screw_d:g}"]
    preload = preload_band(f"{screw_d:g}", cr.TIE_TIGHTEN_NM)[0]   # (its least: friction 0.2)
    kern = (d * d + cr.TIE_ROD_HOLE ** 2) / (8 * d)
    gap_nm = preload * kern / 1000.0
    out: dict = {"rod": f"NETRF{d:g} x {L:g} mm, M{screw_d:g} ends", "share": PASSIVE_SHARE,
                 "radii_mm": {n: round(v, 1) for n, v in r.items()},
                 "gap_moment_nm": round(gap_nm, 2), "preload_n": round(preload),
                 "screw_proof_n": round(proof), "cases": {}, "rows": []}
    rod_row: dict = {"joint": "box:tie rods", "kind": "box", "links": [],
                     "section": f"{d:g} mm steel standoff tie rods across the bay (NETRF{d:g}, "
                                f"as a {d:g} x {bore:g} tube at their tapped ends)",
                     "what": "the cradle box's tie rods racking across the bay (the passive "
                             "half's moment about the mouth)"}
    joint_row: dict = {"joint": "box:tie rod ends", "kind": "box", "links": [],
                       "section": f"the tie rods' M{screw_d:g} end screws (ISO 7380, class "
                                  "10.9) prying a gapped end joint",
                       "what": "the tie rods' end joints",
                       "walk": None}
    for tag, (v, sk, _) in cases.items():
        keep = ~np.asarray(sk, bool)
        m = np.zeros(len(v))
        for jk, (j, b1, b2) in enumerate(joints):
            if link not in (b1, b2):
                continue
            f = v[:, jk] if b1 == link else -v[:, jk]
            q = pts[j] - mouth
            m += q[:, 0] * f[:, 1] - q[:, 1] * f[:, 0]
        cross = PASSIVE_SHARE * np.abs(m[keep]) / 1000.0        # N·m
        mc = float(cross.max()) if len(cross) else 0.0
        theta = mc * 1000.0 / k_rot
        rows = {}
        worst = math.inf
        for n, ri in r.items():
            shear = k_s * ri * theta
            end = shear * L / 2
            twist = k_t * theta
            vm = math.hypot(end / z_tube, math.sqrt(3) * twist / (2 * z_tube))
            sf = NETRF10_YIELD / max(vm, 1e-9)
            worst = min(worst, sf)
            rows[n] = {"shear_n": round(shear, 1), "end_nm": round(end / 1000, 2),
                       "twist_nm": round(twist / 1000, 2), "stress_mpa": round(vm, 1),
                       "safety": round(sf, 2)}
        top = max(rows.values(), key=lambda x: x["end_nm"])
        prying = top["end_nm"] * 1000.0 / (d / 2)            # pivoting on its face's edge
        out["cases"][tag] = {"crossing_nm": round(mc, 2), "rods": rows,
                             "gap_ratio": round(gap_nm / max(top["end_nm"], 1e-9), 2),
                             "prying_n": round(prying),
                             "all_on_passive_safety": round(worst / 2, 2)}
        rod_row[tag] = {"load_n": max(x["shear_n"] for x in rows.values()),
                        "moment_nm": top["end_nm"], "safety": round(worst, 2)}
        if tag == "jam":
            joint_row["jam"] = {"load_n": round(prying), "moment_nm": top["end_nm"],
                                "safety": round(proof / max(prying, 1e-9), 2)}
    out["rows"] = [rod_row, joint_row]
    return out


POST_FACE = (12.7, 6.4)     # a post's end face on its plate (mm): the 1/2 in rod's end over the
#                             plate's 6.4 mm hole
M6_STRIP_AREA = 12.19       # mm^2 of an M6 x 1 internal thread's shear area per mm engaged (FED-
#                             STD-H28/2 Appendix A, A_n = pi n L D_s,min (1/2n + 0.57735 (D_s,min -
#                             E_n,max)): 6g's 5.794 major on 6H's 5.500 pitch diameter)
LEAD_THREAD = 0.5           # the tap's countersunk first thread, not counted engaged (mm)


def thread_strip(engaged: float) -> tuple[float, float]:
    """An M6 thread tapped in a 1/2 in 6061-T6511 post's or rod's end, ``engaged`` mm: what
    strips it (N) at the rod's shear yield (von Mises, 0.577 x its minimum yield) and at its
    shear ultimate (:data:`hardware.cradle_catalog.ROD_SHEAR`)."""
    area = M6_STRIP_AREA * max(engaged - LEAD_THREAD, 0.0)
    return area * 0.577 * ROD_YIELD, area * ROD_SHEAR


def post_engaged(config: BuildConfig) -> float:
    """How far a post's M6 x 16 end screw reaches into its tapped end (mm): its length less
    the DIN 125 washer under its head, the frame plate and the fender washer between the
    plate and the post's end (``construction.cradle.column_parts``; every post's both ends,
    :data:`POST_WASHED`)."""
    from spiderpig.hardware.catalog import get
    from spiderpig.materials import sheet_of, thickness

    col = cr.column_parts(cr.stand_of(config.lk))
    t = thickness(config, sheet_of(config, "frame"))
    w = float(get(col.washer).dims["t"]) if col.washer else 0.0
    f = float(get(col.face).dims["t"]) if col.face else 0.0
    return col.screw_len - w - t - f


def tie_threads(config: BuildConfig, box: dict) -> dict:
    """The tie rods' tapped ends (the simplification round 1, 2026-10-21: 6061, tapped at home):
    each end's M6 thread (``construction.cradle.TIE_ROD_SCREW`` through the cradle link) against
    the most its screw pulls: its preload's top at ``TIE_TIGHTEN_NM`` walking, the prying of a
    gapped end at the jam (:func:`box_racking`'s), and against the screw's head proof (the
    screw, not the thread, should go first). Strip SF at the rod's shear yield."""
    from spiderpig.hardware.fasteners import BUTTON_10_9_PROOF_N, preload_band
    from spiderpig.materials import sheet_of, thickness

    link = cr.cradle_link(config.lk)
    t = thickness(config, sheet_of(config, "link", link))
    engaged = cr.TIE_ROD_SCREW - t
    strip_y, strip_u = thread_strip(engaged)
    top = preload_band(f"{cr.TIE_M.d:g}", cr.TIE_TIGHTEN_NM)[1]
    proof = (BUTTON_10_9_PROOF_N[f"{cr.TIE_M.d:g}"] if cr.TIE_M.kind == "bhcs109"
             else TIE_PROOF_N)
    out = {"engaged_mm": round(engaged - LEAD_THREAD, 2), "strip_yield_n": round(strip_y),
           "strip_ultimate_n": round(strip_u), "screw_proof_n": round(proof),
           "over_proof": round(strip_y / proof, 2)}
    for tag, case in box["cases"].items():
        pull = max(top, float(case["prying_n"])) if tag == "jam" else top
        out[tag] = {"screw_n": round(pull), "strip_sf": round(strip_y / pull, 2)}
    return out
POST_WASHER = (18.0, 1.6)   # the 18-8 fender washer under each post's foot (OD, its least
#                             thickness): it spreads the face's edge into the outer plate
POST_WASHED: dict[str, tuple[str, ...]] = {"pillar": ("foot", "end"),
                                           "column": ("foot", "end")}
"""The post ends a fender washer spreads into its plate, by kind (design review round 3,
laser-cut parts M1): both ends of every post (a pillar's inner one in the clearance gap
under the inner plate), so every post of a side is one length."""
P_LIMIT_MPA = 269.0         # the frame plate's limiting surface pressure under a post's end
#                             (VDI 2230 Part 1, Table A9, p_G: 5052 isn't tabulated; AlMg4.5Mn
#                             F27, 5083, 325 MPa at Rm 260, scaled to 5052-H32's 215 minimum,
#                             ASTM B209M; transcribed by FVA's help and Bossard's
#                             "Flaechenpressung" sheet, fetched 2026-10-20: "short-term values
#                             at room temperature, guide values"). VDI 2230's R10 holds the
#                             pressure under it (S_P >= 1)
P_LIMIT_BASIS = ("VDI 2230-1 (2015) Table A9's limiting surface pressure p_G, AlMg4.5Mn F27 "
                 "325 MPa scaled to 5052-H32's 215 MPa minimum tensile: 269 MPa; R10's S_P >= "
                 "1 (adoc.fva-service.de/help/8.0/de/attr/PMID_19259.htm, Bossard's "
                 "Flaechenpressung sheet 01-2025)")
SLIP_MU = 0.2               # a post's end face on aluminium, threadlocker on it: the least
#                             friction a slip is rated at (steel on aluminium dry 0.47-0.61)


def contact_peak(clamp: np.ndarray, moment: np.ndarray, od: float, bore: float) -> np.ndarray:
    """The peak pressure (MPa) an annulus ``od`` / ``bore`` (mm) clamped by ``clamp`` (N)
    takes under ``moment`` (N·mm), per case: linear while it stays closed (``F / A + M /
    Z``), past its kern the no-tension solution (the pressure linear over what still bears,
    nothing where it lifts: the face's edge carrying it), ``inf`` where the clamp can't hold
    the moment at all (past ``F od / 2``)."""
    clamp = np.maximum(np.asarray(clamp, float), 1e-9)
    moment = np.abs(np.asarray(moment, float))
    area = math.pi * (od * od - bore * bore) / 4
    z = math.pi * (od ** 4 - bore ** 4) / (32 * od)
    kern = z / area
    out = clamp / area + moment / z
    # the annulus on a polar grid; the lifted part's edge (the neutral axis, c) found by
    # bisection on where the pressure's resultant lies, every case at once
    rr = np.linspace(bore / 2, od / 2, 17)
    th = np.linspace(0.0, 2 * math.pi, 97)[:-1]
    r_, t_ = np.meshgrid((rr[:-1] + rr[1:]) / 2, th, indexing="ij")
    da = (r_ * np.diff(rr)[:, None] * (2 * math.pi / len(th))).ravel()
    x = (r_ * np.cos(t_)).ravel()
    open_ = np.nonzero(moment > clamp * kern)[0]
    if len(open_):
        e = moment[open_] / clamp[open_]            # the resultant's offset (mm)
        lo, hi = np.full(len(open_), -od / 2), np.full(len(open_), od / 2)
        for _ in range(48):
            c = (lo + hi) / 2
            w = np.maximum(x[None, :] - c[:, None], 0.0)
            arm = (w * x * da).sum(axis=1) / np.maximum((w * da).sum(axis=1), 1e-300)
            lo, hi = np.where(arm < e, c, lo), np.where(arm < e, hi, c)
        w = np.maximum(x[None, :] - ((lo + hi) / 2)[:, None], 0.0)
        peak = clamp[open_] / np.maximum((w * da).sum(axis=1), 1e-300) * w.max(axis=1)
        out[open_] = np.where(e >= od / 2 * 0.98, np.inf, peak)
    return out


def post_joints(config: BuildConfig, side, got: dict) -> dict:
    """Each post's screwed ends (design review round 3's M5): its end face (``POST_FACE``)
    clamped to its plate by an M6 A2-70 button head at ``fasteners.BUTTON_A2_70_NM``, or at a
    foot ``construction.cradle.STEEL_FEET`` names the tie rods' class 10.9 one at
    ``construction.cradle.TIE_TIGHTEN_NM`` (each the preload band of ``fasteners.preload_band``
    at friction 0.1-0.2; the 10.9's proof its head's, ``fasteners.BUTTON_10_9_PROOF_N``),
    under the moment, shear
    and pull the side frame gives it there (``got``: per case, :func:`structure`'s posts with
    their per-angle forces). Per end and case: the moment over the face's decompression at
    the least preload (``open_x``: 1 the face's edge lifts, ``kern``), the screw's tension
    once it opens (the moment over the face's radius, its preload at least) over A2-70's
    proof (``screw_sf``), the face's peak pressure on the plate at the most preload (the
    no-tension contact, :func:`contact_peak`; at an end with a fender washer,
    ``POST_WASHED``, spread through it at 45°) over the plate's limiting surface pressure
    (:data:`P_LIMIT_MPA`, ``bearing_sf``: VDI 2230's S_P), what strips the M6 thread tapped
    in the 6061 post's end over the screw's most tension (``strip_sf``, :func:`thread_strip`
    over :func:`post_engaged`: the simplification round 1, 2026-10-21),
    and the shear over the face's friction at the least preload (``slip_x``). Walking, every
    face must stay shut and not slip (fretting each pour); at the clutch's cap a face may
    lift (the guide re-tightens them after a slip) if the screw and the plate hold."""
    from spiderpig.hardware.fasteners import BUTTON_10_9_PROOF_N, BUTTON_A2_70_NM, preload_band

    nm, steel_nm = BUTTON_A2_70_NM["6"], cr.TIE_TIGHTEN_NM
    steel = cr.STEEL_FEET.get(config.linkage, ())
    bands = {False: preload_band("6", nm), True: preload_band("6", steel_nm)}
    proofs = {False: TIE_PROOF_N, True: BUTTON_10_9_PROOF_N["6"]}
    od, bore = POST_FACE
    area = math.pi * (od * od - bore * bore) / 4
    kern = math.pi * (od ** 4 - bore ** 4) / (32 * od) / area
    engaged = post_engaged(config)
    strip_y, strip_u = thread_strip(engaged)
    w_od, w_t = POST_WASHER
    spread = min(w_od, od + 2 * w_t)                # its plate's face under the washer
    plate = P_LIMIT_MPA
    rows: dict[str, dict] = {}
    worst: dict[str, dict] = {}
    for p in side:
        for tag, by in got.items():
            g = by.get(p.name)
            if g is None:
                continue
            fo = g["forces"]
            pull = np.maximum(np.asarray(fo.get("axial", np.zeros(1)), float), 0.0)
            for end in ("foot", "end"):
                m = np.hypot(*np.asarray(fo["turn"][end], float).T)
                v = np.hypot(*np.asarray(fo["box"][end], float).T)
                if not len(m):
                    continue
                is_steel = end == "foot" and p.name in steel
                f_lo, f_hi = bands[is_steel]
                c_lo = np.maximum(f_lo - pull, 0.0)
                c_hi = np.maximum(f_hi - pull, 1e-9)
                open_x = m / np.maximum(c_lo * kern, 1e-9)
                screw = np.maximum(f_hi, m / (od / 2) + pull)
                kind = "column" if p.name.startswith("column:") else "pillar"
                face = spread if end in POST_WASHED[kind] else od
                peak = contact_peak(c_hi, m, face, bore)
                slip = v / np.maximum(SLIP_MU * c_lo, 1e-9)
                i = int(np.argmax(open_x))
                row = {"moment_nm": round(float(m.max()) / 1000, 2),
                       "shear_n": round(float(v.max()), 1),
                       "pull_n": round(float(pull.max()), 1),
                       "open_x": round(float(open_x.max()), 2),
                       "screw_n": round(float(screw.max())),
                       "screw_sf": round(proofs[is_steel] / float(screw.max()), 2),
                       "peak_mpa": (round(float(peak.max()), 1)
                                    if math.isfinite(float(peak.max())) else None),
                       "bearing_sf": round(plate / float(peak.max()), 2),
                       "strip_sf": round(strip_y / float(screw.max()), 2),
                       "slip_x": round(float(slip.max()), 2), "at": int(i)}
                rows.setdefault(f"{p.name} {end}", {})[tag] = row
                w = worst.setdefault(tag, {"open_x": 0.0, "slip_x": 0.0, "screw_sf": math.inf,
                                           "bearing_sf": math.inf, "strip_sf": math.inf})
                for k in ("open_x", "slip_x"):
                    if row[k] > w[k]:
                        w[k], w[k + "_at"] = row[k], f"{p.name} {end}"
                for k in ("screw_sf", "bearing_sf", "strip_sf"):
                    if row[k] < w[k]:
                        w[k], w[k + "_at"] = row[k], f"{p.name} {end}"
    f_lo, f_hi = bands[False]
    out = {"screw": f"M6 x 16 button head A2-70 (ISO 7380), {nm:g} N·m",
           "preload_n": [round(f_lo), round(f_hi)], "kern_mm": round(kern, 2),
           "decompression_nm": [round(f_lo * kern / 1000, 2), round(f_hi * kern / 1000, 2)]}
    if steel:
        h_lo, h_hi = bands[True]
        out["steel_feet"] = {"pillars": list(steel),
                             "screw": f"M6 x 16 button head class 10.9 (ISO 7380), "
                                      f"{steel_nm:g} N·m",
                             "preload_n": [round(h_lo), round(h_hi)],
                             "proof_n": round(BUTTON_10_9_PROOF_N["6"]),
                             "decompression_nm": [round(h_lo * kern / 1000, 2),
                                                  round(h_hi * kern / 1000, 2)]}
    out["thread"] = {"engaged_mm": round(engaged - LEAD_THREAD, 2),
                     "strip_yield_n": round(strip_y), "strip_ultimate_n": round(strip_u),
                     "over_proof": round(strip_y / TIE_PROOF_N, 2),
                     "basis": "M6 x 1 tapped in 6061-T6511 (FED-STD-H28/2's internal shear area "
                              "over the engaged length less the first thread; 0.577 x ASTM "
                              "B221's 241 MPa minimum yield)"}
    return {**out,
            "proof_n": round(TIE_PROOF_N), "plate_mpa": plate,
            "plate_basis": P_LIMIT_BASIS, "slip_mu": SLIP_MU,
            "washer_mm": list(POST_WASHER), "rows": rows, "worst": worst}


def strength_check(config: BuildConfig, mech, rows: list[dict], crank_nm: float,
                   struct: dict | None = None) -> dict:
    """The pins and link plates rated by :func:`spiderpig.strength.check` at the cradle's
    loads: "walk" a sealed full 6 L methuselah's, "jam" a firm push at the grip on a blocked
    cradle (per side, as :func:`report` shares them); with ``struct`` (:func:`structure`),
    the links in bending, the pillars and columns on the box and the frame plates' members
    from the force vectors."""
    from spiderpig import strength

    st = strength.check(mech.meta.get("wobble") or {}, mech.meta, config,
                        loads_doc(config, rows, crank_nm, struct))
    keep = ("joint", "kind", "links", "span_mm", "section", "sheet", "thickness_mm")
    return {"rows": [{**{k: r[k] for k in keep if k in r},
                      "walk_sf": (r.get("walk") or {}).get("safety"),
                      "jam_sf": (r.get("jam") or {}).get("safety"),
                      "walk_n": (r.get("walk") or {}).get("load_n"),
                      "jam_n": (r.get("jam") or {}).get("load_n")} for r in st["rows"]],
            "worst": st["worst"], "findings": st["findings"]}


def blocked(config: BuildConfig, mech, *, ts=None, crank_nm: float | None = None,
            thresholds: bool = False, rod_euler_n: float | None = None) -> dict:
    """The known limits: the jam's push (``crank_nm``, default :func:`jam_nm`) against a
    cradle blocked at a point (:data:`BLOCKS`) rather than as a couple, outside the
    toggles: per case, the force there, each joint's largest force, and every joint's,
    link's, column's, plate's and tie rod's jam safety factor (:func:`spiderpig.strength.check`
    on those loads). With ``thresholds`` (design review round 2's M2, 2026-10-15), per case
    where the frame starts to give: the walking loads plus the push beyond what turning the
    cradle takes, ``dT = T - T_walk(angle) >= 0`` (a block can only push back), superposed
    (:func:`_block_threshold`): the first part to fall to SF 1.3 and to 1, at how much more
    torque at the crank and force at the grip, and the block's force then (``rod_euler_n``:
    the rod's Euler load, its buckling rated too). Not designed for: a clutch can't help
    with a block 72 mm from the mouth, the use limits keep the decanter's rim under the
    cradle box (a fixed riser, round 2's stand)."""
    from spiderpig import strength

    if ts is None:
        ts = np.linspace(0.0, 2.0 * math.pi, 360, endpoint=False)
    if crank_nm is None:
        crank_nm = jam_nm(config)
    rigid, joints = planar_model(config, mech, 1.0, cr.DESIGN)
    near = _toggles(config, ts)
    torque, forces = statics(config, rigid, joints, ts)
    rider = next(b for b, (js, _) in config.lk.links.items() if config.lk.crank[1] in js)
    out: dict = {"crank_nm": crank_nm, "cases": {}}
    for name, at in BLOCKS.items():
        held, jam_f = statics(config, rigid, joints, ts, jam_nm=crank_nm, block=at)
        jam_f = np.where(near[:, None], 0.0, jam_f)
        rows = joint_rows(config, rider, joints, forces, jam_f)
        struct = structure(config, mech, crank_nm=crank_nm, ts=ts, skip=near, block=at)
        st = strength.check(mech.meta.get("wobble") or {}, mech.meta, config,
                            loads_doc(config, rows, float(np.abs(torque).max()), struct))
        sfs = {r["joint"]: (r.get("jam") or {}).get("safety") for r in st["rows"]}
        out["cases"][name] = {
            "at_mm": list(at), "radius_mm": round(math.hypot(*at), 1),
            "block_n": round(float(np.abs(np.where(near, 0.0, held)).max()), 1),
            "joints_n": {r["joint"]: r["jam_n"] for r in rows},
            "jam_sf": {k: v for k, v in sorted(sfs.items(), key=lambda kv: kv[1] or math.inf)
                       if v is not None},
            "worst": min((v, k) for k, v in sfs.items() if v is not None)}
        if thresholds:
            out["cases"][name]["threshold"] = _block_threshold(
                config, mech, ts, near, rigid, joints, at, crank_nm, rod_euler_n)
    return out


THRESHOLD_STEPS = (0.12, 0.3, 0.55, 1.0)
"""Where :func:`_block_threshold` rates the push beyond turning: these fractions of the most
the clutch lets past the walking torque at any angle, each angle's push no more than its own
(the safety factors' inverses are near linear in the push: the stresses are, but for the
walking loads' offset, the worst angle moving and the angles the clutch caps)."""


def _block_threshold(config: BuildConfig, mech, ts, near, rigid, joints, at, cap: float,
                     rod_euler_n: float | None) -> dict:
    """:func:`blocked`'s threshold for a block at ``at``: the loads at a push ``x`` beyond
    turning are the walking ones plus ``x`` times each angle's increment per N·m (the
    statics are linear in the crank's torque), each angle's push no more than the clutch lets
    past its own walking torque (``cap - T_walk``: where gravity helps the turn, far more
    than where it fights it; design review round 3's M3, the search had stopped at the least
    over every angle), rated by :func:`structure` and :func:`spiderpig.strength.check` at
    :data:`THRESHOLD_STEPS` of the most any angle takes; each part's ``1 / SF`` interpolated
    (and extrapolated from the first two below them) to where it reaches ``1 / 1.3`` and
    ``1``. Returns the first part each way, the push (N·m at the crank, N at the grip through
    the worm at a hand's speed: the least at any angle), the block's force, and the most
    push the clutch lets through at any angle (``within_nm``, ``within_grip_n``): a part
    that reaches neither within it holds the block until the clutch slips."""
    from spiderpig import strength

    walk_t, _, walk_v = statics_vectors(config, rigid, joints, ts)
    held, _, bv = statics_vectors(config, rigid, joints, ts, jam_nm=cap, block=at)
    keep = ~near
    d_t = np.maximum(cap - walk_t, 1e-6)
    inc = (bv - walk_v) / d_t[:, None, None]
    per_nm = np.abs(held) / d_t                       # the block's force per N·m pushed
    most = float(d_t[keep].max())                     # the push the clutch lets through
    rider = next(b for b, (js, _) in config.lk.links.items() if config.lk.crank[1] in js)
    rod_jk = next(jk for jk, (j, *_) in enumerate(joints) if j == config.lk.crank[1])
    walk_f = np.linalg.norm(walk_v, axis=-1)
    peak = float(np.abs(walk_t).max())
    inv: dict[str, list[float]] = {}
    xs = [most * s for s in THRESHOLD_STEPS]
    for x in xs:
        x_t = np.minimum(x, d_t)                      # each angle: no more than its clutch
        v = walk_v + x_t[:, None, None] * inc
        f = np.where(near[:, None], 0.0, np.linalg.norm(v, axis=-1))
        rows = joint_rows(config, rider, joints, walk_f, f)
        st = structure(config, mech, ts=ts, skip=near, block=at, jam=(walk_t + x_t, v))
        chk = strength.check(mech.meta.get("wobble") or {}, mech.meta, config,
                             loads_doc(config, rows, peak, st))
        sfs = {r["joint"]: (r.get("jam") or {}).get("safety") for r in chk["rows"]}
        if rod_euler_n:
            sfs["rod (buckling)"] = rod_euler_n / max(float(f[keep, rod_jk].max()), 1e-9)
        for k, sf in sfs.items():
            if sf is not None:
                inv.setdefault(k, []).append(1.0 / max(float(sf), 1e-9))

    def reach(ys: list[float], level: float) -> float | None:
        """Where the piecewise-linear ``ys`` over ``xs`` (extended back to 0 and on past the
        last) first reaches ``1 / level``."""
        target = 1.0 / level
        pts = list(zip(xs, ys, strict=True))
        (x0, y0), (x1, y1) = pts[0], pts[1]
        if y0 >= target:            # already there at the first step: back along its slope
            slope = (y1 - y0) / (x1 - x0)
            return max(0.0, x0 - (y0 - target) / slope) if slope > 1e-12 else 0.0
        for (xa, ya), (xb, yb) in itertools.pairwise(pts):
            if yb >= target:
                return xa + (target - ya) * (xb - xa) / (yb - ya)
        return None

    box = HandCrank().box               # N·m at the crank per N at the grip, at a hand's speed
    grip = box.ratio * box.eta_at(HAND_RPM) * handle_spec(config).throw / 1000.0
    out: dict = {"steps_nm": [round(x, 2) for x in xs], "basis": (
        "the walking loads plus the push beyond turning the cradle, superposed (the statics "
        "are linear in the crank's torque), every part rated as the report rates it")}
    for level, tag in ((JAM_FLOOR, "sf_1p3"), (1.0, "sf_1")):
        hits = [(x, k) for k, ys in inv.items() if (x := reach(ys, level)) is not None]
        hits = [(x, k) for x, k in hits if x <= most + 1e-9]   # within the clutch
        if not hits:
            out[tag] = None
            continue
        x, part = min(hits)
        x_t = np.minimum(x, d_t)
        out[tag] = {"part": part, "crank_nm": round(x, 2), "grip_n": round(x / grip, 1),
                    "block_n": round(float((x_t * per_nm)[keep].max()))}
    out["within_nm"] = round(most, 2)
    out["within_grip_n"] = round(most / grip, 1)
    return out


# -- the cradle page's swing (``/api/cradle/{key}``, ``viewer/decant.html``) ------------

VERDICT = {
    "decanter_rcm": ("build", "The main design: an eight-bar double parallelogram turns the "
                     "cradle about a point with nothing there, so nothing stands at the "
                     "mouth; its pivots hang under the mouth, its crank behind. It comes apart "
                     "by hand for the trip. With a full 6 L: a slip clutch in the hand crank, "
                     "set under a ceiling and checked at each set-up and every month, caps the "
                     "crank's torque however hard the push while its check holds, every joint "
                     "holds that cap at SF 1.3 or more away from the ends of the swing (in the "
                     "first and last two and a half handle turns it doesn't: the gearbox's red "
                     "arcs, the use limits), its side frames "
                     "included since their redesign (2026-10-20: two rail columns, a box "
                     "toward the rear), the gearbox carries the bottle and the cap within its "
                     "maximum, its light-load crank turns under 22 N once the gearbox is run "
                     "in (about 25 N new), the stand tips past 25°."),
    "decanter_hinge": ("build", "Builds too (B is the main design): a four-bar crank-rocker "
                       "whose rocker is the cradle, hinged at the bottle's mouth. From 50° "
                       "neck up (an opened full bottle goes in) to 30° neck down, a 750 ml to "
                       "a 6 L. With a full 6 L: a slip clutch in the hand crank, set under a "
                       "ceiling and checked at each set-up and every month, caps the crank's "
                       "torque however hard the push while its check holds, every joint holds "
                       "that cap at SF 1.3 or more away from the ends of the swing (in the "
                       "first and last two and a half handle turns it doesn't: the gearbox's "
                       "red arcs, the use limits), its side frames "
                       "included since their redesign (2026-10-20: a rail column under the "
                       "front leg, the strut to the gearbox widened), the gearbox carries the "
                       "bottle and the cap within its maximum, its 130 mm crank turns at the "
                       "22 N bar once the gearbox is run in, about 25 N new (OPEN_GRIP), the "
                       "stand tips past 15° (its bar)."),
}
"""Each cradle's standing (``docs/agentlib/DECANTER.md``): ``build`` or ``study``, and why: the
bars ``tests/test_cradle.py`` holds, not a copied number; :func:`report`'s ``verdict`` adds its
own numbers (:func:`cradle_drive.verdict`). In the cradle page's order: B, the user's main
design, first (design review round 2's M5: the page had opened on A, "the one to build")."""

OPEN_AT_CAP: dict[str, dict[str, str]] = {}
"""Per cradle, the parts that fall under the jam floor (``cradle_drive.JAM_FLOOR``) at the
hand crank's clutch's cap, and why: an open item, the user's call (the tests hold every other
part at 1.3 and nothing else under it, then xfail on these; the report's verdict names
them). None since the side frames' redesign (2026-10-20, ``docs/agentlib/DECANTER.md``, "The
side frames at the clutch's cap"): design review round 2 (2026-10-15) had rated each side's
posts and both frame plates as one frame (``cradle_structure.side_frame``) and found B's
outer plate (0.6), pillar O2 (0.73) and column D2 (0.97), A's outer plate (0.41), column D3
(0.83), inner plate (1.09) and pillar G (1.25) under it."""

OPEN_JOINTS: dict[str, dict[str, str]] = {}
"""Per cradle, the screwed joints open at their walking bar (every post's face shut walking at
its screw's least preload, :func:`post_joints`; the tie rods' end joints closed,
:func:`box_racking`'s ``gap_ratio``): the user's call; the tests hold each within 10 % and
xfail on it. None since 2026-10-20: design review round 3's M5 had put every M6 button head at
ISO 7380's 4 N·m in A2-70, and both cradles' tie rods' end joints stood at their gap walking
(0.98 of it at the least preload) and A's pillar G's foot lifted (1.06 x its decompression);
the tie rods' ends and that foot take class 10.9 button heads at 6 N·m
(``construction.cradle.TIE_M``, ``STEEL_FEET``): the same heads, no plan changed."""

OPEN_GRIP: dict[str, str] = {
    "decanter_hinge": "A's 130 mm crank, kept (design review round 3's m5): a full 6 L's "
                      "worst start is 21.97 N run in, at the 22 N bar, and 24.98 N while the "
                      "gearbox is new, over it, as B's 24.7 N new (21.7 run in); 132.5 mm "
                      "gives 21.55 N for 2.5 mm less hand room (-2.85 mm for a large hand), "
                      "125 mm 22.85 N; no radius in Figure 13's light row brings a new box "
                      "under the bar (B would need 140 mm, A 148)"}
"""Per cradle, a grip at MIL-STD-1472H's light-load bar (``handcrank.LIGHT_LOAD_N``) run in:
the user's call (``docs/agentlib/DECANTER.md``, "The drive"), settled in design review round
3 (both cranks are over the bar while the gearbox is new, the report's
``handle_force_band_n``; a hand crank never runs a box in, ``handcrank.RUNIN_LOSS``). The
tests hold it no worse than the bar, then xfail on it."""


def motion(config: BuildConfig, n: int = 360) -> dict:
    """A cradle's swing over one crank turn, sampled as its bake's clip is (``n`` crank angles
    from 0, the clip's time over its length): every joint's path (mm, the linkage's plane),
    the bottle's tilt (degrees, neck down positive), the handle's turns from rest (the clip's
    direction), how far the mouth wanders; the bodies' outlines, the drawn bottle, every
    bottle and the V blocks (at the drawn bottle's setting) in the cradle's frame (a point
    ``(x, y)`` there is at ``mouth + R(tilt) (x, y)``), for the cradle page's diagram; each
    bottle's setting (:func:`_setting_row`) and the parts a setting moves (``adjust``: the
    page moves their nodes from the drawn bottle's setting to the one picked)."""
    if not is_cradle(config):
        raise ValueError(f"{config.linkage} is not a decanting cradle")
    lk = config.lk
    ts = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    pts = _world_joints(config, ts)
    o, ax = cr.mouth_frame(lk)
    d = pts[ax] - pts[o]
    tilt = np.degrees(np.unwrap(np.arctan2(d[:, 1], d[:, 0])))
    tilt -= 360.0 * round(float(tilt.mean()) / 360.0)
    rest, full = int(np.argmin(tilt)), int(np.argmax(tilt))
    mouth = cr.mouth_name(lk)
    lip = pts[mouth]
    wander = float(np.linalg.norm(lip - lip.mean(axis=0), axis=1).max())
    ratio = NRV050.ratio
    turns = ((np.arange(n) - rest) % n) / n * ratio
    def outline_of(size: cr.BottleSize) -> list[tuple[float, float]]:
        y0, slope, x0 = cr.seat(size)
        c, s = math.cos(slope), math.sin(slope)
        prof = size.profile()
        return ([(x0 + u * c - r * s, y0 + u * s + r * c) for u, r in prof]
                + [(x0 + u * c + r * s, y0 + u * s - r * c) for u, r in reversed(prof)])

    kind, why = VERDICT.get(lk.key, ("study", ""))

    def r2(a) -> list:
        return np.round(np.asarray(a, dtype=float), 2).tolist()

    return {
        "key": lk.key, "name": lk.name, "notes": lk.notes, "source": lk.source,
        "verdict": kind, "why": why, "labels": dict(lk.labels),
        "frame": list(lk.frame), "crank": list(lk.crank), "mouth": mouth, "axis": ax,
        "output_link": lk.output.link if lk.output else None,
        "links": {b: {"joints": list(js), "edges": [list(e) for e in edges]}
                  for b, (js, edges) in lk.links.items()},
        "points": {k: r2(v) for k, v in pts.items()},
        "tilt_deg": r2(tilt), "handle_turns": r2(turns),
        "rest": rest, "full": full, "n": n,
        "rest_deg": round(float(tilt[rest]), 1), "full_deg": round(float(tilt[full]), 1),
        "turns_to_full": round(float(turns[full]), 1), "ratio": ratio,
        # the toggle zones (handcrank.TOGGLE_TURNS either side of rest and of full tilt): the
        # page's slider shows them (design review round 3's stand and use M4)
        "toggle_turns": TOGGLE_TURNS,
        "mouth_wander_mm": round(wander, 3),
        "bottle": {"key": cr.SHOWN.key, "name": cr.SHOWN.name,
                   "outline": r2(outline_of(cr.SHOWN)), "setting": _setting_row(cr.SHOWN)},
        "bottles": [{"key": z.key, "name": z.name, "outline": r2(outline_of(z)),
                     "setting": _setting_row(z)} for z in cr.SIZES],
        "adjust": {"front": list(cr.FRONT_MOVES), "rear": list(cr.REAR_MOVES),
                   "stop": list(cr.STOP_MOVES)},
        "saddles": [{"name": sd.name, "x": list(sd.vee), "y": [sd.bottom, sd.top],
                     "moves": key} for sd, key in zip(cr.SADDLES, ("front", "rear"),
                                                    strict=True)],
        "ties": {k: list(v) for k, v in _ties_of().items()},
        # how it's used (the guide's own lines, ``construction.cradle.use_steps``): the page's
        # "Using it" (design review round 2's M5: the page had none of the use limits)
        "use": [{"tag": u.tag, "title": u.title, "lines": list(u.lines)}
                for u in cr.use_steps(lk.key)],
    }


def _ties_of() -> dict[str, tuple[float, float]]:
    from spiderpig.linkages.decanters import TIES

    return dict(TIES)


ROOM_BAND = 5.0         # the room's height steps (mm)
_MOVING = ("tie_", "front_saddle", "rear_saddle", "led", "base_stop", "bottle")


def _surface_points(part, step: float = 5.0) -> np.ndarray:
    """Points over ``part``'s surface no more than about ``step`` apart (its tessellation's
    triangles filled in, so a long tie rod's middle counts)."""
    verts, tris = part.tessellate(0.5, 0.5)
    v = np.array([[p.X, p.Y, p.Z] for p in verts], float)
    out = [v]
    for a, b, c in tris:
        pa, pb, pc = v[a], v[b], v[c]
        n = math.ceil(max(np.linalg.norm(pb - pa), np.linalg.norm(pc - pa),
                          np.linalg.norm(pc - pb)) / step)
        if n <= 1:
            continue
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
        keep = i + j <= n
        u, w = i[keep] / n, j[keep] / n
        out.append(pa + np.outer(u, pb - pa) + np.outer(w, pc - pa))
    return np.concatenate(out)


def _radius_at(p, s: float) -> float:
    """``liquid.ts``'s ``radiusAt``: the radius at ``s`` of a profile ``[[s, r], ...]``
    (linear between its points; 0 outside it)."""
    n = len(p)
    if n == 0 or s < p[0][0] or s > p[-1][0]:
        return 0.0
    for i in range(1, n):
        if p[i][0] >= s:
            (s0, r0), (s1, r1) = p[i - 1], p[i]
            return r0 + (r1 - r0) * (s - s0) / (s1 - s0) if s1 > s0 else max(r0, r1)
    return float(p[-1][1])


def fits_room(vessel, riser: float, rows) -> tuple[float, float]:
    """``liquid.ts``'s ``fitsRoom``: a vessel (``[[height, radius], ...]``, base to rim) on a
    riser of ``riser`` mm against a room (:func:`room`'s ``room``): its worst point's margin
    (mm: the room's radius less the vessel's at each height, between the profile's points
    and at the room's corners; over the room's top, the height left) and where (mm over the
    table)."""
    rows = sorted((float(h), float(r)) for h, r in rows)
    top = rows[-1][0]

    def room_at(h: float) -> float:
        if h > top + 1e-6:
            return -math.inf
        if h <= rows[0][0]:
            return rows[0][1]
        return _radius_at(rows, h)

    margin, at = math.inf, 0.0

    def probe(h: float, r: float) -> None:
        nonlocal margin, at
        got = room_at(h + riser)
        m = top - (h + riser) if got == -math.inf else got - r
        if m < margin:
            margin, at = m, h + riser

    for i, (h, r) in enumerate(vessel):
        probe(h, r)
        if i + 1 < len(vessel):
            hn, rn = vessel[i + 1]
            for k in range(1, 4):
                probe(h + (hn - h) * k / 4, r + (rn - r) * k / 4)
    for h, _ in rows:
        hv = h - riser
        if vessel[0][0] <= hv <= vessel[-1][0]:
            probe(hv, _radius_at(vessel, hv))
    return margin, at


def riser_fits(vessel, riser: float, rows, clear=cr.RISER_CLEAR) -> bool:
    """The one rule (``construction.cradle.RISER_CLEAR``; ``liquid.ts``'s ``riserFits``): the
    vessel on ``riser`` keeps ``clear[1]`` mm off the room's side at every height, and raised
    ``clear[0]`` mm more it still fits: its rim that far under the lowest part over it."""
    up, side = clear
    return (fits_room(vessel, riser, rows)[0] >= side
            and fits_room(vessel, riser + up, rows)[0] >= 0.0)


def riser_for(vessel, rows, high: float = 250.0, clear=cr.RISER_CLEAR) -> int | None:
    """The highest riser (whole mm, up to ``high``) the rule (:func:`riser_fits`) lets
    ``vessel`` stand on; None if not even the table."""
    for r in range(int(high), -1, -1):
        if riser_fits(vessel, r, rows, clear):
            return r
    return None


def riser_stacks() -> list[tuple[float, list[str]]]:
    """Every height the printed riser's pieces make (``construction.cradle.riser_pieces``),
    each with its pieces bottom up (``"lifts"`` the two lifts side by side, ``"blocks"`` the
    two blocks, a plate by its thickness), the highest last: any of the lifts, the blocks and
    the plates, in that order, the top piece's boss on top
    (:data:`construction.cradle.RISER_LIP`); none (on the table, in the locator) is 0."""
    lift, hb, lip = cr.RISER_LIFT, cr.RISER_BLOCK[1], cr.RISER_LIP
    plates = sorted(cr.RISER_PLATES, reverse=True)
    out: dict[float, list[str]] = {0.0: []}
    for lifts, blocks in itertools.product((False, True), repeat=2):
        for k in range(len(plates) + 1):
            for chosen in itertools.combinations(plates, k):
                if not (lifts or blocks or chosen):
                    continue
                h = round((lift if lifts else 0.0) + (hb if blocks else 0.0) + sum(chosen)
                          + lip, 2)
                names = ((["lifts"] if lifts else []) + (["blocks"] if blocks else [])
                         + [f"{p:g}" for p in chosen])
                if h not in out or len(names) < len(out[h]):
                    out[h] = names
    return sorted(out.items())


def riser_stack(mm: float | None) -> tuple[float, list[str]] | None:
    """The highest stack (:func:`riser_stacks`) no taller than ``mm``; None for None."""
    if mm is None:
        return None
    return max((s for s in riser_stacks() if s[0] <= mm + 1e-9), key=lambda s: s[0])


def room(config: BuildConfig, mech, n: int = 72) -> dict:
    """The room a vessel has under the mouth (``/api/cradle/{key}/room``, the cradle page's
    fit check): at each height over the table (every :data:`ROOM_BAND` mm up to the mouth),
    the radius round the decanter's axis that no part of the cradle box or the bottle drawn
    (a 6 L methuselah, the V blocks at their lowest) enters over the whole swing (``room``),
    and from the pour on (the cradle past -20°, where an opened full bottle starts to pour:
    ``room_pouring``, reported only: the guide and the cradle page give one riser for the
    whole swing). ``mech``: the cradle fabricated at crank angle 0. The side stacks stand
    outside the bay (``construction.cradle.BAY``), so only the bay's parts count. A thinner
    bottle's V blocks ride higher, away from the decanter at every tilt: the drawn setting
    bounds them. Also ``lowest``: the cradle box's part that comes lowest over the drawn
    decanter's rim (within its rim's radius of the axis) over the swing, how high (mm over
    the table) and at what tilt (design review round 2: the light's printed clip, at rest);
    ``rule``: :data:`construction.cradle.RISER_CLEAR`; ``stacks``: :func:`riser_stacks`;
    ``drawn``: the drawn decanter's riser by the rule and its stack."""
    lip = np.asarray(mech.meta["cradle_lip"], float)
    axis_x = float(lip[0]) + cr.DECANTER_X
    table = float(lip[1]) - cr.LIP_HEIGHT
    ts = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    pts = _world_joints(config, np.concatenate([[0.0], ts]))
    o, ax = cr.mouth_frame(config.lk)
    d = pts[ax] - pts[o]
    tilt = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    moving, fixed, owner = [], [], []
    for b in mech.bodies:
        if b.part is None or b.name.startswith(("L.", "R.")) or b.name.startswith("decanter"):
            continue
        p = _surface_points(b.placed_part())
        if b.name.startswith(_MOVING):
            moving.append(p)
            owner += [b.name] * len(p)
        else:
            fixed.append(p)
    mv = np.concatenate(moving)
    who = np.asarray(owner)
    over = who != "bottle"                  # the cradle box's own parts
    rim = float(cr.DECANTER_PROFILE[-1][1])
    t0 = tilt[0]
    fx = np.concatenate(fixed) if fixed else np.zeros((0, 3))
    bands = np.arange(0.0, cr.LIP_HEIGHT + 1e-6, ROOM_BAND)
    lowest = (math.inf, "", 0.0)

    def radii(points: np.ndarray) -> np.ndarray:
        h = points[:, 1] - table
        r = np.hypot(points[:, 0] - axis_x, points[:, 2])
        k = np.floor(h / ROOM_BAND).astype(int)
        ok = (k >= 0) & (k < len(bands))
        out = np.full(len(bands), np.inf)
        np.minimum.at(out, k[ok], r[ok])
        return out

    whole = radii(fx) if len(fx) else np.full(len(bands), np.inf)
    pouring = whole.copy()
    rel = mv[:, :2] - lip
    for t in tilt[1:]:
        a = t - t0
        ca, sa = math.cos(a), math.sin(a)
        p = np.column_stack([lip[0] + ca * rel[:, 0] - sa * rel[:, 1],
                             lip[1] + sa * rel[:, 0] + ca * rel[:, 1], mv[:, 2]])
        rr = radii(p)
        whole = np.minimum(whole, rr)
        if math.degrees(t) >= -20.0:
            pouring = np.minimum(pouring, rr)
        near = over & (np.hypot(p[:, 0] - axis_x, p[:, 2]) <= rim)
        if near.any():
            k = int(np.argmin(np.where(near, p[:, 1], np.inf)))
            if p[k, 1] - table < lowest[0]:
                lowest = (float(p[k, 1] - table), str(who[k]), math.degrees(t))
    cap = float(cr.BAY / 2)                 # past the bay: the side stacks
    rows = [[float(h), round(float(min(r, cap)), 1)] for h, r in zip(bands, whole, strict=True)]
    drawn = [(float(h), float(r)) for h, r in cr.DECANTER_PROFILE]
    best = riser_for(drawn, rows)
    return {
        "room": rows,
        "room_pouring": [[float(h), round(float(min(r, cap)), 1)]
                         for h, r in zip(bands, pouring, strict=True)],
        "axis_x": axis_x, "mouth_over_table": cr.LIP_HEIGHT,
        "lowest": {"part": lowest[1], "mm": round(lowest[0], 1),
                   "tilt_deg": round(lowest[2], 1), "within_mm": rim},
        "rule": {"rim_under_mm": cr.RISER_CLEAR[0], "side_mm": cr.RISER_CLEAR[1]},
        "stacks": [[h, names] for h, names in riser_stacks()],
        "drawn": {"riser_max_mm": best, "stack": riser_stack(best),
                  "riser_mm": cr.DECANTER_RISER},
    }


def payload(linkage: str, store: str | None = None) -> dict:
    """``/api/cradle/{key}/pack``'s and ``/room``'s bodies: ``cradle_pack.flat_pack`` of the
    cradle fabricated at its pack pose (``cradle_pack.pack_t``) and :func:`room` of it at
    crank angle 0 (the baked clip's first frame; the fabrication cache in ``store`` keeps
    both), a package function for :func:`spiderpig.workers.submit`. (In ``cradle_pack`` until
    2026-10-14: the guide reads the pack's pieces, and must not reach the strength check.)"""
    from spiderpig.cradle_pack import cached_input, gather, pack_input, pack_t
    from spiderpig.store import Store

    cfg = BuildConfig(linkage=linkage, robot=False)
    tmpl = template_for(cfg)
    at = Store.of(store) if store else None
    mech = fabricate(tmpl, cfg, 0.0, store=at)
    inp = cached_input(linkage, at, reuse_plan=False) if at is not None else None
    if inp is None:
        t = pack_t(cfg)
        packed = mech if t == 0.0 else fabricate(tmpl, cfg, t, store=at)
        inp = gather(packed, cfg.lk, t, cfg)
    return {"pack": pack_input(inp), "room": room(cfg, mech)}


if __name__ == "__main__":       # python -m spiderpig.cradle_report stand [linkage]
    import json
    import sys

    args = sys.argv[1:]
    if not args or args[0] != "stand":
        sys.exit("usage: python -m spiderpig.cradle_report stand [decanter_rcm|decanter_hinge]")
    print(json.dumps(stand(*args[1:2]), indent=1))
