"""Joint strength: each pivot's and the crank's safety factor at the design's own loads.

What the audit (:mod:`spiderpig.tools.audit`), ``explain`` and ``verify`` report about
whether the joints hold, per joint:

* **pins and pillars**: the shaft's bending, shear and bearing stresses
  (:func:`construction.wobble.stresses`, the bending case from the joint's link layers
  and the plates a pillar is glued into) at the design's walking and jam loads;
* **the crank**: the twist each crankpin joint of the built-up crankshaft carries (the
  drive torque times :func:`sim.run.crank_joint_factor`, chord / crank radius) against
  what holds it, element by element, the weakest deciding
  (:meth:`construction.crank.BoltCrank.capacity`: the hex standoff in its plates' pockets,
  one bearing model, :func:`construction.crank.hex_bearing_nm`, and the standoff's torsion;
  the round standoff's friction clamp).

The loads (:func:`design_loads`): a walker's own from MuJoCo (:mod:`sim.loads`: a
walking percentile and a jam at the servo's torque limit, per joint and link, cached per
design in the store); ``override`` (``--pin-load WALK,JAM``) puts one walking and one
jam load on every joint; without MuJoCo, or for a design the sim can't model, the
family's conservative measured value (:data:`FALLBACK_PIN_LOADS`, else
:data:`GENERIC_PIN_LOADS`, with a note saying so). A linkage with ranged inputs (an arm:
no floor, no crank) has its own from the quasi-static model the precision budget's ISO RP
solves (:func:`arm_loads`, :func:`repeatability.static_loads`, per joint as the sim's):
operating (the ``walk`` column) its weight and the payload over the inputs' box x the
moves' accelerations, jammed a crash (the output blocked, the drives at their holding
torque). A cranked mechanism (no floor: what it drives is unknown) is rated the same way
(:func:`crank_loads`, 2026-10-14; "not rated" before): operating, its weight and a given
payload over a turn x the accelerations; jammed, its output blocked while the crank's servo
pushes at the most it gives, its torque limit (:func:`config.torque_limit_nm`: what
``spiderpig control servo`` writes into it, as a walker's; the stall for a servo with
none), away from the output's dead points (:data:`CRANK_LEVER_FLOOR`), gravity added.

Findings (:func:`findings`): a joint whose jam safety factor is under
:data:`JAM_ERROR` (1.0) is an **error** (it yields, or slips, at a jam the firmware
allows: the audit fails); under :data:`JAM_WARN` (2.0) jammed or :data:`WALK_WARN`
(3.0) walking a **warning**. Each names the joint, its links, the case, the load, the
safety factor and fixes that clear it, checked by recomputing with the change: another
pin or pillar construction, the shortest span the links could have, a lower servo
torque limit.

**The feet across the plane** (:func:`foot_stiffness`, a linkage that walks, 2026-10-14):
a foot takes sideways loads (a turn, a slip, a push) the joints' in-plane loads don't
see. Each foot's compliance across the plane (mm/N) over the gait and the worst link tilt
and splay under a sideways foot load, from the 3D frame model the arm's precision uses
(:func:`linkage.stiffness.across`: each link its sheet's section at its layer in the
side's plan, each pin and pillar its own shaft between the layers): reported, no finding.
"""

from __future__ import annotations

import math
import re

import numpy as np

from spiderpig.config import BuildConfig
from spiderpig.construction.crank.capacity import crank_capacity  # (its home since W5)
from spiderpig.construction.wobble import Section, bending_case, stresses

JAM_ERROR = 1.0
JAM_WARN = 2.0
WALK_WARN = 3.0

FALLBACK_PIN_LOADS: dict[str, tuple[float, float]] = {
    "strider": (6.4, 38.0),
    "klann": (119.0, 155.0),
}
"""Walking and jam pin loads (N) per linkage **family** when a design can't be simulated
(no MuJoCo, or a model that doesn't build): the demo designs' measured peaks (2026-10),
the Klann's the demo Klann quad's, which bounds the variants (``klann_lego`` quad at
0,0,180,180 measures 12 N walking, 80 N jammed in :mod:`sim.loads`)."""
GENERIC_PIN_LOADS = (119.0, 155.0)
"""A family nobody measured: the most loaded family's (the demo Klann's)."""
PER_JOINT = ("sim", "static")
"""The loads' sources that are per joint (``joints``): the sim's, the static model's (an
arm's, :func:`arm_loads`; a cranked mechanism's, :func:`crank_loads`)."""
CRANK_POSES = 24
"""A cranked mechanism's static loads are solved at ``4 x`` this many crank angles a turn
(:func:`linkage.constraints.sample_inputs`)."""
CRANK_LEVER_FLOOR = 0.1
"""The crank angles where the crank's lever on the output is under this share of its
largest are left out of a mechanism's crash (:func:`repeatability.static_loads`'
``lever_floor``): at the output's dead points a blocked output's force has no bound but the
crank's own stop (the analogue of the arm's box kept to its 10-90 %)."""



def bolt_crank(key: str) -> bool:
    """Is crank ``key`` a :class:`construction.crank.BoltCrank` (``bolt``, ``bolt_round``)?"""
    from spiderpig.construction import CRANKS
    from spiderpig.construction.crank import BoltCrank

    return isinstance(CRANKS.get(key), BoltCrank)


def family_loads(linkage: str) -> tuple[float, float] | None:
    """The fallback walking and jam loads of ``linkage``'s family (a key's own entry
    first), else None."""
    if linkage in FALLBACK_PIN_LOADS:
        return FALLBACK_PIN_LOADS[linkage]
    from spiderpig.linkage import engine

    try:
        family = engine.get(linkage).family
    except KeyError:
        return None
    return FALLBACK_PIN_LOADS.get(family)


def uniform_loads(walk: float, jam: float, source: str, note: str) -> dict:
    return {"source": source, "note": note, "walk_n": walk, "jam_n": jam, "joints": []}


def design_loads(config: BuildConfig, store=None, *, override: tuple[float, float] | None
                 = None, sim: bool | str = True, payload_g: float | None = None) -> dict:
    """The loads the strength check uses for ``config`` (see the module doc): ``source``
    ``sim`` / ``override`` / ``static`` (a linkage with no floor: a ranged one's
    :func:`arm_loads`, a cranked mechanism's :func:`crank_loads`) / ``fallback``, a
    ``note`` saying where they came from, the largest walking and jam joint loads
    (``walk_n``, ``jam_n``), the sim's (or the static model's) ``joints`` and the crank's
    torques. ``sim="cached"``: the design's simulated loads only if the store has them.
    ``payload_g``: the load at the output of a linkage with no floor (a ranged one's default
    :data:`repeatability.PAYLOAD_G`; a cranked mechanism's none)."""
    from spiderpig import linkage as lk
    from spiderpig.config import torque_limit_nm

    if override is None and lk.get(config.linkage).ranged:
        return arm_loads(config, payload_g)
    if override is None and not lk.get(config.linkage).walks:
        return crank_loads(config, payload_g)
    limit = torque_limit_nm(config)
    if override is not None:
        out = uniform_loads(*override, "override", f"--pin-load {override[0]:g},{override[1]:g} "
                            "N on every joint")
    else:
        why = "the sim was not asked for (--no-sim)" if not sim else None
        if sim:
            try:
                from spiderpig.sim.loads import STALL_WARN
                from spiderpig.sim.loads import design_loads as simulated

                doc = simulated(config, store, cached_only=sim == "cached")
                if doc is None:
                    raise LookupError("no simulated loads stored for it yet")
                jams = [j["jam"]["n"] for j in doc["joints"] if not j["crank"]]
                walks = [j["walk"]["n"] for j in doc["joints"] if not j["crank"]]
                stalled = doc.get("jam_stalled", 0.0)
                short = ""
                if stalled < STALL_WARN:
                    n = round((1.0 - stalled) * doc["jam_cases"])
                    short = (f"; WARNING: {n} of {doc['jam_cases']} jam cases never reached the "
                             f"torque limit (pinned foot drifted up to "
                             f"{doc.get('jam_foot_drift_mm', 0.0):g} mm), so the jam loads "
                             f"are undersampled")
                return dict(doc, source="sim",
                            note=(f"the design's own, MuJoCo: walking p{doc['walk_percentile']:g} "
                                  f"over {doc['walk_seconds']:g} s, jammed at the "
                                  f"{doc['torque_limit_nm']:g} N·m torque limit "
                                  f"({doc['jam_cases']} cases, a foot pinned, "
                                  f"{stalled:.0%} stalled){short}"),
                            walk_n=max(walks, default=0.0), jam_n=max(jams, default=0.0))
            except ImportError:
                why = "MuJoCo isn't installed"
            except LookupError as e:
                why = str(e)
            except Exception as e:  # noqa: BLE001 - a model that doesn't build: fall back
                why = f"the sim failed ({type(e).__name__}: {e})"
        fam = family_loads(config.linkage)
        walk, jam = fam if fam is not None else GENERIC_PIN_LOADS
        whose = (f"the {lk.get(config.linkage).family} family's measured peaks"
                 if fam is not None else "the most loaded measured family's (the demo Klann)")
        out = uniform_loads(walk, jam, "fallback", f"no sim: {why}; {whose}, conservative")
    out["torque_limit_nm"] = limit
    return out


def drive_capacity(config: BuildConfig) -> dict[str, float]:
    """Each ranged input's drive capacity at its joint (N·m): its motor's holding torque x
    its transmission's ratio (the side's :class:`construction.transmissions.JointDrive`
    groups, as :func:`precision.drive_budgets` reads them)."""
    from spiderpig.construction.transmissions import JointDrive
    from spiderpig.fabricate import design_side, template_for

    side = config.side_config() if config.robot else config
    d = design_side(template_for(side), side)
    return {g.input: g.motor.holding_nm * g.hub(d.ctx).ratio for g in d.groups
            if isinstance(g, JointDrive)}


def link_g_per_mm(config: BuildConfig) -> dict[str, float]:
    """Each link's mass per mm of its length (g): its sheet's density x thickness x
    ``2 link_radius`` (as the precision budget's ISO RP weighs them)."""
    from spiderpig.materials import sheet, sheet_of

    out = {}
    for b in config.lk.links:
        rec = sheet(sheet_of(config, "link", b))
        out[b] = rec.density * rec.thickness * 2 * config.params.link_radius / 1000
    return out


def static_joints(config: BuildConfig, st) -> list[dict]:
    """The static model's loads (:class:`repeatability.StaticLoads`) laid out as the sim's
    joints: per axle its links (the crank's body left out), whether the frame holds it,
    whether it is the crank's (a crank's rows are :func:`crank_strength`'s), ``walk`` its
    most loaded interface operating, ``jam`` its worst interface's gravity + crash."""
    from spiderpig.linkage.constraints import layout

    lay = layout(config.lk)
    crank = set(config.lk.crank)
    joints = []
    for j in lay.axles:
        ifs = [i for i in lay.interfaces if i[0] == j]
        worst = max(ifs, key=lambda i: st.gravity[i] + st.crash[i])
        joints.append({
            "stem": j, "links": sorted(b for _, b in ifs if b not in ("torso", "conn")),
            "frame": any(b == "torso" for _, b in ifs), "crank": j in crank,
            "walk": {"n": round(max(st.operating[i] for i in ifs), 3), "patterns": []},
            "jam": {"n": round(st.gravity[worst] + st.crash[worst], 3), "patterns": [],
                    "gravity_n": round(st.gravity[worst], 3),
                    "crash_n": round(st.crash[worst], 3)}})
    return joints


def arm_loads(config: BuildConfig, payload_g: float | None = None) -> dict:
    """A ranged linkage's loads (source ``static``): the quasi-static model of
    :func:`repeatability.static_loads` over the inputs' box, as the sim's are laid out (per
    joint, ``walk`` and ``jam``). ``walk``: operating, the links' weight, their hardware,
    the tool and ``payload_g`` at rest x :data:`repeatability.ACCEL_FACTOR`; ``jam``: a
    crash, the output blocked while every drive turns at its holding torque x its ratio
    (:func:`drive_capacity`), gravity added. Every joint's load is its most loaded
    interface's. No crank and no servo limit (``torque_limit_nm`` None): the drives' run
    current is the lever."""
    from spiderpig import repeatability as rep

    payload = rep.PAYLOAD_G if payload_g is None else float(payload_g)
    caps = drive_capacity(config)
    st = rep.static_loads(config.lk, config.values, link_g_per_mm=link_g_per_mm(config),
                          payload_g=payload, capacity_nm=caps)
    joints = static_joints(config, st)
    walks = [g["walk"]["n"] for g in joints]
    jams = [g["jam"]["n"] for g in joints]
    cap = ", ".join(f"{k} {v:.2f}" for k, v in caps.items()) or "none"
    note = (f"the static model over the inputs' box: operating, the arm's weight and a "
            f"{payload:g} g payload x {rep.ACCEL_FACTOR:g} for the moves; jammed, a crash: the "
            f"output blocked while the drives push at their holding torque x ratio ({cap} "
            f"N·m at the joints; up to {st.crash_force_n:.1f} N at the output), gravity added")
    if st.singular:
        note += f"; {st.singular} poses left out (the drives can't set the force there)"
    return {"source": "static", "note": note, "walk_n": max(walks, default=0.0),
            "jam_n": max(jams, default=0.0), "joints": joints, "payload_g": payload,
            "capacity_nm": {k: round(v, 4) for k, v in caps.items()},
            "drive_gravity_nm": {k: round(v, 4) for k, v in st.drive_nm.items()},
            "crash_force_n": round(st.crash_force_n, 2), "torque_limit_nm": None}


def crank_loads(config: BuildConfig, payload_g: float | None = None) -> dict:
    """A cranked mechanism's loads (source ``static``, module docstring): the static model
    of :func:`repeatability.static_loads` over a turn of its crank
    (:data:`CRANK_POSES`), laid out as the sim's (:func:`static_joints`). ``walk``:
    operating, its links' weight, their hardware and ``payload_g`` (default none) at the
    output x :data:`repeatability.ACCEL_FACTOR`; ``jam``: the output blocked while the
    crank's servo pushes at its torque limit (:func:`config.torque_limit_nm`, the most the
    servo set up by ``spiderpig control servo`` gives; its stall without one;
    ``torque_limit_nm``: the crank's jam row reads it too), gravity added, the output's dead
    points left out (:data:`CRANK_LEVER_FLOOR`).
    The crank's walking torque is the largest gravity torque x the accelerations."""
    from spiderpig import repeatability as rep
    from spiderpig import servos
    from spiderpig.config import torque_limit_nm
    from spiderpig.linkage.constraints import sample_inputs

    lk = config.lk
    payload = 0.0 if payload_g is None else float(payload_g)
    limit = torque_limit_nm(config)
    torque = limit if limit is not None else servos.get(config.servo).stall_torque_nm or 0.0
    caps = dict.fromkeys(rep.driven_inputs(lk), torque)
    st = rep.static_loads(lk, config.values, link_g_per_mm=link_g_per_mm(config),
                          payload_g=payload, capacity_nm=caps,
                          ins=sample_inputs(lk, CRANK_POSES), tool_g=0.0,
                          lever_floor=CRANK_LEVER_FLOOR)
    joints = static_joints(config, st)
    walks = [g["walk"]["n"] for g in joints if not g["crank"]]
    jams = [g["jam"]["n"] for g in joints if not g["crank"]]
    walk_t = rep.ACCEL_FACTOR * st.drive_nm.get("t", 0.0)
    what = "torque limit" if limit is not None else "stall"
    note = ("the static model over a turn of the crank: operating, the mechanism's weight"
            + (f" and a {payload:g} g payload" if payload else "")
            + f" x {rep.ACCEL_FACTOR:g} for the motion; jammed, its output blocked while the "
            f"{config.servo} pushes at its {torque:g} N·m {what} (up to "
            f"{st.crash_force_n:.1f} N at the output), gravity added")
    if st.toggled:
        note += (f"; {st.toggled} of {4 * CRANK_POSES} crank angles left out near the "
                 f"output's dead points (the crank's lever under "
                 f"{CRANK_LEVER_FLOOR:.0%} of its largest: there the crank's stop bounds it)")
    if st.singular:
        note += f"; {st.singular} crank angles left out (the drive can't set the force there)"
    return {"source": "static", "note": note, "walk_n": max(walks, default=0.0),
            "jam_n": max(jams, default=0.0), "joints": joints, "payload_g": payload,
            "capacity_nm": {k: round(v, 4) for k, v in caps.items()},
            "drive_gravity_nm": {k: round(v, 4) for k, v in st.drive_nm.items()},
            "crash_force_n": round(st.crash_force_n, 2), "torque_limit_nm": torque or None,
            "walk_torque_nm": round(walk_t, 5) or None}


# -- per joint --------------------------------------------------------------------------


def _stem(name: str) -> str:
    return re.sub(r"_leg\d+$", "", name.split(":", 1)[-1])


def joint_loads(name: str, note: dict, loads: dict) -> dict:
    """``name``'s walking and jam load and patterns: its own sim (or static model's) joint
    (same name stem, most links shared), else the uniform ones (``basis`` says which)."""
    src = loads.get("source")
    if src in PER_JOINT and loads.get("joints"):
        links = {e["link"] for e in note["links"]}
        pillar = name.startswith("pillar:")
        cands = [j for j in loads["joints"] if j["stem"] == _stem(name) and not j["crank"]
                 and bool(j["frame"]) == pillar]
        best = max(cands, key=lambda j: len(links & set(j["links"])), default=None)
        if best is not None and links & set(best["links"]):
            return {"walk_n": best["walk"]["n"], "jam_n": best["jam"]["n"],
                    "walk_patterns": best["walk"]["patterns"] or None,
                    "jam_patterns": best["jam"]["patterns"] or None,
                    "basis": src, "crash_n": best["jam"].get("crash_n"),
                    "gravity_n": best["jam"].get("gravity_n")}
        return {"walk_n": loads["walk_n"], "jam_n": loads["jam_n"], "walk_patterns": None,
                "jam_patterns": None,
                "basis": f"{src}, the design's largest (no {src} joint matched)"}
    return {"walk_n": loads.get("walk_n", 0.0), "jam_n": loads.get("jam_n", 0.0),
            "walk_patterns": None, "jam_patterns": None, "basis": loads.get("source", "")}


def joint_strength(name: str, note: dict, loads: dict) -> dict:
    """One pivot's walking and jam stresses (:func:`construction.wobble.stresses`)."""
    jl = joint_loads(name, note, loads)
    row = {"joint": name, "kind": name.split(":", 1)[0],
           "links": [e["link"] for e in note["links"]], "case": bending_case(note),
           "span_mm": note["span_mm"], "section": note["section"]["name"],
           "basis": jl["basis"]}
    for tag in ("walk", "jam"):
        f = jl[f"{tag}_n"]
        s = stresses(note, f, jl[f"{tag}_patterns"]) if f > 0 else None
        row[tag] = dict(s, load_n=round(f, 2)) if s else None
    return row


def crank_strength(meta: dict, config: BuildConfig, loads: dict) -> dict | None:
    """The crank's crankpin joints: the twist at the walking torque and at the torque limit
    (a jam) against what holds them (:func:`crank_capacity`)."""
    caps = crank_capacity(meta, config)
    if not caps:
        return None
    factor = loads.get("joint_moment_factor")
    if factor is None:
        from spiderpig.sim.run import crank_joint_factor

        factor = crank_joint_factor(config)
    limit = loads.get("torque_limit_nm")
    walk_t = loads.get("walk_torque_nm")
    weakest = min(caps, key=caps.__getitem__)
    cap = caps[weakest]
    row = {"joint": "crank", "kind": "crank", "factor": round(factor, 3), "capacity_nm": caps,
           "weakest": weakest, "construction": config.crank}
    for tag, torque in (("walk", walk_t), ("jam", limit)):
        if not torque:
            row[tag] = None
            continue
        moment = factor * torque
        row[tag] = {"torque_nm": round(torque, 4), "moment_nm": round(moment, 4),
                    "safety": round(cap / moment, 2)}
    row["basis"] = ("sim walking torque, the firmware torque limit jammed"
                    if walk_t else "the firmware torque limit jammed (no walking sim)")
    return row


# -- the link plates --------------------------------------------------------------------

LINK_KT = 2.5          # a pin-loaded hole's stress concentration on the net section
LINK_HOLE = 6.35       # a pin hole (a 6 mm standoff's running fit), but a crank rider's


def rider_hole(config: BuildConfig) -> float:
    """The bore of a link riding a crankpin (the crank's rider hole: the hex crankpin's
    8.5 mm sleeve, 8.85 mm), as :func:`construction.plates.rider_bosses` cuts it."""
    from spiderpig.construction import CRANKS

    crank = CRANKS.get(config.crank)
    if crank is None:
        return config.params.hole(config.params.crankpin_d)
    return config.params.hole(crank.for_sheet(config.crank_sheet).rider_d(config.params))


def link_hole(config: BuildConfig, frame: bool) -> float:
    """The hole a link has at a pivot (mm): a bearing construction's seat (the link holds
    the bearing), else :data:`LINK_HOLE` (a pin's running fit). ``frame``: a pillar's."""
    from spiderpig import construction

    c = construction.AXLES.get(config.pillar if frame else config.pin)
    seat = getattr(c, "seat", None)
    return float(seat.seat_d()) if seat is not None else LINK_HOLE


def link_rows(config: BuildConfig, loads: dict) -> list[dict]:
    """Every link plate's stress at the design's own pin loads (``loads["joints"]``, the
    sim's), against its sheet's allowable: the net section at the most loaded hole
    (``LINK_KT`` x the pin load over ``(w - d) t``) and, for a link of three or more pins,
    the plate as a beam between its two farthest pins with the largest pin load between
    them (``F L / 4`` over the gross section's ``t w^2 / 6``; the loads balance, so no pin
    load is a cantilever's); a foot link whose foot is off its pins, the foot's lever to its
    nearest pin. Conservative: the loads' directions aren't used. ``needs`` names the sheet
    that would hold it when the link's own doesn't (jam SF under 2): the user's rule,
    aluminium only where acrylic can't take the load."""
    from spiderpig import linkage as lk_mod
    from spiderpig.construction.plates import RIDER_BOSS_T
    from spiderpig.materials import FOOT_SHEET, link_sheets, sheet

    joints = loads.get("joints") or []
    if loads.get("source") not in PER_JOINT or not joints:
        return []
    lk = lk_mod.get(config.linkage)
    if lk.ranged:       # (no crank angle: a pose inside the box; a link's pins are rigid)
        sol = lk_mod.LegSolution(1, 0.0, config.values, lk.key)
        pts = {k: tuple(float(c) for c in np.asarray(v).reshape(-1, 2)[0])
               for k, v in sol.at(*lk.inputs_at(*[np.array([0.5])] * len(lk.inputs))).items()}
    else:
        pts = lk.solve(params=dict(config.proportions) or None).joints_at(0.0)
    feet = dict(lk.feet)
    sheets = link_sheets(config)
    w = 2 * config.params.link_radius
    crank_pins = set(lk.crank[1:])
    bore = rider_hole(config)
    hole = {False: link_hole(config, False), True: link_hole(config, True)}
    framed = {j["stem"]: bool(j.get("frame")) for j in joints}
    rows = []
    for key in sorted(lk.links):
        at: dict[str, tuple[float, float]] = {}
        for j in joints:
            if any(m == key or m.startswith(key + "_") for m in j["links"]):
                w_, j_ = at.get(j["stem"], (0.0, 0.0))
                at[j["stem"]] = (max(w_, j["walk"]["n"]), max(j_, j["jam"]["n"]))
        if not at:
            continue
        sh = sheet(sheets.get(key, config.sheet))
        t = sh.thickness if sheets.get(key) else config.pitch
        d_max = max(hole[framed.get(pin, False)] for pin in at)
        zn = t * (w ** 3 - d_max ** 3) / (6 * w)
        # the net section (mm^2) at each pin's hole: a crank rider's bore is the crank's
        # (an aluminium rider's end grown to a boss of 1 x t of web round it,
        # plates.rider_bosses), else the pivot's hole (LINK_HOLE, a bearing's seat)
        boss_w = max(w, bore + 2 * (RIDER_BOSS_T * t + 0.1)) if sh.metal else w
        net = {pin: (boss_w - bore) * t if pin in crank_pins
               else (w - hole[framed.get(pin, False)]) * t for pin in at}
        zg = t * w * w / 6
        pins = [n for n in at if n in pts]
        span = max((math.dist(pts[a], pts[b]) for a in pins for b in pins), default=0.0)
        bending = len(pins) >= 3
        foot = feet.get(key)
        foot_lever = (min(math.dist(pts[foot], pts[n]) for n in pins)
                      if foot is not None and foot not in at and foot in pts and pins else 0.0)

        row = {"joint": f"link:{key}", "kind": "link", "links": [key],
               "sheet": sh.key, "thickness_mm": t, "allowable_mpa": sh.yield_mpa,
               "pins": sorted(at), "bending": bending}
        for i, tag in enumerate(("walk", "jam")):
            F = max(f[i] for f in at.values())
            sig = max(LINK_KT * f[i] / net[pin] for pin, f in at.items())
            if bending:
                sig = max(sig, F * span / 4 / zg)
            if foot_lever:
                sig = max(sig, F * foot_lever / zn)
            row[tag] = {"stress_mpa": round(sig, 2), "load_n": round(F, 2),
                        "safety": round(sh.yield_mpa / sig, 2) if sig > 0 else None}
        jam = (row.get("jam") or {}).get("safety")
        row["needs"] = None
        if jam is not None and jam < JAM_WARN and not sh.metal:
            al = sheet(FOOT_SHEET)
            sig = row["jam"]["stress_mpa"] * t / al.thickness
            row["needs"] = {"sheet": al.key, "jam_safety": round(al.yield_mpa / sig, 2)}
        rows.append(row)
    return rows


# -- the centre plates ------------------------------------------------------------------

SHEAR_FRACTION = 0.577     # a ductile metal's shear yield over its tensile (von Mises)


def _bending(strips: list, moment_nmm: float) -> float | None:
    """The in-plane bending stress (MPa) of a section made of ``strips`` ``(y0, y1, t)``
    (mm) under ``moment_nmm`` (N·mm), about the strips' common centroid."""
    area = sum((b - a) * t for a, b, t in strips)
    if area <= 0:
        return None
    yc = sum((a + b) / 2 * (b - a) * t for a, b, t in strips) / area
    inertia = sum(t * (b - a) ** 3 / 12 + (b - a) * t * ((a + b) / 2 - yc) ** 2
                  for a, b, t in strips)
    c = max(max(abs(a - yc), abs(b - yc)) for a, b, t in strips)
    return moment_nmm * c / inertia


def centre_plate_row(meta: dict, config: BuildConfig, loads: dict) -> dict | None:
    """The centre plates under one servo's reaction (the review of 2026-10-08: the 0.063 in
    plates, cut by the bus window and channel): the servo's drive torque (walking; the
    firmware limit, jammed) goes through its two rear screws into its own plates (a couple:
    torque / spacing per screw) and out through the frame ties' studs (bearing: the tie
    farthest from the ties' centroid takes torque x r / sum r^2), which also clamp the stack.
    Each case's stresses, against the sheet's yield (shear at :data:`SHEAR_FRACTION`): the
    screws' bearing and tear-out across their least web (to a cut-out, as cut), the studs'
    bearing, and the own plates' net section in in-plane bending at the bus window and at the
    pad (the whole torque as the moment: conservative). ``None`` without a chassis."""
    g = meta.get("centre_plate_section")
    if not g:
        return None
    from spiderpig.materials import sheet

    sh = sheet(meta["centre_plate_sheet"])
    t_own = g["own"] * g["t"]
    row = {"joint": "centre_plates", "kind": "chassis", "sheet": sh.key,
           "thickness_mm": g["t"], "own_plates": g["own"], "allowable_mpa": sh.yield_mpa,
           "least_web_mm": g["least_web"]}
    limit = loads.get("torque_limit_nm")
    if limit is None:
        from spiderpig.servos import get as servo

        limit = servo(config.servo).torque_limit_nm
    for tag, torque in (("walk", loads.get("walk_torque_nm")), ("jam", limit)):
        if not torque:
            row[tag] = None
            continue
        tq = torque * 1000.0                                          # N·mm
        f = tq / g["spacing"] if g["screws"] > 1 and g["spacing"] > 0 else tq / 10.0
        ties = g["ties"]
        f_tie = tq * max(ties) / sum(r * r for r in ties) if ties else 0.0
        stress = {
            "screw bearing": f / (g["screw_d"] * t_own),
            "screw tear-out": f / (2 * max(g["least_web"], 1e-6) * t_own) / SHEAR_FRACTION,
            "tie bearing": f_tie / (g["tie_d"] * t_own),
        }
        for name, strips in g["sections"].items():
            sig = _bending(strips, tq)
            if sig is not None:
                stress[f"net section at the {name}"] = sig
        worst = max(stress, key=stress.__getitem__)
        row[tag] = {"torque_nm": round(torque, 4), "load_n": round(f, 2),
                    "stress_mpa": {k: round(v, 2) for k, v in stress.items()},
                    "governs": worst, "safety": round(sh.yield_mpa / stress[worst], 2)}
    return row


# -- the feet across the plane ----------------------------------------------------------

FOOT_POSES = 24
"""Crank angles a revolution a foot's stiffness across the plane is sampled at."""


def foot_load(config: BuildConfig, design) -> dict:
    """The sideways load one foot takes (N) and what it comes from: the robot's weight
    (:func:`walk.nominal_mass_breakdown`, within 2 % of the fabricated mass on the default
    robots) shared evenly by the fewest feet that touch the ground over a straight walk
    (:func:`walk.support`, both sides at one crank angle, the feet at their plan's z), times
    the feet's friction on the floor (:attr:`sim.mjcf.SimParams.friction`, the sim's: TPU
    socks on a hard floor). Past that the foot slips, so it is the most a foot takes across
    the plane walking (a turn, a slip, a push against it)."""
    from spiderpig import walk
    from spiderpig.sim.mjcf import SimParams

    legs = walk.side_legs(config, FOOT_POSES)
    model = walk.walker(config, feet_z=walk.foot_z_planned(config, design), legs=legs,
                        n=FOOT_POSES)
    ts = walk.theta_grid(FOOT_POSES)
    feet, _ = model.feet_at(ts, ts)
    contacts = int(walk.support(feet, model.com).contacts.sum(axis=-1).min())
    mu = SimParams().friction
    weight = model.mass_g / 1000 * 9.81
    return {"load_n": round(mu * weight / max(contacts, 1), 3), "mass_g": round(model.mass_g, 1),
            "contacts": contacts, "friction": mu,
            "basis": (f"the robot's {model.mass_g:.0f} g (nominal) on the fewest feet that "
                      f"touch walking ({contacts}), times the floor's friction {mu:g}")}


def _foot_rod(note: dict | None, d: float):
    """A joint's shaft for the stiffness frame: its wobble note's section (the resolved
    pin's or pillar's: :class:`construction.wobble.Section`), else a steel rod ``d``."""
    from spiderpig.linkage.stiffness import Rod

    s = (note or {}).get("section")
    if not s:
        return Rod(d)
    inertia = float(s["z_bend_mm3"]) * float(s["bearing_d"]) / 2
    return Rod.bending(inertia, float(s.get("modulus_mpa", Rod(d).modulus_mpa)))


def foot_stiffness(config: BuildConfig, design, notes: dict | None = None) -> dict | None:
    """Each foot's stiffness across the plane (module docstring), per leg of a side: the
    leg's links at their layers in ``design``'s plan (each on its layer's floor, its own
    sheet's thickness and modulus, ``2 link_radius`` wide), each pin and pillar its own shaft
    (``notes``: the side's wobble notes, their sections; else a 4 mm pin, a 6 mm pillar),
    the pillars held in both frame plates, the crank rigid (its webs and crankpins: the crank
    sheet's plates on steel crankpins) and held by its drive. A foot's compliance (mm/N),
    and at :func:`foot_load` its deflection and the worst link tilt and splay (deg), worst
    over the revolution. ``None`` for a linkage that doesn't walk."""
    lk = config.lk
    if not lk.walks:
        return None
    from spiderpig.linkage.constraints import layout
    from spiderpig.linkage.stiffness import Rod, Section, across, poses
    from spiderpig.materials import sheet, sheet_of

    pl, ctx = design.plan, design.ctx
    n_legs = len(config.legs)
    lay = layout(lk)
    frame = set(lay.joints["torso"])
    crank = set(lk.crank)
    w = 2 * config.params.link_radius
    crank_sheet = sheet(config.crank_sheet)
    load = foot_load(config, design)
    notes = notes or {}

    def note_of(j: str, body: str) -> dict | None:
        """The wobble note of joint ``j`` whose links hold ``body`` (a pillar two legs
        share is one note)."""
        for name, note in notes.items():
            if _stem(name) == j and body in {e["link"] for e in note.get("links", [])}:
                return note
        return None

    feet: list[dict] = []
    for k in range(n_legs):
        name = {b: b if n_legs == 1 else f"{b}_leg{k}" for b in lk.links}
        z, sections = {}, {}
        for b, nb in name.items():
            t = ctx.sheet_t("link", nb)
            z[b] = pl.z(pl.layers[nb])[0] + t / 2         # links sit on their layer's floor
            sections[b] = Section(w, t, sheet(sheet_of(config, "link", nb)).modulus_gpa * 1000)
        pins = crank - {lk.crank[0]}
        riders = [b for b, (js, _) in lk.links.items() if pins & set(js)]
        z["conn"] = sum(z[b] for b in riders) / len(riders) if riders else 0.0
        sections["conn"] = Section(2 * config.params.web_radius, crank_sheet.thickness,
                                   crank_sheet.modulus_gpa * 1000)
        rods = {}
        for j in lay.axles:
            body = next((nb for b, nb in name.items() if j in lk.links[b][0]), "")
            rods[j] = (Rod(config.params.crankpin_d) if j in crank
                       else _foot_rod(note_of(j, body), 6.0 if j in frame else 4.0))
        res = across(lk, config.values, sections[next(iter(lk.links))],
                     ins=poses(lk, FOOT_POSES), z=z, rods=rods,
                     frame_z=(pl.z(0)[1], pl.z(pl.top)[0]), sections=sections)
        for a in res:
            f = load["load_n"]
            feet.append({
                "foot": f"{name[a.body]}.{a.point}", "leg": k,
                "mm_per_n": round(a.mm_per_n, 4), "at_deg": round(math.degrees(a.at), 1),
                "deflection_mm": round(a.mm_per_n * f, 3),
                "tilt_deg": round(a.tilt_deg_per_n * f, 3),
                "tilt_link": name.get(a.tilt_link, a.tilt_link),
                "splay_deg": round(a.splay_deg_per_n * f, 3),
                "splay_joint": f"{a.splay_joint}{'' if n_legs == 1 else f'_leg{k}'}"})
    if not feet:
        return None
    soft = max(feet, key=lambda r: r["mm_per_n"])
    tilt = max(feet, key=lambda r: r["tilt_deg"])
    splay = max(feet, key=lambda r: r["splay_deg"])
    return {**load, "feet": feet,
            "worst": {"mm_per_n": soft["mm_per_n"], "foot": soft["foot"],
                      "deflection_mm": soft["deflection_mm"],
                      "tilt_deg": tilt["tilt_deg"], "tilt_link": tilt["tilt_link"],
                      "splay_deg": splay["splay_deg"], "splay_joint": splay["splay_joint"]},
            "note": ("linkage.stiffness.across: each link a beam of its sheet at its layer, "
                     "the pins and pillars their shafts, the crank rigid and driven, the "
                     "frame plates rigid; the elastic part only (the joints' play, the wobble "
                     "check's tilt, comes on top); worst over the revolution, swing included")}


# -- findings and fixes -----------------------------------------------------------------


def _sf(row: dict, tag: str) -> float | None:
    r = row.get(tag)
    return None if not r else r.get("safety")


def level(row: dict) -> str | None:
    """``error`` / ``warning`` / None for a joint's (or the crank's) row."""
    jam, walk = _sf(row, "jam"), _sf(row, "walk")
    if jam is not None and jam < JAM_ERROR:
        return "error"
    if (jam is not None and jam < JAM_WARN) or (walk is not None and walk < WALK_WARN):
        return "warning"
    return None


def _alt_sections(kind: str, note: dict) -> list[tuple[str, Section]]:
    """The other sections a pivot could have (each with the option that builds it): every
    registered axle construction of that role with a section of its own (the Chicago
    screw's barrel for a pin, the goBILDA standoff for a pillar, ...)."""
    from spiderpig import construction

    out: dict[str, tuple[str, Section]] = {}     # one per section: the first that builds it
    for key, c in construction.AXLES.items():
        if kind in getattr(c, "roles", ()) and hasattr(c, "section"):
            s = c.section()
            out.setdefault(s.name, (f"--{kind} {key}", s))
    return [v for name, v in out.items() if name != note["section"]["name"]]


def materials_sheet(key: str):
    """:func:`spiderpig.materials.sheet` (imported late: the engine's import order)."""
    from spiderpig.materials import sheet

    return sheet(key)


def stepped(config: BuildConfig) -> bool:
    """Are ``config``'s motors steppers, whose holding torque (their run current) sets a
    crash's load (the motor role ``actuator``, :func:`roles.motor_role`)? A crank's servo's
    lever is its firmware torque limit instead: its fixes never name a holding torque
    (2026-10-14; a mechanism's, rated as an arm's crash, got both before)."""
    from spiderpig import roles

    return roles.motor_role(config.lk) == "actuator"


def fixes(row: dict, note: dict | None, loads: dict, config: BuildConfig) -> list[str]:
    """What would bring ``row`` to :data:`JAM_WARN` jammed and :data:`WALK_WARN` walking,
    each recomputed with the change: a crank servo's torque limit, or steppers' holding
    torque (:func:`stepped`), whichever drives the design."""
    out: list[str] = []
    if row["kind"] == "crank":
        f = row["factor"]
        cap = min(row["capacity_nm"].values())
        lim = cap / (JAM_WARN * f)
        out.append(f"set the servo's torque limit to {lim:.2f} N·m or less (jam SF "
                   f"{JAM_WARN:g}; now {row['jam']['torque_nm']:g})" if row.get("jam") else
                   f"keep the servo's torque limit under {lim:.2f} N·m")
        if bolt_crank(row["construction"]) and "pocket" in row["weakest"] and "hex" in row[
                "weakest"]:
            now = materials_sheet(config.crank_sheet)
            thick = materials_sheet("al6061_3p2mm")
            if thick.thickness > now.thickness + 1e-9:
                out.append(f"a thicker crank sheet (--crank-sheet al6061_3p2mm: "
                           f"{thick.thickness:g} mm of hex pocket against "
                           f"{now.thickness:g} mm now)")
        if bolt_crank(row["construction"]) and "clamped" in row["weakest"]:
            out.append("medium threadlocker on the crankpin screws and the screws tightened to "
                       "2 N·m (the friction clamp is the joint): measure the slip torque on "
                       "the test build")
        return out
    if row["kind"] == "chassis":
        return ["a thicker centre sheet, or more own plates per servo (more rear-screw "
                "grip), or a lower servo torque limit"]
    if row["kind"] == "link":
        need = row.get("needs")
        if need:
            out.append(f"cut {row['links'][0]} from {need['sheet']} (--link-sheet "
                       f"{row['links'][0]}={need['sheet']}): jam SF {need['jam_safety']:g}")
        limit = loads.get("torque_limit_nm")
        jam_sf = _sf(row, "jam")
        if jam_sf is not None and jam_sf < JAM_WARN and limit:
            out.append(f"a servo torque limit of {limit * jam_sf / JAM_WARN:.2f} N·m (now "
                       f"{limit:g}) for jam SF {JAM_WARN:g}")
        if jam_sf is not None and jam_sf < JAM_WARN and stepped(config):
            out.append("lower the drives' holding torque (their run current): a crash's load "
                       "scales with it")
        return out or ["a wider link (Params.link_radius) or a stronger sheet"]
    assert note is not None  # a pin's or pillar's row is made from its note (joint_strength)
    jl = joint_loads(row["joint"], note, loads)

    def sfs(n: dict) -> tuple[float | None, float | None]:
        r = []
        for tag in ("jam", "walk"):
            f = jl[f"{tag}_n"]
            r.append(stresses(n, f, jl[f"{tag}_patterns"])["safety"] if f > 0 else None)
        return r[0], r[1]

    def good(j, w) -> bool:
        return (j is None or j >= JAM_WARN) and (w is None or w >= WALK_WARN)

    def fmt(j, w) -> str:
        return f"jam SF {j:g}" + (f", walking {w:g}" if w is not None else "")

    kind = row["kind"]
    for opt, sec in _alt_sections(kind, note):
        j, w = sfs(dict(note, section=sec.as_dict()))
        if j is not None and j > (_sf(row, "jam") or 0):
            out.append(f"{opt} ({sec.name}): {fmt(j, w)}"
                       + ("" if good(j, w) else ", still short"))
    layers = note.get("layers") or {}
    if kind == "pin" and len(set(layers.values())) > 1:
        order = sorted(set(layers.values()))
        tight = {m: order.index(k) for m, k in layers.items()}
        if order[-1] - order[0] > len(order) - 1:
            n = dict(note, layers=tight, span_mm=(len(order) - 1) * note.get("pitch_mm", 3.0),
                     layer_z={})       # adjacent layers, no gaps between them: every layer t
            j, w = sfs(n)
            out.append(f"a shorter span ({note['span_mm']:g} -> {n['span_mm']:g} mm, the links in "
                       f"adjacent layers: a plan constraint): {fmt(j, w)}")
    if kind == "pillar" and len(note.get("anchors") or []) == 1:
        # held by both frame plates: the outer (layer 0) and the inner (the plan's top; an
        # older note without it, past its highest link)
        top = note.get("top")
        if top is None:
            top = max(layers.values(), default=0) + 1
        n = dict(note, anchors=[0, top])
        j, w = sfs(n)
        out.append(f"anchor the pillar in both frame plates (a beam, not a cantilever): "
                   f"{fmt(j, w)}")
    jam_sf = _sf(row, "jam")
    limit = loads.get("torque_limit_nm")
    if jam_sf is not None and jam_sf < JAM_WARN and limit:
        out.append(f"a servo torque limit of {limit * jam_sf / JAM_WARN:.2f} N·m (now "
                   f"{limit:g}: the jam loads scale with it) for jam SF {JAM_WARN:g}")
    if jam_sf is not None and jam_sf < JAM_WARN and jl.get("crash_n") and stepped(config):
        # a crash's load scales with the drives' holding torque (their run current); the
        # payload's and the arm's weight stay: the share that leaves jam SF JAM_WARN
        need = jl["jam_n"] * jam_sf / JAM_WARN
        share = (need - (jl.get("gravity_n") or 0.0)) / jl["crash_n"]
        if 0 < share < 1:
            out.append(f"the drives' holding torque cut to {share:.0%} (their run current: a "
                       f"crash's load scales with it) for jam SF {JAM_WARN:g}")
    if not out:
        lever = ("the drives' holding torque lowered" if stepped(config)
                 else "a lower servo torque limit")
        out.append(f"no listed pin or pillar construction clears it at this load: {lever}, "
                   "or a linkage scaled up (stiffer links, shorter relative spans)")
    return out


def findings(rows: list[dict], notes: dict, loads: dict, config: BuildConfig) -> list[dict]:
    """Every joint under a limit (see the module doc), worst first, with its fixes."""
    out = []
    for row in rows:
        lv = level(row)
        if lv is None:
            continue
        note = notes.get(row["joint"])
        jam, walk = row.get("jam"), row.get("walk")
        what = (f"{row['joint']}" + (f" ({row['sheet']}, {row['own_plates']} x "
                                     f"{row['thickness_mm']:g} mm own plates; "
                                     f"{(jam or walk or {}).get('governs')})"
                                     if row["kind"] == "chassis" else
                                     f" ({row['sheet']}, {row['thickness_mm']:g} mm, "
                                     f"{row['allowable_mpa']:g} MPa allowable; pins "
                                     f"{', '.join(row['pins'])})"
                                     if row["kind"] == "link" else
                                     f" ({', '.join(row['links'])}; {row['case']}, "
                                     f"{row['span_mm']:g} mm span, {row['section']})"
                                     if row["kind"] != "crank" else
                                     f" ({row['construction']}, the {row['weakest']} holds "
                                     f"{min(row['capacity_nm'].values()):g} N·m; "
                                     f"{row['factor']:g} x the drive torque)"))
        parts = []
        if jam:
            parts.append(f"jam SF {jam['safety']:g} at " + (
                f"{jam['load_n']:g} N" if row["kind"] != "crank"
                else f"{jam['torque_nm']:g} N·m"))
        if walk:
            parts.append(f"walking SF {walk['safety']:g} at " + (
                f"{walk['load_n']:g} N" if row["kind"] != "crank"
                else f"{walk['torque_nm']:g} N·m"))
        fx = fixes(row, note, loads, config)
        out.append({"level": lv, "kind": row["kind"], "joint": row["joint"],
                    "sf_jam": _sf(row, "jam"), "sf_walk": _sf(row, "walk"),
                    "load_jam": (jam or {}).get("load_n", (jam or {}).get("torque_nm")),
                    "load_walk": (walk or {}).get("load_n", (walk or {}).get("torque_nm")),
                    "message": f"{what}: {'; '.join(parts)}", "fixes": fx})
    out.sort(key=lambda f: (f["level"] != "error", f["sf_jam"] if f["sf_jam"] is not None
                            else math.inf))
    return out


def check(notes: dict, meta: dict, config: BuildConfig, loads: dict, design=None) -> dict:
    """Every pivot's and the crank's strength rows, the worst per kind and the findings;
    with the side's ``design`` (its plan), a walker's feet across the plane
    (:func:`foot_stiffness`, ``feet``)."""
    rows = [joint_strength(name, note, loads) for name, note in sorted(notes.items())
            if loads.get("walk_n") or loads.get("jam_n") or loads.get("joints")]
    crank = crank_strength(meta, config, loads)
    if crank is not None:
        rows.append(crank)
    rows += link_rows(config, loads)
    plates = centre_plate_row(meta, config, loads)
    if plates is not None:
        rows.append(plates)
    worst: dict[str, dict] = {}
    for kind in ("pin", "pillar", "crank", "link", "chassis"):
        ks = [r for r in rows if r["kind"] == kind]
        for tag in ("walk", "jam"):
            have = [r for r in ks if r.get(tag)]
            if have:
                w = min(have, key=lambda r: r[tag]["safety"])
                worst.setdefault(kind, {})[tag] = {"safety": w[tag]["safety"],
                                                   "joint": w["joint"]}
    out = {"loads": {k: v for k, v in loads.items() if k != "joints"},
           "rows": rows, "worst": worst, "findings": findings(rows, notes, loads, config)}
    if design is not None and config.lk.walks:
        out["feet"] = foot_stiffness(config, design, notes)
    return out
