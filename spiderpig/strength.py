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

The loads (:func:`design_loads`): the design's own from MuJoCo (:mod:`sim.loads`: a
walking percentile and a jam at the servo's torque limit, per joint and link, cached per
design in the store); ``override`` (``--pin-load WALK,JAM``) puts one walking and one
jam load on every joint; without MuJoCo, or for a design the sim can't model, the
family's conservative measured value (:data:`FALLBACK_PIN_LOADS`, else
:data:`GENERIC_PIN_LOADS`, with a note saying so).

Findings (:func:`findings`): a joint whose jam safety factor is under
:data:`JAM_ERROR` (1.0) is an **error** (it yields, or slips, at a jam the firmware
allows: the audit fails); under :data:`JAM_WARN` (2.0) jammed or :data:`WALK_WARN`
(3.0) walking a **warning**. Each names the joint, its links, the case, the load, the
safety factor and fixes that clear it, checked by recomputing with the change: another
pin or pillar construction, the shortest span the links could have, a lower servo
torque limit.
"""

from __future__ import annotations

import math
import re

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
                 = None, sim: bool | str = True) -> dict:
    """The loads the strength check uses for ``config`` (see the module doc): ``source``
    ``sim`` / ``override`` / ``fallback`` / ``none`` (a mechanism: what it drives sets its
    loads), a ``note`` saying where they came from, the largest walking and jam joint
    loads (``walk_n``, ``jam_n``), the sim's ``joints`` and the crank's torques.
    ``sim="cached"``: the design's simulated loads only if the store has them."""
    from spiderpig import linkage as lk
    from spiderpig.config import torque_limit_nm

    limit = torque_limit_nm(config)
    if override is not None:
        out = uniform_loads(*override, "override", f"--pin-load {override[0]:g},{override[1]:g} "
                            "N on every joint")
    elif lk.get(config.linkage).kind != "walker":
        out = uniform_loads(0.0, 0.0, "none", "a mechanism: its pin loads come from what it "
                            "drives (--pin-load WALK,JAM sets them)")
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


# -- per joint --------------------------------------------------------------------------


def _stem(name: str) -> str:
    return re.sub(r"_leg\d+$", "", name.split(":", 1)[-1])


def joint_loads(name: str, note: dict, loads: dict) -> dict:
    """``name``'s walking and jam load and patterns: its own sim joint (same name stem,
    most links shared), else the uniform ones (``basis`` says which)."""
    if loads.get("source") == "sim" and loads.get("joints"):
        links = {e["link"] for e in note["links"]}
        pillar = name.startswith("pillar:")
        cands = [j for j in loads["joints"] if j["stem"] == _stem(name) and not j["crank"]
                 and bool(j["frame"]) == pillar]
        best = max(cands, key=lambda j: len(links & set(j["links"])), default=None)
        if best is not None and links & set(best["links"]):
            return {"walk_n": best["walk"]["n"], "jam_n": best["jam"]["n"],
                    "walk_patterns": best["walk"]["patterns"] or None,
                    "jam_patterns": best["jam"]["patterns"] or None,
                    "basis": "sim"}
        return {"walk_n": loads["walk_n"], "jam_n": loads["jam_n"], "walk_patterns": None,
                "jam_patterns": None,
                "basis": "sim, the design's largest (no sim joint matched)"}
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
    weakest = min(caps, key=caps.get)
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
    if loads.get("source") != "sim" or not joints:
        return []
    lk = lk_mod.get(config.linkage)
    pts = lk.solve(params=dict(config.proportions) or None).joints_at(0.0)
    feet = dict(lk.feet)
    sheets = link_sheets(config)
    w = 2 * config.params.link_radius
    crank_pins = set(lk.crank[1:])
    bore = rider_hole(config)
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
        an = (w - LINK_HOLE) * t
        zn = t * (w ** 3 - LINK_HOLE ** 3) / (6 * w)
        # the net section (mm^2) at each pin's hole: a crank rider's bore is the crank's
        # (an aluminium rider's end grown to a boss of 1 x t of web round it,
        # plates.rider_bosses), else LINK_HOLE
        boss_w = max(w, bore + 2 * (RIDER_BOSS_T * t + 0.1)) if sh.metal else w
        net = {pin: (boss_w - bore) * t if pin in crank_pins else an for pin in at}
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
    """The other sections a pivot could have (each with the option that builds it): the
    Chicago screw's barrel for a pin, the goBILDA standoff for a pillar (the one-piece
    steel shaft's alternative)."""
    from spiderpig.construction.pivots.chicago import ChicagoShaft, chicago_section
    from spiderpig.construction.pivots.standoff import StandoffAxle

    out = [("--pin chicago", chicago_section(ChicagoShaft())) if kind == "pin" else
           ("--pillar standoff", StandoffAxle().section())]
    return [(k, s) for k, s in out if s.name != note["section"]["name"]]


def materials_sheet(key: str):
    """:func:`spiderpig.materials.sheet` (imported late: the engine's import order)."""
    from spiderpig.materials import sheet

    return sheet(key)


def fixes(row: dict, note: dict | None, loads: dict, config: BuildConfig) -> list[str]:
    """What would bring ``row`` to :data:`JAM_WARN` jammed and :data:`WALK_WARN` walking,
    each recomputed with the change."""
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
        return out or ["a wider link (Params.link_radius) or a stronger sheet"]
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
    if not out:
        out.append("no listed pin or pillar construction clears it at this load: a lower "
                   "servo torque limit, or a linkage scaled up (stiffer links, shorter "
                   "relative spans)")
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
        what = (f"{row['joint']}" + (f" ({row['sheet']}, {row['thickness_mm']:g} mm, "
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


def check(notes: dict, meta: dict, config: BuildConfig, loads: dict) -> dict:
    """Every pivot's and the crank's strength rows, the worst per kind and the findings."""
    rows = [joint_strength(name, note, loads) for name, note in sorted(notes.items())
            if loads.get("walk_n") or loads.get("jam_n") or loads.get("joints")]
    crank = crank_strength(meta, config, loads)
    if crank is not None:
        rows.append(crank)
    rows += link_rows(config, loads)
    worst: dict[str, dict] = {}
    for kind in ("pin", "pillar", "crank", "link"):
        ks = [r for r in rows if r["kind"] == kind]
        for tag in ("walk", "jam"):
            have = [r for r in ks if r.get(tag)]
            if have:
                w = min(have, key=lambda r: r[tag]["safety"])
                worst.setdefault(kind, {})[tag] = {"safety": w[tag]["safety"],
                                                   "joint": w["joint"]}
    return {"loads": {k: v for k, v in loads.items() if k != "joints"},
            "rows": rows, "worst": worst, "findings": findings(rows, notes, loads, config)}
