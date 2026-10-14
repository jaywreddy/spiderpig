"""A decanting cradle's drive limits (:mod:`construction.handcrank`), for
:func:`cradle_report.report`: the hand crank's slip clutch against the worst turn, what the
gearbox and the keyed plates see at the jams, and the jam at the clutch's cap rated over the
whole turn, the crank-rocker's toggles included (design review round 1, 2026-10-14).

* **The clutch** (:func:`drive_limits`): set on the bench to slip at
  ``handcrank.CLUTCH_SLIP_N`` at the grip, within ``handcrank.CLUTCH_SCATTER`` either way
  after; its least slip must clear the worst starting turn (the worm's static efficiency)
  by ``handcrank.CLUTCH_MARGIN``, and its most, through the worm at a hand's speed, is the
  crank torque no push at the grip exceeds (its cap: ``config.PRODUCT_CLUTCH_NM``).
* **The capped jam** (:func:`clutch_case`): the cap on a blocked cradle (held by a couple,
  as the jam case) at every crank angle. Each part's safety factor per angle scales from its
  jam case row (:mod:`spiderpig.strength` rates stresses linear in their loads: a pin's or a
  pillar's by its joint's force, a link's by its pins', the crank's by the torque), the
  rod's buckling and the crankpin's bending from the rod's force. Round each toggle the
  rod's force goes as the cap over the crank's sine to the rod: no torque bounds it there,
  so the case reports, for an excluded band narrowing to nothing, the least safety factor
  outside it, and the band within which each part falls under 1.3 and under 1.
"""

from __future__ import annotations

import numpy as np

JAM_FLOOR = 1.3         # the jam safety factor every part keeps (tests/test_cradle.py)
BANDS = (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0)
"""The excluded bands round each toggle the capped case is rated outside (crank degrees)."""


def drive_limits(box, grip_r: float, peak: float, start_nm: float) -> dict:
    """The drive's rows at the jams: ``box`` the gearbox, ``grip_r`` the grip's radius (mm),
    ``peak`` the bottle's largest crank torque (N·m) and ``start_nm`` the input torque that
    starts it (the static efficiency). The jam's push (``cradle_report.jam_nm``) and the
    clutch's cap against the gearbox's rating; the clutch's slip (input N·m and at the grip)
    against the worst start; the keyed plates' keyway, one plate bearing alone (before the
    four are filed together on the key) and the four."""
    from spiderpig.construction.handcrank import (
        CLUTCH_MARGIN,
        CLUTCH_SLIP_N,
        HAND_CRANK,
        clutch,
    )
    from spiderpig.cradle_report import JAM_HAND_N, jam_nm

    got = clutch(grip_r, box.ratio, box.eta_hand)
    jam = jam_nm(box, grip_r)
    cap = got["crank_cap_nm"]
    one = HAND_CRANK.keyway_one()
    four = HAND_CRANK.capacity()["arm keyway"]
    return {
        "gearbox_jam_x_rating": round(jam / box.rated_nm, 2),
        "clutch": {
            "slip_grip_n": CLUTCH_SLIP_N,
            "slip_input_nm": [got["slip_lo_nm"], got["slip_nm"], got["slip_hi_nm"]],
            "slip_grip_range_n": [round(got["slip_lo_nm"] / grip_r * 1000.0, 1),
                                  round(got["slip_hi_nm"] / grip_r * 1000.0, 1)],
            "start_input_nm": round(start_nm, 3),
            "margin": round(got["slip_lo_nm"] / max(start_nm, 1e-9), 2),
            "margin_floor": CLUTCH_MARGIN,
            "crank_cap_nm": cap,
            "cap_x_peak": round(cap / max(peak, 1e-9), 2),
            "cap_x_jam_push": round(cap / jam, 2),
            "jam_push_n": JAM_HAND_N,
            "gearbox_cap_x_rating": round(cap / box.rated_nm, 2),
            "spring_n": [got["spring_set_n"], got["spring_n"]],      # seated, slipping
        },
        "keyway": {"one_plate_nm": round(one, 1), "four_plates_nm": round(four, 1),
                   "one_plate_sf_walk": round(one / max(peak, 1e-9), 2),
                   "one_plate_sf_cap": round(one / cap, 2),
                   "four_plates_sf_cap": round(four / cap, 2)},
    }


def _offsets(ts, tilt) -> tuple[np.ndarray, np.ndarray]:
    """Each crank angle's distance (degrees) from the rest toggle and from the full one."""
    deg = np.degrees(np.asarray(ts, float))
    rest, full = (np.abs((deg - deg[e] + 180.0) % 360.0 - 180.0)
                  for e in (int(np.argmin(tilt)), int(np.argmax(tilt))))
    return rest, full


def clutch_case(config, ts, tilt, joints, rows: list[dict], strength_rows: list[dict],
                cap_f: np.ndarray, rod: dict, crankpin: dict, jam_nm: float, cap: float
                ) -> dict:
    """The clutch's cap ``cap`` (N·m) on a blocked cradle at every crank angle ``ts``
    (``tilt`` the bottle's, degrees; ``cap_f`` every joint's force there, from
    ``cradle_report.statics``): each part's safety factor per angle, from its jam case row
    (``strength_rows``, at ``rows``' jam forces and ``jam_nm`` on the crank), the rod's
    buckling (``rod``) and the crankpin's bending (``crankpin``). Returns, per excluded band
    round the toggles (:data:`BANDS`), each part's least safety factor outside it and the
    worst; per part the band (crank degrees from each toggle) within which it falls under
    :data:`JAM_FLOOR` and under 1; the band outside which every part holds 1.3, in crank
    degrees, the tilt from each end of the swing and handle turns."""
    lk = config.lk
    rest, full = _offsets(ts, tilt)
    off = np.minimum(rest, full)
    turns_per_deg = 30.0 / 360.0                 # the worm's 30:1
    ref = {}                                      # each joint's jam case force (N)
    per = {}                                      # and its force at the cap per angle
    for jk, (j, _b1, _b2) in enumerate(joints):
        ref[j] = max(ref.get(j, 0.0), float(rows[jk]["jam_n"]))
        per[j] = np.maximum(per.get(j, 0.0), cap_f[:, jk])
    sf: dict[str, np.ndarray] = {}
    for r in strength_rows:
        name, jam_sf = r["joint"], r.get("jam_sf")
        if not jam_sf:
            continue
        stem = name.split(":", 1)[-1]
        if r.get("kind") in ("pin", "pillar") or name.startswith(("pin:", "pillar:")):
            if stem in per and ref.get(stem):
                sf[name] = jam_sf * ref[stem] / np.maximum(per[stem], 1e-9)
        elif name.startswith("link:"):
            pins = list(dict.fromkeys(j for (j, b1, b2) in joints
                                      if stem in (b1, b2) and ref.get(j)))
            if pins:
                worst = np.max([per[j] / ref[j] for j in pins], axis=0)
                sf[name] = jam_sf / np.maximum(worst, 1e-9)
        elif name == "crank":
            sf[name] = np.full(len(ts), jam_sf * jam_nm / cap)
    rod_jk = next(jk for jk, (j, *_) in enumerate(joints) if j == lk.crank[1])
    f_rod = np.maximum(cap_f[:, rod_jk], 1e-9)
    sf["rod (buckling)"] = rod["euler_n"] / f_rod
    sf["crankpin"] = crankpin["capacity_nm"] * 1000.0 / (f_rod * crankpin["lever_mm"])

    def band(v: np.ndarray, floor: float, side: np.ndarray) -> float:
        """How far from a toggle (crank degrees) ``v`` falls under ``floor`` nearest it."""
        near = (side < 90.0) & (v < floor)
        return round(float(side[near].max()), 1) if near.any() else 0.0

    def outside(b: float) -> dict:
        """Each part's least safety factor ``b`` crank degrees and more from the toggles."""
        keep = off >= b - 1e-9
        least = {n: round(float(v[keep].min()), 2) for n, v in sf.items()}
        worst = min(least, key=least.__getitem__)
        return {"band_deg": b, "worst": [least[worst], worst],
                "rod_buckling_sf": least["rod (buckling)"],
                "pillars": {n: v for n, v in least.items() if n.startswith("pillar:")},
                "parts": least, "rod_n": round(float(f_rod[keep].max()), 1)}

    by_band = [outside(b) for b in BANDS]
    parts = {n: {"under_1p3_deg": [band(v, JAM_FLOOR, rest), band(v, JAM_FLOOR, full)],
                 "under_1_deg": [band(v, 1.0, rest), band(v, 1.0, full)]}
             for n, v in sf.items()}
    holds = [max(p["under_1p3_deg"][i] for p in parts.values()) for i in (0, 1)]
    ends = [float(np.min(tilt)), float(np.max(tilt))]
    tilt_in = [round(float(np.max(np.abs(tilt[side <= h] - end))), 2) if h else 0.0
               for side, h, end in ((rest, holds[0], ends[0]), (full, holds[1], ends[1]))]
    return {"crank_nm": cap, "jam_push_nm": jam_nm,
            "basis": "the clutch's most through the worm at a hand's speed, on a blocked cradle "
                     "(a couple on the cradle link) at every crank angle; each part's jam row "
                     "scaled to its load per angle",
            "by_band": by_band, "parts": parts,
            "holds_1p3_outside_deg": holds, "holds_1p3_tilt_deg": tilt_in,
            "holds_1p3_turns": [round(h * turns_per_deg, 2) for h in holds],
            "away": by_band[-1]["worst"]}


def verdict(key: str, rep: dict) -> tuple[str, str]:
    """A cradle's standing and why (``cradle_report.VERDICT``), with this report's own
    numbers: the jam at the clutch's cap away from the toggles, the gearbox's walking
    safety factor, the stand's tip."""
    from spiderpig.cradle_report import VERDICT

    kind, why = VERDICT.get(key, ("study", ""))
    capped = rep.get("jam_capped") or {}
    drive = rep.get("drive") or {}
    stand = rep.get("stand") or {}
    bits = []
    if capped:
        sf, part = capped["away"]
        bits.append(f"at the clutch's {capped['crank_nm']:g} N·m cap every part holds jam SF "
                    f"{sf:g} or more 30° and more from the toggles ({part} least)")
    if drive:
        bits.append(f"the gearbox {drive['gearbox_sf']:g} x its rating walking")
    if stand:
        bits.append(f"the stand tips at {stand['worst_tip_deg']:g}°")
    return kind, (why + (" This report: " + "; ".join(bits) + "." if bits else ""))
