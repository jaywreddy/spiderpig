"""Can the services cut it? Every laser-cut part against its sheet's rules.

The plates go to SendCutSend (aluminium) and Ponoko (acrylic), whose cut rules each sheet
item carries (:mod:`hardware.sheet_catalog`, read through :func:`materials.sheet`):

* **holes**: a round hole at least ``min_hole`` across (SendCutSend in metal: the sheet's
  thickness; Ponoko: 1 mm);
* **edge distance**: from a hole to the part's edge or to another hole at least
  :attr:`materials.Sheet.min_edge` (SendCutSend: 2 x the thickness in aluminium; Ponoko:
  its 1 mm minimum feature);
* **part size**: at least ``min_part`` (SendCutSend aluminium 6.35 x 9.5 mm, Ponoko 6 mm);
* **inside corners**: a pocket's corners come out ``corner_r`` round (SendCutSend in
  aluminium: 0.8 mm), so a pocket that must take a square corner (a hex pocket for a nut or
  a bolt head) needs corner reliefs at least that round: a pocket of straight edges only,
  or with relief arcs tighter than that, is reported.

Levels (the user's rule of 2026-10-04, the design-review limits): a hole closer to an edge
or another hole than **1 x the thickness** in metal is an **error** (the audit fails), under
the service's 2 x a **warning**; a hole under SendCutSend's minimum (the thickness) is an
error (they don't cut it); the rest are warnings. :func:`check` lists them part by part,
the worst first, each with its ``level``.
"""

from __future__ import annotations

import math

from build123d import GeomType, Plane, section

from spiderpig.layout import _lay_flat, _wire_is_circle, laser_bodies, sheet_key
from spiderpig.materials import sheet

TOL = 1e-3
EDGE_ERROR_T = 1.0      # hole-to-edge under this many thicknesses in metal: an error


def _sample(wire, n: int = 96) -> list[tuple[float, float]]:
    import numpy as np

    return [(v.X, v.Y) for v in (wire.position_at(u) for u in np.linspace(0, 1, n,
                                                                            endpoint=False))]


def _profile(body):
    part = _lay_flat(body.part)
    bb = part.bounding_box()
    return section(part, Plane.XY.offset((bb.min.Z + bb.max.Z) / 2))


def part_issues(body, key: str) -> list[dict]:
    """The rules ``body`` breaks on sheet ``key``: ``{"rule", "part", "value", "limit",
    "detail"}`` each (empty: none)."""
    sh = sheet(key)
    out: list[dict] = []
    try:
        sketch = _profile(body)
    except Exception:       # noqa: BLE001 - a part that won't section: nothing to check here
        return out
    wires = list(sketch.wires())
    if not wires:
        return out

    def area(w):
        bb = w.bounding_box()
        return (bb.max.X - bb.min.X) * (bb.max.Y - bb.min.Y)

    outer = max(wires, key=area)
    bb = outer.bounding_box()
    w, h = sorted((bb.max.X - bb.min.X, bb.max.Y - bb.min.Y))
    lo, hi = sorted(sh.min_part)
    if w < lo - TOL or h < hi - TOL:
        out.append({"rule": "min_part", "part": body.name, "level": "warning",
                    "value": [round(w, 2), round(h, 2)],
                    "limit": [lo, hi],
                    "detail": f"{w:.1f} x {h:.1f} mm, under {sh.service}'s {lo:g} x {hi:g} mm"})
    holes, pockets = [], []
    for wire in wires:
        if wire is outer:
            continue
        c = _wire_is_circle(wire)
        if c is not None:
            holes.append(c)
        else:
            pockets.append(wire)
    edge_pts = _sample(outer, 192)
    worst_hole = min(holes, key=lambda c: c[1], default=None)
    if worst_hole is not None and 2 * worst_hole[1] < sh.min_hole - TOL:
        out.append({"rule": "min_hole", "part": body.name, "value": round(2 * worst_hole[1], 2),
                    "limit": sh.min_hole, "level": "error" if sh.metal else "warning",
                    "detail": f"a {2 * worst_hole[1]:.2f} mm hole, under {sh.service}'s "
                              f"{sh.min_hole:g} mm in {sh.thickness:g} mm {sh.material}"})
    need = sh.min_edge
    if need > 0 and holes:
        worst = None
        for i, ((cx, cy), r) in enumerate(holes):
            d = min(math.hypot(x - cx, y - cy) for x, y in edge_pts) - r
            for (qx, qy), rq in holes[i + 1:]:
                d = min(d, math.hypot(qx - cx, qy - cy) - r - rq)
            if worst is None or d < worst[0]:
                worst = (d, 2 * r)
        if worst is not None and worst[0] < need - TOL:
            hard = sh.metal and worst[0] < EDGE_ERROR_T * sh.thickness - TOL
            out.append({"rule": "edge", "part": body.name, "value": round(worst[0], 2),
                        "limit": round(need, 2), "level": "error" if hard else "warning",
                        "detail": f"{worst[0]:.2f} mm from a {worst[1]:.1f} mm hole to an edge "
                                  f"or hole, under {sh.service}'s {need:.2f} mm"})
    if sh.metal and sh.corner_r > 0:     # (a laser's kerf in acrylic: sharp enough)
        for wire in pockets:
            edges = wire.edges()
            arcs = [e.radius for e in edges if e.geom_type == GeomType.CIRCLE]
            straight = [e for e in edges if e.geom_type == GeomType.LINE]
            if len(straight) < 3:
                continue                     # a slot or a rounded cut-out, not cornered
            tight = min(arcs, default=0.0)
            if tight < sh.corner_r - TOL:
                out.append({"rule": "corner", "part": body.name, "level": "warning",
                            "value": round(tight, 2),
                            "limit": sh.corner_r,
                            "detail": (f"a pocket of {len(straight)} straight edges with "
                                       + (f"{tight:.2f} mm corner reliefs" if arcs else
                                          "sharp corners")
                                       + f": {sh.service} cuts inside corners {sh.corner_r:g} "
                                       f"mm round in {sh.material}")})
                break
    return out


def check(mech, default: str) -> dict:
    """Every laser-cut part of ``mech`` against its sheet's rules: ``{"parts": n, "by_rule":
    {rule: count}, "issues": [...], "sheets": {key: label}}`` (the issues worst first per
    rule: the least edge, the smallest hole or part)."""
    issues: list[dict] = []
    sheets: dict[str, str] = {}
    bodies = laser_bodies(mech)
    for b in bodies:
        key = sheet_key(b, default)
        sheets.setdefault(key, sheet(key).label)
        for i in part_issues(b, key):
            issues.append(dict(i, sheet=key))
    by_rule: dict[str, int] = {}
    errors: dict[str, int] = {}
    for i in issues:
        by_rule[i["rule"]] = by_rule.get(i["rule"], 0) + 1
        if i.get("level") == "error":
            errors[i["rule"]] = errors.get(i["rule"], 0) + 1
    issues.sort(key=lambda i: (i["rule"], i["value"] if isinstance(i["value"], (int, float))
                               else min(i["value"])))
    return {"parts": len(bodies), "by_rule": by_rule, "errors": errors, "issues": issues,
            "sheets": sheets}
