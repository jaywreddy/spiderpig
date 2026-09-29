"""Fabrication audit: does the solved assembly physically work?

Four checks per assembly mode, each independent of the unit tests (which
only prove that connected joints coincide):

1. **solids**  — every placed part is a valid B-rep; parts made of more than
   one disconnected solid are flagged (they cannot be printed/cut as one piece).
2. **clash**   — pairwise OCCT intersection volume between placed parts at a
   few crank angles (bounding-box prefilter). ``--joinery`` also applies the
   bake's ClevisPin pass first.
3. **sweep**   — full-cycle interference between laser-cut links that share
   a Z layer, using the vectorized template path: each link is its stadium
   (segment ⊕ disc of radius ``BUFF``), sampled at ``--samples`` crank angles.
4. **dxf**     — replays ``layout.save_sheets``' packer and reports parts
   that silently fail to fit on a sheet.

Exits non-zero when any check fails.

Usage::

    uv run python scripts/audit_fab.py                       # all 2016 modes
    uv run python scripts/audit_fab.py --modes quad --joinery --json audit.json
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO_ROOT))
sys.path.insert(0, str(_REPO_ROOT / "viewer"))

from bake_gltf import _build_assembly, _build_template  # noqa: E402

from shapes import BUFF, THICKNESS  # noqa: E402

_MODES = ("single", "double", "decker", "quad")
# b4 is cut as the segment E -> F, but F is not one of its joints; recover it
# from the design ratio |DF| / |ED| in klann.create_klann_geometry.
_FOOT_RATIO = 2.577 / 0.93
_LINK_ENDS = {"b1": ("M", "D"), "b2": ("B", "E"), "b3": ("A", "C"), "b4": ("E", "D")}


def _bb_overlap(a, b, eps: float = 1e-6) -> bool:
    lo = np.maximum(a.min.to_tuple(), b.min.to_tuple())
    hi = np.minimum(a.max.to_tuple(), b.max.to_tuple())
    return bool(np.all(hi - lo > eps))


def check_solids_and_clash(mode: str, t: float, *, joinery: bool) -> dict:
    mech = _build_assembly(
        mode, t=t, n_legs=1, thickness=THICKNESS, with_parts=True, with_joinery=joinery
    ).solved()
    placed = {b.name: b.placed_part() for b in mech.bodies if b.part is not None}
    solids = {}
    for name, part in placed.items():
        valid = part.is_valid() if callable(part.is_valid) else part.is_valid
        solids[name] = {"solids": len(part.solids()), "valid": bool(valid)}
    boxes = {n: p.bounding_box() for n, p in placed.items()}
    clashes = []
    for a, b in itertools.combinations(placed, 2):
        if not _bb_overlap(boxes[a], boxes[b]):
            continue
        inter = placed[a] & placed[b]  # build123d returns None for an empty result
        vol = 0.0 if inter is None else inter.volume
        if vol > 1e-3:
            clashes.append({"a": a, "b": b, "mm3": round(vol, 2)})
    return {"solids": solids, "clashes": clashes}


def _segdist(p1, q1, p2, q2, n: int = 41) -> np.ndarray:
    """Min distance between two (T, 2)-sampled segments, per sample."""
    s = np.linspace(0.0, 1.0, n)
    a = p1[:, None] + (q1 - p1)[:, None] * s[None, :, None]
    b = p2[:, None] + (q2 - p2)[:, None] * s[None, :, None]
    return np.linalg.norm(a[:, :, None] - b[:, None], axis=-1).min(axis=(1, 2))


def check_sweep(mode: str, samples: int) -> list[dict]:
    tmpl = _build_template(mode, n_legs=1, thickness=THICKNESS, with_joinery=False)
    ts = np.linspace(0.0, 2.0 * math.pi, samples, endpoint=False)
    sp = tmpl.sample(ts)
    stadiums = []  # (body, p, q, z_lo)
    for body in tmpl.bodies:
        cls = re.sub(r"_leg\d+$", "", body.name)
        jw = sp.joint_world[body.name]
        if cls in _LINK_ENDS:
            pairs = [_LINK_ENDS[cls]]
        elif cls.startswith("conn"):
            legs = sorted(n[1:] for n in jw if n.startswith("M"))
            pairs = [(f"O{s}", f"M{s}") for s in legs]
        else:
            continue
        for j0, j1 in pairs:
            p, q = jw[j0][:, :2], jw[j1][:, :2]
            if cls == "b4":
                q = q + (q - p) * _FOOT_RATIO
            stadiums.append((body.name, p, q, sp.world[body.name][:, 2, 3]))
    hits = []
    for (n1, p1, q1, z1), (n2, p2, q2, z2) in itertools.combinations(stadiums, 2):
        if n1 == n2:
            continue
        same_layer = np.abs(z1 - z2) < THICKNESS - 1e-6
        if not same_layer.any():
            continue
        gap = _segdist(p1, q1, p2, q2) - 2.0 * BUFF
        bad = same_layer & (gap < 0)
        if bad.any():
            hits.append({
                "a": n1, "b": n2, "z": round(float(z1[0]), 2),
                "cycle_pct": round(100.0 * bad.mean(), 1),
                "max_penetration_mm": round(float(-gap[bad].min()), 2),
            })
    return hits


def check_dxf(mode: str) -> list[dict]:
    from rectpack import newPacker

    from layout import _DEFAULT_SHEET, _LAYOUT_SKIP, _MARGIN, _body_profile, _wire_bbox_2d

    mech = _build_assembly(
        mode, t=1.0, n_legs=1, thickness=THICKNESS, with_parts=True, with_joinery=False
    ).solved()
    items = []
    for body in mech.bodies:
        if _LAYOUT_SKIP.match(body.name) or body.part is None:
            continue
        outer, _ = _body_profile(body)
        x0, y0, x1, y1 = _wire_bbox_2d(outer)
        items.append((body.name, x1 - x0 + 2 * _MARGIN, y1 - y0 + 2 * _MARGIN))
    packer = newPacker(rotation=False)
    for rid, (_, w, h) in enumerate(items):
        packer.add_rect(math.ceil(w), math.ceil(h), rid=rid)
    for _ in items:
        packer.add_bin(*_DEFAULT_SHEET)
    packer.pack()
    packed = {r.rid for b in packer for r in b}
    return [
        {"body": n, "w": round(w), "h": round(h)}
        for rid, (n, w, h) in enumerate(items) if rid not in packed
    ]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--modes", default=",".join(_MODES))
    ap.add_argument("--ts", default="0,1,2.5,4", help="crank angles for the OCCT clash check")
    ap.add_argument("--samples", type=int, default=720, help="crank samples for the sweep")
    ap.add_argument("--joinery", action="store_true", help="apply bake ClevisPins first")
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args()

    report: dict = {}
    failed = False
    for mode in args.modes.split(","):
        rep = report[mode] = {"clash": {}, "sweep": [], "dxf_dropped": [], "multi_solid": {}}
        for t in (float(x) for x in args.ts.split(",")):
            r = check_solids_and_clash(mode, t, joinery=args.joinery)
            rep["clash"][f"t={t:g}"] = r["clashes"]
            for name, s in r["solids"].items():
                if s["solids"] > 1 or not s["valid"]:
                    rep["multi_solid"][name] = s
        rep["sweep"] = check_sweep(mode, args.samples)
        rep["dxf_dropped"] = check_dxf(mode)

        print(f"== {mode}")
        for name, s in rep["multi_solid"].items():
            print(f"  [solids] {name}: {s['solids']} disconnected solids, valid={s['valid']}")
        for key, clashes in rep["clash"].items():
            for c in clashes:
                print(f"  [clash {key}] {c['a']} x {c['b']}: {c['mm3']} mm^3")
        for h in rep["sweep"]:
            print(f"  [sweep] {h['a']} x {h['b']} (z={h['z']}): collide {h['cycle_pct']}% "
                  f"of cycle, up to {h['max_penetration_mm']} mm")
        for d in rep["dxf_dropped"]:
            print(f"  [dxf] {d['body']} ({d['w']}x{d['h']} mm) does not fit a sheet — dropped")
        clean = not (rep["multi_solid"] or any(rep["clash"].values())
                     or rep["sweep"] or rep["dxf_dropped"])
        print("  OK" if clean else "  FAIL")
        failed |= not clean

    if args.json:
        args.json.write_text(json.dumps(report, indent=1))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
