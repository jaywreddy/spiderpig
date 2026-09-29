"""Fabrication audit: does the built assembly physically work?

Four checks per assembly mode, independent of the unit tests' kinematics:

1. **solids** — every placed part is one valid B-rep solid (a part made of
   disconnected pieces cannot be printed or cut as one).
2. **clash**  — pairwise OCCT intersection volume between placed parts at a
   few crank angles, pins/caps/sleeves included (``--no-joinery`` to omit).
3. **plan**   — :func:`stack.verify_plan` re-checks the stack plan against a
   fresh 1440-sample sweep of the whole crank cycle, object by object.
4. **dxf**    — runs the real sheet layout; a link that fits no sheet fails.

Exits non-zero when any check fails.

Usage::

    uv run python scripts/audit_fab.py                       # all modes
    uv run python scripts/audit_fab.py --modes quad --json audit.json
"""

from __future__ import annotations

import argparse
import itertools
import json
import sys
import tempfile
from pathlib import Path

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO_ROOT))
sys.path.insert(0, str(_REPO_ROOT / "viewer"))

from bake_gltf import _build_assembly, _build_template  # noqa: E402

from fabricate import plan_for  # noqa: E402
from layout import save_sheets  # noqa: E402
from stack import verify_plan  # noqa: E402

_MODES = ("single", "double", "decker", "quad")


def _bb_overlap(a, b, eps: float = 1e-6) -> bool:
    lo = np.maximum([a.min.X, a.min.Y, a.min.Z], [b.min.X, b.min.Y, b.min.Z])
    hi = np.minimum([a.max.X, a.max.Y, a.max.Z], [b.max.X, b.max.Y, b.max.Z])
    return bool(np.all(hi - lo > eps))


def check_solids_and_clash(mode: str, t: float, *, joinery: bool) -> dict:
    mech = _build_assembly(mode, t=t, with_joinery=joinery).solved()
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
    return {"parts": len(placed), "solids": solids, "clashes": clashes}


def check_plan(mode: str) -> tuple[list[str], str]:
    tmpl = _build_template(mode)
    plan = plan_for(tmpl)
    return verify_plan(plan, tmpl), plan.describe()


def check_dxf(mode: str) -> tuple[int, str | None]:
    mech = _build_assembly(mode, t=1.0, with_joinery=False)
    with tempfile.TemporaryDirectory() as d:
        try:
            return len(save_sheets(mech, Path(d) / "sheet")), None
        except ValueError as e:
            return 0, str(e)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--modes", default=",".join(_MODES))
    ap.add_argument("--ts", default="0,1,2.5,4,5.5", help="crank angles for the OCCT clash check")
    ap.add_argument("--no-joinery", action="store_true", help="omit pins, caps and sleeves")
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args()

    report: dict = {}
    failed = False
    for mode in args.modes.split(","):
        rep = report[mode] = {"clash": {}, "multi_solid": {}}
        print(f"== {mode}")
        rep["plan_violations"], layout = check_plan(mode)
        print(layout)
        for t in (float(x) for x in args.ts.split(",")):
            r = check_solids_and_clash(mode, t, joinery=not args.no_joinery)
            rep["parts"] = r["parts"]
            rep["clash"][f"t={t:g}"] = r["clashes"]
            for name, s in r["solids"].items():
                if s["solids"] != 1 or not s["valid"]:
                    rep["multi_solid"][name] = s
        rep["dxf_sheets"], rep["dxf_error"] = check_dxf(mode)

        for name, s in rep["multi_solid"].items():
            print(f"  [solids] {name}: {s['solids']} solids, valid={s['valid']}")
        for key, clashes in rep["clash"].items():
            for c in clashes:
                print(f"  [clash {key}] {c['a']} x {c['b']}: {c['mm3']} mm^3")
        for v in rep["plan_violations"]:
            print(f"  [plan] {v}")
        if rep["dxf_error"]:
            print(f"  [dxf] {rep['dxf_error']}")
        clean = not (rep["multi_solid"] or any(rep["clash"].values())
                     or rep["plan_violations"] or rep["dxf_error"])
        print(f"  {rep['parts']} parts, {rep['dxf_sheets']} DXF sheet(s): "
              f"{'OK' if clean else 'FAIL'}")
        failed |= not clean

    if args.json:
        args.json.write_text(json.dumps(report, indent=1))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
