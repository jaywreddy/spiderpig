"""The identity gate: did a change move any part, plan, BOM row or cut file?

A script, not a test. Before a product change, on the base::

    mise run gate -- snapshot DIR          # (uv run python tests/gate/identity_gate.py ...)

after it, on the branch::

    mise run gate -- compare DIR           # snapshots the tree again and diffs against DIR
    mise run gate -- diff DIR_A DIR_B      # two snapshots

For every design of :data:`DESIGNS` (one process each, ``-j`` at once) it records, as
``spiderpig audit --no-sim`` and ``spiderpig build`` make them:

- the **plan**: layers, top, crank route, heads, gaps, thicknesses, sunk heads, height,
  optimality and proof, ``describe()``;
- the **audit** (:func:`spiderpig.tools.audit.audit_module`, ``sim=False``: the family's
  pin loads) minus its ``seconds``: contract, clashes, solids, wobble, strength, cut
  rules, sheets, BOM summary;
- every **part** of the fabrications at the audit's clash angles (``t=1``, ``t=4.38``):
  class, fab, BOM key, sheet, colour, pose, volume, area, centre of mass, bounding box,
  solid / face / edge counts, and the body order;
- the **build** (``spiderpig build`` of the audit's own ``t=1`` fabrication): ``bom.json``
  and every text it writes (``bom.csv/md``, ``ORDER.md``, the ``parts.csv`` /
  ``order.csv`` files, ``manifest.json``), every DXF (sheets and per-part) as its entities,
  and a hash of every STL.

``compare`` reports, per design, **identical**, **geometry identical, order differs** (a
DXF's entities or a closed outline's start vertex in another order, body order, a text
file's lines reordered, a part's faces/edges counted otherwise at the same volume, area
and box, an STL mesh with the B-rep unchanged) or **DIFFERENT** (anything else, listed).
Exit status 0 / 1 / 2 accordingly. "Order differs" is a user decision (PLAN 4.3), not a
merge.

Reproducible: servos drawn parametrically (no CAD downloads, as the tests), a fresh
store, ``PYTHONHASHSEED=0`` and the planner held by its node budgets alone
(``SPIDERPIG_PLAN_SECONDS`` 3600: the Strider quad's search ends on its 60000-node budget
at ~54 CPU-s, too near the 60 s default for a loaded machine). Numbers compare within
1e-9 relative (1e-6 mm absolute for coordinates).
"""

from __future__ import annotations

import argparse
import concurrent.futures as cf
import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from collections import Counter
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

DESIGNS: dict[str, list[str]] = {
    # the user's three order designs
    "strider_double": ["--linkage", "strider", "--module", "double"],   # BuildConfig()
    "strider_quad": ["--linkage", "strider", "--module", "quad"],
    "klann_lego_quad": ["--linkage", "klann_lego", "--module", "quad"],
    # the demo Klann and the two mechanisms
    "klann_quad": ["--linkage", "klann", "--module", "quad"],
    "hoecken_pantograph": ["--linkage", "hoecken_pantograph"],
    "dwell_rocker": ["--linkage", "dwell_rocker"],
}
"""Each design's ``spiderpig build`` options (the linkage's module and kind defaults
otherwise, as the command line has them)."""

TS_CONTRACT = (0.0, 1.6, 3.2, 4.8)      # the audit's defaults
TS_CLASH = (1.0, 4.38)
REL, ABS = 1e-9, 1e-6

ENV = {
    "SPIDERPIG_OFFLINE": "1",
    "SPIDERPIG_PLAN_SECONDS": "3600",
    "SPIDERPIG_WORKERS": "0",
    "PYTHONHASHSEED": "0",
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
}


# ---------------------------------------------------------------------------
# one design (in its own process)
# ---------------------------------------------------------------------------


def _part_doc(b) -> dict:
    p = b.part
    bb = p.bounding_box()
    com = p.center()
    return {
        "name": b.name, "class": type(p).__name__, "fab": b.fab, "bom_key": b.bom_key,
        "sheet": b.sheet, "color": b.color, "rigid_with": b.rigid_with,
        "pose": [list(map(float, row)) for row in b.pose.matrix],
        "volume": p.volume, "area": p.area, "com": [com.X, com.Y, com.Z],
        "bbox": [bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z],
        "topology": [len(p.solids()), len(p.faces()), len(p.edges()), len(p.vertices())],
    }


def _plan_doc(plan) -> dict:
    route = plan.choices.get("crank")
    return {
        "layers": dict(sorted(plan.layers.items())), "top": plan.top,
        "route": None if route is None else {
            "runs": [[r.at, r.lo, r.hi] for r in route.runs], "bearing": route.bearing},
        "other_choices": sorted(k for k in plan.choices if k != "crank"),
        "heads": plan.heads, "gaps": {str(k): v for k, v in sorted(plan.gaps.items())},
        "thick": {str(k): v for k, v in sorted(plan.thick.items())},
        "sunk": sorted(repr(s) for s in plan.sunk), "height": plan.height,
        "optimal": plan.optimal, "proof": plan.proof, "cost": plan.cost,
        "describe": plan.describe(),
    }


def _r(x: float) -> float:
    return round(float(x), 6) + 0.0      # (no -0.0)


def _dxf_entities(path: Path) -> list:
    """Every entity of a DXF's model space as ``[type, layer, geometry...]`` (mm, 1e-6)."""
    import ezdxf

    out = []
    for e in ezdxf.readfile(str(path)).modelspace():
        t, layer = e.dxftype(), e.dxf.get("layer", "0")
        if t == "LWPOLYLINE":
            pts = [[_r(x), _r(y), _r(bulge)] for x, y, bulge in e.get_points("xyb")]
            out.append([t, layer, bool(e.closed), pts])
        elif t == "LINE":
            out.append([t, layer, [_r(v) for v in (*e.dxf.start, *e.dxf.end)]])
        elif t == "ARC":
            out.append([t, layer, [_r(v) for v in (*e.dxf.center, e.dxf.radius,
                                                    e.dxf.start_angle, e.dxf.end_angle)]])
        elif t == "CIRCLE":
            out.append([t, layer, [_r(v) for v in (*e.dxf.center, e.dxf.radius)]])
        else:
            attrs = {k: ([_r(c) for c in v] if hasattr(v, "__iter__") and not isinstance(v, str)
                         else _r(v) if isinstance(v, float) else v)
                     for k, v in sorted(e.dxfattribs().items()) if k not in ("handle", "owner")}
            out.append([t, layer, json.loads(json.dumps(attrs, default=str))])
    return out


def run_one(name: str, out: Path) -> None:
    """Snapshot one design into ``out/<name>.json``."""
    from OCP.OSD import OSD_ThreadPool

    OSD_ThreadPool.DefaultPool_s(2)
    import spiderpig.build as build_mod
    import spiderpig.tools.audit as audit_mod
    from spiderpig.design import engine_version
    from spiderpig.fabricate import design_side, template_for
    from spiderpig.store import Store

    t0 = time.time()
    work = Path(tempfile.mkdtemp(prefix=f"gate-{name}-"))
    store = Store.of(work / "store")
    argv = DESIGNS[name] + ["--store", str(work / "store"), "--out", str(work / "build")]
    config = build_mod._parse_args(argv).config
    captured: dict[float, object] = {}
    fabricate = audit_mod.fabricate

    def capture(tmpl, cfg, t=1.0):
        mech = fabricate(tmpl, cfg, t)
        captured.setdefault(float(t), mech)
        return mech

    audit_mod.fabricate = capture
    rep = audit_mod.audit_module(config.module, config, TS_CONTRACT, TS_CLASH, store, sim=False)
    rep.pop("seconds", None)
    plan = design_side(template_for(config), config).plan
    parts = {f"t={t:g}": [_part_doc(b) for b in m.bodies if b.part is not None]
             for t, m in sorted(captured.items())}
    # the build of the audit's own t=1 fabrication (the same call `spiderpig build` makes)
    build_mod.fabricate = lambda tmpl, cfg, t=1.0: (captured[1.0] if float(t) == 1.0
                                                    else fabricate(tmpl, cfg, t))
    rc = build_mod.main(argv)
    if rc != 0:
        raise SystemExit(f"{name}: spiderpig build exited {rc}")
    bdir = work / "build"
    files, dxf, mesh = {}, {}, {}
    for f in sorted(bdir.rglob("*")):
        if not f.is_file():
            continue
        rel = str(f.relative_to(bdir))
        if f.suffix == ".dxf":
            dxf[rel] = _dxf_entities(f)
        elif f.suffix == ".stl":
            mesh[rel] = hashlib.sha256(f.read_bytes()).hexdigest()
        elif f.suffix in (".step", ".stp"):
            continue        # (a timestamp in its header; the parts above are its geometry)
        else:
            files[rel] = f.read_text().replace(str(work), "<WORK>")
    doc = {
        "design": name, "argv": DESIGNS[name], "engine_version": engine_version(),
        "config": repr(config), "plan": _plan_doc(plan), "audit": rep, "parts": parts,
        "bom": json.loads(files.pop("bom.json")), "files": files, "dxf": dxf, "mesh": mesh,
        "seconds": round(time.time() - t0, 1),
    }
    (out / f"{name}.json").write_text(json.dumps(doc, indent=1, default=str))


# ---------------------------------------------------------------------------
# snapshots
# ---------------------------------------------------------------------------


def _git(*args: str) -> str:
    try:
        return subprocess.run(["git", *args], cwd=REPO, capture_output=True, text=True,
                              check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def snapshot(out: Path, names: list[str], jobs: int) -> int:
    out.mkdir(parents=True, exist_ok=True)
    cad = tempfile.mkdtemp(prefix="gate-cad-")
    env = {**os.environ, **ENV, "SPIDERPIG_CAD_CACHE": cad}
    t0 = time.time()

    def one(name: str) -> tuple[str, int, float]:
        s = time.time()
        with open(out / f"{name}.log", "w") as log:
            rc = subprocess.run([sys.executable, __file__, "_one", name, str(out)], env=env,
                                cwd=REPO, stdout=log, stderr=subprocess.STDOUT).returncode
        return name, rc, time.time() - s

    failed = []
    with cf.ThreadPoolExecutor(jobs) as ex:
        for name, rc, s in ex.map(one, names):
            print(f"  {name}: {'ok' if rc == 0 else f'FAILED ({rc}), see {out / name}.log'} "
                  f"({s:.0f} s)", flush=True)
            if rc:
                failed.append(name)
    status = _git("status", "--porcelain", "--", "spiderpig")
    (out / "snapshot.json").write_text(json.dumps({
        "commit": _git("rev-parse", "HEAD"), "branch": _git("rev-parse", "--abbrev-ref", "HEAD"),
        "spiderpig_dirty": bool(status), "designs": names, "failed": failed,
        "written_at": time.strftime("%Y-%m-%dT%H:%M:%S"), "seconds": round(time.time() - t0),
    }, indent=1))
    print(f"snapshot of {len(names) - len(failed)}/{len(names)} designs in {out} "
          f"({time.time() - t0:.0f} s)")
    return 1 if failed else 0


# ---------------------------------------------------------------------------
# comparing
# ---------------------------------------------------------------------------


def _close(a, b) -> bool:
    if isinstance(a, bool) or isinstance(b, bool):
        return a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if math.isnan(a) or math.isnan(b):
            return math.isnan(a) and math.isnan(b)
        return abs(a - b) <= max(ABS, REL * max(abs(a), abs(b)))
    return a == b


def deep_diff(a, b, path: str = "") -> list[str]:
    """Where two JSON values differ (numbers within :data:`REL` / :data:`ABS`)."""
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) | set(b), key=str):
            if k not in a:
                out.append(f"{path}/{k}: added {_short(b[k])}")
            elif k not in b:
                out.append(f"{path}/{k}: removed {_short(a[k])}")
            else:
                out += deep_diff(a[k], b[k], f"{path}/{k}")
        return out
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return [f"{path}: {len(a)} -> {len(b)} items"]
        return [d for i, (x, y) in enumerate(zip(a, b, strict=True))
                for d in deep_diff(x, y, f"{path}[{i}]")]
    return [] if _close(a, b) else [f"{path}: {_short(a)} -> {_short(b)}"]


def _short(v) -> str:
    s = json.dumps(v, default=str)
    return s if len(s) <= 100 else s[:97] + "..."


def _canonical(entity: list) -> list:
    """An entity as geometry, whatever its place in the file: a closed polyline from its
    least vertex, in whichever direction gives the lesser sequence."""
    if entity[0] != "LWPOLYLINE" or not entity[2]:
        return entity
    pts = entity[3]
    n = len(pts)
    rev = [[*pts[(i + 1) % n][:2], -pts[i][2] + 0.0] for i in reversed(range(n))]
    best = None
    for seq in (pts, rev):
        for k in range(n):
            cand = seq[k:] + seq[:k]
            if best is None or cand < best:
                best = cand
    return [entity[0], entity[1], True, best]


def _key(entity: list) -> str:
    return json.dumps(entity)


def _dxf_diff(a: list, b: list, path: str) -> tuple[list[str], list[str]]:
    """(geometry differences, order differences) of one DXF."""
    if a == b:
        return [], []
    ca = sorted((_canonical(e) for e in a), key=_key)
    cb = sorted((_canonical(e) for e in b), key=_key)
    geo = deep_diff(ca, cb, path) if len(ca) == len(cb) else [
        f"{path}: {len(ca)} -> {len(cb)} entities"]
    if geo:
        ka, kb = Counter(map(_key, ca)), Counter(map(_key, cb))
        gone, new = list((ka - kb).elements()), list((kb - ka).elements())
        if gone or new:
            geo = [f"{path}: {len(gone)} entities gone, {len(new)} new, e.g. "
                   f"{_short(json.loads((gone or new)[0]))}"]
        return geo, []
    moved = sum(1 for x, y in zip(a, b, strict=True) if x != y)
    return [], [f"{path}: same {len(a)} entities, {moved} in another order or from "
                "another start vertex"]


def _text_diff(a: str, b: str, path: str) -> tuple[list[str], list[str]]:
    if a == b:
        return [], []
    if sorted(a.splitlines()) == sorted(b.splitlines()):
        return [], [f"{path}: the same lines in another order"]
    la, lb = a.splitlines(), b.splitlines()
    first = next((i for i, (x, y) in enumerate(zip(la, lb, strict=False)) if x != y),
                 min(len(la), len(lb)))
    x = la[first] if first < len(la) else "<end>"
    y = lb[first] if first < len(lb) else "<end>"
    return [f"{path}: line {first + 1}: {x[:90]!r} -> {y[:90]!r}"], []


def _parts_diff(a: dict, b: dict) -> tuple[list[str], list[str]]:
    geo, order = [], []
    for t in sorted(set(a) | set(b)):
        pa, pb = a.get(t, []), b.get(t, [])
        na, nb = [p["name"] for p in pa], [p["name"] for p in pb]
        if Counter(na) != Counter(nb):
            gone, new = sorted(set(na) - set(nb)), sorted(set(nb) - set(na))
            geo.append(f"parts {t}: {len(na)} -> {len(nb)} parts; gone {gone[:6]}, "
                       f"new {new[:6]}")
            continue
        if na != nb:
            order.append(f"parts {t}: the same {len(na)} bodies in another order")
        ib = {p["name"]: p for p in pb}
        for p in pa:
            q = ib[p["name"]]
            topo = deep_diff(p["topology"], q["topology"], f"parts {t} {p['name']} topology")
            rest = deep_diff({k: v for k, v in p.items() if k != "topology"},
                             {k: v for k, v in q.items() if k != "topology"},
                             f"parts {t} {p['name']}")
            geo += rest
            if topo:
                (geo if rest else order).extend(
                    d + ("" if rest else " (volume, area, box unchanged)") for d in topo)
    return geo, order


def compare_docs(a: dict, b: dict) -> tuple[list[str], list[str]]:
    """(differences, order-only differences) between two designs' snapshots."""
    diff, order = [], []
    diff += deep_diff(a["plan"], b["plan"], "plan")
    diff += deep_diff(a["audit"], b["audit"], "audit")
    g, o = _parts_diff(a["parts"], b["parts"])
    diff += g
    order += o
    diff += deep_diff(a["bom"], b["bom"], "bom.json")
    for rel in sorted(set(a["files"]) | set(b["files"])):
        if rel not in a["files"] or rel not in b["files"]:
            diff.append(f"{rel}: {'added' if rel in b['files'] else 'removed'}")
            continue
        g, o = _text_diff(a["files"][rel], b["files"][rel], rel)
        diff += g
        order += o
    for rel in sorted(set(a["dxf"]) | set(b["dxf"])):
        if rel not in a["dxf"] or rel not in b["dxf"]:
            diff.append(f"{rel}: {'added' if rel in b['dxf'] else 'removed'}")
            continue
        g, o = _dxf_diff(a["dxf"][rel], b["dxf"][rel], rel)
        diff += g
        order += o
    meshes = [rel for rel in sorted(set(a["mesh"]) | set(b["mesh"]))
              if a["mesh"].get(rel) != b["mesh"].get(rel)]
    for rel in meshes:
        if rel not in a["mesh"] or rel not in b["mesh"]:
            diff.append(f"{rel}: {'added' if rel in b['mesh'] else 'removed'}")
        else:
            (diff if diff else order).append(
                f"{rel}: another mesh" + ("" if diff else " (the parts' B-rep unchanged)"))
    return diff, order


def diff_dirs(base: Path, new: Path, names: list[str]) -> int:
    worst = 0
    print(f"identity gate: {base} -> {new}")
    for name in names:
        fa, fb = base / f"{name}.json", new / f"{name}.json"
        if not fa.is_file() or not fb.is_file():
            print(f"  {name}: MISSING ({fa if not fa.is_file() else fb})")
            worst = 2
            continue
        diff, order = compare_docs(json.loads(fa.read_text()), json.loads(fb.read_text()))
        if diff:
            verdict, worst = "DIFFERENT", 2
        elif order:
            verdict, worst = "geometry identical, order differs", max(worst, 1)
        else:
            verdict = "identical"
        print(f"  {name}: {verdict}")
        for d in diff[:40]:
            print(f"    {d}")
        if len(diff) > 40:
            print(f"    ... {len(diff) - 40} more differences")
        for d in order[:20]:
            print(f"    order: {d}")
        if len(order) > 20:
            print(f"    order: ... {len(order) - 20} more")
    print({0: "identical", 1: "geometry identical, order differs (a user decision)",
           2: "DIFFERENT"}[worst])
    return worst


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    for cmd in ("snapshot", "compare"):
        p = sub.add_parser(cmd)
        p.add_argument("dir", type=Path, help="the snapshot folder (compare: the baseline)")
        p.add_argument("--designs", default=",".join(DESIGNS),
                       help=f"comma-separated, of {', '.join(DESIGNS)}")
        p.add_argument("-j", "--jobs", type=int, default=len(DESIGNS),
                       help="designs at once (one process each)")
        if cmd == "compare":
            p.add_argument("--out", type=Path, default=None,
                           help="where this tree's snapshot goes (default: a temporary folder)")
    p = sub.add_parser("diff")
    p.add_argument("a", type=Path)
    p.add_argument("b", type=Path)
    p.add_argument("--designs", default=None)
    p = sub.add_parser("_one")
    p.add_argument("name")
    p.add_argument("out", type=Path)
    args = ap.parse_args(argv)
    if args.cmd == "_one":
        run_one(args.name, args.out)
        return 0
    names = (args.designs.split(",") if args.designs
             else sorted(p.stem for p in args.a.glob("*.json") if p.stem in DESIGNS))
    unknown = [n for n in names if n not in DESIGNS]
    if unknown:
        ap.error(f"unknown designs {unknown}: {', '.join(DESIGNS)}")
    if args.cmd == "snapshot":
        return snapshot(args.dir, names, args.jobs)
    if args.cmd == "diff":
        return diff_dirs(args.a, args.b, names)
    new = args.out or Path(tempfile.mkdtemp(prefix="gate-compare-"))
    if snapshot(new, names, args.jobs):
        print("the snapshot failed: no comparison")
        return 2
    return diff_dirs(args.dir, new, names)


if __name__ == "__main__":
    raise SystemExit(main())
