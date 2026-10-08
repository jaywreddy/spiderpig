"""The identity gate: did a change move any part, plan, BOM row or cut file?

A script, not a test. Before a product change, on the base::

    mise run gate -- snapshot DIR          # (uv run python tests/gate/identity_gate.py ...)

after it, on the branch::

    mise run gate -- compare DIR           # snapshots the tree again and diffs against DIR
    mise run gate -- diff DIR_A DIR_B      # two snapshots
    mise run gate -- doc DIR               # docs/agentlib/DESIGNS.md from a snapshot

For every design of :data:`DESIGNS` (one process each, ``-j`` at once, with its workers:
the contract angles and the ``t=1`` half in processes of their own, :data:`SPLIT`; both
follow the free cores unless given, :func:`plan_cores`) it records, as ``spiderpig audit
--no-sim`` and ``spiderpig build`` make them:

- the **plan**: layers, top, crank route, heads, gaps, thicknesses, sunk heads, height,
  optimality and proof, ``describe()``;
- the **audit** (:func:`spiderpig.tools.audit.audit_module`, ``sim=False``: the family's
  pin loads) minus its ``seconds``: contract, clashes, solids, wobble, strength, cut
  rules, sheets, BOM summary;
- every **part** of the fabrications at the audit's clash angles (``t=1``, ``t=4.38``):
  class, fab, BOM key, sheet, colour, pose, volume, area, centre of mass, bounding box,
  solid / face / edge counts, and the body order;
- the **build** (``spiderpig build`` of the audit's ``t=1`` fabrication, or with the
  build's worker of that worker's own): ``bom.json`` and every text it writes
  (``bom.csv/md``, ``ORDER.md``, the ``parts.csv`` / ``order.csv`` files,
  ``manifest.json``), every DXF (sheets and per-part) as its entities, and a hash of every
  STL (the STEP file, never read, isn't written).

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
import re
import shutil
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
    from build123d import Solid
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    p = b.part
    bb = p.bounding_box()
    if type(p) is Solid:    # (Solid's volume and center() each make this one call)
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(p.wrapped, props)
        c = props.CentreOfMass()
        volume, com = props.Mass(), [c.X(), c.Y(), c.Z()]
    else:
        c = p.center()
        volume, com = p.volume, [c.X, c.Y, c.Z]
    return {
        "name": b.name, "class": type(p).__name__, "fab": b.fab, "bom_key": b.bom_key,
        "sheet": b.sheet, "color": b.color, "rigid_with": b.rigid_with,
        "pose": [list(map(float, row)) for row in b.pose.matrix],
        "volume": volume, "area": p.area, "com": com,
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


_VOLATILE = re.compile(r'"(design|engine_version)": "[^"]*"')


def _mask(rel: str, text: str) -> str:
    """A text output with what any engine edit changes masked: ``manifest.json``'s design id
    and engine version (the id hashes the engine version, so every edit under spiderpig/
    changes it, whatever the parts)."""
    return _VOLATILE.sub(r'"\1": "<masked>"', text) if rel.endswith("manifest.json") else text


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


SPLIT = "contract:0,1.6|contract:3.2,4.8|build"
"""The work beside the audit's own process, one process per ``|`` (``GATE_SPLIT``; empty:
everything in one process): ``contract:T,...`` the contract at those angles, ``build`` the
``t=1`` fabrication with its clashes, solids and parts, and the build of it. Each worker
takes the plan from the design's store (re-made and verified), where the audit's process
recorded it first. Unset, :func:`plan_cores` picks it from the cores free."""


def free_cores() -> int:
    """The machine's cores less its load average, at most the cores this process may run
    on (at least 1)."""
    total = os.cpu_count() or 1
    mine = len(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else total
    return max(1, min(mine, round(total - os.getloadavg()[0])))


def plan_cores(n_designs: int, jobs: int | None, split: str | None,
               free: int | None = None) -> tuple[int, str]:
    """``(jobs, split)``: the designs at once and each one's workers (:data:`SPLIT`), those
    not given from the free cores. A design's share of them (all of them, or with ``-j``
    given, the free cores over ``jobs``: an explicit ``-j`` caps the processes at about the
    free cores) picks the split: the whole one from 4 cores, the build's worker alone from
    2, none under 2; then, ``jobs`` not given, as many designs at once as their processes
    have cores (at least 1)."""
    free = free_cores() if free is None else free
    if split is None:
        share = free // jobs if jobs else free
        split = SPLIT if share >= 4 else "build" if share >= 2 else ""
    if jobs is None:
        per = 1 + len([t for t in split.split("|") if t])
        jobs = min(n_designs, max(1, free // per))
    return jobs, split


def _occt() -> None:
    """OCCT's pool for this process: two threads (``GATE_OCCT_THREADS``), as the baselines
    were snapshotted (OCCT's last digits depend on it: ``workers.occt_threads``)."""
    from OCP.OSD import OSD_ThreadPool

    OSD_ThreadPool.DefaultPool_s(int(os.environ.get("GATE_OCCT_THREADS", "2")))


def _setup(name: str, work: Path):
    import spiderpig.build as build_mod

    argv = DESIGNS[name] + ["--store", str(work / "store"), "--out", str(work / "build")]
    return argv, build_mod._parse_args(argv).config


def _build_outputs(name: str, work: Path, argv: list[str], mech) -> dict:
    """``spiderpig build`` of the fabrication ``mech`` (its ``t=1``; the call the command
    makes): every text it writes, every DXF as its entities, a hash of every STL. The STEP
    file is never read (a timestamp in its header; the parts are its geometry), so it is
    not written."""
    import spiderpig.build as build_mod
    from spiderpig.mechanism import Mechanism

    fabricate = build_mod.fabricate
    build_mod.fabricate = lambda tmpl, cfg, t=1.0: (mech if float(t) == 1.0
                                                    else fabricate(tmpl, cfg, t))
    Mechanism.export_step = lambda self, path: None
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
            continue
        else:
            files[rel] = _mask(rel, f.read_text().replace(str(work), "<WORK>"))
    return {"files": files, "dxf": dxf, "mesh": mesh}


def _build_task(name: str, work: Path) -> dict:
    """The audit's ``t=1`` half (clashes, solids, the parts) and the build of that same
    fabrication."""
    from spiderpig.construction.contract import bad_solids, clashes
    from spiderpig.fabricate import fabricate, template_for

    argv, config = _setup(name, work)
    _plan(config, work)
    mech = fabricate(template_for(config), config, 1.0)
    res = {"clash": clashes(mech), "solids": bad_solids(mech),
           "parts": [_part_doc(b) for b in mech.bodies if b.part is not None]}
    res.update(_build_outputs(name, work, argv, mech))
    return res


def _plan(config, work: Path):
    from spiderpig import api
    from spiderpig.store import Store

    return api.plan_config(config, Store.of(work / "store"))


def _contract_task(name: str, work: Path, ts) -> dict:
    from spiderpig.construction.contract import check_side
    from spiderpig.fabricate import template_for

    _, config = _setup(name, work)
    design = _plan(config, work)
    tmpl = template_for(config)
    return {f"t={t:g}": check_side(design, tmpl.freeze_at(t)) for t in ts}


def run_task(name: str, work: Path, task: str, result: Path) -> None:
    kind, _, arg = task.partition(":")
    _occt()
    if kind == "build":
        res = _build_task(name, work)
    elif kind == "contract":
        res = _contract_task(name, work, [float(x) for x in arg.split(",")])
    else:
        raise SystemExit(f"unknown task {task!r}")
    result.write_text(json.dumps(res, default=str))


def run_one(name: str, out: Path) -> None:
    """Snapshot one design into ``out/<name>.json``."""
    _occt()
    tasks = [t for t in os.environ.get("GATE_SPLIT", SPLIT).split("|") if t]
    import spiderpig.tools.audit as audit_mod
    from spiderpig.design import engine_version
    from spiderpig.fabricate import design_side, template_for

    t0 = time.time()
    work = Path(tempfile.mkdtemp(prefix=f"gate-{name}-"))
    argv, config = _setup(name, work)
    from spiderpig.store import Store

    store = Store.of(work / "store")
    procs = {}
    if tasks:
        _plan(config, work)         # solved here once, recorded in the store for the workers
        for i, task in enumerate(tasks):
            res = work / f"task{i}.json"
            log = open(work / f"task{i}.log", "w")  # noqa: SIM115 (closed in result())
            procs[task] = (subprocess.Popen(
                [sys.executable, __file__, "_task", name, str(work), task, str(res)],
                stdout=log, stderr=subprocess.STDOUT, cwd=REPO), res, log)

    results: dict[str, dict] = {}

    def result(task: str) -> dict:
        if task not in results:
            p, res, log = procs[task]
            rc = p.wait()
            log.close()
            sys.stdout.write((work / f"task{tasks.index(task)}.log").read_text())
            if rc != 0:
                raise SystemExit(f"{name}: task {task} exited {rc}")
            results[task] = json.loads(res.read_text())
        return results[task]

    contract_tasks = {float(x): t for t in tasks if t.startswith("contract:")
                      for x in t.partition(":")[2].split(",")}
    build_remote = "build" in procs
    captured: dict[float, object] = {}
    fabricate = audit_mod.fabricate
    check_side, clashes, bad_solids = audit_mod.check_side, audit_mod.clashes, audit_mod.bad_solids
    remote_t1 = object()            # the t=1 fabrication, made in the build worker

    def capture(tmpl, cfg, t=1.0):
        if build_remote and float(t) == 1.0:
            return remote_t1
        mech = fabricate(tmpl, cfg, t)
        captured.setdefault(float(t), mech)
        return mech

    contract_ts = iter(TS_CONTRACT)

    def contract(design, mech):
        t = next(contract_ts)
        if t in contract_tasks:
            return result(contract_tasks[t])[f"t={t:g}"]
        return check_side(design, mech)

    audit_mod.fabricate = capture
    audit_mod.check_side = contract
    audit_mod.clashes = lambda m, *a, **k: (result("build")["clash"] if m is remote_t1
                                            else clashes(m, *a, **k))
    audit_mod.bad_solids = lambda m: (result("build")["solids"] if m is remote_t1
                                      else bad_solids(m))
    rep = audit_mod.audit_module(config.module, config, TS_CONTRACT, TS_CLASH, store, sim=False)
    rep.pop("seconds", None)
    plan = design_side(template_for(config), config).plan
    parts = {f"t={t:g}": [_part_doc(b) for b in m.bodies if b.part is not None]
             for t, m in sorted(captured.items())}
    if build_remote:
        built = result("build")
        parts["t=1"] = built["parts"]
        parts = dict(sorted(parts.items(), key=lambda kv: float(kv[0][2:])))
    else:       # the build of the audit's own t=1 fabrication
        built = _build_outputs(name, work, argv, captured[1.0])
    for task in procs:              # every worker done (and its failure raised)
        result(task)
    files = built["files"]
    doc = {
        "design": name, "argv": DESIGNS[name], "engine_version": engine_version(),
        "config": repr(config), "plan": _plan_doc(plan), "audit": rep, "parts": parts,
        "bom": json.loads(files.pop("bom.json")), "files": files, "dxf": built["dxf"],
        "mesh": built["mesh"], "seconds": round(time.time() - t0, 1),
    }
    (out / f"{name}.json").write_text(json.dumps(doc, indent=1, default=str))
    shutil.rmtree(work, ignore_errors=True)     # (its store and build: ~60 MB a design)


# ---------------------------------------------------------------------------
# snapshots
# ---------------------------------------------------------------------------


def _git(*args: str) -> str:
    try:
        return subprocess.run(["git", *args], cwd=REPO, capture_output=True, text=True,
                              check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def snapshot(out: Path, names: list[str], jobs: int | None) -> int:
    out.mkdir(parents=True, exist_ok=True)
    cad = tempfile.mkdtemp(prefix="gate-cad-")
    jobs, split = plan_cores(len(names), jobs, os.environ.get("GATE_SPLIT"))
    print(f"{len(names)} designs, {jobs} at once, each with "
          f"{split.replace('|', ', ') or 'no'} workers ({free_cores()} cores free)",
          flush=True)
    env = {**os.environ, **ENV, "SPIDERPIG_CAD_CACHE": cad, "GATE_SPLIT": split}
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
    """(differences, order-only differences) between two designs' snapshots (a baseline
    recorded before the masking is masked here too)."""
    for doc in (a, b):
        files = doc.get("files") or {}
        for rel in list(files):
            files[rel] = _mask(rel, files[rel])
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


# ---------------------------------------------------------------------------
# the design numbers' one source (docs/agentlib/DESIGNS.md)
# ---------------------------------------------------------------------------

DESIGNS_MD = REPO / "docs" / "agentlib" / "DESIGNS.md"
_CONFIG_FIELD = re.compile(r"\b(linkage|module|robot|sheet|frame_sheet|crank_sheet|link_sheets"
                           r"|servo|pillar|pin|crank|heads)=('[^']*'|None|True|False|\([^)]*\))")


def _config_fields(text: str) -> dict[str, str]:
    """The fields of a snapshot's ``repr(BuildConfig)`` the doc shows (``params`` left out)."""
    head = text.split("params=", 1)[0]
    return {k: v.strip("'") for k, v in _CONFIG_FIELD.findall(head)}


def _first_clause(message: str) -> str:
    """An audit message without its parentheses (the fixes, the details), at most 120
    characters."""
    head, n = message, 1
    while n:
        head, n = re.subn(r"\s+\([^()]*\)", "", head)
    head = head.strip().rstrip(":;, ")
    return head if len(head) <= 120 else head[:117] + "..."


def designs_doc(snap: Path) -> str:
    """``DESIGNS.md`` from a gate snapshot folder: each design's numbers, as the gate saw them."""
    meta = json.loads((snap / "snapshot.json").read_text())
    names = [n for n in meta.get("designs", DESIGNS) if (snap / f"{n}.json").is_file()]
    docs = {n: json.loads((snap / f"{n}.json").read_text()) for n in names}
    commit = (meta.get("commit") or "unknown")[:7]
    out = [
        "# Design numbers (generated)",
        "",
        "<!-- Generated by `mise run gate -- doc <snapshot dir>` "
        "(tests/gate/identity_gate.py). Do not edit by hand. -->",
        "",
        f"The gate's designs as its snapshot `{snap.name}` recorded them: commit `{commit}` "
        f"(branch `{meta.get('branch') or '?'}`, written {meta.get('written_at') or '?'}"
        + (", a dirty tree" if meta.get("spiderpig_dirty") else "") + "). "
        "This file is the one source for the default designs' layer counts, heights, parts "
        "and costs: the other docs link here instead of quoting them. Re-generate it with "
        "each new gate baseline (docs/agentlib/TESTING.md).",
        "",
        "The audit is the gate's: `spiderpig audit --no-sim` (the linkage family's pin loads, "
        "not the design's own MuJoCo loads), so its strength verdict can differ from a full "
        "`mise run audit`. Height is the plan's stack, outer plate to the inner plate's top "
        "heads; parts are the fabricated bodies at `t=1` (hardware included); laser parts "
        "are the cut-rule review's.",
        "",
        "| design | linkage / module | layers | height mm | proven thinnest | crank | pillar "
        "| pin | parts | laser parts | audit | BOM |",
        "|---|---|--:|--:|---|---|---|---|--:|--:|---|--:|",
    ]
    detail: list[str] = []
    for name, d in docs.items():
        cfg, plan, audit = _config_fields(d.get("config", "")), d["plan"], d["audit"]
        problems, warnings = audit.get("problems") or [], audit.get("warnings") or []
        verdict = ("OK" if not problems else f"FAIL ({len(problems)})") + (
            f", {len(warnings)} warning{'s' * (len(warnings) != 1)}" if warnings else "")
        bom = audit.get("bom") or d.get("bom") or {}
        cost = bom.get("cost_usd")
        n_parts = len(next(iter(d.get("parts", {}).values()), []))
        laser = (audit.get("manufacture") or {}).get("parts", "")
        what = f"{cfg.get('linkage', '?')} / {cfg.get('module', '?')}" + (
            "" if cfg.get("robot") == "True" else " (one side)")
        out.append(
            f"| `{name}` | {what} | {audit.get('layers', plan['top'] + 1)} | "
            f"{plan['height']:.1f} | "
            f"{'yes' if plan.get('optimal') else 'no'} | {cfg.get('crank', '?')} | "
            f"{cfg.get('pillar', '?')} | {cfg.get('pin', '?')} | {n_parts} | {laser} | "
            f"{verdict} | {'' if cost is None else f'${cost:,.2f}'} |")
        detail += ["", f"## `{name}`", "",
                   f"- build options: `{' '.join(d.get('argv', []))}`",
                   f"- sheets: links `{cfg.get('sheet', '?')}`, frame "
                   f"`{cfg.get('frame_sheet', '?')}`, crank `{cfg.get('crank_sheet', '?')}`"
                   + ("" if cfg.get("link_sheets") in (None, "None")
                      else f", per link `{cfg['link_sheets']}`"),
                   "- DXF sheets cut (sheet: count): " + ", ".join(
                       f"`{k}` {v}" for k, v in sorted((audit.get("sheets") or {}).items())),
                   f"- plan heads `{plan.get('heads')}`; gaps over layers "
                   + (", ".join(f"{k}: {v} mm" for k, v in (plan.get("gaps") or {}).items())
                      or "none")]
        if bom:
            unpriced = len(bom.get("unpriced") or [])
            detail.append(f"- BOM: {bom.get('items', len(bom.get('purchased') or []))} lines"
                          + ("" if cost is None else f", ${cost:,.2f}")
                          + (f" ({unpriced} lines unpriced, not in the total)" if unpriced
                             else ""))
        for label, msgs in (("problems", problems), ("warnings", warnings)):
            if msgs:
                detail.append(f"- audit {label}:")
                detail += [f"  - {_first_clause(m)}" for m in msgs]
    return "\n".join(out + detail) + "\n"


def write_designs_doc(snap: Path, dest: Path | None) -> int:
    if not (snap / "snapshot.json").is_file():
        print(f"{snap} is no gate snapshot (no snapshot.json)", file=sys.stderr)
        return 2
    text = designs_doc(snap)
    if dest is None:
        sys.stdout.write(text)
    else:
        dest.write_text(text)
        print(f"wrote {dest}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    for cmd in ("snapshot", "compare"):
        p = sub.add_parser(cmd)
        p.add_argument("dir", type=Path, help="the snapshot folder (compare: the baseline)")
        p.add_argument("--designs", default=",".join(DESIGNS),
                       help=f"comma-separated, of {', '.join(DESIGNS)}")
        p.add_argument("-j", "--jobs", type=int, default=None,
                       help="designs at once (default: as the free cores allow, "
                            "plan_cores)")
        if cmd == "compare":
            p.add_argument("--out", type=Path, default=None,
                           help="where this tree's snapshot goes (default: a temporary folder)")
    p = sub.add_parser("diff")
    p.add_argument("a", type=Path)
    p.add_argument("b", type=Path)
    p.add_argument("--designs", default=None)
    p = sub.add_parser("doc", help="write docs/agentlib/DESIGNS.md from a snapshot")
    p.add_argument("dir", type=Path, help="the snapshot folder")
    p.add_argument("--out", default=str(DESIGNS_MD),
                   help="where to write it ('-': stdout; default: %(default)s)")
    p = sub.add_parser("_one")
    p.add_argument("name")
    p.add_argument("out", type=Path)
    p = sub.add_parser("_task")
    for a in ("name", "work", "task", "result"):
        p.add_argument(a)
    args = ap.parse_args(argv)
    if args.cmd == "_one":
        run_one(args.name, args.out)
        return 0
    if args.cmd == "_task":
        run_task(args.name, Path(args.work), args.task, Path(args.result))
        return 0
    if args.cmd == "doc":
        return write_designs_doc(args.dir, None if args.out == "-" else Path(args.out))
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
