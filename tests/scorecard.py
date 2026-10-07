"""The scorecard: every number the roadmap (``docs/agentlib/ROADMAP.md``) states, one command
away (``mise run scorecard``).

    python -m tests.scorecard                         # -> build/scorecard.json, and a table
    python -m tests.scorecard --only build,static     # some sections
    python -m tests.scorecard --gate --full --audit   # the opt-in, heavy sections too
    python -m tests.scorecard --compare A.json B.json # the deltas between two scorecards

Sections (``--only`` names them; the default runs the first four: ~7.5 min at load 15 on the
20-core box, the roadmap's 6 min to be judged on a quiet one):

- ``build``: ``spiderpig build --profile`` of the Strider double (the default design) into a
  temporary folder and store, twice: an empty store (the plan solved), then the same store
  (the plan reused). Each run's stages (``spiderpig.tools.build_profile.STAGES``), its own
  wall time and the process's wall and CPU measured from here. The pair runs ``--runs`` times
  (3); the numbers are the medians.
- ``tiers``: each module tier (``python -m tests.tiers <module>``), wall and CPU, its tests,
  its xdist workers.
- ``quick``: the quick tier (``-m 'not slow and not e2e' -n 4``) with ``coverage.py`` over
  ``spiderpig/`` (pytest-cov, combined across the xdist workers; ``COVERAGE_CORE=sysmon``
  keeps the overhead small), its tests and its 10 slowest. ``--no-coverage`` times it bare.
- ``static``: pyright's error count (``[tool.pyright]``, basic; also without the stub noise of
  the compiled OCP and mujoco), ruff's ``RUF`` findings (``--extend-select RUF``), each
  module over 800 lines (``spiderpig/**/*.py``, ``viewer/src/**/*.ts``), the product's lines,
  CLAUDE.md's lines and words, the doc check's misses, whether CI exists. Run alone
  (``--parallel-static``: beside the tiers, whose times its ~90 CPU-s then inflate).
- ``gate`` (``--gate``): ``mise run gate -- snapshot`` of the six designs, wall and CPU.
- ``full`` (``--full``): the full suite (``-m 'not e2e'``, ``--full-workers``, 6 on the
  shared box: the roadmap's 12 would load it past measuring; recorded as ``workers``), and with
  ``--cold`` again on an empty test cache (the first run after an engine edit).
- ``audit`` (``--audit``): ``spiderpig audit --modules double`` on an empty store, then warm.

Every section records the load average before and after it (``loadavg``): the roadmap's
timings are medians of 3 at a load under 4, and a number taken above that is noise.
``--compare`` prints changed or non-zero exit codes first and warns when the two scorecards
used other xdist workers or build runs for a metric, or ran at loads over 2x apart.
CPU is each process's own ``wait4`` rusage, its waited-for children (the xdist workers)
included.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = ROOT / "build" / "scorecard.json"
SECTIONS = ("build", "tiers", "quick", "static", "gate", "full", "audit")
DEFAULT_SECTIONS = ("build", "tiers", "quick", "static")
LARGE = 800
"""A module over this many lines is listed (``sizes.large``)."""


# --------------------------------------------------------------------------- running


def _load() -> list[float]:
    return [round(x, 2) for x in os.getloadavg()]


def run(cmd: list[str], *, env: dict | None = None, log: Path | None = None,
        timeout: float | None = None) -> dict:
    """Run ``cmd`` from the repo root: its exit code, wall seconds, and CPU seconds (user +
    sys of the process and every descendant it waited for, from ``wait4``). Its output goes
    to ``log`` (else is discarded)."""
    full_env = {**os.environ, **(env or {})}
    out = open(log, "w") if log else subprocess.DEVNULL     # noqa: SIM115
    t0 = time.perf_counter()
    try:
        p = subprocess.Popen(cmd, cwd=ROOT, env=full_env, stdout=out, stderr=subprocess.STDOUT)
        deadline = None if timeout is None else t0 + timeout
        while True:
            pid, status, ru = os.wait4(p.pid, os.WNOHANG if deadline else 0)
            if pid:
                break
            if deadline and time.perf_counter() > deadline:
                p.kill()
                pid, status, ru = os.wait4(p.pid, 0)
                break
            time.sleep(0.2)
        p.returncode = os.waitstatus_to_exitcode(status)
    finally:
        if log:
            out.close()
    return {"rc": p.returncode, "wall_s": round(time.perf_counter() - t0, 2),
            "cpu_s": round(ru.ru_utime + ru.ru_stime, 2)}


def _python() -> list[str]:
    return [sys.executable]


def _junit(path: Path) -> dict:
    """Test counts and the 10 slowest cases of a junit XML file."""
    if not path.exists():
        return {"tests": 0}
    root = ET.parse(path).getroot()
    suites = [root] if root.tag == "testsuite" else list(root.iter("testsuite"))
    counts = {k: sum(int(s.get(k, 0)) for s in suites)
              for k in ("tests", "failures", "errors", "skipped")}
    cases = [(float(c.get("time", 0)), f"{c.get('classname')}::{c.get('name')}")
             for s in suites for c in s.iter("testcase")]
    cases.sort(reverse=True)
    counts["passed"] = (counts["tests"] - counts["failures"] - counts["errors"]
                        - counts["skipped"])
    counts["slowest"] = [{"test": n, "s": round(t, 2)} for t, n in cases[:10]]
    return counts


def _median(values: list[float]) -> float | None:
    values = [v for v in values if v is not None]
    return round(statistics.median(values), 3) if values else None


# --------------------------------------------------------------------------- sections


def section_build(work: Path, runs: int) -> dict:
    """``spiderpig build --profile`` of the default design: empty store, then warm."""
    from spiderpig.tools.build_profile import STAGES

    env = {"SPIDERPIG_OFFLINE": "1"}
    samples: dict[str, list[dict]] = {"cold": [], "warm": []}
    for i in range(runs):
        store = work / f"store{i}"
        for phase in ("cold", "warm"):
            prof = work / f"build{i}_{phase}.json"
            r = run([*_python(), "-m", "spiderpig.cli", "build", "--profile-json", str(prof),
                     "--out", str(work / f"out{i}"), "--store", str(store)],
                    env={**env, "SPIDERPIG_STORE": str(store)},
                    log=work / f"build{i}_{phase}.log")
            doc = json.loads(prof.read_text()) if prof.exists() else {}
            samples[phase].append({**r, "stages": doc.get("stages", {}),
                                   "internal_wall_s": doc.get("wall_s"),
                                   "stages_sum_s": doc.get("stages_sum_s")})
    out = {"design": "strider double robot (BuildConfig())", "runs": runs}
    for phase, rows in samples.items():
        out[phase] = {
            "rc": [r["rc"] for r in rows],
            "wall_s": _median([r["wall_s"] for r in rows]),
            "cpu_s": _median([r["cpu_s"] for r in rows]),
            "internal_wall_s": _median([r["internal_wall_s"] for r in rows]),
            "stages_sum_s": _median([r["stages_sum_s"] for r in rows]),
            "stages": {k: _median([r["stages"].get(k) for r in rows]) for k in STAGES},
            "samples": [{k: v for k, v in r.items() if k != "stages"} for r in rows],
        }
    return out


def section_tiers(work: Path) -> dict:
    """Each module's fast tier, with its default workers."""
    from tests._modules import MODULES
    from tests.tiers import TIER_WORKERS

    out = {}
    for module in MODULES:
        junit = work / f"tier_{module}.xml"
        r = run([*_python(), "-m", "tests.tiers", module, f"--junitxml={junit}"],
                log=work / f"tier_{module}.log")
        counts = _junit(junit)
        counts.pop("slowest", None)
        workers = int(os.environ.get("SPIDERPIG_TIER_WORKERS", TIER_WORKERS.get(module, 4)))
        out[module] = {**r, "workers": workers, **counts}
    out["total_wall_s"] = round(sum(v["wall_s"] for v in out.values()), 1)
    return out


def section_quick(work: Path, coverage: bool, workers: int = 4) -> dict:
    """The quick tier, under coverage.py unless ``coverage`` is false."""
    junit = work / "quick.xml"
    cmd = [*_python(), "-m", "pytest", "-p", "no:warnings", "-m", "not slow and not e2e",
           "-n", str(workers), "--dist", "worksteal", f"--junitxml={junit}"]
    env = {}
    cov_json = work / "coverage.json"
    if coverage:
        cmd += ["--cov=spiderpig", f"--cov-report=json:{cov_json}", "--cov-report="]
        env = {"COVERAGE_CORE": "sysmon", "COVERAGE_FILE": str(work / ".coverage")}
    r = run(cmd, env=env, log=work / "quick.log")
    out = {**r, "workers": workers, "coverage_on": coverage, **_junit(junit)}
    if coverage and cov_json.exists():
        totals = json.loads(cov_json.read_text())["totals"]
        out["coverage_pct"] = round(totals["percent_covered"], 2)
        out["coverage_lines"] = {"covered": totals["covered_lines"],
                                 "statements": totals["num_statements"]}
    return out


def _pyright(work: Path) -> dict:
    exe = ROOT / "viewer" / "node_modules" / ".bin" / "pyright"
    if not exe.exists():
        return {"error": "viewer/node_modules/.bin/pyright missing: mise run viewer-install"}
    env = dict(os.environ)
    if shutil.which("node") is None:        # outside `mise run`: mise's node, if any
        mise = shutil.which("mise") or str(Path.home() / ".local" / "bin" / "mise")
        found = subprocess.run([mise, "which", "node"], cwd=ROOT, capture_output=True,
                               text=True, check=False) if Path(mise).exists() else None
        if not found or found.returncode != 0:
            return {"error": "no node on PATH (run through `mise run scorecard`)"}
        env["PATH"] = f"{Path(found.stdout.strip()).parent}{os.pathsep}{env['PATH']}"
    report = work / "pyright.json"
    with open(report, "w") as f:
        t0 = time.perf_counter()
        p = subprocess.run([str(exe), "--outputjson"], cwd=ROOT, stdout=f, env=env,
                           stderr=subprocess.PIPE, text=True, check=False)
    try:
        doc = json.loads(report.read_text())
    except json.JSONDecodeError:
        return {"error": f"pyright exited {p.returncode}: {p.stderr[-500:]}"}
    errors = [d for d in doc["generalDiagnostics"] if d["severity"] == "error"]
    by_rule: dict[str, int] = {}
    noise = 0
    for d in errors:
        by_rule[d.get("rule", "?")] = by_rule.get(d.get("rule", "?"), 0) + 1
        noise += _stub_noise(d)
    return {"errors": len(errors), "errors_excl_stub_noise": len(errors) - noise,
            "stub_noise": noise, "warnings": doc["summary"]["warningCount"],
            "files": doc["summary"]["filesAnalyzed"], "version": doc.get("version"),
            "by_rule": dict(sorted(by_rule.items(), key=lambda kv: -kv[1])),
            "seconds": round(time.perf_counter() - t0, 1)}


_LINES: dict[str, list[str]] = {}


def _stub_noise(diag: dict) -> int:
    """1 when a pyright error is the compiled modules' missing stubs: an attribute of
    ``mujoco``, or a name imported from ``OCP`` it can't see."""
    msg = diag.get("message", "")
    if re.search(r'of module "(mujoco|OCP)', msg):
        return 1
    if "unknown import symbol" in msg:
        path = diag["file"]
        lines = _LINES.setdefault(path, Path(path).read_text().splitlines())
        line = lines[diag["range"]["start"]["line"]]
        return int(bool(re.match(r"\s*from OCP[\s.]", line)))
    return 0


def _ruff() -> dict:
    p = subprocess.run([*_python(), "-m", "ruff", "check", ".", "--extend-select", "RUF",
                        "--statistics", "--exit-zero", "--no-cache"],
                       cwd=ROOT, capture_output=True, text=True, check=False)
    codes: dict[str, int] = {}
    for line in p.stdout.splitlines():
        m = re.match(r"\s*(\d+)\s+(\S+)", line)
        if m:
            codes[m.group(2)] = int(m.group(1))
    ruf = {k: v for k, v in codes.items() if k.startswith("RUF")}
    return {"ruf": sum(ruf.values()), "ruf_by_code": ruf,
            "total_with_ruf": sum(codes.values()),
            "default_select": sum(v for k, v in codes.items() if not k.startswith("RUF"))}


def _sizes() -> dict:
    files = sorted([*(ROOT / "spiderpig").rglob("*.py"), *(ROOT / "viewer" / "src").rglob("*.ts")])
    lines = {str(f.relative_to(ROOT)): len(f.read_text().splitlines()) for f in files
             if "node_modules" not in f.parts and "dist" not in f.parts}
    py = {k: v for k, v in lines.items() if k.endswith(".py")}
    return {"large": dict(sorted(((k, v) for k, v in lines.items() if v > LARGE),
                                 key=lambda kv: -kv[1])),
            "product_py_lines": sum(py.values()), "product_py_files": len(py),
            "largest_py": max(py.values()) if py else 0,
            "viewer_ts_lines": sum(v for k, v in lines.items() if k.endswith(".ts"))}


def _docs() -> dict:
    text = (ROOT / "CLAUDE.md").read_text()
    out = {"claude_md_lines": len(text.splitlines()), "claude_md_words": len(text.split())}
    from tests import doc_check

    misses = doc_check.check(doc_check.DEFAULT_DOCS)
    out["doc_check_misses"] = len(misses)
    return out


def _ci() -> dict:
    wf = ROOT / ".github" / "workflows" / "ci.yml"
    if not wf.exists():
        return {"present": False}
    jobs = re.findall(r"^  ([a-z][\w-]*):\s*$", wf.read_text().split("\njobs:", 1)[-1], re.M)
    return {"present": True, "workflow": str(wf.relative_to(ROOT)), "jobs": jobs}


def section_static(work: Path) -> dict:
    return {"pyright": _pyright(work), "ruff": _ruff(), "sizes": _sizes(), "docs": _docs(),
            "ci": _ci()}


def section_gate(work: Path) -> dict:
    snap = work / "gate"
    r = run([*_python(), "tests/gate/identity_gate.py", "snapshot", str(snap), "-j", "2"],
            log=work / "gate.log")
    per = {}
    for f in sorted(snap.glob("*.json")):
        doc = json.loads(f.read_text())
        if "seconds" in doc:
            per[f.stem] = doc["seconds"]
    return {**r, "jobs": 2, "per_design_s": per}


def section_full(work: Path, workers: int, cold: bool) -> dict:
    out = {}
    for name, env in (("warm", {}), *((("cold", {"SPIDERPIG_TEST_CACHE": str(work / "cold")}),)
                                       if cold else ())):
        junit = work / f"full_{name}.xml"
        r = run([*_python(), "-m", "pytest", "-p", "no:warnings", "-m", "not e2e",
                 "-n", str(workers), "--dist", "worksteal", f"--junitxml={junit}"],
                env=env, log=work / f"full_{name}.log")
        out[name] = {**r, "workers": workers, **_junit(junit)}
    return out


def section_audit(work: Path) -> dict:
    out = {}
    store = work / "audit_store"
    for phase in ("cold", "warm"):
        out[phase] = run([*_python(), "-m", "spiderpig.cli", "audit", "--modules", "double",
                          "--out", str(work / f"audit_{phase}"), "--store", str(store)],
                         env={"SPIDERPIG_OFFLINE": "1", "SPIDERPIG_STORE": str(store)},
                         log=work / f"audit_{phase}.log")
    out["module"] = "double (strider)"
    return out


# --------------------------------------------------------------------------- report


def _meta() -> dict:
    def git(*a):
        p = subprocess.run(["git", *a], cwd=ROOT, capture_output=True, text=True, check=False)
        return p.stdout.strip()

    from spiderpig.design import engine_version

    return {"time": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "commit": git("rev-parse", "HEAD"),
            "branch": git("rev-parse", "--abbrev-ref", "HEAD"),
            "dirty": bool(git("status", "--porcelain", "--untracked-files=no")),
            "engine_version": engine_version(), "host": platform.node(),
            "cpus": os.cpu_count(), "python": platform.python_version()}


def flatten(doc, prefix: str = "") -> dict[str, float]:
    """Every numeric leaf of ``doc`` by its dotted path (lists of numbers by index)."""
    out: dict[str, float] = {}
    if isinstance(doc, dict):
        for k, v in doc.items():
            out.update(flatten(v, f"{prefix}{k}."))
    elif isinstance(doc, list):
        if all(isinstance(v, int | float) and not isinstance(v, bool) for v in doc):
            for i, v in enumerate(doc):
                out[f"{prefix}{i}"] = float(v)
    elif isinstance(doc, int | float) and not isinstance(doc, bool):
        out[prefix.rstrip(".")] = float(doc)
    return out


SKIP_IN_TABLE = ("samples.", "slowest.", "by_rule.", "ruf_by_code.", "loadavg.",
                 "per_design_s.", "coverage_lines.", "meta.")
LOAD_RATIO = 2.0
"""``--compare`` warns when the two scorecards' mean load differs by more than this."""


def _is_rc(key: str) -> bool:
    return "rc" in key.split(".")


def compare(a: dict, b: dict) -> list[tuple[str, float | None, float | None]]:
    """Every metric that differs (exit codes included), by its dotted path."""
    fa, fb = flatten(a), flatten(b)
    keys = sorted(set(fa) | set(fb))
    return [(k, fa.get(k), fb.get(k)) for k in keys
            if fa.get(k) != fb.get(k) and not any(s in f".{k}" for s in SKIP_IN_TABLE)]


def rc_changes(a: dict, b: dict) -> list[tuple[str, float | None, float | None]]:
    """Every exit code that changed, or that is non-zero in either scorecard."""
    fa, fb = flatten(a), flatten(b)
    return [(k, fa.get(k), fb.get(k)) for k in sorted(set(fa) | set(fb))
            if _is_rc(k) and (fa.get(k) != fb.get(k) or fa.get(k) or fb.get(k))]


def mean_load(card: dict) -> float | None:
    """The mean 1-minute load over every reading the scorecard took."""
    loads = []
    for v in card.get("loadavg", {}).values():
        if isinstance(v, list) and v:
            loads.append(v[0])
        elif isinstance(v, dict):
            loads += [v[k][0] for k in ("before", "after") if v.get(k)]
    return round(sum(loads) / len(loads), 2) if loads else None


def comparability(a: dict, b: dict) -> list[str]:
    """Why two scorecards' timings aren't comparable: different xdist workers or build runs
    for a metric, a load more than :data:`LOAD_RATIO` apart, another host."""
    out = []
    fa, fb = flatten(a), flatten(b)
    for k in sorted(set(fa) & set(fb)):
        if k.split(".")[-1] in ("workers", "runs") and fa[k] != fb[k]:
            out.append(f"{k}: {fa[k]:g} vs {fb[k]:g} (timings not comparable)")
    la, lb = mean_load(a), mean_load(b)
    if la and lb and max(la, lb) / max(min(la, lb), 0.01) > LOAD_RATIO:
        out.append(f"mean load {la} vs {lb}: more than {LOAD_RATIO:g}x apart "
                   "(timings not comparable)")
    ha, hb = a.get("meta", {}).get("host"), b.get("meta", {}).get("host")
    if ha and hb and ha != hb:
        out.append(f"host {ha} vs {hb}")
    return out


def _fmt(v) -> str:
    return "-" if v is None else f"{v:g}" if abs(v) >= 0.01 or v == 0 else f"{v:.2e}"


def compare_report(a: dict, b: dict) -> str:
    """The ``--compare`` text: warnings, exit codes, then every metric's delta."""
    lines = [f"WARNING: {w}" for w in comparability(a, b)]
    rcs = rc_changes(a, b)
    if rcs:
        lines.append("EXIT CODES (changed, or non-zero):")
        lines += [f"  !! {k}: {_fmt(va)} -> {_fmt(vb)}" for k, va, vb in rcs]
    rows = [r for r in compare(a, b) if not _is_rc(r[0])]
    width = max([len(k) for k, *_ in rows] + [10])
    lines.append(f"{'metric':{width}}  {'A':>12}  {'B':>12}  {'delta':>12}  {'%':>7}")
    for k, va, vb in rows:
        d = None if va is None or vb is None else vb - va
        pct = "" if d is None or not va else f"{100 * d / va:+.1f}"
        lines.append(f"{k:{width}}  {_fmt(va):>12}  {_fmt(vb):>12}  {_fmt(d):>12}  {pct:>7}")
    return "\n".join(lines)


def print_table(card: dict) -> None:
    rows: list[tuple[str, str]] = []
    if b := card.get("build"):
        for phase in ("cold", "warm"):
            p = b[phase]
            rows.append((f"build {phase} store: wall / CPU (s)",
                         f"{_fmt(p['wall_s'])} / {_fmt(p['cpu_s'])}  (profile: "
                         f"{_fmt(p['internal_wall_s'])}, stages sum {_fmt(p['stages_sum_s'])})"))
            rows.append((f"  stages ({phase})", ", ".join(f"{k} {_fmt(v)}"
                                                          for k, v in p["stages"].items())))
    if t := card.get("tiers"):
        for m, v in t.items():
            if isinstance(v, dict):
                rows.append((f"tier {m}", f"{v['wall_s']} s wall / {v['cpu_s']} CPU-s, "
                                          f"{v.get('tests', 0)} tests, rc {v['rc']}"))
    if q := card.get("quick"):
        cov = f", coverage {q['coverage_pct']} %" if "coverage_pct" in q else ""
        rows.append(("quick tier", f"{q['wall_s']} s wall / {q['cpu_s']} CPU-s, "
                                   f"{q.get('tests', 0)} tests, rc {q['rc']}{cov}"))
    if s := card.get("static"):
        py = s["pyright"]
        rows.append(("pyright errors", f"{py.get('errors')} ({py.get('errors_excl_stub_noise')} "
                                       "without OCP/mujoco stub noise)"))
        rows.append(("ruff RUF findings", str(s["ruff"]["ruf"])))
        rows.append(("CLAUDE.md lines", str(s["docs"]["claude_md_lines"])))
        rows.append(("doc check misses", str(s["docs"]["doc_check_misses"])))
        rows.append(("product lines (spiderpig/*.py)", str(s["sizes"]["product_py_lines"])))
        for f, n in s["sizes"]["large"].items():
            rows.append((f"  > {LARGE} lines", f"{f} {n}"))
        rows.append(("CI", ", ".join(s["ci"].get("jobs", [])) or "none"))
    if g := card.get("gate"):
        rows.append(("identity gate (snapshot, -j 2)", f"{g['wall_s']} s wall, rc {g['rc']}"))
    if f := card.get("full"):
        for k, v in f.items():
            rows.append((f"full suite ({k} cache, -n {v['workers']})",
                         f"{v['wall_s']} s / {v['cpu_s']} CPU-s, {v.get('tests')} tests, "
                         f"rc {v['rc']}"))
    if a := card.get("audit"):
        rows.append(("audit double (cold / warm store)",
                     f"{a['cold']['wall_s']} / {a['warm']['wall_s']} s"))
    rows.append(("load average (start / end)",
                 f"{card['loadavg']['start']} / {card['loadavg']['end']}"))
    width = max(len(k) for k, _ in rows)
    for k, v in rows:
        print(f"{k:{width}}  {v}")


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", type=Path, default=DEFAULT_OUT)
    p.add_argument("--only", default=None,
                   help=f"comma-separated sections ({', '.join(SECTIONS)})")
    p.add_argument("--gate", action="store_true", help="also time the identity gate's snapshot")
    p.add_argument("--full", action="store_true", help="also time the full suite")
    p.add_argument("--cold", action="store_true",
                   help="with --full: the full suite on an empty test cache too")
    p.add_argument("--full-workers", type=int, default=6)
    p.add_argument("--audit", action="store_true", help="also time the double's audit")
    p.add_argument("--runs", type=int, default=3,
                   help="build runs, each an empty store then warm (the medians; default 3)")
    p.add_argument("--no-coverage", action="store_true", help="the quick tier without coverage")
    p.add_argument("--parallel-static", action="store_true",
                   help="run the static checks beside the module tiers (faster; their CPU "
                        "inflates the tiers' times)")
    p.add_argument("--keep", action="store_true", help="keep the work folder (logs, junit)")
    p.add_argument("--compare", nargs=2, type=Path, metavar=("A", "B"),
                   help="print the deltas between two scorecards and exit")
    args = p.parse_args(argv)
    if args.compare:
        a, b = (json.loads(f.read_text()) for f in args.compare)
        print(compare_report(a, b))
        return 0

    sections = (args.only.split(",") if args.only else
                [*DEFAULT_SECTIONS, *(["gate"] if args.gate else []),
                 *(["full"] if args.full else []), *(["audit"] if args.audit else [])])
    unknown = set(sections) - set(SECTIONS)
    if unknown:
        p.error(f"unknown sections {sorted(unknown)}")
    sys.path.insert(0, str(ROOT))
    work = Path(tempfile.mkdtemp(prefix="scorecard-"))
    card: dict = {"meta": _meta(), "loadavg": {"start": _load()}, "sections": sections}
    t0 = time.perf_counter()
    static_thread = None
    try:
        for name in sections:
            before = _load()
            print(f"[scorecard] {name} (load {before[0]}) ...", file=sys.stderr, flush=True)
            if name == "static" and static_thread is not None:
                static_thread.join()
                continue
            if name == "tiers" and "static" in sections and args.parallel_static:
                import threading

                def static():
                    card["static"] = section_static(work)

                static_thread = threading.Thread(target=static)
                static_thread.start()
            s0 = time.perf_counter()
            if name == "build":
                card[name] = section_build(work, args.runs)
            elif name == "tiers":
                card[name] = section_tiers(work)
            elif name == "quick":
                card[name] = section_quick(work, not args.no_coverage)
            elif name == "static":
                card[name] = section_static(work)
            elif name == "gate":
                card[name] = section_gate(work)
            elif name == "full":
                card[name] = section_full(work, args.full_workers, args.cold)
            elif name == "audit":
                card[name] = section_audit(work)
            card["loadavg"][name] = {"before": before, "after": _load(),
                                     "seconds": round(time.perf_counter() - s0, 1)}
        if static_thread is not None:
            static_thread.join()
    finally:
        if args.keep:
            print(f"[scorecard] work folder: {work}", file=sys.stderr)
        else:
            shutil.rmtree(work, ignore_errors=True)
    card["loadavg"]["end"] = _load()
    card["seconds"] = round(time.perf_counter() - t0, 1)
    card["seconds_excl_gate"] = round(card["seconds"] - card["loadavg"].get("gate", {})
                                      .get("seconds", 0.0), 1)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(card, indent=1) + "\n")
    print_table(card)
    print(f"wrote {args.out} ({card['seconds']} s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
