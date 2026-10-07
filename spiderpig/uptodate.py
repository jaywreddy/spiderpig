"""``spiderpig build`` into a folder that already holds its outputs does nothing.

A build records in its store (``<store>/builds/<hash of --out>.json``, :func:`record`)
what its outputs are a function of, and every output's size and SHA-256 (its
``manifest.json`` included); the same build again finds them unchanged (:func:`check`)
and says so instead of building (``--force`` builds anyway). The output folder holds
nothing more than before.
This module imports nothing of the engine, so :mod:`spiderpig.cli` asks it before
importing :mod:`spiderpig.build` (the engine's import alone is seconds): an up-to-date
build answers in well under a second.

**When a folder counts as current** (anything else builds):

- the store holds a record of a build into that folder, made after the build wrote
  everything (a build forgets it before writing anything: an interrupted build never
  counts), and the folder's manifest is still that build's (an export since, or any
  other writer, changes its hash);
- the same **build key**: every engine source (:func:`spiderpig.keys.engine_digest`:
  the code of the whole package but its front-ends, docstrings stripped, and the package
  and Python versions: any engine edit rebuilds, served from the fabrication cache when
  the parts' code didn't change), every installed distribution's version, the options
  exactly as given but ``--out`` and ``--force``, and the store's folder;
- the store's **plan** is the one the build read (``designs/<id>/plan.json``, less when it
  was written): a re-planned or removed design builds again;
- the **servo model**: ``SPIDERPIG_SERVO_CAD``, ``SPIDERPIG_OFFLINE`` and
  ``SPIDERPIG_CAD_CACHE`` as they were, each pinned CAD file as it was (path, size,
  modification time), and none missing that a build could download now;
- **every output** there with its size and SHA-256, and no other cut, print, STEP or
  STL file beside them (the files a build owns: :func:`outputs`).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path

GENERATED = (".dxf", ".stl", ".csv")
"""What a build or an export writes under ``laser/`` and ``print/`` (the DXFs, the STLs,
their ``parts.csv`` / ``order.csv`` / ``<name>_sheet_parts.csv``)."""

TOP_OUTPUTS = ("ORDER.md", "bom.csv", "bom.md", "bom.json")
CAD_ENV = ("SPIDERPIG_SERVO_CAD", "SPIDERPIG_OFFLINE", "SPIDERPIG_CAD_CACHE")
_OURS = ("--out", "--store", "--force")
STORE_ENV, DEFAULT_ROOT = "SPIDERPIG_STORE", ".spiderpig"
""":mod:`spiderpig.store`'s (that module imports the engine; ``tests/test_uptodate.py``
checks they agree)."""


class Options(argparse.Namespace):
    """What the check reads off the command line (:func:`preparse`)."""

    out: Path
    store: Path
    force: bool
    argv: list[str]


def preparse(argv: list[str]) -> Options | None:
    """``--out``, ``--store`` and ``--force`` read without the engine (spelled in full,
    ``--x VALUE`` or ``--x=VALUE``), the other options as given but the profiler's
    (``--profile``, ``--profile-json FILE``); ``None`` when the check doesn't apply
    (``--list``, ``--help``, an abbreviation of one of those three: argparse reads it)."""
    opts = Options(out=Path("build"), store=None, force=False, argv=[])
    i = 0
    while i < len(argv):
        a = argv[i]
        name, eq, value = a.partition("=")
        if a in ("--list", "-h", "--help"):
            return None
        if a == "--force":
            opts.force = True
        elif a == "--profile":
            pass                    # (the stage timings: no output depends on them)
        elif name == "--profile-json":
            i += 0 if eq else 1
        elif name in ("--out", "--store"):
            if not eq:
                if i + 1 >= len(argv):
                    return None
                value = argv[i + 1]
                i += 1
            setattr(opts, name[2:], Path(value))
        elif name.startswith("--") and len(name) > 2 and any(k.startswith(name) for k in _OURS):
            return None
        else:
            opts.argv.append(a)
        i += 1
    opts.store = Path(opts.store or os.environ.get(STORE_ENV) or DEFAULT_ROOT).resolve()
    return opts


def build_key(opts: Options) -> str:
    """The engine's sources, the options as given (but ``--out`` / ``--force``), the
    store's folder, the Python and every installed distribution's version (an ezdxf or
    build123d upgrade builds again) (module docstring)."""
    import sys
    from importlib import metadata

    from spiderpig.keys import engine_digest

    dists = sorted({(d.metadata["Name"] or "", d.version) for d in metadata.distributions()})
    doc = {"engine": engine_digest(), "argv": opts.argv, "store": str(opts.store),
           "python": sys.version, "dists": dists}
    return hashlib.sha256(json.dumps(doc, sort_keys=True).encode()).hexdigest()


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def outputs(out: Path) -> list[Path]:
    """The files a build owns in ``out``: what it writes under ``laser/`` and ``print/``
    (:data:`GENERATED`), any STEP or STL beside them, the BOM and ``ORDER.md``."""
    out = Path(out)
    files = [f for d in ("laser", "print") if (out / d).is_dir()
             for f in sorted((out / d).rglob("*"))
             if f.is_file() and f.suffix.lower() in GENERATED]
    if out.is_dir():
        files += [f for f in sorted(out.iterdir()) if f.is_file() and (
            f.name in TOP_OUTPUTS or f.suffix.lower() in (".step", ".stp", ".stl"))]
    return files


def plan_file(store: Path, design_id: str) -> Path:
    return Path(store) / "designs" / design_id / "plan.json"


def plan_hash(path: Path) -> str | None:
    """A stored plan (what a build re-makes its plan from), less when it was written and
    how long it took; ``None`` when there is none."""
    try:
        doc = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None
    if not isinstance(doc, dict):
        return None
    for k in ("written_at", "seconds"):
        doc.pop(k, None)
    return hashlib.sha256(json.dumps(doc, sort_keys=True).encode()).hexdigest()


def cad_env() -> dict:
    return {k: os.environ.get(k) for k in CAD_ENV}


def servo_files(config) -> list[list]:
    """Each of the servo's pinned CAD files: its path in the download cache, whether it
    is there, its size and modification time (none when manufacturer models are off)."""
    from spiderpig import servos
    from spiderpig.servos import cad as cadlib

    if not cadlib.cad_enabled():
        return []
    out = []
    for ref in servos.get(config.servo).cads:
        p = cadlib.cached_path(ref)
        try:
            st = p.stat()
            out.append([str(p), True, st.st_size, st.st_mtime_ns])
        except OSError:
            out.append([str(p), False, None, None])
    return out


def record_path(opts: Options) -> Path:
    """Where the store keeps the record of the last build into ``opts.out``."""
    out = str(Path(opts.out).resolve())
    return opts.store / "builds" / f"{hashlib.sha256(out.encode()).hexdigest()[:16]}.json"


def forget(opts: Options) -> None:
    """Drop the record of a build into ``opts.out`` (a build does, before writing)."""
    record_path(opts).unlink(missing_ok=True)


def record(opts: Options, config, key: str) -> dict:
    """Record the build of ``config`` into ``opts.out``, every output written: ``key``
    (:func:`build_key`, computed before the build read anything), the stored plan it read,
    the servo model's files, every output's size and SHA-256 and the manifest's. The
    record, as written (atomically)."""
    from dataclasses import replace

    from spiderpig import api
    from spiderpig.config import default_robot

    plan_id = api.resolve(api.spec_of(replace(config, robot=default_robot(config.linkage))),
                          store=None).id
    out = Path(opts.out)
    doc = {
        "out": str(out.resolve()),
        "build_key": key,
        "plan_design": plan_id,
        "plan_hash": plan_hash(plan_file(opts.store, plan_id)),
        "cad_env": cad_env(),
        "servo_files": servo_files(config),
        "outputs": {str(f.relative_to(out)): [f.stat().st_size, sha256(f)]
                    for f in outputs(out)},
        "manifest": sha256(out / "manifest.json"),
    }
    path = record_path(opts)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    tmp.write_text(json.dumps(doc, indent=1))
    os.replace(tmp, path)
    return doc


def check(opts: Options) -> str | None:
    """Why ``opts.out`` already holds what this build would write, or ``None`` (module
    docstring)."""
    try:
        cur = json.loads(record_path(opts).read_text())
        if cur.get("out") != str(Path(opts.out).resolve()):
            return None
        manifest = opts.out / "manifest.json"
        doc = json.loads(manifest.read_text())
        if doc.get("written_by") != "spiderpig build" or sha256(manifest) != cur["manifest"]:
            return None
        if cur.get("build_key") != build_key(opts):
            return None
        want = cur.get("plan_hash")
        if want is None or plan_hash(plan_file(opts.store, cur["plan_design"])) != want:
            return None
        if cur.get("cad_env") != cad_env():
            return None
        offline = (os.environ.get("SPIDERPIG_OFFLINE", "").strip().lower()
                   in ("1", "true", "yes", "on"))
        for path, present, size, mtime in cur.get("servo_files", []):
            p = Path(path)
            if not present:
                if p.exists() or not offline:
                    return None         # a build now would read it, or try to download it
                continue
            st = p.stat()
            if (st.st_size, st.st_mtime_ns) != (size, mtime):
                return None
        listed = cur["outputs"]
        if {str(f.relative_to(opts.out)) for f in outputs(opts.out)} != set(listed):
            return None
        for rel, (size, digest) in listed.items():
            f = opts.out / rel
            if f.stat().st_size != size or sha256(f) != digest:
                return None
    except (OSError, KeyError, TypeError, ValueError, AttributeError):
        return None
    return f"design {doc.get('design')}, {len(listed)} files unchanged"


def skip(argv: list[str]) -> bool:
    """``spiderpig build argv`` has nothing to do (said on stdout): its folder is current
    and ``--force`` isn't given."""
    opts = preparse(list(argv))
    if opts is None or opts.force:
        return False
    why = check(opts)
    if why is None:
        return False
    print(f"{opts.out} is up to date ({why}): nothing to do (--force rebuilds)")
    return True
