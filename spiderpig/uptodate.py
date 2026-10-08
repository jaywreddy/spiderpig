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
  STL file beside them (the files a build owns: :func:`outputs`). A file whose size,
  modification and change times and inode are the recorded ones isn't read again (a
  change of content changes its ctime, which no one can set); any other is hashed.

The servo model's derived strip record (:func:`spiderpig.servos.cad.prepared_path`) is
one of its files. What the build read (the plan, the servo files) is captured when it
reads it (:func:`inputs`), not after it wrote: a re-plan or a download meanwhile makes
the next build build. The check runs once per command (:func:`check_once`: the CLI's
answer is the build's).
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


def _dists_fingerprint() -> str:
    """Every installed distribution's name and version, hashed; kept on disk by
    ``sys.path`` and its folders' modification times (an install or removal changes
    them), so
    a check reads it in milliseconds instead of listing every distribution (~0.3 s)."""
    import site
    import sys

    sites = [d for d in (*site.getsitepackages(), site.getusersitepackages())
             if os.path.isdir(d)]
    # and every other folder on sys.path (PYTHONPATH, an editable install's .pth): a
    # .dist-info there is listed too
    others = [d for d in sys.path if d and d not in sites and os.path.isdir(d)]
    sig = hashlib.sha256(repr((sys.prefix, sys.version, list(sys.path), [
        (d, os.stat(d).st_mtime_ns) for d in (*sites, *others)])).encode()).hexdigest()
    env = os.environ.get("SPIDERPIG_DIGEST_CACHE", "").strip()
    memo = None
    if env.lower() not in ("off", "0", "false", "no"):
        base = Path(env).expanduser() if env else Path(
            os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache") / "spiderpig" \
            / "engine-version"
        memo = base / f"dists-{sig[:32]}.json"
        try:
            doc = json.loads(memo.read_text())
            if doc.get("signature") == sig:
                return doc["dists"]
        except (OSError, ValueError, KeyError, AttributeError):
            pass
    from importlib import metadata

    dists = sorted({(d.metadata["Name"] or "", d.version) for d in metadata.distributions()})
    fp = hashlib.sha256(json.dumps(dists).encode()).hexdigest()
    if memo is not None:
        try:
            memo.parent.mkdir(parents=True, exist_ok=True)
            tmp = memo.with_name(f".{memo.name}.{os.getpid()}.tmp")
            tmp.write_text(json.dumps({"signature": sig, "dists": fp}))
            os.replace(tmp, memo)
        except OSError:
            pass
    return fp


def build_key(opts: Options) -> str:
    """The engine's sources, the options as given (but ``--out`` / ``--force``), the
    store's folder, the Python and every installed distribution's version (an ezdxf or
    build123d upgrade builds again) (module docstring)."""
    import sys

    from spiderpig.keys import engine_digest

    doc = {"engine": engine_digest(), "argv": opts.argv, "store": str(opts.store),
           "python": sys.version, "dists": _dists_fingerprint()}
    return hashlib.sha256(json.dumps(doc, sort_keys=True).encode()).hexdigest()


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def stamp(path: Path) -> list:
    """``[size, mtime_ns, ctime_ns, inode, sha256]`` of a file."""
    st = Path(path).stat()
    return [st.st_size, st.st_mtime_ns, st.st_ctime_ns, st.st_ino, sha256(path)]


def unchanged(path: Path, want: list) -> bool:
    """Whether ``path`` still is the file :func:`stamp` recorded: its stat the same (not
    read), else its size and SHA-256."""
    st = Path(path).stat()
    if [st.st_size, st.st_mtime_ns, st.st_ctime_ns, st.st_ino] == list(want[:4]):
        return True
    return st.st_size == want[0] and sha256(path) == want[4]


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
    """Each of the servo's pinned CAD files and its derived strip record
    (:func:`spiderpig.servos.cad.prepared_path`): path, whether it is there, size,
    modification time (none when manufacturer models are off)."""
    from spiderpig import servos
    from spiderpig.servos import cad as cadlib
    from spiderpig.servos.model import _strip_key

    if not cadlib.cad_enabled():
        return []
    out = []
    for ref in servos.get(config.servo).cads:
        for p, kind in ((cadlib.cached_path(ref), "model"),
                        (cadlib.prepared_path(ref, _strip_key(ref)), "record")):
            try:
                st = p.stat()
                out.append([str(p), True, st.st_size, st.st_mtime_ns, kind])
            except OSError:
                out.append([str(p), False, None, None, kind])
    return out


def inputs(opts: Options, config) -> dict:
    """What the build of ``config`` reads beside its code, captured when it reads it
    (after the plan, before fabricating): the stored plan, the servo model's files."""
    from dataclasses import replace

    from spiderpig import api
    from spiderpig.config import default_robot

    plan_id = api.resolve(api.spec_of(replace(config, robot=default_robot(config.linkage))),
                          store=None).id
    return {"plan_design": plan_id, "plan_hash": plan_hash(plan_file(opts.store, plan_id)),
            "cad_env": cad_env(), "servo_files": servo_files(config)}


def record_path(opts: Options) -> Path:
    """Where the store keeps the record of the last build into ``opts.out``."""
    out = str(Path(opts.out).resolve())
    return opts.store / "builds" / f"{hashlib.sha256(out.encode()).hexdigest()[:16]}.json"


def forget(opts: Options) -> None:
    """Drop the record of a build into ``opts.out`` (a build does, before writing)."""
    record_path(opts).unlink(missing_ok=True)


def record(opts: Options, key: str, read: dict) -> dict:
    """Record the build into ``opts.out``, every output written: ``key``
    (:func:`build_key`, computed before the build read anything), what it read
    (:func:`inputs`, captured then), every output's stamp and the manifest's. The record,
    as written (atomically)."""
    out = Path(opts.out)
    doc = {
        "out": str(out.resolve()),
        "build_key": key,
        **read,
        "outputs": {str(f.relative_to(out)): stamp(f) for f in outputs(out)},
        "manifest": stamp(out / "manifest.json"),
    }
    path = record_path(opts)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    tmp.write_text(json.dumps(doc, indent=1))
    os.replace(tmp, path)
    return doc


def check(opts: Options, key: str | None = None) -> str | None:
    """Why ``opts.out`` already holds what this build would write, or ``None`` (module
    docstring). ``key``: :func:`build_key` of ``opts``, when known."""
    try:
        cur = json.loads(record_path(opts).read_text())
        if cur.get("out") != str(Path(opts.out).resolve()):
            return None
        manifest = opts.out / "manifest.json"
        doc = json.loads(manifest.read_text())
        if doc.get("written_by") != "spiderpig build" or not unchanged(manifest,
                                                                       cur["manifest"]):
            return None
        if cur.get("build_key") != (key or build_key(opts)):
            return None
        want = cur.get("plan_hash")
        if want is None or plan_hash(plan_file(opts.store, cur["plan_design"])) != want:
            return None
        if cur.get("cad_env") != cad_env():
            return None
        offline = (os.environ.get("SPIDERPIG_OFFLINE", "").strip().lower()
                   in ("1", "true", "yes", "on"))
        for path, present, size, mtime, *kind in cur.get("servo_files", []):
            p = Path(path)
            if not present:
                if p.exists() or (not offline and kind != ["record"]):
                    return None         # a build now would read it, or try to download it
                continue
            st = p.stat()
            if (st.st_size, st.st_mtime_ns) != (size, mtime):
                return None
        listed = cur["outputs"]
        if {str(f.relative_to(opts.out)) for f in outputs(opts.out)} != set(listed):
            return None
        for rel, want_stamp in listed.items():
            if not unchanged(opts.out / rel, want_stamp):
                return None
    except (OSError, KeyError, TypeError, ValueError, AttributeError, IndexError):
        return None
    return f"design {doc.get('design')}, {len(listed)} files unchanged"


class Checked(argparse.Namespace):
    """:func:`check_once`'s answer: ``skip`` (said on stdout), ``opts`` (``None``: the
    check doesn't apply) and ``key`` (the build key, for the build's record)."""

    skip: bool
    opts: Options | None
    key: str | None


_ANSWERS: dict[tuple, Checked] = {}


def check_once(argv: list[str]) -> Checked:
    """The check for ``spiderpig build argv``, done once per command: the CLI's answer is
    kept for the build it then runs (:func:`take`), which neither checks nor keys again.
    Says on stdout when there is nothing to do."""
    opts = preparse(list(argv))
    key = build_key(opts) if opts is not None else None
    why = None if opts is None or opts.force else check(opts, key)
    if why is not None:
        assert opts is not None  # check() ran, so there were options to check
        print(f"{opts.out} is up to date ({why}): nothing to do (--force rebuilds)")
    ans = Checked(skip=why is not None, opts=opts, key=key)
    if not ans.skip:
        _ANSWERS[tuple(argv)] = ans
    return ans


def take(argv: list[str]) -> Checked:
    """The CLI's answer for ``argv`` (consumed), else :func:`check_once` now."""
    return _ANSWERS.pop(tuple(argv), None) or check_once(argv)


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
