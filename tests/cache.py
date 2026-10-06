"""The test suite's caches: fabrications, plans, stores and recorded fixtures.

The contract the module packages code against (``docs/agentlib/TESTING.md``)::

    CACHE_ROOT: Path       # $SPIDERPIG_TEST_CACHE, else $XDG_CACHE_HOME/spiderpig/test-cache
    CACHE_DIR: Path        # CACHE_ROOT / "<engine_version>-<env tag>" (computed on first use)
    cached_design(cfg) -> (tmpl, SideDesign)              # design_side, plan seeded from disk
    cached_side(cfg, t=1.0, *, fresh=False) -> Mechanism  # fabricate_side, BREP-cached
    cached_robot(cfg, t=1.0, *, fresh=False) -> Mechanism # fabricate(robot=True), BREP-cached
    recorded(module, name, make) -> data                  # tests/fixtures/<module>/<name>.json
    assert_current(module, name, make) -> data            # the slow currency test of a fixture
    prebuilt_store(cfg, tmp_path, t=1.0) -> Store         # a copy of a planned + built store

**Honest by construction.** Every entry lives under ``CACHE_DIR``, whose name is
:func:`spiderpig.design.engine_version` (every engine source, docstrings stripped, plus the
planner's defaults) and a tag of what else shapes a part (this cache's format, Python,
build123d and OCP versions). Any engine edit is a new directory: the first run after it
builds each entry once, nothing is ever invalidated in place. A plan from disk is not
trusted either: it is re-made and re-verified exactly as the engine re-makes the robot's
side (:func:`spiderpig.fabricate._reuse`: ``problem.plan`` + ``verify_plan``), and must
give the recorded gaps and thicknesses back.

**Where.** A user cache directory, not the checkout: the key is the engine version, so
worktrees with the same engine sources (every worktree branched from one master commit,
until it edits ``spiderpig/``) share their entries, and worktrees with different ones
never meet. ``SPIDERPIG_TEST_CACHE=<dir>`` moves it, ``SPIDERPIG_TEST_CACHE=off`` (or
``pytest --no-test-cache``) turns the disk cache off (each process then builds what it
needs once, as before). Engine directories nobody used for ``SPIDERPIG_TEST_CACHE_DAYS``
(14) days are removed at the start of a session (:func:`prune`).

**xdist-safe.** One ``fcntl.flock`` per entry while it is built (a second worker waits
for the first, then loads), entries written to a temporary name and renamed into place
(a reader sees a whole entry or none), fixtures and plans written with ``os.replace``.

**Shapes.** A fabricated mechanism is one BinTools BREP of all its parts (one compound:
every double exact, locations and shared sub-shapes kept, triangulations included as
``workers.dump_shape`` writes them) and a pickle of the rest with every ``part`` set to
``None``. Parts come back as ``Solid`` / ``Compound`` / ``Part`` (a ``Box`` or ``Cylinder``,
or any other build123d class, as a generic ``Part``: ``workers.load_shape`` rebuilds by
class name and fails on those). Pickling build123d shapes directly doesn't read back.
A BREP round trip keeps every boolean, volume and box; a test that compares meshes, STL or
DXF text against a fresh build should still use ``fresh=True`` (``docs/agentlib/PERF.md``
saw a BREP round trip change a later mesh once).

Everything returned is shared by the process (one object per key, as the session
fixtures always were): never mutate it.
"""

from __future__ import annotations

import contextlib
import copy
import fcntl
import hashlib
import json
import logging
import os
import pickle
import shutil
import sys
import time
import uuid
import warnings
from collections.abc import Callable, Iterator
from dataclasses import replace
from pathlib import Path
from typing import Any

log = logging.getLogger("spiderpig.tests.cache")

FORMAT = 1
"""This module's on-disk format: part of the directory name, so a change of it starts afresh."""

CACHE_ENV = "SPIDERPIG_TEST_CACHE"      # a directory, or "off" / "0"
DAYS_ENV = "SPIDERPIG_TEST_CACHE_DAYS"  # prune engine directories unused this long (14)
REGEN_ENV = "SPIDERPIG_REGEN"           # "1": recorded() rewrites (what `pytest --regen` sets)

REPO = Path(__file__).resolve().parents[1]
FIXTURES = REPO / "tests" / "fixtures"

ENABLED = True
"""The disk cache on (``conftest`` turns it off for ``--no-test-cache``)."""
REGEN = False
"""``recorded`` / ``assert_current`` rewrite their fixtures (``pytest --regen``)."""

_DIR: list[Path] = []
_MEMO: dict[tuple, Any] = {}


def _root() -> Path:
    env = os.environ.get(CACHE_ENV, "").strip()
    if env and env.lower() not in ("off", "0", "false", "no"):
        return Path(env).expanduser()
    base = os.environ.get("XDG_CACHE_HOME") or str(Path.home() / ".cache")
    return Path(base) / "spiderpig" / "test-cache"


def enabled() -> bool:
    """Whether entries are read from and written to disk: ``ENABLED``, no
    ``SPIDERPIG_TEST_CACHE=off``, and servos drawn parametrically (the tests' own
    ``_offline``; a manufacturer's model depends on a download, not on the engine)."""
    env = os.environ.get(CACHE_ENV, "").strip().lower()
    return ENABLED and env not in ("off", "0", "false", "no") and _parametric_servos()


def _parametric_servos() -> bool:
    from spiderpig.servos import cad as cadlib

    if not cadlib.cad_enabled():
        return True
    if not cadlib.offline():
        return False
    d = cadlib.cache_dir()
    return not d.is_dir() or not any(d.iterdir())


def env_tag() -> str:
    """What else shapes a cached part than the engine's sources: this cache's format, the
    Python, build123d and OCP versions."""
    from importlib import metadata

    def version(dist: str) -> str:
        try:
            return metadata.version(dist)
        except metadata.PackageNotFoundError:
            return "-"

    text = repr((FORMAT, sys.version_info[:2], version("build123d"), version("cadquery-ocp"),
                 version("numpy")))
    return hashlib.sha256(text.encode()).hexdigest()[:8]


def cache_dir() -> Path:
    """``CACHE_ROOT / "<engine_version>-<env tag>"`` (computed once per process)."""
    if not _DIR:
        from spiderpig.design import engine_version

        _DIR.append(_root() / f"{engine_version()}-{env_tag()}")
    return _DIR[0]


def __getattr__(name: str):
    if name == "CACHE_DIR":
        return cache_dir()
    if name == "CACHE_ROOT":
        return _root()
    raise AttributeError(name)


def prune(days: float | None = None) -> list[Path]:
    """Remove the engine directories under ``CACHE_ROOT`` nobody used for ``days``
    (``SPIDERPIG_TEST_CACHE_DAYS``, 14) and mark this one used; the removed paths."""
    days = float(os.environ.get(DAYS_ENV, "14")) if days is None else days
    root, here = _root(), cache_dir()
    here.mkdir(parents=True, exist_ok=True)
    os.utime(here)
    gone = []
    cutoff = time.time() - days * 86400
    for d in root.iterdir() if root.is_dir() else ():
        if d.is_dir() and d != here and d.stat().st_mtime < cutoff:
            shutil.rmtree(d, ignore_errors=True)
            gone.append(d)
    return gone


# ---------------------------------------------------------------------------
# locks and atomic writes
# ---------------------------------------------------------------------------


@contextlib.contextmanager
def _locked(entry: Path) -> Iterator[None]:
    """An exclusive lock on ``entry`` (a sibling ``.lock`` file) across processes."""
    entry.parent.mkdir(parents=True, exist_ok=True)
    with open(entry.parent / f"{entry.name}.lock", "a+") as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(f, fcntl.LOCK_UN)


def _write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.{uuid.uuid4().hex[:8]}.tmp")
    tmp.write_text(text)
    os.replace(tmp, path)


def _publish(entry: Path, write: Callable[[Path], None]) -> None:
    """``write`` a whole entry into a temporary directory, then rename it into place."""
    tmp = entry.with_name(f".{entry.name}.{os.getpid()}.{uuid.uuid4().hex[:8]}.tmp")
    tmp.mkdir(parents=True)
    try:
        write(tmp)
        if entry.exists():          # (a run without the lock: the first one stays)
            shutil.rmtree(tmp)
        else:
            os.rename(tmp, entry)
    except BaseException:
        shutil.rmtree(tmp, ignore_errors=True)
        raise


def _slug(text: str) -> str:
    return "".join(c if c.isalnum() or c in "._-" else "_" for c in text)


# ---------------------------------------------------------------------------
# plans
# ---------------------------------------------------------------------------


class _Seed:
    """A recorded plan in the shape :func:`spiderpig.fabricate._reuse` reads."""

    def __init__(self, doc: dict):
        from spiderpig.construction.crank import CrankRoute, Run

        self.doc = doc
        self.layers = {k: int(v) for k, v in doc["layers"].items()}
        self.top = int(doc["top"])
        route = doc.get("route")
        self.choices = {} if route is None else {
            "crank": CrankRoute(tuple(Run(r["at"], int(r["lo"]), int(r["hi"]))
                                      for r in route["runs"]), bool(route["bearing"]))}
        self.heads = doc["heads"]
        self.optimal, self.proof, self.cost = bool(doc["optimal"]), doc["proof"], doc["cost"]


def plan_doc(plan) -> dict | None:
    """A plan as the JSON the cache seeds from (``None``: a route it can't write)."""
    from spiderpig.construction.crank import CrankRoute

    choices = dict(plan.choices)
    route = choices.pop("crank", None)
    if choices or (route is not None and not isinstance(route, CrankRoute)):
        return None
    return {
        "layers": dict(sorted(plan.layers.items())), "top": plan.top,
        "route": None if route is None else {
            "runs": [{"at": r.at, "lo": r.lo, "hi": r.hi} for r in route.runs],
            "bearing": route.bearing},
        "heads": plan.heads, "optimal": plan.optimal, "proof": plan.proof, "cost": plan.cost,
        "gaps": {str(k): v for k, v in sorted(plan.gaps.items())},
        "thick": {str(k): v for k, v in sorted(plan.thick.items())},
    }


def _plan_path(cfg) -> Path:
    return cache_dir() / "plans" / f"{_slug(cfg.key)}.json"


def _hint_config(cfg):
    """The config :func:`spiderpig.fabricate._leg_hint` plans first (``None``: a single)."""
    if cfg.module == "single":
        return None
    return replace(cfg, module="single", phases=None, robot=False)


def cached_design(cfg):
    """``(tmpl, SideDesign)`` of ``cfg``'s side (``robot`` ignored), as
    :func:`spiderpig.fabricate.design_side` gives it, its plan (and the leg hint's) seeded
    from ``CACHE_DIR/plans/<cfg.key>.json`` when there (re-made and verified, else solved
    again), written there after a solve. One object per config per process."""
    from spiderpig import fabricate

    cfg = replace(cfg, robot=False)
    memo = ("design", cfg)
    if memo in _MEMO:
        return _MEMO[memo]
    tmpl = fabricate.template_for(cfg)
    if not enabled():
        _MEMO[memo] = (tmpl, fabricate.design_side(tmpl, cfg))
        return _MEMO[memo]
    pairs = [(tmpl, cfg)]
    if (hint := _hint_config(cfg)) is not None:
        pairs.append((fabricate.template_for(hint), hint))
    with _locked(_plan_path(cfg)):
        seeds = {}
        for t_, c in pairs:
            key = fabricate._key(t_, c)
            path = _plan_path(c)
            if key not in fabricate._LAYOUTS and path.is_file():
                try:
                    seeds[key] = fabricate._LAYOUTS[key] = _Seed(json.loads(path.read_text()))
                except (ValueError, KeyError, TypeError) as e:
                    log.warning("%s: unreadable cached plan (%s)", path, e)
        design = fabricate.design_side(tmpl, cfg)
        seed = seeds.get(fabricate._key(tmpl, cfg))
        if seed is not None and fabricate._LAYOUTS.get(fabricate._key(tmpl, cfg)) is seed:
            got = plan_doc(design.plan)
            if got is None or any(got[k] != seed.doc.get(k) for k in ("gaps", "thick", "heads")):
                # re-made, the plan's z came out otherwise: never serve it, solve afresh
                log.warning("%s: the cached plan re-makes differently: solving", cfg.key)
                for key in seeds:
                    fabricate._LAYOUTS.pop(key, None)
                    fabricate._DESIGNS.pop(key, None)
                design = fabricate.design_side(tmpl, cfg)
        for t_, c in pairs:
            plan = fabricate._LAYOUTS.get(fabricate._key(t_, c))
            if plan is None or isinstance(plan, _Seed):
                continue
            doc = plan_doc(plan)
            path = _plan_path(c)
            if doc is not None and (not path.is_file() or json.loads(path.read_text()) != doc):
                _write_text(path, json.dumps(doc, indent=1))
    _MEMO[memo] = (tmpl, design)
    return _MEMO[memo]


def seed_plan(cfg) -> bool:
    """Seed ``fabricate._LAYOUTS`` with ``cfg``'s cached plan (no design is made): a later
    :func:`spiderpig.fabricate.design_side` of it re-makes and verifies that plan instead
    of searching. ``False`` when the cache holds none (or is off)."""
    from spiderpig import fabricate

    cfg = replace(cfg, robot=False)
    path = _plan_path(cfg) if enabled() else None
    if path is None or not path.is_file():
        return False
    key = fabricate._key(fabricate.template_for(cfg), cfg)
    fabricate._LAYOUTS.setdefault(key, _Seed(json.loads(path.read_text())))
    return True


# ---------------------------------------------------------------------------
# fabricated mechanisms
# ---------------------------------------------------------------------------


_REBUILT = {"Solid", "Compound", "Part"}
"""build123d classes rebuilt as themselves from a bare shape; any other loads as ``Part``."""


def dump_mechanism(mech, entry: Path) -> None:
    """``mech`` as ``entry/parts.brep`` (one compound of every distinct part) and
    ``entry/mechanism.pickle`` (the rest, parts ``None``)."""
    from OCP.BinTools import BinTools
    from OCP.BRep import BRep_Builder
    from OCP.TopoDS import TopoDS_Compound

    order: dict[int, int] = {}
    shapes, classes, index = [], [], []
    for b in mech.bodies:
        if b.part is None:
            index.append(None)
            continue
        if id(b.part) not in order:
            order[id(b.part)] = len(shapes)
            shapes.append(b.part.wrapped)
            classes.append(type(b.part).__name__)
        index.append(order[id(b.part)])
    comp = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(comp)
    for s in shapes:
        builder.Add(comp, s)
    if not BinTools.Write_s(comp, str(entry / "parts.brep")):
        raise OSError(f"BinTools couldn't write {entry / 'parts.brep'}")
    skeleton = copy.copy(mech)
    skeleton.bodies = [replace(b, part=None) for b in mech.bodies]
    with open(entry / "mechanism.pickle", "wb") as f:
        pickle.dump({"mechanism": skeleton, "index": index, "classes": classes}, f,
                    protocol=pickle.HIGHEST_PROTOCOL)


def load_mechanism(entry: Path):
    """The mechanism :func:`dump_mechanism` wrote."""
    import build123d
    from build123d.topology import downcast
    from OCP.BinTools import BinTools
    from OCP.TopoDS import TopoDS_Iterator, TopoDS_Shape

    with open(entry / "mechanism.pickle", "rb") as f:
        doc = pickle.load(f)
    comp = TopoDS_Shape()
    BinTools.Read_s(comp, str(entry / "parts.brep"))
    shapes = []
    it = TopoDS_Iterator(comp)
    while it.More():
        shapes.append(it.Value())
        it.Next()
    if len(shapes) != len(doc["classes"]):
        raise ValueError(f"{entry}: {len(shapes)} shapes for {len(doc['classes'])} parts")
    parts = [getattr(build123d, cls if cls in _REBUILT else "Part")(downcast(s))
             for s, cls in zip(shapes, doc["classes"], strict=True)]
    mech = doc["mechanism"]
    for b, i in zip(mech.bodies, doc["index"], strict=True):
        b.part = None if i is None else parts[i]
    return mech


def _mechanism(kind: str, cfg, t: float, build: Callable[[], Any], fresh: bool):
    memo = (kind, cfg, float(t))
    if fresh:
        return build()
    if memo in _MEMO:
        return _MEMO[memo]
    if not enabled():
        _MEMO[memo] = build()
        return _MEMO[memo]
    entry = cache_dir() / "mech" / _slug(f"{cfg.key}_{kind}_t{float(t)!r}")
    mech = None
    with _locked(entry):
        if entry.is_dir():
            try:
                mech = load_mechanism(entry)
            except Exception as e:      # noqa: BLE001 - a bad entry is rebuilt, not fatal
                log.warning("%s: unreadable cache entry (%s): rebuilding", entry, e)
                shutil.rmtree(entry, ignore_errors=True)
        if mech is None:
            mech = build()
            _publish(entry, lambda d: dump_mechanism(mech, d))
    _MEMO[memo] = mech
    return mech


def cached_side(cfg, t: float = 1.0, *, fresh: bool = False):
    """One side of ``cfg`` fabricated at crank angle ``t``
    (:func:`spiderpig.fabricate.fabricate_side` of :func:`cached_design`), from
    ``CACHE_DIR/mech/`` when there. ``fresh``: built now, nothing read or written."""
    from spiderpig.fabricate import fabricate_side

    cfg = replace(cfg, robot=False)

    def build():
        tmpl, d = cached_design(cfg)
        return fabricate_side(d, tmpl.freeze_at(t))

    return _mechanism("side", cfg, t, build, fresh)


def cached_robot(cfg, t: float = 1.0, *, fresh: bool = False):
    """The robot of ``cfg`` (both sides and the chassis, :func:`spiderpig.fabricate.fabricate`
    with ``robot=True``) at crank angle ``t``, from ``CACHE_DIR/mech/`` when there.
    ``fresh``: built now, nothing read or written. A mechanism has no robot
    (:class:`spiderpig.config.ParamError`)."""
    from spiderpig.fabricate import fabricate

    cfg = replace(cfg, robot=True)

    def build():
        tmpl, _ = cached_design(cfg)
        return fabricate(tmpl, cfg, t)

    return _mechanism("robot", cfg, t, build, fresh)


# ---------------------------------------------------------------------------
# a prebuilt store
# ---------------------------------------------------------------------------


def prebuilt_store(cfg, tmp_path: Path, t: float = 1.0):
    """A :class:`spiderpig.store.Store` in ``tmp_path / "store"`` holding ``cfg``'s design
    resolved (:func:`spiderpig.api.spec_of`, so ``sides`` follows ``cfg.robot``), planned
    and built at ``t`` (``build/manifest.json`` + STEP parts): a copy of
    ``CACHE_DIR/stores/<cfg.key>_t<t>/``, made once per engine version. Its one design is
    ``store.ids()[0]``; ``api.load(id, store)`` then ``api.build(design, t)`` serves the
    parts from STEP. The copy is the test's own to change."""
    from spiderpig import api
    from spiderpig.store import Store

    dest = Path(tmp_path) / "store"

    def make(root: Path) -> None:
        design = api.resolve(api.spec_of(cfg), Store.of(root))
        rep = api.build(design, t)
        if not rep.ok:
            raise RuntimeError(f"prebuilt_store({cfg.key}): the build failed: "
                               + "; ".join(f.message for f in rep.failures[:2]))

    if not enabled():
        make(dest)
        return Store.of(dest)
    entry = cache_dir() / "stores" / _slug(f"{cfg.key}_t{float(t)!r}")
    with _locked(entry):
        if not entry.is_dir():
            _publish(entry, make)
    shutil.copytree(entry, dest)
    return Store.of(dest)


# ---------------------------------------------------------------------------
# recorded fixtures
# ---------------------------------------------------------------------------


class StaleFixture(UserWarning):
    """A recorded fixture written under another engine version (``mise run
    test-fixtures`` regenerates; the data is still used)."""


def fixture_path(module: str, name: str) -> Path:
    return FIXTURES / module / f"{name}.json"


def _jsonable(data) -> Any:
    """``data`` as it reads back from the fixture (tuples lists, keys strings)."""
    return json.loads(json.dumps(data, sort_keys=True, allow_nan=False))


def _generator() -> str:
    return os.environ.get("PYTEST_CURRENT_TEST", "").rsplit(" (", 1)[0] or sys.argv[0]


def _write_fixture(path: Path, data) -> None:
    from spiderpig.design import engine_version

    doc = {"engine_version": engine_version(), "generator": _generator(), "data": data}
    _write_text(path, json.dumps(doc, indent=1, sort_keys=True, allow_nan=False) + "\n")


def _regen() -> bool:
    return REGEN or os.environ.get(REGEN_ENV, "") == "1"


def read_fixture(module: str, name: str) -> dict | None:
    """The whole document of a recorded fixture (``None`` when there is none)."""
    path = fixture_path(module, name)
    return json.loads(path.read_text()) if path.is_file() else None


def recorded(module: str, name: str, make: Callable[[], Any]) -> Any:
    """The data of ``tests/fixtures/<module>/<name>.json``: a fast test's input, valid
    whatever the engine version. Written from ``make()`` when missing or with ``--regen``
    (``{"engine_version", "generator", "data"}``); one written under another engine
    version is used and warns (:class:`StaleFixture`), never skips. The data is what JSON
    gives back, fresh or read."""
    from spiderpig.design import engine_version

    path = fixture_path(module, name)
    if _regen() or not path.is_file():
        data = _jsonable(make())
        _write_fixture(path, data)
        return data
    doc = json.loads(path.read_text())
    if doc.get("engine_version") != engine_version():
        warnings.warn(StaleFixture(
            f"{path.relative_to(REPO)} was recorded under engine {doc.get('engine_version')}, "
            f"this is {engine_version()}: `mise run test-fixtures` checks and rewrites it"),
            stacklevel=2)
    return doc["data"]


def assert_current(module: str, name: str, make: Callable[[], Any]) -> Any:
    """The currency test of a recorded fixture (mark it ``slow`` and ``fixture_regen``):
    ``make()`` from the engine must equal the recorded data; with ``--regen`` the fixture
    is rewritten instead. Returns the fresh data."""
    data = _jsonable(make())
    path = fixture_path(module, name)
    if _regen() or not path.is_file():
        _write_fixture(path, data)
        return data
    old = json.loads(path.read_text())["data"]
    if old != data:
        raise AssertionError(
            f"{path.relative_to(REPO)} no longer matches the engine:\n"
            + "\n".join(_diff(old, data)[:40])
            + "\nIf the change is intended: `mise run test-fixtures` (pytest --regen "
              "-m fixture_regen) rewrites it.")
    return data


def _diff(a, b, path: str = "") -> list[str]:
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) | set(b)):
            if k not in a:
                out.append(f"  {path}/{k}: added {b[k]!r:.120}")
            elif k not in b:
                out.append(f"  {path}/{k}: removed {a[k]!r:.120}")
            else:
                out += _diff(a[k], b[k], f"{path}/{k}")
        return out
    if isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        return [d for i, (x, y) in enumerate(zip(a, b, strict=True))
                for d in _diff(x, y, f"{path}[{i}]")]
    return [] if a == b else [f"  {path}: {a!r:.120} -> {b!r:.120}"]


def stale_fixtures() -> list[Path]:
    """Every recorded fixture written under another engine version than this one."""
    from spiderpig.design import engine_version

    out = []
    for p in sorted(FIXTURES.rglob("*.json")):
        try:
            if json.loads(p.read_text()).get("engine_version") != engine_version():
                out.append(p)
        except (ValueError, AttributeError):
            out.append(p)
    return out
