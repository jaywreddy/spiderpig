"""The fabrication cache: a fabricated mechanism kept on disk, so an unchanged design is
never fabricated twice.

:func:`spiderpig.fabricate.fabricate` serves a fabrication from here when a store is in
play (its ``store`` argument, or :func:`serving`): ``spiderpig build``, and
:func:`spiderpig.api.build` of a design in a store. ``tests/cache.py`` keeps the test
suite's fabrications with the same code (:func:`dump_mechanism`, :func:`load_mechanism`,
:func:`locked`, :func:`publish`) in its own folder.

**Layout**: ``<store>/fab/<fab key>-<env tag>/<config key>_<side|robot>_t<t>_<hash>/``
holding ``parts.brep`` (one BinTools compound of every distinct part: exact doubles,
locations and shared sub-shapes kept) and ``mechanism.pickle`` (the rest, every part
``None``; each part's class and attributes: a ``Cylinder`` comes back a ``Cylinder``).

**Keying** (why an entry can't be stale): an entry is a function of

- the code: :func:`spiderpig.keys.fab_key`, the hash of every definition planning and
  fabrication can reach (the import closure, symbol by symbol), and the versions of the
  third-party libraries it imports (build123d, OCP, numpy, sympy, ...);
- the folder's env tag: this format, Python, build123d, OCP and numpy versions again;
- the entry's hash: the template's identity and the side's config
  (:func:`spiderpig.fabricate._key`: linkage, module, phases, proportions, every build
  option), robot or side, the crank angle ``t``, the **plan itself** (layers, top, route,
  heads, gaps, thicknesses, sunk heads, the planner's spec but its time budget: a store's
  plan adopted from another engine, or one a budget cut short, is a different entry), and
  the servo model in play (:func:`servo_state`: parametric, or which pinned CAD files are
  in reach: ``SPIDERPIG_SERVO_CAD``, ``SPIDERPIG_OFFLINE``, ``SPIDERPIG_CAD_CACHE``).

Nothing else in the environment shapes a part (``CLAUDE.md``, Environment).

**Concurrency**: one ``flock`` per entry while it is built (a second process waits, then
loads: a design is fabricated once), written into a temporary folder and renamed into
place (a reader sees a whole entry or none). A cache that can't be read or written never
fails a build: it fabricates (logged).

**Off**: ``SPIDERPIG_FAB_CACHE=off`` (the test suite sets it: its own cache is
``tests/cache.py``). :meth:`spiderpig.store.Store.gc` removes other keys' folders
(:func:`gc`).

What comes back is the caller's own object (loaded from disk on every call): a caller may
change it (``spiderpig build`` adds sheet lines to ``bom_extras``). A loaded mechanism
equals a fresh one on every part's volume, area, centre of mass, box and topology, the
BOM and the DXFs (``tests/test_fabcache.py``); the STL and STEP *texts* of a reloaded part
may differ (``-0.`` vs ``0.``, entity order).
"""

from __future__ import annotations

import contextlib
import contextvars
import copy
import fcntl
import hashlib
import logging
import os
import pickle
import shutil
import sys
import time
import uuid
from collections.abc import Callable, Iterator
from dataclasses import replace
from pathlib import Path

log = logging.getLogger("spiderpig.fabcache")

FORMAT = 2
"""The on-disk format: in the folder's name (a change starts afresh)."""

ENV = "SPIDERPIG_FAB_CACHE"
"""``off`` (``0``, ``false``, ``no``): fabricate every time."""

FOLDER = "fab"
"""Under the store's root, beside ``designs/``."""

_STORE: contextvars.ContextVar = contextvars.ContextVar("spiderpig_fab_store", default=None)


def enabled() -> bool:
    return os.environ.get(ENV, "").strip().lower() not in ("off", "0", "false", "no")


@contextlib.contextmanager
def serving(store) -> Iterator[None]:
    """Within the block, :func:`spiderpig.fabricate.fabricate` serves fabrications from
    ``store``'s cache (a :class:`spiderpig.store.Store`, a path, or ``None``: none)."""
    token = _STORE.set(store)
    try:
        yield
    finally:
        _STORE.reset(token)


def current():
    """The store :func:`serving` set (``None``: no cache)."""
    return _STORE.get()


# ---------------------------------------------------------------------------
# keys
# ---------------------------------------------------------------------------


def env_tag() -> str:
    """This format, the Python, build123d, OCP and numpy versions (8 hex)."""
    from importlib import metadata

    def version(dist: str) -> str:
        try:
            return metadata.version(dist)
        except metadata.PackageNotFoundError:
            return "-"

    text = repr((FORMAT, sys.version_info[:2], version("build123d"), version("cadquery-ocp"),
                 version("numpy")))
    return hashlib.sha256(text.encode()).hexdigest()[:8]


def folder_name() -> str:
    """``<fab key>-<env tag>``: the folder of every entry this code may read."""
    from spiderpig.keys import fab_key

    return f"{fab_key()}-{env_tag()}"


def servo_state(config) -> tuple:
    """What decides the servo's model: ``("parametric",)`` when manufacturer models are off
    (``SPIDERPIG_SERVO_CAD=0``) or the servo has none, else each pinned model's hash and
    whether it is in reach (in the download cache, or downloaded now unless
    ``SPIDERPIG_OFFLINE=1``: what the fabrication itself would do), and
    :func:`spiderpig.servos.model.cad_state` (the download's stat and the content of its
    derived strip record, whose own key holds the strip code's key)."""
    from spiderpig import servos
    from spiderpig.servos import cad as cadlib
    from spiderpig.servos.model import cad_state

    spec = servos.get(config.servo)
    if not spec.cads or not cadlib.cad_enabled():
        return ("parametric",)
    present = tuple((ref.sha256, cadlib.fetch(ref) is not None) for ref in spec.cads)
    state = cad_state(spec)
    if any(have for _, have, _ in state) and not any(rec for *_, rec in state):
        # a fresh model cache: derive the strip record now (the fabrication would),
        # so the entry is named by it and the next process finds it
        from spiderpig.servos.model import servo_part

        servo_part(spec, state=state)
        state = cad_state(spec)
    return ("cad", present, state)


def plan_fingerprint(plan) -> str:
    """Everything of a plan a fabrication reads: the layers, the top, the routes, how the
    heads were placed, the gaps, the thicknesses, the sunk heads and the planner's spec
    (but its time budget, and the head search it asked for: a solved plan's spec names
    the search that found it, the same plan re-made from the store the one it was asked
    for; ``plan.heads`` is how the heads are placed)."""
    spec = replace(plan.spec, max_seconds=0.0, heads="")  # (plan.heads is how they were)
    return repr((
        repr(spec), sorted(plan.layers.items()), plan.top,
        sorted((k, repr(v)) for k, v in plan.choices.items()), plan.heads,
        sorted(plan.gaps.items()), sorted(plan.thick.items()),
        sorted(repr(s) for s in plan.sunk),
    ))


def slug(text: str) -> str:
    return "".join(c if c.isalnum() or c in "._-" else "_" for c in text)


def entry_name(tmpl, config, design, t: float) -> str:
    """``<config key>_<side|robot>_t<t>_<hash>``: the entry of ``config`` fabricated at
    ``t`` from ``design`` (module docstring)."""
    from spiderpig.fabricate import _key

    kind = "robot" if config.robot else "side"
    doc = repr((_key(tmpl, config), bool(config.robot), float(t),
                plan_fingerprint(design.plan), servo_state(config)))
    h = hashlib.sha256(doc.encode()).hexdigest()[:16]
    return slug(f"{config.key}_{kind}_t{float(t)!r}_{h}")


def root_of(store) -> Path | None:
    from spiderpig.store import Store

    st = Store.of(store)
    return None if st is None else st.root / FOLDER


# ---------------------------------------------------------------------------
# serving
# ---------------------------------------------------------------------------


def fabricated(store, tmpl, config, design, t: float, build: Callable[[], object]):
    """``build()``'s mechanism, from ``store``'s cache when there (else built, then kept).
    ``store`` ``None`` or the cache off: ``build()``."""
    root = root_of(store) if store is not None and enabled() else None
    if root is None:
        return build()
    t0 = time.perf_counter()
    try:
        entry = root / folder_name() / entry_name(tmpl, config, design, t)
    except Exception as e:      # noqa: BLE001 - a key we can't compute: no cache
        log.warning("fabrication cache: no key (%s): fabricating", e)
        return build()
    with contextlib.ExitStack() as held:
        try:
            held.enter_context(locked(entry))
        except OSError as e:
            # a read-only store: an entry there was published whole (renamed into place),
            # so it is read without the lock; nothing is written; never a failure
            if entry.is_dir():
                try:
                    return load_mechanism(entry)
                except Exception as e2:     # noqa: BLE001
                    log.warning("%s: unreadable cache entry (%s)", entry, e2)
            log.warning("%s: can't lock (%s): fabricating", entry, e)
            return build()
        if entry.is_dir():
            try:
                mech = load_mechanism(entry)
            except Exception as e:      # noqa: BLE001 - a bad entry is rebuilt, not fatal
                log.warning("%s: unreadable cache entry (%s): fabricating", entry, e)
                shutil.rmtree(entry, ignore_errors=True)
            else:
                with contextlib.suppress(OSError):
                    os.utime(entry)                     # (last used: gc by age)
                log.info("fabrication from the cache (%s, %.2f s)", entry.name,
                         time.perf_counter() - t0)
                return mech
        mech = build()
        publish(entry, lambda d: dump_mechanism(mech, d))
    return mech


def gc(store, older_than: float | None = None) -> list[str]:
    """Remove ``store``'s cache folders of other keys (another engine, another format),
    leftovers of interrupted writes, and with ``older_than`` (seconds) the entries not
    used for that long. The paths removed (relative to the store)."""
    root = root_of(store)
    if root is None or not root.is_dir():
        return []
    here = folder_name()
    gone = []
    cutoff = None if older_than is None else time.time() - older_than
    for d in sorted(root.iterdir()):
        if d.name != here:
            if d.is_dir():
                shutil.rmtree(d, ignore_errors=True)
            else:
                d.unlink(missing_ok=True)
            gone.append(f"{FOLDER}/{d.name}")
            continue
        for e in sorted(d.iterdir()):
            stale = e.name.endswith(".tmp") and time.time() - e.stat().st_mtime > 3600
            old = cutoff is not None and e.is_dir() and e.stat().st_mtime < cutoff
            if stale or old:
                # an old entry goes under its lock, renamed away first (a builder waiting
                # on the lock then finds none and builds); its lock file stays: removing it
                # would let a second builder lock a new file while one holds the old
                with locked(e) if old else contextlib.nullcontext():
                    trash = e.with_name(f".{e.name}.{os.getpid()}.{uuid.uuid4().hex[:8]}"
                                        ".gone.tmp")
                    try:
                        os.rename(e, trash)
                    except OSError:
                        continue
                shutil.rmtree(trash, ignore_errors=True)
                gone.append(f"{FOLDER}/{d.name}/{e.name}")
    return gone


# ---------------------------------------------------------------------------
# locks and atomic writes
# ---------------------------------------------------------------------------


@contextlib.contextmanager
def locked(entry: Path) -> Iterator[None]:
    """An exclusive lock on ``entry`` (a sibling ``.lock`` file) across processes."""
    entry.parent.mkdir(parents=True, exist_ok=True)
    with open(entry.parent / f"{entry.name}.lock", "a+") as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(f, fcntl.LOCK_UN)


def publish(entry: Path, write: Callable[[Path], None], strict: bool = False) -> bool:
    """``write`` a whole entry into a temporary folder, then rename it into place; whether
    it was. A failure is logged and dropped (a cache never fails its caller) unless
    ``strict``."""
    tmp = entry.with_name(f".{entry.name}.{os.getpid()}.{uuid.uuid4().hex[:8]}.tmp")
    try:
        tmp.mkdir(parents=True)
        write(tmp)
        if entry.exists():          # (a run without the lock: the first one stays)
            shutil.rmtree(tmp)
            return False
        os.rename(tmp, entry)
        return True
    except Exception as e:
        shutil.rmtree(tmp, ignore_errors=True)
        if strict:
            raise
        log.warning("%s: not cached (%s)", entry, e)
        return False
    except BaseException:
        shutil.rmtree(tmp, ignore_errors=True)
        raise


# ---------------------------------------------------------------------------
# the format
# ---------------------------------------------------------------------------


_NOT_STATE = ("wrapped", "_NodeMixin__children", "topo_parent")
"""A part's attributes the BREP holds (its shape) or that tie it into a tree; the rest
(``label``, ``color``, ``material``, a primitive's own: a ``Cylinder``'s radius) is
pickled beside it."""


def _class_path(part) -> str:
    cls = type(part)
    return f"{cls.__module__}:{cls.__qualname__}"


def _new_part(class_path: str, shape):
    """A build123d part of the class ``class_path`` names around ``shape``: ``Solid`` /
    ``Compound`` / ``Part`` by their constructor; any other part class (``Cylinder``,
    ``Box``: a constructor of dimensions) made without its constructor, as a ``Part``
    (its attributes come from the pickle)."""
    import importlib

    import build123d
    from build123d.topology import downcast

    module, _, qual = class_path.partition(":")
    try:
        cls = importlib.import_module(module)
        for part in qual.split("."):
            cls = getattr(cls, part)
    except (ImportError, AttributeError):
        cls = build123d.Part
    shape = downcast(shape)
    if cls in (build123d.Solid, build123d.Compound, build123d.Part):
        return cls(shape)
    if isinstance(cls, type) and issubclass(cls, build123d.Part):
        obj = cls.__new__(cls)
        build123d.Part.__init__(obj, shape)
        return obj
    if isinstance(cls, type) and issubclass(cls, build123d.Solid):
        obj = cls.__new__(cls)
        build123d.Solid.__init__(obj, shape)
        return obj
    return build123d.Part(shape)


def dump_mechanism(mech, entry: Path) -> None:
    """``mech`` as ``entry/parts.brep`` (one compound of every distinct part) and
    ``entry/mechanism.pickle`` (the rest, parts ``None``; each part's class and
    attributes)."""
    from OCP.BinTools import BinTools
    from OCP.BRep import BRep_Builder
    from OCP.TopoDS import TopoDS_Compound

    order: dict[int, int] = {}
    shapes, classes, states, index = [], [], [], []
    for b in mech.bodies:
        if b.part is None:
            index.append(None)
            continue
        if id(b.part) not in order:
            order[id(b.part)] = len(shapes)
            shapes.append(b.part.wrapped)
            classes.append(_class_path(b.part))
            state = {k: v for k, v in vars(b.part).items() if k not in _NOT_STATE}
            pickle.dumps(state)         # (fails here, not on load: nothing is dropped)
            states.append(state)
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
        pickle.dump({"mechanism": skeleton, "index": index, "classes": classes,
                     "states": states}, f, protocol=pickle.HIGHEST_PROTOCOL)


def load_mechanism(entry: Path):
    """The mechanism :func:`dump_mechanism` wrote: every part of its own class with its
    attributes, parts shared between bodies shared again."""
    from OCP.BinTools import BinTools
    from OCP.TopoDS import TopoDS_Iterator, TopoDS_Shape

    with open(entry / "mechanism.pickle", "rb") as f:
        doc = pickle.load(f)
    comp = TopoDS_Shape()
    if not BinTools.Read_s(comp, str(entry / "parts.brep")):
        raise OSError(f"BinTools couldn't read {entry / 'parts.brep'}")
    shapes = []
    it = TopoDS_Iterator(comp)
    while it.More():
        shapes.append(it.Value())
        it.Next()
    if len(shapes) != len(doc["classes"]):
        raise ValueError(f"{entry}: {len(shapes)} shapes for {len(doc['classes'])} parts")
    parts = []
    for s, cls, state in zip(shapes, doc["classes"], doc["states"], strict=True):
        part = _new_part(cls, s)
        part.__dict__.update(state)
        parts.append(part)
    mech = doc["mechanism"]
    for b, i in zip(mech.bodies, doc["index"], strict=True):
        b.part = None if i is None else parts[i]
    return mech
