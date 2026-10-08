"""The per-project design store (harness v1, step 2; decision 4): every design, its stage
reports, its parts and its log, as files under one folder.

The store is a cache and a record, never a second source of truth: a design
is its ``id`` (the resolved spec and the engine version), and everything
under ``designs/<id>/`` can be recomputed from ``resolved.json``. Root:
``$SPIDERPIG_STORE``, else ``./.spiderpig`` (git-ignored, created on the
first write); :meth:`Store.default` resolves it, ``store=None`` in
:mod:`spiderpig.api` keeps everything in memory.

Layout of ``designs/<id>/``:

=======================  ============================================================
``spec.json``            the spec as given
``resolved.json``        the resolved spec (every inferred value written in) with the
                         engine version the id was computed under, ``spec_hash``,
                         ``created_at``, ``derived_from`` and the merge ``patch`` from it
``check.json``, ...      one file per stage report (``check``, ``plan``, ``walk``,
                         ``recheck``, ``verify``, ``export``): the report's JSON with
                         ``stage``, ``design``, ``engine_version`` and ``written_at``;
                         ``verify.json`` is the latest level, ``verify.<level>.json`` a
                         copy per level (a quick verify doesn't evict a standard one)
``build/manifest.json``  the build report plus, per part, its pose, colour and file
``build/parts/*.step``   one STEP per distinct part at the manifest's ``t``; a right-side
                         part references its left twin (``same_as``, ``mirror``)
``exports/``             what :func:`spiderpig.api.export` writes by default
``log.jsonl``            one line per operation: op, engine version, seconds, ok, cached
=======================  ============================================================

Beside ``designs/``, ``fab/`` is the fabrication cache (:mod:`spiderpig.fabcache`:
``fab/<fab key>-<env tag>/<entry>/``, a fabricated mechanism per design, plan and crank
angle), which ``spiderpig build`` and :func:`spiderpig.api.build` read; :meth:`Store.gc`
removes other code versions' folders.

Validity: a stage file is used as is when its ``engine_version`` is the
running engine's (:func:`spiderpig.design.engine_version`), except a plan,
which is always re-made through :meth:`stack.StackProblem.plan` and
re-checked with :func:`stack.verify_plan` before use (the closures it
needs are rebuilt from the template, nothing is pickled). A plan from
another engine version is re-verified on fresh sampling and adopted
without its optimality proof, or re-solved when the check fails; the
other stages are recomputed under the new engine. Files are written
atomically, so several users may share one folder.

Concurrency, two kinds of write:

- **Single files** (``spec.json``, ``resolved.json``, a stage report, the log) are written
  atomically (a temp file renamed into place) and take **no lock**: a reader sees the old
  file or the new one, two writers of one report write the same thing, and nothing waits.
  They never create a removed design's folder again (:meth:`Store.write_report` skips a
  design that is gone).
- **The multi-file operations** (a build's manifest and STEP parts, an export's files, and
  removing a design) hold the design's lock (:meth:`Store.lock`: an ``fcntl.flock`` on
  ``designs/<id>/.lock``, re-entrant in the thread that holds it, keyed by the file's real
  path). ``api.build`` and ``api.export`` take it, :meth:`Store.write_build` and
  :meth:`Store.remove` too, and ``mcp.jobs.run_op`` holds it across its operation and the
  reads of what it wrote. :meth:`Store.gc` skips a design whose lock is held (in use).
  :meth:`Store.remove` renames the folder away before deleting it, so a waiter wakes to a
  design that is gone (:class:`DesignRemoved`) and never re-creates its folder.

The order: a thread never waits on a design's lock while holding a lock of its own over
other designs (the MCP server's ``_ENGINE``): the MCP runs only single-file writes under
``_ENGINE``; the builds and exports run in its worker processes.
"""

from __future__ import annotations

import contextlib
import fcntl
import json
import logging
import os
import re
import shutil
import tempfile
import threading
from collections.abc import Iterable, Iterator, Mapping
from datetime import UTC, datetime, timedelta
from pathlib import Path

import numpy as np

from spiderpig.design import Design, design_id, jsonable, now_iso, spec_hash

log = logging.getLogger("spiderpig.store")

STORE_ENV = "SPIDERPIG_STORE"
DEFAULT_ROOT = ".spiderpig"
STAGES = ("check", "plan", "walk", "build", "recheck", "verify", "export")
META_KEYS = ("stage", "design", "engine_version", "written_at")
PROJECT = "project"          # the ``store`` argument's default: the project store

TRASH = ".removed-"     # designs/.removed-<id>-<random>/: a removal in progress (or crashed)
_ID = re.compile(r"[0-9a-f]{16}")   # matched whole (fullmatch: no trailing newline)
_UNSAFE = re.compile(r"[^\w.\-]")


class StoreError(ValueError):
    """A design folder that doesn't hold what its id says."""


class DesignRemoved(StoreError):
    """The design was removed (``gc``) while this waited for its lock."""


def _parse_time(s: str | None) -> datetime | None:
    if not s:
        return None
    try:
        t = datetime.fromisoformat(s)
    except ValueError:
        return None
    return t if t.tzinfo is not None else t.replace(tzinfo=UTC)


def _write_json(path: Path, doc, parents: bool = True) -> Path:
    """Write ``doc`` as JSON atomically (a temp file in the same folder, then a rename);
    ``parents=False``: the folder must exist (``FileNotFoundError`` otherwise)."""
    if parents:
        path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.", suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(jsonable(doc), f, indent=1, allow_nan=False)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise
    return path


def _read_json(path: Path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


class _HeldLock:
    """One design lock file as this process holds it: a thread lock (``flock`` is per
    open file, so threads of one process don't exclude each other through it), the depth
    of re-entry by its owning thread and the open file while held."""

    def __init__(self) -> None:
        self.thread = threading.RLock()
        self.depth = 0
        self.fd: int | None = None


_LOCKS: dict[str, _HeldLock] = {}
_LOCKS_GUARD = threading.Lock()


def _held(path: Path) -> _HeldLock:
    # the real path: one store spelled through a symlink is one lock (else a nested lock
    # by the other spelling would wait on the flock this thread holds)
    key = os.path.realpath(path)
    with _LOCKS_GUARD:
        return _LOCKS.setdefault(key, _HeldLock())


def _flock(path: Path, blocking: bool) -> int | None:
    """An open ``path`` holding an exclusive ``flock`` (``None``: busy, non-blocking).
    ``path``'s folder (the design's) must exist: a design removed before or while this
    waited raises :class:`DesignRemoved` (a lock on its unlinked file excludes no one, and
    its folder is not made again)."""
    while True:
        try:
            fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
        except FileNotFoundError:
            raise DesignRemoved(f"{path.parent.name}: the design was removed") from None
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | (0 if blocking else fcntl.LOCK_NB))
        except BlockingIOError:
            os.close(fd)
            return None
        except BaseException:
            os.close(fd)
            raise
        try:
            same = os.stat(path).st_ino == os.fstat(fd).st_ino
        except FileNotFoundError:
            same = False
        if same:
            return fd
        os.close(fd)        # removed while this waited: open it again (then it's gone)


def report_doc(rep) -> dict:
    """A report's JSON form: its own ``to_dict`` when it has one, else the dataclass."""
    return rep.to_dict() if hasattr(rep, "to_dict") else jsonable(rep)


class Store:
    """One project's designs under ``root`` (module docstring)."""

    def __init__(self, root: str | Path):
        self.root = Path(root)

    def __repr__(self) -> str:
        return f"Store({str(self.root)!r})"

    def __eq__(self, other) -> bool:
        return isinstance(other, Store) and self.root.resolve() == other.root.resolve()

    def __hash__(self) -> int:
        return hash(self.root.resolve())

    @classmethod
    def default(cls) -> Store:
        """The project store: ``$SPIDERPIG_STORE``, else ``./.spiderpig``."""
        return cls(os.environ.get(STORE_ENV) or DEFAULT_ROOT)

    @classmethod
    def of(cls, store) -> Store | None:
        """The store an API argument names: :data:`PROJECT` (the default) is the project
        store, ``None`` no store, a path a store there, a :class:`Store` itself."""
        if store is None or isinstance(store, Store):
            return store
        if store == PROJECT:
            return cls.default()
        return cls(store)

    # -- folders -----------------------------------------------------------------

    @property
    def designs(self) -> Path:
        return self.root / "designs"

    def dir(self, id: str) -> Path:
        if not isinstance(id, str) or not _ID.fullmatch(id):
            raise ValueError(f"not a design id: {id!r}")
        return self.designs / id

    def has(self, id: str) -> bool:
        return (self.dir(id) / "resolved.json").is_file()

    def ids(self) -> list[str]:
        """Every design recorded here (a folder with a ``resolved.json``), oldest first."""
        if not self.designs.is_dir():
            return []
        out = [p.name for p in self.designs.iterdir()
               if _ID.fullmatch(p.name) and (p / "resolved.json").is_file()]
        return sorted(out, key=lambda i: (self.read_design(i) or {}).get("created_at", ""))

    def exports_dir(self, id: str) -> Path:
        return self.dir(id) / "exports"

    # -- the design's lock -----------------------------------------------------------

    def lock_path(self, id: str) -> Path:
        return self.dir(id) / ".lock"

    @contextlib.contextmanager
    def lock(self, id: str, blocking: bool = True) -> Iterator[bool]:
        """Hold design ``id``'s lock (``designs/<id>/.lock``, an exclusive ``flock``)
        while the block runs: other processes and other threads wait; the thread holding
        it may take it again. Yields ``True``; with ``blocking=False`` yields ``False``
        at once instead of waiting when someone else holds it (the block must then not
        touch the design). The design's folder must exist: :class:`DesignRemoved` when it
        doesn't, or was removed while this waited."""
        path = self.lock_path(id)
        held = _held(path)
        if not held.thread.acquire(blocking=blocking):
            yield False
            return
        try:
            if held.depth == 0:
                fd = _flock(path, blocking)
                if fd is None:
                    yield False
                    return
                held.fd = fd
            held.depth += 1
            try:
                yield True
            finally:
                held.depth -= 1
                if held.depth == 0 and held.fd is not None:
                    os.close(held.fd)       # releases the flock
                    held.fd = None
        finally:
            held.thread.release()

    # -- the design record ---------------------------------------------------------

    def write_design(self, design: Design) -> bool:
        """Record ``design`` (``spec.json`` + ``resolved.json``) unless it is already here;
        the first record wins (``created_at``, ``derived_from`` and ``patch`` stay)."""
        d = self.dir(design.id)
        if (d / "resolved.json").is_file():
            return False
        cfg = design.config
        # single files, atomic, no lock (module docstring): resolved.json, written last,
        # is what makes the design recorded
        _write_json(d / "spec.json", design.spec.to_dict())
        _write_json(d / "resolved.json", {
            "id": design.id, "engine_version": design.engine_version,
            "spec_hash": spec_hash(design.resolved), "created_at": design.created_at,
            "derived_from": design.derived_from, "patch": design.patch,
            "kind": design.kind, "linkage": cfg.linkage, "module": cfg.module,
            "sides": 2 if cfg.robot else 1, "warnings": list(design.warnings),
            "resolved": design.resolved,
        })
        return True

    def read_design(self, id: str) -> dict | None:
        """The ``resolved.json`` record, or ``None``."""
        return _read_json(self.dir(id) / "resolved.json")

    def read_spec(self, id: str) -> dict | None:
        return _read_json(self.dir(id) / "spec.json")

    def check_id(self, id: str, record: Mapping) -> None:
        """The record's resolved spec and engine version must hash to its id."""
        got = design_id(record.get("resolved", {}), record.get("engine_version", ""))
        if got != id:
            raise StoreError(f"{self.dir(id) / 'resolved.json'} hashes to {got}, not {id}: "
                             "the record was edited or is corrupt")

    # -- stage reports ---------------------------------------------------------------

    def report_path(self, id: str, stage: str, variant: str | None = None) -> Path:
        """``<stage>.json``; a ``variant`` (a verify's level) names ``<stage>.<variant>.json``,
        the copy kept per variant beside the latest."""
        if stage == "build":
            return self.dir(id) / "build" / "manifest.json"
        if variant:
            return self.dir(id) / f"{stage}.{_UNSAFE.sub('_', variant)}.json"
        return self.dir(id) / f"{stage}.json"

    def read_report(self, id: str, stage: str, variant: str | None = None) -> dict | None:
        """The stage's file as written (meta keys included), or ``None``."""
        return _read_json(self.report_path(id, stage, variant))

    def write_report(self, design: Design, stage: str, rep) -> Path:
        """Write ``rep`` as ``<stage>.json`` (a build: the manifest and the part files; a
        verify: also ``verify.<level>.json``, so a quick verify after a standard one doesn't
        cost the standard one again)."""
        if stage == "build":
            try:
                return self.write_build(design, rep)
            except DesignRemoved:
                log.warning("build not written: design %s is no longer in %s", design.id,
                            self.root)
                return self.report_path(design.id, stage)
        doc = {"stage": stage, "design": design.id, "engine_version": design.engine_version,
               "written_at": now_iso(), **report_doc(rep)}
        path = self.report_path(design.id, stage)
        # one file, atomic: no lock (it must never wait on a build's: module docstring);
        # a design removed meanwhile is not made again
        try:
            if stage == "verify" and doc.get("level"):
                _write_json(self.report_path(design.id, stage, str(doc["level"])), doc,
                            parents=False)
            return _write_json(path, doc, parents=False)
        except FileNotFoundError:
            log.warning("%s: not written: design %s is no longer in %s", path.name,
                        design.id, self.root)
            return path

    def stages(self, id: str) -> dict[str, dict]:
        """The stage files present: ``{stage: {ok, engine_version, written_at, ...}}``."""
        out = {}
        for stage in STAGES:
            doc = self.read_report(id, stage)
            if doc is None:
                continue
            entry = {"ok": doc.get("ok"), "engine_version": doc.get("engine_version"),
                     "written_at": doc.get("written_at")}
            for k in ("level", "t", "n_layers", "score"):
                if k in doc:
                    entry[k] = doc[k]
            out[stage] = entry
        return out

    # -- the build: manifest + STEP files -------------------------------------------

    def write_build(self, design: Design, rep) -> Path:
        """The build's manifest and one STEP per distinct part (``build/parts/``). A
        right-side part that is its left twin's exact mirror (the robot mirrors one side
        about z = 0) references the twin instead of a file of its own. The parts written
        are what the engine built, at the report's ``t``. Under the design's lock: a
        second build of the design waits instead of deleting these files mid-write."""
        with self.lock(design.id):
            return self._write_build(design, rep)

    def _write_build(self, design: Design, rep) -> Path:
        """Into a new folder beside ``build/`` (manifest last), then swapped in by two
        renames: a crash at any point leaves the previous build whole, or no build, never
        a manifest naming files that aren't its own (and the file hashes in the manifest
        let :meth:`load_mechanism` notice if they were)."""
        top = self.dir(design.id)
        for stale in top.glob(".build-*"):          # what an earlier crash left (locked)
            shutil.rmtree(stale, ignore_errors=True)
        d = Path(tempfile.mkdtemp(prefix=".build-new-", dir=top))
        try:
            self._write_build_into(d, design, rep)
            final = top / "build"
            if final.exists():
                old = Path(tempfile.mkdtemp(prefix=".build-old-", dir=top))
                os.rename(final, old / "build")
                os.rename(d, final)
                shutil.rmtree(old, ignore_errors=True)
            else:
                os.rename(d, final)
        except BaseException:
            shutil.rmtree(d, ignore_errors=True)
            raise
        return final / "manifest.json"

    def _write_build_into(self, d: Path, design: Design, rep) -> Path:
        import hashlib

        from build123d import export_step

        parts_dir = d / "parts"
        entries = []
        mech = design.mech if rep.ok else None
        if mech is not None:
            parts_dir.mkdir(parents=True)
            for name, part in design.parts.items():
                entry = {**part.to_dict(), "pose": part.pose, "color": mech.body(name).color,
                         "file": None, "same_as": None, "mirror": False}
                twin = mirror_twin(name)
                if twin in design.parts and is_mirror(design.parts[twin], part):
                    entry.update(same_as=twin, mirror=True)
                else:
                    fn = f"{_UNSAFE.sub('_', name)}.step"
                    tmp = parts_dir / f".{fn}.tmp"
                    export_step(part.built, str(tmp))
                    os.replace(tmp, parts_dir / fn)
                    entry["file"] = f"parts/{fn}"
                    entry["sha256"] = hashlib.sha256((parts_dir / fn).read_bytes()).hexdigest()
                entries.append(entry)
        doc = {"stage": "build", "design": design.id, "engine_version": design.engine_version,
               "written_at": now_iso(), **report_doc(rep), "parts": entries}
        if mech is not None:
            doc.update(name=mech.name, fastened=[list(p) for p in mech.meta.get("fastened", [])],
                       bom_extras=[{"key": b.key, "qty": b.qty, "where": b.where}
                                   for b in mech.bom_extras],
                       files=len([e for e in entries if e["file"]]))
        return _write_json(d / "manifest.json", doc)

    def load_mechanism(self, id: str, manifest: Mapping):
        """The fabricated :class:`mechanism.Mechanism` the manifest describes: every part's
        solid from its STEP file (a referenced part from its twin's, mirrored), at its pose,
        with the meta the checks read (the mid-plane, the fastened pairs) and the BOM's
        extras. Raises when a file is missing (``FileNotFoundError``) or isn't the one the
        manifest names (``ValueError``: its hash differs)."""
        import hashlib

        from build123d import Plane, import_step

        from spiderpig.hardware.bom import BomLine
        from spiderpig.mechanism import Body, Mechanism, Pose

        d = self.dir(id) / "build"
        solids: dict[str, object] = {}
        entries = list(manifest.get("parts", []))
        for e in entries:
            if e.get("file"):
                path = d / e["file"]
                if not path.is_file():
                    raise FileNotFoundError(f"{path} is missing")
                if e.get("sha256") and \
                        hashlib.sha256(path.read_bytes()).hexdigest() != e["sha256"]:
                    raise ValueError(f"{path} is not the file its manifest names (its hash "
                                     "differs)")
                solids[e["name"]] = import_step(str(path))
        for e in entries:
            if e.get("same_as"):
                src = solids[e["same_as"]]
                solids[e["name"]] = src.mirror(Plane.XY) if e.get("mirror") else src
        bodies = [Body(name=e["name"], part=solids[e["name"]], color=e.get("color"),
                       pose=Pose(np.array(e["pose"], dtype=float)), rigid_with=e.get("rigid_with"),
                       fab=e.get("fab"), bom_key=e.get("bom_key"), sheet=e.get("sheet"))
                  for e in entries]
        meta = dict(manifest.get("meta", {}))
        meta["fastened"] = [tuple(p) for p in manifest.get("fastened", [])]
        return Mechanism(manifest.get("name", id), bodies, [], meta,
                         [BomLine(**b) for b in manifest.get("bom_extras", [])])

    # -- the log ------------------------------------------------------------------------

    def log(self, id: str, entry: Mapping) -> None:
        d = self.dir(id)
        if not d.is_dir():          # removed (gc): don't make its folder again
            return
        try:
            with open(d / "log.jsonl", "a") as f:
                f.write(json.dumps(jsonable(entry), allow_nan=False) + "\n")
        except FileNotFoundError:
            return

    def read_log(self, id: str) -> list[dict]:
        path = self.dir(id) / "log.jsonl"
        if not path.is_file():
            return []
        out = []
        for line in path.read_text().splitlines():
            try:
                out.append(json.loads(line))
            except ValueError:
                continue
        return out

    def last_activity(self, id: str) -> datetime | None:
        """When the design was last operated on (its log), else recorded, else written."""
        times = [_parse_time(e.get("at")) for e in self.read_log(id)]
        rec = self.read_design(id) or {}
        times.append(_parse_time(rec.get("created_at")))
        times = [t for t in times if t is not None]
        if times:
            return max(times)
        try:
            return datetime.fromtimestamp(self.dir(id).stat().st_mtime, UTC)
        except OSError:
            return None

    # -- listing, gc -------------------------------------------------------------------

    def summary(self, id: str) -> dict:
        """One design's card: what it is (linkage, module, sides, the parameters the spec
        overrides, servo, sheet and thickness, constructions, which metrics it targets),
        when, from what, and which stages it holds."""
        rec = self.read_design(id) or {}
        stages = self.stages(id)
        last = self.last_activity(id)
        v = stages.get("verify")
        res = rec.get("resolved") or {}
        materials = res.get("materials") or {}
        spec = self.read_spec(id) or {}
        return {
            "id": id, "kind": rec.get("kind"), "linkage": rec.get("linkage"),
            "module": rec.get("module"), "sides": rec.get("sides"),
            "params": dict((spec.get("linkage") or {}).get("params") or {}),
            "servo": materials.get("servo"), "sheet": materials.get("sheet"),
            "thickness_mm": materials.get("thickness_mm"),
            "constructions": dict(res.get("constructions") or {}),
            "targets": {s: sorted(res[s]) for s in ("motion", "size", "budget") if res.get(s)},
            "engine_version": rec.get("engine_version"), "created_at": rec.get("created_at"),
            "derived_from": rec.get("derived_from"),
            "last_at": last.isoformat(timespec="seconds") if last else None,
            "stages": stages,
            "verify": None if v is None else {"level": v.get("level"), "ok": v.get("ok"),
                                              "score": v.get("score")},
            "build_t": stages.get("build", {}).get("t"),
        }

    def list_designs(self) -> list[dict]:
        return [self.summary(i) for i in self.ids()]

    def find_plans(self, spec_hash_: str, exclude: Iterable[str] = ()) -> list[tuple[str, dict]]:
        """Other designs of the same resolved spec (another engine version) that hold a
        plan: ``[(id, plan doc)]``, newest first."""
        skip = set(exclude)
        out = []
        for i in self.ids():
            if i in skip:
                continue
            rec = self.read_design(i) or {}
            if rec.get("spec_hash") != spec_hash_:
                continue
            doc = self.read_report(i, "plan")
            if doc is not None and doc.get("ok") and doc.get("layers"):
                out.append((i, doc))
        return sorted(out, key=lambda p: p[1].get("written_at", ""), reverse=True)

    def remove(self, id: str, blocking: bool = True) -> bool:
        """Remove design ``id``'s folder under its lock (waiting for a writer to finish;
        ``blocking=False``: leave a design in use alone). Whether it was removed."""
        d = self.dir(id)
        if not d.is_dir():
            return False
        try:
            with self.lock(id, blocking=blocking) as got:
                if not got:
                    return False
                # renamed away first: whoever waits on the lock wakes to a design that is
                # gone (DesignRemoved), never to a half-deleted folder; into a folder of its
                # own (a unique name: a crash leaves one, gc sweeps it)
                trash = Path(tempfile.mkdtemp(prefix=f"{TRASH}{id}-", dir=self.designs))
                os.rename(d, trash / id)
                shutil.rmtree(trash, ignore_errors=True)
        except DesignRemoved:
            return False
        return True

    def gc(self, keep: Iterable[str] | None = None,
           older_than: datetime | timedelta | float | None = None) -> list[str]:
        """Remove designs: those not in ``keep``, and/or those last operated on before
        ``older_than`` (an instant, an age as a ``timedelta`` or seconds). Both given, a
        design is removed only when it is not kept *and* too old. A design in use (its
        lock held: a build or export writing it) is left for a later gc. Returns the ids
        removed. The fabrication cache (``fab/``, :mod:`spiderpig.fabcache`) loses every
        other code version's folder, and with ``older_than`` the entries not used since
        (:func:`spiderpig.fabcache.gc`, logged)."""
        if keep is None and older_than is None:
            raise ValueError("gc(keep=[ids]) and/or gc(older_than=age) says what to remove")
        kept = None if keep is None else set(keep)
        cutoff = _cutoff(older_than)
        removed = []
        self.sweep()
        from spiderpig import fabcache

        try:
            age = None if cutoff is None else (datetime.now(UTC) - cutoff).total_seconds()
            for path in fabcache.gc(self, older_than=age):
                log.info("gc: removed %s", path)
        except OSError as e:
            log.warning("gc: the fabrication cache not cleaned: %s", e)
        for i in self.ids():
            if kept is not None and i in kept:
                continue
            if cutoff is not None:
                last = self.last_activity(i)
                if last is not None and last >= cutoff:
                    continue
            try:
                if self.remove(i, blocking=False):
                    removed.append(i)
            except OSError as e:            # one design's trouble doesn't stop the rest
                log.warning("gc: %s not removed: %s", i, e)
        return removed

    def sweep(self) -> list[str]:
        """Delete what interrupted removals left (``designs/.removed-*``, folders a crash
        between the rename and the delete left behind). Returns their names."""
        if not self.designs.is_dir():
            return []
        out = []
        for p in self.designs.iterdir():
            if p.name.startswith(TRASH) and p.is_dir():
                shutil.rmtree(p, ignore_errors=True)
                out.append(p.name)
        return out


def _cutoff(older_than) -> datetime | None:
    if older_than is None:
        return None
    if isinstance(older_than, datetime):
        return older_than if older_than.tzinfo else older_than.replace(tzinfo=UTC)
    if isinstance(older_than, timedelta):
        return datetime.now(UTC) - older_than
    return datetime.now(UTC) - timedelta(seconds=float(older_than))


# ---------------------------------------------------------------------------
# Parts: the mirror rule
# ---------------------------------------------------------------------------


def mirror_twin(name: str) -> str | None:
    """``"R.x"`` -> ``"L.x"``: the left-side part a right-side one mirrors (else ``None``)."""
    if len(name) > 2 and name.startswith("R."):
        return "L." + name[2:]
    return None


def is_mirror(left, right, tol: float = 1e-6) -> bool:
    """Is ``right``'s solid ``left``'s mirrored about z = 0 (the robot's mid-plane)? Volume
    and the mirrored bounding box must agree; the assembly mirrors exactly, so this only
    guards against a part that isn't the twin it seems."""
    if abs(left.volume_mm3 - right.volume_mm3) > tol * max(left.volume_mm3, 1.0):
        return False
    a, b = left.built.bounding_box(), right.built.bounding_box()
    scale = max(a.size.X, a.size.Y, a.size.Z, 1.0)
    pairs = ((a.min.X, b.min.X), (a.max.X, b.max.X), (a.min.Y, b.min.Y), (a.max.Y, b.max.Y),
             (a.min.Z, -b.max.Z), (a.max.Z, -b.min.Z))
    return all(abs(x - y) <= tol * scale for x, y in pairs)


# ---------------------------------------------------------------------------
# Comparing
# ---------------------------------------------------------------------------

SKIP_KEYS = frozenset({*META_KEYS, "seconds", "table", "proof", "text", "warnings"})
LIST_KEYS = ("requirement", "name", "part", "point", "link", "at", "stage", "a")


def diff_json(a, b, path: str = "", out: dict | None = None) -> dict:
    """Every leaf where two JSON values differ: ``{"dotted.path": {"a": .., "b": ..}}``.
    Objects recurse; a list of objects with a common name key is compared by that key,
    other lists element by element; volatile keys (timings, tables, texts) are skipped."""
    out = {} if out is None else out
    if isinstance(a, Mapping) and isinstance(b, Mapping):
        for k in sorted(set(a) | set(b), key=str):
            if k in SKIP_KEYS:
                continue
            p = f"{path}.{k}" if path else str(k)
            if k not in a or k not in b:
                out[p] = {"a": a.get(k), "b": b.get(k)}
            else:
                diff_json(a[k], b[k], p, out)
        return out
    if isinstance(a, list) and isinstance(b, list):
        key = _list_key(a, b)
        if key is not None:
            ka, kb = {x[key]: x for x in a}, {x[key]: x for x in b}
            return diff_json(ka, kb, path, out)
        if len(a) == len(b):
            for i, (x, y) in enumerate(zip(a, b, strict=True)):
                diff_json(x, y, f"{path}[{i}]", out)
            return out
    if a != b and not (_num(a) and _num(b) and np.isclose(a, b, rtol=1e-9, atol=1e-12)):
        out[path or "."] = {"a": a, "b": b}
    return out


def _list_key(a: list, b: list) -> str | None:
    items = a + b
    if not items or not all(isinstance(x, Mapping) for x in items):
        return None
    for key in LIST_KEYS:
        if (all(key in x and isinstance(x[key], str) for x in items)
                and len({x[key] for x in a}) == len(a) and len({x[key] for x in b}) == len(b)):
            return key
    return None


def _num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


__all__ = ["DEFAULT_ROOT", "PROJECT", "STAGES", "STORE_ENV", "DesignRemoved", "Store",
           "StoreError", "diff_json",
           "is_mirror", "mirror_twin", "now_iso", "report_doc"]
