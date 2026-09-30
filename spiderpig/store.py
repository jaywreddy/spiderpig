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
                         ``stage``, ``design``, ``engine_version`` and ``written_at``
``build/manifest.json``  the build report plus, per part, its pose, colour and file
``build/parts/*.step``   one STEP per distinct part at the manifest's ``t``; a right-side
                         part references its left twin (``same_as``, ``mirror``)
``exports/``             what :func:`spiderpig.api.export` writes by default
``log.jsonl``            one line per operation: op, engine version, seconds, ok, cached
=======================  ============================================================

Validity: a stage file is used as is when its ``engine_version`` is the
running engine's (:func:`spiderpig.design.engine_version`), except a plan,
which is always re-made through :meth:`stack.StackProblem.plan` and
re-checked with :func:`stack.verify_plan` before use (the closures it
needs are rebuilt from the template, nothing is pickled). A plan from
another engine version is re-verified on fresh sampling and adopted
without its optimality proof, or re-solved when the check fails; the
other stages are recomputed under the new engine. Files are written
atomically, so several users may share one folder.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import tempfile
from collections.abc import Iterable, Mapping
from datetime import UTC, datetime, timedelta
from pathlib import Path

import numpy as np

from spiderpig.design import Design, design_id, jsonable, now_iso, spec_hash

STORE_ENV = "SPIDERPIG_STORE"
DEFAULT_ROOT = ".spiderpig"
STAGES = ("check", "plan", "walk", "build", "recheck", "verify", "export")
META_KEYS = ("stage", "design", "engine_version", "written_at")
PROJECT = "project"          # the ``store`` argument's default: the project store

_ID = re.compile(r"^[0-9a-f]{16}$")
_UNSAFE = re.compile(r"[^\w.\-]")


class StoreError(ValueError):
    """A design folder that doesn't hold what its id says."""


def _parse_time(s: str | None) -> datetime | None:
    if not s:
        return None
    try:
        t = datetime.fromisoformat(s)
    except ValueError:
        return None
    return t if t.tzinfo is not None else t.replace(tzinfo=UTC)


def _write_json(path: Path, doc) -> Path:
    """Write ``doc`` as JSON atomically (a temp file in the same folder, then a rename)."""
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
        if not _ID.match(id):
            raise ValueError(f"not a design id: {id!r}")
        return self.designs / id

    def has(self, id: str) -> bool:
        return (self.dir(id) / "resolved.json").is_file()

    def ids(self) -> list[str]:
        """Every design recorded here (a folder with a ``resolved.json``), oldest first."""
        if not self.designs.is_dir():
            return []
        out = []
        for p in self.designs.iterdir():
            if _ID.match(p.name) and (p / "resolved.json").is_file():
                out.append(p.name)
        return sorted(out, key=lambda i: (self.read_design(i) or {}).get("created_at", ""))

    def exports_dir(self, id: str) -> Path:
        return self.dir(id) / "exports"

    # -- the design record ---------------------------------------------------------

    def write_design(self, design: Design) -> bool:
        """Record ``design`` (``spec.json`` + ``resolved.json``) unless it is already here;
        the first record wins (``created_at``, ``derived_from`` and ``patch`` stay)."""
        d = self.dir(design.id)
        if (d / "resolved.json").is_file():
            return False
        cfg = design.config
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

    def report_path(self, id: str, stage: str) -> Path:
        if stage == "build":
            return self.dir(id) / "build" / "manifest.json"
        return self.dir(id) / f"{stage}.json"

    def read_report(self, id: str, stage: str) -> dict | None:
        """The stage's file as written (meta keys included), or ``None``."""
        return _read_json(self.report_path(id, stage))

    def write_report(self, design: Design, stage: str, rep) -> Path:
        """Write ``rep`` as ``<stage>.json`` (a build: the manifest and the part files)."""
        if stage == "build":
            return self.write_build(design, rep)
        doc = {"stage": stage, "design": design.id, "engine_version": design.engine_version,
               "written_at": now_iso(), **report_doc(rep)}
        return _write_json(self.report_path(design.id, stage), doc)

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
        are what the engine built, at the report's ``t``."""
        from build123d import export_step

        d = self.dir(design.id) / "build"
        parts_dir = d / "parts"
        if parts_dir.exists():
            shutil.rmtree(parts_dir)
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
        extras. Raises when a file is missing."""
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
                solids[e["name"]] = import_step(str(path))
        for e in entries:
            if e.get("same_as"):
                src = solids[e["same_as"]]
                solids[e["name"]] = src.mirror(Plane.XY) if e.get("mirror") else src
        bodies = [Body(name=e["name"], part=solids[e["name"]], color=e.get("color"),
                       pose=Pose(np.array(e["pose"], dtype=float)), rigid_with=e.get("rigid_with"),
                       fab=e.get("fab"), bom_key=e.get("bom_key"))
                  for e in entries]
        meta = dict(manifest.get("meta", {}))
        meta["fastened"] = [tuple(p) for p in manifest.get("fastened", [])]
        return Mechanism(manifest.get("name", id), bodies, [], meta,
                         [BomLine(**b) for b in manifest.get("bom_extras", [])])

    # -- the log ------------------------------------------------------------------------

    def log(self, id: str, entry: Mapping) -> None:
        d = self.dir(id)
        d.mkdir(parents=True, exist_ok=True)
        with open(d / "log.jsonl", "a") as f:
            f.write(json.dumps(jsonable(entry), allow_nan=False) + "\n")

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

    def remove(self, id: str) -> None:
        d = self.dir(id)
        if d.is_dir():
            shutil.rmtree(d)

    def gc(self, keep: Iterable[str] | None = None,
           older_than: datetime | timedelta | float | None = None) -> list[str]:
        """Remove designs: those not in ``keep``, and/or those last operated on before
        ``older_than`` (an instant, an age as a ``timedelta`` or seconds). Both given, a
        design is removed only when it is not kept *and* too old. Returns the ids
        removed; nothing outside ``designs/<id>`` is touched."""
        if keep is None and older_than is None:
            raise ValueError("gc(keep=[ids]) and/or gc(older_than=age) says what to remove")
        kept = None if keep is None else set(keep)
        cutoff = _cutoff(older_than)
        removed = []
        for i in self.ids():
            if kept is not None and i in kept:
                continue
            if cutoff is not None:
                last = self.last_activity(i)
                if last is not None and last >= cutoff:
                    continue
            self.remove(i)
            removed.append(i)
        return removed


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


__all__ = ["DEFAULT_ROOT", "PROJECT", "STAGES", "STORE_ENV", "Store", "StoreError", "diff_json",
           "is_mirror", "mirror_twin", "now_iso", "report_doc"]
