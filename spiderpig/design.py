"""The design handle: a resolved spec, its :class:`config.BuildConfig`, and every stage's
result, held in memory.

A :class:`Design` is what the operations in :mod:`spiderpig.api` read and
write. Its ``id`` is content-addressed (the resolved spec and the engine
version, :func:`engine_version`), so the same spec on the same engine is
the same design. Every stage's report (``reports[stage]``) is a plain
dataclass whose :func:`jsonable` form is what a store (step 2) will write
per stage; the engine objects a stage produced (``template``, ``side``,
``mech``) are kept beside them for the next stage and are rebuilt from the
resolved spec when a design is loaded (a plan through
:meth:`stack.StackProblem.plan` + :func:`stack.verify_plan`, a build from
its part files).

After :func:`spiderpig.api.build`, ``parts[name]`` is a :class:`Part`: the
live build123d solid of that body (the same object the export and the
clash check use), its group, side, fabrication, material, mass and layers.
An agent may measure it or replace it; an edited solid lies outside the
correct-by-construction guarantee until :func:`spiderpig.api.recheck`
passes on it.
"""

from __future__ import annotations

import hashlib
import importlib.metadata
import json
import math
import tomllib
from collections.abc import Mapping
from dataclasses import dataclass, field, fields, is_dataclass
from datetime import UTC, datetime
from pathlib import Path

import numpy as np

from spiderpig.config import BuildConfig
from spiderpig.spec import Spec
from spiderpig.stack import StackSpec

ROOT = Path(__file__).resolve().parent      # the spiderpig package


def package_version() -> str:
    """The installed distribution's version, else the checkout's ``pyproject.toml``."""
    try:
        return importlib.metadata.version("spiderpig")
    except importlib.metadata.PackageNotFoundError:
        pyproject = ROOT.parent / "pyproject.toml"
        if pyproject.is_file():
            with open(pyproject, "rb") as f:
                return tomllib.load(f)["project"]["version"]
        return "0.0.0"


def engine_version() -> str:
    """The package version plus a hash of what changes a design's result: the linkage
    definitions (``spiderpig/linkages/*.py``) and the planner's defaults
    (:class:`spiderpig.stack.StackSpec`)."""
    version = package_version()
    h = hashlib.sha256()
    for p in sorted((ROOT / "linkages").glob("*.py")):
        h.update(p.name.encode())
        h.update(p.read_bytes())
    h.update(repr(StackSpec()).encode())
    return f"{version}+{h.hexdigest()[:12]}"


def canonical_json(obj) -> str:
    """One JSON text per value: sorted keys, no spaces, no NaN."""
    return json.dumps(jsonable(obj), sort_keys=True, separators=(",", ":"), allow_nan=False)


def spec_hash(resolved: Mapping) -> str:
    """The resolved spec alone (no engine version): what one design shares across engine
    versions, so a store can find an earlier plan of it."""
    return hashlib.sha256(canonical_json(resolved).encode()).hexdigest()[:16]


def now_iso() -> str:
    return datetime.now(UTC).isoformat(timespec="seconds")


def design_id(resolved: Mapping, engine: str) -> str:
    return hashlib.sha256((canonical_json(resolved) + engine).encode()).hexdigest()[:16]


SKIP_FIELDS = frozenset({"solid", "built"})     # live solids never serialize


def jsonable(obj):
    """``obj`` as JSON values: dataclasses as objects (solids left out), numpy as Python,
    non-finite floats as ``None``, paths as strings, anything else by ``to_dict`` or
    ``str``."""
    if obj is None or isinstance(obj, (bool, str)):
        return obj
    if isinstance(obj, (int, np.integer)):
        return int(obj)
    if isinstance(obj, (float, np.floating)):
        return float(obj) if math.isfinite(obj) else None
    if isinstance(obj, np.ndarray):
        return jsonable(obj.tolist())
    if is_dataclass(obj) and not isinstance(obj, type):
        return {f.name: jsonable(getattr(obj, f.name)) for f in fields(obj)
                if f.name not in SKIP_FIELDS}
    if isinstance(obj, Mapping):
        return {str(k): jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, set, frozenset)):
        return [jsonable(v) for v in obj]
    if isinstance(obj, Path):
        return str(obj)
    if hasattr(obj, "to_dict"):
        return jsonable(obj.to_dict())
    return str(obj)


@dataclass
class Part:
    """One fabricated body. ``solid`` is the live build123d solid in the body's own frame
    (for every part of a side that is the side's frame; ``pose`` places it in the world:
    :meth:`placed`). Replace it to edit the part; ``edited`` says whether it differs from
    what the engine built, and only :func:`spiderpig.api.recheck` restores the guarantee."""

    name: str
    solid: object
    group: str
    side: str | None
    fab: str
    material: str
    mass_g: float
    volume_mm3: float
    dims_mm: tuple[float, float, float]
    layers: tuple[int, ...]
    bom_key: str | None = None
    rigid_with: str | None = None
    pose: list[list[float]] = field(default_factory=list)
    built: object = field(default=None, repr=False)

    @property
    def edited(self) -> bool:
        return self.solid is not self.built

    def placed(self):
        """The solid in world coordinates (the body's pose applied)."""
        from spiderpig.mechanism import Pose

        return self.solid.moved(Pose.from_matrix(np.array(self.pose)).to_location())

    def to_dict(self) -> dict:
        return {"name": self.name, "group": self.group, "side": self.side, "fab": self.fab,
                "material": self.material, "mass_g": round(self.mass_g, 3),
                "volume_mm3": round(self.volume_mm3, 3),
                "dims_mm": [round(d, 3) for d in self.dims_mm], "layers": list(self.layers),
                "bom_key": self.bom_key, "rigid_with": self.rigid_with, "edited": self.edited}


@dataclass
class Design:
    """A resolved spec and everything the engine has said about it (module docstring)."""

    id: str
    spec: Spec
    resolved: dict
    config: BuildConfig
    engine_version: str
    warnings: list[str] = field(default_factory=list)
    reports: dict[str, object] = field(default_factory=dict)
    log: list[dict] = field(default_factory=list)
    parts: dict[str, Part] = field(default_factory=dict)
    template: object = field(default=None, repr=False)     # the side's MechanismTemplate
    side: object = field(default=None, repr=False)         # fabricate.SideDesign
    mech: object = field(default=None, repr=False)         # the fabricated Mechanism
    build_t: float | None = None                           # crank angle the parts are at
    store: object = field(default=None, repr=False)        # spiderpig.store.Store, or None
    derived_from: str | None = None                        # the design this one's spec patches
    patch: dict | None = None                              # the merge patch from it
    created_at: str = field(default_factory=now_iso)

    @property
    def lk(self):
        return self.config.lk

    @property
    def kind(self) -> str:
        return self.spec.kind

    def report(self, stage: str):
        """The stored report of ``stage`` (``check``, ``plan``, ``walk``, ``build``,
        ``recheck``, ``verify``, ``export``), or ``None``."""
        return self.reports.get(stage)

    def record(self, op: str, seconds: float, ok: bool, cached: bool = False) -> dict:
        """Log an operation on this handle (``cached``: served from the store); the store
        appends the same line to the design's ``log.jsonl``."""
        entry = {"op": op, "engine_version": self.engine_version,
                 "seconds": round(seconds, 3), "ok": ok, "cached": cached, "at": now_iso()}
        self.log.append(entry)
        return entry

    def to_dict(self) -> dict:
        """Everything but the solids and the engine objects: what a store writes."""
        return {
            "id": self.id, "engine_version": self.engine_version, "spec": self.spec.to_dict(),
            "resolved": self.resolved, "warnings": list(self.warnings),
            "derived_from": self.derived_from, "patch": self.patch,
            "created_at": self.created_at,
            "reports": {k: jsonable(v) for k, v in self.reports.items()},
            "parts": [p.to_dict() for p in self.parts.values()],
            "build_t": self.build_t, "log": list(self.log),
        }
