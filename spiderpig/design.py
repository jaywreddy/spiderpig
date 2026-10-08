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

import ast
import hashlib
import importlib.metadata
import json
import math
import tomllib
from collections.abc import Mapping
from dataclasses import dataclass, field, fields, is_dataclass, replace
from datetime import UTC, datetime
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from spiderpig.config import BuildConfig
from spiderpig.spec import Spec
from spiderpig.stack import StackSpec

if TYPE_CHECKING:
    from spiderpig.fabricate import SideDesign
    from spiderpig.mechanism import Mechanism, MechanismTemplate
    from spiderpig.store import Store

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


ENGINE_EXCLUDE = ("server", "mcp", "tools", "guide", "cli.py", "view.py", "__main__.py")
"""Package sources that never change a design's result (the viewer's server, the MCP layer,
the assembly guide, the command lines): left out of :func:`engine_version`."""
_ENGINE_VERSION: list[str] = []


def _code_digest(source: str | bytes) -> bytes:
    """A source's code without its docstrings (a docs-only edit keeps the engine version)."""
    tree = ast.parse(source)
    for node in ast.walk(tree):
        body = getattr(node, "body", None)
        if (isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                and body and isinstance(body[0], ast.Expr)
                and isinstance(body[0].value, ast.Constant)
                and isinstance(body[0].value.value, str)):
            node.body = body[1:] or [ast.Pass()]
    return ast.dump(tree).encode()


def engine_version() -> str:
    """The package version plus a hash of what changes a design's result: the code of every
    package source but :data:`ENGINE_EXCLUDE` (the linkages, the constructions, the planner,
    the catalog and the BOM: a hardware change is a new engine, so a store's stages from
    before it are recomputed, its plans re-verified), docstrings stripped, and the planner's
    defaults (:class:`spiderpig.stack.StackSpec`) less its time budget (``max_seconds``,
    ``SPIDERPIG_PLAN_SECONDS``: a budget bounds the search, not what a plan is). Computed
    once per process, and kept on disk (:func:`_digest_cache`) under a signature of the
    files it hashes, so a process on unchanged sources reads it (milliseconds) instead of
    parsing every source again (1-2 s)."""
    if not _ENGINE_VERSION:
        files = [p for p in sorted(ROOT.rglob("*.py"))
                 if p.relative_to(ROOT).parts[0] not in ENGINE_EXCLUDE]
        spec = repr(replace(StackSpec(), max_seconds=60.0))
        version = package_version()
        cached = _digest_cache(files, spec, version)
        known = None if cached is None else cached.read()
        if known is not None:
            _ENGINE_VERSION.append(known)
            return known
        h = hashlib.sha256()
        for p in files:
            h.update(str(p.relative_to(ROOT)).encode())
            h.update(_code_digest(p.read_bytes()))
        h.update(spec.encode())
        _ENGINE_VERSION.append(f"{version}+{h.hexdigest()[:12]}")
        if cached is not None:
            cached.write(_ENGINE_VERSION[0])
    return _ENGINE_VERSION[0]


DIGEST_CACHE_ENV = "SPIDERPIG_DIGEST_CACHE"
"""A directory for :func:`engine_version`'s cached digests, or ``off`` (``0``) to compute it
every time; default ``$XDG_CACHE_HOME/spiderpig/engine-version`` (``~/.cache/...``)."""


@dataclass
class _DigestEntry:
    """One cached :func:`engine_version`: the file named by the signature of what it
    hashes, holding the signature in full and the version (a read checks both)."""

    path: Path
    signature: str

    def read(self) -> str | None:
        try:
            doc = json.loads(self.path.read_text())
        except (OSError, ValueError):
            return None
        if not isinstance(doc, dict) or doc.get("signature") != self.signature:
            return None
        v = doc.get("version")
        return v if isinstance(v, str) else None

    def write(self, version: str) -> None:
        import os
        import tempfile

        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=self.path.parent, prefix=".engine-")
            with os.fdopen(fd, "w") as f:
                json.dump({"signature": self.signature, "version": version}, f)
            os.replace(tmp, self.path)
        except OSError:
            pass                # a read-only cache: computed again next time, no harm


def _digest_cache(files: list[Path], spec: str, version: str) -> _DigestEntry | None:
    """Where :func:`engine_version`'s digest of ``files`` is kept: a file named by the
    signature of everything the digest reads or depends on (the package root, every
    hashed file's path, size, modification and change times and inode, the planner's
    defaults, the package version, the Python that parses them), so any edit, added or
    removed source, or another interpreter is another entry and computes afresh. ``None``
    when off (:data:`DIGEST_CACHE_ENV`) or a file can't be read."""
    import os
    import sys

    env = os.environ.get(DIGEST_CACHE_ENV, "").strip()
    if env.lower() in ("off", "0", "false", "no"):
        return None
    if env:
        root = Path(env).expanduser()
    else:
        base = os.environ.get("XDG_CACHE_HOME") or str(Path.home() / ".cache")
        root = Path(base) / "spiderpig" / "engine-version"
    h = hashlib.sha256()
    for part in (str(ROOT), sys.version, version, spec):
        h.update(part.encode() + b"\0")
    try:
        for p in files:
            st = p.stat()
            h.update(f"{p.relative_to(ROOT)}\0{st.st_size}\0{st.st_mtime_ns}\0"
                     f"{st.st_ctime_ns}\0{st.st_ino}\n".encode())
    except OSError:
        return None
    sig = h.hexdigest()
    return _DigestEntry(root / f"{sig[:32]}.json", sig)


_SOURCE_VERSION: list[str] = []


def source_version() -> str:
    """The package version plus a hash of every Python source of the package: the key of
    what is cached per *code* rather than per design (the linkage cards, the guide's
    tables), so any edit of the checkout starts them afresh. Computed once per process."""
    if not _SOURCE_VERSION:
        h = hashlib.sha256()
        for p in sorted(ROOT.rglob("*.py")):
            h.update(str(p.relative_to(ROOT)).encode())
            h.update(p.read_bytes())
        _SOURCE_VERSION.append(f"{package_version()}+src.{h.hexdigest()[:12]}")
    return _SOURCE_VERSION[0]


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


SKIP_FIELDS = frozenset({"solid", "built", "_measured"})     # live solids never serialize


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
    if not isinstance(obj, type) and is_dataclass(obj):
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
    and ``pose`` places it in the world (:meth:`placed`). A one-sided design's parts sit
    in the side's frame (the plan's z, the joints' xy); a robot's sit in the world frame,
    the left side moved down by the chassis' mid-plane ``z_mid`` and the right side its
    mirror image moved up (``z_side`` is the part's z range back in its side's frame), so
    a cut placed by the side's coordinates goes through :meth:`locate`. Replace ``solid``
    to edit the part; ``edited`` says whether it differs from what the engine built, and
    only :func:`spiderpig.api.recheck` restores the guarantee (it notes an edit that left
    the volume as built). ``volume_mm3`` and ``mass_g`` are measured on the solid as it
    now is (``density`` in g/cm3; a purchased part with a catalog mass, the servo, keeps
    ``fixed_mass_g``); ``dims_mm`` and ``layers`` are the build's."""

    name: str
    solid: object
    group: str
    side: str | None
    fab: str
    material: str
    dims_mm: tuple[float, float, float]
    layers: tuple[int, ...]
    density: float = 0.0
    fixed_mass_g: float | None = None
    bom_key: str | None = None
    rigid_with: str | None = None
    pose: list[list[float]] = field(default_factory=list)
    sheet: str | None = None                    # a laser-cut part's sheet (catalog key)
    z_mid: float | None = None                  # the robot's mid-plane; None on one side
    z_side: tuple[float, float] | None = None   # the solid's z range in its side's frame
    built: object = field(default=None, repr=False)
    _measured: tuple | None = field(default=None, repr=False)   # (solid, its volume)

    @property
    def edited(self) -> bool:
        return self.solid is not self.built

    def locate(self, xy, z: float | None = None):
        """A build123d ``Location`` in the solid's frame for a tool centred at the side's
        ``xy`` (a joint's, ``design.mech.body(name).joint(j).pose.matrix[:2, 3]``) and the
        side's ``z`` (default: the middle of the part's own layers), whichever side of
        the robot the part is on:
        ``part.solid = part.solid - Cylinder(1.5, 10).moved(part.locate((a + b) / 2))``."""
        from build123d import Location

        x, y = float(xy[0]), float(xy[1])
        if z is None:
            if self.z_side is None:
                raise ValueError(f"{self.name}: no z range recorded: give z")
            z = sum(self.z_side) / 2
        z = float(z)
        if self.z_mid is None or self.side is None:
            return Location((x, y, z))
        if self.side == "L":
            return Location((x, y, z - self.z_mid))
        return Location((x, y, self.z_mid - z))

    @property
    def volume_mm3(self) -> float:
        """The live solid's volume (measured again after ``solid`` is replaced)."""
        m = self._measured
        if m is None or m[0] is not self.solid:
            from spiderpig.hardware.mass import part_props

            self._measured = m = (self.solid, float(part_props(self.solid).volume))
        return m[1]

    @property
    def mass_g(self) -> float:
        """The live solid's mass: its volume at the material's density, or the catalog's
        fixed mass of a purchased part."""
        if self.fixed_mass_g is not None:
            return self.fixed_mass_g
        return self.volume_mm3 / 1000.0 * self.density

    def placed(self):
        """The solid in world coordinates (the body's pose applied)."""
        from spiderpig.mechanism import Pose
        from spiderpig.shapes import moved

        return moved(self.solid, Pose.from_matrix(np.array(self.pose)).to_location())

    def to_dict(self) -> dict:
        return {"name": self.name, "group": self.group, "side": self.side, "fab": self.fab,
                "material": self.material, "mass_g": round(self.mass_g, 3),
                "volume_mm3": round(self.volume_mm3, 3),
                "dims_mm": [round(d, 3) for d in self.dims_mm], "layers": list(self.layers),
                "bom_key": self.bom_key, "rigid_with": self.rigid_with, "sheet": self.sheet,
                "edited": self.edited}


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
    template: MechanismTemplate | None = field(default=None, repr=False)
    side: SideDesign | None = field(default=None, repr=False)
    mech: Mechanism | None = field(default=None, repr=False)          # the fabricated one
    build_t: float | None = None                           # crank angle the parts are at
    store: Store | None = field(default=None, repr=False)
    derived_from: str | None = None                        # the design this one's spec patches
    patch: dict | None = None                              # the merge patch from it
    created_at: str = field(default_factory=now_iso)
    edited: bool = False        # a recheck accepted edited parts: this handle's parts aren't
    #                             the store's (its exports and verifies stay off the store)

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
