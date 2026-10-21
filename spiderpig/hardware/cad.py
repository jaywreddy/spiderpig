"""Fetch, cache and load vendors' models of purchased parts (:class:`CadRef`).

Models are never checked into the repo. Where one comes from, in order:

1. the download cache, ``$SPIDERPIG_CAD_CACHE`` (default
   ``~/.cache/spiderpig/cad``), one folder per pinned hash;
2. the model's ``url``, unless ``SPIDERPIG_OFFLINE=1``. A zip archive is
   checked against ``archive_sha256`` and the ``member`` extracted.

``mise run fetch-cad`` (``python -m servos.cad``) fills the cache ahead of
time, e.g. before going offline.

Every file is checked against its pinned ``sha256`` before use: a file that
doesn't match is never used (a mismatching download isn't cached). By default nothing
here raises: a model that can't be had gives ``None`` (logged), and callers
fall back to their parametric model (the servos' :mod:`servos.model`; a bought part's
catalog dims). A model with no ``url`` (``vendor`` / ``part_number``: a vendor whose
pages refuse a scripted fetch) comes only from the cache, which :func:`import_file`
fills with a file fetched by other means (a browser).

**Strict** (``SPIDERPIG_CAD_STRICT=1``, :func:`strict`): a model a build wants (its switch
on: ``SPIDERPIG_SERVO_CAD`` / ``SPIDERPIG_PART_CAD``) that can't be drawn raises
:class:`CadError` instead of falling back: not in the cache and not downloaded (offline,
a failed download, a hash mismatch, a vendor with no scripted download), a file that
doesn't import, a servo's horn that can't be stripped, a bought part's model that isn't
the catalog's size or can't be placed. For a build whose renders and STEP must carry the
vendors' models; off by default (the gate and the audit run offline, parametric). The
probes (:func:`fetch`, :func:`state`) never raise; ``spiderpig cad check`` loads every
registered model this way.

The servos' models went through here first (:mod:`servos.cad` re-exports this module,
with its own switch ``SPIDERPIG_SERVO_CAD``).
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
import tempfile
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

log = logging.getLogger("spiderpig.cad")

CACHE_ENV = "SPIDERPIG_CAD_CACHE"
OFFLINE_ENV = "SPIDERPIG_OFFLINE"
STRICT_ENV = "SPIDERPIG_CAD_STRICT"     # "1": a wanted model that can't be drawn raises
TIMEOUT = 60.0
MAX_BYTES = 64 << 20


@dataclass(frozen=True)
class CadRef:
    """A vendor's model of a purchased part, aligned to the part's frame by ``transform``.

    ``transform`` is a 4x4 row-major matrix (as a tuple of 16 floats) taking
    the file's coordinates (after scaling by ``scale`` to mm) into the part's
    frame (a servo's: the servo frame). ``sha256`` pins the exact file; a mismatch means
    "don't use it".

    Optional: ``member`` names the model inside a zip archive (``url`` is
    then the archive, pinned by ``archive_sha256``; ``sha256`` pins the
    member). ``strip`` lists the part-frame bounding boxes ``(x0, y0, z0,
    x1, y1, z1)`` of solids to drop (the stock output horn and its screw,
    which :mod:`servos.model` draws separately); ``strip_cut`` a cylinder
    ``(r, z0, z1)`` about the output axis cut away where a model fuses the
    horn into the case. Files are downloaded and cached at build time
    (:mod:`hardware.cad`), never checked into the repo.

    ``vendor`` and ``part_number`` name where a model comes from when it has no direct
    ``url`` (a vendor whose pages refuse a scripted download, e.g. McMaster-Carr): the
    cache is then filled by :func:`import_file` with a file fetched by other means, and
    the pinned ``sha256`` still decides whether the file is used.
    """

    url: str
    sha256: str
    filename: str
    format: str = "step"           # "step" | "stl"
    scale: float = 1.0
    transform: tuple[float, ...] = (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)
    license: str = ""
    source: str = ""               # human-readable provenance
    member: str = ""
    archive_sha256: str = ""
    strip: tuple[tuple[float, ...], ...] = ()
    strip_cut: tuple[float, float, float] | None = None
    vendor: str = ""
    part_number: str = ""



def _flag(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in ("1", "true", "yes", "on")


def offline() -> bool:
    """``SPIDERPIG_OFFLINE=1``: never download."""
    return _flag(OFFLINE_ENV)


def strict() -> bool:
    """``SPIDERPIG_CAD_STRICT=1``: a vendor model that can't be drawn raises
    :class:`CadError` instead of the parametric fallback (module docstring)."""
    return _flag(STRICT_ENV)


class CadError(RuntimeError):
    """A vendor model a build wants and can't draw, under ``SPIDERPIG_CAD_STRICT=1``."""


def unavailable(what: str) -> None:
    """A wanted vendor model can't be drawn (``what`` says why): raised as
    :class:`CadError` under :func:`strict`, else logged (the caller falls back to its
    parametric part)."""
    if strict():
        raise CadError(f"{what} ({STRICT_ENV}=1: no parametric fallback)")
    log.warning("%s", what)


def cache_dir() -> Path:
    env = os.environ.get(CACHE_ENV)
    return Path(env).expanduser() if env else Path.home() / ".cache" / "spiderpig" / "cad"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _good(path: Path, sha: str) -> bool:
    try:
        return path.is_file() and sha256_file(path) == sha.lower()
    except OSError:
        return False


def cached_path(ref: CadRef) -> Path:
    """Where the cache keeps ``ref``'s model file: by its pinned hash, or (an unpinned
    vendor model, ``sha256 == ""``) by vendor and part number."""
    if not ref.sha256:
        return (cache_dir() / "unpinned" / (ref.vendor or "any")
                / (ref.part_number or Path(ref.filename).stem) / ref.filename)
    return cache_dir() / ref.sha256[:16] / ref.filename


def _download(url: str, timeout: float) -> bytes:
    import urllib.request  # (here: the config imports this module; the network stack later)

    req = urllib.request.Request(url, headers={"User-Agent": "spiderpig-cad-fetch/1"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        data = r.read(MAX_BYTES + 1)
    if len(data) > MAX_BYTES:
        raise ValueError(f"{url}: larger than {MAX_BYTES} bytes")
    return data


def _write_atomic(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=".part-")
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(data)
        os.chmod(tmp, 0o644)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


def fetch(ref: CadRef, *, allow_download: bool | None = None,
          timeout: float = TIMEOUT) -> Path | None:
    """A local, hash-verified copy of ``ref``'s model file, or ``None`` (never raises: a
    probe; :func:`load` says why under :func:`strict`).

    ``allow_download`` defaults to "not :func:`offline`".
    """
    return _fetch(ref, allow_download, timeout)[0]


def _fetch(ref: CadRef, allow_download: bool | None = None,
           timeout: float = TIMEOUT) -> tuple[Path | None, str]:
    """:func:`fetch`'s path, and when there is none, why."""
    if allow_download is None:
        allow_download = not offline()
    try:
        path = cached_path(ref)
        if not ref.sha256:          # unpinned: whatever import_file put there
            return (path, "") if path.is_file() else (None, (
                f"not in the CAD cache ({path}; `spiderpig cad fetch` or `spiderpig cad "
                "import`)"))
        if _good(path, ref.sha256):
            return path, ""
        if not allow_download or not ref.url:
            why = ("no download allowed" if ref.url else "no URL to download it from")
            if ref.url and offline():
                why = f"{OFFLINE_ENV}=1"
            return None, f"not in the CAD cache ({path}) and {why}"
        data = _download(ref.url, timeout)
        if ref.member:
            import io
            import zipfile

            got = hashlib.sha256(data).hexdigest()
            if ref.archive_sha256 and got != ref.archive_sha256.lower():
                log.warning("%s: archive sha256 mismatch; not using it", ref.url)
                return None, f"{ref.url}: archive sha256 mismatch"
            with zipfile.ZipFile(io.BytesIO(data)) as z:
                data = z.read(ref.member)
        if hashlib.sha256(data).hexdigest() != ref.sha256.lower():
            log.warning("%s: sha256 mismatch; not using it", ref.url)
            return None, f"{ref.url}: sha256 mismatch"
        _write_atomic(path, data)
        return path, ""
    # a download, zip or disk error: never fail the build over a model (unless strict:
    # load raises with this reason)
    except Exception as e:  # noqa: BLE001
        log.warning("model %s unavailable: %s", ref.filename, e)
        return None, f"download failed: {e}"


def _trsf(ref: CadRef):
    """The file-to-part-frame transform (scale to mm first, then place)."""
    from OCP.gp import gp_Pnt, gp_Trsf

    m = [float(v) for v in ref.transform]
    t = gp_Trsf()
    t.SetValues(*m[:12])
    if ref.scale != 1.0:
        s = gp_Trsf()
        s.SetScale(gp_Pnt(0, 0, 0), float(ref.scale))
        t.Multiply(s)
    return t


def _import(path: Path, fmt: str):
    from build123d import import_step, import_stl

    if fmt == "stl":
        return import_stl(str(path))
    return import_step(str(path))


@lru_cache(maxsize=16)
def _load_cached(path: str, fmt: str, ref: CadRef, digest: str = ""):
    """The model at ``path``, placed (remembered by its pin, or an unpinned file's own
    ``digest``: a re-imported file is loaded afresh)."""
    from build123d import Compound
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform

    shape = _import(Path(path), fmt)
    moved = BRepBuilderAPI_Transform(shape.wrapped, _trsf(ref), True).Shape()
    return Compound(moved)


PREPARED_FORMAT = 1       # bump when what :func:`prepared_path` records is derived differently


def prepared_path(ref: CadRef, key: str) -> Path:
    """Where a record derived from ``ref``'s model is cached (a JSON beside the download):
    by the model's hash, ``key`` (what the derivation depends on) and the format version."""
    tag = hashlib.sha256(f"{PREPARED_FORMAT}|{key}".encode()).hexdigest()[:16]
    return cache_dir() / f"{ref.sha256.lower()}.{tag}.json"


def read_prepared(path: Path) -> dict | None:
    """The record cached at ``path`` (:func:`write_prepared`), or ``None``."""
    try:
        with open(path) as f:
            doc = json.load(f)
    except (OSError, ValueError):
        return None
    return doc if isinstance(doc, dict) else None


def write_prepared(path: Path, doc: dict) -> None:
    """Cache ``doc`` at ``path`` (atomically; never raises)."""
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.", suffix=".tmp")
        with os.fdopen(fd, "w") as f:
            json.dump(doc, f)
        os.replace(tmp, path)
    except (OSError, TypeError, ValueError) as e:
        log.warning("couldn't cache the record at %s: %s", path, e)


def load(ref: CadRef, *, allow_download: bool | None = None):
    """``ref``'s model in the part's frame (a build123d shape), or ``None``
    (:class:`CadError` instead under :func:`strict`)."""
    path, why = _fetch(ref, allow_download)
    if path is None:
        if strict():
            raise CadError(f"model {ref.filename}: {why} ({STRICT_ENV}=1: no parametric "
                           "fallback)")
        return None
    try:
        digest = "" if ref.sha256 else sha256_file(path)
        return _load_cached(str(path), ref.format, ref, digest)
    # OCCT's import errors aren't one class (Standard_Failure and kin through pybind11): a
    # model that doesn't load is left out, never fails the build (unless strict)
    except Exception as e:  # noqa: BLE001
        unavailable(f"model {path} didn't load: {e}")
        return None


def import_file(ref: CadRef, src: Path) -> Path:
    """Put a model fetched by other means (a browser, by hand) into the cache as ``ref``'s:
    a pinned ref's file must match ``ref.sha256`` (an unpinned one's is taken, and its
    hash is what the caches key on: :func:`state`). Returns the cached path;
    :class:`ValueError` when the file isn't ``ref``'s."""
    data = Path(src).read_bytes()
    got = hashlib.sha256(data).hexdigest()
    if ref.sha256 and got != ref.sha256.lower():
        raise ValueError(f"{src}: sha256 {got[:16]}... is not {ref.filename}'s pinned "
                         f"{ref.sha256[:16]}...")
    if not data.lstrip()[:16].upper().startswith(b"ISO-10303-21") and ref.format == "step":
        raise ValueError(f"{src}: not a STEP file (no ISO-10303-21 header)")
    path = cached_path(ref)
    _write_atomic(path, data)
    return path


def state(ref: CadRef) -> tuple[str, str]:
    """What ``ref``'s model is to the caches: ``("parametric", "")`` when no file is in the
    cache, else ``("cad", <the file's sha256>)`` (a pinned ref's is its pin; an unpinned
    one's is read, so a re-imported file re-makes what it shapes)."""
    path = fetch(ref, allow_download=False)
    if path is None:
        return ("parametric", "")
    return ("cad", ref.sha256.lower() or sha256_file(path))


PART_CAD: dict[str, CadRef] = {}
"""Bought parts' vendor models by catalog key (a catalog data module registers them:
:mod:`hardware.bearing_catalog`)."""


def register_part_cad(key: str, ref: CadRef) -> CadRef:
    if key in PART_CAD and PART_CAD[key] != ref:
        raise ValueError(f"part model for {key!r} registered twice")
    PART_CAD[key] = ref
    return ref


def part_cad(key: str) -> CadRef | None:
    """The vendor model registered for catalog item ``key`` (loads the catalog)."""
    from spiderpig.hardware.catalog import _load

    _load()
    return PART_CAD.get(key)


PART_CAD_ENV = "SPIDERPIG_PART_CAD"     # "0" draws every bought part from its catalog dims


def part_cad_enabled() -> bool:
    """``SPIDERPIG_PART_CAD=0`` turns vendors' models of bought parts off (parametric)."""
    return os.environ.get(PART_CAD_ENV, "1").strip().lower() not in ("0", "false", "no", "off")


def load_axisymmetric(ref: CadRef, od: float, width: float, tol: float = 0.02):
    """``ref``'s model of a round part (a bearing, a washer) centred on the origin with its
    axis along z, its faces at ``z = 0`` and ``z = width``; ``None`` unless the file's
    bounding box is the catalog's ``od`` x ``od`` x ``width`` within ``tol`` mm in some
    orientation (a model that isn't the part the catalog says is never used). The axis is
    the box's odd side; the transform is found, not trusted."""
    if not part_cad_enabled():
        return None
    shape = load(ref, allow_download=False)
    if shape is None:
        return None
    try:
        from build123d import Location

        bb = shape.bounding_box()
        size = (bb.size.X, bb.size.Y, bb.size.Z)
        axis = min(range(3), key=lambda i: abs(size[i] - width))
        others = [size[i] for i in range(3) if i != axis]
        if abs(size[axis] - width) > tol or any(abs(o - od) > tol for o in others):
            unavailable(f"model {ref.filename} is {size[0]:.2f} x {size[1]:.2f} x "
                        f"{size[2]:.2f} mm, not the catalog's {od:g} x {od:g} x {width:g}: "
                        "not used")
            return None
        from build123d import Axis

        c = bb.center()
        moved = shape.moved(Location((-c.X, -c.Y, -c.Z)))
        if axis == 0:               # the part's axis along x: turn it onto z about y
            moved = moved.rotate(Axis.Y, 90.0)
        elif axis == 1:             # along y: onto z about x
            moved = moved.rotate(Axis.X, 90.0)
        return moved.moved(Location((0, 0, width / 2)))
    except CadError:
        raise
    # an OCCT failure on a vendor's file: draw the part from the catalog instead
    except Exception as e:  # noqa: BLE001
        unavailable(f"model {ref.filename} couldn't be placed: {e}")
        return None
