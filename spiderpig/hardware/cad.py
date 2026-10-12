"""Fetch, cache and load vendors' models of purchased parts (:class:`CadRef`).

Models are never checked into the repo. Where one comes from, in order:

1. the download cache, ``$SPIDERPIG_CAD_CACHE`` (default
   ``~/.cache/spiderpig/cad``), one folder per pinned hash;
2. the model's ``url``, unless ``SPIDERPIG_OFFLINE=1``. A zip archive is
   checked against ``archive_sha256`` and the ``member`` extracted.

``mise run fetch-cad`` (``python -m servos.cad``) fills the cache ahead of
time, e.g. before going offline.

Every file is checked against its pinned ``sha256`` before use: a file that
doesn't match is never used (a mismatching download isn't cached). Nothing
here raises: a model that can't be had gives ``None`` (logged), and callers
fall back to their parametric model (the servos' :mod:`servos.model`; a bought part's
catalog dims). A model with no ``url`` (``vendor`` / ``part_number``: a vendor whose
pages refuse a scripted fetch) comes only from the cache, which :func:`import_file`
fills with a file fetched by other means (a browser).

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
    """Where the cache keeps ``ref``'s model file."""
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
    """A local, hash-verified copy of ``ref``'s model file, or ``None``.

    ``allow_download`` defaults to "not :func:`offline`".
    """
    if allow_download is None:
        allow_download = not offline()
    try:
        path = cached_path(ref)
        if _good(path, ref.sha256):
            return path
        if not allow_download or not ref.url:
            return None
        data = _download(ref.url, timeout)
        if ref.member:
            import io
            import zipfile

            got = hashlib.sha256(data).hexdigest()
            if ref.archive_sha256 and got != ref.archive_sha256.lower():
                log.warning("%s: archive sha256 mismatch; not using it", ref.url)
                return None
            with zipfile.ZipFile(io.BytesIO(data)) as z:
                data = z.read(ref.member)
        if hashlib.sha256(data).hexdigest() != ref.sha256.lower():
            log.warning("%s: sha256 mismatch; not using it", ref.url)
            return None
        _write_atomic(path, data)
        return path
    # a download, zip or disk error: never fail the build over a model
    except Exception as e:  # noqa: BLE001
        log.warning("model %s unavailable: %s", ref.filename, e)
        return None


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
def _load_cached(path: str, fmt: str, ref: CadRef):
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
    """``ref``'s model in the part's frame (a build123d shape), or ``None``."""
    path = fetch(ref, allow_download=allow_download)
    if path is None:
        return None
    try:
        return _load_cached(str(path), ref.format, ref)
    # OCCT's import errors aren't one class (Standard_Failure and kin through pybind11): a
    # model that doesn't load is left out, never fails the build
    except Exception as e:  # noqa: BLE001
        log.warning("model %s didn't load: %s", path, e)
        return None


def import_file(ref: CadRef, src: Path) -> Path:
    """Put a model fetched by other means (a browser, by hand) into the cache as ``ref``'s:
    the file must match ``ref.sha256``. Returns the cached path; :class:`ValueError` when
    the file isn't ``ref``'s."""
    data = Path(src).read_bytes()
    got = hashlib.sha256(data).hexdigest()
    if got != ref.sha256.lower():
        raise ValueError(f"{src}: sha256 {got[:16]}... is not {ref.filename}'s pinned "
                         f"{ref.sha256[:16]}...")
    path = cached_path(ref)
    _write_atomic(path, data)
    return path
