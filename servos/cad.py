"""Fetch and load the manufacturers' servo models (:class:`servos.spec.CadRef`).

Where a model comes from, in order:

1. ``servos/cad/`` (:data:`VENDORED`): files whose licence allows
   redistribution, checked into the repo (see ``servos/cad/NOTICE``);
2. the download cache, ``$SPIDERPIG_CAD_CACHE`` (default
   ``~/.cache/spiderpig/cad``), one folder per pinned hash;
3. the model's ``url``, unless ``SPIDERPIG_OFFLINE=1``. A zip archive is
   checked against ``archive_sha256`` and the ``member`` extracted.

Every file is checked against its pinned ``sha256`` before use: a file that
doesn't match is never used (a mismatching download isn't cached). Nothing
here raises: a model that can't be had gives ``None`` (logged), and callers
fall back to the parametric model (:mod:`servos.model`).
"""

from __future__ import annotations

import hashlib
import io
import logging
import os
import tempfile
import urllib.request
import zipfile
from functools import lru_cache
from pathlib import Path

from servos.spec import CadRef

log = logging.getLogger("spiderpig.servos")

VENDORED = Path(__file__).with_name("cad")
CACHE_ENV = "SPIDERPIG_CAD_CACHE"
OFFLINE_ENV = "SPIDERPIG_OFFLINE"
USE_CAD_ENV = "SPIDERPIG_SERVO_CAD"      # "0" draws every servo parametrically
TIMEOUT = 60.0
MAX_BYTES = 64 << 20


def _flag(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in ("1", "true", "yes", "on")


def offline() -> bool:
    """``SPIDERPIG_OFFLINE=1``: never download."""
    return _flag(OFFLINE_ENV)


def cad_enabled() -> bool:
    """``SPIDERPIG_SERVO_CAD=0`` turns manufacturer models off (parametric servos only)."""
    return os.environ.get(USE_CAD_ENV, "1").strip().lower() not in ("0", "false", "no", "off")


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


def vendored_path(ref: CadRef) -> Path:
    return VENDORED / ref.filename


def _download(url: str, timeout: float) -> bytes:
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
        vend = vendored_path(ref)
        if vend.is_file():
            if _good(vend, ref.sha256):
                return vend
            log.warning("vendored %s doesn't match its pinned sha256; not using it", vend)
        path = cached_path(ref)
        if _good(path, ref.sha256):
            return path
        if not allow_download:
            return None
        data = _download(ref.url, timeout)
        if ref.member:
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
    except Exception as e:  # never fail the build over a model
        log.warning("servo model %s unavailable: %s", ref.filename, e)
        return None


def _trsf(ref: CadRef):
    """The file-to-servo-frame transform (scale to mm first, then place)."""
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


def load(ref: CadRef, *, allow_download: bool | None = None):
    """``ref``'s model in the servo frame (a build123d shape), or ``None``."""
    path = fetch(ref, allow_download=allow_download)
    if path is None:
        return None
    try:
        return _load_cached(str(path), ref.format, ref)
    except Exception as e:
        log.warning("servo model %s didn't load: %s", path, e)
        return None


__all__ = [
    "CACHE_ENV", "OFFLINE_ENV", "USE_CAD_ENV", "VENDORED", "cache_dir", "cad_enabled",
    "cached_path", "fetch", "load", "offline", "sha256_file", "vendored_path",
]
