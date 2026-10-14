"""The servos' manufacturer models: :mod:`hardware.cad`, with the servos' own switch.

Everything generic (the cache, the pinned-hash fetch, loading a model into the part frame,
the derived "prepared" records) lives in :mod:`spiderpig.hardware.cad`, which every bought
part's model goes through; this module re-exports it for the servos and adds
``SPIDERPIG_SERVO_CAD=0`` (every servo drawn parametrically) and ``mise run fetch-cad``.
"""

from __future__ import annotations

import logging
import os

from spiderpig.hardware.cad import (  # noqa: F401 - the servos' names for the generic module
    CACHE_ENV,
    MAX_BYTES,
    OFFLINE_ENV,
    PREPARED_FORMAT,
    STRICT_ENV,
    TIMEOUT,
    CadError,
    CadRef,
    _download,
    _good,
    _import,
    _load_cached,
    _trsf,
    _write_atomic,
    cache_dir,
    cached_path,
    fetch,
    import_file,
    load,
    offline,
    prepared_path,
    read_prepared,
    sha256_file,
    strict,
    unavailable,
    write_prepared,
)

log = logging.getLogger("spiderpig.servos")

USE_CAD_ENV = "SPIDERPIG_SERVO_CAD"      # "0" draws every servo parametrically


def cad_enabled() -> bool:
    """``SPIDERPIG_SERVO_CAD=0`` turns manufacturer models off (parametric servos only)."""
    return os.environ.get(USE_CAD_ENV, "1").strip().lower() not in ("0", "false", "no", "off")


def main() -> int:
    """Download every registered servo's models into the cache (``mise run fetch-cad``)."""
    from spiderpig import servos

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    missing = 0
    for key in servos.available():
        for ref in servos.get(key).cads:
            path = fetch(ref, allow_download=True)
            log.info("%-12s %-28s %s", key, ref.filename, path or "UNAVAILABLE")
            missing += path is None
    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())
