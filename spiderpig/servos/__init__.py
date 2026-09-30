"""Servo models (see :mod:`servos.spec` for the contract and the servo frame).

``get(key)`` returns a :class:`servos.spec.ServoSpec`; ``DEFAULT`` is what a
build uses unless told otherwise. :mod:`servos.catalog` holds the data,
:mod:`servos.cad` fetches the manufacturers' models, :mod:`servos.model`
builds a servo (CAD or parametric) and its horn, :mod:`servos.mount` puts it
on the inner frame plate.
"""

from __future__ import annotations

from spiderpig.servos.spec import ServoSpec

REGISTRY: dict[str, ServoSpec] = {}
DEFAULT = "sts3215"


def register(spec: ServoSpec) -> ServoSpec:
    REGISTRY[spec.key] = spec
    return spec


def get(key: str) -> ServoSpec:
    from spiderpig.servos import catalog  # noqa: F401  (registers the specs)

    try:
        return REGISTRY[key]
    except KeyError:
        raise KeyError(f"no servo {key!r}; have {sorted(REGISTRY)}") from None


def available() -> list[str]:
    """Servos that can drive a crank (full rotation)."""
    from spiderpig.servos import catalog  # noqa: F401

    return sorted(k for k, s in REGISTRY.items() if s.continuous)
