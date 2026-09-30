"""spiderpig: walking linkages as a compiler, from a Spec to verified geometry.

    from spiderpig import api, Spec
    design = api.resolve({"kind": "walker", "linkage": {"key": "klann"},
                          "size": {"stack_mm": {"max": 40}}})
    report = api.verify(design, "quick")        # rows with an evidence tier
    api.build(design); design.parts["L.b1_leg0"].solid   # a live build123d solid
    api.export(design, ["step", "dxf", "bom"], "out")

The agent-facing surface: :mod:`spiderpig.spec` is the vocabulary (validated,
JSON-schema'd), :mod:`spiderpig.api` the operations (pure functions of a
:class:`spiderpig.design.Design`), :mod:`spiderpig.failure` the structured failure every
engine exception becomes, :mod:`spiderpig.verify` the harness, :mod:`spiderpig.store` the
per-project store, :mod:`spiderpig.mcp` the MCP server and :mod:`spiderpig.cli` the
``spiderpig`` command. See ``docs/agentlib/API.md``.

The engine lives beside them in the same package: :mod:`spiderpig.linkage` (the
symbolic side and the registry), :mod:`spiderpig.linkages` (the definitions),
:mod:`spiderpig.mechanism`, :mod:`spiderpig.stack` (the layer planner),
:mod:`spiderpig.construction`, :mod:`spiderpig.servos`, :mod:`spiderpig.hardware`,
:mod:`spiderpig.config` (:class:`~spiderpig.config.BuildConfig`), :mod:`spiderpig.fabricate`,
:mod:`spiderpig.walk`, :mod:`spiderpig.sim`, :mod:`spiderpig.bake` (the viewer's .glb),
:mod:`spiderpig.build` (STEP/STL/DXF/BOM), :mod:`spiderpig.server` (the viewer's app) and
:mod:`spiderpig.tools`. Importing ``spiderpig`` itself imports none of it: the names below
(the types and helpers) load on first use (PEP 562), so ``spiderpig --help`` and
``import spiderpig`` stay instant. The operations are reached through
:mod:`spiderpig.api` (``api.verify``, ``api.walk``, ...), never as attributes of the
package: several share a name with an engine module (``spiderpig.walk``, ``explain``,
``build``, ``verify``, ``recommend``), and a package attribute is the submodule.
"""

from __future__ import annotations

import importlib

# name -> the module that defines it; never a name a submodule carries.
_EXPORTS: dict[str, str] = {
    **dict.fromkeys(("Design", "Part", "engine_version"), "spiderpig.design"),
    **dict.fromkeys(("Failure", "Recommendation", "apply_patch", "merge_patch"),
                    "spiderpig.failure"),
    **dict.fromkeys(("SPEC_VERSION", "TARGET_FIELDS", "Spec", "SpecError", "SpecErrors",
                     "Target", "spec_schema", "validate"), "spiderpig.spec"),
    "Store": "spiderpig.store",
    **dict.fromkeys(("LEVELS", "Row", "VerifyReport"), "spiderpig.verify"),
}

__all__ = sorted(_EXPORTS)


def __getattr__(name: str):
    module = _EXPORTS.get(name)
    if module is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(importlib.import_module(module), name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(_EXPORTS))
