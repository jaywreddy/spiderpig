"""spiderpig as a compiler: a Spec in, verified geometry out.

    from spiderpig import api, Spec
    design = api.resolve({"kind": "walker", "linkage": {"key": "klann"},
                          "size": {"stack_mm": {"max": 40}}})
    report = api.verify(design, "quick")        # rows with an evidence tier
    api.build(design); design.parts["L.b1_leg0"].solid   # a live build123d solid
    api.export(design, ["step", "dxf", "bom"], "out")

:mod:`spiderpig.spec` is the vocabulary (validated, JSON-schema'd),
:mod:`spiderpig.api` the operations (pure functions of a
:class:`spiderpig.design.Design`), :mod:`spiderpig.failure` the structured
failure every engine exception becomes, :mod:`spiderpig.verify` the harness.
See ``docs/agentlib/API.md``.
"""

from __future__ import annotations

from spiderpig.api import (
    attach_build,
    build,
    check,
    compare,
    derive,
    describe,
    explain,
    export,
    gc,
    list_designs,
    list_linkages,
    load,
    plan,
    recheck,
    recommend,
    resolve,
    verify,
    walk,
)
from spiderpig.design import Design, Part, engine_version
from spiderpig.failure import Failure, Recommendation, apply_patch, merge_patch
from spiderpig.spec import (
    SPEC_VERSION,
    TARGET_FIELDS,
    Spec,
    SpecError,
    SpecErrors,
    Target,
    spec_schema,
    validate,
)
from spiderpig.store import Store
from spiderpig.verify import LEVELS, Row, VerifyReport

__all__ = [
    "LEVELS", "SPEC_VERSION", "TARGET_FIELDS", "Design", "Failure", "Part", "Recommendation",
    "Row", "Spec", "SpecError", "SpecErrors", "Store", "Target", "VerifyReport", "apply_patch",
    "attach_build", "build", "check", "compare", "derive", "describe", "engine_version",
    "explain", "export", "gc", "list_designs", "list_linkages", "load", "merge_patch", "plan",
    "recheck", "recommend", "resolve", "spec_schema", "validate", "verify", "walk",
]
