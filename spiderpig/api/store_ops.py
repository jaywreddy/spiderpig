"""A design's handle and its store: :func:`resolve`, :func:`load`, :func:`derive`,
:func:`compare`, :func:`list_designs`, :func:`gc`, and the stage records every
operation reads and writes."""


from __future__ import annotations

import math
from contextlib import contextmanager
from pathlib import Path
from typing import TYPE_CHECKING

from spiderpig import linkage
from spiderpig.config import (
    BuildConfig,
    ParamError,
    removed_param,
)
from spiderpig.construction.base import Params
from spiderpig.design import (
    Design,
    engine_version,
)
from spiderpig.failure import apply_patch, merge_patch
from spiderpig.spec import (
    FIT_FIELDS,
    Spec,
)
from spiderpig.stages.records import EDITED_STAGES
from spiderpig.stages.resolve import resolve
from spiderpig.store import PROJECT, Store, diff_json, report_doc

if TYPE_CHECKING:
    pass


# ---------------------------------------------------------------------------
# resolve
# ---------------------------------------------------------------------------


def _config_from_resolved(resolved: dict) -> BuildConfig:
    """The config of a recorded design, from its resolved spec alone (every value written
    in, so a default that moved since never changes a stored design)."""
    lk = linkage.get(resolved["linkage"]["key"])
    legs, mat, cons, fit = (resolved[k] for k in ("legs", "materials", "constructions", "fit"))
    for name, value in fit.items():     # a removed Params field: only at its last default
        if (gone := removed_param(name, value)) is not None:
            raise ParamError(gone)
    return BuildConfig(
        linkage=lk.key, module=legs["module"], robot=legs["sides"] == 2,
        phases=tuple(math.radians(p) for p in legs["phases_deg"]),
        proportions=tuple(sorted(resolved["linkage"]["params"].items())),
        sheet=mat["sheet"], thickness=mat.get("thickness_mm"), servo=mat["servo"],
        frame_sheet=mat.get("frame_sheet") or mat["sheet"],
        crank_sheet=mat.get("crank_sheet") or mat["sheet"],
        link_sheets=tuple(sorted((mat.get("link_sheets") or {}).items())),
        pillar=cons["pillar"], pin=cons["pin"], crank=cons["crank"],
        heads=cons.get("heads") or "sink",
        params=Params(**{k: fit[k] for k in FIT_FIELDS if k in fit}),
    )


# ---------------------------------------------------------------------------
# The store: load, list, gc, compare, derive
# ---------------------------------------------------------------------------


def load(id: str, store: Store | str | Path | None = PROJECT) -> Design:
    """The handle of a recorded design: its spec and resolved values from the store (the
    id must hash to them), its config rebuilt from the resolved spec. Its stage reports
    load as the operations ask for them; ``design.warnings`` says when the design was
    recorded under another engine version (its stored plan is then re-verified, the other
    stages recomputed). ``KeyError`` when the store has no such design."""
    store = Store.of(store)
    if store is None:
        raise ValueError("load(id) needs a store")
    rec = store.read_design(id)
    if rec is None:
        raise KeyError(f"no design {id!r} in {store.root}")
    store.check_id(id, rec)
    spec_doc = store.read_spec(id)
    if spec_doc is None:
        raise KeyError(f"design {id!r} in {store.root} has no spec.json")
    resolved = rec["resolved"]
    config = _config_from_resolved(resolved)    # first: a removed construction is a ParamError
    spec = Spec.from_dict(spec_doc)
    engine = engine_version()
    design = Design(id, spec, resolved, config, engine, list(rec.get("warnings", [])),
                    derived_from=rec.get("derived_from"), patch=rec.get("patch"),
                    created_at=rec.get("created_at") or "")
    if rec.get("engine_version") != engine:
        design.warnings.append(f"recorded under engine {rec.get('engine_version')}, now "
                               f"{engine}: the stored plan is re-verified before use, the other "
                               "stages recomputed")
    design.store = store
    return design


def list_designs(store: Store | str | Path | None = PROJECT) -> list[dict]:
    """Every design in the store, oldest first: id, kind, linkage, module, sides, engine
    version, when it was recorded and last used, what it derives from, the stages it
    holds (each with ``ok``) and its latest verify verdict."""
    store = Store.of(store)
    return [] if store is None else store.list_designs()


def gc(keep=None, older_than=None, store: Store | str | Path | None = PROJECT) -> list[str]:
    """Remove designs from the store (:meth:`spiderpig.store.Store.gc`): those not in
    ``keep`` (ids or handles) and/or last used before ``older_than`` (a ``datetime``, a
    ``timedelta`` or seconds). Returns the ids removed."""
    store = Store.of(store)
    if store is None:
        return []
    if keep is not None:
        keep = [k.id if isinstance(k, Design) else k for k in keep]
    return store.gc(keep, older_than)


def derive(design: Design, patch: dict, store=PROJECT) -> Design:
    """Resolve ``design``'s spec with ``patch`` merged in (:func:`apply_patch`; what
    :func:`recommend` hands out), recording the parent and the patch on the child
    (``derived_from``, ``patch``). The child lives in the parent's store unless ``store``
    says otherwise."""
    if store == PROJECT:
        store = design.store
    return resolve(apply_patch(design.spec.to_dict(), patch), store,
                   derived_from=design.id, patch=patch)


def compare(a: Design | str, b: Design | str, store: Store | str | Path | None = PROJECT
            ) -> dict:
    """Two designs side by side (handles or ids in ``store``): the merge patch from
    ``a``'s spec to ``b``'s (and between their resolved specs), whether one derives from
    the other, and every stage report both hold with each differing value
    (``{"stage": {"path": {"a": .., "b": ..}}}``, :func:`spiderpig.store.diff_json`)."""
    st = Store.of(store)
    ia, sa, ra, docs_a = _docs_of(a, st)
    ib, sb, rb, docs_b = _docs_of(b, st)
    ea, eb = _engine_of(a, st), _engine_of(b, st)
    derived = None
    if _derived_from(b, st) == ia:
        derived = f"{ib} derives from {ia}"
    elif _derived_from(a, st) == ib:
        derived = f"{ia} derives from {ib}"
    return {
        "a": ia, "b": ib,
        "spec_patch": merge_patch(sa, sb), "resolved_patch": merge_patch(ra, rb),
        "engine_version": None if ea == eb else {"a": ea, "b": eb},
        "derived": derived,
        "reports": {s: diff_json(docs_a[s], docs_b[s]) for s in sorted(set(docs_a) & set(docs_b))},
        "only_in": {"a": sorted(set(docs_a) - set(docs_b)), "b": sorted(set(docs_b) - set(docs_a))},
    }


def _docs_of(x: Design | str, store: Store | None):
    """``(id, spec doc, resolved doc, {stage: report doc})`` of a handle (its reports over
    the store's) or of a recorded id."""
    if isinstance(x, Design):
        st = x.store or store
        docs = {} if st is None else _store_docs(st, x.id)
        if x.edited:            # the store's are the unedited design's
            docs = {k: v for k, v in docs.items() if k not in EDITED_STAGES}
        docs.update({k: report_doc(v) for k, v in x.reports.items()})
        return x.id, x.spec.to_dict(), x.resolved, docs
    if store is None:
        raise ValueError(f"compare({x!r}) needs a store to read the design from")
    rec = store.read_design(x)
    if rec is None:
        raise KeyError(f"no design {x!r} in {store.root}")
    return x, store.read_spec(x) or {}, rec["resolved"], _store_docs(store, x)


def _store_docs(store: Store, id: str) -> dict[str, dict]:
    from spiderpig.store import STAGES

    out = {}
    for s in STAGES:
        doc = store.read_report(id, s)
        if doc is not None:
            out[s] = doc
    return out


def _engine_of(x: Design | str, store: Store | None) -> str | None:
    if isinstance(x, Design):
        return x.engine_version
    rec = store.read_design(x) if store is not None else None
    return None if rec is None else rec.get("engine_version")


def _derived_from(x: Design | str, store: Store | None) -> str | None:
    if isinstance(x, Design):
        return x.derived_from
    rec = store.read_design(x) if store is not None else None
    return None if rec is None else rec.get("derived_from")


# ---------------------------------------------------------------------------
# check, plan, explain, recommend
# ---------------------------------------------------------------------------


def _manifest(out: Path) -> dict:
    """``out/manifest.json`` (the last export's, or a ``spiderpig build``'s), else ``{}``."""
    import json

    try:
        doc = json.loads((out / "manifest.json").read_text())
    except (OSError, ValueError):
        return {}
    return doc if isinstance(doc, dict) else {}


def _manifest_design(out: Path) -> str | None:
    """The design id ``out/manifest.json`` names (the last export or build into ``out``)."""
    return _manifest(out).get("design")


# ---------------------------------------------------------------------------
# build, recheck
# ---------------------------------------------------------------------------


@contextmanager
def design_lock(design: Design):
    """Hold the design's store lock (:meth:`spiderpig.store.Store.lock`) when it is recorded
    in a store: what the multi-file operations (:func:`build`, :func:`export`) run under,
    so a second build, an export or a ``gc`` of the same design waits for them."""
    st = design.store
    if st is None or not st.has(design.id):
        yield
        return
    with st.lock(design.id):
        yield


def _drop_stored(design: Design, *stages: str) -> None:
    """Delete ``stages``' reports from the store (a verify's per-level copies too)."""
    from spiderpig.verify import LEVELS

    store = design.store
    assert store is not None    # called only on a design recorded in a store
    for stage in stages:
        store.report_path(design.id, stage).unlink(missing_ok=True)
        for level in LEVELS if stage == "verify" else ():
            store.report_path(design.id, stage, level).unlink(missing_ok=True)


def _forget(design: Design, *stages: str) -> None:
    """Drop ``stages``' reports from the handle, so the next call computes them afresh (an
    edited handle never reads the store's: :data:`EDITED_STAGES`)."""
    for stage in stages:
        design.reports.pop(stage, None)
