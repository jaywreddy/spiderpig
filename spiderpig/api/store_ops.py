"""A design's handle and its store: :func:`load`, :func:`derive`, :func:`compare`,
:func:`list_designs`, :func:`gc`, and the build's and the export's store bookkeeping
(:func:`design_lock`; resolve and the stage records are :mod:`spiderpig.stages`')."""


from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path

from spiderpig.config import robot_layout
from spiderpig.design import Design
from spiderpig.failure import apply_patch, merge_patch
from spiderpig.stages.records import EDITED_STAGES

# load and the config of a recorded design sit a layer lower (stages.resolve: the CLIs
# open a stored design by its id there); their names stay importable from here
from spiderpig.stages.resolve import _config_from_resolved as _config_from_resolved
from spiderpig.stages.resolve import _stored_roles as _stored_roles
from spiderpig.stages.resolve import load as load
from spiderpig.stages.resolve import resolve
from spiderpig.store import PROJECT, Store, diff_json, report_doc

# ---------------------------------------------------------------------------
# The store: load (stages.resolve's), list, gc, compare, derive
# ---------------------------------------------------------------------------


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
    spec = design.spec.to_dict()
    legs = design.resolved.get("legs", {})
    if robot_layout(design.lk) == "turret" and "robot" not in spec.get("legs", {}):
        # what the parent resolved to (a side, for an arm stored before legs.robot)
        spec.setdefault("legs", {})["robot"] = bool(legs.get("robot", legs.get("sides") == 2))
    return resolve(apply_patch(spec, patch), store, derived_from=design.id, patch=patch)


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
