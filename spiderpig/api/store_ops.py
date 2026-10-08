"""A design's handle and its store: :func:`resolve`, :func:`load`, :func:`derive`,
:func:`compare`, :func:`list_designs`, :func:`gc`, and the stage records every
operation reads and writes."""


from __future__ import annotations

import logging
import math
import time
from contextlib import contextmanager
from pathlib import Path

from spiderpig import linkage, servos
from spiderpig import walk as walk_model
from spiderpig.config import (
    BuildConfig,
    ParamError,
    removed_param,
)
from spiderpig.construction.base import Params
from spiderpig.design import (
    Design,
    design_id,
    engine_version,
)
from spiderpig.fabricate import template_for as _template_for
from spiderpig.failure import apply_patch, merge_patch
from spiderpig.hardware.catalog import sheet_size, sheet_thickness
from spiderpig.materials import link_sheets
from spiderpig.spec import (
    ALLOWANCE,
    FIT_FIELDS,
    SECTIONS,
    Spec,
    SpecError,
    SpecErrors,
    default_module,
    effective_hard,
    validate,
)
from spiderpig.store import PROJECT, Store, diff_json, report_doc

log = logging.getLogger("spiderpig")

# ---------------------------------------------------------------------------
# resolve
# ---------------------------------------------------------------------------


def resolve(spec: Spec | dict, store: Store | str | Path | None = PROJECT, *,
            derived_from: str | None = None, patch: dict | None = None) -> Design:
    """Validate ``spec`` (a :class:`Spec` or its document), infer what it leaves out, build
    its :class:`config.BuildConfig` and return the :class:`Design` handle. The inferred
    values are in ``design.resolved`` (the complete spec the id is computed from);
    :class:`SpecErrors` lists everything wrong with an invalid spec.

    ``store``: where the design and every stage's result are kept: the project store
    by default (``$SPIDERPIG_STORE``, else ``./.spiderpig``, created now), a path, a
    :class:`spiderpig.store.Store`, or ``None`` to keep everything in memory. A design
    already recorded there keeps its record (``created_at``, ``derived_from``,
    ``patch``); a new one records ``derived_from`` and ``patch`` (see :func:`derive`)."""
    if not isinstance(spec, Spec):
        spec = Spec.from_dict(spec)
    else:
        errors = validate(spec.to_dict())
        if errors:
            raise SpecErrors(errors)
    lk = linkage.get(spec.linkage.key)
    module = spec.legs.module or default_module(lk)
    sides = spec.legs.sides or (2 if lk.kind == "walker" else 1)
    phases = (None if spec.legs.phases_deg is None
              else tuple(math.radians(p) for p in spec.legs.phases_deg))
    d = BuildConfig()
    sheet = spec.materials.sheet or d.sheet
    thickness = spec.materials.thickness_mm          # the nominal is the default: one id
    if thickness is not None:
        thickness = None if float(thickness) == sheet_thickness(sheet) else float(thickness)
    try:
        config = BuildConfig(
            linkage=lk.key, module=module, robot=sides == 2, phases=phases,
            proportions=tuple(sorted(spec.linkage.params.items())),
            sheet=sheet, thickness=thickness,
            servo=spec.materials.servo or d.servo,
            frame_sheet=spec.materials.frame_sheet or d.frame_sheet,
            crank_sheet=spec.materials.crank_sheet or "",     # the linkage's (config)
            link_sheets=(None if spec.materials.link_sheets is None
                         else tuple(sorted(spec.materials.link_sheets.items()))),
            pillar=spec.constructions.pillar or d.pillar, pin=spec.constructions.pin or d.pin,
            crank=spec.constructions.crank or "", heads=spec.constructions.heads or d.heads,
            params=spec.fit.params(),
        )
    except ParamError as e:     # the validator should have said it first
        raise SpecErrors([SpecError("", str(e))]) from None
    resolved = _resolved(spec, config, module, sides)
    engine = engine_version()
    design = Design(design_id(resolved, engine), spec, resolved, config, engine,
                    _warnings(lk, config, sides), derived_from=derived_from, patch=patch)
    _attach_store(design, Store.of(store))
    return design


def config_warnings(config: BuildConfig, sides: int | None = None) -> list[str]:
    """What :func:`resolve` would warn about for this config (a measured thickness far
    from the sheet's nominal, a servo with no listed speed, one side of a walker): for the
    CLIs, which take the same options."""
    return _warnings(linkage.get(config.linkage), config,
                     (2 if config.robot else 1) if sides is None else sides)


def _warnings(lk, config: BuildConfig, sides: int) -> list[str]:
    warnings: list[str] = []
    if lk.kind == "walker" and sides == 1:
        warnings.append("sides = 1 builds one side (no chassis); the walk metrics still "
                        "model the two-sided robot")
    if len(lk.inputs) > 1:
        warnings.append(second_input_note(lk))
    if servos.get(config.servo).speed_rpm is None:
        warnings.append(f"servo {config.servo} lists no speed: speed_mm_s assumes "
                        f"{walk_model.DEFAULT_RPM:g} rpm")
    if config.thickness is not None:
        nominal = sheet_thickness(config.sheet)
        dev = (config.thickness - nominal) / nominal
        if abs(dev) > THICKNESS_TOLERANCE:
            warnings.append(
                f"materials.thickness_mm {config.thickness:g} is {abs(dev):.0%} "
                f"{'under' if dev < 0 else 'over'} {config.sheet}'s nominal {nominal:g} mm "
                f"(real sheets vary by about 8 %): the layer pitch follows the measured "
                f"thickness and every construction sizes its parts by it, so this design is "
                f"built for {config.thickness:g} mm layers, and a construction that can't be "
                f"built that thin says so at check (with the thickness that works)")
    return warnings


THICKNESS_TOLERANCE = 0.12     # a measured thickness this far from the sheet's nominal warns


def second_input_note(lk) -> str:
    """What a two-input mechanism gets told, at ``resolve`` and at the ``drive`` stage: v1
    builds one drive, so it can be resolved, checked for its program and its output, and
    drawn, but not planned or built; which one-input mechanisms can."""
    others = [k for k in linkage.available("mechanism")
              if len(linkage.get(k).inputs) == 1]
    return (f"{lk.key} has {len(lk.inputs)} inputs ({', '.join(lk.inputs)}) and v1 builds one "
            f"drive: check reads its program and its output, but plan, build, verify and "
            f"export stop at the drive stage (second_input_no_drive), a limit of v1, not of "
            f"the spec; the one-input mechanisms are {', '.join(others)}")


def _attach_store(design: Design, store: Store | None) -> None:
    """Record the design in ``store`` (the first record wins) and hang the store on the
    handle, so every operation reads and writes its stage there."""
    design.store = store
    if store is None:
        return
    rec = store.read_design(design.id)
    if rec is None:
        store.write_design(design)
    else:
        design.created_at = rec.get("created_at") or design.created_at
        design.derived_from, design.patch = rec.get("derived_from"), rec.get("patch")


def spec_of(config: BuildConfig, sides: int | None = None) -> dict:
    """The Spec document of a :class:`config.BuildConfig` (a CLI's options as a spec): its
    kind from the linkage, the proportions it overrides, module, phases and sides, the
    materials and constructions, and the fit fields that differ from the defaults; no
    targets. ``resolve(spec_of(config))`` is a design with that config, so ``spiderpig
    view --linkage ... --pin bolt`` can show what ``spiderpig build`` built."""
    lk = config.lk
    doc: dict = {"kind": lk.kind, "linkage": {"key": config.linkage}}
    if config.proportions:
        doc["linkage"]["params"] = dict(config.proportions)
    legs: dict = {"module": config.module,
                  "sides": (2 if config.robot else 1) if sides is None else int(sides)}
    if config.phases is not None:
        legs["phases_deg"] = [round(math.degrees(p), 6) for p in config.phases]
    doc["legs"] = legs
    mats: dict = {"sheet": config.sheet, "servo": config.servo,
                  "frame_sheet": config.frame_sheet, "crank_sheet": config.crank_sheet}
    if config.thickness is not None:
        mats["thickness_mm"] = config.thickness
    if config.link_sheets is not None:
        mats["link_sheets"] = dict(config.link_sheets)
    doc["materials"] = mats
    doc["constructions"] = {"pillar": config.pillar, "pin": config.pin, "crank": config.crank,
                            "heads": config.heads}
    default = Params()
    fit = {k: getattr(config.params, k) for k in FIT_FIELDS
           if getattr(config.params, k) != getattr(default, k)}
    if fit:
        doc["fit"] = fit
    return doc


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


def _resolved(spec: Spec, config: BuildConfig, module: str, sides: int) -> dict:
    """The spec with every inferred value written in (defaults of the engine included, so
    a stored design never depends on a default that later moves)."""
    design = config.design_json()
    fit = {k: getattr(config.params, k) for k in config.params.__dataclass_fields__}
    fit["kerf_mm"] = spec.fit.kerf_mm     # None: each sheet's service kerf (layout.sheet_kerf)
    fit["sheet_size_mm"] = list(spec.fit.sheet_size_mm or sheet_size(config.sheet))
    targets = {s: {} for s in SECTIONS}
    for f, t in spec.targets():
        targets[f.section][f.name] = t.to_dict(hard=effective_hard(t, f))
    if spec.allowance_usd is not None:      # the budget's one plain number
        targets["budget"][ALLOWANCE] = spec.allowance_usd
    return {
        "version": spec.version, "kind": spec.kind,
        "linkage": {"key": config.linkage, "params": design["proportions"]},
        "legs": {"module": module, "phases_deg": design["phases_deg"], "sides": sides},
        "motion": targets["motion"], "size": targets["size"], "budget": targets["budget"],
        "materials": {"sheet": config.sheet, "thickness_mm": config.thickness,
                      "pitch_mm": config.pitch, "servo": config.servo,
                      "frame_sheet": config.frame_sheet, "crank_sheet": config.crank_sheet,
                      "link_sheets": link_sheets(config)},
        "constructions": {"pillar": config.pillar, "pin": config.pin, "crank": config.crank,
                          "heads": config.heads},
        "fit": fit,
        "outputs": list(spec.outputs),
    }


# ---------------------------------------------------------------------------
# check, plan, explain, recommend
# ---------------------------------------------------------------------------


def _record(design: Design, op: str, seconds: float, ok: bool, cached: bool = False) -> None:
    entry = design.record(op, seconds, ok, cached)
    if design.store is not None:
        design.store.log(design.id, entry)


def _commit(design: Design, stage: str, rep, op: str | None = None, write: bool = True,
            cached: bool = False, seconds: float | None = None):
    """Put a finished report on the handle, log the operation (``seconds``: what this
    call took, else the report's), and write it to the store (``write``; one served from
    the store, ``cached``, is only logged)."""
    design.reports[stage] = rep
    _record(design, op or stage, rep.seconds if seconds is None else seconds, rep.ok, cached)
    if ran_out(rep):
        write = False       # the planner's CPU budget, not the design: never a stored verdict
    if design.edited and stage in EDITED_STAGES:
        write = False       # the edited parts' (the store's are the unedited design's)
    if write and not cached and design.store is not None:
        design.store.write_report(design, stage, rep)
    return rep


def _finish(design: Design, stage: str, rep, t0: float, **kw):
    rep.ok = not rep.failures
    rep.seconds = round(time.time() - t0, 3)
    return _commit(design, stage, rep, **kw)


def _stored(design: Design, stage: str, current: bool = True, variant: str | None = None
            ) -> dict | None:
    """The stage's file in the design's store (``current``: only one written by the
    running engine version; ``variant``: the copy kept per variant, a verify's level),
    else ``None``."""
    if design.store is None:
        return None
    doc = design.store.read_report(design.id, stage, variant)
    if doc is None or (current and doc.get("engine_version") != design.engine_version):
        return None
    return doc


def _cached(design: Design, stage: str, cls, op: str | None = None, **need):
    """The stage's report from the handle, else from the store when valid for the running
    engine (then put on the handle and logged as cached); ``need`` are field values it
    must match (a verify's ``level``: the store keeps one report per level, so the levels
    don't evict each other)."""
    rep = design.reports.get(stage)
    if (rep is not None and not ran_out(rep)
            and all(getattr(rep, k, None) == v for k, v in need.items())):
        return rep
    if design.edited and stage in EDITED_STAGES:
        return None         # the store's are the unedited design's
    t0 = time.time()
    level = need.get("level") if stage == "verify" else None
    doc = _stored(design, stage, variant=str(level)) if level else None
    if doc is None:
        doc = _stored(design, stage)
    if doc is None or any(doc.get(k) != v for k, v in need.items()):
        return None
    rep = cls.from_dict(doc)
    if ran_out(rep):            # (written before such reports stopped being stored)
        return None
    return _commit(design, stage, rep, op, cached=True, seconds=time.time() - t0)


def _template(design: Design):
    """The side's kinematic template (built once per handle)."""
    if design.template is None:
        design.template = _template_for(design.config)
    return design.template


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


def ran_out(rep) -> bool:
    """Did any of this report's failures come from the planner's CPU budget running out
    (``no_plan_in_time``)? Such a report (a plan, or a build or verify behind it) is not a
    verdict on the design: it is never written to the store or served from it."""
    return any(getattr(f, "code", None) == "no_plan_in_time"
               for f in getattr(rep, "failures", None) or ())


WARNING_LOGGERS = ("spiderpig.construction", "spiderpig.servos", "spiderpig.hardware")


@contextmanager
def capture_warnings(names: tuple[str, ...] = WARNING_LOGGERS):
    """Collect what the constructions warn about while a stage runs (a printed snap that
    overstrains, a servo model that can't be had), deduplicated in order, so a report
    carries them instead of only the server's stderr."""
    seen: dict[str, None] = {}

    class _Collect(logging.Handler):
        def emit(self, record: logging.LogRecord) -> None:
            seen.setdefault(record.getMessage(), None)

    handler = _Collect(level=logging.WARNING)
    loggers = [logging.getLogger(n) for n in names]
    propagated = [lg.propagate for lg in loggers]
    for lg in loggers:
        lg.addHandler(handler)
        lg.propagate = False      # on the report, not (again) on a terminal's stderr
    out: list[str] = []
    try:
        yield out
    finally:
        for lg, p in zip(loggers, propagated, strict=True):
            lg.removeHandler(handler)
            lg.propagate = p
        out += list(seen)


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

    for stage in stages:
        design.store.report_path(design.id, stage).unlink(missing_ok=True)
        for level in LEVELS if stage == "verify" else ():
            design.store.report_path(design.id, stage, level).unlink(missing_ok=True)


EDITED_STAGES = ("export", "verify")
"""What an edited handle (:attr:`Design.edited`) neither reads from nor writes to the store:
the store's are the unedited design's."""


def _forget(design: Design, *stages: str) -> None:
    """Drop ``stages``' reports from the handle, so the next call computes them afresh (an
    edited handle never reads the store's: :data:`EDITED_STAGES`)."""
    for stage in stages:
        design.reports.pop(stage, None)
