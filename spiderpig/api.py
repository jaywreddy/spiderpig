"""The operations: pure, synchronous functions of a :class:`spiderpig.design.Design`.

    from spiderpig import api
    design = api.resolve({"kind": "walker", "linkage": {"key": "klann"}})
    api.check(design); api.plan(design); api.walk(design)
    api.build(design); api.verify(design, "standard"); api.export(design, out_dir="out")

Each operation maps onto the engine's own pass (:func:`check` onto the
program checks and the planner's static stage, :func:`plan` onto
:func:`fabricate.design_side`, :func:`walk` onto :func:`walk.api_payload`,
:func:`build` onto :func:`fabricate.fabricate`, :func:`export` onto what
``spiderpig build`` writes), stores its report on the handle
(``design.reports[stage]``) and returns it. A design that merely fails a
stage gets a report with ``ok = False`` and a :class:`spiderpig.failure.Failure`
(stage, code, culprits, numbers, checked recommendations); operations raise
only for programming errors and an invalid spec (:class:`SpecErrors` from
:func:`resolve`).

Every operation is memoised on the handle: :func:`plan` runs :func:`check`
first if it hasn't run, :func:`build` runs :func:`plan`; a report already on
the handle is returned as is (``force=True`` recomputes).

With a store (:mod:`spiderpig.store`; the project's by default, ``store=None``
for memory only), each operation first reads its stage from the design's
folder when the file is valid for the running engine version, else computes
and writes it: a plan is re-made and verified on reload, a build's parts come
back from their STEP files, and every operation lands in the design's log.
:func:`load` opens a recorded design, :func:`derive` resolves a patched spec
with its parent recorded, :func:`compare` diffs two designs' specs and
reports, :func:`list_designs` and :func:`gc` manage the folder.
"""

from __future__ import annotations

import contextlib
import logging
import math
import time
import warnings as pywarnings
from contextlib import contextmanager
from dataclasses import dataclass, field, fields, replace
from pathlib import Path

import numpy as np

from spiderpig import linkage, servos
from spiderpig import walk as walk_model
from spiderpig.config import BuildConfig, ParamError
from spiderpig.construction.base import Build, ConstructionError, Params
from spiderpig.construction.contract import MAX_OUTSIDE, TOL, _outside, bad_solids, clashes
from spiderpig.construction.crank import CrankRoute, Run
from spiderpig.construction.envelope import claimed_solid
from spiderpig.construction.robot import FrameTies, assemble_robot
from spiderpig.design import Design, Part, design_id, engine_version, jsonable, spec_hash
from spiderpig.fabricate import (
    SideDesign,
    design_side,
    fabricate_side,
    ground_clearance,
    side_problem,
    static_stage,
)
from spiderpig.fabricate import template_for as _template_for
from spiderpig.failure import Failure, Recommendation, apply_patch, merge_patch
from spiderpig.hardware.bom import BomLine, bom_from_mechanism, group_made
from spiderpig.hardware.catalog import sheet_name, sheet_size, sheet_thickness
from spiderpig.hardware.mass import filament_density, material_of, part_props
from spiderpig.layout import DEFAULT_KERF, save_sheets
from spiderpig.spec import (
    FIT_FIELDS,
    SECTIONS,
    Spec,
    SpecError,
    SpecErrors,
    default_module,
    effective_hard,
    nearest,
    validate,
)
from spiderpig.stack import ClearanceError, PlanError, verify_plan
from spiderpig.store import PROJECT, Store, diff_json, report_doc

log = logging.getLogger("spiderpig")

# ---------------------------------------------------------------------------
# Reports
# ---------------------------------------------------------------------------


class Report:
    """A stage report: a dataclass whose JSON form (:func:`spiderpig.design.jsonable`) a
    store writes and :meth:`from_dict` reads back (unknown keys ignored)."""

    @classmethod
    def from_dict(cls, d):
        kw = {}
        for f in fields(cls):
            if f.name not in d:
                continue
            v = d[f.name]
            if f.name == "failures":
                v = [Failure.from_dict(x) for x in v]
            elif f.name == "rows":
                from spiderpig.verify import Row

                v = [Row.from_dict(x) for x in v]
            elif f.name == "envelope_mm" and v is not None:
                v = tuple(v)
            kw[f.name] = v
        return cls(**kw)


@dataclass
class CheckReport(Report):
    """The stages before any layer: the program's loop closures (``steps``), a mechanism's
    ``output`` check, one leg's ``foot_path`` numbers (a walker), the ``drive``, the static
    ``clearances`` (links that can never share a keep-out's layers), the crank's
    ``crank_facts`` (links that need O free and the crank points that clear them, the body's
    underside) and the ``ground_clearance_mm``."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    steps: list[dict] = field(default_factory=list)
    output: dict | None = None
    foot_path: dict | None = None
    drive: dict = field(default_factory=dict)
    clearances: list[dict] = field(default_factory=list)
    crank_facts: dict | None = None
    ground_clearance_mm: float | None = None
    lowest_body_part: str = ""      # which body shape sets the ground clearance
    seconds: float = 0.0


@dataclass
class PlanReport(Report):
    """The layer plan of one side: every link's layer, the stack (``n_layers`` of
    ``pitch_mm``: ``height_mm``), the crank's ``route`` (runs along its posts, whether it
    keeps its bottom bearing), whether it is proven the thinnest (``optimal``, ``proof``),
    and the plan's table (``table``). A failure carries the blockers and recommendations.
    ``reused`` names the stored plan this one was re-made and verified from (``"store"``:
    the design's own; another design's id: the same spec on another engine version)."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    layers: dict[str, int] = field(default_factory=dict)
    top: int | None = None
    n_layers: int | None = None
    height_mm: float | None = None
    pitch_mm: float | None = None
    route: dict | None = None
    optimal: bool | None = None
    proof: str = ""
    cost: int | None = None
    ground_clearance_mm: float | None = None
    table: str = ""
    reused: str | None = None
    warnings: list[str] = field(default_factory=list)   # the constructions' (a strained snap)
    seconds: float = 0.0


@dataclass
class WalkReport(Report):
    """The quasi-static walk model's metrics (:func:`walk.api_payload`) with each motion
    target's verdict (``rows``); ``skipped`` for a mechanism. Feet sit at their planned
    layers when the side is planned (``feet_z_planned``), the mass is the model's nominal
    one (``mass_nominal``) unless a build gave it."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    skipped: str | None = None
    metrics: dict | None = None
    mass_g: float | None = None
    mass_nominal: bool = True
    feet_z_planned: bool = False
    servo: dict = field(default_factory=dict)
    rows: list = field(default_factory=list)
    notes: list[str] = field(default_factory=list)      # why a stride reads zero
    seconds: float = 0.0


@dataclass
class BuildReport(Report):
    """Every part built at crank angle ``t`` (the manifest: ``parts``), how many of each
    fabrication, the total mass, the envelope (x, y, z extents) and the mechanism's meta."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    t: float | None = None
    n_parts: int = 0
    counts: dict = field(default_factory=dict)
    mass_g: float | None = None
    envelope_mm: tuple[float, float, float] | None = None
    meta: dict = field(default_factory=dict)
    parts: list[dict] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)   # the constructions' while building
    seconds: float = 0.0


@dataclass
class RecheckReport(Report):
    """The contract and clash checks over the parts as they now are: which parts were
    ``edited``, which the contract ``checked`` (inside their group's claims), the
    violations, the pairwise ``clashes`` and the ``bad_solids``."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    edited: list[str] = field(default_factory=list)
    checked: list[str] = field(default_factory=list)
    contract: list[dict] = field(default_factory=list)
    clashes: list[dict] = field(default_factory=list)
    bad_solids: list[dict] = field(default_factory=list)
    seconds: float = 0.0


@dataclass
class ExportReport(Report):
    """What :func:`export` wrote: the ``files`` and the ``manifest`` (also written as
    ``manifest.json``)."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    out_dir: str = ""
    formats: list[str] = field(default_factory=list)
    files: list[str] = field(default_factory=list)
    manifest: dict = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    seconds: float = 0.0


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
            pillar=spec.constructions.pillar or d.pillar, pin=spec.constructions.pin or d.pin,
            crank=spec.constructions.crank or d.crank, params=spec.fit.params(),
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
    mats: dict = {"sheet": config.sheet, "servo": config.servo}
    if config.thickness is not None:
        mats["thickness_mm"] = config.thickness
    doc["materials"] = mats
    doc["constructions"] = {"pillar": config.pillar, "pin": config.pin, "crank": config.crank}
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
    return BuildConfig(
        linkage=lk.key, module=legs["module"], robot=legs["sides"] == 2,
        phases=tuple(math.radians(p) for p in legs["phases_deg"]),
        proportions=tuple(sorted(resolved["linkage"]["params"].items())),
        sheet=mat["sheet"], thickness=mat.get("thickness_mm"), servo=mat["servo"],
        pillar=cons["pillar"], pin=cons["pin"], crank=cons["crank"],
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
    spec = Spec.from_dict(spec_doc)
    config = _config_from_resolved(resolved)
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
    fit["kerf_mm"] = spec.fit.kerf_mm if spec.fit.kerf_mm is not None else DEFAULT_KERF
    fit["sheet_size_mm"] = list(spec.fit.sheet_size_mm or sheet_size(config.sheet))
    targets = {s: {} for s in SECTIONS}
    for f, t in spec.targets():
        targets[f.section][f.name] = t.to_dict(hard=effective_hard(t, f))
    return {
        "version": spec.version, "kind": spec.kind,
        "linkage": {"key": config.linkage, "params": design["proportions"]},
        "legs": {"module": module, "phases_deg": design["phases_deg"], "sides": sides},
        "motion": targets["motion"], "size": targets["size"], "budget": targets["budget"],
        "materials": {"sheet": config.sheet, "thickness_mm": config.thickness,
                      "pitch_mm": config.pitch, "servo": config.servo},
        "constructions": {"pillar": config.pillar, "pin": config.pin, "crank": config.crank},
        "fit": fit,
        "outputs": list(spec.outputs),
    }


# ---------------------------------------------------------------------------
# Linkage cards
# ---------------------------------------------------------------------------


def list_linkages(kind: str | None = None) -> list[dict]:
    """Every registered linkage (``kind``: ``walker`` / ``mechanism`` keeps those): key,
    name, family, kind, its modules (legs per side) and parameters."""
    out = []
    for key in linkage.available(kind):
        lk = linkage.get(key)
        out.append({"key": key, "name": lk.name, "family": lk.family or key, "kind": lk.kind,
                    "modules": {m: len(legs) for m, legs in lk.leg_modules.items()},
                    "params": {k: float(v) for k, v in lk.params.items()},
                    "feet_per_leg": len(lk.feet),
                    "output": lk.output.motion if lk.output else None})
    return out


def describe(key: str) -> dict:
    """One linkage's card: its parameters (default, angle or length, which only scale it),
    links and labels, feet or output, modules with their default phases, the closures at
    the defaults (margins, transmission angles, toggles) and one foot's path numbers (a
    walker) or the output check (a mechanism)."""
    try:
        lk = linkage.get(key)
    except KeyError:
        near = nearest(key, linkage.available())
        raise KeyError(f"unknown linkage {key!r}" + (f"; did you mean {near!r}?" if near
                                                     else "")) from None
    scale = linkage.scale_params(lk)
    card = {
        "key": key, "name": lk.name, "family": lk.family or key, "kind": lk.kind,
        "source": lk.source, "notes": lk.notes,
        "params": [{"name": k, "default": float(v), "angle": k in lk.angles,
                    "signed": k in lk.signed, "scale": k in scale}
                   for k, v in lk.params.items()],
        "scale_params": list(scale),
        "modules": {m: {"legs": len(legs),
                        "phases_deg": [round(math.degrees(ph), 6) for _, ph in legs],
                        "orientations": [o for o, _ in legs]}
                    for m, legs in lk.leg_modules.items()},
        "links": {b: {"joints": list(js), "label": lk.labels.get(b, "")}
                  for b, (js, _) in lk.links.items()},
        "frame": list(lk.frame), "crank": list(lk.crank), "inputs": list(lk.inputs),
        "feet": [list(f) for f in lk.feet],
        "output": jsonable(lk.output) if lk.output else None,
        "closures": [_step_dict(s) for s in lk.check() if s.kind == "closure"],
    }
    if lk.kind == "walker":
        card["foot_path"] = foot_path(lk, {})
        card["sensitivity"] = sensitivity(lk)
        for m, doc in card["modules"].items():
            s = module_stride(key, m)
            doc["stride_mm"] = s
            doc["walks"] = s is not None and s >= NO_TRAVEL_MM
    else:
        try:
            card["output_check"] = _output_dict(lk.output_check())
        except linkage.AssemblyError as e:
            card["output_check"] = {"error": str(e)}
        else:
            card["sensitivity"] = output_sensitivity(lk)
    return card


OUTPUT_SENSITIVITY_KEYS = ("stroke_mm", "straightness_mm", "extent_x_mm", "extent_y_mm",
                           "on_line_fraction", "rotation_deg", "swing_deg", "dwell_deg")


def _output_numbers(lk: linkage.Linkage, params: dict) -> dict[str, float | None]:
    c = _output_dict(lk.output_check(params or None))
    ex = c.get("extent_mm") or [None, None]
    return {"stroke_mm": c["stroke_mm"], "straightness_mm": c["straightness_mm"],
            "extent_x_mm": ex[0], "extent_y_mm": ex[1], "on_line_fraction": c["on_line_fraction"],
            "rotation_deg": c["rotation_deg"], "swing_deg": c["swing_deg"],
            "dwell_deg": c["dwell_deg"]}


def output_sensitivity(lk: linkage.Linkage) -> dict:
    """A mechanism's answer to the walkers' :func:`sensitivity`: what +10 % of each length
    (+5° of an angle) does to the output's numbers at the defaults, in percent (the
    stroke, the straightness band, the path's extent, and the rotation, swing or dwell it
    measures), so a designer knows a stroke that scales with ``unit`` from one that
    depends on a proportion. ``null`` where the loops no longer close."""
    base = _output_numbers(lk, {})
    keys = [k for k in OUTPUT_SENSITIVITY_KEYS if base.get(k) is not None]
    out = {}
    for name, default in lk.params.items():
        angle = name in lk.angles
        value = (float(default) + SENSITIVITY_DEG if angle
                 else float(default) * (1 + SENSITIVITY_STEP))
        try:
            with np.errstate(invalid="ignore", divide="ignore"):
                got = _output_numbers(lk, {name: value})
        except (ValueError, linkage.AssemblyError):
            out[name] = None
            continue
        if not all(got.get(k) is not None and math.isfinite(got[k]) for k in keys):
            out[name] = None
            continue
        out[name] = {"step": f"+{SENSITIVITY_DEG:g}°" if angle else f"+{SENSITIVITY_STEP:.0%}",
                     **{k: (round(100.0 * (got[k] - base[k]) / base[k], 1) if base[k] else None)
                        for k in keys}}
    return out


def foot_path(lk: linkage.Linkage, params: dict, n: int = 720) -> dict:
    """One foot's path over a revolution (leg 0's first foot, its defaults unless
    ``params``): ``lift_mm`` (vertical travel), the stance stride and fraction within 2 mm
    of the lowest point, the crank radius, the leg's height and width."""
    ts = 2.0 * math.pi * np.arange(n) / n
    pts = lk.solve(params=params or None).evaluate(ts)
    f = pts[lk.feet[0][1]]
    y0 = float(f[:, 1].min())
    stance = f[:, 1] <= y0 + 2.0
    top = max(float(pts[j][:, 1].max()) for j in lk.points)
    xs = np.concatenate([pts[j][:, 0] for j in lk.points])
    return {
        "lift_mm": float(np.ptp(f[:, 1])),
        "stance_stride_mm": float(np.ptp(f[stance, 0])) if stance.any() else 0.0,
        "stance_fraction": float(stance.mean()),
        "crank_radius_mm": float(np.linalg.norm(pts[lk.crank[1]][0])),
        "height_mm": top - y0,
        "width_mm": float(np.ptp(xs)),
    }


SENSITIVITY_STEP = 0.10     # a length parameter is moved by +10 %
SENSITIVITY_DEG = 5.0       # an angle by +5°


def sensitivity(lk: linkage.Linkage) -> dict:
    """What each parameter does to one foot's path, from its default: the percent change
    of ``lift_mm``, ``stance_stride_mm``, ``height_mm`` and ``width_mm`` for +10 % of a
    length (``+5°`` of an angle), so a designer knows which raise the leg, lengthen the
    stride or grow the envelope before trying. ``null`` where the loops no longer close."""
    base = foot_path(lk, {})
    keys = ("lift_mm", "stance_stride_mm", "height_mm", "width_mm")
    out = {}
    for name, default in lk.params.items():
        angle = name in lk.angles
        value = (float(default) + SENSITIVITY_DEG if angle
                 else float(default) * (1 + SENSITIVITY_STEP))
        try:
            with np.errstate(invalid="ignore", divide="ignore"):   # a loop that can't close
                fp = foot_path(lk, {name: value})
        except (ValueError, linkage.AssemblyError):
            out[name] = None
            continue
        if not all(math.isfinite(fp[k]) for k in keys):
            out[name] = None
            continue
        out[name] = {"step": f"+{SENSITIVITY_DEG:g}°" if angle else f"+{SENSITIVITY_STEP:.0%}",
                     **{k: (round(100.0 * (fp[k] - base[k]) / base[k], 1) if base[k] else None)
                        for k in keys}}
    return out


def _step_dict(s) -> dict:
    return {"point": s.point, "kind": s.kind, "refs": list(s.refs), "radii_mm": s.radii,
            "margin_mm": s.margin_mm, "worst_deg": s.worst_deg, "fails_deg": s.fails_deg,
            "transmission_deg": s.angle_deg, "fail_fraction": s.fail_fraction,
            "toggles": s.toggles, "text": s.describe()}


def _output_dict(c) -> dict:
    return {"name": c.output.name, "motion": c.output.motion, "point": c.output.point,
            "extent_mm": list(c.extent_mm), "stroke_mm": c.stroke_mm,
            "straightness_mm": c.straightness_mm, "on_line_fraction": c.on_line,
            "rotation_deg": c.rotation_deg, "swing_deg": c.swing_deg, "dwell_deg": c.dwell_deg,
            "broken": c.broken, "text": c.describe()}


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
    if write and not cached and design.store is not None:
        design.store.write_report(design, stage, rep)
    return rep


def _finish(design: Design, stage: str, rep, t0: float, **kw):
    rep.ok = not rep.failures
    rep.seconds = round(time.time() - t0, 3)
    return _commit(design, stage, rep, **kw)


def _stored(design: Design, stage: str, current: bool = True) -> dict | None:
    """The stage's file in the design's store (``current``: only one written by the
    running engine version), else ``None``."""
    if design.store is None:
        return None
    doc = design.store.read_report(design.id, stage)
    if doc is None or (current and doc.get("engine_version") != design.engine_version):
        return None
    return doc


def _cached(design: Design, stage: str, cls, op: str | None = None, **need):
    """The stage's report from the handle, else from the store when valid for the running
    engine (then put on the handle and logged as cached); ``need`` are field values it
    must match (a verify's ``level``)."""
    rep = design.reports.get(stage)
    if rep is not None and all(getattr(rep, k, None) == v for k, v in need.items()):
        return rep
    t0 = time.time()
    doc = _stored(design, stage)
    if doc is None or any(doc.get(k) != v for k, v in need.items()):
        return None
    return _commit(design, stage, cls.from_dict(doc), op, cached=True, seconds=time.time() - t0)


def _template(design: Design):
    """The side's kinematic template (built once per handle)."""
    if design.template is None:
        design.template = _template_for(design.config)
    return design.template


def check(design: Design, force: bool = False) -> CheckReport:
    """The stages before any layer (:class:`CheckReport`): every loop closes (else
    ``program``), a mechanism's output keeps its promises (else ``output``), the drive
    (``drive``: one servo turns ``t``), the static facts and the crank's route points
    (else ``static``, with what would clear it), the ground clearance."""
    if not force and (rep := _cached(design, "check", CheckReport)) is not None:
        return rep
    t0 = time.time()
    cfg, lk = design.config, design.lk
    params = dict(cfg.proportions)
    rep = CheckReport()
    steps = lk.check(params)
    rep.steps = [_step_dict(s) for s in steps]
    bad = next((s for s in steps if s.fails_deg is not None), None)
    if bad is not None:
        rep.failures.append(Failure(
            "program", "loop_cannot_close", f"{lk.key}: {bad.describe()}",
            culprits=[{"joint": bad.point, "refs": list(bad.refs)}],
            numbers={"margin_mm": bad.margin_mm, "worst_deg": bad.worst_deg,
                     "fails_deg": bad.fails_deg, "fail_fraction": bad.fail_fraction,
                     "radii_mm": bad.radii}))
        return _finish(design, "check", rep, t0)
    if lk.output is not None:
        oc = lk.output_check(params)
        rep.output = _output_dict(oc)
        if oc.broken:
            rep.failures.append(Failure(
                "output", "promise_broken", f"{lk.key}: {oc.broken}",
                culprits=[{"output": oc.output.name, "point": oc.output.point,
                           "body": oc.output.link}],
                numbers={k: v for k, v in rep.output.items()
                         if isinstance(v, (int, float)) and v is not None}))
            return _finish(design, "check", rep, t0)
    else:
        rep.foot_path = foot_path(lk, params)
    rep.drive = {"servo": cfg.servo, "rpm_max": walk_model.servo_info(cfg.servo)["rpm_max"],
                 "inputs": list(lk.inputs)}
    try:
        tmpl = design.template = _template_for(cfg)
        ctx, _, problem = side_problem(tmpl, replace(cfg, robot=False))
    except ValueError as e:      # AssemblyError / OutputError (caught above) / ConstructionError
        fl = Failure.from_exception(e, lk=lk)
        if isinstance(e, ConstructionError) and getattr(e, "changes", ()):
            from spiderpig.recommend import construction_fix

            recs, notes = construction_fix(replace(cfg, robot=False), e)
            fl.recommendations += [Recommendation.from_engine(r, lk) for r in recs]
            fl.notes += notes
        rep.failures.append(fl)
        return _finish(design, "check", rep, t0)
    rep.clearances = [{"link": c.link, "keepout": c.keepout.owner, "where": c.keepout.where,
                       "dist_mm": c.dist, "need_mm": c.need, "text": c.describe()}
                      for c in problem.clearances]
    if problem.router is not None:
        f = problem.router.facts
        rep.crank_facts = {
            "o_free": {k: max(v, 0.0) for k, v in f.o_free.items()},
            "hosts": {k: list(v) for k, v in f.hosts.items()},
            "detours": [{"name": d.name, "r_mm": d.r, "angle_deg": d.angle, "sweep_mm": d.sweep}
                        for d in f.detours],
            "underside_lowest_mm": f.envelope.lowest if f.envelope is not None else None,
            "allow_mm": f.allow if math.isfinite(f.allow) else None,
        }
    rep.ground_clearance_mm = ground_clearance(tmpl, ctx)
    under = ctx.interfaces.get("underside")
    # which body shape sets the clearance: a walker's question (a mechanism has no feet)
    rep.lowest_body_part = (under.lowest_part
                            if under is not None and rep.ground_clearance_mm is not None else "")
    try:
        static_stage(tmpl, problem, cfg)
    except ClearanceError as e:
        fl = Failure.from_exception(e, lk=lk)
        fails = problem.router.facts.failures
        fl.culprits = [{"body": f.link, "point": f.pin, "dist_mm": f.dist, "need_mm": f.need,
                        "post_mm": f.post, "link_radius_mm": f.link_r, "margin_mm": f.margin,
                        "detour": f.detour, "allow_mm": f.allow} for f in fails]
        if fails:
            fl.numbers = {"dist_mm": fails[0].dist, "need_mm": fails[0].need}
        rep.failures.append(fl)
    return _finish(design, "check", rep, t0)


def plan(design: Design, force: bool = False) -> PlanReport:
    """The layer plan of one side (:class:`PlanReport`), from :func:`fabricate.design_side`
    (cached by the engine per template and config). A ``plan`` failure carries the
    blockers (count, the two shapes, gap, need) and checked recommendations.

    With a store, a recorded plan is re-made through :meth:`stack.StackProblem.plan` and
    checked with :func:`stack.verify_plan` before use (the design's own, else the same
    spec's on another engine version: ``reused``); one that no longer holds is solved
    again."""
    if not force:
        rep = design.reports.get("plan")
        if rep is not None and (design.side is not None or not rep.ok):
            return rep
        rep = _reuse_plan(design)
        if rep is not None:
            return rep
    t0 = time.time()
    rep = PlanReport()
    cr = check(design, force)
    if not cr.ok:
        rep.failures = list(cr.failures)
        return _finish(design, "plan", rep, t0)
    try:
        with capture_warnings() as warned:
            design.side = design_side(_template(design), design.config)
    except (PlanError, ConstructionError) as e:
        rep.failures.append(Failure.from_exception(e, lk=design.lk))
        return _finish(design, "plan", rep, t0)
    rep = _plan_report(design.side, rep)
    rep.warnings = list(warned)
    return _finish(design, "plan", rep, t0)


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


def _plan_report(d: SideDesign, rep: PlanReport | None = None) -> PlanReport:
    rep = rep or PlanReport()
    p = d.plan
    route = p.choices.get("crank")
    rep.layers = dict(p.layers)
    rep.top, rep.n_layers, rep.height_mm, rep.pitch_mm = p.top, p.top + 1, p.height, p.spec.pitch
    rep.route = (None if route is None else
                 {"runs": [{"at": r.at, "lo": r.lo, "hi": r.hi} for r in route.runs],
                  "bearing": route.bearing})
    rep.optimal, rep.proof, rep.cost = p.optimal, p.proof, p.cost
    rep.ground_clearance_mm = d.ground_clearance_mm
    rep.table = p.describe()
    return rep


def _reuse_plan(design: Design) -> PlanReport | None:
    """A stored plan the design can use, or ``None``: its own (a failing one only from
    the running engine; a valid one re-made and verified, then served as cached), else
    the newest plan of the same resolved spec under another engine version (verified on
    fresh sampling, then written as this design's, without the optimality proof)."""
    store = design.store
    if store is None:
        return None
    t0 = time.time()
    own = store.read_report(design.id, "plan")
    if own is not None:
        same = own.get("engine_version") == design.engine_version
        if not own.get("ok"):
            if same:
                return _commit(design, "plan", PlanReport.from_dict(own), cached=True)
            return None
        side = _remake_plan(design, own, same)
        if side is not None:
            design.side = side
            rep = _plan_report(side)
            rep.reused, rep.seconds = "store", round(time.time() - t0, 3)
            return _commit(design, "plan", rep, cached=same)
        if same:
            log.warning("%s: the stored plan no longer holds under this engine: re-solving",
                        design.id)
    for other, doc in store.find_plans(spec_hash(design.resolved), exclude=(design.id,))[:3]:
        side = _remake_plan(design, doc, same_engine=False)
        if side is not None:
            design.side = side
            rep = _plan_report(side)
            rep.reused, rep.seconds = other, round(time.time() - t0, 3)
            return _commit(design, "plan", rep)
    return None


def _remake_plan(design: Design, doc: dict, same_engine: bool) -> SideDesign | None:
    """The side of ``design`` with the layout of a stored plan, if every claim still
    clears: the groups and the problem are rebuilt from the template, the plan is re-made
    (:meth:`stack.StackProblem.plan`) and checked (:func:`stack.verify_plan`: on the
    plan's own sampling for the same engine, on fresh sampling for another). ``None``
    when the check fails or the design fails an earlier stage."""
    if not check(design).ok:
        return None
    tmpl, cfg = _template(design), replace(design.config, robot=False)
    try:
        ctx, groups, problem = side_problem(tmpl, cfg)
        static_stage(tmpl, problem)
        route = doc.get("route")
        choices = {} if route is None else {
            "crank": CrankRoute(tuple(Run(r["at"], int(r["lo"]), int(r["hi"]))
                                      for r in route["runs"]), bool(route.get("bearing", True)))}
        p = problem.plan({k: int(v) for k, v in doc["layers"].items()}, int(doc["top"]), choices)
        bad = verify_plan(p) if same_engine else verify_plan(p, tmpl)
    except (ValueError, KeyError, TypeError) as e:
        log.info("%s: stored plan not reusable: %s", design.id, e)
        return None
    if bad:
        log.info("%s: stored plan fails verification: %s", design.id, "; ".join(bad[:3]))
        return None
    if same_engine:
        p.optimal, p.proof = bool(doc.get("optimal")), doc.get("proof", "")
        p.cost = int(doc.get("cost") or 0)
    else:
        p.optimal, p.cost = False, int(doc.get("cost") or 0)
        p.proof = (f"re-verified under engine {design.engine_version} (planned under "
                   f"{doc.get('engine_version')}); not proven the thinnest here")
    return SideDesign(cfg, ctx, groups, p, list(problem.clearances), ground_clearance(tmpl, ctx))


def explain(design: Design) -> str:
    """Each stage's verdict on one side of the design, in prose (:func:`explain.explain_config`
    with the design's full config: servo, sheet, constructions and fit). The plan is the
    design's own (:func:`plan`: the handle's, the store's re-made, or solved once); a
    recorded failure is printed, not searched for again."""
    from spiderpig import explain as explain_module

    pr = plan(design)
    failure = None if pr.ok else "\n  ".join(f.message for f in pr.failures[:1]) or "failed"
    text = explain_module.explain_config(design.config, side=design.side if pr.ok else None,
                                         plan_failure=failure)
    if not pr.ok or not design.spec.targets():
        return text
    # 4. the spec's targets that check and plan can read, and what would meet a miss
    got = cheap_measures(design)
    lines = ["", "4. targets"]
    for f, t in design.spec.targets():
        v = got.get(f.path)
        if v is None:
            lines.append(f"  {f.path}: {t.describe()}: measured by verify (the walk, the build "
                         "or the BOM)")
            continue
        met, _ = t.check(float(v))
        lines.append(f"  {f.path}: {v:.4g} {f.unit} vs {t.describe()}: "
                     f"{'ok' if met else ('MISSED (hard)' if effective_hard(t, f) else 'missed')}")
    adv = advise(design)
    if adv.recommendations:
        lines += ["  what would meet it:", *(f"  {r.describe()}" for r in adv.recommendations)]
    lines += [f"  {n}" for n in adv.notes]
    return text + "\n".join(lines)


def recommend(design: Design) -> list[Recommendation]:
    """The checked recommendations of :func:`advise`: the failing stage's (a construction
    that can't be built, the static stage's, else the planner's), or, when every stage
    passes, a scale of the linkage that meets a missed target that scales with it; each
    with the spec patch that applies it. Empty when there is nothing to recommend
    (:func:`advise` says why in its ``notes``)."""
    return list(advise(design).recommendations)


@dataclass
class AdviceReport(Report):
    """What would move the design: ``stage`` is the failing stage the advice is for
    (``construction``, ``static``, ``plan``), ``target`` when every stage passes but a
    target is missed, or ``None`` when nothing is missed; ``recommendations`` are checked
    (each with its spec ``patch``); ``notes`` say what can't help, what wasn't checked,
    and which missed target has no lever the engine can compute."""

    ok: bool = True
    failures: list[Failure] = field(default_factory=list)
    stage: str | None = None
    recommendations: list[Recommendation] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    seconds: float = 0.0


SCALED_METRICS = ("motion.stroke_mm", "motion.straightness_mm", "motion.lift_mm")
CHEAP_OUTPUT = ("stroke_mm", "straightness_mm", "on_line_fraction", "rotation_deg",
                "swing_deg", "dwell_deg")


def cheap_measures(design: Design) -> dict[str, float]:
    """The metrics a target can be read against from ``check`` and ``plan`` alone: a
    mechanism's output numbers, a walker's lift and ground clearance, the stack."""
    out: dict[str, float] = {}
    cr, pr = design.reports.get("check"), design.reports.get("plan")
    if cr is not None and cr.ok:
        if cr.output:
            out.update({f"motion.{k}": cr.output[k] for k in CHEAP_OUTPUT
                        if cr.output.get(k) is not None})
        if cr.foot_path:
            out["motion.lift_mm"] = cr.foot_path["lift_mm"]
        if cr.ground_clearance_mm is not None:
            out["motion.ground_clearance_mm"] = cr.ground_clearance_mm
    if pr is not None and pr.ok and pr.height_mm is not None:
        out["size.stack_mm"] = pr.height_mm
    return out


def missed_targets(design: Design) -> list[tuple[str, float, object]]:
    """``(path, value, target)`` for every spec target :func:`cheap_measures` can read that
    the design misses."""
    got = cheap_measures(design)
    out = []
    for f, t in design.spec.targets():
        v = got.get(f.path)
        if v is None:
            continue
        met, _ = t.check(float(v))
        if not met:
            out.append((f.path, float(v), t))
    return out


def measure_config(config: BuildConfig) -> dict[str, float]:
    """The scaled metrics of ``config`` without a design: a mechanism's stroke and
    straightness, a walker's lift (what :func:`recommend.target_scale` re-measures)."""
    lk = config.lk
    params = dict(config.proportions)
    if lk.output is not None:
        c = _output_dict(lk.output_check(params or None))
        return {f"motion.{k}": c[k] for k in ("stroke_mm", "straightness_mm")
                if c.get(k) is not None}
    return {"motion.lift_mm": foot_path(lk, params)["lift_mm"]}


def advise(design: Design) -> AdviceReport:
    """What would move the design, checked (:class:`AdviceReport`). A stage that fails
    (``check``, then ``plan``) answers with its own recommendations and notes. When every
    stage passes, the targets :func:`cheap_measures` can read are compared with the spec:
    a missed stroke, straightness or lift is met by scaling the linkage
    (:func:`recommend.target_scale`: the least practical scale, measured again and
    planned); a missed stack that is proven the thinnest, or a clearance, gets a note
    saying which levers are left."""
    from spiderpig import verify as _verify
    from spiderpig.recommend import target_scale

    t0 = time.time()
    rep = AdviceReport()
    cr = check(design)
    failing = cr if not cr.ok else None
    pr = None
    if failing is None:
        pr = plan(design)
        failing = pr if not pr.ok else None
    if failing is not None:
        f = failing.failures[0] if failing.failures else None
        rep.stage = f.stage if f is not None else None
        rep.recommendations = list(f.recommendations) if f is not None else []
        rep.notes = list(f.notes) if f is not None else []
        return _finish(design, "advice", rep, t0, write=False)
    misses = missed_targets(design)
    if not misses:
        rep.notes.append("every stage passes and no target check or plan can read is missed; "
                         "verify measures the rest (the walk, the build, the BOM)")
        return _finish(design, "advice", rep, t0, write=False)
    rep.stage = "target"
    scaled = [m for m in misses if m[0] in SCALED_METRICS]
    if scaled:
        rec, note = target_scale(design.config, scaled, measure_config)
        if rec is not None:
            rep.recommendations.append(Recommendation.from_engine(rec, design.lk))
        if note:
            rep.notes.append(note)
    for path, v, t in misses:
        if path in SCALED_METRICS:
            continue
        if path == "size.stack_mm" and pr is not None and pr.optimal:
            rep.notes.append(f"size.stack_mm {v:g} vs {t.describe()}: "
                             + _verify.stack_floor_note(design, pr))
        elif path == "motion.ground_clearance_mm":
            rep.notes.append(f"motion.ground_clearance_mm {v:.1f} vs {t.describe()}: the "
                             f"lowest point is {cr.lowest_body_part or 'the body'}; it grows "
                             f"with the linkage's scale and shrinks with the servo's body and "
                             f"the chassis, none exactly, so no scale is computed: derive on "
                             f"the scale parameter or the servo and read check")
        else:
            rep.notes.append(f"{path} {v:g} vs {t.describe()}: missed; no lever the engine "
                             f"can compute for it")
    return _finish(design, "advice", rep, t0, write=False)


# ---------------------------------------------------------------------------
# walk
# ---------------------------------------------------------------------------


def walk(design: Design, force: bool = False) -> WalkReport:
    """The quasi-static walk model of the robot (:func:`walk.api_payload`; no parts):
    its metrics and each motion target's verdict. A mechanism is skipped."""
    if not force and (rep := _cached(design, "walk", WalkReport)) is not None:
        return rep
    t0 = time.time()
    rep = WalkReport()
    cfg = design.config
    if design.kind == "mechanism":
        rep.skipped = f"{cfg.linkage} is a mechanism: it has an output, not feet"
        return _finish(design, "walk", rep, t0)
    feet_z = None
    if design.side is not None:
        try:
            feet_z = walk_model.foot_z_planned(cfg, design.side)
        except (ValueError, KeyError):
            feet_z = None
    payload = walk_model.api_payload(cfg, feet_z=feet_z)
    rep.servo = payload["servo"]
    rep.feet_z_planned = feet_z is not None
    if not payload["valid"]:
        rep.failures.append(Failure("walk", "linkage_invalid", payload["error"]))
        return _finish(design, "walk", rep, t0)
    rep.metrics = payload["metrics"]
    rep.mass_g = payload["mass_g"]
    from spiderpig import verify as _verify

    rep.rows = _verify.walk_rows(design, rep.metrics)
    if (note := no_travel_note(cfg, rep.metrics)) is not None:
        rep.notes.append(note)
        for r in rep.rows:
            if r.requirement in ("motion.stride_mm", "motion.speed_mm_s"):
                r.detail = note
    return _finish(design, "walk", rep, t0)


NO_TRAVEL_MM = 1.0      # a stride under this is no walk


def no_travel_note(cfg: BuildConfig, metrics: dict) -> str | None:
    """Why a walker's stride is (near) zero, when it is: the module's feet cancel each
    other in the quasi-static model, so the body stands and bobs (a mirrored pair at one
    phase, or two legs one way with nothing to take turns with), and which module of the
    linkage walks (:func:`describe` gives every module's stride)."""
    stride = float(metrics.get("stride_mm") or 0.0)
    if stride >= NO_TRAVEL_MM:
        return None
    lk = linkage.get(cfg.linkage)
    walking = {m: s for m in lk.leg_modules if m != cfg.module
               and (s := module_stride(cfg.linkage, m)) is not None and s >= NO_TRAVEL_MM}
    duty = metrics.get("duty") or []
    stands = all(float(d) >= 0.999 for d in duty) if duty else False
    phases = ([round(math.degrees(p), 1) for p in cfg.phases] if cfg.phases
              else "its default phases")
    others = ", ".join(f"{m} ({s:.0f} mm/rev)" for m, s in walking.items())
    return (f"no net travel: {cfg.linkage}'s {cfg.module} module at {phases} walks "
            f"{stride:.2g} mm per revolution in the quasi-static model"
            + (": every foot stays on the ground (duty 1.0) and the feet's pushes cancel, so "
               "the body stands and bobs" if stands else
               ": the feet's pushes cancel over the cycle")
            + (f"; of this linkage's modules, {others} walk" if walking
               else "; no other module of this linkage walks at its default phases")
            + "; describe(linkage) lists each module's stride_mm")


_MODULE_STRIDES: dict[tuple[str, str], float | None] = {}


def module_stride(key: str, module: str) -> float | None:
    """The walk model's stride (mm per revolution, the two-sided robot) of a linkage's
    module at its default phases and proportions, the feet at a nominal spacing
    (:func:`walk.foot_z_guess`: no layer plan is searched for a card); ``None`` when the
    model can't use it."""
    k = (key, module)
    if k not in _MODULE_STRIDES:
        try:
            cfg = BuildConfig(linkage=key, module=module)
            payload = walk_model.api_payload(cfg, feet_z=walk_model.foot_z_guess(cfg))
            m = payload["metrics"] if payload["valid"] else None
            _MODULE_STRIDES[k] = None if m is None else round(float(m["stride_mm"]), 2)
        except (ValueError, KeyError, ParamError):
            _MODULE_STRIDES[k] = None
    return _MODULE_STRIDES[k]


# ---------------------------------------------------------------------------
# build, recheck
# ---------------------------------------------------------------------------


def build(design: Design, t: float = 1.0, force: bool = False) -> BuildReport:
    """Fabricate every part at crank angle ``t`` (:func:`fabricate.fabricate`: the robot,
    or one side): the parts land in ``design.parts`` with their live solids, the report
    is the manifest (masses, envelope, counts). A build the store holds at this ``t``
    (from the running engine) comes back from its STEP files instead."""
    if not force and "build" in design.reports and design.build_t == t and (
            design.mech is not None or not design.reports["build"].ok):
        return design.reports["build"]
    t0 = time.time()
    if not force:
        rep = _reload_build(design, t, t0)
        if rep is not None:
            return rep
    pr = plan(design, force)
    if not pr.ok:
        rep = BuildReport(failures=list(pr.failures), t=t)
        return _finish(design, "build", rep, t0)
    try:
        with capture_warnings() as warned:
            mech = fabricate_at(design, t)
    except ConstructionError as e:
        rep = BuildReport(failures=[Failure.from_exception(e, stage="fabricate")], t=t)
        return _finish(design, "build", rep, t0)
    return attach_build(design, mech, t, t0, warnings=warned)


def fabricate_at(design: Design, t: float):
    """The design fabricated at crank angle ``t`` from its own side (what
    :func:`fabricate.fabricate` does, without planning again): one side, or the robot
    with its frame ties and chassis."""
    tmpl, cfg, d = _template(design), design.config, design.side
    if d is None:
        raise ValueError("plan(design) first")
    ties = [FrameTies(d.drive)] if cfg.robot else []
    side = fabricate_side(d, tmpl.freeze_at(t), ties)
    return assemble_robot(side, d) if cfg.robot else side


def _reload_build(design: Design, t: float, t0: float) -> BuildReport | None:
    """The build the store holds at ``t`` from the running engine, its parts back from
    their STEP files (a failed build's report as is); ``None`` when there is none or a
    file is missing."""
    doc = _stored(design, "build")
    if doc is None or doc.get("t") != t:
        return None
    if not doc.get("ok"):
        return _commit(design, "build", BuildReport.from_dict(doc), cached=True)
    if not plan(design).ok:
        return None
    try:
        mech = design.store.load_mechanism(design.id, doc)
    except (OSError, KeyError, ValueError) as e:
        log.warning("%s: stored build unreadable (%s): rebuilding", design.id, e)
        return None
    # the files hold solids and poses; the joints and outlines (what an edit is placed by)
    # come from the side's template at the build's crank angle, as a fresh build's do
    frozen = {b.name: b for b in _template(design).freeze_at(t).bodies}
    for b in mech.bodies:
        kin = frozen.get(split_side(b.name)[1])
        if kin is not None:
            b.joints, b.outline = list(kin.joints), kin.outline
    return attach_build(design, mech, t, t0, cached=True)


def attach_build(design: Design, mech, t: float, t0: float | None = None, *,
                 cached: bool = False, warnings: list[str] | None = None) -> BuildReport:
    """Adopt a fabricated mechanism as the design's build (what :func:`build` does after
    fabricating; a store loading part files, or a test holding a fabricated robot, uses it
    directly). Needs the plan (runs it if it hasn't). ``cached``: the parts came from the
    store, so they are logged as such and not written again. ``warnings``: what the
    constructions warned about while fabricating (:func:`capture_warnings`)."""
    t0 = time.time() if t0 is None else t0
    pr = plan(design)
    if not pr.ok:
        return _finish(design, "build", BuildReport(failures=list(pr.failures), t=t), t0)
    cfg, side = design.config, design.side
    design.mech, design.build_t = mech, t
    z_mid = mech.meta.get("mid_plane")
    servo = servos.get(cfg.servo)
    filament = mech.meta.get("filament")
    parts: dict[str, Part] = {}
    lo = np.full(3, math.inf)
    hi = np.full(3, -math.inf)
    for b in mech.bodies:
        if b.part is None:
            continue
        tag, base = split_side(b.name)
        material, density, fixed = material_of(b, cfg.sheet, filament, servo)
        props = part_props(b.part)
        mass = fixed if fixed is not None else props.volume / 1000.0 * density
        bb = b.placed_part().bounding_box()
        lo, hi = np.minimum(lo, [bb.min.X, bb.min.Y, bb.min.Z]), np.maximum(hi, [bb.max.X,
                                                                                  bb.max.Y,
                                                                                  bb.max.Z])
        parts[b.name] = Part(
            name=b.name, solid=b.part, group=group_of(base, side.plan), side=tag, fab=b.fab,
            material=material, dims_mm=(bb.size.X, bb.size.Y, bb.size.Z),
            layers=side_layers(side.plan, side_z(tag, z_mid, (bb.min.Z, bb.max.Z))),
            density=density, fixed_mass_g=fixed,
            bom_key=b.bom_key, rigid_with=b.rigid_with, pose=b.pose.matrix.tolist(),
            built=b.part, _measured=(b.part, float(props.volume)),
        )
        assert abs(parts[b.name].mass_g - mass) < 1e-9
    design.parts = parts
    counts = {}
    for p in parts.values():
        counts[p.fab] = counts.get(p.fab, 0) + 1
    rep = BuildReport(
        t=t, n_parts=len(parts), counts=counts,
        mass_g=round(sum(p.mass_g for p in parts.values()), 2),
        envelope_mm=tuple(float(v) for v in (hi - lo)) if parts else None,
        meta=jsonable({k: v for k, v in mech.meta.items() if k != "fastened"}),
        parts=[p.to_dict() for p in parts.values()],
        warnings=list(warnings or []),
    )
    return _finish(design, "build", rep, t0, cached=cached)


def split_side(name: str) -> tuple[str | None, str]:
    """``"L.b1_leg0"`` -> ``("L", "b1_leg0")``; a chassis part has no side."""
    if len(name) > 2 and name[1] == "." and name[0] in "LR":
        return name[0], name[2:]
    return None, name


def side_z(tag: str | None, z_mid: float | None, z: tuple[float, float]) -> tuple[float, float]:
    """A robot part's z range back in its side's coordinates (the left side is the side
    moved down by the mid-plane, the right side its mirror image moved up)."""
    if z_mid is None or tag is None:
        return z
    if tag == "L":
        return z[0] + z_mid, z[1] + z_mid
    return z_mid - z[1], z_mid - z[0]


def to_side(solid, tag: str | None, z_mid: float | None):
    """A robot part's placed solid back in its side's coordinates (see :func:`side_z`)."""
    from build123d import Location, Plane

    if z_mid is None or tag is None:
        return solid
    if tag == "L":
        return solid.moved(Location((0.0, 0.0, z_mid)))
    return solid.moved(Location((0.0, 0.0, -z_mid))).mirror(Plane.XY)


def side_layers(plan, z: tuple[float, float], eps: float = 1e-6) -> tuple[int, ...]:
    """The layers a z range (side coordinates) reaches into (outside the plates too)."""
    pitch = plan.spec.pitch
    lo, hi = math.floor(z[0] / pitch + eps) - 1, math.ceil(z[1] / pitch - eps) + 1
    out = []
    for k in range(lo, hi + 1):
        z0, z1 = plan.z(k)
        if z0 < z[1] - eps and z1 > z[0] + eps:
            out.append(k)
    return tuple(out)


def group_of(name: str, plan) -> str:
    """The construction group that built a side body, from its name: a link (``links``),
    the plates (``frame``), the servo and its screws (``drive``), the crank's segments and
    screws (``crank``), an axle's segments and caps (``pillar:<axis>`` / ``pin:<axis>``),
    the robot's chassis (``chassis``)."""
    if name in plan.layers:
        return "links"
    if name in plan.topo.frame_bodies or name.startswith("frame"):
        return "frame"
    if name.startswith("servo"):
        return "drive"
    groups = sorted({p.group for p in plan.placed if p.group not in plan.layers},
                    key=len, reverse=True)
    for g in groups:
        stem = g.replace(":", "_")
        if name == stem or name.startswith(stem + "_"):
            return g
    if name.startswith("crank"):
        return "crank"
    if name.startswith(("centre_plate", "tie_", "rear_screw")):
        return "chassis"
    return "other"


CLAIMED_GROUPS = ("links", "crank", "pillar", "pin")


def recheck(design: Design, all_parts: bool = False) -> RecheckReport:
    """Re-run the checks over the parts as they now are (an agent may have replaced a
    :class:`Part`'s ``solid``): every part one valid solid, no two parts intersecting
    (:func:`construction.contract.clashes`), and the edited parts (``all_parts``: every
    part) of a claim-bound group (links, crank, pillars, pins) inside their group's claims
    at the build's crank angle. A passing recheck accepts the edited solids as the
    design's; until then an edited solid is outside the correct-by-construction
    guarantee (the plates, the drive and the chassis are covered by the clash check
    alone)."""
    t0 = time.time()
    if design.mech is None:
        raise ValueError("nothing built yet: build(design) first")
    from build123d import Shape

    rep = RecheckReport(edited=[n for n, p in design.parts.items() if p.edited])
    mech, side = design.mech, design.side
    for n, part in design.parts.items():       # the mechanism mirrors the parts as they are
        if not isinstance(part.solid, Shape):
            raise TypeError(f"parts[{n!r}].solid must be a build123d Shape (a Part, Solid or "
                            f"Compound), got {type(part.solid).__name__}")
        mech.body(n).part = part.solid
    rep.bad_solids = bad_solids(mech)
    rep.clashes = clashes(mech)
    z_mid = mech.meta.get("mid_plane")
    build_ = Build(side.ctx, side.plan, _template(design).freeze_at(design.build_t))
    for name in (list(design.parts) if all_parts else rep.edited):
        part = design.parts[name]
        group = part.group
        if group.split(":")[0] not in CLAIMED_GROUPS:
            continue
        _, base = split_side(name)
        shapes = build_.shapes(base if group == "links" else group)
        if not shapes:
            continue
        rep.checked.append(name)
        env = claimed_solid(build_, shapes, TOL)
        vol = _outside(to_side(part.placed(), part.side, z_mid), env)
        if vol > MAX_OUTSIDE:
            rep.contract.append({"part": name, "group": group, "mm3_outside": round(vol, 3)})
    if rep.bad_solids:
        rep.failures.append(Failure(
            "clash", "bad_solid", "; ".join(f"{s['part']}: {s['solids']} solids, valid="
                                            f"{s['valid']}" for s in rep.bad_solids),
            culprits=[{"body": s["part"]} for s in rep.bad_solids]))
    if rep.clashes:
        rep.failures.append(Failure(
            "clash", "parts_clash", "; ".join(f"{c['a']} x {c['b']}: {c['mm3']} mm^3"
                                              for c in rep.clashes),
            culprits=[{"body": c["a"], "other": c["b"]} for c in rep.clashes],
            numbers={"mm3": max(c["mm3"] for c in rep.clashes)}))
    if rep.contract:
        rep.failures.append(Failure(
            "contract", "part_outside_claim",
            "; ".join(f"{c['part']}: {c['mm3_outside']} mm^3 outside its claims"
                      for c in rep.contract),
            culprits=[{"body": c["part"], "group": c["group"]} for c in rep.contract],
            numbers={"mm3": max(c["mm3_outside"] for c in rep.contract)}))
    if not rep.failures and rep.edited:
        for n in rep.edited:
            design.parts[n].built = design.parts[n].solid
        br = design.reports.get("build")
        if br is not None:      # the handle's build now describes the edited parts
            br.mass_g = round(sum(p.mass_g for p in design.parts.values()), 2)
            br.parts = [p.to_dict() for p in design.parts.values()]
    return _finish(design, "recheck", rep, t0)


# ---------------------------------------------------------------------------
# verify, export
# ---------------------------------------------------------------------------


def verify(design: Design, level: str = "quick"):
    """:func:`spiderpig.verify.verify`: pass/fail per requirement with an evidence tier."""
    from spiderpig import verify as _verify

    return _verify.verify(design, level)


def export(design: Design, formats=None, out_dir: str | Path | None = None,
           force: bool = False) -> ExportReport:
    """Write the chosen ``formats`` (default: the spec's ``outputs``) into ``out_dir``
    (default: the design's ``exports/`` in its store, else ``build/``), as ``spiderpig build``
    does: ``step`` / ``stl`` (the whole machine), ``print`` (one STL per different printed
    part + ``parts.csv``), ``dxf`` (kerf-compensated sheets + ``parts.csv``), ``bom``
    (csv, md, json), ``glb`` (the viewer's animated bake), ``mjcf`` (the MuJoCo model +
    its metadata); and always ``manifest.json``. Builds first if nothing is built; a
    sheet-packing failure is ``layout``, a catalog miss ``bom``. What the bake and the
    constructions warned about while writing (a purchased model's faces the mesher
    skipped) is on the report (``warnings``) and in the manifest. An export the store
    records for the same formats and folder, whose files are all still there, is
    returned as is (``force`` rewrites)."""
    from spiderpig.spec import OUTPUTS

    t0 = time.time()
    formats = list(formats or design.spec.outputs)
    bad = [f for f in formats if f not in OUTPUTS]
    if bad:
        raise ValueError(f"unknown formats {bad}; have {list(OUTPUTS)}")
    if out_dir is None:
        out_dir = design.store.exports_dir(design.id) if design.store else Path("build")
    out = Path(out_dir)
    if not force:      # a prior export of these formats (or more) into this folder
        prior = _cached(design, "export", ExportReport, out_dir=str(out.resolve()))
        if (prior is not None and prior.ok and set(formats) <= set(prior.formats)
                and all(Path(f).is_file() for f in prior.files)):
            return prior
    rep = ExportReport(out_dir=str(out.resolve()), formats=formats)
    if design.mech is None:
        br = build(design)
        if not br.ok:
            rep.failures = list(br.failures)
            return _finish(design, "export", rep, t0)
    out.mkdir(parents=True, exist_ok=True)
    with contextlib.ExitStack() as stack:
        warned = stack.enter_context(capture_warnings(EXPORT_LOGGERS))
        stack.enter_context(pywarnings.catch_warnings())
        pywarnings.filterwarnings("ignore", message="Unknown Compound type")
        files, bom_summary = _export_files(design, formats, out, rep)
    rep.warnings = warned
    cfg = design.config
    pr, br = design.reports["plan"], design.reports["build"]
    vr = design.reports.get("verify")
    rep.manifest = jsonable({
        "design": design.id, "engine_version": design.engine_version, "t_ref": design.build_t,
        "formats": formats, "files": [str(f.relative_to(out)) for f in files],
        "parts": br.parts, "counts": br.counts, "mass_g": br.mass_g,
        "envelope_mm": br.envelope_mm, "sheet": sheet_name(cfg.sheet),
        "plan": {"layers": pr.n_layers, "height_mm": pr.height_mm, "route": pr.route,
                 "optimal": pr.optimal},
        "bom": bom_summary,
        "warnings": rep.warnings,
        "verify": None if vr is None else {"level": vr.level, "ok": vr.ok, "score": vr.score,
                                           "failed": [r.requirement for r in vr.rows
                                                      if not r.passed]},
    })
    import json

    (out / "manifest.json").write_text(json.dumps(rep.manifest, indent=1))
    files.append(out / "manifest.json")
    rep.files = [str(f) for f in files]
    return _finish(design, "export", rep, t0)


EXPORT_LOGGERS = (*WARNING_LOGGERS, "bake_gltf", "spiderpig.bake", "spiderpig.layout")


def _export_files(design: Design, formats: list[str], out: Path, rep: ExportReport
                  ) -> tuple[list[Path], dict | None]:
    """Write the formats into ``out`` (see :func:`export`): the files written, and the
    BOM's summary."""
    cfg, mech, spec = design.config, design.mech, design.spec
    name = cfg.linkage
    filament = mech.meta.get("filament", "pla_filament")
    files: list[Path] = []
    if "step" in formats:
        mech.export_step(out / f"{name}.step")
        files.append(out / f"{name}.step")
    if "stl" in formats:
        mech.export_stl(out / f"{name}.stl")
        files.append(out / f"{name}.stl")
    groups = None
    if "print" in formats or "bom" in formats:
        groups = {m: group_made(mech.bodies, m) for m in ("laser", "printed")}
    if "print" in formats:
        from spiderpig import build as build_cli

        build_cli.export_prints(groups["printed"], out / "print",
                                density=filament_density(filament))
        files += sorted((out / "print").glob("*"))
    extras = list(mech.bom_extras)
    bom_summary = None
    if "dxf" in formats:
        size = tuple(spec.fit.sheet_size_mm or sheet_size(cfg.sheet))
        kerf = spec.fit.kerf_mm if spec.fit.kerf_mm is not None else DEFAULT_KERF
        try:
            sheets = save_sheets(mech, out / "laser" / f"{name}_sheet", sheet_size=size,
                                 kerf=kerf)
            files += sheets + [out / "laser" / f"{name}_sheet_parts.csv"]
            extras.append(BomLine(cfg.sheet, len(sheets), "laser-cut parts"))
        except ValueError as e:
            rep.failures.append(Failure.from_exception(e, stage="layout"))
    if "bom" in formats:
        title = (f"{cfg.module} {'robot' if cfg.robot else 'side'}, {cfg.servo}, {cfg.pillar} "
                 f"pillars, {cfg.pin} pins, {cfg.crank} crank, {cfg.sheet}")
        try:
            bom = bom_from_mechanism(replace(mech, bom_extras=extras), title=title,
                                     filament=filament, groups=groups)
            files += bom.write(out)
            bom_summary = {"items": len(bom.purchased), "cost_usd": round(bom.cost_usd, 2),
                           "printed_g": bom.printed_g,
                           "unpriced": [r.key for r in bom.unpriced]}
        except KeyError as e:
            rep.failures.append(Failure.from_exception(e, stage="bom"))
    if "glb" in formats:
        from spiderpig.bake import bake_gltf

        bake_gltf(out / f"{name}.glb", cfg, profile=False)
        files.append(out / f"{name}.glb")
    if "mjcf" in formats:
        import json

        from spiderpig.sim.mjcf import build_mjcf

        xml, meta = build_mjcf(cfg)
        (out / f"{name}.xml").write_text(xml)
        (out / f"{name}.json").write_text(json.dumps(meta, indent=1))
        files += [out / f"{name}.xml", out / f"{name}.json"]
    return files, bom_summary


__all__ = [
    "BuildReport", "CheckReport", "ExportReport", "PlanReport", "RecheckReport", "Report",
    "WalkReport", "attach_build", "build", "check", "compare", "derive", "describe", "explain",
    "export", "fabricate_at", "foot_path", "gc", "list_designs", "list_linkages", "load",
    "plan", "recheck", "recommend", "resolve", "verify", "walk",
]
