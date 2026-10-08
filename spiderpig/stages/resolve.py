"""A spec resolved into a design: :func:`resolve` (validated, its omissions inferred, its
:class:`config.BuildConfig` built, recorded in a store), :func:`spec_of` (a CLI's options
as a spec) and :func:`config_warnings` (what resolve warns about, for the CLIs)."""


from __future__ import annotations

import math
from pathlib import Path

from spiderpig import linkage, servos
from spiderpig import walk as walk_model
from spiderpig.config import BuildConfig, ParamError
from spiderpig.construction.base import Params
from spiderpig.design import Design, design_id, engine_version
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
from spiderpig.store import PROJECT, Store


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
