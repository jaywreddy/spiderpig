"""A spec resolved into a design: :func:`resolve` (validated, its omissions inferred, its
:class:`config.BuildConfig` built, recorded in a store), :func:`spec_of` (a CLI's options
as a spec), :func:`config_warnings` (what resolve warns about, for the CLIs) and
:func:`load` (a recorded design's handle, its config rebuilt from its resolved spec; the
CLIs' ``DESIGN`` argument through :func:`load_stored`)."""


from __future__ import annotations

import math
from pathlib import Path

from spiderpig import linkage, servos
from spiderpig import roles as rolemod
from spiderpig import walk as walk_model
from spiderpig.config import (
    ROLE_FIELDS,
    BuildConfig,
    ParamError,
    default_params,
    default_robot,
    removed_param,
    robot_layout,
)
from spiderpig.construction.base import Params
from spiderpig.design import Design, design_id, engine_version
from spiderpig.hardware.catalog import sheet_size, sheet_thickness
from spiderpig.materials import link_sheets
from spiderpig.spec import (
    ALLOWANCE,
    FIT_FIELDS,
    OUTPUTS,
    PAYLOAD,
    SECTIONS,
    Spec,
    SpecError,
    SpecErrors,
    default_module,
    effective_hard,
    legs_robot,
    urdf_unsupported,
    validate,
)
from spiderpig.store import PROJECT, Store


def resolve(spec: Spec | dict, store: Store | str | Path | None = PROJECT, *,
            derived_from: str | None = None, patch: dict | None = None) -> Design:
    """Validate ``spec`` (a :class:`Spec` or its document), infer what it leaves out, build
    its :class:`config.BuildConfig` and return the :class:`Design` handle. The inferred
    values are in ``design.resolved`` (the complete spec the id is computed from);
    :class:`SpecErrors` lists everything wrong with an invalid spec.

    A spec whose ``outputs`` name a format its linkage can't have is refused with the
    reason (``urdf``: :func:`spiderpig.spec.urdf_unsupported`); a spec's ``task`` is
    written into ``design.resolved`` as given (it enters the id; a spec without one keeps
    the id it had).

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
    if "urdf" in spec.outputs and (why := urdf_unsupported(lk)):
        raise SpecErrors([SpecError(f"outputs[{spec.outputs.index('urdf')}]", why,
                                    tuple(o for o in OUTPUTS if o != "urdf"))])
    module = spec.legs.module or default_module(lk)
    robot = legs_robot(lk, {"robot": spec.legs.robot, "sides": spec.legs.sides})
    robot = default_robot(lk.key) if robot is None else robot
    sides = 2 if lk.walks and robot else 1     # (the resolved spec's word, as stored: one id)
    phases = (None if spec.legs.phases_deg is None
              else tuple(math.radians(p) for p in spec.legs.phases_deg))
    d = BuildConfig()
    sheet = spec.materials.sheet or d.sheet
    thickness = spec.materials.thickness_mm          # the nominal is the default: one id
    if thickness is not None:
        thickness = None if float(thickness) == sheet_thickness(sheet) else float(thickness)
    try:
        config = BuildConfig(
            linkage=lk.key, module=module, phases=phases,
            robot=robot,
            proportions=tuple(sorted(spec.linkage.params.items())),
            sheet=sheet, thickness=thickness,
            servo=spec.materials.servo or d.servo,
            frame_sheet=spec.materials.frame_sheet or d.frame_sheet,
            crank_sheet=spec.materials.crank_sheet or "",     # the linkage's (config)
            link_sheets=(None if spec.materials.link_sheets is None
                         else tuple(sorted(spec.materials.link_sheets.items()))),
            pillar=spec.constructions.pillar or "", pin=spec.constructions.pin or "",
            crank=spec.constructions.crank or "", heads=spec.constructions.heads or d.heads,
            roles=tuple(sorted((spec.constructions.roles or {}).items())),
            params=default_params(lk, {k: getattr(spec.fit, k) for k in FIT_FIELDS
                                       if getattr(spec.fit, k) is not None}),
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
                     config.sides if sides is None else sides)


def _warnings(lk, config: BuildConfig, sides: int) -> list[str]:
    warnings: list[str] = []
    if lk.walks and sides == 1:
        warnings.append("legs.robot false builds one side (no chassis); the walk metrics "
                        "still model the two-sided robot")
    if undriven_inputs(lk):         # (an arm's inputs are its drives'; a declared one's too)
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


def undriven_inputs(lk) -> tuple[str, ...]:
    """A cranked linkage's inputs past its crank that declare no drive
    (:attr:`linkage.Linkage.drives`): the crank's servo turns the first, a declared one gets
    a stepper's ``JointDrive`` beside it (``construction._joint_drives``), and the drive
    stage stops at any other (``second_input_no_drive``). An arm's inputs are all its
    drives': none."""
    if lk.ranged:
        return ()
    return tuple(k for k in lk.inputs[1:] if k not in lk.drives)


def second_input_note(lk) -> str:
    """What a cranked linkage with an input that declares no drive (:func:`undriven_inputs`:
    the five-bar's ``t2``) gets told, at ``resolve`` and at the ``drive`` stage: it can be
    resolved, checked for its program and its output, and drawn, but not planned or built
    until the linkage declares that input's drive; which one-input mechanisms can."""
    others = [k for k in linkage.available("mechanism")
              if len(linkage.get(k).inputs) == 1]
    free = undriven_inputs(lk)
    return (f"{lk.key} has {len(lk.inputs)} inputs ({', '.join(lk.inputs)}) and "
            f"{', '.join(free)} declare{'s' if len(free) == 1 else ''} no drive (the crank's "
            f"servo turns {lk.inputs[0]}; an input past it is driven where the linkage "
            f"declares its link and frame joint, Linkage.drives, a stepper's JointDrive): "
            f"check reads its program and its output, but plan, build, verify and export "
            f"stop at the drive stage (second_input_no_drive), a limit of the linkage's "
            f"declaration, not of the spec; the one-input mechanisms are {', '.join(others)}")


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
    kind from the linkage, the proportions it overrides, module, phases and robot
    (``sides``: a walker's 1 is its side alone, whatever ``config.robot`` says), the
    materials and constructions, and the fit fields that differ from the defaults; no
    targets. ``resolve(spec_of(config))`` is a design with that config, so ``spiderpig
    view --linkage ... --pin bolt`` can show what ``spiderpig build`` built."""
    lk = config.lk
    doc: dict = {"kind": lk.kind, "linkage": {"key": config.linkage}}
    if config.proportions:
        doc["linkage"]["params"] = dict(config.proportions)
    robot = bool(config.robot) if sides is None or not lk.walks else int(sides) == 2
    legs: dict = {"module": config.module, "robot": robot}
    if config.phases is not None:
        legs["phases_deg"] = [round(math.degrees(p), 6) for p in config.phases]
    doc["legs"] = legs
    mats: dict = {"sheet": config.sheet, "servo": config.servo,
                  "frame_sheet": config.frame_sheet, "crank_sheet": config.crank_sheet}
    for what, role in ROLE_FIELDS.items():      # (an arm's: no drive servo, no crank)
        if not rolemod.applies(role, lk.kind):
            del mats[what]
    if config.thickness is not None:
        mats["thickness_mm"] = config.thickness
    if config.link_sheets is not None:
        mats["link_sheets"] = dict(config.link_sheets)
    doc["materials"] = mats
    doc["constructions"] = {"pillar": config.pillar, "pin": config.pin, "crank": config.crank,
                            "heads": config.heads, **dict(config.roles)}
    if not config.crank:        # an arm has none ("" in a spec would be a wildcard)
        del doc["constructions"]["crank"]
    default = default_params(config.lk)        # the kind's (config.KIND_FIT)
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
    if spec.payload_g is not None:          # the precision section's
        targets["precision"][PAYLOAD] = spec.payload_g
    return {
        "version": spec.version, "kind": spec.kind,
        "linkage": {"key": config.linkage, "params": design["proportions"]},
        "legs": {"module": module, "phases_deg": design["phases_deg"], "sides": sides,
                 **({"robot": config.robot} if robot_layout(config.lk) == "turret" else {})},
        "motion": targets["motion"], "size": targets["size"], "budget": targets["budget"],
        "materials": {"sheet": config.sheet, "thickness_mm": config.thickness,
                      "pitch_mm": config.pitch, "servo": config.servo,
                      "frame_sheet": config.frame_sheet, "crank_sheet": config.crank_sheet,
                      "link_sheets": link_sheets(config)},
        "constructions": {"pillar": config.pillar, "pin": config.pin, "crank": config.crank,
                          "heads": config.heads, **config.applied_roles()},
        "fit": fit,
        "outputs": list(spec.outputs),
        # (only when the spec asks: a design without precision targets keeps its id)
        **({"precision": targets["precision"]} if targets["precision"] else {}),
        # (likewise: the task it was written for, what verify's task.* rows re-check)
        **({"task": dict(spec.task)} if spec.task is not None else {}),
    }


def _stored_roles(cons: dict) -> tuple[tuple[str, str], ...]:
    """The roles a resolved spec's ``constructions`` names beside its own keys; a key that is
    neither its own nor a known role (or a known per-slot role's ``role@slot``) fails: a
    stored design is rebuilt as recorded or not at all."""
    from spiderpig import roles as rolemod

    out = []
    for q, key in cons.items():
        if q in ("pillar", "pin", "crank", "heads"):
            continue
        name, slot = rolemod.split(q)
        try:
            role = rolemod.get(name)
        except KeyError:
            raise ParamError(f"this stored design names a construction role {q!r} the engine "
                             f"no longer has (roles: {sorted(rolemod.ROLES)})") from None
        if not role.stored_in_roles or (slot and role.per == "design"):
            raise ParamError(f"this stored design's constructions.{q} isn't a role's key")
        out.append((q, key))
    return tuple(sorted(out))


def _config_from_resolved(resolved: dict) -> BuildConfig:
    """The config of a recorded design, from its resolved spec alone (every value written
    in, so a default that moved since never changes a stored design)."""
    lk = linkage.get(resolved["linkage"]["key"])
    legs, mat, cons, fit = (resolved[k] for k in ("legs", "materials", "constructions", "fit"))
    for name, value in fit.items():     # a removed Params field: only at its last default
        if (gone := removed_param(name, value)) is not None:
            raise ParamError(gone)
    return BuildConfig(
        # an arm's robot is its own key (legs.robot); a design stored before it was a side
        linkage=lk.key, module=legs["module"], robot=bool(legs.get("robot", legs["sides"] == 2)),
        phases=tuple(math.radians(p) for p in legs["phases_deg"]),
        proportions=tuple(sorted(resolved["linkage"]["params"].items())),
        sheet=mat["sheet"], thickness=mat.get("thickness_mm"), servo=mat["servo"],
        frame_sheet=mat.get("frame_sheet") or mat["sheet"],
        crank_sheet=mat.get("crank_sheet") or mat["sheet"],
        link_sheets=tuple(sorted((mat.get("link_sheets") or {}).items())),
        pillar=cons["pillar"], pin=cons["pin"], crank=cons["crank"],
        heads=cons.get("heads") or "sink",
        roles=_stored_roles(cons),
        params=Params(**{k: fit[k] for k in FIT_FIELDS if k in fit}),
    )


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


def load_stored(design_id: str, store: Store | str | Path | None = None) -> Design:
    """:func:`load` for a command line's ``DESIGN`` argument (``store``: its ``--store``,
    ``None`` the project store): ``KeyError`` names the store and the last designs it holds
    (``spiderpig build|guide|control|precision|export|view|sim DESIGN``: the design as
    recorded, its spec's roles and every resolved value, not the options' defaults)."""
    st = Store.default() if store is None else Store.of(store)
    try:
        return load(design_id, st)
    except KeyError:
        ids = [c["id"] for c in st.list_designs()]
        have = ", ".join(ids[-8:]) if ids else "nothing"
        raise KeyError(f"no design {design_id!r} in {st.root.resolve()} (has: {have})"
                       ) from None


DESIGN_HELP = ("a stored design's id (16 hex digits, from resolve or candidates; "
               "spiderpig mcp's list_designs lists the store's): its config as recorded, "
               "its spec's roles and every resolved value, in place of the design options")
