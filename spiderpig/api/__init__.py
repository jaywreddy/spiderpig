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

The package (a pure move of the former ``api.py``, W5): :mod:`.reports`, :mod:`.store_ops`
(resolve, load, the store's stage records), :mod:`.cards` (linkage cards), :mod:`.planning`
(check, plan, explain, recommend, advise), :mod:`.walking`, :mod:`.building` (build,
recheck) and :mod:`.exports` (verify, export); not ``plan.py`` / ``build.py`` /
``export.py``, which the functions' re-exports would shadow. Every name keeps its old
import path here, and a write to one (a test's ``monkeypatch.setattr(api, "fabricate_at",
...)``) reaches the submodules that read it (:mod:`spiderpig.reexport`); every engine name the
module imported is imported here too (its users reach and patch them as ``api.X``:
``api.design_side``, ``api.engine_version``, ``api.PROJECT``).
"""

from spiderpig import linkage, servos
from spiderpig import walk as walk_model
from spiderpig.api.building import (
    CLAIMED_GROUPS,
    _build,
    _envelope,
    _recheck_parts,
    _reload_build,
    attach_build,
    build,
    cut_rules,
    cut_rules_of,
    fabricate_at,
    group_of,
    recheck,
    side_layers,
    side_z,
    split_side,
    to_side,
)
from spiderpig.api.cards import (
    _CACHE_NAME,
    OUTPUT_SENSITIVITY_KEYS,
    SENSITIVITY_DEG,
    SENSITIVITY_STEP,
    _cache_path,
    _card,
    _linkage,
    _output_dict,
    _output_numbers,
    _read_cache,
    _step_dict,
    _write_cache,
    describe,
    foot_path,
    list_linkages,
    output_sensitivity,
    scale_params_table,
    sensitivity,
)
from spiderpig.api.exports import (
    EXPORT_LOGGERS,
    GROUPED,
    _drop_robot_job,
    _export,
    _export_files,
    _group_job,
    _groups,
    _robot_files,
    _robot_files_job,
    _start_group_job,
    _start_robot_job,
    _timed,
    _walker_formats,
    export,
    verify,
)
from spiderpig.api.planning import (
    CHEAP_OUTPUT,
    SCALED_METRICS,
    PlanTimeout,
    _plan_report,
    _remake_plan,
    _reuse_plan,
    advise,
    cheap_measures,
    check,
    explain,
    measure_config,
    missed_targets,
    plan,
    plan_config,
    recommend,
    timed_out,
)
from spiderpig.api.reports import (
    AdviceReport,
    BuildReport,
    CheckReport,
    ExportReport,
    PlanReport,
    RecheckReport,
    Report,
    WalkReport,
)
from spiderpig.api.store_ops import (
    EDITED_STAGES,
    THICKNESS_TOLERANCE,
    WARNING_LOGGERS,
    _attach_store,
    _cached,
    _commit,
    _config_from_resolved,
    _derived_from,
    _docs_of,
    _drop_stored,
    _engine_of,
    _finish,
    _forget,
    _manifest,
    _manifest_design,
    _record,
    _resolved,
    _store_docs,
    _stored,
    _template,
    _warnings,
    capture_warnings,
    compare,
    config_warnings,
    derive,
    design_lock,
    gc,
    list_designs,
    load,
    log,
    ran_out,
    resolve,
    second_input_note,
    spec_of,
)
from spiderpig.api.walking import (
    _MODULE_STRIDES,
    NO_TRAVEL_MM,
    WALKS_MM,
    _module_stride,
    module_stride,
    no_travel_note,
    walk,
)
from spiderpig.config import (
    BuildConfig,
    ParamError,
    default_robot,
    removed_param,
    torque_limit_note,
)
from spiderpig.construction.base import Build, ConstructionError, Params
from spiderpig.construction.contract import MAX_OUTSIDE, TOL, _outside, bad_solids, clashes
from spiderpig.construction.crank import CrankRoute, Run
from spiderpig.construction.envelope import claimed_solid
from spiderpig.construction.robot import FrameTies, assemble_robot
from spiderpig.design import (
    Design,
    Part,
    design_id,
    engine_version,
    jsonable,
    source_version,
    spec_hash,
)
from spiderpig.fabricate import (
    SideDesign,
    design_side,
    fabricate_side,
    ground_clearance,
    remember,
    side_problem,
    static_stage,
)
from spiderpig.fabricate import template_for as _template_for
from spiderpig.failure import Failure, Recommendation, apply_patch, merge_patch
from spiderpig.hardware.bom import bom_from_mechanism, group_made
from spiderpig.hardware.catalog import sheet_name, sheet_size, sheet_thickness
from spiderpig.hardware.mass import filament_density, material_of, part_props
from spiderpig.layout import save_sheets, sheet_lines
from spiderpig.materials import link_sheets
from spiderpig.reexport import forward_writes
from spiderpig.spec import (
    ALLOWANCE,
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
from spiderpig.store import PROJECT, Store, _read_json, _write_json, diff_json, report_doc

__all__ = [
    "BuildReport", "CheckReport", "ExportReport", "PlanReport", "RecheckReport", "Report",
    "WalkReport", "attach_build", "build", "check", "compare", "derive", "describe", "explain",
    "export", "fabricate_at", "foot_path", "gc", "list_designs", "list_linkages", "load",
    "plan", "recheck", "recommend", "resolve", "verify", "walk",
]

forward_writes(__name__)
