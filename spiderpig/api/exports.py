"""Verification (:func:`verify`) and the exports (:func:`export`)."""


from __future__ import annotations

import contextlib
import logging
import time
import warnings as pywarnings
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path

from spiderpig.api import building  # (build, fabricate_at: through the module, where a patch goes)
from spiderpig.api.reports import BuildReport, ExportReport, PlanReport
from spiderpig.api.store_ops import _manifest, design_lock, load
from spiderpig.config import (
    torque_limit_note,
)
from spiderpig.design import (
    Design,
    jsonable,
)
from spiderpig.failure import Failure
from spiderpig.hardware.bom import bom_from_mechanism, group_made
from spiderpig.hardware.catalog import sheet_name
from spiderpig.hardware.mass import filament_density
from spiderpig.layout import save_sheets, sheet_lines
from spiderpig.stages.planning import plan
from spiderpig.stages.records import (
    WARNING_LOGGERS,
    _cached,
    _finish,
    _report,
    capture_warnings,
    log,
)

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
    with design_lock(design):
        return _export(design, formats, out_dir, force)


def _export(design: Design, formats, out_dir, force: bool) -> ExportReport:
    from spiderpig.spec import OUTPUTS

    t0 = time.time()
    formats = list(formats or design.spec.outputs)
    bad = [f for f in formats if f not in OUTPUTS]
    if bad:
        raise ValueError(f"unknown formats {bad}; have {list(OUTPUTS)}")
    if out_dir is None:
        out_dir = design.store.exports_dir(design.id) if design.store else Path("build")
        if design.edited:       # not over the store's own exports (the unedited design's)
            out_dir = Path(out_dir) / "edited"
    out = Path(out_dir)
    if not force:      # a prior export of these formats (or more) into this folder
        prior = _cached(design, "export", ExportReport, out_dir=str(out.resolve()))
        last = _manifest(out)
        if (prior is not None and prior.ok and set(formats) <= set(prior.formats)
                and all(Path(f).is_file() for f in prior.files)
                # the folder's last writer was this design's export of these formats (not
                # since overwritten: another design's, or any `spiderpig build`'s)
                and last.get("design") == design.id
                and set(formats) <= set(last.get("formats") or ())
                # nor an edited handle's (its parts aren't the design's: never reused)
                and not last.get("edited") and not design.edited):
            return prior
    rep = ExportReport(out_dir=str(out.resolve()), formats=formats)
    # a folder whose manifest (an export's, or a `spiderpig build`'s) names another design,
    # or none: its cut and print files and its shopping list aren't this design's
    last = _manifest(out) if out.is_dir() else {}
    foreign = out.is_dir() and (last.get("design") != design.id
                                or bool(last.get("edited")) != design.edited)
    job = None
    if design.mech is None:
        # the glb and the MJCF need the plan, not the build: their worker starts first
        job = _start_robot_job(design, formats, before_build=True)
        try:
            br = building.build(design)
        except BaseException:
            _drop_robot_job(job)
            raise
        if not br.ok:
            _drop_robot_job(job)
            rep.failures = list(br.failures)
            return _finish(design, "export", rep, t0)
    out.mkdir(parents=True, exist_ok=True)
    if foreign:                     # (only once this design has built: nothing lost before)
        from spiderpig.build import clear_generated

        clear_generated(out / "laser")
        clear_generated(out / "print")
        (out / "ORDER.md").unlink(missing_ok=True)
    with contextlib.ExitStack() as stack:
        warned = stack.enter_context(capture_warnings(EXPORT_LOGGERS))
        stack.enter_context(pywarnings.catch_warnings())
        pywarnings.filterwarnings("ignore", message="Unknown Compound type")
        files, bom_summary = _export_files(design, formats, out, rep, job)
    rep.warnings = warned
    cfg = design.config
    pr, br = _report(design, "plan", PlanReport), _report(design, "build", BuildReport)
    assert pr is not None       # the build above planned first
    assert br is not None       # built above
    from spiderpig.verify import VerifyReport

    vr = _report(design, "verify", VerifyReport)
    rep.manifest = jsonable({
        "design": design.id, "engine_version": design.engine_version, "t_ref": design.build_t,
        "edited": design.edited,         # an edited handle's parts (not the design's own)
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


EXPORT_LOGGERS = (*WARNING_LOGGERS, "bake_gltf", "spiderpig.bake", "spiderpig.layout",
                  "spiderpig.export")


def _export_files(design: Design, formats: list[str], out: Path, rep: ExportReport,
                  job=None) -> tuple[list[Path], dict | None]:
    """Write the formats into ``out`` (see :func:`export`): the files written, and the
    BOM's summary. ``job``: the glb/MJCF worker if it is already running."""
    cfg, mech, spec = design.config, design.mech, design.spec
    assert mech is not None     # _export built the design first
    name = cfg.linkage
    filament = mech.meta.get("filament", "pla_filament")
    files: list[Path] = []
    job = job or _start_robot_job(design, formats)
    grouping = (_start_group_job(mech) if ("print" in formats or "bom" in formats)
                and ("step" in formats or "stl" in formats) else None)
    if "step" in formats:
        with _timed("step"):
            mech.export_step(out / f"{name}.step")
        files.append(out / f"{name}.step")
    if "stl" in formats:
        with _timed("stl"):
            mech.export_stl(out / f"{name}.stl")
        files.append(out / f"{name}.stl")
    groups = None
    if "print" in formats or "bom" in formats:
        with _timed("grouping" if grouping is None else "grouping (waiting for its worker)"):
            groups = _groups(mech, grouping)
    types: list = []

    def labelled() -> list:
        """The part labels (spiderpig.labels): they name the print and cut files, as
        spiderpig build's and the guide's do. Made once."""
        if not types:
            from spiderpig.labels import assembly_order, part_types

            order = assembly_order(mech, design.side) if design.side is not None else None
            types.extend(part_types(mech, order, groups, filament))
        return types

    if "print" in formats:
        from spiderpig import build as build_cli

        assert groups is not None   # grouped just above for "print"
        with _timed("print"):
            build_cli.clear_generated(out / "print")     # no STLs of another design
            from spiderpig.hardware.bom import printed_filaments
            from spiderpig.labels import print_stems

            build_cli.export_prints(groups["printed"], out / "print",
                                    density=filament_density(filament),
                                    filaments=printed_filaments(mech, filament),
                                    stems=print_stems(labelled()))
        files += sorted((out / "print").glob("*"))
    extras = list(mech.bom_extras)
    bom_summary = None
    size: tuple[float, float] | None = None
    if spec.fit.sheet_size_mm:
        w, h = spec.fit.sheet_size_mm
        size = (w, h)
    kerf = spec.fit.kerf_mm               # None: each sheet's service kerf (layout.sheet_kerf)
    if "dxf" in formats:
        try:
            with _timed("dxf"):
                # the packed sheets are written again: none left from before (a build's
                # laser/parts/ of this design, which ORDER.md lists, stay)
                if (out / "laser").is_dir():
                    for f in (out / "laser").glob(f"{name}_sheet*"):
                        if f.suffix in (".dxf", ".csv"):
                            f.unlink()
                from spiderpig.labels import laser_labels

                sheets = save_sheets(mech, out / "laser" / f"{name}_sheet", sheet_size=size,
                                     kerf=kerf, default=cfg.sheet,
                                     labels=laser_labels(labelled()))
            files += [*sheets, out / "laser" / f"{name}_sheet_parts.csv"]
            extras += sheet_lines(mech, cfg.sheet, size)
        except ValueError as e:
            rep.failures.append(Failure.from_exception(e, stage="layout"))
    elif "bom" in formats:      # the BOM buys the sheets whether or not the DXF is written
        try:
            extras += sheet_lines(mech, cfg.sheet, size)
        except ValueError as e:
            rep.failures.append(Failure.from_exception(e, stage="layout"))
    if "bom" in formats:
        title = (f"{cfg.module} {'robot' if cfg.robot else 'side'}, {cfg.servo}, {cfg.pillar} "
                 f"pillars, {cfg.pin} pins, {cfg.crank} crank, {cfg.sheet}")
        try:
            bom = bom_from_mechanism(replace(mech, bom_extras=extras), title=title,
                                     filament=filament, groups=groups)
            if cfg.robot and (note := torque_limit_note(cfg)):
                bom.notes.append(note)
            files += bom.write(out)
            bom_summary = {"items": len(bom.purchased), "cost_usd": round(bom.cost_usd, 2),
                           "printed_g": bom.printed_g,
                           "unpriced": [r.key for r in bom.unpriced]}
        except KeyError as e:
            rep.failures.append(Failure.from_exception(e, stage="bom"))
    with _timed("glb/mjcf" if job is None else "glb/mjcf (waiting for its worker)"):
        files += _robot_files(design, formats, out, job)
    if "mjcf" in formats and design.kind != "walker":
        # the MJCF is the walking robot's (two sides on a floor, the drives walking it); a
        # mechanism has nothing to walk, so the format is skipped, not an error
        logging.getLogger("spiderpig.export").warning(       # on ExportReport.warnings
            "mjcf: %s is a mechanism, with nothing to walk: the MJCF is a walker's robot "
            "model (verify(\"full\") runs it), so the format is skipped", cfg.linkage)
    return files, bom_summary


GROUPED = ("laser", "printed")


@contextmanager
def _timed(what: str):
    """Log (debug) how long a step of the export took in this process."""
    t0 = time.perf_counter()
    try:
        yield
    finally:
        log.debug("export: %s %.1f s", what, time.perf_counter() - t0)


def _start_group_job(mech):
    """:func:`group_made` of the made parts in a worker (the parts sent as binary BReps:
    the same doubles, so the same proofs) while this process writes STEP and STL (the
    caller starts it only then); ``None`` with workers off."""
    from spiderpig import workers

    if not workers.enabled():
        return None
    made = [(b.name, b.fab, workers.dump_shape(b.part)) for b in mech.bodies
            if b.fab in GROUPED and b.part is not None]
    return workers.submit(_group_job, made)


def _group_job(made) -> dict[str, list[tuple[str, list[str], list[str]]]]:
    """In a worker: the groups of ``made`` (name, fab, dumped part) by name."""
    from types import SimpleNamespace

    from spiderpig.workers import load_shape

    bodies = [SimpleNamespace(name=n, fab=fab, part=load_shape(d)) for n, fab, d in made]
    return {m: [(g.ref.name, list(g.names), list(g.mirrored)) for g in group_made(bodies, m)]
            for m in GROUPED}


def _groups(mech, job) -> dict:
    """The made parts' groups: computed here, or the worker's (``job``) on these bodies."""
    from spiderpig.hardware.bom import MadeGroup

    if job is None:
        return {m: group_made(mech.bodies, m) for m in GROUPED}
    by_name = {b.name: b for b in mech.bodies}
    return {m: [MadeGroup(m, by_name[ref], names, mirrored) for ref, names, mirrored in gs]
            for m, gs in job.result().items()}


def _walker_formats(design: Design, formats: list[str]) -> list[str]:
    """The formats of the walker at the bake's reference angle: ``glb``, and ``mjcf`` for a
    walker (a mechanism's is skipped, with a warning)."""
    return [f for f in ("glb", "mjcf") if f in formats
            and (f == "glb" or design.kind == "walker")]


def _start_robot_job(design: Design, formats: list[str], before_build: bool = False):
    """The glb and the MJCF in a worker process (:mod:`spiderpig.workers`) while this one
    builds (``before_build``) or writes the other formats: they share nothing with them
    but the design, which the worker loads from the store (its plan re-made and
    verified) and fabricates at the bake's angle itself, as :func:`_robot_files` does
    here. The worker writes into a folder of its own, whose files :func:`_robot_files`
    moves into the export's. ``None`` (written here, after the others) when the design
    has no store or no plan, there is nothing to write or nothing to do meanwhile (a
    worker's start, 4-5 s, would only add), or workers are off (``SPIDERPIG_WORKERS=0``).
    """
    import tempfile

    from spiderpig import workers

    fmts = _walker_formats(design, formats)
    others = [f for f in formats if f not in ("glb", "mjcf")]
    if (not fmts or not (others or before_build) or design.store is None
            or not workers.enabled() or not plan(design).ok):
        return None
    staging = Path(tempfile.mkdtemp(prefix="spiderpig-export-"))
    future = workers.submit(_robot_files_job, str(design.store.root), design.id, fmts,
                            str(staging))
    future.staging = staging  # pyright: ignore[reportAttributeAccessIssue]  # _robot_files' folder
    return future


def _drop_robot_job(job) -> None:
    """Forget a glb/MJCF worker whose export failed: its folder goes when it is done."""
    if job is not None:
        import shutil

        job.add_done_callback(lambda _: shutil.rmtree(job.staging, ignore_errors=True))


def _robot_files_job(root: str, id: str, formats: list[str], out: str
                     ) -> tuple[list[str], list[str]]:
    """:func:`_robot_files` in a worker: the files written and what was warned meanwhile."""
    design = load(id, root)
    with contextlib.ExitStack() as stack:
        warned = stack.enter_context(capture_warnings(EXPORT_LOGGERS))
        stack.enter_context(pywarnings.catch_warnings())
        pywarnings.filterwarnings("ignore", message="Unknown Compound type")
        if not plan(design).ok:
            raise RuntimeError(f"{id}: the stored plan no longer holds")
        files = _robot_files(design, formats, Path(out))
    return [str(f) for f in files], warned


def _robot_files(design: Design, formats: list[str], out: Path, job=None) -> list[Path]:
    """The ``glb`` and ``mjcf`` formats: written here, or collected from ``job``
    (:func:`_start_robot_job`), whose warnings are logged again here so the export's
    report has them, in the order a serial export would."""
    cfg, name = design.config, design.config.linkage
    files: list[Path] = []
    if job is not None:
        import shutil

        try:
            done, warned = job.result()
            moved = []
            for f in map(Path, done):
                shutil.move(f, out / f.name)
                moved.append(out / f.name)
        finally:
            shutil.rmtree(job.staging, ignore_errors=True)
        for w in warned:
            logging.getLogger("spiderpig.export").warning("%s", w)
        return moved
    robot, props = None, {}          # the parts' mass properties: the bake's, for the MJCF
    if "glb" in formats or ("mjcf" in formats and design.kind == "walker"):
        # the viewer's bake and the MuJoCo model are both of the walker at the bake's
        # reference angle: fabricated once here from the design's own side (its plan), not
        # again by each from the config (which, in a fresh process, planned again first)
        from spiderpig.bake import T_REF

        robot = building.fabricate_at(design, T_REF)
    if "glb" in formats:
        from spiderpig.bake import bake_gltf

        bake_gltf(out / f"{name}.glb", cfg, profile=False, fabricated=robot, side=design.side,
                  props=props)
        files.append(out / f"{name}.glb")
    if "mjcf" in formats and design.kind == "walker":
        import json

        from spiderpig.sim.mjcf import build_mjcf, set_fabricated

        if cfg.robot:        # a one-sided design's MJCF is still the robot's
            assert robot is not None    # fabricated above for a walker's mjcf
            set_fabricated(cfg, robot, props)
        xml, meta = build_mjcf(cfg)
        (out / f"{name}.xml").write_text(xml)
        (out / f"{name}.json").write_text(json.dumps(meta, indent=1))
        files += [out / f"{name}.xml", out / f"{name}.json"]
    return files
