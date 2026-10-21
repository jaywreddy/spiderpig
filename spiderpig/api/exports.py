"""Verification (:func:`verify`) and the exports (:func:`export`)."""


from __future__ import annotations

import contextlib
import logging
import time
import warnings as pywarnings
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path

from spiderpig import fabmethods
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
from spiderpig.spec import urdf_unsupported
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


MAKE_FORMATS = ("guide", "control", "order")
"""What a maker reads beside the files a spec's ``outputs`` name: the assembly guide, the
controller's files, the shopping list (export formats only, never a spec's outputs)."""


def formats_all() -> tuple[str, ...]:
    """Every format :func:`export` writes: a spec's ``outputs`` (:data:`spec.OUTPUTS`) and
    :data:`MAKE_FORMATS`."""
    from spiderpig.spec import OUTPUTS

    return (*OUTPUTS, *MAKE_FORMATS)


def export(design: Design, formats=None, out_dir: str | Path | None = None,
           force: bool = False) -> ExportReport:
    """Write the chosen ``formats`` (default: the spec's ``outputs``) into ``out_dir``
    (default: the design's ``exports/`` in its store, else ``build/``), as ``spiderpig build``
    does: ``step`` / ``stl`` (the whole machine), ``print`` (one STL per different printed
    part + ``parts.csv``), ``dxf`` (kerf-compensated sheets + ``parts.csv``), ``bom``
    (csv, md, json), ``glb`` (the viewer's animated bake), ``mjcf`` (the MuJoCo model +
    its metadata, every kind's: :mod:`spiderpig.sim.mjcf`), ``urdf`` (an arm's URDF for
    ROS, its meshes and joint map: :mod:`spiderpig.urdf`; asked of a crank's design it is
    refused, a ``ValueError`` with :func:`spiderpig.spec.urdf_unsupported`'s reason, as
    ``resolve`` refuses it in a spec's outputs), ``guide`` (the assembly guide,
    ``ASSEMBLY.pdf`` and ``ASSEMBLY.md``: :mod:`spiderpig.guide.make`), ``control`` (the
    controller's files in ``control/``, by what drives the design: a crank's bus servos'
    ``servos.json``, an arm's steppers' Klipper ``printer.cfg``, and either's ``HOMING.md``:
    :mod:`spiderpig.control.files`), ``order`` (``ORDER.md``, a cart per vendor, an upload
    per cutting service and the prints, with the files it names: ``laser/parts/`` and
    ``print/``: :mod:`spiderpig.hardware.order`);
    and always ``manifest.json``. Plans and builds first if nothing is built (every
    format, ``guide``, ``control`` and ``order`` included, reads the planned side: an arm's
    plan is minutes cold, seconds once the store keeps it; the MCP's ``export`` is a job
    for it); a sheet-packing failure is ``layout``, a catalog miss ``bom``. What the bake and the
    constructions warned about while writing (a purchased model's faces the mesher
    skipped) is on the report (``warnings``) and in the manifest. An export the store
    records for the same formats and folder, whose files are all still there, is
    returned as is (``force`` rewrites)."""
    with design_lock(design):
        return _export(design, formats, out_dir, force)


def _export(design: Design, formats, out_dir, force: bool) -> ExportReport:
    t0 = time.time()
    asked = bool(formats)
    formats = list(formats or design.spec.outputs)
    have = formats_all()
    bad = [f for f in formats if f not in have]
    if bad:
        raise ValueError(f"unknown formats {bad}; have {list(have)}")
    # a format the linkage can't have: refused when asked for, as resolve refuses it in a
    # spec's outputs (a spec stored before that only skips it, with the same reason)
    if "urdf" in formats and (why := urdf_unsupported(design.lk)) and asked:
        raise ValueError(why)
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
        for stale in [*(out / "urdf").glob("*.json"), *(out / "urdf").glob("*.xml")]:
            stale.unlink()                   # an arm's: its URDF and joint map
        clear_generated(out / "urdf")
        for owned in ("ORDER.md", "ASSEMBLY.pdf", "ASSEMBLY.md"):
            (out / owned).unlink(missing_ok=True)
        from spiderpig.control.files import FILES

        for name in {f for fs in FILES.values() for f in fs}:
            (out / "control" / name).unlink(missing_ok=True)
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
    want_print = "print" in formats or "order" in formats    # ORDER.md lists the prints
    want_bom = "bom" in formats or "order" in formats        # ... and buys the BOM
    grouping = (_start_group_job(mech) if (want_print or want_bom)
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
    if want_print or want_bom:
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

    print_rows: list[dict] = []
    if want_print:
        from spiderpig import build as build_cli

        assert groups is not None   # grouped just above for "print"
        with _timed("print"):
            build_cli.clear_generated(out / "print")     # no STLs of another design
            from spiderpig.hardware.bom import printed_filaments
            from spiderpig.labels import print_stems

            print_rows = build_cli.export_prints(groups["printed"], out / "print",
                                                 density=filament_density(filament),
                                                 filaments=printed_filaments(mech, filament),
                                                 stems=print_stems(labelled()),
                                                 poses=mech.meta.get("print_poses"),
                                                 shapes=mech.meta.get("print_shapes"))
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
    elif want_bom:      # the BOM buys the sheets whether or not the DXF is written
        try:
            extras += sheet_lines(mech, cfg.sheet, size)
        except ValueError as e:
            rep.failures.append(Failure.from_exception(e, stage="layout"))
    from spiderpig.build import bom_title

    bom = None
    title = bom_title(cfg)
    if want_bom:
        try:
            bom = bom_from_mechanism(replace(mech, bom_extras=extras), title=title,
                                     filament=filament, groups=groups)
            if note := torque_limit_note(cfg):      # every crank servo's (its motor role)
                bom.notes.append(note)
            if "bom" in formats:
                files += bom.write(out)
            bom_summary = {"items": len(bom.purchased), "cost_usd": round(bom.cost_usd, 2),
                           "printed_g": bom.printed_g,
                           "unpriced": [r.key for r in bom.unpriced]}
        except KeyError as e:
            rep.failures.append(Failure.from_exception(e, stage="bom"))
    if "order" in formats and bom is not None:
        assert groups is not None   # grouped above for "order"
        files += _order_files(design, out, groups, bom, print_rows, labelled, title, kerf,
                              rep)
    if "urdf" in formats:
        if why := urdf_unsupported(design.lk):      # (a spec's own outputs: _export said)
            logging.getLogger("spiderpig.export").warning("urdf skipped: %s", why)
        else:
            from spiderpig import urdf

            with _timed("urdf"):
                build_t = design.build_t if design.build_t is not None else 1.0
                files += urdf.write(mech, cfg, build_t, out / "urdf", name)
    with _timed("glb/mjcf" if job is None else "glb/mjcf (waiting for its worker)"):
        files += _robot_files(design, formats, out, job)
    if "control" in formats:
        files += _control_files(design, out, rep)
    if "guide" in formats:
        with _timed("guide"):
            files += _guide_files(design, out)
    return files, bom_summary


def _order_files(design: Design, out: Path, groups: dict, bom, print_rows: list[dict],
                 labelled, title: str, kerf: float | None, rep: ExportReport) -> list[Path]:
    """``ORDER.md`` (:func:`hardware.order.order_markdown`) and the per-part cut files it
    uploads (``laser/parts/``, :func:`layout.save_parts`), as ``spiderpig build`` writes
    them; a part that can't be laid out is a ``layout`` failure (no ORDER.md then)."""
    from spiderpig import build as build_cli
    from spiderpig.hardware.order import order_markdown
    from spiderpig.labels import laser_labels
    from spiderpig.layout import save_parts

    parts = out / "laser" / "parts"
    build_cli.clear_generated(parts)          # no per-part DXF of another design
    try:
        with _timed("order"):
            rows = save_parts(groups["laser"], parts, design.config.sheet, kerf=kerf,
                              labels=laser_labels(labelled()))
    except ValueError as e:
        rep.failures.append(Failure.from_exception(e, stage="layout"))
        return []
    (out / "ORDER.md").write_text(order_markdown(bom, rows, print_rows, title=title,
                                                 build_dir=str(out)))
    written = sorted(f for f in parts.rglob("*") if f.is_file()) if parts.is_dir() else []
    return [*written, out / "ORDER.md"]


def _control_files(design: Design, out: Path, rep: ExportReport) -> list[Path]:
    """The controller's files (:func:`spiderpig.control.files.write`: one dispatch on what
    drives the design) into ``out/control/``, from its planned side; a servo with no bus
    protocol spiderpig knows is a ``control`` failure."""
    from spiderpig.control import files as control_files

    plan(design)                # (the stored plan's side: an arm's axes read its drives)
    try:
        with _timed("control"):
            return control_files.write(design.config, out / "control", design.side)
    except ValueError as e:
        rep.failures.append(Failure.from_exception(e, stage="control"))
        return []


def _guide_files(design: Design, out: Path) -> list[Path]:
    """``ASSEMBLY.pdf`` and ``ASSEMBLY.md`` (:func:`spiderpig.guide.make.build_guide`) of
    the design's planned side, cached beside its fabrication in its store; an edited
    handle's from its own parts (not cached)."""
    from spiderpig.guide.make import build_guide

    plan(design)
    mech = design.mech if design.edited and design.build_t == 1.0 else None
    build_guide(design.config, design.store, out, design=design.side, mech=mech)
    return [out / "ASSEMBLY.pdf", out / "ASSEMBLY.md"]




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
            if b.fab in fabmethods.made() and b.part is not None]
    return workers.submit(_group_job, made)


def _group_job(made) -> dict[str, list[tuple[str, list[str], list[str]]]]:
    """In a worker: the groups of ``made`` (name, fab, dumped part) by name."""
    from types import SimpleNamespace

    from spiderpig.workers import load_shape

    bodies = [SimpleNamespace(name=n, fab=fab, part=load_shape(d)) for n, fab, d in made]
    return {m: [(g.ref.name, list(g.names), list(g.mirrored))
                for g in group_made(bodies, m)] for m in fabmethods.made()}


def _groups(mech, job) -> dict:
    """The made parts' groups: computed here, or the worker's (``job``) on these bodies."""
    from spiderpig.hardware.bom import MadeGroup

    if job is None:
        return {m: group_made(mech.bodies, m) for m in fabmethods.made()}
    by_name = {b.name: b for b in mech.bodies}
    return {m: [MadeGroup(m, by_name[ref], names, mirrored) for ref, names, mirrored in gs]
            for m, gs in job.result().items()}


def _robot_formats(formats: list[str]) -> list[str]:
    """The formats of the design at the bake's reference angle: ``glb`` and ``mjcf`` (every
    kind's: :mod:`spiderpig.sim.mjcf`)."""
    return [f for f in ("glb", "mjcf") if f in formats]


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

    fmts = _robot_formats(formats)
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
    if "glb" in formats or "mjcf" in formats:
        # the viewer's bake and the MuJoCo model are both of the design at the bake's
        # reference angle: fabricated once here from the design's own side (its plan), not
        # again by each from the config (which, in a fresh process, planned again first)
        from spiderpig.bake import T_REF

        robot = building.fabricate_at(design, T_REF)
    if "glb" in formats:
        from spiderpig.bake import bake_gltf

        bake_gltf(out / f"{name}.glb", cfg, profile=False, fabricated=robot, side=design.side,
                  props=props)
        files.append(out / f"{name}.glb")
    if "mjcf" in formats:
        import json

        from spiderpig.sim.mjcf import build_mjcf, model_config, set_fabricated

        if model_config(cfg) == cfg:     # (a walker's one-sided design: still its robot's)
            assert robot is not None    # fabricated above for the mjcf
            set_fabricated(cfg, robot, props)
        xml, meta = build_mjcf(cfg)
        (out / f"{name}.xml").write_text(xml)
        (out / f"{name}.json").write_text(json.dumps(meta, indent=1))
        files += [out / f"{name}.xml", out / f"{name}.json"]
    return files
