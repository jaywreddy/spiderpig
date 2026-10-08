"""Command-line entry point: build the walking robot and everything needed to make it.

Plans the layer stack of one side, fabricates the robot (two mirror-image
sides, servos back to back in one frame; see :mod:`construction.robot`) and
writes into ``--out``:

* ``<name>.step`` / ``<name>.stl`` — the whole robot;
* ``print/`` — one STL per different printed part, placed on the build
  plate, ``<part>_mirrored.stl`` where the right side needs the mirror image,
  and ``parts.csv`` with how many of each to print;
* ``laser/<name>_sheet_*.dxf`` — every laser-cut part, kerf-compensated and
  packed on the sheet stock's size, with ``<name>_sheet_parts.csv`` saying
  which part is where;
* ``laser/parts/`` — the same parts one DXF per different part
  (``<service>_<sheet>/<part>_x<qty>.dxf``) with ``order.csv`` (material,
  thickness, quantity per file): what a per-part service (SendCutSend) takes;
* ``bom.csv`` / ``bom.md`` / ``bom.json`` — what to buy (quantities, packs,
  vendor links, estimated cost), cut and print;
* ``ORDER.md`` — the same as orders: a cart per vendor (direct product pages),
  an upload per cutting service (the per-part DXFs), the prints per filament.

The design parameters: ``--linkage`` (Strider by default; ``--list`` shows
them all), ``--phases`` (every leg's crank phase in degrees, e.g.
``0,175,180,355`` for a quad) and ``--proportion NAME=VALUE`` (repeatable;
overrides one of the linkage's parameters). ``spiderpig tune``
searches for good ones.

Usage
-----
    spiderpig build                           # quad robot, defaults (mise run build)
    spiderpig build --module single --out build/single
    spiderpig build --phases 0,175,180,355       # the tuned quad gait
    spiderpig build --linkage jansen --module double
    spiderpig build --list                    # linkages, modules, servos, constructions
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import math
import os
import re
import shutil
import sys
import tempfile
import warnings
from pathlib import Path

from spiderpig import construction, linkage, servos, uptodate
from spiderpig.config import (
    ParamError,
    add_build_args,
    add_design_args,
    config_from_args,
    torque_limit_note,
)
from spiderpig.fabricate import design_side, fabricate, split_parts, template_for
from spiderpig.hardware.bom import (
    MadeGroup,
    bom_from_mechanism,
    group_made,
    printed_filaments,
)
from spiderpig.hardware.catalog import CATALOG, _load
from spiderpig.hardware.mass import filament_density
from spiderpig.layout import DEFAULT_KERF, save_parts, save_sheets, sheet_lines
from spiderpig.uptodate import GENERATED  # api.export's clear_generated reads it here


def _parse_args(argv) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Walking-robot generator")
    add_design_args(p)
    p.add_argument("--side-only", action="store_true",
                   help="build one side (no second side, no chassis)")
    add_build_args(p)
    p.add_argument("--kerf", type=float, default=None,
                   help="laser kerf compensation in mm on every sheet (default: each sheet's "
                        "service's: 0 at SendCutSend, which compensates itself, 0.2 at "
                        f"Ponoko; {DEFAULT_KERF:g} where a sheet names none)")
    p.add_argument("--sheet-size", type=float, nargs=2, metavar=("W", "H"), default=None,
                   help="usable sheet size in mm (default: the sheet stock's size)")
    p.add_argument("--out", type=Path, default=Path("build"),
                   help="output directory (created if missing). Default: ./build")
    p.add_argument("--name", default=None, help="file-name stem. Default: the linkage (strider)")
    p.add_argument("--no-dxf", action="store_true", help="skip the DXF sheet-packing pass")
    p.add_argument("--list", action="store_true",
                   help="list modules, servos, constructions and sheet stock")
    p.add_argument("--store", metavar="PATH",
                   help="the design store the options resolve into, whose plan is reused "
                        "and whose fabrication cache serves the parts "
                        "(default: $SPIDERPIG_STORE, else ./.spiderpig)")
    p.add_argument("--force", action="store_true",
                   help="build even when --out already holds this build's outputs, "
                        "unchanged (else it says so and does nothing)")
    args = p.parse_args(argv)
    try:            # robot=None: the linkage's kind decides (a mechanism is one side)
        args.config = config_from_args(args, robot=False if args.side_only else None)
    except ParamError as e:
        p.error(str(e))
    args.name = args.name or args.config.linkage
    return args


def _list_options() -> None:
    _load()
    print("linkages (--linkage; --module one of its modules; --proportion NAME=VALUE for its "
          "parameters):")
    for key in linkage.available():
        lk = linkage.get(key)
        params = ", ".join(f"{k}={float(v):g}" for k, v in lk.params.items())
        print(f"  {key:14} {lk.name} ({', '.join(lk.leg_modules)}): {params}")
    print("servos (full rotation):")
    for key in servos.available():
        print(f"  {key:14} {servos.get(key).name}")
    for title, registry in (("pillars / pins (--pillar, --pin)", construction.AXLES),
                            ("cranks (--crank)", construction.CRANKS)):
        print(f"{title}:")
        for key, c in sorted(registry.items()):
            print(f"  {key:14} {getattr(c, 'label', '')}")
    print("sheet stock (--sheet):")
    for key, it in sorted(CATALOG.items()):
        if it.category == "sheet":
            print(f"  {key:14} {it.name}")


def _file_stem(name: str, taken: set[str]) -> str:
    stem = re.sub(r"[^A-Za-z0-9_.-]", "_", re.sub(r"^[LR]\.", "", name))
    if stem in taken:
        stem = re.sub(r"[^A-Za-z0-9_-]", "_", name)
    taken.add(stem)
    return stem


def _on_plate(part):
    """``part`` moved so it stands on z = 0, centred on the origin in XY."""
    from build123d import Location

    from spiderpig.shapes import moved

    bb = part.bounding_box()
    return moved(part, Location((-(bb.min.X + bb.max.X) / 2, -(bb.min.Y + bb.max.Y) / 2,
                                 -bb.min.Z)))


def export_prints(groups, out_dir: Path, density: float = 1.24,
                  filaments: dict[str, str | None] | None = None) -> list[dict]:
    """One STL per different printed part (and its mirror image where needed).

    ``filaments``: each printed body's filament (catalog key, by name:
    :func:`hardware.bom.printed_filaments`): a group whose parts take two filaments is
    two rows, and each row says its filament and grams at its density; without it every
    part is at ``density`` and the filament column is empty."""
    from build123d import Plane

    from spiderpig.hardware.bom import _filament_name, _split_by
    from spiderpig.mesh import export_stl

    out_dir.mkdir(parents=True, exist_ok=True)
    rows, taken = [], set()
    if filaments is not None:
        by_name = {g.ref.name: g.ref for g in groups}
        groups = [part for g in groups for part in _split_by(g, filaments, by_name)]
    for g in groups:
        # the row's own filament: a split row's ref may be a body of the other filament
        fil = (filaments or {}).get(g.names[0] if g.names else g.ref.name)
        # named after its own parts (a split row's ref may be the other filament's body)
        own = g.ref.name if g.ref.name in g.names or not g.names else g.names[0]
        stem = _file_stem(own, taken)
        part = _on_plate(g.ref.part)
        export_stl(part, str(out_dir / f"{stem}.stl"))
        same = g.qty - len(g.mirrored)
        todo = f"print {same}"
        files = [f"{stem}.stl"]
        if g.mirrored:
            export_stl(_on_plate(g.ref.part.mirror(Plane.YZ)),
                       str(out_dir / f"{stem}_mirrored.stl"))
            files.append(f"{stem}_mirrored.stl")
            todo += f", and {len(g.mirrored)} mirrored ({stem}_mirrored.stl)"
        bb = g.ref.part.bounding_box()
        rows.append({
            "file": files[0], "qty": g.qty, "mirrored": len(g.mirrored), "print": todo,
            "size_mm": f"{bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f}",
            "filament": _filament_name(fil) if fil else "",
            "grams_each_100pct": round(g.ref.part.volume / 1000
                                       * (filament_density(fil) if fil else density), 1),
            "parts": " ".join(g.names),
        })
    with open(out_dir / "parts.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else ["file"])
        w.writeheader()
        w.writerows(rows)
    return rows


def clear_generated(folder: Path) -> None:
    """Delete the files a build writes (:data:`GENERATED`) under ``folder`` and the
    folders that leaves empty; anything else there (a user's notes) stays."""
    if not folder.is_dir():
        return
    for f in folder.rglob("*"):
        if f.is_file() and f.suffix.lower() in GENERATED:
            f.unlink()
    for d in sorted((d for d in folder.rglob("*") if d.is_dir()), key=lambda d: -len(d.parts)):
        if not any(d.iterdir()):
            d.rmdir()
    if not any(folder.iterdir()):
        folder.rmdir()


# ---------------------------------------------------------------------------
# the export worker: the grouping and the cut files beside the STEP and STL
# ---------------------------------------------------------------------------

GROUPED = ("laser", "printed")


class _ExportJob:
    """The export worker (:func:`_exports_job`), the file that tells it where the
    fabrication is (:meth:`go`, once), and the folder it writes its cut files into
    (``staging``, in ``--out``): moved into ``--out`` once the build takes them
    (:meth:`adopt`), removed when it fails first (:meth:`abandon`), so a failed build
    leaves no cut file behind."""

    def __init__(self, future, folder: Path, staging: Path):
        self.future, self.folder, self.staging, self.sent = future, folder, staging, False
        future.add_done_callback(lambda _: shutil.rmtree(folder, ignore_errors=True))

    def go(self, entry: Path | None) -> None:
        """The fabrication cache's entry to load (``None``: none, the worker returns)."""
        if self.sent:
            return
        self.sent = True
        tmp = self.folder / "go.tmp"
        tmp.write_text("" if entry is None else str(entry))
        os.replace(tmp, self.folder / "go")         # (whole, or not there)

    def adopt(self, out: Path) -> None:
        """Move what the worker wrote into ``out`` (the same tree under it)."""
        for f in sorted(p for p in self.staging.rglob("*") if p.is_file()):
            dest = out / f.relative_to(self.staging)
            dest.parent.mkdir(parents=True, exist_ok=True)
            os.replace(f, dest)
        shutil.rmtree(self.staging, ignore_errors=True)

    def abandon(self) -> None:
        """The build stops without the worker's files: it is told there is nothing to
        load (if it wasn't told yet), waited for, and what it wrote removed."""
        self.go(None)
        with contextlib.suppress(Exception):
            self.future.result()
        shutil.rmtree(self.staging, ignore_errors=True)


def _start_exports(store, out: Path, args, config) -> _ExportJob | None:
    """Start the export worker before the fabrication, or ``None`` (everything in this
    process) with workers or the fabrication cache off.

    The STEP and STL of the whole robot are this process's: their texts follow the parts'
    locations to the last bit, which a part reloaded from the cache doesn't keep (``-0.``
    for ``0.``: :mod:`spiderpig.fabcache`), while the grouping (whose decisions are the
    boolean's), the sheets and the per-part DXFs come out the same from either."""
    from spiderpig import fabcache, workers

    if not (workers.enabled() and fabcache.enabled()):
        return None
    folder = Path(tempfile.mkdtemp(prefix="spiderpig-build-"))
    staging = Path(tempfile.mkdtemp(prefix=".spiderpig-exports-", dir=out))
    size = tuple(args.sheet_size) if args.sheet_size else None
    future = workers.submit(_exports_job, str(folder / "go"), os.getpid(), str(staging),
                            args.name, config.sheet, args.kerf, size, not args.no_dxf)
    return _ExportJob(future, folder, staging)


def _published(store, tmpl, config, design) -> Path | None:
    """The cache's entry of the fabrication just served, ``None`` when there is none (the
    cache couldn't write it, or the fabrication didn't go through it)."""
    from spiderpig import fabcache

    try:
        entry = fabcache.entry_path(store, tmpl, config, design, 1.0)
    except Exception:       # noqa: BLE001 - no key: the export runs here
        return None
    return entry if entry is not None and entry.is_dir() else None


def _exports_result(job: _ExportJob) -> dict | None:
    """The worker's groups and cut files (``None``: it couldn't load the fabrication)."""
    return job.future.result()


def _outcome(result: tuple):
    """A step's value from the worker, or its exception raised here."""
    ok, value = result
    if not ok:
        raise value
    return value


def _attempt(fn) -> tuple:
    try:
        return True, fn()
    except Exception as e:      # noqa: BLE001 - raised again in the build (_outcome)
        return False, e


def _await_go(go: Path, parent: int) -> str | None:
    """The entry the build names in ``go`` (``None``: none, or the build is gone)."""
    import time

    while not go.is_file():
        if os.getppid() != parent:
            return None
        time.sleep(0.02)
    return go.read_text() or None


def _exports_job(go: str, parent: int, out: str, name: str, sheet: str,
                 kerf: float | None, size: tuple[float, float] | None,
                 dxf: bool) -> dict | None:
    """In a worker: :func:`group_made` of the fabrication the cache holds at the entry
    ``go`` names, and with ``dxf`` the sheets (:func:`save_sheets`, :func:`sheet_lines`)
    and the per-part DXFs (:func:`save_parts`) written into ``out`` as :func:`main` would;
    each step's value or exception (:func:`_outcome`), the groups by name, and each step's
    seconds (``timings``: ``spiderpig build --profile`` logs them as ``worker.*``).
    ``None`` when there is no entry or it can't be read (the build does it all)."""
    import logging
    import time

    from spiderpig import fabcache

    entry = _await_go(Path(go), parent)
    if entry is None:
        return None
    t0 = time.perf_counter()
    try:
        mech = fabcache.load_mechanism(Path(entry))
    except Exception as e:      # noqa: BLE001 - the build groups and cuts itself
        logging.getLogger("spiderpig.build").warning("%s: unreadable (%s)", entry, e)
        return None
    timings = {"load": time.perf_counter() - t0}
    t0 = time.perf_counter()
    groups = {m: group_made(mech.bodies, m) for m in GROUPED}
    timings["group"] = time.perf_counter() - t0
    res: dict = {"groups": {m: [(g.ref.name, list(g.names), list(g.mirrored)) for g in gs]
                            for m, gs in groups.items()}, "timings": timings}
    if not dxf:
        return res
    t0 = time.perf_counter()
    laser = Path(out) / "laser"
    res["sheets"] = _attempt(lambda: [str(p) for p in save_sheets(
        mech, laser / f"{name}_sheet", sheet_size=size, kerf=kerf, default=sheet)])
    if not res["sheets"][0]:
        return res
    res["lines"] = _attempt(lambda: sheet_lines(mech, sheet, size))
    timings["dxf_sheets"] = time.perf_counter() - t0
    if not res["lines"][0]:
        return res
    t0 = time.perf_counter()
    res["order"] = _attempt(lambda: save_parts(groups["laser"], laser / "parts", sheet,
                                               kerf=kerf))
    timings["dxf_parts"] = time.perf_counter() - t0
    return res


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    checked = uptodate.take(argv)   # the CLI's answer, else checked now (before any read)
    if checked.skip:                # --out holds this very build already (said so)
        return 0
    opts, key = checked.opts, checked.key
    args = _parse_args(argv)
    if args.list:
        _list_options()
        return 0
    config = args.config
    out: Path = args.out
    out.mkdir(parents=True, exist_ok=True)
    if opts is not None:
        uptodate.forget(opts)       # (a build that stops short is never current)
    for owned in ("laser", "print"):     # no cut or print files left from an earlier build
        clear_generated(out / owned)
    # an export's manifest no longer describes the folder (api.export reuses one by it):
    # this build writes its own once it has written everything; nor do an earlier build's
    # shopping list and BOM (a build that stops short leaves none of them behind)
    for owned in ("manifest.json", "ORDER.md", "bom.csv", "bom.md", "bom.json"):
        (out / owned).unlink(missing_ok=True)
    from spiderpig.stages.resolve import config_warnings

    for w in config_warnings(config):       # what the API's resolve would warn about
        print(f"warning: {w}", file=sys.stderr)

    try:
        tmpl = template_for(config)     # the template stage checks every loop closes
    except linkage.AssemblyError as e:
        print(f"error: the linkage can't be assembled: {e}", file=sys.stderr)
        return 2
    phases = ",".join(f"{math.degrees(p):g}" for _, p in config.legs)
    custom = config.phases is not None or config.proportions
    design_note = (f"{config.linkage} linkage, leg phases {phases} deg, proportions "
                   f"{dict(config.proportions) or 'its defaults'}")
    if custom or config.linkage != linkage.DEFAULT:
        print(f"design: {design_note}")
    # the plan through the store (stages.plan_config), as explain and audit do: the stored
    # design's when it holds one (re-made and verified), else solved once and recorded;
    # design_side then answers from what plan_config remembered
    from spiderpig import fabcache
    from spiderpig.stages.planning import plan_config
    from spiderpig.store import Store

    store = Store.of(args.store) if args.store else Store.default()
    try:
        plan_config(config, store)
    except ValueError as e:
        print(f"error: no layer plan: {e}", file=sys.stderr)
        return 2
    design = design_side(tmpl, config)
    plan = design.plan
    print(f"{config.module}: layer plan of one side, {plan.top + 1} layers of "
          f"{config.pitch:g} mm ({plan.height:.1f} mm):")
    print(plan.describe())
    # (key is set exactly when opts is: uptodate.check_once)
    read = uptodate.inputs(opts, config) if opts is not None and key is not None else None
    # the grouping and the cut files in a worker beside the STEP and STL, from the
    # fabrication the cache keeps (started now: its imports overlap the fabrication)
    job, entry = _start_exports(store, out, args, config), None
    try:
        try:
            with fabcache.serving(store):   # the store's fabrication when it holds this one
                mech = fabricate(tmpl, config, 1.0)
            split = split_parts(mech)
            if job is not None and not split:
                entry = _published(store, tmpl, config, design)
        finally:
            if job is not None:
                job.go(entry)               # (None: the worker returns at once, unused)
        if entry is None and job is not None:
            job.abandon()
            job = None
        if split:           # no cut file or BOM of a part in pieces
            print("error: parts in pieces: " + ", ".join(f"{name} ({n} solids)"
                                                         for name, n in split),
                  file=sys.stderr)
            return 2
        if config.robot:
            m = mech.meta
            print(f"chassis: {m['centre_plates']} centre plates; rear screws "
                  f"{m.get('rear_screws_per_servo', 0)} x {m.get('rear_screw')} per servo; "
                  f"{m.get('ties', 0)} frame ties ({m.get('tie_screw')})")

        step_path, stl_path = out / f"{args.name}.step", out / f"{args.name}.stl"
        with warnings.catch_warnings():
            # build123d's "Unknown Compound type, color not set" on a purchased model's
            # compound: the colours are ours to set, the file is complete
            warnings.filterwarnings("ignore", message="Unknown Compound type")
            mech.export_step(step_path)
        mech.export_stl(stl_path)
        print(f"wrote {step_path} and {stl_path}")

        done = _exports_result(job) if job is not None else None
    except BaseException:
        if job is not None:
            job.abandon()                   # a failed build leaves none of its cut files
        raise
    if job is not None:
        job.adopt(out)                      # (the build reports on them below)
    if done is None:
        groups = {method: group_made(mech.bodies, method) for method in GROUPED}
    else:
        by_name = {b.name: b for b in mech.bodies}
        groups = {m: [MadeGroup(m, by_name[ref], names, mirrored)
                      for ref, names, mirrored in gs] for m, gs in done["groups"].items()}
    filament = mech.meta.get("filament", "pla_filament")
    rows = export_prints(groups["printed"], out / "print", density=filament_density(filament),
                         filaments=printed_filaments(mech, filament))
    n_print = sum(r["qty"] for r in rows)
    print(f"wrote {len(rows)} printed-part STLs for {n_print} parts to {out / 'print'}:")
    for r in rows:
        print(f"  {r['file']:28} {r['print']}")

    order: list[dict] = []
    if not args.no_dxf:
        size = tuple(args.sheet_size) if args.sheet_size else None
        try:
            sheets = (save_sheets(mech, out / "laser" / f"{args.name}_sheet", sheet_size=size,
                                  kerf=args.kerf, default=config.sheet)
                      if done is None else _outcome(done["sheets"]))
        except ValueError as e:
            print(f"error: the cut files can't be laid out: {e}", file=sys.stderr)
            return 1
        n_laser = sum(g.qty for g in groups["laser"])
        print(f"wrote {len(sheets)} DXF sheet(s) with {n_laser} laser-cut parts "
              f"({len(groups['laser'])} different) to {out / 'laser'}, one set per sheet:")
        for line in (sheet_lines(mech, config.sheet, size) if done is None
                     else _outcome(done["lines"])):
            print(f"  {line.qty} x {line.key}")
            mech.bom_extras.append(line)
        try:
            order = (save_parts(groups["laser"], out / "laser" / "parts", config.sheet,
                                kerf=args.kerf)
                     if done is None else _outcome(done["order"]))
        except ValueError as e:
            print(f"error: the per-part cut files can't be written: {e}", file=sys.stderr)
            return 1
        print(f"wrote {len(order)} per-part DXFs ({sum(r['qty'] for r in order)} parts) and "
              f"order.csv to {out / 'laser' / 'parts'}")

    title = (f"{config.module} {'robot' if config.robot else 'side'}, {config.servo}, "
             f"{config.pillar} pillars, {config.pin} pins, {config.crank} crank, {config.sheet}")
    bom = bom_from_mechanism(mech, title=title, filament=filament, groups=groups)
    if args.no_dxf:
        bom.notes.append("Sheet stock not counted (--no-dxf).")
    if config.robot and (note := torque_limit_note(config)):
        bom.notes.append(note)
    if custom or config.linkage != linkage.DEFAULT:
        bom.notes.append(f"Design: {design_note}.")
    paths = bom.write(out)
    print(f"wrote {', '.join(str(p) for p in paths)}: {len(bom.purchased)} items to buy, "
          f"est. ${bom.cost_usd:.2f} ({len(bom.unpriced)} without a listed price)")
    from spiderpig.hardware.order import order_markdown

    (out / "ORDER.md").write_text(order_markdown(bom, order, rows, title=title,
                                                 build_dir=str(out)))
    print(f"wrote {out / 'ORDER.md'}: the shopping list (a cart per vendor, uploads, prints)")
    _write_manifest(out, config, args)
    if opts is not None and key is not None and read is not None:   # (all set, or none)
        # what makes the same build again a no-op (the store keeps it)
        uptodate.record(opts, key, read)
    return 0


def _write_manifest(out: Path, config, args=None) -> None:
    """``manifest.json`` naming the design built here (its id as :func:`stages.resolve` gives
    it: with ``--kerf``, which shapes the cut files, as the Spec's ``fit.kerf_mm``), so an
    ``api.export`` of the same design into this folder keeps its cut and print files and
    ORDER.md, and one of another design clears them. It lists no ``formats``: an export
    never takes a build's files for its own earlier ones (:func:`api.export`'s reuse)."""
    import json

    from spiderpig.stages.resolve import resolve, spec_of

    spec = spec_of(config)
    kerf = getattr(args, "kerf", None)
    size = getattr(args, "sheet_size", None)
    if kerf is not None:
        spec.setdefault("fit", {})["kerf_mm"] = kerf
    if size:
        spec.setdefault("fit", {})["sheet_size_mm"] = list(size)
    design = resolve(spec, store=None)
    (out / "manifest.json").write_text(json.dumps(
        {"design": design.id, "engine_version": design.engine_version,
         "written_by": "spiderpig build", "kerf_mm": kerf,
         "sheet_size_mm": list(size) if size else None,
         "name": getattr(args, "name", None)}, indent=1))


if __name__ == "__main__":
    sys.exit(main())
