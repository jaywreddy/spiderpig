"""``spiderpig guide``: the assembly guide, ``ASSEMBLY.pdf`` (and ``ASSEMBLY.md``), for a
design.

The steps are :func:`construction.assembly.assembly_steps` (the constructions' hooks and the
robot's order), the part labels :func:`spiderpig.labels.part_types` (the names of
``spiderpig build``'s print files), the pictures :mod:`guide.render` in
:func:`spiderpig.workers.submit` processes (``--jobs``), the wiring :mod:`guide.wiring`,
the layout :mod:`guide.pdf`.

**Cached** beside the design's fabrication (:mod:`spiderpig.fabcache`, ``<store>/fab/<fab
key>/guide-<entry>/``): every picture under a hash of what it draws and the renderer's
code key (:func:`spiderpig.keys.source_key` of :func:`guide.render.render_jobs`), the
meshes, and the finished guide under the code key of the whole guide
(:func:`build_guide`'s, which reaches every construction's ``assembly`` hook): an
unchanged design is a copy, a changed sentence redraws nothing. ``SPIDERPIG_FAB_CACHE=off``
or no store: nothing is kept.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import logging
import os
import shutil
import tempfile
import time
from contextlib import ExitStack
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

log = logging.getLogger("spiderpig.guide")

PICTURE = (1200, 900)
THUMB = (300, 300)
COVER = (1600, 1200)
EXPLODE = 20.0          # mm the parts a stack step adds are drawn lifted
GUIDE_ROOTS = ("spiderpig.guide.make:build_guide",)
RENDER_ROOTS = ("spiderpig.guide.render:render_jobs",)


@dataclass
class GuideReport:
    pdf: Path
    pages: int = 0
    steps: int = 0
    cached: bool = False
    drawn: int = 0              # pictures drawn now (the rest from the cache)
    seconds: dict[str, float] = field(default_factory=dict)


def _views(step) -> list[str]:
    """The cameras a step may choose among (``render.VIEWS``)."""
    side = step.side or ""
    if step.where == "robot":
        return ["robot", "robot-", "robot^"]
    if step.stage == "stack" and not step.sub:
        return [f"bench_{side}"]          # a side's stack reads the same way throughout
    return [f"bench_{side}", f"bench_{side}-"]


def _fill(kind: str | None) -> tuple[int, int, int]:
    from spiderpig.guide.render import BOUGHT_FILL, HIGHLIGHT

    return BOUGHT_FILL if kind == "purchased" else HIGHLIGHT


def _hash(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True).encode()).hexdigest()[:20]


def _atomic(path: Path, data: bytes) -> None:
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)


def build_guide(config, store, out: Path, *, jobs: int = 4, max_steps: int | None = None,
                force: bool = False, design=None, mech=None) -> GuideReport:
    """Write ``out/ASSEMBLY.pdf`` and ``out/ASSEMBLY.md`` for ``config`` (planned through
    ``store``, a :class:`spiderpig.store.Store` or None). ``design`` / ``mech``: its side
    design and fabrication when the caller has them (nothing is cached then)."""
    from spiderpig import api, fabcache, keys
    from spiderpig.fabricate import design_side, fabricate, template_for

    t0 = time.perf_counter()
    times: dict[str, float] = {}

    def lap(key: str, since: float) -> float:
        now = time.perf_counter()
        times[key] = round(now - since, 2)
        log.info("%-14s %6.2f s", key, now - since)
        return now

    out.mkdir(parents=True, exist_ok=True)
    tmpl = template_for(config)
    given = mech is not None
    if design is None:
        if store is not None:
            api.plan_config(config, store)
        design = design_side(tmpl, config)
    root = (fabcache.root_of(store) if store is not None and fabcache.enabled() and not given
            else None)
    with ExitStack() as stack:
        if root is None:
            cache = Path(stack.enter_context(tempfile.TemporaryDirectory(prefix="guide-")))
        else:
            cache = (root / fabcache.folder_name()
                     / f"guide-{fabcache.entry_name(tmpl, config, design, 1.0)}")
            cache.mkdir(parents=True, exist_ok=True)
            os.utime(cache)                                     # (last used: gc by age)
        done = cache / (f"pdf-{keys.source_key(GUIDE_ROOTS, 'guide')}"
                        + (f"-first{max_steps}" if max_steps else ""))
        t = lap("key", t0)
        if (done / "ASSEMBLY.pdf").is_file() and not force:
            for name in ("ASSEMBLY.pdf", "ASSEMBLY.md"):
                shutil.copyfile(done / name, out / name)
            meta = json.loads((done / "guide.json").read_text())
            lap("total", t0)
            return GuideReport(out / "ASSEMBLY.pdf", meta["pages"], meta["steps"], True, 0,
                               times)
        if mech is None:
            with fabcache.serving(store):
                mech = fabricate(tmpl, config, 1.0)
        t = lap("fabricate", t)
        report = _make(config, design, mech, cache, done, out, jobs, max_steps, force, times,
                       t)
    lap("total", t0)
    return report


def _meshes(mech, cache: Path) -> Path:
    """Every body's placed triangles in one ``.npz`` (made once per fabrication)."""
    from spiderpig.mesh import tessellate_many

    path = cache / "meshes.npz"
    if path.is_file():
        return path
    bodies = [b for b in mech.bodies if b.part is not None]
    meshes = tessellate_many([b.placed_part() for b in bodies], tolerance=0.1, angular=0.2)
    arrays = {}
    for b, (pos, idx, _) in zip(bodies, meshes, strict=True):
        arrays[b.name + "|p"] = np.asarray(pos, np.float64)
        arrays[b.name + "|i"] = np.asarray(idx, np.int64)
    tmp = cache / f".meshes.{os.getpid()}.npz"
    np.savez(tmp, **arrays)
    os.replace(tmp, path)
    return path


def _make(config, design, mech, cache: Path, done: Path, out: Path, jobs: int,
          max_steps: int | None, force: bool, times: dict, t: float) -> GuideReport:
    from spiderpig import keys, workers
    from spiderpig.construction.assembly import assembly_steps, body_of
    from spiderpig.guide import pdf
    from spiderpig.guide.doc import Callout, Doc, PartEntry, PrintBatch, StepEntry
    from spiderpig.guide.render import CONTEXT_FILL, PLACED, Mark, bubbles, render_jobs
    from spiderpig.guide.wiring import draw, wiring_of
    from spiderpig.hardware.bom import _filament_name
    from spiderpig.hardware.mass import filament_density
    from spiderpig.labels import by_body, part_types

    def lap(key: str, since: float) -> float:
        now = time.perf_counter()
        times[key] = round(now - since, 2)
        log.info("%-14s %6.2f s", key, now - since)
        return now

    all_steps = assembly_steps(mech, design)
    chosen = all_steps[:max_steps] if max_steps else all_steps
    mesh_path = _meshes(mech, cache)
    t = lap("meshes", t)
    fab_of = {b.name: b.fab for b in mech.bodies}
    rkey = keys.source_key(RENDER_ROOTS, "render")
    img = cache / "img"
    img.mkdir(exist_ok=True)
    jobs_all: list[dict] = []

    def job(rows: list, views: list[str], size, explode: float = 0.0,
            margin: float = 0.06) -> str:
        spec = {"rows": rows, "views": views, "size": list(size), "explode": explode,
                "margin": margin}
        h = f"{rkey}-{_hash(spec)}"
        spec["out"] = str(img / f"{h}.png")
        if force or not (img / f"{h}.json").is_file():
            jobs_all.append(spec)
        return h

    pictures: dict[int, str] = {}
    for st in chosen:
        if not st.adds:
            continue
        clip = {k: list(v) for k, v in st.clip.items()}
        rows = [(p, CONTEXT_FILL, (0, 0, 0), False, clip.get(p)) for p in st.context]
        rows += [(p, PLACED, (0, 0, 0), False, clip.get(p)) for p in st.places]
        rows += [(p, _fill(fab_of.get(body_of(p))), (0, 0, 0), True, clip.get(p))
                 for p in st.adds]
        lift = EXPLODE if st.stage == "stack" and not st.sub and st.context else 0.0
        pictures[st.number] = job(rows, _views(st), PICTURE, lift)
    bodies = [b for b in mech.bodies if b.part is not None]
    cover = job([(b.name, _fill(b.fab) if b.fab != "laser" else CONTEXT_FILL, (0, 0, 0),
                  False, None) for b in bodies], ["robot"], COVER)
    # the labels: the thumbnails need their reference bodies, so they're drawn after
    # (in the same pool); meanwhile the grouping runs here
    order: dict[str, None] = {}
    for st in all_steps:
        for p in st.adds:
            order.setdefault(body_of(p), None)
    pool = [workers.submit(render_jobs, str(mesh_path), b) for b in _batches(jobs_all, jobs)]
    n_drawn = len(jobs_all)
    jobs_all.clear()
    filament = mech.meta.get("filament", "pla_filament")
    types = part_types(mech, list(order), filament=filament)
    t = lap("labels", t)
    thumbs = {ty.label: job([(ty.ref, _fill(ty.kind), (0, 0, 0), False, None)], ["part"],
                            THUMB, margin=0.08) for ty in types}
    pool += [workers.submit(render_jobs, str(mesh_path), b) for b in _batches(jobs_all, jobs)]
    n_drawn += len(jobs_all)
    for f in pool:
        for r in f.result():
            Path(r["out"]).with_suffix(".json").write_text(json.dumps(r))
    t = lap("render", t)
    # the label bubbles, the wiring diagram, the document
    of = by_body(types)
    work = Path(tempfile.mkdtemp(prefix=".work-", dir=cache))
    entries: list[StepEntry] = []
    for st in chosen:
        path = work / f"step_{st.number:03d}.png"
        if st.number in pictures:
            h = pictures[st.number]
            meta = json.loads((img / f"{h}.json").read_text())
            best: dict[str, Mark] = {}
            for pid, (x, y, n) in meta["marks"].items():
                ty = of.get(body_of(pid))
                if ty is not None and (ty.label not in best or n > best[ty.label].n):
                    best[ty.label] = Mark(x, y, n)
            from PIL import Image

            with Image.open(img / f"{h}.png") as im:
                bubbles(im.convert("RGB"), sorted(best.items()))[0].save(path,
                                                                          compress_level=6)
        else:   # the wiring step
            nodes, links, left = wiring_of(mech, {n: t_.label for n, t_ in of.items()})
            draw(nodes, links, left, path, PICTURE)
        counts: dict[str, int] = {}
        for n in st.counted:
            if n in of:
                counts[of[n].label] = counts.get(of[n].label, 0) + 1
        typ = {ty.label: ty for ty in types}
        callouts = [Callout(lab, q, typ[lab].name, img / f"{thumbs[lab]}.png")
                    for lab, q in sorted(counts.items(), key=lambda kv: ("PCH".index(
                        kv[0][0]), kv[0]))]
        entries.append(StepEntry(st.number, st.title, st.stage_title, st.text, path,
                                 callouts, st.sub))
    by_name = {b.name: b for b in mech.bodies}
    prints = [PrintBatch(ty.label, ty.file or "", ty.qty,
                         _filament_name(ty.filament) if ty.filament else "",
                         round(by_name[ty.ref].part.volume / 1000                 # type: ignore[union-attr]
                               * filament_density(ty.filament), 1))
              for ty in types if ty.kind == "printed"]
    parts = [PartEntry(ty.label, ty.kind, ty.name, ty.qty, img / f"{thumbs[ty.label]}.png",
                       ty.file, ty.detail)
             for ty in sorted(types, key=lambda t: ("PCH".index(t.label[0]), t.label))]
    title = (f"{config.linkage} {config.module} {'robot' if config.robot else 'side'}")
    doc = Doc("Assembly guide",
              [title, f"{config.servo} servo, {config.pin} pins, {config.pillar} pillars, "
                      f"{config.crank} crank",
               f"{len(all_steps)} steps, {len(types)} part types, "
               f"{sum(ty.qty for ty in types)} parts"
               + (f" (the first {len(chosen)} steps)" if max_steps else "")],
              img / f"{cover}.png", parts, prints, entries,
              f"spiderpig guide: {title}. Generated; the labels match the print files.")
    pages = pdf.write(work / "ASSEMBLY.pdf", doc)
    (work / "ASSEMBLY.md").write_text(markdown(title, all_steps[:len(chosen)], types))
    (work / "guide.json").write_text(json.dumps({"pages": pages, "steps": len(chosen)}))
    t = lap("pdf", t)
    for name in ("ASSEMBLY.pdf", "ASSEMBLY.md"):
        shutil.copyfile(work / name, out / name)
    if done.exists():
        shutil.rmtree(done, ignore_errors=True)
    try:
        os.rename(work, done)
    except OSError:     # another run published it first
        shutil.rmtree(work, ignore_errors=True)
    return GuideReport(out / "ASSEMBLY.pdf", pages, len(chosen), False, n_drawn, times)


def _batches(jobs_all: list[dict], n: int) -> list[list[dict]]:
    """``jobs_all`` dealt to ``n`` workers, the biggest first (by rows)."""
    order = sorted(jobs_all, key=lambda j: -len(j["rows"]))
    out: list[list[dict]] = [[] for _ in range(max(1, n))]
    load = [0] * len(out)
    for j in order:
        k = load.index(min(load))
        out[k].append(j)
        load[k] += len(j["rows"]) + 20
    return [b for b in out if b]


def markdown(title: str, steps, types) -> str:
    """The steps as text (``ASSEMBLY.md``): each step's title, sentences and parts."""
    from spiderpig.labels import by_body

    of = by_body(types)
    lines = [f"# Assembly: {title}", "",
             "Generated by `spiderpig guide` from the constructions' assembly hooks and "
             "the robot's order; the pictures are in ASSEMBLY.pdf.", ""]
    stage = None
    for st in steps:
        if st.stage_title != stage:
            stage = st.stage_title
            lines += [f"## {stage}", ""]
        lines.append(f"**{st.number}. {st.title}**" + (" (bench sub-assembly)" if st.sub
                                                        else ""))
        lines += [f"- {t}" for t in st.text]
        counts: dict[str, int] = {}
        for n in st.counted:
            if n in of:
                counts[of[n].label] = counts.get(of[n].label, 0) + 1
        if counts:
            lines.append("- Parts: " + ", ".join(f"{lab} x {q}" for lab, q in
                                                 sorted(counts.items())))
        lines.append("")
    lines += ["## Parts", "", "| label | qty | part | print file |", "|---|---|---|---|"]
    lines += [f"| {t.label} | {t.qty} | {t.name} | {t.file or ''} |" for t in types]
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    from spiderpig.config import add_build_args, add_design_args, config_from_args
    from spiderpig.store import Store

    p = argparse.ArgumentParser(prog="spiderpig guide", description=(
        "The assembly guide: ASSEMBLY.pdf (numbered steps with pictures, the parts with "
        "their labels, print batches, bag labels) and ASSEMBLY.md, into --out."))
    add_design_args(p)
    add_build_args(p)
    p.add_argument("--out", type=Path, default=Path("build"), help="the folder (build)")
    p.add_argument("--store", type=Path, default=None,
                   help="the design store (default: $SPIDERPIG_STORE or ./.spiderpig)")
    p.add_argument("--jobs", type=int, default=4, help="picture worker processes (4)")
    p.add_argument("--steps", type=int, default=None, help="only the first N steps")
    p.add_argument("--force", action="store_true", help="draw everything again")
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    logging.getLogger("fontTools").setLevel(logging.WARNING)
    config = config_from_args(args)
    store = Store.of(args.store) if args.store else Store.default()
    rep = build_guide(config, store, args.out, jobs=args.jobs, max_steps=args.steps,
                      force=args.force)
    how = "from the cache" if rep.cached else f"{rep.drawn} pictures drawn"
    print(f"wrote {rep.pdf} and ASSEMBLY.md: {rep.pages} pages, {rep.steps} steps ({how}, "
          f"{rep.seconds.get('total', 0):.1f} s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
