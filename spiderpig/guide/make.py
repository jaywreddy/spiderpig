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
GUIDE_ROOTS = ("spiderpig.guide.make:build_guide", "spiderpig.cradle_pack:pack_kit",
               "spiderpig.cradle_pack:box_steps")
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
    from spiderpig import fabcache, keys
    from spiderpig.fabricate import design_side, fabricate, template_for
    from spiderpig.stages.planning import plan_config

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
            plan_config(config, store)
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
        try:
            report = _make(config, design, mech, cache, done, out, jobs, max_steps, force,
                           times, t, store)
        except BaseException:       # no half-made guide left behind
            for d in cache.glob(f".work-{os.getpid()}-*"):
                shutil.rmtree(d, ignore_errors=True)
            raise
        _prune(cache, done)
    lap("total", t0)
    return report


def _prune(cache: Path, done: Path) -> None:
    """Drop what this code no longer reads: the guides of other code keys, and the
    pictures of another renderer (their names start with its key)."""
    from spiderpig import keys

    base = done.name.split("-first")[0]         # (a guide of the first N steps beside it)
    for d in cache.glob("pdf-*"):
        if d.name != base and not d.name.startswith(base + "-first"):
            shutil.rmtree(d, ignore_errors=True)
    rkey = keys.source_key(RENDER_ROOTS, "render")
    for f in (cache / "img").glob("*"):
        if not f.name.startswith(rkey + "-"):
            f.unlink(missing_ok=True)


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
          max_steps: int | None, force: bool, times: dict, t: float,
          store=None) -> GuideReport:
    from spiderpig import keys, workers
    from spiderpig.construction.assembly import assembly_steps, body_of
    from spiderpig.guide import pdf
    from spiderpig.guide.doc import Callout, Doc, PartEntry, PrintBatch, StepEntry
    from spiderpig.guide.render import CONTEXT_FILL, PLACED, Mark, bubbles, render_jobs
    from spiderpig.guide.wiring import draw, wiring_of
    from spiderpig.hardware.bom import _filament_name
    from spiderpig.hardware.mass import filament_density
    from spiderpig.labels import (
        consumables,
        part_types,
        shim_sentences,
        step_counts,
        tag_labels,
    )

    def lap(key: str, since: float) -> float:
        now = time.perf_counter()
        times[key] = round(now - since, 2)
        log.info("%-14s %6.2f s", key, now - since)
        return now

    kit = box_kit(config, mech, store)
    all_steps = with_box_steps(assembly_steps(mech, design), config, mech, store, kit=kit)
    chosen = all_steps[:max_steps] if max_steps else all_steps
    mesh_path = _meshes(mech, cache)
    t = lap("meshes", t)
    fab_of = {b.name: b.fab for b in mech.bodies}
    rkey = keys.source_key(RENDER_ROOTS, "render")
    img = cache / "img"
    img.mkdir(exist_ok=True)
    jobs_all: list[dict] = []

    def job(rows: list, views: list[str], size, explode: float = 0.0,
            margin: float = 0.06, extra: str | None = None) -> str:
        spec = {"rows": rows, "views": views, "size": list(size), "explode": explode,
                "margin": margin}
        if extra:
            spec["extra"] = extra
        h = f"{rkey}-{_hash(spec)}"
        spec["out"] = str(img / f"{h}.png")
        if force or not (img / f"{h}.json").is_file():
            jobs_all.append(spec)
        return h

    pictures: dict[int, str] = {}
    placed: list[str] = []
    wanted = {s.number for s in chosen}
    for st in all_steps:
        placed += [p for p in st.adds if p not in placed]
        if st.number not in wanted:
            continue
        if st.scene is not None and st.scene.pose == "box" and kit:     # (the pack's)
            from spiderpig.guide import scenes

            got, extra = scenes.box_rows(st.scene, mech, config, kit,
                                         lambda n: _fill(fab_of.get(n)), CONTEXT_FILL,
                                         SPACER_FILL)
            arrays = {f"{n}|{k}": a for n, (q, i) in extra.items()
                      for k, a in (("p", np.asarray(q, np.float64)),
                                   ("i", np.asarray(i, np.int64)))}
            digest = hashlib.sha256()
            for k in sorted(arrays):            # (its name is its meshes': a job's key)
                digest.update(k.encode() + arrays[k].tobytes())
            path = img / f"extra-{digest.hexdigest()[:20]}.npz"
            if not path.is_file():
                np.savez(path, **arrays)
            pictures[st.number] = job([(p, f, (0, 0, 0), mark, None, m) for p, f, mark, m in got],
                                      list(st.scene.views), PICTURE, extra=str(path))
            continue
        if st.scene is not None:            # posed its own way (a cradle's use, its trip)
            from spiderpig.guide import scenes

            got = scenes.rows(st.scene, mech, config, list(placed), st.adds,
                              lambda n: _fill(fab_of.get(n)), CONTEXT_FILL, PLACED)
            if got:
                pictures[st.number] = job([(p, f, (0, 0, 0), mark, None, m)
                                           for p, f, mark, m in got],
                                          list(st.scene.views), PICTURE)
                continue
        if not st.adds and not st.places:     # (the wiring step: its diagram)
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
                            THUMB, margin=0.08) for ty in types if ty.ref}
    for ty in types:                # bought for no body (a resistor): a drawn card instead
        if not ty.ref:
            thumbs[ty.label] = _card(img, ty.label, ty.name)
    pool += [workers.submit(render_jobs, str(mesh_path), b) for b in _batches(jobs_all, jobs)]
    n_drawn += len(jobs_all)
    for f in pool:
        for r in f.result():
            Path(r["out"]).with_suffix(".json").write_text(json.dumps(r))
    t = lap("render", t)
    # the label bubbles, the wiring diagram, the document
    tags = tag_labels(types, mech)
    counts_of = step_counts(all_steps, types, mech)
    typ = {ty.label: ty for ty in types}
    rank = {ty.label: i for i, ty in enumerate(types)}     # (the kinds, then first use)
    work = Path(tempfile.mkdtemp(prefix=f".work-{os.getpid()}-", dir=cache))
    entries: list[StepEntry] = []
    for st in chosen:
        path = work / f"step_{st.number:03d}.png"
        if st.number in pictures:
            h = pictures[st.number]
            meta = json.loads((img / f"{h}.json").read_text())
            best: dict[str, Mark] = {}
            mine = set(st.adds)         # (a scene's highlights are no new parts: no tags)
            boxed = st.scene is not None and st.scene.pose == "box"
            for pid, (x, y, n) in meta["marks"].items():
                if boxed and pid.startswith("spacer_"):     # (a pack's spacer: its number)
                    lab = spacer_tag(pid, kit)
                elif st.scene is not None and pid not in mine:
                    continue
                else:
                    lab = tags.get(body_of(pid))
                if lab is not None and (lab not in best or n > best[lab].n):
                    best[lab] = Mark(x, y, n)
            # a part nothing shows of (under another): its tag says so
            best = {(lab if m.n else f"{lab} (under)"): m for lab, m in best.items()}
            from PIL import Image

            with Image.open(img / f"{h}.png") as im:
                bubbles(im.convert("RGB"), sorted(best.items()))[0].save(path,
                                                                          compress_level=6)
        elif st.stage == "wiring":
            nodes, links, left = wiring_of(mech, tags)
            draw(nodes, links, left, path, PICTURE)
        else:   # (a step with nothing of its own to draw: the cover's picture)
            shutil.copyfile(img / f"{cover}.png", path)
        counts = counts_of[st.number - 1]
        callouts = [Callout(lab, round(counts[lab]), typ[lab].name, img / f"{thumbs[lab]}.png")
                    for lab in sorted(counts, key=rank.__getitem__)]
        text = st.text + [s for s in shim_sentences(st.counted, mech) if s not in st.text]
        entries.append(StepEntry(st.number, st.title, st.stage_title, text, path,
                                 callouts, st.sub))
    by_name = {b.name: b for b in mech.bodies}
    prints = [PrintBatch(ty.label, ty.file or "", ty.qty,
                         _filament_name(ty.filament) if ty.filament else "",
                         round(by_name[ty.ref].part.volume / 1000                 # type: ignore[union-attr]
                               * filament_density(ty.filament), 1))
              for ty in types if ty.kind == "printed"]
    prints += spacer_prints(kit)
    parts = [PartEntry(ty.label, ty.kind, ty.name, ty.qty, img / f"{thumbs[ty.label]}.png",
                       ty.file, ty.detail) for ty in types]
    title = (f"{config.linkage} {config.module} {'robot' if config.robot else 'side'}")
    drive = (f"{config.servo} servo, {config.pin} pins, {config.pillar} pillars, "
             f"{config.crank} crank")
    if config.lk.product:           # a decanting cradle: no servo, both sides
        title = f"{config.lk.name} (decanting cradle)"
        drive = (f"hand crank (NRV050 worm, 30:1), {config.pin} pins, {config.pillar} "
                 "pillars")
    doc = Doc("Assembly guide",
              [title, drive,
               f"{len(all_steps)} steps, {len(types)} part types, "
               f"{sum(ty.qty for ty in types)} parts"
               + (f" (the first {len(chosen)} steps)" if max_steps else "")],
              img / f"{cover}.png", parts, prints, entries,
              f"spiderpig guide: {title}. Generated; the labels match the print files.",
              supplies=[_supply(k) for k in sorted(consumables(mech))], tools=_tools(mech))
    pages = pdf.write(work / "ASSEMBLY.pdf", doc)
    (work / "ASSEMBLY.md").write_text(markdown(title, all_steps[:len(chosen)], types,
                                               counts_of, mech, doc.tools))
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


def box_kit(config, mech, store) -> dict | None:
    """The pack payload (``cradle_pack.pack_kit``) of a cradle that packs as modules
    (``construction.cradle.PACK_POSES``), whose box steps the guide draws; else None."""
    from spiderpig.construction import cradle as cr

    if not mech.meta.get("cradle") or config.linkage not in cr.PACK_POSES:
        return None
    from spiderpig.cradle_pack import pack_kit

    return pack_kit(config, store)


SPACER_FILL = (40, 150, 140)
"""A pack step's own spacers in its picture (teal: apart from the pieces' highlight)."""


def spacer_tag(key: str, kit: dict | None) -> str:
    """A pack spacer's bubble in a box step's picture: ``S7`` (printed), ``F1`` (a filler)."""
    from spiderpig.cradle_pack import is_filler

    d = next((d for d in (kit or {}).get("spacers") or [] if d["key"] == key), {})
    return ("F" if is_filler(d) else "S") + key.removeprefix("spacer_").lstrip("0")


def spacer_prints(kit: dict | None) -> list:
    """The pack's printed spacers in the guide's print list (packing review 2's M2): one
    batch each, ``pack_spacer_NN.stl`` (as ``build`` writes it), PETG, its grams as
    printed (``cradle_pack.spacer_grams``)."""
    from spiderpig.cradle_pack import SPACER_FILAMENT, is_filler
    from spiderpig.guide.doc import PrintBatch
    from spiderpig.hardware.bom import _filament_name

    return [PrintBatch(d["label"], f"pack_{d['key']}.stl", 1, _filament_name(SPACER_FILAMENT),
                       round(float(d.get("mass_kg", 0.0)) * 1000.0, 1))
            for d in (kit or {}).get("spacers") or [] if not is_filler(d)]


def with_box_steps(steps: list, config, mech, store, kit: dict | None = None) -> list:
    """A cradle that packs as modules (``construction.cradle.PACK_POSES``): its steps with
    the box's (``cradle_pack.box_steps`` of ``cradle_pack.pack_kit``, or ``kit``: each layer
    of pieces and the printed spacers into the box after taking it apart, out of it before
    putting it back together), numbered on, each with a picture of the pack as it is then
    (a ``Scene`` posed ``"box"``: what is in, this step's own highlighted:
    ``guide.scenes.box_rows``); any other design's as they are."""
    from dataclasses import replace

    from spiderpig.construction import cradle as cr

    if not mech.meta.get("cradle") or config.linkage not in cr.PACK_POSES:
        return steps
    from spiderpig.construction.assembly import Scene, Step
    from spiderpig.cradle_pack import box_layers, box_steps, pack_kit

    kit = kit or pack_kit(config, store)
    if not kit:
        return steps
    into, out = box_steps(kit)
    apart = [i for i, s in enumerate(steps) if s.title.startswith("Taking it apart")]
    back = [i for i, s in enumerate(steps) if s.title.startswith("Back together")]
    if not apart or not back:
        return steps
    # each step's own keys (the pieces and spacers it puts in), in order: box_steps' layers
    layers = [[d["key"] for _, d in layer] for layer in box_layers(kit)]
    views = ("robot^", "robot")

    def scene(shown: list[str], mine: list[str]) -> Scene:
        return Scene("box", show=tuple(shown), marks=tuple(mine), views=views)

    shown: list[str] = []
    scenes_in = []
    for keys in layers:
        shown = shown + keys
        scenes_in.append(scene(shown, keys))
    scenes_out = []
    for keys in reversed(layers):
        scenes_out.append(scene(shown, keys))
        shown = [k for k in shown if k not in keys]

    def made(rows, head, like, pics) -> list:
        n = len(rows)
        return [Step(0, f"{head} {i}/{n}: {title}", like.stage, like.stage_title, None, [],
                     [], [], list(items), like.where,
                     scene=pics[i - 1] if i - 1 < len(pics) else None)
                for i, (title, items) in enumerate(rows, 1)]

    a, b = apart[-1] + 1, back[0]
    got = (steps[:a] + made(into, "Into its box", steps[a - 1], scenes_in) + steps[a:b]
           + made(out, "Out of its box", steps[b], scenes_out) + steps[b:])
    return [replace(s, number=i) for i, s in enumerate(got, 1)]


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


def _supply(key: str) -> str:
    from spiderpig.hardware import catalog

    try:
        return catalog.get(key).name
    except KeyError:
        return key


def _card(img: Path, label: str, name: str) -> str:
    """A drawn thumbnail for a part with no model (a resistor, a pigtail): its label and
    name on a card. Its file's name, as a picture job's (a hash)."""
    from PIL import Image, ImageDraw

    from spiderpig.guide.render import BOUGHT_FILL, INK, font

    h = "card-" + _hash([label, name])
    path = img / f"{h}.png"
    if not path.is_file():
        im = Image.new("RGB", THUMB, (255, 255, 255))
        d = ImageDraw.Draw(im)
        d.rounded_rectangle((30, 70, THUMB[0] - 30, THUMB[1] - 70), 18, fill=BOUGHT_FILL,
                            outline=INK, width=3)
        d.text((THUMB[0] / 2, THUMB[1] / 2 - 14), label, fill=INK, font=font(26),
               anchor="mm")
        d.text((THUMB[0] / 2, THUMB[1] / 2 + 22), name.split(",")[0][:28], fill=INK,
               font=font(15, bold=False), anchor="mm")
        im.save(path, compress_level=6)
    return h


def _tools(mech) -> list[str]:
    """The tools a design's steps name, grouped (a decanting cradle's:
    ``construction.cradle.TOOLS``); none listed for a walker."""
    if not mech.meta.get("cradle"):
        return []
    from spiderpig.construction.cradle import TOOLS

    return list(TOOLS)


def markdown(title: str, steps, types, counts_of, mech, tools=()) -> str:
    """The steps as text (``ASSEMBLY.md``): each step's title, sentences and parts (and the
    tools, when the design lists them)."""
    from spiderpig.labels import shim_sentences

    lines = [f"# Assembly: {title}", "",
             "Generated by `spiderpig guide` from the constructions' assembly hooks and "
             "the robot's order; the pictures are in ASSEMBLY.pdf.", ""]
    if tools:
        lines += ["## Tools", "", *(f"- {t}" for t in tools), ""]
    stage = None
    for st in steps:
        if st.stage_title != stage:
            stage = st.stage_title
            lines += [f"## {stage}", ""]
        lines.append(f"**{st.number}. {st.title}**" + (" (bench sub-assembly)" if st.sub
                                                        else ""))
        lines += [f"- {t}" for t in st.text + shim_sentences(st.counted, mech)]
        counts = counts_of[st.number - 1]
        if counts:
            lines.append("- Parts: " + ", ".join(f"{lab} x {round(q)}" for lab, q in
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
