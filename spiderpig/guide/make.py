"""``spiderpig guide``: the assembly guide's PDF for a design (prototype).

Plans through the store, fabricates (the fabrication cache), meshes every body once
(:func:`mesh.tessellate_many`), derives the steps (:mod:`guide.model`) and the part labels
(:mod:`guide.labels`), draws each step (:mod:`guide.render`) and lays out the PDF
(:mod:`guide.pdf`). ``--steps N`` stops after N steps (a quick look).
"""

from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path

import numpy as np

from spiderpig.guide.model import Step, layers_of, side_of, steps
from spiderpig.guide.render import (
    CONTEXT_FILL,
    HIGHLIGHT,
    Item,
    View,
    render,
    render_batch,
    visible,
)

log = logging.getLogger("guide")

PLACED = (250, 196, 140)        # a sub-assembly put on in this step: a lighter highlight
VIEWS = {
    "bench_L": {"eye": (0.75, 0.55, 0.85), "up": (0, 0, 1), "light": (0.3, 0.5, 1.0)},
    "unit_L": {"eye": (0.75, 0.55, 0.85), "up": (0, 0, 1), "light": (0.3, 0.5, 1.0)},
    "bench_R": {"eye": (0.75, 0.55, -0.85), "up": (0, 0, -1), "light": (0.3, 0.5, -1.0)},
    "unit_R": {"eye": (0.75, 0.55, -0.85), "up": (0, 0, -1), "light": (0.3, 0.5, -1.0)},
    "robot": {"eye": (1.0, 0.8, -1.2), "up": (0, 1, 0), "light": (0.4, 1.0, -0.6)},
    "unit_L-": {"eye": (0.75, 0.55, -0.85), "up": (0, 0, -1), "light": (0.3, 0.5, -1.0)},
    "unit_R-": {"eye": (0.75, 0.55, 0.85), "up": (0, 0, 1), "light": (0.3, 0.5, 1.0)},
    "robot-": {"eye": (1.0, 0.8, 1.2), "up": (0, 1, 0), "light": (0.4, 1.0, 0.6)},
    "robot^": {"eye": (0.7, 1.4, -0.5), "up": (0, 1, 0), "light": (0.4, 1.0, -0.6)},
    "part": {"eye": (0.6, 0.8, 1.0), "up": (0, 1, 0), "light": (0.3, 1.0, 0.8)},
}


# the cameras a step may choose from (the bench views stay put: a side reads the same way
# from its first layer to its last)
CHOICES = {"unit_L": ["unit_L", "unit_L-"], "unit_R": ["unit_R", "unit_R-"],
           "robot": ["robot", "robot-", "robot^"]}


def choose_view(st: Step, mesh: dict) -> str:
    """The candidate camera that shows most of what the step adds (first on a tie)."""
    options = CHOICES.get(st.view, [st.view])
    if len(options) == 1:
        return st.view
    items = [Item(n, *mesh[n]) for n in st.context + st.places + st.adds]
    want = set(st.adds) | set(st.places)
    scores = [visible(items, View(**VIEWS[o]), want) for o in options]
    return options[scores.index(max(scores))]


def step_items(st: Step, mesh: dict, by_body: dict) -> tuple[list[Item], View]:
    v = VIEWS[choose_view(st, mesh)]
    up = np.asarray(v["up"], float)
    lift = tuple(up * st.explode)  # the adds drawn lifted
    items = [Item(n, *mesh[n]) for n in st.context]
    items += [Item(n, *mesh[n], fill=PLACED) for n in st.places]
    labelled: set[str] = set()
    for n in st.adds:
        t = by_body.get(n)
        lab = None
        if t is not None and t.label not in labelled:
            labelled.add(t.label)
            lab = t.label
        items.append(Item(n, *mesh[n], fill=HIGHLIGHT, offset=lift if st.context else
                          (0, 0, 0), label=lab))
    arrows = []
    if st.explode and st.context and st.adds:
        c = np.concatenate([mesh[n][0] for n in st.adds]).mean(0)
        arrows = [(c + up * st.explode * 0.9, c + up * 1.0)]
    return items, View(eye=v["eye"], up=v["up"], light=v["light"], size=(1400, 1050),
                       arrows=arrows)


def _spec(items: list[Item], view: View) -> tuple:
    """A picture without its meshes (they cross to a worker once, by name)."""
    return ([(it.name, it.fill, it.offset, it.label) for it in items], view)


def main(argv=None) -> int:
    from spiderpig import api, fabcache
    from spiderpig.config import add_build_args, add_design_args, config_from_args
    from spiderpig.fabricate import design_side, fabricate, template_for
    from spiderpig.guide import labels, pdf
    from spiderpig.mesh import tessellate_many
    from spiderpig.store import Store

    p = argparse.ArgumentParser(description=__doc__)
    add_design_args(p)
    add_build_args(p)
    p.add_argument("--out", type=Path, default=Path("build"))
    p.add_argument("--store", type=Path, default=None)
    p.add_argument("--steps", type=int, default=None, help="only the first N steps")
    p.add_argument("--jobs", type=int, default=4, help="render worker processes")
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    logging.getLogger("fontTools").setLevel(logging.WARNING)
    config = config_from_args(args)
    t0 = time.perf_counter()
    times: dict[str, float] = {}

    def lap(key: str, since: float) -> float:
        now = time.perf_counter()
        times[key] = round(now - since, 2)
        log.info("%-14s %6.2f s", key, now - since)
        return now

    store = Store.of(args.store) if args.store else Store.default()
    tmpl = template_for(config)
    api.plan_config(config, store)
    design = design_side(tmpl, config)
    with fabcache.serving(store):
        mech = fabricate(tmpl, config, 1.0)
    t = lap("fabricate", t0)
    bodies = [b for b in mech.bodies if b.part is not None]
    meshes = tessellate_many([b.placed_part() for b in bodies], tolerance=0.1, angular=0.2)
    mesh = {b.name: (m[0].astype(np.float64), m[1]) for b, m in
            zip(bodies, meshes, strict=True)}
    t = lap("tessellate", t)
    rows = [{"name": b.name, "z": [float(mesh[b.name][0][:, 2].min()),
                                 float(mesh[b.name][0][:, 2].max())],
                 "rigid_with": b.rigid_with} for b in bodies]
    layers = layers_of(design.plan)
    all_steps = steps(rows, layers, mech.meta["mid_plane"])
    by_body, types = labels.part_types(mech, all_steps)
    for st in all_steps:
        st.callouts = labels.callouts(by_body, st.adds)
    t = lap("steps+labels", t)
    chosen = all_steps[:args.steps] if args.steps else all_steps
    from PIL import Image

    from spiderpig import workers

    specs = [(st.number, _spec(*step_items(st, mesh, by_body))) for st in chosen]
    t = lap("cameras", t)
    folder = args.out / "guide"
    folder.mkdir(parents=True, exist_ok=True)
    batches = [specs[i::args.jobs] for i in range(args.jobs) if specs[i::args.jobs]]
    futures = [workers.submit(render_batch, mesh, b, str(folder)) for b in batches]
    for f in futures:
        f.result()
    images = {st.number: Image.open(folder / f"step_{st.number:03d}.png").convert("RGB")
              for st in chosen}
    t = lap("render steps", t)
    thumbs = {}
    for ty in types:
        v = VIEWS["part"]
        pos, tri = mesh[ty.ref]
        fill = HIGHLIGHT if ty.kind != "purchased" else CONTEXT_FILL
        thumbs[ty.label] = render([Item(ty.ref, pos - pos.mean(0), tri, fill=fill)],
                                  View(eye=v["eye"], up=v["up"], light=v["light"],
                                       size=(300, 300), margin=0.08))
    t = lap("thumbnails", t)
    cover = render([Item(n, *mesh[n], fill=_cover_fill(b)) for n, b in
                    ((b.name, b) for b in bodies)],
                   View(**VIEWS["robot"], size=(1600, 1200)))
    t = lap("cover", t)
    args.out.mkdir(parents=True, exist_ok=True)
    out = args.out / "ASSEMBLY.pdf"
    m = mech.meta
    n_pages = pdf.write(
        out, title="Assembly guide",
        subtitle=[f"{config.linkage} {config.module} robot, {config.servo} servos",
                  f"{config.pin} pins, {config.pillar} pillars, {config.crank} crank",
                  f"{len(all_steps)} steps, {len(types)} part types, "
                  f"{sum(ty.qty for ty in types)} parts"
                  + ("" if args.steps is None else f" (first {len(chosen)} steps drawn)"),
                  f"{m.get('layers', '?')} layers per side"],
        cover=cover, types=types, thumbs=thumbs, steps=chosen, images=images,
        footer=f"spiderpig {config.linkage} {config.module}; generated, do not edit")
    lap("pdf", t)
    lap("total", t0)
    print(f"wrote {out}: {n_pages} pages, {len(chosen)} steps; timings {times}")
    return 0


def _cover_fill(b) -> tuple[int, int, int]:
    if side_of(b.name) is None and b.name.startswith("deck"):
        return (140, 170, 210)
    return {"laser": (205, 205, 212), "printed": HIGHLIGHT}.get(b.fab or "", (120, 120, 128))


if __name__ == "__main__":
    raise SystemExit(main())
