# The assembly guide: a feasibility study and a prototype (2026-10-08)

**Goal.** Every design gets an `ASSEMBLY.pdf`, produced by a build pipeline with no agent
in the loop. It has numbered steps; each step has a stylised picture (the parts added in
that step highlighted, the earlier assembly greyed, offsets and arrows), a list of the
parts it uses with quantities and identifiers, and text.

**Verdict: feasible.** The prototype on branch `guide-feasibility` (`spiderpig guide`,
`spiderpig/guide/`) writes a 48-page guide for the default Strider double robot: a cover,
four parts pages and 43 steps. It runs end to end in about 100 s cold. Two runs produce
byte-identical output. It adds no new dependency. To open it:
`~/.cache/spiderpig/guide-prototype/ASSEMBLY.pdf`.

Recommended stack:

- **Pictures:** a numpy software renderer (option b): flat toon shading and ink outlines,
  run in `workers.submit` processes.
- **Layout:** reportlab. It is the one new dependency and is about 5x faster than
  matplotlib, which the prototype uses.
- **Steps:** a step model generated from the layer plan. Each construction declares a small
  assembly hook for the parts the plan can't order on its own.

## 1. The step model

### What the design already says

| source | what it gives the guide |
|---|---|
| `construction.robot.ASSEMBLY` (prose) | The robot-level order: each side's leg stack, the crank, the inner-plate unit, the unit onto the stack, the centre plates and studs, the right side, the deck and the cables. It also carries the know-how: threadlocker, 0.8 N·m, "from the cap side", which side is built on the robot. |
| `StackPlan` (`plan.z(k)`, `plan.describe()`) | Each layer's z interval and contents, the same for both sides. A body's bottom face gives its layer: the gap under a layer goes with that layer. |
| body names and `rigid_with` | Each pillar (*pillar_J2_leg0_ring5*, *_gap3_spacer*, *_standoff0*, *_screw0*/*_screw13*), each Chicago pin (*pin_J4_leg0_**, host link through `rigid_with`), each crank chain (*crank_plate<k>* webs, *crank_pin_J1_leg0* hex with its *_lo*/*_hi* screw, washer and collar, sleeve, rider rings), the frame ties (*tie_**), the centre plates, the deck (*deck_**). |
| `fab`, `bom_key`, `bom.group_made` | Part types: printed and laser shape groups, bought items by catalog key. These give the identifiers and quantities. |
| `mech.meta` | Chassis facts: centre plates, rear screws, ties, deck fitted. |

### Data structures (`spiderpig/guide/model.py`)

```python
@dataclass(frozen=True)
class Op:                 # what a construction says about putting its parts on
    stage: tuple          # robot-level phase: ("side", "L", 1), ("unit", "L", 0), ...
    key: tuple            # order within the stage, e.g. (layer, 0) for a bench unit
    bodies: tuple[str, ...]
    title: str
    text: str = ""
    sub: bool = False     # a sub-assembly: built on the bench, drawn alone, placed next step

@dataclass
class Step:
    number: int; title: str
    adds: list[str]       # bodies added here: every body of the robot exactly once
    context: list[str]    # what the picture shows already in place (greyed)
    places: list[str]     # a sub-assembly put on here (light highlight)
    text: list[str]; view: str; callouts: list[Callout]; sub: bool; explode: float

@dataclass
class Callout:
    label: str; qty: int; name: str; ref: str    # P07 x 2 "printed ring 9.3 x 9.3 x 3.0 mm"
```

`steps(rows, layers, mid)` collects the `Op`s of each side and of the robot, checks that no
body appears twice, sorts the ops by a stage order, and assigns each step its context:

- a bench step shows its own side's stack so far;
- a unit step shows the unit so far;
- a robot step shows everything placed so far;
- a sub-assembly is drawn on its own.

### What is generated, and what is templated

**Generated from the data, with no per-design work:**

- Each side's leg stack, bottom up, one step per layer. Each body not claimed by a rule
  joins the step of the layer its bottom face sits in.
- Chicago pins go in with their host link.
- The crank's web units: each web except the hub plate, with the hex standoff that stands
  on it and that hex's lower screw, washer and collar. The lowest web also takes the
  journal stub.
- The part types, quantities and labels.
- The camera: bench views are fixed per side; unit and robot steps pick, among 2-3
  cameras, the one where the added parts show the most pixels (`make.choose_view`).
- The exploded offset along the stack axis, and the arrow.
- The coverage check: every body exactly once.

**Templated by hand, once per construction:**

- The sentences: torques, threadlocker, which screw goes in from which side.
- The sub-assembly boundaries: the crank web unit, the inner-plate unit, the deck
  electronics.
- The ordering exceptions:
  - the hub plate goes with the servo, not with its layer;
  - the pillars' top screws go in at the join;
  - the right side's tie chains go onto the studs before its inner plate (`ASSEMBLY` step 6).
- The robot-level stage order.

In the prototype these live in one rule table over body names (`model.side_ops`,
`model.robot_ops`). The maintainable form gives each group an assembly hook:

```python
class Group:                                   # construction.base.Group
    def assembly(self, build: Build, done: Realized) -> list[Op]:
        return []      # default: the group's bodies fall to the per-layer rule, generic text
```

- `BoltCrank` would return the web units and the hub plate's placement.
- `standoff` would return the column step and the top screws at the join.
- `chicago` would return the pin with its host.
- The drive group would return the servo, the horn and the hub plate.
- The chassis, the deck and the ties would return their own steps.

`construction.robot.ASSEMBLY` would become structured: a list of stages, each with a title
template filled from the design (`{n_centre_plates}`, `{tie_screw}`). The prose would then
be generated from the same data. A new construction that has no hook still gives a
complete guide, through the per-layer rule with generic text. The coverage test keeps it
honest.

### Gaps the prototype leaves

- The right side is built in the left side's order: its unit on the bench, then
  "unit onto the leg stack". `ASSEMBLY` builds the right unit on the robot, and the body is
  turned over onto the right stack. That needs the stage template.
- A Chicago pin is one body (barrel and screw), so the guide can't say "barrel now, screw
  once the link above is on".
- Label bubbles aren't placed by a solver, so they can overlap in busy steps.
- The parts box shows at most 12 types.
- The bus cables are text only.

## 2. Rendering, headless and deterministic

All measurements: 2026-10-08, 20-core shared box under load 3-6, no GPU, the default design.

| option | look | time per image | same bytes on rerun | dependencies |
|---|---|---|---|---|
| **(b) numpy z-buffer** (`guide/render.py`) | flat banded tones, ink silhouettes and creases from the id, depth and normal buffers, colour highlight, 2x supersampling | 1.4-1.7 s for the sample step (12k triangles, 1200x900); 2.3-2.7 s for a whole side (121k triangles, 1400x1050). The 43 steps take 39 s on 4 workers | yes: PNGs and PDF identical across runs and processes (the z-buffer is an integer `maximum.at`) | none new (numpy, Pillow) |
| (a) OCCT hidden lines (build123d *project_to_viewport* + *ExportSVG*) | exact vector line art, like a technical drawing or IKEA; no fills, so a highlight is only a stroke colour | 0.13 s for the sample step, 2.2 s for a whole side (+0.4 s to write the SVG) | yes | none new |
| (c) three.js in Playwright Chromium | the viewer's look; 1-pixel crease lines; noisy on the fine horn mesh | 5.7-6.5 s cold (launch, 10.8 MB glb, outlines), then 0.40-0.43 s per step, almost all of it *toDataURL* | yes on this machine (SwiftShader; any flag set); a GPU machine needs `--use-angle=swiftshader` | Playwright and a Chromium download at runtime (Playwright is dev-only today), three bundled into the viewer's dist, a bake of the glb (the t = 0 pose) |
| VTK 9.3.1 (already installed through cadquery-ocp) | none | fails: its window needs an X server, and the PyPI wheel has no EGL or OSMesa | n/a | n/a |
| matplotlib mplot3d, pyrender/trimesh | not tried. mplot3d sorts whole polygons with no z-buffer, which is wrong for interleaved stacks and slow at 100k polygons. pyrender is not in the lock and needs OSMesa or EGL system libraries | | | |
| (d) Blender | not tried: a 300+ MB dependency. Freestyle would draw beautiful lines | | | |

Samples (`docs/agentlib/guide-samples/`). Each step sample is the same step, layer 1's
links onto their pillars:

- `numpy-sample-step.png`: (b)
- `threejs-sample-step.png`: (c), in the t = 0 pose
- `hlr-sample-step.svg`: (a)
- `hlr-whole-left-side.png`: (a), the whole left side
- `numpy-step09-layer6.png` and `numpy-step03-crank-web.png`: full-pipeline step pictures
  with exploded parts, arrows, label bubbles and the earlier sub-assembly in light orange
- `pdf-page-cover.png`, `pdf-page-parts.png`, `pdf-page-step09.png`, `pdf-page-step17.png`:
  pages of the prototype PDF

**Why (b).**

- It is the only option that shows a coloured highlight over a grey context. That is the
  core of instruction pictures.
- It needs nothing new, is deterministic by construction and runs in plain processes.
- At 1-3 s per picture it parallelises in `workers.submit` (a worker imports numpy and
  Pillow, not the engine), and pictures can be cached.

(a) is the right tool for an IKEA-style line-only edition, or for a hybrid: (b)'s fills
under (a)'s exact vector edges, at +0.1-2 s per step. (c) is the fastest per frame, but it
puts a browser in the build.

## 3. The PDF

| route | measured | same bytes on rerun | cost |
|---|---|---|---|
| matplotlib `PdfPages` (the prototype, `guide/pdf.py`) | 22-28 s for 48 pages: about 0.5 s a page, mostly PNG re-encoding, since every thumbnail is embedded again on every page | yes (`CreationDate` None, TrueType fonts) | none new (matplotlib is installed through cadquery-ocp -> vtk; declare it directly) |
| **reportlab** | 4.8-5.8 s for the same 43 step images, one shared thumbnail XObject per type | yes (*rl_config.invariant*) | one small package (`uv run --with reportlab` pulled 3 packages) |
| Chromium `page.pdf` from HTML/CSS | 0.07-0.1 s a page on an open page | no: 4 bytes of creation date differ (strippable) | the browser at runtime, as in (c) |
| weasyprint, typst | not tried: system Pango libraries, or a separate binary | | |

Recommendation: reportlab. It has real text layout, flowing tables and image reuse, it is
deterministic, and it costs one dependency. Keep matplotlib only if no dependency at all is
allowed.

## 4. Part identification (a proposal: the decision is yours)

The prototype labels each type **P** (printed), **C** (laser-cut) or **H** (bought), numbered
in the order the steps first need it. The default robot comes to 84 types: P01-P36, C01-C16
and H01-H32 or so. The hard case is the 8 mm spacers of 0.7 to 4.0 mm, which look alike.

Options that leave the parts unchanged (the identity gate stays the same):

1. **A 1:1 page.** The PDF knows its scale. A page prints each printed type at actual size,
   side and top view, so a part laid on the page is identified by its outline and its
   thickness against a printed bar. This is cheap and the most robust.
2. **Bags or trays by type.** The print STLs are already one file per type. Name the files
   by label (*P07_ring_9.3x3.0.stl*): this changes output names, not geometry. The guide
   adds a sheet of cut-out bag labels with label, quantity and thickness. A tray map is one
   label grid per print plate.
3. **Colour per type family.** Spacers in one filament, rings in another, sleeves in a
   third. No geometry change, but more filament swaps.

Options that change the parts (the gate's diffs, then a new baseline):

4. **Rim notches.** 1-5 small V notches on a spacer's rim encode its thickness index. They
   fit even an 8 mm ring.
5. **Debossed text** (build123d *Text*) on parts with room: rings and sleeves of 12 mm or
   more, the deck rail, the cradle. It doesn't fit the 8 mm spacers (a 2.4 mm annulus, and
   FDM needs about 2.5 mm characters).

Suggested: 1 and 2 now; 4 if mix-ups still happen.

## 5. Pipeline integration

- **Where.** `spiderpig guide` as a separate command today (`--steps N`, `--jobs N`,
  `--out`). Once the hooks exist, `spiderpig build` would write `ASSEMBLY.pdf` beside
  `ORDER.md`, with a *--no-guide* flag. It would reuse the build's fabrication and
  `group_made` groups, saving about 15 s, and the up-to-date check would cover the guide's
  code.
- **Engine version.** `guide` is in `design.ENGINE_EXCLUDE`, so editing the guide doesn't
  invalidate stores or test caches. Without that, the prototype's first run re-made the
  fabrication.
- **Time budget.** The prototype measured, cold, 101 s in total:

  | stage | time |
  |---|---|
  | fabricate (cache) | 9 s |
  | tessellate | 4 s |
  | labels | 6 s |
  | cameras | 3 s |
  | 43 steps on 4 workers | 39 s |
  | 84 thumbnails, serial | 10 s |
  | cover | 4 s |
  | PDF | 22-28 s |

  Planned changes:
  - put the thumbnails in the workers;
  - use reportlab (about 5 s);
  - mesh once per congruence group, as the bake does (`2_mesh_share`);
  - cache the PNGs in the fabrication cache, keyed by the fabrication entry, a hash of the
    step's spec and the renderer's code digest.

  Target: under 40 s cold, about 5 s when the parts haven't changed.
- **Tests** (`tests/test_guide.py`, sim tier):
  - the renderer draws the highlight and ink and its bytes are stable (quick, `no_fabricate`);
  - a gap goes with the layer above it (quick);
  - every part of the default robot is added exactly once, the steps are numbered
    1..N and each step has text (slow);
  - each side is built bottom up, and each sub-assembly is placed in the next step (slow).

  To add: the same structure test on a Klann and a TrotBot robot; a PDF smoke test with
  `--steps 2` that counts pages.

## 6. Effort

| work | estimate |
|---|---|
| renderer production: workers, PNG cache, shared meshes, label placement | 1 day |
| assembly hooks on the groups, the structured `ASSEMBLY`, the right side's order, split Chicago barrel and screw | 2-3 days |
| reportlab layout, 1:1 page, bag-label sheet, cable page | 1-1.5 days |
| `spiderpig build` integration, *--no-guide*, up-to-date key, docs, other linkages' tests | 1-2 days |
| **total** | **5-8 days** |

## 7. Open decisions

1. The look: shaded toon (b), line art (a), or the hybrid of (b)'s fills under (a)'s edges.
2. reportlab (one dependency) or matplotlib (none).
3. Whether the guide goes in `spiderpig build` by default (+30-40 s cold) or stays
   `spiderpig guide`.
4. Part identification: no geometry change (1:1 page, bags, colours), or rim notches or
   deboss (a gate diff and a new baseline).
5. Turning `ASSEMBLY`'s prose into the structured template the guide and the docstrings
   both read: one source of truth.
