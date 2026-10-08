# The assembly guide

`spiderpig guide` (`mise run guide`) writes `ASSEMBLY.pdf` and `ASSEMBLY.md` for any design,
deterministically and with no agent in the loop. The guide has:

- a cover;
- the parts, each type with its label, quantity and thumbnail;
- the print batches;
- printable bag labels;
- one page per numbered step. Each step page has a shaded toon picture, the parts the step
  adds in orange (bought ones in blue) over the earlier assembly in grey, the stack's parts
  lifted with an arrow, label bubbles, the parts list with quantities, and the sentences.

`spiderpig build` is unchanged except for one thing: its print STLs are named by the same
labels. The decisions behind it are dated in [DECISIONS.md](DECISIONS.md) (2026-10-08). The
feasibility study is at the end of this file.

## 1. The assembly order, structured

`spiderpig/construction/assembly.py` holds the model.

**Who says what.** Each group of a side declares how its own parts go on, through its
`assembly` hook (`construction.base.Group.assembly`, default `[]`). A hook returns `Op`
records. Each `Op` gives:

- a few bodies, or a `Piece` of one (the pieces of a body together cover it);
- the stage they go on in (`STACK`, `UNIT` or `JOIN`);
- a sort key (the layer, for the stack);
- a tag that the robot's order can move;
- a sentence;
- whether the bodies are built on the bench first (`sub`: drawn alone, then put on in the
  next step).

Each hook reads a `SideView`: the side's bodies by bare name, their z in side coordinates,
host, fab, catalog key, the plan's layers and `slot`. `slot` gives the layer a part goes on
with: the gap under a layer belongs to that layer.

| hook | declares |
|---|---|
| `BoltCrank.assembly` | Each web except the hub plate is a bench unit: the standoff that stands on it, screwed on from below. The lowest web also takes the stub and its thrust sleeve. A chain's top screw goes with the web it sits on. The sleeves go on before their riders. The hub plate goes in the unit, with the horn. |
| `ChicagoAxle.assembly` | Each pin is one body in two pieces. The barrel is bonded into its host link and goes on with it. The screw goes in from the cap side once the links above are on. |
| `StandoffAxle.assembly` | The column (standoff, outside button head and washer) goes on the bare outer plate. The inner screw goes in at the join. The take-up shims go on the column's top. |
| `LinkPlates.assembly`, `FramePlates.assembly` | Each link goes in its layer, a foot link's TPU sock first. The outer plate goes first, the inner plate first in its unit. |
| `DriveGroup.assembly` | The servo, its front screws, and the horn with its spacer. |
| `chassis.assembly`, `deck.assembly` (robot level, a `RobotView`) | The tie chains and their screws, the studs, each servo's own centre plates with its rear screw, the deck rails, the deck's electronics on the bench, and the deck onto the rails. |

**The robot's order.** `ROBOT_ORDER` is data (a side on its own uses `SIDE_ORDER`). It lists
the stages, which tags make each step, the sentences a stage puts in place of a hook's, and
the ops it moves:

1. The left leg stack, bottom up, on the bench.
2. The left inner-plate unit, on the bench.
3. The left unit onto its stack.
4. The centre plates and studs. The right servo comes here (`"R.unit.servo"`).
5. The right inner plate, on the robot: its chains go onto the studs before the plate.
6. The right leg stack.
7. The body turned over onto the right leg stack.
8. The wiring.
9. The deck.

`prose(steps)` renders a design's steps as numbered paragraphs, one per stage, every sentence once. `docs/ARCHITECTURE.md` §7.6 is that text for the default robot, written by `python -m spiderpig.guide.prose --write`; a test fails while it is stale. The per-design steps are
`assembly_steps(mech, design)`.

**Defaults and checks.**

- A construction without a hook still gets steps. Each of its bodies joins the step of
  the layer it sits in, with a generic sentence; a robot-level body goes in a last step.
- `assembly_steps` raises if any body is added twice or never, or if a stage takes no op.
- The hooks and this module stay out of the fabrication's code key (`spiderpig.keys`). An
  edited sentence keeps every cache warm. `tests/test_guide.py` checks this. Name a new
  function or field so that it doesn't match an attribute the engine reads: `steps` did,
  and pulled the module in.

## 2. Part labels (`spiderpig/labels.py`)

`part_types` gives each type one label, and the label says what the part is. Reordering
the steps, editing a hook or adding a part can't change what a label means, and two
near-identical parts can't swap labels (review round 1: numbering by first use did both).

- **Printed** (a `group_made` shape group, split by filament as the print files are): a
  family code and its sizes. *SP8-0.7* is a spacer 8 mm across and 0.7 mm high. The other
  round families are RG (ring), RR (crank rider ring), CL (collar), HS (horn spacer) and
  FS (foot sock). SL (sleeve) and TS (thrust sleeve) give across x long, *SL8.5x23.9*. DR
  (deck rail) and BC (battery cradle) give their box.
  - A filament other than the design's adds its name, *-PETG*.
  - A mirror image adds *M*.
- **Laser-cut**: a role code and the outline as its per-part DXF measures it
  (`layout.outline_size`), not the bounding box: *LK75x27*. The codes are LK (link), FR
  (frame plate), CW (crank web), CP (centre plate) and DK (deck plate).
- **Bought**: the catalog key, shortened by `bought_label`, *M3-BH-8* or *CHI-M3-16*. The
  servo's own horn, which has no catalog item, is *HORN-<servo>*.

Two types whose labels still agree get a, b, ... in order of volume, then area: geometry,
never order. Their names then say what differs: the area of plate, or the volume. The
types are listed by kind, then in the order the steps first need them (`assembly_order`).
The labels don't depend on that order (tested).

**No part changes.** The labels name the files:

- `spiderpig build` and `api.export` print STLs: *<label>_<what>.stl*, e.g.
  *SP8-0.7_top_spacer.stl*, with `_mirrored` added for a mirror image (`print_stems`);
- the per-part DXFs: *<label>_<part>_x<qty>.dxf*;
- the sheets' `parts.csv` and `order.csv`, which also get a `label` column
  (`laser_labels`).

The guide's prints table, parts pages and bag labels say the same. The bag labels are a
grid to print at 100 % and cut out, one per printed and bought type, each with its label,
quantity, name, size and thumbnail. To use them, print each batch and bag it with its
label.

**Tests.** Each label's quantities summed over the steps' parts lists equal its type's
quantity (the BOM's), checked on the default robot, `klann_lego` quad and
`hoecken_pantograph`. Every bought label is named from the catalog.

**Open note.** The deck rail's heat-set inserts are becoming captive nuts in the BOM, on
another branch. When that lands, the deck hook's rail sentence ("the heat-set inserts
pressed into the rail") and its `deck_insert` pattern need the new part.

## 3. Pictures (`spiderpig/guide/render.py`)

The renderer is numpy plus Pillow, with no GPU and no display:

- an orthographic projection;
- a scanline z-buffer, resolved by an integer `maximum.at`, so the output is
  byte-identical on every run;
- flat banded (toon) shading;
- ink lines found in image space, where the part, the depth or the normal jumps;
- 2x supersampling.

**Cameras.**

- A side's stack steps keep the stack's axis up the page.
- A sub-assembly, a unit or a robot step picks, among 2-3 cameras, the one where its new
  parts show the most pixels (`choose`).
- A piece of a body (a Chicago pin's barrel) is drawn from its triangles inside the
  piece's z interval.

**Label tags.** `render` reports where each new part shows. Once the labels are known,
`bubbles` puts one tag per type on a leader: a rounded box as wide as its label. Each tag
takes the free spot, at three distances round the part, that overlaps no other tag nor
another part's anchor and covers the least drawing. The font is Noto Sans, subset to Latin
and shipped in `spiderpig/guide/fonts/` (SIL OFL 1.1, `OFL.txt` beside it), so the
pictures are the same on any machine.

**Wiring.** The wiring step's picture is a block diagram (`guide/wiring.py`), drawn
generically:

- Each bought electronics or servo body is a box.
- The links follow the power and the bus: battery, protection board, switch, board, servos,
  with the charger into the protection board.
- Each cable the BOM buys goes on the link its key names (an XT30 pigtail, a DC plug, a bus
  or Y cable). A cable it can't place is listed under the diagram.

**Workers.** The step pictures, the thumbnails and the cover are drawn in
`workers.submit` processes (`render_jobs`, `--jobs`, default 4). A worker imports numpy
and Pillow, not the engine, and reads the meshes from one `.npz`. The main process groups
the parts for the labels meanwhile.

## 4. The PDF (`spiderpig/guide/pdf.py`)

The layout is reportlab: `pdf.write(path, doc)`, where `doc` is a `guide.doc.Doc`, plain
data plus PNG paths. It is A4 landscape, in Helvetica (not embedded). Each image is one
shared XObject however many pages use it. `rl_config.invariant` makes the bytes identical
on every run.

The pages, in order:

- the cover;
- the parts, 24 to a page;
- the prints, 26 rows to a page, with a total;
- the bag labels, 24 to a page;
- one page per step.

`ASSEMBLY.md` beside the PDF has the same steps as text.

## 5. Pipeline and cache

The guide is cached beside the design's fabrication, in
`<store>/fab/<fab key>-<env>/guide-<entry>/`:

- every picture under a hash of what it draws plus the renderer's code key
  (`keys.source_key` of `render_jobs`);
- the meshes (`meshes.npz`);
- the finished guide, under the code key of `build_guide`. That key reaches every
  construction's `assembly` hook by name, and `spiderpig.labels`.

So an unchanged design is a copy, and a changed sentence redraws nothing. When the cache is
off (`SPIDERPIG_FAB_CACHE=off`), there is no store, or a caller passes its own `design` and
`mech` (the tests' seam), nothing is kept. `guide` is in `design.ENGINE_EXCLUDE` and in the
import-linter layers beside `bake` and `build`.

**Measured** on the default Strider double robot, 2026-10-08, on the shared 20-core box at
load 11-18 (others' jobs):

| run | time |
|---|---|
| cold, no picture cached | 56 s, of which a fabrication 10 s (the fab key had just moved) |
| a guide edit, pictures cached | 24 s |
| warm, unchanged design | 0.6 s in the guide, 4.2 s wall with imports |

Where the cold time went:

| stage | time |
|---|---|
| fabricate | 0.2 s from the cache, 10 s afresh |
| meshes | 4.5 s |
| labels' grouping | 6.7 s, overlapping the drawing |
| pictures (129: 46 steps, 82 thumbnails, the cover) | 20 s more on 4 workers |
| PDF | 7 s |

The first run in a checkout also computes the code keys, about 10 s. A full-robot picture
takes 2.7 s of one core, plus 1.2 s to choose its camera. The 40 s cold target, fabrication
aside, holds at normal load (3-6), not at 15. Re-measure before quoting.

**Tests** (`tests/test_guide.py`, sim tier).

Quick (`no_fabricate`):

- the renderer's bytes, highlight and marks;
- label tags never overlap, and fit their labels;
- a gap goes with the layer above it;
- the robot's order;
- labels and print stems on a toy mechanism.

Slow:

- on the default robot: every body is added exactly once (pieces cover their body), the
  stacks go bottom up and mirrored, the stage order holds, the right servo goes in with the
  chassis, a pin's barrel comes before its screw, the hub plate is in the unit, no step
  says a sentence twice, and what the old prose said is still said (journal hole, light
  press, far holes, ...);
- the labels: unique, unchanged by a reversed step order, every part in the steps' parts
  lists as often as the BOM has it, bought ones named from the catalog, laser ones by
  their outline;
- `klann_lego` quad and `hoecken_pantograph` (one side): the same structure and quantity
  checks;
- docs/ARCHITECTURE.md's assembly order is current;
- the PDF's page count;
- the hooks stay out of the fabrication key.

## 6. Feasibility study (2026-10-08, kept for its measurements)

| option | look | time per image | same bytes | dependencies |
|---|---|---|---|---|
| **numpy z-buffer** (chosen) | flat banded tones, ink outlines, colour highlight | 1.4-2.7 s at 1200x900, 2x supersampled | yes | none |
| OCCT hidden lines (build123d *project_to_viewport*) | exact vector line art, no fills | 0.13 s for a small step, 2.2 s for a whole side | yes | none |
| three.js in Playwright Chromium (SwiftShader) | the viewer's look, 1-pixel edges | 6 s cold, then 0.4 s | yes, with `--use-angle=swiftshader` on a GPU machine | a browser at runtime |
| VTK 9.3.1 (from cadquery-ocp) | none | fails: it needs an X server; no EGL or OSMesa in the wheel | n/a | n/a |
| matplotlib mplot3d, pyrender, Blender | not tried: no z-buffer, system GL libraries, or 300+ MB | | | |

For the PDF:

- **reportlab** (chosen): 4-5 s for about 55 pages at normal load, with identical bytes.
- matplotlib's PdfPages: 22-28 s, because it re-encodes every thumbnail on every page.
- Chromium's `page.pdf`: fast, but its dates differ between runs and it needs the browser.

Samples are in `docs/agentlib/guide-samples/`: the final guide's pages (`pdf-page-*.png`)
and the study's renders of one step (`numpy-`, `threejs-`, `hlr-`).
