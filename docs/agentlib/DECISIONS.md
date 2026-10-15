# Decisions

Two records: the agent surface's design decisions (2026-09-30), and the hardware and
design decisions since (2026-10-03 on), one entry each: date, what, why, where in the code.
The numbers here are each decision's own reasons as of its date; the current designs'
numbers are in [DESIGNS.md](DESIGNS.md) (generated), how each part works in
[ARCHITECTURE.md](../ARCHITECTURE.md) §5-7 and the module docstrings. Workstream decisions
of the cleanup (D1-D5) are in [ROADMAP.md](ROADMAP.md).

## Agentic harness spec ([SCOPE.md](../history/SCOPE.md) section 6)

1. Spec breadth: NARROW. v1 Spec has only fields the engine can verify today; unknown fields
   and wildcards are rejected with a message. compile(spec) is a compiler (seconds); search is
   a separate explicit call built on top later. (2026-09-30)

2. Hard vs soft: physical limits (size, budget, stack thickness, ground clearance) HARD by
   default (a miss fails verify); gait quality (bob, slip, speed, stride) SOFT by default
   (reported with a score); any field can flip with `hard: true/false`. (2026-09-30)

3. CAD exposure: EXPOSE build123d objects. The Python API hands back live solids
   (design.parts[name].solid) for agents to measure or modify; files + numbers stay the
   MCP-side representation (solids can't cross that boundary). Implication: modified parts
   bypass the claims contract unless re-checked; the API should offer a re-check
   (contract.clashes / bad_solids) on any edited part. (2026-09-30)

4. Store: PER PROJECT, ./.spiderpig (git-ignored): stable design ids across runs, plan cache,
   parts kept until an explicit gc; multi-user by sharing the folder. (2026-09-30)

5. Leg modules: NAMED PRESETS only in v1 (single | double | decker | quad, plus a linkage's
   own modules) with phases per leg; explicit leg lists later on Module(legs, cranks).
   (2026-09-30)

6. Viewer: SHIP viewer/dist AS PACKAGE DATA; `spiderpig view <design>` serves it without Node
   on the user's machine; the build runs at release time. (2026-09-30)

7. Sequencing: BOUND THE PLANNER FIRST (a compile must never hang: budgets bound the search,
   PlanError with a tally otherwise), then v1 in order, each in a Fable subagent on a
   worktree, merged and verified: 1) Spec + compile/verify Python API with solids exposed,
   2) per-project store, 3) MCP server over files and numbers, 4) viewer as package data and
   `spiderpig view`. Assumed without asking (SCOPE.md proposals): metric semantics pinned per
   field (section 3.1), speed flagged `estimated` until sim confirms it; sync Python API, MCP
   with Tasks and a process pool; conformance compares metrics not STEP bytes; "no plan" means
   "no plan within budget" with the proof always returned; mechanism specs verify OutputCheck
   and skip walking, compound machines unsupported and said so; offline servo CAD marks a
   design `estimated`. (2026-09-30)

## Hardware and design

Oldest first. "User" marks the user's own decisions; the rest were engineering calls checked
by the audit. Most entries moved here from CLAUDE.md on 2026-10-08 (W7).

### 2026-10-03: Chicago screw pins between the links

- **What:** every link pin is an M3 Chicago screw (Harfington / uxcell, 4 mm barrel, 8.5 mm
  heads; stock barrels `fastener_catalog.CHICAGO_LENGTHS`: 1 mm steps 4-16, then 18, 20,
  22, 23, 25, 28 ... 80). One **printed head spacer** per end takes up the barrel's fixed
  length (no PTFE washer or DIN 988 shims); the lowest link is bonded to the barrel with
  epoxy. A planner rule: a pin's links must fit a stock barrel (`ChicagoShaft.column`).
- **Why:** the pivot review of 2026-10-03 (the table in `construction/pivots/chicago.py`'s
  docstring): the least play of the bought pins that assemble from one side.
- **Where:** `spiderpig/construction/pivots/chicago.py`; `--pin chicago`.

### 2026-10-03: the default build is `--pin chicago --pillar standoff --crank bolt`

- **What:** `BuildConfig()` (the Strider double, `linkage.DEFAULT`) is built with Chicago
  pins, standoff pillars and the bolt crank. Before: `--pillar printed --crank keyed`.
- **Why:** the crank study and the pivot review of 2026-10-03: metal shafts, stock parts.
- **Where:** `config.DEFAULT_CRANKS`, `BuildConfig`.

### 2026-10-04: single aluminium web plates for the bolt crank

- **What:** on an aluminium crank sheet the bolt crank resolves to single plates
  (`BoltCrank.for_sheet` / `resolve(ctx)`, `_WebPlates`): every web one plate, no plate
  stack, so no bond and no lock screw. The round crankpin (now `--crank bolt_round`) is a
  goBILDA 1501 round 6 mm standoff clamped between its webs by an M4 button head into each
  end.
- **Why not the joinery plan's M6 bolt:** ISO 4014's 18 mm thread leaves 4-5 bare layers
  under every run unless the bolt is cut, and nothing is cut.
- **Where:** `spiderpig/construction/crank/bolt.py`, `plates.py`.

### 2026-10-04: hex-standoff crankpins (user, decision 1)

- **What:** every crankpin and journal is a stock M3 x 5.5 AF steel hex standoff
  (`crank_catalog.HEX_M3_LENGTHS`) in hex pockets of the single webs (`BoltCrank.pin="hex"`,
  the default); the riders turn on a printed 8.5 mm sleeve. The round friction clamp stays
  as `bolt_round`.
- **Why:** a positive drive instead of a friction clamp whose coefficients are UNVERIFIED
  (`BoltCrank.capacity`); jam SF >= 2.25 on 0.100 in 6061-T6.
- **Where:** `construction/crank/hex.py` (`fit_hex`, `hex_gap_fit`, `BoltCrank.hex_cut`),
  `capacity.py` (`hex_bearing_nm`, `hex_capacity`).

### 2026-10-04: the crank sheet is 0.100 in 6061-T6

- **What:** `config.crank_sheet` is `al6061_2p5mm` (`materials.thinnest_sheet("crank")`).
- **Why:** the thinnest whose hex pockets hold 2 x 0.85 N·m at SF 2 with the recess; 5052
  would need 0.125 in, thicker than its 3 mm layer, which moved the pillars' columns off
  stock lengths. An acrylic crank sheet is refused since W2.
- **Where:** `spiderpig/materials.py`, `tests/test_joinery.py`.

### 2026-10-04: the thinnest sheet per part (user)

- **What:** no weight budget; every plate as thin as strength and the service allow.
  `materials.thinnest_sheet(role)` takes the thinnest stock that passes the role's checks
  (`materials.ROLES`: out-of-plane bending at a pin's clamped end at 155 N jam, SF 2; the
  role's smallest hole against SendCutSend's minimum hole = thickness). Frame plates 0.080 in
  5052 (`al5052_2mm`).
- **Why the frame's 0.080 in:** 0.100 in and up can't take the servo's 2.4 mm holes; 0.063
  in fails the pillar end.
- **Where:** `spiderpig/materials.py`; `BuildConfig` pins the frame and crank sheets.

### 2026-10-04: centre plates 0.090 in (user, decision 4)

- **What:** the centre plates take the thinnest stack that seats the most rear screws
  (`chassis.centre_sheet`: 0.090 in x 4 on the STS3215, over 0.080 x 5).
- **Where:** `spiderpig/construction/chassis.py`.

### 2026-10-04: klann_lego's crank rider b1 in 6061 (user, decision 3)

- **What:** `materials.LINK_SHEETS`: `klann_lego`'s b1 is cut from 6061 (0.125 in, the
  user's allowance), and a Klann variant's foot link too; `BuildConfig.link_sheets`.
- **Why:** b1 jams past what acrylic holds (`strength.link_rows` names 6061 for it).
- **Where:** `spiderpig/materials.py`, `plates.rider_bosses`.

### 2026-10-04: clearance gaps, layer thicknesses, per-part sheets

- **What:** a fastener's head beside a link is a clearance shape (`Placed.gap`) in a thin
  gap or sunk into the layer beside it; each layer as thick as its thickest plate; the plan's
  z made by `stack.finalize`, every claim checked again at it (`Layout.final`), a layering
  that doesn't build there a dead end (`PlanReject`).
- **Why:** full-layer heads made every stack taller than its parts.
- **Where:** `spiderpig/stack/plan_z.py`, `spiderpig/materials.py`.

### 2026-10-04: heads sunk first (user)

- **What:** `StackSpec.heads` "best" (the default): heads sunk; in gaps only when that finds
  no plan. A single-plate crank plans `heads="gap_sink"` (that evening; `stack.HEADS_ORDER`):
  gaps first, a sunk crank head standing in a rider's layer, then the pivots' heads sunk with
  the crank's and the drive's (`stack.GAP_GROUPS`) kept in their gaps
  (`heads_claims(keep=...)`, `finalize(sink_all_but=...)`). The gap search gives up after
  `stack.GIVE_UP` layerings with no route instead of running to its deadline.
- **Why:** the gap search restored the planner's buildability but cost the test suite its
  time; sunk heads first restored it. TrotBot's heel and toe plan only the second way.
- **Where:** `spiderpig/stack/plan_z.py`, `spiderpig/stack/search.py`.

### 2026-10-04: the planner searches up to 60 layers

- **What:** `StackSpec.max_top` is 60.
- **Why:** the Strider quad's plan needs more than the old bound.
- **Where:** `spiderpig/stack/plan.py`.

### 2026-10-04: mechanisms get the bolt crank too

- **What:** `config.DEFAULT_CRANKS` names `bolt` for walkers and mechanisms; a short crank's
  top screw head over the hub plate sits in a pocket of the printed horn spacer
  (`DriveGroup.realize`; `BoltCrank.hub_head_need`, `hub_capped`). The construction a design
  gets is data: `config.default_crank` reads `config.MODULE_CRANKS`, then `LINKAGE_CRANKS`,
  then `DEFAULT_CRANKS`.
- **Where:** `spiderpig/config.py`, `spiderpig/servos/mount.py`.

### 2026-10-04: TrotBot's heel and toe stay on the round crankpin

- **What:** `LINKAGE_CRANKS` holds them on `bolt_round`.
- **Why:** b7 passes crankpin J1 at 10.2 mm; the hex crankpin's 8.5 mm sleeve needs 11.2.
  The round 6 mm standoff clears it and plans.
- **Where:** `config.LINKAGE_CRANKS`.

### 2026-10-04: the hub chain is capped (the first assembly audit)

- **What:** the chain that ends in the hub plate has no screw over the hub plate
  (`BoltCrank.hub_capped`); the hub plate is held by the horn screws, and the hub plate,
  horn, servo and inner plate go on as one unit (the whole robot's order, since 2026-10-08
  `construction.assembly.ROBOT_ORDER`). The removed `--crank bolt_hub_screw` kept the screw.
- **Why:** no assembly order drove that screw with the horn screws coming up through the
  hub plate from below.
- **Open (user):** the round standoff (`bolt_round`) still screws over the hub plate (its
  friction clamp needs both screws), so those designs have no assembly order
  (`_WebPlates.assembly_issue`, an `assembly:` audit error since the second assembly audit):
  a hex plan for them, a hub joint fastened from the horn side, or leaving them out of the
  first build.

### 2026-10-04: axial retention of the capped chain

- **What:** its printed sleeve is a light press on the hex (`BoltCrank.capped_press`
  0.1 mm), so the sleeve, caught between the plates, carries the standoff; the crank body
  stops toward the outer plate on a printed thrust sleeve round the stub (`stub_thrust`,
  `stub_thrust_r`). The removed `--crank bolt_unretained` had neither stop.
- **Why:** an M3 retainer under the outer plate, which the audit proposed, stops the stub
  moving in (which the capped sleeve already does), not out.
- **Where:** `spiderpig/construction/crank/bolt.py`.

### 2026-10-04: bus plugs, one rear screw per servo, the idler horn left out

- **What:** `ServoSpec.bus_ports` (the STS3215's 5264 3P, opening "end", UNVERIFIED) cuts
  an open slot through every centre plate the plugs stand in (`chassis._port_slots`); the
  rear holes within 2 t of it are dropped, so each servo keeps one rear screw. Since the
  second assembly audit the STS3215's rear idler horn stays in the box (`Idler.fitted`,
  `servos.model.unfit_idler`); the plates clear its 6 mm boss (`chassis.RoundRelief`).
- **Why:** the plugs must go in after assembly; the idler's 21 mm square relief left the
  near rear screw 1.1 mm of web. Measure a servo and a plug before cutting.
- **Where:** `spiderpig/construction/chassis.py`, `spiderpig/servos/`.

### 2026-10-04: glue-free chassis, deck rails, feet (the joinery plan)

- **What:** frame ties are chains of uxcell M3 round standoffs with set screws through the
  aluminium centre plates (no adhesive); the deck rails are screwed to the inner plates;
  every foot gets a printed TPU 95A sock (`plates.foot_sock`); the sim's floor friction is
  0.65 (`SimParams.friction`).
- **Where:** `construction/chassis.py`, `construction/deck.py`, `construction/plates.py`.

### 2026-10-04: body plates keep 1 x t; the `web` cut rule

- **What:** a hole closer than 1 x the thickness to an edge or hole in metal, or under the
  service's minimum hole, is a cut-rule error; under 2 x a warning. The web round every
  non-circular cut-out is checked (`web`). Frame plates get bosses of 2 t
  (`plates.boss_web`), ties move to 2 t off the servo's holes (`chassis.tie_locals`).
- **Where:** `spiderpig/manufacture.py`.

### 2026-10-04: the Klann variants' quads at 0, 0, 180, 180

- **What:** `klann_patent`, `klann_lego`, `klann_long_legs`, `klann_high_step` default to
  `linkage.KLANN_QUAD`; the demo `klann` keeps the generic quad.
- **Why:** at 0, 0, 180, 180 the demo's quad found no plan with standoff pillars in 60000
  nodes. Klann stays registered as the wobbly demo.

### 2026-10-05: the Strider decker and quad on the hex crank

- **What:** `config.MODULE_CRANKS` is empty; `BoltCrank.hex_gap_fit` opens the gaps along a
  chain (over its lowest web first, each to 4 mm, printed rings filling them) to the next
  stock length; `fit_hex` splits what stands past the plates so the taller gap need is
  least (`air_over`).
- **Why:** no stock hex standoff fit their long crankpins at the plan's z (the series steps
  5 mm past 25 mm) and 4 mm crankpin gaps pushed the Chicago pins off their stock barrels.
- **Where:** `spiderpig/construction/crank/hex.py`.

### 2026-10-05: rider bosses sized on the resolved crank (r4)

- **What:** an aluminium link riding a crankpin grows a boss round its bore to 1 x its
  thickness of web (`plates.rider_bosses`, `RIDER_BOSS_T`), sized on the crank resolved for
  the crank sheet. Every `klann_lego` module moved to the hex crank.
- **Why:** unresolved it read the round 6 mm pin, which left `klann_lego`'s 8.8 mm hex bore
  2.02 mm from b1's edge.

### 2026-10-05: hoecken_pantograph's crank on 0.080 in 6061 (r4)

- **What:** `config.LINKAGE_CRANK_SHEETS` (`BuildConfig.crank_sheet` "" fills it in).
- **Why:** on 0.100 in its 12 mm crank leaves 2.16 mm between the hub plate's hex pocket
  and a horn screw hole, under 1 x t; on 0.080 in the hex holds at SF 3.89.

### 2026-10-05: one-piece standoff pillars (r5)

- **What:** a column one stock goBILDA 1501 length fills is that standoff; any other is one
  MISUMI NETRF6 round 1018 steel standoff made to its length (`StandoffAxle.one_piece`,
  `crank_catalog.pillar_shaft`), never spliced (the removed option `splice_build="shaft"`).
  The spliced `standoff_hand`, `standoff_bench` and `standoff_m3` were removed by W2; a
  short column the splices couldn't fill stays refused (`StandoffAxle._spliceable`), so the
  plans didn't move.
- **Why:** at the plan's own z the hand-tight splices opened at jam SF 1.43 (double) and
  0.51 (quad).
- **Where:** `spiderpig/construction/pivots/standoff.py`.

### 2026-10-05: the strength check at the plan's own z (r5)

- **What:** each link at its layer's mid-plane, a pillar's supports at the plates' faces,
  gaps and thicker plates included (`wobble.column_wobble`'s `layer_z`,
  `wobble.layer_mid`); a link beside a clearance gap bears on what the column holds there.
- **Why:** every layer as `k x pitch` read a stack's spans up to 2 x short, and a link
  beside a gap fell back to its free tilt.

### 2026-10-05: Chicago barrels on the Strider at most 23 mm

- **What:** `chicago.MAX_BARREL`, a planner rule (`ChicagoAxle.resolve`).
- **Why:** the quad's J7 on a 30 mm barrel was jam SF 1.8; capped, SF 2.6 in the same
  layers, and fewer barrel lengths to buy.

### 2026-10-05: klann_lego's torque limit 0.60 N·m

- **What:** `config.LINKAGE_TORQUE_LIMITS`, written into ORDER.md as a servo-firmware note.
- **Why:** its 6061 leg b4 bends at D under the foot's lever; at 0.85 N·m the jam rates it
  SF 1.58 and no thicker 6061 fits its layer. Walking needs far less.

### 2026-10-05: the deck lowers in past the pillars' heads; no CA

- **What:** the battery cradle is screwed to the deck; `deck.path_notches` notches the plate
  round every static part in its way, `deck.nut_z` (then the inserts' place) moves the deck
  screws into the bay where a notch would leave one too little web, and `deck.deck_path`
  proves the way.
- **Where:** `spiderpig/construction/deck.py`.

### 2026-10-05: ordering outputs, shop supplies on hand (user)

- **What:** `spiderpig build` writes one DXF per different part (`layout.save_parts`,
  `order.csv`) and `ORDER.md` (`hardware/order.py`); shop supplies (filament, threadlocker)
  are taken as on hand (`bom.ON_HAND`): listed, not ordered. `--kerf` overrides every
  sheet's kerf.
- **Why:** SendCutSend and Ponoko take one part per file; the user keeps filament and
  threadlocker.

### 2026-10-07: only the default constructions remain (user, D1; W2)

- **What:** the cranks `bolt` and `bolt_round`, the `chicago` pin and the one-piece
  `standoff` pillar. Removed: the printed, `keyed`, `keyed_float` and acrylic two-plate
  cranks, `bolt_hub_screw`, `bolt_unretained`; the rod, PTFE, bolt, bearing, bushing,
  `chicago_bushing` and printed pins and pillars; the spliced standoffs. A config, spec or
  stored design naming one fails with its replacement.
- **Where:** `config.REMOVED_CONSTRUCTIONS`, `config.REMOVED_PARAMS`.

### 2026-10-07: approved output changes (user, D2-D5; W8)

- **What:** the cantilever pillar's gap ring, tie-stable rounding (`spiderpig/rounding.py`),
  `shapes.pill` as one extruded stadium, mirror-identical prints grouped as "same"
  (`bom._proper_fit`). Each change's gate diff is in [W8-gate-diffs.md](W8-gate-diffs.md);
  the gate baseline is `w8-2a130c8`.

### 2026-10-08: STS3215 bus sockets top-entry, one per servo, both rear screws (user)

- **What:** the research of 2026-10-08 (four sources: Feetech's rear view, Waveshare's photos,
  an SO-101 build photo, both STEP models) puts the STS3215's two bus sockets in a trench in
  the rear face, vertical 5267-type headers, the plugs in along the servo's +z:
  `ServoSpec.bus_ports` is `opening="face"` (no longer UNVERIFIED; one hand measurement
  remains, the plug-plus-wire height above the rear face). Each servo's plug goes in the
  socket on its own +y side (`BusPorts.used="own"`), so the two plugs never oppose and the
  centre stack holds one plug and its wires (`chassis.centre_stack`). First fed by a Y cable
  (no product found); then, the same day, each servo's own boxed cable to one of the driver
  board's two bus ports (`servo_bus_cable_5264`, on hand: Waveshare's docs list "Bus Servo
  Control interfaces", two headers; Seeed's part list puts a "JST Wire" in each servo's
  box), so no Y cable and no extra cart. The centre plates get a closed window round
  each plug and an open channel toward +x for its wires (`chassis._port_slots`,
  `PortCut`). Both rear screws per servo are kept: a shank keeps 1 x t to the window, not
  2 x t (`chassis.BUS_WEB_T`), the near head's recess opens into it (`_recess_bridges`), and
  the raised pad (relabelled from "connector housing") takes its measured outline grown
  0.2 mm (`Relief.grow`). The far hole's web to the pad's relief is then 1.62 mm: 1 x t in
  0.063 in, not in 0.080 in, so `chassis.centre_sheet` takes a sheet thinner than the frame's
  where it seats more rear screws (0.063 in 5052; under the old slot decision 4's 0.090 in
  kept one screw per servo). The SO model's "pins" relief stays, relabelled (they are the
  headers, drawn standing proud where the real ones are sunk: the audit draws that model, and
  the one-socket window doesn't cover the other socket's), not dropped as first decided.
- **Why:** the old "end" slot was wrong (the plugs go in perpendicular to the plates), and two
  opposed plugs would need twice the stack; the far rear hole was lost to the pad's relief,
  not to the slot.
- **Review of 2026-10-08:** each servo's channel is its own, centred on its plug and cut
  only through the plates its wires pass; the plug and wires keep 1 mm of margin
  (`chassis.BUS_WIRE_MARGIN`), so the stack holds the plug, its wires, the margin and the
  other servo's pad: 0.063 in, more plates (DESIGNS.md). The rear screws are the stock
  M2 x 6 the servo ships with (`MountHole.stock`; one SKU, priced at Accu), through two own
  plates. The plug is seated through the window before the stack closes (the centre
  stack's step, `chassis.assembly`). The SO model's header pins are `Relief.model_only`: cleared by the plates, not by
  the stack's plug room. `strength.centre_plate_row` checks the plates at the jam torque.
- **Verify on the first article:** the plug-plus-wire height above the rear face (6.0 taken,
  6.5 the research's upper estimate, 7.0 reserved); the far rear hole's web to the pad's
  relief, 1.617 mm against the 1.6 mm (1 x t) error level, 0.017 mm of margin; the pad
  cut-out clears the WS3 model's pad corner by only about 0.34 mm (0.2 grown, plus the
  rounding); the bus wires' diameter (1.3 mm taken, `BusPorts.wire_w`): the far rear screw's
  head clears the wires by about 0.05 mm in y, so a 1.4 mm wire (0.15 mm more toward +y)
  touches `L/R.rear_screw1`; and whether the boxed bus cable reaches the board.
- **Warnings left** (errors 0): each end and second plate's far rear hole 1.62 mm from the
  pad's relief (an edge where it merges with the other servo's channel); over 1 x t, under
  2 x t.
- **Where:** `spiderpig/servos/spec.py`, `spiderpig/servos/catalog.py`,
  `spiderpig/construction/chassis.py`, `spiderpig/hardware/electronics.py`; the gate diffs in
  [BUS-gate-diffs.md](BUS-gate-diffs.md).

### 2026-10-08: the assembly guide, generated (user)

- **What:** `spiderpig guide` (`mise run guide`) writes `ASSEMBLY.pdf` and `ASSEMBLY.md`
  for any design, with no agent in the loop. The pictures are shaded toon renders from a
  numpy z-buffer renderer in worker processes. The layout is reportlab, a new dependency.
  The robot's assembly order is now structured data: each construction's `assembly` hook,
  plus `construction.assembly.ROBOT_ORDER`. It replaces the prose `construction.robot`
  kept until now. Parts are identified by labels that say what they are: a family and its
  sizes (*SP8-0.7*, a spacer 8 mm across and 0.7 mm high), a laser part's outline
  (*LK75x27*), a bought part's catalog key (*M3-BH-8*). Numbering by first use was
  rejected in review: a reorder renumbered every label. `spiderpig build`'s print STLs
  and cut files are named by label (*SP8-0.7_top_spacer.stl*), and the guide prints the
  bag labels. No part changes.
- **Why:** the user asked for a build-pipeline guide with stylised pictures, and for no
  edits to the parts: identifying parts by labels and bags needs no embossing, so the
  identity gate stays put. Of the renderers tried, OCCT hidden lines can't fill a
  highlight, three.js needs a browser at runtime, and VTK needs an X server
  ([GUIDE.md](GUIDE.md)).
- **Where:** `spiderpig/construction/assembly.py`, the hooks (`BoltCrank.assembly`,
  `ChicagoAxle.assembly`, `StandoffAxle.assembly`, `LinkPlates.assembly`,
  `FramePlates.assembly`, `DriveGroup.assembly`, `chassis.assembly`, `deck.assembly`),
  `spiderpig/labels.py`, `spiderpig/guide/`.

### 2026-10-08: the BOM decisions (user; the BOM study)

- **What (sourcing):** the study's sources (`hardware/sources.py`: Bolt Depot for the M3
  hardware, sold singly; DigiKey for the Wurth parts; one Amazon cart); a sheet a service
  cuts is no purchase (`bom.cut_by`, `bom.bought`: the 3 mm acrylic's Inventables sheet was
  in the total and no cart) but a cutting line per service and sheet, the material
  included (`bom.CutRow`, `bom.cut_estimate`: an area estimate calibrated to SendCutSend's
  live quotes of 2026-10-08, USD 117.14 for the Strider double's 47 parts, the estimate
  117.93), in the BOM's total, ORDER.md's total (shipping apart) and verify's cost floor
  (one part's least cut per sheet); the epoxy's priced offer (J-B Weld) first; the servos'
  M2 x 6 self-tappers on hand (`bom.ON_HAND`: every STS3215 box has 18 screws, Seeed's
  part list and Waveshare's photo).
- **What (design):** the Strider's pins planned on 7, 10, 16 and 22 mm barrels
  (`chicago.BARRELS`, per linkage and crank: a global list leaves both Klann quads, and the
  Strider's `bolt_round` crank, without a plan; the
  step rule of `ChicagoShaft.check` stays on the catalog's steps) and built on 10 and 16
  only; one Tattu 2S 450 mAh LiPo for the Ovonic 4-pack (the cradle drawn from its 62.5 x
  16.2 x 14.7 mm, its outer end in place, `deck.BATTERY_X1`); the clamped M3 shims bought as
  Bolt Depot's DIN 125 washers, modelled and claimed at their 7 mm (`bom.shim_od`), and its
  cup-point set screws (no Accu cart); SendCutSend cuts the 3 mm acrylic too, on its
  0.118 in rules (sendcutsend.com/materials/acrylic, 2026-10-08: hole .047 in, bridge
  .053 in, part .187 x .375 in; `hardware.parts.SCS_RULES_ACRYLIC`), kerf 0, Ponoko
  selectable (`acrylic_3mm_ponoko`), the deck plate's cable-tie web 1.5 mm
  (`deck.CABLE_TIE_WEB`, over the 1.35 mm bridge); captive M3 nuts in the deck rails for
  the heat-set inserts (`deck.deck_nut`, under a 1.6 mm roof, `deck.NUT_ROOF`).
- **HV LiPo (review, 2026-10-08):** the Tattu is an HV pack: charge it as 2S to 8.4 V only,
  never the IP2326 board's 3S / 12.6 V jumper setting (the board stops at 8.3-8.5 V on 2S,
  safe for the HV pack) nor an 8.7 V HV charger, the servos being 7.4 V parts. ORDER.md's
  battery and charger lines and the guide's wiring step say so
  (`hardware.electronics.HV_WARNING`).
- **Why:** a short, cheap order: the Strider double's ORDER.md from 16 carts and 36 lines
  to 6 and 28, one upload for every cut part, no soldering iron; the Strider's pin jam
  safety factor rises with the shorter barrels' spans (the numbers: DESIGNS.md).
- **Where:** each change's gate diff in [BOM-gate-diffs.md](BOM-gate-diffs.md); the gate
  baseline is `bom2-5989d50` (after the review and the merge; `bom-bbf7001` before). Also fixed: the crank router raised IndexError when no screw
  length fits any crank joint (`route._ranges`); it is now a planner blocker.

### 2026-10-11: the platform registries; ball-bearing pivots back (user; the robot-arm workstream)

- **What:** the engine becomes an extensible robot design system (a desktop palletizer arm
  is its first new robot). Phase 0 adds the registries: fab methods, construction roles
  (`BuildConfig.roles`, per design or per drive / joint), vendor CAD for every bought part,
  joint error terms, material stiffness and expansion. Phase 1 brings ball bearings back as
  swappable joinery: `--pin ball` (an MR84ZZ bonded into every rider link of the Chicago
  pin) and `--pillar ball_pillar` (an MR106ZZ in every link on the standoff; `link_radius`
  6.6 mm). The removed MF63ZZ `bearing` key now names `ball` / `ball_pillar` as its
  replacement. The defaults stay `chicago` / `standoff` (the walkers are unchanged).
- **Why:** an arm's precision is its joints' play: a plain hole is 0.2-0.35 mm of diametral
  clearance, a bearing on the barrel 0.06 (the barrel's fit plus ISO 5753-1 CN). The user
  asked for bearings as one of the swappable joinery types, with McMaster-Carr's CAD.
- **McMaster CAD:** McMaster shows its catalog only to a logged-in browser; a scripted
  session gets a login wall, and getting around it is not done. So the models come through
  the user's own session (`spiderpig cad login`, then `fetch`), McMaster's Product
  Information API (an approved customer's certificate: `api-fetch`), or a file downloaded
  by hand (`cad import`); never committed. The bearings' McMaster offers point at the
  trade-number listings, unpriced, until such a fetch confirms a part number.
- **Where:** `spiderpig/roles.py`, `spiderpig/fabmethods.py`, `spiderpig/hardware/cad.py`,
  `spiderpig/construction/joints.py`, `spiderpig/construction/pivots/ball.py`,
  `spiderpig/hardware/bearing_catalog.py`, `spiderpig/tools/cad.py`.

### 2026-10-11: arms: ranged inputs and a tour (the robot-arm workstream, phase 2)

- **What:** a third linkage kind, `arm`: no crank, each input turning a link about a frame
  pivot over a range. Its template's one time is a closed serpentine tour over the
  inputs' box (64 rows), sampled four times as densely as a crank's cycle; the planner's
  bound adds each point's move between rows. The palletizer arm is the first.
- **Why:** every stage samples one cyclic time (the planner's guarantee, `verify_plan`, the
  contract, the bake); a tour keeps them all as they are, and with the rows' bound the
  planner's and `verify_plan`'s guarantees cover the whole box (the contract and the bake
  see the poses they sample), where a second time axis would have meant rewriting each.
- **Where:** `spiderpig/linkage/engine.py` (`Linkage.tour`, `tour_lateral`, `LegSolution.at`),
  `spiderpig/stack/geometry.py`, `spiderpig/stack/topology.py`, `spiderpig/linkages/palletizer.py`,
  `spiderpig/precision.py`.

### 2026-10-11: an arm's drives: belt first, capstan too (user; phase 3)

- **What:** every arm input a NEMA 17 stepper and a transmission, swappable per drive:
  `belt` (GT2 16:80, 5:1, the default: the user's "start with the belt") and `capstan`
  (1 mm Dyneema on a 6 mm sleeve over the motor's shaft and a 30 mm laser-cut drum, 8.7:1).
  The drives on one pivot alternate plates; the motors' shafts are cut to their hubs; a
  belt's motor stands on a slider plate outside its frame plate, clamped from outside, so
  the belt is tensioned once the stack is built (the motor's own screws are inside the
  stack, out of reach); every hub bolts to a pad its driven link grows round the pivot.
- **Why:** two drives on the palletizer's shoulder each take three or four layers beside
  their link, and a stock 24 mm shaft reaches only seven layers from its plate: on one
  plate they can't both fit (one's bolt heads meet the other's pulley). An uncut shaft's
  tip would claim the layers past its pulley (five more on the palletizer). The capstan's
  drum is bounded by the levelling pillar 40 mm from the shoulder.
- **Open (the phase's review, deferred):** `place_motor` sees the pillar heads and the
  other motors, not the links' sweeps, and has no fallback past its angle list; the joints'
  thrust friction is not in the torque budget; no coaxial reduction (planetary,
  strain-wave) yet; the GT2 belt's working tension is not checked against its rating. The
  belt's span stiffness and the cord's EA are unverified figures (measure on the build).
- **Where:** `spiderpig/construction/transmissions/`, `spiderpig/actuators/`,
  `spiderpig/hardware/motion_catalog.py`, the roles `transmission` and `actuator`.


### 2026-10-12: an arm's robot: the turret, its tool, spatial joints, the URDF (phase 4)

- **What:** `BuildConfig.robot` means the whole robot of any kind that makes one
  (`config.ROBOT_LAYOUTS`): a walker's two mirrored sides (`mirrored_pair`, unchanged), an
  arm's side on a turntable (`turret`); an arm builds its robot by default (`--side-only`
  for the side). The turntable is a preset of four parts (role `turntable`:
  `belt_preload` by default, `mt4` the MT4's plain 608 pair, `capstan_preload`); the tool a
  role (`tool`: `suction_cup`, `suction_wrist`, or `none`). Spatial joints are data on the
  mechanism (`kinematics.RobotTree`), which the bake, the URDF and the precision budget
  read. The URDF keeps a spanning tree of the side's pins, a `mimic` where a pin follows
  one input, a `tool0` at the cup's tip, `base_link` on the base's floor; its sidecar maps
  the rest. M3 socket heads of 45 to 70 mm joined the screw table (the hub's screws).
- **Why:** the yaw bears on a fixed 8 mm shaft through two skateboard 608s: stock, cheap,
  McMaster-modelled; the plain pair's tilt (its clearance over the spacing) is the MT4's
  weakness the wave-washer preload removes. The belt and capstan reuse the side's
  transmissions' numbers so a yaw and an elbow are judged alike. The capstan's yaw drum
  radius is 40 mm (11.6:1), not larger: the wraps walk along a sleeve on the motor's 24 mm
  shaft, and a 55 mm radius (15.9:1) would turn only 174 deg. The wrist's NEMA 11 doesn't
  fit beside the end plate inside the stack, so it hangs under the flange (the turntable
  then stands lower for it passing over).
- **The phase's review** (integrated): the capstan drum got its cord anchors (past the
  travel either side of the cord's tangent, notches from the rim); clamping collars (a
  hardened shaft takes no set screw); the homing flag at the turntable's rim, printed
  opaque; a GT2 loop needs 6 teeth in mesh on the small pulley (`belt.MIN_MESH`: the
  side's planar belts too, so the palletizer's motors stand farther out); the yaw motor's
  shaft is no longer cut; the precision budget gained the yaw's homing (repeatability),
  its drive's eccentricity and its axis out of plumb (accuracy: estimates, named in
  `construction.rotary`); the tool's check covers the axles crossing its layers and the
  drives' belts; the assembly text says to drill the pulley centred, bond the bearings on
  the shaft and set the preload with a feeler; the URDF's joints carry each motor's
  effort and a nominal speed.
- **Open:** the shoulder stands high over the base's floor and the palletizer's input box
  reaches little below the shoulder, so the work stands on a platform (or the box grows);
  the frame plates' tipping load on the printed brackets, the base plates' and the
  turntable's stiffness, and the wave washer's force (30 N) are unverified or unmodelled;
  an h6 shaft may hold the upper bearing by interference and defeat the preload (try it);
  the cables loop round the solid shaft (no slip ring, no hollow shaft); the spec and the
  MCP still build an arm's side only (phase 7); the guide puts the turret in one step.
- **Where:** `spiderpig/construction/robot.py` (`LAYOUTS`, `Turret`),
  `spiderpig/construction/rotary/`, `spiderpig/construction/tools.py`,
  `spiderpig/kinematics/`, `spiderpig/urdf.py`, `spiderpig/hardware/robot_catalog.py`.

### 2026-10-12: milled parts on the user's own mill (phase 5)

- **What:** a fab method `milled` and a robot-level role `mill` (`milling.MILLS`: the
  Makera Carvera; `none` by default). A milled part is a `milling.MillPart`: its plate
  stock (`hardware/plate_catalog.py`: 6061-T6 at 1/8, 1/4, 3/8 in, 6 mm acrylic and
  polycarbonate, bought by the 6 x 6 in piece), its outline and 2.5-D operations from one
  side; its solid is drawn from them and `milling.check` holds it to the mill's rules. The
  build writes `mill/` (each part's STEP, `parts.csv`, `SETUP.md`); the BOM buys the plate
  and notes the machine time; milled parts group by their rigid invariants (a housing fitted
  upside down is the same part). With a mill the turntable's hub is two milled bearing
  housings (3/8 in 6061, the seat bored to a slip fit with its shoulder in one piece,
  retaining compound) in place of the laser-cut seats and shoulders.
- **Why:** the user mills aluminium on a Carvera (2026-10-11). A bored seat is concentric
  with its screws and holds a fit a laser-cut stack can't (the phase 4 review's coaxiality
  concern); the rest of the robot stays laser-cut (a flat plate gains nothing milled). The
  capstan drum is not milled: its groove is a side cut, not 2.5-D.
- **The phase's review** (integrated): milled parts group by their specs, not their
  solids' measurements (a moved hole or a mirror image measures the same); a pocket needs
  1.4 x its tool to spiral (the 3.4 mm screw holes are spotted on the mill and drilled on a
  press), a blind pocket a floor, the profile the outline; the seat is drawn at a slip fit's
  middle (+0.01/+0.04, not the bearing's nominal a 1/8 in end mill cuts under); the setup
  sheet says how to hold the plate, rough and finish, try the bearing before the profile
  and deburr; the hub's screws are tightened with both bearings on the shaft (their holes
  alone leave the seats 0.4 mm apart); a mill's plate is refused as any laser sheet.
- **Open:** the removal rates and flute lengths are estimates (the machine's own feeds and
  speeds are the authority); the setup sheet holds no toolpaths (the Carvera's CAM makes
  them from the STEP); a dowel register between the housings would fix their alignment
  without the shaft; `spiderpig export` and the MCP write no `mill/` (they build an arm's
  side only); the walkers never mill (the role is an arm's).
- **Where:** `spiderpig/milling.py`, `spiderpig/fabmethods.py`,
  `spiderpig/hardware/plate_catalog.py`, `spiderpig/construction/rotary/turret.py`
  (`TurretPlan.housing`), `spiderpig/hardware/bom.py` (`milled_stock`).

### 2026-10-12: driving the arm: kinematics, steps, homing, Klipper (phase 6)

- **What:** `spiderpig/control/` and `spiderpig control`: the arm's kinematics in the
  URDF's frame (`ArmModel`: forward from the linkage and the turret, inverse by the
  turntable's bearing and bounded least squares on the planar inputs), each joint's steps
  through its transmission's ratio and its motor's mounting sense (`Axis`), homing (the
  turntable onto its flag's edge from one side; the planar joints by a digital angle gauge
  on their links, since the palletizer's inputs are those links' angles from horizontal;
  then each motor snapped to its nearest full step), a calibration fit of link lengths and
  zeros to measured poses, a Klipper `printer.cfg` (each joint a manual stepper in
  joint degrees, TMC2209s, the endstop, the vacuum's pins) and a Moonraker host that
  streams straight moves as synchronised joint segments.
- **Why:** Klipper on a 3D-printer board is the cheapest stepper controller with UART
  drivers and an HTTP API; it has no arm kinematics, so the host does them and each joint
  moves in its own degrees. The planar joints carry no switch (no hard stop is designed and
  stall homing would hit the arm's own parallelograms); a gauge on a link reads its input
  directly, and the calibration refines it.
- **The phase's review** (integrated): the gauge is read with the motors energised (a
  rotor jumps up to two full steps to its driver's phase when they come on); homing's
  repeatability comes from a recorded reference phase (the driver's `MSCNT` at the
  trigger, as Klipper's `endstop_phase`), not a rounded count; the turntable's search never
  goes farther than the half of its range home is in, slowly; the config never disables the
  motors when idle, reads the phase as `DUMP_TMC STEPPER=q1`, holds with spreadCycle; the
  host's segments run at constant speed (`ACCEL=0`: the joints keep time), its joint state
  changes only once Moonraker takes a move (JSON); calibration says what it can't fix (a
  parallelogram's rod off its opposite side) and applies its offsets to the axes; a side
  alone's frame is the URDF's (right-handed) and its IK refuses a target off its plane.
- **Open:** a board's pins are placeholders (the user's board decides them); each motor's
  sense is the design's and a jog checks it (the phase's sign too); the reference phase is
  recorded by hand; long moves stop and go between segments (Klipper's `GCODE_AXIS`
  coordinated moves, where installed, would not); a hardware repeatability run (a dial
  indicator) is the user's test of the precision budget; switches on the planar joints
  would make homing automatic.
- **Where:** `spiderpig/control/` (`arm.py`, `mapping.py`, `homing.py`, `calibrate.py`,
  `klipper.py`, `host.py`, `cli.py`).

### 2026-10-12: an arm through every surface (phase 7)

- **What:** the spec's `legs.robot` (an arm's only; default the robot) makes the API, the
  MCP, the store and `spiderpig view` build an arm's turret, as `spiderpig build` does;
  `api.export` writes `urdf` (the URDF, its meshes, the joint map and a MuJoCo model of the
  same tree: a hinge per joint, the loops closed by `connect` equalities at their pins, a
  position actuator per motor) and `mill`; the viewer poses an arm by joint sliders
  (`/api/pose/{mode}`, `spiderpig.pose`); `/api/linkages` gives an arm's inputs and
  ranges; the guide's arm steps name drives, not a crank or a servo; the gate's designs
  include `palletizer_turret`, and a new baseline regenerates `DESIGNS.md`.
- **Why:** the workstream's last phase: every surface that took a walker takes an arm
  (the user's direction, 2026-10-11).
- **The phase's review** (integrated): a stored arm loads as the robot it resolved to
  (`legs.robot`; one stored before it, a side, stays one, `derive` too); the MuJoCo model's
  joints carry their motors' rotor inertia through the reduction (`armature`) and each
  position actuator is no stiffer than its inertia holds at the timestep (the wrist's
  diverged), the loops' `connect`s stiffer; a pose answers in milliseconds (the template
  and the glb's nodes cached) and a walker is refused before any bake; the sliders start at
  the clip's first frame (`inputs0`), capture it before anything awaits, and keep one
  request in flight; `mill` and `mjcf` say why they skip; the gate's `compare` skips the
  designs its baseline lacks.
- **Open:** the walkers' MJCF and walk model are a walker's (an arm's MuJoCo model is the
  URDF's sibling, not `spiderpig sim`'s; the steppers' rotor inertias are typical
  datasheet figures, unverified); the viewer's precision overlay (the error budget
  over the workspace) is not drawn yet; the joint sliders were checked by their tests (the
  server's poses against the bake's own frames, the viewer's composition in vitest), not in
  a browser here.
- **Where:** `spiderpig/spec.py` (`LegsSpec.robot`), `spiderpig/stages/resolve.py`,
  `spiderpig/api/exports.py`, `spiderpig/urdf.py` (`mjcf`), `spiderpig/pose.py`,
  `spiderpig/server/app.py`, `viewer/src/arm.ts`, `tests/gate/identity_gate.py`.

### 2026-10-12: an arm's pivots default to ball bearings (user)

- **Decision:** an arm's pins and pillars default to `ball` / `ball_pillar` (an MR84ZZ in
  every rider on the Chicago barrel, an MR106ZZ in every link on the standoff), its links
  to a 6.6 mm radius (the MR106ZZ seat plus `min_wall`); walkers and mechanisms keep the
  Chicago screw and the standoff. The defaults are data per kind
  (`config.DEFAULT_PIVOTS`, `config.KIND_FIT`), read like the crank's
  (`default_pin`, `default_pillar`, `default_params`); a config, spec or CLI naming a pin,
  pillar or fit keeps it, and a design moved to another kind gets that kind's.
- **Why:** the user stepped up to bearings for the palletizer (2026-10-11) and expected
  them by default; they had been built only as a swappable option, so the arm's budget
  was quoted on plain holes, whose play the arm's levers (gain ~9 at O and E) make
  millimetres at the tool.
- **Open:** the budget carries each joint's radial play only; a joint's tilt (a single
  3 mm bearing's, ~1.5 deg free; a plain hole's) does not reach the tool
  (`precision.joint_clearances`). The bore-on-shaft gaps (0.05 / 0.03 mm) are assumed.
- **Where:** `spiderpig/config.py`, `spiderpig/roles.py`, `spiderpig/stages/resolve.py`.

### 2026-10-12: tilt in the budget; preloaded ball pairs (round 1 of the arm's precision review)

- **Decision:** the error budget counts each link's tilt and end float out of the plane
  (`precision.out_of_plane`: a linear programme over the bodies' rise and tilts, the loops
  counted, each column's float shared by its links; added to the in-plane total in
  quadrature), from the fabricated side's wobble notes. New pivots `ball_pair` /
  `ball_pair_pillar` (`construction/pivots/pair.py`): each rider's bearing paired with an
  open one in a hub laser-cut from the link's sheet and bonded to its face, in the spacer
  layer beside it (the column's ring replaced; every spacer layer claims the hub's radius);
  the column clamped through the inner rings by a DIN 2093 disc spring, so no end float. A
  link with no free neighbour layer keeps one preloaded bearing.
- **Why:** the user asked for iterative fixes against the field, weakest link first. With
  tilt counted, the ball pivots' links (about 1.4 deg each) put the tool 10.9 mm out of the
  plane, the largest term by far. Thick (6.35 mm) aluminium links with the pair inside were
  tried first: the planner padded the stack to 54 layers (a thick link thickens its layer,
  the drives' shafts stop reaching their hubs, and a rejected leaf teaches the search
  nothing), so the pair went beside the link instead, as the MT4 puts thickness at the joint.
- **Result (palletizer, belts):** out of the plane 10.85 -> 0.70 mm; 7 of 10 riders paired
  (R, U and W single: their neighbour layers are taken); the plan unchanged in size.
- **Open:** the pair's 0.02 deg tilt and the bore-on-shaft gaps are estimates; the rings'
  inner-ring land is drawn as the plain ring; the planner can't yet place a link to give it a
  hub, nor plan thick links well (`plan_z` thickness at search time, nogoods on leaf
  rejections).
- **Where:** `spiderpig/precision.py`, `spiderpig/construction/pivots/pair.py`,
  `spiderpig/hardware/bearing_catalog.py`, `spiderpig/hardware/fastener_catalog.py`.

### 2026-10-12: ground shafts under the ball pairs (round 2 of the arm's precision review)

- **Decision:** `ball_pair` runs on a ground 4 mm g6 pin cut to length (`pair.GroundShaft`,
  0.5 mm steps, an M2 socket head and a 2.2 x 5 washer at each end, the disc spring under
  the top one, the host bonded to it as to a Chicago barrel), and `ball_pair_pillar` always
  on the one-piece 6 mm steel shaft, ordered ground: the bearings' bore-on-shaft gap 12 um
  at worst (`pair.GROUND_FIT`) where the Chicago barrel's was taken at 50 um and the goBILDA
  standoff's at 30.
- **Why:** with the tilt fixed, the in-plane joint play (1.25 mm) was the largest class,
  every pin's gap multiplied by the elbow-drive parallelogram's gains (9 at O and E, 7 at C
  and R). No precise arm runs bearings on a screw's barrel: the MT4's pairs ride 6 mm
  shafts with M4-tapped ends.
- **Result (palletizer, belts):** in-plane joint play 1.25 -> 0.39 mm; repeatability 2.37
  -> 1.56 mm. The round-1 docstring's claim that a single bearing is preloaded was wrong
  (rings on the inner ring only pass the spring's load inner ring to inner ring); its
  numbers already counted it unloaded.
- **Open:** the ground pin's M2-tapped ends on a 4 mm shaft and its price are unverified at
  MISUMI; the disc spring's setting (0.25 mm of travel, set by 0.1 mm shims) is coarse; the
  pairs' 0.02 deg against the pins' couples from `sim/loads.py` is unchecked.
- **The hub rule (round 3's first finding):** which riders got a hub was the layering's
  luck (with capstans, U, V, E and W lost theirs: out of the plane 1.45 mm). The pair
  pivots now hold the planner to it (`pair.hub_layers` through the axle's `column` hook,
  which `construction.axle` now passes the column's own links, each layer's room and the
  spacer radius): every rider needs a free spacer layer beside it with the hub's room, one
  hub a layer, the lower side first, as `PairSeat.hubs` builds them. Belts: 18 layers,
  repeatability 1.41 mm; capstans and the preloaded capstan turntable: 22 layers, 0.50 mm.

### 2026-10-12: an inclinometer on each arm link (round 3 of the arm's precision review)

- **Decision:** a role `angle_sensor` (per drive; `actuators.sensors.ANGLE_SENSORS`): `gauge`
  (the default: a digital angle gauge laid on each link by hand at homing, 0.05 deg) or
  `scl3300` (a Murata SCL3300-D01 inclinometer bonded to the link, its two in-plane axes'
  `atan2` averaged half a second at rest, 0.02 deg over a few kelvin of offset drift). The
  budget now counts each planar input's homing (`InputBudget.homing_mm`: the sensor's
  angle through the input's lever), which it had left out; the drive buys the sensor; the
  host reads it over SPI (`control.incline`: the datasheet's frames and CRC, the off-frame
  protocol), and `control.homing.plan` reads the inclinometers instead of the gauge and
  skips the phase snap on a capstan turntable (its cord may slip against the motor).
- **Why:** with the pivots and drives fixed, the review found the largest real term
  outside the budget: the gauge's zero at every power-up, 0.11-0.26 mm per joint through
  the 130 / 150 mm levers. The Dobot Magician reads its arms' angles from accelerometers in
  the links; an AS5048 can't sit on the shoulder's pillar (two coaxial inputs, no shaft
  end). Read at rest before each final approach, the sensors also take out a capstan's
  creep and the drives' sag.
- **Open (the review's):** printed rings in the preload path creep under 35 N (metal lands
  at E, O, C and R); a g6 shaft can be 4 um tight in a bore (check each bearing slides);
  Dyneema SK creeps (DM20 or Vectran); the C8 spring's seats; the inclinometer's mounting
  offset and drift are datasheet figures; no SPI driver in Klipper (the host reads it).
- **Revised the same day (user):** "we shouldn't need closed-loop control": the arms the
  design copies home each joint to a switch. The MT4's STEP names an optical sensor plate on
  the base and a J2 detection flag on the upper arm; Thor, PAROL6, the AR4 and the Mirobot
  home every joint to an optical, micro or inductive switch (fast seek, back off, slow
  approach); only the Dobot reads its arms' angles (gravity-referenced sensors). The
  recommended homing is `angle_sensor` `endstop` (a slotted optical endstop on the frame, a
  flag on the link 40 mm out: 20 um there, 0.065 / 0.075 mm at the tool through q2's and
  q3's levers; `control.homing.plan` seeks each joint, backs off and comes in slowly); the
  inclinometer stays an option. Best configuration: 0.62 mm worst case with endstops
  (0.58 with inclinometers). Open: the q2 / q3 endstops and flags are in the budget, the
  BOM and the homing, not yet drawn (their place on the frame plate, the flag's claim on
  the link).

### 2026-10-12: the links' bending in the budget; aluminium links (round 4 of the arm's precision review)

- **Decision:** the budget gains the links' bending (`linkage.stiffness`: a 3D frame of each
  link's section at its layer, the pins and pillars steel rods between the layers and the
  frame plates, the joints free only in their turn; checked against the cantilever's
  formulas and a two-layer twist by hand: `tests/test_stiffness.py`). The recommended arm
  cuts its links (and its pairs' hubs) from 0.125 in 6061-T6 (`--sheet al6061_3p2mm`)
  instead of 3 mm acrylic. Parts sources: VXB bearings, Lesjöfors disc springs (DIN 2093's
  loads checked: the C12.5 is 151 N, not 115), MISUMI SFJW ground pins (4 mm from 20 mm
  long), DM20 cord for the capstans; the bearings' free models pinned and fetched by
  `spiderpig cad download`.
- **Why:** the round's two reviews. The design review: every reference arm has stiff links
  (the MT4's 5 x 20 mm bars, about 125 times our 3 x 13 mm acrylic in the plane), none a
  counterbalance spring. The error-model review: the linear gains check against a
  nonlinear re-solve (0.2 %); gravity seats every pin one way over the whole box, so the
  pins' play is mostly accuracy (ISO 9283 RP by Monte Carlo 0.05-0.1 mm within a session,
  0.10-0.15 across power-ups); the links' compliance and creep was the term the budget
  lacked. With the layers' offsets a load crossing between layers twists the thin links:
  acrylic sags the tool 5.5 mm under 50 g (a 5 g slide picked up: 0.55 mm), 39 mm/N across
  the plane; 6061: 0.22 mm (0.022 mm a slide), 1.6 mm/N; the thermal row 0.21 -> 0.07 mm.
- **Open:** the total still adds the pins' worst play as repeatability (round 5: the
  ISO 9283 split); the frame plates and the turntable are rigid in the stiffness model;
  acrylic's creep is not modelled (moot with aluminium).

### 2026-10-12: ISO 9283 repeatability; vision homing to a reference sheet (round 5 of the arm's precision review)

- **Decision:** the budget reports ISO 9283 pose repeatability by Monte Carlo
  (`spiderpig.repeatability`: 30 cycles to each of five poses, mean + 3 sigma; gravity
  seats the pins by a static solve; a preloaded pair's bore gap frozen by its clamp's
  friction unless its load with the moves' accelerations passes `SLIP_N`; backlash and the
  friction dead band only where gravity doesn't hold the drive; the pairs' tilt by its
  hysteresis; across power-ups each cycle re-homed), within a session and across
  power-ups, beside the worst-case bound. The stiffness model takes each link's outline
  segments and the pairs' moment stiffness (a Hertz estimate) and reports the sag in and
  across the plane and the sway's frequency. Homing's fine zero comes from a camera on the
  tool sighting dots on a reference sheet (`homing_reference` `vision_fiducial`,
  `control.vision`: a sub-pixel dot centroid, least squares of the zeros over two
  sightings or more): the switches only coarse (the TCST2103 boards). An Omron EE-SX67
  homing option (`ee_sx67`, the `capstan_precise` turntable, 1/64 microsteps) stays
  available.
- **Why:** the round's reviews: the worst-case sum is no maker's number (ISO RP is), and
  the remaining across-power-up term was homing (0.23 mm with 20 um switches at 40-62 mm
  flags). The user: "vision based homing to a fiducial target with a reference sheet, we
  don't need super accurate switch based homing": a camera's 10 um at the tip beats any
  switch through a lever, and re-sighting also takes out the links' creep and the frame's
  heat. Within a session the recommended arm (6061 links) gives 0.097 mm ISO RP.
- **Open:** the tool can't reach the table (its tip stays 112 mm above it in today's box:
  round 6); the camera's 10 um is an estimate; the stack order sets the links' stiffness
  about 2x (not a planner objective yet).

### 2026-10-12: the band and the reaching geometry; the ISO model's audit (round 6 of the arm's precision review)

- **Decision:** a linkage may hold a relative limit, `Linkage.band` (degrees of its first
  input less its second, as the MT4's elbow stops): `Linkage.inputs_at` maps the box, the
  tour, its rows' spacing and the ISO poses into it, `in_band` says whether inputs are in
  it, and the inverse kinematics refuses a pose outside it. A second palletizer,
  `palletizer_reach`, holds the review's geometry: the elbow crank 90 mm, the inputs' box
  q2 -10..110 and q3 -110..10 within q2 - q3 30..150, the levelling pivot P 40 mm in front
  at -40 deg and the wrist corner U at 40 deg: every parallelogram 30 deg or more off its
  dead point (conditioning 226), the elbow drive's gains 9.1 / 7.3 -> 6.0 / 3.3. The ISO
  model's audit: a slipped bore gap lands the same way every same-direction cycle (not in
  RP; a power-off drop re-seats the gaps that never slip); the turntable's backlash and
  dead band count (no gravity about the vertical); across power-ups the microsteps'
  nonlinearity unless homing snaps to the driver's phase (with the camera, even a
  capstan's: a slip is a cycle's jump it sees). The vision reference's camera is the
  Camera Module 3 Wide (focus from 5 cm), its sheet an FR-4 board of copper dots (paper
  swells with the air's humidity), and the sightings may fit each joint's gain (a capstan's
  ratio drifting with its motor's heat).
- **Why:** the user's goal is slides on a table: the base palletizer's tip never gets
  under 112 mm over it (the turret puts the turntable under every link point and the
  q2 / q3 motors hang 74 mm under the shoulder). The MT4 reaches down from a 140 mm
  shoulder; its URDF's J2 / J3 limits are a rectangle, its relative limit in its stops.
- **Result (recommended: `ball_pair` pivots, 6061 links, capstans, the preloaded capstan
  turntable, endstops for the coarse find, `vision_fiducial`):** ISO RP 0.042 mm within a
  session, 0.048 mm across power-ups (bound 0.44 mm); the links' bending 0.023 mm a slide
  picked up, 2.3 mm/N across the plane (about 11 Hz).
- **Open (the next step):** the turret's keep-out: the arm kept out of the base's cylinder
  instead of the turntable put under it, the q2 / q3 motors behind the shoulder (an arm's
  motor angles), the base trimmed in front and bolted to one work plate that also holds
  the reference board: then `palletizer_reach` covers a 168 mm square at table height; the
  stack's order as a stiffness objective; the turntable's own stiffness; the hose's drag.

### 2026-10-12: belts back, the moves ended one-sidedly (the user's)

- **Decision:** the recommended arm drives its joints by the stock GT2 belts (5:1, the
  default transmission and `belt_preload` turntable), not the capstans: the host ends a
  move one-sidedly (`control.host.one_sided`, `ArmHost.move_to(..., approach=True)`: a
  joint whose last motion was negative goes `APPROACH_DEG` past and comes back), so every
  stop takes up a drive's backlash the same way, and the ISO model leaves backlash out of
  RP (`repeatability.iso_rp(one_sided=True)`), reporting it in `Rp.any_direction_mm`.
- **Why:** the user: "custom capstan drums are just dumb and should have been cut anyway"
  (16 aluminium discs, $240 of the $500 of cutting). The MT4 runs belts at every joint and
  quotes 0.1 mm; the turntable's belt backlash (0.45 mm at the tool), which no gravity
  holds, was the only reason belts scored 0.31 mm; a one-sided approach costs nothing.
- **Result:** belts: ISO RP 0.042 mm within a session, 0.048 across power-ups (as the
  capstans'), 0.31 mm from any direction without the one-sided approach; the BOM $639
  listed (cutting $305) against $835.

### 2026-10-12: stock parts and acrylic plates (the user's simplification, rounds 1-2)

- **Decision:** the user's constraints: only stock parts but the cut plates, the plates
  mostly acrylic, fewer parts, 0.1 mm kept. No gearboxes on the arm's joints (a cheap
  planetary's output bearing as the shoulder pivot costs 0.2-0.4 mm: Dobot's 0.2 against
  the belt-driven MT4's 0.1; on the turntable its 15' is 2.4x the belt's): the stock GT2
  belts stay. The arm's long links (b1, b4) in 0.125 in 6061, the six short ones in
  SendCutSend's 5.4 mm cast acrylic (`acrylic_5p4mm`), each seating its bearing pair inside
  (`PairSeat.in_link`: no hub; the planner's hub rule skips them). The q2 / q3 belt's
  small pulley's hub stands in the frame plate's window (`BeltDrive.motor_hub`: a layer
  less in the stack, so the 24 mm shaft reaches past a thick link); the drive's reach is
  checked at search time against the thick links placed (`nominal`). The pair pivots'
  printed rings are cut from the default sheet, their spacers stock DIN 988 shim stacks,
  and an inner-ring land shim (4 x 6 / 6 x 8 x 0.5) each side of every rider
  (`pair.stock_spacers`). The precision report reads each link's own sheet (its
  stiffness, weight and expansion).
- **Result (`palletizer_reach`, mixed links, belts, vision homing):** ISO RP 0.037 mm in a
  session, 0.061 across power-ups; a slide picked up 0.048 mm; 3.4 mm/N across the plane
  (~9 Hz); the BOM $684 listed, 68 cut and 7 printed parts (from 41-50 printed).
- **Open (round 3):** the 20 aluminium frame / base plates ($241), the 7 printed parts,
  the ~240 bought pieces (the shims), the ground pins' M2 ends.
- **Round 3:** the column's rings were one default-sheet layer tall wherever they stood
  (`common.ring_z`), so in a layer a 5.4 mm link thickens they left a 2.4 mm gap and the
  disc spring's preload stopped at it (pins E, R, U, V, both pillars; on O the drive
  pulleys' 8 mm bores held nothing). The pair pivots' columns are now stock precision
  collars cut to length, one per gap between neighbouring links, hubs and (a pillar's)
  frame plates and end shims (`pair.stock_tubes`: 5.5 x 4.1 on the pins, 7 x 6.1 on the
  pillars, which passes the pulleys' bores), their OD on the bearings' inner rings, so the
  land shims go. Not done (the turret: it cuts its plates from the frame sheet): the base's
  bottom plate in acrylic, the frame plates' feet screwed through T-slot nuts in place of
  the printed brackets, the turntable's rim as the homing flag, the yaw hub as two flanged
  F608 bearings in cut plates; shoulder screws in place of the ground pins were rejected
  (ISO 7379's 3 mm head and its disc spring don't fit a 3 mm end layer; f9 looser than g6).

### 2026-10-12: the turret in the base, the MT4's way (round 7: the turret's simplification)

- **Why:** the user's "go on to the turret next, same style": stock parts but for cut
  plates, fewer parts, 0.1 mm kept, the tool reaching the table. The classic turret
  (`deep_groove_pair` / `preloaded_pair`) turns a hub on a fixed shaft and stands the
  turntable under every link over the inputs' box: on `palletizer_reach` the shoulder
  stood 329 mm over the floor, on 20 cut 5052 plates and 7 printed parts.
- **Chosen (of the variants reviewed: C, with the reviewer's corrections):** the support
  `base_pair`, preset `mt4_base` (`construction/rotary/base_pair.py`): two F608ZZ retained
  in the base's two 1/8 in 5052 plates, 48 mm apart (the hub's were 25), on four 45 mm hex
  M4 standoffs; the 8 mm shaft turns in them (the upper inner ring clamped by two collars,
  the lower floating under a wave washer), the 80-tooth plate pulley on it, a spacer disc
  and the turntable clamped down by an M4 in the shaft's tapped end (a configured Misumi
  shaft; a hand-pressed collar clamps too lightly); the side's feet on stock steel angle
  brackets; an M2 screw hanging from the turntable through a slot endstop just past the
  yaw's lower end (one-way homing, the MT4's; the camera fixes the zero); the base screwed
  to the bench on 6 mm spacers. Only the links over the base's keep-out (its 84 mm front,
  the tail round the motor) must clear the turntable (`turret._lows`' ``keep``), and
  `palletizer_reach`'s q3 stops at -90 deg (from -110): past it the folded elbow only
  brought the tool down beside the base. The yaw's range is +-120 deg (the tail).
- **Reviewer's corrections taken (round 7's second review):** screwed down, not on
  rubber feet (a base this light rocks under the arm's moment); the tapped-shaft clamp;
  the spacer out to the small pulley; the flag 1 mm over the slot's floor, the board's
  holes slotted; retaining compound in place of epoxy, the columns tightened first;
  `Support.row_share` (the fixed upper row keeps the washer's force less the arm's weight:
  2.9 N·m held, not 5.8; a 60-80 N spring would double it); the columns' racking a lower
  bound (`base_pair.racking`, in `turret_budget`'s notes: accuracy, not repeatability).
- **Not done:** the bottom plate as the work plate in one piece (about 400 mm, past the
  300 mm blank; the camera re-references the sheet at every power-up); a stronger spring
  (none verified in stock).
- **Engine:** a stored plan is re-made for designs differing only in robot roles that
  don't shape the side (`design.SIDE_FREE_ROLES`, `design.side_hash`,
  `store.find_plans(side=True)`): a new turntable no longer re-plans the side, which on
  `palletizer_reach`'s mixed links runs out of the 60000-step budget.
- **Result (`palletizer_reach`, the round-3 arm on `mt4_base`):** the shoulder 170 mm over
  the bench (329); the tool gets down to the bench from 161 to 278 mm out, a 117 mm band:
  the 130 / 150 mm links bound it, not the turret (any q3 stop gives 115-125 mm; round 6's
  168 mm square was wrong); ISO RP 0.025 mm in a session, 0.034 across power-ups (with the
  q3 stop, either turret); the turntable's homing 0.082 mm at the tool (0.099); the BOM
  $628 listed ($659 on the classic turret: the three base plates in 1/8 in 5052, $129, as
  stiff as 6061 at half the rate), 18 cut parts (30) and 1 printed (7: the tool's holder
  left), 220 bought pieces (228).

### 2026-10-12: honest prices for the arm (cost-reduction round 1)

- **Decision:** the arm's BOM prices every part from a cheap current source (Amazon search
  results, Bolt Depot's and 123Bearing's pages, read 2026-10-12), the source first so the
  BOM prices it: generic 10-packs of the miniature bearings ahead of VXB
  (`bearing_catalog.AMAZON`); the steppers, pulleys and belt loops (`motion_catalog.AMAZON`);
  the pump, valve, cup, tube, fitting, endstops, collars, wave washer and spiral wrap
  (`robot_catalog`); Bolt Depot's singles for the M2 / M3 / M4 screws, M2 nut and washers
  and the M4 washer (`sources._BOLT_DEPOT`; no 14 or 18 mm M3 socket head there); one shim
  kit for the 4 x 8 DIN 988 and 8 x 12 shims (the rows share its pack); 123Bearing's disc
  springs; the tapped 8 mm shaft at an estimated USD 10. Kept: the MISUMI ground pins and
  precision collars (an h8 rod fails the 0.1 mm budget; stainless capillary tube is listed
  as an alternative only), the F608ZZ, the Pi Camera 3 Wide (an OV5647 M12 board the
  alternative). The cutting is refitted (`bom.cut_estimate`): per part a fixed charge plus
  a rate per cm² at one off, times SendCutSend's quantity break (`bom.qty_factor`), a
  file's quantity (`bom.same_files`), the rates per sheet (`sheet_catalog.CUT_RATES`,
  `parts.CUT_ACRYLIC`).
- **Why:** the old cut model (2026-10-08, fitted on the Strider's small parts at 2-8 off)
  quoted the arm's 18 parts at USD 229 against SendCutSend's live 163 (its 15-23 cm plates
  2x high, the small 5.4 mm links 2x low); 29 of the arm's rows were unpriced, so its total
  left them out. The new fit: the arm 163.86 against 163.01, the Strider's acrylic 48.72
  against 49.44 and its aluminium 61.69 against 67.70 (its hole-dense plates: the area
  doesn't see the cut length, single aluminium parts 20-25 % rms).
- **Result:** the arm (`palletizer_reach` on `mt4_base`) USD 598.29, purchases 434.43 and
  cutting 163.86 (was 627.65 with 29 rows unpriced: 398.69 + 228.96); unpriced now the M3 x
  14 and x 18 socket heads and the retaining compound.
- **Open:** a cut-length term (5052's residuals fall to about 10 % with it); the Amazon
  listings unverified in a browser; Bolt Depot's and 123Bearing's shipping.

### 2026-10-12: cost-reduction rounds 2-4 (the user's "keep reviewing until it's under $300")

- **Round 2 (taken):** the base_pair turret's bottom plate in 3 mm acrylic
  (`base_pair.BOTTOM_SHEET`, the bearing epoxied in), its turntable lifted on shim stacks
  at its four ties and a stack on the shaft (`LIFT_WASHER`, `CENTRE_SHIM`: no cut spacer
  disc), three more hex columns round the bearings (`CENTRE_R`: the preload's loop closed
  by the axis); a cheaper homing camera (`vision_fiducial_ov5647`, the OV5647 M12 board,
  15 um budgeted); the pins' spacers cut from stainless capillary tube.
- **Round 3 (correctness, from its reviews):** the top plate back in 1/8 in 5052
  (`TOP_SHEET`: on acrylic the seven columns racked 6-25x more than modelled, the plates
  being rigid in the precision model), M4 washers under the screws on acrylic,
  `base_pair.racking` with its real column count and ends; the shim kit's DIN 988 shims
  (not DIN 125 washers, whose 0.45-0.55 mm tilted the turntable); 18 mm collars (the
  cheap 25 mm ones clashed); screws only of a stocked length. **The ball pair pillars' shaft
  was the MISUMI NETRF6 standoff, 6 mm 0 / -0.1, not ground**, though the model took it as
  g6: with that play across power-ups would reach about 0.14 mm. It is now a ground g6
  linear shaft tapped M3 both ends in whole mm (`StandoffAxle.ground`,
  `crank_catalog.ground_pillar_shaft`, the MISUMI SFJW family; part number and price
  estimates); the pillars' spacers K&S brass tube.
- **A correction to the turret's round 7:** its 0.025 / 0.034 mm were computed without the
  side's wobble notes (the pairs' tilt hysteresis, the clamped gaps' re-seating left out);
  with them the arm repeats to 0.037 mm in a session and 0.061-0.065 across power-ups.
- **Round 4 (a study, nothing built):** no build option reaches USD 300 at 0.1 mm. Every
  pivot but `ball_pair` repeats to about 1.3 mm (`ball` / `ball_pillar` on 63 / 43 um of
  clearance, W's load reversing over the tour); without the camera across power-ups is
  0.159 mm; the cheapest arm the options make (chicago pins, 3 mm acrylic links and a 2 mm
  acrylic frame, the `mt4` turntable, switches) is about USD 350 with its cutting priced, at
  about 1.4 mm. A per-joint pin override (single bearings on the levelling rods) saved USD
  16 and was dropped. `--pin ball` with mixed sheet thicknesses plans 32 layers (twelve of
  spacers only): the planner's budget, not the pins (a million steps found none thinner).
- **Result (`palletizer_reach`, `mt4_base`, `vision_fiducial_ov5647`, mixed links):** the
  BOM USD 525.32 with every line priced (purchases 389.50, cutting 135.82), from USD 628
  with 29 lines unpriced (about 820 priced); ISO RP 0.037 / 0.065 mm. Not in it: the
  controller (an SKR Pico, a Pi Zero 2 W, a 12 V supply, about USD 65-110), shipping from
  about ten vendors (USD 30-60), the MISUMI prices (estimates). The rest of the way to 300
  is the user's own work: the plates cut on the Carvera from bought sheet (about USD 70 for
  136 of cutting) and the pins and pillars cut and tapped from McMaster's -0.005 mm rods
  (1327K507, 4143N12; about USD 40 off): some USD 420.

### 2026-10-13: "the best part is no part" (the arm's part-count loops 1-2)

- **Why:** the user's: cross-sections of the pivots showed the spacer tubes, hubs and shims
  only fill layers between a pin's links; "do we even need them?". Then loops of
  investigators (pivots; turret and plates; drives, tool and homing; electronics) and a
  reviewer combining them.
- **The planner:** a role `pin_stack` (`stack.plan.PIN_STACKS`: `free`, `adjacent`, `near`
  one layer at most; `StackSpec.adjacent`, `_Search.together`, `fabricate.adjacent_pins`, and
  `plan_z.finalize` refusing a clearance gap between a pin's neighbours but a column's own
  land washers). On `palletizer_reach` strict adjacency is infeasible (both drives' hubs
  hug opposite frame plates while the elbow and levelling parallelograms chain b1-b4-b3-b2
  and cross b5-b8: every pin strict, or all but one, fails in ~340 steps); `near` planned
  18 layers before the land gaps and none after; `free` now plans 16 (60.5 mm).
- **Taken:** b1 in 5.4 mm acrylic (its bearing pair inside it: both hubs gone); sensorless
  homing (`angle_sensor` `stall`, the turntable preset `mt4_base_stall`, sensor
  `hard_stop`: an M3 under the turntable stalls on a post past the range's end,
  `base_pair.stop_radius`; the camera sets every zero; the three optical endstops, their
  flag and posts gone, the base smaller); the suction tool's printed holder as two cut
  plates that also mount the camera at a fixed offset (no printed part left); a cheaper
  pump; the Pi Zero's CSI cable; the controller in the BOM (role `controller`,
  `actuators.controllers`: SKR Pico + Pi Zero 2 W + 12 V brick + cables, about USD 115;
  `control.klipper` generates its pins and StallGuard homing); MISUMI's whole-mm ground pin
  lengths (`GROUND_LENGTHS`: SFJW4 from 10 mm in 1 mm steps); b4 in 1/8 in 5052.
- **Bugs found and fixed:** the pins' top spacer bore on the top bearing's outer ring (no
  inner-ring preload, a still washer rubbing a turning ring): a 4 x 6 x 0.5 inner-ring land
  (`ChicagoShaft.land_key`); neighbours meeting face to face got a 0.5 mm land gap
  (`pair.land_gaps`); M2 ISO 7380 button-head pin ends (`fasteners` family `button`) so
  the end still sinks into a 3 mm layer; the disc spring's slot was also bought as a rigid
  0.5 mm washer, so the screw flattened the spring (`pair.spring_slot`).
- **Rejected or deferred:** T-slot feet (a captive M3 nut needs the plate as thick as its
  5.5 mm width: the frame plates are 2 mm); one screw per pin (a bond's creep would drop
  the preload silently); single bearings at O/b1, W/b8, E/b6 (power-up RP toward 0.085 mm;
  needs a per-joint seat key: deferred); bent-tab feet and a bent tool L (SendCutSend's
  bend fee may eat the USD 16: a live quote first); smaller motor sliders (USD 2).
- **Result (`palletizer_reach`, `--pin-stack free`, `mt4_base_stall`, `stall`,
  `vision_fiducial_ov5647`, b1-b8 but b4 in 5.4 mm acrylic, b4 1/8 in 5052):** BOM USD
  630.13 with the electronics (about 515 without them, was 524.58), 287 bought pieces, 17
  cut parts, none printed; 16 layers (60.5 mm); ISO RP 0.047 mm in a session, 0.075
  across power-ups.

### 2026-10-14: a smaller base, for looks (the user's "make the plate smaller", turns 1-3, then 4-5)

- **Ask:** a more elegant machine, not a cheaper one. The `mt4_base_stall` base was a
  174 x 144 mm D-shaped plate with a tail for the yaw motor, and the turntable sat
  inside its outline.
- **Turns 1-3 (shown, then dropped):** a round base wrapped in a black acrylic tube
  (Ø142). The user's verdict was that this made the base bigger, not smaller.
- **Kept from them:** the preset `mt4_compact` (`construction.rotary.PRESETS`):
  - The yaw through 16 to 60 teeth on a 158 mm loop (`yaw_transmission("belt_60")`):
    3.75:1, with the motor at 38.4 mm from the axis (64.8 mm on the 80-tooth loop) and at
    least `YAW_MIN_MESH` teeth in mesh.
  - The turntable flush with the base's round core.
  - A symmetric bracket pair.
- **Turns 4-5 (the user's "a tight circle for the rotating part with the motor extending
  out the back is fine"):**
  - The plates are a tight round core plus a lobe round the motor
    (`base_pair.lobe`, cut as one hull of arcs, `shapes.hull`).
  - The core's radius comes from the brackets' real far corners (`base_pair.keep_radius`),
    not the feet's ends, with feet as short as the brackets' pair allows (`BASE_FOOT_L`).
  - Turn 5 uses 16 mm brackets in place of 20 mm ones (`angle_bracket_16`; its width,
    thickness and holes UNVERIFIED from a listing).
  - The tail-swing check is back for the lobe.
- **Cost:** the 60-tooth pulley and the 158 mm loop are priced as estimates.
  `palletizer_turret` keeps `belt_preload` (the gate is identical).

### 2026-10-14: the approved arm named and gated (merge hardening, round 1)

- **The arm by name:** `--linkage palletizer_reach` is the user's approved arm.
  - It gets its pivots from `config.LINKAGE_PIVOTS` (`ball_pair`, `ball_pair_pillar`).
  - It gets its roles from `config.LINKAGE_ROLES`, read by `roles.default_key`: `mt4_compact`, stall homing and the camera.
  - It gets its links' sheets from `materials.LINK_SHEETS`.
  - It was a script's flags before. The identity gate now builds it as `palletizer_reach`.
- **A clash the builds didn't check:** on `mt4_compact`, a column round the bearings touched the hard stop's post screw (0.115 mm³, found by the robot clash test).
  - The fix: `base_pair.columns_xy` now turns a column in 5° steps past the motor and the post.
- **Declared, not inferred:**
  - A transmission now declares `family`, `reverses` and `slips`. Before, `key == "belt"` / `startswith("belt")` and `"capstan" in source` disagreed, and on a classic hub `belt_60` would have taken the capstan's branch.
  - The bracket's dimensions are read from its catalog item (`base_pair.stock_bracket`).
- **Bugs:** `verify.cost_floor` priced a servo and a crank on an arm and raised `ConstructionError`. It now prices the arm's steppers.
- **Hull checks:** `shapes.hull` refuses clockwise, nested or coincident circles.
- **Catalog:** the unused `angle_bracket_20` and `rubber_feet_12mm` are gone.
- **The approved arm's cut rules (the gate's first audit of it):**
  - `b4` in 1/8 in 5052 kept 2.5 mm round its MR84 seats, under SendCutSend's 1 x thickness. An aluminium link (not acrylic: its rule is a 1.35 mm bridge) now grows a boss round each bearing seat its width can't hold (`plates.seat_bosses`, as `rider_bosses` does round a crank bore): 3.27 mm, a warning only.
  - The base's top plate cut the yaw motor's boss clearance as a rectangle, leaving 2.6 mm to the screws' slots. It is now a stadium.
  - The audit is OK, with warnings only.

### 2026-10-14: the walkers learn from the arm (the user's "honest BOM prices" and "fewer printed spacers")

- **Honest prices:**
  - Every Strider line is priced now; there were 5 unpriced, left out of the total.
  - The prices came from sellers' pages read on 2026-10-14. No logins and no bot checks: Amazon, DigiKey, Mouser and MISUMI refused, and they stay as they were.
  - The board's M2.5 nylon hardware is one Adafruit kit (#3299, $16.95, shared pack). Its screws are 6 mm, so `m25_nylon_screw_6` replaces the 5 mm one; the standoff's stud length is unverified.
  - The protection board is diymore's ($4.99). The resistors and DC pigtail come from one ProtoSupplies cart.
  - The battery's Amazon price is no longer marked verified (it came from a search page). The strap is named for what is bought.
- **Fewer printed spacers:**
  - A pivot construction declares its stock spacers (`spacers`, `construction.pivots.common.Stock`). `AxleGroup.realize` applies them as the arm's pivots did: stock tubes cut to length between the links (`stock_tubes`), DIN 988 shims for thin spacers (`stock_spacers`).
  - Chicago pins use the 5.5 x 4.1 tube and 4 x 8 shims; standoff pillars use the 7 x 6.1 brass tube and 6 x 12 shims.
  - The Strider double's printed parts go from 173 to 37: the crank's parts, the foot socks, the deck rail and cradle, and the horn spacer remain.
  - Cost: about $50 of tube and one shim assortment, and about 44 tube pieces to cut by hand.
- **Found on the way:**
  - The pivots placed a link's bearings and tubes from its layer's middle, but links sit on the layer's floor (`pivots.common.link_z`).
  - On the arm, `b4` (3.2 mm in a 5.4 mm layer) had its bearing standing 1 mm proud, and the hub the plan reserved beside it was silently dropped.
  - A tube now stands in a link's layer only inside the seat the axle claims there.
- **Gate:** every walker's parts and BOM change (intended), and `palletizer_reach`'s `b4` gets its planned hub.

### 2026-10-14: the turret pruned to mt4_compact (the user's call)

- **The decision:** "prune all but mt4_compact". The arm's turntable keeps only the
  approved preset (`base_pair`, `belt_60`, `nema17_17hs4401_x64`, `hard_stop`), now the
  role's default for every arm, so `--linkage palletizer` gets it too.
- **What went:**
  - The presets `belt_preload`, `mt4`, `capstan_preload`, `mt4_base`, `mt4_base_stall` and
    `capstan_precise`; the supports `deep_groove_pair` and `preloaded_pair`; the sensors
    `optical_flag` and `ee_sx67`.
  - The classic hub turret: the boss, the hub's seat plates and spacer, the printed
    brackets and flag, its endstop. `turret.py` is now the plan and the feet; `base_pair.py`
    builds the turret (`base_pair.turret_parts`), its optical-endstop branch gone.
  - The cord capstan: its transmission, the yaw drum, `Transmission.family` / `reverses` /
    `slips`, the control's reversed sense and phase flag, the host's flag homing, the
    camera's gain fit, its catalog items (the sleeve, the Dyneema cord).
  - Milling: `spiderpig/milling.py`, the role `mill`, the fab method `milled`, the `mill`
    output, the mill's plates and retaining compound (the user owns a Carvera, but chose
    to prune it).
  - The catalog's `bearing_608zz` and `shaft_8mm_h6_100`, which only the classic hub used.
- **A removed key fails with its replacement:** `config.REMOVED_CONSTRUCTIONS` now holds
  roles' keys too (`turntable` -> `mt4_compact`, `transmission` `capstan` -> `belt`), and
  `config.REMOVED_ROLES` the role `mill`: in a config, on the command line and in a spec.
- **Size:** the code commit (`3a72784`) is 49 files, 2,388 lines removed and 518 added
  (1,870 net): `turret.py` 772 out (96 in), `milling.py` 334, `capstan.py` 182,
  `base_pair.py` 155 out (65 in), `test_milling.py` 155, `rotary/__init__.py` 112 out
  (52 in), `plate_catalog.py` 52; the rest in control, precision, the BOM, the exports,
  the config's removed-key tables and the tests.
- **The gate** (`platform-3a72784` against `platform-379ad7e`): the six walkers identical;
  `palletizer_reach` identical to its own snapshot at `65b9e6d` (its differences from
  `379ad7e` are that commit's seat bosses and stadium cut); `palletizer_turret` different,
  as intended: `belt_preload` became `mt4_compact` (the side's plan unchanged; 185
  parts, 187 before; 17 laser parts, 30 before; DXF sheets 4 -> 3, one of them now the
  base's 1/8 in 5052; BOM 54 lines, $477.75, $558.28 before; audit OK, 3 warnings).
  DESIGNS.md regenerated from it.

### 2026-10-14: one controller role and one control surface for every robot (wave 1)

- **The decision:** a walker's electronics are the role `controller`, the arm's role, not
  catalog items the deck hard-wires; `spiderpig control` drives a walker's servos through
  the arm's entry point. Toward the user's "one seamless repo", not two forked code paths.
- **What a controller drives decides where it goes:** `Controller.drives` (`stepper` or
  `bus_servo`) is matched to the design's motors (`controllers.motors_of`: the role
  `servo`'s bus servos or the role `actuator`'s steppers), never to the kind's name. The
  walker's set (the ESP32 servo driver, the 2S LiPo, charger, protection board, switch and
  harness) is `esp32_bus_servo`, its parts the deck's bodies (`mount="deck"`,
  `controllers.DECK_KIT`); `none` leaves the deck bare (the plate on its rails, its wire
  and cable-tie slots; nothing of the kit bought; the guide's "Deck plate" step, no
  harness; the sim carries the kit's 61 g for your own; an audit warning). A stepper board
  on a walker, or the deck's board on an arm, is a `ConstructionError`.
- **Mechanisms:** none carried electronics (no robot is made of one side), so the role
  stays the walkers' and the arms'; `spiderpig control` still sets a mechanism's one servo
  up (your own board).
- **The ids stay:** a role reaching the walkers would have written `controller` into every
  walker's resolved spec and moved every stored design's id. `roles.Role.implicit` names
  what the designs carried before the role: where it is the default it isn't written
  (`BuildConfig.applied_roles`), and it is pinned (never move a walker's default). The
  resolved specs of the eight gate designs, a side, and an XL330 walker hash as at
  `f3bd61b` (`design.spec_hash`, `side_hash`).
- **The servos' setup** (`control.servos`): each crank's servo (a walker's `L` ID 1, `R`
  ID 2), its legs' crank phases, the forward sense facing its horn (the mirrored sides
  turn opposite ways: `sim.mjcf.crank_sign` and the sides' mirror), the zero (the first
  crankpin's angle at crank angle 0) and `servos.json`: IDs, baud, speed mode, the torque
  limit the joints are rated at, the protocol's register writes (Feetech STS, DYNAMIXEL
  2.0), the sim's phase-lock gains. Zeroing is once per servo: its absolute encoder's
  offset (`control.servos.zero_offset`; the STS's own middle calibration). Whether the
  servo's positive speed is the forward sense is checked by a jog and kept as a `sign`.
- **Found, not changed:** an XL330 or XL430 walker gets the deck's ESP32 board, which
  speaks Feetech's protocol, not DYNAMIXEL's (as before this change: its BOM is the same);
  `spiderpig control servo` says so (`controllers.protocol_note`). Changing its default
  would move those designs' ids and BOMs: the user's call. (Changed in wave 2, below: "the
  controller follows the servo", under the user's request of 2026-10-14 to merge the
  walkers and the arm holistically and fix what the reviews found.)
- **The gate** (against `platform-77c117c`): all eight designs identical (parts, plans,
  BOM, DXF, audit), so no new baseline and DESIGNS.md unchanged.

### 2026-10-14: hardening, wave 1 (no product change: the gate identical)

- **The slow tier on a PR, opt-in.** `ci.yml` gains a `slow` job (`-m "slow and not e2e"`,
  set up as the nightly's) that runs on a PR labelled `slow` (and on every push while the
  label stays) or on a manual run (`workflow_dispatch`). Why opt-in: up to four runner-hours
  a run. Why a label in the same workflow, not a second one: the cheap jobs re-run on a
  `labeled` event rather than being skipped there, since a skipped job reports success and
  would stand in for a failed one on the PR's checks. TESTING.md says how to trigger it.
- **The planner's budget, pinned and recorded.** `SPIDERPIG_PLAN_SECONDS` is CPU time,
  outside a design's id, so how far a search gets (a plan found, its proof, a cheaper
  route) depends on the machine. CI's and the nightly's jobs pin it to 3600, as the gate
  does (the node budgets alone bound a search); a stored plan records the budget its search
  had (`PlanReport.budget_s`, `None` for no clock) and a plan re-made from the store reports
  the record's, with a warning when this process's differs (`budget_recorded` false for a
  record from before). Plans and design ids are unchanged.
- **Strict vendor CAD.** `SPIDERPIG_CAD_STRICT=1`: a vendor model a build wants that can't
  be drawn (not cached and not downloaded, a hash mismatch, an import or placement failure,
  the wrong size, a horn that won't strip) raises `hardware.cad.CadError` instead of the
  parametric fallback; `spiderpig cad check` loads every registered model that way. Off by
  default: the gate, the audit and CI stay offline and parametric.
- **The server's bakes keyed on what shapes them.** `bakes/<config key>-<bake key>.glb`
  (`server.app.bake_key`): the config, the code a bake reaches (the fabrication's roots, the
  plan through the store and the bake; computed in a fresh worker once the sources changed
  under the server), the fab cache's env tag and the servo's and bought parts' CAD state.
  Before, a file named by the config alone was served while newer than the sources'
  mtimes: another worktree's code sharing the store, an installed package or a vendor model
  imported since could all serve a stale bake. A watcher's re-bake now skips a default
  whose key didn't move. `spiderpig bake`'s own file (`bakes/<config key>.glb`) is no
  longer served by the viewer's server.
- **pyright over the tests.** `[tool.pyright]` includes `tests/`; they held 743 errors
  when the ratchet began (optional reads of reports and parts, mostly; the baseline file
  holds 740 since wave 1's merge, `0e0557a`), so they are ratcheted on a count of their
  own (`tests/pyright-baseline.json`'s `"tests"`) beside the package's 21, each failing CI
  if it rises; the scorecard's `errors` stays the package's.
- **The roadmap** gained the platform's phases (2026-10-11 to 2026-10-14) and what is open.

### 2026-10-14: precision and strength are task targets for every kind (merge, wave 1)

- **The decision:** the output's error budget and the parts' strength go through the same
  spec, `verify`, API, MCP and scorecard for walkers, mechanisms and arms, chosen by what a
  linkage can do, never by its kind's name. Before, `spiderpig/precision.py` was a CLI only,
  and `strength.design_loads` gave an arm no loads ("a mechanism"), so `verify` rated
  nothing that doesn't walk.
- **The spec:** a `precision` section of targets (`spec.TARGET_FIELDS["precision"]`):
  `repeatability_mm` (the worst-case bound) and `accuracy_mm` (what calibrates out) for any
  linkage with an output; `rp_mm`, `rp_powerup_mm` (ISO 9283), `out_of_plane_mm` and
  `payload_sag_mm` for ranged inputs; and one plain number, `precision.payload_g` (an arm's,
  default `repeatability.PAYLOAD_G`). Each field names the capability it needs
  (`TargetField.needs`, `spec.CAPABILITIES`: `output`, `ranged`), and a spec asking a
  linkage without it is refused with the reason. `rp_mm` and `repeatability_mm` are hard by
  default (what a task asks of the placement), the rest soft. A spec without the section
  resolves as before (the same id).
- **The stage:** `api.precision(design)` (`api/accuracy.py`, `PrecisionReport`,
  `precision.json` in the store; an MCP tool, a job). An arm's budget is complete only with
  its links' tilt, which reads a fabricated side's wobble notes, so `verify("quick")` reports
  what needs no fabrication (a mechanism's whole budget; an arm's `accuracy_mm`) and leaves
  an arm's tilt-dependent targets unverified until `standard` (the build's notes: no extra
  fabrication) or a stored `precision` report. Measured on the approved arm at load ~30:
  the budget without the tilt 0.8-1.8 s (the turntable's plans now made once a budget, they
  were made three times), the complete one 44 s cold.
- **Strength for an arm:** `strength.arm_loads`, from the quasi-static model the ISO RP
  already solves (`repeatability.static_loads`, over a 5 x 5 grid of the inputs' box):
  operating (the walker's `walk` column), the arm's weight and the payload x 1.5 for the
  moves' accelerations; jammed, a crash, the tool blocked while every drive pushes at its
  holding torque x ratio, gravity added (the walker's jam is its servo stalled at the torque
  limit with a foot pinned: the same idea). These are the design's own loads, so a jam SF
  under 1 fails `verify` and the audit; the fix may be a lower run current. A link's hole is
  its pivot's bearing seat where it holds one (`strength.link_hole`; a plain pin's is the
  6.35 mm it was). A cranked mechanism's loads are still what it drives: not rated (said).
  Walkers' numbers are unchanged.
- **Found on the way:** an arm spec with any target validated in about a minute: the
  validator ran the arm's output check (its whole tour) for every section. It now runs only
  for a motion output target.
- **The scorecard** records the approved arm's RP (section `precision`, default on):
  0.042 mm within a session, 0.065 mm across power-ups (the model's, 2026-10-14).
- **The gate** (`platform-5396098` against `platform-77c117c`): the six walkers and the two
  mechanisms identical; parts, plans, BOMs and DXFs identical everywhere. Only the arms'
  audit strength sections differ, as intended: `palletizer_reach` is rated (static loads,
  a 44 N crash at the tool; the worst joint the elbow `pin:E`, jam SF 1.67: one more
  warning, the audit still OK); `palletizer_turret` (the older `palletizer`, acrylic `b4`
  and single-bearing pins) fails at a crash: `pin:E` jam SF 0.44 and `link:b4` 0.48, with
  the fixes the approved arm already has (an aluminium `b4`, the `ball_pair` pins, the
  links adjacent). DESIGNS.md regenerated from it.

### 2026-10-14: walker models from the arm's: the cut length priced, the feet across the plane (wave 1)

- **The cut length in the cutting cost** (`bom.cut_estimate`, `bom.cut_length_cm`,
  `sheet_catalog.CUT_RATES`): the area model priced the Strider's hole-dense aluminium
  8.9 % under SendCutSend's live quote (61.69 against 67.70). Each part's cut length (every
  contour of its face, holes included: the DXF's contours) is now a term, refitted by
  least squares of log(estimate / quote) on the 42 one-off prices recorded on 2026-10-08
  (the Strider double, 21) and 2026-10-12 (the arm, 16 plus 5 alternatives), each quoted
  DXF measured again. No new quotes; more would settle 6061.
  - **5052** (14 prices): 0.076 / cm² + 0.104 / cm of cut, no fixed charge, at least 2.70
    (just under the cheapest aluminium part quoted; never seen as a floor). Per part rms
    24 % -> 12 %, worst +45 % -> +26 %; each part left out in turn 27 % -> 14 %.
  - **6061** (8 prices): unchanged (3.20 + 0.138 / cm²). A cut rate fits its prices a
    little better (rms 20 -> 15 %) but predicts a part left out worse (22 -> 35 %): the
    arm's b4 and b1 pull it opposite ways; their bounding rectangles tell them apart (the
    next term, on more prices).
  - **Acrylic**: unchanged, no cut term: every 3 mm part up to 36 cm of cut quoted the
    2.00 floor, which bounds any cut rate under 0.002 / cm.
  - **Against the quotes:** the Strider's aluminium 67.70 at 68.98 (+1.9 %, was -8.9 %); its
    acrylic 49.44 at 48.72 (unchanged); the arm 163.01 at 161.58 (-0.9 %, was +0.5 %; its
    5052 -0.6 %, was +1.4 %).
- **One file per shape** (`bom.same_files`): rows of one area to 1e-3 were priced as one
  file, so the Strider's b4 and b8 (8.66 and 8.65 cm²) and b3 / b7 shared a quantity break
  SendCutSend doesn't give them (it quoted them as separate files). Now area and cut length
  to 1e-5. This is most of the acrylic's change below.
- **The BOMs** (cutting, then the total; the gate `walker-f7036a7` against `platform-77c117c`):
  - `strider_double`: cutting $113.92 -> $124.58, total $418.03 -> $428.69. 5052 0.080 in
    26.04 -> 35.54 (the torso and frame plates), 0.063 in 23.74 -> 21.54, acrylic 44.24 ->
    47.60.
  - `strider_quad`: $163.55 -> $175.80; total $526.86 -> $539.11.
  - `klann_lego_quad`: $164.51 -> $174.65; total $434.89 -> $445.03.
  - `klann_quad`: $160.22 -> $178.28; total $522.57 -> $540.63.
  - `hoecken_pantograph`: $32.61 -> $33.90; total $141.89 -> $143.18.
  - `dwell_rocker`: $31.54 -> $36.49; total $130.42 -> $135.37.
  - `palletizer_turret`: $83.40 -> $87.43; total $501.51 -> $505.54.
  - `palletizer_reach`: $106.10 -> $109.21; total $618.89 -> $622.00.
  - verify's cost floor counts a 5052 part at 2.70 (was 3.30).
- **The feet across the plane** (`strength.foot_stiffness`, `linkage.stiffness.across`):
  a walker's feet take sideways loads (a turn, a slip) that nothing reported. The arm's 3D
  frame model now serves walkers by capability (`stiffness.outputs`: a ranged linkage's
  tool point, a walker's feet; `stiffness.drives`: the inputs, or a crank held at its
  centre; a segment split at a joint partway along it, Strider's J1 on b2). The arm's
  numbers are unchanged, bit for bit.
  - **The model:** each leg's links at their plan's layers, their own sheets' sections; each
    pin and pillar its own shaft (the wobble note's section, with its modulus: the Al
    standoff 69 GPa, `wobble.Section.modulus_mpa`); the crank rigid and driven; the frame
    plates rigid.
  - **The sideways load** (`strength.foot_load`): the robot's nominal weight on the fewest
    feet touching over a straight walk (`walk.support`), times the sim's floor friction
    (0.65): the most a foot takes before it slips.
  - **Reported in the audit:** each foot's compliance across the plane (mm/N, worst over
    the revolution), and at that load its deflection and the worst link tilt and splay.
    This is the elastic part; the joints' play (the wobble check) comes on top. No limit.
  - **Numbers:** the Strider double 2.05 mm/N, 2.35 mm at 1.15 N (719 g on 4 feet), b4
    tilting 1.95°. klann_lego quad 0.48 mm/N (its 6061 foot links), 0.70 mm at 1.47 N,
    0.37°. The demo Klann quad 9.6 mm/N, 15 mm at 1.59 N, 4.5° (its long acrylic b4).
- **Open:** a bounding-rectangle (material) term for 6061 and 5.4 mm acrylic, on more
  quotes. The feet's numbers count the swing too (the worst over the revolution), and a
  joint held only by its play is not modelled (the wobble's separate row).

### 2026-10-14: one homing-sensor registry for every joint, the turntable's included (wave 2)

- **The decision:** every joint's homing is one registry, `actuators.sensors.SENSORS`
  (`HomingSensor`; it was `ANGLE_SENSORS` for the planar inputs beside
  `construction.rotary.SENSORS` for the turntable: one method, stall homing, in two
  registries as `stall` and `hard_stop`). Each sensor declares how it homes
  (`HomingSensor.method`: `endstop`, `stall` or none), whether it reads against gravity
  (`gravity`: no use about the yaw's vertical axis) and what it buys; `control.homing.method`,
  the Klipper config, the host, `precision` and a drive's BOM read the declaration, never
  a key's name. The turntable preset `mt4_compact` names `stall` from it
  (`RotaryPreset.check`: the turret builds a stall's stop only); each construction builds
  that method's hardware at its joint (the turret's post and screw, `base_pair`; a planar
  drive's M3 screw and nut, `transmissions.base.HARD_STOP`, as before).
- **A stall's repeatability, derived:** `sensors.STALL_FULL_STEPS` (2) full steps of the
  joint's motor through the joint's own reduction (`HomingSensor.at_joint`): the rotor lags
  its field by up to a full step before it slips and a TMC2209 updates its StallGuard
  result once a full step (an estimate, unmeasured). The belts' 5:1 keeps the planar
  inputs' 0.72 deg; the yaw's 3.75:1 makes 0.96 deg, where the old fixed 0.75 mm at its
  26 mm stop was 1.65 deg (it assumed nothing of the yaw's ratio).
- **`palletizer_reach` (`spiderpig precision`, before / after):** ISO RP 0.042 mm in a
  session and 0.065 across power-ups, both unchanged (the camera,
  `vision_fiducial_ov5647`, sets every zero: each input's homing row stays its 0.015 mm);
  the turntable's coarse stall homing at the output 8.863 -> 5.118 mm (a note, outside the
  budget with the camera).
- **The gate** (against `platform-5396098`): all eight designs identical, audits included.
  DESIGNS.md unchanged.

### 2026-10-14: the surfaces start from a task, for any kind of linkage (wave 2)

- **The decision:** the ways in (the Python API, the MCP server, the viewer) take a task
  and answer for every kind by what each linkage declares, toward the user's "one seamless
  repo that can generate surfaces and linkages for a given task". A task
  (`spiderpig.task.Task`: `walk`, `place` or `trace` and its numbers) names no linkage;
  `api.candidates(task)` ranks the registered ones that can do it, each with its checks
  (value, need, ok, tier, source) and a spec that asks for the task's numbers as targets,
  and says why each other one can't. The MCP tool `candidates`, the resource
  `spiderpig://schema/task` and the prompt `design(task)` replace the walker-only
  prompt.
- **Capabilities, never kinds:** feet (`Linkage.walks`), an output and its motion
  (`spec.CAPABILITIES`), ranged inputs and how many, and the roles a design fills: a place
  task takes three axes (two across the bench, one to lift), so one input (a path) and two
  in a plane are refused with their count, and an arm's two inputs plus its turntable's yaw
  (the role `turntable` applies) qualify. An arm's `repeatability_mm` is ISO 9283's
  `precision.rp_mm` (what a maker quotes), a cranked mechanism's the worst-case bound.
- **Cheap first, nothing fabricated:** the cards and the kinematics (an arm's reach over its
  box about its yaw axis is a `bound`: it refutes, never confirms, since the bench its
  turret stands on is the plan's); then what the store already measured for the same
  machine whatever its targets; `measure=N` plans the best N (an arm's plan took 4-6 min
  cold at load ~70, and its re-made plan 3.7 min). Measured: the approved arm's tool gets
  down to the bench 149-288 mm out from its yaw axis over 240 deg of yaw, so a 200 x 150 mm
  area misses by 29 mm (`sector_fit`); x1.19 of its lengths would hold it (unchecked, the
  base's keep-out doesn't scale); its stored ISO RP, 0.042 mm, meets 0.1 mm.
- **The viewer:** `/api/modes?linkage=` is the views the linkage declares
  (`server.app.modes_of`: a robot per `config.robot_layout`, a walker's module sides, an arm's
  or a mechanism's one side); the page's picker follows the linkage on screen, and a machine
  picker lists every linkage without feet (it listed mechanisms only, and an arm's
  `?linkage=` fell back to the Strider).
- **The arm's guide:** the turret's base goes on in seven steps on the bench, then the side
  stands on it (`base_pair.assembly`, its sentences the turret's meta, tagged), in the
  layout's order (`assembly.LAYOUT_ORDERS`, `TURRET_ORDER`). The wiring step and its diagram
  are the controller's declared wiring (`Controller.nodes`, `wiring`, `wiring_text`,
  `wiring_extras`; `assembly.controller_ops`, `guide.wiring.wiring_of`): the walker's
  harness sentence moved from the deck's hook into `esp32_bus_servo`, word for word; the
  arm's step names its supply, host, ports (`q1 on X, q2 on Y, q3 on Z`), outputs and
  camera, and its parts list buys the board's lines. A BOM line is a box only where no body
  is its role's.
- **The gate** (against `platform-5396098`): all eight designs identical (parts, plans,
  BOM, DXF), so no new baseline and DESIGNS.md unchanged; the hooks stay out of the
  fabrication's code key (the turret's sentences are `turret_parts`' meta, its steps
  `base_pair.assembly`'s).

### 2026-10-14: one MuJoCo model path for every design (wave 2)

- **The decision:** `spiderpig/sim/mjcf.py` is the one MJCF emitter: every design (walker,
  arm, mechanism) goes through one reduction (`sim.mjcf.robot_model`: the planar side's
  bodies and pins, the spatial joints of `meta["tree"]`, `kinematics.RobotTree`, the tree
  the bake and the URDF read) and one emitter, chosen by what the design declares, never by
  its kind's name. Before, an arm's model came from a second writer beside its URDF
  (`urdf.mjcf`, removed with its `urdf/<name>.mjcf.xml`), the export skipped `mjcf` for
  anything that doesn't walk, and `spiderpig sim` ran walkers only.
- **What a design declares:** `Linkage.walks` puts it free on a floor with its feet's
  contacts (the walker's model, byte for byte as before: the Klann single and quad, the
  Strider single and double and the TrotBot built before and after, identical XML and
  metadata); anything else is fixed to the world, upright, with no floor (its base's floor
  at the origin: a turret's under the yaw axis, so MuJoCo's world is the URDF's
  `base_link` and `control.arm.ArmModel`'s base frame). A crank gets the servo's velocity
  actuator as before; a joint `actuators.joint_ratings` rates (an input's transmission, the
  turntable, the wrist: the URDF's limits read the same ratings) gets a stepper's position
  actuator: its holding stiffness through the reduction (T_hold x rotor teeth x ratio²),
  no stiffer than `HOLD_OMEGA_DT` (1.0) allows at the 1 ms step on the joint's tree inertia
  (`dof_M0`), critically damped, its force its holding torque x ratio, its rotor's inertia
  x ratio² the joint's armature. An input's hinge hangs from the frame on its drive's
  pivot and reads its design angle (`ref`), so its `ctrl` is the input in radians.
- **Why the tree inertia, not the loops':** capped on the inertia the elbow drives through
  its parallelogram (a null-space projection of the joint-space inertia), the elbow's
  stiff actuator fought its light rod's soft loop (a 2 N·m oscillation, 0.7 mm open,
  measured on the palletizer). On `dof_M0` it is calm: the approved arm's `q3` at 184 of
  500 N·m/rad, `q1` and `q2` at their motors' own.
- **What runs it:** `api.export` writes `mjcf` for every kind through one path (the URDF
  stays the ROS export, an arm's); `spiderpig sim` and `verify("full")` hold a fixed
  design at its pose under gravity with its motors (`sim.run.hold`, the row `sim.holds`,
  soft): `palletizer_reach` sags 0.043 mm at the tool's tip in 3 s, `q2` at 8 % of its
  holding torque, its loops closed to 0.009 mm. A recorded fixture of it
  (`tests/fixtures/sim/mjcf_palletizer_reach.json`) makes that a fast test.
- **The gate:** parts, plans, BOMs and DXFs identical on all eight designs; the two arms'
  `spiderpig build` no longer writes `urdf/<name>.mjcf.xml` (its MuJoCo model is the
  export's `mjcf`), the one intended difference.

### 2026-10-14: every design's defaults pass their audit; the controller follows the servo (wave 2)

- **The decision:** the default build of every design the gate holds passes its own audit,
  and a design's defaults follow what it is (its servo's protocol, its kind's roles), toward
  the user's "one seamless repo".
- **The plain `palletizer` holds a crash now:** it failed at a crash at the tool (`pin:E` jam SF 0.44, acrylic
  `link:b4` 0.48). The approved arm's pivots are now every arm's (`config.DEFAULT_PIVOTS`:
  `ball_pair` / `ball_pair_pillar`; `LINKAGE_PIVOTS` empty) and its links' sheets both
  palletizers' (`materials.PALLETIZER_LINKS`): the audit OK, 8 warnings (the worst jam SF
  pillar `O` 1.24, pin `E` 1.76). Its geometry, roles (gauge homing) and turret stay. The
  approved arm is unchanged (config, ids, plan).
- **Found on the way:** a drive hub's bolt heads were claimed in the layer past their link,
  but bear on the link, so on a plan with a clearance gap there (the palletizer's new one)
  they stood outside their claims (the contract, 26.8 mm³ each). They claim that gap too.
- **The demo `klann` quad still fails** (`tests/gate/identity_gate.py`
  `KNOWN_AUDIT_FAILURES`): pins `C`, `D`, `E` of two legs span 27-40 mm (jam SF 0.30-0.43 at
  the family's 155 N; 0.71 on ground `ball_pair` pins). Tried, with the planner at 3600 s:
  its barrels capped at 16, 20 or 23 mm (`chicago.MAX_BARREL`), every pin's links at most a
  layer apart, the variants' 0, 0, 180, 180 phasing: no plan within the node budgets (the
  crank's riders and the cantilever pillars `B` keep a leg's links far apart; capped at
  14 mm with that phasing and ten times the budgets, none in 21 layers or fewer, proven; at
  16 mm with ten times the budgets, stopped unfinished after an hour). A torque limit
  can't fix it: its walking SF is 0.39. **Proposed, the user's call:** prune the demo
  (superseded by `klann_lego`, the order design, and the diywalkers variants); it is the
  family's measured loads (`strength.FALLBACK_PIN_LOADS`) and a test design in 44 test files
  (72 files in all), so pruning it means moving those to a variant.
- **Pruning the plain `palletizer`** (proposed): it is superseded by `palletizer_reach` too (the same links; a
  shorter elbow crank and a box that doesn't reach the table). Pruning it would touch 10 test
  files (`test_arm`, `test_band`, `test_control`, `test_pair`, `test_precision`,
  `test_repeatability`, `test_stiffness`, `test_transmissions`, `test_turret`,
  `viewer/src/arm.test.ts`) and the gate's `palletizer_turret`: proposed, not done.
- **The gate checks the audits:** `compare` / `snapshot` list a design whose audit fails and
  exit 3, unless `KNOWN_AUDIT_FAILURES` says why (one that passes is reported for the table
  to lose it); `mise run gate -- audit DIR`; the slow `tests/test_gate_audits.py` runs the
  eight audits (a known failure a strict `xfail`).
- **The controller follows the servo** (`controllers.BUS_DEFAULTS`; the wave-1 entry left it
  as the user's call: changed under the user's request of 2026-10-14 to merge the walkers
  and the arm holistically and fix what the reviews found): an XL330 / XL430 walker
  had the deck's ESP32 board, which speaks Feetech's protocol, not DYNAMIXEL's. It now gets
  `u2d2_dynamixel`: ROBOTIS's U2D2 (USD 36.92) on its Power Hub Board Set (USD 21.85), both
  from robotis.us's product pages read 2026-10-14 (no login, no bot check; both shown sold
  out that day), bought as lines, the deck bare; your computer runs the walk over its USB,
  your supply at the servo's voltage powers the bus. An STS3215 walker keeps the deck kit
  (the role's `implicit` key): every STS3215 design's id is as it was; an XL walker's resolved
  spec now names its controller, so its `spec_hash` moves (`side_hash` and `config.key`
  don't).
- **An arm doesn't carry a servo or a crank sheet** (`config.ROLE_FIELDS`): the roles
  `servo` and `crank` aren't an arm's, so another servo or crank sheet is refused (a config,
  a spec, a switch of linkage) and `spec_of` leaves them out. They are not dropped from the
  key or the resolved spec: every stored arm's `config.key`, `spec_hash` and `side_hash`
  hashes their defaults, so dropping them would move every arm's id (checked on the eight
  gate designs: all unchanged by this part).
- **One profiler:** the bake profiles with `spiderpig/tools/profiler.py`'s `Profiler`
  (`bake._profiler`: title `bake`, total `bake_total`, logger `bake_gltf`), its copy gone; the
  summary's text and the stage keys as they were.
- **The gate** (`platform-78e35e8` against `platform-5396098`): the six walkers, the two
  mechanisms and `palletizer_reach` identical; `palletizer_turret` different, as intended:
  `ball_pair` pins and pillars, the 5.4 mm acrylic links and the 1/8 in 5052 `b4` (a fourth
  DXF sheet), 16 layers 61.7 mm (46.1 before; heads in a 3.5 mm gap over layer 6), 201 parts
  (167), 19 laser parts (17), BOM 71 lines $595.79 ($501.51; two DIN 988 6 x 12 shim
  thicknesses newly unpriced, not guessed), audit OK with 8 warnings (FAIL (2) before);
  `klann_quad` fails, known. DESIGNS.md regenerated from it.

### 2026-10-14: hardening after the merge's reviews (no part, plan or BOM moved)

- **The viewer's server keys a bake under its lock** (`server.app._ensure_baked`): the key
  was taken before a request waited its turn, and `_bake` keyed again, so a source edit
  meanwhile returned a file never written (a 500, remembered as baked). Now the key is
  taken under the lock and the answer is the file `_bake` wrote, named by the code it ran
  (keyed as it starts) and the CAD state it left (a model it downloaded).
- **Strict vendor CAD is in every cache's key**: the fabrication cache's entry
  (`fabcache.entry_name`), the server's `bake_key` and the build skip (`uptodate.CAD_ENV`).
  Before, an entry, a bake or a build made without `SPIDERPIG_CAD_STRICT=1`, which may hold
  a parametric fallback, answered a strict request. Off (the default) the names are as
  they were.
- **A bake key is probed once:** every `GET /api/glb` keyed its design, and the servo's
  probe could download (60 s timeout, again on every request offline). `bake_key` is
  memoised per config, code key and a stat of the CAD cache and its switches
  (`server.app._cad_stamp`), and probes the cache without downloading
  (`fabcache.servo_state(config, download=False)`); the bake may still download, and its
  file is named by the state it left.
- **One file per shape, the cut to its rounding** (`bom.same_files`, `bom.CUT_TOL_CM`): cut
  lengths are settled to 0.001 cm, so a shape's copies could read a step apart, which a
  relative 1e-5 split into two files under 100 cm of cut. They match within 0.002 cm now.
  On the gate's `strider_double` and `palletizer_reach` every sheet's files and price are
  as before (their rows are already grouped).
- **CI:** a label other than `slow` runs in a concurrency group of its own, so adding one
  no longer cancels an in-flight slow tier (nor starts it again); removing `slow` starts
  nothing (TESTING.md). The nightly gate goes on to its determinism compare after a first
  snapshot whose audit fails (exit 3) and fails the job after it; `compare` reports both
  verdicts (`identity_gate.exit_status`: 4 and 5 are "order differs" and "DIFFERENT" with
  an audit failure).
- **The cut model, left out:** `tests/test_order.py` now refits 5052's two rates with each
  quote the repository records left out (the Strider's six one-off prices of 2026-10-08
  and the arm's two 5052 sheets of 2026-10-12, summed: its per-part prices aren't kept
  here) and predicts it: rms 23 % with the cut term against 42 % for a fixed charge and the
  area, the fit on all of them within 6 and 9 % of the shipped rates. The per-part figures
  of 2026-10-14 (14 % against 27 %, 14 prices) were measured on the quoted DXFs, which
  this repository doesn't hold.
- **`spiderpig.spec` and `spiderpig.task`** are under a lint-imports contract: neither
  reaches into the pipeline or a front-end.

### 2026-10-14: one task flow on every surface, from the task to the files that make, wire and buy it (review fix 1)

- **The decision:** a design id carries from `candidates` to every file a maker needs, on
  every surface, toward the user's "a seamless merge of capabilities, not just two forking
  code paths". The flow stopped at `export`: an agent holding an id had no assembly guide,
  wiring, controller files or `ORDER.md`, and the CLI took only build options (a task's
  spec, its roles, didn't survive).
- **The maker's formats:** `api.export` writes `guide` (`ASSEMBLY.pdf` / `.md`, cached
  beside the fabrication), `control` (`control/`) and `order` (`ORDER.md` with the
  per-part DXFs of `laser/parts/` it uploads and the `print/` STLs it lists) beside the
  spec's outputs (`api.MAKE_FORMATS`: export formats only, not a spec's `outputs`, whose
  vocabulary `spec.py` owns). The MCP's `FORMATS` and `spiderpig export --formats` are
  `api.formats_all()` (a test holds them equal); the MCP `design(task)` prompt goes on past
  the files to the guide, the controller's files and the order.
- **One dispatch on the motors:** `spiderpig.control.files` writes what drives the design
  needs (`actuators.controllers.motors_of`, a key of `files.WRITERS`): a crank's bus servos'
  `servos.json`, an arm's steppers' Klipper `printer.cfg`, and either's `HOMING.md`; `spiderpig
  control servo|klipper|homing` print and write through the same functions, and `spiderpig
  control files` writes them all. No kind's name is read.
- **The layers:** `spiderpig.control` moved a layer down, beside the guide (it imports
  nothing above the sim and the stages), so the API exports its files; `load` (and the
  config of a recorded design) moved from `api.store_ops` to `stages.resolve` (re-exported,
  the old path kept), so `spiderpig build`, `guide`, `precision` and `control` open a stored
  design under the API: `stages.resolve.load_stored`, its plan through
  `stages.planning.plan_stored`.
- **The CLI:** `spiderpig build|guide|control|precision DESIGN` (as `export`, `view`, `sim`
  already did): the config as recorded, its roles kept (a Strider on your own controller
  writes `servos.json` with none, where the options' default names the ESP32 board);
  `build`'s manifest names the design (another `--kerf` or `--sheet-size`, another design).
  `spiderpig candidates TASK.json [--measure N] [--limit K] [--resolve K]` is
  `api.candidates`, the best `K` (1) resolved into the store for their ids. The help says
  what `sim`, `control` and `export` do for every kind.
- **The viewer:** one linkage picker lists every linkage of `/api/linkages`, grouped by kind
  (an `optgroup` each, `modes.linkageGroups`); the view picker follows the linkage picked
  (one with feet re-baked as a robot to drive, one without in its own default view). The
  separate machine picker of wave 2 is gone.
- **The docs:** `docs/ARCHITECTURE.md` sections 1 and 3 now start from the task and run
  the pipeline for all three kinds (a stage reads capabilities), API.md has the task flow
  from the command line, the MCP guide a "make it" loop.
- **The gate** (against `platform-78e35e8`, `strider_double` and `palletizer_reach`): the
  parts and the plans identical. The BOM differs as that baseline predates the cut model
  (`cut_cm`, the cutting estimate, ORDER.md's cutting line), the Strider's audit has the
  feet's strength row (wave 1) and the arm's export no `urdf/*.mjcf.xml` (wave 2's one
  MuJoCo path): none of it this change's, so no new baseline and DESIGNS.md unchanged.

### 2026-10-14: one task and spec vocabulary for every kind; an arm traces a line (fix 3)

- **The decision:** a task and a spec say the same thing the same way whatever the machine,
  toward the user's "a seamless merge of capabilities, not two forking code paths".
- **An arm takes a trace task.** `api.candidates` excluded a ranged linkage whenever a task
  asked an output metric beside the stroke, and a straight `path_mm` always asks a
  straightness, so no arm could ever trace a line. Now the task's path (else a line of its
  stroke, tried at 0, 90, 45 and -45 deg) is solved point by point by the controller's own
  inverse kinematics (`api.tasks.trace_path`: `control.arm.ArmModel.ik`, a waypoint every
  5 mm as `control.host.line` moves, the joints linear between them, the inputs' ranges and
  band held), the arm standing where the path fits in its plane (translated, not turned;
  the placements deepest in its inputs' box tried first). The path reached is a `path_mm`
  check and the stroke, `measured`; the tip's band across the line (the sag between
  waypoints: 0.005 mm on a 100 mm line for `palletizer_reach`) is a `bound` on the
  straightness until a precision budget adds twice its worst-case lost motion. A crank's
  measures (`dwell_deg`, `swing_deg`, `on_line_fraction`, `rotation_deg`) still exclude an
  arm. A bound's number no longer ranks as merit, and a bound met is "not ruled out", not
  met, wherever it is (an arm's box-diagonal stroke too). For it the layers put `control`
  under `api` (`pyproject.toml`: `control` imports nothing of the API's).
- **`repeatability_mm` is the worst-case bound for every kind** (`precision.repeatability_mm`,
  which every linkage with an output has): the guarantee, where ISO 9283 RP is a
  Monte Carlo estimate of the scatter under one-sided approach, and an arm's only. An arm's
  RP is shown beside it (`numbers.rp_mm`, the check's detail); a spec may still target
  `precision.rp_mm`. The cost, the user's to weigh: the approved arm's bound is several
  times its RP (`test_the_approved_arms_precision_is_a_stored_report`: RP under 0.1 mm,
  the bound over it), so a 0.1 mm place task no longer passes it on its RP.
- **One spec vocabulary:** `kind` is read from the linkage when not given (checked when it
  is); `legs.robot` is every kind's robot (a walker's two sides, an arm's turret; a
  mechanism's `false` only), `legs.sides` its older name, still read for a stored spec (a
  walker's 2 is `robot: true`) and written into `robot` by `Spec.from_dict`; `spec_of`
  writes `robot`. `design.resolved` keeps `legs.sides` (and an arm's `legs.robot`) as
  before. `BuildConfig()`'s `robot` defaults to the linkage's kind's (`default_robot`), so
  `BuildConfig(linkage="hoecken")` is its one side instead of an error.
- **Every id kept:** on the eight gate designs, `config.key`, `spec_hash` and `side_hash`
  are unchanged from specs written the old way (`spec_of` with `sides`, `kind` given, a
  walker's `sides: 1`) and resolve to the same from the new (no `kind`, `legs.robot`).
- **Bugs:** a precision metric a budget left unbounded (a singular pose) was skipped as
  unmeasured; it is a miss. An obstacle replaced the task's whole `lift_mm` (its max, tol,
  weight, hardness lost); it raises the min only. A walker's speed read `BuildConfig()`'s
  servo, not the linkage's default (`config.role_field_default`).

### 2026-10-14: a task's repeatability is ISO 9283 RP where the design has one (supersedes the "worst-case bound for every kind" bullet above)

- **The rule:** a task's `repeatability_mm` targets ISO 9283 RP (`precision.rp_mm`, the mean + 3 sigma a maker quotes) wherever the design has one, i.e. a positioner with ranged inputs. Elsewhere it targets the worst-case bound (`precision.repeatability_mm`), which stays beside RP as information (`numbers.bound_mm`).
- **Why:** the user's requirement for the arm was "ISO 9283 repeatability <= 0.1 mm". Holding it to the worst-case bound made the approved arm (RP 0.042 mm) fail a 0.1 mm placement task it meets.
- **Straightness of a traced line** keeps the bound: twice the worst-case lost motion either side of the line, a guarantee rather than a scatter.
- The choice is made by capability (whether the `rp_mm` target applies to the linkage), not by kind.

### 2026-10-14: one engine for every machine: controllers, strength, sim and exports by capability (fix round 2)

- **The decision:** what a design gets follows what it declares (its inputs, `Linkage.ranged`
  / `walks`, its robot layout), never its kind's name, toward the user's "one seamless repo
  ... for a given task".
- **Every design with motors has a controller.** The role `controller` applies to mechanisms
  too (and `pin` / `pillar` to arms, whose `ball_pair` pivots the guide's cover now names).
  A mechanism's default is its servo's protocol's (`controllers.BUS_DEFAULTS`: the ESP32 board
  for the STS3215, `u2d2_dynamixel` for an XL); its side is the whole machine (no robot is
  made of it), so `BuildConfig.side_config` keeps its robot roles and `fabricate_side` buys
  the controller as lines (`construction.robot.controller_lines(deck=False)`, no deck to draw
  it), wired in its side's last step (`SIDE_ORDER`'s wiring stage; the step lists every part
  bought). The default STS3215 mechanisms' ids stay (the role's `implicit` key); their BOM
  gains the ESP32 kit's 14 lines (the board, its 2S LiPo, charger, protection board, switch,
  harness, the board's nylon standoffs): a one-sided demo on a bench still needs a board and
  power, and the kit's battery keeps it untethered (`none` buys nothing).
- **One drive vocabulary:** the motor role is chosen once from the inputs
  (`roles.motor_role`: a crank's `servo`, ranged inputs' `actuator`); `Controller.drives`,
  `controllers.DRIVES`, `construction.DRIVES` and `spiderpig control` use those names
  (`"bus_servo"` / `"stepper"` before). `construction._joint_drives` serves any declared drive,
  and a cranked linkage's second input may declare one (`Linkage.drives`, a stepper's
  `JointDrive` beside the crank's servo, the constraints' layout holding it): the engine side
  of a five-bar. `five_bar` declares none yet, so it still stops at `drive` /
  `second_input_no_drive` (the API's note is another package's this round).
- **A mechanism's jam is rated** as the arm's crash (`strength.crank_loads`: the static model
  over a turn of the crank; operating its weight x 1.5, jammed its output blocked while the
  servo pushes at its torque limit, the output's dead points, under 10 % of the crank's
  largest lever, left out). At the STS3215's 0.85 N·m every one-input mechanism failed (jam SF
  0.22-0.85: small cranks on acrylic links). Each now has its own limit
  (`config.LINKAGE_TORQUE_LIMITS`, klann_lego's lever: no part changed): the largest that puts
  every joint and link at jam SF 2, 0.09 (rocker_amplifier) to 0.36 N·m (hoecken); MuJoCo
  turns each at 0.006-0.031 N·m, so each keeps 6.8 x or more of headroom over its own motion.
  The torque limit is what `spiderpig control servo` writes into the servo, as a walker's,
  hence the rating at it rather than the stall. **The user's call:** a mechanism driving a load
  needs its limit raised and its links rated again (aluminium links clear it: the fixes say).
- **The fixed designs' sim:** `sim.run.hold` holds a crank in the servo's position mode (a P
  loop over its speed loop; the zero-speed command alone crept) and an arm with the payload
  its precision assumes (`verify.hold_payload_g`: the spec's, else `PAYLOAD_G`); `sim.run.turn`
  drives a fixed design's crank through a turn (the second, past the spin-up): `verify full`'s
  soft `sim.torque`, as a walker's.
- **Smaller fixes:** a DYNAMIXEL walker's bare deck carries the U2D2 and its hub
  (`controllers.carried_g`, `sim.mjcf.bare_deck_g`), not the ESP32 kit's 61 g; `u2d2_dynamixel`
  declares its wiring (boxes, links, sentences, its two lines in its step); the walker's
  port sentence moved into `esp32_bus_servo.wiring_text`, the chassis' routing stays the
  mirrored pair's (`ROBOT_ORDER`'s wiring intro); a robot's mechanism names its layout
  (`meta["layout"]`, no `"mirrored_pair"` fallback; the drive groups no `"walker"` one); a
  precision budget that raised fails `verify` only with a precision target.
- **The URDF** stays a ranged linkage's: `urdf.write` refuses a crank's design with
  `urdf.unsupported`'s reason (its joints follow the crank's full turn, which a URDF tree can
  only table; a walker has no fixed base), not exported half-way; its MuJoCo model holds it.
- **The gate** (against `platform-78e35e8`): every design's parts, plans, DXF and meshes
  identical. Intended: the two mechanisms' BOMs gain the ESP32 kit's 14 lines
  (`hoecken_pantograph` USD 141.89 -> 248.59, `dwell_rocker` 130.42 -> 240.78, the cutting's
  share of it the cut model's) and their audits' strength is rated (worst jam SF 2.04 and
  2.01, link `b1`; the crank 15.8 and 38.0 at the new limits), both audits OK. The cut model's
  BOM diffs (`cut_cm`, cutting USD), the walkers' `feet` and the arms' dropped
  `urdf/*.mjcf.xml` predate this round (the baseline is older than the merged waves).
  DESIGNS.md is not regenerated (no part moved).

### 2026-10-14: a crank servo's torque limit everywhere it matters; a mechanism's controller unnamed stays none (fix round 3)

- **The torque limit reaches every servo design's BOM.** `config.torque_limit_note` is
  keyed on the design's motor role (`roles.motor_role`: a crank's `servo`), not on a
  walker's robot (`BuildConfig.mirrored` before): the export (`api.exports`) and `spiderpig
  build` add it to every design a crank's servo drives. A mechanism's limit
  (`config.LINKAGE_TORQUE_LIMITS`, 0.09-0.36 N·m) is far under the STS3215's own 0.85 and
  its joints are rated at it (`strength.crank_loads`): left at the servo's own, every one of
  them fails a jam. A walker side's BOM gets the note too; an arm's steppers have no
  firmware limit (none).
- **A fixed design's turn is bounded by that limit.** `sim.run.turn` caps the crank's servo
  at the limit it is set to (`sim.run.crank_torque_limit`: `config.torque_limit_nm` of the
  servo and linkage the model's meta names, an exported MJCF's too; `torque_limit=` to
  override) on top of its speed-torque line, and reports against it (`torque_limit`, `stall`
  beside it); `verify` full's `sim.torque` row is bounded by it, and `sim.run.hold` caps the
  crank the same way. Before, the turn ran up to the stall (1.91 N·m), so a mechanism passed
  at torques its firmware never gives. A walker's `simulate` is unchanged (its jam loads pin a
  foot at the limit, `sim.loads`).
- **The fixes name the lever of the design's own motors.** `strength.stepped` (the motor
  role `actuator`): only a stepper design's findings name the drives' holding torque (their
  run current); a mechanism, rated as an arm's crash since round 2, got that stepper advice
  beside its servo-limit fix.
- **Same id, same parts and BOM, for the mechanisms too.** Round 2 gave a mechanism the
  ESP32 kit by default under the role's `implicit` key, so its id stayed while its BOM gained
  14 lines: a mechanism stored before read back with a kit it never had. A resolved spec that
  doesn't name a role reads back with the role's default (`stages.resolve` builds the config
  from the stored keys alone), so an unnamed role must mean, forever, what the kind's stored
  designs carried. `roles.Role.implicit` is now per kind (`((kind, key), ...)`): the walkers'
  `esp32_bus_servo`, the mechanisms' `none`; where it is `none` it is also the kind's default
  (`roles.default_key`). A mechanism's controller is `none` unless named; naming the kit
  (`esp32_bus_servo`, bought as lines, wired in its last step) writes it into the resolved
  spec and the id. The alternative, the kit by default with its key in new mechanisms' ids,
  needs a stored spec without the key read as the kind's implicit key: one line in
  `stages.resolve._config_from_resolved` (another package's this round). With it a
  mechanism's default could be the kit again, and a DYNAMIXEL walker stored before its
  default moved to `u2d2_dynamixel` (the user's call, round 2) would read back with the kit it
  had; until then that one case still reads back with the U2D2.
- **The motor role stays a function of the inputs** (`roles.motor_role`: a crank's servo,
  ranged inputs' steppers), not a design choice: not contained. The controllers
  (`Controller.drives`), `construction.DRIVES` and `verify.cost_floor` already key on
  `roles.MOTOR_ROLES`, but a DYNAMIXEL-servo arm or a stepper-driven crank also needs: a
  `JointDrive` for a servo (its body and horn on a transmission's hub; the `actuator`
  registry offers `StepperSpec`s only) and a crank driven by a stepper through a hub (the
  bolt crank seats a servo horn); the MJCF's actuator chosen per motor, not per kind of input
  (a crank a velocity servo from `ServoSpec`, a ranged input a stepper's position gains);
  strength's jam lever per motor (`crank_loads` the servo's limit, `arm_loads` the steppers'
  holding torque x ratio); and control (`control.servos`' speed mode and crank phases against
  Klipper's ranged, homed axes: a stepper crank needs continuous rotation, a servo arm
  position mode and its IK in servo ticks). This is an open item for ROADMAP (not edited here:
  another package's this round): "the motor role a design choice: servo arms, stepper cranks".
- **The gate** (against `platform-9aae3e3`): every design's parts, plans, DXF and meshes
  identical; the walkers and the arms identical in all. Intended: the two mechanisms' BOMs
  drop the ESP32 kit's 14 lines round 2 added (`hoecken_pantograph` USD 248.59 -> 143.18,
  `dwell_rocker` 240.78 -> 135.37: unnamed, their controller is `none`) and gain the
  servo-firmware note (0.21 and 0.11 N·m); their audits OK. `klann_quad` fails its audit as
  known. DESIGNS.md is not regenerated (no part moved).
