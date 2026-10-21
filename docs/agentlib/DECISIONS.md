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

### 2026-10-10: the decanting cradles (user: "A+B", hand crank, an LED, aluminium first)

- **What:** two hand-cranked cradles on the walker engine, a product (`Linkage.product`,
  `config.PRODUCT_*`): `decanter_hinge` (A) and `decanter_rcm` (B). The design record is
  [DECANTER.md](DECANTER.md). Then three adversarial reviews (mechanical, fabrication,
  code) and a round of fixes.
- **Drive:**
  - an NRV030 30:1 worm box: statically irreversible, Phidgets, $48.10;
  - its stock 14 mm keyed shaft with an arm of two 0.125 in 6061 plates on the key (a
    5.1 mm keyway is illegal in 0.250 in at SendCutSend);
  - the crankpin a 10 mm/M8 shoulder screw: an M6's thread yielded at the jam, SF 0.7;
  - a printed offset hand crank. A crank on the input shaft swept through the crank arm,
    and a handwheel small enough to stay outboard needed 50 N at its grip.
- **Pins and standoffs:**
  - M6 Chicago pins (`chicago6`), not bonded (`ChicagoShaft.bonded`): the barrel's head is
    in reach to hold, so the order drops its epoxy;
  - every pillar, column and tie a MISUMI NETRF10. A 6 mm standoff bent at B's jam; the
    10 mm is 4.6× as strong and cheaper.
  - A cradle's printed rings fill their layers, so every column is bought at its gap.
- **Sheets:**
  - frames 0.187 in, 3.4× as stiff across as 0.125 in (0.250 in can't take M5 holes);
  - links 0.125 in, the rod 0.187 in, B's links all 0.187 in.
- **Stand:**
  - A base plate with the decanter in a notch. A tipped at 10 N on the bottle's top; now
    31° and 29 N or more.
  - A's inner plate down its front leg, on four columns.
  - No bay tie over the mouth: it blocked uncorking and loading.
- **Swing:** A re-fitted to −25°..+55° (crank 96.4, rod 239.9, rocker 150): an opened full
  3 L pours at −10°.
- **Bottles:** both bottles' mouths on the pivot. Magnum adapters and a 3 L collar; the 3 L
  had sat 20 mm off and wandered 21 mm.
- **Loads:** every joint rated at the full planar load. `SHARE` 0.6 understated the drive
  side 1.6-2.1×.
- **B:** its −25° re-fit failed (a 90 mm lever over its pins, its rear ties through its
  inner bar), so it stays a −10° study.
- **Why:** the user's mandate, "a nice thing to order and build", and the reviewers'
  blockers: the handle's sweep, the spill at rest, the crankpin's thread in the outer
  plate, the shaft modelled 2.5 mm short, the unbuyable or unfittable printed drive
  parts, B's illegal holes.
- **Where:** `spiderpig/construction/{cradle,handcrank}.py`, `spiderpig/cradle_report.py`,
  `spiderpig/hardware/cradle_catalog.py` (each offer's page and date),
  `tests/test_cradle.py`. The walker designs are unchanged: identity gate against
  `next-320384d`, identical.

### 2026-10-10: the decanting cradles, review round 2

- **What:** a second round of three adversarial reviews (mechanical and safety, fabrication
  and BOM, code), and these changes to A ([DECANTER.md](DECANTER.md) has the design):
  - **The jam case** is a firm 40 N push at the grip through the worm at a hand's speed:
    36 N·m at the crank (`cradle_report.jam_nm`). The gearbox's 20 N·m was reached at less
    than the worst turn's own grip force, so it bounded nothing.
  - **The crank block:** eight 0.125 in 6061 plates from the key up past the shaft's end, a
    NETRF10 tie through them, the crankpin a NETRF10 socketed in them. The rod's force is
    5.5 mm off the socket, not 21 mm off a shoulder screw's M8 thread; crankpin SF 3.1 at
    the new jam, where the old one was 1.0-1.25 at the old (the report had taken the
    screw's 965 MPa strength as its yield and left out the preload). The block also puts
    the keyed plates on the key's straight flanks, ends the crankpin's thread over the
    outer plate, and drops the cap, the printed spacers and the jam nut.
  - **The base stop** (a printed spine and cup behind the bottle's base): at rest only the
    liners' friction had held the bottle, and a corkscrew's push slid it.
  - **The base plate** 500 mm across: the stand's numbers had been read at one crank angle
    (round 1's "31° and 29 N" were wrong: 23.6° and 18 N at worst over the swing); now
    31.8° and 29.9 N or more.
  - **The light's clip** a part of its own under the front saddle: merged, it printed 67 mm
    in the air.
  - **The guard:** the outer plate solid over the crank's sweep.
  - **The hinges' play:** 0.3 mm on the pillar (the printed parts had been 0.075 mm longer
    than the steel the plates clamp), 0.25 mm on the M6 Chicago pin.
  - **Pin E stays the M6 Chicago** at SF 1.34 jammed: a 10 mm standoff pin's two M5
    threads don't fit its 11 mm stack, nor a shoulder screw's thread under the inner plate.
    Unbonded: the order's epoxy goes.
  - **The guide:** the drive side on the base plate before the box, the passive inner plate
    last (A has no bay tie), the use limits (headroom, the decanter's envelope, never force
    the crank) in its last step.
- **Why:** the reviews' MAJOR findings, every one: no bound on the hand, the crankpin's
  rating, the bottle held by friction at rest, the stand's numbers, the prints' poses, the
  hinges clamped, the guide's bay tie.
- **Where:** `spiderpig/construction/{cradle,handcrank,assembly}.py`,
  `spiderpig/construction/pivots/{standoff,chicago}.py`, `spiderpig/cradle_report.py`,
  `tests/test_cradle.py`. The walker designs are unchanged (identity gate against
  `next-320384d`).

### 2026-10-10: the decanting cradles, review round 3

- **What:** a third round of the three reviews, and these changes to A:
  - **A product never reuses another engine's plan** (`stages.planning._reuse_plan`): the
    order files had been built on a stored plan from an older engine, valid but two layers
    thicker, on which pin E was at jam SF 0.82 (rated 1.34 on its own plan). A cradle plans
    in seconds, so it re-plans, proven.
  - **The cutting estimate** for a cradle is a fit to SendCutSend's live quotes for A's files
    (`hardware.bom.CRADLE_CUT_FITS`: USD 367.50 quoted; the walkers' per-area rates had said
    USD 1000).
  - **The crank block:** its top recess plate dropped (seven plates), the rod 1.5 mm over the
    end screw's and the tie's heads (the stack's tolerance had left 0.33 mm), 10.3 mm sockets;
    the crankpin's jam SF 2.0. Its play stays 0.3 mm: its washer keeps the rod off the tie
    rods' screw heads under the cradle link, 0.43 mm over it (more room there forced a gap
    that widened pin E's span past jam SF 1); the rod's spacer is sanded, never shimmed.
  - **The base plate's bolts:** M6 carriage bolts in square holes (Bolt Depot 18219): under
    the assembled cradle nothing can hold a screw's head.
  - **Verify rates a cradle's joints** at its own statics (`strength.PER_JOINT`): B fails it.
  - The guide's hand-crank and crank-block steps, the light clip's and brackets' print
    poses, the base stop's spine (shorter, rounded: at rest it comes down to 40 mm over the
    base plate), the use limits (90 mm across up to 270 mm; never lift it by the crank).
  - Tried and dropped: pin E's links in adjacent layers (jam SF 1.9): no plan has them.
- **Why:** the reviews' BLOCKER (the stale plan's order files) and MAJORs (the estimate, the
  guide's text, verify's blind spot).
- **Where:** as round 2's, with `spiderpig/stages/planning.py`, `spiderpig/verify.py`,
  `spiderpig/hardware/{bom,order,cradle_catalog}.py`. The walker designs are unchanged.

### 2026-10-10: the decanting cradles, review round 4

- **What:** a fourth round of the reviews (fabrication: "ready to order, no BLOCKER, no
  MAJOR"; code: three MAJORs), and these changes:
  - **Verify's quick level** (its default, the MCP tool's too) ran on a cradle: its catalog
    floor buys the gearbox and output shaft, not a servo, and leaves the mass and envelope
    to a build (the walk model weighs servos and legs).
  - **One cutting estimate:** the cradle fit's quantity discount is on a file's count, the
    parts merged by area first, so verify's and audit's ungrouped BOMs come to the build's
    (they had said USD 670 for A where ORDER.md says USD 583).
  - **The guide:** every carriage bolt goes in with the base plate; the hand crank right
    after it (the checks then turn it); the passive side set down 7-15 mm out and pushed
    home; the sub-assembly placed on its own layer; the end screw dry until the sleeve has
    settled; the M6 Chicago screws on 243 in the BOM as in the guide.
  - **Fabrication:** the carriage bolts' 7.3 mm square holes drawn with the 0.8 mm round
    corners SendCutSend cuts (the 7 mm ones cleared a sharp neck by 0.04 mm); the brackets'
    neck relief 0.3 mm deeper; the light clip's set-screw boss a rib down to its foot; the
    crank block's tie 0.85 mm short of the top plate; no M5 screw under 10 mm.
  - A walker's strength basis text back to its own words ("no sim joint matched").
  - **The rod and the rear tie's screw head** (the mechanical review): the washer keeps the
    rod 0.26 mm (nominal) under the head where it crosses, inside the chain's +-0.3 mm. The
    crankpin's length now rounds down (0.2-0.3 mm of play) and the guide checks the
    crossing with a sheet of paper; a rub is a scrape, not a jam. Moving the rear ties back
    past the rod's sweep (x 295) was dropped: the rear saddle would meet the base stop's cup
    for a magnum.
  - **The decanter's limits** restated with about 4 mm of radius to the box at rest: up to
    275 mm tall, 80 mm across up to 270 mm, its mouth 60 mm across or more (the old 310 mm
    neck had none at the front saddle).
  - Not done (MINOR, information): a printed skirt over the drive stack's bay face, for a
    household with children; the slow pinch points have their written limits.
- **Why:** the code review's MAJORs (the quick tier red, verify quick crashing on a cradle,
  the overpriced cradle in verify and audit), the fabrication and mechanical reviews'
  MINORs.
- **Where:** `spiderpig/verify.py`, `spiderpig/hardware/bom.py`,
  `spiderpig/construction/{assembly,cradle,handcrank}.py`,
  `spiderpig/construction/pivots/chicago.py`, `spiderpig/strength.py`. The walker designs
  and their guides are unchanged.

### 2026-10-10: the decanting cradles, nearly upright, flat-packed (user)

- **What:** the user asked for photorealistic materials, standard decanters and glasses,
  better-designed side plates (the big disc), a flat pack, and the bottle resting nearly
  upright (−50° to −60°, from −25°). A:
  - re-synthesized for −50° to +30° (`decanters.decanter_hinge`); the mouth at 460 mm, the
    bay 260 mm wide, the saddles 160 mm long (`cradle.LIP_HEIGHT`, `BAY`, `SADDLE_HALF`);
  - its plates a filleted truss, the same file on both sides; the guard over the crank the
    gearbox's flange (the riser's disc), 12 mm past the sweep (later the same day dropped
    at the user's call: a minimal gearbox mount, no guard);
  - the vessel on a lab-jack riser, sized by the room under the mouth
    (`cradle_report.room`), a standard decanter drawn (`cradle.DECANTER_PROFILE`);
  - a flat pack (`spiderpig/cradle_pack.py`) and the cradle page's photorealistic look,
    wine, pour and vessels (`viewer/src/decant/`).
- **Why:** the user's asks. +30°, not +45°: at +20° both bottles drain, and a 95° swing put
  the gearbox at SF 1.13; the rod's buckling in the jam set its length (267 mm, SF 1.52).
- **Trade-offs:** the guard is 12 mm past the sweep where ISO 13857 asks 20 for the layer
  under the block; at rest the cradle box hangs under the mouth, so a vessel left there
  for the whole swing stands about 175 mm under the mouth (closer only while pouring).
- **Where:** `spiderpig/linkages/decanters.py`, `spiderpig/construction/{cradle,handcrank}.py`,
  `spiderpig/cradle_report.py`, `spiderpig/cradle_pack.py`, `spiderpig/server/app.py`,
  `viewer/src/decant/`. The walker designs are unchanged.

### 2026-10-10: cradle B refined to build (user: "B looks really sick, let's keep refining")

- **What:** B (`decanter_rcm`) goes from a study to a cradle to build, on the new targets
  shared with A: −50° (rest, an opened full bottle stands in it) to +30°, `LIP_HEIGHT` 460,
  `BAY` 260. The design is in [DECANTER.md](DECANTER.md), "B, the virtual hinge".
  - **The geometry** (`decanters.RCM`): the pivots hang under the mouth on a ray 83° down,
    109 and 328 mm out. The arms are 204 mm, 23.5° ahead of the bottle. Y sits on the copy
    arm's line through O1. The crank is 480 mm behind and 320 mm under the mouth: crank
    90.9, rod 410.6, lever 143 at 54.5°.
  - **The sheets** (`materials.LINK_SHEETS`): the rod 0.250 in 5052, the arms and the bar
    0.187 in 6061, b5 and b6 0.187 in 5052.
  - **The stand** (`cradle.STANDS`): an outer truss on O1, O2 and O, two feet, a rail. The
    inner plate is the bar O1–O2. No bay tie.
  - **Its standing:** `cradle.STUDIES` is empty, and `cradle_report.VERDICT` says build.
- **Numbers:** transmission 37.7° at worst. The gearbox's SF is 1.31 (a 15.25 N·m peak),
  21.7 N at the grip. Jam: pin P 1.38, pillar O2 1.46, every other joint and plate 1.64
  or more. Walking: 3.9 or more. The rod buckles at 1.8 × its jam force. The stand tips at
  31.6°, or under a 37.9 N push. The only clashes are the cradle box against the drawn
  decanter at rest, which comes from the decanter's own size.
- **Why this layout:**
  - **The ties never cross the inner bar.** The rear ties ride 292–320 mm out and never
    come further round than 72° down; the bar lies at 83°.
  - **The cradle link never sweeps a pivot**, and each pair (b2 with b3, b4 with b5)
    shares a layer. The planner then finds the old sandwich, every pin across one gap.
  - **The handle turns behind everything**, the crank being the rearmost moving part.
  - **The gearbox and the jam pull against each other.** The gearbox's peak comes from a
    sealed 3 L near level, at the stroke's fastest. Equal strokes need a long rod for the
    crank. The rod's force at a jam falls with a longer crank. A 90.9 mm crank on a
    410.6 mm rod balances the two at 1.31 and 1.38. The first targets' 95° swing (to +45°)
    held the gearbox to SF 1.13, so full tilt became +30°.
- **Tried and dropped:**
  - **Pivots in front of the mouth:** the crank lands in front, and the handle turns back
    through the parallelogram.
  - **Pivots past every tie** (350 mm out or more): b4 grows past 400 mm, and the walk loads
    triple.
  - **A 300 mm rod in 0.187 in:** the jam force stays over 950 N.
  - **376 mm on a 109 mm crank:** jam 1.44, but gearbox 1.17.
  - **405 mm on a 67 mm crank:** gearbox 1.33, but jam 1.1.
  - **The old diagonal ray (−40°)** with the pivots inside the ties: the cradle link can't
    reach its rear ties without sweeping over a pivot.
- **Where:** `spiderpig/linkages/decanters.py`, `spiderpig/construction/cradle.py`,
  `spiderpig/materials.py`, `spiderpig/cradle_report.py`, `tests/test_cradle.py`,
  `tests/test_mechanisms.py`. Only B's own code and data change: the walkers are
  untouched.

### 2026-10-10: regular 750 ml bottles too; a minimal gearbox mount (user)

- **What:** the cradles take a 750 ml bottle as well (`cradle.STANDARD`, a Bordeaux, 300 x
  76.6 mm): its front adapter liner ends at its shoulder, the stop for a neck shorter than
  `cradle.STOP_X`; its rear one is closed behind its base, which ends inside the rear
  saddle (`cradle.adapter`, `_adapter`). The gearbox sits on its small riser discs, with
  no guard flange (the user: "the worm gear attachment is still a big disk").
- **Kept, the user's answers:** the 3 L and the −50° rest, so the mouth stays near 460 mm
  (the 3 L's length hangs below the mouth at rest); B, now the main design, is slimmed
  around them (its footprint and stack), not lowered by dropping a bottle.
- **Where:** `spiderpig/construction/{cradle,handcrank}.py`, `spiderpig/cradle_report.py`
  (every bottle's outline for the cradle page), `viewer/src/decant/pour.ts` (a bottle
  picker), `tests/test_cradle.py`.

### 2026-10-10: cradle B slimmed (user: B is the main design; slimmer and lower, the 3 L and the −50° rest kept)

- **What:** B's crank moves from 480 mm to 366 mm behind the mouth. The base plate goes
  from 827 × 500 mm to 596 × 500 mm, and the stack is thinner.
  - **The geometry** (`decanters.RCM`): the pivots hang straight under the mouth (a ray
    90.5° down, 105 and 307 mm out). The arms are 174 mm, 8° ahead of the bottle. The crank
    O is 366 behind and 322 under the mouth: crank 93.5, rod 336.2, lever 147 at 64°.
  - **The cradle link b6** is 0.125 in 5052 (`materials.LINK_SHEETS`): it is alone under
    the inner plate, at jam SF 6.
  - **The stand** (`cradle.STANDS`): the rear leg runs straight down from O, and the base
    plate spans 160 mm in front of the mouth to 436 mm behind it.
- **Numbers** (the box's 160 mm saddles, the minimal gearbox mount):
  - The gearbox's SF is 1.38 (a 14.46 N·m peak), 20.6 N at the grip.
  - Jam: pin P 1.36, pillar O2 1.42, pillar O1 1.50, links b3 and b4 1.5. Walking 4.1 or
    more.
  - The rod buckles at 2.67 × its jam force. Transmission 36.7°.
  - The stand tips at 30.1° (a full 3 L), or under a 26.6 N push.
  - The stack is 1.6 mm thinner (b6).
- **Why:**
  - With the rod nearly level, the crank sits a rod's length behind the lever, and the
    stand runs to the crank.
  - Equal strokes (the gearbox's peak) want a long rod for the crank, and a jam a long
    crank. The lighter box (160 mm saddles) gave the gearbox room, so the rod could shorten
    to 336 mm and keep both at SF 1.35 or more.
  - The lever must stay off b3 so that b2 and b3 share a layer. Nearer cranks (280–320 mm
    behind) crossed b3: the plan put b3 between the rod and b2, and pin P's longer span
    fell to jam SF 1.2.
- **Tried and dropped:**
  - **A 300 mm rod in 0.187 in:** buckling 1.34.
  - **The crank in front of the mouth:** pillar O2 at jam SF 0.8.
  - **b4 and b5 in 0.125 in:** b4 jam SF about 1.3. Not tried in full.
- **Where:** `spiderpig/linkages/decanters.py`, `spiderpig/construction/cradle.py`,
  `spiderpig/materials.py`, `spiderpig/cradle_report.py`, `tests/test_cradle.py`,
  `tests/test_mechanisms.py`. Only B changes.

### 2026-10-13: the flat pack, rewritten (user: "look at the packing and make it better")

- **What:** `spiderpig/cradle_pack.py` packs again from scratch ([DECANTER.md](DECANTER.md),
  "Flat pack").
  - **More pieces:** the cradle box comes apart where it is fastened: its tie screws, the
    saddles' set screws, the light's and the base stop's screws. Each cradle link lies
    flat with its loose spacers.
  - **Levelled:** each such piece turns in its plane to its least bounding box.
  - **The panels** drop onto a heightmap and nest.
  - **The rest** fill a voxel grid of the box, cavities included, in an order they could
    go in.
  - **A hanging link** (B's b5, on J2 alone once its cradle link is off) swings into its
    panel.
  - **The steps** are read off the parts and the cradle link's joints. B's had been A's:
    its cradle link b6 is pinned at X and Y, not on a pillar.
- **Numbers** (outside, x × z × height, at e2597e9's parts):
  - B: 615 × 547 × 516 mm (173 L) became 618 × 517 × 207 mm (66 L).
  - A: 656 × 516 × 363 mm (123 L) became 658 × 518 × 189 mm (64 L).
  - `flat_pack` takes about 2-3 s after the fabrication.
- **Why:**
  - The cradle box travelled whole and upright, about 391 mm tall: half the box was air.
  - The pieces sat at odd heights on the bounding boxes of diagonal links.
  - Apart and levelled, the cradle box's pieces fit in the cavities under the panels'
    plates. The panels' pillars stand between each other's gearbox and brackets.
- **Trade-offs:**
  - It packs bounding boxes, one per part: a diagonal link's box is large, and a truss
    plate's has no holes. Foam fills the rest.
  - Every piece goes in straight down; nothing slides in sideways.
  - The cradle box goes back together at the other end, from the fasteners it was built
    with (a fresh drop of threadlocker on the tie screws).
- **Where:** `spiderpig/cradle_pack.py`, `viewer/src/decant/{pack,main}.ts`,
  `tests/test_cradle.py`.

### 2026-10-13: the cradles carry a 6 L bottle (user: "we need to fit 6L magnums"; "both")

- **What:** the user asked for 6 L bottles. Asked which, a Bordeaux impériale or a
  Champagne methuselah, they answered "both". The methuselah (571 x 174 mm, 11.1 kg full)
  is now the design bottle; every other size lies on adapters
  ([DECANTER.md](DECANTER.md), "Sized for a 6 L").
  - **Geometry:** the mouth 550 mm over the table (from 460); the saddles 190 mm across,
    the rear one 125 mm long and a V shell; the tie points lower and further back; the base
    stop's spine to the 6 L's base.
  - **The drive:** the NRV040 at 30:1 (Phidgets TRM4401_0, 45 N·m) for the NRV030; a
    100 mm handle (A 105).
  - **The drive path:** NETRF12 pillars and crankpin; B's b2-b4 in 7075-T6; the rods
    0.250 in 5052 and every link 24 mm wide (B's rod's buckling; a 0.313 in rod failed the
    web rule round its pin); four keyed plates; steel pins (the user's choice).
  - **The stand:** the base board 530 x 626 mm.
  - **The base board** in 9 mm Baltic birch plywood (the user's, the same day), routed at
    SendCutSend, with M6 x 20 carriage bolts in round holes.
- **Numbers** (B at 5167f80, the pins aside):
  - The crank's peak 32.0 N·m (the NRV040's SF 1.41), 23.7 N at the grip.
  - The jam 66 N·m: pillars O1 1.38, O2 1.30; links b3 1.45, b4 1.46; the crank's keyway
    2.0; the crankpin 1.81.
  - The stand tips at 25.9°.
- **Why:**
  - A full 6 L's moment about the mouth is 2.2 x the 3 L's, and the NRV030 is rated 20 N·m.
  - The handle force bar (under 25 N) and the jam push (40 N, about twice the worst turn)
    fix the jam at about twice the peak, whatever the ratio or the handle's length. So the
    drive path needed about twice its strength, not just a bigger gearbox.
  - A per-cradle handle: A's worst turn took 24.98 N on a 100 mm handle. A longer handle on
    both would have pushed B's pillar O2 under 1.3.
- **Tried and dropped:**
  - **Designing for the impériale alone:** the mouth up only about 15 mm and 1.5 x the
    loads, but a Champagne 6 L wouldn't fit (the user: both).
  - **A slip clutch on the handle:** to keep normal turning it must pass the worst turn's
    torque at the worm's static efficiency. At the hand's efficiency that still lets 1.6 x
    the peak through, so the parts need upgrading anyway.
  - **A counterbalance spring:** it would over-balance a 750 ml, and it's a new mechanism.
  - **M8 Chicago screws for the pins:** 1.69 x the section, and rod pin P only about 1.25.
- **Where:** `spiderpig/construction/cradle.py`, `spiderpig/construction/handcrank.py`,
  `spiderpig/construction/pivots/standoff.py`, `spiderpig/linkages/decanters.py`,
  `spiderpig/cradle_report.py`, `spiderpig/config.py`, `spiderpig/materials.py`,
  `spiderpig/hardware/{cradle_catalog,sheet_catalog,fasteners}.py`, `spiderpig/bake.py`,
  `viewer/src/decant/`, `tests/test_cradle.py`.

### 2026-10-13: the cradles' pins, hardened steel pivot pins (user: "Steel shoulder screws")

- **What:** every cradle pin is a hardened 8 mm steel pivot pin made to its stack, an E-ring
  at each end: MISUMI CCGH8-L-NC2.4 (`pivot8`, `construction.pivots.pivot_pin`,
  `config.PRODUCT_PINS`). The M6 Chicago screw (`chicago6`) stays registered.
- **Why:** at a 6 L bottle's jam (66 N·m at B's crank, 69.3 at A's) the M6 Chicago's 8 x 6
  stainless barrel bent: B's pins P 0.74 and K 0.93, A's E 0.65. The pin is solid 1045 at
  40-45 HRC, rated at class 8.8's minimum yield (640 MPa, conservative for that hardness):
  4.3 x the barrel. On the same plans, jam SF: B's P 3.51, K 4.48, J2 6.7, Y 7.3, X 8.5,
  J 10.6; A's E 3.12. Walking 18.4 and more. (The links keep their 8.2 mm holes.)
- **The user's pick, tried first:** an ISO 7379 shoulder screw (McMaster 92981A, 8 mm, M6,
  black-oxide alloy steel, USD 2.81-6.84). It was built and planned. Its 6 mm head over the
  links and its 11 mm of thread, washer and nut under them take whole layers or a 6.3 mm
  gap:
  - A planned at 62.3 mm (59.4 before, at the 10 mm pillars and 0.187 in rod of the day);
  - B at 88-91 mm (77 before), its links spread out (pin X's span 40 mm), its rod off the
    crank block (the crankpin at jam SF 0.32) and its pillars weaker (O1 / O2 0.60 / 0.57
    on NETRF10).
  The pivot pin is steel with as wide a margin and keeps the stacks, the same layers, 0.2 mm
  thinner than with the Chicago screws: B 79.95 mm (80.15), A 63.95 (64.15).
- **The end:** the standard pin stands 3 mm past each groove; with the play, a hair over a
  3.175 mm layer, so every end of B's took a clearance gap (94.5 mm). The alteration NC2.4
  (the least for 8 mm) sinks every end into its layer.
- **Why not another solid pin:** tapped 10 mm standoffs, rotary shafts and clevis pins start
  at 15 mm long (MISUMI NETRF10, rotary shafts, both-ends-tapped clevis pins); the pins'
  stacks are 11-16 mm. A 10 mm pin would also cut the links' net section. The pivot pin
  starts at 10 mm, in 0.1 mm steps, and keeps the 8.2 mm holes.
- **Cost:** USD 24.05 one-off, 22.42 at 2-4, 21.36 at 5-19 per length (the NC2.4
  alteration 2.31 of it; pages rendered 2026-10-13), against the shoulder screw's ~3 and the
  Chicago screw's ~0.50. B buys 11 pins in four lengths (USD 241.89), A one. Steel only
  at the pins the Chicago screw can't hold (B's P and K, A's E) would buy 4; not done.
- **Where:** `spiderpig/construction/pivots/pivot_pin.py`,
  `spiderpig/hardware/cradle_catalog.py` (its own block), `spiderpig/cradle_pack.py`,
  `tests/test_cradle.py`.

### 2026-10-13: cradle B packs as three modules (user: "3 pieces plus the crank with no small parts")

- **What:** the user's choice: "The passive side panel unpins from the cradle box at 2
  quick-release pins. Both side panels lift off the base board at 4 knob nuts, and the
  hand crank pulls off. The drive side and cradle box travel as one unit. That's 3 pieces
  plus the crank with no small parts, but a box of roughly 200 L."
  ([DECANTER.md](DECANTER.md), "Flat pack" and "The quick-release pins").
  - **The pack** (`cradle_pack.module_pieces`, `PACK_TURNS`, `pack_t`): B is cranked to 4
    handle turns from rest, lifted off its board, its two clips pulled, its passive side
    slid off. The pieces: the base board, the passive side panel, the drive side with the
    cradle box, the hand crank, and the other bottles' nine adapters and collars, loose.
    The payload says its `mode`, its `pose` (the viewer poses the clip there first) and
    `slide_off`. A keeps its flat pack: its cradle link rides pillar G.
  - **The pins at X and Y** (`clevis8`, `construction.pivots.clevis`, both sides): an 18-8
    clevis pin 8 x 20 (McMaster 90378A701, USD 5.47), its head inside the stack over a
    printed take-up spacer, a hairpin cotter (98350A131, 5 for USD 3.68) through it from
    the bay. The planner's new `tips` hook claims its end in the inner plate's open (a seat),
    and `CradlePlates` keeps the plate off its path as off the tie rods'. Its usable length
    is capped as a Chicago barrel's (`chicago.MAX_BARREL`): uncapped, the 25-44 mm stock let
    the planner spread X's links 30 mm apart.
  - **The base board's nuts:** a knurled thumb nut (DIN 467 form, 24 x 6, McMaster
    92815A203, USD 1.69) on an M6 x 25 carriage bolt (Bolt Depot 18221, USD 0.26), 5 mm of
    thread past it; the bolt 22 mm off the outer plate (from 19) so the knurl clears the
    bracket's M5 screw head. The brackets' M5 screws stay: each bracket travels on its side.
  - **The hand crank:** an M3 x 9 knurled thumb screw on the input key (McMaster 92545A115,
    18-8, USD 1.76) for the M3 set screw; `cradle_report.wheel_sweep` sweeps its head.
- **Numbers** (B):
  - The box: 643 x 611 x 478 mm outside, 188 L, 13 pieces (it was 643 x 547 x 318, 112 L,
    27 pieces).
  - The pose: 35° neck up, b6 R 14.6 mm clear of the bar O1-O2 in plan (38.3 at full tilt;
    it overlaps from rest to 46° neck up). The sweep of the panel along z meets nothing.
  - The stack: 79.68 mm a side (80.15): the clips' gap is 2.7 mm (the Chicago heads' 3.2),
    and the layer between b4/b5 and b6 a 3.2 mm gap.
  - Jam SF at 66 N·m: X 2.70, Y 2.34 (Chicago 1.72, 1.49); every other row as before but
    pillar O2 1.33 (1.31), O1 1.41 (1.39). Walking 4.1 and more. The gearbox SF 1.41.
- **Why:**
  - **The pose:** full tilt clears most (38 mm), but the cradle box stands up 290 mm over
    the mouth there: a 259 L box. 4 turns from rest is about 190 L, with 14.6 mm of
    clearance (3 to 6 turns all clear).
  - **The pin:** the cheapest tool-free one that keeps the stack. MISUMI's one-end-ring,
    other-end-cross-hole pin is made to length but from USD 17.62 (unhardened); its headed
    clevis pins start at 18 mm usable (USD 9.36); a ball-lock pin's tip would stand into
    the layers past b4. The 15 mm stock pin needs a 3.5 mm printed spacer under its head:
    the head and spacer reach into the layer of b2 and b3, which never come near X or Y.
  - **The nut:** a wing nut's wings (DIN 315 M6: 30-33 mm; McMaster's American style 28
    mm) hit the bracket's upright 14 mm off the bolt. Bolt Depot sells no metric thumb nut.
  - **The order:** the panel's brackets sit on the board's bolts, so the cradle comes off
    the board before the panel slides off (the user's list had the slide first).
- **Cost:** the four clevis pins USD 21.88 and a pack of clips 3.68, for a pack of 20 M6 x
  12 Chicago screws (8.50); four thumb nuts 6.76 for four thin nuts 0.40; the bolts 0.03
  more each; the thumb screw 1.76 for a 0.09 set screw. About USD 25 more.
- **Where:** `spiderpig/construction/pivots/clevis.py`, `spiderpig/construction/axle.py`
  (`tips`), `spiderpig/construction/cradle.py` (`_bracket`, the inner plate's keep-outs),
  `spiderpig/construction/handcrank.py`, `spiderpig/config.py`,
  `spiderpig/hardware/cradle_catalog.py`, `spiderpig/cradle_pack.py`,
  `spiderpig/cradle_report.py`, `viewer/src/decant/{pack,main}.ts`, `tests/test_cradle.py`.

### 2026-10-14: the cradles' saddles adjust; the adapters go (user: "Two knobs plus the stop replace all the adapters")

- **What:** the user's choice: "Each saddle's V rides up or down on a thumbscrew, with a
  scale marked by bottle size. A flip-down lip gauge marks where the mouth goes, and the
  existing base stop locks the bottle there. The top stays open, so bottles still lay in
  from above." The 10 printed adapter liners and collars are gone
  ([DECANTER.md](DECANTER.md), "The saddles, adjustable").
  - **The saddles:** each is a carrier on its two tie rods and a V block that rides up and
    down in it on a thumbscrew, with a scale beside it (`cradle.setting`, `cradle.SADDLES`).
    - The front block slides in a box guide on an M6 screw: 15 mm of travel, the 6 L's neck
      at the bottom and the 750 ml's 12.7 mm up.
    - The rear block rides a NETRF12 post and an M8 lead screw: 72 mm of travel, 68.9 mm up
      for a 750 ml.
    - At a 6 L methuselah's setting both sit on their tie rods. The carriers' bottoms are
      the old saddles', and `decanters.TIES` didn't move, so neither cradle re-plans.
  - **The lip gauge** (`cradle._gauge`) flips down from the front carrier onto the top of the
    mouth's rim and stays down while pouring: it is the forward stop.
  - **The base stop** (`cradle._rails`, `cradle._gate`) is a gate on the rear V block's flats
    and the rails behind it, clamped anywhere by an M5 in a slot. It serves every base from
    285 to 576 mm, inside the rear V too.
  - **The model** draws a 6 L methuselah (`cradle.SHOWN`), the V blocks at their lowest:
    the envelope for the clashes, the room and the loads. `cradle.at_setting` sets it for
    another bottle (`cradle.bottle_hits` checks every size), and the cradle page moves the
    nodes (`motion`'s `bottles`, `adjust`).
- **Why:**
  - **The thumbscrew, not detents:** it is exact for any bottle and adds no depth. A 5 mm
    detent pitch puts the mouth up to 0.8 mm off the pivot.
  - **Not the tie rods in vertical slots:** the slot runs the whole travel under the rod, so
    the rear block would hang 69 mm deeper, onto the decanter's axis at rest.
  - **The gauge as the forward stop:** a lip over the base's heel can't hold a bottle
    sliding toward its mouth, because the heel tapers toward the base. Only the shoulder
    and the mouth's face face the mouth. A shoulder stop would be a third adjustment, 45 mm
    along the axis, where the light stands. The gauge is on the pivot for any bottle, its
    pad over the bore, the stream below it, its load along its hinge.
  - **The 12 mm post:** the rear screw holds its block at one end, so a bottle's weight at
    the axis rolls the block, about 11 N·m with a 6 L hung on the screw. On a NETRF10 post
    that is yield SF 2.3; the NETRF12 gives 3.4. Gussets carry the couple into the block.
- **Numbers** (2026-10-14):
  - The saddles at a full 6 L's worst: printed parts at 0.2-5.4 MPa against a 10 MPa
    allowable for a held load; the post 3.4 and the screw 6.0 x their yield; the gate's
    lock grips 3.7 x its load.
  - Clash-free over the swing at every size's setting, on A and B.
  - The room is no smaller: 250 mm across up to 165 mm, 125 across above, the rim at most
    380.
  - The lowest point at rest is unmoved, 18.8 mm over the board (the rails' end).
  - About USD 23 more bought per cradle (`hardware.cradle_catalog`, pages rendered
    2026-10-13), the post most of it.
- **Corrections to 2026-10-13's numbers:**
  - **The load case kept the drawn magnum's adapter liners.** `cradle_report.masses` was
    asked to leave the adapters out when `any(adapter(bottle))` was false. For the 6 L it
    was `(0, 0, 43)`: its collar made it true, so the magnum's 1.25 kg of rear and front
    liners stayed in the 6 L's load case. Without them the crank's peak is 31.3 N·m on A
    (SF 1.44, was 1.33) and 29.5 on B (SF 1.52, was 1.41); 22.1 and 21.9 N at the grip.
  - **The stand counted the spare adapters as its own weight.** The 4 collars and 8 spare
    liners drawn on the table, about 3.8 kg of solid PLA on the frame, were in
    `cradle_report.stability`'s standing mass. They lowered its centre of mass, so a full
    6 L "tipped past 25°" (A 25.6°, B 26.0°). Without them: A 21.3°, B 20.0°, both over
    the rear edge.
    - The coordinator's call: grow the board, keep the bars. `cradle.BASE_HALF` 265 → 292
      (584 mm across); the rear edge 440 → 482 mm behind the mouth on A, 466 → 524 on B.
    - Now A tips at 25.3° (a 30.2 N push at the bottle's top), B at 25.8° (31.6 N).
    - The room behind the base board: 100 mm (was 150: the base stop's rails reach 84 mm
      past A's edge, 42 past B's).
- **Tried and dropped:**
  - **A two-rail cradle**, its rails' spacing set by one screw: the V opens instead of
    rising, the rods guide it, and a fixed spine passes between the rails. But it isn't the
    user's "V rides up or down", and it needs a left-hand thread or a rack.
  - **Two screws per V linked by a belt in the carrier's floor:** no roll couple, but a
    belt to tension in printed parts. The front ties sit 20 mm apart, too close for one
    between them.
  - **A shorter rear V with the 6 L's base carried by the stop:** a third support. Its
    height decides how much the bottle tips, so the mouth leaves the pivot.
- **Where:** `spiderpig/construction/cradle.py`, `spiderpig/cradle_report.py`,
  `spiderpig/cradle_pack.py`, `spiderpig/hardware/cradle_catalog.py`,
  `spiderpig/hardware/bom.py`, `viewer/src/decant/{motion,pour,diagram}.ts`,
  `tests/test_cradle.py`.

### 2026-10-14: the cradles' saddles made to work (design review round 1, mounting)

- **What:** the user's adjustable saddles kept (each V on a thumbscrew with a scale, the
  lip gauge and the base stop set the bottle), fixed where review round 1 found them
  failing ([DECANTER.md](DECANTER.md), "The saddles, adjustable"; the review
  `/root/scratch/decant/review5/r1/mounting.md` and the stand's M5 on the development box).
  - **The rear support runs to 588 mm.** The base stop's two rails, rising with the rear V
    block, carry 45° flanges lined with TPU that continue its V; each rail's tongue sits in
    a socket under the block's flats (`cradle._rails`, `cradle.FLANGE`, `cradle.SOCKET`).
    `cradle_report.balance` checks every size at rest, level and full tilt.
  - **The rear thumbscrew is a Tr8 x 8 lead screw** (9 turns, was 55), its bronze barrel
    nut in the block, its clamping collar on a TPU brake washer in the carrier (the block's
    load on it holds the screw: an 8 mm lead isn't self-locking). Its knob moved outboard to
    |z| 110, over a tower whose arm holds the screw's top (`cradle._lead_screw`,
    `cradle.REAR_LEAD`, `cradle.TOWER`).
  - **The base stop sets by a scale, locks by hand:** a groove per bottle across the +z
    flat and rail (a dot each per place), the gate's pointer on it; a knob on an M5 tap bolt
    through its ear into a nut and washer in a T-slot (`cradle._gate`, `cradle.TSLOT`). The
    43 printed pilots and the hex key are gone.
  - **The lip gauge:** its bar beside the neck (a bottle lays in under nothing), two
    knuckles either side of an 18.4 mm post on a through bolt (the push in bearing, its turn
    on the pin), a lug that stops it at 70° open with its centre of mass inboard: it falls
    back shut (`cradle._gauge`, `cradle.gauge_return`).
  - **The liners are pegged** into their Vs (`cradle.PEG`); three named grooves in front
    (`cradle.FRONT_GROUPS`), dots on every scale; drains in the front carrier; the V blocks
    print standing; the rails print apart; grease on the thumbscrews (the BOM's notes).
  - **The procedure:** set all three by their scales with the bottle out, lay it in onto
    the gate with the gauge down; lift the gauge only to uncork.
  - **A finger's gap under the base stop** (round 1's stand review, ISO 13854's 25 mm, left
    to the base stop): the flanges and their liners end at 588 mm (past a 6 L's heel at
    561) and the +z rail's keel is chamfered at its end, so at rest, set for a 6 L, the base
    stop comes no nearer the board than 26.4 mm on B and 26.5 on A (the rails' end had been
    18.8). `cradle.finger_gap`, checked at every angle of the clash test; the use limits no
    longer list it as a pinch, and say the gauge falls back (to uncork: hold it on its stop
    or rest it on the corkscrew).
- **Why:**
  - **A full 6 L tipped nose-up at rest (B1).** The gate pushes the base below the axis,
    a couple that moves the saddles' reaction toward the base by its depth x tan 50°: at the
    review's centre of mass (390 mm from the mouth for the methuselah, not the 331 of a
    42 % guess) to 482 mm, past the rear V's end at 380. Lengthening the block would have
    needed two prints; the rails already rise with it.
  - **The coarse screw, not detents:** the mouth follows the rear V 0.25 mm per mm (the
    front 1.25), so the rear's fine M8 thread bought nothing; pins in holes (the review's
    S1) aren't the user's thumbscrew. The front keeps its self-locking M6.
  - **A T-slot nut, not a ratchet:** set by the scale with the bottle out, the gate goes
    exactly where it should (half a millimetre behind the base); a 2 mm ratchet would let
    the bottle slide up to 2.5 mm onto the gauge mid-pour.
  - **The gauge falls back** rather than staying up: left up, a wet 6 L slides about 45 mm
    toward the decanter at full tilt (the review's M4).
- **Numbers** (`cradle_report.saddles`, 2026-10-14):
  - Every bottle's weight centres on its support by 62 mm or more (at rest, the
    jeroboam's), against 102 mm past it for the methuselah before.
  - Printed parts at 0.2-8.4 MPa against 10; the post 3.05 x its yield rolled with a 6 L
    hung on the screw, 3.1 x at rest with a hung impériale (a row the review asked for,
    m2); the lead screw 3.8 x at a 50 N knock (the M8 yielded at it); the brake 4.1 x the
    screw's turn back; the lock's grip 3.1 x a 6 L's push at rest.
  - Clash-free on A and B at rest, full tilt and mid-swing, every size at its setting.
  - About USD 14.70 more bought per cradle (the lead screw, its nut and collar from
    ServoCity, pages rendered 2026-10-14).
  - 0.3 kg more on the cradle box (solid, the rails' flanges most of it; they were slimmed
    from 0.46 kg to 0.28: a 12 mm bar, a keel only round the T-slot): with a full 6 L at the
    shell model's 390 mm the crank's peak 34.3 -> 35.1 N·m on B (gearbox SF 1.31 -> 1.28,
    under its 1.3 bar; 25.4 -> 26.0 N at the grip) and 36.0 -> 36.9 on A (1.25 -> 1.22;
    26.0 N): past the drive's relaxed bars (B's gearbox, both grips), left to the drive's
    owner, not relaxed here. The stand tips at 25.0° on B, 24.1° on A, over the rear edge.
- **Tried and dropped:**
  - **The free lead screw guarded by a sleeve** (the review's M2 fix): the block's boss
    rides up the screw, so a sleeve round it blocks the travel; the tower holds the screw's
    top instead.
  - **The gauge's return with its bar over the neck:** a bottle couldn't then lay in with
    the gauge down, and holding it up while lowering a 6 L is the third hand again.
  - **A boss bearing on the tower to take the post's pitch at rest:** with clearances, the
    stiffer post takes it all; the post is rated for it instead.
- **Not done (the review's):** the rails' length per bottle (m10), the light (m6, S4), the
  tie rods (S5), a deeper post socket (m2).
- **Where:** `spiderpig/construction/cradle.py`, `spiderpig/cradle_report.py`,
  `spiderpig/hardware/cradle_catalog.py`, `spiderpig/hardware/bom.py`,
  `tests/test_cradle.py`.

### 2026-10-14: the cradles' stand, use and breakdown (design review round 1)

- **What:** the fixes to round 1's stand-and-use review, with the base board, cost and
  over-constraint findings of its parts and linkage reviews; B mainly
  ([DECANTER.md](DECANTER.md): "The base board", "The vessel under the mouth", "Flat pack",
  "Building one").
  - **The breakdown no longer drops the drive module** (`cradle.module_steps`, used by
    `cradle_pack.module_pieces` and the guide's two new last steps). Standing, the 6.8 kg
    drive module tipped onto its cradle box once the pins cleared b6 (its centre of mass
    66 mm past its one rail), and the panel fell the other way. Now the hand crank comes
    off, then the cradle comes off the board by its outer plates' top members and is laid
    on its drive side on a towel, the gearbox down. The clips come out from the bay, and
    the passive panel lifts straight up off the pins, level, the pins in it. Lying so, no
    link can swing. The drive module stays lying as it goes in the box.
    - **Not on the passive side, as the review proposed** (and as the coordinator's brief
      passed on). Each quick-release pin's head lies on its link's outer face (z 152-155
      on the passive side, `construction.pivots.clevis`), with nothing under it in plan for
      50 mm. With that side down and its clip out, the pin drops out of its link, and the
      spacer between the links is left loose. With the drive side down the pins stand on
      their heads in the panel; the two spacers stay on the cradle link and go back on the
      pins with the clips.
  - **Putting it back together is written down** (the payload's `reassembly`, the cradle
    page's list, the guide's "Putting it back together"):
    1. The drive module lies on its drive side.
    2. Paint marks, made at the first breakdown, set the panel's links; masking tape holds
       them while it turns over.
    3. The spacers go over the cradle link's holes, and the panel is lowered level: pin X
       first, then b5 is nudged until Y drops in.
    4. Both clips go in.
    5. The cradle goes onto the board between its cleats.
    6. The passive side's knobs are snugged last, after a whole pour.
    7. The first build's checks again, with a grip-force bound for the empty cradle
       (`cradle.EMPTY_GRIP_N`, the report's `empty_handle_force_n`).
  - **The pack pose is a mark you can see:** the crank's arm along the drive side's outer
    plate's rear leg (`cradle.PACK_POSES`, `pack_pose`). That is about 5½ turns from rest
    and 31° neck up.
  - **One jack height for the whole swing:** the raise-after-−20° step is gone from
    `cradle.USE_LIMITS`, the guide's "Decanting" step and `viewer/src/decant/pour.ts`.
    - The jack locator (`cradle._jack_locator`) centres a 100 x 100 mm jack's base under
      the mouth. It is two glued layers of the board's birch in the notch, with ears in the
      notch's bays.
  - **The board** (`cradle._base_plate`):
    - B's sits 40 mm toward its drive side (`cradle.BASE_Z`).
    - Its corners are rounded R35 and the notch's mouth R8.
    - The carriage bolts' holes are routed squares with dog-bones.
    - Each rail sits between two glued birch cleats (`cradle._cleats`).
  - **The brackets** (`cradle._bracket`):
    - The foot's hole is 8.5 mm, 1.25 mm of play.
    - An 18-8 fender washer goes under a printed thumb knob, with an 18-8 M6 nut pressed
      into the knob (`cradle.KNOB`). It replaces the black-oxide thumb nut.
  - **The use limits** (`cradle.USE_LIMITS`) now say:
    - where it pinches: the bay faces at each return to rest, the crank's arc outside the
      outer plate, and the rails 18 mm over the board, under ISO 13854's 25 mm;
    - to free a trapped finger by turning back;
    - to take the handle off when the cradle is left (the child lock);
    - that letting go doesn't stop the stream;
    - to check the lip gauge is down before level;
    - that sparkling wine under pressure doesn't go in corked;
    - to check the knobs, and what to do if one works loose;
    - that the feet rock on a counter that isn't flat;
    - how to lift it.
  - **Cost honesty** (`hardware.bom`):
    - The BOM's and ORDER.md's totals say "The total excludes N unpriced cut lines", each
      line with its area (`unpriced_cut_note`).
    - Where a fitted sheet bounds an unpriced line, the note gives an estimate marked as one,
      with its basis (`CRADLE_CUT_PROXIES`, `cut_ranges`): the 7075 at the 0.187 in 5052
      fit, x1 to x2.
    - The birch stays unpriced.
    - `CRADLE_CUT_SOURCE` names A's quote as A's. It had printed A's "USD 359-368 in all"
      in B's order as if it were B's.
    - A walker's BOM is unchanged: `Bom.cut_ranges` is None.
- **Numbers** (B, 2026-10-14):
  - **The stand** (`cradle_report.stability`'s new `edges`):

    | edge | tips at | was |
    |---|---|---|
    | rear (a full 6 L) | 27.2° | 26.1° |
    | drive side | 29.9° | 26.0° |
    | passive side | 29.3° | 31.7° |
    | front | 39.5° | 39.1° |

    - The least push at a bottle's top is 39.4 N, up from 32.0 N.
    - The rear edge binds. The board's new parts (cleats, locator, knobs: 0.66 kg, low and
      forward) gave it 1.1°.
    - Each 10 mm a full 6 L's centre of mass moves toward its base costs the rear about
      0.5°: 26.7° at 10 mm, 26.2° at 20, 25.2° at 40. Past about 45 mm the rear edge
      needs the board longer at the back. `python -m spiderpig.cradle_report stand`
      re-runs the stand alone.
  - **The pack:** 702 x 606 x 485 mm outside, 206 L (it was 703 x 611 x 478, 205 L).
    - The knobs, at 17 mm over the board, and the cleats raise the panels 6 mm.
    - At 31° neck up b6 clears the bar by about 18 mm (it was 14.6 at 35°). The
      `slide_off` sweep meets nothing.
  - **The parts:**
    - Birch: 7 parts, 4439 cm² (the board, 4 cleats of 452 x 26 mm as one file, the
      locator's 2 layers as one file).
    - Bought: 4 more 18-8 M6 nuts and fender washers (USD 0.92) and a bottle of Titebond
      II (5.49), for 4 thumb nuts (6.76). The estimated total is USD 659.05 (it was
      659.40), plus the 7075's USD 68-136 estimate; the birch is unpriced.
  - **The empty cradle** turns with 3.6 N at the grip.
- **Why:**
  - **A mark, not a count:** rest is a toggle dwell; the bottle moves under 2° over its last
    1½ turns.
    - "Rest, then 4 turns" gave 35° neck up one way and 39° the other.
    - With rest called 1½ to 2 turns early it gave 44-47°, where b6's boss is 0-4 mm off the
      bar or jams on it.
    - The arm and the leg are both visible, and the pose is one crank angle.
    - From 44° to 20° the box doesn't grow: the board sets its length.
  - **One jack height:** forgotten, the raised jack is a glass-crushing trap. Near −30°
    each newton at the grip puts 20-25 N on the rim, and the extra effort reads as "a stiff
    bit" next to the normal 21.9 N. It gained a RIEDEL Ultra 12 mm, and −20° can't be seen
    on the cradle.
  - **The cleats:** each side stood on two small printed brackets, with nothing across the
    bay.
    - The reviewer's estimate is a few N/mm of sideways stiffness at the bottle.
    - Through-cut strips of the board's own sheet brace each rail and locate it on
      reassembly with no pocket routing, which SendCutSend may not do.
    - Gluing them to the as-built cradle makes them agree with it.
  - **The oversize holes, the passive side last:** B's two remote centres must agree to
    within the joints' play. A misfit of Δ forces 1.29 Δ of motion into the joints over a
    swing.
  - **The knob:** the user wants a short, cheap order.
    - McMaster's 18-8 thumb nuts (90368A300, 96445A116) are 15 mm tall, would need a 30 mm
      bolt, and show no price to a script. Aspen's 18-8 thin ones come 50 to a box (USD
      167).
    - A printed knob on an 18-8 nut already on the order costs 0.11 + filament. The nut
      bears on the washer, and the knob only turns it.
  - **The offset:** the gearbox (2.1 kg at z −258) puts the centre of mass 68 mm toward the
    drive side. 40 mm keeps the passive brackets' knobs (244 mm out) 8 mm inside the
    board.
  - **The 7075's estimate:** no 7075 was ever quoted.
    - Its floor is the 0.187 in 5052 fit (the dearer alloy can't be cheaper).
    - The x2 is the repo's walker quotes' 6061-to-5052 ratio per area, not a bound.
    - Wood was never quoted, so the birch gets no number.
- **Not done:**
  - **The front trim** (35 mm, the front at 39.5°): with the board offset, the passive
    front foot's pad would sit on that side's carriage bolt's head.
  - **The rails' 18 mm over the board:** they are the base stop's (the saddles' round). The
    board says so and the use limits warn.
  - **A tilt pointer and scale:** it needs notches in the plates or a cradle link (the
    structure round's).
  - **Witness notches:** paint marks instead.
  - **A crank guard:** the user's call of 2026-10-10 stands.
  - **A's pack:** still for shipping once. DECANTER.md and `cradle_pack`'s docstring now
    say so.
- **Where:** `spiderpig/construction/cradle.py`, `spiderpig/cradle_pack.py`,
  `spiderpig/cradle_report.py`, `spiderpig/hardware/bom.py`, `spiderpig/hardware/order.py`,
  `spiderpig/hardware/cradle_catalog.py`, `viewer/src/decant/{pour,pack,main}.ts`,
  `tests/test_cradle.py`.

### 2026-10-14: the cradles' structure, rated from the force vectors (design review round 1: structure and strength)

- **What:** the review's linkage and laser-cut-parts reports
  (`/root/scratch/decant/review5/r1/`) found the strength check blind where it mattered
  most, and the bottles' centres of mass too far forward. Fixed, in this order:
  - **The bottles' centres of mass** (`cradle.SIZES`): from a glass-shell-and-wine model
    scaled to each empty mass, 31-36 % of the length up from the base. The 6 L
    methuselah's is 390 mm from the mouth, not 331 (42 % from the base had been assumed).
  - **The checks** (`spiderpig/cradle_structure.py`, `cradle_report.structure`, fed to
    `strength.check` through the loads document): every link a plane frame of its chords
    under the statics' force vectors (`cradle_report.statics_vectors`), rated at each
    chord's end on the net section of its hole, and in Euler for a chord in compression;
    the posts (pillars and columns) solved as one box with the inner plate rigid in its
    plane and each end held by its plate's strips; the frame plates' members in-plane, in
    Euler out of the plane, and where a node isn't held across (the gearbox's O), its
    tilt. `link_rows`' `F L / 4` and the pillars' "between plates" stay for walkers.
  - **B:** b2 a triangle (the chord P-K); b6 0.187 in with the chord X-Tf1, its rear arm
    X-Tr1 40 mm wide and X-Y 32 mm (`config.PRODUCT_CHORD_R`); the inner plate a triangle
    O1-O2-D1 and a leg D1-D2 on two columns; the outer plate's O2-O 64 mm wide; clevis
    pins at K and J2 (P and J hardened pivot pins: no Chicago screw); the base board 680 mm
    long (with the stand round's 40 mm offset toward the drive side, `cradle.BASE_Z`: the
    least that keeps the rear edge past 25°, at 25.2°).
  - **B's pack** (`cradle_pack.PACK_PLANS`): the panel's new inner triangle, leg and
    columns stand where the drive side's gearbox had dropped past it, so the drive side
    landed on the panel and tipped. The panel now lies turned the other way, at the board's
    +x end, the drive side on its drive side over it: 696 x 499 x 608 mm, 211 L (202 L
    before), every piece supported.
  - **A:** b2's E-Tr1 32 mm wide; a fifth column, D3, down the front leg; G-O 48 mm wide.
  - **Both:** the columns 12 mm (NETRF12 on M6, `Stand.column_d`); the strength check's
    holes the cut ones (`strength.cut_hole`: 8.2 mm, not 8.35).
- **Why:**
  - **Bending from `|r x F|`:** a link of three pins isn't a beam between its farthest two.
    B's b2 is a bell crank on O2: the whole jammed cradle moment, 230 N·m, passes its
    lever's root. Rated so, it held 0.80 at its pillar's hole, where the old rule read
    2.61. b6's rear arm, a cantilever off X, held 0.32 in 0.125 in; it had only been rated
    in net tension.
  - **The box, not "between plates":** the inner plate is held in its plane by nothing but
    the posts' bending. B's inner plate was the bar O1-O2 on its two pillars: it turns
    about one and holds the other only along itself. Across it O2 was a cantilever off
    the outer plate, about 0.4 jammed (the old rule: 1.33). A triangle and a leg on
    columns make a box; then every post shares the net force the jam puts on the pivots
    (about the rod's), and every post is rated, the columns too (never rated before). The
    plates' strips hold a post's end far less than a clamp would, so the posts are solved
    with each end's spring (`cradle_structure.plate_spring`) rather than assumed clamped or
    pinned. A post's tapped ends are rated on the tube the tap leaves, the rest on the bar.
  - **The frame members:** B's O2-O carries the rod's push back to the gearbox, 1.45 kN
    jammed; 24 mm wide it buckled out of the plate at 0.72. A's front leg carries 2 kN from
    D2 to its foot and buckled at 0.59. The gearbox's node O is held across the plate only
    by its members, which the tilt check counts (B's rear leg had read 1.25).
  - **The blocked cradle** (`cradle_report.blocked`) is a known limit, not designed for:
    at 66 N·m against a block at the front tie, 72 mm from the mouth, B's J takes 6.6 kN
    and O1 6.2 kN, its weakest part (the outer plate) 0.17; at the rear saddle, 0.73. A:
    0.21 and 0.84. The use limits keep the decanter out of the cradle box's path; a torque
    limiter at the hand is the cure, for the drive's next round.
- **Numbers** (2026-10-14, `cradle_report.report`, a full 6 L walking, a 40 N push jammed):
  - B: every row at jam SF 1.31 or more (pillars O1 and O2 1.31, b6 1.33, pin K 1.43,
    b4 1.44, the outer plate's members 1.5); walking 3.85 or more. Its stack 84.23 mm a
    side (79.68: the clevis pin at K takes a layer under b2, and b6 is thicker).
  - A: every row at jam SF 1.61 or more (b2 1.61, pillar G 1.63, the crank's keyway 1.9,
    the outer plate 2.01, column C2 2.17); walking 3.66 or more. Its stack unchanged, 65.53.
  - Before (the old rules, the old centre of mass): B's least jam SF 1.33 (pillar O2), A's
    1.7 (b2).
- **Open, for the other streams** (the corrected centre of mass, not the structure):
  - **The drive:** the crank's peak rose to 34.3 N·m on B (the NRV040 at SF 1.31) and
    36.0 on A (1.25: under its 1.3 bar), and the worst turn to 25.4 N at the grip on both
    (the bar was 25 N). The jam is fixed at 40 N at the grip, so a handle or a ratio that
    eases the turn raises the jam with it: the 25 N bar, the gearbox, or a counterweight
    is the drive's call. The tests carry the bars at 26 N and (A) 1.2 meanwhile, saying so.
  - **The stand:** A tips at 24.4° over its rear edge (25.3° before; its test's bar is
    15°, B's 25°); 10 mm more of A's board would bring it past 25° (25.3°). B's board is
    680 mm: every edge past 25° (rear 25.2°, the drive side 29.7°, the passive side 28.9°,
    the front 40.1°).
  - **The saddles:** the methuselah's centre now lies 10 mm behind the rear V's end (the
    mounting review's B1); the text that says the rear V reaches behind it is theirs to fix.
- **Not done:**
  - **Fillets on the links and the crank plates** (the frames' grow-then-shrink closing):
    a link's planner claims are pills on its joints, and a fillet's material lies outside
    them; claiming it needs points fixed to a link (the topology has them only for the
    crank: `Topology.add_crank_point`). B's b6 window is split by the chord X-Tf1 instead;
    its corners stay sharp. The crank plates are the drive's (`handcrank.py`).
  - **The passive side planned on its own** (about 35 mm thick instead of 84): written up
    in DECANTER.md, B's stand; left for later.
  - **J as a clevis pin:** its clip and end would stand in b6's layer, which its rear arm
    sweeps right over J; the planner finds no plan. J stays a pivot pin.
- **Tried and dropped:**
  - **A column between O1 and D1** on B (a third, to share more): it stands in the lever
    b2's sweep, and nothing plans.
  - **Wider members round O2** to stiffen the post ends: the moment moves to the ends
    rather than falling.
- **Where:** `spiderpig/cradle_structure.py`, `spiderpig/cradle_report.py`,
  `spiderpig/cradle_pack.py`, `spiderpig/strength.py`, `spiderpig/construction/cradle.py`,
  `spiderpig/construction/plates.py`, `spiderpig/config.py`, `spiderpig/materials.py`,
  `spiderpig/linkages/decanters.py`, `tests/test_cradle.py`.
### 2026-10-14: the cradles' drive (design review round 1)

- **What:** the fixes to round 1's drive review (`/root/scratch/decant/review5/r1/drive.md`),
  both cradles ([DECANTER.md](DECANTER.md): "The drive", "Loads and checks", "Flat pack",
  "Building one"), on the structure and mounting rounds' masses.
  - **The riser is four round spacers** (B1): 12 mm aluminium spacers for M6 (McMaster
    94669A349, 13 mm OD) on the face screws, bearing on the gearbox's machined output face
    (`handcrank.SPACER`, `riser_for`). The three 0.125 in discs were bored to the output
    hub's drawn 30 mm on the spigot ring's assumed 2.5 mm: two unpublished housing
    dimensions met at zero margin, and a hub standing proud would have been braked by the
    screws' preload and lifted the block into the rod. Now the spigot ring and the hub's
    end stand 9.5 mm under the plate whatever their heights; only the machined face is in
    the stack-up. The screws are the same M6 x 25 (8.25 mm into the face).
  - **The crank block sits on the shaft's steel end** (M2): its top plate on 0.2 mm DIN 988
    shim rings (Accu, eight bought, `handcrank.SHIM`), the end screw through both into the
    shaft's tap with 243 at 8 N·m. The printed sleeve is gone: at 3 N·m its 0.05 mm of
    creep would have taken the preload, and a re-torque only reset it.
  - **The go/no-go** (the guide's riser step): with the gearbox on its spacers under the
    plate, the shaft's end over the plate's inner face is 28.75 mm drawn and the block's top
    plate sits at 29.22, so the shims are the difference in whole rings (two drawn). Over
    29.22: 13 mm spacers; under 27.62 (the keyed plates' straight flanks): 11 mm. The
    shims hold a hub up to 0.5 mm prouder or 1.1 mm lower than drawn; the other lengths
    another millimetre either way.
  - **A slip clutch in the hand crank** (M1, `handcrank.clutch`, `CLUTCH`): the crank is two
    printed parts, the hub and its tube keyed on the input shaft and the arm on the tube's
    journal, coupled by three 6 mm steel balls (McMaster 9291K77) on springs (McMaster
    9434K49) and M8 cup-point set screws (Bolt Depot 8743), pressed into six 90° conical
    seats in the tube's flange. Set on the bench with a luggage scale to slip at 3.375 N·m
    on the input (`CLUTCH_SLIP_NM`: 27 N at B's grip, 39.7 at A's), ±10 % once set, so at
    most 61.3 N·m on either crank (`config.PRODUCT_CLUTCH_NM`) through the worm at a
    hand's speed, however hard anyone pushes. The arm is held on the journal by a printed
    cap and a second M3 thumb screw: the crank comes off the shaft whole.
  - **The jam case is the clutch's cap** (`cradle_report.jam_nm`, `config.PRODUCT_JAM_NM`,
    61.3 N·m on both): the 40 N push it had been (`JAM_HAND_N`, 66 and 69.3 N·m) is no
    bound, the cap is. The push's torque stays a row (`jam_push_nm`).
  - **A clutch-capped case over the whole turn** (`cradle_drive.clutch_case`, the report's
    `jam_capped`): the cap on a blocked cradle at every crank angle, the toggles included,
    by the structure round's model; per excluded band (0 to 30°) each part's least safety
    factor, and the band within which each falls under 1.3 and under 1. The report also
    rates the gearbox at the jams, the input shaft's radial load and the keyway one plate
    and four (`cradle_drive.drive_limits`), and its `verdict` carries its own numbers.
  - **Cheapened** (round 1's "simplify"): the block's tie, a NETRF10 and two M5s, is an M6 x
    30 button head through every plate into a DIN 439 thin nut under the lowest, 243
    (`handcrank.TIE_NUT`): a nyloc (6 mm with its washer and the bolt's end past it) doesn't
    fit the 7 mm head layer, and over the block it would raise the rod. The grip pin, a
    NETRF10 and two M5s, is an M8 x 85 partially threaded hex bolt (Bolt Depot 5826), its
    plain shank under the grip, a nyloc on the arm's inner face; a shoulder screw cost
    USD 5.66 (McMaster 92981A753). The NRV040 stays at 30:1.
  - **The minors:** the keyed plates' slots cut at the key's 6.0 mm and filed together on
    the key, the block built and tied on the bench first; the grip 60 x 16 mm (was a 25 mm
    knob); the hub 21 mm on the shaft, printed solid, its bore 11.1 reamed to 11; an M3
    heat-set insert (Accu HTBI-M3-BR) under the thumb screw and the input key glued in
    (the hand crank comes off with nothing loose); the guide's run-in and
    wipe, the crankpin greased, the worm's 10-minute creep test and the O-ring's drag,
    where to stand; the 4 x 4 x 16 key (it said 3 x 3); `VERDICT`'s numbers gone (the
    tests' bars instead, the report's own numbers in its `verdict`); DECANTER.md's
    crankpin lever 9.5 mm.
- **Why the slip clutch is back** (2026-10-13 dropped it: "it still lets 1.6 x the peak
  through, so the parts need upgrading anyway"): the point is a true bound, not a lower
  number. The 40 N push the parts were rated at is no bound (a hand on a crank that feels
  stuck pushes 60-150 N); the clutch's cap is, and the parts already hold it.
  - **Sized:** at its least (−10 %) it slips at 1.11 x A's worst starting turn at the input
    (2.735 N·m, the worm's static efficiency 0.45, a sealed full 6 L on the mounting
    round's saddles) and 1.17 x B's (2.602): the bar is 1.1, so a full 6 L started from a
    stop never ratchets. A lower slip would.
  - **Where it doesn't help:** round a toggle the rod's force goes as the cap over the sine
    of the crank's angle to the rod: no torque bounds it. B holds 1.3 everywhere 30° and
    more from the toggles (pillar O2 1.36; 1.15 at 25°, 0.93 at 20°, 0.46 at 10°); its rod
    buckles, elastically, within 17° of rest and 9° of full tilt. A holds 1.3 outside 25°
    (b2 1.80 at 30°, 0.85 at 15°; the rod buckles within 12° and 6°). Only clearance
    protects there (the use limits), or a force limiter in the rod (not done). Blocked at
    the front tie, the cap leaves B's outer plate at 0.18 and its pillars at 0.32-0.36, A's
    plate at 0.23: a known limit, kept out by the use limits.
  - **The ball detent over a friction clutch:** its slip depends on the seats' angle and the
    springs, not on a friction coefficient that greasing or a humid day can halve; its set
    screws let it be set to a scale reading; a printed detent's ±25 % becomes ±10 % once set.
- **The bars, each on a cited source** (the coordinator's: no bare relaxing). The structure
  and mounting rounds' heavier load case (35.1 N·m on B, 36.9 on A) had put the gearbox at
  1.28 and 1.22 x its 45 N·m and both grips at 26.0 N, over bars of 1.3 and 25 N that had
  no source.
  - **The gearbox** (`WormBox.rated_hand_nm`, `gearbox_sf_hand`, bar 1.3): Motovario's VSF
    IEC catalogue (EN, rev 0 2017, my.motovario.com, read 2026-10-14) rates the NRV040 at
    30:1 at Mn2 34 N·m at 2800 rpm input, 46 at 1750, 48 at 1400, 51 at 1140, 53 at 900
    and 62 at 500, the slowest it lists (the NRV performance tables, printed pp. 161, 171,
    181, 191, 201, 211): the rating rises as the input slows. A hand turns it at about 60
    rpm, so the 500 rpm figure is the cited one nearest, on the safe side. Phidgets rates
    its TRM4401_0 clone 45 N·m with no speed (phidgets.com/?prodid=869), so the clone is
    taken at 62 x 45/48 = 58.1 N·m: SF 1.65 on B, 1.57 on A. The 45 N·m row stays
    (`gearbox_sf`, 1.28 and 1.22; the test asks 1). The cap, 61.3 N·m, is 0.68 x the
    catalogue's momentary limit, 200 % of Mn2 on the 45 (p. 9).
  - **The grip** (`handcrank.LIGHT_LOAD_N`, 22 N): MIL-STD-1472H (15 September 2020,
    everyspec.com, read 2026-10-14), §5.1.4.1.2.4.7, Figure 13 "Cranks" (p. 76): a crank
    under 22 N is a light load, wrist and finger movement, at a 38-125 mm turning radius
    below 100 rpm, its handle 25-75 mm long and 10-16 mm across; at 22 N and over it is an
    arm's crank at 190-510 mm. A pour is about 15 handle turns, well under 100 rpm: the
    many-turn task the standard puts cranks to (§5.1.4.1.2.4.1.1); the figure has no turn
    count beyond that, so the bar is its force, read at a full 6 L's worst start (the
    static efficiency: a stop at the peak is restarted). B's crank is the figure's longest,
    125 mm (`HANDLE_R`): 20.8 N. The grip went to 16 mm across (was 24), the figure's most.
  - **A misses the grip bar: open.** At 125 mm A would need 21.9 N, but its hand crank
    meets its own cradle: b2 and the rear tie Tr2's screw pass 100-135 mm behind the input
    shaft's axis at about 165-195° and 330-345° of crank, so the crank's disc clears only
    up to about 97 mm. A's old 105 mm crank met them there too (with round 1's grip on its
    85 mm bolt); the slow clash test's five angles had missed it, and now also checks 165°,
    180° and 345°. A's crank is 85 mm (`HANDLE_RS`, clear at 5° steps there and 15°
    elsewhere): 32.2 N at the worst start (26.3 running), the test's last check failing.
    What would close it:
    1. The counterbalance below: A's worst start 14.9 N at 85 mm.
    2. A 130 mm longer tube: it clears a 125 mm crank (21.9 N, 0.1 under the bar). The
       hub-and-tube piece becomes about 250 mm long, over the 210 mm it prints at upright
       (hub down, seats up): two printed pieces or a bought tube; the grip about 300 mm
       behind the input, where the input shaft still takes it (the catalogue's radial load
       there 98 N against 29.7 at the clutch's top).
    3. Not a ratio: at 85 mm A would need 27.1 N at 40:1 (static efficiency 0.40), 24.1 at
       50:1 (0.36), 22.6 at 60:1 (0.32); Phidgets sells only 30:1.
    4. Not another input direction: turned round O every 15-30°, a 125 mm crank meets the
       outer plate, a column or the cradle at every angle tried (an estimate on the parts
       as built).
  - **The input shaft's radial load** (`input_overhung`, a new row): the grip's middle 155 mm
    past the input's shoulder, where the catalogue allows Fr1 a / (b + x) = 149 N on the
    NRV040 (2.11.2, p. 43: Fr1 350 N, a 106, b 95); the clutch's top is 29.7 N at B's grip
    (SF 5.0) and 43.7 at A's (3.4).
- **B's board is 682 mm** (`Stand.base`, 2 mm more at the rear): with the drive's parts the
  680 mm board's rear edge tipped at 24.9° (25.0° on the mounting round, the bar 25°);
  682 mm holds 25.1°. Nothing else moves; the pack's bound follows the board.
- **Numbers** (B, A; the mounting round's masses):

  | | before (the structure round, the push at 100 and 105 mm) | after |
  |---|---|---|
  | gearbox SF walking | 1.31, 1.25 on the 45 N·m (1.28, 1.22 with the mounting round's saddles) | 1.65, 1.57 at a hand's speed (58.1 N·m); 1.28, 1.22 on the 45 |
  | grip at the worst start | 25.4, 25.4 N (26.0, 26.0) | B 20.8 N at 125 mm; A 32.2 N at 85 mm (its 105 mm crank met its cradle) |
  | crank torque bound | none (66, 69.3 N·m at a 40 N push) | 61.3 N·m, the clutch's cap, the jam case |
  | the clutch's least slip over the worst start | | 1.17, 1.11 (bar 1.1) |
  | gearbox at the jam | unreported (1.47, 1.54 x its 45) | 1.36 x its 45; 0.68 x its momentary limit |
  | jam SF, 30° and more from the toggles | B pillars 1.31; A b2 1.61 (at the push) | B pillar O2 1.36; A b2 1.80 |
  | in the toggle zones | not rated | B at 20°: O2 0.93, rod 1.14; 10°: 0.46, 0.59; 5°: 0.22, 0.29 |
  | blocked at the front tie | B plate 0.17, pillars 0.30-0.34; A 0.21 | 0.18, 0.32-0.36; A 0.23 |
  | keyway | 2.0 at 66 N·m (as if the four all bore) | 2.15 x the cap filed together; one plate 0.54 (0.94 walking) |
  | input shaft's radial load | unrated | SF 5.0, 3.4 at the clutch's top |
  | the riser | 3 discs on an assumed 2.5 mm spigot | 4 spacers on the machined face, 9.5 mm clear |
  | the block's clamp | a PLA sleeve at 3 N·m | steel and aluminium, 8 N·m |
  | B's stand | 25.0° on 680 mm (the mounting round) | 25.1° on 682 mm; A 24.0° (bar 15°) |

- **No stow clip** (the review's packing note, not done): the crank's only place on the drive
  side where nothing moves is its outer plate's outside, and B's pack plan lays that face
  on the passive panel. Built and packed (two printed clips snapped round the strut O-O2),
  the stowed crank, 54 mm proud, lifted the drive side by as much: 478 mm to 532 on the plan
  before the structure round, over the 500 mm bound. The crank stays a piece of its own,
  with nothing loose on it, packed in the panels' openings.
- **Cost** (the BOM's estimate): B USD 723.26 (purchases 421.68, cutting 301.58), A 600.52
  (340.58, 259.94), with the structure and mounting rounds' parts. The drive's changes took
  USD 25.52 off B on the stand round's base. Its new lines: the spacers 6.48, the shims 6.40,
  the clutch 16.54 (balls 7.89 for a pack of 50, three used: MISUMI sells 52100 balls singly
  at 2.08 by its price API; springs 4.58 for five; set screws 0.63; an insert and a thumb
  screw), the grip's bolt, nut and washers 1.94, the tie's bolt and nut 0.45. Gone: three
  riser discs (about 31 of cutting), two NETRF10s (26.92) and their four M5s and two
  washers.
- **The landed cost of the gearbox** (pages of 2026-10-14): TRM4401_0 USD 62.90 and the shaft
  TRM4006_0 6.15, both made in China (HTS 8483.40.50.10 and 8483.10.50.00). Since 2025-05-02
  Phidgets collects US duties, tariffs and brokerage at checkout through UPS Global Checkout,
  with a fee of its own. The duty is 40 % on the gearbox (2.5 % + 25 % Section 301 + 12.5 %
  since 2026-07) and 50 % on the shaft: USD 28.24; no de minimis. Goods and duty USD 97.29;
  UPS shipping, the brokerage and the checkout fee show only at checkout (the cart wasn't
  run): about USD 135-165 landed on assumed shipping (25-45) and fees (10-20). Not in the
  BOM's total.
- **Proposed, not built: a counterbalance** (asked for in round 1). A gas spring on a
  second crank on the output shaft's far end, outboard of the gearbox: from far off, a
  near-constant force on a pin b from O puts a sine torque on the crank, which is the
  crank-rocker's gravity torque's first harmonic (it changes sign only at the toggles).
  Fitted to every bottle, sealed full, and the empty cradle at once (the mounting round's
  masses; `/root/.cache/spiderpig/tmp/r1drive/cbal2.py`, `cbal3.py`):

  | | B | A |
  |---|---|---|
  | worst crank torque, walking | 35.1 -> 15.8 N·m | 36.9 -> 17.1 N·m |
  | gearbox SF at a hand's speed | 1.65 -> 3.68 | 1.57 -> 3.40 |
  | grip at the worst start | 20.8 -> 9.4 N (125 mm) | 32.2 -> 14.9 N (85 mm): under the bar |
  | the sine it takes | 20.25 N·m at 324° | 21.0 N·m at 247° |

  - **The ends of the range:** a spring sized for the 6 L alone (32.5 N·m) all but cancels
    it (35.1 -> 7.3 N·m on B, 36.9 -> 9.1 on A) and overbalances the light end: a 750 ml
    goes from 7.6 to 26.4 N·m the other way on B (7.7 to 26.7 on A), the empty cradle from
    6.0 to 27.8 (5.9 to 28.1): as hard to turn as an imperial is now, 15.7 N at B's grip
    and 23.3 at A's 85 mm one (over the bar). The fit over every bottle (the table) shares
    it out: the 750 ml at 14.3 N·m (A 15.9), the empty cradle 15.7 (17.1), the 6 L 15.8
    (17.0). Either way the light end turns harder than now and the empty cradle is driven
    toward full tilt by the spring, held only by the worm (static efficiency 0.45: the
    creep test is a full bottle's, not this). A setting per bottle (the lever's pin in one
    of several holes) would be moved against 400 N.
  - **Breakdown with 400 N stored:** a gas spring is never slack in the mechanism (396 N at
    full extension, 512 compressed: McMaster 4138T131); it pushes the crank at every angle,
    held by the worm alone, about 20 N·m at the crank. So it is unloaded before the drive
    comes apart: its anchor on a jack screw (an M8 through the bracket, about 25 mm of
    travel), backed off at the crank angle where the spring is longest until its rod rests
    on its own stop, then its ball socket unclipped; at set-up the reverse, the jack screw
    drawn up against 400-500 N (about 1 N·m on an M8, some 20 turns). B packs as modules,
    the crank on the drive module: loaded, the box would ship 20 N·m held by a worm whose
    clone may creep, and a crank block taken off its shaft would whip round. That is two
    more steps to each pack and set-up, an M8 jack screw and a nut on the bill.
  - **Parts** (pages of 2026-10-14): Phidgets' double output shaft TRM4007_0, USD 6.85 (19
    mm, 39 mm past each snap ring: replaces the single 6.15 one, the same 50 % duty);
    McMaster's gas spring 4138T131, USD 28.36 (396 N extended, 512 N compressed, 100 mm
    stroke, 276 mm long, ball sockets and studs included); a keyed 6061 lever 50 mm to its
    stud (two laser-cut plates) and a bracket from the outer plate out past the gearbox to
    the anchor (laser-cut, unpriced: SendCutSend quotes at upload). About USD 36 bought and
    two cut parts. (A long extension spring can't do it: McMaster's 400-500 N springs are
    2.8-4 N/mm, so their force swings with the stroke; 9654K709 holds 507 N at USD 10.14.)
  - **Against:** a second exposed crank and 400 N of stored spring by the hand crank, a
    stiff bracket cantilevered 100 mm out from a truss plate (the structure round found
    the drive side soft across), the light end turning harder, the release at every pack,
    and the gas spring's fade over years. Not simple, so a proposal: it would halve the
    walking peak and the gearbox's load and bring A's grip under its bar at 85 mm. The jam
    would then be the cap plus the spring's residual, not the cap alone.
- **Not done:** a force limiter in the rod (the only cure for the toggle zones); a guard
  (the user's call of 2026-10-10); a 40:1 box (Phidgets has none; it wouldn't bring A
  under its bar either); the lowest crank plate in the file of six (round 1's "small",
  USD 2).
- **Where:** `spiderpig/construction/handcrank.py`, `spiderpig/cradle_drive.py`,
  `spiderpig/cradle_report.py`, `spiderpig/config.py`, `spiderpig/strength.py`,
  `spiderpig/construction/cradle.py`, `spiderpig/cradle_pack.py`,
  `spiderpig/hardware/cradle_catalog.py`, `tests/test_cradle.py`.

### 2026-10-14: the planner plans the lowest stack, proves it soundly, and searches in parallel jobs (the brief: the planner encodes the true optimum, not proxies and arbitrary tie-breaks; the search in parallel, both head placements, the walkers' improvements committed)

- **What was unsound.** The search's z was a pitch a layer with no gaps, and rules read it
  as if it were the plan's. They refused layerings every real plan of them builds:
  - **a cradle's pins made to length** (`PivotPinShaft`): two 0.190 in links in
    neighbouring layers looked 6.35 mm thick, not 9.65. A 10 mm pin's end then looked
    3.4 mm too tall for any slot. B was "proven" thinnest a layer or two too thick;
  - **standoff pillars** (`StandoffAxle.column`): a short column that no stock length fills
    at a pitch a layer can be filled once the plan's gaps are in;
  - **the crank's stock standoffs and stub** (`BoltCrank._web_span_ok`, `stub_layers_web`):
    their tables were made at a few nominal z's;
  - **the hand crank** (`HandCrank.claims`): the layers its block crosses and the rod's least
    height came from a z estimate.
- **The fix: the search's z is a bound** of every plan its layering can become
  (`stack.bound_layout`; `Layout.t_hi`, `span_hi`, `z_hi`):
  - each layer is at least its frame, fixed or placed links' sheet, with no gaps;
  - each layer is at most the thickest sheet it may yet hold, and each gap at most
    `GAP_MAX`;
  - a rule that refuses what is too long reads the lower bound; one that refuses what is too
    short reads the upper; a window refuses only where no z between them fits; everything
    else waits for the plan's z.

  `tests/test_seam_stack.py` checks this against `tests/brute.py` (`brute.lowest`) on a
  cradle pin. The old search gave 5 layers, "proven"; the brute force and the new search
  give 4.
- **The plan's z** (`finalize`): it starts where it always did. Where that settles into a z
  some claim can't be built at, it starts again from the least z (a cradle's thick links).
  Then each gap is lowered to the thinnest the plan still builds at (`_descend`), since
  settling only ever grew them. A standoff longer than its column holds the plates apart
  (`Claim.hold`, `StandoffAxle.hold`: up to `max_long`, 0.8 mm, was always accepted, the
  links loose by as much), so a gap is lowered only where the stack really is lower, and a
  hold left is taken into a gap (`_take_up`): the plan's height is what the parts stand
  at. A hold no gap can take counts in the objective (`StackPlan.hold`).
- **The objective** (`StackProblem.key`), in order:
  1. the height, frame plate to frame plate: what the module occupies between its plates,
     what packs and what the pillars and pins span (what stands outside the plates is the
     same for every plan of a design);
  2. the crank route's cost;
  3. the stock parts' cost (each claim's `tally`, at unit prices);
  4. the parts printed;
  5. a fixed tie-break.

  Products plan on it, proven. **Walkers plan on "layers"** (`config.plan_objective`): the
  fewest layers, then the cheapest route, a size's search stopping at its first plan with no
  route cost, as before. On the height the Strider double planned 15 layers at 67.96 mm
  against 14 at 68.76 (0.8 mm lower, a layer more), unproven after 166 s wall on 16
  workers, 1,678 CPU s (measured at `91479cc`, before the router's tables were made sound):
  the walkers' crank router prunes on its route's cost, which bounds nothing on the
  height, so every size the proof needs runs out of budget. `config.LINKAGE_OBJECTIVES`
  puts a linkage on the height when that is wanted.
- **Both head placements, side by side.** The 2026-10-04 rule (heads in gaps only where
  sunk finds no plan) is gone; each placement is a job of its own, run in parallel. The rule
  cost the cradles nothing: neither A nor B has a plan with its heads sunk. Among the
  walkers, the dwell rocker and the `klann_lego` quad now plan sunk (below).
- **Parallel jobs** (`stack/drive.py`, `stack_pool.py`). A job is a stack size, a head
  placement, a node budget and perhaps a plan to beat. Its answer is its own, so the plan is
  the same with any number of workers (`SPIDERPIG_PLAN_WORKERS`, up to eight; none with
  `SPIDERPIG_WORKERS=0`; one under pytest-xdist). A short job stops at its first plan.
  The plain search and a leg at a time run side by side, and a short job whose answer isn't
  read is stopped; once sizes get dear (`stack.drive.DEAR`), idle workers run the size the
  search likely asks next; a proof job is split by its first link's layer, four sizes at a
  time.
  Each placement has its own `max_total_nodes`, as each had when the second ran only after
  the first: an unproven design (the Strider quad) spends twice the CPU. A job's CPU
  deadline is `max_seconds`, a placement's jobs one and a half of them together
  (`stack.drive.SPENT_SECONDS`: TrotBot's toe quad returns in 80 s under a 15 s deadline,
  155 s when each job had its own). The leg hint is the single module's proven plan, as it
  was (its first plan alone, a layer thicker on the XL330, left the Strider double there
  with no plan in 61 layers). Planning time as
  `design_side` (the leg hint included), wall seconds (min-max), interleaved runs, on
  20 cores shared with other agents. At load 2-6 (three runs, `58ebfca`, before the cheap
  sizes ran here):

  | design | before (serial) | serial | 8 workers |
  |---|---|---|---|
  | Strider double | 3.52-4.39 | 3.62-4.70 | 3.62-4.38 (CPU 10-12 s) |
  | B (`decanter_rcm`) | 7.23-8.62 | 4.07-4.80 | 2.96-3.45 |
  | A (`decanter_hinge`) | 5.12-5.86 | 0.29-0.33 | 0.37-0.43 |
  | Strider quad | 27.3-35.2 | 71.0-83.4 | 21.2-22.0 (CPU 123-125 s) |

  Under load the Strider double's workers could lose (at load 14-44, 5.3-21 s against
  4.6-11.9 before): they spent 2.5 times its CPU, its sizes cheap but each worker's start
  and caches its own. Since `d8c1090` a size runs in the workers only once sizes get dear
  (`stack.drive.DEAR`), and here, one job at a time, before. At load 18-21 (`d8c1090`):
  the Strider double 7.45 s before, 7.91 serial, 7.61 with workers (7.4, 7.9 and 7.7 CPU
  s); B 16.0, 8.8, 6.3; A 6.7, 0.38, 0.68. At load 54-73: the Strider quad 115.9 s
  before, 95.8 with workers (53 against 182 CPU s).

- **The sleeve rule** (`construction.pivots.common.sleeve_runs`): one printed sleeve per run
  of rings and gap rings between an axle's retained parts. It is stepped where it prints and
  split only where a ring would be cut under what it keeps. The study's "+12 rings" on B's
  14-layer plan was the old one-ring-per-slot bookkeeping. Its "+2 real" was a gap ring
  under X's and Y's clevis heads, where the clevis pin's take-up spacer already is: that
  gap is now the spacer's (`ClevisPinAxle.sleeve_skip`).
- **The cradles**, before and after, on design review round 1's design (`decanting-cradle`
  `a52a821` against it merged here; `cradle_report`, each pin's span and its jam SF, walking
  in brackets):

  | | layers | height mm | pin spans mm | jam (walk) SF: pins; pillars | bought | printed |
  |---|--:|--:|---|---|---|--:|
  | B before | 17 | 84.23 | J, J2, K 8.00; P 11.94; X, Y 7.79 | J 11.36 (100.5), J2 2.30 (20.9), K 1.54 (7.17), P 2.87 (12.9), X 2.73 (9.52), Y 2.36 (21.2); O1 1.40 (13.27), O2 1.40 (5.70) | 46 lines, USD 421.68 | 73 |
  | B after | 16 | 84.05 | J, J2, K 7.83; P, X, Y the same | J 11.57 (102.4), J2 2.35 (21.3), K 1.57 (7.30), P, X, Y the same; O1 1.40 (13.35), O2 1.41 (5.71) | the same lines, USD 421.68 | 61 |
  | A before | 13 | 65.53 | E 8.73 | E 3.26 (15.5); G 1.82 (12.41) | 43 lines, USD 340.58 | 43 |
  | A after | 12 | 65.35 | E 8.55 | E 3.31 (15.8); G 1.83 (12.48) | the same lines, USD 340.58 | 43 |

  B's pillars, made to length in steps, stand 0.05 mm proud of their columns, which no gap
  takes: it plans 84.10 mm as the parts stand, the height its objective ranked. The lowest
  plan also has every pin's shortest span here; every other SF (the links, the crank, the
  columns, the frame plates, the rod, the crankpin) is the same or a little higher.
  Strength stays downstream (`strength.check`), not part of the objective. B's pack
  (`spiderpig pack-inspect decanter_rcm`) keeps its box: 699 x 496 x 610 mm outside, 211.6
  litres, automatic (its plan's packing 699 x 499 x 608 to 699 x 498 x 608).
- **The walkers** (identity gate against `next-320384d`; the new baseline
  `planner-f667d6b` and every diff: [PLANNER-gate-diffs.md](PLANNER-gate-diffs.md)). Every
  design keeps its layer count and route; every stack is lower (0.1 to 6.6 mm) and prints
  fewer parts (the sleeve rule: 173 to 111 on the Strider double). The `klann_lego` quad
  and the dwell rocker plan with their heads sunk and cost less; the Klann quad costs USD
  33.52 more (a pillar made to length: the height comes first). Strength and wobble stay
  outside the objective: the Klann quad's cantilever pillar falls to jam SF 0.84 (its audit
  failed before and fails on as many joints), the `klann_lego` quad's pillars keep 0.2 mm
  their 0.5 mm shims can't take.
- **Where:** `spiderpig/stack/{geometry,plan,plan_z,search,drive}.py`,
  `spiderpig/stack_pool.py`, `spiderpig/construction/axle.py`,
  `spiderpig/construction/handcrank.py`,
  `spiderpig/construction/pivots/{common,chicago,clevis,pivot_pin,standoff}.py`,
  `spiderpig/construction/crank/web.py`, `spiderpig/config.py`,
  `spiderpig/fabricate.py`, `tests/brute.py`, `tests/test_seam_stack.py`,
  `tests/test_seam_pivots.py`.

### 2026-10-14: cradle B's pack, turn 1: the cradle box off the drive side (user: "iterate multiple turns on the packing ... You can make design changes to assist with packing")

- **What:** B comes apart into the base board, the passive side panel, the cradle box, the
  drive side panel and the hand crank ([DECANTER.md](DECANTER.md), "Flat pack"). The
  drive side no longer carries the cradle box: once the passive panel is off, the drive
  side's two R-clips (its pins X and Y, quick-release clevis pins already) come out and the
  box lifts straight up off it, the drive side still lying on its gearbox.
  - **The keepers** (`cradle.KEEPER`, `cradle.keeper_names`): the ring each of the four
    pins' plans puts in the gap between b4 (b5) and the cradle link is printed in TPU 95A
    with a 7.6 mm bore on the 8 mm pin, so it grips the pin (`cradle_report.keepers`: 6.1 N
    at the least interference, 0.2 mm, and TPU 95A's least published modulus, 26 MPa; 55 x
    the pin's weight, the test's bar 10; on with about 52 N). A pin of a side lying down,
    its clip out, has its head under its link with nothing under it: bare, it drops out. The
    keeper holds it, so the box can come off the drive side as it lies, and the passive
    panel's pins lift off with their keepers on: the loose spacer and its two steps go. The
    keepers are the planned gap rings re-made in `assemble_cradle` (the pivots are the
    planner's to edit): under the planner's sleeve rule (`pivots.common.sleeve_runs`) the
    gap between b4 (b5) and the cradle link is one slot and one sleeve on each pin
    (`pin_X_gap13_spacer` on the 16-layer plan; the head's gap is the take-up spacer's,
    `sleeve_skip`), and the keeper is that sleeve with its bore filled in to 7.6 mm: it owns
    the gap, no second part (B prints 61 parts, as before the keepers).
    `bom.part_filament` prints them in TPU (`meta["keepers"]`), and each is fastened to its
    pin (`meta["fastened"]`: an interference fit, not a clash).
  - **The steps** (`cradle.module_steps`, the guide's): eight to take it apart (six before:
    two more, the drive side's clips and the box's lift), ten back together (nine before:
    the box onto the drive side's pins first, pin X, then the box turned about it and b5
    swung until Y drops in, its clips in; the passive panel's spacer handling gone).
    `cradle_pack.slide_hits` sweeps the box off the drive side too (its new `way`): nothing
    meets.
  - **The plan** (`cradle_pack.PACK_PLANS`): the cradle box on the board as it stands, the
    hand crank beside it, the drive panel outer plate down over it, its gearbox beside the
    box (6 foam pads to 54 mm), the passive panel bay down over the drive panel's, turned a
    quarter (8 pads to 51 mm). Two searches (150 s x 4, 300 s x 5) found the same.
- **Numbers** (on the planner's 16-layer plan of the same day; on the 17-layer one 211.9 and
  140.6 L): 699 x 498 x 608 mm, 211.8 L, fill 4.0 % before; 740 x 316 x 600 mm, 140.5 L,
  fill 6.2 % (-34 %), every piece supported, no overlaps, the mass centre 40 % up. The box
  is the cradle box's length (723 mm levelled, Y's pins 122 mm in front of the mouth to the
  rails' end 600 behind it: 41 mm over the board) by the board's width, the board, the box
  standing (178 mm) and the two panels nested over it. Strength, stand and drive: no part
  moved (the keepers have the gap rings' outlines; every report row as before). The BOM:
  47 lines, USD 723.26 before and after (the four rings print in TPU, on hand).
- **Why not turn the drive module over** (the drive side up, its pins' heads up, as the
  passive's are when they lift): on its cradle box the module stands with its centre of
  mass 18 mm inside the tie rods' heads and 274 mm up (it tips at 3.8°; 26 mm at best over
  the pack poses' 20-44°); the passive panel and the box turned over together are a free
  linkage (no worm on that side), and standing on the board neither side can slide off its
  pins (its rail between its cleats, over its bolts). Standing the drive module back on the
  board to slide the box off would be four more steps each way.
- **The other leads, measured** (each a what-if searched 90-120 s, on this turn's pieces):
  - The board narrower (its knobs' 508 mm): 137 L, 2.5 %: the panels stop nesting beside
    each other; the sideways tipping would need outriggers. Dropped.
  - The passive side planned on its own (44 mm thinner): 144 L, no gain: the panels already
    nest over the standing box. Dropped (a second plan, its own plates and posts).
  - The board shorter: no gain while the cradle box (723 mm) is longer than the board.
  - The base stop's rails off the box: 124 L (-12 % more), the box then the board's length;
    and with them off a 570 mm board: 116 L (-18 %). That takes a fifth piece with its four
    socket screws made tool-free under the rear V block, and the board's rear tipping edge
    (25.1°, the bar 25°) on a fold-out foot: proposed for the next turn, not made.
- **Where:** `spiderpig/construction/cradle.py`, `spiderpig/cradle_pack.py`,
  `spiderpig/cradle_report.py`, `spiderpig/hardware/bom.py`, `tests/test_cradle.py`,
  `viewer/src/decant/pack.ts` (its comment).

### 2026-10-15: the cradles' stand, use and guide (design review round 2, stand and use)

The round's review (`/root/scratch/decant/review5/r2/stand_use.md`, with the pack's review 1's
M2 and the linkage and drive reviews' blocked cradle and toggle zones) walked the procedure as
a first-time user with the generated guide. What changed:

- **The decanter's height, one rule (B1).** The guide's rule had named the front saddle, 66 mm
  over the real lowest part over the rim (the light's printed clip, 380 mm at rest: `room`'s
  `lowest` now names it); the page kept 3 mm off the room's side and nothing over the rim,
  standing a RIEDEL Ultra's rim 7 mm under the clip in the toggle dwell; four heights were given
  for one decanter. Now `cradle.RISER_CLEAR`: the rim 20 mm under the lowest part, 10 mm off the
  room's side, in Python (`cradle_report.riser_fits`, `riser_for`) and its twin on the page
  (`liquid.ts`), both held to `tests/fixtures/cradle/riser_reference.json`.
- **A printed riser, not a lab jack (B1, M7.2, m3).** Fixed heights stacked in the riser
  locator (the jack locator renamed): two 85 mm blocks and 20, 10 and 5 mm plates, bosses into
  recesses, 122 mm in all, the drawn decanter's; 7-37 and 87-122 mm every 5. It can't be raised
  mid-pour without lifting the decanter off it. The blocks are also the drive panel's lay-flat
  blocks (M2), so no part more. $0 (PLA on hand); the jack was unbought ($17-250 by the review's
  fetches).
- **B's first build lays the cradle down for its passive side (M1)**,
  `construction.assembly.CRADLE_MODULE_ORDER`: the passive panel built whole on the bench (its
  stack, keepers, inner plate, brackets) and lowered onto its cradle link as the trip's way
  back does. Gone: the passive stack pushed sideways onto two hidden pins, its inner plate
  screwed in inside the bay, and "lift the side off" to glue cleats on a side pinned to the box.
- **Rated rest poses (M2)**, `cradle.REST_POSES`, `cradle_pack.rest_poses`: the cradle and the
  drive side lie level on the gearbox and the riser's blocks (`cradle.LAY_BLOCKS`: under O1's
  and D2's screw heads, in the blocks' reliefs): the drive panel alone 35.2° and 14.4 N, from
  6.8° and 2.5 N; the box on its links' long edges, its saddles' tops down, 41.5° and 15.5 N,
  from 10.0° and 2.8 N on two screw heads (its "41 mm" had been to an outline it never sat on).
- **The keepers (M3).** Rated against the real load, up to half the 3.2 kg box on a pin's tip
  (15.5 N), not the pin's 0.11 N: the bore 7.6 -> 7.4 mm (at least 0.4 mm of grip, 12.2 N), a
  30 N bench push per keeper on a kitchen scale (`cradle.KEEPER_PUSH_N`, 1.94 x the load), and
  the way back holds each piece's weight until both its pins are in. No lanyards: a clip's
  lanyard has no anchor that doesn't turn about its pin; the clips go back into their pins and
  the pack of five leaves a spare.
- **The guide's use (M4).** Explicit order (`construction.assembly.USE_TAGS`: the use had sorted
  alphabetically, the trip apart before the setting); each use step and each trip step its own
  step with its own picture (`construction.assembly.Scene`, `spiderpig.guide.scenes`: the
  cradle posed at rest, full tilt, level or the pack pose, laid on its side, its pieces lifted
  off; `Op.lines` items, not paragraphs); the 626-word use limits a five-line card and a
  reference (`cradle.use_steps`, also the page's). The decanter now stays on its riser until
  the bottle is back at rest, and "nothing under the box" reads "once set, nothing raised or
  added under the box".
- **The page (M5).** B first and "the one to build", A "also builds" (`cradle_report.VERDICT`'s
  order, `server.app._cradle_keys`); it opens on a magnum poured into a decanter that holds it
  (`vessels.vesselFor`: no overflow; a 6 L says when to swap); "Using it" shows the use steps
  and the card; no riser until the room arrives.
- **Glue, then finish (M6).** The board stays bare until the cleats and the locator are glued;
  then polyurethane on every face and edge (Titebond's FAQ: its glues bond bare wood). Not oil.
- **Guards for the return to rest (M7).** B's red toggle flag behind the gearbox
  (`cradle.TOGGLE_FLAG`: the crank's arm points into it over the last two turns to rest; on the
  frame side, apart from the drive's own cue); the fixed riser; a window in the board under the
  base stop (`cradle.BOARD_WINDOW`: 26 -> 43 mm). Its birch and the flag took B's rear tipping
  edge to 24.997°: the board grew 2 mm at the rear (684 mm, 25.2°). The optional bay-face skirt
  is not drawn (the front tie rods pass 8 mm from the bar's end).
- **Minors.** The stand's honest margin (`stability`'s `pouring`: the rear 28.5°, the bound
  25.2°); the cleats' 2 mm lead-ins; the jack out of the docs; the docs' numbers; the tape on b3
  only (b5 is nudged); the passive side snugged at level (8½ turns from rest); the decanter swap
  written; stainless tie rods not priced (MISUMI refuses a script, HTTP 403 on 2026-10-15; the
  front saddle, rods and light wiped after each pour instead); the 5 N check by the luggage
  scale (0.5 kg); a tools list on the guide's cover (`cradle.TOOLS`); witness notches not cut
  (each would sit round a jam-rated pin's hole; the paint pen's lines are made in the first
  build and renewed).
- **Where:** `spiderpig/construction/cradle.py`, `spiderpig/construction/assembly.py`,
  `spiderpig/cradle_report.py` (and `payload`, moved from `cradle_pack`: the guide reads the
  pack's pieces and must not reach the strength check), `spiderpig/cradle_pack.py`,
  `spiderpig/guide/` (`scenes.py`, the renderer's posed rows, the cover's tools),
  `spiderpig/server/app.py`, `viewer/src/decant/`, `tests/test_cradle.py`,
  `tests/fixtures/cradle/riser_reference.json`.

### 2026-10-15: the cradles' drive, round 2 (design review round 2; the user's gearbox call of 2026-10-14)

- **What:** the fixes to round 2's drive review (`/root/scratch/decant/review5/r2/drive.md`),
  both cradles ([DECANTER.md](DECANTER.md): "The drive", "Loads and checks", "Building
  one"), with the user's decisions of 2026-10-14: **B gets Phidgets' 50-series worm gearbox**
  (TRM4402_0, 30:1) instead of the 40-series NRV040, the same on A; **no counterbalance**;
  **A is kept and fixed**, not parked.
- **The gearbox, NRV050** (`handcrank.NRV050`, each number cited in its docstring). Phidgets'
  product page (phidgets.com/?prodid=870, the specification tab rendered 2026-10-14): USD
  92.50, "Maximum Strength of Gears 85 N·m", "Shaft Maximum Radial Load 4.8 kN", a 14 mm
  input with a 5 mm keyway, a 25 mm bore with an 8 mm keyway, backlash 1/2°, China, HTS
  8483.40.50.10. Its drawings, fetched the same day: the NRV dimension page its "Mechanical
  Drawings" link serves (TRM4400_0Mechanical.pdf: the 050's input D1 14 j6, B 30, t1 16, an
  M6 end tap; its shoulder G3 64, the far end G2 74, the worm I 50 off the output axis; the
  low-speed shaft 25 h6, B1 53.5, an M10 end tap) and the output shaft TRM4008_0's
  (TRM4004_0Mechanical.pdf: a 32 x 3.5 collar, 50 past it, the keyway 40 long ending 5
  short, 4 deep, an M10 x 16 tap; C45, HB 220-260; USD 6.85). Where Phidgets is silent,
  Motovario's NRV050 (VSF IEC catalogue, EN rev 0 2017; Phidgets sells a clone of the
  pattern, so these are the pattern's): the face 4 x M8 x 10 on an 85 mm PCD, a 70 h8
  spigot, the face's 100 mm boss, 87 mm face to face; 3.3 kg dry; the mesh at 30:1, lead
  6°19', module 2.56, ηd 0.73 at 1400 rpm, ηs 0.44 "only valid after the unit has been run
  in ... at 25°C"; Mn2 113 N·m at 500 rpm (0.30 kW, n2 17: efficiency 0.657), 88 at 1400,
  and the ATEX catalogue's 84 at every speed from 1750 down; Fr1 490 N (a 129, b 114), Fr2
  max 4840 N (a 101, b 76), "static conditions and/or ... limited operations".
  - **Rated against Phidgets' maximum only** (`WormBox.rated_nm`, 85 N·m): walking SF 2.42
    on B, 2.30 on A; the clutch's cap 1.36 and 1.29 x under it
    (`cradle_drive.GEARBOX_CAP_FLOOR`, 1.25: the maximum's basis is unpublished, and the cap
    is already taken at the bench check's top and the hand's fastest).
  - **Round 1's "rating at a hand's speed" is withdrawn** (M-1): it took Motovario's NRV040
    at 30:1, Mn2 62 N·m at 500 rpm in the standard catalogue, scaled by 45/48 to Phidgets'
    45 N·m: 58.1 N·m, and allowed momentary peaks to 2 x the 45. Motovario's own ATEX
    catalogue rates the same NRV040 30:1 at 46 N·m at 500 rpm (45-46 from 1750 down), so
    the standard catalogue's rise to 62 is a wear and heat rating, not the gears'
    strength; and Phidgets calls its 45 a maximum, not a nominal to scale or double.
    `WormBox.rated_hand_nm` and `peak_factor` are gone.
  - **Its efficiency** (M-5; `WormBox.eta_at`, `eta_new`, `handcrank.RUNIN_LOSS`): static
    0.44 run in, 0.387 new (SEW-EURODRIVE's run-in table, "1-start approx. 12%", the
    MOVIONE project planning's "Run-in phase", rendered 2026-10-14; its run-in lasts 48
    hours, which a hand crank never gives a box). At a hand's speed the catalogues say
    nothing: the 500 rpm row's 0.657 is carried down by AGMA's worm-gear friction against
    sliding speed (engineersedge.com's "AGMA WormGear Equations", rendered 2026-10-14),
    0.558 at 100 rpm, MIL-STD-1472H's crank rate (`handcrank.HAND_RPM`).
- **The riser on studs** (`handcrank.riser_for`): the NRV050's face is M8, and no stock
  metal spacer for M8 is 13 mm across or less (McMaster's are 16 mm stainless, which sits on
  the 70 mm spigot's edge on the 85 mm PCD; its aluminium ones stop at M6; the listings
  rendered 2026-10-14). An M8 x 45 set screw in each tapped hole (243), an M8 nut on each set
  to one height by a printed sleeve (13 mm across, carrying nothing once clamped), the plate
  on the nuts and a jam nut over it: the face 25.2 mm under the plate. The nuts' height is
  set: a clone's hub prouder or lower than drawn is turned out (m2: no 13 mm spacers ordered
  ahead), and the sleeves must sit flat on the machined face (m3, in the guide).
- **The crank block** (`handcrank.BLOCK_PLATES`, `KEYED_PLATES`, `KEY_SLOT`): eight plates,
  its top where round 1 had it (the rod rides where it did; the planner's plan unchanged).
  The 050's key is 8 x 7, standing 3 mm proud, its straight flanks ending 9 under the
  shaft's end, so three plates are keyed (one alone 49.4 N·m on the sheet's thinnest, 1.41
  x B's walking peak; the three 2.37 x the cap); their slots cut at 7.75 and filed together
  to the key (m4), the three over them slotted 0.4 mm clear of it, so only the three bearing
  plates are filed (round 2's laser-cut review, minor 2: round 1 had seven slotted at the
  key's width, counted four, and rated them on a ±0.11 mm sheet; SendCutSend's 0.125 in 6061
  table, fetched 2026-10-15, is +0.152 / -0.127 mm, and the keyway is rated on its thinnest,
  `handcrank.PLATE_UNDER`); the
  seventh seats on the shaft's end, an M10 x 16 button head into its tap at 20 N·m (A2-70:
  30 N·m at a friction of 0.1, TR Fastenings, rendered 2026-10-14; m1), the eighth rings
  its head. The shims went with the fixed spacers: the nuts set the shaft's end under the
  seat (0 to 0.7 mm, the go/no-go).
- **The clutch on plungers** (B-1; `handcrank.clutch`, `clutch_slip`, `clutch_drift`):
  three McMaster 5901A17 ball-nose spring plungers (1/2"-13, a 7.14 mm 440C ball, 6-12 lbf,
  USD 4.59; the listing rendered 2026-10-14) through hex nuts in the arm's boss, a jam nut
  on each, on a 0.125 in 6061 seat ring cut with the crank plates: six 3.57 mm holes on a 40
  mm circle, each ball on its hole's edge, a 30° seat. Not the reviewer's 3408A81: a 30°
  seat for its 4.8 mm ball is a 2.4 mm hole, under SendCutSend's least hole in 0.125 in
  6061 (3.175), and its 55 N/mm spring makes an eighth of a turn 20 % of the slip; the 7.14
  mm ball's 30° seat is a 3.57 mm hole, and an eighth of a turn of one of three plungers
  about 3 % (m5). USD 13.77 for the three, against round 1's balls, springs and set screws'
  13.10.
  - **The slip, modelled honestly:** each ball pivots on its hole's edge as the arm turns,
    its contact angle falling from 30° to 0 over the climb, the plunger's spring pushing
    harder; the force on it P = F k / (1 - μ2 k), k = tan(angle + atan μ1), μ1 at the edge,
    μ2 in the plunger's bore, the peak over the climb, plus the cap's thrust washer.
  - **Its drift, rated:** greased (RoyMech's greasy mild steel 0.09-0.19 sliding; no greasy
    steel-on-aluminium figure is published, Engineering ToolBox's greasy static steel 0.16
    is inside it), set anywhere in that range and drifting anywhere in it, the slip moves
    from 0.72 to 1.40 x its setting, 1.47 x with the holes' edges worn 0.1 mm; dry (steel on
    aluminium 0.47-0.61, steel on steel 0.42-0.78) it locks (from μ 0.58). "±10 % once set"
    is retired.
  - **Set at 1.3 x a measured start** (the user's call): the heaviest full bottle, where it
    turns hardest, a luggage scale on the grip; set on the bench at 1.3 x that reading,
    three pulls in a row within 8 % (`CLUTCH_CHECK`), checked at every set-up and after
    greasing (the guide's procedure). The setting doesn't depend on the efficiency: the
    scale reads the box as it is.
  - **The cap** (`config.PRODUCT_CLUTCH_NM`, the jam case for every rating): the check's top
    through the worm at the hand's fastest, 1.3 x 1.08 x the 6 L's peak x 0.558/0.44:
    **62.5 N·m on B, 65.7 on A** (61.3 before), 1.36 and 1.29 x under the 85 N·m.
    Unchecked, the greased drift alone takes B's to 85.1 N·m (the gearbox's maximum) and
    its slip to 0.93 x the start (it ratchets): the check is the cap's condition, and the
    report says so (`unchecked_cap_nm`).
- **The output shaft's radial load** (M-2; `WormBox.output_allowed`, the report's
  `output_overhung`, and a row of `jam_capped`): the rod's force at its mid-plane, 69.4 mm
  past the hub's end on either, against Fr2 a / (b + x) = 3363 N: walking SF 8.37 on B, 7.6
  on A; at the cap 1.77 30° from the toggles on B (1.3 outside 25°, 1 outside 20°), 1.49 on
  A (1.3 outside 30°, 1 outside 20°: A's least part there), rated with the other parts over
  the whole turn. (The NRV040's had been 1.31 and 1.16 at round 1's cap, unrated.)
- **A's hand crank** (M-3; `handcrank.HANDLES`, `HANDLE_OVER_RADIUS`, `HUB_PRINT_MAX`,
  `cradle_report.hand_sweep`): 130 mm on a 230 mm reach, its 60 x 16 grip on an M8 x 90.
  - **The hand's room:** a hand round the grip (the grip and 20 mm, from the arm to past
    the bolt's head; the reviewer took 17) against every part over the whole turn at 0.5°,
    by adaptive surface sampling: 14.1 mm clear (the base stop's rail behind the cradle),
    the crank's own solids 17.8 (b2). B: 40.4 and 39.0 at 1°. A slow test checks both
    cradles at 1° (`test_a_hand_on_the_grip_clears_the_cradle_over_the_whole_turn`).
  - **The grip:** a full 6 L's worst start, 21.5 N at 130 mm run in (24.5 new), under
    MIL-STD-1472H's 22 N. The xfail on A's grip is gone.
  - **The options, measured** (an occupancy map of A's parts in the input's frame over the
    turn, `/root/.cache/spiderpig/tmp/r2-fix-drive/scripts/xr_map.py`, `xr_judge.py`): at
    Figure 13's light-load maximum, 125 mm, the hand clears from a 215 mm reach (18.8 mm)
    but the start is 22.4 N, over the bar; 127.5 mm is 21.9 N (too near it); 130 mm 21.5 N
    with 13.8 mm of hand room; 140 mm 3.8 mm; 150 mm meets the base stop's rails. A
    heavy-load crank, 190 mm or more (14.7 N, a 25 mm grip), meets A's bottle and the base
    stop's rails at every reach from 100 to 300 mm. So A's crank is 5 mm over the light
    row's radius (`HANDLE_OVER_RADIUS`, which the test allows A alone); inside a row on A
    takes a counterbalance (the user's no) or another ratio (Phidgets' 50 series is 30:1
    only). Not tried: the worm under the output (the gearbox turned over), which moves the
    input 100 mm down.
  - **Its tube:** 230 mm past the input's end, so the hub and tube print in two PETG
    pieces over 200 mm (`HUB_PRINT_MAX`): a 22 mm hex spigot into the head's socket, an M3
    x 10 button head through the socket's wall into a heat-set insert. B's (100 mm) prints
    whole. The input's radial load at A's grip: SF 5.2 at the clutch's top (156 N allowed
    292 mm out).
- **The toggle zones' cue** (M-4; `handcrank.TOGGLE_TURNS`): the output hub turns with the
  crank on the gearbox's far face, in the user's view. At set-up, paint-pen lines across the
  hub and the housing at rest, 2.5 handle turns from rest, 2.5 before full tilt and at full
  tilt (both hold jam SF 1.3 outside 30° of crank, 2.5 turns); the use
  limits: nothing touches the cradle or the bottle while the hub's line is between a pair
  ("the bottle barely moves per turn there"). Free: the paint pen is the pack marks'.
- **The minors:** the end screw's torque on its grade (m1, above); the riser's nuts for the
  spacer variants (m2); the sleeves flat on the face (m3); the keyways cut under the key
  (m4); the setting increment and no self-tapped M8 (m5: the plungers thread through nuts);
  the cap keyed on the journal's flat, over a greased M16 washer (m6); the hub, arm, cap and
  grip in PETG (`handcrank.PETG_PARTS`, `bom.part_filament`) and "check the clutch after a
  trip in a hot car" (m7); the arm's bar 14 mm deep (m8: an out-of-plane 100 N at B's grip
  21 MPa, at A's 32); A's verdict no longer "the one to build ... every check passes" (B is
  the main design), ORDER.md's Phidgets cart with its duty and checkout note
  (`order.LINE_NOTES`), threadlocker 243 throughout the drive's steps, a tools list in the
  BOM's notes (`bom.CRADLE_TOOLS_NOTE`) (m9); the drill bypass in the guide and the use
  limits: run the box in before it goes on, never a drill on the built cradle's input, nor
  a hand on the hub or the clutch's flange (m10).
- **The packs** (`spiderpig pack-inspect`, by their plans): B 740 x 316 x 601 mm, 140.7 L
  (140.5 before), every piece supported; A 700 x 212 x 601 mm, 89.3 L (78.0 before, 185 mm
  tall): the NRV050's housing and riser stand 27 mm taller off the drive panel's outer
  plate, which lies face down. `PACK_BOX` (A) is 215 mm tall now.
- **B's board is 684 mm** (`Stand.base`, 2 mm more at the rear): the NRV050 (1.45 kg more)
  and its hand crank took the 682 mm board's rear tip from 25.1° to 24.9° (the bar 25°);
  684 holds 25.1°.
- **Numbers** (B, A):

  | | before (round 1) | after |
  |---|---|---|
  | gearbox | NRV040, 1.28 / 1.22 x the walking peak on its 45 N·m maximum; the cap 1.36 x over it | NRV050: 2.42 / 2.30 x; the cap 1.36 / 1.29 x under its 85 |
  | clutch's cap (and the jam case) | 61.3 / 61.3 N·m, "±10 % once set" | 62.5 / 65.7 N·m at the bench check's top (8 %); unchecked, greased, 0.72-1.47 x |
  | grip at a full 6 L's worst start | 20.8 N at 125 mm / 32.2 N at 85 mm | 21.3 N run in, 24.2 new at 125 mm / 21.5, 24.5 at 130 mm |
  | output shaft's radial load (walking; at the cap 30° off) | unrated (1.31 / 1.16 at the cap) | 8.37 / 7.6; 1.77 / 1.49 |
  | jam SF at the cap, 30° from the toggles | pillar O2 1.36 / b2 1.80 | pillar O2 1.34 / the output shaft 1.49 (b2 1.70) |
  | the hand on the grip, least room | B 21 mm / A -11.4 (through b2 and Tr2's screw for 64.5° of crank) | 40.4 / 14.1 mm |
  | the clutch unchecked (greased drift, worn seats) | "±10 %" | its cap 85.1 / 89.5 N·m, its slip down to 0.93 x the start; set new and never re-checked, 71.1 / 74.7 |
  | keyed plates | one 0.94 / 0.89 walking, the four 2.15 x the cap | one 1.41 / 1.34 walking, the three 2.37 / 2.26 x the cap (on the sheet's thinnest) |
  | rod and crankpin at the jam | 1.71, 2.11 / 2.32, 1.81 | 1.68, 2.07 / 2.17, 1.69 |
  | stand | 25.1° (682 mm board) / 24.0° | 25.1° (684 mm) / 24.5° (to its drive side; bar 15°) |
  | mass (the drive side's gearbox) | 2.1 kg | 3.45 kg |
  | BOM total | USD 723.26 / 600.52 | USD 763.94 / 642.52 |

- **Cost** (the BOM's estimate; every changed purchase from a page rendered 2026-10-14 or
  15, the cutting the BOM's fit to SendCutSend's quotes): **B USD 723.26 -> 763.94**
  (purchases 421.68 -> 448.16, cutting 301.58 -> 315.78); **A USD 600.52 -> 642.52** (purchases 340.58 -> 368.74, cutting 259.94 -> 273.78; its tube's joint adds an insert and an M3 x 10). B's purchases:
  the gearbox +29.60 (TRM4402_0 92.50 for TRM4401_0 62.90), its shaft +0.70 (TRM4008_0
  6.85); the riser's studs and nuts 5.36 for the spacers and M6 x 25s' 7.60; the shims
  -6.40; the M10 end screw 1.14 (Bolt Depot 13370); the clutch 16.83 (the plungers 13.77,
  their 1/2"-13 nuts and jam nuts 2.58, Bolt Depot 2566 and 4124; the M16 thrust washer
  0.48, 4522) for the balls, springs and set screws' 13.10; the grip's M8 x 90 1.68 (5827)
  for the 85's 1.53. Cutting: the seat ring and the bigger block plates +12.61 (0.125 in
  6061), the outer plate's wider pad +1.59.
- **The landed cost** (`order.LINE_NOTES`, in ORDER.md's "Before you order"): Phidgets'
  cart, the gearbox and its shaft, USD 99.35; made in China, so US duty is collected at
  checkout through UPS Global Checkout (phidgets.com/docs/U.S._Tariffs, rendered
  2026-10-15: since 2025-05-02 every order, no de minimis, with brokerage and a fee of the
  checkout's own): 40 % on the gearbox (HTS 8483.40.50.10: 2.5 % + 12.5 % Section 301
  Forced Labor since 2026-07-16 + 25 % Section 301) and 50 % on the shaft (8483.10.50.00),
  tariffs.wove.com rendered 2026-10-15: USD 40.43, so **USD 139.78 before UPS's shipping,
  the brokerage and the checkout's fee**, which show only at checkout (unpriced: the cart
  wasn't run). Round 1's NRV040 cart was USD 97.29 so; the 50 series adds 42.49 landed
  before shipping. Not in the BOM's total.
- **Not done:** a force limiter in the rod (the only cure for the toggle zones); a guard;
  a turn counter (the paint marks cost nothing); a counterbalance (the user's no).
- **Where:** `spiderpig/construction/handcrank.py`, `spiderpig/cradle_drive.py`,
  `spiderpig/cradle_report.py`, `spiderpig/config.py`, `spiderpig/construction/cradle.py`,
  `spiderpig/hardware/cradle_catalog.py`, `spiderpig/hardware/fasteners.py`,
  `spiderpig/hardware/bom.py`, `spiderpig/hardware/order.py`, `tests/test_cradle.py`.

### 2026-10-15: round 2's drive meets the stand round (the user's calls of 2026-10-15)

- **What:** the stand, use and guide round (merged at 5b0e32e) merged into round 2's drive,
  with the user's four calls on what the two rounds disagreed about
  ([DECANTER.md](DECANTER.md): "The base board", "The drive", "Flat pack").
- **One toggle cue, the gearbox's paint marks:** they mark both toggles on both cradles for
  nothing. The stand round's red toggle flag on B (`toggle_flag`, its two M5 screws and
  nuts, the stand nodes `S1` and `S2`, their holes in the outer plate and their claims;
  `TOGGLE_FLAG`, `TOGGLE_ZONE`, `toggle_zone`) is gone; the use limits' third line, the
  guide and the cradle page name the marks. A render of B's crank area at rest
  (`/root/.cache/spiderpig/tmp/r2-fix-drive/out/render/`) shows the gearbox's far face and
  its output hub in the open from the drive side's rear corner, from above and from
  outboard: the hand crank turns beside the gearbox, not over its far face.
- **The riser's lifts** (`cradle.RISER_LIFT`): the drive panel's lay-flat blocks must stand
  as far as the gearbox does off the outer plate, 114.7 mm on the NRV050 (87.5 on the
  NRV040). The user's call was taller blocks and fewer plates, the whole riser still the
  drawn decanter's. Built so (blocks of 112.2 mm, plates of 5 and 3), the stacks without
  the blocks stopped at 10 mm, under the locator's walls: three listed vessels (the RIEDEL
  Cabernet Magnum, Le Château, the Zalto Burgundy) had no stack the rule allows. So the
  blocks stay 85 mm and stand on two 27.2 mm lifts, half plates under them in the riser
  and under each block when the panel lies down: 114.2 with the boss. The plates are 5 and
  4 mm, the whole set 123.2 mm (122 before), the drawn decanter's rim 31.8 mm under the
  light's clip. Every listed vessel's stack is within 7 mm of the stand round's.
- **A's 130 mm crank** (`handcrank.HANDLE_OVER_RADIUS`), 5 mm over MIL-STD-1472H Figure
  13's 125 mm light-load maximum: accepted as a documented deviation, the grip under its
  22 N bar (21.5 N run in).
- **The use text** (`cradle.use_steps`): its clutch figures come from
  `handcrank.drawn_clutch` and `handle_r` (the setting 1.3 x the measured start, about 28 N
  at either grip for a full 6 L), its toggle zones from `handcrank.TOGGLE_TURNS`; the drill
  and hub bypasses and the PETG hand crank are in its reference lines. One tools list,
  `cradle.TOOLS` (the guide's cover): the drive's torque wrench to 20 N·m, its 13 mm and
  3/4 in wrenches and 14 mm bit joined it; `bom.CRADLE_TOOLS_NOTE` went.
- **B's board is 686 mm** (`Stand.base`, 2 mm more at the rear): the window under the base
  stop and the NRV050 together took the 684 mm board's rear edge to 25.0° to the tenth; 686
  holds 25.2° (28.4° pouring). A: 24.3° to its drive side (bar 15°).
- **The rest poses on the NRV050** (`cradle_pack.rest_poses`): B's cradle lying on its
  blocks tips at 15.7° (16.9 on the NRV040; bar 15°), the drive side with the box at
  15.0°, at its bar (16.3), the drive panel alone at 32.2° (35.2); A's lying cradle 19.3°.
  The heavier, further-out gearbox lifts the lying cradle's centre of mass; the steps keep a
  hand on the box. Wider lay-flat points (`cradle.LAY_BLOCKS`) are the stand's to weigh.
- **The packs:** B 740 x 316 x 601 mm, 140.7 L; A 700 x 212 x 601 mm, 89.3 L (unchanged by
  the merge).
- **Where:** `spiderpig/construction/cradle.py`, `spiderpig/construction/handcrank.py`,
  `spiderpig/construction/assembly.py`, `spiderpig/cradle_pack.py`, `spiderpig/cradle_report.py`,
  `spiderpig/guide/scenes.py`, `spiderpig/hardware/bom.py`, `viewer/src/decant/liquid.ts`,
  `viewer/src/decant/pour.ts`, `tests/test_cradle.py`, `tests/fixtures/cradle/riser_reference.json`.

### 2026-10-15: the cradles' saddles made to hold real bottles (design review round 2, mounting)

- **What:** the saddles, the lip gauge and the base stop fixed where review round 2 found
  them failing ([DECANTER.md](DECANTER.md), "The saddles, adjustable", "Review round 2";
  the review `/root/scratch/decant/review5/r2/mounting.md`, its probes in
  `/root/.cache/spiderpig/tmp/r2-mounting/` on the development box).
  - **The rear post beside the lead screw (N1, the blocker).** The rear V block was guided
    by one post at -z, 213 mm across from the screw that pushes it up: a drawer, which
    jammed raising it past a friction of 0.149 on the post (0.095 at a 750 ml's setting),
    under the 0.24-0.30 measured for printed PETG on steel (Stoimenov et al., Tribology in
    Industry 46(1), 2024). The post now stands beside the screw at +z, 27 mm in front of
    it (`cradle.REAR_POST`): the push can't cock the block, and it jams only past
    `engagement / 54`, 0.74 at a 750 ml's setting (40.1 mm of the boss on the post's
    round) to 0.98. `cradle_report.drawers` rates it at every bottle's setting, and the
    front block's box guide too (m5: 40 mm off its middle, it had bound past 0.40; its -z
    wall 12 mm taller and a cheek up the block's -z face, `cradle.FRONT_GUIDE`, make it
    0.56, greased); `saddles` lists them (2 x the most friction cited for the faces).
  - **The post 15 mm, plated** (m3, and N1's own consequence). With the post on the screw's
    side it alone takes the roll of a load at the axis, as it did; counted with what rides
    the block at its weight (about 1 kg: the report's, not the 0.75 kg the test had used), a
    6 L hung a hair off the rods took the 12 mm post at 2.99 x its yield (2.96 with this
    round's PETG rails), under the 3 asked. MISUMI PETR15-150 (15 mm, M8, electroless nickel, a stocked
    length, USD 19.25: 2.54 less than a plated 12 mm, PETRF12-130), 5.7 x; its socket 30
    mm deep under a collar (the boss 53 mm long over it), rated in bearing (8 x); an M8 x 12
    button head into its foot (Bolt Depot 6654, USD 0.40), the screw table's first M8
    (`fasteners.SCREWS`).
  - **Real bottles (N2).** The base stop's named grooves gave way to a plain length scale,
    a groove every 5 mm (`cradle.stop_marks`), and a vernier on the gate's pointer reaching
    forward along it (`cradle.VERNIER`: four lines 6 mm apart, the k-th meeting a groove for
    k mm); the gate sits 2.5 mm behind the base (`cradle.STOP_GAP`, it had been 0.5: a
    bottle 0.7 mm long put its rim under the gauge's pad), and the guide measures every
    bottle. `cradle.SIZES` against the makers' pages rendered 2026-10-14 and -15: the
    750 ml became Saverglass's 4427 (300.5 mm, it had been 300); the 3 L is Saverglass's
    Jero Vignoble 7201 (470.3 x 140.8 mm, 2.9 kg: the record's figures; the review compared
    it with their Standard 0718, 474.3 x 135.2 mm, a different bottle); the magnum and the
    6 Ls match to 0.2 mm. The 0718 and 750s of 308 and 315 mm lie clear at their own
    settings, as does every size against its gate.
  - **The rails (N3).** Their tongues have crush ribs on top (`cradle.RIBS`), the socket's
    clearance moved above them, so they sit down on the sockets' floors, flush and without
    play (0.3 mm, 0.6° of rock, before); a 4 mm keel down each rail's outer face
    (`cradle.RAIL_KEEL`) doubles its stiffness (61 000 mm⁴, was 30 000), and its flange,
    keel and tongue end level at -91 mm so it prints standing. `balance` now rates the
    flanges' sag behind the rigid V block (380 mm): the mouth lifts at most 0.89 mm (a
    methuselah at rest), 0.23 past level (1.5-4 mm before).
  - **The lip gauge fixed (N4, m2).** The review's simplest option: its bridge flat on the
    post's flat top, the bolt under it, tight (`cradle.gauge_fixed`); no hinge, stop or
    lug. A bottle is uncorked standing on the table before it goes in; one set too long
    can't lie in under the gauge. A knock on the pad: 8.3 MPa on the post's edge, SF 5.7.
  - **The rear knob locks (N5).** It slides on a hex hub set-screwed on the screw's top and
    drops over a star of 12 teeth on the tower's arm (`cradle._lead_screw`,
    `cradle.LOCK_TEETH`): a positive lock, whatever the grease (the teeth at 0.4 MPa under a
    hung 6 L's turn back, the thread greased). Lifted 3.6 mm, it turns. The TPU brake
    washer stays, slowing the screw with the knob lifted (4.1 x dry, 0.8 x greased: told,
    not rated). The grease note now keeps grease to the thread above the collar, off the
    collar, the washer and the teeth.
  - **The print basis (N6, m9).** `cradle_report.PRINTED`: Prusament's datasheets (PLA
    v1.1 of 2022-07-27: 51 MPa in its layers, 17 across; PETG v1.1 of 2022-02-16: 47 and
    18; specimens printed solid); each row rated in or across its layers as the part prints,
    a quarter of it for a load that stays, 1/2.5 for the pour's or a knock's
    (`SUSTAINED_SF`, `TRANSIENT_SF`). `PLA_ALLOW` is now 4.25 MPa, a quarter of PLA's 17
    across (it had been 10, "a quarter of ~40", which no datasheet gives). The parts that
    carry the bottle print in PETG at 100 % infill (`cradle.PETG_PARTS`, `meta["petg"]`,
    `bom.part_filament`; on hand); the rails stand (their T-slot's lips bend in their
    layers), the gauge lies on its side; the lips are 5.5 mm (5) for a firm hand's 1 N·m.
  - **Minor:** base first when loading (m1); the gauge's pad 0.5 mm lower (m4: 3.25 mm down
    a 750's rim, clear of a methuselah's bore) and the guide measures the neck 80 mm from
    the lip, the body where it lies (m4); a drain from the +z rail's T-slot's back end
    (m6); "tug the gate" (m7); the T-slot's ends chamfered 0.5 mm at the block-to-rail
    joint (m8).
  - **Lighter elsewhere:** the rear V shell 7.5 mm (9), the rear carrier's -z block 15 mm
    wide (29: it only rides the rods now), the rails' flanges 7 mm (8.5): the printed
    mounting parts are 1220 cm³ solid (1223 before), so the 15 mm post's 90 g more steel and
    PETG's density leave B's stand tipping past 25° (25.02° on 682 mm, 25.08 before;
    merged with round 2's stand, 25.1° on its 684 mm board).
- **Why:**
  - **The post beside the screw, not a second post or PTFE bushings:** a second post near
    the screw would have needed the tower moved; bushings (about 0.1) would still jam at a
    750 ml's setting. A post at the -z end with the screw at +z can't be made free at any
    plausible friction: the push is 213 mm off.
  - **The fixed gauge, not a 50° stop or a return spring:** both keep a hinge that wine
    can stick; uncorking in the cradle was three tasks for two hands. The cost: no
    uncorking in the cradle (the -50° rest still takes an opened full bottle).
  - **Teeth under a lifting knob, not a jam knob or a sprung detent:** positive (no
    friction in it), all printed, no loose part, one hand: lift and turn.
  - **Crush ribs, not glue or an interference fit:** a printed press fit is either loose
    or doesn't go; ribs crush to fit across a print's tolerance, and the rail still comes
    off its two screws.
  - **The rated strengths by layer:** a part's weak direction is the one its print puts
    across its layers; rating every part against the across-layer figure would have thrown
    away the in-layer strength the rails' new pose buys.
- **Numbers** (`cradle_report.saddles`, 2026-10-15): every row over its bar; the worst
  printed one the rails' tongues, 4.2 x (4) at rest; the post 5.7 x its yield; both
  guides 2.5 x or more the most friction cited. `balance`: every bottle centred 62.9 mm
  or more inside its support, the mouth lifted 0.89 mm at most. Clash-free at rest, level
  and full tilt on B and A, every size at its setting with the bottle at the pad and
  against the gate, and the off-list bottles above; the finger gap 26.4 mm on B, 26.5 on
  A, as before. The cradle 0.12 kg heavier (the post's steel): B's crank peak 35.4 N·m
  (35.1; the gearbox 1.27 x its 45 N·m, was 1.28; 21.0 N at the grip, was 20.8), A's 37.2
  (36.9; 1.21, was 1.22; 32.4 N, was 32.2); B's stand tips at 25.02° (25.08), A's 23.9°
  (24.0); A's clutch clears the worst start by 1.10 (1.11; its floor 1.1). The pack
  (`spiderpig pack-inspect`): B 741 x 316 x 600 mm, 140.7 L (140.5), A 700 x 185 x 601 mm,
  77.9 L (78.0), every piece supported. The bought parts: USD 41.00 (38.50), the post and
  its screw.
- **Merged with round 2's stand** (its fixed risers, the board's window, B's 684 mm board,
  the rated rest poses): the rear knob's arm, teeth, knob and hub sit 10 mm lower (the lead
  screw's boss 10 mm shorter, `cradle.ARM`, `cradle.REAR_HUB`): at 19 mm the hub had become
  the cradle box's third resting point turned over (3.8°); with the hub's top at 6 mm the
  box rests on the base stop's knob again, 44.3° (the bar 35). The use steps carry the
  measuring, uncorking, gate and knob lines (`cradle.use_steps`). On the merge B tips at
  25.1° over its rear edge (40.0 front, 29.7 drive side, 28.4 passive side), A at 23.9°;
  the base stop clears the table through the board's window by 43.4 mm (B) and 43.5 (A).
- **Not done:** the light, the tie rods, the rails' length per bottle (as round 1); the
  impériale's 31.5 mm bore has its top 5 mm under the pad (the first of its pour runs down
  the pad's back into the stream: the pad must reach a 750's rim).
- **Where:** `spiderpig/construction/cradle.py`, `spiderpig/cradle_report.py`,
  `spiderpig/hardware/bom.py`, `spiderpig/hardware/cradle_catalog.py`,
  `spiderpig/hardware/fasteners.py`, `tests/test_cradle.py`.

### 2026-10-15: round 2's drive meets the mounting round

- **What:** the mounting round (merged at 507457e: the rear knob stack 10 mm lower, the
  PETR15-150 post, the bottle's carriers in PETG, the use card's fourth line) merged into
  round 2's drive; both rounds' parts kept ([DECANTER.md](DECANTER.md): "The drive",
  "Loads and checks").
- **Re-rated on the heavier cradle** (0.12 kg more in the saddles; B 14.75 kg, A 13.68):
  a full 6 L's drawn start 2.68 and 2.817 N·m at the input (`handcrank.DESIGN_START_NM`),
  the clutch's cap 63.0 and 66.2 N·m (`config.PRODUCT_CLUTCH_NM`, `PRODUCT_JAM_NM`; 62.5
  and 65.7 before), 1.35 and 1.28 x under the NRV050's 85 N·m (floor 1.25); the crank's
  peak 35.4 and 37.2 N·m (SF 2.40, 2.29); the grip 21.4 and 21.7 N run in (bar 22); every
  part 1.33 (B, pillar O2) and 1.48 (A, the output shaft) or more at the cap 30° from the
  toggles; the keyed plates 2.35 and 2.24 x the cap. The clutch's margin is not the
  mounting round's 1.10 on A: set at 1.3 x the user's measured start, its least slip in
  the check's window is 1.2 x the start on both, whatever the mass.
- **B's stand:** 25.2° over its rear edge on its 686 mm board (28.8° drive side, 31.2°
  passive side, 41.9° front; 28.3° at the rear pouring); A 24.5° to its drive side.
- **The rest poses:** the drive side with the cradle box lying on the blocks tips at 14.3°
  (16.3 on the NRV040; the box with the mounting round's saddles weighs 70 g more and its
  knob stack sits lower, which steadies the box on its edges: 44.3°, but lifts nothing
  here). No place for the blocks on the outer plate does better than 15.0° (every pair of
  stand nodes and pillars, the O1 block turned every 15°, and moved up off O1 it leaves
  the plate): its test's bar is 14° with the reason, the steps keep a hand on the box. A
  wider stance for the lying cradle is the stand's to weigh.
- **Where:** `spiderpig/construction/handcrank.py`, `spiderpig/config.py`,
  `spiderpig/hardware/bom.py`, `spiderpig/hardware/fasteners.py`, `tests/test_cradle.py`.

### 2026-10-15: the cradles' structure and laser-cut parts (design review round 2, structure and parts)

The round's linkage and laser-cut-parts reviews (`/root/scratch/decant/review5/r2/linkage.md`,
`lc_parts.md` on the development box). The user's rule for this round: "Let's optimize the
manufacturing method in later spins, not this one". Correctness, strength and honest numbers
are in. Vendors and materials chosen to save money are out, and are listed at the end with
their numbers.

- **A side as one frame (B1's rule, and what it found).** Each side's posts and both frame
  plates are now solved together (`cradle_structure.side_frame`). Each plate is a grillage out
  of its plane, the outer one standing on its feet, the inner one held by nothing but the
  posts. The posts are stiff along their axes. The gearbox's shaft turns the outer plate at the
  crank by the rod's force times its height over the plate.
  - The springs it replaces (`plate_spring`, each post's end on its plate's strips clamped at
    their far nodes) held no plate's own equilibrium out of its plane. They also missed the
    gearbox's turn and the inner plate twisting as a whole.
  - An independent 3D frame of 12-dof beams agrees within about 10 %. A portal case is a
    unit test.
  - The review's own probes had applied each post's moment 90° off: the slopes' generalized
    moment as the turn. Its "196 MPa at D1's column hole" is 145 MPa turned right.
  - **Rated this way, neither cradle holds the clutch's 61.3 N·m cap everywhere.** B: the
    outer plate's front leg from D2 to the foot 0.62, pillar O2 0.75, column D2 1.0, the rear
    leg at the flag's screws 0.89. A: its front leg 0.45, column D3 0.9, the strut G-O 0.69,
    the inner plate 1.19.
  - Walking, both hold: B 2.86, A 3.74.
  - **On the merged drive** (round 2's drive, `decanting-cradle` at d25b7b4, merged into this
    round: the NRV050, its clutch's cap re-rated on this round's heavier cradle box, 63.7
    N·m on B and 67.1 on A). B: the front leg 0.60, pillar O2 0.73, column D2 0.97; its
    rear leg O-Fr 0.99 at O, no longer under 1.3: the toggle flag's screw holes went with
    the flag. A: its front leg 0.41, column D3 0.83, the strut G-O 0.63, the inner plate
    1.09, and pillar G 1.25 (1.38 at 61.3), on the list now. Walking: B 2.83, A 3.71.
  - Two causes, which the springs couldn't see:
    - Each side's posts stand nearly on one line, while the drive's force runs across it.
      The floating inner plate can't turn their ends back, so the box sways and pillar O2
      bends as a near cantilever: 47.9 N·m against 26.7.
    - The gearbox turns the outer plate at O while the pillars carry the force 54 mm up.
  - **Open item: the side frames at the cap** (`cradle_report.OPEN_AT_CAP`). The fix is a box
    toward the rear, modelled on B and not built:
    - 12 mm columns stand along the rail at 60, 180 and 290 mm behind the mouth;
    - the inner plate runs a bottom chord from D2 to them.

    That takes B's pillars O1 and O2 and column D2 to 1.43, 1.58 and 2.55. Three parts stay
    under 1.3 and want one more iteration: the inner plate's leg D1-D2 in its plane (0.86),
    the outer plate's O2-Fr (1.08) and the last rail column (1.17).
  - **It is not built**, for three reasons. It redesigns the stand round the gearbox's mount,
    which the round's drive is changing (NRV050). The pack and the guide change with it. And
    it is a cost the user should see first.
  - The parts open at the cap are listed in `cradle_report.OPEN_AT_CAP`. The report's verdict
    names them, and the tests hold every other part at 1.3, then xfail on them.
- **Re-rated on the merged drive** (the drive round's own way, its merge with the mounting
  round). This round's cradle box is 0.3 kg heavier, all of it the 12 mm tie rods
  (3.61 kg; 3.2 before). So a full 6 L's drawn start is 2.712 N·m at B's input and 2.856 at
  A's (`handcrank.DESIGN_START_NM`; 2.68 and 2.817), and the clutch's cap 63.7 and 67.1 N·m
  (`config.PRODUCT_CLUTCH_NM`, `PRODUCT_JAM_NM`; 63.0 and 66.2).
  - The gearbox: 1.33 and 1.27 x under its 85 N·m maximum (floor 1.25), 2.37 and 2.25 x
    over the walking peaks of 35.8 and 37.7 N·m.
  - The grip run in: 21.7 N on B. **A's is 21.97 N at its 130 mm crank, at MIL-STD-1472H's
    22 N bar to the report's tenth** (21.7 before). That is open, the user's call
    (`cradle_report.OPEN_GRIP`; its test holds it no worse and xfails). The drive round had
    judged 127.5 mm's 21.9 N too near. 132.5 mm gives 21.6 N with 12.2 mm of hand room
    (14.7 at 130, 9.7 at 135; `hand_sweep` at 1°). But that is 7.5 mm over Figure 13's
    125 mm, and 5 mm over is what the user accepted.
  - B's stand: 25.5° (rear), 28.9° to the drive side; A's 24.5° to its drive side.
  - Every other part holds the cap at 1.3 or more 30° from the toggles: B 1.31 (the inner
    plate), A 1.38 (the tie rods).
  - The cradle box 3.6 kg in the guide's lift step and the keepers' bench check (30 N, 1.7 x
    half the box).
- **Two clashes the slow tests found**, there since this round's first commit, before the
  merge:
  - The front thumbscrew (`cradle.FRONT_JACK`) stood at x 63, its tip in the carrier's floor
    touching a 10 mm Tf1 and 1 mm into the 12 mm one. It moves to 65, midway between the
    front rods, 1 mm from each; the V block's tab is 2 mm longer for its nut (`FRONT_TAB`).
  - B's 40 mm front leg's round end dipped 7.5 mm into the base board at its foot. The
    outer plate is cut off at the board's top (`cradle.CradleStand`); the leg's section
    through the foot stays whole.
  - The recorded room under the cradle box (`tests/fixtures/cradle/riser_reference.json`) is
    up to 2 mm less with the 12 mm rods: the Zalto Burgundy's riser by the rule is 73 mm
    (78), its stack unchanged.
- **B1's widening.** B's outer front leg is 40 mm from O2 to the foot (`STANDS`' `wide`,
  layer 0 only: the plan is the same). On the side as one frame, that leg went from 0.44 to
  0.62, and the side's stiffness across at O1 from 2.0 to 2.67 N/mm.
- **The plate rows (B1's rule).** Every member end is rated out of its plane on the cut plate
  (`cradle_structure.plate_rows`):
  - on a straight run, the net section through its hole;
  - at a junction, at its strip's root.

  They replace the rows that spread a post's moment over the summed sections at the node. A
  frame row's `governs` names that member. Members' sway at an untied node uses the side's own
  stiffness there (B's O: 12.4 N/mm, not 2.9: minor 3).
- **The cradle box (M1).**
  - The tie rods are NETRF12 on M6 x 16 A2-70 button heads, tightened to 6 N·m
    (`cradle.TIE_D`, `TIE_M`, `TIE_ROD_SCREW`, `TIE_TIGHTEN_NM`), in 6.4 mm holes.
  - The box's racking is rated (`cradle_report.box_racking`): the passive half's moment about
    the mouth crosses in the rods, fixed at both ends.
    - Walking, B's rods are at SF 7.4 and the joint stays closed (1.97 x its end moment).
    - Jammed (half the couple), the rods hold 1.75 and their screws' prying 2.63 (A: 1.51 and
      2.28).
  - The rods sit 0.95 mm lower (`decanters.TIES`) so the V blocks still sit on their tops.
  - Prices, NETRF12-274.9 from MISUMI's page, rendered 2026-10-15: USD 21.07 at 1-4, 15.33
    at 5-9, 7.73 at 10-19 (NETRF10's 15.30, 11.08, 5.57).
  - The extra M6 heads' height under the cradle link adds 0.6 mm to B's stack (84.65 mm, 16
    layers): the plan's one change. A's inner plate's G-D1 is 23.6 mm so the 12 mm front rod
    passes it by the plan's margin.
- **The blocked cradle (M2).** `blocked`'s `threshold` gives, by superposition, the first part
  to fall to 1.3 and to 1, and at how much more force at the grip.
  - B, against the front tie: its outer plate reaches 1.3 at +4.2 N and 1 at +5.8 N (577 N at
    the tie).
  - A: +6.7 N and +9.3 N.
  - A 6 L's base against a shelf is a third case.
  - "The glass breaks first" is gone: nothing supports it.
- **The rod (minor 1)** as a beam-column at its far pin's offset (`rod_secant_sf`): B 2.05,
  A 2.65 jammed.
- **The posts' end joints (minor 2).** M6 x 16 A2-70 button heads (Fastenal, ISO 3506-1,
  pages rendered 2026-10-15: USD 0.37 each) with DIN 125 washers, in 6.4 mm holes (ISO 273
  fine), on the pillars and the columns. Bolt Depot's 18-8 names no class; its grade chart's
  A-2 minimum yield is 210 MPa, A2-50's.
- **Minor 5.** b2's P end passes 1.93 mm from b3. That is over the planner's 1.0 mm margin,
  so it is left as it is.
- **Honest cutting prices (parts B1).** `bom.CRADLE_CUT_FITS` is refitted per sheet on the
  review's 40 live SendCutSend quotes of 2026-10-14.
  - Aluminium: `k x area^0.249 x cut^0.229 x blank^0.326`. 7075 runs 1.87 x 5052 one-off,
    2.16 x at two.
  - The birch is fitted on its blank's sides.
  - Each sheet keeps its own quantity discount (x0.67-0.78 at two).
  - Every quoted file is within 11 %. B's total is USD 760.28 against 758.19 live, A's
    580.03 against 573.82. Nothing is left unpriced.
  - Today's files: B's cutting USD 768.44 (the BOM had printed 301.58), its whole build
    1,190.54. A's cutting is USD 580.35, its whole build 920.93. On the merged drive: B's
    cutting 797.23, its build 1,307.24; A's 605.39 and 1,005.50.
  - `MadeRow` carries each part's cut length, pierces and blank. ORDER.md's upload table
    gives each file's estimate and whether it arrives deburred.
- **Cut rules (parts M3, minor 5).** SendCutSend's pages fetched 2026-10-15 give:
  - the thick sheets' minimum part (12.7 x 19.05 mm);
  - each sheet's thickness tolerance (`thickness_tol`);
  - deburring 1 x 3 in to 36 x 46 in, anodizing to 23 x 23 in, flatness ±0.030 in a foot.

  A part under the deburring minimum is a warning (`manufacture.deburr_issue`), on the
  cradles' sheets only: the walkers' crank plates would gain warnings and the gate stays
  identical. The 0.8 mm inside corner stays the joinery plan's: no page gives one.
- **Fit-up (parts M3, M5, minor 9).**
  - The guide's first stage (`assembly.CRADLE_ARRIVAL`): deburr the as-cut links by hand,
    caliper every link at its pin holes.
  - ORDER.md: order the MISUMI pins after the parts arrive (the stack + 0.3 mm).
  - The pins' ring and clip steps: sand a printed spacer by the excess.
  - The board on the flat table first, its concave face down.
  - `cradle_report.take_up` rates each pin's play against the sheets' tolerances: B's P is
    -0.49 mm at worst, its Y -0.06; A's E -0.52.
- **The BOM's laser size** is the flat blank for a product (minor 8).
- **The links' shapes (parts M3, M4, minor 1, minor 4).**
  - The rods b1 are 26 mm wide (`config.PRODUCT_CHORD_R`): SendCutSend's free deburr.
  - **Fillets.** Each product link's chords and bosses are closed: grown by a radius and
    shrunk back (`plates.product_outline`). So every inside corner, elbow, boss cusp and
    window corner is rounded to that radius, and a window narrower than twice it is filled.
    - The radius is R15 (`plates.PRODUCT_FILLET_R`). B's cradle link b6 and A's b2 are R18
      (`plates.PRODUCT_FILLETS`).
    - b6's sliver window is gone and its elbows are rounded (the review's priced shape). A's
      b2 loses both its windows. B's b2 keeps its window, its three corners rounded R15.
    - What the closing adds is claimed as pills on points fixed to the link
      (`stack.Topology.add_link_points`, `plates.Outline`), standing at most 1 mm off the
      outline into the air (3 mm into a window the link keeps). `Build.xy` places those
      points, and `verify_plan` re-samples them. So the planner checks the fillets against
      everything over the turn, as it checks a chord.
  - **Bosses** (`config.PRODUCT_BOSS_R`), each a disc claimed as one. They bring the cut
    rules' hole-to-edge warnings up to SendCutSend's 2 t for 7075:
    - b1 at M, r 15.8 (9.6 mm of web, 1.5 t of 0.250 in);
    - B's b2 at O2 and b3 at O1, and A's b2 at G, r 17 (10.8 mm, 2.2 t).
  - **Checked on the design, 7,200 samples of the turn**, each link's cut outline against
    every other claim in its layer:
    - Nothing comes within the planner's 1 mm margin.
    - A's b2 passes a stand column at 5.3 mm.
    - B's b2 P end passes b3 at 1.93 mm, the same as before the change (minor 5).
    - B's b6 passes 2.8 mm outside pillar O1's head claim, in plan (minor 3). O1 ends at b3:
      its 18 mm printed ring is two layers under b6, and its M6 head and washer stop 3.5 mm
      under b6, 5.0 mm away at rest as fabricated. The review's 0.73 mm was in plan, to the
      ring.
    - Each link lies inside its claims (`construction.contract`).
  - The board's notch points, the locator's ears and the cleats' ends are rounded.
  - Products only. The walkers' identity gate against `planner-f667d6b` is identical.
- **Where:** `spiderpig/cradle_structure.py`, `spiderpig/cradle_report.py`,
  `spiderpig/cradle_drive.py`, `spiderpig/strength.py`, `spiderpig/construction/cradle.py`,
  `spiderpig/construction/pivots/standoff.py`, `spiderpig/construction/plates.py`,
  `spiderpig/stack/topology.py`, `spiderpig/hardware/` (`bom.py`, `order.py`,
  `sheet_catalog.py`, `cradle_catalog.py`), `spiderpig/manufacture.py`,
  `spiderpig/linkages/decanters.py`, `spiderpig/config.py`, `tests/test_cradle.py`.

**Deferred: the manufacturing method's savings** (the user's rule; the lc_parts review's live
quotes of 2026-10-14):

| change | USD | note |
|---|---:|---|
| b2 and b4 from 7075 to 0.187 in 5052 | -67.74 | b2 85.62 -> 40.00, b4 43.72 -> 21.60; the linkage review's strength check: b2 5.6, b4's buckling 1.52, the rear block 1.07 |
| the birch cut at home from one 24 x 48 in panel (Online Metals, USD 44.45) | -191.64 + shipping | the board, four cleats, the locator; a drilling template printed |
| or only the board routed, the locator printed, the cleats from offcuts | -95.70 | |
| OSH Cut instead of SendCutSend | +124.18 on the aluminium | its 5052 arrives undeburred |
| anodizing | +118 or more | A's 683.6 mm frames over the 23 in maximum; the 24 mm links can't be |
| frames in 0.250 in (2.37 x stiffer across) | +32.26 | not a saving: an option for the sway |

### 2026-10-20: the cradles' side frames redesigned to hold the clutch's cap (the user's brief: every part SF 1.3 or more at the cap, B first)

Round 2's structure entry (2026-10-15, above) left the side frames open at the hand crank's
clutch's cap (`cradle_report.OPEN_AT_CAP`): B's outer plate 0.60, pillar O2 0.73, column D2
0.97; A's outer plate 0.41, column D3 0.83, inner plate 1.09, pillar G 1.25. Two causes: the
posts stood nearly on one line across the drive's force, and the gearbox turns the outer
plate at O. Both are fixed; `OPEN_AT_CAP` is empty and the tests assert again (A's grip,
`OPEN_GRIP`, stays the user's call).

- **What was weighed** (B, `cradle_report.structure` with the stand varied, each rated at
  63.7 N·m; costs from the build's BOM):

  | option | worst at the cap | added | BOM |
  |---|---|---|---|
  | two rail columns at 220 and 280, an inner bottom chord, members widened (built) | 1.37 (unchanged parts; the frame 1.39) | 4 columns, 8 screws, 8 washers | +66.70 |
  | round 2's concept, three rail columns, the same widths | 1.36 at 160/230/290, 1.25 at 60/180/290 | 6, 12, 12 | about +80 |
  | one rail column at 240 | 1.28, the column itself | 2, 4, 4 | about +65 |
  | the inner plate tied to the board (a tab in a notch; the cheaper alternative tried) | posts 1.36 or more, but the rail buckles at 0.75-1.13 carrying the rod's reaction to the feet | a notch, cleats | not cheaper |
  | 0.250 in frames alone | pillar O2 0.77, column D2 1.02 | none | about +75 |

  - The rearmost column takes the inner plate's turn in its plane, whatever the number of
    columns: a third between doesn't share it. So two, which the order's 10-pack of B's
    post length already holds (6 used before): no standoff is bought for them.
  - The board tie was modelled by a penalty on the inner plate's motion at the tab (a
    script, not the library). It also needs the inner plate to reach the board, so it
    saves nothing on the plate, and its 4.75 mm tab bears on the birch's edge, found blind
    on the way back.
- **B, built** (`cradle.STANDS`): columns E1 and E2 on the rail, the inner plate's 40 mm
  chord D2-E1-E2 (its leg D1-D2 32 mm).
  - The rail is 44 mm wide (`cradle.rail_y`): its line, its feet's screws and the columns
    stand 10 mm higher, its bottom edge 0.5 mm over the board as before, so the columns
    clear the cleats' tops by 7.5 mm. The feet's printed brackets' uprights stand as much
    taller (`cradle._bracket`).
  - The gearbox's turn: the rear leg O-Fr 56 mm wide, O1-O 40. O2-Fr is 32, for its
    buckling.
  - At the cap: pillar O1 and b6 1.37 (unchanged), the outer plate 1.39 (the rail at E2,
    whose foot puts 26 N·m into it), column E2 1.40, pillar O2 1.58, the inner plate 1.61,
    column D2 2.66. Walking 4.15 or more (the crank's keyway). Over the whole turn 1.34 or
    more 30° from the toggles.
  - The side's stiffness across at O1 is 6.35 N/mm (2.67), and a 20 N push sways it 1.57 mm.
- **A, built:** a column E1 on the rail under D3 with the inner leg down to it (40 mm). The
  strut G-O is 96 mm wide (48), the legs from O 40, the rail 40 and 8 mm up (at 34 mm,
  5 up, the leg to E1 dipped 2.5 mm into the board: the clash test found it).
  - At the cap: the outer plate 1.37 (G-O at O), the tie rods 1.38, pillar G 1.40, the
    inner plate 1.45, column D3 1.61, column E1 1.77 (1.95 at 34 mm). Walking 3.94 (the
    keyway). Over the turn 1.36 (pillar G).
  - G takes 9.01 N/mm across (2.92; 8.64 at 34 mm), a 20 N push 1.11 mm. Two columns (-100
    and 60 mm) gave 1.38: one does.
- **Checked:** no column in a link's sweep (the nearest 94 mm, B's E2 from the crank's), the
  inner plates' new members under the tie rods' paths. The plans are the same (B 16 layers,
  84.65 mm). The clutch's case, the blocked cases and their thresholds re-rated.
  - A block at the front tie now finds B's inner plate first (0.26 at the cap); the frame
    starts to give at +6.7 N at the grip (+4.1 before).
  - At the rear saddle and a 6 L's base nothing falls under 1.3 within either clutch.
- **Heavier:** each side panel 0.61 kg on B (the cradle 16.45 kg, 1.24 more), the cradle
  1.10 kg on A (15.23 kg). The weight sits low, so the stand tips later: B at 26.7° rear
  (25.5), A at 25.3° (24.5).
- **The board cut:** the outer plate is cut at the board's top from the mouth's height
  (`CradleStand.realize`), not from the feet, which a raised rail moves.
- **Lay-flat** (`cradle.LAY_BLOCKS`, `REST_POSES`): the weight low on the rail moved the
  lying sides' centres away from the binding edge (O1's block to the gearbox).
  - B's cradle lying tips at 18.2° (15.4) and the drive side with the box at 17.7° (14.3):
    its test's bar is back at 15°, from round 2's drive's temporary 14°.
  - On D2's head the heavier drive panel alone tipped at 25.3°, under its 30° bar. The
    second block moves to the rail's column E1 (39.7°). A lies at 17.6°.
  - The breakdown is unchanged otherwise: the columns are inside each side panel, and the
    steps are the same (the pack pose, the hand crank off, 4 thumb knobs, 2 R-clips, the
    passive panel lifted off). The guide's lay step
    names E1's head (`cradle._blocks_words`); its column steps take the new columns as they
    are.
- **The pack.** Round 2's 18 mm taller pack (740 x 316 x 601, 140.7 L, to 740 x 334 x 600,
  148.1 L) is traced (`pack-inspect`, 723ff13 against dcd2693):
  - Round 2's 40 mm front leg made the passive panel's outer plate 9.6 mm longer at its
    front foot. The quarter-turned panel no longer fitted beside the drive panel's rod b1
    and pin P, 3 mm off the box's wall.
  - The packer set it 11 mm over, its inner plate on them: 16.6 mm higher.
  - The posts' DIN 125 washers made it 1.6 mm thicker (the stack 0.6 mm).
  - This redesign's inner chords would have taken it to 171 L (740 x 385 x 600; the
    search found nothing better): the passive panel's chord lands on the drive panel's b2
    and b3. Turned end for end (`cradle_pack.PACK_PLANS`), it packs in 740 x 337 x 600,
    149.4 L, under the test's 150.
  - A's drive panel end for end landed its gearbox on the passive panel's wider outer
    plate (107 L). Turned a quarter, it packs in 700 x 213 x 601, 89.6 L (89.3 before).
- **The BOM:** B USD 1,307.24 to 1,373.94 (+66.70): the inner plate +53.10 (its blank 361
  x 67 to 525 x 282 mm), the outer plate +10.16, the screws and washers +3.44, the
  standoffs 0. A USD 1,005.50 to 1,056.74 (+51.24): the outer plate +17.58, the inner plate
  +19.96, two standoffs +11.98 (12 to 14 at the 10-tier), the screws and washers +1.72. One
  column, screw and washer size: NETRF12 at the side's post length, M6 x 16 A2-70, DIN 125.
- Products only: the walkers' identity gate against `planner-f667d6b` is identical.
- **Where:** `spiderpig/construction/cradle.py` (`STANDS`, `rail_y`, `_bracket`,
  `CradleStand.realize`, `LAY_BLOCKS`, `_blocks_words`), `spiderpig/cradle_report.py`
  (`OPEN_AT_CAP`, `VERDICT`), `spiderpig/cradle_drive.py` (`verdict`),
  `spiderpig/cradle_pack.py` (`PACK_PLANS`), `tests/test_cradle.py`,
  `docs/agentlib/DECANTER.md` ("The side frames at the clutch's cap").

### 2026-10-20: cradle B's pack, turn 2: its brackets on the board, printed spacers in the foam's place, pin X first (packing review 1's L1, M1, M2 and minors)

- **What:** B's pack is now parts, not a convention, and smaller: 739 x 318 x 600 mm, 141.0 L
  (740 x 337 x 600, 149.4 L before; -5.6 %), every piece held to within 3 mm of its place
  (up to 357 mm before), the mass centre 41 % up. A packs as before, 700 x 213 x 601 mm,
  89.6 L, now held too ([DECANTER.md](DECANTER.md), "Flat pack").
  - **L1, the brackets on the board** (`cradle.BOARD_BRACKETS`, `_bracket`'s `board`):
    each outer plate's foot M5 is a stud (an M5 x 16 button head from the plate's inside, in
    the claim and the space the nut had; a jam nut on the outer face, threadlocked once),
    the bracket's upright slotted from it to its top (`STUD_SLOT`, 12 mm), and a printed
    knob on the stud (`STUD_KNOB`, an M5 nut pressed in) clamps the upright by hand. The
    foot is held by a plain M6 nut, snugged once at the build: the knob there is gone,
    and with it the four knobs and washers taken off and put back on the board's bolts at
    every trip (the review's m2). The passive panel, on top in the box, had stood 30 mm
    taller on its brackets' foot legs; its studs stand 11.2 mm: 141.1 L. The studs' knob
    had to sit between the upright and the foot's fastener, 5.5 mm thick, which the thumb
    knob's 24 mm left no room for: hence the plain nut (the review's "printed knob on the
    M5" with the upright slotted open at the top, otherwise as it asked). The stock 30 x 24
    x 12 in carton (Uline S-4417) then takes the pack's parts with 2.8 mm over them, not the
    review's 25: the side frames' redesign had added 21 mm since it measured.
    A tapped plate (a captive knob-bolt in the bracket, nothing proud of the plate) would
    have saved 8.5 mm more of height (about 3.8 L) but takes a tap the user hasn't or a
    tapping service the BOM doesn't price: not made. A keeps its brackets (on its board its
    panels couldn't lie flat: 100.7 L).
  - **M1, printed spacers** (`voxpack.spacers`, `cradle_pack.spacer_json`, `pack_kit`,
    `box_steps`): every pad becomes a support spacer and every piece that would still slide
    gets a restraint, each a block of the judged 3 mm grid's empty cells shaped to what it
    holds (lips round it), grown until it doesn't slide itself, numbered in the order the
    pack goes together. B has 14 (2.1 L of solid; about 640 g printed at 15 % gyroid, TPU
    95A on hand: no cost), A 25 (1.6 L, about 500 g). The build writes them
    (`print/pack_spacer_*.stl`) and the BOM lists them (`bom_from_mechanism`'s `extra`); the
    guide puts them in the box (B's "Into its box" and "Out of its box",
    `guide.make.with_box_steps`); `pack-inspect` judges each as a piece and reports them
    (`spacers`).
    The hand crank packs in the cradle box's bay (`PACK_PLANS`; m3), held there by one.
    No hold-down part: the carton's lid is 2.8 mm over the pack's top in the stock box and
    3 mm in a box cut to it, so nothing lifts more.
    - Why spacers shaped from the pack, not a few standard feet: the pads are where B's
      panels rest on edges, pins and gauges, 15-65 mm apart in height and none the same, and
      a foot must also stop the slide; a block shaped to both pieces does both, and the
      search can't make a layout needing a part nobody prints. TPU, not PLA: the spacers are
      also the pack's cushion (where foam had been assumed). They are 2.1 L because the
      drive panel stands 35-60 mm off the board and the box.
    - Why not a carrier's 2 in of cushioning (UPS's guide): 842 x 420 x 702 mm, 248 L,
      Oversized at USPS ($242.30 at zone 5, against $158.15 for the snug box): $84 a
      shipment more. The snug carton with the TPU spacers is the cheapest honest pack; for a
      carrier, insure it.
  - **M2, pin Y pushed out** (`cradle_report.keepers`' `second_tip_n`, `module_steps`): the
    cradle box's centre of mass lies 56 mm past pin X from Y and 105 mm off the line, so
    with X in its hole the box's weight turns it off Y's tip (-7 N): the review's 15 N on
    Y's tip doesn't arise in the order the step gives. The other order would put 42 N on
    X's tip, more than the box's weight. The step now says "pin X first, always" and why,
    and the way back starts by pressing each drive-side pin's tip with a thumb (about 3 kg):
    a keeper that has relaxed since the build's 30 N bench check shows before the box goes
    on, not under it. The passive panel's pins hang by their heads as it goes on (a tip
    that meets its link lifts its pin, never out). Round 2 had already made the bore 7.4
    (least grip 12.2 N, from 6.1) and held each piece's weight in the steps.
    Turning the drive side and box over the other way round (the box under) tips: 0.2°.
  - **m1:** the keeper's push on at the tight end (the bore printed 0.15 mm under, the pin
    at its 7.99 mm most, 0.74 mm) is 116 N, under the 120 N bar (`push_on_tight_n`,
    tested); the guide's "one that won't go on by hand, 0.1 mm looser" stands.
    **m2:** per trip only the four R-clips (back in their pins' holes for the trip, as
    before) and the four stud knobs (turned down onto their jam nuts) are handled; the
    board's knobs and washers are gone. No lanyards (the review's about $1 a clip, not in
    the catalog: the clips go back through their pins' holes). **m3:** the hand
    crank in the box's bay. **m4:** the passive panel rests on its outer plate's O1 head
    and its two studs, 0.9° tilted, tipping at 64.1° (it rocked on its brackets at 3.4°).
- **Not made, measured:** L2, the base stop off as a fifth piece: 702 x 333 x 600, 140.2 L
  (3 seeds of 60 s: 140.2-141.7), -0.6 %, for a fifth piece, 4 thumb screws and two steps
  each way. A's gearbox off its panel (its riser's four M6 nuts): 698 x 205 x 601, 86.0 L
  (3 seeds of 60 s: 86.0-93.4), -4 % (the review's -12 % was on round 1's A), for a loose
  2.1 kg piece and the worm's alignment to check again: not made.
- **Costs:** B's BOM USD 1,373.94 to 1,374.22 (+0.28: four M5 nuts, the studs' jam nuts; the
  knobs, brackets and spacers print in filament on hand); A's unchanged, USD 1,056.74. B's
  TPU at 100 % infill is 2.7 kg in the BOM's on-hand line (about 0.7 kg as printed). USPS
  Ground Advantage retail at zone 5: $160.20 to $158.15 a shipment (DIM 73 to 68 lb), in a
  box cut to the pack; $165.70 in the stock S-4417 (its 31 in length).
- **Tests:** `test_the_cradle_packs_in_one_box` (`MODULE_BOX` 340 mm and 150 L to 325 and
  143; `PACK_SLIDE` B 3 mm, A 6), `test_printed_spacers_hold_what_the_pads_held`,
  `test_the_guide_puts_the_pieces_and_their_spacers_in_the_box`,
  `test_a_board_bracket_takes_a_stud_and_a_knob`, the keepers' (`second_tip_n`,
  `push_on_tight_n`) and the steps' words.
- **Where:** `spiderpig/construction/cradle.py` (`BOARD_BRACKETS`, `STUD_SLOT`, `STUD_KNOB`,
  `_bracket`, `_stud_slot`, `_stud_knob`, `module_steps`, the first build's texts),
  `spiderpig/voxpack.py` (`Spacer`, `spacers`, `vox_mesh`), `spiderpig/cradle_pack.py`
  (`PACK_PLANS`, `layout_payload`, `pack_kit`, `box_steps`, `spacer_stl`, `in_order`),
  `spiderpig/cradle_report.py` (`keepers`), `spiderpig/build.py` (`_pack_spacers`),
  `spiderpig/hardware/bom.py` (`extra`), `spiderpig/mesh.py` (`write_stl`),
  `spiderpig/guide/make.py` (`with_box_steps`), `spiderpig/tools/pack_inspect.py`,
  `tests/test_cradle.py`, `docs/agentlib/DECANTER.md`, `docs/agentlib/TESTING.md`.

### 2026-10-20: the cradles' drive, round 3 (design review round 3's drive; the stand and use review's drive items)

- **What:** the fixes to round 3's drive review (`/root/scratch/decant/review5/r3/drive.md`:
  B1, M1, M2, m1-m9) and the stand and use review's drive items (`r3/stand_use.md`: M2, M4,
  m12), both cradles, B first ([DECANTER.md](DECANTER.md): "The drive", "The clutch's cap",
  "Building one").
- **B1, the guide's first setting locked the clutch** (`handcrank.SET_BACK`, `NEAR_STOP`,
  `plunger_setting`). "Turn each plunger in until its back stands flush with its jam nut's
  top" pushed each 1.98 mm; its nose lands on the seat ring at 1.352 mm (the 1.83 mm travel
  less the ball's 0.478 mm climb out of its hole), the ball still down: no clutch.
  - Now a felt stop: the arm at a click, each plunger in until it stops hard (its nose on
    the ring, wherever its nuts are), out half a turn (0.977 mm: a push of 0.375 mm, 32.2 N,
    0.98 mm of room for the climb), never nearer the stop than an eighth (0.24 mm of room,
    over the code's 0.2). A plunger whose ball bottoms first only stops sooner: the safe
    side. The plungers are drawn at that start; a test measures the drawn parts on B's and
    A's crank (the body's nose over the ring's face by its room, the room at the limit).
  - No hard stop was added: a printed lip in the plunger's bore would have to hang 0.4 mm
    under the boss's face, into its 0.6 mm gap over the ring, to stop the nose 0.2 mm short
    of the lock; not printable. The felt stop is the ring itself.
- **M1, the honest cap: 63.7 N·m on B, 67.1 on A, unchanged** (`config.PRODUCT_CLUTCH_NM`,
  `PRODUCT_JAM_NM`), now a bound on the clutch's setting, held by:
  - **a ceiling** (`handcrank.clutch_ceiling`): no pull of the check may read over the drawn
    6 L's window top, 30.5 N on B's grip, 30.8 on A's, whatever the start read. Set on a new
    box and run in after, 1.3 x had let 72.5 / 76.3 N·m through (the gearbox 1.17 / 1.11
    under its 85); under the ceiling it stays 63.7 / 67.1, and a new box's sealed 6 L still
    turns at 1.05 x its start at the window's bottom (`new_box_margin`). The start reading's
    angle and scale errors fall under it too.
  - **a check with no vise** (the stand and use review's m12; `handcrank.check_stub`, a PETG
    tool the build prints, `build._cradle_tools`): a 100 x 30 mm bar in the riser locator's
    pocket and a 14 mm keyed post, the hub on it; the bar turns under 2° and jams. Not a
    body of the model, so the flat pack doesn't place it yet (100 x 30 x 42 mm: the packing
    stream's).
  - **a schedule:** the last build step, every set-up (the trip's way back names it), after
    greasing, after a hot car, and every month or ten pours standing set up.
  - What it doesn't bound, reported (`cradle_drive.drive_limits`): spun at 200 rpm
    (`handcrank.HAND_FAST_RPM`, an empty cradle cranked back), the worm passes 67.9 / 71.5
    N·m (the least part 1.26 / 1.28 there, the gearbox 1.25 / 1.19): the use limits say turn
    it, don't spin it, about a turn a second at most. A hand at the grip's outer end, the
    journal greased, 2 % over the checked slip (dry 6 %; m6). Unchecked, the grease's drift
    takes it to 86.8 / 91.4 N·m: only the check bounds that.
  - Weighed and not built:
    - A run-in procedure: SEW's run-in is about 48 h under load; a drill on the input turns
      the worm unloaded, and the cradle offers no load path for it. The box stays "new" for
      a hand crank's life, which the ceiling now covers.
    - Rating everything at the higher cap (72.5 / 76.3): the side frames' least part 1.18 /
      1.20 and the gearbox 1.17 / 1.11 there; the structure stream reports SFs at it, but the
      ceiling makes it unnecessary.
    - A spring force limiter in the rod (round 2's M-4 cure, the one bound on every jam, the
      toggles' included): a preloaded double-acting cartridge at about 820 N (2 x B's walking
      rod force) in a rod that crosses O over the crank block: 20-25 mm deep against the rod's
      6.35 mm, so a re-plan of the stack, the structure's rod and the pack; two die springs, a
      guide and its ends, roughly USD 30-50 (unpriced). The user's call.
    - A clutch whose slip doesn't move with friction: a roller detent (a bearing rolling into
      a notch) would drift about 1.1 x where the balls drift 1.4 x (an estimate: the seat's
      sliding friction gone, the plunger's guide's left); a new clutch, its spring's
      tolerance unstated too. Not built. No ball-seat angle helps (20-45°: the drift within a
      few per cent of 1.4 x).
- **M2, the gearbox's riser is a clamped stack** (`handcrank.riser_for`, `RISER_STUD`,
  `RISER_RING`, `RISER_SLEEVE`, `RISER_SHIM`; rated by `cradle_drive.riser_rating`, walking in
  the report's `drive.riser` and at the cap a row of `jam_capped`):
  - Four class 8.8 M8 studs, cut 45 mm from one 200 mm zinc-plated DIN 976 rod (McMaster
    99067A279, USD 5.11, the listing rendered 2026-10-20), threaded in until they stop: an
    18-8 set screw's class is for a headless screw not loaded in tension (TR Fastenings).
  - A ring of the crank sheet on the face, centred on the 70 mm spigot ring (the side load
    into it through the spigot), out to the 100 mm boss, an 8.3 mm hole per stud (its web to
    the bore 3.2 mm, over the sheet's 3.175).
  - A 304 sleeve on each stud, 8 x 12 x 20 mm (Harfington p-1579935, five for USD 7.53, the
    product JSON fetched 2026-10-20): the stock size that clears the spigot (1.5 mm) and sits
    inside the boss (1.5 mm); its 8.0 bore snug on the stud.
  - The go/no-go's shims over the sleeves: 0.5 mm, 8 x 14 (Harfington p-1378778, twenty for
    USD 6.14, fetched 2026-10-20), about four a stud drawn, four DIN 125 washers as coarse
    ones: a millimetre either way. They replace the nuts under the plate and the printed
    setting sleeves.
  - The plate and a thin nut over it at 10 N·m (6.25-8.33 kN of clamp over a nut factor of
    0.15-0.2). The rating: four fixed-guided columns under the rigid housing, a quarter of the
    force and a share of the turn each, the moment as push-pull pairs, the stud's share of
    them 0.45 (its stiffness and the stack's). B / A walking: no slip at an oiled friction of
    0.15 by 2.92 / 2.72, every row 2.35 / 2.32 or more. At the cap 30° from the toggles: the
    stud (tension and, if it slips, shear) SF 2.33 / 2.29 on class 8.8's proof, the ring's
    bearing under a sleeve 1.73 / 1.63, A's columns' feet open a little (the sleeve then 2.28);
    it holds 1.3 outside 15° of a toggle. Oiled it may slip at the cap (0.96 / 0.83): the
    snug sleeve puts the side load into the stud in shear, which its row takes.
  - Round 2's studs at the cap: 314-383 MPa in bending, SF 0.55-0.62 as A2-50 (the review's).
  - Not made: the cheaper stack without the ring (-USD 12.97): the 14 mm shims then bear on
    the gearbox's die-cast face at 100-140 MPa at the cap, against an alloy whose limit
    neither Phidgets nor Motovario states.
- **Minors:**
  - m1: `END_SCREW_NM` 15 and `SCREW_NM` 4: TR Fastenings' maxima for ISO 7380 button heads in
    A2-70 at M10 and M6 (its socket screws' table). The torque wrench is 4-15 N·m
    (`cradle.TOOLS`). The structure's own ISO 7380 screws at 5 N·m are the structure
    stream's.
  - m2: the plunger nuts are pressed in from the boss's inner face, their pocket 0.15 mm over
    (`handcrank.NUT_POCKET`): its floor takes the springs' push and the jam nut's clamp.
  - m3: the clutch test holds every plunger force at the guide's limit (42.9 N), not the lock;
    a 10 % weak spring at the greased friction's low end can't reach A's setting (39.7 N
    needed, 38.6 there: `plunger_reach`), and the guide leaves the slip there, under the
    target, while it clears 1.1 x the start.
  - m4: the ring is replaced at any bright land you can see round a hole's edge
    (`SEAT_WEAR`, 0.1 mm), the balls on its laser entry face.
  - m5: A's crank stays 130 mm (`cradle_report.OPEN_GRIP`, settled): at the 22 N bar run in
    (21.97 N), over it new (24.98), as B's (21.7 / 24.7); 132.5 mm gives 21.55 N for 2.5 mm
    less hand room (-2.85 mm for a large hand), 125 mm 22.85 N.
  - m6: the journal is greased (the guide); its share of the slip reported
    (`handcrank.journal_share`).
  - m7: the use limits, the guide and the verdicts say "set and checked" and the ceiling.
  - m8: ORDER.md's Phidgets note adds the page's own example brokerage, USD 31.99: about USD
    172 landed before shipping, still not in the total (an account's checkout).
  - m9: each stud's end filed to 0.8 mm or less over its thin nut (`STUD_PAST_MAX`, held in
    `riser_for`): the crank block's lowest plate 2.2 mm over the four drawn, 0.59 mm with the
    go/no-go's window and the six plates under the seat at their thickest (1.45 drawn before);
    the guide's whole-turn check names the studs.
- **The stand and use review's M2, M4:**
  - The clutch's setting, the worm's hold and the toggle marks move to a last build step,
    "Before the first pour" (`construction.assembly.FIRST_POUR`,
    `handcrank.HandCrank.first_pour`), after the board's finish: the hand crank's step only
    builds it and sets its plungers' start. "Before you use it" and the trip's last step
    name it.
  - The toggles are found by the crank and the rod (at rest the arm folded under the rod in
    one line, at full tilt the two end to end), not the bottle; an arrow on the hub's end and
    the housing's rim painted red 2.5 turns either side of each toggle (a radius line had
    left the return to rest unmarked). Rule 3: hands out, and nothing added under the box,
    while the arrow is in a red arc; the decanter stays put. The cradle page's slider shows
    the three zones red and its readout says so (`cradle_report.motion`'s `toggle_turns`,
    `viewer/src/decant/motion.ts`' `toggleZones`).
- **A BOM fix:** the passive side had kept the drive's pack-bought lines (`cradle.DRIVE_WHERE`):
  the threadlocker had been counted twice (one bottle either way).
- **Costs** (the builds' BOMs): **B USD 1,374.22 to 1,401.53 (+27.31)**, **A USD 1,056.74 to 1,084.05 (+27.31)**:
  the studs' rod +5.11 for the set screws' 4.08, the sleeves +7.53, the shims +6.14, four
  coarse washers +0.32, the nuts under the plate -0.68: purchases +14.34; the ring's cutting
  in 6061 +12.97 on either; the check stub prints in PETG on hand. Harfington is a new cart on both (free shipping over USD 30, USD 3.99 under it; not in the totals).
- **Not changed:** the walkers (the identity gate against `planner-f667d6b`, in two chunks:
  identical); the frame's members (the structure stream's).
- **Where:** `spiderpig/construction/handcrank.py`, `spiderpig/cradle_drive.py`,
  `spiderpig/cradle_report.py`, `spiderpig/config.py`, `spiderpig/construction/cradle.py`,
  `spiderpig/construction/assembly.py`, `spiderpig/build.py`,
  `spiderpig/hardware/cradle_catalog.py`, `spiderpig/hardware/order.py`,
  `viewer/src/decant/`, `viewer/decant.html`, `tests/test_cradle.py`.

### 2026-10-20: design review round 3's use, bottle mounting and laser-cut parts' details (both cradles, B first)

The reviews: `/root/scratch/decant/review5/r3/stand_use.md` (M1, M3, M5 and its minors but
m12; M2, M4 and m12 are the drive stream's), `mounting.md` (all of it) and `lc_parts.md` beside it
(minors 1-8 and 10; B1, M1-M3 and minor 9 are the structure stream's). Every fix is a text,
a printed part's shape or a swapped part at the same price, but the food-grade grease.

**The stand and its use.**
- **M1: the way back backs each stud knob off two full turns before the cradle goes down**
  (`cradle.KNOB_BACK_TURNS`, `M5_PITCH`). Turned down onto its 4.0 mm jam nut for the trip a
  knob leaves less than the bracket's 5.0 mm upright, and "back it off a turn" came after
  lowering: the 12.9 kg cradle hung on four knob rims, 29 mm up and out of its cleats, every
  trip. Two turns stand it 1.6 mm off the nut (the upright 0.6 mm clear, the knob's nut still
  all on its 16 mm stud; three would run it off). The slots' mouths are chamfered 2 mm at 45°
  (`cradle.SLOT_LEAD`, m11: square, they left 1.4 mm to aim four studs by eye).
  `test_a_board_bracket_takes_a_stud_and_a_knob` holds the trip state and the chamfer.
- **M3: B's drive side gets its studs on the bench and drops into its brackets**
  (`cradle.studs_first`, `DROP_DRIVE`, `board_text`; the guide's own step, tag `studs_L`, before
  the base board's). Lying on the riser's blocks as the trip lays it, the jam nuts are reached
  under the plate; the brackets go on the board, their nuts run down to the washer; the side is
  lowered, each jam nut into its open slot, and the knobs hold it standing. Put on standing it
  had needed three hands (8.06 kg on a 4.75 mm edge, a hex key inside, a wrench outside) and a
  wrench inside the 12 mm slot.
- **M5: two people for the lifts, or one in an order that can be done**
  (`cradle.lay_down`, `LIFTS`: 12.9 kg the cradle, 10.3 the first build's drive side and box,
  held to `rest_poses`' masses): the gearbox's end down first, the front end held up while the
  blocks slide in, E1's (nearer the gearbox) then O1's, their middles about 460 mm apart. The
  NIOSH lifting equation's limit for that lift is 11.7 kg at most (the reviewer's index 1.10).
  The blocks self-centre: a 13.5 mm bore through each block's top under a 30° cone 26 mm across
  (`cradle.RISER_RELIEF`; m2: the 12 x 3.6 mm relief took neither the head's 12 mm washer nor
  its 4.9 mm, so the plate stood 1.3-1.6 mm off the boss). The rest poses are rated on the
  blocks' hulls, unchanged.
- **m1: pin X told by where it is** (`cradle.pin_words`: "pin X, in the long link b4, nearer the
  gearbox", from the linkage), painted beside its hole on each cradle link at the first build's
  lay, called out in the pictures (`Scene.marks` on pin X alone); the code name
  (`second_tip_n`) is gone from the user's words.
- **m3:** B's "Before you use it" wiggles its brackets (it has no thumb knobs) and its carriage
  bolts keep their brackets for good (`board_text`); DECANTER's rest-pose numbers re-measured.
- **m4: B's foot nuts are 18-8 M6 nylon-insert nuts** (`cradle.FOOT_NUT`: Bolt Depot 4795, USD
  0.11 as the plain 4776; a new SKU for four nuts, at the same price), snugged once to 1.5 N·m
  with the luggage scale on the 10 mm wrench (1.25 kN, 6.3 MPa on the PLA foot, half its
  sustained rating; a wrench's 2-5 N·m had been 8-21 MPa and nothing held the nut), and B's
  brackets print in PETG (on hand: they ride in the car).
- **m5:** 150 mm free behind the board and beside its drive side (`cradle.ROOM_FREE`): B's grip
  bolt stands 103 mm past the rear edge, A's grip 102 mm past the drive-side edge, a hand on it
  118 and 130 mm.
- **m6:** the card's rule 4 says the pad over the rim. **m7:** the way back restores the riser
  as stacked for the user's decanter (written on the locator's underside). **m8:** the passive
  panel's clips come out before it turns over. **m9:** it is set down on the towel.
- **m10: the grease is bought, food grade** (`cradle.GREASE`, the catalog's `grease_h1`: Super
  Lube 21030, NSF H1, 3 oz, USD 11.99 at Altex): the front saddle's box guide, its thumbscrew and
  the rear lead screw sit in the wine's path, and "any lithium or PTFE grease" had been an
  unbought shop supply (the user's on hand is filament and threadlocker). The same tube does the
  clutch and, wiped thin, the steel standoffs in place of light oil. Drop the line if a
  food-grade grease is on hand.
- **m13:** the cradle page lists the printed spacers among the pieces in the order they go in
  (`viewer/src/decant/pack.ts`'s `packOrder`, as `cradle_pack.in_order`), and the guide's
  decant step counts the turns the page does (`cradle.full_turns`: 14.6, it had said 15).
- **Laser-cut parts' minor 7: the router's tabs** are sanded flush before anything is glued,
  the cleats' faces first (SendCutSend's birch can't be detabbed; a 4.8 mm nub eats the rail's
  0.15 mm of play): `cradle.BOARD_TEXT`. **Minor 10:** DECANTER's costs are the build's.
- **Not mine here:** M2 (the clutch set before the box), M4 (the toggle cue) and m12 (the
  check stub) are the drive stream's. The page's toggle bands (m13's third) go with M4.

**Bottle mounting** (`/root/scratch/decant/review5/r3/mounting.md`; the reviewer traced ten real bottles from their makers' drawings and laid them in as the guide says).
- **M1: the front V bears on the neck only to 64.5 mm behind the mouth** (`cradle.FRONT_BEAR`;
  it bore to 79, and Saverglass's 4427 flares from 71). Behind it the block's V is relieved
  6 mm (`cradle.FRONT_RELIEF`, a 1 mm floor kept), its liner ends there (its pegs moved to
  mid-bearing), and the carrier's back wall is notched 6 mm deeper (`cradle.FRONT_WALL_RELIEF`).
  The guide measures the neck 62 mm from the lip (`cradle.NECK_AT`, the bearing's end less
  `cradle.STOP_GAP`; it said 80, where a steep Bordeaux is 2.4-2.9 mm wider in radius: set 3.9-5.1
  mm low, its shoulder on the bare back wall and the pad short of its rim). Chosen over the
  reviewer's text-only fix (measure at 76.5) because a shoulder inside the bearing makes the
  setting hang on where the caliper sits (6 mm of V per 2 mm of caliper on a 4427) and a slide
  onto the pad lifts the mouth 6 mm. The 750 ml and the impériale are now drawn from their
  makers' drawings, traced (`cradle.SAVERGLASS_4427`, `cradle.UNIVERRE_NGBD06QFR`,
  `BottleSize.outline`), so the clash test lays the real shoulders in at every crank angle;
  `cradle.bottle_room` checks each listed bottle's outline fast (a 4427 slid onto the pad: 1.4 mm
  to the relief, 3.0 to the back wall).
- **M2: the penlight moved forward from 118 to 100 mm** (`cradle.LED_X`): univerre's 6 L impériale
  (its shoulder r 41 at 115 mm, the model's 24.8) lay 7-10 mm into its head on the drawing; now
  10.7 mm clear, a 4427 7.4 (6.7 before), the methuselah's neck 21. The clip, still the lowest part
  over the decanter, rose 14 mm (394 mm over the table, 4° from rest; 380 at rest): the room grew,
  so `tests/fixtures/cradle/riser_reference.json` is re-recorded and several vessels stand higher
  (RIEDEL Cabernet Magnum 87 -> 96 mm, Ultra Magnum 114.2 -> 119.2; the RIEDEL Ultra keeps the
  whole set, now 45 mm under the clip). Not 45 mm aside: that keeps the clip's height but wants a
  clip offset over the decanter.
- **M3: the gate's top bar, ears and pointer stand 9 mm over the rails** (`cradle.GATE_TOP`; 6),
  the ears' roots filleted 4 mm (`cradle.EAR_FILLET`), and the gate prints on its top
  (`cradle.print_poses`; on its face the clamped ear's root bent across its layers):
  `cradle_report.saddles` rates the ear's root (6.1 MPa in its layers, SF 7.7, bar 4) and the
  plate hanging from its top bar (across them, SF 19). Not the reviewer's 12 mm: the M5 x 25 lock
  bolt runs 1.1 mm past its nut at 9 and would need an M5 x 30 at 12 (not stocked, unpriced).
- **m1: a wet bottle's slide onto the pad is rated** (`cradle_report.gauge_slide`). A 90° V's
  two faces press a bottle with √2 its weight, so dry TPU (over 1) never slides in ±30°; wet it
  can only under tan 30°/√2 = 0.41, and its friction falls to TPU's wet kinetic 0.30
  (`cradle_report.TPU_WET`: Yu et al., Polymers 17(1):30, 2024): 14 N drives it the guide's 4 mm
  into the gauge (a 25 N/mm spring, its bar), peaking at 70 N against the 54 N static pull. The
  gauge's rows are rated at the peak; its post is widened outboard from 13.7 to 21.7 mm
  (`cradle.GAUGE_POST_OUT`: SF 2.88 at the static pull before, 4.26 at the peak now, bar 2.5);
  the setting step wipes the bottle and the liners dry. Not the reviewer's TPU face with the gate
  1 mm behind (a solid 95A face is stiffer than the gauge and would press a bottle read 1 mm long
  with hundreds of N), nor a TPU leaf on the gate (it adds to the slide's drive).
- **m2, m4: the pointer 4 mm wide** (2), its inner edge where it was, its outer over the rail's
  outer face, printed in its layers with the gate on its top (a 2 x 6 mm column before: 3 N to
  snap); the vernier notched through its inner edge to the rail's top (lines in its top read
  across a 6 mm step: ±1 mm at 10° off square).
- **m3: the rear knob's hub's set screw bears on a flat filed on the lead screw's top**
  (`cradle.LEAD_FLAT`, 1 x 12 mm, a flat file: by friction on a thread crest, at the least cited
  PETG-on-steel friction at both faces, it rated 3.3 against the bar of 4); both M3 set screws
  threadlocked (on hand) and rated in `saddles` (the hub's 14.5 in bearing, the barrel nut's 9.8
  by friction across its printed thread's layers); the guide checks them, the knob down.
- **m5: "out of the sun, under 50 °C"** in the use limits, the print note and
  `cradle.PETG_PARTS`: the bottle's PETG parts bear up to 11 MPa as long as a bottle rests, and
  Eastman's Eastar 6763 deflects at 64 °C under 1.8 MPa ("holds its shape to 68 °C" was an
  unloaded part's). The 24 h sag test of a rail at 23 and 50 °C stays open.
- **m6:** the setting step tugs the gate again before the first turn when a bottle has rested;
  creep stays unrated (`cradle_report.SUSTAINED_SF` stands for it).
- **m7: the body measured 255 mm from the lip, where it lies on the rear V's front edge.** The
  reviewer's fix as worded changes nothing on a body that narrows from its shoulder (its widest
  from 255 on is at 255): univerre's 6 L (82.2 -> 79.3 mm) and 3 L lie tipped back on it, the neck
  3.2-3.9 mm over its V, the mouth 4.2-4.4 mm high (harmless: the pad covers the rim by 15 mm, the
  stream moves 2-3 mm). The guide's paper check under the neck finds it, and the rear V set for a
  body 5 mm wider brings the neck down (the mouth then 1.0 and 0.8 mm high).
- **m8: the methuselah's neck stays an estimate** (no maker's drawing found): `bottle_room` shows
  a shoulder as full as the impériale's clears the light by 10.7 mm. Open: measure one.
- **m9: the front V block's cheek 10 mm taller over the lip gauge's post** (`cradle.FRONT_CHEEK`,
  between the gauge's cheeks, 8.5 mm under its bridge at full travel), against the post's face:
  55 mm engaged, jam friction 0.69, 2.29 x the most cited for PETG on steel dry; `drawers` rates
  both guides dry (the front held only greased: 1.87 dry). Not the reviewer's taller -z wall: it
  meets the gauge's front cheek (from 20 mm up) at a 750's setting.
- **m10:** the print note: the 249 mm rails want a 250 mm bed (a Prusa MK4, not a 220 mm
  Ender-3).

**The laser-cut and routed parts** (`/root/scratch/decant/review5/r3/lc_parts.md`).
- **Minor 1: the crank block's plates are closed R10 at the neck** (`plates.CRANK_FILLET_R`,
  `plates.arm_plate`): two 55.6° cusps had sat at the arm's most bent section on all eight
  plates. What the closing adds is claimed inside the crank's sweep (`plates.arm_fills`, on its
  `fixed` claim). The keyway's web stays 1.52 t, a 6061 warning at 2.5 x SendCutSend's hard
  minimum: a hub 1.5 mm bigger (2 t) cost USD 1.20 on B. Merging files a and b (-USD 2.24) is
  not worth the filing.
- **Same minor: a cradle's cut-outs are read corner by corner** (`manufacture.inside_corners`)
  against SendCutSend's published laser radius (`sheet_catalog.SCS_LASER_CORNER_R`, 0.3 mm:
  "approximately the kerf width", its sheet-cutting guidelines fetched 2026-10-20; it had been
  0.8). The keyways' square ends had passed on the bore's arc (`min(arcs)`); they and the seat
  ring's hex are warnings now, harmless (a key's chamfers, a printed hex). Only a cradle
  (`mech.meta["cradle_linkage"]`): the walkers keep 0.8 mm and the arcs-only reading, so the gate
  is unmoved.
- **Minor 2: the clutch's seat ring** gets its clean (entry) face marked at arrival and its six
  seats broken alike, two turns of the deburring tool's countersink; it goes on that face out,
  to the balls (`assembly.CRADLE_ARRIVAL`; one phrase in the hand crank's clutch step).
- **Minor 3: SendCutSend's deburring minimum is its quoting app's 0.995 x 3 in**
  (`sheet_catalog.SCS_DEBURR_MM`, the spec API's "min_length"). b5 stays 24 mm and is deburred
  by hand (USD 0, no plan or pack change); the arrival step names the as-cut parts from the
  fabrication (`assembly.as_cut_words`: b4 and b5 today; b4 drops out once the structure
  stream's r 14 bosses land).
- **Minor 4: pin P in the 0.250 in rod:** each 8 mm pin is tried through its holes at arrival;
  a tight one (the rod's 8.2 mm hole can come out 7.95 at its exit face against a g6 pin's
  7.99) is opened from that face, sandpaper on a 7 mm drill bit's shank or an 8 mm H8 reamer.
  The hole stays 8.2 mm (8.3 would change every pin's play).
- **Minor 5: B's frame plates' members run station to station as the hull of their discs**
  (`cradle.TAPERED`) where a boss or a member running straight on stands at most 4.5 mm proud
  (`cradle.STEP_MAX`, `STEP_TURN` 30°), with a 1:10 taper into a plain strip (`TAPER_RUN`): the
  closing's S-shaped steps on its inner plate's D1-O2 leg are gone. A taper that would reach a
  keep-out keeps its step. It follows any member widths. A's plates keep their steps
  (cosmetic) until the pack's turn 3: tapered, they took the room A's flat pack's printed
  spacers stood in (one grew to a 327 mm print, another was left loose 66 mm:
  `test_the_cradle_packs_in_one_box`), and no pack plan changes in this round. Every convex
  corner on both is rounded 3 mm (`cradle.EDGE_R`: the board cut had left a sharp point at
  B's rear foot, on an R28 member's end).
- **Minor 6: labels.** The guide's parts table names the crank block's plates, the seat ring,
  the board, the locator and the cleats, and calls the birch CNC-routed (by the sheet's
  `process`); the arrival step and ORDER.md ask for a paint-pen label on every part (the keyed
  and clear crank plates differ by a 0.65 mm slot: the gearbox's key drops through a clear one).
- **Minor 8 (reported only):** the fit's worst files are b1 at 0.250 in (-12.5 % on B, -11.5 % on
  A), B's inner plate (-10.4 %) and A's outer plate (+11.5 %); the totals are within 0.3 %. The
  reviewer's 30 quotes could refit it.

**The clutch's cap, a tenth up.** The saddles' changes start a full 6 L 0.002 N·m harder at the
input (`handcrank.DESIGN_START_NM` 2.714 on B, 2.858 on A, the report's worst start), which
rounds the clutch's cap to 63.8 and 67.2 N·m (`config.PRODUCT_CLUTCH_NM`, `PRODUCT_JAM_NM`;
63.7 and 67.1 before). Every part still holds it at SF 1.3 or more.


**Costs** (the builds' BOMs): B USD 1,374.22 to 1,384.80 (+10.58: the grease +11.99; cutting
-1.41, the crank plates' necks -1.55 and B's inner plate's tapers +0.14; the nylon-insert foot
nuts at the plain ones' USD 0.11, a SKU more for four nuts); A USD 1,056.74 to 1,067.28 (+10.54:
the grease +11.99, the crank plates -1.45). Without the grease line, B -1.41 and A -1.45.
Purchases B 525.72, A 425.80; cutting B 859.08, A 641.48. Filament, threadlocker and the flat
file are on hand.

**Tests:** `test_a_board_bracket_takes_a_stud_and_a_knob` (the trip state, the chamfered
mouth, the nylon-insert nut), `test_the_trip_and_the_first_build_read_as_a_user_does_them`,
`test_the_base_board_is_rounded_offset_and_locates_its_riser` (the blocks' bore and cone),
`test_every_pose_the_guide_rests_a_piece_in_is_rated` (the lifts' masses), the mounting's
`bottle_room` and print-pose tests, the cut parts' outline, crank-plate and label tests, and the
viewer's `packOrder`.

**Where:** `spiderpig/construction/cradle.py` (the stand's, the saddles' and the frame plates'
code and texts), `spiderpig/construction/assembly.py` (the drive side's studs stage, the
arrival step), `spiderpig/construction/plates.py`, `spiderpig/construction/handcrank.py` (the
crank plates' closing, one clutch phrase, `DESIGN_START_NM`), `spiderpig/config.py`,
`spiderpig/cradle_report.py` (`saddles`, `gauge_slide`, `drawers`), `spiderpig/manufacture.py`,
`spiderpig/hardware/sheet_catalog.py`, `spiderpig/hardware/cradle_catalog.py`
(`m6_nylon_nut`, `grease_h1`), `spiderpig/hardware/bom.py`, `spiderpig/hardware/order.py`,
`spiderpig/labels.py`, `viewer/src/decant/pack.ts`, `viewer/src/decant/main.ts`,
`tests/test_cradle.py`, `docs/agentlib/DECANTER.md`.

### 2026-10-20: the cradles' structure model corrected and their frames refitted to it (design review round 3, linkage and structure, and the laser-cut parts' share)

Round 3's linkage review built an independent 3D frame of each side and found the report
rating a frame that wasn't the one built (B's pillar O1), on typical yields, first order,
with no buckling check, a ridge propping the solve, and the posts' screwed ends unrated. The
laser-cut parts review priced the cut side of the same findings. Products only: the walkers'
identity gate against `planner-f667d6b` is identical.

- **B1, pillar O1 a cantilever.** Round 2 ended O1 under b6 (a NETRF12-59.3, its inner plate
  uncut there), while `_side_posts` gave every pillar the plates' full gap: SF 0.39 at the
  cap as built, read 1.37.
  - A cradle pillar's printed neck may thin to 1.2 mm (`standoff.NECK_WALL`, three 0.4 mm
    perimeters: it only locates its links) where a link passes it: b6 leaves 7.54 mm round O1
    in layer 15, the rings' 1.5 mm wall needed 7.68. The plan is the same (B 16 layers), and
    O1 is a NETRF12 of the other posts' length into the inner plate's new hole (its boss the
    plate's 2 t). -USD 20.62 in standoffs.
  - The report takes each post's span from the parts (`cradle_report.post_spans`,
    `check_spans`: every post within `SPAN_TOL` of both plates' faces, a hole through each on
    its axis) and a pillar's note must say both plates hold it; else it stops.
    `test_every_post_the_side_frame_rates_stands_plate_to_plate`.
- **M1, typical yields.** A product is rated on each alloy's specified minimum in its gauge
  (`materials.allowable_mpa`, `sheet_catalog.MIN_YIELD`, ASTM B209-14 fetched today: 5052-H32
  23.0 ksi, 158.6 MPa, not 193; 6061-T6 35.0; 7075-T6 0.190 in 69.0). MIL-HDBK-5J's S-basis
  gives 5052-H32 22 ksi across the grain: B209's single figure is the bar, 4 % over it. The
  posts' steel is 235 MPa, the specified minimum of the EN 1.0038 MISUMI names for NETRF
  (`cradle_catalog.NETRF_STEEL`; its Americas catalogue says "1018 Carbon Steel or Equivalent",
  and 1018 has no specified minimum: 220, its hot-rolled estimate, had been taken). The
  walkers keep the typical values their rules and audits were set on.
- **M2, buckling; minor 1, P-Delta.** The plates' members' in-plane forces and the posts'
  axial ones enter as a geometric stiffness (`cradle_structure._geo4`, `side_frame_solve`):
  each angle solved second order, and the side's buckling factor per angle a row of its own
  (`frame:side buckling`). The old frames buckled at 1.35 x the cap (A) and 1.70 (B).
- **M4, the ridge.** `cradle_structure._solve`: Jacobi-scaled, one refinement, no ridge; a
  frame not held raises `SingularFrame`, every solve held to 1e-6 of its loads (a mode
  nothing holds nor loads, a lone post's inner plate turning about it, is fixed exactly).
- **Minor 2:** both idealizations of the plates (faces, and mid-planes with each post's end
  a rigid half-thickness off), the worse per row. **Minor 1:** each plate section's
  in-plane and out-of-plane stresses combined (`combined_sf`, `grillage_axial`). **Minor 3:**
  the tapped ends at the thread's roots (`NETRF12_ROOT` 6.0). A pillar's link load spread
  over its bore; a link's chord rated past its boss (or a wider chord's end disc there).
- **The frames on that model** (`cradle.STANDS`): the old ones fell to B's inner plate 0.86,
  outer 1.04, b6 1.12, O1 1.23; A's outer plate 0.39, column E1 0.47, inner plate 0.75.
  Weighed (`cradle_report.structure` with the stand varied, the posts then on 220 MPa):

  | option | B worst | A worst | cost |
  |---|---|---|---|
  | **wider members, 0.187 in 5052 (built); A a second rail column** | O1 1.37 | the outer plate 1.38 | the cutting's growth (BOM below), A's column USD 0 (from the 10-pack) |
  | the frames in 0.250 in 5052 | O1 1.29, b6 1.12 | column E1 1.02 | about +37 % on the frames' cutting; O1's boss then meets Tf2, and the drive's stack moves |
  | 6061-T6 frames and b6 | O1 1.23, b6 1.70 | buckling stays 1.35 (P-Delta) | +USD 170.62 on B (the laser-cut review's live quotes) |
  | a third rail column on B, the old widths | the inner plate 0.81, O1 1.19 (the rear column moves the inner plate's turn toward O1) | | a column more |

  - B: the rail 60 mm, O-Fr 80, O1-O 72, O2-O 88, O2-Fr 40, the inner chord D2-E1-E2 60,
    D1-D2 44, O1-D1 27 (as Tf2 leaves); b6's rear arm 48 (`config.PRODUCT_CHORD_R`).
  - A: a column E2 on the rail under the mouth, the inner chord along the rail to it 40 mm
    (60 cut into the riser locator by 0.5 mm: the verify test found it), the rail 60, G-O
    140, the legs from O 56, G-Ff 48, the triangles round G 40, C2-D1 34 (as Tf1 leaves),
    the inner legs 40 and 60.
  - The rails' lines stand 8 mm (B) and 10 (A) higher, their bottom edges where they were;
    the brackets' uprights follow (`rail_y`).
- **Laser-cut parts M3/M2:** b2 back to 0.187 in 5052 (its worst stress at the cap 36 MPa,
  SF 6.3 on the minimum; -USD 45.98 live); b3 and b4 bossed r 14 at their 8.2 mm holes
  (`PRODUCT_BOSS_R`: 2.05 t of 0.190 in 7075, SendCutSend's rule; b4 then 28 mm, deburred).
  b4 stays 7075.
- **M5, the posts' screwed ends** (`cradle_report.post_joints`): M6 A2-70 button heads at ISO
  7380's most, 4 N·m (TR Fastenings' socket screws table: `fasteners.BUTTON_A2_70_NM`; the
  posts' 6, the columns' none and the tie rods' 6 were drawn), the preload from TR's A2-70
  table at friction 0.1-0.2 (`fasteners.preload_band`: 2.5-4.6 kN). At the cap every face
  lifts; the screws hold A2-70's proof 1.95 x; under the bare 12 mm edges the 5052 would pass
  VDI 2230's limiting pressure (Table A9: 5052 isn't in it; AlMg4.5Mn's 325 MPa scaled to
  5052-H32's tensile, 269), so an 18 mm 18-8 fender washer (Bolt Depot 7315, in the order
  already) goes on both ends of every post (`cradle.FACE_WASHER`; a pillar's inner one in
  the clearance gap under the inner plate, where its printed gap ring had been): a pillar's
  inner end needed none, but with a washer at each end every post of a side is one length
  (`StandoffAxle.segment` takes a cradle post's gap to 0.01 mm; the columns' standoffs by the
  same rule), one part number at its ten's price: B's twelve NETRF12-71.8 USD 71.88 (10 of
  75.2 and 2 of 59.3 had been 92.50), A's sixteen 53.2. S_P 1.29 (B, column E2's foot) and
  1.61 (A). Walking every face shut at the least preload on B (0.99), none slipping;
  the guide's column steps and the use limits say 4 N·m, and back to it after a clutch slip.
- **M3, the blocked thresholds:** each angle pushed as far as its clutch lets it, not the
  least over the turn (`_block_threshold`, its structure with the block's tie rods). The rear
  saddle gives within the clutch: B's outer plate at 1.3 at +21.0 N at the grip, 1 at +37.5 N
  (the clutch passes up to 47.7 there); A's pillar G at 1.3 at +19.6 N. A 6 L's base: B's b6
  at +20.6 and +32.7 N, A's b2 at +14.3 and +19.8. DECANTER.md said "none within the
  clutch". The use limits' third ("nothing under the cradle box or the bottle") is the guard.
- **M6, the cleats:** glued against the rail itself (`cradle.CLEAT` 0.02 mm a side; a sheet of
  paper's 0.15 had let each side rock +-1.3°, +-9.7 mm at O1), the rock over the rail's 6.5 mm
  in them counted in the sway (`side_stiffness`'s `rock_mm`): B 1.22 mm elastic and 2.63 of
  rock under 20 N, 3.86; A 1.09 and 3.28, 4.37 (the test's bar 5).
- **Minor 4, Tf2:** under the cap O1's link moves 1.38 mm against the inner plate (0.39
  walking, on the walking bound), inside Tf2's 2.8 mm past the bar at rest.
- **Open items (the user's):** the tie rods' end joints are at their gap walking at the least
  preload (0.98, both cradles), and A's pillar G's foot lifts (1.06): ISO 7380's 4 N·m
  limits the preload (`cradle_report.OPEN_JOINTS`; a full-head screw at 5 N·m or a 10.9
  button head at 8 N·m holds each). The packs: the wider frames land B's drive panel on the
  cradle box (unsupported, 157.5 L) and stand A's panels 255 mm tall (`PACK_OPEN`): the
  packing round re-plans them.
- **The BOM** (`spiderpig build`, against the reviewed snapshot's): B USD 1,374.22 to
  1,342.12 (-32.10: the standoffs -20.62, 29 fender washers +2.88, the screws +0.16; the
  cutting -14.52, b2 out of 7075 -80.44 against the 5052's +65.92 for b2, the wider frames
  and b6's arm), 59 lines to 57. A USD 1,056.74 to 1,119.08 (+62.34: its column E2 a side,
  2 standoffs, 4 screws and 4 washers, 39 fender washers, +17.54; the cutting +44.80 for
  the wider frames and the inner plate's chord to E2). No new SKU.
- **The drive's numbers moved:** b6's wider arm took B's worst start to 2.717 N·m at the
  input (`handcrank.DESIGN_START_NM`, 2.712), its drawn clutch's cap to 63.9 N·m
  (`config.PRODUCT_CLUTCH_NM`, `PRODUCT_JAM_NM`). The structure holds it, and at the drive
  review's new-box 72.5 (B) and 76.3 (A) it falls to O1 1.18 and A's outer plate 1.16.
- **Where:** `spiderpig/cradle_structure.py`, `spiderpig/cradle_report.py`,
  `spiderpig/construction/cradle.py`, `spiderpig/construction/pivots/standoff.py`,
  `spiderpig/construction/axle.py`, `spiderpig/materials.py`, `spiderpig/strength.py`,
  `spiderpig/config.py`, `spiderpig/hardware/` (`sheet_catalog.py`, `cradle_catalog.py`,
  `fasteners.py`), `spiderpig/construction/handcrank.py` (`DESIGN_START_NM` only),
  `tests/test_cradle.py`, `docs/agentlib/DECANTER.md`.
