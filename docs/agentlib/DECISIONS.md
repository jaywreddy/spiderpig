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
