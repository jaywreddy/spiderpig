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
