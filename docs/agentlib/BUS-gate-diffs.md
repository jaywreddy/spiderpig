# The STS3215 bus sockets: the identity gate's diffs

What the bus-socket change (the user's decisions of 2026-10-08 and the review round that
followed, DECISIONS.md) did to the gate's six designs, all on the STS3215, compared with the
baseline `~/.cache/spiderpig/gate/w8-2a130c8/`. The base branch (`quickfix`, 8de9de6) was
identical to that baseline on all six designs. The bus branch's own baseline was
`~/.cache/spiderpig/gate/bus-39502f7/`; merged with `t3code/next` (speed round 2 and the
assembly guide) the baseline is `~/.cache/spiderpig/gate/bus2-92d11e4/` (TESTING.md), which
differs from `bus-39502f7` only by the guide's labels (below). The XL330 and XL430 aren't in the gate;
their centre sheet, stack and rear screws are unchanged (`tests/test_robot.py`).

Where the part count changes the gate lists only the new names; the per-part comparison
below is of the parts both snapshots hold (the gate's own `deep_diff`, the same tolerance).

## The cause of every diff

1. **The servo's raised pad** (`servos.catalog`): `Relief(17.2, 30.0, ±9.3)` became the
   measured `Relief(17.2, 29.7, ±9.2, grow=0.2)`, so the parametric servo is smaller there.
2. **The centre plates**: 0.090 in x 4 became 0.063 in x 6 (`chassis.centre_sheet`: only
   0.063 in seats both rear screws; the stack holds one plug, its wires, a 1 mm margin and
   the other servo's pad: `chassis.centre_stack`). Each servo's window and its own channel
   (`chassis._port_slots`) replace the old "end" slot; the SO model's header-pin relief
   stays (`Relief.model_only`).
3. **Both rear screws per servo**: `L/R.rear_screw1` added; still the stock M2 x 6 (two own
   plates, 2.8 mm into the pilot; was one 0.090 in plate, 3.714 mm).
4. **The stack is 0.456 mm thicker** (9.6 against 9.144), so each side moves 0.228 mm away
   from the mid-plane: every part of a side (the legs, cranks, frames, servo, ties, deck
   rails) shifts in z; nothing else about it changes (its printed parts' STLs re-mesh, and
   a printed size in `print/parts.csv` can round the other way).
5. **The rear frame ties move 2.75 mm** along the servo (`chassis.tie_locals`: 2 x t is
   now 3.2 mm, so the first clear place is 1 mm in from 31.31, not 3.75 mm), and the centre
   plates' outline round a tie is smaller (`tie_pad_r`: 2 x t): the inner frame plates
   (`L/R.torso`) get their tie holes at the new places, and the chassis' top is 1.372 mm
   lower.
6. **The deck** sits on the chassis' top, so it comes down 1.372 mm (`deck_y` 21.68 -> 20.31,
   `rail_y` 13.68 -> 12.31, `top_y` 43.57 -> 42.2; the sweep gap 3.621 -> 2.249 mm, the
   nearest parts now the other side's mirror twins) and is 0.46 mm wider (`plate_mm[1]`
   71.14 -> 71.6: the bay between the inner plates); its notches and the charger move
   0.23 mm out (`charger_z_mm`).
7. **The BOM**: the Y cable (`bus_y_cable_5264`, unpriced: no product found) and two more
   M2 x 6 (8 at Accu, $3.68, was 6, $2.76): +$0.92 on each robot, one more unpriced line.
   The sheet changes (0.063 in for 0.090 in) aren't priced in the BOM.
8. **The strength check** gains the centre plates' row (`strength.centre_plate_row`): jam SF
   33.15 (klann_lego: 46.96), the screws' tear-out across the far hole's 1.62 mm web
   governing.

## strider_double (strider_quad: the same rows, its own counts)

| what | before | after |
|---|---|---|
| plan (layers, top, route, heads, height) | | identical |
| `plan/proof` (its counts only) | 371 nodes to the cheapest route; strider_quad: 11344 layouts met | 369; 11445 (the tie heads under the inner plate, a claim, moved: cause 5) |
| `audit/chassis` | centre_plates 4, 1 rear screw per servo, 3.714 mm | 6, 2, 2.8 mm |
| `audit/sheets`, `manufacture/sheets`, `kerf` | al5052_2p3mm | al5052_1p6mm |
| `audit/deck/*` | | cause 6 |
| `audit/manufacture/by_rule/web` | 8 (quad 14) | 10 (16): the far rear holes' web, below |
| `audit/manufacture/dxf` | centre_plate0..3 (0.090 in) | centre_plate0..5 (0.063 in, new contours); L/R.torso area 5183.75 -> 5174.86 mm², DXF deviation 0.0 -> 0.00198 mm (within 0.0102); deck_plate 9311.70 -> 9373.72 mm² |
| `audit/strength` | 15 rows | 16, `worst/chassis` 33.15 jammed |
| `audit/warnings` | 3 (incl. "only 1 rear screw(s) per servo") | 2 (that one gone) |
| `audit/bom` | $347.65, 45 items, 10 unpriced, 9 unverified links | $348.57, 46, 11, 10 (quad: $369.45 -> $370.37) |
| `audit/parts` | 387 (quad 655) | 391 (659): + L/R.rear_screw1, centre_plate4, centre_plate5 |
| parts both hold | | 376 (quad 644) shifted only (bbox, com: causes 4-6); L/R.servo, L/R.torso, deck_plate: volume and area too; centre_plate0..3: sheet, topology, volume |
| `bom.csv`, `bom.md`, `ORDER.md` | M2 x 6 x 6, Accu (3 lines, $10.40) | M2 x 6 x 8, Accu (3 lines, $11.32); the Y cable line |
| DXFs | `SendCutSend_al5052_2p3mm/centre_plate0_x2`, `centre_plate1_x2`, its sheet | `SendCutSend_al5052_1p6mm/centre_plate0_x2`, `centre_plate1_x1`, `centre_plate2_x2`, `centre_plate4_x1`, its sheet; `L-torso_x2` 6 circles (the ties' holes) moved; the deck plate's 18 entities, the acrylic sheet's |
| STLs | | another mesh: `strider.stl`, `deck_rail.stl` and a few crank collars and sleeves (cause 4) |

The cut-rule issues on the centre plates (warnings: over 1 x t, under 2 x t; errors 0): the
far rear hole 1.62 mm from the pad's relief, a `web` in the end plates (0, 5) and an `edge`
in the second plates (1, 4), where the relief merges with the other servo's channel, open to
the plates' edge. It was 2.58 mm from the near hole to the old slot (centre_plate0/3).

## klann_lego_quad, klann_quad

The same causes and rows: centre plates 4 -> 6 (0.063 in), rear screws 1 -> 2 (M2 x 6, 2.8
mm), the deck rows (klann_quad keeps its charger pad strip, `deck_charger_pad1` shifted,
`charger_z_mm` -33.07/-13.07 -> -33.3/-13.3), `by_rule/web` 10 -> 12 / 12 -> 14, the warnings
15 -> 14 / 17 -> 16 (the one-rear-screw warning gone), the BOM $339.61 -> $340.53 /
$399.30 -> $400.22, parts 441 -> 445 / 517 -> 521, the torso's tie holes, the plan identical
(no proof change), a few printed spacers and collars re-meshed (one klann_quad collar's
printed height in `print/parts.csv` rounds 0.7 -> 0.6 mm: its z shift, the same part).

## hoecken_pantograph, dwell_rocker

Only the servo (cause 1): its volume 34877.84 -> 34862.49 mm³, area 7078.86 -> 7076.96 mm²,
its centre of mass by under 0.01 mm, at both angles, and the design's STL mesh. No chassis
(a one-sided mechanism): plan, BOM, DXFs identical.

## The guide's labels (the merge of `t3code/next`)

Against `bus-39502f7`, the merged tree differs only in file names and label columns, on
every design: each DXF and print STL named by its label (`LK44.5x12b_L-b1_leg0_x6.dxf`,
`CP50x45_centre_plate0_x2.dxf`, `HS19.9-1.8_horn_spacer.stl`, ...), a `label` column in
`laser/parts/order.csv` and the sheets' `*_parts.csv`, the new names in `ORDER.md` and
`print/parts.csv`. Plans, parts, audits and BOM numbers are identical.

## Assembly changes (now in `chassis.assembly` and `construction.assembly`)

Ported into the structured order at the merge: `chassis.assembly`'s centre-plates ops (the
own plates from the meta's `rear_own_plates`, a middle-plates op tagged `plates_mid` in
`ROBOT_ORDER`) and `construction.assembly.WIRING_TEXT`; `docs/ARCHITECTURE.md`'s generated
order. Before the merge they were `construction/robot.py` `ASSEMBLY` steps 5 and 8 (the
default: 6 centre plates, 2 own plates per servo):

- **Step 5**: the studs into the left chains' ends; the left servo's own plates (0, 1) on its
  rear face over the studs, its two rear screws (the stock M2 x 6) through them; **its bus
  plug (one branch of the Y cable) pushed straight down through the plates' window into the
  socket on the servo's +y side, the wires bent toward +x and laid along its own channel**;
  then the middle plates (2, 3) over the studs and the wires. The right servo's own plates
  (5, 4) screwed to it the same way, its plug seated through their window into its own +y
  socket (the robot's other side), its wires along its own channel; then that servo and its
  plates onto the studs, rear faces together, no wire pinched.
- **Step 8**: no plugging (was: each plug "along the centre plates' slot from their far
  edge", impossible with top-entry sockets and narrower than a plug): each servo's wires from
  its channel's end at the plates' +x edge up through the deck's wire slot over it, the Y
  cable's trunk to the board; then the deck as before.

## Beyond the gate

- The walk model's nominal deck (`spiderpig/walk.py`): the chassis' top 0.99 mm over the
  servo (was 2.36), the deck's centre 13.15 mm over it (was 13.45), 118.2 g (was 117.7).
  The Klann quad walk
  reference (`tests/_linkage.py` `QUAD_REFERENCE`, which the viewer's model test reads too)
  follows the feet's shift.
- The demo Klann's ground clearance: the crank's sweep is now its lowest part (the centre
  plates' outline, 2 x t round the ties and recesses, shrank; it was the centre plates).
- The STS3215 audit (`mise run audit`, with the sim) on the Strider double: OK, cut-rule
  errors 0, the centre plates' jam SF 33.15.
