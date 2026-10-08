# The STS3215 bus sockets: the identity gate's diffs

The record of what the bus-socket change (the user's decisions of 2026-10-08, DECISIONS.md)
did to the gate's six designs, all on the STS3215, compared with the baseline
`~/.cache/spiderpig/gate/w8-2a130c8/`. The base branch (`quickfix`, 8de9de6) was identical to
that baseline on all six designs before the change. The new baseline is
`~/.cache/spiderpig/gate/bus-b8c9548/` (TESTING.md). The XL330 and XL430 aren't in the gate;
their centre sheet and stack are unchanged (`tests/test_robot.py`).

Where the part count changes the gate lists only the new names; the per-part comparison
below is of the parts both snapshots hold (the gate's own `deep_diff`, the same tolerance).

## The cause of every diff

1. **The servo's raised pad** (`servos.catalog`): `Relief(17.2, 30.0, ±9.3)` became the
   measured `Relief(17.2, 29.7, ±9.2, grow=0.2)`, so the parametric servo is smaller there.
2. **The centre plates**: 0.090 in x 4 became 0.063 in x 5 (`chassis.centre_sheet`: only
   0.063 in seats both rear screws), each with the bus window and channel
   (`chassis._port_slots`) instead of the old "end" slot (the SO model's header-pin relief
   kept, for the other socket: commit b8c9548, after the audit met it in the CAD servo).
3. **Both rear screws per servo**: `L/R.rear_screw1` added; the screws are M2 x 5 (were
   M2 x 6: one own plate of 0.063 in, 3.4 mm engagement, was 3.714).
4. **The stack is 1.144 mm thinner**, so each side moves 0.572 mm toward the mid-plane: every
   part of a side (the legs, cranks, frames, servo, ties, deck rails) shifts in z by +0.572
   (L) / -0.572 (R); nothing else about it changes.
5. **The rear frame ties move 2.75 mm** along the servo (`chassis.tie_locals`: 2 x t is
   now 3.2 mm, so the first clear place is 1 mm in from 31.31, not 3.75 mm), and the centre
   plates' outline round a tie is smaller (`tie_pad_r`: 2 x t): the inner frame plates
   (`L/R.torso`) get their tie holes at the new places, and the chassis top is 1.372 mm lower.
6. **The deck** sits on the chassis' top, so it comes down 1.372 mm (`deck_y` 21.68 -> 20.31,
   `rail_y` 13.68 -> 12.31, `top_y` 43.57 -> 42.2; the sweep gap 3.621 -> 2.249 mm) and is
   1.14 mm narrower (`plate_mm[1]` 71.14 -> 70.0: the bay between the inner plates); its path
   notches move 0.57 mm in. The charger, which no longer fits flush under the deck past the
   board's nuts, gets its two printed pad strips (`deck_charger_pad0/1`, 5.3 mm) on the three
   order designs and steps 0.57 mm in (`charger_z_mm`).
7. **The BOM**: the Y cable (`bus_y_cable_5264`, unpriced), M2 x 5 self-tapping screws (4,
   Amazon pack of 100, unpriced) instead of M2 x 6 (6, Accu, $2.76), the 0.063 in sheet
   instead of 0.090 in, the pad strips' filament.

## strider_double (and strider_quad: the same list, its own part counts)

| what | before | after |
|---|---|---|
| plan (layers, top, route, heads, height) | | identical |
| `plan/proof` (its counts only) | 371 nodes to the cheapest route; strider_quad: 11344 layouts met | 369; 11445 (the tie heads under the inner plate, a claim, moved: cause 5) |
| `audit/chassis/centre_plates` | 4 | 5 |
| `audit/chassis/rear_screws_per_servo` | 1 | 2 |
| `audit/chassis/rear_screw` / `rear_engagement_mm` | m2_self_tap_6 / 3.714 | m2_self_tap_5 / 3.4 |
| `audit/sheets`, `manufacture/sheets`, `kerf` | al5052_2p3mm | al5052_1p6mm |
| `audit/deck/*` | | cause 6 (`charger_pad_mm` 0 -> 5.3, `charger_z_mm` -34.57/-14.57 -> -34.0/-14.0, the notches' z ±0.57) |
| `audit/manufacture/by_rule/edge` | 2 | 4 (below) |
| `audit/manufacture/dxf` | centre_plate0..3 (0.090 in), 1640.22 / 1616.78 / 1616.78 / 1640.22 mm² | centre_plate0..4 (0.063 in, new contours), 1661.20 / 1611.69 / 1844.54 / 1611.69 / 1661.20 mm²; L/R.torso area 5183.75 -> 5174.86 mm², DXF deviation 0.0 -> 0.00198 mm (within 0.0102); deck_plate area 9311.70 -> 9156.12 mm² |
| `audit/warnings` | 3 (incl. "only 1 rear screw(s) per servo") | 2 (that one gone) |
| `audit/bom` | $347.65, 45 items, 10 unpriced, 9 unverified links | $346.73, 47 items, 12 unpriced, 10 unverified (strider_quad: $369.45 -> $368.53, 47 -> 49) |
| `audit/parts` | 387 | 392 (+ L/R.rear_screw1, centre_plate4, deck_charger_pad0/1); strider_quad 655 -> 660 |
| parts both hold | | 374 (quad: 642) shifted only (bbox, com: causes 4-6); L/R.servo, L/R.torso, deck_plate: volume and area too; L/R.rear_screw0: M2 x 5; centre_plate0..3: sheet, topology, volume |
| `bom.json` | | the purchases and made lists above; PLA 42.3 -> 43.5 g (quad 74.9 -> 76.1) |
| `bom.csv`, `bom.md`, `ORDER.md` | M2 x 6, Accu (3 lines, $10.40) | M2 x 5, Amazon; Accu (3 lines, $9.48); the Y cable line |
| DXFs | `SendCutSend_al5052_2p3mm/centre_plate0_x2`, `centre_plate1_x2`, its sheet | `SendCutSend_al5052_1p6mm/centre_plate0_x2`, `centre_plate1_x2`, `centre_plate2_x1`, its sheet; `L-torso_x2` 6 circles (the ties' holes) moved; the deck plate's 18 entities, the acrylic sheet's |
| STLs | | `print/deck_charger_pad0/1.stl` added; `deck_rail.stl`, `strider.stl` another mesh |

The cut-rule issues: `centre_plate0`/`4` 1.62 mm from the far rear hole to the pad's relief
(merged with the window and the channel, so open to the edge: the edge rule), was 2.58 mm
from the near hole to the old slot (centre_plate0/3); `centre_plate1`/`3` 2.91 mm from a
rear tie hole to the far head recess bridged into the pad's relief. Warnings (over 1 x t,
under 2 x t); errors 0.

## klann_lego_quad, klann_quad

The same causes and rows as the Strider: centre plates 4 -> 5 (0.063 in), rear screws 1 -> 2
(M2 x 5), the deck rows (klann_quad already had its charger pad strips: `deck_charger_pad1`
another mesh, `charger_z_mm` -33.07/-13.07 -> -32.5/-12.5, no `charger_pad_mm` change),
`by_rule/edge` 18 -> 20 / 10 -> 12, the warnings 15 -> 14 / 17 -> 16 (the one-rear-screw
warning gone), the BOM $339.61 -> $338.69 / $399.30 -> $398.38, parts 441 -> 446 / 517 -> 520,
the torso's tie holes (7 / 16 DXF entities), the plan identical (no proof change).

## hoecken_pantograph, dwell_rocker

Only the servo (cause 1): its volume 34877.84 -> 34862.49 mm³, area 7078.86 -> 7076.96 mm²,
its centre of mass by under 0.01 mm, at both angles, and the design's STL mesh. No chassis
(a one-sided mechanism): plan, BOM, DXFs identical.

## Beyond the gate

- The walk model's nominal deck (`spiderpig/walk.py`): the chassis' top 0.99 mm over the
  servo (was 2.36), the deck's centre 13.05 mm over it (was 13.45). The Klann quad walk
  reference (`tests/_linkage.py` `QUAD_REFERENCE`, which the viewer's model test reads too):
  the feet 0.572 mm nearer the mid-plane, the least margin 52.6 -> 52.0 mm.
- The demo Klann's ground clearance 72.23 -> 72.89 mm: the centre plates' outline (2 x t
  round the ties and recesses) shrank, so the crank's sweep is now its lowest part.
- The STS3215 audit (`mise run audit`, with the sim) on the Strider double: OK, cut-rule
  errors 0. The CAD servo's header pins met `centre_plate0`/`4` (12.6 mm³) until the SO
  model's pin relief was kept (b8c9548).
