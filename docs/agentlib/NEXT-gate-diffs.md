# The integration branch: every gate diff and its source

The combined record of what the integration branch (the STS3215 bus window, the BOM
decisions and the assembly guide, merged in `bus-window`) does to the gate's six designs
against W8's baseline `~/.cache/spiderpig/gate/w8-2a130c8/`. The baseline is
`~/.cache/spiderpig/gate/next-320384d/` (TESTING.md). Every diff has one of three sources,
each with its own record:

- **bus**: the bus window ([BUS-gate-diffs.md](BUS-gate-diffs.md): its causes 1-8);
- **BOM A / B1-B4 / R1**: the BOM decisions ([BOM-gate-diffs.md](BOM-gate-diffs.md));
- **labels**: the assembly guide's self-describing file names and `label` columns (in both
  records: BUS "The guide's labels", BOM R1).

## How it was checked

The gate compared the merged tree with `w8-2a130c8` (`mise run gate -- compare`), and every
difference was attributed by machine against the two branches' own baselines (`bus2-92d11e4`,
the bus window with the guide; `bom2-5989d50`, the BOM decisions with the guide), each
against `w8-2a130c8`: a value that equals one branch's is that branch's; a value both
branches changed is "combined" (below); the BOM's rows and the files were matched by key,
name and path, not list position. **Nothing is unexplained**: every diff is one branch's,
or the two branches' changes to the same number added together.

| design | differences | bus | BOM | both, the same (labels) | both, combined | BOM rows and files matched by key |
|---|--:|--:|--:|--:|--:|--:|
| strider_double | 333 | 69 | 164 | 90 | 8 | 2 |
| strider_quad | 456 | 71 | 264 | 111 | 8 | 2 |
| klann_lego_quad | 618 | 80 | 14 | 88 | 4 | 432 |
| klann_quad | 658 | 73 | 18 | 119 | 5 | 443 |
| hoecken_pantograph | 154 | 11 | 95 | 48 | 0 | 0 |
| dwell_rocker | 218 | 0 | 115 | 49 | 54 | 0 |

Counts are by gate line. "Both, the same" are lines both branches changed identically (the
labels, the BOM schema both carry). "BOM rows and files matched by key" are the
`bom.json` purchased and made rows that moved in the list when the bus window's Y-cable
row was inserted (the gate compares lists by position). Matched by key, field by field,
every such field is the BOM branch's value or the bus window's. The exceptions are the
centre plates' `sheet` and the deck plate's DXF, listed under "Combined". Two are the bus
window's own later changes, made after its branch baseline `bus2-92d11e4` (both sections
below): the boxed bus cables (`servo_bus_cable_5264` for the Y cable) and the congruent
centre plates (the made row `centre_plate1` now names `centre_plate4`, its twin). dwell_rocker's
and hoecken_pantograph's "combined" lines are the gate's per-part lines, which group the
servo's pad change (bus) and the horn shims' 7 mm rings (B2) under one path.

## By source

**bus** (BUS-gate-diffs.md): the centre plates 0.090 in x 4 -> 0.063 in x 6 with the bus
windows and each servo's own channel; `L/R.rear_screw1` (both rear screws, the stock
M2 x 6, 2.8 mm engagement); each side 0.228 mm farther from the mid-plane (every side part's
z); the rear frame ties 2.75 mm along the servo and the torso's tie holes; the chassis' top
1.372 mm lower, so the deck lower and 0.46 mm wider; the servo's measured pad (the parametric
servo's volume, every design); the audit's `chassis` rows, the centre plates' strength row
(`worst/chassis`), the one-rear-screw warning gone; the BOM's bus-cable line (below).

**BOM A** (sourcing): every purchased row's vendor, SKU, pack and price, `cut_by`,
`on_hand` (the M2 x 6 the servo bags carry), the carts, the BOM total = ORDER.md's.
**B1** (the Strider's Chicago barrels 10/16 mm): the Strider double's and quad's plans
(66.46 -> 68.86 mm, 132.16 -> 133.16 mm), their pins, pillars, crankpins, spacers and rings,
the parts that come and go with them, the strength at the new spans. **B2** (the Tattu LiPo,
DIN 125 shim rings 7 mm, cup-point set screws): the battery and its cradle, every horn and
tie shim ring 6 -> 7 mm OD. **B3/B3b** (SendCutSend acrylic, kerf 0; the deck's cable-tie
web): the acrylic DXFs' folder and nominal size, the deck plate's slots. **B4** (captive deck
nuts): `deck_insert*` -> `deck_rail_deck_nut*`, the rails. **R1**: the cutting lines and
totals, the made rows' `sheet`, ORDER.md's HV warnings, the deck nuts 0.15 mm lower.

**labels**: every DXF and print STL named by its label, `label` columns in `order.csv`,
`*_parts.csv`, `parts.csv`, the names in ORDER.md (both branches, the same).

## Combined (both branches changed the same number)

- **The robots' BOM and part totals**: purchases are the BOM branch's exactly (the bus
  cables and the extra M2 x 6 are on hand), the cutting is the BOM branch's plus the
  bus's centre plates (6 x 0.063 in at SendCutSend, not 4 x 0.090 in):

  | design | w8 total | bom2 total | next: purchases + cutting = total |
  |---|--:|--:|---|
  | strider_double | $347.65 | $342.18 | $224.25 + $124.78 = $349.03 |
  | strider_quad | $369.45 | $419.89 | $243.69 + $183.05 = $426.74 |
  | klann_lego_quad | $339.61 | $409.13 | $196.03 + $220.09 = $416.12 |
  | klann_quad | $399.30 | $506.21 | $289.04 + $224.08 = $513.12 |

  The item count likewise (bom2's plus the on-hand bus-cable line).
- **Part counts**: bom2's plus the bus's four (`L/R.rear_screw1`, `centre_plate4/5`):
  strider_double 375 -> 379, strider_quad 627 -> 631, klann_lego_quad 441 -> 445,
  klann_quad 517 -> 521 (w8: 387, 655, 441, 517).
- **Made rows of the centre plates**: the `sheet` field (BOM R1's) holds the bus's
  `al5052_1p6mm`; the deck plate's DXF is `SendCutSend_acrylic_3mm/DK136x71.5_deck_plate_x1`
  (B3's folder, the bus's 0.46 mm wider deck in the label).
- **Printed grams**: B1's and B2's prints and the bus's deck change in one total.
- **hoecken_pantograph, dwell_rocker**: the servo (bus: the pad) and the horn shims (B2) in
  the same part lists; their totals are the BOM branch's (no chassis).

## The audit verdicts

As in both branches: OK on five designs; klann_quad's gate audit (no-sim family loads)
fails on the same five Chicago pins it failed on in `w8-2a130c8` (jam SF 0.30-0.43 at the
fallback 155 N; unchanged by either branch).

With the sim (`mise run audit`, not the gate's): the Strider double and the klann_lego quad
pass with 0 cut-rule errors. The Klann quad fails on strength, as it did on `t3code/next`
before the bus merge, and its cut rules have 0 errors. The bus window adds one finding
there. `pin:C_leg3`'s rated jam load goes 259.6 -> 318.7 N. That is not the jam itself:
the jam-only load is unchanged (117.1 N). `sim.loads.simulate_loads` takes as the jam load
the greater of the jam and the walking single-step peak. The walking peak went 259.6 ->
318.7 N, a dynamic contact spike, while its p99 barely moved (67.7 -> 69.6 N). So the SF
drop, 1.07 -> 0.87 (and `link:b1`'s 0.49 -> 0.40), comes from one sampled contact peak,
not from the chassis' geometry. The demo design's other six strength problems are
unchanged.


## The bus cable (`next-ba41c10` -> `next-ba68611`)

The first combined baseline, `next-ba41c10`, carried an unpriced "Servo bus Y cable" line
that took a cart of its own. No ready-made 5264 3-pin Y splitter was found, and none is
needed. The driver board has two bus servo ports: Waveshare's docs list "Bus Servo Control
interfaces", and its photo shows two 3-pin headers. Every STS3215's box holds its cable
(Seeed's C001 part list: "JST Wire x1"). So each servo's own cable goes to one board port:
`servo_bus_cable_5264`, x2, on hand. The gate against `next-ba41c10` changes only in the four
robots' BOM and ORDER text: the row (`bom.json`, `bom.csv`, `bom.md`); the `search` cart
gone from ORDER.md (the Strider double has 7 carts again); one fewer unpriced line and
unverified link in the audit's BOM summary. Totals, parts, plans and DXFs are identical.


## The congruent centre plates (`next-ba68611` -> `next-320384d`)

The final review found centre plates 1 and 4, and 2 and 3, were not mirror twins.
`chassis._merge_close` stretched whichever plug cut came first in its list. In plate 4 that
was the left channel, widened to y -0.6 along its length. The cut that grows least now
reaches into the other, so plate k and plate n-1-k are one part, turned a half turn about the
servo's axis (`tests/test_robot.py`). Against `next-ba68611`, on the four robots, only the
centre plates change:
- plates 2, 3 and 4: volume, area, faces;
- the made rows, 45 -> 44 on the Strider double (`centre_plate1` x2 now names
  `centre_plate4`);
- their DXFs: `CP57.5x54_centre_plate1_x2`, where there were `x1` plus `CP59x55.5_centre_plate4_x1`;
- the cutting estimate, +$0.02 (the Strider double: $349.03 -> $349.05).

Plans, other parts, cut-rule issues and strength are identical.
