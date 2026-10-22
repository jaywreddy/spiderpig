# The planner's lowest stack: every gate diff and its source

What the planner of 2026-10-14 does to the gate's six designs against
`~/.cache/spiderpig/gate/next-320384d/` (the 2026-10-14 entry of
[DECISIONS.md](DECISIONS.md) has the why). The new baseline is
`~/.cache/spiderpig/gate/planner-f667d6b/` (TESTING.md; `planner-9da0548`, the planner's
before the merge of the cradles' design review round 1, differs from it only by the 15 mm
hex standoff's price, below). The planner's changes:

- the search's z is a bound of every plan (`stack.bound_layout`);
- the plan's z is lowered to the least it builds at (`stack.plan_z._descend`), and a
  standoff longer than its column is taken into a gap (`stack.plan_z._take_up`,
  `stack.Claim.hold`);
- both head placements are searched;
- the sleeve rule (`construction.pivots.common.sleeve_runs`).

The walkers plan on the objective "layers" (`config.plan_objective`): the fewest layers,
then the cheapest crank route. No design changed its layer count or its route's cost.

| design | layers | height mm | heads | gaps | printed parts | bought USD | cut USD | total USD | optimal |
|---|--:|--:|---|--:|--:|--:|--:|--:|---|
| strider_double | 14 | 68.86 -> 68.76 | gap | 9 | 173 -> 111 | 224.25 | 124.8 | 349.05 | True |
| strider_quad | 24 | 133.16 -> 132.46 | gap | 19 | 333 -> 207 | 243.69 | 183.07 | 426.76 | False |
| klann_lego_quad | 13 | 64.41 -> 57.81 | gap -> sink | 8 -> 7 | 207 -> 125 | 196.03 -> 191.08 | 220.11 -> 220.12 | 416.14 -> 411.2 | True |
| klann_quad | 13 | 72.99 -> 67.96 | gap | 10 -> 9 | 271 -> 143 | 289.04 -> 322.56 | 224.1 -> 224.13 | 513.14 -> 546.69 | True |
| hoecken_pantograph | 9 | 36.76 -> 36.36 | gap | 4 | 34 -> 15 | 73.99 -> 74.35 | 24.6 | 98.59 -> 98.95 | True |
| dwell_rocker | 7 | 30.96 -> 28.06 | gap -> sink | 4 -> 3 | 27 -> 15 | 57.59 -> 50.37 | 27.24 | 84.83 -> 77.61 | True |

(The cut cents: the plates' outlines at the new z, their DXF areas. The Hoecken
pantograph's USD 0.36 and part of the others': the 15 mm hex standoff, priced on
2026-10-14, unpriced before.)

## By source

- **The plan's z settles lower** (`finalize`'s start, `_descend`). After the settling, each
  gap is lowered to the thinnest the plan still builds at. Every design's stack is lower,
  with the same parts bought:
  - the Strider double's by 0.1 mm (its gap over layer 8, 3.0 to 2.9 mm);
  - the Strider quad's by 0.7 (the gaps over layers 6, 7 and 18);
  - the Hoecken pantograph's by 0.4 (the gap over layer 0, 2.9 to 2.5).
- **The search's z is a bound** (`stack.bound_layout`; the standoff pillar's rule, the
  crank's stock-standoff and stub tables): layerings the old nominal z refused are
  searched. On "layers" a size's search stops at its first plan with no route cost, so
  where the search meets another such plan first, the plan is another layering:
  - **the Klann quad**: 72.99 to 67.96 mm. One pillar is now a steel standoff made to length
    (`pillar_shaft_6_m3_32.2`, 2 at USD 14.97) where a goBILDA 4-pack served, with other
    barrel and standoff lengths: USD 33.52 more. It is the only diff that costs more: the
    height comes before the price in the objective. Its audit fails on strength as before
    (five joints under SF 1): the worst pin is better (jam SF 0.30 to 0.36, its span 39.9
    to 33.2 mm), and the cantilever pillar `B_leg1` is now among the five (jam SF 1.02 to
    0.84). Strength stays downstream of the planner (`strength.check`), as the objective
    has it. Its pillars' play falls from 0.725 to 0.1 mm.
  - **the `klann_lego` quad**: see the next source.
- **Both head placements** (the 2026-10-04 rule is gone). The `klann_lego` quad and the
  dwell rocker now plan with the pivots' heads sunk:
  - the `klann_lego` quad: 64.41 to 57.81 mm, USD 4.94 less. Every gap is at its heads'
    need, and its pillars' columns at that z are 0.7 mm over their 52 mm goBILDA
    standoffs: the take-up shims come in 0.5 mm steps (`StandoffAxle.shims`, stock 0.5 mm
    washers, `m4_washer_433`, USD 6.00 the 100 at Accu), so 0.2 mm stays as play (below).
    Its pillars' jam SF goes from 1.73 to 1.51 at worst (a warning before and after). Its
    crank's journal is a 15 mm hex standoff (`hex_standoff_m3_15`, 16 mm before), priced on
    2026-10-14 at USD 0.36 (`crank_catalog.WURTH_HEX_PRICES`: it was the line left
    unpriced at `planner-9da0548`, as on the Hoecken pantograph and the dwell rocker).
  - the dwell rocker: 30.96 to 28.06 mm, USD 7.22 less (its 15 mm hex standoff now priced). At the sunk plan's least z its
    24 mm goBILDA standoffs stood 0.2 mm proud of their 23.8 mm column; the plates stand
    on them, so that is taken into the gap over layer 2 (1.8 to 2.0 mm,
    `plan_z._take_up`): the height is the real one, and the pillars' play falls from 0.2
    to 0.1 mm.
- **The sleeve rule.** One printed sleeve per run of rings and gap rings between an axle's
  retained parts: the printed parts in the table, and with them the part files, the labels
  and the print lists.
- **Outside the gate.** The demo Klann single plans in 8 layers (10 before) and its double
  in 9 (10 before): their short standoff columns were refused at a pitch a layer, and fill
  at the plan's z. The single's 8 layers put its pin D across 14.8 mm (b4 in layer 1, b1 in
  4; 5.5 mm in neighbouring layers before): jam SF 0.79, walking 1.03 (1.57 and 2.05
  before), strength being downstream of the objective (`strength.check`). The XL330 Strider
  double plans 14 layers, 68.66 mm (68.86 before). The walk model's nominal mass is
  refitted to the lower stacks (`walk._PILLAR_PER_PIVOT_G` 12.0 to 7.0 g,
  `_PIN_PER_JOINT_G` 3.3 to 4.5 g: within 1.2 % of the built Strider, Klann and
  `klann_lego` robots, 2.4 % on `klann_lego` before), and the walk reference's Klann quad
  has its feet at the new plan's z.

## Known regression

- **The `klann_lego` quad's pillars keep 0.2 mm of play** (worst tilt 0.61 to 1.85
  degrees; `mise run audit` passes it, no warning): at its lowest plan every gap is at its
  heads' need, its pillars' columns stand 0.7 mm over their standoffs, and the take-up
  comes in 0.5 mm washers. The fixes, none taken yet:
  - finer take-up: DIN 988 shims or 0.3 mm washers beside the 0.5 mm ones
    (`StandoffAxle.shims`), one BOM line more;
  - take the 0.3 mm a second washer needs into a gap the columns cross (58.11 mm, the
    plan's z then what the parts stand at, as `plan_z._take_up` does for a standoff too
    long);
  - a play term in the objective after the height, so of equally low plans the one whose
    pillars fit wins.
