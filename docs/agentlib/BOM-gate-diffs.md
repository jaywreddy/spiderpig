# The BOM decisions: the identity gate's diffs, per change

The record of what the user's BOM decisions of 2026-10-08 (DECISIONS.md, "the BOM
decisions") did to the gate's six designs. Each change was its own commit; after each one
the gate compared the tree with the W8 baseline `~/.cache/spiderpig/gate/w8-2a130c8/` (the
base, `t3code/next` at c4c01d0, was **identical** to it on all six designs), and each
section below is that snapshot diffed against the previous change's (`gate diff`). The
union of the sections is the diff from `w8-2a130c8` to the new baseline
`~/.cache/spiderpig/gate/bom-bbf7001/` (TESTING.md; identical to the B4 snapshot).

Unless a section says otherwise, every plan, every part (volume, area, centre of mass,
box), every DXF and the audit's contract, clashes, solids, strength and wobble are
unchanged at the gate's tolerance; "BOM" means `bom.json`, `bom.csv`, `bom.md`, `ORDER.md`
and the audit's `bom` summary.

## A. Sourcing (the study's `sources.patch`, the BOM total, the servos' M2 screws)

Four commits: the patch, applied unchanged; `bom.cut_by` / `bought` (a sheet a service
cuts is in no purchase total); the servo boxes' M2 x 6 self-tappers on hand
(`bom.ON_HAND`); the crank router's reason (no output change).

| design | BOM total (bom.md) | ORDER.md purchases | carts |
|---|---:|---:|---:|
| strider_double | $347.65 -> $292.64 | $279.66 -> $292.64 | 16 -> 7 |
| strider_quad | $369.45 -> $315.80 | $301.46 -> $315.80 | 16 -> 8 |
| klann_lego_quad | $339.61 -> $220.12 | $207.62 -> $220.12 | 16 -> 9 |
| klann_quad | $399.30 -> $313.13 | $299.31 -> $313.13 | 17 -> 10 |
| hoecken_pantograph | $126.48 -> $66.00 | $76.49 -> $66.00 | 7 -> 8 |
| dwell_rocker | $115.45 -> $51.00 | $65.46 -> $51.00 | 7 -> 8 |

**BOM only, on every design**: each row's vendor, SKU, pack and price from the patch (Bolt
Depot sold singly for the M3 button heads, nuts and DIN 9021 washers; DigiKey for the Wurth
parts; the Amazon cart), `bom.json` rows gain `cut_by`, the M2 x 6 row is `on_hand`, and the
BOM's total is now ORDER.md's (the sheets' rows, Inventables' acrylic and SendCutSend's
aluminium estimates, are uploads). The audit's `unverified_links` grew from 9 to 20 (the
patch's Amazon offers from search results). Nothing else.

## B1. Chicago barrels per linkage (`chicago.BARRELS`)

(Keyed by linkage and crank in a later commit, the Strider's `bolt_round` crank keeping
every catalog length: no gate design changes, the final snapshot is identical to B4's.)

**klann_lego_quad, klann_quad, hoecken_pantograph, dwell_rocker: identical.**

**strider_double**: the plan keeps 14 layers (proven optimal) at new z: 66.46 -> 68.86 mm
(`plan/gaps`, `plan/describe`, `audit/plate_z`, `stack_mm`, the pin and pillar layers);
every pin on a 10 or 16 mm barrel (16 x 10 + 8 x 16; were 10, 15, 16, 18, 22, 7, 9);
the pillars' one-piece shafts 62.4 -> 64.8 mm; the hex crankpins 25 + 30 -> 30 mm x 4;
the journal stub's round standoff 12 -> 10 mm; 387 -> 375 parts (the printed spacers,
rings and collars the new z needs or drops); the link plates' bonded / running holes
follow the new lowest links (`laser`: L.b1_leg0 area 457.90 -> 457.57 mm^2 and its group
x6 -> x8); `audit/strength` and `wobble` at the new spans: pin jam SF 3.20 -> 5.75 (worst
J7 -> J3), pillar 6.69 -> 5.86, crank 2.46 (the gate's no-sim loads); BOM $292.64 ->
$248.34, 45 -> 39 items; `print` (the collars, rings and spacers), `strider.stl`.

**strider_quad**: 24 layers (unproven, as before), 132.16 -> 133.16 mm; every pin on 10 or
16 mm (32 + 16; were 8 lengths); pillar shafts 128.1 -> 129.1 mm; hex crankpins 16, 30 x 4;
655 -> 627 parts; pin jam SF 2.90 -> 5.75, the pillar's 1.81 jam warning unchanged (its
span 104.9 -> 89 mm); BOM $315.80 -> $267.78; `laser`, `print`, `strider.stl`.

## B2. One Tattu LiPo, DIN 125 washers and cup-point set screws (no Accu)

**hoecken_pantograph: identical** (no deck, no clamped M3 shims).

**strider_double, strider_quad, klann_lego_quad, klann_quad**: no plan, contract, clash,
strength or deck-clearance change. Parts: `deck_battery` (62.5 x 16.2 x 14.7, was 61.9 x
16.3 x 13.4), `deck_cradle` 65.7 -> 66.3 mm long with its outer end in place
(`deck.BATTERY_X1` -4.0 -> -3.4), its ears' screws and nuts 0.05 mm in z, the deck plate's
strap slots (the battery's mid-point, `laser`: deck_plate's 4 slot entities), every
`crank_horn_shims*` and `tie_shims*` ring 6 -> 7 mm OD (the DIN 125 washers they are bought
as, `bom.shim_od`; the horn screws' claim grows with them, and the plans are unchanged);
BOM: the LiPo row (Tattu, 1 x $9.29 for the Ovonic 4-pack $32.99), 48 DIN 125 washers at
Bolt Depot for the DIN 433 at Accu, the M3 x 16 set screws cup point at Bolt Depot: no Accu
cart (strider_double $248.34 -> $220.47, 7 -> 6 carts); `print/deck_cradle.stl`, the robot
STL.

**dwell_rocker**: its `crank_horn_shims*` rings 7 mm OD; BOM (the washers): $51.00 -> $49.60.

## B3. SendCutSend cuts the 3 mm acrylic

Every design: `acrylic_3mm` is SendCutSend's (`kerf_mm` 0, its acrylic rules); the acrylic
DXFs move from `laser/parts/Ponoko_acrylic_3mm/` to `laser/parts/SendCutSend_acrylic_3mm/`
and from `*_Ponoko_acrylic_3mm_*.dxf` sheets to `*_SendCutSend_acrylic_3mm_*.dxf` at nominal
size (no 0.2 mm kerf offset: holes 0.1 mm larger in radius, outlines 0.1 mm smaller);
ORDER.md's one SendCutSend upload carries every cut part (strider_double: 47 parts in 15
files, was 33 in 10 at Ponoko + 14 in 7); the audit's `manufacture/kerf` and `sheets`;
BOM (the sheet row's vendor, `cut_by`). Parts, plans, totals: unchanged.

Cut rules under SendCutSend's acrylic rules: **0 errors** on every design; one new warning
on the four deck designs, `deck_plate`'s 1.00 mm web between a wire slot and a cable tie's
slot (SendCutSend's least bridge is 1.35 mm), fixed in B3b.

## B3b. The deck plate's cable-tie web (`deck.CABLE_TIE_WEB`)

**hoecken_pantograph, dwell_rocker: identical.** The four deck designs: the deck plate's
four cable-tie slots 0.5 mm further from their wire slots (`laser`: 4 entities), the
protection board 0.5 mm along x (`deck_bms`, it keeps clear of the ties' run), the robot
STL; the cut-rule warning gone: **no cut-rule issue on any acrylic part**.

## B4. Captive nuts in the deck rails

**hoecken_pantograph, dwell_rocker: identical.** The four deck designs: `L/R.deck_insert0/1`
gone, `L/R.deck_rail_deck_nut0/1` (M3 nuts) new; the rails (`print/deck_rail.stl`, 5.9 ->
5.8 g each: the hex pockets and slots for the 4.0 mm insert holes); `audit/deck`'s
`insert_z` is `nut_z` (5.5 mm, unchanged); BOM: no heat-set inserts, 4 more M3 nuts
(strider_double $220.47 -> $216.26, 29 -> 28 lines); the robot STL. The deck screws stay
M3 x 8, the deck's way down (`deck_path`) and its clearances are unchanged.

A last commit cut the rails' pockets with the boolean operators the rail's other cuts use
(pyright's ratchet): the same solid (volume 4644.28 mm^3, one solid); the final snapshot
(the new baseline) was taken after it.
