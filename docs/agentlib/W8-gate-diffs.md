# W8: the identity gate's diffs, per change

The record of what W8's output changes (approved by the user on 2026-10-07; ROADMAP, W8) did
to the gate's six designs. Each change was its own commit, and after each one the gate was
compared with the baseline `~/.cache/spiderpig/gate/master-1eba876/` (the integration branch
at f11da3f was identical to it). Each section below lists only what that change moved
(diffed against the previous change's snapshot). The final diff against `master-1eba876`
is exactly the union of these sections. The new baseline is
`~/.cache/spiderpig/gate/w8-<sha>/` (TESTING.md).

Unless a section says otherwise, every part's volume, area, centre of mass and bounding
box is unchanged at 1e-9 relative (the gate's own tolerance), and so is every plan.

## 1. Bugs found by W4a (commit "W8 bugs")

`materials.aluminium_sheets` now loads the catalog, and `build.export_prints` labels a
filament-split print row by its own filament.

**Gate: identical on all six designs.** None of them has a print group whose parts take
two filaments. The two strict xfails pass, and their marks are removed.

## 2. D3, tie-stable rounding (`spiderpig.rounding`)

| design | file / value | before | after |
|---|---|---|---|
| klann_quad | `audit/manufacture/issues[9]` (R.b4_leg0, edge rule) value and detail | 3.93 mm | 3.92 mm |
| klann_lego_quad | `audit/manufacture/issues` (L.b1_leg0, R.b1_leg0, edge rule) value and detail | 3.28 mm | 3.27 mm |
| klann_lego_quad | `audit/manufacture/issues[2..9]` order | L/R.b1_leg1..3 before L/R.b1_leg0 | L.b1_leg0..3, R.b1_leg0..3 (the 3.27 mm issues now sort with their twins) |

Everything else is identical. The demo Klann quad's `audit --no-sim` is the same with
`SPIDERPIG_OCCT_THREADS=1` as with the default pool (`audit.json` and `audit.md` match
except for the run's seconds).

## 3. D5, mirror-identical printed parts grouped as "same"

`bom._proper_fit` tries a pure translation first. In each listed group the two parts go
from "print 1, and 1 mirrored" to "print 2" (`bom.json` `made[].mirrored` 1 -> 0). Their
`print/<group>_mirrored.stl` is no longer written, and their lines in `bom.csv`, `bom.md`,
`print/parts.csv` and `ORDER.md`'s print table change to match.

| design | groups (x2 each) |
|---|---|
| strider_double (6) | crank_pin_collar_hi_J1_leg0, crank_pin_collar_lo_J1_leg0, crank_pin_collar_lo_J1_leg1, crank_pin_sleeve_J1_leg0, crank_pin_sleeve_J1_leg1, servo_horn_spacer |
| strider_quad (12) | crank_pin_collar_hi_J1_leg0/1/2, crank_pin_collar_hi_journal10, crank_pin_collar_lo_J1_leg0/1/3, crank_pin_collar_lo_journal10, crank_pin_sleeve_J1_leg0/1/2/3 |
| klann_lego_quad (5) | crank_pin_collar_hi_M_leg0, crank_pin_collar_hi_journal5, crank_pin_collar_lo_M_leg2, crank_pin_sleeve_M_leg0, servo_horn_spacer |
| klann_quad (9) | crank_pin_collar_hi_M_leg0/2, crank_pin_collar_lo_M_leg1/2, crank_pin_sleeve_M_leg0/1/2/3, servo_horn_spacer |
| hoecken_pantograph, dwell_rocker | none (identical) |

There are no other changes: quantities, filament grams and parts are unchanged. The
build's `group` stage on the Strider double went from 6.7 to 4.4 s wall.

## 4. D4, `shapes.pill` as one extruded stadium

**Geometry check:** every part of the six designs at both `t` has the same volume, area,
centre of mass and bounding box as the baseline, within 1e-6 relative. They are also equal
at the gate's 1e-9, and the solid, face, edge and vertex counts and topology class are all
unchanged (4,270 part records). Audits and plans are unchanged. The 3.93 -> 3.92 of D3 was
already absorbed by the rounding, so nothing in any audit moves.

What moves is order and packing:

| design | verdict | changed files |
|---|---|---|
| hoecken_pantograph | geometry identical, order differs | DXF start vertices in both packed sheets, `parts/Ponoko_acrylic_3mm/b6_x1.dxf`, `parts/SendCutSend_al5052_2mm/torso_x1.dxf`; the robot STL's mesh |
| dwell_rocker | geometry identical, order differs | DXF start vertices in both packed sheets, `parts/.../b2_x1`, `b3_x1`, `b4_x1`, `frame_outer_x1`, `torso_x1`; the robot STL's mesh |
| klann_quad | packing reshuffled | `laser/klann_sheet_parts.csv` (which of the identical b3, b2 and b4 links sits in which slot, and some slots moved by 1 mm or rotated), so `klann_sheet_Ponoko_acrylic_3mm_0.dxf` (30 entities moved) and `klann_sheet_SendCutSend_al6061_3p2mm_0.dxf` (11 moved); start vertices in the other sheets and in 7 per-part DXFs; `klann.stl` mesh |
| klann_lego_quad | packing reshuffled | `laser/klann_lego_sheet_parts.csv` (L.b2_leg1 -> L.b2_leg0 ...), `klann_lego_sheet_Ponoko_acrylic_3mm_0.dxf` (30 moved), `klann_lego_sheet_SendCutSend_al5052_2mm_0.dxf` (9 moved); start vertices in 3 sheets and 2 per-part DXFs; `klann_lego.stl` mesh |
| strider_double | packing reshuffled | `laser/strider_sheet_parts.csv` (identical b4 and b8 links swap slots; b1 and b5 slots move), `strider_sheet_Ponoko_acrylic_3mm_0.dxf` (35 moved); start vertices in 2 sheets and 9 per-part DXFs; `strider.stl` mesh |
| strider_quad | packing reshuffled | `laser/strider_sheet_parts.csv` (R.b6_leg1 -> L.b6_leg3 ...), `strider_sheet_Ponoko_acrylic_3mm_0.dxf` (57 moved) and `_1.dxf` (37 moved); start vertices in 2 sheets and 13 per-part DXFs; `strider.stl` mesh |

The number of sheets per design and the entity count of each sheet are unchanged. The
packer sorts by the parts' outlines, so a part's new start vertex changes which of two
identical links it places first. That is why all four robot designs reshuffle, not only the
demo Klann quad as expected. Print STLs, the BOM and ORDER.md are unchanged.

## 5. D2, the cantilever standoff pillar

Of the six gate designs, only **klann_quad** has cantilever pillars: B_leg0 (free at its
lower end, its head in layer 6) and B_leg1 (free at its upper end, its head in layer 5),
on both sides. The other five are identical. The plan is unchanged: the new claim, the
washer disc in the gap between the last link and the free end's layer, fits the plan it
had.

| file / value | before | after |
|---|---|---|
| parts (both `t`) | 513 | 517: new `L/R.pillar_B_leg0_gap6_spacer`, `L/R.pillar_B_leg1_gap4_spacer` (printed PLA rings, 3.6 / 3.7 mm) |
| `audit/parts` | 513 | 517 |
| `audit/wobble/pillar` worst | 6.654 deg (pillar:B_leg1 b2_leg3) | 3.49 deg (pillar:B_leg1 b2_leg1) |
| `audit/wobble/pillar/mean_deg` | 2.623 | 1.884 |
| audit warnings | B_leg0 b2_leg0 5.4 deg, B_leg1 b2_leg3 6.7 deg | 2.6 deg, 3.5 deg |
| `bom.json` made[35] / made[36] (the gap-ring print groups they join) | qty 4 / 10 | 6 / 12 |
| PLA filament | 63.6 g (0.064 spool) | 65.1 g (0.065 spool); printed total 65.8 -> 67.2 g |
| `bom.csv`, `bom.md`, `ORDER.md`, `print/parts.csv` | | the same quantities and grams |
| `klann.stl` | | another mesh (the new rings) |

The audit has no new problem and no new clash. `check_side` stays `[]`, and the contract
passes.
