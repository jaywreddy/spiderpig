# Joint strength — 2026-10-03 (the bolt crank and standoff pillars: 2026-10-04, real stock lengths)

What `spiderpig audit` (step 9, `spiderpig/strength.py`) says about whether the pins,
the pillars and the crank's joints hold, at each design's own loads.

## The model

* **Loads, per design** (`spiderpig/sim/loads.py`, cached per design in the store's
  `pin_loads/`): MuJoCo, the force every link puts on every pin, as vectors. *Walking*:
  3 s at full speed through the phase lock, the 99th percentile of each joint's most
  loaded link. *Jammed*: the base welded, one foot of the left side caught, the left
  drive at full command with its torque held to the servo's firmware limit
  (45 % of stall, 0.85 N·m on the STS3215), 24 crank angles x every foot x both
  directions, **the floor's contacts off** (since 2026-10-04, the cache's `VERSION` 3:
  with the base welded where it stood, a foot of the same side resting on the floor took
  part of the stalled torque as a second, unmodelled hold, so the caught leg's joints read
  low by however deep the other feet happened to sit; off, the caught foot is the only
  hold besides the weld, the case the check names; the stall bookkeeping is unchanged); a
  joint's jam load is the largest. The weld and the foot pin are as stiff
  as the model's loop equalities (`solref 0.002 1`, `solimp 0.99 0.999 0.0001`): with
  MuJoCo's default softness the pinned foot crept 11-41 mm and 32 of the Strider double's
  96 cases never stalled (the crank still turning at 5.3 rad/s), under-reading leg0's
  joints (J8_leg0 30.8 N against J8_leg1 49.0 N). Now every case stalls, the foot stays
  within 0.06 mm and the mirrored legs agree within 3 % (J8 40.6 / 40.8 N); the cache
  records the stalled fraction and each case's foot drift, and the loads note, the
  audit's warnings and a `RuntimeWarning` say so when a case does not stall. 12 angles
  under-read a joint by up to 16 % (J1); 24 are within 1 % of 48. `--pin-load WALK,JAM`
  overrides the sim; `--no-sim` (or no MuJoCo) falls back to the family's measured peaks
  (Strider 6.4 / 38 N, Klann 119 / 155 N, else the Klann's) and says so.
* **How far to trust the jam loads.** A leg with its foot pinned and the base held is one
  constraint over its single freedom, so how its load splits depends on where the give
  is, which the model only approximates (everything as stiff as the step allows). Two
  other holds, measured on the Strider double, bracket it: the base weld left soft (it
  gives ~4 mm) reads 1.4-1.8x higher (J6 77 N, J7_leg1 81 N, J2 72 N against 50 N, floor
  contact or not); the foot blocked along its path only (`sim.loads.JAM_MODES`
  `"path"`, statically determinate) reads ~1.2x higher (J6 59 N: pillar SF 1.75 -> 1.50,
  pin 3.55 -> 2.96), but near a foot's dead points, where it moves ~1 mm per crank
  radian, it reads up to 6x (the Klann single's pin C 2.15 -> 0.34), which a real joint's
  0.2 mm clearance would let the crank slip past. The table is the stiff pinned hold; a
  jam SF under ~1.8 has no margin left for those.
* **Bending case, per joint, from its link layers** (`construction/wobble.py`
  `beam`): a pin is held by nothing but its links, so the couple its loads leave is
  shared equally by the bores. Two links `s` apart: `M = F s / 2` (the old `F s / 4`
  assumed both ends clamped square; it halved every pin's bending). A clevis (middle
  link against the outer two): `F a b / s`. A printed pillar glued into one plate is a
  cantilever from the plate's face; into both, a beam between the faces. With the sim's
  vectors the measured patterns are used; with a bare load, the worst pair or clevis.
* **The crank**: each crankpin joint carries the drive torque x chord / crank radius
  (2.0 on the Strider double and the 180° quads, 1.41 on a decker, 1.0 single), rated
  element by element, the weakest deciding. Every hex in a plastic socket is **one bearing
  model** (`construction/crank.py` `hex_bearing_nm`): the flats' loaded halves fully
  plastic at 50 MPa (printed PLA in plane, cast acrylic; unverified for a given print or
  sheet), `0.75 p a^2 L` over the hex engaged, less a lead-in or corner relief, plus the
  clamp's friction where a clamp bears beside it. The **bolt crank** (the walkers' default
  since 2026-10-03): the M6 head's pocket (4 mm of 10 AF, the flats shortened by the 0.5 mm
  corner reliefs) 4.17 N·m, the nylock's pocket (6 mm) 6.26, the thread's torsion (stress
  diameter 4.92 mm, 8.8: 640 MPa) 8.64, the **nut's lock on its thread** 2.91 (the nylock's
  ISO 2320 prevailing torque, 0.4 N·m, plus a medium threadlocker's breakaway scaled from
  the TDS's 26 N·m on M10 by thread area x radius and halved for plated steel; the
  weakest, and an estimate: a test of one joint settles it), and the weakest
  solvent-welded plate interface the twist crosses (5 MPa over its overlap, unverified;
  8.4 N·m and more on the designs here). The **keyed crank**: the brass key's torsion
  (3.08) and the key's hex in its printed web socket and post cavity at the least
  engagement the float leaves (1.6 mm less the 0.4 mm lead-in), each with the clamp's
  0.17 N·m: **0.55 N·m**, SF 0.32 at the torque limit. Before 2026-10-03 the keyed crank was
  rated by the PLA post shell round the key's cavity (1.8 N·m), which the key's own
  sockets don't reach: that figure is gone. The printed crank: its clamp's friction (0.17).
* **Pillars**: a printed pillar glued into one plate is a cantilever from the plate's face,
  into both a beam between the faces. The **standoff pillar** (the default since
  2026-10-03, `construction/pivots/standoff.py`) is a 6 x 3.3 mm 6061 tube (240 MPa; the M4
  tap drill as if through) as a **beam per bay** between its supports, the frame plates
  (a pillar a link stops short of one plate: a cantilever). A splice (two stock segments
  butted on a ring by an M4 stud) is a joint, not a support: nothing ties the ring to the
  frame. Its capacity is the moment that starts to open it, the stud's preload (0.8 N·m:
  1 kN) x `(ro^2 + ri^2) / 4 ro` of the 6 / 4.3 mm annulus, 1.14 N·m, against the bay's
  moment at the splice (`wobble.moment_at_per_newton`), and the splice goes at the fewest
  ring layers where, under the check's unit load patterns on a beam between the column's
  ends, the worst splice sees the least moment.
* **Limits**: jam SF < 1 is an **error** (the audit fails, `verify` reports
  `strength` / `joint_overload` with the joint and its fixes as culprits); jam SF < 2
  or walking SF < 3 a **warning**. Each finding names the joint, its links, the case,
  the load and the SF, and fixes recomputed to clear it (another construction, the
  links in adjacent layers, a thicker printed pillar, a lower torque limit).

## Every walker x module (default constructions: chicago pins, standoff pillars, bolt crank)

`mise run remote -- ...` sweep of `spiderpig audit --linkage L --modules M`, 70 audits
10 at a time on ao-server in a fresh store (run `20261004-033639`,
`build/remote/20261004-033639/sweep`), the jam with the floor's contacts off, **with the
catalog's real stock only** (2026-10-04): goBILDA 1501 standoffs in the lengths goBILDA
sells (no 15, 21, 33, 39, 45, 51 or 57 mm: in 3 mm layers a column of those is spliced too)
and M6 ISO 4014 bolts from 30 mm (M6 x 25 is sold only fully threaded, DIN 933, so it is
gone, and a chain's run is at least 4 layers: the 30 mm bolt's 12 mm plain shank must end
above the nylock's complete thread). The run before, `20261004-011726`, assumed both and
is superseded. SF columns are `jam / walking`. "err" fails the audit.

| design | plans? | worst pin SF jam / walk (joint) | worst pillar SF jam / walk (joint) | crank SF jam / walk | warnings |
|---|---|---|---|---|---|
| fourbar_decker | yes (19 layers) | 6.31 / 2285.6 (K_leg0) | 5.04 / 1453.58 (H_leg0) | 2.42 / 130.1 | 0 err, 0 warn |
| fourbar_double | yes (13 layers) | 6.32 / 80.85 (K_leg1) | 7.95 / 113.96 (H_leg1) | 3.42 / 26.31 | 0 err, 0 warn |
| fourbar_quad | no | - | - | - | no plan within the budget (up to 30 layers, 1952 steps) |
| fourbar_single | yes (13 layers) | 6.33 / 2285.6 (K) | 8.83 / 2821.29 (H) | 3.42 / 293.64 | 0 err, 0 warn |
| fourbar_spot_micro_decker | yes (19 layers) | 5.46 / 2839.25 (K_leg0) | 4.37 / 1759.51 (H_leg0) | 2.42 / 133.48 | 0 err, 0 warn |
| fourbar_spot_micro_double | yes (13 layers) | 5.47 / 77.75 (K_leg1) | 6.88 / 109.8 (H_leg1) | 3.42 / 26.72 | 0 err, 0 warn |
| fourbar_spot_micro_quad | no | - | - | - | no plan within the budget (up to 30 layers, 2468 steps) |
| fourbar_spot_micro_single | yes (13 layers) | 5.48 / 2839.25 (K) | 7.64 / 3465.28 (H) | 3.42 / 299.69 | 0 err, 0 warn |
| fourbar_spot_micro_v2_decker | yes (19 layers) | 5.34 / 2229.85 (K_leg0) | 4.27 / 1428.52 (H_leg0) | 2.42 / 130.1 | 0 err, 0 warn |
| fourbar_spot_micro_v2_double | yes (13 layers) | 5.45 / 83.94 (K_leg0) | 6.87 / 118.98 (H_leg1) | 3.42 / 28.61 | 0 err, 0 warn |
| fourbar_spot_micro_v2_quad | no | - | - | - | no plan within the budget (up to 30 layers, 2120 steps) |
| fourbar_spot_micro_v2_single | yes (13 layers) | 5.47 / 2229.85 (K) | 7.62 / 2760.22 (H) | 3.42 / 296.63 | 0 err, 0 warn |
| jansen_decker | no | - | - | - | no plan within the budget (up to 41 layers, 60001 steps) |
| jansen_double | yes (15 layers) | 0.78 / 5.8 (C_leg1) | 0.84 / 6.41 (A_leg1) | 3.42 / 25.73 | 3 err, 2 warn |
| jansen_quad | no | - | - | - | no plan within the budget (up to 37 layers, 10390 steps) |
| jansen_single | yes (13 layers) | 1.04 / 22.84 (C) | 1.51 / 87.52 (A) | 3.42 / 37.13 | 0 err, 2 warn |
| klann_decker | yes (21 layers) | 0.51 / 26.51 (E_leg0) | 1.39 / 57.45 (B_leg0) | 1.22 / 14.02 | 2 err, 2 warn |
| klann_double | yes (13 layers) | 0.98 / 11.06 (D_leg1) | 2.07 / 20.66 (B_leg0) | 3.42 / 8.1 | 1 err, 1 warn |
| klann_high_step_decker | yes (19 layers) | 3.57 / 682.27 (D_leg1) | 3.15 / 456.04 (A_leg0) | 2.42 / 67.84 | 0 err, 0 warn |
| klann_high_step_double | yes (13 layers) | 3.56 / 38.81 (D_leg0) | 4.95 / 51.22 (A_leg1) | 3.42 / 18.33 | 0 err, 0 warn |
| klann_high_step_quad | yes (31 layers) | 3.58 / 21.27 (D_leg1) | 0.54 / 4.15 (A_leg1) | 1.71 / 4.12 | 4 err, 1 warn |
| klann_high_step_single | yes (13 layers) | 3.54 / 682.27 (D) | 5.53 / 985.49 (A) | 3.42 / 114.9 | 0 err, 0 warn |
| klann_lego_decker | yes (19 layers) | 3.24 / 1043.65 (D_leg0) | 2.64 / 835.69 (A_leg0) | 2.42 / 84.24 | 0 err, 0 warn |
| klann_lego_double | yes (13 layers) | 3.25 / 104.25 (D_leg0) | 4.16 / 132.33 (A_leg1) | 3.42 / 26.82 | 0 err, 0 warn |
| klann_lego_quad | yes (31 layers) | 3.24 / 38.6 (D_leg1) | 0.46 / 7.61 (A_leg1) | 1.71 / 6.23 | 3 err, 2 warn |
| klann_lego_quad --phases 0,0,180,180 | yes (20 layers) | 3.22 / 44.33 (D_leg0) | 1.44 / 17.41 (A_leg1) | 1.71 / 14.42 | 0 err, 3 warn |
| klann_lego_single | yes (13 layers) | 3.25 / 877.39 (D) | 4.61 / 1189.57 (A) | 3.42 / 146.08 | 0 err, 0 warn |
| klann_long_legs_decker | yes (19 layers) | 3.22 / 875.71 (C_leg0) | 2.58 / 684.88 (A_leg0) | 2.42 / 80.93 | 0 err, 0 warn |
| klann_long_legs_double | yes (13 layers) | 3.21 / 100.29 (D_leg0) | 4.06 / 127.18 (A_leg1) | 3.42 / 28.06 | 0 err, 0 warn |
| klann_long_legs_quad | yes (31 layers) | 3.23 / 28.81 (D_leg1) | 0.45 / 4.98 (A_leg1) | 1.71 / 5.36 | 4 err, 1 warn |
| klann_long_legs_single | yes (13 layers) | 3.2 / 822.16 (D) | 4.49 / 1062.68 (A) | 3.42 / 143.91 | 0 err, 0 warn |
| klann_patent_decker | yes (19 layers) | 2.98 / 837.21 (D_leg1) | 2.55 / 926.79 (A_leg0) | 2.42 / 76.41 | 0 err, 0 warn |
| klann_patent_double | yes (13 layers) | 2.99 / 92.2 (D_leg0) | 4.02 / 116.63 (A_leg1) | 3.42 / 21.5 | 0 err, 0 warn |
| klann_patent_quad | yes (31 layers) | 2.96 / 31.19 (D_leg0) | 0.44 / 5.54 (A_leg1) | 1.71 / 5.51 | 3 err, 2 warn |
| klann_patent_single | yes (13 layers) | 2.99 / 653.03 (D) | 4.45 / 898.04 (A) | 3.42 / 128.06 | 0 err, 0 warn |
| klann_quad | yes (31 layers) | 0.44 / 2.59 (E_leg0) | 0.28 / 1.41 (A_leg1) | 1.71 / 2.79 | 9 err, 3 warn |
| klann_quad --phases 0,0,180,180 | no | - | - | - | no plan within the budget (up to 41 layers, 60001 steps) |
| klann_single | yes (13 layers) | 1.21 / 71.02 (E) | 2.09 / 122.11 (B) | 3.42 / 89.72 | 0 err, 1 warn |
| sixbar_decker | yes (19 layers) | 1.97 / 103.81 (F_leg0) | 10.64 / 280.22 (H_leg0) | 2.42 / 62.1 | 0 err, 1 warn |
| sixbar_double | yes (15 layers) | 4.44 / 45.81 (K_leg0) | 11.58 / 174 (H_leg1) | 3.42 / 24.87 | 0 err, 0 warn |
| sixbar_quad | yes (31 layers) | 1.98 / 19.89 (F_leg0) | 1.58 / 14.17 (H_leg1) | 1.71 / 12.2 | 0 err, 3 warn |
| sixbar_single | yes (13 layers) | 4.45 / 144.6 (K) | 12.43 / 364.77 (H) | 3.42 / 136.48 | 0 err, 0 warn |
| sixbar_v1_decker | yes (19 layers) | 2.01 / 57.13 (F_leg0) | 9.88 / 114.87 (H_leg0) | 2.42 / 35.44 | 0 err, 0 warn |
| sixbar_v1_double | yes (15 layers) | 4.39 / 27.76 (K_leg0) | 10.76 / 144.5 (H_leg1) | 3.42 / 19.82 | 0 err, 0 warn |
| sixbar_v1_quad | yes (31 layers) | 2.01 / 18.98 (F_leg0) | 1.46 / 12.9 (H_leg1) | 1.71 / 11.48 | 0 err, 3 warn |
| sixbar_v1_single | yes (13 layers) | 4.41 / 75.18 (K) | 11.54 / 234.67 (H) | 3.42 / 68.24 | 0 err, 0 warn |
| sixbar_v2_decker | yes (19 layers) | 4.5 / 46.29 (F_leg1) | 13.21 / 50.07 (H_leg0) | 2.42 / 9.13 | 0 err, 0 warn |
| sixbar_v2_double | yes (15 layers) | 6.41 / 50.9 (F_leg1) | 16.67 / 94.22 (H_leg0) | 3.42 / 15.32 | 0 err, 0 warn |
| sixbar_v2_quad | yes (31 layers) | 3.53 / 29.39 (F_leg1) | 3.63 / 12.25 (H_leg1) | 1.71 / 6.51 | 0 err, 1 warn |
| sixbar_v2_single | yes (13 layers) | 4.55 / 77.49 (F) | 19.84 / 160.82 (H) | 3.42 / 29.66 | 0 err, 0 warn |
| sixbar_v3_decker | yes (19 layers) | 2.56 / 82.22 (F_leg0) | 15.01 / 600.06 (H_leg0) | 2.42 / 41.69 | 0 err, 0 warn |
| sixbar_v3_double | yes (15 layers) | 6.59 / 64.43 (K_leg0) | 13.23 / 125.66 (H_leg0) | 3.42 / 41.65 | 0 err, 0 warn |
| sixbar_v3_quad | yes (31 layers) | 2.56 / 17.44 (F_leg0) | 2.21 / 13.81 (H_leg1) | 1.71 / 11.68 | 0 err, 1 warn |
| sixbar_v3_single | yes (13 layers) | 4.64 / 373.6 (K) | 17.57 / 1586.1 (H) | 3.42 / 220.23 | 0 err, 0 warn |
| strider_decker | yes (24 layers) | 2.74 / 14.46 (J7_leg0) | 4.41 / 23.93 (J2_leg0) | 2.42 / 8.25 | 0 err, 0 warn |
| **strider_double (default)** | yes (24 layers) | 2.77 / 21.93 (J7_leg0) | 4.42 / 32.94 (J2_leg0) | 1.71 / 8.81 | 0 err, 1 warn |
| strider_quad | no | - | - | - | no plan within the budget (up to 41 layers, 30603 steps) |
| strider_single | yes (15 layers) | 2.76 / 20.13 (J7) | 11.81 / 70.63 (J2) | 3.42 / 19.83 | 0 err, 0 warn |
| trotbot_decker | yes (22 layers) | 6.51 / 45.13 (J5_leg0) | 7.14 / 34.13 (J3_leg0) | 2.42 / 10.3 | 0 err, 0 warn |
| trotbot_double | yes (16 layers) | 4.33 / 12.13 (J5_leg0) | 6.91 / 36.01 (J3_leg1) | 3.42 / 19.59 | 0 err, 0 warn |
| trotbot_heel_decker | yes (26 layers) | 2.94 / 18.92 (J11_leg1) | 7.87 / 97.58 (J3_leg0) | 2.42 / 7.85 | 0 err, 0 warn |
| trotbot_heel_double | yes (21 layers) | 2.31 / 13.68 (J11_leg1) | 11.69 / 48.39 (J3_leg1) | 3.42 / 16.91 | 0 err, 0 warn |
| trotbot_heel_quad | no | - | - | - | no plan within the budget (up to 37 layers, 10241 steps) |
| trotbot_heel_single | yes (16 layers) | 2.98 / 26.22 (J11) | 17.76 / 140.85 (J3) | 3.42 / 29.22 | 0 err, 0 warn |
| trotbot_quad | no | - | - | - | no plan within the budget (up to 37 layers, 9022 steps) |
| trotbot_single | yes (14 layers) | 6.43 / 112.38 (J5) | 11.94 / 153.45 (J3) | 3.42 / 32.63 | 0 err, 0 warn |
| trotbot_toe_decker | yes (26 layers) | 2.23 / 18.53 (J10_leg1) | 4.54 / 107.02 (J3_leg0) | 2.42 / 6.48 | 0 err, 0 warn |
| trotbot_toe_double | yes (21 layers) | 1.74 / 8.21 (J10_leg0) | 6.73 / 40.74 (J3_leg1) | 3.42 / 12.87 | 0 err, 1 warn |
| trotbot_toe_quad | no | - | - | - | no plan within the budget (up to 41 layers, 12287 steps) |
| trotbot_toe_single | yes (16 layers) | 2.26 / 27.29 (J10) | 14.36 / 175.14 (J3) | 3.42 / 28.78 | 0 err, 0 warn |

The same stock, the defaults before against the defaults now:

| design | crank, pillar | layers (mm a side) | parts | est. cost | pin SF jam | pillar SF jam (joint) | crank SF jam / walk | audit |
|---|---|---:|---:|---:|---|---|---|---|
| strider double | bolt, standoff (now) | 24 (72) | 344 | $230.12 | 2.77 | 4.42 (J2, its splice) | 1.71 / 8.81 | OK, 1 warning (the crank) |
| strider double | keyed, printed (before) | 18 (54) | 246 | $197.76 | 3.55 | 1.75 (J6) | 0.32 / 1.76 | FAIL: the crank |
| klann_lego quad 0,0,180,180 | bolt, standoff (now) | 20 (60) | 398 | $233.64 | 3.22 | 1.44 (A_leg1, its splice) | 1.71 / 14.42 | OK, 3 warnings (pillars A, the crank) |
| klann_lego quad 0,0,180,180 | keyed, printed (before) | 13 (39) | 242 | $206.09 | 3.21 | 0.96 (A_leg1) | 0.32 / 3.09 | FAIL: the crank, pillar A_leg1 |
| strider quad | keyed, standoff | 36 (108) | 534 | $221.06 | 3.68 | 1.12 (J6, its splice) | 0.32 / 6.18 | FAIL: the crank |

(The "before" rows are the first sweep's stacks, the keyed crank re-rated; the Strider quad
row is run `20261004-025644`: with the bolt crank it finds no plan, so its pillars were
gated with the keyed crank.)

Reading it. **The default Strider double passes**, unchanged by the stock fix (its 30, 35
and 40 mm bolts and its 54 + 12 mm spliced columns were real already): one warning, the
crank at jam SF 1.71 (its nut's lock, 2.91 N·m against 2 x 0.85); the standoff pillars J2
/ J6 at 4.42 / 4.81 (1.96 / 1.75 printed), each 66 mm column spliced at layer 19, whose
opening moment governs; the worst pin J7 at 2.77. `klann_lego` quad at 0,0,180,180, the
second gate design, **passes** with warnings: still 20 layers, but its 57 mm column (no
such standoff) is now 30 + a splice plate + 24 mm, and that splice's opening moment
(1.14 N·m) at ~137 N jammed reads 1.44 (it read 2.56 on the unspliced 57 mm segment
that doesn't exist); the printed pillar there failed at 0.96. The crank 1.71 as before.

**The crank** holds on every design that plans, never an error: 1.71 where two crankpins
sit 180° apart (factor 2: the Strider double, every quad; a warning), 2.42 on a decker
(1.41) and 3.42 elsewhere (1.0). The keyed crank, re-rated, fails everywhere (0.32 at
factor 2; a mechanism's, factor 1, 0.64), which is why a mechanism's audit now fails with
its default crank (the mechanisms keep `--crank keyed`: the bolt crank plans every
one-input mechanism but the Hoecken pantograph, whose pins sit too close for its 12 mm
pockets).

**The pillars**: 8 of the 60 designs that plan have an error (4 of 59 in the superseded
run). New since the stock fix: the Klann-variant quads at their default phases
(`klann_lego`, `klann_long_legs`, `klann_patent`, `klann_high_step`: pillar A 0.44-0.54,
B down to 0.77). Their 31-layer stacks (26 with the M6 x 25 bolt) make A's column about
87 mm, two stock standoffs spliced mid-span, and the splice's opening moment (1.14 N·m)
against ~137 N jammed governs (the tube itself holds about 1.5); printed pillars failed
there at 26 layers already (0.68-0.74). The fixes the audit names: a servo torque limit of
0.20-0.27 N·m, or the 0,0,180,180 phases (20 layers, 1.44). The others: the demo Klann's
pins and pillars (0.44 / 0.28 on its quad, 0.98 and 0.51 on the double and decker) and the
Jansen double (pins 0.78, pillars 0.84), as before. The six-bar quads, which plan now, hold
(pillars 1.46-3.63).

**What it costs: layers.** The bolt crank stacks a chain as tip + two-plate nut stack +
at least four run layers (a bare layer under the riders, the bolt's thread runout) + a
two-plate head stack, and its hub plates need the horn's face on a layer boundary (the
drive's printed horn spacer grows 2.8 mm): 24 layers on the Strider double (18 keyed), 31
on every Klann-family, four-bar and six-bar quad (16 keyed), 20 on `klann_lego` quad at
0,0,180,180 (13), 13 on a Klann or four-bar single (7), 19 on a decker. At 31 layers the
planner's quick pass, size by size, didn't reach a plan within its 60 CPU s on 9 quads;
since 2026-10-04 the crank's joint rules give the planner a lower bound
(`route.CrankRouter.min_top`: one chain per ridden crankpin, each at least 8 layers
between its outer webs, at most 2 shared by two, over the tip layer and up to the hub; 30
layers on a quad), so it starts there, and those quads plan (31 layers, not proven
thinnest within the budget). Ten designs find no plan within the budget: the Strider quad
(the keyed crank's 36 layers; the bolt crank's 42, at ~10 min of search and over the
planner's `max_top` of 40, so `mise run remote-audit` covers the Strider's single, double
and decker only), the three four-bar quads (31 layers with a budget ~30x the default), the
TrotBot quad and its heel and toe quads (39 layers at ~30x), the Jansen decker and quad,
and the demo Klann quad at 0,0,180,180. Every TrotBot heel and toe single, double and
decker plans (at its family's 10.5 mm unit; the keyed crank's 8.5 mm post didn't clear
b7). The parts count rises by about 100-150 a robot (rings in every pillar layer, the
crank's plates and hardware: +98 on the Strider double, +156 on `klann_lego` quad at
0,0,180,180) and the estimated cost by $27-32.

Against the first strength sweep (`20261003-182147`: the floor on during the jam, the
keyed crank, printed pillars) the floor's contacts mattered on some designs and not on
others: the Strider double's keyed and printed rows read the same (3.55 / 1.75), while on
`klann_lego` quad at 0,0,180,180 pin D's jam load fell (SF 2.62 -> 3.21, the pillars
unchanged): a resting foot had been loading the caught leg's pins through the floor.

## The PTFE tube liner pin (`--pin ptfe`)

(From the first sweep: the keyed crank, printed pillars, the floor on during the jam.)

A 3 mm rod in a 3 x 4 mm PTFE tube liner pressed in every link (`construction/pivots/
ptfe.py`), its limit the liner's 10 MPa over the `F / (d L)` bearing pressure.

| design | pin | layers | parts | est. cost | pin tilt worst / mean (free) deg | pin SF jam / walk |
|---|---|---:|---:|---:|---|---|
| strider double | chicago (default) | 18 | 246 | $197.76 | 0.72 / 0.39 (3.8) | 3.55 / 29.7 |
| strider double | ptfe | 18 | 288 | $219.09 | 0.68 / 0.61 (0.96) | 1.79 / 17.9 (warning) |
| strider double | rod | 18 | 232 | $216.59 | 0.68 / 0.61 (3.8) | 2.23 / 19.1 |
| klann_lego quad 0,0,180,180 | chicago (default) | 13 | 242 | $206.09 | 0.72 / 0.36 (3.8) | 2.62 / 55.3 |
| klann_lego quad 0,0,180,180 | ptfe | 13 | 266 | $230.08 | 0.59 / 0.59 (0.96) | 0.52 / 11.1 (error) |
| klann_lego quad 0,0,180,180 | rod | 13 | 218 | $227.58 | 0.59 / 0.59 (3.8) | 2.02 / 42.6 |
| klann quad 0,0,180,180 (demo) | chicago | 12 | 246 | $196.43 | 0.72 / 0.36 (3.8) | 0.80 / 16.9 (error) |
| klann quad 0,0,180,180 (demo) | rod | 12 | 222 | $216.59 | 0.68 / 0.60 (3.8) | 0.53 / 11.1 (error) |

Walking, the liner carries 0.6-0.9 MPa (SF 11-18, under PTFE's creep range). Jammed at
the torque limit it does not hold everywhere: 50 N on the Strider's 3 x 3 mm bore is
5.6 MPa (SF 1.79, a warning), and `klann_lego` quad at 0,0,180,180 jams 172 N into pin
D, 19.2 MPa against the liner's 10 (SF 0.52, an **error** on 13 pins; the first sweep's
soft foot pin had read 78 N and SF 1.15). The Chicago screw's barrel holds 3.55 and
2.62 there. What the liner buys is the free tilt, 0.96 deg against 3.8 (a 0.05 mm bore
clearance against 0.2), for $21-24 and 24-42 parts more than the Chicago screw.
**Chicago stays the default**; `--pin ptfe` is selectable where free tilt matters more
than jam margin, on a design whose pins jam under ~90 N (Strider-sized), best with the
torque limit lowered (0.45 N·m roughly halves the jam loads: SF ~3.4 on the Strider,
still under 1 on `klann_lego`).
