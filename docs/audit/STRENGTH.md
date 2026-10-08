# Joint strength — 2026-10-03 (the bolt crank and standoff pillars: 2026-10-04, real stock lengths; re-swept 2026-10-04 evening: thinnest sheets, single-plate crank, cut rules)

(A dated record. Since 2026-10-07 only the `bolt` / `bolt_round` cranks, the `chicago` pin
and the `standoff` pillar exist: the printed, keyed, rod, PTFE and spliced options below
were removed, `config.REMOVED_CONSTRUCTIONS`. Every layer count, stack height and safety
factor below is its sweep's own, as of the date in its heading: the current designs' numbers
are in [DESIGNS.md](../agentlib/DESIGNS.md), generated from the identity gate's snapshot, and a
current strength verdict is `mise run audit`'s.)

What `spiderpig audit` (step 9, `spiderpig/strength.py`) says about whether the pins,
the pillars and the crank's joints hold, at each design's own loads.

## The hex-standoff crank (2026-10-04, the walkers' and mechanisms' default)

The user's decision of 2026-10-04: every crankpin (and journal) of the single-plate bolt
crank is a stock M3 x 5.5 AF female-female steel hex standoff whose hex ends sit in hex
pockets of the aluminium webs; screws and DIN 9021 washers retain the plates, the riders turn
on a printed sleeve. One element carries the twist, the hex in each pocket, rated with the
one bearing model (`hex_bearing_nm`) in the plate's own depth, at the weaker of the plate's
yield and the standoff's (300 MPa steel), each flat 0.3 mm short for the standoff's rounded
corners (the pocket's dog-bone reliefs leave the flats whole but 0.09 mm at each end):

| crank sheet | hex engaged | holds | jam SF at 2 x 0.85 N·m |
|---|---|---|---|
| 0.063 in 5052 (the sheet before) | 1.6 mm | 1.91 N·m | 1.13 |
| 0.100 in 5052 | 2.24 mm (recessed 0.3) | 2.68 N·m | 1.58 |
| **0.100 in 6061-T6 (the default)** | 2.24-2.54 mm | 3.83-4.35 N·m | **2.25-2.56** |
| 0.125 in 5052 | 2.875-3.175 mm | 3.44-3.80 N·m | 2.02-2.24 |

The chain capped by the hub plate (no screw over it) is rated in the hub plate's pocket less
the crank body's float toward the outer plate (the stub's thrust sleeve's 0.1 mm play, the
assembly audit of 2026-10-04): 2.44 mm on the default, inside the table's 2.24-2.54.

The standoff's torsion (the tube in its flats round the M3 thread, 300 MPa) is 5.16 N·m.
0.125 in 5052 passes too, but is thicker than its 3 mm layer: every crank layer became
3.175 mm and the demo Klann quad's pillar columns stopped landing on stock standoff lengths
(no plan), so the crank takes 6061, the thinnest sheet that passes inside its layer
(`materials.ROLES["crank"]`, `ROLE_ALLOYS`). The crank on the default Strider double was a
warning at 1.71 (the round standoff's friction clamp, UNVERIFIED); with the hex it holds at
SF >= 2.25 on every design (factor 2 the worst), so it no longer warns. UNVERIFIED: the
fully plastic bearing at the sheet's yield (a pressed test of one pocket settles it).

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
  link against the outer two): `F a b / s`. A pillar anchored in one plate is a
  cantilever from the plate's face; in both, a beam between the faces. With the sim's
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
  The **single-plate bolt crank** (on the aluminium crank sheet, since 2026-10-04): each
  web clamped on its standoff crankpin's end by an M4 screw, a friction joint (`BoltCrank.
  _web_capacity`: 2200 N clamp, mu 0.3 at the web, 0.2 under the head; UNVERIFIED), 3.032
  N·m (`--crank bolt_round` since; the default hex-standoff crankpins rate a hex bearing:
  the first section).
* **Pillars**: (the printed pillar, removed in 2026-10, was a cantilever from the face of
  the plate it was glued into, or a beam between both faces.) The **standoff pillar** (the default since
  2026-10-03, `construction/pivots/standoff.py`) is a **beam per bay** between its
  supports, the frame plates (a pillar a link stops short of one plate: a cantilever), its
  section the standoff as a tube bored to its tap drill (as if tapped through): a goBILDA
  column a 6 x 3.3 mm 6061 tube (240 MPa), the one-piece MISUMI NETRF6 column the default
  makes of any column no stock length fills (since 2026-10-05, r5, below) a 6 x 2.5 mm 1018
  tube (220 MPa). The rest of this bullet is the spliced column (`--pillar standoff_hand` /
  `standoff_bench` / `standoff_m3`, the default until r5). A splice (two stock segments
  butted on a ring by an M4 stud) is a joint, not a support: nothing ties the ring to the
  frame. Its capacity is the moment that starts to open it, the stud's preload (0.8 N·m:
  1 kN) x `(ro^2 + ri^2) / 4 ro` of the 6 / 4.3 mm annulus, 1.14 N·m, against the bay's
  moment at the splice (`wobble.moment_at_per_newton`), and the splice goes at the fewest
  ring layers where, under the check's unit load patterns on a beam between the column's
  ends, the worst splice sees the least moment.
  **Since 2026-10-04 (the user's decision 2): the supported splice.** The splice plate is a
  stack of DIN 988 steel shims and the clamp is the two segments turned together on the
  stud to 1.0 N·m, each in soft-jaw pliers (1250 N: 1.42 N·m to open; finger tight, 0.4
  N·m, 0.57 N·m, failed the Strider quad's jam). Accepted UNVERIFIED: **to be tested on the
  first build** (torque a spliced pair to 1.0 N·m in soft jaws, check the running surface
  where a link turns is unmarked, load it in bending to the gapping moment). The sweep's
  tables below predate it (they rate the splices at 1.14 N·m).
  **Since 2026-10-05 (the user's decision): the hand-tight splice.** The splice is built in
  the normal bottom-up order, the upper segment turned onto the threadlocked stud (243 or
  263) by hand, and rated at 0.4 N·m (500 N: 0.57 N·m to open); the threadlocker retains
  it, it adds no rated clamp. The bench-built 1.0 N·m column stays selectable for long
  pillars (`--pillar standoff_bench`: its splices only under the pillar's lowest link, so
  the finished column takes its links over its top). A spliced pillar whose jam moment at
  the splice exceeds 0.57 N·m now reads under 1 (the Strider quad's 51 mm J2 was 1.71 at
  1.0 N·m: about 0.68 at 0.4); the Strider double's J6 had one, at layer 3 (2.31 at
  `k x pitch`, 1.43 at the plan's own z: the r5 table below). Since r5 the default pillar
  has no splice at all.
* **Limits**: jam SF < 1 is an **error** (the audit fails, `verify` reports
  `strength` / `joint_overload` with the joint and its fixes as culprits); jam SF < 2
  or walking SF < 3 a **warning**. Each finding names the joint, its links, the case,
  the load and the SF, and fixes recomputed to clear it (another construction, the
  links in adjacent layers, a pillar anchored in both plates, a thicker crank or link
  sheet, a lower torque limit).

## The plan's own z, one-piece pillars and the Strider quad, 2026-10-05 (r5)

The beams were at `k x pitch` (every layer 3 mm), which the clearance gaps of 2026-10-04 made
wrong: the Strider quad's 24 layers are 132 mm, not 72. Rated at the plan's own z (each link
at its layer's mid-plane, a pillar's supports at the plates' faces, a splice at its layer):

| design | joint | at `k x pitch` | at the plan's z |
|---|---|---|---|
| strider_double | pillar J6 (hand splice, layer 3) | 2.31 | 1.43 |
| strider_quad | pillar J6 (hand splices, layers 6, 15) | 0.89 | 0.51 |
| strider_quad | pin J7_leg2 (30 mm barrel, links 26 mm apart) | 3.7 | 1.8 |

No splice placement clears the quad: its column is 128 mm and goBILDA stops at 60, so a
splice falls between 28 and 100 mm from the outer plate, where even the bench-built 1.0 N·m
splice (1.42 N·m to open) reads under 2. So a column that would be spliced is **one piece**:
a MISUMI NETRF6 circular standoff (1018 steel, 6 mm 0/-0.1, M3 x 6 deep both ends, any
length in 0.1 mm steps, +-0.1; USD 11.12 each at 5-9, 15.32 at 1-4, rendered 2026-10-05),
rated as a 6 x 2.5 tube at 220 MPa (1018's hot-rolled minimum). The quad's J7_leg2: the
Strider's pins take barrels of at most 23 mm (a planner rule), which the same 24 layers meet
with J7_leg2 on 23 mm.

| design | layers / stack | pin | pillar | crank | link | pin / pillar tilt |
|---|---|---|---|---|---|---|
| strider_double (default) | 14 / 66.5 mm (unchanged) | 3.14 (J4_leg0) | 5.73 (J6) | 2.46 | 2.19 (b6) | 1.44 / 0.48 deg |
| strider_quad | 24 / 132.2 mm (unchanged) | 2.63 (J7_leg2) | 2.8 (J2) | 2.46 | 2.2 (b2) | 1.44 / 0.48 deg |

(Jam SF, MuJoCo loads; the pin tilts were 3.81 deg before the gap faces counted.)

## klann_lego and two mechanisms on the hex crank, 2026-10-05 (r4)

Each design debugged on its own, audited on ao-server (runs `20261004-235449`,
`20261004-235940`). `klann_lego`'s quad at 0,0,180,180.

| design | crank (sheet) | layers | pin SF | pillar SF | crank SF | link SF | cut rules | audit |
|---|---|---|---|---|---|---|---|---|
| klann_lego_single | hex (0.100 in 6061) | 8 | 3.27 / 86.2 | 7.62 / 199.48 | 4.91 / 86.64 | 1.6 / 52.59 (b4) | 0 err, 12 warn | OK |
| klann_lego_double | hex (0.100 in 6061) | 9 | 3.28 / 68.73 | 6.43 / 131.42 | 4.91 / 28.84 | 1.6 / 40.2 (b4) | 0 err, 16 warn | OK |
| klann_lego_decker | hex (0.100 in 6061) | 10 | 3.29 / 62.72 | 6.38 / 160.31 | 3.08 / 35.45 | 1.61 / 35.82 (b4) | 0 err, 18 warn | OK |
| klann_lego_quad | hex (0.100 in 6061) | 14 | 3.23 / 29.55 | 4.05 / 36.2 | 2.46 / 14.3 | 1.58 / 20.24 (b4) | 0 err, 28 warn | OK |
| hoecken_pantograph | hex (0.080 in 6061) | 9 | - | - | 3.89 | - | 0 err, 3 warn | OK |
| dwell_rocker | hex (0.100 in 6061) | 8 | - | - | 4.91 | - | 0 err, 3 warn | OK |

What was wrong and what fixed it:

* **klann_lego** (every module; it was on the round crank, `bolt_round`, which screws over
  the hub plate: no assembly order). On the hex crank its 6061 b1's 8.8 mm sleeve bore was
  2.02 mm from the link's **edge**, not from pin C (56 mm off, as the earlier note had it):
  `plates.rider_bosses` read the crank from the registry unresolved, and an unresolved
  `BoltCrank` reports the round 6 mm pin, so b1's end grew for a 6.3 mm bore (6.43 mm
  radius). Resolved for the crank sheet, the end grows to 7.7 mm round the 8.8 mm bore:
  3.27 mm of web, over 1 x t (a warning under 2 x t). That is the "wider end on a metal
  link" (a boss at the crank end only; the rest of b1 keeps the 6 mm half-width: its 4.2 mm
  pin holes have 3.9 mm, over 1 x t). The double then showed a 0.165 mm^3 clash between a
  printed hex collar and its washer: `BoltCrank.fit_hex` rounded the collar to 0.01 mm, up
  as often as down; it is floored now. b1 stays 0.125 in 6061 (jam SF 5.7+); the acrylic
  links pass (b2, b3); b4 (6061, the foot) is the weakest link, 1.6, a warning.
* **hoecken_pantograph** (the hex crank, a cut-rule error): its 12 mm crank puts crankpin
  M's hex pocket in the hub plate 2.16 mm from a horn screw hole (r 7 mm), under 1 x the
  0.100 in sheet. On 0.080 in 6061 (2.03 mm) that web is over 1 x t, and the hex pocket
  holds the drive's torque at SF 3.89. `config.LINKAGE_CRANK_SHEETS` gives it that sheet
  (the thinnest-sheet rule, at a mechanism's load); `--crank-sheet al6061_2p5mm` keeps the
  thicker one. The round standoff plans (10 layers) but has no assembly order; the keyed
  crank's key holds SF 0.64.
* **dwell_rocker**: nothing wrong on the defaults (the hex crank since the merge of
  2026-10-04: the horn spacer takes the short crank's head). Pinned by
  `tests/test_klann_lego_cranks.py`.

## Every walker x module, 2026-10-04 evening (thinnest sheets, single-plate crank)

(On the round standoff crank, before the hex-standoff crank merged: the crank column and the
plan heights here are the round one's; see the merged numbers at the top and in CLAUDE.md.)

A sweep of `spiderpig audit --linkage L --modules M`, 68 audits 6 at a time on
ao-server in a fresh store (run `20261004-160408`, `build/remote/20261004-160408/sweep`),
on branch `pw/rules`: the thinnest stock sheet per part (0.080 in 5052 frame plates, 0.063
in crank webs, 0.090 in centre plates), the **single-plate bolt crank** (every web one
aluminium plate, every crankpin a goBILDA 1501 round standoff clamped between its webs by an
M4 button head into each end, rated as a friction clamp: 3.032 N·m, UNVERIFIED
coefficients), heads in clearance gaps (`heads="gap_sink"`: TrotBot's heel and toe with the
pivots' heads sunk), the splices at a 1.0 N·m clamp, the glue-free joinery, `klann_lego`'s
b1 in 6061 (the user's decision 3), and the cut rules as errors and warnings. SF columns
are `jam / walking`; "cut rules" counts the parts (an error fails the audit: a hole under 1 x
the thickness from an edge in metal, or under the service's minimum hole); "audit" counts
the problems and warnings (one per rule or joint). The morning's table (below) is
superseded.

| design | plans? | worst pin SF jam / walk (joint) | worst pillar SF jam / walk (joint) | crank SF jam / walk | link SF jam / walk (link) | cut rules | audit |
|---|---|---|---|---|---|---|---|
| fourbar_decker | yes (10 layers, 40.7 mm) | 6.32 / 819.21 (pin:K_leg0) | 11.97 / 1491.66 (pillar:H_leg0) | 2.52 / 142.93 (crank) | 0.52 / 89.67 (link:b1) | 0 err, 14 warn | 1 err, 4 warn |
| fourbar_double | yes (9 layers, 33.2 mm) | 6.32 / 57.18 (pin:K_leg1) | 12.02 / 118.13 (pillar:H_leg1) | 3.57 / 25.08 (crank) | 0.52 / 5.33 (link:b1) | 0 err, 14 warn | 9 err, 3 warn |
| fourbar_quad | yes (14 layers, 63.8 mm) | 6.29 / 45.49 (pin:K_leg2) | 7.04 / 41.73 (pillar:H_leg0) | 1.78 / 8.97 (crank) | 0.52 / 5.08 (link:b1) | 0 err, 14 warn | 9 err, 10 warn |
| fourbar_single | yes (8 layers, 28.8 mm) | 6.33 / 1348.43 (pin:K) | 14.27 / 2763.84 (pillar:H) | 3.57 / 369.76 (crank) | 0.52 / 150.77 (link:b1) | 0 err, 14 warn | 1 err, 4 warn |
| fourbar_spot_micro_decker | yes (10 layers, 40.7 mm) | 5.48 / 1163.15 (pin:K_leg0) | 10.37 / 2554 (pillar:H_leg0) | 2.52 / 171.52 (crank) | 0.35 / 139.34 (link:b1) | 0 err, 14 warn | 1 err, 4 warn |
| fourbar_spot_micro_double | yes (9 layers, 33.2 mm) | 5.47 / 61.02 (pin:K_leg1) | 10.41 / 126.36 (pillar:H_leg1) | 3.57 / 25 (crank) | 0.35 / 5.93 (link:b1) | 0 err, 14 warn | 9 err, 3 warn |
| fourbar_spot_micro_quad | yes (14 layers, 64.9 mm) | 1.81 / 17.04 (pin:K_leg1) | 6.45 / 58.58 (pillar:H_leg1) | 1.78 / 6.42 (crank) | 0.34 / 3.58 (link:b1) | 0 err, 14 warn | 13 err, 9 warn |
| fourbar_spot_micro_single | yes (8 layers, 28.8 mm) | 5.49 / 1626.76 (pin:K) | 12.38 / 3241.87 (pillar:H) | 3.57 / 374.32 (crank) | 0.36 / 194.88 (link:b1) | 0 err, 14 warn | 1 err, 4 warn |
| fourbar_spot_micro_v2_decker | yes (10 layers, 40.7 mm) | 5.37 / 942.51 (pin:K_leg0) | 10.18 / 2185.4 (pillar:H_leg0) | 2.52 / 154.24 (crank) | 0.53 / 117.09 (link:b1) | 0 err, 14 warn | 1 err, 4 warn |
| fourbar_spot_micro_v2_double | yes (9 layers, 33.2 mm) | 5.49 / 67.83 (pin:K_leg0) | 10.49 / 139.37 (pillar:H_leg1) | 3.57 / 26.62 (crank) | 0.53 / 6.36 (link:b1) | 0 err, 14 warn | 9 err, 3 warn |
| fourbar_spot_micro_v2_quad | yes (14 layers, 63.8 mm) | 5.36 / 45.96 (pin:K_leg2) | 5.99 / 47.79 (pillar:H_leg0) | 1.78 / 10.88 (crank) | 0.53 / 5.66 (link:b1) | 0 err, 14 warn | 9 err, 10 warn |
| fourbar_spot_micro_v2_single | yes (8 layers, 28.8 mm) | 5.42 / 1015.82 (pin:K) | 12.23 / 2134.4 (pillar:H) | 3.57 / 329.57 (crank) | 0.53 / 126.2 (link:b1) | 0 err, 14 warn | 1 err, 4 warn |
| jansen_decker | no | - | - | - | - | - | no plan within the budget |
| jansen_double | yes (11 layers, 42.1 mm) | 0.78 / 5.17 (pin:C_leg1) | 1.35 / 9.58 (pillar:A_leg1) | 3.57 / 23.29 (crank) | 0.24 / 0.81 (link:b6) | 0 err, 14 warn | 11 err, 14 warn |
| jansen_quad | no | - | - | - | - | - | no plan within the budget |
| jansen_single | yes (10 layers, 38.8 mm) | 1.05 / 16.98 (pin:C) | 1.44 / 54.29 (pillar:A) | 3.57 / 32.29 (crank) | 0.24 / 4.22 (link:b6) | 0 err, 14 warn | 6 err, 10 warn |
| klann_decker | yes (12 layers, 54.1 mm) | 0.57 / 14.99 (pin:D_leg1) | 2.21 / 67.98 (pillar:B_leg0) | 2.52 / 17.85 (crank) | 0.47 / 14.24 (link:b4) | 0 err, 18 warn | 3 err, 13 warn |
| klann_double | yes (11 layers, 42.6 mm) | 0.72 / 7.29 (pin:D_leg0) | 2.52 / 16.23 (pillar:B_leg0) | 3.57 / 6.83 (crank) | 0.55 / 3.06 (link:b4) | 0 err, 16 warn | 4 err, 8 warn |
| klann_high_step_decker | yes (10 layers, 41.4 mm) | 3.53 / 79.64 (pin:D_leg1) | 7.5 / 226.65 (pillar:A_leg0) | 2.52 / 39.92 (crank) | 1.07 / 20.7 (link:b1) | 0 err, 18 warn | 0 err, 7 warn |
| klann_high_step_double | yes (9 layers, 34.0 mm) | 3.53 / 25.3 (pin:D_leg1) | 7.54 / 51.94 (pillar:A_leg1) | 3.57 / 14.6 (crank) | 1.07 / 7.66 (link:b1) | 0 err, 16 warn | 0 err, 10 warn |
| klann_high_step_quad | yes (14 layers, 58.5 mm) | 2.16 / 28.66 (pin:D_leg2) | 4.74 / 32.58 (pillar:A_leg0) | 1.78 / 8.93 (crank) | 1.05 / 8.68 (link:b1) | 0 err, 20 warn | 4 err, 20 warn |
| klann_high_step_single | yes (8 layers, 30.7 mm) | 3.58 / 138.61 (pin:D) | 8.98 / 399.89 (pillar:A) | 3.57 / 59.33 (crank) | 1.08 / 39.55 (link:b1) | 0 err, 16 warn | 0 err, 7 warn |
| klann_lego_decker | yes (10 layers, 42.8 mm) | 3.28 / 62.38 (pin:D_leg1) | 6.36 / 161.64 (pillar:A_leg0) | 2.52 / 28.32 (crank) | 1.61 / 37 (link:b4) | 4 err, 18 warn | 1 err, 8 warn |
| klann_lego_double | yes (9 layers, 34.1 mm) | 3.27 / 70.6 (pin:D_leg1) | 6.43 / 135.26 (pillar:A_leg1) | 3.57 / 21.22 (crank) | 1.6 / 40.18 (link:b4) | 4 err, 17 warn | 1 err, 10 warn |
| klann_lego_quad | yes (14 layers, 58.2 mm) | 3.23 / 31.5 (pin:D_leg0) | 4.02 / 38.85 (pillar:A_leg0) | 1.78 / 10.97 (crank) | 1.58 / 21.77 (link:b4) | 8 err, 21 warn | 1 err, 10 warn |
| klann_lego_single | yes (8 layers, 30.9 mm) | 3.27 / 89.97 (pin:D) | 7.59 / 206.51 (pillar:A) | 3.57 / 65.2 (crank) | 1.6 / 52.42 (link:b4) | 2 err, 16 warn | 1 err, 6 warn |
| klann_long_legs_decker | yes (10 layers, 42.4 mm) | 3.26 / 81.04 (pin:C_leg0) | 6.18 / 249.87 (pillar:A_leg0) | 2.52 / 37.55 (crank) | 0.99 / 20.79 (link:b1) | 0 err, 18 warn | 1 err, 8 warn |
| klann_long_legs_double | yes (9 layers, 34.0 mm) | 3.26 / 69.99 (pin:C_leg0) | 6.23 / 133.89 (pillar:A_leg1) | 3.57 / 23.4 (crank) | 0.99 / 21.2 (link:b1) | 0 err, 17 warn | 1 err, 10 warn |
| klann_long_legs_quad | yes (14 layers, 58.1 mm) | 3.21 / 35.12 (pin:D_leg3) | 3.91 / 40.13 (pillar:A_leg0) | 1.78 / 12.24 (crank) | 0.97 / 10.51 (link:b1) | 0 err, 21 warn | 1 err, 18 warn |
| klann_long_legs_single | yes (8 layers, 30.7 mm) | 3.26 / 130.57 (pin:D) | 7.38 / 472.25 (pillar:A) | 3.57 / 57.86 (crank) | 0.99 / 39.55 (link:b1) | 0 err, 16 warn | 1 err, 6 warn |
| klann_patent_decker | yes (10 layers, 42.4 mm) | 3 / 90.97 (pin:D_leg1) | 6.17 / 216.06 (pillar:A_leg0) | 2.52 / 40.15 (crank) | 0.91 / 23.79 (link:b1) | 0 err, 18 warn | 1 err, 8 warn |
| klann_patent_double | yes (9 layers, 34.0 mm) | 3.01 / 63.31 (pin:D_leg0) | 6.25 / 120.86 (pillar:A_leg1) | 3.57 / 16.95 (crank) | 0.89 / 20.15 (link:b1) | 0 err, 18 warn | 1 err, 10 warn |
| klann_patent_quad | yes (14 layers, 58.1 mm) | 2.96 / 27.76 (pin:D_leg0) | 3.9 / 26.45 (pillar:A_leg0) | 1.78 / 11.25 (crank) | 0.9 / 8.25 (link:b1) | 0 err, 22 warn | 1 err, 18 warn |
| klann_patent_single | yes (8 layers, 30.7 mm) | 3.01 / 371.64 (pin:D) | 7.35 / 802.27 (pillar:A) | 3.57 / 117.98 (crank) | 0.91 / 118.3 (link:b1) | 0 err, 16 warn | 1 err, 6 warn |
| klann_quad | yes (14 layers, 77.2 mm) | 0.66 / 4.26 (pin:D_leg1) | 1.22 / 6.03 (pillar:A_leg1) | 1.78 / 1.98 (crank) | 0.46 / 1.77 (link:b1) | 0 err, 20 warn | 5 err, 26 warn |
| klann_single | yes (10 layers, 34.9 mm) | 0.96 / 41.28 (pin:D) | 2.58 / 116.9 (pillar:B) | 3.57 / 66.64 (crank) | 0.57 / 19.39 (link:b4) | 0 err, 16 warn | 3 err, 4 warn |
| sixbar_decker | yes (13 layers, 55.8 mm) | 5.94 / 179.12 (pin:F_leg1) | 11.15 / 201.04 (pillar:H_leg0) | 2.52 / 60.22 (crank) | 3.9 / 103.09 (link:b1) | 0 err, 14 warn | 4 err, 10 warn |
| sixbar_double | yes (12 layers, 45.2 mm) | 3.73 / 44.4 (pin:F_leg0) | 13.03 / 162.37 (pillar:H_leg1) | 3.57 / 21.94 (crank) | 3.87 / 38.37 (link:b1) | 0 err, 14 warn | 0 err, 5 warn |
| sixbar_quad | yes (19 layers, 88.0 mm) | 3.74 / 54.19 (pin:F_leg0) | 3.71 / 30.38 (pillar:H_leg1) | 1.78 / 9.14 (crank) | 3.89 / 34.09 (link:b1) | 0 err, 14 warn | 0 err, 16 warn |
| sixbar_single | yes (10 layers, 38.5 mm) | 5.95 / 222.12 (pin:F) | 15.08 / 388.96 (pillar:H) | 3.57 / 123.25 (crank) | 3.9 / 145.77 (link:b1) | 0 err, 14 warn | 4 err, 4 warn |
| sixbar_v1_decker | yes (13 layers, 55.8 mm) | 6.09 / 114.25 (pin:F_leg1) | 10.33 / 109.88 (pillar:H_leg0) | 2.52 / 32.83 (crank) | 4 / 56.98 (link:b1) | 0 err, 14 warn | 4 err, 10 warn |
| sixbar_v1_double | yes (12 layers, 45.2 mm) | 3.78 / 44.81 (pin:F_leg0) | 12.09 / 143.74 (pillar:H_leg1) | 3.57 / 19.74 (crank) | 3.96 / 35.51 (link:b1) | 0 err, 14 warn | 0 err, 5 warn |
| sixbar_v1_quad | yes (19 layers, 88.0 mm) | 2.64 / 40.59 (pin:F_leg0) | 3.44 / 28.74 (pillar:H_leg1) | 1.78 / 8.56 (crank) | 3.98 / 31.6 (link:b1) | 0 err, 14 warn | 0 err, 16 warn |
| sixbar_v1_single | yes (10 layers, 38.5 mm) | 6.08 / 139.96 (pin:Q) | 13.97 / 231.74 (pillar:H) | 3.57 / 61.01 (crank) | 3.99 / 78.23 (link:b1) | 0 err, 14 warn | 4 err, 4 warn |
| sixbar_v2_decker | yes (12 layers, 52.3 mm) | 10.57 / 56.99 (pin:F_leg1) | 21.25 / 71.61 (pillar:H_leg0) | 2.52 / 6.98 (crank) | 7.84 / 27.25 (link:b3) | 0 err, 14 warn | 0 err, 6 warn |
| sixbar_v2_double | yes (11 layers, 44.8 mm) | 10.13 / 57.2 (pin:P_leg0) | 23.44 / 92.08 (pillar:H_leg0) | 3.57 / 14.21 (crank) | 7.83 / 30.03 (link:b1) | 0 err, 14 warn | 0 err, 5 warn |
| sixbar_v2_quad | yes (18 layers, 86.5 mm) | 10.59 / 33.71 (pin:F_leg3) | 7.99 / 19.92 (pillar:H_leg1) | 1.78 / 5.11 (crank) | 7.86 / 17.7 (link:b3) | 0 err, 14 warn | 0 err, 13 warn |
| sixbar_v2_single | yes (9 layers, 39.8 mm) | 4.51 / 58.73 (pin:F) | 33.63 / 128.15 (pillar:H) | 3.57 / 20.34 (crank) | 7.78 / 30.83 (link:b3) | 0 err, 14 warn | 0 err, 5 warn |
| sixbar_v3_decker | yes (12 layers, 50.6 mm) | 4.81 / 1016.63 (pin:F_leg1) | 18.23 / 8231.81 (pillar:H_leg0) | 2.52 / 109.95 (crank) | 5.77 / 1141.41 (link:b1) | 0 err, 14 warn | 0 err, 7 warn |
| sixbar_v3_double | yes (11 layers, 44.7 mm) | 6.66 / 38.09 (pin:K_leg0) | 19.67 / 109.54 (pillar:H_leg0) | 3.57 / 33.69 (crank) | 5.71 / 45.95 (link:b1) | 0 err, 14 warn | 0 err, 5 warn |
| sixbar_v3_quad | yes (19 layers, 84.1 mm) | 4.82 / 45.63 (pin:F_leg0) | 4.6 / 29.08 (pillar:H_leg1) | 1.78 / 8.95 (crank) | 5.74 / 33.84 (link:b1) | 0 err, 14 warn | 0 err, 11 warn |
| sixbar_v3_single | yes (9 layers, 33.6 mm) | 3.34 / 1693.03 (pin:F) | 25.93 / 5568.21 (pillar:H) | 3.57 / 240.63 (crank) | 5.76 / 1164.95 (link:b1) | 0 err, 14 warn | 0 err, 5 warn |
| strider_decker | yes (16 layers, 70.4 mm) | 3.69 / 18.29 (pin:J7_leg0) | 5.02 / 26.89 (pillar:J6_leg0) | 2.52 / 7.06 (crank) | 2.18 / 7.11 (link:b6) | 0 err, 14 warn | 0 err, 15 warn |
| **strider_double (default)** | yes (15 layers, 66.1 mm) | 3.73 / 24.74 (pin:J7_leg0) | 9.29 / 169.49 (pillar:J6_leg0) | 1.78 / 8.05 (crank) | 2.2 / 13.5 (link:b6) | 0 err, 14 warn | 0 err, 10 warn |
| strider_quad | yes (25 layers, 119.3 mm) | 3.72 / 91.42 (pin:J7_leg0) | 1.71 / 39.56 (pillar:J2_leg0) | 1.78 / 31.65 (crank) | 2.19 / 25.29 (link:b6) | 0 err, 14 warn | 0 err, 22 warn |
| strider_single | yes (11 layers, 43.4 mm) | 3.71 / 16.07 (pin:J7) | 22.91 / 86.94 (pillar:J2) | 3.57 / 13.04 (crank) | 2.19 / 7.1 (link:b6) | 0 err, 14 warn | 0 err, 7 warn |
| trotbot_decker | yes (14 layers, 63.6 mm) | 11.38 / 56.08 (pin:J6_leg1) | 27.07 / 90.32 (pillar:J3_leg0) | 2.52 / 9.21 (crank) | 4.77 / 15.9 (link:b1) | 0 err, 14 warn | 0 err, 14 warn |
| trotbot_double | yes (12 layers, 47.2 mm) | 5.19 / 9.6 (pin:J5_leg0) | 24.07 / 63.56 (pillar:J3_leg1) | 3.57 / 13.04 (crank) | 4.65 / 10.87 (link:b1) | 0 err, 14 warn | 0 err, 6 warn |
| trotbot_heel_decker | no | - | - | - | - | - | no plan within the budget |
| trotbot_heel_double | yes (19 layers, 68.1 mm) | 2.17 / 4.9 (pin:J5_leg0) | 13.11 / 38.48 (pillar:J3_leg1) | 3.57 / 12.84 (crank) | 2.77 / 11.42 (link:b1) | 0 err, 14 warn | 0 err, 4 warn |
| trotbot_heel_quad | no | - | - | - | - | - | no plan within the budget |
| trotbot_heel_single | yes (14 layers, 56.6 mm) | 3.01 / 17.07 (pin:J11) | 21.63 / 105.87 (pillar:J3) | 3.57 / 23.93 (crank) | 2.78 / 23.66 (link:b1) | 0 err, 14 warn | 0 err, 4 warn |
| trotbot_quad | yes (22 layers, 104.6 mm) | 5.99 / 19.5 (pin:J7_leg0) | 2.67 / 7.45 (pillar:J3_leg0) | 1.78 / 2.83 (crank) | 2.29 / 6.85 (link:b2) | 0 err, 14 warn | 0 err, 28 warn |
| trotbot_single | yes (10 layers, 41.7 mm) | 5.93 / 20.08 (pin:J7) | 31.07 / 144.7 (pillar:J3) | 3.57 / 23.8 (crank) | 4.87 / 22.69 (link:b2) | 0 err, 14 warn | 0 err, 7 warn |
| trotbot_toe_decker | yes (22 layers, 77.6 mm) | 1.93 / 17.69 (pin:J11_leg0) | 7.07 / 69.82 (pillar:J3_leg0) | 2.52 / 5.87 (crank) | 1.72 / 11.27 (link:b6) | 0 err, 14 warn | 0 err, 10 warn |
| trotbot_toe_double | yes (19 layers, 68.2 mm) | 1.49 / 4.82 (pin:J5_leg0) | 7.59 / 35.37 (pillar:J3_leg1) | 3.57 / 7.36 (crank) | 1.7 / 7.14 (link:b6) | 0 err, 14 warn | 0 err, 7 warn |
| trotbot_toe_quad | no | - | - | - | - | - | no plan within the budget |
| trotbot_toe_single | yes (14 layers, 47.9 mm) | 2.26 / 17.07 (pin:J10) | 15.48 / 108.44 (pillar:J3) | 3.57 / 22.76 (crank) | 1.75 / 19.58 (link:b6) | 0 err, 14 warn | 0 err, 5 warn |

Reading it.

**The default Strider double passes**: 15 layers (66.1 mm a side), 399 parts, $266.14, 0
errors and 10 warnings. The crank is the weakest joint, jam SF **1.78** (the friction
clamp's 3.032 N·m against 2 x 0.85 N·m; a warning, and an estimate: the test build should
measure the slip torque, and the hex-standoff crankpins of the user's decision 1 replace it
with a hex bearing); then the link b6 (acrylic, a three-pin plate in bending) 2.2, the pins
3.73 (J7), the pillars 9.29 (J6; J2 3.08 walking at 50 N jammed). The **Strider quad plans**
now (25 layers, 119.3 mm, not proven thinnest within the budget) and passes with warnings:
its 51 mm pillar J2 at jam SF **1.71**, the splice rated at the 1.0 N·m clamp of the user's
decision 2 (segments turned together in soft-jaw pliers): **to be tested on the first
build** (finger tight, 0.4 N·m, it failed). Every Strider module passes; so do the six-bar
v2 and v3, TrotBot and its heel and toe (single and double; the toe decker too), and the
Klann high-step but its quad (four deck findings).

**The crank** is 1.78 wherever two crankpins sit 180° apart (factor 2), 2.52 on a decker,
3.57 elsewhere: never an error, a warning at factor 2 on every design.

**The link plates** (`strength.link_rows`, the sim's pin loads on each link's net section
and, with three pins, its bending) are the new errors, honest ones: acrylic can't take the
jam loads on fourbar b1 (SF 0.35-0.53), the Jansen b6 and b3 (0.24, 0.71), the demo Klann b1
(0.46-0.63) and the Klann variants' b1 (`klann_patent` 0.89-0.91, `klann_long_legs`
0.97-0.99; `klann_high_step` 1.05-1.08, a warning). Each row names the aluminium sheet that
would hold it. The user's rule (aluminium only where acrylic can't take the load) says
those go to aluminium; only `klann_lego`'s b1 has been decided (6061), and there the **cut
rules** fail instead: the crank rider's 6.3 mm bore in a 12 mm link leaves a 2.82 mm web,
under 1 x the 3.175 mm thickness. 6061 holds it at jam SF 5.7-5.8; 5052 0.090 in would hold
it at about 2.9 with a 2.82 mm web over 1 x its 2.29 mm (a warning), in a 3 mm layer (0.7 mm
of end play to take up); or a wider link end round the crank bore (the plates). Left for the
user: `link_sheets` selects either.

**The pins**: the demo Klann (D, 0.57-0.96) and the Jansen double (C, 0.78) as before; the
rest hold (TrotBot's toe double 1.49 and decker 1.93 warn).

**The pillars**: no errors anywhere (the demo Klann quad 1.22, the Jansen 1.35-1.44 warn).

**Other problems the audit finds** (not strength, other builders' parts): on the four-bar doubles and quads and
the six-bar single and decker (v1 too) a pillar H screw clashes with the deck plate (5-37 mm^3),
the Jansen's foot socks clash with b6, and on the four-bar spot-micro and Klann high-step
quads a pin screw sweeps through the deck rail screws.

**What doesn't plan** (5 of 68): the Jansen decker and quad, the TrotBot heel decker and quad
and the toe quad (the heel and toe in gaps give up on the crankpin's washers, sunk they run
out of budget).

## Mechanisms x crank, 2026-10-04 evening

**Since the merge of 2026-10-04** (the hex-standoff crank is the `bolt` crank): this table
is the round standoff's (now `bolt_round`). With the hex crank `hoecken_pantograph` and
`dwell_rocker` plan (9 layers each: the horn spacer is a layer thicker for the short crank's
head, or caps a pin wholly under it), so they no longer keep the keyed crank
(`config.LINKAGE_CRANKS` holds only TrotBot's heel and toe, on `bolt_round`: the hex's 8.5
mm sleeve doesn't clear b7 at crankpin J1).

Every one-input mechanism, one side, with each crank (`config.DEFAULT_CRANKS` /
`LINKAGE_CRANKS`; the crank's rating from the drive torque alone, factor 1; run
`20261004-143516`). The bolt crank plans all but two, at jam SF **3.57** (the keyed crank's
key in its printed web socket holds 0.545 N·m, SF 0.64, an error; the printed crank's clamp
0.17, SF 0.20); `hoecken_pantograph` and `dwell_rocker` find no bolt-crank plan (their
crankpin M is so short that its top screw head over the hub plate needs 2.5 mm of the
printed horn spacer, which is 1.83 mm, at every layering), so they keep the keyed crank, the
strongest that plans for them, and their audit fails on it until the drive's horn spacer or
the hex-standoff crank takes that head. `five_bar` has two inputs (one servo per machine).

| mechanism | bolt (default) | keyed | printed |
|---|---|---|---|
| hoecken | 8 layers, 28.2 mm, SF 3.57 | 8 layers, SF 0.64 | 7 layers, SF 0.20 |
| watt_crank | 8 layers, 30.0 mm, SF 3.57 | 8, 0.64 | 7, 0.20 |
| peaucellier_crank | 9 layers, 35.6 mm, SF 3.57 | 10, 0.64 | 10, 0.20 |
| parallelogram_lift | 8 layers, 30.0 mm, SF 3.57 | 8, 0.64 | 7, 0.20 |
| watt_table_lift | 8 layers, 35.7 mm, SF 3.57 | 8, 0.64 | 8, 0.20 |
| hoecken_pantograph | no plan (horn spacer) | **11 layers, 31.1 mm, SF 0.64 (default)** | 11, 0.20 |
| crank_rocker | 8 layers, 28.2 mm, SF 3.57 | 8, 0.64 | 7, 0.20 |
| rocker_amplifier | 8 layers, 32.3 mm, SF 3.57 | 9, 0.64 | 9, 0.20 |
| dwell_rocker | no plan (horn spacer) | **8 layers, 22.1 mm, SF 0.64 (default)** | 7, 0.20 |
| hoecken_table | 8 layers, 35.9 mm, SF 3.57 | 8, 0.64 | 7, 0.20 |

## Superseded: every walker x module, 2026-10-04 morning (the two-plate bolt crank, 0.125 in plates)

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
