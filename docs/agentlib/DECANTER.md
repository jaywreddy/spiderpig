# Decanting cradles

This design record covers the hand-cranked decanting cradles in this repo. The user asked
for them on 2026-10-09: a linkage cradle that tips a 1.5 L magnum or a 3 L bottle to any
angle, driven by a hand crank, not a servo, with an LED. Regular 750 ml bottles came on
2026-10-10, and 6 L bottles on 2026-10-13 (the user: "both", a Bordeaux impériale and a
Champagne methuselah); the methuselah is the design bottle since ("Sized for a 6 L").

Two linkages are built, both from the same parts: the drive, the cradle box, the stand
and the light.
- `decanter_rcm` (B), **the one to build**, the user's main design: the hinge is virtual,
  a double parallelogram (a remote centre of motion), so no pin sits at the mouth. It swings
  from −50° (rest, an opened full bottle stands in it) to +30°, and comes apart by hand for
  the trip. Its design is under "B, the virtual hinge".
- `decanter_hinge` (A), **also builds**: the cradle swings on a real hinge at the bottle's
  mouth, from −50° (nearly upright: an opened full bottle goes in) to +30° (both bottles
  drain). The redesign of 2026-10-10 (the user's −50° rest) is under "A, redone". (Until
  design review round 2 of 2026-10-14 the docs and the cradle page called A "the one to
  build" and opened on it.)

Simplification round 1 (2026-10-21: the user's materials rule, no steel in a structural
part, and the Carvera for what the laser can't cut) made every post, tie rod, pin and the
crankpin 6061 rod and cut the cleats and the riser locator at home: its state, numbers and
open items are under "Simplification round 1".

The adversarial reviews of 2026-10-10 (mechanical, fabrication, code) shaped what follows.
Each finding and its fix is in [DECISIONS.md](DECISIONS.md).

To watch either one pour, run `mise run view` and open `/decant.html` (`viewer/src/decant/`).
It shows the 3D cradle beside its mechanism in the linkage's plane, with the bottle's tilt and
the handle's turns from rest.

## Prior art (the research of 2026-10-09)

**The traditional cradle.** In the antique cradles (late 1800s to early 1900s), a crank
turns a lead screw that tips a bottle cage. A candle in a brass holder under the neck
lights the shoulder, and the pour stops when the sediment shows there. The sources:
- [Wine Spectator: what is this decanting device](https://www.winespectator.com/articles/what-is-this-decanting-device-i-bought-46277)
- [an antique cradle for sale](https://www.chezpluie.com/products/early-20th-century-mechanical-wine-decanting-cradle-141-2)

**Modern professional units.**
- [Vcanter](https://rosehillwinecellars.com/products/vcanter-rustico-6-liter): 1.5–6 L and
  up, with a gear arm.
- [Vinum Design pourer](https://vinumdesign.com/en/collections/wine/products/large-bottles-pourer-decanter-cradle-machine):
  1.5–18 L, 13.5 kg, CA$2,795.

**Patents.**
- Bersano (US 3,868,047, 1975) is a hand-cranked cradle pivoting along the cradle, so the
  mouth drops as you crank; it also has a light beam through the neck.
- [EP0398015](https://patents.google.com/patent/EP0398015B1) names that flaw: the swivel
  axis isn't at the mouth, so the hand holding the carafe has to follow it.
- [US7708241](https://patents.google.com/patent/US7708241) uses a motorized worm and puts
  the tilt axis at the neck, so the lip stays touching the decanter's lip. Its range is
  −28° to +37.5°.

**Why the light.** A light under the shoulder shows the sediment arriving, and the pour
stops there:
- [VinePair](https://vinepair.com/wine-geekly/ritual-decanting/)
- [Chowhound](https://chowhound.com/1598172/why-sommeliers-use-candles-test-wine)

## Requirements

| what | value | why |
|---|---|---|
| bottles | a 750 ml to a 6 L (impériale, methuselah) | the user, 2026-10-09 (magnum and 3 L), 2026-10-10 (regular bottles too), 2026-10-13 (6 L, both shapes) |
| bottle diameter | 74–174 mm | Bordeaux 750 ml 74–77, Bordeaux magnum 99, Champagne jeroboam 140.8, Bordeaux impériale 156.5–164.5, Champagne methuselah 170–174 (Saverglass) |
| bottle length | 295–571 mm | 750 ml 295–310, magnum 356–404, 3 L 380–470, impériale 493, methuselah 571 |
| full mass | 1.3–11.1 kg | glass 0.5–5.1 kg plus the wine |
| the mouth | stays put over the whole pour, for either bottle | the decanter stands under it (US7708241) |
| tilt | −50° (rest, neck up) to +30° (neck down) | the user, 2026-10-10: start nearly upright. At +20° both bottles' shoulders lie over the mouth's lowest lip, so they drain; one crank turn is one decant and back |
| the mouth's height | 550 mm over the table | at −50° the base stop's rails hang about 514 mm under it |
| the bay | 260 mm between the inner plates | a 174 mm bottle beside the rear saddle's post and knob; a standard decanter (up to 240 mm across) stands wholly between the side stacks |
| vessel | a standard decanter or glass on the printed riser in the base board's riser locator, one fixed height per decanter, its rim 20 mm or more under the lowest part over it | the vessel study (204–300 mm tall decanters, 155–240 across); the room under the mouth, below; round 1 of 2026-10-14 (no raising it mid-pour); round 2 (no lab jack: a fixed riser) |
| drive | hand crank, self-locking | holds any angle when let go |
| light | an LED under the shoulder, riding the cradle | the candle's job |
| material | aluminium first; the base board wood | the user (2026-10-13: "the bottom board in wood"); the look is decided from the viewer |

## The machine

- **Two side stacks**, each planned by `stack` as usual: an outer frame plate, the links,
  an inner frame plate. The bottle lies in the bay between the two inner plates.
  - The drive side (`L.`) carries the crank.
  - The passive side (`R.`) is the drive side's mirror image without the crank, the rod
    and the gearbox; the cradle box drives it. Its plates are the drive side's (the same
    files, flipped).
  - Code: `spiderpig/construction/cradle.py`; the linkages are in
    `spiderpig/linkages/decanters.py`.
- **The cradle box.**
  - Four tie rods run between the two cradle links, at `decanters.TIES`. Each is a 1/2 in
    6061-T6511 rod sawn from McMaster's 3 ft stick (Speedy Metals' cut piece until
    simplification round 2's P5), faced square and tapped M6 at home
    (`cradle_catalog.al_rod`, since simplification round 1, 2026-10-21, the user's
    materials rule; a 12 mm steel standoff, MISUMI NETRF12, from design review round 2's M1),
    with an M6 x 16 button head through the link into each end (`cradle.tie_screw`): on A
    class 10.9 steel at 6 N·m (since 2026-10-20: A2-70 at ISO 7380's 4 N·m left the joints at
    their gap), on B A2-70 at 4 N·m again since simplification round 2's P8 (its racking
    brace takes the jam's twist). Two carry the front saddle under the neck, two the rear
    saddle 275 and 360 mm behind the mouth.
  - On B a racking brace (since simplification round 2, 2026-10-21: `decanters.BRACE`): a
    2 in 6061 tube across the bay behind the rear saddle, under the base stop's rails, its
    ends keyed into toothed sockets in both cradle links; it carries the box's twist at a
    jam ("The cradle box across the bay"). B's cradle link b6 is cut to a band along the
    bottle's axis (`config.PRODUCT_BAND`, the round's way back), so its boss at X and its
    wide rear chords don't stand the cradle box taller in the pack.
  - Two printed saddles ride on the tie rods (`cradle.SADDLES`; since 2026-10-14, under
    "The saddles, adjustable"). Each is a carrier locked on its rods by M3 set screws up
    from its underside, and a V block that rides up and down in it on a thumbscrew.
    - The front one carries the neck: its V block slides in the carrier's box guide, an M6
      thumbscrew at its end. Its V bears on the neck only to 64.5 mm from the mouth
      (`cradle.FRONT_BEAR`, since design review round 3): behind that it is relieved 6 mm,
      and the carrier's back wall notched 6 mm deeper, so no shoulder lands on either.
    - The rear one carries the body: its V block rides a 5/8 in 6061 post and, 27 mm
      beside it at the same end, a printed coarse lead screw (13 mm, 8 mm a turn; both
      since simplification round 1) that pushes it up
      (since review round 2; the post had been at the far end, and the block jammed on it);
      the screw's knob drops onto a ring of teeth that lock it. The base stop's two rails
      continue its V behind it on 45° flanges to 580 mm, so a 6 L's whole body is carried:
      lying level or tipped, a bottle's weight centres on the saddles
      (`cradle_report.balance`, "The saddles, adjustable").
    - The Vs are 90°, lined with 2 mm TPU pegged into them. At a 6 L methuselah's setting
      each V block sits on its tie rods (the rear one in a seat over each); a thinner
      bottle's ride higher.
  - **The lip gauge** (`cradle._gauge`): a printed arm fixed on the front carrier (two
    cheeks either side of a post, an M5 tap bolt through them, its bridge flat on the
    post's top). Its pad stands in front of the mouth, its back face on the pivot's plane,
    over the top of the rim (above a cork's bore), and its bar runs back beside the neck,
    not over it: a bottle lays in under nothing. It checks where the mouth is, and it is
    the forward stop: past level a bottle would slide toward its mouth, and the rim's top
    bears on it. Dry, the liners hold a bottle on the gate the whole pour; a wet one slides
    onto the pad mid-pour, which its post (21.7 mm across since round 3,
    `cradle.GAUGE_POST_OUT`) takes (`cradle_report.gauge_slide`). A bottle is uncorked
    standing on the table before it goes in (since review round 2: the gauge had hinged up
    to uncork and fallen back by its own weight, barely more than its hinge's friction).
  - **The base stop** (`cradle._rails`, `cradle._gate`): a printed gate across the V behind
    the bottle's base, riding the rear V block's flats and two printed rails behind the
    block. It is the back stop: at rest the bottle stands 50° neck up and leans on it (at
    25° nose up the liners' friction alone had held a bottle, and a corkscrew's push of
    6-13 N slid it back off the pivot: the mechanical review).
    - Keys under its ears run in a groove along each flat and rail. It is set by a scale
      grooved across the +z flat and rail and locked by hand: a printed knob on an M5 tap
      bolt through one ear into a nut and a fender washer in a T-slot under the groove.
      Its top bar and ears stand 9 mm over the rails (`cradle.GATE_TOP`), the ears' roots
      filleted, and it prints on its top, so the clamped ear's root and the pointer bend in
      its layers.
    - It serves every base from 285 mm (inside the rear V: a 750 ml's at 300.5) to 576 mm
      (a 6 L's at 571) behind the mouth, its face 2.5 mm behind the base
      (`cradle.STOP_GAP`).
    - It rides the rear V block, so it rises with it. At rest, set for a 6 L, it comes
      down to 26 mm over the base board's level, a finger's 25 mm (ISO 13854): the flanges
      and keels end at 580 mm, past a 6 L's heel, and the +z rail's T-slot keel is
      chamfered at its end (`cradle.FLANGE_END`, `cradle.finger_gap`). Since round 2 the
      board has a window under it (`cradle.BOARD_WINDOW`): what lies there lies on the table,
      43 mm under it.
- **The bottles** (`cradle.setting`, `cradle.seat`): every one lies with the mouth on the
  pivot and the axis level, so the mouth stays put for each. Its setting is the two V
  blocks' rise and the base stop's place, read off the scales with the bottle out (see
  "The saddles, adjustable"); the base stop and the lip gauge hold it there.
  - The 750 ml and the 6 L impériale are drawn as their makers draw them (Saverglass's
    4427 and univerre's NGBD06QFR, traced: `cradle.SAVERGLASS_4427`,
    `cradle.UNIVERRE_NGBD06QFR`), the others as a cylinder neck, a cosine shoulder and a
    cylinder body. The methuselah's neck is still an estimate scaled from the jeroboam's:
    set the front knob to the neck in hand.
  - Every bottle is measured first (two of a size differ: 750 ml Bordeaux run 300.5 to 315
    mm long): each knob set to its neck's diameter (62 mm from the lip, the front V's
    bearing's back end: `cradle.NECK_AT`) and its body's (255 mm from the lip, where it
    lies on the rear V's front edge) on the scales' graded rows, the base stop to its
    length on a plain length scale with a vernier. `cradle.bottle_room` checks a real
    outline at its setting against the V's relief, the back wall and the light.
- **The drive** (`spiderpig/construction/handcrank.py`, `config.crank == "hand"`; round 1's
  drive review of 2026-10-14 reworked its riser, its block, its tie and its hand crank;
  round 2's, of the same day, its gearbox, riser, block and clutch; round 3's, of
  2026-10-20, its riser, the clutch's setting and cap, and the toggle cue: DECISIONS.md).
  - **Gearbox:** an NRV050 worm gear reducer at 30:1 (Phidgets TRM4402_0, "Maximum Strength
    of Gears 85 N·m"; `handcrank.NRV050`), the user's call of 2026-10-14: the NRV040's
    45 N·m maximum was under the clutch's cap, and no clutch that still starts a full 6 L
    stays under it. It stands outboard on the drive side's outer plate, on a clamped stack
    on four M8 studs in its tapped face (85 mm PCD). A full 6 L peaks at 36-38 N·m at the crank: SF 2.37 on B,
    2.25 on A against the 85 N·m; the clutch's cap 1.33 and 1.27 x under it
    (`cradle_drive.GEARBOX_CAP_FLOOR`, 1.25).
  - **Its data** (each cited in `handcrank.NRV050`'s docstring): Phidgets' specification tab
    (85 N·m, 4.8 kN radial, a 14 mm input with a 5 mm key, a 25 mm bore with an 8 mm key)
    and its drawings (the input's 30 mm past its shoulder 64 mm from the output axis, the
    worm 50 off it, an M6 end tap; the output shaft TRM4008_0, 25 h6, a 32 x 3.5 collar,
    50 past it, keyed 8 x 7 x 40 ending 5 short, an M10 x 16 end tap); where Phidgets is
    silent, Motovario's NRV050 (Phidgets sells a clone of the pattern): the face 4 x M8 x 10
    on 85 PCD, a 70 h8 spigot, the face's 100 mm boss, 87 mm face to face, 3.3 kg dry; the
    mesh at 30:1, lead 6°19', static efficiency 0.44 run in (0.387 new: SEW's 12 % run-in
    loss for a single-start worm), 0.657 at 500 rpm input (113 N·m, 0.30 kW, 17 rpm); the
    input's Fr1 490 N (a 129, b 114) and the output's Fr2 4840 N (a 101, b 76).
  - **No rating at a hand's speed** (round 2's M-1): round 1 had taken Motovario's 62 N·m
    at 500 rpm, scaled to Phidgets' 45, as the box's rating at a hand's speed (58.1 N·m),
    and its momentary limit as 2 x the 45. Motovario's own ATEX catalogue rates the same
    NRV040 30:1 at 46 N·m at 500 rpm (45-46 from 1750 down), so the rise to 62 is one
    catalogue's wear and heat rating, not the gears' strength, and Phidgets calls its 45 a
    maximum. Withdrawn: the box is rated against Phidgets' maximum only.
  - **Its efficiency at a hand's speed** (`WormBox.eta_at`, `handcrank.HAND_RPM`): the
    catalogue's 0.657 at 500 rpm carried down by AGMA's worm-gear friction against sliding
    speed (`handcrank.agma_mu`): 0.558 at 100 rpm, MIL-STD-1472H's crank rate. The clutch's
    cap takes it (a cradle blocked while the handle turns stops the worm at that speed).
  - **The riser** (`handcrank.riser_for`, `RISER_STUD`, `RISER_RING`, `RISER_SLEEVE`,
    `RISER_SLEEVE_RANGE`, `RISER_THIN`; round 3's M2): a clamped stack on the face, which stands 25.2
    mm under the plate's outer face, the spigot ring and the hub's end well under it whatever
    their heights.
    - Four M8 class 8.8 studs, cut 45 mm from one 200 mm zinc-plated DIN 976 rod (McMaster
      99067A279), each threaded into a tapped hole until it stops (243). Round 2's 18-8 set
      screws carried the gearbox in bending over the printed sleeves' 18.7 mm, rated nowhere:
      SF 0.55-0.62 at the cap as A2-50 (the review's), and a set screw's class is for a
      headless screw not loaded in tension (TR Fastenings).
    - A ring of the crank sheet on the face, its bore over the 70 mm spigot ring (it centres
      on it, the side load into it through the spigot), out to the face's 100 mm boss, an
      8.3 mm hole on each stud (the web to the bore 3.2 mm, over the sheet's thickness).
    - A 304 sleeve on each stud (8 x 12 x 20 mm, Harfington's, five to a pack): 12 mm across
      clears the spigot by 1.5 mm on the 85 mm PCD, the stock size that does.
    - The go/no-go's shims over the sleeves (0.5 mm, 8 x 14, twenty to a pack, four DIN 125
      washers as coarse ones): about four a stud drawn; a clone's hub prouder or lower than
      drawn is shimmed out, a millimetre either way. (Until simplification round 2: since,
      the sleeves are faced to the gearbox's calipered height, 21-23 mm, `RISER_AIM` over
      the go/no-go window's low end, and no shim is bought.)
    - The plate, and a thin nut over it on each stud at 10 N·m: one stack, clamped. Friction
      carries the gearbox's side load and turn, the sleeves the columns' bending, the studs
      their tension (`cradle_drive.riser_rating`): walking it doesn't slip at an oiled
      friction of 0.15 by 2.9 (B) and 2.7 (A); at the cap 30° from the toggles the stud is SF
      2.33 / 2.29 on class 8.8's proof, the ring's bearing under a sleeve 1.73 / 1.63, and it
      holds 1.3 outside 15° of a toggle (a row of the capped case). Oiled, it may slip at the
      cap; the snug sleeve then puts the side load into the stud in shear (in its rating).
    - Each stud's end filed to 0.8 mm or less over its thin nut (`handcrank.STUD_PAST_MAX`):
      the crank block's lowest plate passes over the four, 2.2 mm drawn and 0.6 mm with the
      go/no-go's window and the six plates under the seat at their thickest (round 3's m9:
      1.45 mm drawn had been under that stack-up).
  - **Holding:** its static efficiency at 30:1 is 0.44 (Motovario's mesh data), under the
    0.5 that the makers' catalogues give for static irreversibility. So the cradle holds
    any angle when let go. Phidgets' unit is a Chinese NMRV clone, not Motovario's mesh, so
    the guide bench-tests it (a full bottle at level, the table tapped, 10 minutes); one
    that creeps settles back neck up, never further, and an O-ring's drag on the input
    doubles the margin.
  - **The crank block:** eight 0.125 in 6061 plates (`handcrank.BLOCK_PLATES`), each the
    arm's whole outline, on the stock 25 mm keyed output shaft; its top where round 1 had
    it, so the rod rides where it did and the planner's plan is unchanged.
    - Each plate's outline (the arm's pill on its hub's disc) is closed R10 at the neck
      (`plates.CRANK_FILLET_R`, design review round 3's laser-cut minor 1: two 55.6° cusps
      there, the arm's most bent section); what the closing adds lies in the crank's own
      sweep and is claimed (`plates.arm_fills`). The keyway's web to the hub's edge stays
      4.83 mm (1.52 t: a 6061 warning, 2.5 x SendCutSend's hard minimum); a hub 1.5 mm
      bigger would make it 2 t for USD 1.20 more on B, not done (every round-3 fix USD 0).
    - The lowest three are keyed on the 8 mm key's straight flanks (`KEYED_PLATES`; the
      keyseat is end-milled, so the key's ends are round). Their slots are cut 0.25 mm
      under the key (`KEY_SLOT`: a laser's ±0.13 stays under it) and filed together on the
      key to a slide fit, so each bears (round 2's m4). Since simplification round 2 every
      bored plate is the same cut (one file of seven, USD 3.64 less live): all seven are
      stacked and filed together on the key before the block goes together, and still only
      the keyed three bear (round 2's laser-cut review, its minor 2: round 1 had cut all seven
      slotted plates at the key's width and counted four; the three over them had been
      slotted 0.4 mm wider than the key until simplification round 2). One
      plate alone holds 49.4 N·m at bearing yield on the sheet's thinnest, 3.048 mm
      (`handcrank.PLATE_UNDER`: SendCutSend's 0.125 in 6061, +0.152 / -0.127 mm, its spec
      table fetched 2026-10-15), 1.40 x B's walking peak; the three 148 (2.35 x the cap).
    - It is built and fitted on the bench as one block, tied by an M6 button head through
      every plate into a thin nut under the lowest (`handcrank.TIE_NUT`).
    - Its seventh plate seats on the shaft's steel end face, an M10 x 16 button head through
      it into the shaft's end tap, threadlocker, 15 N·m (TR Fastenings' maximum for an ISO
      7380 button head in A2-70 at M10, its socket screws' table; round 3's m1: 20 N·m had
      been over it; the drive's M6 button heads 4 N·m, the same table's); the top plate rings
      the screw's head. The riser's shims put the shaft's end under the seat
      (`handcrank.SEAT_LOW`: up to 0.7 mm under, the block then sitting that much low over
      the head layer).
    - The go/no-go (the guide's): the gearbox on its riser under the plate, the shaft's end
      measures 26.05 mm over the plate's inner face drawn; from 25.35 to 26.05 it is right;
      outside, a shim more or fewer on every sleeve (0.5 mm each).
  - **Crankpin:** a 12 mm NETRF12 standing in the block from its lowest plate (an M6 up
    through it) through the rod, an M6 and a wide washer on its end over the rod (a 6 L's
    jam bent the 10 mm one: SF 1.13), 0.2-0.3 mm of play (its length rounds down; a printed
    spacer under the rod, sanded on the bench, takes the plates' tolerance), greased where
    the rod turns. The washer keeps the rod 0.26 mm (nominal) under the rear tie's screw
    head where that head crosses it, 1 to 4 handle turns from rest. The chain's tolerance
    is about 0.3 mm, so the guide checks it with a sheet of paper: a rub there is a scrape,
    not a jam. The rod rides 3 mm over the block's screw heads, its force 9.5 mm off the
    crankpin's socket (the lever the build measures, `crankpin_lever_mm`).
  - **Hand crank** (`handcrank.HandCrank._handle`): printed in PETG (its clutch's plungers
    push on it always; `handcrank.PETG_PARTS`, round 2's m7) in two parts joined by a slip
    clutch, on the gearbox's 14 mm input shaft.
    - The hub is printed solid, its bore reamed to 14 mm, 28 mm long on the shaft, keyed on
      its 5 x 5 key (glued into the shaft's keyseat), with an M3 x 9 knurled thumb screw
      (McMaster 92545A115) through a brass heat-set insert onto the key: it comes off by
      hand. It runs on as a tube to the clutch's flange, a hex boss and a journal.
    - **The clutch** (`handcrank.clutch`, `CLUTCH`, `PLUNGER`; round 2's B-1). A laser-cut
      0.125 in 6061 seat ring (cut with the crank plates) sits on the flange, its hex bore
      on the hex boss, its laser entry face to the balls (its dross face on the flange);
      three steel ball-nose spring plungers (McMaster 5901A17: 1/2"-13, a 7.14 mm 440C ball,
      6-12 lbf) threaded through hex nuts pressed into the arm's boss from its inner face
      (the pocket's floor takes the springs' push and the jam nut's clamp: round 3's m2), a
      jam nut on each, press their balls onto the edges of the ring's six 3.57 mm holes on a
      40 mm circle: a 30° seat (the ball on the hole's edge; a 3.57 mm hole is over
      SendCutSend's least, 3.175, which a 30° seat for the reviewer's 4.8 mm ball isn't).
    - **Its plungers' setting** (`handcrank.SET_BACK`, `NEAR_STOP`, `plunger_setting`; round
      3's B1): round 2's "turn each in until its back is flush with its jam nut" pushed each
      1.98 mm, its nose onto the ring at 1.35 mm with its ball still 0.48 mm down in its hole:
      no clutch at all. Now the stop is felt, not measured: the arm at a click, each plunger
      turned in until it stops hard (its nose on the ring, wherever its nuts are), then out
      half a turn (0.98 mm: a push of 0.375 mm, 32 N, the ball 0.98 mm of room to climb;
      drawn so), and never nearer the stop than an eighth of a turn (0.24 mm of room, over
      the code's 0.2). A test holds the drawn parts slippable, the plunger's body over the
      ring's face by its room on B's and A's crank. A plunger whose ball bottoms before its
      nose meets the ring only stops sooner: the safe side.
      The arm turns on the journal, held by a steel M16 washer and a printed cap keyed on
      the journal's flat (round 2's m6: the cap can't turn with a slipping arm and unscrew
      its thumb screw). Its slip (`handcrank.clutch_slip`): each ball pivots on its hole's
      edge, its contact angle falling from 30° as it climbs, the plunger pushing harder;
      the force on it P = F k / (1 - mu2 k), k = tan(angle + atan mu1), mu1 at the edge and
      mu2 in its body's bore, with the cap's thrust washer's friction; greased contacts
      never lock (they would at a friction of 0.58), dry ones do.
    - **Set on a measurement, under a ceiling** (`handcrank.CLUTCH_SET`, the user's call;
      `clutch_ceiling`, round 3's M1): once the cradle is built (the guide's last build step,
      "Before the first pour", round 3's stand and use M2: it had been set before the cradle
      box existed), the heaviest full bottle the user pours, turned to where it is heaviest,
      the luggage scale on the grip reads the start; the clutch is set to slip at 1.3 times
      that, three pulls in a row within 8 % (`CLUTCH_CHECK`), an eighth of a turn of one
      plunger at a time (about 3 %: round 2's m5), and no pull ever over the ceiling: the
      drawn 6 L's window top, 30.5 N on B's grip, 30.8 on A's. The check is on a printed stub
      in the riser locator's pocket (`handcrank.check_stub`, a tool the build prints, not a
      body of the model, so the flat pack doesn't place it yet: round 3's m12, no vise), at
      every set-up (the trip's way back sends the user there), after
      greasing, after a hot car, and every month or ten pours while it stands set up. A full
      6 L reads 21.7 N on B's grip (24.7 new), 22.0 on A's (25.0): set at 28.2 and 28.6.
      With every plunger at its limit, a 10 % weak spring at the greased friction's low end
      can't reach A's (39.7 N needed, 38.6 there): the guide leaves the slip there, under the
      target, while it clears 1.1 x the start (round 3's m3; `plunger_reach`).
    - **Its cap** (`config.PRODUCT_CLUTCH_NM`): the check's top through the worm at a hand's
      100 rpm, 1.3 x 1.08 x the peak x 0.558/0.44: 63.8 N·m on B, 67.2 on A (63.7 / 67.1 before round 3's
      saddles, whose full 6 L starts 0.002 N·m harder), the jam case's torque
      (`cradle_report.jam_nm`), now a bound on the setting:
      - Set on a new box and run in after, round 2's 1.3 x took it to 72.5 / 76.3 N·m (the
        gearbox 1.17 / 1.11 x under its maximum, the review's); under the ceiling it stays
        63.8 / 67.2, the new box's 6 L still turning at 1.05 x its start at the window's
        bottom. A hand crank never runs a box in in SEW's sense (about 48 h; a pour is 15
        turns), so a run-in procedure was no fix.
      - Spun at 200 rpm (an empty cradle cranked back), the worm passes 67.9 / 71.5 N·m (the
        report's `fast_cap_nm`; the least part 1.26 / 1.28 there, the gearbox 1.25 / 1.19):
        the use limits say turn it, don't spin it (about a turn a second at most).
      - A hand at the grip's outer end, greased journal: 2 % over the checked slip (dry, 6 %;
        `handcrank.journal_share`, round 3's m6: the guide greases the journal).
      - Unchecked, the greased friction (0.09 to 0.19: RoyMech's greasy steel; no greasy
        steel-on-aluminium figure is published) moves the slip from 0.72 to 1.40 x its
        setting, 1.47 x with the seats' edges worn 0.1 mm (`handcrank.clutch_drift`): the cap
        would reach 86.8 N·m on B (over the gearbox's 85) and the slip fall to 0.93 x the
        start (it ratchets); dry (0.47-0.78) it locks. Only the check bounds that: no clutch
        geometry takes the friction out (a 20-45° seat moves the drift by a few per cent),
        and a spring force limiter in the rod, the one bound on every jam, is round 2's open
        alternative (DECISIONS.md, round 3). A worn seat is replaced at any bright land you
        can see round a hole (`handcrank.SEAT_WEAR`, 0.1 mm: round 3's m4).
    - Its grip spins on the plain shank of an M8 hex bolt through the arm's end, a nyloc nut
      on the arm's inner face. The arm's bar is 14 mm deep (round 2's m8: 10 put an
      out-of-plane 100 N at the grip at 42 MPa).
    - **B's** is MIL-STD-1472H's longest light-load crank, 125 mm (`handcrank.HANDLES`), a
      60 x 16 grip on an M8 x 90: under 22 N at the grip (`handcrank.LIGHT_LOAD_N`), a
      38-125 mm radius below 100 rpm, wrist and finger movement (§5.1.4.1.2.4.7, Figure 13,
      p. 76). A full 6 L's worst start is 21.7 N run in; 24.7 N new (`RUNIN_LOSS`), over
      the bar until the box runs in (round 2's M-5). A pour is about 15 turns (30:1 over half
      a crank turn), the many-turn task the standard puts cranks to (§5.1.4.1.2.4.1.1).
    - **A's** is 130 mm on a 230 mm reach (`handcrank.HANDLES`, round 2's M-3): the reach
      puts the arm and a hand on its grip behind A's cradle link b2 and its rear tie Tr2
      over the whole turn (round 1's 85 mm crank on a 100 mm reach had the hand through
      them for 64.5° of crank) and in front of the base stop's rails behind the cradle
      (`cradle_report.hand_sweep`, at 0.5°: the crank's solids clear everything by 17.8 mm,
      b2 the nearest; a hand on the grip, 20 mm round it, by 14.1 mm, the base stop's rail;
      on B 39.0 and 40.4 mm). A full 6 L's worst start is 22.0 N at 130 mm run in (25.0 new):
      at the 22 N bar since round 2's structure put 0.3 kg of 12 mm tie rods on the cradle
      box (21.97 N; 21.7 before). Settled at 130 mm (round 3's m5, `cradle_report.OPEN_GRIP`):
      at the bar run in, over it new, as B's (24.7 N new); 132.5 mm gives 21.55 N for 2.5 mm
      less hand room (-2.85 mm for a large hand), 125 mm 22.85 N, and no light-load radius
      brings a new box under the bar. 130 mm is 5 mm over Figure 13's light-load radius
      (`handcrank.HANDLE_OVER_RADIUS`, accepted as a documented deviation, the user's call
      of 2026-10-15): at 125 mm it is 22.8 N, and a heavy-load crank (190
      mm or more, 14.7 N) meets A's bottle and the base stop's rails at every reach to
      300 mm. (Other ways under the bar on A, not taken: a counterbalance, the user's no; a
      ratio, Phidgets' 50 series is 30:1 only.) Its long tube prints in two PETG pieces, a
      hex spigot into a socket and an M3 button head through the socket's wall into an
      insert (`handcrank.HUB_PRINT_MAX`).
    - Turn it from the drive side's rear corner, the neck in view.
  - **The toggle zones' red arcs** (round 2's M-4, round 3's stand and use M4,
    `handcrank.TOGGLE_TURNS`): within 2.5 handle turns of rest and of full tilt some part
    holds the cap at under 1.3. The output hub turns with the crank on the gearbox's far
    face, in view. In the last build step the toggles are found by the crank and the rod,
    not the bottle (it moves half a degree in three quarters of a turn there): at rest the
    arm lies folded under the rod in one straight line, at full tilt the two make one line
    end to end. An arrow painted on the hub's end, from its middle to its rim; the housing's
    rim painted red from 2.5 turns before each toggle to 2.5 after it (round 2's line across
    the hub, read as a radius, had left the return to rest unmarked). The use limits' rule 3:
    hands out, and nothing added under the box, while the arrow is in a red arc; the
    decanter stays put. The cradle page's slider shows the three zones red, its readout
    "toggle zone: hands out" (`cradle_report.motion`'s `toggle_turns`).
  - **No guard over the crank** (the user's call, 2026-10-10: the guard, a disc in the
    outer plate and then the gearbox's flange, read as a big disc either way). The gearbox
    sits on its riser; the crank turns about 2 rpm behind the outer plate's truss. On B
    its top-rear arc and the rod's end turn in the open, above and behind the gearbox,
    right in front of the hand crank (round 1): the use limits name them with the other
    pinch points (`cradle.use_steps`' use limits), and taking the handle off locks it.
  - **The handle comes off by hand** (its thumb screw): nothing turns without it, so it is
    the child lock, and the use limits say to take it off when the cradle is left. Only its
    arm may turn the cradle: a drill on the input once built, or a hand twisting the hub or
    the clutch's flange, bypasses the clutch (round 2's m10: the guide runs the box in
    before it goes on).
- **The pins** (`pivot8`, `config.PRODUCT_PINS`, since 2026-10-13): hardened steel pivot
  pins, MISUMI's precision pivot pin, straight, retaining rings (CCGH8-L-NC2.4;
  `construction.pivots.pivot_pin`).
  - A solid 8 mm g6 pin of 1045 steel hardened to 40-45 HRC, black oxide. The length L
    between its two ring grooves is made to the pin's stack to the 0.1 mm, so nothing takes
    up: 0.25-0.35 mm of axial play. An E-ring (No. 7, 14 mm across) in each groove holds the
    links on.
  - The links turn on the pin between the rings, in the M6 Chicago barrel's 8.2 mm holes.
  - Each end stands 2.4 mm past its link (the alteration NC2.4; the standard 3 mm end didn't
    fit a 3.175 mm layer with the play), so it sinks into the layer beside it as the Chicago
    screws' heads did: the stacks are as thin as with them.
  - Rated as a solid 8 mm rod at class 8.8's minimum yield, 640 MPa (conservative for its
    hardness): 4.3 x the M6 barrel. At the jam B's rod pin P went from SF 0.74 to 3.5, A's
    pin E from 0.65 to 3.1.
  - On the bench, a ring into each pin's lower groove and the pin up through its lowest
    link; the other ring once its highest link is on. Nothing is glued or threadlocked.
  - About USD 21-24 a pin (the discount is per length), so only where the jam needs it or
    nothing else fits (`config.PRODUCT_PIN_JOINTS`): B's rod pin P and its J, A's one pin
    E. (Until the review of 2026-10-14 B's K was one too, and its J and J2 M6 Chicago
    screws: 243 threadlocker, the barrel's head held from below with taped pliers.) The
    shoulder screws tried first are in [DECISIONS.md](DECISIONS.md).
  - B's J can't be a clevis pin: a clevis pin's clip and end stand in the cradle link's
    layer, and b6's rear arm X-Tr1 passes right over J (the planner finds no plan).
- **The clevis pins** (`clevis8`, `construction.pivots.clevis`, since 2026-10-13): B's X
  and Y, where its cradle link b6 meets each side, on both sides (the quick-release pins);
  since 2026-10-14 also its K and J2 (jam SF 1.43 and 2.16), so B has two pin types and
  no Chicago screw.
  - An 18-8 clevis pin, 8 x 20 mm, head 14 x 3, 15 mm usable to its 2 mm cotter hole
    (McMaster 90378A701, USD 5.47), and a hairpin cotter (an R-clip: 2 mm wire, 50 mm long,
    McMaster 98350A131, 5 for USD 3.68).
  - Its head is inside the stack, on b4's (b5's) outer face over a printed take-up spacer:
    15 mm less the 11.2 mm stack and the least play, to the 0.1 mm (3.5 mm: 0.3 mm of play).
    The head and the spacer stand 6.8 mm out, into the layer of b2 and b3, which never come
    within 70 mm of X or Y.
  - The clip goes through the pin's hole from the bay side by hand, over b6, in the gap
    under the inner plate; the pin's end passes the inner plate's layer in its open, as a
    tie rod does (the bar O1-O2 never comes within 81 mm of X or Y).
  - With the cradle laid on its drive side, pull the two clips and the passive side panel
    lifts straight up off the cradle link, its pins with it; then pull the drive side's two
    and the cradle box lifts straight up off it ("Flat pack"). The clips go back in the
    pins' holes. Without its clip a pin is held only by its keeper's grip: the steps back
    together check all four are in.
  - **The keepers** (`cradle.KEEPER`, since the pack's turn 1 of 2026-10-14): the printed
    ring in the gap between b4 (b5) and the cradle link on each of the four, printed in TPU
    95A with a 7.4 mm bore (7.6 until round 2), grips its pin. A side lying down with its
    clips out holds its pins by them: each pin's head is under its link with nothing under
    it, and a bare pin drops out. In use it is the gap ring it was. The guide pushes each
    pin's tip with 30 N on a kitchen scale (`cradle.KEEPER_PUSH_N`: one that slides is
    reprinted 0.2 mm tighter) and holds each piece's weight on the way back until both its
    pins are in ("Flat pack").
  - **No lanyards** (round 1 had proposed one per R-clip; round 2 asked why not): a clip's
    lanyard needs an anchor that doesn't turn about the clip's pin, and there is none. The
    clip sits at the pin's tip on the bay side of the cradle link; b4 (b5), the inner plate
    and the cradle link each turn about the pin through the swing (b6 turns 80° about X
    relative to b4), so a cord from the clip to any of them winds round the joint, beside
    the decanter and the bay faces' shear. Each clip goes straight back into its own pin's
    hole instead (the steps), and the BOM's pack of five R-clips leaves one spare.
  - Rated as a solid 8 mm rod at annealed 304's minimum yield (205 MPa): 1.39 x the M6
    Chicago barrel it replaced. Jammed, X 2.52 and Y 2.21 (as Chicago screws they had
    held 1.72 and 1.49).
  - K's and J2's heads stand under b2 and b3, their clips in the gap over b4 and b5, their
    ends in the cradle link's layer, where b6 never comes near them.
  - B's stack is 81.03 mm a side in 15 layers, proven (simplification round 1's rod pins:
    K and J2 lost the clevis pins' head and spacer layers); 84.65 mm in 16 layers before
    (84.05 until round 2's M6 tie screws' heads
    under the cradle link; the planner of 2026-10-14: 84.23 in 17 before it; 79.68 until 2026-10-14: the clevis pin at K moves b2 and b3 a layer up, and
    b6 is 0.187 in).
- **The pillars** are 1/2 in 6061-T6511 rod since simplification round 1 (2026-10-21):
  sawn from McMaster's hard-anodized 6750K16 where a link turns on them (B's O1 and O2, A's
  G: `cradle_catalog.al_rod_hard`), faced and tapped M6 at home; the stand's columns plain
  rod from Speedy Metals (`al_rod`). Until then 12 mm steel standoffs (MISUMI NETRF12, M6
  ends; `StandoffAxle.cradle_shaft`), each made to its plates' gap: a 6 L's jam bent the 10 mm
  ones (SF 0.78 at B's O2). An M6 button head goes through each plate without a washer, to
  6 N·m; its 3.3 mm head stands outside the plate, where a cradle's side is open
  (`StandoffAxle.outside`). The stand's columns are 12 mm too (NETRF12 on M6 x 12 button
  heads, `Stand.column_d`, `cradle.column_parts`, since 2026-10-14; they had been 10 mm,
  as the tie rods).
  - A pillar and the columns are posts between the side's two plates, and the inner plate
    is held in its plane by nothing but them: a pillar's end is held only as far as the
    other posts' bending holds the inner plate (`cradle_structure.posts`, below under
    "Loads and checks").
  - Every printed ring fills its layer (`construction.axle.full_rings`), so every column's
    bought length is the gap between its plates. Touching rings print as one sleeve.
  - On the hinge the printed parts and the cradle link stop 0.3 mm short of the standoff
    (`pivots.standoff.RUN_PLAY`): the plates clamp the steel, and the link turns.
- **The sheets.**
  - Links: 0.125 in 5052, the cradle's default sheet.
  - Frame plates: 0.187 in 5052, 3.4× as stiff across as 0.125 in, and still cut with the
    screws' holes. In 0.250 in, SendCutSend's least hole would be 6.35 mm.
  - The rod: 0.250 in 5052 (`materials.CRADLE_ROD_SHEET`), so it doesn't buckle.
  - The crank block: 0.125 in 6061 (the keyways' 7.75 mm slots are legal in it).
  - B's links are 0.187-0.190 in: its arms b2 and b3 and its bar b4 in 7075-T6 (a 6 L's
    jam: 6061 held 0.83; SendCutSend's datasheet, yield 69 ksi), b5 and its cradle link b6
    in 5052 (b6 was 0.125 in until 2026-10-14); its 336 mm rod is 0.250 in 5052, the rod
    and b6 each alone in its layer.
  - Some chords are wider than 24 mm (`config.PRODUCT_CHORD_R`): B's b6's rear arm X-Tr1
    (40 mm) and its chord X-Y (32), A's b2's E-Tr1 (32).
  - The metal is cut at SendCutSend: `materials.LINK_SHEETS` and `config.PRODUCT_*`. Its
    cutting, the 7075's and the birch's too, is a fit to SendCutSend's live quotes of
    2026-10-14 (`hardware.bom.CRADLE_CUT_FITS`; "Building one").
  - The cradle's links are 24 mm wide (`config.PRODUCT_*`'s `link_radius` 12, from 22): B's
    336 mm rod in 0.250 in then holds 1.6 x its jam force in buckling (at 22 mm, 1.46), and
    the bigger pins' holes keep a sheet's thickness of web.
- **The stand** (`cradle.STANDS`).
  - **Outer plate:** a truss outside every moving part's layer: from the hinge G down in
    front of the decanter to the front foot and back to the crank O, O's legs down and
    back, a rail along the base. Its inside corners are rounded 20 mm (`Stand.fillet_r`:
    the outline grown and shrunk back, `cradle.plate_part`), so the joints read as one
    shape. Both sides' outer plates are the same file.
  - **Both plates' outlines** (design review round 3's laser-cut minor 5): on B
    (`cradle.TAPERED`), where a boss, or a member running straight on (within
    `cradle.STEP_TURN`, 30°), stands at most `cradle.STEP_MAX` (4.5 mm) proud of a strip,
    the strip runs as the hull of the two stations' discs, and from a proud station into a
    plain strip as a straight taper of 1 in `cradle.TAPER_RUN` (`cradle._tapered`): the
    closing had left S-shaped 2.5-4 mm steps in straight edges (two on B's inner leg D1-O2).
    A taper that would reach the inner plate's keep-outs keeps its step; a designed widening
    (B's 64 mm member at O2) keeps its shape. B's inner plate gains 157 mm² a plate. A keeps
    its steps (three on its outer plate, its inner plate's columns) until the pack's turn 3:
    tapered (601 and 246 mm² more), its flat pack's printed spacers lost their room, one
    grown to a 327 mm print and one left loose 66 mm. Every convex corner on both is rounded
    `cradle.EDGE_R` (3 mm: an opening, after the board's cut, which had left a sharp point
    where it met an R28 member's end at B's rear foot).
  - **Inner plate:** a block round the hinge, and a leg over the outer plate's front leg
    down to about 200 mm over the table, tied to the outer plate by five columns (the
    fifth, D3, since 2026-10-14: the outer leg takes 2 kN jammed, and from D2 to the foot
    it buckled out of its plane at SF 0.6). Since the side frames' redesign (2026-10-20) a
    40 mm leg runs on from D3 down to a sixth column, E1, on the rail 80 mm in front of
    the mouth, so the posts no longer stand on one line ("The side frames at the clutch's
    cap").
    - Each side is then a deep box through its front leg.
  - The strut from the hinge G back to the gearbox at O is 96 mm wide (`Stand.wide`; 48
    until the redesign), the legs from O 40 mm and the rail 40: O is held across the plate
    only by its members, the legs from O leaned on the strut, and with them it takes the
    gearbox's turn out of the plate there.
  - Nothing crosses the bay over the mouth.
- **The base board** (`cradle._base_plate`, the part `base_plate`): 9 mm Baltic birch
  plywood, routed by SendCutSend (`birch_9mm`, the user's of 2026-10-13), 584 mm across
  the bay (`cradle.BASE_HALF`), 684 mm long on B and 682 on A (`Stand.base`: a full 6 L
  tips past 25° over every edge of B's; A's rear edge 23.9°, its bar 15°). B's sits 40 mm
  toward its drive side (`cradle.BASE_Z`, round 1 of 2026-10-14: the gearbox and the hand
  crank put the cradle's centre of mass 68 mm that way). Its corners are rounded 35 mm and
  the notch's mouth 8 mm (`cradle.BASE_CORNER_R`, `NOTCH_MOUTH_R`): a square plywood corner
  chips and catches a shin. SendCutSend publishes no plywood price; the BOM's figure is a
  fit to its live quotes of 2026-10-14 (the board alone USD 140.39 then).
  - **Glued, then finished** (design review round 2's M6, 2026-10-14). Wood glue bonds bare
    wood: "Most of our glues are designed to bond bare wood. Painting or staining a wood
    blocks the pores", and over a finish a bond "will only be as strong as the bond
    between the" finish and the wood (Titebond's FAQ,
    <https://www.titebond.com/resources/use/all/faqs>, fetched 2026-10-14 by the review). So
    the board stays bare through the build: the riser locator's layers are glued at once,
    the cleats once both sides stand on it; then every face and edge of the board, the
    cleats and the locator takes polyurethane (the guide's last build step). Until round 2
    the guide sealed the edges and oiled the faces first, then glued the cleats onto the
    oil, and an oiled face is no barrier to red wine under the pour.
  - **A window under the base stop** (`cradle.BOARD_WINDOW`, round 2's M7): 105 x 210 mm
    with 20 mm corners, 285 to 390 mm behind the mouth across the bay's middle. At rest,
    set for a 6 L, the base stop comes down to 26 mm over the board, a finger's 25 mm (ISO
    13854), and less than the 24 mm cork just pulled and laid there; over the window
    whatever lies there lies on the table, 43 mm under it (`cradle.finger_gap`, the clash
    test's bar 40). It cuts no cleat, bolt or foot; 159 mm of birch stand between it and the
    notch and 134 mm behind it (A: 92). Its 135 g of birch, with round 2's NRV050, took
    B's 684 mm board's rear edge to 25.0° to the tenth: the board grew 2 mm at the rear, to
    686 mm (25.2°; 25.5° with round 2's heavier cradle box).
  - **The cleats** (`cradle._cleats`, the parts `base_cleat_*`; round 1): two 26 mm strips
    of the same birch glued to the board either side of each outer plate's rail, between
    its brackets, against each face (`cradle.CLEAT`: 0.02 mm of play, the glue-up's; 0.15,
    a sheet of paper's, until design review round 3's M6). They hold each side upright on the
    board, where its two printed brackets alone let it sway several millimetres under a
    20 N push at the mouth, and they drop the rails into place when the cradle goes back
    on. They're glued to the cradle as built: the guide pencils each round its rail, slid
    against its faces with nothing between, lifts the cradle off, then glues them on the marks
    (Titebond II, the `wood_glue` line) and sands a 2 mm lead-in at 45° on each one's top
    edge by the rail (`cradle.CLEAT_LEAD`, round 2: the way back lowers 9.5 kg so both rails
    drop into their slots at once, mostly blind).
  - **The brackets** (`cradle._bracket`): both outer plates' rails bolt onto the board with
    printed L-brackets: an M5 screw and nut through the upright and the outer plate, an
    M6 x 25 carriage bolt up through the board and the foot (Bolt Depot 18221).
    - The bolt's hole in the board is a routed 6.7 mm square with a dog-bone relief in each
      corner (`cradle.CARRIAGE_HOLE`, `DOG_BONE_R`: the router's 1/8 in bit), so the square
      neck locks by its shape. The round hole it replaces had the neck's corners crush 1.4
      mm of birch each: a mallet, and split face veneers.
    - The foot's hole is 8.5 mm (`cradle.BRACKET_HOLE`): 1.25 mm of play round the bolt, so
      a side settles where the linkage puts it. B's two remote centres must agree to within
      the joints' play (round 1's linkage review), so the guide snugs the passive side's
      knobs last, with the bottle level (round 2's m6: the forced misfit is 0.68 of the
      joints' play snugged at level, 1.29 at rest).
    - On the bolt go an 18-8 fender washer (6.4 x 18, Bolt Depot 7315) and a printed thumb
      knob (`cradle.KNOB`, 24 mm across, fluted) with an 18-8 M6 nut (Bolt Depot 4776)
      pressed into its hex pocket. The washer spreads the clamp over 200 mm² of the printed
      foot, where the old thumb nut's 12 mm collar had 81. No thread is plastic and nothing
      rusts at a wine bar; the black-oxide thumb nut (McMaster 92815A203) is gone, and with
      it a cart line. (McMaster's 18-8 thumb nuts are 15 mm tall and show no price to a
      script.)
    - The bolt stands 22 mm off the outer plate (`cradle.BOLT_OUT`), so the knob clears the
      upright's M5 screw head by 2.25 mm. An M6 wing nut (28-33 mm wings) would hit the
      upright.
    - Check the knobs are snug before each use. One that keeps working loose has let its
      bracket's foot creep under the washer: snug it, and reprint that bracket in PETG. (B's
      knobs are its studs' and its brackets stay on the board: its "Before you use it" says
      to wiggle each bracket instead, `cradle.use_steps`.)
    - All the bolts go in with the board. On A the passive side is set down 7-15 mm out from
      its place, inside its bolts, and pushed home; on B the whole cradle is set on the
      board, its passive panel built and lowered lying down ("Building one").
    - **B's brackets stay on the board** (`cradle.BOARD_BRACKETS`, the pack's turn 2,
      2026-10-20; the packing review's L1). Each outer plate's foot M5 is a stud: an M5 x 16
      button head from the plate's inside (its head where the nut was, in the layer over
      the plate: the same claim, the plan unchanged), its jam nut on the plate's outer face
      (threadlocker, once: an 8 mm wrench and a 3 mm hex key at the build). The bracket's
      upright is slotted from the stud to its top, 12 mm wide (`cradle.STUD_SLOT`: the jam
      nut's corners and 1.3 mm of play either side, the float the foot's oversize hole had
      given), and a printed knob on the stud (`cradle.STUD_KNOB`, 26 mm across, an 18-8 M5
      nut pressed in) bears on the upright either side of the slot and clamps it to the
      plate by hand. The foot is held down by its carriage bolt, washer and an 18-8 M6
      nylon-insert nut (`cradle.FOOT_NUT`: Bolt Depot 4795, USD 0.11 as the plain 4776 was;
      design review round 3's stand and use, m4), snugged once the cradle has set the
      brackets where its sides want them, to 1.5 N·m: the 10 mm wrench pulled by the
      luggage scale 100 mm out, to 1.5 kg. That is 1.25 kN, 6.3 MPa on the printed foot
      under the fender washer, half the sustained PLA bearing rating (a wrench's "snug", 2-5
      N·m, had been 8-21 MPa, and nothing held the nut: a bracket turned on its one bolt
      misses its stud next trip, and with all four studs loose the cradle tips about its own
      rails, 17.5° and 23.5 N where the board's feet give 30.0° and 53.4 N). B's brackets
      print in PETG: they ride in the car on the board. No knob turns at the board again.
      The upright's slot has its mouth chamfered 2 mm at 45° (`cradle.SLOT_LEAD`, round 3's
      m11), so a stud lowered 1.4 mm off finds it. The knob, 5.5 mm thick,
      sits between the upright and the foot's nut, 5.7 mm clear of its corners; it couldn't
      beside the thumb knob that was there (that knob's edge was 5 mm off the upright, and the board's
      edge 3.5 mm past it on the passive side). So the cradle lifts straight off the board
      with its four knobs loosened a few turns (its studs, jam nuts and knobs with it), and
      goes back with each stud dropping into its slot; the brackets, their bolts and nuts
      stay on the board. Turned down onto its jam nut for the trip, a knob stands only the
      nut's 4.0 mm off the plate, where the 5.0 mm upright must pass: the way back backs each
      off two turns first (`cradle.KNOB_BACK_TURNS`: 5.6 mm, the knob's nut still all on its
      stud; round 3's M1: "a turn back" after lowering had left the cradle hanging on its four
      knobs, 29 mm up and out of its cleats, every trip). B's drive side gets its studs on
      the bench, lying on the riser's blocks, and is lowered into its brackets' open slots
      (round 3's M3: put on standing, 8.06 kg on a 4.75 mm edge, it had needed three hands,
      and its jam nuts a wrench inside the 12 mm slot). The passive panel lying on its outer plate rests on pillar O1's
      screw head and its two studs (0.9° tilted, it tips at 64.1° and 19.8 N, where its
      30 mm brackets had rocked it at 3.4°: the review's m4). Per bracket an M5 nut more
      (the jam nut, $0.07: $0.28 for B) and the printed thumb knob traded for a printed
      stud knob. A keeps its brackets with its panels (with the brackets on its board its
      panels can't lie flat: 100.7 L, 89.6 now).
  - **The riser locator** (`cradle._riser_locator`, the parts `riser_locator_0` and
    `riser_locator_1`; round 1's jack locator): two layers of the board's birch glued face to
    face. At 18 mm it meets the board's whole edge (the board stands 8 mm up on its feet).
    Its outline is the notch's, 1 mm in, from 70 mm in front of the mouth to the notch's
    round end, with an ear into each of the notch's two side bays (`cradle.NOTCH_BAY`), so it
    can't slide out forward. Its 101 mm square pocket, dog-boned, takes the printed riser's
    100 mm base under the mouth (`cradle.RISER_BASE`): in the bare notch a lab jack had
    floated 76 mm either way, and the stream lands within 16 mm of the axis. It lifts out to
    stand a vessel on the table.
  - **The riser** (`cradle.riser_pieces`, the parts `decanter_riser_*`; round 2's B1 and M7,
    in place of the lab jack round 1 had left unbought, unspecified and unpriced): printed,
    fixed heights, stacked in the locator's pocket. Bottom up: two lifts side by side
    (`cradle.RISER_LIFT`, 100 x 49.8 x 27.2 mm), two blocks on them (`cradle.RISER_BLOCK`,
    100 x 49.8 x 85 mm), both cups open underneath, then plates of 5 and 4 mm
    (`cradle.RISER_PLATES`; 20, 10 and 5 before the lifts); each piece's 2 mm 45° boss
    (`cradle.RISER_LIP`) rides up into the piece over it with 0.5 mm of play, so the stack
    can't slide apart, and each prints upside down with no support. The whole set is 123.2
    mm (`cradle.DECANTER_RISER`, the drawn RIEDEL Ultra's). It can't be raised mid-pour
    without lifting the decanter off it: that closes round 2's linkage review's
    raised-decanter case physically.
    - The blocks, each on its lift, are also the drive panel's lay-flat blocks on the trip
      apart and back (`cradle.LAY_BLOCKS`, "Flat pack"): 114.2 mm with their boss, as far as
      round 2's NRV050 stands off its outer plate (114.7), and a relief in each boss's
      middle takes the screw head it sits under (`cradle.RISER_RELIEF`, design review round
      3's stand and use, M5 and m2): a 13.5 mm bore through the block's top into its cup
      (the heads stand 4.9 mm proud on their 12 mm washers; the 12 x 3.6 mm relief it
      replaced took neither, so the plate had stood 1.3-1.6 mm off the boss), under a 30°
      cone 26 mm across, so a head set down within 7 mm of a block's middle slides home. Two
      jobs, no part more to store or lose.
      (The stand round's blocks stood 87 under the NRV040's 87.5. Blocks 27 mm taller
      instead, the user's first call of 2026-10-15, would have left the riser over the
      drawn decanter's rule with the old plates, or with plates thin enough for it, three
      listed vessels with no stack clear of the locator: the plates alone stop at 11 mm.)
    - The stacks it makes (`cradle_report.riser_stacks`): any of the lifts, the blocks and
      the plates in that order, 6 to 38.2 mm without the blocks and 87 to 123.2 on them.
      Every listed vessel's stack is within 7 mm of the stand round's (the RIEDEL Ultra
      Magnum and Zalto Axium 114.2 against 117, Le Château 96 against 102, the drawn RIEDEL
      Ultra 123.2 against 122) and within 7 of the rule's most but for the Zalto Burgundy
      (38.2 against 73, 78 before round 2's 12 mm tie rods took up to 2 mm off the room under
      the cradle box; lower is safe).
    - About 353 cm³ of PLA in all (the lifts 43 cm³ each, the blocks 92, the plates 46 and
      36; 350 before the lifts).
  - Four printed TPU feet snap into its corner holes (a cone and a bore up the stem). On a
    counter that isn't flat they take up about 1 mm; past that it rocks, and a thicker foot
    goes under the corner that lifts.
- **No toggle flag** (the user's call of 2026-10-15): the stand round's red flag on B's
  outer plate (its arc 108-128 mm round O over the rest's toggle zone, two M5 screws through
  the rear leg) went, with its holes and their claims, for one cue on both cradles at both
  toggles: the paint marks across the gearbox's output hub and housing ("The drive": the
  toggle zones' marks; `handcrank.TOGGLE_TURNS`).
- **The light:** a 14 x 60.5 mm AAA keychain light (Olight i3E EOS) stands in a printed
  clip under the front saddle's back end (its flange on the carrier's underside, two M5s),
  so the clip prints upright and round. Its head sits 100 mm behind the mouth
  (`cradle.LED_X`), under where a 750's shoulder starts and a 6 L's neck meets its shoulder
  (since round 3: at 118 univerre's real 6 L impériale, its shoulder 27 mm fuller there than
  the model's, lay on its head); an M3 set screw holds it while you twist the head to switch
  it on.
  - A longer light's tail reaches a decanter's shoulder at rest.
  - Its printed clip is the lowest part of the cradle box over the decanter's rim: 394 mm
    over the table, near rest ("The vessel under the mouth"; 380 before round 3 moved it).
- **Display only** (`fab="display"`: drawn, never bought, cut or printed): the bottle (a
  6 L methuselah, `cradle.SHOWN`: the V blocks drawn at their lowest; the cradle page sets
  them for the bottle picked) and a RIEDEL Ultra decanter (225 mm, `cradle.DECANTER_PROFILE`)
  on the printed riser, the whole set (`cradle.DECANTER_RISER`, 123.2 mm), in the riser
  locator's pocket.

## A, redone (2026-10-10)

The user asked for a rest nearly upright (−50° to −60°). A's crank-rocker was re-synthesized
for −50° to +30° (`decanters.decanter_hinge`): the two rocker positions fix the crank and
the rod for any crank centre O, and a scan over O and the rocker picked the one that meets
every bar at once.
- O 285 mm under the mouth and 85 behind it (175 mm over the table), the rocker 140, the
  crank 89.5, the rod 267.3; the worst transmission angle 38.4°.
- +30°, not +45°: a 95° swing put the gearbox at SF 1.13 for a sealed full 3 L; 80° keeps
  it at 1.32.
- The rod's length is the binding bar: a longer rod buckles in the jam (SF 1.19 at 298 mm,
  1.52 at 267); the best transmission (47°) wanted the longer one.
- The mouth rose to 460 mm and the bay widened to 260 mm (`cradle.LIP_HEIGHT`,
  `cradle.BAY`); the saddles stayed 160 mm long, centred on the rods. (The 6 L raised the
  mouth to 550 mm and the saddles to 190: "Sized for a 6 L".)

## Sized for a 6 L (2026-10-13)

The user asked for 6 L bottles and, asked which, answered "both": a Bordeaux impériale
(Saverglass Bordelaise Tradition 600 cl: 493 x 164.5 mm, 3.1 kg empty) and a Champagne
methuselah (Saverglass Standard 600 cl: 571 x 174 mm, 5.1 kg empty, 11.1 kg full). The
methuselah, fattest, longest and heaviest, is the design bottle (`cradle.DESIGN`); every
other size lay on adapters (adjustable saddles since 2026-10-14: "The saddles,
adjustable"). What it changed, and why:
- **The mouth 550 mm over the table** (from 460): at rest, 50° neck up, the 6 L's base and
  its base stop hang about 515 mm under the mouth. The stand's legs are 90 mm longer; the
  linkages are unchanged relative to the mouth.
- **The saddles and ties** (`cradle.SADDLES`, `decanters.TIES`): Vs for a 174 mm body and a
  48 mm neck; the rear saddle 125 mm long, reaching behind the 6 L's centre of mass; the
  tie points lower and further back. Both cradles re-plan round them.
- **The drive:** a full 6 L takes about 32 N·m at the crank (B; A 34), 2.2 x the 3 L's. The
  NRV030 (20 N·m) became an NRV040 (45 N·m: SF 1.41 on B, 1.33 on A), and the handle grew
  to 100 mm (A 105), so the worst turn stays under 25 N at the grip. (Corrected 2026-10-14:
  29.5 and 31.3 N·m, SF 1.52 and 1.44. The load case had kept the drawn magnum's 1.25 kg of
  adapter liners: DECISIONS.md. Round 1's drive: 125 mm on B, 85 on A; round 2's: an NRV050,
  85 N·m, and A's 130 mm crank on a 230 mm reach, "The drive" above.)
- **The jam grows with the handle.** A firm 40 N push on a 100 mm handle through the worm
  at a hand's speed is 66 N·m at the crank (A 69.3, then the jam case), where it had
  been 36. Whatever the ratio or the handle, a jam push about twice the worst turn's puts
  the drive path at about twice the bottle's peak: every part between the crank and the
  cradle needed 1.4-1.9 x its strength. (Round 1 of 2026-10-14: a slip clutch in the hand
  crank now bounds it, "Loads and checks".)
  - The pillars and the crankpin: 12 mm NETRF12 (1.73 x the 10 mm's bending section).
  - B's arms b2, b3 and bar b4: 7075-T6 0.190 in (1.72 x 6061's yield).
  - The rods: 0.250 in 5052 on both, and every link 24 mm wide (their buckling; a 0.313 in
    rod failed SendCutSend's web rule round its pin).
  - The crank block: four keyed plates on the NRV040's 19 mm shaft and 6 mm key, as
    counted (all seven under the seat were slotted at the key's width); three on the
    NRV050's 25 mm shaft and 8 mm key since round 2's drive, the three over them slotted
    clear.
  - The pins: steel ("The pins" above), the user's choice of 2026-10-13: an M6 Chicago
    screw's 8 mm stainless barrel held 0.69 at B's rod pin P.
- **The stand:** the base board 530 x 626 mm (from 500 x 596): a full 6 L tips past 25° over
  its sides and its rear edge. (Corrected 2026-10-14: the stand's check had counted the
  spare adapters lying on the table, about 3.8 kg, as its own weight. Without them it
  tipped at 21-20°, so the board grew to 584 x 682 mm on A and 584 x 684 on B.)
- **The base board in wood** (the same day's ask): 9 mm Baltic birch, routed.

## The saddles, adjustable (2026-10-14)

The user's choice: "Each saddle's V rides up or down on a thumbscrew, with a scale marked by
bottle size. A flip-down lip gauge marks where the mouth goes, and the existing base stop
locks the bottle there. The top stays open, so bottles still lay in from above. Two knobs
plus the stop replace all the adapters." The 10 printed adapter liners and collars are gone.
Design review round 1 (the same day) found it didn't yet work: a full 6 L tipped nose-up
off the front V at rest, setting a big bottle took three hands and a hex key, and the rear
knob took 55 turns and bent at a knock. Round 2 (the same day) found the rear V block
jammed raising it, the scale and the lip gauge too fussy for real bottles, the rails
rocking in their sockets, the lead screw's brake undone by its own grease and the printed
parts rated on a wrong figure. What changed is under "Review round 1" and "Review round 2"
below; the rest of this section describes the saddles as they now are.

**The setting** (`cradle.setting`): a bottle lies with its axis level through the pivot
when each 90° V's apex is `(r + 2) √2` under the axis (`r` its radius, 2 the liner), so
each V block rises `√2` times what the neck or the body is thinner than the 6 L
methuselah's. The base stop's face goes 2.5 mm behind the base (`cradle.STOP_GAP`): at
rest the mouth lies 2.7 mm behind the lip gauge's pad, and past level the bottle slides
onto it.

| bottle | front V block rises | rear V block rises | base stop's face behind the mouth |
|---|---|---|---|
| 750 ml Bordeaux (Saverglass 4427, 300.5 mm) | 12.7 mm | 68.9 mm | 303.0 mm |
| 1.5 L magnum | 12.0 | 53.0 | 358.5 |
| 3 L jeroboam (Saverglass Jero Vignoble 7201) | 7.1 | 23.3 | 472.5 |
| 6 L impériale | 0.7 | 6.6 | 495.5 |
| 6 L methuselah | 0 (on the rods) | 0 (on the rods) | 573.5 |

These are examples: two bottles of a size differ (Saverglass's 750 ml Bordeaux run 300.5
to 315 mm long, its Standard 3 L is 474.3 x 135.2 mm against the Vignoble's 470.3 x
140.8), so every bottle is measured and set by its own length, neck and body. The front V
block travels 15 mm, the rear one 72: a neck from 28 to 48 mm across, a body from 76 to
174.

**The mechanism.**

```
  front saddle (looking along the axis)        rear saddle (looking along the axis)

    bar  gauge's pad       knob (M6)                                post, and lead screw
     ||   ____             _|_                         6 L        27 mm behind it: its knob
     ||  |pad |  .-.      |#|                        .-'''-.          _[|#|]_ <- teeth, arm
   __||__|____| ( O ) ____|#|_ <- tab + M6 nut      /       \       ___|#|__|_
  | cheek   \__  \ /  _/  |#| |                ____ \  axis /  ____|  bosses| | <- tower
  |  wall    V  block     |#| |               |    \ \____/  /|    |(nut)   | |
  |___|___________________|_|_|               |_____\  V    /_|____|___||___|_|
   ===o========o===  tie rods                  ===o===\block/=====o==[collar]=== tie rods
```

- **Front:** the V block slides in a box (the carrier's walls, the front one notched under
  the lowest V, the back one 6 mm deeper; two drains through the back one at the floor, its
  low end at rest). Its V and liner bear on the neck from 48.3 to 64.5 mm behind the mouth
  (`cradle.FRONT_BEAR`); behind that the V is 6 mm lower (`cradle.FRONT_RELIEF`, a 1 mm floor
  kept), so the V holds a neck only in front of its shoulder (design review round 3,
  mounting M1: borne to 79 mm, Saverglass's 4427, which flares from 71, lay on its
  shoulder, set by its neck 3.9 mm low, and slid onto the pad its shoulder climbed the
  back wall). Its tab at the +z end holds an M6 nut. The M6 x 60 tap bolt stands in a blind bore
  in the carrier's floor, through the nut, its printed knob on its head over the wall. The
  knob stays put: turning it walks the nut, and the block, up or down (13 turns end to
  end). The block bears down on the nut, so the thread (self-locking) holds it.
  - The push at the tab is 40 mm off the box's middle, so it cocks the block onto the -z
    wall's top and the +z wall's foot, and their friction grows with the push: a drawer.
    The -z wall stands 12 mm taller (`cradle.FRONT_GUIDE`), a cheek up the block's -z face
    along it, and over the lip gauge's post the cheek stands 10 mm taller still against the
    post's face (`cradle.FRONT_CHEEK`, round 3's mounting m9): the pair 55 mm apart at every
    bottle's setting (45 before, 32 before round 2), the block jams raising only past
    friction 0.69 (`cradle_report.drawers`; 2.3 x the most cited for printed PETG on steel,
    dry: the guide's grease helps, but a spill washes it out; it had held 1.9 x dry).
- **Rear:** the V block's two bosses at its +z end ride a 5/8 in 6061-T6511 post (Speedy
  Metals, cut 6 in, faced to length, its flats milled and its foot tapped M6 on the
  Carvera; a 15 mm nickel-plated steel standoff, MISUMI PETR15-150, until simplification
  round 1), and 27 mm behind it a printed lead screw (`cradle.LEAD_SCREW`: PETG, 13 mm, 4
  starts, 8 mm a turn, 9 turns end to end; a ServoCity Tr8 x 8 stainless screw, bronze
  barrel nut and aluminium collar until then), whose printed hex nut floats in its boss's
  hex pocket from below (`cradle.LEAD_FLOAT`: its flats keep it from turning). Both stand
  in the carrier's +z side block: the post in a socket 30 mm deep (a collar round its top),
  an M6 button head up into it; the screw's foot's spigot in a bore, its shoulder on a
  printed thrust ring in a pocket on a TPU washer. A tower on
  that block rises beside the screw to an arm over its top: a bushing round the screw, and
  a star of 12 teeth on it under the knob (`cradle.LOCK_TEETH`).
  - The screw pushes the block up beside its guide, so it can't cock it: raising it jams
    only past friction `engagement / 54` (the boss's 53 mm on the post's round, between
    its wrench flats: 40.1 mm at a 750 ml's setting): 0.74 to 0.98, against 0.22-0.30 for
    PET on bare aluminium dry at room temperature (`cradle_report.PETG_ALU`: the only cited
    figure, an undergraduate paper's, so the guide's bench check matters; 0.24-0.30 for
    printed PETG on steel before), 2.44 x at a 750 ml's setting (`cradle_report.drawers`, a
    row per bottle in `saddles`). (The post had been at -z, 213 mm across from the screw: it jammed past
    0.15, 0.095 at a 750 ml's setting: review round 2, N1.)
  - An 8 mm lead may not hold itself (the printed thread's lead angle is 11.9°; PETG on
    PETG's friction is unmeasured, none counted). The knob locks it: it slides up and down
    a hex hub set-screwed on the screw's top (on a flat printed there; filed on the steel
    screw from round 3), and
    down, its underside's star socket sits over the arm's teeth, so the screw can't turn
    (`cradle._lead_screw`). Lifted 3.6 mm off them, it turns the screw; let go and pressed
    down, it locks it again, a twelfth of a turn at most from where it was (0.7 mm of the
    block, 0.2 mm at the mouth). The teeth carry the thread's turn back under a hung 6 L,
    frictionless, at 0.6 MPa. The thrust ring's TPU washer under the block's load still
    slows the screw with the knob lifted (3.1 x its turn back, dry; 0.6 x greased: not a
    hold). The thread runs dry. Turned with the bottle out, the knob takes about 93 N·mm
    (7 N at its rim).
  - The screw holds the block at its +z end. A bottle's weight at the axis rolls the block
    about it, and the post takes all that couple (the nut floats, so the screw takes
    none): 5/8 in 6061 takes a 6 L hung a hair off the rods and the parts riding the block
    at 7.1 x its 241 MPa least yield (the steel 15 mm had taken its share at 5.7 x; 12 mm,
    round 1's, 2.99 x). Its friction also makes a loaded block hard to drive down.
  - The printed screw is knocked at the knob at SF 2.87 across its layers (bar 2.5: a 50 N
    knock midway between the nut and the bushing, on its thread's root); its thread bears
    at 0.5 MPa, the nut on its pocket's ceiling at 0.8 MPa under a hung 6 L. A 24 h bench
    hold is the guide's: a 12 kg hang, the knob down, dry and greased, the block mustn't
    drop 0.2 mm. Its thread is drawn in its print file only (`meta["print_shapes"]`): the
    assembly holds its envelope, so the clash checks stay quick (the helical solid had
    taken B's from 9.6 to 22 s an angle, its bottles' hits from 10 to 55 s).
  - The screw's knob sits at |z| 110, 10 mm outboard of a 6 L's widest as it's laid in.
    A knock at the knob goes into the tower's arm, not the screw, which is held at both
    ends (`saddles`: 5.8 MPa across the tower's layers at 50 N).
  - A gusset under the +z flat, along both bosses, spreads the couple into the block. The
    carrier's -z side block only rides the rods (15 mm wide), and the V shell is 7.5 mm
    thick: what the 15 mm post adds in steel they take off in plastic, so the stand still
    tips past 25° (B's rear edge, 25.1° on round 2's 684 mm board).
- **The scales:** grooves the V block's top edge (front) or its screw boss's top (rear, on
  the tower's face) reads. One row has a groove per listed bottle, a dot each per place
  from the top: in front three (`cradle.FRONT_GROUPS`: the 750 ml and the magnum share one,
  the two 6 L another; their necks rise the block 0.7 mm apart, so each pair is set at the
  mean, 0.45 mm off at the mouth), at the rear five. The other row is graded in diameter:
  the neck's every 4 mm in front, the body's every 10 mm at the rear: a bottle is set by
  its own (measured). No text is printed on them: glyphs would depend on the fonts of the
  machine that builds the STL.
- **The base stop's scale** (`cradle.stop_marks`): a plain length scale across the +z flat
  and rail, a groove every 5 mm of a bottle's length (a long one every 10; one dot in front
  of each 50 mm groove, three, four and five at 300, 400 and 500), the gate's face 2.5 mm
  behind the length it reads. The gate's pointer reaches forward along the scale's outer
  half with a vernier (`cradle.VERNIER`): four notches 6 mm apart in its inner edge, the
  k-th from the face meeting a groove for a length k mm past one, where both reach the
  rail's top. (It had been a named groove per bottle: review round 2, N2. The notches had
  been lines cut in the pointer's top, 6 mm over the grooves, and the pointer 2 mm wide,
  printing as a column: round 3's mounting m4 and m2; it is 4 mm wide, its outer edge over
  the rail's outer face.)
- **No depth added:** each carrier's bottom is the old saddle's (front -60, rear -149).
  The V block sits on the rods at its lowest, and its guides stand beside the bottle (|z|
  88 to 126 at the rear, inside the bay's 130). `decanters.TIES` didn't move: neither
  cradle re-plans.

**Resting on both saddles** (`cradle_report.balance`; review round 1, B1). At rest the gate
pushes the base back below the axis while the weight pulls along it; that couple moves the
centre of the saddles' reaction toward the base by the push's depth times tan 50°. With the
gate's push as low as it can be (the base's ring or the gate's bottom edge) and the centre
of mass where review round 1's shell model puts a full bottle (`BottleSize.com_from_mouth`
since that round, 31-36 % of its length from the base: 390 mm from the mouth for the
methuselah), the reaction centred up to 482 mm from the mouth, 102
past the rear V's end: the neck lifted off the front V and the bottle pivoted on the V's
back edge. The base stop's rails now carry 45° flanges continuing the V behind the block,
lined with TPU, so the rear support runs to 580 mm (the body's heel, 10 mm short of its
base, ends it for each bottle):

| bottle | at rest | level | full tilt | its support's end | the rails' sag lifts the mouth (at rest, level, full) |
|---|---|---|---|---|---|
| 750 ml | 224 | 192 | 199 | 290.5 | - |
| magnum | 276 | 230 | 238 | 346 | - |
| 3 L jeroboam | 397 | 325 | 334 | 460 | 0.03 mm, -, - |
| 6 L impériale | 405 | 319 | 329 | 483 | 0.10, -, - |
| 6 L methuselah | 482 | 390 | 400 | 561 | 0.89, 0.14, 0.23 |

The reaction's centre (mm from the mouth); every one at least 62 mm inside its support. At
full tilt the lip gauge's push over the axis moves it back too (by the rim's height times
tan 30°). The rear V block is rigid to its back end, 380 mm from the mouth; behind it the
rails' flanges bend. A centre behind 380 rests the bottle on that end and the flanges'
end, the flanges taking at most `W (x - 380) / (end - 380)` (39 N for the methuselah at
rest); each rail, a cantilever from its socket, sags under its half and the bottle tips
about the block's end, lifting the mouth (`cradle_report.balance`, at most
`cradle_report.MOUTH_LIFT_MAX`, 1 mm). As the pour moves the centre in front of 380
(about level for the methuselah) the lift goes smoothly back to nothing. (It had been
taken as rigid; the rails' tongues rocked 0.6° in their sockets and the rails were half as
stiff: 1.5 to 4 mm at the neck, review round 2, N3.) Each rail joins the block by a tongue
under its flat, 29 mm into a socket: crush ribs along its top (`cradle.RIBS`, 0.2 mm of
interference) press it down onto the socket's floor, where its two M5s hold it, so it sits
with no play and its top flush with the flats. A keel down its outer face to the flange's
bottom (`cradle.RAIL_KEEL`) doubles its stiffness (61 000 mm⁴, was 30 000), and the
flange, the keel and the tongue end level at -91 mm, so it prints standing on them.

**Why not the tie rods in vertical slots** (the first idea): the slot would have to run
the whole travel under the rod, so the rear block would hang 69 mm deeper at a 6 L's
setting. That is onto the decanter's axis at rest, 217 mm over the table. The front block
would hang 8 mm deeper. The rods can't go higher: they already sit just under the 6 L's V
apex.

**Why a coarse screw at the rear, a fine one in front:** the mouth's height follows the
front V 1.25 mm per millimetre and the rear one only 0.25 (the rear V is 4 x as far from
the pivot), so the rear needs coarse steps and the front fine ones. The M8's 1.25 mm took 55
turns for a 72 mm travel that buys nothing a 8 mm lead doesn't; the front M6 stays (13
turns). Detents (review round 1's pins in a row of holes, S1) would have been simpler still
and saved the screw, but the user's choice is a V on a thumbscrew, and a 5 mm detent pitch
leaves a front V up to 2.5 mm off (0.8 mm at the mouth).

**Why the gauge is the forward stop:** with no collar, a stop must hold the bottle from
sliding toward its mouth past level. A lip over the base's heel can't: the heel tapers
toward the base, and nothing on the bottle faces the mouth but the shoulder and the
mouth's own face. A shoulder stop would need a third adjustment, 45 mm along the axis
(and the light stands there). The gauge is at the pivot by construction, for any bottle.
Its pad bears on the top of the rim, over the bore, 3.25 mm down a 750 ml's rim (its ring
27.5 mm across) and clear of a methuselah's 20.5 mm bore; the stream leaves the bottom of
the bore, and the decanter is below. (An impériale's bore, 31.5 mm across, has its top
5 mm under the pad: the first of a pour runs down the pad's back into the stream.) Its
bar runs back beside the neck, so the bottle lays in under nothing; a bottle's push on the
pad is 41 mm off the bolt's line, and the gauge's two cheeks either side of the post take
that turn on the bolt (the push itself bears the back cheek on the post's back face, not
on a thread).

**The slide onto the pad** (`cradle_report.gauge_slide`, round 3's mounting m1): at rest a
bottle lies on the gate, its rim 2 to 4 mm behind the pad. A 90° V's two faces press it with
√2 its weight, so its liners hold it past level at a friction of tan(tilt) / √2: dry TPU on
glass, over 1, never slides in the swing (that needs under 0.41 at +30°). Wet (a sweating
bottle, liners not dried), it can, and its friction falls to TPU's wet kinetic 0.30 (pure
TPU in water, Yu et al., Polymers 17(1):30, 2024): the difference, 14 N for a full 6 L
methuselah, drives it the 4 mm into the pad, the gauge a 25 N/mm spring (its bar bent by
the push off its line), peaking at 70 N against the 54 N static pull. The gauge's parts
are rated at the peak; its post, 21.7 mm across since (13.7 before, SF 2.88 at the static
pull), takes it at 4.3 (bar 2.5). And the guide has the bottle and the liners wiped dry.

**Why the gauge is fixed** (review round 2, N4 and m2): it had hinged up to uncork and
fallen back by its own weight, a turn only 1.3 x its hinge's friction at rest (0.8 N·mm
against 0.6 at a printed hinge's 0.3): dried wine, or a nut done up a little tight, would
hold it up, and a bottle cranked past level with it up slides forward up to 45 mm. And
uncorking in the cradle was three tasks for two hands (the gauge, the neck, the
corkscrew), the bottle spinning in its Vs. Uncorked standing on the table, the bottle goes
in already open (the -50° rest exists for that), and the gauge needs no hinge, no stop and
no return. Its bridge sits flat on the post's flat top, the bolt 6.5 mm under it: to turn
about the bolt one edge of the bridge would have to sink into the post
(`cradle.gauge_fixed`); a 50 N knock on the pad bears 306 N on the post top's edge, 8.3
MPa. A bottle set too long can't lie in under it (its rim meets the pad: move the gate
back), where the hinged gauge rode up onto the rim. A 50° stop (about 3 x the friction)
or a TPU spring would have kept the hinge.

**Why the base stop rides the V block:** a 750 ml's base (300 mm) and a magnum's (356) lie
inside the rear V (255-380), where the old spine couldn't reach. A gate across the V,
keyed to the block's flats and continued by rails behind it, serves them and the 6 L's
(571) alike, and the rails, rising with the block, give the V's flanges behind it the same
height at every setting. It drops the 155 g spine; at rest it comes no nearer the board
than 26 mm (the old rails' end had been 18.8 mm over it, under a finger's 25 mm).

**Use** (the guide's "Setting it for a bottle"):
1. Measure the bottle, every one: its length standing on the table, base to lip, to the
   millimetre; its neck's diameter 62 mm down from the lip (`cradle.NECK_AT`: the front V's
   bearing's back end, the bottle on the gate; in front of every traced shoulder, so a
   neck's widest under the V; round 3's mounting M1: measured at 80, a steep Bordeaux's
   shoulder, 2.4-2.9 mm wider in radius there, set the V 3.9-5.1 mm low); its body's 255 mm
   from the lip, where it lies on the rear V's front edge. Uncork it there, standing on the
   table, and wipe it and the V blocks' liners dry.
2. At rest, the bottle out, set all three: the gate's face to the bottle's length on the
   5 mm grooves, its vernier for the millimetres, its knob tight by hand, then tug the gate
   toward the saddle: it must not move; each V block's reading edge to the bottle's
   diameters on the graded rows (or a listed size's groove). The rear knob turns only
   lifted off its teeth; let go and pressed down, it locks the screw.
3. Lay the bottle in base first, its label up: its heel on the rails just in front of the
   gate, slid back against it, then the neck lowered into the front V, the rim passing
   behind the gauge's pad (it won't go down if the bottle is longer than set: the gate
   back). Tug the gate again (and before the first turn, after the bottle has rested for
   its sediment: the lock's printed lips creep under its preload, round 3's m6). The pad's
   bottom edge covers the rim's top, 2 to 4 mm in front of it; a strip of paper under the
   neck, either side, is pinched. A body that narrows toward its base (univerre's 6 L
   impériale 82.2 to 79.3 mm, its 3 L too) lies tipped back on it, its neck 3-4 mm over the
   V and its mouth 4 mm high (round 3's m7; harmless: the pad covers its rim by 15 mm):
   lowering the rear V by 5 mm of diameter brings its neck down (its mouth then 1 mm
   high).

**The parts** (prices from pages rendered 2026-10-13 to 2026-10-15, qty 1, per cradle;
the sourcing in `/root/scratch/decant/saddles/sourcing/`,
`/root/scratch/decant/review5/r1_mounting/pages/` and
`/root/.cache/spiderpig/tmp/r2fix/pages/` on the development box):

| part | where | USD |
|---|---|---|
| Tr8 x 8 lead screw, 150 mm, stainless | ServoCity 3501-0804-0150 | 6.99 |
| its barrel nut, bronze, 12 x 16 mm | ServoCity 3500-0804-1216 | 4.99 |
| its clamping collar, 21 x 9 mm, aluminium | ServoCity 3504-0804-2109 | 6.99 |
| M6 x 60 tap bolt, fully threaded, 18-8 | Bolt Depot 5973 | 0.60 |
| M6 hex nut, 18-8 | Bolt Depot 4776 | 0.11 |
| PETR15-150 standoff, 15 mm, nickel plated (the rear post; review round 2, N1 and m3) | MISUMI (2026-10-15) | 19.25 |
| M8 x 12 button head (the post's foot) | Bolt Depot 6654 (2026-10-15) | 0.40 |
| M5 x 16 button heads: the rails' four | Bolt Depot 6634 | 4 x 0.14 |
| M5 x 25 tap bolt (the base stop's lock) | Bolt Depot 5952 | 0.22 |
| M5 nut and two fender washers (in its T-slot, on its ear) | Bolt Depot 4775, 7314 | 0.07 + 2 x 0.10 |
| M5 x 40 tap bolt and nylon-insert nut (the gauge's bolt) | Bolt Depot 5955, 4794 | 0.34 + 0.09 |
| M3 x 6 set screws: the barrel nut's, the rear knob's hub's | Bolt Depot 8707 | 2 x 0.09 |

That is about USD 41.00 (38.50 with round 1's bare NETRF12-130 post and its M6 x 12: the
post and its screw are review round 2's only bought change; the same day a plated 12 mm,
PETRF12-130, is 21.79, a 15 mm without flats, PETRF15-130, 29.43).
Everything else is printed: the two carriers, the two V blocks, the TPU liners (the Vs' and
the rails' flanges') and the rear screw's brake washer, three knobs and the rear one's hub,
the gauge, the rails and the gate; those that carry the bottle in PETG, solid
(`cradle.PETG_PARTS`; on hand).

**What holds** (`cradle_report.saddles`; a sealed full bottle, its worst way for each part,
its centre of mass the shell model's). Each printed part is rated against its cited
strength in its layers or across them as it prints (`cradle_report.PRINTED`: Prusament
PETG's datasheet, 47 MPa in its layers, 18 across them, printed solid; PLA's 51 and 17):
a quarter for a load that stays, 1/2.5 for the pour's or a knock's. Steel against its
yield, 3 x; a grip at the least friction cited (printed PETG on steel: 0.24-0.30 dry,
0.13-0.17 oiled: Stoimenov et al., Tribology in Industry 46(1), 2024), 2 x:

| part | load | stress | SF (bar) |
|---|---|---|---|
| rear V block on its rods' seats | 109 N (a 6 L, level) | 0.28 MPa | 169 (4) |
| rear barrel nut on the block | 119 N (a 6 L hung with what rides the block, 1.0 kg) | 2.2 MPa | 21 (4) |
| rear post (15 mm), bending | 12.9 N·m roll, a 6 L hung at the screw | 39 MPa | 5.7 on 220 (3) |
| rear post at rest | 12.7 N·m: a hung impériale's pull along the axis, 121 mm over its root, and its roll | 38 MPa | 5.7 (3) |
| rear post's socket, 30 mm deep | the post's couple, as bearing | 5.7 MPa | 8.2 (4) |
| rear lead screw | its share of the roll; a 50 N knock between its nut and bushing | 14, 56 MPa | 15, 3.8 (3) |
| rear V block at the post's boss | the post's share of the roll | 0.9 MPa | 54 (4) |
| rear lead screw's lock (the knob's teeth) | 114 N·mm: a 6 L hung, the thread greased | 0.4 MPa | 113 (4) |
| rear lead screw's brake, the knob lifted | the block's load on the collar's TPU washer | 4.1 x the thread's turn back dry, 0.8 x greased | told, not the hold |
| rear knob's tower, across its layers | a 50 N knock at the knob | 5.8 MPa | 3.1 (2.5) |
| the V blocks' guides raised (`cradle_report.drawers`) | rear: 40.1-53 mm engaged, 27 mm off; front: 44.9 mm, 40 mm off | jam past 0.74-0.98; 0.56 | 2.5-3.3 on 0.30 dry; 3.3 on 0.17 greased (2) |
| base stop's gate, bending | 96 N at rest (with a corkscrew's push) | 5.4 MPa | 8.7 (4) |
| base stop's lock | 96 N | its grip 240 N at 0.24 (0.5 N·m by hand, two faces) | 2.5 (2) |
| base stop's T-slot lips | the lock's 1000 N preload at a firm hand's 1 N·m | 11.0 MPa | 4.3 (4) |
| base stop's rails at their sockets | 5.3 N·m each: a 6 L's body on the flanges' end at rest | 1.9 MPa | 25 (4) |
| their tongues where they leave the sockets; the ribs; the sockets' floors | the same | 11.1; 6.1; 6.3 MPa | 4.2; 7.7; 7.5 (4) |
| lip gauge's pad, arm (across), bar, bridge | 54 N at full tilt (no friction) | 1.2, 2.6, 8.4, 5.7 MPa | 40, 7.0, 5.6, 8.3 (2.5) |
| lip gauge's post, across its layers | the same, bent and twisted (its 41 mm turn) | 6.2 MPa | 2.9 (2.5) |
| lip gauge's bolt (M5) and the post's bore | the cheeks' 89 N couple | 42 MPa; 7.5 MPa bearing | 4.9 (3); 6.3 (2.5) |
| lip gauge on its post | a 50 N knock on its pad: 306 N on the post top's edge | 8.3 MPa | 5.7 (2.5) |

The V blocks carry no jam load: the tie rods and the cradle links do.

**What they weigh:** the printed parts of the saddles, the gauge and the base stop come to
about 1.5 kg, solid (1220 cm³: the report counts a printed part at full infill, the parts
that carry the bottle at PETG's 1.27 g/cm³), the 15 mm post 0.20 kg. Review round 2 left
the printed volume as it was (1223 cm³ before: the rails' keels and the knob's hub against
a thinner V shell and a narrower carrier) and added about 90 g of steel in the post. With
round 2's stand, drive and structure (the printed risers and their lifts, the board's
window, the NRV050 and its hand crank, B's 686 mm board, the 12 mm tie rods) the cradle
stands at 15.22 kg on B and 14.13 on A. With a sealed full 6 L at its shell model's centre
of mass (390 mm) the crank's peak is 35.8 N·m on B (the NRV050's SF 2.37 on its 85 N·m;
21.7 N at the 125 mm grip run in) and 37.7 on A (2.25; 22.0 N at its 130 mm grip, at the
bar 22: open); the clutch's cap 63.8 and 67.2 N·m (1.33 and 1.27 x under the gearbox's
maximum). B's stand tips at 25.5° over its rear edge (41.7° front, 28.9° drive side, 31.2°
passive side), A's at 24.5° to its drive side (39.2°, 25.3° rear, 33.0°). The side frames'
redesign of 2026-10-20 makes the cradle 16.45 kg on B and 15.23 on A, and its stand tips at
26.7° on B (rear), 25.3° on A (drive side). The clutch is set at 1.3 x the user's measured start, so its
least slip in the check's window stays 1.2 x the start on both, whatever the mass (the
NRV040's fixed setting had cleared A's by 1.10).

**Review round 1** (the same day; `/root/scratch/decant/review5/r1/mounting.md` on the
development box). What it found, and what changed:
- **A full 6 L tipped nose-up at rest (B1).** The rear support ran out at 380 mm; it now
  runs to 588 on the rails' flanges, and `cradle_report.balance` checks every size at rest,
  level and full tilt (above).
- **Setting a big bottle took three hands and a hex key (M1, and the stand's M5).** The
  gate is set first, by its scale, with the bottle out, and locks by a knob on a T-slot nut
  (no pilot re-threaded at each change, m3); the knobs likewise; the bottle then lays in
  onto the gate with the gauge already down.
- **The rear knob was in a 6 L's way and its M8 yielded at a 50 N knock (M2); 55 turns
  (M5).** The lead screw's knob moved outboard over a tower's arm that holds the screw's
  top; 9 turns.
- **The gauge's post took 10.8 MPa across its layers (M3) and nothing returned the gauge
  (M4).** Two knuckles on an 18.4 mm post, the push in bearing, rated with its turn; the
  return by gravity from its stop.
- **The liners walked (M6).** Each is pegged into its V.
- **Minor:** three named grooves in front, dots on every scale (m1); the V blocks print
  standing, the rails on their outer faces, one each, the liners flat or on their edges
  (m4); drains in the front carrier (m7); grease on the thumbscrews (the BOM's notes, m9);
  the guide measures an off-list neck 50-80 mm from its lip (m5).
- **Not done:** the rails still reach 600 mm for every bottle (m10); the light stays the
  penlight (m6, S4); the tie rods stayed NETRF10 (S5; NETRF12 since round 2's structure,
  for strength); the post's socket isn't deeper (m2: the
  post itself is rated at rest instead, SF 3.1).

**Review round 2** (the same day; `/root/scratch/decant/review5/r2/mounting.md` on the
development box). What it found, and what changed:
- **The rear V block jammed raising it (N1, the blocker).** Its one post stood at -z,
  213 mm from the screw that pushes it up: past a friction of 0.149 on the post (0.095 at a
  750 ml's setting) no push lifts it, and printed plastic on steel measures 0.2-0.3. The
  post moved beside the screw at +z, 27 mm in front of it: it jams only past 0.74 (a
  750 ml's setting) to 0.98; `cradle_report.drawers` rates both guides at every bottle's
  setting, and `saddles` lists them.
- **The scales fitted one bottle per size, and the setting 0.7 mm (N2).** The base stop's
  named grooves became a plain length scale with a vernier on the gate's pointer, the gate
  2.5 mm behind the base (it had been 0.5), and the guide measures every bottle. Of
  `cradle.SIZES` against the makers' pages: the 750 ml became Saverglass's 4427 (300.5 mm,
  it had been 300; its centre of mass 192 mm from the mouth); the 3 L is Saverglass's Jero
  Vignoble (470.3 x 140.8 mm, 2.9 kg, the record's), the reviewer's 474.3 x 135.2 their
  Standard: both lie in it at their own settings (`bottle_hits`), as do 750s of 308 and
  315 mm; the 6 Ls and the magnum match to 0.2 mm.
- **The 6 L's rails rocked in their sockets (N3).** Crush ribs on the tongues, the
  clearance moved above them so they sit on the sockets' floors, a keel on each rail (2.0 x
  as stiff); `balance` rates the flanges' sag at the mouth (0.89 mm at rest at worst, 0.23
  past level; 1.5-4 mm before).
- **The lip gauge's return beat its friction by 1.3 x (N4).** Fixed: no hinge; a bottle
  is uncorked standing before it goes in (and m2's three tasks for two hands go with it).
- **The rear brake was friction only, beside a greased screw (N5).** The knob drops onto a
  ring of teeth that lock the screw; the grease note keeps grease to the thread above the
  collar, off the collar, the washer and the teeth.
- **The printed parts' 10 MPa was not a quarter of anything cited, and two parts bent
  across their layers (N6, m9).** Each row rated against Prusament's cited strength in or
  across its layers, by how it prints; the parts that carry the bottle in PETG at 100 %
  infill (their stresses are solid sections'); the rails print standing (their T-slot's
  lips bend in their layers), the gauge on its side; the T-slot's lips 5.5 mm (5), to take
  a firm hand's 1 N·m on the lock.
- **Minor:** base first when loading (m1); uncork standing (m2); the post nickel plated,
  and 15 mm (it had taken a hung 6 L at 2.99 x its yield, under the 3 asked), MISUMI
  PETR15-150, USD 19.25, on an M8 x 12 (m3); the pad's bottom 0.5 mm lower, 3.25 mm down a 750's
  rim, and the guide measures the neck 80 mm from the lip and the body where it lies (m4);
  the front V's box guide 45 mm tall at its push's side, greased (m5); a drain from the
  T-slot's low end through the +z rail's keel (m6); "tug the gate" in the setting steps
  (m7); the T-slot's ends chamfered 0.5 mm where block and rail meet (m8). And, to pay
  for the 15 mm post's weight, the rear V shell 7.5 mm (9) and the carrier's -z block 15 mm
  wide (29): the printed mounting parts 1220 cm³ solid (1223 before).

**Review round 3** (2026-10-20; `/root/scratch/decant/review5/r3/mounting.md` on the
development box, its traced drawings and probes in `/root/.cache/spiderpig/tmp/r3-mounting/`).
It traced ten real bottles from their makers' drawings and laid them in as the guide says.
What it found, and what changed (USD 0, a few grams of PETG):
- **The neck was measured 3.5 mm from where the V bears (M1).** The V ends its bearing at
  64.5 mm, relieved 6 mm behind it, the carrier's back wall notched 6 mm deeper, and the
  guide measures the neck 62 mm from the lip (`cradle.FRONT_BEAR`, `cradle.NECK_AT`). Set
  so, every traced 750 lies within 0.2 mm of the pivot, and a 4427's shoulder clears the
  relief by 1.4 mm and the back wall by 3.0 even slid onto the pad (`cradle.bottle_room`).
  The 750 ml and the impériale are drawn as traced (`cradle.SAVERGLASS_4427`,
  `cradle.UNIVERRE_NGBD06QFR`), so the clash test lays the real shoulders in at every
  crank angle.
- **A real 6 L impériale lay on the light (M2).** The light moved forward to 100 mm
  (`cradle.LED_X`; 118 before): its head 10.7 mm under the impériale's glass (it had
  overlapped 7-10 mm on the drawing; 4 by the reviewer's rest), 7.4 under a 4427's (6.7
  before), 21 under the methuselah's neck. The clip, the lowest part over the decanter,
  rose 14 mm (394 mm over the table, near rest): the RIEDEL Ultra's whole riser keeps its
  20 mm rule with more to spare. Moving it 45 mm aside would have kept its height and
  needed a new clip.
- **The gate's clamped ear took the push's lever across its layers, unrated (M3).** The
  gate's top bar, ears and pointer stand 9 mm over the rails (6), the ears' roots filleted
  4 mm, and it prints on its top, so their roots bend in its layers: the row in `saddles`,
  6.1 MPa, SF 7.7 (bar 4), and its plate hanging from its top bar across them, SF 19. The
  reviewer's 12 mm would want an M5 x 30 lock bolt; at 9 the M5 x 25 still runs 1.1 mm
  past its nut.
- **Minor:** the wet slide onto the pad rated, its post widened, the bottle and liners
  wiped dry (m1, above); the gate prints on its top, its pointer 4 mm wide (m2); the rear
  knob's hub's M3 set screw bears on a flat filed on the lead screw's top
  (`cradle.LEAD_FLAT`: by friction on a thread crest it had rated 3.3, against the bar of 4),
  both set screws threadlocked and rated (`saddles`: the hub's 14.5 in bearing, the barrel
  nut's 9.8 by friction) and checked in the guide, the knob down (m3); the vernier notched in the
  pointer's edge, read at the rail's top (m4); "out of the sun, under 50 °C", against
  Eastman's HDT (m5); the gate tugged again before the first turn (m6); tapered bodies in
  the guide's paper check (m7); the front box guide's cheek 10 mm taller over the gauge's
  post, rated dry (m9); a 250 mm bed for the 249 mm rails in the print note (m10).
- **Not done:** the methuselah's neck and shoulder are still an estimate (m8: no maker's
  drawing found; the light clears a shoulder as full as the impériale's by 10.7 mm); a
  24 h bench sag test of a rail at 23 and 50 °C (m5) and the printed parts' creep (m6)
  stay unmeasured: `cradle_report.SUSTAINED_SF` 4 stands for them; the reviewer's sparse
  PETG infill (S3) waits for those rows to be re-rated.

## The vessel under the mouth

At rest, 50° neck up, the cradle box hangs straight under the mouth: the front tie rods, the
front saddle and the light cross the decanter's axis. `cradle_report.room` measures the room
a vessel has round its axis at every height, over the whole swing and from the pour on (the
cradle past −20°), with the drawn 6 L methuselah in the saddles (the V blocks at their
lowest: a thinner bottle's ride higher, off the decanter), and names the cradle box's part
that comes lowest over the drawn decanter's rim (within its 37 mm of the axis) over the
swing (`room`'s `lowest`): **the light's printed clip, 394 mm over the table, near rest**
(4° from it; 380 at rest before design review round 3 moved the light 18 mm toward the
mouth, mounting M2). The front saddle the guide had named is 53 mm higher (447 mm).
- **The one rule** (`cradle.RISER_CLEAR`, design review round 2's B1, 2026-10-14): a
  vessel's rim at least 20 mm under that lowest part, and every point of it 10 mm off the
  room's side (the box beside it, the bay's faces), on the highest printed stack under
  that (`cradle_report.riser_fits`, `riser_for`, `riser_stack`). Rest is a toggle dwell: the
  bottle moves under 2° over the last 1½ turns while the rod's force grows past any part's
  rating (10.7 kN at 5°), so whatever the box comes down on there it crushes; the 20 mm
  takes a handmade decanter's tolerance, a profile traced from a photo, the joints' 0.98 mm
  of play and the box's 0.58 mm sag under a 6 L.
- One number per decanter, everywhere: the guide, the cradle page and this record give the
  same stack by the same rule. The page (`viewer/src/decant/liquid.ts`, `riserFits`,
  `bestRiser`, `riserStack`) is the Python's twin, held to it by
  `tests/fixtures/cradle/riser_reference.json` (every listed vessel's stack on B's room),
  which `liquid.test.ts` and `tests/test_cradle.py` both read; the room's rule and stacks
  come from the API (`/api/cradle/{key}/room`).
- The drawn RIEDEL Ultra (225 mm): the rule allows it 129 mm; it stands on the whole printed
  set, 123.2 mm (`cradle.DECANTER_RISER`), its rim at 348.2 mm, 45 mm under the clip (32
  before design review round 3 moved the light). A RIEDEL Cabernet Magnum carafe (269 mm):
  the blocks and both plates, 96 mm (the blocks alone, 87, before); a RIEDEL Ultra Magnum
  (240 mm): the lifts, the blocks and the 5 mm plate, 119.2 (114.2). Before round 2 four heights were given for the RIEDEL Ultra: the guide's size rule
  124 mm, the model 135, the guide's finger rule (aimed at the front saddle) about 137 or,
  read literally, 205 (into the clip), and the page 148, whose rim stood 7 mm under the clip:
  its `bestRiser` kept 3 mm off the room's side and nothing over the rim.
- **One height for the whole swing** (round 1, 2026-10-14). From the pour on the room is
  larger: the rim up to about 515 mm over the table (55 mm across above 475), 35 mm under
  the mouth (`cradle_report.room`'s `room_pouring`). Using it meant raising the vessel once
  the cradle was past −20° and lowering it before cranking back to rest. Forgotten, the
  worm drives the cradle box down onto the glass while the handle only feels a little
  stiff: near −30° each newton at the grip puts 20-25 N on the rim. The printed riser can't
  be raised mid-pour without lifting the decanter off it; the guide, the use limits and the
  cradle page give one stack, set at rest, and say never to raise the decanter mid-pour.
- **A bigger bottle than its decanter** (a 6 L fills the largest listed decanter four times
  over): the guide's Decanting step and the page's "full at N turns" say to turn back a turn
  until the stream stops, slide the decanter out under the raised box and set the next on
  the same riser.

## Flat pack

`spiderpig/cradle_pack.py` (`flat_pack`) takes the cradle apart for the trip and stacks it
in one box. The cradle page's Flat pack button flies each piece into the box, in the order
they go in (`/api/cradle/{key}/pack`). B packs as modules, A flat.

**B: four pieces, the riser and the hand crank** (`cradle_pack.module_pieces`, the user's
of 2026-10-13; the steps rewritten in round 1 of 2026-10-14 and again in round 2:
`cradle.module_steps`, each a `cradle.ModuleStep` with its title, text and picture, the
guide's and the cradle page's, and since the pack's turn 2 the guide's "Into its box" and
"Out of its box" steps between them, `cradle_pack.box_steps`; the cradle box on its own
since the pack's turn 1, the same day). The base board (its four brackets on it since the
pack's turn 2, the clutch's check stub taped in its riser locator's pocket since turn 3), the
passive side panel, the cradle box, the drive side panel, the riser and the hand crank, 11
printed spacers and 2 corrugated fillers that hold them. No tool, no small part left
loose: 745 x 394 x 600 mm outside, 175.9 L (the real-shape packer's plan, on the planner's
16-layer stack, with round 2's NRV050, the riser's lifts, the side frames' redesign of
2026-10-20, simplification round 2's racking brace and the pack's turn 3), 17.5 kg with its
spacers and fillers, where the drive side carrying the cradle box had packed in 699 x 498 x
608, 212 L (its flat pack had been 112).
- **Pack turn 3** (2026-10-21, packing review 2's B1, M1-M5; DECISIONS.md): the box as
  round 2 left it, 745 x 394 x 600, 175.9 L, but now a pack, not a placement:
  - **B1, the load paths:** nothing bears on the lip gauge, a pin or its R-clip, the
    gearbox or a screw head (`cradle_pack.KEEP_CLEAR`); the drive panel rests on its outer
    plate on five spacers over the cradle links' edges, the rear saddle's hub and the hand
    crank's arm (its gearbox 3-6 mm clear: it had carried 78 % of the panel), the passive
    panel on its links b2 and b3 and its outer plate (it had carried 56 % on a button head),
    the cradle box on the board (its only +x stop had been the lip gauge, rated to 1.4 g).
    Settled on the real meshes (packing review 2's `settle.py`): the drive panel 37.4 N on
    spacer 3 and 30.0 N on spacer 4, both under its outer plate, none on the gearbox; the
    passive panel 19.7 N on b2, 4.8 N on b3, 1.4 N on its outer plate.
  - **M4, lifting:** the judge counts up (`voxpack.metrics`); every piece and spacer is held
    each way, up too, to a cell (3 mm): the riser has a cap, the board two corrugated
    fillers at its ends (it slid 36-48 mm along the carton, the cradle box's length).
  - **M1, what moves:** the clips keep a cell clear; each panel's free links are taped at
    their paint lines before it goes in (the passive side's b3 to its inner bar and b5 to
    b3, the drive side's b5 to b3: `cradle_pack.SET_LINKS`), as the way back already did.
  - **M3, the prints:** PETG (sliced by BambuStudio 2.08, X1C, 0.20 mm, 15 % gyroid, two
    walls: 512 g, 18.8 h; turn 2's 14 TPU spacers were 651 g, 43.1 h), 11 prints where
    there were 14 (three pairs joined, `voxpack._bridges`; spacer 3 and spacer 12 taped to
    what they stand on, `Spacer.taped`), each its number and an arrow engraved.
  - **M2, the guide:** a picture of the pack for each box step, the spacers in it numbered;
    the spacers in the print list; 3 box steps each way where there were 6, and the trip's
    18 steps where there were 31 (`cradle.TRIP_MERGES`).
  - **M5, the carton:** Uline's S-4509 cut at its 16 in score (275# board, rated to 65
    lb; "The carton and shipping" below).
  - **The volume:** unchanged. The cradle box stands 212.5 mm tall since round 2's brace;
    standing is its least height (on its side 291 mm), and the two panels nest over it as
    they did. The searches round no plan with the new rules (6 seeds x 100 s) found 164.0
    L at best, the cradle box on top of the two panels on 25 spacers, three of them free to
    slide; the plan's layout with the drive panel over the box and the passive panel beside
    it (162.1 L, a search without the rules) leaves the drive panel resting on its gearbox.
    A cut or stock carton bills the 70 lb USPS price either way (the dimensional weight is
    over 70 lb from 13 in up): litres no longer buy postage.
- **Pack turn 2** (2026-10-20, the packing review's L1, M1, M2 and minors): 740 x 337 x 600,
  149.4 L before.
  - The brackets stay on the board (L1, "The brackets" under "The base board"): the
    passive panel, on top, had been 30 mm taller with its two brackets' foot legs standing
    off its outer plate. Now its studs and knobs stand 11.2 mm off it: 739 x 318 x 600,
    141.1 L (-5.6 %).
  - The hand crank packs in the cradle box's bay, between its two cradle links in front of
    the front saddle (m3), caged by the box: it had lain loose on the board, free to slide
    134 mm. The box doesn't grow (141.0 L).
  - Printed spacers replace the foam pads (M1, "The spacers" below): every piece held to
    within a cell (3 mm) of its place, where it had been free to slide up to 357 mm (the
    riser). The pads had been a voxel convention: no part, no order line, no step.
- Before round 2's structure it was 740 x 316 x 601, 140.7 L, and round 2's took it to 740 x
  334 x 600, 148.1 L. The 18 mm is the passive panel's (`pack-inspect`, traced 2026-10-20):
  round 2's 40 mm front leg (B1) made its outer plate 9.6 mm longer at its front foot, so the
  plan's quarter-turned panel no longer fitted 3 mm off the box's wall beside the drive
  panel's rod b1 and pin P. The packer set it 11 mm over, its inner plate on them: 16.6 mm
  higher, and the posts' new DIN 125 washers made it 1.6 mm thicker (the stack 0.6).
- The side frames' redesign put a bottom chord on each inner plate, and quarter-turned the
  passive panel's chord landed on the drive panel's b2 and b3: 740 x 385 x 600, 171 L, and
  the packer's search found nothing better. Turned end for end instead
  (`cradle_pack.PACK_PLANS`), its chord lies clear of them: 149.4 L, 3 mm taller than round
  2's.

1. Crank it to the pack pose: from rest, turn the handle the way that swings the crank's arm
   down, until the arm lies along the drive side's outer plate's rear leg, straight down
   from the gearbox (`cradle.PACK_POSES`, `pack_pose`; `cradle_pack.pack_t`). Seen from
   outside the leg hides it. It is about 5½ turns from rest and the bottle is 31° neck up.
   The decanter away, the riser comes out of its locator: its plates, then its two blocks,
   each on its lift.
   - At rest the passive side's cradle link b6 overlaps that side's inner bar O1-O2 (its
     front ties' boss over the bar's O1 end, by about 2 mm) until 46° neck up. Past that it
     clears it: about 4 mm at 44°, 15 mm at 35°, 20 mm at 30°.
   - A mark, not a count: rest is a toggle dwell (the bottle moves under 2° over the last
     1½ turns), and the old "4 turns on" gave 35° one way and 39° the other, or 44-47°
     for a rest called early, where b6's boss jams on the bar (round 1's stand review).
   - From 44° to 20° neck up the box doesn't grow: the board sets its length. At full tilt
     the cradle box stands up out of the side (a 259 L box).
   - The paint pen's lines that mark the pose for the way back are made in the first build
     (the guide's step laying it down): a line across the arm b3 and the inner bar at its
     O1 end, and one across b5 and b3 by J2, on each side. Paint on bare 7075 wears; the
     step says to mark them again if they have. (Laser-cut witness notches instead, round
     2's m11, were weighed and not cut: each would sit in the ring of metal round a
     jam-rated pin's hole, b3's at O1 or b5's at J2, which the net-section and tear-out
     rules rate without a notch.)
2. Hand crank: loosen its thumb screw and pull it off. Nothing turns without it. (Its
   thumb screws stay in their brass inserts and the input key is glued in: nothing small
   comes loose. A stow clip on the drive side's outer face was tried in round 1's drive
   review: B's plan lays that face on the passive panel, so the clipped crank lifted the
   drive side 54 mm, over the box's bound. It packs in the cradle box's bay since the
   pack's turn 2.)
3. Loosen the four knobs on the outer plates' studs by hand, a few turns each (two at
   least), and lift the cradle straight up off the board by its two outer plates' top
   members (never by the cradle box: only the worm holds it), two people if they can, one
   each side: each stud and its jam nut rise out of its bracket's open slot, the rails out
   of their cleats. The brackets stay bolted to the board (pack
   turn 2); each knob is turned back down onto its jam nut for the trip, so nothing on the
   board or the plates comes loose. (Until turn 2: the four thumb knobs undone off the
   board's bolts and put back on them.)
4. Stand the riser's two blocks, each on its lift, on a folded towel and lay the cradle on
   its drive side on them and the gearbox (`cradle.LAY_BLOCKS`): one under pillar O1's
   screw head (the head in the cone and bore in its block's top), one under column E1's on
   the rail (D2's on the front leg until the side frames' redesign of 2026-10-20). It lies
   level, the passive side up; gravity now runs along the pins, so no link can swing. A
   hand on the cradle box while the other works: lying, the cradle tips past 18.9° and 21.3
   N at its top.
   - **The lifts** (design review round 3's stand and use, M5): off the board onto the
     blocks and back up onto the board, the cradle is 13.8 kg (14.0 until simplification
     round 1; `cradle.LIFTS`, held to
     `rest_poses`' masses), lifted by its outer plates' 4.75 mm top edges 430 mm apart,
     their tops 465 mm over a counter. The NIOSH lifting equation's recommended limit
     there is 11.7 kg before its distance, asymmetry and coupling multipliers (load
     constant 23 kg, HM 25/40, VM at 136.5 cm), a lifting index of 1.20 or more. So the
     guide says two people, one lowering, one setting the blocks under their heads, and
     gives one person an order that can be done (`cradle.lay_down`): the gearbox's end down
     first, the front end held up while the blocks slide in, E1's (nearer the gearbox) then
     O1's, their middles about 460 mm apart. The blocks' cones take a head set down within
     7 mm (`cradle.RISER_RELIEF`): lowered blind, a 114 mm block on its 49.8 mm side had
     toppled at 24°.
5. Pull the two R-clips of pins X and Y on the passive side, reaching into the bay, from
   under the passive cradle link ("The quick-release pins"). Each pin stays in its link,
   standing on its head, its TPU keeper on it.
6. Lift the passive side panel straight up off the pins by its outer plate, level, its pins
   and their keepers with it, push each clip back through its pin's hole, and set the panel
   down on its outer plate on the towel (it rests on two studs' bare thread ends and O1's
   head, round 3's m9). The drive side with the cradle box stays lying as it is, the
   worm holding it.
7. Pull the two R-clips of pins X and Y on the drive side, reaching in from the side between
   the drive side's cradle link and its inner plate (the cradle box is open at its sides),
   a hand on the box. Each pin stays in its link: its head is under the link, and its keeper
   holds it.
8. Lift the cradle box straight up off the drive side's pins, level, by its top cradle link
   and a tie rod (3.6 kg), turn it a quarter about its tie rods and set it down on its two
   cradle links' long edges, its saddles' open tops down (the base stop's knob the third
   point; the tie rods level, so no load on the saddles' set screws). Push each clip back
   through its pin's hole. The drive side stays on its blocks; its link b5, on its one pin
   J2 now, turns flat.
   - Standing, as the steps had it before round 1, the 6.8 kg drive module tipped over once
     the pins cleared b6 (its centre of mass 66 mm past the one rail it stood on), dropping
     its cradle box onto the table, and the panel fell the other way. Standing on the board
     neither side can slide off the pins (each rail is between its cleats, over its bolts),
     and only the drive side holds the box (the worm): the passive side's links are free,
     so the passive panel comes off first and the box last.
   - **The keepers** (`cradle.KEEPER`, pack turn 1; round 2's M3): each pin's head lies on
     its link's outer face (`construction.pivots.clevis`), 50 mm and more over that side's
     outer plate with nothing under it in plan, so with that side down and its clip out a
     bare pin drops out of its link. The printed ring each pin's plan puts in the gap
     between b4 (b5) and the cradle link is printed in TPU 95A with a 7.4 mm bore on the
     8 mm pin: it grips the pin and holds it, its side down, the clip out. That is what lets
     the cradle box come off the drive side as it lies, and it takes the loose spacer out of
     the passive panel's steps. Its grip (`cradle_report.keepers`): at the least
     interference a print gives (0.4 mm on the diameter: the pin 7.95-7.99, the bore printed
     to within 0.15) and TPU 95A's least published modulus (26 MPa, UltiMaker's TDS v5.00;
     its v2.01 of 2025-09-15 gives 56-67), on an oiled pin (friction 0.3), 12.2 N, 109 x the
     pin's 0.11 N (the bar 10); at its nominal 0.6 mm, 67 MPa and dry friction it goes on
     with a push of about 78 N, a short tube over the pin (the bar 120).
   - **The keepers' real load** (round 2's M3 and the pack's review 1's M2): the pin's own
     weight is not what they meet. Lowering the box (3.6 kg) on pin X and turning it until
     Y lines up was taken to rest up to half its weight, 17.6 N, on Y's tip, which would
     push the pin head-first out of its link through a keeper at its least grip (12.2 N;
     6.1 N at the old 7.6 mm bore). Its statics (pack turn 2, `keepers`' `second_tip_n`):
     the box's centre of mass lies 56 mm past pin X from Y (and 105 mm off the line X-Y), so
     with X in its hole the box's weight turns it about X off Y's tip (-7 N: none on it);
     the other way round, Y first, it would rest 42 N on X's tip, more than its whole
     weight. So the step says "pin X first, always", and why. Three more answers: the way
     back holds each piece's weight until both its pins are in their holes, never on a
     pin's tip (and says how to push a slid pin home); its first step presses each pin's
     tip with a thumb, about 3 kg, before the box goes on (a keeper that has relaxed shows
     there, not under the box); and the guide's bench check at the build pushes each with
     30 N (`cradle.KEEPER_PUSH_N`, 1.7 x the 17.6 N: `keepers`' `load_ratio`, the bar 1.5).
     The passive panel's pins hang by their heads as it goes on: a tip that meets the link
     lifts its pin, head proud, never out. And the keeper's push on at its tight end (the
     bore 0.15 mm under, the pin at its 7.99 mm most: 0.74 mm) is 116 N, under the 120 N bar
     (`push_on_tight_n`, the packing review's m1): still by hand, a tube over the pin.
   - Turning the drive module over onto its cradle box instead (the box under, the drive
     side up, its pins' heads up) was weighed and dropped: on its box the module stands with
     its centre of mass 18 mm inside the tie rods' heads, 274 mm up (it tips at 3.8°; 26 mm
     at best over the pack poses' range), and the passive panel and box turned over
     together are a free linkage (no worm on that side).
   - `cradle_pack.slide_hits` sweeps the panel's parts along z in 0.5 mm steps against the
     drive side and the cradle box, then the cradle box's against the drive side: nothing
     meets (the payload's `slide_off`, with the box's `box`).

**How each piece rests** (`cradle.REST_POSES`, rated by `cradle_pack.rest_poses` on the
pieces' real shapes, each its hull's stable face nearest the way it's set down; the bars in
`tests/test_cradle.py`; round 2's M2):

| pose (B) | kg | rests on | tilt | tips at | push at its top |
|---|---|---|---|---|---|
| the cradle lying on its drive side on the riser's blocks | 14.01 | the gearbox and the two blocks on their lifts | 0.1° | 18.9° | 21.3 N |
| the drive side and the box, lying, the passive panel off | 10.95 | the same | 0.1° | 19.2° | 14.0 N |
| the drive side panel alone on its blocks (the way back starts here) | 7.34 | the same | 0.1° | 40.5° | 29.0 N |
| the cradle box on its links' long edges, its saddles' tops down | 3.62 | both cradle links' edges and the base stop's knob | 11.0° | 40.8° | 17.2 N |
| the passive side panel on its outer plate | 3.05 | pillar O1's screw head and its two studs | 0.9° | 64.7° | 23.9 N |
| A, off its board while the finish cures, standing on its brackets' feet | 13.21 | its six brackets' feet | 0.0° | 27.0° | 32.3 N |

The side frames' redesign (2026-10-20) put about 0.6 kg more on each side panel, most of it
low on the rail and its inner chord, away from the binding edge (below): the cradle lying
tips at 18.2° (15.4 before), the drive side with the box at 17.5° (14.3, its test's bar 15°
again: 14° from round 2's drive), A at 17.6° (17.2). The second block went from column D2's
head on the front leg to the rail's column E1's (`cradle.LAY_BLOCKS`): on D2's the heavier
drive panel alone tipped at 25.3°, under its 30° bar; on E1's (or E2's) 39.4°. (The
table re-measured 2026-10-20 on design review round 3's saddles, blocks and brackets, then
on its structure: its wider members, b2 in 5052, the fender washers, A's column E2.)

Round 3's structure put B's sides 0.6 kg heavier, low again (the cradle 14.0 kg, 12.85
before; it tips at 18.9°, the drive side with the box at 19.2°). It took A's centre of mass
toward its gearbox: lying on its drive side on the blocks it tipped at 13.4° under the 15°
bar wherever they stood on the plate (every stand node and a grid of points were tried). A
doesn't come apart, so nothing needs it on its side: off its board while the finish cures, it
stands on the bench on its brackets' feet as it stood on the board (2026-10-20,
`cradle.REST_POSES`), tipping at 27.0° and 32.3 N; the guide's step says so, and its 13.2 kg
lift (`cradle.LIFTS`).

Before the redesign: on round 2's NRV050 (1.35 kg more, 27 mm further out) and with the mounting round's saddles
the lying poses tip sooner: the cradle 15.2° (16.9 on the NRV040), the drive side with the
box 14.3° (16.3; its test's bar 14° since, 15 before), the drive panel alone 32.2° (35.2).
With round 2's 12 mm tie rods (the table's numbers) the box weighs 3.58 kg and stands on its
edges to 41.5° (44.3), the drive side with it still 14.3°, the cradle 15.4°, A's 17.2°
(18.3).
The binding edge runs from the O1 block to the gearbox; moved or turned on the plate, the
block had done no better than 15.0° (`cradle_pack.lay_blocks` tried under every stand node
and pillar pair, and the O1 block turned every 15°), and moved up off O1 it leaves the
plate. The steps keep a hand on the box.

Round 1's poses had been perches: the cradle and the drive side lay 14° tilted on the
gearbox, the O1 pillar's screw head and the front bracket (the whole tipped at 15.4°, 9.1 N;
the drive panel alone, where the way back starts, at 6.8° and 2.5 N), and the box "set down
as it was, on its other cradle link" stood on L.b6's edge and two of the four tie screws'
2.75 mm heads, its centre of mass 22 mm outside their quadrilateral: it tipped at 10.0° and
2.8 N. (The "41 mm" quoted for it was to L.b6's outline, which it never sat flat on.) The
stacked poses gain little from the blocks (the box at the pack pose overhangs the line O1-O):
the steps keep a hand on the box.

**Back together** (the payload's `reassembly`, the guide's "Back together" steps):
1. The riser's two blocks, each on its lift, on a folded towel, the drive side panel laid
   on them and its gearbox, outer plate down; its pins X and Y standing in their links on
   their keepers, the clips out, each pushed fully home by its head, then each one's tip
   pressed hard with a thumb (about 3 kg): it mustn't slide.
2. The cradle box lowered onto it, level, its knobs' side up, its weight held until both
   pins are in their holes: pin X into its hole first, always (the one in the long link b4,
   nearer the gearbox, `cradle.pin_words`: the first build paints an X beside its hole on
   each cradle link, and the pictures call it out; round 3's m1: X and Y are the same pin
   with the same keeper, and nothing had told them apart), then the box turned about it
   and b5 swung until Y lines up (with X in, the box's weight lifts it off Y's tip: "The
   keepers' real load"). Never on a pin's tip. Both clips in from the side.
3. The passive panel on its outer plate on the towel, its pins held by their clips; its
   links set to the
   pack pose by the paint marks, a strip of masking tape across b3 and its inner bar (not
   b5: it is the one nudged into place, round 2's m5).
4. Each clip out first, the panel still lying (each pin stands on its head, its keeper on
   it), then turned over and held level: each pin hangs by its head, its keeper on it
   (round 3's m8: the clips had been pulled with the panel held level in both hands).
5. Lowered level onto the cradle link, its weight held until both pins are in: pin X into
   its hole first, then b5 nudged until Y lines up. The tape off.
6. Both clips in from under the cradle link: a pin without its clip is held by its keeper's
   grip alone.
7. Each knob backed off its jam nut two full turns first (`cradle.KNOB_BACK_TURNS`: it
   stands 1.6 mm off the nut, and the 5 mm upright passes between it and the plate; round
   3's M1). Then stood up (two people if they can) and lowered onto the board: each rail
   between its cleats, each stud's jam nut down into its bracket's slot (the cleats'
   lead-ins and the slots' chamfered mouths guide it); the drive side's knobs snugged, the
   passive side's only started. (Until turn 2: each bracket's foot found its bolt blind,
   9.5 kg held, and the washers and knobs went on.)
8. The riser back in its locator, stacked for the user's decanter (the cradle page's Riser
   line, written on the locator's underside; round 3's m7: the whole riser restored stood a
   shorter decanter too close under the clip).
9. The hand crank on.
10. Turned to level (about 8½ turns from rest) with the passive knobs loose, then snugged
    there: B's two remote centres agree best at level (round 2's m6).
11. The first build's checks again, the bottle out: both cradle links swing freely, the
    handle turns evenly with a fingertip (under 5 N at the grip, the luggage scale hooked on
    it under 0.5 kg: `cradle.EMPTY_GRIP_N`; the report's `empty_handle_force_n` is 3.6), and
    the rod's paper check. All four clips in.

The pieces: the base board (its feet, cleats and riser locator, its four brackets on their
bolts, washers and nuts), the riser (its lifts, blocks and plates, one piece:
`cradle_pack.RISER`), the passive side panel (every part of that side but its cradle link
and its brackets: its studs and knobs with it), the cradle box (both cradle links, the ties, the saddles with their V blocks,
knobs, post and lip gauge, the light, the base stop), the drive side panel (the gearbox and
every part of that side but its cradle link and the hand crank: the worm holds its links as
cranked) and the hand crank. Each side's b5, hanging on J2, swings into its panel
(`cradle_pack.tuck`). B's plan (`cradle_pack.PACK_PLANS`): the cradle box in first, on the
board as it stands in use, along its length; the hand crank in its bay; the drive panel as
it lay when the box came off, its outer plate down, its plates over the box and its
gearbox down beside it, on five printed spacers; the passive panel last, its bay face down
over the drive panel's, turned end for end, on six; the riser in the board's front notch;
the mass centre 41 % up the box. The box's length is the cradle box's, 723 mm levelled
from its links' front pins Y (122 mm in front of the mouth) to its rails' end (600 behind
it), 41 mm longer than the board; its height the board, the box standing (178 mm) and the
two panels nested over it. Packed the other ways a person would try: the passive panel on
the board, the drive panel bay to bay over it, the box on top, 182 L; the drive panel on the
board under the passive panel, the box on top, 186 L.

**The pack's leads, measured** (turn 1, 2026-10-14, on round 1's design and its 17-layer
plan; `--split` and a board swapped for a smaller box in a scratch driver; each searched
90-120 s on 4 processes; on the planner's 16-layer plan the first two rows are 211.8 and
140.5 L):

| lead | box, outside | litres | what it takes |
|---|---|---|---|
| none (the drive side carries the box) | 699 x 499 x 608 | 212 | |
| **the cradle box off the drive side (made)** | 740 x 316 x 600 | 141 | the drive side's clips out and a lift; TPU keepers on X and Y |
| and the passive side planned on its own, 44 mm thinner | 741 x 323 x 601 | 144 | a second plan, its own plates and posts: no gain, the panels nest over the box anyway |
| and the board 76 mm narrower (its knobs' width) | 741 x 352 x 526 | 137 | outriggers for the 25° sideways: 2.5 %, the panels no longer nest beside each other |
| and the base stop's rails off the box | 700 x 296 x 600 | 124 | a fifth piece, its four socket screws made tool-free (thumb screws reachable under the rear V block) |
| and the rails off and the board 570 mm long | 586 x 329 x 601 | 116 | the rear tipping edge (25.1°, the bar 25°) on a fold-out foot or outrigger |

The board stops setting the box's length once the cradle box packs on its own: the box
(723 mm) is longer than it. A shorter board or a split one pays only once the rails come
off the box too.

The steps lift the cradle off the board before a panel comes off: its studs drop out of
its brackets' slots, its rails out of the cleats. B's brackets stay bolted to the board,
their carriage bolts' heads locked in its square holes (A's knobs go back on the bolts:
the carriage bolts' square holes stop them turning but don't grip them).

**A: flat.** Its cradle link rides pillar G between the side's plates, so a side can't come
off it at pins alone. It comes apart where it is already fastened (`cradle_pack.pieces`).
It is a pack for shipping once, not a breakdown for every trip: the inner plates' screws
take a hex key, the tie screws threadlocker, pin E's E-ring pliers (round 1).
1. The hand crank comes off its shaft (its thumb screw).
2. The base board comes off (the brackets' thumb knobs). The carriage bolts stay in it.
3. Each inner plate comes off (its screws, from the bay side).
4. Each side panel comes off its cradle link, where the link meets the side
   (`mech.connections`).
   - A pin: its upper E-ring comes off; the pin stays in the side's links, the ring back on
     it for the trip. A's pin E (drive side).
   - A pillar slides out of the link's bore: A's pillar G.
   - The cradle box then comes away whole.
5. The cradle box comes apart:
   - the base stop and the light (its clip's screws) off their saddles;
   - one cradle link off the tie rods (its tie screws, threadlocked);
   - the saddles slid off the rods (their set screws), each whole: its V block, liner,
     thumbscrew and knob (and the front one's lip gauge) with it;
   - the other link off. Each tie rod travels with its two screws back in its ends, and
     each cradle link with the loose spacers that sat over it.

A link left hanging on one joint once its cradle link is off swings into its panel
(`cradle_pack.tuck`).

**How it stacks** (`spiderpig/voxpack.py`, since 2026-10-14). It packs the parts' real
shapes, not their bounding boxes: each part's tessellation filled solid on 3 mm voxels
(`voxpack.voxelize`: every cell the surface passes through and every cell whose centre is
inside), 3 mm between two pieces (`CLEAR`, one cell all round: the play the printed
spacers leave, the foam it stood for until the pack's turn 2).
- **Levelled.** Every piece but the panels and the base board is turned in the mechanism's
  plane to its least bounding box (`level`). A piece then lies in any of the cube's 24
  turns of that frame (`voxpack.ROTATIONS`: the voxels transposed, exactly); a side's
  pieces name their `outer` and `bay` faces.
- **The base board** stays at the bottom, where it is. The box is as wide as the largest
  piece lying flat.
- **Dropped in order.** Each piece in turn drops straight down onto what is in, column by
  column (its bottom over the box's heightmap), so it falls into another's open triangles,
  between its links, into a bay; the order they drop in is an order they can be put in
  (none is both over and under another).
- **Where.** Of its orientations and slots, the one that grows the box least, then sits
  lowest, then lies nearest a corner, where it rests on what is under it
  (`voxpack.support`): its centre of mass over the convex hull of its contacts (dropped
  straight a piece first meets one point; it may sink 6 mm onto the next ones, `SETTLE`),
  else over pads filling gaps of up to 60 mm under it (`PAD_MAX`; each measured on the
  real meshes as a block that fits, its height the real gap: `voxpack.measure_pads`, a cut
  list), else it would tip.
- **Plans** (`cradle_pack.PACK_PLANS`, `voxpack.Plan`): a human packer's intent per piece
  (the face down, a quarter turn, a corner, the piece it rests on or nests into, what goes
  in first, why). The packer honours it and picks what it leaves free; a piece it doesn't
  name goes in after, largest first (the panels first, in every order).
- **The spacers** (`voxpack.spacers`, pack turn 2, the packing review's M1; reworked in turn
  3 for review 2's B1, M1, M3 and M4): printed in PETG (TPU 95A until turn 3: it printed at
  a third of PETG's speed, and PLA softens in a car), each one shaped to the pack. On the
  judge's 3 mm grid of the packed pieces, a spacer is empty cells, one run a column, never
  within a cell of a part nothing may bear on (`cradle_pack.KEEP_CLEAR`: the lip gauge,
  every pin and its R-clip and keeper, the gearbox and its riser and output shaft, every
  screw head, nut, washer, knob, thumbscrew and plunger, the light; `keep_clear_cells`,
  `voxpack.part_cells`), so what moves (a clip turning with its pin) or must not carry a
  load (the gauge, the gearbox) keeps 3-6 mm clear, and no piece rests on one either
  (`voxpack.metrics`' `bearless`, the packer's `Item.bearless`: the drive panel had rested
  78 % on its gearbox, the passive panel 56 % on a button head, the cradle box's only stop
  had been its lip gauge):
  - *a support* where a piece's pads were (its pads within 30 mm of each other one
    spacer, `SPACER_JOIN`): its footprint theirs grown 12 mm each way, in each column from
    what is under it to the piece's underside, and up to 9 mm past them beside a part
    (`SPACER_LIP`): lips round what it carries and stands on. One that would still slide
    more than a cell grows away from where it slides (its far side wraps the part it
    carries) or toward it (to meet what stops it), up to 180 mm a side; two of one piece
    that meet are one print; one that would tip loses its overhang;
  - *a restraint* beside a piece that still slides more than a cell: in the gap that way
    where it is least (low on the piece first), from the piece (round its end) to what it
    would meet or the box's wall, standing on what is under it, in just after the piece it
    holds (where nothing over it is in by then: turn 3, review 2's M2), else as soon as what
    is under it is;
  - *a cap* on a piece or spacer that would lift more than a cell (turn 3, review 2's M4:
    `voxpack.metrics` counts up since, the lid holding what is within a cell of it): a
    block on its top up to what is over it there or the lid, no taller than 120 mm
    (`CAP_REACH`);
  - *a filler* at each end of the base board (turn 3, review 2's M5): the carton is the
    cradle box's length, 92 mm longer than the board, which slid 36-48 mm along it. A spare
    carton of the bundle folded into a tube the gap's size, wall to wall across, from the
    floor to the lowest thing over it (B's: under the cradle box's ends), taped
    (`voxpack._filler`; cut, not printed);
  - then two printed spacers of one piece that face each other along x or z, within 120 mm
    (`BRIDGE_MAX`) and the bed, are joined by a bar of the cells between them
    (`voxpack._bridges`: fewer prints, review 2's M3); a support that still slides is left
    out where what it carries rests directly without it; one that still slides, standing on
    one piece, is taped to it (`Spacer.taped`, the guide says so: it moves with that
    piece, and the judge reads it so, `cradle_pack.judged_pieces`); and one longer than the
    256 mm bed less its margins (`SPACER_BED`, 250 mm) is cut where its section is least
    (A's two of 255 and 306 mm, review 2's M3).
  - Each goes in at its place in the order (`rank`, `cradle_pack.in_order`), never under a
    piece that went in before it; numbered in that order (spacer_01 ..), a filler too.
  A spacer meets each part with up to a cell's gap (the parts' cells are their
  conservative fill), so what it holds moves 3 mm at most: on B every piece and spacer
  moves a cell at most any way, up too (`voxpack.free_travel`). Each printed one is the
  build's `print/pack_spacer_*.stl` (`cradle_pack.spacer_print`): its cells at 1.5 mm, its
  number and an arrow toward the carton's +x end (B's: the cradle box's base stop end)
  engraved 1.5 mm into the face that goes up in the box (`voxpack.engrave`; on its largest
  side, the arrow up, where the top is too small), laid on the bed with the least overhang
  (`voxpack.print_pose`: a tall thin one lies down); its grams as a slicer lays it, two
  0.45 mm walls over its surface and 15 % of the rest (`cradle_pack.spacer_grams`: the flat
  25 % of the solid had under-counted A's small ones by 18 %). They are in the BOM's
  printed rows, the guide's print list (`guide.make.spacer_prints`) and its box steps.

**Judging and searching it** (`spiderpig pack-inspect`, `docs/agentlib/TESTING.md`): a
contact sheet per candidate (the top, front, side and isometric views, each piece in its
colour and numbered in the order it goes in; slices at 25, 50 and 75 % of the height; the
legend), and the metrics (`voxpack.metrics`, any layout voxelized afresh): the box, the
true fill (the parts' own volumes over the box's inside), the air per layer, how each piece
rests (on what, directly, on pads, or not), the pads' cut list, how far each piece slides
along the floor before it meets a neighbour or the wall (`voxpack.free_travel`), the
real-shape overlaps (cells two pieces share, measured again on 1 mm cells), the order's
faults, the mass centre's height, and the pieces a plan's step had no room for (the sheet
red, the command's exit 1). `--search` anneals the order, the orientations and the corners
round the plan (`voxpack.search`, in worker processes; `--seeds N`, the spread across
them); `--split` packs a what-if (some parts off their piece: a design change before it is
made; each piece keeps its orientation, `--relevel` levels one afresh). The cradle page's pack view edits a layout by hand (Edit the packing:
click a piece, drag its arrows in 3 mm steps, R turns it a quarter turn, T tips it; Export
layout) for `--layout` to judge.

No MuJoCo settling: per-part convex hulls fill a truss plate's triangles and a panel's bay,
so every nested piece would start inside another's hull; an honest model needs a box per
column of voxels (thousands of geoms for B's drive side). The voxel support check (centre
of mass over real contacts, pads named) answers "does it float" without it.

| round 1's design (2026-10-14), outside | before: the boxes' packer, judged on real shapes | after: the plan |
|---|---|---|
| B | 702 x 485 x 606 mm, 206.2 L, fill 3.8 %; the mass centre 52 % up | 700 x 478 x 604 mm, 201.9 L, fill 3.9 %; every piece resting directly; the mass centre 27 % up |
| A | 698 x 295 x 602 mm, 123.9 L, fill 6.2 %; the hand crank and the rear saddle tip (the saddle floats on a box that isn't there) | 700 x 185 x 601 mm, 78.0 L, fill 10.1 %; the passive panel on 5 pads to 33 mm, two tie rods on pads to 42 mm. Since round 2's drive 700 x 212 x 601 mm, 89.3 L, fill 10.5 %, every piece supported: the NRV050's housing and riser stand 27 mm taller off the drive panel. Since the side frames' redesign (2026-10-20) the drive panel goes in turned a quarter, not end for end (its gearbox would land on the passive panel's wider outer plate: 107 L): 700 x 213 x 601 mm, 89.6 L |

The search (60 s, 5 processes) found 77.8 L for A (698 mm long) and nothing better for B:
B's box was its board's footprint times the drive side's 441 mm across the bay; A's is the
board's times the drive panel's pillar G and stand columns, the hand crank and the rear
saddle (148-153 mm). Before round 1's pack pose B's drive side packed cradle box down, on 7
pads to 60 mm, its mass centre 55 % up: on its gearbox it tipped there.
What-ifs (`--split`, on the design before round 1): B's cradle box off the drive side,
754 x 316 x 602 mm, 143.5 L (searched; the cradle box on 4 pads over the two panels): made in
the pack's turn 1, 740 x 316 x 600, 141 L by its plan (the searches of 150 and 300 s, 4 and
5 processes, found the same); B's gearbox off the drive side, no gain (its brackets and
crank train stand as far out); A's stand columns off, no gain.

Pack turn 2 (2026-10-20, on the side frames' redesign; `pack-inspect`, its plan; searches
of 60 s x 3 seeds):

| B | box, outside | litres | slides (most) | what it takes |
|---|---|---|---|---|
| before turn 2 | 740 x 337 x 600 | 149.4 | 357 mm (the riser; 18 foam pads) | |
| **the brackets on the board (made)** | 739 x 318 x 600 | 141.1 | | four knobs on M5 studs, an M5 jam nut each, slotted uprights |
| **and the hand crank in the box's bay (made)** | 739 x 318 x 600 | 141.0 | | a plan step |
| **and the printed spacers (made)** | 739 x 318 x 600 | 141.0 | 3 mm (every piece and spacer) | 14 TPU prints, about 640 g |
| and the base stop's rails off (L2) | 702 x 333 x 600 | 140.2 (140.2-141.7) | | a fifth piece, 4 thumb screws, 2 steps each way: not made (-0.6 %) |

| A | box, outside | litres | slides (most) | what it takes |
|---|---|---|---|---|
| before turn 2 | 700 x 213 x 601 | 89.6 | 282 mm (7 foam pads) | |
| **the printed spacers (made)** | 700 x 213 x 601 | 89.6 | 6 mm (one spacer; every piece 3) | 25 TPU prints, about 500 g |
| and its gearbox off its panel | 698 x 205 x 601 | 86.0 (86.0-93.4) | | a loose 2.1 kg piece, its riser's 4 M6 nuts, the worm's alignment checked again: not made (-4 %) |

**The carton and shipping** (pack turn 3, packing review 2's M5; `hardware.shipping`, the
build's ORDER.md "Ship it" section: `carton_for`, `ship_lines`). Every carton and price below
was read from Uline's and USPS's pages on 2026-10-21. The build names the cheapest stock
carton in the 30 x 24 in footprint whose inside takes the pack (its parts' bounds and their 3
mm, any way round on the floor), cut down at a score line where it has them, and whose board
Uline's table rates (UPS's guideline) for the pack's weight with its carton (4 lb):
- **S-12613** (30 x 24 x 12 in, 275# double wall, rated to 80 lb, $11.71 a box in tens) is
  Uline's only double-wall carton in the footprint: 305 mm inside, too shallow for B's 384
  since round 2's racking brace (turn 2's 308 mm pack would have fitted). S-4417, the 200#
  single wall named until turn 3, is rated to 40 lb, under B's 42.6.
- **B: S-4509** (30 x 24 x 24 in, 275# single wall, scored at 24, 22, 20, 18, 16 and 14 in,
  rated to 65 lb, $11.64 a box, $116.40 the bundle of 10) cut down at its 16 in score:
  27, 23 and 20 mm spare round the pack (its spacers and fillers hold the pieces; the spare
  is the carton's), 42.6 lb. USPS Ground Advantage retail, zones 1 / 4 / 5 / 8: $99.00 /
  $133.50 / $165.70 / $235.30 (95 lb dimensional, the 70 lb price, the 30 in and 2 cubic
  feet fees). A box cut to the pack, 745 x 394 x 600 mm: $93.50 / $128.00 / $160.20 /
  $229.80 (83 lb dimensional; the 22 in fee).
- **A** (pre-brace, 700 x 253 x 607 mm, about 37 lb): S-13330 (30 x 24 x 10 in, 200# single
  wall, rated to 40 lb, $5.94 a box in fifteens), or S-11248 (30 x 24 x 12 in, 200#, scored
  at 12, 10, 8, 6 and 4 in, $6.90) cut at 10 in.
- The review's carrier option, the carton inside an outer one with 2 in of foam all round:
  Uline stocks no outer carton that size (34 x 27 x 18 in or so; the nearest double walls
  are 36 x 24 x 18, too narrow, and 36 x 36 x 18, past USPS's 130 in): a custom box,
  Oversized at USPS ($211.30 at zone 5 before the box), and 2 in EPS sheets at $11.50.

What the pack is for: the car and hand-carrying, where it rides upright and is set down.
The spacers locate each piece on its plates and hold it each way to a cell; they are
printed PETG, stiff, not a cushion. A carrier drops a parcel this heavy from 30 in (FedEx's
ISTA 6-FedEx-A, ten times); for a carrier, insure it, or pay for the custom outer box and
its foam.

The box's bounds per design are in `tests/test_cradle.py`: `PACK_BOX` (A) and
`MODULE_BOX` (B, read off its base board's size), and each one's most slide `PACK_SLIDE`.

## Loads and checks

The design case is a sealed full 6 L methuselah: 11.1 kg, its centre of mass 390 mm
behind the mouth along the axis, in the saddles at its setting (the V blocks at their
lowest: `cradle_report.masses`), plus the cradle's own weight.

**The bottles' centres of mass** (`cradle.SIZES`, since the review of 2026-10-14; they had
been 42 % of the length from the base, too far forward): a glass shell over each bottle's
outline, the neck's wall 1.5 x the body's and the base 3 x thick under a punt, its
thickness solved so the glass weighs the bottle's empty mass, and the wine filled from the
base, upright and sealed. A full bottle's centre lies 31-36 % of its length up from its
base (a 750 ml's about 11 cm up, the usual figure):

| bottle | full | from the mouth (was) |
|---|---|---|
| 750 ml Bordeaux | 1.3 kg | 191 mm (174) |
| 1.5 L magnum | 2.6 | 230 (205) |
| 3 L jeroboam | 5.9 | 325 (265) |
| 6 L impériale | 9.1 | 319 (286) |
| 6 L methuselah | 11.1 | 390 (331) |

The methuselah's 59 mm further back adds about 16 % to the crank's peak: the gearbox and
the grip are under "Turning it" below and in B's table. (Its centre now lies 10 mm behind
the rear V's end: the mounting review's finding, the saddles' own.)

The jam case is the most the hand crank's slip clutch lets onto a blocked cradle: 63.8 N·m
on B, 67.2 on A (`cradle_report.jam_nm`, `config.PRODUCT_JAM_NM`; "The clutch's cap"
below), however hard the push, while the clutch is checked at each set-up. Until round 1's
drive it was a firm push at the grip (`cradle_report.JAM_HAND_N`, 40 N: about 1.7 x what
the worst turn takes) through the worm at a hand's speed: 66 N·m at B's crank on its 100
mm handle, 69.3 at A's on its 105, and no bound; round 1 made it the clutch's 61.3.
- Rated at the gearbox's rating instead, it would be reached at less than the worst turn's
  own force: no bound (all three reviews of 2026-10-10).
- The joints' rows leave out 30° of crank round either toggle, where a blocked
  crank-rocker's forces grow without bound; the clutch's case rates them (below). There
  only clearance protects: the decanter's lip and anything under the cradle.
- What A holds there (each side as one frame, below, at the clutch's 67.2 N·m; the
  side frames of design review round 3, on its model: "The structure model corrected"):
  - the outer plate 1.38 (the rail at column E1), the tie rods across the bay 1.43, the
    inner plate 1.51 (its leg at D3), pillar G 1.57;
  - column D3 1.76, column E1 1.81, column E2 1.90, the other columns 2.92 and more; the
    side's buckling out of its plates 2.34;
  - its cradle link b2 1.44 (in bending, chord by chord; 0.187 in 5052 on its minimum yield,
    its E-Tr1 32 mm wide against buckling);
  - pin E (steel) 2.86, the crankpin 1.76, the keyed plates 2.21, the tie rods' end screws
    2.08;
  - the rod's buckling 2.3, and 2.33 as a beam-column at pin E's 4.6 mm offset.

  Round 2's side frames had held the outer plate 0.41 (its front leg from column D3 to the
  foot), column D3 0.83, the inner plate 1.09 and pillar G 1.25 there ("The side frames at
  the clutch's cap"). (At the 40 N push's 69.3 N·m, round 1's springs: 2.88, 1.6, 1.9, 2.05,
  1.63, 1.61, 2.17, 2.01, 2.97.)
- Turning it: 37.7 N·m at A's crank at worst, a sealed full 6 L on the mounting round's
  saddles (round 2's), with round 2's 12 mm tie rods: the NRV050 SF 2.25 on its 85 N·m
  (round 1's NRV040: 1.22 on its 45). 22.0 N at A's 130 mm grip run in, 25.0 new (the bar
  22 N, open: "The drive"; round 1's 85 mm crank had 32.2). It tips at 25.3° to its drive
  side (its test's bar is 15°), 26.7° over its rear edge (30.0° with the wine leaving as it
  pours), 33.6° to the other side and 39.5° forward, and takes a 42.3 N push at a bottle's
  top (toward the drive side), the worst over the swing (the side frames' redesign of
  2026-10-20 put 1.1 kg more low on the stand; before it 24.5°, 25.3°, 33.0°, 39.2° and
  38.3 N; before round 2's structure 24.5°,
  25.3°, 33.1°, 39.4° and 36.7 N; round 2's window under the base stop, on the NRV040: 23.9°, 25.5°, 30.5°,
  38.4° and 35.6 N; before the window 24.0°, 25.6°, 30.6°, 38.6° and 36.1 N).
- The rating is the plan's: a wider stack between the rod and the cradle link puts pin E
  lower. A product is never served a plan from another engine version
  (`stages.planning._reuse_plan`): it re-plans, proven thinnest.
- A link's net section is rated at its own pin's or pillar's hole as cut (8.2-12.35 mm:
  `strength.cut_hole`, the shaft and its links' running clearance), not a walker's
  6.35 mm. (Until 2026-10-14 the check took 8.35 mm for the 8.2 mm the pins are cut to.)

**The structure, from the force vectors** (`spiderpig/cradle_structure.py`, the design
review of 2026-10-14): the statics solves every joint's force vector at every crank angle
(`cradle_report.statics_vectors`), and `cradle_report.structure` rates from them what the
generic rules (a three-pin link as a beam between its farthest pins, `F L / 4`; a pillar
as a beam between both plates) missed. They feed `strength.check` (`loads_doc`'s
`link_frames`, `posts` and `rated`), so the report, `verify` and the audit see the same.
- **Links in bending** (`cradle_structure.link_bending`): each link a plane frame of its
  chords, loaded at its pins; the cradle link's other loads (the bottle, the box, the jam's
  couple) spread over its tie rods as a rigid box would. Each chord's axial force and
  moment, rated at the end's hole on the net section, and Euler out of the plane for a
  chord in compression. A tree of chords is rated by `|r x F|` about each joint: B's bell
  crank b2 as an L held the jam at 0.80 at its pillar's hole (the generic rule read 2.61),
  so it is a triangle now. B's cradle link b6 in 0.125 in held 0.32 at X under the jam.
- **Posts on the box**: the pillars and columns between a side's plates, solved as one
  frame. Each post is a steel beam, its foot fixed in the outer plate's plane, its end moving
  with the inner plate, which is rigid in its plane and held by nothing but the posts. A
  plate on two posts in a line (B's bar O1-O2 alone) turns about one and barely holds the
  other across the line. A post's tapped ends are rated on the tube the tap leaves, the rest
  on the bar. Until round 2 each end was held by springs of its plate's 24 mm strips
  (`cradle_structure.plate_spring`, `cradle_structure.posts`).
- **A side as one frame** (`cradle_structure.side_frame`, design review round 2, 2026-10-15):
  the posts (pillars and columns) and both frame plates solved together. Each plate is a
  grillage out of its plane (its members' weak-axis bending and twist), the outer one standing
  on its feet, held by their cleats; the inner one held by nothing but the posts, which are
  stiff along their axes; each post's ends turn with its plates. The gearbox's shaft carries
  the rod's force from the crankpin, so it turns the outer plate at the crank by that force
  times the rod's height over the plate (`cradle_report.rod_height`, 41.9 mm). Until round 2,
  each post's ends sat on springs of its plates' members, clamped at their far nodes
  (`plate_spring`, kept for `cradle_structure.posts`). Those springs held no plate's own
  equilibrium out of its plane. They also ignored the gearbox's turn, and the inner plate
  twisting as a whole. An independent 3D frame of 12-dof beams (the review's probe, in
  `/root/.cache/spiderpig/tmp/r2-structure/frame3d.py` on the development box) agrees with it
  within about 10 %. A unit test holds its portal case (`test_a_side_frame_holds_its_posts_only_as_its_plates_do`).
- **Each plate out of its plane** (`cradle_structure.plate_rows`, round 2's B1): every member
  end rated on the plate as cut:
  - at a node on a straight run (a column on a leg, a foot's screw), the net
    section through its hole (the boss round it where wider);
  - where members meet, at its strip's root (`strip_root`), at the moment there.

  The rows had spread a post's end moment over the summed sections of every member at the
  node. The review's own probes had applied the posts' moments 90° off: the slopes'
  generalized moment as the turn's vector (`test_a_posts_foot_turns_square_to_its_load`). Its
  "196 MPa at D1's column hole" is 145 MPa turned right, on the springs' moments. On the side
  as one frame it is worse elsewhere.
- **The frame plates' members** (`cradle_structure.members`): each plate a plane frame of
  its members under the posts' feet (and the gearbox's force and torque at O), standing
  on its feet. Each member's in-plane stress is rated, and its Euler load out of the plane
  between nodes the box holds. Where a node isn't held (the gearbox's O), its tilt on what
  holds it across is rated: since round 2 that is the side's own stiffness there
  (`cradle_report.side_stiffness`, 12.4 N/mm at B's O). The members there as cantilevers had
  given 2.9 N/mm and ranked O-Fr's sway first (round 2's minor 3).
- **The cradle box across the bay** (`cradle_report.box_racking`, round 2's M1; a 3D frame
  since simplification round 1, 2026-10-21: `cradle_structure.box_frame`). The passive side
  holds its cradle link in place but lets it turn about the mouth: B's double
  parallelogram, A's hinge at G.
  - So what the passive side can't hold crosses the bay in the box: walking half of what
    the drive side's pins put on the cradle about the mouth (`cradle_report.PASSIVE_SHARE`:
    the bottle and the box straddle the bay's middle); jammed, all of the clutch's couple
    (`cradle_report.JAM_SHARE`, 1.0 since simplification round 1; half until then). A block
    on the passive side's linkage puts the whole couple across the bay; a block on the
    bottle or a saddle (a decanter's rim under the mouth) puts it through the drive half,
    the passive half carrying nothing: the same twist in every rod, less bending. The whole
    couple across the whole bay holds both.
  - The box: both cradle links as grillages of their chords out of their plane, at their
    cut widths (`cradle_report.box_links`; B's X-Y split at the mouth on it), each held at
    its pins three ways (`cradle_structure.BOX_HOLDS`: each pin fixing the link, holding it
    across only, or nothing but the box held across at one pin; the truth lies between and
    the rows take the worst); the four tie rods as 3D beams on their tie nodes; each end
    joint rigid in bending (its face shut: `cradle_structure.TieRod.gap_nmm`; gapped, the
    screw is still far stiffer than the link) and in twist up to its friction, the face's
    and, the screw threadlocked into the rod, its head's (`TieRod.slip_nmm`, at
    `cradle_report.SLIP_MU`: 6.84 N·m with the class 10.9 M6 at 6 N·m, 3.58 the face
    alone), past it slipping.
  - Until then the links were held rigid out of their plane, each rod fixed at both ends,
    and the box racked on the rods' bending (B 565 N·m/°). A link is a 4.75 mm plate, 5-13
    x softer out of its plane than the rods it was to fix (the design review's finding,
    `r1/review_box.md`): the box racks on the rods' twist (B 44-52 N·m/° by the hold; A
    45-50), most of its moment crossing as the rods' torque.
  - Walking (25.4 N·m across) it holds: B racks 0.49-0.58°, A 0.51-0.56°; each rod's
    joints take up to 5.77 N·m of twist (A 5.57), under their 6.84 (`slip_x` 0.84, A 0.81:
    none slips, no fretting); the faces stay shut (B 4.98 x their gapping moment, A 4.33);
    the rods' tapped ends 7.31 (A 7.61), the cradle link out of its plane with its in-plane
    stress 5.26 (A 5.70). The mouth doesn't move (both remote centres are on one axis); the
    saddles at mid-bay lag the drive side by about half the angle. The rear carrier's two
    side blocks turn 0.37-0.43° against each other, and its printed bridge under the V
    block (`cradle.REAR_BRIDGE`, 6 mm deep since simplification round 1: a forced bar's
    stress grows with its depth) takes it at SF 8.5-10.1 across its layers (bar 2.5, each
    pour; 1/2 in 6061 rods 3.87, where the 10 mm bridge had 2.33); the front
    carrier's ends turn 0.19-0.21° on its rods, its bores' play 0.86°.
  - Jammed, unbraced, it doesn't (A today; B until simplification round 2): every joint's
    twist passes its friction (B 43-50 N·m stuck against 6.84, A 53-59), they slip, and
    what's left goes into bending the links out of their plane, far past yield (B at its
    63.2 N·m cap, 221 N·m across: the rods 0.40, their screws 0.86, the link 0.18; A at
    64.9, 262 across: 0.55, 1.02, 0.17 on 6061 rods), the box racking tens of degrees on
    the linear model. Half the couple (the old case) fails too (the link about 0.5). The
    box holds about 55 N·m across at SF 1.3 (B and A, steel or 1/2 in 6061 rods, the worst
    hold; 92 with dry faces, steel on aluminium's least 0.47), a quarter of the cap's, and
    slips from 30-36 N·m: on A an open item at the clutch's cap
    (`cradle_report.OPEN_AT_CAP`, the user's call).
  - No cheap fix closes it: a second screw doesn't fit a 12-16 mm rod's end with its M6
    tap, and a key there would bear at 200-400 MPa in the 4.75 mm link; threadlocker is
    already on; a wider chord only slows it; dry faces (threadlocker on the thread only)
    give 92 N·m; a lower cap would have to sit under the running peak (55 / 221.1 of 63.2
    N·m is 15.7).
  - **B's racking brace** (simplification round 2, 2026-10-21, the user's choice;
    `decanters.BRACE`, `cradle.BRACE_TUBE`, `cradle_structure.BoxBrace`): a 2 in x 0.065
    in 6061-T6 round tube (McMaster 9056K83, sawn from a 1 ft stick in the rods' cart:
    `cradle_catalog.BRACE_STOCK`) across the bay behind the rear saddle and
    under the base stop's rails, its axis at (412, -124) in the cradle's frame, 430 mm from
    the mouth (its front 6.6 mm behind the rear carrier, its top 7.6 mm under the rails).
    - Why a tube: a brace in one plane (a shear panel or straps in the rear rods' plane, a
      gusseted plate behind the saddle, a floor) carries the twist as shear in its own plane,
      and a shear across the bay needs an equal couple out of each link's plane at its ends
      (about the whole couple for a panel at the rear rods, 78 N·m for one behind the
      saddle): the links can't (0.15-0.45), the rods' own failure. A closed section carries
      a twist in torsion and puts it into each link in the link's own plane, as a couple
      round its socket. Under the bottle between the saddles the decanter stands at rest;
      the rear carrier rides the rear rods; in front of the mouth the stream and the gauge.
    - Each end is keyed into its cradle link: a laser-cut socket (`cradle.BRACE_SOCKET`:
      53.2 mm across, 12 teeth 6 mm wide standing in to 46.9 mm, their roots rounded r 1),
      the tube's ends notched for them at home, one end marked from a socket, the lines
      carried along the tube to the other, sawn and filed to fit, 0.1 mm over the teeth as
      cut at most (`BRACE_CLEAR`: its free turn 0.47° both ends, `cradle_report.box_brace`),
      its ends 0.25 mm short of the links' outer faces. No screw: the teeth key it, the rods'
      screws close the box on it. 192 g; GJ/L 254 N·m/°.
    - b6 carries the couple from the socket to its pins in its plane
      (`cradle_structure.link_bending`'s `couples`: all of the box's moment at the socket):
      its rear arm X-Tr1 64 mm (48 before), Tr1-Tr2 64 (24), a chord Tr2-Br 76 (the socket's
      pad), a boss r 50 at X spreading it into the chord Tf2-X, which can't grow (pillar O1).
      The plan is unchanged; b6 is about 0.3 kg the pair heavier.
    - b6 is cut to a band across the bottle's axis (`config.PRODUCT_BAND`, the round's way
      back): from 0.5 mm under Br's pad (the socket's web whole) to 1 mm under the front
      saddle's lip gauge's top, its corners R5 (`plates.BAND_CORNER_R`). What grew inward
      stays; what stood out past the box's other parts goes (X's boss's top, the rear arm's
      round ends, the rear chords' lower edges near Tr2), and every section is rated on what
      the band leaves (`cradle_structure.Band`, `strip_section`: an eccentric net section).
      The cradle box stands about 212 mm tall in the pack (249 uncut, 178 before the brace).
    - In `box_frame` the tube is a 3D beam on the links' node Br, rigid in bending at its
      ends, its twist free in its play and borne past it, never slipping; its rows
      (`box_racking`): the tube on 6061-T6's 35 ksi (the Aluminum Association's Table 12.1),
      its teeth bearing on the link's 5052 and the tube's wall (all twelve sharing: the 5052
      gives a few hundredths of a millimetre at the first jam), their roots in shear.
    - At B's cap (the round's integrated head; the rows at 63.9 N·m, 223.6 across, before
      the way back took the cap to 63.7): the box racks 1.24°, the tube takes 195.9 N·m (88
      %) at SF 4.45, its teeth bear at 84.7 MPa (1.87), the rods 9.12, their A2-70 screws
      18.9, the cradle link 1.40 (Tf2-X past X's boss: the couple through the arm, in its
      plane). The tie rods' end joints, A2-70 at 4 N·m since the round's P8 (`cradle.TIE_SCREWS`),
      reach their friction in twist there and slip past it (`slip_x` 1.11; threadlocked into
      the rods, the screws keep their preload), their faces shut (1.65). Walking (25.6 N·m
      across): 0.52° (the tube in its play to 0.47°, then 13 N·m), the rods 20.7, the link
      7.50, `slip_x` 0.47, the faces 3.93 x their gap moment. (On the brace's own branch, its
      class 10.9 screws at 6 N·m: the tube 193.9 N·m at 4.50, no joint slipping, 0.74.)
    - Not on A: A's hand crank (130 mm on a 230 mm reach) sweeps the bay's drive end behind
      the rear saddle, the tube 16-28 mm inside a hand on its grip wherever it goes there (a
      reach of about 300 mm clears it and its socket's pad: a probe, the rest unchecked),
      and the reshaped link the brace needs (a chord E-Br over the crankpin, which passes
      under Tr2) put A's worst start 4-5 % up and its grip at 22.1-22.3 N run in, over the
      22 N bar (a 132 mm crank would bring it back under).
      A's box stays open at the cap (`OPEN_AT_CAP`); the user's call.
  - Rods with stiffer twist attract more of it: rear rods of 5/8 in 6061 on A2-70 screws at
    4 N·m (the stand review's R1) slip walking (their joints hold 5.14 N·m, the rear rods'
    stuck twist 6.5-8.0).
  - Since round 2 the rods are MISUMI NETRF12 (12 mm, M6 x 12 deep each end): USD 21.07 at
    one to four, 15.33 at five, 7.73 at ten (NETRF12-274.9's page, rendered 2026-10-15;
    NETRF10's 15.30, 11.08 and 5.57).
  - Their centres sit 0.95 mm lower in the cradle's frame (`decanters.TIES`), so their tops,
    which the V blocks sit on at a 6 L's setting, stay where the 10 mm rods' were. The
    carriers keep 6.05 mm of print under them: an M3 x 6 set screw and 0.05 mm.
- **The side's stiffness across** (`cradle_report.side_stiffness`, round 2's M3): a push
  across the plates at a pivot, on the side as one frame.
  - B's O1 takes 8.18 N/mm, its O2 39.4, since design review round 3 (2026-10-20; 6.35 and
    29.3 on the round's old model and frames, round 2's 2.67 and 12.8). A 20 N hand sideways
    on the bottle, both sides sharing it, sways the pivots 1.22 mm, and each side rocks 2.63
    mm more at O1 on its rail's play in the cleats: 3.86 mm (A's G: 9.17 N/mm, 1.09 and
    3.28 mm, 4.37; it had read 9.01 N/mm and 1.11 mm through the solve's ridge, round 3's
    M4).
  - The review's grillage of the outer plate alone had 1.41 N/mm and 7 mm.
  - Round 2's wider front leg (B1) is the cheap improvement taken: 2.0 N/mm on the side as
    one frame with the 24 mm leg.
  - The test holds the sway under 5 mm. A gusset or a cross-tie doesn't reach the pivots:
    the sway is the outer plate's legs bending from the board.
- **The rod as a beam-column** (`cradle_report.rod_secant_sf`, round 2's minor 1). Its far
  pin spans two layers, so the rod's force comes in half that span off its mid-plane: 5.97
  mm at B's P, 4.58 at A's E.
  - Pinned at the crankpin, the moment grows by 1/sin(kL). Against the sheet's yield that
    gives B 1.9 jammed (its Euler 1.78), A 2.33 (Euler 2.3), at the merged drive's caps.
  - b1 is 26 mm wide since round 2 (the parts' M3).

**The known limits: a blocked cradle** (`cradle_report.blocked`, the report's `blocked`).
The jam case is a couple on the cradle. A real block is a force at a contact, and near the
mouth its lever is short. The same cap, 63.9 N·m on B and 67.2 on A, against a block (the
frames of design review round 3, rated on its model; round 2's in brackets):

| block at | B: the block's force | B: weakest | A: weakest |
|---|---|---|---|
| the front tie Tf1 (the front saddle, the light, the front ties on a raised decanter's rim), 73 mm from the mouth | 3.54 kN | the inner plate 0.24 (the outer plate 0.16, pillar O2 0.20) | column E2 0.20 (the outer plate 0.18) |
| the rear saddle's underside, 351 mm out | 0.74 kN | the outer plate 0.90 (pillar O2 0.48) | pillar G 1.02 (the outer plate 0.42) |
| a 6 L's base at its top edge (a shelf or a cabinet's front), 563 mm out | 0.46 kN | b6 0.80 (pillar O2 0.61) | b2 0.57 (the outer plate 0.5) |

**Where it starts to give** (`blocked`'s `threshold`, round 2's M2). The loads are the
walking ones plus the push beyond what turning the cradle takes, `dT = T - T_walk >= 0`
(a block only pushes back), superposed: the statics are linear in the crank's torque. Every
part is rated as the report rates it. The extra force at the grip, through the worm at a
hand's speed, is on top of what the turn already takes (up to 21.7 N on B, 22.0 on A, at
the worm's static efficiency). Each angle is pushed as far as its own clutch lets it (design
review round 3's M3): where the bottle's weight fights the turn, 14.4 N more on B, 15.5 on
A; where it helps, up to 47.7 N on B and 48.2 on A. (Until round 3 the search stopped at the
least, and so read "none within the clutch" at the rear saddle.)

| block at | B: first to 1.3 | B: first to 1 | A: first to 1.3 | A: first to 1 |
|---|---|---|---|---|
| the front tie Tf1 | the inner plate, +6.2 N at the grip (620 N at the tie) | the inner plate, +8.1 N (817 N) | pillar G, +6.7 N (805 N) | pillar G, +8.9 N (1076 N) |
| the rear saddle | the outer plate, +21.0 N (439 N) | the outer plate, +37.5 N (735 N) | pillar G, +19.6 N (491 N) | none within the clutch |
| a 6 L's base | b6, +20.6 N (268 N) | b6, +32.7 N (424 N) | b2, +14.3 N (222 N) | b2, +19.8 N (308 N) |

(Round 2's side frames: B's outer plate first at the front tie, +4.1 and +5.8 N, and at the
rear saddle and the base too, +9.7 and +9.9 N; A's outer plate at +4.3 and +6.1 N at the
front tie, +7.4 and +8.6 N at the others. A's pushes are smaller than before the merge on
its 130 mm crank: they had been at the grip of round 1's 85 mm one, +6.7 and +9.3 N at the
front tie.) So a decanter raised into the front ties starts bending B's frame at about 7 N
over the normal turn, which a hand
can't feel against the turn's own 22 N. No clutch a hand can turn helps a block 73 mm from
the mouth. Since round 2 the decanter stands on a printed riser that can't be raised
mid-pour, its rim 20 mm under the lowest part over it ("The vessel under the mouth"), and
the use limits keep everything else from under the box. (Until round 2 this said the glass
decanter would break first; nothing supports that, and the review found no rim-strength
figure to cite. At the 40 N push, before round 1's drive: B's outer plate 0.17, pillars
0.30 and 0.34; A's plate 0.21.)

**The side frames at the clutch's cap (design review round 2, 2026-10-15; redesigned
2026-10-20).** Round 2 rated each side as one frame (`cradle_structure.side_frame`) and
found that it held a full 6 L walking but not the clutch's cap everywhere. At the merged
drive's 63.7 N·m, B's outer plate's front leg held 0.60 (D2 to the foot), pillar O2 0.73 and
column D2 0.97; A's outer plate 0.41 (the front leg from column D3 to the foot, the strut G-O
0.63 at the gearbox), column D3 0.83, its inner plate's leg D2-D3 1.09 and pillar G 1.25, at
67.1 N·m. Two things made it so:
- **The posts stood almost on one line.** B's O1, O2, D1 and D2 lie within 66 mm of the line
  down the front, A's G, C1, C2, D1-D3 on its front leg, and the drive's force runs mostly
  across that line.
  - The inner plate floats on its posts and is rigid only in its plane. Nothing held it
    from turning about that line, so the posts' ends turned freely across it.
  - The box swayed in the drive's direction, and pillar O2 bent nearly as a cantilever off
    the outer plate: 49.8 N·m at its foot, where round 1's springs had read 26.7.
  - The links sit near the inner plate (B's b2 to b6 in layers 12-14 of 15), so most of
    the pillars' load goes to it: up to 1.4 kN at each of O1 and O2 at the cap.
- **The gearbox turns the outer plate at the crank.** The rod's force reaches the frame at
  O through the gearbox's shaft, 41.9 mm under the rod (`cradle_report.rod_height`), and
  only the outer plate's strips carry that moment out of their plane there.

**The redesign** (`cradle.STANDS`, the user's brief of 2026-10-20: every part 1.3 or more
at the cap):
- **B: a box toward the rear.** Two more 12 mm columns, E1 and E2, stand on the rail 220
  and 280 mm behind the mouth, and the inner plate runs a 40 mm bottom chord from D2 to
  them (its leg D1-D2 32 mm).
  - The posts then stand across the drive's force, so the inner plate is held in its plane
    and the box holds the posts' ends.
  - The rear column resists the inner plate's turn in its plane, and takes the most: its
    foot puts up to 26 N·m into the rail.
  - So the rail is 44 mm wide (`cradle.rail_y`): its bottom edge stays 0.5 mm over the
    board, between its cleats. Its line, its feet's screws and the columns stand 10 mm
    higher, so the columns' bodies and heads clear the cleats' tops by 7.5 mm. The feet's
    printed brackets' uprights stand 10 mm taller with them.
  - For the gearbox's turn: the rear leg O-Fr is 56 mm wide and O1-O 40. O2-Fr is 32,
    against its buckling out of the plate.
- **A: a column under the front leg, and the strut widened.** A column E1 stands on the
  rail 80 mm in front of the mouth, under D3, and the inner plate's leg runs down to it,
  40 mm wide.
  - Its posts then hold their ends; the inner plate's legs need no widening.
  - The gearbox's turn goes mostly into the strut G-O, so it is 96 mm wide (48 before). The
    legs from O (Ff-O, O-Fm, O-Fr) are 40 and the rail 40 mm, 8 mm higher: its edge and
    the 40 mm leg to E1 0.5 mm over the board.
- Every new column is the stand's own: a NETRF12 of the side's post length on M6 x 16
  A2-70 button heads and DIN 125 washers. The guide's column steps take them as they are.
- No column stands in a link's sweep: the nearest, B's E2, is 94 mm from the crank's. The
  inner plates' new members pass well under the tie rods' paths (the lowest, Tr2, comes
  down to 364 mm under the mouth on B; the chord's top edge is at 409 or lower).

**What they hold** (`cradle_report.report`):

| | B, 63.8 N·m | A, 67.2 N·m |
|---|---|---|
| the worst part at the cap | pillar O1 and b6 1.37 (unchanged) | the outer plate 1.37 (G-O at O) |
| the side frame | the outer plate 1.39 (the rail at E2), column E2 1.40, pillar O2 1.58 (0.73), the inner plate 1.61, column E1 2.01, column D2 2.66 (0.97) | the outer plate 1.37 (0.41), pillar G 1.40 (1.25), the inner plate 1.45 (1.09), column D3 1.61 (0.83), column E1 1.77 |
| walking, the worst | the crank's keyway 4.15; the outer plate 4.7 (2.83) | the crank's keyway 3.94; pillar G 6.15 (3.71 the side frame) |
| the cap over the whole turn | 1.34 or more 30° and more from the toggles (pillar O1) | 1.36 or more (pillar G) |
| the side's stiffness across | O1 6.35 N/mm (2.67), O2 29.3 (12.8); a 20 N push sways the pivots 1.57 mm (3.75) | G 9.01 N/mm (2.92), 1.11 mm (3.4) |
| weight | the cradle 1.24 kg heavier (each side panel 0.61 kg) | 1.10 kg heavier |

(Round 2's numbers in brackets.)

**The options weighed** (on B; the model of `cradle_report.structure` with the stand varied,
each rated at the cap; the prices from the build's BOM):

| option | worst at the cap | parts added | cost |
|---|---|---|---|
| **two rail columns at 220 and 280, the widths above (built)** | 1.37 (the frame 1.39) | 4 columns, 8 screws, 8 washers | +USD 66.70: the 4 standoffs come from the 10 the order already buys for its 6, so USD 0; the inner plate +53.10 (its blank, 361 x 67 to 525 x 282 mm), the outer plate +10.16, the screws and washers +3.44 |
| three rail columns (round 2's concept), the same widths | 1.36 at 160/230/290; 1.25 at round 2's 60/180/290 | 6 columns, 12 screws, 12 washers | about +USD 80 (2 more standoffs, the tier's +11.98) |
| one rail column at 240 | 1.28 (the column itself) | 2, 4, 4 | about +USD 65 |
| the inner plate tied to the base board (a tab in a routed notch, no columns) | the posts hold (O2 1.43-1.59), but the rail then carries the rod's reaction to the feet and buckles at 0.75-1.13 | a notch, two cleats | not cheaper: the same inner plate, and a 4.75 mm tab bearing on the birch's edge, found blind on the way back |
| the frames in 0.250 in (6.35 mm) alone | pillar O2 0.77, column D2 1.02 | none | about +USD 75 |
| widths alone (round 2) | pillar O2 0.81 | none | |

The rear column takes the most whatever their number: a third one between doesn't share it
(the rear column 1.25-1.38 with three, 1.40 with two), and alone it fails at 1.28. On A one
column under D3 does it (two, at -100 and 60, 1.38): A's BOM +USD 51.04, the outer plate
+17.16, the inner plate +20.18, two standoffs +11.98 (12 to 14 at the 10-tier's price).

**The known limits change too** (the blocked cradle above, re-rated): a block at the front
tie now finds B's inner plate first, 0.26 at the cap (the outer plate 0.38, was 0.16), and
the frame starts to give at +6.7 N at the grip (was +4.1); at the rear saddle and a 6 L's
base no part falls under 1.3 within the clutch (+9.7 and +9.9 N before). A's front tie:
pillar G first, +5.3 N (the outer plate at +4.3 before); the rear saddle and the base:
nothing within the clutch.

**The structure model corrected (design review round 3, 2026-10-20).** Round 3's linkage
review modelled each side again in an independent 3D frame and found the report rating a
frame that wasn't the one built, and leaning on idealizations as large as its margins.
`cradle_report.structure` now:
- **Takes each post's span from the parts** (`cradle_report.post_spans`, `check_spans`; the
  review's B1). B's pillar O1 had stopped 15.9 mm under the inner plate (a NETRF12-59.3, no
  hole there) since round 2 ended it under b6, while the model tied it to the inner plate:
  SF 0.39 at the cap as built, read as 1.37. O1 runs through again (a cradle pillar's printed
  neck may thin to 1.2 mm, `standoff.NECK_WALL`, where a link passes it: b6 7.5 mm from its
  axis), on a NETRF12 of the other posts' length, screwed into the inner plate's new hole (its
  boss 2 t). A post that stops short of a plate, or a plate with no hole on its axis, now
  stops the report.
- **Solves the side exactly** (`cradle_structure._solve`, its M4): Jacobi-scaled, no ridge on
  the diagonal; a frame that isn't held raises `SingularFrame`, and every solve is held to
  its equilibrium. The ridge had propped A's frame: its stiffness across at G 9.01 N/mm
  where it was 5.23.
- **Rates both idealizations of the plates** (its minor 2): at their faces, where the posts
  end, and at their mid-planes, where they bend (each post's end a rigid half-thickness off
  it, so its shear bends the plate too), the worse per row.
- **Second order and buckling** (its M2, minor 1): the plates' members' in-plane forces and
  the posts' axial ones as a geometric stiffness on the side's bending out of its plates
  (`cradle_structure.side_frame_solve`): each angle solved with it (P-Delta), and the side's
  buckling factor per angle (`(K + lambda K_G) phi = 0`), a row of its own,
  `frame:side buckling`. On the old frames A buckled at 1.35 x the cap and B at 1.70; at
  1.35 the mode's deflections grow 3.9 x, which took A's column E1's foot from 24 to 67 N·m.
- **Combines each plate section's stresses** (its minor 1, `cradle_structure.combined_sf`):
  at the member's edge the axial, in-plane and out-of-plane bending stresses add, at its
  face's middle the twist's shear joins the axial and out-of-plane ones; they had been rated
  apart.
- **Rates every product part on its specified minimum** (its M1, `materials.allowable_mpa`):
  5052-H32 158.6 MPa (ASTM B209: 23.0 ksi; 193 typical had been taken), 6061-T6 241.3,
  7075-T6 at 0.190 in 475.7 (`sheet_catalog.MIN_YIELD`); the posts' steel 235 MPa, the
  specified minimum of the EN 1.0038 MISUMI names for NETRF (`cradle_catalog.NETRF_STEEL`;
  1018's estimated hot-rolled 220 had been taken). The walkers keep the typical values their
  rules were set on.
- **Rates a post's tapped ends at the female thread's roots** (its minor 3, 6.0 mm for M6),
  a pillar's link loads spread over its bore, and a link's chord past a boss (or a wider
  chord's end there) as well as through its hole.
- **Rates the posts' screwed ends** (its M5, `cradle_report.post_joints`): each end face (12
  mm on the plate's 6.4 mm hole) clamped by an M6 A2-70 button head at ISO 7380's most, 4 N·m
  (TR Fastenings' table; 6 had been drawn), 2.5-4.6 kN at friction 0.2-0.1 (A's pillar G's
  foot the tie rods' class 10.9 one at 6 N·m, 3.8-7.0 kN: `cradle.STEEL_FEET`, below); walking each face
  shut at the least and not slipping, and at the cap its screw within A2-70's proof at 1.3
  and the plate under it within VDI 2230's limiting pressure (5052 not tabulated: AlMg4.5Mn's
  325 MPa scaled to 5052's tensile, 269; its R10's S_P 1). At the cap every face lifts: the
  bare 12 mm edges would have dented the 5052, so an 18 mm fender washer goes on both ends
  of every post (`cradle.FACE_WASHER`; a pillar's inner one in the clearance gap under the
  inner plate), every post of a side one length (B's twelve NETRF12-71.8, A's sixteen 53.2:
  one part number, at its ten's price), and the guide takes the
  screws back to 4 N·m after a clutch slip.
- **Searches a block over the push each angle's clutch lets through** (its M3,
  `_block_threshold`): where gravity helps the turn, up to 48 N at the grip, not the least
  over the turn. A block under the rear saddle or a 6 L's base does give within the clutch
  (the tables below); the use limits' "nothing under the cradle box" is its guard.
- **Counts the rail's rock in its cleats** (its M6): the cleats glued against the rail itself
  (`cradle.CLEAT`, 0.02 mm of play a side; a sheet of paper's 0.15 had let each side rock
  +-1.3°, +-9.7 mm at O1), the side's sway the elastic one and that rock over the rail's 6.5
  mm in its cleats (`side_stiffness`'s `rock_mm`).

On that model both cradles' old frames fell well under the cap (B: the inner plate 0.86, the
outer 1.04, b6 1.12, pillar O1 1.23; A: the outer plate 0.39, column E1 0.47, the inner
plate 0.75). **The fix, cheapest first** (`cradle.STANDS`): wider members in the same 0.187 in
5052, where the model shows the moment: the frames in 0.250 in were weighed (A's buckling
3.11, every plate row 1.35 or more, but O1's boss then meets the tie rod Tf2, the drive's
stack moves and the cutting rises about 37 %), and 6061-T6 (+USD 170.62 for B's plates and b6,
the laser-cut parts review's live quotes; it doesn't lift A's buckling).
- **B:** the rail 60 mm (44), the rear leg O-Fr 80 (56), O1-O 72 (40), O2-O 88 (64), O2-Fr 40
  (32), the inner chord D2-E1-E2 60 (40), its leg D1-D2 44 (32), O1-D1 27 (24: as wide as the
  tie rod Tf2 leaves); b6's rear arm X-Tr1 48 (40); b2 back to 0.187 in 5052 (its worst
  stress at the cap 36 MPa: SF 6.3); b3 and b4 bossed r 14 at their 8.2 mm holes (SendCutSend's
  2 t in 7075, the laser-cut parts review's M3).
- **A:** a second rail column E2 under the mouth, the inner chord along the rail to it (40 mm,
  its edge clear of the riser locator), the rail 60 (40), the strut G-O 140 (96), the legs from
  O 56 (40), the front leg G-Ff 48, the triangles round G 40, C2-D1 34 (as wide as Tf1
  leaves), the inner leg to D3 40 and to E1 60 (40).

**What they hold now** (`cradle_report.report`; round 2's frames' redesign of 2026-10-20 on
the old model in brackets):

| | B, 63.9 N·m | A, 67.2 N·m |
|---|---|---|
| the worst part at the cap | pillar O1 1.37 (1.37) | the outer plate 1.38 (1.37: the rail at E1) |
| the side frame | column E2 1.46, the outer plate 1.52 (the rail at E2), the inner plate 1.53 (the chord at D2), pillar O2 1.54, column E1 1.99, column D2 2.56 | the inner plate 1.51, pillar G 1.57, column D3 1.76, column E1 1.81, column E2 1.89 |
| the side buckling out of its plates | 2.15 x the cap (1.70 on the old frame) | 2.34 (1.35) |
| the links | b6 1.48 (Tf2-X past X), b4 1.48 (its chord's buckling), b1 1.98, b3 2.46, b2 6.33 | b2 1.44, b1 2.25 |
| walking, the worst | the crank's keyway 4.13; the outer plate 4.51 | the crank's keyway 3.93; b2 7.15 |
| the cap over the whole turn | 1.33 or more 30° and more from the toggles (pillar O1) | 1.38 or more (the outer plate) |
| the posts' screwed ends at the cap | screws 1.95 on A2-70's proof; the plates S_P 1.29 (column E2's foot, on its washer); every face lifts (to 5.3 x) | 1.92 (G's foot, on the 10.9 head's 13.3 kN); S_P 2.16 (column D3's end) |
| walking | every post's face shut (0.99 at worst, O2's foot), none slipping (0.54) | every face shut (0.70 at worst, G's foot on its 10.9 head: 1.06 on the A2-70 one), none slipping (0.53) |
| the side's sway across | O1 8.18 N/mm (6.35 on the old model), 1.22 mm elastic under 20 N and 2.63 of rock: 3.86 mm | G 9.17 N/mm, 1.09 and 3.28: 4.37 mm |
| the tie rods' end joints walking | closed, 1.48 x the end moment at the least preload (class 10.9 heads at 6 N·m) | 1.47 |

The joints the round left open (`cradle_report.OPEN_JOINTS`, empty since 2026-10-20): on
ISO 7380's 4 N·m in A2-70 both cradles' tie rods' end joints stood at their gap walking
(0.98 at the least preload) and A's pillar G's foot lifted (1.06), under the walking bound
that puts the whole bottle on one side. The tie rods' eight end screws and A's two G feet
now take the same button head in class 10.9 steel at 6 N·m (`cradle.TIE_M`,
`cradle.STEEL_FEET`, `StandoffAxle.steel_foot`; Accu's SSB-M6-16-10.9, unplated, USD 0.44
each at 6): the head's shape is the same, so no claim, plan or part moved. A full-strength
A2-70 head (ISO 4017's 4.15 mm hex, ISO 4762's 6 mm socket) doesn't fit under the cradle
links (above), and only A's G foot could have taken one: one screw for both keeps one part
number and one torque. The guide says which (the plain-steel ones, their own bag, 6 N·m on
the key) and takes them back to 6 N·m after a clutch slip, the stainless ones to 4.

**At a higher cap** (the drive review's: a new box's setting, 72.5 N·m on B and 76.3 on A):
B's rod as a beam-column 1.07 (the drive's), pillar O1 1.18, b6 and column E2 1.28, the rest
1.31 or more, its buckling 1.92, the plates under the posts S_P 0.77; A's outer plate 1.16,
the tie rods 1.26, b2 1.27, the output shaft 1.28, the rest 1.32 or more, its buckling 2.08,
S_P 1.44 (column E1's foot; 1.09 at G's foot on its A2-70 head, before its 10.9 one).

**The clutch's cap** (rounds 1 and 2's drive reviews, 2026-10-14, and round 3's, 2026-10-20:
`config.PRODUCT_CLUTCH_NM`, `handcrank.clutch`, `clutch_ceiling`, `cradle_drive.clutch_case`,
the report's `jam_capped`). The hand crank's slip clutch is set to slip at 1.3 x the start the
user measures with the heaviest bottle in the built cradle (a full 6 L: 3.53 N·m on B's
input, 28.2 N at its grip; 3.71 N·m on A's, 28.6 N), checked within 8 % on its stub at every
set-up and monthly, no pull over the ceiling (30.5 N on B's grip, 30.8 on A's), so the crank
sees at most 1.3 x 1.08 x the peak x the worm's efficiency at 100 rpm over its static one
(0.558/0.44): 63.8 N·m on B, 67.2 on A, however hard the push (63.0 and 66.2 before round 2's
12 mm tie rods; 63.7 and 67.1 before round 3's saddles, whose full 6 L starts 0.002 N·m
harder: `handcrank.DESIGN_START_NM` 2.714 and 2.858), set on a new box or a run-in one alike (round 3's M1: 72.5 / 76.3 without
the ceiling; spun at 200 rpm, 67.9 / 71.5). That is a bound while the check is kept, and the
jam case; the 40 N push that had been the case isn't one (a hand on a crank that feels stuck
pushes 60-150 N). Unchecked, the greased friction and the seats' wear can carry the slip to
1.47 x its setting (B's cap 86.8 N·m, over the gearbox's 85 maximum) or down to 0.72 x (it
ratchets on the start), and dry it locks (`handcrank.clutch_drift`, the report's
`drift_unchecked`). The report rates the cap over the whole turn, the toggles included:
- The report's own model rates it (`cradle_report.structure` with no toggle skipped, the
  links chord by chord, the posts on their box, the plates' members, the gearbox's output
  shaft; since round 2 each side's posts and plates as one frame; since round 3 on its
  corrected model): 30° of crank and more from either toggle every part holds it, B at SF
  1.33 or more (pillar O1; column E2 1.43, b6 1.45, the rod as a beam-column 1.46), A at 1.38
  (the outer plate; the tie rods 1.43, b2 1.44, the output shaft 1.46), the side frames
  included since round 3's redesign ("The structure model corrected"; their redesign of
  2026-10-20 had read B 1.34 and A 1.36 on the old model; round 2's had held B's outer plate
  0.58, pillar O2 0.7, column D2 0.95, A's outer plate 0.41, column D3 0.83, inner plate
  1.09 and pillar G 1.21 there). (On round 1's springs: B 1.33, pillar O2; A 1.48, the
  output shaft.)
- Nearer a toggle the rod's force goes as the cap over the sine of the crank's angle to the
  rod, and no torque bounds it. B: every part holds 1.3 outside ±30° (the last 6.2° of tilt
  before rest, 7.6° before full); pillar O1, the outer plate and column E2 1.17-1.18 at
  ±25°, 0.96-0.99 at ±20° (round 2: pillar O2 0.6 at ±25°, 0.49 at ±20°); the rod buckles
  (elastically, into its 3 mm gaps) within ±16° of rest and ±9° of full tilt. A: every part
  holds 1.3 outside ±30° (the last 5.5° of tilt before rest, 8.5° before full), the outer
  plate 1.14 at ±25° and 0.91 at ±20°, the output shaft 1.23 and 1.0; the rod buckles
  within ±12° and ±6°. Only clearance protects there: the use limits keep everything from under the
  cradle box and the bottle in the first and last 2.5 handle turns, which the guide marks
  on the gearbox's far face (`handcrank.TOGGLE_TURNS`).
- The gearbox at the cap: 1.33 x under its 85 N·m maximum on B, 1.27 on A
  (`cradle_drive.GEARBOX_CAP_FLOOR`, 1.25). (Round 1's NRV040 had the cap 1.36 x over its
  45.)
- The output shaft's radial load (`output_overhung`, round 2's M-2): the rod's force at its
  mid-plane, 69.4 mm past the hub's end on either, where Motovario allows Fr2 a /
  (b + x) = 3363 N on the NRV050 (p. 44: 4840 N, a 101, b 76, a static or
  limited-operation maximum; Phidgets gives 4.8 kN, nowhere placed): walking SF 8.2 on B,
  7.43 on A; at the cap 1.74 30° from the toggles on B (1.3 outside ±25°, 1 outside ±20°),
  1.46 on A (1.3 outside ±30°, 1 outside ±20°). (Round 1's NRV040
  had been 1.31 and 1.16 at its cap, unrated.)
- The input shaft's radial load (`input_overhung`): the grip's middle 162 mm past the
  input's shoulder on B (292 on A's 230 mm reach), where the catalogue allows Fr1 a / (b +
  x) = 229 N on the NRV050 (p. 43: 490 N, a 129, b 114; 156 on A); the clutch's top is 30.5
  N at B's grip (SF 7.5), 30.8 at A's (5.05).
- The keyed plates hold 2.33 x the cap filed together on the key (2.21 on A), rated on
  the sheet's thinnest (`handcrank.PLATE_UNDER`); one plate bearing alone 0.78 (0.74), and
  1.38 at B's walking peak (1.31); the guide files them together.
- The bars are the tests' (`tests/test_cradle.py`): the clutch's least slip in its check's
  window at least 1.1 x the worst starting turn (1.2 on both: 1.3 x 0.92), every part at
  1.3 or more 30° from the toggles (but A's cradle box since simplification round 1:
  `cradle_report.OPEN_AT_CAP`; B's braced since round 2), the output shaft 3 x walking, the
  gearbox's maximum 1.25 x over the cap.

Every joint on both sides is rated at the full planar load (`cradle_report.SHARE` = 1),
not a share of it. The rod drives one side, whose pivots carry half the weight plus the
rod's whole force.

`spiderpig/cradle_report.py` (`report(config)`) measures:
- **Swing:** the swing and each bottle's own mouth wander.
- **Drive:** the crank's peak torque, the gearbox's safety factor on its 85 N·m maximum
  walking and at the clutch's cap, the worm's efficiency (static run in and new, at the
  hand's 100 rpm), the force at the crank's grip at both static efficiencies against
  MIL-STD-1472H's 22 N, the jam's torque (the clutch's cap) and the 40 N push's beside it,
  the turns per pour; the clutch's setting, its check's window and margin over the worst
  start, its cap, its unchecked drift and the cap that would give, the plungers' force
  against their range; the input and output shafts' radial loads, the keyed plates, the
  end screw's torque on its grade (`cradle_drive.drive_limits`).
- **A hand on the grip** (`cradle_report.hand_sweep`, a slow test at 1°): the crank's swept
  solids and a hand round its grip against every other part over the whole turn, by
  adaptive surface sampling (round 2's M-3).
- **The clutch's cap** (`jam_capped`, `cradle_drive.clutch_case`): every part at the cap over
  the whole turn, per excluded band round the toggles (0 to 30°), and where each falls under
  1.3 and 1.
- **Rod and crankpin:** the rod's buckling, the crankpin standoff as a cantilever off the
  block.
- **Stand:** the whole cradle's centre of mass over the swing against the base board's
  feet (the board where `cradle.BASE_Z` puts it), each bottle at its own setting: the
  margin, the tilt it tips at and the push at the bottle's top that tips it, each the worst
  over the swing (the code review: they had been read at one crank angle), and per edge
  (`edges`: front, rear, the drive side, the passive side). `cradle_report.stand` runs it
  alone: `python -m spiderpig.cradle_report stand decanter_rcm`.
- **The empty cradle:** the grip force with the bottle out (`empty_handle_force_n`), the
  bench check after it goes back together (`cradle.EMPTY_GRIP_N`).
- **Joints and plates:** every pin, pillar, column, link plate and frame plate rated by
  `strength.check` at both cases (`strength.design_loads` takes the report's loads for a
  cradle), the links, posts and plates from the force vectors (`structure`).
- **Blocked:** the known limits, a block at the front tie and at the rear saddle
  (`blocked`).
- **Saddles:** the loads and stresses of the saddles' printed parts and steel, the lip
  gauge's and the base stop's, and where each bottle's weight centres on them at rest,
  level and full tilt (`cradle_report.saddles`, `cradle_report.balance`: "The saddles,
  adjustable").
- **Clashes:** the whole cradle's clashes over the swing, the hand crank's swept volume
  against everything, and every bottle at its own setting, its V blocks and base stop
  moved (`cradle.bottle_hits`, `cradle.at_setting`).
- **Keepers** (B): each quick-release pin's TPU keeper's grip on its pin at the least
  interference and modulus, against the pin's weight, and its push on by hand
  (`cradle_report.keepers`, "Flat pack").

`tests/test_cradle.py` holds the thresholds, for A:
- each bottle's mouth wander under 0.5 mm;
- tipping at 15° or more, and 25 N or more at the bottle's top, the worst over the swing;
- the crank's grip force at a sealed full 6 L's worst start under 22 N (MIL-STD-1472H,
  Figure 13) at the catalogue's run-in efficiency, its throw and grip within the figure's
  row for its load: B's light-load crank within 125 mm, a grip 25-75 long and 10-16
  across; A's 5 mm over (`handcrank.HANDLE_OVER_RADIUS`, no crank inside a row holds both
  bars on A); the report rates the new box's too (A's at the bar, open:
  `cradle_report.OPEN_GRIP`, held no worse, then xfailed);
- the gearbox's safety factor at least 1.3 on Phidgets' 85 N·m maximum walking, the peak a
  sealed full 6 L at one tilt, and its maximum 1.25 x over the clutch's cap;
- the clutch's least slip in its check's window 1.1 x the worst starting turn or more, its
  cap the jam case's torque, the cap the config's, the plungers' force within their range
  for every set-up; the input shaft's radial load at the clutch's top at least 2 x under
  the catalogue's, the output shaft's walking 3 x; the toggle zones within the guide's
  marks (`handcrank.TOGGLE_TURNS`);
- the crank's swept solids and a hand on its grip clear of every part over the whole turn
  at 1° (`cradle_report.hand_sweep`);
- every pin, pillar, column, link and frame plate at least 3 walking and 1.3 jammed (the
  links, posts and plates from the force vectors), the side frames included since their
  redesign of 2026-10-20; the keyed plates at least 2.5 walking
  and 1.5 jammed; the crankpin and the rod 1.5 jammed;
- the saddles' printed parts under 10 MPa at a full 6 L's worst, the post and the
  screws 3 x under their yield, the base stop's lock gripping and the rear lead screw's
  brake holding 3 x what they hold; every bottle's weight centred on its saddles by 40 mm
  or more at rest, level and full tilt, and the lip gauge falling back from its stop;
- on B, each quick-release pin's keeper grips it with 10 x its weight or more, and goes on
  with under 60 N;
- the cut rules pass, every printed part fits a 250 x 210 x 210 mm bed in its pose;
- the assembly order covers every part.

```bash
uv run python -c "import json; from spiderpig.config import BuildConfig; \
from spiderpig.cradle_report import report; \
print(json.dumps(report(BuildConfig(linkage='decanter_hinge', robot=False)), indent=1))"
```

## B, the virtual hinge

B (`decanters.RCM`, refined 2026-10-10) swings −50° to +30°: one crank turn is one decant
and back, the crank-rocker's toggles the soft stops at both ends. It shares the drive, the
cradle box, the saddles, the base stop, the base board and the light with A.

**The mechanism.** Two parallelograms keep the cradle turning about the mouth with nothing
there:
- **The pivots** O1 and O2 hang straight under the mouth (a ray 90.5° down), 105 and
  307 mm from it.
- **The arms** b3 (O1–J) and b2 (O2–K) are 174 mm long. They swing 8° ahead of the
  bottle (`skew`).
- **The bar** b4 (X–J–K) translates, and the cradle b6 rides it at X.
- **The cradle link** b5 (Y–J2) holds the cradle's other pin. J2 lies 110 mm past O1 on
  the copy arm's own line (`y_across` 0).
- **The drive:** the crank O sits 366 mm behind and 322 mm under the mouth. A 336.2 mm rod
  runs from its 93.5 mm crank to the lever on b2 (147 mm, 64° off the arm).
- **The bell crank** b2 is a triangle, O2–P–K (since 2026-10-14): as an L its lever's root
  took the whole jammed cradle moment, 230 N·m, and held it at SF 0.80 through its
  pillar's hole. The chord P–K carries it round as axial force (1.7 kN, Euler 3.05 x).
- **The cradle link** b6 has the chord X–Tf1 across its front quad X–Y–Tf1–Tf2, which bent
  at X as a four-bar frame under the jam; its rear arm X–Tr1, a cantilever off X carrying
  the rear saddle, is 40 mm wide, and X–Y, 284 mm in compression, 32 mm.

**Why it looks this way.** The old B rested at −10°. Its −25° re-fit put the rear tie rods
through the inner bar O1–O2 and overloaded its pins. Each choice below answers one of those
or a rule of the build:
- **The ties never cross the inner bar.** The rear ties ride 292–320 mm from the mouth and
  never come further round than 72° down. The bar lies at 90.5°, from 93 mm out (the front
  ties stop at 85 mm).
- **The cradle link never sweeps a pivot.** Seen from the cradle, the pivots stay
  between 40° and 120° below the bottle's axis. b6's members cross the pivots' distances
  from the mouth only above that sector (X is 8° over the axis, the rear ties 21° under
  it) or in front of the mouth (Y). They pass O1 29 mm off at the closest.
- **Each pair shares a layer.** Y sits on the copy arm's line, so the bars b4 and b5 never
  meet. The lever on b2 stays 26 mm off b3. The planner finds the old sandwich: the rod,
  then b2 with b3, then b4 with b5, then the cradle link under the inner plate, every pin
  across one gap. (A crank nearer the mouth let the lever cross b3. The plan then put b3
  between the rod and b2, and pin P's longer span fell to jam SF 1.2.)
- **The handle is behind everything.** It turns behind the crank, and the crank is the
  machine's rearmost moving part.
- **The decanter stands in the bay.** The bay is 260 mm, so the decanter stands between
  the inner bars and never reaches a side's links.
- **The gearbox and the jam pull against each other.** A sealed full 3 L peaks where the
  crank-rocker turns the cradle fastest, near level. So the gearbox wants the two strokes
  alike, which takes a long rod for its crank.
  - At a jam the rod's force is worst 30° past the folded toggle: about 36 N·m over the
    crank times sin(30° (rod − crank)/rod). So the pins want a long crank.
  - The crank's distance behind the mouth sets the stand's length: the rod is nearly
    level, so O sits about a rod's length behind the lever.
  - 93.5 mm on 336.2 mm is the shortest rod that holds both: the gearbox at SF 1.38, pin
    P at 1.36. (The 160 mm saddles' lighter box gave the gearbox the room.)
  - The rod is 0.250 in: at 0.187 in a 300 mm rod buckles at 1.34 × its jam force, and
    its thinner layer doesn't shorten pin P's span enough to pay for it.
- **The cradle link b6 is 0.187 in** (0.125 until 2026-10-14, when it was first rated in
  bending: its rear arm held the jam at 0.32).

**What it holds** (`cradle_report.report`, a sealed full 6 L walking, its centre of mass
390 mm from the mouth, the hand crank's clutch's cap jammed; after the design review's
structure, mounting and drive rounds of 2026-10-14 and round 2's drive and structure, the
side as one frame):

| check | B |
|---|---|
| swing | −50.0° to +30.0°, the mouth still (0 mm), every bottle's mouth on the pivot |
| transmission | 36.7° at worst (the rod's); the parallelograms 48.5° and 51.5° |
| gearbox | NRV050: 35.8 N·m peak, SF 2.37 on its 85 N·m maximum, the clutch's cap 1.33 x under it; 21.7 N at the 125 mm grip at the worst start run in, 24.7 new (the bar 22 N, MIL-STD-1472H) |
| jam (63.7 N·m, the clutch's cap since simplification round 2: its racking brace, 0.46 kg on the box, then b6 cut to its band; 61.8 after round 1) | b3 1.33 (6061 0.187 in since round 2), the box's cradle link 1.41 (out of its plane with its in-plane stress, the brace's couple through its rear arm), b6 1.54, the outer and inner plates 1.59, b4 1.66, pin K 1.72, b2 and pillar O1 1.79, pillar O2 1.93, b1 and the side's buckling 1.99, column E2 2.06, pin P 2.32, the crank 2.33, pin Y 2.47, pin J2 2.58, pin X 2.85, column E1 3.26, pin J 4.07, column D2 4.30, b5 4.48; the brace's teeth 1.88, its tube 4.47, the tie rods 9.13, their A2-70 screws 18.9: every part 1.3 or more, the cradle box rated as a 3D frame since simplification round 1 (all of the couple across it), braced since round 2 ("The cradle box across the bay") |
| walking | every joint, link, column and plate 4.15 or more (the crank's keyway; the outer plate 4.78); the output shaft's radial load 8.2 (410 N at the rod's plane, 69.4 mm past the hub, against 3363 N) |
| rod, crankpin | buckling 1.78 × its jam force (1.9 as a beam-column); the crankpin 2.03 jammed |
| the clutch's cap over the whole turn | every part 1.31 or more 30° and more from the toggles (b3, 6061 since simplification round 2; 1.34, pillar O1 and b6, before the round's brace); 1.16 at ±25° (b3), 1.17 at ±25° before, 0.96 at ±20° (the outer plate, pillar O1, column E2; round 2's pillar O2 0.6 and 0.49); the gearbox's output shaft 1.3 outside ±25°, 1 outside ±20°; the rod buckles within ±16° of rest, ±9° of full tilt (rounds 1 and 2's drive reviews) |
| stand | tips at 26.1° at worst on simplification round 2's head (27.1° after round 1; a full 6 L at every tilt, over the rear edge: a bound; with the wine leaving as it pours, 30.2° before round 1); the drive side 29.9°, the passive side 31.9°, the front 39.3° (round 2's front foot 50 mm back); a push of 52.7 N or more at a bottle's top (toward the drive side). The board 40 mm toward the drive side and 686 mm long (round 2: its window under the base stop and the NRV050 took the 684 mm board's rear to 25.0°; the side frames' redesign, 1.2 kg more low on the stand, took the rear from 25.5° to 26.7°) |
| stack | 81.03 mm a side, 15 layers, proven (simplification round 1's rod pins; unchanged by round 2's brace); 84.55 mm in 16 layers before (b2 in 0.187 in 5052 since round 3, 0.1 mm thinner than the 7075; 84.65 with round 2's M6 tie screws' heads under the cradle link, 0.6 mm; 84.05 before, the planner of 2026-10-14; 84.23 in 17 before it, 79.68 before the clevis pin at K and b6 in 0.187 in) |

Before the review's round (2026-10-13, the old rules and a 331 mm centre of mass): pillar
O2 1.33, pin J2 1.40, pillar O1 1.42, b3 and b4 1.74, b2 2.61, b6 3.51; the gearbox 1.52
on a 29.5 N·m peak, 21.9 N at the grip, the stand 25.9°. Rated by the new rules, the old
b2 held 0.80, b6 0.32 and pillar O2 about 0.4.

(For a 3 L on the NRV030, before 2026-10-13: the gearbox 1.38 on a 14.46 N·m peak, pin P
1.36 at a 36 N·m jam, the stand 30.1°.)

**The stand** (`cradle.STANDS`):
- **The outer plate** is two triangles on the pivots' bar: O1–O, O2–O and O1–O2. A front
  leg runs from O2, a rear leg straight down from O, and a rail along the base board.
  O2–O, the strut that takes the rod's push back to the gearbox (1.45 kN jammed), is 64
  mm wide: at 24 it buckled out of the plate at 0.72, and the gearbox's node O, held
  across only by the plate's members, leans on it. The front leg is 40 mm wide from O2 to
  the foot (design review round 2's B1, layer 0 only: the plan is the same), cut off
  straight at the board's top (its round end had dipped 7.5 mm into the board). At 24 it
  held the clutch's cap out of its plane at 0.44 on the side as one frame; it held 0.60 at
  the merged drive's 63.7 until the side frames' redesign (2026-10-20, "The side frames at
  the clutch's cap"), 1.59 since, and the side's stiffness across at O1 went from 2.0 to
  2.67 N/mm, 6.35 since. The redesign also made the rail 44 mm wide (its line, its feet's
  screws and its columns 10 mm higher, its bottom edge where it was), the rear leg 56, O1–O
  40 and O2–Fr 32.
- **The inner plate** is the bar O1–O2, a triangle O1–O2–D1 off it and a leg D1–D2 down
  over the outer plate's front leg, on two 12 mm columns (since 2026-10-14: the bar alone,
  on its two pillars, held them only along itself; across it O2 was a cantilever off the
  outer plate, about SF 0.4 jammed). Each side is now a box from the pivots down, as A's
  is. The triangle and the leg are in front of the bar, where no tie rod or pin end passes;
  a column between O1 and D1 would stand in the lever b2's sweep. Since the side frames'
  redesign (2026-10-20) a 40 mm bottom chord runs on from D2 to two more columns on the
  rail, E1 and E2, 220 and 280 mm behind the mouth, under every link's sweep and the tie
  rods' paths: the box reaches toward the rear, so its posts no longer stand on one line.
  Nothing crosses the bay.
- **The base board** runs from 160 mm in front of the mouth to 524 mm behind it: 684 by
  584 mm (a full 6 L tips past 25° over its sides and its rear edge; it was 596 by 500 for
  a 3 L, 626 by 530 until the stand's check of 2026-10-14, 684 until the corrected centre
  of mass and the structure round the same day, 680 until round 1's drive, 684 until round
  2's window under the base stop and its NRV050). The first refit's had run 827
  mm, to 640 mm behind the mouth, with the crank 480 mm back. Its middle is 40 mm toward
  the drive side (`cradle.BASE_Z`, round 1). Its front keeps its 160 mm: the reviewer's
  35 mm trim (the front tips at 39.5°) would put the passive front foot's pad on that
  side's carriage bolt's head, now the board's passive edge is 40 mm nearer the bolts.
  - The rear edge binds, at 25.5° (bar 25°): with the centre of mass corrected (390 mm
    from the mouth), round 1's structure, the mounting round's saddles, round 2's window
    under the base stop and its NRV050, 686 mm is the least board that holds it (684:
    25.0° to the tenth on the drive round's cradle, 686 mm 25.2°; with round 2's 12 mm tie
    rods it is 25.5°, with the side frames' redesign of 2026-10-20 26.7°: the board keeps
    its 686 mm, the pack turn's call). Each 10 mm of board at the rear is about 0.5°; `python -m
    spiderpig.cradle_report stand` re-runs the stand alone.
  - **That is a bound, not the use case** (round 2's m1): `stability` counts each bottle
    full at every tilt, and the rear's worst is a full 6 L at 27.6° neck down, which would
    have poured most of its wine by then. With the wine leaving as it pours (full while neck
    up, poured past level: what it keeps then sits forward, in the shoulder, and only
    steadies the rear; `stability`'s `pouring`) the rear holds 30.2°, the drive side 33.3°,
    the passive side 38.1° (28.7°, 31.8° and 37.3° before the side frames' redesign): about
    5° of real margin over the bar. (A: 30.0° at the rear, 28.6° before.)
- **Planned on its own, the passive side would be about 35 mm thick** (the review's
  estimate, not built): its layers 1–8 hold only the pillars' sleeves under the drive's
  crank block. How: plan the passive side as its own design (the side's linkage with no
  crank, no rod and no pin P: a module of the cradle the planner takes as a mechanism with
  no drive) so its stack is the outer plate, b2 and b3, b4 and b5, b6 and the inner plate;
  make its plates its own files (its pillars and columns about 26 mm, its outer plate
  44 mm further in), and let `assemble_cradle` place the two sides at their own z either
  side of the bay instead of mirroring the drive side. The pillars' lever over the outer
  plate falls from about 51 mm to 7 and the panel packs 44 mm thinner. It is left for
  later: the plates stop being one file flipped (the ×2 price), and the pack, the guide
  and the BOM all assume the mirror.

**Tried and dropped** (a search over the parameters, each candidate then planned and
reported; [DECISIONS.md](DECISIONS.md) has the entry):
- **Pivots in front of the mouth**, a forward ray: the crank lands in front, and the
  handle turns back through the parallelogram.
- **Pivots past every tie**, 350 mm out or more: the bar b4 grows past 400 mm, and the walk
  loads triple.
- **A short rod**, 300 mm or less in 0.187 in: it buckles under 1.5 × its jam force, or
  the rod's jam force tops pin P. Rod 376 mm on a 109 mm crank (the heavier box): jam 1.44,
  gearbox 1.17. Rod 405 mm on a 67 mm crank: gearbox 1.33, jam 1.1.
- **The crank in front of the mouth:** the pillar O2 and the lever b2 fall under jam
  SF 0.8.
- **A crank 280–320 mm behind the mouth:** the lever crosses b3, the stack's order
  changes, and pin P (10.3 mm span) and pillar O2 fall to jam SF 1.2–1.3.

## Simplification round 1 (2026-10-21)

The user's ask (2026-10-20): laser cut everything that can be, the Makera Carvera for what
can't, and simplify each part for price; and the materials rule: no steel in any structural
part, aluminium only where a part needs it. The round's brainstorms, reviews and checks are
in `/root/scratch/decant/simplify/r1/`; the three checked branches (the stand, the drive,
the cradle box) were merged and the user's three decisions of 2026-10-21 applied
([DECISIONS.md](DECISIONS.md), "simplification round 1").

**What the cradles are made of now:**

| part | f5e2737 | now |
|---|---|---|
| posts (pillars, columns), tie rods | 12 mm 1018 steel standoffs (MISUMI NETRF12) | 1/2 in 6061-T6511 rod: the pivot posts (B's O1, O2; A's G) sawn from McMaster's hard-anodized 6750K16, the columns and tie rods cut to length by Speedy Metals; each faced square and tapped M6 at home. B's column D1 and A's C2 gone |
| link pins | hardened CCGH8 and 18-8 clevis pins | 5/16 in hard-anodized 6061 (McMaster 6750K14), 3/8 in at B's P and A's E (6750K15), cut to their calipered stacks, M3 button heads and wide washers on their tapped ends (`construction.pivots.rod_pin`) |
| crankpin | NETRF12 steel | 1/2 in hard-anodized 6061, sawn from the pivot posts' stick, tapped M6 both ends (capacity 47.4 N·m) |
| rear saddle | MISUMI PETR15 steel post, ServoCity's lead screw and nut | a 5/8 in 6061 post, a printed 13 mm lead screw, nut and thrust ring |
| the gearbox's riser | class 8.8 studs, 304 sleeves | A2-70 studs (Fastenal's M8 x 60 hex screws, heads sawn off), 6061 sleeves; 18-8 shims |
| b4, b5 | 7075 at 24 mm, b5 24 mm | b4 0.187 in 5052 at a 28 mm chord, b5 26 mm |
| cleats, riser locator | SendCutSend's routed 9 mm birch | Woodcraft's 3/8 in Baltic birch panels, cut at home on the Carvera (B's 452 mm cleats in two); the board stays SendCutSend's |
| the hand crank's knobs | heat-set inserts | captured nuts |

**Every surface a link turns on is hard anodized and greased** (the user's decision): the
pivot posts, every pin and the crankpin. The bores (5052, 7075, the 0.250 in rod) turn on a
coat harder than themselves, greased with the BOM's food-grade grease at each set-up; no
source puts a number on the wear at this duty (9-10 MPa walking, 42-51 at the cap, about
±40° a pour, a few thousand pours: the drive check's §1.4), so the guide looks for scoring at
each set-up. McMaster's hard-anodized rod is extruded, +-0.008 in (5/16 and 3/8 in) or
+-0.009 in (1/2 in) with a 0.001 in coat: the guide calipers each stick on arrival and opens
the laser-cut holes of the links it goes through when it reads over 8.05 mm (5/16 in),
9.64 mm (3/8 in) or 12.85 mm (1/2 in: the posts' and the crankpin's 13.05 mm bores to 13.10).
A pin's play is 0.26 mm nominal, up to 0.59 (the cold-finished rod's 0.43; the 8 mm g6
pins' 0.34): the mouth sags a few tenths of a millimetre more at worst, against the stream's
21 mm to the decanter's rim; its kinematic wander stays 0.0.

**The numbers** (the report on the merged head; f5e2737's in brackets):

| | B (`decanter_rcm`) | A (`decanter_hinge`) |
|---|---|---|
| a full 6 L's worst start at the input (`handcrank.DESIGN_START_NM`) | 2.629 N·m (2.719) | 2.763 (2.858) |
| the clutch's cap (`config.PRODUCT_CLUTCH_NM`); spun at 200 rpm | 61.8 N·m (63.9); 65.8 | 64.9 (67.2); 69.1 |
| the running peak at the crank | 34.7 N·m (35.9) | 36.5 (37.7) |
| every cap row but the box's | 1.53 or more (b6) | 1.40 or more (b2) |
| `jam_capped`, 30° and more from the toggles | 1.44 (the gearbox riser; 1.33 pillar O1) | 1.37 (the riser; 1.38) |
| the box at the cap (`OPEN_AT_CAP`) | rods 0.58, their screws 1.08, the cradle link 0.18 | 0.55, 1.02, 0.16 |
| walking: every row; the post joints' `open_x` (bar 1.0) | 4.28 or more; 0.86 (0.99) | 3.72 or more; 0.83 (0.70 on 10.9) |
| the box walking: `slip_x`, gap ratio | 0.73, 3.19 | 0.68, 2.82 |
| the grip, run in / new (bar 22 N) | 21.0 / 23.9 N | 21.3 / 24.2 N (`OPEN_GRIP` closed) |
| tipping (bar 25°); the push at the top (bar 25 N) | 27.1°, 52.5 N | 26.0°, 43.2 N |
| sway at the bottle, 20 N (bar 5 mm); frame buckling | 3.91 mm; 2.13 | 4.42 mm; 2.32 |
| the device | 16.23 kg (17.60) | 15.58 kg (16.87) |
| the lifts (`cradle.LIFTS`) | 12.7 kg the cradle, 9.9 the drive side with the box | 12.0 kg |
| B's stack | 81.03 mm in 15 layers, proven (84.55 in 16) | 65.95 mm in 12 |

**The BOM** (`spiderpig build`): B USD 1,007.55 (f5e2737: 1,380.45, -372.90): purchases 279.70, SendCutSend's cutting 727.85 (the fit); A USD 847.47 (1,157.43, -309.96): 254.50 and 592.97, one line unpriced (A's tube joint's M3 x 12 button head, McMaster's 100-pack). Out: MISUMI's posts, tie rods, crankpin, rear post and hardened pins, the 18-8 clevis pins, ServoCity's lead screw and nut, the riser's class 8.8 studs and 304 sleeves, B's column D1 and A's C2, and SendCutSend's routed cleats and locator (USD 97.28 on B: three Woodcraft panels, 11.91, in their place). In: the hard-anodized rods (B USD 57.17: two feet of 1/2 in, a foot each of 5/16 and 3/8 in; A 28.84), Speedy's cut rods (B 14.47, A 17.93), Fastenal's classed screws and studs.

**The carts** (folded: ORDER.md's `bom.CRADLE_CARTS_NOTE`): seven and the upload, from twelve on the merged branches: SendCutSend (the upload: the aluminium and the board), McMaster-Carr (the hard-anodized rods, the clutch's plungers, the R-clips, the grease, the riser's shims), Bolt Depot (the stainless screws, nuts and washers), Fastenal (the classed A2-70 screws and studs, the tie rods' class 10.9 screws), Speedy Metals (the plain rods cut to length: one cradle's under its USD 20 minimum, B 14.47, A 17.93, both 32.40; a process fee of about USD 8 on a line of more than ten pieces), Phidgets (the gearbox), Woodcraft (the panels and the glue) and Battery Junction (the light). Shipping is priced for SendCutSend (free over USD 39) and Woodcraft (USD 13.99 standard to 94103); McMaster, Bolt Depot, Fastenal, Phidgets, Battery Junction and Speedy are unpriced. One cart fewer: McMaster's multipurpose 6061 rod (8974K28, +-0.006 in, 6 ft USD 13.75) for Speedy's pieces, every piece sawn at home.

**Home work** (estimates, the checks'): B about 8 h at the bench and on the Carvera (the posts, tie rods and sleeves faced, drilled and tapped, 4.5 h; the pins and the crankpin cut, faced and tapped, 2-2.5 h; the birch, 1.5 h), A about 7.5 h (5.5, 0.5, 1.5); the tools to buy about USD 45 (the taps, wrench and drills 26.19-28.43 shipped, Makera's two bits 7.98 + 10.00 shipping, Tap Magic 9.95). Printing: the mounting parts slice to 48:53 h (the box check: PETG 40:19, PLA 0:50, TPU 7:44); the rest isn't sliced (B prints 2.03 kg of PETG, 0.50 of PLA and 1.77 of TPU at 100 %).

**Open items:**
- **The cradle box at the clutch's cap** (`OPEN_AT_CAP`'s three rows, both cradles): the
  joints slip and the links bend past yield; the user chose a racking brace. Round 2 closed
  B's ("Simplification round 2"); A's stay open (its hand crank). Walking holds.
- **Wear on the hard coat:** cited as the fix for aluminium galling, unquantified at this
  duty; the guide's scoring check at each set-up is the screen.
- **The anodized rod's tolerance:** McMaster doesn't say whether its +-0.008 in is the coated
  rod's; the guide's caliper rule covers either (`rod_pin.PIN_OVER`, `cradle.POST_OVER`).
- **The steel left:** the tie rods' eight class 10.9 M6 screws (Fastenal 1139912, the box
  review's preload), the riser's four A2-70 studs and its 18-8 shims (no 8 mm aluminium ring
  shim at McMaster), the clutch's three plungers, the gearbox's own shaft, worm and key and
  its output shaft, and the commodity stainless screws, nuts and washers.
- **First-article bench checks:** the rear post's drawer at a 750 ml setting, dry (PETG on
  6061: one cited figure); the printed lead screw's 24 h hold with 12 kg on the V block.
- **B's rod b1 on arrival** (at least 5.79 mm: its buckling falls to 1.22 on SendCutSend's
  thinnest 0.250 in).
- **Fastenal's 1139912** names no standard on its page (its head is ISO 7380's), and
  Fastenal refused scripts on the day; check the head on the page as it goes in the cart.
- **A's tip margin** stays the tightest: 26.0° against the 25° bar.
- **The packs** stay open (`PACK_OPEN`, `pack-inspect --plan default`): B 742 x 350 x 600
  mm, 155.9 L (157.5 at f5e2737), its drive panel unsupported on the cradle box, 16 spacers
  (408 g); A 700 x 252 x 607 mm, 106.9 L, a piece out of order (its panels over `PACK_BOX`'s
  215 mm). (B 175.9 L since round 2's racking brace and its way back: "Simplification round 2".)


## Simplification round 2 (2026-10-21)

**The user's two decisions** (2026-10-21):
- B stays the main design ("keep B"): no linkage change.
- A racking brace for the cradle box's jam (round 1's open item).

The round's review quoted 26 files live at SendCutSend. Its checker built the survivors and
kept what passed (`/root/scratch/decant/simplify/r2/check.md`). The brace was built on its
own branch. The integration merged the two, then did the brace's items (the review's P8,
A2's go/no-go) and the pack's way back (DECISIONS.md, "simplification round 2").

**What changed:**
- **The cutting fit** refitted on those quotes (`bom.CRADLE_CUT_FITS`: the deburring a part
  and the blank's cap; "Things to know before cutting", below).
- **The cut files** turned to their least-area box (`Body.turn`, a cradle's only): B's inner
  plate 497 x 239 mm, its outer plate 516 x 464.
- **B's b3** in 6061-T6 0.187 in at a 25 mm chord: 7075 leaves the order.
- **The riser's sleeves** faced to the gearbox's calipered height, 21-23 mm (`RISER_AIM`
  over the go/no-go window's low end, long rather than short): no shims.
- **B's stand:** the front foot and the board's front 50 mm back (the board 636 x 584 mm),
  the rail columns E1 and E2 60 mm forward.
- **The crank block's seven bored plates** one file, filed together on the key.
- **The posts' DIN 125 washers** only at the pillars' feet (`cradle.HEAD_WASHED`).
- **The M3 set screws** one length (5 mm), **the M3 socket heads** one (10 mm).
- **A's M3 x 12** from Bolt Depot.
- **The rods** sawn from McMaster's sticks (`cradle_catalog.ROD_STICK`): Speedy's cart goes.
- **B's racking brace** ("The cradle box across the bay"): a 2 in x 0.065 in 6061-T6 tube
  across the bay, keyed into toothed sockets in both cradle links.
  - It comes from McMaster's cart (9056K83, a 1 ft stick, sawn at home). Speedy's 4604 would
    have been that cart's only line, under its USD 20 minimum.
  - b6 is reshaped for its couple, then cut to a band across the bottle's axis
    (`config.PRODUCT_BAND`, the way back).
- **B's tie rods' end screws** A2-70 at 4 N·m (the review's P8, `cradle.TIE_SCREWS`): the last
  structural steel on B. A keeps class 10.9 at 6 N·m.

**The brace's concepts** (rough numbers at B's cap, 221 N·m across; A's 268):

| concept | how it carries the twist | rough SF | verdict |
|---|---|---|---|
| a shear panel (0.080 in 5052) between the rear tie rods, bolted at their ends | shear in its plane; its end couple, about the whole couple, out of each link's plane | links about 0.2 | out (and the rear carrier rides those rods) |
| diagonal straps (an X in the rear rods' plane) | the same statics: each strap 5.5 kN, its pull along the bay unbalanced at one link, 0.44 kN·m out of its plane | about 0.1 | out |
| a gusseted end plate behind the rear saddle | shear in its plane, 0.57 kN at 390 mm; its end couple 78 N·m out of each link's plane | about 0.45 | out |
| closing the box with a floor | planar again; and the decanter stands under the box between the saddles at rest | | out |
| **a closed tube** behind the rear saddle, keyed into both links | torsion; its couple in each link's own plane | the tube 4.45, its teeth 1.87, the box 1.40 | **built (B)** |
| a 1.5 in tube | torsion, 131 N·m/° | teeth about 1.3 | out (margin) |
| a laser-cut 4-plate box | torsion; its corners carry 9-15 kN along the bay each | | out (40 screws, heavier) |

**The way back** (B's pack): the brace's boss at X (r 50) and its wider rear chords (X-Tr1 and
Tr1-Tr2 64 mm, Tr2-Br 76) had stood the cradle box 249 mm tall in the pack (178 before).
- The band (`config.PRODUCT_BAND`) cuts b6 from 0.5 mm under Br's pad (the socket's web
  whole) to 1 mm under the front saddle's lip gauge's top, its corners R5.
- What the link needs grew inward and stays. Every section is rated on what the band leaves
  (`cradle_structure.Band`, `strip_section`: an eccentric net section).
- Narrowing the rear chords to 50 mm instead had failed: b6 1.24, the box's link 1.15.
- The box now stands about 212.5 mm tall.

**A2** (B's passive side planned with no inner plate, about USD -31 live): no-go.
- Its sway at the bottle is 4.40 mm or more, against the go/no-go's bar of 4.5. That is a
  lower bound: the model still ties the pillars' tips across the missing plate.
- The passive pivot O1 drops from 7.94 to 3.37 N/mm.
- That isn't a clear pass, so it wasn't built.

**The numbers** (`cradle_report.report` on the round's head; 0d9449b's in brackets):

| | B (`decanter_rcm`) | A (`decanter_hinge`) |
|---|---|---|
| every cap row, 30° from the toggles | 1.31 or more (b3; b6 1.54, the box's cradle link 1.41, the brace 4.47, its teeth 1.88, the riser 1.43): none open (the box's three, 0d9449b) | 1.37 or more but the box's three, open (`OPEN_AT_CAP`: the link 0.17, the rods 0.55, their screws 1.02) |
| b3 at the cap / walking | 1.33 / 12.13 (2.55 / 23.2 in 7075) | |
| the box at the cap, across | 222.9 N·m: racks 1.23°; the tube 195.2 N·m (88 %), SF 4.47; the rods 9.13, their A2-70 screws 18.9; the end joints slip in twist (`slip_x` 1.11), the faces shut (1.65) | open |
| the box walking, across | 25.6 N·m: 0.52°; the rods 20.7, the link 7.50, `slip_x` 0.47, the faces 3.93 x their gap moment | the link 4.09, `slip_x` 0.68, the faces 2.82 x |
| every walking row | 4.15 or more (the crank) | 4.07 or more (the crank) |
| the frame plates at the cap, outer / inner | 1.59 / 1.59 (1.66 / 1.72) | 1.44 / 1.60 |
| the side's buckling; column E2 | 1.99; 2.06 (2.13; 2.27) | 2.32 |
| the post joints walking: `open_x`, `slip_x` (bar 1.0); the bare heads' `head_sf` | 0.93, 0.65 (0.86, 0.51); 2.38 | 0.83, 0.63 (0.83, 0.50); 2.38 |
| tipping front / rear / drive / passive (bar 25°) | 39.3 / 26.1 / 29.9 / 31.9° (42.9 / 27.1 / 30.3 / 32.4) | 39.9 / 28.1 / 26.0 / 34.4° |
| sway at the bottle, 20 N (bar 5 mm) | 3.89 mm (3.91) | 4.42 mm |
| the start; the clutch's cap | 2.712 N·m (2.629); 63.7 N·m (61.8), spun 67.9 | 2.763; 64.9 |
| the grip run in / new; the gearbox | 21.7 / 24.7 N (21.0 / 23.9); 2.37 x walking, 1.33 x the cap | 21.3 / 24.2 N; 2.33 x, 1.31 x |
| clashes, 6 crank angles and both tilt ends | 0 | 0 |
| the device; the lifts (`cradle.LIFTS`) | 16.26 kg (16.23); 12.8 kg the cradle, 10.2 the drive side with the box (12.7, 9.9) | 15.55 kg (15.58); 11.9 kg (12.0) |
| the room under the mouth | the light's clip lowest, 394.4 mm over the table near rest; the drawn decanter's rim 46.2 mm under it | |
| the stack | 80.95 mm in 15 layers, proven (81.03) | as before |

**The BOM** (`spiderpig build`, the refitted fit): B USD 1,002.06 at 0d9449b -> 969.62 (purchases 279.70 ->
287.72, cutting 722.36 -> 681.90): the checker's survivors 940.34, the brace +36.25 (the tube
20.81, b6's cutting +15.44), P8 -6.97. A 812.04 -> 797.67 (254.50 -> 252.25, 557.54 ->
545.42). On the old fit 0d9449b was B 1,007.55 and A 847.47. Six carts and the upload
(Speedy's gone); 32 fastener SKUs on B (36), 33 on A; B buys 193 pieces (244), A 168 (235).
B's b6 blank 581 x 185 mm, turned.

**The pack** (`pack-inspect --plan default`; still open, `PACK_OPEN`): B 745 x 394 x 600 mm, 175.9 L (the
brace's uncut b6: 745 x 467 x 600, 208.8 L; round 1: 155.9 L), every piece supported, 14
spacers (620 g), one free to slide 153 mm; the cradle box stands 212.5 mm tall as in use
(`cradle_pack._items` stands it so: on its least footprint the squarer box had turned on end).
A unchanged: 700 x 252 x 607 mm, 106.9 L, 22 spacers (709 g), a piece out of order.

**Home work:**
- The tube is sawn from its stick and brought to length by hand (120-grit on the flat table,
  to 0.1 mm).
- Its 24 notches are marked from a socket and along the tube, then sawn and filed to fit (an
  estimate: 1.5-2 h).
- The guide's shop step: `cradle.shop_ops`, "brace".
- Hand tools only: whether the Carvera's vise takes a 2 in tube isn't on Makera's pages found
  that day.

**Not taken:**
- A's front trim: no plan at 50 mm, and its front tie rod 1.9 mm off the inner plate at 40.
- The light clip's M5 x 16 and the rear post's M6 x 16.
- A2.

**Open:**
- **A's cradle box at the clutch's cap** (`OPEN_AT_CAP`: rods 0.55, screws 1.02, link 0.17):
  the brace can't go where A's hand crank sweeps. The user's call:
  - a longer reach on A's hand crank: about 300 mm clears the tube and its pad. Its input
    shaft's overhung load, its hub's print and its pack are unchecked;
  - with that, A's link reshaped for the couple (a chord E-Br 48-56 mm wide) and its crank's
    throw to 132 mm to hold the 22 N grip;
  - or A's box as it is (walking holds).
- **The teeth share:** rated with all twelve bearing (the 5052 beds in a few hundredths at
  the first jam). The guide files each notch to its tooth.
- **A block at a saddle:** the box rows rate the whole couple across the bay (round 1's
  bounding case). A block's force on the rods at mid-bay, as a beam load, is the known-limits
  study's (`cradle_report.blocked`); the box no longer governs on B there.
- **The packs** (both open) and the pack spacers in PETG: the packing turn's.


## Building one

```bash
spiderpig build --linkage decanter_rcm --out build/cradle_b     # STEP, STL, DXF, BOM, ORDER.md
spiderpig guide --linkage decanter_rcm --out build/cradle_b     # ASSEMBLY.pdf and .md
spiderpig bake --linkage decanter_rcm                            # the viewer's .glb (8 s loop)
```

The assembly guide's order is `construction.assembly.CRADLE_MODULE_ORDER` on B and
`CRADLE_ORDER` on A:
0. **The shop work** (`assembly.SHOP_STAGE`, `cradle.shop_ops`; simplification round 1,
   2026-10-21: the posts, tie rods, pins and crankpin are 6061 rod made at home, the cleats
   and the riser locator cut on the Carvera), five steps before the first part goes on (six
   on B since simplification round 2):
   - the arrival checks: the cut parts labelled, deburred and calipered (`CRADLE_ARRIVAL`);
     B's rod b1 at least 5.79 mm thick (`cradle.ARRIVAL_LEAST_T`: its buckling at the cap
     goes as its thickness cubed); each rod stick calipered and its reading written on it,
     a pin stick over 8.05 mm (5/16 in) or 9.64 mm (3/8 in) opening its links' holes with a
     21/64 or 25/64 in drill (`rod_pin.PIN_OVER`), B's keepers printed to the 5/16 in stick
     as read (their STL scaled by (reading - 0.56) / 7.38);
   - the rods to length: the pivot posts sawn from their hard-anodized stick, every piece
     faced square on the Carvera (to 0.05 mm in length, 0.1° square: `cradle.FACE_SQUARE_DEG`),
     each pivot post and the crankpin calipered at its journal (over 12.85 mm, its link's
     13.05 mm bore opened to 13.10: `cradle.POST_OVER`), the riser's studs sawn from A2-70
     screws, its sleeves drilled;
   - on B, the racking brace's tube: squared to its length by hand (to 0.1 mm), one end
     marked from a socket's teeth, the lines carried down it to the other end (both ends'
     notches in line, as the links' teeth are), its 24 notches sawn and filed to fit;
   - the pins cut to their calipered stacks + 0.3 mm (`order.PIN_TAKE_UP_MM`, ORDER.md's
     pins note: `order.ROD_PIN_NOTE`), the crankpin to its drawn length;
   - every tapped end drilled and tapped (M6 at the posts', tie rods' and crankpin's ends and
     the rear post's foot; M3 at the pins'; the clip pins' 2 mm cross holes);
   - the cleats and the riser locator on the Carvera (`cradle.CARVERA_TEXT`): each 12 in
     panel sawn to 238 mm wide, taped down over its whole face, the outlines with a 1/8 in
     single-flute end mill, tabs, a 2 mm chamfered lead-in along each cleat's long top edges
     with a 90° chamfer bit, B's 452 mm cleats in two pieces each (`cradle.CARVERA_LONGEST`,
     `CLEAT_JOINT`: butted 1 mm apart, each glued along its length).
1. **The drive side**, bottom up on its outer plate:
   - the gearbox under the plate first;
   - the crank block from its lowest plate, the two standoffs screwed to it on the bench;
   - on B, X's and Y's TPU keepers onto their pins over b4 and b5 (each pushed with 30 N on
     a kitchen scale);
   - its cradle link goes on with the four tie rods already screwed to it;
   - its inner plate.
2. **The base board, bare:** the router's tabs sanded flush first (SendCutSend's router
   leaves 4.8 mm tabs on the birch, which it can't detab; design review round 3's
   laser-cut parts minor 7; the Carvera's cleats and locator come with theirs sanded), its
   riser locator's layers glued, its feet, bolts in; the drive side onto it (it can't stand
   alone). On B its studs go on at the bench first, the side lying on the riser's blocks,
   its brackets onto the board, and it drops into their open slots, the knobs snugged
   (`cradle.studs_first`, `DROP_DRIVE`; round 3's M3). No cleats and no finish yet: they go
   on once both sides stand on it (step 6).
3. **The hand crank**, its clutch set, and the toggle zones' paint marks on the gearbox.
4. **The cradle box:** the saddles slide onto the tie rods, each V block in its carrier
   on its thumbscrew (the lip gauge on the front one), then the light and the base stop go
   on.
5. **The passive side.**
   - B (design review round 2's M1, as the trip's way back): cranked to the pack pose and
     marked with the paint pen, the hand crank off, the drive side and its box lifted off the
     board and laid on its drive side on the riser's two blocks; the passive cradle link
     onto the tie rods' tops; the passive side panel built whole on the bench, lying on its
     outer plate as the drive side was (its stack, keepers, inner plate and screws, its
     brackets); set to the drive side's pose, turned over and lowered level onto the cradle
     link, its weight held until both pins are in; its two clips in. (Until round 2 it went
     on standing: set down 7-15 mm out on the board and pushed sideways onto two pins hidden
     behind its own plates, its inner plate threaded into the bay and screwed on inside it,
     and "lift the side off" to glue its cleats, which can't be done: it is pinned to the
     box.)
   - A: its cradle link onto the tie rods' free ends; its stack, from outside: the pillar
     goes up through the cradle link; what goes over the link, its inner plate, its
     brackets, its knobs only started.
   - B's brackets (pack turn 2): all four bolted to the board with the board, their
     nylon-insert nuts run down to the washer; each outer plate's studs with its jam nuts on
     the bare plate, their knobs on; the brackets' nuts snugged to 1.5 N·m (the luggage
     scale on the 10 mm wrench) once the cradle stands in them at level, and never again.
   - The lifts onto and off the blocks: two people, or one in `cradle.lay_down`'s order
     (round 3's M5); pin X painted on each cradle link at the lay (round 3's m1).
6. **The cleats and the finish** (round 2's M6): both sides on the board, each rail's cleats
   pencilled round it with a sheet of paper for the play; the cradle lifted off and laid on
   its drive side on the riser's blocks (A stood on its brackets' feet beside the board: on
   its side it tipped at 13.4° since round 3's structure); the cleats glued to the bare
   board, their Carvera-cut lead-ins up and to the rail (sanded by hand until simplification
   round 1); then every face and edge of the board, the cleats and the locator finished with
   polyurethane; the cradle back on, the passive side's knobs snugged at level, the checks.
7. **Using it** (round 2's M4, each its own step with its own picture, in this order, the
   order `construction.assembly.USE_TAGS` gives: untagged they had sorted alphabetically,
   the trip apart before the setting): before you use it, setting it for a bottle, the
   decanter's riser, decanting, the use limits (a five-line card and the rest for
   reference: `cradle.use_steps`, which the cradle page shows too); on B then the trip apart
   (8 steps), into its box (6) and out of it (6), and back together (11), each a step with
   its picture (the box's steps text only)
   (`construction.assembly.Scene`, drawn by `spiderpig.guide.scenes`: the cradle at its pose,
   laid on its side, its pieces lifted off).

The guide's cover lists the tools the steps name (`cradle.TOOLS`, round 2's m10: about 24,
named only inside the steps until then).

Things to know before cutting and on the bench:
- **Get a live SendCutSend quote** (upload the DXFs). The BOM's cutting figure is a fit
  (`hardware.bom.CRADLE_CUT_FITS`), refitted in simplification round 2 on SendCutSend's
  live quotes of 2026-10-21 (`bom.CRADLE_LIVE_QUOTES`: 26 files of both cradles and their
  variants, one off and at their quantity, and round 1's four) with the 2026-10-14 set it
  had been fitted on (design review round 2's parts B1, 40 quotes).
  - Since round 2, aluminium: `k x area^0.186 x cut^0.253 x long^0.336 x short^0.396`, plus
    USD 1.47 a part SendCutSend deburrs (its default finish, from 0.995 x 3 in: 2.94 a pair
    on every file quoted with and without it), the blank capped at 2,503 cm² (past it the
    price stops following the blank: B's outer plate turned 70° quoted 0.00 less). Every
    2026-10-21 quote at its quantity within -6.7 to +5.7 %, one off -9.8 to +5.2 %; the
    2026-10-14 set -6.7 to +10.8 %. A 6061-T6 0.187 in fit (b3's sheet since round 2).
    The 2026-10-14 law had A's outer plate 19 % over its quote and B's b5 20 % under.
  - Until round 2, aluminium: `k x area^0.249 x cut length^0.229 x blank^0.326`,
    `k` per sheet. 7075 runs 1.87 x 5052 one-off, 2.16 x at two.
  - The birch is fitted on its blank's two sides.
  - The quantity discount is each sheet's as quoted: 5052 0.187 in x0.67 at two, 7075
    x0.78, 0.250 in x0.76, 6061 x0.72, the birch x0.75. It had been a flat 0.72.
  - Every quoted file is within 11 % (`tests/test_cradle.py` holds it). B comes to USD
    760.28 against 758.19 quoted, A to 580.03 against 573.82.
  - Today's files, after simplification round 1 (2026-10-21: the posts, tie rods, pins and
    crankpin 6061 rod, b4 in 5052, the cleats and locator on the Carvera, the carts folded):
    B's cutting USD 727.85 (the birch now the board alone), the whole build USD 1,007.55
    (purchases 279.70); A's cutting 592.97, the whole build 847.47 (purchases 254.50, one
    line unpriced). Checked on 2026-10-21's live quotes of B's frame plates, not refitted
    (`bom.CRADLE_LIVE_QUOTES`): the outer plate 175.72 the pair against the fit's 190.12
    (+8.2 %), the inner 104.54 against 97.00 (-7.2 %); one law per sheet can't take one down
    without taking the other 15 % under its own quote.
  - After round 3's structure (2026-10-20: its wider frames, b2 in 5052,
    b3 and b4 bossed, A's column E2, the posts one length on fender washers): B's cutting USD
    857.41 (5052 0.187 in 413.44, 7075 96.98), the whole build USD 1,380.45 (purchases
    523.04); A's cutting USD 699.25 (5052 0.187 in 356.24), the whole build 1,157.43
    (purchases 458.18). That is with the tie rods' and A's G feet's class 10.9 button heads
    (Accu, a line more: B +0.56 for 8, A +0.50 for 10, bought as 12 at the dozen's price);
    on the A2-70 ones B had been 1,379.89, A 1,156.93.
  - Before round 3's structure, after its drive and its use and parts fixes
    (2026-10-20): B's cutting USD 872.05 (5052 0.187 in 347.64, 7075 177.42, the birch
    237.84, 6061 86.42, 0.250 in 22.73), the whole build USD 1,412.11 (purchases 540.06). A's
    cutting USD 654.45 (5052 0.187 in 311.44, the birch 238.61, 6061 85.09, 0.250 in 19.31),
    the whole build 1,094.59 (purchases 440.14). Round 3's drive added the riser's clamped
    stack (+27.31 each: studs, sleeves, shims, washers, the ring's 6061); its use and parts
    fixes the food-grade grease (USD 11.99 each) and closed the crank plates' necks (6061
    -1.55 on B, -1.45 on A; B's inner plate's tapers +0.14); B's foot nuts went to
    nylon-insert ones at the same USD 0.11. (With the side frames' redesign, before round 3:
    B 860.49 and 1,374.22, purchases 513.73 (DECANTER had said 1,373.94 and 513.45, before
    pack turn 2's jam nuts: round 3's laser-cut minor 10), A 642.93 and 1,056.74; round 2's
    drive merged, before the redesign: B 797.23 and 1,307.24, A 605.39 and 1,005.50; before
    that merge B 768.44 and 1,190.54, A 580.35 and 920.93.)
  - Until round 2 the BOM printed B's cutting at USD 301.58. That left out a "USD 68-136"
    7075 band and an unpriced birch line: about USD 457 short.
  - Each file's estimate is in ORDER.md's upload table, and which files arrive as cut (under
    SendCutSend's deburring minimum, `bom.hand_deburred`: 0.995 x 3 in, its quoting app's
    limit, which its tables round to 1 x 3 in; `sheet_catalog.SCS_DEBURR_MM`).
- **The cut rules' inside corners** (design review round 3's laser-cut minor 1): a cradle's
  cut-outs are read corner by corner (`manufacture.inside_corners`: every tangent break
  turning round the cut-out) against SendCutSend's published laser radius, "approximately
  the kerf width" (`sheet_catalog.SCS_LASER_CORNER_R`, 0.3 mm; the 0.8 mm had no source).
  The keyways' square ends (the bore's arc had hidden them) and the clutch seat ring's hex
  are warnings, harmless (a key's chamfered edges, a printed hex). The walkers keep the
  arcs-only reading and 0.8 mm (their audits are the identity gate's).
- **On arrival** (the guide's first stage, `assembly.CRADLE_ARRIVAL`; design review round
  3's laser-cut minors 2, 3, 4 and 6):
  - A paint-pen label on every part (its file's): some files differ only by a hole or a
    slot (the crank block's keyed and clear plates: the gearbox's key drops through a
    clear one's slot). ORDER.md says so under its upload table; the guide's parts table
    names the crank block plates and calls the birch CNC-routed (`labels.part_types`).
  - Deburr by hand the parts that come as cut, which the step names from the fabrication
    (`assembly.as_cut_words`: B's b4 and b5, 24 mm wide).
  - The clutch's seat ring: its clean (entry) face marked, each of its six holes' edges
    broken alike there with a countersink, two turns by hand; it goes on that face out, to
    the balls (the seats are the holes' edges: a burr wears off and drifts the setting).
  - Each 8 mm pin slides through its holes by hand before it goes on: the 0.250 in rod's
    8.2 mm holes can come 0.05 mm under the g6 pin (±0.005 in and the thick sheet's
    taper), opened from the exit face with fine sandpaper rolled round a 7 mm drill bit's
    shank (or an 8 mm H8 reamer). The hole stays 8.2 mm: no part changes.
  - Caliper every link at its pin holes. SendCutSend's sheets run up to +0.69 mm thick at
    0.250 in and +0.23 mm at 0.187 in (its spec tables).
  - Then order the made-to-length pins (MISUMI CCGH8: the measured stack + 0.3 mm; ORDER.md
    says to order them last).
  - The report rates each pin's take-up against those tolerances (`cradle_report.take_up`,
    the parts' M5). B's pin P can be 0.49 mm short of its play and Y 0.06 mm at worst, A's
    E 0.52 mm. Where a ring or clip won't seat, sand that pin's printed spacer by the excess.
  - The base board goes on the flat table first (SendCutSend's sheet is flat to ±0.030 in a
    foot before cutting, 1.7 mm over the board), its concave face down.
- **The posts' screws are A2-70, A's tie rods' class 10.9** (design review round 2's m2 and
  M1; round 3's M5 and its finish, 2026-10-20; B's tie rods' A2-70 since simplification
  round 2's P8).
  - Each is an M6 x 16 button head of a stated class (ISO 3506-1: Fastenal's, 9.0 kN proof;
    ISO 898-1: Fastenal's 10.9, black oxide), in 6.4 mm holes (a DIN 125 washer only at the
    pillars' feet since simplification round 2's P6).
  - Tighten the stainless ones to 4 N·m on the 4 mm key (ISO 7380's most in A2-70: 2.5-4.6
    kN), A's tie rods' steel ones to 6 (3.8-7.0 kN): each end joint stays closed walking.
    Keep the two in their own bags.
  - Jammed, the joints gap. The tie rods' screws then pry at 3.4-4 kN; pillar O2's foot
    screw pries at about 8 kN at its 48 N·m (open with the pillar).
  - Bolt Depot's 18-8 names no class, and A2-50's 210 MPa would yield there.
- **Run the gearbox in and wipe it:** a new worm box can weep oil at its seals, and it
  stands over the base board beside the wine. A few hundred input turns (a drill, slowly),
  then wipe it: before it goes on the cradle, never after (a drill on the built cradle's
  input turns it past the clutch).
- **The gearbox's go/no-go** (the guide's riser step): on its studs and nuts under the
  outer plate, the shaft's end must stand 25.35-26.05 mm over the plate's inner face; else
  the four nuts under the plate are turned the same ("The drive" above). The output shaft
  matches Phidgets' own shaft drawing (25 h6, collar 32 x 3.5, 50 past it, a 40 mm keyway);
  the housing's spigot height, hub protrusion and hub diameter are unpublished, and the
  riser keeps them out of the stack-up.
- **File the keyed plates' slots together on the key**, the block tied on the bench, to a
  slide fit: the lowest three are cut 0.25 mm under it so each can be, and until they are
  fitted the tightest plate carries the crank alone; the three over them are cut clear of
  it (no file).
- **Set the clutch's plungers from a felt stop** (the guide's hand crank step, round 3's
  B1): the arm at a click, each plunger in until it stops hard, then out half a turn; never
  nearer the stop than an eighth of a turn.
- **Measure the start, set the hand crank's clutch** once the cradle is built (the guide's
  last build step, "Before the first pour": round 3's stand and use M2) at 1.3 x it, never
  over the ceiling, with a luggage scale and the printed check stub in the riser locator's
  pocket (no vise: its m12), greased, and check it at every set-up, after any greasing,
  after a trip in a hot car, and every month or ten pours standing set up; once a year
  re-grease its seats and journal and look for any bright land round a hole's edge.
- **Paint the toggle zones' red arcs** on the gearbox's far face (the same step: an arrow on
  the output hub, the housing's rim red 2.5 handle turns either side of each toggle, the
  toggles found by the crank and the rod: round 3's stand and use M4).
- **Tools** (`cradle.TOOLS`, the guide's cover and `ASSEMBLY.md`; one list since the stand
  merge; simplification round 1 added the rod's): a torque wrench from 4 to 15 N·m, hex keys,
  8, 10, two 13 mm and a 3/4 in wrench, a caliper with a depth rod, a machinist's square, a
  luggage scale, a fine-tooth hacksaw (the hard-anodized sticks, the riser's studs), the
  Carvera with its vise and an aluminium end mill (the rods' faces, the rear post's flats),
  a 1/8 in single-flute end mill and a 90° chamfer bit (the birch), M3 and M6 taps (plug and
  bottoming) in a T-handle wrench, Tap Magic, drills of 2.0, 2.5, 5.0 and 8.0 mm, 21/64,
  25/64 and 33/64 in (a stick or a post over its size), a drill press with a V-block, a
  14 mm drill bit and a drill (the run-in), a flat file, a vise (pressing nuts), a paint pen
  and red paint. (No soldering iron since the drive check's knobs on captured nuts, no
  E-ring pliers since its rod pins.) The tools to buy, priced by the round's checkers on
  2026-10-21: the taps, wrench and drills USD 26.19 (BoltsandNuts, shipped free) or 28.43
  (Drills and Cutters, shipped); Makera's 3.175 mm single flute USD 2.99 and its 90° chamfer
  4.99 (+10.00 shipping); Tap Magic aluminium 4 oz USD 9.95.
- **Glue the input shaft's key** into its keyseat (threadlocker or super glue): it falls out
  when the crank comes off otherwise.
- **Grease every hard-anodized journal thin** (simplification round 1, the user's choice of
  2026-10-21: the pivot posts, every link pin and the crankpin, where a link turns, are
  McMaster's hard-anodized 6061; the bare 6061 columns and tie rods carry no link and take
  none): a film of the food-grade grease the BOM buys (McMaster 1378K31, NSF H1 with PTFE,
  Super Lube 21030's specs; `cradle.GREASE`, design review round 3's stand and use, m10: the
  front saddle's box guide and its thumbscrew sit in the wine's path) on each journal as it
  goes in and at each set-up, the crankpin where the rod turns on it a full film; the
  decanting step wipes the front saddle, the tie rods and the light after each pour. (Until
  then the steel standoffs, bare 1018, were wiped against rust.)
- **The worm's hold:** with a full bottle at level, tap the table and watch a pencil line
  across the crank and the plate for 10 minutes (the guide's last build step).
  The clone's mesh isn't Motovario's: a 16 % drop in its friction loses self-locking. One
  that creeps settles back neck up; an O-ring's drag between the hub and the input seal
  doubles the margin.
- **Dry-fit the hinge:** the cradle link must swing under its own weight once the
  pillar's screws are tight.
- **Break the laser-cut edges** where hands go: the bay and the frame tops.
- **Glue, then finish the base board** (round 2's M6): the cleats and the riser locator's
  layers glued to bare birch, then polyurethane on every face and edge of the board, the
  cleats and the locator (the notch's end grain above all; the locator stands on the table,
  in any spill). Not oil: an oiled face is no barrier to red wine, and glue doesn't hold to
  it.
- **The use limits** (`cradle.use_steps`, the guide's "The use limits" and the cradle
  page's "Using it": round 2's M4 cut the 626-word paragraph into a card and a reference).
  The card:
  1. Room: 950 mm of headroom (a 6 L's base at full tilt reaches about 910 mm over the
     table) and 150 mm free behind the board and beside its drive side (`cradle.ROOM_FREE`;
     design review round 3's stand and use, m5: it had been 100 mm behind, the base stop's
     rails' reach). The hand crank's grip turns about the input shaft at the same reach at
     every angle: B's grip bolt's head stands 103 mm past the board's rear edge, its grip 76
     mm past the drive-side edge at its outermost (a hand on it 118 and 104 mm); A's grip
     stays inside its rear edge and passes the drive-side edge by 102 mm (a hand 130).
     Against a backsplash 100 mm off, B's bolt head had rubbed the wall all the way round.
  2. The decanter on its riser, set once at rest, its rim 20 mm or more under the light's
     clip; once set, nothing raised or added under the cradle box (round 2: "nothing under
     the cradle box at rest" had contradicted the decanter that stands there by design).
  3. Hands out, and nothing added under the box, while the gearbox hub's arrow is in a red
     arc (the 2.5 turns either side of rest and of full tilt); the decanter stays put
     (round 3's stand and use M4: "nothing under the cradle box or the bottle" had
     contradicted the decanter that stands there by design).
  4. The bottle uncorked before it goes in, and the pad over its rim (look along the neck:
     past level only the pad holds the bottle forward; round 3's stand and use, m6); the
     gate's knob tight (tug the gate) and the rear knob down on its teeth whenever a bottle
     is in (round 2's mounting: the gauge is fixed, and it had been "the lip gauge down
     before the bottle passes level").
  5. The handle off (its thumb screw) when you leave it: nothing turns without it.
- **Where it pinches** (the use limits' reference, round 1): both side stacks, their bay
  faces too (each return to rest the cradle links shear past the inner bars' front ends);
  on B the crank and the rod's end, turning in the open above and behind the gearbox. (At
  rest the base stop comes down to 26 mm over the base board behind the decanter, 43 mm
  over the table through the board's window under it.) A trapped finger: turn the handle
  back the way it came (the worm holds what it has caught). An optional printed skirt over
  each inner bar's O1 end on the bay face (round 2's M7.4, for homes with children) is not
  drawn: it would stand where the front tie rods pass, 85 mm from the mouth against the
  bar's end at 93, and needs a clash study of its own.
- **Never force the crank.** If it stiffens, something is in the way. The clutch clicks
  round past its bench setting (1.3 x the measured start), so a push can't break the drive
  away from the ends of the swing; within the first and last 2.5 handle turns (the
  gearbox's marks) the cradle's force grows without bound however the clutch is set:
  nothing under the cradle box or the bottle, or touching either, there.
- **Decanting** (the guide's step): the decanter on its riser stays where it is while the
  bottle comes back to rest (its rim keeps its 20 mm under the clip); at rest it is taken
  away and the neck, the front saddle, the tie rods and the light wiped. (Round 1's guide
  had said both "take the decanter away before cranking back" and "leave it until the
  bottle is back at rest".) A bigger bottle than its decanter: turn back a turn, slide the
  full one out, the next onto the same riser.
- **Letting go stops the bottle, not the stream:** to stop it, turn back about a turn.
- **Uncork standing on the table, before the bottle goes in** (since review round 2: the
  lip gauge is fixed over the mouth): a sparkling wine under pressure the same, its cork
  eased out by hand, the mouth pointed away from people. The -50° rest takes an opened
  full bottle.
- **The rear knob down on its teeth** whenever a bottle is in: lifted, only the brake
  washer slows the lead screw.
- **Lift it** by its base board in use; to take it off the board, by its outer plates' top
  members, never by the cradle box or the hand crank.
- **Out of the sun, under 50 °C** (since design review round 3, mounting m5): the parts
  that carry the bottle and the hand crank are PETG, the bottle's parts bearing up to 11 MPa
  as long as a bottle rests, and PETG's resin deflects at 64 °C under 1.8 MPa, 70 under 0.46
  (Eastman's Eastar 6763; "68 °C" had been an unloaded part's); PLA softens at about 55 °C.
  A cradle that travels in a hot car wants the brackets and the thumb knobs in
  PETG too.
