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
| the bay | 260 mm between the inner plates | a 174 mm bottle between the rear saddle's post and knob; a standard decanter (up to 240 mm across) stands wholly between the side stacks |
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
  - Four tie rods run between the two cradle links, at `decanters.TIES`. Each is a 10 mm
    steel standoff made to length (MISUMI NETRF10, M5 ends), with an M5 button head
    through the link into each end. Two carry the front saddle under the neck, two the
    rear saddle 275 and 360 mm behind the mouth.
  - Two printed saddles ride on the tie rods (`cradle.SADDLES`; since 2026-10-14, under
    "The saddles, adjustable"). Each is a carrier locked on its rods by M3 set screws up
    from its underside, and a V block that rides up and down in it on a thumbscrew.
    - The front one carries the neck: its V block slides in the carrier's box guide, an M6
      thumbscrew at its end.
    - The rear one carries the body: its V block rides a 12 mm steel post at one end and a
      coarse lead screw (8 mm a turn) at the other. The base stop's two rails continue its
      V behind it on 45° flanges to 588 mm, so a 6 L's whole body is carried: lying level
      or tipped, a bottle's weight centres on the saddles (`cradle_report.balance`, "The
      saddles, adjustable").
    - The Vs are 90°, lined with 2 mm TPU pegged into them. At a 6 L methuselah's setting
      each V block sits on its tie rods (the rear one in a seat over each); a thinner
      bottle's ride higher.
  - **The lip gauge** (`cradle._gauge`): a printed arm on a hinge along the axis on the
    front carrier (two knuckles either side of a post, an M5 tap bolt through them). Down,
    its pad stands in front of the mouth, its back face on the pivot's plane, over the top
    of the rim (above a cork's bore), and its bar runs back beside the neck, not over it: a
    bottle lays in with it down. It checks where the mouth is, and it is the forward stop:
    past level a bottle would slide toward its mouth, and the rim's top bears on it. It
    lifts 70° onto a stop to uncork; let go, it falls back by itself (its centre of mass
    stays inboard of the hinge), so it can't be left up.
  - **The base stop** (`cradle._rails`, `cradle._gate`): a printed gate across the V behind
    the bottle's base, riding the rear V block's flats and two printed rails behind the
    block. It is the back stop: at rest the bottle stands 50° neck up and leans on it (at
    25° nose up the liners' friction alone had held a bottle, and a corkscrew's push of
    6-13 N slid it back off the pivot: the mechanical review).
    - Keys under its ears run in a groove along each flat and rail. It is set by a scale
      grooved across the +z flat and rail and locked by hand: a printed knob on an M5 tap
      bolt through one ear into a nut and a fender washer in a T-slot under the groove.
    - It serves every base from 285 mm (inside the rear V: a 750 ml's at 300) to 576 mm
      (a 6 L's at 571) behind the mouth.
    - It rides the rear V block, so it rises with it. At rest, set for a 6 L, it comes
      down to 26 mm over the base board, a finger's 25 mm (ISO 13854): the flanges end at
      588 mm, past a 6 L's heel, and the +z rail's keel is chamfered at its end
      (`cradle.FLANGE_END`, `cradle.finger_gap`). Since round 2 the board has a window under it (`cradle.BOARD_WINDOW`): what lies there lies on the table, 43 mm under it.
- **The bottles** (`cradle.setting`, `cradle.seat`): every one lies with the mouth on the
  pivot and the axis level, so the mouth stays put for each. Its setting is the two V
  blocks' rise and the base stop's place, read off the scales with the bottle out (see
  "The saddles, adjustable"); the base stop and the lip gauge hold it there.
  - The 6 L necks' lengths and radii are estimates scaled from the jeroboam's (no maker
    draws them): set the front knob to the neck in hand.
  - Another bottle: set each knob to its neck's (measured 50-80 mm from the lip) and its
    body's diameter on the scales' other rows, and the base stop to its length.
- **The drive** (`spiderpig/construction/handcrank.py`, `config.crank == "hand"`; round 1's
  drive review of 2026-10-14 reworked its riser, its block, its tie and its hand crank).
  - **Gearbox:** an NRV040 worm gear reducer at 30:1 (Phidgets TRM4401_0, rated 45 N·m;
    `handcrank.NRV040`). It stands outboard on the drive side's outer plate, held by four M6
    button heads into its tapped face holes (75 mm PCD). A full 6 L peaks at 35-37 N·m at
    the crank: the NRV030 (20 N·m) that held a 3 L couldn't.
  - **Its rating at a hand's speed** (`WormBox.rated_hand_nm`, round 1's drive): 58.1 N·m.
    Motovario's NRV040 at 30:1 is rated Mn2 34 N·m at 2800 rpm input, 46 at 1750, 48 at
    1400, 51 at 1140, 53 at 900 and 62 at 500, the slowest its tables list (the VSF IEC
    catalogue, EN rev 0 2017, printed pp. 161, 171, 181, 191, 201, 211): the mesh's wear and
    heat set it, and both fall as the input slows. A hand turns it at about 60 rpm, so the
    500 rpm figure is the cited one nearest, on the safe side. Phidgets rates its clone 45
    with no speed, so the clone is taken at Motovario's 62 times 45/48. The walking peak
    is rated against both (`gearbox_sf_hand`, bar 1.3; `gearbox_sf` on the 45 N·m, kept as
    a row). The clutch's cap is held to the catalogue's momentary limit, 2 x Mn2 (p. 9:
    "must not exceed 200%"), taken on the 45: 90 N·m.
  - **The riser** (`handcrank.riser_for`, `handcrank.SPACER`): four 12 mm aluminium round
    spacers (McMaster 94669A349, 13 mm OD) on the face screws, bearing on the gearbox's
    machined output face outside its spigot ring. The raised spigot ring and the turning
    output hub's end stand 9.5 mm under the plate, so neither unmeasured height is in the
    stack-up. (Until round 1, three 0.125 in discs were bored to the hub's drawn 30 mm, on
    the spigot's assumed 2.5 mm: a hub standing proud would have been braked by the four
    screws' preload, and would have lifted the block into the rod.)
  - **Holding:** its static efficiency at 30:1 is 0.45 (Motovario's mesh data), under the
    0.5 that the makers' catalogues give for static irreversibility. So the cradle holds
    any angle when let go. Phidgets' unit is a Chinese NMRV clone, not Motovario's mesh, so
    the guide bench-tests it (a full bottle at level, the table tapped, 10 minutes); one
    that creeps settles back neck up, never further, and an O-ring's drag on the input
    doubles the margin.
  - **The crank block:** 0.125 in 6061 plates, each the arm's whole outline, stacked on the
    stock 19 mm keyed output shaft (Phidgets TRM4006_0, modelled with its collar and key).
    - Every plate round the shaft is keyed, the lowest four on the 6 mm key's straight
      flanks: the shaft's keyseat is end-milled, so the key's ends are round (the
      fabrication review). The slots are cut at the key's 6.0 mm and filed together on the
      key, the block tied, to a slide fit: one plate alone holds 33 N·m at bearing yield,
      under the clutch's cap; the four together 132.
    - It is built and fitted on the bench as one block, tied by an M6 x 30 button head
      through every plate into a thin nut under the lowest (`handcrank.TIE_NUT`; a NETRF10
      and two screws until round 1).
    - Its top plate seats on the shaft's steel end face over 0.2 mm DIN 988 shim rings
      (`handcrank.SHIM`, two drawn), an M6 button head through both into the shaft's end
      tap, threadlocker, 8 N·m. Only steel and aluminium are in that clamp: the printed
      sleeve the block had clamped on until round 1 crept out of its preload.
    - The go/no-go (the guide's): the gearbox on its spacers under the plate, the shaft's
      end measures 28.75 mm over the plate's inner face; the block's top plate sits at
      29.22, so the shims are the difference in whole rings, up to 1.6 mm. Higher, 13 mm
      spacers; lower than 27.6, 11 mm ones.
  - **Crankpin:** a 12 mm NETRF12 standing in the block from its lowest plate (an M6 up
    through it) through the rod, an M6 and a wide washer on its end over the rod (a 6 L's
    jam bent the 10 mm one: SF 1.13), 0.2-0.3 mm of play (its length rounds down; a printed
    spacer under the rod, sanded on the bench, takes the plates' tolerance), greased where
    the rod turns. The washer keeps the rod 0.26 mm (nominal) under the rear tie's screw
    head where that head crosses it, 1 to 4 handle turns from rest. The chain's tolerance
    is about 0.3 mm, so the guide checks it with a sheet of paper: a rub there is a scrape,
    not a jam. The rod rides 3 mm over the block's screw heads, its force 9.5 mm off the
    crankpin's socket (the lever the build measures, `crankpin_lever_mm`). (Before, the
    arm sat on the key and the rod 21 mm over it: an ISO 7379 crankpin's M8 thread was a
    cantilever that long, rated at SF 1.0-1.25 once its yield and preload were right.)
  - **Hand crank** (`handcrank.HandCrank._handle`): printed in two parts joined by a
    ball-detent slip clutch, on the gearbox's 11 mm input shaft.
    - The hub is printed solid, its bore reamed to 11 mm, 21 mm long on the shaft, keyed
      on its 4 x 4 key (glued into the shaft's keyseat), with an M3 x 9 knurled thumb screw
      (McMaster 92545A115) through a brass heat-set insert onto the key: it comes off by
      hand. It runs on as a 100 mm tube to the clutch's flange and a journal.
    - The arm turns on the journal, held by a printed cap and a second thumb screw. Three 6
      mm steel balls in its boss, each on a spring (McMaster 9434K49) and an M8 set screw,
      press into six 90° conical seats in the flange (`handcrank.CLUTCH`). It is set on the
      bench with a luggage scale to slip at 3.375 N·m on the input
      (`handcrank.CLUTCH_SLIP_NM`, ±10 % once set): 27 N at B's grip, 40 at A's. At its
      least that is 1.1 times a sealed full 6 L's worst starting turn on A (2.735 N·m at the
      input; B's 2.602). Through the worm at a hand's speed it is at most 61.3 N·m on either
      crank (`config.PRODUCT_CLUTCH_NM`), however hard anyone pushes, and that is the jam
      case's torque (`cradle_report.jam_nm`). A firm 40 N push had been the case: 82.5 N·m
      at B's grip, no bound.
    - Its 60 x 16 mm printed grip spins on the plain shank of an M8 x 85 hex bolt through
      the arm's end, a nyloc nut on the arm's inner face (a NETRF10 standoff and two button
      heads until round 1: a 25 mm knob was a two-finger pinch). 60 x 16 is within
      MIL-STD-1472H's light-load crank handle, 25-75 mm long and 10-16 across.
    - The arm is 125 mm on B (`handcrank.HANDLE_R`), behind every moving part of the stack.
      That is MIL-STD-1472H's longest light-load crank: under 22 N at the grip
      (`handcrank.LIGHT_LOAD_N`), a 38-125 mm radius below 100 rpm, wrist and finger
      movement (§5.1.4.1.2.4.7, Figure 13, p. 76). From 22 N up it is an arm's crank, 190 mm
      or more. A full 6 L's worst start is 20.8 N on B. A pour is about 15 turns (30:1 over
      half a crank turn), the many-turn task the standard puts cranks to (§5.1.4.1.2.4.1.1).
    - A's arm is 85 mm (`handcrank.HANDLE_RS`): its cradle link b2 and its rear tie Tr2's
      screw pass 100-135 mm behind the input shaft's axis at about 165-195° and 330-345° of
      crank, so the crank's disc (the throw, half the grip and 1 mm) clears only up to about
      97 mm. Its 105 mm arm had met them there; the slow test's five angles had missed it.
      At 85 mm a full 6 L's worst start is 32.2 N at A's grip, over the 22 N bar: open
      (DECISIONS.md, 2026-10-14, the drive).
    - Turn it from the drive side's rear corner, the neck in view.
  - **No guard over the crank** (the user's call, 2026-10-10: the guard, a disc in the
    outer plate and then the gearbox's flange, read as a big disc either way). The gearbox
    sits on its spacers; the crank turns about 2 rpm behind the outer plate's truss. On B
    its top-rear arc and the rod's end turn in the open, above and behind the gearbox,
    right in front of the hand crank (round 1): the use limits name them with the other
    pinch points (`cradle.use_steps`' use limits), and taking the handle off locks it. Near rest B's crank's arm points into a red toggle flag behind the gearbox over the last two turns ("The base board" above; round 2).
  - **The handle comes off by hand** (its thumb screw): nothing turns without it, so it is
    the child lock, and the use limits say to take it off when the cradle is left.
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
  - B's stack is 84.05 mm a side in 16 layers (the planner of 2026-10-14: 84.23 in 17
    before it; 79.68 until 2026-10-14: the clevis pin at K moves b2 and b3 a layer up, and
    b6 is 0.187 in).
- **The pillars** are 12 mm steel standoffs (MISUMI NETRF12, M6 ends;
  `StandoffAxle.cradle_shaft`), each made to its plates' gap: a 6 L's jam bent the 10 mm
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
  - The crank block: 0.125 in 6061 (the keyway's 6.1 mm slot is legal in it).
  - B's links are 0.187-0.190 in: its arms b2 and b3 and its bar b4 in 7075-T6 (a 6 L's
    jam: 6061 held 0.83; SendCutSend's datasheet, yield 69 ksi), b5 and its cradle link b6
    in 5052 (b6 was 0.125 in until 2026-10-14); its 336 mm rod is 0.250 in 5052, the rod
    and b6 each alone in its layer.
  - Some chords are wider than 24 mm (`config.PRODUCT_CHORD_R`): B's b6's rear arm X-Tr1
    (40 mm) and its chord X-Y (32), A's b2's E-Tr1 (32).
  - The metal is cut at SendCutSend: `materials.LINK_SHEETS` and `config.PRODUCT_*`. The
    7075 has no fitted cutting price yet (quote at upload).
  - The cradle's links are 24 mm wide (`config.PRODUCT_*`'s `link_radius` 12, from 22): B's
    336 mm rod in 0.250 in then holds 1.6 x its jam force in buckling (at 22 mm, 1.46), and
    the bigger pins' holes keep a sheet's thickness of web.
- **The stand** (`cradle.STANDS`).
  - **Outer plate:** a truss outside every moving part's layer: from the hinge G down in
    front of the decanter to the front foot and back to the crank O, O's legs down and
    back, a rail along the base. Its inside corners are rounded 20 mm (`Stand.fillet_r`:
    the outline grown and shrunk back, `cradle.plate_part`), so the joints read as one
    shape. Both sides' outer plates are the same file.
  - **Inner plate:** a block round the hinge, and a leg over the outer plate's front leg
    down to about 200 mm over the table, tied to the outer plate by five columns (the
    fifth, D3, since 2026-10-14: the outer leg takes 2 kN jammed, and from D2 to the foot
    it buckled out of its plane at SF 0.6).
    - Each side is then a deep box through its front leg.
  - The strut from the hinge G back to the gearbox at O is 48 mm wide (`Stand.wide`): O
    is held across the plate only by its members, and the legs from O leaned on it.
  - Nothing crosses the bay over the mouth.
- **The base board** (`cradle._base_plate`, the part `base_plate`): 9 mm Baltic birch
  plywood, routed by SendCutSend (`birch_9mm`, the user's of 2026-10-13), 584 mm across
  the bay (`cradle.BASE_HALF`), 684 mm long on B and 682 on A (`Stand.base`: a full 6 L
  tips past 25° over every edge of B's; A's rear edge 23.9°, its bar 15°). B's sits 40 mm
  toward its drive side (`cradle.BASE_Z`, round 1 of 2026-10-14: the gearbox and the hand
  crank put the cradle's centre of mass 68 mm that way). Its corners are rounded 35 mm and
  the notch's mouth 8 mm (`cradle.BASE_CORNER_R`, `NOTCH_MOUTH_R`): a square plywood corner
  chips and catches a shin. SendCutSend publishes no plywood price: its cutting is quoted at
  upload.
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
    notch and 134 mm behind it (A: 92). Its 135 g of birch, with the toggle flag's 26 g
    behind the gearbox, took B's rear edge from 25.1° to 24.997°: the board grew 2 mm at the
    rear, to 684 mm (25.2°).
  - **The cleats** (`cradle._cleats`, the parts `base_cleat_*`; round 1): two 26 mm strips
    of the same birch glued to the board either side of each outer plate's rail, between
    its brackets, 0.15 mm off each face (`cradle.CLEAT`). They hold each side upright on the
    board, where its two printed brackets alone let it sway several millimetres under a
    20 N push at the mouth, and they drop the rails into place when the cradle goes back
    on. They're glued to the cradle as built: the guide pencils each round its rail with a
    sheet of paper for the play, lifts the cradle off, then glues them on the marks
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
      bracket's foot creep under the washer: snug it, and reprint that bracket in PETG.
    - All the bolts go in with the board. On A the passive side is set down 7-15 mm out from
      its place, inside its bolts, and pushed home; on B the whole cradle is set on the
      board, its passive panel built and lowered lying down ("Building one").
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
    fixed heights, stacked in the locator's pocket. Bottom up: two blocks side by side
    (`cradle.RISER_BLOCK`, 100 x 49.8 x 85 mm, cups open underneath), then plates of 20, 10
    and 5 mm (`cradle.RISER_PLATES`); each piece's 2 mm 45° boss (`cradle.RISER_LIP`) rides
    up into a recess under the next with 0.5 mm of play, so the stack can't slide apart, and
    each prints upside down with no support. The whole set is 122 mm (`cradle.DECANTER_RISER`,
    the drawn RIEDEL Ultra's). It can't be raised mid-pour without lifting the decanter off
    it: that closes round 2's linkage review's raised-decanter case physically.
    - The blocks are also the drive panel's lay-flat blocks on the trip apart and back
      (`cradle.LAY_BLOCKS`, "Flat pack"): 87 mm with their boss, as far as the gearbox stands
      off its outer plate (87.5), and a 12 mm relief in each boss's middle takes the screw
      head it sits under. Two jobs, no part more to store or lose.
    - The stacks it makes (`cradle_report.riser_stacks`): the plates alone, 7 to 37 mm, or
      on the blocks, 87 to 122 mm, every 5 mm. For the listed vessels it is within 5 mm of
      the rule's most but for one glass (the Zalto Burgundy: 37 against 78; lower is safe).
    - About 350 cm³ of PLA in all (the blocks 93 cm³ each, the plates 66, 55 and 46).
  - Four printed TPU feet snap into its corner holes (a cone and a bore up the stem). On a
    counter that isn't flat they take up about 1 mm; past that it rocks, and a thicker foot
    goes under the corner that lifts.
- **The toggle flag** (B: `cradle.TOGGLE_FLAG`, `cradle._toggle_flag`, the part
  `toggle_flag`; round 2's M7): a 4 mm plate printed in red on the drive side's outer face,
  screwed to its rear leg (two M5 button heads through the leg's holes at the stand nodes
  `S1`, `S2` into nuts over its inside, claimed in layer 1 as a foot bracket's), its arc 108
  to 128 mm round the crank's centre O over the rest's toggle zone, 25° of crank either side
  of the arm's direction at rest (`cradle.TOGGLE_ZONE`: two handle turns, round 2's drive
  review's O2 < 1.3 band). Behind the gearbox, in the open, the crank's arm points into the
  red over the last two turns back to rest: "hands out, nothing under the cradle box" (the
  use limits' third line). Near full tilt B's arm turns behind its outer plate's strut O2-O,
  so that end is the bottle's tilt and the text's; A's arm is behind its plate at both ends:
  no flag. It keeps clear of the crank's sweep (the crankpin reaches 105 mm) and the hand
  crank's (the clash test's sweep). It is on the frame side; round 2's drive work cues the
  toggles on the drive side.
- **The light:** a 14 x 60.5 mm AAA keychain light (Olight i3E EOS) stands in a printed
  clip under the front saddle's back end (its flange on the carrier's underside, two M5s),
  so the clip prints upright and round. Its head sits under the
  shoulder; an M3 set screw holds it while you twist the head to switch it on.
  - A longer light's tail reaches a decanter's shoulder at rest.
  - Its printed clip is the lowest part of the cradle box over the decanter's rim: 380 mm
    over the table, at rest ("The vessel under the mouth").
- **Display only** (`fab="display"`: drawn, never bought, cut or printed): the bottle (a
  6 L methuselah, `cradle.SHOWN`: the V blocks drawn at their lowest; the cradle page sets
  them for the bottle picked) and a RIEDEL Ultra decanter (225 mm, `cradle.DECANTER_PROFILE`)
  on the printed riser, the whole set (`cradle.DECANTER_RISER`, 122 mm), in the riser
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
  adapter liners: DECISIONS.md. Round 1's drive: 125 mm on B, 85 on A, "The drive" above.)
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
  - The crank block: four keyed plates on the NRV040's 19 mm shaft and 6 mm key.
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
knob took 55 turns and bent at a knock. What changed for it is under "Review round 1"
below; the rest of this section describes the saddles as they now are.

**The setting** (`cradle.setting`): a bottle lies with its axis level through the pivot
when each 90° V's apex is `(r + 2) √2` under the axis (`r` its radius, 2 the liner), so
each V block rises `√2` times what the neck or the body is thinner than the 6 L
methuselah's. The base stop's face goes half a millimetre behind the base.

| bottle | front V block rises | rear V block rises | base stop's face behind the mouth |
|---|---|---|---|
| 750 ml Bordeaux | 12.7 mm | 68.9 mm | 300.5 mm |
| 1.5 L magnum | 12.0 | 53.0 | 356.5 |
| 3 L jeroboam | 7.1 | 23.3 | 470.5 |
| 6 L impériale | 0.7 | 6.6 | 493.5 |
| 6 L methuselah | 0 (on the rods) | 0 (on the rods) | 571.5 |

The front V block travels 15 mm, the rear one 72: a neck from 28 to 48 mm across, a body
from 76 to 174.

**The mechanism.**

```
  front saddle (looking along the axis)        rear saddle (looking along the axis)

    bar  gauge's pad       knob (M6)           post                    knob (lead screw)
     ||   ____             _|_                  ||      6 L           ___|___  <- tower's arm
     ||  |pad |  .-.      |#|                   ||   .-'''-.           |#|  |
   __||__|____| ( O ) ____|#|_ <- tab + M6 nut  ||  /       \       ___|#|__|_
  |  post   \__  \ /  _/  |#| |              ___||_ \  axis /  ____|  boss  | | <- tower
  |   |      V  block     |#| |             |   ||\  \____/  /|    | (nut)  | |
  |___|___________________|_|_|             |___||_\  V    /_|____|___||___|_|
   ===o========o===  tie rods                ===o=====\block/=====o==[collar]=== tie rods
```

- **Front:** the V block slides in a box (the carrier's walls, the front and back ones
  notched under the lowest V; two drains through the back one at the floor, its low end at
  rest). Its tab at the +z end holds an M6 nut. The M6 x 60 tap bolt stands in a blind bore
  in the carrier's floor, through the nut, its printed knob on its head over the wall. The
  knob stays put: turning it walks the nut, and the block, up or down (13 turns end to
  end). The block bears down on the nut, so the thread (self-locking) holds it.
- **Rear:** the V block's two end bosses ride a NETRF12 post (12 mm, at -z) and a
  Tr8 x 8 lead screw (+z, 8 mm a turn: 9 turns end to end), whose bronze barrel nut sits in
  the boss from below, an M3 set screw on it. The post stands in the carrier's -z side
  block; the screw's foot in the +z one, through its clamping collar in a pocket there on a
  TPU brake washer. A tower on that block rises beside the screw to an arm over its top: a
  bushing round the screw under the printed knob.
  - An 8 mm lead isn't self-locking (the thread's lead angle, 20°, is over its friction's).
    The collar holds it: the block's load presses the collar's 21 mm face on the TPU
    washer, whose grip at that radius is 4.1 x the thread's turn back under the same load,
    whatever the bottle (`cradle_report.saddles`, a greased thread). The knob turns easily
    with the bottle out, when the block carries only itself.
  - The screw holds the block at one end. A bottle's weight at the axis rolls the block
    about it, and the post takes that couple (it is 22 x stiffer than the lead screw's
    core). That is why the post is 12 mm, not 10.
  - The screw's knob sits at |z| 110, 11 mm outboard of a 6 L's widest as it's laid in
    (the M8's was at 4). A knock at the knob goes into the tower's arm, not the screw, which
    is held at both ends (`saddles`: 6.3 MPa in the tower at 50 N; the old free M8 yielded
    at about 50 N).
  - Gussets under the flats spread the couple into the block.
- **The scales:** grooves the V block's top edge (front) or its screw boss's top (rear, on
  the tower's face) reads. One row has a groove per bottle, a dot each per place from the
  top: in front three (`cradle.FRONT_GROUPS`: the 750 ml and the magnum share one, the two
  6 L another; their necks rise the block 0.7 mm apart, so each pair is set at the mean,
  0.45 mm off at the mouth), at the rear five. The other row is graded in diameter: the
  neck's every 4 mm in front, the body's every 10 mm at the rear. No text is printed on
  them: glyphs would depend on the fonts of the machine that builds the STL.
- **No depth added:** each carrier's bottom is the old saddle's (front -60, rear -149).
  The V block sits on the rods at its lowest, and its guides stand beside the bottle (|z|
  91 to 126 at the rear, inside the bay's 130). `decanters.TIES` didn't move: neither
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
lined with TPU, so the rear support runs to 588 mm (the body's heel, 10 mm short of its
base, ends it for each bottle):

| bottle | at rest | level | full tilt | its support's end |
|---|---|---|---|---|
| 750 ml | 223 | 191 | 199 | 290 |
| magnum | 276 | 230 | 238 | 346 |
| 3 L jeroboam | 397 | 325 | 334 | 460 |
| 6 L impériale | 405 | 319 | 329 | 483 |
| 6 L methuselah | 482 | 390 | 400 | 561 |

The reaction's centre (mm from the mouth); every one at least 62 mm inside its support. At
full tilt the lip gauge's push over the axis moves it back too (by the rim's height times
tan 30°). The rails join the block in a socket under each flat, their tongues 28 mm in, so
the body's load on the flanges' end goes into the block in bearing.

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
Its pad bears on the top of the rim, over the bore; the stream leaves the bottom of the
bore, and the decanter is below. Its bar runs back beside the neck, so the bottle lays in
under nothing; a bottle's push on the pad is 41 mm off the hinge's line, and the hinge's
two knuckles either side of the post take that turn on the pin (the push itself bears the
back knuckle on the post's back face, not on a thread). It can't be forgotten up: lifted
to uncork, it stops at 70° with its centre of mass still inboard of the hinge, and falls
back onto the mouth when let go.

**Why the base stop rides the V block:** a 750 ml's base (300 mm) and a magnum's (356) lie
inside the rear V (255-380), where the old spine couldn't reach. A gate across the V,
keyed to the block's flats and continued by rails behind it, serves them and the 6 L's
(571) alike, and the rails, rising with the block, give the V's flanges behind it the same
height at every setting. It drops the 155 g spine; at rest it comes no nearer the board
than 26 mm (the old rails' end had been 18.8 mm over it, under a finger's 25 mm).

**Use** (the guide's "Setting it for a bottle"):
1. At rest, the bottle out, set all three by their scales: the gate's pointer to the
   bottle's groove across the rail or flat, its knob tight by hand; each V block's reading
   edge to the bottle's groove (a dot per place on every scale). Another bottle: its length
   (plus half a millimetre) on the 10 mm grooves, its neck's diameter measured 50-80 mm
   from the lip (where the front V holds it: a Port bottle's bulb or a tapered neck sets by
   that), its body's.
2. Lay the bottle in, its label up, its base to the gate: it slides back onto it. The
   gauge stays down; its pad stands a millimetre or less in front of the rim.
3. To uncork, lift the gauge onto its stop and hold it (or let it rest on the corkscrew);
   let go and it falls back onto the mouth. It must be down before the handle turns.

**The parts** (prices from pages rendered 2026-10-13 and 2026-10-14, qty 1, per cradle;
the sourcing in `/root/scratch/decant/saddles/sourcing/` and
`/root/scratch/decant/review5/r1_mounting/pages/` on the development box):

| part | where | USD |
|---|---|---|
| Tr8 x 8 lead screw, 150 mm, stainless | ServoCity 3501-0804-0150 | 6.99 |
| its barrel nut, bronze, 12 x 16 mm | ServoCity 3500-0804-1216 | 4.99 |
| its clamping collar, 21 x 9 mm, aluminium | ServoCity 3504-0804-2109 | 6.99 |
| M6 x 60 tap bolt, fully threaded, 18-8 | Bolt Depot 5973 | 0.60 |
| M6 hex nut, 18-8 | Bolt Depot 4776 | 0.11 |
| NETRF12-130 standoff (the rear post) | MISUMI | 16.98 |
| M6 x 12 button head (the post's foot) | Bolt Depot 6644 | 0.19 |
| M5 x 16 button heads: the rails' four | Bolt Depot 6634 | 4 x 0.14 |
| M5 x 25 tap bolt (the base stop's lock) | Bolt Depot 5952 | 0.22 |
| M5 nut and two fender washers (in its T-slot, on its ear) | Bolt Depot 4775, 7314 | 0.07 + 2 x 0.10 |
| M5 x 40 tap bolt and nylon-insert nut (the gauge's pin) | Bolt Depot 5955, 4794 | 0.34 + 0.09 |
| M3 x 6 set screws: the barrel nut's, the rear knob's | Bolt Depot 8707 | 2 x 0.09 |

That is about USD 38.50, USD 14.70 more than with the M8 screw and coupling nut, the screw,
its nut and collar most of it (a 3D printer's T8 x 8 kit off the shelf, screw, nut and
collar, would do; ServoCity's are the ones priced). Everything else is printed: the two
carriers, the two V blocks, the TPU liners (the Vs' and the rails' flanges') and the
rear screw's brake washer, three knobs, the gauge, the rails and the gate.

**What holds** (`cradle_report.saddles`; a sealed full bottle, its worst way for each part,
its centre of mass the shell model's; printed parts against 10 MPa, a quarter of
printed PLA's strength across its layers, for a load that stays):

| part | load | stress |
|---|---|---|
| rear V block on its rods' seats | 109 N (a 6 L, level) | 0.28 MPa |
| rear barrel nut on the block | 116 N (a 6 L hung) | 2.2 MPa |
| rear post, bending | 12.2 N·m roll, a 6 L hung at the screw | 72 MPa (SF 3.05 on 1018's 220) |
| rear post at rest | 12.1 N·m: a hung impériale's pull along the axis, 121 mm over its root, and its roll | 71 MPa (SF 3.1) |
| rear lead screw | its share of the roll; a 50 N knock between its nut and bushing | 33, 56 MPa (SF 6.4, 3.8) |
| rear lead screw's brake | the block's load on the collar's TPU washer | 4.1 x the thread's turn back |
| rear knob's tower | a 50 N knock at the knob | 6.3 MPa |
| base stop's gate, bending | 96 N at rest (with a corkscrew's push) | 5.4 MPa |
| base stop's lock | 96 N | its grip 300 N (SF 3.1: 0.5 N·m by hand, two faces) |
| base stop's T-slot lips | the lock's 500 N preload | 6.9 MPa |
| base stop's rails at their sockets | 6.5 N·m each: a 6 L's body on the flanges' end, level | 4.1 MPa |
| the sockets' floors | 346 N under each tongue's end | 7.7 MPa |
| lip gauge's pad, arm, bar, bridge | 54 N at full tilt (no friction) | 1.1, 2.6, 8.4, 5.9 MPa |
| lip gauge's post | the same, bent and twisted (its 41 mm turn) | 6.4 MPa |
| lip gauge's pin (M5) and the post's bore | the knuckles' 89 N couple | 42 MPa (SF 4.9); 7.5 MPa bearing |

The V blocks carry no jam load: the tie rods and the cradle links do.

**What they weigh:** the saddles, the gauge and the base stop come to about 1.6 kg, solid
(the report counts a printed part at full infill), 0.3 kg more than with the M8 screw, most
of it the rails' flanges 100-210 mm behind the rear V. With a sealed full 6 L at its
shell model's centre of mass (390 mm) that puts the crank's peak at 35.1 N·m on B (34.3 with
the old saddles: the gearbox's SF 1.28, was 1.31; 26.0 N at the grip, was 25.4) and 36.9 on
A (36.0: SF 1.22, was 1.25; 26.0 N); the stand tips over its rear edge at 25.0° on B and
24.1° on A. B's gearbox (its 1.3 bar) and both grips (the relaxed 26 N bar) are past what
the tests ask: the drive's to settle (2026-10-14). Round 1's drive set both bars on cited
sources ("The drive"): the gearbox rated at a hand's speed, 1.65 on B and 1.57 on A; the
grip under MIL-STD-1472H's 22 N, 20.8 N on B's 125 mm crank. A's 85 mm crank (the longest
its cradle clears) takes 32.2 N: open. B's board is 682 mm (25.1°; 684 since round 2's window and flag, 25.2°).

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
  penlight (m6, S4); the tie rods stay NETRF10 (S5); the post's socket isn't deeper (m2: the
  post itself is rated at rest instead, SF 3.1).

## The vessel under the mouth

At rest, 50° neck up, the cradle box hangs straight under the mouth: the front tie rods, the
front saddle and the light cross the decanter's axis. `cradle_report.room` measures the room
a vessel has round its axis at every height, over the whole swing and from the pour on (the
cradle past −20°), with the drawn 6 L methuselah in the saddles (the V blocks at their
lowest: a thinner bottle's ride higher, off the decanter), and names the cradle box's part
that comes lowest over the drawn decanter's rim (within its 37 mm of the axis) over the
swing (`room`'s `lowest`): **the light's printed clip, 380 mm over the table, at rest**. The
front saddle the guide had named is 66 mm higher (447 mm).
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
  set, 122 mm (`cradle.DECANTER_RISER`), its rim at 347 mm, 33 mm under the clip. A RIEDEL
  Cabernet Magnum carafe (269 mm): the blocks alone, 87 mm; a RIEDEL Ultra Magnum (240 mm):
  117. Before round 2 four heights were given for the RIEDEL Ultra: the guide's size rule
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
guide's last 19 steps and the cradle page's; the cradle box on its own since the pack's
turn 1, the same day). The base board, the passive side panel, the cradle box, the drive
side panel, the riser and the hand crank. No tool, no small part left loose: 740 x 316 x 600
mm outside, about 141 L (the real-shape packer's plan, on the planner's 16-layer stack of
2026-10-14; the riser fits its free room: the box is unchanged), where the drive side
carrying the cradle box had packed in 699 x 498 x 608, 212 L (its flat pack had been 112).
1. Crank it to the pack pose: from rest, turn the handle the way that swings the crank's arm
   down, until the arm lies along the drive side's outer plate's rear leg, straight down
   from the gearbox (`cradle.PACK_POSES`, `pack_pose`; `cradle_pack.pack_t`). Seen from
   outside the leg hides it. It is about 5½ turns from rest and the bottle is 31° neck up.
   The decanter away, the riser comes out of its locator: its plates, then its two blocks.
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
   drive side 54 mm, over the box's bound.)
3. Undo the four thumb knobs on the base board's bolts by hand and lift the cradle off the
   board by its two outer plates' top members (never by the cradle box: only the worm holds
   it). The bolts stay in the board, the washers and knobs back on them.
4. Stand the riser's two blocks on a folded towel and lay the cradle on its drive side on
   them and the gearbox (`cradle.LAY_BLOCKS`): one under pillar O1's screw head (the head in
   its boss's relief), one under column D2's on the front leg. It lies level, the passive
   side up; gravity now runs along the pins, so no link can swing. A hand on the cradle box
   while the other works: lying, the cradle tips past 16.9° and 12 N at its top.
5. Pull the two R-clips of pins X and Y on the passive side, reaching into the bay, from
   under the passive cradle link ("The quick-release pins"). Each pin stays in its link,
   standing on its head, its TPU keeper on it.
6. Lift the passive side panel straight up off the pins by its outer plate, level, its pins
   and their keepers with it, push each clip back through its pin's hole, and set the panel
   down on its outer plate. The drive side with the cradle box stays lying as it is, the
   worm holding it.
7. Pull the two R-clips of pins X and Y on the drive side, reaching in from the side between
   the drive side's cradle link and its inner plate (the cradle box is open at its sides),
   a hand on the box. Each pin stays in its link: its head is under the link, and its keeper
   holds it.
8. Lift the cradle box straight up off the drive side's pins, level, by its top cradle link
   and a tie rod (3.2 kg), turn it a quarter about its tie rods and set it down on its two
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
     weight is not what they meet. Lowering the box (3.16 kg) on pin X and turning it until
     Y lines up rests up to half its weight, 15.5 N, on a pin's tip, which pushes the pin
     head-first out of its link through a keeper at its least grip (12.2 N; 6.1 N at the old
     7.6 mm bore). Two answers: the way back now holds each piece's weight until both its
     pins are in their holes, never on a pin's tip (and says how to push a slid pin home);
     and the guide's bench check pushes each pin's tip with 30 N (`cradle.KEEPER_PUSH_N`,
     1.94 x the 15.5 N: `keepers`' `load_ratio`, the bar 1.5), so a keeper that would let go
     is reprinted before it ever does.
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
| the cradle lying on its drive side on the riser's blocks | 9.57 | the gearbox and the two blocks | 0.1° | 16.9° | 12.1 N |
| the drive side and the box, lying, the passive panel off | 7.71 | the same | 0.1° | 16.3° | 8.5 N |
| the drive side panel alone on its blocks (the way back starts here) | 4.55 | the same | 0.1° | 35.2° | 14.4 N |
| the cradle box on its links' long edges, its saddles' tops down | 3.16 | both cradle links' edges and the base stop's knob | 10.9° | 41.5° | 15.5 N |
| the passive side panel on its outer plate | 1.86 | pillar O1's screw head and its two brackets | 3.6° | 56.2° | 10.9 N |
| A, off its board while the finish cures, on its drive side on the blocks | 8.4 | the gearbox and the two blocks | | 16° | 10 N |

Round 1's poses had been perches: the cradle and the drive side lay 14° tilted on the
gearbox, the O1 pillar's screw head and the front bracket (the whole tipped at 15.4°, 9.1 N;
the drive panel alone, where the way back starts, at 6.8° and 2.5 N), and the box "set down
as it was, on its other cradle link" stood on L.b6's edge and two of the four tie screws'
2.75 mm heads, its centre of mass 22 mm outside their quadrilateral: it tipped at 10.0° and
2.8 N. (The "41 mm" quoted for it was to L.b6's outline, which it never sat flat on.) The
stacked poses gain little from the blocks (the box at the pack pose overhangs the line O1-O):
the steps keep a hand on the box.

**Back together** (the payload's `reassembly`, the guide's "Back together" steps):
1. The riser's two blocks on a folded towel, the drive side panel laid on them and its
   gearbox, outer plate down; its pins X and Y standing in their links on their keepers, the
   clips out, each pushed fully home by its head.
2. The cradle box lowered onto it, level, its knobs' side up, its weight held until both
   pins are in their holes: pin X into its hole first, then the box turned about it and b5
   swung until Y lines up. Never on a pin's tip. Both clips in from the side.
3. The passive panel on its outer plate, its pins held by their clips; its links set to the
   pack pose by the paint marks, a strip of masking tape across b3 and its inner bar (not
   b5: it is the one nudged into place, round 2's m5).
4. Turned over and held level: each clip out; each pin hangs by its head, its keeper on it.
5. Lowered level onto the cradle link, its weight held until both pins are in: pin X into
   its hole first, then b5 nudged until Y lines up. The tape off.
6. Both clips in from under the cradle link: a pin without its clip is held by its keeper's
   grip alone.
7. Stood up and set on the board: each rail between its cleats (their lead-ins guide it),
   the brackets' feet over their bolts; the washers and knobs on, the passive side's only
   started.
8. The riser back in its locator.
9. The hand crank on.
10. Turned to level (about 8½ turns from rest) with the passive knobs loose, then snugged
    there: B's two remote centres agree best at level (round 2's m6).
11. The first build's checks again, the bottle out: both cradle links swing freely, the
    handle turns evenly with a fingertip (under 5 N at the grip, the luggage scale hooked on
    it under 0.5 kg: `cradle.EMPTY_GRIP_N`; the report's `empty_handle_force_n` is 3.6), and
    the rod's paper check. All four clips in.

The pieces: the base board (its feet, cleats and riser locator, its bolts, washers and
knobs), the riser (its blocks and plates, one piece: `cradle_pack.RISER`), the passive side
panel (every part of that side but its cradle link, its
brackets), the cradle box (both cradle links, the ties, the saddles with their V blocks,
knobs, post and lip gauge, the light, the base stop), the drive side panel (the gearbox and
every part of that side but its cradle link and the hand crank: the worm holds its links as
cranked) and the hand crank. Each side's b5, hanging on J2, swings into its panel
(`cradle_pack.tuck`). B's plan (`cradle_pack.PACK_PLANS`): the cradle box in first, on the
board as it stands in use, along its length; the hand crank on the board beside it; the
drive panel as it lay when the box came off, its outer plate down, its plates over the box
and its gearbox down beside it, on six foam pads (54 mm at most); the passive panel last,
its bay face down over the drive panel's, turned a quarter, on eight pads (51 mm at most);
the mass centre 40 % up the box. The box's length is the cradle box's, 723 mm levelled
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

The steps lift the cradle off the board before a panel comes off: its brackets' feet
sit on the bolts, between the cleats. The carriage bolts' square holes stop them turning but
don't grip them: with the cradle off, the knobs go back on the bolts.

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
inside), 3 mm of foam between two pieces (`CLEAR`, one cell all round).
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
  else over foam pads cut to fill gaps of up to 60 mm under it (`PAD_MAX`; the pads are
  reported), else it would tip.
- **Plans** (`cradle_pack.PACK_PLANS`, `voxpack.Plan`): a human packer's intent per piece
  (the face down, a quarter turn, a corner, the piece it rests on or nests into, what goes
  in first, why). The packer honours it and picks what it leaves free; a piece it doesn't
  name goes in after, largest first (the panels first, in every order).

**Judging and searching it** (`spiderpig pack-inspect`, `docs/agentlib/TESTING.md`): a
contact sheet per candidate (the top, front, side and isometric views, each piece in its
colour and numbered in the order it goes in; slices at 25, 50 and 75 % of the height; the
legend), and the metrics (`voxpack.metrics`, any layout voxelized afresh): the box, the
true fill (the parts' own volumes over the box's inside), the air per layer, how each piece
rests (on what, directly, on pads, or not), the real-shape overlaps (cells two pieces share,
measured again on 1 mm cells), the order's faults, the mass centre's height. `--search`
anneals the order, the orientations and the corners round the plan (`voxpack.search`, in
worker processes); `--split` packs a what-if (some parts off their piece: a design change
before it is made). The cradle page's pack view edits a layout by hand (Edit the packing:
click a piece, drag its arrows in 3 mm steps, R turns it a quarter turn, T tips it; Export
layout) for `--layout` to judge.

No MuJoCo settling: per-part convex hulls fill a truss plate's triangles and a panel's bay,
so every nested piece would start inside another's hull; an honest model needs a box per
column of voxels (thousands of geoms for B's drive side). The voxel support check (centre
of mass over real contacts, pads named) answers "does it float" without it.

| round 1's design (2026-10-14), outside | before: the boxes' packer, judged on real shapes | after: the plan |
|---|---|---|
| B | 702 x 485 x 606 mm, 206.2 L, fill 3.8 %; the mass centre 52 % up | 700 x 478 x 604 mm, 201.9 L, fill 3.9 %; every piece resting directly; the mass centre 27 % up |
| A | 698 x 295 x 602 mm, 123.9 L, fill 6.2 %; the hand crank and the rear saddle tip (the saddle floats on a box that isn't there) | 700 x 185 x 601 mm, 78.0 L, fill 10.1 %; the passive panel on 5 pads to 33 mm, two tie rods on pads to 42 mm |

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

The box's bounds per design are in `tests/test_cradle.py`: `PACK_BOX` (A) and
`MODULE_BOX` (B, read off its base board's size).

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

The jam case is the most the hand crank's slip clutch lets onto a blocked cradle: 61.3 N·m
at either crank (`cradle_report.jam_nm`, `config.PRODUCT_JAM_NM`; "The clutch's cap"
below), however hard the push. Until round 1's drive it was a firm push at the grip
(`cradle_report.JAM_HAND_N`, 40 N: about 1.7 x what the worst turn takes) through the worm
at a hand's speed (`WormBox.eta_hand`, 0.55 on the NRV040): 66 N·m at B's crank on its
100 mm handle, 69.3 at A's on its 105, and no bound.
- Rated at the gearbox's rating instead, it would be reached at less than the worst turn's
  own force: no bound (all three reviews of 2026-10-10).
- The joints' rows leave out 30° of crank round either toggle, where a blocked
  crank-rocker's forces grow without bound; the clutch's case rates them (below). There
  only clearance protects: the decanter's lip and anything under the cradle.
- What A holds there: pin E (steel) 3.31 (3.26 before the planner of 2026-10-14 put it in
  12 layers, its span 8.55 mm), the crankpin 1.81, the keyed plates 2.15, the
  rod's buckling 2.32, pillar G 1.83 (on its box: below), its cradle link b2 1.82 (in
  bending, chord by chord; 0.187 in 5052, its E-Tr1 32 mm wide against buckling), the
  columns 2.44 and more, the outer plate's members 2.25, the inner's 3.34 (at the 40 N
  push's 69.3 N·m: 2.88, 1.6, 1.9, 2.05, 1.63, 1.61, 2.17, 2.01, 2.97).
- Turning it: 36.9 N·m at A's crank at worst, a sealed full 6 L on the mounting round's
  saddles: the NRV040's SF 1.57 at a hand's speed (1.22 on its 45 N·m). 32.2 N at A's 85
  mm grip, over the 22 N bar (MIL-STD-1472H's light-load crank): open (DECISIONS.md,
  2026-10-14, the drive). It tips at 23.9° over its rear edge (its test's bar is 15°; 27.0°
  with the wine leaving as it pours), 25.5° to its drive side, 30.5° to the other and 38.4°
  forward, and takes a 35.6 N push at a bottle's top (toward the drive side), the worst over
  the swing (round 2's window under the base stop: 24.0°, 25.6°, 30.6°, 38.6° and 36.1 N
  before).
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
- **Posts on the box** (`cradle_structure.posts`): the pillars and columns between a
  side's plates solved as one frame: each post a steel beam, its foot fixed in the outer
  plate's plane, its end moving with the inner plate, which is rigid in its plane and held
  by nothing but the posts; each end held against turning only as stiffly as its plate's
  24 mm strips bend and twist there (`cradle_structure.plate_spring`). A plate on two posts
  in a line (B's bar O1-O2 alone) turns about one and barely holds the other across the
  line. A post's tapped ends are rated on the tube the tap leaves, the rest on the bar.
- **The frame plates' members** (`cradle_structure.members`): each plate a plane frame of
  its members under the posts' feet (and the gearbox's force and torque at O), standing
  on its feet; each member's in-plane stress, Euler out of the plane between nodes the box
  holds, and where a node isn't held (the gearbox's O) its tilt on the members there
  (`k L`). Also each plate where a post's end turns it out of its plane.

**The known limits: a blocked cradle** (`cradle_report.blocked`, the report's `blocked`).
The jam case is a couple on the cradle. A real block is a force at a contact, and near the
mouth its lever is short. The same 61.3 N·m, the clutch's cap, against a block:

| block at | B: the block's force | B: J / O1 / X / K | B: weakest | A: weakest |
|---|---|---|---|---|
| the front tie Tf1 (the front saddle, the light, the front ties on a raised decanter's rim), 72 mm from the mouth | 3.43 kN | 6.2 / 5.9 / 3.5 / 3.4 kN | the outer plate 0.18, pillars O1 0.32 and O2 0.36, pin K 0.70 | the outer plate 0.23, pillar G 0.81 (G 3.9 kN) |
| the rear saddle's underside, 351 mm out | 0.71 kN | 1.5 / 2.0 / 1.0 / 1.8 kN | the outer plate 0.78, pillar O2 0.93, O1 1.07 | the outer plate 0.94 |

Not designed for: nothing that holds a block 72 mm from the mouth at the cap fits the
cradle, and no clutch a hand can turn brings those to 1. "In practice the glass decanter
breaks first" was unverified (round 2's linkage review found no rim-strength figure): the
frame starts to give at about 10 N over the normal turn at B's grip, about 1 kN at the front
tie (the outer plate O1-O buckles at +6.5 N, the pillar O1 and the plate round its foot
yield at +10 N). Since round 2 the decanter stands on a printed riser that can't be raised
mid-pour, its rim 20 mm under the lowest part over it ("The vessel under the mouth"). (At the 40 N push, before round 1's
drive: B's outer plate 0.17, pillars 0.30 and 0.34; A's plate 0.21.)

**The clutch's cap** (round 1's drive review, 2026-10-14: `config.PRODUCT_CLUTCH_NM`,
`cradle_drive.clutch_case`, the report's `jam_capped`). The hand crank's slip clutch slips
at 3.375 N·m on the input (27 N at B's grip, 40 at A's), at most 10 % more once set, so the
crank never sees more than 61.3 N·m on either, through the worm at a hand's speed, however
hard the push. That is a bound, and the jam case; the 40 N push that had been the case
isn't one (a hand on a crank that feels stuck pushes 60-150 N). The report rates the cap
over the whole turn, the toggles included:
- The report's own model rates it (`cradle_report.structure` with no toggle skipped, the
  links chord by chord, the posts on their box, the plates' members): 30° of crank and more
  from either toggle every part holds it, B at SF 1.36 or more (pillar O2; 1.31 at the
  40 N push's 66 N·m with the old saddles), A at 1.80 (b2; 1.61 at 69.3 N·m).
- Nearer a toggle the rod's force goes as the cap over the sine of the crank's angle to the
  rod, and no torque bounds it. B: pillar O2 1.15 at ±25°, 0.93 at ±20°, 0.46 at ±10°; the
  rod buckles (elastically, into its 3 mm gaps) within ±17° of rest and ±9° of full tilt. A:
  every part holds 1.3 outside ±25° (the last 3.8° of tilt before rest, 5.9° before full);
  b2 0.85 at ±15°; the rod buckles within ±12° and ±6°. Only clearance protects there: the
  use limits keep everything from under the cradle box and the bottle in the last two or
  three turns.
- The gearbox at the cap: 1.36 x its 45 N·m rating, 0.68 x the catalogue's momentary limit
  (2 x Mn2, p. 9: 90 N·m), on either (the 40 N push had been 1.47 and 1.54 x).
- The input shaft's radial load (`input_overhung`): the grip's middle 155 mm past the
  input's shoulder, where the catalogue allows Fr1 a / (b + x) = 149 N on the NRV040 (p. 43:
  350 N, a 106, b 95); the clutch's top is 29.7 N at B's grip (SF 5.0), 43.7 at A's (3.4).
- The keyed plates hold 2.15 x the cap filed together on the key; one plate bearing alone
  0.54, and 0.94 at B's walking peak, so the guide files them together.
- The bars are the tests' (`tests/test_cradle.py`): the clutch's least slip at least 1.1 x
  the worst starting turn (1.17 on B, 1.11 on A), every part at 1.3 or more 30° from the
  toggles, the cap within the gearbox's momentary limit.

Every joint on both sides is rated at the full planar load (`cradle_report.SHARE` = 1),
not a share of it. The rod drives one side, whose pivots carry half the weight plus the
rod's whole force.

`spiderpig/cradle_report.py` (`report(config)`) measures:
- **Swing:** the swing and each bottle's own mouth wander.
- **Drive:** the crank's peak torque, the gearbox's safety factor at a hand's speed and on
  its 45 N·m, the force at the crank's grip against MIL-STD-1472H's 22 N, the jam's torque
  (the clutch's cap) and the 40 N push's beside it, the turns per pour; the clutch's slip
  against the worst starting turn and its cap, the gearbox at the jam against its rating
  and its momentary limit, the input shaft's radial load, the keyed plates
  (`cradle_drive.drive_limits`).
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
- the crank's grip force at a sealed full 6 L's worst start under 22 N (MIL-STD-1472H's
  light-load crank, Figure 13), its throw within 125 mm: A misses it at 32.2 N on its 85
  mm crank (open; the test checks it last);
- the gearbox's safety factor at least 1.3 at a hand's speed (Motovario's NRV040 30:1 at
  its slowest listed input, 500 rpm, scaled to Phidgets' 45 N·m: 58.1 N·m) and 1 on its
  45 N·m rating, the peak a sealed full 6 L at one tilt;
- the clutch's least slip 1.1 x the worst starting turn or more, its cap within the
  gearbox's momentary limit (2 x its rating), the input shaft's radial load at the
  clutch's top at least 2 x under the catalogue's;
- every pin, pillar, column, link and frame plate at least 3 walking and 1.3 jammed (the
  links, posts and plates from the force vectors); the keyed plates at least 2.5 walking
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
390 mm from the mouth, the hand crank's clutch's cap jammed; 2026-10-14, after the design
review's structure, mounting and drive rounds):

| check | B |
|---|---|
| swing | −50.0° to +30.0°, the mouth still (0 mm), every bottle's mouth on the pivot |
| transmission | 36.7° at worst (the rod's); the parallelograms 48.5° and 51.5° |
| gearbox | 35.1 N·m peak: SF 1.65 on the NRV040's 58.1 N·m at a hand's speed (1.28 on its 45 N·m); 20.8 N at the 125 mm grip at the worst start (the bar 22 N, MIL-STD-1472H) |
| jam (61.3 N·m, the clutch's cap) | pillars O1 and O2 1.4 (on their box), b6 1.42, b4 1.55 (its chord's buckling), pin K 1.57 (clevis), the outer plate's members 1.61, the inner's 1.76, b1 1.91, b3 2.0, the crank's keyway 2.15, pin J2 2.35 (clevis), column D1 2.35, pin Y 2.36 and pin X 2.73 (clevis), pin P 2.87 (steel), b2 3.29, pin J 11.6 (steel) |
| walking | every joint, link, column and plate 4.15 or more; the crank's keyway 3.76 |
| rod, crankpin | buckling 1.71 × its jam force; the crankpin 2.11 jammed |
| the clutch's cap over the whole turn | every part 1.36 or more 30° and more from the toggles (pillar O2); pillar O2 1.15 at ±25°, 0.93 at ±20°; the rod buckles within ±17° of rest, ±9° of full tilt (round 1's drive review) |
| stand | tips at 25.2° at worst (a full 6 L at every tilt, over the rear edge: a bound; with the wine leaving as it pours, 28.5°); the drive side 29.5°, the passive side 28.6°, the front 40.0°; a push of 45.0 N or more at a bottle's top (an empty 750 ml, toward the drive side). The board 40 mm toward the drive side and 684 mm long (round 2; 682 in round 1: it had been 25.9° before, 26.0° to the drive side, 31.7° to the other, and 32.0 N) |
| stack | 84.05 mm a side, 16 layers (the planner of 2026-10-14; 84.23 in 17 before it, 79.68 before the clevis pin at K and b6 in 0.187 in) |

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
  across only by the plate's members, leans on it.
- **The inner plate** is the bar O1–O2, a triangle O1–O2–D1 off it and a leg D1–D2 down
  over the outer plate's front leg, on two 12 mm columns (since 2026-10-14: the bar alone,
  on its two pillars, held them only along itself; across it O2 was a cantilever off the
  outer plate, about SF 0.4 jammed). Each side is now a box from the pivots down, as A's
  is. The triangle and the leg are in front of the bar, where no tie rod or pin end passes;
  a column between O1 and D1 would stand in the lever b2's sweep. Nothing crosses the bay.
- **The base board** runs from 160 mm in front of the mouth to 524 mm behind it: 684 by
  584 mm (a full 6 L tips past 25° over its sides and its rear edge; it was 596 by 500 for
  a 3 L, 626 by 530 until the stand's check of 2026-10-14, 684 until the corrected centre
  of mass and the structure round the same day, 680 until round 1's drive, 682 until round
  2's window under the base stop and toggle flag). The first refit's had run 827
  mm, to 640 mm behind the mouth, with the crank 480 mm back. Its middle is 40 mm toward
  the drive side (`cradle.BASE_Z`, round 1). Its front keeps its 160 mm: the reviewer's
  35 mm trim (the front tips at 39.5°) would put the passive front foot's pad on that
  side's carriage bolt's head, now the board's passive edge is 40 mm nearer the bolts.
  - The rear edge binds, at 25.2° (bar 25°): with the centre of mass corrected (390 mm
    from the mouth), round 1's structure, the mounting round's saddles, the drive round's
    parts and round 2's window and flag, 684 mm is the least board that holds it (682:
    24.997°). Each 10 mm of board at the rear is about 0.5°; `python -m
    spiderpig.cradle_report stand` re-runs the stand alone.
  - **That is a bound, not the use case** (round 2's m1): `stability` counts each bottle
    full at every tilt, and the rear's worst is a full 6 L at 27.6° neck down, which would
    have poured most of its wine by then. With the wine leaving as it pours (full while neck
    up, poured past level: what it keeps then sits forward, in the shoulder, and only
    steadies the rear; `stability`'s `pouring`) the rear holds 28.5°, the drive side 33.0°,
    the passive side 34.3°: about 3.3° of real margin over the bar, worth some 30 mm of
    board at the rear. (A: 27.0° at the rear.)
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

## Building one

```bash
spiderpig build --linkage decanter_rcm --out build/cradle_b     # STEP, STL, DXF, BOM, ORDER.md
spiderpig guide --linkage decanter_rcm --out build/cradle_b     # ASSEMBLY.pdf and .md
spiderpig bake --linkage decanter_rcm                            # the viewer's .glb (8 s loop)
```

The assembly guide's order is `construction.assembly.CRADLE_MODULE_ORDER` on B and
`CRADLE_ORDER` on A (B's guide: 55 steps, 66 pages; A's: 28 steps, 38 pages):
1. **The drive side**, bottom up on its outer plate:
   - the gearbox under the plate first;
   - the crank block from its lowest plate, the two standoffs screwed to it on the bench;
   - on B, X's and Y's TPU keepers onto their pins over b4 and b5 (each pushed with 30 N on
     a kitchen scale);
   - its cradle link goes on with the four tie rods already screwed to it;
   - its inner plate.
2. **The base board, bare:** its riser locator's layers glued, its feet, bolts in; the drive
   side bolts onto it (it can't stand alone). No cleats and no finish yet: they go on once
   both sides stand on it (step 6).
3. **The hand crank**, and on B **the toggle flag** (printed red) on the drive side's rear
   leg.
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
6. **The cleats and the finish** (round 2's M6): both sides on the board, each rail's cleats
   pencilled round it with a sheet of paper for the play; the cradle lifted off and laid on
   its drive side on the riser's blocks; the cleats glued to the bare board, their lead-ins
   sanded; then every face and edge of the board, the cleats and the locator finished with
   polyurethane; the cradle back on, the passive side's knobs snugged at level, the checks.
7. **Using it** (round 2's M4, each its own step with its own picture, in this order, the
   order `construction.assembly.USE_TAGS` gives: untagged they had sorted alphabetically,
   the trip apart before the setting): before you use it, setting it for a bottle, the
   decanter's riser, decanting, the use limits (a five-line card and the rest for
   reference: `cradle.use_steps`, which the cradle page shows too); on B then the trip apart
   (8 steps) and back together (11), each a step with its picture
   (`construction.assembly.Scene`, drawn by `spiderpig.guide.scenes`: the cradle at its pose,
   laid on its side, its pieces lifted off).

The guide's cover lists the tools the steps name (`cradle.TOOLS`, round 2's m10: about 24,
named only inside the steps until then).

Things to know before cutting and on the bench:
- **Get a live SendCutSend quote** (upload the DXFs). The BOM's cutting figure is a fit to
  the live quotes of 2026-10-10 for A's files (`hardware.bom.CRADLE_CUT_FITS`: USD 367.50
  then); the walkers' per-area rates had priced the old aluminium base plate alone at USD
  488. The plywood (the board, its cleats and the riser locator) and the 7075 have no fitted
  price. The BOM's and ORDER.md's totals say so (`bom.unpriced_cut_note`): "The total
  excludes 2 unpriced cut lines". For the 7075 they give an estimate, marked as one, with
  its basis: the 0.187 in 5052 fit's price for the same parts, x1 to x2
  (`bom.CRADLE_CUT_PROXIES`); on B that is USD 68-136. The birch, never quoted, stays
  unpriced with its area (4439 cm² on B). The cutting's source line names A's quote as A's
  (it had printed "USD 359-368 in all" in B's order).
- **Run the gearbox in and wipe it:** a new worm box can weep oil at its seals, and it
  stands over the base board beside the wine. A few hundred input turns (a drill, slowly),
  then wipe it.
- **The gearbox's go/no-go** (the guide's riser step): on its spacers under the outer plate,
  the shaft's end over the plate's inner face sets the shim rings under the crank block's
  top plate, or calls for 11 or 13 mm spacers ("The drive" above). The output shaft
  matches Phidgets' own shaft drawing (19 h6, collar 24 x 3, 40 past it, a 30 mm keyway);
  the housing's spigot height, hub protrusion and hub diameter are unpublished, and the
  spacers keep them out of the stack-up.
- **File the keyed plates' slots together on the key**, the block tied on the bench, to a
  slide fit: a laser-cut slot comes back ±0.13 mm in width and place, and until they are
  fitted the tightest plate carries the crank alone.
- **Set the hand crank's clutch** with a luggage scale before it goes on (the guide's hand
  crank step), dry, and check it again yearly or after it has slipped hard.
- **Glue the input shaft's key** into its keyseat (threadlocker or super glue): it falls out
  when the crank comes off otherwise.
- **Oil the steel standoffs:** they are bare 1018. Wipe each with light oil, grease the
  crankpin where the rod turns on it, and keep the cradle dry; the decanting step wipes the
  front saddle, the tie rods and the light after each pour. (Stainless tie rods, round 2's
  m8, are not priced: MISUMI's SETRF10 page refuses a script, HTTP 403 when fetched on
  2026-10-15, and the review's figure, USD 18.52 each at five against NETRF10's 11.08 (+37.20
  for five), came from MISUMI's price API with the grade unconfirmed.)
- **The worm's hold, on the bench:** with a full bottle at level, tap the table and watch a
  pencil line across the crank and the plate for 10 minutes (the guide's hand crank step).
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
     table) and 100 mm free behind the board (the base stop's rails reach past its rear
     edge at full tilt, A 74 mm, B 32; B's hand crank's grip turns past it too).
  2. The decanter on its riser, set once at rest, its rim 20 mm or more under the light's
     clip; once set, nothing raised or added under the cradle box (round 2: "nothing under
     the cradle box at rest" had contradicted the decanter that stands there by design).
  3. Hands out, and nothing under the cradle box or the bottle, while B's crank's arm points
     into the red flag (the last two turns back to rest) and in the last two turns before
     full tilt.
  4. The lip gauge down before the bottle passes level.
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
  round past its bench setting (27 N at B's grip, 40 at A's), so a push can't break the
  drive away from the ends of the swing; within the last two turns before rest or full tilt
  the cradle's force grows without bound however the clutch is set: nothing under the
  cradle box or the bottle there.
- **Decanting** (the guide's step): the decanter on its riser stays where it is while the
  bottle comes back to rest (its rim keeps its 20 mm under the clip); at rest it is taken
  away and the neck, the front saddle, the tie rods and the light wiped. (Round 1's guide
  had said both "take the decanter away before cranking back" and "leave it until the
  bottle is back at rest".) A bigger bottle than its decanter: turn back a turn, slide the
  full one out, the next onto the same riser.
- **Letting go stops the bottle, not the stream:** to stop it, turn back about a turn.
- **The lip gauge down before the bottle passes level:** it falls back by itself, so
  check nothing holds it up (a corkscrew left in the cork); with it up a wet bottle slides
  forward at full tilt, its mouth off the pivot over the decanter.
- **Uncork at rest**, the base stop set: lift the lip gauge onto its stop and hold it, or
  let it rest on the corkscrew, holding the neck, a corkscrew's lever lifted straight up;
  let go and the gauge falls back onto the mouth. Not a
  sparkling wine under pressure: at rest the mouth points 50° up toward the front (the 6 L
  methuselah is a Champagne bottle); ease such a cork out by hand first, the mouth pointed
  away from people.
- **Lift it** by its base board in use; to take it off the board, by its outer plates' top
  members, never by the cradle box or the hand crank.
- **PLA softens at about 55 °C:** a cradle that travels in a hot car wants the hand crank,
  the brackets and the thumb knobs in PETG.
