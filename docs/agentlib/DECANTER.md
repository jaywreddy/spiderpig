# Decanting cradles

This design record covers the hand-cranked decanting cradles in this repo. The user asked
for them on 2026-10-09: a linkage cradle that tips a 1.5 L magnum or a 3 L bottle to any
angle, driven by a hand crank, not a servo, with an LED. Regular 750 ml bottles came on
2026-10-10, and 6 L bottles on 2026-10-13 (the user: "both", a Bordeaux impériale and a
Champagne methuselah); the methuselah is the design bottle since ("Sized for a 6 L").

Two linkages are built, both from the same parts: the drive, the cradle box, the stand
and the light.
- `decanter_hinge` (A), **the one to build**: the cradle swings on a real hinge at the
  bottle's mouth, from −50° (nearly upright: an opened full bottle goes in) to +30° (both
  bottles drain). The redesign of 2026-10-10 (the user's −50° rest) is under "A, redone".
- `decanter_rcm` (B), **to build too**: the hinge is virtual, a double parallelogram (a
  remote centre of motion), so no pin sits at the mouth. It swings from −50° (rest, an
  opened full bottle stands in it) to +30°. Its design is under "B, the virtual hinge".

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
| vessel | a standard decanter or glass on a lab jack (100 x 100 mm) in the base board's jack locator, one height for the whole swing | the vessel study (204–300 mm tall decanters, 155–240 across); the room under the mouth, below; round 1 of 2026-10-14 (no raising it mid-pour) |
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
    - The rear one carries the body: its V block rides a 12 mm steel post at one end and an
      M8 lead screw at the other. It is 125 mm long, so it reaches behind the 6 L's centre
      of mass (331 mm from the mouth) and still under a 750 ml's body: a bottle bears on
      both saddles, lying level or tipped.
    - The Vs are 90°, lined with 2 mm TPU. At a 6 L methuselah's setting each V block sits
      on its tie rods (the rear one in a seat over each); a thinner bottle's ride higher.
  - **The lip gauge** (`cradle._gauge`): a printed arm hinged on the front carrier (an M5
    button head along the axis). Flipped down, its pad stands in front of the mouth, its
    back face on the pivot's plane, over the top of the rim (above a cork's bore). It
    marks where the mouth goes, and it is the forward stop: past level a bottle would
    slide toward its mouth, and the rim's top bears on it. It flips up and out to lay a
    bottle in or to uncork.
  - **The base stop** (`cradle._rails`, `cradle._gate`): a printed gate across the V behind
    the bottle's base, riding the rear V block's flats and two printed rails behind the
    block. It is the back stop: at rest the bottle stands 50° neck up and leans on it (at
    25° nose up the liners' friction alone had held a bottle, and a corkscrew's push of
    6-13 N slid it back off the pivot: the mechanical review).
    - Keys under its ears run in a groove along each flat and rail; an M5 button head
      through a slot in one ear into a row of pilots (8 mm apart) clamps it anywhere, so
      it sets to the bottle in hand. Should it slip, the slot's end stops it within 8.5 mm.
    - It serves every base from 285 mm (inside the rear V: a 750 ml's at 300) to 576 mm
      (a 6 L's at 571) behind the mouth.
    - It rides the rear V block, so it rises with it. At rest, set for a 6 L, the rails'
      end comes down to about 18 mm over the base board (keep it clear there).
- **The bottles** (`cradle.setting`, `cradle.seat`): every one lies with the mouth on the
  pivot and the axis level, so the mouth stays put for each. Its setting is the two V
  blocks' rise and the base stop's place, read off the scales (see "The saddles,
  adjustable"); the lip gauge and the base stop hold it there.
  - The 6 L necks' lengths and radii are estimates scaled from the jeroboam's (no maker
    draws them): set the front knob to the neck in hand.
  - Another bottle: set each knob to its neck's and its body's diameter on the scales'
    other rows, and the base stop to its base with the gauge down.
- **The drive** (`spiderpig/construction/handcrank.py`, `config.crank == "hand"`; round 1's
  drive review of 2026-10-14 reworked its riser, its block, its tie and its hand crank).
  - **Gearbox:** an NRV040 worm gear reducer at 30:1 (Phidgets TRM4401_0, rated 45 N·m;
    `handcrank.NRV040`). It stands outboard on the drive side's outer plate, held by four M6
    button heads into its tapped face holes (75 mm PCD). A full 6 L peaks at about 30 N·m
    at the crank: the NRV030 (20 N·m) that held a 3 L couldn't.
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
      bench with a luggage scale to click round at 31.5 N at the grip
      (`handcrank.CLUTCH_SLIP_N`, ±10 % once set): 1.1 times a sealed full 6 L's worst
      starting turn at its least. Through the worm at a hand's speed that is at most 57.2
      N·m on B's crank and 60.0 on A's (`config.PRODUCT_CLUTCH_NM`), however hard anyone
      pushes.
    - Its 60 x 24 mm printed grip spins on the plain shank of an M8 x 85 hex bolt through
      the arm's end, a nyloc nut on the arm's inner face (a NETRF10 standoff and two button
      heads until round 1: a 25 mm knob was a two-finger pinch).
    - The arm is 100 mm on B and 105 on A (`handcrank.HANDLE_RS`), behind every moving part
      of the stack. Turn it from the drive side's rear corner, the neck in view.
  - **No guard over the crank** (the user's call, 2026-10-10: the guard, a disc in the
    outer plate and then the gearbox's flange, read as a big disc either way). The gearbox
    sits on its spacers; the crank turns about 2 rpm behind the outer plate's truss. On B
    its top-rear arc and the rod's end turn in the open, above and behind the gearbox,
    right in front of the hand crank (round 1): the use limits name them with the other
    pinch points (`cradle.USE_LIMITS`), and taking the handle off locks it.
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
    lifts straight up off the cradle link, its pins with it ("Flat pack"). The clips go back
    in the pins' holes. Without its clip a pin is held only by friction: the steps back
    together check both are in. (With the passive side down, a pin whose clip is out drops
    out of its link: its head is under the link, nothing under it.)
  - Rated as a solid 8 mm rod at annealed 304's minimum yield (205 MPa): 1.39 x the M6
    Chicago barrel it replaced. Jammed, X 2.52 and Y 2.21 (as Chicago screws they had
    held 1.72 and 1.49).
  - K's and J2's heads stand under b2 and b3, their clips in the gap over b4 and b5, their
    ends in the cradle link's layer, where b6 never comes near them.
  - B's stack is 84.23 mm a side (79.68 until 2026-10-14: the clevis pin at K moves b2
    and b3 a layer up, and b6 is 0.187 in).
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
  the bay (`cradle.BASE_HALF`), 682 mm long on A and 680 on B (`Stand.base`: a full 6 L
  tips past 25° over every edge of B's; A's rear edge 24.4° since the corrected centre of
  mass, its bar 15°). B's sits 40 mm toward its drive side (`cradle.BASE_Z`,
  round 1 of 2026-10-14: the gearbox and the hand crank put the cradle's centre of mass 68
  mm that way). Its corners are rounded 35 mm and the notch's mouth 8 mm
  (`cradle.BASE_CORNER_R`, `NOTCH_MOUTH_R`): a square plywood corner chips and catches a
  shin. SendCutSend publishes no plywood price: its cutting is quoted at upload. Seal its
  edges with a clear finish (the notch's end grain would wick wine) and oil its faces; it
  isn't weather-proof.
  - **The cleats** (`cradle._cleats`, the parts `base_cleat_*`; round 1): two 26 mm strips
    of the same birch glued to the board either side of each outer plate's rail, between
    its brackets, 0.15 mm off each face (`cradle.CLEAT`). They hold each side upright on the
    board, where its two printed brackets alone let it sway several millimetres under a
    20 N push at the mouth, and they drop the rails into place when the cradle goes back
    on. They're glued to the cradle as built: the guide pencils each round its rail with a
    sheet of paper for the play, then glues it on the marks (Titebond II, the `wood_glue`
    line).
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
      knobs last, after a whole pour.
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
    - All the bolts go in with the board. The passive side is set down 7-15 mm out from its
      place, inside its bolts, and pushed home.
  - **The jack locator** (`cradle._jack_locator`, the parts `jack_locator_0` and
    `jack_locator_1`; round 1): two layers of the board's birch glued face to face. At 18 mm it meets the board's
    whole edge (the board stands 8 mm up on its feet). Its outline is the notch's, 1 mm in,
    from 70 mm in front of the mouth to the notch's round end, with an ear into each of
    the notch's two side bays (`cradle.NOTCH_BAY`), so it can't slide out forward. Its
    101 mm square pocket, dog-boned, takes a 100 x 100 mm lab jack's base under the mouth
    (`cradle.JACK_BASE`): in the bare notch the jack had floated 76 mm either way, and the
    stream lands within 16 mm of the axis. It lifts out to stand a vessel on the table.
  - Four printed TPU feet snap into its corner holes (a cone and a bore up the stem). On a
    counter that isn't flat they take up about 1 mm; past that it rocks, and a thicker foot
    goes under the corner that lifts.
- **The light:** a 14 x 60.5 mm AAA keychain light (Olight i3E EOS) stands in a printed
  clip under the front saddle's back end (its flange on the carrier's underside, two M5s),
  so the clip prints upright and round. Its head sits under the
  shoulder; an M3 set screw holds it while you twist the head to switch it on.
  - A longer light's tail reaches a decanter's shoulder at rest.
- **Display only** (`fab="display"`: drawn, never bought, cut or printed): the bottle (a
  6 L methuselah, `cradle.SHOWN`: the V blocks drawn at their lowest; the cradle page sets
  them for the bottle picked), a RIEDEL Ultra decanter (225 mm, `cradle.DECANTER_PROFILE`) and
  its riser (`cradle.DECANTER_RISER`, 135 mm: the highest that clears the whole swing), in
  the jack locator's pocket.

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
  adapter liners: DECISIONS.md.)
- **The jam grows with the handle.** A firm 40 N push on a 100 mm handle through the worm
  at a hand's speed is 66 N·m at the crank (A 69.3: `config.PRODUCT_JAM_NM`), where it had
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

     gauge           knob                          post          knob (M8 screw)
      ____          (M6)                            ||    6 L      _|_
     |    \__________|_                             ||  .-'''-.     |
     |  cheek |  .-.  |#|                         __||_/       \_ __|__
     |  post  | ( O ) |#| <- tab + M6 nut         |  ||\  axis  /  || | <- coupling nut
     |   |\___|  \ /  |#|                         |  || \_____/   || |
     |   |  V  block  |#|                         |  ||\  V   /   || |
     |___|____________|_|                         |__||_\_block/__||_|
   ===o========o===  tie rods                     ===o=====o========o=== tie rods (side blocks)
```

- **Front:** the V block slides in a box (the carrier's walls, the front and back ones
  notched under the lowest V). Its tab at the +z end holds an M6 nut. The M6 x 60 tap bolt
  stands in a blind bore in the carrier's floor, through the nut, its printed knob on its
  head over the wall. The knob stays put: turning it walks the nut, and the block, up or
  down. The block bears down on the nut, so the thread (self-locking) holds it.
- **Rear:** the V block's two end bosses ride a NETRF12 post (12 mm, at -z) and the M8 x
  150 tap bolt (+z), whose coupling nut sits in the boss. The post and the screw stand in
  the carrier's two side blocks on the tie rods, joined by a bridge under the block. Its
  knob is at the screw's head, beside the bottle, reached from above.
  - The screw holds the block at one end. A bottle's weight at the axis rolls the block
    about it, and the post takes that couple (it is 5.7 x stiffer than the screw's core).
    That is why the post is 12 mm, not 10.
  - Gussets under the flats spread the couple into the block.
- **The scales:** grooves the V block's top edge (front) or its screw boss's top (rear)
  reads. One row has a groove per bottle of the five. The other is graduated in diameter:
  the neck's every 4 mm in front, the body's every 10 mm at the rear. No text is printed
  on them: glyphs would depend on the fonts of the machine that builds the STL.
- **No depth added:** each carrier's bottom is the old saddle's (front -60, rear -149).
  The V block sits on the rods at its lowest, and its guides stand beside the bottle (|z|
  97 to 115 at the rear, inside the bay's 130). `decanters.TIES` didn't move: neither
  cradle re-plans.

**Why not the tie rods in vertical slots** (the first idea): the slot would have to run
the whole travel under the rod, so the rear block would hang 69 mm deeper at a 6 L's
setting. That is onto the decanter's axis at rest, 217 mm over the table. The front block
would hang 8 mm deeper. The rods can't go higher: they already sit just under the 6 L's V
apex.

**Why not detents:** a 5 mm pitch leaves a front V up to 2.5 mm off. Over the 252 mm to
the rear V, that puts the mouth 0.8 mm off the pivot, and it wanders 1 mm over the swing.
The thumbscrew is exact and adds no depth.

**Why the gauge is the forward stop:** with no collar, a stop must hold the bottle from
sliding toward its mouth past level. A lip over the base's heel can't: the heel tapers
toward the base, and nothing on the bottle faces the mouth but the shoulder and the
mouth's own face. A shoulder stop would need a third adjustment, 45 mm along the axis
(and the light stands there). The gauge is at the pivot by construction, for any bottle.
Its pad bears on the top of the rim, over the bore; the stream leaves the bottom of the
bore, and the decanter is below. The bottle's push runs along the hinge's axis, so it
can't flip the gauge open.

**Why the base stop rides the V block:** a 750 ml's base (300 mm) and a magnum's (356) lie
inside the rear V (255-380), where the old spine couldn't reach. A gate across the V,
keyed to the block's flats and continued by rails behind it, serves them and the 6 L's
(571) alike. It drops the 155 g spine, and the lowest point at rest doesn't move (the
rails' end, 18.8 mm over the board, against the spine's 18.1).

**Use** (the guide's "Setting it for a bottle"):
1. At rest, the bottle out, the gauge up: turn each knob to the bottle's groove.
2. Lay the bottle in, its label up. Flip the gauge down; slide the bottle up to it.
3. Slide the gate up to the base and lock it.
4. The gauge stays down while it pours. Flip it up to uncork or to take the bottle out.

**The parts** (prices from pages rendered 2026-10-13, qty 1, per cradle; the sourcing in
`/root/scratch/decant/saddles/sourcing/` on the development box):

| part | where | USD |
|---|---|---|
| M8 x 150 tap bolt, fully threaded, 18-8 | Bolt Depot 11110 | 2.87 |
| M8 x 24 coupling nut, 18-8 (DIN 6334) | Bolt Depot 27224 | 2.52 |
| M6 x 60 tap bolt, fully threaded, 18-8 | Bolt Depot 5973 | 0.60 |
| M6 hex nut, 18-8 | Bolt Depot 4776 | 0.11 |
| NETRF12-130 standoff (the rear post) | MISUMI | 16.98 |
| M6 x 12 button head (the post's foot) | Bolt Depot 6644 | 0.19 |
| M5 x 16 button heads: the gauge's pin, the rails' two, the gate's lock | Bolt Depot 6634 | 4 x 0.14 |

That is about USD 23 more per cradle than the old saddles, the post most of it (a NETRF10
would be 14.08, at yield SF 2.3 under the couple). Everything else is printed: the two
carriers, the two V blocks and their TPU liners, two knobs, the gauge, the rails and the
gate.

**What holds** (`cradle_report.saddles`; a full 6 L, 11.1 kg, its worst way for each part;
printed parts against 10 MPa, a quarter of printed PLA's strength across its layers, for a
load that stays):

| part | load | stress |
|---|---|---|
| rear V block on its rods' seats | 109 N | 0.28 MPa |
| rear coupling nut on the block | 116 N (hung) | 1.3 MPa |
| rear post, bending | 11 N·m couple | 65 MPa (SF 3.4 on 1018's 220) |
| rear screw, bending | its share | 35 MPa (SF 6.0) |
| rear V block at the post's boss | the post's share | 1.2 MPa |
| base stop's gate, bending | 96 N at rest (with a corkscrew's push) | 5.4 MPa |
| base stop's lock | 96 N | its grip 360 N (SF 3.7) |
| lip gauge's pad, bar, arm | 54 N at full tilt (no friction) | 3.8, 3.0, 4.3 MPa |

The V blocks carry no jam load: the tie rods and the cradle links do.

## The vessel under the mouth

At rest, 50° neck up, the cradle box hangs straight under the mouth: the front tie rods, the
front saddle and the light cross the decanter's axis from 385 mm over the table up.
`cradle_report.room` measures the room a vessel has round its axis at every height, over
the whole swing and from the pour on (the cradle past −20°), with the drawn 6 L methuselah
in the saddles (the V blocks at their lowest: a thinner bottle's ride higher, off the
decanter); the cradle page (`/api/cradle/{key}/room`) stands each vessel on the highest
riser that clears it (`viewer/src/decant/liquid.ts`, `bestRiser`).
- For the whole swing: 250 mm across up to 165 mm over the table, 125 mm across above
  that, its rim no higher than 380 mm. A RIEDEL Ultra stands on a 135 mm riser, its rim
  about 190 mm under the mouth.
- **One height for the whole swing** (round 1, 2026-10-14). From the pour on the room is
  larger: the rim up to about 515 mm over the table (55 mm across above 475), 35 mm under
  the mouth (`cradle_report.room`'s `room_pouring`). Using it meant raising the jack once
  the cradle was past −20° and lowering it before cranking back to rest. Forgotten, the
  worm drives the cradle box down onto the glass while the handle only feels a little
  stiff: near −30° each newton at the grip puts 20-25 N on the rim. And −20° can't be seen
  on the cradle; a RIEDEL Ultra gained 12 mm. The guide, the use limits and the cradle page
  (`viewer/src/decant/pour.ts`) give one height, set at rest, and say never to raise it
  mid-pour.

## Flat pack

`spiderpig/cradle_pack.py` (`flat_pack`) takes the cradle apart for the trip and stacks it
in one box. The cradle page's Flat pack button flies each piece into the box, in the order
they go in (`/api/cradle/{key}/pack`). B packs as three modules, A flat.

**B: three modules and the hand crank** (`cradle_pack.module_pieces`, the user's of
2026-10-13; the steps rewritten in round 1 of 2026-10-14: `cradle.module_steps`, also the
guide's last two steps). No tool, no small part left loose: 696 x 608 x 499 mm outside, about
211 L (the real-shape packer's plan; 202 L before round 1's structure: the box's inner
plate and columns), where its flat pack was 112.
1. Crank it to the pack pose: from rest, turn the handle the way that swings the crank's arm
   down, until the arm lies along the drive side's outer plate's rear leg, straight down
   from the gearbox (`cradle.PACK_POSES`, `pack_pose`; `cradle_pack.pack_t`). Seen from
   outside the leg hides it. It is about 5½ turns from rest and the bottle is 31° neck up.
   - At rest the passive side's cradle link b6 overlaps that side's inner bar O1-O2 (its
     front ties' boss over the bar's O1 end, by about 2 mm) until 46° neck up. Past that it
     clears it: about 4 mm at 44°, 15 mm at 35°, 20 mm at 30°.
   - A mark, not a count: rest is a toggle dwell (the bottle moves under 2° over the last
     1½ turns), and the old "4 turns on" gave 35° one way and 39° the other, or 44-47°
     for a rest called early, where b6's boss jams on the bar (round 1's stand review).
   - From 44° to 20° neck up the box doesn't grow: the board sets its length. At full tilt
     the cradle box stands up out of the side (a 259 L box).
   - The first time, mark the pose for the way back with a paint pen: a line across the arm
     b3 and the inner bar at its O1 end, and one across b5 and b3 by J2.
2. Hand crank: loosen its thumb screw and pull it off. Nothing turns without it. (Its
   thumb screws stay in their brass inserts and the input key is glued in: nothing small
   comes loose. A stow clip on the drive side's outer face was tried in round 1's drive
   review: B's plan lays that face on the passive panel, so the clipped crank lifted the
   drive side 54 mm, over the box's bound.)
3. Undo the four thumb knobs on the base board's bolts by hand and lift the cradle off the
   board by its two outer plates' top members (never by the cradle box: only the worm holds
   it). The bolts stay in the board, the washers and knobs back on them.
4. Lay it on its drive side on a folded towel, the gearbox down: it rests a little tilted
   on the gearbox and that side's brackets, the passive side up. Gravity now runs along the
   pins, so no link can swing.
5. Pull the two R-clips of pins X and Y on the passive side, reaching into the bay, from
   under the passive cradle link ("The quick-release pins"). Each pin stays in its link,
   standing on its head.
6. Lift the passive side panel straight up off the pins by its outer plate, level, its pins
   with it. The printed spacer that sat on each pin over the cradle link stays on the link:
   slip it back onto its pin's end and push the clip through the pin's hole. The drive side
   with the cradle box stays lying as it is, as it goes in the box, the worm holding it.
   - Standing, as the steps had it before, the 6.8 kg drive module tipped over once the
     pins cleared b6 (its centre of mass 66 mm past the one rail it stood on), dropping its
     cradle box onto the table, and the panel fell the other way.
   - Not on the passive side, as round 1's review proposed: each pin's head lies on its
     link's outer face (`construction.pivots.clevis`), 50 mm and more over that side's outer
     plate with nothing under it in plan, so with that side down and its clip out the pin
     drops out of its link; the spacer between the links is left loose too.
   - `cradle_pack.slide_hits` sweeps the panel's parts along z in 0.5 mm steps against the
     drive side and the cradle box: nothing meets (the payload's `slide_off`).

**Back together** (the payload's `reassembly`, the guide's "Putting it back together"):
1. The drive module on its drive side on a folded towel, its passive cradle link up.
2. The passive panel on its outer plate, its pins held by their clips; its links set to the
   pack pose by the paint marks, a strip of masking tape across each pair while it turns
   over.
3. Turned over and held level: each clip out, each spacer off its pin's end and over its
   hole in the cradle link.
4. Lowered level onto the cradle link: pin X through its spacer into its hole first, then
   b5 nudged until Y drops into its. The tape off.
5. Both clips in from under the cradle link: a pin without its clip is held by nothing.
6. Stood up and set on the board: each rail between its cleats, the brackets' feet over
   their bolts; the washers and knobs on, the passive side's only started.
7. The hand crank on; a whole pour and back with the passive knobs loose, then snug them at
   rest: B's two remote centres settle together.
8. The first build's checks again, the bottle out: both cradle links swing freely, the
   handle turns evenly with a fingertip (under 5 N at the grip: `cradle.EMPTY_GRIP_N`; the
   report's `empty_handle_force_n` is 3.6), and the rod's paper check.

The pieces: the base board (its feet, cleats and jack locator, its bolts, washers and
knobs), the passive side panel (every part of that side but its cradle link, its brackets),
the drive side with the cradle box (both cradle links, the ties, the saddles with their V
blocks, knobs, post and lip gauge, the light, the base stop: the worm holds it as cranked)
and the hand crank. The passive side's b5, hanging on J2, swings into its panel
(`cradle_pack.tuck`). B's plan (`cradle_pack.PACK_PLANS`) packs them as they come apart:
the passive panel in first, flat on the board at its +x end, its outer face up; the drive
side over it as it lies once the panel is off, on its drive side, the gearbox down, its
plates over the panel's, the cradle box up, 441 mm across the bay; the hand crank on the
board under the drive side's plates. The drive side rests on the panel, no pads; the panel
on the board on foam pads (24 mm at most); the mass centre 29 % up the box. (Since round
1's structure the panel's inner plate has a triangle and a leg on two columns where the
drive side's gearbox had dropped past it: the panel lies turned the other way, at the
board's end.)

The steps lift the cradle off the board before the panel comes off: its brackets' feet
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
B's box is its board's footprint times the drive side's 441 mm across the bay; A's is the
board's times the drive panel's pillar G and stand columns, the hand crank and the rear
saddle (148-153 mm). Before round 1's pack pose B's drive side packed cradle box down, on 7
pads to 60 mm, its mass centre 55 % up: on its gearbox it tipped there.
What-ifs (`--split`, on the design before round 1): B's cradle box off the drive side,
754 x 316 x 602 mm, 143.5 L (searched; the cradle box on 4 pads over the two panels); B's
gearbox off the drive side, no gain (its brackets and crank train stand as far out); A's
stand columns off, no gain.

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

The jam case is a firm push at the crank's grip on a blocked cradle
(`cradle_report.JAM_HAND_N`, 40 N: about 1.7 x what the worst turn takes), through the
worm at a hand's speed (`WormBox.eta_hand`, 0.55 on the NRV040): 66 N·m at B's crank on
its 100 mm handle, 69.3 at A's on its 105 (`cradle_report.jam_nm`,
`config.PRODUCT_JAM_NM`, per linkage).
- Rated at the gearbox's rating instead, it would be reached at less than the worst turn's
  own force: no bound (all three reviews of 2026-10-10).
- It is not counted within 30° of crank round either toggle, where a blocked crank-rocker's
  forces grow without bound. There, only clearance protects: the decanter's lip and
  anything under the cradle.
- The crank's torque is a parameter of the structure's check (`cradle_report.structure`'s
  `crank_nm` and `skip`): a clutch's cap, if one comes, rates the toggle zones too.
- What A holds there: pin E (steel) 2.88, the crankpin 1.6, the keyed plates 1.9, the rod's
  buckling 2.05, pillar G 1.63 (on its box: below), its cradle link b2 1.61 (in bending,
  chord by chord; 0.187 in 5052, its E-Tr1 32 mm wide against buckling), the columns
  2.17 and more, the outer plate's members 2.01, the inner's 2.97.
- Turning it: 36.0 N·m at A's crank at worst (the NRV040's SF 1.25), 25.4 N at the grip:
  both over their bars since the corrected centre of mass (31.3 N·m, 1.44 and 22.1 N
  before), open for the drive (DECISIONS.md, 2026-10-14). It tips at 24.4° over its rear
  edge (25.3° before; its test's bar is 15°), 25.7° to its drive side, 30.8° to the other
  and 38.6° forward, and takes a 34.9 N push at a bottle's top (an empty 750 ml, toward
  the drive side), the worst over the swing; 10 mm more board at the rear would bring it
  past 25° (25.3°).
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
mouth its lever is short. The same 66 N·m (A 69.3) against a block:

| block at | B: the block's force | B: J / O1 / X / K | B: weakest | A: weakest |
|---|---|---|---|---|
| the front tie Tf1 (the front saddle, the light, the front ties on a raised decanter's rim), 72 mm from the mouth | 3.65 kN | 6.6 / 6.2 / 3.7 / 3.6 kN | the outer plate 0.17, pillars O1 0.30 and O2 0.34, pin K 0.66 | the outer plate 0.21, pillar G 0.72 (G 4.4 kN) |
| the rear saddle's underside, 351 mm out | 0.75 kN | 1.6 / 2.1 / 1.0 / 2.0 kN | the outer plate 0.73, pillar O2 0.86, O1 1.01 | the outer plate 0.84 |

Not designed for: nothing that holds a block 72 mm from the mouth at a firm push fits the
cradle. In practice the glass decanter breaks first. The use limits keep the decanter out
of the cradle box's path (raise its jack only past −20°, lower it before cranking back),
and the cure is a torque limiter at the hand (a slip hub just over the worst turn's
force), the drive's next round.

**The clutch's cap** (round 1's drive review, 2026-10-14: `config.PRODUCT_CLUTCH_NM`,
`cradle_drive.clutch_case`, the report's `jam_capped`). The hand crank's slip clutch clicks
round at 31.5 N at the grip, at most 10 % more once set, so the crank never sees more than
57.2 N·m on B and 60.0 on A, through the worm at a hand's speed, however hard the push. That
is a bound; the 40 N push above isn't one (a hand on a crank that feels stuck pushes 60-150
N). The report rates the cap over the whole turn, the toggles included:
- The report's own model rates it (`cradle_report.structure` with no toggle skipped, the
  links chord by chord, the posts on their box, the plates' members): 30° of crank and more
  from either toggle every part holds it, B at SF 1.46 or more (pillars O1 and O2; 1.31 at
  the 40 N push), A at 1.84 (b2; 1.61).
- Nearer a toggle the rod's force goes as the cap over the sine of the crank's angle to the
  rod, and no torque bounds it. B: pillar O2 1.23 at ±25°, 1.00 at ±20°, 0.49 at ±10°; the
  rod buckles (elastically, into its 3 mm gaps) within ±16° of rest and ±9° of full tilt. A:
  every part holds 1.3 outside ±25° (the last 3.8° of tilt before rest, 5.9° before full);
  b2 0.87 at ±15°; the rod buckles within ±11° and ±6°. Only clearance protects there: the
  use limits keep everything from under the cradle box and the bottle in the last two or
  three turns.
- Blocked at a point (`cradle_report.blocked` at the cap, the report's `blocked_capped`): a
  block at the front tie still takes B's outer plate to 0.19 and its pillars to 0.34-0.39
  (0.17 and 0.30-0.34 at the 40 N push), A's outer plate to 0.23. No clutch a hand can turn
  brings those to 1: a known limit, kept out by the use limits.
- The gearbox at the cap: 1.27 x its 45 N·m rating on B, 1.33 on A, a momentary peak (the
  40 N push had been 1.47 and 1.54 x, unreported until round 1).
- The keyed plates hold 2.3 x the cap filed together on the key; one plate bearing alone
  0.58, and 0.96 at B's walking peak, so the guide files them together.
- The bars are the tests' (`tests/test_cradle.py`): the clutch's least slip at least 1.1 x
  the worst starting turn (1.12 on both), every part at 1.3 or more 30° from the toggles.

Every joint on both sides is rated at the full planar load (`cradle_report.SHARE` = 1),
not a share of it. The rod drives one side, whose pivots carry half the weight plus the
rod's whole force.

`spiderpig/cradle_report.py` (`report(config)`) measures:
- **Swing:** the swing and each bottle's own mouth wander.
- **Drive:** the crank's peak torque, the gearbox's safety factor, the force at the crank's
  grip, the jam's push and torque, the turns per pour; the clutch's slip against the worst
  starting turn and its cap, the gearbox and the keyed plates at both jams
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
  gauge's and the base stop's (`cradle_report.saddles`: "The saddles, adjustable").
- **Clashes:** the whole cradle's clashes over the swing, the hand crank's swept volume
  against everything, and every bottle at its own setting, its V blocks and base stop
  moved (`cradle.bottle_hits`, `cradle.at_setting`).

`tests/test_cradle.py` holds the thresholds, for A:
- each bottle's mouth wander under 0.5 mm;
- tipping at 15° or more, and 25 N or more at the bottle's top, the worst over the swing;
- the crank's grip force under 26 N (25 until the corrected centre of mass: open for the
  drive);
- the gearbox's safety factor at least 1.2, against its 45 N·m rating (the peak is a
  sealed full 6 L at one tilt, at hand speed; 1.3 until the corrected centre of mass:
  open for the drive);
- every pin, pillar, column, link and frame plate at least 3 walking and 1.3 jammed (the
  links, posts and plates from the force vectors); the keyed plates at least 2.5 walking
  and 1.5 jammed; the crankpin and the rod 1.5 jammed;
- the saddles' printed parts under 10 MPa at a full 6 L's worst, the post and the
  thumbscrew 3 x under their yield, the base stop's lock gripping 3 x its load;
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
390 mm from the mouth, a 40 N push at the grip jammed; 2026-10-14, after the design
review's structure round):

| check | B |
|---|---|
| swing | −50.0° to +30.0°, the mouth still (0 mm), every bottle's mouth on the pivot |
| transmission | 36.7° at worst (the rod's); the parallelograms 48.5° and 51.5° |
| gearbox | 34.3 N·m peak: SF 1.31 on the NRV040's 45 N·m; 25.4 N at the 100 mm grip (over its 25 N bar since the corrected centre of mass: open for the drive) |
| jam (66 N·m) | pillars O1 and O2 1.31 (on their box), b6 1.33, pin K 1.43 (clevis), b4 1.44 (its chord's buckling), the outer plate's members 1.5, the inner's 1.64, b1 1.77, b3 1.87, the crank's keyway 2.0, pin J2 2.16 (clevis), column D1 2.19, pin Y 2.21 and pin X 2.52 (clevis), pin P 2.66 (steel), b2 3.05, pin J 10.6 (steel) |
| walking | every joint, link, column and plate 4.25 or more; the crank's keyway 3.85 |
| rod, crankpin | buckling 1.59 × its jam force; the crankpin 1.96 jammed |
| the clutch's cap (57.2 N·m) | every part 1.46 or more 30° and more from the toggles (pillars O1 and O2); pillar O2 1.23 at ±25°, 1.00 at ±20°; the rod buckles within ±16° of rest, ±9° of full tilt (round 1's drive review) |
| stand | tips at 25.2° at worst (a full 6 L, over the rear edge); the drive side 29.7°, the passive side 28.9°, the front 40.1°; a push of 44.1 N or more at a bottle's top (an empty 750 ml, toward the drive side). The board 40 mm toward the drive side and 680 mm long, round 1: it had been 25.9° (26.0° to the drive side, 31.7° to the other) and 32.0 N |
| stack | 84.23 mm a side (79.68 before: the clevis pin at K, b6 in 0.187 in) |

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
- **The base board** runs from 160 mm in front of the mouth to 520 mm behind it: 680 by
  584 mm (a full 6 L tips past 25° over its sides and its rear edge; it was 596 by 500 for
  a 3 L, 626 by 530 until the stand's check of 2026-10-14, 684 until the corrected centre
  of mass and the structure round the same day). The first refit's had run 827
  mm, to 640 mm behind the mouth, with the crank 480 mm back. Its middle is 40 mm toward
  the drive side (`cradle.BASE_Z`, round 1). Its front keeps its 160 mm: the reviewer's
  35 mm trim (the front tips at 39.5°) would put the passive front foot's pad on that
  side's carriage bolt's head, now the board's passive edge is 40 mm nearer the bolts.
  - The rear edge binds, at 25.2° (bar 25°): with the centre of mass corrected (390 mm
    from the mouth) and round 1's structure, 680 mm is the least board that holds it
    (674 mm: 24.6°; 684: 25.6°). Each 10 mm of board at the rear is about 0.5°; `python
    -m spiderpig.cradle_report stand` re-runs the stand alone.
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
spiderpig build --linkage decanter_hinge --out build/cradle_a   # STEP, STL, DXF, BOM, ORDER.md
spiderpig guide --linkage decanter_hinge --out build/cradle_a   # ASSEMBLY.pdf and .md
spiderpig bake --linkage decanter_hinge                          # the viewer's .glb (8 s loop)
```

The assembly guide's order is `construction.assembly.CRADLE_ORDER`:
1. **The drive side**, bottom up on its outer plate:
   - the gearbox under the plate first;
   - the crank block from its lowest plate, the two standoffs screwed to it on the bench;
   - its cradle link goes on with the four tie rods already screwed to it;
   - its inner plate.
2. **The base board:** sealed, its feet, bolts and jack locator in; the drive side bolts onto
   it (it can't stand alone), its rail's two cleats pencilled round the rail with a sheet
   of paper for the play, glued on the marks with the side lifted off, the side set back
   between them.
3. **The cradle box:** the saddles slide onto the tie rods, each V block in its carrier
   on its thumbscrew (the lip gauge on the front one), then the light and the base stop go
   on.
4. **The passive side:**
   - its cradle link onto the tie rods' free ends;
   - its stack, from outside: the pillar goes up through the cradle link;
   - what goes over the link, then its inner plate, then its brackets and cleats (as the
     drive side's), its knobs snugged last, after a whole pour.
5. **The hand crank**, then **the bottle:** set the cradle for it (the two knobs, the lip
   gauge, the base stop), then decanting; on B, taking it apart for the trip and putting
   it back together ("Flat pack").

Things to know before cutting and on the bench:
- **Get a live SendCutSend quote** (upload the DXFs). The BOM's cutting figure is a fit to
  the live quotes of 2026-10-10 for A's files (`hardware.bom.CRADLE_CUT_FITS`: USD 367.50
  then); the walkers' per-area rates had priced the old aluminium base plate alone at USD
  488. The plywood (the board, its cleats and the jack locator) and the 7075 have no fitted
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
  crankpin where the rod turns on it, and keep the cradle dry.
- **The worm's hold, on the bench:** with a full bottle at level, tap the table and watch a
  pencil line across the crank and the plate for 10 minutes (the guide's hand crank step).
  The clone's mesh isn't Motovario's: a 16 % drop in its friction loses self-locking. One
  that creeps settles back neck up; an O-ring's drag between the hub and the input seal
  doubles the margin.
- **Dry-fit the hinge:** the cradle link must swing under its own weight once the
  pillar's screws are tight.
- **Break the laser-cut edges** where hands go: the bay and the frame tops.
- **Seal the base board's edges** (the notch's end grain wicks wine) and the jack
  locator's: a clear finish such as polyurethane; oil the faces.
- **The room it needs** (`cradle.USE_LIMITS`, the guide's "Decanting" step):
  - 950 mm of headroom: a 6 L's base at full tilt reaches about 910 mm over the table.
  - 100 mm behind the base board: the base stop's rails reach past its rear edge at full
    tilt (A 84 mm, B 42), and B's hand crank's 60 mm grip turns up to 87 mm past it.
  - The vessel: as "The vessel under the mouth" above, one height for the whole swing. It
    stands under the mouth's centre (`cradle.DECANTER_X`) on a 100 x 100 mm lab jack in
    the jack locator's pocket.
- **Where it pinches** (the use limits, round 1): both side stacks, their bay faces too
  (each return to rest the cradle links shear past the inner bars' front ends); on B the
  crank and the rod's end, turning in the open above and behind the gearbox; and the base
  board behind the decanter, where at rest the base stop's rails come down to 18 mm over
  it, under the 25 mm a finger needs (ISO 13854). A trapped finger: turn the handle back
  the way it came (the worm holds what it has caught). Take the handle off when the
  cradle is left: nothing turns without it.
- **Never force the crank.** If it stiffens, something is in the way. The clutch clicks
  round past about 31 N at the grip, so a push can't break the drive away from the ends of
  the swing; within the last two or three handle turns before rest or full tilt the
  cradle's force grows without bound however the clutch is set: nothing under the cradle
  box or the bottle there.
- **Letting go stops the bottle, not the stream:** to stop it, turn back about a turn.
- **The lip gauge down before the bottle passes level:** with it up a wet bottle slides
  forward at full tilt, its mouth off the pivot over the decanter.
- **Uncork at rest**, the base stop set and the lip gauge up, holding the neck, a
  corkscrew's lever lifted straight up; then flip the gauge down onto the mouth. Not a
  sparkling wine under pressure: at rest the mouth points 50° up toward the front (the 6 L
  methuselah is a Champagne bottle); ease such a cork out by hand first, the mouth pointed
  away from people.
- **Lift it** by its base board in use; to take it off the board, by its outer plates' top
  members, never by the cradle box or the hand crank.
- **PLA softens at about 55 °C:** a cradle that travels in a hot car wants the hand crank,
  the brackets and the thumb knobs in PETG.
