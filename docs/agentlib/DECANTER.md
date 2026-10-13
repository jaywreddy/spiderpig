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
| the mouth's height | 550 mm over the table | at −50° the 6 L's base stop hangs about 520 mm under it |
| the bay | 260 mm between the inner plates | a 174 mm bottle in 190 mm saddles; a standard decanter (up to 240 mm across) stands wholly between the side stacks |
| vessel | a standard decanter or glass on a lab-jack riser | the vessel study (204–300 mm tall decanters, 155–240 across); the room under the mouth, below |
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
  - Two printed saddles ride on the tie rods. The front one carries the neck, and its back
    face is the shoulder stop. The rear one carries the body. Each is locked on its rods
    by M3 set screws up from its underside, in printed pilots.
  - Their Vs are sized for the 6 L methuselah, 190 mm across (`cradle.SADDLE_HALF`). TPU
    liners line them. The rear saddle is 125 mm long, so it reaches behind the 6 L's
    centre of mass (331 mm from the mouth) and still under a 750 ml's body: a bottle bears
    on both saddles, lying level or tipped. It is a V shell 22 mm thick
    (`cradle.SHELL`): the block's outer corners carried nothing and weighed a third of it.
  - **The base stop** (`cradle._base_stop`): a printed spine screwed to the rear saddle's
    back face runs under the bottle, and a printed cup on it stops the bottle's base. An M5
    button head through the cup into a pilot of the spine's row sets it, in 5 mm steps.
    - At rest the bottle stands 50° neck up and leans on it: the stop carries the bottle
      then (at 25° nose up the liners' friction alone had held it, and a corkscrew's push
      of 6-13 N slid it back off the pivot: the mechanical review).
    - It is a stop at rest, not a backstop: the spine is printed. At rest its rounded end
      comes down to about 18 mm over the base board (keep it clear there).
    - The spine runs to the 6 L's base (`cradle.SPINE_END`), 201 mm past the rear saddle;
      its row of pilots serves every base behind that saddle (the 3 L and both 6 L).
- **The bottles** (`cradle.adapter`, `cradle.seat`): every one lies with the mouth on the
  pivot and the axis level, so the mouth stays put for each.
  - A 6 L methuselah (`cradle.METHUSELAH`, the design bottle) lies in the saddles' liners
    alone, a printed TPU C-collar 43 mm long round its neck against the stop. A 6 L
    impériale and a 3 L lie on their own adapter liners, with collars too.
  - A magnum lies on two printed TPU adapter liners: 8.5 mm in front, 37.5 mm at the rear,
    the rear one closed behind its base (it ends inside the rear saddle). Its shoulder
    rests on the stop.
  - A 750 ml (`cradle.STANDARD`, a Bordeaux, 300 x 76.6 mm) lies on its own two: 9 mm in
    front, ending at its shoulder (its neck is 5 mm shorter than the stop's distance, so the
    liner's end is its stop), and 48.7 mm at the rear, closed 1 mm behind its base (it ends
    inside the rear saddle, where the base stop's cup can't reach).
  - The 6 L necks' lengths and radii are estimates scaled from the jeroboam's (no maker
    draws them): print the collars and front liners to the bottle in hand.
  - The model draws the magnum on its adapters; the other sizes' collars stand in a row on
    the table in front, their liners lie in two rows before them.
  - Another bottle: print adapters to its own neck and body (`adapter`).
- **The drive** (`spiderpig/construction/handcrank.py`, `config.crank == "hand"`).
  - **Gearbox:** an NRV040 worm gear reducer at 30:1 (Phidgets TRM4401_0, rated 45 N·m;
    `handcrank.NRV040`). It stands outboard on the drive side's outer plate, on 0.125 in
    riser discs, held by four M6 button heads into its tapped face holes (75 mm PCD). A
    full 6 L peaks at about 32 N·m at the crank: the NRV030 (20 N·m) that held a 3 L
    couldn't.
  - **Holding:** its static efficiency at 30:1 is 0.45 (Motovario's mesh data), under the
    0.5 that the makers' catalogues give for static irreversibility. So the cradle holds
    any angle when let go.
  - **The crank block:** 0.125 in 6061 plates, each the arm's whole outline, stacked on the
    stock 19 mm keyed output shaft (Phidgets TRM4006_0, modelled with its collar and key).
    - Every plate round the shaft is keyed, the lowest four on the 6 mm key's straight
      flanks: the shaft's keyseat is end-milled, so the key's ends are round (the
      fabrication review).
    - The top plate lies over the shaft's end: an M6 button head through it into the
      shaft's end tap, to 3 N·m (it clamps the block on the printed sleeve). It goes in dry:
      the sleeve settles, so after the first few pours it comes out for threadlocker and
      goes back to 3 N·m.
    - A NETRF10 through the block (`WormBox.tie_r`: 52 mm on the NRV040), screwed at both
      ends, makes the plates one rigid block and clamps them, its screw head and wide
      washer clear of the riser screws.
  - **Crankpin:** a 12 mm NETRF12 standing in the block from its lowest plate (an M6 up
    through it) through the rod, an M6 and a wide washer on its end over the rod (a 6 L's
    jam bent the 10 mm one: SF 1.13), 0.2-0.3 mm of
    play (its length rounds down; a printed spacer under the rod, sanded on the bench,
    takes the plates' tolerance). The washer keeps the rod 0.26 mm (nominal) under the rear
    tie's screw head where that head crosses it, 1 to 4 handle turns from rest. The chain's
    tolerance is about 0.3 mm, so the guide checks it with a sheet of paper: a rub there is
    a scrape, not a jam. The rod rides 3 mm over the block's screw heads, its force 8.7 mm
    off the crankpin's socket. (Before, the arm sat on the key and the rod 21 mm over it: an ISO 7379
    crankpin's M8 thread was a cantilever that long, rated at SF 1.0-1.25 once its yield
    and preload were right.)
  - **Hand crank:** a printed offset crank on the gearbox's 11 mm input shaft.
    - Its hub is keyed on the shaft, with an M3 set screw on the key.
    - A 100 mm tube carries the arm out to where no moving part of the stack ever reaches,
      clear of the outer plate. The arm is 100 mm on B and 105 on A (`handcrank.HANDLE_RS`):
      a full 6 L's worst turn under 25 N at the grip.
    - Its grip is a printed sleeve spinning on a NETRF10 standing in a socket in the arm, an
      M5 up through the arm into it and an M5 and a wide washer on its end. (An M6 shoulder
      screw's 11 mm thread through the 10 mm arm had caught 1 mm of its nut; the standoff
      also takes McMaster off the order.)
  - **No guard over the crank** (the user's call, 2026-10-10: the guard, a disc in the
    outer plate and then the gearbox's flange, read as a big disc either way). The gearbox
    sits on its small riser discs; the crank turns about 2 rpm behind the outer plate's
    truss, and the use limit keeps fingers out of the drive side.
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
  - About USD 21-24 a pin (the discount is per length), so only where the jam needs it
    (`config.PRODUCT_PIN_JOINTS`): B's rod pin P and arm pin K, A's one pin E. B's J, J2, X
    and Y stay M6 Chicago screws (8 mm barrel, 243 threadlocker, the barrel's head held
    from below with taped pliers), which hold the 6 L's jam there at SF 1.39 to 2.19. The
    shoulder screws tried first are in [DECISIONS.md](DECISIONS.md).
- **The pillars** are 12 mm steel standoffs (MISUMI NETRF12, M6 ends;
  `StandoffAxle.cradle_shaft`), each made to its plates' gap: a 6 L's jam bent the 10 mm
  ones (SF 0.78 at B's O2). An M6 button head goes through each plate without a washer, to
  6 N·m; its 3.3 mm head stands outside the plate, where a cradle's side is open
  (`StandoffAxle.outside`). The stand's columns are 10 mm, as the tie rods.
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
    jam: 6061 held 0.83; SendCutSend's datasheet, yield 69 ksi), b5 in 5052; its 336 mm
    rod is 0.250 in 5052 and its cradle link 0.125 in 5052,
    each alone in its layer.
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
    down to about 300 mm over the table, tied to the outer plate by four columns.
    - Each side is then a deep box through its front leg.
  - Nothing crosses the bay over the mouth.
- **The base board** (`cradle._base_plate`, the part `base_plate`): 9 mm Baltic birch
  plywood, routed by SendCutSend (`birch_9mm`, the user's of 2026-10-13), 530 mm across
  the bay (`cradle.BASE_HALF`: a full 6 L tips past 25° sideways), 3× as stiff as the
  0.125 in aluminium it replaced. SendCutSend publishes no plywood price: its cutting is
  quoted at upload. Oil it; it isn't weather-proof.
  - Both outer plates' rails bolt onto it with printed L-brackets: an M5 screw and nut
    through the upright and the outer plate, an M6 x 20 carriage bolt up through the board
    and the foot (Bolt Depot 18220). Its hole is round (`cradle.CARRIAGE_HOLE`); its square
    neck is pulled into the wood, as a carriage bolt is meant to be, so its nut goes on from
    above: nothing to reach under the board. (A router can't cut the square hole the
    aluminium plate had: its bit leaves 1.6 mm corners.)
    - All the bolts go in with the board. The passive side is set down 7-15 mm out from its
      place, inside its bolts, and pushed home.
  - A U-notch from its front edge takes the vessel's riser, slid in on the table; the
    notch's round end puts it under the mouth.
  - Four printed TPU feet snap into its corner holes (a cone and a bore up the stem).
- **The light:** a 14 x 60.5 mm AAA keychain light (Olight i3E EOS) stands in a printed
  clip under the front saddle's back end (its flange on the saddle's underside, two M5s),
  so the clip prints upright and round and the saddle on its end. Its head sits under the
  shoulder; an M3 set screw holds it while you twist the head to switch it on.
  - A longer light's tail reaches a decanter's shoulder at rest.
- **Display only** (`fab="display"`: drawn, never bought, cut or printed): the bottle (a
  1.5 L Bordeaux magnum), a RIEDEL Ultra decanter (225 mm, `cradle.DECANTER_PROFILE`) and
  its riser (`cradle.DECANTER_RISER`, 135 mm: the highest that clears the whole swing).

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
other size lies on adapters. What it changed, and why:
- **The mouth 550 mm over the table** (from 460): at rest, 50° neck up, the 6 L's base and
  its base stop hang about 520 mm under the mouth. The stand's legs are 90 mm longer; the
  linkages are unchanged relative to the mouth.
- **The saddles and ties** (`cradle.SADDLES`, `decanters.TIES`): Vs for a 174 mm body and a
  48 mm neck, 190 mm across; the rear saddle 125 mm long, reaching behind the 6 L's centre
  of mass, and a V shell; the tie points lower and further back. Both cradles re-plan round
  them.
- **The drive:** a full 6 L takes about 32 N·m at the crank (B; A 34), 2.2 x the 3 L's. The
  NRV030 (20 N·m) became an NRV040 (45 N·m: SF 1.41 on B, 1.33 on A), and the handle grew
  to 100 mm (A 105), so the worst turn stays under 25 N at the grip.
- **The jam grows with the handle.** A firm 40 N push on a 100 mm handle through the worm
  at a hand's speed is 66 N·m at the crank (A 69.3: `config.PRODUCT_JAM_NM`), where it had
  been 36. Whatever the ratio or the handle, a jam push about twice the worst turn's puts
  the drive path at about twice the bottle's peak: every part between the crank and the
  cradle needed 1.4-1.9 x its strength.
  - The pillars and the crankpin: 12 mm NETRF12 (1.73 x the 10 mm's bending section).
  - B's arms b2, b3 and bar b4: 7075-T6 0.190 in (1.72 x 6061's yield).
  - The rods: 0.250 in 5052 on both, and every link 24 mm wide (their buckling; a 0.313 in
    rod failed SendCutSend's web rule round its pin).
  - The crank block: four keyed plates on the NRV040's 19 mm shaft and 6 mm key.
  - The pins: steel ("The pins" above), the user's choice of 2026-10-13: an M6 Chicago
    screw's 8 mm stainless barrel held 0.69 at B's rod pin P.
- **The stand:** the base board 530 x 626 mm (from 500 x 596): a full 6 L tips past 25° over
  its sides and its rear edge.
- **The base board in wood** (the same day's ask): 9 mm Baltic birch, routed.

## The vessel under the mouth

At rest, 50° neck up, the cradle box hangs straight under the mouth: the front tie rods, the
front saddle and the light cross the decanter's axis from 385 mm over the table up.
`cradle_report.room` measures the room a vessel has round its axis at every height, over
the whole swing and from the pour on (the cradle past −20°), with a 6 L methuselah in the
saddles too; the cradle page (`/api/cradle/{key}/room`) stands each vessel on the highest
riser that clears it (`viewer/src/decant/liquid.ts`, `bestRiser`). The spare sizes'
adapters lying on the table don't ride the cradle (only the drawn magnum's do).
- For the whole swing: 240 mm across up to 165 mm over the table, 95 mm across above that,
  its rim no higher than 380 mm. A RIEDEL Ultra stands on a 135 mm riser, its rim about
  190 mm under the mouth.
- From the pour on: its rim up to about 520 mm over the table (55 mm across above 475), 30
  mm under the mouth. Raise the jack once the cradle is past −20° and lower it before
  cranking back to rest: the worm would push the cradle box down onto it.

## Flat pack

`spiderpig/cradle_pack.py` (`flat_pack`) takes the cradle apart for the trip and stacks it
in one box. The cradle page's Flat pack button flies each piece into the box, in the order
they go in (`/api/cradle/{key}/pack`).

**What comes apart.** Only what is already fastened. The steps are read off the parts and
the linkage, so they follow the design (`cradle_pack.pieces`):
1. The bottle's adapters come out of the saddles. Every adapter and collar travels loose.
2. The hand crank comes off its shaft (its set screw).
3. The base board comes off (the brackets' nuts). The carriage bolts stay in it.
4. Each inner plate comes off (its screws, from the bay side).
5. Each side panel comes off its cradle link, where the link meets the side
   (`mech.connections`).
   - A pin: its upper E-ring comes off; the pin stays in the side's links, the ring back on
     it for the trip. A's pin E (drive side), B's pins X and Y (both sides).
   - A pillar slides out of the link's bore: A's pillar G.
   - The cradle box then comes away whole.
6. The cradle box comes apart:
   - the base stop and the light (its clip's screws) off their saddles;
   - one cradle link off the tie rods (its tie screws, threadlocked);
   - the saddles slid off the rods (their set screws), each with its liner;
   - the other link off. Each tie rod travels with its two screws back in its ends, and
     each cradle link with the loose spacers that sat over it.

A link left hanging on one joint once its cradle link is off swings into its panel
(`cradle_pack.tuck`): B's b5 on J2, which stood above the frame.

**How it stacks** (`cradle_pack.pack`). It works on each part's bounding box, 3 mm apart
(`CLEAR`):
- **Levelled.** Every piece but the panels and the base board is turned in the
  mechanism's plane to its least bounding box (`level`). It then lies in one of its six
  axis orientations.
- **The base board** stays at the bottom, where it is. The box is as wide as the largest
  piece lying flat.
- **The side panels** drop onto a heightmap, in both orders, either face down, either way
  round; the lowest pair wins. They nest: one panel's pillars stand between the other's
  gearbox and brackets.
- **The rest fill in**, largest first, at the lowest free slot of a voxel grid. That
  includes the cavities under a panel's plate. A slot is picked by:
  - what raises the box least;
  - then the lowest, the flattest;
  - then the nearest a corner.
  A piece never goes both over and under another: it couldn't go in.
- **The order in**: bottom up, as the stack allows (what is under a piece goes in first).

The box's bounds per design are in `tests/test_cradle.py` (`PACK_BOX`).

## Loads and checks

The design case is a sealed full 6 L methuselah: 11.1 kg, its centre of mass about 331 mm
behind the mouth along the axis (estimated: 42 % of its length from the base), in the bare
saddles (the drawn magnum's adapters out: `cradle_report.masses`), plus the cradle's own
weight.

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
- What A holds there: pin E (steel) 3.1, the crankpin 1.6, the keyed plates 1.9, the rod's
  buckling 2.05, pillar G 2.85, the links 1.62 and more.
- Turning it: 33.8 N·m at A's crank at worst (the NRV040's SF 1.33), 23.8 N at the grip.
  It tips at 25.6° and takes a 40 N push at a bottle's top, the worst over the swing.
- The rating is the plan's: a wider stack between the rod and the cradle link puts pin E
  lower. A product is never served a plan from another engine version
  (`stages.planning._reuse_plan`): it re-plans, proven thinnest.
- A link's net section is rated at its own pin's or pillar's hole (8.35-12.35 mm:
  `strength.link_rows`' ``holes``), not a walker's 6.35 mm.

Every joint on both sides is rated at the full planar load (`cradle_report.SHARE` = 1),
not a share of it. The rod drives one side, whose pivots carry half the weight plus the
rod's whole force.

`spiderpig/cradle_report.py` (`report(config)`) measures:
- **Swing:** the swing and each bottle's own mouth wander.
- **Drive:** the crank's peak torque, the gearbox's safety factor, the force at the crank's
  grip, the jam's push and torque, the turns per pour.
- **Rod and crankpin:** the rod's buckling, the crankpin standoff as a cantilever off the
  block.
- **Stand:** the whole cradle's centre of mass over the swing against the base board's
  feet: the margin, the tilt it tips at and the push at the bottle's top that tips it,
  each the worst over the swing (the code review: they had been read at one crank angle).
- **Joints and plates:** every pin, pillar and link plate rated by `strength.check` at
  both cases (`strength.design_loads` takes the report's loads for a cradle).
- **Clashes:** the whole cradle's clashes over the swing, the hand crank's swept volume
  against everything, and a 6 L methuselah in the saddles (`cradle.bottle_hits`).

`tests/test_cradle.py` holds the thresholds, for A:
- each bottle's mouth wander under 0.5 mm;
- tipping at 15° or more, and 25 N or more at the bottle's top, the worst over the swing;
- the crank's grip force under 25 N;
- the gearbox's safety factor at least 1.3, against its 45 N·m rating (the peak is a
  sealed full 6 L at one tilt, at hand speed);
- every pin and plate at least 3 walking and 1.3 jammed; the keyed plates at least 2.5
  walking and 1.5 jammed; the crankpin and the rod 1.5 jammed;
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
- **The stack is thin where it can be.** The cradle link b6 is alone under the inner plate
  at jam SF 6, so it is 0.125 in.

**What it holds** (`cradle_report.report`, a sealed full 6 L walking, a 40 N push at the
grip jammed, 2026-10-13):

| check | B |
|---|---|
| swing | −50.0° to +30.0°, the mouth still (0 mm), every bottle's mouth on the pivot |
| transmission | 36.7° at worst (the rod's); the parallelograms 48.5° and 51.5° |
| gearbox | 32.0 N·m peak: SF 1.41 on the NRV040's 45 N·m; 23.7 N at the 100 mm grip |
| jam (66 N·m) | pillar O2 1.31, pillar O1 1.39, pin J2 1.39 (Chicago), pin Y 1.49, pin X 1.72, links b3 and b4 1.73, the crank's keyway 2.0, pin J 2.19, pin P 3.51 and pin K 4.48 (steel) |
| walking | every joint 6.5 or more; the crank's keyway 4.1 |
| rod, crankpin | buckling 1.59 × its jam force; the crankpin 1.96 jammed |
| stand | tips at 26.0° at worst (a full 6 L), a push of 41.9 N or more at a bottle's top |

(For a 3 L on the NRV030, before 2026-10-13: the gearbox 1.38 on a 14.46 N·m peak, pin P
1.36 at a 36 N·m jam, the stand 30.1°.)

**The stand** (`cradle.STANDS`):
- **The outer plate** is two triangles on the pivots' bar: O1–O, O2–O and O1–O2. A front
  leg runs from O2, a rear leg straight down from O, and a rail along the base board.
- **The inner plate** is the bar O1–O2 alone, on its two pillars. Nothing crosses the bay.
- **The base board** runs from 160 mm in front of the mouth to 466 mm behind it: 626 by
  530 mm (a full 6 L tips past 25° over its sides and its rear edge; it was 596 by 500 for
  a 3 L). The first refit's had run 827 mm, to 640 mm behind the mouth, with the crank 480
  mm back.

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
2. **The base board:** the drive side bolts onto it (it can't stand alone).
3. **The cradle box:** the saddles slide onto the tie rods, then the light and the base
   stop go on.
4. **The passive side:**
   - its cradle link onto the tie rods' free ends;
   - its stack, from outside: the pillar goes up through the cradle link;
   - what goes over the link, then its inner plate, then its brackets.
5. **The hand crank**, then **the bottle:** its adapters (all but a 6 L methuselah) and its
   collar (a 3 L or a 6 L), the base stop set, then decanting.

Things to know before cutting and on the bench:
- **Get a live SendCutSend quote** (upload the DXFs). The BOM's cutting figure is a fit to
  the live quotes of 2026-10-10 for A's files (`hardware.bom.CRADLE_CUT_FITS`: USD 367.50
  then); the walkers' per-area rates had priced the old aluminium base plate alone at USD
  488. The plywood board and the 7075 have no fitted price: the BOM
  leaves them to the quote.
- **Measure the gearbox** when it arrives: the output face's spigot (2.5 mm is drawn; under
  2 mm, put an M6 washer under each riser screw's head), the output shaft's collar and key
  (drawn from Motovario's 18 mm NRV040 shaft: Phidgets' is 19 mm) and where the key sits.
- **File the keyed plates' keyways to the key** if they bind. A laser-cut slot comes back
  ±0.13 mm.
- **Oil the steel standoffs:** they are bare 1018. Wipe each with light oil, and keep the
  cradle dry.
- **Dry-fit the hinge:** the cradle link must swing under its own weight once the
  pillar's screws are tight.
- **Break the laser-cut edges** where hands go: the bay and the frame tops.
- **The room it needs** (`cradle.USE_LIMITS`, the guide's last step):
  - 950 mm of headroom: a 6 L's base at full tilt reaches about 910 mm over the table.
  - 150 mm behind the base board: a 6 L's base reaches past its rear edge at full tilt
    (B 112 mm, A 138).
  - The vessel: as "The vessel under the mouth" above. It stands under the mouth's centre
    (`cradle.DECANTER_X`) on its riser, which the notch puts there.
- **Keep the base board clear behind the decanter:** at rest the base stop comes down to
  about 18 mm over it.
- **Never force the crank.** If it stiffens, something is in the way: the drive holds a
  40 N push at the grip, and near either end of travel the cradle's force grows without
  bound. Keep fingers out of the drive side while it turns.
- **Uncork at rest**, the base stop set, holding the neck, a corkscrew's lever lifted
  straight up.
