# Decanting cradles

This design record covers the hand-cranked decanting cradles in this repo. The user asked
for them on 2026-10-09: a linkage cradle that tips a 1.5 L magnum or a 3 L bottle to any
angle, driven by a hand crank, not a servo, with an LED.

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
| bottles | a 750 ml to a 3 L (double magnum, jeroboam) | the user, 2026-10-09 (magnum and 3 L), 2026-10-10 (regular bottles too) |
| bottle diameter | 74–141 mm | Bordeaux 750 ml 74–77, Bordeaux magnum 99, Burgundy ~106, Champagne magnum 114.5, 3 L Bordeaux 120.6–139, Champagne jeroboam 140.8 |
| bottle length | 295–470 mm | 750 ml 295–310, magnum 356–404, 3 L 380–470 |
| full mass | 1.3–5.9 kg | glass 0.5–2.9 kg plus the wine |
| the mouth | stays put over the whole pour, for either bottle | the decanter stands under it (US7708241) |
| tilt | −50° (rest, neck up) to +30° (neck down) | the user, 2026-10-10: start nearly upright. At +20° both bottles' shoulders lie over the mouth's lowest lip, so they drain; one crank turn is one decant and back |
| the mouth's height | 460 mm over the table | at −50° the 3 L's base stop hangs about 430 mm under it |
| the bay | 260 mm between the inner plates | a standard decanter (up to 240 mm across) stands wholly between the side stacks |
| vessel | a standard decanter or glass on a lab-jack riser | the vessel study (204–300 mm tall decanters, 155–240 across); the room under the mouth, below |
| drive | hand crank, self-locking | holds any angle when let go |
| light | an LED under the shoulder, riding the cradle | the candle's job |
| material | aluminium first | the user; the look is decided from the viewer |

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
    through the link into each end.
  - Two printed saddles ride on the tie rods. The front one carries the neck, and its back
    face is the shoulder stop. The rear one carries the body. Each is locked on its rods
    by M3 set screws up from its underside, in printed pilots.
  - Their Vs are sized for the 3 L. TPU liners line them.
  - **The base stop** (`cradle._base_stop`): a printed spine screwed to the rear saddle's
    back face runs under the bottle, and a printed cup on it stops the bottle's base. An M5
    button head through the cup into a pilot of the spine's row sets it, in 5 mm steps.
    - At rest the bottle stands 50° neck up and leans on it: the stop carries the bottle
      then (at 25° nose up the liners' friction alone had held it, and a corkscrew's push
      of 6-13 N slid it back off the pivot: the mechanical review).
    - It is a stop at rest, not a backstop: the spine is printed. At rest its rounded end
      comes down to about 20 mm over the base plate (keep it clear there).
- **The bottles** (`cradle.adapter`, `cradle.seat`): both lie with the mouth on the pivot
  and the axis level, so the mouth stays put for either.
  - A magnum lies on two printed TPU adapter liners: 3.5 mm in front, 21 mm at the rear.
    Its shoulder rests on the stop.
  - A 750 ml (`cradle.STANDARD`, a Bordeaux, 300 x 76.6 mm) lies on its own two: 4 mm in
    front, ending at its shoulder (its neck is 5 mm shorter than the stop's distance, so the
    liner's end is its stop), and 32 mm at the rear, closed 1 mm behind its base (it ends
    inside the rear saddle, where the base stop's cup can't reach). The model draws them
    lying on the table in front.
  - A 3 L lies in the liners alone, with a printed TPU C-collar, 20 mm long, snapped round
    its neck against the stop.
  - The model draws the magnum on its adapters, and the 3 L's collar lying on the table.
  - Another bottle: print adapters to its own neck and body (`adapter`).
- **The drive** (`spiderpig/construction/handcrank.py`, `config.crank == "hand"`).
  - **Gearbox:** an NRV030 worm gear reducer at 30:1 (Phidgets TRM4400_0). It stands
    outboard on the drive side's outer plate, on one 0.125 in riser disc, held by four M6
    button heads into its tapped face holes.
  - **Holding:** its static efficiency at 30:1 is 0.39, under the 0.5 that the makers'
    catalogues give for static irreversibility. So the cradle holds any angle when let go.
  - **The crank block:** seven 0.125 in 6061 plates, each the arm's whole outline, stacked
    on the stock 14 mm keyed output shaft (Phidgets TRM4004_0, modelled with its collar and
    key).
    - Every plate round the shaft is keyed, the lowest three on the key's straight flanks:
      the shaft's keyseat is end-milled, so the key's ends are round (the fabrication
      review).
    - The top plate lies over the shaft's end: an M6 button head through it into the
      shaft's end tap, to 3 N·m (it clamps the block on the printed sleeve). It goes in dry:
      the sleeve settles, so after the first few pours it comes out for threadlocker and
      goes back to 3 N·m.
    - A NETRF10 through the block at 47 mm (`handcrank.TIE_R`), screwed at both ends, makes
      the plates one rigid block and clamps them, its screw head and wide washer clear of
      the riser screws.
  - **Crankpin:** a NETRF10 standing in the block from its lowest plate (an M5 up through
    it) through the rod, an M5 and a wide washer on its end over the rod, 0.2-0.3 mm of
    play (its length rounds down; a printed spacer under the rod, sanded on the bench,
    takes the plates' tolerance). The washer keeps the rod 0.26 mm (nominal) under the rear
    tie's screw head where that head crosses it, 1 to 4 handle turns from rest. The chain's
    tolerance is about 0.3 mm, so the guide checks it with a sheet of paper: a rub there is
    a scrape, not a jam. The rod rides 3 mm over the block's screw heads, its force 8.7 mm
    off the crankpin's socket. (Before, the arm sat on the key and the rod 21 mm over it: an ISO 7379
    crankpin's M8 thread was a cantilever that long, rated at SF 1.0-1.25 once its yield
    and preload were right.)
  - **Hand crank:** a printed offset crank on the gearbox's 9 mm input shaft.
    - Its hub is keyed on the shaft, with an M3 set screw on the key.
    - A 100 mm tube carries the 60 mm arm out to where no moving part of the stack ever
      reaches, clear of the outer plate.
    - Its grip is a printed sleeve spinning on a NETRF10 standing in a socket in the arm, an
      M5 up through the arm into it and an M5 and a wide washer on its end. (An M6 shoulder
      screw's 11 mm thread through the 10 mm arm had caught 1 mm of its nut; the standoff
      also takes McMaster off the order.)
  - **No guard over the crank** (the user's call, 2026-10-10: the guard, a disc in the
    outer plate and then the gearbox's flange, read as a big disc either way). The gearbox
    sits on its small riser discs; the crank turns about 2 rpm behind the outer plate's
    truss, and the use limit keeps fingers out of the drive side.
- **The pins** (`chicago6`, `config.PRODUCT_PINS`): M6 Chicago screws with an 8 mm
  barrel.
  - Unlike a walker's pins, these aren't glued in (`ChicagoShaft.bonded`). Every link runs
    on the barrel, 0.25 mm of axial play at least.
  - Hold the barrel's head from below with taped pliers while the screw goes in; the head
    is in the open under the rod. No epoxy goes on the order. The screw takes the medium
    threadlocker (243): the low-strength 222 cures poorly between two stainless parts.
- **The pillars and columns** are the same 10 mm steel standoffs as the tie rods
  (`StandoffAxle.cradle_shaft`), each made to its plates' gap. An M5 button head goes
  through each plate without a washer, to 4 N·m.
  - Every printed ring fills its layer (`construction.axle.full_rings`), so every column's
    bought length is the gap between its plates. Touching rings print as one sleeve.
  - On the hinge the printed parts and the cradle link stop 0.3 mm short of the standoff
    (`pivots.standoff.RUN_PLAY`): the plates clamp the steel, and the link turns.
- **The sheets.**
  - Links: 0.125 in 5052, the cradle's default sheet.
  - Frame plates: 0.187 in 5052, 3.4× as stiff across as 0.125 in, and still cut with the
    M5 screws' 5.5 mm holes. In 0.250 in, SendCutSend's least hole would be 6.35 mm.
  - The rod: 0.187 in 5052, so it doesn't buckle.
  - The crank block: 0.125 in 6061 (the keyway's 5.1 mm slot is legal in it).
  - B's links are 0.187 in, its arms b2 and b3 and its bar b4 in 6061; its 411 mm rod is
    0.250 in 5052, alone in its layer.
  - Everything is cut at SendCutSend: `materials.LINK_SHEETS` and `config.PRODUCT_*`.
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
- **The base plate** (`cradle._base_plate`): 0.125 in 5052, 500 mm across the bay
  (`cradle.BASE_HALF`: the stand then takes a 30 N push at a bottle's top over its whole
  swing, where 400 mm took 18 N).
  - Both outer plates' rails bolt onto it with printed L-brackets: an M5 screw and nut
    through the upright and the outer plate, an M6 carriage bolt up through the base plate
    and the foot (its square neck in a square hole, so its nut goes on from above: nothing
    to reach under the plate).
    - The holes are 7.3 mm square, drawn with SendCutSend's 0.8 mm round inside corners:
      they clear a sharp 6.48 mm neck's corners by 0.25 mm (`cradle.CARRIAGE_SQUARE`).
    - All six bolts go in with the base plate. The passive side is set down 7-15 mm out
      from its place, inside its bolts, and pushed home.
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
  its riser (`cradle.DECANTER_RISER`, 55 mm: the highest that clears the whole swing).

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
  `cradle.BAY`); the saddles stay 160 mm long (`cradle.SADDLE_HALF`), centred on the rods.

## The vessel under the mouth

At rest, 50° neck up, the cradle box hangs straight under the mouth: the front tie rods, the
front saddle and the light cross the decanter's axis from 300 mm over the table up.
`cradle_report.room` measures the room a vessel has round its axis at every height, over
the whole swing and from the pour on (the cradle past −20°), with a 3 L bottle in the
saddles too; the cradle page (`/api/cradle/{key}/room`) stands each vessel on the highest
riser that clears it (`viewer/src/decant/liquid.ts`, `bestRiser`).
- For the whole swing: 240 mm across up to 135 mm over the table, 125 mm across above that,
  its rim no higher than 285 mm. A RIEDEL Ultra stands on a 55-60 mm riser, its rim about
  175 mm under the mouth.
- From the pour on: its rim up to about 390 mm over the table (65 mm across above 330), 70
  mm under the mouth. Raise the jack once the cradle is past −20° and lower it before
  cranking back to rest: the worm would push the cradle box down onto it.

## Flat pack

The cradle comes apart where it is already fastened (`spiderpig/cradle_pack.py`,
`flat_pack`): the hand crank (its set screw), the base plate (the six brackets' nuts; the
carriage bolts stay in it), each inner plate (its five screws from the bay side), pin E's
Chicago screw, then each side panel slides off its cradle link, leaving the cradle box (both
links, the tie rods, the saddles, the light) whole; the base stop comes off the rear
saddle (two screws). The modules stack in one box, about 656 x 516 x 363 mm outside (123 L):
the base plate under the passive panel, the drive panel over it, the cradle box on top, the
small pieces round them. The cradle page's Flat pack button flies each module there
(`/api/cradle/{key}/pack`); taking the cradle box apart too (its eight tie screws) would
bring the box down to about 250 mm.

## Loads and checks

The design case is a sealed full 3 L bottle: 5.9 kg, with its centre of mass about 265 mm
behind the mouth along the axis, plus the cradle's own weight.

The jam case is a firm push at the crank's grip on a blocked cradle
(`cradle_report.JAM_HAND_N`, 40 N: about twice what the worst turn takes), through the
worm at a hand's speed (`cradle_report.ETA_HAND`, 0.5): 36 N·m at the crank
(`cradle_report.jam_nm`, `config.PRODUCT_JAM_NM`).
- Rated at the gearbox's 20 N·m instead, as before, it was reached at less than the worst
  turn's own force (16.8 against 21 N): no bound (all three reviews).
- It is not counted within 30° of crank round either toggle, where a blocked crank-rocker's
  forces grow without bound. There, only clearance protects: the decanter's lip and
  anything under the cradle.
- What A holds there: the crankpin SF 1.9, the keyed plates 1.6, the rod's buckling 1.5,
  the pillars and plates 2.9 and more, pin E (the M6 Chicago barrel) 1.33. Pin E yields
  first, at about 53 N at the grip: its stainless barrel bends (the worm still holds).
- Turning it: 15.1 N·m at the crank at worst (the gearbox's SF 1.32), 21.5 N at the grip.
  It tips at 28.6° and takes a 27 N push at a bottle's top, the worst over the swing.
- The rating is the plan's: a wider stack between the rod and the cradle link puts pin E
  lower (a stale plan's put it at 0.82). A product is never served a plan from another
  engine version (`stages.planning._reuse_plan`): it re-plans, proven thinnest.

Every joint on both sides is rated at the full planar load (`cradle_report.SHARE` = 1),
not a share of it. The rod drives one side, whose pivots carry half the weight plus the
rod's whole force.

`spiderpig/cradle_report.py` (`report(config)`) measures:
- **Swing:** the swing and each bottle's own mouth wander.
- **Drive:** the crank's peak torque, the gearbox's safety factor, the force at the crank's
  grip, the jam's push and torque, the turns per pour.
- **Rod and crankpin:** the rod's buckling, the crankpin standoff as a cantilever off the
  block.
- **Stand:** the whole cradle's centre of mass over the swing against the base plate's
  feet: the margin, the tilt it tips at and the push at the bottle's top that tips it,
  each the worst over the swing (the code review: they had been read at one crank angle).
- **Joints and plates:** every pin, pillar and link plate rated by `strength.check` at
  both cases (`strength.design_loads` takes the report's loads for a cradle).
- **Clashes:** the whole cradle's clashes over the swing, the hand crank's swept volume
  against everything, and a 3 L bottle in the saddles (`cradle.jeroboam_hits`).

`tests/test_cradle.py` holds the thresholds, for A:
- each bottle's mouth wander under 0.5 mm;
- tipping at 15° or more, and 25 N or more at the bottle's top, the worst over the swing;
- the crank's grip force under 25 N;
- the gearbox's safety factor at least 1.3, against its 20 N·m continuous rating at
  1400 rpm input (the peak is a sealed full 3 L at one tilt, at hand speed);
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
cradle box, the saddles, the base stop, the base plate and the light with A.

**The mechanism.** Two parallelograms keep the cradle turning about the mouth with nothing
there:
- **The pivots** O1 and O2 hang under the mouth on a ray 83° down, 109 and 328 mm from it.
- **The arms** b3 (O1–J) and b2 (O2–K) are 204 mm long. They swing 23.5° ahead of the
  bottle (`skew`).
- **The bar** b4 (X–J–K) translates, and the cradle b6 rides it at X.
- **The cradle link** b5 (Y–J2) holds the cradle's other pin. J2 lies 108 mm past O1 on
  the copy arm's own line (`y_across` 0).
- **The drive:** the crank O sits 480 mm behind and 320 mm under the mouth. A 410.6 mm rod
  runs from its 90.9 mm crank to the lever on b2 (143 mm, 54.5° off the arm).

**Why it looks this way.** The old B rested at −10°. Its −25° re-fit put the rear tie rods
through the inner bar O1–O2 and overloaded its pins. Each choice below answers one of those
or a rule of the build:
- **The ties never cross the inner bar.** The rear ties ride 292–320 mm from the mouth and
  never come further round than 72° down. The bar lies at 83°, from 97 mm out (the front
  ties stop at 85 mm).
- **The cradle link never sweeps a pivot.** Seen from the cradle, the pivots stay
  between 33° and 113° below the bottle's axis. b6's members cross the pivots' distances
  from the mouth only above that sector (X is 23.5° over the axis, the rear ties 21° under
  it) or in front of the mouth (Y). They pass O1 25 mm off at the closest.
- **Each pair shares a layer.** Y sits on the copy arm's line, so the bars b4 and b5 never
  meet. The lever on b2 never reaches b3. The planner finds the old sandwich: the rod, then
  b2 with b3, then b4 with b5, then the cradle link under the inner plate, every pin across
  one gap.
- **The handle is behind everything.** It turns behind the crank, and the crank is the
  machine's rearmost moving part.
- **The decanter stands in the bay.** The bay is 260 mm, so the decanter stands between
  the inner bars and never reaches a side's links.
- **The gearbox and the jam pull against each other.** A sealed full 3 L peaks where the
  crank-rocker turns the cradle fastest, near level. So the gearbox wants the two strokes
  alike, which takes a long rod for its crank.
  - At a jam the rod's force is worst 30° past the folded toggle: about 36 N·m over the
    crank times sin(30° (rod − crank)/rod). So the pins want a long crank.
  - 90.9 mm on 410.6 mm balances the two: the gearbox at SF 1.31, pin P at 1.38.
  - The rod is 0.250 in so that it doesn't buckle at that length: 1.8 × its jam force.

**What it holds** (`cradle_report.report`, a sealed full 3 L walking, a 40 N push at the
grip jammed):

| check | B |
|---|---|
| swing | −50.0° to +30.0°, the mouth still (0 mm), both bottles' mouths on the pivot |
| transmission | 37.7° at worst (the rod's); the parallelograms 43.5° and 56.5° |
| gearbox | 15.25 N·m peak: SF 1.31 on its 20 N·m; 21.7 N at the grip |
| jam | pin P 1.38, pillar O2 1.46, the crank's keyway 1.64, link b3 1.81, pillar O1 1.88, the rest 2 or more |
| walking | every joint 7.9 or more; the crank's keyway 3.9 |
| rod, crankpin | buckling 1.82 × its jam force; the crankpin 2.11 jammed |
| stand | tips at 31.6° at worst, a push of 37.9 N or more at a bottle's top |

**The stand** (`cradle.STANDS`):
- **The outer plate** is two triangles on the pivots' bar: O1–O, O2–O and O1–O2. A front
  leg runs from O2, a rear leg from O, and a rail along the base plate.
- **The inner plate** is the bar O1–O2 alone, on its two pillars. Nothing crosses the bay.
- **The base plate** runs from 187 mm in front of the mouth to 640 mm behind it.

**Tried and dropped** (a search over the parameters, each candidate then planned and
reported; [DECISIONS.md](DECISIONS.md) has the entry):
- **Pivots in front of the mouth**, a forward ray: the crank lands in front, and the
  handle turns back through the parallelogram.
- **Pivots past every tie**, 350 mm out or more: the bar b4 grows past 400 mm, and the walk
  loads triple.
- **A short rod**, 300 mm or less in 0.187 in: the rod's jam force stays over 950 N,
  and the gearbox exceeds its SF 1.3. Rod 376 mm on a 109 mm crank: jam 1.44, gearbox
  1.17. Rod 405 mm on a 67 mm crank: gearbox 1.33, jam 1.1.

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
2. **The base plate:** the drive side bolts onto it (it can't stand alone).
3. **The cradle box:** the saddles slide onto the tie rods, then the light and the base
   stop go on.
4. **The passive side:**
   - its cradle link onto the tie rods' free ends;
   - its stack, from outside: the pillar goes up through the cradle link;
   - what goes over the link, then its inner plate, then its brackets.
5. **The hand crank**, then **the bottle:** its adapters (a magnum) or its collar (a 3 L),
   the base stop set, then decanting.

Things to know before cutting and on the bench:
- **Get a live SendCutSend quote** (upload the DXFs). The BOM's cutting figure is a fit to
  the live quotes of 2026-10-10 for A's files (`hardware.bom.CRADLE_CUT_FITS`: USD 367.50
  then); the walkers' per-area rates had priced the base plate alone at USD 488.
- **Measure the gearbox** when it arrives: the output face's spigot (2.5 mm is drawn; under
  2 mm, put an M6 washer under each riser screw's head) and where the output shaft's key
  sits.
- **File the keyed plates' keyways to the key** if they bind. A laser-cut slot comes back
  ±0.13 mm.
- **Oil the steel standoffs:** they are bare 1018. Wipe each with light oil, and keep the
  cradle dry.
- **Dry-fit the hinge:** the cradle link must swing under its own weight once the
  pillar's screws are tight.
- **Break the laser-cut edges** where hands go: the bay and the frame tops.
- **The room it needs** (`cradle.USE_LIMITS`, the guide's last step):
  - 800 mm of headroom: a 3 L's base at full tilt reaches about 760 mm over the table.
  - 50 mm behind the base plate: a 3 L's base comes to its rear edge at full tilt.
  - The vessel: as "The vessel under the mouth" above. It stands under the mouth's centre
    (`cradle.DECANTER_X`) on its riser, which the notch puts there.
- **Keep the base plate clear behind the decanter:** at rest the base stop comes down to
  about 20 mm over it.
- **Never force the crank.** If it stiffens, something is in the way: the drive holds a
  40 N push at the grip, and near either end of travel the cradle's force grows without
  bound. Keep fingers out of the drive side while it turns.
- **Uncork at rest**, the base stop set, holding the neck, a corkscrew's lever lifted
  straight up.
