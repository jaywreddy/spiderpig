# Decanting cradles

This design record covers the hand-cranked decanting cradles in this repo. The user asked
for them on 2026-10-09: a linkage cradle that tips a 1.5 L magnum or a 3 L bottle to any
angle, driven by a hand crank, not a servo, with an LED.

Two linkages are built, both from the same parts: the drive, the cradle box, the stand
and the light.
- `decanter_hinge` (A), **the one to build**: the cradle swings on a real hinge at the
  bottle's mouth, from −25° to +55°.
- `decanter_rcm` (B), **a design study**: the hinge is virtual, a double parallelogram (a
  remote centre of motion), so no pin sits at the mouth. It rests at −10°, where an opened
  full bottle pours. Why it stops there is under "B, the study".

The adversarial reviews of 2026-10-10 (mechanical, fabrication, code) shaped what follows.
Each finding and its fix is in [DECISIONS.md](DECISIONS.md).

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
| bottles | 1.5 L magnum to 3 L (double magnum, jeroboam) | the user, 2026-10-09 |
| bottle diameter | 99–141 mm | Bordeaux magnum 99, Burgundy ~106, Champagne magnum 114.5, 3 L Bordeaux 120.6–139, Champagne jeroboam 140.8 |
| bottle length | 356–470 mm | magnum 356–404, 3 L 380–470 |
| full mass | 2.4–5.9 kg | glass 0.9–2.9 kg plus the wine |
| the mouth | stays put over the whole pour, for either bottle | the decanter stands under it (US7708241) |
| tilt | −25° (rest, neck up) to +55° (neck down) | an opened full 3 L holds from about −20° (the mechanical review); one crank turn is one decant and back |
| decanter | up to 310 mm tall, slid under the mouth | its lip about 29 mm under a 3 L's at full tilt |
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
    - At rest the bottle lies 25° nose up, and only the liners' friction held it: a
      corkscrew's push of 6-13 N slid it back off the pivot (the mechanical review).
    - It is a stop at rest, not a backstop: the spine is printed, and a swing of the loaded
      box into the base plate from more than about 10° past rest would break it. At rest
      its rounded end comes down to about 40 mm over the base plate (keep it clear there).
- **The bottles** (`cradle.adapter`, `cradle.seat`): both lie with the mouth on the pivot
  and the axis level, so the mouth stays put for either.
  - A magnum lies on two printed TPU adapter liners: 3.5 mm in front, 21 mm at the rear.
    Its shoulder rests on the stop.
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
      shaft's end tap, to 3 N·m (it clamps the block on the printed sleeve).
    - A NETRF10 through the block at 47 mm (`handcrank.TIE_R`), screwed at both ends, makes
      the plates one rigid block and clamps them, its screw head and wide washer clear of
      the riser screws.
  - **Crankpin:** a NETRF10 standing in the block from its lowest plate (an M5 up through
    it) through the rod, an M5 and a wide washer on its end over the rod, 0.3 mm of play
    (a printed spacer under the rod, sanded on the bench, takes the plates' tolerance). The
    rod rides 1.5 mm over the block's screw heads, its force 8.7 mm off the crankpin's
    socket. (Before, the arm sat on the key and the rod 21 mm over it: an ISO 7379
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
  - **The guard:** the outer plate is solid over the whole crank's sweep and 20 mm past
    it (`handcrank.GUARD_PAST`), so no finger reaches the block or the rod from outside.
- **The pins** (`chicago6`, `config.PRODUCT_PINS`): M6 Chicago screws with an 8 mm
  barrel.
  - Unlike a walker's pins, these aren't glued in (`ChicagoShaft.bonded`). Every link runs
    on the barrel, 0.25 mm of axial play at least.
  - Hold the barrel's head from below with taped pliers while the screw goes in; the head
    is in the open under the rod. No epoxy goes on the order.
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
  - B's links are all 0.187 in; its levers b3 and b4 are 6061.
  - Everything is cut at SendCutSend: `materials.LINK_SHEETS` and `config.PRODUCT_*`.
- **The stand** (`cradle.STANDS`).
  - **Outer plate:** a truss outside every moving part's layer: from the hinge G down in
    front of the decanter to the front foot and back to the crank O, O's legs down and
    back, a rail along the base, solid round the crank.
  - **Inner plate:** a block round the hinge, and a leg over the outer plate's front leg
    down to about 200 mm over the table, tied to the outer plate by four columns.
    - Each side is then a deep box through its front leg.
  - Nothing crosses the bay over the mouth.
- **The base plate** (`cradle._base_plate`): 0.125 in 5052, 500 mm across the bay
  (`cradle.BASE_HALF`: the stand then takes a 30 N push at a bottle's top over its whole
  swing, where 400 mm took 18 N).
  - Both outer plates' rails bolt onto it with printed L-brackets: an M5 screw and nut
    through the upright and the outer plate, an M6 carriage bolt up through the base plate
    and the foot (its square neck in a square hole, so its nut goes on from above: nothing
    to reach under the plate).
  - A U-notch from its front edge takes the decanter, slid in on the table; the notch's
    round end locates it under the mouth.
  - Four printed TPU feet snap into its corner holes (a cone and a bore up the stem).
- **The light:** a 14 x 60.5 mm AAA keychain light (Olight i3E EOS) stands in a printed
  clip under the front saddle's back end (its flange on the saddle's underside, two M5s),
  so the clip prints upright and round and the saddle on its end. Its head sits under the
  shoulder; an M3 set screw holds it while you twist the head to switch it on.
  - A longer light's tail reaches a decanter's shoulder at rest.
- **Display only** (`fab="display"`: drawn, never bought, cut or printed): the bottle (a
  1.5 L Bordeaux magnum) and a decanter under the mouth.

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
- What A holds there: the crankpin SF 2.0, the keyed plates 1.6, the rod's buckling 1.9,
  the pillars and plates 2.9 and more, pin E (the M6 Chicago barrel) 1.34. Pin E yields
  first, at about 54 N at the grip: its stainless barrel bends (the worm still holds).
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

## B, the study

B's 80° swing from −25° didn't close. The crank-rocker re-fitted for it needs a 90 mm
lever (the parallelograms kept off their fold by a `skew` of 35° between the arms and the
bottle). The jam loads then exceed B's pins (X and Y at SF 0.65), and the rear tie rods
sweep through B's inner bar O1–O2 at rest.

So B keeps its validated −10° to +55° swing, with every shared fix: the drive, the
saddles, the base plate and the NETRF10s. It stands and holds a sealed full bottle with
every joint at SF 2.6 or more walking. But at the full load share its jam case falls
under 1 (pillar O1, pin K, lever b4), and it can't hold an opened full bottle at rest.
Build A.

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
  - 800 mm of headroom: a 3 L's base at full tilt reaches 775 mm over the table.
  - 50 mm behind the base plate: a 3 L reaches 35 mm past its rear edge.
  - A decanter fits up to 180 mm tall at 260 mm across, 190 mm across up to 200 mm, 90 mm
    across up to 270 mm (the light's clip at rest), and 56 mm across (its neck) up to
    310 mm, its mouth 55 mm across or more. It stands under the mouth's centre
    (`cradle.DECANTER_X`), where the pour lands from about 15 mm in front to 16 mm behind.
- **Keep the base plate clear behind the decanter:** at rest the base stop comes down to
  about 40 mm over it.
- **Never force the crank.** If it stiffens, something is in the way: the drive holds a
  40 N push at the grip, and near either end of travel the cradle's force grows without
  bound. Keep fingers out of the drive side while it turns.
- **Uncork at rest**, the base stop set, holding the neck, a corkscrew's lever lifted
  straight up.
