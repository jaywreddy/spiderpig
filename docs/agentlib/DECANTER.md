# Decanting cradles

This design record covers the hand-cranked decanting cradles in this repo. The user asked
for them on 2026-10-09: a linkage cradle that tips a 1.5 L magnum or a 3 L bottle to any
angle, driven by a hand crank, not a servo, with an LED.

Two linkages are built, both from the same parts: the drive, the cradle box, the stand and
the light.
- `decanter_hinge` (A): the cradle swings on a real hinge at the bottle's mouth.
- `decanter_rcm` (B): the hinge is virtual, a double parallelogram (a remote centre of
  motion). No pin sits at the mouth.

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
| the mouth | stays put over the whole pour | the decanter stands under it (US7708241) |
| tilt | −10° (rest, neck up) to +55° (neck down) | one crank turn is one decant; the crank-rocker's toggles are the soft stops |
| drive | hand crank, self-locking | holds any angle when let go |
| light | an LED under the shoulder, riding the cradle | the candle's job |
| material | aluminium first | the user; the look is decided from the viewer |

## The machine

- **Two side stacks**, each planned by `stack` as usual: an outer frame plate, the links,
  an inner frame plate. The bottle lies in the bay between the two inner plates.
  - The drive side (`L.`) carries the crank.
  - The other side, the passive side (`R.`), is the drive side's mirror image without the
    crank, the rod and the gearbox. The cradle box drives it.
  - Code: `spiderpig/construction/cradle.py`; the linkages are in
    `spiderpig/linkages/decanters.py`.
- **The cradle box.**
  - Four tie rods (6 mm steel standoffs made to length, MISUMI NETRF6) run between the two
    cradle links, at `decanters.TIES`.
  - Two printed saddles ride on the tie rods. The front one carries the neck, and its back
    face stops the shoulder. The rear one carries the body.
  - Both saddles have printed TPU liners, so they grip without scratching.
  - The seat puts a 3 L bottle's axis through the pivot. A magnum's axis falls within a
    few mm of it (`cradle.seat`).
- **The drive** (`spiderpig/construction/handcrank.py`, `config.crank == "hand"`).
  - **Gearbox:** an NRV030 worm gear reducer at 30:1 (Phidgets TRM4400_0). It stands
    outboard on the drive side's outer plate, on laser-cut riser discs, and is held by four
    M6 button heads into its tapped face holes.
  - **Holding:** its static efficiency at 30:1 is 0.39, under the 0.5 that the makers'
    catalogues give for static irreversibility. So the cradle holds any angle when let go.
  - **Arm:** the gearbox's stock 14 mm keyed output shaft (Phidgets TRM4004_0) carries a
    0.250 in 6061 arm plate, keyed directly on the shaft. A printed sleeve and spacer
    locate it, and a laser-cut cap with an M6 screw into the shaft's end tap clamps it.
  - **Crankpin:** an ISO 7379 shoulder screw (8 mm shoulder, M6) carries the rod.
  - **Handle:** a revolving grip (GN 798, M8) on a printed arm on the 9 mm input shaft.
- **The pins** (`chicago6`, `config.PRODUCT_PINS`): M6 Chicago screws with an 8 mm
  barrel. The pillars are one-piece 6 mm steel standoffs (`standoff`, made to length).
  Every printed ring fills its layer (`construction.axle.full_rings`), so every column's
  bought length is the gap between its plates.
- **The sheets.**
  - Frame plates and links: 0.125 in 5052.
  - The rod: 0.187 in 5052, so it doesn't buckle at the gearbox's rated torque.
  - The crank arm: 0.250 in 6061.
  - B's links are all 0.187 in, so no link rides in a layer that a thicker one made taller.
    B's levers b3 and b4, the most loaded plates, are 6061.
  - Everything is cut at SendCutSend; the sheets are listed in `materials.LINK_SHEETS`
    and `config.PRODUCT_*`.
- **The stand** (`cradle.STANDS`). Each side's outer plate is a truss carrying the hinge,
  the gearbox and the legs to its feet. A base tie at the rear feet and a bay tie between
  the inner plates hold the two sides apart. The front stays open for the decanter. Printed
  TPU boots sit on the feet.
- **The light:** an AAA LED penlight (Maglite Solitaire LED) stands in a printed clip on
  the front saddle's back. Its head sits under the shoulder; twist the head to switch it on.
- **Display only** (`fab="display"`: drawn, never bought, cut or printed): the bottle (a
  1.5 L Bordeaux magnum) and a decanter under the mouth.

## Loads and checks

The design case is a sealed full 3 L bottle: 5.9 kg, with its centre of mass about 265 mm
behind the mouth along the axis, plus the cradle's own weight. The jam case is the
gearbox's rated 20 N·m output torque on a blocked cradle (a hand forcing the crank against
something in the way). It is not counted within 30° of crank around either toggle, where a
blocked crank-rocker's forces grow without bound and the toggle itself is the stop.

`spiderpig/cradle_report.py` (`report(config)`) measures all of it:
- **Swing:** the swing and the mouth's wander.
- **Drive:** the crank's peak torque, the gearbox's safety factor, the handle force, the
  turns per decant.
- **Rod:** its buckling.
- **Stand:** its stability, as the worst margin of the centre of mass inside the feet over
  the swing, with a jeroboam.
- **Joints and plates:** every pin, pillar and link plate rated by `strength.check` at
  both cases, per side.
- **Clashes:** the whole cradle's clashes over the swing.

`tests/test_cradle.py` holds the thresholds:
- the mouth's wander under 1 µm;
- a stand margin over 20 mm;
- a gearbox safety factor of at least 2;
- a handle force under 25 N;
- every joint's walking safety factor at least 3 and its jam safety factor at least 1.

```bash
uv run python -c "import json; from spiderpig.config import BuildConfig; \
from spiderpig.cradle_report import report; \
print(json.dumps(report(BuildConfig(linkage='decanter_hinge', robot=False)), indent=1))"
```

## Building one

```bash
spiderpig build --linkage decanter_hinge --out build/cradle_a   # STEP, STL, DXF, BOM, ORDER.md
spiderpig build --linkage decanter_rcm --out build/cradle_b
spiderpig bake --linkage decanter_hinge                          # the viewer's .glb (8 s loop)
```

`ORDER.md` is the shopping list: a cart per vendor, the SendCutSend upload and the prints.
Things to do on the bench that the model can't:
- **Measure the gearbox's output shaft before printing the arm's sleeve and spacer.** The
  key's place on the shaft isn't dimensioned. Measure where the key starts, and print the
  sleeve so that the arm sits on the key.
- **Print the saddles and liners for the bottle you pour most.** The model seats a 3 L
  bottle on the pivot. The liners are thin TPU, so reprint them for another bottle shape.
- **Tune the light before use.** The penlight's head should sit about 10 mm under a
  magnum's shoulder. Slide it in its clip to suit the bottle.
