"""Servo data: dimensions (servo frame, see :mod:`servos.spec`) and where to buy.

Sources are listed per servo. Dimensions marked *measured* were read from the
manufacturer's (or the SO-ARM100 project's) STEP model; the rest are from
datasheets and drawings.
"""

from __future__ import annotations

from hardware.catalog import Item, Offer, register
from servos import register as register_servo
from servos.spec import CadRef, HolePattern, Horn, Idler, MountHole, Relief, ServoSpec

# ---------------------------------------------------------------------------
# Feetech STS3215 (Waveshare / Seeed "ST3215"): the SO-100 / SO-101 arm servo
# ---------------------------------------------------------------------------

_STS_FRONT = tuple(MountHole(x, y, 2.4, screw="m2_self_tap_6", z=1.5)
                   for x in (8.3, 29.0) for y in (10.25, -10.25))
_STS_REAR = tuple(MountHole(x, y, 2.4, screw="m2_self_tap_6", z=-30.5)
                  for x in (8.3, 32.75) for y in (10.25, -10.25))

STS3215 = register_servo(ServoSpec(
    key="sts3215",
    name="Feetech STS3215 serial bus servo (7.4 V, 19.5 kg.cm)",
    bom_key="servo_sts3215",
    body=(45.22, 24.72, 32.0),      # hole face to hole face; 35 overall with the bumps
    axis_offset=12.5,               # output axis 10.11 mm from the near end
    spline_od=5.9,                  # 25 teeth
    seat_height=0.2,                # horn hub underside above the face around the shaft
    horn=Horn(
        name="stock aluminium disc horn",
        diameter=19.95,
        thickness=4.5,
        pattern=HolePattern(count=4, pcd=14.0, hole_d=3.4, thread="M3", angle_deg=0.0),
        center_screw_head_d=6.0,    # M3 x 6 centre screw head sits proud of the horn face
    ),
    mount=_STS_FRONT,
    mount_face_z=1.5,               # the hole strips; the face around the shaft is recessed
    front_reliefs=(Relief(9.57, 35.11, -6.7, 6.7, 1.1),),   # raised centre panel
    rear_face_z=-30.5,
    rear_mount=_STS_REAR,
    rear_reliefs=(
        Relief(19.8, 29.2, -9.0, 9.0, 1.9),      # connector bump (width not published)
        Relief(-10.0, 10.0, -10.0, 10.0, 2.6),   # idler boss (tip) and rear horn
    ),
    continuous=True,
    idler=Idler(boss_d=6.0, boss_h=4.1,
                pattern=HolePattern(count=4, pcd=14.0, hole_d=3.4, thread="M3")),
    cad=CadRef(
        url="https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/"
            "5f6d2b876a53a4872e405b991dd925556c9e38a4/STEP/SO100/STS3215_03a.step",
        sha256="cacd717c2f22bec856fdace2c3bc9fa58084d54b6e70378eae90562f34df93ef",
        filename="STS3215_03a.step",
        transform=(-1, 0, 0, 12.5, 0, -1, 0, 0, 0, 0, 1, -14.4, 0, 0, 0, 1),
        license="Apache-2.0",
        source="TheRobotStudio/SO-ARM100 (simplified solid incl. horns)",
    ),
    torque_kgcm=19.5,
    voltage=(4.0, 7.4),
    interface="TTL half-duplex serial bus (Feetech STS protocol), 12-bit magnetic encoder",
    notes="Turns continuously (closed-loop speed mode). Needs a bus adapter board. "
          "Mounting holes are 1.6 mm pilots for M2 self-tapping screws.",
    sources=(
        "https://files.seeedstudio.com/products/Feetech/108090023_STS3215-C001_Datasheet.pdf",
        "https://files.waveshare.com/upload/5/59/ST3215-3D.zip",
        "https://github.com/TheRobotStudio/SO-ARM100",
    ),
))

register(Item(
    "servo_sts3215", "Feetech STS3215 servo, 7.4 V 19.5 kg.cm (C001)", "servo",
    (
        Offer("Seeed Studio", "https://www.seeedstudio.com/STS3215-19kg-cm-7-4V-Serial-Servo-p-6338.html",
              sku="108090023", price_usd=20.0, verified=True),
        Offer("Waveshare", "https://www.waveshare.com/st3215-servo.htm", sku="33014",
              price_usd=16.99, verified=True),
        Offer("Amazon (RCmall)", "https://www.amazon.com/dp/B0F87Z9M3P", sku="B0F87Z9M3P",
              pack_qty=2, verified=True, note="2-pack"),
    ),
    notes="Both horns (output and rear idler) and screws come in the box.",
))
