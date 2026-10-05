"""Servo data: dimensions (servo frame, see :mod:`servos.spec`) and where to buy.

Only servos that turn a full revolution under closed-loop speed control are
registered: a Klann crank needs continuous rotation.

Dimensions marked *measured* were read with build123d/OCCT from the STEP
models listed in each spec (``CadRef``; the transforms into the servo frame
were checked by applying them); the rest are from the manufacturers'
datasheets and drawings, cited next to the value. ``UNVERIFIED`` marks a value
no primary source states (an inference, a single simplified model, or a
conservative stand-in); check it on a real part.

Buy links: ``verified=True`` means the page was fetched and showed the
product when this data was collected (2026-09-29); ``False`` means the link
was only seen in a search result or refused the fetch.
"""

from __future__ import annotations

from spiderpig.hardware.catalog import Item, Offer, register
from spiderpig.hardware.fasteners import shcs
from spiderpig.servos import register as register_servo
from spiderpig.servos.spec import (
                   BusPorts,
                   CadRef,
                   HolePattern,
                   Horn,
                   Idler,
                   MountHole,
                   Recess,
                   Relief,
                   ServoSpec,
)

# ---------------------------------------------------------------------------
# Feetech STS3215 (Waveshare / Seeed "ST3215"): the SO-100 / SO-101 arm servo
# ---------------------------------------------------------------------------
# Sources:
#   [FT]  Feetech STS3215 spec A/0 (2020), outline + horn drawings:
#         https://www.feetechrc.com/Data/feetechrc/upload/file/20200611/6372749961523760249976542.pdf
#   [C001] Seeed C001 datasheet: https://files.seeedstudio.com/products/Feetech/108090023_STS3215-C001_Datasheet.pdf
#   [WS2] Waveshare 2D drawing ("SCS215", 2022-06-08): https://files.waveshare.com/upload/0/08/ST3215-2D.zip
#   [WS3] Waveshare STEP (8 solids, horns separate), measured:
#         https://files.waveshare.com/upload/5/59/ST3215-3D.zip
#   [SO]  TheRobotStudio/SO-ARM100 STEP (Apache-2.0, one simplified solid), measured.
_STS_SCREW = "m2_self_tap_6"      # 8 x PA2.0 self-tapping [FT]; hole depth not published
_STS_FRONT = tuple(MountHole(x, y, 2.4, screw=_STS_SCREW)
                   for x in (8.3, 29.0) for y in (10.25, -10.25))       # [WS2] 18.41 / 20.7 / 20.5
_STS_REAR = tuple(MountHole(x, y, 2.4, screw=_STS_SCREW)
                  for x in (8.3, 32.75) for y in (10.25, -10.25))       # [WS2] 24.45 x 20.5

STS3215 = register_servo(ServoSpec(
    key="sts3215",
    name="Feetech STS3215 serial bus servo (C001: 7.4 V, 19.5 kg.cm, 1:345)",
    bom_key="servo_sts3215",
    body=(45.22, 24.72, 32.0),      # [WS2] 45.22 x 24.72; 32 between the hole faces [FT][WS2]
    axis_offset=12.5,               # [FT] axis 12.5 from the body centre; [WS2] 10.11 from the end
    spline_od=5.9,                  # 25 teeth [FT]
    spline_top=3.4,                 # [FT] spline end 3.4 above the face around the shaft
    seat_height=0.2,                # horn hub underside [WS3 measured]
    horn=Horn(
        name="stock aluminium disc horn (6061-T6), in the box",
        diameter=19.95,             # [FT]; [WS2]/[WS3] draw 19.2: the larger sizes the clearance
        thickness=4.5,              # [FT] 4.5 overall, 2.5 flange; [WS3] outer face at z = 4.7
        flange_thickness=2.5,
        hub_d=9.0,                  # [FT]
        center_hole_d=3.2,          # [FT]
        pattern=HolePattern(
            count=4, pcd=14.0, hole_d=3.4, thread="M3", angle_deg=0.0,   # [FT] 4-M3 on 14
            thread_depth=2.5,       # threads run through the 2.5 mm flange
            # [WS3 measured]: no case material under the flange at r = 5.3..8.7 above
            # z = 0.05, so a screw may pass the flange (underside z = 2.2) by up to
            # ~2.1 mm; keep 0.3 mm clear of the case: 4.7 - 0.05 - 0.3 - 0.05 = 4.3
            max_depth=4.3,
        ),
        # UNVERIFIED: the M3 x 6 centre screw's head type/size isn't published and its
        # head stands proud of the horn face [FT]; [SO] models it 5.4 x 1.5 (drawn so).
        # Clear an ISO 4762 M3 head (5.5 x 3.0) to be safe.
        center_screw_head_d=6.0,
        center_screw_head_h=3.0,
        center_boss=(5.4, 1.5),
    ),
    mount=_STS_FRONT,
    mount_face_z=1.5,               # hole strips [WS3][SO measured]; face around the shaft recessed
    front_recess=Recess(r=11.7, x_end=7.0, depth=1.5),   # [WS3 measured] face at z = 0
    front_reliefs=(
        # raised centre panel, 1.1 above the strips [WS3]. [WS3]: from an arc r ~ 11.7 about
        # the axis (corners at x = 9.57, |y| = 6.7) to the far end; [SO]: x >= 8.66,
        # |y| <= 7.0. The rectangle covers both models.
        Relief(8.6, 35.2, -7.0, 7.0, 1.1, label="raised centre panel"),
    ),
    rear_face_z=-30.5,              # [WS3][FT] 32 between the hole faces
    rear_mount=_STS_REAR,
    # face around the shaft at -29.0 [FT][WS3]; outline assumed as the front (UNVERIFIED)
    rear_recess=Recess(r=11.7, x_end=7.0, depth=1.5),
    rear_reliefs=(
        # connector bump: [WS3 measured] x 17.4..29.7, |y| <= 9.2, to z = -32.4;
        # [SO] x 17.3..29.9, |y| <= 9.2
        Relief(17.2, 30.0, -9.3, 9.3, 1.9, label="connector housing"),
        # the idler boss (6 mm, tip at z = -33.1 [FT]) alone: the rear horn (19.95 mm, outer
        # face -32.55 [WS3]) is left in the box (Idler.fitted, the assembly audit of
        # 2026-10-04): its 21 mm relief left the near rear screw 1.1-1.5 mm of web in the
        # 0.090 in centre plates, under 1 x t, and nothing on this robot uses the idler
        Relief(-3.0, 3.0, -3.0, 3.0, 2.6, solid=False, round=True,
               label="idler boss (the rear horn left off)"),
        # [SO] only: six 2 x 2 mm pins at x = 13..15 reaching z = -33.8 (not in [WS3])
        Relief(12.9, 15.2, -8.8, 8.8, 3.3, solid=False, label="pins in the SO-ARM100 model"),
    ),
    # The two bus sockets (5264 3P, [WS wiki]) in the connector housing. UNVERIFIED: which
    # way they open (both STEP models draw the housing solid); "end" is the SO-ARM100's
    # wiring, the plugs in along -x from the housing's far end. Plug: Molex 50-37-5033
    # (5264, 3 circuits) 9.9 wide, 3.9 thick, 8 long [Molex via distributors]. Measure a
    # servo and a plug before the centre plates are cut (assembly audit, 2026-10-04).
    bus_ports=BusPorts(17.2, 30.0, -9.3, 9.3, opening="end", count=2, plug_w=9.9,
                       plug_h=3.9, plug_len=8.0, label="bus sockets (5264 3P)"),
    continuous=True,                # [C001] "Limit angle: no limit", mode 1 = closed-loop speed
    idler=Idler(
        boss_d=6.0, boss_h=4.1, base_z=-29.0,    # [FT] rear boss 6 x 4.1 on the -29.0 face
        pattern=HolePattern(count=4, pcd=14.0, hole_d=3.4, thread="M3"),  # [FT] rear horn 4-M3
        # UNVERIFIED: passive idler inferred from the rear horn's plain 6.05 mm bore [FT]
        horn_d=19.95, horn_thickness=3.35, horn_face_z=-32.55,           # [FT] / [WS3]
        included=True,
        fitted=False,       # the rear horn stays in the box (the centre plates' webs)
    ),
    cad=CadRef(
        url="https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/"
            "5f6d2b876a53a4872e405b991dd925556c9e38a4/STEP/SO100/STS3215_03a.step",
        sha256="cacd717c2f22bec856fdace2c3bc9fa58084d54b6e70378eae90562f34df93ef",
        filename="STS3215_03a.step",
        transform=(-1, 0, 0, 12.5, 0, -1, 0, 0, 0, 0, 1, -14.4, 0, 0, 0, 1),
        license="Apache-2.0",
        source="TheRobotStudio/SO-ARM100 @5f6d2b8 STEP/SO100/STS3215_03a.step "
               "(one simplified solid, horns fused in)",
        strip_cut=(10.3, 0.05, 6.0),    # the fused output horn, its centre screw, the spline end
    ),
    cad_alternates=(
        CadRef(
            url="https://files.waveshare.com/upload/5/59/ST3215-3D.zip",
            archive_sha256="2735561e0c1dc899b35d4740273e5340f15967028d2dd4014feed03d8175562c",
            member="ST3215.step",
            sha256="58e38e4dc49f97df738c5f229f9aa8a7dce64a0a1d01335486a52d53e6017e8a",
            filename="ST3215.step",
            transform=(1, 0, 0, 25.5, 0, 0, -1, 0, 0, 1, 0, -4.9, 0, 0, 0, 1),
            license="not stated",
            source="Waveshare ST3215 STEP (8 solids, both horns separate)",
            strip=((-9.6, -9.6, 0.2, 9.6, 9.6, 4.7),),     # the output horn
        ),
    ),
    torque_kgcm=19.5,               # [C001] stall at 7.4 V (16.5 at 6 V)
    rated_kgcm=5.0,                 # [C001] rated torque
    voltage=(4.0, 7.4),             # [C001]
    speed_rpm=52,                   # [C001] 0.192 s/60 deg at 7.4 V, no load
    weight_g=55,                    # [FT]
    interface="TTL half-duplex serial bus (Feetech SMS/STS protocol, 1 Mbps default), "
              "12-bit magnetic encoder; 2 x 5264-3P connectors",
    notes="Turns continuously (closed-loop speed mode). Needs a TTL bus adapter board. "
          "Mounting holes are 1.6 mm pilots for M2 self-tapping screws (depth not published: "
          "measure before choosing screw length). Same case: C044 (1:191), C046 (1:147), "
          "C047 (12 V, 30 kg.cm).",
    sources=(
        "https://www.feetechrc.com/Data/feetechrc/upload/file/20200611/6372749961523760249976542.pdf",
        "https://files.seeedstudio.com/products/Feetech/108090023_STS3215-C001_Datasheet.pdf",
        "https://files.waveshare.com/upload/0/08/ST3215-2D.zip",
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
              verified=True, note="7.4 V version is SKU 33014; $16.99-21.99 by variant"),
        Offer("Amazon (RCmall)", "https://www.amazon.com/dp/B0F87Z9M3P", sku="B0F87Z9M3P",
              pack_qty=2, verified=True, note="2-pack; price not shown to the fetcher"),
    ),
    notes="Both horns (output and rear idler) and screws come in the box.",
))

# ---------------------------------------------------------------------------
# ROBOTIS DYNAMIXEL XL430-W250-T
# ---------------------------------------------------------------------------
# Sources:
#   [EM4] e-manual: https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/
#   [DR4] drawing "XL-430" 24-Mar-18 (FOR REFERENCE ONLY):
#         https://www.dropbox.com/s/j4uzfinaxd2vjws/XL-430_new.pdf?dl=1
#   [ST4] STEP "XL430_new" (25 solids; horn separate, idler fused in), measured.
_XL430_FRONT = tuple(
    # the 4 corner case screws (4-FHS M2.5x10 [DR4]) swap for longer M2.5 screws that clamp
    # a frame plate; through a 3 mm plate an M2.5 x 12 reaches 9 mm into the case (the
    # stock 10 mm flat-head reaches ~10)
    MountHole(x, y, 2.9, screw=shcs("2p5", 12), depth=10.0)
    for x in (-8.0, 32.0) for y in (11.0, -11.0))                 # [DR4] 40 x 22, [ST4]
_XL430_REAR = tuple(
    # UNVERIFIED: [ST4] shows the same corner bodies behind the back face; the drawing
    # calls out the corner screws once
    MountHole(x, y, 2.9, screw=shcs("2p5", 12), depth=10.0)
    for x in (-8.0, 32.0) for y in (11.0, -11.0))

XL430_W250 = register_servo(ServoSpec(
    key="xl430_w250",
    name="ROBOTIS DYNAMIXEL XL430-W250-T (11.1 V, 1.4 N.m)",
    bom_key="servo_xl430_w250",
    body=(46.5, 28.5, 34.0),        # [EM4] 28.5 x 46.5 x 34 (case only)
    axis_offset=12.0,               # [DR4] axis 11.25 from the near end
    spline_od=0.0,                  # not published by ROBOTIS
    seat_height=-1.5,               # the horn sits 1.5 deep in a pocket in the case [ST4 measured]
    horn=Horn(
        name="HN11-N101 horn (pre-installed)",
        diameter=20.5,              # [DR4]
        thickness=3.5,              # [ST4] z = -1.5 .. 2.0; outer face 2.0 above the case [DR4]
        pattern=HolePattern(
            count=4, pcd=16.0, hole_d=2.4, thread="M2", angle_deg=0.0,   # [DR4] 4-M2.0 TAP PCD 16
            thread_depth=3.5, max_depth=3.3,   # [DR4] DP3.5 (MAX.); 0.2 mm short of the bottom
        ),
        extra_holes=(HolePattern(count=4, pcd=16.0, hole_d=2.4, thread="M2", angle_deg=45.0,
                                 tapping=True, thread_depth=3.5),),       # [DR4] 4-1.7 DP3.5
        # raised centre ring on the output, through the horn: 10.1-10.6 across (x -5.3..5.07),
        # 1.0 above the horn face [ST4 measured]; centre screw FHS M2.6 x 8 tapping [DR4]
        center_screw_head_d=10.8,
        center_screw_head_h=1.0,
        center_boss=(10.4, 1.0),
        center_hole_d=10.8,
    ),
    mount=_XL430_FRONT,
    mount_face_z=0.0,               # flat front face [ST4]
    rear_face_z=-34.0,
    rear_mount=_XL430_REAR,
    rear_reliefs=(
        # idler HN11-I101: 20.5 disc 2.0 proud, 7.9 hub 3.9 proud [DR4]; [ST4] models it fitted
        Relief(-10.5, 10.5, -10.5, 10.5, 3.9, solid=False, label="HN11-I101 idler"),
    ),
    continuous=True,                # [EM4] velocity / extended position control modes
    idler=Idler(
        boss_d=7.9, boss_h=3.9, base_z=-34.0,          # [DR4] rear view
        pattern=HolePattern(count=4, pcd=16.0, hole_d=2.4, thread="M2", thread_depth=3.5),
        horn_bom_key="idler_hn11_i101",
        horn_d=20.5, horn_thickness=2.0, horn_face_z=-36.0,
        included=False,             # the HN11-I101 set is sold separately
    ),
    cad=CadRef(
        url="https://www.dropbox.com/s/wpy20ym8g1wvgbx/XL-430_new.stp?dl=1",
        sha256="5eeefe74bed670f70e993e103beab1d25e7bf2b9b297a7923066d96db550f2a1",
        filename="XL-430_new.stp",
        transform=(0, -1, 0, 0, -1, 0, 0, 0, 0, 0, -1, -17, 0, 0, 0, 1),
        license="not stated (ROBOTIS drawings are marked FOR REFERENCE ONLY)",
        source="ROBOTIS e-manual download no=773 (redirects to this Dropbox file)",
        strip=((-10.25, -10.25, -1.5, 10.25, 10.25, 2.0),),   # the HN11-N101 horn
    ),
    torque_kgcm=14.28,              # [EM4] 1.4 N.m at 11.1 V (1.0 at 9 V, 1.5 at 12 V)
    voltage=(6.5, 12.0),            # [EM4]; 11.1 V recommended
    speed_rpm=57,                   # [EM4] no load at 11.1 V
    weight_g=57.2,                  # [EM4]; the robotis.us listing says 65 g
    interface="TTL half-duplex multidrop bus, DYNAMIXEL Protocol 2.0, 12-bit absolute encoder",
    notes="Needs 6.5-12 V (3S Li-ion) and a DYNAMIXEL TTL adapter (U2D2 or similar). The 2 x "
          "2.1 mm pilot holes the drawing puts 8 mm from the axis on the horn side are on the "
          "idler side in the STEP: not used here.",
    sources=(
        "https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/",
        "https://www.dropbox.com/s/j4uzfinaxd2vjws/XL-430_new.pdf?dl=1",
        "https://www.dropbox.com/s/wpy20ym8g1wvgbx/XL-430_new.stp?dl=1",
        "https://www.robotis.us/dynamixel-xl430-w250-t/",
    ),
))

register(
    Item(
        "servo_xl430_w250", "ROBOTIS DYNAMIXEL XL430-W250-T", "servo",
        (
            Offer("ROBOTIS America", "https://www.robotis.us/dynamixel-xl430-w250-t/",
                  sku="902-0135-000", price_usd=27.5, verified=True),
            Offer("ROBOTIS", "https://en.robotis.com/shop_en/item.php?it_id=902-0135-000",
                  sku="902-0135-000", price_usd=23.9, verified=True),
            Offer("Trossen Robotics", "https://www.trossenrobotics.com/dynamixel-xl430-w250-t.aspx"),
        ),
        notes="HN11-N101 horn comes fitted; horn and frame screws in the box.",
    ),
    Item(
        "idler_hn11_i101", "ROBOTIS HN11-I101 idler set (XL430)", "horn",
        (
            Offer("ROBOTIS America", "https://www.robotis.us/hn11-i101-set/",
                  sku="903-0265-000", price_usd=8.05, verified=True),
            Offer("ROBOTIS", "https://en.robotis.com/shop_en/item.php?it_id=903-0265-000",
                  sku="903-0265-000", price_usd=7.0),
        ),
        notes="Idler, idler cap, FHS M3x5, 5 x PHS M2x5. Not for XM/XH430 (they use HN12-I101).",
    ),
)

# ---------------------------------------------------------------------------
# ROBOTIS DYNAMIXEL XL330-M288-T
# ---------------------------------------------------------------------------
# Sources:
#   [EM3] e-manual: https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/
#   [DR3] drawing "X330" 28-May-20: https://www.dropbox.com/s/gxgye7wt5sbt4i4/XL,XC-330.pdf?dl=1
#   [ST3] STEP "XL,XC-330" (15 solids: horn, horn screw, idler set separate), measured.
_XL330_SCREW = "m2_self_tap_10"   # 3 mm plate + 3.5 (front) / 4.5 (rear) non-gripping entry [DR3]
_XL330_HOLES = tuple((x, y) for x in (-7.5, 22.5) for y in (8.0, -8.0))   # [DR3] 30 x 16

XL330_M288 = register_servo(ServoSpec(
    key="xl330_m288",
    name="ROBOTIS DYNAMIXEL XL330-M288-T (5 V, 0.52 N.m)",
    bom_key="servo_xl330_m288",
    body=(34.0, 20.0, 23.0),        # [EM3] 20 x 34 x 26 incl. the 3 mm horn; case 23 [DR3]
    axis_offset=7.5,                # [DR3] axis 9.5 from the near end
    spline_od=0.0,                  # not published by ROBOTIS
    seat_height=0.1,                # horn flange underside [ST3 measured]
    horn=Horn(
        name="stock horn (pre-installed; spare: HNX330-N102)",
        diameter=16.0,              # [DR3]
        # [ST3] flange z = 0.1 .. 3.0 (outer face 3 above the case [DR3]); its 5 mm hub goes
        # into the case to z = -2.6, where the (simplified) case model is solid: not drawn
        thickness=2.9,
        pattern=HolePattern(
            # [DR3] "4-1.6 HOLE DP3.0(Max.) P.C.D 12 Using M2 Tapping Screw"
            count=4, pcd=12.0, hole_d=2.4, thread="M2", angle_deg=0.0, tapping=True,
            thread_depth=3.0, max_depth=2.8,
        ),
        center_screw_head_d=5.6,    # [ST3] the centre screw head sits flush in a 5.6 recess
        center_screw_head_h=0.0,
        center_hole_d=2.6,          # [ST3] screw shank (thread not published)
    ),
    mount=tuple(MountHole(x, y, 2.4, screw=_XL330_SCREW, depth=23.0)   # through the case
                for x, y in _XL330_HOLES),
    mount_face_z=0.0,
    rear_face_z=-23.0,
    rear_mount=tuple(MountHole(x, y, 2.4, screw=_XL330_SCREW, depth=23.0)
                     for x, y in _XL330_HOLES),
    rear_reliefs=(
        Relief(-8.5, 8.5, -8.5, 8.5, 3.0, solid=False, label="X330 idler (FPX330-H101 set)"),
    ),
    continuous=True,                # [EM3] velocity / extended position control modes
    idler=Idler(
        boss_d=16.0, boss_h=3.0, base_z=-23.0,         # [DR3] idler view: 29 = 3 + 23 + 3
        pattern=HolePattern(count=4, pcd=12.0, hole_d=2.4, thread="M2", tapping=True,
                            thread_depth=3.0),
        horn_d=16.0, horn_thickness=3.0, horn_face_z=-26.0,
        included=False,             # only sold in the FPX330-H101 frame set
    ),
    cad=CadRef(
        url="https://www.dropbox.com/s/qlzmp8mlvzrxmzu/XL,XC-330.stp?dl=1",
        sha256="e2f7b060801a1d6a21f23bca2554f29a402f7d73b8498cb201c9e6adf3139eb6",
        filename="XL,XC-330.stp",
        transform=(0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, -3.5, 0, 0, 0, 1),
        license="not stated (ROBOTIS drawings are marked FOR REFERENCE ONLY)",
        source="ROBOTIS e-manual download no=1987 (redirects to this Dropbox file)",
        strip=(
            (-8.0, -8.0, -2.6, 8.0, 8.0, 3.0),          # horn
            (-2.75, -2.75, -6.6, 2.75, 2.75, 2.97),     # horn screw
            (-8.0, -8.0, -26.0, 8.0, 8.0, -21.85),      # idler (not in the box)
            (-3.4, -3.4, -25.8, 3.4, 3.4, -21.2),       # idler cap
            (-2.75, -2.75, -25.77, 2.75, 2.75, -16.2),  # idler screw
        ),
    ),
    torque_kgcm=5.3,                # [EM3] 0.52 N.m at 5 V (0.42 at 3.7 V, 0.6 at 6 V)
    voltage=(3.7, 6.0),             # [EM3]; 5 V recommended
    speed_rpm=103,                  # [EM3] no load at 5 V
    weight_g=18,                    # [EM3]
    interface="TTL half-duplex multidrop bus (3.3 V logic, 5 V tolerant), DYNAMIXEL Protocol "
              "2.0, 12-bit absolute encoder; JST EHR-03 connectors",
    notes="Marginal torque for a single-servo walker. The horn centre-screw thread and the "
          "spline are not published.",
    sources=(
        "https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/",
        "https://www.dropbox.com/s/gxgye7wt5sbt4i4/XL,XC-330.pdf?dl=1",
        "https://www.dropbox.com/s/qlzmp8mlvzrxmzu/XL,XC-330.stp?dl=1",
        "https://www.robotis.us/dynamixel-xl330-m288-t/",
    ),
))

register(Item(
    "servo_xl330_m288", "ROBOTIS DYNAMIXEL XL330-M288-T", "servo",
    (
        Offer("ROBOTIS America", "https://www.robotis.us/dynamixel-xl330-m288-t/",
              sku="902-0163-000", price_usd=27.49, verified=True),
        Offer("ROBOTIS", "https://en.robotis.com/shop_en/item.php?it_id=902-0163-000",
              sku="902-0163-000", price_usd=23.9, verified=True),
        Offer("Mouser", "https://www.mouser.com/en/ProductDetail/ROBOTIS/902-0163-000"
              "?qs=aP1CjGhiNiFtAAOCtIQShg%3D%3D", sku="902-0163-000"),
    ),
    notes="Horn comes fitted; 6 x PHS M2x6 TAP (horn) and 10 x PHS M2x8 TAP (frames) in the box.",
))

