"""One engine for every kind (2026-10-14, the second round's review): a design's motors, its
controller, its strength, its sim, its pivots and its exports follow its declared
capabilities (its inputs, ``Linkage.ranged`` / ``walks``, its robot layout), never its
kind's name."""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any, cast

import pytest

from spiderpig import construction, linkage, roles, strength
from spiderpig.actuators import controllers
from spiderpig.config import BuildConfig

MECH = "hoecken_pantograph"
ARM = "palletizer_reach"


# -- one drive vocabulary ---------------------------------------------------------------


@pytest.mark.no_fabricate
def test_the_motor_role_is_chosen_from_the_inputs_once():
    """A crank's drive is the role ``servo``, ranged inputs the role ``actuator``
    (:func:`roles.motor_role`); the roles' kinds, the drive groups and the controllers all
    speak those names."""
    assert set(controllers.DRIVES) == set(roles.MOTOR_ROLES) == set(construction.DRIVES)
    assert {c.drives for c in controllers.CONTROLLERS.values()} <= set(roles.MOTOR_ROLES)
    for key in linkage.available():
        lk = linkage.get(key)
        role = roles.motor_role(lk)
        assert role == ("actuator" if lk.ranged else "servo"), key
        assert roles.applies(role, lk.kind), key
        (other,) = [r for r in roles.MOTOR_ROLES if r != role]
        assert not roles.applies(other, lk.kind), key
        # (a product's hand crank, a decanting cradle's: no motor at all)
        want = "" if lk.product else role
        assert controllers.motors_of(BuildConfig(linkage=key, robot=False)) == want, key


@pytest.mark.no_fabricate
def test_a_cranked_linkage_may_declare_its_second_inputs_drive():
    """The engine side of a five-bar: a cranked linkage's second input may declare the link
    it turns on a frame joint (a stepper's ``JointDrive`` beside the crank's servo, and a
    held body in the constraints' layout); the crank's ``t`` may not, nor an input it
    lacks."""
    from dataclasses import replace

    from spiderpig.linkage.constraints import layout
    from spiderpig.repeatability import driven_inputs

    five = linkage.get("five_bar")
    assert layout(five).driven == ("conn",)          # undeclared: the servo's crank only
    driven = replace(five, key="five_bar_driven", drives={"t2": ("b3", "O2")})
    assert layout(driven).driven == ("conn", "b3")
    assert driven_inputs(driven) == ("t", "t2")
    with pytest.raises(ValueError, match="second input"):
        replace(five, key="five_bar_t", drives={"t": ("b3", "O2")})
    with pytest.raises(ValueError, match="frame joint"):
        replace(five, key="five_bar_p", drives={"t2": ("b3", "P")})


# -- the controller: every design with motors -------------------------------------------


@pytest.mark.no_fabricate
def test_every_design_with_motors_has_a_controller():
    """The role ``controller`` is a slot of every kind (a mechanism's too: the board that
    speaks its servo's protocol, named); a mechanism's side keeps its robot roles (no robot
    round it); unnamed, a mechanism's is none, what the mechanisms stored before the role
    reached them carried (``Role.implicit``), so their ids keep their BOMs."""
    for name in ("controller", "pin", "pillar"):
        assert set(roles.get(name).kinds) == {"walker", "mechanism", "arm"}
    mech = BuildConfig(linkage=MECH, robot=False)
    assert controllers.of(mech) is None
    assert mech.role("controller") == ""
    assert "controller" not in mech.applied_roles()          # implicit: its id stays
    kit = BuildConfig(linkage=MECH, robot=False, roles=(("controller", "esp32_bus_servo"),))
    assert controllers.of(kit) is controllers.get(controllers.DECK_KIT)
    assert kit.applied_roles() == {"controller": "esp32_bus_servo"}   # named: in its id
    dxl = BuildConfig(linkage=MECH, robot=False, servo="xl330_m288",
                      roles=(("controller", "u2d2_dynamixel"),))
    assert controllers.of(dxl) is controllers.get("u2d2_dynamixel")
    own = BuildConfig(linkage=MECH, robot=False, roles=(("controller", "none"),))
    assert own == mech                                        # (none is its default)
    assert controllers.of(own) is None
    assert kit.side_config().role("controller") == controllers.DECK_KIT   # (the machine)
    walker = BuildConfig(roles=(("controller", "none"),))
    assert walker.side_config().roles == ()                   # (its robot holds it)
    skr = BuildConfig(linkage=MECH, robot=False, roles=(("controller", "skr_pico_pi_zero"),))
    with pytest.raises(ValueError, match="drives steppers and hoecken_pantograph's motors "
                                         "are serial-bus servos"):
        controllers.of(skr)


@pytest.mark.no_fabricate
def test_a_role_reaching_a_kind_never_moves_what_its_stored_designs_read_back():
    """Same id, same parts and BOM: a resolved spec that doesn't name a role reads back with
    its default (``BuildConfig.role``), so where a kind's designs carried nothing before the
    role reached it (``Role.implicit``: ``none``) its default stays nothing for every
    linkage of the kind, and where they carried a key that key, its default, isn't written.
    A key chosen enters the id: a stored mechanism with no controller key reads back bare,
    one that names the kit with it."""
    from spiderpig.construction.robot import controller_lines
    from spiderpig.stages.resolve import _config_from_resolved, resolve, spec_of

    for r in roles.ROLES.values():
        for kind, key in r.implicit:
            assert kind in r.kinds, (r.name, kind)
            if key != roles.NONE:
                continue
            for lk in linkage.available(kind):
                cfg = BuildConfig(linkage=lk, robot=False)
                assert roles.default_key(r.name, cfg) == roles.NONE, (r.name, lk)
                assert r.name not in cfg.applied_roles(), (r.name, lk)
    with pytest.raises(ValueError, match="not a kind of it"):
        roles.Role("x", "x", "m:X", ("arm",), implicit=(("walker", "k"),))
    with pytest.raises(ValueError, match="needs optional"):
        roles.Role("x", "x", "m:X", ("arm",), implicit=(("arm", roles.NONE),))
    bare = BuildConfig(linkage=MECH, robot=False)
    kit = BuildConfig(linkage=MECH, robot=False, roles=(("controller", "esp32_bus_servo"),))
    stored = {}
    for cfg in (bare, kit):
        d = resolve(spec_of(cfg), None)
        assert _config_from_resolved(d.resolved) == cfg
        stored[d.id] = d.resolved
    (old, new) = stored.values()
    assert "controller" not in old["constructions"]           # what master stored
    assert new["constructions"]["controller"] == "esp32_bus_servo"
    assert controller_lines(_config_from_resolved(old), deck=False) == []
    assert controller_lines(_config_from_resolved(new), deck=False)


@pytest.mark.no_fabricate
def test_a_machine_with_no_deck_buys_its_controller_as_lines():
    """A walker's deck draws the ESP32 kit (no lines); a mechanism has no deck, so its
    controller's every part is a BOM line, and its wiring step lists them all."""
    from spiderpig.construction.assembly import RobotView, controller_ops
    from spiderpig.construction.robot import controller_lines

    kit = controllers.get(controllers.DECK_KIT)
    assert controller_lines(BuildConfig()) == []
    mech = BuildConfig(linkage=MECH, robot=False, roles=(("controller", kit.key),))
    lines = controller_lines(mech, deck=False)
    assert [(x.key, x.qty) for x in lines] == list(kit.bom)
    assert {x.where for x in lines} == {"controller: esp32_bus_servo"}
    assert controller_lines(BuildConfig(linkage=MECH, robot=False,
                                        roles=(("controller", "none"),)), deck=False) == []
    view = RobotView({"servo": (0.0, 1.0)}, {"servo": None}, {"servo": "purchased"},
                     {"servo": "servo_sts3215"}, {}, tuple(k for k, _ in kit.bom))
    (op,) = controller_ops(view, kit, ("t",), as_lines=True)
    assert set(op.extras) == {k for k, _ in kit.bom}
    assert "bus servo ports" in op.text                       # the board's own sentence
    (deck,) = controller_ops(view, kit, ("t",))
    assert set(deck.extras) == set(kit.wiring_extras)         # the deck's bodies: its own step


@pytest.mark.no_fabricate
def test_the_dynamixel_controller_declares_its_wiring():
    """``u2d2_dynamixel``: its boxes, links and sentences, its bought lines in its step."""
    from spiderpig.construction.assembly import RobotView, controller_ops

    ctl = controllers.get("u2d2_dynamixel")
    assert ctl.role_of("u2d2") == "board"
    assert ctl.role_of("u2d2_power_hub_set") == "hub"
    assert ctl.role_of("servo_xl330_m288") == "servo"
    view = RobotView({"L.servo": (0.0, 1.0)}, {"L.servo": None}, {"L.servo": "purchased"},
                     {"L.servo": "servo_xl330_m288"}, {}, ("u2d2", "u2d2_power_hub_set"))
    (op,) = controller_ops(view, ctl, ("t",), as_lines=True)
    assert op.title == "USB interface and bus"
    assert "U2D2's TTL port" in op.text
    assert "chained" in op.text
    assert set(op.extras) == {"u2d2", "u2d2_power_hub_set"}


@pytest.mark.no_fabricate
def test_a_bare_deck_carries_its_own_controller_not_the_kit():
    """A DYNAMIXEL walker's bare deck carries the U2D2 and its hub (their catalogued mass),
    not the ESP32 kit's; your own electronics stand in at the kit's mass."""
    from spiderpig.sim.mjcf import SimParams, bare_deck_g, kit_mass_g, payload_g

    dxl = BuildConfig(servo="xl330_m288")
    assert bare_deck_g(dxl) == pytest.approx(controllers.carried_g(
        controllers.get("u2d2_dynamixel")))
    assert bare_deck_g(dxl) == pytest.approx(9.0)            # the U2D2 (the hub unweighed)
    assert bare_deck_g(BuildConfig(roles=(("controller", "none"),))) == \
        pytest.approx(kit_mass_g())
    bare = cast(Any, SimpleNamespace(meta={"deck": {"fitted": True, "electronics": False}},
                                     walks=True, config=dxl))
    assert payload_g(bare, SimParams()) == pytest.approx(9.0)


# -- strength and sim: a mechanism rated, held and turned --------------------------------


@pytest.mark.no_fabricate
def test_a_mechanisms_jam_is_the_arms_crash():
    """A cranked mechanism's loads are the static model's (no "not rated"): operating its
    weight over a turn, jammed its output blocked while its servo pushes at its torque
    limit, the output's dead points left out; per joint as the sim's, the crank's flagged."""
    from spiderpig.config import torque_limit_nm

    cfg = BuildConfig(linkage=MECH, robot=False)
    lo = strength.design_loads(cfg)
    assert lo["source"] == "static"
    assert lo["torque_limit_nm"] == pytest.approx(torque_limit_nm(cfg))
    assert lo["capacity_nm"] == {"t": pytest.approx(torque_limit_nm(cfg))}
    assert "dead points" in lo["note"]
    assert 0 < lo["walk_n"] < lo["jam_n"]
    stems = {j["stem"]: j for j in lo["joints"]}
    assert stems["O"]["crank"]
    assert stems["M"]["crank"]
    assert not stems["P"]["crank"]
    assert all("conn" not in j["links"] for j in lo["joints"])
    heavy = strength.design_loads(cfg, payload_g=500.0)
    assert heavy["walk_n"] > lo["walk_n"]


@pytest.mark.no_fabricate
def test_a_mechanisms_overload_fails_verify(monkeypatch):
    """Its static loads are its own: an overload is a ``strength`` failure, as an arm's. At
    its own torque limit (``config.LINKAGE_TORQUE_LIMITS``) its links hold; at the servo's
    0.85 N·m they don't."""
    from spiderpig import config as configmod
    from spiderpig import verify

    design = SimpleNamespace(mech=SimpleNamespace(meta={}), store=None,
                             config=BuildConfig(linkage=MECH, robot=False),
                             spec=SimpleNamespace(payload_g=None))
    rep = verify.VerifyReport("standard")
    rows = verify._strength_rows(cast(Any, design), rep, "standard")
    assert rows[0].requirement == "strength.joints"
    assert rows[0].hard
    assert rows[0].passed
    assert float(cast(float, rows[0].value)) >= strength.JAM_WARN
    assert "over a turn of the crank" in rows[0].detail
    monkeypatch.delitem(configmod.LINKAGE_TORQUE_LIMITS, MECH)
    rep = verify.VerifyReport("standard")
    rows = verify._strength_rows(cast(Any, design), rep, "standard")
    assert not rows[0].passed
    assert (rep.failures[0].stage, rep.failures[0].code) == ("strength", "joint_overload")
    fixes = [fx for c in rep.failures[0].culprits for fx in c["fixes"]]
    assert any("torque limit" in fx for fx in fixes)
    # a crank's servo: its firmware limit is the lever, never a stepper's run current
    assert not [fx for fx in fixes if "holding torque" in fx or "run current" in fx]


@pytest.mark.no_fabricate
def test_holding_torque_fixes_are_a_steppers():
    """The fixes name the drives' holding torque only for a design whose motors are steppers
    (the motor role ``actuator``), whatever its loads' source: a link short of jam SF 2 on a
    cranked mechanism gets the servo's torque limit alone, on an arm the holding torque."""
    row = {"kind": "link", "links": ["b1"], "jam": {"safety": 1.0}, "walk": None}
    loads = {"source": "static", "torque_limit_nm": 0.2}
    assert not strength.stepped(BuildConfig(linkage=MECH, robot=False))
    assert strength.stepped(BuildConfig(linkage=ARM))
    mech = strength.fixes(row, None, loads, BuildConfig(linkage=MECH, robot=False))
    assert mech == ["a servo torque limit of 0.10 N·m (now 0.2) for jam SF 2"]
    arm = strength.fixes(row, None, dict(loads, torque_limit_nm=None), BuildConfig(linkage=ARM))
    assert arm == ["lower the drives' holding torque (their run current): a crash's load "
                   "scales with it"]


# -- the torque limit a crank's servo is set to: its BOM and its sim ---------------------


@pytest.mark.no_fabricate
def test_every_crank_servos_bom_names_its_torque_limit():
    """The firmware torque limit the joints are rated at is a BOM / ORDER.md note for every
    design a crank's servo drives (its motor role, not a walker's robot alone): a
    mechanism's own lowered limit, a walker side's; none for an arm's steppers. The export
    and the build add it ungated."""
    import inspect

    from spiderpig import build
    from spiderpig.api import exports
    from spiderpig.config import torque_limit_nm, torque_limit_note

    for key in linkage.available("mechanism"):
        cfg = BuildConfig(linkage=key, robot=False)
        note = torque_limit_note(cfg)
        if linkage.get(key).product:    # a hand on a worm box: no servo, no firmware limit
            assert note is None, key
            continue
        assert note is not None, key
        assert f"to {torque_limit_nm(cfg):g} N·m" in note, key
        assert "a blocked output" in note, key
    assert "LINKAGE_TORQUE_LIMITS" in str(torque_limit_note(BuildConfig(linkage=MECH,
                                                                        robot=False)))
    assert "a jammed foot" in str(torque_limit_note(BuildConfig(robot=False)))
    assert torque_limit_note(BuildConfig(linkage=ARM)) is None
    for src in (inspect.getsource(exports._export_files), inspect.getsource(build.main)):
        assert "if note := torque_limit_note(" in src
        assert "mirrored and (note" not in src


_ONE_CRANK = """<mujoco><option timestep="0.001" integrator="implicitfast"/>
<worldbody><body name="crank">
<joint name="t" type="hinge" axis="0 0 1" damping="0.05"/>
<geom type="cylinder" size="0.05 0.005" mass="0.1"/></body></worldbody>
<actuator><velocity name="drive" joint="t" kv="0.3" ctrlrange="-6 6" forcerange="-1.9 1.9"/>
</actuator></mujoco>"""
"""A crank on a damped hinge, its servo's velocity actuator: turning it at the servo's
speed-torque line needs about 0.26 N·m, over a mechanism's limit and under the stall."""


@pytest.mark.no_fabricate
def test_a_fixed_designs_turn_is_bounded_by_its_servos_torque_limit():
    """``sim.run.turn`` drives the crank under the torque limit its servo is set to (the
    model's meta names the servo and linkage: ``config.torque_limit_nm``), not the stall,
    and reports against it: a crank that needs more than the limit saturates on it."""
    pytest.importorskip("mujoco")
    from spiderpig.config import servo_torque_limit_nm
    from spiderpig.sim.run import crank_torque_limit, hold, turn

    cfg = BuildConfig(linkage=MECH, robot=False)
    meta = {"freejoint": None, "config": {"servo": "sts3215", "linkage": MECH},
            "actuators": {"drive": {"type": "velocity", "ctrlrange": [-6.0, 6.0],
                                    "forcerange": [-1.9, 1.9]}}}
    limit = servo_torque_limit_nm("sts3215", MECH)
    assert limit is not None
    assert crank_torque_limit(cfg, meta) == pytest.approx(limit) == 0.21
    t = turn(cfg, model_xml=_ONE_CRANK, model_meta=meta)
    assert t is not None
    assert t["torque_limit"] == pytest.approx(limit)
    assert t["stall"] == pytest.approx(1.9)
    assert t["torque_peak"] <= limit + 1e-9
    assert t["saturates"]                    # it needs ~0.26 N·m: on the limit
    free = turn(cfg, model_xml=_ONE_CRANK, model_meta=meta, torque_limit=1.0)
    assert free is not None
    assert not free["saturates"]
    assert limit < free["torque_peak"] < 0.5
    h = hold(cfg, 0.2, model_xml=_ONE_CRANK, model_meta=meta)
    assert h["torque_fraction"]["drive"] < 1.0      # of the limit: holding needs ~nothing


@pytest.mark.slow     # (fabricates every one-input mechanism's side: ~2 min)
def test_every_mechanisms_default_holds_its_jam():
    """Each mechanism's default at its own torque limit: every joint and link at jam SF 2
    or more (the limits' rule)."""
    from tests import cache

    for key in linkage.available("mechanism"):
        if len(linkage.get(key).inputs) > 1:
            continue                                  # (five_bar: no drive for t2 yet)
        cfg = BuildConfig(linkage=key, robot=False)
        mech = cache.cached_side(cfg)
        st = strength.check(mech.meta.get("wobble") or {}, mech.meta, cfg,
                            strength.design_loads(cfg))
        assert not [f for f in st["findings"] if f["level"] == "error"], key
        sfs = [r["jam"]["safety"] for r in st["rows"] if r.get("jam")]
        assert min(sfs) >= strength.JAM_WARN - 0.05, (key, min(sfs))


@pytest.mark.no_fabricate
def test_a_held_design_carries_its_payload():
    """``sim.holds`` holds an arm with the payload its precision and strength assume (the
    spec's, else ``PAYLOAD_G``); a mechanism with the spec's, else none."""
    from spiderpig.repeatability import PAYLOAD_G
    from spiderpig.verify import hold_payload_g

    def design(key, payload=None):
        return cast(Any, SimpleNamespace(lk=linkage.get(key),
                                         spec=SimpleNamespace(payload_g=payload)))

    assert hold_payload_g(design(ARM)) == PAYLOAD_G
    assert hold_payload_g(design(ARM, 20.0)) == 20.0
    assert hold_payload_g(design(MECH)) == 0.0
    assert hold_payload_g(design(MECH, 5.0)) == 5.0


@pytest.mark.no_fabricate
def test_no_precision_target_no_failure_from_the_budget(monkeypatch):
    """A budget whose model raised fails ``verify`` only when the spec asks for a precision;
    else its row is soft, informational."""
    from spiderpig import api, verify
    from spiderpig.api import accuracy
    from spiderpig.api.reports import PrecisionReport
    from spiderpig.failure import Failure

    broken = PrecisionReport(failures=[Failure("precision", "ValueError", "no budget")])
    monkeypatch.setattr(accuracy, "precision", lambda d: broken)
    monkeypatch.setattr(api, "_cached", lambda *a, **k: None)

    def design(targets):
        return cast(Any, SimpleNamespace(lk=linkage.get(MECH),
                                         spec=SimpleNamespace(precision=targets)))

    rep = verify.VerifyReport("standard")
    (row,) = verify._precision_rows(design({}), rep, "standard")
    assert not row.hard
    assert rep.failures == []
    assert "informational" in row.detail
    rep = verify.VerifyReport("standard")
    (row,) = verify._precision_rows(design({"accuracy_mm": object()}), rep, "standard")
    assert row.hard
    assert rep.failures


@pytest.mark.slow     # (MuJoCo: the mechanism's model fabricated and stepped, ~15 s)
def test_a_mechanism_holds_in_position_and_turns_its_crank():
    """Held: its servo in position mode, no creep under gravity; turned: one turn at full
    speed, its torque well under the torque limit its servo is set to (not the stall)."""
    pytest.importorskip("mujoco")
    from spiderpig.sim.run import hold, turn

    cfg = BuildConfig(linkage=MECH, robot=False)
    h = hold(cfg)
    assert h["holds"]
    assert abs(h["drift"]["drive"]) < 0.05                    # deg: no creep
    t = turn(cfg)
    assert t is not None
    assert t["completed"]
    assert not t["saturates"]
    assert 0 < t["torque_peak"] < 0.5 * t["torque_limit"]
    from spiderpig.config import torque_limit_nm

    assert t["torque_limit"] == pytest.approx(torque_limit_nm(cfg))   # its firmware's
    assert t["stall"] > 4 * t["torque_limit"]
    assert turn(BuildConfig(linkage=ARM, robot=False)) is None    # steppers: nothing to turn


# -- the URDF: by capability -------------------------------------------------------------


@pytest.mark.no_fabricate
def test_the_urdf_refuses_a_crank_with_its_reason():
    from spiderpig import urdf

    assert urdf.unsupported(linkage.get(ARM)) == ""
    why = urdf.unsupported(linkage.get(MECH))
    assert "Linkage.ranged" in why
    assert "crank's full turn" in why
    assert "mjcf" in why
    assert "feet" in urdf.unsupported(linkage.get("strider"))
    with pytest.raises(ValueError, match="has no URDF"):
        urdf.model(cast(Any, SimpleNamespace(bodies=[], meta={})),
                   BuildConfig(linkage=MECH, robot=False), 0.0)


# -- the guide: the arm's cover names its pivots -----------------------------------------


@pytest.mark.no_fabricate
def test_the_arms_cover_names_its_pivots():
    from spiderpig.guide.make import constructions_line

    line = constructions_line(BuildConfig(linkage=ARM))
    assert "ball_pair pins" in line
    assert "ball_pair_pillar pillars" in line
    kit = BuildConfig(linkage=MECH, robot=False, roles=(("controller", "esp32_bus_servo"),))
    assert "esp32_bus_servo controller" in constructions_line(kit)


@pytest.mark.no_fabricate
def test_spiderpig_sim_turns_a_fixed_designs_crank(monkeypatch, capsys):
    """``spiderpig sim`` on a cranked fixed design holds it, then turns its crank: the turn's
    torque printed, exit 1 when the crank stalls."""
    from spiderpig.tools import sim_walk

    held = {"finite": True, "holds": True, "sag": {"output": 0.01}, "sag_max": 0.01,
            "drift": {"drive": 0.0}, "torque": {"drive": 0.001},
            "torque_fraction": {"drive": 0.001}, "loop_error": 0.001}
    turned = {"torque_peak": 0.03, "torque_mean": 0.01, "torque_limit": 1.91,
              "completed": True, "saturates": False, "revolutions": 1.0}
    monkeypatch.setattr(sim_walk, "hold", lambda *a, **k: held)
    monkeypatch.setattr(sim_walk, "turn", lambda *a, **k: turned)
    assert sim_walk.main(["--linkage", MECH]) == 0
    assert "one turn at full speed: peak 0.030 N·m" in capsys.readouterr().out
    monkeypatch.setattr(sim_walk, "turn", lambda *a, **k: dict(
        turned, completed=False, saturates=True, revolutions=0.4))
    assert sim_walk.main(["--linkage", MECH]) == 1
    assert "STALLED at 0.40 of a turn" in capsys.readouterr().out
