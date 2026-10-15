"""Construction groups and the constructions that build them (see :mod:`construction.base`).

Registries map a config key to a construction:

* ``AXLES``: pillars and link pins (:mod:`construction.pivots`: the ``standoff`` pillar
  and the ``chicago`` pin)
* ``CRANKS``: the crankshaft (:mod:`construction.crank`: ``bolt``, and ``bolt_round`` for
  TrotBot's heel and toe)

The keys removed on 2026-10-07 (the printed, rod, bolt, insert and spliced pivots; the
printed, keyed and acrylic cranks) are :data:`config.REMOVED_CONSTRUCTIONS`, each with its
replacement.

Add a construction by implementing ``dims`` (validation + the radii its
claims use) and ``realize`` (parts inside those claims), then registering
it here. Add a kind of group (:class:`construction.base.Group`) by
appending its factory to :data:`GROUP_FACTORIES`: :func:`side_groups`
runs them in that order, which is the groups' dependency order.
"""

from __future__ import annotations

from collections.abc import Callable

from spiderpig.construction.axle import AxleGroup as AxleGroup
from spiderpig.construction.base import ConstructionError, Context, Group
from spiderpig.construction.crank import BOLT_ROUND, BoltCrank
from spiderpig.construction.crank import CrankGroup as CrankGroup
from spiderpig.construction.pivots import PIVOTS
from spiderpig.construction.plates import FramePlates, LinkPlates
from spiderpig.construction.transmissions import TRANSMISSIONS as TRANSMISSIONS
from spiderpig.construction.transmissions import JointDrive

AXLES = {c.key: c for c in PIVOTS}
# ``bolt`` (the default, config.DEFAULT_CRANKS): hex standoff crankpins on single aluminium
# web plates; ``bolt_round`` the same with the friction-clamped round standoff (TrotBot's heel
# and toe, config.LINKAGE_CRANKS)
CRANKS = {c.key: c for c in (BoltCrank(), BOLT_ROUND)}


def _pick(registry: dict, key: str, what: str):
    try:
        return registry[key]
    except KeyError:
        have = sorted(registry)
    raise ConstructionError(f"no {what} construction {key!r}; have {have}") from None


def axle(key: str):
    return _pick(AXLES, key, "axle")


def crank(key: str) -> BoltCrank:
    return _pick(CRANKS, key, "crank")


def transmission(key: str):
    return _pick(TRANSMISSIONS, key, "transmission")


def _servo_drive(ctx: Context, config) -> list[Group]:
    """The servo on the crank (the role ``servo``)."""
    from spiderpig.servos.mount import ServoDrive  # (it imports the constructions)

    return [ServoDrive(ctx.servo)]


def _joint_drives(ctx: Context, config) -> list[Group]:
    """Each input a link turns on a frame joint declares (:attr:`linkage.Linkage.drives`) a
    stepper and a transmission to that link (:mod:`construction.transmissions`; the roles
    ``actuator`` and ``transmission``, per drive): an arm's every input. Keyed on the
    declared drives, not on the inputs' kind, so a cranked linkage whose second input
    declares one (a five-bar's ``t2``) gets its drive here beside the crank's servo."""
    from spiderpig import actuators

    lk = config.lk
    out: list[Group] = []
    for name in (n for n in lk.inputs if n in lk.drives):
        link, about = lk.drives[name]
        out.append(JointDrive(name, link, about,
                              transmission(config.role("transmission", name)),
                              actuators.get(config.role("actuator", name))))
    return out


DRIVES = {"servo": _servo_drive, "actuator": _joint_drives}
"""The drive groups by the motor role that fills them (:data:`roles.MOTOR_ROLES`): a design
takes its own (:func:`roles.motor_role`, from its inputs: a crank's servo, ranged inputs'
actuators; 2026-10-14: a kind test before). A cranked linkage's other inputs that declare a
drive get theirs too (:func:`_joint_drives`)."""


def _drive_groups(ctx: Context, config) -> list[Group]:
    """The drives of the design's motor role (:data:`DRIVES`), and a cranked linkage's
    declared joint drives beside its servo."""
    from spiderpig import roles as rolemod

    role = rolemod.motor_role(config.lk)
    groups = DRIVES[role](ctx, config)
    if role != "actuator" and config.lk.drives:
        groups += _joint_drives(ctx, config)
    return groups


def _crank_groups(ctx: Context, config) -> list[Group]:
    if ctx.topo.center is None:
        return []
    return [CrankGroup(crank(config.crank).resolve(ctx))]


def _axle_groups(ctx: Context, config) -> list[Group]:
    """One axle per pillar (``config.pillar``) and per link pin (``config.pin``)."""
    kinds = {"frame": config.pillar, "pin": config.pin}
    made = {}
    for kind, key in kinds.items():
        a = axle(key)
        made[kind] = a.resolve(ctx) if hasattr(a, "resolve") else a   # per design (a rule)
    return [AxleGroup(ax, made[ax.kind]) for ax in ctx.topo.axes if ax.kind in kinds]


def _plate_groups(ctx: Context, config) -> list[Group]:
    return [LinkPlates(), FramePlates()]


# The groups of a side, in dependency order (``config`` is the :class:`config.BuildConfig`).
GROUP_FACTORIES: list[Callable[[Context, object], list[Group]]] = [
    _drive_groups, _crank_groups, _axle_groups, _plate_groups,
]


def side_groups(ctx: Context, config) -> list[Group]:
    """The groups of one side, in dependency order (the plates last)."""
    return [g for make in GROUP_FACTORIES for g in make(ctx, config)]
