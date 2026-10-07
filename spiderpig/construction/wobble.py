"""Link tilt (wobble) at a pivot: how far a link can tip out of the plane on its axle.

Two limits, per link on an axle (all angles in degrees):

* **free tilt** — the link's hole (or the insert in it) has a diametral clearance
  ``c`` over the shaft and bears along a length ``L`` (the sheet, or an insert's
  body): it can rock until opposite edges of the bore touch the shaft,
  ``atan(c / L)``. A plain 3.2 mm hole on a 3 mm rod in 3 mm sheet: 3.8 deg.
* **supported tilt** — the faces either side of the link (a neighbouring link, a
  spacer ring, a washer, a clip, a head) stop it once it has used up the axial
  play ``g`` of its column: a slab ``t`` thick touching faces of radius ``r`` on
  opposite sides tips until ``t cos(a) + 2 r sin(a) = t + g`` (about
  ``g / 2r`` rad). The whole column's play is assumed to gather at one link
  (the worst case), and the smaller face of the two sides is the one it
  pivots on.

A link's **tilt** is the smaller of the two. At 50 mm from the joint 1 deg is
0.87 mm out of plane. Each construction reports, per joint, its links' tilts and
the numbers behind them in ``Realized.notes["wobble"]``; :mod:`spiderpig.tools.audit`
reports the worst per design. The numbers are the construction's nominal fits:
play set at assembly by feel (a push-on clip, a snug nylock) is an assumption the
construction names (``play_basis``).

The same note carries what the strength check needs: the span between the
outermost links' mid-planes, each link's layer, the frame plates a pillar is glued into
(``anchors``) and the shaft's section (``section``), so the audit turns a pin load into
bending, shear and bearing stresses (:func:`stresses`). The bending case comes from the
layers (:func:`beam`): a **pin** is held by nothing but its links, so two links ``s``
apart carrying ``F`` and ``-F`` bend it by ``M = F s / 2`` (each bore takes half the
couple; ``F s / 4``, the figure before 2026-10-03, assumed both ends held square, which
no pin here is), three links by what their loads leave (a clevis, the middle link against
the outer two: ``F a b / s``); a **pillar** glued into one frame plate is a cantilever
from that plate's face, one glued into both a beam between the faces. With the design's
own link forces (the sim's, :mod:`spiderpig.sim.loads`) the patterns are the measured
ones; with only a load's size, the worst pairs and clevises of :func:`unit_patterns`.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

import numpy as np

DEG = 180.0 / math.pi


def free_tilt_deg(clearance: float, length: float) -> float:
    """Tilt of a bore ``length`` long with diametral ``clearance`` on its shaft."""
    if length <= 0:
        return 90.0
    return math.atan2(max(clearance, 0.0), length) * DEG


def supported_tilt_deg(thickness: float, play: float, face_r: float) -> float:
    """Tilt of a slab ``thickness`` thick between parallel faces ``thickness + play`` apart
    that it touches at radius ``face_r`` (the root of ``t cos a + 2 r sin a = t + g``)."""
    if face_r <= 0:
        return 90.0
    if play <= 0:
        return 0.0
    t, r, g = thickness, face_r, play
    # t cos a + 2 r sin a = R cos(a - phi), R = hypot(t, 2r), phi = atan2(2r, t)
    R, phi = math.hypot(t, 2 * r), math.atan2(2 * r, t)
    if t + g >= R:
        return 90.0
    return (phi - math.acos((t + g) / R)) * DEG


@dataclass(frozen=True)
class Section:
    """A pivot shaft's section where the links bear on it (mm, MPa)."""

    name: str
    bearing_d: float          # diameter the link (or its insert's bore) bears on
    z_bend: float             # elastic section modulus (mm^3)
    a_shear: float            # shear area (mm^2)
    yield_mpa: float          # the shaft's (or tube's) yield strength
    bearing_limit_mpa: float | None = None   # what a liner the links bear on takes (PTFE)

    @classmethod
    def rod(cls, d: float, yield_mpa: float = 215.0, name: str = "rod") -> Section:
        return cls(name, d, math.pi * d ** 3 / 32, math.pi * d * d / 4, yield_mpa)

    @classmethod
    def tube(cls, od: float, id_: float, yield_mpa: float = 215.0,
             name: str = "tube") -> Section:
        return cls(name, od, math.pi * (od ** 4 - id_ ** 4) / (32 * od),
                   math.pi * (od * od - id_ * id_) / 4, yield_mpa)

    def as_dict(self) -> dict:
        out = {"name": self.name, "bearing_d": self.bearing_d,
               "z_bend_mm3": round(self.z_bend, 3), "a_shear_mm2": round(self.a_shear, 3),
               "yield_mpa": self.yield_mpa}
        if self.bearing_limit_mpa is not None:
            out["bearing_limit_mpa"] = self.bearing_limit_mpa
        return out


def link_entry(link: str, *, clearance: float, length: float, thickness: float, play: float,
               face_r: float) -> dict:
    free = free_tilt_deg(clearance, length)
    sup = supported_tilt_deg(thickness, play, face_r)
    return {"link": link, "clearance_mm": round(clearance, 4), "bearing_mm": round(length, 3),
            "play_mm": round(play, 3), "face_r_mm": round(face_r, 3),
            "free_deg": round(free, 3), "supported_deg": round(sup, 3),
            "tilt_deg": round(min(free, sup), 3)}


def column_wobble(build, group, col, *, clearance, length, play: float, play_basis: str,
                  section: Section, link_radius: float | None = None,
                  bearing_len: float | None = None) -> dict:
    """The wobble note of one axle at its solved plan.

    ``clearance(link)`` / ``length(link)``: a link's diametral clearance on the shaft and
    its bearing length (callables, or numbers for every link); ``play``: the column's
    axial play; ``col``: its :class:`construction.pivots.common.Column` (or anything with
    ``links`` and ``roles``, and ``gaps``). A face is what the column holds in the clearance
    gap beside the link where there is one (an end's head, cap or printed head spacer, the
    washers or ring it carries through: their claimed radius; until 2026-10-05 a link next
    to a gap read no face and fell back to its free tilt), else a neighbouring link of the
    same axle (its pad, ``link_radius``), else whatever the column holds in that layer at
    its claimed radius (a ring, a sleeve, a washer, a clip, a head, a frame plate: the
    plate's arm).
    """
    p = build.ctx.params
    pad = p.link_radius if link_radius is None else link_radius
    t = build.ctx.pitch

    gaps = getattr(col, "gaps", None) or {}

    def face(k: int) -> float:
        if k in col.links:
            return pad
        role, r = col.roles.get(k, ("", 0.0))
        if role == "anchor":
            return p.frame_radius
        return min(r, pad)

    def side(k: int, up: int) -> float:
        """The face beside link layer ``k`` above (``up`` 1) or below (-1): what the column
        holds in the clearance gap between them where it has one (an end's head or spacer,
        the washers or ring it carries through), else the next layer's."""
        g = gaps.get(k if up > 0 else k - 1)
        if g is not None and g[1] > 0:
            return min(g[1], pad)
        return face(k + up)

    entries = []
    for k in sorted(col.links):
        for m in col.links[k]:
            c = clearance(m) if callable(clearance) else clearance
            L = length(m) if callable(length) else length
            entries.append(link_entry(m, clearance=c, length=L, thickness=t, play=play,
                                      face_r=min(side(k, -1), side(k, 1))))
    ks = sorted(col.links)
    # every layer's z at the plan (gaps and thicker plates included): the beam's lengths
    # are the stack's own, not ``k x pitch`` (since the clearance gaps of 2026-10-04 a
    # stack is up to twice its layer count x pitch)
    anchors = sorted(getattr(col, "anchors", ()))
    top = getattr(getattr(build, "plan", None), "top", None)
    lo = min([*ks, *anchors], default=0)
    hi = max([*ks, *anchors], default=-1)
    if anchors and top is not None:
        # a pillar's note keeps both frame plates' z (the strength check's "anchored in both
        # plates" fix for a cantilever reads the inner plate's face)
        lo, hi = min(lo, 0), max(hi, top)
    layer_z = {}
    z_of = getattr(build, "z", None)
    if z_of is not None:
        for k in range(lo, hi + 1):
            z0, z1 = z_of(k)
            layer_z[str(k)] = [round(z0, 3), round(z1, 3)]
    if ks and layer_z:
        span = (sum(layer_z[str(ks[-1])]) - sum(layer_z[str(ks[0])])) / 2
    else:
        span = (ks[-1] - ks[0]) * t if ks else 0.0
    return {"links": entries, "play_mm": round(play, 3), "play_basis": play_basis,
            "worst_deg": max((e["tilt_deg"] for e in entries), default=0.0),
            "worst_free_deg": max((e["free_deg"] for e in entries), default=0.0),
            "span_mm": round(span, 3),
            "bearing_len_mm": round(t if bearing_len is None else bearing_len, 3),
            "pitch_mm": round(t, 4),
            "layers": {m: k for k in ks for m in col.links[k]},
            "anchors": anchors,
            "top": top,
            "layer_z": layer_z,
            "section": section.as_dict()}


def layer_mid(note: dict, k: int) -> float:
    """Layer ``k``'s mid-plane (mm from layer 0's bottom face): the plan's z where the note
    has it (``layer_z``), else ``(k + 1/2) x pitch``."""
    lz = (note.get("layer_z") or {}).get(str(k))
    if lz is not None:
        return (lz[0] + lz[1]) / 2
    t = note.get("pitch_mm") or note.get("bearing_len_mm") or 3.0
    return (k + 0.5) * t


def _face(note: dict, k: int, side: int) -> float:
    """Layer ``k``'s upper (``side`` 1) or lower (-1) face, at the plan's z where known."""
    lz = (note.get("layer_z") or {}).get(str(k))
    if lz is not None:
        return lz[1] if side > 0 else lz[0]
    t = note.get("pitch_mm") or note.get("bearing_len_mm") or 3.0
    return (k + (1.0 if side > 0 else 0.0)) * t


def _layout(note: dict) -> tuple[list[str], np.ndarray, tuple]:
    """The links on a joint, their mid-planes (mm, from layer 0) and how the shaft is held:
    ``("free",)`` a pin (only its links hold it), ``("cantilever", z_face, sign)`` a pillar
    glued into one frame plate (``sign``: the side its links are on), ``("simple", z_a,
    z_b)`` one glued into both (taken as simply supported at the plates' inner faces: their
    fixity ignored, conservative). A note without its layers (an older one, or a test's) is
    a two-link pin over its span."""
    t = note.get("pitch_mm") or note.get("bearing_len_mm") or 3.0
    layers = note.get("layers")
    if not layers:
        return ["a", "b"], np.array([0.0, max(note["span_mm"], note["bearing_len_mm"])]), \
            ("free",)
    links = sorted(layers, key=lambda m: (layers[m], m))
    anchors = note.get("anchors") or []
    supports = sorted(note.get("supports") or [])
    if note.get("layer_z"):
        # the plan's z (its gaps and plate thicknesses): each link at its layer's mid-plane,
        # a plate's support at its face toward the links (where the column ends)
        zs = np.array([layer_mid(note, layers[m]) for m in links], dtype=float)
        if len(supports) > 2:
            faces = [_face(note, supports[0], 1),
                     *(_face(note, k, -1) for k in supports[1:-1]), _face(note, supports[-1], -1)]
            return links, zs, ("bays", tuple(faces))
        if len(anchors) >= 2:
            return links, zs, ("simple", _face(note, min(anchors), 1),
                               _face(note, max(anchors), -1))
        if len(anchors) == 1:
            k = anchors[0]
            sign = 1.0 if zs.mean() >= layer_mid(note, k) else -1.0
            return links, zs, ("cantilever", _face(note, k, 1 if sign > 0 else -1), sign)
        return links, zs, ("free",)
    # (a note without the plan's z, an older one or a test's: every layer ``t``, the plates'
    # supports at their mid-planes, as before 2026-10-05)
    zs = np.array([layers[m] * t for m in links], dtype=float)
    if len(supports) > 2:
        # a beam per bay between consecutive supports (each bay simply supported)
        faces = [supports[0] * t + t / 2, *(k * t for k in supports[1:-1]),
                 supports[-1] * t - t / 2]
        return links, zs, ("bays", tuple(faces))
    if len(anchors) >= 2:
        lo, hi = min(anchors), max(anchors)
        return links, zs, ("simple", lo * t + t / 2, hi * t - t / 2)
    if len(anchors) == 1:
        k = anchors[0]
        sign = 1.0 if zs.mean() >= k * t else -1.0
        return links, zs, ("cantilever", k * t + sign * t / 2, sign)
    return links, zs, ("free",)


def bending_case(note: dict) -> str:
    """How the joint's shaft is loaded, in words: ``two-link pin``, ``three-link pin``, ...,
    ``cantilever pillar`` (glued into one plate) or ``pillar between plates``."""
    links, _, support = _layout(note)
    if support[0] == "cantilever":
        return "cantilever pillar"
    if support[0] == "bays":
        return f"pillar in {len(support[1]) - 1} bays"
    if support[0] == "simple":
        return "pillar between plates"
    words = {1: "one", 2: "two", 3: "three", 4: "four", 5: "five"}
    return f"{words.get(len(links), str(len(links)))}-link pin"


def beam(zs: np.ndarray, forces: np.ndarray, support: tuple) -> tuple[float, float]:
    """The largest bending moment (N·mm) and shear force (N) along a shaft carrying the
    in-plane point loads ``forces`` (N, one ``(fx, fy)`` row per link) at the links'
    mid-planes ``zs`` (mm), held as ``support`` says (:func:`_layout`).

    A **pin** has nothing but its links: the loads balance (any residual is spread over
    them) and the couple they leave, ``sum(F_i z_i)``, is taken by the links' bores in
    equal shares, a link bearing across its thickness. Two links, ``F`` against ``-F``
    ``s`` apart, give ``M = F s / 2`` (each bore takes half the couple: the pin is not held
    square at its ends, so ``F s / 4`` would need both ends clamped); a clevis (a middle
    link against two outer ones, half each) ``F a b / s``. A **cantilever pillar**: the
    moment at the plate's face, ``sum(F_i d_i)``; **between plates**: a simply supported
    beam between the faces.
    """
    f = np.atleast_2d(np.asarray(forces, dtype=float))
    order = np.argsort(zs, kind="stable")
    z, f = np.asarray(zs, dtype=float)[order], f[order]
    if support[0] == "free":
        f = f - f.mean(axis=0)
        c = (f * z[:, None]).sum(axis=0) / len(z)        # each link's share of the couple
        m_best = v_best = 0.0
        m = np.zeros(2)
        v = np.zeros(2)
        for i in range(len(z)):
            if i:
                m = m + v * (z[i] - z[i - 1])
            m_best = max(m_best, float(np.hypot(*m)))
            v = v + f[i]
            m = m + c
            m_best = max(m_best, float(np.hypot(*m)))
            v_best = max(v_best, float(np.hypot(*v)))
        return m_best, v_best
    if support[0] == "cantilever":
        zf, sign = support[1], support[2]
        d = np.maximum((z - zf) * sign, 0.0)
        pts = np.concatenate([[0.0], d])
        m_best = max(float(np.hypot(*(f * np.maximum(d - p, 0.0)[:, None]).sum(axis=0)))
                     for p in pts)
        v_best = max(float(np.hypot(*f[d >= p].sum(axis=0))) for p in pts)
        return m_best, v_best
    if support[0] == "bays":
        faces = support[1]
        m_best = v_best = 0.0
        for za, zb in itertools.pairwise(faces):
            inside = (z > za) & (z < zb)
            if inside.any():
                m, v = beam(z[inside], f[inside], ("simple", za, zb))
                m_best, v_best = max(m_best, m), max(v_best, v)
        return m_best, v_best
    za, zb = support[1], support[2]
    span = max(zb - za, 1e-9)
    rb = -(f * (z - za)[:, None]).sum(axis=0) / span
    ra = -f.sum(axis=0) - rb
    m_best = v_best = 0.0
    for p in z:
        m = ra * (p - za) + (f * np.maximum(p - z, 0.0)[:, None]).sum(axis=0)
        m_best = max(m_best, float(np.hypot(*m)))
    for p in np.concatenate([[za], z]):
        v = ra + f[z <= p].sum(axis=0)
        v_best = max(v_best, float(np.hypot(*v)))
    v_best = max(v_best, float(np.hypot(*ra)), float(np.hypot(*rb)))
    return m_best, v_best


def unit_patterns(note: dict) -> list[dict[str, tuple[float, float]]]:
    """The load patterns a joint is checked under when all that is known is the largest
    force one link puts on it (``F``, scaled to 1 here): a pin, every pair of its links
    ``F`` against ``-F`` and every link ``F`` against two others ``-F / 2`` each (a clevis);
    a pillar, each link alone and every link at once, all one way."""
    links, _, support = _layout(note)
    out: list[dict[str, tuple[float, float]]] = []
    if support[0] == "free":
        for i, j in itertools.permutations(range(len(links)), 2):
            out.append({links[i]: (1.0, 0.0), links[j]: (-1.0, 0.0)})
        for i in range(len(links)):
            for j, k in itertools.combinations([x for x in range(len(links)) if x != i], 2):
                out.append({links[i]: (1.0, 0.0), links[j]: (-0.5, 0.0),
                            links[k]: (-0.5, 0.0)})
    else:
        out += [{m: (1.0, 0.0)} for m in links]
        out.append({m: (1.0, 0.0) for m in links})
    return out or [{}]


def moment_per_newton(note: dict, patterns=None) -> tuple[float, float]:
    """The worst (moment N·mm, shear N) per newton of the largest link force over
    ``patterns`` (``{link: (fx, fy)}``; the links' own, :func:`unit_patterns` by default),
    each scaled so its largest link force is 1."""
    links, zs, support = _layout(note)
    if patterns is None:
        patterns = unit_patterns(note)
    idx = {m: i for i, m in enumerate(links)}
    m_best = v_best = 0.0
    for pat in patterns:
        f = np.zeros((len(links), 2))
        for m, v in pat.items():
            if m in idx:
                f[idx[m]] += v
        peak = float(np.hypot(f[:, 0], f[:, 1]).max()) if len(f) else 0.0
        if peak <= 1e-12:
            continue
        m, v = beam(zs, f / peak, support)
        m_best, v_best = max(m_best, m), max(v_best, v)
    return m_best, v_best


def stresses(note: dict, load_n: float, patterns=None) -> dict:
    """Bending, shear and bearing stress (MPa) of a joint whose most loaded link puts
    ``load_n`` on it, and the safety factor on the shaft's yield.

    Bending and shear from :func:`beam` per :func:`_layout`'s case (``case``) over the
    load ``patterns`` (``{link: (fx, fy)}``, the design's own from the sim; else
    :func:`unit_patterns`, the worst ones for a load that size). Bearing: ``F / (d L)`` on
    the shaft (or the insert's or liner's bore). ``safety``: yield over the von Mises
    stress of the bending and shear together, or the liner's limit over the bearing
    pressure where the section has one and that is lower (``governs`` says which).
    """
    s = note["section"]
    m_per, v_per = moment_per_newton(note, patterns)
    moment, shear_n = load_n * m_per, load_n * v_per
    bend = moment / s["z_bend_mm3"]
    shear = shear_n / s["a_shear_mm2"]
    bearing = load_n / (s["bearing_d"] * note["bearing_len_mm"])
    vm = math.hypot(bend, math.sqrt(3) * shear)
    safety, governs = (s["yield_mpa"] / vm, "shaft") if vm > 0 else (math.inf, "shaft")
    limit = s.get("bearing_limit_mpa")
    if limit and bearing > 0 and limit / bearing < safety:
        safety, governs = limit / bearing, "liner bearing"
    return {"bending_mpa": round(bend, 1), "shear_mpa": round(shear, 1),
            "bearing_mpa": round(bearing, 1), "moment_nmm": round(moment, 1),
            "case": bending_case(note), "governs": governs,
            "safety": round(safety, 2) if math.isfinite(safety) else None}
