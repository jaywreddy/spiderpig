# ruff: noqa: E501  (SVG markup in f-strings reads best unbroken)
"""Build docs/architecture/index.html: docs/ARCHITECTURE.md as one page, its ASCII figures
replaced by drawings computed from the code, plus renders from the viewer (img/, made by
shots.py) and the default robot's DXF sheets.

    uv run spiderpig build --out build            # writes build/laser/*.dxf
    uv run python docs/architecture/shots.py      # needs a running viewer server (see there)
    uv run --with markdown-it-py python docs/architecture/build_page.py [--dxf build/laser]
"""
import argparse
import html
import math
import re
from pathlib import Path

import ezdxf
import numpy as np
from markdown_it import MarkdownIt

from spiderpig.linkage import get

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
GH = 'https://github.com/jaywreddy/spiderpig/blob/HEAD/'
DXF_DIR = ROOT / 'build' / 'laser'


def f(v):
    return f'{v:.1f}'.rstrip('0').rstrip('.')


# ---------------------------------------------------------------- Figure 1: the leg
def leg_svg():
    s = get('klann').solve(1, 0.0)
    P = {k: v[0] for k, v in s.evaluate(np.array([1.0])).items()}
    G = {k: v[0] for k, v in s.evaluate(np.array([math.radians(217.3)])).items()}
    ts = np.linspace(0, 2 * np.pi, 240, endpoint=False)
    path = s.evaluate(ts)['F']
    sc, x0, y0, pad = 2.3, -40.0, 108.0, 14
    W = round((262 - x0) * sc + 2 * pad)
    H = round((y0 + 112) * sc + 2 * pad)

    def X(x):
        return pad + (x - x0) * sc

    def Y(y):
        return pad + (y0 - y) * sc

    def pt(q):
        return f'{f(X(q[0]))},{f(Y(q[1]))}'

    o = [f'<svg viewBox="0 0 {W} {H}" role="img" class="dia" '
         'aria-label="The Klann leg drawn to scale at crank angle 1 rad: fixed pivots O, A, B; '
         'crank O-M; links b1 to b4; the loop the foot F follows; and b1 lying across O at 217 degrees.">']
    r = 24.72
    o.append(f'<circle cx="{f(X(0))}" cy="{f(Y(0))}" r="{f(r * sc)}" class="guide"/>')
    o.append('<polyline class="fp" points="' + ' '.join(pt(q) for q in list(path) + [path[0]]) + '"/>')
    # ghost of b1 and the crank at 217 degrees: the moment b1 lies across O
    o.append(f'<line x1="{f(X(G["M"][0]))}" y1="{f(Y(G["M"][1]))}" x2="{f(X(G["D"][0]))}" '
             f'y2="{f(Y(G["D"][1]))}" class="ghost" stroke-width="{f(12 * sc)}"/>')
    o.append(f'<line x1="{f(X(0))}" y1="{f(Y(0))}" x2="{f(X(G["M"][0]))}" y2="{f(Y(G["M"][1]))}" '
             'class="ghost-crank"/>')
    o.append(f'<text x="{f(X(0))}" y="{f(Y(-r) + 32)}" class="lbl" '
             'text-anchor="middle">dashed: b1 at 217°</text>')
    for a, b in [('E', 'F'), ('A', 'C'), ('B', 'E'), ('M', 'D')]:   # b4, b3, b2, b1
        o.append(f'<line x1="{f(X(P[a][0]))}" y1="{f(Y(P[a][1]))}" x2="{f(X(P[b][0]))}" '
                 f'y2="{f(Y(P[b][1]))}" class="lk" stroke-width="{f(12 * sc)}"/>')
        o.append(f'<line x1="{f(X(P[a][0]))}" y1="{f(Y(P[a][1]))}" x2="{f(X(P[b][0]))}" '
                 f'y2="{f(Y(P[b][1]))}" class="lk-c"/>')
    o.append(f'<line x1="{f(X(0))}" y1="{f(Y(0))}" x2="{f(X(P["M"][0]))}" y2="{f(Y(P["M"][1]))}" '
             f'class="ck" stroke-width="{f(12 * sc)}"/>')
    # link labels at a point along each link, nudged off the bar
    lab = {'b1': ('M', 'D', 0.42, (0, -20)), 'b2': ('B', 'E', 0.5, (-4, -18)),
           'b3': ('A', 'C', 0.5, (16, 4)), 'b4': ('E', 'F', 0.62, (20, 0))}
    for name, (a, b, u, (dx, dy)) in lab.items():
        q = P[a] + u * (P[b] - P[a])
        o.append(f'<text x="{f(X(q[0]) + dx)}" y="{f(Y(q[1]) + dy)}" class="lbl lk-lbl" '
                 f'text-anchor="middle">{name}</text>')
    for k in 'OABMCDEF':
        x, y = X(P[k][0]), Y(P[k][1])
        if k in 'OAB':
            o.append(f'<polygon points="{f(x)},{f(y + 4)} {f(x - 7)},{f(y + 14)} {f(x + 7)},{f(y + 14)}" '
                     'class="fixed"/>')
            o.append(f'<line x1="{f(x - 10)}" y1="{f(y + 14)}" x2="{f(x + 10)}" y2="{f(y + 14)}" class="fixed-l"/>')
        o.append(f'<circle cx="{f(x)}" cy="{f(y)}" r="4.5" class="jt"/>')
        off = {'O': (-17, 5), 'A': (-14, 4), 'B': (-14, -6), 'M': (-10, -12), 'C': (2, -12),
               'D': (12, -10), 'E': (10, -10), 'F': (14, 6)}[k]
        o.append(f'<text x="{f(x + off[0])}" y="{f(y + off[1])}" class="lbl jt-lbl" '
                 'text-anchor="middle">' + k + '</text>')
    lo = path[np.argmin(path[:, 1])]
    o.append(f'<text x="{f(X(lo[0]) - 70)}" y="{f(Y(lo[1]) + 22)}" class="lbl fp-lbl" '
             'text-anchor="middle">foot path (stance along the bottom)</text>')
    o.append(f'<text x="{f(X(0))}" y="{f(Y(-r) + 16)}" class="lbl mut" text-anchor="middle">crank circle</text>')
    # scale bar: 50 mm
    sx, sy = X(200), Y(95)
    o.append(f'<line x1="{f(sx)}" y1="{f(sy)}" x2="{f(sx + 50 * sc)}" y2="{f(sy)}" class="scale"/>')
    o.append(f'<text x="{f(sx + 25 * sc)}" y="{f(sy - 7)}" class="lbl mut" text-anchor="middle">50 mm</text>')
    o.append('</svg>')
    return '\n'.join(o)


# ------------------------------------------------- Figure 2: the crank, seen from the side
def crank_side_svg():
    """The crank and b1 of one Klann leg, to scale in x at t = 217.3 deg (b1 across O);
    height exaggerated 3x."""
    s = get('klann').solve(1, 0.0)
    G = {k: v[0] for k, v in s.evaluate(np.array([math.radians(217.3)])).items()}
    mx = float(G['M'][0])
    dx_ = float(G['D'][0])
    sx, sz = 4.6, 13.8            # px per mm across, px per mm up (3x)
    xmin, xmax = -40.0, 92.0
    padl, padr, padt, padb = 64, 250, 70, 30
    L = 3.0
    nl = 7
    W = round(padl + (xmax - xmin) * sx + padr)
    H = round(padt + nl * L * sz + padb)

    def X(x):
        return padl + (x - xmin) * sx

    def Z(z):
        return padt + (nl * L - z) * sz

    def band(layer, x0, x1, cls, z0=None, z1=None):
        za = layer * L if z0 is None else z0
        zb = (layer + 1) * L if z1 is None else z1
        return (f'<rect x="{f(X(x0))}" y="{f(Z(zb))}" width="{f((x1 - x0) * sx)}" '
                f'height="{f((zb - za) * sz)}" class="{cls}"/>')

    o = [f'<svg viewBox="0 0 {W} {H}" role="img" class="dia" '
         'aria-label="One Klann leg seen from the side at the moment b1 lies across O: the crank leaves O '
         'with a web in layer 1, runs along the post at M through layer 2, and returns with a web in layer 3; '
         'one screw clamps the chain; the hub couples it to the servo horn; the stub turns in the outer plate.">']
    for k in range(nl):
        o.append(f'<line x1="{padl - 6}" y1="{f(Z(k * L))}" x2="{f(W - padr + 6)}" y2="{f(Z(k * L))}" class="grid"/>')
        o.append(f'<text x="{padl - 12}" y="{f(Z(k * L + 1.5) + 4)}" class="lbl mut num" text-anchor="end">{k}</text>')
    o.append(f'<text x="{padl - 12}" y="{padt - 14}" class="lbl mut" text-anchor="end">layer</text>')
    # frame plates
    o.append(band(0, -30, 85, 'plate'))
    o.append(band(6, -30, 85, 'plate'))
    # servo above the inner plate
    o.append(f'<rect x="{f(X(-24))}" y="{f(Z(7 * L) - 44)}" width="{f(68 * sx)}" height="40" rx="3" class="servo"/>')
    o.append(f'<text x="{f(X(10))}" y="{f(Z(7 * L) - 19)}" class="lbl inv" text-anchor="middle">servo (output face down)</text>')
    # O axis
    o.append(f'<line x1="{f(X(0))}" y1="{padt - 50}" x2="{f(X(0))}" y2="{f(Z(0) + 14)}" class="axis"/>')
    o.append(f'<text x="{f(X(0))}" y="{f(Z(0) + 26)}" class="lbl" text-anchor="middle">O</text>')
    o.append(f'<text x="{f(X(mx))}" y="{f(Z(0) + 26)}" class="lbl" text-anchor="middle">M</text>')
    # b1 in layer 2, across O
    o.append(band(2, mx - 6, dx_ + 6, 'link-b'))
    # crank pieces (printed)
    o.append(band(0, -4, 4, 'crank'))                 # stub in the outer plate
    o.append(band(1, mx - 6, 6, 'crank'))             # journal + lower web
    o.append(band(2, mx - 3, mx + 3, 'crank'))        # post at M
    o.append(band(3, mx - 6, 6, 'crank'))             # journal + upper web
    o.append(band(4, -11, 11, 'crank'))               # hub
    o.append(band(5, -11, 11, 'crank'))
    o.append(band(6, -11, 11, 'horn'))                # horn in the plate's hole
    # screw through the post: head in the lower web, nut in the upper web
    o.append(f'<line x1="{f(X(mx))}" y1="{f(Z(1 * L + 0.3))}" x2="{f(X(mx))}" y2="{f(Z(4 * L - 0.3))}" class="screw"/>')
    o.append(band(1, mx - 2.8, mx + 2.8, 'screw-h', 1 * L, 1 * L + 1.65))
    o.append(band(3, mx - 2.75, mx + 2.75, 'screw-h', 4 * L - 2.4, 4 * L - 0.3))
    # labels on the right
    rx = W - padr + 14
    for z, t in [(0.5, 'stub turns in the outer plate'), (1.5, 'journal + lower web, screw head'),
                 (2.5, 'b1 turns on the post at M; nothing on O'), (3.5, 'journal + upper web, nut'),
                 (4.5, 'hub'), (5.5, 'hub'), (6.5, 'horn, in the plate')]:
        o.append(f'<text x="{rx}" y="{f(Z(z * L) + 4)}" class="lbl">{t}</text>')
    o.append('</svg>')
    return '\n'.join(o)


# ------------------------------------------------- Figure 3: the running example's crank
def quad_crank_svg():
    nl = 12
    rowh = 27
    padl, padt, padb = 64, 30, 34
    colx = {'O': 330, 'R': 440, 'Lf': 220}
    W = 760
    H = padt + nl * rowh + padb

    def Z(layer):           # top of a layer row
        return padt + (nl - 1 - layer) * rowh

    def rect(x0, x1, layer, cls, frac0=0.0, frac1=1.0):
        y = Z(layer) + (1 - frac1) * rowh
        return f'<rect x="{x0}" y="{f(y)}" width="{x1 - x0}" height="{f((frac1 - frac0) * rowh)}" class="{cls}"/>'

    pins = {0: ('R', 'M0'), 1: ('Lf', 'M1'), 2: ('R', 'M2'), 3: ('Lf', 'M3')}
    rider = {0: 2, 1: 4, 2: 6, 3: 8}
    o = [f'<svg viewBox="0 0 {W} {H}" role="img" class="dia" '
         'aria-label="The running example crankshaft through 12 layers: four runs, one per leg, at layers 2, 4, '
         '6 and 8; journals and webs in layers 1, 3, 5, 7 and 9; the hub in 9 and 10; five printed segments; '
         'four screws.">']
    for k in range(nl):
        o.append(f'<line x1="{padl - 6}" y1="{Z(k) + rowh}" x2="{W - 190}" y2="{Z(k) + rowh}" class="grid"/>')
        o.append(f'<text x="{padl - 12}" y="{Z(k) + rowh / 2 + 4}" class="lbl mut num" text-anchor="end">{k}</text>')
    o.append(rect(90, 570, 0, 'plate'))
    o.append(rect(90, 570, 11, 'plate'))
    o.append(f'<line x1="{colx["O"]}" y1="{padt - 10}" x2="{colx["O"]}" y2="{H - padb + 8}" class="axis"/>')
    o.append(f'<text x="{colx["O"]}" y="{H - 8}" class="lbl" text-anchor="middle">O</text>')
    # riders across O
    for leg, layer in rider.items():
        side, name = pins[leg]
        cx = colx[side]
        x0, x1 = (colx['O'] - 70, cx + 18) if side == 'R' else (cx - 18, colx['O'] + 70)
        o.append(rect(x0, x1, layer, 'link-b', 0.2, 0.8))
        o.append(rect(cx - 7, cx + 7, layer, 'crank'))
        lx = x1 + 6 if side == 'R' else x0 - 6
        anchor = 'start' if side == 'R' else 'end'
        o.append(f'<text x="{lx}" y="{Z(layer) + rowh / 2 + 4}" class="lbl" text-anchor="{anchor}">b1 of leg {leg}</text>')
    # webs and journals
    webs = {1: [0], 3: [0, 1], 5: [1, 2], 7: [2, 3], 9: [3]}
    for layer, legs in webs.items():
        xs = [colx['O']] + [colx[pins[g][0]] for g in legs]
        o.append(rect(min(xs) - 12, max(xs) + 12, layer, 'crank'))
    o.append(rect(colx['O'] - 30, colx['O'] + 30, 9, 'crank'))
    o.append(rect(colx['O'] - 30, colx['O'] + 30, 10, 'crank'))
    o.append(rect(colx['O'] - 8, colx['O'] + 8, 0, 'crank'))
    o.append(rect(colx['O'] - 30, colx['O'] + 30, 11, 'horn'))
    # screws: one per chain, head in the lower web, nut in the upper web
    for leg, layer in rider.items():
        cx = colx[pins[leg][0]]
        o.append(f'<line x1="{cx}" y1="{Z(layer - 1) + rowh - 3}" x2="{cx}" y2="{Z(layer + 1) + 3}" class="screw"/>')
        o.append(f'<rect x="{cx - 6}" y="{Z(layer - 1) + rowh - 7}" width="12" height="6" class="screw-h"/>')
        o.append(f'<rect x="{cx - 6}" y="{Z(layer + 1) + 1}" width="12" height="7" class="screw-h"/>')
    for side, name in [('R', 'M0, M2'), ('Lf', 'M1, M3')]:
        o.append(f'<text x="{colx[side]}" y="{H - 8}" class="lbl" text-anchor="middle">{name}</text>')
    # printed segments, bracketed on the right
    bx = W - 175
    for i, (a, b) in enumerate([(0, 1), (3, 3), (5, 5), (7, 7), (9, 10)], 1):
        y0, y1 = Z(b) + 3, Z(a) + rowh - 3
        o.append(f'<path d="M{bx},{y0} h6 V{y1} h-6" class="bracket"/>')
        lay = f'layer {a}' if a == b else f'layers {a}–{b}'
        o.append(f'<text x="{bx + 14}" y="{f((y0 + y1) / 2 + 4)}" class="lbl">segment {i}: {lay}</text>')
    o.append(f'<text x="{colx["O"]}" y="{Z(11) + rowh / 2 + 4}" class="lbl inv" text-anchor="middle">horn</text>')
    o.append(f'<text x="{colx["O"]}" y="{Z(10) + rowh / 2 + 4}" class="lbl inv" text-anchor="middle">hub</text>')
    o.append('</svg>')
    return '\n'.join(o)


# ------------------------------------------------- the DXF sheets
def dxf_svg():
    d = str(DXF_DIR) + '/'
    parts = []
    gap = 24
    S = 300
    W = 2 * S + gap
    for i, name in enumerate(['klann_sheet_0.dxf', 'klann_sheet_1.dxf']):
        ox = i * (S + gap)
        doc = ezdxf.readfile(d + name)
        g = [f'<rect x="{ox}" y="0" width="{S}" height="{S}" class="sheet"/>']
        for e in doc.modelspace():
            if e.dxftype() == 'LWPOLYLINE':
                pts = [(p[0], p[1]) for p in e.get_points('xy')]
                dd = 'M' + ' L'.join(f'{f(ox + x)},{f(S - y)}' for x, y in pts) + 'Z'
                g.append(f'<path d="{dd}" class="cut"/>')
            elif e.dxftype() == 'CIRCLE':
                c = e.dxf.center
                g.append(f'<circle cx="{f(ox + c.x)}" cy="{f(S - c.y)}" r="{f(e.dxf.radius)}" class="cut"/>')
        g.append(f'<text x="{ox + S / 2}" y="{S + 16}" class="lbl mut" text-anchor="middle">'
                 f'sheet {i + 1}, 300 × 300 mm</text>')
        parts += g
    return (f'<svg viewBox="-2 -2 {W + 4} {S + 24}" role="img" class="dia dxf" '
            'aria-label="The two laser-cutting sheets of the running example, drawn from the DXF files: '
            '39 parts, frame plates, centre plates and links, packed onto two 300 mm sheets.">'
            + ''.join(parts) + '</svg>')


# ------------------------------------------------- charts
def hbar_chart(rows, unit, ticks, label, width=640, value_fmt=None, labw=200):
    """Single-series horizontal bars: rows = [(label, value, tip)]."""
    rowh, bar = 38, 22
    padt, padb, padr = 8, 30, 70
    W = width
    H = padt + len(rows) * rowh + padb
    vmax = ticks[-1]
    plot = W - labw - padr

    def X(v):
        return labw + v / vmax * plot

    o = [f'<svg viewBox="0 0 {W} {H}" role="img" class="chart" aria-label="{html.escape(label)}">']
    for t in ticks:
        o.append(f'<line x1="{f(X(t))}" y1="{padt}" x2="{f(X(t))}" y2="{H - padb + 4}" class="cgrid"/>')
        tl = f'{t:g} {unit}' if t == ticks[-1] else f'{t:g}'
        o.append(f'<text x="{f(X(t))}" y="{H - padb + 18}" class="clbl num" text-anchor="middle">{tl}</text>')
    for i, (lab, v, tip) in enumerate(rows):
        y = padt + i * rowh + (rowh - bar) / 2
        x0, x1 = X(0), X(v)
        w = max(x1 - x0, 2)
        r = min(4, w / 2)
        dpath = (f'M{f(x0)},{f(y)} H{f(x0 + w - r)} Q{f(x0 + w)},{f(y)} {f(x0 + w)},{f(y + r)} '
                 f'V{f(y + bar - r)} Q{f(x0 + w)},{f(y + bar)} {f(x0 + w - r)},{f(y + bar)} H{f(x0)} Z')
        txt = value_fmt(v) if value_fmt else f'{v:g}'
        o.append(f'<g class="mark" tabindex="0" data-tip="{html.escape(tip)}">'
                 f'<rect x="0" y="{f(y - 7)}" width="{W}" height="{bar + 14}" class="hit"/>'
                 f'<path d="{dpath}" class="bar"/></g>')
        o.append(f'<text x="{labw - 10}" y="{f(y + bar / 2 + 4)}" class="clbl2" text-anchor="end">{html.escape(lab)}</text>')
        o.append(f'<text x="{f(x0 + w + 6)}" y="{f(y + bar / 2 + 4)}" class="cval num">{txt}</text>')
    o.append(f'<line x1="{f(X(0))}" y1="{padt}" x2="{f(X(0))}" y2="{H - padb + 4}" class="cbase"/>')
    o.append('</svg>')
    return '\n'.join(o)


RAMP = ['#cde2fb', '#b7d3f6', '#9ec5f4', '#86b6ef', '#6da7ec', '#5598e7', '#3987e5', '#2a78d6',
        '#256abf', '#1c5cab', '#184f95', '#104281']


def heat_table():
    rows = [('Klann', [7, 8, 11, 12]), ('Jansen', [8, 9, 13, 16]), ('four-bar', [6, 7, 10, 14]),
            ('six-bar', [9, 12, 14, ('24–25', 24.5, True)]), ('Strider', [10, 16, 16, ('28', 28, True)]),
            ('TrotBot', [12, 12, 20, ('≈36, or no plan', 36, True)])]
    lo, hi = 6, 36

    def cell(v):
        if isinstance(v, tuple):
            txt, val, unproven = v
        else:
            txt, val, unproven = str(v), v, False
        k = round((val - lo) / (hi - lo) * (len(RAMP) - 1))
        bg = RAMP[k]
        ink = '#0b0b0b' if k <= 5 else '#ffffff'
        cls = ' class="unproven"' if unproven else ''
        mark = '<span class="star">*</span>' if unproven else ''
        tip = f'{txt} layers ({float(val) * 3:g} mm)' + (', unproven: the 60 s deadline ran out' if unproven else ', proven thinnest')
        return (f'<td{cls} style="background:{bg};color:{ink}" tabindex="0" data-tip="{html.escape(tip)}">'
                f'{html.escape(txt)}{mark}</td>')

    out = ['<div class="table-wrap"><table class="heat"><thead><tr><th>linkage</th>'
           '<th><code>single</code></th><th><code>double</code></th><th><code>decker</code></th>'
           '<th><code>quad</code></th></tr></thead><tbody>']
    for name, vals in rows:
        out.append(f'<tr><th scope="row">{name}</th>' + ''.join(cell(v) for v in vals) + '</tr>')
    out.append('</tbody></table></div>')
    out.append('<p class="note">Layers per side; each layer is 3 mm. Darker means taller. '
               '<span class="star">*</span> unproven: the 60 s deadline ran out before the search could '
               'rule out thinner stacks.</p>')
    return ''.join(out)


# ------------------------------------------------- markdown -> html
def slug(text):
    t = text.strip().lower()
    t = re.sub(r'[^\w\- ]', '', t)
    return t.replace(' ', '-')


def figure(inner, caption_html, cls=''):
    return f'<figure class="fig {cls}">{inner}<figcaption>{caption_html}</figcaption></figure>'


def img(name, alt, w, h):
    return (f'<div class="shot"><img src="img/{name}.webp" alt="{html.escape(alt)}" width="{w}" '
            f'height="{h}" loading="lazy"></div>')


def build():
    md_src = (ROOT / 'docs' / 'ARCHITECTURE.md').read_text()
    md = MarkdownIt('commonmark', {'html': False, 'typographer': False}).enable('table')

    figs = {}

    def cap_and_block(n):
        nonlocal md_src
        m = re.search(r'\*\*Figure ' + str(n) + r'\.\*\*(.*?)\n\n```\n(.*?)\n```\n', md_src, re.S)
        assert m, n
        caption = md.renderInline('**Figure ' + str(n) + '.**' + m.group(1).replace('\n', ' '))
        md_src = md_src[:m.start()] + f'@@FIG{n}@@\n\n' + md_src[m.end():]
        return caption, m.group(2)

    c1, _ = cap_and_block(1)
    figs[1] = figure(leg_svg(), c1 + ' Drawn from the compiled program; the dashed bar is the crank '
                     'rider b1 a little later in the turn, at 217°, lying right across O.')
    c2, block2 = cap_and_block(2)
    rows = []
    for line in block2.split('\n'):
        mm = re.match(r'layer (-?\d+)\s+(.*)', line.strip())
        if mm and not line.strip().startswith('layer 6   ======') and '[' not in line and '<' not in line:
            rows.append((mm.group(1), mm.group(2)))
    seen = {}
    for k, v in rows:
        seen.setdefault(k, v)
    trs = ''.join(f'<tr><th scope="row" class="num">{html.escape(k)}</th><td>{html.escape(v)}</td></tr>'
                  for k, v in seen.items())
    table2 = ('<div class="table-wrap"><table class="layers"><thead><tr><th>layer</th><th>what it holds</th>'
              f'</tr></thead><tbody>{trs}</tbody></table></div>')
    figs[2] = figure(table2 + crank_side_svg(),
                     c2 + ' Below the table, the crank and its rider seen from the side, drawn to scale across '
                     '(height exaggerated three times) at the moment b1 lies across O: in layer 2 nothing of '
                     'the crank can sit on O, so the crankshaft runs along the post at M.')
    c3, _ = cap_and_block(3)
    figs[3] = figure(quad_crank_svg(),
                     c3 + ' Schematic: crankpins drawn to the left or right of O (M2 and M3 are really a quarter '
                     'turn from M0 and M1). Each screw runs from its head in the lower web to a nut in the upper.')

    # placeholders for images and charts, inserted after known sentences
    def after(anchor_regex, token):
        nonlocal md_src
        m = re.search(anchor_regex, md_src, re.S | re.I)
        assert m, anchor_regex
        end = md_src.find('\n\n', m.end())
        md_src = md_src[:end] + f'\n\n{token}' + md_src[end:]

    after(r'one screw runs up through the post.*?together\.', '@@IMG_SINGLE@@')
    after(r'adds the chassis\. The side is planned\s+once: both sides use the same plan\.', '@@IMG_FRONT@@')
    after(r'Round\s+holes are written as exact circles; every other outline is a closed polyline of 96\s+points\.',
          '@@DXF@@')
    after(r'\| `7_serialize` \| write the file \| under 1 s \|', '@@BAKECHART@@')
    after(r'servo\'s 52 rpm, and 23\.7 mm of \*\*bob\*\*', '@@IMG_SIDE@@')
    after(r'\| kinematic gait \| the body rides the lowest foot \| 296 mm \|', '@@WALKCHART@@')
    # the plan-size table in 5.6 becomes a heat table
    m = re.search(r'Layers per side \(unproven results marked \*\):\n\n(\|.*?\n)\n', md_src, re.S)
    assert m
    md_src = md_src[:m.start()] + '@@HEAT@@\n\n' + md_src[m.end():]
    # the inline contents line goes: the page has its own
    md_src = re.sub(r'\*\*Contents\.\*\*\n(\[.*?\n)+', '', md_src)

    head, body = md_src.split('## About this report', 1)
    body = '## About this report' + body

    def render(src):
        tokens = md.parse(src)
        for i, t in enumerate(tokens):
            if t.type == 'heading_open':
                text = tokens[i + 1].content
                t.attrSet('id', slug(text))
            if t.type == 'inline' and t.children:
                for c in t.children:
                    if c.type == 'link_open':
                        h = c.attrGet('href')
                        if h and not h.startswith(('#', 'http')):
                            p = h[3:] if h.startswith('../') else 'docs/' + h
                            c.attrSet('href', GH + p)
                            c.attrSet('target', '_blank')
                            c.attrSet('rel', 'noopener')
        return md.renderer.render(tokens, md.options, {})

    out_body = render(body)
    out_head = render(head)

    # tables scroll in their own box
    out_body = out_body.replace('<table>', '<div class="table-wrap"><table>').replace('</table>', '</table></div>')

    walk = hbar_chart([('quasi-static model', 102.4, 'quasi-static model: 102.4 mm per crank turn (feet never slide)'),
                       ('MuJoCo simulation', 192, 'MuJoCo: 192 mm per crank turn, with heavy foot slip'),
                       ('kinematic gait', 296, 'kinematic gait: 296 mm per crank turn (body rides the lowest foot)')],
                      'mm', [0, 100, 200, 300], 'Travel per crank turn of the running example by three estimates',
                      value_fmt=lambda v: f'{v:g} mm')
    bake = hbar_chart([('fabricate at t = 0', 16.1, '1_reference_build: 16.1 s, 56 % of the bake'),
                       ('mesh with OCCT', 9.7, '2_tessellate_total: 9.7 s, 34 %'),
                       ('share meshes', 1.7, '2_mesh_share: 1.7 s'),
                       ('nodes and channels', 0.73, '5_gltf_nodes_channels: 0.73 s'),
                       ('everything else', 0.4, 'packing, animation sampling, extras and writing: about 0.4 s')],
                      's', [0, 5, 10, 15, 20], 'Where the 28.6 s bake of the running example goes',
                      value_fmt=lambda v: f'{v:g} s')

    subs = {
        '<p>@@FIG1@@</p>': figs[1],
        '<p>@@FIG2@@</p>': figs[2],
        '<p>@@FIG3@@</p>': figs[3],
        '<p>@@IMG_SINGLE@@</p>': figure(img('single_three_quarter', 'One Klann leg as built, in the viewer', 1330, 1203),
                                         'One Klann leg as built, rendered by spiderpig\'s own viewer from the baked '
                                         'glb: the orange frame plates hold the pillars, the links are translucent '
                                         'blue acrylic, and the printed axles and crank are violet. The red loop is '
                                         'the foot path.', 'narrow'),
        '<p>@@IMG_FRONT@@</p>': figure(img('robot_front', 'The running example from the front', 662, 964),
                                        'The running example from the front: two 12-layer side stacks, mirror images, '
                                        'with the two servos back to back between the centre plates and the printed '
                                        'tie columns in the middle.', 'narrow'),
        '<p>@@IMG_SIDE@@</p>': figure(img('robot_side', 'The running example from the side', 1400, 736),
                                       'The running example from the side, in the viewer. Four legs on this side '
                                       'are spread a quarter turn apart, so some feet are always on the ground. The '
                                       'red loop is one foot\'s path.'),
        '<p>@@DXF@@</p>': figure(dxf_svg(), 'The running example\'s two laser-cutting sheets, drawn straight from the '
                                 'DXF files <code>spiderpig build</code> wrote: 39 parts (frame plates, centre '
                                 'plates and 32 links), every hole a true circle, every outline a 96-point '
                                 'polyline.'),
        '<p>@@BAKECHART@@</p>': figure(bake, 'Where the bake\'s 28.6 s go: fabricating the robot again and meshing it '
                                       'take nine tenths; the animation itself is negligible.', 'chart-fig'),
        '<p>@@WALKCHART@@</p>': figure(walk, 'The three estimates of how far the running example travels per crank '
                                       'turn differ by a factor of three. No measurement of a built robot exists '
                                       'to settle which is right.', 'chart-fig'),
        '<p>@@HEAT@@</p>': heat_table(),
    }
    for k, v in subs.items():
        assert k in out_body, k
        out_body = out_body.replace(k, v)

    # table of contents from h2/h3
    toc = []
    for m in re.finditer(r'<h([23]) id="([^"]+)">(.*?)</h\1>', out_body):
        lvl, hid, text = m.groups()
        text = re.sub('<[^>]+>', '', text)
        toc.append((int(lvl), hid, text))
    toc_html = ['<ol class="toc">']
    open_sub = False
    for lvl, hid, text in toc:
        if lvl == 2:
            if open_sub:
                toc_html.append('</ol></li>')
                open_sub = False
            toc_html.append(f'<li><a href="#{hid}">{text}</a>')
            nxt = toc[toc.index((lvl, hid, text)) + 1:toc.index((lvl, hid, text)) + 2]
            if nxt and nxt[0][0] == 3:
                toc_html.append('<ol>')
                open_sub = True
            else:
                toc_html.append('</li>')
        else:
            toc_html.append(f'<li><a href="#{hid}">{text}</a></li>')
    if open_sub:
        toc_html.append('</ol></li>')
    toc_html.append('</ol>')

    # hero from the head
    hm = re.search(r'<h1[^>]*>(.*?)</h1>\s*<p>(.*?)</p>\s*<p>(.*?)</p>\s*<p><strong>In short\.</strong></p>\s*(<ul>.*?</ul>)',
                   out_head, re.S)
    assert hm
    title, lead, meta, short = hm.groups()

    tpl = (HERE / 'template.html').read_text()
    page = (tpl.replace('{{LEAD}}', lead).replace('{{META}}', meta).replace('{{SHORT}}', short)
            .replace('{{TOC}}', '\n'.join(toc_html)).replace('{{BODY}}', out_body))
    head_part, _, body_part = page.partition('<div class="page">')
    standalone = ('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
                  '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
                  + head_part + '</head>\n<body>\n<div class="page">' + body_part + '\n</body>\n</html>\n')
    (HERE / 'index.html').write_text(standalone)
    print('wrote', HERE / 'index.html', len(page), 'bytes')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--dxf', type=Path, default=DXF_DIR, help='folder of the default build\'s DXF sheets')
    DXF_DIR = ap.parse_args().dxf
    build()
