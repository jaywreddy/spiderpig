"""The guide's PDF, laid out with matplotlib (in the lock already: cadquery-ocp -> vtk ->
matplotlib), A4 landscape: a cover, the parts inventory, a page per step.

Deterministic: no creation date, TrueType fonts embedded (``pdf.fonttype`` 42), images
lossless (Flate).
"""

from __future__ import annotations

import textwrap
from pathlib import Path

import numpy as np

A4 = (11.69, 8.27)
INK = "#18181c"
ACCENT = "#f58c28"


def _fig():
    from matplotlib.figure import Figure

    fig = Figure(figsize=A4)          # not pyplot's: nothing kept once saved
    fig.patch.set_facecolor("white")
    return fig


def _image(fig, img, rect):
    ax = fig.add_axes(rect)
    ax.imshow(np.asarray(img), interpolation="none")
    ax.set_axis_off()
    return ax


def _footer(fig, text: str, page: int):
    fig.text(0.03, 0.025, text, fontsize=7, color="#888")
    fig.text(0.97, 0.025, str(page), fontsize=9, color="#555", ha="right")


def write(path: Path, *, title: str, subtitle: list[str], cover, types, thumbs: dict,
          steps, images: dict, footer: str) -> int:
    """Write the guide; returns its page count."""
    import matplotlib
    from matplotlib.backends.backend_pdf import PdfPages
    from matplotlib.patches import FancyBboxPatch

    matplotlib.rcParams.update({"pdf.fonttype": 42, "font.family": "DejaVu Sans",
                                "svg.hashsalt": "spiderpig"})
    page = 0
    with PdfPages(path, metadata={"CreationDate": None, "ModDate": None,
                                  "Creator": "spiderpig guide", "Producer": None,
                                  "Title": title}) as pdf:
        # cover
        fig = _fig()
        page += 1
        fig.text(0.05, 0.9, title, fontsize=28, weight="bold", color=INK)
        for i, line in enumerate(subtitle):
            fig.text(0.05, 0.84 - i * 0.035, line, fontsize=11, color="#444")
        _image(fig, cover, [0.25, 0.06, 0.72, 0.66])
        _footer(fig, footer, page)
        pdf.savefig(fig)
        fig.clf()
        # inventory: 6 x 4 per page
        per, cols = 24, 6
        for start in range(0, len(types), per):
            fig = _fig()
            page += 1
            fig.text(0.03, 0.94, "Parts" + (" (continued)" if start else ""), fontsize=20,
                     weight="bold", color=INK)
            fig.text(0.03, 0.905, "P printed, C laser-cut, H bought. Labels are numbered in "
                     "the order the steps need them.", fontsize=9, color="#555")
            for i, t in enumerate(types[start:start + per]):
                r, c = divmod(i, cols)
                x, y = 0.03 + c * 0.16, 0.68 - r * 0.205
                _image(fig, thumbs[t.label], [x, y + 0.035, 0.15, 0.15])
                fig.text(x + 0.005, y + 0.02, f"{t.label}", fontsize=12, weight="bold",
                         color=INK)
                fig.text(x + 0.055, y + 0.02, f"x {t.qty}", fontsize=12, color=ACCENT,
                         weight="bold")
                fig.text(x + 0.005, y - 0.003, "\n".join(textwrap.wrap(t.name, 34)[:2]),
                         fontsize=6.5, color="#444", va="top")
            _footer(fig, footer, page)
            pdf.savefig(fig)
            fig.clf()
        # steps
        for st in steps:
            fig = _fig()
            page += 1
            fig.text(0.03, 0.93, f"{st.number}", fontsize=34, weight="bold", color=INK)
            fig.text(0.1, 0.94, st.title, fontsize=15, weight="bold", color=INK)
            if st.sub:
                fig.text(0.1, 0.915, "sub-assembly", fontsize=9, color=ACCENT,
                         weight="bold")
            _image(fig, images[st.number], [0.02, 0.17, 0.68, 0.73])
            body = "\n".join(textwrap.fill(t, 120) for t in st.text)
            fig.text(0.04, 0.145, body, fontsize=9, color=INK, va="top", linespacing=1.4)
            # the parts this step adds
            fig.add_artist(FancyBboxPatch(
                (0.715, 0.12), 0.265, 0.8, boxstyle="round,pad=0.005,rounding_size=0.01",
                fc="#f6f6f8", ec="#ccc", transform=fig.transFigure, zorder=-1))
            fig.text(0.73, 0.89, "Parts in this step", fontsize=10, weight="bold",
                     color=INK)
            rows = st.callouts[:12]
            h = min(0.064, 0.74 / max(len(rows), 1))
            for i, c in enumerate(rows):
                y = 0.86 - (i + 1) * h
                _image(fig, thumbs[c.label], [0.722, y, h * 0.75, h * 0.95])
                fig.text(0.722 + h * 0.8, y + h * 0.55, f"{c.label}  x {c.qty}",
                         fontsize=9.5, weight="bold", color=INK)
                fig.text(0.722 + h * 0.8, y + h * 0.2, textwrap.shorten(c.name, 46),
                         fontsize=6.5, color="#555")
            if len(st.callouts) > len(rows):
                fig.text(0.73, 0.13, f"and {len(st.callouts) - len(rows)} more types",
                         fontsize=8, color="#555")
            _footer(fig, footer, page)
            pdf.savefig(fig)
            fig.clf()
    return page
