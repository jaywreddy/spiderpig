"""The guide's PDF (:class:`guide.doc.Doc` in, a file out), laid out with reportlab's canvas,
A4 landscape: a cover, the parts inventory, the print batches, the bag labels, a page per step.

Deterministic bytes: ``rl_config.invariant`` (no dates, a fixed document id), the base-14
Helvetica fonts (nothing embedded), images lossless (Flate). One ``ImageReader`` per path, so
a thumbnail on many pages is one shared image XObject.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from pathlib import Path

from reportlab import rl_config
from reportlab.lib.colors import Color, HexColor
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.utils import ImageReader, simpleSplit
from reportlab.pdfbase.pdfmetrics import stringWidth
from reportlab.pdfgen.canvas import Canvas

from spiderpig.guide.doc import Doc, PartEntry, PrintBatch, StepEntry

W, H = landscape(A4)
M = 30.0                          # page margin
FOOT = 40.0                       # the content's floor, above the footer
INK = HexColor("#18181c")
GREY = HexColor("#7a7a82")
RULE = HexColor("#d4d4da")
PANEL = HexColor("#f2f2f5")
ZEBRA = HexColor("#f6f6f8")
ACCENT = HexColor("#f58c28")
WHITE = HexColor("#ffffff")
REG, BOLD = "Helvetica", "Helvetica-Bold"

# WinAnsi (cp1252) is what the base-14 fonts draw; the rest gets an ASCII stand-in
_SUBST = str.maketrans({
    "\u2212": "-", "\u2192": "->", "\u2190": "<-", "\u2248": "~", "\u2264": "<=",
    "\u2265": ">=", "\u2713": "ok", "\u2011": "-", "\u2300": "\u00d8", "\u00a0": " ",
    "\u2009": " ",
})

Page = Callable[[Canvas], None]


def _safe(text: str) -> str:
    text = text.translate(_SUBST)
    return text.encode("cp1252", "replace").decode("cp1252")


def _fit(text: str, font: str, size: float, width: float) -> str:
    """``text`` cut with an ellipsis to ``width``."""
    text = _safe(text)
    if stringWidth(text, font, size) <= width:
        return text
    while text and stringWidth(text + "…", font, size) > width:
        text = text[:-1]
    return text.rstrip() + "…"


def _wrap(text: str, font: str, size: float, width: float, lines: int = 0) -> list[str]:
    """``text`` wrapped to ``width``; at most ``lines`` (0: any), the last one cut."""
    out = simpleSplit(_safe(text), font, size, width)
    if lines and len(out) > lines:
        out = [*out[: lines - 1], _fit(" ".join(out[lines - 1:]), font, size, width)]
    return out


class _Images:
    """One reader per path (one XObject in the file), and each picture's pixel size."""

    def __init__(self) -> None:
        self._readers: dict[Path, ImageReader] = {}

    def reader(self, path: Path) -> ImageReader:
        key = Path(path)
        if key not in self._readers:
            self._readers[key] = ImageReader(str(key))
        return self._readers[key]

    def draw(self, c: Canvas, path: Path, x: float, y: float, w: float, h: float,
             *, valign: str = "middle") -> tuple[float, float, float, float]:
        """The picture fitted into the box, aspect kept, centred across; its drawn rect."""
        img = self.reader(path)
        iw, ih = img.getSize()
        s = min(w / iw, h / ih)
        dw, dh = iw * s, ih * s
        dx = x + (w - dw) / 2
        dy = {"top": y + h - dh, "bottom": y}.get(valign, y + (h - dh) / 2)
        c.drawImage(img, dx, dy, dw, dh, mask="auto")
        return dx, dy, dw, dh


def _text(c: Canvas, x: float, y: float, text: str, font: str, size: float,
          color: Color = INK, *, align: str = "left") -> None:
    c.setFont(font, size)
    c.setFillColor(color)
    text = _safe(text)
    if align == "right":
        c.drawRightString(x, y, text)
    elif align == "centre":
        c.drawCentredString(x, y, text)
    else:
        c.drawString(x, y, text)


def _header(c: Canvas, title: str, note: str = "") -> None:
    _text(c, M, H - M - 18, title, BOLD, 20)
    c.setFillColor(ACCENT)
    c.rect(M, H - M - 27, 36, 2.5, stroke=0, fill=1)
    if note:
        _text(c, M, H - M - 42, note, REG, 9, GREY)


def _footer(c: Canvas, text: str, page: int, pages: int) -> None:
    c.setStrokeColor(RULE)
    c.setLineWidth(0.5)
    c.line(M, 28, W - M, 28)
    _text(c, M, 17, _fit(text, REG, 7.5, W - 2 * M - 60), REG, 7.5, GREY)
    _text(c, W - M, 17, f"{page} / {pages}", BOLD, 9, GREY, align="right")


# ----- the pages -----

def _cover(doc: Doc, images: _Images) -> Page:
    def draw(c: Canvas) -> None:
        col = W * 0.36
        y = H - M - 70
        for line in _wrap(doc.title, BOLD, 34, col - M):
            _text(c, M, y, line, BOLD, 34)
            y -= 40
        c.setFillColor(ACCENT)
        c.rect(M, y + 18, 60, 4, stroke=0, fill=1)
        y -= 14
        for sub in doc.subtitle:
            for line in _wrap(sub, REG, 13, col - M):
                _text(c, M, y, line, REG, 13, GREY)
                y -= 18
            y -= 4
        images.draw(c, doc.cover, col + 10, FOOT + 10, W - M - col - 10, H - 2 * M - FOOT)
    return draw


def _size_for(text: str, size: float, width: float, least: float = 7.0) -> float:
    """The font size, at most ``size``, at which ``text`` fits ``width`` (Helvetica Bold)."""
    while size > least and stringWidth(_safe(text), BOLD, size) > width:
        size -= 0.5
    return size


def _parts(doc: Doc, images: _Images) -> list[Page]:
    cols, rows, gap = 6, 4, 8.0
    top, bottom = H - M - 52, FOOT + 4
    cw = (W - 2 * M - (cols - 1) * gap) / cols
    ch = (top - bottom - (rows - 1) * gap) / rows
    per = cols * rows
    chunks = [doc.parts[i:i + per] for i in range(0, len(doc.parts), per)] or [[]]

    def card(c: Canvas, p: PartEntry, x: float, y: float) -> None:
        c.setStrokeColor(RULE)
        c.setFillColor(WHITE)
        c.setLineWidth(0.6)
        c.roundRect(x, y, cw, ch, 5, stroke=1, fill=1)
        pad = 6.0
        images.draw(c, p.thumb, x + pad, y + 36, cw - 2 * pad, ch - 36 - pad)
        qty = f"× {p.qty}"
        room = cw - 2 * pad - stringWidth(_safe(qty), BOLD, 11) - 4
        _text(c, x + pad, y + 24, p.label, BOLD, _size_for(p.label, 11, room))
        _text(c, x + cw - pad, y + 24, qty, BOLD, 11, ACCENT, align="right")
        for i, line in enumerate(_wrap(p.name, REG, 6.8, cw - 2 * pad, 2)):
            _text(c, x + pad, y + 14 - i * 8, line, REG, 6.8, GREY)

    def page(chunk: list[PartEntry], n: int) -> Page:
        def draw(c: Canvas) -> None:
            _header(c, "Parts" + (" (continued)" if n else ""),
                    "A label says what the part is: printed SP spacer, RG ring, CL collar, "
                    "SL sleeve, with sizes; laser-cut LK link, CW crank web with its outline; "
                    "bought, its catalog key.")
            for i, p in enumerate(chunk):
                r, k = divmod(i, cols)
                card(c, p, M + k * (cw + gap), top - (r + 1) * ch - r * gap)
        return draw

    return [page(chunk, n) for n, chunk in enumerate(chunks)]


def _prints(doc: Doc) -> list[Page]:
    if not doc.prints:
        return []
    rh = 16.0
    top = H - M - 62
    per = int((top - FOOT - 2 * rh) // rh)          # leaves room for the total row
    chunks = [doc.prints[i:i + per] for i in range(0, len(doc.prints), per)]
    # column: (title, x of its left or right edge, right-aligned)
    x0, x1 = M, W - M
    cols = [("Label", x0 + 6, False), ("Print file", x0 + 70, False),
            ("Qty", x0 + 470, True), ("Filament", x0 + 500, False),
            ("g each", x1 - 90, True), ("g total", x1 - 6, True)]
    file_w = 470 - 70 - 30
    fil_w = x1 - 90 - 50 - (x0 + 500)
    total_g = sum(b.qty * b.grams_each for b in doc.prints)
    total_n = sum(b.qty for b in doc.prints)

    def row(c: Canvas, y: float, cells: list[str], font: str) -> None:
        for i, ((_, x, right), cell) in enumerate(zip(cols, cells, strict=True)):
            _text(c, x, y + 4.5, cell, BOLD if i == 0 else font, 9,
                  align="right" if right else "left")

    def cells(b: PrintBatch) -> list[str]:
        return [b.label, _fit(b.file, REG, 9, file_w), str(b.qty),
                _fit(b.filament, REG, 9, fil_w), f"{b.grams_each:.1f}",
                f"{b.qty * b.grams_each:.1f}"]

    def page(chunk: list[PrintBatch], n: int, last: bool) -> Page:
        def draw(c: Canvas) -> None:
            _header(c, "Prints" + (" (continued)" if n else ""),
                    "One print file per label: print each batch, then bag it with its label "
                    "(next pages).")
            y = top - rh
            for title, x, right in cols:
                _text(c, x, y + 4.5, title, BOLD, 9, GREY, align="right" if right else "left")
            c.setStrokeColor(INK)
            c.setLineWidth(0.8)
            c.line(x0, y, x1, y)
            for i, b in enumerate(chunk):
                y -= rh
                if i % 2:
                    c.setFillColor(ZEBRA)
                    c.rect(x0, y, x1 - x0, rh, stroke=0, fill=1)
                row(c, y, cells(b), REG)
            if last:
                c.setStrokeColor(INK)
                c.line(x0, y, x1, y)
                y -= rh
                row(c, y, ["Total", f"{len(doc.prints)} print files", str(total_n), "", "",
                           f"{total_g:.0f} g"], BOLD)
        return draw

    return [page(ch, n, n == len(chunks) - 1) for n, ch in enumerate(chunks)]


def _labels(doc: Doc, images: _Images) -> list[Page]:
    bag = [p for p in doc.parts if p.kind != "laser"]
    if not bag:
        return []
    cols, rows = 4, 6
    top, bottom = H - M - 36, FOOT
    cw, ch = (W - 2 * M) / cols, (top - bottom) / rows
    per = cols * rows
    chunks = [bag[i:i + per] for i in range(0, len(bag), per)]

    def cell(c: Canvas, p: PartEntry, x: float, y: float) -> None:
        pad = 8.0
        th = ch - 2 * pad
        images.draw(c, p.thumb, x + cw - pad - th, y + pad, th, th)
        tw = cw - 3 * pad - th
        qty = f"× {p.qty}"
        size = _size_for(p.label, 28, tw - stringWidth(_safe(qty), BOLD, 14) - 6)
        _text(c, x + pad, y + ch - pad - 24, p.label, BOLD, size)
        lw = stringWidth(_safe(p.label), BOLD, size)
        _text(c, x + pad + lw + 6, y + ch - pad - 24, qty, BOLD, 14, ACCENT)
        ly = y + ch - pad - 37
        for line in _wrap(p.name, REG, 7.5, tw, 2):
            _text(c, x + pad, ly, line, REG, 7.5)
            ly -= 9
        if p.detail:
            _text(c, x + pad, ly, _fit(p.detail, REG, 7.5, tw), REG, 7.5, GREY)

    def page(chunk: list[PartEntry], n: int) -> Page:
        def draw(c: Canvas) -> None:
            _header(c, "Bag labels — print at 100 %, cut along the dashed lines"
                    + (" (continued)" if n else ""))
            for i, p in enumerate(chunk):
                r, k = divmod(i, cols)
                cell(c, p, M + k * cw, top - (r + 1) * ch)
            # the cut lines: around the cells in use only
            rows_used = math.ceil(len(chunk) / cols)
            width = [min(cols, len(chunk) - r * cols) for r in range(rows_used)]
            c.setStrokeColor(GREY)
            c.setLineWidth(0.5)
            c.setDash(4, 3)
            for r in range(rows_used + 1):
                span = max(width[r - 1] if r else 0, width[r] if r < rows_used else 0)
                c.line(M, top - r * ch, M + span * cw, top - r * ch)
            for r, span in enumerate(width):
                for k in range(span + 1):
                    c.line(M + k * cw, top - r * ch, M + k * cw, top - (r + 1) * ch)
            c.setDash()
        return draw

    return [page(chunk, n) for n, chunk in enumerate(chunks)]


def _step(step: StepEntry, images: _Images) -> Page:
    left = (W - 2 * M) * 0.68
    px = M + left + 16                      # the parts panel
    pw = W - M - px
    head = H - M - 62                       # below the step's heading
    size, lead, gap = 10.0, 13.5, 5.0

    def heading(c: Canvas) -> None:
        num = str(step.number)
        _text(c, M, H - M - 40, num, BOLD, 48)
        tx = M + stringWidth(num, BOLD, 48) + 14
        tw = M + left - tx
        sy = H - M - 9
        _text(c, tx, sy, _fit(step.stage, REG, 9, tw - 90), REG, 9, GREY)
        if step.sub:
            sx = tx + stringWidth(_fit(step.stage, REG, 9, tw - 90), REG, 9) + 8
            label = "SUB-ASSEMBLY"
            lw = stringWidth(label, BOLD, 7) + 10
            c.setFillColor(ACCENT)
            c.roundRect(sx, sy - 3, lw, 12, 3, stroke=0, fill=1)
            _text(c, sx + 5, sy + 0.5, label, BOLD, 7, WHITE)
        lines = _wrap(step.title, BOLD, 17, tw, 2)
        for i, line in enumerate(lines):
            _text(c, tx, H - M - 29 - i * 19, line, BOLD, 17)

    def body(c: Canvas) -> None:
        paras = [_wrap(t, REG, size, left - 4) for t in step.text]
        th = sum(len(p) for p in paras) * lead + max(len(paras) - 1, 0) * gap
        ih = head - FOOT - (th + 12 if paras else 0)
        _, iy, _, _ = images.draw(c, step.image, M, head - ih, left, ih, valign="top")
        y = iy - 8 - size
        for para in paras:
            for line in para:
                _text(c, M + 2, y, line, REG, size)
                y -= lead
            y -= gap

    def panel(c: Canvas) -> None:
        ptop, pbot = H - M, FOOT
        c.setFillColor(PANEL)
        c.roundRect(px, pbot, pw, ptop - pbot, 8, stroke=0, fill=1)
        pad = 10.0
        _text(c, px + pad, ptop - pad - 11, "Parts in this step", BOLD, 11)
        top, avail = ptop - pad - 22, ptop - pbot - 2 * pad - 22
        calls = step.callouts
        if not calls:
            _text(c, px + pad, top - 14, "No new parts: this step uses what is built.",
                  REG, 8.5, GREY)
            return
        rh = min(56.0, avail / len(calls))
        shown = calls
        if rh < 30.0:
            rh = 30.0
            n = int((avail - 14) // rh)
            shown = calls[:n]
        y = top
        tw = pw - 2 * pad - rh - 4
        for cl in shown:
            y -= rh
            ts = rh - 6
            c.setFillColor(WHITE)
            c.roundRect(px + pad, y + 3, ts, ts, 3, stroke=0, fill=1)
            images.draw(c, cl.thumb, px + pad + 2, y + 5, ts - 4, ts - 4)
            tx = px + pad + ts + 8
            names = _wrap(cl.name, REG, 7.5, tw - 4, 2 if rh >= 44 else 1)
            block = 12 + 9 * len(names)
            ty = y + rh / 2 + block / 2 - 10
            _text(c, tx, ty, cl.label, BOLD, 10.5)
            _text(c, tx + stringWidth(_safe(cl.label), BOLD, 10.5) + 6, ty, f"× {cl.qty}",
                  BOLD, 10.5, ACCENT)
            for i, line in enumerate(names):
                _text(c, tx, ty - 11 - i * 9, line, REG, 7.5, GREY)
        if len(shown) < len(calls):
            _text(c, px + pad, y - 12, f"and {len(calls) - len(shown)} more", BOLD, 9, GREY)

    def draw(c: Canvas) -> None:
        heading(c)
        body(c)
        panel(c)

    return draw


def write(path: Path, doc: Doc) -> int:
    """Lay ``doc`` out into the PDF at ``path``; the page count."""
    images = _Images()
    pages: list[Page] = [_cover(doc, images), *_parts(doc, images), *_prints(doc),
                         *_labels(doc, images), *(_step(s, images) for s in doc.steps)]
    was = rl_config.invariant, rl_config.useA85
    rl_config.invariant, rl_config.useA85 = 1, 0     # no dates; binary streams, not ASCII85
    try:
        c = Canvas(str(path), pagesize=(W, H), invariant=1, pageCompression=1)
        c.setTitle(_safe(doc.title))
        c.setAuthor("spiderpig")
        c.setCreator("spiderpig")
        c.setProducer("spiderpig")
        for n, page in enumerate(pages, 1):
            page(c)
            _footer(c, doc.footer, n, len(pages))
            c.showPage()
        c.save()
    finally:
        rl_config.invariant, rl_config.useA85 = was
    return len(pages)
