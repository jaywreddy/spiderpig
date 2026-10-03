# ruff: noqa: E501
"""Render the viewer's images for docs/architecture/: the default robot from three sides and one
leg, cropped to the model and saved as WebP in img/.

Start the viewer's server first, on a store that holds (or may bake) the default designs:

    uv run python -m uvicorn spiderpig.server.app:app --port 8765
    uv run python docs/architecture/shots.py [--url http://127.0.0.1:8765]
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image
from playwright.sync_api import sync_playwright

OUT = Path(__file__).resolve().parent / 'img'

SHOTS = [  # file, viewer mode, camera preset, clip time
    ('robot_three_quarter', 'robot', 'three-quarter', 0.12),
    ('robot_front', 'robot', 'front', 0.12),
    ('robot_side', 'robot', 'side', 0.3),
    ('single_three_quarter', 'klann', 'three-quarter', 0.2),
]


def crop(png: Path, out: Path, width: int = 1400) -> None:
    """Crop to the model (pixels well off the background), pad 6 %, save as WebP."""
    im = Image.open(png).convert('RGB')
    a = np.asarray(im).astype(int)
    diff = np.abs(a - a[5, 5]).max(axis=2)
    ys, xs = np.where(diff > 70)
    x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
    pad = int(0.06 * max(x1 - x0, y1 - y0))
    c = im.crop((max(0, x0 - pad), max(0, y0 - pad), min(im.width, x1 + pad), min(im.height, y1 + pad)))
    w = min(width, c.width)
    c = c.resize((w, round(c.height * w / c.width)), Image.LANCZOS)
    c.save(out, 'WEBP', quality=84, method=6)


def main() -> None:
    ap = argparse.ArgumentParser(description='Render the viewer images for docs/architecture/.')
    ap.add_argument('--url', default='http://127.0.0.1:8765', help="the viewer server's address")
    url = ap.parse_args().url
    OUT.mkdir(exist_ok=True)
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist'])
        # A large viewport at scale 1: the viewer's canvas doesn't follow devicePixelRatio.
        pg = b.new_page(viewport={'width': 2400, 'height': 1540}, device_scale_factor=1)
        for name, mode, view, t in SHOTS:
            pg.goto(f'{url}/?mode={mode}&view={view}&t={t}', timeout=300_000)
            pg.wait_for_function('window.__viewer && window.__viewer.ready', timeout=400_000)
            time.sleep(2.5)
            pg.add_style_tag(content='body > *:not(canvas){visibility:hidden !important} '
                                     'canvas{visibility:visible !important}')
            time.sleep(0.5)
            png = OUT / f'{name}.png'
            pg.screenshot(path=str(png))
            crop(png, OUT / f'{name}.webp')
            png.unlink()
            print('rendered', name, file=sys.stderr)
        b.close()


if __name__ == '__main__':
    main()
