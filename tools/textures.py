"""Procedural tomb textures (sandstone, hieroglyph walls, sealed door).

Everything is drawn with NumPy + Pillow from fixed seeds, so the textures are
our own work (no copied images) and identical on every machine.
"""

import math
import os

import numpy as np
from PIL import Image, ImageDraw

SIZE = 512
SAND = np.array([205, 170, 115], dtype=float)
INK = (70, 42, 20)
LAPIS = (35, 70, 140)
RED = (150, 50, 30)
GOLD = (222, 175, 60)


def _noise(rng, size, octaves=((8, 0.5), (32, 0.3), (128, 0.2))):
    """Smooth value noise in [-1, 1] built from upscaled random grids."""
    out = np.zeros((size, size))
    for cells, weight in octaves:
        small = rng.uniform(-1, 1, (cells, cells)).astype(np.float32)
        img = Image.fromarray(small).resize((size, size), Image.BICUBIC)
        out += weight * np.asarray(img)
    return out


def sandstone(seed=1, blocks=True):
    rng = np.random.default_rng(seed)
    n = _noise(rng, SIZE)
    grain = rng.normal(0, 0.06, (SIZE, SIZE))
    shade = 1.0 + 0.12 * n + grain
    img = np.clip(SAND[None, None, :] * shade[..., None], 0, 255)
    if blocks:
        # mortar lines: rows every 128 px, joints staggered every other row
        mortar = SAND * 0.62
        for r in range(0, SIZE, 128):
            img[r:r + 3] = mortar
            off = 0 if (r // 128) % 2 == 0 else 85
            for c in range(off, SIZE + 170, 170):
                img[r:r + 128, c % SIZE:(c % SIZE) + 3] = mortar
    return Image.fromarray(img.astype(np.uint8))


# ------------------------------------------------------------------ glyphs
def _ankh(d, x, y, s, c):
    d.ellipse([x - s * 0.22, y - s * 0.5, x + s * 0.22, y - s * 0.05], outline=c, width=4)
    d.line([x, y - s * 0.05, x, y + s * 0.5], fill=c, width=5)
    d.line([x - s * 0.32, y + s * 0.02, x + s * 0.32, y + s * 0.02], fill=c, width=5)


def _eye(d, x, y, s, c):
    d.ellipse([x - s * 0.42, y - s * 0.18, x + s * 0.42, y + s * 0.18], outline=c, width=4)
    d.ellipse([x - s * 0.1, y - s * 0.1, x + s * 0.1, y + s * 0.1], fill=c)
    d.line([x - s * 0.42, y - s * 0.28, x + s * 0.42, y - s * 0.28], fill=c, width=4)
    d.line([x - s * 0.05, y + s * 0.18, x - s * 0.15, y + s * 0.45], fill=c, width=4)
    d.arc([x + s * 0.0, y + s * 0.15, x + s * 0.35, y + s * 0.5], 180, 450, fill=c, width=4)


def _bird(d, x, y, s, c):
    d.polygon([(x - s * 0.35, y + s * 0.1), (x + s * 0.2, y - s * 0.1),
               (x + s * 0.3, y + s * 0.2), (x - s * 0.1, y + s * 0.3)], fill=c)
    d.ellipse([x + s * 0.1, y - s * 0.35, x + s * 0.32, y - s * 0.12], fill=c)
    d.line([x + s * 0.3, y - s * 0.25, x + s * 0.45, y - s * 0.2], fill=c, width=4)
    d.line([x, y + s * 0.28, x - s * 0.05, y + s * 0.5], fill=c, width=3)
    d.line([x + s * 0.12, y + s * 0.26, x + s * 0.12, y + s * 0.5], fill=c, width=3)


def _water(d, x, y, s, c):
    pts = [(x - s * 0.45 + i * s * 0.1, y + (s * 0.08 if i % 2 else -s * 0.08)) for i in range(10)]
    d.line(pts, fill=c, width=4)


def _scarab(d, x, y, s, c):
    d.ellipse([x - s * 0.2, y - s * 0.25, x + s * 0.2, y + s * 0.35], fill=c)
    d.ellipse([x - s * 0.12, y - s * 0.42, x + s * 0.12, y - s * 0.22], fill=c)
    for k in (-1, 1):
        for dy in (-0.1, 0.08, 0.26):
            d.line([x + k * s * 0.18, y + dy * s, x + k * s * 0.4, y + (dy - 0.12) * s], fill=c, width=3)


def _sun(d, x, y, s, c):
    d.ellipse([x - s * 0.3, y - s * 0.3, x + s * 0.3, y + s * 0.3], outline=c, width=5)
    d.ellipse([x - s * 0.06, y - s * 0.06, x + s * 0.06, y + s * 0.06], fill=c)


def _reed(d, x, y, s, c):
    d.polygon([(x, y - s * 0.5), (x + s * 0.12, y - s * 0.1), (x + s * 0.04, y + s * 0.5),
               (x - s * 0.04, y + s * 0.5), (x - s * 0.12, y - s * 0.1)], fill=c)


def _snake(d, x, y, s, c):
    pts = [(x - s * 0.4 + i * s * 0.08, y + s * 0.12 * math.sin(i * 0.9)) for i in range(11)]
    d.line(pts, fill=c, width=5)
    d.ellipse([x + s * 0.38, y - s * 0.12, x + s * 0.5, y + s * 0.02], fill=c)


GLYPHS = (_ankh, _eye, _bird, _water, _scarab, _sun, _reed, _snake)


def hieroglyphs(seed=2):
    rng = np.random.default_rng(seed)
    img = sandstone(seed, blocks=False)
    d = ImageDraw.Draw(img)
    # painted bands at the top and bottom of the wall
    for y0, cols in ((0, (LAPIS, GOLD, RED)), (SIZE - 36, (RED, GOLD, LAPIS))):
        for k, col in enumerate(cols):
            d.rectangle([0, y0 + k * 12, SIZE, y0 + k * 12 + 11], fill=col)
    # columns of glyphs separated by vertical rules, as on real tomb walls
    col_w, row_h, top = 64, 70, 50
    for ci in range(SIZE // col_w):
        x0 = ci * col_w
        d.line([x0, top - 8, x0, SIZE - 44], fill=INK, width=2)
        for ri in range((SIZE - top - 50) // row_h):
            glyph = GLYPHS[rng.integers(len(GLYPHS))]
            colour = (INK, INK, LAPIS, RED)[rng.integers(4)]
            glyph(d, x0 + col_w / 2, top + ri * row_h + row_h / 2, 52, colour)
    return img


def sealed_door(seed=3):
    img = sandstone(seed, blocks=False)
    d = ImageDraw.Draw(img)
    d.rectangle([12, 12, SIZE - 12, SIZE - 12], outline=GOLD, width=18)
    d.rectangle([44, 44, SIZE - 44, SIZE - 44], outline=LAPIS, width=8)
    _eye(d, SIZE / 2, SIZE * 0.33, 300, LAPIS)
    _scarab(d, SIZE / 2, SIZE * 0.68, 200, GOLD)
    return img


def write_all(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    sandstone().save(os.path.join(out_dir, "sandstone.png"))
    hieroglyphs().save(os.path.join(out_dir, "hieroglyphs.png"))
    sealed_door().save(os.path.join(out_dir, "door.png"))
