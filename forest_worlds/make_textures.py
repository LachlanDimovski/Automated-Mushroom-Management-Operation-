"""Procedural textures: the hedge foliage (composited from the exported leaf cut-outs) and the river water.
python3 make_textures.py   (run after the models are exported)"""
import glob
import os

import numpy as np
from PIL import Image, ImageEnhance

HERE = os.path.dirname(os.path.abspath(__file__))


def normal_from(h, s):
    gy, gx = np.gradient(h)
    n = np.dstack([-gx * s, -gy * s, np.ones_like(h)])
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    return Image.fromarray(((n * 0.5 + 0.5) * 255).astype(np.uint8))


def hedge(size=1024):
    """Tileable dense evergreen: green-only crops of the fir/pine twig and fern atlases, overlapped on a dark base."""
    rng = np.random.default_rng(5)
    canvas = Image.new('RGBA', (size, size), (16, 27, 12, 255))
    crops = []
    files = (sorted(glob.glob(os.path.join(HERE, 'models/tree/*/materials/*twig*_albedo.png')))[::3]
             + glob.glob(os.path.join(HERE, 'models/bush/fern_02/materials/*albedo.png')))
    for f in files:
        a = np.asarray(Image.open(f).convert('RGBA')).copy()
        r, g, b = (a[..., i].astype(int) for i in range(3))
        a[..., 3] = np.where((g > r * 1.08) & (g > b * 1.1) & (a[..., 3] > 128), 255, 0)   # drop cones/bark
        im = Image.fromarray(a)
        ys, xs = np.where(a[..., 3] > 0)
        for _ in range(40):
            i = rng.integers(len(xs))
            rr = int(rng.integers(70, 150))
            c = im.crop((xs[i] - rr, ys[i] - rr, xs[i] + rr, ys[i] + rr))
            if (np.asarray(c)[..., 3] > 0).mean() > 0.15:
                crops.append(c)
    for k in range(3000):
        c = crops[rng.integers(len(crops))]
        sc = rng.uniform(0.3, 0.75)
        c = c.resize((max(8, int(c.width * sc)), max(8, int(c.height * sc)))).rotate(rng.uniform(0, 360), expand=True)
        c = ImageEnhance.Brightness(c).enhance(rng.uniform(0.45, 0.9) * (0.65 + 0.5 * k / 3000))
        x, y = rng.integers(0, size, 2)
        for dx in (-size, 0, size):
            for dy in (-size, 0, size):
                px, py = int(x + dx - c.width / 2), int(y + dy - c.height / 2)
                if -c.width < px < size and -c.height < py < size:
                    canvas.paste(c, (px, py), c)
    canvas.convert('RGB').save(os.path.join(HERE, 'textures', 'hedge_leaves_diff.jpg'), quality=90)
    normal_from(np.asarray(canvas.convert('L'), float) / 255, 4).save(
        os.path.join(HERE, 'textures', 'hedge_leaves_nor_gl.jpg'), quality=90)


def water(n=512):
    """Tileable ripple normal map (summed periodic waves) and a flat green-teal albedo."""
    d = os.path.join(HERE, 'world_assets', '_water_template', 'materials', 'textures')
    os.makedirs(d, exist_ok=True)
    rng = np.random.default_rng(7)
    y, x = np.mgrid[0:n, 0:n] / n * 2 * np.pi
    h = np.zeros((n, n))
    for f in range(1, 40):
        for _ in range(3):
            kx, ky = rng.integers(-f, f + 1, 2)
            h += np.sin(kx * x + ky * y + rng.uniform(0, 6.28)) / (1 + f) ** 1.3
    normal_from(h, 6.0).save(os.path.join(d, 'river_normal.png'))
    a = np.zeros((64, 64, 3), np.uint8)
    a[:] = (52, 74, 62)
    Image.fromarray(a).save(os.path.join(d, 'water_albedo.png'))


if __name__ == '__main__':
    hedge()
    water()
    print('textures ok')
