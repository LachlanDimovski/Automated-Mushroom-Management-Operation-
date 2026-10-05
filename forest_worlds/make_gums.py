"""Procedural eucalyptus (gum) trees as light Gazebo models: crooked tapering trunk and open upward branches with
Poly Haven blue-gum bark, drooping clusters of narrow grey-green leaf cards.
python3 make_gums.py [count]   (writes models/gum/gum_NN; needs textures/bark_bluegum_*.jpg from fetch_assets.py)"""
import math
import os
import shutil
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
SIDES = 7


def leaf_atlas(path, size=512, seed=3):
    """A drooping spray of lanceolate eucalyptus leaves on transparent background (one card texture)."""
    rng = np.random.default_rng(seed)
    im = Image.new('RGBA', (size, size), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    for _ in range(9):                                   # twigs hanging from the top edge
        x0 = rng.uniform(0.15, 0.85) * size
        pts = [(x0 + math.sin(t * 2.2) * 25, t * size * 0.95) for t in np.linspace(0.02, 1, 12)]
        d.line(pts, fill=(96, 72, 52, 255), width=3)
        for k, (px, py) in enumerate(pts[2:], 2):
            for side in (-1, 1):
                if rng.random() < 0.25:
                    continue
                L = rng.uniform(45, 85) * (1.0 - 0.3 * k / 12)
                ang = math.radians(90 + side * rng.uniform(15, 40))    # mostly hanging down
                w = L * 0.16
                poly = []
                for t in np.linspace(0, 1, 14):                     # sickle-shaped lanceolate leaf
                    bend = 8 * math.sin(t * math.pi) * side
                    cx = px + math.cos(ang) * L * t - math.sin(ang) * bend
                    cy = py + math.sin(ang) * L * t + math.cos(ang) * bend
                    poly.append((cx, cy, math.sin(t * math.pi) ** 0.8 * w))
                left = [(x - math.sin(ang) * hw, y + math.cos(ang) * hw) for x, y, hw in poly]
                right = [(x + math.sin(ang) * hw, y - math.cos(ang) * hw) for x, y, hw in poly][::-1]
                g = rng.uniform(0.75, 1.1)
                col = (int(88 * g), int(108 * g), int(78 * g), 255)            # eucalypt grey-green
                d.polygon(left + right, fill=col)
                d.line([(x, y) for x, y, _ in poly], fill=(int(120 * g), int(128 * g), int(92 * g), 255), width=1)
    im = im.filter(ImageFilter.SMOOTH)
    a = np.asarray(im).copy()
    a[..., 3] = np.where(a[..., 3] > 100, 255, 0)       # hard alpha for alpha-testing
    Image.fromarray(a).save(path)


def pale_bark(src, dst):
    """Smooth-barked gums are pale grey-cream: lift and desaturate the bark photo."""
    from PIL import ImageEnhance
    im = Image.open(src).convert('RGB')
    im = ImageEnhance.Color(im).enhance(0.55)
    im = ImageEnhance.Brightness(im).enhance(1.7)
    im.save(dst, quality=90)


class Mesh:
    def __init__(self):
        self.V, self.UV, self.N, self.F = [], [], [], []

    def add(self, V, UV, N, F):
        o = len(self.V)
        self.V += V
        self.UV += UV
        self.N += N
        self.F += [(a + o, b + o, c + o) for a, b, c in F]

    def write(self, path):
        with open(path, 'w') as fh:
            fh.write('usemtl m\n')
            fh.writelines(f'v {x:.3f} {y:.3f} {z:.3f}\n' for x, y, z in self.V)
            fh.writelines(f'vt {u:.4f} {v:.4f}\n' for u, v in self.UV)
            fh.writelines(f'vn {x:.3f} {y:.3f} {z:.3f}\n' for x, y, z in self.N)
            fh.writelines(f'f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}\n' for a, b, c in np.array(self.F) + 1)


def frame(t):
    t = t / np.linalg.norm(t)
    a = np.array([0, 0, 1.0]) if abs(t[2]) < 0.9 else np.array([1.0, 0, 0])
    u = np.cross(t, a)
    u /= np.linalg.norm(u)
    return u, np.cross(t, u)


def tube(mesh, pts, radii, v0=0.0):
    """Tapered tube along a polyline; bark UVs in metres (u round the trunk, v along it)."""
    V, UV, N, F = [], [], [], []
    along = v0
    for i, (p, r) in enumerate(zip(pts, radii)):
        t = pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]
        u, w = frame(t)
        if i:
            along += np.linalg.norm(pts[i] - pts[i - 1])
        for s in range(SIDES + 1):
            a = 2 * math.pi * s / SIDES
            n = math.cos(a) * u + math.sin(a) * w
            V.append(tuple(p + n * r))
            N.append(tuple(n))
            UV.append((s / SIDES * max(r * 2 * math.pi, 0.3) / 0.8, along / 0.8))
    for i in range(len(pts) - 1):
        for s in range(SIDES):
            a, b = i * (SIDES + 1) + s, (i + 1) * (SIDES + 1) + s
            F += [(a, b, b + 1), (a, b + 1, a + 1)]
    mesh.add(V, UV, N, F)


def crooked(rng, base, direction, length, segs, wander):
    pts = [np.array(base, float)]
    d = np.array(direction, float)
    d /= np.linalg.norm(d)
    for _ in range(segs):
        d = d + rng.normal(0, wander, 3)
        d[2] = max(d[2], 0.15)
        d /= np.linalg.norm(d)
        pts.append(pts[-1] + d * length / segs)
    return np.array(pts)


def leaf_cluster(mesh, rng, p, size):
    """Three crossed hanging cards: sprays droop from p."""
    for k in range(3):
        a = rng.uniform(0, math.pi) + k * math.pi / 3
        ux, uy = math.cos(a), math.sin(a)
        w, h = size, size * 0.75
        top = p + np.array([0, 0, 0.15 * size])
        quad = [top + np.array([-ux * w / 2, -uy * w / 2, 0]), top + np.array([ux * w / 2, uy * w / 2, 0]),
                top + np.array([ux * w / 2, uy * w / 2, -h]), top + np.array([-ux * w / 2, -uy * w / 2, -h])]
        n = (-uy, ux, 0.25)
        mesh.add([tuple(q) for q in quad], [(0, 1), (1, 1), (1, 0), (0, 0)], [n] * 4, [(0, 1, 2), (0, 2, 3)])
        mesh.add([tuple(q) for q in quad], [(0, 1), (1, 1), (1, 0), (0, 0)], [(-n[0], -n[1], n[2])] * 4,
                 [(0, 2, 1), (0, 3, 2)])     # back face, so both sides render


def gum(seed):
    rng = np.random.default_rng(seed)
    bark, leaves = Mesh(), Mesh()
    H = rng.uniform(14, 22)
    r0 = rng.uniform(0.22, 0.34)
    trunk = crooked(rng, (0, 0, -0.2), (rng.normal(0, 0.05), rng.normal(0, 0.05), 1), H * 0.68, 9, 0.06)
    tube(bark, trunk, np.linspace(r0 * 1.25, r0 * 0.45, len(trunk)))
    tips = []
    # gum canopies start high: limbs from the top 45 % of the trunk, angled up and out
    for i in range(int(rng.integers(4, 7))):
        k = int(rng.integers(len(trunk) * 0.6, len(trunk) - 1))
        az = rng.uniform(0, 2 * math.pi)
        d = (math.cos(az), math.sin(az), rng.uniform(0.7, 1.4))
        L = H * rng.uniform(0.25, 0.4)
        limb = crooked(rng, trunk[k], d, L, 5, 0.25)
        rr = r0 * rng.uniform(0.35, 0.5)
        tube(bark, limb, np.linspace(rr, rr * 0.25, len(limb)))
        for j in range(2, len(limb)):
            az2 = az + rng.normal(0, 0.9)
            twig = crooked(rng, limb[j], (math.cos(az2), math.sin(az2), rng.uniform(0.2, 0.9)), L * 0.35, 3, 0.3)
            tube(bark, twig, np.linspace(rr * 0.25, rr * 0.08, len(twig)))
            tips += list(twig[1:])
        tips.append(limb[-1])
    for p in tips:
        leaf_cluster(leaves, rng, p + rng.normal(0, 0.3, 3), rng.uniform(0.9, 1.5))
    return bark, leaves, H


def model_sdf(name, h):
    def vis(part, tex, normal, shadows):
        nm = f'<normal_map>model://{name}/materials/{normal}</normal_map>' if normal else ''
        return (f'<visual name="{part}"><geometry><mesh><uri>model://{name}/mesh/{part}.obj</uri></mesh></geometry>'
                f'<material><diffuse>1 1 1 1</diffuse><pbr><metal><metalness>0</metalness><roughness>0.9</roughness>'
                f'<albedo_map>model://{name}/materials/{tex}</albedo_map>{nm}</metal></pbr></material>'
                f'<cast_shadows>{shadows}</cast_shadows></visual>')
    return (f'<?xml version="1.0"?>\n<sdf version="1.8"><model name="{name}"><static>true</static><link name="link">'
            + vis('bark', 'bark_diff.jpg', 'bark_nor_gl.jpg', 'true') + vis('leaves', 'leaves.png', None, 'false')
            + '<collision name="trunk"><pose>0 0 2 0 0 0</pose><geometry><cylinder><radius>0.3</radius>'
              '<length>4</length></cylinder></geometry></collision></link></model></sdf>\n')


def main(count):
    out = os.path.join(HERE, 'models', 'gum')
    atlas = os.path.join(HERE, 'textures', 'gum_leaves.png')
    leaf_atlas(atlas)
    for i in range(count):
        name = f'gum_{i:02d}'
        mdir = os.path.join(out, name)
        os.makedirs(os.path.join(mdir, 'mesh'), exist_ok=True)
        os.makedirs(os.path.join(mdir, 'materials'), exist_ok=True)
        bark, leaves, h = gum(100 + i)
        bark.write(os.path.join(mdir, 'mesh', 'bark.obj'))
        leaves.write(os.path.join(mdir, 'mesh', 'leaves.obj'))
        pale_bark(os.path.join(HERE, 'textures', 'bark_bluegum_diff.jpg'), os.path.join(mdir, 'materials', 'bark_diff.jpg'))
        shutil.copy(os.path.join(HERE, 'textures', 'bark_bluegum_nor_gl.jpg'),
                    os.path.join(mdir, 'materials', 'bark_nor_gl.jpg'))
        shutil.copy(atlas, os.path.join(mdir, 'materials', 'leaves.png'))
        with open(os.path.join(mdir, 'model.sdf'), 'w') as fh:
            fh.write(model_sdf(name, h))
        with open(os.path.join(mdir, 'model.config'), 'w') as fh:
            fh.write(f'<?xml version="1.0"?>\n<model><name>{name}</name><version>1.0</version>'
                     f'<sdf version="1.8">model.sdf</sdf></model>\n')
        with open(os.path.join(mdir, 'dims.txt'), 'w') as fh:
            fh.write(f'6 6 {h:.1f} {len(bark.F) + len(leaves.F)}\n')
        print(name, f'{h:.1f} m', len(bark.F) + len(leaves.F), 'tris')


if __name__ == '__main__':
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 8)
