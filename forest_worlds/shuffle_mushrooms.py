"""New random mushroom and decoy placement in an existing river world, everything else unchanged.

python3 shuffle_mushrooms.py --world river_moderate_aus --seed 3  ->  worlds/river_moderate_aus_m3.sdf (+ .meta.json)
Reuses the world's assets, so no rebuild. Same placement rules as build_world.py.
"""
import argparse
import json
import math
import os
import re
import xml.etree.ElementTree as ET

import numpy as np
from scipy.interpolate import LinearNDInterpolator

HERE = os.path.dirname(os.path.abspath(__file__))

ap = argparse.ArgumentParser()
ap.add_argument('--world', default='river_moderate_aus')
ap.add_argument('--seed', type=int, required=True)
args = ap.parse_args()

src = os.path.join(HERE, 'worlds', args.world)
meta = json.load(open(src + '.meta.json'))
n_mush, n_nat = len(meta['mushrooms']), len(meta['natives'])
hedge_r = meta['hedge']['half_size']
river = np.array(meta['river']['line'])            # x, y, bank-top half width (water half width + 1.4)
crossings = meta.get('crossings', meta.get('fords'))
tangents = meta['river'].get('crossing_tangents', meta['river'].get('ford_tangents'))

obstacles = []
for c in ET.parse(os.path.join(HERE, 'world_assets', f'forest_colliders_{args.world}', 'model.sdf')).getroot().iter('collision'):
    if c.find('pose') is not None:
        obstacles.append(tuple(map(float, c.find('pose').text.split()[:2])))
obstacles = np.array(obstacles)

v = np.array([list(map(float, l.split()[1:4])) for l in
              open(os.path.join(HERE, 'world_assets', f'{args.world}_terrain', 'mesh', 'collision.obj')) if l.startswith('v ')])
height = LinearNDInterpolator(v[:, :2], v[:, 2])


def river_d(x, y):
    d = np.hypot(river[:, 0] - x, river[:, 1] - y)
    k = int(np.argmin(d))
    return d[k] - (river[k, 2] - 1.4)


def side_of(x, y):
    return np.sign(y - np.interp(x, river[:, 0], river[:, 1]))


def near_crossing(x, y, r=4.0):
    for (fx, fy), (tx, ty) in zip(crossings, tangents):
        dx, dy = x - fx, y - fy
        if math.hypot(dx, dy) < r or (abs(dx * tx + dy * ty) < 3.2 and abs(-dx * ty + dy * tx) < 10):
            return True
    return False


rng = np.random.default_rng(args.seed)
mushrooms, natives = [], []
for _ in range(20000):
    if len(mushrooms) >= n_mush and len(natives) >= n_nat:
        break
    x, y = rng.uniform(-hedge_r + 3.5, hedge_r - 3.5, 2)
    k = len(mushrooms) + len(natives)
    if (side_of(x, y) != (1 if k % 2 == 0 else -1) or river_d(x, y) < 3.0 or near_crossing(x, y) or math.hypot(x, y) < 6
            or np.min(np.hypot(obstacles[:, 0] - x, obstacles[:, 1] - y)) < 1.8
            or any(math.hypot(x - m['x'], y - m['y']) < 4 for m in mushrooms + natives)):
        continue
    native = len(mushrooms) >= n_mush or (len(natives) < n_nat and k % 3 == 2)
    group = natives if native else mushrooms
    group.append({'name': f'{"native" if native else "mushroom"}_{len(group):02d}', 'x': round(float(x), 3),
                  'y': round(float(y), 3), 'z': round(float(height(x, y)), 3)})
if len(mushrooms) < n_mush or len(natives) < n_nat:
    raise SystemExit(f'only placed {len(mushrooms)} mushrooms, {len(natives)} natives')

name = f'{args.world}_m{args.seed}'
sdf = open(src + '.sdf').read()
sdf = re.sub(r'<include><uri>model://mushroom_(amanita|native)</uri>.*?</include>', '', sdf)
inc = ''.join(f'<include><uri>model://mushroom_{"native" if m in natives else "amanita"}</uri><name>{m["name"]}</name>'
              f'<pose>{m["x"]} {m["y"]} {m["z"]} 0 0 {rng.uniform(0, 6.283):.2f}</pose></include>'
              for m in natives + mushrooms)
sdf = sdf.replace(f'<include><uri>model://forest_baked_{args.world}</uri>', inc + f'<include><uri>model://forest_baked_{args.world}</uri>', 1)
sdf = sdf.replace(f'<world name="{args.world}">', f'<world name="{name}">', 1)  # mission nodes address the world by file name
meta.update(mushrooms=mushrooms, natives=natives)
meta['world']['name'] = name
out = os.path.join(HERE, 'worlds', name)
open(out + '.sdf', 'w').write(sdf)
json.dump(meta, open(out + '.meta.json', 'w'), indent=1)
print(f'{out}.sdf: ' + ', '.join(f'({m["x"]:.1f},{m["y"]:.1f})' for m in mushrooms))
