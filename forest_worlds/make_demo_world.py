"""Copy of a river world keeping only some of its mushrooms, for a short live demo.

python3 make_demo_world.py --world river_moderate_aus --keep 1,4,6,7,9,11  ->  worlds/river_moderate_aus_demo6.sdf
--add 2.3,-9.8 --name stuck  adds a mushroom there (ground height from the terrain mesh)
"""
import argparse
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))

ap = argparse.ArgumentParser()
ap.add_argument('--world', default='river_moderate_aus')
ap.add_argument('--keep', default='1,4,6,7,9,11')
ap.add_argument('--add', action='append', default=[])
ap.add_argument('--name', default='')
args = ap.parse_args()

keep = {f'mushroom_{int(i):02d}' for i in args.keep.split(',')}
src = os.path.join(HERE, 'worlds', args.world)
name = f'{args.world}_{args.name or "demo" + str(len(keep))}'
out = os.path.join(HERE, 'worlds', name)

sdf = open(src + '.sdf').read()
sdf = re.sub(r'<include><uri>model://mushroom_amanita</uri><name>(mushroom_\d+)</name>.*?</include>',
             lambda m: m.group(0) if m.group(1) in keep else '', sdf)
sdf = re.sub(r'<world name="[^"]*"', f'<world name="{name}"', sdf, count=1)
added = []
if args.add:
    import numpy as np
    from scipy.interpolate import LinearNDInterpolator
    v = np.array([list(map(float, l.split()[1:4])) for l in open(os.path.join(
        HERE, 'world_assets', f'{args.world}_terrain', 'mesh', 'collision.obj')) if l.startswith('v ')])
    height = LinearNDInterpolator(v[:, :2], v[:, 2])
    for i, xy in enumerate(args.add):
        x, y = map(float, xy.split(','))
        added.append({'name': f'mushroom_{90 + i}', 'x': x, 'y': y, 'z': round(float(height(x, y)), 3)})
    inc = ''.join(f'<include><uri>model://mushroom_amanita</uri><name>{m["name"]}</name>'
                  f'<pose>{m["x"]} {m["y"]} {m["z"]} 0 0 0</pose></include>' for m in added)
    sdf = sdf.replace('<include><uri>model://forest_baked_', inc + '<include><uri>model://forest_baked_', 1)
open(out + '.sdf', 'w').write(sdf)

meta = json.load(open(src + '.meta.json'))
meta['mushrooms'] = [m for m in meta['mushrooms'] if m['name'] in keep] + added
meta['world']['name'] = name
json.dump(meta, open(out + '.meta.json', 'w'), indent=1)
print(f'{out}.sdf: ' + ', '.join(f'({m["x"]:.1f},{m["y"]:.1f})' for m in meta['mushrooms']))
