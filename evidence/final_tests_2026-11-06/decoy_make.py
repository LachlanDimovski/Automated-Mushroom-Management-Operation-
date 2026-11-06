import json, math, random, re, sys
import numpy as np
from PIL import Image as PI
REPO = sys.argv[1]; F = f'{REPO}/forest_worlds/worlds/'
s = open(F + 'river_moderate_aus_dt2.sdf').read(); m = json.load(open(F + 'river_moderate_aus_dt2.meta.json'))
td = f'{REPO}/forest_worlds/world_assets/river_moderate_aus_terrain'; t = open(td + '/model.sdf').read()
sx, sy, sz = map(float, re.search(r'<heightmap>.*?<size>([^<]+)</size>', t, re.S).group(1).split()); px, py, pz = map(float, re.search(r'<heightmap>.*?<pos>([^<]+)</pos>', t, re.S).group(1).split())
img = np.asarray(PI.open(td + '/heightmap.png').convert('L'), float) / 255 * sz + pz
def tz(x, y):
    n, c = img.shape; return float(img[int(round((sy / 2 - (y - py)) / sy * (n - 1))), int(round((x - px + sx / 2) / sx * (c - 1)))])
river = np.array(m['river']['line'], float); objs = [(o['x'], o['y'], o.get('radius', .5)) for o in m['objects'] if 'x' in o]
pts = [(q['x'], q['y']) for q in m['mushrooms'] + m['natives']]
rng = random.Random(41068); out = []
while len(out) < 6:
    x, y = rng.uniform(-24, 24), rng.uniform(-24, 24)
    if math.hypot(x, y) < 6 or np.min(np.hypot(river[:, 0] - x, river[:, 1] - y)) < 3.5: continue
    if any(math.hypot(x - a, y - b) < r + 1.0 for a, b, r in objs) or any(math.hypot(x - a, y - b) < 3 for a, b in pts + [(p[0], p[1]) for p in out]): continue
    out.append((round(x, 2), round(y, 2), 'dome' if len(out) % 2 == 0 else 'box'))
red = '<material><ambient>0.75 0.05 0.05 1</ambient><diffuse>0.8 0.06 0.06 1</diffuse></material>'; white = '<material><ambient>0.9 0.9 0.85 1</ambient><diffuse>0.9 0.9 0.85 1</diffuse></material>'
add = ''
for i, (x, y, k) in enumerate(out):
    z = tz(x, y)
    if k == 'dome':
        body = f'<visual name="stem"><pose>0 0 0.1 0 0 0</pose><geometry><cylinder><radius>0.05</radius><length>0.2</length></cylinder></geometry>{white}</visual><visual name="cap"><pose>0 0 0.22 0 0 0</pose><geometry><cylinder><radius>0.17</radius><length>0.1</length></cylinder></geometry>{red}</visual>'
    else:
        body = f'<visual name="b"><pose>0 0 0.13 0 0 0</pose><geometry><box><size>0.26 0.26 0.26</size></box></geometry>{red}</visual>'
    add += f'<model name="decoy_{i}"><static>true</static><pose>{x} {y} {z} 0 0 0</pose><link name="l">{body}</link></model>\n'
s = s.replace('name="river_moderate_aus_dt2"', 'name="river_moderate_aus_decoy"').replace('</world>', add + '</world>')
open(F + 'river_moderate_aus_decoy.sdf', 'w').write(s); json.dump(m, open(F + 'river_moderate_aus_decoy.meta.json', 'w'))
json.dump(out, open(sys.argv[2], 'w')); print(out)
