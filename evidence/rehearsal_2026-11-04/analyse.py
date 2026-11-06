#!/usr/bin/env python3
"""Recompute Tables 23 and 25 of the C5 report from the logs in this folder.

Usage: python3 analyse.py [path to river_moderate_aus_dt2.meta.json]
The world file comes with `bash demo/get_world.sh`.
"""
import glob, gzip, json, math, os, re, statistics as st, sys

here = os.path.dirname(os.path.abspath(__file__))
meta = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, '../../forest_worlds/worlds/river_moderate_aus_dt2.meta.json')
truth = [(m['x'], m['y']) for m in json.load(open(meta))['mushrooms']]
nearest = lambda x, y: min(math.hypot(x - a, y - b) for a, b in truth)
reports, sightings, wall, sim, aligns, on_camera = [], [], [], [], 0, 0

for path in sorted(glob.glob(os.path.join(here, 'logs/run*_sim.log.gz'))):
    lines = gzip.open(path, 'rt', errors='ignore').read().splitlines()
    text = '\n'.join(lines)
    reports += [nearest(float(a), float(b)) for a, b in re.findall(r'Mushroom M\d+ at \((-?[\d.]+),(-?[\d.]+)\)', text)]
    sightings += [nearest(float(a), float(b)) for a, b in re.findall(r'vis: locked \(([+-][\d.]+),([+-][\d.]+)\)', text)]
    inside = used = False
    for l in lines:
        if '-> ALIGN' in l:
            on_camera += inside and used
            inside, used = True, False
            aligns += 1
        elif 'ALIGN steering on camera' in l and inside:
            used = True
        elif re.search(r'-> (LOWER|DRIVE|NAV)', l) and inside:
            on_camera += used
            inside = False
    on_camera += inside and used
    m = re.search(r'after (\d+) s from sim start, sim (\d+) s', open(path.replace('_sim.log.gz', '_timer.out'), errors='ignore').read())
    wall.append(int(m[1]))
    sim.append(int(m[2]))

def show(name, v):
    v = sorted(v)
    print(f'{name}: n={len(v)} median={st.median(v):.3f} mean={st.mean(v):.3f} p95={v[int(.95 * len(v))]:.2f} '
          f'max={v[-1]:.2f} within 0.5 m={100 * sum(e <= .5 for e in v) / len(v):.0f}%')

print(f'runs={len(wall)} wall {min(wall)} to {max(wall)} s, median {st.median(wall)}; sim {min(sim)} to {max(sim)} s, median {st.median(sim)}')
show('quadcopter first report', reports)
show('rover camera, live', sightings)
print(f'ALIGN {aligns}, steered by camera {on_camera}')
