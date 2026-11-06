#!/usr/bin/env python3
"""Score the 7 November layout runs: one 2 + 1 fleet run on each released shuffled layout.

Usage: python3 score_layouts.py [path to forest_worlds/worlds]
The world files come with `bash demo/get_world.sh`.
"""
import gzip, json, math, os, re, statistics as st, sys

here = os.path.dirname(os.path.abspath(__file__))
worlds = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, '../../forest_worlds/worlds')

def read(path):
    if os.path.exists(path + '.gz'):
        return gzip.open(path + '.gz', 'rt', errors='ignore').read()
    return open(path, errors='ignore').read() if os.path.exists(path) else ''

errs = []
for w in ('m1', 'm2', 'm3'):
    meta = json.load(open(os.path.join(worlds, f'river_moderate_aus_{w}.meta.json')))
    truth = [(m['x'], m['y']) for m in meta['mushrooms']]
    nat = [(m['x'], m['y']) for m in meta['natives']]
    log = read(os.path.join(here, w, 'part1_sim.log'))
    near = lambda x, y, pts: min(math.hypot(x - a, y - b) for a, b in pts)
    reports = [(float(a), float(b)) for a, b in re.findall(r'Mushroom M\d+ at \((-?[\d.]+),(-?[\d.]+)\)', log)]
    e = [near(x, y, truth) for x, y in reports]
    errs += e
    logged = [(float(a), float(b)) for a, b in re.findall(r'native mushroom \d+ logged at \((-?[\d.]+), ?(-?[\d.]+)\)', log)]
    real = {min(range(len(nat)), key=lambda i: math.hypot(x - nat[i][0], y - nat[i][1])) for x, y in logged if near(x, y, nat) <= 1.5}
    false = sum(near(x, y, nat) > 1.5 for x, y in logged)
    onnat = sum(near(x, y, nat) < 0.75 and near(x, y, truth) > 0.75 for x, y in reports)
    coll = sum(int(c) for c in re.findall(r'Collected (\d+) mushroom\(s\)', log))
    t = re.search(r'after (\d+) s from sim start, sim (\d+) s', open(os.path.join(here, w, 'timer.out'), errors='ignore').read())
    print(f'{w}: collected {coll}/12, wall {t[1] if t else "-"} s, sim {t[2] if t else "-"} s, reports {len(reports)}, '
          f'median error {st.median(e) if e else 0:.3f} m, max {max(e) if e else 0:.2f} m, natives {len(real)}/6, '
          f'false natives {false}, reports on a native {onnat}')
print(f'all reports: n={len(errs)} mean={st.mean(errs):.3f} max={max(errs):.2f} within 0.5 m={100 * sum(x <= .5 for x in errs) / len(errs):.0f}%')
