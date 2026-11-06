#!/usr/bin/env python3
"""Recompute Tables 27 and 28 of the C5 report from r1.csv and r12.csv."""
import csv, os, statistics as st

here = os.path.dirname(os.path.abspath(__file__))
rows = list(csv.DictReader(open(os.path.join(here, 'r1.csv'))))
for rng in ('1.0', '2.0', '3.0', '4.0'):
    placed = [r for r in rows if r['range'] == rng]
    clear = [r for r in placed if r['blocked'] == '0']
    hits = [r for r in clear if float(r['best_conf']) >= .25]
    recall = lambda t: 100 * sum(float(r['best_conf']) >= t for r in clear) / len(clear)
    print(f"{rng} m: placements {len(placed)}, clear {len(clear)}, recall {recall(.25):.0f}% at 0.25, {recall(.6):.0f}% at 0.6, "
          f"median box {st.median(float(r['bw']) for r in hits):.1f} px")
rows = list(csv.DictReader(open(os.path.join(here, 'r12.csv'))))
for obstacle in ('wall', 'pole', 'low_box'):
    for path in ('raw', 'nav'):
        gaps = [float(r['min_base_gap']) for r in rows if r['obstacle'] == obstacle and r['path'] == path]
        print(f'{obstacle} {path}: closest {min(gaps):.2f} m, median {st.median(gaps):.2f} m, runs {len(gaps)}')
print('runs at 1.0 m or more:', sum(float(r['min_base_gap']) >= 1 for r in rows), 'of', len(rows))
