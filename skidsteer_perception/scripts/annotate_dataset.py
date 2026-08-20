#!/usr/bin/env python3
"""Draw a YOLO dataset's labels onto its images so they can be checked by eye.

Safe to run while capture_sim_dataset is still writing - it only reads, and
skips any image whose label file has not landed yet.

    python3 annotate_dataset.py [dataset_dir]

Writes <dataset_dir>/annotated/<split>_<name>.jpg and prints a sanity check:
what fraction of boxes actually contain red mushroom pixels. A good dataset
scores well above 90%. Anything low means the labels are misaligned - do not
train on it.
"""
import glob
import os
import sys

import cv2
import numpy as np

D = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1
                       else '~/mushroom_sim_dataset')
out = os.path.join(D, 'annotated')
os.makedirs(out, exist_ok=True)

# Red cap in HSV. Red wraps around hue 0/180 in OpenCV's 0-179 range.
LO1, HI1 = np.array([0, 80, 40]), np.array([12, 255, 255])
LO2, HI2 = np.array([168, 80, 40]), np.array([179, 255, 255])

n = tot = hit = 0
orphans = []
for f in sorted(glob.glob(f'{D}/images/*/*.jpg')):
    lf = f.replace('/images/', '/labels/').replace('.jpg', '.txt')
    if not os.path.exists(lf):
        continue                       # still being written
    img = cv2.imread(f)
    if img is None:
        continue
    H, W = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LO1, HI1) | cv2.inRange(hsv, LO2, HI2)
    covered = np.zeros_like(mask)

    for line in open(lf):
        p = line.split()
        if len(p) != 5:
            continue
        cx, cy, bw, bh = (float(v) for v in p[1:])
        x1, y1 = max(0, int((cx - bw / 2) * W)), max(0, int((cy - bh / 2) * H))
        x2, y2 = int((cx + bw / 2) * W), int((cy + bh / 2) * H)
        tot += 1
        # does this box actually contain any mushroom?
        red = mask[y1:max(y1 + 1, y2), x1:max(x1 + 1, x2)].sum() > 0
        if red:
            hit += 1
        covered[y1:y2, x1:x2] = 255
        cv2.rectangle(img, (x1, y1), (x2, y2),
                      (0, 255, 0) if red else (0, 0, 255), 2)

    # red pixels no box covers = unlabelled mushrooms, the damaging error
    orphans.append(int(((mask > 0) & (covered == 0)).sum()))
    split = f.split('/images/')[1].split('/')[0]
    cv2.imwrite(f'{out}/{split}_{os.path.basename(f)[:-4]}.jpg', img,
                [cv2.IMWRITE_JPEG_QUALITY, 90])
    n += 1

print(f'annotated {n} frames -> {out}')
if tot:
    print(f'  boxes: {tot}, containing a mushroom: {hit} ({100 * hit / tot:.0f}%)'
          '   [green = has mushroom, red = empty box]')
    print(f'  unlabelled red pixels per frame: median '
          f'{int(np.median(orphans))}, max {max(orphans)}')
    if hit / tot < 0.9:
        print('  WARNING: labels look misaligned - do not train on this.')
