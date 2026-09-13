#!/usr/bin/env python3
"""Drop labels whose box contains no visible mushroom.

The capture script projects a mushroom's known world position into the image.
That is exact, but it says nothing about whether the mushroom can be SEEN - a
target behind a tree trunk still gets a perfectly placed box over bark, which
teaches the detector that bark is a mushroom. Captures made before the depth
occlusion filter existed carry a few percent of these.

This is the same test annotate_dataset.py reports, applied as a fix: a box
with no red cap pixels in it is removed. A frame left with no boxes is removed
entirely, matching what the capture script itself does.

    ./prune_occluded.py <dataset_dir> [--dry-run]

Boxes are judged on red pixel COUNT, not presence, because JPEG ringing around
a dark trunk can produce one or two stray red pixels. Requiring a small
cluster avoids keeping a box on that basis - while staying far below what even
the smallest genuine mushroom in these sets shows.
"""
import glob
import os
import shutil
import sys

import cv2
import numpy as np

LO1, HI1 = np.array([0, 80, 40]), np.array([12, 255, 255])
LO2, HI2 = np.array([168, 80, 40]), np.array([179, 255, 255])
MIN_RED_PX = 6

D = os.path.expanduser(sys.argv[1])
dry = '--dry-run' in sys.argv
removed_boxes = removed_frames = kept_boxes = 0

for f in sorted(glob.glob(f'{D}/images/*/*.jpg')):
    lf = f.replace('/images/', '/labels/').replace('.jpg', '.txt')
    if not os.path.exists(lf):
        continue
    img = cv2.imread(f)
    if img is None:
        continue
    H, W = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LO1, HI1) | cv2.inRange(hsv, LO2, HI2)

    keep = []
    for line in open(lf):
        p = line.split()
        if len(p) != 5:
            continue
        cx, cy, bw, bh = (float(v) for v in p[1:])
        x1, y1 = max(0, int((cx - bw / 2) * W)), max(0, int((cy - bh / 2) * H))
        x2, y2 = int((cx + bw / 2) * W), int((cy + bh / 2) * H)
        if mask[y1:max(y1 + 1, y2), x1:max(x1 + 1, x2)].sum() // 255 >= MIN_RED_PX:
            keep.append(line.rstrip('\n'))
        else:
            removed_boxes += 1

    if not keep:
        removed_frames += 1
        if not dry:
            os.remove(f)
            os.remove(lf)
        continue

    kept_boxes += len(keep)
    if not dry:
        with open(lf, 'w') as fh:
            fh.write('\n'.join(keep) + '\n')

shutil.rmtree(f'{D}/annotated', ignore_errors=True)
print(f'{"would remove" if dry else "removed"}: {removed_boxes} occluded boxes, '
      f'{removed_frames} frames left with nothing to label')
print(f'kept: {kept_boxes} boxes')
