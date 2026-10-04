#!/usr/bin/env python3
"""Fine-tune the deployed model on close-range frames, without forgetting.

The v3 model tops out at 86 px because nothing in its training data was
larger - the capture floor put the nearest mushroom 0.8 m from the lens. Drive
the Husky up to one and it fills 100-200 px, which the model has never seen.

Rather than recapture and retrain from scratch, this fine-tunes the existing
weights on a small close-range set. Two things keep it from simply forgetting
everything else:

  * The backbone is frozen (`freeze`). Only the detection head adapts, so the
    features that already work at 10 px are left alone.
  * The close-range frames are mixed with a sample of the original data
    (`--mix`), so every batch still contains distant targets. Training on
    close-ups alone is what would wreck it.

Learning rate is an order of magnitude below a fresh run for the same reason.

    ./finetune_closeups.py --close ~/mushroom_closeups
    ./finetune_closeups.py --close ~/mushroom_closeups --device 0 --mix 600
"""
import argparse
import os
import random
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', '..', 'perception', 'models')


def _count_frames(root):
    n = 0
    for split in ('train', 'val'):
        d = os.path.join(root, 'images', split)
        if os.path.isdir(d):
            n += sum(1 for f in os.listdir(d) if f.endswith('.jpg'))
    return n


def link_split(src, dst, split, names):
    for stem in names:
        for kind, ext in (('images', '.jpg'), ('labels', '.txt')):
            s = os.path.join(src, kind, split, stem + ext)
            if os.path.exists(s):
                shutil.copy(s, os.path.join(dst, kind, split, stem + ext))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--close', default=os.path.expanduser('~/mushroom_closeups'),
                    help='close-range capture to fine-tune on')
    ap.add_argument('--base', default=os.path.expanduser('~/mushroom_sim_v3'),
                    help='the set the current model was trained on')
    ap.add_argument('--weights', default=os.path.join(
        MODELS, 'amanita_muscaria_yolo11n.pt'))
    ap.add_argument('--out', default=os.path.expanduser('~/mushroom_finetune'))
    ap.add_argument('--mix', type=int, default=250,
                    help='how many original frames to keep in the mix; 0 means '
                         'close-ups only, which will forget')
    ap.add_argument('--epochs', type=int, default=30)
    ap.add_argument('--freeze', type=int, default=10,
                    help='freeze the first N layers (the backbone)')
    ap.add_argument('--lr0', type=float, default=0.001)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--batch', type=int, default=16)
    ap.add_argument('--device', default='cpu')
    args = ap.parse_args()

    # --- assemble the mixed set -------------------------------------------
    shutil.rmtree(args.out, ignore_errors=True)
    for split in ('train', 'val'):
        for kind in ('images', 'labels'):
            os.makedirs(os.path.join(args.out, kind, split), exist_ok=True)

    rng = random.Random(0)
    total = 0
    for src, n in ((args.close, None), (args.base, args.mix)):
        for split in ('train', 'val'):
            d = os.path.join(src, 'images', split)
            if not os.path.isdir(d):
                continue
            stems = sorted(f[:-4] for f in os.listdir(d) if f.endswith('.jpg'))
            if n is not None:
                # Take the same fraction from each split, so val stays
                # representative of what the model is actually judged on.
                frac = min(1.0, n / max(1, _count_frames(src)))
                k = max(1, round(frac * len(stems)))
                stems = rng.sample(stems, min(k, len(stems)))
            link_split(src, args.out, split, stems)
            total += len(stems)

    with open(os.path.join(args.out, 'data.yaml'), 'w') as fh:
        fh.write('train: images/train\nval: images/val\n\n'
                 'nc: 1\nnames:\n  0: amanita_muscaria_red\n')

    n_tr = len(os.listdir(os.path.join(args.out, 'images', 'train')))
    n_va = len(os.listdir(os.path.join(args.out, 'images', 'val')))
    print(f'mixed set: {n_tr} train / {n_va} val  -> {args.out}')
    print(f'starting from: {args.weights}')
    print(f'freeze={args.freeze}  lr0={args.lr0}  epochs={args.epochs}  '
          f'device={args.device}\n')

    from ultralytics import YOLO
    YOLO(args.weights).train(
        data=os.path.join(args.out, 'data.yaml'),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        freeze=args.freeze,
        lr0=args.lr0,
        lrf=0.1,
        warmup_epochs=1.0,
        # Mosaic shrinks objects by tiling four images into one, which works
        # against the whole point of this run. Off.
        mosaic=0.0,
        scale=0.3,
        hsv_h=0.010,
        hsv_s=0.7,
        hsv_v=0.4,
        fliplr=0.5,
        flipud=0.0,
        name='finetune',
        project=os.path.join(args.out, 'runs'),
    )


if __name__ == '__main__':
    main()
