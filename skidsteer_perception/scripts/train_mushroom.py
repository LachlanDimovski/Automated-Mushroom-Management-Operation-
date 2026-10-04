#!/usr/bin/env python3
"""Fine-tune YOLO11 on the Gazebo-captured Amanita muscaria dataset.

Runs on the training PC, not the laptop - it needs a CUDA GPU. The dataset
bundle produced by package_dataset.sh unpacks next to this file:

    mushroom_bundle/
      data.yaml
      images/{train,val}/
      labels/{train,val}/
      train_mushroom.py     <- this file
      README.md

Usage:

    python3 train_mushroom.py                     # single GPU, yolo11n
    python3 train_mushroom.py --model yolo11s.pt  # bigger model
    python3 train_mushroom.py --device 0,1        # both 3060s (see README)
"""
import argparse
import os

from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model',
                    default=os.path.join(HERE, 'amanita_realphoto_yolo11n.pt'),
                    help='Starting weights. Defaults to the real-photo model '
                         'shipped in this bundle, which is what the 0.972 run '
                         'started from; pass yolo11n.pt for a clean slate')
    ap.add_argument('--data', default=os.path.join(HERE, 'data.yaml'))
    ap.add_argument('--epochs', type=int, default=200)
    ap.add_argument('--imgsz', type=int, default=640,
                    help='Match this to inference. perception/yolo_node.py '
                         'calls predict() without imgsz, so it runs at the '
                         'ultralytics default of 640 - training at anything '
                         'else means the model sees a different object scale '
                         'in the sim than it trained on. Raise both together '
                         'or neither.')
    ap.add_argument('--batch', type=int, default=16,
                    help='Per the README: autobatch (-1) is single-GPU only')
    ap.add_argument('--device', default='0')
    ap.add_argument('--name', default='mushroom')
    args = ap.parse_args()

    YOLO(args.model).train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        name=args.name,
        patience=50,
        cache=True,          # dataset is small; keep it in RAM

        # --- augmentation, tuned for this target --------------------------
        # The detector keys on a red cap with white warts, so hue is the one
        # thing not to jitter - shifting it turns fly agarics into some other
        # fungus. Saturation and brightness swing wide instead, which is what
        # actually varies between Gazebo lighting and any later domain.
        hsv_h=0.010,
        hsv_s=0.7,
        hsv_v=0.4,
        # Mosaic tiles four images into one, shrinking objects further - the
        # standard trick for teaching small-object detection. Closed off for
        # the last 10 epochs so training ends on real framing.
        mosaic=1.0,
        close_mosaic=10,
        scale=0.5,
        fliplr=0.5,
        flipud=0.0,          # mushrooms do not appear upside down
        degrees=5.0,
    )


if __name__ == '__main__':
    main()
