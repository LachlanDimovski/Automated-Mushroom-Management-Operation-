# Training the mushroom detector on the GPU box

Unpack, set up once, train. Everything below runs on the PC (Ubuntu 26,
2x RTX 3060) — nothing here needs ROS.

```bash
tar xzf mushroom_bundle.tar.gz && cd mushroom_bundle
cat MANIFEST.txt          # confirm the capture geometry matches your sim
```

## 1. Environment

Ubuntu 26 ships a Python newer than the PyTorch wheels usually support, and
`pip install torch` against an unsupported interpreter fails in a confusing
way — it resolves to a CPU-only build, or to nothing. Check first:

```bash
python3 --version
```

**If 3.12 or older**, a plain venv is fine:

```bash
python3 -m venv ~/yolo-env
source ~/yolo-env/bin/activate
pip install --upgrade pip
pip install ultralytics
```

**If 3.13 or newer**, pin an interpreter PyTorch actually builds for:

```bash
sudo apt install python3.12-venv
python3.12 -m venv ~/yolo-env
source ~/yolo-env/bin/activate
pip install --upgrade pip ultralytics
```

Then confirm CUDA is real, not a CPU fallback:

```bash
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.device_count())"
nvidia-smi
```

You want `True 2`. If it says `False`, you have a CPU build — reinstall with
the CUDA index URL from pytorch.org rather than debugging around it.

## 2. Train

The venv has to be active in whatever shell you train from - a new terminal
will not have it.

```bash
source ~/yolo-env/bin/activate
python3 train_mushroom.py
```

Defaults: yolo11n, 200 epochs, imgsz 960, batch 16, GPU 0. Results land in
`runs/detect/mushroom/`, with the weights you want at
`runs/detect/mushroom/weights/best.pt`.

### Use one GPU first

Both 3060s is `--device 0,1`, but **try a single GPU before reaching for it.**
Ultralytics uses DistributedDataParallel across GPUs, which synchronises
gradients every step over PCIe. yolo11n is 2.6 M parameters — small enough
that on a few hundred images the sync cost can eat most of the second card's
benefit. Expect roughly 1.4–1.6x, not 2x.

A single 3060 will finish a few-hundred-image run in well under an hour.
Reach for both cards when you move to yolo11s/m or a much larger dataset.

Two DDP gotchas if you do use `--device 0,1`:

- **Autobatch (`--batch -1`) does not work under DDP.** Pass an explicit
  number. `batch` is the *total* across GPUs, so `--batch 32` puts 16 on each.
- If a run dies, DDP can leave orphaned processes holding VRAM. Check with
  `nvidia-smi` and clear them before relaunching.

### imgsz must not be below the capture width

`--imgsz` defaults to 960. If your capture is 1440 wide, train at 1440 — the
whole point of the exercise is pixels on target, and training smaller throws
them away again. Whatever you pick here, **use the same value at inference**,
or the model sees a different object scale than it trained on.

## 3. Check it before trusting it

```
runs/detect/mushroom/
  results.png            loss curves + mAP over epochs
  confusion_matrix.png
  val_batch0_pred.jpg    predictions on held-out frames — look at these
```

`results.png` is the one to read. Box loss still falling at epoch 200 means
train longer; val mAP peaking early then sagging means overfitting, so add
data rather than epochs.

Single-class mAP50 above ~0.85 on sim data is a reasonable bar. Much higher
than that on a few hundred frames usually means the val split is too similar
to train — the capture script splits by frame index, and consecutive frames
share a camera pose.

## 4. Back to the laptop

```bash
scp runs/detect/mushroom/weights/best.pt <user>@<laptop>:~/
```

On the laptop, drop it in and rebuild:

```bash
cp ~/best.pt ~/41068_ws/src/AMMO-Mushroom-Fleet/skidsteer_perception/models/amanita_muscaria_yolo11n.pt
cd ~/41068_ws && colcon build --packages-select skidsteer_perception perception
```

`perception/perception/yolo_node.py:30` points at that filename, so replacing
it in place needs no code change. Keep the old weights under a different name
until the new ones are measured — a retrain that scores worse is a normal
outcome, not a failure, and you want to be able to go back.
