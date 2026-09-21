#!/usr/bin/env bash
# Bundle a captured dataset + the training script + instructions into one
# tarball to copy to the training PC.
#
#   ./package_dataset.sh [dataset_dir] [output.tar.gz]
#
# Defaults to ~/mushroom_sim_dataset -> ~/mushroom_bundle.tar.gz
set -euo pipefail

SRC="${1:-$HOME/mushroom_sim_dataset}"
OUT="${2:-$HOME/mushroom_bundle.tar.gz}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

[ -d "$SRC/images/train" ] || { echo "no dataset at $SRC - run the capture first"; exit 1; }

STAGE="$(mktemp -d)/mushroom_bundle"
mkdir -p "$STAGE"

# Dataset proper. Skip annotated/ - it is a human-readable debug artefact,
# not training input, and it roughly doubles the tarball.
cp -r "$SRC/images" "$SRC/labels" "$STAGE/"
cp "$SRC/data.yaml" "$STAGE/"
cp "$HERE/train_mushroom.py" "$HERE/TRAINING_PC.md" "$STAGE/"

# The starting checkpoint travels with the data. Training from the real-photo
# weights rather than stock yolo11n is what let the last run converge in 40
# epochs, and it keeps this run comparable to that one.
WEIGHTS="$HERE/../../perception/models/variants/amanita_realphoto_yolo11n.pt"
[ -f "$WEIGHTS" ] && cp "$WEIGHTS" "$STAGE/" || echo "warning: starting weights not found at $WEIGHTS"

TRAIN=$(find "$STAGE/images/train" -type f | wc -l)
VAL=$(find "$STAGE/images/val" -type f | wc -l)
BOXES=$(cat "$STAGE"/labels/*/*.txt 2>/dev/null | wc -l)
cat > "$STAGE/MANIFEST.txt" <<EOF
Captured : $(date -Iseconds)
From     : $SRC
Train    : $TRAIN images
Val      : $VAL images
Boxes    : $BOXES
Capture geometry (must match what the model is later deployed against):
$(grep -E "^(IMG_W|HFOV|MIN_RANGE_M)" "$HERE/../skidsteer_perception/capture_sim_dataset.py" || true)
EOF

tar czf "$OUT" -C "$(dirname "$STAGE")" mushroom_bundle
rm -rf "$(dirname "$STAGE")"
echo "wrote $OUT  ($(du -h "$OUT" | cut -f1))  -  $TRAIN train / $VAL val / $BOXES boxes"
echo "copy it over with:  scp $OUT <user>@<pc>:~/"
