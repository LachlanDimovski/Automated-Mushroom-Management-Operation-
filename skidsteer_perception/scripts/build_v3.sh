#!/usr/bin/env bash
# Everything from an empty terminal to a bundle ready to scp:
# capture -> verify -> prune -> merge with the existing sets -> package.
#
#   ./build_v3.sh
#   ./build_v3.sh --frames 400
#
# Takes about 45 minutes, most of it the capture. Safe to Ctrl-C: the capture
# cleans up Gazebo, and nothing is merged until the capture has finished.
set -euo pipefail

FRAMES=600
[ "${1:-}" = "--frames" ] && FRAMES=$2

WS="$HOME/41068_ws"
S="$WS/src/RS1_Team5_/skidsteer_perception/scripts"
# Overridable so the pipeline can be rehearsed on scratch paths without
# touching the real datasets or the bundle.
NEW="${NEW:-$HOME/mushroom_sim_near}"
V3="${V3:-$HOME/mushroom_sim_v3}"
BUNDLE="${BUNDLE:-$HOME/mushroom_bundle.tar.gz}"
BASE_GROUND="${BASE_GROUND:-$HOME/mushroom_sim_dataset}"
BASE_AIR="${BASE_AIR:-$HOME/mushroom_sim_dataset_air}"

for d in "$BASE_GROUND" "$BASE_AIR"; do
    [ -d "$d/images/train" ] || { echo "missing existing dataset: $d"; exit 1; }
done

echo "=============================================================="
echo " 1/3  capturing $FRAMES frames"
echo "=============================================================="
"$S/run_capture.sh" --frames "$FRAMES" --out "$NEW"

echo
echo "=============================================================="
echo " 2/3  merging with the existing ground and aerial sets"
echo "=============================================================="
"$S/merge_datasets.sh" "$V3" "$BASE_GROUND" "$BASE_AIR" "$NEW"

echo
echo "=============================================================="
echo " 3/3  packaging"
echo "=============================================================="
"$S/package_dataset.sh" "$V3" "$BUNDLE"

echo
echo "=============================================================="
echo " done"
echo "=============================================================="
echo "  send it:   scp $BUNDLE <user>@<pc>:~/"
echo "  then:      tar xzf mushroom_bundle.tar.gz && cd mushroom_bundle"
echo "             python3 train_mushroom.py --imgsz 640"
