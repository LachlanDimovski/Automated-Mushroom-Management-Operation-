#!/usr/bin/env bash
# Capture a training set end to end: start the sim, capture, verify, prune.
#
#   ./run_capture.sh                                  # 600 mushroom frames
#   ./run_capture.sh --frames 400 --out ~/mydata
#   ./run_capture.sh --target walking_person --frames 400
#   ./run_capture.sh --platform parrot1 --hover 2.5
#   ./run_capture.sh --world river_moderate_aus --target mushroom_native   # forest_worlds world, on its terrain
#   ./run_capture.sh ... --spots banks.json          # camera spots [[x, y], ...] instead of the world's mushroom spots
#
# One sim at a time, deliberately. Running two in parallel halved the frame
# rate and eventually stalled one of them.
set -uo pipefail

TARGET=mushroom_amanita
PLATFORM=husky1
FRAMES=600
REPOSITION=5          # frames per camera pose; low = more distinct scenes
HOVER=""
WORLD=simple_trees
OUT=""
SPOTS=""
SEED=""

while [ $# -gt 0 ]; do
    case "$1" in
        --target)     TARGET=$2; shift 2 ;;
        --platform)   PLATFORM=$2; shift 2 ;;
        --frames)     FRAMES=$2; shift 2 ;;
        --reposition) REPOSITION=$2; shift 2 ;;
        --hover)      HOVER=$2; shift 2 ;;
        --world)      WORLD=$2; shift 2 ;;
        --out)        OUT=$2; shift 2 ;;
        --spots)      SPOTS=$2; shift 2 ;;
        --seed)       SEED=$2; shift 2 ;;
        *) echo "unknown option: $1"; exit 1 ;;
    esac
done
OUT=${OUT:-$HOME/${TARGET}_${PLATFORM}_$(date +%m%d_%H%M)}

WS="$HOME/41068_ws"
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
S="$REPO/skidsteer_perception/scripts"
LOG=/tmp/ammo_capture
mkdir -p "$LOG"

# colcon's setup.bash reads COLCON_TRACE without a default, which
# aborts under `set -u`. Relax it just for the sourcing.
set +u; source "$WS/install/setup.bash"; set -u

# forest_worlds worlds need their own models on the Gazebo path
F="$REPO/forest_worlds"
export IGN_GAZEBO_RESOURCE_PATH="$(ls -d "$F"/models/* "$F"/world_assets 2>/dev/null | tr '\n' ':')${IGN_GAZEBO_RESOURCE_PATH:-}"

FW="$REPO/forest_worlds/worlds/$WORLD.sdf"
LAUNCH_WORLD=$WORLD; [ -f "$FW" ] && LAUNCH_WORLD=${FW%.sdf}

cleanup() { pkill -f "ign gazebo"; pkill -f parameter_bridge; pkill -f ruby; }
trap cleanup EXIT INT TERM

# A leftover server answering on the same world name makes `ign` requests
# alternate between simulations at random, which silently mislabels frames.
echo "clearing any running sim..."
cleanup; sleep 3
pgrep -f "ign gazebo" >/dev/null && { echo "could not clear Gazebo - do it by hand"; exit 1; }

echo "starting sim (headless, no detectors - capture does not use them)..."
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py \
    husky:="$( [ "$PLATFORM" = husky1 ] && echo True || echo False )" \
    parrot:="$( [ "$PLATFORM" = parrot1 ] && echo True || echo False )" \
    world:="$LAUNCH_WORLD" rviz:=False headless:=True yolo:=False hazard:=False \
    > "$LOG/sim.log" 2>&1 &

for i in $(seq 120); do   # heavy forest worlds take minutes to load
    sleep 2
    timeout 10 ign model --list 2>/dev/null | grep -q "$PLATFORM" && break
    [ "$i" = 120 ] && { echo "$PLATFORM never appeared - see $LOG/sim.log"; exit 1; }
done
echo "  $PLATFORM up"

rm -rf "$OUT"
ARGS=(-p target:="$TARGET" -p platform:="$PLATFORM" -p num_frames:="$FRAMES"
      -p reposition_every:="$REPOSITION" -p out_dir:="$OUT")
[ -n "$HOVER" ] && ARGS+=(-p hover_z:="$HOVER")
[ -n "$SPOTS" ] && ARGS+=(-p spots_file:="$SPOTS")
[ -n "$SEED" ] && ARGS+=(-p seed:="$SEED")
[ -f "$FW" ] && ARGS+=(-p world_sdf:="$FW" -p world:="$(grep -o '<world name="[^"]*' "$FW" | cut -d'"' -f2)")

echo "capturing $FRAMES frames of $TARGET -> $OUT"
ros2 run skidsteer_perception capture_sim_dataset --ros-args "${ARGS[@]}" \
    2>&1 | tee "$LOG/capture.log"

[ -d "$OUT/images/train" ] || { echo "capture produced nothing"; exit 1; }

# the red-pixel checks below only suit the red amanita; the capture's depth test already drops occluded boxes
if [ "$TARGET" != mushroom_amanita ]; then echo; echo "done -> $OUT"; exit 0; fi

echo; echo "verifying labels..."
python3 "$S/annotate_dataset.py" "$OUT" | tail -3
echo; echo "pruning boxes with nothing visible in them..."
python3 "$S/prune_occluded.py" "$OUT"
echo; echo "re-checking..."
python3 "$S/annotate_dataset.py" "$OUT" | grep "boxes:"

echo
echo "done -> $OUT"
echo "next:  $S/merge_datasets.sh ~/combined <this> <other sets...>"
