#!/usr/bin/env bash
# Capture the Husky and Parrot training sets side by side.
#
#   ./capture_both.sh [frames] [parrot_hover_m]
#   ./capture_both.sh 800 3.0
#
# The two captures CANNOT share a Gazebo world: both name their mushrooms
# capmush_N and each deletes any mushroom it does not own, so they would
# fight over the same models and mislabel every frame. Each lane therefore
# gets its own simulation, isolated by IGN_PARTITION (Ignition transport)
# and ROS_DOMAIN_ID (ROS 2). Both are required - one alone still leaks.
set -uo pipefail

FRAMES=${1:-800}
HOVER=${2:-3.0}
WS="$HOME/41068_ws"
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
LOGS="/tmp/ammo_capture"

mkdir -p "$LOGS"

# --- pre-flight -------------------------------------------------------------
# A leftover server answering on the same world name makes `ign` requests
# round-robin between simulations at random. That produced silently wrong
# labels once already, so refuse to start rather than risk it.
if pgrep -f "ign gazebo" > /dev/null; then
    echo "A Gazebo server is already running. Clear it first:"
    echo '    pkill -f "ign gazebo"; pkill -f parameter_bridge; pkill -f ruby'
    exit 1
fi

PIDS=()
cleanup() {
    echo
    echo "stopping..."
    for pid in "${PIDS[@]}"; do kill -- "-$pid" 2>/dev/null; done
    pkill -f "ign gazebo"; pkill -f parameter_bridge; pkill -f ruby
}
trap cleanup EXIT INT TERM

# --- one lane: isolated sim + capture ---------------------------------------
lane() {
    local name=$1 partition=$2 domain=$3 robot=$4 outdir=$5
    shift 5

    export IGN_PARTITION="$partition" GZ_PARTITION="$partition"
    export ROS_DOMAIN_ID="$domain"
    # colcon's setup.bash reads COLCON_TRACE without a default, which
# aborts under `set -u`. Relax it just for the sourcing.
set +u; source "$WS/install/setup.bash"; set -u

    rm -rf "$outdir"

    ros2 launch 41068_ignition_bringup 41068_ignition.launch.py \
        husky:="$( [ "$robot" = husky1 ] && echo True || echo False )" \
        parrot:="$( [ "$robot" = parrot1 ] && echo True || echo False )" \
        rviz:=False headless:=True > "$LOGS/${name}_sim.log" 2>&1 &

    # Gazebo, the bridge and the spawn timers need to settle before the
    # capture reads a pose. 25 s is generous; the model check below is what
    # actually gates it.
    sleep 25
    if ! timeout 15 ign model --list 2>/dev/null | grep -q "$robot"; then
        echo "[$name] $robot never appeared - see $LOGS/${name}_sim.log"
        return 1
    fi

    echo "[$name] sim up, capturing $FRAMES frames -> $outdir"
    ros2 run skidsteer_perception capture_sim_dataset --ros-args \
        -p platform:="$robot" -p num_frames:="$FRAMES" \
        -p out_dir:="$outdir" "$@" > "$LOGS/${name}_capture.log" 2>&1
    echo "[$name] finished"
}

echo "frames per lane: $FRAMES     logs: $LOGS"
echo

setsid bash -c "$(declare -f lane); FRAMES=$FRAMES WS=$WS LOGS=$LOGS \
    lane ground ammo_ground 41 husky1 $HOME/mushroom_sim_dataset" &
PIDS+=($!)

setsid bash -c "$(declare -f lane); FRAMES=$FRAMES WS=$WS LOGS=$LOGS \
    lane air ammo_air 42 parrot1 $HOME/mushroom_sim_dataset_air \
    -p hover_z:=$HOVER" &
PIDS+=($!)

echo "watch progress with:"
echo "    tail -f $LOGS/ground_capture.log"
echo "    tail -f $LOGS/air_capture.log"
echo

wait
trap - EXIT
cleanup
echo
echo "done. verify both before training:"
echo "    python3 $REPO/skidsteer_perception/scripts/annotate_dataset.py ~/mushroom_sim_dataset"
echo "    python3 $REPO/skidsteer_perception/scripts/annotate_dataset.py ~/mushroom_sim_dataset_air"
