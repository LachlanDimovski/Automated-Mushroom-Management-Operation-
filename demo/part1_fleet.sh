#!/bin/bash
# Part 1: 2 quads + 1 rover on the demo river world, headless Gazebo, with the AMMO Fleet Debug window.
#   bash demo/part1_fleet.sh [launch args]   env: WORLD, QUADS, ROVERS, HEADLESS=false (adds the Gazebo window)
# 12/12 in about 9-12 min from sim start with the CPU below 70 C. Logs in ~/.ros/ammo_demo/.
D=$(cd "$(dirname "$0")" && pwd)
bash "$D/stop_sim.sh" > /dev/null
source "$D/env.sh" || exit 1
W=$F/worlds/${WORLD:-river_moderate_aus_dt2}
[ -f "$W.sdf" ] || { echo "Missing $W.sdf: run bash demo/get_world.sh first." >&2; exit 1; }
M=$(ros2 pkg prefix perception)/share/perception/models/variants/amanita_river_v6_yolo11n.pt
ros2 launch quad_zcams fleet_bench.launch.py world:=$W quads:=${QUADS:-2} rovers:=${ROVERS:-1} cruise_speed:=2.0 \
    lite:=true meta_goals:=false headless:=${HEADLESS:-true} rviz:=false model_path:=$M "$@" > "$LOGS/part1_sim.log" 2>&1 &
SIM=$!
# the Fleet Debug window's timeline, clock and state cards start on the bridge's "start" command (the operator GUI
# sends it on Launch); send it once the sim clock is running
WN=$(basename "$W")
( until timeout 20 ign topic -e -n 1 -t /world/$WN/stats 2>/dev/null | grep -q "sec:"; do sleep 2; done
  ros2 topic pub --once -w 1 ammo/mission/command std_msgs/msg/String \
      "{data: '{\"cmd\": \"start\", \"world\": \"$WN\", \"total\": ${TOTAL:-12}, \"quads\": ${QUADS:-2}, \"rovers\": ${ROVERS:-1}}'}" \
      > "$LOGS/part1_start.log" 2>&1 ) &
ros2 run ammo_gui_node mission_bridge --ros-args -p use_sim_time:=true > "$LOGS/part1_bridge.log" 2>&1 &
ros2 run ammo_gui_node fleet_debug --ros-args -p use_sim_time:=true > "$LOGS/part1_debug.log" 2>&1 &
echo "Part 1 running (sim log: $LOGS/part1_sim.log). Stop with: bash demo/stop_sim.sh"
wait $SIM
