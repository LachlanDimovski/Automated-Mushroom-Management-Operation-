#!/bin/bash
# Part 2: the operator GUI (1 quad + 1 rover via cage_mission, PS4 teleop). Pick the world on the setup page,
# press Prepare, then Launch. bash demo/part2_gui.sh
D=$(cd "$(dirname "$0")" && pwd)
bash "$D/stop_sim.sh" > /dev/null
source "$D/env.sh" || exit 1
exec ros2 launch ammo_gui_node gui.launch.py
