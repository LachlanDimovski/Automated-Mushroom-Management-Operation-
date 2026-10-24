#!/bin/bash
# Full mission on the river world with the Gazebo window and the AMMO dashboard: ./demo_river.sh
F=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/humble/setup.bash; source ~/41068_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH="$(ls -d "$F"/models/* "$F"/world_assets | tr '\n' ':')${IGN_GAZEBO_RESOURCE_PATH:-}"
(sleep 40; exec ros2 launch ammo_dashboard dashboard.launch.py map_range_m:=16.0) &
exec ros2 launch ammo_mission_control cage_mission.launch.py world:="$F/worlds/river_moderate_aus" \
    meta_goals:=false rviz:=false headless:=false \
    model_path:="$(ros2 pkg prefix perception)/share/perception/models/variants/amanita_river_v6_yolo11n.pt"
