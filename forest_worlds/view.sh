#!/bin/bash
# Open a river preset in Gazebo, closing any Gazebo that is already running: view.sh [sparse|moderate|dense|hilly]
D=$(cd "$(dirname "$0")" && pwd)
# ign gazebo's server/gui run as `ruby` processes; match only those so this shell is never caught
for p in $(pgrep -x ruby); do grep -qa "ign" /proc/$p/cmdline 2>/dev/null && kill -9 "$p"; done
sleep 1
export IGN_GAZEBO_RESOURCE_PATH=$(ls -d "$D"/models/* "$D"/world_assets | tr '\n' ':')
setsid nohup ign gazebo -r "$D/worlds/river_${1:-moderate}.sdf" > /dev/null 2>&1 &
