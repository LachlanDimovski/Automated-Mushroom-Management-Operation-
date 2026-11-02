#!/bin/bash
# Stop every simulation, mission and GUI process (SIGINT, then SIGKILL after 12 s) and report what is left.
# Run it between parts: two sims at once starve the CPU and the robots stall.
PAT='ign gazebo|ruby /usr/bin/ign|bin/ros2 launch|41068_ws/install/|/install/(ammo_|quad_|perception|skidsteer|41068)|opt/ros/humble/lib/|parameter_bridge|joy_node|quad_showcase.py'
P=$(pgrep -f "$PAT" | grep -vx $$)
[ -n "$P" ] && kill -INT $P 2>/dev/null
for i in $(seq 1 12); do
    L=$(for p in $P; do [ -d /proc/$p ] && echo $p; done)
    [ -z "$L" ] && break
    sleep 1
done
[ -n "$L" ] && kill -9 $L 2>/dev/null
sleep 1
pgrep -af "$PAT" | grep -v -e pgrep -e stop_sim || echo "CLEAN: no sim processes"
