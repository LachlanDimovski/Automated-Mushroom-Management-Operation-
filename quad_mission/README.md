# quad_mission and quad_mission_interfaces

The quad mission state machine (C++), from earlier in the project. `ros2 run quad_mission quad_mission_node` offers
`/parrot1/mission/*` services (take off, start a search with a path shape, return home, land, abort);
`quad_mission_interfaces` defines them. It publishes `parrot1/cmd_vel`, so never run it alongside the demo's quad nodes. To try it, run
`ros2 launch 41068_ignition_bringup 41068_ignition_parrot.launch.py` in one terminal, the node in a second, and
`ros2 service call /parrot1/mission/liftoff std_srvs/srv/Trigger` in a third, after `source demo/env.sh` in each.
Not part of the demo launches.
