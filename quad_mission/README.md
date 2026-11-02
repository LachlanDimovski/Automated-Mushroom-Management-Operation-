# quad_mission and quad_mission_interfaces

The quad mission state machine (C++), from earlier in the project. `ros2 run quad_mission quad_mission_node` offers
`/parrot1/mission/*` services (take off, start a search with a path shape, return home, land, abort);
`quad_mission_interfaces` defines them. It publishes `parrot1/cmd_vel`, so run it with `parrot:=false` and not
alongside the demo's quad nodes. Not part of the demo launches.
