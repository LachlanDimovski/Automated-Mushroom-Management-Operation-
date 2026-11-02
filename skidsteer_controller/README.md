# skidsteer_controller

Rover safety. `collision_guard_node` (C++) sits between every rover velocity source and the robot: it reads the lidar
and the depth camera and blocks forward motion near an obstacle. Nav2 and the capture node send to
`husky1/cmd_vel_nav_guarded` and `husky1/cmd_vel_raw`; only the guard writes `husky1/cmd_vel`. See "Topic ownership"
in the top-level README.
