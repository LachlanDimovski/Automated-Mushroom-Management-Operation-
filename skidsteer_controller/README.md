# skidsteer_controller

Rover safety. `collision_guard_node` (C++) sits between normal rover driving and the robot: it reads the lidar and the
depth camera and blocks motion near an obstacle. The capture node and teleop send to `husky1/cmd_vel_raw`, which gets
the full check. Nav2 sends to `husky1/cmd_vel_nav_guarded`, which gets the depth stop, rear stop and flank check. The
guard writes `husky1/cmd_vel`, apart from four slow moves that bypass it. See "Topic ownership" in the top-level README.
