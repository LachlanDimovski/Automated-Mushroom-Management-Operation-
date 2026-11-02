# demo

The scripts for the two-part demo. Run them from the repo folder; the top-level README explains each part.

| Script | Does |
|---|---|
| `part1_fleet.sh` | Part 1: 2 quads + 1 rover on the river world, with the AMMO Fleet Debug window |
| `part1_timer.sh` | Beside Part 1: wall time from the sim clock starting to 12/12 |
| `part2_gui.sh` | Part 2: the operator GUI (Prepare, Launch, Pause, PS4 takeover, End) |
| `stop_sim.sh` | Stops every sim, mission and GUI process. Run it between parts |
| `env.sh` | Sourced by the others: ROS, the built workspace and the river model path |
| `get_world.sh` | Downloads the river worlds from the v1.0 release into `forest_worlds/` |
| `pack_world.sh` | Maintainer: builds that release file |

Logs go to `~/.ros/ammo_demo/`.
