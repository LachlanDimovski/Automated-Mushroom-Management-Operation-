# ammo_mission_control

The one-quad, one-rover mission and the shared localisation. The operator GUI's Prepare button runs
`launch/cage_mission.launch.py` ("cage" is the older name for the gripper; `ammo_main.launch.py` is the same launch).

| File | Role |
|---|---|
| `launch/cage_mission.launch.py`, `launch/ammo_main.launch.py` | Sim, Nav2, localisation, collision guard, quad and rover. Arguments are listed in the top-level README |
| `truth_localizer.py` | Gazebo ground-truth pose for Nav2 (replaced SLAM Toolbox, which drifted in the forest) |
| `ground_projection.py` | Quad camera detection to a world (x, y) point |
| `quad_search_node.py`, `cage_capture_node.py` | The earlier quad search and rover capture nodes (`quad:=stock`); the demo uses the `quad_zcams` versions |
| `drive_utils.py` | Shared steering maths |
