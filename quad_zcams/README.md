# quad_zcams

The quads, the fleet and the rover's capture logic used in the demo. "zcam" is the quad with an up and a down depth
camera (the `z` cameras). Names follow the glossary in the top-level README: quad = `parrotN`, rover = `husky1`.

| File | Role |
|---|---|
| `launch/fleet_mission.launch.py` | Up to 4 quads and 5 rovers with the coordinator (`quads:=N rovers:=M`) |
| `launch/fleet_bench.launch.py` | `fleet_mission` with lighter sim options and up to 6 quads; what `demo/part1_fleet.sh` runs |
| `launch/zcam_mission.launch.py` | One zcam quad and one rover; included by `cage_mission.launch.py` (`quad:=zcam`, the default) |
| `zcam_quad_node.py` | Quad search: plans round every obstacle at cruise height, confirms each sighting, hands it to a rover |
| `fleet_quad_node.py` | `zcam_quad_node` for one quad of several: searches its own sector |
| `fleet_coordinator.py` | One mushroom list for the fleet; assigns each mushroom to exactly one rover |
| `fleet_yolo_pool.py` | One YOLO process (OpenVINO) for every camera in the fleet |
| `quad_standdown.py` | Lands the extra quads once every mushroom is known |
| `rover_capture_node.py` | Rover: drive to each mushroom with Nav2, align on its own camera, close the gripper |
| `ps4_teleop.py` | PS4 controller teleop for both robots (hold R1 for the rover, L1 for the quad) |
| `obstacle_map.py`, `shared_map.py`, `husky_projection.py` | The quad's planning grid, the map it shares with the rover, and rover camera projection |
| `models/` | YOLO weights exported to OpenVINO for the pool |
