# RS1_Team5_
Robotics Studio 1. Master Repository for Team 5. 


AMMO -> Automated Mushroom Management Operations 


<img width="1254" height="1254" alt="Adobe Express - file" src="https://github.com/user-attachments/assets/64d25d77-6fab-4a71-aa8a-e338860c7090" />

A Parrot quadcopter searches a forest for invasive fly agaric mushrooms with
YOLO, projects each sighting to a ground point, and hands it to a Husky
skid-steer, which drives there with Nav2 and collects it with a front gripper.
Everything runs in Gazebo (Ignition) on ROS 2 Humble.

## Packages

| Package | What it holds |
|---|---|
| `41068_ignition_bringup` | Worlds (`simple_trees`, generated `forest_flat` / `forest_random`), models (mushroom, decoys, trees, walking person), robot URDFs, Nav2 config, sim launch files, `scripts/generate_forest_world.py`, `scripts/ps4_drone_test.py` |
| `ammo_mission_control` | The mission: `quad_search_node`, `cage_capture_node`, `truth_localizer`, `ground_projection`, and `cage_mission.launch.py` |
| `skidsteer_controller` | `collision_guard_node` (C++): lidar and depth safety gate for the Husky |
| `perception` | `yolo_node` (one copy per camera), `hazard_brake` (person stop), `combined_view`, trained weights in `models/` |
| `skidsteer_perception` | Dataset capture and training scripts (`scripts/`, see `TRAINING_PC.md`) |
| `ammo_dashboard` | OpenCV HUD: both YOLO feeds, speed, altitude, minimap |
| `quad_mission`, `quad_mission_interfaces` | Quad mission state machine (C++): take-off, search, return, land, abort services. Not part of `cage_mission.launch.py`, see below |

Build from the workspace root:

    cd ~/41068_ws && colcon build --symlink-install && source install/setup.bash

## Running the mission

One launch runs everything - Gazebo, Nav2, the ground-truth localiser, the
collision guard, the rover's capture node and (with `parrot:=true`, the
default) the quad's search flight and detector:

    ros2 launch ammo_mission_control cage_mission.launch.py world:=simple_trees headless:=true \
        mushrooms:=6,-5,-5,-5.5,-2,6.5          # demo mushrooms; the quad has to find them
    ros2 launch ammo_mission_control cage_mission.launch.py world:=forest_flat headless:=true meta_goals:=false
    ros2 launch ammo_mission_control cage_mission.launch.py parrot:=false count:=4   # rover-only test
    ros2 launch ammo_mission_control cage_mission.launch.py hazard:=true              # + person stop on the rover

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `world` | `simple_trees` | World name, or a path to a generated `.sdf` |
| `headless` / `rviz` | `false` / `true` | No Gazebo GUI / RViz |
| `parrot` | `true` | Spawn the quad and run `quad_search_node` |
| `mushrooms` | empty | `x,y,x,y,...` mushrooms spawned for the quad to find |
| `count` | `0` | Random test mushrooms queued straight to the rover |
| `meta_goals` | `true` | Queue the world's own mushrooms (from `<world>.meta.json`) to the rover. `false` = the quad must find them |
| `quad_waypoints` | empty | `x,y,x,y,...` legs for the quad instead of the lawnmower |
| `yolo` / `husky_yolo` | `true` / `false` | Quad detector / rover mushroom detector (off to save CPU) |
| `hazard` | `false` | COCO person detector + `hazard_brake` on the rover |
| `tool` | `gripper` | `cage` selects the older boom cage (also needs `cage:=true` in `husky.urdf.xacro`) |

## Mission flow

1. **Search.** The quad cruises under the canopy at 2 m and flies a lawnmower
   over the world (rows 3 m apart, 4.5 m in from the boundary) or the given
   legs, detouring round known trunks and lidar returns.
2. **Confirm.** On a YOLO hit it moves to a canopy-clear point facing the
   mushroom, drops to 1.2 m and holds still. A waypoint is published only when
   the quad has been still for 2 s, confidence is at least 0.5, two
   consecutive projections agree within 1 m, and the point is not within 2 m of
   the rover. Up to three viewpoints are tried before giving up.
3. **Hand over.** After three consistent waypoints on `husky1/cage/waypoints`
   the quad resumes searching; the rover works in parallel.
4. **Collect.** The rover queues the point, drives with Nav2 to a standoff
   1.7 m from it on the clearest side, creeps in on odometry, lowers and closes
   the gripper, and publishes `husky1/cage/done` (or `husky1/cage/missed` if
   nothing was there, so the quad can find and dispatch it again). A point Nav2 cannot reach is
   tried from up to five directions, then deferred to the back of the queue.

## Topic ownership

Never run two publishers of the same velocity topic.

- `husky1/cmd_vel` is written only by `collision_guard_node`, except for the
  final creep onto the mushroom, which `cage_capture_node` writes directly
  because the guard's 1 m stop would veto it.
- `husky1/cmd_vel_raw` -> guard: `cage_capture_node` (transit), teleop
  (`ipad_teleop.html`, keyboard).
- `husky1/cmd_vel_nav_guarded` -> guard: Nav2 (its `cmd_vel_smoothed`,
  remapped in `cage_mission.launch.py`).
- `parrot1/cmd_vel`: `quad_search_node` (autonomous), `quad_mission`, or
  `ps4_drone_test.py` (manual). Only one at a time.

`hazard_brake` does not publish velocities; it cancels the current Nav2 goal
when a person fills more than 120 px of the rover camera (about 3 m).

## Viewing

Cheap, no Gazebo GUI: `ros2 run ammo_dashboard dashboard` - one OpenCV HUD with
both YOLO feeds, speed/altitude and a minimap (`-p window:=false` to publish it
for the iPad only). iPad: `rosbridge_websocket`, `web_video_server`,
`python3 ipad_teleop_server.py`, then `http://<pc>:8000/` (see `ipad_teleop.html`).

## Other pieces

- **Quad state machine** (`quad_mission`): `ros2 run quad_mission quad_mission_node` exposes
  `/parrot1/mission/*` services (liftoff, start search with a path shape,
  return home, land, abort). It publishes `parrot1/cmd_vel`, so run it with
  `parrot:=false` or not alongside `quad_search_node`.
- **Operator GUI**: Danil's PySide6 GUI and Debug UI live on the `danil`
  branch (`ammo_gui_node/`), not yet merged into `main`.
- **World generator**: `python3 41068_ignition_bringup/scripts/generate_forest_world.py --seed 7 --preset dense`
  writes a seeded forest `.sdf`, its terrain, and a `.meta.json` with the
  ground truth of every tree, mushroom and decoy. See `worlds/README_forest_random.md`.
- **Detector training**: `skidsteer_perception/scripts/` captures labelled
  images in Gazebo and trains YOLO11n (`TRAINING_PC.md`).

## Known limitations

- Localisation is Gazebo ground truth (`truth_localizer`), a documented
  simplification that does not transfer to hardware.
- The quad projects a sighting with its depth image when it has a fresh frame,
  and otherwise assumes flat ground at the spawn clearing's height, so uneven
  terrain (`forest_random`) can add metres of error. Demo on `forest_flat`.
- Detection is reliable to about 3.5 m, hence the 2 m cruise and 3 m rows.
- The rover does not re-check a target with its own camera before gripping,
  so a look-alike that passes the quad's checks gets collected.
- "Collected" is a sim stand-in: any mushroom within 0.6 m of the gripper
  centre is removed from Gazebo.
