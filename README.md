# AMMO: Automated Mushroom Management Operations

41068 Robotics Studio 1, Team 5, Spring 2026.

<img width="300" alt="AMMO logo" src="https://github.com/user-attachments/assets/64d25d77-6fab-4a71-aa8a-e338860c7090" />

A fleet of Parrot quadcopters searches a simulated Australian river forest for invasive fly agaric
(*Amanita muscaria*) with YOLO and projects each sighting to a ground point. A coordinator assigns
each mushroom to a Husky rover, which drives there with Nav2, aligns on its own camera and collects
it with a front gripper. Native mushrooms are detected, logged and left in place. Everything runs in
Ignition Gazebo Fortress on ROS 2 Humble, with an operator GUI, a debug window and PS4 teleop.

## Team

| Member | GitHub |
|---|---|
| Lachlan Dimovski | `LachlanDimovski` |
| Darcy Moyes | `Darb05` |
| Danil Vasiliev | `danilvasiliev` |
| `hpdsalter` | `hpdsalter` |

## Setup

Tested on Ubuntu 22.04 with ROS 2 Humble and Ignition Gazebo Fortress.

```bash
sudo apt install -y git-lfs libxcb-cursor0 ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-ros-gz ros-humble-robot-localization ros-humble-xacro ros-humble-joy \
    ros-humble-vision-msgs ros-humble-cv-bridge
pip3 install ultralytics openvino PySide6 scipy "numpy<2"

mkdir -p ~/41068_ws/src && cd ~/41068_ws/src
git lfs install
git clone https://github.com/LachlanDimovski/AMMO-Mushroom-Fleet.git
cd ~/41068_ws && colcon build --symlink-install && source install/setup.bash
bash src/AMMO-Mushroom-Fleet/demo/get_world.sh      # demo river worlds, about 130 MB (needs `gh auth login`)
```

- **Git LFS is required.** Meshes and textures are stored in LFS. Without it the clone holds
  130-byte pointer files and the mushrooms spawn with no geometry. Run `git lfs pull` and rebuild if
  `41068_ignition_bringup/models/mushroom_amanita/meshes/amanita_muscaria.glb` is not about 9 MB.
- **The river worlds are not in git.** `demo/get_world.sh` downloads them from the v1.0 release
  (or pass it a copy of `ammo_river_worlds.tar.gz` downloaded from the Releases page). They can
  also be built from source with `forest_worlds/setup.sh` (internet and Blender 4.2 or later).
- Tested Python versions: ultralytics 8.4.66, openvino 2024.6.0, PySide6 6.10.2, numpy 1.26.4.
- Build with or without `--symlink-install`, but do not switch between the two in one workspace.
- Headless GUI and bridge checks (no ROS or Gazebo needed):
  `python3 src/AMMO-Mushroom-Fleet/ammo_gui_node/tests/run_tests.py`.

## Running the demo

Two parts. Run `bash demo/stop_sim.sh` between them and wait for `CLEAN`: two simulations at once
starve the CPU and the robots stall. Set `powerprofilesctl set performance` first, and start with the
CPU below 70 °C.

**Part 1: fleet run (2 quads, 1 rover, 12 invasive and 6 native mushrooms).**

```bash
bash demo/part1_fleet.sh      # headless Gazebo + the AMMO Fleet Debug window
bash demo/part1_timer.sh      # optional, second terminal: wall time to 12/12
```

All 12 are collected in about 9 to 12 minutes from sim start. The quads search in square loops and
hold over each find, the coordinator sends each mushroom to the rover, natives appear as blue N markers
and are never collected, and the rover crosses the river only at the fords. Once every mushroom is
known the second quad stands down and lands. `HEADLESS=false` adds the Gazebo window (much slower).

**Part 2: operator GUI and PS4 controller (1 quad, 1 rover).**

```bash
bash demo/part2_gui.sh
```

Pick `river_moderate_aus` on the setup page and press **Prepare**; the robots stay frozen while the
checklist fills in. Press **Launch** when it shows READY. Pause and Resume, then hold **R1** to drive the
rover or **L1** to fly the quad (see [PS4 teleop](#ps4-teleop)); release and the robot resumes its
mission. End the mission from the GUI, then run `bash demo/stop_sim.sh`.

Logs from both parts are in `~/.ros/ammo_demo/`. `quad_showcase/` holds an extra display of the quad's
state machine and depth mapping that runs beside a Part 2 mission (see its README).

## Packages

| Package | What it holds |
|---|---|
| `demo` | The demo scripts above, and `pack_world.sh` / `get_world.sh` for the river world bundle |
| `41068_ignition_bringup` | Worlds (`simple_trees`, generated `forest_flat` / `forest_random`), models (mushrooms, decoys, trees, walking person), robot URDFs, Nav2 config, sim launch files, `scripts/generate_forest_world.py`, `scripts/ps4_drone_test.py` |
| `forest_worlds` | River world generators (`setup.sh`, `build_world.py`, `make_demo_world.py`, `shuffle_mushrooms.py`); the generated worlds and models are not in git |
| `ammo_mission_control` | Single-quad mission: `quad_search_node`, `cage_capture_node`, `truth_localizer`, `ground_projection`, `cage_mission.launch.py` (also `ammo_main.launch.py`) |
| `quad_zcams` | The z-camera quad (`zcam_quad_node`), the fleet (`fleet_mission.launch.py`, `fleet_coordinator`, `fleet_yolo_pool`, `quad_standdown`), `rover_capture_node` and PS4 teleop |
| `perception` | `yolo_node` (one copy per camera), `hazard_brake` (person stop), `combined_view`, trained weights in `models/` |
| `ammo_gui_node` | PySide6 operator GUI, Debug UI and Fleet Debug UI, `mission_bridge`, headless tests |
| `skidsteer_controller` | `collision_guard_node` (C++): lidar and depth safety gate for the Husky |
| `skidsteer_perception` | Dataset capture and YOLO training scripts (`scripts/`, see `TRAINING_PC.md`) |
| `ammo_dashboard` | OpenCV HUD: both YOLO feeds, speed, altitude, minimap |
| `quad_mission`, `quad_mission_interfaces` | Quad mission state machine (C++): take-off, search, return, land, abort services. Not part of the demo launches |
| `quad_showcase` | Quad state machine and mapping display (run by path, not a ROS package) |
| `tools/ipad_teleop` | iPad teleop and camera viewer page |

## Running the mission by hand

The demo scripts wrap these launches. Source the workspace first; for a `forest_worlds` world, also
put its models on Gazebo's path (`source demo/env.sh` does both), or Gazebo cannot find them
(`Unable to find uri[model://gum_05]`), the robots never spawn and the sim looks frozen.

    ros2 launch ammo_mission_control cage_mission.launch.py world:=simple_trees headless:=true \
        mushrooms:=6,-5,-5,-5.5,-2,6.5          # demo mushrooms; the quad has to find them
    ros2 launch ammo_mission_control cage_mission.launch.py parrot:=false count:=4   # rover-only test
    ros2 launch ammo_mission_control cage_mission.launch.py hazard:=true              # + person stop on the rover

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `world` | `simple_trees` | World name, or a path to a generated `.sdf` |
| `headless` / `rviz` | `false` / `true` | No Gazebo GUI / RViz |
| `parrot` | `true` | Spawn the quad |
| `quad` | `zcam` | `zcam`: the z-camera quad (`quad_zcams`), plans round every obstacle at 1 m and never climbs over canopies. `stock`: `quad_search_node` |
| `imgsz`, `center_out`, `det_range`, `quad_routes`, ... | see `quad_zcams/launch/zcam_mission.launch.py` | Quad search and rover routing options for `quad:=zcam`. The defaults are the configuration that collected 24/24 in a 40 m leaf-mesh forest (three runs, 23-26 min each) |
| `mushrooms` | empty | `x,y,x,y,...` mushrooms spawned for the quad to find |
| `count` | `0` | Random test mushrooms queued straight to the rover |
| `meta_goals` | `true` | Queue the world's own mushrooms (from `<world>.meta.json`) to the rover. `false` = the quad must find them |
| `quad_waypoints` | empty | `x,y,x,y,...` legs for the quad instead of the lawnmower |
| `yolo` / `husky_yolo` | `true` / `true` | Quad detector / rover mushroom detector (the rover's camera steers the final alignment, `false` to save CPU) |
| `hazard` | `false` | COCO person detector + `hazard_brake` on the rover |
| `tool` | `gripper` | `cage` selects the older boom cage (also needs `cage:=true` in `husky.urdf.xacro`) |

## Running the fleet by hand

`demo/part1_fleet.sh` is the tested configuration. By hand, with `source demo/env.sh` done:

    ros2 launch quad_zcams fleet_mission.launch.py world:=$F/worlds/river_moderate_aus \
        meta_goals:=false quads:=2 rovers:=1 headless:=true rviz:=false
    ros2 launch ammo_gui_node fleet_debug.launch.py     # second terminal

- `fleet_debug.launch.py` also starts `mission_bridge`, which publishes the
  `ammo/mission/status` and `ammo/mission/events` topics the timeline and state cards read.
  Running `ros2 run ammo_gui_node fleet_debug` on its own shows the cameras and map, but the
  timeline stays empty. Do not run it alongside the operator GUI, which starts its own bridge.
- Drop `headless:=true` to also open the Gazebo window. It costs about 60% of a core; the
  Fleet Debug UI already shows every camera and the map.
- The fleet loads the two-class detector (`amanita_river_v6_yolo11n`, invasive + native) by
  default. `model_path:=...` overrides it; a one-class model sees no natives.
- Stop every process of a run before launching the next. Two sims at once starve the CPU and
  the robots stall.

## Native (non-invasive) mushrooms

The two-class detector labels `amanita_muscaria_red` (invasive, collected) and
`native_mushroom` (monitored, never collected). Native caps are bright blue in the sim
(`mushroom_native/meshes/native_cap.png`): the original brown caps matched the leaf litter, and the
detector could not tell them apart.

- **Camera feeds:** invasive boxes are red, native boxes are teal.
- **Quads:** a native is logged when it is within 3 m of the quad (`det_range`, as for
  invasive ones), scores at least `native_min_conf` (default 0.5) and its box is at least
  `native_min_px` wide (default 12; far-off specks were the false ones), and is seen a second
  time within 1.5 m at least 0.4 s later (one-frame false natives are dropped). The log line is
  `native mushroom N logged at (x, y) - left in place`. No rover is sent, and an invasive
  sighting within 0.75 m of a native, logged or seen once, is ignored.
- **Detector:** one box per object (class-agnostic NMS), so a native cannot also come out as
  an invasive box underneath.
- **`mission_bridge`** reads every quad's native log lines (`quad_search_controller` and the
  fleet's `fleet_quad_*`) and its `parrot1/quad_state`, merges sightings within 2 m
  (`NATIVE_MERGE_M`), so one mushroom seen twice or by two quads counts once, and announces
  each on `ammo/mission/events`. The status carries the list as `natives`.
- **Debug UI and Fleet Debug UI:** natives show as teal markers on the minimap and timeline,
  in the event feed, and (fleet) in a NATIVE MUSHROOMS count card.
- **Single quad** (`cage_mission.launch.py`): pass the two-class model with
  `model_path:=$(ros2 pkg prefix perception)/share/perception/models/variants/amanita_river_v6_yolo11n.pt`
  (`forest_worlds/demo_river.sh` already does). The default one-class model sees no natives.

## Mission flow

1. **Search.** The quad cruises under the canopy at 1 m and flies a lawnmower
   over the world (square loops 3 m apart out to 2 m inside the playable area) or the given
   legs, on an A* path round everything in the world file and what the lidar sees.
   It never climbs over canopies: a sweep point it cannot reach is skipped.
2. **Confirm.** On a YOLO hit it moves to a canopy-clear point facing the
   mushroom, drops to 1.2 m and holds still. A waypoint is published only when
   the quad has been still for 2 s, confidence is at least 0.5, two
   consecutive projections agree within 1 m, and the point is not within 2 m of
   the rover. Up to three viewpoints are tried before giving up.
3. **Hand over.** After three consistent waypoints on `husky1/cage/waypoints`
   the quad resumes searching; the rover works in parallel.
4. **Collect.** The rover queues the point, drives with Nav2 to a standoff
   1.7 m from it on the clearest side, turns to face it (ALIGN), creeps in, lowers and closes
   the gripper, and publishes `husky1/cage/done` (or `husky1/cage/missed` if
   nothing was there, so the quad can find and dispatch it again). A point Nav2 cannot reach is
   tried from up to five directions, then deferred to the back of the queue.
   During ALIGN the rover steers on its own camera's sighting (projected with the pose at that
   frame) when it is under 2.5 s old and within 15 deg of the odometry heading; otherwise on
   odometry. Native boxes are never a target.

## Topic ownership

Never run two publishers of the same velocity topic.

- `husky1/cmd_vel` is written only by `collision_guard_node`, except for the
  final creep onto the mushroom, which `cage_capture_node` writes directly
  because the guard's 1 m stop would veto it.
- `husky1/cmd_vel_raw` -> guard: `cage_capture_node` (transit), teleop
  (`ps4_teleop`, `ipad_teleop.html`, keyboard).
- `husky1/cmd_vel_nav_guarded` -> guard: Nav2 (its `cmd_vel_smoothed`,
  remapped in `cage_mission.launch.py`).
- `parrot1/cmd_vel`: `quad_search_node` / `zcam_quad_node` (autonomous), `quad_mission`,
  `ps4_teleop` or `ps4_drone_test.py` (manual). Only one at a time.
- `husky1/teleop_active`, `parrot1/teleop_active` (`ps4_teleop`): heartbeats while a
  deadman is held; `rover_capture_node` and `zcam_quad_node` stand down until they stop.

## PS4 teleop

`zcam_mission.launch.py` starts `joy_node` and `ps4_teleop` (`ps4:=false` to leave them out).
With the mission running, hold **R1** to drive the rover (left stick forward/back, right stick
turn; still through `collision_guard`) or **L1** to fly the quad (left stick move, right stick
yaw/climb). The robot being driven pauses its own mission; let go and it re-plans from where
it was left. Nothing is published while no button is held. Wrong stick or button: with no
mission running, `ros2 run quad_zcams ps4_teleop --ros-args -p inspect:=true` prints the axis and
button numbers; pass them as parameters (`rover_button`, `quad_button`, `axis_forward`, `axis_turn`, ...).

| Control | Action |
|---|---|
| **R1** held | Drive the rover: left stick forward/back, right stick turn |
| **L1** held | Fly the quad: left stick forward/back and strafe, right stick yaw and climb |
| **OPTIONS** | Teleop mode on/off: the rover (first) stays the operator's without a button held |
| **TRIANGLE** | In teleop mode: switch between the rover and the quad |
| **CROSS** | Rover in hand: grab in front of the claw. Quad in hand: send the mushroom in view to the rover |
| **L2** held | Rover creep at 0.2 m/s past `collision_guard`, to close in on a mushroom |

The stick signs follow `ps4_drone_test.py`. Check them on the real controller before a demo:
`inspect:=true`, push each stick up and right, and confirm the numbers match `axis_forward`
(up = negative) and `axis_turn` (right = positive).

`hazard_brake` does not publish velocities; it cancels the current Nav2 goal
when a person fills more than 120 px of the rover camera (about 3 m).

## Viewing

Cheap, no Gazebo GUI: `ros2 run ammo_dashboard dashboard` - one OpenCV HUD with
both YOLO feeds, speed/altitude and a minimap (`-p window:=false` to publish it
for the iPad only). iPad: `rosbridge_websocket`, `web_video_server`,
`python3 tools/ipad_teleop/ipad_teleop_server.py`, then `http://<pc>:8000/` (see `tools/ipad_teleop/ipad_teleop.html`).

## Other pieces

- **Quad state machine** (`quad_mission`): `ros2 run quad_mission quad_mission_node` exposes
  `/parrot1/mission/*` services (liftoff, start search with a path shape,
  return home, land, abort). It publishes `parrot1/cmd_vel`, so run it with
  `parrot:=false` or not alongside `quad_search_node`.
- **Operator GUI**: `ros2 launch ammo_gui_node gui.launch.py` (PySide6 GUI with `mission_bridge`,
  `ammo_gui_node/`). A run starts in two stages: **Prepare** starts `cage_mission.launch.py`
  (one quad, one rover, PS4 teleop) and freezes both mission nodes while a checklist fills in and
  the mushrooms are spawned; **Launch** releases them and starts the clock. End Mission keeps
  Gazebo up for the next run. See `ammo_gui_node/README.md`. Pause freezes the mission
  nodes; they get `ammo_gui_node/config/fastdds_udp_only.xml` (1 h discovery lease, loopback
  UDP) so the other processes keep them through a long pause. Without it, a pause over 20 s
  could leave the rover without `/clock` or Nav2 after Resume.
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
  terrain (`forest_random`) can add metres of error.
- Detection is reliable to about 3.5 m, hence the 2 m cruise and 3 m rows.
- Native detection is trained on the sim's blue caps only (`river_v6`: blue caps at 1-5 m,
  river banks included). With the second-sighting rule, fleet runs on `river_moderate_aus`
  logged 6/6 real natives with 0-1 false ones, sent no native to the rover, and collected 12/12.
  The false one that recurs is at the water's edge near (17.7, -12.2), likely the blue water
  read as a cap; it is logged only, never collected.
  The training data and scripts are in `~/mushroom_river` (`train_v6.sh`) on the capture PC,
  captured with `skidsteer_perception/scripts/run_capture.sh --hover 1.0 [--spots banks.json]`.
- The rover's camera steers the final alignment when it has a fresh sighting, and an empty grab sends
  the rover back once to where its camera saw the mushroom, but the camera cannot veto a target: a
  look-alike that passes the quad's checks gets collected.
- "Collected" is a sim stand-in: any mushroom within 0.6 m of the gripper
  centre is removed from Gazebo.
