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
| Henry Salter | `hpdsalter` |

## Names used in this project

The code grew over a semester, so each robot has a few names. They all mean the same thing:

| In this README | Also appears as | What it is |
|---|---|---|
| **quad** | `parrot1`, `parrot2`, ... (ROS namespaces), quadcopter, Parrot, `zcam` (the quad with up and down depth cameras) | The searching drone |
| **rover** | `husky1` (rover 1), `r2/husky1` (rover 2), Husky, skidsteer (in the GUI) | The ground robot that collects |
| **gripper** | `cage` in older file, node and topic names (`cage_mission`, `cage_capture`, `husky1/cage/waypoints`) | The rover's front claw. An earlier design used a cage; the names stayed |
| **invasive mushroom** | `amanita_muscaria_red`, fly agaric, red mushroom | Collected |
| **native mushroom** | `native_mushroom`, blue cap | Logged and left in place |
| **coordinator** | `fleet_coordinator` | Assigns each found mushroom to one rover in a fleet run |
| **bridge** | `mission_bridge` | Turns logs and topics into `ammo/mission/status` and `ammo/mission/events` for the windows |

Main topics (rover 1 and quad 1 shown; quad N uses `parrotN/`, rover N uses `rN/husky1/`):

| Topic | Meaning |
|---|---|
| `parrot1/camera/image`, `husky1/camera/image` | Camera feeds (`.../detections/image` with YOLO boxes) |
| `husky1/cage/waypoints` | A mushroom position sent to the rover |
| `husky1/cage/done`, `husky1/cage/missed` | The rover collected it, or found nothing there |
| `fleet/reports`, `fleet/status` | Quad sightings to the coordinator, and its summary |
| `parrot1/ground_obstacles` | The quad's occupancy grid, used by the rover to pick a clear approach |
| `ammo/mission/status`, `ammo/mission/events`, `ammo/mission/command`, `ammo/mission/pause` | GUI and bridge topics |

Which launch to use:

| To run | Use |
|---|---|
| The fleet demo (Part 1) | `bash demo/part1_fleet.sh` (runs `quad_zcams fleet_bench.launch.py`, the fleet launch with lighter sim options) |
| The operator GUI (Part 2) | `bash demo/part2_gui.sh` (its Prepare button runs `ammo_mission_control cage_mission.launch.py`) |
| One quad and one rover without the GUI | `ros2 launch ammo_mission_control ammo_main.launch.py` (same as `cage_mission.launch.py`) |
| A chosen fleet size | `ros2 launch quad_zcams fleet_mission.launch.py quads:=N rovers:=M` (up to 4 quads and 5 rovers; `fleet_bench.launch.py` takes up to 6 quads) |
| The course starter simulation | `ros2 launch 41068_ignition_bringup 41068_ignition_husky.launch.py` (see that package's README). Its autonomy and dynamic world demos start no simulation, so run them in a second terminal with one already running |

## Setup

Tested line by line on a blank Ubuntu 22.04 system with ROS 2 Humble and Ignition Gazebo Fortress 6.x.
It needs about 8 CPU cores and no GPU. The package downloads take about 12 minutes.

```bash
# Terminal 1
sudo apt update && sudo apt install -y curl software-properties-common
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-humble-desktop ignition-fortress python3-colcon-common-extensions \
    python3-pip gh git-lfs libxcb-cursor0 ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-ros-gz ros-humble-ros-ign-bridge ros-humble-ros-ign-gazebo \
    ros-humble-ros-ign-interfaces ros-humble-robot-localization ros-humble-xacro \
    ros-humble-joy ros-humble-vision-msgs ros-humble-cv-bridge
pip3 install torch==2.12.0 torchvision==0.27.0 ultralytics==8.4.66 openvino==2024.6.0 \
    PySide6==6.10.2 scipy "numpy<2"

gh auth login                  # the repository is private
gh auth setup-git              # lets git use that login
source /opt/ros/humble/setup.bash
mkdir -p ~/41068_ws/src && cd ~/41068_ws/src
git lfs install
git clone https://github.com/LachlanDimovski/AMMO-Mushroom-Fleet.git
cd ~/41068_ws && colcon build --symlink-install && source install/setup.bash
cd src/AMMO-Mushroom-Fleet
bash demo/get_world.sh         # river worlds, about 130 MB
python3 ammo_gui_node/tests/check_env.py
python3 ammo_gui_node/tests/run_tests.py
```

- **Git LFS is required.** Meshes and textures are stored in LFS. Without it the clone holds
  130-byte pointer files and the mushrooms spawn with no geometry. Run `git lfs pull`, then
  `cd ~/41068_ws && colcon build --symlink-install`, if
  `41068_ignition_bringup/models/mushroom_amanita/meshes/amanita_muscaria.glb` is not about 9 MB.
- **The river worlds are not in git.** `demo/get_world.sh` downloads them from the v1.0 release.
  Without the GitHub CLI, clone with a personal access token as the password, download
  `ammo_river_worlds.tar.gz` from the Releases page and run `bash demo/get_world.sh FILE.tar.gz`.
  The worlds can also be built from source with `bash forest_worlds/setup.sh`, which needs internet,
  Blender 4.2 or later, about 2 GB and about 10 minutes, and replaces the files in `forest_worlds/`.
- Tested Python versions: torch 2.12.0, torchvision 0.27.0, ultralytics 8.4.66, openvino 2024.6.0, PySide6 6.10.2, numpy 1.26.4.
- `python3 ammo_gui_node/tests/check_env.py` checks the environment and prints the fix for anything
  missing. In a new terminal, run `source demo/env.sh` first or it reports ROS as missing. Its `QT_QPA_PLATFORM` and `LIBGL_ALWAYS_SOFTWARE` warnings can be ignored: both demo
  parts ran without them.
- Build with or without `--symlink-install`, but do not switch between the two in one workspace.
- Headless GUI and bridge checks, with no ROS or Gazebo (663 pass):
  `python3 ammo_gui_node/tests/run_tests.py`.

## Running the demo

### Terminals

Start every new terminal in the repository folder:

```bash
cd ~/41068_ws/src/AMMO-Mushroom-Fleet
```

The `demo/` scripts set up their own environment. Any terminal that runs `ros2`, `ign` or
`check_env.py` directly needs `source demo/env.sh` first, and that is also what defines `$F`.

There are two parts, called Scenario 1 and Scenario 2 in the report. Run `bash demo/stop_sim.sh` between them and wait for `CLEAN`: two simulations at
once starve the CPU and the robots stall. Before a timed run, let the CPU cool below 70 °C.

**Part 1: fleet run (2 quads, 1 rover, 12 invasive and 6 native mushrooms).**

```bash
# Terminal 1
powerprofilesctl set performance
bash demo/part1_fleet.sh      # headless Gazebo and the AMMO Fleet Debug window

# Terminal 2, optional
bash demo/part1_timer.sh      # wall time to 12 of 12
```

One window opens, AMMO Fleet Debug, and the sim clock starts after about 20 s. The quads search and
hold over each find, and red marks appear on the minimap as they report. The coordinator sends each
mushroom to the rover, and green crosses replace the red marks as it collects. Natives appear as teal
circled marks and are never collected. The rover crosses the river only at the fords. Once every
mushroom is known, every quad except `parrot1` flies home and lands. `parrot1` lands when the mission
completes.

All 12 are collected in about 9 to 14 minutes. The simulation ends itself 1 to 2.5 minutes later. The
window stops updating and Terminal 1 returns to its prompt. Run `bash demo/stop_sim.sh` there and wait
for `CLEAN`.

Set environment variables in front of the script and add launch arguments after it:

```bash
# Terminal 1, one at a time
QUADS=3 bash demo/part1_fleet.sh row_spacing:=4.0
WORLD=river_moderate_aus_m1 bash demo/part1_fleet.sh
```

To stop a run early, use `bash demo/stop_sim.sh` in Terminal 2.

| Setting | Default | Effect |
|---|---|---|
| `QUADS`, `ROVERS` | 2, 1 | Fleet size, up to 6 quads |
| `WORLD` | `river_moderate_aus_dt2` | World file. `_m1` to `_m3` are shuffled layouts, also with 12 invasive and 6 native mushrooms |
| `HEADLESS` | `true` | `false` opens the Gazebo window, which slows the run |

**Part 2: operator GUI and PS4 controller (1 quad, 1 rover).**

```bash
# Terminal 1
bash demo/part2_gui.sh
```

1. On the setup page, pick `river_moderate_aus` and "World's own 12 mushrooms".
2. Untick **Show the Gazebo window**, then press **PREPARE TO LAUNCH**. The robots stay held while the
   checklist fills in, and READY takes about 20 s.
3. Optional 3D view, in Terminal 2: `source demo/env.sh && ign gazebo -g`. Do not press its
   play or pause button.
4. Press **LAUNCH MISSION**. The mission page opens and the clock starts.
5. Press **PAUSE**, wait 20 s, then **RESUME**. Pause early: the first find comes 5 to 9 minutes after
   Launch in this world.
6. With a PS4 controller plugged in, hold **R1** to drive the rover or **L1** to fly the quad (see
   [PS4 teleop](#ps4-teleop)). Release to hand it back.
7. Press **END MISSION**. The simulation closes and the setup page returns. Then run
   `bash demo/stop_sim.sh` in Terminal 3, because Terminal 1 is still running the GUI. That also closes
   the GUI.

Part 1 logs are in `~/.ros/ammo_demo/` and the operator GUI's log is `~/.ros/ammo_gui_mission.log`.
`quad_showcase/` holds an extra display of the quad's state machine and depth mapping that runs beside
a Part 2 mission (see its README).

The system rehearsed for the final demonstration is commit `366d30f`. Later commits on `main` change
documentation and two `forest_worlds` helper scripts the demo does not use.

### Common problems

Run these fixes in a free terminal.

| Symptom | Cause and fix |
|---|---|
| Sim appears frozen; robots never spawn | Gazebo cannot find the forest models. Use the `demo/` scripts, or run `source demo/env.sh` first |
| Mushrooms invisible; every count stays 0 | Git LFS pointer files. Run `git lfs pull`, then `cd ~/41068_ws && colcon build --symlink-install` |
| Robots stall or drift to a crawl | Two simulations are running, or the CPU is hot. Run `bash demo/stop_sim.sh` and wait for `CLEAN` |
| After 60 s, telemetry and camera rows stay empty | The Gazebo window hang. Press **BACK TO SETUP**, untick the box, then Prepare again |
| Every row ticked except Nav2 | Wait: the rover starts Nav2 itself after 60 s |
| R1 forward does nothing | The guard is stopping the rover in front of an obstacle. Turn on the spot first |
| End Mission and Abort off screen | The window needs 1548 px of width. Use 1920 x 1080 or more |
| No progress for about 10 minutes | A stalled run. Run `bash demo/stop_sim.sh` and restart |

## Packages

Grouped by role. "Demo" marks what the two-part demo runs.

| Role | Package | What it holds | Demo |
|---|---|---|---|
| Demo scripts | `demo` | `part1_fleet.sh`, `part2_gui.sh`, `stop_sim.sh`, `env.sh`, and `get_world.sh` / `pack_world.sh` for the river world bundle | yes |
| Simulation | `41068_ignition_bringup` | The course starter package, extended: robot URDFs, Nav2 config, models (mushrooms, trees, walking person), `simple_trees` and generated `forest_flat` / `forest_random` worlds | yes |
| Simulation | `forest_worlds` | River world generators (`setup.sh`, `build_world.py`, `make_demo_world.py`, `shuffle_mushrooms.py`); the generated worlds come from the release | yes |
| Fleet and robots | `quad_zcams` | The zcam quad (`zcam_quad_node`, `fleet_quad_node`), the fleet (`fleet_mission.launch.py`, `fleet_coordinator`, `fleet_yolo_pool`, `quad_standdown`), the rover's capture node (`rover_capture_node`) and PS4 teleop (`ps4_teleop`) | yes |
| Fleet and robots | `ammo_mission_control` | The one-quad, one-rover launch (`cage_mission.launch.py`, alias `ammo_main.launch.py`), `truth_localizer`, `ground_projection`, and the earlier `quad_search_node` / `cage_capture_node` (`quad:=stock`) | yes |
| Perception | `perception` | `yolo_node` (one per camera in the operator mission), `hazard_brake` (stops the rover for a person), `combined_view`, trained YOLO weights in `models/` | yes |
| Perception | `skidsteer_perception` | Dataset capture in Gazebo and YOLO training scripts (`scripts/`, see `TRAINING_PC.md`) | no |
| Rover safety | `skidsteer_controller` | `collision_guard_node` (C++): lidar and depth safety gate on normal rover driving (see [Topic ownership](#topic-ownership)) | yes |
| Operator interface | `ammo_gui_node` | PySide6 operator GUI, Debug UI and Fleet Debug UI, `mission_bridge`, headless tests | yes |
| Operator interface | `quad_showcase` | Display of the quad's state machine and depth mapping (run by path, not a ROS package) | optional |
| Operator interface | `ammo_dashboard` | OpenCV HUD: both YOLO feeds, speed, altitude, minimap | no |
| Operator interface | `tools/ipad_teleop` | iPad teleop and camera viewer page | no |
| Earlier work | `quad_mission`, `quad_mission_interfaces` | Quad mission state machine (C++): take-off, search, return, land and abort services | no |

## Running the mission by hand

The demo scripts wrap these launches. Run `source demo/env.sh` in the terminal first. It sets up the
workspace and puts the `forest_worlds` models on Gazebo's path. Without it Gazebo cannot find them
(`Unable to find uri[model://gum_05]`), the robots never spawn and the sim looks frozen.

```bash
# Terminal 1, one at a time; stop each with `bash demo/stop_sim.sh` in Terminal 2
ros2 launch ammo_mission_control cage_mission.launch.py world:=simple_trees headless:=true \
    mushrooms:=6,-5,-5,-5.5,-2,6.5          # demo mushrooms; the quad has to find them
ros2 launch ammo_mission_control cage_mission.launch.py parrot:=false count:=4   # rover-only test
ros2 launch ammo_mission_control cage_mission.launch.py hazard:=true              # + person stop on the rover
```

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `world` | `simple_trees` | World name, or a path to a generated `.sdf` |
| `headless` / `rviz` | `false` / `true` | No Gazebo GUI / RViz |
| `parrot` | `true` | Spawn the quad |
| `quad` | `zcam` | `zcam`: the z-camera quad (`quad_zcams`), plans round every obstacle with 0.6 m clearance and never climbs over canopies. `stock`: `quad_search_node` |
| `imgsz`, `center_out`, `det_range`, `quad_routes`, ... | see `quad_zcams/launch/zcam_mission.launch.py` | Quad search and rover routing options for `quad:=zcam`. The defaults are the configuration that collected 24/24 in a 40 m leaf-mesh forest (three runs, 23-26 min each) |
| `mushrooms` | empty | `x,y,x,y,...` mushrooms spawned for the quad to find |
| `count` | `0` | Random test mushrooms queued straight to the rover |
| `meta_goals` | `true` | Queue the world's own mushrooms (from `<world>.meta.json`) to the rover. `false` = the quad must find them |
| `quad_waypoints` | empty | `x,y,x,y,...` legs for the quad instead of the lawnmower |
| `yolo` / `husky_yolo` | `true` / `true` | Quad detector / rover mushroom detector (the rover's camera steers the final alignment, `false` to save CPU) |
| `hazard` | `false` | COCO person detector + `hazard_brake` on the rover |
| `tool` | `gripper` | `cage` selects the older boom cage (also needs `cage:=true` in `husky.urdf.xacro`) |

## Running the fleet by hand

`demo/part1_fleet.sh` is the tested configuration. By hand:

```bash
# Terminal 1
source demo/env.sh
ros2 launch quad_zcams fleet_mission.launch.py world:=$F/worlds/river_moderate_aus \
    meta_goals:=false quads:=2 rovers:=1 headless:=true rviz:=false

# Terminal 2
source demo/env.sh
ros2 launch ammo_gui_node fleet_debug.launch.py
```

- `fleet_debug.launch.py` also starts `mission_bridge`, which publishes the
  `ammo/mission/status` and `ammo/mission/events` topics the timeline and state cards read.
  Running `ros2 run ammo_gui_node fleet_debug` on its own shows the cameras and map, but the
  timeline stays empty, and Ctrl-C does not close it: use `bash demo/stop_sim.sh`. Do not run it alongside the operator GUI, which starts its own bridge.
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

1. **Search.** Each quad flies its own sector with passes 3 m apart. One quad flies square loops
   outward from the spawn point, two fly nested U loops, one in each half of the world, and three or
   more each fly an equal strip. Quads cruise 1 m above the local ground, on an A* path round
   everything in the world file and what the lidar sees, with 0.6 m clearance. They never climb over
   canopies: a sweep point a quad cannot reach is skipped.
2. **Confirm.** On a YOLO hit the quad closes to about 1.6 m from the projected point, faces it and
   holds still. It reports only when confidence is at least 0.5, the point is within 3 m of the quad
   and more than 2 m from the rover, the box is no taller than 1.35 times its width, the quad has been
   still for 1 s, and two projections agree within 1 m. With no report after 12 s it moves to a new
   viewpoint, up to three.
3. **Hand over.** After two waypoints 1 s apart the quad resumes searching. In a fleet run the report
   goes on `fleet/reports`, and the coordinator merges it with any known mushroom within 1.5 m and
   assigns it to a rover on `husky1/cage/waypoints`. With one quad it goes straight to that topic.
   The rover works in parallel.
4. **Collect.** The rover drives with Nav2 to a standoff 1.7 m from the mushroom on the clearest side,
   turns to face it (ALIGN), creeps in, lowers and closes the gripper, and publishes `husky1/cage/done`
   (or `husky1/cage/missed` if nothing was there, so the quad can find and dispatch it again). A point
   Nav2 cannot reach is tried from up to five directions, then moved to the back of the queue.
   During ALIGN the rover steers on its own camera's sighting (projected with the pose at that frame)
   when it is under 2.5 s old and within 15 deg of the odometry heading; otherwise on odometry. Native
   boxes are never a target.
5. **Finish.** Once every mushroom is known, every quad except `parrot1` goes home and lands, and
   `parrot1` lands when the mission completes. Once all are collected and
   every robot is home, a Part 1 run ends the simulation.

## Topic ownership

Never run two publishers of the same velocity topic.

- `husky1/cmd_vel` is written by `collision_guard_node`, except for four slow moves that bypass it on
  purpose, because its 1 m stop would otherwise keep the rover from reaching a mushroom: the final
  ALIGN creep, the PS4 L2 creep (at most 0.2 m/s), the river crossing lanes and the UNWEDGE reverse.
- `husky1/cmd_vel_raw` -> guard, with the full check: `rover_capture_node` (transit), teleop
  (`ps4_teleop`, `ipad_teleop.html`, keyboard).
- `husky1/cmd_vel_nav_guarded` -> guard, with the depth stop, rear stop and flank check only: Nav2
  (its `cmd_vel_smoothed`, remapped in `cage_mission.launch.py`). Nav2's costmap covers the rest.
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

With a mission running, in a second terminal after `source demo/env.sh`:
`ros2 run ammo_dashboard dashboard` opens one OpenCV HUD with both YOLO feeds, speed, altitude and a
minimap, with no Gazebo GUI. Add `--ros-args -p window:=false` to publish it for the iPad only.

The iPad page needs two more packages and three more terminals beside the mission:

```bash
sudo apt install -y ros-humble-rosbridge-server ros-humble-web-video-server

# Terminal 2, after source demo/env.sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# Terminal 3, after source demo/env.sh
ros2 run web_video_server web_video_server
# Terminal 4
python3 tools/ipad_teleop/ipad_teleop_server.py
```

Then open `http://<pc>:8000/` on the iPad.

## Other pieces

- **Quad state machine** (`quad_mission`): a standalone node, not part of the demo. It exposes
  `/parrot1/mission/*` services (liftoff, start search with a path shape, return home, land, abort)
  and publishes `parrot1/cmd_vel`, so never run it alongside the mission nodes. After
  `source demo/env.sh` in each terminal:

  ```bash
  # Terminal 1
  ros2 launch 41068_ignition_bringup 41068_ignition_parrot.launch.py
  # Terminal 2
  ros2 run quad_mission quad_mission_node
  # Terminal 3
  ros2 service call /parrot1/mission/liftoff std_srvs/srv/Trigger
  ```

- **Operator GUI**: `ros2 launch ammo_gui_node gui.launch.py` (PySide6 GUI with `mission_bridge`,
  `ammo_gui_node/`). A run starts in two stages: **Prepare** starts `cage_mission.launch.py`
  (one quad, one rover, PS4 teleop) and holds both robots while a checklist fills in and the
  mushrooms are spawned; **Launch** releases them and starts the clock. End Mission closes the
  simulation and returns to the setup page. See `ammo_gui_node/README.md`. Pause cancels the Nav2
  goal and holds both robots with zero velocity and `teleop_active` heartbeats. Freezing the node
  processes is a last resort; they get `ammo_gui_node/config/fastdds_udp_only.xml` (1 h discovery
  lease, loopback UDP) so the other processes keep them through a long freeze.
- **World generator**: `python3 41068_ignition_bringup/scripts/generate_forest_world.py --seed 7 --preset dense`
  writes a seeded forest `.sdf`, its terrain, and a `.meta.json` with the
  ground truth of every tree, mushroom and decoy, replacing the tracked `forest_random` files. See
  `worlds/README_forest_random.md`.
- **Detector training**: `skidsteer_perception/scripts/` captures labelled
  images in Gazebo and trains YOLO11n (`TRAINING_PC.md`).

## Known limitations

- Localisation is Gazebo ground truth (`truth_localizer`), a documented
  simplification that does not transfer to hardware.
- The quad projects a sighting by marching the camera ray to the terrain heightmap, so it needs a
  known terrain model. The older `quad:=stock` node uses its depth image and falls back to flat
  ground, which can add metres of error on uneven terrain (`forest_random`).
- Detection is reliable to about 3.5 m, hence the 3 m reporting range and 3 m rows.
- Native detection is trained on the sim's blue caps only (`river_v6`: blue caps at 1-5 m,
  river banks included). With the second-sighting rule, fleet runs on `river_moderate_aus`
  logged 4 to 6 of the 6 natives, because a run ends before the search covers everything, sent no
  native to the rover, and collected 12/12. One false native recurs at the water's edge near
  (17.7, -12.2). The cause is unconfirmed; it is logged only, never collected.
  The training data and scripts are in `~/mushroom_river` (`train_v6.sh`) on the capture PC,
  captured with `skidsteer_perception/scripts/run_capture.sh --hover 1.0 [--spots banks.json]`.
- The rover's camera steers the final alignment when it has a fresh sighting, and an empty grab sends
  the rover back once to where its camera saw the mushroom, but the camera cannot veto a target: a
  look-alike that passes the quad's checks gets collected.
- Pause does not fully hold a quad: the heartbeat runs on sim time, and a quad crept up to about
  0.5 m over 25 s on test runs.
- A report with no rover-camera sighting, missed after the quad covering that half has stood down, is
  never searched again, and the run stalls short of 12.
- "Collected" is a sim stand-in: any mushroom within 0.6 m of the gripper
  centre is removed from Gazebo.
