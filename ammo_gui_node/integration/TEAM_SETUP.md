# Running the AMMO GUI on your machine

One page. Five minutes, plus the world build if you want the river world.

## 1. Install Git LFS first - this is the step people skip

```bash
sudo apt install -y git-lfs
cd ~/41068_ws/src/AMMO-Mushroom-Fleet
git lfs install
git lfs pull
```

`41068_ignition_bringup/.gitattributes` puts every `*.png`, `*.jpg`, `*.obj`
and `*.mtl` in Git LFS. Without `git-lfs`, a clone or pull writes 130-byte
text pointers in place of the real files - the mushroom mesh and its texture,
the tree bark, the terrain heightmaps, the pond and hedge meshes.

This fails silently, which is why it cost us a week. Gazebo loads a model
whose mesh will not parse as a real entity with **no geometry on it**:
`create` reports success, `ign model --list` lists it, the collision is
there - and nothing is drawn. So the mushrooms spawn invisibly, the
quadcopter's camera sees bare ground, YOLO has nothing to detect, and every
count reads zero with no error anywhere.

Check it worked:

```bash
ls -la 41068_ignition_bringup/models/mushroom_amanita/meshes/mushroom.obj
```

About **1.3 MB** is right. **130 bytes** means LFS has not run.

## 2. The rest of the dependencies

```bash
sudo apt install -y libxcb-cursor0 python3-scipy python3-pil
pip3 install PySide6
pip3 install ultralytics
pip3 install "opencv-python<5" "numpy<2"   # see below - LAST, and both together
pip3 install torch --index-url https://download.pytorch.org/whl/cpu
pip3 install openvino                  # only needed to run a fleet
```

**Pin numpy below 2, and do it after anything that might pull numpy in.**
`cv_bridge` and the torch wheels for Humble are C extensions built against
the numpy 1.x ABI. `pip install ultralytics` cheerfully installs numpy 2.x on
top of them, and the result does not complain about versions - it fails with
an `ImportError` deep inside `cv_bridge` that never mentions numpy, or torch
prints a wall of text about a module compiled with NumPy 1.x. The detectors
then never start, so `*/detections` has no publisher, the readiness checklist
never ticks them, and nothing is ever detected.

**Pin opencv in the same breath.** `ultralytics` installs whatever
`opencv-python` pip offers it, and that is now 5.x, which *requires* numpy
>= 2. So pinning numpy alone leaves pip announcing a conflict it cannot
resolve, and pinning opencv alone leaves numpy 2 in place. Pin both, in one
command, so pip settles them together. opencv 4.x is what ultralytics has
always run against.

Nothing in the repo declares any of this. `perception/package.xml` lists
rclpy, sensor_msgs, vision_msgs and cv_bridge, while `perception/perception/
yolo_node.py` imports torch, cv2 and ultralytics - so `rosdep install` cannot
produce a working machine, and everyone has pip-installed by hand on a
different day. That is why a run that works for one of us fails for the next
with no visible difference. The checker in step 4 is the missing declaration.

The GUI itself needs only PySide6 and numpy - it converts ROS images to Qt
images directly and never touches OpenCV.

## 3. Build - after the LFS pull, not before

```bash
cd ~/41068_ws
colcon build --symlink-install
source install/setup.bash
```

**Build everything, not just the package you are working on.** `colcon build`
copies `41068_ignition_bringup/models` into `install/`, and that copy is what
Gazebo's resource path points at. Rebuilding only `ammo_gui_node` for weeks
leaves an installed model set from whenever you last built bringup - and a
model added since then is missing, so the world fails to load with
`Unable to find uri[model://...]` and the server exits, which looks exactly
like Gazebo opening black and closing on its own.

Use one build style for the life of a workspace. A new workspace builds with
`--symlink-install`, as the top-level README does; a workspace first built
without it must stay plain. Mixing the two gives
`PackageNotFoundError: No package metadata was found for ammo-gui-node`.

Order matters: the build copies `41068_ignition_bringup/models` into
`install/`, so a build done before `git lfs pull` leaves pointer files in the
installed copy however good the source tree looks. If you built first, just
build again.

## 4. Check the machine, then the code

**Run this first, and run it any time something behaves oddly:**

```bash
python3 src/RS1_Team5_/ammo_gui_node/tests/check_env.py
```

It checks every one of the above - python, numpy's version against what
cv_bridge and torch can take, ROS and the workspace, torch, ultralytics,
cv2, openvino, PySide6 and whether Qt can actually start, Gazebo on the
PATH, git-lfs, and whether the mushroom mesh, the terrain heightmaps and the
YOLO weights are real files or LFS pointers, in both the source tree and the
built `install/` copy. Each line says what it found and the one command that
fixes it, and it ends with a list of what will stop you running. No ROS, no
simulation and no display needed.

Then the code's own tests:

```bash
python3 src/AMMO-Mushroom-Fleet/ammo_gui_node/tests/run_tests.py
```

A few seconds, no ROS, no Gazebo, no display. It prints two blocks (107 and
then the rest) and ends with "everything passed". If this fails, it is a
workspace problem, not a simulation problem - much faster to find here.

## 5. Run it - two terminals

```bash
# 1. operator GUI (starts the status bridge, and Gazebo, itself)
cd ~/41068_ws && source /opt/ros/humble/setup.bash && source install/setup.bash \
  && export QT_QPA_PLATFORM=xcb && export LIBGL_ALWAYS_SOFTWARE=1 \
  && ros2 launch ammo_gui_node gui.launch.py

# 2. Debug UI (optional, read-only, open or close any time)
cd ~/41068_ws && source /opt/ros/humble/setup.bash && source install/setup.bash \
  && export QT_QPA_PLATFORM=xcb && export LIBGL_ALWAYS_SOFTWARE=1 \
  && ros2 launch ammo_gui_node debug.launch.py
```

There is no third terminal for the simulation. The GUI starts Gazebo itself
and refuses to start a second one. For the demo, `bash demo/part2_gui.sh` does
terminal 1 in one step (see the top-level README).

## 6. The river world, if you want it

`simple_trees` works out of the box. `river_moderate_aus` is not in git
(`forest_worlds/.gitignore` excludes the built worlds, models and textures).
Download the built worlds from the repo's v1.0 release:

```bash
bash ~/41068_ws/src/AMMO-Mushroom-Fleet/demo/get_world.sh
```

Or build them yourself:

```bash
sudo snap install blender --classic
sudo apt install -y python3-numpy python3-pil
cd ~/41068_ws/src/AMMO-Mushroom-Fleet/forest_worlds && bash setup.sh
./view.sh moderate_aus          # check it opens on its own first
```

About five minutes and roughly 1.2 GB of CC0 asset downloads. Until that file
exists the world simply is not in the GUI's dropdown. It is the only world
with natives in it and the only one the two-class `amanita_river_v6` weights
were trained against.

## If something goes wrong

| What you see | What it is |
|---|---|
| Mushrooms never appear, counts stay zero | Git LFS not pulled, or built before pulling |
| Gazebo black screen, no camera or odometry ever arrives | Same - the terrain heightmap is a pointer. Also untick **Show the Gazebo window** |
| `PackageNotFoundError: ammo-gui-node` | The workspace mixes plain and `--symlink-install` builds. `rm -rf build/ammo_gui_node install/ammo_gui_node src/AMMO-Mushroom-Fleet/ammo_gui_node/*.egg-info`, rebuild plain, new terminal |
| Gazebo will not close | `for p in $(pgrep -x ruby); do grep -qa ign /proc/$p/cmdline && kill -9 "$p"; done` |
| River world missing from the dropdown | Run `bash demo/get_world.sh` (or `forest_worlds/setup.sh`) |
| `Unable to find uri[model://...]`, Gazebo opens black then closes | A stale `install/`. `colcon build --packages-select 41068_ignition_bringup` - the resource path points at the built copy, not your source tree |
| Detectors never tick on the checklist | numpy 2.x on top of cv_bridge/torch. `pip3 install "numpy<2"` |
| `ImportError` in cv_bridge mentioning nothing useful | Same - it is the numpy ABI, not a missing package |
| pip says opencv-python requires numpy>=2 | Pin both: `pip3 install "opencv-python<5" "numpy<2"` |
| Ticked the Gazebo window, got no window | More than one robot of either kind runs headless on purpose. The checkbox greys itself out and says so |

The GUI's own event log names most of these as they happen, including the
LFS one, with the command that fixes it.
