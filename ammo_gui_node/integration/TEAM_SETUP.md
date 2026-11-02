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
sudo apt install -y libxcb-cursor0     # Qt 6.5+ needs this for the xcb plugin
pip3 install PySide6
```

That is the whole list for the GUI. No OpenCV.

## 3. Build - after the LFS pull, not before

```bash
cd ~/41068_ws
colcon build
source install/setup.bash
```

Plain `colcon build`, **never `--symlink-install`** - this workspace was built
without it and mixing the two gives
`PackageNotFoundError: No package metadata was found for ammo-gui-node`.

Order matters: the build copies `41068_ignition_bringup/models` into
`install/`, so a build done before `git lfs pull` leaves pointer files in the
installed copy however good the source tree looks. If you built first, just
build again.

## 4. Check it without starting a simulation

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
and refuses to start a second one.

## 6. The river world, if you want it

`simple_trees` works out of the box. `river_moderate_aus` is **generated per
machine** - it is not in git and cannot be, since `forest_worlds/.gitignore`
excludes the built worlds, models and textures.

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
| `PackageNotFoundError: ammo-gui-node` | Built with `--symlink-install` at some point. `rm -rf build/ammo_gui_node install/ammo_gui_node src/AMMO-Mushroom-Fleet/ammo_gui_node/*.egg-info`, rebuild plain, new terminal |
| Gazebo will not close | `for p in $(pgrep -x ruby); do grep -qa ign /proc/$p/cmdline && kill -9 "$p"; done` |
| River world missing from the dropdown | `forest_worlds/setup.sh` has not been run on this machine |
| Ticked the Gazebo window, got no window | More than one robot of either kind runs headless on purpose. The checkbox greys itself out and says so |

The GUI's own event log names most of these as they happen, including the
LFS one, with the command that fixes it.
