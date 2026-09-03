# Randomised forest world (`forest_random`)

A procedurally generated open-forest world for stress-testing navigation:
rolling hills, a green grass floor (leaf litter up the hillsides), mesh trees,
whole trees fallen on their side (mesh) plus broken logs, rocks and low
clutter, a few ponds in carved basins, plus mushrooms + decoys as a mission
payload. Reproducible from a seed.

The robot spawns in a broad natural hollow with the hills rising around it; the
spawn pad and the lanes stay drivable.

**Fully separate from `simple_trees` / `large_demo`** — nothing here changes the
existing worlds or launch files.

## Files it owns

| Path | What |
|---|---|
| `scripts/generate_forest_world.py` | the generator (numpy + Pillow, no ROS) |
| `worlds/forest_random.sdf` | the world (generated) |
| `worlds/forest_random.meta.json` | seed, params, ground-truth pose of every object (generated) |
| `models/forest_random_terrain/` | heightmap model (generated: `heightmap.png`, `terrain_diffuse.png`, `flat_normal.png`, SDF) |
| `models/tree_oak/`, `models/tree_pine/` | vendored OSRF Fuel tree meshes + textures, mesh collision swapped for a trunk cylinder |
| `launch/forest_world.launch.py` | standalone bringup, mirrors `41068_ignition.launch.py` |

Trees are the `tree_oak` / `tree_pine` models: a real **mesh trunk** (cheap
trunk-cylinder collision) plus, by default, one **opaque canopy shape**
(`--foliage simple`). The alpha foliage *mesh* (`--foliage mesh`) looks better
but is the dominant GUI render cost - only use it on a discrete GPU. A fraction
of trees are bare-cylinder dead "snags". Fully local, deterministic.

Counts are tuned for **Intel integrated graphics**. On a real GPU, push
`--tree-density` up or use `--preset dense`.

## Use

```bash
# 1. generate (from the source tree)
python3 src/RS1_Team5_/41068_ignition_bringup/scripts/generate_forest_world.py --seed 7 --preset moderate

# 2. build  (first time, and any time --name adds a new terrain model dir;
#            regenerating an existing name just rewrites symlinked files)
colcon build --packages-select 41068_ignition_bringup
source install/setup.bash

# 3. run
ros2 launch 41068_ignition_bringup forest_world.launch.py
ros2 launch 41068_ignition_bringup forest_world.launch.py nav2:=true rviz:=true
ros2 launch 41068_ignition_bringup forest_world.launch.py parrot:=true
```

Launch args mirror `41068_ignition.launch.py`: `use_sim_time`, `husky`, `parrot`,
`slam`, `nav2`, `rviz`, `yolo`, `headless`.

## Presets

| Preset | Character (approx, per seed) |
|---|---|
| `light` | ~30 trees - smoothest GUI on weak hardware |
| `sparse` | low density, gentle hills - baseline / sanity |
| `moderate` | ~65 trees, ~19 fallen, ~3 m hills, 2 ponds (default) |
| `dense` | ~140 trees, tight gaps - planner stress (needs a GPU) |
| `blowdown` | ~40 whole fallen trees on their side - obstacle course |
| `hilly` | ~5 m terrain range, 4 ponds - odometry / 2D-SLAM stress |

Fallen items are ~80% whole tree meshes tipped on their side (trunk-cylinder
collision, an upturned root disc) and ~20% broken log sections. They bias
toward standing trees and are kept off the clearing and lanes.

The terrain diffuse is one image baked from the height field (green valley ->
brown slope -> pale dry hilltop) and mapped 1:1, so the elevation gradient
reads clearly without the multi-texture blend that stippled on integrated GPUs.

### If the GUI is sluggish (integrated graphics)

- `--preset light`, or lower `--tree-density` / `--size`
- `ros2 launch ... forest_world.launch.py husky:=false` - just the scenery
- `ros2 launch ... forest_world.launch.py headless:=true rviz:=true` - no 3D
  viewport at all; RViz shows scan / map / costmap / path, which is what nav
  dev actually needs

Any explicit flag overrides the preset. Key knobs:
`--size`, `--tree-density`, `--logs`, `--rocks`, `--clutter`, `--ponds`,
`--terrain-amplitude` (hill height), `--terrain-roughness`,
`--heightmap-samples` ((2^n)+1), `--clearing-radius`, `--lanes`, `--min-gap`,
`--mushrooms`, `--decoys`, `--boundary {trees,wall,none}`, `--name`. See `--help`.

Ponds are visual-only water discs in carved basins - the basin walls are what
physically stop the robot. Terrain `z_span` (actual height range, hills +
basins) is recorded in `meta.json`.

## What the world does to the nav stack (known, by design)

1. **Low obstacles are invisible to Nav2.** The costmaps
   (`nav2_params_husky1.yaml`) mark from `scan` only, and the lidar sits ~0.75 m
   up. Fallen logs (top ~0.2-0.35 m), rocks and clutter are below the beam, so
   the planner never sees them - only `collision_guard_node`'s forward depth
   check stops the robot. This is the point of `blowdown`. If you want the
   planner to see them, add `camera/depth/points` as a costmap
   `observation_source`.
2. **`inflation_radius: 2.0` vs forest density.** Two trees closer than ~4 m have
   fully overlapping inflation, so in `dense` woods the planner will report "no
   path" through gaps the 1.0 m robot physically fits. `--min-gap` plus the
   `meta.json` object list let you measure where that line is.
3. **Terrain vs 2D SLAM.** Slopes pitch the robot and the flat scan then sweeps
   into / over the ground. The spawn clearing and lanes are kept flat; raise
   `--terrain-amplitude` deliberately as a stressor.

## `meta.json` (ground truth for scoring)

```
seed, preset, params
world:            { name, playable_size_m, terrain_size_m }
terrain:          { model, heightmap, samples, requested_amplitude_m, z_span_m }
spawn_clearing:   { center, radius, ground_z }
lanes:            [[ [x,y], ... ], ...]
counts:           { trees, fallen_trees, logs, rocks_stumps_clutter, ponds, boundary, mushrooms, decoys }
mushrooms/decoys: [ { name, x, y, z }, ... ]
ponds:            [ { name, x, y, z, radius, basin_radius, depth }, ... ]
objects:          [ { name, type, x, y, z, yaw, radius/length/height, ... }, ... ]
```

## Phase 2 (not built yet)

A `forest_scorer` node + batch multi-seed runner that consumes `meta.json`:
collisions (Gazebo `/world/<world>/contacts`), area covered, Nav2 "no path"
events, time / success to locate all mushrooms - run seeds 1..N headless and
aggregate a pass-rate + failure-mode table.

## Optional integration (deferred - do when ready)

- add `forest_random` to the `world` `choices` in `41068_ignition.launch.py`
  and `skidsteer_test.launch.py` (+ a `scatter:=` toggle) to reach it via the
  normal `world:=` arg
- add the generator to `install(PROGRAMS ...)` in `CMakeLists.txt` for `ros2 run`
- add `<exec_depend>python3-pil</exec_depend>` to `package.xml`
