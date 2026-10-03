# River forest worlds

Realistic forest worlds for Ignition Gazebo Fortress: terrain from Forest3D's real elevation survey, a river with
three rocky fords, a hedge round the map edge, Poly Haven CC0 trees, shrubs, rocks and grass. Built to run on
integrated graphics (about 2 M triangles, all physics in one static link).

## Run it on any laptop

Tested on Ubuntu 22.04 with ROS 2 Humble and Ignition Gazebo Fortress (6.x) on an Intel UHD 620 laptop with 8 cores.
You need about 2.5 GB of free disk and an internet connection for the first build.

1. Gazebo Fortress (skip if `ign gazebo --versions` already prints 6.x):

   ```bash
   sudo apt update && sudo apt install -y ignition-fortress
   ```

2. Blender 4.2 or newer and the Python libraries:

   ```bash
   sudo snap install blender --classic
   sudo apt install -y python3-numpy python3-pil
   ```

3. Get the code and build the worlds (about 5 minutes; downloads about 1.2 GB of free CC0 assets):

   ```bash
   cd ~/41068_ws/src/RS1_Team5_        # wherever the repo is cloned
   git fetch
   git checkout mushroom-cage
   git pull
   cd forest_worlds
   bash setup.sh
   ```

   If Blender is not on your PATH, run `BLENDER=/path/to/blender bash setup.sh`.

4. Open a world:

   ```bash
   ./view.sh moderate
   ```

   The window takes 20 to 60 seconds to load the meshes on integrated graphics; wait for the trees to appear.

Nothing the build creates is committed (see `.gitignore`), so re-run `bash setup.sh` after pulling changes.

## Open a world

```bash
./view.sh moderate     # sparse | moderate | dense | hilly; closes any Gazebo already running
```

To use one in your own launch, add `forest_worlds/models/*` and `forest_worlds/world_assets` to
`IGN_GAZEBO_RESOURCE_PATH` and load `forest_worlds/worlds/river_<preset>.sdf`.

| Preset   | Trees | Terrain relief | Boulder clusters |
|----------|-------|----------------|------------------|
| sparse   | 43    | 2.5 m          | 10               |
| moderate | 65    | 3.4 m          | 18               |
| dense    | 137   | 3.0 m          | 22               |
| hilly    | 58    | 6.5 m          | 35               |

## Crossing the river

The channel is a 0.75 m drop, so robots can only cross at the fords (printed by `build_world.py`; with the default
seed they are at about (-8.8, -4.8), (8.8, -1.5) and (-22.0, -13.8)). Each ford is a pebble causeway with boulders on
both edges and a clear lane about 3.2 m wide. The water is visual only.

## Rebuilding

`python3 build_world.py --preset dense --seed 4` makes a different layout. Any preset value can be overridden, for
example `--trees 80 --relief 4`.
