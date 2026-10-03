# River forest worlds

Realistic forest worlds for Ignition Gazebo Fortress: terrain from Forest3D's real elevation survey, a river with
three rocky fords, a hedge round the map edge, Poly Haven CC0 trees, shrubs, rocks and grass. Built to run on
integrated graphics (about 2 M triangles, all physics in one static link).

## Setup (once)

Needs internet, Blender 4.2 or newer (`sudo snap install blender --classic`), and python3 with numpy and pillow.

```bash
cd forest_worlds
bash setup.sh          # ~20 min: downloads ~1.2 GB of assets, exports models, builds the four presets
```

Nothing it generates is committed (see `.gitignore`).

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
