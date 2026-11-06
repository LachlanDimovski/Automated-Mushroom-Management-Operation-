# Importing the mushroom mesh

Steps for getting a Sketchfab download into this model. Target is Gazebo Sim
6.18 (Fortress).

## 1. Format: Fortress does NOT read glTF

**Gazebo Fortress supports only `.dae`, `.obj` and `.stl`.** Its
`ignition-common4` ships exactly three mesh loaders:

```bash
nm -D /usr/lib/x86_64-linux-gnu/libignition-common4-graphics.so.4 \
  | grep -oiE "(Collada|OBJ|STL|Assimp|GLTF)Loader" | sort -u
# ColladaLoader  OBJLoader  STLLoader
```

There is **no glTF/GLB loader** — that arrived in gz-common5 (Garden). A `.glb`
model spawns as a perfectly valid entity with **no visible geometry and no
error message**: `ign model --list` shows it, the collision works, and nothing
renders. If a model is invisible but present, suspect the format first.

So a Sketchfab `.glb` must be converted. Either use Blender, or convert
headlessly with trimesh (this is how the current `mushroom.obj` was made — see
step 3).

## 2. Blender fixes worth doing regardless of format

Sketchfab models are almost never sim-ready. In Blender:

**Scale.** Check dimensions in the N panel. A real fly agaric is ~10-15 cm tall.
But your camera sees these from 3-9 m away, where a 12 cm mushroom is a handful
of pixels and will not be detected reliably. **Scale it up to roughly 0.3-0.5 m**
so it is comparable to the 0.15 m radius `target_ball`. Detection matters more
than botanical accuracy here.

**Origin at the base.** Select the mesh, put the 3D cursor at the bottom centre,
`Object > Set Origin > Origin to 3D Cursor`. Gazebo places models by their
origin, so a base origin means it sits on the ground rather than half-buried or
hovering.

**Apply transforms.** `Ctrl+A > All Transforms`. Un-applied scale/rotation is
the single most common cause of a mesh arriving in Gazebo at the wrong size or
lying on its side.

**Z-up.** Blender and Gazebo are both Z-up, so no axis conversion is needed —
but check the model is standing upright, since Sketchfab models are often
authored Y-up and arrive rotated. Rotate and re-apply transforms if so.

**Triangulate.** `Add Modifier > Triangulate`, or tick it on export. Avoids
n-gon artefacts.

## 3. Headless conversion (what was actually done here)

`mushroom.obj` was produced from `amanita_muscaria.glb` with this script. It
fixes the axis convention, the origin and the scale in one pass, and writes the
textures out alongside:

```python
import trimesh, numpy as np
s = trimesh.load('meshes/amanita_muscaria.glb')

# glTF is Y-up, Gazebo is Z-up
s.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))

# recentre on x/y, put the base at z=0
lo, hi = s.bounds
s.apply_transform(trimesh.transformations.translation_matrix(
    [-(lo[0]+hi[0])/2, -(lo[1]+hi[1])/2, -lo[2]]))

# scale to 0.40 m tall
s.apply_transform(trimesh.transformations.scale_matrix(0.40 / s.extents[2]))

s.export('meshes/mushroom.obj')   # writes material.mtl + texture PNGs too
```

Needs `pip install trimesh pycollada pillow`. The source `.glb` is kept in
`meshes/` so the conversion can be redone with different parameters.

Because the transform is baked into the mesh, `model.sdf` uses
`<scale>1 1 1</scale>` — do not scale it again there.

## 4. Exporting DAE from Blender (alternative)

`File > Export > Collada (.dae)`, into `meshes/`:

- **Include Material Textures** — on
- **Copy** textures — on, so they land next to the mesh
- **Apply Modifiers** — on
- **Triangulate** — on
- **Global Orientation** — leave at Blender defaults (Z-up)

Then put the texture files in `materials/textures/` and check the paths that the
`.dae` references. Open it in a text editor and look for `<init_from>` — the
paths must be **relative** (`../materials/textures/foo.png`), not absolute
(`/home/henry/Downloads/...`). Absolute paths are the usual reason a mesh loads
untextured on someone else's machine.

## 5. Wire it into the sim

Worlds place the mushrooms themselves: `forest_worlds/build_world.py` writes them into the river worlds, and
`mushrooms:=x,y,x,y` on `cage_mission.launch.py` spawns them in any other world. With the origin at the mushroom's
base, it sits on the ground.

Then rebuild, because models are read from `install/share`, not `src`:

```bash
cd ~/41068_ws && colcon build --packages-select 41068_ignition_bringup
```

## 6. Check it before trusting it

```bash
source ~/41068_ws/src/AMMO-Mushroom-Fleet/demo/env.sh
ros2 launch ammo_mission_control cage_mission.launch.py mushrooms:=6,-5
```

Watch `husky1/detections/image` in RViz. Two separate questions:

1. **Does the mushroom look right in Gazebo?** Correct size, upright, textured,
   sitting on the ground rather than sunk or floating.
2. **Does YOLO detect it?** This is the real unknown. The model was trained on
   photographs of real mushrooms; a rendered mesh under simulated lighting is a
   different domain.

If it renders correctly but goes undetected, the mesh's appearance is the
likely cause. The detector learned **white warts on a red cap** — a plain red
mesh with no spots is missing the feature it keys on most. A texture with
visible white spots is worth more than any amount of retraining.

## Troubleshooting

| Symptom | Cause |
|---|---|
| Mesh invisible, no error | `<uri>` filename does not match the file in `meshes/` |
| Loads white/untextured | Absolute or broken texture paths in the `.dae` |
| Huge or microscopic | Un-applied scale in Blender; fix and re-export |
| Lying on its side | Y-up source; rotate in Blender, apply transforms |
| Sunk into ground | Origin not at base, or spawn `-z` too low |
| Works for you, not teammates | Textures not committed, or absolute paths |

If `model://` URIs fail to resolve outside the launch files, add the models to the resource path:

```bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:~/41068_ws/install/41068_ignition_bringup/share/41068_ignition_bringup/models
```

## Attribution

Sketchfab models carry licences — most are CC-BY, requiring credit. Record the
model name, author, URL and licence here before committing:

- Source:
- Author:
- Licence:
