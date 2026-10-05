#!/bin/bash
# One-time build of the river forest worlds (needs internet, Blender >= 4.2, python3 with numpy + pillow).
#   bash setup.sh            -> downloads assets, exports models, builds sparse/moderate/dense/hilly
#   ./view.sh moderate       -> open one in Gazebo
set -e
cd "$(dirname "$0")"
BLENDER=${BLENDER:-blender}
python3 fetch_assets.py

# Poly Haven -> light Gazebo models: one LOD per variant, capped face counts for integrated GPUs
export_cat() {  # category lod max_faces
    for b in assets_src/$1/*.blend; do "$BLENDER" -b "$b" --python export_lod.py -- models "$1" "$2" "$3" > /dev/null 2>&1; done
}
export_cat tree 2 12000
export_cat bush 1 1000
export_cat rock 1 1500
export_cat grass 1 400
for r in rock_07 stone_01; do "$BLENDER" -b assets_src/rock/$r.blend --python export_lod.py -- models rock 1 600 > /dev/null 2>&1; done
rm -rf models/*/*twig* models/grass/*geonodes* models/grass/*tiny* models/bush/shrub_01_[ghi]

python3 make_sdf.py models assets_src
python3 split_obj.py models
for f in models/*/*/mesh/*.obj; do
    sed -i '/^mtllib/d' "$f"
    grep -q '^usemtl' "$f" || sed -i "1i usemtl $(basename "$f" .obj)" "$f"
done
rm -f models/*/*/mesh/*.mtl
# leaf cards cast no shadows (they flicker in Fortress)
python3 - <<'PY'
import glob, re
for f in glob.glob('models/tree/*/model.sdf'):
    t = open(f).read()
    open(f, 'w').write(re.sub(r'(<visual name="[^"]*twig[^"]*".*?<cast_shadows>)true(</cast_shadows>)', r'\1false\2', t))
PY

python3 make_textures.py
for p in sparse moderate dense hilly; do python3 build_world.py --preset "$p"; done
python3 make_gums.py
python3 build_world.py --preset moderate --flora australian   # the mushroom demo world (demo_river.sh)
echo "done - open one with: ./view.sh moderate"
