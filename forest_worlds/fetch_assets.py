"""Download the free assets the worlds are built from: Poly Haven CC0 models/textures and Forest3D's DEM.
python3 fetch_assets.py   (writes assets_src/, textures/, dem/; skips files already present)"""
import json
import os
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
opener = urllib.request.build_opener()
opener.addheaders = [('User-Agent', 'Mozilla/5.0 forest-worlds')]   # Poly Haven rejects the default agent
urllib.request.install_opener(opener)

MODELS = {
    'tree': ['fir_tree_01', 'pine_tree_01', 'pine_sapling_medium', 'fir_sapling_medium'],
    'bush': ['shrub_01', 'shrub_02', 'shrub_03', 'shrub_04', 'fern_02'],
    'rock': ['boulder_01', 'rock_moss_set_01', 'rock_moss_set_02', 'rock_07', 'stone_01'],
    'grass': ['grass_medium_01'],
}
TEXTURES = ['forest_leaves_04', 'brown_mud_leaves_01', 'river_small_rocks', 'forest_ground_04']
DEM = 'https://raw.githubusercontent.com/unitsSpaceLab/Forest3D/main/dem/terrain_enhanced.tif'


def get(url, path):
    if not os.path.exists(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        urllib.request.urlretrieve(url, path)


for cat, names in MODELS.items():
    for n in names:
        b = json.load(urllib.request.urlopen(f'https://api.polyhaven.com/files/{n}'))['blend']['1k']['blend']
        d = os.path.join(HERE, 'assets_src', cat)          # Forest3D layout: <cat>/<name>.blend + <cat>/textures/
        get(b['url'], os.path.join(d, f'{n}.blend'))
        for rel, inc in b.get('include', {}).items():
            get(inc['url'], os.path.join(d, rel))
        print('model', cat, n)
for t in TEXTURES:
    f = json.load(urllib.request.urlopen(f'https://api.polyhaven.com/files/{t}'))
    for k, key in (('Diffuse', 'diff'), ('nor_gl', 'nor_gl'), ('Rough', 'rough')):
        get(f[k]['2k']['jpg']['url'], os.path.join(HERE, 'textures', f'{t}_{key}.jpg'))
    print('texture', t)
get(DEM, os.path.join(HERE, 'dem', 'terrain_enhanced.tif'))
print('dem ok')
