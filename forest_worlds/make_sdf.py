"""Write model.sdf/model.config for every exported model: per-submesh PBR textures (RGBA albedo for cut-out
leaves), simple collision for trunks and big rocks. python3 make_sdf.py <models_dir> <blender_assets_dir>"""
import os
import re
import sys

import numpy as np
from PIL import Image

models, assets = sys.argv[1:3]


def tex(cat, mat, kind):
    d = os.path.join(assets, cat, 'textures')
    names = sorted(os.listdir(d))
    for stem in (mat, mat.rstrip('s'), mat + 's'):
        for n in names:
            if n.startswith(f'{stem}_{kind}_') and not n.endswith('.exr'):
                return os.path.join(d, n)
    return None


def collision(cat, dx, dy, dz):
    if cat == 'tree':   # trunk only: lidar sees the foliage, the rover only bumps the trunk
        return (f'<collision name="trunk"><pose>0 0 2 0 0 0</pose><geometry><cylinder><radius>0.25</radius>'
                f'<length>4</length></cylinder></geometry></collision>')
    if cat == 'rock' and min(dx, dy) > 0.6 or cat == 'bush' and dz > 0.8:
        return (f'<collision name="body"><pose>0 0 {dz * 0.4:.3f} 0 0 0</pose><geometry><box><size>'
                f'{dx * 0.8:.3f} {dy * 0.8:.3f} {dz * 0.8:.3f}</size></box></geometry></collision>')
    return ''


for cat in sorted(os.listdir(models)):
    for name in sorted(os.listdir(os.path.join(models, cat))):
        mdir = os.path.join(models, cat, name)
        obj = os.path.join(mdir, 'mesh', name + '.obj')
        groups = [l.split(None, 1)[1].strip() for l in open(obj) if l.startswith('o ')]
        dx, dy, dz, _ = map(float, open(os.path.join(mdir, 'dims.txt')).read().split())
        os.makedirs(os.path.join(mdir, 'materials'), exist_ok=True)
        vis = []
        for g in groups:
            mat = re.sub(r'\.\d+$', '', g)
            diff, alpha, nor = tex(cat, mat, 'diff'), tex(cat, mat, 'alpha'), tex(cat, mat, 'nor_gl')
            m = '<diffuse>1 1 1 1</diffuse><pbr><metal><metalness>0</metalness><roughness>0.9</roughness>'
            if diff:
                out = f'{mat}_albedo.png'
                if not os.path.exists(os.path.join(mdir, 'materials', out)):
                    im = Image.open(diff).convert('RGB')
                    if alpha:
                        a = Image.open(alpha).convert('L').resize(im.size)
                        im = Image.fromarray(np.dstack([np.asarray(im), np.asarray(a)]), 'RGBA')
                    im.save(os.path.join(mdir, 'materials', out))
                m += f'<albedo_map>model://{name}/materials/{out}</albedo_map>'
            if nor:
                out = f'{mat}_normal.png'
                if not os.path.exists(os.path.join(mdir, 'materials', out)):
                    Image.open(nor).convert('RGB').save(os.path.join(mdir, 'materials', out))
                m += f'<normal_map>model://{name}/materials/{out}</normal_map>'
            m += '</metal></pbr>'
            vis.append(f'<visual name="{mat}_{len(vis)}"><geometry><mesh><uri>model://{name}/mesh/{name}.obj</uri>'
                       f'<submesh><name>{g}</name></submesh></mesh></geometry><material>{m}</material>'
                       f'<cast_shadows>{"true" if cat in ("tree", "rock") else "false"}</cast_shadows></visual>')
        for f in os.listdir(os.path.join(mdir, 'mesh')):   # Blender's copied textures are now unused
            if not f.endswith(('.obj', '.mtl')):
                os.remove(os.path.join(mdir, 'mesh', f))
        with open(os.path.join(mdir, 'model.sdf'), 'w') as fh:
            fh.write(f'<?xml version="1.0"?>\n<sdf version="1.8"><model name="{name}"><static>true</static>'
                     f'<link name="link">{"".join(vis)}{collision(cat, dx, dy, dz)}</link></model></sdf>\n')
        with open(os.path.join(mdir, 'model.config'), 'w') as fh:
            fh.write(f'<?xml version="1.0"?>\n<model><name>{name}</name><version>1.0</version>'
                     f'<sdf version="1.8">model.sdf</sdf><description>Poly Haven CC0 {cat}</description></model>\n')
        print(name, len(groups), 'submeshes', f'{dx:.1f}x{dy:.1f}x{dz:.1f}')
