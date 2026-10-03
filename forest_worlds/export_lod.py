"""Blender (5.x) export of Poly Haven assets as light Gazebo (Fortress: OBJ, no glTF) models: one model per variant, one LOD each.

blender -b X.blend --python export_lod.py -- <out_models_dir> <category> <lod> <max_faces>
"""
import os
import re
import sys

import bpy

out_dir, cat, lod, max_faces = sys.argv[sys.argv.index('--') + 1:]
lod, max_faces = int(lod), int(max_faces)
base = os.path.splitext(os.path.basename(bpy.data.filepath))[0]

variants = {}
for o in bpy.data.objects:
    if o.type != 'MESH':
        continue
    m = re.match(r'(.*)_LOD(\d)$', o.name)
    if m:
        variants.setdefault(m.group(1), {})[int(m.group(2))] = o
# no LODs (single mesh assets): every top-level mesh is one model
if not variants:
    variants = {base: {lod: o for o in bpy.data.objects if o.type == 'MESH' and o.parent is None}}

for vname, lods in variants.items():
    obj = lods.get(lod) or lods[max(lods)]
    for o in bpy.context.view_layer.objects:
        o.select_set(False)
    if obj.name not in bpy.context.view_layer.objects:
        bpy.context.scene.collection.objects.link(obj)
    obj.hide_set(False)
    obj.hide_viewport = obj.hide_render = False
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    obj.data = obj.data.copy()
    a = obj.data.attributes.get('UVMap')
    if not obj.data.uv_layers and a is not None and a.domain == 'CORNER':   # UVs stored as a 3D attribute
        vals = [v.vector[:2] for v in a.data]
        obj.data.attributes.remove(a)
        uv = obj.data.uv_layers.new(name='UVMap')
        for d, v in zip(uv.data, vals):
            d.uv = v
    n = len(obj.data.polygons)
    if n > max_faces:
        d = obj.modifiers.new('dec', 'DECIMATE')
        d.ratio = max_faces / n
        bpy.ops.object.modifier_apply(modifier='dec')
    obj.location = (0, 0, 0)
    obj.rotation_euler = (0, 0, 0)
    dx, dy, dz = obj.dimensions
    nf = len(obj.data.polygons)
    # one OBJ group per material so the SDF can texture each submesh (leaf alpha, bark normals)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.separate(type='MATERIAL')
    bpy.ops.object.mode_set(mode='OBJECT')
    for part in bpy.context.selected_objects:
        if part.material_slots and part.material_slots[0].material:
            part.name = part.material_slots[0].material.name

    name = re.sub(r'_LOD\d$', '', vname)
    mdir = os.path.join(out_dir, cat, name)
    os.makedirs(os.path.join(mdir, 'mesh'), exist_ok=True)
    bpy.ops.wm.obj_export(filepath=os.path.join(mdir, 'mesh', name + '.obj'), export_selected_objects=True,
                          export_materials=True, path_mode='COPY', forward_axis='Y', up_axis='Z',
                          export_triangulated_mesh=True)
    with open(os.path.join(mdir, 'dims.txt'), 'w') as fh:
        fh.write(f'{dx} {dy} {dz} {nf}\n')
    print('EXPORTED', name, nf, f'{dx:.2f}x{dy:.2f}x{dz:.2f}')
