#!/usr/bin/env python3
"""Forest3D-populated trees + this package's own water/fallen-tree/boulder
obstacles, combined into one world.

Builds on generate_forest_world_forest3d.py (Forest3D's WorldPopulator places
the trees) and adds what Forest3D has no assets for: ponds carved into the
terrain (emit_water), fallen trees / broken logs (emit_fallen_tree / emit_log),
and rock/stump boulders (emit_blob) as extra lidar/nav obstacles. All reused
directly from generate_forest_world.py - no reimplementation.

Usage:
    python3 generate_forest_world_forest3d_enriched.py --seed 7 --preset dense
"""

import argparse
import json
import math
import os
import sys
from xml.etree import ElementTree as ET

import numpy as np
from stl import mesh as stl_mesh

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'third_party', 'forest3d', 'src'))
from forest3d.core.forest import WorldPopulator  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
from generate_forest_world import (  # noqa: E402
    PRESETS, build_heightfield, write_terrain_model, _world_grid, _default_pkg_dir,
    carve_basin, sample_elevation, scatter_points, sample_tree,
    emit_fallen_tree, emit_log, emit_blob, emit_water, emit_include, _f,
)

# Standing dead trees join WorldPopulator's normal 'tree' variant pool (mixed in
# with live oak/pine); the fallen one is already modelled lying down, so it's
# placed through the log/fallen-tree scatter loop instead, untipped.
DEADTREE_STANDING_VARIANTS = ('deadtree_halloween', 'deadtree_stump')
DEADTREE_FALLEN_MODEL = 'deadtree_fallen'

CLEARING_RADIUS_M = 4.0


def write_terrain_stl(models_dir, field, terrain_size):
    """STL matching write_terrain_model's world mapping, for WorldPopulator's
    barycentric height sampling (it reads models/ground/mesh/terrain.stl
    directly, separate from the heightmap.png Gazebo renders)."""
    n = field.shape[0]
    gx, gy = _world_grid(n, terrain_size)
    verts = np.stack([gx, gy, field], axis=-1).reshape(-1, 3)

    faces = []
    for y in range(n - 1):
        for x in range(n - 1):
            v0, v1, v2, v3 = y * n + x, y * n + x + 1, (y + 1) * n + x, (y + 1) * n + x + 1
            faces.append([v0, v1, v2])
            faces.append([v1, v3, v2])
    faces = np.array(faces)

    terrain = stl_mesh.Mesh(np.zeros(len(faces), dtype=stl_mesh.Mesh.dtype))
    for i, f in enumerate(faces):
        for j in range(3):
            terrain.vectors[i][j] = verts[f[j]]

    mesh_dir = os.path.join(models_dir, 'ground', 'mesh')
    os.makedirs(mesh_dir, exist_ok=True)
    terrain.save(os.path.join(mesh_dir, 'terrain.stl'))


def link_tree_variants(models_dir):
    """Symlink the vendored tree models into models/tree/<variant>/ so
    WorldPopulator can discover them and Gazebo can resolve model://tree/<variant>."""
    tree_dir = os.path.join(models_dir, 'tree')
    os.makedirs(tree_dir, exist_ok=True)
    variants = [('oak', 'tree_oak'), ('pine', 'tree_pine')]
    variants += [(v, v) for v in DEADTREE_STANDING_VARIANTS]
    for variant, source_name in variants:
        link_path = os.path.join(tree_dir, variant)
        if os.path.islink(link_path) or os.path.exists(link_path):
            continue
        os.symlink(os.path.join(models_dir, source_name), link_path)
    # models/rock/{rock_a,rock_b} are vendored directly (Gazebo Fuel "Falling
    # Rock 1"/"4" meshes, marked static) - no symlinking needed, WorldPopulator
    # discovers them the same way it discovers tree/.


def add_ponds(rng, field, terrain_size, playable_half, count):
    """Carve basins into `field` in place; return [(cx, cy, radius, depth), ...]."""
    ponds = []
    tries = 0
    while len(ponds) < count and tries < count * 60 + 20:
        tries += 1
        px = float(rng.uniform(-playable_half + 7, playable_half - 7))
        py = float(rng.uniform(-playable_half + 7, playable_half - 7))
        pr = float(rng.uniform(3.0, 6.0))
        if math.hypot(px, py) < CLEARING_RADIUS_M + pr + 5:
            continue
        if any(math.hypot(px - ox, py - oy) < pr + orr + 5 for ox, oy, orr, _ in ponds):
            continue
        ponds.append((px, py, pr, float(rng.uniform(1.2, 2.2))))
    for (px, py, pr, pdepth) in ponds:
        field[:] = carve_basin(field, terrain_size, px, py, pr, pdepth)
    return ponds


def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', type=int, default=None,
                     help='omit for a fresh random forest each run')
    ap.add_argument('--name', default='forest_forest3d_enriched')
    ap.add_argument('--preset', default='moderate', choices=sorted(PRESETS))
    ap.add_argument('--size', type=float, default=60.0, help='square terrain size in metres')
    ap.add_argument('--heightmap-samples', type=int, default=129, dest='samples')
    ap.add_argument('--pkg-dir', default=None)
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.seed is None:
        args.seed = int.from_bytes(os.urandom(4), 'little')
    rng = np.random.default_rng(args.seed)
    np.random.seed(args.seed % (2**32 - 1))  # WorldPopulator uses the global np.random state

    pkg_dir = args.pkg_dir or _default_pkg_dir()
    worlds_dir = os.path.join(pkg_dir, 'worlds')
    models_dir = os.path.join(pkg_dir, 'models')
    preset = PRESETS[args.preset]
    playable_half = args.size / 2.0

    # --- terrain: hills + ponds carved in, same shaping as the native generator ---
    field = build_heightfield(rng, args.samples, preset['terrain_amplitude'], preset['terrain_roughness'])
    gx, gy = _world_grid(args.samples, args.size)
    sigma = playable_half * 0.55
    field -= np.exp(-(gx * gx + gy * gy) / (2.0 * sigma * sigma)) * preset['terrain_amplitude'] * 0.55

    ponds = add_ponds(rng, field, args.size, playable_half, int(preset['ponds']))
    field -= float(field.min())
    z_span = max(float(field.max()), 0.05)

    write_terrain_model(models_dir, 'ground', field, args.size, z_span)
    write_terrain_stl(models_dir, field, args.size)
    link_tree_variants(models_dir)

    def ground(x, y):
        return sample_elevation(field, x, y, args.size)

    def near_pond(x, y, pad=1.0):
        return any(math.hypot(x - px, y - py) < pr + pad for px, py, pr, _ in ponds)

    def blocked(x, y):
        return math.hypot(x, y) < CLEARING_RADIUS_M + 1.0 or near_pond(x, y)

    # --- trees + boulders: Forest3D's WorldPopulator (zone-weighted, clustering,
    # cross-category-distance-aware placement; rocks prefer terrain edges) -----
    tree_count = max(1, int(preset['tree_density'] * args.size * args.size))
    rock_count = max(1, int(round(int(preset['rocks']) * 0.65)))  # rest are stump primitives below
    populator = WorldPopulator(base_path=pkg_dir)
    world_path = populator.create_forest_world({'tree': tree_count, 'rock': rock_count})
    stats = populator.get_model_statistics()
    tree_xy = [(x, y) for x, y, _z, _s in populator.placed_models['tree']]

    # --- fallen trees / logs, biased toward standing trees ---------------
    log_xml = []
    for i in range(int(preset['logs'])):
        if tree_xy and rng.random() < 0.72:
            bx, by = tree_xy[int(rng.integers(len(tree_xy)))]
            ang = float(rng.uniform(0.0, 2.0 * math.pi))
            d = float(rng.uniform(1.0, 4.0))
            x, y = bx + d * math.cos(ang), by + d * math.sin(ang)
        else:
            x = float(rng.uniform(-playable_half + 3, playable_half - 3))
            y = float(rng.uniform(-playable_half + 3, playable_half - 3))
        yaw = float(rng.uniform(0.0, 2.0 * math.pi))
        as_tree = rng.random() < 0.80
        span = (rng.uniform(5.5, 9.0) if as_tree else rng.uniform(2.0, 5.5))
        fx, fy = x + span * math.cos(yaw), y + span * math.sin(yaw)
        if any(blocked(x + f * span * math.cos(yaw), y + f * span * math.sin(yaw))
               for f in (0.0, 0.28, 0.55)):
            continue
        base = min(ground(x, y), ground(fx, fy))
        if as_tree and rng.random() < 0.25:
            # already modelled lying down - no tip-over rotation needed
            log_xml.append(emit_include(f'model://{DEADTREE_FALLEN_MODEL}',
                                         f'fallentree_{i:04d}_mossy', x, y, base, yaw))
        elif as_tree:
            sp = 'oak' if rng.random() < 0.5 else 'pine'
            s = sample_tree(rng, sp, simple=False)
            s['_scale'] = float(rng.uniform(0.75, 1.25))
            tip = float(rng.uniform(1.18, 1.57))
            r = s['base_trunk_r'] * s['_scale']
            z = base + r * 0.75
            log_xml.append(emit_fallen_tree(f'fallentree_{i:04d}_{sp}', x, y, z, yaw, tip, s))
        else:
            radius = float(rng.uniform(0.16, 0.36))
            tilt = float(rng.uniform(0.0, 0.32))
            pitch = math.pi / 2.0 - tilt
            z = base + radius * 0.45 + math.sin(tilt) * span * 0.28
            log_xml.append(emit_log(f'log_{i:04d}', x, y, z, yaw, pitch, radius, span,
                                     rng.random() < 0.3))

    # --- stumps: primitive cylinders (no stump mesh exists on Fuel; boulders
    # above are real meshes placed by WorldPopulator's 'rock' category) --------
    rock_xy = [(x, y) for x, y, _z, _s in populator.placed_models['rock']]

    def blocked_for_stump(x, y):
        return blocked(x, y) or any(math.hypot(x - rx, y - ry) < 2.0 for rx, ry in rock_xy)

    stump_count = max(0, int(preset['rocks']) - rock_count)
    stump_pts = scatter_points(rng, playable_half - 1.5, stump_count, 1.4, blocked=blocked_for_stump)
    rock_xml = []
    for i, (x, y) in enumerate(stump_pts):
        base = ground(x, y)
        r = float(rng.uniform(0.25, 0.42))
        h = float(rng.uniform(0.3, 0.65))
        geom = f'<cylinder><radius>{_f(r)}</radius><length>{_f(h)}</length></cylinder>'
        z = base + h / 2.0 - 0.05
        yaw = float(rng.uniform(0.0, 2.0 * math.pi))
        rock_xml.append(emit_blob(f'stump_{i:04d}', x, y, z, yaw, geom, (0.26, 0.18, 0.10, 1.0), collide=True))

    # --- ponds: translucent water disc in each carved basin ----------------
    water_xml = []
    for i, (px, py, pr, pdepth) in enumerate(ponds):
        wz = ground(px, py) + pdepth * 0.5
        water_xml.append(emit_water(f'pond_{i:02d}', px, py, wz, pr * 0.72))

    # --- splice the extra obstacles into Forest3D's world file -------------
    tree_placed = stats['by_category'].get('tree', 0)
    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    for xml_text in log_xml + rock_xml + water_xml:
        world.append(ET.fromstring(xml_text))
    ET.indent(tree, space='    ')
    final_path = os.path.join(worlds_dir, f'{args.name}.sdf')
    tree.write(final_path, encoding='utf-8', xml_declaration=True)
    os.remove(world_path)

    rock_placed = stats['by_category'].get('rock', 0)
    meta_path = os.path.join(worlds_dir, f'{args.name}.meta.json')
    with open(meta_path, 'w') as fh:
        json.dump({
            'seed': args.seed, 'preset': args.preset, 'size': args.size,
            'tree_requested': tree_count, 'tree_placed': tree_placed,
            'boulder_requested': rock_count, 'boulder_placed': rock_placed,
            'fallen_trees_and_logs': len(log_xml), 'stumps': len(rock_xml),
            'ponds': len(water_xml),
        }, fh, indent=2)

    print(f'seed {args.seed} / preset {args.preset}')
    print(f'  {final_path}')
    print(f'  {meta_path}')
    print(f'  trees={tree_placed}/{tree_count} boulders={rock_placed}/{rock_count} '
          f'logs/fallen={len(log_xml)} stumps={len(rock_xml)} ponds={len(water_xml)}')


if __name__ == '__main__':
    main()
