#!/usr/bin/env python3
"""Randomised forest world generator built on the Forest3D population engine
(third_party/forest3d submodule, github.com/LachlanDimovski/Forest3D).

Reuses this package's own terrain heightfield + heightmap-model writer
(generate_forest_world.py) for the ground, then hands tree placement to
Forest3D's WorldPopulator (zone-weighted, cross-category-distance-aware,
clustering placement) instead of this package's scatter_points().

Only the `tree` category is populated: rock/bush/grass/sand have no vendored
assets in models/ yet. Writes:

  worlds/<name>.sdf            Forest3D's world (renamed from forest_world.world)
  worlds/<name>.meta.json       seed + tree count, for reproducibility
  models/ground/                heightmap terrain model (shared writer)
  models/ground/mesh/terrain.stl  STL copy for WorldPopulator's height sampling
  models/tree/oak, models/tree/pine  symlinks to the vendored tree_oak/tree_pine models

Usage:
    python3 generate_forest_world_forest3d.py --seed 7 --preset dense
"""

import argparse
import json
import os
import sys

import numpy as np
from stl import mesh as stl_mesh

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'third_party', 'forest3d', 'src'))
from forest3d.core.forest import WorldPopulator  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
from generate_forest_world import (  # noqa: E402
    PRESETS, build_heightfield, write_terrain_model, _world_grid, _default_pkg_dir,
)


def write_terrain_stl(models_dir, field, terrain_size):
    """STL matching write_terrain_model's world mapping, for WorldPopulator's
    barycentric height sampling (it reads models/ground/mesh/terrain.stl directly,
    separate from the heightmap.png Gazebo renders)."""
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


def link_tree_variants(pkg_models_dir, project_models_dir):
    """Symlink the vendored tree models into models/tree/<variant>/ so
    WorldPopulator can discover them and Gazebo can resolve model://tree/<variant>."""
    tree_dir = os.path.join(project_models_dir, 'tree')
    os.makedirs(tree_dir, exist_ok=True)
    for variant, source_name in (('oak', 'tree_oak'), ('pine', 'tree_pine')):
        link_path = os.path.join(tree_dir, variant)
        if os.path.islink(link_path) or os.path.exists(link_path):
            continue
        os.symlink(os.path.join(pkg_models_dir, source_name), link_path)


def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', type=int, default=None,
                     help='omit for a fresh random forest each run')
    ap.add_argument('--name', default='forest_forest3d')
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
    field = build_heightfield(rng, args.samples, preset['terrain_amplitude'], preset['terrain_roughness'])
    field -= field.min()
    z_span = float(field.max())

    write_terrain_model(models_dir, 'ground', field, args.size, z_span)
    write_terrain_stl(models_dir, field, args.size)
    link_tree_variants(models_dir, models_dir)

    tree_count = max(1, int(preset['tree_density'] * args.size * args.size))

    populator = WorldPopulator(base_path=pkg_dir)
    world_path = populator.create_forest_world({'tree': tree_count})
    stats = populator.get_model_statistics()

    final_path = os.path.join(worlds_dir, f'{args.name}.sdf')
    os.replace(world_path, final_path)

    meta_path = os.path.join(worlds_dir, f'{args.name}.meta.json')
    with open(meta_path, 'w') as fh:
        json.dump({'seed': args.seed, 'preset': args.preset, 'size': args.size,
                    'tree_requested': tree_count,
                    'tree_placed': stats['by_category'].get('tree', 0)}, fh, indent=2)

    print(f'seed {args.seed} / preset {args.preset}')
    print(f'  {final_path}')
    print(f'  {meta_path}')
    print(f"  trees placed {stats['by_category'].get('tree', 0)}/{tree_count}")


if __name__ == '__main__':
    main()
