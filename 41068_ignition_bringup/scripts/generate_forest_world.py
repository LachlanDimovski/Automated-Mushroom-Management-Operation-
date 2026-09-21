#!/usr/bin/env python3
"""Procedurally generate a randomised forest world for navigation stress testing.

Writes three artifacts into the package source tree:

  worlds/<name>.sdf            self-contained Gazebo world
  worlds/<name>.meta.json      seed, params, and ground-truth pose of every object
  models/<name>_terrain/       generated heightmap model (heightmap.png + SDF)

The same seed always produces the same world. Terrain is a gently undulating
heightmap; trees are the vendored mesh models (models/tree_oak, models/tree_pine)
plus bare-cylinder "snags"; fallen logs, rocks and low clutter are primitives;
and the existing mushroom / decoy models are scattered as a mission payload.
Fully local - no Fuel downloads, deterministic.

This is deliberately standalone (numpy + Pillow only, no ROS imports) so it can
be run and inspected on its own:

    python3 generate_forest_world.py --seed 7 --preset dense

Nothing here touches the existing worlds. `<name>.sdf` and `models/<name>_terrain/`
are new files referenced only by each other. See worlds/README_forest_random.md.

Design notes worth knowing before editing:
  * Static models ignore gravity, so every object's z is set explicitly from the
    generator's own copy of the heightfield (`sample_elevation`). If terrain and
    objects ever look vertically misaligned in Gazebo, the world<->pixel mapping
    in `sample_elevation` / `write_terrain_model` is the thing to check.
  * The Husky's 2D lidar sits ~0.75 m up. Fallen logs are placed so their top is
    ~0.2-0.35 m: below the lidar, above the depth-camera floor. That is
    intentional - it is the regime the Nav2 (scan-only) costmap cannot see.
  * Trees use a mesh visual but a cheap trunk-cylinder collision - the full mesh
    collision the Fuel models ship is far too heavy for ~200 trees x a 360-beam
    lidar. The lidar only ever sees the trunk anyway.
"""

import argparse
import json
import math
import os
import sys

import numpy as np
from PIL import Image


# --------------------------------------------------------------------------
# Presets: a starting point per stress profile. Any value explicitly passed on
# the command line overrides the preset.
# --------------------------------------------------------------------------
# Tree counts are deliberately conservative: the target machine has Intel UHD
# integrated graphics and the alpha foliage is the dominant GUI render cost.
# Push --tree-density up on a stronger GPU.
PRESETS = {
    'light': dict(
        tree_density=0.008, logs=8, rocks=6, clutter=8, ponds=2,
        terrain_amplitude=2.4, terrain_roughness=0.5, min_gap=2.0),
    'sparse': dict(
        tree_density=0.012, logs=12, rocks=8, clutter=10, ponds=1,
        terrain_amplitude=1.8, terrain_roughness=0.4, min_gap=2.0),
    'moderate': dict(
        tree_density=0.018, logs=26, rocks=14, clutter=18, ponds=2,
        terrain_amplitude=3.0, terrain_roughness=0.5, min_gap=1.4),
    'dense': dict(
        tree_density=0.038, logs=34, rocks=20, clutter=28, ponds=2,
        terrain_amplitude=2.6, terrain_roughness=0.5, min_gap=0.9),
    'blowdown': dict(
        tree_density=0.020, logs=60, rocks=15, clutter=22, ponds=3,
        terrain_amplitude=2.8, terrain_roughness=0.5, min_gap=1.2),
    'hilly': dict(
        tree_density=0.016, logs=18, rocks=22, clutter=18, ponds=4,
        terrain_amplitude=5.2, terrain_roughness=0.7, min_gap=1.6),
}

# Tree species. `oak` / `pine` use the vendored mesh models in models/tree_*/
# (mesh visual, cheap trunk-cylinder collision). `snag` is a bare dead trunk -
# a cheap primitive, and realistic forest clutter. `scale` multiplies the mesh
# and the collision cylinder together for per-tree size variety.
TREE_SPECIES = {
    'oak': dict(
        weight=0.46, model='tree_oak', mesh='meshes/oak_tree.dae',
        branch_tex='materials/textures/branch_diffuse.png',
        bark_tex='materials/textures/bark_diffuse.png',
        base_trunk_r=0.26, base_trunk_h=5.5, z_off=-0.20, scale=(0.75, 1.35),
        canopy='sphere', canopy_r=1.3, canopy_z=1.0, canopy_rgba=(0.17, 0.33, 0.14, 1)),
    'pine': dict(
        weight=0.42, model='tree_pine', mesh='meshes/pine_tree.dae',
        branch_tex='materials/textures/branch_2_diffuse.png',
        bark_tex='materials/textures/bark_diffuse.png',
        base_trunk_r=0.20, base_trunk_h=7.0, z_off=-0.05, scale=(0.8, 1.5),
        canopy='capsule', canopy_r=0.95, canopy_len=4.6, canopy_z=0.78,
        canopy_rgba=(0.14, 0.29, 0.15, 1)),
    'snag': dict(  # bare dead standing trunk - a clean lidar obstacle
        weight=0.12, model=None,
        trunk_r=(0.14, 0.28), trunk_h=(2.5, 6.0),
        trunk_rgba=(0.30, 0.24, 0.18, 1.0)),
}

# Margin the playable heightmap adds around the object area so boundary
# structures still stand on terrain rather than off its edge.
TERRAIN_MARGIN_M = 12.0


# --------------------------------------------------------------------------
# Fractal heightfield
# --------------------------------------------------------------------------
def _bilinear_upsample(grid, out_n):
    """Resample a small (m, m) grid up to (out_n, out_n) bilinearly."""
    m = grid.shape[0]
    coords = np.linspace(0.0, m - 1.0, out_n)
    i0 = np.floor(coords).astype(int)
    i1 = np.minimum(i0 + 1, m - 1)
    frac = coords - i0
    rows = grid[i0] * (1.0 - frac)[:, None] + grid[i1] * frac[:, None]
    return rows[:, i0] * (1.0 - frac)[None, :] + rows[:, i1] * frac[None, :]


def build_heightfield(rng, samples, amplitude, roughness):
    """Sum a few octaves of upsampled random grids -> smooth value noise.

    The low grid_n=2 octave dominates: 3 control points across the terrain give
    one or two broad hills. Returned array is float metres in [0, amplitude],
    row 0 = world +Y.
    """
    octaves = ((2, 1.00), (4, 0.60), (8, 0.32), (16, 0.16), (32, 0.08))
    field = np.zeros((samples, samples), dtype=np.float64)
    total = 0.0
    for grid_n, base_weight in octaves:
        weight = base_weight * (roughness if grid_n >= 8 else 1.0)
        low = rng.random((grid_n + 1, grid_n + 1))
        field += weight * _bilinear_upsample(low, samples)
        total += weight
    field /= total
    field -= field.min()
    peak = field.max()
    if peak > 1e-9:
        field *= amplitude / peak
    return field


def carve_basin(field, terrain_size, cx, cy, radius, depth):
    """Press a smooth cosine bowl into the terrain (for a pond)."""
    gx, gy = _world_grid(field.shape[0], terrain_size)
    t = np.clip(np.hypot(gx - cx, gy - cy) / radius, 0.0, 1.0)
    bowl = 0.5 * (1.0 + np.cos(np.pi * t))     # 1 at centre -> 0 at the rim
    return field - depth * bowl


def _world_grid(samples, terrain_size):
    """Per-cell world (x, y), consistent with `sample_elevation` (row 0 = +Y)."""
    axis = np.linspace(-terrain_size / 2.0, terrain_size / 2.0, samples)
    gx, gy = np.meshgrid(axis, -axis)
    return gx, gy


def sample_elevation(field, x, y, terrain_size):
    """Bilinearly sample terrain height (metres) at world (x, y).

    Gazebo convention assumed: heightmap image column 0 is world x = -size/2,
    row 0 is world y = +size/2 (image y flipped vs world y). `write_terrain_model`
    writes the PNG with the matching orientation.
    """
    n = field.shape[0]
    u = (x + terrain_size / 2.0) / terrain_size
    v = (terrain_size / 2.0 - y) / terrain_size
    fx = min(max(u * (n - 1), 0.0), n - 1.0)
    fy = min(max(v * (n - 1), 0.0), n - 1.0)
    x0, y0 = int(math.floor(fx)), int(math.floor(fy))
    x1, y1 = min(x0 + 1, n - 1), min(y0 + 1, n - 1)
    tx, ty = fx - x0, fy - y0
    top = field[y0, x0] * (1 - tx) + field[y0, x1] * tx
    bot = field[y1, x0] * (1 - tx) + field[y1, x1] * tx
    return float(top * (1 - ty) + bot * ty)


def flatten_disc(field, terrain_size, cx, cy, radius, blend, target=None):
    """Blend a circular region of `field` toward `target` (or its centre height)."""
    gx, gy = _world_grid(field.shape[0], terrain_size)
    dist = np.hypot(gx - cx, gy - cy)
    if target is None:
        target = sample_elevation(field, cx, cy, terrain_size)
    w = np.clip((radius + blend - dist) / max(blend, 1e-6), 0.0, 1.0)
    return field * (1.0 - w) + target * w


def local_slope(field, x, y, terrain_size):
    """Approximate ground gradient magnitude at (x, y)."""
    d = 1.5
    zx = (sample_elevation(field, x + d, y, terrain_size)
          - sample_elevation(field, x - d, y, terrain_size)) / (2 * d)
    zy = (sample_elevation(field, x, y + d, terrain_size)
          - sample_elevation(field, x, y - d, terrain_size)) / (2 * d)
    return math.hypot(zx, zy)


# --------------------------------------------------------------------------
# Lanes: gently curving corridors from the clearing to the boundary. Flattened
# and kept clear of obstacles so there is always at least one drivable route.
# --------------------------------------------------------------------------
def make_lane(rng, clearing_radius, playable_half):
    ang = rng.uniform(0.0, 2.0 * math.pi)
    x = (clearing_radius + 1.0) * math.cos(ang)
    y = (clearing_radius + 1.0) * math.sin(ang)
    heading = ang + float(rng.uniform(-0.4, 0.4))
    pts = [(x, y)]
    for _ in range(200):
        heading += float(rng.uniform(-0.32, 0.32))
        x += 2.0 * math.cos(heading)
        y += 2.0 * math.sin(heading)
        pts.append((x, y))
        if abs(x) > playable_half or abs(y) > playable_half:
            break
    return pts


def point_near_polyline(px, py, polyline, tol):
    for (ax, ay), (bx, by) in zip(polyline, polyline[1:]):
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        t = 0.0 if seg2 == 0 else ((px - ax) * dx + (py - ay) * dy) / seg2
        t = min(max(t, 0.0), 1.0)
        if math.hypot(px - (ax + t * dx), py - (ay + t * dy)) < tol:
            return True
    return False


# --------------------------------------------------------------------------
# Placement
# --------------------------------------------------------------------------
def scatter_points(rng, playable_half, count, min_dist,
                   accept=None, blocked=None, tries_factor=45):
    """Dart-throwing blue-noise scatter in [-half, half]^2.

    `accept(x, y) -> prob` biases density; `blocked(x, y) -> bool` forbids a
    region. Stops at `count` points or after too many consecutive misses.
    """
    cell = max(min_dist, 0.05) / math.sqrt(2)
    grid = {}
    pts = []
    max_tries = int(count * tries_factor) + 50
    for _ in range(max_tries):
        if len(pts) >= count:
            break
        px = float(rng.uniform(-playable_half, playable_half))
        py = float(rng.uniform(-playable_half, playable_half))
        if blocked is not None and blocked(px, py):
            continue
        if accept is not None and rng.random() > accept(px, py):
            continue
        gx, gy = int((px + playable_half) / cell), int((py + playable_half) / cell)
        ok = True
        for ix in range(gx - 2, gx + 3):
            for iy in range(gy - 2, gy + 3):
                q = grid.get((ix, iy))
                if q is not None and math.hypot(q[0] - px, q[1] - py) < min_dist:
                    ok = False
                    break
            if not ok:
                break
        if not ok:
            continue
        grid[(gx, gy)] = (px, py)
        pts.append((px, py))
    return pts


def density_mask(rng, samples, terrain_size):
    """Low-frequency field in [0.15, 1.0] -> clumps and natural clearings."""
    m = build_heightfield(rng, samples, 1.0, roughness=0.35)
    m -= m.min()
    if m.max() > 1e-9:
        m /= m.max()
    return 0.15 + 0.85 * m


# --------------------------------------------------------------------------
# SDF emission
# --------------------------------------------------------------------------
def _f(v):
    return f'{v:.4f}'


def _mat(rgba):
    r, g, b, a = rgba
    return (f'<material><ambient>{_f(r)} {_f(g)} {_f(b)} {_f(a)}</ambient>'
            f'<diffuse>{_f(r)} {_f(g)} {_f(b)} {_f(a)}</diffuse>'
            f'<specular>0.05 0.05 0.05 1</specular></material>')


def _mesh_visual(vis_name, model, mesh, submesh, tex, scale):
    return (
        f'        <visual name="{vis_name}">'
        f'<geometry><mesh><uri>model://{model}/{mesh}</uri>'
        f'<submesh><name>{submesh}</name></submesh>'
        f'<scale>{_f(scale)} {_f(scale)} {_f(scale)}</scale></mesh></geometry>'
        f'<material><diffuse>1 1 1 1</diffuse><pbr><metal>'
        f'<albedo_map>model://{model}/{tex}</albedo_map>'
        f'<metalness>0</metalness><roughness>1</roughness></metal></pbr></material></visual>')


def _tree_visuals(s, sc, h):
    """Visual XML for a live mesh tree. In 'simple' mode the alpha foliage mesh
    (the dominant GUI render cost) is swapped for one opaque primitive canopy;
    the bark trunk stays a real mesh either way."""
    bark = _mesh_visual('bark', s['model'], s['mesh'], 'Bark', s['bark_tex'], sc)
    if not s.get('_simple'):
        return [_mesh_visual('branch', s['model'], s['mesh'], 'Branch', s['branch_tex'], sc), bark]
    cr = s['canopy_r'] * sc
    cz = h * s['canopy_z']
    if s['canopy'] == 'capsule':
        geom = f'<capsule><radius>{_f(cr)}</radius><length>{_f(s["canopy_len"] * sc)}</length></capsule>'
    else:
        geom = f'<sphere><radius>{_f(cr)}</radius></sphere>'
    canopy = (f'        <visual name="canopy"><pose>0 0 {_f(cz)} 0 0 0</pose>'
              f'<geometry>{geom}</geometry>{_mat(s["canopy_rgba"])}</visual>')
    return [bark, canopy]


def emit_tree(name, x, y, z, yaw, s):
    """A tree: mesh visual + trunk-cylinder collision (or a bare cylinder snag)."""
    if s['_kind'] == 'snag':
        r, h = s['_trunk_r'], s['_trunk_h']
        cyl = f'<geometry><cylinder><radius>{_f(r)}</radius><length>{_f(h)}</length></cylinder></geometry>'
        return '\n'.join([
            f'    <model name="{name}">',
            f'      <static>true</static>',
            f'      <pose>{_f(x)} {_f(y)} {_f(z)} 0 0 {_f(yaw)}</pose>',
            f'      <link name="link">',
            f'        <collision name="trunk"><pose>0 0 {_f(h / 2)} 0 0 0</pose>{cyl}</collision>',
            f'        <visual name="trunk_visual"><pose>0 0 {_f(h / 2)} 0 0 0</pose>{cyl}'
            f'{_mat(s["trunk_rgba"])}</visual>',
            f'      </link>',
            f'    </model>',
        ])
    sc = s['_scale']
    r = s['base_trunk_r'] * sc
    h = s['base_trunk_h'] * sc
    return '\n'.join([
        f'    <model name="{name}">',
        f'      <static>true</static>',
        f'      <pose>{_f(x)} {_f(y)} {_f(z)} 0 0 {_f(yaw)}</pose>',
        f'      <link name="link">',
        f'        <collision name="trunk"><pose>0 0 {_f(h / 2)} 0 0 0</pose>'
        f'<geometry><cylinder><radius>{_f(r)}</radius><length>{_f(h)}</length></cylinder></geometry></collision>',
        *_tree_visuals(s, sc, h),
        f'      </link>',
        f'    </model>',
    ])


def emit_fallen_tree(name, x, y, z, yaw, tip, s, root_plate=True):
    """A whole tree tipped onto its side: the tree mesh rotated ~90 deg about Y
    (so the trunk lies along `yaw`), with a matching horizontal trunk cylinder
    for collision. `tip` near pi/2 = flat on the ground; less = leaning / caught.
    """
    sc = s['_scale']
    r = s['base_trunk_r'] * sc
    h = s['base_trunk_h'] * sc
    out = [
        f'    <model name="{name}">',
        f'      <static>true</static>',
        f'      <pose>{_f(x)} {_f(y)} {_f(z)} 0 {_f(-tip)} {_f(yaw)}</pose>',
        f'      <link name="link">',
        f'        <collision name="trunk"><pose>0 0 {_f(h / 2)} 0 0 0</pose>'
        f'<geometry><cylinder><radius>{_f(r)}</radius><length>{_f(h)}</length></cylinder></geometry></collision>',
        *_tree_visuals(s, sc, h),
    ]
    if root_plate:
        # An upturned root disc standing at the base end, perpendicular to the
        # trunk (a thin cylinder in the model's local XY plane).
        rr = min(r * 2.6, 0.9)
        out.append(
            f'        <visual name="rootplate"><pose>0 0 {_f(rr * 0.35)} 0 0 0</pose>'
            f'<geometry><cylinder><radius>{_f(rr)}</radius><length>{_f(r * 0.6)}</length>'
            f'</cylinder></geometry>{_mat((0.19, 0.14, 0.09, 1.0))}</visual>')
    out += ['      </link>', '    </model>']
    return '\n'.join(out)


def sample_tree(rng, spec_name, simple=False):
    """Resolve a species dict + random per-tree size into a ready-to-emit spec."""
    s = dict(TREE_SPECIES[spec_name])
    s['_kind'] = spec_name if spec_name == 'snag' else 'mesh'
    s['_simple'] = simple
    if spec_name == 'snag':
        s['_trunk_r'] = float(rng.uniform(*s['trunk_r']))
        s['_trunk_h'] = float(rng.uniform(*s['trunk_h']))
    else:
        s['_scale'] = float(rng.uniform(*s['scale']))
    return s


def emit_log(name, x, y, z, yaw, pitch, radius, length, root_ball):
    """Fallen log: a Z-axis cylinder pitched ~90 deg to lie (near) horizontal."""
    out = [
        f'    <model name="{name}">',
        f'      <static>true</static>',
        f'      <pose>{_f(x)} {_f(y)} {_f(z)} 0 {_f(pitch)} {_f(yaw)}</pose>',
        f'      <link name="link">',
        f'        <collision name="log">',
        f'          <geometry><cylinder><radius>{_f(radius)}</radius><length>{_f(length)}</length></cylinder></geometry>',
        f'        </collision>',
        f'        <visual name="log_visual">',
        f'          <geometry><cylinder><radius>{_f(radius)}</radius><length>{_f(length)}</length></cylinder></geometry>',
        f'          {_mat((0.24, 0.16, 0.09, 1.0))}',
        f'        </visual>',
    ]
    if root_ball:
        rb = radius * 2.4
        out += [
            f'        <visual name="root_ball">',
            f'          <pose>0 0 {_f(length / 2)} 0 0 0</pose>',
            f'          <geometry><sphere><radius>{_f(rb)}</radius></sphere></geometry>',
            f'          {_mat((0.18, 0.13, 0.08, 1.0))}',
            f'        </visual>',
        ]
    out += ['      </link>', '    </model>']
    return '\n'.join(out)


def emit_blob(name, x, y, z, yaw, geom_xml, rgba, collide=True):
    out = [
        f'    <model name="{name}">',
        f'      <static>true</static>',
        f'      <pose>{_f(x)} {_f(y)} {_f(z)} 0 0 {_f(yaw)}</pose>',
        f'      <link name="link">',
    ]
    if collide:
        out += [
            f'        <collision name="collision"><geometry>{geom_xml}</geometry></collision>',
        ]
    out += [
        f'        <visual name="visual"><geometry>{geom_xml}</geometry>{_mat(rgba)}</visual>',
        f'      </link>',
        f'    </model>',
    ]
    return '\n'.join(out)


def emit_include(uri, name, x, y, z, yaw=0.0):
    return (f'    <include>\n'
            f'      <uri>{uri}</uri>\n'
            f'      <name>{name}</name>\n'
            f'      <pose>{_f(x)} {_f(y)} {_f(z)} 0 0 {_f(yaw)}</pose>\n'
            f'    </include>')


def emit_water(name, x, y, z, radius):
    """A still pond surface: a thin translucent disc sitting in a carved basin.

    Visual only - the basin's terrain walls are what actually contain the robot.
    """
    return '\n'.join([
        f'    <model name="{name}">',
        f'      <static>true</static>',
        f'      <pose>{_f(x)} {_f(y)} {_f(z)} 0 0 0</pose>',
        f'      <link name="link">',
        f'        <visual name="surface">',
        f'          <transparency>0.25</transparency>',
        f'          <geometry><cylinder><radius>{_f(radius)}</radius><length>0.06</length></cylinder></geometry>',
        f'          <material>'
        f'<ambient>0.10 0.19 0.27 0.75</ambient>'
        f'<diffuse>0.13 0.28 0.40 0.75</diffuse>'
        f'<specular>0.6 0.7 0.8 1</specular>'
        f'<pbr><metal><metalness>0.0</metalness><roughness>0.08</roughness></metal></pbr>'
        f'</material>',
        f'        </visual>',
        f'      </link>',
        f'    </model>',
    ])


WORLD_HEADER = """<?xml version="1.0"?>
<!-- GENERATED by scripts/generate_forest_world.py (seed {seed}, preset {preset}).
     Do not hand-edit; regenerate instead. Full params in {world_name}.meta.json -->
<sdf version="1.8">
  <world name="{world_name}">
    <!-- Shadows off, matching large_demo.sdf: ogre2's directional shadow pass
         stipples large surfaces like the heightmap. -->
    <light name="sun" type="directional">
      <cast_shadows>0</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
      <intensity>1.0</intensity>
    </light>
    <gravity>0 0 -9.81</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <physics name="default_physics" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.7 0.7 0.7 1</ambient>
      <background>0.6 0.75 0.9 1</background>
      <shadows>0</shadows>
      <grid>0</grid>
    </scene>
    <spherical_coordinates>
      <latitude_deg>-33.8688</latitude_deg>
      <longitude_deg>151.2093</longitude_deg>
      <elevation>10.0</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>

    <include>
      <uri>model://{terrain_name}</uri>
      <name>{terrain_name}</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
"""

WORLD_FOOTER = """  </world>
</sdf>
"""


# --------------------------------------------------------------------------
# Terrain model
# --------------------------------------------------------------------------
def terrain_model_config(terrain_name):
    return f"""<?xml version="1.0"?>
<model>
  <name>{terrain_name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>
    Generated undulating terrain. Written by
    scripts/generate_forest_world.py - do not hand-edit; regenerate instead.
  </description>
</model>
"""


def terrain_model_sdf(terrain_name, terrain_size, z_span):
    size = _f(terrain_size)
    zs = max(z_span, 0.05)
    hm = (f'<heightmap>'
          f'<uri>model://{terrain_name}/heightmap.png</uri>'
          f'<size>{size} {size} {_f(zs)}</size>'
          f'<pos>0 0 0</pos>'
          f'<sampling>2</sampling>')
    # A single elevation-coloured texture mapped 1:1 across the terrain (size ==
    # terrain size, so it never tiles). One <texture>, no <blend> - the multi-
    # texture blend splat was what stippled the ground on integrated graphics.
    tex = (f'            <texture>\n'
           f'              <diffuse>model://{terrain_name}/terrain_diffuse.png</diffuse>\n'
           f'              <normal>model://{terrain_name}/flat_normal.png</normal>\n'
           f'              <size>{size}</size>\n'
           f'            </texture>\n')
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{terrain_name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          {hm}</heightmap>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          {hm}
{tex}          </heightmap>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


def _write_terrain_diffuse(tdir, norm, out_size=768):
    """Bake an elevation colour ramp (green valley -> brown slope -> dry hilltop)
    into one image mapped 1:1 across the terrain. `norm` is the height field in
    [0, 1], row 0 = world +Y (matches heightmap.png)."""
    from PIL import ImageFilter
    low = np.array([90, 140, 66], float)     # valley green
    mid = np.array([122, 96, 60], float)     # mid-slope brown
    high = np.array([172, 158, 132], float)  # dry pale hilltop
    n = norm[..., None]
    t1 = np.clip((n - 0.10) / 0.42, 0.0, 1.0)
    t2 = np.clip((n - 0.62) / 0.28, 0.0, 1.0)
    col = low * (1 - t1) + mid * t1
    col = col * (1 - t2) + high * t2
    im = Image.fromarray(np.clip(col, 0, 255).astype(np.uint8)).resize(
        (out_size, out_size), Image.BILINEAR).filter(ImageFilter.GaussianBlur(1.2))
    a = np.asarray(im).astype(float)
    # a little organic mottling so it is not a flat wash
    rng = np.random.default_rng(0)
    speck = np.asarray(Image.fromarray(
        (rng.random((out_size // 6, out_size // 6)) * 255).astype(np.uint8)
    ).resize((out_size, out_size), Image.BILINEAR).filter(ImageFilter.GaussianBlur(2)))
    a *= (0.90 + 0.16 * (speck[..., None] / 255.0))
    Image.fromarray(np.clip(a, 0, 255).astype(np.uint8)).save(
        os.path.join(tdir, 'terrain_diffuse.png'))


def write_terrain_model(models_dir, terrain_name, field, terrain_size, z_span):
    tdir = os.path.join(models_dir, terrain_name)
    os.makedirs(tdir, exist_ok=True)
    zs = max(z_span, 0.05)
    # 8-bit grayscale: gz-common's heightmap loader is only reliable at 8-bit
    # (16-bit PNGs get down-converted). At these amplitudes the ~15 mm vertical
    # quantisation is negligible and the field is already smooth. Row 0 of
    # `field` is world +Y, matching sample_elevation; Gazebo maps pixel 0 -> 0 m
    # and 255 -> <size> z. `field` is pre-shifted to [0, z_span] so object z
    # values line up with the rendered surface.
    norm = np.clip(field / zs, 0.0, 1.0)
    img = Image.fromarray(np.round(norm * 255.0).astype(np.uint8), mode='L')
    img.save(os.path.join(tdir, 'heightmap.png'))
    # Flat tangent-space normal map (all normals point straight up = 128,128,255).
    Image.new('RGB', (64, 64), (128, 128, 255)).save(os.path.join(tdir, 'flat_normal.png'))
    _write_terrain_diffuse(tdir, norm)
    with open(os.path.join(tdir, 'model.config'), 'w') as fh:
        fh.write(terrain_model_config(terrain_name))
    with open(os.path.join(tdir, 'model.sdf'), 'w') as fh:
        fh.write(terrain_model_sdf(terrain_name, terrain_size, zs))


# --------------------------------------------------------------------------
# Generation
# --------------------------------------------------------------------------
def resolve_params(args):
    p = dict(PRESETS[args.preset])
    for key in ('tree_density', 'logs', 'rocks', 'clutter', 'ponds',
                'terrain_amplitude', 'terrain_roughness', 'min_gap'):
        val = getattr(args, key)
        if val is not None:
            p[key] = val
    return p


def generate(args):
    rng = np.random.default_rng(args.seed)
    p = resolve_params(args)
    # Default to simple (opaque) foliage - the alpha foliage mesh is the dominant
    # GUI render cost and the target machine is on integrated graphics. Pass
    # --foliage mesh for photoreal foliage on a discrete GPU.
    simple = args.foliage != 'mesh'

    playable = float(args.size)
    playable_half = playable / 2.0
    terrain_size = playable + 2.0 * TERRAIN_MARGIN_M
    samples = int(args.heightmap_samples)
    amplitude = float(p['terrain_amplitude'])
    clearing_r = float(args.clearing_radius)

    lanes = [make_lane(rng, clearing_r, playable_half) for _ in range(int(args.lanes))]

    def _on_lane_raw(x, y, tol):
        return any(point_near_polyline(x, y, ln, tol) for ln in lanes)

    # --- terrain: hills, a spawn hollow, pond basins, then grade the route ---
    field = build_heightfield(rng, samples, amplitude, float(p['terrain_roughness']))

    # A broad, soft depression toward the spawn area so the robot starts in a
    # natural hollow with hills rising around it - no hard-edged crater.
    gx, gy = _world_grid(samples, terrain_size)
    sigma = playable_half * 0.55
    field -= np.exp(-(gx * gx + gy * gy) / (2.0 * sigma * sigma)) * amplitude * 0.55

    ponds = []   # (cx, cy, basin_radius, depth)
    tries = 0
    while len(ponds) < int(p['ponds']) and tries < int(p['ponds']) * 60 + 20:
        tries += 1
        px = float(rng.uniform(-playable_half + 7, playable_half - 7))
        py = float(rng.uniform(-playable_half + 7, playable_half - 7))
        pr = float(rng.uniform(3.0, 6.0))
        if math.hypot(px, py) < clearing_r + pr + 5:
            continue
        if _on_lane_raw(px, py, pr + 3.0):
            continue
        if any(math.hypot(px - ox, py - oy) < pr + orr + 5 for ox, oy, orr, _ in ponds):
            continue
        ponds.append((px, py, pr, float(rng.uniform(1.2, 2.2))))
    for (px, py, pr, pdepth) in ponds:
        field = carve_basin(field, terrain_size, px, py, pr, pdepth)

    # Grade the lanes so a trail follows the hills instead of cutting a trench.
    for lane in lanes:
        for (lx, ly) in lane:
            field = flatten_disc(field, terrain_size, lx, ly, 1.8, blend=3.5)

    # Vertical datum: lift so the deepest point (a pond bottom) sits at z=0.
    field -= float(field.min())

    # A small flat pad at the exact spawn point (the hollow already did the work).
    pad_target = min(sample_elevation(field, 0.0, 0.0, terrain_size), 0.2)
    field = flatten_disc(field, terrain_size, 0.0, 0.0, clearing_r,
                         blend=4.0, target=pad_target)

    z_span = max(float(field.max()), 0.05)

    def ground(x, y):
        return sample_elevation(field, x, y, terrain_size)

    # --- placement predicates ---------------------------------------------
    lane_tol = 2.2

    def in_clearing(x, y, pad=0.0):
        return math.hypot(x, y) < clearing_r + pad

    def on_lane(x, y, tol=lane_tol):
        return _on_lane_raw(x, y, tol)

    def near_pond(x, y, pad=1.0):
        return any(math.hypot(x - px, y - py) < pr + pad for px, py, pr, _ in ponds)

    def blocked_core(x, y):
        return in_clearing(x, y, pad=1.0) or on_lane(x, y) or near_pond(x, y)

    dmask = density_mask(rng, samples, terrain_size)

    def tree_accept(x, y):
        return sample_elevation(dmask, x, y, terrain_size)

    min_gap = float(p['min_gap'])
    objects = []          # ground-truth records
    tree_xy = []          # for biasing logs toward trees

    # --- procedural trees -----------------------------------------------
    tree_count = int(round(float(p['tree_density']) * playable * playable))
    tree_min_dist = max(min_gap + 0.4, 1.0)
    tree_pts = scatter_points(
        rng, playable_half - 1.0, tree_count, tree_min_dist,
        accept=tree_accept, blocked=blocked_core)

    names = list(TREE_SPECIES)
    weights = np.array([TREE_SPECIES[n]['weight'] for n in names])
    weights = weights / weights.sum()

    tree_models = []
    for i, (x, y) in enumerate(tree_pts):
        spec_name = names[int(rng.choice(len(names), p=weights))]
        s = sample_tree(rng, spec_name, simple)
        yaw = float(rng.uniform(0.0, 2.0 * math.pi))
        # On a slope a vertical trunk floats on the uphill side - sink it in.
        slope_sink = min(local_slope(field, x, y, terrain_size) * 1.6, 0.7)
        if spec_name == 'snag':
            trunk_r, trunk_h = s['_trunk_r'], s['_trunk_h']
            z = ground(x, y) - 0.10 - slope_sink
        else:
            trunk_r = s['base_trunk_r'] * s['_scale']
            trunk_h = s['base_trunk_h'] * s['_scale']
            z = ground(x, y) + s['z_off'] - slope_sink
        name = f'tree_{i:04d}_{spec_name}'
        tree_models.append(emit_tree(name, x, y, z, yaw, s))
        tree_xy.append((x, y))
        objects.append(dict(name=name, type='tree', species=spec_name,
                            x=round(x, 3), y=round(y, 3), z=round(z, 3),
                            yaw=round(yaw, 3), radius=round(trunk_r, 3),
                            height=round(trunk_h, 3)))

    # --- fallen trees + broken logs ---------------------------------
    # Most are whole trees on their side (mesh); the rest are broken trunk
    # sections. Both bias toward standing trees ("that one fell").
    log_models = []
    for i in range(int(p['logs'])):
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
        # Reject only if the trunk (base .. ~half way) lands on the
        # clearing/lane/pond; a crown may overhang.
        if any(blocked_core(x + f * span * math.cos(yaw), y + f * span * math.sin(yaw))
               for f in (0.0, 0.28, 0.55)):
            continue
        # Sit the trunk at the lower of its two ends so it beds into a slope
        # instead of floating off the downhill end.
        base = min(ground(x, y), ground(fx, fy))
        if as_tree:
            sp = 'oak' if rng.random() < 0.5 else 'pine'
            s = sample_tree(rng, sp, simple)
            s['_scale'] = float(rng.uniform(0.75, 1.25))
            tip = float(rng.uniform(1.18, 1.57))       # near-flat .. fully flat
            r = s['base_trunk_r'] * s['_scale']
            z = base + r * 0.75                          # trunk clears the grass
            name = f'fallentree_{i:04d}_{sp}'
            log_models.append(emit_fallen_tree(name, x, y, z, yaw, tip, s))
            objects.append(dict(name=name, type='fallen_tree', species=sp,
                                x=round(x, 3), y=round(y, 3), z=round(z, 3),
                                yaw=round(yaw, 3), radius=round(r, 3),
                                length=round(s['base_trunk_h'] * s['_scale'], 3),
                                tip=round(tip, 3)))
        else:
            radius = float(rng.uniform(0.16, 0.36))
            tilt = float(rng.uniform(0.0, 0.32))
            pitch = math.pi / 2.0 - tilt
            z = base + radius * 0.45 + math.sin(tilt) * span * 0.28
            name = f'log_{i:04d}'
            log_models.append(emit_log(name, x, y, z, yaw, pitch, radius, span,
                                       rng.random() < 0.3))
            objects.append(dict(name=name, type='log', x=round(x, 3), y=round(y, 3),
                                z=round(z, 3), yaw=round(yaw, 3),
                                radius=round(radius, 3), length=round(span, 3)))

    # --- rocks / stumps ----------------------------------------------
    blob_models = []
    rock_pts = scatter_points(rng, playable_half - 1.5, int(p['rocks']),
                              max(min_gap, 0.8), blocked=blocked_core)
    for i, (x, y) in enumerate(rock_pts):
        base = ground(x, y)
        if rng.random() < 0.35:                        # stump
            r = float(rng.uniform(0.25, 0.42))
            h = float(rng.uniform(0.3, 0.65))
            geom = f'<cylinder><radius>{_f(r)}</radius><length>{_f(h)}</length></cylinder>'
            z = base + h / 2.0 - 0.05
            rgba = (0.26, 0.18, 0.10, 1.0)
            kind, extent = 'stump', r
        else:                                          # boulder
            r = float(rng.uniform(0.25, 0.6))
            geom = f'<sphere><radius>{_f(r)}</radius></sphere>'
            z = base + r - float(rng.uniform(0.30, 0.55)) * r
            rgba = (0.42, 0.42, 0.44, 1.0)
            kind, extent = 'rock', r
        yaw = float(rng.uniform(0.0, 2.0 * math.pi))
        name = f'{kind}_{i:04d}'
        blob_models.append(emit_blob(name, x, y, z, yaw, geom, rgba, collide=True))
        objects.append(dict(name=name, type=kind, x=round(x, 3), y=round(y, 3),
                            z=round(z, 3), yaw=round(yaw, 3), radius=round(extent, 3)))

    # --- low clutter (mostly below the lidar beam) --------------------
    clutter_pts = scatter_points(rng, playable_half - 1.0, int(p['clutter']),
                                 max(min_gap * 0.6, 0.5), blocked=blocked_core)
    for i, (x, y) in enumerate(clutter_pts):
        base = ground(x, y)
        r = float(rng.uniform(0.18, 0.45))
        geom = f'<sphere><radius>{_f(r)}</radius></sphere>'
        z = base + r * float(rng.uniform(0.15, 0.5))   # squashed into the ground
        yaw = float(rng.uniform(0.0, 2.0 * math.pi))
        name = f'clutter_{i:04d}'
        blob_models.append(emit_blob(name, x, y, z, yaw, geom,
                                     (0.20, 0.30, 0.14, 1.0), collide=True))
        objects.append(dict(name=name, type='clutter', x=round(x, 3), y=round(y, 3),
                            z=round(z, 3), yaw=round(yaw, 3), radius=round(r, 3)))

    # --- boundary ----------------------------------------------------
    boundary_models = []
    if args.boundary == 'trees':
        ring_n = int(2 * math.pi * playable_half / 6.0)   # sparse ring - big pines
        for i in range(ring_n):
            a = 2.0 * math.pi * i / ring_n
            rr = playable_half + float(rng.uniform(0.5, 4.0))
            x, y = rr * math.cos(a), rr * math.sin(a)
            s = sample_tree(rng, 'pine', simple)
            s['_scale'] = float(rng.uniform(1.5, 2.1))   # tall enough to read as an edge
            z = (ground(x, y) + s['z_off']
                 - min(local_slope(field, x, y, terrain_size) * 1.6, 0.8))
            boundary_models.append(
                emit_tree(f'edge_{i:04d}', x, y, z, float(rng.uniform(0, 6.28)), s))
    elif args.boundary == 'wall':
        seg = 9.32
        for i, (sx, sy, syaw) in enumerate(_wall_ring(playable_half + 1.0, seg)):
            boundary_models.append(emit_include('model://forest_wall',
                                                f'wall_{i:02d}', sx, sy, 2.0, syaw))

    # --- mission payload: mushrooms + decoys ------------------------
    mission_models = []
    mushrooms, decoys = [], []
    mush_pts = scatter_points(rng, playable_half - 3.0, int(args.mushrooms), 3.0,
                              blocked=lambda x, y: in_clearing(x, y, 1.0))
    for i, (x, y) in enumerate(mush_pts):
        z = ground(x, y)
        name = f'mushroom_{i:02d}'
        mission_models.append(emit_include('model://mushroom_amanita', name, x, y, z))
        rec = dict(name=name, x=round(x, 3), y=round(y, 3), z=round(z, 3))
        mushrooms.append(rec)
        objects.append(dict(type='mushroom', **rec))

    decoy_pts = scatter_points(rng, playable_half - 3.0, int(args.decoys), 3.0,
                               blocked=lambda x, y: in_clearing(x, y, 1.0))
    for i, (x, y) in enumerate(decoy_pts):
        model = 'decoy_box' if rng.random() < 0.5 else 'decoy_cylinder'
        z = ground(x, y) + 0.3     # decoy origins are centred, not base-aligned
        name = f'decoy_{i:02d}'
        mission_models.append(emit_include(f'model://{model}', name, x, y, z))
        rec = dict(name=name, model=model, x=round(x, 3), y=round(y, 3), z=round(z, 3))
        decoys.append(rec)
        objects.append(dict(type='decoy', **rec))

    # --- ponds: a translucent water disc in each carved basin -----------
    water_models = []
    pond_recs = []
    for i, (px, py, pr, pdepth) in enumerate(ponds):
        wz = ground(px, py) + pdepth * 0.5
        wr = pr * 0.72
        name = f'pond_{i:02d}'
        water_models.append(emit_water(name, px, py, wz, wr))
        rec = dict(name=name, x=round(px, 3), y=round(py, 3), z=round(wz, 3),
                   radius=round(wr, 3), basin_radius=round(pr, 3),
                   depth=round(pdepth, 3))
        pond_recs.append(rec)
        objects.append(dict(type='pond', **rec))

    # --- write terrain + world + meta -------------------------------
    terrain_name = f'{args.name}_terrain'
    write_terrain_model(args.models_dir, terrain_name, field, terrain_size, z_span)

    chunks = [WORLD_HEADER.format(seed=args.seed, preset=args.preset,
                                  world_name=args.name, terrain_name=terrain_name)]
    for title, group in (('trees', tree_models),
                         ('fallen logs', log_models),
                         ('rocks, stumps and clutter', blob_models),
                         ('ponds', water_models),
                         ('boundary', boundary_models),
                         ('mission payload', mission_models)):
        if group:
            chunks.append(f'\n    <!-- {title} ({len(group)}) -->\n')
            chunks.append('\n'.join(group))
            chunks.append('\n')
    chunks.append(WORLD_FOOTER)
    world_sdf = ''.join(chunks)

    world_path = os.path.join(args.worlds_dir, f'{args.name}.sdf')
    with open(world_path, 'w') as fh:
        fh.write(world_sdf)

    meta = dict(
        seed=args.seed,
        preset=args.preset,
        params=dict(p, size=playable, heightmap_samples=samples,
                    clearing_radius=clearing_r, lanes=int(args.lanes),
                    boundary=args.boundary, mushrooms=int(args.mushrooms),
                    decoys=int(args.decoys),
                    foliage='simple' if simple else 'mesh'),
        world=dict(name=args.name, playable_size_m=playable,
                   terrain_size_m=terrain_size),
        terrain=dict(model=terrain_name,
                     heightmap=f'models/{terrain_name}/heightmap.png',
                     samples=samples, requested_amplitude_m=round(amplitude, 3),
                     z_span_m=round(z_span, 3)),
        spawn_clearing=dict(center=[0.0, 0.0], radius=clearing_r,
                            ground_z=round(ground(0.0, 0.0), 3)),
        lanes=[[[round(x, 2), round(y, 2)] for (x, y) in ln] for ln in lanes],
        counts=dict(trees=len(tree_models),
                    fallen_trees=sum(o['type'] == 'fallen_tree' for o in objects),
                    logs=sum(o['type'] == 'log' for o in objects),
                    rocks_stumps_clutter=len(blob_models),
                    ponds=len(water_models), boundary=len(boundary_models),
                    mushrooms=len(mushrooms), decoys=len(decoys)),
        mushrooms=mushrooms,
        decoys=decoys,
        ponds=pond_recs,
        objects=objects,
    )
    meta_path = os.path.join(args.worlds_dir, f'{args.name}.meta.json')
    with open(meta_path, 'w') as fh:
        json.dump(meta, fh, indent=2)

    return world_path, meta_path, meta


def _wall_ring(half, seg):
    out = []
    for k in (-1, 0, 1):
        out.append((half, k * seg, math.pi / 2))
        out.append((-half, k * seg, math.pi / 2))
        out.append((k * seg, half, 0.0))
        out.append((k * seg, -half, 0.0))
    return out


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def _default_pkg_dir():
    # scripts/ -> package root
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def parse_args(argv):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', type=int, default=None,
                    help='omit for a fresh random forest each run; pass a value to reproduce one')
    ap.add_argument('--name', default='forest_random',
                    help='output world name (worlds/<name>.sdf)')
    ap.add_argument('--preset', default='moderate', choices=sorted(PRESETS))
    ap.add_argument('--size', type=float, default=60.0,
                    help='square playable area in metres')
    ap.add_argument('--tree-density', type=float, default=None,
                    dest='tree_density', help='trees per m^2')
    ap.add_argument('--logs', type=int, default=None)
    ap.add_argument('--rocks', type=int, default=None)
    ap.add_argument('--clutter', type=int, default=None)
    ap.add_argument('--ponds', type=int, default=None,
                    help='number of carved water basins')
    ap.add_argument('--terrain-amplitude', type=float, default=None,
                    dest='terrain_amplitude', help='hill height in metres (before basins)')
    ap.add_argument('--terrain-roughness', type=float, default=None,
                    dest='terrain_roughness')
    ap.add_argument('--heightmap-samples', type=int, default=129,
                    dest='heightmap_samples', help='(2^n)+1, e.g. 65 / 129 / 257')
    ap.add_argument('--clearing-radius', type=float, default=4.0,
                    dest='clearing_radius')
    ap.add_argument('--lanes', type=int, default=1)
    ap.add_argument('--min-gap', type=float, default=None, dest='min_gap',
                    help='minimum spacing between obstacle centres (m)')
    ap.add_argument('--mushrooms', type=int, default=5)
    ap.add_argument('--decoys', type=int, default=4)
    ap.add_argument('--boundary', default='trees', choices=('trees', 'wall', 'none'))
    ap.add_argument('--foliage', default='simple', choices=('mesh', 'simple'),
                    help="default 'simple' = opaque canopy shape, light on the GPU. "
                         "'mesh' = photoreal alpha foliage, needs a discrete GPU.")
    ap.add_argument('--pkg-dir', default=None, dest='pkg_dir',
                    help='package root to write into (default: this checkout)')
    args = ap.parse_args(argv)

    pkg = args.pkg_dir or _default_pkg_dir()
    args.worlds_dir = os.path.join(pkg, 'worlds')
    args.models_dir = os.path.join(pkg, 'models')
    if not os.path.isdir(args.worlds_dir) or not os.path.isdir(args.models_dir):
        ap.error(f'expected worlds/ and models/ under {pkg} - pass --pkg-dir')
    if args.heightmap_samples < 9 or (args.heightmap_samples - 1) & (args.heightmap_samples - 2) != 0:
        ap.error('--heightmap-samples must be (2^n)+1 (e.g. 65, 129, 257)')
    if args.seed is None:
        args.seed = int.from_bytes(os.urandom(4), 'little')
    return args


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    world_path, meta_path, meta = generate(args)
    c = meta['counts']
    print(f'seed {args.seed} / preset {args.preset}')
    print(f'  {world_path}')
    print(f'  {meta_path}')
    print(f'  trees={c["trees"]} fallen_trees={c["fallen_trees"]} logs={c["logs"]} '
          f'rocks/stumps/clutter={c["rocks_stumps_clutter"]} ponds={c["ponds"]} '
          f'boundary={c["boundary"]} '
          f'mushrooms={c["mushrooms"]} decoys={c["decoys"]}')
    print(f'  terrain z-span={meta["terrain"]["z_span_m"]} m, '
          f'clearing ground z={meta["spawn_clearing"]["ground_z"]} m')


if __name__ == '__main__':
    main()
