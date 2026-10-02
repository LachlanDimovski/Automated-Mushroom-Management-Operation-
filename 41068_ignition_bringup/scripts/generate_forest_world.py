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


def _rpy(roll, pitch, yaw):
    """SDF rotation Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _box(sx, sy, sz):
    v = np.array([[x, y, z] for x in (-sx / 2, sx / 2) for y in (-sy / 2, sy / 2) for z in (-sz / 2, sz / 2)])
    f = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6),
         (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return v, np.array(f)


def _prism(r, length, n=6):
    """Open n-sided tube along z (a stem: the ends are never seen)."""
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    v = np.array([[r * math.cos(t), r * math.sin(t), z] for z in (-length / 2, length / 2) for t in a])
    f = [t for i in range(n) for t in ((i, (i + 1) % n, n + (i + 1) % n), (i, n + (i + 1) % n, n + i))]
    return v, np.array(f)


def _sphere(r, nu=8, nv=5):
    v = [[0, 0, r]] + [[r * math.sin(math.pi * j / nv) * math.cos(2 * math.pi * i / nu),
                        r * math.sin(math.pi * j / nv) * math.sin(2 * math.pi * i / nu),
                        r * math.cos(math.pi * j / nv)] for j in range(1, nv) for i in range(nu)] + [[0, 0, -r]]
    ring = lambda j, i: 1 + (j - 1) * nu + i % nu
    f = [(0, ring(1, i), ring(1, i + 1)) for i in range(nu)]
    f += [t for j in range(1, nv - 1) for i in range(nu)
          for t in ((ring(j, i), ring(j + 1, i), ring(j + 1, i + 1)), (ring(j, i), ring(j + 1, i + 1), ring(j, i + 1)))]
    f += [(len(v) - 1, ring(nv - 1, i + 1), ring(nv - 1, i)) for i in range(nu)]
    return np.array(v, float), np.array(f)


def _disc(r, n=10):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    v = np.array([[0, 0, 0]] + [[r * math.cos(t), r * math.sin(t), 0] for t in a])
    return v, np.array([(0, 1 + i, 1 + (i + 1) % n) for i in range(n)])


class _Obj:
    """Many small parts merged into one OBJ, one group per material: a few draw calls instead of thousands
    of SDF visuals (the per-visual cost took a 60 m world from ~1x to 0.2x real time)."""

    CHUNK = 60000  # vertices per group: Ogre asserts on a submesh past 16-bit indices

    def __init__(self):
        self.v, self.vt, self.groups, self.count = [], [], {}, {}

    def add(self, mtl, verts, tris, R=None, t=(0, 0, 0), uv=None):
        verts = np.asarray(verts, float) @ (R if R is not None else np.eye(3)).T + np.asarray(t, float)
        base = len(self.v)
        self.v += verts.tolist()
        self.vt += (np.asarray(uv).tolist() if uv is not None else [None] * len(verts))
        chunk, n = self.count.get(mtl, (0, 0))
        if n + len(verts) > self.CHUNK:
            chunk, n = chunk + 1, 0
        self.count[mtl] = (chunk, n + len(verts))
        self.groups.setdefault((mtl, chunk), []).extend((np.asarray(tris) + base).tolist())

    def write(self, path, colours):
        """colours: {material: (r, g, b)}; materials missing from it are left to the SDF <material>."""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        mtl = os.path.splitext(os.path.basename(path))[0] + '.mtl'
        with open(os.path.join(os.path.dirname(path), mtl), 'w') as fh:
            for m in self.count:
                c = colours.get(m, (1.0, 1.0, 1.0))
                fh.write(f'newmtl {m}\nKa {c[0]:.3f} {c[1]:.3f} {c[2]:.3f}\nKd {c[0]:.3f} {c[1]:.3f} {c[2]:.3f}\n'
                         f'Ks 0.02 0.02 0.02\nd 1.0\nillum 1\n')
        v = np.array(self.v)
        tris = np.array([t for ts in self.groups.values() for t in ts])
        nrm = np.zeros_like(v)  # per-vertex normals and UVs on every mesh: Ogre asserts on a mesh without them
        fn = np.cross(v[tris[:, 1]] - v[tris[:, 0]], v[tris[:, 2]] - v[tris[:, 0]])
        for k in range(3):
            np.add.at(nrm, tris[:, k], fn)
        nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-9)
        nrm[np.linalg.norm(nrm, axis=1) < 0.5] = (0.0, 0.0, 1.0)
        with open(path, 'w') as fh:
            fh.write(f'mtllib {mtl}\n')
            fh.writelines(f'v {x:.3f} {y:.3f} {z:.3f}\n' for x, y, z in self.v)
            fh.writelines(f'vt {u:.3f} {w:.3f}\n' if u is not None else 'vt 0 0\n'
                          for u, w in ((t if t is not None else (None, None)) for t in self.vt))
            fh.writelines(f'vn {x:.2f} {y:.2f} {z:.2f}\n' for x, y, z in nrm)
            for (m, chunk), ts in self.groups.items():
                fh.write(f'g {m}_{chunk}\nusemtl {m}\n')
                fh.writelines('f ' + ' '.join(f'{i + 1}/{i + 1}/{i + 1}' for i in t) + '\n' for t in ts)


_DAE = {}


def _dae_part(path, node_name):
    """(verts, uvs, tris) of one node of a COLLADA mesh, node transform applied."""
    if (path, node_name) in _DAE:
        return _DAE[(path, node_name)]
    import re
    d = open(path).read()
    geoms = dict(re.findall(r'<geometry id="([^"]+)"[^>]*>(.*?)</geometry>', d, re.S))
    mat, gid = re.search(rf'<node[^>]*name="{node_name}"[^>]*>\s*<matrix[^>]*>([^<]+)</matrix>\s*'
                         rf'<instance_geometry url="#([^"]+)"', d).groups()
    M = np.array(mat.split(), float).reshape(4, 4)
    g = geoms[gid]
    arrays = dict(re.findall(r'<source id="([^"]+)">\s*<float_array[^>]*>([^<]+)<', g))
    pos_src = re.search(r'<vertices[^>]*>.*?source="#([^"]+)"', g, re.S).group(1)
    pos = np.array(arrays[pos_src].split(), float).reshape(-1, 3) @ M[:3, :3].T + M[:3, 3]
    vs, uvs, tris = [], [], []
    for tri in re.findall(r'<triangles[^>]*>(.*?)</triangles>', g, re.S):
        stride = max(int(o) for o in re.findall(r'offset="(\d+)"', tri)) + 1
        voff = int(re.search(r'semantic="VERTEX"[^>]*offset="(\d+)"', tri).group(1))
        tm = re.search(r'semantic="TEXCOORD"[^>]*source="#([^"]+)"[^>]*offset="(\d+)"', tri)
        tc = np.array(arrays[tm.group(1)].split(), float).reshape(-1, 2)
        idx = np.array(re.search(r'<p>([^<]+)</p>', tri).group(1).split(), int).reshape(-1, stride)
        base = len(vs)
        vs += pos[idx[:, voff]].tolist()
        uvs += tc[idx[:, int(tm.group(2))]].tolist()
        tris += [(base + 3 * k, base + 3 * k + 1, base + 3 * k + 2) for k in range(len(idx) // 3)]
    _DAE[(path, node_name)] = (np.array(vs), np.array(uvs), np.array(tris))
    return _DAE[(path, node_name)]


def emit_hedge(name, bushes, rng, simple, models_dir, world):
    """Shrubs at (x, y, z_ground, r) merged into one mesh: a lumpy shaded core (the whole bush with simple foliage,
    solid for the lidar either way) under oak leaf cards, each with a sphere collision - a wall the rover sees."""
    core, leaves = _Obj(), _Obj()
    oak = TREE_SPECIES['oak']
    oak_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models', oak['model'])
    lv, luv, lt = (None, None, None) if simple else _dae_part(os.path.join(oak_dir, oak['mesh']), 'Branch')
    out = [f'    <model name="{name}">', '      <static>true</static>', '      <pose>0 0 0 0 0 0</pose>',
           '      <link name="link">']
    for i, (x, y, z, r) in enumerate(bushes):
        out.append(f'        <collision name="c{i}"><pose>{_f(x)} {_f(y)} {_f(z + r * 0.55)} 0 0 0</pose>'
                   f'<geometry><sphere><radius>{_f(r)}</radius></sphere></geometry></collision>')
        for k in range(int(rng.integers(3, 6))):
            a, d = rng.uniform(0, 2 * math.pi), rng.uniform(0.0, 0.55 if simple else 0.3) * r
            br = r * (rng.uniform(0.5, 0.85) if simple else rng.uniform(0.38, 0.52))
            sv, sf = _sphere(br)
            core.add(f'core{int(rng.integers(0, 3))}', sv, sf,
                     t=(x + d * math.cos(a), y + d * math.sin(a), z + br * rng.uniform(0.45, 0.9)))
        if not simple:
            sc = r * 0.3 * rng.uniform(0.9, 1.15)
            leaves.add('leaf', lv * sc, lt, R=_rpy(0, 0, rng.uniform(0, 2 * math.pi)), t=(x, y, z - 2.2 * sc), uv=luv)
    shades = ({'core0': (0.14, 0.30, 0.11), 'core1': (0.17, 0.35, 0.13), 'core2': (0.20, 0.38, 0.15)} if simple else
              {'core0': (0.15, 0.20, 0.07), 'core1': (0.17, 0.22, 0.08), 'core2': (0.19, 0.25, 0.09)})
    rel = f'meshes/{world}_{name}_core.obj'
    core.write(os.path.join(models_dir, ASSETS, rel), shades)
    out.append(f'        <visual name="core"><geometry><mesh><uri>model://{ASSETS}/{rel}</uri></mesh></geometry></visual>')
    if not simple:
        rel = f'meshes/{world}_{name}_leaves.obj'
        leaves.write(os.path.join(models_dir, ASSETS, rel), {})
        out.append(f'        <visual name="leaves"><geometry><mesh><uri>model://{ASSETS}/{rel}</uri></mesh></geometry>'
                   f'<material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>model://{oak["model"]}/{oak["branch_tex"]}'
                   f'</albedo_map><metalness>0</metalness><roughness>1</roughness></metal></pbr></material></visual>')
    return '\n'.join(out + ['      </link>', '    </model>'])


def emit_include(uri, name, x, y, z, yaw=0.0):
    return (f'    <include>\n'
            f'      <uri>{uri}</uri>\n'
            f'      <name>{name}</name>\n'
            f'      <pose>{_f(x)} {_f(y)} {_f(z)} 0 0 {_f(yaw)}</pose>\n'
            f'    </include>')


ASSETS = 'forest_pond'  # shared generated assets: pond textures, per-world pond / hedge meshes
POND_TEX = f'model://{ASSETS}/materials/textures'


def write_pond_textures(models_dir):
    """Tileable ripple normal map, water and mud albedo for every pond (procedural, fixed seed)."""
    d = os.path.join(models_dir, 'forest_pond', 'materials', 'textures')
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(models_dir, 'forest_pond', 'model.config'), 'w') as fh:
        fh.write('<?xml version="1.0"?>\n<model><name>forest_pond</name><version>1.0</version>'
                 '<sdf version="1.8">model.sdf</sdf><description>pond textures</description></model>\n')
    with open(os.path.join(models_dir, 'forest_pond', 'model.sdf'), 'w') as fh:
        fh.write('<?xml version="1.0"?>\n<sdf version="1.8"><model name="forest_pond"><static>true</static>'
                 '<link name="link"/></model></sdf>\n')
    n, rng = 512, np.random.default_rng(11)
    u = np.linspace(0, 2 * np.pi, n, endpoint=False)
    U, V = np.meshgrid(u, u)
    h = np.zeros((n, n))
    for _ in range(40):  # integer wave numbers keep it tileable
        kx, ky = rng.integers(-10, 11, 2)
        h += np.sin(kx * U + ky * V + rng.uniform(0, 2 * np.pi)) / (1.0 + math.hypot(kx, ky)) ** 0.6
    gy, gx = np.gradient(h)
    nrm = np.dstack([-gx * 2.0, -gy * 2.0, np.ones_like(h)])  # gentle ripples on still water
    nrm /= np.linalg.norm(nrm, axis=2, keepdims=True)
    Image.fromarray(((nrm * 0.5 + 0.5) * 255).astype(np.uint8)).save(os.path.join(d, 'water_normal.png'))
    t = (h - h.min()) / (np.ptp(h) + 1e-9)
    water = np.dstack([78 + 10 * t, 104 + 10 * t, 112 + 10 * t])  # sky-tinted grey-green, lighter on the crests
    Image.fromarray(water.astype(np.uint8)).save(os.path.join(d, 'water_albedo.png'))
    m = rng.normal(0, 1, (n // 8, n // 8))
    m = np.kron(m, np.ones((8, 8))) * 0.5 + rng.normal(0, 1, (n, n)) * 0.5
    mud = np.dstack([62 + 10 * m, 48 + 8 * m, 32 + 6 * m])
    Image.fromarray(np.clip(mud, 0, 255).astype(np.uint8)).save(os.path.join(d, 'mud_albedo.png'))


def _pbr(rgba, albedo=None, normal=None, rough=0.9, spec=0.05):
    r, g, b, a = rgba
    maps = (f'<albedo_map>{POND_TEX}/{albedo}</albedo_map>' if albedo else '') + \
           (f'<normal_map>{POND_TEX}/{normal}</normal_map>' if normal else '')
    return (f'<material><ambient>{_f(r)} {_f(g)} {_f(b)} {_f(a)}</ambient><diffuse>{_f(r)} {_f(g)} {_f(b)} {_f(a)}</diffuse>'
            f'<specular>{_f(spec)} {_f(spec)} {_f(spec)} 1</specular>'
            f'<pbr><metal>{maps}<roughness>{_f(rough)}</roughness><metalness>0.0</metalness></metal></pbr></material>')


def _shoreline(r0, rng):
    """Radius as a function of bearing: a round pond with a natural, irregular edge."""
    harm = [(k, rng.uniform(0.03, 0.10) / k ** 0.5, rng.uniform(0, 2 * math.pi)) for k in (2, 3, 5, 7)]
    return lambda a: r0 * (1.0 + sum(amp * math.sin(k * a + ph) for k, amp, ph in harm))


def write_pond_mesh(path, shore, grow=0.0, n_ang=120, rings=8, uv_tile=4.0):
    """Flat OBJ disc following `shore` (+ grow m), with planar UVs so the ripple texture tiles evenly."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    angs = [2 * math.pi * i / n_ang for i in range(n_ang)]
    v, vt, f = [(0.0, 0.0)], [], []
    for k in range(1, rings + 1):
        for a in angs:
            rr = (shore(a) + grow) * k / rings
            v.append((rr * math.cos(a), rr * math.sin(a)))
    idx = lambda k, i: 1 + (k - 1) * n_ang + (i % n_ang) + 1  # OBJ is 1-based; vertex 1 is the centre
    for i in range(n_ang):
        f.append((1, idx(1, i), idx(1, i + 1)))
    for k in range(1, rings):
        for i in range(n_ang):
            a, b, c, d = idx(k, i), idx(k, i + 1), idx(k + 1, i + 1), idx(k + 1, i)
            f += [(a, d, c), (a, c, b)]
    with open(path, 'w') as fh:
        for x, y in v:
            fh.write(f'v {x:.4f} {y:.4f} 0\n')
        for x, y in v:
            fh.write(f'vt {x / uv_tile:.4f} {y / uv_tile:.4f}\n')
        fh.write('vn 0 0 1\n')
        for t in f:
            fh.write('f ' + ' '.join(f'{j}/{j}/1' for j in t) + '\n')


def emit_pond(name, shore_name, x, y, z, radius, rng, models_dir, world):
    """A pond on level ground: rippled water and a mud margin following an irregular shoreline (visual only - no
    hole, no collision), lily pads, and a shore of grass tussocks and cattails with collision, so the rover's
    lidar sees it and drives round."""
    shore = _shoreline(radius, rng)
    mesh = {}
    for part_name, grow in (('water', 0.0), ('mud', 0.7)):
        rel = f'meshes/{world}_{name}_{part_name}.obj'
        write_pond_mesh(os.path.join(models_dir, ASSETS, rel), shore, grow)
        mesh[part_name] = f'<mesh><uri>model://{ASSETS}/{rel}</uri></mesh>'
    water = [f'    <model name="{name}">', '      <static>true</static>',
             f'      <pose>{_f(x)} {_f(y)} {_f(z)} 0 0 0</pose>', '      <link name="link">',
             f'        <visual name="mud"><pose>0 0 0.012 0 0 0</pose><geometry>{mesh["mud"]}</geometry>'
             f'{_pbr((0.30, 0.24, 0.16, 1.0), albedo="mud_albedo.png", rough=0.55, spec=0.2)}</visual>',
             f'        <visual name="water"><pose>0 0 0.03 0 0 0</pose><geometry>{mesh["water"]}</geometry>'
             f'{_pbr((0.32, 0.42, 0.44, 1.0), albedo="water_albedo.png", normal="water_normal.png", rough=0.03, spec=1.0)}'
             f'<cast_shadows>false</cast_shadows></visual>']
    water += ['      </link>', '    </model>']

    obj = _Obj()
    out = [f'    <model name="{shore_name}">', '      <static>true</static>',
           f'      <pose>{_f(x)} {_f(y)} {_f(z)} 0 0 0</pose>', '      <link name="link">']
    def blade(mtl, geom, bx, by, length, tilt, toward):
        """A stem / blade standing at (bx, by), leaning `tilt` rad toward bearing `toward`; returns its axis."""
        R = _rpy(0, tilt, toward)
        d = R[:, 2]
        obj.add(mtl, *geom, R=R, t=(bx + d[0] * length / 2, by + d[1] * length / 2, d[2] * length / 2))
        return d
    for i in range(int(rng.integers(10, 20))):  # lily pads, mostly near the edge
        a = rng.uniform(0, 2 * math.pi)
        rr = shore(a) * math.sqrt(rng.uniform(0.3, 0.85))
        obj.add(f'lily{int(rng.integers(0, 2))}', *_disc(rng.uniform(0.08, 0.24)), t=(rr * math.cos(a), rr * math.sin(a), 0.036))
    n_t = int(2 * math.pi * radius / 0.85)
    for i in range(n_t):  # grass tussocks: the solid base of the shore
        a = 2 * math.pi * (i + rng.uniform(-0.3, 0.3)) / n_t
        rr = shore(a) + rng.uniform(0.25, 0.85)
        cx, cy = rr * math.cos(a), rr * math.sin(a)
        out.append(f'        <collision name="ct{i}"><pose>{_f(cx)} {_f(cy)} 0.2 0 0 0</pose>'
                   f'<geometry><sphere><radius>0.32</radius></sphere></geometry></collision>')
        shade = f'blade{int(rng.integers(0, 3))}'
        for k in range(int(rng.integers(12, 20))):
            toward = rng.uniform(0, 2 * math.pi)
            off, bl = rng.uniform(0, 0.12), rng.uniform(0.35, 0.75)
            blade(shade, _box(0.03, 0.004, bl), cx + off * math.cos(toward), cy + off * math.sin(toward), bl,
                  rng.uniform(0.15, 0.65), toward)
    for i in range(int(2 * math.pi * radius / 0.16)):  # cattails with their leaves
        a = rng.uniform(0, 2 * math.pi)
        rr = shore(a) + rng.uniform(-0.2, 0.9)
        cx, cy = rr * math.cos(a), rr * math.sin(a)
        L, tilt, toward = rng.uniform(1.1, 1.8), rng.uniform(0.0, 0.14), rng.uniform(0, 2 * math.pi)
        d = blade('stem', _prism(0.011, L), cx, cy, L, tilt, toward)
        if rng.random() < 0.65:
            obj.add('head', *_prism(0.024, 0.2), R=_rpy(0, tilt, toward), t=(cx + d[0] * (L - 0.12), cy + d[1] * (L - 0.12),
                                                                              d[2] * (L - 0.12)))
        for k in range(int(rng.integers(1, 3))):
            ll = L * rng.uniform(0.7, 1.05)
            blade('leaf', _box(0.022, 0.003, ll), cx, cy, ll, rng.uniform(0.08, 0.3), rng.uniform(0, 2 * math.pi))
    rel = f'meshes/{world}_{shore_name}.obj'
    obj.write(os.path.join(models_dir, ASSETS, rel),
              {'blade0': (0.27, 0.36, 0.13), 'blade1': (0.31, 0.41, 0.15), 'blade2': (0.35, 0.44, 0.17),
               'stem': (0.42, 0.50, 0.24), 'head': (0.28, 0.17, 0.08), 'leaf': (0.33, 0.45, 0.19),
               'lily0': (0.16, 0.34, 0.10), 'lily1': (0.20, 0.40, 0.12)})
    out.append(f'        <visual name="shore"><geometry><mesh><uri>model://{ASSETS}/{rel}</uri></mesh></geometry></visual>')
    out += ['      </link>', '    </model>']
    return '\n'.join(water), '\n'.join(out)


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


GROUND_RAMPS = {
    'elevation': ([90, 140, 66], [122, 96, 60], [172, 158, 132]),  # valley green -> brown slope -> dry pale hilltop
    # green -> olive -> dry olive grass: the mushroom detector scored a cap 0.85 on green and under 0.5 on the brown
    'forest': ([82, 128, 60], [98, 116, 62], [124, 124, 82]),
}


def _write_terrain_diffuse(tdir, norm, out_size=768, ground='elevation'):
    """Bake an elevation colour ramp (low -> mid slope -> hilltop, see GROUND_RAMPS)
    into one image mapped 1:1 across the terrain. `norm` is the height field in
    [0, 1], row 0 = world +Y (matches heightmap.png)."""
    from PIL import ImageFilter
    low, mid, high = (np.array(c, float) for c in GROUND_RAMPS[ground])
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


def write_terrain_model(models_dir, terrain_name, field, terrain_size, z_span, ground='elevation'):
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
    _write_terrain_diffuse(tdir, norm, ground=ground)
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

    ponds = []   # (cx, cy, radius, unused depth draw - kept so the layout matches older worlds)
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
    for (px, py, pr, _) in ponds:  # level ground for the water to lie on - no basin
        field = flatten_disc(field, terrain_size, px, py, pr + 1.0, blend=2.5)

    # Grade the lanes so a trail follows the hills instead of cutting a trench.
    for lane in lanes:
        for (lx, ly) in lane:
            field = flatten_disc(field, terrain_size, lx, ly, 1.8, blend=3.5)

    # Vertical datum: lift so the lowest point sits at z=0.
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

    hedge_r = None
    if args.hedge:  # a closed square of bushes round the playable area: the rover never drives off the map
        hedge_r = playable_half + 0.3
        n = int(math.ceil(2 * hedge_r / 0.8))
        bushes, hrng = [], np.random.default_rng([args.seed, 1])  # own stream: the layout is unchanged
        for side in range(4):
            for i in range(n):
                t = -hedge_r + 2 * hedge_r * i / n
                x, y = [(t, -hedge_r), (hedge_r, t), (-t, hedge_r), (-hedge_r, -t)][side]
                bushes.append((x, y, ground(x, y), float(hrng.uniform(0.6, 0.8))))
        boundary_models.append(emit_hedge('hedge', bushes, hrng, simple, args.models_dir, args.name))

    # --- mission payload: mushrooms + decoys ------------------------
    mission_models = []
    mushrooms, decoys = [], []
    def _seg_dist(px, py, ax, ay, bx, by):
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / max(dx * dx + dy * dy, 1e-9)))
        return math.hypot(px - ax - t * dx, py - ay - t * dy)

    low_verts = {}

    def low_reach(species, sc, top=1.3):
        """How far a tree's foliage below `top` m spreads from its trunk: the rover's lidar and depth camera see
        those leaf cards as solid (pines are foliaged almost to the ground) though they have no collision."""
        spec = TREE_SPECIES.get(species)
        if not spec or not spec.get('model'):
            return 0.0
        if species not in low_verts:
            mdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models', spec['model'])
            low_verts[species] = _dae_part(os.path.join(mdir, spec['mesh']), 'Branch')[0]
        v = low_verts[species]
        low = v[v[:, 2] * sc < top]
        return float(np.hypot(low[:, 0], low[:, 1]).max() * sc) if len(low) else 0.0

    def unreachable(x, y):
        """Where no rover can put its gripper on a mushroom, or the quad's camera can't see it: against a trunk or
        inside a tree's low foliage, under a fallen tree's crown (they lie toward -yaw), on a log, rock, stump or clutter."""
        for o in objects:
            t = o['type']
            if t == 'tree':
                spec = TREE_SPECIES.get(o.get('species'), {})
                sc = o['radius'] / spec['base_trunk_r'] if spec.get('base_trunk_r') else 1.0
                if math.hypot(x - o['x'], y - o['y']) < max(o['radius'] + 1.5, low_reach(o.get('species'), sc) + 1.0):
                    return True
            if t == 'fallen_tree':
                ux, uy = -math.cos(o['yaw']), -math.sin(o['yaw'])
                L = o['length'] + 2.5
                if _seg_dist(x, y, o['x'], o['y'], o['x'] + ux * L, o['y'] + uy * L) < 3.0:
                    return True
            if t == 'log':
                ux, uy = math.cos(o['yaw']) * o['length'] / 2, math.sin(o['yaw']) * o['length'] / 2
                if _seg_dist(x, y, o['x'] - ux, o['y'] - uy, o['x'] + ux, o['y'] + uy) < o['radius'] + 1.0:
                    return True
            if t in ('rock', 'stump', 'clutter') and math.hypot(x - o['x'], y - o['y']) < o['radius'] + 1.0:
                return True
        return False

    # Where the rover can drive: everything it can't pass (trunks with their low foliage, fallen trees, logs,
    # rocks, the pond shore, the hedge) grown by its half-width plus margin, flooded out from the spawn clearing.
    # A mushroom needs a standoff point joined to that and a clear run-in, or it sits in a pocket the rover gets
    # wedged in (t24: two pines 6.5 m apart closed the corridor between them).
    from scipy import ndimage
    RES, CLEAR, STANDOFF = 0.25, 0.9, 1.7  # CLEAR: the rover's route clearance and costmap inflation
    n_cells = int(math.ceil(2 * playable_half / RES))
    cc = (np.arange(n_cells) + 0.5) * RES - playable_half
    GX, GY = np.meshgrid(cc, cc, indexing='ij')
    occ = np.zeros_like(GX, bool)

    def seg_mask(ax, ay, bx, by, rad):
        dx, dy = bx - ax, by - ay
        t = np.clip(((GX - ax) * dx + (GY - ay) * dy) / max(dx * dx + dy * dy, 1e-9), 0, 1)
        return np.hypot(GX - ax - t * dx, GY - ay - t * dy) < rad

    mesh_cache = {}

    def mark_tree(o, pitch):
        """The tree's own leaf and bark vertices at its pose, kept where they sit 0.15-1.3 m above the ground under
        them - what the rover's lidar and depth camera see. (A radius from the trunk base under-read it: on a slope
        the downhill branches hang far lower, and fallen crowns spread unevenly.)"""
        spec = TREE_SPECIES.get(o.get('species'), {})
        if not spec.get('model'):
            return
        if o['species'] not in mesh_cache:
            mdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models', spec['model'])
            mesh_cache[o['species']] = np.concatenate(
                [_dae_part(os.path.join(mdir, spec['mesh']), part)[0] for part in ('Branch', 'Bark')])
        sc = o['radius'] / spec['base_trunk_r']
        w = (mesh_cache[o['species']] * sc) @ _rpy(0, pitch, o['yaw']).T + np.array([o['x'], o['y'], o['z']])
        h = w[:, 2] - np.array([ground(px, py) for px, py in w[:, :2]])
        w = w[(h > 0.15) & (h < 1.3)]
        i = np.floor((w[:, 0] + playable_half) / RES).astype(int)
        j = np.floor((w[:, 1] + playable_half) / RES).astype(int)
        ok = (i >= 0) & (i < n_cells) & (j >= 0) & (j < n_cells)
        occ[i[ok], j[ok]] = True

    for o in objects:
        t = o['type']
        if t == 'tree':
            occ |= np.hypot(GX - o['x'], GY - o['y']) < o['radius']
            mark_tree(o, 0.0)
        elif t == 'fallen_tree':
            ux, uy = -math.cos(o['yaw']), -math.sin(o['yaw'])
            occ |= seg_mask(o['x'], o['y'], o['x'] + ux * o['length'], o['y'] + uy * o['length'], o['radius'] + 0.3)
            mark_tree(o, -o['tip'])
        elif t == 'log':
            ux, uy = math.cos(o['yaw']) * o['length'] / 2, math.sin(o['yaw']) * o['length'] / 2
            occ |= seg_mask(o['x'] - ux, o['y'] - uy, o['x'] + ux, o['y'] + uy, o['radius'])
        elif t in ('rock', 'stump', 'clutter'):
            occ |= np.hypot(GX - o['x'], GY - o['y']) < o['radius']
    for px, py, pr, _ in ponds:
        occ |= np.hypot(GX - px, GY - py) < pr * 1.25 + 1.0
    occ |= np.maximum(abs(GX), abs(GY)) > playable_half - 0.5  # the hedge
    free = ~ndimage.binary_dilation(occ, iterations=int(math.ceil(CLEAR / RES)))
    labels, _ = ndimage.label(free)
    spawn = labels[n_cells // 2, n_cells // 2]
    drivable = (labels == spawn) if spawn else free

    def cell(x, y):
        i, j = int((x + playable_half) / RES), int((y + playable_half) / RES)
        return (i, j) if 0 <= i < n_cells and 0 <= j < n_cells else None

    def rover_cannot_reach(x, y):
        for k in range(12):
            a = k * math.pi / 6
            sx, sy = x + STANDOFF * math.cos(a), y + STANDOFF * math.sin(a)
            c = cell(sx, sy)
            if c is None or not drivable[c]:
                continue
            if all((cc_ := cell(x + (sx - x) * f, y + (sy - y) * f)) is not None and not occ[cc_]
                   for f in np.linspace(0.15, 1.0, 8)):
                return False
        return True

    mush_pts = scatter_points(rng, playable_half - 3.0, int(args.mushrooms), 3.0,
                              blocked=lambda x, y: in_clearing(x, y, 1.0) or near_pond(x, y, 2.5) or unreachable(x, y)
                              or rover_cannot_reach(x, y), tries_factor=400)
    for i, (x, y) in enumerate(mush_pts):
        z = ground(x, y)
        name = f'mushroom_{i:02d}'
        mission_models.append(emit_include('model://mushroom_amanita', name, x, y, z))
        rec = dict(name=name, x=round(x, 3), y=round(y, 3), z=round(z, 3))
        mushrooms.append(rec)
        objects.append(dict(type='mushroom', **rec))

    decoy_pts = scatter_points(rng, playable_half - 3.0, int(args.decoys), 3.0,
                               blocked=lambda x, y: in_clearing(x, y, 1.0) or near_pond(x, y, 2.5))
    for i, (x, y) in enumerate(decoy_pts):
        model = 'decoy_box' if rng.random() < 0.5 else 'decoy_cylinder'
        z = ground(x, y) + 0.3     # decoy origins are centred, not base-aligned
        name = f'decoy_{i:02d}'
        mission_models.append(emit_include(f'model://{model}', name, x, y, z))
        rec = dict(name=name, model=model, x=round(x, 3), y=round(y, 3), z=round(z, 3))
        decoys.append(rec)
        objects.append(dict(type='decoy', **rec))

    # --- ponds: water on level ground, ringed by cattails and sedge the rover's sensors see -----------
    water_models = []
    pond_recs = []
    write_pond_textures(args.models_dir)
    for i, (px, py, pr, _) in enumerate(ponds):
        wz = ground(px, py)
        name = f'pond_{i:02d}'
        water_models += emit_pond(name, f'reeds_{i:02d}', px, py, wz, pr, np.random.default_rng([args.seed, 2 + i]),
                                  args.models_dir, args.name)
        rec = dict(name=name, x=round(px, 3), y=round(py, 3), z=round(wz, 3), radius=round(pr, 3),
                   shore_radius=round(pr * 1.25 + 1.0, 3))
        pond_recs.append(rec)
        objects.append(dict(type='pond', **rec))

    # --- write terrain + world + meta -------------------------------
    terrain_name = f'{args.name}_terrain'
    write_terrain_model(args.models_dir, terrain_name, field, terrain_size, z_span, args.ground)

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
                    boundary=args.boundary, hedge=bool(args.hedge), ground=args.ground, mushrooms=int(args.mushrooms),
                    decoys=int(args.decoys),
                    foliage='simple' if simple else 'mesh'),
        world=dict(name=args.name, playable_size_m=playable,
                   terrain_size_m=terrain_size),
        terrain=dict(model=terrain_name,
                     heightmap=f'models/{terrain_name}/heightmap.png',
                     samples=samples, requested_amplitude_m=round(amplitude, 3),
                     z_span_m=round(z_span, 3)),
        hedge=dict(half_size=round(hedge_r, 3)) if hedge_r else None,
        spawn_clearing=dict(center=[0.0, 0.0], radius=clearing_r,
                            ground_z=round(ground(0.0, 0.0), 3)),
        lanes=[[[round(x, 2), round(y, 2)] for (x, y) in ln] for ln in lanes],
        counts=dict(trees=len(tree_models),
                    fallen_trees=sum(o['type'] == 'fallen_tree' for o in objects),
                    logs=sum(o['type'] == 'log' for o in objects),
                    rocks_stumps_clutter=len(blob_models),
                    ponds=len(pond_recs), boundary=len(boundary_models),
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
    ap.add_argument('--ground', default='elevation', choices=sorted(GROUND_RAMPS),
                    help='ground colouring: elevation (green to brown to pale) or forest (green to olive, no bare soil)')
    ap.add_argument('--no-hedge', dest='hedge', action='store_false',
                    help='leave out the ring of bushes along the playable edge')
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
