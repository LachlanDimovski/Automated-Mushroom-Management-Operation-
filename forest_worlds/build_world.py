"""Base realistic forest world: rolling terrain cut by a meandering river (pebble bed, muddy banks, animated water),
Poly Haven trees/shrubs/rocks/grass placed to suit it. No logs, no mushrooms or hedge yet.

python3 build_world.py [--seed N] [--size 60] [--out worlds/river_forest.sdf]
"""
import argparse
import math
import os
import shutil

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, 'models')
ASSETS = os.path.join(HERE, 'world_assets')
TEX = os.path.join(HERE, 'textures')
BRINGUP_MODELS = os.path.join(HERE, '..', '41068_ignition_bringup', 'models')


def noise2(rng, n, octaves=5):
    """Smooth value noise on an n x n grid, roughly in [-1, 1]."""
    out = np.zeros((n, n))
    for o in range(octaves):
        k = 3 * 2 ** o
        g = rng.uniform(-1, 1, (k + 1, k + 1))
        xs = np.linspace(0, k, n)
        i = np.minimum(xs.astype(int), k - 1)
        f = xs - i
        f = f * f * (3 - 2 * f)
        a = g[i][:, i] * (1 - f)[None, :] + g[i][:, i + 1] * f[None, :]
        b = g[i + 1][:, i] * (1 - f)[None, :] + g[i + 1][:, i + 1] * f[None, :]
        out += (a * (1 - f)[:, None] + b * f[:, None]) * 0.5 ** o
    return out / 1.9


def dem_patch(rng, n, relief):
    """A window of Forest3D's real elevation survey (dem/terrain_enhanced.tif), resampled to n x n and scaled so
    its height range is `relief` metres, centred on 0."""
    from PIL import Image
    dem = np.array(Image.open(os.path.join(HERE, 'dem', 'terrain_enhanced.tif')), dtype=float)
    w = int(min(dem.shape) * 0.45)
    r0, c0 = rng.integers(0, dem.shape[0] - w), rng.integers(0, dem.shape[1] - w)
    win = dem[r0:r0 + w, c0:c0 + w]
    t = np.linspace(0, w - 1, n)
    i, f = np.minimum(t.astype(int), w - 2), t - np.minimum(t.astype(int), w - 2)
    rows = win[i] * (1 - f)[:, None] + win[i + 1] * f[:, None]
    z = rows[:, i] * (1 - f)[None, :] + rows[:, i + 1] * f[None, :]
    z = (z - z.min()) / max(np.ptp(z), 1e-6)
    return (z - 0.5) * relief


def river_centreline(rng, half):
    """Meandering polyline crossing the map west to east, dense samples (x, y)."""
    xs = np.linspace(-half - 4, half + 4, 900)
    ph = rng.uniform(0, 6.28, 3)
    ys = (5.5 * np.sin(xs / 9.0 + ph[0]) + 2.5 * np.sin(xs / 4.3 + ph[1]) + 1.0 * np.sin(xs / 2.1 + ph[2])
          + rng.uniform(-6, 6))
    return np.stack([xs, ys], 1)


def dist_to_line(px, py, line):
    """Distance of every point to the polyline, plus the index of the nearest sample."""
    best = np.full(px.shape, np.inf)
    idx = np.zeros(px.shape, int)
    for j in range(0, len(line), 1):
        d = (px - line[j, 0]) ** 2 + (py - line[j, 1]) ** 2
        m = d < best
        best[m] = d[m]
        idx[m] = j
    return np.sqrt(best), idx


def write_obj(path, V, UV, N, F):
    with open(path, 'w') as fh:
        fh.write('usemtl m\n')
        fh.writelines(f'v {a:.3f} {b:.3f} {c:.3f}\n' for a, b, c in V)
        fh.writelines(f'vt {a:.4f} {b:.4f}\n' for a, b in UV)
        fh.writelines(f'vn {a:.3f} {b:.3f} {c:.3f}\n' for a, b, c in N)
        fh.writelines(f'f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}\n' for a, b, c in F + 1)


def sub_mesh(V, UV, N, F):
    used = np.unique(F)
    remap = np.full(len(V), -1)
    remap[used] = np.arange(len(used))
    return V[used], UV[used], N[used], remap[F]


def pbr(tex, tile_note=''):
    return (f'<material><diffuse>1 1 1 1</diffuse><specular>0.05 0.05 0.05 1</specular><pbr><metal>'
            f'<albedo_map>model://river_terrain/materials/{tex}_diff.jpg</albedo_map>'
            f'<normal_map>model://river_terrain/materials/{tex}_nor_gl.jpg</normal_map>'
            f'<roughness_map>model://river_terrain/materials/{tex}_rough.jpg</roughness_map>'
            f'<metalness>0</metalness></metal></pbr></material>')


# Matched to the team presets in generate_forest_world.py (trees = tree_density x area), no logs in any of them.
PRESETS = {
    'sparse':   dict(relief=2.0, saplings=0.3, boulders=10, trees=43, ground_cover=450, bank_rocks=26, bank_plants=110, bed_stones=50, gap=3.6),
    'moderate': dict(relief=3.0, saplings=0.35, boulders=18, trees=65, ground_cover=650, bank_rocks=34, bank_plants=150, bed_stones=60, gap=3.0),
    'dense':    dict(relief=2.5, saplings=0.6, boulders=22, trees=137, ground_cover=1000, bank_rocks=40, bank_plants=200, bed_stones=60, gap=2.2),
    'hilly':    dict(relief=6.0, saplings=0.35, boulders=35, trees=58, ground_cover=600, bank_rocks=46, bank_plants=150, bed_stones=70, gap=3.0),
}


def build(args):
    for k, v in PRESETS[args.preset].items():
        if getattr(args, k, None) is None:
            setattr(args, k, v)
    rng = np.random.default_rng(args.seed)
    half = args.size / 2
    res = 0.3
    n = int(args.size / res) + 1
    xs = np.linspace(-half, half, n)
    X, Y = np.meshgrid(xs, xs)

    line = river_centreline(rng, half)
    y0 = float(np.interp(0.0, line[:, 0], line[:, 1]))
    if abs(y0) < 9.0:   # robots spawn at the origin: keep the river at least 9 m from it
        line[:, 1] += (9.0 - abs(y0)) * (1 if y0 >= 0 else -1)
    d, near = dist_to_line(X, Y, line)
    width = 2.6 + 0.7 * np.sin(line[:, 0] / 7.0 + 1.3)    # half-width of the water, varies along the river
    hw = width[near]

    # rolling hills that rise away from the river valley
    Z = dem_patch(rng, n, args.relief) + 0.12 * noise2(rng, n, 3)
    Z += 0.25 * args.relief * np.clip((d - hw - 3) / 18.0, 0, 1) ** 1.3
    # flat-ish floodplain, then a carved channel: bank slope down to a rounded bed
    valley = np.exp(-np.maximum(d - hw, 0) ** 2 / 30.0)
    Z = Z * (1 - 0.7 * valley)
    t = np.clip(d / (hw + 1.4), 0, 1)
    Z -= 0.75 * (1 - t ** 2.2) * (d < hw + 1.4)
    Z += 0.06 * noise2(rng, n, 2) * (d < hw)                  # uneven bed

    # water level along the river (smoothed bed + 0.45 m), then rocky fords: the bed rises just above the water and
    # the banks are graded down to it, giving the rover a drivable crossing (the open channel is a 0.75 m drop)
    def grid(x, y):
        return (np.clip(np.round((np.asarray(y) + half) / res).astype(int), 0, n - 1),
                np.clip(np.round((np.asarray(x) + half) / res).astype(int), 0, n - 1))
    lv_line = np.convolve(np.pad(Z[grid(line[:, 0], line[:, 1])], 25, mode='edge'), np.ones(51) / 51, 'valid') + 0.45
    arc_line = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(line, axis=0), axis=1))])
    inside = np.where(np.abs(line[:, 0]) < half - 8)[0]
    fords = [int(inside[int(len(inside) * f)]) for f in (0.3, 0.7)]
    # third ford at the river's southern-most stretch, well away from the other two
    far = [i for i in inside if all(abs(arc_line[i] - arc_line[f]) > 10 for f in fords)]
    fords.append(int(min(far, key=lambda i: line[i, 1])))
    along = np.min([np.abs(arc_line[near] - arc_line[f]) for f in fords], axis=0)
    fi = np.argmin([np.abs(arc_line[near] - arc_line[f]) for f in fords], axis=0)
    tf = lv_line[np.array(fords)][fi] + 0.04
    wa = 1 - np.clip((along - 1.8) / 1.6, 0, 1)                   # full across a 3.6 m lane, fades over 1.6 m
    wa = wa * wa * (3 - 2 * wa)
    rb = np.clip((d - hw - 0.3) / 3.2, 0, 1)
    rb = rb * rb * (3 - 2 * rb)
    graded = tf + (Z - tf) * rb + 0.03 * noise2(np.random.default_rng(args.seed + 2), n, 3)
    Z = np.where(d < hw + 3.5, Z * (1 - wa) + graded * wa, Z)
    # spawn clearing: ground at z = 0 within 4 m of the origin, blending back to the terrain over 3 m
    z0 = float(Z[grid(0.0, 0.0)])
    Z -= z0
    lv_line = lv_line - z0          # the water and fords move down with the terrain
    rc = np.clip((np.hypot(X, Y) - 4.0) / 3.0, 0, 1)
    Z = Z * rc * rc * (3 - 2 * rc)

    # mesh
    V = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
    UV = np.stack([X.ravel() / 3.0, -Y.ravel() / 3.0], 1)     # 3 m texture tile
    gy, gx = np.gradient(Z, res)
    N = np.stack([-gx.ravel(), -gy.ravel(), np.ones(n * n)], 1)
    N /= np.linalg.norm(N, axis=1, keepdims=True)
    q = np.arange(n * n).reshape(n, n)[:-1, :-1].ravel()
    F = np.concatenate([np.stack([q, q + 1, q + n + 1], 1), np.stack([q, q + n + 1, q + n], 1)])

    # surface class per triangle: riverbed, muddy bank, forest floor (leaf litter / mossy ground patches)
    cx = V[F, 0].mean(1)
    cy = V[F, 1].mean(1)
    fd = d.ravel()[F].mean(1)
    fhw = hw.ravel()[F].mean(1)
    edge_noise = 0.5 * np.sin(cx * 1.7) * np.cos(cy * 1.3)
    patch = noise2(np.random.default_rng(args.seed + 1), n).ravel()[F].mean(1)
    cls = np.where(fd < fhw + 0.2 + edge_noise * 0.4, 0, np.where(fd < fhw + 1.9 + edge_noise, 1,
                                                                    np.where(patch > 0.18, 3, 2)))

    tdir = os.path.join(ASSETS, 'river_terrain')
    os.makedirs(os.path.join(tdir, 'mesh'), exist_ok=True)
    os.makedirs(os.path.join(tdir, 'materials'), exist_ok=True)
    texs = ['river_small_rocks', 'brown_mud_leaves_01', 'forest_leaves_04', 'forest_ground_04']
    if args.flora == 'australian':   # dry eucalypt leaf litter instead of pine needles
        texs = ['river_small_rocks', 'brown_mud_leaves_01', 'dry_decay_leaves', 'leaves_forest_ground']
    for tx in texs:
        for k in ('diff', 'nor_gl', 'rough'):
            shutil.copy(os.path.join(TEX, f'{tx}_{k}.jpg'), os.path.join(tdir, 'materials'))
    vis = []
    if args.ground == 'green':
        shutil.copy(os.path.join(BRINGUP_MODELS, 'grass_plane', 'materials', 'textures', 'empty_grass.png'),
                    os.path.join(tdir, 'materials'))
    for c, tx in enumerate(texs):
        Fi = F[cls == c]
        if len(Fi):
            write_obj(os.path.join(tdir, 'mesh', f'{tx}.obj'), *sub_mesh(V, UV, N, Fi))
            mat = pbr(tx)
            if args.ground == 'green' and c > 0:   # the grass the team's YOLO was trained on (bringup grass_plane)
                mat = ('<material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse><pbr><metal>'
                       '<albedo_map>model://river_terrain/materials/empty_grass.png</albedo_map></metal></pbr></material>')
            vis.append(f'<visual name="{tx}"><geometry><mesh><uri>model://river_terrain/mesh/{tx}.obj</uri></mesh>'
                       f'</geometry>{mat}<cast_shadows>false</cast_shadows></visual>')
    # physics only needs a coarse ground: every 4th vertex (1.2 m grid, ~5k triangles instead of ~80k)
    st = 4
    cidx = np.arange(n)[::st]
    if cidx[-1] != n - 1:
        cidx = np.append(cidx, n - 1)
    cn = len(cidx)
    Vc = np.stack([X[np.ix_(cidx, cidx)].ravel(), Y[np.ix_(cidx, cidx)].ravel(), Z[np.ix_(cidx, cidx)].ravel()], 1)
    qc = np.arange(cn * cn).reshape(cn, cn)[:-1, :-1].ravel()
    Fc = np.concatenate([np.stack([qc, qc + 1, qc + cn + 1], 1), np.stack([qc, qc + cn + 1, qc + cn], 1)])
    write_obj(os.path.join(tdir, 'mesh', 'collision.obj'), Vc, np.zeros((cn * cn, 2)),
              np.tile([0.0, 0.0, 1.0], (cn * cn, 1)), Fc)
    vis.append(hedge_core(tdir, half - 0.9, lambda x, y: Z[np.clip(np.round((y + half) / res).astype(int), 0, n - 1),
                                                          np.clip(np.round((x + half) / res).astype(int), 0, n - 1)], rng))
    # the quad planner (quad_zcams obstacle_map) reads terrain height from a heightmap.png plus <heightmap> size/pos
    from PIL import Image
    zmin, zspan = float(Z.min()), float(np.ptp(Z))
    Image.fromarray(np.round((Z[::-1] - zmin) / zspan * 255).astype(np.uint8)).save(os.path.join(tdir, 'heightmap.png'))
    vis.append(f'<!-- planner heightmap: <heightmap><size>{args.size} {args.size} {zspan:.3f}</size>'
               f'<pos>0 0 {zmin:.3f}</pos></heightmap> -->')
    model_files(tdir, 'river_terrain', ''.join(vis))

    def height(x, y):
        i = np.clip(np.round((np.asarray(y) + half) / res).astype(int), 0, n - 1)
        j = np.clip(np.round((np.asarray(x) + half) / res).astype(int), 0, n - 1)
        return Z[i, j]

    # water ribbon: follows the centreline at a level just below the bank tops
    in_map = (np.abs(line[:, 0]) <= half - 0.9) & (np.abs(line[:, 1]) <= half - 0.9)   # stops at the hedge
    seg = line[in_map]
    tang = np.gradient(seg, axis=0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    nrm = np.stack([-tang[:, 1], tang[:, 0]], 1)
    w = 2.6 + 0.7 * np.sin(seg[:, 0] / 7.0 + 1.3) - 0.15
    arc = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))])
    lv = lv_line[in_map]
    level = float(lv.mean())
    m = 8
    Vw, UVw = [], []
    for k in range(len(seg)):
        for a in range(m + 1):
            s = a / m
            p = seg[k] + nrm[k] * w[k] * (2 * s - 1)
            Vw.append((p[0], p[1], lv[k]))
            UVw.append((arc[k] / 4.0, s))
    Vw, UVw = np.array(Vw), np.array(UVw)
    Nw = np.tile([0.0, 0.0, 1.0], (len(Vw), 1))
    Fw = []
    for k in range(len(seg) - 1):
        for a in range(m):
            i0 = k * (m + 1) + a
            Fw += [(i0, i0 + m + 2, i0 + 1), (i0, i0 + m + 1, i0 + m + 2)]   # counter-clockwise from above
    wdir = os.path.join(ASSETS, 'river_water')
    shutil.rmtree(wdir, ignore_errors=True)
    shutil.copytree(os.path.join(ASSETS, '_water_template'), wdir)   # water textures (make_textures.py)
    os.makedirs(os.path.join(wdir, 'mesh'), exist_ok=True)
    write_obj(os.path.join(wdir, 'mesh', 'water.obj'), Vw, UVw, Nw, np.array(Fw))
    model_files(wdir, 'river_water', water_visual())

    # vegetation
    inc = []
    cat = {c: sorted(os.listdir(os.path.join(MODELS, c))) for c in os.listdir(MODELS)}
    big_trees = [t for t in cat['tree'] if 'sapling' not in t]
    saplings = [t for t in cat['tree'] if 'sapling' in t]
    if args.flora == 'australian':   # procedural gums (make_gums.py); no conifers
        big_trees = saplings = cat['gum']

    seg_idx = np.where(in_map)[0]

    def near_ford(x, y, r=5.0):
        """Inside a ford's keep-clear zone: a disc of radius r, or the 4.4 m driving corridor that runs across the
        river and 7 m up both banks."""
        for f in fords:
            dx, dy = x - line[f, 0], y - line[f, 1]
            t = np.gradient(line, axis=0)[f]
            t = t / np.linalg.norm(t)
            if math.hypot(dx, dy) < r or (abs(dx * t[0] + dy * t[1]) < 2.2 + 1.0 and abs(-dx * t[1] + dy * t[0]) < 10):
                return True
        return False

    def river_d(x, y):
        dd, ii = dist_to_line(np.array([x]), np.array([y]), line)
        return dd[0] - (2.6 + 0.7 * math.sin(line[ii[0], 0] / 7.0 + 1.3))

    placed = []
    tree_set = set(cat['tree']) | set(cat.get('gum', []))
    colliders = ['<collision name="ground"><geometry><mesh><uri>model://river_terrain/mesh/collision.obj</uri>'
                 '</mesh></geometry></collision>']

    obstacles = []

    def put(model, x, y, z=None, yaw=None, sink=0.0, collide=True):
        if math.hypot(x, y) < 5.0:
            return
        if args.ground == 'green':   # reddish-brown rocks read as caps to the team YOLO: use the grey ones
            model = {'boulder_01': 'rock_moss_set_02', 'rock_07': 'stone_01'}.get(model, model)
        if collide and model_collisions(model):
            obstacles.append((x, y))
        z = float(height(x, y)) - sink if z is None else z
        yaw = rng.uniform(0, 6.283) if yaw is None else yaw
        for k, (pose, geom) in enumerate(model_collisions(model) if collide else []):   # gathered into one static model below
            px, py, pz = pose
            c, s_ = math.cos(yaw), math.sin(yaw)
            colliders.append(f'<collision name="c{len(colliders)}"><pose>{x + c * px - s_ * py:.3f} '
                             f'{y + s_ * px + c * py:.3f} {z + pz:.3f} 0 0 {yaw:.2f}</pose><geometry>{geom}'
                             f'</geometry></collision>')
        if model in tree_set:   # trees stay instanced: one shared GPU mesh each; baking copies millions of verts
            inc.append(f'<include><uri>model://{model}</uri><name>{model}_{len(inc)}</name>'
                       f'<pose>{x:.2f} {y:.2f} {z:.3f} 0 0 {yaw:.2f}</pose></include>')
        else:
            placed.append((model, x, y, z, yaw))

    trees = []
    for _ in range(args.trees * 40):
        if len(trees) >= args.trees:
            break
        x, y = rng.uniform(-half + 1, half - 1, 2)
        rd = river_d(x, y)
        if rd < 3.0 or near_ford(x, y, 6.0) or math.hypot(x, y - 0) < 0 or any((x - a) ** 2 + (y - b) ** 2 < args.gap ** 2 for a, b in trees):
            continue
        trees.append((x, y))
        put(rng.choice(saplings if rng.random() < args.saplings else big_trees), x, y, sink=0.05)
    # bank dressing: boulders half-buried at the waterline, ferns and shrubs on the banks, stones in the bed
    for f in fords:   # stepping-stone boulders along the ford's upstream and downstream lips, lane left clear
        k = int(np.searchsorted(seg_idx, f))
        t = np.gradient(line, axis=0)[f]
        t = t / np.linalg.norm(t)
        for lip in (-1, 1):
            # rock centres 2.9 m either side of the ford centreline, straight along its tangent: stepping along the
            # river line put the SW ford's rocks 1.4 m in on its bend, and the rover wedged between them (river11)
            for a in np.linspace(-1, 1, 5):
                p = line[f] + t * lip * 2.9 + np.array([-t[1], t[0]]) * a * w[min(k, len(w) - 1)]
                put('boulder_01', p[0], p[1], sink=rng.uniform(0.15, 0.3))
    for k in rng.choice(len(seg), args.bank_rocks, replace=False):
        side = rng.choice([-1, 1])
        p = seg[k] + nrm[k] * side * (w[k] - 0.1 + rng.uniform(-0.3, 0.6))
        if near_ford(p[0], p[1]):
            continue
        put(rng.choice(['boulder_01', 'rock_moss_set_01', 'rock_moss_set_02']), p[0], p[1], sink=0.35)
    for k in rng.choice(len(seg), args.bank_plants, replace=True):
        side = rng.choice([-1, 1])
        p = seg[k] + nrm[k] * side * (w[k] + rng.uniform(0.3, 2.5))
        if near_ford(p[0], p[1]):
            continue
        put(rng.choice([b for b in cat['bush'] if b.startswith(('fern', 'shrub_02', 'shrub_04'))]), p[0], p[1])
    for k in rng.choice(len(seg), args.bed_stones, replace=True):
        p = seg[k] + nrm[k] * rng.uniform(-1, 1) * w[k] * 0.8
        if near_ford(p[0], p[1]):
            continue
        put(rng.choice(['stone_01', 'rock_07']), p[0], p[1], z=float(height(p[0], p[1])) + 0.02)
    for _ in range(args.ground_cover):
        x, y = rng.uniform(-half + 0.5, half - 0.5, 2)
        if river_d(x, y) < 1.0 or near_ford(x, y, 3.5):
            continue
        r = rng.random()
        if r < 0.55:
            put(rng.choice(cat['grass']), x, y)
        elif r < 0.85:
            put(rng.choice(cat['bush']), x, y)
        else:
            put(rng.choice(['rock_07', 'stone_01', 'boulder_01']), x, y, sink=0.15)

    # boulder clusters through the forest: a big mossy rock with a couple of smaller ones round it
    for _ in range(args.boulders):
        x, y = rng.uniform(-half + 3, half - 3, 2)
        if river_d(x, y) < 2.0 or near_ford(x, y, 6.0) or math.hypot(x, y) < 4:
            continue
        put(rng.choice(['rock_moss_set_01', 'rock_moss_set_02', 'boulder_01']), x, y, sink=rng.uniform(0.15, 0.4))
        for _ in range(rng.integers(1, 4)):
            a, r = rng.uniform(0, 6.283), rng.uniform(1.0, 2.0)
            bx, by = x + r * math.cos(a), y + r * math.sin(a)
            if not near_ford(bx, by, 6.0):
                put(rng.choice(['boulder_01', 'rock_07', 'stone_01']), bx, by, sink=0.2)

    # perimeter hedge: two staggered rows of 1 m shrubs and ferns just inside the map edge, so lidar/depth see a wall
    # and the robots never drive off; physics gets four long boxes in the shared collider link
    hedge_r = half - 0.9
    hedge_boxes = []
    shrubs = [b for b in cat['bush'] if b.startswith('shrub_02')]
    for row, off in ((-1, 0.3), (1, 0.55)):
        step = 2.5
        for k in range(int(8 * hedge_r / step)):
            t_ = k * step + off * step
            side, u = divmod(t_, 2 * hedge_r)
            side = int(side) % 4
            a = -hedge_r + u
            r = hedge_r + row * 0.55
            x, y = [(a, -r), (r, a), (-a, r), (-r, -a)][side]
            put('fern_02' if rng.random() < 0.7 else rng.choice(shrubs), x, y, collide=False)
    for i, (x, y, sx, sy) in enumerate([(0, -hedge_r - 0.3, 2 * hedge_r + 1.2, 1.2), (0, hedge_r + 0.3, 2 * hedge_r + 1.2, 1.2),
                                        (-hedge_r - 0.3, 0, 1.2, 2 * hedge_r + 1.2), (hedge_r + 0.3, 0, 1.2, 2 * hedge_r + 1.2)]):
        hedge_boxes.append(f'<collision name="hedge{i}"><pose>{x:.2f} {y:.2f} {float(height(x, y)) + 0.6:.2f} 0 0 0</pose>'
                         f'<geometry><box><size>{sx:.2f} {sy:.2f} 3</size></box></geometry></collision>')

    inc.append(f'<model name="hedge"><static>true</static><link name="link">{"".join(hedge_boxes)}</link></model>')

    # mushrooms: on open ground, clear of water, fords, obstacles, the hedge and the spawn clearing; half each side
    mushrooms = []
    side_of = lambda x, y: np.sign(y - np.interp(x, line[:, 0], line[:, 1]))
    natives = []  # non-invasive decoys the detector must tell apart; never collected
    for _ in range(8000):
        if len(mushrooms) >= args.mushrooms and len(natives) >= args.natives:
            break
        x, y = rng.uniform(-hedge_r + 3.5, hedge_r - 3.5, 2)
        k = len(mushrooms) + len(natives)  # not n: height() closes over the grid size n
        want = 1 if k % 2 == 0 else -1
        if (side_of(x, y) != want or river_d(x, y) < 2.5 or near_ford(x, y, 4.0) or math.hypot(x, y) < 6
                or any(math.hypot(x - a, y - b) < 1.8 for a, b in obstacles + trees)
                or any(math.hypot(x - m['x'], y - m['y']) < 4 for m in mushrooms + natives)):
            continue
        native = len(mushrooms) >= args.mushrooms or (len(natives) < args.natives and k % 3 == 2)
        group = natives if native else mushrooms
        group.append({'name': f'{"native" if native else "mushroom"}_{len(group):02d}', 'x': round(float(x), 3), 'y': round(float(y), 3),
                          'z': round(float(height(x, y)), 3)})
    for m in natives:
        inc.append(f'<include><uri>model://mushroom_native</uri><name>{m["name"]}</name>'
                   f'<pose>{m["x"]} {m["y"]} {m["z"]} 0 0 {rng.uniform(0, 6.283):.2f}</pose></include>')
    for m in mushrooms:
        inc.append(f'<include><uri>model://mushroom_amanita</uri><name>{m["name"]}</name>'
                   f'<pose>{m["x"]} {m["y"]} {m["z"]} 0 0 {rng.uniform(0, 6.283):.2f}</pose></include>')

    bake(placed, half)
    inc.append('<include><uri>model://forest_baked</uri><name>forest_baked</name></include>')

    # one link for every static obstacle: DART does not collision-check shapes within a link against each other,
    # whereas hundreds of separate static models cost a full core even with nothing moving
    cdir = os.path.join(ASSETS, 'forest_colliders')
    os.makedirs(cdir, exist_ok=True)
    model_files(cdir, 'forest_colliders', ''.join(colliders))
    inc.append('<include><uri>model://forest_colliders</uri><name>forest_colliders</name></include>')

    suffix = ('_aus' if args.flora == 'australian' else '') + ('_green' if args.ground == 'green' else '')
    out = os.path.join(HERE, args.out or f'worlds/river_{args.preset}{suffix}.sdf')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    world = world_sdf(''.join(inc)).replace('"river_forest"', f'"{os.path.basename(out)[:-4]}"', 1)  # mission nodes address the world by file name
    # give this world's generated models their own names so presets don't overwrite each other
    tag = os.path.splitext(os.path.basename(out))[0]
    names = {b: f'{b}_{tag}' for b in ('river_water', 'forest_baked', 'forest_colliders')}
    names['river_terrain'] = f'{tag}_terrain'      # the quad planner finds the terrain by the _terrain suffix
    for base, new in names.items():
        dst = os.path.join(ASSETS, new)
        shutil.rmtree(dst, ignore_errors=True)
        os.rename(os.path.join(ASSETS, base), dst)
        world = world.replace(f'model://{base}<', f'model://{new}<').replace(f'<name>{base}</name>', f'<name>{new}</name>')
    for new in names.values():
        for f in ('model.sdf', 'model.config'):
            fp = os.path.join(ASSETS, new, f)
            txt = open(fp).read()
            for b2, n2 in names.items():
                txt = (txt.replace(f'model://{b2}/', f'model://{n2}/').replace(f'<name>{b2}</name>', f'<name>{n2}</name>')
                       .replace(f'model name="{b2}"', f'model name="{n2}"'))
            open(fp, 'w').write(txt)
    with open(out, 'w') as fh:
        fh.write(world)
    import json
    meta = {'world': {'name': tag, 'playable_size_m': round(2 * hedge_r - 2, 1), 'terrain_size_m': args.size},
            'hedge': {'half_size': hedge_r}, 'spawn_clearing': {'center': [0.0, 0.0], 'radius': 4.0, 'ground_z': 0.0},
            'mushrooms': mushrooms, 'natives': natives, 'fords': [[round(float(line[f, 0]), 2), round(float(line[f, 1]), 2)] for f in fords],
            # for robots' route maps: the channel to bank top (water half-width + 1.4 m) along the centreline, and each
            # ford's unit tangent along the river (its clear lane runs across, 3.2 m wide)
            'river': {'line': [[round(float(x), 2), round(float(y), 2), round(2.6 + 0.7 * math.sin(x / 7.0 + 1.3) + 1.4, 2)]
                               for x, y in line[::4]],
                      'ford_tangents': [[round(float(v), 3) for v in np.gradient(line, axis=0)[f]
                                         / np.linalg.norm(np.gradient(line, axis=0)[f])] for f in fords]},
            'objects': [{'type': 'tree', 'x': round(float(x), 3), 'y': round(float(y), 3), 'radius': 0.3} for x, y in trees]}
    with open(out[:-4] + '.meta.json', 'w') as fh:
        json.dump(meta, fh, indent=1)
    sl = np.degrees(np.arctan(np.hypot(*np.gradient(Z, res))))
    print(f'terrain relief {np.ptp(Z):.1f} m, slope p95 {np.percentile(sl, 95):.0f} deg, max {sl.max():.0f} deg')
    fp = ', '.join(f'({line[f, 0]:.1f}, {line[f, 1]:.1f})' for f in fords)
    print(f'wrote {out}: {len(trees)} trees, {len(placed)} baked items, fords at {fp}')


def hedge_core(tdir, r, height, rng, h=1.35, w=1.3):
    """Solid, lumpy evergreen band round the square perimeter (opaque needle texture), so the hedge reads as a wall
    to cameras, lidar and depth; loose shrubs and ferns are scattered over its faces separately."""
    L = 8 * r
    m = int(L / 0.25)
    ring = 14
    s_ = np.arange(m) * L / m
    side, u = np.divmod(s_, 2 * r)
    side = side.astype(int)
    a = -r + u
    px = np.select([side == 0, side == 1, side == 2, side == 3], [a, np.full(m, r), -a, np.full(m, -r)])
    py = np.select([side == 0, side == 1, side == 2, side == 3], [np.full(m, -r), a, np.full(m, r), -a])
    ox = np.select([side == 0, side == 1, side == 2, side == 3], [0 * a, 1 + 0 * a, 0 * a, -1 + 0 * a])
    oy = np.select([side == 0, side == 1, side == 2, side == 3], [-1 + 0 * a, 0 * a, 1 + 0 * a, 0 * a])
    lump = 1 + 0.18 * np.sin(s_ * 2.1) * np.sin(s_ * 0.73 + 1) + 0.08 * rng.standard_normal(m)
    V, UV, N = [], [], []
    for i in range(m):
        gz = float(height(px[i], py[i]))
        for j in range(ring + 1):
            th = math.pi * j / ring                     # half-ellipse arch from outer foot to inner foot
            cw, sh = math.cos(th), math.sin(th)
            V.append((px[i] + ox[i] * cw * w / 2 * lump[i], py[i] + oy[i] * cw * w / 2 * lump[i],
                      gz - 0.15 + sh * h * lump[i]))
            N.append((ox[i] * cw, oy[i] * cw, sh))
            UV.append((s_[i] / 1.6, j / ring * 1.9))
    F = []
    for i in range(m):
        i2 = (i + 1) % m
        for j in range(ring):
            a0, a1, b0, b1 = i * (ring + 1) + j, i * (ring + 1) + j + 1, i2 * (ring + 1) + j, i2 * (ring + 1) + j + 1
            F += [(a0, b0, b1), (a0, b1, a1)]
    write_obj(os.path.join(tdir, 'mesh', 'hedge.obj'), np.array(V), np.array(UV), np.array(N), np.array(F))
    for k in ('diff', 'nor_gl'):
        shutil.copy(os.path.join(TEX, f'hedge_leaves_{k}.jpg'), os.path.join(tdir, 'materials', f'hedge_{k}.jpg'))
    return ('<visual name="hedge"><geometry><mesh><uri>model://river_terrain/mesh/hedge.obj</uri></mesh></geometry>'
            '<material><diffuse>1 1 1 1</diffuse><pbr><metal><metalness>0</metalness><roughness>0.95</roughness>'
            '<albedo_map>model://river_terrain/materials/hedge_diff.jpg</albedo_map>'
            '<normal_map>model://river_terrain/materials/hedge_nor_gl.jpg</normal_map></metal></pbr></material>'
            '<cast_shadows>true</cast_shadows></visual>')


def _load_obj(path):
    V, T, N, F = [], [], [], []
    for l in open(path):
        t = l.split()
        if not t:
            continue
        if t[0] == 'v': V.append(t[1:4])
        elif t[0] == 'vt': T.append(t[1:3])
        elif t[0] == 'vn': N.append(t[1:4])
        elif t[0] == 'f': F.append([c.split('/') for c in t[1:4]])
    V, T, N = (np.array(a, float) for a in (V, T, N))
    F = np.array(F, int) - 1                        # (nf, 3 corners, v/vt/vn)
    # de-index to one vertex per (v, vt, vn) corner combination
    key, inv = np.unique(F.reshape(-1, 3), axis=0, return_inverse=True)
    return V[key[:, 0]], T[key[:, 1]], N[key[:, 2]], inv.reshape(-1, 3)


def bake(placed, half, tile=20.0, max_verts=60000):
    """Merge every placed library model into a few static meshes: one per material per 20 m tile (each <= 60k verts).
    Hundreds of separate models cost a full CPU core in Fortress's entity systems even when nothing moves."""
    import re
    bdir = os.path.join(ASSETS, 'forest_baked')
    shutil.rmtree(bdir, ignore_errors=True)
    os.makedirs(os.path.join(bdir, 'mesh'))
    os.makedirs(os.path.join(bdir, 'materials'))
    lib = {}
    groups = {}
    paths = {}
    for model, x, y, z, yaw in placed:
        if model not in lib:
            mdir = next(os.path.join(MODELS, c, model) for c in os.listdir(MODELS)
                        if os.path.isdir(os.path.join(MODELS, c, model)))
            parts = []
            for v in re.findall(r'<visual .*?</visual>', open(os.path.join(mdir, 'model.sdf')).read()):
                mesh = re.search(r'<uri>model://[^/]+/([^<]+)</uri>', v).group(1)
                alb = re.search(r'<albedo_map>model://[^/]+/([^<]+)</albedo_map>', v)
                nor = re.search(r'<normal_map>model://[^/]+/([^<]+)</normal_map>', v)
                key = tuple(os.path.join(mdir, m.group(1)) if m else None for m in (alb, nor))
                key = (key, tuple(os.path.basename(k) if k else None for k in key))
                shadows = '<cast_shadows>true' in v
                parts.append((key, shadows, _load_obj(os.path.join(mdir, mesh))))
            lib[model] = parts
        c, s_ = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
        cell = (int((x + half) // tile), int((y + half) // tile))
        for key, shadows, (V, T, N, F) in lib[model]:
            g = groups.setdefault((key[1], shadows, cell), [[]])
            paths.setdefault(key[1], key[0])
            if g[-1] and sum(len(p[0]) for p in g[-1]) + len(V) > max_verts:
                g.append([])
            g[-1].append((V @ R.T + (x, y, z), T, N @ R.T, F))
    vis, texmap = [], {}
    for gi, ((key, shadows, cell), chunks) in enumerate(groups.items()):
        mat = '<diffuse>1 1 1 1</diffuse><pbr><metal><metalness>0</metalness><roughness>0.9</roughness>'
        for tex, tag in zip(paths[key], ('albedo_map', 'normal_map')):
            if tex:
                if tex not in texmap:
                    texmap[tex] = f'{len(texmap)}_{os.path.basename(tex)}'
                    shutil.copy(tex, os.path.join(bdir, 'materials', texmap[tex]))
                mat += f'<{tag}>model://forest_baked/materials/{texmap[tex]}</{tag}>'
        mat += '</metal></pbr>'
        for ci, parts in enumerate(chunks):
            off, Fs = 0, []
            for V, T, N, F in parts:
                Fs.append(F + off)
                off += len(V)
            name = f'g{gi}_{ci}'
            write_obj(os.path.join(bdir, 'mesh', name + '.obj'), np.concatenate([p[0] for p in parts]),
                      np.concatenate([p[1] for p in parts]), np.concatenate([p[2] for p in parts]),
                      np.concatenate(Fs))
            vis.append(f'<visual name="{name}"><geometry><mesh><uri>model://forest_baked/mesh/{name}.obj</uri>'
                       f'</mesh></geometry><material>{mat}</material>'
                       f'<cast_shadows>{"true" if shadows else "false"}</cast_shadows></visual>')
    model_files(bdir, 'forest_baked', ''.join(vis))
    print(f'baked {len(placed)} models into {len(vis)} meshes')


_coll_cache = {}


def model_collisions(model):
    """Collision shapes of a library model as [(xyz, geometry_xml)]; strips them from its model.sdf (visual-only)."""
    import re
    if model not in _coll_cache:
        mdir = next(os.path.join(MODELS, c, model) for c in os.listdir(MODELS)
                    if os.path.isdir(os.path.join(MODELS, c, model)))
        store = os.path.join(mdir, 'collision.xml')
        sdf_p = os.path.join(mdir, 'model.sdf')
        sdf = open(sdf_p).read()
        if not os.path.exists(store):
            open(store, 'w').write(''.join(re.findall(r'<collision .*?</collision>', sdf)))
        open(sdf_p, 'w').write(re.sub(r'<collision .*?</collision>', '', sdf))
        out = []
        for c in re.findall(r'<collision .*?</collision>', open(store).read()):
            pose = re.search(r'<pose>([^<]*)</pose>', c)
            xyz = tuple(map(float, pose.group(1).split()[:3])) if pose else (0.0, 0.0, 0.0)
            out.append((xyz, re.search(r'<geometry>(.*?)</geometry>', c).group(1)))
        _coll_cache[model] = out
    return _coll_cache[model]


def water_visual():
    """Still-looking but reflective water: standard PBR with a ripple normal map. (The animated ShaderParam version
    flickered on and off in the Fortress GUI.)"""
    return ('<visual name="water"><geometry><mesh><uri>model://river_water/mesh/water.obj</uri></mesh></geometry>'
            '<cast_shadows>false</cast_shadows><material><diffuse>1 1 1 1</diffuse>'
            '<specular>0.6 0.6 0.6 1</specular><pbr><metal>'
            '<albedo_map>model://river_water/materials/textures/water_albedo.png</albedo_map>'
            '<normal_map>model://river_water/materials/textures/river_normal.png</normal_map>'
            '<roughness>0.3</roughness><metalness>0</metalness></metal></pbr></material></visual>')


def model_files(mdir, name, link_xml):
    with open(os.path.join(mdir, 'model.sdf'), 'w') as fh:
        fh.write(f'<?xml version="1.0"?>\n<sdf version="1.8"><model name="{name}"><static>true</static>'
                 f'<link name="link">{link_xml}</link></model></sdf>\n')
    with open(os.path.join(mdir, 'model.config'), 'w') as fh:
        fh.write(f'<?xml version="1.0"?>\n<model><name>{name}</name><version>1.0</version>'
                 f'<sdf version="1.8">model.sdf</sdf></model>\n')


def world_sdf(includes):
    return f'''<?xml version="1.0"?>
<sdf version="1.8"><world name="river_forest">
<physics name="default" type="ignored"><max_step_size>0.01</max_step_size><real_time_factor>1</real_time_factor></physics>
<plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
<plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
<plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
<plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<scene><ambient>0.5 0.52 0.5 1</ambient><background>0.62 0.74 0.86 1</background><shadows>false</shadows></scene>
<light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 40 0 0 0</pose>
<diffuse>1 0.94 0.82 1</diffuse><specular>0.3 0.3 0.3 1</specular><direction>-0.45 0.35 -0.82</direction></light>
<include><uri>model://river_terrain</uri><name>river_terrain</name></include>
<include><uri>model://river_water</uri><name>pond_river_water</name></include>
{includes}
</world></sdf>
'''


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=11)
    ap.add_argument('--size', type=float, default=60.0)
    ap.add_argument('--trees', type=int, default=None)
    ap.add_argument('--bank_rocks', type=int, default=None)
    ap.add_argument('--bank_plants', type=int, default=None)
    ap.add_argument('--bed_stones', type=int, default=None)
    ap.add_argument('--ground_cover', type=int, default=None)
    ap.add_argument('--boulders', type=int, default=None)
    ap.add_argument('--saplings', type=float, default=None, help='fraction of trees that are (lighter) saplings')
    ap.add_argument('--preset', default='moderate', choices=sorted(PRESETS))
    ap.add_argument('--relief', type=float, default=None, help='terrain height range (m), from the Forest3D DEM')
    ap.add_argument('--gap', type=float, default=None)
    ap.add_argument('--mushrooms', type=int, default=12)
    ap.add_argument('--natives', type=int, default=6, help='brown non-invasive decoy mushrooms')
    ap.add_argument('--ground', default='natural', choices=['natural', 'green'],
                    help='green = the grass_plane texture the team YOLO was trained on')
    ap.add_argument('--flora', default='conifer', choices=['conifer', 'australian'])
    ap.add_argument('--out', default=None, help='default worlds/river_<preset>.sdf (+ _aus for australian)')
    build(ap.parse_args())
