"""Everything the quad could hit at cruise height, read from the world file, as a grid it can plan on.

Every visual (primitives, and the tree bark meshes whose oak branches spread 2-4 m out at
cruise height) is sampled, kept where it lies within BAND above the local terrain and
rasterised. A* plans over that grid inflated by the quad's clearance; lidar returns seen
in flight are layered on top for anything the world file does not hold.
"""
import heapq
import math
import os
import re

import numpy as np
from PIL import Image
from scipy import ndimage

BAND = (0.8, 3.0)   # m above the local ground: cruise 2.0 m and hold 1.2 m, plus the quad's size
STEP = 0.15         # sampling step on primitives (m)
MESH_STEP = 0.07    # sampling step on meshes before their per-tree scale (up to ~2.1)
RES = 0.25          # grid cell (m)
PREFER_M = 1.8      # paths pay extra within this of an obstacle, so they keep off the clearance edge
NEAR_COST = 0.5     # ...up to (1 + NEAR_COST) x the length there; more pushed the camera off ground by trees
SKIP_PREFIXES = ('mushroom', 'decoy', 'demo_mushroom', 'cage_test')


def _rot(roll, pitch, yaw):
    """SDF pose rotation: Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _pose(text):
    m = re.match(r'\s*<pose>([^<]+)</pose>', text)
    v = [float(x) for x in m.group(1).split()] if m else [0.0] * 6
    return np.array(v[:3]), _rot(*v[3:6])


def _disk(r, step):
    """Points filling a disk of radius r in the xy plane."""
    pts = [(0.0, 0.0)]
    for rr in np.arange(step, r + 1e-9, step):
        k = max(6, int(2 * math.pi * rr / step))
        a = np.linspace(0, 2 * math.pi, k, endpoint=False)
        pts += list(zip(rr * np.cos(a), rr * np.sin(a)))
    if r > 0 and (r % step) > 1e-6:
        k = max(6, int(2 * math.pi * r / step))
        a = np.linspace(0, 2 * math.pi, k, endpoint=False)
        pts += list(zip(r * np.cos(a), r * np.sin(a)))
    return np.array(pts)


def _solid(kind, geom):
    """Points filling a primitive in its own frame (z is the cylinder / capsule axis)."""
    num = lambda tag: float(re.search(rf'<{tag}>([^<]+)</{tag}>', geom).group(1))
    out = []
    if kind == 'cylinder':
        r, L = num('radius'), num('length')
        d = _disk(r, STEP)
        for z in np.arange(-L / 2, L / 2 + 1e-9, STEP):
            out.append(np.column_stack([d, np.full(len(d), z)]))
    elif kind in ('sphere', 'capsule'):
        r = num('radius')
        half = num('length') / 2 if kind == 'capsule' else 0.0
        for z in np.arange(-half - r, half + r + 1e-9, STEP):
            over = max(0.0, abs(z) - half)
            rz = math.sqrt(max(0.0, r * r - over * over))
            d = _disk(rz, STEP)
            out.append(np.column_stack([d, np.full(len(d), z)]))
    elif kind == 'box':
        sx, sy, sz = (float(v) for v in re.search(r'<size>([^<]+)</size>', geom).group(1).split())
        g = np.mgrid[-sx / 2:sx / 2 + 1e-9:STEP, -sy / 2:sy / 2 + 1e-9:STEP, -sz / 2:sz / 2 + 1e-9:STEP]
        out.append(g.reshape(3, -1).T)
    return np.concatenate(out) if out else np.zeros((0, 3))


_MESHES = {}


def _mesh_points(path, node_name):
    """Surface samples of one named node of a COLLADA mesh (node transform applied), cached."""
    key = (path, node_name)
    if key in _MESHES:
        return _MESHES[key]
    s = open(path).read()
    geoms = dict(re.findall(r'<geometry id="([^"]+)"[^>]*>(.*?)</geometry>', s, re.S))
    pts = []
    for name, mat, gid in re.findall(r'<node[^>]*name="([^"]+)"[^>]*>\s*<matrix[^>]*>([^<]+)</matrix>\s*'
                                     r'<instance_geometry url="#([^"]+)"', s):
        if node_name and name != node_name:
            continue
        M = np.array(mat.split(), float).reshape(4, 4)
        g = geoms[gid]
        arr = (re.search(r'<float_array[^>]*id="[^"]*position[^"]*"[^>]*>([^<]+)<', g, re.I)
               or re.search(r'<float_array[^>]*>([^<]+)<', g)).group(1)
        pos = np.array(arr.split(), float).reshape(-1, 3) @ M[:3, :3].T + M[:3, 3]
        for tri in re.findall(r'<triangles[^>]*>(.*?)</triangles>', g, re.S):
            stride = max(int(o) for o in re.findall(r'offset="(\d+)"', tri)) + 1
            voff = int(re.search(r'semantic="VERTEX"[^>]*offset="(\d+)"', tri).group(1))
            idx = np.array(re.search(r'<p>([^<]+)</p>', tri).group(1).split(), int)[voff::stride]
            for a, b, c in pos[idx.reshape(-1, 3)]:
                k = max(1, int(math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b))
                                         / MESH_STEP)))
                for i in range(k + 1):
                    for j in range(k + 1 - i):
                        pts.append(a + (b - a) * (i / k) + (c - a) * (j / k))
    _MESHES[key] = np.array(pts) if pts else np.zeros((0, 3))
    return _MESHES[key]


def _resolve(uri, dirs):
    if not uri.startswith('model://'):
        return uri if os.path.exists(uri) else None
    rel = uri[len('model://'):]
    for d in dirs:
        p = os.path.join(d, rel)
        if os.path.exists(p):
            return p
    return None


def _model_dirs(sdf_path, extra=()):
    dirs = [os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(sdf_path)), '..', 'models'))]
    for var in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH'):
        dirs += [p for p in os.environ.get(var, '').split(':') if p]
    return dirs + list(extra)


class Terrain:
    """Heightmap terrain of a generated world (<name>_terrain), or flat z = 0."""

    def __init__(self, sdf_text, dirs):
        self.img = None
        m = re.search(r'<uri>model://(\w+_terrain)</uri>', sdf_text)
        path = _resolve(f'model://{m.group(1)}/model.sdf', dirs) if m else None
        if not path:
            return
        t = open(path).read()
        self.sx, self.sy, sz = (float(v) for v in re.search(r'<heightmap>.*?<size>([^<]+)</size>', t, re.S).group(1).split())
        self.px, self.py, pz = (float(v) for v in re.search(r'<heightmap>.*?<pos>([^<]+)</pos>', t, re.S).group(1).split())
        png = os.path.join(os.path.dirname(path), 'heightmap.png')
        self.img = np.asarray(Image.open(png).convert('L'), float) / 255.0 * sz + pz

    def z(self, x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        if self.img is None:
            return np.zeros_like(x)
        n, m = self.img.shape
        fx = np.clip((x - self.px + self.sx / 2) / self.sx * (m - 1), 0, m - 1)
        fy = np.clip((self.sy / 2 - (y - self.py)) / self.sy * (n - 1), 0, n - 1)  # image row 0 is +y
        x0, y0 = np.floor(fx).astype(int), np.floor(fy).astype(int)
        x1, y1 = np.minimum(x0 + 1, m - 1), np.minimum(y0 + 1, n - 1)
        tx, ty = fx - x0, fy - y0
        top = self.img[y0, x0] * (1 - tx) + self.img[y0, x1] * tx
        bot = self.img[y1, x0] * (1 - tx) + self.img[y1, x1] * tx
        return top * (1 - ty) + bot * ty


def boundary_ring(sdf_path):
    """Radius of the innermost boundary ('edge_*') tree, or None if the world has no tree ring. The search
    stays inside it (a wall there at the planner's clearance), so no sweep, route or hold leaves the forest."""
    s = open(sdf_path).read()
    r = [math.hypot(float(a), float(b)) for a, b in
         re.findall(r'<model name="edge_\d+">\s*<static>true</static>\s*<pose>([-\d.]+) ([-\d.]+)', s)]
    return min(r) if r else None


def world_obstacle_points(sdf_path, band=BAND, extra_dirs=(), skip_submeshes=()):
    """(N, 2) xy of every visual surface / volume sample lying within `band` above the local ground
    (leaving out mesh visuals whose submesh is in `skip_submeshes`, e.g. 'Branch' for the leaf cards)."""
    s = open(sdf_path).read()
    dirs = _model_dirs(sdf_path, extra_dirs)
    terrain = Terrain(s, dirs)
    out = []
    for name, body in re.findall(r'<model name="([^"]+)">(.*?)</model>', s, re.S):
        if name.startswith(SKIP_PREFIXES) or name.endswith('_terrain'):
            continue
        mpos = re.search(r'<pose>([^<]+)</pose>', body)
        mv = [float(x) for x in mpos.group(1).split()] if mpos else [0.0] * 6
        mt, mR = np.array(mv[:3]), _rot(*mv[3:6])
        for vis in re.findall(r'<visual name="[^"]*">(.*?)</visual>', body, re.S):
            vt, vR = _pose(vis)
            geom = re.search(r'<geometry>(.*?)</geometry>', vis, re.S).group(1)
            kind = re.match(r'\s*<(\w+)', geom).group(1)
            if kind == 'mesh':
                path = _resolve(re.search(r'<uri>([^<]+)</uri>', geom).group(1), dirs)
                if not path:
                    continue
                sub = re.search(r'<submesh>\s*<name>([^<]+)</name>', geom)
                if sub and sub.group(1) in skip_submeshes:
                    continue
                sc = re.search(r'<scale>([^<]+)</scale>', geom)
                scale = np.array(sc.group(1).split(), float) if sc else np.ones(3)
                local = _mesh_points(path, sub.group(1) if sub else None) * scale
            else:
                local = _solid(kind, geom)
            if not len(local):
                continue
            w = (local @ vR.T + vt) @ mR.T + mt
            h = w[:, 2] - terrain.z(w[:, 0], w[:, 1])
            out.append(w[(h >= band[0]) & (h <= band[1])][:, :2])
    return np.concatenate(out) if out else np.zeros((0, 2))


class ObstacleGrid:
    """Occupancy from the world file plus recent lidar returns; A* over it inflated by `clearance`."""

    def __init__(self, pts, half, clearance, lidar_ttl=30.0, ring=None):
        self.half, self.clearance, self.lidar_ttl = half, clearance, lidar_ttl
        self.n = int(math.ceil(2 * half / RES))
        self.static = np.zeros((self.n, self.n), bool)
        i, j = self._cells(pts[:, 0], pts[:, 1]) if len(pts) else (np.array([], int), np.array([], int))
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        self.static[i[ok], j[ok]] = True
        self.outside = np.zeros((self.n, self.n), bool)
        if ring is not None:  # the boundary tree ring is the edge of the search: the quad never goes past it (a boundary,
            c = (np.arange(self.n) + 0.5) * RES - half  # not an obstacle, so no clearance off it - that sealed corridors)
            X, Y = np.meshgrid(c, c, indexing='ij')
            self.outside = np.hypot(X, Y) >= ring
        self.lidar_seen = np.full((self.n, self.n), -1e9)
        self.now = 0.0
        self.rebuild(0.0)

    def _cells(self, x, y):
        return (np.floor((np.asarray(x) + self.half) / RES).astype(int),
                np.floor((np.asarray(y) + self.half) / RES).astype(int))

    def cell(self, x, y):
        return int(math.floor((x + self.half) / RES)), int(math.floor((y + self.half) / RES))

    def xy(self, i, j):
        return (i + 0.5) * RES - self.half, (j + 0.5) * RES - self.half

    def inside(self, i, j):
        return 0 <= i < self.n and 0 <= j < self.n

    def rebuild(self, now):
        self.now = now
        self.occ = self.static | (self.lidar_seen > now - self.lidar_ttl)
        self.dist = ndimage.distance_transform_edt(~self.occ) * RES  # m to the nearest occupied cell centre
        self.blocked = (self.dist < self.clearance + RES / 2) | self.outside

    def add_lidar(self, xs, ys, now):
        """Mark lidar returns; True if any of them landed on a cell that was free."""
        i, j = self._cells(xs, ys)
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        i, j = i[ok], j[ok]
        fresh = bool(np.any(~self.occ[i, j]))
        self.lidar_seen[i, j] = now
        expired = np.any((self.lidar_seen > now - self.lidar_ttl - 5.0) & (self.lidar_seen <= now - self.lidar_ttl))
        if fresh or expired:
            self.rebuild(now)
        return fresh

    def is_blocked(self, x, y):
        i, j = self.cell(x, y)
        return not self.inside(i, j) or bool(self.blocked[i, j])

    def clearance_at(self, x, y):
        i, j = self.cell(x, y)
        return float(self.dist[i, j]) if self.inside(i, j) else 0.0

    def segment_clear(self, a, b, grid=None):
        grid = self.blocked if grid is None else grid
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        for t in np.linspace(0.0, 1.0, max(2, int(L / (RES / 2)) + 1)):
            i, j = self.cell(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            if not self.inside(i, j) or grid[i, j]:
                return False
        return True

    def nearest_free(self, x, y, max_r):
        """Nearest unblocked cell centre within max_r of (x, y), or None."""
        i0, j0 = self.cell(x, y)
        k = int(math.ceil(max_r / RES))
        best = None
        for i in range(max(0, i0 - k), min(self.n, i0 + k + 1)):
            for j in range(max(0, j0 - k), min(self.n, j0 + k + 1)):
                if not self.blocked[i, j]:
                    d = (i - i0) ** 2 + (j - j0) ** 2
                    if d * RES * RES <= max_r * max_r and (best is None or d < best[0]):
                        best = (d, i, j)
        return self.xy(best[1], best[2]) if best else None

    def plan(self, start, goal, snap=4.0):
        """Smoothed A* waypoints from start to goal (goal snapped to free space), or None."""
        s = start if not self.is_blocked(*start) else self.nearest_free(*start, 3.0)
        g = goal if not self.is_blocked(*goal) else self.nearest_free(*goal, snap)
        if s is None or g is None:
            return None
        si, sj = self.cell(*s)
        gi, gj = self.cell(*g)
        cost_mult = 1.0 + NEAR_COST * np.clip((PREFER_M - self.dist) / max(PREFER_M - self.clearance, 1e-6), 0.0, 1.0)
        best = {(si, sj): 0.0}
        prev = {}
        h = lambda i, j: RES * (max(abs(i - gi), abs(j - gj)) + (math.sqrt(2) - 1) * min(abs(i - gi), abs(j - gj)))
        heap = [(h(si, sj), 0.0, si, sj)]
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]
        found = False
        while heap:
            _, c, i, j = heapq.heappop(heap)
            if (i, j) == (gi, gj):
                found = True
                break
            if c > best.get((i, j), 1e18):
                continue
            for di, dj, L in moves:
                ni, nj = i + di, j + dj
                if not (0 <= ni < self.n and 0 <= nj < self.n) or self.blocked[ni, nj]:
                    continue
                nc = c + L * RES * cost_mult[ni, nj]
                if nc < best.get((ni, nj), 1e18):
                    best[(ni, nj)] = nc
                    prev[(ni, nj)] = (i, j)
                    heapq.heappush(heap, (nc + h(ni, nj), nc, ni, nj))
        if not found:
            return None
        cells = [(gi, gj)]
        while cells[-1] != (si, sj):
            cells.append(prev[cells[-1]])
        cells.reverse()
        pts = [self.xy(i, j) for i, j in cells]
        pts[0], pts[-1] = s, g
        d = np.array([self.dist[i, j] for i, j in cells])
        # shortcut corners, but only where the straight line keeps as far from obstacles as the A* path did
        out, k = [pts[0]], 0
        while k < len(pts) - 1:
            nxt = k + 1
            for m in range(min(len(pts) - 1, k + 60), k + 1, -1):
                if self._min_dist(pts[k], pts[m]) >= min(d[k:m + 1].min(), PREFER_M) - 0.05:
                    nxt = m
                    break
            out.append(pts[nxt])
            k = nxt
        return out if s != start else out[1:] or [g]

    def _min_dist(self, a, b):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        t = np.linspace(0.0, 1.0, max(2, int(L / (RES / 2)) + 1))
        i, j = self._cells(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        if np.any((i < 0) | (i >= self.n) | (j < 0) | (j >= self.n)):
            return 0.0
        return float(self.dist[i, j].min())
